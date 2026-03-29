"""
Flexible Crankshaft Module
==========================
使用几何精确梁单元 (ObjectBeamGeometricallyExact) 创建柔性曲柄轴。
支持变截面、偏心段建模，与 EXUDYN 的接触系统配合使用。

作者: Assistant
日期: 2024
"""

import sys
sys.exudynFast = True
sys.exudynCPUhasAVX2 = True

import numpy as np
from typing import List, Tuple, Dict, Any, Optional

import exudyn as exu
from exudyn.itemInterface import (
    ObjectBeamGeometricallyExact, NodeRigidBodyEP,
    VObjectBeamGeometricallyExact, MarkerNodeRigid,
    MarkerBodyRigid, GenericJoint, VObjectJointGeneric,
    RigidBodySpringDamper, VRigidBodySpringDamper
)


# =================== 梁截面工具函数 ===================

def create_circular_beam_section(
    radius: float,
    E: float = 2.1e11,
    nu: float = 0.3,
    rho: float = 7850.0
) -> 'exu.BeamSection':
    """
    创建圆形截面的梁单元属性。
    
    Parameters
    ----------
    radius : 截面半径 (m)
    E : 杨氏模量 (Pa)，默认钢材
    nu : 泊松比
    rho : 密度 (kg/m³)
    
    Returns
    -------
    exu.BeamSection 对象
    """
    G = E / (2 * (1 + nu))
    A = np.pi * radius**2
    I = np.pi * radius**4 / 4
    J = 2 * I  # 极惯性矩（圆截面）
    k = 6 * (1 + nu) / (7 + 6 * nu)  # Timoshenko 剪切修正系数
    
    section = exu.BeamSection()
    section.stiffnessMatrix = np.diag([
        E * A,      # 轴向刚度
        k * G * A,  # 剪切刚度 y
        k * G * A,  # 剪切刚度 z
        G * J,      # 扭转刚度
        E * I,      # 弯曲刚度 y
        E * I       # 弯曲刚度 z
    ])
    section.massPerLength = rho * A
    section.inertia = rho * np.diag([J, I, I])
    return section


def create_circular_section_geometry(radius: float, n_points: int = 16) -> 'exu.BeamSectionGeometry':
    """创建圆形截面的可视化几何（使用多边形近似）"""
    geo = exu.BeamSectionGeometry()
    
    # 使用多边形点定义圆形截面
    lp = exu.Vector2DList()
    for i in range(n_points):
        angle = 2 * np.pi * i / n_points
        y = radius * np.cos(angle)
        z = radius * np.sin(angle)
        lp.Append([y, z])
    geo.polygonalPoints = lp
    
    return geo


# =================== 曲柄轴段定义 ===================

class CrankSegment:
    """曲柄轴单个段的定义"""
    
    def __init__(
        self,
        length: float,
        radius: float,
        eccentricity_y: float = 0.0,
        eccentricity_x: float = 0.0,
        color: List[float] = None,
        name: str = ""
    ):
        """
        Parameters
        ----------
        length : 段长度 (m)
        radius : 截面半径 (m)
        eccentricity_y : y方向偏心量 (m)
        eccentricity_x : x方向偏心量 (m)
        color : RGBA颜色
        name : 段名称（用于标识）
        """
        self.length = length
        self.radius = radius
        self.eccentricity_y = eccentricity_y
        self.eccentricity_x = eccentricity_x
        self.color = color if color is not None else [0.5, 0.5, 0.5, 1.0]
        self.name = name


# =================== 柔性曲柄轴类 ===================

class FlexibleCrankshaft:
    """
    柔性曲柄轴类，使用几何精确梁单元建模。
    
    支持：
    - 多段变截面
    - 偏心段
    - 自动创建接触标记
    """
    
    def __init__(
        self,
        segments: List[CrankSegment],
        E: float = 2.1e11,
        nu: float = 0.3,
        rho: float = 7850.0,
        n_elements_per_segment: int = 2
    ):
        """
        Parameters
        ----------
        segments : 曲柄轴各段定义列表
        E : 杨氏模量 (Pa)
        nu : 泊松比
        rho : 密度 (kg/m³)
        n_elements_per_segment : 每段的梁单元数量
        """
        self.segments = segments
        self.E = E
        self.nu = nu
        self.rho = rho
        self.n_elements_per_segment = n_elements_per_segment
        
        # 计算总长度和各段起始z位置
        self.total_length = sum(seg.length for seg in segments)
        self._compute_z_positions()
        
        # 将在 build() 中填充
        self.nodes = []
        self.beam_objects = []
        self.node_z_positions = []
        self.is_built = False
    
    def _compute_z_positions(self):
        """计算各段的起始和结束z位置"""
        self.segment_z_start = []
        self.segment_z_end = []
        z = 0.0
        for seg in self.segments:
            self.segment_z_start.append(z)
            z += seg.length
            self.segment_z_end.append(z)
    
    @property
    def z_start(self) -> float:
        """曲柄轴起始z坐标（第一个节点）"""
        return 0.0
    
    @property
    def z_end(self) -> float:
        """曲柄轴结束z坐标（最后一个节点）"""
        return self.total_length
    
    def build(
        self,
        mbs,
        reference_position: List[float],
        z_offset: float = 0.0,
        initial_rotation: List[float] = None,
        use_spring_connection: bool = True,
        connection_stiffness_factor: float = 1.0
    ) -> Dict[str, Any]:
        """
        在 EXUDYN 系统中创建柔性曲柄轴（分段独立建模）。
        
        曲柄轴沿Z轴放置，偏心在X-Y平面。
        每段独立建模，节点在该段自己的轴线上（包含偏心），
        段之间用6自由度弹簧连接。
        
        Parameters
        ----------
        mbs : MainSystem
        reference_position : 曲柄轴轴线的参考位置 [x, y, 0]
        z_offset : z方向偏移（第一个节点的z坐标）
        initial_rotation : 初始欧拉参数 [e0, e1, e2, e3]，默认无旋转
        use_spring_connection : 是否使用弹簧连接（True）或刚性连接（False）
        connection_stiffness_factor : 连接刚度系数（1.0=标准刚度）
        
        Returns
        -------
        包含节点、梁单元、特殊位置节点等信息的字典
        """
        if initial_rotation is None:
            # 梁单元轴向现在沿Z轴（C++已修改），无需旋转
            initial_rotation = [1., 0., 0., 0.]
        
        x_ref, y_ref = reference_position[0], reference_position[1]
        G = self.E / (2 * (1 + self.nu))
        
        self.nodes = []
        self.beam_objects = []
        self.node_z_positions = []
        self.segment_nodes = []
        self.segment_beams = []
        self.connection_springs = []
        
        segment_info = {}
        
        z_current = z_offset
        prev_seg_end_node = None
        prev_seg_end_pos = None
        prev_seg = None
        
        for seg_idx, seg in enumerate(self.segments):
            section = create_circular_beam_section(seg.radius, self.E, self.nu, self.rho)
            section_geo = create_circular_section_geometry(seg.radius)
            
            n_elements = self.n_elements_per_segment
            elem_length = seg.length / n_elements
            
            # 偏心在X-Y平面
            x_seg = x_ref + seg.eccentricity_x
            y_seg = y_ref + seg.eccentricity_y
            
            seg_nodes = []
            seg_beams = []
            
            # 创建该段的第一个节点
            n0 = mbs.AddNode(NodeRigidBodyEP(
                referenceCoordinates=[x_seg, y_seg, z_current] + list(initial_rotation)
            ))
            seg_nodes.append(n0)
            self.nodes.append(n0)
            self.node_z_positions.append(z_current)
            
            # 如果有上一段，添加段间连接
            if prev_seg_end_node is not None:
                self._add_segment_connection(
                    mbs, prev_seg_end_node, n0,
                    prev_seg, seg,
                    use_spring_connection,
                    connection_stiffness_factor,
                    G,
                    prev_seg_end_pos,
                    [x_seg, y_seg, z_current]
                )
            
            # 创建该段的梁单元
            for i_elem in range(n_elements):
                z_next = z_current + elem_length
                
                n1 = mbs.AddNode(NodeRigidBodyEP(
                    referenceCoordinates=[x_seg, y_seg, z_next] + list(initial_rotation)
                ))
                seg_nodes.append(n1)
                self.nodes.append(n1)
                self.node_z_positions.append(z_next)
                
                # 添加梁单元
                oBeam = mbs.AddObject(ObjectBeamGeometricallyExact(
                    nodeNumbers=[n0, n1],
                    physicsLength=elem_length,
                    sectionData=section,
                    visualization=VObjectBeamGeometricallyExact(
                        sectionGeometry=section_geo,
                        color=seg.color
                    )
                ))
                seg_beams.append(oBeam)
                self.beam_objects.append(oBeam)
                
                n0 = n1
                z_current = z_next
            
            self.segment_nodes.append(seg_nodes)
            self.segment_beams.append(seg_beams)
            
            # 记录该段信息
            center_idx = len(seg_nodes) // 2
            segment_info[seg_idx] = {
                'nodes': seg_nodes,
                'beams': seg_beams,
                'center_node': seg_nodes[center_idx],
                'first_node': seg_nodes[0],
                'last_node': seg_nodes[-1],
                'ecc_x': seg.eccentricity_x,
                'ecc_y': seg.eccentricity_y,
                'radius': seg.radius,
            }
            if seg.name:
                segment_info[seg.name] = segment_info[seg_idx]
            
            prev_seg_end_node = seg_nodes[-1]
            prev_seg_end_pos = [x_seg, y_seg, z_current]
            prev_seg = seg
        
        self.is_built = True
        
        return {
            'nodes': self.nodes,
            'beam_objects': self.beam_objects,
            'node_z_positions': self.node_z_positions,
            'segment_nodes': self.segment_nodes,
            'segment_beams': self.segment_beams,
            'segment_info': segment_info,
            'connection_springs': self.connection_springs,
            'first_node': self.nodes[0],
            'last_node': self.nodes[-1],
        }
    
    def _add_segment_connection(
        self, mbs, node1, node2, seg1, seg2,
        use_spring: bool, stiffness_factor: float, G: float,
        pos1: List[float], pos2: List[float]
    ):
        """添加段间连接（弹簧或刚性）"""
        m1 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node1))
        m2 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node2))
        
        if use_spring:
            # 计算过渡区等效刚度
            r_trans = min(seg1.radius, seg2.radius)
            L_equiv = r_trans  # 等效长度
            
            A = np.pi * r_trans**2
            I = np.pi * r_trans**4 / 4
            J = 2 * I
            
            # 6自由度刚度矩阵
            K_axial = self.E * A / L_equiv * stiffness_factor
            K_shear = G * A / L_equiv * stiffness_factor
            K_torsion = G * J / L_equiv * stiffness_factor
            K_bend = self.E * I / L_equiv * stiffness_factor
            
            stiffness = np.diag([K_axial, K_shear, K_shear, K_torsion, K_bend, K_bend])
            
            # 阻尼（取刚度的0.01倍作为阻尼系数）
            damping = 0.01 * stiffness
            
            # 计算初始位移作为 offset，使弹簧初始无应力
            # pos1, pos2 是两个节点的初始位置 [x, y, z]
            initial_displacement = [
                pos2[0] - pos1[0],  # dx
                pos2[1] - pos1[1],  # dy  
                pos2[2] - pos1[2],  # dz
                0., 0., 0.          # 旋转偏移（假设初始无相对旋转）
            ]
            
            spring = mbs.AddObject(RigidBodySpringDamper(
                markerNumbers=[m1, m2],
                stiffness=stiffness,
                damping=damping,
                offset=initial_displacement,  # 初始位移作为offset
                visualization=VRigidBodySpringDamper(show=False)
            ))
            self.connection_springs.append(spring)
        else:
            # 刚性连接
            joint = mbs.AddObject(GenericJoint(
                markerNumbers=[m1, m2],
                constrainedAxes=[1, 1, 1, 1, 1, 1],
                visualization=VObjectJointGeneric(show=False)
            ))
            self.connection_springs.append(joint)
    
    def get_node_at_z(self, z_target: float) -> int:
        """获取最接近指定z位置的节点"""
        if not self.is_built:
            raise RuntimeError("Must call build() first")
        idx = np.argmin(np.abs(np.array(self.node_z_positions) - z_target))
        return self.nodes[idx]
    
    def get_node_index_at_z(self, z_target: float) -> int:
        """获取最接近指定z位置的节点索引"""
        if not self.is_built:
            raise RuntimeError("Must call build() first")
        return np.argmin(np.abs(np.array(self.node_z_positions) - z_target))
    
    def get_segment_center_node(self, segment_idx_or_name) -> int:
        """获取指定段的中心节点"""
        if not self.is_built:
            raise RuntimeError("Must call build() first")
        # 这需要在 build 返回的字典中查找
        raise NotImplementedError("Use the segment_center_nodes from build() result")
    
    def create_marker_at_z(self, mbs, z_target: float) -> int:
        """在指定z位置创建 MarkerNodeRigid"""
        node = self.get_node_at_z(z_target)
        return mbs.AddMarker(MarkerNodeRigid(nodeNumber=node))
    
    def create_marker_at_node(self, mbs, node_index: int) -> int:
        """在指定节点创建 MarkerNodeRigid"""
        if not self.is_built:
            raise RuntimeError("Must call build() first")
        return mbs.AddMarker(MarkerNodeRigid(nodeNumber=self.nodes[node_index]))


# =================== 三曲柄轴系统工厂函数 ===================

def create_three_crankshafts(
    mbs,
    oGround,
    segments: List[CrankSegment],
    crank_distribution_radius: float,
    angle_offset: float = 0.0,
    z_offset: float = 0.0,
    E: float = 2.1e11,
    nu: float = 0.3,
    rho: float = 7850.0,
    n_elements_per_segment: int = 2
) -> Dict[str, Any]:
    """
    创建三个均布的柔性曲柄轴。
    
    Parameters
    ----------
    mbs : MainSystem
    oGround : 地面对象
    segments : 曲柄轴各段定义
    crank_distribution_radius : 曲柄轴分布圆半径
    angle_offset : 角度偏移（从哪个角度开始）
    z_offset : z方向偏移
    E, nu, rho : 材料参数
    n_elements_per_segment : 每段梁单元数量
    
    Returns
    -------
    包含所有曲柄轴信息的字典
    """
    n_cranks = 3
    crankshafts = []
    all_nodes = []
    all_objects = []
    build_results = []
    
    for i_crank in range(n_cranks):
        angle_crank = angle_offset + 2 * np.pi * i_crank / n_cranks
        x_crank = crank_distribution_radius * np.cos(angle_crank)
        y_crank = crank_distribution_radius * np.sin(angle_crank)
        
        # 创建曲柄轴实例
        crank = FlexibleCrankshaft(
            segments=segments,
            E=E, nu=nu, rho=rho,
            n_elements_per_segment=n_elements_per_segment
        )
        
        # 构建
        result = crank.build(
            mbs,
            reference_position=[x_crank, y_crank, 0],
            z_offset=z_offset
        )
        
        crankshafts.append(crank)
        all_nodes.append(result['nodes'])
        all_objects.append(result['beam_objects'])
        build_results.append(result)
    
    return {
        'crankshafts': crankshafts,
        'nodes': all_nodes,
        'objects': all_objects,
        'build_results': build_results,
        'n_cranks': n_cranks
    }


def add_crankshaft_constraints(
    mbs,
    oGround,
    crankshaft: FlexibleCrankshaft,
    reference_position: List[float],
    z_base: float,
    constrained_axes: List[int] = None
) -> int:
    """
    为曲柄轴添加边界约束。
    
    Parameters
    ----------
    mbs : MainSystem
    oGround : 地面对象
    crankshaft : 柔性曲柄轴对象
    reference_position : [x, y] 参考位置
    z_base : 约束位置的z坐标
    constrained_axes : 约束轴 [x,y,z,rx,ry,rz]，默认 [0,0,1,1,1,0]
    
    Returns
    -------
    GenericJoint 对象索引
    """
    if constrained_axes is None:
        constrained_axes = [0, 0, 1, 1, 1, 0]  # 约束 Z 平移和 XY 倾转
    
    x_ref, y_ref = reference_position[0], reference_position[1]
    
    # 地面标记
    mGroundCrank = mbs.AddMarker(MarkerBodyRigid(
        bodyNumber=oGround,
        localPosition=[x_ref, y_ref, z_base]
    ))
    
    # 曲柄轴基础节点标记
    node_base = crankshaft.get_node_at_z(z_base)
    mCrankBase = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node_base))
    
    # 添加通用关节
    joint = mbs.AddObject(GenericJoint(
        markerNumbers=[mGroundCrank, mCrankBase],
        constrainedAxes=constrained_axes,
        visualization=VObjectJointGeneric(axesRadius=0.5e-3, axesLength=1.0e-3)
    ))
    
    return joint


# =================== 标准曲柄轴配置 ===================

def create_standard_crank_segments(
    L_main1: float,
    L_eccentric1: float,
    L_middle: float,
    L_eccentric2: float,
    L_main2: float,
    r_main: float,
    r_eccentric: float,
    eccentric_offset: float
) -> List[CrankSegment]:
    """
    创建标准的五段曲柄轴配置。
    
    Parameters
    ----------
    L_main1 : 主轴段1长度
    L_eccentric1 : 偏心段1长度
    L_middle : 中间段长度
    L_eccentric2 : 偏心段2长度
    L_main2 : 主轴段2长度
    r_main : 主轴半径
    r_eccentric : 偏心段半径
    eccentric_offset : 偏心距
    
    Returns
    -------
    CrankSegment 列表
    """
    return [
        CrankSegment(L_main1, r_main, 0, 0, 
                     [0.4, 0.4, 0.4, 1.0], "main1"),
        CrankSegment(L_eccentric1, r_eccentric, eccentric_offset, 0, 
                     [1.0, 0.2, 0.2, 1.0], "ecc1"),
        CrankSegment(L_middle, r_main, 0, 0, 
                     [0.4, 0.4, 0.4, 1.0], "middle"),
        CrankSegment(L_eccentric2, r_eccentric, -eccentric_offset, 0, 
                     [1.0, 0.5, 0.0, 1.0], "ecc2"),
        CrankSegment(L_main2, r_main, 0, 0, 
                     [0.4, 0.4, 0.4, 1.0], "main2"),
    ]
