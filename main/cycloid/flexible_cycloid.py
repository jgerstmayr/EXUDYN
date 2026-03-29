"""
Flexible Cycloid Wheel Module
============================
使用 NGsolve/Netgen 生成摆线轮有限元网格，计算模态，创建柔性体。
支持与 EXUDYN 的 ObjectContactCurveCircles 配合使用。

作者: Assistant
日期: 2024
"""

import numpy as np
from typing import Tuple, List, Optional, Dict, Any

# NGsolve/Netgen imports
try:
    from netgen.occ import *
    from netgen.occ import OCCGeometry
    from ngsolve import Mesh, H1, BilinearForm, LinearForm, GridFunction
    from ngsolve import grad, dx, ds, Integrate
    NGSOLVE_AVAILABLE = True
except ImportError:
    NGSOLVE_AVAILABLE = False
    print("Warning: NGsolve/Netgen not available. Flexible cycloid will not work.")

# EXUDYN imports
import exudyn as exu
from exudyn.FEM import FEMinterface, ObjectFFRFreducedOrderInterface
from exudyn.itemInterface import MarkerSuperElementRigid, MarkerSuperElementPosition


def create_cycloid_geometry_occ(
    profile_x: np.ndarray,
    profile_y: np.ndarray,
    thickness: float,
    hole_centers: List[Tuple[float, float]],
    hole_radius: float,
    center_hole_radius: Optional[float] = None
) -> 'Solid':
    """
    使用 OpenCASCADE 创建摆线轮3D几何体。
    
    Parameters
    ----------
    profile_x, profile_y : 摆线轮齿廓的x, y坐标数组
    thickness : 摆线轮厚度 (m)
    hole_centers : 孔中心坐标列表 [(x1,y1), (x2,y2), ...]
    hole_radius : 孔半径 (m)
    center_hole_radius : 中心孔半径 (m)，如果为None则没有中心孔
    
    Returns
    -------
    OCC Solid 几何体
    """
    if not NGSOLVE_AVAILABLE:
        raise ImportError("NGsolve/Netgen is required for flexible cycloid")
    
    # 将齿廓点转换为 OCC 样条曲线
    n_points = len(profile_x)
    
    # 创建齿廓点列表
    profile_points = [Pnt(profile_x[i], profile_y[i], 0) for i in range(n_points)]
    
    # 创建闭合样条曲线
    spline = SplineApproximation(profile_points, deg=3, tol=1e-6)
    wire_outer = Wire([spline])
    
    # 创建外轮廓面
    face_outer = Face(wire_outer)
    
    # 拉伸成3D实体
    cycloid_solid = face_outer.Extrude(thickness * Z)
    
    # 3D 布尔运算减去孔
    for (cx, cy) in hole_centers:
        hole_cyl = Cylinder(Pnt(cx, cy, -thickness), Z, hole_radius, 3*thickness)
        cycloid_solid = cycloid_solid - hole_cyl
    
    # 减去中心孔（如果有）
    if center_hole_radius is not None and center_hole_radius > 0:
        center_cyl = Cylinder(Pnt(0, 0, -thickness), Z, center_hole_radius, 3*thickness)
        cycloid_solid = cycloid_solid - center_cyl
    
    # 转换为 OCCGeometry
    geo = OCCGeometry(cycloid_solid)
    return geo


def create_cycloid_geometry_occ_simple(
    profile_x: np.ndarray,
    profile_y: np.ndarray,
    thickness: float,
    hole_centers: List[Tuple[float, float]],
    hole_radius: float,
    center_hole_radius: Optional[float] = None
) -> 'OCCGeometry':
    """
    简化版：使用多边形而非样条创建摆线轮几何体。
    对于大量点的齿廓，这种方法更稳定。
    """
    if not NGSOLVE_AVAILABLE:
        raise ImportError("NGsolve/Netgen is required for flexible cycloid")
    
    n_points = len(profile_x)
    
    # 创建齿廓多边形的边
    edges = []
    for i in range(n_points):
        i_next = (i + 1) % n_points
        p1 = Pnt(profile_x[i], profile_y[i], 0)
        p2 = Pnt(profile_x[i_next], profile_y[i_next], 0)
        edges.append(Segment(p1, p2))
    
    # 创建闭合线框
    wire_outer = Wire(edges)
    face_outer = Face(wire_outer)
    
    # 拉伸成实体
    cycloid_solid = face_outer.Extrude(thickness * Z)
    
    # 3D 布尔运算减去孔（更稳健）
    for (cx, cy) in hole_centers:
        # 创建足够长的圆柱体以确保穿透
        hole_cyl = Cylinder(Pnt(cx, cy, -thickness), Z, hole_radius, 3*thickness)
        cycloid_solid = cycloid_solid - hole_cyl
    
    # 减去中心孔
    if center_hole_radius is not None and center_hole_radius > 0:
        center_cyl = Cylinder(Pnt(0, 0, -thickness), Z, center_hole_radius, 3*thickness)
        cycloid_solid = cycloid_solid - center_cyl
    
    # 转换为 OCCGeometry
    geo = OCCGeometry(cycloid_solid)
    return geo


def create_cycloid_face_2d(
    profile_x: np.ndarray,
    profile_y: np.ndarray,
    hole_centers: List[Tuple[float, float]],
    hole_radius: float,
    center_hole_radius: Optional[float] = None
) -> 'Face':
    """
    创建摆线轮的2D面（用于拉伸生成棱柱网格）。
    """
    if not NGSOLVE_AVAILABLE:
        raise ImportError("NGsolve/Netgen is required")
    
    n_points = len(profile_x)
    
    # 创建齿廓多边形的边
    edges = []
    for i in range(n_points):
        i_next = (i + 1) % n_points
        p1 = Pnt(profile_x[i], profile_y[i], 0)
        p2 = Pnt(profile_x[i_next], profile_y[i_next], 0)
        edges.append(Segment(p1, p2))
    
    # 创建闭合线框
    wire_outer = Wire(edges)
    face_outer = Face(wire_outer)
    
    # 减去孔
    for (cx, cy) in hole_centers:
        hole_circle = Circle(Pnt(cx, cy, 0), Z, hole_radius)
        hole_face = Face(Wire([hole_circle]))
        face_outer = face_outer - hole_face
    
    # 减去中心孔
    if center_hole_radius is not None and center_hole_radius > 0:
        center_circle = Circle(Pnt(0, 0, 0), Z, center_hole_radius)
        center_face = Face(Wire([center_circle]))
        face_outer = face_outer - center_face
    
    return face_outer


def generate_cycloid_mesh(
    profile_x: np.ndarray,
    profile_y: np.ndarray,
    thickness: float,
    hole_centers: List[Tuple[float, float]],
    hole_radius: float,
    center_hole_radius: Optional[float] = None,
    mesh_size: float = 5e-3,  # 默认5mm网格尺寸
    mesh_size_profile: Optional[float] = None,  # 齿廓处的网格尺寸
    n_layers: int = 2,  # 未使用（EXUDYN只支持四面体）
    use_simple_geometry: bool = True
) -> 'Mesh':
    """
    生成摆线轮有限元网格（3D四面体，EXUDYN只支持四面体单元）。
    
    Parameters
    ----------
    profile_x, profile_y : 齿廓坐标
    thickness : 厚度 (m)
    hole_centers : 孔中心列表
    hole_radius : 孔半径 (m)
    center_hole_radius : 中心孔半径 (m)
    mesh_size : 全局网格尺寸 (m)
    mesh_size_profile : 齿廓处网格尺寸，默认与mesh_size相同
    n_layers : 未使用（EXUDYN只支持四面体）
    use_simple_geometry : 使用简化多边形几何（更稳定）
    
    Returns
    -------
    NGsolve Mesh (四面体单元)
    """
    if not NGSOLVE_AVAILABLE:
        raise ImportError("NGsolve/Netgen is required")
    
    if mesh_size_profile is None:
        mesh_size_profile = mesh_size
    
    # EXUDYN 只支持四面体单元，使用3D几何体直接生成
    # 创建3D几何体
    if use_simple_geometry:
        geo3d = create_cycloid_geometry_occ_simple(
            profile_x, profile_y, thickness,
            hole_centers, hole_radius, center_hole_radius
        )
    else:
        geo3d = create_cycloid_geometry_occ(
            profile_x, profile_y, thickness,
            hole_centers, hole_radius, center_hole_radius
        )
    
    # 生成3D四面体网格 (Netgen mesh)
    netgen_mesh = geo3d.GenerateMesh(maxh=mesh_size)
    
    # 转换为 NGsolve mesh (EXUDYN FEM接口需要 NGsolve mesh)
    ngsolve_mesh = Mesh(netgen_mesh)
    
    return ngsolve_mesh


def create_fem_from_mesh(
    mesh: 'Mesh',
    density: float,
    youngs_modulus: float,
    poissons_ratio: float,
    n_modes: int = 30
) -> Tuple[FEMinterface, Dict]:
    """
    从 NGsolve 网格创建 EXUDYN FEM 接口并计算模态。
    
    Parameters
    ----------
    mesh : NGsolve Mesh
    density : 密度 (kg/m³)
    youngs_modulus : 弹性模量 (Pa)
    poissons_ratio : 泊松比
    n_modes : 保留的模态数量
    
    Returns
    -------
    fem : FEMinterface 对象
    info : 包含额外信息的字典
    """
    fem = FEMinterface()
    
    # 从 NGsolve 导入网格（自动计算质量和刚度矩阵）
    print(f"Importing mesh and computing matrices...")
    fem.ImportMeshFromNGsolve(mesh, density=density, youngsModulus=youngs_modulus, 
                               poissonsRatio=poissons_ratio)
    
    # 计算自由-自由模态（无边界约束，排除刚体模态）
    print(f"Computing {n_modes} eigenmodes...")
    fem.ComputeEigenmodes(nModes=n_modes, excludeRigidBodyModes=6)  # 排除6个刚体模态
    
    # 获取信息
    n_nodes = fem.NumberOfNodes()
    n_elements = len(fem.elements) if hasattr(fem, 'elements') else 0
    
    # 获取特征值（属性而非方法）
    eigenvalues = fem.eigenValues if hasattr(fem, 'eigenValues') else np.array([])
    eigenfreqs = np.sqrt(np.abs(eigenvalues)) / (2 * np.pi) if len(eigenvalues) > 0 else np.array([])
    
    info = {
        'n_nodes': n_nodes,
        'n_elements': n_elements,
        'n_modes': n_modes,
        'eigenfrequencies': eigenfreqs  # Hz
    }
    
    print(f"FEM created: {n_nodes} nodes, {n_modes} modes")
    if len(eigenfreqs) >= 5:
        print(f"First 5 eigenfrequencies: {eigenfreqs[:5]} Hz")
    
    return fem, info


def get_profile_node_indices(
    fem: FEMinterface,
    profile_x: np.ndarray,
    profile_y: np.ndarray,
    z_plane: float = 0.0,
    tolerance: float = None  # 自动设置
) -> np.ndarray:
    """
    获取齿廓上的有限元节点索引，按齿廓曲线顺序排序。
    
    Parameters
    ----------
    fem : FEMinterface
    profile_x, profile_y : 齿廓坐标（按环绕顺序）
    z_plane : z平面位置
    tolerance : 搜索容差 (m)，默认自动设置
    
    Returns
    -------
    按齿廓顺序排序的节点索引数组
    """
    nodes = fem.GetNodePositionsAsArray()
    n_nodes = len(nodes)
    n_profile = len(profile_x)
    
    # 自动设置容差（基于齿廓点间距）
    if tolerance is None:
        # 估算齿廓点间距
        profile_spacing = np.mean(np.sqrt(np.diff(profile_x)**2 + np.diff(profile_y)**2))
        tolerance = profile_spacing * 2  # 2倍间距作为容差
        print(f"  Auto tolerance: {tolerance*1000:.2f} mm")
    
    # 找外表面节点（z ≈ 0）
    z_values = nodes[:, 2]
    z_min_mesh, z_max_mesh = np.min(z_values), np.max(z_values)
    z_tol = (z_max_mesh - z_min_mesh) * 0.1
    
    # 筛选出 z ≈ z_min 的节点（底面）
    surface_mask = np.abs(z_values - z_min_mesh) < z_tol
    surface_node_indices = np.where(surface_mask)[0]
    surface_nodes_xy = nodes[surface_node_indices, :2]  # 只取 x, y
    
    # 对每个原始齿廓点，找最近的表面节点
    # 允许同一网格节点被多个齿廓点使用（因为网格节点数远少于齿廓点数）
    profile_nodes = []
    
    for i in range(n_profile):
        px, py = profile_x[i], profile_y[i]
        
        # 计算到所有表面节点的距离
        distances = np.sqrt((surface_nodes_xy[:, 0] - px)**2 + 
                           (surface_nodes_xy[:, 1] - py)**2)
        
        # 找最近的节点（允许重复使用）
        min_idx = np.argmin(distances)
        if distances[min_idx] < tolerance:
            profile_nodes.append(surface_node_indices[min_idx])
        else:
            # 容差内找不到，使用最近的节点
            profile_nodes.append(surface_node_indices[min_idx])
    
    return np.array(profile_nodes, dtype=int)


def get_hole_center_node(
    fem: FEMinterface,
    hole_center_x: float,
    hole_center_y: float,
    z_plane: float = 0.0
) -> int:
    """
    获取最接近孔中心的节点索引。
    """
    nodes = fem.GetNodePositionsAsArray()
    
    # 计算所有节点到孔中心的距离
    distances = np.sqrt((nodes[:, 0] - hole_center_x)**2 + 
                       (nodes[:, 1] - hole_center_y)**2 +
                       (nodes[:, 2] - z_plane)**2)
    
    return int(np.argmin(distances))


def get_hole_nodes(
    fem: FEMinterface,
    hole_center_x: float,
    hole_center_y: float,
    hole_radius: float,
    n_nodes: int = 8
) -> List[int]:
    """
    获取孔周围的多个节点（用于 MarkerSuperElementRigid）。
    返回距离孔边缘最近的 n_nodes 个节点。
    """
    nodes = fem.GetNodePositionsAsArray()
    
    # 计算每个节点到孔边缘的距离
    dist_to_center = np.sqrt((nodes[:, 0] - hole_center_x)**2 + 
                             (nodes[:, 1] - hole_center_y)**2)
    dist_to_edge = np.abs(dist_to_center - hole_radius)
    
    # 找到距离孔边缘最近的节点
    sorted_indices = np.argsort(dist_to_edge)
    hole_nodes = sorted_indices[:n_nodes].tolist()
    
    return hole_nodes


def create_flexible_cycloid(
    exu,
    mbs,
    fem: FEMinterface,
    position_ref: List[float],
    rotation_matrix_ref: np.ndarray = None,
    gravity: List[float] = None,
    modal_damping: float = 0.01
) -> Dict[str, Any]:
    """
    在 EXUDYN 系统中创建柔性摆线轮。
    
    Parameters
    ----------
    exu : exudyn module
    mbs : MainSystem
    fem : FEMinterface (已计算模态)
    position_ref : 参考位置 [x, y, z]
    rotation_matrix_ref : 参考旋转矩阵 (3x3)，默认单位阵
    gravity : 重力向量，默认 [0,0,0]
    modal_damping : 模态阻尼比
    
    Returns
    -------
    包含 bodyNumber, nodeNumber, cms 等信息的字典
    """
    if rotation_matrix_ref is None:
        rotation_matrix_ref = np.eye(3)
    if gravity is None:
        gravity = [0, 0, 0]
    
    # 创建 CMS 接口
    cms = ObjectFFRFreducedOrderInterface(fem)
    
    # 添加柔性体（需要 exu 和 mbs 两个参数）
    objFFRF = cms.AddObjectFFRFreducedOrderWithUserFunctions(
        exu, mbs,
        positionRef=position_ref,
        rotationMatrixRef=rotation_matrix_ref,
        gravity=gravity,
        massProportionalDamping=modal_damping,
        stiffnessProportionalDamping=modal_damping * 0.001
    )
    
    # 返回的键是: nRigidBody, nGenericODE2, oFFRFreducedOrder, loadGravity
    return {
        'bodyNumber': objFFRF['oFFRFreducedOrder'],  # 体编号
        'nodeNumber': objFFRF['nRigidBody'],         # 刚体节点编号
        'modalNodeNumber': objFFRF['nGenericODE2'],  # 模态坐标节点编号
        'cms': cms,
        'fem': fem
    }


def create_segments_data_from_profile(
    profile_x: np.ndarray,
    profile_y: np.ndarray
) -> np.ndarray:
    """
    从原始齿廓曲线创建接触用的 segmentsData。
    
    每行格式: [x0, y0, x1, y1]
    使用原始曲线坐标，C++ 端会根据网格节点位移进行变形。
    """
    n_profile = len(profile_x)
    segments_data = np.zeros((n_profile, 4))
    
    for i in range(n_profile):
        segments_data[i, 0] = profile_x[i]  # x0
        segments_data[i, 1] = profile_y[i]  # y0
        segments_data[i, 2] = profile_x[(i + 1) % n_profile]  # x1
        segments_data[i, 3] = profile_y[(i + 1) % n_profile]  # y1
    
    return segments_data


# =================== 高级接口 ===================

class FlexibleCycloidWheel:
    """
    柔性摆线轮类，封装网格生成、模态计算和体创建。
    """
    
    def __init__(
        self,
        profile_x: np.ndarray,
        profile_y: np.ndarray,
        thickness: float,
        hole_centers: List[Tuple[float, float]],
        hole_radius: float,
        center_hole_radius: Optional[float] = None,
        density: float = 7850.0,  # kg/m³ (steel)
        youngs_modulus: float = 2.1e11,  # Pa (steel)
        poissons_ratio: float = 0.3,
        n_modes: int = 30,
        mesh_size: float = 5e-3,
        n_layers: int = 2,  # 厚度方向层数
        # 位置相关刚度参数
        flange_hole_angles: Optional[List[float]] = None,  # 法兰孔角度 (度)
        base_stiffness: float = 1e11,  # 基础刚度 (N/m)
        stiffness_min_ratio: float = 0.3,  # 孔处最小刚度比例
        stiffness_influence_angle: float = 30.0,  # 影响范围 (度)
    ):
        """初始化柔性摆线轮参数"""
        self.profile_x = np.asarray(profile_x)
        self.profile_y = np.asarray(profile_y)
        self.thickness = thickness
        self.hole_centers = hole_centers
        self.hole_radius = hole_radius
        self.center_hole_radius = center_hole_radius
        
        self.density = density
        self.youngs_modulus = youngs_modulus
        self.poissons_ratio = poissons_ratio
        self.n_modes = n_modes
        self.mesh_size = mesh_size
        self.n_layers = n_layers
        
        # 位置相关刚度参数
        self.flange_hole_angles = flange_hole_angles if flange_hole_angles else [60.0, 180.0, 300.0]
        self.base_stiffness = base_stiffness
        self.stiffness_min_ratio = stiffness_min_ratio
        self.stiffness_influence_angle = stiffness_influence_angle
        
        # 将在 build() 中填充
        self.mesh = None
        self.fem = None
        self.profile_node_indices = None
        self.segments_data = None
        self.stiffness_distribution = None  # 刚度分布数组
        self.is_built = False
    
    def build(self, verbose: bool = True):
        """生成网格并计算模态"""
        if verbose:
            print("=" * 50)
            print("Building flexible cycloid wheel...")
            print(f"  Profile points: {len(self.profile_x)}")
            print(f"  Thickness: {self.thickness * 1000:.2f} mm")
            print(f"  Holes: {len(self.hole_centers)}")
            print(f"  Mesh size: {self.mesh_size * 1000:.2f} mm")
        
        # 生成网格（3D四面体，EXUDYN只支持四面体）
        if verbose:
            print("Generating 3D tetrahedral mesh...")
        self.mesh = generate_cycloid_mesh(
            self.profile_x, self.profile_y, self.thickness,
            self.hole_centers, self.hole_radius, self.center_hole_radius,
            mesh_size=self.mesh_size, use_simple_geometry=True
        )
        
        # 创建 FEM 并计算模态
        if verbose:
            print("Computing FEM and modes...")
        self.fem, info = create_fem_from_mesh(
            self.mesh, self.density, self.youngs_modulus, self.poissons_ratio,
            n_modes=self.n_modes
        )
        
        # 获取齿廓节点（在z=0表面）
        if verbose:
            print("Finding profile nodes...")
        self.profile_node_indices = get_profile_node_indices(
            self.fem, self.profile_x, self.profile_y
        )
        
        # 生成接触用的 segments data（使用原始曲线坐标）
        if len(self.profile_node_indices) > 0:
            self.segments_data = create_segments_data_from_profile(
                self.profile_x, self.profile_y
            )
        
        # 计算位置相关刚度分布
        self.stiffness_distribution = self.compute_stiffness_distribution()
        
        self.is_built = True
        
        if verbose:
            print(f"Build complete!")
            print(f"  FEM nodes: {info['n_nodes']}")
            print(f"  Profile nodes: {len(self.profile_node_indices)}")
            print(f"  Eigenfrequencies: {info['eigenfrequencies'][:3]} Hz ...")
            print(f"  Stiffness range: {self.stiffness_distribution.min():.2e} - {self.stiffness_distribution.max():.2e} N/m")
            print("=" * 50)
        
        return self
    
    def add_to_system(
        self,
        exu,
        mbs,
        position_ref: List[float],
        rotation_matrix_ref: np.ndarray = None,
        gravity: List[float] = None
    ) -> Dict[str, Any]:
        """将柔性体添加到 EXUDYN 系统"""
        if not self.is_built:
            raise RuntimeError("Must call build() first")
        
        result = create_flexible_cycloid(
            exu, mbs, self.fem, position_ref, rotation_matrix_ref, gravity
        )
        result['profile_node_indices'] = self.profile_node_indices
        result['segments_data'] = self.segments_data
        result['wheel'] = self
        
        return result
    
    def get_hole_node(self, hole_index: int) -> int:
        """获取指定孔中心的节点索引"""
        if not self.is_built:
            raise RuntimeError("Must call build() first")
        
        cx, cy = self.hole_centers[hole_index]
        return get_hole_center_node(self.fem, cx, cy, self.thickness / 2)
    
    def get_hole_nodes_list(self, hole_index: int, n_nodes: int = 8) -> List[int]:
        """获取指定孔周围的多个节点索引（用于 MarkerSuperElementRigid）"""
        if not self.is_built:
            raise RuntimeError("Must call build() first")
        
        cx, cy = self.hole_centers[hole_index]
        return get_hole_nodes(self.fem, cx, cy, self.hole_radius, n_nodes)
    
    def compute_stiffness_distribution(self) -> np.ndarray:
        """
        计算每个齿廓点的基体刚度分布。
        
        刚度在法兰孔角度附近降低，使用余弦函数平滑过渡。
        
        Returns
        -------
        stiffness : np.ndarray
            与 profile_x/profile_y 长度相同的刚度数组 (N/m)
        """
        n_points = len(self.profile_x)
        stiffness = np.zeros(n_points)
        
        # 转换影响角度为弧度
        influence_rad = np.radians(self.stiffness_influence_angle)
        
        for i in range(n_points):
            # 计算该点的极坐标角度 (弧度)
            angle_rad = np.arctan2(self.profile_y[i], self.profile_x[i])
            angle_deg = np.degrees(angle_rad) % 360
            
            # 计算到最近法兰孔的角度距离
            min_dist_deg = 180.0
            for hole_angle in self.flange_hole_angles:
                # 处理角度环绕
                dist = abs(angle_deg - hole_angle)
                dist = min(dist, 360 - dist)
                min_dist_deg = min(min_dist_deg, dist)
            
            # 平滑刚度衰减 (余弦形状)
            if min_dist_deg < self.stiffness_influence_angle:
                # 从 k_min_ratio 到 1.0 的余弦过渡
                # 在 hole 处: ratio = k_min_ratio
                # 在 influence_angle 边界: ratio = 1.0
                t = min_dist_deg / self.stiffness_influence_angle  # 0 到 1
                # 余弦插值: cos(0)=1 (hole处), cos(pi)=-1 (边界)
                # 映射到 k_min_ratio 到 1.0
                ratio = self.stiffness_min_ratio + (1 - self.stiffness_min_ratio) * (1 - np.cos(np.pi * t)) / 2
            else:
                ratio = 1.0
            
            stiffness[i] = self.base_stiffness * ratio
        
        return stiffness
    
    def get_stiffness_per_segment(self) -> np.ndarray:
        """
        返回与 segments_data 对应的刚度数组。
        
        Returns
        -------
        stiffness : np.ndarray
            刚度数组，长度与 segments_data 行数相同
        """
        if self.stiffness_distribution is None:
            self.stiffness_distribution = self.compute_stiffness_distribution()
        return self.stiffness_distribution
