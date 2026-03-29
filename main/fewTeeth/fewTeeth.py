import sys, os

sys.exudynFast = True          # 让 exudyn 优先加载 fast 版（依赖 AVX2）
sys.exudynCPUhasAVX2 = True 

import numpy as np
import math

import exudyn as exu
from exudyn.utilities import *
from fewTeeth_profile import FewTeethGearPair, generate_few_teeth_flanks
import exudyn.graphics as graphics
from tooth_stiffness import create_stiffness_table_for_gear
from exudyn.itemInterface import (
    ObjectContactCircleCircle,
    MarkerBodyPosition,
    SensorObject,
    ObjectContactFewTeeth,
    MarkerBodyRigid,
    NodeGenericData,
    VObjectContactFewTeeth,
)

useGraphics = True  # 启用图形窗口显示
SC = exu.SystemContainer()
mbs = SC.AddSystem()

# Contact visualization toggles for consistent arrow output
SHOW_CONTACT_GEOMETRY = True  # 必须为True才能显示ContactCurveCircles的力箭头
CONTACT_FORCE_SCALE = 3e-5  # enlarge contact force arrows for visibility
SHOW_CONTACT_FORCE_VALUES = True
CONTACT_POINT_SIZE = 1.5e-3

# Aggregated contact monitor registry (filled after contact objects are created)

CONTACT_MONITOR_GROUPS = {}
CONTACT_NODE_INFO = []

# 限制调试输出规模：只对特定分组或当接触异常时打印详细信息
CONTACT_DEBUG_GROUPS = {'FlangeHole', 'FlangeShaft'}
CONTACT_STEP_PRINT_INTERVAL = 200  # 常规输出的时间步间隔
CONTACT_DEBUG_GAP_TRIGGER = -5e-4  # 当间隙小于该阈值时强制打印
CONTACT_DEBUG_FORCE_TRIGGER = 5e5  # 当法向力超过该值时强制打印

# 注册各类接触对象，便于独立开关
contact_objects_by_group = {}

def _register_contact_object(group_name, object_index):
    contact_objects_by_group.setdefault(group_name, []).append(object_index)

def _register_contact_info(group_name, label, node_number, contact_object, segment_lengths, stiffness, damping):
    info = {
        'group': group_name,
        'label': label,
        'node': node_number,
        'object': contact_object,
        'segment_lengths': np.asarray(segment_lengths, dtype=float),
        'stiffness': float(stiffness),
        'damping': float(damping),
    }
    CONTACT_NODE_INFO.append(info)
    CONTACT_MONITOR_GROUPS.setdefault(group_name, []).append(info)


# =================== 参数（SI 单位：米-千克-秒） ===================
# 所有长度参数使用米（原毫米值 × 1e-3）

# 少齿差齿轮参数
z_b = 60                  # 外齿轮（主动轮）齿数

R_cycloid = 119.5e-3      # 摆线轮（外齿轮）外径半径 (m)

# 曲柄轴参数
r_crank_main = 20e-3      # 曲柄主轴半径 (m)
r_crank_eccentric = 22.5e-3  # 曲柄偏心段半径 (m)
L_crank_main1 = 20e-3     # 曲柄主轴段1长度 (m)
L_crank_eccentric1 = 15e-3   # 偏心段1长度 (m)
L_crank_middle = 5e-3     # 中间段长度 (m)
L_crank_eccentric2 = 15e-3   # 偏心段2长度 (m)
L_crank_main2 = 20e-3     # 曲柄主轴段2长度 (m)
# eccentric_offset 在 gear_geometry 计算后赋值（见第183行）

z_eccentric1 = L_crank_eccentric1 / 2
z_eccentric2 = L_crank_eccentric1 + L_crank_middle + L_crank_eccentric2 / 2

# 滚针轴承参数（摆线轮孔）
r_needle = 4.25e-3        # 滚针半径 (m)
n_needles = 18            # 每个轴承的滚针数量
bearing_clearance = 0.01e-3  # 轴承径向间隙 (m)
r_needle_pitch = r_crank_eccentric + r_needle  # 滚针节圆半径 (m)
r_hole = r_needle_pitch + r_needle + bearing_clearance  # 摆线轮孔半径 (m)

# 壳体（内齿圈壳）参数
thickness_shell = 50e-3   # 壳体厚度 (m)
R_shell_outer = 140e-3    # 壳体外圆半径 (m)
R_shell_inner = R_cycloid + 0.5e-3  # 壳体内圆半径（略大于摆线轮）(m)

# 三曲柄轴配置参数
n_cranks = 3  # 曲柄轴数量
crank_distribution_radius = 63e-3  # 曲柄轴分布圆半径 (m)

ANGLE_OFFSET = 0.5 * np.pi  # 统一让圆周点从 +Y 轴开始

# 接触参数（SI单位）
contactDamping_tooth = 1e6  # N/(m/s) - 主齿轮-内齿圈接触阻尼

# 齿轮材料和几何参数（用于刚度计算）
face_width_tooth = 15e-3      # 齿宽 [m]
elasticModulus_tooth = 2.1e11  # 弹性模量 [Pa]
poissonRatio_tooth = 0.3       # 泊松比

# 位置相关的单齿刚度表（沿齿廓参数 t ∈ [0,1] 均匀分布）
# t=0 齿根, t=0.5 齿中, t=1 齿顶
# 注意：这里只是默认值，实际值在 gear_profile_params 定义后由 tooth_stiffness 计算
# 外齿轮单齿刚度表 [N/m]
stiffnessOuter_tooth = [1.0e8, 1.2e8, 1.4e8, 1.2e8, 0.9e8]  # 5个点：根→顶
# 内齿轮单齿刚度表 [N/m]
stiffnessInner_tooth = [1.5e8, 1.8e8, 2.0e8, 1.8e8, 1.4e8]  # 5个点：根→顶
contactStiffness_hole = 1e9  # N/m - 孔接触刚度
contactDamping_hole = 1e6  # N/(m/s) - 孔接触阻尼

contactStiffness_cage = 1e7  # N/m - 滚针保持架孔接触刚度
contactDamping_cage = 1e4  # N/(m/s) - 滚针保持架孔接触阻尼

# 输入轴-曲柄轴齿轮模拟参数
TORSIONAL_SPRING_STIFFNESS = 1e7  # N·m/rad
TORSIONAL_SPRING_DAMPING = 1e4    # N·m·s/rad
TORSIONAL_GEAR_RATIO = -1.0/3.0       # 让曲柄轴反向等速转动；根据需要修改

TORSIONAL_BACKLASH = np.deg2rad(0.002)  # 齿隙（弧度）
TORSIONAL_BACKLASH_STIFFNESS_SCALE = 0.05  # 齿隙区刚度比例
TORSIONAL_BACKLASH_DAMPING_SCALE = 0.05    # 齿隙区阻尼比例

# TORSIONAL_BACKLASH = np.deg2rad(0.0)  # 齿隙（弧度）
# TORSIONAL_BACKLASH_STIFFNESS_SCALE = 0.0  # 齿隙区刚度比例
# TORSIONAL_BACKLASH_DAMPING_SCALE = 0.0    # 齿隙区阻尼比例

# 接触摩擦系数（库仑摩擦）
friction_cycloid_pin = 0.2             # 主齿轮-内齿圈
friction_crank_needle = 0.1            # 偏心轴-滚针
friction_needle_cycloid_hole = 0.15     # 滚针-摆线孔
friction_flange_bearing = 0.1         # 法兰轴承滚针

# Bristle 静摩擦刚度 (N/m)，0=禁用，使用速度惩罚模型
# 典型值：1e7 ~ 1e9，粘滞区位移 = μN/k（如 0.2×1000N/1e8 = 2μm）
frictionStiffness_cycloid_pin = 1e8        # 主齿轮-内齿圈 (N/m)
frictionStiffness_bearing = 1e8            # 轴承（滚针接触）(N/m)

# friction_cycloid_pin = 0.0             # 主齿轮-内齿圈
# friction_crank_needle = 0.0            # 偏心轴-滚针
# friction_needle_cycloid_hole = 0.0     # 滚针-摆线孔
# friction_flange_bearing = 0.0          # 法兰轴承滚针

# 保持架可视化参数
cage_segments_ring = 96
cage_segments_hole = 48
cage_ring_margin = 0.4e-3
cage_hole_clearance = 0.2e-3
cage_color_cycloid = [0.95, 0.6, 0.2, 1.0]
cage_color_flange = [0.2, 0.6, 0.95, 1.0]
cage_mass_cycloid = 0.02  # kg
cage_mass_flange = 0.02  # kg

# 法兰轴承参数
r_flange_shaft = r_crank_main  # 法兰孔与曲柄主轴接触（主轴半径）(m)
r_flange_needle = 5.0e-3  # 法兰轴承滚针半径 (m)
n_flange_needles = 12  # 每个法兰轴承的滚针数量
flange_bearing_clearance = 0.01e-3  # 法兰轴承间隙 (m)
r_flange_needle_pitch = r_flange_shaft + r_flange_needle
r_flange_hole = r_flange_needle_pitch + r_flange_needle + flange_bearing_clearance  # 法兰孔半径 (m)

# 孔位置误差参数（制造公差）
np.random.seed(42)  # 固定种子，确保可重复
hole_position_error_std = 5e-6   # 孔位置误差标准差 5μm

print("=" * 60)
print("三曲柄轴双片摆线针轮减速器仿真")
print(f"曲柄: {n_cranks}, 摆线滚针: {n_needles}/轴承, 法兰滚针: {n_flange_needles}/轴承")
print("=" * 60)

oGround = mbs.CreateGround()

# 生成少齿差齿面数据
gear_profile_params = FewTeethGearPair(
    z_outer=z_b,              # 外齿轮齿数 = 60
    z_inner=z_b + 2,          # 内齿圈齿数 = 62（少齿差：2齿）
    module=4.0e-3,            # 模数 m = 4mm
    x_outer=0.0,              # 外齿轮变位系数（零变位）
    x_inner=0.25,              # 内齿圈变位系数（正变位）
    alpha_deg=20.0,           # 压力角 = 20°（标准值）
    ha_star=0.64,             # 齿顶高系数 = 0.75
    c_star=0.5,               # 顶隙系数 = 0.5
    flank_points_outer=120,   # 外齿面采样点数
    flank_points_inner=160,   # 内齿面采样点数
    tip_probe_length=0.005,   # 齿顶探针长度（与接触阈值保持一致）
)

gear_flanks, gear_geometry = generate_few_teeth_flanks(gear_profile_params)

# 根据少齿差几何计算实际中心距作为偏心距
e = gear_geometry.center_distance
eccentric_offset = e

print(f"实际中心距 / 偏心距 e = {e*1e3:.3f} mm")

# ============================================================================
# 使用 tooth_stiffness 计算结构刚度表（不含接触刚度，接触刚度在 C++ 中实时计算）
# ============================================================================
print("\n计算单齿结构刚度表...")
stiffnessOuter_tooth = create_stiffness_table_for_gear(
    z=gear_profile_params.z_outer,
    module=gear_profile_params.module,
    face_width=face_width_tooth,
    alpha_deg=gear_profile_params.alpha_deg,
    ha_star=gear_profile_params.ha_star,
    c_star=gear_profile_params.c_star,
    x=gear_profile_params.x_outer,
    is_internal=False,
    E=elasticModulus_tooth,
    poisson=poissonRatio_tooth,
    n_points=5,
    include_contact=False  # 只计算结构刚度（弯曲+齿根）
)

stiffnessInner_tooth = create_stiffness_table_for_gear(
    z=gear_profile_params.z_inner,
    module=gear_profile_params.module,
    face_width=face_width_tooth,
    alpha_deg=gear_profile_params.alpha_deg,
    ha_star=gear_profile_params.ha_star,
    c_star=gear_profile_params.c_star,
    x=gear_profile_params.x_inner,
    is_internal=True,
    E=elasticModulus_tooth,
    poisson=poissonRatio_tooth,
    n_points=5,
    include_contact=False  # 只计算结构刚度（弯曲+齿根）
)

print(f"  外齿轮结构刚度表: {[f'{k/1e6:.1f}' for k in stiffnessOuter_tooth]} MN/m")
print(f"  内齿轮结构刚度表: {[f'{k/1e6:.1f}' for k in stiffnessInner_tooth]} MN/m")
print(f"  接触刚度将在 C++ 中实时计算（使用等效曲率半径）")

theta_offset_inner = math.pi / gear_profile_params.z_inner


def _flank_sign(flank_side: str) -> float:
    return 1.0 if flank_side == "right" else -1.0


def build_outer_analytic(flank_side: str, *, rotation_offset: float = 0.0) -> dict:
    return {
        "isInner": False,
        "replicateAllTeeth": True,
        "numberOfTeeth": int(gear_profile_params.z_outer),
        "singleToothIndex": 0,
        "module": float(gear_profile_params.module),
        "alpha": float(gear_profile_params.alpha),
        "haStar": float(gear_profile_params.ha_star),
        "cStar": float(gear_profile_params.c_star),
        "x": float(gear_profile_params.x_outer),
        "deltaY": 0.0,
        "baseRotation": float(rotation_offset),
        "flankSign": _flank_sign(flank_side),
        "reverseParam": flank_side == "left",
    }


def build_inner_analytic(tooth_index: int, flank_side: str) -> dict:
    return {
        "isInner": True,
        "replicateAllTeeth": False,
        "numberOfTeeth": int(gear_profile_params.z_inner),
        "singleToothIndex": int(tooth_index),
        "module": float(gear_profile_params.module),
        "alpha": float(gear_profile_params.alpha),
        "haStar": float(gear_profile_params.ha_star),
        "cStar": float(gear_profile_params.c_star),
        "x": float(gear_profile_params.x_inner),
        "deltaY": float(gear_geometry.delta_y),
        "baseRotation": float(theta_offset_inner),
        "flankSign": _flank_sign(flank_side),
        "reverseParam": flank_side == "left",
    }


def _stack_points(flanks_dict, gear_id, flank_side=None):
    """提取并堆叠齿面点集"""
    if flank_side is None:
        points_list = [points for (gid, _, _), points in flanks_dict.items() if gid == gear_id]
    else:
        points_list = [points for (gid, _, fside), points in flanks_dict.items()
                       if gid == gear_id and fside == flank_side]
    if points_list:
        return np.vstack(points_list)
    return np.zeros((0, 2), dtype=float)


def rotate_points(points: np.ndarray, angle: float) -> np.ndarray:
    if points.size == 0:
        return points.copy()
    c = math.cos(angle)
    s = math.sin(angle)
    rot = np.array([[c, -s], [s, c]], dtype=float)
    return (rot @ points.T).T


def rotate_segments(segments: np.ndarray, angle: float) -> np.ndarray:
    if segments.size == 0:
        return segments.copy()
    c = math.cos(angle)
    s = math.sin(angle)
    rot = np.array([[c, -s], [s, c]], dtype=float)
    rotated = np.empty_like(segments)
    rotated[:, 0:2] = (rot @ segments[:, 0:2].T).T
    rotated[:, 2:4] = (rot @ segments[:, 2:4].T).T
    return rotated


def points_to_segments(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if points.shape[0] < 2:
        return np.zeros((0, 4), dtype=float)
    segments = []
    prev = points[0]
    for curr in points[1:]:
        if np.any(np.isnan(prev)) or np.any(np.isnan(curr)):
            prev = curr
            continue
        segments.append([prev[0], prev[1], curr[0], curr[1]])
        prev = curr
    if segments:
        return np.asarray(segments, dtype=float)
    return np.zeros((0, 4), dtype=float)


def build_segments_from_flanks(flanks_dict, gear_id, *, tooth_count):
    segments = []
    for tooth_idx in range(tooth_count):
        for flank_side in ["left", "right"]:
            key = (gear_id, tooth_idx, flank_side)
            if key not in flanks_dict:
                continue
            pts = np.asarray(flanks_dict[key], dtype=float)
            segs = points_to_segments(pts)
            if segs.size:
                segments.append(segs)
    if segments:
        return np.vstack(segments)
    return np.zeros((0, 4), dtype=float)


# 生成分齿面的齿廓点集
outer_points_master_left = _stack_points(gear_flanks, "outer", "left")
outer_points_master_right = _stack_points(gear_flanks, "outer", "right")
inner_points_master_left = _stack_points(gear_flanks, "inner", "left")
inner_points_master_right = _stack_points(gear_flanks, "inner", "right")

# 保留完整齿廓（用于可视化）
outer_points_master = _stack_points(gear_flanks, "outer")
inner_points_master = _stack_points(gear_flanks, "inner")
outer_segments_master = build_segments_from_flanks(gear_flanks, "outer", tooth_count=gear_profile_params.z_outer)
inner_segments_master = build_segments_from_flanks(gear_flanks, "inner", tooth_count=gear_profile_params.z_inner)


def rotate_point_cloud(points: np.ndarray, angle: float) -> np.ndarray:
    return rotate_points(points, angle)


# 第二片摆线轮齿廓（旋转180度）
outer_points_shifted = rotate_point_cloud(outer_points_master, math.pi)
outer_points_shifted_left = rotate_point_cloud(outer_points_master_left, math.pi)
outer_points_shifted_right = rotate_point_cloud(outer_points_master_right, math.pi)
outer_segments_shifted = rotate_segments(outer_segments_master, math.pi)


def ComputeSegmentLengths(pointsData):
    pointsData = np.asarray(pointsData, dtype=float)
    if pointsData.shape[0] < 2:
        return np.zeros(0)
    delta = pointsData[1:] - pointsData[:-1]
    return np.linalg.norm(delta, axis=1)


def create_circle_contact(mbs, *, name, markerA, markerB,
                          radiusA, radiusB,
                          stiffness, damping,
                          friction=0.0,
                          frictionVelocityPenalty=1000.0,    # 速度惩罚摩擦模型，更稳定
                          frictionProportionalZone=0.00,    # 平滑零速度附近的摩擦力
                          frictionStiffness=0.0,            # Bristle静摩擦刚度，0=禁用
                          active=True,
                          group=None):
    node = mbs.AddNode(NodeGenericData(
        initialCoordinates=[0., 0., 0., 0., 0., 0., 0., 0.],  # 增加2个用于Bristle粘滞位置
        numberOfDataCoordinates=8
    ))
    obj = mbs.AddObject(ObjectContactCircleCircle(
        name=name,
        markerNumbers=[markerA, markerB],
        nodeNumber=node,
        radius1=radiusA,
        radius2=radiusB,
        contactStiffness=stiffness,
        contactDamping=damping,
        frictionCoefficient=friction,
        frictionVelocityPenalty=frictionVelocityPenalty,
        frictionProportionalZone=frictionProportionalZone,
        frictionStiffness=frictionStiffness,
        activeConnector=active
    ))

    if group is not None:
        info = {
                    'group': group,
                    'label': name,
                    'node': node,
                    'object': obj,
            'stiffness': float(stiffness),
            'damping': float(damping),
        }
        CONTACT_NODE_INFO.append(info)
        CONTACT_MONITOR_GROUPS.setdefault(group, []).append(info)

    return node, obj


segmentsData_tooth1 = outer_segments_master
segmentsData_tooth2 = outer_segments_shifted
nSeg_tooth1 = segmentsData_tooth1.shape[0]
nSeg_tooth2 = segmentsData_tooth2.shape[0]
segment_lengths_tooth1 = ComputeSegmentLengths(outer_points_master)
segment_lengths_tooth2 = ComputeSegmentLengths(outer_points_shifted)


def circle_line_graphics(radius, color, *, n_segments=cage_segments_ring, center=(0.0, 0.0, 0.0)):
    lines = []
    cx, cy, cz = center
    segments = max(3, int(n_segments))
    for i in range(segments):
        phi1 = 2 * np.pi * i / segments
        phi2 = 2 * np.pi * (i + 1) / segments
        x1 = cx + radius * np.cos(phi1)
        y1 = cy + radius * np.sin(phi1)
        x2 = cx + radius * np.cos(phi2)
        y2 = cy + radius * np.sin(phi2)
        lines.append({'type': 'Line', 'color': color, 'data': [x1, y1, cz, x2, y2, cz]})
    return lines


def create_cage_ring_graphics(*, center, pitch_radius, hole_radius, n_holes,
                              ring_inner_radius, ring_outer_radius,
                              color_ring, color_hole,
                              angle_offset=0.0,
                              ring_segments=cage_segments_ring,
                              hole_segments=cage_segments_hole):
    graphics_list = []

    ring_mid_radius = pitch_radius
    if ring_inner_radius is not None and ring_outer_radius is not None and ring_outer_radius > 0.0:
        ring_mid_radius = 0.5 * max(ring_outer_radius + ring_inner_radius, 0.0)
    elif ring_outer_radius is not None and ring_outer_radius > 0.0:
        ring_mid_radius = ring_outer_radius
    elif ring_inner_radius is not None and ring_inner_radius > 0.0:
        ring_mid_radius = ring_inner_radius

    ring_mid_radius = max(ring_mid_radius, 1e-9)
    graphics_list.extend(circle_line_graphics(ring_mid_radius, color_ring,
                                              n_segments=ring_segments, center=center))

    if n_holes > 0 and hole_radius > 0.0:
        cx, cy, cz = center
        for i in range(n_holes):
            angle = angle_offset + 2 * np.pi * i / n_holes
            hx = cx + pitch_radius * np.cos(angle)
            hy = cy + pitch_radius * np.sin(angle)
            graphics_list.extend(circle_line_graphics(hole_radius, color_hole,
                                                     n_segments=hole_segments,
                                                     center=(hx, hy, cz)))

    return graphics_list


def compute_cage_radii(pitch_radius, roller_radius, outer_limit, *,
                       hole_clearance=cage_hole_clearance,
                       ring_margin=cage_ring_margin):
    cage_hole_radius = roller_radius + hole_clearance
    safe_outer_limit = max(outer_limit - 0.25 * hole_clearance, cage_hole_radius * 1.2)
    ring_outer = min(pitch_radius + cage_hole_radius + ring_margin, safe_outer_limit)
    ring_outer = max(ring_outer, cage_hole_radius * 1.2)

    ring_inner = max(pitch_radius - cage_hole_radius - ring_margin, cage_hole_radius * 0.6)
    ring_inner = min(ring_inner, ring_outer - 0.25 * hole_clearance)
    ring_inner = max(ring_inner, 0.0)

    return cage_hole_radius, ring_inner, ring_outer


# =================== 三曲柄轴系统 ===================
# 材料参数与常用体积计算（SI 单位）
rho_steel = 7800.0  # kg/m^3

def cylinder_mass(radius_m: float, length_m: float, hollow_inner_radius_m: float = 0.0) -> float:
    """计算圆柱体质量（SI单位：米、千克）"""
    volume = np.pi * (radius_m**2 - hollow_inner_radius_m**2) * length_m
    return rho_steel * volume

def cylinder_inertia(mass: float, radius_m: float, length_m: float, axis: int = 2) -> RigidBodyInertia:
    """计算圆柱体惯性矩（SI单位：千克、米），返回 kg·m²"""
    I_long = 0.5 * mass * radius_m**2
    I_trans = (1.0/12.0) * mass * (3.0 * radius_m**2 + length_m**2)
    if axis == 0:
        inertia_matrix = np.diag([I_long, I_trans, I_trans])
    elif axis == 1:
        inertia_matrix = np.diag([I_trans, I_long, I_trans])
    else:
        inertia_matrix = np.diag([I_trans, I_trans, I_long])
    return RigidBodyInertia(mass=mass, inertiaTensor=inertia_matrix, com=[0, 0, 0])

def solid_disc_inertia(mass: float, radius_m: float, thickness_m: float) -> np.ndarray:
    """计算实心圆盘惯性矩（SI单位：千克、米），返回 kg·m²"""
    Izz = 0.5 * mass * radius_m**2
    Ixy = mass * (0.25 * radius_m**2 + (1.0/12.0) * thickness_m**2)
    return np.array([[Ixy, 0, 0],[0, Ixy, 0],[0, 0, Izz]])

def annulus_inertia(mass: float, r_outer_m: float, r_inner_m: float, thickness_m: float) -> np.ndarray:
    """计算圆环惯性矩（SI单位：千克、米），返回 kg·m²"""
    Izz = 0.5 * mass * (r_outer_m**2 + r_inner_m**2)
    Ixy = mass * (0.25 * (r_outer_m**2 + r_inner_m**2) + (1.0/12.0) * thickness_m**2)
    return np.array([[Ixy, 0, 0],[0, Ixy, 0],[0, 0, Izz]])


def ring_inertia(mass: float, radius_m: float) -> RigidBodyInertia:
    """近似将保持架视为薄环，返回对应的刚体惯量"""
    I_trans = 0.5 * mass * radius_m**2
    I_rot = mass * radius_m**2
    inertia_tensor = np.diag([I_trans, I_trans, I_rot])
    return RigidBodyInertia(mass=mass, inertiaTensor=inertia_tensor, com=[0, 0, 0])

# 曲柄轴参数（近似为实心圆柱）
crank_total_length = L_crank_main1 + L_crank_eccentric1 + L_crank_middle + L_crank_eccentric2 + L_crank_main2
mass_crank = cylinder_mass(r_crank_main, crank_total_length)
I_crank = cylinder_inertia(mass_crank, r_crank_main, crank_total_length, axis=2)

# 中心输入轴（假设实心圆柱）
input_shaft_radius = 10.0e-3  # m
input_shaft_length = 30.0e-3  # m
mass_input_shaft = cylinder_mass(input_shaft_radius, input_shaft_length)
I_input_shaft = cylinder_inertia(mass_input_shaft, input_shaft_radius, input_shaft_length, axis=2)

# 在输入轴上绘制径向标记，便于识别转动
input_shaft_graphics = [
    graphics.Cylinder(pAxis=[0, 0, -15e-3], vAxis=[0, 0, 30e-3], radius=10.0e-3, color=graphics.color.steelblue, nTiles=32),
    graphics.Basis(length=10e-3)
]

for i in range(6):
    angle = i * np.pi / 3
    x_end = 10.0e-3 * np.cos(angle)
    y_end = 10.0e-3 * np.sin(angle)
    input_shaft_graphics.append(
        graphics.Lines(
            [[0.0, 0.0, -15e-3], [x_end, y_end, -15e-3], [x_end, y_end, 15e-3]],
            color=graphics.color.dodgerblue
        )
    )

for i in range(6):
    angle = np.pi / 6 + i * np.pi / 3
    x_end = 10.0e-3 * np.cos(angle)
    y_end = 10.0e-3 * np.sin(angle)
    input_shaft_graphics.append(
        graphics.Lines(
            [[0.0, 0.0, 15e-3], [x_end, y_end, 15e-3], [x_end, y_end, -15e-3]],
            color=[0.6, 0.8, 1.0, 1.0]
        )
    )

# 输入轴位置调整到靠近输入法兰的位置
oInputShaft = mbs.CreateRigidBody(
    referencePosition=[0, 0, 60e-3],
    inertia=I_input_shaft,
    gravity=[0, 0, 0],
    graphicsDataList=input_shaft_graphics,
    nodeType=exu.NodeType.RotationRxyz
)

# 输入轴与地面之间仅保留绕Z轴的转动自由度
mbs.CreateRevoluteJoint(
    bodyNumbers=[oGround, oInputShaft],
    position=[0, 0, 60e-3],
    axis=[0, 0, 1],
    show=False,
    axisRadius=0.6e-3,
    axisLength=4.0e-3
)

omega_input_target = 0  # 目标输入转速（rad/s）
outputTorqueZ = 3200.0   # 目标输出扭矩（N·m）
torqueSegmentTime = 0.01  # 每段持续时间；默认序列 0→+max→0→-max→0→+max→0

TORQUE_SEQUENCE_DEFAULT = [0.0, 1.0, 0.0, -1.0, 0.0]

def torque_multi_step(value, t, segment_time=None, sequence=None):
    seq = TORQUE_SEQUENCE_DEFAULT if sequence is None else sequence
    if not seq:
        return 0.0
    if len(seq) == 1:
        return value * seq[0]
    seg_time = torqueSegmentTime if segment_time is None else segment_time
    if seg_time <= 0:
        return value * seq[-1]
    num_segments = len(seq) - 1
    period = num_segments * seg_time
    if period <= 0:
        return value * seq[-1]
    tau = t % period
    segment_index = int(tau / seg_time)
    if segment_index >= num_segments:
        segment_index = num_segments - 1
    local_time = tau - segment_index * seg_time
    fraction = local_time / seg_time
    start_level = seq[segment_index]
    end_level = seq[segment_index + 1]
    normalized = start_level + (end_level - start_level) * fraction
    return value * normalized

def crank_input_angle_offset(mbs, t, itemIndex, currentOffset):
    ramp_time = 0.2
    if t <= 0.0:
        return 0.0
    elif t < ramp_time:
        omega_t = omega_input_target * (t / ramp_time)
        return 0.5 * omega_input_target / ramp_time * t * t
    else:
        return 0.5 * omega_input_target * ramp_time + omega_input_target * (t - ramp_time)

# 恒定转速作用在输入轴上
mbs.CreateCoordinateConstraint(
    bodyNumbers=[None, oInputShaft],
    coordinates=[None, 5],
    offsetUserFunction=crank_input_angle_offset,
    show=False
)

# 创建三个曲柄轴
crankshaft_bodies = []
crankshaft_markers_ecc1 = []  # 第一片摆线轮对应的偏心段标记
crankshaft_markers_ecc2 = []  # 第二片摆线轮对应的偏心段标记

for i_crank in range(n_cranks):
    angle_crank = ANGLE_OFFSET + 2 * np.pi * i_crank / n_cranks  # 120度均布
    x_crank = crank_distribution_radius * np.cos(angle_crank)
    y_crank = crank_distribution_radius * np.sin(angle_crank)
    
    # 每个曲柄轴的偏心方向（第一片正向，第二片反向）
    crankshaft_graphics = [
        graphics.Cylinder(pAxis=[0, 0, -L_crank_main1], vAxis=[0, 0, L_crank_main1], 
                         radius=r_crank_main, color=graphics.color.darkgrey, nTiles=32),
        graphics.Cylinder(pAxis=[0, eccentric_offset, 0], vAxis=[0, 0, L_crank_eccentric1], 
                         radius=r_crank_eccentric, color=graphics.color.red, nTiles=32),
        graphics.Cylinder(pAxis=[0, 0, L_crank_eccentric1], vAxis=[0, 0, L_crank_middle], 
                         radius=r_crank_main, color=graphics.color.darkgrey, nTiles=32),
        graphics.Cylinder(pAxis=[0, -eccentric_offset, L_crank_eccentric1 + L_crank_middle], 
                         vAxis=[0, 0, L_crank_eccentric2], 
                         radius=r_crank_eccentric, color=graphics.color.orange, nTiles=32),
        graphics.Cylinder(pAxis=[0, 0, L_crank_eccentric1 + L_crank_middle + L_crank_eccentric2], 
                         vAxis=[0, 0, L_crank_main2], 
                         radius=r_crank_main, color=graphics.color.darkgrey, nTiles=32),
    ]
    
    oCrank = mbs.CreateRigidBody(referencePosition=[x_crank, y_crank, 0], 
                                  inertia=I_crank, gravity=[0, 0, 0], 
                                  graphicsDataList=crankshaft_graphics)
    crankshaft_bodies.append(oCrank)
    
    # 曲柄轴采用部分约束的通用关节：释放 XY 平移，只限制 Z 平移与 X/Y 倾转
    mGroundCrank = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[x_crank, y_crank, 0]))
    mCrank = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank, localPosition=[0, 0, 0]))
    mbs.AddObject(GenericJoint(
        markerNumbers=[mGroundCrank, mCrank],
        constrainedAxes=[0, 0, 1, 1, 1, 0],
        rotationMarker0=np.eye(3),
        rotationMarker1=np.eye(3),
        visualization=VObjectJointGeneric(axesRadius=0.5e-3, axesLength=1.0e-3)
    ))
    
    # 用旋转弹簧连接曲柄轴与输入轴，提供弹性约束
    mInputShaftCrank = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oInputShaft, localPosition=[0, 0, 0]))
    mCrankSpring = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank, localPosition=[0, 0, 0]))

    # 齿轮式扭转弹簧的连续角度节点（相对角、marker0角、marker1角）
    nGearSpringData = mbs.AddNode(NodeGenericData(initialCoordinates=[0., 0., 0.], numberOfDataCoordinates=3))
    
    # 添加旋转弹簧-阻尼器，约束绕Z轴旋转
    mbs.AddObject(ObjectConnectorTorsionalSpringDamper(
        markerNumbers=[mInputShaftCrank, mCrankSpring],
        nodeNumber=nGearSpringData,
        stiffness=TORSIONAL_SPRING_STIFFNESS,
        damping=TORSIONAL_SPRING_DAMPING,
        factorMarker0=TORSIONAL_GEAR_RATIO,
        factorMarker1=1.0,
        backlash=TORSIONAL_BACKLASH,
        backlashStiffnessScale=TORSIONAL_BACKLASH_STIFFNESS_SCALE,
        backlashDampingScale=TORSIONAL_BACKLASH_DAMPING_SCALE,
        visualization=VObjectConnectorTorsionalSpringDamper(show=False)
    ))
    
    # 创建偏心段标记
    mEcc1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank, 
                                          localPosition=[0, eccentric_offset, z_eccentric1]))
    mEcc2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank, 
                                          localPosition=[0, -eccentric_offset, z_eccentric2]))
    crankshaft_markers_ecc1.append(mEcc1)
    crankshaft_markers_ecc2.append(mEcc2)


# =================== 输入输出法兰系统 ===================
# 法兰位置
z_input_flange = -L_crank_main1 / 2  # 输入法兰在曲柄轴下方
z_output_flange = L_crank_eccentric1 + L_crank_middle + L_crank_eccentric2 + L_crank_main2 / 2  # 输出法兰在曲柄轴上方

# 法兰参数（近似为圆环）
r_flange_outer = 104e-3  # m
r_flange_inner = 26e-3  # m
thickness_flange = 5.0e-3  # m
flange_volume = np.pi * (r_flange_outer**2 - r_flange_inner**2) * thickness_flange
mass_flange = rho_steel * flange_volume
I_flange = RigidBodyInertia(
    mass=mass_flange,
    inertiaTensor=annulus_inertia(mass_flange, r_flange_outer, r_flange_inner, thickness_flange),
    com=[0, 0, 0]
)

# 输入法兰图形
input_flange_graphics = []
n_flange_circle_points = 60
# 外圆
for i in range(n_flange_circle_points):
    angle1 = ANGLE_OFFSET + i * 2 * np.pi / n_flange_circle_points
    angle2 = ANGLE_OFFSET + (i + 1) * 2 * np.pi / n_flange_circle_points
    x1 = r_flange_outer * np.cos(angle1)
    y1 = r_flange_outer * np.sin(angle1)
    x2 = r_flange_outer * np.cos(angle2)
    y2 = r_flange_outer * np.sin(angle2)
    input_flange_graphics.append({'type': 'Line', 'color': [0.3, 0.3, 0.8, 1], 
                                  'data': [x1, y1, 0, x2, y2, 0]})
# 内圆
for i in range(n_flange_circle_points):
    angle1 = ANGLE_OFFSET + i * 2 * np.pi / n_flange_circle_points
    angle2 = ANGLE_OFFSET + (i + 1) * 2 * np.pi / n_flange_circle_points
    x1 = r_flange_inner * np.cos(angle1)
    y1 = r_flange_inner * np.sin(angle1)
    x2 = r_flange_inner * np.cos(angle2)
    y2 = r_flange_inner * np.sin(angle2)
    input_flange_graphics.append({'type': 'Line', 'color': [0.3, 0.3, 0.8, 1], 
                                  'data': [x1, y1, 0, x2, y2, 0]})
# 3个孔
for i_hole in range(n_cranks):
    angle_hole = ANGLE_OFFSET + 2 * np.pi * i_hole / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    for i in range(40):
        angle1 = ANGLE_OFFSET + i * 2 * np.pi / 40
        angle2 = ANGLE_OFFSET + (i + 1) * 2 * np.pi / 40
        x1 = x_hole_center + r_flange_hole * np.cos(angle1)
        y1 = y_hole_center + r_flange_hole * np.sin(angle1)
        x2 = x_hole_center + r_flange_hole * np.cos(angle2)
        y2 = y_hole_center + r_flange_hole * np.sin(angle2)
        input_flange_graphics.append({'type': 'Line', 'color': [0.8, 0.8, 0.2, 1], 
                                      'data': [x1, y1, 0, x2, y2, 0]})

# 输出法兰图形（类似但颜色不同）
output_flange_graphics = []
# 外圆
for i in range(n_flange_circle_points):
    angle1 = ANGLE_OFFSET + i * 2 * np.pi / n_flange_circle_points
    angle2 = ANGLE_OFFSET + (i + 1) * 2 * np.pi / n_flange_circle_points
    x1 = r_flange_outer * np.cos(angle1)
    y1 = r_flange_outer * np.sin(angle1)
    x2 = r_flange_outer * np.cos(angle2)
    y2 = r_flange_outer * np.sin(angle2)
    output_flange_graphics.append({'type': 'Line', 'color': [0.8, 0.3, 0.3, 1], 
                                   'data': [x1, y1, 0, x2, y2, 0]})
# 内圆
for i in range(n_flange_circle_points):
    angle1 = ANGLE_OFFSET + i * 2 * np.pi / n_flange_circle_points
    angle2 = ANGLE_OFFSET + (i + 1) * 2 * np.pi / n_flange_circle_points
    x1 = r_flange_inner * np.cos(angle1)
    y1 = r_flange_inner * np.sin(angle1)
    x2 = r_flange_inner * np.cos(angle2)
    y2 = r_flange_inner * np.sin(angle2)
    output_flange_graphics.append({'type': 'Line', 'color': [0.8, 0.3, 0.3, 1], 
                                   'data': [x1, y1, 0, x2, y2, 0]})
# 3个孔
for i_hole in range(n_cranks):
    angle_hole = ANGLE_OFFSET + 2 * np.pi * i_hole / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    for i in range(40):
        angle1 = ANGLE_OFFSET + i * 2 * np.pi / 40
        angle2 = ANGLE_OFFSET + (i + 1) * 2 * np.pi / 40
        x1 = x_hole_center + r_flange_hole * np.cos(angle1)
        y1 = y_hole_center + r_flange_hole * np.sin(angle1)
        x2 = x_hole_center + r_flange_hole * np.cos(angle2)
        y2 = y_hole_center + r_flange_hole * np.sin(angle2)
        output_flange_graphics.append({'type': 'Line', 'color': [0.8, 0.8, 0.2, 1], 
                                       'data': [x1, y1, 0, x2, y2, 0]})

# 创建输入法兰（不固定在地面，可自由运动）
oInputFlange = mbs.CreateRigidBody(
    referencePosition=[0, 0, z_input_flange],
    inertia=I_flange,
    gravity=[0, 0, 0],
    graphicsDataList=input_flange_graphics,
    nodeType=exu.NodeType.RotationRxyz
)
# 输入法兰只能绕Z轴旋转（相对地面），但XY平移和Z位置固定
mGroundInputFlange = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, z_input_flange]))
mInputFlange = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oInputFlange, localPosition=[0, 0, 0]))
mbs.AddObject(GenericJoint(markerNumbers=[mGroundInputFlange, mInputFlange],
                           constrainedAxes=[1, 1, 1, 1, 1, 0],  
                           visualization=VObjectJointGeneric(axesRadius=0.5e-3, axesLength=1e-3)))

# 创建输出法兰（不固定在地面）
oOutputFlange = mbs.CreateRigidBody(
    referencePosition=[0, 0, z_output_flange],
    inertia=I_flange,
    gravity=[0, 0, 0],
    graphicsDataList=output_flange_graphics,
    nodeType=exu.NodeType.RotationRxyz
)

# 输出法兰也固定在地面上，只允许绕Z轴旋转
mGroundOutputFlange = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, z_output_flange]))
mOutputFlangeMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oOutputFlange, localPosition=[0, 0, 0]))
mbs.AddObject(GenericJoint(markerNumbers=[mGroundOutputFlange, mOutputFlangeMarker],
                           constrainedAxes=[1, 1, 1, 1, 1, 0],  # 固定 XYZ + RxRy，只允许 Rz
                           visualization=VObjectJointGeneric(axesRadius=0.5e-3, axesLength=1e-3)))

# 输入法兰和输出法兰之间添加扭转弹簧，传递扭矩
mInputFlangeSpring = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oInputFlange, localPosition=[0, 0, 0]))
mOutputFlangeSpring = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oOutputFlange, localPosition=[0, 0, 0]))
nFlangeCouplingData = mbs.AddNode(NodeGenericData(initialCoordinates=[0., 0., 0.], numberOfDataCoordinates=3))
mbs.AddObject(ObjectConnectorTorsionalSpringDamper(
    markerNumbers=[mInputFlangeSpring, mOutputFlangeSpring],
    nodeNumber=nFlangeCouplingData,
    stiffness=1e10,  # 输入输出法兰之间的刚性耦合刚度 (N·m/rad)
    damping=1e6,    # 阻尼 (N·m·s/rad)
    visualization=VObjectConnectorTorsionalSpringDamper(show=False)
))

# 输出法兰恒定扭矩控制
mOutputFlangeLoad = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oOutputFlange, localPosition=[0, 0, 0]))

def output_torque_user_function(mbs, t, loadVector):
    return [0.0, 0.0, torque_multi_step(outputTorqueZ, t)]

mbs.AddLoad(LoadTorqueVector(markerNumber=mOutputFlangeLoad, loadVector=[0, 0, 0],
                             loadVectorUserFunction=output_torque_user_function))

# 创建法兰孔的标记点
input_flange_hole_markers = []
output_flange_hole_markers = []
crank_shaft_input_markers = []  # 曲柄轴在输入法兰处的标记
crank_shaft_output_markers = []  # 曲柄轴在输出法兰处的标记

for i_crank in range(n_cranks):
    angle_hole = ANGLE_OFFSET + 2 * np.pi * i_crank / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    
    # 添加位置误差（每个法兰孔独立误差）
    x_error_in = np.random.normal(0, hole_position_error_std)
    y_error_in = np.random.normal(0, hole_position_error_std)
    x_error_out = np.random.normal(0, hole_position_error_std)
    y_error_out = np.random.normal(0, hole_position_error_std)
    
    # 输入法兰孔标记（带误差）
    mInputHole = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oInputFlange, 
                                localPosition=[x_hole_center + x_error_in, 
                                               y_hole_center + y_error_in, 0]))
    input_flange_hole_markers.append(mInputHole)
    
    # 输出法兰孔标记（带误差）
    mOutputHole = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oOutputFlange, 
                                 localPosition=[x_hole_center + x_error_out, 
                                                y_hole_center + y_error_out, 0]))
    output_flange_hole_markers.append(mOutputHole)
    
    # 曲柄轴在输入法兰处的标记（主轴段）
    mCrankInput = mbs.AddMarker(MarkerBodyRigid(bodyNumber=crankshaft_bodies[i_crank], 
                                                 localPosition=[0, 0, z_input_flange]))
    crank_shaft_input_markers.append(mCrankInput)
    
    # 曲柄轴在输出法兰处的标记（主轴段）
    mCrankOutput = mbs.AddMarker(MarkerBodyRigid(bodyNumber=crankshaft_bodies[i_crank], 
                                                  localPosition=[0, 0, z_output_flange]))
    crank_shaft_output_markers.append(mCrankOutput)


# =================== 摆线轮（带3个均布孔） ===================
thickness_cycloid = 20.0e-3  # m
mass_cycloid = rho_steel * np.pi * R_cycloid**2 * thickness_cycloid  # kg
I_cycloid = RigidBodyInertia(mass=mass_cycloid, inertiaTensor=solid_disc_inertia(mass_cycloid, R_cycloid, thickness_cycloid), com=[0,0,0])

# 第一片摆线轮
cycloid1_graphics = []

# 外齿廓 - 沿z轴拉伸
tooth_extrusion_height = 10.0e-3  # 拉伸高度 (m)
z_offset = -tooth_extrusion_height / 2  # 居中
for seg in segmentsData_tooth1:
    x0, y0, x1, y1 = seg[0], seg[1], seg[2], seg[3]
    # 创建矩形面片：底部两点 + 顶部两点
    points = [x0, y0, z_offset,
              x1, y1, z_offset,
              x1, y1, z_offset + tooth_extrusion_height,
              x0, y0, z_offset + tooth_extrusion_height]
    triangles = [0, 1, 2,  0, 2, 3]  # 两个三角形组成矩形
    colors = [0.9, 0.1, 0.1, 1] * 4
    cycloid1_graphics.append({
        'type': 'TriangleList',
        'colors': colors,
        'points': points,
        'triangles': triangles
    })

# 3个均布的孔
n_hole_points = 40
for i_hole in range(n_cranks):
    angle_hole = ANGLE_OFFSET + 2 * np.pi * i_hole / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    for i in range(n_hole_points):
        angle1 = ANGLE_OFFSET + i * 2 * np.pi / n_hole_points
        angle2 = ANGLE_OFFSET + (i + 1) * 2 * np.pi / n_hole_points
        x1 = x_hole_center + r_hole * np.cos(angle1)
        y1 = y_hole_center + r_hole * np.sin(angle1)
        x2 = x_hole_center + r_hole * np.cos(angle2)
        y2 = y_hole_center + r_hole * np.sin(angle2)
        cycloid1_graphics.append({'type': 'Line', 'color': [0.1, 0.7, 0.1, 1], 'data': [x1, y1, 0, x2, y2, 0]})

# 第一片摆线轮初始位置：与第一个曲柄轴的偏心段对齐
oCycloid1 = mbs.CreateRigidBody(referencePosition=[0, eccentric_offset, z_eccentric1], inertia=I_cycloid, gravity=[0,0,0], graphicsDataList=cycloid1_graphics)
mCycloid1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid1, localPosition=[0,0,0]))
mGroundCycloid1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, eccentric_offset, z_eccentric1]))
mbs.AddObject(GenericJoint(markerNumbers=[mGroundCycloid1, mCycloid1], constrainedAxes=[0,0,1,1,1,0], visualization=VObjectJointGeneric(axesRadius=0.5e-3, axesLength=1e-3)))

# 第二片摆线轮
cycloid2_graphics = []

# 外齿廓 - 沿z轴拉伸
for seg in segmentsData_tooth2:
    x0, y0, x1, y1 = seg[0], seg[1], seg[2], seg[3]
    points = [x0, y0, z_offset,
              x1, y1, z_offset,
              x1, y1, z_offset + tooth_extrusion_height,
              x0, y0, z_offset + tooth_extrusion_height]
    triangles = [0, 1, 2,  0, 2, 3]
    colors = [0.6, 0.1, 0.9, 1] * 4
    cycloid2_graphics.append({
        'type': 'TriangleList',
        'colors': colors,
        'points': points,
        'triangles': triangles
    })

# 3个均布的孔
for i_hole in range(n_cranks):
    angle_hole = ANGLE_OFFSET + 2 * np.pi * i_hole / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    for i in range(n_hole_points):
        angle1 = ANGLE_OFFSET + i * 2 * np.pi / n_hole_points
        angle2 = ANGLE_OFFSET + (i + 1) * 2 * np.pi / n_hole_points
        x1 = x_hole_center + r_hole * np.cos(angle1)
        y1 = y_hole_center + r_hole * np.sin(angle1)
        x2 = x_hole_center + r_hole * np.cos(angle2)
        y2 = y_hole_center + r_hole * np.sin(angle2)
        cycloid2_graphics.append({'type': 'Line', 'color': [0.1, 0.7, 0.9, 1], 'data': [x1, y1, 0, x2, y2, 0]})

# 第二片摆线轮初始位置：与第二个曲柄轴的偏心段对齐
oCycloid2 = mbs.CreateRigidBody(referencePosition=[0, -eccentric_offset, z_eccentric2], inertia=I_cycloid, gravity=[0,0,0], graphicsDataList=cycloid2_graphics)
mCycloid2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid2, localPosition=[0,0,0]))
mGroundCycloid2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, -eccentric_offset, z_eccentric2]))
mbs.AddObject(GenericJoint(markerNumbers=[mGroundCycloid2, mCycloid2], constrainedAxes=[0,0,1,1,1,0], visualization=VObjectJointGeneric(axesRadius=0.5e-3, axesLength=1e-3)))

# 创建摆线轮孔的标记（用于轴承连接）
cycloid1_hole_markers = []
cycloid2_hole_markers = []
for i_hole in range(n_cranks):
    angle_hole = ANGLE_OFFSET + 2 * np.pi * i_hole / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    
    # 添加位置误差（每个摆线轮孔独立误差）
    x_error_1 = np.random.normal(0, hole_position_error_std)
    y_error_1 = np.random.normal(0, hole_position_error_std)
    x_error_2 = np.random.normal(0, hole_position_error_std)
    y_error_2 = np.random.normal(0, hole_position_error_std)
    
    mHole1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid1, 
                           localPosition=[x_hole_center + x_error_1, 
                                          y_hole_center + y_error_1, 0]))
    mHole2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid2, 
                           localPosition=[x_hole_center + x_error_2, 
                                          y_hole_center + y_error_2, 0]))
    cycloid1_hole_markers.append(mHole1)
    cycloid2_hole_markers.append(mHole2)

# 注意：输出法兰不与摆线轮直接连接，仅通过法兰轴承与曲柄轴连接


# =================== 针齿壳刚体 ===================
z_shell = (z_eccentric1 + z_eccentric2) / 2
shell_graphics_list = []

# 绘制内齿圈齿廓 - 沿z轴拉伸
inner_tooth_extrusion_height = 30.0e-3  # 内齿圈拉伸高度
inner_z_offset = -inner_tooth_extrusion_height / 2
for seg in inner_segments_master:
    x0, y0, x1, y1 = seg[0], seg[1], seg[2], seg[3]
    points = [x0, y0, inner_z_offset,
              x1, y1, inner_z_offset,
              x1, y1, inner_z_offset + inner_tooth_extrusion_height,
              x0, y0, inner_z_offset + inner_tooth_extrusion_height]
    triangles = [0, 1, 2,  0, 2, 3]
    colors = [0.3, 0.7, 0.3, 1] * 4  # 绿色表示内齿圈
    shell_graphics_list.append({
        'type': 'TriangleList',
        'colors': colors,
        'points': points,
        'triangles': triangles
    })

# 可选：绘制壳体外轮廓（仅用于参考）
n_shell_outer_points = 120
for i in range(n_shell_outer_points):
    angle1 = ANGLE_OFFSET + i * 2 * np.pi / n_shell_outer_points
    angle2 = ANGLE_OFFSET + (i + 1) * 2 * np.pi / n_shell_outer_points
    x1 = R_shell_outer * np.cos(angle1); y1 = R_shell_outer * np.sin(angle1)
    x2 = R_shell_outer * np.cos(angle2); y2 = R_shell_outer * np.sin(angle2)
    shell_graphics_list.append({'type':'Line','color':[0.4,0.4,0.4,0.5],'data':[x1,y1,0,x2,y2,0]})

# 创建针齿壳刚体（而不是放在Ground上）
# 计算针齿壳质量和惯性（近似为空心圆盘）
mass_shell = rho_steel * np.pi * (R_shell_outer**2 - R_shell_inner**2) * thickness_shell
I_shell = RigidBodyInertia(
    mass=mass_shell,
    inertiaTensor=annulus_inertia(mass_shell, R_shell_outer, R_shell_inner, thickness_shell),
    com=[0, 0, 0]
)

oShell = mbs.CreateRigidBody(
    referencePosition=[0, 0, z_shell],
    inertia=I_shell,
    gravity=[0, 0, 0],
    graphicsDataList=shell_graphics_list,
    nodeType=exu.NodeType.RotationRxyz
)

# 将针齿壳完全固定在地面上（6个自由度全部约束）
mGroundShell = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, z_shell]))
mShellBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oShell, localPosition=[0, 0, 0]))
mbs.AddObject(GenericJoint(
    markerNumbers=[mGroundShell, mShellBody],
    constrainedAxes=[1, 1, 1, 1, 1, 1],  # 完全固定
    visualization=VObjectJointGeneric(axesRadius=0.5e-3, axesLength=1.0e-3, show=False)
))

# 针销部分已删除（内齿圈结构不需要针销）

# =================== 改进的多齿面接触系统 ===================
# 为内齿圈的每个齿单独创建接触对象，实现真正的多齿同时接触力计算
# 每个内齿圈齿面只与外齿轮所有齿面建立一个接触对象

print("=" * 60)
print("建立每齿独立接触系统：")
print(f"  外齿轮齿数: {gear_profile_params.z_outer}")
print(f"  内齿圈齿数: {gear_profile_params.z_inner}")
print(f"  将为内齿圈每个齿的每个齿面创建独立接触对象...")
print("=" * 60)

# 提取每个齿的齿面数据
def get_tooth_flank(flanks_dict, gear_id, tooth_index, flank_side):
    """获取指定齿的指定齿面点集"""
    key = (gear_id, tooth_index, flank_side)
    if key in flanks_dict:
        return np.asarray(flanks_dict[key], dtype=float)
    return np.zeros((0, 2), dtype=float)

# 第一片摆线轮的接触对象
contact_count = 0
for tooth_idx in range(gear_profile_params.z_inner):
    for flank_side in ["left", "right"]:
        # 获取内齿圈当前齿的当前齿面
        inner_flank = get_tooth_flank(gear_flanks, "inner", tooth_idx, flank_side)
        
        if inner_flank.shape[0] == 0:
            continue
        
        if flank_side == "left":
            color = [0.0, 0.0, 0.8, 0.7]  # 深蓝色
        else:
            color = [0.0, 0.6, 1.0, 0.7]  # 浅蓝色

        node = mbs.AddNode(NodeGenericData(initialCoordinates=[0.0] * 9, numberOfDataCoordinates=9))  # 增加2个用于Bristle粘滞位置
        outer_flank = "right" if flank_side == "left" else "left"
        obj = mbs.AddObject(ObjectContactFewTeeth(
            name=f"Gear1_T{tooth_idx:02d}_{flank_side[0].upper()}",
            markerNumbers=[mCycloid1, mShellBody],
            nodeNumber=node,
            contactDamping=contactDamping_tooth,
            frictionCoefficient=friction_cycloid_pin,
            frictionVelocityPenalty=1000.0,        # 速度惩罚摩擦模型，更稳定
            frictionProportionalZone=0.00,         # 平滑零速度附近的摩擦力
            frictionStiffness=frictionStiffness_cycloid_pin,  # Bristle静摩擦刚度
            stiffnessOuter=stiffnessOuter_tooth,  # 外齿轮结构刚度表（不含接触）
            stiffnessInner=stiffnessInner_tooth,  # 内齿轮结构刚度表（不含接触）
            faceWidth=face_width_tooth,           # 齿宽（用于 C++ 计算接触刚度）
            elasticModulus=elasticModulus_tooth,  # 弹性模量（用于 C++ 计算接触刚度）
            poissonRatio=poissonRatio_tooth,      # 泊松比（用于 C++ 计算接触刚度）
            tipProbeLength=gear_profile_params.tip_probe_length,
            outerAnalytic=build_outer_analytic(outer_flank, rotation_offset=0.0),
            innerAnalytic=build_inner_analytic(tooth_idx, flank_side),
            visualization=VObjectContactFewTeeth(show=SHOW_CONTACT_GEOMETRY, color=color),
        ))
        # 使用刚度表的平均值用于统计
        avg_stiffness = sum(stiffnessOuter_tooth) / len(stiffnessOuter_tooth)
        _register_contact_info("FewTeethGears", f"Gear1_T{tooth_idx:02d}_{flank_side}",
                              node, obj, ComputeSegmentLengths(inner_flank),
                              avg_stiffness, contactDamping_tooth)
        contact_count += 1

print(f"✓ 第1片摆线轮: 创建了 {contact_count} 个接触对象")

# 第二片摆线轮的接触对象
contact_count2 = 0
for tooth_idx in range(gear_profile_params.z_inner):
    for flank_side in ["left", "right"]:
        # 获取内齿圈当前齿的当前齿面
        inner_flank = get_tooth_flank(gear_flanks, "inner", tooth_idx, flank_side)
        
        if inner_flank.shape[0] == 0:
            continue
        
        if flank_side == "left":
            color = [1.0, 0.0, 0.8, 0.7]  # 洋红色
        else:
            color = [1.0, 0.5, 1.0, 0.7]  # 浅洋红色

        node = mbs.AddNode(NodeGenericData(initialCoordinates=[0.0] * 9, numberOfDataCoordinates=9))  # 增加2个用于Bristle粘滞位置
        outer_flank = "right" if flank_side == "left" else "left"
        obj = mbs.AddObject(ObjectContactFewTeeth(
            name=f"Gear2_T{tooth_idx:02d}_{flank_side[0].upper()}",
            markerNumbers=[mCycloid2, mShellBody],
            nodeNumber=node,
            contactDamping=contactDamping_tooth,
            frictionCoefficient=friction_cycloid_pin,
            frictionVelocityPenalty=1000.0,        # 速度惩罚摩擦模型，更稳定
            frictionProportionalZone=0.00,         # 平滑零速度附近的摩擦力
            frictionStiffness=frictionStiffness_cycloid_pin,  # Bristle静摩擦刚度
            stiffnessOuter=stiffnessOuter_tooth,  # 外齿轮结构刚度表（不含接触）
            stiffnessInner=stiffnessInner_tooth,  # 内齿轮结构刚度表（不含接触）
            faceWidth=face_width_tooth,           # 齿宽（用于 C++ 计算接触刚度）
            elasticModulus=elasticModulus_tooth,  # 弹性模量（用于 C++ 计算接触刚度）
            poissonRatio=poissonRatio_tooth,      # 泊松比（用于 C++ 计算接触刚度）
            tipProbeLength=gear_profile_params.tip_probe_length,
            outerAnalytic=build_outer_analytic(outer_flank, rotation_offset=math.pi),
            innerAnalytic=build_inner_analytic(tooth_idx, flank_side),
            visualization=VObjectContactFewTeeth(show=SHOW_CONTACT_GEOMETRY, color=color),
        ))
        # 使用刚度表的平均值用于统计
        avg_stiffness = sum(stiffnessOuter_tooth) / len(stiffnessOuter_tooth)
        _register_contact_info("FewTeethGears", f"Gear2_T{tooth_idx:02d}_{flank_side}",
                              node, obj, ComputeSegmentLengths(inner_flank),
                              avg_stiffness, contactDamping_tooth)
        contact_count2 += 1

print(f"✓ 第2片摆线轮: 创建了 {contact_count2} 个接触对象")
print(f"✓ 总计: {contact_count + contact_count2} 个独立齿面接触对象")
print("=" * 60)

# 滚针轴承参数
contactStiffness_bearing = 1e9  # N/m (SI单位) #轴承刚度
contactDamping_bearing = 1e5  # N/(m/s)    #轴承阻尼
needle_length = 30.0e-3  # m
mass_needle = cylinder_mass(r_needle, needle_length)
inertia_needle = cylinder_inertia(mass_needle, r_needle, needle_length, axis=2)

# 法兰轴承参数
contactStiffness_flange_bearing = 1e9  # N/m   #轴承刚度
contactDamping_flange_bearing = 1e5  # N/(m/s) #轴承阻尼
flange_needle_length = 40.0e-3  # m
mass_flange_needle = cylinder_mass(r_flange_needle, flange_needle_length)
inertia_flange_needle = cylinder_inertia(mass_flange_needle, r_flange_needle, flange_needle_length, axis=2)

# 存储滚针位置标记（按曲柄分组）
needle_markers_cyc1 = [[] for _ in range(n_cranks)]
needle_markers_cyc2 = [[] for _ in range(n_cranks)]
needle_markers_in = [[] for _ in range(n_cranks)]
needle_markers_out = [[] for _ in range(n_cranks)]
cage_markers_cyc1 = [[] for _ in range(n_cranks)]
cage_markers_cyc2 = [[] for _ in range(n_cranks)]
cage_markers_in = [[] for _ in range(n_cranks)]
cage_markers_out = [[] for _ in range(n_cranks)]
cage_bodies_cyc1 = []
cage_bodies_cyc2 = []
cage_bodies_in = []
cage_bodies_out = []

# 预计算保持架参考位置与惯量（直接使用已知初始位置）
ref_cycloid1 = np.array([0.0, eccentric_offset, z_eccentric1], dtype=float)
ref_cycloid2 = np.array([0.0, -eccentric_offset, z_eccentric2], dtype=float)
ref_input_flange = np.array([0.0, 0.0, z_input_flange], dtype=float)
ref_output_flange = np.array([0.0, 0.0, z_output_flange], dtype=float)

inertia_cage_cycloid = ring_inertia(cage_mass_cycloid, r_needle_pitch)
inertia_cage_flange = ring_inertia(cage_mass_flange, r_flange_needle_pitch)

# 使用 ObjectContactCircleCircle 替换圆-圆接触
for i_crank in range(n_cranks):
    angle_hole = ANGLE_OFFSET + 2 * np.pi * i_crank / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    hole_local = np.array([x_hole_center, y_hole_center, 0.0])
    cage_hole_radius_cyc, cage_ring_inner_cyc, cage_ring_outer_cyc = compute_cage_radii(
        r_needle_pitch, r_needle, r_hole)

    cage_graphics_cycloid = create_cage_ring_graphics(
        center=[0, 0, 0],
        pitch_radius=r_needle_pitch,
        hole_radius=cage_hole_radius_cyc,
        n_holes=n_needles,
        ring_inner_radius=cage_ring_inner_cyc,
        ring_outer_radius=cage_ring_outer_cyc,
        color_ring=cage_color_cycloid,
        color_hole=cage_color_cycloid,
        angle_offset=ANGLE_OFFSET
    )

    cage_ref_pos1 = ref_cycloid1 + hole_local
    oCage1Body = mbs.CreateRigidBody(
        referencePosition=cage_ref_pos1.tolist(),
        inertia=inertia_cage_cycloid,
    gravity=[0, 0, 0],
        graphicsDataList=cage_graphics_cycloid,
    nodeType=exu.NodeType.RotationRxyz
)
    cage_bodies_cyc1.append(oCage1Body)
    mCage1Center = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCage1Body, localPosition=[0, 0, 0]))
    mHole1Center = cycloid1_hole_markers[i_crank]
    mbs.AddObject(GenericJoint(
        markerNumbers=[mHole1Center, mCage1Center],
        constrainedAxes=[0, 0, 1, 1, 1, 0],
        visualization=VObjectJointGeneric(axesRadius=0.12e-3, axesLength=0.3e-3, show=False)
    ))

    cage_ref_pos2 = ref_cycloid2 + hole_local
    oCage2Body = mbs.CreateRigidBody(
        referencePosition=cage_ref_pos2.tolist(),
        inertia=inertia_cage_cycloid,
        gravity=[0, 0, 0],
        graphicsDataList=cage_graphics_cycloid,
        nodeType=exu.NodeType.RotationRxyz
    )
    cage_bodies_cyc2.append(oCage2Body)
    mCage2Center = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCage2Body, localPosition=[0, 0, 0]))
    mHole2Center = cycloid2_hole_markers[i_crank]
    mbs.AddObject(GenericJoint(
        markerNumbers=[mHole2Center, mCage2Center],
        constrainedAxes=[0, 0, 1, 1, 1, 0],
        visualization=VObjectJointGeneric(axesRadius=0.12e-3, axesLength=0.3e-3, show=False)
    ))

    for i_needle in range(n_needles):
        angle_needle = ANGLE_OFFSET + 2 * np.pi * i_needle / n_needles

        # 先创建保持架孔标记
        cage_local_pos1 = [r_needle_pitch * np.cos(angle_needle),
                           r_needle_pitch * np.sin(angle_needle),
                           0.0]
        mCycloid1CageHole = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCage1Body,
                                                         localPosition=cage_local_pos1))
        cage_markers_cyc1[i_crank].append(mCycloid1CageHole)

        # 创建滚针刚体
        x_needle_1 = x_hole_center + r_needle_pitch * np.cos(angle_needle)
        y_needle_1 = y_hole_center + eccentric_offset + r_needle_pitch * np.sin(angle_needle)
        oNeedle1 = mbs.CreateRigidBody(
            referencePosition=[x_needle_1, y_needle_1, z_eccentric1],
            inertia=inertia_needle,
            gravity=[0, 0, 0],
            graphicsDataList=[graphics.Cylinder(
                pAxis=[0, 0, -2e-3], vAxis=[0, 0, 4e-3],
                radius=r_needle, color=graphics.color.darkgrey, nTiles=16)]
        )
        mNeedle1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle1, localPosition=[0, 0, 0]))
        
        # 滚针约束到保持架孔（而不是地面）
        mbs.AddObject(GenericJoint(markerNumbers=[mCycloid1CageHole, mNeedle1],
                                   constrainedAxes=[1, 1, 1, 1, 1, 0],
                                   visualization=VObjectJointGeneric(axesRadius=0.1e-3, axesLength=0.2e-3, show=False)))

        create_circle_contact(
            mbs,
            name=f"CrankNeedle-{i_crank:02d}-{i_needle:02d}",
            markerA=crankshaft_markers_ecc1[i_crank],
            markerB=mNeedle1,
            radiusA=r_crank_eccentric,
            radiusB=r_needle,
            stiffness=contactStiffness_bearing,
            damping=contactDamping_bearing,
            friction=friction_crank_needle,
            group="CrankNeedle"
        )
        create_circle_contact(
            mbs,
            name=f"NeedleCycloidHole-{i_crank:02d}-{i_needle:02d}",
            markerA=mNeedle1,
            markerB=cycloid1_hole_markers[i_crank],
            radiusA=r_needle,
            radiusB=-r_hole,
            stiffness=contactStiffness_bearing,
            damping=contactDamping_bearing,
            friction=friction_needle_cycloid_hole,
            group="NeedleCycloidHole"
        )
        # 注释掉滚针与保持架孔的接触（已通过关节约束）
        # create_circle_contact(
        #     mbs,
        #     name=f"CycloidCage-{i_crank:02d}-{i_needle:02d}",
        #     markerA=mNeedle1,
        #     markerB=mCycloid1CageHole,
        #     radiusA=r_needle,
        #     radiusB=-cage_hole_radius_cyc,
        #     stiffness=contactStiffness_cage,
        #     damping=contactDamping_cage,
        #     friction=0.0,
        #     group="CycloidNeedleCage"
        # )

        # 记录第一层摆线滚针位置
        mPosNeedle1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oNeedle1, localPosition=[0, 0, 0]))
        needle_markers_cyc1[i_crank].append(mPosNeedle1)

        # 先创建保持架孔标记
        cage_local_pos2 = [r_needle_pitch * np.cos(angle_needle),
                           r_needle_pitch * np.sin(angle_needle),
                           0.0]
        mCycloid2CageHole = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCage2Body,
                                                         localPosition=cage_local_pos2))
        cage_markers_cyc2[i_crank].append(mCycloid2CageHole)

        # 创建滚针刚体
        x_needle_2 = x_hole_center + r_needle_pitch * np.cos(angle_needle)
        y_needle_2 = y_hole_center - eccentric_offset + r_needle_pitch * np.sin(angle_needle)
        oNeedle2 = mbs.CreateRigidBody(
            referencePosition=[x_needle_2, y_needle_2, z_eccentric2],
            inertia=inertia_needle,
            gravity=[0, 0, 0],
            graphicsDataList=[graphics.Cylinder(
                pAxis=[0, 0, -2e-3], vAxis=[0, 0, 4e-3],
                radius=r_needle, color=graphics.color.darkgrey, nTiles=16)]
        )
        mNeedle2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle2, localPosition=[0, 0, 0]))
        
        # 滚针约束到保持架孔（而不是地面）
        mbs.AddObject(GenericJoint(markerNumbers=[mCycloid2CageHole, mNeedle2],
                                   constrainedAxes=[1, 1, 1, 1, 1, 0],
                                   visualization=VObjectJointGeneric(axesRadius=0.1e-3, axesLength=0.2e-3, show=False)))

        create_circle_contact(
            mbs,
            name=f"CrankNeedleB-{i_crank:02d}-{i_needle:02d}",
            markerA=crankshaft_markers_ecc2[i_crank],
            markerB=mNeedle2,
            radiusA=r_crank_eccentric,
            radiusB=r_needle,
            stiffness=contactStiffness_bearing,
            damping=contactDamping_bearing,
            friction=friction_crank_needle,
            group="CrankNeedle"
        )
        create_circle_contact(
            mbs,
            name=f"NeedleCycloidHoleB-{i_crank:02d}-{i_needle:02d}",
            markerA=mNeedle2,
            markerB=cycloid2_hole_markers[i_crank],
            radiusA=r_needle,
            radiusB=-r_hole,
            stiffness=contactStiffness_bearing,
            damping=contactDamping_bearing,
            friction=friction_needle_cycloid_hole,
            group="NeedleCycloidHole"
        )
        # 注释掉滚针与保持架孔的接触（已通过关节约束）
        # create_circle_contact(
        #     mbs,
        #     name=f"CycloidCageB-{i_crank:02d}-{i_needle:02d}",
        #     markerA=mNeedle2,
        #     markerB=mCycloid2CageHole,
        #     radiusA=r_needle,
        #     radiusB=-cage_hole_radius_cyc,
        #     stiffness=contactStiffness_cage,
        #     damping=contactDamping_cage,
        #     friction=0.0,
        #     group="CycloidNeedleCage"
        # )

        # 记录第二层摆线滚针位置
        mPosNeedle2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oNeedle2, localPosition=[0, 0, 0]))
        needle_markers_cyc2[i_crank].append(mPosNeedle2)

for i_crank in range(n_cranks):
    angle_hole = ANGLE_OFFSET + 2 * np.pi * i_crank / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    hole_local = np.array([x_hole_center, y_hole_center, 0.0])
    flange_cage_hole_radius, flange_ring_inner, flange_ring_outer = compute_cage_radii(
        r_flange_needle_pitch, r_flange_needle, r_flange_hole)

    cage_graphics_flange = create_cage_ring_graphics(
        center=[0, 0, 0],
        pitch_radius=r_flange_needle_pitch,
        hole_radius=flange_cage_hole_radius,
        n_holes=n_flange_needles,
        ring_inner_radius=flange_ring_inner,
        ring_outer_radius=flange_ring_outer,
        color_ring=cage_color_flange,
        color_hole=cage_color_flange,
        angle_offset=ANGLE_OFFSET
    )

    cage_ref_in = ref_input_flange + hole_local
    oCageInBody = mbs.CreateRigidBody(
        referencePosition=cage_ref_in.tolist(),
        inertia=inertia_cage_flange,
        gravity=[0, 0, 0],
        graphicsDataList=cage_graphics_flange,
        nodeType=exu.NodeType.RotationRxyz
    )
    cage_bodies_in.append(oCageInBody)
    mCageInCenter = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCageInBody, localPosition=[0, 0, 0]))
    mInputHole = input_flange_hole_markers[i_crank]
    mbs.AddObject(GenericJoint(
        markerNumbers=[mInputHole, mCageInCenter],
        constrainedAxes=[0, 0, 1, 1, 1, 0],
        visualization=VObjectJointGeneric(axesRadius=0.12e-3, axesLength=0.3e-3, show=False)
    ))

    cage_ref_out = ref_output_flange + hole_local
    oCageOutBody = mbs.CreateRigidBody(
        referencePosition=cage_ref_out.tolist(),
        inertia=inertia_cage_flange,
        gravity=[0, 0, 0],
        graphicsDataList=cage_graphics_flange,
        nodeType=exu.NodeType.RotationRxyz
    )
    cage_bodies_out.append(oCageOutBody)
    mCageOutCenter = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCageOutBody, localPosition=[0, 0, 0]))
    mOutputHole = output_flange_hole_markers[i_crank]
    mbs.AddObject(GenericJoint(
        markerNumbers=[mOutputHole, mCageOutCenter],
        constrainedAxes=[0, 0, 1, 1, 1, 0],
        visualization=VObjectJointGeneric(axesRadius=0.12e-3, axesLength=0.3e-3, show=False)
    ))

    for i_needle in range(n_flange_needles):
        angle_needle = ANGLE_OFFSET + 2 * np.pi * i_needle / n_flange_needles
        
        # 先创建保持架孔标记
        cage_local_pos_in = [r_flange_needle_pitch * np.cos(angle_needle),
                              r_flange_needle_pitch * np.sin(angle_needle),
                              0.0]
        mInputCageHole = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCageInBody,
                                                       localPosition=cage_local_pos_in))
        cage_markers_in[i_crank].append(mInputCageHole)

        # 创建滚针刚体
        x_needle = x_hole_center + r_flange_needle_pitch * np.cos(angle_needle)
        y_needle = y_hole_center + r_flange_needle_pitch * np.sin(angle_needle)
        oNeedleIn = mbs.CreateRigidBody(
            referencePosition=[x_needle, y_needle, z_input_flange],
            inertia=inertia_flange_needle,
            gravity=[0, 0, 0],
            graphicsDataList=[graphics.Cylinder(
                pAxis=[0, 0, -1.5e-3], vAxis=[0, 0, 3e-3],
                radius=r_flange_needle, color=graphics.color.grey, nTiles=16)]
        )
        mNeedleIn = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedleIn, localPosition=[0, 0, 0]))
        
        # 滚针约束到保持架孔
        mbs.AddObject(GenericJoint(markerNumbers=[mInputCageHole, mNeedleIn],
                                   constrainedAxes=[1, 1, 1, 1, 1, 0],
                                   visualization=VObjectJointGeneric(axesRadius=0.1e-3, axesLength=0.2e-3, show=False)))

        create_circle_contact(
            mbs,
            name=f"InputFlangeShaft-{i_crank:02d}-{i_needle:02d}",
            markerA=crank_shaft_input_markers[i_crank],
            markerB=mNeedleIn,
            radiusA=r_flange_shaft,
            radiusB=r_flange_needle,
            stiffness=contactStiffness_flange_bearing,
            damping=contactDamping_flange_bearing,
            friction=friction_flange_bearing,
            group="FlangeShaft"
        )
        create_circle_contact(
            mbs,
            name=f"InputFlangeHole-{i_crank:02d}-{i_needle:02d}",
            markerA=mNeedleIn,
            markerB=input_flange_hole_markers[i_crank],
            radiusA=r_flange_needle,
            radiusB=-r_flange_hole,
            stiffness=contactStiffness_flange_bearing,
            damping=contactDamping_flange_bearing,
            friction=friction_flange_bearing,
            group="FlangeHole"
        )
        # 注释掉滚针与保持架孔的接触（已通过关节约束）
        # create_circle_contact(
        #     mbs,
        #     name=f"InputFlangeCage-{i_crank:02d}-{i_needle:02d}",
        #     markerA=mNeedleIn,
        #     markerB=mInputCageHole,
        #     radiusA=r_flange_needle,
        #     radiusB=-flange_cage_hole_radius,
        #     stiffness=contactStiffness_cage,
        #     damping=contactDamping_cage,
        #     friction=0.0,
        #     group="FlangeNeedleCage"
        # )

        # 记录输入法兰滚针位置
        mPosNeedleIn = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oNeedleIn, localPosition=[0, 0, 0]))
        needle_markers_in[i_crank].append(mPosNeedleIn)

        # 先创建保持架孔标记
        cage_local_pos_out = [r_flange_needle_pitch * np.cos(angle_needle),
                               r_flange_needle_pitch * np.sin(angle_needle),
                               0.0]
        mOutputCageHole = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCageOutBody,
                                                        localPosition=cage_local_pos_out))
        cage_markers_out[i_crank].append(mOutputCageHole)

        # 创建滚针刚体
        oNeedleOut = mbs.CreateRigidBody(
            referencePosition=[x_needle, y_needle, z_output_flange],
            inertia=inertia_flange_needle,
            gravity=[0, 0, 0],
            graphicsDataList=[graphics.Cylinder(
                pAxis=[0, 0, -1.5e-3], vAxis=[0, 0, 3e-3],
                radius=r_flange_needle, color=graphics.color.grey, nTiles=16)]
        )
        mNeedleOut = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedleOut, localPosition=[0, 0, 0]))
        
        # 滚针约束到保持架孔（而不是地面）
        mbs.AddObject(GenericJoint(markerNumbers=[mOutputCageHole, mNeedleOut],
                                   constrainedAxes=[1, 1, 1, 1, 1, 0],
                                   visualization=VObjectJointGeneric(axesRadius=0.1e-3, axesLength=0.2e-3, show=False)))

        create_circle_contact(
            mbs,
            name=f"OutputFlangeShaft-{i_crank:02d}-{i_needle:02d}",
            markerA=crank_shaft_output_markers[i_crank],
            markerB=mNeedleOut,
            radiusA=r_flange_shaft,
            radiusB=r_flange_needle,
            stiffness=contactStiffness_flange_bearing,
            damping=contactDamping_flange_bearing,
            friction=friction_flange_bearing,
            group="FlangeShaft"
        )
        create_circle_contact(
            mbs,
            name=f"OutputFlangeHole-{i_crank:02d}-{i_needle:02d}",
            markerA=mNeedleOut,
            markerB=output_flange_hole_markers[i_crank],
            radiusA=r_flange_needle,
            radiusB=-r_flange_hole,
            stiffness=contactStiffness_flange_bearing,
            damping=contactDamping_flange_bearing,
            friction=friction_flange_bearing,
            group="FlangeHole"
        )
        # 注释掉滚针与保持架孔的接触（已通过关节约束）
        # create_circle_contact(
        #     mbs,
        #     name=f"OutputFlangeCage-{i_crank:02d}-{i_needle:02d}",
        #     markerA=mNeedleOut,
        #     markerB=mOutputCageHole,
        #     radiusA=r_flange_needle,
        #     radiusB=-flange_cage_hole_radius,
        #     stiffness=contactStiffness_cage,
        #     damping=contactDamping_cage,
        #     friction=0.0,
        #     group="FlangeNeedleCage"
        # )

        # 记录输出法兰滚针位置
        mPosNeedleOut = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oNeedleOut, localPosition=[0, 0, 0]))
        needle_markers_out[i_crank].append(mPosNeedleOut)

# =================== 传感器/组装/仿真 ===================
mbs.CreateGround(graphicsDataList=[])

# 传感器（需在仿真前创建并 storeInternal 以记录数据）
sCycloid1Pos = mbs.AddSensor(SensorBody(bodyNumber=oCycloid1, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
sCycloid2Pos = mbs.AddSensor(SensorBody(bodyNumber=oCycloid2, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
sCrankRot    = mbs.AddSensor(SensorBody(bodyNumber=crankshaft_bodies[0], storeInternal=True, outputVariableType=exu.OutputVariableType.RotationMatrix))
sInputShaftRot = mbs.AddSensor(SensorBody(bodyNumber=oInputShaft, storeInternal=True, outputVariableType=exu.OutputVariableType.RotationMatrix))
sOutputFlangeRot = mbs.AddSensor(SensorBody(bodyNumber=oOutputFlange, storeInternal=True, outputVariableType=exu.OutputVariableType.RotationMatrix))
contact_sensors = {}

# 获取所有轴承接触对象并为每组创建代表性传感器
bearing_groups = ['CrankNeedle', 'NeedleCycloidHole', 'FlangeShaft', 'FlangeHole']
for group_name in bearing_groups:
    group_contacts = [info for info in CONTACT_NODE_INFO if info['group'] == group_name]
    if group_contacts:
        # 为每组的前几个接触对象创建传感器（避免过多传感器）
        n_samples = min(4, len(group_contacts))  # 每组取4个代表
        for i in range(n_samples):
            info = group_contacts[i]
            sensor_name = f"{group_name}-{i}"
            contact_sensors[sensor_name] = {
                'type': 'circle_force',
                'sensor': mbs.AddSensor(SensorObject(
                    objectNumber=info['object'],
                    storeInternal=True,
                    outputVariableType=exu.OutputVariableType.ForceLocal
                ))
            }

# 接触力日志（针销日志已删除，内齿圈结构不需要）
def _post_step_contact_logger(mbs, step_information):
    # 简化版本：不记录详细的针销接触力
    return True  # 返回True表示继续仿真

mbs.Assemble()
print(f"已添加 {len(contact_sensors)} 个接触力传感器")

simulationSettings = exu.SimulationSettings()

# === 启用多核求解 ===

import psutil  # 可选，仅用于获取物理核心数

simulationSettings.parallel.numberOfThreads = psutil.cpu_count(logical=False)

simulationSettings.parallel.useLoadBalancing = False  # 动态负载均衡，适用于步长不均匀的任务
stepSize = 1e-6  # 设置初始步长（从 1e-8 改为 1e-6）
tEnd = 0.04  # 增加仿真时长以获得更多数据点

# 自动计算采样间隔：确保每段扭矩至少10个点，总数据点约200个
n_data_points = 200  # 目标数据点数
sample_interval_by_time = tEnd / n_data_points
sample_interval_by_torque = torqueSegmentTime / 100  # 每段扭矩至少10个采样点
contact_sample_interval = min(sample_interval_by_time, sample_interval_by_torque)

# === 解文件输出设置（压缩数据以减小文件大小）===
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.binarySolutionFile = True

# 指定并创建解文件输出目录，避免默认文件缺失或被其他会话覆盖
solDir = os.path.join(os.path.dirname(__file__), 'solution')
try:
    os.makedirs(solDir, exist_ok=True)
except Exception:
    pass

# 二进制文件自动使用 .sol 扩展名，文本文件用 .txt
solutionFile = os.path.join(solDir, 'cycloid_bearing_solution')
simulationSettings.solutionSettings.coordinatesSolutionFileName = solutionFile
# simulationSettings.timeIntegration.numberOfSteps = int(tEnd / stepSize)  # 注释掉固定步数
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.simulateInRealtime = False

# 使用稀疏求解器以处理大规模系统
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

# Newton求解器设置（放宽容差以加快收敛）
simulationSettings.timeIntegration.newton.useModifiedNewton = True  # 与cycloid.py保持一致
# simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 1e-4
# simulationSettings.timeIntegration.newton.relativeTolerance = 5e-6
# simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-2
# simulationSettings.timeIntegration.newton.maxIterations = 100  # 限制最大迭代次数

# Generalized Alpha设置（提高 spectralRadius 减少数值阻尼）
# simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.3  # 减少数值阻尼以提高效率
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True

# 设置post-step函数（在mbs.Assemble()之后调用）
# 自适应步长控制
simulationSettings.timeIntegration.adaptiveStep = True
# simulationSettings.timeIntegration.initialStepSize = stepSize  # 设置初始步长
# simulationSettings.timeIntegration.adaptiveStepIncrease = 1.2  # 步长增加因子（保守一些）
# simulationSettings.timeIntegration.adaptiveStepDecrease = 0.5   # 步长减少因子
simulationSettings.timeIntegration.minimumStepSize = 1e-10      # 最小步长
# simulationSettings.timeIntegration.stepSizeMaxIncrease = 1.5  # 步长最大增加因子（限制增长）
# simulationSettings.timeIntegration.stepSizeSafety = 0.9       # 步长安全因子

simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.displayComputationTime = True
simulationSettings.displayStatistics = True

# 输出频率根据采样间隔自动设置
SC.visualizationSettings.general.graphicsUpdateInterval = 0.01
simulationSettings.solutionSettings.solutionWritePeriod = contact_sample_interval
simulationSettings.solutionSettings.sensorsWritePeriod = contact_sample_interval

SC.visualizationSettings.window.renderWindowSize = [1000, 800]
SC.visualizationSettings.general.backgroundColor = [0.7, 0.7, 0.7, 1.0]

SC.visualizationSettings.loads.show = True
SC.visualizationSettings.connectors.showContact = SHOW_CONTACT_GEOMETRY
SC.visualizationSettings.loads.loadSizeFactor = 1e-4  # 载荷箭头长度
SC.visualizationSettings.loads.defaultRadius = 0.6     # 箭头半径
SC.visualizationSettings.loads.defaultSize = 4.0       # 基础尺寸更大
SC.visualizationSettings.loads.fixedLoadSize = False   # 随力大小缩放（方案B：同步loadVector可见）
SC.visualizationSettings.loads.drawWithUserFunction = True  # 绘制用户函数载荷
SC.visualizationSettings.loads.showNumbers = False

# 内置接触对象（齿廓-针齿）的接触力可视化
SC.visualizationSettings.contact.contactPointsDefaultSize = CONTACT_POINT_SIZE
SC.visualizationSettings.contact.showContactForces = True
SC.visualizationSettings.contact.showContactForcesValues = SHOW_CONTACT_FORCE_VALUES
SC.visualizationSettings.contact.contactForcesFactor = CONTACT_FORCE_SCALE

# 设置post-step函数
mbs.SetPostStepUserFunction(_post_step_contact_logger)

print("开始仿真...")

# 启动渲染器（如果需要可视化）
if useGraphics:
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()

try:
    # 非交互式运行仿真（不启动渲染窗口）
    if not useGraphics:
        mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.GeneralizedAlpha)
    else:
        # 如果有可视化，先设置非实时模式然后启动仿真
        simulationSettings.timeIntegration.simulateInRealtime = False
        mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.GeneralizedAlpha)
except Exception as e:
    print(f"\n仿真错误: {e}")
    import sys
    sys.exit(1)

if useGraphics:
    try:
        SC.renderer.Stop()
    except:
        pass

print("\n仿真完成！读取数据...")

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
try:
    data_rot = np.asarray(mbs.GetSensorStoredData(sCrankRot))
    data_rot = None if data_rot.size == 0 else data_rot
except:
    data_rot = None

try:
    data_cyc1 = np.asarray(mbs.GetSensorStoredData(sCycloid1Pos))
    data_cyc1 = None if data_cyc1.size == 0 else data_cyc1
except:
    data_cyc1 = None

try:
    data_cyc2 = np.asarray(mbs.GetSensorStoredData(sCycloid2Pos))
    data_cyc2 = None if data_cyc2.size == 0 else data_cyc2
except:
    data_cyc2 = None

try:
    data_input_rot = np.asarray(mbs.GetSensorStoredData(sInputShaftRot))
    data_input_rot = None if data_input_rot.size == 0 else data_input_rot
except:
    data_input_rot = None

try:
    data_output_rot = np.asarray(mbs.GetSensorStoredData(sOutputFlangeRot))
    data_output_rot = None if data_output_rot.size == 0 else data_output_rot
except:
    data_output_rot = None

# 调试读取DisplacementLocal / VelocityLocal
# （已删除：过往用于调试的 sCycloid1Gap / sCycloid1Vel 传感器）

# 读取并处理接触数据
print("接触力统计:")
contact_force_data = {}

# 获取时间数组（从任意传感器）
time_array = None
if data_rot is not None and len(data_rot) > 0:
    time_array = data_rot[:, 0]
elif data_cyc1 is not None and len(data_cyc1) > 0:
    time_array = data_cyc1[:, 0]

for name, sensor_info in contact_sensors.items():
    try:
        data = np.asarray(mbs.GetSensorStoredData(sensor_info['sensor']))
        if data.size == 0:
            continue

        if sensor_info['type'] == 'circle_force':
            if data.ndim == 2 and data.shape[1] >= 4:
                times = data[:, 0]
                force_components = data[:, 1:4]
                force_magnitude = np.linalg.norm(force_components, axis=1)
                if time_array is None:
                    time_array = times
                contact_force_data[name] = np.column_stack([times, force_magnitude])
                max_force = np.max(force_magnitude)
                mean_force = np.mean(force_magnitude[force_magnitude > 0]) if np.any(force_magnitude > 0) else 0
                print(f"  {name}: max={max_force:.1f}N, avg={mean_force:.1f}N")

        elif sensor_info['type'] == 'curve_force':
            if data.ndim == 2 and data.shape[1] >= 4:
                times = data[:, 0]
                force_components = data[:, 1:4]
                total_force = np.linalg.norm(force_components, axis=1)
                if time_array is None:
                    time_array = times
                contact_force_data[name] = np.column_stack([times, total_force])
    except Exception:
        pass

# 针销接触力日志已删除（内齿圈结构不需要针销接触力统计）

plotDir = os.path.join(solDir, 'plots')
try:
    os.makedirs(plotDir, exist_ok=True)
except Exception:
    pass

def save_plot(fig, filename):
    fig.tight_layout()
    fig.savefig(os.path.join(plotDir, filename), dpi=300)
    plt.close(fig)

def extract_z_rotation(data_rot_matrix):
    """从旋转矩阵数据中提取绕Z轴的总转角"""
    if data_rot_matrix is None or len(data_rot_matrix) == 0:
        return None, None
    z_rotation = []
    for i in range(len(data_rot_matrix)):
        R = np.array([[data_rot_matrix[i,1], data_rot_matrix[i,2], data_rot_matrix[i,3]],
                      [data_rot_matrix[i,4], data_rot_matrix[i,5], data_rot_matrix[i,6]],
                      [data_rot_matrix[i,7], data_rot_matrix[i,8], data_rot_matrix[i,9]]])
        angle_z = np.arctan2(R[1,0], R[0,0])
        z_rotation.append(angle_z)
    return data_rot_matrix[:,0], np.unwrap(np.array(z_rotation))

# Crankshaft rotation
time_crank, angle_crank = extract_z_rotation(data_rot)
if time_crank is not None and angle_crank is not None:
    fig = plt.figure(figsize=(8, 5))
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(time_crank, angle_crank, 'b-', linewidth=2)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Rotation (rad)', fontsize=11)
    ax.set_title('Crankshaft Z-Axis Rotation Angle', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    save_plot(fig, 'crankshaft_rotation.png')

# Input vs Output rotation (dual Y-axis)
time_input, angle_input = extract_z_rotation(data_input_rot)
time_output, angle_output = extract_z_rotation(data_output_rot)
if ((time_input is not None and angle_input is not None) or
        (time_output is not None and angle_output is not None)):
    fig = plt.figure(figsize=(8, 5))
    ax_left = fig.add_subplot(1, 1, 1)
    ax_right = ax_left.twinx()
    legend_handles = []
    legend_labels = []
    if time_input is not None and angle_input is not None:
        h1, = ax_left.plot(time_input, angle_input, 'b-', linewidth=2, label='Input Shaft')
        ax_left.set_ylabel('Input Shaft Rotation (rad)', fontsize=11, color='b')
        ax_left.tick_params(axis='y', labelcolor='b')
        legend_handles.append(h1)
        legend_labels.append('Input Shaft')
    if time_output is not None and angle_output is not None:
        h2, = ax_right.plot(time_output, angle_output, 'r-', linewidth=2, label='Output Flange')
        ax_right.set_ylabel('Output Flange Rotation (rad)', fontsize=11, color='r')
        ax_right.tick_params(axis='y', labelcolor='r')
        legend_handles.append(h2)
        legend_labels.append('Output Flange')
    ax_left.set_xlabel('Time (s)', fontsize=11)
    ax_left.set_title('Input Shaft vs Output Flange Rotation', fontsize=12, fontweight='bold')
    ax_left.grid(True, alpha=0.3)
    if legend_handles:
        ax_left.legend(legend_handles, legend_labels, loc='upper left', fontsize=10)
    save_plot(fig, 'input_output_rotation.png')

# Output flange torque vs rotation angle (torque: x-axis, angle: y-axis)
if time_output is not None and angle_output is not None:
    torque_values = np.array([torque_multi_step(outputTorqueZ, float(t)) for t in np.asarray(time_output)])
    if torque_values.size > 0 and len(angle_output) == len(torque_values):
        fig = plt.figure(figsize=(8, 5))
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(torque_values, angle_output, 'm-', linewidth=2)
        ax.set_xlabel('Applied Torque (N·m)', fontsize=11)
        ax.set_ylabel('Output Flange Rotation (rad)', fontsize=11)
        ax.set_title('Output Flange Rotation vs Applied Torque', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        save_plot(fig, 'output_torque_vs_angle.png')

# 针销接触力绘图已删除（内齿圈结构不需要）

# Bearing contact forces
bearing_keywords = ['Needle', 'Crank', 'FlangeShaft', 'FlangeHole']
bearing_contacts = [(k, v) for k, v in contact_force_data.items()
                    if any(kw in k for kw in bearing_keywords) and not k.endswith('_individual')]
if bearing_contacts:
    colors = ['r', 'g', 'b', 'orange', 'purple', 'cyan', 'brown', 'pink']
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(1, 1, 1)
    for idx, (name, data) in enumerate(bearing_contacts):
        if data.ndim == 2 and data.shape[1] >= 2:
            ax.plot(data[:, 0], data[:, 1], color=colors[idx % len(colors)], linewidth=2, label=name)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Contact Force (N)', fontsize=11)
    ax.set_title('All Bearing Contact Forces (Cycloid + Flange)', fontsize=12, fontweight='bold')
    ax.legend(fontsize=8, ncol=2, loc='best')
    ax.grid(True, alpha=0.3)
    save_plot(fig, 'bearing_contact_forces.png')

print(f"图形结果已保存至: {plotDir}")

def _is_valid_solution_file(path):
    try:
        if not os.path.isfile(path):
            return False
        with open(path, 'r', encoding='utf-8', errors='ignore') as f:
            # 读前几行，寻找包含多个数值的行
            linesChecked = 0
            for line in f:
                s = line.strip()
                if not s or s.startswith('#'):
                    continue
                tokens = s.split(',')  # 解文件用逗号分隔
                # 至少包含时间+一个坐标列
                if len(tokens) >= 2:
                    return True
                linesChecked += 1
                if linesChecked > 10:  # 减少检查行数
                    break
        return False
    except Exception:
        return False

# 启动SolutionViewer回放器
if useGraphics:
    try:
        mbs.SolutionViewer()
    except:
        pass
