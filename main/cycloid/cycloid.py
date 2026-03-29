import sys, os

sys.exudynFast = True          # 让 exudyn 优先加载 fast 版（依赖 AVX2）
sys.exudynCPUhasAVX2 = True 

import numpy as np

import exudyn as exu
from exudyn.utilities import *
from cycloid_profile import CycloidProfileParams, generate_cycloid_profile, compute_curvature_with_shaping
import exudyn.graphics as graphics
from exudyn.itemInterface import ObjectContactCircleCircle, MarkerBodyPosition, SensorObject, SensorNode
from exudyn.itemInterface import MarkerSuperElementRigid, MarkerSuperElementPosition, MarkerNodeRigid
from exudyn.itemInterface import ObjectConnectorRigidBodySpringDamper, VObjectConnectorRigidBodySpringDamper

# 柔性摆线轮模块
try:
    from flexible_cycloid import FlexibleCycloidWheel, NGSOLVE_AVAILABLE
except ImportError:
    NGSOLVE_AVAILABLE = False
    print("Warning: flexible_cycloid module not available")

# 柔性曲柄轴模块
from flexible_crankshaft import (
    FlexibleCrankshaft, CrankSegment, 
    create_standard_crank_segments,
    add_crankshaft_constraints
)

# 角接触球轴承模块
from exudyn.machines import GetBallBearingData, CreateBallBearing

# =================== 批量运行参数接口 ===================
# 从命令行参数文件读取（如果有）
BATCH_MODE = False
BATCH_PARAMS = {}
SIMULATION_MODE = 'stiffness'  # 默认刚度模式

# 检查是否有命令行参数（参数文件）
if len(sys.argv) > 1:
    param_file = sys.argv[1]
    if os.path.exists(param_file):
        import json
        with open(param_file, 'r', encoding='utf-8') as f:
            BATCH_PARAMS = json.load(f)
        BATCH_MODE = True
        SIMULATION_MODE = BATCH_PARAMS.get('simulation_mode', 'stiffness')
        print(f"[BATCH MODE] 从文件读取参数: {param_file}")
        print(f"[BATCH MODE] 仿真模式: {SIMULATION_MODE}")

# 默认误差参数（会被apply_batch_params覆盖）
eccentric_error = 0  # 偏心圆半径误差
eccentricity_error = 0  # 偏心距误差
angle_error = 0  # 偏心角误差
bearing_clearance_adjust = 0  # 轴承间隙调整
output_dir = './solution'

# 刚度测试扭矩
STIFFNESS_TEST_TORQUE = 3200.0  # N·m

useGraphics = True  # 默认启用图形窗口显示（批量模式会自动关闭）
SC = exu.SystemContainer()
mbs = SC.AddSystem()


# Contact visualization toggles for consistent arrow output
SHOW_CONTACT_GEOMETRY = True  # 必须为True才能显示ContactCurveCircles的力箭头
CONTACT_FORCE_SCALE = 3e-5  # enlarge contact force arrows for visibility
SHOW_CONTACT_FORCE_VALUES = True
CONTACT_POINT_SIZE = 1.5

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
z_b = 52
z_g = z_b - 1
e = 1.5e-3  # 偏心距 (m)
R_z = 119.5e-3  # 针齿分布圆半径 (m)
r_z = 3e-3  # 针齿半径 (m)

r_crank_main = 7.5e-3  # 曲柄主轴半径 (m)
r_crank_eccentric = 13e-3  # 曲柄偏心段半径 (m)
L_crank_main1 = 20e-3  # 曲柄主轴段1长度 (m)
L_crank_eccentric1 = 15e-3  # 偏心段1长度 (m)
L_crank_middle = 5e-3  # 中间段长度 (m)
L_crank_eccentric2 = 15e-3  # 偏心段2长度 (m)
L_crank_main2 = 20e-3  # 曲柄主轴段2长度 (m)
eccentric_offset = e

z_eccentric1 = L_crank_eccentric1 / 2
z_eccentric2 = L_crank_eccentric1 + L_crank_middle + L_crank_eccentric2 / 2

# 滚针轴承参数
r_needle = 2.5e-3  # 滚针半径 (m)
n_needles = 16  # 每个轴承的滚针数量
bearing_clearance = 0.005e-3  # 轴承径向间隙 (m)
r_needle_pitch = r_crank_eccentric + r_needle + bearing_clearance  # 滚针节圆半径 (m)
r_hole = r_needle_pitch + r_needle + bearing_clearance  # 摆线轮孔半径 (m)

r_pin_shell = R_z - 0.001e-3  # 针齿壳分布圆半径 (m)
r_pin_hole = r_z + 0.01e-3  # 针齿壳孔半径 (m)
thickness_shell = 50e-3  # 壳体厚度 (m)

R_shell_outer = R_z + 20.5e-3  # 壳体外圆半径 (m)
R_shell_inner = R_z + 0.01e-3  # 壳体内圆半径 (m)

# 三曲柄轴配置参数
n_cranks = 3  # 曲柄轴数量
crank_distribution_radius = 70e-3  # 曲柄轴分布圆半径 (m)

ANGLE_OFFSET = 0.5 * np.pi  # 统一让圆周点从 +Y 轴开始

# 接触参数（SI单位）
contactStiffness_tooth = 1e9  # N/m - 摆线齿廓-针齿接触刚度
contactDamping_tooth = 1e5  # N/(m/s) - 摆线齿廓-针齿接触阻尼
contactStiffness_hole = 1e9  # N/m - 孔接触刚度
contactDamping_hole = 1e5  # N/(m/s) - 孔接触阻尼
contactStiffness_pin_hole = 1e9  # N/m - 针齿-壳体孔接触刚度
contactDamping_pin_hole = 3e5  # N/(m/s) - 针齿-壳体孔接触阻尼
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
friction_cycloid_pin = 0.1            # 摆线齿廓-针齿
friction_crank_needle = 0.05            # 偏心轴-滚针
friction_needle_cycloid_hole = 0.01     # 滚针-摆线孔
friction_flange_bearing = 0.06         # 法兰轴承滚针
friction_pin_shell = 0.06               # 针齿-壳体孔

# friction_cycloid_pin = 0.0             # 摆线齿廓-针齿
# friction_crank_needle = 0.0            # 偏心轴-滚针
# friction_needle_cycloid_hole = 0.0     # 滚针-摆线孔
# friction_flange_bearing = 0.0          # 法兰轴承滚针
# friction_pin_shell = 0.0               # 针齿-壳体孔

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
r_flange_needle = 4.0e-3  # 法兰轴承滚针半径 (m)
n_flange_needles = 12  # 每个法兰轴承的滚针数量
flange_bearing_clearance = -0.001e-3  # 法兰轴承间隙 (m)
r_flange_needle_pitch = r_flange_shaft + r_flange_needle + flange_bearing_clearance
r_flange_hole = r_flange_needle_pitch + r_flange_needle + flange_bearing_clearance  # 法兰孔半径 (m)

# =================== ACBB角接触球轴承参数 ===================

# 轴承型号参数（薄壁轴承，安装在法兰外缘附近）
# 外径/内径都在210mm左右（法兰外径104mm*2=208mm）
acbb_radiusBalls = 4e-3               # 滚珠半径 4mm
acbb_outsideDiameter = 216e-3         # 外径 216mm
acbb_boreDiameter = 200e-3            # 内径 200mm（薄壁）
acbb_width = 0.012                    # 宽度 12mm
acbb_nBalls = 40                      # 滚珠数量（薄壁大直径需要较多滚珠）
acbb_radiusCage = (acbb_outsideDiameter/2 + acbb_boreDiameter/2) / 2  # 保持架半径 = (108+100)/2 = 104mm
acbb_innerGrooveRadius = acbb_radiusBalls * 1.04  # 内圈沟道曲率半径
acbb_outerGrooveRadius = acbb_radiusBalls * 1.04  # 外圈沟道曲率半径
acbb_innerRingShoulderRadius = acbb_radiusCage - 0.3 * acbb_radiusBalls  # 内圈肩部半径
acbb_outerRingShoulderRadius = acbb_radiusCage + 0.3 * acbb_radiusBalls  # 外圈肩部半径
acbb_axialPreload = 0  # 轴向预紧（通过滚珠过盈模拟）
acbb_ballOversize = 1.01  # 滚珠过盈系数（1.01 = 1%过盈预压）

# ACBB接触参数
acbb_contactStiffness = 5e6  # 接触刚度 (N/m)
acbb_contactDamping = 50     # 接触阻尼 (Ns/m)
acbb_dynamicFriction = 0.2   # 动摩擦系数
acbb_contactStiffnessExponent = 1  # 刚度指数 (1=线性, 1.5=Hertzian)

# =================== 批量参数应用 ===================
def apply_batch_params():
    """从BATCH_PARAMS应用参数到全局变量"""
    global eccentric_error, eccentricity_error, angle_error
    global bearing_clearance_adjust, useGraphics, output_dir
    global e, r_crank_eccentric, bearing_clearance, eccentric_offset
    global r_needle_pitch, r_hole
    global ANGLE_OFFSET
    global e_list, r_crank_eccentric_list, angle_offset_list, r_needle_pitch_list, r_hole_list

    if BATCH_MODE:
        # 几何误差参数
        eccentric_error = BATCH_PARAMS.get('eccentric_error', 0)
        eccentricity_error = BATCH_PARAMS.get('eccentricity_error', 0)
        angle_error = BATCH_PARAMS.get('angle_error', 0)
        bearing_clearance_adjust = BATCH_PARAMS.get('bearing_clearance_adjust', 0)

        # 输出目录
        output_dir = BATCH_PARAMS.get('output_dir', './results')

        # 图形显示（批量模式默认关闭）
        global useGraphics
        useGraphics = BATCH_PARAMS.get('show_graphics', False)

        # 应用几何误差（为每个曲柄生成独立的随机误差）
        np.random.seed(42)  # 固定种子以保证可重复性
        
        # 定义误差范围（均匀分布）
        # 即使eccentric_error=0, 也要生成全零数组
        err_r_list = np.random.uniform(-abs(eccentric_error), abs(eccentric_error), n_cranks) if eccentric_error != 0 else np.zeros(n_cranks)
        err_e_list = np.random.uniform(-abs(eccentricity_error), abs(eccentricity_error), n_cranks) if eccentricity_error != 0 else np.zeros(n_cranks)
        err_a_list = np.random.uniform(-abs(angle_error), abs(angle_error), n_cranks) if angle_error != 0 else np.zeros(n_cranks)
        
        # 计算每个曲柄的具体参数
        # 1. 偏心距 e
        e_list = e + err_e_list
        # 2. 偏心圆半径 r_crank_eccentric
        r_crank_eccentric_list = r_crank_eccentric + err_r_list
        # 3. 轴承间隙 bearing_clearance (目前假设所有孔的间隙基准一致，加上调整量)
        # 注意：这里 bearing_clearance_adjust 是标量，我们将其应用到所有轴承，但也可以设为随机
        # 此处遵照 run_batch_crankshaft_only 的逻辑，adjust 是一个扫描参数，非随机误差
        bearing_clearance = 0.005e-3 + bearing_clearance_adjust
        
        # 重新计算依赖参数 (注意：r_needle_pitch 和 r_hole 将变为列表)
        # r_needle_pitch = r_crank + r_needle + bearing_clearance
        r_needle_pitch_list = r_crank_eccentric_list + r_needle + bearing_clearance
        
        # r_hole 通常是定值（摆线轮制造一致），间隙变化通常由曲柄或孔的公差引起
        # 这里为了模拟间隙变化，让孔半径跟随 pitch 变化
        r_hole_list = r_needle_pitch_list + r_needle + bearing_clearance

        # 应用角度误差到曲柄轴相位
        angle_offset_list = (0.5 * np.pi) + err_a_list

        print(f"[BATCH MODE] 已应用随机参数 (Seed=42):")
        print(f"  偏心圆半径误差范围: ±{abs(eccentric_error)*1e6:.2f} μm")
        print(f"  偏心距误差范围: ±{abs(eccentricity_error)*1e6:.2f} μm")
        print(f"  偏心角误差范围: ±{np.rad2deg(abs(angle_error))*3600:.1f} arcsec")
        print(f"  轴承间隙调整: {bearing_clearance_adjust*1e6:.2f} μm")
        print(f"  输出目录: {output_dir}")
    else:
        # 非批量模式，使用默认值（全部一致）
        e_list = np.full(n_cranks, e)
        r_crank_eccentric_list = np.full(n_cranks, r_crank_eccentric)
        bearing_clearance = 0.005e-3
        angle_offset_list = np.full(n_cranks, 0.5 * np.pi)
        
        r_needle_pitch_list = r_crank_eccentric_list + r_needle + bearing_clearance
        r_hole_list = r_needle_pitch_list + r_needle + bearing_clearance

# 应用批量参数（必须在建模之前调用）
apply_batch_params()

print("=" * 60)
print("三曲柄轴双片摆线针轮减速器仿真")
print(f"曲柄: {n_cranks}, 摆线滚针: {n_needles}/轴承, 法兰滚针: {n_flange_needles}/轴承")
print("=" * 60)


oGround = mbs.CreateGround()

# 创建摆线齿廓参数对象
profile_params = CycloidProfileParams(
    z_b=z_b,
    e=e,
    R_z=R_z,
    r_z=r_z,
)

# 生成两片摆线轮的齿廓（统一以正Y轴为参考方向）
phase_cycloid = 0
x_cycloid1, y_cycloid1 = generate_cycloid_profile(profile_params, phi_h=phase_cycloid, n_points=1000)
x_cycloid2, y_cycloid2 = generate_cycloid_profile(profile_params, phi_h=phase_cycloid + np.pi, n_points=1000)

def CreateToothSegments(x_profile, y_profile):
    nSeg = len(x_profile)
    segmentsData = np.zeros((nSeg, 4))
    for i in range(nSeg):
        segmentsData[i, 0:2] = [x_profile[i], y_profile[i]]
        segmentsData[i, 2:4] = [x_profile[(i + 1) % nSeg], y_profile[(i + 1) % nSeg]]
    return segmentsData, nSeg

def ComputeSegmentLengths(segmentsData):
    if len(segmentsData) == 0:
        return np.zeros(0)
    delta = segmentsData[:, 2:4] - segmentsData[:, 0:2]
    return np.linalg.norm(delta, axis=1)


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

segmentsData_tooth1, nSeg_tooth1 = CreateToothSegments(x_cycloid1, y_cycloid1)
segmentsData_tooth2, nSeg_tooth2 = CreateToothSegments(x_cycloid2, y_cycloid2)
segment_lengths_tooth1 = ComputeSegmentLengths(segmentsData_tooth1)
segment_lengths_tooth2 = ComputeSegmentLengths(segmentsData_tooth2)


def create_circle_contact(mbs, *, name, markerA, markerB,
                          radiusA, radiusB,
                          stiffness, damping,
                          friction=0.0,
                          frictionVelocityPenalty=1000.0,    # 速度惩罚摩擦模型，更稳定
                          frictionProportionalZone=0.0,    # 平滑零速度附近的摩擦力
                          frictionStiffness=1e8,           # Bristle静摩擦刚度 (N/m)
                          active=True,
                          group=None):
    node = mbs.AddNode(NodeGenericData(
        initialCoordinates=[0., 0., 0., 0., 0., 0., 0., 0.],  # 8个坐标（含Bristle粘滞位置）
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
# outputTorqueZ = 0


torqueSegmentTime = 0.01  # 每段持续时间；默认序列 0→+max→0→-max→0→+max→0

TORQUE_SEQUENCE_DEFAULT = [0.0, 1.0, 0.0, -1.0, 0.0, 1.0]  # 5段：0→+1→0→-1→0→+1

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
    # 仿真结束时返回最终值，避免周期边界跳回0
    if t >= period:
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

# =================== 柔性/刚体曲柄轴选择 ===================
USE_FLEXIBLE_CRANKSHAFT = False  # True=柔性曲柄轴(梁单元), False=刚体曲柄轴

# =================== 创建三个曲柄轴 ===================
# 定义曲柄轴各段
# 定义曲柄轴各段
# 注意：现在改为在循环内部针对每个曲柄分别创建，因为参数不同
# crank_segments = create_standard_crank_segments(...)

# 存储曲柄轴对象和标记（适用于刚性和柔性）
crankshaft_bodies = []        # 刚体曲柄轴列表（刚性模式）
crankshaft_flex = []          # FlexibleCrankshaft 对象列表（柔性模式）
crankshaft_nodes = []         # 每个曲柄轴的节点列表（柔性模式）
crankshaft_build_results = [] # build() 返回的结果（柔性模式）
crankshaft_markers_ecc1 = []  # 第一片摆线轮对应的偏心段标记
crankshaft_markers_ecc2 = []  # 第二片摆线轮对应的偏心段标记

# 法兰处节点/标记（用于后续创建标记）
_crank_input_nodes = []   # 柔性模式
_crank_output_nodes = []  # 柔性模式
_crank_input_bodies = []  # 刚性模式
_crank_output_bodies = [] # 刚性模式

# 曲柄轴总长（用于刚性模式和法兰标记计算）
L_crank_total = L_crank_main1 + L_crank_eccentric1 + L_crank_middle + L_crank_eccentric2 + L_crank_main2

if USE_FLEXIBLE_CRANKSHAFT:
    # =================== 柔性曲柄轴（梁单元） ===================
    print("\n" + "="*60)
    print("Creating FLEXIBLE crankshafts with beam elements...")
    print("="*60)
    
    for i_crank in range(n_cranks):
        # 使用随机化参数创建当前曲柄的段
        current_crank_segments = create_standard_crank_segments(
            L_main1=L_crank_main1,
            L_eccentric1=L_crank_eccentric1,
            L_middle=L_crank_middle,
            L_eccentric2=L_crank_eccentric2,
            L_main2=L_crank_main2,
            r_main=r_crank_main,
            r_eccentric=r_crank_eccentric_list[i_crank],
            eccentric_offset=e_list[i_crank]
        )
        
        angle_crank = angle_offset_list[i_crank] + 2 * np.pi * i_crank / n_cranks
        x_crank = crank_distribution_radius * np.cos(angle_crank)
        y_crank = crank_distribution_radius * np.sin(angle_crank)
        
        # 创建柔性曲柄轴
        crank = FlexibleCrankshaft(
            segments=current_crank_segments,
            n_elements_per_segment=2  # 每段2个梁单元
        )
        
        # 构建曲柄轴（z_offset = -L_crank_main1，使第一段从负z开始）
        # 使用分段独立建模 + 6自由度弹簧连接
        result = crank.build(
            mbs,
            reference_position=[x_crank, y_crank, 0],
            z_offset=-L_crank_main1,
            use_spring_connection=True,      # 使用弹簧连接
            connection_stiffness_factor=1.0  # 标准刚度
        )
        
        crankshaft_flex.append(crank)
        crankshaft_nodes.append(result['nodes'])
        crankshaft_build_results.append(result)
        
        # 边界约束：在第一个节点处约束 Z 平移和 XY 倾转
        node_base = result['first_node']
        mGroundCrank = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                                      localPosition=[x_crank, y_crank, -L_crank_main1]))
        mCrankBase = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node_base))
        mbs.AddObject(GenericJoint(
            markerNumbers=[mGroundCrank, mCrankBase],
            constrainedAxes=[0, 0, 1, 0, 0, 0],  # 释放 XY 平移和三个旋转，只约束 Z 平移
            visualization=VObjectJointGeneric(axesRadius=0.5e-3, axesLength=1.0e-3)
        ))
        
        # 用旋转弹簧连接曲柄轴与输入轴，提供弹性约束
        mInputShaftCrank = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oInputShaft, localPosition=[0, 0, 0]))
        mCrankSpring = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node_base))

        # 齿轮式扭转弹簧的连续角度节点
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
        
        # 创建偏心段接触标记（使用 segment_info 获取中心节点）
        seg_info = result['segment_info']
        
        # 偏心段1 - 中心节点
        ecc1_center_node = seg_info['ecc1']['center_node']
        mEcc1 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=ecc1_center_node))
        crankshaft_markers_ecc1.append(mEcc1)
        
        # 偏心段2 - 中心节点
        ecc2_center_node = seg_info['ecc2']['center_node']
        mEcc2 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=ecc2_center_node))
        crankshaft_markers_ecc2.append(mEcc2)
        
        # 记录法兰处节点
        _crank_input_nodes.append(crank.get_node_at_z(-L_crank_main1 / 2))
        _crank_output_nodes.append(crank.get_node_at_z(
            L_crank_eccentric1 + L_crank_middle + L_crank_eccentric2 + L_crank_main2 / 2))
    
    print(f"Created {n_cranks} flexible crankshafts with {len(crankshaft_nodes[0])} nodes each")

else:
    # =================== 刚体曲柄轴 ===================
    print("\n" + "="*60)
    print("Creating RIGID crankshafts...")
    print("="*60)
    
    # 曲柄轴惯量
    I_crank = InertiaCylinder(density=rho_steel, length=L_crank_total, outerRadius=r_crank_main, axis=2)
    
    for i_crank in range(n_cranks):
        angle_crank = angle_offset_list[i_crank] + 2 * np.pi * i_crank / n_cranks
        x_crank = crank_distribution_radius * np.cos(angle_crank)
        y_crank = crank_distribution_radius * np.sin(angle_crank)
        
        # 质心z位置（相对于曲柄轴起点 z=-L_crank_main1）
        z_center = (L_crank_total / 2 - L_crank_main1)
        
        # 使用随机化参数创建当前曲柄的段
        current_crank_segments = create_standard_crank_segments(
            L_main1=L_crank_main1,
            L_eccentric1=L_crank_eccentric1,
            L_middle=L_crank_middle,
            L_eccentric2=L_crank_eccentric2,
            L_main2=L_crank_main2,
            r_main=r_crank_main,
            r_eccentric=r_crank_eccentric_list[i_crank],
            eccentric_offset=e_list[i_crank]
        )

        # 可视化图形（坐标相对于刚体质心）
        crank_graphics = []
        z_pos = -L_crank_main1 - z_center  # 转换为相对质心的坐标
        for seg in current_crank_segments:
            # 偏心段的可视化：圆柱体中心偏移（偏心在y方向）
            ecc_x = seg.eccentricity_x
            ecc_y = seg.eccentricity_y
            cyl = graphics.Cylinder(pAxis=[ecc_x, ecc_y, z_pos], 
                                    vAxis=[0, 0, seg.length],
                                    radius=seg.radius, color=seg.color)
            crank_graphics.append(cyl)
            z_pos += seg.length
        
        # 创建刚体曲柄轴
        oCrank = mbs.CreateRigidBody(
            inertia=I_crank,
            referencePosition=[x_crank, y_crank, z_center],
            gravity=[0, 0, 0],
            graphicsDataList=crank_graphics
        )
        crankshaft_bodies.append(oCrank)
        
        # 约束：绕z轴旋转约束（铰链）
        # 地面标记z位置需与刚体质心z位置一致，避免初始窜动
        mGroundCrank = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                                      localPosition=[x_crank, y_crank, z_center]))
        mCrankCenter = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank, localPosition=[0, 0, 0]))
        mbs.AddObject(GenericJoint(
            markerNumbers=[mGroundCrank, mCrankCenter],
            constrainedAxes=[0, 0, 1, 0, 0, 0],  # 释放XY平移，约束Z平移和XY倾转，释放Z旋转（与柔性一致）
            visualization=VObjectJointGeneric(axesRadius=0.5e-3, axesLength=1.0e-3)
        ))
        
        # 用旋转弹簧连接曲柄轴与输入轴
        mInputShaftCrank = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oInputShaft, localPosition=[0, 0, 0]))
        mCrankSpring = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank, localPosition=[0, 0, 0]))
        
        nGearSpringData = mbs.AddNode(NodeGenericData(initialCoordinates=[0., 0., 0.], numberOfDataCoordinates=3))
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
        
        # 偏心段标记：通过 localPosition 实现偏心（偏心在y方向）
        # z位置相对于main1末端（即z=0全局），需要减去z_center转为相对质心
        z_ecc1_center = L_crank_eccentric1 / 2  # 偏心段1中心（从main1末端算）
        z_ecc2_center = L_crank_eccentric1 + L_crank_middle + L_crank_eccentric2 / 2  # 偏心段2中心
        
        # localPosition: [x, y, z] 其中 y 方向是偏心方向
        mEcc1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank, 
                                               localPosition=[0, eccentric_offset, z_ecc1_center - z_center]))
        crankshaft_markers_ecc1.append(mEcc1)
        
        mEcc2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank, 
                                               localPosition=[0, -eccentric_offset, z_ecc2_center - z_center]))
        crankshaft_markers_ecc2.append(mEcc2)
        
        # 记录法兰处刚体
        _crank_input_bodies.append(oCrank)
        _crank_output_bodies.append(oCrank)
    
    print(f"Created {n_cranks} rigid crankshafts")


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
# 输入法兰约束：根据USE_ACBB选择ACBB轴承或GenericJoint
mGroundInputFlange = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, z_input_flange]))
mInputFlange = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oInputFlange, localPosition=[0, 0, 0]))

# 创建输出法兰（不固定在地面）
oOutputFlange = mbs.CreateRigidBody(
    referencePosition=[0, 0, z_output_flange],
    inertia=I_flange,
    gravity=[0, 0, 0],
    graphicsDataList=output_flange_graphics,
    nodeType=exu.NodeType.RotationRxyz
)

# 输出法兰Marker（用于ACBB轴承）
mGroundOutputFlange = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, z_output_flange]))
mOutputFlangeMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oOutputFlange, localPosition=[0, 0, 0]))

# =================== ACBB角接触球轴承创建 ===================
print("\n" + "="*60)
print("Creating ACBB bearings for input/output flanges...")
print("="*60)

# 创建轴承数据（轴向为Z轴）
acbb_axis = [0, 0, 1]

# 输入端ACBB轴承数据
acbb_bearingData_input = GetBallBearingData(
    axis=acbb_axis,
    outsideDiameter=acbb_outsideDiameter,
    boreDiameter=acbb_boreDiameter,
    width=acbb_width,
    nBalls=acbb_nBalls,
    radiusBalls=acbb_radiusBalls,
    radiusCage=acbb_radiusCage,
    innerGrooveRadius=acbb_innerGrooveRadius,
    outerGrooveRadius=acbb_outerGrooveRadius,
    innerRingShoulderRadius=acbb_innerRingShoulderRadius,
    outerRingShoulderRadius=acbb_outerRingShoulderRadius,
)

# 输出端ACBB轴承数据（相同参数）
acbb_bearingData_output = GetBallBearingData(
    axis=acbb_axis,
    outsideDiameter=acbb_outsideDiameter,
    boreDiameter=acbb_boreDiameter,
    width=acbb_width,
    nBalls=acbb_nBalls,
    radiusBalls=acbb_radiusBalls,
    radiusCage=acbb_radiusCage,
    innerGrooveRadius=acbb_innerGrooveRadius,
    outerGrooveRadius=acbb_outerGrooveRadius,
    innerRingShoulderRadius=acbb_innerRingShoulderRadius,
    outerRingShoulderRadius=acbb_outerRingShoulderRadius,
)

# 接触参数
acbb_contactParams = {
    'contactStiffness': acbb_contactStiffness,
    'contactDamping': acbb_contactDamping,
    'dynamicFriction': acbb_dynamicFriction,
    'contactStiffnessExponent': acbb_contactStiffnessExponent,
    'frictionProportionalZone': 1e-2,
}

# 滚珠预过盈：增加滚珠半径以产生预压
acbb_bearingData_input['radiusBalls'] *= acbb_ballOversize
acbb_bearingData_output['radiusBalls'] *= acbb_ballOversize

# 创建输入端ACBB（内圈=输入法兰，外圈=针齿壳/Ground）
acbb_items_input = CreateBallBearing(
    mbs, acbb_bearingData_input,
    markerInnerRing=mInputFlange,
    markerOuterRing=mGroundInputFlange,
    densityBalls=rho_steel,
    densityCage=rho_steel * 0.3,  # 保持架密度较轻
    cageInitialAngularVelocity=[0, 0, 0],
    ballsInitialAngularVelocity=[0, 0, 0],
    gravity=[0, 0, 0],
    springStiffnessCage=1e5,
    springDampingCage=1e2,
    contactParametersRingBalls=acbb_contactParams
)
print(f"  Input ACBB: {acbb_nBalls} balls created")

# 创建输出端ACBB（内圈=输出法兰，外圈=针齿壳/Ground）
acbb_items_output = CreateBallBearing(
    mbs, acbb_bearingData_output,
    markerInnerRing=mOutputFlangeMarker,
    markerOuterRing=mGroundOutputFlange,
    densityBalls=rho_steel,
    densityCage=rho_steel * 0.3,
    cageInitialAngularVelocity=[0, 0, 0],
    ballsInitialAngularVelocity=[0, 0, 0],
    gravity=[0, 0, 0],
    springStiffnessCage=1e5,
    springDampingCage=1e2,
    contactParametersRingBalls=acbb_contactParams
)
print(f"  Output ACBB: {acbb_nBalls} balls created")
print("="*60)

# 输入法兰和输出法兰之间添加六自由度刚度耦合
mInputFlangeSpring = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oInputFlange, localPosition=[0, 0, 0]))
mOutputFlangeSpring = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oOutputFlange, localPosition=[0, 0, 0]))

# 六自由度刚度矩阵 [kx, ky, kz, kRx, kRy, kRz]
flange_stiffness_trans = 1e0    # 平移刚度 (N/m) - 很大表示刚性约束
flange_stiffness_rot = 1e10      # RxRy旋转刚度 (N·m/rad)
flange_stiffness_torsion = 1e10  # Rz扭转刚度 (N·m/rad) - 保持与原TorsionalSpringDamper一致
flange_stiffness_6dof = np.diag([
    flange_stiffness_trans,   # kx - X方向平移刚度
    flange_stiffness_trans,   # ky - Y方向平移刚度  
    flange_stiffness_trans,   # kz - Z方向平移刚度（轴向）
    flange_stiffness_rot,     # kRx - 绕X轴旋转刚度
    flange_stiffness_rot,     # kRy - 绕Y轴旋转刚度
    flange_stiffness_torsion, # kRz - 绕Z轴旋转刚度（扭转）
])

flange_damping_trans = 1e6      # 平移阻尼 (N·s/m)
flange_damping_rot = 1e6        # 旋转阻尼 (N·m·s/rad)
flange_damping_6dof = np.diag([
    flange_damping_trans,     # dx
    flange_damping_trans,     # dy
    flange_damping_trans,     # dz
    flange_damping_rot,       # dRx
    flange_damping_rot,       # dRy
    flange_damping_rot,       # dRz
])

nFlangeCouplingData = mbs.AddNode(NodeGenericData(initialCoordinates=[0., 0., 0.], numberOfDataCoordinates=3))
mbs.AddObject(ObjectConnectorRigidBodySpringDamper(
    markerNumbers=[mInputFlangeSpring, mOutputFlangeSpring],
    nodeNumber=nFlangeCouplingData,
    stiffness=flange_stiffness_6dof,
    damping=flange_damping_6dof,
    offset=[0., 0., 0., 0., 0., 0.],  # 初始偏移为0
    intrinsicFormulation=True,  # 使用内在公式（大变形准确）
    visualization=VObjectConnectorRigidBodySpringDamper(show=True, drawSize=5e-3)
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
    
    # 输入法兰孔标记
    mInputHole = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oInputFlange, 
                                                localPosition=[x_hole_center, y_hole_center, 0]))
    input_flange_hole_markers.append(mInputHole)
    
    # 输出法兰孔标记
    mOutputHole = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oOutputFlange, 
                                                 localPosition=[x_hole_center, y_hole_center, 0]))
    output_flange_hole_markers.append(mOutputHole)
    
    # 曲柄轴在输入/输出法兰处的标记
    if USE_FLEXIBLE_CRANKSHAFT:
        # 柔性模式：使用梁节点
        mCrankInput = mbs.AddMarker(MarkerNodeRigid(nodeNumber=_crank_input_nodes[i_crank]))
        mCrankOutput = mbs.AddMarker(MarkerNodeRigid(nodeNumber=_crank_output_nodes[i_crank]))
    else:
        # 刚性模式：使用刚体标记
        z_center = (L_crank_total / 2 - L_crank_main1)
        mCrankInput = mbs.AddMarker(MarkerBodyRigid(bodyNumber=_crank_input_bodies[i_crank],
                                                     localPosition=[0, 0, z_input_flange - z_center]))
        mCrankOutput = mbs.AddMarker(MarkerBodyRigid(bodyNumber=_crank_output_bodies[i_crank],
                                                      localPosition=[0, 0, z_output_flange - z_center]))
    crank_shaft_input_markers.append(mCrankInput)
    crank_shaft_output_markers.append(mCrankOutput)


# =================== 摆线轮（带3个均布孔） ===================
thickness_cycloid = 12.5e-3  # m
mass_cycloid = rho_steel * np.pi * R_z**2 * thickness_cycloid  # kg
I_cycloid = RigidBodyInertia(mass=mass_cycloid, inertiaTensor=solid_disc_inertia(mass_cycloid, R_z, thickness_cycloid), com=[0,0,0])

# =================== 柔性/刚体摆线轮选择 ===================
USE_FLEXIBLE_CYCLOID = False  # True=柔性摆线轮, False=刚体摆线轮
FLEXIBLE_MESH_SIZE = 8e-3     # 柔性体网格尺寸 (m)
FLEXIBLE_N_MODES = 30         # 保留模态数

# 孔中心位置（用于刚体和柔性体）
cycloid_hole_centers = []
for i_hole in range(n_cranks):
    angle_hole = ANGLE_OFFSET + 2 * np.pi * i_hole / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    cycloid_hole_centers.append((x_hole_center, y_hole_center))

# 柔性体相关变量（如果使用柔性体则填充）
flex_cycloid1_data = None
flex_cycloid2_data = None
cycloid_segments_data1 = None
cycloid_segments_data2 = None

if USE_FLEXIBLE_CYCLOID and NGSOLVE_AVAILABLE:
    print("\n" + "="*60)
    print("Creating FLEXIBLE cycloid wheels with NGsolve/Netgen...")
    print("="*60)
    
    # 创建第一片柔性摆线轮（齿廓相位 0°）
    flex_wheel1 = FlexibleCycloidWheel(
        profile_x=x_cycloid1,
        profile_y=y_cycloid1,
        thickness=thickness_cycloid,
        hole_centers=cycloid_hole_centers,
        hole_radius=r_hole,
        center_hole_radius=None,
        density=rho_steel,
        youngs_modulus=2.1e11,
        poissons_ratio=0.3,
        n_modes=FLEXIBLE_N_MODES,
        mesh_size=FLEXIBLE_MESH_SIZE
    )
    flex_wheel1.build(verbose=True)
    
    # 创建第二片柔性摆线轮（齿廓相位 180°，孔位置不变）
    flex_wheel2 = FlexibleCycloidWheel(
        profile_x=x_cycloid2,  # 使用 180° 相位差的齿廓
        profile_y=y_cycloid2,
        thickness=thickness_cycloid,
        hole_centers=cycloid_hole_centers,  # 孔位置不变
        hole_radius=r_hole,
        center_hole_radius=None,
        density=rho_steel,
        youngs_modulus=2.1e11,
        poissons_ratio=0.3,
        n_modes=FLEXIBLE_N_MODES,
        mesh_size=FLEXIBLE_MESH_SIZE
    )
    flex_wheel2.build(verbose=True)
    
    # 保存 segments data（用于接触）
    cycloid_segments_data1 = flex_wheel1.segments_data.copy()
    cycloid_segments_data2 = flex_wheel2.segments_data.copy()
    
    # 第一片柔性摆线轮
    flex_cycloid1_data = flex_wheel1.add_to_system(
        exu, mbs,
        position_ref=[0, eccentric_offset, z_eccentric1],
        rotation_matrix_ref=np.eye(3),
        gravity=[0, 0, 0]
    )
    oCycloid1 = flex_cycloid1_data['bodyNumber']
    nRigid1 = flex_cycloid1_data['nodeNumber']
    
    # 获取齿廓节点索引（用于接触）
    profile_node_indices1 = flex_wheel1.profile_node_indices
    
    # 柔性体 Marker（使用 MarkerBodyRigid，与刚体版本一致）
    mCycloid1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid1, localPosition=[0,0,0]))
    
    # 约束：与刚体版本完全一样
    mGroundCycloid1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, eccentric_offset, z_eccentric1]))
    mbs.AddObject(GenericJoint(markerNumbers=[mGroundCycloid1, mCycloid1], constrainedAxes=[0,0,1,1,1,0], visualization=VObjectJointGeneric(axesRadius=0.5e-3, axesLength=1e-3)))
    
    # 第二片柔性摆线轮（使用 180° 相位差的齿廓）
    flex_cycloid2_data = flex_wheel2.add_to_system(
        exu, mbs,
        position_ref=[0, -eccentric_offset, z_eccentric2],
        rotation_matrix_ref=np.eye(3),
        gravity=[0, 0, 0]
    )
    oCycloid2 = flex_cycloid2_data['bodyNumber']
    nRigid2 = flex_cycloid2_data['nodeNumber']
    
    # 获取第二片的齿廓节点索引
    profile_node_indices2 = flex_wheel2.profile_node_indices
    
    # 柔性体 Marker
    mCycloid2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid2, localPosition=[0,0,0]))
    
    # 约束
    mGroundCycloid2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, -eccentric_offset, z_eccentric2]))
    mbs.AddObject(GenericJoint(markerNumbers=[mGroundCycloid2, mCycloid2], constrainedAxes=[0,0,1,1,1,0], visualization=VObjectJointGeneric(axesRadius=0.5e-3, axesLength=1e-3)))
    
    print("Flexible cycloid wheels created successfully!")
    print("="*60 + "\n")

else:
    # =================== 刚体摆线轮（原代码） ===================
    if USE_FLEXIBLE_CYCLOID and not NGSOLVE_AVAILABLE:
        print("Warning: NGsolve not available, falling back to rigid body cycloid")
    
    # 第一片摆线轮
    cycloid1_graphics = []
    # 外齿廓
    for i in range(len(x_cycloid1)):
        i_next = (i + 1) % len(x_cycloid1)
        cycloid1_graphics.append({'type':'Line','color':[0.9,0.1,0.1,1],'data':[x_cycloid1[i],y_cycloid1[i],0,x_cycloid1[i_next],y_cycloid1[i_next],0]})

    # 3个均布的孔
    n_hole_points = 40
    for i_hole in range(n_cranks):
        x_hole_center, y_hole_center = cycloid_hole_centers[i_hole]
        for i in range(n_hole_points):
            angle1 = ANGLE_OFFSET + i * 2 * np.pi / n_hole_points
            angle2 = ANGLE_OFFSET + (i + 1) * 2 * np.pi / n_hole_points
            x1 = x_hole_center + r_hole * np.cos(angle1)
            y1 = y_hole_center + r_hole * np.sin(angle1)
            x2 = x_hole_center + r_hole * np.cos(angle2)
            y2 = y_hole_center + r_hole * np.sin(angle2)
            cycloid1_graphics.append({'type':'Line','color':[0.1,0.7,0.1,1],'data':[x1,y1,0,x2,y2,0]})

    # 第一片摆线轮初始位置：与第一个曲柄轴的偏心段对齐
    oCycloid1 = mbs.CreateRigidBody(referencePosition=[0, eccentric_offset, z_eccentric1], inertia=I_cycloid, gravity=[0,0,0], graphicsDataList=cycloid1_graphics)
    mCycloid1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid1, localPosition=[0,0,0]))
    mGroundCycloid1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, eccentric_offset, z_eccentric1]))
    mbs.AddObject(GenericJoint(markerNumbers=[mGroundCycloid1, mCycloid1], constrainedAxes=[0,0,1,1,1,0], visualization=VObjectJointGeneric(axesRadius=0.5e-3, axesLength=1e-3)))

    # 第二片摆线轮
    cycloid2_graphics = []
    # 外齿廓
    for i in range(len(x_cycloid2)):
        i_next = (i + 1) % len(x_cycloid2)
        cycloid2_graphics.append({'type':'Line','color':[0.6,0.1,0.9,1],'data':[x_cycloid2[i],y_cycloid2[i],0,x_cycloid2[i_next],y_cycloid2[i_next],0]})

    # 3个均布的孔
    for i_hole in range(n_cranks):
        x_hole_center, y_hole_center = cycloid_hole_centers[i_hole]
        for i in range(n_hole_points):
            angle1 = ANGLE_OFFSET + i * 2 * np.pi / n_hole_points
            angle2 = ANGLE_OFFSET + (i + 1) * 2 * np.pi / n_hole_points
            x1 = x_hole_center + r_hole * np.cos(angle1)
            y1 = y_hole_center + r_hole * np.sin(angle1)
            x2 = x_hole_center + r_hole * np.cos(angle2)
            y2 = y_hole_center + r_hole * np.sin(angle2)
            cycloid2_graphics.append({'type':'Line','color':[0.1,0.7,0.9,1],'data':[x1,y1,0,x2,y2,0]})

    # 第二片摆线轮初始位置：与第二个曲柄轴的偏心段对齐
    oCycloid2 = mbs.CreateRigidBody(referencePosition=[0, -eccentric_offset, z_eccentric2], inertia=I_cycloid, gravity=[0,0,0], graphicsDataList=cycloid2_graphics)
    mCycloid2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid2, localPosition=[0,0,0]))
    mGroundCycloid2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, -eccentric_offset, z_eccentric2]))
    mbs.AddObject(GenericJoint(markerNumbers=[mGroundCycloid2, mCycloid2], constrainedAxes=[0,0,1,1,1,0], visualization=VObjectJointGeneric(axesRadius=0.5e-3, axesLength=1e-3)))

# =================== 摆线轮孔的标记（用于轴承连接） ===================
cycloid1_hole_markers = []
cycloid2_hole_markers = []
for i_hole in range(n_cranks):
    x_hole_center, y_hole_center = cycloid_hole_centers[i_hole]
    
    if USE_FLEXIBLE_CYCLOID and NGSOLVE_AVAILABLE and flex_cycloid1_data is not None:
        # 柔性体：使用 MarkerSuperElementRigid 以追踪柔性变形
        
        # 第一片摆线轮孔节点和标记
        hole_nodes1 = flex_wheel1.get_hole_nodes_list(i_hole, n_nodes=8)
        n_hole_nodes1 = len(hole_nodes1)
        weighting_factors1 = [1.0 / n_hole_nodes1] * n_hole_nodes1

        mHole1 = mbs.AddMarker(MarkerSuperElementRigid(
            bodyNumber=oCycloid1,
            meshNodeNumbers=hole_nodes1,
            weightingFactors=weighting_factors1,
            useAlternativeApproach=True
        ))
        
        # 第二片摆线轮孔节点和标记（必须独立获取，因为网格不同）
        hole_nodes2 = flex_wheel2.get_hole_nodes_list(i_hole, n_nodes=8)
        n_hole_nodes2 = len(hole_nodes2)
        weighting_factors2 = [1.0 / n_hole_nodes2] * n_hole_nodes2
        
        mHole2 = mbs.AddMarker(MarkerSuperElementRigid(
            bodyNumber=oCycloid2,
            meshNodeNumbers=hole_nodes2,
            weightingFactors=weighting_factors2,
            useAlternativeApproach=True
        ))
    else:
        # 刚体：使用 MarkerBodyRigid
        mHole1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid1, localPosition=[x_hole_center, y_hole_center, 0]))
        mHole2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid2, localPosition=[x_hole_center, y_hole_center, 0]))
    
    cycloid1_hole_markers.append(mHole1)
    cycloid2_hole_markers.append(mHole2)

# 注意：输出法兰不与摆线轮直接连接，仅通过法兰轴承与曲柄轴连接


# =================== 针齿壳刚体 ===================
z_shell = (z_eccentric1 + z_eccentric2) / 2
shell_graphics_list = []
n_shell_outer_points = 120
for i in range(n_shell_outer_points):
    angle1 = ANGLE_OFFSET + i * 2 * np.pi / n_shell_outer_points
    angle2 = ANGLE_OFFSET + (i + 1) * 2 * np.pi / n_shell_outer_points
    x1 = R_shell_outer * np.cos(angle1); y1 = R_shell_outer * np.sin(angle1)
    x2 = R_shell_outer * np.cos(angle2); y2 = R_shell_outer * np.sin(angle2)
    shell_graphics_list.append({'type':'Line','color':[0.5,0.5,0.5,1],'data':[x1,y1,0,x2,y2,0]})
for i in range(n_shell_outer_points):
    angle1 = ANGLE_OFFSET + i * 2 * np.pi / n_shell_outer_points
    angle2 = ANGLE_OFFSET + (i + 1) * 2 * np.pi / n_shell_outer_points
    x1 = R_shell_inner * np.cos(angle1); y1 = R_shell_inner * np.sin(angle1)
    x2 = R_shell_inner * np.cos(angle2); y2 = R_shell_inner * np.sin(angle2)
    shell_graphics_list.append({'type':'Line','color':[0.5,0.5,0.5,1],'data':[x1,y1,0,x2,y2,0]})

n_pins_show = z_b
n_hole_circle_points = 48
for i in range(n_pins_show):
    angle_pin = ANGLE_OFFSET + 2 * np.pi * i / n_pins_show
    x_hole_center = r_pin_shell * np.sin(angle_pin)
    y_hole_center = r_pin_shell * np.cos(angle_pin)
    for j in range(n_hole_circle_points):
        angle1 = j * 2 * np.pi / n_hole_circle_points
        angle2 = (j + 1) * 2 * np.pi / n_hole_circle_points
        x1 = x_hole_center + r_pin_hole * np.cos(angle1)
        y1 = y_hole_center + r_pin_hole * np.sin(angle1)
        x2 = x_hole_center + r_pin_hole * np.cos(angle2)
        y2 = y_hole_center + r_pin_hole * np.sin(angle2)
        shell_graphics_list.append({'type':'Line','color':[0.9,0.5,0.1,1],'data':[x1,y1,0,x2,y2,0]})

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

def CreatePinHoleSegments(hole_radius, hole_center, nPoints=40):
    pList = []
    for i in range(nPoints):
        phi = i * 2 * np.pi / nPoints
        x = hole_center[0] + hole_radius * np.cos(phi)
        y = hole_center[1] + hole_radius * np.sin(phi)
        pList.append([x, y])
    nSeg = len(pList)
    segmentsData = np.zeros((nSeg, 4))
    for i in range(nSeg):
        segmentsData[i, 0:2] = pList[i]
        segmentsData[i, 2:4] = pList[(i + 1) % nSeg]
    return segmentsData, nSeg

# 针齿（可运动）
pin_bodies = []
pin_markers = []
pin_radii = []
pin_hole_segments = []
shell_hole_markers = []

pin_length = 20.0e-3  # m
mass_pin = cylinder_mass(r_z, pin_length)
inertia_pin = cylinder_inertia(mass_pin, r_z, pin_length, axis=2)

for i in range(n_pins_show):
    angle = ANGLE_OFFSET + 2 * np.pi * i / n_pins_show
    x_pin = r_pin_shell * np.sin(angle)
    y_pin = r_pin_shell * np.cos(angle)
    z_pin = z_shell
    oPin = mbs.CreateRigidBody(referencePosition=[x_pin, y_pin, z_pin], inertia=inertia_pin, gravity=[0,0,0], graphicsDataList=[graphics.Cylinder(pAxis=[0,0,-pin_length/2], vAxis=[0,0,pin_length], radius=r_z, color=graphics.color.steelblue, nTiles=32)])
    # 将针齿壳孔标记从Ground改为针齿壳刚体（关键修改：使摩擦力计算有完整的雅可比矩阵）
    mShellHole = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oShell, localPosition=[x_pin, y_pin, 0]))
    shell_hole_markers.append(mShellHole)
    mPinPlane = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oPin, localPosition=[0, 0, 0]))
    mbs.AddObject(GenericJoint(markerNumbers=[mShellHole, mPinPlane], constrainedAxes=[1,1,1,1,1,1], visualization=VObjectJointGeneric(axesRadius=0.15e-3, axesLength=0.4e-3, show=False)))
    mPin = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oPin, localPosition=[0,0,0]))
    pin_markers.append(mPin)
    pin_radii.append(r_z)
    pin_bodies.append(oPin)
    hole_segments, nSeg_hole = CreatePinHoleSegments(r_pin_hole, [x_pin, y_pin], nPoints=60)
    hole_lengths = ComputeSegmentLengths(hole_segments)
    pin_hole_segments.append((hole_segments, nSeg_hole, exu.MatrixContainer(hole_segments), hole_lengths))

# 外齿廓-针齿接触（每个针销单独建立一个 ContactCurveCircles）
cycloid_pin_contact_objects = []

# 根据是否使用柔性体选择 segmentsData 和段数
if USE_FLEXIBLE_CYCLOID and NGSOLVE_AVAILABLE and cycloid_segments_data1 is not None:
    # 柔性体：使用 FEM 节点生成的 segments data
    active_segData1 = cycloid_segments_data1
    active_segData2 = cycloid_segments_data2
    active_nSeg1 = len(cycloid_segments_data1)
    active_nSeg2 = len(cycloid_segments_data2)
    active_segLengths1 = np.linalg.norm(np.diff(cycloid_segments_data1[:, :2], axis=0), axis=1)
    active_segLengths2 = np.linalg.norm(np.diff(cycloid_segments_data2[:, :2], axis=0), axis=1)
    # 节点映射（用于 C++ 柔性体变形追踪）- 两片摆线轮各自的节点索引
    active_profileNodeIndices1 = np.array(profile_node_indices1, dtype=int)
    active_profileNodeIndices2 = np.array(profile_node_indices2, dtype=int)
    # 获取基体刚度分布（考虑法兰孔的影响）
    base_stiffness_cycloid1 = flex_wheel1.get_stiffness_per_segment()
    base_stiffness_cycloid2 = flex_wheel2.get_stiffness_per_segment()
    print(f"Base stiffness range: {base_stiffness_cycloid1.min():.2e} - {base_stiffness_cycloid1.max():.2e} N/m")
else:
    # 刚体：使用原始齿廓点生成的 segments data
    active_segData1 = segmentsData_tooth1
    active_segData2 = segmentsData_tooth2
    active_nSeg1 = nSeg_tooth1
    active_nSeg2 = nSeg_tooth2
    active_segLengths1 = segment_lengths_tooth1
    active_segLengths2 = segment_lengths_tooth2
    # 刚体不需要节点映射
    active_profileNodeIndices1 = np.array([], dtype=int)
    active_profileNodeIndices2 = np.array([], dtype=int)
    # 刚体：使用默认刚度（空数组表示不使用基体刚度）
    base_stiffness_cycloid1 = np.array([])
    base_stiffness_cycloid2 = np.array([])

# 使用 CasADi 自动微分计算摆线轮曲率（考虑修形）
print("Computing curvature with CasADi automatic differentiation...")
_, curvature_cycloid1 = compute_curvature_with_shaping(profile_params, use_casadi=True)
# 第二片摆线轮相位差 180°（π弧度），需要将曲率数组循环移位半个周期
# 移位量 = n_points / z_b / 2（因为总齿数z_b对应完整2π，相位差π对应一半）
n_points_curvature = len(curvature_cycloid1)
shift_amount = n_points_curvature // 2  # 180°相位差 = 半周期移位
curvature_cycloid2 = np.roll(curvature_cycloid1, shift_amount)
print(f"Curvature range: {curvature_cycloid1.min():.2e} - {curvature_cycloid1.max():.2e} 1/m")


# 第一片摆线轮 - 为每个针销单独建立接触
for idx_pin, (mPin, pin_radius) in enumerate(zip(pin_markers, pin_radii)):
    initialCoords_tooth1 = []
    for _ in range(active_nSeg1):
        initialCoords_tooth1.extend([-1.0, 0.0, 0.0, 0.0, 0.0])  # 5个变量（含Bristle粘滞位置）
    nGenericData_tooth1 = mbs.AddNode(NodeGenericData(
        initialCoordinates=initialCoords_tooth1, 
        numberOfDataCoordinates=5 * active_nSeg1
    ))

    objCycloid1Pin = mbs.AddObject(ObjectContactCurveCircles(
        markerNumbers=[mCycloid1, mPin],  # 一条曲线 + 一个圆
        nodeNumber=nGenericData_tooth1,
        circlesRadii=[pin_radius],  # 只有一个圆的半径
        segmentsData=exu.MatrixContainer(active_segData1),
        profileNodeIndices=active_profileNodeIndices1,  # 第一片摆线轮节点映射
        contactStiffness=contactStiffness_tooth,
        contactDamping=contactDamping_tooth,
        dynamicFriction=friction_cycloid_pin,
        frictionProportionalZone=0.0,      # 平滑零速度附近的摩擦力
        frictionVelocityPenalty=1000.0,     # 速度惩罚摩擦模型，更稳定
        frictionStiffness=1e8,             # Bristle静摩擦刚度 (N/m)
        contactModel=0,
        # Hertz接触参数
        useHertzContact=True,
        faceWidth=thickness_cycloid,
        elasticModulus=2.1e11,
        poissonRatio=0.3,
        curvaturePerPoint=curvature_cycloid1.tolist(),
        baseStiffnessPerPoint=base_stiffness_cycloid1.tolist(),  # 考虑法兰孔的基体刚度
        visualization=VObjectContactCurveCircles(show=SHOW_CONTACT_GEOMETRY, color=graphics.color.blue)
    ))
    _register_contact_info("CycloidPins", f"Cycloid1Pin-{idx_pin:02d}", nGenericData_tooth1, objCycloid1Pin, active_segLengths1, contactStiffness_tooth, contactDamping_tooth)
    cycloid_pin_contact_objects.append(objCycloid1Pin)

# 第二片摆线轮 - 为每个针销单独建立接触
for idx_pin, (mPin, pin_radius) in enumerate(zip(pin_markers, pin_radii)):
    initialCoords_tooth2 = []
    for _ in range(active_nSeg2):
        initialCoords_tooth2.extend([-1.0, 0.0, 0.0, 0.0, 0.0])  # 5个变量（含Bristle粘滞位置）
    nGenericData_tooth2 = mbs.AddNode(NodeGenericData(
        initialCoordinates=initialCoords_tooth2, 
        numberOfDataCoordinates=5 * active_nSeg2
    ))

    objCycloid2Pin = mbs.AddObject(ObjectContactCurveCircles(
        markerNumbers=[mCycloid2, mPin],  # 一条曲线 + 一个圆
        nodeNumber=nGenericData_tooth2,
        circlesRadii=[pin_radius],  # 只有一个圆的半径
        segmentsData=exu.MatrixContainer(active_segData2),
        profileNodeIndices=active_profileNodeIndices2,  # 第二片摆线轮节点映射
        contactStiffness=contactStiffness_tooth,
        contactDamping=contactDamping_tooth,
        dynamicFriction=friction_cycloid_pin,
        frictionProportionalZone=0.0,      # 平滑零速度附近的摩擦力
        frictionVelocityPenalty=1000.0,     # 速度惩罚摩擦模型，更稳定
        frictionStiffness=1e8,             # Bristle静摩擦刚度 (N/m)
        contactModel=0,
        # Hertz接触参数
        useHertzContact=True,
        faceWidth=thickness_cycloid,
        elasticModulus=2.1e11,
        poissonRatio=0.3,
        curvaturePerPoint=curvature_cycloid2.tolist(),
        baseStiffnessPerPoint=base_stiffness_cycloid2.tolist(),  # 考虑法兰孔的基体刚度
        visualization=VObjectContactCurveCircles(show=SHOW_CONTACT_GEOMETRY, color=graphics.color.magenta)
    ))
    _register_contact_info("CycloidPins", f"Cycloid2Pin-{idx_pin:02d}", nGenericData_tooth2, objCycloid2Pin, active_segLengths2, contactStiffness_tooth, contactDamping_tooth)
    cycloid_pin_contact_objects.append(objCycloid2Pin)

# 滚针轴承参数
contactStiffness_bearing = 1e9  # N/m (SI单位) #轴承刚度
contactDamping_bearing = 1e5  # N/(m/s)    #轴承阻尼
needle_length = 12.0e-3  # m
mass_needle = cylinder_mass(r_needle, needle_length)
inertia_needle = cylinder_inertia(mass_needle, r_needle, needle_length, axis=2)

# 法兰轴承参数
contactStiffness_flange_bearing = 1e9  # N/m   #轴承刚度
contactDamping_flange_bearing = 1e5  # N/(m/s) #轴承阻尼
flange_needle_length = 12.0e-3  # m
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
    current_r_needle_pitch = r_needle_pitch_list[i_crank]
    current_r_hole = r_hole_list[i_crank]
    current_angle_offset = angle_offset_list[i_crank]
    current_e = e_list[i_crank]

    angle_hole = current_angle_offset + 2 * np.pi * i_crank / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    hole_local = np.array([x_hole_center, y_hole_center, 0.0])
    cage_hole_radius_cyc, cage_ring_inner_cyc, cage_ring_outer_cyc = compute_cage_radii(
        current_r_needle_pitch, r_needle, current_r_hole)

    cage_graphics_cycloid = create_cage_ring_graphics(
        center=[0, 0, 0],
        pitch_radius=current_r_needle_pitch,
        hole_radius=cage_hole_radius_cyc,
        n_holes=n_needles,
        ring_inner_radius=cage_ring_inner_cyc,
        ring_outer_radius=cage_ring_outer_cyc,
        color_ring=cage_color_cycloid,
        color_hole=cage_color_cycloid,
        angle_offset=current_angle_offset
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
        angle_needle = current_angle_offset + 2 * np.pi * i_needle / n_needles

        # 先创建保持架孔标记
        cage_local_pos1 = [current_r_needle_pitch * np.cos(angle_needle),
                           current_r_needle_pitch * np.sin(angle_needle),
                           0.0]
        mCycloid1CageHole = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCage1Body,
                                                         localPosition=cage_local_pos1))
        cage_markers_cyc1[i_crank].append(mCycloid1CageHole)

        # 创建滚针刚体
        x_needle_1 = x_hole_center + current_r_needle_pitch * np.cos(angle_needle)
        y_needle_1 = y_hole_center + current_e + current_r_needle_pitch * np.sin(angle_needle)
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
            radiusA=r_crank_eccentric_list[i_crank],
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
            radiusB=-current_r_hole,
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
        x_needle_2 = x_hole_center + current_r_needle_pitch * np.cos(angle_needle)
        y_needle_2 = y_hole_center - current_e + current_r_needle_pitch * np.sin(angle_needle)
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
            radiusA=r_crank_eccentric_list[i_crank],
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
            radiusB=-current_r_hole,
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

# 注释掉针销与针齿壳孔的接触（已通过关节约束）
# for idx_pin, (mPin, pin_body) in enumerate(zip(pin_markers, pin_bodies)):
#     mShellHole = shell_hole_markers[idx_pin]

#     create_circle_contact(
#         mbs,
#         name=f"PinShell-{idx_pin:02d}",
#         markerA=mPin,
#         markerB=mShellHole,
#         radiusA=r_z,
#         radiusB=-r_pin_hole,
#         stiffness=contactStiffness_pin_hole,
#         damping=contactDamping_pin_hole,
#         friction=friction_pin_shell,
#         group="PinShell"
#     )

# =================== 传感器/组装/仿真 ===================
mbs.CreateGround(graphicsDataList=[])

# 传感器（需在仿真前创建并 storeInternal 以记录数据）
if USE_FLEXIBLE_CYCLOID and NGSOLVE_AVAILABLE and flex_cycloid1_data is not None:
    # 柔性体：使用 SensorNode 监测刚体参考节点
    sCycloid1Pos = mbs.AddSensor(SensorNode(nodeNumber=nRigid1, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
    sCycloid2Pos = mbs.AddSensor(SensorNode(nodeNumber=nRigid2, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
else:
    # 刚体：使用 SensorBody
    sCycloid1Pos = mbs.AddSensor(SensorBody(bodyNumber=oCycloid1, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
    sCycloid2Pos = mbs.AddSensor(SensorBody(bodyNumber=oCycloid2, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
# 曲柄轴传感器
if USE_FLEXIBLE_CRANKSHAFT:
    sCrankRot = mbs.AddSensor(SensorNode(nodeNumber=crankshaft_nodes[0][0], storeInternal=True, outputVariableType=exu.OutputVariableType.RotationMatrix))
else:
    sCrankRot = mbs.AddSensor(SensorBody(bodyNumber=crankshaft_bodies[0], storeInternal=True, outputVariableType=exu.OutputVariableType.RotationMatrix))
sPin0Pos     = mbs.AddSensor(SensorBody(bodyNumber=pin_bodies[0], storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
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

# 接触力日志（记录每个针齿的实际力）
n_pin_circles = len(pin_markers)
contact_sample_times = []
contact_log_cyc1 = []
contact_log_cyc2 = []

def _sample_cycloid_contact_forces(mbs, object_index, node_index, n_circles):
    seg_fx = np.asarray(mbs.GetObjectParameter(object_index, 'segmentsForceLocalX'), dtype=float)
    if seg_fx.size == 0:
        return np.zeros(n_circles, dtype=float)
    seg_fy = np.asarray(mbs.GetObjectParameter(object_index, 'segmentsForceLocalY'), dtype=float)
    if seg_fy.size == 0:
        return np.zeros(n_circles, dtype=float)

    node_data = np.asarray(mbs.GetNodeOutput(node_index, exu.OutputVariableType.Coordinates,
                                             configuration=exu.ConfigurationType.Current), dtype=float)
    if node_data.size == 0:
        return np.zeros(n_circles, dtype=float)

    circle_indices = node_data[0::5].astype(np.int32, copy=False)  # 每线段5个变量，第0个是圆索引
    n_segments = min(len(circle_indices), len(seg_fx))
    if n_segments == 0:
        return np.zeros(n_circles, dtype=float)

    circle_indices = circle_indices[:n_segments]
    seg_fx = seg_fx[:n_segments]
    seg_fy = seg_fy[:n_segments]

    valid = (circle_indices >= 0) & (circle_indices < n_circles)
    if not np.any(valid):
        return np.zeros(n_circles, dtype=float)

    force_magnitudes = np.hypot(seg_fx[valid], seg_fy[valid])
    forces = np.bincount(circle_indices[valid], weights=force_magnitudes, minlength=n_circles)
    if forces.size < n_circles:
        forces = np.pad(forces, (0, n_circles - forces.size))
    return forces

def _post_step_contact_logger(mbs, step_information):
    if contact_sample_interval is None:
        return True
    current_time = mbs.systemData.GetTime()
    if (not contact_sample_times) or (current_time >= contact_sample_times[-1] + contact_sample_interval - 1e-12):
        contact_sample_times.append(current_time)
        forces_cyc1 = np.zeros(n_pin_circles, dtype=float)
        forces_cyc2 = np.zeros(n_pin_circles, dtype=float)

        for info in CONTACT_NODE_INFO:
            if info.get('group') != 'CycloidPins':
                continue

            label = info.get('label', '')
            try:
                pin_id = int(label.split('-')[-1])
            except (ValueError, AttributeError, IndexError):
                pin_id = None

            sample = _sample_cycloid_contact_forces(
                mbs,
                info['object'],
                info['node'],
                1
            )
            magnitude = float(sample[0]) if sample.size else 0.0

            if 'Cycloid1' in label:
                if pin_id is not None and 0 <= pin_id < n_pin_circles:
                    forces_cyc1[pin_id] = magnitude
            elif 'Cycloid2' in label:
                if pin_id is not None and 0 <= pin_id < n_pin_circles:
                    forces_cyc2[pin_id] = magnitude

        contact_log_cyc1.append(forces_cyc1)
        contact_log_cyc2.append(forces_cyc2)
    return True  # 返回True表示继续仿真

mbs.Assemble()
print(f"已添加 {len(contact_sensors)} 个接触力传感器")

simulationSettings = exu.SimulationSettings()

# === 启用多核求解 ===

import psutil  # 可选，仅用于获取物理核心数

simulationSettings.parallel.numberOfThreads = psutil.cpu_count(logical=False)

simulationSettings.parallel.useLoadBalancing = True  # 动态负载均衡，适用于步长不均匀的任务
stepSize = 1e-5  # 设置初始步长（从 1e-8 改为 1e-6）
tEnd = 0.05  # 仿真时长（与扭矩序列匹配：5段 × 0.01秒）

# 自动计算采样间隔：确保每段扭矩至少100个点，总数据点约200个
n_data_points = 200  # 目标数据点数
sample_interval_by_time = tEnd / n_data_points
sample_interval_by_torque = torqueSegmentTime / 100  # 每段扭矩至少100个采样点
contact_sample_interval = min(sample_interval_by_time, sample_interval_by_torque)

# === 解文件输出设置（压缩数据以减小文件大小）===
# 批量模式不输出解文件以节省空间和时间
simulationSettings.solutionSettings.writeSolutionToFile = not BATCH_MODE
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
simulationSettings.timeIntegration.numberOfSteps = int(tEnd / stepSize)  # 注释掉固定步数
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.simulateInRealtime = False

# 使用稀疏求解器以处理大规模系统
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

# Newton求解器设置（放宽容差以加快收敛）
simulationSettings.timeIntegration.newton.useModifiedNewton = True
# simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 1e-4
# simulationSettings.timeIntegration.newton.relativeTolerance = 1e-2
# simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-2
# simulationSettings.timeIntegration.newton.maxIterations = 100  # 限制最大迭代次数

# Generalized Alpha设置（提高 spectralRadius 减少数值阻尼）
# simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.3  # 减少数值阻尼以提高效率
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True

# 设置post-step函数（在mbs.Assemble()之后调用）
# 自适应步长控制
simulationSettings.timeIntegration.adaptiveStep = True
simulationSettings.timeIntegration.initialStepSize = stepSize  # 设置初始步长
simulationSettings.timeIntegration.adaptiveStepIncrease = 1.2  # 步长增加因子（保守一些）
simulationSettings.timeIntegration.adaptiveStepDecrease = 0.8   # 步长减少因子
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

# 减小节点坐标系箭头大小（柔性曲柄轴梁节点）
SC.visualizationSettings.nodes.defaultSize = 0.002  # 节点默认大小
SC.visualizationSettings.nodes.drawNodesAsPoint = True  # 节点显示为点而非坐标系

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

contact_sample_times.clear()
contact_log_cyc1.clear()
contact_log_cyc2.clear()

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
try:
    data_gap = np.asarray(mbs.GetSensorStoredData(sCycloid1Gap))
    print(f"调试: DisplacementLocal shape = {data_gap.shape}")
    if data_gap.size:
        print(f"调试: DisplacementLocal 首行 = {data_gap[0]}")
except Exception as e:
    print(f"调试: 读取DisplacementLocal失败: {e}")
    data_gap = None

try:
    data_gap_vel = np.asarray(mbs.GetSensorStoredData(sCycloid1Vel))
    print(f"调试: VelocityLocal shape = {data_gap_vel.shape}")
    if data_gap_vel.size:
        print(f"调试: VelocityLocal 首行 = {data_gap_vel[0]}")
except Exception as e:
    print(f"调试: 读取VelocityLocal失败: {e}")
    data_gap_vel = None

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

# 利用日志数据补充每个针齿的实际接触力
individual_contact_data = {}
if contact_sample_times:
    sample_times = np.asarray(contact_sample_times)
    cyc1_matrix = np.vstack(contact_log_cyc1) if contact_log_cyc1 else np.zeros((0, n_pin_circles))
    cyc2_matrix = np.vstack(contact_log_cyc2) if contact_log_cyc2 else np.zeros((0, n_pin_circles))

    if cyc1_matrix.size:
        total_cyc1 = cyc1_matrix.sum(axis=1)
        contact_force_data['Cycloid1-Pins'] = np.column_stack([sample_times, total_cyc1])
        individual_contact_data['Cycloid1-Pins'] = {'time': sample_times, 'forces': cyc1_matrix}
    if cyc2_matrix.size:
        total_cyc2 = cyc2_matrix.sum(axis=1)
        contact_force_data['Cycloid2-Pins'] = np.column_stack([sample_times, total_cyc2])
        individual_contact_data['Cycloid2-Pins'] = {'time': sample_times, 'forces': cyc2_matrix}

    # 打印每个摆线轮针齿的统计信息
    for label, matrix in (('Cycloid1-Pins', cyc1_matrix), ('Cycloid2-Pins', cyc2_matrix)):
        if matrix.size == 0:
            continue
        max_per_pin = matrix.max(axis=0)
        active_mask = max_per_pin > 1e-6
        active_indices = np.nonzero(active_mask)[0]
        active_count = active_indices.size
        if active_count == 0:
            print(f"  {label}: 无有效接触")
            continue
        sum_per_pin = matrix.sum(axis=0)
        contact_counts = np.count_nonzero(matrix > 1e-6, axis=0)
        mean_per_pin = np.zeros_like(sum_per_pin)
        valid = contact_counts > 0
        mean_per_pin[valid] = sum_per_pin[valid] / contact_counts[valid]
        print(f"  {label}: 活跃针齿 {active_count}/{n_pin_circles}")
        for pin_idx in active_indices:
            print(f"    Pin{pin_idx:02d}: max={max_per_pin[pin_idx]:.2f}N, avg={mean_per_pin[pin_idx]:.2f}N")

contact_force_data.update({f"{k}_individual": v for k, v in individual_contact_data.items()})

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

# Pin contact forces
cycloid1_individual = contact_force_data.get('Cycloid1-Pins_individual')
cycloid2_individual = contact_force_data.get('Cycloid2-Pins_individual')
if (cycloid1_individual is not None and cycloid1_individual['forces'].size > 0) or \
   (cycloid2_individual is not None and cycloid2_individual['forces'].size > 0):
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(1, 1, 1)
    pin_count = 0
    if cycloid1_individual is not None:
        time = cycloid1_individual['time']
        forces = cycloid1_individual['forces']
        for pin_idx in range(forces.shape[1]):
            force = forces[:, pin_idx]
            max_force = float(np.max(force))
            if max_force > 0.1:
                ax.plot(time, force, linewidth=2, alpha=0.8,
                        label=f'Cyc1-Pin{pin_idx} (max={max_force:.1f}N)')
                pin_count += 1
    if cycloid2_individual is not None:
        time = cycloid2_individual['time']
        forces = cycloid2_individual['forces']
        for pin_idx in range(forces.shape[1]):
            force = forces[:, pin_idx]
            max_force = float(np.max(force))
            if max_force > 0.1:
                ax.plot(time, force, linewidth=2, alpha=0.8, linestyle='--',
                        label=f'Cyc2-Pin{pin_idx} (max={max_force:.1f}N)')
                pin_count += 1
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Contact Force (N)', fontsize=11)
    ax.set_title(f'All Active Pin Contact Forces ({pin_count} pins)', fontsize=12, fontweight='bold')
    ax.legend(fontsize=8, ncol=2, loc='best')
    ax.grid(True, alpha=0.3)
    save_plot(fig, 'pin_contact_forces.png')

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

# =================== 批量模式结果保存 ===================
if BATCH_MODE:
    print("\n" + "="*60)
    print("计算扭转刚度和迟滞特性...")
    print("="*60)

    def calculate_stiffness_batch(data_input_rot, data_output_rot):
        """计算扭转刚度和迟滞特性（批量模式）"""
        if data_input_rot is None or data_output_rot is None:
            print("警告：缺少输入或输出旋转数据")
            return None, None, None, None

        # 提取Z轴旋转角度
        def extract_z_angle(data_rot):
            angles = []
            for i in range(len(data_rot)):
                R = np.array([[data_rot[i,1], data_rot[i,2], data_rot[i,3]],
                              [data_rot[i,4], data_rot[i,5], data_rot[i,6]],
                              [data_rot[i,7], data_rot[i,8], data_rot[i,9]]])
                angle_z = np.arctan2(R[1,0], R[0,0])
                angles.append(angle_z)
            return np.unwrap(np.array(angles))

        time_output = data_output_rot[:, 0]
        angle_output = extract_z_angle(data_output_rot)

        # 输出轴角度转换为arcmin
        angle_output_arcmin = angle_output * (180/np.pi) * 60

        # 对应的扭矩值
        torque_values = np.array([torque_multi_step(outputTorqueZ, float(t)) for t in time_output])

        # 计算扭转刚度（线性拟合加载段）
        loading_mask = torque_values > 0
        if np.sum(loading_mask) > 10:
            torque_loading = torque_values[loading_mask]
            angle_loading = angle_output_arcmin[loading_mask]

            coeffs = np.polyfit(torque_loading, angle_loading, 1)
            stiffness_slope = 1.0 / coeffs[0] if abs(coeffs[0]) > 1e-10 else np.nan

            # 计算R²
            angle_pred = np.polyval(coeffs, torque_loading)
            ss_res = np.sum((angle_loading - angle_pred)**2)
            ss_tot = np.sum((angle_loading - np.mean(angle_loading))**2)
            r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else np.nan
        else:
            stiffness_slope = np.nan
            r_squared = np.nan

        # 计算回程间隙
        lost_motion = np.max(angle_output_arcmin) - np.min(angle_output_arcmin)

        # 计算迟滞面积
        try:
            from scipy.integrate import simpson as simps_func
        except ImportError:
            from scipy.integrate import simps as simps_func

        try:
            hysteresis_area = abs(simps_func(angle_output_arcmin, x=torque_values))
        except:
            hysteresis_area = 0.0

        metrics = {
            'torsional_stiffness_Nm_per_arcmin': float(stiffness_slope) if not np.isnan(stiffness_slope) else 0.0,
            'stiffness_r_squared': float(r_squared) if not np.isnan(r_squared) else 0.0,
            'lost_motion_arcmin': float(lost_motion),
            'hysteresis_area': float(hysteresis_area),
            'max_angle_arcmin': float(np.max(angle_output_arcmin)),
            'min_angle_arcmin': float(np.min(angle_output_arcmin)),
        }

        return metrics, time_output, angle_output_arcmin, torque_values

    # 计算刚度
    stiffness_metrics, time_common, angle_output, torque_values = calculate_stiffness_batch(
        data_input_rot, data_output_rot
    )

    if stiffness_metrics:
        print("\n刚度计算结果:")
        print(f"  扭转刚度: {stiffness_metrics['torsional_stiffness_Nm_per_arcmin']:.2f} N·m/arcmin")
        print(f"  回程间隙: {stiffness_metrics['lost_motion_arcmin']:.4f} arcmin")
        print(f"  迟滞面积: {stiffness_metrics['hysteresis_area']:.2f} N·m·arcmin")
        print(f"  最大角度: {stiffness_metrics['max_angle_arcmin']:.2f} arcmin")

        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)

        # 保存扭矩-角度曲线
        curve_data = np.column_stack([time_common, torque_values, angle_output])
        np.savetxt(f'{output_dir}/torque_angle_curve.txt', curve_data,
                   header='time torque_Nm angle_arcmin', comments='')

        # 保存指标（供批量脚本读取）
        with open(f'{output_dir}/stiffness_metrics.txt', 'w', encoding='utf-8') as f:
            f.write("RV减速器刚度分析结果\n")
            f.write("="*50 + "\n")
            f.write(f"参数设置:\n")
            f.write(f"  偏心圆半径误差: {eccentric_error*1e6:.2f} μm\n")
            f.write(f"  偏心距误差: {eccentricity_error*1e6:.2f} μm\n")
            f.write(f"  偏心角误差: {np.rad2deg(angle_error)*3600:.1f} arcsec\n")
            f.write(f"  轴承间隙调整: {bearing_clearance_adjust*1e6:.2f} μm\n")
            f.write(f"\n刚度指标:\n")
            f.write(f"  扭转刚度: {stiffness_metrics['torsional_stiffness_Nm_per_arcmin']:.2f} N·m/arcmin\n")
            f.write(f"  stiffness_r_squared: {stiffness_metrics['stiffness_r_squared']:.4f}\n")
            f.write(f"  回程间隙: {stiffness_metrics['lost_motion_arcmin']:.4f} arcmin\n")
            f.write(f"  迟滞面积: {stiffness_metrics['hysteresis_area']:.2f} N·m·arcmin\n")
            f.write(f"  最大角度: {stiffness_metrics['max_angle_arcmin']:.2f} arcmin\n")
            f.write(f"  最小角度: {stiffness_metrics['min_angle_arcmin']:.2f} arcmin\n")

        print(f"\n数据已保存至: {output_dir}")
        print("="*60)
    else:
        print("警告：无法计算刚度指标")
