import sys, os

sys.exudynFast = True          # 让 exudyn 优先加载 fast 版（依赖 AVX2）
sys.exudynCPUhasAVX2 = True 

import numpy as np

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
from exudyn.itemInterface import ObjectContactCircleCircle
from exudyn.machines import GetBallBearingData, CreateBallBearing

useGraphics = True
SC = exu.SystemContainer()
mbs = SC.AddSystem()

# Contact visualization toggles for consistent arrow output
SHOW_CONTACT_GEOMETRY = True
CONTACT_FORCE_SCALE = 3e-3  # enlarge contact force arrows for visibility
SHOW_CONTACT_FORCE_VALUES = False
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


# =================== 参数（与原脚本一致） ===================
z_b = 40
z_g = z_b - 1
e = 1.3
delta_e = 0
R_z = 64
r_z = 3

second_delta_rp = 0
second_delta_r_rp = 0
second_delta = 0
second_zp = z_b
second_a = e + delta_e
second_rp = R_z
second_r_rp = r_z

r_crank_main = 10
r_crank_eccentric = 12
L_crank_main1 = 20
L_crank_eccentric1 = 15
L_crank_middle = 5
L_crank_eccentric2 = 15
L_crank_main2 = 20
eccentric_offset = e

z_eccentric1 = L_crank_eccentric1 / 2
z_eccentric2 = L_crank_eccentric1 + L_crank_middle + L_crank_eccentric2 / 2

# 滚针轴承参数
r_needle = 2  # 滚针半径
n_needles = 16  # 每个轴承的滚针数量
bearing_clearance = 0.002  # 轴承径向间隙（很小但避免初始穿透）
r_needle_pitch = r_crank_eccentric + r_needle + bearing_clearance  # 滚针节圆半径
r_hole = r_needle_pitch + r_needle + bearing_clearance  # 摆线轮孔半径

r_pin_shell = R_z - 0.015
r_pin_hole = r_z + 0.01
thickness_shell = 8

R_shell_outer = R_z + 12
R_shell_inner = R_z - 12

# 三曲柄轴配置参数
n_cranks = 3  # 曲柄轴数量
crank_distribution_radius = 30.0  # 曲柄轴分布圆半径（增大以更往外扩）

print("="*60)
print("三曲柄轴双片摆线针轮减速器系统（带滚针轴承）")
print("="*60)
print(f"曲柄轴数量: {n_cranks}")
print(f"曲柄轴分布圆半径: {crank_distribution_radius} mm")
print(f"偏心轴外圆半径: {r_crank_eccentric} mm")
print(f"摆线轮孔半径: {r_hole:.3f} mm")
print(f"滚针半径: {r_needle} mm")
print(f"每个轴承滚针数: {n_needles}")
print(f"滚针节圆半径: {r_needle_pitch:.3f} mm")
print(f"轴承径向间隙: {bearing_clearance} mm (每侧)")
print(f"偏心轴-滚针名义间隙: {r_needle_pitch - r_crank_eccentric - r_needle:.3f} mm")
print(f"滚针-孔壁名义间隙: {r_hole - r_needle_pitch - r_needle:.3f} mm")
print(f"针齿半径: {r_z} mm")
print(f"针齿壳孔半径: {r_pin_hole} mm")
print(f"针齿-孔间隙: {r_pin_hole - r_z} mm")
print(f"针齿壳孔分布圆半径: {r_pin_shell} mm")
print(f"第一片摆线轮Z位置: {z_eccentric1} mm")
print(f"第二片摆线轮Z位置: {z_eccentric2} mm")
print(f"两片摆线轮相位差: 180°")
print(f"每个摆线轮孔数: {n_cranks} (与曲柄轴对应)")

contactStiffness_tooth = 1e7
contactDamping_tooth = 1e3
contactStiffness_hole = 1e7
contactDamping_hole = 1e3
contactStiffness_pin_hole = 1e7
contactDamping_pin_hole = 1e3

# 法兰轴承参数
r_flange_shaft = r_crank_main  # 法兰孔与曲柄主轴接触（主轴半径）
r_flange_needle = 2.0  # 法兰轴承滚针半径（比摆线轮轴承小）
n_flange_needles = 10  # 每个法兰轴承的滚针数量
flange_bearing_clearance = -0.002
r_flange_needle_pitch = r_flange_shaft + r_flange_needle + flange_bearing_clearance
r_flange_hole = r_flange_needle_pitch + r_flange_needle + flange_bearing_clearance  # 法兰孔半径

oGround = mbs.CreateGround()

def _cycloid_xy_from_phi(phi_array, phi_h):
    phi_array = np.asarray(phi_array)
    second_k1_pie = second_a * second_zp / (second_rp - second_delta_rp)
    second_i_H = second_zp / (second_zp - 1)
    phi_g = phi_h / z_g

    cos_phi = np.cos(phi_array)
    second_s_pie = 1 + second_k1_pie**2 - 2 * second_k1_pie * cos_phi
    sqrt_s = np.sqrt(second_s_pie)

    sin_term1 = np.sin((1 - second_i_H) * phi_array - second_delta)
    cos_term1 = np.cos((1 - second_i_H) * phi_array - second_delta)
    sin_term2 = np.sin(second_i_H * phi_array + second_delta)
    cos_term2 = np.cos(second_i_H * phi_array + second_delta)

    common_factor = (second_rp - second_delta_rp) - (second_r_rp + second_delta_r_rp) / sqrt_s
    x_local = -common_factor * sin_term1 - second_a / (second_rp - second_delta_rp) * ((second_rp - second_delta_rp) - second_zp * (second_r_rp + second_delta_r_rp) / sqrt_s) * sin_term2
    y_local = common_factor * cos_term1 - second_a / (second_rp - second_delta_rp) * ((second_rp - second_delta_rp) - second_zp * (second_r_rp + second_delta_r_rp) / sqrt_s) * cos_term2

    cos_g = np.cos(phi_g)
    sin_g = np.sin(phi_g)
    x_rot = x_local * cos_g - y_local * sin_g
    y_rot = x_local * sin_g + y_local * cos_g
    return x_rot, y_rot


def GenerateCycloidProfile(phi_h=0, n_points=1000, *, equal_arc_length=True, refinement_factor=20, max_iter=6):
    """生成摆线轮剖面坐标。

    Parameters
    ----------
    phi_h : float
        相位偏移。
    n_points : int
        目标点数。
    equal_arc_length : bool, optional
        是否执行等弧长重采样，默认 True。
    refinement_factor : int, optional
        初始密集采样倍数，用于建立弧长表。
    max_iter : int, optional
        迭代寻找等弧长参数时的最大迭代次数。
    """

    phi_period = 2 * (second_zp - 1) * np.pi

    if not equal_arc_length:
        phi_uniform = np.linspace(0.0, phi_period, n_points, endpoint=False)
        x_vals, y_vals = _cycloid_xy_from_phi(phi_uniform, phi_h)
        return x_vals.tolist(), y_vals.tolist()

    n_dense = max(refinement_factor * n_points, n_points * 4)
    phi_dense = np.linspace(0.0, phi_period, n_dense + 1)
    x_dense, y_dense = _cycloid_xy_from_phi(phi_dense, phi_h)

    dx = np.diff(x_dense)
    dy = np.diff(y_dense)
    seg_len = np.hypot(dx, dy)
    s_dense = np.concatenate(([0.0], np.cumsum(seg_len)))
    total_length = s_dense[-1]

    # 数值速度，用于牛顿修正
    speed_dense = np.hypot(np.gradient(x_dense, phi_dense), np.gradient(y_dense, phi_dense))
    speed_dense = np.maximum(speed_dense, 1e-12)

    target_s = np.linspace(0.0, total_length, n_points, endpoint=False)
    phi_targets = np.interp(target_s, s_dense, phi_dense)

    for idx in range(n_points):
        phi_val = phi_targets[idx]
        s_target = target_s[idx]
        for _ in range(max_iter):
            s_current = np.interp(phi_val, phi_dense, s_dense)
            speed = np.interp(phi_val, phi_dense, speed_dense)
            delta = (s_current - s_target) / speed
            if abs(delta) < 1e-10:
                break
            phi_val = np.clip(phi_val - delta, 0.0, phi_period)
        phi_targets[idx] = phi_val

    x_resampled, y_resampled = _cycloid_xy_from_phi(phi_targets, phi_h)
    return x_resampled.tolist(), y_resampled.tolist()

x_cycloid1, y_cycloid1 = GenerateCycloidProfile(phi_h=0, n_points=10000)
x_cycloid2, y_cycloid2 = GenerateCycloidProfile(phi_h=np.pi, n_points=10000)

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

segmentsData_tooth1, nSeg_tooth1 = CreateToothSegments(x_cycloid1, y_cycloid1)
segmentsData_tooth2, nSeg_tooth2 = CreateToothSegments(x_cycloid2, y_cycloid2)
segment_lengths_tooth1 = ComputeSegmentLengths(segmentsData_tooth1)
segment_lengths_tooth2 = ComputeSegmentLengths(segmentsData_tooth2)


def create_circle_contact(mbs, *, name, markerA, markerB,
                          radiusA, radiusB,
                          stiffness, damping,
                          friction=0.0,
                          active=True,
                          group=None):
    node = mbs.AddNode(NodeGenericData(
        initialCoordinates=[0., 0., 0., 0., 0., 0.],
        numberOfDataCoordinates=6
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
# 曲柄轴参数
mass_crank = 80.0  # 质量乘以10倍
I_crank = RigidBodyInertia(
    mass=mass_crank,
    inertiaTensor=np.array([[1500, 0, 0],[0, 1500, 0],[0, 0, 3000]]),  # 转动惯量也乘以10倍
    com=[0, 0, 0]
)

# 中心输入轴（仅绕Z轴旋转，位置靠近输入法兰）
mass_input_shaft = 20.0
I_input_shaft = RigidBodyInertia(
    mass=mass_input_shaft,
    inertiaTensor=np.array([[200, 0, 0],[0, 200, 0],[0, 0, 400]]),
    com=[0, 0, 0]
)

input_shaft_graphics = [
    graphics.Cylinder(pAxis=[0, 0, -15], vAxis=[0, 0, 30], radius=10.0, color=graphics.color.steelblue, nTiles=32),
    graphics.Basis(length=10)
]

# 输入轴位置调整到靠近输入法兰的位置
oInputShaft = mbs.CreateRigidBody(
    referencePosition=[0, 0, 60],
    inertia=I_input_shaft,
    gravity=[0, 0, 0],
    graphicsDataList=input_shaft_graphics,
    nodeType=exu.NodeType.RotationRxyz
)

# 输入轴与地面之间仅保留绕Z轴的转动自由度
mbs.CreateRevoluteJoint(
    bodyNumbers=[oGround, oInputShaft],
    position=[0, 0, 60],
    axis=[0, 0, 1],
    show=False,
    axisRadius=0.6,
    axisLength=4.0
)

omega_input_target = 2  # 输入恒定转速（rad/s）
outputTorqueZ = 100.0  # 输出法兰恒定扭矩（N·m）
print(f"输入恒定转速: {omega_input_target} rad/s = {omega_input_target*60/(2*np.pi):.2f} RPM")
print(f"输出法兰恒定扭矩: {outputTorqueZ} N·m")


def crank_input_angle_offset(mbs, t, itemIndex, currentOffset):
    """位置层级约束的目标角度 = ω * t"""
    return omega_input_target * t

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
    angle_crank = 2 * np.pi * i_crank / n_cranks  # 120度均布
    x_crank = crank_distribution_radius * np.cos(angle_crank)
    y_crank = crank_distribution_radius * np.sin(angle_crank)
    
    # 每个曲柄轴的偏心方向（第一片正向，第二片反向）
    crankshaft_graphics = [
        graphics.Cylinder(pAxis=[0, 0, -L_crank_main1], vAxis=[0, 0, L_crank_main1], 
                         radius=r_crank_main, color=graphics.color.darkgrey, nTiles=32),
        graphics.Cylinder(pAxis=[eccentric_offset, 0, 0], vAxis=[0, 0, L_crank_eccentric1], 
                         radius=r_crank_eccentric, color=graphics.color.red, nTiles=32),
        graphics.Cylinder(pAxis=[0, 0, L_crank_eccentric1], vAxis=[0, 0, L_crank_middle], 
                         radius=r_crank_main, color=graphics.color.darkgrey, nTiles=32),
        graphics.Cylinder(pAxis=[-eccentric_offset, 0, L_crank_eccentric1 + L_crank_middle], 
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
    
    # 采用部分约束的通用关节：释放 XY 平移，只限制 Z 平移与 X/Y 倾转
    mGroundCrank = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[x_crank, y_crank, 0]))
    mCrank = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank, localPosition=[0, 0, 0]))
    mbs.AddObject(GenericJoint(
        markerNumbers=[mGroundCrank, mCrank],
        constrainedAxes=[0, 0, 1, 1, 1, 0],
        rotationMarker0=np.eye(3),
        rotationMarker1=np.eye(3),
        visualization=VObjectJointGeneric(axesRadius=0.5, axesLength=1.0)
    ))
    
    # 用旋转弹簧连接曲柄轴与输入轴，提供弹性约束
    mInputShaftCrank = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oInputShaft, localPosition=[0, 0, 0]))
    mCrankSpring = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank, localPosition=[0, 0, 0]))
    
    # 添加旋转弹簧-阻尼器，约束绕Z轴旋转
    mbs.AddObject(ObjectConnectorTorsionalSpringDamper(
        markerNumbers=[mInputShaftCrank, mCrankSpring],
        stiffness=1e5,  # 旋转刚度 (N·m/rad)
        damping=100.0,  # 旋转阻尼 (N·m·s/rad)
        visualization=VObjectConnectorTorsionalSpringDamper(show=False)
    ))
    
    # 创建偏心段标记
    mEcc1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank, 
                                          localPosition=[eccentric_offset, 0, z_eccentric1]))
    mEcc2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank, 
                                          localPosition=[-eccentric_offset, 0, z_eccentric2]))
    crankshaft_markers_ecc1.append(mEcc1)
    crankshaft_markers_ecc2.append(mEcc2)

print(f"创建了 {n_cranks} 个曲柄轴，分布圆半径: {crank_distribution_radius} mm")

# =================== 输入输出法兰系统 ===================
# 法兰位置
z_input_flange = -L_crank_main1 / 2  # 输入法兰在曲柄轴下方
z_output_flange = L_crank_eccentric1 + L_crank_middle + L_crank_eccentric2 + L_crank_main2 / 2  # 输出法兰在曲柄轴上方

# 法兰参数
mass_flange = 50.0  # 质量乘以10倍
r_flange_outer = crank_distribution_radius + 15  # 法兰外半径
r_flange_inner = crank_distribution_radius - 10  # 法兰内半径
thickness_flange = 5.0

I_flange = RigidBodyInertia(
    mass=mass_flange,
    inertiaTensor=np.array([[3000, 0, 0], [0, 3000, 0], [0, 0, 6000]]),  # 转动惯量也乘以10倍
    com=[0, 0, 0]
)

# 输入法兰图形
input_flange_graphics = []
n_flange_circle_points = 60
# 外圆
for i in range(n_flange_circle_points):
    angle1 = i * 2 * np.pi / n_flange_circle_points
    angle2 = (i + 1) * 2 * np.pi / n_flange_circle_points
    x1 = r_flange_outer * np.cos(angle1)
    y1 = r_flange_outer * np.sin(angle1)
    x2 = r_flange_outer * np.cos(angle2)
    y2 = r_flange_outer * np.sin(angle2)
    input_flange_graphics.append({'type': 'Line', 'color': [0.3, 0.3, 0.8, 1], 
                                  'data': [x1, y1, 0, x2, y2, 0]})
# 内圆
for i in range(n_flange_circle_points):
    angle1 = i * 2 * np.pi / n_flange_circle_points
    angle2 = (i + 1) * 2 * np.pi / n_flange_circle_points
    x1 = r_flange_inner * np.cos(angle1)
    y1 = r_flange_inner * np.sin(angle1)
    x2 = r_flange_inner * np.cos(angle2)
    y2 = r_flange_inner * np.sin(angle2)
    input_flange_graphics.append({'type': 'Line', 'color': [0.3, 0.3, 0.8, 1], 
                                  'data': [x1, y1, 0, x2, y2, 0]})
# 3个孔
for i_hole in range(n_cranks):
    angle_hole = 2 * np.pi * i_hole / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    for i in range(40):
        angle1 = i * 2 * np.pi / 40
        angle2 = (i + 1) * 2 * np.pi / 40
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
    angle1 = i * 2 * np.pi / n_flange_circle_points
    angle2 = (i + 1) * 2 * np.pi / n_flange_circle_points
    x1 = r_flange_outer * np.cos(angle1)
    y1 = r_flange_outer * np.sin(angle1)
    x2 = r_flange_outer * np.cos(angle2)
    y2 = r_flange_outer * np.sin(angle2)
    output_flange_graphics.append({'type': 'Line', 'color': [0.8, 0.3, 0.3, 1], 
                                   'data': [x1, y1, 0, x2, y2, 0]})
# 内圆
for i in range(n_flange_circle_points):
    angle1 = i * 2 * np.pi / n_flange_circle_points
    angle2 = (i + 1) * 2 * np.pi / n_flange_circle_points
    x1 = r_flange_inner * np.cos(angle1)
    y1 = r_flange_inner * np.sin(angle1)
    x2 = r_flange_inner * np.cos(angle2)
    y2 = r_flange_inner * np.sin(angle2)
    output_flange_graphics.append({'type': 'Line', 'color': [0.8, 0.3, 0.3, 1], 
                                   'data': [x1, y1, 0, x2, y2, 0]})
# 3个孔
for i_hole in range(n_cranks):
    angle_hole = 2 * np.pi * i_hole / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    for i in range(40):
        angle1 = i * 2 * np.pi / 40
        angle2 = (i + 1) * 2 * np.pi / 40
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
                           constrainedAxes=[0, 0, 1, 1, 1, 0],  
                           visualization=VObjectJointGeneric(axesRadius=0.5, axesLength=1)))

# 创建输出法兰（不固定在地面）
oOutputFlange = mbs.CreateRigidBody(
    referencePosition=[0, 0, z_output_flange],
    inertia=I_flange,
    gravity=[0, 0, 0],
    graphicsDataList=output_flange_graphics,
    nodeType=exu.NodeType.RotationRxyz
)

# 输入法兰和输出法兰之间刚性固定（形成一个整体）
mInputFlangeConnect = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oInputFlange, localPosition=[0, 0, 0]))
mOutputFlangeConnect = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oOutputFlange, localPosition=[0, 0, 0]))
mbs.AddObject(GenericJoint(markerNumbers=[mInputFlangeConnect, mOutputFlangeConnect],
                           constrainedAxes=[1, 1, 0, 1, 1, 1],  # 完全固定
                           visualization=VObjectJointGeneric(axesRadius=0.8, axesLength=1.5, show=True)))

# 输出法兰恒定扭矩控制
mOutputFlangeLoad = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oOutputFlange, localPosition=[0, 0, 0]))
mbs.AddLoad(LoadTorqueVector(markerNumber=mOutputFlangeLoad, loadVector=[0, 0, outputTorqueZ]))

# 创建法兰孔的标记点
input_flange_hole_markers = []
output_flange_hole_markers = []
crank_shaft_input_markers = []  # 曲柄轴在输入法兰处的标记
crank_shaft_output_markers = []  # 曲柄轴在输出法兰处的标记

for i_crank in range(n_cranks):
    angle_hole = 2 * np.pi * i_crank / n_cranks
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
    
    # 曲柄轴在输入法兰处的标记（主轴段）
    mCrankInput = mbs.AddMarker(MarkerBodyRigid(bodyNumber=crankshaft_bodies[i_crank], 
                                                 localPosition=[0, 0, z_input_flange]))
    crank_shaft_input_markers.append(mCrankInput)
    
    # 曲柄轴在输出法兰处的标记（主轴段）
    mCrankOutput = mbs.AddMarker(MarkerBodyRigid(bodyNumber=crankshaft_bodies[i_crank], 
                                                  localPosition=[0, 0, z_output_flange]))
    crank_shaft_output_markers.append(mCrankOutput)

print(f"创建了输入输出法兰系统")
print(f"  输入法兰位置: Z = {z_input_flange:.2f} mm")
print(f"  输出法兰位置: Z = {z_output_flange:.2f} mm")
print(f"  两法兰之间: 刚性固定（形成一体）")
print(f"  法兰系统相对地面: 可绕Z轴旋转")
print(f"  法兰孔半径: {r_flange_hole:.3f} mm")
print(f"  法兰孔分布圆半径: {crank_distribution_radius} mm")
print(f"  法兰与曲柄轴通过滚针轴承连接")

# =================== 摆线轮（带3个均布孔） ===================
mass_cycloid = 80.0  # 质量乘以10倍
I_cycloid = RigidBodyInertia(mass=mass_cycloid, inertiaTensor=np.array([[5000,0,0],[0,5000,0],[0,0,10000]]), com=[0,0,0])  # 转动惯量也乘以10倍

# 第一片摆线轮
cycloid1_graphics = []
# 外齿廓
for i in range(len(x_cycloid1)):
    i_next = (i + 1) % len(x_cycloid1)
    cycloid1_graphics.append({'type':'Line','color':[0.9,0.1,0.1,1],'data':[x_cycloid1[i],y_cycloid1[i],0,x_cycloid1[i_next],y_cycloid1[i_next],0]})

# 3个均布的孔
n_hole_points = 40
for i_hole in range(n_cranks):
    angle_hole = 2 * np.pi * i_hole / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    for i in range(n_hole_points):
        angle1 = i * 2 * np.pi / n_hole_points
        angle2 = (i + 1) * 2 * np.pi / n_hole_points
        x1 = x_hole_center + r_hole * np.cos(angle1)
        y1 = y_hole_center + r_hole * np.sin(angle1)
        x2 = x_hole_center + r_hole * np.cos(angle2)
        y2 = y_hole_center + r_hole * np.sin(angle2)
        cycloid1_graphics.append({'type':'Line','color':[0.1,0.7,0.1,1],'data':[x1,y1,0,x2,y2,0]})

# 第一片摆线轮初始位置：与第一个曲柄轴的偏心段对齐
oCycloid1 = mbs.CreateRigidBody(referencePosition=[eccentric_offset, 0, z_eccentric1], inertia=I_cycloid, gravity=[0,0,0], graphicsDataList=cycloid1_graphics)
mCycloid1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid1, localPosition=[0,0,0]))
mGroundCycloid1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[eccentric_offset, 0, z_eccentric1]))
mbs.AddObject(GenericJoint(markerNumbers=[mGroundCycloid1, mCycloid1], constrainedAxes=[0,0,1,1,1,0], visualization=VObjectJointGeneric(axesRadius=0.5, axesLength=1)))

# 第二片摆线轮
cycloid2_graphics = []
# 外齿廓
for i in range(len(x_cycloid2)):
    i_next = (i + 1) % len(x_cycloid2)
    cycloid2_graphics.append({'type':'Line','color':[0.6,0.1,0.9,1],'data':[x_cycloid2[i],y_cycloid2[i],0,x_cycloid2[i_next],y_cycloid2[i_next],0]})

# 3个均布的孔
for i_hole in range(n_cranks):
    angle_hole = 2 * np.pi * i_hole / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    for i in range(n_hole_points):
        angle1 = i * 2 * np.pi / n_hole_points
        angle2 = (i + 1) * 2 * np.pi / n_hole_points
        x1 = x_hole_center + r_hole * np.cos(angle1)
        y1 = y_hole_center + r_hole * np.sin(angle1)
        x2 = x_hole_center + r_hole * np.cos(angle2)
        y2 = y_hole_center + r_hole * np.sin(angle2)
        cycloid2_graphics.append({'type':'Line','color':[0.1,0.7,0.9,1],'data':[x1,y1,0,x2,y2,0]})

# 第二片摆线轮初始位置：与第二个曲柄轴的偏心段对齐
oCycloid2 = mbs.CreateRigidBody(referencePosition=[-eccentric_offset, 0, z_eccentric2], inertia=I_cycloid, gravity=[0,0,0], graphicsDataList=cycloid2_graphics)
mCycloid2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid2, localPosition=[0,0,0]))
mGroundCycloid2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[-eccentric_offset, 0, z_eccentric2]))
mbs.AddObject(GenericJoint(markerNumbers=[mGroundCycloid2, mCycloid2], constrainedAxes=[0,0,1,1,1,0], visualization=VObjectJointGeneric(axesRadius=0.5, axesLength=1)))

# 创建摆线轮孔的标记（用于轴承连接）
cycloid1_hole_markers = []
cycloid2_hole_markers = []
for i_hole in range(n_cranks):
    angle_hole = 2 * np.pi * i_hole / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    
    mHole1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid1, localPosition=[x_hole_center, y_hole_center, 0]))
    mHole2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid2, localPosition=[x_hole_center, y_hole_center, 0]))
    cycloid1_hole_markers.append(mHole1)
    cycloid2_hole_markers.append(mHole2)

# 注意：输出法兰不与摆线轮直接连接，仅通过法兰轴承与曲柄轴连接


# 针齿壳（可视部分）
z_shell = (z_eccentric1 + z_eccentric2) / 2
shell_graphics_list = []
n_shell_outer_points = 120
for i in range(n_shell_outer_points):
    angle1 = i * 2 * np.pi / n_shell_outer_points
    angle2 = (i + 1) * 2 * np.pi / n_shell_outer_points
    x1 = R_shell_outer * np.cos(angle1); y1 = R_shell_outer * np.sin(angle1)
    x2 = R_shell_outer * np.cos(angle2); y2 = R_shell_outer * np.sin(angle2)
    shell_graphics_list.append({'type':'Line','color':[0.5,0.5,0.5,1],'data':[x1,y1,z_shell,x2,y2,z_shell]})
for i in range(n_shell_outer_points):
    angle1 = i * 2 * np.pi / n_shell_outer_points
    angle2 = (i + 1) * 2 * np.pi / n_shell_outer_points
    x1 = R_shell_inner * np.cos(angle1); y1 = R_shell_inner * np.sin(angle1)
    x2 = R_shell_inner * np.cos(angle2); y2 = R_shell_inner * np.sin(angle2)
    shell_graphics_list.append({'type':'Line','color':[0.5,0.5,0.5,1],'data':[x1,y1,z_shell,x2,y2,z_shell]})

n_pins_show = 40
n_hole_circle_points = 48
for i in range(n_pins_show):
    angle_pin = 2 * np.pi * (i + 0.25) / n_pins_show
    x_hole_center = r_pin_shell * np.sin(angle_pin)
    y_hole_center = r_pin_shell * np.cos(angle_pin)
    for j in range(n_hole_circle_points):
        angle1 = j * 2 * np.pi / n_hole_circle_points
        angle2 = (j + 1) * 2 * np.pi / n_hole_circle_points
        x1 = x_hole_center + r_pin_hole * np.cos(angle1)
        y1 = y_hole_center + r_pin_hole * np.sin(angle1)
        x2 = x_hole_center + r_pin_hole * np.cos(angle2)
        y2 = y_hole_center + r_pin_hole * np.sin(angle2)
        shell_graphics_list.append({'type':'Line','color':[0.9,0.5,0.1,1],'data':[x1,y1,z_shell,x2,y2,z_shell]})

mbs.CreateGround(graphicsDataList=shell_graphics_list)

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

mass_pin = 3.0  # 质量乘以10倍
inertia_pin = InertiaCylinder(density=mass_pin*100, length=0.1, outerRadius=r_z, axis=2)

for i in range(n_pins_show):
    angle = 2 * np.pi * (i + 0.25) / n_pins_show
    x_pin = r_pin_shell * np.sin(angle)
    y_pin = r_pin_shell * np.cos(angle)
    z_pin = z_shell
    oPin = mbs.CreateRigidBody(referencePosition=[x_pin, y_pin, z_pin], inertia=inertia_pin, gravity=[0,0,0], graphicsDataList=[graphics.Cylinder(pAxis=[0,0,-10], vAxis=[0,0,20], radius=r_z, color=graphics.color.steelblue, nTiles=32)])
    mGroundPinPlane = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[x_pin, y_pin, z_pin]))
    mPinPlane = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oPin, localPosition=[0, 0, 0]))
    mbs.AddObject(GenericJoint(markerNumbers=[mGroundPinPlane, mPinPlane], constrainedAxes=[0,0,1,1,1,0], visualization=VObjectJointGeneric(axesRadius=0.15, axesLength=0.4, show=False)))
    mPin = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oPin, localPosition=[0,0,0]))
    pin_markers.append(mPin)
    pin_radii.append(r_z)
    pin_bodies.append(oPin)
    hole_segments, nSeg_hole = CreatePinHoleSegments(r_pin_hole, [x_pin, y_pin], nPoints=60)
    hole_lengths = ComputeSegmentLengths(hole_segments)
    pin_hole_segments.append((hole_segments, nSeg_hole, exu.MatrixContainer(hole_segments), hole_lengths))

# 外齿廓-针齿接触（仍用内置 ContactCurveCircles）
initialCoords_tooth1 = []
for _ in range(nSeg_tooth1):
    initialCoords_tooth1.extend([-1.0, 0.0, 0.0])
nGenericData_tooth1 = mbs.AddNode(NodeGenericData(initialCoordinates=initialCoords_tooth1, numberOfDataCoordinates=3 * nSeg_tooth1))
objCycloid1Pins = mbs.AddObject(ObjectContactCurveCircles(
    markerNumbers=[mCycloid1] + pin_markers,
    nodeNumber=nGenericData_tooth1,
    circlesRadii=pin_radii,
    segmentsData=exu.MatrixContainer(segmentsData_tooth1),
    contactStiffness=contactStiffness_tooth,
    contactDamping=contactDamping_tooth,
    visualization=VObjectContactCurveCircles(show=SHOW_CONTACT_GEOMETRY, color=graphics.color.blue)
))
_register_contact_info("CycloidPins", "CycloidPins-0", nGenericData_tooth1, objCycloid1Pins, segment_lengths_tooth1, contactStiffness_tooth, contactDamping_tooth)

initialCoords_tooth2 = []
for _ in range(nSeg_tooth2):
    initialCoords_tooth2.extend([-1.0, 0.0, 0.0])
nGenericData_tooth2 = mbs.AddNode(NodeGenericData(initialCoordinates=initialCoords_tooth2, numberOfDataCoordinates=3 * nSeg_tooth2))
objCycloid2Pins = mbs.AddObject(ObjectContactCurveCircles(
    markerNumbers=[mCycloid2] + pin_markers,
    nodeNumber=nGenericData_tooth2,
    circlesRadii=pin_radii,
    segmentsData=exu.MatrixContainer(segmentsData_tooth2),
    contactStiffness=contactStiffness_tooth,
    contactDamping=contactDamping_tooth,
    visualization=VObjectContactCurveCircles(show=SHOW_CONTACT_GEOMETRY, color=graphics.color.magenta)
))
_register_contact_info("CycloidPins", "CycloidPins-1", nGenericData_tooth2, objCycloid2Pins, segment_lengths_tooth2, contactStiffness_tooth, contactDamping_tooth)

cycloid_pin_contact_objects = [objCycloid1Pins, objCycloid2Pins]

# 滚针轴承参数
contactStiffness_bearing = 1e7
contactDamping_bearing = 1e3
mass_needle = 0.5  # 质量乘以10倍
inertia_needle = InertiaCylinder(density=mass_needle*100, length=0.1, outerRadius=r_needle, axis=2)

# 法兰轴承参数
contactStiffness_flange_bearing = 1e7
contactDamping_flange_bearing = 1e3
mass_flange_needle = 0.3  # 质量乘以10倍
inertia_flange_needle = InertiaCylinder(density=mass_flange_needle*100, length=0.1, outerRadius=r_flange_needle, axis=2)

# 使用 ObjectContactCircleCircle 替换圆-圆接触
for i_crank in range(n_cranks):
    angle_hole = 2 * np.pi * i_crank / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)

    for i_needle in range(n_needles):
        angle_needle = 2 * np.pi * i_needle / n_needles

        x_needle_1 = x_hole_center + eccentric_offset + r_needle_pitch * np.cos(angle_needle)
        y_needle_1 = y_hole_center + r_needle_pitch * np.sin(angle_needle)
        oNeedle1 = mbs.CreateRigidBody(
            referencePosition=[x_needle_1, y_needle_1, z_eccentric1],
            inertia=inertia_needle,
            gravity=[0, 0, 0],
            graphicsDataList=[graphics.Cylinder(
                pAxis=[0, 0, -2], vAxis=[0, 0, 4],
                radius=r_needle, color=graphics.color.darkgrey, nTiles=16)]
        )
        mGroundNeedle1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround,
                                                       localPosition=[x_needle_1, y_needle_1, z_eccentric1]))
        mNeedle1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle1, localPosition=[0, 0, 0]))
        mbs.AddObject(GenericJoint(markerNumbers=[mGroundNeedle1, mNeedle1],
                                   constrainedAxes=[0, 0, 1, 1, 1, 0],
                                   visualization=VObjectJointGeneric(axesRadius=0.1, axesLength=0.2, show=False)))

        create_circle_contact(
            mbs,
            name=f"CrankNeedle-{i_crank:02d}-{i_needle:02d}",
            markerA=crankshaft_markers_ecc1[i_crank],
            markerB=mNeedle1,
            radiusA=r_crank_eccentric,
            radiusB=r_needle,
            stiffness=contactStiffness_bearing,
            damping=contactDamping_bearing,
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
            group="NeedleCycloidHole"
        )

        x_needle_2 = x_hole_center - eccentric_offset + r_needle_pitch * np.cos(angle_needle)
        y_needle_2 = y_hole_center + r_needle_pitch * np.sin(angle_needle)
        oNeedle2 = mbs.CreateRigidBody(
            referencePosition=[x_needle_2, y_needle_2, z_eccentric2],
            inertia=inertia_needle,
            gravity=[0, 0, 0],
            graphicsDataList=[graphics.Cylinder(
                pAxis=[0, 0, -2], vAxis=[0, 0, 4],
                radius=r_needle, color=graphics.color.darkgrey, nTiles=16)]
        )
        mGroundNeedle2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround,
                                                       localPosition=[x_needle_2, y_needle_2, z_eccentric2]))
        mNeedle2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle2, localPosition=[0, 0, 0]))
        mbs.AddObject(GenericJoint(markerNumbers=[mGroundNeedle2, mNeedle2],
                                   constrainedAxes=[0, 0, 1, 1, 1, 0],
                                   visualization=VObjectJointGeneric(axesRadius=0.1, axesLength=0.2, show=False)))

        create_circle_contact(
            mbs,
            name=f"CrankNeedleB-{i_crank:02d}-{i_needle:02d}",
            markerA=crankshaft_markers_ecc2[i_crank],
            markerB=mNeedle2,
            radiusA=r_crank_eccentric,
            radiusB=r_needle,
            stiffness=contactStiffness_bearing,
            damping=contactDamping_bearing,
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
            group="NeedleCycloidHole"
        )

for i_crank in range(n_cranks):
    angle_hole = 2 * np.pi * i_crank / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)

    for i_needle in range(n_flange_needles):
        angle_needle = 2 * np.pi * i_needle / n_flange_needles
        x_needle = x_hole_center + r_flange_needle_pitch * np.cos(angle_needle)
        y_needle = y_hole_center + r_flange_needle_pitch * np.sin(angle_needle)

        oNeedleIn = mbs.CreateRigidBody(
            referencePosition=[x_needle, y_needle, z_input_flange],
            inertia=inertia_flange_needle,
            gravity=[0, 0, 0],
            graphicsDataList=[graphics.Cylinder(
                pAxis=[0, 0, -1.5], vAxis=[0, 0, 3],
                radius=r_flange_needle, color=graphics.color.grey, nTiles=16)]
        )
        mGroundNeedleIn = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround,
                                                        localPosition=[x_needle, y_needle, z_input_flange]))
        mNeedleIn = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedleIn, localPosition=[0, 0, 0]))
        mbs.AddObject(GenericJoint(markerNumbers=[mGroundNeedleIn, mNeedleIn],
                                   constrainedAxes=[0, 0, 1, 1, 1, 0],
                                   visualization=VObjectJointGeneric(axesRadius=0.1, axesLength=0.2, show=False)))

        create_circle_contact(
            mbs,
            name=f"InputFlangeShaft-{i_crank:02d}-{i_needle:02d}",
            markerA=crank_shaft_input_markers[i_crank],
            markerB=mNeedleIn,
            radiusA=r_flange_shaft,
            radiusB=r_flange_needle,
            stiffness=contactStiffness_flange_bearing,
            damping=contactDamping_flange_bearing,
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
            group="FlangeHole"
        )

        oNeedleOut = mbs.CreateRigidBody(
            referencePosition=[x_needle, y_needle, z_output_flange],
            inertia=inertia_flange_needle,
            gravity=[0, 0, 0],
            graphicsDataList=[graphics.Cylinder(
                pAxis=[0, 0, -1.5], vAxis=[0, 0, 3],
                radius=r_flange_needle, color=graphics.color.grey, nTiles=16)]
        )
        mGroundNeedleOut = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround,
                                                         localPosition=[x_needle, y_needle, z_output_flange]))
        mNeedleOut = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedleOut, localPosition=[0, 0, 0]))
        mbs.AddObject(GenericJoint(markerNumbers=[mGroundNeedleOut, mNeedleOut],
                                   constrainedAxes=[0, 0, 1, 1, 1, 0],
                                   visualization=VObjectJointGeneric(axesRadius=0.1, axesLength=0.2, show=False)))

        create_circle_contact(
            mbs,
            name=f"OutputFlangeShaft-{i_crank:02d}-{i_needle:02d}",
            markerA=crank_shaft_output_markers[i_crank],
            markerB=mNeedleOut,
            radiusA=r_flange_shaft,
            radiusB=r_flange_needle,
            stiffness=contactStiffness_flange_bearing,
            damping=contactDamping_flange_bearing,
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
            group="FlangeHole"
        )

print("创建了滚针和法兰轴承圆-圆接触（ObjectContactCircleCircle）")

# 为每个针齿创建对应的壳孔 marker（孔中心与针齿中心重合）
for idx_pin, (mPin, pin_body) in enumerate(zip(pin_markers, pin_bodies)):
    # 获取针齿的初始位置（孔中心应该在同一位置）
    angle = 2 * np.pi * (idx_pin + 0.25) / n_pins_show
    x_pin = r_pin_shell * np.sin(angle)
    y_pin = r_pin_shell * np.cos(angle)
    z_pin = z_shell
    
    # 为每个针齿创建一个壳孔 marker
    mGroundShellHole = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[x_pin, y_pin, z_pin]))
    
    create_circle_contact(
        mbs,
        name=f"PinShell-{idx_pin:02d}",
        markerA=mPin,
        markerB=mGroundShellHole,
        radiusA=r_z,
        radiusB=-r_pin_hole,
        stiffness=contactStiffness_pin_hole,
        damping=contactDamping_pin_hole,
        group="PinShell"
    )

print("创建了针齿-壳孔圆-圆接触（ObjectContactCircleCircle）")

# =================== 传感器/组装/仿真 ===================
mbs.CreateGround(graphicsDataList=[])

# 传感器（需在仿真前创建并 storeInternal 以记录数据）
sCycloid1Pos = mbs.AddSensor(SensorBody(bodyNumber=oCycloid1, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
sCycloid2Pos = mbs.AddSensor(SensorBody(bodyNumber=oCycloid2, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
sCrankRot    = mbs.AddSensor(SensorBody(bodyNumber=crankshaft_bodies[0], storeInternal=True, outputVariableType=exu.OutputVariableType.RotationMatrix))
sPin0Pos     = mbs.AddSensor(SensorBody(bodyNumber=pin_bodies[0], storeInternal=True, outputVariableType=exu.OutputVariableType.Position))

mbs.Assemble()

# _contact_step_report_state = {'count': 0}

# def _format_optional(value):
#     if value is None or not np.isfinite(value):
#         return 'n/a'
#     return f"{value:.6e}"

# def _report_contacts_per_step(mbs, t):
#     _contact_step_report_state['count'] += 1
#     step_index = _contact_step_report_state['count']
#     
#     # 常规输出节奏控制
#     should_print_step = (step_index <= 10) or (step_index % CONTACT_STEP_PRINT_INTERVAL == 0)
#
#     pending_lines = []
#     debug_lines = []
#
#     for group_name, infos in CONTACT_MONITOR_GROUPS.items():
#         if not infos:
#             continue
#         active_lines = []
#         for local_idx, info in enumerate(infos):
#             coords = np.asarray(mbs.GetNodeOutput(info['node'], exu.OutputVariableType.Coordinates), dtype=float)
#             if coords.size == 0:
#                 continue
#
#             # Handle both curve-based contacts (with segment_lengths) and circle-circle contacts
#             if 'segment_lengths' in info:
#                 # Curve-based contact (ObjectContactCurveCircles)
#                 segments = coords.reshape((-1, 3))
#                 if segments.shape[0] != info['segment_lengths'].shape[0]:
#                     continue
#
#                 circle_ids = segments[:, 0]
#                 gaps = segments[:, 1]
#                 vtangent = segments[:, 2]
#
#                 contact_mask = (circle_ids >= 0) & (gaps <= CONTACT_IDLE_GAP_LIMIT)
#                 active_indices = np.flatnonzero(contact_mask)
#                 active_count = int(active_indices.size)
#                 total_pairs = segments.shape[0]
#
#                 active_gaps = gaps[active_indices] if active_count else np.zeros((0,))
#                 lengths = info['segment_lengths'][active_indices] if active_count else np.zeros((0,))
#                 vt = vtangent[active_indices] if active_count else np.zeros((0,))
#
#                 # approximate normal force based purely on penalty term
#                 approx_forces = np.maximum(0.0, -(active_gaps * info['stiffness']) * lengths) if active_count else np.zeros((0,))
#             else:
#                 # Circle-circle contact (ObjectContactCircleCircle) - 6 coordinates: gap, gapVel, force, normal[3]
#                 if coords.size < 6:
#                     continue
#                 
#                 gap = coords[0]
#                 gap_vel = coords[1] 
#                 force = coords[2]
#                 
#                 # Single contact point
#                 contact_active = gap <= CONTACT_IDLE_GAP_LIMIT
#                 active_count = 1 if contact_active else 0
#                 total_pairs = 1
#                 
#                 # define arrays consistent with curve-case so downstream code can be unified
#                 active_gaps = np.array([gap]) if contact_active else np.zeros((0,))
#                 lengths = np.array([1.0]) if contact_active else np.zeros((0,))  # unit length for circle contact
#                 vt = np.array([gap_vel]) if contact_active else np.zeros((0,))
#                 approx_forces = np.array([force]) if contact_active else np.zeros((0,))
#                 # define active_indices for the single-contact case (index into the virtual segment list)
#                 active_indices = np.array([0], dtype=int) if contact_active else np.zeros((0,), dtype=int)
#                 # define placeholder arrays for circle_ids and gaps to avoid later indexing errors
#                 circle_ids = np.array([0])
#                 gaps = np.array([gap])
#
#             min_gap = float(active_gaps.min()) if active_count else None
#             max_force = float(np.max(approx_forces)) if approx_forces.size else 0.0
#
#             # show top contacts (up to 3) for readability; handle both curve and circle cases safely
#             details = []
#             if active_count and approx_forces.size:
#                 order = np.argsort(-approx_forces)
#                 top_k = min(3, approx_forces.size)
#                 for i_local in range(top_k):
#                     local_idx = order[i_local]
#                     # map local index back to segment index for curve-based contacts
#                     if 'segment_lengths' in info:
#                         seg_idx = int(active_indices[local_idx]) if active_indices.size else int(local_idx)
#                         circle_id = int(circle_ids[seg_idx]) if circle_ids.size > seg_idx else -1
#                         gap_val = float(gaps[seg_idx]) if gaps.size > seg_idx else float(active_gaps[local_idx])
#                     else:
#                         # circle-circle: use the local arrays
#                         seg_idx = int(local_idx)
#                         circle_id = -1
#                         gap_val = float(active_gaps[local_idx])
#                     
#                     force_val = float(approx_forces[local_idx])
#                     vt_val = float(vt[local_idx]) if vt.size > local_idx else 0.0
#                     if circle_id >= 0:
#                         details.append(f"seg#{seg_idx:03d}: circle={circle_id}, gap={gap_val:.3e}, Vt={vt_val:.3e}, Fn≈{force_val:.3e}")
#                     else:
#                         details.append(f"seg#{seg_idx:03d}: gap={gap_val:.3e}, Vt={vt_val:.3e}, Fn≈{force_val:.3e}")
#
#             detail_str = '; '.join(details) if details else '无接触对'
#             if active_count:
#                 line = (
#                     f"    - {info['label']} {{activePairs: {active_count}/{total_pairs}, minGap: {_format_optional(min_gap)}, maxFn≈{max_force:.6e}}} -> {detail_str}"
#                 )
#
#                 debug_trigger = False
#                 if group_name in CONTACT_DEBUG_GROUPS:
#                     if min_gap is not None and min_gap < CONTACT_DEBUG_GAP_TRIGGER:
#                         debug_trigger = True
#                     if max_force > CONTACT_DEBUG_FORCE_TRIGGER:
#                         debug_trigger = True
#
#                 if should_print_step:
#                     active_lines.append(line)
#                 elif debug_trigger:
#                     debug_lines.append((group_name, line))
#
#         if active_lines:
#             pending_lines.append((group_name, active_lines))
#
#     if not should_print_step and not debug_lines:
#         return True
#
#     print(f"\n[CONTACT STEP {step_index:06d} | t = {t:.6f} s]")
#
#     # 先输出常规分组
#     for group_name, lines in pending_lines:
#         print(f"  {group_name}:")
#         for line in lines:
#             print(line)
#
#     # 输出被触发的调试分组（避免重复打印）
#     if debug_lines:
#         printed_groups = set()
#         for group_name, line in debug_lines:
#             if group_name not in printed_groups:
#                 print(f"  {group_name} (debug trigger):")
#                 printed_groups.add(group_name)
#             print(line)
#     return True

# mbs.SetPostStepUserFunction(_report_contacts_per_step)

simulationSettings = exu.SimulationSettings()

# === 启用多核求解 ===

import psutil  # 可选，仅用于获取物理核心数

simulationSettings.parallel.numberOfThreads = psutil.cpu_count(logical=False)
simulationSettings.parallel.useLoadBalancing = True  # 动态负载均衡，适用于步长不均匀的任务
stepSize = 0.001  # 减小步长以提高稳定性
tEnd = 2.0  # 增加仿真时长以获得更多数据点

# === 解文件输出设置（压缩数据以减小文件大小）===
simulationSettings.solutionSettings.writeSolutionToFile = True

# 启用二进制格式：文件大小约为文本格式的 25-30%
simulationSettings.solutionSettings.binarySolutionFile = True

# 仅导出位置坐标，不导出速度/加速度（进一步减小50-70%）
simulationSettings.solutionSettings.exportVelocities = False
simulationSettings.solutionSettings.exportAccelerations = False
simulationSettings.solutionSettings.exportDataCoordinates = False  # 不导出GenericData节点

# 降低输出精度（文本模式下有效，二进制模式固定为双精度）
simulationSettings.solutionSettings.outputPrecision = 6  # 默认16位，降至6位

simulationSettings.solutionSettings.sensorsWritePeriod = 0.01

# 指定并创建解文件输出目录，避免默认文件缺失或被其他会话覆盖
solDir = os.path.join(os.path.dirname(__file__), 'solution')
try:
    os.makedirs(solDir, exist_ok=True)
except Exception:
    pass

# 二进制文件自动使用 .sol 扩展名，文本文件用 .txt
solutionFile = os.path.join(solDir, 'cycloid_bearing_solution')
simulationSettings.solutionSettings.coordinatesSolutionFileName = solutionFile
simulationSettings.timeIntegration.numberOfSteps = int(tEnd / stepSize)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.simulateInRealtime = False

# 使用稀疏求解器以处理大规模系统
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

# Newton求解器设置
simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 1e-8
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-8

# Generalized Alpha设置
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.3  # 增加数值阻尼
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True

# 自适应步长
simulationSettings.timeIntegration.adaptiveStep = True
simulationSettings.timeIntegration.adaptiveStepIncrease = 1.5
simulationSettings.timeIntegration.adaptiveStepDecrease = 0.5
simulationSettings.timeIntegration.minimumStepSize = 1e-8

simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.displayComputationTime = True
simulationSettings.displayStatistics = True

SC.visualizationSettings.general.graphicsUpdateInterval = stepSize
simulationSettings.solutionSettings.solutionWritePeriod = stepSize   # 或者显式写 0.001
simulationSettings.solutionSettings.sensorsWritePeriod  = stepSize   # 如果也希望传感器每步存一次

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

print("\n" + "="*60)
print("系统组装完成，开始仿真...")
print("="*60)

if useGraphics:
    SC.renderer.Start(); SC.renderer.DoIdleTasks()

import traceback, sys
try:
    mbs.SolveDynamic(simulationSettings)
except Exception:
    print("\n仿真出错，打印完整 traceback：")
    traceback.print_exc()
    sys.exit(1)

if useGraphics:
    try:
        SC.renderer.Stop()
    except:
        pass

print("\n" + "="*60)
print("仿真成功完成！")
print("="*60)

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# 读取数据并绘图（仿真后）
print("正在读取传感器数据...")
try:
    data_rot = np.asarray(mbs.GetSensorStoredData(sCrankRot))
    print(f"曲柄轴旋转数据形状: {data_rot.shape}")
    if data_rot.size == 0:
        data_rot = None
except Exception as e:
    print(f"读取曲柄轴旋转数据失败: {e}")
    data_rot = None

try:
    data_cyc1 = np.asarray(mbs.GetSensorStoredData(sCycloid1Pos))
    print(f"第一片摆线轮位置数据形状: {data_cyc1.shape}")
    if data_cyc1.size == 0:
        data_cyc1 = None
except Exception as e:
    print(f"读取第一片摆线轮位置数据失败: {e}")
    data_cyc1 = None

try:
    data_cyc2 = np.asarray(mbs.GetSensorStoredData(sCycloid2Pos))
    print(f"第二片摆线轮位置数据形状: {data_cyc2.shape}")
    if data_cyc2.size == 0:
        data_cyc2 = None
except Exception as e:
    print(f"读取第二片摆线轮位置数据失败: {e}")
    data_cyc2 = None

fig = plt.figure(figsize=(16, 12))
ax1 = plt.subplot(3, 2, 1)
ax2 = plt.subplot(3, 2, 2)
ax3 = plt.subplot(3, 2, 3)
ax4 = plt.subplot(3, 2, 4)
ax5 = plt.subplot(3, 2, 5)
ax6 = plt.subplot(3, 2, 6)

if data_rot is not None and len(data_rot) > 0:
    # 从旋转矩阵中提取绕Z轴的总转角，并做unwrap消除±π跳变
    z_rotation = []
    for i in range(len(data_rot)):
        # 旋转矩阵数据格式：[t, R11, R12, R13, R21, R22, R23, R31, R32, R33]
        R = np.array([[data_rot[i,1], data_rot[i,2], data_rot[i,3]],
                      [data_rot[i,4], data_rot[i,5], data_rot[i,6]],
                      [data_rot[i,7], data_rot[i,8], data_rot[i,9]]])
        # 从旋转矩阵提取绕Z轴的转角
        angle_z = np.arctan2(R[1,0], R[0,0])
        z_rotation.append(angle_z)
    
    z_rotation = np.unwrap(np.array(z_rotation))

    ax1.plot(data_rot[:,0], z_rotation, 'b-', linewidth=2)
    ax1.set_xlabel('Time (s)'); ax1.set_ylabel('Rotation (rad)'); ax1.set_title('Crankshaft Z-Axis Rotation Angle'); ax1.grid(True)
else:
    ax1.text(0.5, 0.5, 'No rotation data', ha='center', va='center', transform=ax1.transAxes)
    ax1.set_title('Crankshaft Rotation Angle (No Data)')

if data_cyc1 is not None and len(data_cyc1) > 0:
    ax2.plot(data_cyc1[:,0], data_cyc1[:,1], 'r-', label='X1', linewidth=2)
    ax2.plot(data_cyc1[:,0], data_cyc1[:,2], 'g-', label='Y1', linewidth=2)
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Position (mm)'); ax2.set_title('Cycloid Wheel 1 Position'); ax2.legend(); ax2.grid(True)
else:
    ax2.text(0.5, 0.5, 'No cycloid 1 data', ha='center', va='center', transform=ax2.transAxes)
    ax2.set_title('Cycloid Wheel 1 Position (No Data)')

if data_cyc2 is not None and len(data_cyc2) > 0:
    ax3.plot(data_cyc2[:,0], data_cyc2[:,1], 'r--', label='X2', linewidth=2)
    ax3.plot(data_cyc2[:,0], data_cyc2[:,2], 'g--', label='Y2', linewidth=2)
    ax3.set_xlabel('Time (s)'); ax3.set_ylabel('Position (mm)'); ax3.set_title('Cycloid Wheel 2 Position'); ax3.legend(); ax3.grid(True)
else:
    ax3.text(0.5, 0.5, 'No cycloid 2 data', ha='center', va='center', transform=ax3.transAxes)
    ax3.set_title('Cycloid Wheel 2 Position (No Data)')

if data_cyc1 is not None and len(data_cyc1) > 0:
    ax4.plot(data_cyc1[:,1], data_cyc1[:,2], 'r-', linewidth=2, label='Cycloid 1')
    ax4.plot(data_cyc1[0,1], data_cyc1[0,2], 'go', markersize=10, label='Start 1')
    ax4.plot(data_cyc1[-1,1], data_cyc1[-1,2], 'ro', markersize=10, label='End 1')
    ax4.set_xlabel('X (mm)'); ax4.set_ylabel('Y (mm)'); ax4.set_title('Cycloid Wheel 1 Trajectory'); ax4.axis('equal'); ax4.legend(); ax4.grid(True)
else:
    ax4.text(0.5, 0.5, 'No cycloid 1 data', ha='center', va='center', transform=ax4.transAxes)
    ax4.set_title('Cycloid Wheel 1 Trajectory (No Data)')

if data_cyc2 is not None and len(data_cyc2) > 0:
    ax5.plot(data_cyc2[:,1], data_cyc2[:,2], 'b-', linewidth=2, label='Cycloid 2')
    ax5.plot(data_cyc2[0,1], data_cyc2[0,2], 'mo', markersize=10, label='Start 2')
    ax5.plot(data_cyc2[-1,1], data_cyc2[-1,2], 'co', markersize=10, label='End 2')
    ax5.set_xlabel('X (mm)'); ax5.set_ylabel('Y (mm)'); ax5.set_title('Cycloid Wheel 2 Trajectory'); ax5.axis('equal'); ax5.legend(); ax5.grid(True)
else:
    ax5.text(0.5, 0.5, 'No cycloid 2 data', ha='center', va='center', transform=ax5.transAxes)
    ax5.set_title('Cycloid Wheel 2 Trajectory (No Data)')

if data_cyc1 is not None and len(data_cyc1) > 0:
    ax6.plot(data_cyc1[:,1], data_cyc1[:,2], 'r-', linewidth=2, label='Cycloid 1', alpha=0.7)
    ax6.plot(data_cyc1[0,1], data_cyc1[0,2], 'ro', markersize=10)
if data_cyc2 is not None and len(data_cyc2) > 0:
    ax6.plot(data_cyc2[:,1], data_cyc2[:,2], 'b-', linewidth=2, label='Cycloid 2', alpha=0.7)
    ax6.plot(data_cyc2[0,1], data_cyc2[0,2], 'bo', markersize=10)

for i in range(40):
    angle = 2 * np.pi * (i+0.25) / 40
    x_pin = r_pin_shell * np.sin(angle); y_pin = r_pin_shell * np.cos(angle)
    circle = Circle((x_pin, y_pin), r_z, color='gray', fill=False, alpha=0.3)
    ax6.add_patch(circle)
ax6.set_xlabel('X (mm)'); ax6.set_ylabel('Y (mm)'); ax6.set_title('Both Cycloid Wheels Trajectories'); ax6.axis('equal'); ax6.legend(); ax6.grid(True)

plt.tight_layout(); plt.show()

print("\n启动运动回放器...")
print("="*60)
print("可视化说明（三曲柄轴双片摆线轮 + 滚针轴承）：")
print("  【三曲柄轴系统】")
print("    - 3根曲柄轴，120°均布在分布圆上")
print("    - 每根曲柄轴包含：")
print("      * 灰色圆柱：主轴段")
print("      * 红色圆柱：第一个偏心段（正向）")
print("      * 橙色圆柱：第二个偏心段（反向）")
print("  【滚针轴承】")
print(f"    - 6个滚针轴承（3曲柄 × 2摆线轮）")
print(f"    - 每个轴承有{n_needles}个深灰色小圆柱滚针")
print(f"    - 滚针通过距离约束保持相对位置固定")
print(f"    - 滚针半径: {r_needle} mm，节圆半径: {r_needle_pitch:.2f} mm")
print("  【第一片摆线轮】（下方）")
print("    - 红色轮廓线：第一片摆线轮齿廓")
print(f"    - 3个绿色内孔圆：孔半径 {r_hole:.2f} mm")
print("    - 通过滚针轴承与3个红色偏心段连接")
print("  【第二片摆线轮】（上方）")
print("    - 紫色轮廓线：第二片摆线轮齿廓（相位差180°）")
print(f"    - 3个青色内孔圆：孔半径 {r_hole:.2f} mm")
print("    - 通过滚针轴承与3个橙色偏心段连接")
print("  【针齿壳}")
print("    - 灰色双层圆：针齿壳内外圈")
print("    - 40个橙色小圆：针齿孔位置标记")
print("  【针齿}")
print("    - 蓝色圆柱：40个针齿（同时与两片摆线轮接触）")
print("  【接触}")
print("    - 蓝色接触线：第一片摆线轮 <-> 针齿")
print("    - 紫色接触线：第二片摆线轮 <-> 针齿")
print("    - 圆-圆接触：偏心轴外圆 <-> 滚针")
print("    - 圆-圆孔接触：滚针 <-> 摆线轮孔内壁")
print("="*60)

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

# 现在使用简化的圆-圆接触模型，可以正常使用SolutionViewer回放器
print("使用简化的圆-圆接触模型，可以正常回放")

# 启动SolutionViewer回放器
if useGraphics:
    try:
        # 尝试简单的回放器启动
        mbs.SolutionViewer()
    except Exception as e:
        print(f"启动回放器时出错: {e}")
        print("尝试使用基础回放器...")
        try:
            # 备用方案：使用基础回放器
            from exudyn.interactive import SolutionViewer
            SolutionViewer(mbs)
        except Exception as e2:
            print(f"基础回放器也失败: {e2}")
            print("请检查SolutionViewer是否可用")

print("\n程序结束")
visForceScale = 0.02  # 可视化力箭头缩放（单位依场景调）

def set_contact_force_visibility(group_name, visible: bool):
    """Toggle contact-force visualization for a specific contact group."""
    object_ids = contact_objects_by_group.get(group_name, [])
    for obj_id in object_ids:
        obj = mbs.GetObject(obj_id)
        vis = obj.GetVisualization()
        vis.SetShow(visible)
