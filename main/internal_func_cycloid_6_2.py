import sys, os

sys.exudynFast = True          # 让 exudyn 优先加载 fast 版（依赖 AVX2）
sys.exudynCPUhasAVX2 = True 

import numpy as np

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
from exudyn.itemInterface import ObjectContactCircleCircle, ObjectConnectorDistance, MarkerBodyPosition
from exudyn.itemInterface import VObjectConnectorDistance, SpringDamper, VSpringDamper
from exudyn.machines import GetBallBearingData, CreateBallBearing

useGraphics = True  # 启用图形窗口显示
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

contactStiffness_tooth = 1e7
contactDamping_tooth = 1e3
contactStiffness_hole = 1e7
contactDamping_hole = 1e3
contactStiffness_pin_hole = 1e7
contactDamping_pin_hole = 1e3

# 接触摩擦系数（库仑摩擦）
friction_cycloid_pin = 0.08             # 摆线齿廓-针齿
friction_crank_needle = 0.02            # 偏心轴-滚针
friction_needle_cycloid_hole = 0.02     # 滚针-摆线孔
friction_flange_bearing = 0.015         # 法兰轴承滚针
friction_pin_shell = 0.03               # 针齿-壳体孔

# 法兰轴承参数
r_flange_shaft = r_crank_main  # 法兰孔与曲柄主轴接触（主轴半径）
r_flange_needle = 2.0  # 法兰轴承滚针半径（比摆线轮轴承小）
n_flange_needles = 10  # 每个法兰轴承的滚针数量
flange_bearing_clearance = -0.002
r_flange_needle_pitch = r_flange_shaft + r_flange_needle + flange_bearing_clearance
r_flange_hole = r_flange_needle_pitch + r_flange_needle + flange_bearing_clearance  # 法兰孔半径

print("=" * 60)
print("三曲柄轴双片摆线针轮减速器仿真")
print(f"曲柄: {n_cranks}, 摆线滚针: {n_needles}/轴承, 法兰滚针: {n_flange_needles}/轴承")
print("=" * 60)

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
print(f"输入: {omega_input_target*60/(2*np.pi):.1f} RPM, 输出扭矩: {outputTorqueZ} N·m")


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
    dynamicFriction=friction_cycloid_pin,
    frictionProportionalZone=1e-3,
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
    dynamicFriction=friction_cycloid_pin,
    frictionProportionalZone=1e-3,
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

# 目标滚针间距（弦长）
chord_cyc = 2 * r_needle_pitch * np.sin(np.pi / n_needles)
chord_flg = 2 * r_flange_needle_pitch * np.sin(np.pi / n_flange_needles)

# 滚针间距柔性约束参数（低刚度允许一定变形）
needle_spacing_stiffness = 5e4  # N/m，柔性刚度
needle_spacing_damping = 1e2    # N/(m/s)，阻尼

# 存储滚针位置标记（按曲柄分组）
needle_markers_cyc1 = [[] for _ in range(n_cranks)]
needle_markers_cyc2 = [[] for _ in range(n_cranks)]
needle_markers_in = [[] for _ in range(n_cranks)]
needle_markers_out = [[] for _ in range(n_cranks)]

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

        # 记录第一层摆线滚针位置
        mPosNeedle1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oNeedle1, localPosition=[0, 0, 0]))
        needle_markers_cyc1[i_crank].append(mPosNeedle1)

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

        # 记录第二层摆线滚针位置
        mPosNeedle2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oNeedle2, localPosition=[0, 0, 0]))
        needle_markers_cyc2[i_crank].append(mPosNeedle2)

    # 添加摆线轴承滚针柔性间距约束
    for marker_list in (needle_markers_cyc1[i_crank], needle_markers_cyc2[i_crank]):
        n_markers = len(marker_list)
        if n_markers == 0:
            continue
        for idx in range(n_markers):
            mA = marker_list[idx]
            mB = marker_list[(idx + 1) % n_markers]
            mbs.AddObject(SpringDamper(
                markerNumbers=[mA, mB],
                referenceLength=chord_cyc,
                stiffness=needle_spacing_stiffness,
                damping=needle_spacing_damping,
                visualization=VSpringDamper(show=False)
            ))

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

        # 记录输入法兰滚针位置
        mPosNeedleIn = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oNeedleIn, localPosition=[0, 0, 0]))
        needle_markers_in[i_crank].append(mPosNeedleIn)

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

        # 记录输出法兰滚针位置
        mPosNeedleOut = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oNeedleOut, localPosition=[0, 0, 0]))
        needle_markers_out[i_crank].append(mPosNeedleOut)

    # 添加法兰轴承滚针柔性间距约束
    for marker_list in (needle_markers_in[i_crank], needle_markers_out[i_crank]):
        n_markers = len(marker_list)
        if n_markers == 0:
            continue
        for idx in range(n_markers):
            mA = marker_list[idx]
            mB = marker_list[(idx + 1) % n_markers]
            mbs.AddObject(SpringDamper(
                markerNumbers=[mA, mB],
                referenceLength=chord_flg,
                stiffness=needle_spacing_stiffness,
                damping=needle_spacing_damping,
                visualization=VSpringDamper(show=False)
            ))

print(f"滚针轴承: {n_cranks*2} 个, 间距约束: {n_cranks*2*n_needles + n_cranks*2*n_flange_needles} 个 (刚度 {needle_spacing_stiffness:.0e} N/m)")

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
        friction=friction_pin_shell,
        group="PinShell"
    )

# =================== 传感器/组装/仿真 ===================
mbs.CreateGround(graphicsDataList=[])

# 传感器（需在仿真前创建并 storeInternal 以记录数据）
sCycloid1Pos = mbs.AddSensor(SensorBody(bodyNumber=oCycloid1, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
sCycloid2Pos = mbs.AddSensor(SensorBody(bodyNumber=oCycloid2, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
sCrankRot    = mbs.AddSensor(SensorBody(bodyNumber=crankshaft_bodies[0], storeInternal=True, outputVariableType=exu.OutputVariableType.RotationMatrix))
sPin0Pos     = mbs.AddSensor(SensorBody(bodyNumber=pin_bodies[0], storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
contact_sensors = {}

# 摆线轮-针齿接触（使用Object传感器读取实际接触力）
contact_sensors['Cycloid1-Pins'] = {
    'type': 'curve_force',
    'sensor': mbs.AddSensor(SensorObject(
        objectNumber=objCycloid1Pins,
        storeInternal=True,
        outputVariableType=exu.OutputVariableType.ForceLocal
    ))
}
contact_sensors['Cycloid2-Pins'] = {
    'type': 'curve_force',
    'sensor': mbs.AddSensor(SensorObject(
        objectNumber=objCycloid2Pins,
        storeInternal=True,
        outputVariableType=exu.OutputVariableType.ForceLocal
    ))
}

# 调试传感器：获取DisplacementLocal和VelocityLocal
sCycloid1Gap = mbs.AddSensor(SensorObject(
    objectNumber=objCycloid1Pins,
    storeInternal=True,
    outputVariableType=exu.OutputVariableType.DisplacementLocal
))
sCycloid1Vel = mbs.AddSensor(SensorObject(
    objectNumber=objCycloid1Pins,
    storeInternal=True,
    outputVariableType=exu.OutputVariableType.VelocityLocal
))

# 获取几个代表性的滚针轴承接触对象（使用实际ForceLocal输出）
bearing_contacts = [info for info in CONTACT_NODE_INFO if info['group'] in ['CrankNeedle', 'NeedleCycloidHole']]
if len(bearing_contacts) >= 4:
    # 记录前4个轴承接触（2个曲柄-滚针，2个滚针-孔）
    for i, info in enumerate(bearing_contacts[:4]):
        sensor_name = f"{info['group']}-{i}"
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

    circle_indices = node_data[0::3].astype(np.int32, copy=False)
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
        contact_log_cyc1.append(_sample_cycloid_contact_forces(mbs, objCycloid1Pins, nGenericData_tooth1, n_pin_circles))
        contact_log_cyc2.append(_sample_cycloid_contact_forces(mbs, objCycloid2Pins, nGenericData_tooth2, n_pin_circles))
    return True  # 返回True表示继续仿真

mbs.Assemble()
print(f"已添加 {len(contact_sensors)} 个接触力传感器")

simulationSettings = exu.SimulationSettings()

# === 启用多核求解 ===

import psutil  # 可选，仅用于获取物理核心数

simulationSettings.parallel.numberOfThreads = psutil.cpu_count(logical=False)
simulationSettings.parallel.useLoadBalancing = True  # 动态负载均衡，适用于步长不均匀的任务
stepSize = 0.001  # 减小步长以提高稳定性
tEnd = 2.0  # 增加仿真时长以获得更多数据点

# 接触力采样间隔（设置采样间隔以启用接触力日志记录）
contact_sample_interval = stepSize  # 每步采样一次

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

# 设置post-step函数（在mbs.Assemble()之后调用）
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
        mbs.SolveDynamic(simulationSettings)
    else:
        # 如果有可视化，先设置非实时模式然后启动仿真
        simulationSettings.timeIntegration.simulateInRealtime = False
        mbs.SolveDynamic(simulationSettings)
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

fig = plt.figure(figsize=(20, 16))
ax1 = plt.subplot(4, 2, 1)
ax2 = plt.subplot(4, 2, 2)
ax3 = plt.subplot(4, 2, 3)
ax4 = plt.subplot(4, 2, 4)
ax5 = plt.subplot(4, 2, 5)
ax6 = plt.subplot(4, 2, 6)
ax7 = plt.subplot(4, 2, 7)  # 接触力图1
ax8 = plt.subplot(4, 2, 8)  # 接触力图2

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

# 绘制接触力数据（ax7和ax8）
colors = ['r', 'g', 'b', 'orange', 'purple', 'cyan', 'brown', 'pink']
if contact_force_data:
    # ax7: 摆线轮-针齿接触力（显示所有针齿的单独力）
    cycloid1_individual = contact_force_data.get('Cycloid1-Pins_individual')
    cycloid2_individual = contact_force_data.get('Cycloid2-Pins_individual')
    
    if cycloid1_individual or cycloid2_individual:
        pin_count = 0
        if cycloid1_individual:
            time = cycloid1_individual['time']
            force_matrix = cycloid1_individual['forces']
            for pin_idx in range(force_matrix.shape[1]):
                force = force_matrix[:, pin_idx]
                max_f = np.max(force)
                if max_f > 0.1:
                    ax7.plot(time, force, linewidth=2, alpha=0.8, label=f'Cyc1-Pin{pin_idx} (max={max_f:.1f}N)')
                    pin_count += 1
        
        if cycloid2_individual:
            time = cycloid2_individual['time']
            force_matrix = cycloid2_individual['forces']
            for pin_idx in range(force_matrix.shape[1]):
                force = force_matrix[:, pin_idx]
                max_f = np.max(force)
                if max_f > 0.1:
                    ax7.plot(time, force, linewidth=2, alpha=0.8, linestyle='--', label=f'Cyc2-Pin{pin_idx} (max={max_f:.1f}N)')
                    pin_count += 1
        
        ax7.set_xlabel('Time (s)', fontsize=11)
        ax7.set_ylabel('Contact Force (N)', fontsize=11)
        ax7.set_title(f'All Active Pin Contact Forces ({pin_count} pins)', fontsize=11, fontweight='bold')
        ax7.legend(fontsize=8, ncol=2, loc='best')
        ax7.grid(True, alpha=0.3)
    else:
        ax7.text(0.5, 0.5, 'No individual pin data', ha='center', va='center', transform=ax7.transAxes)
        ax7.set_title('Pin Contact Forces')
    
    # ax8: 滚针轴承接触力
    bearing_contacts = [(k, v) for k, v in contact_force_data.items() if 'Needle' in k or 'Crank' in k]
    if bearing_contacts:
        for idx, (name, data) in enumerate(bearing_contacts):
            if data.ndim == 2 and data.shape[1] >= 2:
                ax8.plot(data[:, 0], data[:, 1], color=colors[idx % len(colors)], linewidth=2, label=name)
        ax8.set_xlabel('Time (s)', fontsize=11); 
        ax8.set_ylabel('Contact Force (N)', fontsize=11); 
        ax8.set_title('Needle Bearing Contact Forces', fontsize=12, fontweight='bold'); 
        ax8.legend(fontsize=9); 
        ax8.grid(True, alpha=0.3)
    else:
        ax8.text(0.5, 0.5, 'No bearing data', ha='center', va='center', transform=ax8.transAxes)
        ax8.set_title('Bearing Contact Forces')
else:
    ax7.text(0.5, 0.5, 'No contact force data', ha='center', va='center', transform=ax7.transAxes)
    ax7.set_title('Contact Forces (No Data)')
    ax8.text(0.5, 0.5, 'No bearing force data', ha='center', va='center', transform=ax8.transAxes)
    ax8.set_title('Bearing Forces (No Data)')

plt.tight_layout(); plt.show()

print("\n可视化颜色: 红/紫=摆线轮1/2, 蓝=针齿, 灰=滚针, 橙=偏心轴")

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
