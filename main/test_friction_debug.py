"""
简化的摩擦力调试脚本
只运行很短时间，打印详细的速度信息
"""
import sys
import os
sys.exudynFast = True
sys.exudynCPUhasAVX2 = True

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'build', 'lib.win-amd64-cpython-313')) 

import exudyn as exu
import exudyn.graphics as graphics
import numpy as np
from exudyn.utilities import InertiaCylinder
from exudyn.itemInterface import (
    MarkerBodyRigid,
    NodeGenericData,
    ObjectContactCurveCircles,
    RevoluteJointZ,
    SensorBody,
    SensorObject,
    GenericJoint,
    VObjectJointGeneric,
)

def create_circle_segments(radius, n_segments=60, clockwise=False):
    """生成圆的折线近似"""
    angles = np.linspace(0.0, 2.0 * np.pi, num=n_segments, endpoint=False)
    if clockwise:
        angles = -angles
    points = np.stack((radius * np.cos(angles), radius * np.sin(angles)), axis=1)
    segments = np.zeros((n_segments, 4), dtype=float)
    for i in range(n_segments):
        p0 = points[i]
        p1 = points[(i + 1) % n_segments]
        segments[i, 0:2] = p0
        segments[i, 2:4] = p1
    return segments

# 创建系统
sc = exu.SystemContainer()
mbs = sc.AddSystem()

print("="*60)
print("摩擦力异常调试")
print("="*60)

# 参数
lower_radius = 0.5
lower_mass = 10.0
lower_angular_vel = -0.1 * np.pi

upper_radius = 2.0
upper_mass = 5.0

# 凸凹模式测试
concave_mode = os.environ.get('EXU_CONCAVE', '0') == '1'

# 修复1：极小的初始穿透
initial_gap = 1e-6 if concave_mode else -1e-6  # 凸凹用正间隙，凸凸用负间隙

# 修复2：更大的阻尼，更低的刚度
kn = 5e3  # 再降低刚度
damping = kn * 0.5  # 50%阻尼比（过阻尼）
# 修复3：先用极小摩擦系数测试
mu = 0.01  # 降低到0.01（仅用于测试稳定性）

if concave_mode:
    # 凸凹：小圆在大圆内部
    lower_center = [upper_radius - lower_radius + initial_gap, 0, 0]
    upper_center = [0, 0, 0]
else:
    # 凸凸：小圆在左，大圆在右
    lower_center = [0, 0, 0]
    upper_center = [lower_radius + upper_radius + initial_gap, 0, 0]

print(f"初始设置:")
print(f"  下圆: 半径={lower_radius}m, 角速度={lower_angular_vel}rad/s, 中心={lower_center}")
print(f"  上圆: 半径={upper_radius}m, 中心={upper_center}")
print(f"  初始间隙={initial_gap}m")
print(f"  理论切向速度={lower_angular_vel * lower_radius:.4f} m/s")

o_ground = mbs.CreateGround()

# 创建下圆（主动旋转）
lower_graphics = [
    graphics.Circle(point=[0, 0, 0], radius=lower_radius, color=[0.2, 0.2, 0.8, 1]),
]

o_lower = mbs.CreateRigidBody(
    referencePosition=lower_center,
    initialAngularVelocity=[0, 0, lower_angular_vel],
    inertia=InertiaCylinder(lower_mass, lower_radius, 0.01, 2),
    gravity=[0, 0, 0],
    graphicsDataList=lower_graphics,
)

m_lower = mbs.AddMarker(MarkerBodyRigid(bodyNumber=o_lower, localPosition=[0, 0, 0]))
m_ground = mbs.AddMarker(MarkerBodyRigid(bodyNumber=o_ground, localPosition=lower_center))
mbs.AddObject(GenericJoint(
    markerNumbers=[m_ground, m_lower],
    constrainedAxes=[1, 1, 1, 1, 1, 0],
    visualization=VObjectJointGeneric(axesRadius=0.02, axesLength=0.1)
))

# 创建上圆（被动，用折线近似）
upper_graphics = [
    graphics.Circle(point=[0, 0, 0], radius=upper_radius, color=[0.9, 0.1, 0.1, 1]),
]

n_segments_curve = 60
# 凸凹模式需要反向段（内部接触）
curve_segments_local = create_circle_segments(upper_radius, n_segments_curve, clockwise=concave_mode)

o_upper = mbs.CreateRigidBody(
    referencePosition=upper_center,
    inertia=InertiaCylinder(upper_mass, upper_radius, 0.01, 2),
    gravity=[0, 0, 0],
    graphicsDataList=upper_graphics,
)

m_upper = mbs.AddMarker(MarkerBodyRigid(bodyNumber=o_upper, localPosition=[0, 0, 0]))
m_ground_upper = mbs.AddMarker(MarkerBodyRigid(bodyNumber=o_ground, localPosition=upper_center))
mbs.AddObject(RevoluteJointZ(markerNumbers=[m_ground_upper, m_upper]))

# 创建接触
segments_matrix = exu.MatrixContainer(curve_segments_local)
initial_contact_data = []
for _ in range(n_segments_curve):
    initial_contact_data.extend([-1.0, 0.0, 0.0])

node_contact = mbs.AddNode(NodeGenericData(
    initialCoordinates=initial_contact_data,
    numberOfDataCoordinates=3 * n_segments_curve
))

obj_contact = mbs.AddObject(ObjectContactCurveCircles(
    markerNumbers=[m_upper, m_lower],
    nodeNumber=node_contact,
    circlesRadii=[lower_radius],
    segmentsData=segments_matrix,
    contactStiffness=kn,
    contactDamping=damping,
    dynamicFriction=mu,
    frictionProportionalZone=1e-3,
    contactModel=0,
    activeConnector=True
))

# 传感器
sensor_lower_omega = mbs.AddSensor(SensorBody(
    bodyNumber=o_lower, 
    outputVariableType=exu.OutputVariableType.AngularVelocityLocal,
    storeInternal=True
))

sensor_upper_omega = mbs.AddSensor(SensorBody(
    bodyNumber=o_upper, 
    outputVariableType=exu.OutputVariableType.AngularVelocityLocal,
    storeInternal=True
))

sensor_contact_slip = mbs.AddSensor(SensorObject(
    objectNumber=obj_contact,
    outputVariableType=exu.OutputVariableType.Velocity,
    storeInternal=True
))

sensor_contact_gap = mbs.AddSensor(SensorObject(
    objectNumber=obj_contact,
    outputVariableType=exu.OutputVariableType.DisplacementLocal,
    storeInternal=True
))

mbs.Assemble()

# 仿真设置
settings = exu.SimulationSettings()
settings.solutionSettings.writeSolutionToFile = False
settings.timeIntegration.numberOfSteps = 1000  # 修复4：1000步，步长=0.1μs
settings.timeIntegration.endTime = 1e-4  # 只运行0.1毫秒
settings.timeIntegration.verboseMode = 0
settings.displayComputationTime = False

# 使用固定步长以便观察
settings.timeIntegration.adaptiveStep = False
settings.timeIntegration.generalizedAlpha.spectralRadius = 0.8  # 增加数值耗散
settings.timeIntegration.generalizedAlpha.computeInitialAccelerations = False  # 关闭初始加速度计算
settings.timeIntegration.newton.absoluteTolerance = 1e-4  # 放宽容差
settings.timeIntegration.newton.relativeTolerance = 1e-4
settings.timeIntegration.newton.maxIterations = 20  # 限制迭代次数

print(f"\n开始仿真（{settings.timeIntegration.numberOfSteps}步，步长={settings.timeIntegration.endTime/settings.timeIntegration.numberOfSteps:.2e}s）...")
try:
    mbs.SolveDynamic(settings, showHints=False)
    print("✓ 仿真完成")
except Exception as e:
    print(f"仿真出错: {e}")

# 分析结果
print("\n" + "="*60)
print("详细分析:")
print("="*60)

lower_omega_data = np.asarray(mbs.GetSensorStoredData(sensor_lower_omega))
upper_omega_data = np.asarray(mbs.GetSensorStoredData(sensor_upper_omega))
slip_data = np.asarray(mbs.GetSensorStoredData(sensor_contact_slip))
gap_data = np.asarray(mbs.GetSensorStoredData(sensor_contact_gap))

print("\n逐步数据（前20步）:")
max_steps_to_show = min(20, len(lower_omega_data))
for i in range(max_steps_to_show):
    if i < len(lower_omega_data) and i < len(slip_data) and i < len(gap_data):
        t = lower_omega_data[i][0]
        omega_lower = lower_omega_data[i][3] if len(lower_omega_data[i]) > 3 else 0
        omega_upper = upper_omega_data[i][3] if i < len(upper_omega_data) and len(upper_omega_data[i]) > 3 else 0
        v_slip = slip_data[i][1:4] if len(slip_data[i]) > 3 else [0,0,0]
        gap = gap_data[i][1] if len(gap_data[i]) > 1 else 0
        
        v_slip_mag = np.linalg.norm(v_slip)
        v_tang_lower_theory = omega_lower * lower_radius
        
        print(f"\nStep {i}: t={t:.2e}s")
        print(f"  Gap={gap:.6e}m")
        print(f"  Lower omega={omega_lower:.6f} rad/s (理论={lower_angular_vel:.4f})")
        print(f"  Upper omega={omega_upper:.6f} rad/s")
        print(f"  |v_slip|={v_slip_mag:.6f} m/s")
        print(f"  理论切向速度={v_tang_lower_theory:.6f} m/s")
        
        # 检测异常
        warnings = []
        if abs(omega_lower) > abs(lower_angular_vel) * 2:
            warnings.append("下圆角速度异常增大")
        if abs(omega_upper) > 1.0:
            warnings.append("上圆角速度过大")
        if v_slip_mag > 1.0:
            warnings.append("滑移速度过大")
        if warnings:
            print(f"  ⚠️ {', '.join(warnings)}")

print("\n" + "="*60)

