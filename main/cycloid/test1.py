"""
柔性摆线轮单独测试脚本
========================
测试 FlexibleCycloidWheel 类的网格生成、模态计算和可视化
"""

import sys
sys.exudynFast = True
sys.exudynCPUhasAVX2 = True

import numpy as np
import exudyn as exu
from exudyn.utilities import *
from exudyn.itemInterface import ObjectContactCurveCircles, VObjectContactCurveCircles, NodeGenericData
import exudyn.graphics as graphics

# 导入柔性摆线轮模块
from flexible_cycloid import FlexibleCycloidWheel, NGSOLVE_AVAILABLE
from cycloid_profile import CycloidProfileParams, generate_cycloid_profile

# =================== 参数设置（与cycloid.py一致） ===================
# 摆线轮齿廓参数
z_b = 40              # 针销数 (与cycloid.py一致)
e = 2.2e-3            # 偏心距 (m)
R_z = 119.5e-3        # 针销分布圆半径 (m)
r_z = 5e-3            # 针销半径 (m)
n_profile_points = 1000  # 齿廓点数

# 摆线轮物理参数
thickness = 15e-3      # 厚度 (m) - 与偏心段长度类似
rho = 7850             # 密度 (kg/m³)
E_modulus = 2.1e11     # 弹性模量 (Pa)
nu = 0.3               # 泊松比

# 曲柄轴和轴承参数
crank_distribution_radius = 63e-3  # 曲柄轴分布圆半径 (m)
r_crank_eccentric = 22.5e-3        # 曲柄偏心段半径 (m)
r_needle = 4.25e-3                 # 滚针半径 (m)
bearing_clearance = 0.005e-3       # 轴承间隙 (m)
r_needle_pitch = r_crank_eccentric + r_needle + bearing_clearance  # 滚针节圆半径

# 孔位置（3个均布孔 - 与cycloid.py一致）
n_holes = 3
r_hole_center = crank_distribution_radius  # 孔分布半径 = 曲柄轴分布圆半径
r_hole = r_needle_pitch + r_needle + bearing_clearance  # 摆线轮孔半径
ANGLE_OFFSET = np.pi / 2  # 相位偏移（+Y轴开始）

hole_centers = []
for i in range(n_holes):
    angle = ANGLE_OFFSET + 2 * np.pi * i / n_holes
    x = r_hole_center * np.cos(angle)
    y = r_hole_center * np.sin(angle)
    hole_centers.append((x, y))

# 仿真参数
USE_GRAPHICS = True
SIMULATION_TIME = 0.1
TIME_STEP = 1e-4
MESH_SIZE = 8e-3    # 网格尺寸 (m)
N_MODES = 30        # 保留模态数

# =================== 生成齿廓 ===================
print("=" * 60)
print("柔性摆线轮测试")
print("=" * 60)

profile_params = CycloidProfileParams(
    z_b=z_b,           # 针销数
    e=e,               # 偏心距
    R_z=R_z,           # 针销分布圆半径
    r_z=r_z,           # 针销半径
)
x_profile, y_profile = generate_cycloid_profile(profile_params, n_points=n_profile_points)
print(f"生成齿廓: {len(x_profile)} 个点")

# =================== 系统初始化 ===================
SC = exu.SystemContainer()
mbs = SC.AddSystem()
oGround = mbs.CreateGround()

# =================== 创建柔性摆线轮 ===================
if not NGSOLVE_AVAILABLE:
    print("错误: NGsolve/Netgen 不可用，无法创建柔性摆线轮")
    sys.exit(1)

print("\n创建柔性摆线轮...")
flex_wheel = FlexibleCycloidWheel(
    profile_x=x_profile,
    profile_y=y_profile,
    thickness=thickness,
    hole_centers=hole_centers,
    hole_radius=r_hole,
    center_hole_radius=None,
    density=rho,
    youngs_modulus=E_modulus,
    poissons_ratio=nu,
    n_modes=N_MODES,
    mesh_size=MESH_SIZE
)
flex_wheel.build(verbose=True)

# 添加到系统
z_position = thickness / 2
result = flex_wheel.add_to_system(
    exu, mbs,
    position_ref=[0, 0, z_position],
    rotation_matrix_ref=np.eye(3),
    gravity=[0, 0, 0]
)

oCycloid = result['bodyNumber']
nCycloid = result['nodeNumber']
print(f"柔性体编号: {oCycloid}, 刚体节点编号: {nCycloid}")

# =================== 边界条件 ===================
# 约束摆线轮只能绕Z轴旋转（约束XY平移、Z平移、XY倾转）
mCycloidCenter = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid, localPosition=[0, 0, 0]))
mGroundCenter = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, z_position]))

# 约束XY平移、Z平移和XY倾转，只允许绕Z轴旋转
mbs.AddObject(GenericJoint(
    markerNumbers=[mGroundCenter, mCycloidCenter],
    constrainedAxes=[1, 1, 1, 1, 1, 0],  # 约束x,y,z平移 + Rx,Ry倾转，只释放Rz
    visualization=VObjectJointGeneric(axesRadius=2e-3, axesLength=10e-3)
))

# =================== 施加扭矩 ===================
# =================== 施加扭矩 ===================
APPLIED_TORQUE = 50.0  # N·m

def torque_function(mbs, t, loadVector):
    ramp_time = 0.02
    factor = min(t / ramp_time, 1.0)
    return [0, 0, factor * APPLIED_TORQUE]

# 注释掉扭矩施加，测试静态平衡
# mbs.AddLoad(LoadTorqueVector(
#     markerNumber=mCycloidCenter,
#     loadVector=[0, 0, APPLIED_TORQUE],
#     loadVectorUserFunction=torque_function
# ))
# print(f"施加扭矩: {APPLIED_TORQUE} N·m (绕Z轴) [已禁用]")

# =================== 传感器 ===================
sensor_rotation = mbs.AddSensor(SensorNode(
    nodeNumber=nCycloid,
    storeInternal=True,
    outputVariableType=exu.OutputVariableType.Rotation
))

# =================== 添加单个针销进行接触测试 ===================
# =================== 添加单个针销进行接触测试 ===================
# 针销参数初始设置
pin_angle = 0.0  # 初始尝试放在+X方向
pin_radius = r_z

# --------------- 自动调整针销位置以避免穿透 ---------------
print("\n[自动调整] 正在检查初始间隙...")
x_arr = np.array(x_profile)
y_arr = np.array(y_profile)
radii = np.sqrt(x_arr**2 + y_arr**2)
angles = np.arctan2(y_arr, x_arr)

# 找到最接近 pin_angle 的点
idx_pin = np.argmin(np.abs(angles - pin_angle))
r_cycloid_at_pin = radii[idx_pin]
gap = (R_z - pin_radius) - r_cycloid_at_pin

print(f"  初始尝试角度: {np.rad2deg(pin_angle):.1f} deg")
print(f"  摆线轮半径: {r_cycloid_at_pin*1000:.4f} mm")
print(f"  针销内表面: {(R_z - pin_radius)*1000:.4f} mm")
print(f"  初始间隙: {gap*1000:.4f} mm")

if gap < 0.2e-3:  # 如果间隙小于0.2mm（或者负值），则寻找最近的齿槽
    print(f"  警告: 发现穿透或间隙过小! 正在寻找最近的齿槽...")
    
    # 寻找局部极小值（齿槽是半径最小的地方）
    from scipy.signal import argrelextrema
    # 稍微平滑一下数据以避免噪声干扰极值查找（虽然理论生成数据很平滑）
    min_indices = argrelextrema(radii, np.less, order=5)[0]
    
    if len(min_indices) > 0:
        # 找到角度差最小的齿槽
        angle_diffs = np.abs(angles[min_indices] - pin_angle)
        # 处理角度周期性（虽然这里在0附近，应该没事）
        angle_diffs = np.minimum(angle_diffs, 2*np.pi - angle_diffs)
        
        nearest_groove_idx = min_indices[np.argmin(angle_diffs)]
        best_angle = angles[nearest_groove_idx]
        best_r = radii[nearest_groove_idx]
        new_gap = (R_z - pin_radius) - best_r
        
        print(f"  找到最近齿槽: 角度 {np.rad2deg(best_angle):.4f} deg, 半径 {best_r*1000:.4f} mm")
        print(f"  新位置间隙: {new_gap*1000:.4f} mm")
        
        pin_angle = best_angle
        print(f"  >> 已自动调整针销角度到: {np.rad2deg(pin_angle):.4f} deg")
    else:
        print("  错误: 未能自动找到齿槽!")
else:
    print("  间隙正常。")

# 计算最终位置
pin_x = R_z * np.cos(pin_angle)
pin_y = R_z * np.sin(pin_angle)
# --------------------------------------------------------

print(f"\n创建针销...")
print(f"  针销位置: ({pin_x*1000:.1f}, {pin_y*1000:.1f}) mm")
print(f"  针销半径: {pin_radius*1000:.2f} mm")

# 创建固定针销
inertia_pin = InertiaCylinder(density=rho, length=thickness, outerRadius=pin_radius, axis=2)
oPin = mbs.CreateRigidBody(
    referencePosition=[pin_x, pin_y, z_position],
    inertia=inertia_pin,
    gravity=[0, 0, 0],
    graphicsDataList=[graphics.Cylinder(pAxis=[0, 0, -thickness/2], vAxis=[0, 0, thickness], 
                                         radius=pin_radius, color=graphics.color.red, nTiles=32)]
)

# 固定针销到地面
mPinRigid = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oPin, localPosition=[0, 0, 0]))
mGroundPin = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[pin_x, pin_y, z_position]))
mbs.AddObject(GenericJoint(
    markerNumbers=[mGroundPin, mPinRigid],
    constrainedAxes=[1, 1, 1, 1, 1, 1],  # 完全固定
    visualization=VObjectJointGeneric(axesRadius=1e-3, axesLength=5e-3, show=False)
))

# 创建针销接触标记
mPin = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oPin, localPosition=[0, 0, 0]))

# =================== 添加滚针进行圆圆接触测试 ===================
# 滚针位于第一个孔(+Y方向)内
hole_idx = 0  # 第一个孔
hole_x, hole_y = hole_centers[hole_idx]

# 滚针位置：在滚针节圆上，相对于孔中心
needle_angle = 0.0  # 放在孔中心的+X方向（相对于孔）
needle_local_x = r_needle_pitch * np.cos(needle_angle)
needle_local_y = r_needle_pitch * np.sin(needle_angle)
needle_x = hole_x + needle_local_x
needle_y = hole_y + needle_local_y

print(f"\n创建滚针...")
print(f"  滚针位置: ({needle_x*1000:.1f}, {needle_y*1000:.1f}) mm")
print(f"  滚针半径: {r_needle*1000:.2f} mm")
print(f"  孔半径: {r_hole*1000:.2f} mm")

# 创建滚针刚体
inertia_needle = InertiaCylinder(density=rho, length=thickness, outerRadius=r_needle, axis=2)
oNeedle = mbs.CreateRigidBody(
    referencePosition=[needle_x, needle_y, z_position],
    inertia=inertia_needle,
    gravity=[0, 0, 0],
    graphicsDataList=[graphics.Cylinder(pAxis=[0, 0, -thickness/2], vAxis=[0, 0, thickness], 
                                         radius=r_needle, color=graphics.color.orange, nTiles=32)]
)

# 滚针约束：只能在XY平面平移，可绕Z轴旋转
mNeedleRigid = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle, localPosition=[0, 0, 0]))
mGroundNeedle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[needle_x, needle_y, z_position]))
mbs.AddObject(GenericJoint(
    markerNumbers=[mGroundNeedle, mNeedleRigid],
    constrainedAxes=[0, 0, 1, 1, 1, 0],  # 约束Z平移和XY倾转，允许XY平移和Z旋转
    visualization=VObjectJointGeneric(axesRadius=1e-3, axesLength=5e-3, show=False)
))

# 滚针接触标记
mNeedle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle, localPosition=[0, 0, 0]))

# 准备接触数据 - 齿廓曲线接触
n_profile = len(x_profile)
segmentsData = np.zeros((n_profile, 4))
for i in range(n_profile):
    segmentsData[i, 0] = x_profile[i]
    segmentsData[i, 1] = y_profile[i]
    segmentsData[i, 2] = x_profile[(i + 1) % n_profile]
    segmentsData[i, 3] = y_profile[(i + 1) % n_profile]

# 获取齿廓节点索引（用于柔性体变形追踪）
profileNodeIndices = result.get('profile_node_indices', np.array([], dtype=int))

# 创建曲线接触数据节点
nDataVars = 5  # 每段5个变量
initialCoords = [-1.0, 0.0, 0.0, 0.0, 0.0] * n_profile
nContactData = mbs.AddNode(NodeGenericData(
    initialCoordinates=initialCoords,
    numberOfDataCoordinates=nDataVars * n_profile
))

# 接触参数
contactStiffness = 1e9   # N/m
contactDamping = 1e5     # N·s/m
friction = 0.1
contactStiffness_bearing = 1e9
contactDamping_bearing = 1e5

# =================== 创建曲线-圆接触（摆线齿廓-针销） ===================
print(f"\n创建曲线-圆接触（摆线齿廓-针销）...")
print(f"  齿廓段数: {n_profile}")
print(f"  profileNodeIndices 长度: {len(profileNodeIndices)}")

objContactPin = mbs.AddObject(ObjectContactCurveCircles(
    markerNumbers=[mCycloidCenter, mPin],
    nodeNumber=nContactData,
    circlesRadii=[pin_radius],
    segmentsData=exu.MatrixContainer(segmentsData),
    profileNodeIndices=profileNodeIndices,
    contactStiffness=contactStiffness,
    contactDamping=contactDamping,
    dynamicFriction=friction,
    frictionProportionalZone=0.0,
    frictionVelocityPenalty=1000.0,
    frictionStiffness=1e8,
    contactModel=0,
    visualization=VObjectContactCurveCircles(show=True, color=graphics.color.blue)
))
print(f"  曲线-圆接触对象编号: {objContactPin}")

# =================== 创建圆-圆接触（滚针-摆线轮孔） ===================
from exudyn.itemInterface import ObjectContactCircleCircle, VObjectContactCircleCircle, MarkerSuperElementRigid

# 对于柔性体，需要使用 MarkerSuperElementRigid 来正确追踪变形
# 获取孔周围的节点索引
hole_nodes = flex_wheel.get_hole_nodes_list(hole_idx, n_nodes=8)
print(f"\n创建圆-圆接触（滚针-摆线轮孔）...")
print(f"  孔 {hole_idx} 周围节点: {hole_nodes[:4]}... (共{len(hole_nodes)}个)")

# 使用 MarkerSuperElementRigid 来追踪柔性体上孔的位置
# 这会根据周围节点的加权平均来计算孔的当前位置
n_hole_nodes = len(hole_nodes)
weighting_factors = [1.0 / n_hole_nodes] * n_hole_nodes  # 均匀权重，总和为1

mCycloidHole = mbs.AddMarker(MarkerSuperElementRigid(
    bodyNumber=oCycloid,
    meshNodeNumbers=hole_nodes,           # 孔周围的节点
    weightingFactors=weighting_factors,   # 权重因子（总和必须为1）
    useAlternativeApproach=True           # 使用备用方法计算刚体运动
))

# 创建接触数据节点（圆-圆接触需要8个坐标）
nContactDataCircle = mbs.AddNode(NodeGenericData(
    initialCoordinates=[0.0] * 8,
    numberOfDataCoordinates=8
))

objContactNeedle = mbs.AddObject(ObjectContactCircleCircle(
    markerNumbers=[mNeedle, mCycloidHole],
    nodeNumber=nContactDataCircle,
    radius1=r_needle,
    radius2=-r_hole,  # 负值表示内接触（孔）
    contactStiffness=contactStiffness_bearing,
    contactDamping=contactDamping_bearing,
    frictionCoefficient=0.05,
    frictionVelocityPenalty=1000.0,
    frictionProportionalZone=1e-4,
    frictionStiffness=1e8,
    visualization=VObjectContactCircleCircle(show=True)
))
print(f"  圆-圆接触对象编号: {objContactNeedle}")

# =================== 添加保持架约束（复现cycloid.py结构）===================
print("\n创建保持架约束（测试 MarkerSuperElementRigid + GenericJoint 兼容性）...")

# 创建保持架刚体（位于孔中心）
hole_x, hole_y = hole_centers[hole_idx]
cage_radius = r_hole * 0.8  # 保持架半径略小于孔
cage_thickness = thickness * 0.5
cage_mass = rho * np.pi * cage_radius**2 * cage_thickness
cage_inertia = RigidBodyInertia(
    mass=cage_mass,
    inertiaTensor=np.diag([cage_mass * cage_radius**2 / 4, cage_mass * cage_radius**2 / 4, cage_mass * cage_radius**2 / 2]),
    com=[0, 0, 0]
)

oCage = mbs.CreateRigidBody(
    referencePosition=[hole_x, hole_y, z_position],
    inertia=cage_inertia,
    gravity=[0, 0, 0],
    graphicsDataList=[graphics.Cylinder(pAxis=[0, 0, -cage_thickness/2], vAxis=[0, 0, cage_thickness], 
                                         radius=cage_radius, color=graphics.color.cyan, nTiles=32)]
)

# 保持架中心标记 (MarkerBodyRigid)
mCageCenter = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCage, localPosition=[0, 0, 0]))

# 关键测试：使用 GenericJoint 连接 MarkerSuperElementRigid (mCycloidHole) 和 MarkerBodyRigid (mCageCenter)
# 这正是 cycloid.py 中的结构
mbs.AddObject(GenericJoint(
    markerNumbers=[mCycloidHole, mCageCenter],  # MarkerSuperElementRigid ↔ MarkerBodyRigid
    constrainedAxes=[0, 0, 1, 1, 1, 0],  # 允许XY平移和Z旋转
    visualization=VObjectJointGeneric(axesRadius=0.5e-3, axesLength=2e-3, show=True)
))
print(f"  保持架约束已添加: mCycloidHole (MarkerSuperElementRigid) ↔ mCageCenter (MarkerBodyRigid)")

# =================== 组装系统 ===================
mbs.Assemble()
print("系统组装完成")

# =================== 仿真设置 ===================
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = int(SIMULATION_TIME / TIME_STEP)
simulationSettings.timeIntegration.endTime = SIMULATION_TIME
simulationSettings.timeIntegration.verboseMode = 1

# =================== 启用多线程（测试多线程稳定性） ===================
import multiprocessing
n_threads = min(multiprocessing.cpu_count(), 28)
simulationSettings.parallel.numberOfThreads = n_threads
print(f"启用多线程求解: {n_threads} 个线程")

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.newton.maxIterations = 50
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-5

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.9
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True

# =================== 可视化和仿真 ===================
if USE_GRAPHICS:
    SC.visualizationSettings.general.autoFitScene = True
    SC.visualizationSettings.nodes.showBasis = True
    SC.visualizationSettings.nodes.basisSize = 0.02
    exu.StartRenderer()
    mbs.WaitForUserToContinue()

print("\n开始仿真...")
print("-" * 60)

try:
    mbs.SolveDynamic(simulationSettings, showHints=True)
    print("-" * 60)
    print("仿真成功!")
    success = True
except Exception as ex:
    print("-" * 60)
    print(f"仿真失败: {ex}")
    success = False

if USE_GRAPHICS:
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer()

# =================== 结果分析 ===================
print("\n" + "=" * 60)
print("结果分析")
print("=" * 60)

rot_data = mbs.GetSensorStoredData(sensor_rotation)

if len(rot_data) > 0:
    final_rot = rot_data[-1, 1:4]
    print(f"末端最终旋转 (deg): [{np.rad2deg(final_rot[0]):.4f}, {np.rad2deg(final_rot[1]):.4f}, {np.rad2deg(final_rot[2]):.4f}]")
else:
    print("警告: 传感器无数据")

print(f"\n仿真结果: {'成功' if success else '失败'}")
print("=" * 60)
