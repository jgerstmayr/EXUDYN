"""
柔性曲柄轴单独测试脚本
========================
测试修改后的C++ EXUDYN梁单元（Z轴版本）
"""

import sys
sys.exudynFast = True
sys.exudynCPUhasAVX2 = True

import numpy as np
import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics

# 导入柔性曲柄轴模块
from flexible_crankshaft import (
    FlexibleCrankshaft, CrankSegment,
    create_standard_crank_segments,
)

# =================== 参数设置 ===================
e = 2.2e-3  # 偏心距 (m)
r_crank_main = 20e-3  # 曲柄主轴半径 (m)
r_crank_eccentric = 22.5e-3  # 曲柄偏心段半径 (m)
L_crank_main1 = 20e-3
L_crank_eccentric1 = 15e-3
L_crank_middle = 5e-3
L_crank_eccentric2 = 15e-3
L_crank_main2 = 20e-3
eccentric_offset = e

L_crank_total = L_crank_main1 + L_crank_eccentric1 + L_crank_middle + L_crank_eccentric2 + L_crank_main2

# 仿真参数
USE_GRAPHICS = True
SIMULATION_TIME = 0.1
TIME_STEP = 1e-5
APPLIED_TORQUE = 10.0  # N·m

# =================== 系统初始化 ===================
SC = exu.SystemContainer()
mbs = SC.AddSystem()
oGround = mbs.CreateGround()

print("=" * 60)
print("柔性曲柄轴测试 - Z轴版本 (C++已修改)")
print("=" * 60)
print(f"曲柄轴总长度: {L_crank_total*1000:.1f} mm")

# =================== 创建曲柄轴 ===================
crank_segments = create_standard_crank_segments(
    L_main1=L_crank_main1,
    L_eccentric1=L_crank_eccentric1,
    L_middle=L_crank_middle,
    L_eccentric2=L_crank_eccentric2,
    L_main2=L_crank_main2,
    r_main=r_crank_main,
    r_eccentric=r_crank_eccentric,
    eccentric_offset=eccentric_offset
)

crank = FlexibleCrankshaft(
    segments=crank_segments,
    n_elements_per_segment=2
)

result = crank.build(
    mbs,
    reference_position=[0, 0, 0],
    z_offset=-L_crank_main1,
    use_spring_connection=True,
    connection_stiffness_factor=1.0
)

print(f"创建了 {len(result['nodes'])} 个节点, {len(result['beam_objects'])} 个梁单元")

# =================== 边界条件 ===================
node_base = result['first_node']
z_base = crank.node_z_positions[0]
mGroundCrank = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, z_base]))
mCrankBase = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node_base))
mbs.AddObject(GenericJoint(
    markerNumbers=[mGroundCrank, mCrankBase],
    constrainedAxes=[1, 1, 1, 1, 1, 1],
    visualization=VObjectJointGeneric(axesRadius=1e-3, axesLength=5e-3)
))

# =================== 施加扭矩 ===================
node_end = result['last_node']
mCrankEnd = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node_end))

def torque_function(mbs, t, loadVector):
    ramp_time = 0.02
    factor = min(t / ramp_time, 1.0)
    return [0, 0, factor * APPLIED_TORQUE]  # 绕Z轴扭矩

mbs.AddLoad(LoadTorqueVector(
    markerNumber=mCrankEnd,
    loadVector=[0, 0, APPLIED_TORQUE],
    loadVectorUserFunction=torque_function
))
print(f"在末端施加扭矩: {APPLIED_TORQUE} N·m (绕Z轴)")

# =================== 传感器 ===================
sensor_displacement = mbs.AddSensor(SensorNode(
    nodeNumber=node_end,
    storeInternal=True,
    outputVariableType=exu.OutputVariableType.Displacement
))

sensor_rotation = mbs.AddSensor(SensorNode(
    nodeNumber=node_end,
    storeInternal=True,
    outputVariableType=exu.OutputVariableType.Rotation
))

# =================== 组装系统 ===================
mbs.Assemble()
print("系统组装完成")

# =================== 仿真设置 ===================
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = int(SIMULATION_TIME / TIME_STEP)
simulationSettings.timeIntegration.endTime = SIMULATION_TIME
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.newton.maxIterations = 50
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-5
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-6

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.9
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True

# =================== 可视化和仿真 ===================
if USE_GRAPHICS:
    SC.visualizationSettings.general.autoFitScene = True
    SC.visualizationSettings.nodes.showBasis = True
    SC.visualizationSettings.nodes.basisSize = 0.01
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

disp_data = mbs.GetSensorStoredData(sensor_displacement)
rot_data = mbs.GetSensorStoredData(sensor_rotation)

if len(disp_data) > 0:
    final_disp = disp_data[-1, 1:4]
    final_rot = rot_data[-1, 1:4]
    print(f"末端最终位移 (mm): [{final_disp[0]*1000:.4f}, {final_disp[1]*1000:.4f}, {final_disp[2]*1000:.4f}]")
    print(f"末端最终旋转 (deg): [{np.rad2deg(final_rot[0]):.4f}, {np.rad2deg(final_rot[1]):.4f}, {np.rad2deg(final_rot[2]):.4f}]")
else:
    print("警告: 传感器无数据")

print(f"\n仿真结果: {'成功' if success else '失败'}")
print("=" * 60)
