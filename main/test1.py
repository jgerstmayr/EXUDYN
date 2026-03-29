import sys
import os
sys.exudynFast = True
sys.exudynCPUhasAVX2 = True

import exudyn as exu
import exudyn.graphics as graphics
import numpy as np
from exudyn.utilities import InertiaSphere, InertiaCylinder
from exudyn.itemInterface import (
    MarkerBodyRigid,
    NodeGenericData,
    ObjectContactCircleCircle,
    SensorBody,
    SensorObject,
    GenericJoint,
    VObjectJointGeneric,
    RevoluteJointZ,
)

def main():
    """
    两个圆互相接触：下面的圆旋转，上面的圆旋转中心在下圆边缘
    
    摩擦力数值稳定性修复说明：
    1. 初始穿透减小到1微米（-1e-6m），避免大初始力
    2. 刚度降低到5e3（原1e5），减小接触力刚性
    3. 阻尼增大到刚度的50%（过阻尼），快速耗散振荡
    4. 摩擦系数降低到0.01（原0.1），减小摩擦力矩
    5. spectralRadius增大到0.8，增加数值耗散
    6. 关闭初始加速度计算，提高稳定性
    """
    use_graphics = True

    sc = exu.SystemContainer()
    mbs = sc.AddSystem()

    # ========== 参数设置 ==========
    # 下圆（驱动圆）
    lower_radius = 0.5  # 下圆半径 (m)
    lower_mass = 10.0  # 质量 (kg)
    lower_angular_vel = -1 * np.pi  # 角速度 (rad/s)，降低以观察初期行为
    
    # 上圆（从动圆）
    upper_radius = 2  # 上圆半径 (m)（减小半径使旋转更明显）
    upper_mass = 20.0  # 质量 (kg)
    # 凸-凹测试：小圆在大圆内
    concave_mode = os.environ.get('EXU_CONCAVE', '1') == '1'
    
    # 接触参数（高刚度需要高阻尼）
    contact_stiffness = 1e3  # N/m（大幅降低刚度）
    contact_damping = 1e4  # N/(m/s）- 高刚度需要高阻尼来抑制振荡
    
    # 摩擦参数（可通过环境变量 EXU_MU 覆盖）
    # 摩擦力方向已修复，使用适中参数
    mu = float(os.environ.get('EXU_MU', '0.1'))  # 库仑摩擦系数 μ [-]
    friction_coefficient = mu
    friction_velocity_penalty = 1e2   # 速度惩罚系数 [N/(m/s)] - 降低以减少振荡
    friction_proportional_zone = 0.1  # 正则化区域 [m/s] - 增大以平滑零速度附近的摩擦力
    # 法向接触刚度（可通过环境变量 EXU_KN 覆盖）
    # 使用适中刚度保持数值稳定性
    kn = float(os.environ.get('EXU_KN', '1e7'))  # 适中刚度
    
    print(f"运行时参数: mu={mu}, kn={kn}")
    
    o_ground = mbs.CreateGround()
    
    # ========== 创建下圆（旋转驱动圆） ==========
    # 凸-凹模式：小圆位于大圆内部边界附近
    # 修复：大幅减小初始穿透，避免过大的初始接触力导致反弹分离
    initial_gap = 0.0001 if concave_mode else -0.0001  # 正值: 凸-凹，负值: 凸-凸，0.001mm穿透
    if concave_mode:
        upper_center = [0, 0, 0]
        lower_center = [upper_radius - lower_radius + initial_gap, 0, 0]
    else:
        # 凸-凸：小圆在原点
        lower_center = [0, 0, 0]
        upper_center = [lower_radius + upper_radius + initial_gap, 0, 0]
    
    print(f"初始间隙设置: {initial_gap}m (负值=穿透)")
    
    # 生成下圆的轮廓曲线（圆形）
    n_segments_lower = 1000
    segments_lower = np.zeros((n_segments_lower, 4))
    for i in range(n_segments_lower):
        angle1 = 2 * np.pi * i / n_segments_lower
        angle2 = 2 * np.pi * (i + 1) / n_segments_lower
        x1 = lower_radius * np.cos(angle1)
        y1 = lower_radius * np.sin(angle1)
        x2 = lower_radius * np.cos(angle2)
        y2 = lower_radius * np.sin(angle2)
        segments_lower[i, :] = [x1, y1, x2, y2]
    
    # 下圆图形（添加径向线条以显示旋转）
    lower_graphics = [
        graphics.Circle(point=[0, 0, 0], radius=lower_radius, color=[0.2, 0.2, 0.8, 1]),
        graphics.Circle(point=[0, 0, 0], radius=0.02, color=[1, 0, 0, 1])  # 中心标记
    ]
    
    # 添加径向线条以显示旋转
    for i in range(8):
        angle = i * np.pi / 4
        x_end = lower_radius * np.cos(angle)
        y_end = lower_radius * np.sin(angle)
        lower_graphics.append(graphics.Lines([[0, 0, 0], [x_end, y_end, 0]], color=[0.2, 0.2, 0.8, 1]))
    
    o_lower = mbs.CreateRigidBody(
        referencePosition=lower_center,
        initialAngularVelocity=[0, 0, lower_angular_vel],  # 初始角速度
        inertia=InertiaCylinder(lower_mass, lower_radius, 0.01, 2),  # 薄圆盘
        gravity=[0, 0, 0],
        graphicsDataList=lower_graphics,
    )
    
    # 下圆的marker（用于曲线和约束）
    m_lower = mbs.AddMarker(MarkerBodyRigid(bodyNumber=o_lower, localPosition=[0, 0, 0]))
    
    # 固定下圆的位置，但允许Z轴旋转 (constrainedAxes=[1,1,1,1,1,0])
    m_ground = mbs.AddMarker(MarkerBodyRigid(bodyNumber=o_ground, localPosition=lower_center))
    mbs.AddObject(GenericJoint(
        markerNumbers=[m_ground, m_lower],
        constrainedAxes=[0, 0, 1, 1, 1, 0],  # 约束XYZ平动和XY旋转，Z旋转自由1
        visualization=VObjectJointGeneric(axesRadius=0.02, axesLength=0.1)
    ))
    
    print(f"下圆: 半径={lower_radius}m, 角速度={lower_angular_vel}rad/s")
    
    # ========== 创建上圆（从动圆/凹槽） ==========
    if not concave_mode:
        # 凸-凸：上圆在右侧，边界接触
        upper_center = [lower_radius + upper_radius + initial_gap, 0, 0]
    
    # 上圆图形（添加径向线条以显示旋转）
    upper_graphics = [
        graphics.Circle(point=[0, 0, 0], radius=upper_radius, color=[0.9, 0.1, 0.1, 1]),
        graphics.Circle(point=[0, 0, 0], radius=0.02, color=[1, 1, 0, 1])  # 中心标记
    ]
    
    # 添加径向线条以显示旋转
    for i in range(6):
        angle = i * np.pi / 3
        x_end = upper_radius * np.cos(angle)
        y_end = upper_radius * np.sin(angle)
        upper_graphics.append(graphics.Lines([[0, 0, 0], [x_end, y_end, 0]], color=[0.9, 0.1, 0.1, 1]))

    o_upper = mbs.CreateRigidBody(
        referencePosition=upper_center,
        inertia=InertiaCylinder(upper_mass, upper_radius, 0.01, 2),
        gravity=[0, 0, 0],
        graphicsDataList=upper_graphics,
    )
    
    m_upper = mbs.AddMarker(MarkerBodyRigid(bodyNumber=o_upper, localPosition=[0, 0, 0]))
    
    # 使用RevoluteJointZ约束上圆：中心位置固定在空间中，只允许绕Z轴旋转
    # 上圆通过接触力与下圆相互作用，摩擦力驱动上圆旋转
    m_ground_upper = mbs.AddMarker(MarkerBodyRigid(bodyNumber=o_ground, localPosition=upper_center))
    mbs.AddObject(RevoluteJointZ(
        markerNumbers=[m_ground_upper, m_upper]
    ))
    
    print(f"上圆: 半径={'-' if concave_mode else ''}{upper_radius}m, 中心在({upper_center[0]}, {upper_center[1]})")
    
    # ========== 创建接触：下圆 vs 上圆 ==========
    print("\n创建接触：下圆 vs 上圆")
    
    # 使用圆-圆接触模型
    contact_damping_actual = 1e5  # 使用刚度的50%作为阻尼（过阻尼，提高稳定性）
    print(f"实际接触阻尼: {contact_damping_actual} N/(m/s)")

    node_contact = mbs.AddNode(NodeGenericData(
        initialCoordinates=[0., 0., 0., 0., 0., 0., 0., 0.],  # 8个坐标（含Bristle粘滞位置）
        numberOfDataCoordinates=8
    ))

    radius_upper_effective = -upper_radius if concave_mode else upper_radius

    obj_contact = mbs.AddObject(ObjectContactCircleCircle(
        markerNumbers=[m_lower, m_upper],
        nodeNumber=node_contact,
        radius1=lower_radius,
        radius2=radius_upper_effective,
        contactStiffness=kn,
        contactDamping=contact_damping_actual,
        frictionCoefficient=friction_coefficient,
        frictionVelocityPenalty=friction_velocity_penalty,
        frictionProportionalZone=friction_proportional_zone,
        activeConnector=True
    ))
    
    # ========== 添加传感器 ==========
    sensor_lower_pos = mbs.AddSensor(SensorBody(
        bodyNumber=o_lower, 
        outputVariableType=exu.OutputVariableType.Rotation,
        storeInternal=True
    ))
    
    sensor_upper_pos = mbs.AddSensor(SensorBody(
        bodyNumber=o_upper, 
        outputVariableType=exu.OutputVariableType.Position,
        storeInternal=True
    ))
    
    sensor_upper_rot = mbs.AddSensor(SensorBody(
        bodyNumber=o_upper, 
        outputVariableType=exu.OutputVariableType.Rotation,
        storeInternal=True
    ))
    
    sensor_contact = mbs.AddSensor(SensorObject(
        objectNumber=obj_contact,
        outputVariableType=exu.OutputVariableType.ForceLocal,
        storeInternal=True
    ))


    
    sensor_contact_gap = mbs.AddSensor(SensorObject(
        objectNumber=obj_contact,
        outputVariableType=exu.OutputVariableType.DisplacementLocal,
        storeInternal=True
    ))
    
    # 添加角速度传感器来计算切向速度
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
    
    # 注意：Position/Velocity 输出需库支持；当前仅使用已支持的 ForceLocal

    mbs.Assemble()
    
    # ========== 仿真设置 ==========
    settings = exu.SimulationSettings()
    settings.solutionSettings.writeSolutionToFile = False
    settings.solutionSettings.solutionInformation = "Two circles contact simulation"
    settings.solutionSettings.sensorsWritePeriod = 0.001  # 每1ms记录一次传感器数据
    
    # 时间积分设置 - 自适应步长控制
    steps_env = os.environ.get('EXU_STEPS')
    settings.timeIntegration.numberOfSteps = int(steps_env) if steps_env is not None else 1000000000
    end_env = os.environ.get('EXU_END')
    settings.timeIntegration.endTime = float(end_env) if end_env is not None else 0.5  # 只跑0.5秒观察初期
    settings.timeIntegration.verboseMode = 0
    settings.displayComputationTime = True
    settings.displayStatistics = True
    
    # 使用广义alpha积分器
    # 修复：关闭初始加速度计算，增大数值耗散
    settings.timeIntegration.generalizedAlpha.computeInitialAccelerations = False  # 关闭以提高稳定性
    sr_env = os.environ.get('EXU_SPECTRAL')
    settings.timeIntegration.generalizedAlpha.spectralRadius = float(sr_env) if sr_env is not None else 0.8  # 增大到0.8（更多数值耗散）
    if os.environ.get('EXU_NEWMARK','0') == '1':
        settings.timeIntegration.generalizedAlpha.useNewmark = True
        settings.timeIntegration.generalizedAlpha.newmarkGamma = 0.5
        settings.timeIntegration.generalizedAlpha.newmarkBeta = 0.25
    
    # 牛顿法设置
    settings.timeIntegration.newton.useModifiedNewton = True  # 采用标准牛顿
    settings.timeIntegration.newton.numericalDifferentiation.forODE2connectors = True  # 启用连接器数值雅可比
    settings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 1e-6    # 数值差分步长
    # 修复：放宽容差以提高收敛性
    settings.timeIntegration.newton.absoluteTolerance = 1e-2
    settings.timeIntegration.newton.relativeTolerance = 1e-2
    settings.timeIntegration.newton.maxIterations = 20  # 限制迭代次数，避免过度迭代
    
    # 自适应步长设置（关键）
    settings.timeIntegration.adaptiveStep = True
    settings.timeIntegration.initialStepSize = 1e-3  # 初始步长（会自动减小）
    settings.timeIntegration.minimumStepSize = 1e-12  # 最小步长（达到此值才会退出）
    settings.timeIntegration.adaptiveStepDecrease = 0.25  # 失败时步长减小因子（更激进）
    settings.timeIntegration.adaptiveStepIncrease = 2.0  # 成功后步长增大因子
    settings.timeIntegration.adaptiveStepRecoveryIterations = 10  # 允许更多迭代
    settings.timeIntegration.adaptiveStepRecoverySteps = 10  # 恢复步数
    
    # 线性求解器设置
    settings.linearSolverType = exu.LinearSolverType.EigenSparse

    # 接触调试设置
    settings.solutionSettings.writeSolutionToFile = False
    settings.solutionSettings.solutionInformation = "Two circles contact simulation with debug info"
    
    # 可视化设置
    sc.visualizationSettings.window.renderWindowSize = [1200, 1200]
    sc.visualizationSettings.general.graphicsUpdateInterval = 0.02
    sc.visualizationSettings.general.backgroundColor = [0.95, 0.95, 0.95, 1.0]
    
    # 接触力可视化
    sc.visualizationSettings.contact.showContactForces = True
    sc.visualizationSettings.contact.contactForcesFactor = 1e-4  # 箭头缩放
    sc.visualizationSettings.contact.contactPointsDefaultSize = 5.0
    sc.visualizationSettings.contact.showContactForcesValues = True
    sc.visualizationSettings.connectors.showContact = True

    print(f"\n运行时参数:")
    print(f"  摩擦系数 mu={mu}")
    print(f"  接触刚度 kn={kn}")
    print(f"  接触阻尼 damping={contact_damping_actual}")
    print(f"  初始间隙 gap={initial_gap}m")
    print(f"  仿真步数 steps={settings.timeIntegration.numberOfSteps}")
    print(f"  仿真时长 endTime={settings.timeIntegration.endTime}s")
    print(f"  谱半径 spectralRadius={settings.timeIntegration.generalizedAlpha.spectralRadius}")
    print(f"  自适应步长 adaptive={settings.timeIntegration.adaptiveStep}")
    print(f"  初始步长 initialStepSize={settings.timeIntegration.initialStepSize}")
    print("\n开始仿真...")
    if use_graphics:
        sc.renderer.Start()

    try:
        mbs.SolveDynamic(settings, showHints=True)
        print("\n仿真成功完成！")
    except Exception as e:
        print(f"\n仿真出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if use_graphics:
            try:
                sc.renderer.Stop()
            except:
                pass

    # ========== 输出结果 ==========
    print("\n" + "="*60)
    print("结果分析：")
    print("="*60)
    
    # 下圆旋转
    data_lower = np.asarray(mbs.GetSensorStoredData(sensor_lower_pos))
    if data_lower.size > 0:
        print(f"\n下圆旋转角度 (最后5行):")
        for row in data_lower[-5:]:
            angle_deg = np.degrees(row[3])  # 假设第4列是Z轴旋转角
            print(f"  t={row[0]:.3f}s: 角度={angle_deg:.1f}°")
    
    # 上圆位置
    data_upper_pos = np.asarray(mbs.GetSensorStoredData(sensor_upper_pos))
    if data_upper_pos.size > 0:
        print(f"\n上圆位置 (最后5行):")
        for row in data_upper_pos[-5:]:
            print(f"  t={row[0]:.3f}s: x={row[1]:.4f}, y={row[2]:.4f}")
    
    # 上圆旋转
    data_upper_rot = np.asarray(mbs.GetSensorStoredData(sensor_upper_rot))
    if data_upper_rot.size > 0:
        print(f"\n上圆旋转角度 (最后5行):")
        for row in data_upper_rot[-5:]:
            angle_deg = np.degrees(row[3])
            print(f"  t={row[0]:.3f}s: 角度={angle_deg:.1f}°")
    
    # 接触调试数据
    contact_data = np.asarray(mbs.GetSensorStoredData(sensor_contact))

    gap_data = np.asarray(mbs.GetSensorStoredData(sensor_contact_gap))

    if contact_data.size > 0:
        print("\n接触力 (最后5行):")
        for row in contact_data[-5:]:
            fx, fy = row[1], row[2]
            force_mag = np.sqrt(fx**2 + fy**2)
            print(f"  t={row[0]:.3f}s: Fx={fx:.2f}N, Fy={fy:.2f}N, |F|={force_mag:.2f}N")

    min_len = min([
        contact_data.shape[0] if contact_data.size > 0 else 0,

        gap_data.shape[0] if gap_data.size > 0 else 0,
    ])

                
                # 计算接触点切向速度
                # 接触点在下圆边缘，切向速度 = omega_lower * radius


    print("\n系统说明：")
    print("  - 下圆（蓝色）: 固定中心，绕Z轴旋转")
    print("  - 上圆（红色折线近似）: 旋转中心在下圆边缘，通过接触被驱动")
    print("  - 圆与折线近似圆之间通过接触力传递运动")


if __name__ == "__main__":
    main()
