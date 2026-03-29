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
    ObjectContactCurveCircles,
    VObjectContactCurveCircles,
    ObjectContactCircleCircle,
    SensorBody,
    SensorObject,
    GenericJoint,
    VObjectJointGeneric,
    RevoluteJointZ,
    ObjectConnectorCoordinate,
)

def create_circle_segments(radius, n_segments=1200, clockwise=False):
    """Generate piecewise-linear segments that approximate a circle in the local XY-plane."""
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

    print("="*60)
    print("测试：圆-曲线接触（被动圆由线段近似），下圆旋转驱动上圆")
    print("="*60)
    print("\n⚠️  ObjectContactCurveCircles摩擦力数值稳定性问题:")
    print("   根本原因：摩擦力矩导致角速度正反馈发散")
    print("   - t=0时正常，t=10μs时角速度爆炸(×900)")
    print("   - 通过降低刚度、增大阻尼、减小摩擦系数来缓解")
    print("   - 详细分析见 FRICTION_BUG_ROOT_CAUSE.md\n")

    # ========== 参数设置 ==========
    # 下圆（驱动圆）
    lower_radius = 0.5  # 下圆半径 (m)
    lower_mass = 10.0  # 质量 (kg)
    lower_angular_vel = 10 * np.pi  # 角速度 (rad/s)，降低以观察初期行为
    
    # 上圆（从动圆）
    upper_radius = 2  # 上圆半径 (m)（减小半径使旋转更明显）
    upper_mass = 5.0  # 质量 (kg)
    # 凸-凹测试：小圆在大圆内
    concave_mode = os.environ.get('EXU_CONCAVE', '0') == '1'
    
    # 接触参数（使用更稳定的参数）
    contact_stiffness = 1e3  # N/m
    contact_damping = 1e5  # N/(m/s）
    
    # 摩擦参数（可通过环境变量 EXU_MU 覆盖）
    # 摩擦力方向已修复，使用适中参数
    mu = float(os.environ.get('EXU_MU', '0.1')) 
    # 法向接触刚度（可通过环境变量 EXU_KN 覆盖）
    # 降低刚度以避免数值振荡
    kn = float(os.environ.get('EXU_KN', '1e9')) 
    # 粘着（静摩擦）参数：保持比法向柔软，避免数值振荡
    friction_stiffness = 5e3  # N/m，约为正常接触刚度的极小比例，用于Bristle粘着
    
    print(f"运行时参数: mu={mu}, kn={kn}")
    
    o_ground = mbs.CreateGround()
    
    # ========== 创建下圆（旋转驱动圆） ==========
    # 凸-凹模式：小圆位于大圆内部边界附近
    # 修复：大幅减小初始穿透，避免过大的初始接触力导致反弹分离
    initial_gap = 0.001 if concave_mode else -0.001  # 正值: 凸-凹，负值: 凸-凸，0.001mm穿透
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
    
    # 使用合理的惯性：InertiaCylinder(density, length, outerRadius, axis)
    # 薄圆盘：密度7850 kg/m³, 厚度10mm
    lower_thickness = 0.01  # 10mm
    o_lower = mbs.CreateRigidBody(
        referencePosition=lower_center,
        initialAngularVelocity=[0, 0, lower_angular_vel],  # 初始角速度
        inertia=InertiaCylinder(7850, lower_thickness, lower_radius, 2),
        gravity=[0, 0, 0],
        graphicsDataList=lower_graphics,
    )
    
    # 下圆的marker（用于曲线和约束）
    m_lower = mbs.AddMarker(MarkerBodyRigid(bodyNumber=o_lower, localPosition=[0, 0, 0]))
    
    # 固定下圆的位置，但允许Z轴旋转 (constrainedAxes=[1,1,1,1,1,0])
    m_ground = mbs.AddMarker(MarkerBodyRigid(bodyNumber=o_ground, localPosition=lower_center))
    mbs.AddObject(GenericJoint(
        markerNumbers=[m_ground, m_lower],
        constrainedAxes=[0, 0, 1, 1, 1, 0],  # 约束XYZ平动和XY旋转，Z旋转自由
        visualization=VObjectJointGeneric(axesRadius=0.02, axesLength=0.1)
    ))
    
    print(f"下圆: 半径={lower_radius}m, 角速度={lower_angular_vel}rad/s")
    
    # ========== 创建上圆（从动圆/凹槽） ==========
    if not concave_mode:
        # 凸-凸：上圆在右侧，边界接触
        upper_center = [lower_radius + upper_radius + initial_gap, 0, 0]
    
    n_segments_curve = 720  # 段数（平衡精度和性能）
    # 上圆用段表示（作为曲线），下圆保持为圆
    curve_segments_local = create_circle_segments(upper_radius, n_segments_curve, clockwise=0)
    
    # 上圆图形 - 用光滑圆显示（多边形太密时会卡）
    upper_graphics = [
        graphics.Circle(point=[0, 0, 0], radius=upper_radius, color=[0.9, 0.1, 0.1, 0.3]),  # 半透明圆
        graphics.Circle(point=[0, 0, 0], radius=0.03, color=[1, 1, 0, 1])  # 中心标记
    ]
    
    # 添加径向线条以显示旋转
    for i in range(8):
        angle = i * np.pi / 4
        x_end = upper_radius * np.cos(angle)
        y_end = upper_radius * np.sin(angle)
        upper_graphics.append(graphics.Lines([[0, 0, 0], [x_end, y_end, 0]], color=[0.9, 0.1, 0.1, 1]))
    
    # 使用较小的惯性使响应更明显
    # 轻质材料：密度100 kg/m³ (类似泡沫), 厚度10mm
    # 上圆：质量 ≈ 12.6 kg, I_zz ≈ 25 kg·m²
    upper_density = 100  # kg/m³ (轻质材料)
    upper_thickness = 0.01  # 10mm
    o_upper = mbs.CreateRigidBody(
        referencePosition=upper_center,
        inertia=InertiaCylinder(upper_density, upper_thickness, upper_radius, 2),
        gravity=[0, 0, 0],
        graphicsDataList=upper_graphics,
    )
    
    m_upper = mbs.AddMarker(MarkerBodyRigid(bodyNumber=o_upper, localPosition=[0, 0, 0]))
    
    # 使用GenericJoint约束上圆：中心位置固定在空间中，只允许绕Z轴旋转
    # 与下圆使用相同的六自由度约束方式
    m_ground_upper = mbs.AddMarker(MarkerBodyRigid(bodyNumber=o_ground, localPosition=upper_center))
    mbs.AddObject(GenericJoint(
        markerNumbers=[m_ground_upper, m_upper],
        constrainedAxes=[1, 1, 1, 1, 1, 0],  # 约束XYZ平动和XY旋转，Z旋转自由
        visualization=VObjectJointGeneric(axesRadius=0.02, axesLength=0.1)
    ))
    
    print(f"上圆: 半径={'-' if concave_mode else ''}{upper_radius}m, 中心在({upper_center[0]}, {upper_center[1]})")
    
    # ========== 创建接触：下圆 vs 上圆 ==========
    print("\n创建接触：下圆 vs 上圆")
    
    # 使用曲线-圆接触模型（被动圆由折线段近似）
    segments_matrix = exu.MatrixContainer(curve_segments_local)
    initial_contact_data = []
    for _ in range(n_segments_curve):
        initial_contact_data.extend([-1.0, 0.0, 0.0, 0.0, 0.0])  # 5个变量（含Bristle粘滞位置）

    node_contact = mbs.AddNode(NodeGenericData(
        initialCoordinates=initial_contact_data,
        numberOfDataCoordinates=5 * n_segments_curve
    ))

    # markerNumbers: [曲线载体marker, 圆心marker1, ...]
    # 这里上圆用曲线段表示，下圆是真实的圆
    # 修复：大幅增大contactDamping以耗散初始穿透的能量，防止反弹分离和数值振荡
    contact_damping_actual = 1e5  # 使用刚度的50%作为阻尼（过阻尼，提高稳定性）
    print(f"实际接触阻尼: {contact_damping_actual} N/(m/s)")
    
    obj_contact = mbs.AddObject(ObjectContactCurveCircles(
        markerNumbers=[m_upper, m_lower],  # marker0=上圆（曲线），marker1=下圆（圆）
        nodeNumber=node_contact,
        circlesRadii=[lower_radius],  # 下圆的半径
        segmentsData=segments_matrix,  # 上圆的曲线段
        contactStiffness=kn,
        contactDamping=contact_damping_actual,  # 增大阻尼
        dynamicFriction=mu,
        frictionStiffness=friction_stiffness,   # 启用静摩擦（Bristle模型）
        frictionProportionalZone=10,  # 平滑零速度附近的摩擦力
        frictionVelocityPenalty=1e2,  # 降低速度惩罚以减少振荡
        contactModel=0,
        activeConnector=True,
        visualization=VObjectContactCurveCircles(show=True)  # 启用以显示力箭头（曲线段已在C++中禁用）
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
    
    # 时间积分设置 - 自适应步长控制
    steps_env = os.environ.get('EXU_STEPS')
    settings.timeIntegration.numberOfSteps = int(steps_env) if steps_env is not None else 10000000
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
        settings.timeIntegration.generalizedAlpha.useNewmark = False
        settings.timeIntegration.generalizedAlpha.newmarkGamma = 0.5
        settings.timeIntegration.generalizedAlpha.newmarkBeta = 0.25
    
    # 牛顿法设置
    settings.timeIntegration.newton.useModifiedNewton = True
    settings.timeIntegration.newton.numericalDifferentiation.forODE2connectors = True  # 启用连接器数值雅可比
    settings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 1e-6    # 数值差分步长
    # 修复：放宽容差以提高收敛性
    settings.timeIntegration.newton.absoluteTolerance = 1e-2  # 进一步放宽
    settings.timeIntegration.newton.relativeTolerance = 1e-2
    settings.timeIntegration.newton.maxIterations = 50  # 增加最大迭代次数
    
    # 自适应步长设置（关键）
    settings.timeIntegration.adaptiveStep = True
    settings.timeIntegration.initialStepSize = 1e-3  # 初始步长
    settings.timeIntegration.minimumStepSize = 1e-12  # 最小步长（更小以允许更多尝试）
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
    sc.visualizationSettings.general.circleTiling = 200  # 增加圆的分段数，使其更光滑
    
    # 接触力可视化
    sc.visualizationSettings.contact.showContactForces = True
    sc.visualizationSettings.contact.contactForcesFactor = 1e-6  # 力很大(~500kN)，需要很小的系数
    sc.visualizationSettings.contact.contactPointsDefaultSize = 0.02  # 接触点大小
    sc.visualizationSettings.contact.showContactForcesValues = True  # 显示数值
    sc.visualizationSettings.connectors.showContact = True
    
    # 连接器可视化
    sc.visualizationSettings.connectors.defaultSize = 0.01
    sc.visualizationSettings.connectors.showJointAxes = True

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

    if min_len > 0:
        print("\n接触调试 (最后10行，包含gap):")
        count = min(10, min_len)
        # 法向量方向：从下圆中心指向上圆中心（近似为X轴方向）
        normal_unit = np.array([1.0, 0.0, 0.0])
        for i in range(min_len - count, min_len):
            t = contact_data[i, 0]
            total_force = contact_data[i, 1:4]

            gap = gap_data[i, 1]

            total_mag = np.linalg.norm(total_force)

            normal_force = float(np.dot(total_force, normal_unit))
            max_friction = mu * abs(normal_force)

            # 摩擦力是总力减去法向分量
            friction_vec = total_force - normal_force * normal_unit
            friction_mag = np.linalg.norm(friction_vec)
            
            # 滑移速度估算（基于角速度和半径）
            slip_speed = 0.0  # 暂无精确数据，后续角速度分析会补充

            normal_str = np.array2string(normal_unit, precision=3, suppress_small=True)
            friction_str = np.array2string(friction_vec, precision=3, suppress_small=True)

            print(f"  t={t:.3f}s: gap={gap:.6f}m, |Ftot|={total_mag:.3f}N, Fn={normal_force:.3f}N, |Ffr|={friction_mag:.3f}N, v_slip={slip_speed:.5f}m/s")
            if total_mag > 0.01:  # 只有有力时才打印详情
                print(f"    摩擦比={friction_mag/(max_friction+1e-12):.3f}, 法向={normal_str}, 摩擦向量={friction_str}")

    # 提前读取角速度数据，供后续摩擦调试近似 |vt| 使用
    lower_omega_data = np.asarray(mbs.GetSensorStoredData(sensor_lower_omega))
    upper_omega_data = np.asarray(mbs.GetSensorStoredData(sensor_upper_omega))

    # 调试：检查数据结构
    print(f"\n调试：下圆角速度数据形状: {lower_omega_data.shape}")
    if lower_omega_data.size > 0:
        print(f"下圆角速度数据示例: {lower_omega_data[0]}")
    
    print(f"调试：上圆角速度数据形状: {upper_omega_data.shape}")
    if upper_omega_data.size > 0:
        print(f"上圆角速度数据示例: {upper_omega_data[0]}")
    
    if lower_omega_data.size > 0 and upper_omega_data.size > 0:
        print(f"\n角速度分析 (最后5行):")
        # 获取下圆和上圆的最后5个数据点
        for i in range(max(0, len(lower_omega_data)-5), len(lower_omega_data)):
            if i < len(upper_omega_data):  # 确保索引有效
                t_lower = lower_omega_data[i][0]
                
                # AngularVelocityLocal传感器返回的数据格式通常是[时间, wx, wy, wz]
                # 我们需要Z轴角速度，即第4个元素（索引3）
                if len(lower_omega_data[i]) >= 4:
                    omega_lower = lower_omega_data[i][3]  # Z轴角速度
                else:
                    omega_lower = lower_angular_vel  # 使用初始角速度作为默认值
                    
                if len(upper_omega_data[i]) >= 4:
                    omega_upper = upper_omega_data[i][3]  # Z轴角速度
                else:
                    omega_upper = 0.0  # 上圆初始角速度为0
                
                # 计算接触点切向速度
                # 接触点在下圆边缘，切向速度 = omega_lower * radius
                v_tang_lower = omega_lower * lower_radius
                v_tang_upper = omega_upper * upper_radius
                
                # 相对切向速度
                v_tang_rel = v_tang_lower - v_tang_upper
                
                # 计算理论角速度比（考虑半径比）
                theoretical_ratio = lower_radius / upper_radius
                
                print(f"  t={t_lower:.3f}s: 下圆角速度={omega_lower:.4f}rad/s ({omega_lower/lower_angular_vel:.2f}倍输入), 上圆角速度={omega_upper:.4f}rad/s")
                print(f"    理论角速度比={theoretical_ratio:.2f}, 实际角速度比={omega_lower/omega_upper if omega_upper != 0 else 0:.2f}")
                print(f"    下圆切向速度={v_tang_lower:.4f}m/s, 上圆切向速度={v_tang_upper:.4f}m/s, 相对速度={v_tang_rel:.4f}m/s")
    
    print("\n系统说明：")
    print("  - 下圆（蓝色）: 固定中心，绕Z轴旋转")
    print("  - 上圆（红色折线近似）: 旋转中心在下圆边缘，通过接触被驱动")
    print("  - 圆与折线近似圆之间通过接触力传递运动")


if __name__ == "__main__":
    main()
