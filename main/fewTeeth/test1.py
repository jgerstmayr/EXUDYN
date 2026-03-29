#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Minimal single-tooth contact test bench for ObjectContactFewTeeth.

This script isolates one inner/outer flank pair and positions the outer gear
body relative to the inner gear so that you can quickly inspect gap/force
values without running the full transmission model.  Use --help for options.
"""
import sys, os

sys.exudynFast = True          # 让 exudyn 优先加载 fast 版（依赖 AVX2）
sys.exudynCPUhasAVX2 = True 

import argparse
import math

import numpy as np

import exudyn as exu
import exudyn.graphics as graphics
from exudyn.utilities import RotationMatrixZ, RigidBodyInertia
from exudyn.itemInterface import (
    GenericJoint,
    LoadTorqueVector,
    MarkerBodyRigid,
    MarkerNodeCoordinate,
    NodeGenericData,
    NodePointGround,
    ObjectContactFewTeeth,
    CoordinateConstraint,
    SensorNode,
    VObjectContactFewTeeth,
    VObjectJointGeneric,
)

# Contact visualization toggles for consistent arrow output
# SHOW_CONTACT_GEOMETRY = True  # 必须为True才能显示ContactCurveCircles的力箭头
# CONTACT_FORCE_SCALE = 3e-3  # enlarge contact force arrows for visibility
# SHOW_CONTACT_FORCE_VALUES = True
# CONTACT_POINT_SIZE = 1.5e-3
from fewTeeth_profile import FewTeethGearPair, generate_few_teeth_flanks
from tooth_stiffness import create_stiffness_table_for_gear

def flank_polyline(points2d, translation=(0.0, 0.0), z=0.0):
    """Convert 2D flank points to a 3D polyline for graphics."""
    if points2d.size == 0:
        return None
    data = []
    tx, ty = translation
    for pt in points2d:
        data.append([pt[0] + tx, pt[1] + ty, z])
    return data

def build_flank_graphics(points2d, *, translation=(0.0, 0.0), color=(0.2, 0.2, 0.8, 1.0), z=0.0, line_width=2.0):
    polyline = flank_polyline(points2d, translation=translation, z=z)
    if polyline is None:
        return []
    return [graphics.Lines(polyline, color=color)]


def build_outer_analytic(params, flank_side, *, rotation_offset=0.0,
                         replicate_all=True, single_tooth_index=0):
    """Replicate the helper from fewTeeth.py for convenience."""
    flank_sign = 1.0 if flank_side == "right" else -1.0
    reverse_param = flank_side == "left"
    return {
        "isInner": False,
        "replicateAllTeeth": replicate_all,
        "numberOfTeeth": int(params.z_outer),
        "singleToothIndex": int(single_tooth_index),
        "module": float(params.module),
        "alpha": float(params.alpha),
        "haStar": float(params.ha_star),
        "cStar": float(params.c_star),
        "x": float(params.x_outer),
        "deltaY": 0.0,
        "baseRotation": float(rotation_offset),
        "flankSign": flank_sign,
        "reverseParam": reverse_param,
    }


def build_inner_analytic(params, geometry, tooth_index, flank_side):
    """Match the inner flank helper logic used in the main model."""
    flank_sign = 1.0 if flank_side == "right" else -1.0
    reverse_param = flank_side == "left"
    return {
        "isInner": True,
        "replicateAllTeeth": False,
        "numberOfTeeth": int(params.z_inner),
        "singleToothIndex": int(tooth_index),
        "module": float(params.module),
        "alpha": float(params.alpha),
        "haStar": float(params.ha_star),
        "cStar": float(params.c_star),
        "x": float(params.x_inner),
        "deltaY": float(geometry.delta_y),
        "baseRotation": float(math.pi / params.z_inner),
        "flankSign": flank_sign,
        "reverseParam": reverse_param,
    }


def create_contact_test(args):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    oGround = mbs.CreateGround()
    # 内外齿地面marker将在center计算后创建

    gear_params = FewTeethGearPair(
        z_outer=args.z_outer,
        z_inner=args.z_inner,
        module=args.module,
        x_outer=args.x_outer,
        x_inner=args.x_inner,
        alpha_deg=args.alpha,
        ha_star=args.ha_star,
        c_star=args.c_star,
        flank_points_outer=120,
        flank_points_inner=160,
        tip_probe_length=args.distance_threshold,
    )

    gear_flanks, geometry = generate_few_teeth_flanks(gear_params)

    # ============================================================================
    # 使用 tooth_stiffness 计算结构刚度表（不含接触刚度，接触刚度在 C++ 中实时计算）
    # ============================================================================
    face_width = 15e-3      # 齿宽 [m]
    E_modulus = 2.1e11      # 弹性模量 [Pa]
    poisson = 0.3           # 泊松比
    friction_coefficient = 0.1       # 库仑摩擦系数 μ [-]
    friction_velocity_penalty = 1e5  # 速度惩罚系数 [N/(m/s)]
    friction_proportional_zone = 0.001  # 正则化区域 [m/s]
    
    print("\n计算单齿结构刚度表...")
    stiffnessOuter_tooth = create_stiffness_table_for_gear(
        z=gear_params.z_outer,
        module=gear_params.module,
        face_width=face_width,
        alpha_deg=gear_params.alpha_deg,
        ha_star=gear_params.ha_star,
        c_star=gear_params.c_star,
        x=gear_params.x_outer,
        is_internal=False,
        E=E_modulus,
        poisson=poisson,
        n_points=25,
        include_contact=True  # 只计算结构刚度（弯曲+齿根）
    )
    
    stiffnessInner_tooth = create_stiffness_table_for_gear(
        z=gear_params.z_inner,
        module=gear_params.module,
        face_width=face_width,
        alpha_deg=gear_params.alpha_deg,
        ha_star=gear_params.ha_star,
        c_star=gear_params.c_star,
        x=gear_params.x_inner,
        is_internal=True,
        E=E_modulus,
        poisson=poisson,
        n_points=25,
        include_contact=True  # 只计算结构刚度（弯曲+齿根）
    )
    
    print(f"  外齿轮结构刚度表: {[f'{k/1e6:.1f}' for k in stiffnessOuter_tooth]} MN/m")
    print(f"  内齿轮结构刚度表: {[f'{k/1e6:.1f}' for k in stiffnessInner_tooth]} MN/m")
    print(f"  接触刚度将在 C++ 中实时计算（使用等效曲率半径）\n")

    # 收集所有外齿的图形（左右两面）
    visual_outer_flank = "right" if args.flank == "left" else "left"
    # 定义颜色方案：按接触对应关系着色
    # 内齿左面 ↔ 外齿右面 → 蓝色
    # 内齿右面 ↔ 外齿左面 → 红色
    color_left = [0.9, 0.2, 0.2, 0.7]  # 红色，半透明
    color_right = [0.1, 0.3, 0.9, 0.7]  # 蓝色，半透明
    
    outer_graphics_list = []
    # 绘制所有外齿
    print(f"绘制外齿: {gear_params.z_outer} 个齿")
    for tooth_idx in range(gear_params.z_outer):
        for flank_side in ["left", "right"]:
            outer_points = gear_flanks.get(("outer", tooth_idx, flank_side), np.zeros((0, 2)))
            if outer_points.size == 0:
                continue
            # 外齿左面对应内齿右面(红色)，外齿右面对应内齿左面(蓝色)
            color = color_left if flank_side == "left" else color_right
            width = args.line_width * 0.6  # 稍细一些以减少视觉混乱
            outer_graphics_list.extend(
                build_flank_graphics(
                    outer_points,
                    translation=(0.0, 0.0),
                    color=color,
                    z=0.0,
                    line_width=width,
                )
            )

    inner_graphics = []
    # 绘制所有内齿
    print(f"绘制内齿: {gear_params.z_inner} 个齿")
    for tooth_idx in range(gear_params.z_inner):
        for flank_side in ["left", "right"]:
            pts = gear_flanks.get(("inner", tooth_idx, flank_side), np.zeros((0, 2)))
            if pts.size == 0:
                continue
            # 内齿左面对应外齿右面(蓝色)，内齿右面对应外齿左面(红色)
            color = color_right if flank_side == "left" else color_left
            width = args.line_width * 0.6
            inner_graphics.extend(
                build_flank_graphics(
                    pts,
                    translation=(0.0, 0.0),
                    color=color,
                    z=0.0,
                    line_width=width,
                )
            )
    if not inner_graphics:
        inner_graphics = build_flank_graphics(
            np.zeros((0, 2)),
            translation=(0.0, 0.0),
            color=args.inner_color,
            z=0.0,
            line_width=args.line_width,
        )

    inner_center = np.array([args.inner_x_shift, args.inner_y_shift, args.inner_z], dtype=float)
    outer_center = np.array(
        [
            args.x_shift,
            geometry.center_distance + args.radial_shift,
            args.z_shift,
        ],
        dtype=float,
    )
    
    # 更新内外齿地面marker位置，使约束锁定在偏移后的位置
    # 注意：地面marker只能指定位置，不能指定方向
    # 内齿的旋转通过刚体的referenceRotationMatrix实现，但会被完全固定约束锁住
    mGroundInner = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                                   localPosition=inner_center.tolist()))
    mGroundOuter = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                                   localPosition=outer_center.tolist()))

    outer_inertia = RigidBodyInertia(mass=args.outer_mass, inertiaTensor=np.diag([args.outer_inertia] * 3))
    inner_inertia = RigidBodyInertia(mass=args.inner_mass, inertiaTensor=np.diag([args.inner_inertia] * 3))

    outer_body = mbs.CreateRigidBody(
        referencePosition=outer_center.tolist(),
        referenceRotationMatrix=RotationMatrixZ(math.radians(args.outer_angle)),
        initialAngularVelocity=[0, 0, 0],  # 初始角速度由CoordinateConstraint控制
        inertia=outer_inertia,
        graphicsDataList=outer_graphics_list,
        gravity=[0, 0, 0],
        nodeType=exu.NodeType.RotationRxyz,
    )
    inner_body = mbs.CreateRigidBody(
        referencePosition=inner_center.tolist(),
        referenceRotationMatrix=RotationMatrixZ(math.radians(args.inner_angle)),
        inertia=inner_inertia,
        graphicsDataList=inner_graphics,
        gravity=[0, 0, 0],
        nodeType=exu.NodeType.RotationRxyz,
    )

    mOuter = mbs.AddMarker(MarkerBodyRigid(bodyNumber=outer_body, localPosition=[0, 0, 0]))
    mInner = mbs.AddMarker(MarkerBodyRigid(bodyNumber=inner_body, localPosition=[0, 0, 0]))

    mbs.AddObject(
        GenericJoint(
            markerNumbers=[mGroundInner, mInner],
            constrainedAxes=[1, 1, 1, 1, 1, 0],
            visualization=VObjectJointGeneric(show=False),
        )
    )
    mbs.AddObject(
        GenericJoint(
            markerNumbers=[mGroundOuter, mOuter],
            constrainedAxes=[1, 1, 1, 1, 1, 0],
            visualization=VObjectJointGeneric(show=False),
        )
    )

    # 添加速度控制（如果指定了目标角速度）- 加在内齿轮上
    if abs(args.inner_omega) > 0.0:
        # 获取内齿轮节点号
        nInner = mbs.GetObject(inner_body)['nodeNumber']
        # Z轴旋转坐标（Rxyz节点的coordinate=5）
        mInnerRotZ = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nInner, coordinate=5))
        # 地面坐标标记
        nGroundCoord = mbs.AddNode(NodePointGround())
        mGroundCoord = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGroundCoord, coordinate=0))
        
        # 定义角度偏移函数（斜坡加速）
        omega_target = args.inner_omega
        ramp_time = args.ramp_time
        
        def inner_angle_offset(mbs, t, itemIndex, currentOffset):
            if t <= 0.0:
                return 0.0
            elif t < ramp_time:
                # 斜坡阶段：角度 = 0.5 * omega * t^2 / ramp_time
                return 0.5 * omega_target / ramp_time * t * t
            else:
                # 匀速阶段
                return 0.5 * omega_target * ramp_time + omega_target * (t - ramp_time)
        
        mbs.AddObject(CoordinateConstraint(
            markerNumbers=[mGroundCoord, mInnerRotZ],
            offset=0.0,
            offsetUserFunction=inner_angle_offset
        ))
        print(f"已添加内齿轮速度控制: 目标角速度={omega_target} rad/s, 斜坡时间={ramp_time} s")

    if abs(args.drive_torque) > 0.0:
        mbs.AddLoad(LoadTorqueVector(markerNumber=mOuter, loadVector=[0, 0, args.drive_torque]))

    contact_nodes = []
    contact_objects = []
    contact_count = 0
    
    # 为所有内齿创建接触对象
    print(f"开始创建接触对象...")
    for inner_tooth_idx in range(gear_params.z_inner):
        for inner_flank in ["left", "right"]:
            inner_profile = build_inner_analytic(gear_params, geometry, inner_tooth_idx, inner_flank)
            inner_pts = gear_flanks.get(("inner", inner_tooth_idx, inner_flank), np.zeros((0, 2)))
            if inner_pts.size == 0:
                continue
            # 配对关系: 内齿左面↔外齿右面, 内齿右面↔外齿左面
            outer_flank_match = "right" if inner_flank == "left" else "left"

            node = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=9, initialCoordinates=[0.0] * 9))  # 9个坐标（含Bristle粘滞位置）
            outer_profile = build_outer_analytic(
                gear_params,
                outer_flank_match,
                rotation_offset=args.outer_rotation,
                replicate_all=True,  # 使用所有外齿
                single_tooth_index=0,  # 当 replicate_all=True 时此参数无效
            )
            
            # 设置接触可视化颜色
            if inner_flank == "left":
                contact_color = [0.1, 0.3, 0.9, 0.5]  # 蓝色半透明
            else:
                contact_color = [0.9, 0.2, 0.2, 0.5]  # 红色半透明
            
            obj = mbs.AddObject(
                ObjectContactFewTeeth(
                    name=f"Contact_InnerT{inner_tooth_idx:02d}_{inner_flank[0].upper()}",
                    markerNumbers=[mOuter, mInner],
                    nodeNumber=node,
                    outerAnalytic=outer_profile,
                    innerAnalytic=inner_profile,
                    contactDamping=args.contact_damping,
                    stiffnessOuter=stiffnessOuter_tooth,  # 结构刚度表（不含接触）
                    stiffnessInner=stiffnessInner_tooth,  # 结构刚度表（不含接触）
                    faceWidth=face_width,                 # 齿宽（用于 C++ 计算接触刚度）
                    elasticModulus=E_modulus,             # 弹性模量（用于 C++ 计算接触刚度）
                    poissonRatio=poisson,                 # 泊松比（用于 C++ 计算接触刚度）
                    frictionCoefficient=friction_coefficient,
                    frictionVelocityPenalty=friction_velocity_penalty,
                    frictionProportionalZone=friction_proportional_zone,
                    contactReferenceDistance=args.reference_distance,
                    tipProbeLength=gear_params.tip_probe_length,
                    visualization=VObjectContactFewTeeth(show=True, color=contact_color),
                )
            )
            contact_nodes.append(node)
            contact_objects.append(obj)
            contact_count += 1

    if not contact_nodes:
        raise RuntimeError("No valid flank data to generate contacts.")
    else:
        print(f"✓ 创建了 {contact_count} 个接触对象（{gear_params.z_inner} 个内齿 × 2 个齿面）")
    primary_contact_node = contact_nodes[0]

    gap_sensor = mbs.AddSensor(
        SensorNode(
            nodeNumber=primary_contact_node,
            outputVariableType=exu.OutputVariableType.Coordinates,
            storeInternal=True,
            fileName="sensor_data_raw.txt",
            writeToFile=True,
        )
    )
    
    # 添加外齿旋转角度传感器
    outer_body_node = mbs.GetObject(outer_body)['nodeNumber']
    rotation_sensor = mbs.AddSensor(
        SensorNode(
            nodeNumber=outer_body_node,
            outputVariableType=exu.OutputVariableType.Rotation,
            storeInternal=True,
            fileName="outer_rotation.txt",
            writeToFile=True,
        )
    )

    SC.visualizationSettings.contact.contactPointsDefaultSize = args.point_size
    SC.visualizationSettings.contact.showContactForces = True
    SC.visualizationSettings.contact.contactForcesFactor = args.force_scale
    SC.visualizationSettings.window.renderWindowSize = [1280, 720]


    mbs.Assemble()

    started_renderer = False
    if args.render:
        exu.StartRenderer()
        started_renderer = True

    solution_file_path = os.path.abspath(args.solution_file)

    if args.skip_solver:
        sim_settings = exu.SimulationSettings()
        sim_settings.staticSolver.verboseMode = 1 if args.verbose else 0
        mbs.SolveStatic(sim_settings)
    else:
        sim_settings = exu.SimulationSettings()
        if args.replay:
            sol_dir = os.path.dirname(solution_file_path)
            if sol_dir:
                os.makedirs(sol_dir, exist_ok=True)
            sim_settings.solutionSettings.writeSolutionToFile = True
            sim_settings.solutionSettings.coordinatesSolutionFileName = solution_file_path
            sim_settings.solutionSettings.solutionWritePeriod = args.time_step
        else:
            sim_settings.solutionSettings.writeSolutionToFile = False
        sim_settings.displayComputationTime = args.verbose
        sim_settings.timeIntegration.endTime = args.sim_time
        sim_settings.timeIntegration.numberOfSteps =1000
        sim_settings.timeIntegration.verboseMode = 1
        sim_settings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
        sim_settings.timeIntegration.newton.useModifiedNewton = False
        sim_settings.timeIntegration.newton.relativeTolerance = 1e-8
        
        # 启用传感器记录
        sim_settings.solutionSettings.sensorsWritePeriod = 1e-6
        import psutil  # 可选，仅用于获取物理核心数

        sim_settings.parallel.numberOfThreads = psutil.cpu_count(logical=False)
        mbs.SolveDynamic(sim_settings)

    if args.render:
        # keep window open until user closes it
        mbs.WaitForUserToContinue()
        if started_renderer:
            exu.StopRenderer()

    data_values = mbs.GetSensorValues(gap_sensor)
    gap = data_values[0]
    contact_force = data_values[2]

    print("\n=== 摆线针轮全齿面接触仿真 ===")
    print(f"外齿数: {gear_params.z_outer} 个")
    print(f"内齿数: {gear_params.z_inner} 个")
    print(f"接触对象总数: {contact_count} 个")
    print(f"\n主监测点（内齿0号，左侧齿面）:")
    print(f"  间隙值 (m): {gap:.6e}")
    print(f"  法向力 (N): {contact_force:.6e}")
    
    # 将传感器历史数据保存到文件
    if not args.skip_solver:
        sensor_data = mbs.GetSensorStoredData(gap_sensor)
        output_file = os.path.join(os.getcwd(), "sensor_data.txt")
        
        # sensor_data 是numpy数组，每行: [time, coord0, coord1, coord2, ...]
        data_array = np.array(sensor_data)
        
        print(f"\nSensor data shape: {data_array.shape}")
        print(f"Saving to: {output_file}")
        
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write("# Contact sensor data\n")
            f.write(f"# Inner tooth {args.tooth}, flank {args.flank} vs. outer tooth {args.outer_tooth}, flank {visual_outer_flank}\n")
            f.write("# Time(s)\tGap(mm)\tForce(N)\n")
            
            for i in range(data_array.shape[0]):
                time_val = data_array[i, 0]
                gap_val = data_array[i, 1] * 1000  # 转换为毫米 (coordinate 0)
                force_val = data_array[i, 3]  # coordinate 2
                f.write(f"{time_val:.6f}\t{gap_val:.6f}\t{force_val:.6f}\n")
        
        print(f"Sensor data saved successfully!")
        print(f"Total data points: {data_array.shape[0]}")

    def _is_valid_solution_file(path):
        return os.path.isfile(path) and os.path.getsize(path) > 0

    if args.replay and not args.skip_solver:
        if _is_valid_solution_file(solution_file_path):
            try:
                # 重置系统状态，确保回放从干净状态开始
                mbs.Assemble()
                mbs.SolutionViewer(runMode=0, runOnStart=True, rowIncrement=1, timeout=0.02)
            except Exception as err:
                print(f"SolutionViewer failed: {err}")
        else:
            print(f"Solution file '{solution_file_path}' not found; skipping replay.")


def parse_arguments():
    parser = argparse.ArgumentParser(description="Single-tooth contact debugger for ObjectContactFewTeeth.")
    parser.add_argument("--tooth", type=int, default=0, help="Inner gear tooth index to probe.")
    parser.add_argument("--flank", choices=["left", "right"], default="right", help="Flank side to test.")
    parser.add_argument("--outer-tooth", type=int, default=0, help="Outer gear tooth index to visualize.")
    parser.add_argument("--outer-angle", type=float, default=0, help="Outer body rotation about +Z in degrees.")
    parser.add_argument("--outer-omega", type=float, default=10.0, help="Initial angular velocity of outer gear about +Z (rad/s).")
    parser.add_argument("--outer-rotation", type=float, default=0.0, help="Additional rotation offset for analytic profile (rad).")
    parser.add_argument("--inner-angle", type=float, default=0., help="Initial inner gear angle about +Z (deg).")
    parser.add_argument("--inner-omega", type=float, default=10.0, help="Target angular velocity for inner gear (rad/s). Set to 0 to disable.")
    parser.add_argument("--inner-x-shift", type=float, default=0, help="X translation of inner center (m).")
    parser.add_argument("--inner-y-shift", type=float, default=0, help="Y translation of inner center (m).")
    parser.add_argument("--radial-shift", type=float, default=0e-3, help="Radial shift applied to outer center (m).")
    parser.add_argument("--x-shift", type=float, default=0.0, help="X translation of outer center (m).")
    parser.add_argument("--z-shift", type=float, default=0.0, help="Z translation of outer center (m).")
    parser.add_argument("--inner-z", type=float, default=0.0, help="Z translation of inner gear center (m).")
    parser.add_argument("--reference-distance", type=float, default=0.0, help="Contact reference distance (m).")
    parser.add_argument("--distance-threshold", type=float, default=0.005, help="Tip probe length (m).")
    parser.add_argument("--contact-stiffness", type=float, default=1e9, help="Penalty stiffness (N/m).")
    parser.add_argument("--contact-damping", type=float, default=1e2, help="Penalty damping (N*s/m). Default 1e4 provides ~10%% critical damping.")
    parser.add_argument("--drive-torque", type=float, default=5000 , help="Applied torque on outer gear about +Z (N·m).")
    parser.add_argument("--ramp-time", type=float, default=0.05, help="Ramp time for velocity control (s).")
    parser.add_argument("--sim-time", type=float, default=0.2, help="Dynamic simulation duration (s).")
    parser.add_argument("--time-step", type=float, default=1e-6, help="Integrator step size (s).")
    parser.add_argument("--skip-solver", action="store_true", help="Skip SolveStatic/Dynamic (useful for pure kinematic evaluation).")
    parser.add_argument("--render", dest="render", action="store_true", help="Show OpenGL window (default).")
    parser.add_argument("--no-render", dest="render", action="store_false", help="Disable renderer window.")
    parser.add_argument("--verbose", action="store_true", help="Enable verbose static solver output.")
    parser.add_argument("--outer-mass", type=float, default=10.0, help="Outer body mass (kg).")
    parser.add_argument("--inner-mass", type=float, default=50.0, help="Inner body mass (kg).")
    parser.add_argument("--outer-inertia", type=float, default=1e-2, help="Outer body diagonal inertia component (kg·m²).")
    parser.add_argument("--inner-inertia", type=float, default=5e-3, help="Inner body diagonal inertia component (kg·m²).")
    parser.add_argument("--line-width", type=float, default=3.0, help="Polyline width used for gear graphics.")
    parser.add_argument("--point-size", type=float, default=1e-4, help="Contact point marker size for visualization.")
    parser.add_argument("--force-scale", type=float, default=1e-5, help="Visualization scaling factor for contact forces.")
    parser.add_argument("--outer-color", nargs=4, type=float, default=[0.1, 0.3, 0.9, 1.0], help="Outer tooth RGBA color.")
    parser.add_argument("--inner-color", nargs=4, type=float, default=[0.9, 0.2, 0.2, 1.0], help="Inner tooth RGBA color.")
    parser.add_argument("--outer-y-offset", type=float, default=0e-3, help="Additional +Y offset applied to outer gear (m).")
    parser.add_argument("--solution-file", type=str, default="solution/fewTeethTest.txt", help="Path to store solution for playback.")
    parser.add_argument("--replay", dest="replay", action="store_true", help="Open solution viewer after simulation.")
    parser.add_argument("--no-replay", dest="replay", action="store_false", help="Disable solution viewer playback.")

    parser.add_argument("--z-outer", dest="z_outer", type=int, default=98, help="Outer gear tooth count.")
    parser.add_argument("--z-inner", dest="z_inner", type=int, default=100, help="Inner gear tooth count.")
    parser.add_argument("--module", type=float, default=2.5e-3, help="Module (m).")
    parser.add_argument("--x-outer", dest="x_outer", type=float, default=0.0, help="Outer gear addendum modification.")
    parser.add_argument("--x-inner", dest="x_inner", type=float, default=0.25, help="Inner gear addendum modification.")
    parser.add_argument("--alpha", type=float, default=20.0, help="Pressure angle in degrees.")
    parser.add_argument("--ha-star", dest="ha_star", type=float, default=0.64, help="Normalized addendum height.")
    parser.add_argument("--c-star", dest="c_star", type=float, default=0.5, help="Normalized clearance coefficient.")

    parser.set_defaults(render=True, replay=True)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()
    create_contact_test(args)


# 阻尼力过大造成抵消弹性力, 导致负力被截断为0，造成看起来没有接触
