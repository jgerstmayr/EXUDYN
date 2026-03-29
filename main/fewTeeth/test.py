#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
External Involute Gear Contact Test for ObjectContactExternalInvolute.

This script creates a pair of external spur gears and tests the contact force
calculation using the new ObjectContactExternalInvolute connector.
"""
import sys, os

sys.exudynFast = True
sys.exudynCPUhasAVX2 = True

import numpy as np
import math
import argparse
import exudyn as exu
import exudyn.graphics as graphics
from exudyn.utilities import RotationMatrixZ, RigidBodyInertia
from exudyn.itemInterface import (
    NodeGenericData,
    NodePointGround,
    MarkerBodyRigid,
    MarkerNodeCoordinate,
    GenericJoint,
    CoordinateConstraint,
    LoadTorqueVector,
    SensorNode,
    ObjectContactExternalInvolute,
    VObjectContactExternalInvolute,
    VObjectJointGeneric,
)
import involute_profile
import tooth_stiffness

# =============================================================================
# Argument Parsing
# =============================================================================
def parse_arguments():
    parser = argparse.ArgumentParser(description="External Involute Gear Contact Test")
    
    # Gear 1 parameters (Driver - smaller gear)
    parser.add_argument("--z1", type=int, default=21, help="Teeth count Gear 1")
    parser.add_argument("--x1", type=float, default=0.0, help="Profile shift Gear 1")
    
    # Gear 2 parameters (Driven - larger gear)
    parser.add_argument("--z2", type=int, default=57, help="Teeth count Gear 2")
    parser.add_argument("--x2", type=float, default=0.0, help="Profile shift Gear 2")
    
    # Common gear parameters
    parser.add_argument("--module", type=float, default=2e-3, help="Module (m)")
    parser.add_argument("--alpha", type=float, default=20.0, help="Pressure angle (deg)")
    parser.add_argument("--width", type=float, default=0.015, help="Face width (m)")
    parser.add_argument("--ha_star", type=float, default=1.0, help="Addendum coefficient")
    parser.add_argument("--c_star", type=float, default=0.25, help="Clearance coefficient")
    
    # Simulation parameters
    parser.add_argument("--sim_time", type=float, default=20, help="Simulation time (s)")
    parser.add_argument("--omega1", type=float, default=0.0, help="Input angular velocity (rad/s)")
    parser.add_argument("--load_torque", type=float, default=100.0, help="Load torque on Gear 2 (Nm)")
    parser.add_argument("--ramp_time", type=float, default=0.02, help="Ramp time for velocity (s)")
    parser.add_argument("--contact_damping", type=float, default=1e5, help="Contact damping (Ns/m)")
    
    # Visualization
    parser.add_argument("--render", dest="render", action="store_true", help="Enable renderer")
    parser.add_argument("--no-render", dest="render", action="store_false", help="Disable renderer")
    parser.add_argument("--replay", dest="replay", action="store_true", help="Enable solution viewer")
    parser.add_argument("--no-replay", dest="replay", action="store_false", help="Disable solution viewer")
    parser.add_argument("--force_scale", type=float, default=1e-4, help="Contact force visualization scale")
    
    parser.set_defaults(render=True, replay=True)
    return parser.parse_args()

# =============================================================================
# Helper Functions
# =============================================================================
def build_gear_analytic(z, module, alpha_rad, ha_star, c_star, x, base_rotation=0.0, flank_sign=1.0,
                        replicate_all_teeth=True, single_tooth_index=0):
    """Build gear analytic profile dictionary for ObjectContactExternalInvolute."""
    return {
        "numberOfTeeth": int(z),
        "module": float(module),
        "alpha": float(alpha_rad),
        "haStar": float(ha_star),
        "cStar": float(c_star),
        "x": float(x),
        "baseRotation": float(base_rotation),
        "flankSign": float(flank_sign),
        "reverseParam": flank_sign < 0,
        "replicateAllTeeth": replicate_all_teeth,
        "singleToothIndex": int(single_tooth_index),
    }

def build_gear_graphics(gear_params, color=(0.6, 0.6, 0.8, 0.8)):
    """Generate graphics for external gear using involute_profile."""
    flanks, geometry = involute_profile.generate_external_gear_flanks(gear_params, replicate_all_teeth=True)
    graphics_list = []
    for (tooth_idx, flank_side), pts in flanks.items():
        polyline = involute_profile.as_3d_polyline(pts, z_level=0.0)
        graphics_list.append(graphics.Lines(polyline, color=color))
    return graphics_list

# =============================================================================
# Main Simulation
# =============================================================================
def main():
    args = parse_arguments()
    
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    
    # -------------------------------------------------------------------------
    # 1. Gear Parameters Definition
    # -------------------------------------------------------------------------
    alpha_rad = math.radians(args.alpha)
    
    tip_probe_length = 0.002  # 探针长度 5mm
    
    # 探针只加在齿轮1上（C++中探针从gear1发射检测gear2）
    gear1_params = involute_profile.ExternalGearParams(
        z=args.z1,
        module=args.module,
        alpha_deg=args.alpha,
        x=args.x1,
        ha_star=args.ha_star,
        c_star=args.c_star,
        flank_points=60,
        tip_probe_length=tip_probe_length,
    )
    
    # 齿轮2不需要探针
    gear2_params = involute_profile.ExternalGearParams(
        z=args.z2,
        module=args.module,
        alpha_deg=args.alpha,
        x=args.x2,
        ha_star=args.ha_star,
        c_star=args.c_star,
        flank_points=60,
        tip_probe_length=0.0,  # 无探针
    )
    
    # Calculate center distance
    def solve_alpha_w(inv_alpha_target):
        x = 0.3
        for _ in range(15):
            fx = math.tan(x) - x - inv_alpha_target
            dfx = math.tan(x)**2
            if abs(dfx) < 1e-12:
                break
            x = x - fx / dfx
        return x

    inv_alpha = math.tan(alpha_rad) - alpha_rad
    sum_x = args.x1 + args.x2
    
    if abs(sum_x) > 1e-6:
        inv_alpha_w = inv_alpha + 2 * sum_x * math.tan(alpha_rad) / (args.z1 + args.z2)
        alpha_w = solve_alpha_w(inv_alpha_w)
        center_distance = (args.z1 + args.z2) * args.module / 2.0 * math.cos(alpha_rad) / math.cos(alpha_w)
    else:
        center_distance = (args.z1 + args.z2) * args.module / 2.0

    print(f"\n=== External Gear Contact Test ===")
    print(f"Gear 1: z={args.z1}, x={args.x1}")
    print(f"Gear 2: z={args.z2}, x={args.x2}")
    print(f"Module: {args.module*1000:.3f} mm")
    print(f"Center Distance: {center_distance*1000:.4f} mm")

    # -------------------------------------------------------------------------
    # 2. Stiffness Calculation (using tooth_stiffness.py)
    # -------------------------------------------------------------------------
    print("\nCalculating tooth stiffness tables...")
    
    stiffness_table_1 = tooth_stiffness.create_stiffness_table_for_gear(
        z=args.z1,
        module=args.module,
        face_width=args.width,
        alpha_deg=args.alpha,
        ha_star=args.ha_star,
        c_star=args.c_star,
        x=args.x1,
        is_internal=False,
        n_points=15,
        include_contact=False
    )
    
    stiffness_table_2 = tooth_stiffness.create_stiffness_table_for_gear(
        z=args.z2,
        module=args.module,
        face_width=args.width,
        alpha_deg=args.alpha,
        ha_star=args.ha_star,
        c_star=args.c_star,
        x=args.x2,
        is_internal=False,
        n_points=15,
        include_contact=False
    )
    
    print(f"  Gear 1 stiffness: {[f'{k/1e6:.1f}' for k in stiffness_table_1]} MN/m")
    print(f"  Gear 2 stiffness: {[f'{k/1e6:.1f}' for k in stiffness_table_2]} MN/m")

    # -------------------------------------------------------------------------
    # 3. Create Graphics
    # -------------------------------------------------------------------------
    gear1_graphics = build_gear_graphics(gear1_params, color=(0.2, 0.5, 0.9, 0.8))
    gear2_graphics = build_gear_graphics(gear2_params, color=(0.9, 0.3, 0.2, 0.8))

    # -------------------------------------------------------------------------
    # 4. Bodies and Ground
    # -------------------------------------------------------------------------
    oGround = mbs.CreateGround()
    
    rho_steel = 7850
    r1 = args.module * args.z1 / 2.0
    mass1 = math.pi * r1**2 * args.width * rho_steel
    J1 = 0.5 * mass1 * r1**2
    
    r2 = args.module * args.z2 / 2.0
    mass2 = math.pi * r2**2 * args.width * rho_steel
    J2 = 0.5 * mass2 * r2**2
    
    print(f"\nGear 1: r={r1*1000:.2f}mm, mass={mass1:.3f}kg, J={J1:.6e}kg·m²")
    print(f"Gear 2: r={r2*1000:.2f}mm, mass={mass2:.3f}kg, J={J2:.6e}kg·m²")

    # Gear positions: Gear1 at origin, Gear2 along +Y
    gear1_center = np.array([0.0, 0.0, 0.0])
    gear2_center = np.array([0.0, center_distance, 0.0])
    
    inertia1 = RigidBodyInertia(mass=mass1, inertiaTensor=np.diag([J1, J1, J1]))
    inertia2 = RigidBodyInertia(mass=mass2, inertiaTensor=np.diag([J2, J2, J2]))
    
    body1 = mbs.CreateRigidBody(
        referencePosition=gear1_center.tolist(),
        referenceRotationMatrix=RotationMatrixZ(0),
        inertia=inertia1,
        graphicsDataList=gear1_graphics,
        gravity=[0, 0, 0],
        nodeType=exu.NodeType.RotationRxyz,
    )
    
    body2 = mbs.CreateRigidBody(
        referencePosition=gear2_center.tolist(),
        referenceRotationMatrix=RotationMatrixZ(0),
        inertia=inertia2,
        graphicsDataList=gear2_graphics,
        gravity=[0, 0, 0],
        nodeType=exu.NodeType.RotationRxyz,
    )
    
    # Ground and body markers
    mGround1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=gear1_center.tolist()))
    mGround2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=gear2_center.tolist()))
    mBody1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body1, localPosition=[0, 0, 0]))
    mBody2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body2, localPosition=[0, 0, 0]))
    
    # Fix positions, allow Z rotation only
    mbs.AddObject(GenericJoint(
        markerNumbers=[mGround1, mBody1],
        constrainedAxes=[1, 1, 1, 1, 1, 0],
        visualization=VObjectJointGeneric(show=False),
    ))
    mbs.AddObject(GenericJoint(
        markerNumbers=[mGround2, mBody2],
        constrainedAxes=[1, 1, 1, 1, 1, 0],
        visualization=VObjectJointGeneric(show=False),
    ))

    # -------------------------------------------------------------------------
    # 5. Drive and Load
    # -------------------------------------------------------------------------
    node1 = mbs.GetObject(body1)['nodeNumber']
    mGear1RotZ = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node1, coordinate=5))
    nGroundCoord = mbs.AddNode(NodePointGround())
    mGroundCoord = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGroundCoord, coordinate=0))
    
    omega_target = args.omega1
    ramp_time = args.ramp_time
    
    def gear1_angle_offset(mbs, t, itemIndex, currentOffset):
        if t <= 0.0:
            return 0.0
        elif t < ramp_time:
            return 0.5 * omega_target / ramp_time * t * t
        else:
            return 0.5 * omega_target * ramp_time + omega_target * (t - ramp_time)
    
    mbs.AddObject(CoordinateConstraint(
        markerNumbers=[mGroundCoord, mGear1RotZ],
        offset=0.0,
        offsetUserFunction=gear1_angle_offset
    ))
    
    if abs(args.load_torque) > 0:
        mbs.AddLoad(LoadTorqueVector(markerNumber=mBody2, loadVector=[0, 0, -args.load_torque]))
    
    print(f"\nDrive: ω1={omega_target} rad/s, ramp={ramp_time}s")
    print(f"Load torque: {args.load_torque} Nm")

    # -------------------------------------------------------------------------
    # 6. Contact Elements (与test1一致：为gear1每个齿的左右齿面创建接触对象)
    # -------------------------------------------------------------------------
    print("\nCreating contact objects...")
    
    contact_nodes = []
    contact_objects = []
    
    # 为gear1的每个齿创建接触对象（左右齿面各一个）
    # gear1: replicateAllTeeth=False, singleToothIndex=tooth_idx (只检测单个齿)
    # gear2: replicateAllTeeth=True (遍历所有齿)
    for tooth1_idx in range(args.z1):
        for flank_side in ["right", "left"]:
            flank1_sign = 1.0 if flank_side == "right" else -1.0
            # 配对关系调整：gear1 与 gear2 使用同侧齿面（右↔右，左↔左）
            flank2_sign = flank1_sign
            
            nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=11, initialCoordinates=[0.0]*11))
            contact_nodes.append(nData)
            
            # gear1: 只检测第tooth1_idx个齿
            gear1_analytic = build_gear_analytic(
                z=args.z1, module=args.module, alpha_rad=alpha_rad,
                ha_star=args.ha_star, c_star=args.c_star, x=args.x1,
                base_rotation=0.0, flank_sign=flank1_sign,
                replicate_all_teeth=False, single_tooth_index=tooth1_idx
            )
            # gear2: 遍历所有齿
            gear2_analytic = build_gear_analytic(
                z=args.z2, module=args.module, alpha_rad=alpha_rad,
                ha_star=args.ha_star, c_star=args.c_star, x=args.x2,
                base_rotation=0.0, flank_sign=flank2_sign,
                replicate_all_teeth=True
            )
            
            # 蓝色=左齿面, 红色=右齿面
            contact_color = [0.1, 0.3, 0.9, 0.5] if flank_side == "left" else [0.9, 0.2, 0.2, 0.5]
            
            oContact = mbs.AddObject(ObjectContactExternalInvolute(
                name=f"Contact_G1T{tooth1_idx:02d}_{flank_side[0].upper()}",
                markerNumbers=[mBody1, mBody2],
                nodeNumber=nData,
                gear1Analytic=gear1_analytic,
                gear2Analytic=gear2_analytic,
                stiffnessGear1=stiffness_table_1,
                stiffnessGear2=stiffness_table_2,
                contactDamping=args.contact_damping,
                tipProbeLength=tip_probe_length,
                faceWidth=args.width,
                elasticModulus=2.1e11,
                poissonRatio=0.3,
                frictionCoefficient=0.1,
                frictionVelocityPenalty=1e5,
                frictionProportionalZone=0.001,
                visualization=VObjectContactExternalInvolute(show=True, color=contact_color),
            ))
            contact_objects.append(oContact)
    
    print(f"  Created {len(contact_objects)} contact objects (z1={args.z1} teeth × 2 flanks)")

    # -------------------------------------------------------------------------
    # 7. Sensors
    # -------------------------------------------------------------------------
    primary_contact_node = contact_nodes[0]
    gap_sensor = mbs.AddSensor(SensorNode(
        nodeNumber=primary_contact_node,
        outputVariableType=exu.OutputVariableType.Coordinates,
        storeInternal=True,
        fileName="external_gear_contact.txt",
        writeToFile=True,
    ))

    # -------------------------------------------------------------------------
    # 8. Visualization Settings
    # -------------------------------------------------------------------------
    SC.visualizationSettings.contact.contactPointsDefaultSize = 5e-4
    SC.visualizationSettings.contact.showContactForces = True
    SC.visualizationSettings.contact.contactForcesFactor = args.force_scale
    SC.visualizationSettings.window.renderWindowSize = [1280, 720]

    # -------------------------------------------------------------------------
    # 9. Simulation
    # -------------------------------------------------------------------------
    mbs.Assemble()
    
    print(f"\nSimulation: endTime={args.sim_time}s")
    
    started_renderer = False
    if args.render:
        exu.StartRenderer()
        started_renderer = True
        mbs.WaitForUserToContinue()  # 按空格开始计算
    
    simSettings = exu.SimulationSettings()
    simSettings.timeIntegration.endTime = args.sim_time
    simSettings.timeIntegration.numberOfSteps = int(args.sim_time / 1e-5)
    simSettings.timeIntegration.verboseMode = 1
    simSettings.timeIntegration.newton.useModifiedNewton = True
    simSettings.timeIntegration.newton.relativeTolerance = 1e-8
    simSettings.solutionSettings.sensorsWritePeriod = 1e-5
    simSettings.solutionSettings.writeSolutionToFile = args.replay
    simSettings.solutionSettings.coordinatesSolutionFileName = "solution/externalGearTest.txt"
    
    mbs.SolveDynamic(simSettings)
    
    if args.render:
        mbs.WaitForUserToContinue()  # 计算完成后等待用户
        if started_renderer:
            exu.StopRenderer()
    
    # -------------------------------------------------------------------------
    # 10. Results
    # -------------------------------------------------------------------------
    data_values = mbs.GetSensorValues(gap_sensor)
    gap = data_values[0]
    contact_force = data_values[2]
    
    print(f"\n=== Results ===")
    print(f"Primary contact gap: {gap:.6e} m")
    print(f"Primary contact force: {contact_force:.2f} N")
    
    if args.replay:
        try:
            mbs.SolutionViewer(runMode=0, runOnStart=True)
        except Exception as e:
            print(f"SolutionViewer error: {e}")


if __name__ == '__main__':
    main()
