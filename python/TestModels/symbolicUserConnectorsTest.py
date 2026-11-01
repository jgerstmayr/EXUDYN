#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  CreateCartesianSpringDamper, CreateRigidBodySpringDamper (both formulations), CreateCoordinateConstraint
#           and CreateTorque of UserItemCreator (#3193) against the built-in items of the Create functions of mbs: two
#           rigid bodies on NodeRigidBodyRxyz, the first held to the ground by a rigid-body spring-damper with a full
#           stiffness and damping matrix in a turned joint frame, the second on the first by a Cartesian spring-damper and
#           a rigid-body spring-damper, its z-coordinate constrained to that of a mass point and a torque on it; the
#           Python user items, the recorded ones, and SymbolicSystem by SolveGeneralizedAlpha. The result is the final
#           position of the second body plus the number of checks that pass.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.symbolics.userItems import UserItemCreator
from exudyn.symbolics.systemModel import SymbolicSystem
from exudyn.symbolics.solvers import SolveGeneralizedAlpha
import exudyn.cMath as cMath
import numpy as np

def UFtorque(mbs, t, loadVector):
    return cMath.sin(8*t)*loadVector

K = np.diag([2e3, 3e3, 4e3, 50., 60., 70.])
K[0, 4] = K[4, 0] = 20.
D = 0.02*K
offset = [0.01, 0, -0.02, 0.02, 0, 0.01]

def Build(userItems, record=False, intrinsic=True):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    creator = UserItemCreator(mbs) if userItems else mbs
    ground = creator.CreateGround()
    inertia = InertiaCuboid(density=1000, sideLengths=[0.3, 0.1, 0.08])
    bodies = [creator.CreateRigidBody(inertia=inertia, referencePosition=[0.3*(i + 1), 0, 0], referenceRotationMatrix=RotationMatrixZ(0.1*i),
                                      initialAngularVelocity=[0.3, -0.2*i, 0.5], initialVelocity=[0.05*i, 0.1, 0], gravity=[0, -9.81, 0],
                                      nodeType=exu.NodeType.RotationRxyz) for i in range(2)]
    point = creator.CreateMassPoint(referencePosition=[0.6, -0.2, 0], mass=0.5, gravity=[0, -9.81, 0])
    creator.CreateRigidBodySpringDamper(itemNumbers=[ground, bodies[0]], localPosition0=[0.15, 0, 0], localPosition1=[-0.15, 0, 0],
                                        stiffness=K, damping=D, offset=offset, rotationMatrixJoint=RotationMatrixX(0.3),
                                        useIntrinsicFormulation=intrinsic)
    creator.CreateRigidBodySpringDamper(itemNumbers=[bodies[0], bodies[1]], localPosition0=[0.15, 0, 0], localPosition1=[-0.15, 0, 0],
                                        stiffness=K, damping=D, rotationMatrixJoint=RotationMatrixY(0.2), useGlobalFrame=False,
                                        useIntrinsicFormulation=intrinsic)
    creator.CreateCartesianSpringDamper(itemNumbers=[bodies[0], bodies[1]], localPosition0=[0, 0.05, 0], localPosition1=[0, 0.05, 0],
                                        stiffness=[1e3, 2e3, 500.], damping=[5., 10., 2.], offset=[0.3, 0, 0])
    creator.CreateCartesianSpringDamper(itemNumbers=[point, bodies[1]], stiffness=[200., 200., 0.], damping=[1., 1., 0.],
                                        offset=[0.3, 0.2, 0])
    creator.CreateCoordinateConstraint(itemNumbers=[point, bodies[1]], coordinates=[2, 2], factor1=1., offset=0.)
    creator.CreateTorque(itemNumber=bodies[1], loadVector=[0.2, 0, 0.5], bodyFixed=True, loadVectorUserFunction=UFtorque)
    if record:
        creator.Record()
    mbs.variables['creator'] = creator #the recordings live as long as the system
    mbs.Assemble()
    return (mbs, bodies)

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 100
simulationSettings.timeIntegration.endTime = 0.2
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-12
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-12
simulationSettings.timeIntegration.newton.useModifiedNewton = False
simulationSettings.solution.file.write = False

def State(mbs, bodies):
    return np.array([x for body in bodies for x in list(mbs.GetObjectOutputBody(body, exu.OutputVariableType.Position, [0.1, 0.02, 0.01]))
                     + list(mbs.GetObjectOutputBody(body, exu.OutputVariableType.AngularVelocity, [0, 0, 0]))])

passed = 0
for intrinsic in [True, False]:
    results = []
    for (userItems, record) in [(False, False), (True, False), (True, True)]:
        (mbs, bodies) = Build(userItems, record, intrinsic)
        mbs.SolveDynamic(simulationSettings)
        results.append(State(mbs, bodies))
    differences = [np.max(np.abs(result - results[0])) for result in results[1:]]
    exu.Print('intrinsic' if intrinsic else 'Tait-Bryan', ': Python, recorded user items against built-in:', differences)
    passed += int(max(differences) < 1e-9)

    (mbs, bodies) = Build(True, True, intrinsic)
    system = SymbolicSystem(mbs)
    SolveGeneralizedAlpha(system, simulationSettings)
    difference = np.max(np.abs(State(mbs, bodies) - results[2]))
    exu.Print('  SymbolicSystem by SolveGeneralizedAlpha against SolveDynamic:', difference)
    passed += int(difference < 1e-9)

u = np.sum(results[0][0:3]) + passed
exu.Print('solution of symbolicUserConnectorsTest=', u)
exu.sys['testResult'] = u
