#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  mbs.CreateLinearSpringDamper (#1953, #1954): a rigid body on a prismatic joint along an inclined
#           axis, held by a linear spring-damper along the same axis against gravity. The static deflection
#           along the axis is m g_axis / k; the spring-damper given with the axis in global coordinates, in the
#           frame of a turned body 0 (useGlobalFrame=False), and between two markers gives it. A dynamic run
#           checks the damping: the oscillation decays towards the static deflection.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

axis = np.array([1.,1.,0.])/np.sqrt(2.)  #the axis of the joint and the spring
gravity = np.array([0.,-9.81,0.])
stiffness = 2e3
inertia = InertiaCuboid(density=1000, sideLengths=[0.2,0.1,0.1])
deflectionExpected = inertia.mass * (gravity @ axis) / stiffness

def Model(variant, dynamic=False):
    """the displacement of the body along the axis, static or after 2 s"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    rotation0 = RotationMatrixZ(0.3) #a turned ground body, for the axis in its frame
    oBase = mbs.CreateRigidBody(inertia=inertia, referencePosition=[0,0,0], referenceRotationMatrix=rotation0)
    mbs.CreateGenericJoint(itemNumbers=[mbs.CreateGround(), oBase], position=[0,0,0])  #fixed
    oBody = mbs.CreateRigidBody(inertia=inertia, referencePosition=[1,1,0], gravity=gravity)
    mbs.CreatePrismaticJoint(itemNumbers=[oBase, oBody], position=[1,1,0], axis=axis)
    if variant == 'global':
        mbs.CreateLinearSpringDamper(itemNumbers=[oBase, oBody], position=[1,1,0], axis=axis,
                                     stiffness=stiffness, damping=20)
    elif variant == 'local':
        mbs.CreateLinearSpringDamper(itemNumbers=[oBase, oBody], position=rotation0.T @ np.array([1,1,0]),
                                     axis=rotation0.T @ axis, useGlobalFrame=False, stiffness=stiffness, damping=20)
    elif variant == 'markers':
        m0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oBase, localPosition=rotation0.T @ np.array([1,1,0])))
        m1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oBody, localPosition=[0,0,0]))
        mbs.CreateLinearSpringDamper(itemNumbers=[m0, m1], position=[], axis=axis, stiffness=stiffness, damping=20)
    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.solution.file.write = False
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.staticSolver.verboseMode = 0
    if dynamic:
        simulationSettings.timeIntegration.numberOfSteps = 2000
        simulationSettings.timeIntegration.endTime = 2
        mbs.SolveDynamic(simulationSettings)
    else:
        mbs.SolveStatic(simulationSettings)
    return mbs.GetObjectOutputBody(oBody, exu.OutputVariableType.Displacement, localPosition=[0,0,0]) @ axis

u = 0
for variant in ['global', 'local', 'markers']:
    deflection = Model(variant)
    exu.Print(variant, ': deflection', deflection, ', expected', deflectionExpected)
    u += abs(deflection - deflectionExpected) < 1e-10
deflectionDynamic = Model('global', dynamic=True)
exu.Print('dynamic: deflection after 2 s', deflectionDynamic)
u += deflectionDynamic

exu.Print('solution of createLinearSpringDamperTest=', u)

exu.sys['testResult'] = u
