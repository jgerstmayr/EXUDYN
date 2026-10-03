#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Performance test of the access functions of a body (#2744): models whose markers ask the bodies for their
#           Jacobians in every step - rigid bodies (Euler parameters and Tait-Bryan angles) joined by
#           ObjectConnectorRigidBodySpringDamper on body markers at offset points (position and rotation
#           Jacobians, and the derivative of the transposed Jacobian times the force in the implicit run),
#           2D rigid bodies joined by spring-dampers, and an ANCF cable on an elastic foundation of spring-dampers
#           at points off its axis, whose derivative of the transposed Jacobian times the force is computed by
#           automatic differentiation of its position. The access functions by automatic differentiation of the
#           rigid bodies were measured against the hand-written ones here (revision2026b step RG9.3.5): the same
#           results, 20-35 % slower implicit solves.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-02
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import numpy as np

import testRunnerTools

testIsActive = exu.sys.get('testIsActive', True)

inertia = InertiaCuboid(density=1000, sideLengths=[0.1, 0.05, 0.05])


def BuildRigidChain(mbs, nBodies, rotations):
    """a chain of rigid bodies joined by rigid-body spring-dampers on body markers at offset points"""
    oGround = mbs.AddObject(ObjectGround())
    lastMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
    for i in range(nBodies):
        omega = [0.1, 0.2, 0.3]
        if rotations == 'EP':
            n = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0.2*(i+1), 0, 0] + list(eulerParameters0),
                                            initialVelocities=[0, 0.1*np.sin(i), 0.05] + list(
                                                AngularVelocity2EulerParameters_t(omega, eulerParameters0))))
        else:
            n = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0.2*(i+1), 0, 0, 0.1, 0.2, 0.1],
                                              initialVelocities=[0, 0.1*np.sin(i), 0.05] + omega))
        b = mbs.AddObject(ObjectRigidBody(nodeNumber=n, mass=inertia.Mass(), inertia=inertia.GetInertia6D()))
        m0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b, localPosition=[-0.05, 0.01, 0]))
        m1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b, localPosition=[0.05, 0.01, 0]))
        mbs.AddObject(RigidBodySpringDamper(markerNumbers=[lastMarker, m0], stiffness=np.diag([1000]*3+[10]*3),
                                            damping=np.diag([1]*3+[0.01]*3), offset=[0.1, 0, 0, 0, 0, 0]))
        lastMarker = m1


def BuildRigid2DChain(mbs, nBodies):
    """a chain of 2D rigid bodies joined by spring-dampers on body markers at offset points"""
    oGround = mbs.AddObject(ObjectGround())
    lastMarker = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround))
    for i in range(nBodies):
        n = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0.2*(i+1), 0, 0.1*i], initialVelocities=[0, 0.1*np.sin(i), 0.5]))
        b = mbs.AddObject(ObjectRigidBody2D(nodeNumber=n, mass=1, inertia=0.01))
        m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b, localPosition=[-0.05, 0.01, 0]))
        m1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b, localPosition=[0.05, 0.01, 0]))
        mbs.AddObject(SpringDamper(markerNumbers=[lastMarker, m0], referenceLength=0.1, stiffness=1000, damping=1))
        lastMarker = m1


def BuildCable(mbs, nElements):
    """an ANCF cable on an elastic foundation: per element a spring-damper from a point off the axis to the ground"""
    oGround = mbs.AddObject(ObjectGround())
    L = 1/nElements
    nodes = [mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[i*L, 0, 1, 0], initialVelocities=[0, 0.1*np.sin(i), 0, 0]))
             for i in range(nElements+1)]
    for i in range(nElements):
        e = mbs.AddObject(ObjectANCFCable2D(nodeNumbers=[nodes[i], nodes[i+1]], length=L, massPerLength=1,
                                            bendingStiffness=1, axialStiffness=1000))
        mCable = mbs.AddMarker(MarkerBodyPosition(bodyNumber=e, localPosition=[0.5*L, 0.01, 0]))
        mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[(i+0.5)*L, -0.09, 0]))
        mbs.AddObject(SpringDamper(markerNumbers=[mGround, mCable], referenceLength=0.1, stiffness=100, damping=0.1))


#The runs: the model, the size, explicit (RK44, h=1e-4) or implicit (generalized-alpha, h=1e-3), and the steps
runList = [
    {'model': 'rigidEP',   'size': 100, 'explicit': False, 'numberOfSteps': 100},
    {'model': 'rigidRxyz', 'size': 100, 'explicit': False, 'numberOfSteps': 100},
    {'model': 'rigidRxyz', 'size': 100, 'explicit': True,  'numberOfSteps': 600},
    {'model': 'rigid2D',   'size': 200, 'explicit': False, 'numberOfSteps': 200},
    {'model': 'cable',     'size': 200, 'explicit': False, 'numberOfSteps': 200},
    {'model': 'rigid2D',   'size': 200, 'explicit': True,  'numberOfSteps': 1000},
    ]

result = 0
for run in runList:
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    if run['model'] == 'rigidEP':
        BuildRigidChain(mbs, run['size'], 'EP')
    elif run['model'] == 'rigidRxyz':
        BuildRigidChain(mbs, run['size'], 'Rxyz')
    elif run['model'] == 'rigid2D':
        BuildRigid2DChain(mbs, run['size'])
    else:
        BuildCable(mbs, run['size'])
    mbs.Assemble()

    h = 1e-4 if run['explicit'] else 1e-3
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = run['numberOfSteps']
    simulationSettings.timeIntegration.endTime = run['numberOfSteps']*h
    simulationSettings.solution.file.write = False
    simulationSettings.timeIntegration.verboseMode = 1
    simulationSettings.linearSolver.solverType = exu.LinearSolverType.EigenSparse
    mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.RK44 if run['explicit']
                     else exu.DynamicSolverType.GeneralizedAlpha)

    result = float(np.abs(mbs.systemData.GetODE2Coordinates()).sum())
    runName = ('perfAccessFunctionsAD:' + run['model'] + '-n' + str(run['size'])
               + ('-explicit' if run['explicit'] else '-implicit'))
    exu.Print('result ' + runName + '=', result)
    testRunnerTools.AddTiming(runName, mbs, result)

exu.sys['testResult'] = result
