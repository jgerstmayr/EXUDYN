#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The rotation of a joint or connector frame is given to its markers, as localHT (#2745):
#           a body on a GenericJoint and a body on a RigidBodySpringDamper, each turned once by
#           rotationMarker0/1 and once by the localHT of the markers, move the same. rotationMarker0/1
#           are deprecated: Assemble() warns about them with a DeprecationWarning once per session,
#           and not again (#2745); the RigidBodySpringDamper applies rotationMarker0 on the same side
#           as all joints (#2801).
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-03
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import numpy as np
import warnings

testIsActive = exu.sys.get('testIsActive', False)

A0 = RotationMatrixX(0.3) @ RotationMatrixY(0.5) #the joint frame on the ground
A1 = RotationMatrixZ(-0.4) @ RotationMatrixX(0.2) #the joint frame on the body
Ab = A0 @ A1.T #the body, turned so that both joint frames agree in the reference configuration
pb = -Ab @ np.array([-0.2, 0, 0]) #and its marker at the origin

def Simulate(useLocalHT, connector):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.AddObject(ObjectGround())
    inertia = InertiaCuboid(density=1000, sideLengths=[0.4, 0.1, 0.1])
    node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=list(pb) + list(RotationMatrix2EulerParameters(Ab)),
                                       initialVelocities=[0]*7))
    body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(), physicsInertia=inertia.GetInertia6D()))
    if useLocalHT:
        m0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localHT=HomogeneousTransformation(A0, [0, 0, 0])))
        m1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localHT=HomogeneousTransformation(A1, [-0.2, 0, 0])))
        rotations = {}
    else:
        m0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, 0]))
        m1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localPosition=[-0.2, 0, 0]))
        rotations = {'rotationMarker0': A0, 'rotationMarker1': A1}
    if connector:
        k = 2000.
        mbs.AddObject(RigidBodySpringDamper(markerNumbers=[m0, m1], stiffness=np.diag([k, 2*k, 3*k, 0.1*k, 0.2*k, 0.3*k]),
                                            damping=np.diag([1, 1, 1, 0.1, 0.1, 0.1]), **rotations))
    else:
        mbs.AddObject(GenericJoint(markerNumbers=[m0, m1], constrainedAxes=[1, 1, 1, 1, 1, 0], **rotations))
    mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerBodyPosition(bodyNumber=body, localPosition=[0.2, 0, 0])),
                                loadVector=[0, -10, 5]))

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter('always')
        mbs.Assemble()
        mbs.Assemble() #once per session: never a second time
    nWarnings = sum(1 for entry in caught if issubclass(entry.category, DeprecationWarning)
                    and 'rotationMarker0' in str(entry.message))

    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 200
    simulationSettings.timeIntegration.endTime = 0.2
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.solutionSettings.writeSolutionToFile = False
    mbs.SolveDynamic(simulationSettings)
    return (np.array(mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)), nWarnings)

errors = 0
u = 0.
nWarnings = 0
for connector in [False, True]:
    (qLocalHT, n0) = Simulate(True, connector)
    (qRotationMarker, n1) = Simulate(False, connector)
    exu.Print('connector' if connector else 'joint', ': difference localHT - rotationMarker =', np.linalg.norm(qLocalHT - qRotationMarker))
    if np.linalg.norm(qLocalHT - qRotationMarker) > 1e-10:
        errors += 1
    if n0 != 0: #localHT never warns
        errors += 1
    nWarnings += n1
    u += np.sum(qLocalHT)
if nWarnings > 1: #once per session: 1, or 0 if an earlier model of this session was warned already
    errors += 1

exu.Print('rotationMarkerDeprecationTest: errors', errors)
u += errors
exu.Print('solution of rotationMarkerDeprecationTest=', u)

exu.sys['testResult'] = u
