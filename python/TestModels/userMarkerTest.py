#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  MarkerBodyUser (#3065, #3103), a marker on several bodies whose position and rotation are Python user functions of the
#           coordinates of the nodes of these bodies. (1) A marker at the midpoint of two mass points, held by a
#           Cartesian spring to the ground and loaded by a force: with equal initial conditions, each mass point carries
#           half of it, as two mass points with half the spring and half the force each; the SensorMarker of the marker.
#           (2) A pendulum: a rigid marker on a rigid body, its position and rotation written with
#           exudyn.symbolics.kinematics and numpy, in a revolute joint, against MarkerBodyRigid; the Jacobians by central
#           differences and given by user functions; providedMarkerType Position | Orientation. (3) What the marker
#           refuses, and the type it reports.
#           Run alone, it draws the pendulum and opens the SolutionViewer.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-24
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.symbolics.kinematics as su
import exudyn.graphics as graphics
import numpy as np
testIsActive = exu.sys.get('testIsActive', False) #run alone: the renderer, the solution file and the SolutionViewer

SC = exu.SystemContainer()

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 500
simulationSettings.timeIntegration.endTime = 1
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.solution.file.write = False

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(1) the midpoint of two mass points
m = 1.
k = 100.
force = [0, 0, 2.]

def UFmidpoint(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return 0.5*(qTotal[0:3] + qTotal[3:6])

def UFmidpointJacobian(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return 0.5*np.hstack((np.eye(3), np.eye(3)))

def SolveMidpoint(kind):
    mbs = SC.AddSystem()
    ground = mbs.AddMarker(MarkerBodyPosition(bodyNumber=mbs.AddObject(ObjectGround())))
    nodes = [mbs.AddNode(NodePoint(referenceCoordinates=[x, 0, 0], initialCoordinates=[0, 0.1, 0])) for x in [-0.5, 0.5]]
    bodies = [mbs.AddObject(MassPoint(nodeNumber=node, mass=m)) for node in nodes]
    sensor = None
    if kind == 'builtIn':
        for body in bodies:
            marker = mbs.AddMarker(MarkerBodyPosition(bodyNumber=body))
            mbs.AddObject(CartesianSpringDamper(markerNumbers=[ground, marker], stiffness=[k/2]*3,
                                                offset=mbs.GetObjectOutputBody(body, exu.OutputVariableType.Position, configuration=exu.ConfigurationType.Reference)))
            mbs.AddLoad(LoadForceVector(markerNumber=marker, loadVector=0.5*np.array(force)))
    else:
        marker = mbs.AddMarker(MarkerBodyUser(bodyNumbers=bodies, positionUserFunction=UFmidpoint,
                                          positionJacobianUserFunction=UFmidpointJacobian if kind == 'jacobian' else 0))
        mbs.AddObject(CartesianSpringDamper(markerNumbers=[ground, marker], stiffness=[k]*3))
        mbs.AddLoad(LoadForceVector(markerNumber=marker, loadVector=force))
        sensor = mbs.AddSensor(SensorMarker(markerNumber=marker, outputVariableType=exu.OutputVariableType.Position, storeInternal=True))
    mbs.Assemble()
    mbs.SolveDynamic(simulationSettings)
    result = np.hstack((mbs.GetNodeOutput(nodes[0], exu.OutputVariableType.Displacement), mbs.GetNodeOutput(nodes[1], exu.OutputVariableType.Velocity)))
    sensorDifference = 0
    if sensor is not None: #the sensor at the end is the midpoint
        midpoint = 0.5*(np.array(mbs.GetNodeOutput(nodes[0], exu.OutputVariableType.Position)) + mbs.GetNodeOutput(nodes[1], exu.OutputVariableType.Position))
        sensorDifference = np.max(np.abs(mbs.GetSensorStoredData(sensor)[-1, 1:4] - midpoint))
    return result, sensorDifference

midpoints = [SolveMidpoint(kind) for kind in ['builtIn', 'numerical', 'jacobian']]
differenceMidpoint = max(np.max(np.abs(midpoints[1][0] - midpoints[0][0])), np.max(np.abs(midpoints[2][0] - midpoints[0][0])),
                         midpoints[1][1], midpoints[2][1])
u = np.sum(midpoints[0][0])
exu.Print('midpoint marker: largest difference to two mass points with half the spring and force:', differenceMidpoint)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(2) a pendulum on a revolute joint about z, at the end of a rigid body on a NodeRigidBodyRxyz
inertia = InertiaCuboid(density=1000, sideLengths=[1, 0.1, 0.1])
pLocal = np.array([-0.5, 0, 0])

def UFposition(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return qTotal[0:3] + su.RotXYZ2RotationMatrix(qTotal[3:6], np) @ pLocal

def UFrotation(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return su.RotXYZ2RotationMatrix(qTotal[3:6], np)

def UFpositionJacobian(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    A = su.RotXYZ2RotationMatrix(qTotal[3:6], np)
    return np.hstack((np.eye(3), -A @ su.Skew(pLocal, np) @ su.RotXYZ2GLocal(qTotal[3:6], np)))

def UFrotationJacobian(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return np.hstack((np.zeros((3, 3)), su.RotXYZ2RotationMatrix(qTotal[3:6], np) @ su.RotXYZ2GLocal(qTotal[3:6], np)))

def SolvePendulum(kind, show=False):
    mbs = SC.AddSystem()
    node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0.5, 0, 0, 0, 0, 0]))
    body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(), inertia=inertia.GetInertia6D(),
                                         visualization=VObjectRigidBody(graphicsData=[graphics.Brick(size=[1, 0.1, 0.1], color=graphics.color.steelblue)])))
    if kind == 'builtIn':
        marker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localPosition=pLocal))
    else:
        jacobians = {} if kind == 'numerical' else {'positionJacobianUserFunction': UFpositionJacobian, 'rotationJacobianUserFunction': UFrotationJacobian}
        marker = mbs.AddMarker(MarkerBodyUser(bodyNumbers=[body], providedMarkerType=exu.MarkerType.Position | exu.MarkerType.Orientation,
                                              positionUserFunction=UFposition, rotationMatrixUserFunction=UFrotation, **jacobians))
    mbs.AddObject(RevoluteJointZ(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=mbs.AddObject(ObjectGround()))), marker]))
    mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=body)), loadVector=[0, -9.81, 0]))
    mbs.Assemble()
    if show and not testIsActive:
        simulationSettings.solution.file.write = True
        SC.renderer.Start()
    mbs.SolveDynamic(simulationSettings)
    if show and not testIsActive:
        SC.renderer.Stop()
        mbs.SolutionViewer() #it reads the solution file of this run (#3098)
        simulationSettings.solution.file.write = False
    return np.hstack((mbs.GetNodeOutput(node, exu.OutputVariableType.Position), mbs.GetNodeOutput(node, exu.OutputVariableType.AngularVelocity)))

pendulums = [SolvePendulum(kind, show=(kind == 'jacobian')) for kind in ['builtIn', 'numerical', 'jacobian']]
differencePendulum = max(np.max(np.abs(pendulums[1] - pendulums[0])), np.max(np.abs(pendulums[2] - pendulums[0])))
u += np.sum(pendulums[0])
exu.Print('pendulum on MarkerBodyUser: largest difference to MarkerBodyRigid:', differencePendulum)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(3) what the marker refuses: a joint on a marker of type Position, a marker without bodies, a rotation without
#Orientation, Orientation without rotation, another type; the type the marker reports
rigidType = exu.MarkerType.Position | exu.MarkerType.Orientation
def Refused(bodies, message, providedMarkerType=exu.MarkerType.Position, rotation=0, joint=False):
    mbs = SC.AddSystem()
    node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0.5, 0, 0, 0, 0, 0]))
    body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(), inertia=inertia.GetInertia6D()))
    marker = mbs.AddMarker(MarkerBodyUser(bodyNumbers=[body]*bodies, providedMarkerType=providedMarkerType,
                                          positionUserFunction=UFposition, rotationMatrixUserFunction=rotation))
    if joint: #a joint needs Orientation
        mbs.AddObject(RevoluteJointZ(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=mbs.AddObject(ObjectGround()))), marker]))
    try:
        mbs.Assemble()
    except Exception as e:
        return int(message in str(e))
    return 0

refused = (Refused(1, 'Orientation', joint=True) + Refused(0, 'bodyNumbers is empty')
           + Refused(1, 'rotationMatrixUserFunction is given, but providedMarkerType is Position', rotation=UFrotation)
           + Refused(1, 'providedMarkerType is Position|Orientation, but the rotationMatrixUserFunction is not given', providedMarkerType=rigidType)
           + Refused(1, 'providedMarkerType is 128, but must be', providedMarkerType=exu.MarkerType.Coordinate))
mbsType = SC.AddSystem()
nodeType = mbsType.AddNode(NodePoint())
markerType = mbsType.AddMarker(MarkerBodyUser(bodyNumbers=[mbsType.AddObject(MassPoint(nodeNumber=nodeType, mass=1))],
                                              providedMarkerType=rigidType, positionUserFunction=UFmidpoint, rotationMatrixUserFunction=UFrotation))
refused += int(int(mbsType.GetMarker(markerType)['providedMarkerType']) == int(rigidType)) + int(mbsType.GetMarker(markerType)['markerType'] == 'BodyUser')
exu.Print('refused and types, of 7:', refused)

u += int(differenceMidpoint > 1e-10) + int(differencePendulum > 1e-8) + (7 - refused)
exu.Print('solution of userMarkerTest=', u)

exu.sys['testResult'] = u
