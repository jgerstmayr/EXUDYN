#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ObjectConnectorUser (#2975, #3119): its forceUserFunction of the marker states, which returns the forces,
#           and its markerAccessUserFunction, which gets the markers as exudyn.ConnectorMarker objects. (1) A Cartesian
#           spring-damper between two mass points, as forceUserFunction and as markerAccessUserFunction with
#           GetPosition, GetVelocity and ApplyForce, against CartesianSpringDamper; a rotational damper and a spring
#           of a rigid body to the ground as forceUserFunction with the rotations of the markers. (2) A rigid body with a
#           rotational damper (GetAngularVelocity, ApplyTorque) and a spring at a point of the body (GetBodyPosition,
#           GetBodyVelocity, ApplyBodyForce), against RigidBodySpringDamper and CartesianSpringDamper. (3) A ball
#           resting on a simply supported ANCF cable: the contact searches the closest point on each element of the
#           cable and acts there; the static solution carries the weight and deflects the cable as a point load
#           does. (4) What a ConnectorMarker refuses; a marker without the requestedMarkerType refused at Assemble; both user
#           functions, a forceUserFunction of one marker or of a wrong result size refused.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-23
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.cMath as cMath
import numpy as np

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 200
simulationSettings.timeIntegration.endTime = 0.5
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-10
simulationSettings.solution.file.write = False

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(1) a Cartesian spring-damper as user connector
k, d = [200., 300., 400.], [2., 1., 3.]
offset = [0.5, 0., 0.1]

def UFspringDamper(mbs, t, itemNumber, markers, dataCoordinates, parameters):
    (m0, m1) = markers
    force = np.array(k)*(m1.GetPosition() - m0.GetPosition() - offset) + np.array(d)*(m1.GetVelocity() - m0.GetVelocity())
    m0.ApplyForce(force)
    m1.ApplyForce(-force)

#the same as forceUserFunction of the marker states, returning the forces on marker 0 and marker 1
def UFspringDamperForces(mbs, t, itemNumber, HT0, twist0, HT1, twist1, dataCoordinates, parameters):
    force = np.array(k)*(HT1.translation - HT0.translation - offset) + np.array(d)*(twist1.velocity - twist0.velocity)
    return np.hstack([force, -force])

def SpringModel(isUser):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    nodes = [mbs.AddNode(NodePoint(referenceCoordinates=p, initialVelocities=v))
             for (p, v) in [([0, 0, 0], [0.1, 0, 0]), ([0.6, 0.1, 0.2], [0, -0.5, 0.3])]]
    for (node, m) in zip(nodes, [1., 2.]):
        mbs.AddObject(MassPoint(nodeNumber=node, mass=m))
    markers = [mbs.AddMarker(MarkerNodePosition(nodeNumber=node)) for node in nodes]
    if isUser == 'markerAccess':
        mbs.AddObject(ObjectConnectorUser(markerNumbers=markers, markerAccessUserFunction=UFspringDamper))
    elif isUser == 'forces':
        mbs.AddObject(ObjectConnectorUser(markerNumbers=markers, forceUserFunction=UFspringDamperForces))
    else:
        mbs.AddObject(CartesianSpringDamper(markerNumbers=markers, stiffness=k, damping=d, offset=offset))
    mbs.AddLoad(LoadForceVector(markerNumber=markers[1], loadVector=[0, -9.81*2, 0]))
    mbs.Assemble()
    mbs.SolveDynamic(simulationSettings)
    return np.array([mbs.GetNodeOutput(node, exu.OutputVariableType.Position) for node in nodes])

springs = [SpringModel(isUser) for isUser in [False, 'markerAccess', 'forces']]
differenceSpring = max(np.max(np.abs(springs[1] - springs[0])), np.max(np.abs(springs[2] - springs[0])))
exu.Print('(1) spring-damper: largest difference to CartesianSpringDamper', differenceSpring)

#the rotational damper and a spring of the body origin to the ground as forceUserFunction: the angular velocities and
#a torque of the markers, with Orientation requested
def UFrigidForces(mbs, t, itemNumber, HT0, twist0, HT1, twist1, dataCoordinates, parameters):
    torque = -dRot*(twist1.angularVelocity - twist0.angularVelocity)
    force = -kPoint*(HT1.translation - HT0.translation) - 0.1*(twist1.velocity - twist0.velocity)
    return np.hstack([-force, -torque, force, torque])

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(2) a rigid body: a rotational damper and a spring at a point of the body
dRot = 0.5          #rotational damping
kPoint = 50.        #stiffness of the spring at pPoint
pPoint = [0.3, 0.1, 0]

def UFrigid(mbs, t, itemNumber, markers, dataCoordinates, parameters):
    (ground, body) = markers
    body.ApplyTorque(-dRot*body.GetAngularVelocity())
    #the spring to the ground position of pPoint in the reference configuration, at the point of the body
    displacement = body.GetBodyPosition(pPoint) - (ground.GetPosition() + pPoint)
    force = -kPoint*displacement - 0.1*body.GetBodyVelocity(pPoint)
    body.ApplyBodyForce(force, pPoint)
    ground.ApplyForce(-force)

def RigidForcesModel(isUser):
    """the rotational damper and a spring at the origin of the body, as forceUserFunction or as built-in connectors"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    inertia = InertiaCuboid(density=1000, sideLengths=[0.4, 0.2, 0.1])
    body = mbs.CreateRigidBody(inertia=inertia, referencePosition=[0, 0, 0], initialAngularVelocity=[1, 2, 5],
                               initialVelocity=[0.2, 0, 0], gravity=[0, -9.81, 0])
    ground = mbs.AddObject(ObjectGround())
    markers = [mbs.AddMarker(MarkerBodyRigid(bodyNumber=item)) for item in [ground, body]]
    if isUser:
        mbs.AddObject(ObjectConnectorUser(markerNumbers=markers, forceUserFunction=UFrigidForces,
                                          requestedMarkerType=exu.MarkerType.Position | exu.MarkerType.Orientation))
    else:
        mbs.AddObject(RigidBodySpringDamper(markerNumbers=markers, stiffness=np.diag([kPoint]*3 + [0]*3),
                                            damping=np.diag([0.1]*3 + [dRot]*3)))
    mbs.Assemble()
    mbs.SolveDynamic(simulationSettings)
    return np.hstack([mbs.GetObjectOutputBody(body, exu.OutputVariableType.Position, localPosition=[0, 0, 0]),
                      mbs.GetObjectOutputBody(body, exu.OutputVariableType.AngularVelocity, localPosition=[0, 0, 0])])

rigidForces = [RigidForcesModel(isUser) for isUser in [False, True]]
differenceRigidForces = np.max(np.abs(rigidForces[1] - rigidForces[0]))
exu.Print('(2) rigid body, forceUserFunction with torques: largest difference to RigidBodySpringDamper', differenceRigidForces)

def RigidModel(isUser):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    inertia = InertiaCuboid(density=1000, sideLengths=[0.4, 0.2, 0.1])
    body = mbs.CreateRigidBody(inertia=inertia, referencePosition=[0, 0, 0], initialAngularVelocity=[1, 2, 5],
                               initialVelocity=[0.2, 0, 0], gravity=[0, -9.81, 0])
    ground = mbs.AddObject(ObjectGround())
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=ground))
    if isUser:
        mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body))
        mbs.AddObject(ObjectConnectorUser(markerNumbers=[mGround, mBody], markerAccessUserFunction=UFrigid,
                                          requestedMarkerType=exu.MarkerType.Position | exu.MarkerType.Orientation))
    else:
        mbs.AddObject(RigidBodySpringDamper(markerNumbers=[mGround, mbs.AddMarker(MarkerBodyRigid(bodyNumber=body))],
                                            stiffness=np.zeros((6, 6)), damping=np.diag([0, 0, 0, dRot, dRot, dRot])))
        mbs.AddObject(CartesianSpringDamper(markerNumbers=[mbs.AddMarker(MarkerBodyPosition(bodyNumber=ground, localPosition=pPoint)),
                                                           mbs.AddMarker(MarkerBodyPosition(bodyNumber=body, localPosition=pPoint))],
                                            stiffness=[kPoint]*3, damping=[0.1]*3))
    mbs.Assemble()
    mbs.SolveDynamic(simulationSettings)
    return np.hstack([mbs.GetObjectOutputBody(body, exu.OutputVariableType.Position, localPosition=pPoint),
                      mbs.GetObjectOutputBody(body, exu.OutputVariableType.AngularVelocity, localPosition=[0, 0, 0])])

rigid = [RigidModel(isUser) for isUser in [False, True]]
differenceRigid = np.max(np.abs(rigid[1] - rigid[0]))
exu.Print('(2) rigid body: largest difference to RigidBodySpringDamper and CartesianSpringDamper', differenceRigid)

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(3) a ball on a simply supported ANCF cable, the closest point searched on each element
nElements = 4
length = 2.
Le = length/nElements
radius = 0.1
massBall = 0.5
gravity = 9.81
kContact = 1e5      #penalty stiffness of the contact
EI, EA = 10., 1e5

def ClosestPoint(marker, point):
    """the local x of the point on the axis of the element closest to point: a golden-section search, then Newton on
    the condition that the distance vector is normal to the axis, to rounding - a coarser point makes the numerical
    Jacobian of the connector noisy"""
    def Distance2(x):
        difference = marker.GetBodyPosition([x, 0, 0]) - point
        return difference @ difference
    (a, b) = (0., Le)
    ratio = 0.5*(cMath.sqrt(5.) - 1)
    for i in range(15):
        x1, x2 = b - ratio*(b - a), a + ratio*(b - a)
        if Distance2(x1) < Distance2(x2):
            b = x2
        else:
            a = x1
    x = 0.5*(a + b)
    h = 1e-5
    for i in range(4):
        r = marker.GetBodyPosition([x, 0, 0])
        tangent = (marker.GetBodyPosition([x + h, 0, 0]) - marker.GetBodyPosition([x - h, 0, 0]))/(2*h)
        curvature = (marker.GetBodyPosition([x + h, 0, 0]) - 2*r + marker.GetBodyPosition([x - h, 0, 0]))/h**2
        f = tangent @ (r - point)
        f_x = tangent @ tangent + curvature @ (r - point)
        x = min(Le, max(0., x - f/f_x))
    return x

contactForce = [np.zeros(3)]  #the contact force on the ball of the last call
def UFcontact(mbs, t, itemNumber, markers, dataCoordinates, parameters):
    """the contact at the closest point of the cable: of all elements the one closest to the ball, so that a ball
    above a node, which two elements share, touches once"""
    ball = markers[0]
    center = ball.GetPosition()
    closest = None
    for element in markers[1:]:
        x = ClosestPoint(element, center)
        difference = center - element.GetBodyPosition([x, 0, 0])
        distance = cMath.norm(difference)
        if closest is None or distance < closest[0]:
            closest = (distance, element, x, difference)
    (distance, element, x, difference) = closest
    gap = distance - radius
    contactForce[0] = np.zeros(3)
    if gap < 0:
        normal = difference/distance
        force = -kContact*gap*normal
        ball.ApplyForce(force)
        element.ApplyBodyForce(-force, [x, 0, 0])
        contactForce[0] = force

def CableModel(kind):
    """the cable with the ball ('contact') or with a point load of the weight at mid span ('load')"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    nodes = [mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[i*Le, 0, 1, 0])) for i in range(nElements + 1)]
    elements = [mbs.AddObject(ObjectANCFCable2D(nodeNumbers=nodes[i:i+2], length=Le, massPerLength=1, bendingStiffness=EI,
                                                axialStiffness=EA)) for i in range(nElements)]
    nGround = mbs.AddNode(NodePointGround())
    for (node, coordinates) in [(nodes[0], [0, 1]), (nodes[-1], [1])]: #pinned and on rollers
        for coordinate in coordinates:
            mbs.AddObject(CoordinateConstraint(markerNumbers=[mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0)),
                                                              mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=coordinate))]))
    ball = None
    if kind == 'contact':
        #in contact at the start, with the penetration of the weight; a weak spring holds x, as a frictionless contact on
        #the straight cable has no stiffness along it
        ball = mbs.AddNode(NodePoint2D(referenceCoordinates=[0.5*length, radius - massBall*gravity/kContact]))
        mbs.AddObject(MassPoint2D(nodeNumber=ball, mass=massBall))
        mBall = mbs.AddMarker(MarkerNodePosition(nodeNumber=ball))
        mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0)),
                                                            mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=ball, coordinate=0))],
                                             stiffness=1.))
        mbs.AddLoad(LoadForceVector(markerNumber=mBall, loadVector=[0, -massBall*gravity, 0]))
        markers = [mBall] + [mbs.AddMarker(MarkerBodyPosition(bodyNumber=element)) for element in elements]
        mbs.AddObject(ObjectConnectorUser(markerNumbers=markers, markerAccessUserFunction=UFcontact))
    else:
        mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerBodyPosition(bodyNumber=elements[1], localPosition=[Le, 0, 0])),
                                    loadVector=[0, -massBall*gravity, 0]))
    mbs.Assemble()
    staticSettings = exu.SimulationSettings()
    staticSettings.staticSolver.useLoadFactor = False #the whole weight from the start: less would lift the ball off
    staticSettings.staticSolver.newton.relativeTolerance = 1e-12
    staticSettings.solution.file.write = False
    mbs.SolveStatic(staticSettings)
    middle = mbs.GetObjectOutputBody(elements[1], exu.OutputVariableType.Position, localPosition=[Le, 0, 0])
    ballPosition = mbs.GetNodeOutput(ball, exu.OutputVariableType.Position) if ball is not None else None
    return (middle, ballPosition)

(middleContact, ballPosition) = CableModel('contact')
(middleLoad, _) = CableModel('load')
penetration = radius - (ballPosition[1] - middleContact[1])
#the cable stretches and its middle moves along x on the roller, the ball with it; the weak spring holds it there
errorsCable = [abs(contactForce[0][1] - massBall*gravity)/(massBall*gravity),           #the contact carries the weight
               abs(contactForce[0][0] - 1.*(ballPosition[0] - 0.5*length)),             #and the spring the horizontal force
               abs(penetration - massBall*gravity/kContact)/penetration,                #the penalty law, at the node
               np.max(np.abs(middleContact - middleLoad))]                              #as the point load
exu.Print('(3) ball on cable: mid-span deflection', middleContact[1], ', with the point load', middleLoad[1],
          ', contact force', contactForce[0][0:2], ', penetration', penetration, ', errors', errorsCable)

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(4) a torque on a position marker is refused, and a ConnectorMarker kept after the call
kept = []
def UFrefused(mbs, t, itemNumber, markers, dataCoordinates, parameters):
    kept.append(markers[0])
    if t > 0:
        markers[0].ApplyTorque([0, 0, 1])

refusedMessages = []
for when in ['during', 'after']:
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    node = mbs.AddNode(NodePoint())
    mbs.AddObject(MassPoint(nodeNumber=node, mass=1))
    mbs.AddObject(ObjectConnectorUser(markerNumbers=[mbs.AddMarker(MarkerNodePosition(nodeNumber=node))],
                                      markerAccessUserFunction=UFrefused if when == "during" else lambda mbs, t, i, markers, d, p: kept.append(markers[0])))
    mbs.Assemble()
    try:
        mbs.SolveDynamic(simulationSettings)
        if when == 'after':
            kept[-1].GetPosition()
    except ValueError as e:
        refusedMessages.append(str(e))
    except Exception as e: #the solver reports the error of the user function as its own
        refusedMessages.append(str(e))
#a position marker for a connector that requests Position and Orientation (#3105)
SC = exu.SystemContainer()
mbs = SC.AddSystem()
node = mbs.AddNode(NodePoint())
mbs.AddObject(MassPoint(nodeNumber=node, mass=1))
mbs.AddObject(ObjectConnectorUser(markerNumbers=[mbs.AddMarker(MarkerNodePosition(nodeNumber=node))], markerAccessUserFunction=UFrefused,
                                  requestedMarkerType=exu.MarkerType.Position | exu.MarkerType.Orientation))
try:
    mbs.Assemble()
except Exception as e:
    refusedMessages.append(str(e))
#the forceUserFunction (#3119): both user functions, one marker, a result of the wrong size
def RefusedForces(kind):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    nodes = [mbs.AddNode(NodePoint(referenceCoordinates=[i, 0, 0])) for i in range(2)]
    for node in nodes:
        mbs.AddObject(MassPoint(nodeNumber=node, mass=1))
    markers = [mbs.AddMarker(MarkerNodePosition(nodeNumber=node)) for node in nodes]
    if kind == 'both':
        mbs.AddObject(ObjectConnectorUser(markerNumbers=markers, forceUserFunction=UFspringDamperForces, markerAccessUserFunction=UFspringDamper))
    elif kind == 'oneMarker':
        mbs.AddObject(ObjectConnectorUser(markerNumbers=markers[0:1], forceUserFunction=UFspringDamperForces))
    else:
        mbs.AddObject(ObjectConnectorUser(markerNumbers=markers, forceUserFunction=lambda mbs, t, i, *states: [0, 0, 0]))
    try:
        mbs.Assemble()
        mbs.SolveDynamic(simulationSettings)
    except Exception as e:
        return str(e)
    return ''

refused = (int('are both given' in RefusedForces('both')) + int('needs two markers' in RefusedForces('oneMarker'))
            + int('returned 3 values, but 6' in RefusedForces('size')))
refused += (int(any('not of type Orientation' in m for m in refusedMessages)) + int(any('valid during that call only' in m for m in refusedMessages))
            + int(any("invalid type 'NodePosition'" in m for m in refusedMessages)))
exu.Print('(4) refused:', refused, 'of 6')

u = np.sum(springs[1]) + np.sum(rigid[1]) + middleContact[1]
u += int(differenceSpring > 1e-10) + int(differenceRigid > 1e-10) + int(differenceRigidForces > 1e-10) + int(max(errorsCable[0:2]) > 1e-8)
u += int(errorsCable[2] > 1e-2) + int(errorsCable[3] > 1e-5) + (6 - refused)
exu.Print('solution of userConnectorTest=', u)

exu.sys['testResult'] = u
