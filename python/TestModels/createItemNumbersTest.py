#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The Create functions that connect two items take them in itemNumbers (#2863): a body at a
#           local position, a node or a marker, as far as the function accepts it; CreateForce and
#           CreateTorque take one item in itemNumber. Each function is called with every kind it
#           accepts, and the motion is the same for each; a kind it does not accept is a ValueError, as
#           is a local position for a node. bodyOrNodeList, and bodyNumbers given together with
#           itemNumbers, are a TypeError; bodyNumbers alone is the deprecated name and does the same as
#           itemNumbers.
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
import warnings

testIsActive = exu.sys.get('testIsActive', False)

errors = 0
def Check(condition, text):
    global errors
    if not condition:
        errors += 1
        exu.Print('createItemNumbersTest failed:', text)

def Solve(mbs, endTime=0.2):
    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 100
    simulationSettings.timeIntegration.endTime = endTime
    simulationSettings.solution.file.write = False
    simulationSettings.timeIntegration.verboseMode = 0
    mbs.SolveDynamic(simulationSettings)

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#a mass point, connected to the ground as a body, through its node, or through a marker
def MassPoint(kind, Connect):
    """the position of a mass point after 0.2 s, which Connect(mbs, oGround, item) connects to the ground"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.CreateGround()
    oMass = mbs.CreateMassPoint(referencePosition=[1,0,0], initialVelocity=[0,0.5,0], mass=2, gravity=[0,-9.81,0])
    nMass = mbs.GetObject(oMass)['nodeNumber']
    item = {'body': oMass, 'node': nMass,
            'marker': mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass)) if kind == 'marker' else None}[kind]
    Connect(mbs, oGround, item)
    Solve(mbs)
    return mbs.GetNodeOutput(nMass, exu.OutputVariableType.Position)

massPointConnections = {
    'CreateSpringDamper': lambda mbs, g, item: mbs.CreateSpringDamper(itemNumbers=[g, item], localPosition0=[0,0.5,0], stiffness=200, damping=2),
    'CreateCartesianSpringDamper': lambda mbs, g, item: mbs.CreateCartesianSpringDamper(itemNumbers=[g, item], localPosition0=[1,0,0],
                                                                                        stiffness=[100,200,0], damping=[1,2,0]),
    'CreateDistanceConstraint': lambda mbs, g, item: mbs.CreateDistanceConstraint(itemNumbers=[g, item], localPosition0=[0,0.5,0]),
    'CreateForce': lambda mbs, g, item: mbs.CreateForce(itemNumber=item, loadVector=[5,0,0]),
    'CreateSphereSphereContact': lambda mbs, g, item: mbs.CreateSphereSphereContact(itemNumbers=[g, item], localPosition0=[1,-0.6,0],
                                                                                    spheresRadii=[0.5, 0.1], contactStiffness=1e4,
                                                                                    contactDamping=10),
    }
u = 0
for (name, Connect) in massPointConnections.items():
    results = [MassPoint(kind, Connect) for kind in ['body', 'node', 'marker']]
    Check(np.linalg.norm(results[1]-results[0]) < 1e-12 and np.linalg.norm(results[2]-results[0]) < 1e-12, name + ': ' + str(results))
    u += np.sum(results[0])
    exu.Print(name, results[0])

#a coordinate constraint takes a body or a node, and None or a ground object for the ground
for name in ['bodyGround', 'nodeGround', 'bodyNone']:
    def Constrain(mbs, g, item):
        mbs.CreateCoordinateConstraint(itemNumbers=[None if name == 'bodyNone' else g, item], coordinates=[None, 1])
    result = MassPoint('node' if name == 'nodeGround' else 'body', Constrain)
    Check(abs(result[1]) < 1e-12 and abs(result[0]-1) < 1e-12, 'CreateCoordinateConstraint ' + name + ': ' + str(result))

#the coordinate of a body with several nodes counts over its nodes
SC = exu.SystemContainer()
mbs = SC.AddSystem()
n0 = mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[0,0,1,0]))
n1 = mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[1,0,1,0]))
oCable = mbs.AddObject(ObjectANCFCable2D(nodeNumbers=[n0, n1], length=1, massPerLength=1,
                                         bendingStiffness=1, axialStiffness=100))
oConstraint = mbs.CreateCoordinateConstraint(itemNumbers=[None, oCable], coordinates=[None, 5])
marker = mbs.GetMarker(mbs.GetObject(oConstraint)['markerNumbers'][1])
Check(int(marker['nodeNumber']) == int(n1) and marker['coordinate'] == 1, 'CreateCoordinateConstraint of a cable: ' + str(marker))

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ONLY the regular module refuses the wrong kind of item: exudynCPPfast skips the checks of the Create functions by
#design, so there a wrong item is not refused, or fails later with another error (#2911)
hasItemChecks = '[FAST]' not in exu.config.Version(True)

#a rigid body, connected as a body, through its node (where accepted) or through a marker
def RigidBody(kind, Connect):
    """the position and the rotation matrix of a rigid body after 0.2 s, which Connect(mbs, oGround, item) connects"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.CreateGround()
    oBody = mbs.CreateRigidBody(inertia=InertiaCuboid(density=1000, sideLengths=[1,0.1,0.1]), referencePosition=[1,0,0],
                                initialAngularVelocity=[0,0.5,1], gravity=[0,-9.81,0])
    nBody = mbs.GetObject(oBody)['nodeNumber']
    item = {'body': oBody, 'node': nBody, 'marker': mbs.AddMarker(MarkerBodyRigid(bodyNumber=oBody, localPosition=[-0.5,0,0]))
            if kind == 'marker' else None}[kind]
    Connect(mbs, oGround, item, kind)
    Solve(mbs)
    return np.hstack([mbs.GetNodeOutput(nBody, exu.OutputVariableType.Position),
                      mbs.GetNodeOutput(nBody, exu.OutputVariableType.RotationMatrix)])

def JointPosition(kind):
    """the joint at [0.5,0,0]: given as position, or set by the marker"""
    return [] if kind == 'marker' else [0.5,0,0]

rigidBodyConnections = {
    'CreateRevoluteJoint': lambda mbs, g, item, kind: mbs.CreateRevoluteJoint(itemNumbers=[g, item], position=JointPosition(kind), axis=[0,0,1]),
    'CreatePrismaticJoint': lambda mbs, g, item, kind: mbs.CreatePrismaticJoint(itemNumbers=[g, item], position=JointPosition(kind), axis=[0,1,0]),
    'CreateSphericalJoint': lambda mbs, g, item, kind: mbs.CreateSphericalJoint(itemNumbers=[g, item], position=JointPosition(kind)),
    'CreateGenericJoint': lambda mbs, g, item, kind: mbs.CreateGenericJoint(itemNumbers=[g, item], position=JointPosition(kind),
                                                                            constrainedAxes=[1,1,1,1,1,0]),
    'CreateTorsionalSpringDamper': lambda mbs, g, item, kind: mbs.CreateTorsionalSpringDamper(itemNumbers=[g, item], position=JointPosition(kind),
                                                                                              axis=[0,0,1], stiffness=10, damping=0.1),
    'CreateRigidBodySpringDamper': lambda mbs, g, item, kind: mbs.CreateRigidBodySpringDamper(itemNumbers=[g, item],
                                      localPosition0=[0.5,0,0] if kind == 'marker' else [1,0,0],
                                      stiffness=np.diag([1e3,1e3,1e3,10,10,10]), damping=np.diag([1,1,1,0.1,0.1,0.1])),
    'CreateTorque': lambda mbs, g, item, kind: mbs.CreateTorque(itemNumber=item, loadVector=[0,0,2]),
    }
#which give the same result through the node: the spring-damper and the torque, at the center of mass; the joints take no node
takesNode = ['CreateRigidBodySpringDamper', 'CreateTorque']
for (name, Connect) in rigidBodyConnections.items():
    kinds = ['body', 'marker'] + (['node'] if name in takesNode else [])
    if name in takesNode: #the marker at the center of mass, as body and node
        Connect0 = Connect
        def Connect(mbs, g, item, kind, Connect0=Connect0):
            if kind == 'marker':
                item = mbs.AddMarker(MarkerBodyRigid(bodyNumber=mbs.GetMarker(item)['bodyNumber'], localPosition=[0,0,0]))
                kind = 'body'
            return Connect0(mbs, g, item, kind)
    results = [RigidBody(kind, Connect) for kind in kinds]
    Check(all(np.linalg.norm(result-results[0]) < 1e-10 for result in results), name + ': ' + str([list(r[:3]) for r in results]))
    u += np.sum(results[0][:3])
    exu.Print(name, results[0][:3])
    if name not in takesNode and hasItemChecks:
        try:
            RigidBody('node', Connect)
            Check(False, name + ' accepts a node')
        except ValueError: #the wrong kind of item
            pass

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the sphere of a sphere-triangle contact may be on a node or a marker, the triangle only on a body
SC = exu.SystemContainer()
mbs = SC.AddSystem()
oGround = mbs.CreateGround()
oMass = mbs.CreateMassPoint(referencePosition=[0.2,0.2,0.2], mass=1)
nMass = mbs.GetObject(oMass)['nodeNumber']
mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=mbs.CreateRigidBody(inertia=InertiaSphere(1,0.1), returnDict=True)['nodeNumber']))
mbs.CreateSphereTriangleContact(itemNumbers=[mNode, oGround], sphereRadius=0.1, contactStiffness=1e4)
mbs.CreateSphereQuadContact(itemNumbers=[nMass, oGround], sphereRadius=0.1, contactStiffness=1e4)
if hasItemChecks:
    for itemNumbers in [[oGround, nMass], [oMass, mNode]]:
        try:
            mbs.CreateSphereQuadContact(itemNumbers=itemNumbers, sphereRadius=0.1, contactStiffness=1e4)
            Check(False, 'the quad accepts ' + str(itemNumbers[1]))
        except ValueError: #the wrong kind of item
            pass
    #a node needs no local position
    try:
        mbs.CreateSpringDamper(itemNumbers=[oGround, nMass], localPosition1=[1,0,0])
        Check(False, 'a node with a local position')
    except ValueError:
        pass

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#bodyNumbers is the deprecated name of itemNumbers; bodyOrNodeList is not accepted, nor two names at once
deprecations = exu.special.deprecations
warnOnceStored = deprecations.warnOnce
deprecations.warnOnce = False
with warnings.catch_warnings(record=True) as caught:
    warnings.simplefilter('always')
    resultOld = MassPoint('body', lambda mbs, g, item: mbs.CreateSpringDamper(bodyNumbers=[g, item], localPosition0=[0,0.5,0], stiffness=200, damping=2))
    forceOld = MassPoint('body', lambda mbs, g, item: mbs.CreateForce(bodyNumber=item, loadVector=[5,0,0]))
deprecations.warnOnce = warnOnceStored
caught = [str(entry.message) for entry in caught if issubclass(entry.category, DeprecationWarning)]
Check(len(caught) == 2 and 'use itemNumbers' in caught[0] and 'use itemNumber' in caught[1], 'warnings: ' + str(caught))
Check(np.linalg.norm(resultOld - MassPoint('body', massPointConnections['CreateSpringDamper'])) == 0, 'bodyNumbers')
Check(np.linalg.norm(forceOld - MassPoint('body', massPointConnections['CreateForce'])) == 0, 'bodyNumber')
for (arguments, text) in [({'bodyOrNodeList': [oGround, oMass]}, 'use itemNumbers'),
                          ({'itemNumbers': [oGround, oMass], 'bodyNumbers': [oGround, oMass]}, 'use itemNumbers only')]:
    try:
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            mbs.CreateDistanceConstraint(**arguments)
        Check(False, str(arguments))
    except TypeError as error:
        Check(text in str(error), str(error))

exu.Print('createItemNumbersTest: errors', errors)
u += errors
exu.Print('solution of createItemNumbersTest=', u)

exu.sys['testResult'] = u
