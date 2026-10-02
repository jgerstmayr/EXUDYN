#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The HT parameters of the items (#2793): ObjectGround takes its frame as referencePosition and
#           referenceRotation or at once as referenceHT - a 4x4 matrix, its 16 values or an exu.HT. A parameter left
#           None is not given; GetObject lists all three, and such a dictionary can be added again; an HT and a part
#           given together must agree, else the item raises. SetObjectParameter of a part keeps the other part.
#           CreateGround and CreateRigidBody take referenceHT and initialHT for the position and rotation matrix;
#           for the three rigid body nodes they give the same coordinates, and an HT with one of its parts raises.
#           The rigid markers take localHT, the marker frame being the body or node frame times localHT: the rotation
#           matrix and the local angular velocity of MarkerBodyRigid, MarkerNodeRigid and MarkerKinematicTreeRigid; a node marker refuses a
#           translation; a joint whose markers carry its rotation in localHT moves as one with rotationMarker0/1.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-03
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()

A = RotXYZ2RotationMatrix([0.3, -0.2, 0.7])
p = np.array([1., 2., 3.])
H44 = HomogeneousTransformation(A, p)
localPosition = [0.5, -0.1, 0.2]
pointExpected = A @ localPosition + p

errors = []
total = 0.
#the same ground frame in four ways
grounds = [mbs.AddObject(ObjectGround(referencePosition=p, referenceRotation=A)),
           mbs.AddObject(ObjectGround(referenceHT=H44)),
           mbs.AddObject(ObjectGround(referenceHT=H44.flatten())),       #16 values, as a sensor stores an HT
           mbs.AddObject(ObjectGround(referenceHT=exu.HT(A, p)))]
for g in grounds:
    errors += [np.abs(mbs.GetObjectOutputBody(g, exu.OutputVariableType.Position, localPosition,
                                               exu.ConfigurationType.Reference) - pointExpected).max()]

#the dictionary lists the frame three times; it can be added again
d = mbs.GetObject(grounds[1])
errors += [np.abs(d['referencePosition'] - p).max(), np.abs(d['referenceRotation'] - A).max(),
           np.abs(d['referenceHT'] - H44).max()]
d = {key: d[key] for key in ['objectType', 'referencePosition', 'referenceRotation', 'referenceHT']}
gCopy = mbs.AddObject(d)
errors += [np.abs(mbs.GetObjectParameter(gCopy, 'referenceHT') - H44).max()]

#default: the identity
gDefault = mbs.AddObject(ObjectGround())
errors += [np.abs(mbs.GetObjectParameter(gDefault, 'referenceHT') - np.eye(4)).max()]

#a part keeps the other part
mbs.SetObjectParameter(gDefault, 'referencePosition', p)
mbs.SetObjectParameter(gDefault, 'referenceRotation', A)
errors += [np.abs(mbs.GetObjectParameter(gDefault, 'referenceHT') - H44).max()]
mbs.SetObjectParameter(gDefault, 'referenceHT', np.eye(4))
errors += [np.abs(mbs.GetObjectParameter(gDefault, 'referencePosition')).max()]

#an HT and a part that differ: the item raises
raised = 0
for parameters in [{'referenceHT': H44, 'referencePosition': [0, 0, 0]},
                   {'referenceHT': H44, 'referenceRotation': np.eye(3)},
                   {'referenceHT': A}]: #not a 4x4 matrix
    try:
        mbs.AddObject(ObjectGround(**parameters))
    except Exception:
        raised += 1
errors += [3 - raised]

#CreateGround and CreateRigidBody: referenceHT and initialHT for the position and rotation matrix (#2794)
gCreated = mbs.CreateGround(referenceHT=H44)
errors += [np.abs(mbs.GetObjectParameter(gCreated, 'referenceHT') - H44).max()]
Ainit = RotXYZ2RotationMatrix([0.1, 0.2, -0.3])
dInit = np.array([0.1, -0.2, 0.05])
inertia = InertiaCuboid(density=1000, sideLengths=[0.4, 0.2, 0.1])
for nodeType in [exu.NodeType.RotationEulerParameters, exu.NodeType.RotationRxyz, exu.NodeType.RotationRotationVector]:
    bParts = mbs.CreateRigidBody(inertia=inertia, nodeType=nodeType, referencePosition=p, referenceRotationMatrix=A,
                                 initialDisplacement=dInit, initialRotationMatrix=Ainit, returnDict=True)
    bHT = mbs.CreateRigidBody(inertia=inertia, nodeType=nodeType, referenceHT=exu.HT(A, p),
                              initialHT=HomogeneousTransformation(Ainit, dInit), returnDict=True)
    for key in ['referenceCoordinates', 'initialCoordinates']:
        errors += [np.abs(np.array(mbs.GetNodeParameter(bParts['nodeNumber'], key))
                          - mbs.GetNodeParameter(bHT['nodeNumber'], key)).max()]
raised = 0
for parameters in [{'referenceHT': H44, 'referencePosition': p}, {'initialHT': H44, 'initialRotationMatrix': A},
                   {'referenceHT': A}]:
    try:
        mbs.CreateRigidBody(inertia=inertia, **parameters)
    except Exception:
        raised += 1
errors += [3 - raised]

#the rigid markers (#2795): the marker frame is the body or node frame times localHT
SC2 = exu.SystemContainer()
mbs2 = SC2.AddSystem()
Al = RotXYZ2RotationMatrix([0.4, 0.1, -0.5])
pl = np.array([0.2, -0.1, 0.3])
oBody = mbs2.CreateRigidBody(inertia=inertia, referencePosition=p, referenceRotationMatrix=A,
                             initialAngularVelocity=[0.3, -1.2, 2.], initialVelocity=[0.1, 0.2, 0.3])
nBody = mbs2.GetObject(oBody)['nodeNumber']
mBodyHT = mbs2.AddMarker(MarkerBodyRigid(bodyNumber=oBody, localHT=HomogeneousTransformation(Al, pl)))
mBodyParts = mbs2.AddMarker(MarkerBodyRigid(bodyNumber=oBody, localPosition=pl))
mNodeHT = mbs2.AddMarker(MarkerNodeRigid(nodeNumber=nBody, localHT=exu.HT(rotation=Al)))
mbs2.Assemble()
OV = exu.OutputVariableType
omegaLocal = mbs2.GetMarkerOutput(mBodyParts, OV.AngularVelocityLocal)
for m in [mBodyHT, mNodeHT]:
    errors += [np.abs(mbs2.GetMarkerOutput(m, OV.RotationMatrix).reshape(3,3) - A @ Al).max(),
               np.abs(mbs2.GetMarkerOutput(m, OV.AngularVelocityLocal) - Al.T @ omegaLocal).max(),
               np.abs(mbs2.GetMarkerOutput(m, OV.AngularVelocity) - mbs2.GetMarkerOutput(mBodyParts, OV.AngularVelocity)).max()]
errors += [np.abs(mbs2.GetMarkerOutput(mBodyHT, OV.Position) - mbs2.GetMarkerOutput(mBodyParts, OV.Position)).max(),
           np.abs(mbs2.GetMarkerOutput(mBodyHT, OV.Velocity) - mbs2.GetMarkerOutput(mBodyParts, OV.Velocity)).max(),
           np.abs(mbs2.GetMarker(mBodyHT)['localPosition'] - pl).max()]

#a kinematic tree: the rotation of localHT on the link frame
nTree = mbs2.AddNode(NodeGenericODE2(referenceCoordinates=[0.], initialCoordinates=[0.3],
                                     initialCoordinates_t=[0.7], numberOfODE2Coordinates=1))
oTree = mbs2.AddObject(ObjectKinematicTree(nodeNumber=nTree, jointTypes=[exu.JointType.RevoluteZ], linkParents=[-1],
                                           jointTransformations=exu.Matrix3DList([np.eye(3)]),
                                           jointOffsets=exu.Vector3DList([[0,0,0]]),
                                           linkInertiasCOM=exu.Matrix3DList([np.eye(3)]),
                                           linkCOMs=exu.Vector3DList([[0,0,0]]), linkMasses=[2.]))
mLinkHT = mbs2.AddMarker(MarkerKinematicTreeRigid(objectNumber=oTree, linkNumber=0, localHT=HomogeneousTransformation(Al, pl)))
mLink = mbs2.AddMarker(MarkerKinematicTreeRigid(objectNumber=oTree, linkNumber=0, localPosition=pl))
mbs2.Assemble()
errors += [np.abs(mbs2.GetMarkerOutput(mLinkHT, OV.RotationMatrix).reshape(3,3)
                  - mbs2.GetMarkerOutput(mLink, OV.RotationMatrix).reshape(3,3) @ Al).max(),
           np.abs(mbs2.GetMarkerOutput(mLinkHT, OV.AngularVelocityLocal) - Al.T @ mbs2.GetMarkerOutput(mLink, OV.AngularVelocityLocal)).max(),
           np.abs(mbs2.GetMarkerOutput(mLinkHT, OV.Position) - mbs2.GetMarkerOutput(mLink, OV.Position)).max()]

#a node marker with a translation in localHT is refused at Assemble
mbs2.AddMarker(MarkerNodeRigid(nodeNumber=nBody, localHT=HomogeneousTransformation(np.eye(3), pl)))
try:
    mbs2.Assemble()
    errors += [1]
except Exception:
    errors += [0]

#a joint: the rotation in localHT of its markers or in rotationMarker0/1 of the joint gives the same motion
def SwingingBody(useLocalHT, nodeMarker):
    SCj = exu.SystemContainer()
    mbsj = SCj.AddSystem()
    gGround = mbsj.AddObject(ObjectGround())
    oPendulum = mbsj.CreateRigidBody(inertia=inertia, referencePosition=[0.5, 0, 0], gravity=[0, -9.81, 0],
                                     initialAngularVelocity=[0.2, 0.1, 0.])
    Aaxis = RotXYZ2RotationMatrix([0.3, -0.4, 0.2]) #the joint axis z turned
    HT0 = HomogeneousTransformation(Aaxis, [0, 0, 0])
    HT1 = HomogeneousTransformation(Aaxis, [-0.5, 0, 0])
    mGround = mbsj.AddMarker(MarkerBodyRigid(bodyNumber=gGround, localHT=HT0 if useLocalHT else None))
    if nodeMarker: #the joint at the node of the body, its rotation in the node marker
        mPendulum = mbsj.AddMarker(MarkerNodeRigid(nodeNumber=mbsj.GetObject(oPendulum)['nodeNumber'],
                                                   localHT=HT0 if useLocalHT else None))
        mGround = mbsj.AddMarker(MarkerBodyRigid(bodyNumber=gGround, localPosition=[0.5, 0, 0],
                                                 localHT=None))
        if useLocalHT:
            mbsj.SetMarkerParameter(mGround, 'localHT', HomogeneousTransformation(Aaxis, [0.5, 0, 0]))
    else:
        mPendulum = mbsj.AddMarker(MarkerBodyRigid(bodyNumber=oPendulum, localHT=HT1 if useLocalHT else None,
                                                   localPosition=None if useLocalHT else [-0.5, 0, 0]))
    if useLocalHT:
        mbsj.AddObject(ObjectJointGeneric(markerNumbers=[mGround, mPendulum], constrainedAxes=[1,1,1,1,1,0]))
    else:
        mbsj.AddObject(ObjectJointGeneric(markerNumbers=[mGround, mPendulum], constrainedAxes=[1,1,1,1,1,0],
                                          rotationMarker0=Aaxis, rotationMarker1=Aaxis))
    mbsj.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 200
    simulationSettings.timeIntegration.endTime = 0.5
    simulationSettings.timeIntegration.verboseMode = 0
    mbsj.SolveDynamic(simulationSettings)
    return mbsj.GetObjectOutputBody(oPendulum, OV.Position, [0.5, 0.1, 0.])

for nodeMarker in [False, True]:
    pRotationMarkers = SwingingBody(False, nodeMarker)
    pLocalHT = SwingingBody(True, nodeMarker)
    exu.Print('joint with localHT, nodeMarker =', nodeMarker, ':', pLocalHT, ', difference', np.abs(pLocalHT - pRotationMarkers).max())
    errors += [np.abs(pLocalHT - pRotationMarkers).max()*1e6] #both solved the same equations
    total += np.sum(pLocalHT)

exu.Print('homogeneousTransformationParameterTest: errors', np.round(errors, 15))
u = sum(errors) + np.sum(mbs.GetObjectParameter(gCopy, 'referenceHT')) + total
exu.Print('solution of homogeneousTransformationParameterTest=', u)

exu.sys['testResult'] = u
