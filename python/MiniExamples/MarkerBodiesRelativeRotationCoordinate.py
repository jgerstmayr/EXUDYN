#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class MarkerBodiesRelativeRotationCoordinate
# 
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
# 
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics

import numpy as np

#create an environment for mini example
SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))

#the rotation of body 1 relative to body 0 about an axis of body 0, held by a coordinate constraint;
#the data node continues the angle beyond +-pi
inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0,0,0, 0,0,0]))
body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(),
                                     inertia=inertia.GetInertia6D()))
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=1, initialCoordinates=[0]))
mRel = mbs.AddMarker(MarkerBodiesRelativeRotationCoordinate(bodyNumbers=[oGround, body], axis0=[0,0,1],
                                                           nodeNumber=nData))
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
oHold = mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mGround, mRel]))
mbs.AddLoad(LoadTorqueVector(markerNumber=mbs.AddMarker(MarkerNodeRigid(nodeNumber=node)), loadVector=[0,0,2]))

mbs.Assemble()
mbs.SolveDynamic()

#the constraint holds the relative rotation about z against the torque: its force is the reaction torque
exu.sys['testResult'] = mbs.GetObjectOutput(oHold, exu.OutputVariableType.Force) #2

exu.Print("example for MarkerBodiesRelativeRotationCoordinate completed, test result =", exu.sys['testResult'])

