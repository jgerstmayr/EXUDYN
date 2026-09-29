#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class MarkerNodeRotationCoordinate
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

#a rotation coordinate of a rigid body node, held by a coordinate constraint
inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0,0,0, 0,0,0]))
mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(),
                              physicsInertia=inertia.GetInertia6D()))
mRotZ = mbs.AddMarker(MarkerNodeRotationCoordinate(nodeNumber=node, rotationCoordinate=2))
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
oHold = mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mGround, mRotZ]))
mbs.AddLoad(LoadTorqueVector(markerNumber=mbs.AddMarker(MarkerNodeRigid(nodeNumber=node)), loadVector=[0,0,2]))

mbs.Assemble()
mbs.SolveDynamic()

#the constraint holds the rotation about z against the torque: its force is the reaction torque
exu.sys['testResult'] = mbs.GetObjectOutput(oHold, exu.OutputVariableType.Force) #2

exu.Print("example for MarkerNodeRotationCoordinate completed, test result =", exu.sys['testResult'])

