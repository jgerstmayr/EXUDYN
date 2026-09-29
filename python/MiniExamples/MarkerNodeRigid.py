#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class MarkerNodeRigid
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

#position and orientation of a rigid body node: a torque spins the body up
inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0,0,0, 0,0,0]))
mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(),
                              physicsInertia=inertia.GetInertia6D()))
mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node))
mbs.AddLoad(LoadTorqueVector(markerNumber=mNode, loadVector=[0,0,1]))

mbs.Assemble()
mbs.SolveDynamic()

#angle = M/(2*J_zz)*t^2 at t=1
Jzz = inertia.GetInertia6D()[2]
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Rotation)[2]*2*Jzz #1

exu.Print("example for MarkerNodeRigid completed, test result =", exu.sys['testResult'])

