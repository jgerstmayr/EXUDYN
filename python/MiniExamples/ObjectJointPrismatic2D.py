#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectJointPrismatic2D
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

#a planar rigid body sliding along an axis of the ground: the axis in marker 0, the normal in marker 1
node = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0,0,0]))
body = mbs.AddObject(ObjectRigidBody2D(nodeNumber=node, physicsMass=2, physicsInertia=0.1))
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localPosition=[0,0,0]))
mbs.AddObject(ObjectJointPrismatic2D(markerNumbers=[mGround, mBody], axisMarker0=[1,1,0], normalMarker1=[-1,1,0]))
mbs.AddLoad(LoadForceVector(markerNumber=mBody, loadVector=[2,0,0])) #its part along the axis moves the body

mbs.Assemble()
mbs.SolveDynamic()

#along the 45 degree axis: s = (F/sqrt(2))/(2m)*t^2, so x = s/sqrt(2) = F/(4m) at t=1
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)[0] #0.25

exu.Print("example for ObjectJointPrismatic2D completed, test result =", exu.sys['testResult'])

