#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class LoadForceVector
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

#a body-fixed force on a planar rigid body turned by 90 degrees: the local x-direction is global y
node = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0.5,0.2,0.5*np.pi]))
body = mbs.AddObject(ObjectRigidBody2D(nodeNumber=node, mass=2, inertia=0.1))
mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localPosition=[0,0,0]))
mbs.AddLoad(LoadForceVector(markerNumber=mBody, loadVector=[1,0,0], bodyFixed=True))

mbs.Assemble()
mbs.SolveDynamic()

#y = F/(2m)*t^2 at t=1; x stays 0
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Displacement)[1] #0.25

exu.Print("example for LoadForceVector completed, test result =", exu.sys['testResult'])

