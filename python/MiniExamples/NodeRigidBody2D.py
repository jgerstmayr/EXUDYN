#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class NodeRigidBody2D
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

#a planar rigid body: x, y and the rotation angle, thrown with a spin
node = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0.5,0.2,0], initialVelocities=[1,0,2]))
mbs.AddObject(ObjectRigidBody2D(nodeNumber=node, mass=2, inertia=0.1))

mbs.Assemble()
mbs.SolveDynamic()

#x = 1*t, angle = 2*t at t=1
exu.sys['testResult'] = sum(mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)) #3

exu.Print("example for NodeRigidBody2D completed, test result =", exu.sys['testResult'])

