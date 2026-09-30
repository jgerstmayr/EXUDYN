#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class NodeRigidBodyRxyz
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

#a rigid body spinning about its z-axis; the rotation coordinates are Tait-Bryan angles
node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0.5,0.2,0.1, 0,0,0],
                                     initialVelocities=[0,0,0, 0,0,0.5*np.pi])) #angle rates
inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(),
                              physicsInertia=inertia.GetInertia6D()))

mbs.Assemble()
mbs.SolveDynamic()

#the third rotation coordinate after 1 second:
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)[5] #pi/2

exu.Print("example for NodeRigidBodyRxyz completed, test result =", exu.sys['testResult'])

