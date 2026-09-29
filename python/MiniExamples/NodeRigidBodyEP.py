#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class NodeRigidBodyEP
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

#a rigid body spinning about its z-axis; the velocity coordinates are the time derivatives of the Euler parameters
omega = [0,0,0.5*np.pi]
ep0 = eulerParameters0
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0,0,0]+ep0,
                                   initialVelocities=[0,0,0]+list(AngularVelocity2EulerParameters_t(omega, ep0))))
inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(),
                              physicsInertia=inertia.GetInertia6D()))

mbs.Assemble()
mbs.SolveDynamic()

#the node adds the constraint of the Euler parameters itself; the angle about z after 1 second:
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Rotation)[2] #pi/2, to the accuracy of the time integration

exu.Print("example for NodeRigidBodyEP completed, test result =", exu.sys['testResult'])

