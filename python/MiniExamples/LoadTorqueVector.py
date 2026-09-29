#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class LoadTorqueVector
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

#a torque about z spins a rigid body up
inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0,0,0]+eulerParameters0))
body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(),
                                     physicsInertia=inertia.GetInertia6D()))
mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localPosition=[0,0,0]))
mbs.AddLoad(LoadTorqueVector(markerNumber=mBody, loadVector=[0,0,1]))

mbs.Assemble()
mbs.SolveDynamic()

#angular velocity = M/J_zz*t at t=1
Jzz = inertia.GetInertia6D()[2]
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.AngularVelocity)[2]*Jzz #1, to the accuracy of the time integration

exu.Print("example for LoadTorqueVector completed, test result =", exu.sys['testResult'])

