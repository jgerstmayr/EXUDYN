#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectRigidBody
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

#a rigid body thrown with a spin about a principal axis, under gravity
inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0.5,0.2,0]+eulerParameters0,
                                   initialVelocities=[0,0,5]+list(AngularVelocity2EulerParameters_t([0,0,1], eulerParameters0))))
body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(),
                                     physicsInertia=inertia.GetInertia6D()))
mMass = mbs.AddMarker(MarkerBodyMass(bodyNumber=body))
mbs.AddLoad(LoadMassProportional(markerNumber=mMass, loadVector=[0,0,-9.81]))

mbs.Assemble()
mbs.SolveDynamic()

#z = v0*t - g/2*t^2, and the angle about z is omega*t, at t=1
p = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)
angle = mbs.GetNodeOutput(node, exu.OutputVariableType.Rotation)[2]
exu.sys['testResult'] = p[2] + angle #0.095 + 1

exu.Print("example for ObjectRigidBody completed, test result =", exu.sys['testResult'])

