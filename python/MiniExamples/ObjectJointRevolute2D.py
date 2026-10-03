#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectJointRevolute2D
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

#a planar rigid body pendulum held at its end by a planar revolute joint
node = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0.5,0,0]))
body = mbs.AddObject(ObjectRigidBody2D(nodeNumber=node, mass=1, inertia=1/12))
mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerBodyPosition(bodyNumber=body)), loadVector=[0,-9.81,0]))
mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0]))
mBody = mbs.AddMarker(MarkerBodyPosition(bodyNumber=body, localPosition=[-0.5,0,0]))
mbs.AddObject(ObjectJointRevolute2D(markerNumbers=[mGround, mBody]))

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
mbs.SolveDynamic(simulationSettings)

#the pendulum falls from horizontal; the angle after 1 second (numerical)
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)[2]

exu.Print("example for ObjectJointRevolute2D completed, test result =", exu.sys['testResult'])

