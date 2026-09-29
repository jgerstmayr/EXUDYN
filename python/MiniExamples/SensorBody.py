#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class SensorBody
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

#a point of a body given by its local position: a planar rigid body spinning about its center
node = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0,0,0], initialVelocities=[0,0,0.5*np.pi]))
body = mbs.AddObject(ObjectRigidBody2D(nodeNumber=node, physicsMass=1, physicsInertia=0.1))
sPoint = mbs.AddSensor(SensorBody(bodyNumber=body, localPosition=[0.5,0,0],
                                  outputVariableType=exu.OutputVariableType.Position,
                                  storeInternal=True, writeToFile=False))

mbs.Assemble()
mbs.SolveDynamic()

#after a quarter turn the point [0.5,0,0] is at [0,0.5,0]
exu.sys['testResult'] = mbs.GetSensorValues(sPoint)[1] #0.5

exu.Print("example for SensorBody completed, test result =", exu.sys['testResult'])

