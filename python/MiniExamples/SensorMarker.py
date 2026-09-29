#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class SensorMarker
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

#what a marker provides, here the velocity of a point of a body
node = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,0], initialVelocities=[0,2,0]))
body = mbs.AddObject(ObjectMassPoint(nodeNumber=node, physicsMass=1))
mBody = mbs.AddMarker(MarkerBodyPosition(bodyNumber=body, localPosition=[0,0,0]))
mbs.AddLoad(LoadForceVector(markerNumber=mBody, loadVector=[0,-1,0]))
sVelocity = mbs.AddSensor(SensorMarker(markerNumber=mBody, outputVariableType=exu.OutputVariableType.Velocity,
                                       storeInternal=True, writeToFile=False))

mbs.Assemble()
mbs.SolveDynamic()

#v = v0 + F/m*t at t=1
exu.sys['testResult'] = mbs.GetSensorValues(sVelocity)[1] #1

exu.Print("example for SensorMarker completed, test result =", exu.sys['testResult'])

