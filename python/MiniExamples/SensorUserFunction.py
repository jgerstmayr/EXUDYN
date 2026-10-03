#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class SensorUserFunction
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

#a value computed from other sensors: the distance between two mass points
nA = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,0], initialVelocities=[-1,0,0]))
nB = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0], initialVelocities=[0,1,0]))
mbs.AddObject(ObjectMassPoint(nodeNumber=nA, mass=1))
mbs.AddObject(ObjectMassPoint(nodeNumber=nB, mass=1))
sA = mbs.AddSensor(SensorNode(nodeNumber=nA, outputVariableType=exu.OutputVariableType.Position, writeToFile=False))
sB = mbs.AddSensor(SensorNode(nodeNumber=nB, outputVariableType=exu.OutputVariableType.Position, writeToFile=False))
def UFdistance(mbs, t, sensorNumbers, factors, configuration):
    pA = mbs.GetSensorValues(sensorNumbers[0], configuration)
    pB = mbs.GetSensorValues(sensorNumbers[1], configuration)
    return [np.linalg.norm(np.array(pB) - np.array(pA))]
sDistance = mbs.AddSensor(SensorUserFunction(sensorNumbers=[sA, sB], sensorUserFunction=UFdistance,
                                             storeInternal=True, writeToFile=False))

mbs.Assemble()
mbs.SolveDynamic()

#at t=1: pA = [-1,0,0], pB = [1,1,0]
exu.sys['testResult'] = mbs.GetSensorValues(sDistance) #sqrt(5), a scalar for one value

exu.Print("example for SensorUserFunction completed, test result =", exu.sys['testResult'])

