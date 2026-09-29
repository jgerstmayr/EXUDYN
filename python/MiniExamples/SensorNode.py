#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class SensorNode
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

#the position of a node, stored during the simulation
node = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,0], initialVelocities=[1,0,0]))
mbs.AddObject(ObjectMassPoint(nodeNumber=node, physicsMass=1))
sNode = mbs.AddSensor(SensorNode(nodeNumber=node, outputVariableType=exu.OutputVariableType.Position,
                                 storeInternal=True, writeToFile=False))

mbs.Assemble()
mbs.SolveDynamic()

#rows [t, x, y, z]; the last row at t=1
data = mbs.GetSensorStoredData(sNode)
exu.sys['testResult'] = data[-1,0] + data[-1,1] #1+1

exu.Print("example for SensorNode completed, test result =", exu.sys['testResult'])

