#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class SensorLoad
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

#the value of a load, here of a load with a user function, which the load vector does not show
node = mbs.AddNode(Node1D(referenceCoordinates=[0]))
mbs.AddObject(ObjectMass1D(nodeNumber=node, physicsMass=1))
mCoord = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=0))
def UFload(mbs, t, load):
    return load*np.cos(np.pi*t)
lCoord = mbs.AddLoad(LoadCoordinate(markerNumber=mCoord, load=2, loadUserFunction=UFload))
sLoad = mbs.AddSensor(SensorLoad(loadNumber=lCoord, storeInternal=True, writeToFile=False))

mbs.Assemble()
mbs.SolveDynamic()

#the load at t=1: 2*cos(pi)
exu.sys['testResult'] = mbs.GetSensorValues(sLoad) #-2

exu.Print("example for SensorLoad completed, test result =", exu.sys['testResult'])

