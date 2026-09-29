#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class SensorSuperElement
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

#a mesh node of a super element, here of a ObjectGenericODE2 of two free mass points
n0 = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,0]))
n1 = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0]))
oSuper = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[n0,n1], massMatrix=np.eye(6)))
mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=n1)), loadVector=[1,0,0]))
sMesh = mbs.AddSensor(SensorSuperElement(bodyNumber=oSuper, meshNodeNumber=1,
                                         outputVariableType=exu.OutputVariableType.Displacement,
                                         storeInternal=True, writeToFile=False))

mbs.Assemble()
mbs.SolveDynamic()

#mesh node 1: x = F/(2m)*t^2 at t=1
exu.sys['testResult'] = mbs.GetSensorValues(sMesh)[0] #0.5

exu.Print("example for SensorSuperElement completed, test result =", exu.sys['testResult'])

