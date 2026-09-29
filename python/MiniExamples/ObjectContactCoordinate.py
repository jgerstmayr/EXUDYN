#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectContactCoordinate
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

#a coordinate that contacts a stop: a 1D mass falls onto the ground coordinate (gap = q1 - q0 - offset)
node = mbs.AddNode(Node1D(referenceCoordinates=[0], initialCoordinates=[0.1]))
mbs.AddObject(ObjectMass1D(nodeNumber=node, physicsMass=1))
mCoord = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=0))
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=1, initialCoordinates=[0.1])) #the gap
mbs.AddObject(ObjectContactCoordinate(markerNumbers=[mGround, mCoord], nodeNumber=nData,
                                      contactStiffness=1e4, contactDamping=100))
mbs.AddLoad(LoadCoordinate(markerNumber=mCoord, load=-10))

mbs.Assemble()
mbs.SolveDynamic()

#at rest on the stop, pressed in by F/k
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates) #-0.001

exu.Print("example for ObjectContactCoordinate completed, test result =", exu.sys['testResult'])

