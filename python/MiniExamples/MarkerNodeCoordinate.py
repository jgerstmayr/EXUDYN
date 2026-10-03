#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class MarkerNodeCoordinate
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

#one coordinate of a node: a coordinate spring between the ground node and a 1D mass
node = mbs.AddNode(Node1D(referenceCoordinates=[0]))
mbs.AddObject(ObjectMass1D(nodeNumber=node, mass=1))
mCoord = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=0))
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
mbs.AddObject(ObjectConnectorCoordinateSpringDamper(markerNumbers=[mGround, mCoord], stiffness=100))
mbs.AddLoad(LoadCoordinate(markerNumber=mCoord, load=10))

mbs.Assemble()
mbs.SolveStatic()

#the spring is stretched by F/k
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates) #0.1

exu.Print("example for MarkerNodeCoordinate completed, test result =", exu.sys['testResult'])

