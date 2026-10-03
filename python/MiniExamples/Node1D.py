#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class Node1D
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

#one coordinate, here the displacement of a 1D mass, pulled by a constant force
node = mbs.AddNode(Node1D(referenceCoordinates=[0], initialVelocities=[1]))
mbs.AddObject(ObjectMass1D(nodeNumber=node, mass=2))
mCoord = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=0))
mbs.AddLoad(LoadCoordinate(markerNumber=mCoord, load=4))

mbs.Assemble()
mbs.SolveDynamic()

#q = v0*t + F/(2m)*t^2 at t=1
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates) #2 (a scalar for one coordinate)

exu.Print("example for Node1D completed, test result =", exu.sys['testResult'])

