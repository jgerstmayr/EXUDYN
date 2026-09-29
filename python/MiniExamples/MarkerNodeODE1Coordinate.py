#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class MarkerNodeODE1Coordinate
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

#a coordinate of a first-order system: a constant input to q_t = -q + f
node = mbs.AddNode(NodeGenericODE1(numberOfODE1Coordinates=1, referenceCoordinates=[0],
                                   initialCoordinates=[0]))
mbs.AddObject(ObjectGenericODE1(nodeNumbers=[node], systemMatrix=[[-1]]))
mCoord = mbs.AddMarker(MarkerNodeODE1Coordinate(nodeNumber=node, coordinate=0))
mbs.AddLoad(LoadCoordinate(markerNumber=mCoord, load=1))

mbs.Assemble()
mbs.SolveDynamic(solverType=exu.DynamicSolverType.RK44)

#q(1) = 1 - exp(-1)
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates) #0.632

exu.Print("example for MarkerNodeODE1Coordinate completed, test result =", exu.sys['testResult'])

