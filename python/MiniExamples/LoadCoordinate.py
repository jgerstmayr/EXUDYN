#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class LoadCoordinate
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

#a load on one coordinate, growing in time through its user function
node = mbs.AddNode(Node1D(referenceCoordinates=[0]))
mbs.AddObject(ObjectMass1D(nodeNumber=node, physicsMass=1))
mCoord = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=0))
def UFload(mbs, t, load):
    return load*t
mbs.AddLoad(LoadCoordinate(markerNumber=mCoord, load=1, loadUserFunction=UFload))

mbs.Assemble()
mbs.SolveDynamic()

#q = t^3/6 at t=1
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates) #0.1667

exu.Print("example for LoadCoordinate completed, test result =", exu.sys['testResult'])

