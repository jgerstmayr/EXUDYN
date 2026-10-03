#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class NodePoint2DSlope1
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

#a cantilever of one ANCF cable element: position and slope (r_x) at each node
L = 1; EI = 100; F = -0.1
n0 = mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[0,0, 1,0])) #position, slope = axis
n1 = mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[L,0, 1,0]))
mbs.AddObject(ObjectANCFCable2D(nodeNumbers=[n0,n1], length=L, massPerLength=1,
                                bendingStiffness=EI, axialStiffness=1e5))
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
for i in [0,1,3]: #clamped: x, y and the y-component of the slope
    mCoord = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=i))
    mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mGround, mCoord]))
mTip = mbs.AddMarker(MarkerNodePosition(nodeNumber=n1))
mbs.AddLoad(LoadForceVector(markerNumber=mTip, loadVector=[0,F,0]))

mbs.Assemble()
mbs.SolveStatic()

#the cubic element is exact for a tip load: F*L^3/(3*EI) = -1/3000
exu.sys['testResult'] = mbs.GetNodeOutput(n1, exu.OutputVariableType.Displacement)[1]*1000 #-1/3

exu.Print("example for NodePoint2DSlope1 completed, test result =", exu.sys['testResult'])

