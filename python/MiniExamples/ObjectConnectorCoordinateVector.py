#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectConnectorCoordinateVector
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

#all coordinates of two nodes, tied by a coordinate vector constraint X1 qB - X0 qA = offset; the
#coordinates INCLUDE the reference values, so qB - qA = [1,0,0] keeps the two points where they are
nA = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,0]))
nB = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0]))
mbs.AddObject(ObjectMassPoint(nodeNumber=nA, physicsMass=1))
mbs.AddObject(ObjectMassPoint(nodeNumber=nB, physicsMass=1))
mA = mbs.AddMarker(MarkerNodeCoordinates(nodeNumber=nA))
mB = mbs.AddMarker(MarkerNodeCoordinates(nodeNumber=nB))
mbs.AddObject(ObjectConnectorCoordinateVector(markerNumbers=[mA, mB], scalingMarker0=np.eye(3),
                                             scalingMarker1=np.eye(3), offset=[1,0,0]))
mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=nA)), loadVector=[2,0,0]))

mbs.Assemble()
mbs.SolveDynamic()

#both masses move together: a = F/(2m) = 1, x = a/2*t^2 at t=1
exu.sys['testResult'] = mbs.GetNodeOutput(nB, exu.OutputVariableType.Displacement)[0] #0.5

exu.Print("example for ObjectConnectorCoordinateVector completed, test result =", exu.sys['testResult'])

