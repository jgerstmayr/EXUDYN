#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class MarkerObjectODE2Coordinates
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

#all coordinates of an object, here of a ObjectGenericODE2 with two free coordinates, tied by a
#coordinate vector constraint X1 q - X0 q_ground = offset, which is q0 - q1 = 0
node = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=2, referenceCoordinates=[0,0],
                                   initialCoordinates=[0,0], initialCoordinates_t=[0,0]))
oGeneric = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node], massMatrix=np.eye(2)))
mAll = mbs.AddMarker(MarkerObjectODE2Coordinates(objectNumber=oGeneric))
mNone = mbs.AddMarker(MarkerNodeCoordinates(nodeNumber=nGround)) #the ground node has no coordinates
mbs.AddObject(ObjectConnectorCoordinateVector(markerNumbers=[mNone, mAll], scalingMarker0=np.zeros((1,0)),
                                             scalingMarker1=[[1,-1]], offset=[0]))
mbs.AddLoad(LoadCoordinate(markerNumber=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=0)), load=2))

mbs.Assemble()
mbs.SolveDynamic()

#both coordinates move together: a = F/2 = 1, q1 = a/2*t^2 at t=1
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)[1] #0.5

exu.Print("example for MarkerObjectODE2Coordinates completed, test result =", exu.sys['testResult'])

