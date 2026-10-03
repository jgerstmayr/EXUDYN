#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class NodeGenericData
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

#data coordinates hold a state that is no degree of freedom and that the object updates after each step:
#here the limit stop of a connector, which a mass is pushed against
node = mbs.AddNode(Node1D(referenceCoordinates=[0]))
mbs.AddObject(ObjectMass1D(nodeNumber=node, mass=1))
mMass = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=0))
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=3, initialCoordinates=[0,0,0]))
mbs.AddObject(ObjectConnectorCoordinateSpringDamperExt(markerNumbers=[mGround, mMass], nodeNumber=nData,
              damping=20, useLimitStops=True, limitStopsLower=-1, limitStopsUpper=0.05,
              limitStopsStiffness=1e4, limitStopsDamping=100))
mbs.AddLoad(LoadCoordinate(markerNumber=mMass, load=10))

mbs.Assemble()
mbs.SolveDynamic()

#the mass rests at the stop, pressed into it by F/k_limits
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates) #0.05+0.001

exu.Print("example for NodeGenericData completed, test result =", exu.sys['testResult'])

