#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectConnectorCoordinateSpringDamperExt
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

#a coordinate spring with a limit stop; the stop's state is kept in a data node
node = mbs.AddNode(Node1D(referenceCoordinates=[0]))
mbs.AddObject(ObjectMass1D(nodeNumber=node, mass=1))
mCoord = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=0))
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=3, initialCoordinates=[0,0,0]))
mbs.AddObject(ObjectConnectorCoordinateSpringDamperExt(markerNumbers=[mGround, mCoord], nodeNumber=nData,
              stiffness=100, damping=20, useLimitStops=True, limitStopsLower=-1, limitStopsUpper=0.05,
              limitStopsStiffness=1e4, limitStopsDamping=100))
mbs.AddLoad(LoadCoordinate(markerNumber=mCoord, load=10))

mbs.Assemble()
mbs.SolveDynamic()

#spring and stop share the load: 100*q + 1e4*(q - 0.05) = 10
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates) #0.050495

exu.Print("example for ObjectConnectorCoordinateSpringDamperExt completed, test result =", exu.sys['testResult'])

