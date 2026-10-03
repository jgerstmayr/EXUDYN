#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectJointALEMoving2D
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

from exudyn.beams import GenerateStraightLineANCFCable2D
#a mass point carried by the material of an axially moving cable
nALE = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=1, referenceCoordinates=[0],
                                   initialCoordinates=[0], initialCoordinates_t=[0]))
cable = ObjectALEANCFCable2D(massPerLength=1, bendingStiffness=10, axialStiffness=1e4)
cable.nodeNumbers[2] = nALE
[nodes, elements, *_] = GenerateStraightLineANCFCable2D(mbs, positionOfNode0=[0,0,0], positionOfNode1=[2,0,0],
                        numberOfElements=4, cableTemplate=cable,
                        fixedConstraintsNode0=[1,1,1,1], fixedConstraintsNode1=[1,1,1,1])
mbs.AddLoad(LoadCoordinate(markerNumber=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nALE, coordinate=0)), load=1))

nMass = mbs.AddNode(NodePoint2D(referenceCoordinates=[0.6,0]))
mbs.AddObject(ObjectMassPoint2D(nodeNumber=nMass, mass=2))
cableMarkers = [mbs.AddMarker(MarkerBodyCable2DCoordinates(bodyNumber=e)) for e in elements]
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=1, initialCoordinates=[1])) #element 1
mbs.AddObject(ObjectJointALEMoving2D(markerNumbers=[mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass)), cableMarkers[1]],
                                     slidingMarkerNumbers=cableMarkers, slidingMarkerOffsets=[0.5*i for i in range(4)],
                                     slidingOffset=0.6, nodeNumbers=[nData, nALE]))

mbs.Assemble()
mbs.SolveDynamic()

#cable material (mass 2) and mass point (2) move together: x = 0.6 + F/(2*4)*t^2 at t=1
exu.sys['testResult'] = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Position)[0] #0.725

exu.Print("example for ObjectJointALEMoving2D completed, test result =", exu.sys['testResult'])

