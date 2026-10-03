#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectALEANCFCable2D
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
#an axially moving cable: the material slides through clamped nodes, described by one ALE coordinate
nALE = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=1, referenceCoordinates=[0],
                                   initialCoordinates=[0], initialCoordinates_t=[0]))
cable = ObjectALEANCFCable2D(massPerLength=1, bendingStiffness=10, axialStiffness=1e4)
cable.nodeNumbers[2] = nALE #the ALE node of every element
[nodes, elements, *_] = GenerateStraightLineANCFCable2D(mbs, positionOfNode0=[0,0,0], positionOfNode1=[2,0,0],
                        numberOfElements=4, cableTemplate=cable,
                        fixedConstraintsNode0=[1,1,1,1], fixedConstraintsNode1=[1,1,1,1])
mALE = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nALE, coordinate=0))
mbs.AddLoad(LoadCoordinate(markerNumber=mALE, load=1)) #pulls the material along the cable

mbs.Assemble()
mbs.SolveDynamic()

#the material of mass 2 is accelerated by 1 N: s = F/(2m)*t^2 at t=1
exu.sys['testResult'] = mbs.GetNodeOutput(nALE, exu.OutputVariableType.Coordinates) #0.25

exu.Print("example for ObjectALEANCFCable2D completed, test result =", exu.sys['testResult'])

