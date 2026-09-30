#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class MarkerBodyCable2DShape
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
#the shape of an ANCF cable element as line segments, for contact: a cantilever falls onto a circle
cable = ObjectANCFCable2D(physicsMassPerLength=1, physicsBendingStiffness=10, physicsAxialStiffness=1e4,
                          physicsBendingDamping=0.1)
[nodes, elements, *_] = GenerateStraightLineANCFCable2D(mbs, positionOfNode0=[0,0,0], positionOfNode1=[1,0,0],
                        numberOfElements=4, cableTemplate=cable, massProportionalLoad=[0,-9.81,0],
                        fixedConstraintsNode0=[1,1,0,1])
mCircle = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0.8,-0.2,0]))
nSegments = 4
for e in elements:
    mShape = mbs.AddMarker(MarkerBodyCable2DShape(bodyNumber=e, numberOfSegments=nSegments))
    nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=nSegments, initialCoordinates=[0.1]*nSegments))
    mbs.AddObject(ObjectContactCircleCable2D(markerNumbers=[mCircle, mShape], nodeNumber=nData,
                                             numberOfContactSegments=nSegments, circleRadius=0.1,
                                             contactStiffness=1e4))

mbs.Assemble()
mbs.SolveDynamic()

#the tip rests beyond the circle, whose top is at y=-0.1
exu.sys['testResult'] = mbs.GetNodeOutput(nodes[-1], exu.OutputVariableType.Position)[1]

exu.Print("example for MarkerBodyCable2DShape completed, test result =", exu.sys['testResult'])

