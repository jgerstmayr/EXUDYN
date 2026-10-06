#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectContactFrictionCircleCable2D
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

from exudyn.beams import GenerateBeamElementsAlongLine
#contact with friction between a circle and an ANCF cable: a cantilever falls onto a circle
cable = ObjectANCFCable2D(massPerLength=1, bendingStiffness=10, axialStiffness=1e4,
                          bendingDamping=0.1)
beamInfo = GenerateBeamElementsAlongLine(mbs, positionStart=[0,0,0], positionEnd=[1,0,0],
                        numberOfElements=4, beamTemplate=cable, gravity=[0,-9.81,0],
                        groundConstraintsStart=[1,1,0,1])
nodes, elements = beamInfo['nodes'], beamInfo['elements']
mCircle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0.8,-0.2,0]))
nSegments = 4
for e in elements:
    mShape = mbs.AddMarker(MarkerBodyCable2DShape(bodyNumber=e, numberOfSegments=nSegments))
    nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=3*nSegments,
                                    initialCoordinates=[0.1]*nSegments+[0]*2*nSegments)) #gaps, friction states
    mbs.AddObject(ObjectContactFrictionCircleCable2D(markerNumbers=[mCircle, mShape], nodeNumber=nData,
                                             numberOfContactSegments=nSegments, circleRadius=0.1,
                                             contactStiffness=1e4, contactDamping=10,
                                             frictionVelocityPenalty=100, frictionCoefficient=0.5))

mbs.Assemble()
mbs.SolveDynamic()

#the tip rests beyond the circle, whose top is at y=-0.1
exu.sys['testResult'] = mbs.GetNodeOutput(nodes[-1], exu.OutputVariableType.Position)[1]

exu.Print("example for ObjectContactFrictionCircleCable2D completed, test result =", exu.sys['testResult'])

