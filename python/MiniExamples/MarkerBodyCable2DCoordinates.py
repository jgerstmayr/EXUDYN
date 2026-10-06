#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class MarkerBodyCable2DCoordinates
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
#the coordinates of ANCF cable elements for a sliding joint: a mass point slides along a clamped, stiff cable
cable = ObjectANCFCable2D(massPerLength=1, bendingStiffness=1e4, axialStiffness=1e6)
beamInfo = GenerateBeamElementsAlongLine(mbs, positionStart=[0,0,0], positionEnd=[2,0,0],
                        numberOfElements=4, beamTemplate=cable,
                        groundConstraintsStart=[1,1,1,1], groundConstraintsEnd=[1,1,1,1])
nodes, elements = beamInfo['nodes'], beamInfo['elements']
nMass = mbs.AddNode(NodePoint2D(referenceCoordinates=[0.6,0]))
mbs.AddObject(ObjectMassPoint2D(nodeNumber=nMass, mass=1))
mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
mbs.AddLoad(LoadForceVector(markerNumber=mMass, loadVector=[1,0,0]))

cableMarkers = [mbs.AddMarker(MarkerBodyCable2DCoordinates(bodyNumber=e)) for e in elements]
offsets = [0.5*i for i in range(4)] #the element length is 0.5
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=2, initialCoordinates=[1, 0.6])) #element 1, sliding coordinate
mbs.AddObject(ObjectJointSliding2D(markerNumbers=[mMass, cableMarkers[1]], slidingMarkerNumbers=cableMarkers,
                                   slidingMarkerOffsets=offsets, nodeNumber=nData))

mbs.Assemble()
mbs.SolveDynamic()

#the mass slides: x = 0.6 + F/(2m)*t^2 at t=1, the stiff cable deflects little
exu.sys['testResult'] = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Position)[0] #1.1

exu.Print("example for MarkerBodyCable2DCoordinates completed, test result =", exu.sys['testResult'])

