#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class MarkerUser
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

#a marker at the midpoint of two mass points, held by a spring to the ground
nA = mbs.AddNode(NodePoint(referenceCoordinates=[-0.5,0,0], initialCoordinates=[0,0.1,0]))
nB = mbs.AddNode(NodePoint(referenceCoordinates=[0.5,0,0], initialCoordinates=[0,0.1,0]))
bA = mbs.AddObject(MassPoint(nodeNumber=nA, mass=1))
bB = mbs.AddObject(MassPoint(nodeNumber=nB, mass=1))
def UFposition(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return 0.5*(qTotal[0:3] + qTotal[3:6])
mMid = mbs.AddMarker(MarkerUser(bodyNumbers=[bA, bB], positionUserFunction=UFposition))
mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround))
k = 100
mbs.AddObject(CartesianSpringDamper(markerNumbers=[mGround, mMid], stiffness=[k,k,k]))

mbs.Assemble()
mbs.SolveDynamic()

#the midpoint oscillates with omega = sqrt(k/(2 m)), from 0.1: 0.1*cos(omega) at t=1
exu.sys['testResult'] = mbs.GetMarkerOutput(mMid, exu.OutputVariableType.Position)[1]

exu.Print("example for MarkerUser completed, test result =", exu.sys['testResult'])

