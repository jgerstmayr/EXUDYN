#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class MarkerBodyPosition
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

#a point of a body - here of the ground, at a local position - connected to a mass point by a spring
node = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0]))
body = mbs.AddObject(ObjectMassPoint(nodeNumber=node, mass=1))
mBody = mbs.AddMarker(MarkerBodyPosition(bodyNumber=body, localPosition=[0,0,0]))
mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[1,0,0]))
mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers=[mGround, mBody], stiffness=[100,100,100]))
mbs.AddLoad(LoadForceVector(markerNumber=mBody, loadVector=[0,0,-10]))

mbs.Assemble()
mbs.SolveStatic()

#the spring is stretched by F/k
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Displacement)[2] #-0.1

exu.Print("example for MarkerBodyPosition completed, test result =", exu.sys['testResult'])

