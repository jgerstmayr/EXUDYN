#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class NodePoint2D
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

#a planar point mass under gravity, thrown with an initial velocity
node = mbs.AddNode(NodePoint2D(referenceCoordinates=[0,0], initialVelocities=[1,2]))
oMass = mbs.AddObject(ObjectMassPoint2D(nodeNumber=node, mass=1))
mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=node))
mbs.AddLoad(LoadForceVector(markerNumber=mMass, loadVector=[0,-9.81,0]))

mbs.Assemble()
mbs.SolveDynamic()

#y = v0*t - g/2*t^2 at t=1
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[1] #2-4.905=-2.905

exu.Print("example for NodePoint2D completed, test result =", exu.sys['testResult'])

