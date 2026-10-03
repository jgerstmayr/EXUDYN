#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectGround
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

#a ground object at a reference position: a fixed point for markers and connectors
oFixed = mbs.AddObject(ObjectGround(referencePosition=[0,2,0]))
mFixed = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oFixed, localPosition=[0,0,0]))
node = mbs.AddNode(NodePoint(referenceCoordinates=[0,1,0]))
mbs.AddObject(ObjectMassPoint(nodeNumber=node, mass=1))
mNode = mbs.AddMarker(MarkerNodePosition(nodeNumber=node))
mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers=[mFixed, mNode], stiffness=[100,100,100],
                                                   offset=[0,-1,0]))
mbs.AddLoad(LoadForceVector(markerNumber=mNode, loadVector=[0,-9.81,0]))

mbs.Assemble()
mbs.SolveStatic()

#the mass hangs 1 below the ground point, lowered by m*g/k
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Displacement)[1] #-0.0981

exu.Print("example for ObjectGround completed, test result =", exu.sys['testResult'])

