#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class NodePointGround
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

#a ground node: a fixed point that markers and connectors can use, without coordinates
node = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0]))
mbs.AddObject(ObjectMassPoint(nodeNumber=node, mass=1))
mNode = mbs.AddMarker(MarkerNodePosition(nodeNumber=node))
nFixed = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))
mFixed = mbs.AddMarker(MarkerNodePosition(nodeNumber=nFixed))
mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers=[mFixed, mNode], stiffness=[100,100,100],
                                                   offset=[1,0,0]))
mbs.AddLoad(LoadForceVector(markerNumber=mNode, loadVector=[10,0,0]))

mbs.Assemble()
mbs.SolveStatic()

#the spring is stretched by F/k
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Displacement)[0] #0.1

exu.Print("example for NodePointGround completed, test result =", exu.sys['testResult'])

