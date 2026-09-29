#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class NodeGenericODE1
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

#a first-order system q_t = A q, here an exponential decay
node = mbs.AddNode(NodeGenericODE1(numberOfODE1Coordinates=1, referenceCoordinates=[0],
                                   initialCoordinates=[1]))
mbs.AddObject(ObjectGenericODE1(nodeNumbers=[node], systemMatrix=[[-1]]))

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.RK44)

#q(1) = exp(-1)
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates) #0.3679

exu.Print("example for NodeGenericODE1 completed, test result =", exu.sys['testResult'])

