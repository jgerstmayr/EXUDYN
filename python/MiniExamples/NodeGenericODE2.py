#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class NodeGenericODE2
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

#two coordinates of a user-defined second-order system: a mass on a spring and a free mass
node = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=2, referenceCoordinates=[0,0],
                                   initialCoordinates=[0.1,0], initialCoordinates_t=[0,1]))
M = np.diag([1,1])
K = np.diag([(2*np.pi)**2, 0]) #eigenfrequency 1 Hz for the first coordinate
mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node], massMatrix=M, stiffnessMatrix=K))

mbs.Assemble()
mbs.SolveDynamic()

#after one period, q0 = 0.1 again; q1 = 1*t
exu.sys['testResult'] = sum(mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)) #1.1

exu.Print("example for NodeGenericODE2 completed, test result =", exu.sys['testResult'])

