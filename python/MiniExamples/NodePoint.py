#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class NodePoint
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

#a point mass moving freely: reference position, initial displacement and initial velocity
node = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0],
                             initialCoordinates=[0,0.5,0],   #displacement from the reference
                             initialVelocities=[2,0,0]))
mbs.AddObject(ObjectMassPoint(nodeNumber=node, mass=1))

mbs.Assemble()
mbs.SolveDynamic() #default: 1 second

#position = reference + displacement: [1+0+2*1, 0.5, 0]
exu.sys['testResult'] = sum(mbs.GetNodeOutput(node, exu.OutputVariableType.Position)) #3.5

exu.Print("example for NodePoint completed, test result =", exu.sys['testResult'])

