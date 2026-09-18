#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectMass1D
# 
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
# 
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import sys
sys.path.append('../TestModels')
sys.path.append('../../TestModels') #for direct run in directory

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics

from modelUnitTests import ExudynTestStructure, exudynTestGlobals
import numpy as np

#create an environment for mini example
SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))

node = mbs.AddNode(Node1D(referenceCoordinates = [1], 
                          initialCoordinates=[0.5],
                          initialVelocities=[0.5]))
mass = mbs.AddObject(Mass1D(nodeNumber = node, physicsMass=1))

#assemble and solve system for default parameters
mbs.Assemble()
mbs.SolveDynamic()

#check result, get current mass position at local position [0,0,0]
exudynTestGlobals.testResult = mbs.GetObjectOutputBody(mass, exu.OutputVariableType.Position, [0,0,0])[0]
#final x-coordinate of position shall be 2

exu.Print("example for ObjectMass1D completed, test result =", exudynTestGlobals.testResult)

