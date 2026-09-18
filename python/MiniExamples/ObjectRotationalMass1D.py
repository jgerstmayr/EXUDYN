#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectRotationalMass1D
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

node = mbs.AddNode(Node1D(referenceCoordinates = [1], #\psi_0ref
                          initialCoordinates=[0.5],   #\psi_0ini
                          initialVelocities=[0.5]))   #\psi_t0ini
rotor = mbs.AddObject(Rotor1D(nodeNumber = node, physicsInertia=1))

#assemble and solve system for default parameters
mbs.Assemble()
mbs.SolveDynamic()

#check result, get current rotor z-rotation at local position [0,0,0]
exudynTestGlobals.testResult = mbs.GetObjectOutputBody(rotor, exu.OutputVariableType.Rotation, [0,0,0])
#final z-angle of rotor shall be 2

exu.Print("example for ObjectRotationalMass1D completed, test result =", exudynTestGlobals.testResult)

