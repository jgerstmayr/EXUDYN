#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectConnectorCartesianSpringDamper
# 
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
# 
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import sys
sys.path.append('../TestModels')
sys.path.append('../../TestModels') #for direct run in directory

import exudyn as exu
from exudyn.utilities import *

from modelUnitTests import ExudynTestStructure, exudynTestGlobals
import numpy as np

#create an environment for mini example
SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))

#example with mass at [1,1,0], 5kg under load 5N in -y direction
k=5000
nMass = mbs.AddNode(NodePoint(referenceCoordinates=[1,1,0]))
oMass = mbs.AddObject(MassPoint(physicsMass = 5, nodeNumber = nMass))

mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [1,1,0]))
mbs.AddObject(CartesianSpringDamper(markerNumbers = [mGround, mMass], 
                                    stiffness = [k,k,k], 
                                    damping = [0,k*0.05,0], offset = [0,0,0]))
mbs.AddLoad(Force(markerNumber = mMass, loadVector = [0, -5, 0])) #static solution=-5/5000=-0.001m

#assemble and solve system for default parameters
mbs.Assemble()
mbs.SolveDynamic()

#check result at default integration time
exudynTestGlobals.testResult = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Displacement)[1]

exu.Print("example for ObjectConnectorCartesianSpringDamper completed, test result =", exudynTestGlobals.testResult)

