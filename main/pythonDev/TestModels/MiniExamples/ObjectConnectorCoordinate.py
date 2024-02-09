#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectConnectorCoordinate
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

def OffsetUF(mbs, t, itemNumber, lOffset): #gives 0.05 at t=1
    return 0.5*(1-np.cos(2*3.141592653589793*0.25*t))*lOffset

nMass=mbs.AddNode(Point(referenceCoordinates = [2,0,0]))
massPoint = mbs.AddObject(MassPoint(physicsMass = 5, nodeNumber = nMass))

groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nMass, coordinate = 0))

#Spring-Damper between two marker coordinates
mbs.AddObject(CoordinateConstraint(markerNumbers = [groundMarker, nodeMarker], 
                                   offset = 0.1, offsetUserFunction = OffsetUF)) 

#assemble and solve system for default parameters
mbs.Assemble()
mbs.SolveDynamic()

#check result at default integration time
exudynTestGlobals.testResult  = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Displacement)[0]

exu.Print("example for ObjectConnectorCoordinate completed, test result =", exudynTestGlobals.testResult)

