#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class MarkerNodePosition
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

#the position of a node: a mass hanging on a spring from a ground node, released at rest
nMass = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,-1]))
mbs.AddObject(ObjectMassPoint(nodeNumber=nMass, mass=1))
mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
mFixed = mbs.AddMarker(MarkerNodePosition(nodeNumber=nGround))
k = (2*np.pi)**2 #1 Hz
mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[mFixed, mMass], stiffness=k, referenceLength=1))
mbs.AddLoad(LoadForceVector(markerNumber=mMass, loadVector=[0,0,-9.81]))

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.endTime = 0.5 #half a period
mbs.SolveDynamic(simulationSettings)

#lowest point: twice the static deflection, -2*g/k
exu.sys['testResult'] = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Displacement)[2] #-0.497

exu.Print("example for MarkerNodePosition completed, test result =", exu.sys['testResult'])

