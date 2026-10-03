#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectConnectorHydraulicActuatorSimple
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

#a hydraulic cylinder between the ground and a mass, both valves closed: the oil in the two chambers
#is a spring; chamber pressures p0, p1 (a NodeGenericODE1) balance the weight
m = 100; A = 0.01; p1 = 1e5
p0 = (m*9.81 + p1*A)/A #the pressure that holds the weight
nMass = mbs.AddNode(NodePoint(referenceCoordinates=[0,1,0]))
mbs.AddObject(ObjectMassPoint(nodeNumber=nMass, mass=m))
mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
mbs.AddLoad(LoadForceVector(markerNumber=mMass, loadVector=[0,-m*9.81,0]))
mBase = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0]))
nPressures = mbs.AddNode(NodeGenericODE1(numberOfODE1Coordinates=2, referenceCoordinates=[0,0],
                                         initialCoordinates=[p0,p1]))
oCylinder = mbs.AddObject(ObjectConnectorHydraulicActuatorSimple(markerNumbers=[mBase, mMass],
              nodeNumbers=[nPressures], offsetLength=0.5, strokeLength=1,
              chamberCrossSection0=A, chamberCrossSection1=A, hoseVolume0=1e-3, hoseVolume1=1e-3,
              valveOpening0=0, valveOpening1=0, actuatorDamping=1e4, oilBulkModulus=1e9,
              nominalFlow=1e-4, systemPressure=2e7, tankPressure=0))

mbs.Assemble()
mbs.SolveDynamic()

#the mass stays where it is
exu.sys['testResult'] = mbs.GetObjectOutput(oCylinder, exu.OutputVariableType.Distance) #1

exu.Print("example for ObjectConnectorHydraulicActuatorSimple completed, test result =", exu.sys['testResult'])

