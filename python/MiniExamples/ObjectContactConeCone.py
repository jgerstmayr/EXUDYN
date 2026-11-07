#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectContactConeCone
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

#a truncated cone standing on its larger cap on a fixed truncated cone
inertia = InertiaCylinder(density=1000, length=0.2, outerRadius=0.1, axis=2) #about that of the cone, for the example
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0,0,0.21]+eulerParameters0))
cone = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(), inertia=inertia.GetInertia6D()))
mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=cone)), loadVector=[0,0,-9.81]))
mCone = mbs.AddMarker(MarkerBodyRigid(bodyNumber=cone, localPosition=[0,0,0]))
mFixed = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=12, initialCoordinates=[0]*12))
mbs.AddObject(ObjectContactConeCone(markerNumbers=[mCone, mFixed], nodeNumber=nData, conesRadii0=[0.1,0.05],
                                    conesRadii1=[0.3,0.2], conesLengths=[0.2,0.2], contactStiffness=1e5, contactDamping=1e3))

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
mbs.SolveDynamic(simulationSettings)

#at rest on 3 points of its rim: 0.1 + 0.1 - m*g/(3*k)
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[2]

exu.Print("example for ObjectContactConeCone completed, test result =", exu.sys['testResult'])

