#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectContactCylinderCylinder
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

#a short cylinder (a disc) dropped onto a cylinder lying along the x axis, fixed to the ground
inertia = InertiaCylinder(density=1000, length=0.1, outerRadius=0.1, axis=2)
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0.2,0,0.5]+eulerParameters0))
disc = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(), inertia=inertia.GetInertia6D()))
mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=disc)), loadVector=[0,0,-9.81]))
mCylinder = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0], localHT=HTrotateY(0.5*pi)))
mDisc = mbs.AddMarker(MarkerBodyRigid(bodyNumber=disc, localPosition=[0,0,0]))
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=12, initialCoordinates=[0]*12))
mbs.AddObject(ObjectContactCylinderCylinder(markerNumbers=[mDisc, mCylinder], nodeNumber=nData, cylindersRadii=[0.1,0.2],
                                            cylindersLengths=[0.1,1], contactStiffness=1e5, contactDamping=1e3))

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
mbs.SolveDynamic(simulationSettings)

#at rest with its cap on top of the cylinder, on the two ends of the line of contact: 0.2 + 0.05 - m*g/(2*k)
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[2]

exu.Print("example for ObjectContactCylinderCylinder completed, test result =", exu.sys['testResult'])

