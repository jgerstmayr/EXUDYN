#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectContactSphereCylinder
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

#a ball dropped onto a cylinder lying along the x axis, fixed to the ground
inertia = InertiaSphere(mass=1, radius=0.1)
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0.2,0,0.5]+eulerParameters0))
ball = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(), inertia=inertia.GetInertia6D()))
mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=ball)), loadVector=[0,0,-9.81]))
mCylinder = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0], localHT=HTrotateY(0.5*pi)))
mBall = mbs.AddMarker(MarkerBodyRigid(bodyNumber=ball, localPosition=[0,0,0]))
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=3, initialCoordinates=[0,0,0]))
mbs.AddObject(ObjectContactSphereCylinder(markerNumbers=[mBall, mCylinder], nodeNumber=nData, sphereRadius=0.1,
                                          cylinderRadius=0.2, cylinderLength=1, contactStiffness=1e5, contactDamping=1e3))

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
mbs.SolveDynamic(simulationSettings)

#at rest on top of the cylinder: 0.2 + 0.1 - m*g/k
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[2] #0.2999

exu.Print("example for ObjectContactSphereCylinder completed, test result =", exu.sys['testResult'])

