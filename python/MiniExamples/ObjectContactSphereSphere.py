#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectContactSphereSphere
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

#a ball dropped onto a large fixed sphere: penalty contact with its state in a data node
inertia = InertiaSphere(mass=1, radius=0.1)
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0,0,1.2]+eulerParameters0))
ball = mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(), physicsInertia=inertia.GetInertia6D()))
mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=ball)), loadVector=[0,0,-9.81]))
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
mBall = mbs.AddMarker(MarkerBodyRigid(bodyNumber=ball, localPosition=[0,0,0]))
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=4, initialCoordinates=[0,0,0,0]))
mbs.AddObject(ObjectContactSphereSphere(markerNumbers=[mGround, mBall], nodeNumber=nData, spheresRadii=[1, 0.1],
                                        contactStiffness=1e5, contactDamping=1e3))

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
mbs.SolveDynamic(simulationSettings)

#at rest on top: 1.1 - m*g/k
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[2] #1.0999

exu.Print("example for ObjectContactSphereSphere completed, test result =", exu.sys['testResult'])

