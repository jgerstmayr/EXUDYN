#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectContactSphereTorus
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

#a ball in the groove of a torus, as in a ball bearing: pushed radially into the groove by a spring
inertia = InertiaSphere(mass=0.1, radius=0.01)
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0.1,0,0]+eulerParameters0))
ball = mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(), physicsInertia=inertia.GetInertia6D()))
mRing = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
mBall = mbs.AddMarker(MarkerBodyRigid(bodyNumber=ball, localPosition=[0,0,0]))
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=4, initialCoordinates=[0,0,0,0]))
#groove of an outer ring: torus about z with major radius 0.1 and groove radius 0.011
mbs.AddObject(ObjectContactSphereTorus(markerNumbers=[mBall, mRing], nodeNumber=nData, radiusSphere=0.01,
                                       torusMajorRadius=0.1, torusMinorRadius=0.011, torusAxis=[0,0,1],
                                       contactStiffness=1e6, contactDamping=1e3))
mbs.AddLoad(LoadForceVector(markerNumber=mBall, loadVector=[10,0,0])) #pushes outwards

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
mbs.SolveDynamic(simulationSettings)

#the ball rests in the groove: outwards by the play 0.001 and F/k
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[0] #0.10101

exu.Print("example for ObjectContactSphereTorus completed, test result =", exu.sys['testResult'])

