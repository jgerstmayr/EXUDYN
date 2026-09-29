#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectConnectorReevingSystemSprings
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

#a rope from a fixed point over no sheave to a hanging body: the rope as one spring along its length
inertia = InertiaCuboid(density=1000, sideLengths=[0.1,0.1,0.1])
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0,-1,0]+eulerParameters0))
body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(), physicsInertia=inertia.GetInertia6D()))
mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=body)), loadVector=[0,-9.81,0]))
mTop = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localPosition=[0,0,0]))
EA = 1e4
mbs.AddObject(ObjectConnectorReevingSystemSprings(markerNumbers=[mTop, mBody], stiffnessPerLength=EA,
              dampingPerLength=100, referenceLength=1, sheavesAxes=exu.Vector3DList([[0,0,1],[0,0,1]]),
              sheavesRadii=[0,0]))

mbs.Assemble()
mbs.SolveDynamic()

#the rope is stretched by m*g*L/EA (damped to rest)
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Displacement)[1]/(inertia.Mass()*9.81/EA) #-1

exu.Print("example for ObjectConnectorReevingSystemSprings completed, test result =", exu.sys['testResult'])

