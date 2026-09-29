#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class MarkerBodyMass
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

#gravity on a planar rigid body: the load acts on the mass of the whole body
node = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0,0,0]))
body = mbs.AddObject(ObjectRigidBody2D(nodeNumber=node, physicsMass=2, physicsInertia=0.1))
mMass = mbs.AddMarker(MarkerBodyMass(bodyNumber=body))
mbs.AddLoad(LoadMassProportional(markerNumber=mMass, loadVector=[0,-9.81,0]))

mbs.Assemble()
mbs.SolveDynamic()

#free fall: y = -g/2*t^2 at t=1, independent of the mass
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[1] #-4.905

exu.Print("example for MarkerBodyMass completed, test result =", exu.sys['testResult'])

