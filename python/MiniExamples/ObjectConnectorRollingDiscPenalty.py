#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectConnectorRollingDiscPenalty
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

#a disc rolling on the ground plane with penalty contact and friction
r = 0.2
inertia = InertiaCylinder(density=1000, length=0.05, outerRadius=r, axis=0)
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0,0,r]+eulerParameters0,
                   initialVelocities=[0,-2,0]+list(AngularVelocity2EulerParameters_t([2/r,0,0], eulerParameters0))))
disc = mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(), physicsInertia=inertia.GetInertia6D()))
mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=disc)), loadVector=[0,0,-9.81]))
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
mDisc = mbs.AddMarker(MarkerBodyRigid(bodyNumber=disc, localPosition=[0,0,0]))
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=3, initialCoordinates=[0,0,0]))
mbs.AddObject(ObjectConnectorRollingDiscPenalty(markerNumbers=[mGround, mDisc], nodeNumber=nData, discRadius=r,
                                                discAxis=[1,0,0], planeNormal=[0,0,1], dryFriction=[0.5,0.5],
                                                dryFrictionProportionalZone=1e-2, contactStiffness=1e5, contactDamping=1e3))

mbs.Assemble()
mbs.SolveDynamic()

#it rolls on with the initial velocity: y = -2*t at t=1
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[1] #-2

exu.Print("example for ObjectConnectorRollingDiscPenalty completed, test result =", exu.sys['testResult'])

