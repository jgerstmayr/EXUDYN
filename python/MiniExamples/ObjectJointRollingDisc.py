#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectJointRollingDisc
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

#a disc rolling without slip on the ground plane: the constraint of ideal rolling
r = 0.2
inertia = InertiaCylinder(density=1000, length=0.05, outerRadius=r, axis=0)
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0,0,r]+eulerParameters0,
                   initialVelocities=[0,-2,0]+list(AngularVelocity2EulerParameters_t([2/r,0,0], eulerParameters0))))
disc = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(), inertia=inertia.GetInertia6D()))
mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=disc)), loadVector=[0,0,-9.81]))
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
mDisc = mbs.AddMarker(MarkerBodyRigid(bodyNumber=disc, localPosition=[0,0,0]))
mbs.AddObject(ObjectJointRollingDisc(markerNumbers=[mGround, mDisc], discRadius=r, discAxis=[1,0,0],
                                     planeNormal=[0,0,1]))

mbs.Assemble()
mbs.SolveDynamic()

#it rolls on with the initial velocity: y = -2*t at t=1
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[1] #-2

exu.Print("example for ObjectJointRollingDisc completed, test result =", exu.sys['testResult'])

