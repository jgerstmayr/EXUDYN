#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectJointPrismaticX
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

#a body that may only slide along the x-axis of the joint frame, here turned to the global y-axis
inertia = InertiaCuboid(density=1000, sideLengths=[0.1,0.1,0.1])
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0,0,0]+eulerParameters0))
body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(), inertia=inertia.GetInertia6D()))
HTjoint = exu.HT().SetRotationZ(0.5*np.pi) #the joint x-axis is the global y-axis
mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localHT=HTjoint))
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localHT=HTjoint))
mbs.AddObject(ObjectJointPrismaticX(markerNumbers=[mGround, mBody]))
mbs.AddLoad(LoadForceVector(markerNumber=mBody, loadVector=[1,1,1])) #only the y-part moves the body

mbs.Assemble()
mbs.SolveDynamic()

#y = F_y/(2m)*t^2 at t=1, x and z stay 0
exu.sys['testResult'] = sum(mbs.GetNodeOutput(node, exu.OutputVariableType.Displacement))*2*inertia.Mass() #1

exu.Print("example for ObjectJointPrismaticX completed, test result =", exu.sys['testResult'])

