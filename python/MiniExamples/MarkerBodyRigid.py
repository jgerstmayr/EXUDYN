#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class MarkerBodyRigid
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

#position and orientation of a rigid body: a torque on it, held by a rigid body spring-damper
inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[1,0,0]+eulerParameters0))
body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(),
                                     physicsInertia=inertia.GetInertia6D()))
mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localPosition=[0,0,0]))
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[1,0,0]))
mbs.AddObject(ObjectConnectorRigidBodySpringDamper(markerNumbers=[mGround, mBody],
                                                   stiffness=np.diag([1e4,1e4,1e4, 100,100,100]),
                                                   damping=np.zeros((6,6))))
mbs.AddLoad(LoadTorqueVector(markerNumber=mBody, loadVector=[0,0,1]))

mbs.Assemble()
mbs.SolveStatic()

#rotation about z: M/k_rot, for the small angle
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Rotation)[2] #0.01

exu.Print("example for MarkerBodyRigid completed, test result =", exu.sys['testResult'])

