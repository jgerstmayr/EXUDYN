#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectJointGeneric
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

#a joint whose constrained axes are chosen: here all but the rotation about z - a revolute joint -
#holding a rigid body pendulum at its end
inertia = InertiaCuboid(density=1000, sideLengths=[1,0.1,0.1])
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0.5,0,0]+eulerParameters0))
body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(), physicsInertia=inertia.GetInertia6D()))
mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=body)), loadVector=[0,-9.81,0]))
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localPosition=[-0.5,0,0]))
mbs.AddObject(ObjectJointGeneric(markerNumbers=[mGround, mBody], constrainedAxes=[1,1,1, 1,1,0]))

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
mbs.SolveDynamic(simulationSettings)

#the pendulum falls from horizontal; the angle after 1 second (numerical)
exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Rotation)[2]

exu.Print("example for ObjectJointGeneric completed, test result =", exu.sys['testResult'])

