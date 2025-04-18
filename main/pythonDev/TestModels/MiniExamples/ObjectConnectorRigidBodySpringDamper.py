#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectConnectorRigidBodySpringDamper
# 
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
# 
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import sys
sys.path.append('../TestModels')
sys.path.append('../../TestModels') #for direct run in directory

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics

from modelUnitTests import ExudynTestStructure, exudynTestGlobals
import numpy as np

#create an environment for mini example
SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))

#example with rigid body at [0,0,0], 1kg under initial velocity
k=500
nBody = mbs.AddNode(RigidRxyz(initialVelocities=[0,1e3,0, 0,0,0]))
oBody = mbs.AddObject(RigidBody(physicsMass=1, physicsInertia=[1,1,1,0,0,0], 
                                nodeNumber=nBody))

mBody = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nBody))
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                        localPosition = [0,0,0]))
mbs.AddObject(RigidBodySpringDamper(markerNumbers = [mGround, mBody], 
                                    stiffness = np.diag([k,k,k, 0,0,0]), 
                                    damping = np.diag([0,k*0.01,0, 0,0,0]), 
                                    offset = [0,0,0, 0,0,0]))

#assemble and solve system for default parameters
mbs.Assemble()
mbs.SolveDynamic(exu.SimulationSettings())

#check result at default integration time
exudynTestGlobals.testResult = mbs.GetNodeOutput(nBody, exu.OutputVariableType.Displacement)[1] 

exu.Print("example for ObjectConnectorRigidBodySpringDamper completed, test result =", exudynTestGlobals.testResult)

