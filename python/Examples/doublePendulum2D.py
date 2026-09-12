#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Simple double pendulum
#
# Author:   Johannes Gerstmayr (with "help" of Bing AI)
# Date:     2023-05-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
from math import sin, cos, pi

SC = exu.SystemContainer()
mbs = SC.AddSystem()

# create nodes:
n0=mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0,0,0], initialVelocities=[0,0,0]))
n1=mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[1,0,0], initialVelocities=[0,0,0]))

# create bodies:
b0=mbs.AddObject(RigidBody2D(physicsMass=1, physicsInertia=1,nodeNumber=n0,
       visualization=VRigidBody2D(graphicsData=[graphics.Lines([[-0.5,0,0],[0.5,0,0]])])))
b1=mbs.AddObject(RigidBody2D(physicsMass=1, physicsInertia=1,nodeNumber=n1,
                 visualization=VRigidBody2D(graphicsData=[graphics.Lines([[-0.5,0,0],[0.5,0,0]])])))

# add markers and loads:
oGround = mbs.AddObject(ObjectGround())
mGround=mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[-0.5, 0., 0.]))
m00 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[-0.5, 0., 0.]))
m01 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[ 0.5, 0., 0.]))
m10 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=[-0.5, 0., 0.]))
#m12 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=[ 0.5, 0., 0.]))

# add joints:
jg0 = mbs.AddObject(RevoluteJoint2D(markerNumbers=[mGround,m00]))
j01 = mbs.AddObject(RevoluteJoint2D(markerNumbers=[m01,m10]))
 
# add loads:

mLoad0=mbs.AddMarker(MarkerNodePosition(nodeNumber=n0))
mLoad1=mbs.AddMarker(MarkerNodePosition(nodeNumber=n1))

mbs.AddLoad(Force(markerNumber=mLoad0, loadVector=[0,-9.81,0]))
mbs.AddLoad(Force(markerNumber=mLoad1, loadVector=[0,-9.81,0]))


# add time integration scheme:
mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 10000
simulationSettings.timeIntegration.endTime = 10

#remove this if you want to simulate fast:
simulationSettings.timeIntegration.simulateInRealtime = True #otherwise, nothing is visible

simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.solutionWritePeriod = 0.01

simulationSettings.timeIntegration.verboseMode = 1
# simulationSettings.displayStatistics = True
# simulationSettings.displayComputationTime = True

#simulate:
SC.renderer.Start()
SC.renderer.DoIdleTasks()
mbs.SolveDynamic(simulationSettings)
SC.renderer.Stop()