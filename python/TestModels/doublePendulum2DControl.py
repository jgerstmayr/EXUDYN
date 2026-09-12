#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Double pendulum with added torques for control; 
#           This example shows jacobian settings for considering dependencies of controlled loads;
#           This way leads to efficient and stable integration of feedback-controlled bodies
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

mbs.AddLoad(Force(markerNumber=mLoad0, loadVector=[0,-9.81*1,0]))
mbs.AddLoad(Force(markerNumber=mLoad1, loadVector=[0,-9.81*1,0]))

#%%++++++++++++++++++++++++++++++++++++++++++
#add controller using loads
nGround = mbs.AddNode(NodePointGround(visualization=VNodePointGround(show=False)) )
mCGround= mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
mRotNode0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=2))
mRotNode1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n1, coordinate=2))

#load user function, first joint:
def UFtorque1(mbs, t, load):
    omega = 0.5
    phi0 = mbs.GetMarkerOutput(mRotNode0, exu.OutputVariableType.CoordinatesTotal)[0]
    deltaPhi = mbs.GetMarkerOutput(mRotNode1, exu.OutputVariableType.CoordinatesTotal)[0] - phi0

    phi0_t = mbs.GetMarkerOutput(mRotNode0, exu.OutputVariableType.Coordinates_t)[0]
    deltaPhi_t = mbs.GetMarkerOutput(mRotNode1, exu.OutputVariableType.Coordinates_t)[0] - phi0_t

    phiDesired = -pi*0.5*sin(omega*pi*t)
    phiDesired_t = -pi*0.5*omega*cos(omega*pi*t)

    k = 5000
    d = 0.02*k
    torque = k*(phiDesired-deltaPhi) + d*(phiDesired_t-deltaPhi_t)
    return torque

#load user function, second joint:
def UFtorque0(mbs, t, load):
    phi0 = mbs.GetMarkerOutput(mRotNode0, exu.OutputVariableType.Coordinates)[0]
    omega = 0.5
    phiDesired = pi*0.5*sin(omega*pi*t)
    phi0_t = mbs.GetMarkerOutput(mRotNode0, exu.OutputVariableType.Coordinates_t)[0]
    phiDesired_t = pi*0.5*omega*cos(omega*pi*t)
    k = 5000*100 #500000 only works well with load jacobian and dependencies (or doSystemWideDifferentiation)
    d = 0.02*k
    torque = k*(phiDesired-phi0) + d*(phiDesired_t-phi0_t)
    
    return torque - UFtorque1(mbs,t,0) #torque1 also acting on body0

lTorque0 = mbs.AddLoad(LoadCoordinate(markerNumber = mRotNode0,
                            load = 1, loadUserFunction = UFtorque0))

lTorque1 = mbs.AddLoad(LoadCoordinate(markerNumber = mRotNode1,
                            load = 1, loadUserFunction = UFtorque1))


# add time integration scheme:
mbs.Assemble()

#add dependencies of load user functions on nodal coordinates:
ltgN0 = mbs.systemData.GetNodeLTGODE2(n0)
ltgN1 = mbs.systemData.GetNodeLTGODE2(n1)
#both loads depend on both bodies; this information needs to be added in order that
#  the solver knows dependencies when computing Jacobians
mbs.systemData.AddODE2LoadDependencies(lTorque0, list(ltgN0)+list(ltgN1))
mbs.systemData.AddODE2LoadDependencies(lTorque1, list(ltgN0)+list(ltgN1))

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
simulationSettings.timeIntegration.endTime = 5
# simulationSettings.timeIntegration.simulateInRealtime = True

# simulationSettings.timeIntegration.newton.numericalDifferentiation.doSystemWideDifferentiation = True
simulationSettings.timeIntegration.computeLoadsJacobian = 2
# simulationSettings.timeIntegration.newton.useModifiedNewton = True

simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.solutionWritePeriod = 0.01

simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.displayStatistics = True
simulationSettings.displayComputationTime = True

SC.renderer.Start()
SC.renderer.DoIdleTasks()
mbs.SolveDynamic(simulationSettings)
SC.renderer.Stop()