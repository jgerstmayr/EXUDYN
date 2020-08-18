#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  3D rigid body example with joints
#
# Author:   Johannes Gerstmayr
# Date:     2020-03-14
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
from exudyn.graphicsDataUtilities import *

import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()


g = [0,-9.81,0] #gravity
bodyDim = [1,0.1,0.1] #body dimensions
p0 = [0,0,0] #origin of pendulum
pMid0 = [bodyDim[0]*0.5,0,0] #center of mass, body0

#inertia with helper function
iCube = InertiaCuboid(density=5000, sideLengths=[1,0.1,0.1])
print(iCube)

#graphics for body
graphicsBody = GraphicsDataRigidLink(p0=[-0.5*bodyDim[0],0,0],p1=[0.5*bodyDim[0],0,0], 
                                     axis1=[0,0,1], radius=[0.01,0.01], 
                                     thickness = 0.01, width = [0.02,0.02], color=color4lightred)

[n0,b0]=AddRigidBody(mainSys = mbs, 
                     inertia = iCube, 
                     nodeType = str(exu.NodeType.RotationEulerParameters), 
                     position = pMid0, 
                     rotationMatrix = np.diag([1,1,1]),
                     angularVelocity = [0,0,0], 
                     gravity = g, 
                     graphicsDataList = [graphicsBody])

#ground body and marker
oGround = mbs.AddObject(ObjectGround())
markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))

#markers for rigid body:
markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[-0.5*bodyDim[0],0,0]))

#revolute joint (free z-axis)
mbs.AddObject(GenericJoint(markerNumbers=[markerGround, markerBody0J0], 
                           constrainedAxes=[1,1,1,1,1,0],
                           visualization=VObjectJointGeneric(axesRadius=0.01, axesLength=0.1)))

mbs.Assemble()
#mbs.systemData.Info()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

simulationSettings.timeIntegration.numberOfSteps = 100000
simulationSettings.timeIntegration.endTime = 4 #0.2 for testing
#simulationSettings.solutionSettings.solutionWritePeriod = 0.01
#simulationSettings.solutionSettings.sensorsWritePeriod = 0.01
#simulationSettings.timeIntegration.verboseMode = 1

#simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
#simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True

#SC.visualizationSettings.connectors.showJointAxes = True
#SC.visualizationSettings.connectors.jointAxesLength = 0.02
#SC.visualizationSettings.connectors.jointAxesRadius = 0.002

exu.StartRenderer()
mbs.WaitForUserToContinue()

SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simulationSettings)

SC.WaitForRenderEngineStopFlag()
exu.StopRenderer() #safely close rendering window!



