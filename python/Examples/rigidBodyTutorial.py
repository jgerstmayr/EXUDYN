#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Single 3D rigid body example with generic joint
#
# Author:   Johannes Gerstmayr
# Date:     2020-03-14
# Modified: 2024-06-04 (updated to CreateRigidBody)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import ObjectGround, InertiaCuboid, AddRigidBody, MarkerBodyRigid, GenericJoint, \
                             VObjectJointGeneric, SensorBody
#to be sure to have all items and functions imported, just do:
#from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()


g = [0,-9.81,0] #gravity
bodyDim = [1,0.1,0.1] #body dimensions
p0 = [0,0,0] #origin of pendulum
pMid0 = [bodyDim[0]*0.5,0,0] #center of mass, body0

#inertia for cubic body with dimensions in sideLengths
iCube = InertiaCuboid(density=5000, sideLengths=[1,0.1,0.1])
print(iCube)

#graphics for body
graphicsBody = graphics.RigidLink(p0=[-0.5*bodyDim[0],0,0],p1=[0.5*bodyDim[0],0,0], 
                                     axis1=[0,0,1], radius=[0.01,0.01], 
                                     thickness = 0.01, width = [0.02,0.02], color=graphics.color.lightred)

#create rigid body with gravity load with one create function, which creates node, object, marker and load!
b0=mbs.CreateRigidBody(inertia = iCube,
                       referencePosition = pMid0,
                       gravity = g,
                       graphicsDataList = [graphicsBody])

#ground body and marker
oGround = mbs.CreateGround()
markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))

#markers are needed to link joints and bodies; also needed for loads
#markers for rigid body:
markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[-0.5*bodyDim[0],0,0]))

#revolute joint (free z-axis)
mbs.AddObject(GenericJoint(markerNumbers=[markerGround, markerBody0J0], 
                           constrainedAxes=[1,1,1,1,1,0],
                           visualization=VObjectJointGeneric(axesRadius=0.01, axesLength=0.1)))

mbs.Assemble()
#mbs.systemData.Info()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 4
stepSize = 1e-4 #could be larger, but then we do not see the simulation ...

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1 #to see some output

#start graphics
SC.renderer.Start()
SC.renderer.DoIdleTasks() #wait until user presses space

#start generalized alpha solver
mbs.SolveDynamic(simulationSettings = simulationSettings)

SC.renderer.DoIdleTasks()
SC.renderer.Stop() #safely close rendering window!



