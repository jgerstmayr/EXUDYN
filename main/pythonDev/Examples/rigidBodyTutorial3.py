#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  3D rigid body tutorial with 2 bodies and revolute joints, using new utilities functions
#
# Author:   Johannes Gerstmayr
# Date:     2021-08-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import * #includes graphics and rigid body utilities
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++
#physical parameters
g =     [0,-9.81,0] #gravity
L = 1               #length
w = 0.1             #width
bodyDim=[L,w,w] #body dimensions
p0 =    [0,0,0]     #origin of pendulum
pMid0 = np.array([L*0.5,0,0]) #center of mass, body0

#ground body
oGround = mbs.AddObject(ObjectGround())

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++
#first link:
iCube0 = InertiaCuboid(density=5000, sideLengths=bodyDim)
iCube0 = iCube0.Translated([-0.25*L,0,0]) #transform COM, COM not at reference point!

#graphics for body
graphicsBody0 = GraphicsDataRigidLink(p0=[-0.5*L,0,0],p1=[0.5*L,0,0], 
                                     axis0=[0,0,1], axis1=[0,0,0], radius=[0.5*w,0.5*w], 
                                     thickness = w, width = [1.2*w,1.2*w], color=color4red)
graphicsCOM0 = GraphicsDataBasis(origin=iCube0.com, length=2*w)

[n0,b0]=AddRigidBody(mainSys = mbs,
                     inertia = iCube0, #includes COM
                     nodeType = exu.NodeType.RotationEulerParameters,
                     position = pMid0,
                     rotationMatrix = np.diag([1,1,1]),
                     gravity = g,
                     graphicsDataList = [graphicsCOM0, graphicsBody0])


#%%++++++++++++++++++++++++++
#revolute joint (free z-axis)

#markers for ground and rigid body (not needed for option 3):
markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[-0.5*L,0,0]))

#example if wrong marker position is chosen:
#markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[-0.4*L,0,0]))


#revolute joint option 1:
# mbs.AddObject(GenericJoint(markerNumbers=[markerGround, markerBody0J0], 
#                            constrainedAxes=[1,1,1,1,1,0],
#                            visualization=VObjectJointGeneric(axesRadius=0.2*w, axesLength=1.4*w)))

#revolute joint option 2:
# mbs.AddObject(ObjectJointRevoluteZ(markerNumbers = [markerGround, markerBody0J0], 
#                                   rotationMarker0=np.eye(3),
#                                   rotationMarker1=np.eye(3),
#                                   visualization=VObjectJointRevoluteZ(axisRadius=0.2*w, axisLength=1.4*w)
#                                   )) 

#revolute joint option 3:
AddRevoluteJoint(mbs, body0=oGround, body1=b0, point=[0,0,0], 
                  axis=[0,0,1], useGlobalFrame=True, showJoint=True,
                  axisRadius=0.2*w, axisLength=1.4*w)


#%%++++++++++++++++++++++++++
#second link:
graphicsBody1 = GraphicsDataRigidLink(p0=[0,0,-0.5*L],p1=[0,0,0.5*L], 
                                     axis0=[1,0,0], axis1=[0,0,0], radius=[0.06,0.05], 
                                     thickness = 0.1, width = [0.12,0.12], color=color4lightgreen)

iCube1 = InertiaCuboid(density=5000, sideLengths=[0.1,0.1,1])

pMid1 = np.array([L,0,0]) + np.array([0,0,0.5*L]) #center of mass, body1
[n1,b1]=AddRigidBody(mainSys = mbs,
                     inertia = iCube1,
                     nodeType = exu.NodeType.RotationEulerParameters,
                     position = pMid1,
                     rotationMatrix = np.diag([1,1,1]),
                     angularVelocity = [0,0,0],
                     gravity = g,
                     graphicsDataList = [graphicsBody1])

#revolute joint (free x-axis)
AddRevoluteJoint(mbs, body0=b0, body1=b1, point=[L,0,0], 
                  axis=[1,0,0], useGlobalFrame=True, showJoint=True,
                  axisRadius=0.2*w, axisLength=1.4*w)

# #alternative with GenericJoint:
# #markers for rigid body:
# markerBody0J1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[ 0.5*L,0,0]))
# markerBody1J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=[0,0,-0.5*L]))
# mbs.AddObject(GenericJoint(markerNumbers=[markerBody0J1, markerBody1J0], 
#                             constrainedAxes=[1,1,1,0,1,1],
#                             visualization=VObjectJointGeneric(axesRadius=0.2*w, axesLength=1.4*w)))

#position sensor at tip of body1
sens1=mbs.AddSensor(SensorBody(bodyNumber=b1, localPosition=[0,0,0.5*L],
                               fileName='solution/sensorPos.txt',
                               outputVariableType = exu.OutputVariableType.Position))

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++
#assemble system before solving
mbs.Assemble()
if False:
    mbs.systemData.Info() #show detailed information
if False:
    #from exudyn.utilities import DrawSystemGraph
    DrawSystemGraph(mbs, useItemTypes=True) #draw nice graph of system

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 4 #simulation time
h = 1e-3 #step size
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
#simulationSettings.timeIntegration.simulateInRealtime = True
simulationSettings.solutionSettings.solutionWritePeriod = 0.005 #store every 5 ms

SC.visualizationSettings.window.renderWindowSize=[1600,1200]
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.general.autoFitScene = False

SC.visualizationSettings.nodes.drawNodesAsPoint=False
SC.visualizationSettings.nodes.showBasis=True

# exu.StartRenderer()
# if 'renderState' in exu.sys: #reload old view
#     SC.SetRenderState(exu.sys['renderState'])

#mbs.WaitForUserToContinue() #stop before simulating

exu.SolveDynamic(mbs, simulationSettings = simulationSettings,
                 solverType=exu.DynamicSolverType.TrapezoidalIndex2)

# SC.WaitForRenderEngineStopFlag() #stop before closing
# exu.StopRenderer() #safely close rendering window!

sol = LoadSolutionFile('coordinatesSolution.txt')
from exudyn.interactive import SolutionViewer
SolutionViewer(mbs, sol)

if False:
    from exudyn.plot import PlotSensor
    PlotSensor(mbs, [sens1],[1])


