#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  demo showing import of simple .obj file using specialized interface functions;
#           requires pymeshlab to be installed!
#
# Author:   Johannes Gerstmayr
# Date:     2026-02-11
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict #includes graphics and rigid body utilities
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++
#physical parameters
g =     [0,0,-9.81] #gravity
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
#chose one of the files:
fileName = '../Examples/testData/objImportTest.obj'

graphicsBody0 = graphics.FromPyMeshlabFile(fileName, 
                                           invertNormals=False,
                                           invertTriangles=False,
                                           defaultColor = graphics.color.dodgerblue, 
                                           useDefaultColor = True,
                                           verbose=True)

#+++++++++++++++++++++++++++++++++++++++++++++
# repair with trimesh
# [V, F] = graphics.ToPointsAndTrigs( graphicsBody0)
# import trimesh
# mesh = trimesh.Trimesh(vertices=V, faces=F, process=False)
# trimesh.repair.fix_winding(mesh)
# mesh.fix_normals()
# triangles = mesh.faces
# triangles = np.take(triangles, [0,2,1], axis=1)

# graphicsBody0 = graphics.FromPointsAndTrigs(points=mesh.vertices, 
#                                             triangles=triangles,
#                                             normals=mesh.vertex_normals,
#                                             color=graphics.color.steelblue)
#+++++++++++++++++++++++++++++++++++++++++++++

#transform or scale:
#graphicsBody0 = graphics.Move(graphicsBody0, [0,0,0], 0.001*np.eye(3))

#+++++++++++++++++++++++++++++++++++++++++++++
#smoothen
graphicsBody0 = graphics.AddEdgesAndSmoothenNormals(graphicsBody0, edgeAngle=0.25*pi, 
                                                    edgeColor=[0,0,0,0])
#+++++++++++++++++++++++++++++++++++++++++++++


graphicsCOM0 = graphics.Basis(origin=iCube0.com, length=2*w)

dictCube0 = mbs.CreateRigidBody(
              inertia=iCube0, 
              referencePosition=pMid0,
              referenceRotationMatrix=np.diag([1,1,1]),
              gravity=g,
              graphicsDataList=[graphicsCOM0, graphicsBody0],
              returnDict=True)
[n0, b0] = [dictCube0['nodeNumber'], dictCube0['bodyNumber']]


#%%++++++++++++++++++++++++++
#revolute joint (free z-axis)

#revolute joint option 3:
mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b0], position=[0,0,0], axis=[0,1,0], 
                        axisRadius=0.2*w, axisLength=1.4*w)

# AddRevolute*Joint(mbs, body0=oGround, body1=b0, point=[0,0,0], 
#                   axis=[0,0,1], useGlobalFrame=True, showJoint=True,
#                   axisRadius=0.2*w, axisLength=1.4*w)

#assemble system before solving
mbs.Assemble()
simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 4 #simulation time
h = 1e-3 #step size
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
#simulationSettings.timeIntegration.simulateInRealtime = True
simulationSettings.solutionSettings.solutionWritePeriod = 0.005 #store every 5 ms

SC.visualizationSettings.view0.window.renderWindowSize=[1600,1200]
SC.visualizationSettings.openGL.multiSampling = 1
SC.visualizationSettings.openGL.lineWidth = 1
SC.visualizationSettings.openGL.light0.shadow = 0.2*0
# SC.visualizationSettings.openGL.advanced.lightModelTwoSide = True
# SC.visualizationSettings.openGL.light0.position = [3,4,10,1]
SC.visualizationSettings.general.autoFitScene = True

SC.visualizationSettings.nodes.drawNodesAsPoint=False
SC.visualizationSettings.nodes.showBasis=True
SC.visualizationSettings.general.useGradientBackground = True

SC.renderer.Start()
if 'renderState' in exu.sys: #reload old view
    SC.renderer.SetState(exu.sys['renderState'])

SC.renderer.DoIdleTasks() #stop before simulating

mbs.SolveDynamic(simulationSettings = simulationSettings,
                 solverType=exu.DynamicSolverType.TrapezoidalIndex2)

SC.renderer.DoIdleTasks() #stop before closing
SC.renderer.Stop() #safely close rendering window!

