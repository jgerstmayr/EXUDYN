#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Textures (#3033): a wooden floor from an image file and a spinning cylinder with a pattern painted by
#           sin and cos; graphics.ImageFromFile reads testData/hardwood_small.jpg (from OpenGameArt, CC0), the
#           pattern is a numpy array; SC.renderer.AddTexture adds both, graphics.Quad and graphics.Cylinder put them
#           on the geometry, graphics.ProjectTexture puts the wood on a brick. With useRaytracer=True the solution
#           viewer renders with the raytracer, with soft shadows and multisampling.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-18
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics
import exudyn.cMath as cMath
import numpy as np

useRaytracer = True #False: OpenGL only, faster for adjusting the scene

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the textures: the image of a wooden floor, and a pattern computed from sin and cos
wood = graphics.ImageFromFile(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'testData', 'hardwood_small.jpg')) #1024 x 1024 x 3, uint8
SC.renderer.AddTexture('hardwood', wood, interpretation=exu.TextureInterpretation.RGB)

(height, width) = (256, 512) #rows along the axis of the cylinder, columns around it
(X, Y) = np.meshgrid(np.linspace(0, 1, width, endpoint=False), np.linspace(0, 1, height))
red = 0.5 + 0.5*cMath.sin(2*pi*(3*X + 2*Y))     #whole periods around, so that there is no seam
green = 0.5 + 0.5*cMath.cos(2*pi*(5*X - Y))*cMath.sin(2*pi*Y)
blue = 0.5 + 0.5*cMath.sin(2*pi*8*X)*cMath.cos(2*pi*3*Y)
pattern = np.stack([red, green, blue], axis=2)  #floats in [0, 1]
SC.renderer.AddTexture('pattern', pattern, interpretation=exu.TextureInterpretation.RGB)

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the ground: a floor of 4 x 4 m with the wood repeated twice in each direction, and a wooden box on it
floor = graphics.Quad([[-2, -2, 0], [2, -2, 0], [2, 2, 0], [-2, 2, 0]], texture='hardwood', textureTiles=[2, 2])
box = graphics.ProjectTexture(graphics.Brick(centerPoint=[-1.2, 1.0, 0.25], size=[0.8, 0.5, 0.5]), 'hardwood',
                              projection='box', size=1.)
oGround = mbs.CreateGround(graphicsDataList=[floor, box])

#the cylinder: radius 0.4 m, length 1 m, spinning about its horizontal axis in a revolute joint
radius = 0.4
length = 1.
cylinder = graphics.Cylinder(pAxis=[-0.5*length, 0, 0], vAxis=[length, 0, 0], radius=radius, nTiles=64,
                             texture='pattern', textureTiles=[1, 1])
oCylinder = mbs.CreateRigidBody(inertia=InertiaCylinder(density=500, length=length, outerRadius=radius, axis=0),
                                referencePosition=[0.3, -0.2, radius + 0.1],
                                initialAngularVelocity=[2*pi*0.25, 0, 0], #a quarter turn per second
                                gravity=[0, 0, -9.81],
                                graphicsDataList=[cylinder])
mbs.CreateRevoluteJoint(itemNumbers=[oGround, oCylinder], position=[0.3, -0.2, radius + 0.1], axis=[1, 0, 0],
                        axisRadius=0.02, axisLength=1.2*length)

mbs.Assemble()

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tEnd = 4
stepSize = 0.01
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solution.file.writePeriod = 0.02

v = SC.visualizationSettings
v.view0.scene.drawWorldBasis = True
v.general.showSolverInformation = False
v.connectors.show = False
v.openGL.light0.position = [3, -4, 6, 0]  #a directional light from above, casting the shadows
v.openGL.light0.shadow = 0.4
v.openGL.light0.lightRadius = 0.15          #soft shadows in the raytracer
v.openGL.lightModelAmbient = [0.5, 0.5, 0.5, 1]

if useRaytracer:
    v.view0.camera.useRaytracer = True
    v.raytracer.numberOfThreads = 8         #about the number of cores of the computer
    v.raytracer.multiSampling = 2           #smooth edges and textures, 4 times the work
    v.raytracer.lightRadiusVariations = 7   #soft shadows from 7 points of the light
    v.raytracer.imageSizeFactor = 1         #2 or 4 for a faster preview
    v.raytracer.keepWindowActive = True

mbs.SolveDynamic(simulationSettings)

exu.Print('angular velocity of the cylinder after', tEnd, 's:', mbs.GetObjectOutputBody(oCylinder, exu.OutputVariableType.AngularVelocity)[0])

SC.renderer.Start()
SC.renderer.SetModelView(2.8, [-1.0, 0, 0.5], [-0.2, 0.2, 0.3]) #zoom, rotation vector and center of the view
mbs.SolutionViewer()
SC.renderer.Stop()
