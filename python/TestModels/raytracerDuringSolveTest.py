#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The raytracer during a solve (#3438): a chain of rigid bodies on revolute joints, solved on 4 threads; a
#           PreStepUserFunction renders the current state with the raytracer (RedrawAndGetImage) on 1 and on 4 threads.
#           The raytracer has threads of its own, independent of the solver's, so both images are equal and show the
#           bodies moved. The result is the number of rendered pairs plus the number of errors.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-12
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#a chain of bricks on revolute joints about the z-axis, under gravity
(L, w, numberOfBodies) = (0.5, 0.08, 6)
oGround = mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[0, -1.8, -0.1], size=6)])
previousBody = oGround
for i in range(numberOfBodies):
    color = graphics.colorList[i % len(graphics.colorList)]
    oBody = mbs.CreateRigidBody(inertia=InertiaCuboid(density=1000, sideLengths=[L, w, w]),
                                referencePosition=[(i+0.5)*L, 0, 0], gravity=[0, -9.81, 0],
                                graphicsDataList=[graphics.Brick(size=[L, w, w], color=color)])
    mbs.CreateRevoluteJoint(itemNumbers=[previousBody, oBody], position=[i*L, 0, 0], axis=[0, 0, 1],
                            axisRadius=0.6*w, axisLength=1.2*w)
    previousBody = oBody
mbs.Assemble()

v = SC.visualizationSettings
v.view0.window.renderWindowSize = [240, 180]
v.view0.window.showComputationInfo = False
v.view0.scene.drawCoordinateSystem = 0
v.view0.scene.drawWorldBasis = False
v.connectors.show = False
v.nodes.show = False
SC.renderer.ZoomAll()

images = [] #the images on 1 and 4 threads, rendered at the steps of renderSteps
renderSteps = [1, 39]
def PreStepUserFunction(mbs, t):
    step = int(round(t/h))
    if step in renderSteps:
        pair = []
        for numberOfThreads in [1, 4]:
            v.raytracer.numberOfThreads = numberOfThreads
            pair.append(SC.renderer.RedrawAndGetImage(useRaytracer=True).astype(int))
        images.append(pair)
    return True
mbs.SetPreStepUserFunction(PreStepUserFunction)

h = 2e-3
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 40
simulationSettings.timeIntegration.endTime = 40*h
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.solution.file.write = False
simulationSettings.parallel.numberOfThreads = 4 #the solver's threads, while the raytracer starts its own
mbs.SolveDynamic(simulationSettings)

errors = int(len(images) != len(renderSteps))
for (imageSerial, imageParallel) in images:
    errors += int(not np.array_equal(imageSerial, imageParallel))
moved = int(np.abs(images[-1][0] - images[0][0]).sum()) if len(images) == len(renderSteps) else 0
errors += int(moved == 0) #the chain falls, so the images differ
exu.Print('raytracer during the solve: images', len(images), ', serial and parallel equal:', errors == 0, ', changed pixel values', moved)
if errors:
    exu.Print('raytracerDuringSolveTest: FAILED')

if not testIsActive:
    mbs.SetPreStepUserFunction(0)
    simulationSettings.timeIntegration.numberOfSteps = 1000
    simulationSettings.timeIntegration.endTime = 2
    simulationSettings.solution.file.write = True
    v.raytracer.numberOfThreads = 0
    mbs.SolveDynamic(simulationSettings)
    mbs.SolutionViewer()

result = len(images) + errors #the pixels themselves may differ by round-off between platforms
exu.Print('solution of raytracerDuringSolveTest=', result)
exu.sys['testResult'] = result
