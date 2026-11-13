#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Blurred reflections in the raytracer (#3473): a reflective floor of the material chrome with a brick and a
#           sphere above it, rendered with roughness 0 (sharp), with roughness 0.08 and raytracer.reflectionSamples = 1
#           (the blur turned off: the same image as the sharp one) and with roughness 0.08 and 8 or 64 reflected rays
#           per pixel. The image of 8 rays is closer to that of 64 rays than the sharp one is. The result is the number
#           of these checks that hold, plus the mean difference between the sharp and the blurred image in 0..255,
#           rounded to one digit.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-13
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
#a local run must not open a plot window when windows are suppressed (#2477)
if exu.special.userInterface.suppressPlots: import matplotlib; matplotlib.use('Agg')
import exudyn.graphics as graphics
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()
floor = graphics.CheckerBoard(point=[0, 0, 0], size=6, nTiles=6, normal=[0, 0, 1],
                              color=[0.8, 0.8, 0.8] + [graphics.material.indexChrome],
                              alternatingColor=[0.6, 0.6, 0.6] + [graphics.material.indexChrome], addEdges=False)
brick = graphics.Brick([-0.8, 0, 0.8], size=[0.8, 1.2, 0.8], color=graphics.color.red)
sphere = graphics.Sphere([1., 0.3, 0.7], radius=0.6, color=graphics.color.dodgerblue, nTiles=24)
mbs.CreateGround(graphicsDataList=[floor, brick, sphere])
mbs.Assemble()

settings = SC.visualizationSettings
settings.view0.window.renderWindowSize = [160, 120]
settings.general.useGradientBackground = False
settings.view0.window.showComputationInfo = False
settings.general.showSolverInformation = False
settings.view0.scene.drawWorldBasis = False
settings.view0.scene.drawCoordinateSystem = 0
settings.raytracer.numberOfThreads = 4
settings.raytracer.maxReflectionDepth = 1
settings.openGL.light0.position = [3, -4, 8, 1]
settings.openGL.light0.shadow = 0.3
settings.view0.camera.perspective = 1
SC.renderer.RedrawAndGetImage(True) #initializes the render state, which SetModelView then changes
SC.renderer.SetModelView(3., [-1., 0, 0], [0, 0.3, 0.5]) #looking at the floor at an angle

chrome = SC.renderer.materials.Get('chrome')
chrome.reflectivity = 0.5

def Render(roughness, reflectionSamples):
    """the raytraced image with the floor of this roughness and number of reflected rays"""
    chrome.roughness = roughness
    SC.renderer.materials.Set('chrome', chrome)
    settings.raytracer.reflectionSamples = reflectionSamples
    return SC.renderer.RedrawAndGetImage(True)[:, :, :3].astype(int)

sharp = Render(0., 8)
blurOff = Render(0.08, 1)
blurred = Render(0.08, 8)
blurredFine = Render(0.08, 64)

def MeanDifference(image0, image1):
    return float(np.mean(np.abs(image0 - image1)))

checks = [bool(np.array_equal(sharp, blurOff)), not np.array_equal(sharp, blurred),
          MeanDifference(blurred, blurredFine) < MeanDifference(sharp, blurredFine)]
meanDifference = MeanDifference(blurred, sharp)
exu.Print('checks:', checks, ', against 64 rays: 8 rays', MeanDifference(blurred, blurredFine), ', sharp',
          MeanDifference(sharp, blurredFine), ', blurred against sharp', meanDifference)

if not testIsActive:
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(1, 2)
    axes[0].imshow(sharp.astype(np.uint8)); axes[0].set_title('roughness 0')
    axes[1].imshow(blurred.astype(np.uint8)); axes[1].set_title('roughness 0.08, 8 rays')
    plt.show()

u = sum(checks) + round(meanDifference, 1)
exu.Print('solution of raytracerGlossyReflectionTest=', u)
exu.sys['testResult'] = u
