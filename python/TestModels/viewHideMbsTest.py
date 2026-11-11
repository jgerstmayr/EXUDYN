#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  view0.scene.hideMbs (#3442): a SystemContainer with two systems, a red brick in mbs 0 and a blue sphere in
#           mbs 1, is drawn by the raytracer (no window). With hideMbs = [1] the image equals that of a container with
#           mbs 0 alone, with hideMbs = [0, 1] that of an empty scene, and hideMbs = [] shows both; the zoom of the view
#           also leaves the hidden system out. Run alone, it opens the render window with the sphere hidden in view 0
#           and the brick hidden in view 1.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-11
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics
import numpy as np
testIsActive = exu.sys.get('testIsActive', False) #run alone: the render window with two views

errors = 0 #the number of failed checks

def Check(condition, text):
    global errors
    if not condition:
        errors += 1
        exu.Print('viewHideMbsTest: FAILED:', text)

def Container(withSphere=True, withBrick=True):
    """two systems: the brick in mbs 0, the sphere in mbs 1 (either may be left empty)"""
    SC = exu.SystemContainer()
    mbs0 = SC.AddSystem()
    mbs1 = SC.AddSystem()
    if withBrick:
        mbs0.CreateGround(graphicsDataList=[graphics.Brick(centerPoint=[-1, 0, 0], size=[1, 1, 1], color=graphics.color.red)])
    if withSphere:
        mbs1.CreateGround(graphicsDataList=[graphics.Sphere(point=[1, 0, 0], radius=0.6, color=graphics.color.blue, nTiles=32)])
    for mbs in [mbs0, mbs1]:
        mbs.Assemble()
    s = SC.visualizationSettings
    s.view0.window.renderWindowSize = [160, 120]
    s.general.autoFitScene = False
    s.raytracer.numberOfThreads = 2
    s.raytracer.verbose = 0
    s.openGL.multiSampling = 1
    SC.renderer.SetModelView(1, [0, 0, 0], [0, 0, 0])
    return SC

def Image(SC, hideMbs):
    SC.visualizationSettings.view0.scene.hideMbs = hideMbs
    return SC.renderer.RedrawAndGetImage(True).astype(float)

SC = Container()
both = Image(SC, [])
brickOnly = Image(SC, [1])
nothing = Image(SC, [0, 1])
ignored = Image(SC, [5]) #a number without a system is ignored

reference = Container(withSphere=False)
Check(np.abs(brickOnly - Image(reference, [])).max() == 0, 'hideMbs=[1] equals the container with mbs 0 alone')
empty = Container(withSphere=False, withBrick=False)
Check(np.abs(nothing - Image(empty, [])).max() == 0, 'hideMbs=[0,1] equals the empty scene')
Check(np.abs(both - brickOnly).max() > 0, 'hideMbs=[] shows the sphere')
Check(np.abs(both - ignored).max() == 0, 'a number without a system is ignored')
Check(list(SC.visualizationSettings.view0.scene.hideMbs) == [5], 'the setting is stored')

#the share of blue pixels: the sphere is shown only without hiding
def Blue(image): return float(np.mean((image[:, :, 2] > 150) & (image[:, :, 0] < 100)))
exu.Print('viewHideMbsTest: blue share', Blue(both), Blue(brickOnly), Blue(nothing))
Check(Blue(both) > 0.01 and Blue(brickOnly) == 0, 'the blue sphere only without hiding')

if not testIsActive:
    SC = Container()
    SC.visualizationSettings.general.autoFitScene = True
    SC.visualizationSettings.view0.scene.hideMbs = [1]
    SC.visualizationSettings.view1.window.showWindow = True
    SC.visualizationSettings.view1.scene.hideMbs = [0]
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop()

u = float(errors) #the pixels themselves differ between platforms
exu.Print('viewHideMbsTest: failed checks:', errors)
exu.Print('solution of viewHideMbsTest=', u)
exu.sys['testResult'] = u
