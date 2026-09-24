#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Builds an `ObjectGround` carrying graphics data - a textured background and a rigid body
#           assembled from primitives - and checks it without drawing anything. This test can only
#           crash, not disagree, so its result is 0 by construction.
#
# Author:   Johannes Gerstmayr
# Date:     2019-11-01, reworked 2026-09-24
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics

testIsActive = exu.sys.get('testIsActive', False)
exu.sys['testTolerance'] = 4e-13 #the tolerance RunAllModelUnitTests used for these ten

SC = exu.SystemContainer()
mbs = SC.AddSystem()

rect = [-2.5,-2,2.5,1] #xmin,ymin,xmax,ymax
background0 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
graphicsRigid1 ={'type':'Circle', 'color':[1,0.1,0.8,1], 'position':[1,2,0], 'radius': 1}  

oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background0,graphicsRigid1])))

mbs.Assemble()

if not testIsActive: 
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

testResult = 0 #this test only fails (crashes) but does not give an error value
exu.Print('solution of GraphicsDataTest=', testResult)
exu.sys['testResult'] = testResult
