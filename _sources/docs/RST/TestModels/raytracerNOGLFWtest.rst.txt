
.. _testmodels-raytracernoglfwtest:

**********************
raytracerNOGLFWtest.py
**********************

You can view and download this file on Github: `raytracerNOGLFWtest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/raytracerNOGLFWtest.py>`_

.. code-block:: python
   :linenos:

   # -*- coding: utf-8 -*-
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Create scissor-like chain of bodies and prismatic joints to test functionality
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-01-14
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   useGraphics = True #without test
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #you can erase the following lines and all exudynTestGlobals related operations if this is not intended to be used as TestModel:
   try: #only if called from test suite
       from modelUnitTests import exudynTestGlobals #for globally storing test results
       useGraphics = exudynTestGlobals.useGraphics
   except:
       class ExudynTestGlobals:
           pass
       exudynTestGlobals = ExudynTestGlobals()
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   useGraphics = False
   import numpy as np
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   gBack = graphics.CheckerBoard(point=[0,0,0], size=10, nTiles=10, normal=[0.,0.,1],
                                 color=[0.9,0.9,0.9]+[graphics.material.indexChrome], 
                                 alternatingColor=[0.8,0.8,0.8]+[graphics.material.indexChrome],
                                 addEdges=False)
   
   text = '''
    !"#$%&'()*+,-./
   0123456789:;<=>?
   @ABCDEFGHIJKLMNO
   PQRSTUVWXYZ[\\]^_
   `abcdefghijklmno
   pqrstuvwxyz{|}~
   ΓΔΘΛΞΠΣΦΨΩαβγδεζ
   ηθικλμνξοπρστυφχ
   ψωϕϵ₀₁₂₃₄₅₆₇₈₉⁰¹
   ²³⁴⁵⁶⁷⁸⁹∂∫♥√≈∞🙂😒
   ÀÁÂÃÄÅÆÇÈÉÊËÌÍÎÏ
   ÐÑÒÓÔÕÖ×ØÙÚÛÜÝÞß
   àáâãäåæçèéêëìíîï
   ðñòóôõö÷øùúûüýþÿ'''
   
   gText = graphics.Text(point=[0,-5,1], text=text, color=[0.2,0.2,0.2,1], fontSize=12)
   
   mbs.CreateGround(referencePosition=[0,0,-1], graphicsDataList=[gBack]+[gText]*(1-useGraphics) )
   
   gObjectsList = []
   
   nMaterials = 10
   nTiles = 16*2
   
   for i in range(nMaterials//2):
       i0 = i*2
       i1 = i*2+1
       color0 = graphics.colorList[i0]
       color1 = graphics.colorList[i1]
       gBrick = graphics.Brick(size=[0.8,1.25,0.7], 
                                     color=color0[:3]+[graphics.material.indexDefault+i0],
                                     addEdges=True,addNormals=True,
                                     )
   
       gSphere = graphics.Sphere(radius=0.8,
                                 color=color1[:3]+[graphics.material.indexDefault+i1],
                                 nTiles=nTiles)
   
       gSphere = graphics.Move(gSphere, [0,0,0], np.diag([1.2,0.8,0.8]) )
       
       gObjectsList.append( mbs.CreateGround(referencePosition=[0,0,0],
                                             graphicsDataList=[gBrick]) )
       gObjectsList.append( mbs.CreateGround(referencePosition=[0,0,0],
                                             graphicsDataList=[gSphere]) )
   
   #just to simulate something
   mbs.CreateRigidBody(referencePosition=[3,0.8,1], 
                       inertia=InertiaCuboid(1000, [2,1.2,1]),
                       show=False)
   
   mbs.Assemble()
   
   import matplotlib.pyplot as plt
   
   mbs.variables['imageCounter'] = 0
   #rotation matrix for view CTRL-7:
   A=RotationVector2RotationMatrix(52/180*pi*np.array([-0.82,0.25,0.5149]))
   
   def PreStepUserFunction(mbs, t):
       if t > 0:
           rot = RotationMatrix2RotationVector(A@RotationMatrixZ(0.5*t*0.5*pi))
           SC.renderer.SetModelView(5.7,rot,[0,0,0])
           if useGraphics:
               image = SC.renderer.RedrawAndGetImage(True)
               imageCounter = mbs.variables['imageCounter']
               exu.Print(f'image: frame{imageCounter:05}.png:',image.shape)
               plt.imsave(f"images/frame{imageCounter:05}.png", image)
               mbs.variables['imageCounter'] += 1
   
       fact = t if t < 1 else 1
       #fact = 1
       for i, obj in enumerate(gObjectsList):
           offset = 0.2*t*0.5*pi
           radius = 4*fact
           phi = offset + i/nMaterials * 2 * pi
           pos = [radius*sin(phi),radius*cos(phi),1.25]
           mbs.SetObjectParameter(obj, 'referencePosition', pos)
           rot = RotXYZ2RotationMatrix([5*phi, 2*phi,0])
           mbs.SetObjectParameter(obj, 'referenceRotation', rot)
       return True
   
   mbs.SetPreStepUserFunction(PreStepUserFunction)
   
   #sizeFactor=1, nTiles=128, lightVariations=71, multisampling=1:
   #OLD: approx 0.7s / image
   windowSize = [1200,1000]
   if not useGraphics:
       windowSize = [400,300] #3
   
   SC.visualizationSettings.view0.window.renderWindowSize = windowSize
   
   SC.visualizationSettings.general.useMultiThreadedRendering = False
   #SC.visualizationSettings.view0.window.showComputationInfo = False
   SC.visualizationSettings.nodes.showNumbers = True
   SC.visualizationSettings.nodes.show = True
   # SC.visualizationSettings.general.textAlwaysInFront = False
   
   
   SC.visualizationSettings.openGL.multiSampling = 2
   SC.visualizationSettings.openGL.light1.enable = False
   SC.visualizationSettings.openGL.light0.position = [0,0,15,1]
   SC.visualizationSettings.openGL.light1.position = [-3,-3,-10,1]
   SC.visualizationSettings.openGL.light1.diffuse = SC.visualizationSettings.openGL.light0.diffuse
   #SC.visualizationSettings.openGL.light1.specular = SC.visualizationSettings.openGL.light0.specular
   SC.visualizationSettings.openGL.light0.shadow = 0.2
   SC.visualizationSettings.raytracer.numberOfThreads = 8+32*useGraphics
   SC.visualizationSettings.openGL.light0.lightRadius = 0.5
   SC.visualizationSettings.raytracer.lightRadiusVariations = 13
   SC.visualizationSettings.raytracer.advanced.shadowSmoothingSteps = 2
   SC.visualizationSettings.raytracer.advanced.shadowScalingFactor = 3
   SC.visualizationSettings.raytracer.verbose = useGraphics+1
   SC.visualizationSettings.raytracer.maxReflectionDepth = 2
   SC.visualizationSettings.raytracer.maxTransparencyDepth = 2
   #SC.visualizationSettings.view0.camera.useRaytracer = True
   SC.visualizationSettings.raytracer.imageSizeFactor = 1
   SC.visualizationSettings.raytracer.keepWindowActive = True
   SC.visualizationSettings.view0.camera.clippingPlaneNormal = [0,1*0,0]
   SC.visualizationSettings.view0.camera.clippingPlaneDistance = 2.
   SC.visualizationSettings.view0.camera.perspective = 1
   
   SC.visualizationSettings.view0.scene.drawCoordinateSystem = 2
   SC.visualizationSettings.general.useGradientBackground = True
   
   
   m = SC.renderer.materials.Get("chrome")
   m.reflectivity = 0.12
   SC.renderer.materials.Set("chrome", m)
   
   if False:
       import matplotlib.pyplot as plt
       exu.Print('get image...')
       SC.renderer.SetState({'displayScaling':1.5})
       SC.renderer.SetModelView(9,[0,0,1],[1,0,0])
       image = SC.renderer.RedrawAndGetImage(True)
       exu.Print('image:',image.shape)
       plt.imshow(image)
       plt.show(block=True)
   
   
   tEnd = 4
   stepSize = 2e-2
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = False
   #simulationSettings.timeIntegration.simulateInRealtime = True
   #simulationSettings.timeIntegration.realtimeFactor = 0.2
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   
   if useGraphics:
       SC.renderer.Start()              #start graphics visualization
       SC.renderer.SetModelView(5.7,52/180*pi*np.array([-0.82,0.25,0.5149]),[0,0,0])
   
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop()
   
   if not useGraphics:
       #this example shows how to retrieve a single image:
       import matplotlib.pyplot as plt
   
       exu.Print('get image...')
       #rotation matrix for view CTRL-7:
       SC.renderer.SetState({'displayScaling':1.5})
       SC.renderer.SetModelView(5.7,52/180*pi*np.array([-0.82,0.25,0.5149]),[0,0,0])
       image = SC.renderer.RedrawAndGetImage(True)[0:int(windowSize[1]*0.9),:,:] #cut out version info!
       if False: #save image for manual check
           exu.Print('image:',image.shape, ', save as images/test.jpg')
           plt.imsave("images/test.jpg", image)
           plt.imshow(image)
           plt.show(block=False)
   
       #compute checksum for image:
       flat_pixels = image.ravel().astype(np.int64) 
       n = flat_pixels.size
       counters = np.arange(n, 2*n, dtype=np.int64) #start at n, so pixels are weighted different, but similar
       checksum = np.sum(flat_pixels * counters)/np.int64(1e14)
       exu.Print('raytracerNOGLFWtest: image checksum=', checksum)
   
       exudynTestGlobals.testResult = checksum
   


