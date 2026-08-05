
.. _examples-renderernoglfwexample:

************************
rendererNOGLFWexample.py
************************

You can view and download this file on Github: `rendererNOGLFWexample.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/rendererNOGLFWexample.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  An example for offline rendering and merging of images with a ball jumping on ground
   #
   # Author:   Johannes Gerstmayr
   # Date:     2026-02-07
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import InertiaSphere, MarkerBodyRigid, SensorBody
   import exudyn.graphics as graphics
   import numpy as np
   
   useGraphics = False # without test
   
   testSolution = 0
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   radius=0.06
   mass = 0.2                              #mass in kg
   contactStiffness = 2e5*10               #stiffness of spring-damper in N/m
   contactDamping = 1e-4*contactStiffness  #damping constant in N/(m/s); as we have always contact, we only need some damping for initial effects
   dynamicFriction = 0.3
   
   isExplicitSolver = False
   tEnd = 2.5     #end time of simulation
   
   stepSize = 5e-4 #*10
   
   g = 9.81
   
   size = 2.5
   
   quadEdges = graphics.Lines([[-0.5*size,-0.5*size,0],
                              [ 0.5*size,-0.5*size,0],
                              [ 0.5*size, 0.5*size,0],
                              [-0.5*size, 0.5*size,0],
                              [-0.5*size,-0.5*size,0]],color=[0.9,0.9,0.9,1]
                              )
   oGround = mbs.CreateGround(graphicsDataList=[quadEdges,graphics.CheckerBoard(point=[0,0,0],size=size,
                                                                      color=graphics.color.lightgrey[0:3]+[graphics.material.indexChrome],
                                                                      alternatingColor=graphics.color.lightgrey2[0:3]+[graphics.material.indexChrome],
                                                                      ), ])
   mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
   
   
   nSpheres = 5
   
   x = -1
   y = 0
   z = radius+1
   vx = 1
   vy = 0
   vz = 2
   
   oMass = mbs.CreateRigidBody(referencePosition=[x,y,z],
                               initialVelocity=[vx,vy,vz],
                               initialAngularVelocity=[0,0,0],
                               nodeType=exu.NodeType.RotationEulerParameters,
                               inertia=InertiaSphere(mass=mass, radius=radius),
                               gravity = [0,0,-g],
                               graphicsDataList=[graphics.Sphere(radius=radius, 
                                                                 color=graphics.color.dodgerblue,
                                                                 nTiles=32)],
                               )
   mMass = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass))
   
   quadPoints = exu.Vector3DList([[-size,-size,0],[size,-size,0],[size,size,0],[-size,size,0]])
   oSSC = mbs.CreateSphereQuadContact(bodyNumbers=[oMass, oGround],
                                      quadPoints=quadPoints,
                                      includeEdges=15, #all edges
                                      radiusSphere=radius,
                                      contactStiffness = contactStiffness,
                                      contactDamping = contactDamping,
                                      dynamicFriction=dynamicFriction,
                                      show = True
                                      )
   sPos=mbs.AddSensor(SensorBody(bodyNumber=oMass, storeInternal=True,
                                 outputVariableType=exu.OutputVariableType.Position))
   
   
   #exu.Print(mbs)
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.02
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.001  #output interval
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   #simulationSettings.timeIntegration.simulateInRealtime = True
   simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-6
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
   
   simulationSettings.timeIntegration.stepInformation = 3 #remove flag 64 which shows step reduction warnings
   
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   
   simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   SC.visualizationSettings.openGL.lineWidth = 2
   SC.visualizationSettings.view0.scene.drawCoordinateSystem = False
   SC.visualizationSettings.general.showSolverInformation = False
   SC.visualizationSettings.connectors.showContact = False
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 1.25*radius
   
   SC.visualizationSettings.loads.show = False
   
   SC.visualizationSettings.view0.window.showComputationInfo = False
   SC.visualizationSettings.general.backgroundColor = [0,0,0,1]
   
   SC.visualizationSettings.openGL.multiSampling = 2
   SC.visualizationSettings.view0.window.renderWindowSize = [700*3,400*3]
   SC.visualizationSettings.raytracer.numberOfThreads = 16
   SC.visualizationSettings.raytracer.maxReflectionDepth = 0
   SC.visualizationSettings.raytracer.maxTransparencyDepth = 0
   
   if useGraphics:
       SC.renderer.Start()              #start graphics visualization
   
   SC.renderer.ZoomAll() #before first call to RedrawAndGetImage
   SC.renderer.SetModelView(zoom=0.85763,
                            rotationVector=[-1.570796,0,0],
                            centerPoint=[-0.1946863,0.7006764,0.6388125])
   
   if useGraphics:
       SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   mbs.variables['dt'] = 0.075
   mbs.variables['lastRedraw'] = -mbs.variables['dt']
   mbs.variables['listImages'] = []
   def PreStepUserFunction(mbs, t):
       """User function to store images."""
       dt = mbs.variables['dt']
       if t - mbs.variables['lastRedraw'] >= dt-1e-6:
           mbs.variables['lastRedraw'] += dt
           image = SC.renderer.RedrawAndGetImage(True)
           mbs.variables['listImages'].append(image)
           print('get image:', t)
       
       return True
   
   if not useGraphics:
       mbs.SetPreStepUserFunction(PreStepUserFunction)
   
   
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics:
       SC.renderer.Stop()               #safely close rendering window!
   else:
       import matplotlib.pyplot as plt
       if False:
           for image in mbs.variables['listImages']:
               plt.imshow(image)
               plt.show(block=True)
   
       def merge_images(image_list):
           """Merges a list of RGB images by treating black (0,0,0) as transparent."""
           if not image_list:
               return None
               
           # Initialize the canvas with the first image
           merged = image_list[0].astype(np.uint8)
           
           for i in range(1, len(image_list)):
               # Element-wise maximum retains the non-black pixels
               merged = np.maximum(merged, image_list[i])
               
           return merged
       
       image = merge_images(mbs.variables['listImages'])
       plt.imshow(image)
       plt.axis('off')
       plt.show()
       #make sure that directory exists:
       plt.imsave("images/mergedImageBallContact.jpg", image)
   
   mbs.PlotSensor(sPos, components=[2])
       
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   mbs.SolutionViewer()
   


