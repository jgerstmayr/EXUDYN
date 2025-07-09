
.. _examples-graphicsdataexample:

**********************
graphicsDataExample.py
**********************

You can view and download this file on Github: `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/graphicsDataExample.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example for graphicsData
   #
   # Author:   Johannes Gerstmayr
   # Date:     2023-03-26
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #rigid pendulum:
   # L = 1    #x-dim of pendulum
   # b = 0.1  #y-dim of pendulum
   
   #create list of graphics data for background
   gList = []
   
   gList += [graphics.CheckerBoard(point=[0,0,-2], normal=[0,0,1], size=5, 
                                   color=graphics.color.lightgrey[0:3]+[graphics.material.indexChrome],
                                   alternatingColor=graphics.color.lightgrey2[0:3]+[graphics.material.indexChrome] )]
   gList += [graphics.Arrow(pAxis=[-2,-2,-1], vAxis=[0,0,2], radius=0.04, color=graphics.color.yellow)]
   gList += [graphics.Basis(origin=[-2,-1,-1], length=0.5)]
   gList += [graphics.Cylinder(pAxis=[-2,0,-1], vAxis=[0,0,2], radius=0.3, addEdges=4, color=graphics.color.white, nTiles=32)]
   gList += [graphics.Sphere(point=[-2,1,0], radius=0.5, color=graphics.color.white, nTiles=16,
                                           addEdges=6, addFaces=True)]
   gList += [graphics.Brick(centerPoint=[-2,-3,-1], size=[0.4,0.5,0.6], color=graphics.color.dodgerblue)]
   
   #for hollow solid, use closed contour list:
   contour=[[0,0.],[0,0.1],[0.4,0.1],[0.4,0.2],[0.8,0.3],[1.2,0.]] #if wrong orientation (check normals): contour.reverse()
   gList += [graphics.SolidOfRevolution(pAxis=[0,-2,-1], vAxis=[0,0,1],
           contour=contour, color=[0.8,0.1,0.1,1], nTiles = 40, addEdges=5)]
   
   #the next two graphics lists are merged into a single list:
   contour=[[0,0.1],[0.,0.2],[1.5,0.3],[1.5,0.15],[0,0.1]]
   #contour.reverse()
   g1 = graphics.SolidOfRevolution(pAxis=[0,0,-1], vAxis=[0,0,1],
           contour=contour, color=[0.8,0.1,0.1,1], nTiles = 6, smoothContour=False, addEdges=6)
   
   g2 = graphics.SolidExtrusion(vertices=[[-0.4,-0.4], [0.4,-0.4], [ 0.4,0.4], [0.1,0.4], 
                                                  [0.1,  0.2], [-0.1,0.2], [-0.1,0.4], [-0.4,0.4]], 
                                        segments=[[0,1], [1,2], [2,3], [3,4], [4,5], [5,6], [6,7], [7,0]],
                                        pOff = [0,2,-1], height=1.5, 
                                        color=graphics.color.steelblue, addEdges=2)
   g3 = graphics.MergeTriangleLists(g1,g2)
   #now move and rotate graphicsData:
   g3 = graphics.Move(g3, pOff=[0.25,0,1], Aoff=RotationMatrixX(0.125*pi))
   
   gList+=[g3]
   
   for i, gData in enumerate(gList):
       exu.Print(f'gData{i}:')
       if 'type' in gData:
           exu.Print('  graphicsData type=',gData['type'])
           if gData['type'] == 'TriangleList' and 'normals' in gData:
               nWrong = graphics.InconsistentTriangles(gData)
               exu.Print('  inconsistent trigs=',nWrong,'of',len(gData['triangles']//3))
           
   
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= gList)))
   
   # graphicsCube = graphics.Brick(size= [L,b,b], color= graphics.color.dodgerblue, addEdges=True)
   # graphicsJoint = graphics.Cylinder(pAxis=[-0.5*L,0,-0.6*b], vAxis= [0,0,1.2*b], radius = 0.55*b, color=graphics.color.darkgrey, addEdges=True)
   
   mbs.CreateRigidBody(inertia = InertiaSphere(1, 0.5), 
                referencePosition = [1,1,0], 
                initialAngularVelocity =[1,0,0],
                graphicsDataList = [graphics.Sphere(radius=0.5, color=graphics.color.orange, nTiles=32,
                                                       addEdges=4, addFaces=True)])
   
   
   mbs.Assemble()
   print(mbs)
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   simulationSettings.timeIntegration.numberOfSteps = 100000
   simulationSettings.timeIntegration.endTime = 2000
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.timeIntegration.simulateInRealtime = True
   
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.openGL.lineWidth = 1
   # SC.visualizationSettings.openGL.drawVertexNormals = True
   # SC.visualizationSettings.openGL.drawFaceNormals = True
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
   SC.visualizationSettings.window.renderWindowSize=[1920,1200]
   
   SC.renderer.Start()
   mbs.SolveDynamic(simulationSettings)
   
   SC.renderer.DoIdleTasks()
   SC.renderer.Stop() #safely close rendering window!
   


