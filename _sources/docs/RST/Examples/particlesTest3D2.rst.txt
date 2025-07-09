
.. _examples-particlestest3d2:

*******************
particlesTest3D2.py
*******************

You can view and download this file on Github: `particlesTest3D2.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/particlesTest3D2.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  test with parallel computation and particles
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-11-01
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.graphicsDataUtilities import *
   
   import numpy as np
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #create an environment for mini example
   
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))
   #mLast = mbs.AddMarker(MarkerNodePosition(nodeNumber=nGround))
   
   np.random.seed(1) #always get same results
   
   useGraphics = True
   
   L = 1
   n = 50000
   a = 0.2*L*0.5*10*0.5
   radius = 0.5*a
   m = 0.05
   k = 8e4*2 #4e3 needs h=1e-4
   d = 0.001*k*4*0.5*0.2
   markerList = []
   radiusList = []
   gDataList = []
   
   
   rb = 30*L
   H = 8*L
   Hy=3*L
   pos0 = [0,-rb-0.5*H,0]
   pos1 = [-rb-H,-Hy,0]
   pos2 = [ rb+H,-Hy,0]
   pos3 = [ 0,-Hy,rb+H]
   pos4 = [ 0,-Hy,-rb-H]
   posList=[pos0,pos1,pos2,pos3,pos4]
   for pos in posList:
       #gDataList += [{'type':'Circle','position':pos,'radius':rb, 'color':graphics.color.grey}]
       #gDataList += [graphics.Cylinder(pAxis=pos, vAxis=[0,0,0.1], radius=rb, color= graphics.color.grey, nTiles=200)]
       colBG = graphics.color.grey
       colBG[3] = 0.05
       gDataList += [graphics.Sphere(point=pos, radius=rb, color= colBG, nTiles=100)]
       #gDataList += [GraphicsDataRectangle(-1.2*H,-H,1.2*H,14*H,color=graphics.color.red)]#80000 particles
       nMass = mbs.AddNode(NodePointGround(referenceCoordinates=pos,
                           visualization=VNodePointGround(show=False)))
       #oMass = mbs.AddObject(MassPoint(physicsMass=m, nodeNumber=nMass))
       mThis = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
       markerList += [mThis]
       radiusList += [rb]
   
   
   color4node = graphics.color.blue
   print("start create: number of masses =",n)
   for i in range(n):
   
       kk = int(i/int(n/16))
       color4node = graphics.colorList[min(kk%9,9)]
   
       if (i%20000 == 0): print("create mass",i)
       offy = 0
       row = 8*2 #160
       
       iy = int(i/(row*row))
       ix = i%row
       iz = int(i/row)%row
   
       if iy % 2 == 1:
           ix+=0.5
           iz+=0.5
   
       offy = -0.25*H-3.5*a+iy*a*0.74 #0.70x is limit value!
       offx = -0.6*a-H*0.5 + (ix+1)*a
       offz = -0.6*a-H*0.5 + (iz+1)*a
   
       valueRand = np.random.random(1)[0]
       rFact = 0.2 #random part
       gRad = radius*(1-rFact+rFact*valueRand)
       nMass = mbs.AddNode(NodePoint(referenceCoordinates=[offx,offy,offz],
                                     initialVelocities=[0,-20,0],
                                     visualization=VNodePoint(show=True,drawSize=2*gRad, color=color4node)))
       
       oMass = mbs.AddObject(MassPoint(physicsMass=m, nodeNumber=nMass,
                                       #visualization=VMassPoint(graphicsData=[gSphere,gSphere2])
                                       # visualization=VMassPoint(graphicsData=gData)
                                       ))
       mThis = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
       mbs.AddLoad(Force(markerNumber=mThis, loadVector= [0,-m*9.81,0]))
       markerList += [mThis]
       radiusList += [gRad]
   
       mLast = mThis
   print("finish create")
   #put here, such that it is transparent in background
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                      visualization=VObjectGround(graphicsData=gDataList)))
   
   if True:
       gContact = mbs.AddGeneralContact()
       gContact.verboseMode = 1
   
       for i in range(len(markerList)):
           m = markerList[i]
           r = radiusList[i]
           gContact.AddSphereWithMarker(m, radius=r, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)
   
       # f=n/32000
       ssx = 20 #search tree size
       #ssy = int(500*f) #search tree size
       ssy = 200
       # mbs.Assemble()
       # gContact.FinalizeContact(mbs, searchTreeSize=np.array([ssx,ssy,ssx]), frictionPairingsInit=np.eye(1), 
       #                          searchTreeBoxMin=np.array([-1.2*H,-H,-1.2*H]), searchTreeBoxMax=np.array([1.2*H,14*H,1.2*H]) #80000 particles
       #                          )
       gContact.SetFrictionPairings(np.eye(1))
       gContact.SetSearchTreeCellSize(numberOfCells=[ssx,ssy,ssx])
       gContact.SetSearchTreeBox(pMin=np.array([-1.2*H,-H,-1.2*H]), pMax=np.array([1.2*H,14*H,1.2*H]))
       print('treesize=',ssx*ssx*ssy)
   
   mbs.Assemble()
   print("finish gContact")
   
   tEnd = 10
   h= 0.0001*0.25
   simulationSettings = exu.SimulationSettings()
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   #simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.02
   simulationSettings.solutionSettings.outputPrecision = 5 #make files smaller
   simulationSettings.solutionSettings.exportAccelerations = False
   simulationSettings.solutionSettings.exportVelocities = False
   simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/test.txt'
   simulationSettings.displayComputationTime = True
   #simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.parallel.numberOfThreads = 4
   
   simulationSettings.timeIntegration.newton.numericalDifferentiation.forODE2 = False
   simulationSettings.timeIntegration.newton.useModifiedNewton = False
   
   SC.visualizationSettings.general.graphicsUpdateInterval=0.5
   SC.visualizationSettings.general.circleTiling=200
   SC.visualizationSettings.general.drawCoordinateSystem=False
   SC.visualizationSettings.loads.show=False
   SC.visualizationSettings.bodies.show=True
   SC.visualizationSettings.markers.show=False
   
   SC.visualizationSettings.nodes.show=True
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   SC.visualizationSettings.nodes.defaultSize = 0 #must not be -1, otherwise uses autocomputed size
   SC.visualizationSettings.nodes.tiling = 4
   
   SC.visualizationSettings.window.renderWindowSize=[1200,1200]
   #SC.visualizationSettings.window.renderWindowSize=[1024,1400]
   SC.visualizationSettings.openGL.multiSampling = 4
   #improved OpenGL rendering
   
   SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
   SC.visualizationSettings.exportImages.saveImageTimeOut=10000 #5000 is too shot sometimes!
   if False:
       simulationSettings.solutionSettings.recordImagesInterval = 0.025
       SC.visualizationSettings.general.graphicsUpdateInterval=2
   
   
   simulate=True
   if simulate:
       if useGraphics:
           SC.visualizationSettings.general.autoFitScene = False
           SC.renderer.Start()
           if 'renderState' in exu.sys:
               SC.renderer.SetState(exu.sys['renderState'])
           SC.renderer.DoIdleTasks()
   
       #initial gContact statistics
       #simulationSettings.timeIntegration.numberOfSteps = 1
       #simulationSettings.timeIntegration.endTime = h
       #mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitEuler)
       #print(gContact)
   
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.timeIntegration.explicitIntegration.computeEndOfStepAccelerations = False #increase performance, accelerations less accurate
       mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitEuler)
       #print(gContact)
       #p = mbs.GetNodeOutput(n, variableType=exu.OutputVariableType.Position)
       #print("pEnd =", p[0], p[1])
       #print(gContact)
   
       if useGraphics:
           SC.renderer.DoIdleTasks()
           SC.renderer.Stop() #safely close rendering window!
   else:
       SC.visualizationSettings.general.autoFitScene = False
       SC.visualizationSettings.general.graphicsUpdateInterval=0.5
       
       print('load solution file')
       sol = LoadSolutionFile('particles3Db.txt', safeMode=True)
       #sol = LoadSolutionFile('coordinatesSolution2.txt')
       print('start SolutionViewer')
       mbs.SolutionViewer(sol)


