
.. _examples-particlestest:

****************
particlesTest.py
****************

You can view and download this file on Github: `particlesTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/particlesTest.py>`_

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
   from exudyn.utilities import *
   from exudyn.graphicsDataUtilities import *
   
   import numpy as np
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #create an environment for mini example
   
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))
   #mLast = mbs.AddMarker(MarkerNodePosition(nodeNumber=nGround))
   
   np.random.seed(1) #always get same results
   
   L = 1
   n = 32000 #*8*4 #32*8*8
   a = 0.2*L*0.5
   radius = 0.35*a
   m = 0.05
   k = 4e3*10 #4e3 needs h=1e-4
   d = 0.001*k*4*0.5
   markerList = []
   radiusList = []
   gDataList = []
   
   
   rb = 30*L
   H = 8*L
   pos0 = [0,-rb-0.5*H,0]
   pos1 = [-rb-H,0,0]
   pos2 = [ rb+H,0,0]
   posList=[pos0,pos1,pos2]
   for pos in posList:
       #gDataList += [{'type':'Circle','position':pos,'radius':rb, 'color':color4grey}]
       gDataList += [GraphicsDataCylinder(pAxis=pos, vAxis=[0,0,0.1], radius=rb, color= color4grey, nTiles=200)]
       #gDataList += [GraphicsDataRectangle(-1.2*H,-H*0.75,1.2*H,10*H,color=color4red)]
       nMass = mbs.AddNode(NodePointGround(referenceCoordinates=pos))
       #oMass = mbs.AddObject(MassPoint(physicsMass=m, nodeNumber=nMass))
       mThis = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
       markerList += [mThis]
       radiusList += [rb]
   
       oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                          visualization=VObjectGround(graphicsData=gDataList)))
   
   ns = 20
   gDataSphere = []
   for i in range(ns):
       gRad = radius*(0.75+0.4*(i/ns))
       gSphere = GraphicsDataCylinder(pAxis=[0,0,-0.25], vAxis=[0,0,0.5], radius=gRad, color=color4blue, nTiles=12)
       gSphere2 = GraphicsDataCylinder(pAxis=[0,0,-0.3], vAxis=[0,0,0.6], radius=0.8*gRad, color=color4steelblue, nTiles=10)
       gDataSphere += [[gSphere, gSphere2]]
   
   
   print("start create: number of masses =",n)
   for i in range(n):
       if (i%20000 == 0): print("create mass",i)
       offy = 0
       row = 80*2
       offy = -2*L-a*9+int(i/row)*a+a*0.2*np.random.random(1)[0]
   
       valueRand = np.random.random(1)[0]
       gRad = radius*(0.75+0.4*valueRand)
       #gSphere = GraphicsDataCylinder(pAxis=[0,0,-0.25], vAxis=[0,0,0.25], radius=gRad, color= color4steelblue, nTiles=16)
       #gSphere2 = GraphicsDataCylinder(pAxis=[0,0,-0.3], vAxis=[0,0,0.3], radius=0.8*gRad, color= color4blue, nTiles=12)
       nMass = mbs.AddNode(NodePoint(referenceCoordinates=[-0.6*a-H + (i%row+1)*a+0.2*a*np.random.random(1)[0],offy,0],
                                     initialVelocities=[0,-5,0]))
       oMass = mbs.AddObject(MassPoint(physicsMass=m, nodeNumber=nMass,
                                       #visualization=VMassPoint(graphicsData=[gSphere,gSphere2])
                                       visualization=VMassPoint(graphicsData=gDataSphere[int(valueRand*ns)])
                                       ))
       mThis = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
       mbs.AddLoad(Force(markerNumber=mThis, loadVector= [0,-m*9.81,0]))
       markerList += [mThis]
       radiusList += [gRad]
       #if (i==n-1):
       #    mbs.AddLoad(Force(markerNumber = mThis, loadVector = [5, -20, 0])) 
   
       #mbs.AddObject(CartesianSpringDamper(markerNumbers=[mLast, mThis], 
       #                                    stiffness = [k,k,k], damping=[d,d,d], offset=[a,0,0],
       #                                    visualization = VCartesianSpringDamper(drawSize = 0.1*a)))
   
       mLast = mThis
   print("finish create")
   
   if True:
       gContact = mbs.AddGeneralContact()
       gContact.verboseMode = 1
   
       for i in range(len(markerList)):
           m = markerList[i]
           r = radiusList[i]
           gContact.AddSphereWithMarker(m, radius=r, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)
   
       f=n/32000
       ssx = 100 #search tree size
       ssy = int(500*f) #search tree size
       # mbs.Assemble()
       # gContact.FinalizeContact(mbs, searchTreeSize=np.array([ssx,ssy,1]), frictionPairingsInit=np.eye(1), 
       #                          searchTreeBoxMin=np.array([-1.2*H,-0.75*H,0]), searchTreeBoxMax=np.array([1.2*H,10*H*f,1])
       #                          )
       gContact.SetFrictionPairings(np.eye(1))
       gContact.SetSearchTreeCellSize(numberOfCells=[ssx,ssy,1])
       gContact.SetSearchTreeBox(pMin=np.array([-1.2*H,-0.75*H,0]), pMax=np.array([1.2*H,10*H*f,1]))
   
   mbs.Assemble()
   print("finish gContact")
   
   tEnd = 20
   h= 0.0001*0.25
   simulationSettings = exu.SimulationSettings()
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   #simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.02
   
   simulationSettings.displayComputationTime = True
   #simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.parallel.numberOfThreads = 8
   
   simulationSettings.timeIntegration.newton.numericalDifferentiation.forODE2 = False
   simulationSettings.timeIntegration.newton.useModifiedNewton = False
   
   SC.visualizationSettings.general.graphicsUpdateInterval=10
   SC.visualizationSettings.general.circleTiling=200
   SC.visualizationSettings.general.drawCoordinateSystem=False
   SC.visualizationSettings.loads.show=False
   SC.visualizationSettings.window.renderWindowSize=[1600,1200]
   #SC.visualizationSettings.window.renderWindowSize=[1024,1400]
   SC.visualizationSettings.openGL.multiSampling = 4
   #improved OpenGL rendering
   
   SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
   if False:
       simulationSettings.solutionSettings.recordImagesInterval = 0.025
       SC.visualizationSettings.general.graphicsUpdateInterval=2
   
   
   simulate=True
   if simulate:
       useGraphics = True
       if useGraphics:
           exu.StartRenderer()
           mbs.WaitForUserToContinue()
   
       #initial gContact statistics
       #simulationSettings.timeIntegration.numberOfSteps = 1
       #simulationSettings.timeIntegration.endTime = h
       #mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitEuler)
       #print(gContact)
   
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.timeIntegration.explicitIntegration.computeEndOfStepAccelerations = False #increase performance, accelerations less accurate
       mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.RK67)
       print(gContact)
       #p = mbs.GetNodeOutput(n, variableType=exu.OutputVariableType.Position)
       #print("pEnd =", p[0], p[1])
       print(gContact)
   
       if useGraphics:
           SC.WaitForRenderEngineStopFlag()
           exu.StopRenderer() #safely close rendering window!
   else:
       SC.visualizationSettings.general.autoFitScene = False
       SC.visualizationSettings.general.graphicsUpdateInterval=0.5
       
       sol = LoadSolutionFile('particles.txt')
       mbs.SolutionViewer(sol)


