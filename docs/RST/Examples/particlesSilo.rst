
.. _examples-particlessilo:

****************
particlesSilo.py
****************

You can view and download this file on Github: `particlesSilo.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/particlesSilo.py>`_

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
   from exudyn.utilities import *
   from exudyn.graphicsDataUtilities import *
   
   import numpy as np
   
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #create an environment for mini example
   
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))
   #mLast = mbs.AddMarker(MarkerNodePosition(nodeNumber=nGround))
   
   np.random.seed(1) #always get same results
   
   useGraphics = True
   
   
   useRigidBody = True
   L = 1
   n = 4000*25 #test: 4000
   n = 8000    #fast simulation for testing
   row = 8*2
   a = L*0.5*0.5
   h= 0.0001
   m = 0.05
   ss=16*2
   holeRad = 3*a
   
   if n >= 4000*8:
       a*=0.4
       row=40
       ss = 16*3
       holeRad *= 1.4 #better 1.4 !
   
   if n >= 4000*64:
       a*=0.5
       row*=2
       h *= 0.5
       m *=0.125
       ss*=2
       
   radius = 0.5*a
   t = 0.5*a
   k = 8e4*2 #4e3 needs h=1e-4
   d = 0.001*k*4*0.5*0.2
   
   frictionCoeff = 0.5
   if not useRigidBody:
       frictionCoeff = 0
   
   markerList = []
   radiusList = []
   gDataList = []
   
   # rb = 30*L
   H = 8*L
   Hy=3*L
   
   gContact = mbs.AddGeneralContact()
   gContact.verboseMode = 1
   gContact.SetFrictionPairings(frictionCoeff*np.eye(1))
   gContact.SetSearchTreeCellSize(numberOfCells=[ss,ss,ss])
   #gContact.SetSearchTreeBox(pMin=np.array([-1.2*H,-H,-1.2*H]), pMax=np.array([1.2*H,14*H,1.2*H]))
   #print('treesize=',ssx*ssx*ssy)
   
   
   #%% ground
   LL=6*L
   p0 = np.array([0,0,-0.5*t])
   color4wall = [0.6,0.6,0.6,0.5]
   addNormals = False
   hw=10*a
   gFloor = GraphicsDataOrthoCubePoint(p0,[LL,LL,t],color4steelblue,addNormals)
   gFloorAdd = GraphicsDataOrthoCubePoint(p0+[-0.5*LL,0,0.5*hw],[t,LL,hw],color4wall,addNormals)
   gFloor = MergeGraphicsDataTriangleList(gFloor, gFloorAdd)
   gFloorAdd = GraphicsDataOrthoCubePoint(p0+[ 0.5*LL,0,0.5*hw],[t,LL,hw],color4wall,addNormals)
   gFloor = MergeGraphicsDataTriangleList(gFloor, gFloorAdd)
   gFloorAdd = GraphicsDataOrthoCubePoint(p0+[0,-0.5*LL,0.5*hw],[LL,t,hw],color4wall,addNormals)
   gFloor = MergeGraphicsDataTriangleList(gFloor, gFloorAdd)
   gFloorAdd = GraphicsDataOrthoCubePoint(p0+[0, 0.5*LL,0.5*hw],[LL,t,hw],color4wall,addNormals)
   gFloor = MergeGraphicsDataTriangleList(gFloor, gFloorAdd)
   
   gDataList = [gFloor]
   
   
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0] ))
   mGround = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGround))
   #mGroundC = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
   
   [meshPoints, meshTrigs] = GraphicsData2PointsAndTrigs(gFloor)
   #[meshPoints, meshTrigs] = RefineMesh(meshPoints, meshTrigs) #just to have more triangles on floor
   # [meshPoints, meshTrigs] = RefineMesh(meshPoints, meshTrigs) #just to have more triangles on floor
   gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mGround, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,
       pointList=meshPoints,  triangleList=meshTrigs)
   
   if True: #looses color
       gFloor = GraphicsDataFromPointsAndTrigs(meshPoints, meshTrigs, color=color4wall) #show refined mesh
       gDataList = [gFloor]
   
   
   color4node = color4blue
   print("start create: number of masses =",n)
   for i in range(n):
   
       kk = int(i/int(n/8))
       color4node = color4list[min(kk%9,9)]
   
       if (i%20000 == 0): print("create mass",i)
       offy = 0
       
       iz = int(i/(row*row))
       ix = i%row
       iy = int(i/row)%row
   
       if iz % 2 == 1:
           ix+=0.5
           iy+=0.5
   
       offz = 5*L+0.5*a+iz*a*0.74 #0.70x is limit value!
       offx = -0.6*a-row*0.5*a + (ix+1)*a
       offy = -0.6*a-row*0.5*a + (iy+1)*a
   
       valueRand = np.random.random(1)[0]
       rFact = 0.2 #random part
       gRad = radius*(1-rFact+rFact*valueRand)
       v0 = [0,0,-2]
       pRef = [offx,offy,offz]
       
       if not useRigidBody:
           nMass = mbs.AddNode(NodePoint(referenceCoordinates=pRef,
                                         initialVelocities=v0,
                                         visualization=VNodePoint(show=True,drawSize=2*gRad, color=color4node)))
           
           oMass = mbs.AddObject(MassPoint(physicsMass=m, nodeNumber=nMass,
                                           #visualization=VMassPoint(graphicsData=[gSphere,gSphere2])
                                           # visualization=VMassPoint(graphicsData=gData)
                                           ))
           mThis = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
       else:
           RBinertia = InertiaSphere(m, radius)
           [nMass, oMass] = AddRigidBody(mainSys=mbs, inertia=RBinertia, 
                                   nodeType=exu.NodeType.RotationRotationVector,
                                   position=pRef, velocity=v0,
                                   #graphicsDataList=gList,
                                   )
           mbs.SetNodeParameter(nMass, 'VdrawSize', 2*gRad)
           mbs.SetNodeParameter(nMass, 'Vcolor', color4node)
           mThis = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
   
       mbs.AddLoad(Force(markerNumber=mThis, loadVector= [0,0,-m*9.81]))
   
       gContact.AddSphereWithMarker(mThis, radius=gRad, contactStiffness=k, contactDamping=d, 
                                        frictionMaterialIndex=0)
   
   
   
   if True: #add Silo
       SR = 3.1*L
       SH = 2*L
       SH2 = 1*L #hole
       SR2 = holeRad   #hole
       ST = 0.25*L
       #contour=8*np.array([[0,0.2],[0.3,0.2],[0.5,0.3],[0.7,0.4],[1,0.4],[1,0.]])
       contour=np.array([[0,SR2],[0,SR2+ST],[SH2-ST,SR2+ST],[2*SH2-ST,SR+ST],[2*SH2+SH,SR+ST],
                         [2*SH2+SH,SR],[2*SH2,SR],[SH2,SR2],[0,SR2]])
       contour = list(contour)
       contour.reverse()
       gSilo = GraphicsDataSolidOfRevolution(pAxis=[0,0,3*L], vAxis=[0,0,1],
               contour=contour, color=[0.8,0.1,0.1,0.5], nTiles = 64)
       
       [meshPoints, meshTrigs] = GraphicsData2PointsAndTrigs(gSilo)
       gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mGround, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,
           pointList=meshPoints,  triangleList=meshTrigs)
   
   
   #put here, such that it is transparent in background
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                      visualization=VObjectGround(graphicsData=[gSilo]+gDataList)))
   
   
   mbs.Assemble()
   print("finish gContact")
   
   items=gContact.GetItemsInBox(pMin=[-4,-4,0], pMax=[4,4,20])
   print('n spheres=',len(items['MarkerBasedSpheres'])) 
   
   
   tEnd = 50
   #tEnd = h*100
   simulationSettings = exu.SimulationSettings()
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   #simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.01
   simulationSettings.solutionSettings.outputPrecision = 5 #make files smaller
   simulationSettings.solutionSettings.exportAccelerations = False
   simulationSettings.solutionSettings.exportVelocities = False
   simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/test.txt'
   simulationSettings.displayComputationTime = True
   #simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.parallel.numberOfThreads = 8
   
   simulationSettings.timeIntegration.newton.numericalDifferentiation.forODE2 = False
   simulationSettings.timeIntegration.newton.useModifiedNewton = False
   
   SC.visualizationSettings.general.graphicsUpdateInterval=0.5*4
   SC.visualizationSettings.general.circleTiling=200
   SC.visualizationSettings.general.drawCoordinateSystem=True
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
       simulationSettings.solutionSettings.recordImagesInterval = 0.005
       SC.visualizationSettings.general.graphicsUpdateInterval=2
   
   
   simulate=True
   if simulate:
       if useGraphics:
           SC.visualizationSettings.general.autoFitScene = False
           exu.StartRenderer()
           if 'renderState' in exu.sys:
               SC.SetRenderState(exu.sys['renderState'])
           mbs.WaitForUserToContinue()
   
       #initial gContact statistics
       #simulationSettings.timeIntegration.numberOfSteps = 1
       #simulationSettings.timeIntegration.endTime = h
       #mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitEuler)
       #print(gContact)
   
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitEuler)
       #print(gContact)
       #p = mbs.GetNodeOutput(n, variableType=exu.OutputVariableType.Position)
       #print("pEnd =", p[0], p[1])
       #print(gContact)
   
       if useGraphics:
           SC.WaitForRenderEngineStopFlag()
           exu.StopRenderer() #safely close rendering window!
           
   if not simulate or True:
       SC.visualizationSettings.general.autoFitScene = False
       SC.visualizationSettings.general.graphicsUpdateInterval=0.5
       
       print('load solution file')
       #sol = LoadSolutionFile('solution/test2.txt', safeMode=False)
       sol = LoadSolutionFile('solution/test.txt', safeMode=True)#, maxRows=100)
       print('start SolutionViewer')
       mbs.SolutionViewer(sol)
   


