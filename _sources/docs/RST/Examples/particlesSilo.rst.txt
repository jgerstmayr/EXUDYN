
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
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.graphicsDataUtilities import *
   
   import numpy as np
   
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))
   
   np.random.seed(1) #always get same results
   
   useGraphics = True
   
   useRigidBody = True    #needed for friction
   staticTriangles = True #True speeds up in case of many static objects (silo)
   
   L = 1
   n = 8000 #fast
   # n = 500000
   row = 8*2
   a = L*0.5*0.5*0.75
   stepSize= 0.0001 #Velocity Verlet also works with 2e-4 with 8000 particles
   m = 0.05
   ss=16*3  #the more cells, the more time the search tree needs to build, but contact search is more efficient
   holeRad = 3*a
   SHsilo = 4*L #height of container; adapt to fit all particles
   
   if n >= 4000*8:
       a*=0.4
       m *=0.2
       row=40
       ss = 16*4
       holeRad *= 1.4 #better 1.4 !
   
   if n >= 4000*64:
       #a*=0.75
       SHsilo = 8*L
       row=int(1.3*row)
       stepSize *= 0.25
       ss=100
       
   radius = 0.5*a
   t = 0.5*a
   k = 16e4 
   d = 0.0004*k
   
   frictionCoeff = 0.5
   if not useRigidBody:
       frictionCoeff = 0
   
   markerList = []
   radiusList = []
   gDataList = []
   
   
   gContact = mbs.AddGeneralContact()
   #gContact.verboseMode = 1
   gContact.SetFrictionPairings(frictionCoeff*np.eye(1))
   gContact.SetSearchTreeCellSize(numberOfCells=[ss,ss,ss])
   # gContact.computeExactStaticTriangleBins = False #default = True; speeds up a lot for static triangles
   
   
   #%% ground
   LL=6*L
   p0 = np.array([0,0,-0.5*t])
   color4wall = [0.6,0.6,0.6,0.5]
   addNormals = False
   hw=10*a
   gFloor = graphics.Brick(p0,[LL,LL,t],graphics.color.steelblue,addNormals)
   gFloorAdd = graphics.Brick(p0+[-0.5*LL,0,0.5*hw],[t,LL,hw],color4wall,addNormals)
   gFloor = graphics.MergeTriangleLists(gFloor, gFloorAdd)
   gFloorAdd = graphics.Brick(p0+[ 0.5*LL,0,0.5*hw],[t,LL,hw],color4wall,addNormals)
   gFloor = graphics.MergeTriangleLists(gFloor, gFloorAdd)
   gFloorAdd = graphics.Brick(p0+[0,-0.5*LL,0.5*hw],[LL,t,hw],color4wall,addNormals)
   gFloor = graphics.MergeTriangleLists(gFloor, gFloorAdd)
   gFloorAdd = graphics.Brick(p0+[0, 0.5*LL,0.5*hw],[LL,t,hw],color4wall,addNormals)
   gFloor = graphics.MergeTriangleLists(gFloor, gFloorAdd)
   
   gDataList = [gFloor]
   
   
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0] ))
   mGround = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGround))
   
   [meshPoints, meshTrigs] = graphics.ToPointsAndTrigs(gFloor)
   #[meshPoints, meshTrigs] = RefineMesh(meshPoints, meshTrigs) #just to have more triangles on floor
   gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mGround, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,
       pointList=meshPoints,  triangleList=meshTrigs, staticTriangles=staticTriangles)
   
   if True: #looses color
       gFloor = graphics.FromPointsAndTrigs(meshPoints, meshTrigs, color=color4wall) #show refined mesh
       gDataList = [gFloor]
   
   
   color4node = graphics.color.blue
   print("start create: number of masses =",n)
   for i in range(n):
       kk = int(i/int(n/8))
       color4node = graphics.colorList[min(kk%9,9)]
   
       if (i%20000 == 0 and i>0): print("create mass",i)
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
           dictMass = mbs.CreateRigidBody(
                         inertia=RBinertia, 
                         nodeType=exu.NodeType.RotationRotationVector,
                         referencePosition=pRef, 
                         initialVelocity=v0,
                         returnDict=True)
           [nMass, oMass] = [dictMass['nodeNumber'], dictMass['bodyNumber']]
   
           mbs.SetNodeParameter(nMass, 'VdrawSize', 2*gRad)
           mbs.SetNodeParameter(nMass, 'Vcolor', color4node)
           mThis = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
   
       mbs.AddLoad(Force(markerNumber=mThis, loadVector= [0,0,-m*9.81]))
   
       gContact.AddSphereWithMarker(mThis, radius=gRad, contactStiffness=k, contactDamping=d, 
                                        frictionMaterialIndex=0)
   
   
   
   if True: #add Silo
       SR = 3.1*L
       SH2 = 1*L #hole
       SR2 = holeRad   #hole
       ST = 0.25*L
       #contour=8*np.array([[0,0.2],[0.3,0.2],[0.5,0.3],[0.7,0.4],[1,0.4],[1,0.]])
       contour=np.array([[0,SR2],[0,SR2+ST],[SH2-ST,SR2+ST],[2*SH2-ST,SR+ST],[2*SH2+SHsilo,SR+ST],
                         [2*SH2+SHsilo,SR],[2*SH2,SR],[SH2,SR2],[0,SR2]])
       contour = list(contour)
       contour.reverse()
       gSilo = graphics.SolidOfRevolution(pAxis=[0,0,3*L], vAxis=[0,0,1],
               contour=contour, color=[0.8,0.1,0.1,0.5], nTiles = 64)
       
       [meshPoints, meshTrigs] = graphics.ToPointsAndTrigs(gSilo)
       gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mGround, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,
           pointList=meshPoints,  triangleList=meshTrigs, staticTriangles=staticTriangles)
   
   
   #put here, such that it is transparent in background
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                      visualization=VObjectGround(graphicsData=[gSilo]+gDataList)))
   
   
   mbs.Assemble()
   print("finish gContact")
   # print(gContact.GetPythonObject())
   
   items=gContact.GetItemsInBox(pMin=[-4,-4,0], pMax=[4,4,20])
   print('n spheres=',len(items['MarkerBasedSpheres']), ', ss=',ss)
   
   
   tEnd = 10
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
   simulationSettings.timeIntegration.stepInformation += 32 #show time to go
   simulationSettings.parallel.numberOfThreads = 8 #this should not be higher than the number of real cores (not threads)
   
   simulationSettings.timeIntegration.explicitIntegration.computeEndOfStepAccelerations = False
   simulationSettings.timeIntegration.explicitIntegration.computeMassMatrixInversePerBody = True
   
   SC.visualizationSettings.general.graphicsUpdateInterval=2
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
   
   
   simulate=True
   if simulate:
       if useGraphics:
           SC.visualizationSettings.general.autoFitScene = False
           SC.renderer.Start()
           if 'renderState' in exu.sys:
               SC.renderer.SetState(exu.sys['renderState'])
           #SC.renderer.DoIdleTasks()
   
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
       simulationSettings.timeIntegration.endTime = tEnd
       mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.VelocityVerlet)
       #print(gContact)
   
       if useGraphics:
           SC.renderer.DoIdleTasks()
           SC.renderer.Stop() #safely close rendering window!
           
   if not simulate:
       SC.visualizationSettings.general.autoFitScene = False
       SC.visualizationSettings.general.graphicsUpdateInterval=0.5
       
       print('load solution file')
       #sol = LoadSolutionFile('solution/test2.txt', safeMode=False)
       sol = LoadSolutionFile('solution/test.txt', safeMode=True, verbose = True)#, maxRows=100)
       print('start SolutionViewer')
       mbs.SolutionViewer(sol)
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #timings (best of 3), a=0.25, Core i9-7940X@3.10GHz:
   #1.9.66, ss=32, staticTriangles = False
   # n spheres= 8000 , ss= 16
   # +++++++++++++++++++++++++++++++
   # EXUDYN V1.9.68.dev1 solver: explicit time integration (VelocityVerlet)
   # Start multi-threading with 8 threads
   # STEP526, t = 0.0526s, timeToGo = 5.61318s, Nit/step = 0
   # STEP1031, t = 0.1031s, timeToGo = 3.75953s, Nit/step = 0
   # STEP1530, t = 0.153s, timeToGo = 1.84418s, Nit/step = 0
   # STEP2000, t = 0.2s, timeToGo = 2.26949e-13s, Nit/step = 0
   # Solver terminated successfully after 7.96149 seconds.
   # ====================
   # CPU-time statistics:
   #   total time   = 7.96 seconds
   #   measured time= 7.65 seconds (=96.1%) 
   #   non-zero timer [__ sub-timer]:
   #   newtonIncrement   = 2.97%
   #   integrationFormula= 6.03%
   #   ODE2RHS           = 84.2%
   #   writeSolution     = 5.01%
   #   overhead          = 1.74%
   #   visualization/user= 0.00322%
   # special timers:
   #   Contact:BoundingBoxes = 0.92391 (12.1%)s
   #   Contact:SearchTree = 0.59905 (7.83%)s
   #   Contact:Overall = 5.1361 (67.2%)s
   
   # n spheres= 8000 , ss= 32
   # +++++++++++++++++++++++++++++++
   # EXUDYN V1.9.68.dev1 solver: explicit time integration (VelocityVerlet)
   # Start multi-threading with 8 threads
   # STEP562, t = 0.0562s, timeToGo = 5.12578s, Nit/step = 0
   # STEP1126, t = 0.1126s, timeToGo = 3.10595s, Nit/step = 0
   # STEP1688, t = 0.1688s, timeToGo = 1.10932s, Nit/step = 0
   # STEP2000, t = 0.2s, timeToGo = 2.03936e-13s, Nit/step = 0
   # Solver terminated successfully after 7.15063 seconds.
   # ====================
   # CPU-time statistics:
   #   total time   = 7.15 seconds
   #   measured time= 6.86 seconds (=95.9%) 
   #   non-zero timer [__ sub-timer]:
   #   newtonIncrement   = 3.16%
   #   integrationFormula= 6.44%
   #   ODE2RHS           = 84.1%
   #   writeSolution     = 4.47%
   #   overhead          = 1.81%
   #   visualization/user= 0.00365%
   # special timers:
   #   Contact:BoundingBoxes = 0.89462 (13%)s
   #   Contact:SearchTree = 1.142 (16.7%)s
   #   Contact:Overall = 4.5078 (65.8%)s
   
   # n spheres= 8000 , ss= 48
   # +++++++++++++++++++++++++++++++
   # EXUDYN V1.9.68.dev1 solver: explicit time integration (VelocityVerlet)
   # Start multi-threading with 8 threads
   # STEP440, t = 0.044s, timeToGo = 7.10356s, Nit/step = 0
   # STEP861, t = 0.0861s, timeToGo = 5.29512s, Nit/step = 0
   # STEP1294, t = 0.1294s, timeToGo = 3.27367s, Nit/step = 0
   # STEP1707, t = 0.1707s, timeToGo = 1.37385s, Nit/step = 0
   # STEP2000, t = 0.2s, timeToGo = 2.71236e-13s, Nit/step = 0
   # Solver terminated successfully after 9.50154 seconds.
   # ====================
   # CPU-time statistics:
   #   total time   = 9.5 seconds
   #   measured time= 9.17 seconds (=96.5%) 
   #   non-zero timer [__ sub-timer]:
   #   newtonIncrement   = 2.45%
   #   integrationFormula= 4.94%
   #   ODE2RHS           = 87.5%
   #   writeSolution     = 3.6%
   #   overhead          = 1.54%
   #   visualization/user= 0.00405%
   # special timers:
   #   Contact:BoundingBoxes = 0.92908 (10.1%)s
   #   Contact:SearchTree = 2.3468 (25.6%)s
   #   Contact:Overall = 6.676 (72.8%)s
   
   
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #1.9.66, staticTriangles = True
   
   # n spheres= 8000 , ss= 16
   # +++++++++++++++++++++++++++++++
   # EXUDYN V1.9.68.dev1 solver: explicit time integration (VelocityVerlet)
   # Start multi-threading with 8 threads
   # WARNING: VelocityVerlet: still under development
   # STEP526, t = 0.0526s, timeToGo = 5.60837s, Nit/step = 0
   # STEP1091, t = 0.1091s, timeToGo = 3.33273s, Nit/step = 0
   # STEP1655, t = 0.1655s, timeToGo = 1.25102s, Nit/step = 0
   # STEP2000, t = 0.2s, timeToGo = 2.0821e-13s, Nit/step = 0
   # Solver terminated successfully after 7.30395 seconds.
   # ====================
   # CPU-time statistics:
   #   total time   = 7.3 seconds
   #   measured time= 7.01 seconds (=96%) 
   #   non-zero timer [__ sub-timer]:
   #   newtonIncrement   = 3.19%
   #   integrationFormula= 6.58%
   #   ODE2RHS           = 83.6%
   #   writeSolution     = 4.71%
   #   overhead          = 1.93%
   #   visualization/user= 0.003%
   # special timers:
   #   Contact:BoundingBoxes = 0.98863 (14.1%)s
   #   Contact:SearchTree = 0.4626 (6.6%)s
   #   Contact:Overall = 4.3393 (61.9%)s
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #update to 1.9.68 (exact bin computation for triangles)
   # start create: number of masses = 8000
   # create mass 0
   # finish gContact
   # n spheres= 8000 , ss= 32
   # +++++++++++++++++++++++++++++++
   # EXUDYN V1.9.68.dev1 solver: explicit time integration (VelocityVerlet)
   # Start multi-threading with 8 threads
   # STEP639, t = 0.0639s, timeToGo = 4.26062s, Nit/step = 0
   # STEP1309, t = 0.1309s, timeToGo = 2.11214s, Nit/step = 0
   # STEP2000, t = 0.2s, timeToGo = 1.70989e-13s, Nit/step = 0
   # Solver terminated successfully after 5.99597 seconds.
   # ====================
   # CPU-time statistics:
   #   total time   = 6 seconds
   #   measured time= 5.72 seconds (=95.4%) 
   #   non-zero timer [__ sub-timer]:
   #   newtonIncrement   = 3.65%
   #   integrationFormula= 7.59%
   #   ODE2RHS           = 81.1%
   #   writeSolution     = 5.57%
   #   overhead          = 2.07%
   #   visualization/user= 0.00279%
   # special timers:
   #   Contact:BoundingBoxes = 0.86635 (15.1%)s
   #   Contact:SearchTree = 0.59121 (10.3%)s
   #   Contact:Overall = 3.3974 (59.4%)s
   
   
   # runfile('C:/DATA/cpp/EXUDYN_git/main/pythonDev/Examples/particlesSilo.py', wdir='C:/DATA/cpp/EXUDYN_git/main/pythonDev/Examples')
   # start create: number of masses = 8000
   # create mass 0
   # finish gContact
   # n spheres= 8000 , ss= 48
   # +++++++++++++++++++++++++++++++
   # EXUDYN V1.9.68.dev1 solver: explicit time integration (VelocityVerlet)
   # Start multi-threading with 8 threads
   # STEP517, t = 0.0517s, timeToGo = 5.74121s, Nit/step = 0
   # STEP1051, t = 0.1051s, timeToGo = 3.614s, Nit/step = 0
   # STEP1601, t = 0.1601s, timeToGo = 1.49891s, Nit/step = 0
   # STEP2000, t = 0.2s, timeToGo = 2.14357e-13s, Nit/step = 0
   # Solver terminated successfully after 7.51458 seconds.
   # ====================
   # CPU-time statistics:
   #   total time   = 7.51 seconds
   #   measured time= 7.21 seconds (=95.9%) 
   #   non-zero timer [__ sub-timer]:
   #   newtonIncrement   = 3.12%
   #   integrationFormula= 6.21%
   #   ODE2RHS           = 84.7%
   #   writeSolution     = 4.11%
   #   overhead          = 1.84%
   #   visualization/user= 0.00239%
   # special timers:
   #   Contact:BoundingBoxes = 0.88801 (12.3%)s
   #   Contact:SearchTree = 0.93017 (12.9%)s
   #   Contact:Overall = 4.8152 (66.8%)s
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #large test:
   # 500000 particles, stepSize=2.5e-5
   # STEP40046, t = 1.00115s, timeToGo = 5.81135 days, tCPU=2.84971h, Nit/step = 0
   # STEP121643, t = 3.04107s, timeToGo = 6.78003 days, tCPU=10.5378h, Nit/step = 0
   # STEP121644 (stopped), t = 3.04107s, tCPU=37936.2s, Nit/step = 0
   # solver stopped by user after 37935.8 seconds.
   # ====================
   # CPU-time statistics:
   #   total time   = 3.79e+04 seconds
   #   measured time= 3.63e+04 seconds (=95.8%) 
   #   non-zero timer [__ sub-timer]:
   #   newtonIncrement   = 3.02%
   #   integrationFormula= 5.06%
   #   ODE2RHS           = 86.1%
   #   writeSolution     = 0.974%
   #   overhead          = 1.18%
   #   visualization/user= 3.61%
   # special timers:
   #   Contact:BoundingBoxes = 3156.8 (8.68%)s
   #   Contact:SearchTree = 3350.5 (9.22%)s
   #   Contact:Overall = 25633 (70.5%)s


