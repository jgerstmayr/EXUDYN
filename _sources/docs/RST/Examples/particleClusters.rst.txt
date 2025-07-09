
.. _examples-particleclusters:

*******************
particleClusters.py
*******************

You can view and download this file on Github: `particleClusters.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/particleClusters.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  test with clusters of spheres attached to rigid body
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-12-23
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
   from math import sqrt
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   np.random.seed(1) #always get same results
   
   L = 1
   # n = 16000 #*8*4 #32*8*8
   a = 0.2*L
   # radius = 0.5*0.125
   # rows = 14
   # rows2 = 40*2
   # mu = 0.1
   m = 0.0125
   k = 4e3 #2e4
   d = 0.0004*k
   # ssx = 32 #search tree size
   # ssy = 16 #search tree size
   # ssz = 12 #search tree size
   LL=12*L
   hWall = 8*L
   wWall = 4*L
   
   useClusters = True
   useNodeMarker = not useClusters
   n = 200#0*3
   rows = 7
   rows2 = 12
   radius = 0.5*0.25
   mu = 0.4*3
   
   ssx = 16 #search tree size
   ssy = 10 #search tree size
   ssz = 8 #search tree size
   hWall = 12*L
   drx = radius*0.75 #for cluster
   drz = radius*0.5 #for cluster
   
   markerList = []
   radiusList = []
   p0 = np.array([0.45*LL,-4*radius,0])
   color4wall = [0.9,0.9,0.7,0.25]
   gFloor = graphics.Brick(p0,[LL,a,wWall],graphics.color.steelblue)
   gFloorAdd = graphics.Brick(p0+[-0.5*LL,0.5*hWall,0],[a,hWall,wWall],color4wall)
   gFloor = graphics.MergeTriangleLists(gFloor, gFloorAdd)
   gFloorAdd = graphics.Brick(p0+[ 0.5*LL,0.5*hWall,0],[a,hWall,wWall],color4wall)
   gFloor = graphics.MergeTriangleLists(gFloor, gFloorAdd)
   gFloorAdd = graphics.Brick(p0+[ 0,0.5*hWall,-0.5*wWall],[LL,hWall,a],color4wall)
   gFloor = graphics.MergeTriangleLists(gFloor, gFloorAdd)
   gFloorAdd = graphics.Brick(p0+[ 0,0.5*hWall, 0.5*wWall],[LL,hWall,a],color4wall)
   gFloor = graphics.MergeTriangleLists(gFloor, gFloorAdd)
   
   [meshPoints, meshTrigs] = graphics.ToPointsAndTrigs(gFloor)
   
   gDataList = [gFloor]
   
   
   rb = radius
   H = 8*L
   RR = 2*radius*sqrt(0.5)-1e-12
   pos0 = [0.1,-RR*2,0]
   pos1 = [5*radius,-RR*2,0]
   #posList=[pos0, pos1]
   #posList=[pos0]
   posList=[]
   for pos in posList:
       #gDataList += [{'type':'Circle','position':pos,'radius':rb, 'color':graphics.color.grey}]
       gDataList += [graphics.Sphere(point=pos, radius=rb, color= graphics.color.grey, nTiles=40)]
       #gDataList += [GraphicsDataRectangle(-1.2*H,-H*0.75,1.2*H,10*H,color=graphics.color.red)]
       nMass = mbs.AddNode(NodePointGround(referenceCoordinates=pos))
       #oMass = mbs.AddObject(MassPoint(physicsMass=m, nodeNumber=nMass))
       mThis = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
       markerList += [mThis]
       radiusList += [rb]
   
   
   
   #ns = 20
   minP = np.array([1e10,1e10,1e10])
   maxP = -minP
   gContact = mbs.AddGeneralContact()
   
   lastColor=[0,0,0,0]    
   for i in range(n):
   
       color4node = graphics.colorList[min(9, int((int(i/rows)%rows2 * 10)/rows2))]
       if lastColor != color4node:
           lastColor = color4node
           gList = [graphics.Sphere(point=[0,0,0], radius=radius, color= color4node, nTiles=8)]
           if useClusters:
               gList = []
               gList += [graphics.Sphere(point=[   0,0,-drz], radius=radius, color= color4node, nTiles=8)]
               gList += [graphics.Sphere(point=[   0,0, drz], radius=radius, color= color4node, nTiles=8)]
               gList += [graphics.Sphere(point=[-drx,0,-drz], radius=radius, color= color4node, nTiles=8)]
               gList += [graphics.Sphere(point=[ drx,0,-drz], radius=radius, color= color4node, nTiles=8)]
               gList += [graphics.Sphere(point=[ drx,0, drz], radius=radius, color= color4node, nTiles=8)]
               gList += [graphics.Sphere(point=[-drx,0, drz], radius=radius, color= color4node, nTiles=8)]
               #gList = [graphics.Brick(centerPoint=[0,0,0], size=[2*radius+2*drx,2*radius,2*radius+2*drz], color= color4node)]
   
   
       dd = 2.5*radius
       ox=(int(i/rows)%2)*0.5*dd#rows
       pRef = [(i%rows)*(dd+2*drx)+ox, (int(i/rows)%rows2)*0.8*dd, -wWall*0.5+dd+ox+int(i/(rows*rows2))*(dd+drz)]
   
       minP = np.minimum(minP, pRef)
       maxP = np.maximum(maxP, pRef)
       v0 = [0,-10*0.1,0]
       rot0 = np.eye(3)
       forceY = -m*9.81*0.25
   
       RBinertia = InertiaSphere(m, radius*1)
       isCube = (i == n-1)
       cubeX = 8*radius
       cubeY = 8*radius
       cubeZ = 8*radius
       if isCube:
           pRef = [10*L, 2*L,-wWall*0.5+cubeZ]
           rot0=RotationMatrixZ(0.2) @ RotationMatrixX(0.1)
           RBinertia = InertiaSphere(10*m, radius*8)
           forceY *= 10
           gCube=graphics.Brick(centerPoint=[0,0,0], size=[2*radius+cubeX,2*radius+cubeY,2*radius+cubeZ], color= graphics.color.steelblue)
           gList = [gCube]
   
           if isCube:
               for ix in range(2):
                   for iy in range(2):
                       for iz in range(2):
                           gList += [graphics.Sphere(point=[ cubeX*(ix-0.5), cubeY*(iy-0.5), cubeZ*(iz-0.5) ], radius=radius, color= color4node, nTiles=8)]
   
       dictMass = mbs.CreateRigidBody(
                     inertia=RBinertia, 
                     nodeType=exu.NodeType.RotationRotationVector,
                     referencePosition=pRef, 
                     initialVelocity=v0,
                     referenceRotationMatrix=rot0,
                     graphicsDataList=gList,
                     returnDict=True)
       [nMass, oMass] = [dictMass['nodeNumber'], dictMass['bodyNumber']]
       
       mbs.SetNodeParameter(nMass, 'VdrawSize', radius*2)
       mbs.SetNodeParameter(nMass, 'Vcolor', color4node)
   
       if i == n-2:
           nMassOutput = nMass #for solution output
   
   
       mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
       if useNodeMarker:
           markerList += [mNode]
       elif not useClusters:
           mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass, localPosition=[0,0,0]))
           markerList += [mBody]
       else:
           if isCube:
               [meshPointsCube, meshTrigsCube] = graphics.ToPointsAndTrigs(gCube)
               mCube = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass, localPosition=[0,0,0]))
               gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mCube, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,
                                                   pointList=meshPointsCube,  triangleList=meshTrigsCube)
               for ix in range(2):
                   for iy in range(2):
                       for iz in range(2):
                           mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass, localPosition=[ cubeX*(ix-0.5), cubeY*(iy-0.5), cubeZ*(iz-0.5)]))
                           markerList += [mBody]
           else:
               mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass, localPosition=[   0, 0,-drz]))
               markerList += [mBody]
               mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass, localPosition=[   0, 0, drz]))
               markerList += [mBody]
               mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass, localPosition=[-drx, 0,-drz]))
               markerList += [mBody]
               mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass, localPosition=[ drx, 0,-drz]))
               markerList += [mBody]
               mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass, localPosition=[ drx, 0, drz]))
               markerList += [mBody]
               mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass, localPosition=[-drx, 0, drz]))
               markerList += [mBody]
   
   
       #mThis = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
       #if (i < 2): 
       mbs.AddLoad(Force(markerNumber=mNode, loadVector= [0,forceY,0]))
   
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                       visualization=VObjectGround(graphicsData=gDataList)))
   mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
   
   gContact.verboseMode = 1
   #gContact.sphereSphereContact = False
   gContact.frictionProportionalZone = 1e-6
   #[meshPoints,  meshTrigs] = RefineMesh(meshPoints,  meshTrigs)
   gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mGround, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,
       pointList=meshPoints,  triangleList=meshTrigs)
   
   for i in range(len(markerList)):
       m = markerList[i]
       gContact.AddSphereWithMarker(m, radius=radius, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)
   
   #gContact.FinalizeContact(mbs, searchTreeSize=np.array([ssx,ssy,1]), frictionPairingsInit=np.eye(1), 
   #                         searchTreeBoxMin=np.array([-H,-H,-H]), searchTreeBoxMax=np.array([H,H,H])
   #                         )
   gContact.SetFrictionPairings(mu*np.eye(1))
   gContact.SetSearchTreeCellSize(numberOfCells=[ssx,ssy,ssz])
   #gContact.SetSearchTreeBox(pMin=np.array([-H,-H,-0.2*H]), pMax=np.array([H,H,0.2*H]))
   
   sNode = mbs.AddSensor(SensorNode(nodeNumber=nMassOutput, fileName='solution/particlesNode.txt',
                            outputVariableType=exu.OutputVariableType.Position))
   
   mbs.Assemble()
   
   tEnd = 1
   h= 0.0001
   
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/test.txt'
   
   simulationSettings.displayComputationTime = True
   #simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.parallel.numberOfThreads = 8
   
   #SPEEDUPs:
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.timeIntegration.explicitIntegration.computeEndOfStepAccelerations = False
   simulationSettings.solutionSettings.solutionWritePeriod = 0.02
   simulationSettings.solutionSettings.outputPrecision = 5 #make files smaller
   simulationSettings.solutionSettings.exportAccelerations = False
   simulationSettings.solutionSettings.exportVelocities = False
   
   SC.visualizationSettings.general.graphicsUpdateInterval=1
   SC.visualizationSettings.general.circleTiling=200
   SC.visualizationSettings.general.drawCoordinateSystem=True
   SC.visualizationSettings.general.drawCoordinateSystem=False
   SC.visualizationSettings.loads.show=False
   SC.visualizationSettings.nodes.showBasis = False
   SC.visualizationSettings.nodes.basisSize = radius*2
   
   SC.visualizationSettings.window.renderWindowSize=[1600,1200]
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
           SC.renderer.Start()
           SC.renderer.DoIdleTasks()
   
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitEuler)
   
       p = mbs.GetSensorValues(sNode)
       #after one second (200 particles, h=0.0001):
       #p= [0.854546923638082, 0.4801722062275968, -0.7822195683611741]
       print("p=", list(p))
       
       if useGraphics:
           SC.renderer.DoIdleTasks()
           SC.renderer.Stop() #safely close rendering window!
       
       if True:
           
           mbs.PlotSensor(sensorNumbers=[sNode,sNode,sNode], components=[0,1,2], closeAll=True)
   else:
       SC.visualizationSettings.general.autoFitScene = False
       SC.visualizationSettings.general.graphicsUpdateInterval=0.1
       
       sol = LoadSolutionFile('particles.txt')
       mbs.SolutionViewer(sol)


