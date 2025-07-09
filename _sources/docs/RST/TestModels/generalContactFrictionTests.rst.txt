
.. _testmodels-generalcontactfrictiontests:

******************************
generalContactFrictionTests.py
******************************

You can view and download this file on Github: `generalContactFrictionTests.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/generalContactFrictionTests.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  test friction of spheres and spheres-trigs
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-12-06
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   import numpy as np
   
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
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))
   
   #isPerformanceTest = exudynTestGlobals.isPerformanceTest
   #useGraphics = False
   # isPerformanceTest = True
   # tEnd = 0.1
   # if isPerformanceTest: tEnd *= 0.5
   
   #%%+++++++++++++++++++++++++++++++++
   #sphere-sphere with coordinate constraints, prestressed; fixed torque on one side, linear increasing torque on other side
   #sphere on ground, rolling
   #cube on ground, sliding (f=[f, f*mu*t, 0]), tangential force changing
   #cube on ground with initial velocity
   #cube-cube contact (meshed)
   
   
   L = 1 #surrounding
   a = 0.1 #base dimention of objects
   r = 0.5*a #radius
   t = 0.25*a #thickness
   
   #contact coefficients:
   mu = 0.8      #dry friction
   m = 0.025     #mass
   k = 1e3       #(linear) normal contact stiffness
   d = 2*1e-4*k  #(linear) contact damping
   gFact = 10
   g = [0,0,-gFact]
   
   gContact = mbs.AddGeneralContact()
   gContact.verboseMode = 1
   #gContact.sphereSphereContact = False
   gContact.frictionProportionalZone = 1e-3
   #gContact.excludeDuplicatedTrigSphereContactPoints = False
   fricMat = mu*np.eye(2)
   fricMat[0,1] = 0.2
   fricMat[1,0] = 0.2
   gContact.SetFrictionPairings(fricMat)
   gContact.SetSearchTreeCellSize(numberOfCells=[4,4,4])
   
   #%% ground
   p0 = np.array([0,0,-0.5*t])
   color4wall = [0.9,0.9,0.7,0.5]
   addNormals = False
   gFloor = graphics.Brick(p0,[L,L,t],graphics.color.steelblue,addNormals)
   gFloorAdd = graphics.Brick(p0+[-0.5*L,0,a],[t,L,2*a],color4wall,addNormals)
   gFloor = graphics.MergeTriangleLists(gFloor, gFloorAdd)
   gFloorAdd = graphics.Brick(p0+[ 0.5*L,0,a],[t,L,2*a],color4wall,addNormals)
   gFloor = graphics.MergeTriangleLists(gFloor, gFloorAdd)
   gFloorAdd = graphics.Brick(p0+[0,-0.5*L,a],[L,t,2*a],color4wall,addNormals)
   gFloor = graphics.MergeTriangleLists(gFloor, gFloorAdd)
   gFloorAdd = graphics.Brick(p0+[0, 0.5*L,a],[L,t,2*a],color4wall,addNormals)
   gFloor = graphics.MergeTriangleLists(gFloor, gFloorAdd)
   
   bb = 0.75*a
   bh = 0.25*a
   p1 = np.array([0.5*L,0.5*L,0])
   gFloorAdd = graphics.Brick(p1+[-bb*3, -bb, 0.5*bh],[bb,bb,bh],color4wall,addNormals)
   gFloor = graphics.MergeTriangleLists(gFloor, gFloorAdd)
   gFloorAdd = graphics.Brick(p1+[-bb*2, -bb, 1.5*bh],[bb,bb,bh],color4wall,addNormals)
   gFloor = graphics.MergeTriangleLists(gFloor, gFloorAdd)
   gFloorAdd = graphics.Brick(p1+[-bb*1, -bb, 2.5*bh],[bb,bb,bh],color4wall,addNormals)
   gFloor = graphics.MergeTriangleLists(gFloor, gFloorAdd)
   
   gDataList = [gFloor]
   
   
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0] ))
   mGround = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGround))
   mGroundC = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
   
   [meshPoints, meshTrigs] = graphics.ToPointsAndTrigs(gFloor)
   #[meshPoints, meshTrigs] = RefineMesh(meshPoints, meshTrigs) #just to have more triangles on floor
   # [meshPoints, meshTrigs] = RefineMesh(meshPoints, meshTrigs) #just to have more triangles on floor
   gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mGround, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,
       pointList=meshPoints,  triangleList=meshTrigs)
   
   if True: #looses color
       gFloor = graphics.FromPointsAndTrigs(meshPoints, meshTrigs, color=color4wall) #show refined mesh
       gDataList = [gFloor]
   
   evalNodes = [] #collect nodes that are evaluated for test
   #%%++++++++++++++++++++++++++++++++++++++++++++
   #free rolling sphere:
   gList = [graphics.Sphere(point=[0,0,0], radius=r, color= graphics.color.red, nTiles=24)]
   omega0 = -4.*np.array([5,1.,0.])
   pRef = [-0.4*L,-0.4*L,r-0*m*gFact/k]
   RBinertia = InertiaSphere(m, r)
   dictMass = mbs.CreateRigidBody(
                         inertia=RBinertia, 
                         nodeType=exu.NodeType.RotationRotationVector,
                         referencePosition=pRef,
                         initialVelocity=-np.cross([0,0,r], omega0),
                         referenceRotationMatrix=RotationMatrixX(0.),
                         initialAngularVelocity=omega0,
                         # gravity=g,
                         graphicsDataList=gList,
                         returnDict=True)
   [nMass, oMass] = [dictMass['nodeNumber'], dictMass['bodyNumber']]
   
   
   nNode0 = nMass
   mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
   mbs.AddLoad(Force(markerNumber=mNode, loadVector= [0,0,-k*r*0.01])) #==> uz = 2*r*0.01
   exu.Print('expect u0z=',2*r*0.01)
   gContact.AddSphereWithMarker(mNode, radius=r, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)
   if useGraphics:
       sNode0 = mbs.AddSensor(SensorNode(nodeNumber=nNode0, storeInternal=True, #fileName='solution/contactNode0.txt',
                                         outputVariableType=exu.OutputVariableType.Displacement))
       vNode0 = mbs.AddSensor(SensorNode(nodeNumber=nNode0, storeInternal=True, #fileName='solution/contactNode0Vel.txt',
                                         outputVariableType=exu.OutputVariableType.Velocity))
   evalNodes += [nMass] 
   
   #%%++++++++++++++++++++++++++++++++++++++++++++
   #free rolling sphere at midpoint, many triangles in close contact; slowly go through critical points:
   gList = [graphics.Sphere(point=[0,0,0], radius=r, color= graphics.color.yellow, nTiles=24)]
   omega0 = -1e-12*np.array([1,0.1,0.])
   pRef = [1e-15,-1e-14,r-2*m*gFact/k]
   RBinertia = InertiaSphere(m, r)
   dictMass = mbs.CreateRigidBody(
                         inertia=RBinertia, 
                         nodeType=exu.NodeType.RotationRotationVector,
                         referencePosition=pRef,
                         initialVelocity=-np.cross([0,0,r], omega0),
                         referenceRotationMatrix=RotationMatrixX(0.),
                         initialAngularVelocity=omega0,
                         gravity=g,
                         graphicsDataList=gList,
                         returnDict=True)
   [nMass, oMass] = [dictMass['nodeNumber'], dictMass['bodyNumber']]
   
   nNode1 = nMass
   mNode1 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
   #mbs.AddLoad(Force(markerNumber=mNode1, loadVector= [0,0,-k*r*0.01])) #==> uz = 2*r*0.01
   #exu.Print('expect u0z=',2*r*0.01)
   gContact.AddSphereWithMarker(mNode1, radius=r, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)
   
   sNode1 = mbs.AddSensor(SensorNode(nodeNumber=nNode1, storeInternal=True, #fileName='solution/contactNode1.txt',
                                     outputVariableType=exu.OutputVariableType.Displacement))
   # vNode1 = mbs.AddSensor(SensorNode(nodeNumber=nNode0, storeInternal=True, #fileName='solution/contactNode0Vel.txt',
   #                                   outputVariableType=exu.OutputVariableType.Velocity))
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++
   #fixed pressure tests:
   pf = np.array([-1.2*L,0,0])
   nGroundF = mbs.AddNode(NodePointGround(referenceCoordinates=pf ))
   mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGroundF))
   gContact.AddSphereWithMarker(mNode, radius=r, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)
   gDataList += [graphics.Sphere(point=pf, radius=r, color= graphics.color.grey, nTiles=24)]
   
   gList = [graphics.Sphere(point=[0,0,0], radius=r, color= graphics.color.lightgreen, nTiles=24)]
   
   pRef = pf+[0,2*r,0] #[-0.4*L,-0.4*L,r-m*gFact/k]
   RBinertia = InertiaSphere(m, r)
   dictF = mbs.CreateRigidBody(
                         inertia=RBinertia,  
                         nodeType=exu.NodeType.RotationRotationVector,
                         referencePosition=pRef,
                         referenceRotationMatrix=RotationMatrixX(0.),
                         graphicsDataList=gList,
                         returnDict=True)
   [nMassF, oMassF] = [dictF['nodeNumber'], dictF['bodyNumber']]
   
   mC = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMassF, coordinate=0))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundC, mC]))
   
   mNodeF = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMassF))
   mbs.AddLoad(Force(markerNumber=mNodeF, loadVector= [0,-k*r*0.1,0])) #==> u =  k*r*0.1/(0.5*k) = 2*r*0.1
   exu.Print('expect uFy=',2*r*0.1)
   gContact.AddSphereWithMarker(mNodeF, radius=r, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)
   if useGraphics:
       sNodeF = mbs.AddSensor(SensorNode(nodeNumber=nMassF, storeInternal=True, #fileName='solution/contactNodeF.txt',
                                         outputVariableType=exu.OutputVariableType.Displacement))
   evalNodes += [nMassF] 
   
   #%%++++++++++++++++++++++++++++++++++++++++++++
   # sliding between spheres:
   pr = np.array([-1.2*L,0.5*L,0])
   nGroundF2 = mbs.AddNode(NodePointGround(referenceCoordinates=pr ))
   mNode2 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGroundF2))
   gContact.AddSphereWithMarker(mNode2, radius=r, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)
   gDataList += [graphics.Sphere(point=pr, radius=r, color= graphics.color.lightgrey, nTiles=24)]
   
   gList = [graphics.Sphere(point=[0,0,0], radius=r, color= graphics.color.lightred, nTiles=24)]
   
   dRol = r*0.01
   pRef = pr+[0,2*r-2*dRol,0] #force=k*r*0.01
   fRol = k*dRol
   exu.Print('force rolling=', fRol, ', torque=', fRol*mu*r)
   RBinertia = InertiaSphere(m, r)
   dictR = mbs.CreateRigidBody(
                         inertia=RBinertia, 
                         nodeType=exu.NodeType.RotationRotationVector,
                         referencePosition=pRef,
                         referenceRotationMatrix=RotationMatrixX(0.),
                         graphicsDataList=gList,
                         returnDict=True)
   [nMassR, oMassR] = [dictR['nodeNumber'], dictR['bodyNumber']]
   
   
   
   
   mC = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMassR, coordinate=0))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundC, mC]))
   mC = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMassR, coordinate=1))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundC, mC]))
   mC = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMassR, coordinate=2))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundC, mC]))
   
   mNodeR = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMassR))
   
   def UFtorque(mbs, t, loadVector):
       torque = 10*t*fRol*mu*r
       if t > 0.3:
           torque = 0
       return [torque,0,0]
   mbs.AddLoad(Torque(markerNumber=mNodeR, loadVectorUserFunction=UFtorque, 
                      loadVector= [1,0,0])) #==> u =  k*r*0.1/(0.5*k) = 2*r*0.1
   
   gContact.AddSphereWithMarker(mNodeR, radius=r, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)
   if useGraphics:
       sNodeR = mbs.AddSensor(SensorNode(nodeNumber=nMassR, storeInternal=True, #fileName='solution/contactNodeR.txt',
                                         outputVariableType=exu.OutputVariableType.Rotation))
       vNodeR = mbs.AddSensor(SensorNode(nodeNumber=nMassR, storeInternal=True, #fileName='solution/contactNodeRvel.txt',
                                         outputVariableType=exu.OutputVariableType.AngularVelocity))
   
   evalNodes += [nMassR] 
   
   #%%++++++++++++++++++++++++++++++++++++++++++++
   #sphere on stairs
   #%%++++++++++++++++++++++++++++++++++++++++++++
   #free rolling sphere at midpoint, many triangles in close contact; slowly go through critical points:
   gList = [graphics.Sphere(point=[0,0,0], radius=0.5*r, color= graphics.color.yellow, nTiles=24)]
   omega0 = np.array([-0.05,-5,0.])
   pRef = [0.5*L-1.45*bb, 0.5*L-1.20*bb, 3*bh+0.5*r-2*m*gFact/k] #[0.5*L-1.45*bb, 0.5*L-1.40*bb, ..] goes to edge
   RBinertia = InertiaSphere(m, 0.5*r)
   dictStair = mbs.CreateRigidBody( #note that this node causes inaccuracies/repeatability issues in test suite!
                 inertia=RBinertia, 
                 nodeType=exu.NodeType.RotationRotationVector,
                 referencePosition=pRef,
                 initialVelocity=-np.cross([0,0,0.5*r], omega0),
                 referenceRotationMatrix=RotationMatrixX(0.),
                 initialAngularVelocity=omega0,
                 gravity=g,
                 graphicsDataList=gList,
                 returnDict=True)
   [nMassStair, oMassStair] = [dictStair['nodeNumber'], dictStair['bodyNumber']]
   
   nNode3 = nMassStair
   mNode3 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMassStair))
   gContact.AddSphereWithMarker(mNode3, radius=0.5*r, contactStiffness=k, contactDamping=20*d, frictionMaterialIndex=0)
   
   if useGraphics:
       sNode3 = mbs.AddSensor(SensorNode(nodeNumber=nNode3, storeInternal=True, #fileName='solution/contactNode3.txt',
                                         outputVariableType=exu.OutputVariableType.Displacement))
   evalNodes += [nMassStair] 
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #contact of cube with ground
   tTrig = 0.25*r #size of contact points on mesh ('thickness')
   gCube = graphics.Brick(size=[3*r,2*r,r], color= graphics.color.steelblue,addNormals=addNormals)
   [meshPoints, meshTrigs] = graphics.ToPointsAndTrigs(gCube)
   
   #for tests, 1 refinement!
   [meshPoints, meshTrigs] = RefineMesh(meshPoints, meshTrigs) #just to have more triangles on floor
   # exu.Print("n points=",len(meshPoints))
   # [meshPoints, meshTrigs] = RefineMesh(meshPoints, meshTrigs) #just to have more triangles on floor
   # exu.Print("==> n points refined=",len(meshPoints))
   # refinements give 26,98,386 points!
   
   [meshPoints2, meshTrigs2] = ShrinkMeshNormalToSurface(meshPoints, meshTrigs, tTrig)
   
   #add mesh to visualization
   gCube = graphics.FromPointsAndTrigs(meshPoints, meshTrigs, color=graphics.color.steelblue) #show refined mesh
   gList = [gCube]
   
   #add points for contact to visualization (shrinked)
   for p in meshPoints2:
       gList += [graphics.Sphere(point=p, radius=tTrig, color=graphics.color.red)]
       
   pRef = [0.5*L-2*r, 0.25*L, 0.5*r+1.5*tTrig]
   v0 = np.array([-2,0,0])
   RBinertia = InertiaCuboid(density=m/(r*2*r*3*r), sideLengths=[3*r,2*r,r])
   dictCube0 = mbs.CreateRigidBody(
                 inertia=RBinertia, 
                 nodeType=exu.NodeType.RotationRotationVector,
                 referencePosition=pRef,
                 initialVelocity=v0,
                 initialAngularVelocity=[0,0,0],
                 gravity=g,
                 graphicsDataList=gList,
                 returnDict=True)
   [nMassCube0, oMassCube0] = [dictCube0['nodeNumber'], dictCube0['bodyNumber']]
   
   nCube0 = nMassCube0
   mCube0 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMassCube0))
   
   
   gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mCube0, contactStiffness=k, contactDamping=d, frictionMaterialIndex=1,
       pointList=meshPoints,  triangleList=meshTrigs)
   
   for p in meshPoints2:
       mPoint = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMassCube0, localPosition=p))
       gContact.AddSphereWithMarker(mPoint, radius=tTrig, contactStiffness=k, contactDamping=d, frictionMaterialIndex=1)
   
   if useGraphics:
       sCube0 = mbs.AddSensor(SensorNode(nodeNumber=nCube0, storeInternal=True, #fileName='solution/contactCube0.txt',
                                         outputVariableType=exu.OutputVariableType.Displacement))
   
   evalNodes += [nMassCube0]
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++
   
   #add as last because of transparency
   oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=gDataList)))
   
   #%%+++++++++++++++++++++++++++++++++
   mbs.Assemble()
   
   tEnd = 0.8 #tEnd = 0.8 for test suite
   h= 0.0002  #h= 0.0002 for test suite
   # h*=0.1
   # tEnd*=3
   simulationSettings = exu.SimulationSettings()
   #simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.solutionSettings.writeSolutionToFile = False
   if useGraphics:
       simulationSettings.solutionSettings.solutionWritePeriod = 0.001
       simulationSettings.solutionSettings.writeSolutionToFile = True
       simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/coordinatesSolution.txt'
   else:
       simulationSettings.solutionSettings.exportAccelerations = False
       simulationSettings.solutionSettings.exportVelocities = False
       
   simulationSettings.solutionSettings.sensorsWritePeriod = h*10
   simulationSettings.solutionSettings.outputPrecision = 8 #make files smaller
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.numericalDifferentiation.forODE2 = False
   simulationSettings.timeIntegration.newton.useModifiedNewton = False
   
   SC.visualizationSettings.general.graphicsUpdateInterval=0.05
   # SC.visualizationSettings.general.drawWorldBasis = True
   SC.visualizationSettings.general.circleTiling=200
   SC.visualizationSettings.general.drawCoordinateSystem=True
   SC.visualizationSettings.loads.show=False
   SC.visualizationSettings.bodies.show=True
   SC.visualizationSettings.markers.show=False
   
   SC.visualizationSettings.nodes.show=True
   SC.visualizationSettings.nodes.showBasis =True
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   SC.visualizationSettings.nodes.defaultSize = 0 #must not be -1, otherwise uses autocomputed size
   SC.visualizationSettings.nodes.tiling = 4
   SC.visualizationSettings.openGL.drawFaceNormals = False
   
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.openGL.shadow = 0.25
   SC.visualizationSettings.openGL.light0position = [-3,3,10,0]
   
   if useGraphics:
       SC.visualizationSettings.general.autoFitScene = False
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys['renderState'])
       SC.renderer.DoIdleTasks()
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.explicitIntegration.computeEndOfStepAccelerations = False #increase performance, accelerations less accurate
   mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitEuler)
   # mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ODE23)
   
   #compute error:
   uSum=0
   for node in evalNodes:
       u = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)
       exu.Print('coords node'+str(node)+' =',u)
       for c in u:
           uSum += abs(c) #add up all coordinates for comparison
   
   
   exu.Print('solution of generalContactFrictionTest=',uSum)
   exudynTestGlobals.testError = uSum - (10.132106712933348 ) 
   
   exudynTestGlobals.testResult = uSum
   
       
   if useGraphics:
       SC.renderer.DoIdleTasks()
   
       if True:
           SC.visualizationSettings.general.autoFitScene = False
           SC.visualizationSettings.general.graphicsUpdateInterval=0.02
           
           sol = LoadSolutionFile('solution/coordinatesSolution.txt', safeMode=True)#, maxRows=100)
           exu.Print('start SolutionViewer')
           mbs.SolutionViewer(sol)
   
       SC.renderer.Stop() #safely close rendering window!
   
   if useGraphics:
       
       
       mbs.PlotSensor([], closeAll=True)
       mbs.PlotSensor([sNode3]*3, [0,1,2], figureName='node stair')


