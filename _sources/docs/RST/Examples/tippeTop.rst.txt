
.. _examples-tippetop:

***********
tippeTop.py
***********

You can view and download this file on Github: `tippeTop.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/tippeTop.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  example of tippe top toy using GeneralContact, switching up-down after certain time
   #           NOTE: this is only a demonstration of GeneralContact, but due to soft contact
   #           this computational model may not fully capture effects of the real contact, which
   #           would require further adjustment of parameters and comparison with experimental results!
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-12-12
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #contact
   
   L = 5
   n = 1
   r = 0.2
   mu = 0.4
   m = 0.01
   k = 1e3
   d = 0.001*k
   
   ssx = 3 #search tree size
   ssy = 3 #search tree size
   ssz = 3 #search tree size
   
   tt = 0.2*r
   zz = -0.04
   markerList = []
   p0 = np.array([0.,0,-0.5*tt])
   gFloor = graphics.Brick(p0,[L,L,tt],graphics.color.lightgrey)
   [meshPoints, meshTrigs] = graphics.ToPointsAndTrigs(gFloor)
   #gDataList = [gFloor]
   
   gDataList = [graphics.CheckerBoard([0,0,0],size=L)]
   
   useRigidBody = True
   #ns = 20
   minP = np.array([1e10,1e10,1e10])
   maxP = -minP
   gContact = mbs.AddGeneralContact()
   height = r*2.4
   r2 = 0.2*r
   
   for i in range(n):
   
       color4node = graphics.color.steelblue
       pS0 = [0,0,-zz]
       pS1 = [0,0,height-r-r2-zz]
       gList = [graphics.Sphere(point=pS0, radius=r, color= color4node, nTiles=24)]
       gList += [graphics.Sphere(point=pS1, radius=r2, color= color4node, nTiles=16)]
   
       pRef = [0,0,r+zz]
       v0 = [0,-10*0.,0]
       omega0 = 20*np.array([0,0.02,10])
       rot0 = np.eye(3)
   
       RBinertia = InertiaSphere(m, r*1)
   
       RBinertia = InertiaCuboid(m/(2*r*2*r*height), [2*r,2*r,height])
   
       dictMass = mbs.CreateRigidBody(
                     inertia=RBinertia, 
                     nodeType=exu.NodeType.RotationRotationVector,
                     referencePosition=pRef, 
                     initialVelocity=v0,
                     referenceRotationMatrix=rot0,
                     initialAngularVelocity=omega0,
                     gravity=[0., 0., -9.81],
                     graphicsDataList=gList,
                     returnDict=True)
       [nMass, oMass] = [dictMass['nodeNumber'], dictMass['bodyNumber']]
   
   
   
       mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
       #if useNodeMarker:
       #    markerList += [mNode]
       mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass, localPosition=pS0))
       gContact.AddSphereWithMarker(mBody, radius=r, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)
       mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass, localPosition=pS1))
       gContact.AddSphereWithMarker(mBody, radius=r2, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)
   
       #mbs.AddLoad(Force(markerNumber=mNode, loadVector= [0,forceY,0]))
   
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                       visualization=VObjectGround(graphicsData=gDataList)))
   mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
   
   gContact.verboseMode = 1
   #gContact.sphereSphereContact = False
   gContact.frictionProportionalZone = 1e-5
   #[meshPoints,  meshTrigs] = RefineMesh(meshPoints,  meshTrigs)
   gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mGround, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,
       pointList=meshPoints,  triangleList=meshTrigs)
   
   gContact.SetFrictionPairings(mu*np.eye(1))
   gContact.SetSearchTreeCellSize(numberOfCells=[ssx,ssy,ssz])
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   mbs.Assemble()
   print(mbs)
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 40
   h = 2*1e-4
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.002
   #simulationSettings.displayComputationTime = True
   #simulationSettings.displayGlobalTimers= True
   simulationSettings.timeIntegration.verboseMode = 1
   # simulationSettings.timeIntegration.simulateInRealtime = True #turn this off to run faster!
   
   simulationSettings.displayStatistics = True
   
   SC.visualizationSettings.nodes.defaultSize = 0.01
   
   
   simulationSettings.solutionSettings.outputPrecision = 6
   simulationSettings.solutionSettings.exportVelocities = False
   simulationSettings.solutionSettings.exportAccelerations = False
   
   simulationSettings.timeIntegration.newton.numericalDifferentiation.forODE2 = False
   simulationSettings.timeIntegration.newton.useModifiedNewton = False
   simulationSettings.timeIntegration.discontinuous.iterationTolerance = 0.1
   
   
   SC.visualizationSettings.general.graphicsUpdateInterval=0.01
   #SC.visualizationSettings.general.circleTiling=50
   SC.visualizationSettings.general.drawCoordinateSystem=True
   SC.visualizationSettings.loads.show=False
   SC.visualizationSettings.nodes.show=True
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = r*2
   SC.visualizationSettings.nodes.defaultSize = r
   SC.visualizationSettings.nodes.tiling = 2
   SC.visualizationSettings.window.renderWindowSize=[1200,800]
   SC.visualizationSettings.openGL.multiSampling = 4
   
   showContact = False
   SC.visualizationSettings.contact.showSearchTreeCells = showContact
   SC.visualizationSettings.contact.showSearchTree = showContact
   SC.visualizationSettings.contact.showBoundingBoxes = showContact
   
   simulate=True
   if simulate:
       useGraphics = True
       if useGraphics:
           SC.renderer.Start()
           if 'renderState' in exu.sys:
               SC.renderer.SetState(exu.sys['renderState'])
           SC.renderer.DoIdleTasks()
   
   
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitEuler)
       # mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitMidpoint)
       #mbs.SolveDynamic(simulationSettings)
   
       if useGraphics:
           SC.renderer.DoIdleTasks()
           SC.renderer.Stop() #safely close rendering window!
   
   if not simulate or True:
       SC.visualizationSettings.general.autoFitScene = False
       
       mbs.SolutionViewer()


