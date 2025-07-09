
.. _testmodels-generalcontactcylindertrigstest:

**********************************
generalContactCylinderTrigsTest.py
**********************************

You can view and download this file on Github: `generalContactCylinderTrigsTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/generalContactCylinderTrigsTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  model with GeneralContact of a cylinder modelled by triangles rolling on a bed of spherical markers
   #
   # Author:   Johannes Gerstmayr
   # Date:     2024-03-16
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
   
   #create an environment for mini example
   
   gravity = [0,0,-9.81]
   
   p0 = np.array([0,0,0])
   planeL = 0.3
   xFact = 5
   nPlane = 6 #spheres along x and y
   rMarker = 0.02 #radius of spheres for contact points
   
   rCyl = 0.25
   tCyl = 0.25
   
   sRad = 0.02
   k = 2e4 
   d = 0.0005*k
   stepSize = 1e-3
   
   frictionCoeff = 0.1
   ss = 10
   
   markerList = []
   radiusList = []
   gDataList = []
   
   
   gContact = mbs.AddGeneralContact()
   # gContact.verboseMode = 1
   gContact.SetFrictionPairings(frictionCoeff*np.eye(1))
   gContact.SetSearchTreeCellSize(numberOfCells=[ss,ss,ss])
   # gContact.SetSearchTreeBox(pMin=np.array([-0.5*planeL,-0.5*planeL,-0.1]), 
   #                           pMax=np.array([0.5*planeL,0.5*planeL,0.1]))
   #print('treesize=',ssx*ssx*ssy)
   #gContact.sphereSphereContact = False #does not influence results (only performance)!
   
   
   #%% ground
   gFloor = graphics.CheckerBoard(p0+[planeL*xFact*0.5,0,0],size=planeL*xFact, size2=planeL, nTiles=8)
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                       visualization=VObjectGround(graphicsData=[gFloor])))
   
   
   # nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0] ))
   # mGround = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGround))
   
   gCyl = graphics.Cylinder(pAxis=[0,-0.5*tCyl,0],vAxis=[0,tCyl,0], radius=rCyl, 
                                          color=[0.3,0.3,0.3,1],
                                          alternatingColor=graphics.color.lightgrey,nTiles=64)
   
   
   #rigid body containing sphere markers:
   omegaY = 10
   bCyl=mbs.CreateRigidBody(referencePosition=[0,0,rCyl*1.0],
                       initialVelocity=[omegaY*rCyl,0,0],
                       #initialAngularVelocity=[0,omegaY,0],
                       # referenceRotationMatrix=RotationMatrixX(0.1),
                       inertia=InertiaCylinder(200, tCyl, rCyl, 1),
                       gravity = gravity,
                       nodeType = exu.NodeType.RotationRotationVector,
                       graphicsDataList=[gCyl],
                       )
   mCyl = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCyl, localPosition=[0,0,0]))
   sPos = mbs.AddSensor(SensorBody(bodyNumber=bCyl, storeInternal=True, 
                                   outputVariableType=exu.OutputVariableType.Position))
   sRot = mbs.AddSensor(SensorBody(bodyNumber=bCyl, storeInternal=True, 
                                   outputVariableType=exu.OutputVariableType.Rotation))
   sVel = mbs.AddSensor(SensorBody(bodyNumber=bCyl, storeInternal=True, 
                                   outputVariableType=exu.OutputVariableType.Velocity))
   sOmega = mbs.AddSensor(SensorBody(bodyNumber=bCyl, storeInternal=True, 
                                   outputVariableType=exu.OutputVariableType.AngularVelocity))
   
   [meshPoints, meshTrigs] = graphics.ToPointsAndTrigs(gCyl)
   
   
   gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mCyl, 
                                       contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,
       pointList=meshPoints,  triangleList=meshTrigs)
   
   
   
   for ix in range(nPlane*xFact+1):
       for iy in range(nPlane+1):
           x = (ix/nPlane-0.0)*planeL
           y = (iy/nPlane-0.5)*planeL
           z = -rMarker
       
           m = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[x,y,z]))
           gContact.AddSphereWithMarker(m, radius=rMarker, contactStiffness=k, contactDamping=d, 
                                        frictionMaterialIndex=0)
       
   
   
   
   
   mbs.Assemble()
   
   items=gContact.GetItemsInBox(pMin=[-4,-4,0], pMax=[4,4,20])
   # print('n spheres=',len(items['MarkerBasedSpheres'])) 
   
   
   tEnd = 1
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = False
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.parallel.numberOfThreads = 4
   
   SC.visualizationSettings.general.graphicsUpdateInterval=0.02
   SC.visualizationSettings.general.drawCoordinateSystem=True
   SC.visualizationSettings.loads.show=False
   SC.visualizationSettings.bodies.show=True
   SC.visualizationSettings.markers.show=False
   
   SC.visualizationSettings.nodes.show=True
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   SC.visualizationSettings.nodes.defaultSize = 0 #must not be -1, otherwise uses autocomputed size
   SC.visualizationSettings.nodes.tiling = 4
   
   SC.visualizationSettings.window.renderWindowSize=[1900,1200]
   SC.visualizationSettings.openGL.multiSampling = 4
   #improved OpenGL rendering
   
   SC.visualizationSettings.contact.showSpheres = True
   SC.visualizationSettings.contact.tilingSpheres = 4
   
   
   if useGraphics:
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys['renderState'])
       # SC.renderer.DoIdleTasks()
   
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitEuler)
   
   if useGraphics:
       #SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   #%%+++++++++++++++++++
   q = mbs.GetSensorValues(sPos)
   q += mbs.GetSensorValues(sVel)
   q += mbs.GetSensorValues(sOmega)
   q += mbs.GetSensorValues(sRot)
   #print('q=', q)
   
   u = NormL2(q)
   exu.Print('solution of generalContactCylinderTest =',u)
   
   exudynTestGlobals.testError = u - (5.486908430912642) 
   exudynTestGlobals.testResult = u


