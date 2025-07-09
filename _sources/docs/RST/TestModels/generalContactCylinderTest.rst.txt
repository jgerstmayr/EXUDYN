
.. _testmodels-generalcontactcylindertest:

*****************************
generalContactCylinderTest.py
*****************************

You can view and download this file on Github: `generalContactCylinderTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/generalContactCylinderTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  model with GeneralContact of a cylinder modelled by spheres rolling on triangle ground mesh
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
   
   planeL = 4
   p0 = [0.5*planeL,0,0]
   
   nPhi = 80  #spheres around circumference
   nThick = 1 #spheres along cylinder axis (+1)
   
   rCyl = 0.25
   tCyl = 0.1
   rMarker = 0.02 #radius of spheres for contact points
   
   sRad = 0.02
   k = 1e4 
   d = 0.0005*k
   stepSize = 1e-3
   
   frictionCoeff = 0.2
   ss = 10
   
   markerList = []
   radiusList = []
   gDataList = []
   
   
   gContact = mbs.AddGeneralContact()
   #gContact.verboseMode = 1
   gContact.SetFrictionPairings(frictionCoeff*np.eye(1))
   gContact.SetSearchTreeCellSize(numberOfCells=[ss,ss,ss])
   # gContact.SetSearchTreeBox(pMin=np.array([-0.5*planeL,-0.5*planeL,-0.1]), 
   #                           pMax=np.array([0.5*planeL,0.5*planeL,0.1]))
   #print('treesize=',ssx*ssx*ssy)
   # gContact.sphereSphereContact = False
   
   #%% ground
   gFloor = graphics.CheckerBoard(p0,size=planeL, nTiles=10)
   gDataList = [gFloor]
   
   
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0] ))
   mGround = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGround))
   
   [meshPoints, meshTrigs] = graphics.ToPointsAndTrigs(gFloor)
   #[meshPoints, meshTrigs] = RefineMesh(meshPoints, meshTrigs) #just to have more triangles on floor
   gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mGround, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,
       pointList=meshPoints,  triangleList=meshTrigs)
   
   
   #rigid body containing sphere markers:
   inertia = InertiaCylinder(1000, tCyl, rCyl, 1)
   # print(inertia)
   
   omegaY = 12
   bCyl=mbs.CreateRigidBody(referencePosition=[0,0,rCyl*1.0],
                       initialVelocity=[omegaY*rCyl,0,0],
                       initialAngularVelocity=[0,omegaY*0.5,0],
                       initialRotationMatrix=RotationMatrixX(0.1),
                       inertia=inertia,
                       gravity = gravity,
                       nodeType = exu.NodeType.RotationRotationVector,
                       graphicsDataList=[graphics.Cylinder(pAxis=[0,-0.5*tCyl,0],vAxis=[0,tCyl,0], radius=rCyl, 
                                                              color=[0.3,0.3,0.3,1],
                                                              alternatingColor=graphics.color.lightgrey,nTiles=64)]
                       )
   nCyl = mbs.GetObject(bCyl)['nodeNumber']
   #print(mbs.GetNode(nCyl))
   sPos = mbs.AddSensor(SensorBody(bodyNumber=bCyl, storeInternal=True, 
                                   outputVariableType=exu.OutputVariableType.Position))
   sRot = mbs.AddSensor(SensorBody(bodyNumber=bCyl, storeInternal=True, 
                                   outputVariableType=exu.OutputVariableType.Rotation))
   sVel = mbs.AddSensor(SensorBody(bodyNumber=bCyl, storeInternal=True, 
                                   outputVariableType=exu.OutputVariableType.Velocity))
   sOmega = mbs.AddSensor(SensorBody(bodyNumber=bCyl, storeInternal=True, 
                                   outputVariableType=exu.OutputVariableType.AngularVelocity))
   
   for phiI in range(nPhi):
       phi = phiI/nPhi*2*pi
       for j in range(nThick+1):
           rCylMod = rCyl-rMarker
           #compute local coordinates for markers
           y = (j/nThick-0.5)*tCyl
           x = rCylMod*sin(phi)
           z = rCylMod*cos(phi)
       
           m = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCyl, localPosition=[x,y,z]))
           gContact.AddSphereWithMarker(m, radius=rMarker, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)
       
   
   
   #put ground here, such that it is transparent in background
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                       visualization=VObjectGround(graphicsData=gDataList)))
   
   
   mbs.Assemble()
   
   items=gContact.GetItemsInBox(pMin=[-4,-4,0], pMax=[4,4,20])
   #print('n spheres=',len(items['MarkerBasedSpheres'])) 
   
   
   tEnd = 2
   #tEnd = h*100
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = False
   #simulationSettings.displayComputationTime = True
   #simulationSettings.displayStatistics = True
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
   
   SC.visualizationSettings.window.renderWindowSize=[2000,1200]
   SC.visualizationSettings.openGL.multiSampling = 4
   #improved OpenGL rendering
   
   SC.visualizationSettings.contact.showSpheres = True
   
   SC.visualizationSettings.general.autoFitScene = False
   
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
   
   exudynTestGlobals.testError = u - (12.42377622187738 ) 
   exudynTestGlobals.testResult = u
   
   
   
   


