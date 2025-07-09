
.. _testmodels-distancesensor:

*****************
distanceSensor.py
*****************

You can view and download this file on Github: `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/distanceSensor.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  test distance sensor with sphere and ANCFCable2D
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
   
   import numpy as np
   from math import sin, cos, sqrt,pi
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #parameters:
   m=1
   L = 2
   a = 0.1
   radius = 0.5*a
   t = 0.1*a
   k = 1e4 #4e3 needs h=1e-4
   d = 0.001*k
   g = 9.81
   
   #integration and contact settings
   stepSize = 1e-4 #step size
   gContact = mbs.AddGeneralContact()
   gContact.verboseMode = 1
   gContact.SetFrictionPairings(0*np.eye(1))
   noc = 8
   gContact.SetSearchTreeCellSize(numberOfCells=[noc,noc,noc])
   
   rRot = 0.2 #rotating table radius
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # add sphere and ground
   addEdges=False
   gFloor = graphics.Brick([0.45*L,0,-0.5*t],[L,L,t],graphics.color.steelblue,False,addEdges=addEdges)
   gFloorAdd = graphics.Cylinder([0,1*rRot,0],[0,2*rRot,0], radius=0.5*rRot, color=graphics.color.dodgerblue, addEdges=addEdges, 
                                    #angleRange=[0.5*pi,1.65*pi],lastFace=False,
                                    #angleRange=[0.5*pi,1.5*pi],lastFace=False,
                                    nTiles=4*8)#,
   gFloor = graphics.MergeTriangleLists(gFloorAdd, gFloor)
   gDataList = [gFloor]
   [meshPoints, meshTrigs] = graphics.ToPointsAndTrigs(gFloor)
   
   
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0] ))
   mGround = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGround))
   
   # [meshPoints, meshTrigs] = RefineMesh(meshPoints, meshTrigs) #just to have more triangles on floor
   gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mGround, contactStiffness=k, contactDamping=d, 
                                       frictionMaterialIndex=0, pointList=meshPoints, triangleList=meshTrigs)
   
   #gDataList = [graphics.FromPointsAndTrigs(meshPoints, meshTrigs, graphics.color.green)]
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add rotating table:
   gTable = [graphics.Cylinder([0,0,0],[0,0,0.05*rRot],radius=rRot, color=graphics.color.orange,addEdges=True, nTiles=64)]
   gTable+= [graphics.Cylinder([0,-rRot,0.05*rRot],[0,0,0.05*rRot],radius=0.1*rRot, color=graphics.color.orange,addEdges=True, nTiles=16)]
   nTable = mbs.AddNode(Node1D(referenceCoordinates=[0], initialVelocities=[4*pi]))
   oTable = mbs.AddObject(ObjectRotationalMass1D(physicsInertia=1, nodeNumber=nTable, referencePosition=[0,rRot,rRot], 
                                                 referenceRotation=np.eye(3), 
                                                 visualization=VRotor1D(graphicsData=gTable)))
   mTable = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oTable, localPosition=[0,-rRot,0]))
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add sphere:
   f=m*g
   p0 = np.array([0,0,radius-f/(k*0.5)]) #stiffness is serial from sphere and trigs ==> 0.5*k ...
   v0 = np.array([0.2,0,0])
   omega0 = np.array([0,0*v0[0]/radius,0])
   
   gObject = [graphics.Sphere(radius=radius, color=graphics.color.orange, nTiles=20)]
   gObject += [graphics.Basis(length=2*radius)]
   RBinertia = InertiaSphere(m, radius)
   oMass = mbs.CreateRigidBody(referencePosition=p0, 
                               initialVelocity=v0,
                               initialAngularVelocity=omega0,
                               inertia=RBinertia,
                               nodeType=exu.NodeType.RotationRotationVector, #for explicit integration
                               gravity=[0,0,-g],
                               graphicsDataList=gObject,
                               )
   nMass = mbs.GetObject(oMass)['nodeNumber']
   
   mThis = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
   
   gContact.AddSphereWithMarker(mThis, radius=radius, contactStiffness=k, contactDamping=d, 
                                frictionMaterialIndex=0)
   
   #put here, such that it is transparent in background
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                      visualization=VObjectGround(graphicsData=gDataList)))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add ANCFCable2D
   L=0.5                   # length of ANCF element in m
   E=1e7                   # Young's modulus of ANCF element in N/m^2
   rho=7800                # density of ANCF element in kg/m^3
   b=0.01                  # width of rectangular ANCF element in m
   h=0.05                  # height of rectangular ANCF element in m
   A=b*h                   # cross sectional area of ANCF element in m^2
   I=b*h**3/12             # second moment of area of ANCF element in m^4
   
   
   #generate ANCF beams with utilities function
   cableTemplate = Cable2D(physicsMassPerLength = rho*A,
                           physicsBendingStiffness = E*I,
                           physicsAxialStiffness = E*A,
                           physicsBendingDamping = 0.005*E*I,
                           useReducedOrderIntegration = 2,
                           visualization=VCable2D(drawHeight=h)
                           )
   
   positionOfNode0 = [-2*L, 0, 0.] # starting point of line
   positionOfNode1 = [-L, 0, 0.] # end point of line
   numberOfElements = 4
   
   #alternative to mbs.AddObject(Cable2D(...)) with nodes:
   ancf=GenerateStraightLineANCFCable2D(mbs,
                   positionOfNode0, positionOfNode1,
                   numberOfElements,
                   cableTemplate, #this defines the beam element properties
                   massProportionalLoad = [0,-9.81,0], #optionally add gravity
                   fixedConstraintsNode0 = [1,1,0,1], #add constraints for pos and rot (r'_y)
                   fixedConstraintsNode1 = [0,0,0,0])
   
   # #add all cable elements to contact
   for oIndex in ancf[1]:
       gContact.AddANCFCable(objectIndex=oIndex, halfHeight=0.5*h, 
                             contactStiffness=1, contactDamping=0, frictionMaterialIndex=0)
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # for i in range(10):
   #     sDist0 = mbs.CreateDistanceSensor(positionOrMarker=[i*2*radius,-1.3*radius,4*radius], dirSensor=[0,0,-2*radius], minDistance=0, maxDistance=2*t+4*radius, 
   #                                cylinderRadius=radius*0.5, storeInternal=True, addGraphicsObject=True)
   
   #alternative way:
   # def UFsensor0(mbs, t, sensorNumbers, factors, configuration):
   #     p0 = np.array([radius*3,1,0])
   #     d = gContact.ShortestDistanceAlongLine(pStart = p0, direction = [0,-1,0], 
   #                                            minDistance=0, maxDistance=1.0, cylinderRadius=0.01)
   #     return [d] 
   # sDistanceSphere = mbs.AddSensor(SensorUserFunction(sensorNumbers=[], factors=[],
   #                                           storeInternal=True,
   #                                           sensorUserFunction=UFsensor0))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add sensors
   ngc = mbs.NumberOfGeneralContacts()-1 #index of GeneralContact object that has been added last (0 here ...)
   sDistanceSphere = mbs.CreateDistanceSensor(ngc, positionOrMarker=[2*radius,4*radius,radius], dirSensor=[0,-2*radius,0], minDistance=0, maxDistance=1*t+4*radius, measureVelocity=True, 
                              cylinderRadius=radius*0.5, storeInternal=True, addGraphicsObject=True, selectedTypeIndex=exu.ContactTypeIndex.IndexSpheresMarkerBased)
   
   sDistanceSphere2 = mbs.CreateDistanceSensor(ngc, positionOrMarker=[2*radius,0,4*radius], dirSensor=[0,0,-2*radius], minDistance=0, maxDistance=1*t+4*radius, measureVelocity=True, 
                              cylinderRadius=radius*0.5, storeInternal=True, addGraphicsObject=True, selectedTypeIndex=exu.ContactTypeIndex.IndexSpheresMarkerBased)
   
   sDistanceTable = mbs.CreateDistanceSensor(ngc, positionOrMarker=mTable, dirSensor=[0,0,-2*radius], 
                                      minDistance=-10, maxDistance=1*t+4*radius, measureVelocity=True, 
                                      cylinderRadius=0, storeInternal=True, addGraphicsObject=True)#, selectedTypeIndex=exu.ContactTypeIndex.IndexSpheresMarkerBased)
   
   sANCF = mbs.CreateDistanceSensor(ngc, positionOrMarker=[-L*1.5,0,0], dirSensor=[0,-0.1,0], minDistance=0, maxDistance=L, measureVelocity=True, 
                             storeInternal=True, addGraphicsObject=True)
   
   sANCFdist = mbs.CreateDistanceSensor(ngc, positionOrMarker=[-0.6061511314921351,0,0], dirSensor=[0,-0.1,0], minDistance=0, maxDistance=L, measureVelocity=True, 
                             storeInternal=True, addGraphicsObject=True)
   
   sANCFdisp = mbs.AddSensor(SensorNode(nodeNumber=ancf[0][-1], storeInternal=True, outputVariableType=exu.OutputVariableType.Displacement))
   
   sVelocitySphere = mbs.AddSensor(SensorMarker(markerNumber=mThis, storeInternal=True,
                                                outputVariableType=exu.OutputVariableType.Velocity))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   mbs.Assemble()
   # exu.Print(gContact)
   
   tEnd = 0.25
   #tEnd = h*100
   simulationSettings = exu.SimulationSettings()
   # simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.solutionSettings.writeSolutionToFile = False
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.01
   simulationSettings.displayComputationTime = useGraphics
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
   
   # simulationSettings.timeIntegration.simulateInRealtime = True
   # simulationSettings.timeIntegration.realtimeFactor = 0.5
   simulationSettings.timeIntegration.verboseMode = 1
   
   # SC.visualizationSettings.loads.show=False
   SC.visualizationSettings.window.renderWindowSize=[1600,1200]
   SC.visualizationSettings.openGL.multiSampling = 4
   # SC.visualizationSettings.openGL.shadow = 0.3
   SC.visualizationSettings.openGL.light0position = [-5,-5,20,0]
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   
   if False: #show bounding boxes
       SC.visualizationSettings.contact.showSearchTree =True
       SC.visualizationSettings.contact.showSearchTreeCells =True
       SC.visualizationSettings.contact.showBoundingBoxes = True
   
   if useGraphics:
       SC.visualizationSettings.general.autoFitScene = False
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys['renderState'])
       SC.renderer.DoIdleTasks()
   
   
   mbs.SolveDynamic(simulationSettings, 
                    #solverType=exu.DynamicSolverType.ExplicitEuler,
                    solverType=exu.DynamicSolverType.RK44,
                    )
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   x=mbs.GetNodeOutput(ancf[0][-1], variableType=exu.OutputVariableType.Position)
   exu.Print('pLast=',list(x),'\n')
   #[-0.546983567323076, -0.19231209764430873, 0.0]
   
   
   s1 = (mbs.GetSensorValues(sDistanceSphere))
   s2 = (mbs.GetSensorValues(sDistanceSphere2))
   s3 = (mbs.GetSensorValues(sANCF))
   s4 = (mbs.GetSensorValues(sANCFdist))
   s5 = (mbs.GetSensorValues(sVelocitySphere))
   
   exu.Print('sensors=',s1,s2,s3,s4,s5,'\n')
   
   u = NormL2(s1) + NormL2(s2) + NormL2(s3) + NormL2(s4) + NormL2(s5)
   
   exu.Print('solution of distanceSensor=',u)
   exudynTestGlobals.testResult = u
           
   #%%
   if useGraphics:
       
       mbs.PlotSensor(closeAll=True)
       mbs.PlotSensor(sDistanceSphere, components=0, colorCodeOffset=0, labels=['y-axis'])
       mbs.PlotSensor(sDistanceSphere2, components=0, colorCodeOffset=1, newFigure=False, labels=['z-axis'])
       mbs.PlotSensor(sDistanceTable, components=0, colorCodeOffset=2, newFigure=False, labels=['table z-dist'])
   
       mbs.PlotSensor(sDistanceSphere, components=1, colorCodeOffset=3, newFigure=False, labels=['LDV'])
       mbs.PlotSensor(sANCFdist, components=0, colorCodeOffset=5, newFigure=False, labels=['ANCF distance'])
       mbs.PlotSensor(sANCFdisp, components=1, colorCodeOffset=6, newFigure=False, labels=['ANCF displacement'], factors=[-1])
       # mbs.PlotSensor(sVelocitySphere, components=0, closeAll=True)
       
   


