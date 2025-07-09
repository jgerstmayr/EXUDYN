
.. _examples-reevingsystem:

****************
reevingSystem.py
****************

You can view and download this file on Github: `reevingSystem.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/reevingSystem.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example to calculate rope line of reeving system
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-02-02
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.beams import *
   
   import numpy as np
   from math import sin, cos, pi, sqrt , asin, acos, atan2
   import copy 
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #settings:
   useGraphics= True
   useContact = True
   tEnd = 20 #end time of dynamic simulation
   stepSize = 2e-3 #step size
   useFriction = True
   dryFriction = 0.5
   contactStiffness = 2e5
   contactDamping = 1e-3*contactStiffness
   
   wheelMass = 1
   wheelInertia = 0.01
   
   rotationDampingWheels = 0.00 #proportional to rotation speed
   torque = 1
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create circles
   #complicated shape:
   nANCFnodes = 200
   preStretch=-0.001
   circleList = [[[0,0],0.3,'L'],
                 [[1,0],0.3,'R'],
                 [[1,1],0.3,'R'],
                 [[2,1],0.4,'R'],
                 [[3,1],0.4,'R'],
                 [[4,1],0.4,'L'],
                 [[5,1],0.4,'R'],
                 [[5,-1],0.3,'L'],
                 [[0,-1],0.4,'L'],
                 [[0,0],0.3,'L'],
                 [[1,0],0.3,'R'],
                 ]
   
   #simple shape:
   # nANCFnodes = 50
   # preStretch=-0.005
   # circleList = [[[0,0],0.2,'L'],
   #               [[0,1],0.2,'L'],
   #               [[0.8,0.8],0.4,'L'],
   #               [[1,0],0.2,'L'],
   #               [[0,0],0.2,'L'],
   #               [[0,1],0.2,'L'],
   #               ]
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create geometry:
   reevingDict = CreateReevingCurve(circleList, drawingLinesPerCircle = 64, 
                                    removeLastLine=True, #allows closed curve
                                    numberOfANCFnodes=nANCFnodes, graphicsNodeSize= 0.01)
   
   del circleList[-1] #remove circles not needed for contact/visualization
   del circleList[-1] #remove circles not needed for contact/visualization
   
   gList=[]
   if False: #visualize reeving curve, without simulation
       gList = reevingDict['graphicsDataLines'] + reevingDict['graphicsDataCircles']
   
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], 
                                      visualization=VObjectGround(graphicsData= gList)))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create ANCF elements:
   
   gVec = [0,-9.81,0]      # gravity
   E=1e9                   # Young's modulus of ANCF element in N/m^2
   rhoBeam=1000            # density of ANCF element in kg/m^3
   b=0.002                 # width of rectangular ANCF element in m
   h=0.002                 # height of rectangular ANCF element in m
   A=b*h                   # cross sectional area of ANCF element in m^2
   I=b*h**3/12             # second moment of area of ANCF element in m^4
   dEI = 1e-3*E*I #bending proportional damping
   dEA = 1e-2*E*A #axial strain proportional damping
   
   dimZ = b #z.dimension
   
   cableTemplate = Cable2D(#physicsLength = L / nElements, #set in GenerateStraightLineANCFCable2D(...)
                           physicsMassPerLength = rhoBeam*A,
                           physicsBendingStiffness = E*I,
                           physicsAxialStiffness = E*A,
                           physicsBendingDamping = dEI,
                           physicsAxialDamping = dEA,
                           physicsReferenceAxialStrain = preStretch, #prestretch
                           visualization=VCable2D(drawHeight=h),
                           )
   
   ancf = PointsAndSlopes2ANCFCable2D(mbs, reevingDict['ancfPointsSlopes'], reevingDict['elementLengths'], 
                                      cableTemplate, massProportionalLoad=gVec, 
                                      #fixedConstraintsNode0=[1,1,1,1], fixedConstraintsNode1=[1,1,1,1],
                                      firstNodeIsLastNode=True, graphicsSizeConstraints=0.01)
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add contact:
   if useContact:
   
       gContact = mbs.AddGeneralContact()
       gContact.verboseMode = 1
       gContact.frictionProportionalZone = 1
       gContact.ancfCableUseExactMethod = False
       gContact.ancfCableNumberOfContactSegments = 8
       ssx = 16#32 #search tree size
       ssy = 16#32 #search tree size
       ssz = 1 #search tree size
       gContact.SetSearchTreeCellSize(numberOfCells=[ssx,ssy,ssz])
       #gContact.SetSearchTreeBox(pMin=np.array([-1,-1,-1]), pMax=np.array([4,1,1])) #automatically computed!
   
       halfHeight = 0.5*h*0
       dimZ= 0.01 #for drawing
       # wheels = [{'center':wheelCenter0, 'radius':rWheel0-halfHeight, 'mass':mWheel}, 
       #           {'center':wheelCenter1, 'radius':rWheel1-halfHeight, 'mass':mWheel}, 
       #           {'center':rollCenter1, 'radius':rRoll-halfHeight, 'mass':mRoll}, #small mass for roll, not to influence beam
       #           ]
       sWheelRot = [] #sensors for angular velocity
   
       nGround = mbs.AddNode(NodePointGround())
       mCoordinateGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
       
       for i, wheel in enumerate(circleList):
           p = [wheel[0][0], wheel[0][1], 0] #position of wheel center
           r = wheel[1]
           
           rot0 = 0 #initial rotation
           pRef = [p[0], p[1], rot0]
           gList = [graphics.Cylinder(pAxis=[0,0,-dimZ],vAxis=[0,0,-dimZ], radius=r,
                                         color= graphics.color.dodgerblue, nTiles=64),
                    graphics.Arrow(pAxis=[0,0,0], vAxis=[0.9*r,0,0], radius=0.01*r, color=graphics.color.orange)]
   
           omega0 = 0 #initial angular velocity
           v0 = np.array([0,0,omega0]) 
   
           nMass = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=pRef, initialVelocities=v0,
                                               visualization=VNodeRigidBody2D(drawSize=dimZ*2)))
           oMass = mbs.AddObject(ObjectRigidBody2D(physicsMass=wheelMass, physicsInertia=wheelInertia,
                                                   nodeNumber=nMass, visualization=
                                                   VObjectRigidBody2D(graphicsData=gList)))
           mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
           mGroundWheel = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=p))
           
           mbs.AddObject(RevoluteJoint2D(markerNumbers=[mGroundWheel, mNode]))
           sWheelRot += [mbs.AddSensor(SensorNode(nodeNumber=nMass, 
                                             fileName='solution/wheel'+str(i)+'angVel.txt',
                                             outputVariableType=exu.OutputVariableType.AngularVelocity))]
           
           def UFvelocityDrive(mbs, t, itemNumber, lOffset): #time derivative of UFoffset
               v = 10*t
               vMax = 5
               return max(v, vMax)
   
           velocityControl = True
           if i == 0:
               if velocityControl:
                   mCoordinateWheel = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=2))
                   mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCoordinateWheel],
                                                       velocityLevel=True, offsetUserFunction_t=UFvelocityDrive))
                   
           #     else:
           #         mbs.AddLoad(LoadTorqueVector(markerNumber=mNode, loadVector=[0,0,torque]))
           if i > 0: #friction on rolls:
               mCoordinateWheel = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=2))
               mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mCoordinateGround, mCoordinateWheel], 
                                                     damping=rotationDampingWheels, 
                                                     visualization=VCoordinateSpringDamper(show=False)))
   
           frictionMaterialIndex=0
           gContact.AddSphereWithMarker(mNode, radius=r, contactStiffness=contactStiffness, 
                                        contactDamping=contactDamping, frictionMaterialIndex=frictionMaterialIndex)
           
   
   
       for oIndex in ancf[1]:
           gContact.AddANCFCable(objectIndex=oIndex, halfHeight=halfHeight, #halfHeight should be h/2, but then cylinders should be smaller
                                 contactStiffness=contactStiffness, contactDamping=contactDamping, 
                                 frictionMaterialIndex=0)
   
       frictionMatrix = np.zeros((2,2))
       frictionMatrix[0,0]=int(useFriction)*dryFriction
       frictionMatrix[0,1]=0 #no friction between some rolls and cable
       frictionMatrix[1,0]=0 #no friction between some rolls and cable
       gContact.SetFrictionPairings(frictionMatrix)
   
   
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/coordinatesSolution.txt'
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.005
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.001
   #simulationSettings.displayComputationTime = True
   simulationSettings.parallel.numberOfThreads = 1 #use 4 to speed up for > 100 ANCF elements
   simulationSettings.displayStatistics = True
   
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.stepInformation= 3+128+256
   
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
   simulationSettings.displayStatistics = True
   
   SC.visualizationSettings.general.circleTiling = 24
   SC.visualizationSettings.loads.show=False
   SC.visualizationSettings.nodes.defaultSize = 0.01
   SC.visualizationSettings.openGL.multiSampling = 4
   
   if False:
       SC.visualizationSettings.contour.outputVariableComponent=0
       SC.visualizationSettings.contour.outputVariable=exu.OutputVariableType.ForceLocal
   
   #visualize contact:
   if False:
       SC.visualizationSettings.contact.showSearchTree =True
       SC.visualizationSettings.contact.showSearchTreeCells =True
       SC.visualizationSettings.contact.showBoundingBoxes = True
   
   if useGraphics: 
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   doDynamic = True
   if doDynamic :
       mbs.SolveDynamic(simulationSettings) #183 Newton iterations, 0.114 seconds
   else:
       mbs.SolveStatic(simulationSettings) #183 Newton iterations, 0.114 seconds
   
   
   if useGraphics and True:
       SC.visualizationSettings.general.autoFitScene = False
       SC.visualizationSettings.general.graphicsUpdateInterval=0.02
       
       mbs.SolutionViewer()
   
   
   if useGraphics: 
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
       
       # if True:
       #     
       #     mbs.PlotSensor(sensorNumbers=[sAngVel[0],sAngVel[1]], components=2, closeAll=True)
       #     mbs.PlotSensor(sensorNumbers=sMeasureRoll, components=1)
   
   
   
   
   
      
       
   
   
   
   
   


