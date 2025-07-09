
.. _examples-beltdrivescomparison:

***********************
beltDrivesComparison.py
***********************

You can view and download this file on Github: `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/beltDrivesComparison.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Comparison of functionality of different belt drives
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-11-05
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import sys
   sys.exudynFast = True
   
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
   useGraphics = True
   useGeneralContact = False
   useContact = True
   staticEqulibrium = not useGeneralContact
   
   tEnd = 20 #end time of dynamic simulation
   stepSize = 2*1e-3 #step size
   useFriction = True
   dryFriction = 0.35
   nContactSegments = 8
   dEAfact = 2
   
   #GeneralContact:
   contactStiffness = 4e5
   contactDamping = 1e-3*contactStiffness
   
   #CableFriction contact:
   
   if not useGeneralContact:
       contactStiffness = 4e5*0.1
       contactDamping = 1e-2*contactStiffness*0.1
       dEAfact = 1
       stepSize = 0.25*1e-3 #step size
       dryFriction = 0.2 #lower because more accurate ...
       contactFrictionStiffness = contactStiffness*0.2
       frictionVelocityPenalty = 1e-4*contactFrictionStiffness
       nContactSegments = 2
   
   wheelMass = 1
   wheelInertia = 0.01
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #belt:
   g = 9.81
   gVec = [0,-g,0]         # gravity vector
   E=1e9                   # Young's modulus of ANCF element in N/m^2
   rhoBeam=1000            # density of ANCF element in kg/m^3
   b=0.002                 # width of rectangular ANCF element in m
   h=0.002                 # height of rectangular ANCF element in m
   A=b*h                   # cross sectional area of ANCF element in m^2
   I=b*h**3/12             # second moment of area of ANCF element in m^4
   EI = E*I                # bending stiffness
   EA = E*A                # axial stiffness
   dEI = 0*1e-3*EI #bending proportional damping
   dEA = dEAfact*1e-2*EA #axial strain proportional damping
   dimZ = 0.02 #z.dimension
   
   nANCFnodes = 50*2
   preStretch=-0.005 #relative stretch in beltdrive
   
   #wheels
   angularVelocity = 2*pi*1
   velocityControl = True
   rWheel = 0.25
   dWheels = 0.6
   leverArm = 0.5
   massArmTip = 1
   
   def UFvelocityDrive(mbs, t, itemNumber, lOffset): #time derivative of UFoffset
       return lOffset*min(angularVelocity*t, angularVelocity)
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create reeving system for each belt drive
   #complicated shape:
   circleList0 = [
                 [[0,0],rWheel,'L'],
                 [[dWheels,0],rWheel,'L'],
                 [[0,0],rWheel,'L'],
                 [[dWheels,0],rWheel,'L'],
                 ]
   circleList1 = [
                 [[0,0],rWheel,'R'],
                 [[dWheels,0],rWheel,'L'],
                 [[0,0],rWheel,'R'],
                 [[dWheels,0],rWheel,'L'],
                 ]
   factRadius = 0.5
   circleList2 = [
                 [[0,0],rWheel*factRadius,'L'],
                 [[dWheels,0],rWheel,'L'],
                 [[0,0],rWheel*factRadius,'L'],
                 [[dWheels,0],rWheel,'L'],
                 ]
   tensionerY = rWheel*0.7
   circleList3 = [
                 [[0,0],rWheel*factRadius,'L'],
                 [[dWheels*0.4,tensionerY],rWheel*0.2,'R'],
                 [[dWheels,0],rWheel,'L'],
                 [[dWheels*0.4,-tensionerY],rWheel*0.2,'R'],
                 [[0,0],rWheel*factRadius,'L'],
                 [[dWheels*0.4,tensionerY],rWheel*0.2,'R'],
                 ]
   
   
   reevingSystems = [circleList0,circleList1,circleList2,circleList3]
   # reevingSystems = [circleList0,circleList1,circleList2,]
   # reevingSystems = [circleList3]
   
   contactObjects = [] #ContactFrictionCable objects to modify in static initialization
   velControlList = [] #for static initialization
   fixWheelsList = []  #for static initialization
   
   for cnt, circleList in enumerate(reevingSystems):
       offX = (dWheels+rWheel*3)*cnt
       hasTensioner = int(len(circleList) == 6) #this is a special case with tensioners
       # print('cnt=',cnt, ', tensioner=', hasTensioner)
   
       for item in circleList:
           item[0][0]+=offX
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
       
       # print('belt length=',reevingDict['totalLength'])
       
       oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], 
                                          visualization=VObjectGround(graphicsData= gList)))
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #create ANCF elements:
       cableTemplate = Cable2D(#physicsLength = L / nElements, #set in GenerateStraightLineANCFCable2D(...)
                               physicsMassPerLength = rhoBeam*A,
                               physicsBendingStiffness = E*I,
                               physicsAxialStiffness = E*A,
                               physicsBendingDamping = dEI,
                               physicsAxialDamping = dEA,
                               physicsReferenceAxialStrain = preStretch, #prestretch
                               useReducedOrderIntegration=2,
                               visualization=VCable2D(drawHeight=2*h),
                               )
       
       ancf = PointsAndSlopes2ANCFCable2D(mbs, reevingDict['ancfPointsSlopes'], reevingDict['elementLengths'], 
                                          cableTemplate, massProportionalLoad=gVec, 
                                          #fixedConstraintsNode0=[1,1,1,1], fixedConstraintsNode1=[1,1,1,1],
                                          firstNodeIsLastNode=True, graphicsSizeConstraints=0.01)
       
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #add contact:
       if useContact:
       
           if useGeneralContact:
               gContact = mbs.AddGeneralContact()
               gContact.verboseMode = 1
               gContact.frictionProportionalZone = 0.1
               gContact.ancfCableUseExactMethod = False
               gContact.ancfCableNumberOfContactSegments = nContactSegments
               ssx = 64#32 #search tree size
               ssy = 12#32 #search tree size
               ssz = 1 #search tree size
               gContact.SetSearchTreeCellSize(numberOfCells=[ssx,ssy,ssz])
   
           sWheelRot = [] #sensors for angular velocity
       
           nGround = mbs.AddNode(NodePointGround())
           mCoordinateGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
           
           if hasTensioner:
               #add tensioning system
               nTensioner = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[offX+0.4*dWheels,0,0],
                                                   visualization=VNodeRigidBody2D(drawSize=dimZ*2)))
               #gTensioner = [graphics.Brick([0,0,dimZ],size=[0.1*rWheel,2.2*tensionerY,dimZ],color=graphics.color.orange)]
               cyl0 = graphics.Cylinder([0,tensionerY+0.05*rWheel,dimZ],vAxis=[0,-2*tensionerY-0.1*rWheel,0], 
                                           radius=0.05*rWheel, color=graphics.color.orange)
               cyl1 = graphics.Cylinder([0,tensionerY,dimZ],vAxis=[0.6*dWheels,-tensionerY,0], radius=0.05*rWheel, color=graphics.color.orange)
               cyl2 = graphics.Cylinder([0,-tensionerY,dimZ],vAxis=[0.6*dWheels,tensionerY,0], radius=0.05*rWheel, color=graphics.color.orange)
               gTensioner = [cyl0,cyl1,cyl2]
   
               oTensioner = mbs.AddObject(ObjectRigidBody2D(physicsMass=wheelMass*10, physicsInertia=wheelInertia*10,
                                                       nodeNumber=nTensioner, visualization=
                                                       VObjectRigidBody2D(graphicsData=gTensioner)))
               mTensionerCenter = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nTensioner))
               # mbs.AddLoad(Force(markerNumber=mTensionerCenter, loadVector=[1,-g*wheelMass,0]))
   
               mTensionerSupport = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oTensioner, localPosition=[0.6*dWheels,0,0]))
               mTensionerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[offX+1*dWheels,0,0]))
               mbs.AddObject(RevoluteJoint2D(markerNumbers=[mTensionerGround, mTensionerSupport]))
   
               mTensionerTop = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oTensioner, localPosition=[0,tensionerY,0]))
               mTensionerBottom = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oTensioner, localPosition=[0,-tensionerY,0]))
   
               if staticEqulibrium: #fix all wheels
                   mCoordinateTensioner = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nTensioner, coordinate=2))
                   cc = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCoordinateTensioner]))
                   fixWheelsList += [cc]
            
               # print(mbs.GetMarker(mTensionerTop))
               # print(mbs.GetMarker(mTensionerBottom))
            
           for i, wheel in enumerate(circleList):
               p = [wheel[0][0], wheel[0][1], 0] #position of wheel center
               r = wheel[1]
               
               rot0 = 0 #initial rotation
               pRef = [p[0], p[1], rot0]
               gList = [graphics.Cylinder(pAxis=[0,0,-0.5*dimZ],vAxis=[0,0,dimZ], radius=r-h,
                                             color= graphics.color.dodgerblue, nTiles=64),
                        graphics.Arrow(pAxis=[0,0,dimZ], vAxis=[0.9*r,0,0], radius=0.01*r, color=graphics.color.orange)]
   
               if i == 1+hasTensioner:
                   gList += [graphics.Brick([0,-0.5*leverArm,-dimZ],size=[leverArm*0.1,leverArm,dimZ],color=graphics.color.red)]
   
       
               omega0 = 0 #initial angular velocity
               v0 = np.array([0,0,omega0]) 
       
               nMass = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=pRef, initialVelocities=v0,
                                                   visualization=VNodeRigidBody2D(drawSize=dimZ*2)))
               oMass = mbs.AddObject(ObjectRigidBody2D(physicsMass=wheelMass, physicsInertia=wheelInertia,
                                                       nodeNumber=nMass, visualization=
                                                       VObjectRigidBody2D(graphicsData=gList)))
               mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
               mGroundWheel = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=p))
               
               if hasTensioner==0 or i==0 or i==2:
                   mbs.AddObject(RevoluteJoint2D(markerNumbers=[mGroundWheel, mNode]))
               elif hasTensioner==1:
                   if i==1:
                       mbs.AddObject(RevoluteJoint2D(markerNumbers=[mTensionerTop, mNode]))
                   elif i==3:
                       # mbs.AddObject(RevoluteJoint2D(markerNumbers=[mGroundWheel, mNode]))
                       mbs.AddObject(RevoluteJoint2D(markerNumbers=[mTensionerBottom, mNode]))
                       
               
               sWheelRot += [mbs.AddSensor(SensorNode(nodeNumber=nMass, 
                                                 fileName='solution/beltDrive'+str(cnt)+'wheel'+str(i)+'angVel.txt',
                                                 outputVariableType=exu.OutputVariableType.AngularVelocity))]
               
               mCoordinateWheel = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=2))
               if staticEqulibrium: #fix all wheels
                   cc = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCoordinateWheel]))
                   fixWheelsList += [cc]
   
               if i == 0:
                   if velocityControl:
                       fact = 1 #drive direction
                       if circleList[0][2] == 'R':
                           fact = -1
                       cc = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCoordinateWheel],
                                                               velocityLevel=True,offset=fact,
                                                               offsetUserFunction=UFvelocityDrive))
                       velControlList += [cc]
   
               elif i == 1+hasTensioner:
                   mLever = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oMass, localPosition=[0,-leverArm,0]))
                   mbs.AddLoad(Force(markerNumber=mLever, loadVector=[0,-g*massArmTip,0]))
                   
               if useGeneralContact:
                   frictionMaterialIndex=0
                   # if hasTensioner==0 or i==0 or i==2:
                   gContact.AddSphereWithMarker(mNode, radius=r, contactStiffness=contactStiffness, 
                                                contactDamping=contactDamping, frictionMaterialIndex=frictionMaterialIndex)
               else: #conventional contact, one contact per element and pulley
                   cableList = ancf[1]
                   mCircleBody = mNode
                   for k in range(len(cableList)):
                       nseg = nContactSegments
                       initialGapList = [0.1]*nseg + [-2]*nseg + [0]*nseg #initial gap of 0., isStick (0=slip, +-1=stick, -2 undefined initial state), lastStickingPosition (0)
       
                       mCable = mbs.AddMarker(MarkerBodyCable2DShape(bodyNumber=cableList[k], 
                                                                     numberOfSegments = nseg, verticalOffset=-h/2))
                       nodeDataContactCable = mbs.AddNode(NodeGenericData(initialCoordinates=initialGapList,
                                                                          numberOfDataCoordinates=nseg*3 ))
       
                       co = mbs.AddObject(ObjectContactFrictionCircleCable2D(markerNumbers=[mCircleBody, mCable], nodeNumber = nodeDataContactCable, 
                                                                numberOfContactSegments=nseg, 
                                                                contactStiffness = contactStiffness, 
                                                                contactDamping = contactDamping, 
                                                                frictionVelocityPenalty = frictionVelocityPenalty, 
                                                                frictionStiffness = contactFrictionStiffness, 
                                                                frictionCoefficient = dryFriction,
                                                                circleRadius = r,
                                                                # useSegmentNormals=False,
                                                                visualization=VObjectContactFrictionCircleCable2D(showContactCircle=False)))
                       contactObjects+=[co]
                       
       
       
           if useGeneralContact:
               halfHeight = 0.5*h*0
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
   # simulationSettings.displayComputationTime = True
   simulationSettings.parallel.numberOfThreads = 1 #use 4 to speed up for > 100 ANCF elements
   # simulationSettings.displayStatistics = True
   
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.stepInformation= 3+8+32+128+256
   
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
   simulationSettings.timeIntegration.discontinuous.maxIterations = 5+2
   # if not useGeneralContact:
   #     simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-4
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
   
   SC.visualizationSettings.general.circleTiling = 24
   SC.visualizationSettings.loads.show=False
   SC.visualizationSettings.nodes.defaultSize = 0.005
   SC.visualizationSettings.nodes.show=False
   SC.visualizationSettings.openGL.multiSampling = 4
   
   SC.visualizationSettings.general.textSize=14
   SC.visualizationSettings.general.showSolverInformation=False
   SC.visualizationSettings.general.renderWindowString = 'Comparision of belt drive configurations:\n - prescribed drive velocity at left pulley\n - lever arm under gravity attached to right pulley\n - showing axial forces as vertical bars along beams'
   SC.visualizationSettings.window.renderWindowSize=[1920,1080]
   SC.visualizationSettings.general.drawCoordinateSystem=False
   
   if True:
       SC.visualizationSettings.bodies.beams.axialTiling = 1
       SC.visualizationSettings.bodies.beams.drawVertical = True
       SC.visualizationSettings.bodies.beams.drawVerticalLines = True
   
       SC.visualizationSettings.contour.outputVariableComponent=0
       SC.visualizationSettings.contour.outputVariable=exu.OutputVariableType.ForceLocal
   
       SC.visualizationSettings.bodies.beams.drawVerticalFactor = 0.005
       SC.visualizationSettings.bodies.beams.drawVerticalOffset = -6*0
           
       SC.visualizationSettings.bodies.beams.reducedAxialInterploation = True
   
   
   #visualize contact:
   SC.visualizationSettings.connectors.showContact = True
   SC.visualizationSettings.contact.showContactForces = True
   SC.visualizationSettings.contact.contactForcesFactor = 0.025
   
   if False:
       SC.visualizationSettings.contact.showSearchTree =True
       SC.visualizationSettings.contact.showSearchTreeCells =True
       SC.visualizationSettings.contact.showBoundingBoxes = True
   
   if useGraphics: 
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   #simulationSettings.staticSolver.newton.absoluteTolerance = 1e-10
   simulationSettings.staticSolver.adaptiveStep = False
   simulationSettings.staticSolver.loadStepGeometric = True;
   simulationSettings.staticSolver.loadStepGeometricRange=1e3
   simulationSettings.staticSolver.numberOfLoadSteps = 10
   #simulationSettings.staticSolver.useLoadFactor = False
   simulationSettings.staticSolver.stabilizerODE2term = 1e5*10
   simulationSettings.staticSolver.newton.relativeTolerance = 1e-6
   simulationSettings.staticSolver.newton.absoluteTolerance = 1e-6
   simulationSettings.staticSolver.newton.numericalDifferentiation.minimumCoordinateSize = 1   #needed for static solver to converge
   simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-5      #needed for static solver to converge
   if staticEqulibrium: #precompute static equilibrium
       for velControl in velControlList:
           mbs.SetObjectParameter(velControl, 'activeConnector', False)
   
       for obj in contactObjects:
           mbs.SetObjectParameter(obj, 'frictionCoefficient', 0.)
           mbs.SetObjectParameter(obj, 'frictionStiffness', 1) #do not set to zero, as it needs to do some initialization...
               
       mbs.SolveStatic(simulationSettings, updateInitialValues=True)
   
       
       for obj in contactObjects:
           mbs.SetObjectParameter(obj, 'frictionCoefficient', dryFriction)
           mbs.SetObjectParameter(obj, 'frictionStiffness', contactFrictionStiffness)
           
       for velControl in velControlList:
           mbs.SetObjectParameter(velControl, 'activeConnector', True)
   
       for obj in fixWheelsList:
           mbs.SetObjectParameter(obj, 'activeConnector', False)
   
   
   mbs.SolveDynamic(simulationSettings,
                    # solverType=exu.DynamicSolverType.TrapezoidalIndex2
                    ) #183 Newton iterations, 0.114 seconds
   
   
   
   if useGraphics:
       SC.visualizationSettings.general.autoFitScene = False
       SC.visualizationSettings.general.graphicsUpdateInterval=0.02
       
       sol = LoadSolutionFile('solution/coordinatesSolution.txt', safeMode=True)#, maxRows=100)
       mbs.SolutionViewer(sol)
   
   
   if useGraphics: 
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
       
       # if True:
       #     
       #     mbs.PlotSensor(sensorNumbers=[sAngVel[0],sAngVel[1]], components=2, closeAll=True)
       #     mbs.PlotSensor(sensorNumbers=sMeasureRoll, components=1)
   
   
   
   
   
      
       
   
   
   
   
   


