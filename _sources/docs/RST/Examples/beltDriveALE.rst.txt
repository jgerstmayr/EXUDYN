
.. _examples-beltdriveale:

***************
beltDriveALE.py
***************

You can view and download this file on Github: `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/beltDriveALE.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Model for belt drive; using ALE ANCF cable and regular cable
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-07-08
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
   from math import sin, cos, pi, sqrt , asin, acos, atan2, exp
   import copy 
   
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #improvedBelt = True #True: improved belt model (tEnd ~= 2.5 seconds simulation, more damping, better initial conditions, etc.)
   
   #%%
   #settings:
   useGraphics = True
   useContact = True
   doDynamic = True
   makeAnimation = False
   useALE = True
   useFrictionStiffness = True
   
   stepSize = 0.5*0.5*1e-4  #accurate: 2.5e-5 # for frictionVelocityPenalty = 1e7*... it must be not larger than 2.5e-5
   discontinuousIterations = 0+3 #larger is more accurate, but smaller step size is equivalent
   
   if useFrictionStiffness:
       stepSize = 0.25*0.5*1e-4  #accurate: 2.5e-5 # for frictionVelocityPenalty = 1e7*... it must be not larger than 2.5e-5
       # discontinuousIterations = 6+3 #larger is more accurate, but smaller step size is equivalent
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Parameters for the belt
   gVec = [0,-9.81*1,0]     # gravity
   Emodulus=1e7             # Young's modulus of ANCF element in N/m^2
   b=0.08 #0.002            # width of rectangular ANCF element in m
   hc = 0.0001              # height (geometric) of rectangular ANCF element in m
   hcStiff = 0.01           # stiffness relevant height
   rhoBeam=1036.            # density of ANCF element in kg/m^3
   A=b*hcStiff              # cross sectional area of ANCF element in m^2
   I=(b*hcStiff**3)/12      # second moment of area of ANCF element in m^4
   EI = 0.02*Emodulus*I
   EA = Emodulus*A
   rhoA = rhoBeam*A
   dEI = 0
   dEA = 1 
   
   # EI *= 1000*2
   # EI  *= 500*5 #for test
   #%%
   
   #h = 1e-3 #step size
   tAccStart = 0.05
   tAccEnd = 0.6
   omegaFinal = 12
   
   useFriction = True
   dryFriction = 0.5#0.5#1.2
   contactStiffness = 1e8#2e5
   contactDamping = 0#1e-3*contactStiffness
   
   nSegments = 2 #4, for nANCFnodes=60, nSegments = 2 lead to less oscillations inside, but lot of stick-slip...
   nANCFnodes =2*2*30#2*60#120 works well, 60 leads to oscillatory tangent/normal forces for improvedBelt=True
   
   wheelMass = 50#1 the wheel mass is not given in the paper, only the inertia 
   # for the second pulley
   wheelInertia = 0.25#0.01
   rotationDampingWheels = 0 #zero in example in 2013 paper; torque proportional to rotation speed
   
   #torque = 1
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create circles
   #complicated shape:
   #initialDisplacement = -0.0025 #not used in improvedBelt!
   radiusPulley = 0.09995
   positionPulley2x = 0.1*pi
   #initialDistance = positionPulley2x
   initialLength = 2*positionPulley2x + 2* pi*(radiusPulley + hcStiff/2)
   #finalLength = initialLength - 2* initialDisplacement
   #preStretch = -(finalLength - initialLength)/ initialLength 
   
   #factorStriplen = (2*initialDistance+2*pi*radiusPulley)/(2*initialDistance+2*pi*(radiusPulley + hcStiff/2));
   #print('factorStriplen =', factorStriplen )
   #preStretch += (1-1./factorStriplen) #this is due to an error in the original paper 2013
   
   
   rotationDampingWheels = 2 #to reduce vibrations of driven pulley
   tEnd = 2.45 #at 2.45 node 1 is approximately at initial position!
   preStretch = -0.05
   staticEqulibrium = True
   #dryFriction = 0
   
   print('preStretch=', preStretch)
   circleList = [[[0,0], radiusPulley,'L'],
                 [[positionPulley2x,0], radiusPulley,'L'],
                 # [[initialDisplacement0,0], radiusPulley,'L'],              
                 # [[positionPulley2x,0], radiusPulley,'L'],
                 ]
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create geometry:
   reevingDict = CreateReevingCurve(circleList, drawingLinesPerCircle = 64, 
                                   radialOffset=0.5*hc, closedCurve=True, #allows closed curve
                                   numberOfANCFnodes=nANCFnodes, graphicsNodeSize= 0.01)
   
   
   
   # set precurvature at location of pulleys:
   elementCurvatures = reevingDict['elementCurvatures']
   
   gList=[]
   if False: #visualize reeving curve, without simulation
       gList = reevingDict['graphicsDataLines'] + reevingDict['graphicsDataCircles']
   
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(show=False)))#, visualization = {'show : False'}
   nGround = mbs.AddNode(NodePointGround())
   mCoordinateGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
   
   
   #mbs.SetObjectParameter(objectNumber = oGround, parameterName = 'Vshow', value=False)
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create ANCF elements:
   dimZ = b #z.dimension
   
   ANCFElementType = Cable2D
   nodesANCF = [-1,-1]
   if useALE:
       ANCFElementType = ALECable2D
       nALE = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=1, referenceCoordinates=[0], 
                                             initialCoordinates=[0], initialCoordinates_t=[0], 
                                             visualization = VNode1D(show = False)))#, color = [0.,0.,0.,1.])
       mALE = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nALE, coordinate=0, 
                                                 visualization = {'show':True})) #ALE velocity
       nodesANCF = [-1,-1, nALE]
   
       #Constraint for eulerian coordinate
       oCCvALE=mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mALE], offset=0, 
                                                   velocityLevel = False, 
                                                   visualization=VCoordinateConstraint(show=False)))
   
   cableTemplate = ANCFElementType(#physicsLength = L / nElements, #set in GenerateStraightLineANCFCable2D(...)
                           nodeNumbers = nodesANCF,
                           physicsMassPerLength = rhoA,
                           physicsBendingStiffness = EI,
                           physicsAxialStiffness = EA,
                           physicsBendingDamping = dEI,
                           physicsReferenceAxialStrain = preStretch,
                           physicsReferenceCurvature = 0.,
                           useReducedOrderIntegration = 2,
                           strainIsRelativeToReference = False, #would cause reference configuration to be precurved
                           visualization=VCable2D(drawHeight=hc),
                           )
   
   if useALE:
       cableTemplate.physicsAddALEvariation = False
   
   ancf = PointsAndSlopes2ANCFCable2D(mbs, reevingDict['ancfPointsSlopes'], 
                                      reevingDict['elementLengths'], 
                                      cableTemplate, massProportionalLoad=gVec, 
                                      fixedConstraintsNode0=[1*staticEqulibrium,0,0,0], #fixedConstraintsNode1=[1,1,1,1],
                                      #elementCurvatures  = elementCurvatures, #do not set this, will cause inhomogeneous curvatures
                                      firstNodeIsLastNode=True, graphicsSizeConstraints=0.01)
   
   lElem = reevingDict['totalLength'] / nANCFnodes
   cFact=b*lElem/nSegments #stiffness shall be per area, but is applied at every segment
   
   contactStiffness*=40*cFact
   contactDamping = 40*2000*cFact #according to Dufva 2008 paper ... seems also to be used in 2013 PEchstein Gerstmayr
   frictionStiffness = 50e8*cFact #1e7 converges good; 1e8 is already quite accurate
   massSegment = rhoA*lElem/nSegments
   frictionVelocityPenalty = 10*sqrt(frictionStiffness*massSegment) #bristle damping; should be adjusted to reduce vibrations induced by bristle model
   
   if useFrictionStiffness:
       frictionStiffness*=0.1
   else:
       frictionStiffness*=0
       frictionVelocityPenalty*= 2
       
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create sensors for all nodes
   sMidVel = []
   sAxialForce = []
   sCable0Pos = []
   # sObjectDisp =[] 
   
   ancfNodes = ancf[0]
   ancfObjects = ancf[1]
   positionList2Node = [] #axial position at x=0 and x=0.5*lElem
   positionListMid = [] #axial position at midpoint of element
   positionListSegments = [] #axial position at midpoint of segments
   currentPosition = 0 #is increased at every iteration
   for i,obj in enumerate(ancfObjects):
       lElem = reevingDict['elementLengths'][i]
       positionList2Node += [currentPosition, currentPosition + 0.5*lElem]
       positionListMid += [currentPosition + 0.5*lElem]
   
       for j in range(nSegments):
           segPos = (j+0.5)*lElem/nSegments + currentPosition
           positionListSegments += [segPos]
       currentPosition += lElem
   
       sAxialForce += [mbs.AddSensor(SensorBody(bodyNumber = obj, 
                                                 storeInternal=True,
                                                 localPosition=[0.*lElem,0,0], 
                                                 outputVariableType=exu.OutputVariableType.ForceLocal))]
       sAxialForce += [mbs.AddSensor(SensorBody(bodyNumber = obj, 
                                                 storeInternal=True,
                                                 localPosition=[0.5*lElem,0,0], 
                                                 outputVariableType=exu.OutputVariableType.ForceLocal))]
       sMidVel += [mbs.AddSensor(SensorBody(bodyNumber = obj, 
                                             storeInternal=True,
                                             localPosition=[0.5*lElem,0,0], #0=at left node
                                             outputVariableType=exu.OutputVariableType.VelocityLocal))]
       sCable0Pos += [mbs.AddSensor(SensorBody(bodyNumber = obj, 
                                               storeInternal=True,
                                               localPosition=[0.*lElem,0,0],
                                               outputVariableType=exu.OutputVariableType.Position))]
       # sObjectDisp += [mbs.AddSensor(SensorBody(bodyNumber = obj, 
       #                                           storeInternal=True,
       #                                           localPosition=[0.5*lElem,0,0],
       #                                           outputVariableType=exu.OutputVariableType.Displacement))]
   
   
       
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add contact:
   if useContact:
   
       contactObjects = [[],[]] #list of contact objects
       
       dimZ= 0.01 #for drawing
       sWheelRot = [] #sensors for angular velocity
   
       nMassList = []
       wheelSprings = [] #for static computation
       for i, wheel in enumerate(circleList):
           p = [wheel[0][0], wheel[0][1], 0] #position of wheel center
           r = wheel[1]
       
           rot0 = 0 #initial rotation
           pRef = [p[0], p[1], rot0]
           gList = [graphics.Cylinder(pAxis=[0,0,-dimZ],vAxis=[0,0,-dimZ], radius=r,
                                         color= graphics.color.dodgerblue, nTiles=64),
                    graphics.Arrow(pAxis=[0,0,0], vAxis=[-0.9*r,0,0], radius=0.01*r, color=graphics.color.orange),
                    graphics.Arrow(pAxis=[0,0,0], vAxis=[0.9*r,0,0], radius=0.01*r, color=graphics.color.orange)]
   
           omega0 = 0 #initial angular velocity
           v0 = np.array([0,0,omega0]) 
   
           nMass = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=pRef, initialVelocities=v0,
                                               visualization=VNodeRigidBody2D(drawSize=dimZ*2)))
           nMassList += [nMass]
           oMass = mbs.AddObject(ObjectRigidBody2D(physicsMass=wheelMass, physicsInertia=wheelInertia,
                                                   nodeNumber=nMass, visualization=
                                                   VObjectRigidBody2D(graphicsData=gList)))
           mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
           mGroundWheel = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=p, visualization = VMarkerBodyRigid(show = False)))
       
           #mbs.AddObject(RevoluteJoint2D(markerNumbers=[mGroundWheel, mNode], visualization=VRevoluteJoint2D(show=False)))
   
           mCoordinateWheelX = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=0))
           mCoordinateWheelY = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=1))
           constraintX = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCoordinateWheelX],
                                                    visualization=VCoordinateConstraint(show = False)))
           constraintY = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCoordinateWheelY],
                                                    visualization=VCoordinateConstraint(show = False)))
           if i==0:
               constraintPulleyLeftX = constraintX
   
           if True:
           
               sWheelRot += [mbs.AddSensor(SensorNode(nodeNumber=nMass, 
                                                      storeInternal=True,
                                                      fileName='solutionDelete/wheel'+str(i)+'angVel.txt',
                                                      outputVariableType=exu.OutputVariableType.AngularVelocity))]
           tdisplacement = 0.05
     
                            
           def UFvelocityDrive(mbs, t, itemNumber, lOffset): #time derivative of UFoffset
               if t < tAccStart:
                   v = 0
               if t >= tAccStart and t < tAccEnd:
                   v = omegaFinal/(tAccEnd-tAccStart)*(t-tAccStart)
               elif t >= tAccEnd:
                   v = omegaFinal
               return v    
           
           if doDynamic:    
               if i == 0:
                   mCoordinateWheel = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=2))
                   velControl = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCoordinateWheel],
                                                       velocityLevel=True, offsetUserFunction_t= UFvelocityDrive,
                                                       visualization=VCoordinateConstraint(show = False)))#UFvelocityDrive
               if i == 1:
                   mCoordinateWheel = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=2))
                   mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mCoordinateGround, mCoordinateWheel],
                                                        damping = rotationDampingWheels,
                                                        visualization=VCoordinateSpringDamper(show = False)))
                   
                   #this is used for times > 1 in order to see influence of torque step in Wheel1
                   def UFforce(mbs, t, load):
                       tau = 0.
                       tau +=  25.*(SmoothStep(t, 1., 1.5, 0., 1.) - SmoothStep(t, 3.5, 4., 0., 1.))
                       #tau += 16.*(SmoothStep(t, 5, 5.5, 0., 1.) - SmoothStep(t, 7.5, 8., 0., 1.))
                       return -tau
                   
                   mbs.AddLoad(LoadCoordinate(markerNumber=mCoordinateWheel,
                                              load = 0, loadUserFunction = UFforce))
   
           if staticEqulibrium:
               mCoordinateWheel = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=2))
               csd = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCoordinateWheel],
                                                        visualization=VCoordinateConstraint(show = False)))
               wheelSprings += [csd]
           
   
           cableList = ancf[1]
           mCircleBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass))
           #mCircleBody = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
           for k in range(len(cableList)):
               initialGapList = [0.1]*nSegments + [-2]*(nSegments) + [0]*(nSegments) #initial gap of 0., isStick (0=slip, +-1=stick, -2 undefined initial state), lastStickingPosition (0)
   
               mCable = mbs.AddMarker(MarkerBodyCable2DShape(bodyNumber=cableList[k], 
                                                             numberOfSegments = nSegments, verticalOffset=-0*hc/2))
               nodeDataContactCable = mbs.AddNode(NodeGenericData(initialCoordinates=initialGapList,
                                                                  numberOfDataCoordinates=nSegments*(1+2) ))
   
               co = mbs.AddObject(ObjectContactFrictionCircleCable2D(markerNumbers=[mCircleBody, mCable], nodeNumber = nodeDataContactCable, 
                                                        numberOfContactSegments=nSegments, 
                                                        contactStiffness = contactStiffness, 
                                                        contactDamping=contactDamping, 
                                                        frictionVelocityPenalty = frictionVelocityPenalty, 
                                                        frictionStiffness = frictionStiffness, 
                                                        frictionCoefficient=int(useFriction)*dryFriction,
                                                        circleRadius = r,
                                                        visualization=VObjectContactFrictionCircleCable2D(showContactCircle=False)))
               contactObjects[i] += [co]
   
   
   
   #+++++++++++++++++++++++++++++++++++++++++++
   #create list of sensors for contact
   sContactDisp = [[],[]]
   sContactForce = [[],[]]
   for i in range(len(contactObjects)):
       for obj in contactObjects[i]:
           sContactForce[i] += [mbs.AddSensor(SensorObject(objectNumber = obj, 
                                                           storeInternal=True,
                                                           outputVariableType=exu.OutputVariableType.ForceLocal))]
           sContactDisp[i] += [mbs.AddSensor(SensorObject(objectNumber = obj, 
                                                           storeInternal=True,
                                                           outputVariableType=exu.OutputVariableType.Coordinates))]
           
   
   mbs.Assemble()
   
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/testCoords.txt'
   
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.002
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.001
   simulationSettings.parallel.numberOfThreads = 1 #use 4 to speed up for > 100 ANCF elements
   
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.stepInformation= 255
   
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   #simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
   #simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
   
   simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-3
   simulationSettings.timeIntegration.discontinuous.maxIterations = discontinuousIterations #3
   
   simulationSettings.displayStatistics = True
   simulationSettings.displayComputationTime = False
   
   
   SC.visualizationSettings.general.circleTiling = 24
   SC.visualizationSettings.loads.show=False
   SC.visualizationSettings.sensors.show=False
   SC.visualizationSettings.markers.show=False
   SC.visualizationSettings.nodes.defaultSize = 0.002
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.openGL.lineWidth = 2
   SC.visualizationSettings.window.renderWindowSize = [1920,1080]
   
   SC.visualizationSettings.connectors.showContact = True
   SC.visualizationSettings.contact.contactPointsDefaultSize = 0.0002
   SC.visualizationSettings.contact.showContactForces = True
   SC.visualizationSettings.contact.contactForcesFactor = 0.005
   
   
   if True:
       SC.visualizationSettings.bodies.beams.axialTiling = 1
       SC.visualizationSettings.bodies.beams.drawVertical = True
       SC.visualizationSettings.bodies.beams.drawVerticalLines = True
   
       SC.visualizationSettings.contour.outputVariableComponent=0
       SC.visualizationSettings.contour.outputVariable=exu.OutputVariableType.ForceLocal
   
       SC.visualizationSettings.bodies.beams.drawVerticalFactor = 0.0003
       SC.visualizationSettings.bodies.beams.drawVerticalOffset = -220
           
       SC.visualizationSettings.bodies.beams.reducedAxialInterploation = True
       
       # SC.visualizationSettings.contour.outputVariable=exu.OutputVariableType.VelocityLocal
       # SC.visualizationSettings.bodies.beams.drawVerticalFactor = -0.25
       # SC.visualizationSettings.bodies.beams.drawVerticalOffset = 0.30
       # SC.visualizationSettings.bodies.beams.reducedAxialInterploation = False
   
   
   if useGraphics: 
       SC.renderer.Start()
       #SC.renderer.DoIdleTasks()
   
   #simulationSettings.staticSolver.newton.absoluteTolerance = 1e-10
   simulationSettings.staticSolver.adaptiveStep = True
   simulationSettings.staticSolver.loadStepGeometric = False;
   simulationSettings.staticSolver.loadStepGeometricRange=1e4
   simulationSettings.staticSolver.numberOfLoadSteps = 10
   #simulationSettings.staticSolver.useLoadFactor = False
   simulationSettings.staticSolver.stabilizerODE2term = 1e5*10
   simulationSettings.staticSolver.newton.relativeTolerance = 1e-6
   simulationSettings.staticSolver.newton.absoluteTolerance = 1e-6
   
   if staticEqulibrium: #precompute static equilibrium
       mbs.SetObjectParameter(velControl, 'activeConnector', False)
   
       for i in range(len(contactObjects)):
           for obj in contactObjects[i]:
               mbs.SetObjectParameter(obj, 'frictionCoefficient', 0.)
               mbs.SetObjectParameter(obj, 'frictionStiffness', 1e-8) #do not set to zero, as it needs to do some initialization...
               
       # simulationSettings.solutionSettings.appendToFile=False
       mbs.SolveStatic(simulationSettings, updateInitialValues=True)
       # simulationSettings.solutionSettings.appendToFile=True    
   
       #check total force on support, expect: supportLeftX \approx 2*preStretch*EA
       supportLeftX = mbs.GetObjectOutput(constraintPulleyLeftX,variableType=exu.OutputVariableType.Force)
       print('Force x in support of left pulley = ', supportLeftX)
       print('Belt pre-tension=', preStretch*EA)
       
       for i in range(len(contactObjects)):
           for obj in contactObjects[i]:
               mbs.SetObjectParameter(obj, 'frictionCoefficient', dryFriction)
               mbs.SetObjectParameter(obj, 'frictionStiffness', frictionStiffness)
   
       # useALE = False
       for coordinateConstraint in ancf[4]: #release constraints for dynamic Solver
           if not useALE: #except ALE constraint
               mbs.SetObjectParameter(coordinateConstraint, 'activeConnector', False)
       
       if useALE: 
           mbs.SetObjectParameter(oCCvALE, 'activeConnector', False) #do not fix ALE coordinate any more
           
           
       mbs.SetObjectParameter(velControl, 'activeConnector', True)
       for csd in wheelSprings:
           mbs.SetObjectParameter(csd, 'activeConnector', False)
   
   if True:
       mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2) #183 Newton iterations, 0.114 seconds
   #mbs.SolveDynamic(simulationSettings)
   
   if useGraphics and True:
       SC.visualizationSettings.general.autoFitScene = False
       SC.visualizationSettings.general.graphicsUpdateInterval=0.02
       
       sol = LoadSolutionFile('solution/testCoords.txt', safeMode=True)#, maxRows=100)
       mbs.SolutionViewer(sol)
   
   
   if useGraphics: 
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   #%%++++++++++++++++++++++++++++++++++++++++
   if True:
       solDir = 'solutionDelete/'
       #shift data depending on axial position by subtracting xOff; put negative x values+shiftValue to end of array
       def ShiftXoff(data, xOff, shiftValue):
           indOff = 0
           n = data.shape[0]
           data[:,0] -= xOff
           for i in range(n):
              if data[i,0] < 0:
                  indOff+=1
                  data[i,0] += shiftValue
           print('indOff=', indOff)
           data = np.vstack((data[indOff:,:], data[0:indOff,:]))
           return data
                  
       import matplotlib.pyplot as plt
       import matplotlib.ticker as ticker
       from exudyn.plot import DataArrayFromSensorList
   
       mbs.PlotSensor(closeAll=True)
       
       #compute axial offset, to normalize results:
       nodePos0 = mbs.GetSensorValues(sCable0Pos[0])
       xOff = nodePos0[0]
       maxXoff = 0.5*positionPulley2x
       maxYoff = 0.1*r
       # indOff = 0 #single data per element
       # indOff2 = 0 #double data per element
       correctXoffset = True
       if abs(nodePos0[1]-r) > maxYoff or nodePos0[0] > maxXoff or nodePos0[0] < -0.1*maxXoff:
           print('*****************')
           print('warning: final position not at top of belt or too far away')
           print('nodePos0=',nodePos0)
           print('*****************')
           xOff = 0
           correctXoffset = False
       else:
           print('******************\nxOff=', xOff)
               
       
       dataVel = DataArrayFromSensorList(mbs, sensorNumbers=sMidVel, positionList=positionListMid)
       if correctXoffset:
           dataVel=ShiftXoff(dataVel,xOff, reevingDict['totalLength'])
       
       # mbs.PlotSensor(sensorNumbers=[dataVel], components=0, labels=['axial velocity'], 
       #            xLabel='axial position (m)', yLabel='velocity (m/s)')
   
       #axial force over beam length:
       dataForce = DataArrayFromSensorList(mbs, sensorNumbers=sAxialForce, positionList=positionList2Node)
       if correctXoffset:
           dataForce = ShiftXoff(dataForce,xOff, reevingDict['totalLength'])
       # mbs.PlotSensor(sensorNumbers=[dataForce], components=0, labels=['axial force'], colorCodeOffset=2,
       #            xLabel='axial position (m)', yLabel='axial force (N)')
   
   
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #contact forces are stored (x/y) for every segment ==> put into consecutive array
       contactForces =[[],[]] #these are the contact forces of the whole belt, but from both pulleys!
       for i in range(len(sContactForce)):
           contactForces[i] = np.zeros((len(sContactForce[i])*nSegments, 3)) #per row: [position, Fx, Fy]
           for j, sensor in enumerate(sContactForce[i]):
               values = mbs.GetSensorValues(sensor)
               for k in range(nSegments):
                   row = j*nSegments + k
                   contactForces[i][row,0] = positionListSegments[row]
                   contactForces[i][row, 1:] = values[k*2:k*2+2]
   
       contactForcesTotal = contactForces[0]
       contactForcesTotal[:,1:] += contactForces[1][:,1:]
   
       if correctXoffset:
           contactForcesTotal = ShiftXoff(contactForcesTotal,-xOff, reevingDict['totalLength'])
       #plot contact forces over beam length:
       mbs.PlotSensor(sensorNumbers=[contactForcesTotal,contactForcesTotal], components=[0,1], labels=['tangential force','normal force'], 
                  xLabel='axial position (m)', yLabel='contact forces (N)', newFigure=True)
       # mbs.PlotSensor(sensorNumbers=[contactForces[1],contactForces[1]], components=[0,1], labels=['tangential force','normal force'], 
       #            xLabel='axial position (m)', yLabel='contact forces (N)', newFigure=False)
       mbs.PlotSensor(sensorNumbers=[solDir+'contactForcesh5e-05n2s2cs40ALE1.txt'], components=[0,1], 
                  labels=['tangential force ALE','normal force ALE'], 
                   xLabel='axial position (m)', yLabel='contact forces (N)', colorCodeOffset=2, newFigure=False)
       mbs.PlotSensor(sensorNumbers=[solDir+'contactForcesh5e-05n2s2cs40ALE0.txt'], components=[0,1], 
                  labels=['tangential force Ref','normal force Ref'], 
                   xLabel='axial position (m)', yLabel='contact forces (N)', colorCodeOffset=4, newFigure=False)
   
       contactDisp =[[],[]] #slip and gap
       for i in range(len(sContactDisp)):
           contactDisp[i] = np.zeros((len(sContactDisp[i])*nSegments, 3)) #per row: [position, Fx, Fy]
           for j, sensor in enumerate(sContactDisp[i]):
               values = mbs.GetSensorValues(sensor)
               for k in range(nSegments):
                   row = j*nSegments + k
                   contactDisp[i][row,0] = positionListSegments[row]
                   contactDisp[i][row, 1:] = values[k*2:k*2+2]
   
       if correctXoffset:
           contactDisp[0] = ShiftXoff(contactDisp[0],-xOff, reevingDict['totalLength'])
           contactDisp[1] = ShiftXoff(contactDisp[1],-xOff, reevingDict['totalLength'])
   
       if False:
           mbs.PlotSensor(sensorNumbers=[contactDisp[0],contactDisp[0]], components=[0,1], labels=['slip','gap'], 
                      xLabel='axial position (m)', yLabel='slip, gap (m)', newFigure=True)
           mbs.PlotSensor(sensorNumbers=[contactDisp[1],contactDisp[1]], components=[0,1], labels=['slip','gap'], 
                      xLabel='axial position (m)', yLabel='slip, gap (m)', newFigure=False)
           mbs.PlotSensor(sensorNumbers=[solDir+'contactDisph5e-05n2s2cs40ALE1.txt'], components=[0,1], labels=['slipALE','gapALE'], 
                      xLabel='axial position (m)', yLabel='slip, gap (m)', colorCodeOffset=2, newFigure=False)
       
   
   
       header  = ''
       header += 'endTime='+str(tEnd)+'\n'
       header += 'stepSize='+str(stepSize)+'\n'
       header += 'nSegments='+str(nSegments)+'\n'
       header += 'nANCFnodes='+str(nANCFnodes)+'\n'
       header += 'contactStiffness='+str(contactStiffness)+'\n'
       header += 'contactDamping='+str(contactDamping)+'\n'
       header += 'frictionStiffness='+str(frictionStiffness)+'\n'
       header += 'frictionVelocityPenalty='+str(frictionVelocityPenalty)+'\n'
       header += 'dryFriction='+str(dryFriction)+'\n'
       header += 'useALE='+str(useALE)+'\n'
       fstr  = 'h'+str(stepSize)+'n'+str(int(nANCFnodes/60))+'s'+str(nSegments)+'cs'+str(int((contactStiffness/41800)))
       fstr += 'ALE'+str(int(useALE))
       #fstr += 'fs'+str(int((frictionStiffness/52300)))
   
       #export solution:
       contactDispSave = contactDisp[0]
       contactDispSave[:,1:] += contactDisp[1][:,1:]
   
       if False: #for saving
           np.savetxt(solDir+'contactForces'+fstr+'.txt', contactForcesTotal, delimiter=',', 
                      header='Exudyn: solution of belt drive, contact forces over belt length\n'+header, encoding=None)
           np.savetxt(solDir+'contactDisp'+fstr+'.txt', contactDispSave, delimiter=',', 
                      header='Exudyn: solution of belt drive, slip and gap over belt length\n'+header, encoding=None)
   
   
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
       if False:
           # mbs.PlotSensor(sensorNumbers=[sWheelRot[0], sWheelRot[1]], components=[2,2])#,sWheelRot[1]
           mbs.PlotSensor(sensorNumbers=[sWheelRot[0], sWheelRot[1], 
                                          solDir+'wheel0angVelALE.txt',solDir+'wheel1angVelALE.txt'], components=[2,2,2,2],
                      colorCodeOffset=2)#,sWheelRot[1]
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   


