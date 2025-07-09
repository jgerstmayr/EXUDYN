
.. _examples-beltdrivereevingsystem:

*************************
beltDriveReevingSystem.py
*************************

You can view and download this file on Github: `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_

.. code-block:: python
   :linenos:

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Model for belt drive; According to thread deliverable D2.2 Test case
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-02-27
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
   
   improvedBelt = True #True: improved belt model (tEnd ~= 2.5 seconds simulation, more damping, better initial conditions, etc.)
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Parameters for the belt
   gVec = [0,-9.81*1,0]     # gravity
   Emodulus=1e7*1           # Young's modulus of ANCF element in N/m^2
   b=0.08 #0.002            # width of rectangular ANCF element in m
   hc = 0.01                # height (geometric) of rectangular ANCF element in m
   hcStiff = 0.01           # stiffness relevant height
   rhoBeam=1036.            # density of ANCF element in kg/m^3
   A=b*hcStiff              # cross sectional area of ANCF element in m^2
   I=(b*hcStiff**3)/12      # second moment of area of ANCF element in m^4
   EI = Emodulus*I
   EA = Emodulus*A
   rhoA = rhoBeam*A
   dEI = 0*1e-3*Emodulus*I         #REMARK: bending proportional damping. Set zero in the 2013 paper there is not. We need the damping for changing the initial configuration.
   #dEA = 0.1*1e-2*Emodulus*A          #axial strain proportional damping. Same as for the
   dEA = 1 #dEA=1 in paper PechsteinGerstmayr 2013, according to HOTINT C++ files ...
   # bending damping.
   #%%
   
   
   #%%
   #settings:
   useGraphics= True
   useContact = True
   doDynamic = True
   makeAnimation = False
   velocityControl = True
   staticEqulibrium = False #False in 2013 paper; starts from pre-deformed reference configuration
   useBristleModel = improvedBelt
   
   #in 2013 paper, reference curvature is set according to initial geometry and released until tAccStart
   preCurved = False #uses preCurvature according to straight and curved initial segments
   strainIsRelativeToReference = 1. #0: straight reference, 1.: curved reference
   useContactCircleFriction = True
   
   movePulley = False #as in 2013 paper, move within first 0.05 seconds; but this does not work with Index 2 solver
   
   tEnd = 1#1*2.25#*0.1 #*5 #end time of dynamic simulation
   stepSize = 0.25*1e-4  #accurate: 2.5e-5 # for frictionVelocityPenalty = 1e7*... it must be not larger than 2.5e-5
   if improvedBelt:
       stepSize = 1e-4
   discontinuousIterations = 3 #larger is more accurate, but smaller step size is equivalent
   
   #h = 1e-3 #step size
   tAccStart = 0.05
   tAccEnd = 0.6
   omegaFinal = 12
   
   useFriction = True
   dryFriction = 0.5#0.5#1.2
   contactStiffness = 1e8#2e5
   contactDamping = 0#1e-3*contactStiffness
   
   nSegments = 2 #4, for nANCFnodes=60, nSegments = 2 lead to less oscillations inside, but lot of stick-slip...
   nANCFnodes = 8*30#2*60#120 works well, 60 leads to oscillatory tangent/normal forces for improvedBelt=True
   
   wheelMass = 50#1 the wheel mass is not given in the paper, only the inertia 
   # for the second pulley
   wheelInertia = 0.25#0.01
   rotationDampingWheels = 0 #zero in example in 2013 paper; torque proportional to rotation speed
   
   #torque = 1
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create circles
   #complicated shape:
   initialDisplacement = -0.0025 #not used in improvedBelt!
   initialDisplacement0 = initialDisplacement*int(1-movePulley) #this is set at t=0
   
   #h = 0.25e-3
   radiusPulley = 0.09995
   positionPulley2x = 0.1*pi
   #preStretch = -1*(pi*0.4099+0.005)/ pi*0.4099
   initialDistance = positionPulley2x - 0
   initialLength = 2*initialDistance +2* pi*(radiusPulley + hcStiff/2)
   finalLength = initialLength - 2* initialDisplacement0
   preStretch = -(finalLength - initialLength)/ initialLength 
   
   factorStriplen = (2*initialDistance+2*pi*radiusPulley)/(2*initialDistance+2*pi*(radiusPulley + hcStiff/2));
   print('factorStriplen =', factorStriplen )
   
   preStretch += (1-1./factorStriplen) #this is due to an error in the original paper 2013
   
   
   if improvedBelt:
       rotationDampingWheels = 2 #to reduce vibrations of driven pulley
       tEnd = 2.45 #at 2.45 node 1 is approximately at initial position!
       preStretch = -0.05
       initialDisplacement0 = 0
       staticEqulibrium = True
       strainIsRelativeToReference = False
       #dryFriction = 0
       hc *= 0.01
       EI *= 0.02
   
   print('preStretch=', preStretch)
   circleList = [[[initialDisplacement0,0], radiusPulley,'L'],
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
   elementCurvatures = [] #no pre-curvatures
   if preCurved:
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
   
   cableTemplate = Cable2D(#physicsLength = L / nElements, #set in GenerateStraightLineANCFCable2D(...)
                           physicsMassPerLength = rhoA,
                           physicsBendingStiffness = EI,
                           physicsAxialStiffness = EA,
                           physicsBendingDamping = dEI,
                           physicsAxialDamping = dEA,
                           physicsReferenceAxialStrain = preStretch*int(improvedBelt), #prestretch
                           physicsReferenceCurvature = 0.,#-1/(radiusPulley + hc/2),
                           useReducedOrderIntegration = 2, #2=improved axial strain in postprocessing!
                           strainIsRelativeToReference = strainIsRelativeToReference,
                           visualization=VCable2D(drawHeight=hc),
                           )
   
   ancf = PointsAndSlopes2ANCFCable2D(mbs, reevingDict['ancfPointsSlopes'], 
                                      reevingDict['elementLengths'], 
                                      cableTemplate, massProportionalLoad=gVec, 
                                      fixedConstraintsNode0=[1*staticEqulibrium,0,0,0], #fixedConstraintsNode1=[1,1,1,1],
                                      elementCurvatures  = elementCurvatures,
                                      firstNodeIsLastNode=True, graphicsSizeConstraints=0.01)
   
   if useContactCircleFriction:
       lElem = reevingDict['totalLength'] / nANCFnodes
       cFact=b*lElem/nSegments #stiffness shall be per area, but is applied at every segment
       print('cFact=',cFact, ', lElem=', lElem)
   
       contactStiffness*=cFact
       contactDamping = 2000*cFact #according to Dufva 2008 paper ... seems also to be used in 2013 PEchstein Gerstmayr
       if useBristleModel:
           frictionStiffness = 1e8*cFact #1e7 converges good; 1e8 is already quite accurate
           massSegment = rhoA*lElem/nSegments
           frictionVelocityPenalty = 1*sqrt(frictionStiffness*massSegment) #bristle damping; should be adjusted to reduce vibrations induced by bristle model
       else:
           frictionVelocityPenalty = 0.1*1e7*cFact #1e7 is original in 2013 paper; requires smaller time step
           #frictionVelocityPenalty = 0.25e7*cFact # 0.25e7*cFact  with  discontinuous.maxIterations = 4
           frictionStiffness = 0 #as in 2013 paper
   
   if improvedBelt:
       frictionStiffness *= 10*5
       frictionVelocityPenalty *= 10
       contactStiffness *= 10*4
       contactDamping *=10*4
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
   
   
   #for testing, fix two nodes:
   if False:
       ii0 = 1
       ii1 = 14
       for i in range(4):
           mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=ancfNodes[ii0], coordinate=i))
           mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=ancfNodes[ii1], coordinate=i))
           mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mANCF0],
                                              visualization=VCoordinateConstraint(show = False)))
           mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mANCF1],
                                              visualization=VCoordinateConstraint(show = False)))
   
       #reference solution for clamped-clamped beam:
       lElem = reevingDict['elementLengths'][0] #all same
       L = lElem * 13 #span is 13 elements long
       wMax = rhoA*9.81*L**4 /(384*EI)
       print('wMax=',wMax)
       
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add contact:
   if useContact:
   
       contactObjects = [[],[]] #list of contact objects
       
       gContact = mbs.AddGeneralContact()
       gContact.verboseMode = 1
   
       gContact.frictionProportionalZone = 0.005 #limit velocity. I didn't find 
       #gContact.frictionVelocityPenalty = 0*1e3   #limit velocity. I didn't find 
       #this in the paper
       gContact.ancfCableUseExactMethod = False
       gContact.ancfCableNumberOfContactSegments = nSegments
       ssx = 16#32 #search tree size
       ssy = 16#32 #search tree size
       ssz = 1 #search tree size
       gContact.SetSearchTreeCellSize(numberOfCells=[ssx,ssy,ssz])
       #gContact.SetSearchTreeBox(pMin=np.array([-1,-1,-1]), pMax=np.array([4,1,1])) #automatically computed!
   
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
                   if velocityControl:
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
           
   
           frictionMaterialIndex=0
           gContact.AddSphereWithMarker(mNode, radius=r, contactStiffness=contactStiffness, 
                                        contactDamping=contactDamping, frictionMaterialIndex=frictionMaterialIndex)
   
           if not useContactCircleFriction:
               for oIndex in ancf[1]:
                   gContact.AddANCFCable(objectIndex=oIndex, halfHeight= hc/2, #halfHeight should be h/2, but then cylinders should be smaller
                                         contactStiffness=contactStiffness, contactDamping=contactDamping, 
                                         frictionMaterialIndex=0)
           else:
               cableList = ancf[1]
               mCircleBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass))
               #mCircleBody = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
               for k in range(len(cableList)):
                   initialGapList = [0.1]*nSegments + [-2]*(nSegments) + [0]*(nSegments) #initial gap of 0., isStick (0=slip, +-1=stick, -2 undefined initial state), lastStickingPosition (0)
   
                   mCable = mbs.AddMarker(MarkerBodyCable2DShape(bodyNumber=cableList[k], 
                                                                 numberOfSegments = nSegments, verticalOffset=-hc/2))
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
   
       frictionMatrix = np.zeros((2,2))
       frictionMatrix[0,0]=int(useFriction)*dryFriction
       frictionMatrix[0,1]=0 #no friction between some rolls and cable
       frictionMatrix[1,0]=0 #no friction between some rolls and cable
       gContact.SetFrictionPairings(frictionMatrix)
   
   
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
           
   
   
   #user function to smoothly transform from curved to straight reference configuration as
   #in paper 2013, Pechstein, Gerstmayr
   def PreStepUserFunction(mbs, t):
   
       if True and t <= tAccStart+1e-10:
           cableList = ancf[1]
           fact = (tAccStart-t)/tAccStart #from 1 to 0
           if fact < 1e-12: fact = 0. #for very small values ...
           #curvatures = reevingDict['elementCurvatures']
           #print('fact=', fact)
           for i in range(len(cableList)):
               oANCF = cableList[i]
               mbs.SetObjectParameter(oANCF, 'strainIsRelativeToReference', 
                                      fact)
               mbs.SetObjectParameter(oANCF, 'physicsReferenceAxialStrain', 
                                       preStretch*(1.-fact))
   
           # if movePulley:
           #     #WARNING: this does not work for Index2 solver:
           #     mbs.SetObjectParameter(constraintPulleyLeftX, 'offset', initialDisplacement*(1.-fact))
           #     #print('offset=', initialDisplacement*(1.-fact))
       
       return True
   
   
   mbs.Assemble()
   
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution_nosync/testCoords.txt'
   
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.002
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.001
   simulationSettings.displayComputationTime = True
   simulationSettings.parallel.numberOfThreads = 1 #use 4 to speed up for > 100 ANCF elements
   simulationSettings.displayStatistics = True
   
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
   
   if makeAnimation == True:
       simulationSettings.solutionSettings.recordImagesInterval = 0.02
       SC.visualizationSettings.exportImages.saveImageFileName = "animationNew/frame"
   
   
   if True:
       SC.visualizationSettings.bodies.beams.axialTiling = 1
       SC.visualizationSettings.bodies.beams.drawVertical = True
       SC.visualizationSettings.bodies.beams.drawVerticalLines = True
   
       SC.visualizationSettings.contour.outputVariableComponent=0
       SC.visualizationSettings.contour.outputVariable=exu.OutputVariableType.ForceLocal
       SC.visualizationSettings.bodies.beams.drawVerticalFactor = 0.001
       SC.visualizationSettings.bodies.beams.drawVerticalOffset = -120
       if improvedBelt:
           SC.visualizationSettings.bodies.beams.drawVerticalFactor = 0.0003
           SC.visualizationSettings.bodies.beams.drawVerticalOffset = -220
           
       SC.visualizationSettings.bodies.beams.reducedAxialInterploation = True
       
       # SC.visualizationSettings.contour.outputVariable=exu.OutputVariableType.VelocityLocal
       # SC.visualizationSettings.bodies.beams.drawVerticalFactor = -0.25
       # SC.visualizationSettings.bodies.beams.drawVerticalOffset = 0.30
       # SC.visualizationSettings.bodies.beams.reducedAxialInterploation = False
   
   #visualize contact:
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
   simulationSettings.staticSolver.loadStepGeometricRange=1e4
   simulationSettings.staticSolver.numberOfLoadSteps = 10
   #simulationSettings.staticSolver.useLoadFactor = False
   simulationSettings.staticSolver.stabilizerODE2term = 1e5
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
   
       for coordinateConstraint in ancf[4]:
           mbs.SetObjectParameter(coordinateConstraint, 'activeConnector', False)
           
       mbs.SetObjectParameter(velControl, 'activeConnector', True)
       for csd in wheelSprings:
           mbs.SetObjectParameter(csd, 'activeConnector', False)
   else:
       mbs.SetPreStepUserFunction(PreStepUserFunction)
   
   if True:
       mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2) #183 Newton iterations, 0.114 seconds
   #mbs.SolveDynamic(simulationSettings)
   
   if useGraphics and False:
       SC.visualizationSettings.general.autoFitScene = False
       SC.visualizationSettings.general.graphicsUpdateInterval=0.02
       
       sol = LoadSolutionFile('solution_nosync/testCoords.txt', safeMode=True)#, maxRows=100)
       mbs.SolutionViewer(sol)
   
   
   if useGraphics: 
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   
   #%%++++++++++++++++++++++++++++++++++++++++
   if False:
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
           #compute offset index:
           # for (i,s) in enumerate(sCable0Pos):
           #     p = mbs.GetSensorValues(s)
           #     print('p'+str(i)+'=', p)
           #     if p[0] > 0  and i > int(0.8*nANCFnodes):
           #         indOff+=1
           #         indOff2+=2
           # indOff -= 1
           # indOff2 -= 2
           print('******************\nxOff=', xOff)
               
       
       dataVel = DataArrayFromSensorList(mbs, sensorNumbers=sMidVel, positionList=positionListMid)
       if correctXoffset:
           dataVel=ShiftXoff(dataVel,xOff, reevingDict['totalLength'])
       
       mbs.PlotSensor(sensorNumbers=[dataVel], components=0, labels=['axial velocity'], 
                  xLabel='axial position (m)', yLabel='velocity (m/s)')
   
       #axial force over beam length:
       dataForce = DataArrayFromSensorList(mbs, sensorNumbers=sAxialForce, positionList=positionList2Node)
       if correctXoffset:
           dataForce = ShiftXoff(dataForce,xOff, reevingDict['totalLength'])
       mbs.PlotSensor(sensorNumbers=[dataForce], components=0, labels=['axial force'], colorCodeOffset=2,
                  xLabel='axial position (m)', yLabel='axial force (N)')
   
   
       if improvedBelt and dryFriction==0.5 and nANCFnodes==120:
           #analytical exponential curve, Euler's/Eytelwein's solution:
           na = 12 #number of data points
           dataExp = np.zeros((na*2, 2))
           #f0 = 278.733 #this is at low level, but exp starts later
           f0 = 191.0#287.0
           x0 = 0.5860 #1.1513 #starting coordinate, drawn in -x direction
           d = 0.28     #amount along x drawn
           for i in range(na):
               x = i/na*d
               beta = x/(radiusPulley + hc/2)
               val = f0*exp(beta*dryFriction)
               #print('x=',x,',exp=',val)
               dataExp[i,0] = x0-x
               dataExp[i,1] = val
       
           f0 = 193.4#287.0
           x0 = 0.984 #1.1513 #starting coordinate, drawn in -x direction
           for i in range(na):
               x = i/na*d
               beta = x/(radiusPulley + hc/2)
               val = f0*exp(beta*dryFriction)
               dataExp[i+na,0] = x0+x
               dataExp[i+na,1] = val
   
           mbs.PlotSensor(sensorNumbers=[dataExp], components=0, labels=['analytical Eytelwein'], colorCodeOffset=3, newFigure=False,
                      lineStyles=[''], markerStyles=['x '], markerDensity=2*na)
   
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
   
       #plot contact forces over beam length:
       mbs.PlotSensor(sensorNumbers=[contactForcesTotal,contactForcesTotal], components=[0,1], labels=['tangential force','normal force'], 
                  xLabel='axial position (m)', yLabel='contact forces (N)', newFigure=True)
       # mbs.PlotSensor(sensorNumbers=[contactForces[1],contactForces[1]], components=[0,1], labels=['tangential force','normal force'], 
       #            xLabel='axial position (m)', yLabel='contact forces (N)', newFigure=False)
   
       contactDisp =[[],[]] #slip and gap
       for i in range(len(sContactDisp)):
           contactDisp[i] = np.zeros((len(sContactDisp[i])*nSegments, 3)) #per row: [position, Fx, Fy]
           for j, sensor in enumerate(sContactDisp[i]):
               values = mbs.GetSensorValues(sensor)
               for k in range(nSegments):
                   row = j*nSegments + k
                   contactDisp[i][row,0] = positionListSegments[row]
                   contactDisp[i][row, 1:] = values[k*2:k*2+2]
   
       mbs.PlotSensor(sensorNumbers=[contactDisp[0],contactDisp[0]], components=[0,1], labels=['slip','gap'], 
                  xLabel='axial position (m)', yLabel='slip, gap (m)', newFigure=True)
       mbs.PlotSensor(sensorNumbers=[contactDisp[1],contactDisp[1]], components=[0,1], labels=['slip','gap'], 
                  xLabel='axial position (m)', yLabel='slip, gap (m)', newFigure=False)
   
   
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
       fstr  = 'h'+str(stepSize)+'n'+str(int(nANCFnodes/60))+'s'+str(nSegments)+'cs'+str(int((contactStiffness/41800)))
       fstr += 'fs'+str(int((frictionStiffness/52300)))
   
       #export solution:
       if improvedBelt:
           np.savetxt('solutionDelete/contactForces'+fstr+'.txt', contactForces[0]+contactForces[1], delimiter=',', 
                      header='Exudyn: solution of belt drive, contact forces over belt length\n'+header, encoding=None)
           np.savetxt('solutionDelete/contactDisp'+fstr+'.txt', contactDisp[0]+contactDisp[1], delimiter=',', 
                      header='Exudyn: solution of belt drive, slip and gap over belt length\n'+header, encoding=None)
       
   
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
       mbs.PlotSensor(sensorNumbers=[sWheelRot[0], sWheelRot[1]], components=[2,2])#,sWheelRot[1]
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   


