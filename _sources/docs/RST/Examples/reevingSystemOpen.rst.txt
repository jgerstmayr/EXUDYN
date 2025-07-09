
.. _examples-reevingsystemopen:

********************
reevingSystemOpen.py
********************

You can view and download this file on Github: `reevingSystemOpen.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/reevingSystemOpen.py>`_

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
   from exudyn.robotics import *
   
   import numpy as np
   from math import sin, cos, pi, sqrt , asin, acos, atan2
   import copy 
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #+++  MOTION PLANNING and TRAJECTORIES  +++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #**function: Compute parameters for optimal trajectory using given duration and distance
   #**notes: DEPRECATED, DO NOT USE - moved to robotics.motion
   #**input: duration in seconds and distance in meters or radians
   #**output: returns [vMax, accMax] with maximum velocity and maximum acceleration to achieve given trajectory
   def ConstantAccelerationParameters(duration, distance):
       accMax = 4*distance/duration**2
       vMax = (accMax * distance)**0.5
       return [vMax, accMax]
   
   #**function: Compute angle / displacement s, velocity v and acceleration a
   #**input: 
   #  t: current time to compute values
   #  tStart: start time of profile
   #  sStart: start offset of path
   #  duration: duration of profile
   #  distance: total distance (of path) of profile
   #**notes: DEPRECATED, DO NOT USE - moved to robotics.motion
   #**output: [s, v, a] with path s, velocity v and acceleration a for constant acceleration profile; before tStart, solution is [0,0,0] while after duration, solution is [sStart+distance, 0, 0]
   def ConstantAccelerationProfile(t, tStart, sStart, duration, distance):
       [vMax, accMax] = ConstantAccelerationParameters(duration, distance)
       
       s = sStart
       v = 0
       a = 0
       
       x = t-tStart
       if x < 0:
           s=0
       elif x < 0.5*duration:
           s = sStart + 0.5*accMax*x**2
           v = x*accMax
           a = accMax
       elif x < duration:
           s = sStart + distance - 0.5*accMax * (duration-x)**2
           v = (duration - x)*accMax
           a = -accMax
       else:
           s = sStart + distance
       
       return [s, v, a]
   
   #**function: Compute joint value, velocity and acceleration for given robotTrajectory['PTP'] of point-to-point type, evaluated for current time t and joint number
   #**input:
   #  t: time to evaluate trajectory
   #  robotTrajectory: dictionary to describe trajectory; in PTP case, either use 'time' points, or 'time' and 'duration', or 'time' and 'maxVelocity' and 'maxAccelerations' in all consecutive points; 'maxVelocities' and 'maxAccelerations' must be positive nonzero values that limit velocities and accelerations; 
   #  joint: joint number for which the trajectory shall be evaluated
   #**output: for current time t it returns [s, v, a] with path s, velocity v and acceleration a for current acceleration profile; outside of profile, it returns [0,0,0] !
   #**notes: DEPRECATED, DO NOT USE - moved to robotics.motion
   #**example:
   # q0 = [0,0,0,0,0,0] #initial configuration
   # q1 = [8,5,2,0,2,1] #other configuration
   # PTP =[]
   # PTP+=[{'q':q0, 
   #        'time':0}]
   # PTP+=[{'q':q1,
   #        'time':0.5}]
   # PTP+=[{'q':q1, 
   #        'time':1e6}] #forever
   # RT={'PTP':PTP}
   # [u,v,a] = MotionInterpolator(t=0.5, robotTrajectory=RT, joint=1)
   def MotionInterpolator(t, robotTrajectory, joint):
   
       n = len(robotTrajectory['PTP'])
       if n < 2:
           print("ERROR in MotionInterpolator: trajectory must have at least 2 points!")
       
       i = 0
       while (i < n) and (t >= robotTrajectory['PTP'][i]['time']):
           i += 1
   
       if (i==0) or (i==n):
           return [0,0,0] #outside of trajectory
       
       #i must be > 0 and < n now!
       q0 = robotTrajectory['PTP'][i-1] #must always exist
       q1 = robotTrajectory['PTP'][i] #must always exist
       
       return ConstantAccelerationProfile(t, q0['time'], q0['q'][joint], 
                                          q1['time'] - q0['time'], 
                                          q1['q'][joint] - q0['q'][joint])
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #settings:
   useGraphics= True
   useContact = True
   useFriction = True
   # kProp = 10
   dryFriction = 2*0.5
   contactStiffness = 1e5
   contactDamping = 2e-3*contactStiffness
   
   wheelMass = 1
   wheelInertia = 0.01
   
   rotationDampingWheels = 0.01 #proportional to rotation speed
   torque = 1
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create circles
   #complicated shape:
   nANCFnodes = 1*50
   stepSize = 5e-4
   tEnd = 10
   R=0.45
   preStretch=-0.002*0 #not needed here, system is open!
   circleList = [
                 [[  0,-3  ],R,'L'],
                 [[  0,0   ],R,'L'],
                 [[0.5,-0.8],R,'R'],
                 [[  1,0   ],R,'L'],
                 [[  1,-3  ],R,'L'],
                 ]
   
   #create motion profile:
   point0={'q':[0],  #initial configuration
           'time':0}
   point1={'q':[2.5/R],
           'time':2}
   point2={'q':[-2.5/R],
           'time':4}
   point3={'q':[0],
           'time':5}
   pointLast={'q':[0], #add this a second time to stay this forever
           'time':1e6} #forever
   RT={'PTP':[point0,point1,point2,point3,pointLast]}
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create geometry:
   reevingDict = CreateReevingCurve(circleList, drawingLinesPerCircle = 64, 
                                    removeLastLine=False, #allows closed curve
                                    numberOfANCFnodes=nANCFnodes, graphicsNodeSize= 0.01)
   
   del circleList[-1] #remove circles not needed for contact/visualization
   del circleList[0] #remove circles not needed for contact/visualization
   
   gList=[]
   if False: #visualize reeving curve, without simulation
       gList = reevingDict['graphicsDataLines'] + reevingDict['graphicsDataCircles']
   
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], 
                                      visualization=VObjectGround(graphicsData= gList)))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create ANCF elements:
   
   gVec = np.array([0,-9.81,0])      # gravity
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
                                      firstNodeIsLastNode=False, graphicsSizeConstraints=0.01)
   
   #+++++++++++++++++++++++++++++++++++++++
   #add weights
   node0 = ancf[0][0]
   nodeL = ancf[0][-1]
   massLoad=2
   
   bMass0 = mbs.AddObject(ObjectMassPoint2D(physicsMass=massLoad, 
                                            nodeNumber=node0,
                                   visualization=VMassPoint2D(graphicsData=[graphics.Sphere(radius=0.1, nTiles=32)])))
   bMassL = mbs.AddObject(ObjectMassPoint2D(physicsMass=massLoad, 
                                            nodeNumber=nodeL,
                                   visualization=VMassPoint2D(graphicsData=[graphics.Sphere(radius=0.1, nTiles=32)])))
   
   mBody0=mbs.AddMarker(MarkerBodyPosition(bodyNumber=bMass0))
   mbs.AddLoad(Force(markerNumber=mBody0, loadVector=massLoad*gVec))
   mBodyL=mbs.AddMarker(MarkerBodyPosition(bodyNumber=bMassL))
   mbs.AddLoad(Force(markerNumber=mBodyL, loadVector=massLoad*gVec))
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add contact:
   if useContact:
   
       gContact = mbs.AddGeneralContact()
       gContact.verboseMode = 1
       gContact.frictionProportionalZone = 0.5
       gContact.ancfCableUseExactMethod = False
       gContact.ancfCableNumberOfContactSegments = 4 
       gContact
       ssx = 32#32 #search tree size
       ssy = 32#32 #search tree size
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
                                         color= graphics.color.lightgrey, nTiles=64),
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
           
           #add drive with prescribed velocity:
           def UFvelocityDrive(mbs, t, itemNumber, lOffset): #time derivative of UFoffset
               v = 10*t
               vMax = 5
               return max(v, vMax)
           
           def UFmotionDrive(mbs, t, itemNumber, lOffset): 
               [u,v,a] = MotionInterpolator(t, robotTrajectory=RT, joint=0)
               return u
   
   
           velocityControl = False
           if i == 1:
               mCoordinateWheel = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=2))
               if velocityControl:
                   mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCoordinateWheel],
                                                       velocityLevel=True, offsetUserFunction_t=UFvelocityDrive))
               else: #position control
                   mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCoordinateWheel],
                                                       offsetUserFunction=UFmotionDrive))
                   
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
   simulationSettings.parallel.numberOfThreads = 4 #use 4 to speed up for > 100 ANCF elements
   simulationSettings.displayStatistics = True
   
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.stepInformation= 3+4+32#+128+256
   #simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-4
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
   #simulationSettings.timeIntegration.discontinuous.iterationTolerance = 10
   simulationSettings.timeIntegration.newton.modifiedNewtonJacUpdatePerStep=True
   simulationSettings.timeIntegration.newton.maxIterations=16
   simulationSettings.timeIntegration.verboseMode = 1
   # simulationSettings.timeIntegration.adaptiveStepRecoveryIterations = 9
   simulationSettings.timeIntegration.adaptiveStepRecoverySteps = 40
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
   simulationSettings.displayStatistics = True
   
   SC.visualizationSettings.general.circleTiling = 24
   SC.visualizationSettings.loads.show=False
   SC.visualizationSettings.nodes.defaultSize = 0.01
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.openGL.lineWidth = 2
   
   # SC.visualizationSettings.general.useGradientBackground = True
   simulationSettings.solutionSettings.solutionInformation = 'elevator'
   # SC.visualizationSettings.general.textSize = 14
   
   SC.visualizationSettings.window.renderWindowSize = [1024,2000]
   
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
   
   if True:
       doDynamic = True
       if doDynamic :
           mbs.SolveDynamic(simulationSettings) #183 Newton iterations, 0.114 seconds
       else:
           mbs.SolveStatic(simulationSettings) #183 Newton iterations, 0.114 seconds
   
   
   if useGraphics and True:
       SC.visualizationSettings.general.autoFitScene = False
       SC.visualizationSettings.general.graphicsUpdateInterval=0.02
       # from exudyn.interactive import SolutionViewer
       # sol = LoadSolutionFile('solution/coordinatesSolution.txt', safeMode=True)#, maxRows=100)
       # SolutionViewer(mbs, sol)
       mbs.SolutionViewer()
   
   
   if useGraphics: 
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
       
       # if True:
       #     from exudyn.plot import PlotSensor
       #     PlotSensor(mbs, sensorNumbers=[sAngVel[0],sAngVel[1]], components=2, closeAll=True)
       #     PlotSensor(mbs, sensorNumbers=sMeasureRoll, components=1)
   
   
   
   
   
      
       
   
   
   
   
   


