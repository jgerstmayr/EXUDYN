
.. _examples-mobilemecanumwheelrobotwithlidar:

***********************************
mobileMecanumWheelRobotWithLidar.py
***********************************

You can view and download this file on Github: `mobileMecanumWheelRobotWithLidar.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/mobileMecanumWheelRobotWithLidar.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Simple vehicle model with Mecanum wheels and 'rotating' laser scanner (Lidar)
   #           Model supports simple trajectories, calculate Odometry and transform 
   #           Lidar data into global frame. 
   #
   # Author:   Peter Manzl
   # Date:     2024-04-23
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.robotics.utilities import AddLidar
   from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration
   
   import numpy as np
   from math import sin, cos, tan
   import matplotlib.pyplot as plt
   
   useGraphics=True
   
   
   #%% adjust the following parameters
   useGroundTruth = True  # read position/orientation data from simulation for transforming Lidar Data into global frame
   flagOdometry = True    # calculate Odometry; if flagReadPosRot
   flagLidarNoise = False # add normally distributed noise to Lidar data with std. deviation below 
   lidarNoiseLevel = np.array([0.05, 0.01]) # std deviation on length and angle of lidar
   
   # normally distributed noise added to angular velocity of wheels to estimate odometry
   # Note that slipping can already occur because of the implemented non-ideal contact between Mecanum wheel and ground
   flagVelNoise = False
   velNoiseLevel = 0.025
   
   
   
   #%% predefined trajectory
   # trajectory is defined asÂ´[x, y, phi_z] in global system
   trajectory = Trajectory(initialCoordinates=[0   ,0   ,0], initialTime=0)
   trajectory.Add(ProfileConstantAcceleration([0   ,-4  ,0], 3))
   trajectory.Add(ProfileConstantAcceleration([0   ,-4  ,2.1*np.pi], 6))
   #trajectory.Add(ProfileConstantAcceleration([0   ,0   ,2*np.pi], 6))
   trajectory.Add(ProfileConstantAcceleration([0   ,0   ,2.1*np.pi], 4))
   trajectory.Add(ProfileConstantAcceleration([10  ,0   ,2.1*np.pi], 4))
   trajectory.Add(ProfileConstantAcceleration([10  ,0   ,0*np.pi], 4))
   # trajectory.Add(ProfileConstantAcceleration([3.6 ,0   ,2.1*np.pi], 0.5)) # wait for 0.5 seconds
   # trajectory.Add(ProfileConstantAcceleration([3.6 ,4.2 ,2.1*np.pi], 6))
   trajectory.Add(ProfileConstantAcceleration([10  ,7 ,0*np.pi], 4))
   trajectory.Add(ProfileConstantAcceleration([10  ,7 ,0.5*np.pi], 4))
   trajectory.Add(ProfileConstantAcceleration([12  ,14 ,0.5*np.pi], 4))
   # trajectory.Add(ProfileConstantAcceleration([2   ,7 ,0*np.pi], 4))
   # trajectory.Add(ProfileConstantAcceleration([3.6 ,4.2 ,0*np.pi], 6))
   
   
   
   #%%
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   g = [0,0,-9.81]     #gravity in m/s^2
   
   #++++++++++++++++++++++++++++++
   #wheel parameters:
   rhoWheel = 500          # density kg/m^3
   rWheel = 0.4            # radius of disc in m
   wWheel = 0.2            # width of disc in m, just for drawing
   p0Wheel = [0, 0, rWheel]        # origin of disc center point at reference, such that initial contact point is at [0,0,0]
   initialRotationCar = RotationMatrixZ(0)
   
   v0 = 0  # initial car velocity in y-direction
   omega0Wheel = [v0/rWheel, 0, 0]                   # initial angular velocity around z-axis
   
   
   # %% ++++++++++++++++++++++++++++++
   # define car (mobile robot) parameters:
   # car setup:
   # ^Y, lCar
   # | W2 +---+ W3
   # |    |   |
   # |    | + | car center point
   # |    |   |
   # | W0 +---+ W1
   # +---->X, wCar
   
   p0Car = [0, 0, rWheel]   # origin of disc center point at reference, such that initial contact point is at [0,0,0]
   lCar = 2                # y-direction
   wCar = 1.5              # x-direction
   hCar = rWheel           # z-direction
   mCar = 500
   omega0Car = [0,0,0]                   #initial angular velocity around z-axis
   v0Car = [0,-v0,0]                  #initial velocity of car center point
   
   #inertia for infinitely small ring:
   inertiaWheel = InertiaCylinder(density=rhoWheel, length=wWheel, outerRadius=rWheel, axis=0)
   
   inertiaCar = InertiaCuboid(density=mCar/(lCar*wCar*hCar),sideLengths=[wCar, lCar, hCar])
   
   rLidar = 0.5*rWheel
   pLidar1 = [(-wCar*0.5-rLidar)*0, 0*(lCar*0.5+rWheel+rLidar), hCar*0.8]
   # pLidar2 = [ wCar*0.5+rLidar,-lCar*0.5-rWheel-rLidar,hCar*0.5]
   graphicsCar = [graphics.Brick(centerPoint=[0,0,0],size=[wCar-1.1*wWheel, lCar+2*rWheel, hCar], 
                                            color=graphics.color.steelblue)]
   
   
   graphicsCar += [graphics.Cylinder(pAxis=pLidar1, vAxis=[0,0,0.5*rLidar], radius=rLidar, clor=graphics.color.darkgrey)]
   graphicsCar += [graphics.Basis(headFactor = 4, length=2)]
   # graphicsCar += [graphics.Cylinder(pAxis=pLidar2, vAxis=[0,0,0.5*rLidar], radius=rLidar, clor=graphics.color.darkgrey)]
   
   dictCar = mbs.CreateRigidBody(
                 inertia=inertiaCar, 
                 referencePosition=p0Car, 
                 referenceRotationMatrix=initialRotationCar,
                 initialAngularVelocity=omega0Car,
                 initialVelocity=v0Car,
                 gravity=g, 
                 graphicsDataList=graphicsCar,
                 returnDict=True)
   [nCar, bCar] = [dictCar['nodeNumber'], dictCar['bodyNumber']]
   
   markerCar = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCar, localPosition=[0,0,hCar*0.5]))
   
   
   markerCar1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCar, localPosition=pLidar1))
   
   nWheels = 4
   markerWheels=[]
   markerCarAxles=[]
   oRollingDiscs=[]
   sAngularVelWheels=[]
   
   # car setup:
   # ^Y, lCar
   # | W2 +---+ W3
   # |    |   |
   # |    | + | car center point
   # |    |   |
   # | W0 +---+ W1
   # +---->X, wCar
   
   #ground body and marker
   LL = 8
   gGround = graphics.CheckerBoard(point=[0.25*LL,0.25*LL,0],size=2*LL)
   
   #obstacles:
   zz=1
   gGround = graphics.MergeTriangleLists(graphics.Brick(centerPoint=[0,7,0.5*zz],size=[2*zz,zz,1*zz], color=graphics.color.dodgerblue), gGround)
   gGround = graphics.MergeTriangleLists(graphics.Brick(centerPoint=[6,5,1.5*zz],size=[zz,2*zz,3*zz], color=graphics.color.dodgerblue), gGround)
   gGround = graphics.MergeTriangleLists(graphics.Brick(centerPoint=[3,-2.5,0.5*zz],size=[2*zz,zz,1*zz], color=graphics.color.dodgerblue), gGround)
   gGround = graphics.MergeTriangleLists(graphics.Cylinder(pAxis=[-3,0,0],vAxis=[0,0,zz], radius=1.5, color=graphics.color.dodgerblue, nTiles=64), gGround)
   
   #walls:
   tt=0.2
   gGround = graphics.MergeTriangleLists(graphics.Brick(centerPoint=[0.25*LL,0.25*LL-LL,0.5*zz],size=[2*LL,tt,zz], color=graphics.color.dodgerblue), gGround)
   gGround = graphics.MergeTriangleLists(graphics.Brick(centerPoint=[0.25*LL,0.25*LL+LL,0.5*zz],size=[2*LL,tt,zz], color=graphics.color.dodgerblue), gGround)
   gGround = graphics.MergeTriangleLists(graphics.Brick(centerPoint=[0.25*LL-LL,0.25*LL,0.5*zz],size=[tt,2*LL,zz], color=graphics.color.dodgerblue), gGround)
   gGround = graphics.MergeTriangleLists(graphics.Brick(centerPoint=[0.25*LL+LL,0.25*LL,0.5*zz],size=[tt,2*LL,zz], color=graphics.color.dodgerblue), gGround)
   
   
   oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[gGround])))
   mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #set up general contact geometry where sensors measure
   # helper function to create 2D rotation Matrix
   def Rot2D(phi): 
       return np.array([[np.cos(phi),-np.sin(phi)],
                        [np.sin(phi), np.cos(phi)]])
   
   [meshPoints, meshTrigs] = graphics.ToPointsAndTrigs(gGround)
   
   ngc = mbs.CreateDistanceSensorGeometry(meshPoints, meshTrigs, rigidBodyMarkerIndex=mGround, searchTreeCellSize=[8,8,1])
   maxDistance = 20 #max. distance of sensors; just large enough to reach everything; take care, in zoom all it will show this large area
   
   # dict mbs.variables can be accessed globally in the "control" functions
   mbs.variables['Lidar'] = [pi*0.25, pi*0.75, 50]
   mbs.variables['LidarAngles'] = np.linspace(mbs.variables['Lidar'][0], mbs.variables['Lidar'][1], mbs.variables['Lidar'] [2])
   mbs.variables['R'] = []
   for phi in mbs.variables['LidarAngles']: 
       mbs.variables['R']  += [Rot2D(phi)] # zero-angle of Lidar is at x-axis
   
   
   mbs.variables['sLidarList'] = AddLidar(mbs, generalContactIndex=ngc, positionOrMarker=markerCar1, minDistance=0, maxDistance=maxDistance, 
             numberOfSensors=mbs.variables['Lidar'][2], addGraphicsObject=True,
             angleStart=mbs.variables['Lidar'][0], 
             angleEnd=mbs.variables['Lidar'][1], # 1.5*pi-pi,
             lineLength=1, storeInternal=True, color=graphics.color.red, inclination=0., rotation=RotationMatrixZ(np.pi/2*0))
   
   if False: # here additional Sensors could be created to have e.g. two markers diagonally on the car (robot)
       AddLidar(mbs, generalContactIndex=ngc, positionOrMarker=markerCar2, minDistance=0, maxDistance=maxDistance, 
                 numberOfSensors=100,angleStart=0, angleEnd=1.5*pi, inclination=-4/180*pi,
                 lineLength=1, storeInternal=True, color=graphics.color.grey )
       
   
       
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if useGraphics:
       sCarVel = mbs.AddSensor(SensorBody(bodyNumber=bCar, storeInternal=True, #fileName='solution/rollingDiscCarVel.txt', 
                                   outputVariableType = exu.OutputVariableType.Velocity))
   
   mbs.variables['sRot'] = mbs.AddSensor(SensorBody(bodyNumber=bCar, storeInternal=True, outputVariableType=exu.OutputVariableType.RotationMatrix))
   mbs.variables['sPos'] = mbs.AddSensor(SensorBody(bodyNumber=bCar, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
   sPos = []
   sTrail=[]
   sForce=[]
   
   # create Mecanum wheels and ground contact
   for iWheel in range(nWheels):
       frictionAngle = 0.25*np.pi # 45Â°
       if iWheel == 0 or iWheel == 3: # difference in diagonal
           frictionAngle *= -1
   
       #additional graphics for visualization of rotation (JUST FOR DRAWING!):
       graphicsWheel = [graphics.Brick(centerPoint=[0,0,0],size=[wWheel*1.1,0.7*rWheel,0.7*rWheel], color=graphics.color.lightred)]
       nCyl = 12
       rCyl = 0.1*rWheel
       for i in range(nCyl): #draw cylinders on wheels
           iPhi = i/nCyl*2*np.pi
           pAxis = np.array([0,rWheel*np.sin(iPhi),-rWheel*np.cos(iPhi)])
           vAxis = [0.5*wWheel*np.cos(frictionAngle),0.5*wWheel*np.sin(frictionAngle),0]
           vAxis2 = RotationMatrixX(iPhi)@vAxis
           rColor = graphics.color.grey
           if i >= nCyl/2: rColor = graphics.color.darkgrey
           graphicsWheel += [graphics.Cylinder(pAxis=pAxis-vAxis2, vAxis=2*vAxis2, radius=rCyl, 
                                                  color=rColor)]
           graphicsWheel+= [graphics.Basis()]
   
       dx = -0.5*wCar
       dy = -0.5*lCar
       if iWheel > 1: dy *= -1
       if iWheel == 1 or iWheel == 3: dx *= -1
   
       kRolling = 1e5
       dRolling = kRolling*0.01
   
       initialRotation = RotationMatrixZ(0)
   
       #v0Wheel = Skew(omega0Wheel) @ initialRotationWheel @ [0,0,rWheel]   #initial angular velocity of center point
       v0Wheel = v0Car #approx.
   
       pOff = [dx,dy,0]
   
       #add wheel body
   
       dictWheel = mbs.CreateRigidBody(
                     inertia=inertiaWheel, 
                     referencePosition=VAdd(p0Wheel, pOff), 
                     referenceRotationMatrix=initialRotation,
                     initialAngularVelocity=omega0Wheel,
                     initialVelocity=v0Wheel,
                     gravity=g, 
                     graphicsDataList=graphicsWheel,
                     returnDict=True)
       [n0, b0] = [dictWheel['nodeNumber'], dictWheel['bodyNumber']]
   
       #markers for rigid body:
       mWheel = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0,0,0]))
       markerWheels += [mWheel]
   
       mCarAxle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCar, localPosition=pOff))
       markerCarAxles += [mCarAxle]
   
       lockedAxis0 = 0 # could be used to lock an Axis
       #if iWheel==0 or iWheel==1: freeAxis = 1 #lock rotation
       mbs.AddObject(GenericJoint(markerNumbers=[mWheel,mCarAxle],rotationMarker1=initialRotation,
                                  constrainedAxes=[1,1,1,lockedAxis0,1,1])) #revolute joint for wheel
   
       nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
       oRolling = mbs.AddObject(ObjectConnectorRollingDiscPenalty(markerNumbers=[mGround, mWheel], nodeNumber = nGeneric,
                                                     discRadius=rWheel, dryFriction=[1.,0.001], dryFrictionAngle=frictionAngle,
                                                     dryFrictionProportionalZone=1e-1, 
                                                     rollingFrictionViscous=0.01,
                                                     contactStiffness=kRolling, contactDamping=dRolling,
                                                     visualization=VObjectConnectorRollingDiscPenalty(discWidth=wWheel, color=graphics.color.blue)))
       oRollingDiscs += [oRolling]
   
       strNum = str(iWheel)
       sAngularVelWheels += [mbs.AddSensor(SensorBody(bodyNumber=b0, storeInternal=True,#fileName='solution/rollingDiscAngVelLocal'+strNum+'.txt', 
                                  outputVariableType = exu.OutputVariableType.AngularVelocityLocal))]
   
       if useGraphics:
           sPos+=[mbs.AddSensor(SensorBody(bodyNumber=b0, storeInternal=True,#fileName='solution/rollingDiscPos'+strNum+'.txt', 
                                      outputVariableType = exu.OutputVariableType.Position))]
       
           sTrail+=[mbs.AddSensor(SensorObject(name='Trail'+strNum,objectNumber=oRolling, storeInternal=True,#fileName='solution/rollingDiscTrail'+strNum+'.txt', 
                                      outputVariableType = exu.OutputVariableType.Position))]
       
           sForce+=[mbs.AddSensor(SensorObject(objectNumber=oRolling, storeInternal=True,#fileName='solution/rollingDiscForce'+strNum+'.txt', 
                                      outputVariableType = exu.OutputVariableType.ForceLocal))]
   
   
   
   # takes as input the translational and angular velocity and outputs the velocities for all 4 wheels
   # wheel axis is mounted at x-axis; positive angVel rotates CCW in x/y plane viewed from top
   # car setup:
   # ^Y, lCar
   # | W2 +---+ W3
   # |    |   |
   # |    | + | car center point
   # |    |   |
   # | W0 +---+ W1
   # +---->X, wCar
   # values given for wheel0/3: frictionAngle=-pi/4, wheel 1/2: frictionAngle=pi/4; dryFriction=[1,0] (looks in lateral (x) direction)
   # ==>direction of axis of roll on ground of wheel0: [1,-1] and of wheel1: [1,1]
   def MecanumXYphi2WheelVelocities(xVel, yVel, angVel, R, Lx, Ly):
       LxLy2 = (Lx+Ly)/2
       mat = (1/R)*np.array([[ 1,-1, LxLy2],
                             [-1,-1,-LxLy2],
                             [-1,-1, LxLy2],
                             [ 1,-1,-LxLy2]])    
       return mat @ [xVel, yVel, angVel]
   
   def WheelVelocities2MecanumXYphi(w, R, Lx, Ly):
       LxLy2 = (Lx+Ly)/2
       mat = (1/R)*np.array([[ 1,-1, LxLy2],
                             [-1,-1,-LxLy2],
                             [-1,-1, LxLy2],
                             [ 1,-1,-LxLy2]])    
       return np.linalg.pinv(mat) @ w
   
   
   pControl = 100 # P-control on wheel velocity
   mbs.variables['wheelMotor'] = []
   mbs.variables['loadWheel'] = []
   for i in range(4):
       # Torsional springdamper always acts in z-Axis
       RM1 = RotationMatrixY(np.pi/2) 
       RM0 = RotationMatrixY(np.pi/2)
       nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates = 1, initialCoordinates=[0])) # records multiples of 2*pi
       mbs.variables['wheelMotor'] += [mbs.AddObject(TorsionalSpringDamper(name='Wheel{}Motor'.format(i), 
                                               # mobileRobotBackDic['mAxlesList'][i]
                                               markerNumbers=[markerCarAxles[i], markerWheels[i]],
                                               nodeNumber= nData, # for continuous Rotation
                                               stiffness = 0, damping = pControl, 
                                               rotationMarker0=RM0, 
                                               rotationMarker1=RM1))]
   #%% 
   # function to read data from Lidar sensors into array of global [x,y] values. 
   def GetCurrentData(mbs, Rot, pos): 
       data = np.zeros([mbs.variables['nLidar'] , 2])
       if not(flagLidarNoise): 
           for i, sensor in enumerate(mbs.variables['sLidarList']): 
               if useGroundTruth: 
                   R_ = np.eye(3)
                   R_[0:2, 0:2] = mbs.variables['R'][i]
                   data[i,:] =  (pos + Rot @ R_ @ [mbs.GetSensorValues(sensor), 0, 0])[0:2] #  GetSensorValues contains X-value
               else: 
                   data[i,:] =  pos[0:2] + Rot[0:2,0:2] @ mbs.variables['R'][i] @ [mbs.GetSensorValues(sensor), 0]
       else: 
           noise_distance = np.random.normal(0, lidarNoiseLevel[0], mbs.variables['nLidar'])
           noise_angle = np.random.normal(0, lidarNoiseLevel[1], mbs.variables['nLidar'])
           for i, sensor in enumerate(mbs.variables['sLidarList']): 
               data[i,:] =  pos[0:2] + Rot2D(noise_angle[i]) @ Rot[0:2,0:2] @ mbs.variables['R'][i] @ (mbs.GetSensorValues(sensor) + [noise_distance[i],0]).tolist() #  + [0.32]
               
       return data
   
   
   #%% PreStepUF is called before every step. There odometry is calculated, velocity
   def PreStepUF(mbs, t):
       # using Prestep instead of UFLoad reduced simulation time fopr 24 seconds from 6.11887 to 4.02554 seconds (~ 34%)
       u, v, a = trajectory.Evaluate(t) # 
       wDesired = MecanumXYphi2WheelVelocities(v[0],v[1],v[2],rWheel,wCar,lCar)
       dt = mbs.sys['dynamicSolver'].it.currentStepSize # for integration of values
       
       # wheel control
       for iWheel in range(4):
           wCurrent = mbs.GetSensorValues(sAngularVelWheels[iWheel])[0] #local x-axis = wheel axis
           mbs.variables['wWheels'][iWheel] = wCurrent # save current wheel velocity
           mbs.SetObjectParameter(mbs.variables['wheelMotor'][iWheel], 'velocityOffset', wDesired[iWheel]) # set wheel velocity for control
           
       # calculate odometry
       if flagOdometry: 
           # odometry: vOdom = pinv(J) @ wWheels
           # obtain position from vOdom by integration
           if flagVelNoise: 
               vOdom = WheelVelocities2MecanumXYphi(mbs.variables['wWheels'] + np.random.normal(0, velNoiseLevel, 4), 
                                                    rWheel, wCar, lCar)
           else: 
               vOdom = WheelVelocities2MecanumXYphi(mbs.variables['wWheels'], rWheel, wCar, lCar)
           mbs.variables['rotOdom'] += vOdom[-1] * dt  # (t - mbs.variables['tLast'])
           mbs.variables['posOdom'] += Rot2D(mbs.variables['rotOdom']) @ vOdom[0:2] * dt
           # print('pos: ', mbs.variables['posOdom'])
           
       if (t - mbs.variables['tLast']) > mbs.variables['dtLidar']: 
           mbs.variables['tLast'] += mbs.variables['dtLidar']
           
           if useGroundTruth: 
               # position and rotation taken from the gloabl data --> accurate! 
               Rot = mbs.GetSensorValues(mbs.variables['sRot']).reshape([3,3])
               pos = mbs.GetSensorValues(mbs.variables['sPos'])
           elif flagOdometry: 
               Rot = Rot2D(mbs.variables['rotOdom'])
               pos = mbs.variables['posOdom']
           data = GetCurrentData(mbs, Rot, pos)
           k = int(t/mbs.variables['dtLidar'])
           # print('data {} at t: {}'.format(k, round(t, 2))) 
           mbs.variables['lidarDataHistory'][k,:,:] = data
           mbs.variables['posHistory'][k] = pos[0:2]
           mbs.variables['RotHistory'][k] = Rot[0:2,0:2]
   
           
       return True
   
   # allocate dictionary values for Lidar measurements
   h=0.005
   tEnd = trajectory.GetTimes()[-1] + 2 + h # add +h to call preStepFunction at tEnd
   mbs.variables['wWheels'] = np.zeros([4])
   mbs.variables['posOdom'], mbs.variables['rotOdom'], mbs.variables['tLast'] = np.array([0,0], dtype=np.float64), 0, 0
   mbs.variables['phiWheels'] = np.zeros(4)
   mbs.variables['tLast'] = 0
   mbs.variables['dtLidar'] = 0.1 #50e-3
   mbs.variables['nLidar'] = len(mbs.variables['sLidarList'])
   nMeasure = int(tEnd/mbs.variables['dtLidar']) + 1
   mbs.variables['lidarDataHistory'] = np.zeros([nMeasure, mbs.variables['nLidar'], 2])
   mbs.variables['RotHistory'] = np.zeros([nMeasure, 2,2])
   mbs.variables['RotHistory'][0] = np.eye(2)
   mbs.variables['posHistory'] = np.zeros([nMeasure, 2])
   
   mbs.SetPreStepUserFunction(PreStepUF)
   mbs.Assemble() # Assemble creats system equations and enables reading data for timestep 0
   data0 = GetCurrentData(mbs, mbs.GetSensorValues(mbs.variables['sRot']).reshape([3,3]), mbs.GetSensorValues(mbs.variables['sPos']))
   mbs.variables['lidarDataHistory'][0] = data0
   
   
   #%%create simulation settings
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.1
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.displayComputationTime = False
   simulationSettings.displayStatistics = False
   
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 # 0.5
   simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.discontinuous.ignoreMaxIterations = False #reduce step size for contact switching
   simulationSettings.timeIntegration.discontinuous.iterationTolerance = 0.1
   simulationSettings.linearSolverType=exu.LinearSolverType.EigenSparse
   
   speedup=True
   if speedup:
       simulationSettings.timeIntegration.discontinuous.ignoreMaxIterations = False #reduce step size for contact switching
       simulationSettings.timeIntegration.discontinuous.iterationTolerance = 0.1
       
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.01
   SC.visualizationSettings.nodes.show = True
   SC.visualizationSettings.nodes.drawNodesAsPoint  = False
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.015
   
   SC.visualizationSettings.openGL.lineWidth = 2
   SC.visualizationSettings.openGL.shadow = 0.3
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.openGL.perspective = 0.7
   
   #create animation:
   if useGraphics:
       SC.visualizationSettings.window.renderWindowSize=[1920,1080]
       SC.visualizationSettings.openGL.multiSampling = 4
   
       if False: #save images
           simulationSettings.solutionSettings.sensorsWritePeriod = 0.01 #to avoid laggy visualization
           simulationSettings.solutionSettings.recordImagesInterval = 0.04
           SC.visualizationSettings.exportImages.saveImageFileName = "images/frame"
   
   if useGraphics:
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   
   #%% 
   p0=mbs.GetObjectOutputBody(bCar, exu.OutputVariableType.Position, localPosition=[0,0,0])
   
   if useGraphics: #
       plt.close('all')
       plt.figure()
       from matplotlib import colors as mcolors
       myColors = dict(mcolors.BASE_COLORS, **mcolors.CSS4_COLORS)
       col1 = mcolors.to_rgb(myColors['red'])
       col2 = mcolors.to_rgb(myColors['green'])    
       for i in range(0, mbs.variables['lidarDataHistory'].shape[0]):   
           col_i = np.array(col1)* (1 - i/(nMeasure-1)) + np.array(col2)* (i/(nMeasure-1))
           plt.plot(mbs.variables['lidarDataHistory'][i,:,0], mbs.variables['lidarDataHistory'][i,:,1], 
                                'x', label='lidar m' + str(i), color=col_i.tolist())
           e1 = mbs.variables['RotHistory'][i][:,1]
           p = mbs.variables['posHistory'][i]
           plt.plot(p[0], p[1], 'o', color=col_i)
           plt.arrow(p[0], p[1], e1[0], e1[1], color=col_i, head_width=0.2)
           
       plt.title('lidar data: using ' + 'accurate data' * bool(useGroundTruth) + 'inaccurate Odometry' * bool(not(useGroundTruth) and flagOdometry))
       plt.grid()
       plt.axis('equal')
       plt.xlabel('x in m')
       plt.ylabel('y in m')
   
   ##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
   #plot results
   # if useGraphics and False:
   #     mbs.PlotSensor(sTrail, componentsX=[0]*4, components=[1]*4, title='wheel trails', closeAll=True,
   #                markerStyles=['x ','o ','^ ','D '], markerSizes=12)
   #     mbs.PlotSensor(sForce, components=[1]*4, title='wheel forces')
       


