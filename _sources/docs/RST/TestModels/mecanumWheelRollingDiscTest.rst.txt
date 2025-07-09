
.. _testmodels-mecanumwheelrollingdisctest:

******************************
mecanumWheelRollingDiscTest.py
******************************

You can view and download this file on Github: `mecanumWheelRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/mecanumWheelRollingDiscTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  mecanum wheels modeled by ObjectConnectorRollingDiscPenalty
   #           specific friction angle of rolling disc is used to model rolls of mecanum wheels
   #           formulation is still under development and needs more testing
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-06-19
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
   
   g = [0,0,-9.81]     #gravity in m/s^2
   
   doBreaking = False
   
   #++++++++++++++++++++++++++++++
   #wheel parameters:
   rhoWheel = 500      #density kg/m^3
   rWheel = 0.4            #radius of disc in m
   wWheel = 0.2             #width of disc in m, just for drawing
   p0Wheel = [0,0,rWheel]        #origin of disc center point at reference, such that initial contact point is at [0,0,0]
   initialRotationCar = RotationMatrixZ(0)
   
   v0 = -5*0 #initial car velocity in y-direction
   omega0Wheel = [v0/rWheel,0,0]                   #initial angular velocity around z-axis
   
   #v0 = [0,0,0]                                   #initial translational velocity
   #exu.Print("v0Car=",v0)
   
   #++++++++++++++++++++++++++++++
   #car parameters:
   p0Car = [0,0,rWheel]    #origin of disc center point at reference, such that initial contact point is at [0,0,0]
   lCar = 3                #y-direction
   wCar = 3                #x-direction
   hCar = rWheel           #z-direction
   mCar = 500
   omega0Car = [0,0,0]                   #initial angular velocity around z-axis
   v0Car = [0,-v0,0]                  #initial velocity of car center point
   
   #inertia for infinitely small ring:
   inertiaWheel = InertiaCylinder(density=rhoWheel, length=wWheel, outerRadius=rWheel, axis=0)
   #exu.Print(inertiaWheel)
   
   inertiaCar = InertiaCuboid(density=mCar/(lCar*wCar*hCar),sideLengths=[wCar, lCar, hCar])
   #exu.Print(inertiaCar)
   
   
   graphicsCar = graphics.Brick(centerPoint=[0,0,0],size=[wCar-1.1*wWheel, lCar, hCar], 
                                            color=graphics.color.steelblue)
   #create car body:
   dictCar = mbs.CreateRigidBody(referencePosition=p0Car,  
                                 referenceRotationMatrix=initialRotationCar,  
                                 initialVelocity=v0Car,  
                                 initialAngularVelocity=omega0Car,  
                                 inertia=inertiaCar,  
                                 gravity=g,  
                                 graphicsDataList=[graphicsCar],  
                                 returnDict=True)  
   [nCar, bCar] = [dictCar['nodeNumber'], dictCar['bodyNumber']]
   
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
   gGround = graphics.Brick(centerPoint=[4,4,-0.001],size=[12,12,0.002], color=graphics.color.lightgrey[0:3]+[0.2])
   oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[gGround])))
   markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
   
   if useGraphics:
       sCarVel = mbs.AddSensor(SensorBody(bodyNumber=bCar, storeInternal=True, #fileName='solution/rollingDiscCarVel.txt', 
                                   outputVariableType = exu.OutputVariableType.Velocity))
   
   sPos=[]
   sTrail=[]
   sForce=[]
   
   
   for iWheel in range(nWheels):
       frictionAngle = 0.25*np.pi #45Â°
       if iWheel == 0 or iWheel == 3: #difference in diagonal
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
       dict0 = mbs.CreateRigidBody(referencePosition=VAdd(p0Wheel,pOff),  
                                   referenceRotationMatrix=initialRotation,  
                                   initialVelocity=v0Wheel,  
                                   initialAngularVelocity=omega0Wheel,  
                                   inertia=inertiaWheel,  
                                   gravity=g,  
                                   graphicsDataList=graphicsWheel,  
                                   returnDict=True)  
       [n0, b0] = [dict0['nodeNumber'], dict0['bodyNumber']]
   
       #markers for rigid body:
       mWheel = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0,0,0]))
       markerWheels += [mWheel]
   
       mCarAxle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCar, localPosition=pOff))
       markerCarAxles += [mCarAxle]
   
       lockedAxis0 = 0
       if doBreaking: lockedAxis0 = 1
       #if iWheel==0 or iWheel==1: freeAxis = 1 #lock rotation
       mbs.AddObject(GenericJoint(markerNumbers=[mWheel,mCarAxle],rotationMarker1=initialRotation,
                                  constrainedAxes=[1,1,1,lockedAxis0,1,1])) #revolute joint for wheel
   
       #does not work, because revolute joint does not accept off-axis
       #kSuspension = 1e4
       #dSuspension = kSuspension*0.01
       #mbs.AddObject(CartesianSpringDamper(markerNumbers=[mWheel,mCarAxle],stiffness=[0,0,kSuspension],damping=[0,0,dSuspension]))
   
       nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
       oRolling = mbs.AddObject(ObjectConnectorRollingDiscPenalty(markerNumbers=[markerGround, mWheel], nodeNumber = nGeneric,
                                                     discRadius=rWheel, dryFriction=[1.,0.], dryFrictionAngle=frictionAngle,
                                                     dryFrictionProportionalZone=1e-1, 
                                                     rollingFrictionViscous=0.2*0,
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
   
   
   torqueFactor = 100
   def UFBasicTorque(mbs, t, torque):
       if t < 0.2:
           return torque
       else:
           return [0,0,0]
   
   #takes as input the translational and angular velocity and outputs the velocities for all 4 wheels
   #wheel axis is mounted at x-axis; positive angVel rotates CCW in x/y plane viewed from top
   # car setup:
   # ^Y, lCar
   # | W2 +---+ W3
   # |    |   |
   # |    | + | car center point
   # |    |   |
   # | W0 +---+ W1
   # +---->X, wCar
   #values given for wheel0/3: frictionAngle=-pi/4, wheel 1/2: frictionAngle=pi/4; dryFriction=[1,0] (looks in lateral (x) direction)
   #==>direction of axis of roll on ground of wheel0: [1,-1] and of wheel1: [1,1]
   def MecanumXYphi2WheelVelocities(xVel, yVel, angVel, R, Lx, Ly):
       LxLy2 = (Lx+Ly)/2
       mat = (1/R)*np.array([[ 1,-1, LxLy2],
                             [-1,-1,-LxLy2],
                             [-1,-1, LxLy2],
                             [ 1,-1,-LxLy2]])    
       return mat @ [xVel, yVel, angVel]
   
   #compute velocity trajectory
   def ComputeVelocity(t):
       vel = [0,0,0] #vx, vy, angVel; these are the local velocities!!!
       f=1
       if t < 4:
         vel = [f,0,0]
       elif t < 8:
         vel = [0,f,0]
       elif t < 16:
         vel = [0,0,0.125*np.pi]
       elif t < 20:
         vel = [f,0,0]
       return vel
   
   pControl = 500
   #compute controlled torque; torque[0] contains wheel number
   def UFtorque(mbs, t, torque):
       iWheel = int(torque[0]) #wheel number
   
       v = ComputeVelocity(t) #desired velocity
       vDesired = MecanumXYphi2WheelVelocities(v[0],v[1],v[2],rWheel,wCar,lCar)[iWheel]
       vCurrent = mbs.GetSensorValues(sAngularVelWheels[iWheel])[0] #local x-axis = wheel axis
       
       cTorque = pControl*(vDesired-vCurrent)
       #print("W",iWheel, ": vDes=", vDesired, ", vCur=", vCurrent, ", torque=", cTorque)
       
       return [cTorque,0,0]
   
   if False:
       mbs.AddLoad(Torque(markerNumber=markerWheels[0],loadVector=[ torqueFactor,0,0], bodyFixed = True, loadVectorUserFunction=UFBasicTorque))
       mbs.AddLoad(Torque(markerNumber=markerWheels[1],loadVector=[-torqueFactor,0,0], bodyFixed = True, loadVectorUserFunction=UFBasicTorque))
       mbs.AddLoad(Torque(markerNumber=markerWheels[2],loadVector=[-torqueFactor,0,0], bodyFixed = True, loadVectorUserFunction=UFBasicTorque))
       mbs.AddLoad(Torque(markerNumber=markerWheels[3],loadVector=[ torqueFactor,0,0], bodyFixed = True, loadVectorUserFunction=UFBasicTorque))
   
   if True:
       for i in range(4):
           mbs.AddLoad(Torque(markerNumber=markerWheels[i],loadVector=[ i,0,0], bodyFixed = True, loadVectorUserFunction=UFtorque))
   
   #mbs.AddSensor(SensorObject(objectNumber=oRolling, fileName='solution/rollingDiscTrailVel.txt', 
   #                           outputVariableType = exu.OutputVariableType.VelocityLocal))
   
   
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 0.5
   if useGraphics:
       tEnd = 0.5 #24
   
   h=0.002
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   #simulationSettings.solutionSettings.solutionWritePeriod = 0.01
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.002
   simulationSettings.timeIntegration.verboseMode = 0
   simulationSettings.displayComputationTime = False
   simulationSettings.displayStatistics = False
   
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5#0.5
   simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.discontinuous.ignoreMaxIterations = False #reduce step size for contact switching
   simulationSettings.timeIntegration.discontinuous.iterationTolerance = 0.1
   
   SC.visualizationSettings.nodes.show = True
   SC.visualizationSettings.nodes.drawNodesAsPoint  = False
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.015
   
   #create animation:
   if useGraphics:
       SC.visualizationSettings.window.renderWindowSize=[1920,1080]
       SC.visualizationSettings.openGL.multiSampling = 4
       if False:
           simulationSettings.solutionSettings.recordImagesInterval = 0.05
           SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
   
   if useGraphics:
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   p0=mbs.GetObjectOutputBody(bCar, exu.OutputVariableType.Position, localPosition=[0,0,0])
   exu.Print('solution of mecanumWheelRollingDiscTest=',p0[0]) #use x-coordinate
   
   exudynTestGlobals.testError = p0[0] - (0.2714267238324345) #2020-06-20: 0.2714267238324345
   exudynTestGlobals.testResult = p0[0]
   
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   ##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
   #plot results
   if useGraphics:
       
       
       mbs.PlotSensor(sTrail, componentsX=[0]*4, components=[1]*4, title='wheel trails', closeAll=True,
                  markerStyles=['x ','o ','^ ','D '], markerSizes=12)
       mbs.PlotSensor(sForce, components=[1]*4, title='wheel forces')
       


