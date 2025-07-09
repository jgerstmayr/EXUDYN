
.. _testmodels-carrollingdisctest:

*********************
carRollingDiscTest.py
*********************

You can view and download this file on Github: `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  car with wheels modeled by ObjectConnectorRollingDiscPenalty
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
   wWheel = 0.1             #width of disc in m, just for drawing
   p0Wheel = [0,0,rWheel]        #origin of disc center point at reference, such that initial contact point is at [0,0,0]
   initialRotationCar = RotationMatrixZ(0)
   
   v0 = -5*0 #initial car velocity in y-direction
   omega0Wheel = [v0/rWheel,0,0]                   #initial angular velocity around z-axis
   
   #v0 = [0,0,0]                                   #initial translational velocity
   #exu.Print("v0Car=",v0)
   
   #%%++++++++++++++++++++++++++++++
   #car parameters and inertia:
   p0Car = [0,0,rWheel]        #origin of disc center point at reference, such that initial contact point is at [0,0,0]
   lCar = 3
   wCar = 2
   hCar = rWheel
   mCar = 500
   omega0Car = [0,0,0]                   #initial angular velocity around z-axis
   v0Car = [0,-v0,0]                  #initial velocity of car center point
   
   #inertia for infinitely small ring:
   inertiaWheel = InertiaCylinder(density=rhoWheel, length=wWheel, outerRadius=rWheel, axis=0)
   #exu.Print(inertiaWheel)
   
   inertiaCar = InertiaCuboid(density=mCar/(lCar*wCar*hCar),sideLengths=[wCar, lCar, hCar])
   #exu.Print(inertiaCar)
   
   #%%++++++++++++++++++++++++++++++
   #create car node and body:
   graphicsCar = graphics.Brick(centerPoint=[0,0,0],size=[wCar-1.1*wWheel, lCar, hCar], color=graphics.color.lightred)
   bCar=mbs.CreateRigidBody(inertia = inertiaCar, 
                            referencePosition = p0Car, 
                            referenceRotationMatrix = initialRotationCar,
                            initialAngularVelocity = omega0Car,
                            initialVelocity = v0Car,
                            gravity = g, 
                            graphicsDataList = [graphicsCar])
   
   nCar = mbs.GetObject(bCar)['nodeNumber']
   nWheels = 4
   markerWheels=[]
   markerCarAxles=[]
   oRollingDiscs=[]
   
   # car setup:
   # ^Y, lCar
   # | W2 +---+ W3
   # |    |   |
   # |    | + | car center point
   # |    |   |
   # | W0 +---+ W1
   # +---->X, wCar
   
   #ground body and marker
   gGround = graphics.Brick(centerPoint=[0,0,-0.001],size=[30,30,0.002], color=graphics.color.lightgrey)
   oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[gGround])))
   markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
   
   sCarVel = mbs.AddSensor(SensorBody(bodyNumber=bCar, #fileName='solution/rollingDiscCarVel.txt', 
                                      storeInternal=True,
                                      outputVariableType = exu.OutputVariableType.Velocity))
   
   sAngVels=[]
   sWheelPos=[]
   sRollPos=[]
   sRollForce=[]
   
   #%%++++++++++++++++++++++++++++++
   #create wheels bodies and nodes:
   for iWheel in range(nWheels):
       #additional graphics for visualization of rotation:
       graphicsWheel = graphics.Brick(centerPoint=[0,0,0],size=[wWheel*1.1,0.7*rWheel,0.7*rWheel], color=graphics.color.lightred)
   
       dx = -0.5*wCar
       dy = -0.5*lCar
       if iWheel > 1: dy *= -1
       if iWheel == 1 or iWheel == 3: dx *= -1
   
       kRolling = 1e5
       dRolling = kRolling*0.01
   
       rSteering = 5
       phiZwheelLeft = 0
       phiZwheelRight = 0
       if rSteering != 0:
           phiZwheelLeft = np.arctan(lCar/rSteering) #5/180*np.pi   #steering angle
           phiZwheelRight = np.arctan(lCar/(wCar+rSteering)) #5/180*np.pi   #steering angle
   
       initialRotationWheelLeft = RotationMatrixZ(phiZwheelLeft)
       initialRotationWheelRight = RotationMatrixZ(phiZwheelRight)
   
       initialRotation = RotationMatrixZ(0)
       if iWheel == 2:
           initialRotation = initialRotationWheelLeft
       if iWheel == 3:
           initialRotation = initialRotationWheelRight
   
       #v0Wheel = Skew(omega0Wheel) @ initialRotationWheel @ [0,0,rWheel]   #initial angular velocity of center point
       v0Wheel = v0Car #approx.
   
       pOff = [dx,dy,0]
   
   
       #add wheel body
       b0 = mbs.CreateRigidBody(inertia = inertiaWheel, 
                                referencePosition = VAdd(p0Wheel,pOff), 
                                referenceRotationMatrix = initialRotation, #np.diag([1,1,1]),
                                initialAngularVelocity = omega0Wheel,
                                initialVelocity = v0Wheel,
                                gravity = g, 
                                graphicsDataList = [graphicsWheel])
   
       n0 = mbs.GetObject(b0)['nodeNumber']
   
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
   
       nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
       oRolling = mbs.AddObject(ObjectConnectorRollingDiscPenalty(markerNumbers=[markerGround, mWheel], nodeNumber = nGeneric,
                                                     discRadius=rWheel, dryFriction=[0.4,0.4], 
                                                     dryFrictionProportionalZone=1e-1, 
                                                     rollingFrictionViscous=0.2*0,
                                                     contactStiffness=kRolling, contactDamping=dRolling,
                                                     visualization=VObjectConnectorRollingDiscPenalty(discWidth=wWheel, color=graphics.color.blue)))
       oRollingDiscs += [oRolling]
   
       strNum = str(iWheel)
       if useGraphics:
           sAngVels+=[mbs.AddSensor(SensorBody(bodyNumber=b0, #fileName='solution/rollingDiscAngVelLocal'+strNum+'.txt', 
                                    storeInternal=True,
                                    outputVariableType = exu.OutputVariableType.AngularVelocityLocal))]
       
           sWheelPos+=[mbs.AddSensor(SensorBody(bodyNumber=b0, #fileName='solution/rollingDiscPos'+strNum+'.txt', 
                                        storeInternal=True,
                                        outputVariableType = exu.OutputVariableType.Position))]
       
           sRollPos+=[mbs.AddSensor(SensorObject(objectNumber=oRolling, #fileName='solution/rollingDiscTrail'+strNum+'.txt', 
                                               storeInternal=True,
                                               outputVariableType = exu.OutputVariableType.Position))]
       
           sRollForce+=[mbs.AddSensor(SensorObject(name='wheelForce'+strNum,objectNumber=oRolling, #fileName='solution/rollingDiscForce'+strNum+'.txt', 
                                                  storeInternal=True,
                                                  outputVariableType = exu.OutputVariableType.ForceLocal))]
   
   
   #user function for time-dependent torque on two wheels 0,1
   def UFtorque(mbs, t, torque):
       if t < 4:
           return torque
       else:
           return [0,0,0]
   
   mbs.AddLoad(Torque(markerNumber=markerWheels[0],loadVector=[-200,0,0], bodyFixed = True, loadVectorUserFunction=UFtorque))
   mbs.AddLoad(Torque(markerNumber=markerWheels[1],loadVector=[-200,0,0], bodyFixed = True, loadVectorUserFunction=UFtorque))
   
   
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 0.5 #40#1.2
   h=0.002 #no visual differences for step sizes smaller than 0.0005
   
   if useGraphics:
       tEnd = 4
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.verboseMode = 1
   
   
   #simulationSettings.timeIntegration.discontinuous.ignoreMaxIterations = False #reduce step size for contact switching
   #simulationSettings.timeIntegration.discontinuous.iterationTolerance = 0.1
   
   SC.visualizationSettings.nodes.show = True
   SC.visualizationSettings.nodes.drawNodesAsPoint  = False
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.015
   
   mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   c=mbs.GetNodeOutput(n0, variableType=exu.OutputVariableType.Coordinates)
   u=sum(c)
   exu.Print("carRollingDiscTest u=",u)
   
   exudynTestGlobals.testError = u - (-0.23940048717113419) #2020-12-18: -0.23940048717113419
   exudynTestGlobals.testResult = u
   
   ##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
   #plot results
   if useGraphics:
       
       
       mbs.PlotSensor(sensorNumbers=sCarVel, components=[0,1,2], title='car velocitiy', closeAll=True)
       for i in range(4):
           mbs.PlotSensor(sensorNumbers=sRollPos[i], componentsX=0, components=1, 
                      labels='wheel trail '+str(i), newFigure=(i==0), colorCodeOffset=i)
           #trail and wheel pos are almost same, just if car slightly tilts, there is a deviation
           mbs.PlotSensor(sensorNumbers=sWheelPos[i], componentsX=0, components=1, 
                      labels='wheel pos '+str(i), newFigure=False, colorCodeOffset=i+7,
                      lineStyles='', markerStyles='x')
       
       mbs.PlotSensor(sensorNumbers=sRollForce, components=[2]*4, title='wheel contact forces')
   
       mbs.PlotSensor(sensorNumbers=sRollForce*2, components=[0]*4+[1]*4, title='wheel lateral (X) and drive/acceleration (Y) forces')
   
       mbs.PlotSensor(sensorNumbers=sAngVels, components=[0]*4, title='wheel local angular velocity')
   


