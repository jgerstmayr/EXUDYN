
.. _testmodels-rollingdisctangentialforces:

******************************
rollingDiscTangentialForces.py
******************************

You can view and download this file on Github: `rollingDiscTangentialForces.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/rollingDiscTangentialForces.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for tangential forces of rolling disc
   #
   # Author:   Johannes Gerstmayr, Michael Pieber
   # Date:     2024-04-29
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
   
   
   #Dimensions
   mass = 30  #kg
   r = 1.8 #m
   d = 3   #m
   kAB=1.2 #inertia radius
   
   JradAB = mass*kAB**2 
   #wheel
   w=0.1 #for drawing
   
   
   
   
   #####################################
   Fn=400 #N
   g=9.81
   
   omegaY = np.sqrt(r*(Fn-mass*g)/JradAB)
   omegaZ = d/r*omegaY #rad/s # = angular velocity omega_y
   
   exu.Print('omegaY=',omegaY)
   exu.Print('omegaZ=',omegaZ)
   
   
   theta = np.array([[mass*r**2,0,0], [0,mass*r**2,0], [0,0,JradAB]])
   wV= np.array([0,omegaY*0,-omegaZ])
   
   
   wL=np.array([0,omegaY,0])
   M=Skew(wL)@theta@wV
   
   exu.Print('necessary torque=',d*(Fn-mass*g), ', gyro torque=', -M[0])
   exu.Print(d*(mass*g-Fn))
   #centrifugal force in simulation: -396.37
   exu.Print('centrifugal force:',d * omegaY**2 * mass)
   
   #%%
   #####################################
   #####################################
   omegaZ=omegaY*1
   omegaX=-d/r*omegaZ #rad/s angular velocity of the wheel
   g = [0,0,-9.81] #gravity
   bodyDim = [d,0.1,0.1] #body dimensions
   
   
   v0 = omegaZ * d
   v0Vec = np.array([0,v0,0])
   p0 = [0,0,0] #origin of pendulum
   # p1 = [bodyDim[0],0,0] #origin of wheel
   
   pMid0 = [d*0.5,0,r] #center of mass, body0
   pMid1 = [d,0,r] #center of mass, body1
   
   theta = np.array([[JradAB,0,0], [0,mass*r**2,0], [0,0,mass*r**2]])
   iWheel = RigidBodyInertia(mass,theta,p0)
   # exu.Print(iWheel)
   iCube = InertiaCuboid(1, sideLengths=[0.1,0.1,0.1])
   # exu.Print(iCube)
   
   
   w1=np.array([-omegaZ,0,omegaX])     #wheel + frame
   w2=np.array([0,0,omegaX])           #frame
   
   M=Skew(w2)@theta@w1                 #possible for symmetric rotor, but dangerous!
   
   Mgyro = w2[2]*theta[0,0]*w1[0]      #rotor solution; condition: rotor is symmetric
   
   exu.Print('omega1 x (Theta * omega2) = ',M[1])
   exu.Print('Mgyro                     = ',Mgyro)
   exu.Print('omega1 x (Theta * omega1) = ',Skew(w1)@theta@w1)
   exu.Print(d*(-Fn+mass*9.81))
   
   
   ###############################################################################
   #
   
   
   #initial acceleration:
   angAcc = np.array([0., -14.28472222, 0. ])
   exu.Print('Theta * angAcc = ',theta@angAcc)
   exu.Print('Theta * angAcc + omega1 x (Theta * omega1) = ',theta@angAcc + Skew(w1)@theta@w1)
   
   planeNormal = RotationMatrixX(0.*pi)@np.array([0,0,1])
   vOff = -r*planeNormal+[0,0,r]
   
   #graphics for body
   graphicsBody = graphics.RigidLink(p0=[-0.5*bodyDim[0],0,0], p1=[0.5*bodyDim[0],0,0], 
                                        axis1=[0,0,1], radius=[0.01,0.01], 
                                        thickness = 0.2, width = [0.2,0.2], color=graphics.color.lightred)
   
   #%%
   dict0 = mbs.CreateRigidBody(name='',   
                               referencePosition=pMid0,  
                               referenceRotationMatrix=np.diag([1,1,1]),  
                               initialVelocity=list(0.5 * v0Vec),  
                               initialAngularVelocity=[0,0,omegaZ],  
                               inertia=iCube,  
                               nodeType=exu.NodeType.RotationEulerParameters,  
                               graphicsDataList=[graphicsBody],  
                               returnDict=True)  
   [n0, b0] = [dict0['nodeNumber'], dict0['bodyNumber']]
   
   graphicsBodyWheel = graphics.Brick(centerPoint=[0,0,0],size=[w*2,1.4*r,1.4*r], color=graphics.color.lightred)
   dict1 = mbs.CreateRigidBody(name='',   
                               referencePosition=pMid1,  
                               referenceRotationMatrix=np.diag([1,1,1]),  
                               initialVelocity=list(v0Vec),  
                               initialAngularVelocity=[omegaX,0,omegaZ],  
                               inertia=iWheel,  
                               gravity=g,  
                               nodeType=exu.NodeType.RotationEulerParameters,  
                               graphicsDataList=[graphicsBodyWheel],  
                               returnDict=True)  
   [n1, bWheel] = [dict1['nodeNumber'], dict1['bodyNumber']]
   
   #ground body and marker
   #graphicsPlane = graphics.Brick(centerPoint=[0,0,-0.1],size=[3*d,3*d,0.2], color=graphics.color.grey)
   graphicsPlane = graphics.CheckerBoard(point=vOff, normal=planeNormal, size=3*d)
   
   oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[graphicsPlane])))
   markerSupportGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,r]))
   
   #markers for rigid body:
   markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[-0.5*bodyDim[0],0,0]))
   markerBody0J1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0.5*bodyDim[0],0,0]))
   
   markerBodyWheel = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bWheel, localPosition=[0,0,0]))
   
   mbs.AddObject(SphericalJoint(markerNumbers=[markerSupportGround,markerBody0J0],
                                 constrainedAxes=[1,1,1],
                                 visualization=VObjectJointSpherical(jointRadius=0.025)))
   
   mbs.AddObject(GenericJoint(markerNumbers=[markerBody0J1, markerBodyWheel], 
                               constrainedAxes=[0*1,1,1,0,1,1],
                               visualization=VObjectJointGeneric(axesRadius=0.01, axesLength=0.1)))
   
   # rolling disc joint:
   markerRollingPlane = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=vOff))
   #oRolling=mbs.AddObject(ObjectJointRollingDisc(markerNumbers=[markerRollingPlane,markerBody0J1], 
   
   if True:
       oRolling=mbs.AddObject(ObjectJointRollingDisc(markerNumbers=[markerRollingPlane,markerBodyWheel], 
                                                     constrainedAxes=[1,1,1], #note that tangential constraints lead to additional forces on ground ==> changes force on ground!
                                                     discRadius=r, 
                                                     #planeNormal = planeNormal,
                                                     visualization=VObjectJointRollingDisc(discWidth=w,color=graphics.color.blue)))
   else:
       nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
       oRolling=mbs.AddObject(ObjectConnectorRollingDiscPenalty(markerNumbers=[markerRollingPlane,markerBodyWheel],
                                                                nodeNumber=nGeneric,
                                                                discRadius=r, discAxis=[1,0,0],
                                                                planeNormal = planeNormal,
                                                                contactStiffness=1e5, contactDamping=1e3, 
                                                                dryFriction=[1,1],
                                                                visualization=VObjectConnectorRollingDiscPenalty(discWidth=w,color=graphics.color.blue)))
   
   sForce = mbs.AddSensor(SensorObject(objectNumber=oRolling, storeInternal=True,
                                       outputVariableType = exu.OutputVariableType.ForceLocal))
   
   sTrailVel = mbs.AddSensor(SensorObject(objectNumber=oRolling, storeInternal=True,
                                      outputVariableType = exu.OutputVariableType.Velocity))
   
   sAngVel = mbs.AddSensor(SensorBody(bodyNumber=bWheel, storeInternal=True,
                                      outputVariableType = exu.OutputVariableType.AngularVelocity))
   sAngVelLocal = mbs.AddSensor(SensorBody(bodyNumber=bWheel, storeInternal=True,
                                      outputVariableType = exu.OutputVariableType.AngularVelocityLocal))
   sAngAcc = mbs.AddSensor(SensorBody(bodyNumber=bWheel, storeInternal=True,
                                      outputVariableType = exu.OutputVariableType.AngularAcceleration))
   
   mbs.Assemble()
   
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   useGraphics=False
   tEnd = 0.1
   if useGraphics:
       tEnd = 2
   
   h = 0.001
   simulationSettings.timeIntegration.endTime = tEnd #0.2 for testing
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   #simulationSettings.solutionSettings.solutionWritePeriod = 0.01
   #simulationSettings.solutionSettings.sensorsWritePeriod = 0.01
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
   
   simulationSettings.timeIntegration.simulateInRealtime = True
   
   SC.visualizationSettings.connectors.showJointAxes = True
   SC.visualizationSettings.connectors.jointAxesLength = 0.3
   SC.visualizationSettings.connectors.jointAxesRadius = 0.08
   SC.visualizationSettings.openGL.lineWidth=2 #maximum
   SC.visualizationSettings.openGL.shadow=0.2
   SC.visualizationSettings.openGL.multiSampling = 4
   
   if useGraphics:
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys['renderState'])
       SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   force = mbs.GetSensorValues(sForce)
   u = 1e-3*(abs(force[0]) + abs(force[1]) + abs(force[2]))
   exu.Print('rollingDiscTangentialForces: F=',force) #use x-coordinate
   exu.Print('solution of rollingDiscTangentialForces=',u) #use x-coordinate
   
   exudynTestGlobals.testError = u - (1.0342017388721547) 
   exudynTestGlobals.testResult = u
   
   
   if True:
       from exudyn.plot import PlotSensor
       PlotSensor(mbs,closeAll=True) 
       PlotSensor(mbs, sensorNumbers=[sForce], components=[0,1,2]) 
       # PlotSensor(mbs, sensorNumbers=sAngVel, components=[0,1,2]) 
       # PlotSensor(mbs, sensorNumbers=sAngVelLocal, components=[0,1,2]) 
       # PlotSensor(mbs, sensorNumbers=sAngAcc, components=[0,1,2]) 
   
       # PlotSensor(mbs, sensorNumbers=sTrailVel, components=[0,1]) 


