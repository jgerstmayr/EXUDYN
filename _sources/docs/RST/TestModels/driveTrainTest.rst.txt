
.. _testmodels-drivetraintest:

*****************
driveTrainTest.py
*****************

You can view and download this file on Github: `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/driveTrainTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test of Node1D, Mass1D and Rotor1D for drivetrains;4-piston compressor
   #           Uses different constraints to realize the drivetrain; 
   #           includes joints to connect 3D bodies and 1D drivetrain;
   #           the crank is elastically supported (except z-direction); 
   #           this makes a direct coupling of the rotation angle to the drivetrain more difficult
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-04-22
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.FEM import *
   
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
   
   color = [0.1,0.1,0.8,1]
   s = 0.1 #width of cube
   sx = 3*s #length of cube/body
   
   background0 = GraphicsDataRectangle(-1,-1,1,1,graphics.color.white)
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], 
                                      visualization=VObjectGround(graphicsData= [background0])))
   
   L0 = 0.2    #crank length
   L1 = 0.5    #connecting rod length
   L2 = 0.1    #cylinder length
   a = 0.025   #general width dimension
   c = 0.05    #piston radius
   
   discBig = 0.15
   discSmall = 0.05
   
   rho=7850 #steel density
   
   kJoint = 1e4 #for elastic support of crankshaft
   dJoint = 2e2 #for elastic support of crankshaft
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add crank:
   inertiaCrank = InertiaCuboid(density=rho, sideLengths=[L0,a,a])
   
   
   omega0 = [0,0,2*pi*100*0]    #initial angular velocity of bodies
   #ep0 = eulerParameters0
   p0 = [0,0,0]        #reference position / COM of crank
   v0 = [0,0,0]     #initial translational velocity
   
   color0= graphics.color.grey
   gGraphics0 = graphics.BrickXYZ(-0.5*L0,-a*0.9,-a*0.9,0.5*L0,a*0.9,a*0.9, color0)
   gGraphics0b = graphics.BrickXYZ(-0.5*L0,-a*0.9,-a*0.9+4*a,0.5*L0,a*0.9,a*0.9+4*a, color0)
   gGraphics0c = graphics.Cylinder([0.5*L0,0,-a],[0,0,6*a],a, graphics.color.darkgrey)
   gGraphics0d = graphics.Cylinder([-0.5*L0,0,3*a],[0,0,4*a],a, graphics.color.darkgrey)
   
   oRB0 = mbs.CreateRigidBody(inertia=inertiaCrank, 
                             referencePosition=p0, 
                             referenceRotationMatrix=np.eye(3),
                             initialVelocity=v0,
                             initialAngularVelocity=omega0, 
                             gravity=[0.,-9.81*0,0.],
                             graphicsDataList=[gGraphics0,gGraphics0b,gGraphics0c,gGraphics0d])
   
   nRB0 = mbs.GetObject(oRB0)['nodeNumber']
   
   mGround0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oGround, localPosition = [0,0,-a]))
   mRigid0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB0, localPosition = [0,0,-a]))
   mRigid0RotationCoordinate = mbs.AddMarker(MarkerNodeRotationCoordinate(nodeNumber=nRB0, rotationCoordinate=2))#z-axis
   
   useElasticSupport=True
   if not useElasticSupport:
       mbs.AddObject(GenericJoint(markerNumbers = [mRigid0,mGround0], 
                                  constrainedAxes=[1,1,1, 1,1,0],
                                  visualization= VObjectJointGeneric(axesRadius=a,axesLength=4*a)))
   else:
       mGround0b = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oGround, localPosition = [0,0,6*a]))
       mRigid0b = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB0, localPosition = [0,0,6*a]))
       mbs.AddObject(GenericJoint(markerNumbers = [mRigid0,mGround0], 
                                  constrainedAxes=[0,0,1, 1,1,0],
                                  visualization= VObjectJointGeneric(axesRadius=a,axesLength=4*a)))
       mbs.AddObject(CartesianSpringDamper(markerNumbers = [mRigid0,mGround0], 
                                           stiffness=[kJoint,kJoint,0],
                                           damping = [dJoint,dJoint,0]))
       mbs.AddObject(CartesianSpringDamper(markerNumbers = [mRigid0b,mGround0b], 
                                           stiffness=[kJoint,kJoint,0],
                                           damping = [dJoint,dJoint,0]))
   
   sCrankPos = mbs.AddSensor(SensorNode(nodeNumber=nRB0, #fileName="solution/sensorCrankPos.txt", 
                            storeInternal=True,
                            outputVariableType=exu.OutputVariableType.Position))
   sCrankAngVel = mbs.AddSensor(SensorNode(nodeNumber=nRB0, #fileName="solution/sensorCrankAngVel.txt", 
                            storeInternal=True,
                            outputVariableType=exu.OutputVariableType.AngularVelocity))
   sCrankAngle = mbs.AddSensor(SensorNode(nodeNumber=nRB0, #fileName="solution/sensorCrankAngle.txt", 
                            storeInternal=True,
                            outputVariableType=exu.OutputVariableType.Rotation))
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add connecting rods and pistons:
   for i in range(4):
       inertiaConrod = InertiaCuboid(density=rho, sideLengths=[L1,a,a])
       phi = i/4*(2*pi) #rotation of subsystem
       A = RotationMatrixZ(phi) #transformation of subsystem
   
   
       A2 = RotationMatrixZ(0)    
       v2 = np.array([0,0,0])
       offsetPiston=0
       if i==1 or i==3:
           phi2=np.arctan2(0.5*L0,L1) #additional conrod angle
           A2 = RotationMatrixZ(phi2) #additional transformation of conrod
           v2 = A@np.array([-0.5*L0,-0.25*L0,0])
           offsetPiston = L1*(1.-np.cos(phi2)) + 0.5*L0
   
       offZ = 0    
       if i < 2:
           Acrank = RotationMatrixZ(0)
       else:
           Acrank = RotationMatrixZ(pi)
           offZ = 4*a
       
       color1= graphics.color.grey
       gGraphics1 = graphics.BrickXYZ(-0.5*L1,-a*0.9,-a*0.9,0.5*L1,a*0.9,a*0.9, color1)
       omega1 = [0,0,0]    #initial angular velocity of bodies
   
       p1 = [0.5*L0+0.5*L1,0,2*a+offZ]        #reference position / COM of crank
       p1 = list(v2 + A @ np.array(p1))
       
       v1 = [0,0,0]     #initial translational velocity
       
       oRB1 = mbs.CreateRigidBody(inertia=inertiaConrod, 
                                 referencePosition=p1, 
                                 referenceRotationMatrix=A@A2, 
                                 initialAngularVelocity=omega1, 
                                 initialVelocity=v1,
                                 gravity=[0.,-9.81*0,0.],
                                 graphicsDataList=[gGraphics1])
       
       
       locPos0 = list(Acrank @ np.array([0.5*L0,0,a+offZ]))
       mbs.CreateGenericJoint(bodyNumbers=[oRB0,oRB1], 
                              position=locPos0, constrainedAxes=[1,1,1, 1,1,0],
                              useGlobalFrame=False,
                              axesRadius=0.5*a,axesLength=2*a)
       #alternative, using markers and objects:
       # mRigid01 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB0, localPosition = locPos0))  #connection crank->conrod
       # mRigid10 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB1, localPosition = [-0.5*L1,0,-a]))#connection conrod->crank
       # mbs.AddObject(GenericJoint(markerNumbers = [mRigid01,mRigid10], 
       #                            constrainedAxes=[1,1,1, 1,1,0],
       #                            visualization= VObjectJointGeneric(axesRadius=0.5*a,axesLength=2*a)))
           
       #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #add piston as Mass1D:
       
       axPiston = list(A @ np.array([L2,0,0]))
       refPosPiston = list(A @ np.array([0.5*L0+L1-offsetPiston,0,2*a+offZ]))
       gGraphicsPiston = graphics.Cylinder(pAxis=[0,0,0],vAxis=axPiston, radius=2*a, color=graphics.color.steelblue)
       pistonMass = 0.2
       if i==3: 
           pistonMass *=1.5 #add disturbance into system ...
           gGraphicsPiston = graphics.Cylinder(pAxis=[0,0,0],vAxis=axPiston, radius=2*a, color=graphics.color.red)
   
       n1D1 = mbs.AddNode(Node1D(referenceCoordinates=[0]))
       oPiston1 = mbs.AddObject(Mass1D(physicsMass = pistonMass, 
                                       nodeNumber = n1D1,
                                       referencePosition=refPosPiston,
                                       referenceRotation=A,
                                       visualization=VObjectMass1D(graphicsData=[gGraphicsPiston])))
       
       mbs.CreateSphericalJoint(bodyNumbers=[oPiston1,oRB1], position=[0,0,0],
                                constrainedAxes=[1,1,0], useGlobalFrame=False,
                                jointRadius=1.5*a)
       #alternatively, using markers and objects:
       # mRigid11 = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRB1, localPosition = [ 0.5*L1,0,0])) #connection to piston
       # mPiston = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oPiston1, localPosition = [ 0,0,0]))
       # mbs.AddObject(SphericalJoint(markerNumbers=[mRigid11,mPiston], 
       #                              constrainedAxes=[1,1,0],
       #                              visualization=VObjectJointSpherical(jointRadius=1.5*a)))
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #drive train
   inertiaDiscBig = InertiaCylinder(density=rho, length=2*a, outerRadius=discBig, axis=2)
   inertiaDiscSmall = InertiaCylinder(density=rho, length=2*a, outerRadius=discSmall, axis=2)
   #print("Jzz=", inertiaDiscBig.GetInertia6D()[2], 0.5*rho*pi*discBig**4*(2*a))
   
   gGraphicsDiscBig0a = graphics.Cylinder([0,0,-a],[0,0,2*a], discBig, graphics.color.lightred, 64)
   gGraphicsDiscBig0b = graphics.BrickXYZ(0,-0.25*a,-a*1.01, discBig, 0.25*a, a*1.01, graphics.color.lightgrey) #add something to the cylinder to see rotation
   gGraphicsDiscSmall0a = graphics.Cylinder([0,0,-a],[0,0,2*a], discSmall, graphics.color.lightred, 32)
   gGraphicsDiscSmall0b = graphics.BrickXYZ(0,-0.25*a,-a*1.01, discSmall, 0.25*a, a*1.01, graphics.color.lightgrey) #add something to the cylinder to see rotation
   
   #Gear0:
   nDT0 = mbs.AddNode(Node1D(referenceCoordinates = [0]))
   oDT0 = mbs.AddObject(Rotor1D(nodeNumber = nDT0, 
                                physicsInertia=inertiaDiscBig.GetInertia6D()[2],
                                referencePosition = [0,0,-2*a],
                                visualization=VObjectRotationalMass1D(graphicsData=[gGraphicsDiscBig0a,gGraphicsDiscBig0b])))
   
   mDT0Rigid = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oDT0, localPosition=[0,0,a]))
   mDT0Coordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nDT0, coordinate=0)) #coordinate for rotation
   
   mbs.AddObject(ObjectJointGeneric(markerNumbers=[mRigid0,mDT0Rigid],
                                  constrainedAxes=[0,0,0, 0,0,1],
                                  visualization= VObjectJointGeneric(axesRadius=0.6*a,axesLength=1.95*a)))
   
   #Gear1:
   nDT1 = mbs.AddNode(Node1D(referenceCoordinates = [0]))
   oDT1 = mbs.AddObject(Rotor1D(nodeNumber = nDT1, 
                                physicsInertia=inertiaDiscSmall.GetInertia6D()[2],
                                referencePosition = [discBig+discSmall,0,-2*a],
                                visualization=VObjectRotationalMass1D(graphicsData=[gGraphicsDiscSmall0a,gGraphicsDiscSmall0b])))
   
   mDT1Coordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nDT1, coordinate=0)) #coordinate for rotation
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mDT0Coordinate,mDT1Coordinate],
                                      factorValue1=-discSmall/discBig,
                                      visualization=VObjectConnectorCoordinate(show=False)))
   
   #Gear2:
   gGraphicsDiscAxis2 = graphics.Cylinder([0,0,-2*a],[0,0,7*a], a, graphics.color.grey)
   nDT2 = mbs.AddNode(Node1D(referenceCoordinates = [0]))
   oDT2 = mbs.AddObject(Rotor1D(nodeNumber = nDT2, 
                                physicsInertia=inertiaDiscBig.GetInertia6D()[2],
                                referencePosition = [discBig+discSmall,0,-5*a],
                                visualization=VObjectRotationalMass1D(graphicsData=[gGraphicsDiscAxis2,gGraphicsDiscBig0a,gGraphicsDiscBig0b])))
   
   mDT2Coordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nDT2, coordinate=0)) #coordinate for rotation
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mDT1Coordinate,mDT2Coordinate],factorValue1=1,
                                      visualization=VObjectConnectorCoordinate(show=False)))
   
   #Gear3:
   gGraphicsDiscAxis3 = graphics.Cylinder([0,0,-2*a],[0,0,4*a], a, graphics.color.grey)
   nDT3 = mbs.AddNode(Node1D(referenceCoordinates = [0]))
   oDT3 = mbs.AddObject(Rotor1D(nodeNumber = nDT3, 
                                physicsInertia=inertiaDiscSmall.GetInertia6D()[2],
                                referencePosition = [(discBig+discSmall)*2,0,-5*a],
                                visualization=VObjectRotationalMass1D(graphicsData=[gGraphicsDiscAxis3,gGraphicsDiscSmall0a,gGraphicsDiscSmall0b])))
   
   mDT3Coordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nDT3, coordinate=0)) #coordinate for rotation
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mDT2Coordinate,mDT3Coordinate],
                                      factorValue1=-discSmall/discBig,
                                      visualization=VObjectConnectorCoordinate(show=False)))
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #flywheel, connected with MarkerNodeRotationCoordinate:
   gGraphicsDiscFlyWheel0a = graphics.Cylinder([0,0,-2*a],[0,0,2*a], discBig, [0.4,0.9,0.4,0.5], 64)
   gGraphicsDiscFlyWheel0b = graphics.BrickXYZ(0,-0.25*a,-a*2.01, discBig, 0.25*a, a*0.01, [0.7,0.7,0.7,0.5]) #add something to the cylinder to see rotation
   nDT4 = mbs.AddNode(Node1D(referenceCoordinates = [0]))
   oDT4 = mbs.AddObject(Rotor1D(nodeNumber = nDT4, 
                                physicsInertia=5*inertiaDiscBig.GetInertia6D()[2],
                                referencePosition = [0,0,9*a],
                                visualization=VObjectRotationalMass1D(graphicsData=[gGraphicsDiscAxis3,gGraphicsDiscFlyWheel0a,gGraphicsDiscFlyWheel0b])))
   
   mDT4Coordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nDT4, coordinate=0)) #coordinate for rotation
   
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mDT4Coordinate,mRigid0RotationCoordinate],
                                      velocityLevel = True, #needed to securly compute multiple rotations
                                      visualization=VObjectConnectorCoordinate(show=False)))
   
   sFlyWheelAngVel = mbs.AddSensor(SensorBody(bodyNumber=oDT4, #fileName="solution/sensorFlyWheelAngVel.txt", 
                            storeInternal=True,
                            outputVariableType=exu.OutputVariableType.AngularVelocity))
   sFlyWheelAngle = mbs.AddSensor(SensorNode(nodeNumber=nDT4, #fileName="solution/sensorFlyWheelRotation.txt", 
                            storeInternal=True,
                            outputVariableType=exu.OutputVariableType.CoordinatesTotal))
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add torque (could also use LoadTorqueVector() on mDT0Rigid)
   def UFLoad(mbs, t, load):
       if t < 0.25:
           return load
       else:
           return 0
   mbs.AddLoad(LoadCoordinate(markerNumber=mDT3Coordinate, load=100, loadUserFunction=UFLoad))
   
   mbs.Assemble()
   #exu.Print(mbs)
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 0.1
   h=1e-4
   if useGraphics:
       tEnd = 2
       
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/1000
   simulationSettings.solutionSettings.sensorsWritePeriod = h
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
   #simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 
   
   simulationSettings.solutionSettings.solutionInformation = "rigid body tests"
   SC.visualizationSettings.nodes.defaultSize = 0.025
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   SC.visualizationSettings.nodes.showBasis = True
   
   #simulationSettings.displayComputationTime = True
   #simulationSettings.displayStatistics = True
   
   #simulationSettings.solutionSettings.recordImagesInterval = 0.005
   #SC.visualizationSettings.exportImages.saveImageFileName = "images/frame"
   SC.visualizationSettings.window.renderWindowSize = [1920,1080]
   SC.visualizationSettings.openGL.multiSampling = 4
   
   if useGraphics:
       SC.renderer.Start()
       if 'lastRenderState' in vars():
           SC.renderer.SetState(lastRenderState) #load last model view
       SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   phiCrank = mbs.GetSensorValues(sCrankAngle)[2]
   phiFlyWheel = mbs.GetSensorValues(sFlyWheelAngle) #scalar coordinate!
   
   phiCrankData = mbs.GetSensorStoredData(sCrankAngle)[:,2:4] #y,z values
   
   exu.Print("phiCrank",phiCrank)
   exu.Print("phiFlyWheel",phiFlyWheel)
   u = phiCrank-phiFlyWheel
   exu.Print("solution of driveTrainTest=", u)
   exudynTestGlobals.testError = u - (0.8813172426357362 - 0.8813173353288565) #2020-05-28: 0.8813172426357362 - 0.8813173353288565
   exudynTestGlobals.testResult = u
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
       lastRenderState = SC.renderer.GetState() #store model view for next simulation
   
       mbs.PlotSensor(sensorNumbers=[sFlyWheelAngle], 
                  components=[0], closeAll=True, offsets=-phiCrankData,
                  labels=['crank angle - flywheel angle'])
   
   if useGraphics:
       mbs.PlotSensor(sensorNumbers=[sCrankPos, sCrankAngVel, sCrankAngle, sFlyWheelAngVel, sFlyWheelAngle], 
                  components=[0,2,2,2,0], markerStyles=['^ ','o ','H ','x','v '],closeAll=True,markerSizes=12,
                  labels=['crank position','crank angular velocity','crank angle','flywheel angular velocity', 'flywheel angle'])
       


