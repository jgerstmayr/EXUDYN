
.. _examples-slidercrank3dwithancfbeltdrive:

*********************************
sliderCrank3DwithANCFbeltDrive.py
*********************************

You can view and download this file on Github: `sliderCrank3DwithANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Create slider-crank mechanism with belt drive modeled with ANCF cable elements
   #
   # Authors: David Wibmer and Dominik Sponring
   # Date: Created on Thu May  19 12:22:52 2020
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   # Notes: PROJECT Exercise:  Drive System + Crank System; VU Industrielle Mechatronik 2 - Robotics and Simulation
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #note: tested with PYTHON version = 3.6.10 and EXUDYN version = 0.1.342
   
   import exudyn as exu
   from exudyn.itemInterface import*
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   import numpy as np
   
   
   #Print EXUDYN version
   print('EXUDYN version='+exu.config.Version())
   
   
   #Paramters
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #General
   tSim = 2        #Simulation time + time to fasten belt
   duration = 0.25 #Time for fasten up belt
   
   omega = 80      #Angular velocity to controll
   sweep = True    #Controll a sweep of omega
   
   gravitation = False
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ##Crank-System##
   
   #Parameters crank
   L_A = 0.15                                  #[m]
   
   ba_0 = 0.1                                  #[m]
   ba_1 = 0.05                                 #[m]
   ba_2 = 0.05                                 #[m]
   
   Jxx = 0.005                                 #[kg m^2]
   Jyy = 0.007                                 #[kg m^2]
   Jzz = 0.0025                                #[kg m^2]
   
   massCrank = 0.5                             #[Kg]
   
   #Parameters Slider
   massSlider = 0.2                            #[Kg]
   
   #Parameters connection rod
   density = 7850                              #[Kg/m^3]
   h_B = 0.025                                 #[m]
   L_B = 0.3                                   #[m]
   V_CR = L_B*h_B**2                           #[m^3]
   massCR = V_CR*density                       #[Kg]
   Jxx_CR = (1/12)*massCR*(h_B**2 + h_B**2)    #[kg m^2]
   Jyy_CR = (1/12)*massCR*(h_B**2 + L_B**2)    #[kg m^2]
   Jzz_CR = (1/12)*massCR*(h_B**2 + L_B**2)    #[kg m^2]
   
   #Parameters bearing
                                               ##rigidity has been increased## 
   stiffnes = 4e5                              #[N/m]
   damping = 8e2                               #[N/(ms)]
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ##Belt-System##
                                 ##wheelbase has been inreased##
   L0 = 0.51                     #wheelbase [m]
   r0 = 0.05                     #radius disk 0 [m] 
   r1 = 0.1                      #radius disk 1 [m]
   a = 0.01                      #belt width [m]
   A = a*a                       #belt cross section [m^2]
   L_eff = 1.5                   #effective belt length [m]
   
   #Material parameters 
   E_B = 2e9                     #modulus of elasticity [N/m^2]
   rho = 1000                    #density [kg/m^3]
   I = a*a*a*a/12                #second moment of area of ANCF element in m^4
   mu = 0.7                      #friction coefficient []
   
   m_disk0 = 0.5                 #mass disk0[kg]
   m_disk1 = 1                   #mass disk1[kg]
   
   J_disk0 = m_disk0*r0*r0/2     #Moment of inertia disk0 [kg m^2]
   J_disk1 = m_disk1*r1*r1/2     #Moment of inertia disk1 [kg m^2]
   
   zOff = 0.12                    #additional offset, mainly for drawing reasons
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Creat system container
   SC = exu.SystemContainer()
   
   mbs = SC.AddSystem()
   
   #Generate ground
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   nGround = mbs.AddNode(NodePointGround())
   oGround = mbs.AddObject(ObjectGround(referencePosition=[0,0,zOff]))
   mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
   
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Create crank system
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #Generate Visualisation-Objects
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   vSlider = graphics.Cylinder([0.05,0,0], [-0.1,0,0],
                                  0.05, [1,0,0,1], nTiles=64)
   vRod = graphics.BrickXYZ(-L_B/2, -h_B/2, -h_B/2, L_B/2,
                                h_B/2, h_B/2, [0,1,0,1])
   
   vCrank0 = graphics.Cylinder([0,0,-2*ba_1], [0,0,0.01],
                                  r1+a/2,color=[0.3,0.3,0.9,1], nTiles=128)
   vCrank1 = graphics.Cylinder([0,0,0.01], [0,0,-ba_0-0.01],
                                  0.01,color=[0.3,0.3,0.9,1])
   vCrank2 = graphics.Cylinder([0,0,ba_1-0.01], [0,0,ba_2+0.01],
                                  0.01, color=[0.3,0.3,0.9,1])
   #vCrank3 = graphics.Cylinder([-L_A/2,0,-0.0125], [0,0,0.025],
   #                               0.1, color=[0.3,0.3,0.9,0.9])
   #vCrank4 = graphics.Cylinder([-L_A/2,0,0.0375], [0,0,0.025],
   #                               0.1, color=[0.3,0.3,0.9,0.9])
   
   vCrank3 = graphics.Brick([-L_A/2,0,+0.005], [L_A+0.01,0.01,0.008],
                                  color=[0.3,0.3,0.9,0.9])
   vCrank4 = graphics.Brick([-L_A/2,0,0.05-0.005], [L_A+0.01,0.01,0.008],
                                  color=[0.3,0.3,0.9,0.9])
   
   vCrank5 = graphics.Cylinder([-L_A,0,0.0], [0,0,0.05],
                                  0.01,color=[0.3,0.3,0.9,1])
   
   vDisk_line0 = GraphicsDataRectangle(0,-0.001,r0,0.001)
   vDisk_line1 = GraphicsDataRectangle(0,-0.001,r1,0.001)
   
   cylDisc0 = graphics.Cylinder([0,0,-0.005], [0,0,0.01],
                                  r0+a/2,color=[0.3,0.3,0.9,1], nTiles=64)
   
   #Generate Nodes and Objects
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   ##Crank##
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ep0 = [1,0,0,0] #no rotation
   nCrank_3D = mbs.AddNode(RigidEP(referenceCoordinates=[0,0,-ba_1/2+zOff]+ep0))
   oCrank_3D = mbs.AddObject(RigidBody(physicsMass=massCrank,
                                       physicsInertia=[Jxx,Jyy,Jzz,0,0,0],
                                       nodeNumber=nCrank_3D,
                                       visualization=VObjectRigidBody2D(graphicsData=[vCrank0,
                                                                                      vCrank1,
                                                                                      vCrank2,
                                                                                      vCrank3,
                                                                                      vCrank4,vCrank5])))
   
   ##Rod##
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   nRod = mbs.AddNode(RigidEP(referenceCoordinates=[-(L_A+L_B/2),0,0+zOff]+ep0));
   oRod = mbs.AddObject(RigidBody(physicsMass=massCR,
                                              physicsInertia=[Jxx_CR,Jyy_CR,Jzz_CR,0,0,0],
                                              nodeNumber=nRod,
                                              visualization=VObjectRigidBody2D(graphicsData=[vRod])))
   
   
   ##Slider##
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   nSlider = mbs.AddNode(Point(referenceCoordinates=[-(L_A+L_B), 0,0+zOff]))
   oSlider = mbs.AddObject(MassPoint(physicsMass = massSlider,
                                     nodeNumber = nSlider,
                                     visualization=VObjectMassPoint(graphicsData= [vSlider])))
   
   
   #Generate Joints
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   ##Markers##
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   mCrank_Rod = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank_3D,
                                              localPosition=[-L_A,0,ba_1/2]))
   mjoint2leftC = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank_3D,
                                                localPosition = [0,0,-ba_0]))
   mjoint2rightC = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank_3D,
                                                 localPosition = [0,0,ba_1+ba_2]))
   
   mRodLeft = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRod,
                                            localPosition=[-L_B/2,0,0]))
   mRodRight = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRod,
                                             localPosition=[ L_B/2,0,0])) 
   
   mSlider = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oSlider,
                                              localPosition=[0,0,0]))
   
   mjoint2leftG = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround,
                                                   localPosition=[0,0,-(ba_0+ba_1/2)]))
   mjoint2rightG = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround,
                                                    localPosition=[0,0,ba_2+ba_1/2]))
   
   ##Joints##
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   mbs.AddObject(SphericalJoint(markerNumbers = [mjoint2leftG,mjoint2leftC],
                                constrainedAxes = [0,0,0],
                                visualization = VObjectJointSpherical(jointRadius= 0.01)))
   mbs.AddObject(SphericalJoint(markerNumbers = [mjoint2rightG,mjoint2rightC],
                                constrainedAxes = [0,0,0],
                                visualization = VObjectJointSpherical(jointRadius= 0.01)))
   mbs.AddObject(SphericalJoint(markerNumbers = [mCrank_Rod,mRodRight],
                                constrainedAxes = [1,1,1],
                                visualization = VObjectJointSpherical(jointRadius= 0.01)))
   mbs.AddObject(SphericalJoint(markerNumbers = [mRodLeft,mSlider],
                                constrainedAxes = [1,1,1],
                                visualization = VObjectJointSpherical(jointRadius= 0.01)))
   
   
   ##Joints for bearing##
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   mbs.AddObject(CartesianSpringDamper(markerNumbers=[mjoint2leftG,mjoint2leftC],
                                       stiffness=[stiffnes,stiffnes,stiffnes],
                                       damping=[damping,damping,damping]))
   mbs.AddObject(CartesianSpringDamper(markerNumbers=[mjoint2rightG,mjoint2rightC],
                                       stiffness=[stiffnes,stiffnes,stiffnes],
                                       damping=[damping,damping,damping]))
   
   
   ##Constrains##
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Y-coordinate is constrained
   mSliderY = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nSlider,
                                                 coordinate=1))
   #Z-coordinate is constrained
   mSliderZ = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nSlider,
                                                 coordinate=2))
   
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mSliderY]))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mSliderZ]))
   
   
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Create Belt system
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   #Generate disks
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #Disk 1 (Driven Disk)
   nDisk1 = mbs.AddNode(Rigid2D(referenceCoordinates=[0, 0, 0]))
   oDisk1 = mbs.AddObject(RigidBody2D(physicsMass=m_disk1, physicsInertia=J_disk1,
                                      nodeNumber=nDisk1,
                                      visualization=VObjectRigidBody2D(graphicsData=[vDisk_line1])))
   mDisk1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oDisk1))
   
   #Disk 0 (Driver Disk)
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   nDisk0 = mbs.AddNode(Rigid2D(referenceCoordinates=[0, 0, 0]))
   oDisk0 = mbs.AddObject(RigidBody2D(physicsMass=m_disk0, physicsInertia=J_disk0,
                                      nodeNumber=nDisk0,
                                      visualization=VObjectRigidBody2D(graphicsData=[vDisk_line0,cylDisc0])))
   mDisk0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oDisk0))
   #Marker for contactCable
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   mBDisk0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oDisk0))
   #Marker for coordinateConstraint
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   mNDisk00 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nDisk0, coordinate=0))
   mNDisk01 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nDisk0, coordinate=1))
   
   
   #Generate disc joints
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   def OffsetUF(mbs, t, itemIndex, lOffset):
       if t < duration: return L0*(1-np.cos(t*np.pi/duration))/2
       else: return L0
   
   #Joint for Disk1 ->Rigid
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   mpoint1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround))
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[mpoint1, mDisk1]))
   #Joint for Disk0 ->tighten the belt
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround, mNDisk01]))
   oTighten = mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround, mNDisk00],
                                                 offsetUserFunction=OffsetUF))
   
   
   #Generate belt
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++           
   cableList = []
   nodeList = []
   markerList = []
   
   #Calculate element parameters
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   beltRadius = L_eff/(2*np.pi)
   nElements = 30*2
   angleSegments = 2*np.pi/nElements
   arcLenght = angleSegments*beltRadius
   
   #Create belt-nodes
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   for i in range(nElements):
       s = np.sin(angleSegments*i)
       c = np.cos(angleSegments*i)
       nodeList += [mbs.AddNode(Point2DS1(referenceCoordinates = [c*beltRadius,s*beltRadius,-s,c]))]
       
   #Create belt-objects
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   for i in range(1, nElements):
       cableList += [mbs.AddObject(Cable2D(physicsLength=arcLenght, physicsMassPerLength=rho*A,
                                           physicsBendingStiffness=E_B*I, physicsAxialStiffness=E_B*A,
                                           physicsReferenceCurvature=1/beltRadius,
                                           nodeNumbers=[nodeList[i-1],nodeList[i]]))]
   
   #Connect first and last belt-object
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   cableList += [mbs.AddObject(Cable2D(physicsLength=arcLenght, physicsMassPerLength=rho*A,
                               physicsBendingStiffness=E_B*I, physicsAxialStiffness=E_B*A,
                               physicsReferenceCurvature=1/beltRadius,
                               nodeNumbers=[nodeList[-1],nodeList[0]]))]
   
   
   
   #Add gravity
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if gravitation:
       for i in range(len(nodeList)):
           markerList +=  [mbs.AddMarker(MarkerNodePosition(nodeNumber=nodeList[i]))]
           mbs.AddLoad(Force(markerNumber=markerList[i], loadVector=[0, -9.81*rho*A*arcLenght, 0]))
   
   
   #Add contact
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
   #Contact parameters
   cStiffness = 1e6
   cDamping = 1000
   
   
   nSegments = 3
   initialGapList = [0.1, 0.1, 0.1]*nSegments
   
   for i in range(len(cableList)):
       #Generate markers for the ANCF-Elements
       mCable = mbs.AddMarker(MarkerBodyCable2DShape(bodyNumber=cableList[i], numberOfSegments=nSegments))
       #Generate contact for disk1
       nodeDataContactCable = mbs.AddNode(NodeGenericData(initialCoordinates=initialGapList, 
                                                          numberOfDataCoordinates=3*nSegments))
       mbs.AddObject(ObjectContactFrictionCircleCable2D(markerNumbers=[mDisk1, mCable],
                                                        nodeNumber = nodeDataContactCable,
                                                        contactStiffness = cStiffness,
                                                        contactDamping = cDamping,
                                                        frictionVelocityPenalty = 1000,
                                                        frictionCoefficient = mu,
                                                        circleRadius = r1+a/2))
       #Generate contact for disk0
       nodeDataContactCable = mbs.AddNode(NodeGenericData(initialCoordinates=initialGapList, 
                                                          numberOfDataCoordinates=3*nSegments))
       mbs.AddObject(ObjectContactFrictionCircleCable2D(markerNumbers=[mBDisk0, mCable],
                                                        nodeNumber = nodeDataContactCable,
                                                        contactStiffness = cStiffness,
                                                        contactDamping = cDamping,
                                                        frictionVelocityPenalty = 1000,
                                                        frictionCoefficient = mu,
                                                        circleRadius = r0+a/2))
   
   
   #Generate Load
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   def loadUF(mbs, t,loadVector):
       if t < 2*duration : return [0,0,0] 
       if sweep == True:
           omega_soll = 10 + omega*(t-2*duration)/tSim
       else:
           omega_soll = omega
       
       omega_ist = mbs.GetNodeOutput(nCrank_3D, exu.OutputVariableType.AngularVelocity)[2]
       
       tor = (omega_soll - omega_ist)*5
       
       return [0,0,tor]    
   
   
   #apply torque at disk0
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   mbs.AddLoad(Torque(markerNumber=mDisk0,
                      loadVectorUserFunction=loadUF))
   
   mbs.AddObject(GenericJoint(markerNumbers=[mDisk1,mjoint2rightC],
                              constrainedAxes=[0,0,0,0,0,1],
                              visualization=VObjectJointGeneric(show=False)))
   
   
   #Generate Sensors
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #Sensor for measuring disk0 
   mbs.AddSensor(SensorObject(objectNumber=oTighten, 
                              fileName='Preload_overallS.txt',
                              outputVariableType=exu.OutputVariableType.Force))
   mbs.AddSensor(SensorBody(bodyNumber=oDisk0, 
                            fileName='Pos_Disk0_overallS.txt',
                            outputVariableType=exu.OutputVariableType.Position))
   
   
   mbs.AddSensor(SensorNode(nodeNumber=nCrank_3D, 
                            fileName='Angular_velocity_overallS.txt',
                            outputVariableType=exu.OutputVariableType.AngularVelocity))
   
   
   #Assamble the system
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   mbs.Assemble()
   
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   simulationSettings = exu.SimulationSettings()
   
   simulationSettings.timeIntegration.numberOfSteps = tSim*1000
   simulationSettings.timeIntegration.endTime = tSim
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-10
   
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8*10 #10000
   simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10*100
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = False
   simulationSettings.timeIntegration.newton.maxModifiedNewtonIterations = 5
   simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
   simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 6.055454452393343e-06*0.1 #eps^(1/3)
   simulationSettings.timeIntegration.newton.modifiedNewtonContractivity = 1e8
   
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 
   simulationSettings.displayStatistics = True
   
   
   
   #Visualisation Settings
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
   SC.visualizationSettings.nodes.showNumbers = False
   SC.visualizationSettings.bodies.showNumbers = False
   SC.visualizationSettings.connectors.showNumbers = False
   SC.visualizationSettings.connectors.defaultSize = 0.005
   SC.visualizationSettings.contact.contactPointsDefaultSize = 0.01
   SC.visualizationSettings.connectors.showContact = True
   
   #create animation:
   if False:
       simulationSettings.solutionSettings.recordImagesInterval = 0.002
       SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
       SC.visualizationSettings.window.renderWindowSize = [1920,1080]
       SC.visualizationSettings.openGL.multiSampling = 4
   
   
   SC.renderer.Start()
   if 'lastRenderState' in vars():
       SC.renderer.SetState(lastRenderState) #load last model view
   SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   
   SC.renderer.DoIdleTasks()
   SC.renderer.Stop() #safely close rendering window!
   lastRenderState = SC.renderer.GetState() #store model view for next simulation
   
   
   
   
   #Show Sensor
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
   import matplotlib.pyplot as plt
   import matplotlib.ticker as ticker
   
   #Import sensor data
   data = np.loadtxt('Angular_velocity_overallS.txt', comments='#', delimiter=',')
   plt.figure(1)
   plt.plot(data[:,0], data[:,3], 'r-', label='controlled sweep')
   
   
   ax=plt.gca() # get current axes
   ax.grid(True, 'major', 'both')
   ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
   plt.xlabel('time (s)')
   ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
   plt.ylabel('Angular velocity')
   plt.legend() #show labels as legend
   plt.tight_layout()
   plt.show() 
   
   
   #Import sensor data
   data = np.loadtxt('Preload_overallS.txt', comments='#', delimiter=',')
   plt.figure(2)
   # 1.column = time  2.column = load 
   plt.plot(data[:,0], data[:,1], 'r-', label='Preload')
   
   
   ax=plt.gca() # get current axes
   ax.grid(True, 'major', 'both')
   ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
   plt.xlabel('time (s)')
   ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
   plt.ylabel('force (N)')
   plt.legend() #show labels as legend
   plt.tight_layout()
   plt.show() 
   
   
   
   
   #Import sensor data
   data = np.loadtxt('Pos_Disk0_overallS.txt', comments='#', delimiter=',')
   plt.figure(3)
   # 1.column = time  2.column = x-axis
   plt.plot(data[:,0], data[:,1], 'r-', label='Position X-Axis')
   
   
   ax=plt.gca() # get current axes
   ax.grid(True, 'major', 'both')
   ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
   plt.xlabel('time (s)')
   ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
   plt.ylabel('position (m)')
   plt.legend() #show labels as legend
   plt.tight_layout()
   plt.show() 
   
   
   
   
   
   
   
   
   
   
   
   


