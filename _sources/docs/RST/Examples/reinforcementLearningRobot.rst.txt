
.. _examples-reinforcementlearningrobot:

*****************************
reinforcementLearningRobot.py
*****************************

You can view and download this file on Github: `reinforcementLearningRobot.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/reinforcementLearningRobot.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Reinforcement learning example with stable-baselines3; 
   #           training a mobile platform with differential drives to meet target points
   #           NOTE: frictional contact requires small enough step size to avoid artifacts!
   #           GeneralContact works less stable than RollingDisc objects
   #
   # Author:    Johannes Gerstmayr
   # Date:      2024-04-27
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #NOTE: this model is using the stable-baselines3 version 1.7.0, which requires:
   #pip install exudyn
   #pip install pip install wheel==0.38.4 setuptools==66.0.0
   #      => this downgrades setuptools to be able to install gym==0.21
   #pip install stable-baselines3==1.7.0
   #tested within a virtual environment: conda create -n venvP311 python=3.11 scipy matplotlib tqdm spyder-kernels=2.5 ipykernel psutil -y
   
   import sys
   sys.exudynFast = True
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.robotics import *
   from exudyn.artificialIntelligence import *
   import math
   
   import copy
   import os
   os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"
   import torch 
   
   import stable_baselines3
   useOldGym = tuple(map(int, stable_baselines3.__version__.split('.'))) <= tuple(map(int, '1.8.0'.split('.')))
   
   ##%% here the number of links can be changed. Note that for n < 3 the actuator 
   
   #**function: Add model of differential drive robot (two wheels which can be actuated independently);
   #            model is parameterized for kinematics, inertia parameters as well as for graphics;
   #            the model is created as a minimum coordinate model to use it together with explicit integration;
   #            a contact model is added if it does not exist; 
   def DifferentialDriveRobot(SC, mbs, 
                              platformInertia = None,
                              wheelInertia = None,
                              platformPosition = [0,0,0], #this is the location of the platform ground centerpoint
                              wheelDistance = 0.4,        #wheel midpoint-to-midpoint distance
                              platformHeight = 0.1,
                              platformRadius = 0.22,
                              platformMass = 5,
                              platformGroundOffset = 0.02,
                              planarPlatform = True,
                              dimGroundX = 8, dimGroundY = 8,
                              gravity = [0,0,-9.81],
                              wheelRadius = 0.04,
                              wheelThickness = 0.01,
                              wheelMass = 0.05,
                              pControl = 0,
                              dControl = 0.02,
                              usePenalty = True, #use penalty formulation in case useGeneralContact=False
                              frictionProportionalZone = 0.025,
                              frictionCoeff = 1, stiffnessGround = 1e5,
                              gContact = None, 
                              frictionIndexWheel = None, frictionIndexFree = None, 
                              useGeneralContact = False #generalcontact shows large errors currently
                              ):
   
       #add class which can be returned to enable user to access parameters
       class ddr: pass 
   
       #+++++++++++++++++++++++++++++++++++++++++++
       #create contact (if not provided)
       ddr.gGround = graphics.CheckerBoard(normal= [0,0,1], 
                                              size=dimGroundX, size2=dimGroundY, nTiles=8)
       
       ddr.oGround= mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                               visualization=VObjectGround(graphicsData= [ddr.gGround])))
       ddr.mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=ddr.oGround))
       
       
       ddr.frictionCoeff = frictionCoeff
       ddr.stiffnessGround = stiffnessGround
       ddr.dampingGround = ddr.stiffnessGround*0.01
       if gContact == None and useGeneralContact:
           frictionIndexGround = 0
           frictionIndexWheel = 0
           frictionIndexFree = 1
   
           ddr.gContact = mbs.AddGeneralContact()
           ddr.gContact.frictionProportionalZone = frictionProportionalZone
           #ddr.gContact.frictionVelocityPenalty = 1e4
   
           ddr.gContact.SetFrictionPairings(np.diag([ddr.frictionCoeff,0])) #second index is for frictionless contact
           ddr.gContact.SetSearchTreeCellSize(numberOfCells=[4,4,1]) #just a few contact cells
   
           #add ground to contact
           [meshPoints, meshTrigs] = graphics.ToPointsAndTrigs(ddr.gGround) #could also use only 1 quad ...
   
           ddr.gContact.AddTrianglesRigidBodyBased( rigidBodyMarkerIndex = ddr.mGround, 
                                               contactStiffness = ddr.stiffnessGround, contactDamping = ddr.dampingGround, 
                                               frictionMaterialIndex = frictionIndexGround,
                                               pointList=meshPoints, triangleList=meshTrigs)
   
   
       #+++++++++++++++++++++++++++++++++++++++++++
       #create inertias (if not provided)
       if wheelInertia == None:
           ddr.iWheel = InertiaCylinder(wheelMass/(wheelRadius**2*pi*wheelThickness), 
                                    wheelThickness, wheelRadius, axis=0) #rotation about local X-axis
       else:
           ddr.iWheel = RigidBodyInertia(mass=wheelMass, inertiaTensorAtCOM=np.diag(wheelInertia))
   
       if platformInertia == None:
           ddr.iPlatform = InertiaCylinder(platformMass/(platformRadius**2*pi*platformHeight), 
                                    platformHeight, platformRadius, axis=0) #rotation about local X-axis
           ddr.iPlatform = ddr.iPlatform.Translated([0,0,0.5*platformHeight+platformGroundOffset]) #put COM at mid of platform; but referencepoint is at ground level!
       else:
           ddr.iPlatform = RigidBodyInertia(mass=platformMass, inertiaTensorAtCOM=np.diag(platformInertia))
       
       #+++++++++++++++++++++++++++++++++++++++++++
       #create kinematic tree for wheeled robot    
       ddr.gPlatform = [graphics.Cylinder([0,0,platformGroundOffset], [0,0,platformHeight], platformRadius, color=graphics.color.steelblue, nTiles=64, addEdges=True, addFaces=False)]
       ddr.gPlatform += [graphics.Cylinder([0,platformRadius*0.8,platformGroundOffset*1.5], [0,0,platformHeight], platformRadius*0.2, color=graphics.color.grey)]
       ddr.gPlatform += [graphics.Basis(length=0.1)]
       ddr.gWheel = [graphics.Cylinder([-wheelThickness*0.5,0,0], [wheelThickness,0,0], wheelRadius, color=graphics.color.red, nTiles=32)]
       ddr.gWheel += [graphics.Brick([0,0,0],[wheelThickness*1.1,wheelRadius*1.3,wheelRadius*1.3], color=graphics.color.grey)]
       ddr.gWheel += [graphics.Basis(length=0.075)]
   
       #create node for unknowns of KinematicTree
       ddr.nJoints = 3+3+2 - 3*planarPlatform #6 (3 in planar case) for the platform and 2 for the wheels;
       referenceCoordinates=[0.]*ddr.nJoints
       referenceCoordinates[0:len(platformPosition)] = platformPosition
       ddr.nKT = mbs.AddNode(NodeGenericODE2(referenceCoordinates=referenceCoordinates,
                                                  initialCoordinates=[0.]*ddr.nJoints,
                                                  initialCoordinates_t=[0.]*ddr.nJoints,
                                                  numberOfODE2Coordinates=ddr.nJoints))
   
       ddr.linkMasses = []
       ddr.gList = [] #list of graphics objects for links
       ddr.linkCOMs = exu.Vector3DList()
       ddr.linkInertiasCOM=exu.Matrix3DList()
       ddr.jointTransformations=exu.Matrix3DList()
       ddr.jointOffsets = exu.Vector3DList()
       ddr.jointTypes = [exu.JointType.PrismaticX,exu.JointType.PrismaticY,exu.JointType.RevoluteZ]
       ddr.linkParents = list(np.arange(3)-1)
   
       ddr.platformIndex = 2
       if not planarPlatform:
           ddr.jointTypes+=[exu.JointType.PrismaticZ,exu.JointType.RevoluteY,exu.JointType.RevoluteX]
           ddr.linkParents+=[2,3,4]
           ddr.platformIndex = 5
   
       #add data for wheels:
       ddr.jointTypes += [exu.JointType.RevoluteX]*2
       ddr.linkParents += [ddr.platformIndex]*2
   
       #now create offsets, graphics list and inertia for all links
       for i in range(len(ddr.jointTypes)):
           ddr.jointTransformations.Append(np.eye(3))
           
           if i < ddr.platformIndex:
               ddr.gList += [[]]
               ddr.jointOffsets.Append([0,0,0])
               ddr.linkInertiasCOM.Append(np.zeros([3,3]))
               ddr.linkCOMs.Append([0,0,0])
               ddr.linkMasses.append(0)
           elif i == ddr.platformIndex:
               ddr.gList += [ddr.gPlatform]
               ddr.jointOffsets.Append([0,0,0])
               ddr.linkInertiasCOM.Append(ddr.iPlatform.InertiaCOM())
               ddr.linkCOMs.Append(ddr.iPlatform.COM())
               ddr.linkMasses.append(ddr.iPlatform.Mass())
           else: 
               ddr.gList += [ddr.gWheel]
               sign = -1+(i>ddr.platformIndex+1)*2
               offZ = wheelRadius
               if planarPlatform and (useGeneralContact or usePenalty): 
                   offZ *= 0.999 #to ensure contact
               ddr.jointOffsets.Append([sign*wheelDistance*0.5,0,offZ])
               ddr.linkInertiasCOM.Append(ddr.iWheel.InertiaCOM())
               ddr.linkCOMs.Append(ddr.iWheel.COM())
               ddr.linkMasses.append(ddr.iWheel.Mass())
           
   
       ddr.jointDControlVector = [0]*ddr.nJoints
       ddr.jointPControlVector = [0]*ddr.nJoints
       ddr.jointPositionOffsetVector = [0]*ddr.nJoints
       ddr.jointVelocityOffsetVector = [0]*ddr.nJoints
   
       ddr.jointPControlVector[-2:] = [pControl]*2
       ddr.jointDControlVector[-2:] = [dControl]*2
       
       
       #create KinematicTree
       ddr.oKT = mbs.AddObject(ObjectKinematicTree(nodeNumber=ddr.nKT, 
                                         jointTypes=ddr.jointTypes, 
                                         linkParents=ddr.linkParents,
                                         jointTransformations=ddr.jointTransformations, 
                                         jointOffsets=ddr.jointOffsets, 
                                         linkInertiasCOM=ddr.linkInertiasCOM, 
                                         linkCOMs=ddr.linkCOMs, 
                                         linkMasses=ddr.linkMasses, 
                                         baseOffset = [0.,0.,0.], gravity=gravity,
                                         jointPControlVector=ddr.jointPControlVector,
                                         jointDControlVector=ddr.jointDControlVector,
                                         jointPositionOffsetVector=ddr.jointPositionOffsetVector,
                                         jointVelocityOffsetVector=ddr.jointVelocityOffsetVector,
                                         visualization=VObjectKinematicTree(graphicsDataList = ddr.gList)))
   
       ddr.sPlatformPos = mbs.AddSensor(SensorKinematicTree(objectNumber=ddr.oKT, linkNumber = ddr.platformIndex,
                                                            storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
       ddr.sPlatformVel = mbs.AddSensor(SensorKinematicTree(objectNumber=ddr.oKT, linkNumber = ddr.platformIndex,
                                                            storeInternal=True, outputVariableType=exu.OutputVariableType.Velocity))
       ddr.sPlatformAng = mbs.AddSensor(SensorKinematicTree(objectNumber=ddr.oKT, linkNumber = ddr.platformIndex,
                                                            storeInternal=True, outputVariableType=exu.OutputVariableType.Rotation))
       ddr.sPlatformAngVel = mbs.AddSensor(SensorKinematicTree(objectNumber=ddr.oKT, linkNumber = ddr.platformIndex,
                                                            storeInternal=True, outputVariableType=exu.OutputVariableType.AngularVelocity))
       
       #create markers for wheels and add contact
       ddr.mWheels = []
       for i in range(2):
           mWheel = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=ddr.oKT, 
                                                           linkNumber=ddr.platformIndex+1+i, 
                                                           localPosition=[0,0,0]))
           ddr.mWheels.append(mWheel)
           if useGeneralContact:
               ddr.gContact.AddSphereWithMarker(mWheel, 
                                                radius=wheelRadius,
                                                contactStiffness=ddr.stiffnessGround,
                                                contactDamping=ddr.dampingGround,
                                                frictionMaterialIndex=frictionIndexWheel)
               #for 3D platform, we need additional support points:
               if not planarPlatform:
                   rY = platformRadius-platformGroundOffset
                   mPlatformFront = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=ddr.oKT, 
                                                                   linkNumber=ddr.platformIndex, 
                                                                   localPosition=[0,rY,1.01*platformGroundOffset]))
                   mPlatformBack = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=ddr.oKT, 
                                                                   linkNumber=ddr.platformIndex, 
                                                                   localPosition=[0,-rY,1.01*platformGroundOffset]))
                   
                   fact = 1
                   ddr.gContact.AddSphereWithMarker(mPlatformFront, 
                                                    radius=platformGroundOffset,
                                                    contactStiffness=ddr.stiffnessGround*fact,
                                                    contactDamping=ddr.dampingGround*fact,
                                                    frictionMaterialIndex=frictionIndexFree)
                   ddr.gContact.AddSphereWithMarker(mPlatformBack, 
                                                    radius=platformGroundOffset,
                                                    contactStiffness=ddr.stiffnessGround*fact,
                                                    contactDamping=ddr.dampingGround*fact,
                                                    frictionMaterialIndex=frictionIndexFree)
           else:
               if not planarPlatform:
                   raise ValueError('DifferentialDriveRobot: if useGeneralContact==False then planarPlatform must be True!')
               if not usePenalty:
                   ddr.oRollingDisc = mbs.AddObject(ObjectJointRollingDisc(markerNumbers=[ddr.mGround , mWheel], 
                                                        constrainedAxes=[i,1,1-planarPlatform], discRadius=wheelRadius,
                                                        visualization=VObjectJointRollingDisc(discWidth=wheelThickness,color=graphics.color.blue)))
               else:
                   nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
                   ddr.oRollingDisc = mbs.AddObject(ObjectConnectorRollingDiscPenalty(markerNumbers=[ddr.mGround , mWheel], 
                                                   nodeNumber = nGeneric,
                                                   discRadius=wheelRadius, 
                                                   useLinearProportionalZone=True, 
                                                   dryFrictionProportionalZone=0.05,
                                                   contactStiffness=ddr.stiffnessGround, 
                                                   contactDamping=ddr.dampingGround, 
                                                   dryFriction=[ddr.frictionCoeff]*2,
                                                   visualization=VObjectConnectorRollingDiscPenalty(discWidth=wheelThickness,color=graphics.color.blue)))
               
   
   
       #compute wheel velocities for given forward and rotation velocity
       def WheelVelocities(forwardVelocity, vRotation, wheelRadius, wheelDistance):
           vLeft = -forwardVelocity/wheelRadius
           vRight = vLeft
           vOff = vRotation*wheelDistance*0.5/wheelRadius
           vLeft += vOff
           vRight -= vOff
           return [vLeft, vRight]
   
       ddr.WheelVelocities = WheelVelocities
   
       #add some useful graphics settings
       
       SC.visualizationSettings.general.circleTiling=200
       SC.visualizationSettings.general.drawCoordinateSystem=True
       SC.visualizationSettings.loads.show=False
       SC.visualizationSettings.bodies.show=True
       SC.visualizationSettings.markers.show=False
       SC.visualizationSettings.bodies.kinematicTree.frameSize = 0.1
       SC.visualizationSettings.bodies.kinematicTree.showJointFrames = False
       
       SC.visualizationSettings.nodes.show=True
       # SC.visualizationSettings.nodes.showBasis =True
       SC.visualizationSettings.nodes.drawNodesAsPoint = False
       SC.visualizationSettings.nodes.defaultSize = 0 #must not be -1, otherwise uses autocomputed size
       
       SC.visualizationSettings.openGL.multiSampling = 4
       # SC.visualizationSettings.openGL.shadow = 0.25
       #SC.visualizationSettings.openGL.light0position = [-3,3,10,0]
       # SC.visualizationSettings.contact.showBoundingBoxes = True
       SC.visualizationSettings.contact.showTriangles = True
       SC.visualizationSettings.contact.showSpheres = True
   
       return ddr
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #for testing with a simple trajectory:
   if False:
       SC = exu.SystemContainer()
       mbs = SC.AddSystem()
       
       useGeneralContact = False
       usePenalty = True
       wheelRadius = 0.04
       wheelDistance = 0.4
       ddr = DifferentialDriveRobot(SC, mbs,useGeneralContact=useGeneralContact, 
                                    usePenalty=usePenalty, planarPlatform=True,
                                    wheelRadius=wheelRadius, wheelDistance=wheelDistance)
       mbs.Assemble()
           
       #create some nice trajectory
       def PreStepUserFunction(mbs, t):
           vSet = ddr.jointVelocityOffsetVector #nominal values
           vSet[-2:] = [0,0]
   
           if t < 2:
               vSet[-2:] = ddr.WheelVelocities(0.5, 0, wheelRadius, wheelDistance)
           elif t < 3: pass
           elif t < 4:
               vSet[-2:] = ddr.WheelVelocities(0, 0.5*pi, wheelRadius, wheelDistance)
           elif t < 5: pass
           elif t < 7:
               vSet[-2:] = ddr.WheelVelocities(-1, 0, wheelRadius, wheelDistance)
           elif t < 8: pass
           elif t < 9:
               vSet[-2:] = ddr.WheelVelocities(0.5, 0.5*pi, wheelRadius, wheelDistance)
   
           mbs.SetObjectParameter(ddr.oKT, "jointVelocityOffsetVector", vSet)
   
           return True
   
       mbs.SetPreStepUserFunction(PreStepUserFunction)
   
       tEnd = 12 #tEnd = 0.8 for test suite
       stepSize = 0.002 #h= 0.0002 for test suite
       if useGeneralContact or usePenalty:
           stepSize = 2e-4
       # h*=0.1
       # tEnd*=3
       simulationSettings = exu.SimulationSettings()
       simulationSettings.solutionSettings.solutionWritePeriod = 0.01
       simulationSettings.solutionSettings.writeSolutionToFile = False
       simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/coordinatesSolution.txt'
       
       simulationSettings.solutionSettings.sensorsWritePeriod = stepSize*10
       # simulationSettings.displayComputationTime = True
       # simulationSettings.displayStatistics = True
       # simulationSettings.timeIntegration.verboseMode = 1
       #simulationSettings.timeIntegration.simulateInRealtime = True
       simulationSettings.timeIntegration.discontinuous.maxIterations = 1 #speed up
       #simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-5
       
       
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys['renderState'])
       SC.renderer.DoIdleTasks()
       
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.timeIntegration.explicitIntegration.computeEndOfStepAccelerations = False #increase performance, accelerations less accurate
       
       SC.visualizationSettings.window.renderWindowSize=[1600,1024]
       SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
   
       if useGeneralContact or usePenalty:
           mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitEuler)
           # mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitMidpoint)
       else:
           mbs.SolveDynamic(simulationSettings)
       
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
       if True:
           mbs.PlotSensor(ddr.sPlatformVel, components=[0,1],closeAll=True)
           mbs.PlotSensor(ddr.sPlatformAngVel, components=[0,1,2])
   
   def Rot2D(phi): 
       return np.array([[np.cos(phi),-np.sin(phi)],
                        [np.sin(phi), np.cos(phi)]])
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   class RobotEnv(OpenAIGymInterfaceEnv):
           
       #**classFunction: OVERRIDE this function to create multibody system mbs and setup simulationSettings; call Assemble() at the end!
       #                 you may also change SC.visualizationSettings() individually; kwargs may be used for special setup
       def CreateMBS(self, SC, mbs, simulationSettings, **kwargs):
   
           #%%++++++++++++++++++++++++++++++++++++++++++++++
           self.mbs = mbs
           self.SC = SC
   
           self.dimGroundX = 4 #dimension of ground
           self.dimGroundY = 4
           self.maxRotations = 0.6 #maximum number before learning stops
           
           self.maxWheelSpeed = 2*pi #2*pi = 1 revolution/second
           self.wheelRadius = 0.04
           self.wheelDistance = 0.4
           self.maxVelocity = self.wheelRadius * self.maxWheelSpeed
           self.maxPlatformAngVel = self.wheelRadius/(self.wheelDistance*0.5)*self.maxWheelSpeed
           
           useGeneralContact = False
           usePenalty = True
           ddr = DifferentialDriveRobot(SC, mbs,useGeneralContact=useGeneralContact, 
                                        dimGroundX=self.dimGroundY, dimGroundY=self.dimGroundY,
                                        usePenalty=usePenalty, 
                                        planarPlatform=True,
                                        stiffnessGround=1e4,
                                        wheelRadius=self.wheelRadius, 
                                        wheelDistance=self.wheelDistance)
           
           self.ddr = ddr
           self.oKT = ddr.oKT
           self.nKT = ddr.nKT
   
           #add graphics for desination
           gDestination = graphics.Sphere(point=[0,0,0.05],radius = 0.02, color=graphics.color.red, nTiles=16)
           self.oDestination = mbs.CreateGround(graphicsDataList=[gDestination])
   
           mbs.Assemble()
           self.stepSize = 1e-3
           self.stepUpdateTime = 0.05
           
           simulationSettings.solutionSettings.solutionWritePeriod = 0.1
   
           writeSolutionToFile = False
           if 'writeSolutionToFile' in kwargs:
               writeSolutionToFile = kwargs['writeSolutionToFile']
   
           useGraphics = False
           if 'useGraphics' in kwargs:
               useGraphics = kwargs['useGraphics']
   
           simulationSettings.solutionSettings.writeSolutionToFile = writeSolutionToFile 
           simulationSettings.solutionSettings.writeSolutionToFile = False
   
           simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/coordinatesSolution.txt'
           
           # simulationSettings.displayComputationTime = True
           #simulationSettings.displayStatistics = True
           #simulationSettings.timeIntegration.verboseMode = 1
           #simulationSettings.timeIntegration.simulateInRealtime = True
           simulationSettings.timeIntegration.discontinuous.maxIterations = 1 #speed up
           #simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-5
           
           
           simulationSettings.timeIntegration.numberOfSteps = int(self.stepUpdateTime/self.stepSize)
           simulationSettings.timeIntegration.endTime = self.stepUpdateTime
           simulationSettings.timeIntegration.explicitIntegration.computeEndOfStepAccelerations = False #increase performance, accelerations less accurate
           
           SC.visualizationSettings.window.renderWindowSize=[1600,1024]
           SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
           
           #+++++++++++++++++++++++++++++++++++++++++++++++++++++
           self.randomInitializationValue = [0.4*self.dimGroundX, 0.4*self.dimGroundY, self.maxRotations*2*pi*0.99,
                                             self.maxVelocity*0,self.maxVelocity*0,self.maxPlatformAngVel*0,
                                             0.3*self.dimGroundX, 0.3*self.dimGroundY, #destination points
                                             ]
           
           #must return state size
           self.numberOfStates = 3 #posx, posy, rot
           self.destinationStates = 2 #define here, if destination is included in states
           self.destination = [0.,0.] #default value for destination
   
           return self.destinationStates + self.numberOfStates * 2 #the number of states (position/velocity that are used by learning algorithm)
   
       #**classFunction: OVERRIDE this function to set up self.action_space and self.observation_space
       def SetupSpaces(self):
   
           high = np.array(
               [
                   self.dimGroundX*0.5,
                   self.dimGroundY*0.5,
                   2*pi*self.maxRotations #10 full revolutions; no more should be needed for any task
               ] +
               [                
                   np.finfo(np.float32).max,
               ] * self.numberOfStates + 
               [self.dimGroundX*0.5, 
                self.dimGroundY*0.5]*(self.destinationStates>0)
               ,
               dtype=np.float32,
           )
           
           
           #+++++++++++++++++++++++++++++++++++++++++++++++++++++
           #see https://github.com/openai/gym/blob/64b4b31d8245f6972b3d37270faf69b74908a67d/gym/core.py#L16
           #for Env:
               
           self.action_space = spaces.Box(low=np.array([-self.maxWheelSpeed,-self.maxWheelSpeed], dtype=np.float32),
                                          high=np.array([self.maxWheelSpeed,self.maxWheelSpeed], dtype=np.float32), dtype=np.float32)
           
           self.observation_space = spaces.Box(-high, high, dtype=np.float32)        
           #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
       #**classFunction: this function is overwritten to map the action given by learning algorithm to the multibody system (environment)
       def MapAction2MBS(self, action):
           # force = action[0] * self.force_mag
           # self.mbs.SetLoadParameter(self.lControl, 'load', force)
           vSet = self.ddr.jointVelocityOffsetVector #nominal values
           vSet[-2:] = action
           # vSet[-1] = vSet[-2]
           # vSet[-2:] = [2,2.5]
           # print('action:', action)
   
           self.mbs.SetObjectParameter(self.oKT, "jointVelocityOffsetVector", vSet)
           
   
       #**classFunction: this function is overwrritten to collect output of simulation and map to self.state tuple
       #**output: return bool done which contains information if system state is outside valid range
       def Output2StateAndDone(self):
           
           #+++++++++++++++++++++++++
           #implemented for planar model only!
           statesVector =  self.mbs.GetNodeOutput(self.nKT, variableType=exu.OutputVariableType.Coordinates)[0:self.numberOfStates]
           statesVectorGlob_t =  self.mbs.GetNodeOutput(self.nKT, variableType=exu.OutputVariableType.Coordinates_t)[0:self.numberOfStates]
   
           # vLoc = Rot2D(statesVector[2]).T @ statesVectorGlob_t[0:2]
           # print('vLoc=',vLoc)
           # statesVector_t = np.array([vLoc[1], statesVectorGlob_t[2]])
           statesVector_t = statesVectorGlob_t #change to local in future!
           
           self.state = list(statesVector) + list(statesVector_t)
           if self.destinationStates:
               self.state += list(self.destination)
           self.state = tuple(self.state)
   
           done = bool(
               statesVector[0] < -self.dimGroundX
               or statesVector[0] > self.dimGroundX
               or statesVector[1] < -self.dimGroundY
               or statesVector[1] > self.dimGroundY
               or statesVector[2] < -self.maxRotations*2*pi
               or statesVector[2] > self.maxRotations*2*pi
               )
   
           return done
   
       
       #**classFunction: OVERRIDE this function to map the current state to mbs initial values
       #**output: return [initialValues, initialValues\_t] where initialValues[\_t] are ODE2 vectors of coordinates[\_t] for the mbs
       def State2InitialValues(self):
           #+++++++++++++++++++++++++++++++++++++++++++++
           #states: x, y, phi, x_t, y_t, phi_t
           initialValues = list(self.state[0:self.numberOfStates])+[0,0] #wheels do not initialize
           initialValues_t = list(self.state[self.numberOfStates:2*self.numberOfStates])+[0,0]
           
           if self.destinationStates:
               if self.destination[0] != self.state[-2] or self.destination[1] != self.state[-1]:
                   # print('set new destination:', self.destination)
                   self.destination = self.state[-2:] #last two values are destination
                   self.mbs.SetObjectParameter(self.oDestination, 'referencePosition', 
                                                list(self.destination)+[0])
           
           return [initialValues,initialValues_t]
           
       def getReward(self): 
           X = self.dimGroundX
           Y = self.dimGroundY
           v = np.array([self.destination[0] - self.state[0], self.destination[1] - self.state[1]])
           dist = NormL2(v)
           
           phi = self.state[2]
           localSpeed = Rot2D(phi).T @ [self.state[3],self.state[4]]
           forwardSpeed = localSpeed[1]
           
           reward = 1
           #take power of 0.5 of dist to penalize small distances
           #reward -= (dist/(0.5*NormL2([X,Y])))**0.5
           
           #add penalty on rotations at a certain time (at beginning rotation may be needed...)
           #reward -= 0.2*abs(self.state[5])/self.maxPlatformAngVel
           # t = self.mbs.systemData.GetTime()
           # if t > 4:
           #     fact = 1
           #     if t < 5: fact  = 5-t
           #     reward -= fact*0.1*abs(self.state[5])/self.maxPlatformAngVel
           
           #add penalty on reverse velocity: this supports solutions in forward direction!
           # backwardMaxSpeed = 0.1
           # if forwardSpeed < -backwardMaxSpeed*self.maxVelocity:
           #     reward -= abs(forwardSpeed)/self.maxVelocity+backwardMaxSpeed
   
           reward -= 0.5*abs(forwardSpeed)
           
           if dist > 0:
               v0 = v*(1/dist)
               vDir = Rot2D(phi) @ [0,1]
               # print('v0=',v0,', dir=',vDir)
               reward -= NormL2(vDir-v0)*0.5
           
           # print('rew=', round(reward,3), ', vF=', round(0.5*abs(forwardSpeed),3), 
           #       ', dir=', round(NormL2(vDir-v0)*0.5,4),
           #       'v0=', v0, 'vDir=',vDir)
           
           #reward -= max(0,abs(self.state[2])-pi)/(4*pi)
           if reward < 0: reward = 0
           
           # print('forwardSpeed',round(forwardSpeed/self.maxVelocity,3),
           #       ', reward',round(reward,3))
   
           # print('reward=',reward, ', t=', self.mbs.systemData.GetTime())
           return reward 
   
       #**classFunction: openAI gym interface function which is called to compute one step
       def step(self, action):
           err_msg = f"{action!r} ({type(action)}) invalid"
           assert self.action_space.contains(action), err_msg
           assert self.state is not None, "Call reset before using step method."
           
           #++++++++++++++++++++++++++++++++++++++++++++++++++
           #main steps:
           [initialValues,initialValues_t] = self.State2InitialValues()
           # print('initialValues_t:',initialValues_t)
           # print(self.mbs)
           qOriginal = self.mbs.systemData.GetODE2Coordinates(exu.ConfigurationType.Initial)
           q_tOriginal = self.mbs.systemData.GetODE2Coordinates_t(exu.ConfigurationType.Initial)
           initialValues[self.numberOfStates:] = qOriginal[self.numberOfStates:]
           initialValues_t[self.numberOfStates:] = q_tOriginal[self.numberOfStates:]
   
           self.mbs.systemData.SetODE2Coordinates(initialValues, exu.ConfigurationType.Initial)
           self.mbs.systemData.SetODE2Coordinates_t(initialValues_t, exu.ConfigurationType.Initial)
   
           self.MapAction2MBS(action)
           
           #this may be time consuming for larger models!
           self.IntegrateStep()
           
           done = self.Output2StateAndDone()
           if self.mbs.systemData.GetTime() > 16: #if it is too long, stop for now!
               done = True
   
           # print('state:', self.state, 'done: ', done)
           #++++++++++++++++++++++++++++++++++++++++++++++++++
           if not done:
               reward = self.getReward()
           elif self.steps_beyond_done is None:
               self.steps_beyond_done = 0
               reward = self.getReward()
           else:
               
               if self.steps_beyond_done == 0:
                   logger.warn(
                       "You are calling 'step()' even though this "
                       "environment has already returned done = True. You "
                       "should always call 'reset()' once you receive 'done = "
                       "True' -- any further steps are undefined behavior."
                   )
               self.steps_beyond_done += 1
               reward = 0.0
   
           info = {}
           terminated, truncated = done, False # since stable-baselines3 > 1.8.0 implementations terminated and truncated 
           if useOldGym:
               return np.array(self.state, dtype=np.float32), reward, terminated, info
           else:
               return np.array(self.state, dtype=np.float32), reward, terminated, truncated, info
   
   
   
   
   # sys.exit()
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++
   if __name__ == '__main__': #this is only executed when file is direct called in Python
       import time
           
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++
       #use some learning algorithm:
       #pip install stable_baselines3
       from stable_baselines3 import A2C, SAC
       
       
       # here the model is loaded (either for vectorized or scalar environmentÂ´using SAC or A2C).     
       def GetModel(myEnv, modelType='SAC'): 
           if modelType=='SAC': 
               model = SAC('MlpPolicy',
                      env=myEnv,
                      #learning_rate=8e-4,
                      device='cpu', #usually cpu is faster for this size of networks
                      #batch_size=128,
                      verbose=1)
           elif modelType == 'A2C': 
               model = A2C('MlpPolicy', 
                       myEnv, 
                       device='cpu',
                       #n_steps=5,
                       # policy_kwargs = dict(activation_fn=torch.nn.ReLU,
                       #  net_arch=dict(pi=[8]*2, vf=[8]*2)),
                       verbose=1)
           else: 
               print('Please specify the modelType.')
               raise ValueError
   
           return model
   
       # sys.exit()
       #create model and do reinforcement learning
       modelType='A2C'
       modelName = 'openAIgymDDrobot_'+modelType
       if True: #'scalar' environment:
           env = RobotEnv()
           #check if model runs:
           #env.SetSolver(exu.DynamicSolverType.ExplicitMidpoint)
           #env.SetSolver(exu.DynamicSolverType.RK44) #very acurate
           # env.TestModel(numberOfSteps=2000, seed=42, sleepTime=0.02*0, useRenderer=True)
           model = GetModel(env, modelType=modelType)
           env.useRenderer = True
           # env.render()
           # SC.renderer.Start()
   
           ts = -time.time()
           model.learn(total_timesteps=200000) 
           
           print('*** learning time total =',ts+time.time(),'***')
       
           #save learned model
           
           model.save("solution/" + modelName)
       else:
           import torch #stable-baselines3 is based on pytorch
           n_cores= max(1,int(os.cpu_count()/2)) #n_cores should be number of real cores (not threads)
           #n_cores = 8 #vecEnv can handle number of threads, while torch should rather use real cores
           #torch.set_num_threads(n_cores) #seems to be ideal to match the size of subprocVecEnv
           torch.set_num_threads(n_cores) #seems to be ideal to match the size of subprocVecEnv
           
           print('using',n_cores,'cores')
   
           from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
           vecEnv = SubprocVecEnv([RobotEnv for i in range(n_cores)])
           
       
           #main learning task;  training of double pendulum: with 20 cores 800 000 steps take in the continous case approximatly 18 minutes (SAC), discrete (A2C) takes 2 minutes. 
           model = GetModel(vecEnv, modelType=modelType)
   
           ts = -time.time()
           print('start learning of agent with {}'.format(str(model.policy).split('(')[0]))
           # model.learn(total_timesteps=50000) 
           model.learn(total_timesteps=int(500_000),log_interval=500)
           print('*** learning time total =',ts+time.time(),'***')
       
           #save learned model
           model.save("solution/" + modelName)
   
       if False: #set True to visualize results
           #%%++++++++++++++++++++++++++++++++++++++++++++++++++
           #only load and test
           if False: 
               modelName = 'openAIgymDDrobot_A2C_16M'
               modelType='A2C'
               if modelType == 'SAC':
                   model = SAC.load("solution/" + modelName)
               else: 
                   model = A2C.load("solution/" + modelName)
           
           env = RobotEnv() #larger threshold for testing
           solutionFile='solution/learningCoordinates.txt'
           env.TestModel(numberOfSteps=800, seed=3, model=model, solutionFileName=solutionFile, 
                         stopIfDone=False, useRenderer=False, sleepTime=0) #just compute solution file
   
           #++++++++++++++++++++++++++++++++++++++++++++++
           #visualize (and make animations) in exudyn:
           from exudyn.interactive import SolutionViewer
           env.SC.visualizationSettings.general.autoFitScene = False
           solution = LoadSolutionFile(solutionFile)
           SolutionViewer(env.mbs, solution, timeout = 0.01, rowIncrement=2) #loads solution file via name stored in mbs
   
   
   


