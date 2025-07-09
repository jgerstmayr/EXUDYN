
.. _examples-pistonengine:

***************
pistonEngine.py
***************

You can view and download this file on Github: `pistonEngine.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/pistonEngine.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Create piston engine with variable number of pistons, crank and piston angles;
   #           Showing unbalance and harmonics of unbalance
   #
   # Model:    Generic piston engine
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-12-20
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   # *clean example*
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   ## import basic libaries
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from math import sin, cos, asin, acos, pi, exp, log, tan, atan, radians
   
   
   ## some simulation parameters for
   createFigures = False
   useLogY = False
   showSolutionViewer = True
   omegaDrive = 2*pi #angular velocity
   tEnd = 2.5+40 #simulation time
   nodeType = exu.NodeType.RotationEulerParameters
   fixedSpeed = True #if false, the speed is given only for first 1 second
   
   
   ## a class to store engine parameters and with geometric functions for piston engine
   class EngineParameters:
       def __init__(self, crankAnglesDegrees=[], pistonAnglesDegrees=[]):
           #parameters in m, s, kg, rad, ...
           self.crankAnglesDegrees = crankAnglesDegrees
           if pistonAnglesDegrees == []:
               self.pistonAnglesDegrees = list(0*np.array(crankAnglesDegrees))
           else:
               self.pistonAnglesDegrees = pistonAnglesDegrees
   
           crankAngles = pi/180*np.array(crankAnglesDegrees)
           self.crankAngles = list(crankAngles)
   
           pistonAngles = pi/180*np.array(self.pistonAnglesDegrees)
           self.pistonAngles = list(pistonAngles)
   
           densitySteel = 7850
           #kinematics & inertia & drawing 
           fZ = 1#0.2
           self.pistonDistance = 0.08
           self.pistonMass = 0.5
           self.pistonLength = 0.05
           self.pistonRadius = 0.02
   
           self.conrodLength = 0.1 #X
           self.conrodHeight = 0.02*fZ#Y
           self.conrodWidth = 0.02*fZ #Z
           self.conrodRadius = 0.012*fZ #Z
   
           self.crankArmLength = 0.04      #X
           self.crankArmHeight = 0.016     #Y
           self.crankArmWidth = 0.01*fZ       #Z width of arm
           self.crankBearingWidth = 0.012*fZ   #Z
           self.crankBearingRadius = 0.01
   
           self.conrodCrankCylLength = 0.024*fZ  #Z; length of cylinder (bearing conrod-crank)
           self.conrodCrankCylRadius = 0.008 #radius of cylinder (bearing conrod-crank)
   
           self.pistonDistance = self.crankBearingWidth + 2*self.crankArmWidth + self.conrodCrankCylLength #Z distance
   
           self.inertiaConrod = InertiaCuboid(densitySteel, sideLengths=[self.conrodLength, self.conrodHeight, self.conrodWidth])
           
           eL = self.Length()
           #last bearing:
           densitySteel2 = densitySteel
           self.inertiaCrank = InertiaCylinder(densitySteel2, self.crankBearingWidth, self.crankBearingRadius, axis=2).Translated([0,0,0.5*eL-0.5*self.crankBearingWidth])
   
       
   
           for cnt, angle in enumerate(self.crankAngles):
               A = RotationMatrixZ(angle)
               zOff = -0.5*eL + cnt*self.pistonDistance
               arm = InertiaCuboid(densitySteel2, sideLengths=[self.crankArmLength, self.crankArmHeight, self.crankArmWidth])
               cylCrank = InertiaCylinder(densitySteel2, self.crankBearingWidth, self.crankBearingRadius, axis=2)
               cylConrod = InertiaCylinder(densitySteel2, self.conrodCrankCylLength, self.conrodCrankCylRadius, axis=2)
               #add inertias:
               self.inertiaCrank += cylCrank.Translated([0,0,zOff+self.crankBearingWidth*0.5])
               self.inertiaCrank += arm.Rotated(A).Translated(A@[self.crankArmLength*0.5,0,zOff+self.crankBearingWidth+self.crankArmWidth*0.5])
               self.inertiaCrank += cylConrod.Translated(A@[self.crankArmLength,0,zOff+self.crankBearingWidth+self.crankArmWidth+self.conrodCrankCylLength*0.5])
               self.inertiaCrank += arm.Rotated(A).Translated(A@[self.crankArmLength*0.5,0,zOff+self.crankBearingWidth+self.crankArmWidth*1.5+self.conrodCrankCylLength])
   
           # self.inertiaCrank = InertiaCylinder(1e-8*densitySteel, length=self.pistonLength, 
           #                                      outerRadius=self.pistonRadius, innerRadius=0.5*self.pistonRadius, axis=2)
   
           self.inertiaPiston = InertiaCylinder(densitySteel, length=self.pistonLength, 
                                                outerRadius=self.pistonRadius, innerRadius=0.5*self.pistonRadius, axis=0)
   
       def Length(self):
           return self.pistonDistance*len(self.crankAngles) + self.crankBearingWidth
   
       def MaxDimX(self):
           return self.crankArmLength + self.conrodLength + self.pistonLength
   
   ## compute essential geometrical parameters for slider-crank with crank angle, piston angle, crank length l1 and conrod length l2
   def ComputeSliderCrank(angleCrank, anglePiston, l1, l2):
       phi1 = angleCrank-anglePiston
       h = l1*sin(phi1) #height of crank-conrod bearing
       phi2 = asin(h/l2) #angle of conrod in 2D slider-crank, corotated with piston rotation
       angleConrod = anglePiston-phi2
       Acr = RotationMatrixZ(angleConrod)
       dp = l1*cos(phi1) + l2*cos(phi2) #distance of piston from crank rotation axis
       return [phi1,phi2, angleConrod, Acr, dp]
   
   
   ## function to create multibody system for certain crank and piston configuration
   def CreateEngine(P):
   
       colorCrank = graphics.color.grey
       colorConrod = graphics.color.dodgerblue
       colorPiston = graphics.color.brown[0:3]+[0.5]
       showJoints = True
   
       ## set up ground object    
       gravity = [0,-9.81*0,0]
       eL = P.Length()
       oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [])))
       nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
   
       gEngine = [graphics.Brick(centerPoint=[0,0,0], size=[P.MaxDimX()*2, P.MaxDimX(), eL*1.2], 
                                             color=[0.6,0.6,0.6,0.1], addEdges=True, 
                                             edgeColor = [0.8,0.8,0.8,0.3], addFaces=False)]
       
       ## create rigid body for housing; this body allows to measure support forces and torques
       oEngine = mbs.CreateRigidBody(referencePosition=[0,0,0],
                                                inertia=InertiaCuboid(1000, sideLengths=[1,1,1]), #dummy engine inertia
                                                nodeType = nodeType,
                                                graphicsDataList = gEngine
                                                )
       nEngine = mbs.GetObjectParameter(oEngine, 'nodeNumber')
       
       ## create joint between engine and ground to measure forces
       oEngineJoint = mbs.CreateGenericJoint(bodyNumbers=[oEngine, oGround],
                                             position=[0,0,0],
                                             constrainedAxes=[1,1,1, 1,1,1],
                                             show=False)
   
       ## add sensors for 
       sEngineForce = mbs.AddSensor(SensorObject(objectNumber=oEngineJoint, storeInternal=True,
                                                 outputVariableType=exu.OutputVariableType.ForceLocal))
       sEngineTorque = mbs.AddSensor(SensorObject(objectNumber=oEngineJoint, storeInternal=True,
                                                 outputVariableType=exu.OutputVariableType.TorqueLocal))
       
       ## loop over all slider-cranks in n-piston engine
       bConrodList = []
       bPistonList = []
       gCrank = []
       for cnt, angleCrank in enumerate(P.crankAngles):
           anglePiston = P.pistonAngles[cnt]
           Ac = RotationMatrixZ(angleCrank)
           Ap = RotationMatrixZ(anglePiston)
           [phi1,phi2, angleConrod, Acr, dp] = ComputeSliderCrank(angleCrank, anglePiston, P.crankArmLength, P.conrodLength)
           
           zOff = -0.5*eL + cnt*P.pistonDistance
           zAdd = 0
           if cnt>0: zAdd = P.crankArmWidth
           
           ### create graphics for crank part
           gCrank += [graphics.Cylinder(pAxis=[0,0,zOff-zAdd], vAxis=[0,0,P.crankBearingWidth+P.crankArmWidth+zAdd], 
                                           radius=P.crankBearingRadius, color=graphics.color.red)]
           ### create graphics for crank arm1
           arm1 = graphics.Brick([P.crankArmLength*0.5,0,zOff+P.crankArmWidth*0.5+P.crankBearingWidth], 
                                                 size=[P.crankArmLength,P.crankArmHeight,P.crankArmWidth], color=colorCrank)
           gCrank += [graphics.Move(arm1, [0,0,0], Ac)]
           
           ### create graphics for conrod bearing
           gCrank += [graphics.Cylinder(pAxis=Ac@[P.crankArmLength,0,zOff+P.crankBearingWidth+P.crankArmWidth*0], 
                                          vAxis=[0,0,P.conrodCrankCylLength+2*P.crankArmWidth], radius=P.conrodCrankCylRadius, color=colorCrank)]
   
           ### create graphics for crank arm2
           arm2 = graphics.Brick([P.crankArmLength*0.5,0,zOff+P.crankArmWidth*1.5+P.crankBearingWidth+P.conrodCrankCylLength], 
                                                 size=[P.crankArmLength,P.crankArmHeight,P.crankArmWidth],
                                                 color=colorCrank)
           gCrank += [graphics.Move(arm2, [0,0,0], Ac)]
   
           if cnt == len(P.crankAngles)-1:
               gCrank += [graphics.Cylinder(pAxis=[0,0,zOff+P.crankArmWidth+P.crankBearingWidth+P.conrodCrankCylLength], vAxis=[0,0,P.crankBearingWidth+P.crankArmWidth], 
                                               radius=P.crankBearingRadius, color=graphics.color.red)]
   
           #++++++++++++++++++++++++++++++++++++++            
           ### create graphics for conrod
           gConrod = [ graphics.RigidLink(p0=[-0.5*P.conrodLength, 0, 0], p1=[0.5*P.conrodLength,0,0], axis0= [0,0,1], axis1= [0,0,1], 
                                              radius= [P.conrodRadius]*2, 
                                              thickness= P.conrodHeight, width=[P.conrodWidth]*2, color= colorConrod, nTiles= 16)]
   
           ### create rigid body for conrod
           bConrod = mbs.CreateRigidBody(inertia = P.inertiaConrod,
                                         nodeType = nodeType,
                                         referencePosition=Ac@[P.crankArmLength,0,0] + Acr@[0.5*P.conrodLength,0,
                                                               zOff+P.crankArmWidth+P.crankBearingWidth+0.5*P.conrodCrankCylLength],
                                         referenceRotationMatrix=Acr,
                                         gravity = gravity,
                                         graphicsDataList = gConrod
                                         )
           bConrodList += [bConrod]
           #++++++++++++++++++++++++++++++++++++++            
           ### create graphics for piston
           gPiston = [graphics.Cylinder(pAxis=[-P.conrodRadius*0.5,0,0],
                                            vAxis=[P.pistonLength,0,0], radius=P.pistonRadius, color=colorPiston)]
           ### create rigid body for piston
           bPiston = mbs.CreateRigidBody(inertia = P.inertiaPiston,
                                         nodeType = nodeType,
                                         referencePosition=Ap@[dp,0,
                                                     zOff+P.crankArmWidth+P.crankBearingWidth+0.5*P.conrodCrankCylLength],
                                         referenceRotationMatrix=Ap,
                                         gravity = gravity,
                                         graphicsDataList = gPiston
                                         )
           bPistonList += [bPiston]
       
       ## create rigid body for crankshaft
       bCrank = mbs.CreateRigidBody(inertia = P.inertiaCrank,
                                    nodeType = nodeType,
                                    referencePosition=[0,0,0],
                                    gravity = gravity,
                                    graphicsDataList = gCrank
                                    )
       nCrank = mbs.GetObjectParameter(bCrank, 'nodeNumber')
   
       ## add sensor for crank angular velocity
       sCrankAngVel = mbs.AddSensor(SensorNode(nodeNumber=nCrank, storeInternal=True,
                                                 outputVariableType=exu.OutputVariableType.AngularVelocity))
   
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       ## create revulute joint between engine and crankshaft
       oJointCrank = mbs.CreateRevoluteJoint(bodyNumbers=[oEngine, bCrank],
                                             position=[0,0,-0.5*eL], 
                                             axis=[0,0,1], 
                                             show=showJoints, 
                                             axisRadius=P.crankBearingRadius*1.2, 
                                             axisLength=P.crankBearingWidth*0.8)
   
       ## loop over all slider cranks to create joints
       for cnt, angleCrank in enumerate(P.crankAngles):
           anglePiston = P.pistonAngles[cnt]
           Ac = RotationMatrixZ(angleCrank)
           Ap = RotationMatrixZ(anglePiston)
           [phi1,phi2, angleConrod, Acr, dp] = ComputeSliderCrank(angleCrank, anglePiston, P.crankArmLength, P.conrodLength)
   
           zOff = -0.5*eL + cnt*P.pistonDistance
           #zOff = 0
   
           ### create revolute joint between crankshaft and conrod
           oJointCC = mbs.CreateRevoluteJoint(bodyNumbers=[bCrank, bConrodList[cnt]], 
                                              position=Ac@[P.crankArmLength,0,zOff + P.crankBearingWidth+P.crankArmWidth+0.5*P.conrodCrankCylLength], 
                                              axis=[0,0,1], 
                                              show = showJoints, 
                                              axisRadius=P.crankBearingRadius*1.3, 
                                              axisLength=P.crankBearingWidth*0.8)
           
           ### create revolute joint between conrod and piston
           pPiston = Ap@[dp,0,zOff + P.crankBearingWidth+P.crankArmWidth+0.5*P.conrodCrankCylLength]
           oJointCP = mbs.CreateRevoluteJoint(bodyNumbers=[bConrodList[cnt], bPistonList[cnt]], 
                                              position=pPiston, 
                                              axis=[0,0,1], 
                                              show=showJoints, 
                                              axisRadius=P.crankBearingRadius*1.3, 
                                              axisLength=P.crankBearingWidth*0.8)
   
           ### create prismatic joint between piston and engine, using a generic joint
           mbs.CreateGenericJoint(bodyNumbers=[bPistonList[cnt], oEngine], 
                                  position=[0,0,0],
                                  constrainedAxes=[0,1,0, 0,0,1],
                                  useGlobalFrame=False, 
                                  show=True, 
                                  axesRadius=P.conrodRadius*1.4, 
                                  axesLength=0.05)
   
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       ## define user function for crankshaft angle (not used, because velocity level is used):
       def UFoffset(mbs, t, itemNumber, lOffset):
           return 0
       
       ## define user function for crankshaft angular velocity:
       def UFoffset_t(mbs, t, itemNumber, lOffset): #time derivative of UFoffset
           return SmoothStep(t, 0, 0.5, 0, omegaDrive)
       
       ## create coordinate constraint for crankshaft velocity
       mCrankRotation = mbs.AddMarker(MarkerNodeRotationCoordinate(nodeNumber=nCrank, rotationCoordinate=2))
       mNodeEngine = mbs.AddMarker(MarkerNodeRotationCoordinate(nodeNumber=nEngine, rotationCoordinate=2))
       oRotationConstraint = mbs.AddObject(CoordinateConstraint(markerNumbers=[mNodeEngine, mCrankRotation], 
                                                                velocityLevel=True, 
                                           offsetUserFunction=UFoffset,
                                           offsetUserFunction_t=UFoffset_t,
                                           visualization=VCoordinateConstraint(show=False)))
   
       return [oEngine, oEngineJoint, sEngineForce, sEngineTorque, sCrankAngVel, oRotationConstraint, nCrank, bCrank]
   
   ## define engine parameters for certain case
   # engine = EngineParameters([0])                                           #R1
   # engine = EngineParameters([0,180])                                       #R2
   # engine = EngineParameters([0,180,180,0])                                 #R4 straight-four engine, Reihen-4-Zylinder
   # engine = EngineParameters([0,90,270,180])                                #R4 in different configuration
   engine = EngineParameters([0,180,180,0],[0,180,180,0])                   #Boxer 4-piston perfect mass balancing
   
   # engine = EngineParameters([0,120,240])                                   #R3
   # engine = EngineParameters(list(np.arange(0,5)*144))]                      #R5
   # engine = EngineParameters([0,120,240,240,120,0])                         #R6
   # engine = EngineParameters([0,0,120,120,240,240],[-30,30,-30,30,-30,30])  #V6
   # engine = EngineParameters([0,0,120,120,240,240,240,240,120,120,0,0],[-30,30,-30,30,-30,30,30,-30,30,-30,30,-30]) #V12
   
   # engine = EngineParameters([0,90,180,270,270,180,90,360])                  #R8
   # engine = EngineParameters([0,0,90,90,270,270,180,180], [-45,45,-45,45, 45,-45,45,-45]) #V8
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   [oEngine, oEngineJoint, sEngineForce, sEngineTorque, sCrankAngVel, oRotationConstraint, 
    nCrank, bCrank] = CreateEngine(engine)
   
   ## add prestep user function to turn off drive in case fixedSpeed=False
   def PreStepUF(mbs, t):
       u = mbs.systemData.GetODE2Coordinates()
       
       if not fixedSpeed and t >= 1: #at this point, the mechanism runs freely
           mbs.SetObjectParameter(oRotationConstraint, 'activeConnector', False)
   
       return True
   
   ## add prestep user function
   mbs.SetPreStepUserFunction(PreStepUF)
   
   ## assemble system
   mbs.Assemble()
   
   ## setup simulation parameters
   stepSize = 0.002
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.simulateInRealtime = True
   
   simulationSettings.solutionSettings.solutionWritePeriod=0.01
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.sensorsWritePeriod = stepSize
   simulationSettings.solutionSettings.writeInitialValues = False #otherwise values are duplicated
   simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/coordinatesSolution.txt'
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
   simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = False
   
   simulationSettings.timeIntegration.generalizedAlpha.lieGroupAddTangentOperator = False
   simulationSettings.linearSolverType=exu.LinearSolverType.EigenSparse
   
   simulationSettings.solutionSettings.solutionInformation = "Piston engine"
   
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.01
   SC.visualizationSettings.general.drawWorldBasis = True
   SC.visualizationSettings.general.worldBasisSize = 0.1
   
   SC.visualizationSettings.loads.show = False
   SC.visualizationSettings.nodes.show = False
   SC.visualizationSettings.connectors.show = False
   
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.openGL.lineWidth = 3
   SC.visualizationSettings.openGL.perspective = 0.5
   SC.visualizationSettings.openGL.light0position = [0.25,1,3,0]
   SC.visualizationSettings.window.renderWindowSize = [1600,1200]
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   ## start visualization and solve
   SC.visualizationSettings.general.autoFitScene = False #use loaded render state
   SC.renderer.Start()
   if 'renderState' in exu.sys:
       SC.renderer.SetState(exu.sys[ 'renderState' ])
   
   SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   SC.renderer.Stop() #safely close rendering window!
   
   
   ## import plot tools and plot some sensors
   from exudyn.plot import PlotSensor,PlotSensorDefaults
   
   PlotSensor(mbs, closeAll=True)
   PlotSensor(mbs, [sCrankAngVel], components=[2], title='crank speed', sizeInches=[2*6.4,2*4.8])
   PlotSensor(mbs, sEngineForce, components=[0,1,2], title='joint forces')
   PlotSensor(mbs, sEngineTorque, components=[0,1,2], title='joint torques')
   
   
   
   
   
   


