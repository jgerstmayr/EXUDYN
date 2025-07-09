
.. _examples-openvrengine:

***************
openVRengine.py
***************

You can view and download this file on Github: `openVRengine.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/openVRengine.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  test creating piston engine with variable number of pistons and piston angles;
   #           possibility to interact with openVR
   #
   # Author:   Johannes Gerstmayr
   # Date:     2023-01-17
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from math import sin, cos, asin, acos, pi, exp, log, tan, atan, radians
   from exudyn.interactive import InteractiveDialog
   
   
   omegaDrive = 4*pi*0.5
   tEnd = 3600
   nodeType = exu.NodeType.RotationEulerParameters
   fixedSpeed = False #if false, the speed is given only for first 1 second
   
   # nodeType = exu.NodeType.RotationRxyz
   #nodeType = exu.NodeType.RotationRotationVector
   
   # import matplotlib.pyplot as plt
   # plt.close('all')
   zOffAdd = -0.5
   
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
   
           #self.inertiaCrank.com = [0,0,0]
           # print('crank COM=',np.array(self.inertiaCrank.com).round(8))
           # print('inertiaCrank=',self.inertiaCrank)
           # print('inertiaConrod=',self.inertiaConrod)
           # print('inertiaPiston=',self.inertiaPiston)
           
       def Length(self):
           return self.pistonDistance*len(self.crankAngles) + self.crankBearingWidth
   
       def MaxDimX(self):
           return self.crankArmLength + self.conrodLength + self.pistonLength
   
   def ComputeSliderCrank(angleCrank, anglePiston, l1, l2):
       phi1 = angleCrank-anglePiston
       h = l1*sin(phi1) #height of crank-conrod bearing
       phi2 = asin(h/l2) #angle of conrod in 2D slider-crank, corotated with piston rotation
       angleConrod = anglePiston-phi2
       Acr = RotationMatrixZ(angleConrod)
       dp = l1*cos(phi1) + l2*cos(phi2) #distance of piston from crank rotation axis
       return [phi1,phi2, angleConrod, Acr, dp]
   
   
   #this function (re-)creates gear geometry
   def CreateEngine(P):
   
       colorCrank = graphics.color.grey
       colorConrod = graphics.color.dodgerblue
       colorPiston = graphics.color.brown[0:3]+[0.5]
       showJoints = True
       
       gravity = [0,-9.81*0,0]
       eL = P.Length()
       oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,zOffAdd], visualization=VObjectGround(graphicsData= [])))
       nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,zOffAdd]))
   
       # gEngine = [graphics.Brick(centerPoint=[0,0,0], size=[P.MaxDimX()*2, P.MaxDimX(), eL*1.2], 
       #                                       color=[0.6,0.6,0.6,0.1], addEdges=True, 
       #                                       edgeColor = [0.8,0.8,0.8,0.3], addFaces=False)]
   
       gEngine = [] #no block
       dictEngine = mbs.CreateRigidBody(
                     inertia=InertiaCuboid(1000, sideLengths=[1, 1, 1]),  # dummy engine inertia
                     nodeType=nodeType,
                     referencePosition=[0, 0, zOffAdd],
                     graphicsDataList=gEngine,
                     returnDict=True,
                     show=False)
   
       [nEngine, oEngine] = [dictEngine['nodeNumber'], dictEngine['bodyNumber']]
   
       mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
       mEngine = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oEngine))
       sEngineForce = 0
       oEngineJoint = 0
       sEngineTorque  = 0
       oEngineJoint = mbs.AddObject(GenericJoint(markerNumbers=[mEngine, mGround], constrainedAxes=[1,1,1, 1,1,1],
                                       visualization=VGenericJoint(show=False)))
       sEngineForce = mbs.AddSensor(SensorObject(objectNumber=oEngineJoint, storeInternal=True,
                                                 outputVariableType=exu.OutputVariableType.ForceLocal))
       sEngineTorque = mbs.AddSensor(SensorObject(objectNumber=oEngineJoint, storeInternal=True,
                                                 outputVariableType=exu.OutputVariableType.TorqueLocal))
       
       bConrodList = []
       bPistonList = []
       gCrank = []
       for cnt, angleCrank in enumerate(P.crankAngles):
           anglePiston = P.pistonAngles[cnt]
           Ac = RotationMatrixZ(angleCrank)
           Ap = RotationMatrixZ(anglePiston)
           [phi1,phi2, angleConrod, Acr, dp] = ComputeSliderCrank(angleCrank, anglePiston, P.crankArmLength, P.conrodLength)
           
           zOff = -0.5*eL + cnt*P.pistonDistance + zOffAdd
           #zOff = 0
           #crank bearing
           zAdd = 0
           if cnt>0: zAdd = P.crankArmWidth
           gCrank += [graphics.Cylinder(pAxis=[0,0,zOff-zAdd], vAxis=[0,0,P.crankBearingWidth+P.crankArmWidth+zAdd], 
                                           radius=P.crankBearingRadius, color=graphics.color.red)]
           #arm1
           arm1 = graphics.Brick([P.crankArmLength*0.5,0,zOff+P.crankArmWidth*0.5+P.crankBearingWidth], 
                                                 size=[P.crankArmLength,P.crankArmHeight,P.crankArmWidth], color=colorCrank)
           gCrank += [graphics.Move(arm1, [0,0,0], Ac)]
           #conrod bearing
           gCrank += [graphics.Cylinder(pAxis=Ac@[P.crankArmLength,0,zOff+P.crankBearingWidth+P.crankArmWidth*0], 
                                          vAxis=[0,0,P.conrodCrankCylLength+2*P.crankArmWidth], radius=P.conrodCrankCylRadius, color=colorCrank)]
   
           #arm2
           arm2 = graphics.Brick([P.crankArmLength*0.5,0,zOff+P.crankArmWidth*1.5+P.crankBearingWidth+P.conrodCrankCylLength], 
                                                 size=[P.crankArmLength,P.crankArmHeight,P.crankArmWidth],
                                                 color=colorCrank)
           gCrank += [graphics.Move(arm2, [0,0,0], Ac)]
   
           if cnt == len(P.crankAngles)-1:
               gCrank += [graphics.Cylinder(pAxis=[0,0,zOff+P.crankArmWidth+P.crankBearingWidth+P.conrodCrankCylLength], vAxis=[0,0,P.crankBearingWidth+P.crankArmWidth], 
                                               radius=P.crankBearingRadius, color=graphics.color.red)]
   
           #++++++++++++++++++++++++++++++++++++++            
           #conrod
           gConrod = [ graphics.RigidLink(p0=[-0.5*P.conrodLength, 0, 0], p1=[0.5*P.conrodLength,0,0], axis0= [0,0,1], axis1= [0,0,1], 
                                              radius= [P.conrodRadius]*2, 
                                              thickness= P.conrodHeight, width=[P.conrodWidth]*2, color= colorConrod, nTiles= 16)]
   
           bConrod = mbs.CreateRigidBody(
                           inertia=P.inertiaConrod,
                           nodeType=nodeType,
                           referencePosition=Ac @ [P.crankArmLength, 0, 0] + Acr @ [0.5 * P.conrodLength, 0,
                                             zOff + P.crankArmWidth + P.crankBearingWidth + 0.5 * P.conrodCrankCylLength],
                           referenceRotationMatrix=Acr,
                           gravity=gravity,
                           graphicsDataList=gConrod)
           bConrodList += [bConrod]
           
           #++++++++++++++++++++++++++++++++++++++            
           #gPiston setup
           gPiston = [graphics.Cylinder(pAxis=[-P.conrodRadius*0.5, 0, 0],
                                        vAxis=[P.pistonLength, 0, 0], radius=P.pistonRadius, color=colorPiston)]
           
           bPiston = mbs.CreateRigidBody(
                           inertia=P.inertiaPiston,
                           nodeType=nodeType,
                           referencePosition=Ap @ [dp, 0,
                                             zOff + P.crankArmWidth + P.crankBearingWidth + 0.5 * P.conrodCrankCylLength],
                           referenceRotationMatrix=Ap,
                           gravity=gravity,
                           graphicsDataList=gPiston)
           bPistonList += [bPiston]
           
       dictCrank = mbs.CreateRigidBody(
                       inertia=P.inertiaCrank,
                       nodeType=nodeType,
                       referencePosition=[0, 0, 0],
                       gravity=gravity,
                       graphicsDataList=gCrank,
                       returnDict=True)
       [nCrank, bCrank] = [dictCrank['nodeNumber'], dictCrank['bodyNumber']]
   
       sCrankAngVel = mbs.AddSensor(SensorNode(nodeNumber=nCrank, storeInternal=True,
                                                 outputVariableType=exu.OutputVariableType.AngularVelocity))
   
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #JOINTS:
       mbs.CreateRevoluteJoint(bodyNumbers=[oEngine, bCrank], position=[0,0,-0.5*eL], axis=[0,0,1], 
                               axisRadius=P.crankBearingRadius*1.2, axisLength=P.crankBearingWidth*0.8)
       # [oJointCrank, mBody0Crank, mBody1Crank] = AddRevolute*Joint(mbs, oEngine, bCrank, point=[0,0,-0.5*eL], axis=[0,0,1], showJoint=showJoints, 
       #                                             axisRadius=P.crankBearingRadius*1.2, axisLength=P.crankBearingWidth*0.8)
       
   
       for cnt, angleCrank in enumerate(P.crankAngles):
           anglePiston = P.pistonAngles[cnt]
           Ac = RotationMatrixZ(angleCrank)
           Ap = RotationMatrixZ(anglePiston)
           [phi1,phi2, angleConrod, Acr, dp] = ComputeSliderCrank(angleCrank, anglePiston, P.crankArmLength, P.conrodLength)
   
           zOff = -0.5*eL + cnt*P.pistonDistance
           #zOff = 0
   
           mbs.CreateRevoluteJoint(bodyNumbers=[bCrank, bConrodList[cnt]], 
                                   position=Ac@[P.crankArmLength,0,zOff + P.crankBearingWidth+P.crankArmWidth+0.5*P.conrodCrankCylLength], 
                                   axis=[0,0,1], 
                                   axisRadius=P.crankBearingRadius*1.3, axisLength=P.crankBearingWidth*0.8)
           # [oJointCC, mBody0CC, mBody1CC] = AddRevoluteJoint(mbs, bCrank, bConrodList[cnt], 
           #                                                   point=Ac@[P.crankArmLength,0,zOff + P.crankBearingWidth+P.crankArmWidth+0.5*P.conrodCrankCylLength], 
           #                                                   axis=[0,0,1], showJoint=showJoints, 
           #                                                   axisRadius=P.crankBearingRadius*1.3, axisLength=P.crankBearingWidth*0.8)
   
           #pPiston = A@[P.crankArmLength+P.conrodLength,0,zOff + P.crankBearingWidth+P.crankArmWidth+0.5*P.conrodCrankCylLength]
           pPiston = Ap@[dp,0,zOff + P.crankBearingWidth+P.crankArmWidth+0.5*P.conrodCrankCylLength]
           mbs.CreateRevoluteJoint(bodyNumbers=[bConrodList[cnt], bPistonList[cnt]], 
                                   position=pPiston,
                                   axis=[0,0,1], 
                                   axisRadius=P.crankBearingRadius*1.3, axisLength=P.crankBearingWidth*0.8)
           # [oJointCP, mBody0CP, mBody1CP] = AddRevoluteJoint(mbs, bConrodList[cnt], bPistonList[cnt], 
           #                                                   point=pPiston, 
           #                                                   axis=[0,0,1], showJoint=showJoints, 
           #                                                   axisRadius=P.crankBearingRadius*1.3, axisLength=P.crankBearingWidth*0.8)
   
           # AddPrismaticJoint(mbs, oEngine, bPistonList[cnt], 
           #                                                 point=pPiston, 
           #                                                 axis=A@[1,0,0], showJoint=showJoints, 
           #                                                 axisRadius=P.crankBearingRadius*1.3, axisLength=P.crankBearingWidth*0.8)
           mEngine = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oEngine, localPosition=pPiston))
           mPiston = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bPistonList[cnt], localPosition=[0,0,0]))
           mbs.AddObject(GenericJoint(markerNumbers=[mPiston, mEngine], constrainedAxes=[0,1,0, 0,0,1],
                                      # rotationMarker0=A.T,
                                      rotationMarker1=Ap,
                                      visualization=VGenericJoint(show=False, axesRadius=P.conrodRadius*1.4,axesLength=0.05)))
   
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #DRIVE:
       def UFoffset(mbs, t, itemNumber, lOffset):
           return 0
       
       def UFoffset_t(mbs, t, itemNumber, lOffset): #time derivative of UFoffset
           return SmoothStep(t, 0, 0.5, 0, omegaDrive)
           
       mCrankRotation = mbs.AddMarker(MarkerNodeRotationCoordinate(nodeNumber=nCrank, rotationCoordinate=2))
       mNodeEngine = mbs.AddMarker(MarkerNodeRotationCoordinate(nodeNumber=nEngine, rotationCoordinate=2))
       oRotationConstraint = mbs.AddObject(CoordinateConstraint(markerNumbers=[mNodeEngine, mCrankRotation], velocityLevel=True, 
                                           offsetUserFunction=UFoffset,
                                           offsetUserFunction_t=UFoffset_t,
                                           visualization=VCoordinateConstraint(show=False)))
   
       return [oEngine, oEngineJoint, sEngineForce, sEngineTorque, sCrankAngVel, oRotationConstraint, nCrank, bCrank]
   
   engines = []
   engines+=[EngineParameters([0])]                                           #R1
   engines+=[EngineParameters([0,180])]                                       #R2
   engines+=[EngineParameters([0,180,180,0])]                                 #R4 straight-four engine, Reihen-4-Zylinder
   engines+=[EngineParameters([0,90,270,180])]                                #R4 in different configuration
   engines+=[EngineParameters([0,180,180,0],[0,180,180,0])]                   #Boxer 4-piston perfect mass balancing
   
   engines+=[EngineParameters([0,120,240])]                                   #R3
   engines+=[EngineParameters(list(np.arange(0,5)*144))]                      #R5
   engines+=[EngineParameters([0,120,240,240,120,0])]                         #R6
   engines+=[EngineParameters([0,0,120,120,240,240],[-30,30,-30,30,-30,30])]  #V6
   engines+=[EngineParameters([0,0,120,120,240,240,240,240,120,120,0,0],[-30,30,-30,30,-30,30,30,-30,30,-30,30,-30])] #V12
   
   engines+=[EngineParameters([0,90,180,270,270,180,90,360])]                  #R8
   engines+=[EngineParameters([0,0,90,90,270,270,180,180], [-45,45,-45,45, 45,-45,45,-45])] #V8
   
   # n=12
   # a=list(np.arange(0,n)*30)
   # b=list(np.arange(n-1,-1,-1)*30)
   # #engines+=[EngineParameters(a+a,b+b)
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #
   engines=[EngineParameters([0,90,270,180], [90]*4)]
   #engines=[EngineParameters([0,0,120,120,240,240,240,240,120,120,0,0],[60,120,60,120,60,120,120,60,120,60,120,60])] #V12
   
   for engine in engines:
       
       SC = exu.SystemContainer()
       mbs = SC.AddSystem()
       
       [oEngine, oEngineJoint, sEngineForce, sEngineTorque, sCrankAngVel, oRotationConstraint, 
         nCrank, bCrank] = CreateEngine(engine)
   
       d = 2.4     #box size
       h = 0.5*d #box half size    
       w = d
       gDataList = []
       gDataList += [graphics.CheckerBoard(point=[0,0,-h], normal=[0,0,1], size=2*d, size2=d, nTiles=12*2, nTiles2=12, color=graphics.color.grey)]
       gDataList += [graphics.CheckerBoard(point=[-w,0,0], normal=[ 1,0,0], size=d, nTiles=12, color=graphics.color.lightgrey)]
       gDataList += [graphics.CheckerBoard(point=[ w,0,0], normal=[-1,0,0], size=d, nTiles=12, color=graphics.color.lightgrey)]
       gDataList += [graphics.CheckerBoard(point=[0,-h,0], normal=[0,-1,0], size=2*d, size2=d, nTiles=12*2, nTiles2=12, color=graphics.color.dodgerblue)]
       gDataList += [graphics.CheckerBoard(point=[0, h,0], normal=[0, 1,0], size=2*d, size2=d, nTiles=1, color=[0.8,0.8,1,1])]#, alternatingColor=[0.8,0.8,1,1])]
       # gDataList += [graphics.CheckerBoard(point=[0, 0,h], normal=[0, 0,-1], size=d, nTiles=1, color=[0.8,0.8,0.8,0.9])]
       
       oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                      visualization=VObjectGround(graphicsData=gDataList)))
   
       
       def PreStepUF(mbs, t):
           u = mbs.systemData.GetODE2Coordinates()
           
           if not fixedSpeed and t >= 1: #at this point, the mechanism runs freely
               mbs.SetObjectParameter(oRotationConstraint, 'activeConnector', False)
       
           #mbs.systemData.SetODE2Coordinates(u)
           return True
       
       mbs.SetPreStepUserFunction(PreStepUF)
       
       
       mbs.Assemble()
       
       stepSize = 0.002
       simulationSettings = exu.SimulationSettings() #takes currently set values or default values
       
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
       simulationSettings.timeIntegration.endTime = tEnd
       # simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8*0.01
       # simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10*0.01
       simulationSettings.timeIntegration.verboseMode = 1
       
       # simulationSettings.timeIntegration.simulateInRealtime = True
       
       simulationSettings.solutionSettings.solutionWritePeriod=0.01
       simulationSettings.solutionSettings.sensorsWritePeriod = stepSize*10
       simulationSettings.solutionSettings.writeSolutionToFile = False
       #simulationSettings.solutionSettings.writeInitialValues = False #otherwise values are duplicated
       #simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/coordinatesSolution.txt'
       
       simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = False
       
       simulationSettings.timeIntegration.generalizedAlpha.lieGroupAddTangentOperator = False
       #simulationSettings.displayStatistics = True
       # simulationSettings.displayComputationTime = True
       simulationSettings.linearSolverType=exu.LinearSolverType.EigenSparse
       
       #SC.visualizationSettings.nodes.defaultSize = 0.05
       
       simulationSettings.solutionSettings.solutionInformation = "Engine"
       
       SC.visualizationSettings.general.graphicsUpdateInterval = 0.01
       #SC.visualizationSettings.general.drawWorldBasis = True
       #SC.visualizationSettings.general.worldBasisSize = 0.1
       
       SC.visualizationSettings.markers.show = False
       SC.visualizationSettings.loads.show = False
       SC.visualizationSettings.nodes.show = False
       SC.visualizationSettings.connectors.show = False
       
       SC.visualizationSettings.openGL.multiSampling = 4
       SC.visualizationSettings.openGL.shadow = 0.3 #set to 0, if your graphics card cannot handle this!
       SC.visualizationSettings.openGL.lineWidth = 3
       SC.visualizationSettings.openGL.light0position = [0.25,1,3,0]
   
       #++++++++++++++++++++++++++++++++
       #openVR:
       SC.visualizationSettings.general.drawCoordinateSystem = False
       #good for openVR
       SC.visualizationSettings.general.graphicsUpdateInterval = 0.005 #small enough to get large enough fps
       simulationSettings.timeIntegration.simulateInRealtime = True
   
       useOpenVR = False #set this true for openVR to run!!!
       SC.visualizationSettings.window.renderWindowSize=[1176, 1320] # this needs to fit to your VR HMD (Head Mounted Display) settings (will show in console when openVR is started and openVR.logLevel is large enough!)
       if useOpenVR:
           SC.visualizationSettings.openGL.initialZoom = 1# 0.4*20 #0.4*max scene size
           #SC.visualizationSettings.openGL.initialCenterPoint = [0,0,2]
           SC.visualizationSettings.general.autoFitScene = False
           SC.visualizationSettings.window.limitWindowToScreenSize = False #this allows a larger window size than your monitor can display in case!
           SC.visualizationSettings.window.startupTimeout = 100000 #if steam / VRidge, etc. not found
           SC.visualizationSettings.interactive.openVR.enable = True
           SC.visualizationSettings.interactive.lockModelView = True #lock rotation/translation/zoom of model
           SC.visualizationSettings.interactive.openVR.logLevel = 3
           SC.visualizationSettings.interactive.openVR.actionManifestFileName = "C:/DATA/cpp/DocumentationAndInformation/openVR/hellovr_actions.json"
   
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       
           
       SC.visualizationSettings.general.autoFitScene = False #use loaded render state
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
       cws = SC.renderer.GetState()['currentWindowSize']
       print('window size=', cws, '(check that this is according to needs of Head Mounted Display)')
       # if 'renderState' in exu.sys:
       #     SC.renderer.SetState(exu.sys[ 'renderState' ])
       
       mbs.SolveDynamic(simulationSettings)
       
       SC.renderer.Stop() #safely close rendering window!
       


