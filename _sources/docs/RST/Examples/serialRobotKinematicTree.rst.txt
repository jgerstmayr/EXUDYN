
.. _examples-serialrobotkinematictree:

***************************
serialRobotKinematicTree.py
***************************

You can view and download this file on Github: `serialRobotKinematicTree.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/serialRobotKinematicTree.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example of a serial robot with minimal and redundant coordinates
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-06-26
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.rigidBodyUtilities import *
   from exudyn.graphicsDataUtilities import *
   from exudyn.robotics import *
   from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration, ProfilePTP
   
   import numpy as np
   from numpy import linalg as LA
   from math import pi
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   sensorWriteToFile = False
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   compensateStaticTorques = True #static torque compensation converges slowly!
   useKinematicTree = True
   #kinematic tree and redundant mbs agrees for stdDH version up to 1e-10, with compensateStaticTorques = False
   # KT:      rotations at tEnd= 1.8464514676503092 , [0.4921990591981066, 0.2718999073958087, 0.818158053005264, -0.0030588904101585936, 0.26831938569719394, -0.0010660472359057434] 
   # red. MBS:rotations at tEnd= 1.8464514674961 ,   [ 0.49219906  0.27189991  0.81815805 -0.00305889  0.26831939 -0.00106605]
   
   mode='newDH'
   # mode='newModDH' #modified DH parameters #position agrees for all digits: 1.846440411790352
   
   jointWidth=0.1
   jointRadius=0.06
   linkWidth=0.1
   
   graphicsBaseList = [graphics.Brick([0,0,-0.15], [0.12,0.12,0.1], graphics.color.grey)]
   graphicsBaseList +=[graphics.Cylinder([0,0,0], [0.5,0,0], 0.0025, graphics.color.red)]
   graphicsBaseList +=[graphics.Cylinder([0,0,0], [0,0.5,0], 0.0025, graphics.color.green)]
   graphicsBaseList +=[graphics.Cylinder([0,0,0], [0,0,0.5], 0.0025, graphics.color.blue)]
   graphicsBaseList +=[graphics.Cylinder([0,0,-jointWidth], [0,0,jointWidth], linkWidth*0.5, graphics.colorList[0])] #belongs to first body
   
   ty = 0.03
   tz = 0.04
   zOff = -0.05
   toolSize= [0.05,0.5*ty,0.06]
   graphicsToolList = [graphics.Cylinder(pAxis=[0,0,zOff], vAxis= [0,0,tz], radius=ty*1.5, color=graphics.color.red)]
   graphicsToolList+= [graphics.Brick([0,ty,1.5*tz+zOff], toolSize, graphics.color.grey)]
   graphicsToolList+= [graphics.Brick([0,-ty,1.5*tz+zOff], toolSize, graphics.color.grey)]
   
   #control parameters, per joint:
   fc=1
   Pcontrol = np.array([40000, 40000, 40000, 100, 100, 10])
   Dcontrol = np.array([400,   400,   100,   1,   1,   0.1])
   Pcontrol = fc*Pcontrol
   Dcontrol = fc*Dcontrol
   
   
   #changed to new robot structure July 2021:
   robot = Robot(gravity=[0,0,9.81],
                 base = RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList)),
                 tool = RobotTool(HT=HTtranslate([0,0,0.1]), visualization=VRobotTool(graphicsData=graphicsToolList)),
                referenceConfiguration = []) #referenceConfiguration created with 0s automatically
   
   #modDHKK according to Khalil and Kleinfinger, 1986
   link0={'stdDH':[0,0,0,pi/2], 
          'modDHKK':[0,0,0,0], 
           'mass':20,  #not needed!
           'inertia':np.diag([1e-8,0.35,1e-8]), #w.r.t. COM! in stdDH link frame
           'COM':[0,0,0]} #in stdDH link frame
   
   link1={'stdDH':[0,0,0.4318,0],
          'modDHKK':[0.5*pi,0,0,0], 
           'mass':17.4, 
           'inertia':np.diag([0.13,0.524,0.539]), #w.r.t. COM! in stdDH link frame
           'COM':[-0.3638, 0.006, 0.2275]} #in stdDH link frame
   
   link2={'stdDH':[0,0.15,0.0203,-pi/2], 
          'modDHKK':[0,0.4318,0,0.15], 
           'mass':4.8, 
           'inertia':np.diag([0.066,0.086,0.0125]), #w.r.t. COM! in stdDH link frame
           'COM':[-0.0203,-0.0141,0.07]} #in stdDH link frame
   
   link3={'stdDH':[0,0.4318,0,pi/2], 
          'modDHKK':[-0.5*pi,0.0203,0,0.4318], 
           'mass':0.82, 
           'inertia':np.diag([0.0018,0.0013,0.0018]), #w.r.t. COM! in stdDH link frame
           'COM':[0,0.019,0]} #in stdDH link frame
   
   link4={'stdDH':[0,0,0,-pi/2], 
          'modDHKK':[0.5*pi,0,0,0], 
           'mass':0.34, 
           'inertia':np.diag([0.0003,0.0004,0.0003]), #w.r.t. COM! in stdDH link frame
           'COM':[0,0,0]} #in stdDH link frame
   
   link5={'stdDH':[0,0,0,0], 
          'modDHKK':[-0.5*pi,0,0,0], 
           'mass':0.09, 
           'inertia':np.diag([0.00015,0.00015,4e-5]), #w.r.t. COM! in stdDH link frame
           'COM':[0,0,0.032]} #in stdDH link frame
   linkList=[link0, link1, link2, link3, link4, link5]
   
   #++++++++++++++++++++++++++
   #test example for graphicsData in VRobotLink
   s0=0.08
   wj = 0.12
   rj = 0.06
   
   if mode=='newDH':
       for cnt, link in enumerate(linkList):
           robot.AddLink(RobotLink(mass=link['mass'], 
                                      COM=link['COM'], 
                                      inertia=link['inertia'], 
                                      localHT=StdDH2HT(link['stdDH']),
                                      PDcontrol=(Pcontrol[cnt], Dcontrol[cnt]),
                                      visualization=VRobotLink(linkColor=graphics.colorList[cnt])
                                      ))
   elif mode=='newModDH': #computes preHT and localHT, but ALSO converts inertia parameters from stdDH to modDHKK (NEEDED!)
       for cnt, link in enumerate(linkList): 
           [preHT, localHT] =  ModDHKK2HT(link['modDHKK'])
           stdLocalHT =  StdDH2HT(link['stdDH'])
           HT = InverseHT(stdLocalHT) @ (localHT) #from stdHT back and forward in localHT of ModDHKK
           
           rbi = RigidBodyInertia()
           rbi.SetWithCOMinertia(link['mass'], link['inertia'], link['COM'])
   
           rbi = rbi.Transformed(InverseHT(HT)) #inertia parameters need to be transformed to new modDHKK link frame
           
           robot.AddLink(RobotLink(mass=rbi.mass,
                                      COM=rbi.COM(), 
                                      inertia=rbi.InertiaCOM(),
                                      preHT = preHT,
                                      localHT=localHT,
                                      PDcontrol=(Pcontrol[cnt], Dcontrol[cnt]),
                                      visualization=VRobotLink(linkColor=graphics.colorList[cnt])
                                      #visualization=VRobotLink(showCOM=True, showMBSjoint=False, graphicsData=gLinkList[cnt])
                                      ))
   else:
       raise ValueError('invalid mode!')
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #configurations and trajectory
   q0 = [0,0,0,0,0,0] #zero angle configuration
   
   #this set of coordinates only works with TSD, not with old fashion load control:
   # q1 = [0, pi/8, pi*0.75, 0,pi/8,0] #configuration 1
   # q2 = [pi,-pi, -pi*0.5,1.5*pi,-pi*2,pi*2] #configuration 2
   # q3 = [3*pi,0,-0.25*pi,0,0,0] #zero angle configuration
   
   #this set also works with load control:
   q1 = [0, pi/8, pi*0.5, 0,pi/8,0] #configuration 1
   q2 = [0.8*pi,-0.8*pi, -pi*0.5,0.75*pi,-pi*0.4,pi*0.4] #configuration 2
   q3 = [0.5*pi,0,-0.25*pi,0,0,0] #zero angle configuration
   
   #trajectory generated with optimal acceleration profiles:
   trajectory = Trajectory(initialCoordinates=q0, initialTime=0)
   trajectory.Add(ProfileConstantAcceleration(q3,0.25))
   trajectory.Add(ProfileConstantAcceleration(q1,0.25))
   trajectory.Add(ProfileConstantAcceleration(q2,0.25))
   trajectory.Add(ProfileConstantAcceleration(q0,0.25))
   #traj.Add(ProfilePTP([1,1],syncAccTimes=False, maxVelocities=[1,1], maxAccelerations=[5,5]))
   
   # x = traj.EvaluateCoordinate(t,0)
   
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #test robot model
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   jointList = [0]*robot.NumberOfLinks() #this list must be filled afterwards with the joint numbers in the mbs!
   
   ## user function to compute static torque compensation (slows down simulation!)
   def ComputeMBSstaticRobotTorques(robot):
       
       if not useKinematicTree:
           q=[]
           for joint in jointList:
               q += [mbs.GetObjectOutput(joint, exu.OutputVariableType.Rotation)[2]] #z-rotation
       else:
           q = mbs.GetObjectOutputBody(oKT, exu.OutputVariableType.Coordinates)
   
       HT=robot.JointHT(q)
       return robot.StaticTorques(HT)
   
   #++++++++++++++++++++++++++++++++++++++++++++++++
   #base, graphics, object and marker:
   
   objectGround = mbs.AddObject(ObjectGround(referencePosition=HT2translation(robot.GetBaseHT()), 
                                         #visualization=VObjectGround(graphicsData=graphicsBaseList)
                                             ))
   
   
   #baseMarker; could also be a moving base!
   baseMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=objectGround, localPosition=[0,0,0]))
   
   
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #build mbs robot model:
   if not useKinematicTree:
       robotDict = robot.CreateRedundantCoordinateMBS(mbs, baseMarker=baseMarker, 
                                                         createJointTorqueLoads=False,
                                                         )
           
       jointList = robotDict['jointList'] #must be stored there for the load user function
   else:
       robotDict = robot.CreateKinematicTree(mbs)
       oKT = robotDict['objectKinematicTree']
       
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #user function which is called only once per step, speeds up simulation drastically
   def PreStepUF(mbs, t):
       if compensateStaticTorques:
           staticTorques = ComputeMBSstaticRobotTorques(robot)
           #print("tau=", staticTorques)
       else:
           staticTorques = np.zeros(len(jointList))
           
       [u,v,a] = trajectory.Evaluate(t)
   
       #compute load for joint number
       if not useKinematicTree:
           for i in range(len(robot.links)):
               joint = jointList[i]
               phi = mbs.GetObjectOutput(joint, exu.OutputVariableType.Rotation)[2] #z-rotation
               omega = mbs.GetObjectOutput(joint, exu.OutputVariableType.AngularVelocityLocal)[2] #z-angular velocity
               tsd = torsionalSDlist[i]
               mbs.SetObjectParameter(tsd, 'offset', u[i])
               mbs.SetObjectParameter(tsd, 'velocityOffset', v[i])
               mbs.SetObjectParameter(tsd, 'torque', staticTorques[i]) #additional torque from given velocity 
       else:
           #in case of kinematic tree, very simple operations!
           mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector', u)
           mbs.SetObjectParameter(oKT, 'jointVelocityOffsetVector', v)
           mbs.SetObjectParameter(oKT, 'jointForceVector', -staticTorques) #negative sign needed here!
       
       return True
   
   mbs.SetPreStepUserFunction(PreStepUF)
   
   sListJointAngles = []
   sListTorques = []
   nJoints = len(jointList)
   if not useKinematicTree:
       torsionalSDlist = robotDict['springDamperList']
       sJointRotComponents = [0]*nJoints #only one component
       sTorqueComponents = [0]*nJoints   #only one component
       
       #add sensors:
       cnt = 0
       for i in range(len(jointList)):
           jointLink = jointList[i]
           tsd = torsionalSDlist[i]
           #using TSD:
           sJointRot = mbs.AddSensor(SensorObject(objectNumber=tsd, storeInternal=True, 
                                      fileName="solution/joint" + str(i) + "Rot.txt",
                                      outputVariableType=exu.OutputVariableType.Rotation,
                                      writeToFile = sensorWriteToFile))
           sListJointAngles += [sJointRot]
           sJointAngVel = mbs.AddSensor(SensorObject(objectNumber=jointLink, storeInternal=True, 
                                                     fileName="solution/joint" + str(i) + "AngVel.txt",
                                                     outputVariableType=exu.OutputVariableType.AngularVelocityLocal,
                                                     writeToFile = sensorWriteToFile))
   
       cnt = 0
       for iSD in robotDict['springDamperList']:
           sTorque = mbs.AddSensor(SensorObject(objectNumber=iSD, storeInternal=True, 
                                                fileName="solution/jointTorque" + str(cnt) + ".txt", 
                                                outputVariableType=exu.OutputVariableType.TorqueLocal,
                                                writeToFile = sensorWriteToFile))
           sListTorques += [sTorque]
           
           cnt+=1
   else:
       sJointRotComponents = list(np.arange(0,nJoints))
       sTorqueComponents = list(np.arange(0,nJoints))
   
       sJointRot = mbs.AddSensor(SensorBody(bodyNumber=oKT, 
                                            storeInternal=True, 
                                            outputVariableType=exu.OutputVariableType.Coordinates))
       sListJointAngles = [sJointRot]*6
   
       #exu.OutputVariableType.Force is not the joint torque!
       sTorques = mbs.AddSensor(SensorBody(bodyNumber=oKT, storeInternal=True, 
                                             outputVariableType=exu.OutputVariableType.Force))
       sListTorques = [sTorques]*6
   
   
   mbs.Assemble()
   #mbs.systemData.Info()
   
   SC.visualizationSettings.connectors.showJointAxes = True
   SC.visualizationSettings.connectors.jointAxesLength = 0.02
   SC.visualizationSettings.connectors.jointAxesRadius = 0.002
   
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.1
   SC.visualizationSettings.loads.show = False
   
   SC.visualizationSettings.openGL.multiSampling=4
       
   # tEnd = 1.25
   # h = 0.002
   tEnd = 1.25
   h = 0.001#*0.1*0.01
   
   #SC.renderer.DoIdleTasks()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = 0.005
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.005
   simulationSettings.solutionSettings.binarySolutionFile = True
   #simulationSettings.solutionSettings.writeSolutionToFile = False
   # simulationSettings.timeIntegration.simulateInRealtime = True
   # simulationSettings.timeIntegration.realtimeFactor = 0.25
   
   simulationSettings.timeIntegration.verboseMode = 1
   # simulationSettings.displayComputationTime = True
   simulationSettings.displayStatistics = True
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   
   #simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
   SC.visualizationSettings.general.autoFitScene=False
   SC.visualizationSettings.window.renderWindowSize=[1920,1200]
   useGraphics = False
   
   if useGraphics:
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys['renderState'])
       SC.renderer.DoIdleTasks()
       
   mbs.SolveDynamic(simulationSettings, 
                    solverType=exu.DynamicSolverType.TrapezoidalIndex2, 
                    showHints=True)
   #explicit integration also possible for KinematicTree:
   # mbs.SolveDynamic(simulationSettings, 
   #                  solverType=exu.DynamicSolverType.RK33,
   #                  showHints=True)
   
   
   if useGraphics:
       SC.visualizationSettings.general.autoFitScene = False
       SC.renderer.Stop()
   
   if False:
       mbs.SolutionViewer()
   
   
   if not useKinematicTree:
       #compute final torques:
       measuredTorques=[]
       for sensorNumber in sListTorques:
           measuredTorques += [abs(mbs.GetSensorValues(sensorNumber)) ]
       exu.Print("torques at tEnd=", VSum(measuredTorques))
   
       measuredRot = []
       for sensorNumber in sListJointAngles :
           measuredRot += [(mbs.GetSensorValues(sensorNumber)) ]
       exu.Print("rotations at tEnd=", VSum(measuredRot), ',', measuredRot)
   
   else:
       q = mbs.GetObjectOutputBody(oKT, exu.OutputVariableType.Coordinates)
       exu.Print("rotations at tEnd=", VSum(q), ',', q)
       f = mbs.GetObjectOutputBody(oKT, exu.OutputVariableType.Force)
       exu.Print("torques at tEnd=", VSum(f), ',', f)
       
   
   
   
   #%%++++++++++++++++++++++++++++
   if True:
       mbs.PlotSensor(sensorNumbers=sListJointAngles, components=sJointRotComponents, 
                      title='joint angles', closeAll=True)
       if useKinematicTree: #otherwise not available
           mbs.PlotSensor(sensorNumbers=sListTorques, components=sTorqueComponents, 
                          title='joint torques')
       #sListJointAngles
   


