
.. _examples-serialrobotinversekinematics:

*******************************
serialRobotInverseKinematics.py
*******************************

You can view and download this file on Github: `serialRobotInverseKinematics.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/serialRobotInverseKinematics.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example of a serial robot with minimal coordinates using the inverse kinematics Funcion of Exudyn
   #
   # Author:   Johannes Gerstmayr
   # Date:     2023-03-29
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #q
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.rigidBodyUtilities import *
   from exudyn.graphicsDataUtilities import *
   from exudyn.robotics import *
   from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration, ProfilePTP
   from exudyn.robotics.models import ManipulatorPuma560, ManipulatorPANDA, ManipulatorUR5, LinkDict2Robot
   from exudyn.lieGroupBasics import LogSE3, ExpSE3
   
   import numpy as np
   
   exu.RequireVersion('1.6.31')
   # exuVersion = np.array(exu.__version__.split('.')[:3], dtype=int)
   exuVersion = exu.__version__.split('.')[:3]
   exuVersion = exuVersion[0] + exuVersion[1] + exuVersion[2]
   if int(exuVersion) < 1631: 
       print('\nWarning: use at least exudyn verion 1.6.31.dev1')
       print('You can install the latest development version with\npip install -U exudyn --pre\n\n')
       
   
   #%% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #motion planning:
   jointSpaceInterpolation = False #false interpolates TCP position in work space/Cartesian coordinates
   motionCase = 2 # case 1 and 2 move in different planes
   
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #kinematic tree and redundant mbs agrees for stdDH version up to 1e-10, with compensateStaticTorques = False
   # KT:      rotations at tEnd= 1.8464514676503092 , [0.4921990591981066, 0.2718999073958087, 0.818158053005264, -0.0030588904101585936, 0.26831938569719394, -0.0010660472359057434] 
   # red. MBS:rotations at tEnd= 1.8464514674961 ,   [ 0.49219906  0.27189991  0.81815805 -0.00305889  0.26831939 -0.00106605]
   
   #some graphics settings to get base and tool visualization
   jointWidth=0.1
   jointRadius=0.06
   linkWidth=0.1
   
   graphicsBaseList = [graphics.Brick([0,0,-0.35], [0.12,0.12,0.5], graphics.color.grey)]
   graphicsBaseList +=[graphics.Cylinder([0,0,0], [0.5,0,0], 0.0025, graphics.color.red)]
   graphicsBaseList +=[graphics.Cylinder([0,0,0], [0,0.5,0], 0.0025, graphics.color.green)]
   graphicsBaseList +=[graphics.Cylinder([0,0,0], [0,0,0.5], 0.0025, graphics.color.blue)]
   graphicsBaseList +=[graphics.Cylinder([0,0,-jointWidth], [0,0,jointWidth], linkWidth*0.5, graphics.colorList[0])] #belongs to first body
   graphicsBaseList +=[graphics.CheckerBoard([0,0,-0.6], [0,0,1], size=2)]
   
   ty = 0.03
   tz = 0.04
   zOff = -0.05
   toolSize= [0.05,0.5*ty,0.06]
   graphicsToolList = [graphics.Cylinder(pAxis=[0,0,zOff], vAxis= [0,0,tz], radius=ty*1.5, color=graphics.color.red)]
   graphicsToolList+= [graphics.Brick([0,ty,1.5*tz+zOff], toolSize, graphics.color.grey)]
   graphicsToolList+= [graphics.Brick([0,-ty,1.5*tz+zOff], toolSize, graphics.color.grey)]
   
   # here another robot could be loaded; note that the robots sizes and workspaces and zero configuration 
   # differ, so the HTmove may need to be adjusted to be inside the workspace
   robotDef = ManipulatorPuma560()
   # robotDef = ManipulatorUR5()
   # robotDef = ManipulatorPANDA()
   HTtool = HTtranslate([0,0,0.08])
   
   robot = Robot(gravity=[0,0,-9.81],
                 base = RobotBase(HT=HTtranslate([0,0,0]), visualization=VRobotBase(graphicsData=graphicsBaseList)),
                 tool = RobotTool(HT=HTtool, visualization=VRobotTool(graphicsData=graphicsToolList)),
                 referenceConfiguration = []) #referenceConfiguration created with 0s automatically
   
   robot=LinkDict2Robot(robotDef, robot)
   
   ik = InverseKinematicsNumerical(robot=robot, useRenderer=False, 
                                   #flagDebug=True, 
                                   #jointStiffness=1e4
                                   ) #, useAlternativeConstraints=True)
   SC = exu.SystemContainer() 
   SC.renderer.Attach()
   mbs = SC.AddSystem()
   # the system container holds one or several systems; the inverse kinematics uses a second system container without visualization
   # in the mbs the mainsystem (multibody system) is stored; in the SC more than one mbs can be created but they do not interact with each other. 
   
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #configurations and trajectory
   q0 = [0,0,0,0,0,0] #zero angle configuration
   # q1 = [0, pi/8, pi*0.5, 0,pi/8,0] #configuration 1
   # q2 = [0.8*pi,-0.8*pi, -pi*0.5,0.75*pi,-pi*0.4,pi*0.4] #configuration 2
   # q3 = [0.5*pi,0,-0.25*pi,0,0,0] # configuration 3
   
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create robot model in mbs
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   mbs.variables['myIkine'] = ik
   jointHTs = robot.JointHT(q0)
   HTlastJoint = jointHTs[-1]@HTtool
   
   #prescribed motion:
   #HTmove = HTtranslate([-0.25,0.,0.3])
   if motionCase == 1: 
       HTmove = HT(RotationMatrixX(-0.3*pi),[-0.45,0.,0.]) #goes through singularity
   elif motionCase == 2: 
       HTmove = HT(RotationMatrixX(0.3*pi),[0.,0.,-0.3])    #no singularity
   else: 
       print('no valid motionCase provided')
   
   ## here another motionCase/HTmove could be added 
   
   logMove = LogSE3(HTmove)
   rotMove = Skew2Vec(logMove[0:3,0:3])
   dispMove = HT2translation(logMove)
   
   q0=[0,0,0,0.01,0.02,0.03]
   [q1, success] = ik.SolveSafe(HTlastJoint@HTmove, q0)
   if not success: print('[q1, success]=',[q1, success])
   [q2, success] = ik.SolveSafe(HTlastJoint@HTmove@HTmove, q1)
   if not success: print('[q2, success]=',[q2, success])
   
   #initialize for first simulation step:
   [q, success] = ik.Solve(HTlastJoint, q0)
   print('[q, success]=',[q, success])
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #trajectory generated with optimal acceleration profiles, for joint-interpolation:
   trajectory = Trajectory(initialCoordinates=q0, initialTime=0)
   trajectory.Add(ProfileConstantAcceleration(q1,1))
   trajectory.Add(ProfileConstantAcceleration(q1,1))
   trajectory.Add(ProfileConstantAcceleration(q2,1))
   trajectory.Add(ProfileConstantAcceleration(q2,1))
   #traj.Add(ProfilePTP([1,1],syncAccTimes=False, maxVelocities=[1,1], maxAccelerations=[5,5]))
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   
   
   #use frame for prescribed TCP:
   gFrame = [graphics.Frame(HTlastJoint, length=0.3, colors=[graphics.color.lightgrey]*3)]
   gFrame += [graphics.Frame(HTlastJoint@HTmove, length=0.3, colors=[graphics.color.grey]*3)]
   gFrame += [graphics.Frame(HTlastJoint@HTmove@HTmove, length=0.3, colors=[graphics.color.dodgerblue]*3)]
   oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=gFrame)))
   
   robotDict = robot.CreateKinematicTree(mbs)
   oKT = robotDict['objectKinematicTree']
   
   #add sensor for joint coordinates (relative to reference coordinates)
   sJoints = mbs.AddSensor(SensorNode(nodeNumber=robotDict['nodeGeneric'], storeInternal=True, 
                            outputVariableType=exu.OutputVariableType.Coordinates))
   
   sPosTCP = mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=5, localPosition=HT2translation(HTtool), 
                                               storeInternal=True, 
                                               outputVariableType=exu.OutputVariableType.Position))
   
   sRotTCP = mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=5, localPosition=HT2translation(HTtool), 
                                               storeInternal=True, 
                                               outputVariableType=exu.OutputVariableType.RotationMatrix))
   
   #user function which is called only once per step, speeds up simulation drastically
   def PreStepUF(mbs, t):
       if not jointSpaceInterpolation:
           ik = mbs.variables['myIkine']
           if t < 2:
               tt = min(t,1)
               T = HTlastJoint@ExpSE3(list(tt*dispMove)+list(tt*rotMove))
           elif t < 4:
               tt=min(t-2,1)
               T = HTlastJoint@HTmove@ExpSE3(list(tt*dispMove)+list(tt*rotMove))
           else:
               T = HTlastJoint@HTmove@HTmove
   
           [q,success]=ik.Solve(T) #, q0) #takes 60 us internally
           mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector', q)
           
       else:
           #standard trajectory planning:
           
           [u,v,a] = trajectory.Evaluate(t)
           #in case of kinematic tree, very simple operations!
           mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector', u)
           mbs.SetObjectParameter(oKT, 'jointVelocityOffsetVector', v)
       
       return True
   
   mbs.SetPreStepUserFunction(PreStepUF)
   
   
   
   
   mbs.Assemble() # assemble the dynamic system
   #mbs.systemData.Info() 
   
   #%% setting for visualization
   SC.visualizationSettings.connectors.showJointAxes = True
   SC.visualizationSettings.connectors.jointAxesLength = 0.02
   SC.visualizationSettings.connectors.jointAxesRadius = 0.002
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.1 
   SC.visualizationSettings.loads.show = False # shows external loads 
   SC.visualizationSettings.bodies.kinematicTree.showJointFrames = False # shows the frames for each joint of the robot
   SC.visualizationSettings.openGL.multiSampling=4
   SC.visualizationSettings.general.autoFitScene=False
   SC.visualizationSettings.window.renderWindowSize=[1920,1200]
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.01
   SC.visualizationSettings.openGL.shadow=0.3
   SC.visualizationSettings.openGL.perspective=0.5
   
   #traces:
   SC.visualizationSettings.sensors.traces.listOfPositionSensors = [sPosTCP]
   SC.visualizationSettings.sensors.traces.listOfTriadSensors =[sRotTCP]
   SC.visualizationSettings.sensors.traces.showPositionTrace=True
   SC.visualizationSettings.sensors.traces.showTriads=True
   SC.visualizationSettings.sensors.traces.showVectors=False
   SC.visualizationSettings.sensors.traces.showFuture=False
   SC.visualizationSettings.sensors.traces.triadsShowEvery=5
   
   #%% 
   tEnd = 5 # endtime of the simulation
   h = 0.002 #500 steps take 0.16 seconds, 0.3ms / step (83% Python + inverse kinematics)
   
   # settings for the time integration / simulation
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h) 
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = 0.02 
   # determines the timesteps in which the solution is saved; when it is decreased more solutions are saved, which also 
   # enables the solutionViewer to
   
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.005
   #simulationSettings.solutionSettings.writeSolutionToFile = False
   # simulationSettings.timeIntegration.simulateInRealtime = True
   # simulationSettings.timeIntegration.realtimeFactor = 0.025 # slow down simulation to look at
   
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.displayComputationTime = True
   # simulationSettings.displayStatistics = True
   #simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   # solver type
   simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
   
   #%% 
   useGraphics = True 
   # if useGraphics is false rendering while calculating the solution is dectivated, 
   # the solution is still saved and can be then visualized afterwards for the solution viewer
   
   
   if useGraphics:
       # start graphics
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys['renderState'])
       SC.renderer.DoIdleTasks()
       
   # hte the simulation with the set up simulationsettings is started 
   mbs.SolveDynamic(simulationSettings, 
                    #solverType = exudyn.DynamicSolverType.TrapezoidalIndex2, # different solver types can be used depending on the problem
                    showHints=True)
   
   
   if useGraphics: # when graphics are deactivated the renderer does not need to be stopped
       SC.visualizationSettings.general.autoFitScene = False
       SC.renderer.Stop()
   
   #%%++++++++++++++++++++++++++++++++++++++++++++
   if True:
       # import the solution viewer which loads 
       
       mbs.SolutionViewer()
   #%%++++++++++++++++++++++++++++++++++++++++++++
   
   
   q = mbs.GetObjectOutputBody(oKT, exu.OutputVariableType.Coordinates)
   exu.Print("rotations at tEnd=", VSum(q), ',', q)
       
   #%%++++++++++++++++++++++++++++++++++++++++++++
   if True:
       
       labels = []
       for i in range(6):
           labels += ['joint '+str(i)]
       # plot data from the sensor sJoints 6 times and take the values 0 to 5 in the steps
       mbs.PlotSensor(sensorNumbers=[sJoints]*6, components=list(np.arange(6)), yLabel='joint coordinates', labels=labels) 
       


