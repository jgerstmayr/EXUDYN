
.. _examples-serialrobottsd:

*****************
serialRobotTSD.py
*****************

You can view and download this file on Github: `serialRobotTSD.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/serialRobotTSD.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example of a serial robot with redundant coordinates
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-02-16
   # Revised:  2021-07-09
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
   
   sensorWriteToFile = True
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   mode='newDH'
   
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
   
   
   #changed to new robot structure July 2021:
   newRobot = Robot(gravity=[0,0,9.81],
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
   
   for link in linkList:
       newRobot.AddLink(RobotLink(mass=link['mass'], 
                                  COM=link['COM'], 
                                  inertia=link['inertia'], 
                                  localHT=StdDH2HT(link['stdDH']),
                                  ))
   
   cnt = 0
   for link in newRobot.links:
       color = graphics.colorList[cnt]
       color[3] = 0.75 #make transparent
       link.visualization = VRobotLink(jointRadius=jointRadius, jointWidth=jointWidth, showMBSjoint=False,
                                       linkWidth=linkWidth, linkColor=color, showCOM= True )
       cnt+=1
   
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
   #control parameters, per joint:
   fc=1
   Pcontrol = np.array([40000, 40000, 40000, 100, 100, 10])
   Dcontrol = np.array([400,   400,   100,   1,   1,   0.1])
   Pcontrol = fc*Pcontrol
   Dcontrol = fc*Dcontrol
   #soft:
   # Pcontrol = [4000, 4000, 4000, 100, 100, 10]
   # Dcontrol = [40,   40,   10,   1,   1,   0.1]
   
   #desired angles:
   qE = q0
   qE = [pi*0.5,-pi*0.25,pi*0.75, 0,0,0]
   tStart = [0,0,0, 0,0,0]
   duration = 0.1
   
   
   jointList = [0]*newRobot.NumberOfLinks() #this list must be filled afterwards with the joint numbers in the mbs!
   
   def ComputeMBSstaticRobotTorques(newRobot):
       q=[]
       for joint in jointList:
           q += [mbs.GetObjectOutput(joint, exu.OutputVariableType.Rotation)[2]] #z-rotation
       HT=newRobot.JointHT(q)
       return newRobot.StaticTorques(HT)
   
   #++++++++++++++++++++++++++++++++++++++++++++++++
   #base, graphics, object and marker:
   
   objectGround = mbs.AddObject(ObjectGround(referencePosition=HT2translation(newRobot.GetBaseHT()), 
                                         #visualization=VObjectGround(graphicsData=graphicsBaseList)
                                             ))
   
   
   #baseMarker; could also be a moving base!
   baseMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=objectGround, localPosition=[0,0,0]))
   
   
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #build mbs robot model:
   robotDict = newRobot.CreateRedundantCoordinateMBS(mbs, baseMarker=baseMarker)
       
   jointList = robotDict['jointList'] #must be stored there for the load user function
   
   unitTorques0 = robotDict['unitTorque0List'] #(left body)
   unitTorques1 = robotDict['unitTorque1List'] #(right body)
   
   loadList0 = robotDict['jointTorque0List'] #(left body)
   loadList1 = robotDict['jointTorque1List'] #(right body)
   #print(loadList0, loadList1)
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #control robot
   compensateStaticTorques = True
   torsionalSDlist = []
   
   for i in range(len(jointList)):
       joint = jointList[i]
       rot0 = mbs.GetObject(joint)['rotationMarker0']
       rot1 = mbs.GetObject(joint)['rotationMarker1']
       markers = mbs.GetObject(joint)['markerNumbers']
       nGeneric=mbs.AddNode(NodeGenericData(initialCoordinates=[0], 
                                            numberOfDataCoordinates=1)) #for infinite rotations
       tsd = mbs.AddObject(TorsionalSpringDamper(markerNumbers=markers,
                                           nodeNumber=nGeneric,
                                           rotationMarker0=rot0,
                                           rotationMarker1=rot1,                                            
                                           stiffness=Pcontrol[i],
                                           damping=Dcontrol[i],
                                           visualization=VTorsionalSpringDamper(drawSize=0.1)
                                           ))
       torsionalSDlist += [tsd]
       
   
   #user function which is called only once per step, speeds up simulation drastically
   def PreStepUF(mbs, t):
       if compensateStaticTorques:
           staticTorques = ComputeMBSstaticRobotTorques(newRobot)
           #print("tau=", staticTorques)
       else:
           staticTorques = np.zeros(len(jointList))
           
       [u,v,a] = trajectory.Evaluate(t)
   
       #compute load for joint number
       for i in range(len(jointList)):
           joint = jointList[i]
           phi = mbs.GetObjectOutput(joint, exu.OutputVariableType.Rotation)[2] #z-rotation
           omega = mbs.GetObjectOutput(joint, exu.OutputVariableType.AngularVelocityLocal)[2] #z-angular velocity
           #[u1,v1,a1] = MotionInterpolator(t, robotTrajectory, i)
           u1 = u[i]
           v1 = v[i]
           tsd = torsionalSDlist[i]
           mbs.SetObjectParameter(tsd, 'offset', u1)
           mbs.SetObjectParameter(tsd, 'velocityOffset', v1)
           mbs.SetObjectParameter(tsd, 'torque', staticTorques[i]) #additional torque from given velocity 
       
       return True
   
   mbs.SetPreStepUserFunction(PreStepUF)
   
   
   
   #add sensors:
   cnt = 0
   jointTorque0List = []
   for i in range(len(jointList)):
       jointLink = jointList[i]
       tsd = torsionalSDlist[i]
       #using TSD:
       sJointRot = mbs.AddSensor(SensorObject(objectNumber=tsd, 
                                  fileName="solution/joint" + str(i) + "Rot.txt",
                                  outputVariableType=exu.OutputVariableType.Rotation,
                                  writeToFile = sensorWriteToFile))
   
       sJointAngVel = mbs.AddSensor(SensorObject(objectNumber=jointLink, 
                                  fileName="solution/joint" + str(i) + "AngVel.txt",
                                  outputVariableType=exu.OutputVariableType.AngularVelocityLocal,
                                  writeToFile = sensorWriteToFile))
   
       sTorque = mbs.AddSensor(SensorObject(objectNumber=tsd, 
                               fileName="solution/joint" + str(i) + "Torque.txt",
                               outputVariableType=exu.OutputVariableType.TorqueLocal,
                               writeToFile = sensorWriteToFile))
   
       jointTorque0List += [sTorque]
   
   
   mbs.Assemble()
   #mbs.systemData.Info()
   
   SC.visualizationSettings.connectors.showJointAxes = True
   SC.visualizationSettings.connectors.jointAxesLength = 0.02
   SC.visualizationSettings.connectors.jointAxesRadius = 0.002
   
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.1
   SC.visualizationSettings.loads.show = False
   
   SC.visualizationSettings.openGL.multiSampling=4
       
   tEnd = 1.25
   h = 0.002
   
   #SC.renderer.DoIdleTasks()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = h*1
   simulationSettings.solutionSettings.sensorsWritePeriod = h*10
   simulationSettings.solutionSettings.binarySolutionFile = True
   #simulationSettings.solutionSettings.writeSolutionToFile = False
   # simulationSettings.timeIntegration.simulateInRealtime = True
   # simulationSettings.timeIntegration.realtimeFactor = 0.25
   
   simulationSettings.timeIntegration.verboseMode = 1
   # simulationSettings.displayComputationTime = True
   simulationSettings.displayStatistics = True
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   
   #simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints
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
       
   mbs.SolveDynamic(simulationSettings, showHints=True)
   
   
   if useGraphics:
       SC.visualizationSettings.general.autoFitScene = False
       SC.renderer.Stop()
   
   
   mbs.SolutionViewer()
   
   lastRenderState = SC.renderer.GetState() #store model view
   
   #compute final torques:
   measuredTorques=[]
   for sensorNumber in jointTorque0List:
       measuredTorques += [abs(mbs.GetSensorValues(sensorNumber))]
   exu.Print("torques at tEnd=", VSum(measuredTorques))
   
   
   if True:
       import matplotlib.pyplot as plt
       import matplotlib.ticker as ticker
       plt.rcParams.update({'font.size': 14})
       plt.close("all")
   
       doJointTorques = False
       if doJointTorques:
           for i in range(6):
               data = np.loadtxt("solution/jointTorque" + str(i) + ".txt", comments='#', delimiter=',')
               plt.plot(data[:,0], data[:,3], PlotLineCode(i), label="joint torque"+str(i)) #z-rotation
       
           plt.xlabel("time (s)")
           plt.ylabel("joint torque (Nm)")
           ax=plt.gca() # get current axes
           ax.grid(True, 'major', 'both')
           ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
           ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
           plt.tight_layout()
           ax.legend(loc='center right')
           plt.show() 
           # plt.savefig("solution/robotJointTorques.pdf")
   
       doJointAngles = True
       if doJointAngles:
           plt.close("all")
           
           for i in range(6):
               data = np.loadtxt("solution/joint" + str(i) + "Rot.txt", comments='#', delimiter=',')
               # data = np.loadtxt("solution/joint" + str(i) + "AngVel.txt", comments='#', delimiter=',')
               plt.plot(data[:,0], data[:,1], PlotLineCode(i), label="joint"+str(i)) #z-rotation
               
           plt.xlabel("time (s)")
           plt.ylabel("joint angle (rad)")
           ax=plt.gca() 
           ax.grid(True, 'major', 'both')
           ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
           ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
           plt.tight_layout()
           ax.legend()
           plt.rcParams.update({'font.size': 16})
           plt.show() 
           # plt.savefig("solution/robotJointAngles.pdf")
   


