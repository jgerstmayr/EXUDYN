
.. _examples-serialrobotinteractivelimits:

*******************************
serialRobotInteractiveLimits.py
*******************************

You can view and download this file on Github: `serialRobotInteractiveLimits.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/serialRobotInteractiveLimits.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example of a serial robot with redundant coordinates
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-02-16
   # Revised:  2023-03-22
   # Note:     This example uses the redundant mbs approach; The kinematic tree approach would be much faster!
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
   from exudyn.interactive import InteractiveDialog
   
   import numpy as np
   from numpy import linalg as LA
   from math import pi
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   sensorWriteToFile = True
   
   mbs.variables['controlActive'] = 1 #flag to deactivate control
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #now in the new structure
   
   mode='newDH'
   
   graphicsBaseList = [graphics.Brick([0,0,-0.4], [0.12*1,0.12*1,0.6], graphics.color.grey)]
   graphicsBaseList +=[graphics.Cylinder([0,0,0], [0.5,0,0], 0.0025, graphics.color.red)]
   graphicsBaseList +=[graphics.Cylinder([0,0,0], [0,0.5,0], 0.0025, graphics.color.green)]
   graphicsBaseList +=[graphics.Cylinder([0,0,0], [0,0,0.5], 0.0025, graphics.color.blue)]
   graphicsBaseList +=[graphics.Cylinder([0,0,-0.1], [0,0,0.1], 0.05, graphics.color.blue)]
   graphicsBaseList +=[graphics.CheckerBoard([0,0,-0.7], [0,0,1], size=2)]
   #newRobot.base.visualization['graphicsData']=graphicsBaseList
   
   ty = 0.03
   tz = 0.04
   zOff = -0.05
   toolSize= [0.05,0.5*ty,0.06]
   graphicsToolList = [graphics.Cylinder(pAxis=[0,0,zOff], vAxis= [0,0,tz], radius=ty*1.5, color=graphics.color.red)]
   graphicsToolList+= [graphics.Brick([0,ty,1.5*tz+zOff], toolSize, graphics.color.grey)]
   graphicsToolList+= [graphics.Brick([0,-ty,1.5*tz+zOff], toolSize, graphics.color.grey)]
   
   
   #changed to new robot structure July 2021:
   newRobot = Robot(gravity=[0,0,0*9.81],
                 base = RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList)),
                 tool = RobotTool(HT=HTtranslate([0,0,0.1]), visualization=VRobotTool(graphicsData=graphicsToolList)),
                referenceConfiguration = []) #referenceConfiguration created with 0s automatically
   
   #modDHKK according to Khalil and Kleinfinger, 1986
   link0={'stdDH':[0,0,0,pi/2], 
          'modDHKK':[0,0,0,0], 
           'mass':20,  #not needed!
           'inertia':np.diag([1e-8,0.35,1e-8]), #w.r.t. COM!
           'COM':[0,0,0]}
   
   link1={'stdDH':[0,0,0.4318,0],
          'modDHKK':[0.5*pi,0,0,0], 
           'mass':17.4, 
           'inertia':np.diag([0.13,0.524,0.539]), #w.r.t. COM!
           'COM':[-0.3638, 0.006, 0.2275]}
   
   link2={'stdDH':[0,0.15,0.0203,-pi/2], 
          'modDHKK':[0,0.4318,0,0.15], 
           'mass':4.8, 
           'inertia':np.diag([0.066,0.086,0.0125]), #w.r.t. COM!
           'COM':[-0.0203,-0.0141,0.07]}
   
   link3={'stdDH':[0,0.4318,0,pi/2], 
          'modDHKK':[-0.5*pi,0.0203,0,0.4318], 
           'mass':0.82, 
           'inertia':np.diag([0.0018,0.0013,0.0018]), #w.r.t. COM!
           'COM':[0,0.019,0]}
   
   link4={'stdDH':[0,0,0,-pi/2], 
          'modDHKK':[0.5*pi,0,0,0], 
           'mass':0.34, 
           'inertia':np.diag([0.0003,0.0004,0.0003]), #w.r.t. COM!
           'COM':[0,0,0]}
   
   link5={'stdDH':[0,0,0,0], 
          'modDHKK':[-0.5*pi,0,0,0], 
           'mass':0.09, 
           'inertia':np.diag([0.00015,0.00015,4e-5]), #w.r.t. COM!
           'COM':[0,0,0.032]}
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
       link.visualization = VRobotLink(jointRadius=0.06, jointWidth=0.08, showMBSjoint=True, linkWidth=0.05, 
                                       linkColor=color, showCOM= False )
       cnt+=1
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #configurations and trajectory
   q0 = [0,0,0,0,0,0] #zero angle configuration
   
   # q1 = [0, pi/8, pi*0.75, 0,pi/8,0] #configuration 1
   # q2 = [pi,-pi, -pi*0.5,1.5*pi,-pi*2,pi*2] #configuration 2
   # q3 = [3*pi,0,-0.25*pi,0,0,0] #zero angle configuration
   
   #trajectory generated with optimal acceleration profiles:
   trajectory = Trajectory(initialCoordinates=q0, initialTime=0)
   # trajectory.Add(ProfileConstantAcceleration(q3,0.25))
   # trajectory.Add(ProfileConstantAcceleration(q1,0.25))
   # trajectory.Add(ProfileConstantAcceleration(q2,0.25))
   trajectory.Add(ProfileConstantAcceleration(q0,0.25))
   #traj.Add(ProfilePTP([1,1],syncAccTimes=False, maxVelocities=[1,1], maxAccelerations=[5,5]))
   
   # x = traj.EvaluateCoordinate(t,0)
   
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #test robot model
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #control parameters, per joint:
   fc=1
   Pcontrol = 0.1*np.array([40000, 40000, 40000, 10*100, 10*100, 10*10])
   Dcontrol = np.array([400,   400,   100,   10*1,   10*1,   10*0.1])
   Pcontrol = fc*Pcontrol
   Dcontrol = fc*Dcontrol
   #soft:
   #Pcontrol = [4000, 4000, 4000, 100, 100, 10]
   #Dcontrol = [40,   40,   10,   1,   1,   0.1]
   
   #desired angles:
   qE = q0
   qE = q0
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
   #add CartesianSpringDamper for mouse drag
   gripperBody=robotDict['bodyList'][-1]
   gripperLength = 0.1 #in z-direction
   markerGripper = mbs.AddMarker(MarkerBodyPosition(bodyNumber=gripperBody, localPosition=[0,0,gripperLength]))
   markerGround0 =  mbs.AddMarker(MarkerBodyPosition(bodyNumber=objectGround))
   
   # def UFcartesianSD(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset):
   #     if mbs.variables['controlActive']:
   #         return [0,0,0]
   #     else:
   #         p = SC.GetCurrentMouseCoordinates(True) #True=OpenGL coordinates; 2D
   #         A = np.array(SC.renderer.GetState()['modelRotation'])
   #         # print('p=',p)
   #         # print('u=',displacement)
   #         p3D = A@np.array([p[0],p[1],0.])
           
   #         dp = displacement-p3D
   #         f = [stiffness[0]*dp[0], stiffness[1]*dp[1], stiffness[2]*dp[2]]
   #         return f
       
   
   kSD = 50000*0.1
   dSD = kSD*0.01 #damping included in robot
   gripperSD = mbs.AddObject(CartesianSpringDamper(markerNumbers=[markerGround0, markerGripper], 
                                                   stiffness=[kSD,kSD,kSD],
                                                   damping=[dSD,dSD,dSD],
                                                   #springForceUserFunction=UFcartesianSD,
                                                   visualization=VCartesianSpringDamper(show=False), #do not show, looks weird
                                                   ))
   mbs.variables['gripperSD'] = gripperSD
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #test for joint limits:
   limits = []
   def UFtsd(mbs, t, itemNumber, rotation, angularVelocity, stiffness, damping, offset):
       f = 0.
       if False and mbs.variables['controlActive']:
           f = stiffness*rotation + damping*angularVelocity
       else:
           limTSD = limits[itemNumber]
           if rotation > limTSD[1]:
               f = 50*stiffness*(rotation-limTSD[1])**2 + stiffness*(rotation-offset) + damping*angularVelocity
           elif rotation < limTSD[0]:
               f = -50*stiffness*(rotation-limTSD[0])**2 + stiffness*(rotation-offset) + damping*angularVelocity
           else:
               f = stiffness*(rotation-offset) + damping*angularVelocity
       return f
               
   useUserFunction = 1
   if not useUserFunction:
       UFtsd = 0
   
   #control robot
   compensateStaticTorques = False 
   torsionalSDlist = []
   nGenericList = []
   limits = [[0.,0.]]*mbs.systemData.NumberOfObjects()
   limits += [[-0.75*pi,0.75*pi],
              [ 0.0*pi,1.0*pi],
              [-1.0*pi,0.4*pi],
              [-0.5*pi,0.5*pi],
              [-0.5*pi,0.5*pi],
              [-0.5*pi,0.5*pi],
              ]
   for i in range(len(jointList)):
       joint = jointList[i]
       rot0 = mbs.GetObject(joint)['rotationMarker0']
       rot1 = mbs.GetObject(joint)['rotationMarker1']
       markers = mbs.GetObject(joint)['markerNumbers']
       
       nGeneric=mbs.AddNode(NodeGenericData(initialCoordinates=[0], 
                                            numberOfDataCoordinates=1)) #for infinite rotations
       nGenericList += [nGeneric]
       tsd = mbs.AddObject(TorsionalSpringDamper(markerNumbers=markers,
                                           nodeNumber=nGeneric,
                                           rotationMarker0=rot0,
                                           rotationMarker1=rot1,                                            
                                           stiffness=Pcontrol[i],
                                           damping=Dcontrol[i],
                                           springTorqueUserFunction=UFtsd,
                                           visualization=VTorsionalSpringDamper(drawSize=0.1)
                                           ))
       torsionalSDlist += [tsd]
       
   
   #user function which is called only once per step, speeds up simulation drastically
   def PreStepUF(mbs, t):
   
       # print('nG=',end='')
       # for i in nGenericList:
       #     q = mbs.GetNodeOutput(i, exu.OutputVariableType.Coordinates)
       #     print(round(q,4),', ',end='')
       # print('')
   
       #additional static torque compensation; P and D control in TSD:
       if not mbs.variables['controlActive']:
           p = SC.GetCurrentMouseCoordinates(True) #True=OpenGL coordinates; 2D
           A = np.array(SC.renderer.GetState()['modelRotation'])
           p3D = A@np.array([p[0],p[1],0.])
           
           offset = p3D
           mbs.SetObjectParameter(mbs.variables['gripperSD'], 'offset', offset)
           mbs.SetObjectParameter(mbs.variables['gripperSD'], 'activeConnector', True)
       else:
           mbs.SetObjectParameter(mbs.variables['gripperSD'], 'activeConnector', False)
       
       #with control:
       if compensateStaticTorques:
           staticTorques = ComputeMBSstaticRobotTorques(newRobot)
           #print("tau=", staticTorques)
       else:
           staticTorques = np.zeros(len(jointList))
           
           
       [u,v,a] = trajectory.Evaluate(t)
       
       fact = 1.
       if mbs.variables['controlActive']==2:
           fact =1. #0.1 #soft reset ...
           
       #compute load for joint number
       for i in range(len(jointList)):
           joint = jointList[i]
           phi = mbs.GetObjectOutput(joint, exu.OutputVariableType.Rotation)[2] #z-rotation
           omega = mbs.GetObjectOutput(joint, exu.OutputVariableType.AngularVelocityLocal)[2] #z-angular velocity
           #[u1,v1,a1] = MotionInterpolator(t, robotTrajectory, i)
           tsd = torsionalSDlist[i]
           if mbs.variables['controlActive'] or i>=3:
               mbs.SetObjectParameter(tsd, 'torque', staticTorques[i]) #additional torque from given velocity; positive sign from Exudyn 1.2.38 onwards
               mbs.SetObjectParameter(tsd, 'stiffness', Pcontrol[i]*fact) 
               mbs.SetObjectParameter(tsd, 'damping', Dcontrol[i]*1) 
   
       # with mouse drag:
       for i in range(len(jointList)):
           if not (mbs.variables['controlActive'] or i>=3):
               tsd = torsionalSDlist[i]
               #keep damping, but deactivate stiffness
               mbs.SetObjectParameter(tsd, 'torque', 0) 
               #mbs.SetObjectParameter(tsd, 'stiffness', 0) 
               mbs.SetObjectParameter(tsd, 'damping', Dcontrol[i]*0.1) #keep small damping to improve drag!
       
       return True
   
   mbs.SetPreStepUserFunction(PreStepUF)
   
   
   # mbs.variables['q0Current'] = q0[0]
   for i in range(6): 
       mbs.variables['q{}Current'.format(i)] = q0[i]
   
   #add sensors:
   cnt = 0
   for i in range(len(jointList)):
       jointLink = jointList[i]
       tsd = torsionalSDlist[i]
       sJointRot = mbs.AddSensor(SensorObject(objectNumber=tsd, 
                                  fileName="solution/joint" + str(cnt) + "Rot.txt",
                                  outputVariableType=exu.OutputVariableType.Rotation,
                                  writeToFile = sensorWriteToFile))
       sJointAngVel = mbs.AddSensor(SensorObject(objectNumber=jointLink, 
                                  fileName="solution/joint" + str(cnt) + "AngVel.txt",
                                  outputVariableType=exu.OutputVariableType.AngularVelocityLocal,
                                  writeToFile = sensorWriteToFile))
       cnt+=1
   
   cnt = 0
   jointTorque0List = []
   for load0 in robotDict['jointTorque0List']:
       sTorque = mbs.AddSensor(SensorLoad(loadNumber=load0, fileName="solution/jointTorque" + str(cnt) + ".txt", 
                                          writeToFile = sensorWriteToFile))
       jointTorque0List += [sTorque]
       cnt+=1
   
   
   
   
   def GetPoseString(q): 
       strx = '   x = ['
       strphi = 'phi = [' 
       HT = newRobot.JointHT(q)[-1]
       t = np.round(HT[0:3,-1], 4)
       R = HT[0:3,0:3]
       phi = np.round(RotationMatrix2RotXYZ(R),3)
       
       for i in range(2): 
           strx += '{},\t'.format(t[i])
           strphi += '{},\t'.format(phi[i])
           
       strx += '{}]'.format(t[-1])
       strphi += '{}]'.format(phi[-1])
       diffLen = len(strx) - len(strphi)
       if diffLen > 0: 
           strphi += ' '*diffLen
       elif diffLen < 0: 
           strx += ' '*diffLen
           
       return strx + '\n' + strphi
   
   #define items for dialog
   fAngle=2.
   dialogItems = [{'type':'label', 'text':'Angle 1', 'grid':(0,0,2), 'options':['L']}, 
                  {'type':'slider', 'range': (-fAngle*3.14, fAngle*3.14), 'value':q0[0], 'steps':628, 'variable':'q0Current', 'grid':(0,1)},
                  {'type':'label', 'text':'Angle 2:', 'grid':(1,0)},
                  {'type':'slider', 'range': (-fAngle*3.14, fAngle*3.14), 'value':q0[1], 'steps':628, 'variable':'q1Current', 'grid':(1,1)},
                  {'type':'label', 'text':'Angle 3:', 'grid':(2,0)},
                  {'type':'slider', 'range': (-fAngle*3.14, fAngle*3.14), 'value':q0[2], 'steps':628, 'variable':'q2Current', 'grid':(2,1)},
                  {'type':'label', 'text':'Angle 4:', 'grid':(3,0)},
                  {'type':'slider', 'range': (-fAngle*3.14, fAngle*3.14), 'value':q0[3], 'steps':628, 'variable':'q3Current', 'grid':(3,1)},
                  {'type':'label', 'text':'Angle 5:', 'grid':(4,0)},
                  {'type':'slider', 'range': (-fAngle*3.14, fAngle*3.14), 'value':q0[4], 'steps':628, 'variable':'q4Current', 'grid':(4,1)},
                  {'type':'label', 'text':'Angle 6:', 'grid':(5,0)},
                  {'type':'slider', 'range': (-fAngle*3.14, fAngle*3.14), 'value':q0[5], 'steps':628, 'variable':'q5Current', 'grid':(5,1)},
                  {'type': 'label', 'text': 'Position:', 'grid': (6,0)}, 
                  {'type': 'label', 'text': '{}'.format(GetPoseString(q0)), 'grid': (6,1)},
                  {'type':'radio', 'textValueList':[('Mouse drag',0),('Control on',1),('Reset',2)], 'value':1, 'variable':'controlActive', 
                   'grid': [(7,0),(7,1),(7,2)]}
                  #{'type':'button', 'text':'test button','callFunction':ButtonCall, 'grid':(7,0,2)},
                  ]
   
   
   mbs.Assemble()
   #mbs.systemData.Info()
   
   SC.visualizationSettings.connectors.showJointAxes = True
   SC.visualizationSettings.connectors.jointAxesLength = 0.02
   SC.visualizationSettings.connectors.jointAxesRadius = 0.002
   
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.1
   SC.visualizationSettings.loads.show = False
   
   SC.visualizationSettings.openGL.multiSampling=4
   SC.visualizationSettings.openGL.shadow=0.3
   SC.visualizationSettings.openGL.perspective=0.7
       
   tEnd = 1.25
   h = 0.0005
   
   #SC.renderer.DoIdleTasks()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   simulationSettings.solutionSettings.solutionInformation = 'Hanging Robot Interactive Example'
   
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = h*1
   simulationSettings.solutionSettings.sensorsWritePeriod = h*10
   simulationSettings.solutionSettings.binarySolutionFile = True
   simulationSettings.timeIntegration.verboseMode = 0 
   #simulationSettings.solutionSettings.writeSolutionToFile = False
   # simulationSettings.timeIntegration.simulateInRealtime = True
   # simulationSettings.timeIntegration.realtimeFactor = 0.25
   simulationSettings.solutionSettings.writeInitialValues = False
   
   simulationSettings.displayComputationTime = False
   simulationSettings.displayStatistics = False
   # simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   # simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=False
   
   #simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   SC.visualizationSettings.window.renderWindowSize=[1920,1200]
   SC.visualizationSettings.general.graphicsUpdateInterval=0.005
   
   #this is an exemplariy simulation function, which adjusts some values for simulation
   def SimulationUF(mbs, dialog):
       q = []
       for i in range(6): 
           qi = mbs.variables['q{}Current'.format(i)] #not possible to update this variable
           #qi = dialog.sliderVariables[i].get()
           mbs.SetObjectParameter(torsionalSDlist[i],'offset',qi)
           theta = mbs.GetObjectOutput(torsionalSDlist[i],exu.OutputVariableType.Rotation)
           #q += [mbs.variables['q{}Current'.format(i)]]
           q += [1.*theta] #current rotation
       #dialog.dialogItems[-1]['text'] = GetPoseString(q)
       
       if mbs.variables['controlActive'] == 2:
           for i in range(6): 
               q[i] = 0
               mbs.variables['q{}Current'.format(i)] = 0
               dialog.widgets[2*i+1].set(q[i])
   
       dialog.labelStringVariables[-1].set(GetPoseString(q))
   
       if not mbs.variables['controlActive']:
           for i in range(6): 
               #dialog.sliderVariables[i].set(q[i])
               dialog.widgets[2*i+1].set(q[i])
       #print(q)
       
   SC.visualizationSettings.general.autoFitScene = False #use loaded render state
   SC.renderer.Start()
   if 'renderState' in exu.sys:
       SC.renderer.SetState(exu.sys[ 'renderState' ])
   
   InteractiveDialog(mbs=mbs, simulationSettings=simulationSettings,
                     simulationFunction=SimulationUF, 
                     title='Interactive window',
                     dialogItems=dialogItems, period=0.01, realtimeFactor=4, #realtime is only approx. (does not include time lost for computation ==> 2 is a good choice)
                     runOnStart=True,
                     addLabelStringVariables=True,
                     #addSliderVariables=True
                     )
   
   SC.renderer.Stop()
   
   if 0: 
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
           measuredTorques += [mbs.GetSensorValues(sensorNumber)[2]]
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
   


