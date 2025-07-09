
.. _examples-serialroboturdf:

******************
serialRobotURDF.py
******************

You can view and download this file on Github: `serialRobotURDF.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/serialRobotURDF.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example of a serial robot with URDF file
   #
   # Author:   Johannes Gerstmayr
   # Date:     2024-11-07
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   #from exudyn.rigidBodyUtilities import *
   from exudyn.robotics import *
   from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration, ProfilePTP
   from exudyn.robotics.utilities import GetRoboticsToolboxInternalModel, LoadURDFrobot, GetURDFrobotData
   
   import numpy as np
   from numpy import linalg as LA
   from math import pi
   import sys
   
   #%%+++++++++++++++++++
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[1.5,2,0], size=6)])
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #%%+++++++++++++++++++++++++++++++
   robotModels = ['UR3', 'UR5', 'UR10', 'Puma560', 'LBR', 'Panda', 'vx300', 'wx250', 'px150']
   robotModels = ['UR5'] #only one robot
   onlyShowModel = False #True: only graphics, no simulation
   listKinematicTree = []
   listTrajectory = []
   saveHDF5 = False        #use this to save robot data, allowing to load directly from there
   loadFromHDF5 = False#faster load; does not require roboticstoolbox, pymeshlab, etc.
   
   for modelCnt, modelName in enumerate(robotModels):
       if not loadFromHDF5:
           urdfRobotDict = GetRoboticsToolboxInternalModel(modelName, ignoreURDFerrors=False)
           
           urdf = urdfRobotDict['urdf']
           robot = urdfRobotDict['robot']
           linkColorList = None
           if modelCnt < 6: #robots have no nice colors
               linkColorList = [graphics.color.grey, graphics.color.lightgrey]*10
           robotDataDict = GetURDFrobotData(robot, urdf, 
                                            returnStaticGraphicsList=onlyShowModel,
                                            linkColorList=linkColorList)
           
           if saveHDF5:
               from exudyn.advancedUtilities import SaveDictToHDF5
               SaveDictToHDF5('solution/robot'+modelName+'.h5', robotDataDict)
       else:
           from exudyn.advancedUtilities import LoadDictFromHDF5
           robotDataDict = LoadDictFromHDF5('solution/robot'+modelName+'.h5')
       
       #%%+++++++++++++++++
       #draw in roboticstoolbox:
       #robot.plot(robot.qr) #, backend='swift')
       offsetPosition = np.array([modelCnt%3,1.3*int(modelCnt/3),0])
       #if modelName=='LBR': offsetPosition[0]-=0.8 #robot not placed at [0,0,0]
   
       #static view of robot in Exudyn with staticGraphicsList
       if onlyShowModel:
           if modelCnt == 0: mbs.CreateMassPoint(physicsMass=1) #to avoid zero-DOF system
           mbs.CreateGround(referencePosition=offsetPosition,
                            graphicsDataList=robotDataDict['staticGraphicsList'])
           # continue
       
       
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       linkList = robotDataDict['linkList']
       numberOfJoints = robotDataDict['numberOfJoints']
   
       if len(linkList) != numberOfJoints:
           #if this does not work for robots, probably some weird configuration or branches/multiple arms
           raise ValueError('robot creation: inconsistent data!')
           
       
       graphicsBaseList = robotDataDict['graphicsBaseList']
       graphicsToolList = robotDataDict['graphicsToolList']
       
       #some control parameters, per joint: (have to be replaced by real values)
       Pcontrol = np.array([1e5, 1e5, 1e5, 1e3, 1e3, 1e3]+[1e3]*(numberOfJoints-6))
       Dcontrol = np.array([1e3, 1e3, 1e2, 1e1, 1e1, 1e1]+[1e1]*(numberOfJoints-6))
       if (modelName.startswith('vx')
           or modelName.startswith('wx')
           or modelName.startswith('px')):
           Pcontrol[-3:] *= 1e3 #we need stiffer controller!
           Dcontrol[-3:] *= 1e3
       
       #changed to new robot structure July 2021:
       mbsRobot = Robot(gravity=[0,0,9.81],
                     base = RobotBase(HT = HTtranslate(offsetPosition),
                                      visualization=VRobotBase(graphicsData=graphicsBaseList)),
                     tool = RobotTool(HT=HTtranslate([0,0,0]), visualization=VRobotTool(graphicsData=graphicsToolList)),
                     referenceConfiguration = []) #referenceConfiguration created with 0s automatically
       
       
       #++++++++++++++++++++++++++
       #according numbering of links (0,1,2,...)
       linkNumbers={'None':-1}
       linkNumbers['base_link']=-1
       for cnt, link in enumerate(linkList):
           linkNumbers[link['name']] = cnt
           
       #++++++++++++++++++++++++++
       print('*** load model '+modelName+', nJoints=',numberOfJoints)
       for cnt, link in enumerate(linkList):
           jointType = None
           HT = link['preHT']
           #if cnt <= 3: print('pos =',HT2translation(HT))
   
           if len(link['graphicsDataList']) == 0: #no graphics loaded (no urdf, no pymeshlab)
               visualization=VRobotLink(linkColor=graphics.colorList[cnt])
           else:
               jointWidth=0.1 #graphical parameters
               jointRadius=0.06
               visualization=VRobotLink(graphicsData=link['graphicsDataList'],
                                        showMBSjoint=False,
                                        jointWidth=0.12,jointRadius=0.01)
           mass = link['mass']
           inertia = link['inertiaCOM']
           if mass == 0: #some robots have no mass/inertia => use some values to make it run
               if cnt == 0 and mass == 0:
                   print('robot has mass=0: assuming some values instead!')
               mass = 5/(1+2*cnt)
               inertia = InertiaSphere(mass, 0.25/(1+2*cnt)).InertiaCOM()
           # else:
           #     print('mass',cnt,'=',mass)
           mbsRobot.AddLink(RobotLink(mass=mass, 
                                      parent = linkNumbers[link['parentName']],
                                      COM=link['com'], 
                                      inertia=inertia,
                                      preHT=link['preHT'],
                                      jointType=link['jointType'],
                                      PDcontrol=(Pcontrol[cnt], Dcontrol[cnt]),
                                      visualization=visualization
                                      ))
   
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #configurations and trajectory
       q0 = np.zeros(numberOfJoints) #zero angle configuration
       q1 = np.zeros(numberOfJoints) 
       q2 = np.zeros(numberOfJoints) 
       q3 = np.zeros(numberOfJoints) 
       q0 = robotDataDict['staticJointValues']
       
       #this set also works with load control:
       q1b = [0.5*pi,0,       -0.25*pi,        0,0,0] #zero angle configuration
       q2b = [0,     pi/8,     pi*0.5, 0,pi/8, 0] #configuration 1
       q3b = [0.8*pi,-0.8*pi, -pi*0.5, 0.75*pi,-pi*0.4,pi*0.4] #configuration 2
       if 'UR' in modelName:
           q2b[1] *= -1
           q2b[2] *= -1
           q3b[2] *= 0.8
       if 'Puma560' in modelName:
           q2b[2] *= 0.5
           q3b[1] *= 0.1
           q3b[2] *= 0.25
       if 'Panda' in modelName:
           q3b[3] *= 0.3
           q3b[1] *= 0.5
       if 'vx' in modelName or 'wx' in modelName or 'px' in modelName:
           q3b[1] *= 0.5
           q3b[2] *= 0.5
       
       copyNJoints  = min(6, numberOfJoints )
       q1[0:copyNJoints ] = q1b[0:copyNJoints ]
       q2[0:copyNJoints ] = q2b[0:copyNJoints ]
       q3[0:copyNJoints ] = q3b[0:copyNJoints ]
           
       #trajectory generated with optimal acceleration profiles:
       trajectory = Trajectory(initialCoordinates=q0, initialTime=0)
       trajectory.Add(ProfileConstantAcceleration(q1,0.25))
       trajectory.Add(ProfileConstantAcceleration(q2,0.25))
       trajectory.Add(ProfileConstantAcceleration(q3,0.25))
       trajectory.Add(ProfileConstantAcceleration(q0,0.25))
       listTrajectory.append(trajectory)
       
       
       #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #build mbs mbsRobot model:
       robotDict = mbsRobot.CreateKinematicTree(mbs)
       oKT = robotDict['objectKinematicTree']
       nKT = robotDict['nodeGeneric']
       listKinematicTree.append(oKT)
       #if non-zero values chosen, set them here as well:
       mbs.SetNodeParameter(nKT, 'initialCoordinates', robotDataDict['staticJointValues'])
           
       
       #add sensors (only last sensor is kept)
       sJointRot = mbs.AddSensor(SensorBody(bodyNumber=oKT, 
                                            storeInternal=True, 
                                            outputVariableType=exu.OutputVariableType.Coordinates))
       
       sTorques = mbs.AddSensor(SensorBody(bodyNumber=oKT, storeInternal=True, 
                                             outputVariableType=exu.OutputVariableType.Force))
           
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #user function which is called only once per step, prescribe trajectory
   def PreStepUF(mbs, t):
   
       for cnt, oKT in enumerate(listKinematicTree):
           #set position and velocity in kinematic tree joints:
           [u,v,a] = listTrajectory[cnt].Evaluate(t)
           #note: here we would need to add the gear ratios to u and v, if they shall represent motor angles!!!
           mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector', u)
           mbs.SetObjectParameter(oKT, 'jointVelocityOffsetVector', v)
       
       return True
   
   mbs.SetPreStepUserFunction(PreStepUF)
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #assemble and simulate
   mbs.Assemble()
   
   SC.visualizationSettings.connectors.showJointAxes = True
   SC.visualizationSettings.connectors.jointAxesLength = 0.02
   SC.visualizationSettings.connectors.jointAxesRadius = 0.002
   
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.1
   SC.visualizationSettings.loads.show = False
   SC.visualizationSettings.bodies.kinematicTree.frameSize=0.12
   SC.visualizationSettings.openGL.multiSampling=4
       
   tEnd = 1.5
   stepSize = 0.001 #could be larger!
   
   #SC.renderer.DoIdleTasks()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = 0.005
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.005
   simulationSettings.solutionSettings.binarySolutionFile = True
   #simulationSettings.solutionSettings.writeSolutionToFile = False
   #simulationSettings.timeIntegration.simulateInRealtime = True
   
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
   
   SC.visualizationSettings.general.drawWorldBasis = True
   SC.visualizationSettings.general.autoFitScene=False
   SC.visualizationSettings.window.renderWindowSize=[1920,1200]
   SC.visualizationSettings.openGL.perspective=1
   # SC.visualizationSettings.openGL.shadow=0.25
   SC.visualizationSettings.openGL.light0ambient = 0.5
   SC.visualizationSettings.openGL.light0diffuse = 0.5
   SC.visualizationSettings.openGL.light0position = [2,-4,8,0]
   
   useGraphics = True
   
   if useGraphics:
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys['renderState'])
       SC.renderer.DoIdleTasks()
       
   mbs.SolveDynamic(simulationSettings, 
                    solverType=exu.DynamicSolverType.TrapezoidalIndex2, 
                    showHints=True)
   
   if useGraphics:
       SC.visualizationSettings.general.autoFitScene = False
       SC.renderer.Stop()
   
   if True: #set True to show animation after simulation
       mbs.SolutionViewer()
   
   
   #%%++++++++++++++++++++++++++++
   if False and not onlyShowModel:
       mbs.PlotSensor(sensorNumbers=sJointRot, components=np.arange(numberOfJoints), 
                       title='joint angles', closeAll=True)
       mbs.PlotSensor(sensorNumbers=sTorques, components=np.arange(numberOfJoints), 
                       title='joint torques')


