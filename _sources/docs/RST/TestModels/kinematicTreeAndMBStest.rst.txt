
.. _testmodels-kinematictreeandmbstest:

**************************
kinematicTreeAndMBStest.py
**************************

You can view and download this file on Github: `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN python utility library
   #
   # Details:  several tests for class Robot and kinematicTree; tests compare minimum coordinate and redundant coordinate formulations
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-05-29
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.FEM import *
   
   import numpy as np
   
   useGraphics = True
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
   
   from math import pi, sin, cos#, sqrt
   from copy import copy, deepcopy
   from exudyn.rigidBodyUtilities import Skew, Skew2Vec
   from exudyn.robotics import *
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   useGraphics = False
   performTest = True
   
   printSensors = True
   #useGraphics = False
   #exudynTestGlobals.testError = 0. #not filled, done via result
   exudynTestGlobals.testResult = 0. #values added up
   
   useMBS = True
   useKinematicTree = True
   
   # case = '3Dmechanism'
   case = 'invertedPendulum'
   #case = 'treeStructure'
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #this function is used to compare class Robot internal structures with ObjectKinematicTree sensor
   compareKT = False
   jacobian = np.eye(3)
   def CompareKinematicTreeAndRobot(newRobot, locPos):
       #compare with Python class Robot functions for validation:
       #from exudyn.kinematicTree import *
       global jacobian
       #get coordinates (), INCLUDES reference values:
       q = mbs.GetObjectOutput(oKT,exu.OutputVariableType.Coordinates)
       q_t = mbs.GetObjectOutput(oKT,exu.OutputVariableType.Coordinates_t)
       #print('q=',q)
   
       baseHT = newRobot.GetBaseHT()
       allHT = newRobot.JointHT(q)
       
       print('compare solution for n links=', newRobot.NumberOfLinks())
       for i in range(newRobot.NumberOfLinks()):
           #link = newRobot.GetLink(i)
           #compare position; we need a sensor to access variables
           s=mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=i, localPosition=locPos,
                                               outputVariableType=exu.OutputVariableType.Position))
           mbs.systemIsConsistent = True #adding new sensor requires re-assemble, which is not done here
           pRobot = HT2translation(allHT[i]) + HT2rotationMatrix(allHT[i]) @ locPos
           pKT = mbs.GetSensorValues(s)
           # print('joint pos: Robot=', HT2translation(allHT[i]) + HT2rotationMatrix(allHT[i]) @ locPos, 
           #       ', KT=', mbs.GetSensorValues(s))
           print('position diff=', (pRobot-pKT).round(15))
           
           #compare velocity
           sVel=mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=i, localPosition=locPos,
                                               outputVariableType=exu.OutputVariableType.Velocity))
           sOmega=mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=i, localPosition=locPos,
                                               outputVariableType=exu.OutputVariableType.AngularVelocity))
           mbs.systemIsConsistent = True #adding new sensor requires re-assemble, which is not done here
   
           jacobian = newRobot.Jacobian(allHT[0:i+1], toolPosition=HT2translation(allHT[i]@HTtranslate(locPos)), mode='all')
           #print('jac=', jacobian.round(3))
   
           vOmegaRobot = jacobian @ q_t[0:i+1]
           vOmegaKT = list(mbs.GetSensorValues(sVel)) + list(mbs.GetSensorValues(sOmega))
           #print('vel: Robot=', vOmegaRobot, ', KT=', vOmegaKT)
           print('vel diff=', (vOmegaRobot-vOmegaKT).round(14))
           
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #some spatial mechanism with revolute and prismatic joints:
   
   if case == '3Dmechanism' or performTest:
       mbs.Reset()
       gGround =  graphics.CheckerBoard(point= [0,0,-2], size = 4)
       objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0],
                                                 visualization=VObjectGround(graphicsData=[gGround])))
       baseMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=objectGround, localPosition=[0,0,0]))
       
       L = 0.5 #length
       w = 0.1 #width of links
       pControl = 200*0
       dControl = pControl*0.02
       
       # pControl=None
       # dControl=None
       
       gravity3D = [0,-9.81,0]
       #gravity3D = [0,-9.81,0] #note that this system is extremely sensitive to disturbances: adding 1e-15 will change solution by 1e-7
       graphicsBaseList = [graphics.Brick(size=[L*4, 0.8*w, 0.8*w], color=graphics.color.grey)] #rail
       
       newRobot = Robot(gravity=gravity3D,
                     base = RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList)),
                     tool = RobotTool(HT=HTtranslate([0,0.5*L,0]), visualization=VRobotTool(graphicsData=[
                         graphics.Brick(size=[w, L, w], color=graphics.color.orange)])),
                     referenceConfiguration = []) #referenceConfiguration created with 0s automatically
       
       #cart:
       Jlink = InertiaCuboid(density=5000, sideLengths=[L,w,w]) #w.r.t. reference center of mass
       link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                        jointType='Px', preHT=HT0(), 
                        PDcontrol=(pControl, dControl),
                        visualization=VRobotLink(linkColor=graphics.color.lawngreen))
       newRobot.AddLink(link)
       
       if True:
           Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
           Jlink = Jlink.Translated([0,0.5*L,0])
           link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                            jointType='Rz', preHT=HT0(), 
                            PDcontrol=(pControl, dControl),
                            visualization=VRobotLink(linkColor=graphics.color.blue))
           newRobot.AddLink(link)
       
           
           if True:
               Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
               Jlink = Jlink.Translated([0,0.5*L,0])
               link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                                # jointType='Rz', preHT=HTtranslateY(L),
                                jointType='Rz', preHT=HTtranslateY(L)@HTrotateY(0.25*pi),
                                PDcontrol=(pControl, dControl), 
                                visualization=VRobotLink(linkColor=graphics.color.red))
               newRobot.AddLink(link)
               
           if False:
               Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
               Jlink = Jlink.Translated([0,0.5*L,0])
               link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                                jointType='Px', preHT=HT0(), 
                                PDcontrol=(pControl, dControl),
                                visualization=VRobotLink(linkColor=graphics.color.lawngreen))
               newRobot.AddLink(link)
       
           if True:
               Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
               Jlink = Jlink.Translated([0,0.5*L,0])
               link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                                jointType='Rz', preHT=HTtranslateY(L)@HTrotateZ(-0.5*pi),
                                PDcontrol=(pControl, dControl), 
                                #visualization=VRobotLink(linkColor=graphics.color.brown))
                                visualization=VRobotLink(linkColor=[-1,-1,-1,1]))
               newRobot.AddLink(link)
       
           if False:
               Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
               Jlink = Jlink.Translated([0,0.5*L,0])
               link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                                jointType='Rz', preHT=HTtranslateY(L)@HTrotateZ(-0.5*pi),
                                PDcontrol=(pControl, dControl), 
                                visualization=VRobotLink(linkColor=graphics.color.brown))
               newRobot.AddLink(link)
       
               Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
               Jlink = Jlink.Translated([0,0.5*L,0])
               link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                                jointType='Rz', preHT=HTtranslateY(L)@HTrotateZ(-0.5*pi),
                                PDcontrol=(pControl, dControl), 
                                visualization=VRobotLink(linkColor=graphics.color.brown))
               newRobot.AddLink(link)
       
               Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
               Jlink = Jlink.Translated([0,0.5*L,0])
               link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                                jointType='Rz', preHT=HTtranslateY(L)@HTrotateZ(-0.5*pi),
                                PDcontrol=(pControl, dControl), 
                                visualization=VRobotLink(linkColor=graphics.color.brown))
               newRobot.AddLink(link)
       
       sMBS = []
       locPos = [0.1,0.2,0.3]
       # locPos = [0,0,0]
       nLinks = newRobot.NumberOfLinks()
       if useMBS:
           robDict = newRobot.CreateRedundantCoordinateMBS(mbs=mbs, baseMarker=baseMarker, createJointTorqueLoads=False)
           bodies = robDict['bodyList']
       
           sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[0], localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Position))]
           sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[0], localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Acceleration))]
           sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[1], localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Acceleration))]
           sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[2], localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Acceleration))]
       
           sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Position))]
           sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Rotation))]
           sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Velocity))]
           sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.AngularVelocity))]
           sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.AngularVelocityLocal))]
           sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Acceleration))]
           sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.AngularAcceleration))]
       
       sKT = []
       if useKinematicTree:
           dKT = newRobot.CreateKinematicTree(mbs)
           oKT = dKT['objectKinematicTree']
       
           sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=0, localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Position))]
           sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=0, localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Acceleration))]
           sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=1, localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Acceleration))]
           sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=2, localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Acceleration))]
       
           sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Position))]
           sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Rotation))]
           sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Velocity))]
           sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.AngularVelocity))]
           sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.AngularVelocityLocal))]
           sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Acceleration))]
           sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.AngularAcceleration))]
       
       
       sTitles = [
       'Position link 0',
       'Acceleration link 0',
       'Acceleration link 1',
       'Acceleration link 2',
       'Tip Position',
       'Tip Rotation',
       'Tip Velocity',
       'Tip AngularVelocity',
       'Tip AngularVelocityLocal',
       'Tip Acceleration',
       'Tip AngularAcceleration',
       ]
       
       #exu.Print(mbs)
       mbs.Assemble()
       
       simulationSettings = exu.SimulationSettings()
       
       tEnd = 0.5
       if not performTest:
           tEnd = 2*0.5
   
       h = 1e-3 #0.1
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       # simulationSettings.timeIntegration.numberOfSteps = 1#int(tEnd/h)
       # simulationSettings.timeIntegration.endTime = h*1#tEnd
       simulationSettings.solutionSettings.solutionWritePeriod = 0.01
       simulationSettings.solutionSettings.sensorsWritePeriod = 0.001
       simulationSettings.timeIntegration.verboseMode = 1
       #simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
       simulationSettings.timeIntegration.newton.useModifiedNewton=True
       
       # simulationSettings.displayComputationTime = True
       simulationSettings.displayStatistics = True
       # simulationSettings.linearSolverType=exu.LinearSolverType.EigenSparse
       
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.95 #SHOULD work with 0.9 as well
       
       SC.visualizationSettings.general.autoFitScene=False
       SC.visualizationSettings.window.renderWindowSize = [1600,1200]
       SC.visualizationSettings.general.drawCoordinateSystem=True
       SC.visualizationSettings.general.drawWorldBasis=True
       SC.visualizationSettings.openGL.multiSampling=4
       SC.visualizationSettings.nodes.showBasis = True
       SC.visualizationSettings.nodes.basisSize = 0.5
       if useGraphics:
           SC.renderer.Start()
           if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
       
           SC.renderer.DoIdleTasks() #press space to continue
   
       # mbs.SolveDynamic(simulationSettings, solverType = exu.DynamicSolverType.ExplicitMidpoint)
       mbs.SolveDynamic(simulationSettings)
           
       if useGraphics: #use this to reload the solution and use SolutionViewer
           #sol = LoadSolutionFile('coordinatesSolution.txt')
           
           mbs.SolutionViewer() #can also be entered in IPython ...
       
       if useGraphics:
           SC.renderer.DoIdleTasks()
           SC.renderer.Stop() #safely close rendering window!
       
       
       if len(sMBS) == len(sKT):
           if useGraphics:
               
               mbs.PlotSensor(closeAll=True)
               
               for i in range(len(sMBS)):
                   mbs.PlotSensor(sensorNumbers=[sMBS[i]]*3+[sKT[i]]*3, components=[0,1,2]*2, title=sTitles[i])
           else:
               u = 0.
               for i in range(len(sMBS)):
                   v = mbs.GetSensorValues(sMBS[i])
                   if printSensors:
                       exu.Print('sensor MBS '+str(i)+'=',list(v))
                   u += np.linalg.norm(v)
                   v = mbs.GetSensorValues(sKT[i])
                   if printSensors:
                       exu.Print('sensor KT '+str(i)+' =',list(v))
                   u += np.linalg.norm(v)
   
               exu.Print("solution of kinematicTreeAndMBStest 1=", u)
               exudynTestGlobals.testResult += u
   
           if compareKT:
               CompareKinematicTreeAndRobot(newRobot, [0.1,0.3,0.2])
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if case == 'invertedPendulum' or performTest:
       mbs.Reset()
       gGround =  graphics.CheckerBoard(point= [0,0,-2], size = 12)
       objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0],
                                                 visualization=VObjectGround(graphicsData=[gGround])))
       baseMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=objectGround, localPosition=[0,0,0]))
       
       L = 0.5 #length
       w = 0.1 #width of links
       pControl = 200*0
       dControl = pControl*0.02
       
       gravity3D = [10*0,-9.81,0]
       graphicsBaseList = [graphics.Brick(size=[L*4, 0.8*w, 0.8*w], color=graphics.color.grey)] #rail
       
       newRobot = Robot(gravity=gravity3D,
                     base = RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList)),
                     tool = RobotTool(HT=HTtranslate([0,0.5*L,0]), visualization=VRobotTool(graphicsData=[
                         graphics.Brick(size=[w, L, w], color=graphics.color.orange)])),
                     referenceConfiguration = []) #referenceConfiguration created with 0s automatically
       
       #cart:
       Jlink = InertiaCuboid(density=5000, sideLengths=[L,w,w]) #w.r.t. reference center of mass
       link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                        jointType='Px', preHT=HT0(), 
                        PDcontrol=(pControl, dControl),
                        visualization=VRobotLink(linkColor=graphics.color.lawngreen))
       newRobot.AddLink(link)
   
       nChainLinks = 5
       
       for i in range(nChainLinks):
           Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
           Jlink = Jlink.Translated([0,0.5*L,0])
           preHT = HT0()
           if i > 0:
               preHT = HTtranslateY(L)
   
           link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                            jointType='Rz', preHT=preHT, 
                            PDcontrol=(pControl, dControl),
                            visualization=VRobotLink(linkColor=graphics.color.blue))
           newRobot.AddLink(link)
       
       newRobot.referenceConfiguration[0] = 0.5*0
       # for i in range(nChainLinks):
       #     newRobot.referenceConfiguration[i+1] = (2*pi/360) * 5 
       newRobot.referenceConfiguration[1] = -(2*pi/360) * 5 #-0.5*pi
       # newRobot.referenceConfiguration[2] = (2*pi/360) * 12 #-0.5*pi
           
       
       sMBS = []
       # locPos = [0.1,0.2,0.3]
       locPos = [0,0,0]
       nLinks = newRobot.NumberOfLinks()
       if useMBS:
           robDict = newRobot.CreateRedundantCoordinateMBS(mbs=mbs, baseMarker=baseMarker, createJointTorqueLoads=False)
           bodies = robDict['bodyList']
       
           sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Position))]
       
       sKT = []
       if useKinematicTree:
           dKT = newRobot.CreateKinematicTree(mbs)
           oKT = dKT['objectKinematicTree']
           
           sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Position))]
           
       #exu.Print(mbs)
       mbs.Assemble()
       
       simulationSettings = exu.SimulationSettings()
   
       tEnd = 0.5
       if not performTest:
           tEnd = 0.5
       h = 1e-3 #0.1
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       # simulationSettings.timeIntegration.numberOfSteps = 1#int(tEnd/h)
       # simulationSettings.timeIntegration.endTime = h*1#tEnd
       simulationSettings.solutionSettings.solutionWritePeriod = 0.01
       simulationSettings.solutionSettings.sensorsWritePeriod = 0.001*10
       simulationSettings.timeIntegration.verboseMode = 1
       #simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
       simulationSettings.timeIntegration.newton.useModifiedNewton=True
       
       # simulationSettings.displayComputationTime = True
       # simulationSettings.linearSolverType=exu.LinearSolverType.EigenSparse
       
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.95 #SHOULD work with 0.9 as well
       
       SC.visualizationSettings.general.autoFitScene=False
       SC.visualizationSettings.window.renderWindowSize = [1600,1200]
       SC.visualizationSettings.general.drawCoordinateSystem=True
       SC.visualizationSettings.general.drawWorldBasis=True
       SC.visualizationSettings.openGL.multiSampling=4
       SC.visualizationSettings.nodes.showBasis = True
       SC.visualizationSettings.nodes.basisSize = 0.5
       if useGraphics:
   
           SC.renderer.Start()
           if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
       
           SC.renderer.DoIdleTasks() #press space to continue
   
       # mbs.SolveDynamic(simulationSettings, solverType = exu.DynamicSolverType.ExplicitMidpoint)
       mbs.SolveDynamic(simulationSettings)
           
       if useGraphics: #use this to reload the solution and use SolutionViewer
           #sol = LoadSolutionFile('coordinatesSolution.txt')
           
           mbs.SolutionViewer() #can also be entered in IPython ...
       
       if useGraphics:
           SC.renderer.DoIdleTasks()
           SC.renderer.Stop() #safely close rendering window!
       else:
           #check results for test suite:
           u = 0.
           for i in range(len(sMBS)):
               v = mbs.GetSensorValues(sMBS[i])
               if printSensors:
                   exu.Print('sensor MBS '+str(i)+'=',v)
               u += np.linalg.norm(v)
               v = mbs.GetSensorValues(sKT[i])
               if printSensors:
                   exu.Print('sensor KT '+str(i)+' =',v)
               u += np.linalg.norm(v)
       
           exu.Print("solution of kinematicTreeAndMBStest 2=", u)
           exudynTestGlobals.testResult += u
   
           if compareKT:
               # CompareKinematicTreeAndRobot(newRobot, [0.1,0.3,0.2])
               CompareKinematicTreeAndRobot(newRobot, [0.,0.,0.])
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if case == 'treeStructure' or performTest:
       mbs.Reset()
       gGround =  graphics.CheckerBoard(point= [0,0,-2], size = 12)
       objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0],
                                                 visualization=VObjectGround(graphicsData=[gGround])))
       baseMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=objectGround, localPosition=[0,0,0]))
       
       L = 0.5 #length
       w = 0.1 #width of links
       pControl = 200*0
       dControl = pControl*0.02
       
       gravity3D = [10*0,-9.81,0]
       graphicsBaseList = [graphics.Brick(size=[L*4, 0.8*w, 0.8*w], color=graphics.color.grey)] #rail
       
       newRobot = Robot(gravity=gravity3D,
                     base = RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList)),
                     #tool = RobotTool(HT=HTtranslate([0,0.5*L,0]), visualization=VRobotTool(graphicsData=[graphics.Brick(size=[w, L, w], color=graphics.color.orange)])),
                     referenceConfiguration = []) #referenceConfiguration created with 0s automatically
       
       #cart:
       Jlink = InertiaCuboid(density=5000, sideLengths=[L,w,w]) #w.r.t. reference center of mass
       link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                        jointType='Px', preHT=HT0(),
                        parent = -1,
                        PDcontrol=(pControl, dControl),
                        visualization=VRobotLink(linkColor=graphics.color.lawngreen))
       rootLink = newRobot.AddLink(link)
   
       nChainLinks = 5
       
       parentLink = rootLink
       for i in range(nChainLinks):
           Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
           Jlink = Jlink.Translated([0,0.5*L,0])
           preHT = HTtranslateX(0.5*L)
           if i > 0:
               preHT = HTtranslateY(L)@HTrotateZ(-5*(2*pi/360))
   
           link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                            jointType='Rz', preHT=preHT, 
                            parent = parentLink,
                            PDcontrol=(pControl, dControl),
                            visualization=VRobotLink(linkColor=graphics.color.blue))
           parentLink = newRobot.AddLink(link)
       
       parentLink = rootLink
       for i in range(nChainLinks):
           Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
           Jlink = Jlink.Translated([0,0.5*L,0])
           preHT = HTtranslateX(-0.5*L)
           if i > 0:
               preHT = HTtranslateY(L)@HTrotateZ(5*(2*pi/360))
   
           link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                            jointType='Rz', preHT=preHT, 
                            parent = parentLink,
                            PDcontrol=(pControl, dControl),
                            visualization=VRobotLink(linkColor=graphics.color.blue))
           parentLink = newRobot.AddLink(link)
       
       #newRobot.referenceConfiguration[0] = 0.5*0
       # for i in range(nChainLinks):
       #     newRobot.referenceConfiguration[i+1] = (2*pi/360) * 5 
       #newRobot.referenceConfiguration[1] = -(2*pi/360) * 5 #-0.5*pi
           
       
       sMBS = []
       # locPos = [0.1,0.2,0.3]
       locPos = [0,0,0]
       nLinks = newRobot.NumberOfLinks()
       if useMBS:
           robDict = newRobot.CreateRedundantCoordinateMBS(mbs=mbs, baseMarker=baseMarker, createJointTorqueLoads=False)
           bodies = robDict['bodyList']
       
           sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Position))]
       
       sKT = []
       if useKinematicTree:
           dKT = newRobot.CreateKinematicTree(mbs)
           oKT = dKT['objectKinematicTree']
           
           sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Position))]
           
       #exu.Print(mbs)
       mbs.Assemble()
       
       simulationSettings = exu.SimulationSettings()
       
       tEnd = 0.25
       if not performTest:
           tEnd = 5
       h = 1e-3 #0.1
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       # simulationSettings.timeIntegration.numberOfSteps = 1#int(tEnd/h)
       # simulationSettings.timeIntegration.endTime = h*1#tEnd
       simulationSettings.solutionSettings.solutionWritePeriod = 0.01
       simulationSettings.solutionSettings.sensorsWritePeriod = 0.001*10
       simulationSettings.timeIntegration.verboseMode = 1
       #simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
       simulationSettings.timeIntegration.newton.useModifiedNewton=True
       
       # simulationSettings.displayComputationTime = True
       # simulationSettings.linearSolverType=exu.LinearSolverType.EigenSparse
       
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.95 #SHOULD work with 0.9 as well
       
       SC.visualizationSettings.general.autoFitScene=False
       SC.visualizationSettings.window.renderWindowSize = [1600,1200]
       SC.visualizationSettings.general.drawCoordinateSystem=True
       SC.visualizationSettings.general.drawWorldBasis=True
       SC.visualizationSettings.openGL.multiSampling=4
       SC.visualizationSettings.nodes.showBasis = True
       SC.visualizationSettings.nodes.basisSize = 0.5
       if useGraphics:
   
           SC.renderer.Start()
           if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
       
           SC.renderer.DoIdleTasks() #press space to continue
   
       # mbs.SolveDynamic(simulationSettings, solverType = exu.DynamicSolverType.ExplicitMidpoint)
       mbs.SolveDynamic(simulationSettings)
           
       if useGraphics: #use this to reload the solution and use SolutionViewer
           #sol = LoadSolutionFile('coordinatesSolution.txt')
           
           mbs.SolutionViewer() #can also be entered in IPython ...
       
       if useGraphics:
           SC.renderer.DoIdleTasks()
           SC.renderer.Stop() #safely close rendering window!
       else:
           #check results for test suite:
           u = 0.
           for i in range(len(sMBS)):
               v = mbs.GetSensorValues(sMBS[i])
               if printSensors:
                   exu.Print('sensor MBS '+str(i)+'=',v)
               u += np.linalg.norm(v)
               v = mbs.GetSensorValues(sKT[i])
               if printSensors:
                   exu.Print('sensor KT '+str(i)+' =',v)
               u += np.linalg.norm(v)
       
           exu.Print("solution of kinematicTreeAndMBStest 3=", u)
           exudynTestGlobals.testResult += u
           
               
           if compareKT:
               CompareKinematicTreeAndRobot(newRobot, [0.1,0.3,0.2])
   
   exudynTestGlobals.testResult *= 1e-7 #result is too sensitive to small (1e-15) disturbances, so different results for 32bits and linux
   exu.Print("solution of kinematicTreeAndMBStest all=", exudynTestGlobals.testResult)


