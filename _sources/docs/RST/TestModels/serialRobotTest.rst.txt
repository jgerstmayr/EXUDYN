
.. _testmodels-serialrobottest:

******************
serialRobotTest.py
******************

You can view and download this file on Github: `serialRobotTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/serialRobotTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test Example of a serial robot with redundant coordinates
   #           NOTE: more efficient version in serialRobotTSD.py and serialRobotKinematicTree.py using TorsionalSpringDamper
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-02-16
   # Revised:  2021-07-09
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.rigidBodyUtilities import *
   from exudyn.robotics import *
   from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration
   
   import numpy as np
   from numpy import linalg as LA
   
   useGraphics = True #without test
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
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #useGraphics = False
   if useGraphics:
       sensorWriteToFile = True
   else:
       sensorWriteToFile = False
   
   jointWidth=0.1
   jointRadius=0.06
   linkWidth=0.1
   
   graphicsBaseList = [graphics.Brick([0,0,-0.15], [0.4,0.4,0.1], graphics.color.grey)]
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
   
   newRobot.AddLink(RobotLink(mass=20, COM=[0,0,0], inertia=np.diag([1e-8,0.35,1e-8]), localHT = StdDH2HT([0,0,0,np.pi/2]), visualization=VRobotLink(linkColor=graphics.colorList[0])))
   newRobot.AddLink(RobotLink(mass=17.4, COM=[-0.3638, 0.006, 0.2275], inertia=np.diag([0.13,0.524,0.539]), localHT = StdDH2HT([0,0,0.4318,0]), visualization=VRobotLink(linkColor=graphics.colorList[1])))
   newRobot.AddLink(RobotLink(mass=4.8, COM=[-0.0203,-0.0141,0.07], inertia=np.diag([0.066,0.086,0.0125]), localHT = StdDH2HT([0,0.15,0.0203,-np.pi/2]), visualization=VRobotLink(linkColor=graphics.colorList[2])))
   newRobot.AddLink(RobotLink(mass=0.82, COM=[0,0.019,0], inertia=np.diag([0.0018,0.0013,0.0018]), localHT = StdDH2HT([0,0.4318,0,np.pi/2]), visualization=VRobotLink(linkColor=graphics.colorList[3])))
   newRobot.AddLink(RobotLink(mass=0.34, COM=[0,0,0], inertia=np.diag([0.0003,0.0004,0.0003]), localHT = StdDH2HT([0,0,0,-np.pi/2]), visualization=VRobotLink(linkColor=graphics.colorList[4])))
   newRobot.AddLink(RobotLink(mass=0.09, COM=[0,0,0.032], inertia=np.diag([0.00015,0.00015,4e-5]), localHT = StdDH2HT([0,0,0,0]), visualization=VRobotLink(linkColor=graphics.colorList[5])))
   
   
   # newRobot = Robot(gravity=[0,0,-9.81], referenceConfiguration = [])
   # newRobot.BuildFromDictionary(myRobot) #this allows conversion from old structure
   
   #assumption, as the bodies in the mbs have their COM at the reference position
   #for link in robot['links']:
   #    link['COM']=[0,0,0]
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #configurations and trajectory
   q0 = [0,0,0,0,0,0] #zero angle configuration
   
   q1 = [0,       np.pi/8, np.pi*0.25, 0,np.pi/8,0] #configuration 1
   q2 = [np.pi/2,-np.pi/8,-np.pi*0.125, 0,np.pi/4,0] #configuration 2
   #q2 = [np.pi*0.45,-np.pi*0.35,-np.pi*0.25, np.pi*0.10,np.pi*0.2,np.pi*0.3] #configuration 2
   #q2 = [pi/2,-pi/8,-pi*0.125,0,pi/4,pi/2] #configuration 2
   
   trajectory=Trajectory(q0, 0)
   if False: #tests for static torque compensation
       #trajectory.Add(ProfileConstantAcceleration(q0, 1)) #V1.2.37: torques at tEnd= 0.3773259482700045
       trajectory.Add(ProfileConstantAcceleration(q2, 0.25))
       trajectory.Add(ProfileConstantAcceleration(q2, 0.75))
   else: #standard test case:
       trajectory.Add(ProfileConstantAcceleration(q1, 0.25))
       trajectory.Add(ProfileConstantAcceleration(q2, 0.25))
       trajectory.Add(ProfileConstantAcceleration(q0, 0.5 ))
       trajectory.Add(ProfileConstantAcceleration(q0, 1e6))
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #test robot model
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #control parameters, per joint:
   Pcontrol = np.array([40000, 40000, 40000, 100, 100, 10])
   Dcontrol = np.array([400,   400,   100,   1,   1,   0.05])  #last value 0.05 gives less oscillations as compared to earlier values of 0.1!
   # Pcontrol = 0.01*Pcontrol #soft behavior
   
   
   #++++++++++++++++++++++++++++++++++++++++++++++++
   #base, graphics, object and marker:
   graphicsBaseList = [graphics.Brick([0,0,-0.2], [0.4,0.4,0.1], graphics.color.grey)]
   graphicsBaseList +=[graphics.Cylinder([0,0,0], [0.5,0,0], 0.0025, graphics.color.red)]
   graphicsBaseList +=[graphics.Cylinder([0,0,0], [0,0.5,0], 0.0025, graphics.color.green)]
   graphicsBaseList +=[graphics.Cylinder([0,0,0], [0,0,0.5], 0.0025, graphics.color.blue)]
   #oGround = mbs.AddObject(ObjectGround(referencePosition=list(HT2translation(Tcurrent)), 
   objectGround = mbs.AddObject(ObjectGround(referencePosition=HT2translation(newRobot.GetBaseHT()), 
                                        visualization=VObjectGround(graphicsData=graphicsBaseList)))
   
   #baseMarker; could also be a moving base!
   baseMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=objectGround, localPosition=[0,0,0]))
   
   
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #build mbs robot model:
   robotDict = newRobot.CreateRedundantCoordinateMBS(mbs, baseMarker=baseMarker)
       
   #   !!!!!IMPORTANT!!!!!:
   jointList = robotDict['jointList'] #must be stored there for the load user function
   
   unitTorques0 = robotDict['unitTorque0List'] #(left body)
   unitTorques1 = robotDict['unitTorque1List'] #(right body)
   
   loadList0 = robotDict['jointTorque0List'] #(left body)
   loadList1 = robotDict['jointTorque1List'] #(right body)
   #print(loadList0, loadList1)
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #control robot
   compensateStaticTorques = True
   def ComputeMBSstaticRobotTorques(newRobot):
       q=[]
       for joint in jointList:
           q += [mbs.GetObjectOutput(joint, exu.OutputVariableType.Rotation)[2]] #z-rotation
       HT=newRobot.JointHT(q)
       return newRobot.StaticTorques(HT)
   
   #user function which is called only once per step, speeds up simulation drastically
   def PreStepUF(mbs, t):
       if compensateStaticTorques:
           staticTorques = ComputeMBSstaticRobotTorques(newRobot)
       else:
           staticTorques = np.zeros(len(jointList))
       #compute load for joint number
       for i in range(len(jointList)):
           joint = i
           phi = mbs.GetObjectOutput(jointList[joint], exu.OutputVariableType.Rotation)[2] #z-rotation
           omega = mbs.GetObjectOutput(jointList[joint], exu.OutputVariableType.AngularVelocityLocal)[2] #z-angular velocity
   
           #[u,v,a] = MotionInterpolator(t, robotTrajectory, joint) #OLD, until V1.1.135
           [u,v,a] = trajectory.EvaluateCoordinate(t, joint)
           
       
           torque = -(Pcontrol[joint]*(phi-u) + Dcontrol[joint]*(omega-v)) #negative sign in feedback control!
           # torque = (Pcontrol[joint]*(phi+u) + Dcontrol[joint]*(omega+v)) #until V1.2.; but static Torque compensation is wrong in this case!
           torque -= staticTorques[joint] #add static torque compensation
           
           load0 = torque * unitTorques0[i] #includes sign and correct unit-torque vector
           load1 = torque * unitTorques1[i] #includes sign and correct unit-torque vector
           
       #     #write updated torque to joint loads, applied to left and right body
           mbs.SetLoadParameter(loadList0[i], 'loadVector', list(load0))
           mbs.SetLoadParameter(loadList1[i], 'loadVector', list(load1))
       
       return True
   
   mbs.SetPreStepUserFunction(PreStepUF)
   
   
   
   #add sensors:
   sJointRot = []
   sJointAngVel = []
   sJointTorque = []
   cnt = 0
   for jointLink in jointList:
       sJointRot += [mbs.AddSensor(SensorObject(objectNumber=jointLink, 
                                  storeInternal=True,#fileName="solution/joint" + str(cnt) + "Rot.txt",
                                  outputVariableType=exu.OutputVariableType.Rotation,
                                  writeToFile = sensorWriteToFile))]
       sJointAngVel += [mbs.AddSensor(SensorObject(objectNumber=jointLink, 
                                  storeInternal=True,#fileName="solution/joint" + str(cnt) + "AngVel.txt",
                                  outputVariableType=exu.OutputVariableType.AngularVelocityLocal,
                                  writeToFile = sensorWriteToFile))]
       cnt+=1
   
   cnt = 0
   for load0 in robotDict['jointTorque0List']:
       sJointTorque += [mbs.AddSensor(SensorLoad(loadNumber=load0,storeInternal=True,# fileName="solution/jointTorque" + str(cnt) + ".txt", 
                                                 writeToFile = sensorWriteToFile))]
       cnt+=1
   
   
   
   mbs.Assemble()
   #mbs.systemData.Info()
   
   SC.visualizationSettings.connectors.showJointAxes = True
   SC.visualizationSettings.connectors.jointAxesLength = 0.02
   SC.visualizationSettings.connectors.jointAxesRadius = 0.002
   
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.1
   SC.visualizationSettings.loads.show = False
   
   SC.visualizationSettings.openGL.multiSampling=4
       
   tEnd = 0.2 #0.2 for testing
   h = 0.001
   
   if useGraphics:
       tEnd = 0.2
       #tEnd = 1 #shows exactly static torques ComputeMBSstaticRobotTorques(newRobot) and desired angles (q2) at end
   
   #SC.renderer.DoIdleTasks()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = h
   simulationSettings.solutionSettings.sensorsWritePeriod = h
   simulationSettings.solutionSettings.writeSolutionToFile = useGraphics
   # simulationSettings.timeIntegration.simulateInRealtime = True
   # simulationSettings.timeIntegration.realtimeFactor = 0.25
   
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.displayComputationTime = False
   simulationSettings.displayStatistics = False
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   
   #simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5 #0.6 works well 
   
   simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
   
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics:
       SC.visualizationSettings.general.autoFitScene=False
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys['renderState'])
   
       
       mbs.SolutionViewer()
       SC.renderer.Stop()
   
   lastRenderState = SC.renderer.GetState() #store model view
   
   #compute final torques:
   measuredTorques=[]
   
   for cnt, sensorNumber in enumerate(sJointTorque):
       if useGraphics:
           exu.Print('sensor torque',cnt, '=', mbs.GetSensorValues(sensorNumber))
       measuredTorques += [1e-2*mbs.GetSensorValues(sensorNumber)[2]]
   
   if useGraphics:
       for cnt, sensorNumber in enumerate(sJointRot):
           exu.Print('sensor rot ',cnt, '=', mbs.GetSensorValues(sensorNumber))
   
   
   
   exu.Print("torques at tEnd=", VSum(measuredTorques))
   
   #add larger test tolerance for 32/64bits difference
   exudynTestGlobals.testError = (VSum(measuredTorques) - 0.7681856909852399)  #until 2022-04-21: 7680031232063571; until 2021-09-10: 76.8003123206452; until 2021-08-19 (changed robotics.py): 76.80031232091771; old controller: 77.12176106978085) #OLDER results: up to 2021-06-28: 0.7712176106955341; 2020-08-25: 77.13193176752571 (32bits),   2020-08-24: (64bits)77.13193176846507
   exudynTestGlobals.testResult = VSum(measuredTorques)   
   
   #exu.Print('error=', exudynTestGlobals.testError)
   
   if useGraphics:
       
       
       mbs.PlotSensor(sJointTorque, components=2, closeAll=True, yLabel='joint torques (Nm)', title='joint torques')
   
       mbs.PlotSensor(sJointRot, components=2, yLabel='joint angles (rad)', title='joint angles')
   
       #V1.2.40, P39: with D[-1]=0.05: since 2022-04-22:
       # sensor torque 0 = [  0.           0.         -12.68901871]
       # sensor torque 1 = [-0.         -0.         76.45031947]
       # sensor torque 2 = [-0.         -0.         12.97010176]
       # sensor torque 3 = [ 0.00000000e+00  0.00000000e+00 -8.56924875e-05]
       # sensor torque 4 = [-0.         -0.          0.08725323]
       # sensor torque 5 = [ 0.0000000e+00  0.0000000e+00 -9.5769297e-07]
       # sensor rot  0 = [-1.04899069e-15  3.05694259e-19 -3.19543456e-04]
       # sensor rot  1 = [-4.57411886e-14 -1.77599905e-14  3.63829923e-01]
       # sensor rot  2 = [ 4.68252006e-14 -1.78150803e-14  7.25569701e-01]
       # sensor rot  3 = [-1.07990283e-13  1.95010674e-13  2.42757634e-07]
       # sensor rot  4 = [ 1.07852866e-13 -2.06184741e-13  3.63658987e-01]
       # sensor rot  5 = [ 1.84235149e-16 -6.26054764e-13 -5.07549126e-08]
       # torques at tEnd= 0.7681856909852399 
   
       #V1.2.40, P39: with D[-1]=0.1:
       # sensor torque 0 = [  0.           0.         -12.68901812]
       # sensor torque 1 = [-0.        -0.        76.4503195]
       # sensor torque 2 = [-0.         -0.         12.97010177]
       # sensor torque 3 = [ 0.0000000e+00  0.0000000e+00 -8.9097007e-05]
       # sensor torque 4 = [-0.         -0.          0.08725323]
       # sensor torque 5 = [-0.00000000e+00 -0.00000000e+00  1.09612342e-05]
       # sensor rot  0 = [-1.04899069e-15  3.32799313e-19 -3.19543454e-04]
       # sensor rot  1 = [-4.59632332e-14 -1.77794520e-14  3.63829923e-01]
       # sensor rot  2 = [ 4.71643413e-14 -1.78734434e-14  7.25569701e-01]
       # sensor rot  3 = [-1.07609037e-13  1.95343741e-13  2.42965135e-07]
       # sensor rot  4 = [ 1.07471637e-13 -2.06279100e-13  3.63658987e-01]
       # sensor rot  5 = [ 1.84106879e-16 -6.25874352e-13  8.28833899e-08]
       # torques at tEnd= 0.7681857824086907 
   
       #V1.2.37, P37: ==> uses wrong static torque compensation
       # sensor torque 0 = [  0.           0.         -12.67938332]
       # sensor torque 1 = [-0.         -0.         76.42613004]
       # sensor torque 2 = [-0.         -0.         12.96641046]
       # sensor torque 3 = [ 0.00000000e+00  0.00000000e+00 -8.93114211e-05]
       # sensor torque 4 = [-0.         -0.          0.08723331]
       # sensor torque 5 = [-0.0000000e+00 -0.0000000e+00  1.1143982e-05]
       # sensor rot  0 = [ 1.60812265e-16 -2.71050543e-20  3.19257214e-04]
       # sensor rot  1 = [ 5.06049354e-14 -1.91882065e-14 -3.63432530e-01]
       # sensor rot  2 = [-4.07000942e-14  3.58791681e-14 -7.25182770e-01]
       # sensor rot  3 = [ 9.91554982e-13  4.63740157e-13 -2.41708605e-07]
       # sensor rot  4 = [-9.26617234e-13  3.52539928e-13 -3.63103888e-01]
       # sensor rot  5 = [ 9.21913969e-18  3.74700271e-16 -8.34127775e-08]
       # torques at tEnd= 0.7680031232065901 
       
   


