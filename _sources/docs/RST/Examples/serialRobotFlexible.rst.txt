
.. _examples-serialrobotflexible:

**********************
serialRobotFlexible.py
**********************

You can view and download this file on Github: `serialRobotFlexible.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/serialRobotFlexible.py>`_

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
   from exudyn.FEM import *
   
   import numpy as np
   from numpy import linalg as LA
   from math import pi
   import sys
   import time
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   sensorWriteToFile = True
   
   fileNames = ['testData/netgenRobotBase',
                'testData/netgenRobotArm0',
                'testData/netgenRobotArm1',
                ] #for load/save of FEM data
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   gravity=[0,0,-9.81]
   #geometry, arm lengths:t
   L = [0.075,0.4318,0.15,0.4318]
   W = [0,0,0.015,0]
   rArm = 0.025 #radius arm
   rFlange = 0.05 #radius of flange
   meshSize = rArm*0.5
   meshOrder = 2 #2 is more accurate!
   useFlexBody = False 
   
   Lbase = 0.3
   flangeBaseR = 0.05 #socket of base radius
   flangeBaseL = 0.05 #socket of base length
   rBase = 0.08
   tBase = 0.01 #wall thickness
   
   #standard:
   nModes = 8
   
   rho = 1000
   Emodulus=1e9 #steel: 2.1e11
   nu=0.3
   dampingK = 1e-2 #stiffness proportional damping
   
   nFlexBodies = 1*int(useFlexBody)
   femList = [None]*nFlexBodies
   
   def GetCylinder(p0, axis, length, radius):
       pnt0 = Pnt(p0[0], p0[1], p0[2])
       pnt1 = pnt0 + Vec(axis[0]*length, axis[1]*length, axis[2]*length)
       cyl = Cylinder(pnt0, pnt1, radius)
       plane0 = Plane (pnt0, Vec(-axis[0], -axis[1], -axis[2]) )
       plane1 = Plane (pnt1, Vec(axis[0], axis[1], axis[2]) )
       return cyl*plane0*plane1
   
   
   fb=[] #flexible bodies list of dictionaries
   fb+=[{'p0':[0,0,-Lbase], 'p1':[0,0,0], 'axis0':[0,0,1], 'axis1':[0,0,1]}] #defines flanges
   
   fes = None
   #create flexible bodies
   #requires netgen / ngsolve
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   if True and useFlexBody: #needs netgen/ngsolve to be installed to compute mesh, see e.g.: https://github.com/NGSolve/ngsolve/releases
       femList[0] = FEMinterface()
       import sys
       #adjust path to your ngsolve installation (if not added to global path)
       sys.path.append('C:/ProgramData/ngsolve/lib/site-packages') 
   
       import ngsolve as ngs
       import netgen
       from netgen.meshing import *
   
       from netgen.geom2d import unit_square
       #import netgen.libngpy as libng
       from netgen.csg import *
       
       
       #++++++++++++++++++++++++++++++++++++++++++++++++
       #flange
       geo = CSGeometry()
   
       
       geo.Add(GetCylinder(fb[0]['p0'], fb[0]['axis0'], Lbase-flangeBaseL, rBase) - 
               GetCylinder([0,0,-Lbase+tBase], [0,0,1], Lbase-2*tBase-flangeBaseL, rBase-tBase) + 
               GetCylinder([0,0,-flangeBaseL-tBase*0.5], fb[0]['axis1'], flangeBaseL+tBase*0.5, flangeBaseR))
   
       print('start meshing')
       mesh = ngs.Mesh( geo.GenerateMesh(maxh=meshSize))
       mesh.Curve(1)
       print('finished meshing')
   
       if False: #set this to true, if you want to visualize the mesh inside netgen/ngsolve
           # import netgen
           import netgen.gui
           ngs.Draw(mesh)
           for i in range(10000000):
               netgen.Redraw() #this makes the window interactive
               time.sleep(0.05)
   
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #Use fem to import FEM model and create FFRFreducedOrder object
       [bfM, bfK, fes] = femList[0].ImportMeshFromNGsolve(mesh, density=rho, youngsModulus=Emodulus, poissonsRatio=nu, meshOrder=meshOrder)
       femList[0].SaveToFile(fileNames[0])
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # sys.exit()
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #compute flexible modes
   
   for i in range(nFlexBodies):
       fem = femList[i]
       fem.LoadFromFile(fileNames[i])
       
       nodesPlane0 = fem.GetNodesInPlane(fb[i]['p0'], fb[i]['axis0'])
       # print('body'+str(i)+'nodes0=', nodesPlane0)
       lenNodesPlane0 = len(nodesPlane0)
       weightsPlane0 = np.array((1./lenNodesPlane0)*np.ones(lenNodesPlane0))
       
       nodesPlane1  = fem.GetNodesInPlane(fb[i]['p1'], fb[i]['axis1'])
       # print('body'+str(i)+'nodes1=', nodesPlane1)
       lenNodesPlane1 = len(nodesPlane1)
       weightsPlane1 = np.array((1./lenNodesPlane1)*np.ones(lenNodesPlane1))
       
       boundaryList = [nodesPlane0, nodesPlane1] 
       
       print("nNodes=",fem.NumberOfNodes())
       
       print("compute flexible modes... ")
       start_time = time.time()
       fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryList, 
                                         nEigenModes=nModes, 
                                         useSparseSolver=True,
                                         computationMode = HCBstaticModeSelection.RBE2)
       print("compute modes needed %.3f seconds" % (time.time() - start_time))
               
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #compute stress modes for postprocessing (inaccurate for coarse meshes, just for visualization):
       if fes != None:
           mat = KirchhoffMaterial(Emodulus, nu, rho)
           varType = exu.OutputVariableType.StressLocal
           #varType = exu.OutputVariableType.StrainLocal
           print("ComputePostProcessingModes ... (may take a while)")
           start_time = time.time()
           
           #without NGsolve, but only for linear elements
           # fem.ComputePostProcessingModes(material=mat, 
           #                                outputVariableType=varType)
           fem.ComputePostProcessingModesNGsolve(fes, material=mat, 
                                          outputVariableType=varType)
   
           print("   ... needed %.3f seconds" % (time.time() - start_time))
           # SC.visualizationSettings.contour.reduceRange=False
           SC.visualizationSettings.contour.outputVariable = varType
           SC.visualizationSettings.contour.outputVariableComponent = -1 #x-component
       else:
           SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
           SC.visualizationSettings.contour.outputVariableComponent = 1 
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       print("create CMS element ...")
       cms = ObjectFFRFreducedOrderInterface(fem)
       
       objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                               initialVelocity=[0,0,0], 
                                               initialAngularVelocity=[0,0,0],
                                               stiffnessProportionalDamping=dampingK,
                                               gravity=gravity,
                                               color=[0.1,0.9,0.1,1.],
                                               )
       
       
       #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
       #animate modes
       if False:
           from exudyn.interactive import AnimateModes
           mbs.Assemble()
           SC.visualizationSettings.nodes.show = False
           SC.visualizationSettings.openGL.showFaceEdges = True
           SC.visualizationSettings.openGL.multiSampling=4
           #SC.visualizationSettings.window.renderWindowSize = [1600,1080]
           # SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
           # SC.visualizationSettings.contour.outputVariableComponent = 0 #component
           
           
           #%%+++++++++++++++++++++++++++++++++++++++
           #animate modes of ObjectFFRFreducedOrder (only needs generic node containing modal coordinates)
           SC.visualizationSettings.general.autoFitScene = False #otherwise, model may be difficult to be moved
           
           nodeNumber = objFFRF['nGenericODE2'] #this is the node with the generalized coordinates
           AnimateModes(SC, mbs, nodeNumber)
           import sys
           sys.exit()
   
       
   
       if True:
   
           mPlane0 = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                         meshNodeNumbers=np.array(nodesPlane0), #these are the meshNodeNumbers
                                                         weightingFactors=weightsPlane0))
           mPlane1 = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], 
                                                         meshNodeNumbers=np.array(nodesPlane1), #these are the meshNodeNumbers
                                                         weightingFactors=weightsPlane1))
           
           if i==0:
               baseMarker = mPlane1
               oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
               mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=fb[i]['p0']))
               mbs.AddObject(GenericJoint(markerNumbers=[mGround, mPlane0], 
                                          constrainedAxes = [1,1,1,1,1,1],
                                          visualization=VGenericJoint(axesRadius=rFlange*0.5, axesLength=rFlange)))
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #robotics part
   graphicsBaseList = []
   if not useFlexBody:
       #graphicsBaseList +=[graphics.Brick([0,0,-0.15], [0.12,0.12,0.1], graphics.color.grey)]
   
       graphicsBaseList +=[graphics.Cylinder([0,0,-Lbase], [0,0,Lbase-flangeBaseL], rBase, graphics.color.blue)]
       graphicsBaseList +=[graphics.Cylinder([0,0,-flangeBaseL], [0,0,flangeBaseL], flangeBaseR, graphics.color.blue)]
       graphicsBaseList +=[graphics.Cylinder([0,0,0], [0.25,0,0], 0.00125, graphics.color.red)]
       graphicsBaseList +=[graphics.Cylinder([0,0,0], [0,0.25,0], 0.00125, graphics.color.green)]
       graphicsBaseList +=[graphics.Cylinder([0,0,0], [0,0,0.25], 0.00125, graphics.color.blue)]
   
   #base graphics is fixed to ground!!!
   graphicsBaseList +=[graphics.CheckerBoard([0,0,-Lbase], size=2.5)]
   #newRobot.base.visualization['graphicsData']=graphicsBaseList
   
   ty = 0.03
   tz = 0.04
   zOff = -0.05
   toolSize= [0.05,0.5*ty,0.06]
   graphicsToolList = [graphics.Cylinder(pAxis=[0,0,zOff], vAxis= [0,0,tz], radius=ty*1.5, color=graphics.color.red)]
   graphicsToolList+= [graphics.Brick([0,ty,1.5*tz+zOff], toolSize, graphics.color.grey)]
   graphicsToolList+= [graphics.Brick([0,-ty,1.5*tz+zOff], toolSize, graphics.color.grey)]
   
   
   #changed to new robot structure July 2021:
   newRobot = Robot(gravity=gravity,
                 base = RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList)),
                 tool = RobotTool(HT=HTtranslate([0,0,0.1]), visualization=VRobotTool(graphicsData=graphicsToolList)),
                 referenceConfiguration = []) #referenceConfiguration created with 0s automatically
   
   #modKKDH according to Khalil and Kleinfinger, 1986
   link0={'stdDH':[0,L[0],0,pi/2], 
          'modKKDH':[0,0,0,0], 
           'mass':20,  #not needed!
           'inertia':np.diag([1e-8,0.35,1e-8]), #w.r.t. COM! in stdDH link frame
           'COM':[0,0,0]} #in stdDH link frame
   
   link1={'stdDH':[0,0,L[1],0],
          'modKKDH':[0.5*pi,0,0,0], 
           'mass':17.4, 
           'inertia':np.diag([0.13,0.524,0.539]), #w.r.t. COM! in stdDH link frame
           'COM':[-0.3638, 0.006, 0.2275]} #in stdDH link frame
   
   link2={'stdDH':[0,L[2],W[2],-pi/2], 
          'modKKDH':[0,0.4318,0,0.15], 
           'mass':4.8, 
           'inertia':np.diag([0.066,0.086,0.0125]), #w.r.t. COM! in stdDH link frame
           'COM':[-0.0203,-0.0141,0.07]} #in stdDH link frame
   
   link3={'stdDH':[0,L[3],0,pi/2], 
          'modKKDH':[-0.5*pi,0.0203,0,0.4318], 
           'mass':0.82, 
           'inertia':np.diag([0.0018,0.0013,0.0018]), #w.r.t. COM! in stdDH link frame
           'COM':[0,0.019,0]} #in stdDH link frame
   
   link4={'stdDH':[0,0,0,-pi/2], 
          'modKKDH':[0.5*pi,0,0,0], 
           'mass':0.34, 
           'inertia':np.diag([0.0003,0.0004,0.0003]), #w.r.t. COM! in stdDH link frame
           'COM':[0,0,0]} #in stdDH link frame
   
   link5={'stdDH':[0,0,0,0], 
          'modKKDH':[-0.5*pi,0,0,0], 
           'mass':0.09, 
           'inertia':np.diag([0.00015,0.00015,4e-5]), #w.r.t. COM! in stdDH link frame
           'COM':[0,0,0.032]} #in stdDH link frame
   linkList=[link0, link1, link2, link3, link4, link5]
   
   #control parameters, per joint:
   Pcontrol = np.array([40000, 40000, 40000, 100, 100, 10])
   Dcontrol = np.array([400,   400,   100,   1,   1,   0.1])
   
   for i, link in enumerate(linkList):
       newRobot.AddLink(RobotLink(mass=link['mass'], 
                                  COM=link['COM'], 
                                  inertia=link['inertia'], 
                                  localHT=StdDH2HT(link['stdDH']),
                                  PDcontrol=(Pcontrol[i], Dcontrol[i]),
                                  ))
   
   showCOM = False
   for cnt, link in enumerate(newRobot.links):
       color = graphics.colorList[cnt]
       color[3] = 0.75 #make transparent
       link.visualization = VRobotLink(jointRadius=0.055, jointWidth=0.055*2, showMBSjoint=False,
                                       linkWidth=2*0.05, linkColor=color, showCOM= showCOM )
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #configurations and trajectory
   q0 = [0,0,0,0,0,0] #zero angle configuration
   
   #this set of coordinates only works with TSD, not with old fashion load control:
   # q1 = [0, pi/8, pi*0.75, 0,pi/8,0] #configuration 1
   # q2 = [pi,-pi, -pi*0.5,1.5*pi,-pi*2,pi*2] #configuration 2
   # q3 = [3*pi,0,-0.25*pi,0,0,0] #zero angle configuration
   
   #this set also works with load control:
   q1 = [0, pi/8, pi*0.5, 0,pi/8,0] #configuration 1
   q2 = [0.8*pi,0.5*pi, -pi*0.5,0.75*pi,-pi*0.4,pi*0.4] #configuration 2
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
   
   if not useFlexBody:
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
       
   torsionalSDlist = robotDict['springDamperList']
   
   #user function which is called only once per step, speeds up simulation drastically
   def PreStepUF(mbs, t):
       if compensateStaticTorques:
           staticTorques = ComputeMBSstaticRobotTorques(newRobot)
       else:
           staticTorques = np.zeros(len(jointList))
           
       [u,v,a] = trajectory.Evaluate(t)
   
       #compute load for joint number
       for i in range(len(jointList)):
           joint = jointList[i]
           phi = mbs.GetObjectOutput(joint, exu.OutputVariableType.Rotation)[2] #z-rotation
           omega = mbs.GetObjectOutput(joint, exu.OutputVariableType.AngularVelocityLocal)[2] #z-angular velocity
           tsd = torsionalSDlist[i]
           mbs.SetObjectParameter(tsd, 'offset', u[i])
           mbs.SetObjectParameter(tsd, 'velocityOffset', v[i])
           mbs.SetObjectParameter(tsd, 'torque', staticTorques[i]) #additional torque from given velocity 
       
       return True
   
   mbs.SetPreStepUserFunction(PreStepUF)
   
   
   if useFlexBody:
       baseType = 'Flexible'
   else:
       baseType = 'Rigid'
   
   #add sensors:
   cnt = 0
   jointTorque0List = []
   jointRotList = []
   for i in range(len(jointList)):
       jointLink = jointList[i]
       tsd = torsionalSDlist[i]
       #using TSD:
       sJointRot = mbs.AddSensor(SensorObject(objectNumber=tsd, 
                                  fileName='solution/joint' + str(i) + 'Rot'+baseType+'.txt',
                                  outputVariableType=exu.OutputVariableType.Rotation,
                                  writeToFile = sensorWriteToFile))
       jointRotList += [sJointRot]
   
       sJointAngVel = mbs.AddSensor(SensorObject(objectNumber=jointLink, 
                                  fileName='solution/joint' + str(i) + 'AngVel'+baseType+'.txt',
                                  outputVariableType=exu.OutputVariableType.AngularVelocityLocal,
                                  writeToFile = sensorWriteToFile))
   
       sTorque = mbs.AddSensor(SensorObject(objectNumber=tsd, 
                               fileName='solution/joint' + str(i) + 'Torque'+baseType+'.txt',
                               outputVariableType=exu.OutputVariableType.TorqueLocal,
                               writeToFile = sensorWriteToFile))
   
       sHandPos = mbs.AddSensor(SensorBody(bodyNumber=robotDict['bodyList'][-1], 
                               fileName='solution/handPos'+baseType+'.txt',
                               outputVariableType=exu.OutputVariableType.Position,
                               writeToFile = sensorWriteToFile))
   
       sHandVel = mbs.AddSensor(SensorBody(bodyNumber=robotDict['bodyList'][-1], 
                               fileName='solution/handVel'+baseType+'.txt',
                               outputVariableType=exu.OutputVariableType.Velocity,
                               writeToFile = sensorWriteToFile))
   
       jointTorque0List += [sTorque]
   
   
   mbs.Assemble()
   #mbs.systemData.Info()
   
   SC.visualizationSettings.connectors.showJointAxes = True
   SC.visualizationSettings.connectors.jointAxesLength = 0.02
   SC.visualizationSettings.connectors.jointAxesRadius = 0.002
   
   SC.visualizationSettings.nodes.show = False
   # SC.visualizationSettings.nodes.showBasis = True
   # SC.visualizationSettings.nodes.basisSize = 0.1
   SC.visualizationSettings.loads.show = False
   
   SC.visualizationSettings.openGL.multiSampling=4
       
   tEnd = 2
   h = 0.002
   
   #SC.renderer.DoIdleTasks()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = h*1
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.004
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
   SC.visualizationSettings.window.renderWindowSize=[1200,1200]
   SC.visualizationSettings.openGL.shadow = 0.25
   SC.visualizationSettings.openGL.light0position = [-2,5,10,0]
   useGraphics = True
   
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
   exu.Print('torques at tEnd=', VSum(measuredTorques))
   
   
   #%%+++++++++++++++++++++
   if True:
       
       import exudyn.plot
       exudyn.plot.PlotSensorDefaults().fontSize = 12
       
       title = baseType + ' base'
       mbs.PlotSensor(sensorNumbers=jointTorque0List, components=0, title='joint torques, '+title, closeAll=True,
                  fileName='solution/robotJointTorques'+baseType+'.pdf'
                  )
       mbs.PlotSensor(sensorNumbers=jointRotList, components=0, title='joint angles, '+title,
                  fileName='solution/robotJointAngles'+baseType+'.pdf'
                  )
       
       fPos = 'flexible base, Pos '
       fVel = 'flexible base, Vel '
       rPos = 'rigid base, Pos '
       rVel = 'rigid base, Vel '
       if baseType=='Flexible':
           mbs.PlotSensor(sensorNumbers=[sHandPos]*3+['solution/handPosRigid.txt']*3, components=[0,1,2]*2,
                      labels=[fPos+'X', fPos+'Y', fPos+'Z', rPos+'X', rPos+'Y', rPos+'Z'],
                      fileName='solution/robotPosition'+baseType+'.pdf'
                      )
           mbs.PlotSensor(sensorNumbers=[sHandVel]*3+['solution/handVelRigid.txt']*3, components=[0,1,2]*2,
                      labels=[fVel+'X', fVel+'Y', fVel+'Z', rVel+'X', rVel+'Y', rVel+'Z'],
                      fileName='solution/robotVelocity'+baseType+'.pdf'
                      )
   


