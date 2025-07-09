
.. _examples-serialrobotkinematictreedigging:

**********************************
serialRobotKinematicTreeDigging.py
**********************************

You can view and download this file on Github: `serialRobotKinematicTreeDigging.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/serialRobotKinematicTreeDigging.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example of a serial robot with minimal and redundant coordinates
   #           Robot interacts with particles
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-12-09
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   # import sys
   # sys.exudynFast = True
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   #from exudyn.rigidBodyUtilities import *
   #from exudyn.graphicsDataUtilities import *
   from exudyn.robotics import *
   from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration, ProfilePTP
   
   import numpy as np
   from numpy import linalg as LA
   from math import pi
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   sensorWriteToFile = True
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   compensateStaticTorques = False #static torque compensation converges slowly!
   useKinematicTree = True
   useGraphics = True
   addParticles = True
   doFast = 0 #0 / 1
   #kinematic tree and redundant mbs agrees for stdDH version up to 1e-10, with compensateStaticTorques = False
   # KT:      rotations at tEnd= 1.8464514676503092 , [0.4921990591981066, 0.2718999073958087, 0.818158053005264, -0.0030588904101585936, 0.26831938569719394, -0.0010660472359057434] 
   # red. MBS:rotations at tEnd= 1.8464514674961 ,   [ 0.49219906  0.27189991  0.81815805 -0.00305889  0.26831939 -0.00106605]
   
   
   #cup dimensions
   cupT = 0.005 #wall thickness
   cupR = 0.07 #outer radius
   cupRI = cupR-cupT #inner radius
   cupD = 2*cupR
   cupDI = 2*cupRI
   cupH = 0.15  #height
   
   #cup offset; in TCP coordinates!
   zOffTool = 0.2
   xOffTool = 0.075
   
   tEnd = 200
   stepSize = 0.0001#for 1000 particles
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #ground box with particles
   LL = 1
   t = 0.02*LL
   a = 0.2*LL
   b = 0.35*LL #height of base
   hw=2.4*a
   
   fastFact = 1
   LLx = LL
   if doFast:
       fastFact = 0.1
       LLx = fastFact*LL
   
   p0 = np.array([0.5*LL+0.5*a+0.25*LL*doFast,0,-0.5*t-b])
   p1 = np.array([-0.5*LL,0.5*LL+0.5*a,-0.5*t-b])
   color4wall = [0.6,0.6,0.6,0.5]
   addNormals = False
   gBox1 = graphics.Brick(p0,[LL,LL,t],graphics.color.steelblue,addNormals)
   gBox1Add = graphics.Brick(p0+[-0.5*LLx,0,0.35*hw],[t,LL,0.7*hw],color4wall,addNormals)
   gBox1 = graphics.MergeTriangleLists(gBox1, gBox1Add)
   gBox1Add = graphics.Brick(p0+[ 0.5*LLx,0,0.5*hw],[t,LL,hw],color4wall,addNormals)
   gBox1 = graphics.MergeTriangleLists(gBox1, gBox1Add)
   gBox1Add = graphics.Brick(p0+[0,-0.5*LL,0.5*hw],[LLx,t,hw],color4wall,addNormals)
   gBox1 = graphics.MergeTriangleLists(gBox1, gBox1Add)
   gBox1Add = graphics.Brick(p0+[0, 0.5*LL,0.5*hw],[LLx,t,hw],color4wall,addNormals)
   gBox1 = graphics.MergeTriangleLists(gBox1, gBox1Add)
   
   gBox2 = graphics.Brick(p1,[LL,LL,t],graphics.color.steelblue,addNormals)
   gBox2Add = graphics.Brick(p1+[-0.5*LL,0,0.5*hw],[t,LL,hw],color4wall,addNormals)
   gBox2 = graphics.MergeTriangleLists(gBox2, gBox2Add)
   gBox2Add = graphics.Brick(p1+[ 0.5*LL,0,0.35*hw],[t,LL,0.7*hw],color4wall,addNormals)
   gBox2 = graphics.MergeTriangleLists(gBox2, gBox2Add)
   gBox2Add = graphics.Brick(p1+[0,-0.5*LL,0.35*hw],[LL,t,0.7*hw],color4wall,addNormals)
   gBox2 = graphics.MergeTriangleLists(gBox2, gBox2Add)
   gBox2Add = graphics.Brick(p1+[0, 0.5*LL,0.5*hw],[LL,t,hw],color4wall,addNormals)
   gBox2 = graphics.MergeTriangleLists(gBox2, gBox2Add)
   
   #gDataList = [gBox1]
   
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0] ))
   mGround = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGround))
   
   
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if addParticles:
       np.random.seed(1) #always get same results
   
       boxX = LL-2*t #box size for particles
       boxY = LL-2*t
       boxZ = a
   
       nParticles = 12000 #50000; approx. number of particles
       ss = max(8,int(nParticles**(1/3)*1))
       print('tree cells x=', ss)
       fc = 1
       if nParticles>1000:
           stepSize*=round((1000./nParticles)**(1./2),1)
           if nParticles >= 80000:
               stepSize = 1e-5
           if nParticles >= 80000*2:
               stepSize = 5e-6
           if nParticles <= 12000:
               stepSize = 5e-5
           
           if stepSize <= 2e-5:
               fc = 4
           
       print('step size=',stepSize)
   
       npx = int(nParticles**(1./3.)) #approx particles in one dimension
       radius0 = boxX/(npx*2+1.5)*0.499
       print('LL=',LL,',npx=',npx,',r=',radius0)
       npz = int(npx*0.75) #0.75
       npx *= 2
   
       gContact = mbs.AddGeneralContact()
       gContact.verboseMode = 1
       gContact.resetSearchTreeInterval = 10000 #interval at which search tree memory is cleared 
       frictionCoeff = 0
       gContact.SetFrictionPairings(frictionCoeff*np.eye(1))
       gContact.SetSearchTreeCellSize(numberOfCells=[ss,ss,ss])
       #gContact.SetSearchTreeBox([0,-1,-0.1],[1.1,1,0.5])
       #gContact.SetSearchTreeBox([0,-2,0],[0.5*LL,0.5*LL,2])
   
       #contact parameters:
       k = 2e4*4
       d = 0.002*k #damping also has influence on conservation of (angular) momentum; improved if multiplied with factor 0.05
       density = 1000
       m = density*4/3*pi*radius0**3 #all particles get same mass!
       m /= radius0 #use larger mass for smaller particles ...
   
       if addParticles:
           [meshPoints, meshTrigs] = graphics.ToPointsAndTrigs(gBox1)
           gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mGround, 
                                               contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,
               pointList=meshPoints,  triangleList=meshTrigs)
           [meshPoints, meshTrigs] = graphics.ToPointsAndTrigs(gBox2)
           gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mGround, 
                                               contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,
               pointList=meshPoints,  triangleList=meshTrigs)
   
       #create particles:
       color4node = graphics.color.blue
       cnt = 0
       pBoxRef = p0 + [-0.5*radius0,-0.5*radius0,t+radius0]
       
       npy = npx
       if doFast:
           npx = int(fastFact*npx-2.5)
       
       for ix in range(npx+1):
           for iy in range(npy+1):
               for iz in range(npz+1):
       
                   color4node = graphics.colorList[int(min((iz/npz*10),10) )]
       
                   valueRand = np.random.random(1)[0]
                   radius = radius0 - radius0*0.3*valueRand #add some random size to decrease artifacts
                   
                   
                   pX = (iz%2)*radius0 #create densly packed particles
                   pY = (iz%2)*radius0
                   pRef0 = [(ix-npx*0.5)*radius0*2+pX, 
                            (iy-npy*0.5)*radius0*2+pY, 
                            0.73*iz*radius0*2-0.5*t]
                   # print(pRef0)
                   pRef = np.array(pRef0) + pBoxRef
                   v0 = [0,0,0]
   
                   if (cnt%20000 == 0): print("create mass",cnt)
                   nMass = mbs.AddNode(NodePoint(referenceCoordinates=pRef,
                                                 initialVelocities=v0,
                                                 visualization=VNodePoint(show=True,drawSize=2*radius, color=color4node)))
                   
                   #omitting the graphics speeds up, but does not allow shadow of particles ...
                   oMass = mbs.AddObject(MassPoint(physicsMass=m, nodeNumber=nMass,
                                                   #visualization=VMassPoint(graphicsData=[graphics.Sphere(radius=radius, color=color4node, nTiles=6)])
                                                   ))
                   mThis = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
       
                   mbs.AddLoad(Force(markerNumber=mThis, loadVector= [0,0,-m*9.81]))
       
                   gContact.AddSphereWithMarker(mThis, radius=radius, 
                                                contactStiffness=k, contactDamping=d, frictionMaterialIndex=0)
      
                   cnt += 1
       print('total particles added=', cnt)
   
   gCup=[]
   if True: #add cup
       colorCup = [0.8,0.1,0.1,0.5]
       contour=np.array([[0,0],[0,cupR],[cupH,cupR],[cupH, cupR-cupT],[cupT, cupR-cupT],[cupT, 0]])
       contour = list(contour)
       contour.reverse()
       gCup = graphics.SolidOfRevolution(pAxis=[xOffTool,0,zOffTool], vAxis=[-1,0,0],
               contour=contour, color=colorCup, nTiles = 64)
   
       gCupAdd = graphics.Cylinder(pAxis=[0,0,0], vAxis=[0,0,zOffTool-cupRI*1.01], radius=0.02, color=colorCup)
       gCup = graphics.MergeTriangleLists(gCup, gCupAdd)
       
       [meshPointsTool, meshTrigsTool] = graphics.ToPointsAndTrigs(gCup)
   
   
   
   from exudyn.robotics.models import ManipulatorPuma560, ManipulatorUR5
   
   robotDef = ManipulatorPuma560() #get dictionary that defines kinematics
   
   robotDef['links'][0]['inertia']=np.diag([1e-4,0.35,1e-4])
   #print(robotDef)
   Pcontrol = fc* np.array([40000*fc, 40000*fc, 40000*fc, 100*fc, 100*fc, 10*fc])
   Dcontrol = fc* np.array([400*fc,   400*fc,   100*fc,   1*fc,   1*fc,   0.1*fc])
   
   pBase=[0,0,0]
   gravity=[0,0,-9.81]  #gravity
   
   graphicsBaseList  = []
   graphicsBaseList += [graphics.Brick([0,0,-b*0.5-0.025], [a,a,b+t-0.05], graphics.color.brown)]
   graphicsBaseList += [graphics.CheckerBoard([0,0,-b-0.5*t], size=2.4)] 
   
   
   rRobotTCP = 0.041 
   graphicsToolList = [graphics.Cylinder(pAxis=[0,0,0], vAxis= [0,0,0.06], radius=0.05, color=graphics.color.red, nTiles=8)]
   
   
   graphicsToolList+= [gCup]
   
   
   #changed to new robot structure July 2021:
   robot = Robot(gravity=gravity,
                 base = RobotBase(HT=HTtranslate(pBase), visualization=VRobotBase(graphicsData=graphicsBaseList)),
                 tool = RobotTool(HT=HTtranslate([0,0,0]), visualization=VRobotTool(graphicsData=graphicsToolList)),
                 referenceConfiguration = []) #referenceConfiguration created with 0s automatically
   
   
   
   for cnt, link in enumerate(robotDef['links']):
       robot.AddLink(RobotLink(mass=link['mass'], 
                                  COM=link['COM'], 
                                  inertia=link['inertia'], 
                                  localHT=StdDH2HT(link['stdDH']),
                                  PDcontrol=(Pcontrol[cnt], Dcontrol[cnt]),
                                  visualization=VRobotLink(linkColor=graphics.colorList[cnt], showCOM=False, showMBSjoint=useGraphics)
                                  ))
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #configurations and trajectory
   q0 = [0,0.5*pi,-1.0*pi,0,0,0] #zero angle configuration
   
   q1 = [-0.07*pi,0.20*pi,-0.8*pi,0, 0.0*pi,-0.9*pi] 
   q2 = [-0.07*pi,0.16*pi,-0.9*pi,0, 0.0*pi,-0.6*pi] 
   q3 = [ 0.10*pi,0.16*pi,-0.9*pi,0, 0.0*pi,-0.4*pi] 
   q4 = [ 0.10*pi,0.40*pi,-1.0*pi,0,0.15*pi,-0.15*pi] 
   q5 = [ 0.65*pi,0.40*pi,-1.0*pi,0,0.15*pi, 0.15*pi] 
   q6 = [ 0.65*pi,0.30*pi,-0.9*pi,0, 0.0*pi,-1*pi] 
   q7 = [ 0.65*pi,0.40*pi,-0.9*pi,0, 0.0*pi,-1*pi] 
   
   doFast2 = 1*doFast
   
   if doFast2:
       q1 = [-0.07*pi,0.16*pi,-0.8*pi,0, 0.0*pi,-0.9*pi]  #fast trajectory
   
   #trajectory generated with optimal acceleration profiles:
   trajectory = Trajectory(initialCoordinates=q0, initialTime=0)
   # trajectory.Add(ProfileConstantAcceleration(q0,0.1))
   trajectory.Add(ProfileConstantAcceleration(q1,0.25*(1-0.8*doFast2)))
   # trajectory.Add(ProfileConstantAcceleration(q1,0.5))
   trajectory.Add(ProfileConstantAcceleration(q2,0.5*(1-0.9*doFast2)))
   # trajectory.Add(ProfileConstantAcceleration(q2,0.5))
   trajectory.Add(ProfileConstantAcceleration(q3,0.5*(1-0.9*doFast2)))
   # trajectory.Add(ProfileConstantAcceleration(q3,0.5))
   trajectory.Add(ProfileConstantAcceleration(q4,0.5*1.5))
   # trajectory.Add(ProfileConstantAcceleration(q4,0.5))
   trajectory.Add(ProfileConstantAcceleration(q5,0.5*1.5))
   #trajectory.Add(ProfileConstantAcceleration(q5,0.5))
   trajectory.Add(ProfileConstantAcceleration(q6,0.30))
   trajectory.Add(ProfileConstantAcceleration(q7,0.15))
   
   trajectory.Add(ProfileConstantAcceleration(q0,0.25))
   
   # x = traj.EvaluateCoordinate(t,0)
   
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #test robot model
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   jointList = [0]*robot.NumberOfLinks() #this list must be filled afterwards with the joint numbers in the mbs!
   
   def ComputeMBSstaticRobotTorques(robot):
       
       if not useKinematicTree:
           q=[]
           for joint in jointList:
               q += [mbs.GetObjectOutput(joint, exu.OutputVariableType.Rotation)[2]] #z-rotation
       else:
           q = mbs.GetObjectOutputBody(oKT, exu.OutputVariableType.Coordinates, localPosition=[0,0,0])
   
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
   if True:
       robotDict = robot.CreateKinematicTree(mbs)
       oKT = robotDict['objectKinematicTree']
       
       mbs.SetNodeParameter(robotDict['nodeGeneric'],'initialCoordinates',q0) #set according initial coordinates
       
       sTCP = mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=5, localPosition=[xOffTool,0,zOffTool],
                                                storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
       
       mTCP = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oKT, linkNumber=5, localPosition=[0,0,0]))
       
       if addParticles:
           gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mTCP, contactStiffness=k, contactDamping=d, frictionMaterialIndex=0,
               pointList=meshPointsTool,  triangleList=meshTrigsTool)
       
       #add ground after robot, to enable transparency
       oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                          visualization=VObjectGround(graphicsData=[gBox1,gBox2])))
       
       tMax = trajectory.GetTimes()[-1] #total trajectory time
       print('trajectory cycle time=',round(tMax))
       #user function which is called only once per step, speeds up simulation drastically
       def PreStepUF(mbs, t):
           if compensateStaticTorques:
               staticTorques = ComputeMBSstaticRobotTorques(robot)
               #print("tau=", staticTorques)
           else:
               staticTorques = np.zeros(len(jointList))
           
           tCnt = int(t/tMax)
           tOff = tCnt*tMax
           [u,v,a] = trajectory.Evaluate(t-tOff)
       
           #in case of kinematic tree, very simple operations!
           mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector', u)
           mbs.SetObjectParameter(oKT, 'jointVelocityOffsetVector', v)
           mbs.SetObjectParameter(oKT, 'jointForceVector', staticTorques)
           
           return True
       
       mbs.SetPreStepUserFunction(PreStepUF)
   
   
   mbs.Assemble()
   #mbs.systemData.Info()
   
   SC.visualizationSettings.connectors.showJointAxes = True
   SC.visualizationSettings.connectors.jointAxesLength = 0.02
   SC.visualizationSettings.connectors.jointAxesRadius = 0.002
   
   SC.visualizationSettings.nodes.showBasis = False
   SC.visualizationSettings.loads.show = False
   
   SC.visualizationSettings.openGL.multiSampling=4
       
   
   #SC.renderer.DoIdleTasks()
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.stepInformation = 1+32 #time to go and time spent
   simulationSettings.solutionSettings.solutionWritePeriod = 0.01*2
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.005
   simulationSettings.solutionSettings.binarySolutionFile = True
   simulationSettings.solutionSettings.outputPrecision = 5 #make files smaller
   simulationSettings.solutionSettings.exportAccelerations = False
   simulationSettings.solutionSettings.exportVelocities = False
   simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/test.sol'
   #simulationSettings.solutionSettings.writeSolutionToFile = False
   # simulationSettings.timeIntegration.simulateInRealtime = True
   # simulationSettings.timeIntegration.realtimeFactor = 0.25
   simulationSettings.timeIntegration.explicitIntegration.computeEndOfStepAccelerations = False #speedup ...
   simulationSettings.timeIntegration.explicitIntegration.computeMassMatrixInversePerBody = True #>>speedup ...
   # simulationSettings.timeIntegration.reuseConstantMassMatrix = True
   
   simulationSettings.parallel.numberOfThreads = 8
   
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.timeIntegration.verboseModeFile = 1
   simulationSettings.solutionSettings.solverInformationFileName = 'solution/solverTest.txt'
   # simulationSettings.displayComputationTime = True
   # simulationSettings.displayStatistics = True
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   
   #simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
   SC.visualizationSettings.general.autoFitScene=False
   SC.visualizationSettings.window.renderWindowSize=[1920,1200]
   #SC.visualizationSettings.general.circleTiling = 100
   SC.visualizationSettings.general.textSize = 14
   SC.visualizationSettings.general.showSolutionInformation = False
   SC.visualizationSettings.general.showSolverInformation = False
   SC.visualizationSettings.general.graphicsUpdateInterval = 4#0.05
   SC.visualizationSettings.bodies.kinematicTree.showJointFrames=False
   SC.visualizationSettings.general.drawCoordinateSystem=False
   SC.visualizationSettings.general.drawWorldBasis=False
   
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   SC.visualizationSettings.nodes.show = True
   SC.visualizationSettings.markers.show = False
   SC.visualizationSettings.nodes.defaultSize = 0 #must not be -1, otherwise uses autocomputed size
   SC.visualizationSettings.nodes.tiling = 8
   SC.visualizationSettings.openGL.shadow = 0.4
   # SC.visualizationSettings.contact.showSearchTree = 1
   # SC.visualizationSettings.contact.showSearchTreeCells = 1
   
   if useGraphics:
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys['renderState'])
       SC.renderer.DoIdleTasks()
       
   # pTCP = mbs.GetSensorValues(sTCP)
   # print('pTCP=',pTCP)
   #mbs.SolveDynamic(simulationSettings, showHints=True)
   mbs.SolveDynamic(simulationSettings, 
                     #solverType=exu.DynamicSolverType.RK44,
                     solverType=exu.DynamicSolverType.ExplicitEuler,
                     showHints=True)
   
   
   if useGraphics:
       SC.visualizationSettings.general.autoFitScene = False
       SC.renderer.Stop()
   
   if True:
   #%%++++++++++
       SC.visualizationSettings.general.autoFitScene = False
       # SC.visualizationSettings.general.graphicsUpdateInterval=0.5
       
       # print('load solution file')
       # sol = LoadSolutionFile('solution/test.sol', safeMode=True)#, maxRows=100)
       # print('start SolutionViewer')
       # mbs.SolutionViewer(sol)
       mbs.SolutionViewer()
    
   
   
   
   


