
.. _examples-humanrobotinteraction:

************************
humanRobotInteraction.py
************************

You can view and download this file on Github: `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/humanRobotInteraction.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  3D rigid body tutorial with 2 bodies and revolute joints, using new utilities functions
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-08-05
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   import numpy as np
   from exudyn.robotics import *
   from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration, ProfilePTP
   
   from math import pi
   import copy
   import time
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   addRobot = True
   useKT = True #model articulated body as kinematic tree; allows simpler control
   addHand = True
   addHandKinematic = True and addHand
   addFullBody = True #only graphics
   
   
   tStart = time.time()
   
   #try to turn of warnings for stl geometry:
   try:
       import stl
       def NoWarnings(*args): pass
       stl.base.BaseMesh.warning = NoWarnings
   except: pass
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #physical parameters
   gravity=[0,0,-9.81]  #gravity
   L = 1               #length
   w = 0.1             #width
   bodyDim=[L,w,w] #body dimensions
   p0 =    [0,0,0]     #origin of pendulum
   pMid0 = np.array([L*0.5*0,0,0]) #center of mass, body0
   
   #ground body
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   scaleBody = 0.0254 #conversion: inches to meter (articulated-dummy), original file
   scaleBody = 0.001  #conversion: millimeter to meter (articulated-dummy2), modified file
   
   leftShoulder = np.array([0.086, -0.185, 1.383]) #position of left shoulder joint in meter
   leftElbow = np.array([0.109, -0.206, 1.122]) #position of left shoulder joint in meter
   leftHand = np.array([0.0945, -0.2520, 0.8536]) #position of left shoulder joint in meter
   
   rightShoulder = np.array([0.086, 0.185, 1.383]) #position of left shoulder joint in meter
   rightElbow = np.array([0.109, 0.206, 1.122]) #position of left shoulder joint in meter
   rShoulder = 0.047
   rElbow = 0.032
   rHand = 0.025
   
   density = 1000. #human body average density ... like water
   verbose = False
   graphicsBody =[]
   graphicsUAL =[]
   graphicsLAL =[]
   
   listBody = ['UpperArm_R', 'LowerArm_R', 'Hand_R', 'Fingers_R', 'Head', 'Pelvis',
               'LowerLeg_R', 'LowerLeg_L', 'UpperLeg_R', 'UpperLeg_L', 'Foot_L', 'Foot_R']
   
   listAll = ['Torso', 'UpperArm_L', 'LowerArm_L', 'Hand_L', 'Fingers_L'] + listBody
   myDir = 'C:/DATA/cpp/DocumentationAndInformation/STL/articulated-dummy2/' #graphics for articulated dummy not included in model; download at GrabCAD / articulated-dummy
   
   
   if False:
       #convert ascii files to binary stl:
       from stl import mesh, Mode
       for file in ['Fingers_L']:
       #for file in listAll:
           print('convert',file)
           data=mesh.Mesh.from_file(myDir+file+'.stl')
           data.save(filename=myDir+file+'.stl', mode=Mode.BINARY)
   
   
   #graphics taken from https://grabcad.com/library/articulated-dummy-1
   #print('1*********************************************',flush=True)
   dataUAL = graphics.FromSTLfile(fileName=myDir+'UpperArm_L.stl', 
                                           color=graphics.color.blue, verbose=verbose, density=density,
                                           scale = scaleBody)
   
   dataLAL = graphics.FromSTLfile(fileName=myDir+'LowerArm_L.stl', 
                                           color=graphics.color.dodgerblue, verbose=verbose, density=density,
                                           scale = scaleBody)
   
   if addHand:
       dataHandL = graphics.FromSTLfile(fileName=myDir+'Hand_L.stl', 
                                               color=graphics.color.brown, verbose=verbose, density=density,
                                               scale = scaleBody)
       dataFingersL = graphics.FromSTLfile(fileName=myDir+'Fingers_L.stl', 
                                               color=graphics.color.brown, verbose=verbose, density=density,
                                               scale = scaleBody)
       
       #++++++++++++++++++++++++++++++++++
       #merge mass, COM and inertia of lower arm, hand and fingers:
       rbiLAL = RigidBodyInertia(dataLAL[1]['mass'], dataLAL[1]['inertia'], dataLAL[1]['COM'], inertiaTensorAtCOM=True)
       rbiHandL = RigidBodyInertia(dataHandL[1]['mass'], dataHandL[1]['inertia'], dataHandL[1]['COM'], inertiaTensorAtCOM=True)
       rbiFingersL = RigidBodyInertia(dataFingersL[1]['mass'], dataFingersL[1]['inertia'], dataFingersL[1]['COM'], inertiaTensorAtCOM=True)
       
       # print('rbiLAL=',rbiLAL)
       # print('rbiHandL=',rbiHandL)
       # print('rbiFingersL=',rbiFingersL)
       rbiLAL= rbiLAL + rbiHandL + rbiFingersL
       dataLAL[1]['mass'] = rbiLAL.Mass()
       dataLAL[1]['inertia'] = rbiLAL.InertiaCOM()
       dataLAL[1]['COM'] = rbiLAL.COM()
   #++++++++++++++++++++++++++++++++++
   graphicsBody += [AddEdgesAndSmoothenNormals(graphics.FromSTLfile(fileName=myDir+'Torso.stl', 
                                           color=graphics.color.grey, verbose=verbose, density=density,
                                           scale = scaleBody)[0], addEdges=False)]
   
   
   if addFullBody:
       for part in listBody:
           data = graphics.FromSTLfile(fileName=myDir+''+part+'.stl', 
                                          color=graphics.color.grey, verbose=verbose, density=density*0,
                                          scale = scaleBody)
           graphicsBody += [AddEdgesAndSmoothenNormals(data, addEdges=False)]
           #graphicsBody += [data[0]]
   
   
   # if True: #makes even bigger files ...
       # fileName = myDir+'data.npy'
       # with open(fileName, 'wb') as f:
       #     np.save(f, graphicsBody, allow_pickle=True)
   
       # with open(fileName, 'rb') as f:
       #     graphicsBody = np.load(f, allow_pickle=True).all()
   
   
   #body fixed to ground    
   graphicsBody += [graphics.CheckerBoard(size=4)]
   
   pBody = np.array([0,0,0])
   oGround = mbs.AddObject(ObjectGround(referencePosition=pBody, visualization=VObjectGround(graphicsData=graphicsBody)))
   
   if useKT:
   
       #%%++++++++++++++++++++++++++++++++++++++++
       #motion and control of articulated body
       z0 = -0.15*pi
       q0 = [0,0,0,0,0,0,0] #zero angle configuration, max 7 joints
       q1 = [0,pi*0.125,z0,pi*(0.375-0.03),0,0,0] #zero angle configuration, max 7 joints
       # q1 = [-pi*0.5,0,0,0,0,0,0]
       q2 = [-pi*0.5,pi*0.5,0,0,0,0,0]
       q3 = [-pi*0.5,pi*0.5,pi*0.5,0,0,0,0]
       q4 = [0,pi*0.125,0,pi*0.375,0,0,0]
       
       #trajectory generated with optimal acceleration profiles:
       bodyTrajectory = Trajectory(initialCoordinates=q0, initialTime=0.25)
       bodyTrajectory.Add(ProfileConstantAcceleration(q1,0.4))
       #bodyTrajectory.Add(ProfileConstantAcceleration(q1,0.25))
       # bodyTrajectory.Add(ProfileConstantAcceleration(q2,0.25))
       # bodyTrajectory.Add(ProfileConstantAcceleration(q3,0.25))
       # bodyTrajectory.Add(ProfileConstantAcceleration(q4,0.25))
       # bodyTrajectory.Add(ProfileConstantAcceleration(q0,0.25))
   
       Pcontrol = 0.1*np.array([5000,2*5000,5000, 2*5000, 2000,2000,2000]) #3 x elbow, 1 x shoulder, 3 x hand
       Dcontrol = 0.02 * Pcontrol
       #%%++++++++++++++++++++++++++++++++++++++++
   
       articulatedBody = Robot(gravity=[0,0,9.81],
                     #base = RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList)), #already added to ground
                     #tool = RobotTool(HT=HTtranslate([0,0,0]), visualization=VRobotTool(graphicsData=graphicsToolList)),
                     referenceConfiguration = []) #referenceConfiguration created with 0s automatically
   
       jointRadius = 0.06
       jointWidth  = 0.0125
       linkWidth   = 0.001
       showMBSjoint= False
       showCOM     = False
   
       body = dataUAL
       body[0] = graphics.Move(AddEdgesAndSmoothenNormals(body[0], addEdges=False), -leftShoulder, np.eye(3))
       link = body[1]
   
       articulatedBody.AddLink(RobotLink(jointType='Rx',
                                         mass = 0, COM=[0,0,0], inertia = 0*np.eye(3),
                                         preHT=HomogeneousTransformation(np.eye(3), leftShoulder),
                                         PDcontrol=(Pcontrol[0], Dcontrol[0]),
                                         visualization=VRobotLink(showCOM=showCOM, jointRadius=jointRadius, jointWidth=jointWidth, linkWidth=linkWidth, showMBSjoint=showMBSjoint)
                                         ))
       articulatedBody.AddLink(RobotLink(jointType='Ry',
                                         mass = 0, COM=[0,0,0], inertia = 0*np.eye(3),
                                         #preHT=HT0(),
                                         PDcontrol=(Pcontrol[1], Dcontrol[1]),
                                         visualization=VRobotLink(showCOM=showCOM, jointRadius=jointRadius, jointWidth=jointWidth, linkWidth=linkWidth, showMBSjoint=showMBSjoint)
                                         ))
                                         
       
       articulatedBody.AddLink(RobotLink(jointType='Rz',
                                         mass=link['mass'],
                               COM=link['COM']-leftShoulder,
                               inertia=link['inertia'],
                               #preHT=HomogeneousTransformation(np.eye(3), leftShoulder),
                               PDcontrol=(Pcontrol[2], Dcontrol[2]),
                               visualization=VRobotLink(showCOM=showCOM, jointRadius=jointRadius, jointWidth=jointWidth, linkWidth=linkWidth, showMBSjoint=showMBSjoint,
                                                        graphicsData=[body[0]])
                               ))
   
       body = dataLAL
       body[0] = graphics.Move(AddEdgesAndSmoothenNormals(body[0], addEdges=False), -leftElbow, np.eye(3))
       link = body[1]
       gList = [body[0]]
       
       if addHand:
           if not addHandKinematic:
               dataHandL[0] = graphics.Move(AddEdgesAndSmoothenNormals(dataHandL[0], addEdges=False), -leftElbow, np.eye(3))
               dataFingersL[0] = graphics.Move(dataFingersL[0], -leftElbow, np.eye(3))
               gList = [body[0], dataHandL[0], dataFingersL[0]]
           
           
       gList += [graphics.Sphere(leftHand-leftElbow, radius=rHand, color=graphics.color.brown, nTiles=16)]
       
       articulatedBody.AddLink(RobotLink(jointType='Ry',
                                         mass=link['mass'],
                                         COM=link['COM']-leftElbow,
                                         inertia=link['inertia'],
                                         preHT=HomogeneousTransformation(np.eye(3), leftElbow-leftShoulder),
                                         PDcontrol=(Pcontrol[3], Dcontrol[3]),
                                         visualization=VRobotLink(showCOM=showCOM, jointRadius=jointRadius, jointWidth=jointWidth, linkWidth=linkWidth, showMBSjoint=showMBSjoint,
                                                                  graphicsData=gList)
                                         ))
   
       if addHandKinematic:
           body = dataHandL
           link = body[1]
           dataHandL[0] = graphics.Move(dataHandL[0], -leftHand, np.eye(3))
           dataFingersL[0] = graphics.Move(dataFingersL[0], -leftHand, np.eye(3))
           gList = [dataHandL[0], dataFingersL[0]]
   
   
           articulatedBody.AddLink(RobotLink(jointType='Rx',
                                             mass = 0, COM=[0,0,0], inertia = 0*np.eye(3),
                                             preHT=HomogeneousTransformation(np.eye(3), leftHand-leftElbow),
                                             PDcontrol=(Pcontrol[4], Dcontrol[4]),
                                             visualization=VRobotLink(showCOM=showCOM, jointRadius=jointRadius, jointWidth=jointWidth, linkWidth=linkWidth, showMBSjoint=showMBSjoint)
                                             ))
           articulatedBody.AddLink(RobotLink(jointType='Ry',
                                             mass = 0, COM=[0,0,0], inertia = 0*np.eye(3),
                                             PDcontrol=(Pcontrol[5], Dcontrol[5]),
                                             visualization=VRobotLink(showCOM=showCOM, jointRadius=jointRadius, jointWidth=jointWidth, linkWidth=linkWidth, showMBSjoint=showMBSjoint)
                                             ))
           articulatedBody.AddLink(RobotLink(jointType='Rz',
                                             mass=link['mass'],
                                             COM=link['COM']-leftShoulder,
                                             inertia=link['inertia'],
                                             PDcontrol=(Pcontrol[6], Dcontrol[6]),
                                             visualization=VRobotLink(showCOM=showCOM, jointRadius=jointRadius, jointWidth=jointWidth, linkWidth=linkWidth, showMBSjoint=showMBSjoint,
                                                                    graphicsData=gList)
                                   ))
   
   
       articulatedBody.referenceConfiguration = q0[0:articulatedBody.NumberOfLinks()]
       #create kinematic tree of body
       articulatedBodyDict = articulatedBody.CreateKinematicTree(mbs)
       oKTarticulatedBody = articulatedBodyDict['objectKinematicTree']
   
       mHand = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber = oKTarticulatedBody, linkNumber = 3, 
                                                      localPosition=leftHand-leftElbow))
   
   else:
       #%%++++++++++++++++++++++++++
       #upper arm left:
       if False:
           body = dataUAL
           
           body[0] = graphics.Move(body[0], -body[1]['COM'], np.eye(3))
           
           dictUAL1 = mbs.CreateRigidBody(
                         inertia=RigidBodyInertia(mass=body[1]['mass'], inertiaTensor=body[1]['inertia'], com=[0,0,0]),
                         referencePosition=body[1]['COM'],
                         referenceRotationMatrix=np.eye(3),
                         gravity=gravity,
                         graphicsDataList=[body[0]],
                         returnDict=True)
           [nUAL, bUAL] = [dictUAL1['nodeNumber'], dictUAL1['bodyNumber']]
           
           #markers for ground and rigid body (not needed for option 3):
           markerGroundUAL = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=leftShoulder))
           markerUAL0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bUAL, localPosition=leftShoulder-body[1]['COM']))
           markerUAL1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bUAL, localPosition=leftElbow-body[1]['COM']))
           
           jUAL = mbs.AddObject(SphericalJoint(markerNumbers=[markerGroundUAL, markerUAL0], 
                                               visualization=VSphericalJoint(jointRadius=rShoulder)))
           
           
           #%%++++++++++++++++++++++++++
           #lower arm left:
           body = dataLAL
           
           body[0] = graphics.Move(body[0], -body[1]['COM'], np.eye(3))
           
           dictUAL2 = mbs.CreateRigidBody(
                         inertia=RigidBodyInertia(mass=body[1]['mass'], inertiaTensor=body[1]['inertia'], com=[0,0,0]),
                         referencePosition=body[1]['COM'],
                         referenceRotationMatrix=np.eye(3),
                         gravity=gravity,
                         graphicsDataList=[body[0]],
                         returnDict=True)
           [nUAL, bUAL] = [dictUAL2['nodeNumber'], dictUAL2['bodyNumber']]
           
           #markers for ground and rigid body (not needed for option 3):
           markerLAL0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bUAL, localPosition=leftElbow-body[1]['COM']))
           
           jLAL = mbs.AddObject(GenericJoint(markerNumbers=[markerUAL1, markerLAL0], 
                                             constrainedAxes=[1,1,1, 1,0,1],
                                             visualization=VGenericJoint(axesRadius=0.1*rElbow)))
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #add simple robot:
   
   if addRobot:
       from exudyn.robotics.models import ManipulatorPuma560, ManipulatorUR5
       
       robotDef = ManipulatorPuma560() #get dictionary that defines kinematics
       fc = 0.5
       Pcontrol = fc* np.array([40000, 40000, 40000, 100, 100, 10])
       Dcontrol = fc* np.array([400,   400,   100,   1,   1,   0.1])
   
       pBase = np.array([-1,-0.2,0.75])
       #+++++++++
       #some graphics for Puma560    
       jointWidth=0.1
       jointRadius=0.06
       linkWidth=0.1
       
       graphicsBaseList = [graphics.Brick([0,0,-0.15], [0.12,0.12,0.1], graphics.color.grey)]
       graphicsBaseList = [graphics.Brick([0,0,-0.75*0.5-0.05], [pBase[2],pBase[2],0.65], graphics.color.brown)]
       graphicsBaseList +=[graphics.Cylinder([0,0,0], [0.5,0,0], 0.0025, graphics.color.red)]
       graphicsBaseList +=[graphics.Cylinder([0,0,0], [0,0.5,0], 0.0025, graphics.color.green)]
       graphicsBaseList +=[graphics.Cylinder([0,0,0], [0,0,0.5], 0.0025, graphics.color.blue)]
       graphicsBaseList +=[graphics.Cylinder([0,0,-jointWidth], [0,0,jointWidth], linkWidth*0.5, graphics.colorList[0])] #belongs to first body
       
       rRobotTCP = 0.041 #also used for contact
       ty = 0.03
       tz = 0.04
       zOff = -0.05+0.1
       toolSize= [0.05,0.5*ty,0.06]
       graphicsToolList = [graphics.Cylinder(pAxis=[0,0,zOff], vAxis= [0,0,tz], radius=ty*1.5, color=graphics.color.red)]
       # graphicsToolList+= [graphics.Brick([0,ty,1.5*tz+zOff], toolSize, graphics.color.grey)] #gripper
       # graphicsToolList+= [graphics.Brick([0,-ty,1.5*tz+zOff], toolSize, graphics.color.grey)] #gripper
       graphicsToolList+= [graphics.Sphere(point=[0,0,0.12],radius=rRobotTCP, color=[1,0,0,1], nTiles=16)]
       
       #+++++++++
       
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
                                      visualization=VRobotLink(linkColor=graphics.colorList[cnt], showCOM=False, showMBSjoint=True)
                                      ))
   
       q0 = [0,0.5*pi,-0.5*pi,0,0,0] #zero angle configuration
       
       q1 = [-0.35*pi,0.5*pi,-0.5*pi,0,0,0] #zero angle configuration
       q2 = [-0.35*pi,0.5*pi,-1.0*pi,0,0,0] #zero angle configuration
       q3 = [-0.35*pi,0.25*pi,-0.75*pi,0,0,0] #zero angle configuration
       q4 = [ 0.07*pi,0.38*pi,-0.88*pi,0,0,0] #zero angle configuration
   
       robot.referenceConfiguration = q0
       
       #trajectory generated with optimal acceleration profiles:
       robotTrajectory = Trajectory(initialCoordinates=q0, initialTime=0.)
       robotTrajectory.Add(ProfileConstantAcceleration(q0,0.25))
       # robotTrajectory.Add(ProfileConstantAcceleration(q1,0.25))
       # robotTrajectory.Add(ProfileConstantAcceleration(q2,0.5))
       robotTrajectory.Add(ProfileConstantAcceleration(q3,0.25))
       robotTrajectory.Add(ProfileConstantAcceleration(q4,0.2)) #0.25 is regular speed
       
       robotDict = robot.CreateKinematicTree(mbs)
       oKTrobot = robotDict['objectKinematicTree']
   
       mRobotTCP0 = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber = oKTrobot, linkNumber = 5, 
                                                      localPosition=[0,0,0.12]))
       # mRobotTCP1 = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber = oKTrobot, linkNumber = 5, 
       #                                                localPosition=[0,0,0.12]))
       # mRobotTCP2 = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber = oKTrobot, linkNumber = 5, 
       #                                                localPosition=[0,0,0.14]))
   
   def PreStepUF(mbs, t):
       if useKT:
           [u,v,a] = bodyTrajectory.Evaluate(t)
       
           #in case of kinematic tree, very simple operations!
           mbs.SetObjectParameter(oKTarticulatedBody, 'jointPositionOffsetVector', u)
           mbs.SetObjectParameter(oKTarticulatedBody, 'jointVelocityOffsetVector', v)
           # if compensateStaticTorques:
           # mbs.SetObjectParameter(oKTarticulatedBody, 'jointForceVector', ComputeMBSstaticRobotTorques(articulatedBody))
       if addRobot:
           [u,v,a] = robotTrajectory.Evaluate(t)
       
           #in case of kinematic tree, very simple operations!
           mbs.SetObjectParameter(oKTrobot, 'jointPositionOffsetVector', u)
           mbs.SetObjectParameter(oKTrobot, 'jointVelocityOffsetVector', v)
           # if compensateStaticTorques:
           # mbs.SetObjectParameter(oKTrobot, 'jointForceVector', ComputeMBSstaticRobotTorques(articulatedBody))
       
       return True
   
   mbs.SetPreStepUserFunction(PreStepUF)
   
   print('loading took',time.time()-tStart,'seconds')
   
   
   if True:
       markerList = [mHand, mRobotTCP0]
       radiusList = [rHand*1.1, rRobotTCP]
   
       
       
       gContact = mbs.AddGeneralContact()
       gContact.verboseMode = 1
       
       contactStiffness = 5e5# 1e6*2 #2e6 for test with spheres
       contactDamping = 0.02*contactStiffness
   
       for i in range(len(markerList)):
           m = markerList[i]
           r = radiusList[i]
           gContact.AddSphereWithMarker(m, radius=r, contactStiffness=contactStiffness, contactDamping=contactDamping, frictionMaterialIndex=0)
   
   
       gContact.SetFrictionPairings(0.*np.eye(1))
       gContact.SetSearchTreeCellSize(numberOfCells=[4,4,4]) #could also be 1,1,1
       gContact.SetSearchTreeBox(pMin=np.array([-2,-2,-2]), pMax=np.array([2,2,2])) 
   
   
   
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #assemble system before solving
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 1.25 #simulation time
   h = 0.25*1e-3 #step size
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.timeIntegration.simulateInRealtime = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.005 #store every 5 ms
   
   SC.visualizationSettings.window.renderWindowSize=[1600,1200]
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.general.autoFitScene = False
   
   SC.visualizationSettings.nodes.drawNodesAsPoint=False
   SC.visualizationSettings.nodes.showBasis=True
   SC.visualizationSettings.general.drawWorldBasis=True
   SC.visualizationSettings.bodies.kinematicTree.showJointFrames = False
   
   SC.visualizationSettings.openGL.multiSampling=4
   SC.visualizationSettings.openGL.lineWidth = 2
   # SC.visualizationSettings.openGL.shadow=0.3 #don't do this for fine meshes!
   SC.visualizationSettings.openGL.light0position=[-6,2,12,0]
   
   SC.renderer.Start()
   if 'renderState' in exu.sys: #reload old view
       SC.renderer.SetState(exu.sys['renderState'])
   
   SC.renderer.DoIdleTasks() #stop before simulating
   
   mbs.SolveDynamic(simulationSettings = simulationSettings,
                     solverType=exu.DynamicSolverType.TrapezoidalIndex2)
   
   SC.renderer.Stop() #safely close rendering window!
   
   mbs.SolutionViewer()
   
   if False:
       
       mbs.PlotSensor([sens1],[1])
   
   


