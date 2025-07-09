
.. _examples-kinematictreeandmbs:

**********************
kinematicTreeAndMBS.py
**********************

You can view and download this file on Github: `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN python utility library
   #
   # Details:  Test for kinematicTree; this test uses different versions of Python and C++ implementations for kinematic tree (implementation may not be optimal or have some unnecessary overheads);
   #           The Python tests are not efficient, so use only C++ ObjectKinematicTree for performance reasons!
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-08-20
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #constants and fixed structures:
   import numpy as np
   from math import pi, sin, cos#, sqrt
   from copy import copy, deepcopy
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.rigidBodyUtilities import Skew, Skew2Vec
   from exudyn.robotics import *
   
   from exudyn.kinematicTree import *
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   gGround = graphics.Brick(centerPoint=[-0.5,0,0], size=[1,0.4,0.5], color=graphics.color.lightgrey)
   objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0],
                                             visualization=VObjectGround(graphicsData=[gGround])))
   
   L = 2 #length of links
   w = 0.1 #width of links
   J = InertiaCuboid(density=1000, sideLengths=[L,w,w]) #w.r.t. reference center of mass
   #J.com = np.array([0.5*L,0,0])
   J = J.Translated([0.5*L,0,0])
   
   com = J.com
   Jcom = J.Translated(-com)
   
   gravity3D = [0,-10,0]
   
   n=5 #number of coordinates
   Amat = [np.zeros((3,3))]*n
   vVec = [np.zeros(3)]*n
   
   listCOM = [np.zeros(3)]*n
   listInertiaCOM = [np.zeros((3,3))]*n
   listInertia = [np.zeros((3,3))]*n
   listMass = [0]*n
   for i in range(n):
       A=np.eye(3)
       if i%2 != 0:
           A=RotXYZ2RotationMatrix([0*0.5*pi,0.25*pi,0])
       if i%3 >= 1:
           A=RotXYZ2RotationMatrix([0.5*pi,0.25*pi,0])
           
       v = np.array([L,0,0])
       if i==0:
           v = np.array([0,0,0])
       Amat[i] = A
       vVec[i] = v
       
       listMass[i] = J.mass
       listCOM[i] = J.com
       listInertia[i] = J.inertiaTensor
       listInertiaCOM[i] = Jcom.inertiaTensor
   
   useKT = True
   useMBS = False #for this option, choose dynamic solver!
   useKT2 = False
   useKTcpp = True
   sJointsList = []
   sCases = []
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   baseMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=objectGround, localPosition=[0,0,0]))
   graphicsBaseList = [gGround]
   graphicsBaseList +=[graphics.Cylinder([0,0,0], [0.5,0,0], 0.0025, graphics.color.red)]
   graphicsBaseList +=[graphics.Cylinder([0,0,0], [0,0.5,0], 0.0025, graphics.color.green)]
   graphicsBaseList +=[graphics.Cylinder([0,0,0], [0,0,0.5], 0.0025, graphics.color.blue)]
   newRobot = Robot(gravity=gravity3D,
                 base = RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList)),
                 tool = RobotTool(HT=HTtranslate([0,0,0]), visualization=VRobotTool(graphicsData=[])),
                referenceConfiguration = []) #referenceConfiguration created with 0s automatically
   
   for i in range(n):
       newRobot.AddLink(RobotLink(mass=listMass[i],
                                  COM=listCOM[i], 
                                  inertia=listInertiaCOM[i], 
                                  preHT = HomogeneousTransformation(Amat[i], vVec[i]),
                                  ))
   
   if useMBS:
       robDict = newRobot.CreateRedundantCoordinateMBS(mbs=mbs, baseMarker=baseMarker, createJointTorqueLoads=False)
                                                       # invertJoints=True)
       joints = robDict['jointList']
       #user function for sensor, convert position into angle:
       def UFsensor(mbs, t, sensorNumbers, factors, configuration):
           val = np.zeros(n)
           for i in range(n):
               val[i] = mbs.GetObjectOutput(joints[i], exu.OutputVariableType.Rotation)[2] #z-rotation
           return val #return joint angles
   
       sJointsList += [mbs.AddSensor(SensorUserFunction(sensorNumbers=[],
                               fileName='solution/serialRobMBS.txt',
                               sensorUserFunction=UFsensor))]
       sCases += ['RC MBS']
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #generate kinematic chain:
   jointTypes = []
   transformations = []
   inertias = []
   for i in range(n):
       jointTypes += ['Rz']
       X=RotationTranslation2T66Inverse(A=Amat[i], v=vVec[i]) #A is the transformation from (i) to (i-1), while the Featherstone formalism requires the transformation from (i-1) to i
       #print(X.round(3))
       transformations += [X] #defines transformation to joint in parent link
       inertias += [Inertia2T66(J)]
   
   if True: #manual mode
       KT=KinematicTree66(listOfJointTypes=jointTypes, listOfTransformations=transformations, 
                          listOfInertias=inertias, gravity=gravity3D)
   else:
       KT = newRobot.GetKinematicTree66()
   
   # acc=KT.ForwardDynamicsCRB([0]*n,[0]*n)
   # print("acc=",acc)
   #[M,f]=KT.ComputeMassMatrixAndForceTerms([1]*n,[2]*n)
   # print("M=",M.round(3), ",\n f=", f)
   
       
   
       
   if useKT:
       def UFgenericODE2(mbs, t, itemIndex, q, q_t):
           #f = np.array([sin(t*2*pi)*0.01]*n)    
           [M,f]=KT.ComputeMassMatrixAndForceTerms(q, q_t)
           #exu.Print("t =", t, ", f =", f)
           return -f
       
       M=np.eye(n)
       def UFmassGenericODE2(mbs, t, itemIndex, q, q_t):
           [M,f]=KT.ComputeMassMatrixAndForceTerms(q,q_t)
           return M
       
       def UFgraphics(mbs, itemNumber):
           #t = mbs.systemData.GetTime(exu.ConfigurationType.Visualization) #get time if needed
           nn = mbs.GetObjectParameter(itemNumber,'nodeNumbers')[0] #get first node
           q = mbs.GetNodeOutput(nodeNumber=nn, variableType = exu.OutputVariableType.Coordinates,
                                 configuration = exu.ConfigurationType.Visualization)
           if isinstance(q,float): #q is scalar if dimension is 1
               q=[q]
       
           graphicsList = []
           T66 = np.eye(6) #initial transformation
           
           #only valid for chains: !!!!
           pPrev = [0,0,0]
           for i in range(n):
               # T66prev = T66
               [XJ, MS] = JointTransformMotionSubspace66(KT.jointTypes[i], q[i])
               T66 = XJ @ KT.XL(i) @ T66
   
               [A, p] = T66toRotationTranslation(T66)
               p = -A.T@p
               A = A.T
   
               # alternative:            
               # H = T66toHT(T66Inverse(T66))
               # p = HT2translation(H)
               # A = HT2rotationMatrix(H)
   
               vAxis = A@np.array([0,0,0.25*L])
               graphicsList += [graphics.Cylinder(pAxis=p-0.5*vAxis, vAxis=vAxis, radius=0.05*L, color=graphics.color.red)]
               cube= graphics.Brick(centerPoint=[0.5*L,0,0], size=[L,0.08*L,0.15*L], color=graphics.color.brown)
               cube2 = graphics.Move(cube, p, A)
               graphicsList += [cube2]
               #graphicsList += [{'type':'Line', 'data': list(p)+list(p0), 'color':graphics.color.blue}]
           return graphicsList
   
       nGeneric=mbs.AddNode(NodeGenericODE2(referenceCoordinates = [0]*n, 
                                      initialCoordinates = [0]*n, 
                                      initialCoordinates_t= [0]*n,
                                      numberOfODE2Coordinates = n))
       
       mbs.AddObject(ObjectGenericODE2(nodeNumbers = [nGeneric], 
                                       # massMatrix=M, 
                                       # stiffnessMatrix=K, 
                                       # dampingMatrix=D, 
                                       # forceVector=fv, 
                                       forceUserFunction=UFgenericODE2, 
                                       massMatrixUserFunction=UFmassGenericODE2,
                                       visualization=VObjectGenericODE2(graphicsDataUserFunction=UFgraphics)))
       
       sJointsList += [mbs.AddSensor(SensorNode(nodeNumber=nGeneric, fileName='solution/genericNode.txt',
                                outputVariableType=exu.OutputVariableType.Coordinates))]
       sCases += [' KT T66']
   
   
   if useKT2:
       KT2=KinematicTree33(listOfJointTypes=jointTypes, 
                         listOfRotations=Amat,
                         listOfOffsets=vVec,
                         listOfInertia3D=listInertia,
                         listOfCOM=listCOM, 
                         listOfMass=listMass,
                         gravity=gravity3D)
       # acc=KT.ForwardDynamicsCRB([0]*n,[0]*n)
       # print("acc=",acc)
       [M2,f2]=KT2.ComputeMassMatrixAndForceTerms([1]*n,[2]*n)
       # print("M2=",M2.round(3), ",\n f2=", f2)
       def UFgenericODE2KT2(mbs, t, itemIndex, q, q_t):
           #f = np.array([sin(t*2*pi)*0.01]*n)    
           [M,f]=KT2.ComputeMassMatrixAndForceTerms(q, q_t)
           #exu.Print("t =", t, ", f =", f)
           return -f
       
       M=np.eye(n)
       def UFmassGenericODE2KT2(mbs, t, itemIndex, q, q_t):
           [M,f]=KT2.ComputeMassMatrixAndForceTerms(q,q_t)
           return M
       
       def UFgraphicsKT2(mbs, itemNumber):
           #t = mbs.systemData.GetTime(exu.ConfigurationType.Visualization) #get time if needed
           nn = mbs.GetObjectParameter(itemNumber,'nodeNumbers')[0] #get first node
           q = mbs.GetNodeOutput(nodeNumber=nn, variableType = exu.OutputVariableType.Coordinates,
                                 configuration = exu.ConfigurationType.Visualization)
           if isinstance(q,float): #q is scalar if dimension is 1
               q=[q]
       
           graphicsList = []
           T = HT0() #initial transformation
           pPrev = [0,0,0]
           for i in range(n):
               # T66prev = T66
               [A, v, rotAxis, transAxis] = JointTransformMotionSubspace(KT2.listOfJointTypes[i], q[i])
               XL = KT2.XL(i)
               XLHT = HT(XL[0],XL[1])
               T = T @ XLHT @ HT(A.T,v) #A is inverse transform
   
               p = HT2translation(T)
               A = HT2rotationMatrix(T)
   
               vAxis = A@np.array([0,0,0.28*L])
               graphicsList += [graphics.Cylinder(pAxis=p-0.5*vAxis, vAxis=vAxis, radius=0.045*L, color=graphics.color.red)]
               cube= graphics.Brick(centerPoint=[0.5*L,0,0], size=[L,0.085*L,0.145*L], color=graphics.color.steelblue)
               cube2 = graphics.Move(cube, p, A)
               graphicsList += [cube2]
               #graphicsList += [{'type':'Line', 'data': list(p)+list(p0), 'color':graphics.color.blue}]
           return graphicsList
   
       nGeneric2=mbs.AddNode(NodeGenericODE2(referenceCoordinates = [0]*n, 
                                      initialCoordinates = [0]*n, 
                                      initialCoordinates_t= [0]*n,
                                      numberOfODE2Coordinates = n))
   
       mbs.AddObject(ObjectGenericODE2(nodeNumbers = [nGeneric2],
                                       forceUserFunction=UFgenericODE2KT2, 
                                       massMatrixUserFunction=UFmassGenericODE2KT2,
                                       visualization=VObjectGenericODE2(graphicsDataUserFunction=UFgraphicsKT2)))
       
       sJointsList += [mbs.AddSensor(SensorNode(nodeNumber=nGeneric2, fileName='solution/genericNode2.txt',
                                outputVariableType=exu.OutputVariableType.Coordinates))]
       sCases += [' KT eff']
   
   
   if useKTcpp:
   
       if False:
           offsetsList = exu.Vector3DList(vVec)
           rotList = exu.Matrix3DList(Amat)
           linkCOMs=exu.Vector3DList(listCOM)
           linkInertiasCOM=exu.Matrix3DList(listInertiaCOM)
           linkForces = exu.Vector3DList([[0.,0.,0.]]*n)
           linkTorques = exu.Vector3DList([[0.,0.,0.]]*n)
           
       
           #graphics for link and joint:
           gLink =  graphics.Brick(centerPoint= [0.5*L,0,0], size= [L,w,w], color= graphics.color.dodgerblue)
           gJoint = graphics.Cylinder([0,0,-1.25*w], [0,0,2.5*w], 0.4*w, color=graphics.color.grey)
           gList = [[gLink,gJoint]]*n
       
           nGenericCpp = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0.]*n,
                                                  initialCoordinates=[0.]*n,
                                                  initialCoordinates_t=[0.]*n,
                                                  numberOfODE2Coordinates=n))
       
           VKT = VObjectKinematicTree(graphicsDataList = gList)
           mbs.AddObject(ObjectKinematicTree(nodeNumber=nGenericCpp, jointTypes=[exu.JointType.RevoluteZ]*n, linkParents=np.arange(n)-1,
                                             jointTransformations=rotList, jointOffsets=offsetsList, linkInertiasCOM=linkInertiasCOM,
                                             linkCOMs=linkCOMs, linkMasses=listMass, 
                                             baseOffset = [0.,0.,0.], gravity=np.array(gravity3D), jointForceVector=[0.]*n,
                                             linkForces = linkForces, linkTorques = linkTorques, 
                                             visualization=VKT))
       else:
           #use Robot class function:
           dKT = newRobot.CreateKinematicTree(mbs)
           nGenericCpp = dKT['nodeGeneric']
   
       sJointsList += [mbs.AddSensor(SensorNode(nodeNumber=nGenericCpp, fileName='solution/genericNodeCpp.txt',
                                                outputVariableType=exu.OutputVariableType.Coordinates))]
       
       sCases += [' KT cpp']
   
   #exu.Print(mbs)
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   
   tEnd = 1
   h = 1e-2 #0.1
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = h
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #SHOULD work with 0.9 as well
   
   useGraphics=True
   if True:
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
   
       mbs.SolveDynamic(simulationSettings, solverType = exu.DynamicSolverType.ExplicitEuler)
       # mbs.SolveDynamic(simulationSettings)
   
   #u1 = mbs.GetNodeOutput(nGeneric, exu.OutputVariableType.Coordinates)
   for i in range(len(sJointsList)):
       s = sJointsList[i]
       qq = mbs.GetSensorValues(s)
       exu.Print("joint angles =", qq, ", case ", sCases[i])
   
   if False: #use this to reload the solution and use SolutionViewer
       #sol = LoadSolutionFile('coordinatesSolution.txt')
       
       mbs.SolutionViewer() #can also be entered in IPython ...
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   
   if False:
       
       mbs.PlotSensor(sensorNumbers=[sGeneric], components=[0])
   
   
   
   
   
   
   


