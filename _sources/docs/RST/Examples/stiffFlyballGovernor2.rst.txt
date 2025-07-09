
.. _examples-stiffflyballgovernor2:

************************
stiffFlyballGovernor2.py
************************

You can view and download this file on Github: `stiffFlyballGovernor2.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/stiffFlyballGovernor2.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Stiff flyball governor with rigid and compliant joints (IFToMM benchmark problem);
   #           Ref.: https://www.iftomm-multibody.org/benchmark/problem/Stiff_flyball_governor/
   #           This version uses the newer C++ implemented Lie group solvers
   #
   # Model:    Flyball governor as redundant multibody system
   #
   # Author:   Johannes Gerstmayr, Stefan Holzinger
   # Date:     2020-02-13
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   # *clean example*
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   ## import libaries
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.graphicsDataUtilities import *
   
   import numpy as np
   from numpy import linalg as LA
   
   ## set up MainSystem mbs
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   ## 
   useCompliantCase = False
   useLieGroup = useCompliantCase
   
   
   ## create background graphics and ground object
   color = [0.1,0.1,0.8,1]
   r = 0.2 #radius
   L = 1   #length
   
   background0 = GraphicsDataRectangle(-L,-L,L,L,color)
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background0])))
   
   # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ## set up body dimensions according to reference in m
   
   # shaft
   lengthShaft = 1     #z
   widthShaft  = 0.01  #=height
   
   # rod
   lengthRod = 1
   widthRod  = 0.01    #=height
   
   # slider
   dimSlider = 0.1 #x=y=z
   sSlider = 0.5
   
   # scalar distance between point A and B
   xAB = 0.1   
   beta0 = np.deg2rad(30) 
   initAngleRod = np.deg2rad(60)
   
   # initial angular velocity of shaft and slider
   omega0 = [0., 0., 0.16*2*np.pi]
   
   # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ## body masses according to reference in kg
   
   density = 3000
   
   mShaft        = 0.3
   mRod          = 0.3
   mSlider       = 3
   mMassPoint    = 5
   mRodMassPoint = mRod + mMassPoint
   
   
   # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # gravity
   g = [0,0,-9.81]
   
   ## setup inertia for rod along x-direction
   iRod = InertiaCuboid(density=density, sideLengths=[lengthRod,widthRod,0.01])
   iMass = InertiaMassPoint(mass=mMassPoint).Translated([lengthRod/2,0,0])
   iRodSum = iRod+iMass
   
   #compute reference point of rod (midpoint)
   refRod = -iRodSum.com
   iRodSum = iRodSum.Translated(refRod)
   
   if useLieGroup:
       nodeType = exu.NodeType.RotationRotationVector
   else:
       nodeType = exu.NodeType.RotationEulerParameters
   
   
   nRigidBodyNodes = 4
   #nRB=[-1]*nRigidBodyNodes #final node numbers
   
   ## create inertia for shaft and slider
   inertiaList=[InertiaCuboid(density=density, sideLengths=[widthShaft,widthShaft,lengthShaft]),
                InertiaCuboid(density=density, sideLengths=[dimSlider,dimSlider,dimSlider]),
                iRodSum, iRodSum]
   
   ## set up reference position list
   refPosList=[[0,0,lengthShaft/2], # shaft
               [0,0,sSlider], # slider
               [ xAB/2 + (lengthRod/2-refRod[0])*np.cos(beta0), 0, lengthShaft - (lengthRod/2-refRod[0])*np.sin(beta0)], # rodAC
               [-xAB/2 - (lengthRod/2-refRod[0])*np.cos(beta0), 0, lengthShaft - (lengthRod/2-refRod[0])*np.sin(beta0)]] # rodBD
   
   ## set up initial velocity vector list
   refVelList = [[0., 0., 0.], # shaft
                 [0., 0., 0.], # slider
                 [0,omega0[2]*refPosList[2][0],0], # rodAC
                 [0,omega0[2]*refPosList[3][0],0]] # rodBD
   
   ## set up initial (global) angular velocity vector list
   refAngularVelList = [omega0,     # shaft
                        omega0,     # slider
                        omega0,    # rodAC
                        omega0]    # rodBD
   
   ## create graphics objects for bodies
   graphicsRodAC  = graphics.BrickXYZ(-(lengthRod/2-refRod[0]),-widthRod/2,-widthRod/2, lengthRod/2+refRod[0],widthRod/2,widthRod/2, [0.1,0.1,0.8,1])
   graphicsRodBD  = graphics.BrickXYZ(-lengthRod/2-refRod[0],-widthRod/2,-widthRod/2, lengthRod/2-refRod[0],widthRod/2,widthRod/2, [0.1,0.1,0.8,1])
   graphicsSlider = graphics.BrickXYZ(-dimSlider/2,-dimSlider/2,-dimSlider/2, dimSlider/2,dimSlider/2,dimSlider/2, [0.1,0.1,0.8,1])
   graphicsShaft  = graphics.BrickXYZ(-widthShaft/2,-widthShaft/2,-lengthShaft/2, widthShaft/2,widthShaft/2,lengthShaft/2, [0.1,0.1,0.8,1])
   
   #lists for 4 nodes/bodies: [shaft, slider, rodAC, rodBD]
   graphicsList=[graphicsShaft, graphicsSlider, graphicsRodAC, graphicsRodBD]
   
   #eulerParameters0 = [1, 0, 0, 0]
   rotParList = [] 
   if nodeType == exu.NodeType.RotationEulerParameters:
       refRotParList = [eulerParameters0,             # shaft
                        eulerParameters0,             # slider
                        RotationMatrix2EulerParameters(RotationMatrixY(beta0)),   # rodAC
                        RotationMatrix2EulerParameters(RotationMatrixY(-beta0))]  # rodBD
       refRotMatList = [EulerParameters2RotationMatrix(refRotParList[0]),
                        EulerParameters2RotationMatrix(refRotParList[1]),
                        EulerParameters2RotationMatrix(refRotParList[2]),
                        EulerParameters2RotationMatrix(refRotParList[3])]
       
   elif nodeType == exu.NodeType.RotationRxyz:
       refRotParList = [[0,0,0],       # shaft
                        [0,0,0],       # slider
                        [0,beta0,0],   # rodAC
                        [0,-beta0,0]]  # rodBD
       refRotMatList = [RotXYZ2RotationMatrix(refRotParList[0]),
                        RotXYZ2RotationMatrix(refRotParList[1]),
                        RotXYZ2RotationMatrix(refRotParList[2]),
                        RotXYZ2RotationMatrix(refRotParList[3])]
       
   elif nodeType == exu.NodeType.RotationRotationVector:
       refRotParList = [[0,0,0],       # shaft
                        [0,0,0],       # slider
                        [0,beta0,0],   # rodAC
                        [0,-beta0,0]]  # rodBD
       refRotMatList = [RotationVector2RotationMatrix(refRotParList[0]),
                        RotationVector2RotationMatrix(refRotParList[1]),
                        RotationVector2RotationMatrix(refRotParList[2]),
                        RotationVector2RotationMatrix(refRotParList[3])]
       
   # add rigid bodies to mbs
   nodeNumberList = [-1]*nRigidBodyNodes
   bodyNumberList = [-1]*nRigidBodyNodes
   for i in range(nRigidBodyNodes):    
       dictBody = mbs.CreateRigidBody(
                     inertia=inertiaList[i], 
                     nodeType=nodeType, 
                     referencePosition=refPosList[i], 
                     referenceRotationMatrix=refRotMatList[i],
                     initialVelocity=refVelList[i],
                     initialAngularVelocity=refAngularVelList[i], 
                     gravity=g,
                     graphicsDataList=[graphicsList[i]],
                     returnDict=True)
       [n0, b0] = [dictBody['nodeNumber'], dictBody['bodyNumber']]
       nodeNumberList[i] = n0
       bodyNumberList[i] = b0
       
   
   
   
   # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ## spring-damper parameters for connecting the rods with the slider
   
   # spring
   k  = 8.e5*0.005 # spring stiffness in N/m
   l0 = 0.5  # relaxed spring length in m
   
   # damper
   c = 4.e4*0.005
   
   ## connecting points
   # slider
   pointEslider = [dimSlider/2, 0., 0.]
   pointFslider = [-dimSlider/2, 0., 0.]
   
   # connectin points for connecting rods with slider
   connectingPointRodACWithSlider = [refRod[0], 0, 0]
   connectingPointRodBDWithSlider = [-refRod[0], 0, 0]
   
   # connecting points for connecting rods with shaft
   pointA = [xAB/2, 0, lengthShaft/2]
   pointB = [-xAB/2, 0, lengthShaft/2]
   pointARodAC = [-(lengthRod/2-refRod[0]), 0, 0]
   pointARodBD = [(lengthRod/2-refRod[0]), 0, 0]
   
   # connecting point of shaft with ground
   connectingPointShaftWithGround = [0, 0, -lengthShaft/2]
   
   # markers
   markerShaftCOM     = mbs.AddMarker(MarkerBodyRigid(name='markerShaftCOM', bodyNumber=bodyNumberList[0], localPosition=[0,0,0]))
   markerShaftGround  = mbs.AddMarker(MarkerBodyRigid(name='markerShaftGround', bodyNumber=bodyNumberList[0], localPosition=connectingPointShaftWithGround))
   markerShaftPointA  = mbs.AddMarker(MarkerBodyRigid(name='markerShaftPointA', bodyNumber=bodyNumberList[0], localPosition=pointA))
   markerShaftPointB  = mbs.AddMarker(MarkerBodyRigid(name='markerShaftPointB', bodyNumber=bodyNumberList[0], localPosition=pointB))
   
   markerSliderCOM    = mbs.AddMarker(MarkerBodyRigid(name='markerSliderCOM', bodyNumber=bodyNumberList[1], localPosition=[0,0,0]))
   markerSliderPointE = mbs.AddMarker(MarkerBodyRigid(name='markerSliderPointE', bodyNumber=bodyNumberList[1], localPosition=pointEslider))
   markerSliderPointF = mbs.AddMarker(MarkerBodyRigid(name='markerSliderPointF', bodyNumber=bodyNumberList[1], localPosition=pointFslider))
   
   markerRodACShaft   = mbs.AddMarker(MarkerBodyRigid(name='markerRodACShaft', bodyNumber=bodyNumberList[2], localPosition=pointARodAC))
   markerRodACSlider  = mbs.AddMarker(MarkerBodyRigid(name='markerRodACSlider', bodyNumber=bodyNumberList[2], localPosition=connectingPointRodACWithSlider))
   
   markerRodBDShaft   = mbs.AddMarker(MarkerBodyRigid(name='markerRodBDShaft', bodyNumber=bodyNumberList[3], localPosition=pointARodBD))
   markerRodBDSlider  = mbs.AddMarker(MarkerBodyRigid(name='markerRodBDSlider', bodyNumber=bodyNumberList[3], localPosition=connectingPointRodBDWithSlider))
   
   
   
   oGround = mbs.AddObject(ObjectGround())
   markerGround = mbs.AddMarker(MarkerBodyRigid(name='markerGround', bodyNumber=oGround, localPosition=[0,0,0]))
   
   nj2=-1
   
   if not useCompliantCase:
       
       mbs.AddObject(GenericJoint(markerNumbers=[markerGround, markerShaftGround], constrainedAxes=[1,1,1,1,1,0],
                                   visualization=VObjectJointGeneric(axesRadius=0.01, axesLength=0.1)))
       
       mbs.AddObject(GenericJoint(markerNumbers=[markerShaftCOM, markerSliderCOM], constrainedAxes=[1*0,1*0,0,1,1,1],
                                   visualization=VObjectJointGeneric(axesRadius=0.01, axesLength=0.1)))
       
       mbs.AddObject(GenericJoint(markerNumbers=[markerShaftPointA, markerRodACShaft], constrainedAxes=[1,1,1,1,0,1],
                                   visualization=VObjectJointGeneric(axesRadius=0.01, axesLength=0.1)))
       
       mbs.AddObject(GenericJoint(markerNumbers=[markerShaftPointB, markerRodBDShaft], constrainedAxes=[1,1,1,1,0,1],
                                   visualization=VObjectJointGeneric(axesRadius=0.01, axesLength=0.1)))
   
   else:
       kj=1e5*0.2
       dj = kj*0.05
       
       kj2 = kj*0.05 #rotatory springs can be softer!
       dj2 = kj2*0.05
       
       mbs.AddObject(RigidBodySpringDamper(markerNumbers=[markerGround, markerShaftGround], 
                                           stiffness=np.diag([kj,kj,kj,kj2,kj2,0]), damping=np.diag([dj,dj,dj,dj2,dj2,0])))
       
       mbs.AddObject(RigidBodySpringDamper(markerNumbers=[markerShaftCOM, markerSliderCOM], 
                                           stiffness=np.diag([kj,kj,0,kj2,kj2,kj2]), damping=0*np.diag([dj,dj,0,0,0,0])))
       
       nj2 = mbs.AddObject(RigidBodySpringDamper(markerNumbers=[markerShaftPointA, markerRodACShaft], 
                                           stiffness=np.diag([kj,kj,kj,kj2,0,kj2]), damping=0.*np.diag([dj,dj,dj,0,0,0])))
       
       mbs.AddObject(RigidBodySpringDamper(markerNumbers=[markerShaftPointB, markerRodBDShaft], 
                                           stiffness=np.diag([kj,kj,kj,kj2,0,kj2]), damping=0.*np.diag([dj,dj,dj,0,0,0])))
   
   
   
   
   # spring-damper elements
   mbs.AddObject(SpringDamper(markerNumbers=[markerSliderPointE, markerRodACSlider], stiffness=k, damping=c, referenceLength=l0))
   mbs.AddObject(SpringDamper(markerNumbers=[markerSliderPointF, markerRodBDSlider], stiffness=k, damping=c, referenceLength=l0))
   
   mbs.AddSensor(SensorNode(nodeNumber = nodeNumberList[1], fileName='solution/flyballSliderPosition.txt',outputVariableType=exu.OutputVariableType.Position))
   mbs.AddSensor(SensorNode(nodeNumber = nodeNumberList[2], fileName='solution/flyballSliderRotation.txt',outputVariableType=exu.OutputVariableType.Rotation)) #Tait Bryan rotations
   mbs.AddSensor(SensorNode(nodeNumber = nodeNumberList[0], fileName='solution/flyballShaftAngularVelocity.txt',outputVariableType=exu.OutputVariableType.AngularVelocity))
   
   
   # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   mbs.Assemble()
   
   
   useGraphics=True
   if useGraphics: #only start graphics once, but after background is set
       SC.renderer.Start()
       #SC.renderer.DoIdleTasks()
       
   # dynamicSolver = exu.MainSolverImplicitSecondOrder()
   
   tEnd = 10
   h = 2e-5 #RK44
   #h = 1e-3
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   simulationSettings.timeIntegration.explicitIntegration.useLieGroupIntegration = useLieGroup
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   
   
   SC.visualizationSettings.markers.show = True
   #SC.visualizationSettings.markers.showNumbers = True
   
   #simulationSettings.displayComputationTime = True
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.solutionSettings.sensorsWritePeriod = simulationSettings.timeIntegration.endTime/2000
   simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/2000
   
   if nodeType != exu.NodeType.RotationRotationVector:
       simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True
   else:
       simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = False
   
       
   solverType = exu.DynamicSolverType.TrapezoidalIndex2
   if useLieGroup:
       solverType = exu.DynamicSolverType.RK44
       simulationSettings.timeIntegration.stepSizeSafety = 0.5 #almost no step rejection
       
   mbs.SolveDynamic(simulationSettings, solverType=solverType)
   print(mbs.sys['dynamicSolver'].it)
   
   
   if useGraphics: #only start graphics once, but after background is set
       SC.renderer.Stop() #safely close rendering window!
   
   
   for i in range(4):
       om=mbs.GetNodeOutput(i,exu.OutputVariableType.AngularVelocity)
       # exu.Print("om",i,"=",om)
   
   for i in range(4):
       vel=mbs.GetNodeOutput(i,exu.OutputVariableType.Velocity)
       # exu.Print("v",i,"=",vel)
   
   for i in range(2):
       rot=mbs.GetNodeOutput(i+2,exu.OutputVariableType.RotationMatrix)
       # exu.Print("Rot",i+2,"=",rot)
   
   result = mbs.GetNodeOutput(2,exu.OutputVariableType.Velocity)[1] #y-velocity of bar
   exu.Print('solution of stiffFlyballGovernor=',result)
   
   
   plist=[]
   plist += [mbs.GetObjectOutputBody(objectNumber = bodyNumberList[2], variableType = exu.OutputVariableType.Velocity, localPosition = list(pointARodAC), configuration =
   exu.ConfigurationType.Current)]
   plist += [mbs.GetObjectOutputBody(objectNumber = bodyNumberList[2], variableType = exu.OutputVariableType.Velocity, localPosition = connectingPointRodACWithSlider, configuration =
   exu.ConfigurationType.Current)]
   plist += [mbs.GetObjectOutputBody(objectNumber = bodyNumberList[3], variableType = exu.OutputVariableType.Velocity, localPosition = pointARodBD, configuration =
   exu.ConfigurationType.Current)]
   
   #locU = mbs.GetObjectOutput(objectNumber = nj2, variableType =exu.OutputVariableType.DisplacementLocal)
   #exu.Print('locU=', locU)
   #locR = mbs.GetObjectOutput(objectNumber = nj2, variableType =exu.OutputVariableType.Rotation)
   #exu.Print('locR=', locR)
   
   
   #Rxyz initial velocities:
   #om 0 = [0.         0.         6.28318531]
   #om 1 = [0.         0.         6.28318531]
   #om 2 = [ 0.00000000e+00 -8.54693196e-10  6.28318531e+00]
   #om 3 = [0.00000000e+00 8.54693196e-10 6.28318531e+00]
   #v 0 = [ 0.00000000e+00  0.00000000e+00 -4.90499796e-10]
   #v 1 = [ 0.00000000e+00  0.00000000e+00 -4.90499608e-10]
   #v 2 = [-1.91975841e-16  5.60155553e+00 -4.90500111e-10]
   #v 3 = [ 1.91975841e-16 -5.60155553e+00 -4.90500111e-10]
   
   if useGraphics:
       import matplotlib.pyplot as plt
       import matplotlib.ticker as ticker
       plt.close('all')
       
       data = np.loadtxt('solution/flyballSliderPosition.txt', comments='#', delimiter=',')
       #plt.plot(data[:,0], data[:,3], 'r-') #z coordinate of slider
       #data = np.loadtxt('solution/flyballShaftAngularVelocity.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,1], 'b-') #z coordinate of slider
       plt.plot(data[:,0], data[:,2], 'g-') #z coordinate of slider
       plt.plot(data[:,0], data[:,3], 'k-') #z coordinate of slider
   
       data = np.loadtxt('solution/flyballSliderRotation.txt', comments='#', delimiter=',')
       plt.plot(data[:,0], data[:,1], 'r--') #z coordinate of slider
       plt.plot(data[:,0], data[:,2], 'g--') #z coordinate of slider
       plt.plot(data[:,0], data[:,3], 'b--') #z coordinate of slider
   
       if False:
           #data = np.loadtxt('solution/flyballSliderPositionRxyz.txt', comments='#', delimiter=',')    #rigid joints?
           data = np.loadtxt('solution/flyballSliderPositionRK4Rxyz.txt', comments='#', delimiter=',') #compliant joints
           #plt.plot(data[:,0], data[:,3], 'r:') #z coordinate of slider
           plt.plot(data[:,0], data[:,1], 'b:') #z coordinate of slider
           plt.plot(data[:,0], data[:,2], 'g:') #z coordinate of slider
           plt.plot(data[:,0], data[:,3], 'k:') #z coordinate of slider
       
   #    data = np.loadtxt('solution/flyballSliderPositionRK4Rxyz.txt', comments='#', delimiter=',')
   #    plt.plot(data[:,0], data[:,3], 'g:') #z coordinate of slider
   #    data = np.loadtxt('solution/flyballShaftAngularVelocityRK4Rxyz.txt', comments='#', delimiter=',')
   #    plt.plot(data[:,0], data[:,3], 'k:') #z coordinate of slider
       
       ax=plt.gca() # get current axes
       ax.grid(True, 'major', 'both')
       ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
       ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
       plt.tight_layout()
       plt.show() 
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
    

