
.. _examples-cranereevingsystem:

*********************
craneReevingSystem.py
*********************

You can view and download this file on Github: `craneReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/craneReevingSystem.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  A crane model using two objects ReevingSystemSprings for the rope and hoisting mechanism
   #           Tower as well as arm are modeled as rigid bodies connected by joints, such that the can move
   #
   # Model:    Crane with reeving system
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-06-16
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   # *clean example*
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   ## import exudyn package, utils and math packages
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   import numpy as np
   from math import sin, cos, sqrt,pi
   
   ## create system mbs
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   ## create ground with checker board background
   gGround = graphics.CheckerBoard(point=[0,0,0], normal = [0,1,0], size=60, nTiles=12)
   oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[gGround])))
   
   ## define parameters of crane
   tRoll = 0.05    #thickness rolls (graphics)
   rHook = 0.2     #radius of Hook rolls
   rCarr = 0.3     #radius of Carriage rolls
   g = [0,-9.81,0]
   colorRolls = graphics.color.red
   
   H = 40 #crane height
   L = 30 #boom length
   Dtower = 1.5
   Darm = 1
   Lcarr = 1
   Dcarr = 0.6
   Lhook = 0.5
   Dhook = 0.5
   
   ## define parameters of rope
   rRope = 0.025 #drawing radius
   A = rRope**2*pi
   EArope = 1e9*A
   print('EArope=',EArope)
   stiffnessRope = EArope #stiffness per length
   dampingRope = 0.1*stiffnessRope #dampiung per length
   dampingRopeTorsional = 1e-4*dampingRope
   dampingRopeShear = 0.1*dampingRope*0.001
   
   ## 
   #further crane parameters: (height=Y, arm=X)
   carrYoff = 0.3*Darm
   hookZoff = 0.4*Dhook
   hookYoff = 0.5*H
   #hookZoffVec = np.array([0,0,hookZoff])
   
   posTower = np.array([0,0.5*H,0])
   posArm = 2*posTower + np.array([0.5*L,0,0])
   
   sJoint = 0.5 #overal joint size for main joints
   
   #++++++++++++++++++++++++++++
   ## add ground node and ground marker for constraints
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))
   mNodeGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))#andy coordinate is zero
   
   ## add nodes and markers for prescribed motion in reeving system
   nCoordCarr = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0], initialCoordinates=[0], 
                                            initialCoordinates_t=[0], numberOfODE2Coordinates=1))
   nCoordHook = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0], initialCoordinates=[0], 
                                            initialCoordinates_t=[0], numberOfODE2Coordinates=1))
   mNodeCarr = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nCoordCarr, coordinate=0))
   mNodeHook = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nCoordHook, coordinate=0))
   
   ## add 1D mass object for dynamics of main rope drum
   mbs.AddObject(Mass1D(physicsMass=1, nodeNumber=nCoordCarr))
   mbs.AddObject(Mass1D(physicsMass=1, nodeNumber=nCoordHook))
   
   ## add coordinate constraint for prescribed motion using offset later on
   ccCarr = mbs.AddObject(CoordinateConstraint(markerNumbers=[mNodeGround, mNodeCarr], offset=0))
   ccHook = mbs.AddObject(CoordinateConstraint(markerNumbers=[mNodeGround, mNodeHook], offset=0))
   
   #++++++++++++++++++++++++++++
   ## set up rigid body for tower
   Vtower = Dtower*Dtower*H
   inertiaTower = InertiaCuboid(2000/Vtower, [Dtower,H,Dtower])
   
   ## model tower as body, which may be moved as well ...
   graphicsTower = [graphics.Brick([0,0,0],[Dtower,H,Dtower],color=graphics.color.grey, addEdges = True)]
   graphicsTower += [graphics.Cylinder([0,0.5*H-Darm*0.5,0],[0,0.5*Darm,0],radius=1.1*Darm, color=graphics.color.grey)]
   dictTower = mbs.CreateRigidBody(
                 inertia=inertiaTower, 
                 referencePosition=posTower,
                 gravity=g, 
                 graphicsDataList=graphicsTower,
                 returnDict=True)
   [nTower, bTower] = [dictTower['nodeNumber'], dictTower['bodyNumber']]
   
   ## add joint for tower
   markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
   markerTowerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bTower, localPosition=[0,-0.5*H,0]))
   oJointTower = mbs.AddObject(GenericJoint(markerNumbers=[markerGround, markerTowerGround],
                                       constrainedAxes=[1,1,1,1,1,1],
                                       visualization=VGenericJoint(axesRadius=0.5*sJoint, axesLength=1.5*sJoint)))
   
   
   #++++++++++++++++++++++++++++
   ## set up rigid body for carriage, initially located at midspan or arm
   Vcarr = Dcarr*Dcarr*Lcarr
   
   inertiaCarr = InertiaCuboid(100/Vcarr, [Lcarr,Dcarr,Dcarr])
   posCarr = posArm + np.array([0,-carrYoff,0])
   
   zRoll = np.array([0,0,tRoll])
   pRollCarr =  [None,None,None,None,None]
   pRollCarr[0] = [-0.5*Lcarr,0,-hookZoff]
   pRollCarr[1] = [-0.5*Lcarr,0, hookZoff]
   pRollCarr[2] = [ 0.5*Lcarr,0,-hookZoff]
   pRollCarr[3] = [ 0.5*Lcarr,0, hookZoff]
   pRollCarr[4] = [ 0.5*Lcarr,0, 0*hookZoff]
   
   ## add graphics data for carriage
   graphicsCarr = []
   for p in pRollCarr:
       graphicsCarr += [graphics.Cylinder(p-0.5*zRoll,zRoll,radius=rHook, color=colorRolls, addEdges=True)]
   
   graphicsCarr += [graphics.Brick([0,0,0],[1.2*Lcarr,0.2*Dcarr,1.2*Dcarr],color=graphics.color.grey[0:3]+[0.5], addEdges = True)]
   
   ### add rigid body for carriage
   dictCarr = mbs.CreateRigidBody(
                 inertia=inertiaCarr, 
                 referencePosition=posCarr, 
                 gravity=g, 
                 graphicsDataList=graphicsCarr,
                 returnDict=True)
   [nCarr, bCarr] = [dictCarr['nodeNumber'], dictCarr['bodyNumber']]
   
   #++++++++++++++++++++++++++++
   ## set up arm
   Varm = Darm*Darm*L
   inertiaArm = InertiaCuboid(2000/Varm, [L,Darm,Darm])
   
   pRollArm =  [None,None,None]
   pRollArm[0] = [-0.5*L, rCarr,0]
   pRollArm[1] = [ 0.5*L,0     ,0]
   pRollArm[2] = [-0.5*L,-rCarr,0]
   rRollArm = [0,rCarr,0]
   
   ## add tower as rigid body
   graphicsArm = []
   for i,p in enumerate(pRollArm):
       graphicsArm += [graphics.Cylinder(p-0.5*zRoll,zRoll,radius=max(0.1*rCarr,rRollArm[i]), color=colorRolls, addEdges=True)]
   
   graphicsArm += [graphics.Brick([-0.1*L,0,0],[L*1.2,Darm,Darm],color=[0.3,0.3,0.9,0.5], addEdges = True)]
   dictArm = mbs.CreateRigidBody(
                 inertia=inertiaArm, 
                 referencePosition=posArm, 
                 gravity=g, 
                 graphicsDataList=graphicsArm,
                 returnDict=True)
   [nArm, bArm] = [dictArm['nodeNumber'], dictArm['bodyNumber']]
   
   ## create revolute joint between tower and arm
   markerTowerArm = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bTower, localPosition=[0,0.5*H,0]))
   markerArmTower = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bArm, localPosition=[-0.5*L,0,0]))
   oJointArm = mbs.AddObject(GenericJoint(markerNumbers=[markerTowerArm, markerArmTower],
                                       constrainedAxes=[1,1,1,1,0,1],
                                       visualization=VGenericJoint(axesRadius=0.5*sJoint, axesLength=1.5*sJoint)))
   
   ## create prismatic joint between arm and carriage
   markerArmCarr = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bArm, localPosition=[0,-carrYoff,0]))
   markerCarrArm = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCarr, localPosition=[0,0,0]))
   oJointCarr = mbs.AddObject(GenericJoint(markerNumbers=[markerArmCarr, markerCarrArm],
                                       constrainedAxes=[0,1,1,1,1,1],
                                       visualization=VGenericJoint(axesRadius=0.5*sJoint, axesLength=1.5*sJoint)))
   
   #++++++++++++++++++++++++++++
   ## set up marker lists and local axes for sheaves for reeving system for motion of carriage at tower
   markerListCarriage1 = []
   markerListCarriage1+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bArm, localPosition=pRollArm[0]))]
   markerListCarriage1+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bArm, localPosition=pRollArm[1]))]
   markerListCarriage1+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCarr, localPosition=[ 0.5*Lcarr,0,0]))]
   markerListCarriage1+=[mNodeCarr,mNodeGround]
   
   LrefRopeCarriage1 = L+pi*rCarr+0.5*L-0.5*Lcarr
   
   sheavesAxes1 = exu.Vector3DList()
   for i, radius in enumerate(rRollArm):
       sheavesAxes1.Append([0,0,-1])
   
   ## set up marker lists and local axes for sheaves for reeving system for motion of carriage at arm end
   markerListCarriage2 = []
   markerListCarriage2+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bArm, localPosition=pRollArm[2]))]
   markerListCarriage2+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCarr, localPosition=[-0.5*Lcarr,0,0]))]
   markerListCarriage2+=[mNodeCarr,mNodeGround]
   
   LrefRopeCarriage2 = 0.5*L-0.5*Lcarr
   
   
   #needs just two points:
   sheavesAxes2 = exu.Vector3DList()
   sheavesAxes2.Append([0,0,-1])
   sheavesAxes2.Append([0,0,-1])
   
   ## create first reeving system object for carriage
   oRScarr1=mbs.AddObject(ReevingSystemSprings(markerNumbers=markerListCarriage1, 
                                               hasCoordinateMarkers=True, coordinateFactors=[-1,0],#negative direction X
                                               stiffnessPerLength=stiffnessRope, dampingPerLength=dampingRope, 
                                               referenceLength = LrefRopeCarriage1,
                                               dampingTorsional = dampingRopeTorsional, dampingShear = dampingRopeShear*0,
                                               sheavesAxes=sheavesAxes1, sheavesRadii=rRollArm,
                                               visualization=VReevingSystemSprings(ropeRadius=rRope, color=graphics.color.lawngreen)))
   
   ## create second reeving system object for carriage
   oRScarr2=mbs.AddObject(ReevingSystemSprings(markerNumbers=markerListCarriage2, 
                                               hasCoordinateMarkers=True, coordinateFactors=[1,0], #positive direction X
                                               stiffnessPerLength=stiffnessRope, dampingPerLength=dampingRope, 
                                               referenceLength = LrefRopeCarriage2,
                                               dampingTorsional = dampingRopeTorsional*0,
                                               sheavesAxes=sheavesAxes2, sheavesRadii=[0,0],
                                               visualization=VReevingSystemSprings(ropeRadius=rRope, color=graphics.color.lawngreen)))
   
   #++++++++++++++++++++++++++++
   ## set up inertia and parameters for hook
   Vhook = Dhook*Dhook*Lhook
   
   inertiaHook = InertiaCuboid(100/Vhook, [Lhook,Dhook,Dhook])
   inertiaHook = inertiaHook.Translated([0,-Dhook,0])
   posHook = posCarr + np.array([0,-hookYoff,0])
   
   
   pRollHook =  [None,None,None,None]
   pRollHook[0] = [-0.5*Lhook,0,-hookZoff]
   pRollHook[1] = [-0.5*Lhook,0, hookZoff]
   pRollHook[2] = [ 0.5*Lhook,0,-hookZoff]
   pRollHook[3] = [ 0.5*Lhook,0, hookZoff]
   
   ## set up graphics for hook
   graphicsHook = []
   for p in pRollHook:
       graphicsHook += [graphics.Cylinder(p-0.5*zRoll,zRoll,radius=rHook, color=colorRolls, addEdges=True)]
   
   graphicsHook += [graphics.Brick([0,0,0],[Lhook,0.2*Dhook,Dhook],color=graphics.color.grey[0:3]+[0.5], addEdges = True)]
   graphicsHook += [graphics.Brick([0,-Dhook,0],[4*Lhook,2*Dhook,2*Dhook],color=graphics.color.grey[0:3]+[0.5], addEdges = True)]
   
   ## add rigid body for hook
   dictHook = mbs.CreateRigidBody(
                 inertia=inertiaHook, 
                 referencePosition=posHook, 
                 initialAngularVelocity=[0,0,0],
                 gravity=g, 
                 graphicsDataList=graphicsHook,
                 returnDict=True)
   [nHook, bHook] = [dictHook['nodeNumber'], dictHook['bodyNumber']]
   
   
   ## create list of markers for hook-reeving system
   markerListHook = []
   markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bArm, localPosition=[-0.5*L,-carrYoff+2*rHook,0]))]
   markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCarr, localPosition=pRollCarr[0]))]
   markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bHook, localPosition=pRollHook[0]))]
   markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCarr, localPosition=pRollCarr[1]))]
   markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bHook, localPosition=pRollHook[1]))]
   markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCarr, localPosition=pRollCarr[3]))]
   markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bHook, localPosition=pRollHook[3]))]
   markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCarr, localPosition=pRollCarr[2]))]
   markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bHook, localPosition=pRollHook[2]))]
   markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bCarr, localPosition=pRollCarr[4]))]
   markerListHook+= [mbs.AddMarker(MarkerBodyRigid(bodyNumber=bArm, localPosition=[ 0.5*L,-carrYoff+2*rHook,0]))]
   markerListHook+=[mNodeHook,mNodeGround]
   
   LrefRopeHook = 8*0.5*H+L+8*pi*rHook
   
   ## create list of axes for reeving system of hook
   sheavesAxesHook = exu.Vector3DList()
   sheavesAxesHook.Append([0,0,1])
   sheavesAxesHook.Append([0,0,-1])
   sheavesAxesHook.Append([0,0,1]) #Hook0
   sheavesAxesHook.Append([0,0,1])
   sheavesAxesHook.Append([0,0,1]) #Hook1
   sheavesAxesHook.Append([0,0,-1])
   sheavesAxesHook.Append([0,0,-1]) #Hook2
   sheavesAxesHook.Append([0,0,-1])
   sheavesAxesHook.Append([0,0,-1]) #Hook3
   sheavesAxesHook.Append([0,0,-1])
   sheavesAxesHook.Append([0,0,1])
   
   radiiRollHook = []
   for i in range(len(sheavesAxesHook)):
       radiiRollHook += [rHook]
   
   ## create reeving system for hook
   oRScarr1=mbs.AddObject(ReevingSystemSprings(markerNumbers=markerListHook, 
                                               hasCoordinateMarkers=True, coordinateFactors=[1,0],
                                               stiffnessPerLength=stiffnessRope, dampingPerLength=dampingRope*0.1, referenceLength = LrefRopeHook,
                                               dampingTorsional = dampingRopeTorsional*0.1, dampingShear = dampingRopeShear,
                                               sheavesAxes=sheavesAxesHook, sheavesRadii=radiiRollHook,
                                               visualization=VReevingSystemSprings(ropeRadius=rRope, color=graphics.color.dodgerblue)))
   
   
   ## add sensors to show trace of hook
   sPosTCP = mbs.AddSensor(SensorNode(nodeNumber=nHook, storeInternal=True,
                                      outputVariableType=exu.OutputVariableType.Position))
   sRotTCP = mbs.AddSensor(SensorNode(nodeNumber=nHook, storeInternal=True,
                                      outputVariableType=exu.OutputVariableType.RotationMatrix))
   
   #%% +++++++++++++++++++++++++++++++
   # #add sensors 
   # if True:
   #     sPos1 = mbs.AddSensor(SensorNode(nodeNumber=nodeList[1], storeInternal=True,
   #                                           outputVariableType=exu.OutputVariableType.Position))
   #     sOmega1 = mbs.AddSensor(SensorNode(nodeNumber=nodeList[1], storeInternal=True,
   #                                           outputVariableType=exu.OutputVariableType.AngularVelocity))
   #     sLength= mbs.AddSensor(SensorObject(objectNumber=oRS, storeInternal=True,
   #                                           outputVariableType=exu.OutputVariableType.Distance))
   #     sLength_t= mbs.AddSensor(SensorObject(objectNumber=oRS, storeInternal=True,
   #                                           outputVariableType=exu.OutputVariableType.VelocityLocal))
   
   ## create pre-step user function to drive crane system over time
   def PreStepUserFunction(mbs, t):
       if t <= 10:
           mbs.SetObjectParameter(ccCarr, 'offset', SmoothStep(t,  0, 10, 0, 0.45*L))
       elif t <= 20:
           mbs.SetObjectParameter(ccHook, 'offset', SmoothStep(t, 10, 20, 0, -8*0.40*H))
       elif t <= 30:
           mbs.SetObjectParameter(ccCarr, 'offset', SmoothStep(t, 20, 30, 0.45*L,-0.4*L))
       elif t <= 40:
           mbs.SetObjectParameter(ccHook, 'offset', SmoothStep(t, 30, 40, -8*0.40*H,8*0.45*H))
       else:
           mbs.SetObjectParameter(ccCarr, 'offset', SmoothStep(t, 40, 57, -0.4*L, 0.45*L))
           mbs.SetObjectParameter(ccHook, 'offset', SmoothStep(t, 40, 60, 8*0.45*H, 6*0.45*H))
   
       return True
   
   ## add pre-step user function to mbs
   mbs.SetPreStepUserFunction(PreStepUserFunction)
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   ## assemble and add simulation settings
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 80
   h=0.001
   
   solutionFile = 'solution/coordsCrane.txt'
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.writeSolutionToFile= True #set False for CPU performance measurement
   simulationSettings.solutionSettings.solutionWritePeriod= 0.2
   simulationSettings.solutionSettings.coordinatesSolutionFileName = solutionFile
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.02
   # simulationSettings.timeIntegration.simulateInRealtime=True
   # simulationSettings.timeIntegration.realtimeFactor=5
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.01
   simulationSettings.parallel.numberOfThreads=4
   simulationSettings.displayComputationTime = True
   
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   if True:
       #traces:
       SC.visualizationSettings.sensors.traces.listOfPositionSensors = [sPosTCP]
       SC.visualizationSettings.sensors.traces.listOfTriadSensors =[sRotTCP]
       SC.visualizationSettings.sensors.traces.showPositionTrace=True
       SC.visualizationSettings.sensors.traces.showTriads=True
       SC.visualizationSettings.sensors.traces.triadSize=2
       SC.visualizationSettings.sensors.traces.showVectors=False
       SC.visualizationSettings.sensors.traces.showFuture=False
       SC.visualizationSettings.sensors.traces.triadsShowEvery=5
   
   
   SC.visualizationSettings.nodes.show = True
   SC.visualizationSettings.nodes.drawNodesAsPoint  = False
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.2
   
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.openGL.shadow = 0.3*0
   SC.visualizationSettings.openGL.light0position = [-50,200,100,0]
   
   SC.visualizationSettings.window.renderWindowSize=[1920,1200]
   #SC.visualizationSettings.general.autoFitScene = False #use loaded render state
   
   ## start renderer and dynamic simulation
   useGraphics = True
   if useGraphics:
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys[ 'renderState' ])
       SC.renderer.DoIdleTasks()
   
   
   mbs.SolveDynamic(simulationSettings, 
                    #solverType=exu.DynamicSolverType.TrapezoidalIndex2 #in this case, drift shows up significantly!
                    )
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   
   ## optionally start solution viewer at end of simulation
   if True:
       #%%++++++++++++
       
       SC.visualizationSettings.general.autoFitScene = False
       mbs.SolutionViewer() #loads solution file via name stored in mbs
   
   #%%++++++++++++
   ## optionally plot sensors for crane
   if False:
       
       mbs.PlotSensor(sPos1, components=[0,1,2], labels=['pos X','pos Y','pos Z'], closeAll=True)
       mbs.PlotSensor(sOmega1, components=[0,1,2], labels=['omega X','omega Y','omega Z'])
       mbs.PlotSensor(sLength, components=[0], labels=['length'])
       mbs.PlotSensor(sLength_t, components=[0], labels=['vel'])
   
   
   #compute error for test suite:
   sol2 = mbs.systemData.GetODE2Coordinates(); 
   u = np.linalg.norm(sol2); 
   exu.Print('solution of craneReevingSystem=',u)


