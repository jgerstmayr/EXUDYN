
.. _testmodels-rotatingtabletest:

********************
rotatingTableTest.py
********************

You can view and download this file on Github: `rotatingTableTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/rotatingTableTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  RollindDisc test model with moving ground
   #
   # Author:   Johannes Gerstmayr
   # Date:     2023-01-02
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   #%%
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.graphicsDataUtilities import *
   
   import numpy as np
   from math import pi, sin, cos, exp, sqrt
   
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
   useGraphics = False #without test
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   uTest = 0 #reference solution
   
   for mode in range(2):
       mbs.Reset()
       #Dimensions
       mass = 30               #wheel mass in kg
       r = 0.5                 #wheel diameter in m
       d = 3                   #frame length in m
       g = 9.81                  #gravity constant (in )
       
       #drawing parameters:
       w=0.1                   #width for drawing
       
       
       #%%++++++++++++++++++++++++++++++++++
       gravity = [0,-g,0] #gravity
       vDisc = np.array([0,0,1])
       p0Wheel = [0,0,d]            #initial position of wheel
       
       p0Plane = [0,-r,0]
       n0Plane = np.array([0,1,0])
       
       iWheel = InertiaCylinder(1000, w, r, axis=2) 
       
       graphicsBodyWheel = [graphics.Brick(size=[1.2*r,1.2*r, w*3], color=graphics.color.lightred, addEdges=True)]
       graphicsBodyWheel+= [graphics.Cylinder(pAxis=[0,0,-0.5*w], vAxis=[0,0,w], radius=r, color=graphics.color.dodgerblue, nTiles=32, addEdges=True)]
       graphicsBodyWheel+= [graphics.Cylinder(pAxis=[0,0,-d], vAxis=[0,0,d], radius=0.5*w, 
                                                 color=graphics.color.orange, addEdges=True)]
       bWheel = mbs.CreateRigidBody(inertia = iWheel, 
                                    referencePosition = p0Wheel, 
                                    gravity = gravity, 
                                    graphicsDataList = graphicsBodyWheel)
       
       #ground body and marker
       p0Ground= p0Plane
       iPlane = RigidBodyInertia(mass=100, inertiaTensor=np.eye(3)*1000)
       graphicsPlane = [graphics.CheckerBoard(point=[0,0,0], normal=n0Plane, size=6*d)]
       oGround = mbs.AddObject(ObjectGround(referencePosition=p0Plane))
       
       #ground is also a movable rigid body
       bPlane = mbs.CreateRigidBody(inertia = iPlane, 
                                    referencePosition = p0Plane, 
                                    gravity = gravity, 
                                    graphicsDataList = graphicsPlane)
       
       #++++++++++++++++++++
       #joint for plane
       markerSupportGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
       markerSupportPlane = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bPlane, localPosition=[0,0,0]))
       #marker at wheel fixed to frame:
       
       mbs.AddObject(GenericJoint(markerNumbers=[markerSupportGround,markerSupportPlane],
                                   constrainedAxes=[1,1,1,1,0,1],
                                   visualization=VObjectJointGeneric(axesRadius=0.01, axesLength=0.12)))
       
       oTSD = mbs.AddObject(TorsionalSpringDamper(markerNumbers=[markerSupportGround,markerSupportPlane],
                                                  stiffness=0, damping=0,
                                                  rotationMarker0=RotationMatrixX(0.5*pi), #rotation marker is around z-axis=>change to y-axis
                                                  rotationMarker1=RotationMatrixX(0.5*pi),
                                                  ))
       #++++++++++++++++++++
       #joint between wheel/frame and ground:
       #marker for fixing frame
       markerSupportGroundWheel = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,r,0]))
       #marker at wheel fixed to frame:
       markerWheelFrame = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bWheel, localPosition=[0,0,-d]))
       
       kSD = 1e6
       dSD = 1e4
       mbs.AddObject(RigidBodySpringDamper(markerNumbers=[markerWheelFrame,markerSupportGroundWheel], 
                                           stiffness=kSD*np.diag([1,1,1,0,0,0]), 
                                           damping=dSD*np.diag([1,1,1,0,0,0]) ))
       
       #generic joint shows joint frames, are helpful to understand ...
       # mbs.AddObject(GenericJoint(markerNumbers=[markerWheelFrame,markerSupportGroundWheel],
       #                             constrainedAxes=[1,1,1,0,0,0],
       #                             visualization=VObjectJointGeneric(axesRadius=0.01, axesLength=0.12)))
       
       mbs.AddLoad(Torque(markerNumber=markerWheelFrame, 
                          loadVector=[0,0,20], bodyFixed=True))
       
       #++++++++++++++++++++
       #rolling disc joint:
       markerWheelCenter = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bWheel, localPosition=[0,0,0]))
       markerRollingPlane = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bPlane, localPosition=[0,0,0]))
       
       if True:
           if mode==0:
               oRolling=mbs.AddObject(ObjectJointRollingDisc(markerNumbers=[markerRollingPlane,markerWheelCenter], 
                                                             constrainedAxes=[1,1,1], 
                                                             discRadius=r, 
                                                             discAxis=vDisc,
                                                             planeNormal = n0Plane,
                                                             visualization=VObjectJointRollingDisc(show=False,discWidth=w,color=graphics.color.blue)))
           else:
               nGeneric = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
               oRolling=mbs.AddObject(RollingDiscPenalty(markerNumbers=[markerRollingPlane,markerWheelCenter],
                                                         nodeNumber=nGeneric,
                                                           discRadius=r, 
                                                           discAxis=vDisc,
                                                           planeNormal = n0Plane, contactStiffness=kSD, contactDamping=dSD, 
                                                           dryFriction=[0.5,0.5], dryFrictionProportionalZone=0.01, 
                                                           useLinearProportionalZone=True,
                                                           visualization=VObjectJointRollingDisc(show=False,discWidth=w,color=graphics.color.blue)))
       
           sForce = mbs.AddSensor(SensorObject(objectNumber=oRolling, storeInternal=True,
                                               outputVariableType = exu.OutputVariableType.ForceLocal))
           
           sTrailVel = mbs.AddSensor(SensorObject(objectNumber=oRolling, storeInternal=True,
                                              outputVariableType = exu.OutputVariableType.Velocity))
           
       
       # nGeneric0 = mbs.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
       # oRolling=mbs.AddObject(ObjectConnectorRollingDiscPenalty(markerNumbers=[markerRollingPlane,markerWheelCenter],
       #                                                          nodeNumber=nGeneric0,
       #                                                          contactStiffness=1e5, contactDamping=1e3,
       #                                                          discRadius=r, 
       #                                                          discAxis=AA@vDisc,
       #                                                          planeNormal = AA@n0Plane,
       #                                                          visualization=VObjectJointRollingDisc(discWidth=w,color=graphics.color.blue)))
       
       sAngVel = mbs.AddSensor(SensorBody(bodyNumber=bWheel, storeInternal=True,
                                          outputVariableType = exu.OutputVariableType.AngularVelocity))
       sAngVelLocal = mbs.AddSensor(SensorBody(bodyNumber=bWheel, storeInternal=True,
                                          outputVariableType = exu.OutputVariableType.AngularVelocityLocal))
       sAngAcc = mbs.AddSensor(SensorBody(bodyNumber=bWheel, storeInternal=True,
                                          outputVariableType = exu.OutputVariableType.AngularAcceleration))
       
       
       def PreStepUserFunction(mbs, t):
           if abs(t-2.5) < 1e-4:
               mbs.SetObjectParameter(oTSD, 'damping', 5000)
           return True
       
       mbs.SetPreStepUserFunction(PreStepUserFunction)
       
       mbs.Assemble()
       
       
       simulationSettings = exu.SimulationSettings() #takes currently set values or default values
       
       tEnd = 5
       h = 0.005
       simulationSettings.timeIntegration.endTime = tEnd #0.2 for testing
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       #simulationSettings.solutionSettings.solutionWritePeriod = 0.01
       #simulationSettings.solutionSettings.sensorsWritePeriod = 0.01
       simulationSettings.timeIntegration.verboseMode = 1
       simulationSettings.displayStatistics = True
   
       # simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
       simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
       simulationSettings.timeIntegration.newton.useModifiedNewton = True
       
       simulationSettings.timeIntegration.simulateInRealtime = useGraphics
       
       SC.visualizationSettings.connectors.showJointAxes = True
       SC.visualizationSettings.connectors.jointAxesLength = 0.3
       SC.visualizationSettings.connectors.jointAxesRadius = 0.08
       SC.visualizationSettings.openGL.lineWidth=2 #maximum
       SC.visualizationSettings.openGL.lineSmooth=True
       SC.visualizationSettings.openGL.shadow=0.15
       SC.visualizationSettings.openGL.multiSampling = 4
       SC.visualizationSettings.openGL.light0position = [8,8,10,0]
       simulationSettings.solutionSettings.solutionInformation = "Example Kollermill"
       SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
       
       if useGraphics:
           SC.renderer.Start()
           if 'renderState' in exu.sys:
               SC.renderer.SetState(exu.sys['renderState'])
           SC.renderer.DoIdleTasks()
       
       mbs.SolveDynamic(simulationSettings, 
                        solverType=exu.DynamicSolverType.TrapezoidalIndex2,
                        #showHints=True
                        )
       
       if useGraphics:
           SC.renderer.DoIdleTasks()
           SC.renderer.Stop() #safely close rendering window!
       
       sol2 = mbs.systemData.GetODE2Coordinates(); 
       u = np.linalg.norm(sol2); 
       exu.Print("rotatingTableTest mode",mode, "=", u)
       uTest += u
   
   exu.Print("rotatingTableTest=", uTest)
   
   exudynTestGlobals.testError = (uTest - 7.838680371309492) 
   exudynTestGlobals.testResult = uTest
   
   #%%+++++++++++++++++++++++
   if useGraphics:
       
       mbs.PlotSensor(closeAll=True) 
       mbs.PlotSensor(sensorNumbers=[sForce], components=[0,1,2]) 
       mbs.PlotSensor(sensorNumbers=sAngVel, components=[0,1,2]) 
       mbs.PlotSensor(sensorNumbers=sAngVelLocal, components=[0,1,2]) 
       mbs.PlotSensor(sensorNumbers=sAngAcc, components=[0,1,2]) 
   
       mbs.PlotSensor(sensorNumbers=sTrailVel, components=[0,1,2]) 
       mbs.PlotSensor(sensorNumbers=sTrailVel, components=exu.plot.componentNorm, 
                  newFigure=False, colorCodeOffset=3, labels=['trail velocity norm']) 
   
       forceEnd=mbs.GetSensorValues(sensorNumber=sForce)
       exu.Print('sForce: ',forceEnd)
       
       angAcc0=mbs.GetSensorStoredData(sensorNumber=sAngAcc)[0,1:]
       exu.Print('angAcc: ',angAcc0)
       
       

