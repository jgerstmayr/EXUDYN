
.. _testmodels-ballbearingtest:

******************
ballBearingTest.py
******************

You can view and download this file on Github: `ballBearingTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/ballBearingTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for ContactSphereTorus
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-05-17
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   import numpy as np
   
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
   useGraphics = False #do test
   
   from exudyn.machines import GetBallBearingData, CreateBallBearing
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++
   #simple bearing example
   #+++++++++++++++++++++++++++++++++++++++++++
   #6010/C3 example data sheet:
   outsideDiameter=2*0.040
   boreDiameter=2*0.025
   width=0.016
   ## measured:
   nBalls=14
   radiusCage=0.0325
   radiusBalls=0.00873/2 #usually diameter is given
   outerRingShoulderRadius = 0.035125
   innerRingShoulderRadius = 0.029875
   ## computed:
   innerGrooveRadius = radiusBalls*1.25 #assumption
   outerGrooveRadius = radiusBalls*1.25 #assumption
   innerEdgeChamfer = 0.001  #in fact, it is round
   outerEdgeChamfer = 0.001
   
   #+++++++++++++++++++++++++++++++++++++++++++
   #create overall data for bearing, incl. drawing
   axis=[0,0,1]
   bearingData = GetBallBearingData(axis, outsideDiameter, boreDiameter, width, nBalls, 
                                    radiusBalls=radiusBalls, ballsAngleOffset=0,
                                    radiusCage=radiusCage, 
                                    innerGrooveRadius=innerGrooveRadius, outerGrooveRadius=outerGrooveRadius,
                                    innerEdgeChamfer=innerEdgeChamfer, outerEdgeChamfer=outerEdgeChamfer,
                                    innerRingShoulderRadius=innerRingShoulderRadius,outerRingShoulderRadius=outerRingShoulderRadius,
                                    )
   bearingGraphics = graphics.BallBearingRings(**bearingData, nTilesRings=64)
   
   
   #+++++++++++++++++++++++++++++++++++++++++++
   #we need to create markers for outer ring and inner ring:
   shaftRadius = boreDiameter*0.5
   shaftLength = 0.120
   density = 7800
   gBack = graphics.CheckerBoard([0,0,-shaftLength*0.55], size=outsideDiameter*2)
   
   oGround = mbs.CreateGround(graphicsDataList=[gBack])
   #fixed outer ring:
   mOuterRing = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
   
   #create body for inner ring:
   omega = 2*pi*2
   gravity = [0,-g,0]
   f = 0.002
   shaftGraphics = graphics.SolidOfRevolution([0,0,0], axis, 
                               contour=[[-0.5*shaftLength,0],[-0.5*shaftLength,shaftRadius-f],
                                        [-0.5*shaftLength+f,shaftRadius],[0.5*shaftLength-f,shaftRadius],
                                        [0.5*shaftLength,shaftRadius-f],[0.5*shaftLength,0],], 
                               nTiles=64, color=graphics.color.steelblue)
   
   bodyInner = mbs.CreateRigidBody(referencePosition=[0,0,0],
                                   nodeType=exu.NodeType.RotationRxyz,
                                   inertia=InertiaCylinder(density, shaftLength, shaftRadius, axis=2),
                                   initialAngularVelocity=[0,0,omega],
                                   gravity = gravity,
                                   graphicsDataList=[bearingGraphics['innerRingGraphics'],
                                                     graphics.Basis(origin=[0,0,width],length=1.5*shaftRadius),
                                                     shaftGraphics
                                                     ],
                               )
   #add coordinate constraint to keep velocity constant:
   mbs.CreateCoordinateConstraint(bodyNumbers=[bodyInner, None], 
                                  coordinates=[5,None],
                                  velocityLevel=True, offset=omega, show=False)
   
   mInnerRing = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bodyInner, localPosition=[0,0,0]))
   
   #++++++++++++++++++++++++++++++++++
   contactParametersRingBalls={'contactStiffness':5e6,'contactDamping':50,
                               'dynamicFriction':0.2,'contactStiffnessExponent':1,
                               #'restitutionCoefficient':0.5,'impactModel':2
                               'frictionProportionalZone':1e-2,
                               }
   
   listContactSensors = []
   bearingData['radiusBalls'] *= 1.01 #increase radius for prestressed-configuration
   bearingItems = CreateBallBearing(mbs, bearingData, 
                                    mInnerRing, mOuterRing, density, density,
                                    cageInitialAngularVelocity=[0,0,0], #not correct
                                    ballsInitialAngularVelocity=[0,0,0],
                                    gravity=gravity,
                                    springStiffnessCage=1e5, springDampingCage=1e2,
                                    contactParametersRingBalls=contactParametersRingBalls)
   
   #add sensor for trace
   sPosC=mbs.AddSensor(SensorObject(objectNumber=bearingItems['innerRingBallContacts'][0], storeInternal=True,
                       outputVariableType=exu.OutputVariableType.Position))
   listContactSensors.append(sPosC)
   sPosC=mbs.AddSensor(SensorObject(objectNumber=bearingItems['outerRingBallContacts'][0], storeInternal=True,
                       outputVariableType=exu.OutputVariableType.Position))
   listContactSensors.append(sPosC)
   
   #++++++++++++++++++++++++++++++++++
   #put outer ring graphics here for transparency:
   oGround = mbs.CreateGround(graphicsDataList=[bearingGraphics['outerRingGraphics'],
                                                graphics.Brick(centerPoint=[outsideDiameter*0.6,0,0],
                                                               size=[outsideDiameter*0.25,outsideDiameter*0.2,width], color=graphics.color.brown[0:3]+[0.4]),
                                                graphics.Brick(centerPoint=[-outsideDiameter*0.6,0,0],
                                                               size=[outsideDiameter*0.25,outsideDiameter*0.2,width], color=graphics.color.brown[0:3]+[0.4]),
                                                ])
                
   #++++++++++++++++++++++++++++++++++
   
   timeStartBB = 2 if useGraphics else 0
   
   def UFforce(mbs, t, loadVector):
       global timeStartBB
       ts = timeStartBB
       f0 = 200
       force=0 if t < ts else min(f0*(t-ts),f0)
       if t>2*ts:
           force=max(-f0, f0-2*f0*(t-2*ts))
       if t>3*ts:
           force=min(0, -f0+f0*(t-3*ts))
       
       return [0,0,force]
   
   def UFtorque(mbs, t, loadVector):
       ts = 3.5*timeStartBB
       t0 = 8 #Nm
       torque=0 if t < ts or t > ts+1 else min(t0*(t-ts),t0)
       if t > ts+1 and t < ts+2:
           torque = max(t0*(ts+2-t),0)
       return [torque,0,0]
   
   mbs.CreateForce(bodyNumber=bodyInner, localPosition=[0,0,0],
                   loadVectorUserFunction=UFforce)
   mbs.CreateTorque(bodyNumber=bodyInner, 
                    loadVectorUserFunction=UFtorque)
   
   
   
   mbs.Assemble()
   
   tEnd = 0.05
   if useGraphics:
       tEnd = 10
   
   stepSize = 1e-4
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.004
   simulationSettings.solutionSettings.sensorsWritePeriod = stepSize  #output interval
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.displayStatistics = True
   simulationSettings.displayComputationTime = True
   #simulationSettings.timeIntegration.simulateInRealtime = True
   #simulationSettings.timeIntegration.realtimeFactor = 0.5
   #simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-2
   # simulationSettings.timeIntegration.discontinuous.maxIterations = 2
   #simulationSettings.timeIntegration.discontinuous.useRecommendedStepSize = False
   simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-8
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.parallel.numberOfThreads = 1
   
   simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   
   SC.visualizationSettings.window.renderWindowSize=[1600,1400]
   SC.visualizationSettings.openGL.multiSampling=2
   SC.visualizationSettings.openGL.shadow = 0.25
   #SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.loads.show = False
   SC.visualizationSettings.loads.drawSimplified=False
   SC.visualizationSettings.nodes.basisSize = radiusBalls*1.5
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   
   SC.visualizationSettings.sensors.traces.listOfPositionSensors = listContactSensors
   SC.visualizationSettings.sensors.traces.showPositionTrace = True if len(listContactSensors) else False
   SC.visualizationSettings.sensors.traces.timeSpan = 1.6
   
   if useGraphics:
       SC.renderer.Start()              #start graphics visualization
       SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   #start solver:q
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics:
       SC.renderer.Stop()               #safely close rendering window!
   
   #%%++++
   testError = 0.01*np.linalg.norm(mbs.systemData.GetODE2Coordinates())
   exu.Print('solution of ballBearingTest=',testError)
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   exudynTestGlobals.testError = testError - (0.0)   #2023-06-12: 4.172189649307425
   exudynTestGlobals.testResult = testError
   
   if useGraphics:
       #%%
       mbs.SolutionViewer()
   


