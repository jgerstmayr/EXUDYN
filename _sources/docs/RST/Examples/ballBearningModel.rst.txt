
.. _examples-ballbearningmodel:

********************
ballBearningModel.py
********************

You can view and download this file on Github: `ballBearningModel.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/ballBearningModel.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test bearing model with two ball bearings on rigid shaft
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-05-17
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import *
   import exudyn.graphics as graphics
   import numpy as np
   
   from exudyn.machines import GetBallBearingData, CreateBallBearing
   
   
   useGraphics = True #without test
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
                                    radiusCage=radiusCage, heightCage=radiusBalls*0.2,
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
   gBack = graphics.CheckerBoard([0,0,-shaftLength*0.55], size=outsideDiameter*4)
   
   oGround = mbs.CreateGround(graphicsDataList=[gBack])
   
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
                               )
   
   #add coordinate constraint to keep velocity constant:
   mbs.CreateCoordinateConstraint(bodyNumbers=[bodyInner, None], 
                                  coordinates=[5,None],
                                  velocityLevel=True, offset=omega, show=False)
   
   graphicsDataListInner = [shaftGraphics, graphics.Basis(origin=[0,0,width],length=1.5*shaftRadius),]
   offsetList = [-shaftLength*0.3, shaftLength*0.3, ]
   for offsetZ in offsetList:
       #fixed outer ring:
       mOuterRing = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,offsetZ]))
       #rotating inner ring:
       mInnerRing = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bodyInner, localPosition=[0,0,offsetZ]))
       
       graphicsDataListInner+=[graphics.Move(bearingGraphics['innerRingGraphics'],[0,0,offsetZ])]
   
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
       oGroundDrawing = mbs.CreateGround(referencePosition=[0,0,offsetZ],
                                         graphicsDataList=[bearingGraphics['outerRingGraphics'],
                                                    graphics.Brick(centerPoint=[outsideDiameter*0.6,0,0],
                                                                   size=[outsideDiameter*0.25,outsideDiameter*0.2,width], color=graphics.color.brown[0:3]+[0.4]),
                                                    graphics.Brick(centerPoint=[-outsideDiameter*0.6,0,0],
                                                                   size=[outsideDiameter*0.25,outsideDiameter*0.2,width], color=graphics.color.brown[0:3]+[0.4]),
                                                    ])
                
   mbs.SetObjectParameter(bodyInner, 'VgraphicsData', graphicsDataListInner)
   #++++++++++++++++++++++++++++++++++
   
   def UFforce(mbs, t, loadVector):
       ts = 2
       f0 = 200
       force=0 if t < ts else min(f0*(t-ts),f0)
       if t>2*ts:
           force=max(-f0, f0-2*f0*(t-2*ts))
       if t>3*ts:
           force=min(0, -f0+f0*(t-3*ts))
       
       return [0,0,force]
   
   def UFtorque(mbs, t, loadVector):
       ts = 7
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
   
   stepSize = 1e-4
   tEnd = 10
   
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
   #SC.visualizationSettings.nodes.tiling = 32
   SC.visualizationSettings.nodes.drawNodesAsPoint = False
   SC.visualizationSettings.connectors.showContact = False
   #SC.visualizationSettings.general.sphereTiling = 32
   
   SC.visualizationSettings.sensors.traces.listOfPositionSensors = listContactSensors
   SC.visualizationSettings.sensors.traces.showPositionTrace = True if len(listContactSensors) else False
   SC.visualizationSettings.sensors.traces.timeSpan = 1.6
   
   SC.renderer.Start()              #start graphics visualization
   SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   #start solver:q
   mbs.SolveDynamic(simulationSettings)
   
   # SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
   SC.renderer.Stop()               #safely close rendering window!
   
   if True:
       #%%
       mbs.SolutionViewer()
   


