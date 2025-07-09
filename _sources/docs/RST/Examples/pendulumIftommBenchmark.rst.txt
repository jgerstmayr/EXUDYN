
.. _examples-pendulumiftommbenchmark:

**************************
pendulumIftommBenchmark.py
**************************

You can view and download this file on Github: `pendulumIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/pendulumIftommBenchmark.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Pendulum according to IFToMM multibody benchmarks
   #           https://www.iftomm-multibody.org/benchmark/problem/Planar_simple_pendulum/
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-06-15
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   L = 1 #distance
   mass = 1
   g = 9.81
   
   r = 0.05 #just for graphics
   graphicsBackground = GraphicsDataRectangle(-1.2*L,-1.2*L, 1.2*L, 0.2*L, [1,1,1,1]) #for appropriate zoom
   graphicsSphere = graphics.Sphere(point=[0,0,0], radius=r, color=[1.,0.2,0.2,1], nTiles = 16)
   
   #add ground object and mass point:
   oGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0], 
                              visualization = VObjectGround(graphicsData = [graphicsBackground])))
   nMass = mbs.AddNode(NodePoint2D(referenceCoordinates=[-L,0], 
                                   initialCoordinates=[0,0],
                                   initialVelocities=[0,0]))
   oMass = mbs.AddObject(MassPoint2D(physicsMass = mass, nodeNumber = nMass, 
                                     visualization = VObjectMassPoint2D(graphicsData = [graphicsSphere])))
   
   mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
   mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [0,0,0]))
   oDistance = mbs.AddObject(DistanceConstraint(markerNumbers = [mGround, mMass], distance = L))
   
   #add loads:
   mbs.AddLoad(Force(markerNumber = mMass, loadVector = [0, -mass*g, 0])) 
   
   addSensors = True
   if addSensors:
       sDist = mbs.AddSensor(SensorObject(objectNumber=oDistance, storeInternal=True, 
                                          outputVariableType=exu.OutputVariableType.Distance))
       sPos = mbs.AddSensor(SensorNode(nodeNumber=nMass, storeInternal=True, 
                                          outputVariableType=exu.OutputVariableType.Position))
       
       sVel = mbs.AddSensor(SensorNode(nodeNumber=nMass, storeInternal=True, 
                                       outputVariableType=exu.OutputVariableType.Velocity))
       
       
       def UFenergy(mbs, t, sensorNumbers, factors, configuration):
           pos = mbs.GetSensorValues(sensorNumbers[0])
           vel = mbs.GetSensorValues(sensorNumbers[1])
           Ekin = 0.5*mass*(vel[0]**2 + vel[1]**2)
           Epot = mass * g * pos[1]
           return [Ekin+Epot] #return total energy
       
       sEnergy = mbs.AddSensor(SensorUserFunction(sensorNumbers=[sPos,sVel], storeInternal=True,
                                                sensorUserFunction=UFenergy))
   
   #print(mbs)
   
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   
   #for performance (energy error < 5e-5J):
   #without sensors, takes 0.037 seconds on i7 surface book laptop
   endTime = 10
   stepSize = 0.8e-3
   
   #accuracy:
   # endTime = 9.99 #in benchmark results, values are only given until 9.99 seconds
   # stepSize = 1e-5
   
   
   simulationSettings.timeIntegration.numberOfSteps = int(endTime/stepSize)
   simulationSettings.timeIntegration.endTime = endTime
   simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/5
   simulationSettings.solutionSettings.sensorsWritePeriod = simulationSettings.timeIntegration.endTime/100
   #simulationSettings.displayComputationTime = True
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.timeIntegration.verboseModeFile = 0
   
   #these Newton settings are slightly faster than full Newton:
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   simulationSettings.timeIntegration.newton.modifiedNewtonJacUpdatePerStep = True
   
   #simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.75 
   #simulationSettings.timeIntegration.adaptiveStep = False
   
   #simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True
   #simulationSettings.solutionSettings.coordinatesSolutionFileName= "coordinatesSolution.txt"
   
   simulationSettings.displayStatistics = True
   #simulationSettings.solutionSettings.recordImagesInterval = 0.04
   
   SC.visualizationSettings.nodes.defaultSize = 0.05
   useGraphics = False
   
   if useGraphics:
       SC.renderer.Start()
   
   #SC.renderer.DoIdleTasks()
   mbs.SolveDynamic(simulationSettings, 
                    # solverType=exu.DynamicSolverType.TrapezoidalIndex2
                    )
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   
   #plot constraint error:
   if addSensors:
       
       mbs.PlotSensor(sensorNumbers=sDist, offsets=[-L], closeAll=True)
       mbs.PlotSensor(sensorNumbers=sPos, components=[0,1], newFigure=True)
       
       mbs.PlotSensor(sensorNumbers=sEnergy, yLabel='total energy', newFigure=True)


