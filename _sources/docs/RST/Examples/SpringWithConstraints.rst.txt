
.. _examples-springwithconstraints:

************************
SpringWithConstraints.py
************************

You can view and download this file on Github: `SpringWithConstraints.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/SpringWithConstraints.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  example of SpringDampers and CoordinateConstraints
   #
   # Author:   Johannes Gerstmayr
   # Date:     2019-11-22
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #add constraints for testing:
   L = 1
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
   mGroundCoordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action
   mGroundPosition = mbs.AddMarker(MarkerNodePosition(nodeNumber = nGround)) #Ground node ==> no action
   
   n0 = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,0]))
   n1 = mbs.AddNode(NodePoint(referenceCoordinates=[L,0,0]))
   mN0 = mbs.AddMarker(MarkerNodePosition(nodeNumber = n0)) 
   mN1 = mbs.AddMarker(MarkerNodePosition(nodeNumber = n1)) 
   
   mbs.AddObject(SpringDamper(markerNumbers = [mN0, mN1], stiffness = 1000, referenceLength=L))
   mbs.AddLoad(Force(markerNumber=mN1, loadVector=[10,0,0]))
   
   
   mNC1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = n1, coordinate=2))
   mNC2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = n1, coordinate=1))
   
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundCoordinate,mNC1]))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundCoordinate,mNC2]))
   
   mNC00 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = n0, coordinate=0))
   mNC01 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = n0, coordinate=1))
   mNC02 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = n0, coordinate=2))
   
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundCoordinate,mNC00]))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundCoordinate,mNC01]))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGroundCoordinate,mNC02]))
   
   
   
   mbs.Assemble()
   print(mbs)
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   f = 100000
   simulationSettings.timeIntegration.numberOfSteps = 1*f
   simulationSettings.timeIntegration.endTime = 0.002*f
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/f
   simulationSettings.displayComputationTime = True
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8*1000 #10000
   simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10*100
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = False
   simulationSettings.timeIntegration.newton.maxModifiedNewtonIterations = 5
   simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
   simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 6.055454452393343e-06*0.1 #eps^(1/3)
   simulationSettings.timeIntegration.newton.modifiedNewtonContractivity = 1000
   simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
   simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6
   simulationSettings.displayStatistics = True
   
   #SC.visualizationSettings.nodes.showNumbers = True
   SC.visualizationSettings.bodies.showNumbers = True
   #SC.visualizationSettings.connectors.showNumbers = True
   SC.visualizationSettings.nodes.defaultSize = 0.05
   
   simulationSettings.solutionSettings.solutionInformation = "Planar four-bar-mechanism with initial angular velocity and gravity"
   
   #SC.renderer.Start()
   #mbs.SolveDynamic(simulationSettings)
   #SC.renderer.DoIdleTasks()
   #SC.renderer.Stop() #safely close rendering window!
   
   simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 0.1
   simulationSettings.staticSolver.verboseMode = 3
   SC.renderer.Start()
   mbs.SolveStatic(simulationSettings)
   SC.renderer.DoIdleTasks()
   SC.renderer.Stop() #safely close rendering window!
   
   


