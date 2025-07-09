
.. _examples-ancfcablecantilevered:

************************
ANCFcableCantilevered.py
************************

You can view and download this file on Github: `ANCFcableCantilevered.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/ANCFcableCantilevered.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  ANCFCable test with many cable elements
   #
   # Author:   Johannes Gerstmayr
   # Date:     2023-10-15
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.beams import *
   
   import numpy as np
   
   #create an environment for mini example
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))
   
   np.random.seed(0) #reproducible results (also for SolutionViewer when reloading ...)
   
   rhoA = 78.
   EA = 100000.
   EI = 200
   
   nCables = 10000
   for i in range(nCables):
       p0 = np.random.rand(3)
       if i < nCables/2:
           p0[0]=0.5
           p1 = p0 + [2+np.random.rand()*0.25,0,0]
       else:
           p0[0]=-0.5
           p1 = p0 - [2+np.random.rand()*0.25,0,0]
       
   
       cable = ObjectANCFCable(physicsMassPerLength=rhoA, 
                     physicsBendingStiffness=EI*(0.5+np.random.rand()*0.25), 
                     physicsBendingDamping = EI,
                     physicsAxialStiffness=EA, 
                     )
   
       ancf=GenerateStraightLineANCFCable(mbs=mbs,
                     positionOfNode0=p0, positionOfNode1=p1,
                     numberOfElements=12, #converged to 4 digits
                     cableTemplate=cable, #this defines the beam element properties
                     massProportionalLoad = [0,-9.81,0],
                     fixedConstraintsNode0 = [1,1,1, 0,1,1], #add constraints for pos and rot (r'_y,r'_z)
                     )
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   
   endTime=10
   stepSize = 0.5e-3
   
   simulationSettings = exu.SimulationSettings()
   
   #simulationSettings.solutionSettings.writeSolutionToFile = False
   simulationSettings.solutionSettings.solutionWritePeriod = 0.02 #data not used
   simulationSettings.solutionSettings.binarySolutionFile = True
   simulationSettings.solutionSettings.outputPrecision = 6 #float
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.002 #data not used
   simulationSettings.timeIntegration.verboseMode = 1 #turn off, because of lots of output
   simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   simulationSettings.parallel.numberOfThreads = 8
   simulationSettings.displayComputationTime = True
   simulationSettings.displayStatistics = True
   
   simulationSettings.timeIntegration.numberOfSteps = int(endTime/stepSize)
   simulationSettings.timeIntegration.endTime = endTime
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   #simulationSettings.timeIntegration.simulateInRealtime = True
   #simulationSettings.timeIntegration.realtimeFactor = 0.5
   
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
   SC.visualizationSettings.window.renderWindowSize=[1200,1024]
   SC.visualizationSettings.nodes.show = False
   SC.visualizationSettings.loads.show = False
   SC.visualizationSettings.connectors.show = False
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   SC.renderer.Start()
   #SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   SC.renderer.DoIdleTasks()
   SC.renderer.Stop() #safely close rendering window!
   
   #mbs.SolutionViewer()


