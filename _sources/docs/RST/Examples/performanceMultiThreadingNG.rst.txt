
.. _examples-performancemultithreadingng:

******************************
performanceMultiThreadingNG.py
******************************

You can view and download this file on Github: `performanceMultiThreadingNG.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/performanceMultiThreadingNG.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test performance of unconstrained bodies
   #           Uses NGsolve multi-threading tools
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-09-15
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.graphicsDataUtilities import *
   
   
   #import numpy as np
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   print("version=", exu.config.Version())
   
   #nBodiesList = [10,20,50,100,200,500,1000,2000,5000,10000]
   nBodiesList = [20,40,100,400,1000,5000]
   #nBodiesList = [10,100,1000,10000]
   results = []
   
   for nBodies in nBodiesList:
   
       mbs.Reset()
   
       a=0.03
       gBody = graphics.BrickXYZ(-a, -a, -a, a, a, a, color=graphics.color.steelblue)
   
       m = 1 #mass
       J=[1,2,3,0,0,0] #inertia
       omega0 = [0.01,0.03,0]
   
       useEulerParameters = False
   
       if useEulerParameters:
           ep_t0 = list(AngularVelocity2EulerParameters_t(omega0, eulerParameters0))
           for i in range(nBodies):
               n = mbs.AddNode(RigidEP(referenceCoordinates=[0.1*i,0,0]+list(eulerParameters0), 
                                       initialVelocities=[0,0,0]+ ep_t0))
               o = mbs.AddObject(RigidBody(physicsMass=m, physicsInertia=J, nodeNumber=n, 
                                           visualization=VObjectRigidBody(graphicsData=[gBody])))
       else:
           for i in range(nBodies):
               n = mbs.AddNode(RigidRxyz(referenceCoordinates=[0.1*i,0,0]+[0]*3, 
                                       initialVelocities=[0,0,0]+ omega0))
               o = mbs.AddObject(RigidBody(physicsMass=m, physicsInertia=J, nodeNumber=n, 
                                           visualization=VObjectRigidBody(graphicsData=[gBody])))
   
       mbs.Assemble()
   
       simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
       tEnd = 1*1000/nBodies
       h=0.0005 #no visual differences for step sizes smaller than 0.0005
   
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.solutionSettings.writeSolutionToFile = False
   
       #simulationSettings.solutionSettings.solutionWritePeriod = 0.01
       #simulationSettings.solutionSettings.sensorsWritePeriod = h*4
       simulationSettings.timeIntegration.verboseMode = 1
       simulationSettings.timeIntegration.verboseModeFile = 0
   
       simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
       simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
       simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
       simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5#0.5
       simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
   
       simulationSettings.linearSolverType=exu.LinearSolverType.EigenSparse
       simulationSettings.displayComputationTime = True
       #simulationSettings.displayStatistics = True
   
       SC.visualizationSettings.nodes.show = True
       SC.visualizationSettings.nodes.drawNodesAsPoint  = False
       SC.visualizationSettings.nodes.showBasis = True
       SC.visualizationSettings.nodes.basisSize = 0.015
   
       #SC.renderer.Start()
       #SC.renderer.DoIdleTasks()
   
       threadsList = [1,2,3,4,6,8,10,12]#,14,16]#,20,24]
       #threadsList = [1,4,10]#,14,16]#,20,24]
       tTotalList = []
       tODE2RHSlist = []
       tMassMatrixList = []
       tNewtonIncrementList = []
   
       for nThreads in threadsList:
           simulationSettings.parallel.numberOfThreads = nThreads
           #print("=====================================================")
           print("compute with", nThreads, " threads")
           #print("=======================================")
           solver = exu.MainSolverImplicitSecondOrder()
           solver.SolveSystem(mbs, simulationSettings)
   
           #tNewtonIncrement = solver.timer.newtonIncrement
   
           tTotalList += [solver.timer.total]
           tODE2RHSlist += [solver.timer.ODE2RHS]
           tMassMatrixList += [solver.timer.massMatrix]
           tNewtonIncrementList += [solver.timer.newtonIncrement]
   
       print("=====================================================")
       print("nBodies          =",nBodies)
       print("thread count     =",threadsList)
       print("CPU times        =",tTotalList)
       print("ODE2RHS times    =",tODE2RHSlist)
       print("massMatrix times =",tMassMatrixList)
       print("newtonIncrement t=",tNewtonIncrementList)
   
       results += [{'nBodies':nBodies,
                    'threads':threadsList,
                    'CPU time':tTotalList,
                    'RHS CPU time':tODE2RHSlist,
                    'mass matrix CPU time':tMassMatrixList,
                    'newton increment CPU time':tNewtonIncrementList,
                    }]
   
       #SC.renderer.DoIdleTasks()
       #SC.renderer.Stop() #safely close rendering window!
   
   print("results=",results)
   


