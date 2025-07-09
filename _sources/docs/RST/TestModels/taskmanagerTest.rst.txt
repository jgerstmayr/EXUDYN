
.. _testmodels-taskmanagertest:

******************
taskmanagerTest.py
******************

You can view and download this file on Github: `taskmanagerTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/taskmanagerTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Systematic tests with several types of models, object types, and constraints;
   #           test for implicit, explicit and static solvers for number of threads in task manager
   #
   # Author:   Johannes Gerstmayr
   # Date:     2024-10-08
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
   
   useGraphics = False
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   resultTotal = 0
   #general dimentions
   a = 1
   b = 0.5
   g = [0,-9.81,0]
   mass = 0.2
   stiffness = 1000
   damping = 2
   load = 2
   totalCases = 0
   
   #user function for spring force
   def springForce(mbs, t, itemIndex, u, v, k, d, offset): #changed 2023-01-21:, mu, muPropZone):
       return k*u+k*u**3+v*d
   
   #user function for load
   def userLoadVector(mbs, t, loadVector):
       return np.sin(4*pi*t)*np.array(loadVector)
   
   nMasses = 1
   caseConstraints = 0
   caseODE1 = 0
   caseUser = 0
   caseStatic = 0
   nThreads = 1
   
   for nMasses in [0, 1, 2, 4]:
       for caseConstraints in range(2):
           for caseStatic in range(2):
               for caseUser in range(2):
                   for caseODE1 in range(2):
                       for nThreads in [1,2,5]:
                           mbs.Reset()
                           
                           if caseStatic and caseODE1: continue
                           if nMasses == 0 and not caseODE1: continue
                           
                           useExplicitSolver = (nMasses == 2 and caseConstraints == 0 and caseStatic == 0)
   
                           if not caseUser:
                               springForce = 0
                               
                           case = [nThreads, caseODE1, nMasses, caseConstraints, caseUser, 
                                   caseStatic, int(useExplicitSolver)]
                           
                           ground0 = mbs.CreateGround()
                           ground1 = mbs.CreateGround(referencePosition=[0,b,0])
                           bLast0 = ground0
                           bLast1 = ground1
                           
                           for i in range(nMasses):
                               xOff = i*a
                               b0=mbs.CreateMassPoint(referencePosition=[xOff+a, 0, 0],
                                                   physicsMass=mass,
                                                   drawSize = 0.1*a, color=graphics.color.red,
                                                   create2D=True,
                                                   gravity = g)
                               b1=mbs.CreateMassPoint(referencePosition=[xOff+a, b, 0],
                                                   physicsMass=mass,
                                                   drawSize = 0.1*a, color=graphics.color.red,
                                                   create2D=True,
                                                   gravity = g)
                           
                               mbs.CreateSpringDamper(bodyNumbers=[bLast1, b1], stiffness=stiffness, damping=damping,
                                                      springForceUserFunction=springForce)
                               if caseConstraints:
                                   mbs.CreateDistanceConstraint(bodyNumbers=[bLast0, b0])
                                   mbs.CreateDistanceConstraint(bodyNumbers=[bLast1, b0])
                                   mbs.CreateDistanceConstraint(bodyNumbers=[b0, b1])
                               else:
                                   mbs.CreateSpringDamper(bodyNumbers=[bLast0, b0], stiffness=stiffness, damping=damping)
                                   mbs.CreateSpringDamper(bodyNumbers=[bLast1, b0], stiffness=stiffness, damping=damping)
                                   mbs.CreateSpringDamper(bodyNumbers=[b0, b1], stiffness=stiffness, damping=damping)
                               
                               bLast0 = b0
                               bLast1 = b1
                           
                           if caseUser:
                               mbs.CreateForce(bodyNumber=bLast1, 
                                               loadVector=[10,20,0],
                                               loadVectorUserFunction=userLoadVector)
                           
                           sLast = mbs.AddSensor(SensorBody(bodyNumber=bLast0, storeInternal=True,
                                                 outputVariableType=exu.OutputVariableType.Displacement))
                           
                           sODE1 = None
                           if caseODE1:
                               #set up a 2-DOF system
                               nODE1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0,0,1],
                                                                   initialCoordinates=[1,0,0],
                                                                   numberOfODE1Coordinates=3))
                               
                               #build system matrix and force vector
                               #undamped mechanical system with m=1, K=100, f=1
                               #additionally 1 ODE1 coordinate
                               Amat = np.array([[0,1,0],
                                             [-100,0,0],
                                             [0,0,-10]])
                               bVec = np.array([0,1,50])
                               
                               oGenericODE1 = mbs.AddObject(ObjectGenericODE1(nodeNumbers=[nODE1],
                                                                              systemMatrix=Amat,
                                                                              rhsVector=bVec))
                               sODE1 = mbs.AddSensor(SensorNode(nodeNumber=nODE1, storeInternal=True,
                                                     outputVariableType=exu.OutputVariableType.Coordinates))
                           
                           
                           mbs.Assemble()
                           
                           tEnd = 0.1*20
                           h = 0.01
                           simulationSettings = exu.SimulationSettings()
                           simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
                           simulationSettings.timeIntegration.endTime = tEnd
                           # simulationSettings.displayStatistics = True
                           # simulationSettings.displayComputationTime = True
                           simulationSettings.timeIntegration.verboseMode = 0
                           simulationSettings.staticSolver.verboseMode = 0
                           
                           simulationSettings.parallel.numberOfThreads = nThreads
                           
                           SC.visualizationSettings.nodes.drawNodesAsPoint = False
                           SC.visualizationSettings.nodes.tiling = 32
                           SC.visualizationSettings.window.alwaysOnTop = True
                           
                           #start solver:
                           if useGraphics:
                               SC.renderer.Start()
                               SC.renderer.DoIdleTasks()
                           
                           if caseStatic:
                               mbs.SolveStatic(simulationSettings)
                           else:
                               if not useExplicitSolver:
                                   mbs.SolveDynamic(simulationSettings)
                               else:
                                   solverType = exu.DynamicSolverType.RK44
                                   if caseUser and not caseODE1:
                                       solverType = exu.DynamicSolverType.VelocityVerlet
   
                                   mbs.SolveDynamic(simulationSettings,
                                                    solverType=solverType)
                           
                           if useGraphics:
                               SC.renderer.Stop()
                           
                           #+++++++++++++++++++++++++++++++++++++
                           #evaluate results
                           x = mbs.GetSensorValues(sLast)
                           resultTotal += sum(x)
                           output = 'tip-disp='+str(list(x))
                           
                           if caseODE1:
                               x = mbs.GetSensorValues(sODE1)
                               output += ', ODE1='+str(list(x))
                               resultTotal += sum(x)
                           
                           if True:
                               exu.Print('** CASE '+str(totalCases)+' **:',case,
                                         '; RESULTS:',output)
   
                           totalCases += 1
   
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #evaluate final (=current) output values
   resultTotal *= 0.001 #to avoid too large non-deterministic round-off errors due to multithreading
   exu.Print('\ntotal cases:', totalCases)
   exu.Print('result taskmanagerTest=', resultTotal)
   
   exudynTestGlobals.testResult = resultTotal #-0.17210771618100057 
   
   # mbs.SolutionViewer()
   
   #%%++++++++++++++++
   if False:
       mbs.PlotSensor(sLast, components=[1])
       if sODE1 != None:
           mbs.PlotSensor(sODE1, components=[0,1,2])


