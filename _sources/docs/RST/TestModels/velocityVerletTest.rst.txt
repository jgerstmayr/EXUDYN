
.. _testmodels-velocityverlettest:

*********************
velocityVerletTest.py
*********************

You can view and download this file on Github: `velocityVerletTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/velocityVerletTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for Velocity Verlet integrator;
   #           Note: only second order accurate for conservative systems
   #
   # Author:   Johannes Gerstmayr
   # Date:     2024-10-06
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
   
   SC = exu.SystemContainer()         #container of systems
   mbs = SC.AddSystem()               #add a new system to work with
   
   mass = mbs.CreateMassPoint(referencePosition=[1,0,0], 
                              initialVelocity=[1,0,0], 
                              #gravity=[10,0,0],
                              physicsMass=10, 
                              graphicsDataList=[graphics.Sphere(radius=0.2,color=graphics.color.red)])
   ground = mbs.CreateGround()
   mbs.CreateSpringDamper(bodyNumbers=[ground, mass], referenceLength=1, 
                          stiffness = 1000, damping = 0,
                          drawSize = 0.1, 
                          )
   sMass = mbs.AddSensor(SensorBody(bodyNumber = mass, storeInternal=True, 
                                    outputVariableType=exu.OutputVariableType.Position))
   
   mbs.Assemble()                     #assemble system and solve
   simulationSettings = exu.SimulationSettings()
       
   if useGraphics: #only start graphics once, but after background is set
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   sumSol = 0
   for i in range(4):
       h = 0.001 / 2**i
       tEnd = 2
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.verboseMode=1 #provide some output
       # simulationSettings.timeIntegration.simulateInRealtime = True
   
       #SC.renderer.DoIdleTasks()
       mbs.SolveDynamic(simulationSettings, solverType = exudyn.DynamicSolverType.VelocityVerlet)
       #mbs.SolveDynamic(simulationSettings, solverType = exudyn.DynamicSolverType.ExplicitEuler)
       # mbs.SolveDynamic(simulationSettings, solverType = exudyn.DynamicSolverType.ExplicitMidpoint)
       #mbs.SolveDynamic(simulationSettings, solverType = exudyn.DynamicSolverType.RK67)
       p = mbs.GetSensorValues(sMass)
       #ref solution with RK67 !
       #err = p - [1.0332409398209812, 0.0, 0.0] #init vel
       err = p - [1.091294525072764, 0.0, 0.0] #init vel, no damping
       #err = p - [1.082490077681814, 0.0, 0.0]  #gravity
       #err = p - [1.059191793818665, 0.0, 0.0]  #gravity, no damping
       exu.Print('p  =',list(p))
       exu.Print('h=', round(h,8), ', err=', err[0])
       
       sumSol += p[0]
   
   
       #Euler:
       # p  = [1.0368839392961136, 0.0, 0.0]
       # p  = [1.035017478371429, 0.0, 0.0]
       # p  = [1.0341182324516134, 0.0, 0.0]
       # p  = [1.033676874325782, 0.0, 0.0]
       # h= 0.001 , err= 0.003642999475132358
       # h= 0.0005 , err= 0.0017765385504477926
       # h= 0.00025 , err= 0.0008772926306321871
       # h= 0.000125 , err= 0.00043593450480083895
   
       #VelocityVerlet:
       # p  = [1.0333130215415933, 0.0, 0.0]
       # p  = [1.0332766497073687, 0.0, 0.0]
       # p  = [1.033258711414605, 0.0, 0.0]
       # p  = [1.0332498047052472, 0.0, 0.0]
       # h= 0.001 , err= 7.208172061212714e-05
       # h= 0.0005 , err= 3.570988638745831e-05
       # h= 0.00025 , err= 1.777159362381653e-05
       # h= 0.000125 , err= 8.86488426599108e-06
   
       #ExplicitMidpoint:
       # h= 0.001 , err= 3.6605276949597254e-06
       # h= 0.0005 , err= 9.043581823409141e-07
       # h= 0.00025 , err= 2.247182926407021e-07
       # h= 0.000125 , err= 5.6006646875772503e-08
   
       #RK67:    p= [1.0332409398209816, 0.0, 0.0]
       #RK67/2:  p= [1.0332409398209812, 0.0, 0.0]
   if useGraphics: #only start graphics once, but after background is set
       SC.renderer.Stop()
       mbs.PlotSensor(sMass,components=[0])
   
   exu.Print("velocityVerletTest result=",sumSol)
   
   exudynTestGlobals.testError = sumSol - (4.365184132226787) #2024-10-06
   exudynTestGlobals.testResult = sumSol
   exu.Print("velocityVerletTest error=", exudynTestGlobals.testError)
   


