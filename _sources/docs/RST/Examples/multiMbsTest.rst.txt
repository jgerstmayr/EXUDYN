
.. _examples-multimbstest:

***************
multiMbsTest.py
***************

You can view and download this file on Github: `multiMbsTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/multiMbsTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  test with several mbs
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-03-22
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   import numpy as np
   
   
   def CreateSystem(mbs, pOff, color0):
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
       #physical parameters
       p0=np.array(pOff)
       g =     [0,-9.81,0] #gravity
       bodyDim=[1,0.1,0.1] #body dimensions
       pMid0 = np.array([bodyDim[0]*0.5,0,0]) + p0 #center of mass, body0
       
       #first link:
       #inertia with helper function
       iCube0 = InertiaCuboid(density=5000, sideLengths=[1,0.1,0.1])
       #print(iCube)
       
       #graphics for body
       graphicsBody0 = graphics.RigidLink(p0=[-0.5*bodyDim[0],0,0],p1=[0.5*bodyDim[0],0,0], 
                                            axis0=[0,0,1], axis1=[0,0,0*1], radius=[0.05,0.05], 
                                            thickness = 0.1, width = [0.12,0.12], color=color0)
       
       b0 = mbs.CreateRigidBody(referencePosition = pMid0,
                                inertia = iCube0,
                                gravity = g,
                                graphicsDataList = [graphicsBody0])
   
       #ground body and marker
       oGround = mbs.CreateGround()
       mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b0], 
                               position=p0, axis=[0,0,1],
                               axisRadius=0.01, axisLength=0.1)
       mbs.Assemble()
   
   def Simulate(SC, mbs):
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #assemble system and solve
       
       simulationSettings = exu.SimulationSettings() #takes currently set values or default values
       
       tEnd = 2 #simulation time
       h = 1e-3 #step size
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.timeIntegration.verboseMode = 1
       simulationSettings.timeIntegration.simulateInRealtime = True
       
       SC.visualizationSettings.general.autoFitScene = False
       SC.visualizationSettings.window.renderWindowSize=[1600,1200]
       SC.visualizationSettings.openGL.multiSampling = 4
       
       # SC.renderer.Start()
       # if 'renderState' in exu.sys: #reload old view
       #     SC.renderer.SetState(exu.sys['renderState'])
       
       SC.renderer.DoIdleTasks() #stop before simulating
       
       mbs.SolveDynamic(simulationSettings = simulationSettings,
                        solverType=exu.DynamicSolverType.TrapezoidalIndex2)
       
       # SC.renderer.DoIdleTasks() #stop before closing
       # SC.renderer.Stop() #safely close rendering window!
       
   
   
   SC2 = exu.SystemContainer()
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   mbs2 = SC2.AddSystem()
   
   CreateSystem(mbs, [0,0,0], graphics.color.red)
   CreateSystem(mbs, [1.2,0,0], graphics.color.blue)
   CreateSystem(mbs2, [0,0,0], graphics.color.red)
   CreateSystem(mbs2, [0.6,-1.2,0], graphics.color.green)
   
   SC.renderer.Attach()
   SC.renderer.Start()
   if 'renderState' in exu.sys: #reload old view
       SC.renderer.SetState(exu.sys['renderState'])
       
   Simulate(SC, mbs)
   # SC.renderer.DoIdleTasks()
   #SC.renderer.Detach()
   SC.renderer.DoIdleTasks() #stop before closing
   SC.renderer.Stop() #safely close rendering window!
   
   SC2.renderer.Attach()
   SC.renderer.Start()
   if 'renderState' in exu.sys: #reload old view
       SC2.renderer.SetState(exu.sys['renderState'])
   Simulate(SC2, mbs2)
   
   SC2.renderer.DoIdleTasks() #stop before closing
   SC.renderer.Stop() #safely close rendering window!
   
   if False:
       
       mbs.PlotSensor([sens1],[1])
   
   


