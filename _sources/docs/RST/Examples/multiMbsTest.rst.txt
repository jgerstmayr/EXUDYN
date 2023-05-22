
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
   from exudyn.utilities import *
   
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
       graphicsBody0 = GraphicsDataRigidLink(p0=[-0.5*bodyDim[0],0,0],p1=[0.5*bodyDim[0],0,0], 
                                            axis0=[0,0,1], axis1=[0,0,0*1], radius=[0.05,0.05], 
                                            thickness = 0.1, width = [0.12,0.12], color=color0)
       
       [n0,b0]=AddRigidBody(mainSys = mbs,
                            inertia = iCube0,
                            nodeType = str(exu.NodeType.RotationEulerParameters),
                            position = pMid0,
                            rotationMatrix = np.diag([1,1,1]),
                            angularVelocity = [0,0,0],
                            gravity = g,
                            graphicsDataList = [graphicsBody0])
       
       #ground body and marker
       oGround = mbs.AddObject(ObjectGround())
       markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=p0))
       
       #markers for rigid body:
       markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[-0.5*bodyDim[0],0,0]))
       
       #revolute joint (free z-axis)
       mbs.AddObject(GenericJoint(markerNumbers=[markerGround, markerBody0J0], 
                                  constrainedAxes=[1,1,1,1,1,0],
                                  visualization=VObjectJointGeneric(axesRadius=0.01, axesLength=0.1)))
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
       
       # exu.StartRenderer()
       # if 'renderState' in exu.sys: #reload old view
       #     SC.SetRenderState(exu.sys['renderState'])
       
       mbs.WaitForUserToContinue() #stop before simulating
       
       mbs.SolveDynamic(simulationSettings = simulationSettings,
                        solverType=exu.DynamicSolverType.TrapezoidalIndex2)
       
       # SC.WaitForRenderEngineStopFlag() #stop before closing
       # exu.StopRenderer() #safely close rendering window!
       
   
   
   SC2 = exu.SystemContainer()
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   mbs2 = SC2.AddSystem()
   
   CreateSystem(mbs, [0,0,0], color4red)
   CreateSystem(mbs, [1.2,0,0], color4blue)
   CreateSystem(mbs2, [0,0,0], color4red)
   CreateSystem(mbs2, [0.6,-1.2,0], color4green)
   
   SC.AttachToRenderEngine()
   exu.StartRenderer()
   if 'renderState' in exu.sys: #reload old view
       SC.SetRenderState(exu.sys['renderState'])
       
   Simulate(SC, mbs)
   # mbs.WaitForUserToContinue()
   #SC.DetachFromRenderEngine()
   SC.WaitForRenderEngineStopFlag() #stop before closing
   exu.StopRenderer() #safely close rendering window!
   
   SC2.AttachToRenderEngine()
   exu.StartRenderer()
   if 'renderState' in exu.sys: #reload old view
       SC2.SetRenderState(exu.sys['renderState'])
   Simulate(SC2, mbs2)
   
   SC2.WaitForRenderEngineStopFlag() #stop before closing
   exu.StopRenderer() #safely close rendering window!
   
   if False:
       
       mbs.PlotSensor([sens1],[1])
   
   


