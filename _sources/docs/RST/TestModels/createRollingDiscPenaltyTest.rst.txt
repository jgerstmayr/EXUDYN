
.. _testmodels-createrollingdiscpenaltytest:

*******************************
createRollingDiscPenaltyTest.py
*******************************

You can view and download this file on Github: `createRollingDiscPenaltyTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/createRollingDiscPenaltyTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test model for CreateRollingDiscPenalty; simple model of a two-wheeler without steering
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-02-27
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
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   g = [0,0,-9.81]     #gravity in m/s^2
   m = 0.5             #disc mass in kg
   r = 0.1             #radius of disc in m
   w = 0.01            #width of disc in m, just for drawing
   v0 = 2
   
   L = 0.8             #length of board
   
   omega0 = [0,4*2*np.pi,0]      #initial angular velocity around y-axis
   v0 = Skew(omega0) @ [0,0,r]   #initial angular velocity of center point
   
   #inertia for infinitely small ring:
   inertiaRing = RigidBodyInertia(mass=1, inertiaTensor= np.diag([0.25*m*r**2, 0.5*m*r**2, 0.25*m*r**2]))
   
   #additional graphics for visualization of rotation:
   graphicsWheel = graphics.Brick(centerPoint=[0,0,0],size=[1.2*r,2*w,1.2*r], color=graphics.color.lightred)
   
   oWheel0 = mbs.CreateRigidBody(referencePosition=[-0.5*L,0,r],
                                initialVelocity=v0,  
                                initialAngularVelocity=omega0,  
                                inertia=inertiaRing,  
                                gravity=g,  
                                graphicsDataList=[graphicsWheel]
                                )  
   
   oWheel1 = mbs.CreateRigidBody(referencePosition=[0.5*L,0,r],
                                initialVelocity=v0,  
                                initialAngularVelocity=omega0,  
                                inertia=inertiaRing,  
                                gravity=g,  
                                graphicsDataList=[graphicsWheel]
                                )  
   
   graphicsBody = graphics.Brick(centerPoint=[0,0,0],size=[L*1.1,0.6*r,0.6*r], color=graphics.color.grey)
   oBody = mbs.CreateRigidBody(referencePosition=[0,0,r*1.2],
                                initialVelocity=v0,  
                                inertia=InertiaCuboid(2800, sideLengths=[L,0.5*r,0.5*r]),
                                gravity=g,  
                                graphicsDataList=[graphicsBody]
                                )  
   
   #ground body and marker
   gGround = graphics.CheckerBoard(size=50, nTiles=50)
   oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[gGround])))
   
   mbs.CreateRevoluteJoint(bodyNumbers=[oWheel0, oBody],
                           position = [0,0,0], axis=[0,1,0], useGlobalFrame=False,
                           axisRadius=0.1*r, axisLength=0.6*r)
   
   mbs.CreateRevoluteJoint(bodyNumbers=[oWheel1, oBody],
                           position = [0,0,0], axis=[0,1,0], useGlobalFrame=False,
                           axisRadius=0.1*r, axisLength=0.6*r)
   
   oRolling0 = mbs.CreateRollingDiscPenalty(bodyNumbers=[oGround, oWheel0],
                                axisPosition=[0,0,0], axisVector=[0,1,0],
                                discRadius=r, 
                                contactStiffness=1e5, contactDamping=1e3, dryFriction=[0.4,0.4],
                                discWidth=0.1*r, color=graphics.color.blue)
   
   oRolling1 = mbs.CreateRollingDiscPenalty(bodyNumbers=[oGround, oWheel1],
                                axisPosition=[0,0,0], axisVector=[0,1,0],
                                discRadius=r, 
                                contactStiffness=1e5, contactDamping=1e3, dryFriction=[0.4,0.4],
                                discWidth=0.1*r, color=graphics.color.blue)
   
   
   
   #sensor for trace of contact point:
   if useGraphics:
       sTrail=mbs.AddSensor(SensorObject(objectNumber=oRolling0, storeInternal=True,#fileName='solution/rollingDiscTrail.txt', 
                                  outputVariableType = exu.OutputVariableType.Position))
       
       
   
   
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   
   stepSize=0.002
   tEnd = 1
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   #simulationSettings.solutionSettings.solutionWritePeriod = 0.01
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.005
   simulationSettings.solutionSettings.writeSolutionToFile = useGraphics
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   #simulationSettings.displayComputationTime = True
   #simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse #does not work for initial accelerations
   
   SC.visualizationSettings.nodes.show = True
   SC.visualizationSettings.nodes.drawNodesAsPoint  = False
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.015
   SC.visualizationSettings.openGL.perspective = 2
   
   if useGraphics:
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   p0=mbs.GetObjectOutput(oRolling0, exu.OutputVariableType.Position)
   
   u = np.linalg.norm(p0)
   exu.Print('solution of createRollingDiscPenalty=',u) 
   
   exudynTestGlobals.testError = u - (0) 
   exudynTestGlobals.testResult = u
   
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
       ##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
       #plot results
       if True:
           mbs.PlotSensor(sTrail, componentsX=[0],components=[1], closeAll=True, title='wheel trail')


