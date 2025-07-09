
.. _testmodels-createrollingdisctest:

************************
createRollingDiscTest.py
************************

You can view and download this file on Github: `createRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/createRollingDiscTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test model for CreateRollingDisc; simple model of a wheel
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-03-05
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
   
   r = 0.2
   oDisc = mbs.CreateRigidBody(inertia = InertiaCylinder(density=5000, length=0.02, outerRadius=r, axis=0),
                             referencePosition = [1,0,r],
                             initialAngularVelocity = [-3*2*pi,10,0],
                             initialVelocity = [0,r*3*2*pi,0],
                             gravity = [0,0,-9.81],
                             graphicsDataList = [exu.graphics.Cylinder(pAxis = [-0.01,0,0], vAxis = [0.02,0,0], radius = r*0.99, nTiles=64,
                                                                       color=exu.graphics.color.blue),
                                                 exu.graphics.Basis(length=2*r)])
   oGround = mbs.CreateGround(graphicsDataList=[exu.graphics.CheckerBoard(size=8)])
   
   mbs.CreateRollingDisc(bodyNumbers=[oGround, oDisc], 
                         axisPosition=[0,0,0], axisVector=[1,0,0], #on local wheel frame
                         planePosition = [0,0,0], planeNormal = [0,0,1],  #in ground frame
                         discRadius = r, 
                         discWidth=0.01, color=exu.graphics.color.steelblue)
   
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   
   stepSize=0.002
   tEnd = 1
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.writeSolutionToFile = useGraphics
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   SC.visualizationSettings.nodes.show = True
   SC.visualizationSettings.nodes.drawNodesAsPoint  = False
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.015
   SC.visualizationSettings.openGL.perspective = 2
   SC.visualizationSettings.openGL.shadow=0.3
   SC.visualizationSettings.openGL.multiSampling=4
   
   if useGraphics:
       SC.renderer.Start()
       SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   p0=mbs.GetObjectOutputBody(oDisc, exu.OutputVariableType.Position)
   
   u = np.linalg.norm(p0)
   exu.Print('solution of createRollingDisc=',u) 
   
   exudynTestGlobals.testError = u - (0) 
   exudynTestGlobals.testResult = u
   
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   # mbs.SolutionViewer()


