
.. _testmodels-createkinematictreetest:

**************************
createKinematicTreeTest.py
**************************

You can view and download this file on Github: `createKinematicTreeTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/createKinematicTreeTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Model of a four bar mechanism under gravity
   #           Used as test case for CreateKinematicTree
   #
   # Author:   Johannes Gerstmayr
   # Date:     2025-06-5
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import *
   import exudyn.graphics as graphics
   import numpy as np
   
   #import exudyn.rigidBodyUtilities as erb
   #from exudyn.rigidBodyUtilities import HT0, HT2rotationMatrix, HT2translation
   
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
   
   
   
   
   # create empty system
   SC = exu.SystemContainer() # sc = systems container
   mbs = SC.AddSystem() # mbs = multibodysystem
   
   
   
   L1 = 0.8
   L2 = 0.5
   L3 = 0.3
   w = 0.05
   
   ground = mbs.CreateGround(referencePosition=[L2,L1-L3,0],
                             graphicsDataList=[graphics.CheckerBoard(size=4)])
   
   #link with reference point at [0,0,0], COM at [0.5*L1,0,0]
   link0Inertia = InertiaCuboid(density=1000, sideLengths=[L1,w,w]) #w.r.t. center of mass = [0,0,0]
   link0Inertia = link0Inertia.Translated([0.5*L1,0,0]) #COM at [0.5*L1,0,0]
   
   
   pdControl = (2e4,4e2)
   #create tree links:
   link0 = TreeLink(linkInertia=link0Inertia,
                    jointType=exu.JointType.RevoluteZ,
                    jointHT=HTtranslate([0,0,0.5]),
                    PDcontrol=pdControl,
                    )
   link1 = TreeLink(linkInertia=link0Inertia,
                    jointType=exu.JointType.RevoluteY,
                    jointHT=HTtranslate([0.5,0,0.]),
                    PDcontrol=pdControl
                    )
   #create kinematic tree:
   oKT0 = mbs.CreateKinematicTree(
                                 initialCoordinates=[pi,-0.5*pi],
                                 listOfTreeLinks=[link0, link1],
                                 colors=[graphics.color.blue,graphics.color.red],
                                 gravity=[0,0,-9.81],
                                 baseOffset=[1,0.5,0.],
                                 linkRoundness=1,
                                 )
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #create tree links
   link0 = TreeLink(linkInertia=link0Inertia,
                    jointType=exu.JointType.PrismaticX,
                    parent=-1,
                    jointHT=HTtranslate([0,0,0.1]),
                    PDcontrol=(10*pdControl[0],10*pdControl[1]),
                    )
   
   link1 = TreeLink(linkInertia=link0Inertia,
                    jointType=exu.JointType.RevoluteZ,
                    parent=0,
                    jointHT=HTtranslate([-0.25,0,0.4]),
                    PDcontrol=pdControl,
                    )
   link2 = TreeLink(linkInertia=link0Inertia,
                    jointType=exu.JointType.RevoluteZ,
                    parent=0,
                    jointHT=HTtranslate([0.25,0,0.4]),
                    PDcontrol=pdControl,
                    )
   link3 = TreeLink(linkInertia=link0Inertia,
                    jointType=exu.JointType.RevoluteX,
                    parent=1,
                    jointHT=HTtranslate([0,0.5,0.]),
                    PDcontrol=pdControl
                    )
   link4 = TreeLink(linkInertia=link0Inertia,
                    jointType=exu.JointType.RevoluteX,
                    parent=3,
                    jointHT=HTtranslate([0,0.5,0.]),
                    PDcontrol=pdControl
                    )
   
   link5 = TreeLink(linkInertia=link0Inertia,
                    jointType=exu.JointType.RevoluteX,
                    parent=2,
                    jointHT=HTtranslate([0,0.5,0.]),
                    PDcontrol=pdControl
                    )
                    
   link6 = TreeLink(linkInertia=link0Inertia,
                    jointType=exu.JointType.RevoluteX,
                    parent=5,
                    jointHT=HTtranslate([0,0.5,0.]),
                    PDcontrol=pdControl
                    )
   
   #create kinematic tree:
   oKT1 = mbs.CreateKinematicTree(
                                 initialCoordinates=[0.1, 0.125*pi,-0.125*pi]+[0]*4,
                                 listOfTreeLinks=[link0, link1, link2, link3, link4, link5, link6],
                                 colors=[graphics.color.orange]+[graphics.color.blue,graphics.color.red]*3,
                                 #gravity=[0,0,-9.81],
                                 baseOffset=[-1,-0.5,0.],
                                 linkRoundness=0.5,
                                 baseGraphicsDataList=[graphics.Cylinder(pAxis=[-0.5,0,0.05],vAxis=[1,0,0],radius=0.05,
                                                                      color=graphics.color.dodgerblue)]
                                 )
   
   def PreStepUserFunction(mbs, t):
       u = mbs.GetObjectParameter(oKT1,'jointPositionOffsetVector')
       u[0] = SmoothStep(t, 0.1, 0.2, 0, 0.25)
       u[1] = SmoothStep(t, 0.2, 0.3, 0, 0.25*pi)
       u[2] = SmoothStep(t, 0.2, 0.3, 0, -0.25*pi)
       u[3] = SmoothStep(t, 0.4, 0.5, 0, 0.5*pi)
       u[4] = SmoothStep(t, 0.6, 0.7, 0, 0.5*pi)
       u[5] = SmoothStep(t, 0.8, 0.9, 0, 0.5*pi)
       u[6] = SmoothStep(t, 1.0, 1.1, 0, 0.5*pi)
       mbs.SetObjectParameter(oKT1,'jointPositionOffsetVector',u)
       return True
   
   mbs.SetPreStepUserFunction(PreStepUserFunction)
   
   
   tEnd = 1.2
   stepSize = 4e-3
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.solutionSettings.writeSolutionToFile=True
   simulationSettings.solutionSettings.solutionWritePeriod=0.004
   simulationSettings.timeIntegration.numberOfSteps = tEnd/stepSize
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.timeIntegration.simulateInRealtime = True
   
   SC.visualizationSettings.general.drawWorldBasis = True
   SC.visualizationSettings.openGL.shadow = 0.25
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.window.renderWindowSize=[1600,1200]
   
   if useGraphics:
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys['renderState'])
       SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics:
       SC.renderer.Stop()
       
       mbs.SolutionViewer()
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ode2 = mbs.systemData.GetODE2Coordinates()
   
   u = np.linalg.norm(ode2)
   exu.Print('solution of createKinematicTreeTest=',u) 
   
   exudynTestGlobals.testResult = u
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   


