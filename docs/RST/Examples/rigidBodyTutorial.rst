
.. _examples-rigidbodytutorial:

********************
rigidBodyTutorial.py
********************

You can view and download this file on Github: `rigidBodyTutorial.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/rigidBodyTutorial.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  3D rigid body example with joints
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-03-14
   # Modified: 2023-04-18
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
   import numpy as np
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   g = [0,-9.81,0] #gravity
   bodyDim = [1,0.1,0.1] #body dimensions
   p0 = [0,0,0] #origin of pendulum
   pMid0 = [bodyDim[0]*0.5,0,0] #center of mass, body0
   
   #inertia for cubic body with dimensions in sideLengths
   iCube = InertiaCuboid(density=5000, sideLengths=[1,0.1,0.1])
   print(iCube)
   
   #graphics for body
   graphicsBody = GraphicsDataRigidLink(p0=[-0.5*bodyDim[0],0,0],p1=[0.5*bodyDim[0],0,0], 
                                        axis1=[0,0,1], radius=[0.01,0.01], 
                                        thickness = 0.01, width = [0.02,0.02], color=color4lightred)
   
   [n0,b0]=AddRigidBody(mainSys = mbs,
                        inertia = iCube,
                        nodeType = str(exu.NodeType.RotationEulerParameters),
                        position = pMid0,
                        rotationMatrix = np.diag([1,1,1]),
                        angularVelocity = [0,0,0],
                        gravity = g, #will automatically add a load on body
                        graphicsDataList = [graphicsBody])
   
   #ground body and marker
   oGround = mbs.AddObject(ObjectGround())
   markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
   
   #markers are needed to link joints and bodies; also needed for loads
   #markers for rigid body:
   markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[-0.5*bodyDim[0],0,0]))
   
   #revolute joint (free z-axis)
   mbs.AddObject(GenericJoint(markerNumbers=[markerGround, markerBody0J0], 
                              constrainedAxes=[1,1,1,1,1,0],
                              visualization=VObjectJointGeneric(axesRadius=0.01, axesLength=0.1)))
   
   mbs.Assemble()
   #mbs.systemData.Info()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 4
   stepSize = 1e-4 #could be larger, but then we do not see the simulation ...
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.verboseMode = 1 #to see some output
   
   #start graphics
   exu.StartRenderer()
   mbs.WaitForUserToContinue() #wait until user presses space
   
   #start generalized alpha solver
   mbs.SolveDynamic(simulationSettings = simulationSettings)
   
   SC.WaitForRenderEngineStopFlag()
   exu.StopRenderer() #safely close rendering window!
   
   
   


