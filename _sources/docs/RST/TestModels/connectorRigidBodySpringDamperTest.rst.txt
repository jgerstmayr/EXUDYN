
.. _testmodels-connectorrigidbodyspringdampertest:

*************************************
connectorRigidBodySpringDamperTest.py
*************************************

You can view and download this file on Github: `connectorRigidBodySpringDamperTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/connectorRigidBodySpringDamperTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test for RigidBodySpringDamper with springForceTorqueUserFunction;
   #           the RigidBodySpringDamper can be used to model complicance effects in joints where 
   #           one axis undergoes large rotations, and the other rotations are small
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-01-07
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import sys
   sys.path.append('../TestModels')            #for modelUnitTest as this example may be used also as a unit test
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.graphicsDataUtilities import *
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
   
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))
   
   #example with rigid body at [0,0,0], 1kg under initial velocity
   graphicsBody = graphics.Brick(centerPoint=[0,0,0],size=[0.09,0.09,0.2], color=graphics.color.lightred)
   nBody = mbs.AddNode(RigidRxyz(initialVelocities=[0,10,0, 2*pi*4,0,0]))
   oBody = mbs.AddObject(RigidBody(physicsMass=1, physicsInertia=[1,1,1,0,0,0], nodeNumber=nBody, 
                                   visualization=VRigidBody(graphicsData=[graphicsBody])))
   
   mBody = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nBody))
   mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                           localPosition = [0,0,0]))
   
   def UFforce(mbs, t, itemIndex, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1, offset): 
       k = stiffness
       u = displacement
       v = velocity
       w = angularVelocity
       rot = rotation
       return [u[0]*k[0][0],u[1]*k[1][1]+v[1]*0.01*k[1][1],u[2]*k[2][2],
               rot[0]*k[0][0]+w[0]*0.001*k[0][0],rot[1]*k[0][0],rot[2]*k[0][0]]
   
   #markerNumbers and parameters taken from mini example
   k=5000
   mbs.AddObject(RigidBodySpringDamper(markerNumbers = [mGround, mBody], 
                                       stiffness = np.diag([k,k,k, 0,0,0]), 
                                       damping = np.diag([0,k*0.01,0, 0,0,0]), 
                                       offset = [0,1,0, 0,0,0],
                                       springForceTorqueUserFunction = UFforce))
   
   mbs.Assemble()
   
   tEnd = 0.1
   h=1e-3
   if useGraphics:
       tEnd = 1 #parameters sucht that we can see some motion
       h=1e-5
   
   
   simulationSettings = exu.SimulationSettings()
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.writeSolutionToFile = False
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #no numerical damping
   
   simulationSettings.displayStatistics = True
   simulationSettings.timeIntegration.verboseMode = 1
   
   if useGraphics:
       SC.renderer.Start()              #start graphics visualization
       SC.renderer.DoIdleTasks()    #wait for pressing SPACE bar to continue
   
   #start solver:
   mbs.SolveDynamic(simulationSettings)
   
   if useGraphics:
       SC.renderer.DoIdleTasks()#wait for pressing 'Q' to quit
       SC.renderer.Stop()               #safely close rendering window!
   
   p0=mbs.GetObjectOutputBody(oBody, localPosition=[0.1,0.1,0.1], 
                              variableType = exu.OutputVariableType.Position)
   result = p0[0]+p0[1]
   exu.Print('solution of connectorRigidBodySpringDamperTest=',result) #use x-coordinate
   
   exudynTestGlobals.testError = result - (0.18276224743714353) #2021-01-07: 
   exudynTestGlobals.testResult = result
   
   


