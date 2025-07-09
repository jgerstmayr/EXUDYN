
.. _examples-simple4linkpendulumbing:

**************************
simple4linkPendulumBing.py
**************************

You can view and download this file on Github: `simple4linkPendulumBing.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/simple4linkPendulumBing.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  4-link pendulum
   #
   # Author:   Bing AI with some help by Johannes Gerstmayr; note that doublePendulum2D.py did not exist prior to this model
   # Date:     2023-04-15
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   # create nodes:
   n0=mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0,0,0], initialVelocities=[0,0,0]))
   n1=mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[1,0,0], initialVelocities=[0,0,0]))
   n2=mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[2,0,0], initialVelocities=[0,0,0]))
   n3=mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[3,0,0], initialVelocities=[0,0,0]))
   
   # create bodies:
   b0=mbs.AddObject(RigidBody2D(physicsMass=1, physicsInertia=1,nodeNumber=n0,
          visualization=VRigidBody2D(graphicsData=[graphics.Lines([[-0.5,0,0],[0.5,0,0]])])))
   b1=mbs.AddObject(RigidBody2D(physicsMass=1, physicsInertia=1,nodeNumber=n1,
                    visualization=VRigidBody2D(graphicsData=[graphics.Lines([[-0.5,0,0],[0.5,0,0]])])))
   b2=mbs.AddObject(RigidBody2D(physicsMass=1, physicsInertia=1,nodeNumber=n2,
                    visualization=VRigidBody2D(graphicsData=[graphics.Lines([[-0.5,0,0],[0.5,0,0]])])))
   b3=mbs.AddObject(RigidBody2D(physicsMass=1, physicsInertia=1,nodeNumber=n3,
                    visualization=VRigidBody2D(graphicsData=[graphics.Lines([[-0.5,0,0],[0.5,0,0]])])))
   
   # add markers and loads:
   oGround = mbs.AddObject(ObjectGround())
   mGround=mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[-0.5, 0., 0.]))
   m00 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b0, localPosition=[-0.5, 0., 0.]))
   m01 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b0, localPosition=[ 0.5, 0., 0.]))
   m10 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b1, localPosition=[-0.5, 0., 0.]))
   m12 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b1, localPosition=[ 0.5, 0., 0.]))
   m21 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b2, localPosition=[-0.5, 0., 0.]))
   m23 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b2, localPosition=[ 0.5, 0., 0.]))
   m32 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=b3, localPosition=[-0.5, 0., 0.]))
   
   # add joints:
   jg0 = mbs.AddObject(RevoluteJoint2D(markerNumbers=[mGround,m00]))
   j01 = mbs.AddObject(RevoluteJoint2D(markerNumbers=[m01,m10]))
   j12 = mbs.AddObject(RevoluteJoint2D(markerNumbers=[m12,m21]))
   j23 = mbs.AddObject(RevoluteJoint2D(markerNumbers=[m23,m32]))
    
   # add loads:
   
   mLoad0=mbs.AddMarker(MarkerNodePosition(nodeNumber=n0))
   mLoad1=mbs.AddMarker(MarkerNodePosition(nodeNumber=n1))
   mLoad2=mbs.AddMarker(MarkerNodePosition(nodeNumber=n2))
   mLoad3=mbs.AddMarker(MarkerNodePosition(nodeNumber=n3))
   
   mbs.AddLoad(Force(markerNumber=mLoad0, loadVector=[0,-9.81*1,0]))
   mbs.AddLoad(Force(markerNumber=mLoad1, loadVector=[0,-9.81*1,0]))
   mbs.AddLoad(Force(markerNumber=mLoad2, loadVector=[0,-9.81*1,0]))
   mbs.AddLoad(Force(markerNumber=mLoad3, loadVector=[0,-9.81*1,0]))
   
   # add time integration scheme:
   mbs.Assemble()
   simulationSettings = exu.SimulationSettings()
   simulationSettings.timeIntegration.numberOfSteps = 10000
   simulationSettings.timeIntegration.endTime = 10
   simulationSettings.solutionSettings.writeSolutionToFile = True
   simulationSettings.solutionSettings.solutionWritePeriod = 0.01
   
   SC.renderer.Start()
   SC.renderer.DoIdleTasks()
   mbs.SolveDynamic(simulationSettings)
   SC.renderer.Stop()

