

.. _sec-mainsystemextensions-createmasspoint:

Function: CreateMassPoint
^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateMassPoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L118>`__\ (\ ``name = ''``\ , \ ``referencePosition = [0.,0.,0.]``\ , \ ``initialDisplacement = [0.,0.,0.]``\ , \ ``initialVelocity = [0.,0.,0.]``\ , \ ``physicsMass = 0``\ , \ ``gravity = [0.,0.,0.]``\ , \ ``graphicsDataList = []``\ , \ ``drawSize = -1``\ , \ ``color = [-1.,-1.,-1.,-1.]``\ , \ ``show = True``\ , \ ``create2D = False``\ , \ ``returnDict = False``\ )

- | \ *function description*\ :
  | helper function to create 2D or 3D mass point object and node, using arguments as in NodePoint and MassPoint
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateMassPoint.
- | \ *input*\ :
  | \ ``name``\ : name string for object, node is 'Node:'+name
  | \ ``referencePosition``\ : reference coordinates for point node (always a 3D vector, no matter if 2D or 3D mass)
  | \ ``initialDisplacement``\ : initial displacements for point node (always a 3D vector, no matter if 2D or 3D mass)
  | \ ``initialVelocity``\ : initial velocities for point node (always a 3D vector, no matter if 2D or 3D mass)
  | \ ``physicsMass``\ : mass of mass point
  | \ ``gravity``\ : gravity vevtor applied (always a 3D vector, no matter if 2D or 3D mass)
  | \ ``graphicsDataList``\ : list of GraphicsData for optional mass visualization
  | \ ``drawSize``\ : general drawing size of node
  | \ ``color``\ : color of node
  | \ ``show``\ : True: if graphicsData list is empty, node is shown, otherwise body is shown; otherwise, nothing is shown
  | \ ``create2D``\ : if False, create NodePoint2D and MassPoint2D
  | \ ``returnDict``\ : if False, returns object index; if True, returns dict of all information on created object and node
- | \ *output*\ :
  | Union[dict, ObjectIndex]; returns mass point object index or dict with all data on request (if returnDict=True)
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0=mbs.CreateMassPoint(referencePosition = [0,0,0],
                         initialVelocity = [2,5,0],
                         physicsMass = 1, gravity = [0,-9.81,0],
                         drawSize = 0.5, color=color4blue)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `springDamperTutorialNew.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/springDamperTutorialNew.py>`_\  (Ex), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-createrigidbody:

Function: CreateRigidBody
^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateRigidBody <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L247>`__\ (\ ``name = ''``\ , \ ``referencePosition = [0.,0.,0.]``\ , \ ``referenceRotationMatrix = np.eye(3)``\ , \ ``initialVelocity = [0.,0.,0.]``\ , \ ``initialAngularVelocity = [0.,0.,0.]``\ , \ ``initialDisplacement = None``\ , \ ``initialRotationMatrix = None``\ , \ ``inertia = None``\ , \ ``gravity = [0.,0.,0.]``\ , \ ``nodeType = exudyn.NodeType.RotationEulerParameters``\ , \ ``graphicsDataList = []``\ , \ ``drawSize = -1``\ , \ ``color = [-1.,-1.,-1.,-1.]``\ , \ ``show = True``\ , \ ``create2D = False``\ , \ ``returnDict = False``\ )

- | \ *function description*\ :
  | helper function to create 3D (or 2D) rigid body object and node; all quantities are global (angular velocity, etc.)
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateRigidBody.
- | \ *input*\ :
  | \ ``name``\ : name string for object, node is 'Node:'+name
  | \ ``referencePosition``\ : reference position vector for rigid body node (always a 3D vector, no matter if 2D or 3D body)
  | \ ``referenceRotationMatrix``\ : reference rotation matrix for rigid body node (always 3D matrix, no matter if 2D or 3D body)
  | \ ``initialVelocity``\ : initial translational velocity vector for node (always a 3D vector, no matter if 2D or 3D body)
  | \ ``initialAngularVelocity``\ : initial angular velocity vector for node (always a 3D vector, no matter if 2D or 3D body)
  | \ ``initialDisplacement``\ : initial translational displacement vector for node (always a 3D vector, no matter if 2D or 3D body); these displacements are deviations from reference position, e.g. for a finite element node [None: unused]
  | \ ``initialRotationMatrix``\ : initial rotation provided as matrix (always a 3D matrix, no matter if 2D or 3D body); this rotation is superimposed to reference rotation [None: unused]
  | \ ``inertia``\ : an instance of class RigidBodyInertia, see rigidBodyUtilities; may also be from derived class (InertiaCuboid, InertiaMassPoint, InertiaCylinder, ...)
  | \ ``gravity``\ : gravity vevtor applied (always a 3D vector, no matter if 2D or 3D mass)
  | \ ``graphicsDataList``\ : list of GraphicsData for rigid body visualization; use graphicsDataUtilities function GraphicsData...(...)
  | \ ``drawSize``\ : general drawing size of node
  | \ ``color``\ : color of node
  | \ ``show``\ : True: if graphicsData list is empty, node is shown, otherwise body is shown; False: nothing is shown
  | \ ``create2D``\ : if False, create NodePoint2D and MassPoint2D
  | \ ``returnDict``\ : if False, returns object index; if True, returns dict of all information on created object and node
- | \ *output*\ :
  | Union[dict, ObjectIndex]; returns rigid body object index (or dict with 'nodeNumber', 'objectNumber' and possibly 'loadNumber' and 'markerBodyMass' if returnDict=True)
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [1,0,0],
                           initialVelocity = [2,5,0],
                           initialAngularVelocity = [5,0.5,0.7],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1],
                                                                        color=color4red)])
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `graphicsDataExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/graphicsDataExample.py>`_\  (Ex), \ `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_\  (Ex), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `computeODE2AEeigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2AEeigenvaluesTest.py>`_\  (TM), \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createspringdamper:

Function: CreateSpringDamper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateSpringDamper <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L469>`__\ (\ ``name = ''``\ , \ ``bodyOrNodeList = [None, None]``\ , \ ``localPosition0 = [0.,0.,0.]``\ , \ ``localPosition1 = [0.,0.,0.]``\ , \ ``referenceLength = None``\ , \ ``stiffness = 0.``\ , \ ``damping = 0.``\ , \ ``force = 0.``\ , \ ``velocityOffset = 0.``\ , \ ``show = True``\ , \ ``drawSize = -1``\ , \ ``color = color4default``\ )

- | \ *function description*\ :
  | helper function to create SpringDamper connector, using arguments from ObjectConnectorSpringDamper; similar interface as CreateDistanceConstraint(...)
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateSpringDamper.
- | \ *input*\ :
  | \ ``name``\ : name string for connector; markers get Marker0:name and Marker1:name
  | \ ``bodyOrNodeList``\ : a list of object numbers (with specific localPosition0/1) or node numbers; may also be of mixed types
  | \ ``localPosition0``\ : local position (as 3D list or numpy array) on body0, if not a node number
  | \ ``localPosition1``\ : local position (as 3D list or numpy array) on body1, if not a node number
  | \ ``referenceLength``\ : if None, length is computed from reference position of bodies or nodes; if not None, this scalar reference length is used for spring
  | \ ``stiffness``\ : scalar stiffness coefficient
  | \ ``damping``\ : scalar damping coefficient
  | \ ``force``\ : scalar additional force applied
  | \ ``velocityOffset``\ : scalar offset: if referenceLength is changed over time, the velocityOffset may be changed accordingly to emulate a reference motion
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``drawSize``\ : general drawing size of connector
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | ObjectIndex; returns index of newly created object
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateMassPoint(referencePosition = [2,0,0],
                           initialVelocity = [2,5,0],
                           physicsMass = 1, gravity = [0,-9.81,0],
                           drawSize = 0.5, color=color4blue)
  oGround = mbs.AddObject(ObjectGround())
  #add vertical spring
  oSD = mbs.CreateSpringDamper(bodyOrNodeList=[oGround, b0],
                               localPosition0=[2,1,0],
                               localPosition1=[0,0,0],
                               stiffness=1e4, damping=1e2,
                               drawSize=0.2)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  SC.visualizationSettings.nodes.drawNodesAsPoint=False
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `springDamperTutorialNew.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/springDamperTutorialNew.py>`_\  (Ex), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-createcartesianspringdamper:

Function: CreateCartesianSpringDamper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateCartesianSpringDamper <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L605>`__\ (\ ``name = ''``\ , \ ``bodyOrNodeList = [None, None]``\ , \ ``localPosition0 = [0.,0.,0.]``\ , \ ``localPosition1 = [0.,0.,0.]``\ , \ ``stiffness = [0.,0.,0.]``\ , \ ``damping = [0.,0.,0.]``\ , \ ``offset = [0.,0.,0.]``\ , \ ``show = True``\ , \ ``drawSize = -1``\ , \ ``color = color4default``\ )

- | \ *function description*\ :
  | helper function to create CartesianSpringDamper connector, using arguments from ObjectConnectorCartesianSpringDamper
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateCartesianSpringDamper.
- | \ *input*\ :
  | \ ``name``\ : name string for connector; markers get Marker0:name and Marker1:name
  | \ ``bodyOrNodeList``\ : a list of object numbers (with specific localPosition0/1) or node numbers; may also be of mixed types
  | \ ``localPosition0``\ : local position (as 3D list or numpy array) on body0, if not a node number
  | \ ``localPosition1``\ : local position (as 3D list or numpy array) on body1, if not a node number
  | \ ``stiffness``\ : stiffness coefficients (as 3D list or numpy array)
  | \ ``damping``\ : damping coefficients (as 3D list or numpy array)
  | \ ``offset``\ : offset vector (as 3D list or numpy array)
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``drawSize``\ : general drawing size of connector
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | ObjectIndex; returns index of newly created object
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateMassPoint(referencePosition = [7,0,0],
                            physicsMass = 1, gravity = [0,-9.81,0],
                            drawSize = 0.5, color=color4blue)
  oGround = mbs.AddObject(ObjectGround())
  oSD = mbs.CreateCartesianSpringDamper(bodyOrNodeList=[oGround, b0],
                                localPosition0=[7.5,1,0],
                                localPosition1=[0,0,0],
                                stiffness=[200,2000,0], damping=[2,20,0],
                                drawSize=0.2)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  SC.visualizationSettings.nodes.drawNodesAsPoint=False
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `computeODE2AEeigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2AEeigenvaluesTest.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-createrigidbodyspringdamper:

Function: CreateRigidBodySpringDamper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateRigidBodySpringDamper <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L693>`__\ (\ ``name = ''``\ , \ ``bodyOrNodeList = [None, None]``\ , \ ``localPosition0 = [0.,0.,0.]``\ , \ ``localPosition1 = [0.,0.,0.]``\ , \ ``stiffness = np.zeros((6,6))``\ , \ ``damping = np.zeros((6,6))``\ , \ ``offset = [0.,0.,0.,0.,0.,0.]``\ , \ ``rotationMatrixJoint = np.eye(3)``\ , \ ``useGlobalFrame = True``\ , \ ``show = True``\ , \ ``drawSize = -1``\ , \ ``color = color4default``\ )

- | \ *function description*\ :
  | helper function to create RigidBodySpringDamper connector, using arguments from ObjectConnectorRigidBodySpringDamper
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateRigidBodySpringDamper.
- | \ *input*\ :
  | \ ``name``\ : name string for connector; markers get Marker0:name and Marker1:name
  | \ ``bodyOrNodeList``\ : a list of object numbers (with specific localPosition0/1) or node numbers; may also be of mixed types
  | \ ``localPosition0``\ : local position (as 3D list or numpy array) on body0, if not a node number
  | \ ``localPosition1``\ : local position (as 3D list or numpy array) on body1, if not a node number
  | \ ``stiffness``\ : stiffness coefficients (as 6D matrix or numpy array)
  | \ ``damping``\ : damping coefficients (as 6D matrix or numpy array)
  | \ ``offset``\ : offset vector (as 6D list or numpy array)
  | \ ``rotationMatrixJoint``\ : additional rotation matrix; in case  useGlobalFrame=False, it transforms body0/node0 local frame to joint frame; if useGlobalFrame=True, it transforms global frame to joint frame
  | \ ``useGlobalFrame``\ : if False, the rotationMatrixJoint is defined in the local coordinate system of body0
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``drawSize``\ : general drawing size of connector
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | ObjectIndex; returns index of newly created object
- | \ *example*\ :

.. code-block:: python

  #TODO


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM)



.. _sec-mainsystemextensions-createrevolutejoint:

Function: CreateRevoluteJoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateRevoluteJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L837>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``position = []``\ , \ ``axis = []``\ , \ ``useGlobalFrame = True``\ , \ ``show = True``\ , \ ``axisRadius = 0.1``\ , \ ``axisLength = 0.4``\ , \ ``color = color4default``\ )

- | \ *function description*\ :
  | Create revolute joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateRevoluteJoint.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of object numbers for body0 and body1; must be rigid body or ground object
  | \ ``position``\ : a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  | \ ``axis``\ : a 3D vector as list or np.array: if  useGlobalFrame=True it describes the global rotation axis of the joint in reference configuration; else: local axis in body0
  | \ ``useGlobalFrame``\ : if False, the point and axis vectors are defined in the local coordinate system of body0
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``axisRadius``\ : radius of axis for connector graphical representation
  | \ ``axisLength``\ : length of axis for connector graphical representation
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | [ObjectIndex, MarkerIndex, MarkerIndex]; returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [3,0,0],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1],
                                                                        color=color4steelblue)])
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b0], position=[2.5,0,0], axis=[0,0,1],
                          useGlobalFrame=True, axisRadius=0.02, axisLength=0.14)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_\  (Ex), \ `solutionViewerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/solutionViewerTest.py>`_\  (Ex), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM), \ `perf3DRigidBodies.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/perf3DRigidBodies.py>`_\  (TM)



.. _sec-mainsystemextensions-createprismaticjoint:

Function: CreatePrismaticJoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreatePrismaticJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L935>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``position = []``\ , \ ``axis = []``\ , \ ``useGlobalFrame = True``\ , \ ``show = True``\ , \ ``axisRadius = 0.1``\ , \ ``axisLength = 0.4``\ , \ ``color = color4default``\ )

- | \ *function description*\ :
  | Create prismatic joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreatePrismaticJoint.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of object numbers for body0 and body1; must be rigid body or ground object
  | \ ``position``\ : a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  | \ ``axis``\ : a 3D vector as list or np.array containing the global translation axis of the joint in reference configuration
  | \ ``useGlobalFrame``\ : if False, the point and axis vectors are defined in the local coordinate system of body0
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``axisRadius``\ : radius of axis for connector graphical representation
  | \ ``axisLength``\ : length of axis for connector graphical representation
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | [ObjectIndex, MarkerIndex, MarkerIndex]; returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [4,0,0],
                           initialVelocity = [0,4,0],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1],
                                                                        color=color4steelblue)])
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreatePrismaticJoint(bodyNumbers=[oGround, b0], position=[3.5,0,0], axis=[0,1,0],
                           useGlobalFrame=True, axisRadius=0.02, axisLength=1)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-createsphericaljoint:

Function: CreateSphericalJoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateSphericalJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1025>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``position = []``\ , \ ``constrainedAxes = [1,1,1]``\ , \ ``useGlobalFrame = True``\ , \ ``show = True``\ , \ ``jointRadius = 0.1``\ , \ ``color = color4default``\ )

- | \ *function description*\ :
  | Create spherical joint between two bodies; definition of joint position in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers are automatically computed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateSphericalJoint.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of object numbers for body0 and body1; must be mass point, rigid body or ground object
  | \ ``position``\ : a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  | \ ``constrainedAxes``\ : flags, which determines which (global) translation axes are constrained; each entry may only be 0 (=free) axis or 1 (=constrained axis)
  | \ ``useGlobalFrame``\ : if False, the point and axis vectors are defined in the local coordinate system of body0
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``jointRadius``\ : radius of sphere for connector graphical representation
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | [ObjectIndex, MarkerIndex, MarkerIndex]; returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [5,0,0],
                           initialAngularVelocity = [5,0,0],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1],
                                                                        color=color4orange)])
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreateSphericalJoint(bodyNumbers=[oGround, b0], position=[5.5,0,0],
                           useGlobalFrame=True, jointRadius=0.06)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-creategenericjoint:

Function: CreateGenericJoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateGenericJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1108>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``position = []``\ , \ ``rotationMatrixAxes = np.eye(3)``\ , \ ``constrainedAxes = [1,1,1, 1,1,1]``\ , \ ``useGlobalFrame = True``\ , \ ``show = True``\ , \ ``axesRadius = 0.1``\ , \ ``axesLength = 0.4``\ , \ ``color = color4default``\ )

- | \ *function description*\ :
  | Create generic joint between two bodies; definition of joint position (position) and axes (rotationMatrixAxes) in global coordinates (useGlobalFrame=True) or in local coordinates of body0 (useGlobalFrame=False), where rotationMatrixAxes is an additional rotation to body0; all markers, markerRotation and other quantities are automatically computed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateGenericJoint.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumber0``\ : a object number for body0, must be rigid body or ground object
  | \ ``bodyNumber1``\ : a object number for body1, must be rigid body or ground object
  | \ ``position``\ : a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  | \ ``rotationMatrixAxes``\ : rotation matrix which defines orientation of constrainedAxes; if useGlobalFrame, this rotation matrix is global, else the rotation matrix is post-multiplied with the rotation of body0, identical with rotationMarker0 in the joint
  | \ ``constrainedAxes``\ : flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; each entry may only be 0 (=free) axis or 1 (=constrained axis); ALL constrained Axes are defined relative to reference rotation of body0 times rotation0
  | \ ``useGlobalFrame``\ : if False, the position is defined in the local coordinate system of body0, otherwise it is defined in global coordinates
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``axesRadius``\ : radius of axes for connector graphical representation
  | \ ``axesLength``\ : length of axes for connector graphical representation
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | [ObjectIndex, MarkerIndex, MarkerIndex]; returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [6,0,0],
                           initialAngularVelocity = [0,8,0],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1],
                                                                        color=color4orange)])
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreateGenericJoint(bodyNumbers=[oGround, b0], position=[5.5,0,0],
                         constrainedAxes=[1,1,1, 1,0,0],
                         rotationMatrixAxes=RotationMatrixX(0.125*pi), #tilt axes
                         useGlobalFrame=True, axesRadius=0.02, axesLength=0.2)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `computeODE2AEeigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2AEeigenvaluesTest.py>`_\  (TM), \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM), \ `rigidBodyCOMtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyCOMtest.py>`_\  (TM)



.. _sec-mainsystemextensions-createdistanceconstraint:

Function: CreateDistanceConstraint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateDistanceConstraint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1211>`__\ (\ ``name = ''``\ , \ ``bodyOrNodeList = [None, None]``\ , \ ``localPosition0 = [0.,0.,0.]``\ , \ ``localPosition1 = [0.,0.,0.]``\ , \ ``distance = None``\ , \ ``show = True``\ , \ ``drawSize = -1.``\ , \ ``color = color4default``\ )

- | \ *function description*\ :
  | Create distance joint between two bodies; definition of joint positions in local coordinates of bodies or nodes; if distance=None, it is computed automatically from reference length; all markers are automatically computed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateDistanceConstraint.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyOrNodeList``\ : a list of object numbers (with specific localPosition0/1) or node numbers; may also be of mixed types
  | \ ``localPosition0``\ : local position (as 3D list or numpy array) on body0, if not a node number
  | \ ``localPosition1``\ : local position (as 3D list or numpy array) on body1, if not a node number
  | \ ``distance``\ : if None, distance is computed from reference position of bodies or nodes; if not None, this distance (which must be always larger than zero) is prescribed between the two positions
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``drawSize``\ : general drawing size of node
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | [ObjectIndex, MarkerIndex, MarkerIndex]; returns list [oJoint, mBody0, mBody1], containing the joint object number, and the two rigid body markers on body0/1 for the joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface, graphicsDataUtilities and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                    sideLengths=[1,0.1,0.1]),
                            referencePosition = [6,0,0],
                            gravity = [0,-9.81,0],
                            graphicsDataList = [GraphicsDataOrthoCubePoint(size=[1,0.1,0.1],
                                                                        color=color4orange)])
  m1 = mbs.CreateMassPoint(referencePosition=[5.5,-1,0],
                           physicsMass=1, drawSize = 0.2)
  n1 = mbs.GetObject(m1)['nodeNumber']
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreateDistanceConstraint(bodyOrNodeList=[oGround, b0],
                               localPosition0 = [6.5,1,0],
                               localPosition1 = [0.5,0,0],
                               distance=None, #automatically computed
                               drawSize=0.06)
  mbs.CreateDistanceConstraint(bodyOrNodeList=[b0, n1],
                               localPosition0 = [-0.5,0,0],
                               localPosition1 = [0.,0.,0.], #must be [0,0,0] for Node
                               distance=None, #automatically computed
                               drawSize=0.06)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)

