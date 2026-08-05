

.. _sec-mainsystemextensions-createground:

Function: CreateGround
^^^^^^^^^^^^^^^^^^^^^^
`CreateGround <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L278>`__\ (\ ``name = ''``\ , \ ``referencePosition = [0.,0.,0.]``\ , \ ``referenceRotationMatrix = np.eye(3)``\ , \ ``graphicsDataList = []``\ , \ ``graphicsDataUserFunction = 0``\ , \ ``show = True``\ )

- | \ *function description*\ :
  | helper function to create a ground object, using arguments of ObjectGround; this function is mainly added for consistency with other mainSystemExtensions
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateGround.
- | \ *input*\ :
  | \ ``name``\ : name string for object
  | \ ``referencePosition``\ : reference coordinates for point node (always a 3D vector, no matter if 2D or 3D mass)
  | \ ``referenceRotationMatrix``\ : reference rotation matrix for rigid body node (always 3D matrix, no matter if 2D or 3D body)
  | \ ``graphicsDataList``\ : list of GraphicsData for optional ground visualization
  | \ ``graphicsDataUserFunction``\ : a user function graphicsDataUserFunction(mbs, itemNumber)->BodyGraphicsData (list of GraphicsData), which can be used to draw user-defined graphics; this is much slower than regular GraphicsData
  | \ ``color``\ : color of node
  | \ ``show``\ : True: show ground object;
- | \ *output*\ :
  | (type: ObjectIndex) returns ground object index
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  ground=mbs.CreateGround(referencePosition = [2,0,0],
                          graphicsDataList = [exu.graphics.CheckerBoard(point=[0,0,0], normal=[0,1,0],size=4)])


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFslidingJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint.py>`_\  (Ex), \ `ballBearningModel.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ballBearningModel.py>`_\  (Ex), \ `basicTutorial2024.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/basicTutorial2024.py>`_\  (Ex), \ `beamTutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beamTutorial.py>`_\  (Ex), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `ANCFCableBeamDampingTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFCableBeamDampingTest.py>`_\  (TM), \ `ANCFThinPlateTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFThinPlateTests.py>`_\  (TM), \ `ballBearingTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ballBearingTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createmasspoint:

Function: CreateMassPoint
^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateMassPoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L346>`__\ (\ ``name = ''``\ , \ ``referencePosition = [0.,0.,0.]``\ , \ ``initialDisplacement = [0.,0.,0.]``\ , \ ``initialVelocity = [0.,0.,0.]``\ , \ ``physicsMass = 0``\ , \ ``gravity = [0.,0.,0.]``\ , \ ``graphicsDataList = []``\ , \ ``drawSize = -1``\ , \ ``color = [-1.,-1.,-1.,-1.]``\ , \ ``show = True``\ , \ ``create2D = False``\ , \ ``returnDict = False``\ )

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
  | \ ``show``\ : True: if graphicsData list is empty, node is shown, otherwise body is shown; False: nothing is shown
  | \ ``create2D``\ : if True, create NodePoint2D and MassPoint2D
  | \ ``returnDict``\ : if False, returns object index; if True, returns dict of all information on created object and node
- | \ *output*\ :
  | (type: Union[dict, ObjectIndex]) returns mass point object index or dict with all data on request (if returnDict=True)
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0=mbs.CreateMassPoint(referencePosition = [0,0,0],
                         initialVelocity = [2,5,0],
                         physicsMass = 1, gravity = [0,-9.81,0],
                         drawSize = 0.5, color=exu.graphics.color.blue)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `basicTutorial2024.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/basicTutorial2024.py>`_\  (Ex), \ `cartesianSpringDamper.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamper.py>`_\  (Ex), \ `cartesianSpringDamperUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamperUserFunction.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `NGsolveOCCgeometry.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveOCCgeometry.py>`_\  (Ex), \ `createContactSphereSphere.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createContactSphereSphere.py>`_\  (TM), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TM), \ `deleteItemsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/deleteItemsTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createrigidbody:

Function: CreateRigidBody
^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateRigidBody <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L480>`__\ (\ ``name = ''``\ , \ ``referencePosition = [0.,0.,0.]``\ , \ ``referenceRotationMatrix = np.eye(3)``\ , \ ``initialVelocity = [0.,0.,0.]``\ , \ ``initialAngularVelocity = [0.,0.,0.]``\ , \ ``initialDisplacement = None``\ , \ ``initialRotationMatrix = None``\ , \ ``inertia = None``\ , \ ``gravity = [0.,0.,0.]``\ , \ ``nodeType = exudyn.NodeType.RotationEulerParameters``\ , \ ``graphicsDataList = []``\ , \ ``graphicsDataUserFunction = 0``\ , \ ``drawSize = -1``\ , \ ``color = [-1.,-1.,-1.,-1.]``\ , \ ``show = True``\ , \ ``create2D = False``\ , \ ``returnDict = False``\ )

- | \ *function description*\ :
  | helper function to create 3D (or 2D) rigid body object and node; all quantities are global (angular velocity, etc.); use this function to easily create a rigid body; graphics can be directly obtained from inertia object, e.g. in case of cylindrical or cuboid shape
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
  | \ ``nodeType``\ : optional exudyn.NodeType to define the rotation parameterization: RotationEulerParameters, RotationRotationVector or RotationRxyz
  | \ ``graphicsDataList``\ : list of GraphicsData for rigid body visualization; use exudyn.graphics functions to create GraphicsData for basic solids
  | \ ``graphicsDataUserFunction``\ : a user function graphicsDataUserFunction(mbs, itemNumber)->BodyGraphicsData (list of GraphicsData), which can be used to draw user-defined graphics; this is much slower than regular GraphicsData
  | \ ``drawSize``\ : general drawing size of node
  | \ ``color``\ : color of node
  | \ ``show``\ : True: if graphicsData list is empty, node is shown, otherwise body is shown; False: nothing is shown
  | \ ``create2D``\ : if True, create NodeRigidBody2D and ObjectRigidBody2D
  | \ ``returnDict``\ : if False, returns object index; if True, returns dict of all information on created object and node
- | \ *output*\ :
  | (type: Union[dict, ObjectIndex]) returns rigid body object index (or dict with 'nodeNumber', 'objectNumber' and possibly 'loadNumber' and 'markerBodyMass' if returnDict=True)
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [1,0,0],
                           initialVelocity = [2,5,0],
                           initialAngularVelocity = [5,0.5,0.7],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                        color=exu.graphics.color.red)])
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Ex), \ `ANCFslidingJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint.py>`_\  (Ex), \ `ballBearningModel.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ballBearningModel.py>`_\  (Ex), \ `ballBearingTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ballBearingTest.py>`_\  (TM), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createspringdamper:

Function: CreateSpringDamper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateSpringDamper <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L718>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``localPosition0 = [0.,0.,0.]``\ , \ ``localPosition1 = [0.,0.,0.]``\ , \ ``referenceLength = None``\ , \ ``stiffness = 0.``\ , \ ``damping = 0.``\ , \ ``force = 0.``\ , \ ``velocityOffset = 0.``\ , \ ``springForceUserFunction = 0``\ , \ ``bodyOrNodeList = [None, None]``\ , \ ``bodyList = [None, None]``\ , \ ``show = True``\ , \ ``drawSize = -1``\ , \ ``color = exudyn.graphics.color.default``\ )

- | \ *function description*\ :
  | helper function to create SpringDamper connector, using arguments from ObjectConnectorSpringDamper; similar interface as CreateDistanceConstraint(...), see there for for further information
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateSpringDamper.
- | \ *input*\ :
  | \ ``name``\ : name string for connector; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of two body numbers (ObjectIndex) to be connected; alternatively, MarkerIndex or NodeIndex can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
  | \ ``localPosition0``\ : local position (as 3D list or numpy array) on body0, if not a node of marker number
  | \ ``localPosition1``\ : local position (as 3D list or numpy array) on body1, if not a node of marker number
  | \ ``referenceLength``\ : if None, length is computed from reference position of bodies or nodes; if not None, this scalar reference length is used for spring
  | \ ``stiffness``\ : scalar stiffness coefficient
  | \ ``damping``\ : scalar damping coefficient
  | \ ``force``\ : scalar additional force applied
  | \ ``velocityOffset``\ : scalar offset: if referenceLength is changed over time, the velocityOffset may be changed accordingly to emulate a reference motion
  | \ ``springForceUserFunction``\ : a user function springForceUserFunction(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force)->float ; this function replaces the internal connector force computation
  | \ ``bodyOrNodeList``\ : alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``drawSize``\ : general drawing size of connector
  | \ ``color``\ : color of connector
  | \ ``bodyList``\ : DEPRECATED
- | \ *output*\ :
  | (type: ObjectIndex) returns index of newly created object
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateMassPoint(referencePosition = [2,0,0],
                           initialVelocity = [2,5,0],
                           physicsMass = 1, gravity = [0,-9.81,0],
                           drawSize = 0.5, color=exu.graphics.color.blue)
  oGround = mbs.AddObject(ObjectGround())
  #add vertical spring
  oSD = mbs.CreateSpringDamper(bodyNumbers=[oGround, b0],
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

    \ `basicTutorial2024.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/basicTutorial2024.py>`_\  (Ex), \ `camFollowerExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/camFollowerExample.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `contactCurveWithLongCurve.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/contactCurveWithLongCurve.py>`_\  (Ex), \ `springDamperTutorialNew.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/springDamperTutorialNew.py>`_\  (Ex), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TM), \ `loadUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/loadUserFunctionTest.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-createcartesianspringdamper:

Function: CreateCartesianSpringDamper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateCartesianSpringDamper <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L827>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``localPosition0 = [0.,0.,0.]``\ , \ ``localPosition1 = [0.,0.,0.]``\ , \ ``stiffness = [0.,0.,0.]``\ , \ ``damping = [0.,0.,0.]``\ , \ ``offset = [0.,0.,0.]``\ , \ ``springForceUserFunction = 0``\ , \ ``bodyOrNodeList = [None, None]``\ , \ ``bodyList = [None, None]``\ , \ ``show = True``\ , \ ``drawSize = -1``\ , \ ``color = exudyn.graphics.color.default``\ )

- | \ *function description*\ :
  | helper function to create CartesianSpringDamper connector, using arguments from ObjectConnectorCartesianSpringDamper
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateCartesianSpringDamper.
- | \ *input*\ :
  | \ ``name``\ : name string for connector; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of two body numbers (ObjectIndex) to be connected; alternatively, MarkerIndex or NodeIndex can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
  | \ ``localPosition0``\ : local position (as 3D list or numpy array) on body0, if not a node of marker number
  | \ ``localPosition1``\ : local position (as 3D list or numpy array) on body1, if not a node of marker number
  | \ ``stiffness``\ : stiffness coefficients (as 3D list or numpy array)
  | \ ``damping``\ : damping coefficients (as 3D list or numpy array)
  | \ ``offset``\ : offset vector (as 3D list or numpy array)
  | \ ``springForceUserFunction``\ : a user function springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset)->[float,float,float] ; this function replaces the internal connector force computation
  | \ ``bodyOrNodeList``\ : alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
  | \ ``bodyList``\ : DEPRECATED
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``drawSize``\ : general drawing size of connector
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | (type: ObjectIndex) returns index of newly created object
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateMassPoint(referencePosition = [7,0,0],
                            physicsMass = 1, gravity = [0,-9.81,0],
                            drawSize = 0.5, color=exu.graphics.color.blue)
  oGround = mbs.AddObject(ObjectGround())
  oSD = mbs.CreateCartesianSpringDamper(bodyNumbers=[oGround, b0],
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

    \ `cartesianSpringDamper.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamper.py>`_\  (Ex), \ `cartesianSpringDamperUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamperUserFunction.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `NGsolveFFRFSlidingJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveFFRFSlidingJoint.py>`_\  (Ex), \ `complexEigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/complexEigenvaluesTest.py>`_\  (TM), \ `computeODE2AEeigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2AEeigenvaluesTest.py>`_\  (TM), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-createrigidbodyspringdamper:

Function: CreateRigidBodySpringDamper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateRigidBodySpringDamper <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L902>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``localPosition0 = [0.,0.,0.]``\ , \ ``localPosition1 = [0.,0.,0.]``\ , \ ``stiffness = np.zeros((6,6))``\ , \ ``damping = np.zeros((6,6))``\ , \ ``offset = [0.,0.,0.,0.,0.,0.]``\ , \ ``rotationMatrixJoint = np.eye(3)``\ , \ ``useGlobalFrame = True``\ , \ ``intrinsicFormulation = True``\ , \ ``springForceTorqueUserFunction = 0``\ , \ ``postNewtonStepUserFunction = 0``\ , \ ``bodyOrNodeList = [None, None]``\ , \ ``bodyList = [None, None]``\ , \ ``show = True``\ , \ ``drawSize = -1``\ , \ ``color = exudyn.graphics.color.default``\ )

- | \ *function description*\ :
  | helper function to create RigidBodySpringDamper connector, using arguments from ObjectConnectorRigidBodySpringDamper, see there for the full documentation
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateRigidBodySpringDamper.
- | \ *input*\ :
  | \ ``name``\ : name string for connector; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of two body numbers (ObjectIndex) to be connected; alternatively, MarkerIndex or NodeIndex can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
  | \ ``localPosition0``\ : local position (as 3D list or numpy array) on body0, if not a node of marker number
  | \ ``localPosition1``\ : local position (as 3D list or numpy array) on body1, if not a node of marker number
  | \ ``stiffness``\ : stiffness coefficients (as 6D matrix or numpy array)
  | \ ``damping``\ : damping coefficients (as 6D matrix or numpy array)
  | \ ``offset``\ : offset vector (as 6D list or numpy array)
  | \ ``rotationMatrixJoint``\ : additional rotation matrix; in case  useGlobalFrame=False, it transforms body0/node0 local frame to joint frame; if useGlobalFrame=True, it transforms global frame to joint frame
  | \ ``useGlobalFrame``\ : if False, the rotationMatrixJoint is defined in the local coordinate system of body0
  | \ ``intrinsicFormulation``\ : if True, uses intrinsic formulation of Maserati and Morandini, which uses matrix logarithm and is independent of order of markers (preferred formulation); otherwise, Tait-Bryan angles are used for computation of torque, see documentation
  | \ ``springForceTorqueUserFunction``\ : a user function springForceTorqueUserFunction(mbs, t, itemNumber, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1, offset)->[float,float,float, float,float,float] ; this function replaces the internal connector force / torque computation
  | \ ``postNewtonStepUserFunction``\ : a special user function postNewtonStepUserFunction(mbs, t, Index itemIndex, dataCoordinates, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1, offset)->[PNerror, recommendedStepSize, data[0], data[1], ...] ; for details, see RigidBodySpringDamper for full docu
  | \ ``bodyOrNodeList``\ : alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
  | \ ``bodyList``\ : DEPRECATED
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``drawSize``\ : general drawing size of connector
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | (type: ObjectIndex) returns index of newly created object
- | \ *example*\ :

.. code-block:: python

  #coming later


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `rigidBodySpringDamperIntrinsic.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodySpringDamperIntrinsic.py>`_\  (TM)



.. _sec-mainsystemextensions-createtorsionalspringdamper:

Function: CreateTorsionalSpringDamper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateTorsionalSpringDamper <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1008>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``position = [0.,0.,0.]``\ , \ ``axis = [0.,0.,0.]``\ , \ ``stiffness = 0.``\ , \ ``damping = 0.``\ , \ ``offset = 0.``\ , \ ``velocityOffset = 0.``\ , \ ``torque = 0.``\ , \ ``useGlobalFrame = True``\ , \ ``springTorqueUserFunction = 0``\ , \ ``unlimitedRotations = True``\ , \ ``show = True``\ , \ ``drawSize = -1``\ , \ ``color = exudyn.graphics.color.default``\ )

- | \ *function description*\ :
  | helper function to create TorsionalSpringDamper connector, using arguments from ObjectConnectorTorsionalSpringDamper, see there for the full documentation
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateTorsionalSpringDamper.
- | \ *input*\ :
  | \ ``name``\ : name string for connector; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of two body numbers (ObjectIndex) to be connected; alternatively, MarkerIndex can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
  | \ ``position``\ : a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  | \ ``axis``\ : a 3D vector as list or np.array containing the axis around which the spring acts, either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True)
  | \ ``stiffness``\ : scalar stiffness of spring
  | \ ``damping``\ : scalar damping added to spring
  | \ ``offset``\ : scalar offset, which can be used to realize a P-controlled actuator
  | \ ``velocityOffset``\ : scalar velocity offset, which can be used to realize a D-controlled actuator
  | \ ``torque``\ : additional constant torque added to spring-damper, acting between the two bodies
  | \ ``useGlobalFrame``\ : if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
  | springTorqueUserFunction : a user function springTorqueUserFunction(mbs, t, itemNumber, rotation, angularVelocity, stiffness, damping, offset)->float ; this function replaces the internal connector torque computation
  | \ ``unlimitedRotations``\ : if True, an additional generic data node is added to enable measurement of rotations beyond +/- pi; this also allows the spring to cope with multiple turns.
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``drawSize``\ : general drawing size of connector
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | (type: ObjectIndex) returns index of newly created object
- | \ *example*\ :

.. code-block:: python

  #coming later


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createrevolutejoint:

Function: CreateRevoluteJoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateRevoluteJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1162>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``position = []``\ , \ ``axis = []``\ , \ ``useGlobalFrame = True``\ , \ ``show = True``\ , \ ``axisRadius = 0.1``\ , \ ``axisLength = 0.4``\ , \ ``color = exudyn.graphics.color.default``\ )

- | \ *function description*\ :
  | Create revolute joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateRevoluteJoint.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of two body numbers (ObjectIndex) to be connected; must be rigid body or ground object; alternatively, MarkerIndex (Rigid) can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
  | \ ``position``\ : a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  | \ ``axis``\ : a 3D vector as list or np.array containing the joint axis either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True)
  | \ ``useGlobalFrame``\ : if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``axisRadius``\ : radius of axis for connector graphical representation
  | \ ``axisLength``\ : length of axis for connector graphical representation
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | (type: ObjectIndex) returns index of created joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [3,0,0],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                        color=exu.graphics.color.steelblue)])
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b0], position=[2.5,0,0], axis=[0,0,1],
                          useGlobalFrame=True, axisRadius=0.02, axisLength=0.14)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Ex), \ `involuteGearGraphics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/involuteGearGraphics.py>`_\  (Ex), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TM), \ `createRollingDiscPenaltyTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createRollingDiscPenaltyTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createprismaticjoint:

Function: CreatePrismaticJoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreatePrismaticJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1262>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``position = []``\ , \ ``axis = []``\ , \ ``useGlobalFrame = True``\ , \ ``show = True``\ , \ ``axisRadius = 0.1``\ , \ ``axisLength = 0.4``\ , \ ``color = exudyn.graphics.color.default``\ )

- | \ *function description*\ :
  | Create prismatic joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreatePrismaticJoint.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of two body numbers (ObjectIndex) to be connected; must be rigid body or ground object; alternatively, MarkerIndex (Rigid) can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
  | \ ``position``\ : a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  | \ ``axis``\ : a 3D vector as list or np.array containing the joint axis either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True)
  | \ ``useGlobalFrame``\ : if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``axisRadius``\ : radius of axis for connector graphical representation
  | \ ``axisLength``\ : length of axis for connector graphical representation
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | (type: ObjectIndex) returns index of created joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [4,0,0],
                           initialVelocity = [0,4,0],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                        color=exu.graphics.color.steelblue)])
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreatePrismaticJoint(bodyNumbers=[oGround, b0], position=[3.5,0,0], axis=[0,1,0],
                           useGlobalFrame=True, axisRadius=0.02, axisLength=1)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Ex), \ `involuteGearGraphics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/involuteGearGraphics.py>`_\  (Ex), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM), \ `pickleCopyMbs.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/pickleCopyMbs.py>`_\  (TM), \ `relativeRotationTranslationMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/relativeRotationTranslationMechanism.py>`_\  (TM)



.. _sec-mainsystemextensions-createsphericaljoint:

Function: CreateSphericalJoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateSphericalJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1353>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``position = []``\ , \ ``constrainedAxes = [1,1,1]``\ , \ ``useGlobalFrame = True``\ , \ ``show = True``\ , \ ``jointRadius = 0.1``\ , \ ``color = exudyn.graphics.color.default``\ )

- | \ *function description*\ :
  | Create spherical joint between two bodies; definition of joint position in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers are automatically computed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateSphericalJoint.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of two body numbers (ObjectIndex) to be connected; must be point mass, rigid body or ground object; alternatively, MarkerIndex can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
  | \ ``position``\ : a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  | \ ``constrainedAxes``\ : flags, which determines which (global) translation axes are constrained; each entry may only be 0 (=free) axis or 1 (=constrained axis)
  | \ ``useGlobalFrame``\ : if False, the point and axis vectors are defined in the local coordinate system of body0
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``jointRadius``\ : radius of sphere for connector graphical representation
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | (type: ObjectIndex) returns index of created joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [5,0,0],
                           initialAngularVelocity = [5,0,0],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                        color=exu.graphics.color.orange)])
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreateSphericalJoint(bodyNumbers=[oGround, b0], position=[5.5,0,0],
                           useGlobalFrame=True, jointRadius=0.06)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `newtonsCradle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/newtonsCradle.py>`_\  (Ex), \ `NGsolveCreateFFRFreducedOrder.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCreateFFRFreducedOrder.py>`_\  (Ex), \ `ANCFThinPlateTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFThinPlateTests.py>`_\  (TM), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TM), \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-creategenericjoint:

Function: CreateGenericJoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateGenericJoint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1436>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``position = []``\ , \ ``rotationMatrixAxes = np.eye(3)``\ , \ ``constrainedAxes = [1,1,1, 1,1,1]``\ , \ ``useGlobalFrame = True``\ , \ ``offsetUserFunction = 0``\ , \ ``offsetUserFunction_t = 0``\ , \ ``show = True``\ , \ ``axesRadius = 0.1``\ , \ ``axesLength = 0.4``\ , \ ``color = exudyn.graphics.color.default``\ )

- | \ *function description*\ :
  | Create generic joint between two bodies; definition of joint position (position) and axes (rotationMatrixAxes) in global coordinates (useGlobalFrame=True) or in local coordinates of body0 (useGlobalFrame=False), where rotationMatrixAxes is an additional rotation to body0; all markers, markerRotation and other quantities are automatically computed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateGenericJoint.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of two body numbers (ObjectIndex) to be connected; must be rigid body or ground object; alternatively, MarkerIndex (Rigid) can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
  | \ ``position``\ : a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  | \ ``rotationMatrixAxes``\ : rotation matrix which defines orientation of constrainedAxes; if useGlobalFrame, this rotation matrix is global, else the rotation matrix is post-multiplied with the rotation of body0, identical with rotationMarker0 in the joint
  | \ ``constrainedAxes``\ : flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; each entry may only be 0 (=free) axis or 1 (=constrained axis); ALL constrained Axes are defined relative to reference rotation of body0 times rotation0
  | \ ``useGlobalFrame``\ : if False, the position is defined in the local coordinate system of body0, otherwise it is defined in global coordinates
  | \ ``offsetUserFunction``\ : a user function offsetUserFunction(mbs, t, itemNumber, offsetUserFunctionParameters)->float ; this function replaces the internal (constant) by a user-defined offset. This allows to realize rheonomic joints and allows kinematic simulation
  | \ ``offsetUserFunction_t``\ : a user function offsetUserFunction_t(mbs, t, itemNumber, offsetUserFunctionParameters)->float ; this function replaces the internal (constant) by a user-defined offset velocity; this function is used instead of offsetUserFunction, if velocityLevel (index2) time integration
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``axesRadius``\ : radius of axes for connector graphical representation
  | \ ``axesLength``\ : length of axes for connector graphical representation
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | (type: ObjectIndex) returns index of created joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [6,0,0],
                           initialAngularVelocity = [0,8,0],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                        color=exu.graphics.color.orange)])
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

    \ `bungeeJump.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bungeeJump.py>`_\  (Ex), \ `NGsolveFFRFSlidingJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveFFRFSlidingJoint.py>`_\  (Ex), \ `pistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pistonEngine.py>`_\  (Ex), \ `universalJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/universalJoint.py>`_\  (Ex), \ `ANCFCableBeamDampingTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFCableBeamDampingTest.py>`_\  (TM), \ `ANCFThinPlateTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFThinPlateTests.py>`_\  (TM), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `complexEigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/complexEigenvaluesTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createdistanceconstraint:

Function: CreateDistanceConstraint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateDistanceConstraint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1547>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``localPosition0 = [0.,0.,0.]``\ , \ ``localPosition1 = [0.,0.,0.]``\ , \ ``distance = None``\ , \ ``bodyOrNodeList = [None, None]``\ , \ ``bodyList = [None, None]``\ , \ ``show = True``\ , \ ``drawSize = -1.``\ , \ ``color = exudyn.graphics.color.default``\ )

- | \ *function description*\ :
  | Create distance joint between two bodies; definition of joint positions in local coordinates of bodies or nodes; if distance=None, it is computed automatically from reference length; all markers are automatically computed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateDistanceConstraint.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of two body numbers (ObjectIndex) to be connected; alternatively, MarkerIndex can be used instead of ObjectIndex, setting localPosition0/1==[0,0,0]
  | \ ``localPosition0``\ : local position (as 3D list or numpy array) on body0, if not a node or marker number
  | \ ``localPosition1``\ : local position (as 3D list or numpy array) on body1, if not a node or marker number
  | \ ``distance``\ : if None, distance is computed from reference position of bodies or nodes; if not None, this distance is prescribed between the two positions; if distance = 0, it will create a SphericalJoint as this case is not possible with a DistanceConstraint
  | \ ``bodyOrNodeList``\ : alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
  | \ ``bodyList``\ : DEPRECATED
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``drawSize``\ : general drawing size of node
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | (type: ObjectIndex) returns index of created joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                    sideLengths=[1,0.1,0.1]),
                            referencePosition = [6,0,0],
                            gravity = [0,-9.81,0],
                            graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                        color=exu.graphics.color.orange)])
  m1 = mbs.CreateMassPoint(referencePosition=[5.5,-1,0],
                           physicsMass=1, drawSize = 0.2)
  n1 = mbs.GetObject(m1)['nodeNumber']
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreateDistanceConstraint(bodyNumbers=[oGround, b0],
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

    \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Ex), \ `newtonsCradle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/newtonsCradle.py>`_\  (Ex), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TM), \ `deleteItemsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/deleteItemsTest.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM), \ `taskmanagerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/taskmanagerTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createcoordinateconstraint:

Function: CreateCoordinateConstraint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateCoordinateConstraint <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1681>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``coordinates = [None, None]``\ , \ ``offset = 0.``\ , \ ``factorValue1 = 1.``\ , \ ``velocityLevel = False``\ , \ ``offsetUserFunction = 0``\ , \ ``offsetUserFunction_t = 0``\ , \ ``show = True``\ , \ ``drawSize = -1.``\ , \ ``color = exudyn.graphics.color.default``\ )

- | \ *function description*\ :
  | Create coordinate constraint for two bodies, or body on ground; markers and NodePointGround are automatically created when needed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateCoordinateConstraint.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of two body numbers (ObjectIndex) to be constrained
  | \ ``coordinates``\ : a list of two coordinates for the respective bodies (in case of ground, it shall be None)
  | \ ``offset``\ : an fixed offset between the two coordinate values
  | \ ``factorValue1``\ : an additional factor multiplied with coordinate value1 used in algebraic equation, to enable (e.g. gear) ratio between coordinates
  | \ ``velocityLevel``\ : If true: connector constrains velocities (only works for ODE2 coordinates!); offset is used between velocities; if True, the offsetUserFunction_t is considered and offsetUserFunction is ignored
  | \ ``offsetUserFunction``\ : a Python function which defines the time-dependent offset; see description in CoordinateConstraint
  | \ ``offsetUserFunction_t``\ : time derivative of offsetUserFunction; needed for velocity level constraints; see description in CoordinateConstraint
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``drawSize``\ : general drawing size of node
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | (type: ObjectIndex) returns index of created joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                    sideLengths=[1,0.1,0.1]),
                            referencePosition = [6,0,0],
                            gravity = [0,-9.81,0],
                            graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                        color=exu.graphics.color.orange)])
  m1 = mbs.CreateMassPoint(referencePosition=[5.5,-1,0],
                           physicsMass=1, drawSize = 0.2)
  mbs.CreateCoordinateConstraint(bodyNumbers=[None, b0],
                                 coordinates=[None, 0]) #constrains X-coordinate
  #constrain Y-coordinate of b0 to Z-coordinate of m1:
  mbs.CreateCoordinateConstraint(bodyNumbers=[b0, m1],
                                 coordinates=[1, 2])
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ballBearningModel.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ballBearningModel.py>`_\  (Ex), \ `camFollowerExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/camFollowerExample.py>`_\  (Ex), \ `involuteGearGraphics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/involuteGearGraphics.py>`_\  (Ex), \ `ballBearingTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ballBearingTest.py>`_\  (TM), \ `contactCurveExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/contactCurveExample.py>`_\  (TM), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createrollingdisc:

Function: CreateRollingDisc
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateRollingDisc <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1822>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``axisPosition = []``\ , \ ``axisVector = [1,0,0]``\ , \ ``discRadius = 0.``\ , \ ``planePosition = [0,0,0]``\ , \ ``planeNormal = [0,0,1]``\ , \ ``constrainedAxes = [1,1,1]``\ , \ ``activeConnector = True``\ , \ ``show = True``\ , \ ``discWidth = 0.1``\ , \ ``color = exudyn.graphics.color.default``\ )

- | \ *function description*\ :
  | Create an ideal rolling disc joint between wheel rigid body and ground; the disc is infinitely thin and the ground is a perfectly flat plane; the wheel may lift off; definition of joint position and axis in global coordinates (alternatively in wheel (body1) local coordinates) for reference configuration of bodies; all markers and other quantities are automatically computed; some constraint conditions may be deactivated, e.g. to resolve redundancy of constraints for multi-wheel vehicles
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateRollingDisc.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of object numbers for body0=ground and body1=wheel; must be rigid body or ground object
  | \ ``axisPosition``\ : a 3D vector as list or np.array: position of wheel axis in local body1=wheel coordinates
  | \ ``axisVector``\ : a 3D vector as list or np.array containing the joint (=wheel) axis in local body1=wheel coordinates
  | \ ``discRadius``\ : radius of the disc
  | \ ``planePosition``\ : any 3D position vector of plane in ground object; given as local coordinates in ground object
  | \ ``planeNormal``\ : 3D normal vector of the rolling (contact) plane on ground; given as local coordinates in ground object
  | \ ``constrainedAxes``\ : [j0,j1,j2] flags, which determine which constraints are active, in which j0 represents the constraint for lateral motion, j1 longitudinal (forward/backward) motion and j2 represents the normal (contact) direction
  | \ ``activeConnector``\ : flag to activate or deactivate the joint
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``discWidth``\ : disc with, only used for drawing
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | (type: ObjectIndex) returns index of created joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  r = 0.2
  oDisc = mbs.CreateRigidBody(inertia = InertiaCylinder(density=5000, length=0.1, outerRadius=r, axis=0),
                            referencePosition = [1,0,r],
                            initialAngularVelocity = [-3*2*pi,0,0],
                            initialVelocity = [0,r*3*2*pi,0],
                            gravity = [0,0,-9.81],
                            graphicsDataList = [exu.graphics.Cylinder(pAxis = [-0.05,0,0], vAxis = [0.1,0,0], radius = r*0.99,
                                                                      color=exu.graphics.color.blue),
                                                exu.graphics.Basis(length=2*r)])
  oGround = mbs.CreateGround(graphicsDataList=[exu.graphics.CheckerBoard(size=4)])
  mbs.CreateRollingDisc(bodyNumbers=[oGround, oDisc],
                        axisPosition=[0,0,0], axisVector=[1,0,0], #on local wheel frame
                        planePosition = [0,0,0], planeNormal = [0,0,1],  #in ground frame
                        discRadius = r,
                        discWidth=0.01, color=exu.graphics.color.steelblue)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings()
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TM), \ `createRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createRollingDiscTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createrollingdiscpenalty:

Function: CreateRollingDiscPenalty
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateRollingDiscPenalty <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L1931>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``axisPosition = []``\ , \ ``axisVector = [1,0,0]``\ , \ ``discRadius = 0.``\ , \ ``planePosition = [0,0,0]``\ , \ ``planeNormal = [0,0,1]``\ , \ ``contactStiffness = 0.``\ , \ ``contactDamping = 0.``\ , \ ``dryFriction = [0,0]``\ , \ ``dryFrictionAngle = 0.``\ , \ ``dryFrictionProportionalZone = 0.``\ , \ ``viscousFriction = [0,0]``\ , \ ``rollingFrictionViscous = 0.``\ , \ ``useLinearProportionalZone = False``\ , \ ``activeConnector = True``\ , \ ``show = True``\ , \ ``discWidth = 0.1``\ , \ ``color = exudyn.graphics.color.default``\ )

- | \ *function description*\ :
  | Create penalty-based rolling disc joint between wheel rigid body and ground; the disc is infinitely thin and the ground is a perfectly flat plane; the wheel may lift off; definition of joint position and axis in global coordinates (alternatively in wheel (body1) local coordinates) for reference configuration of bodies; all markers and other quantities are automatically computed
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateRollingDiscPenalty.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of object numbers for body0=ground and body1=wheel; must be rigid body or ground object
  | \ ``axisPosition``\ : a 3D vector as list or np.array: position of wheel axis in local body1=wheel coordinates
  | \ ``axisVector``\ : a 3D vector as list or np.array containing the joint (=wheel) axis in local body1=wheel coordinates
  | \ ``discRadius``\ : radius of the disc
  | \ ``planePosition``\ : any 3D position vector of plane in ground object; given as local coordinates in ground object
  | \ ``planeNormal``\ : 3D normal vector of the rolling (contact) plane on ground; given as local coordinates in ground object
  | \ ``dryFrictionAngle``\ : angle (radiant) which defines a rotation of the local tangential coordinates dry friction; this allows to model Mecanum wheels with specified roll angle
  | \ ``contactStiffness``\ : normal contact stiffness
  | \ ``contactDamping``\ : normal contact damping
  | \ ``dryFriction``\ : 2D list of friction parameters; dry friction coefficients in local wheel coordinates, where for dryFrictionAngle=0, the first parameter refers to forward direction and the second parameter to lateral direction
  | \ ``viscousFriction``\ : 2D list of viscous friction coefficients [SI:1/(m/s)] in local wheel coordinates; proportional to slipping velocity, leading to increasing slipping friction force for increasing slipping velocity; directions are same as in dryFriction
  | \ ``dryFrictionProportionalZone``\ : limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations)
  | \ ``rollingFrictionViscous``\ : rolling friction [SI:1], which acts against the velocity of the trail on ground and leads to a force proportional to the contact normal force;
  | \ ``useLinearProportionalZone``\ : if True, a linear proportional zone is used; the linear zone performs better in implicit time integration as the Jacobian has a constant tangent in the sticking case
  | \ ``activeConnector``\ : flag to activate or deactivate the connector
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``discWidth``\ : disc with, only used for drawing
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | (type: ObjectIndex) returns index of created joint
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  r = 0.2
  oDisc = mbs.CreateRigidBody(inertia = InertiaCylinder(density=5000, length=0.1, outerRadius=r, axis=0),
                            referencePosition = [1,0,r],
                            initialAngularVelocity = [-3*2*pi,0,0],
                            initialVelocity = [0,r*3*2*pi,0],
                            gravity = [0,0,-9.81],
                            graphicsDataList = [exu.graphics.Cylinder(pAxis = [-0.05,0,0], vAxis = [0.1,0,0], radius = r*0.99,
                                                                      color=exu.graphics.color.blue),
                                                exu.graphics.Basis(length=2*r)])
  oGround = mbs.CreateGround(graphicsDataList=[exu.graphics.CheckerBoard(size=4)])
  mbs.CreateRollingDiscPenalty(bodyNumbers=[oGround, oDisc], axisPosition=[0,0,0], axisVector=[1,0,0],
                                discRadius = r, planePosition = [0,0,0], planeNormal = [0,0,1],
                                dryFriction = [0.2,0.2],
                                contactStiffness = 1e5, contactDamping = 2e3,
                                discWidth=0.01, color=exu.graphics.color.steelblue)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings()
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TM), \ `createRollingDiscPenaltyTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createRollingDiscPenaltyTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createspherespherecontact:

Function: CreateSphereSphereContact
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateSphereSphereContact <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L2036>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``localPosition0 = [0.,0.,0.]``\ , \ ``localPosition1 = [0.,0.,0.]``\ , \ ``spheresRadii = [-1,-1]``\ , \ ``isHollowSphere1 = False``\ , \ ``dynamicFriction = 0.``\ , \ ``frictionProportionalZone = 1e-3``\ , \ ``contactStiffness = 0.``\ , \ ``contactDamping = 0.``\ , \ ``contactStiffnessExponent = 1``\ , \ ``constantPullOffForce = 0``\ , \ ``contactPlasticityRatio = 0``\ , \ ``adhesionCoefficient = 0``\ , \ ``adhesionExponent = 1``\ , \ ``restitutionCoefficient = 1``\ , \ ``minimumImpactVelocity = 0``\ , \ ``impactModel = 0``\ , \ ``dataInitialCoordinates = [0,0,0,0]``\ , \ ``activeConnector = True``\ , \ ``bodyOrNodeList = [None, None]``\ , \ ``show = False``\ , \ ``color = exudyn.graphics.color.default``\ )

- | \ *function description*\ :
  | Create penalty-based sphere-sphere contact between two rigid bodies, mass points (if friction coefficient is zero) or according nodes; the contact is based on ObjectContactSphereSphere; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateSphereSphereContact.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of object numbers for sphere0 and sphere1; Note that if body is a mass point, friction due to rolling is not accounted for!
  | \ ``localPosition0``\ : local position (as 3D list or numpy array) of sphere0 on body0, if not a node number
  | \ ``localPosition1``\ : local position (as 3D list or numpy array) of sphere1 on body1, if not a node number
  | \ ``spheresRadii``\ : list containing radius of sphere 0 and radius of sphere 1 [SI:m].
  | \ ``isHollowSphere1``\ : flag, which determines, if sphere attached to marker 1 (radius 1) is a hollow sphere.
  | \ ``dynamicFriction``\ : dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, Section Module: physics
  | \ ``frictionProportionalZone``\ : limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), Section Module: physics
  | \ ``contactStiffness``\ : normal contact stiffness
  | \ ``contactDamping``\ : linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
  | \ ``contactStiffnessExponent``\ : exponent in normal contact model [SI:1]
  | \ ``constantPullOffForce``\ : constant adhesion force [SI:N]; Edinburgh Adhesive Elasto-Plastic Model
  | \ ``contactPlasticityRatio``\ : ratio of contact stiffness for first loading and unloading/reloading [SI:1]; Edinburgh Adhesive Elasto-Plastic Model; see ObjectContactSphereSphere
  | \ ``adhesionCoefficient``\ : coefficient for adhesion [SI:N/m]; Edinburgh Adhesive Elasto-Plastic Model; set to 0 to deactivate adhesion model
  | \ ``adhesionExponent``\ : exponent for adhesion coefficient [SI:1]; Edinburgh Adhesive Elasto-Plastic Model
  | \ ``restitutionCoefficient``\ : coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)
  | \ ``minimumImpactVelocity``\ : minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)
  | \ ``impactModel``\ : number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!
  | \ ``dataInitialCoordinates``\ : a list of four values for initialization of the data node, used for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused
  | \ ``activeConnector``\ : flag to activate or deactivate the connector
  | \ ``bodyOrNodeList``\ : alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | (type: ObjectIndex) returns index of created joint

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `createContactSphereSphere.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createContactSphereSphere.py>`_\  (TM), \ `createSphereQuadContact.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createSphereQuadContact.py>`_\  (TM), \ `createSphereQuadContact2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createSphereQuadContact2.py>`_\  (TM), \ `createSphereTriangleContact.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createSphereTriangleContact.py>`_\  (TM)



.. _sec-mainsystemextensions-createspherequadcontact:

Function: CreateSphereQuadContact
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateSphereQuadContact <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L2173>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``localPosition0 = [0.,0.,0.]``\ , \ ``radiusSphere = 0``\ , \ ``quadPoints = exudyn.Vector3DList([[0,0,0],[1,0,0],[1,1,0],[0,1,0]])``\ , \ ``includeEdges = 15``\ , \ ``dynamicFriction = 0.``\ , \ ``frictionProportionalZone = 1e-3``\ , \ ``contactStiffness = 0.``\ , \ ``contactDamping = 0.``\ , \ ``contactStiffnessExponent = 1``\ , \ ``restitutionCoefficient = 1``\ , \ ``minimumImpactVelocity = 0``\ , \ ``impactModel = 0``\ , \ ``dataInitialCoordinates = [0,0,0,0]``\ , \ ``activeConnector = True``\ , \ ``bodyOrNodeList = [None, None]``\ , \ ``localPosition1 = [0.,0.,0.]``\ , \ ``show = False``\ , \ ``color = exudyn.graphics.color.default``\ )

- | \ *function description*\ :
  | Create penalty-based sphere-quad contact between two rigid bodies, mass points or according nodes; the contact is based on two ObjectContactSphereTriangle; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateSphereQuadContact.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of object numbers for sphere (0) and quad (1); Note that if body is a mass point, friction due to rolling is not accounted for!
  | \ ``localPosition0``\ : local position (as 3D list or numpy array) of sphere0 on body0, if not a node number
  | \ ``radiusSphere``\ : radius of sphere 0 [SI:m].
  | \ ``quadPoints``\ : 4 points as Vector3DList, list or numpy array to define the quad, defined in body1 local coordinates; note that the quad is split into two triangles with point indices [0,1,3] and [1,2,3]
  | \ ``includeEdges``\ : binary flag, where 1 defines contact with edges 0, 2 with edge 1, 4 with edge 2 and 8 with edge 3; 15 means that contact with all edges is included; edge 0 is the edge between node 0 and node 1, etc.
  | \ ``dynamicFriction``\ : dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, Section Module: physics
  | \ ``frictionProportionalZone``\ : limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), Section Module: physics
  | \ ``contactStiffness``\ : normal contact stiffness
  | \ ``contactDamping``\ : linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
  | \ ``contactStiffnessExponent``\ : exponent in normal contact model [SI:1]
  | \ ``restitutionCoefficient``\ : coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)
  | \ ``minimumImpactVelocity``\ : minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)
  | \ ``impactModel``\ : number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!
  | \ ``dataInitialCoordinates``\ : a list of four values for initialization of the data node, used for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused
  | \ ``activeConnector``\ : flag to activate or deactivate the connector
  | \ ``bodyOrNodeList``\ : alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
  | \ ``localPosition1``\ : local position (as 3D list or numpy array) of quad1 on body1; this is usually not needed and adds simply an offset to the quad coordinates
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | (type: dict) dictionary containing oContact0 and oContact1 with ObjectIndex of each contact object

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `rendererNOGLFWexample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rendererNOGLFWexample.py>`_\  (Ex), \ `createSphereQuadContact.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createSphereQuadContact.py>`_\  (TM), \ `createSphereQuadContact2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createSphereQuadContact2.py>`_\  (TM), \ `createSphereTriangleContact.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createSphereTriangleContact.py>`_\  (TM)



.. _sec-mainsystemextensions-createspheretrianglecontact:

Function: CreateSphereTriangleContact
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateSphereTriangleContact <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L2309>`__\ (\ ``name = ''``\ , \ ``bodyNumbers = [None, None]``\ , \ ``localPosition0 = [0.,0.,0.]``\ , \ ``radiusSphere = 0``\ , \ ``trianglePoints = exudyn.Vector3DList([[0,0,0],[1,0,0],[0,1,0]])``\ , \ ``includeEdges = 7``\ , \ ``dynamicFriction = 0.``\ , \ ``frictionProportionalZone = 1e-3``\ , \ ``contactStiffness = 0.``\ , \ ``contactDamping = 0.``\ , \ ``contactStiffnessExponent = 1``\ , \ ``restitutionCoefficient = 1``\ , \ ``minimumImpactVelocity = 0``\ , \ ``impactModel = 0``\ , \ ``dataInitialCoordinates = [0,0,0,0]``\ , \ ``activeConnector = True``\ , \ ``bodyOrNodeList = [None, None]``\ , \ ``localPosition1 = [0.,0.,0.]``\ , \ ``show = False``\ , \ ``color = exudyn.graphics.color.default``\ )

- | \ *function description*\ :
  | Create penalty-based sphere-triangle contact between two rigid bodies, mass points or according nodes; the contact is based on ObjectContactSphereTriangle; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateSphereTriangleContact.
- | \ *input*\ :
  | \ ``name``\ : name string for joint; markers get Marker0:name and Marker1:name
  | \ ``bodyNumbers``\ : a list of object numbers for sphere (0) and triangle (1); Note that if body is a mass point, friction due to rolling is not accounted for!
  | \ ``localPosition0``\ : local position (as 3D list or numpy array) of sphere0 on body0, if not a node number
  | \ ``radiusSphere``\ : radius of sphere 0 [SI:m].
  | \ ``trianglePoints``\ : triangle points as Vector3DList, list or numpy array to define the quad, defined in body1 local coordinates
  | \ ``includeEdges``\ : binary flag, where 1 defines contact with edges 0, 2 with edge 1 and 4 with edge 2; 7 means that contact with all edges is included; edge 0 is the edge between node 0 and node 1, etc.
  | \ ``dynamicFriction``\ : dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, Section Module: physics
  | \ ``frictionProportionalZone``\ : limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), Section Module: physics
  | \ ``contactStiffness``\ : normal contact stiffness
  | \ ``contactDamping``\ : linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
  | \ ``contactStiffnessExponent``\ : exponent in normal contact model [SI:1]
  | \ ``restitutionCoefficient``\ : coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)
  | \ ``minimumImpactVelocity``\ : minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)
  | \ ``impactModel``\ : number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!
  | \ ``dataInitialCoordinates``\ : a list of four values for initialization of the data node, used for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused
  | \ ``activeConnector``\ : flag to activate or deactivate the connector
  | \ ``bodyOrNodeList``\ : alternative to bodyNumbers; a list of object numbers (with specific localPosition0/1) or node numbers; may alse be mixed types; to use this case, set bodyNumbers = [None,None]
  | \ ``localPosition1``\ : local position (as 3D list or numpy array) of triangle1 on body1; this is usually not needed and adds simply an offset to the triangle coordinates
  | \ ``show``\ : if True, connector visualization is drawn
  | \ ``color``\ : color of connector
- | \ *output*\ :
  | (type: ObjectIndex) returns index of created joint

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `createSphereQuadContact.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createSphereQuadContact.py>`_\  (TM), \ `createSphereTriangleContact.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createSphereTriangleContact.py>`_\  (TM)



.. _sec-mainsystemextensions-createkinematictree:

Function: CreateKinematicTree
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateKinematicTree <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L2438>`__\ (\ ``name = ''``\ , \ ``listOfTreeLinks = []``\ , \ ``referenceCoordinates = None``\ , \ ``initialCoordinates = None``\ , \ ``initialCoordinates_t = None``\ , \ ``gravity = [0.,0.,0.]``\ , \ ``baseOffset = [0.,0.,0.]``\ , \ ``linkForces = None``\ , \ ``linkTorques = None``\ , \ ``jointForceVector = None``\ , \ ``jointPositionOffsetVector = None``\ , \ ``jointVelocityOffsetVector = None``\ , \ ``forceUserFunction = 0``\ , \ ``jointRadius = 0.05``\ , \ ``jointWidth = 0.12``\ , \ ``colors = exudyn.graphics.color.default``\ , \ ``colorsJoints = exudyn.graphics.color.default``\ , \ ``baseGraphicsDataList = None``\ , \ ``linkRoundness = 0.2``\ , \ ``show = True``\ )

- | \ *function description*\ :
  | helper function to create 2D or 3D mass point object and node, using arguments as in NodePoint and MassPoint; uses TreeLink as defined in exudyn.rigidBodyUtilities
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateKinematicTree.
- | \ *input*\ :
  | \ ``name``\ : name string for object, node is 'Node:'+name
  | \ ``listOfTreeLinks``\ : list of TreeLink (from exudyn.rigidBodyUtilities) which characterize the KinematicTree
  | \ ``referenceCoordinates``\ : reference coordinates all kinematic tree coordinates (e.g., joint angles); i.e., configuration where displacements are zero
  | \ ``initialCoordinates``\ : initial deviation from reference coordinates (= displacements)
  | \ ``initialCoordinates_t``\ : initial velocities (e.g., of joint angles)
  | \ ``gravity``\ : gravity vevtor applied to kinematic tree (always a 3D vector, no matter if 2D or 3D mass)
  | \ ``baseOffset``\ : constant 3D vector representing the origin of the kinematic tree
  | \ ``linkForces``\ : Vector3DList of forces per link (at joint origin) or None
  | \ ``linkTorques``\ : Vector3DList of torques per link or None
  | \ ``jointForceVector``\ : a list or numpy array of scalar forces per joint, representing joint forces (prismatic joint) or joint torques (revolute joint)
  | \ ``jointPositionOffsetVector``\ : a list or numpy array of scalar set coordinates per joint; use PreStepUserFunction to change values over time
  | \ ``jointVelocityOffsetVector``\ : a list or numpy array of scalar set velocities per joint; use PreStepUserFunction to change values over time
  | \ ``forceUserFunction``\ : A Python user function which computes the generalized force vector on RHS with identical action as jointForceVector; for description see ObjectKinematicTree
  | \ ``show``\ : show kinematic tree
  | \ ``showLinks``\ : set true, if links shall be shown; if graphicsDataList is empty, a standard drawing for links is used (drawing a cylinder from previous joint or base to next joint; size relative to frame size in KinematicTree visualization settings); else graphicsDataList are used per link; NOTE visualization of joint and COM frames can be modified via visualizationSettings.bodies.kinematicTree
  | \ ``showJoints``\ : set true, if joints shall be shown; if graphicsDataList is empty, a standard drawing for joints is used (drawing a cylinder for revolute joints; size relative to frame size in KinematicTree visualization settings)
  | \ ``jointRadius``\ : for generic visualization of joints and links
  | \ ``jointWidth``\ : for generic visualization of joints and links
  | \ ``colors``\ : either one general color for kinematic tree, or list with one color per link
  | \ ``colorsJoints``\ : either one color for all joints or list with one color per joint
  | \ ``baseGraphicsDataList``\ : graphics for base; if None, it is computed automatically; otherwise a list of graphicsData or empty list
  | \ ``linkRoundness``\ : for automatic generation of graphics for links, roundness=0 give brick-shape, roundness<1 give transition of brick to ellipsoid and roundness=1 give cylinders
  | \ ``show``\ : show kinematic tree
- | \ *output*\ :
  | (type: ObjectIndex) returns kinematic tree object index

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Ex), \ `kinematicTreeAndMBS.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreeAndMBS.py>`_\  (Ex), \ `kinematicTreePendulum.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/kinematicTreePendulum.py>`_\  (Ex), \ `openAIgymNLinkAdvanced.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymNLinkAdvanced.py>`_\  (Ex), \ `openAIgymNLinkContinuous.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/openAIgymNLinkContinuous.py>`_\  (Ex), \ `createKinematicTreeTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createKinematicTreeTest.py>`_\  (TM), \ `kinematicTreeAndMBStest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeAndMBStest.py>`_\  (TM), \ `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createffrfreducedorderobject:

Function: CreateFFRFReducedOrderObject
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateFFRFReducedOrderObject <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L2771>`__\ (\ ``name``\ , \ ``femInterface``\ , \ ``referencePosition = [0., 0., 0.]``\ , \ ``initialVelocity = [0., 0., 0.]``\ , \ ``referenceRotationMatrix = np.eye(3)``\ , \ ``initialAngularVelocity = [0., 0., 0.]``\ , \ ``massProportionalDamping = 0.``\ , \ ``stiffnessProportionalDamping = 0.``\ , \ ``gravity = [0., 0., 0.]``\ , \ ``color = exudyn.graphics.color.defaultFFRF``\ , \ ``superElementRigidMarkersOffsets = None``\ , \ ``showMarkers = True``\ , \ ``verbose = False``\ )

- | \ *function description*\ :
  | Create an FFRF reduced order object; the function adds SuperElementRigid markers if boundaries are defined in the given femInterface and thus enables straightforward integration of flexible bodies into a multibody system
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateFFRFReducedOrderObject.
- | \ *input*\ :
  | \ ``name``\ : name of the FFRF reduced order object; used to name the created SuperElementRigid markers (name + ':' + boundaryName), the rigid body node ('NodeRigidBody:' + name), and the generic ODE2 node ('NodeGeneric:' + name); if no name is available, set name=None
  | \ ``femInterface``\ : an instance of EXUDYN's FEMinterface class; this instance must hold at least a position-based mesh and eigenmodes of the system (for model reduction); usually, also boundaries named [boundaryName0, boundaryName1, ...] are defined within the femInterface; if no boundaries are defined, no SuperElementRigid markers are added
  | \ ``referencePosition``\ : reference position of the floating frame (i.e. of the rigid body node) (always a 3D vector)
  | \ ``initialVelocity``\ : initial velocity of the floating frame (i.e. of the rigid body node) (always a 3D vector)
  | \ ``referenceRotationMatrix``\ : reference rotation matrix for the floating frame (i.e. of the rigid body node) (always a 3D matrix)
  | \ ``initialAngularVelocity``\ : initial angular velocity of the floating frame (i.e. of the rigid body node) (always a 3D vector)
  | \ ``massProportionalDamping``\ : Rayleigh damping factor for mass proportional damping (multiplied with reduced mass matrix), added to floating frame/modal coordinates only
  | \ ``stiffnessProportionalDamping``\ : Rayleigh damping factor for stiffness proportional damping (multiplied with reduced stiffness matrix), added to floating frame/modal coordinates only
  | \ ``gravity``\ : gravity applied to the FFRF reduced order object (always a 3D vector)
  | \ ``color``\ : color with which the FFRF reduced order object is drawn (if no contour is set in the visualization settings)
  | \ ``superElementRigidMarkersOffsets``\ : if not None, adds local offsets to the created SuperElementRigid markers; if N boundaries are defined in the femInterface, a N x 3 list or np.array sets an offset for each added marker; the order of the offsets follows the order in [boundaryName0, boundaryName1, ...] used when setting up the femInterface
  | \ ``showMarkers``\ : if True, SuperElementRigid markers are drawn
  | \ ``verbose``\ : if True, additional information will be printed in the console upon calling the function
- | \ *output*\ :
  | (type: dict) dictionary mapping each created SuperElementRigid marker name to its marker number, plus an additional entry under 'FFRFReducedOrderObjectDict' containing information about the created FFRF reduced order object
- | \ *author*\ :
  | Sebastian Weyrer
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.FEM import * # includes fem functionality
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  from netgen import occ
  import ngsolve as ngs
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  materials = {'steel':{'youngsModulus':2e11, 'poissonsRatio':0.3, 'density':7850}}
  cuboid = occ.Box((0, -0.1/2, -0.1/2), (1, 0.1/2, 0.1/2))
  boundaryNamesList = ['boundary0', 'boundary1']
  cuboid.faces.Min((1, 0, 0)).name = boundaryNamesList[0]
  cuboid.faces.Max((1, 0, 0)).name = boundaryNamesList[1]
  cuboid.name = 'steel'
  geo = occ.OCCGeometry(cuboid)
  mesh = ngs.Mesh(geo.GenerateMesh(maxh=0.05))
  cuboidFemInterface = FEMinterface()
  cuboidFemInterface.ImportMeshFromNGsolve(mesh=mesh,
                                           materials=materials,
                                           boundaryNamesList=boundaryNamesList,
                                           meshOrder=1)
  [boundaryNodesList, boundaryWeightsList] = cuboidFemInterface.GetBoundaryNodeSetsAsLists()
  cuboidFemInterface.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryNodesList,
                                                   nEigenModes=6,
                                                   boundaryNodesWeights=boundaryWeightsList)
  createFFRFObjectDict = mbs.CreateFFRFReducedOrderObject(name='cuboid',
                                                          femInterface=cuboidFemInterface)
  mboundary0 = createFFRFObjectDict['cuboid:boundary0']
  mboundary1 = createFFRFObjectDict['cuboid:boundary1']
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  SC.visualizationSettings.nodes.show = False
  mbs.SolveDynamic(simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `NGsolveCreateFFRFreducedOrder.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCreateFFRFreducedOrder.py>`_\  (Ex), \ `NGsolveFFRFSlidingJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveFFRFSlidingJoint.py>`_\  (Ex)



.. _sec-mainsystemextensions-createforce:

Function: CreateForce
^^^^^^^^^^^^^^^^^^^^^
`CreateForce <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L2940>`__\ (\ ``name = ''``\ , \ ``bodyNumber = None``\ , \ ``loadVector = [0.,0.,0.]``\ , \ ``localPosition = [0.,0.,0.]``\ , \ ``bodyFixed = False``\ , \ ``loadVectorUserFunction = 0``\ , \ ``show = True``\ )

- | \ *function description*\ :
  | helper function to create force applied to given body
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateForce.
- | \ *input*\ :
  | \ ``name``\ : name string for object
  | \ ``bodyNumber``\ : body number (ObjectIndex) at which the force is applied to
  | \ ``loadVector``\ : force vector (as 3D list or numpy array)
  | \ ``localPosition``\ : local position (as 3D list or numpy array) where force is applied
  | \ ``bodyFixed``\ : if True, the force is corotated with the body; else, the force is global
  | \ ``loadVectorUserFunction``\ : A Python function f(mbs, t, load)->loadVector which defines the time-dependent load and replaces loadVector in every time step; the arg load is the static loadVector
  | \ ``show``\ : if True, load is drawn
- | \ *output*\ :
  | (type: LoadIndex) returns load index
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0=mbs.CreateMassPoint(referencePosition = [0,0,0],
                         initialVelocity = [2,5,0],
                         physicsMass = 1, gravity = [0,-9.81,0],
                         drawSize = 0.5, color=exu.graphics.color.blue)
  f0=mbs.CreateForce(bodyNumber=b0, loadVector=[100,0,0],
                     localPosition=[0,0,0])
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ballBearningModel.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ballBearningModel.py>`_\  (Ex), \ `cartesianSpringDamper.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamper.py>`_\  (Ex), \ `cartesianSpringDamperUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamperUserFunction.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Ex), \ `ballBearingTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ballBearingTest.py>`_\  (TM), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TM), \ `loadUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/loadUserFunctionTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createtorque:

Function: CreateTorque
^^^^^^^^^^^^^^^^^^^^^^
`CreateTorque <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/mainSystemExtensions.py\#L3026>`__\ (\ ``name = ''``\ , \ ``bodyNumber = None``\ , \ ``loadVector = [0.,0.,0.]``\ , \ ``localPosition = [0.,0.,0.]``\ , \ ``bodyFixed = False``\ , \ ``loadVectorUserFunction = 0``\ , \ ``show = True``\ )

- | \ *function description*\ :
  | helper function to create torque applied to given body
  | - NOTE that this function is added to MainSystem via Python function MainSystemCreateTorque.
- | \ *input*\ :
  | \ ``name``\ : name string for object
  | \ ``bodyNumber``\ : body number (ObjectIndex) at which the torque is applied to
  | \ ``loadVector``\ : torque vector (as 3D list or numpy array)
  | \ ``localPosition``\ : local position (as 3D list or numpy array) where torque is applied
  | \ ``bodyFixed``\ : if True, the torque is corotated with the body; else, the torque is global
  | \ ``loadVectorUserFunction``\ : A Python function f(mbs, t, load)->loadVector which defines the time-dependent load and replaces loadVector in every time step; the arg load is the static loadVector
  | \ ``show``\ : if True, load is drawn
- | \ *output*\ :
  | (type: LoadIndex) returns load index
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [1,3,0],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                        color=exu.graphics.color.red)])
  f0=mbs.CreateTorque(bodyNumber=b0, loadVector=[0,100,0])
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ballBearningModel.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ballBearningModel.py>`_\  (Ex), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Ex), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Ex), \ `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_\  (Ex), \ `ballBearingTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ballBearingTest.py>`_\  (TM), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM), \ `pickleCopyMbs.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/pickleCopyMbs.py>`_\  (TM)

