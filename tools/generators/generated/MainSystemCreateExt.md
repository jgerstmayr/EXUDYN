
(sec-mainsystemextensions-createground)=
#### Function: CreateGround

[`CreateGround(mbs, name = '', referencePosition = None, referenceRotationMatrix = None, graphicsDataList = [], graphicsDataUserFunction = 0, show = True, referenceHT = None) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L288)

- **function description**: helper function to create a ground object, using arguments of ObjectGround; this function is mainly added for consistency with other mainSystemExtensions - NOTE that this function is added to MainSystem via Python function MainSystemCreateGround.
- **input**:
  - `mbs`: the MainSystem where items are created
  - `name`: name string for object
  - `referencePosition`: reference position of the ground (a 3D vector); None: zero
  - `referenceRotationMatrix`: reference rotation matrix of the ground (a 3D matrix); None: the unit matrix
  - `referenceHT`: referenceRotationMatrix and referencePosition at once, as homogeneous transformation: an exu.HT, a 4x4 matrix or its 16 values; None: not given; it raises if given together with referencePosition or referenceRotationMatrix
  - `graphicsDataList`: list of GraphicsData for optional ground visualization
  - `graphicsDataUserFunction`: a user function graphicsDataUserFunction(mbs, itemNumber)->BodyGraphicsData (list of GraphicsData), which can be used to draw user-defined graphics; this is much slower than regular GraphicsData
  - `color`: color of node
  - `show`: True: show ground object;
- **output**: (type: ObjectIndex) returns ground object index

*example*:

```python
  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  ground=mbs.CreateGround(referencePosition = [2,0,0],
                          graphicsDataList = [exu.graphics.CheckerBoard(point=[0,0,0], normal=[0,1,0],size=4)])
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`ANCFslidingJoint.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/ANCFslidingJoint.py) (Ex), [`ballBearningModel.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/ballBearningModel.py) (Ex), [`basicTutorial2024.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/basicTutorial2024.py) (Ex), [`bicycleIftommBenchmark.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/bicycleIftommBenchmark.py) (Ex), [`bungeeJump.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/bungeeJump.py) (Ex), [`ANCFCableBeamDampingTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/ANCFCableBeamDampingTest.py) (TM), [`ANCFThinPlateTests.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/ANCFThinPlateTests.py) (TM), [`appendSystemTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/appendSystemTest.py) (TM), ...


(sec-mainsystemextensions-createmasspoint)=
#### Function: CreateMassPoint

[`CreateMassPoint(mbs, name = '', referencePosition = [0.,0.,0.], initialDisplacement = [0.,0.,0.], initialVelocity = [0.,0.,0.], mass = 0, gravity = [0.,0.,0.], graphicsDataList = [], drawSize = -1, color = [-1.,-1.,-1.,-1.], show = True, create2D = False, returnDict = False, physicsMass = None) -> Union[dict, exudyn.ObjectIndex]`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L356)

- **function description**: helper function to create 2D or 3D mass point object and node, using arguments as in NodePoint and MassPoint - NOTE that this function is added to MainSystem via Python function MainSystemCreateMassPoint.
- **input**:
  - `mbs`: the MainSystem where items are created
  - `name`: name string for object, node is 'Node:'+name
  - `referencePosition`: reference coordinates for point node (always a 3D vector, no matter if 2D or 3D mass)
  - `initialDisplacement`: initial displacements for point node (always a 3D vector, no matter if 2D or 3D mass)
  - `initialVelocity`: initial velocities for point node (always a 3D vector, no matter if 2D or 3D mass)
  - `mass`: mass of mass point
  - `gravity`: gravity vevtor applied (always a 3D vector, no matter if 2D or 3D mass)
  - `graphicsDataList`: list of GraphicsData for optional mass visualization
  - `drawSize`: general drawing size of node
  - `color`: color of node
  - `show`: True: if graphicsData list is empty, node is shown, otherwise body is shown; False: nothing is shown
  - `create2D`: if True, create NodePoint2D and MassPoint2D
  - `returnDict`: if False, returns object index; if True, returns dict of all information on created object and node
  - `physicsMass`: deprecated name of mass
- **output**: (type: Union[dict, ObjectIndex]) returns mass point object index or dict with all data on request (if returnDict=True)

*example*:

```python
  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0=mbs.CreateMassPoint(referencePosition = [0,0,0],
                         initialVelocity = [2,5,0],
                         mass = 1, gravity = [0,-9.81,0],
                         drawSize = 0.5, color=exu.graphics.color.blue)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`basicTutorial2024.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/basicTutorial2024.py) (Ex), [`cartesianSpringDamper.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/cartesianSpringDamper.py) (Ex), [`cartesianSpringDamperUserFunction.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/cartesianSpringDamperUserFunction.py) (Ex), [`chatGPTupdate.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/chatGPTupdate.py) (Ex), [`NGsolveOCCgeometry.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/NGsolveOCCgeometry.py) (Ex), [`appendSystemTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/appendSystemTest.py) (TM), [`binarySolutionFileTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/binarySolutionFileTest.py) (TM), [`constraintDriftTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/constraintDriftTest.py) (TM), ...


(sec-mainsystemextensions-createrigidbody)=
#### Function: CreateRigidBody

[`CreateRigidBody(mbs, name = '', referencePosition = None, referenceRotationMatrix = None, initialVelocity = [0.,0.,0.], initialAngularVelocity = [0.,0.,0.], initialDisplacement = None, initialRotationMatrix = None, inertia = None, gravity = [0.,0.,0.], nodeType = exudyn.NodeType.RotationEulerParameters, graphicsDataList = [], graphicsDataUserFunction = 0, drawSize = -1, color = [-1.,-1.,-1.,-1.], show = True, create2D = False, returnDict = False, referenceHT = None, initialHT = None) -> Union[dict, exudyn.ObjectIndex]`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L487)

- **function description**: helper function to create 3D (or 2D) rigid body object and node; all quantities are global (angular velocity, etc.); use this function to easily create a rigid body; graphics can be directly obtained from inertia object, e.g. in case of cylindrical or cuboid shape - NOTE that this function is added to MainSystem via Python function MainSystemCreateRigidBody.
- **input**:
  - `mbs`: the MainSystem where items are created
  - `name`: name string for object, node is 'Node:'+name
  - `referencePosition`: reference position vector for rigid body node (always a 3D vector, no matter if 2D or 3D body); None: zero
  - `referenceRotationMatrix`: reference rotation matrix for rigid body node (always 3D matrix, no matter if 2D or 3D body); None: the unit matrix
  - `initialVelocity`: initial translational velocity vector for node (always a 3D vector, no matter if 2D or 3D body)
  - `initialAngularVelocity`: initial angular velocity vector for node (always a 3D vector, no matter if 2D or 3D body)
  - `initialDisplacement`: initial translational displacement vector for node (always a 3D vector, no matter if 2D or 3D body); these displacements are deviations from reference position, e.g. for a finite element node [None: unused]
  - `initialRotationMatrix`: initial rotation provided as matrix (always a 3D matrix, no matter if 2D or 3D body); this rotation is superimposed to reference rotation [None: unused]
  - `inertia`: an instance of class RigidBodyInertia, see rigidBodyUtilities; may also be from derived class (InertiaCuboid, InertiaMassPoint, InertiaCylinder, ...)
  - `gravity`: gravity vevtor applied (always a 3D vector, no matter if 2D or 3D mass)
  - `nodeType`: optional exudyn.NodeType to define the rotation parameterization: RotationEulerParameters, RotationRotationVector or RotationRxyz
  - `graphicsDataList`: list of GraphicsData for rigid body visualization; use exudyn.graphics functions to create GraphicsData for basic solids
  - `graphicsDataUserFunction`: a user function graphicsDataUserFunction(mbs, itemNumber)->BodyGraphicsData (list of GraphicsData), which can be used to draw user-defined graphics; this is much slower than regular GraphicsData
  - `drawSize`: general drawing size of node
  - `color`: color of node
  - `show`: True: if graphicsData list is empty, node is shown, otherwise body is shown; False: nothing is shown
  - `create2D`: if True, create NodeRigidBody2D and ObjectRigidBody2D
  - `returnDict`: if False, returns object index; if True, returns dict of all information on created object and node
  - `referenceHT`: referenceRotationMatrix and referencePosition at once, as homogeneous transformation: an exu.HT, a 4x4 matrix or its 16 values; None: not given; it raises if given together with referencePosition or referenceRotationMatrix
  - `initialHT`: initialRotationMatrix and initialDisplacement at once, the transformation added to the reference (the rotation superimposed to the reference rotation, the displacement added to the reference position); an exu.HT, a 4x4 matrix or its 16 values; None: not given; it raises if given together with initialDisplacement or initialRotationMatrix
- **output**: (type: Union[dict, ObjectIndex]) returns rigid body object index (or dict with 'nodeNumber', 'objectNumber' and possibly 'loadNumber' and 'markerBodyMass' if returnDict=True)

*example*:

```python
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
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`addPrismaticJoint.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/addPrismaticJoint.py) (Ex), [`addRevoluteJoint.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/addRevoluteJoint.py) (Ex), [`ANCFrotatingCable2D.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/ANCFrotatingCable2D.py) (Ex), [`ANCFslidingJoint.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/ANCFslidingJoint.py) (Ex), [`ballBearningModel.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/ballBearningModel.py) (Ex), [`ballBearingTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/ballBearingTest.py) (TM), [`bricardMechanism.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/bricardMechanism.py) (TM), [`capsulePrimitiveContactTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/capsulePrimitiveContactTest.py) (TM), ...


(sec-mainsystemextensions-createspringdamper)=
#### Function: CreateSpringDamper

[`CreateSpringDamper(mbs, name = '', itemNumbers = [None, None], localPosition0 = [0.,0.,0.], localPosition1 = [0.,0.,0.], referenceLength = None, stiffness = 0., damping = 0., force = 0., velocityOffset = 0., springForceUserFunction = 0, show = True, drawSize = -1, color = exudyn.graphics.color.default, bodyNumbers = None, bodyOrNodeList = None) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L738)

- **function description**: helper function to create SpringDamper connector, using arguments from ObjectConnectorSpringDamper; similar interface as CreateDistanceConstraint(...), see there for for further information - NOTE that this function is added to MainSystem via Python function MainSystemCreateSpringDamper.
- **input**:
  - `mbs`: the MainSystem where items are created
  - `name`: name string for connector; markers get Marker0:name and Marker1:name
  - `itemNumbers`: a list of two items to be connected, each a body (ObjectIndex) at localPosition0/1, a node (NodeIndex) or a marker (MarkerIndex); for a node or a marker, localPosition0/1 is [0,0,0]
  - `localPosition0`: local position (as 3D list or numpy array) on body0, if not a node of marker number
  - `localPosition1`: local position (as 3D list or numpy array) on body1, if not a node of marker number
  - `referenceLength`: if None, length is computed from reference position of bodies or nodes; if not None, this scalar reference length is used for spring
  - `stiffness`: scalar stiffness coefficient
  - `damping`: scalar damping coefficient
  - `force`: scalar additional force applied
  - `velocityOffset`: scalar offset: if referenceLength is changed over time, the velocityOffset may be changed accordingly to emulate a reference motion
  - `springForceUserFunction`: a user function springForceUserFunction(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force)->float ; this function replaces the internal connector force computation
  - `show`: if True, connector visualization is drawn
  - `drawSize`: general drawing size of connector
  - `color`: color of connector
  - `bodyNumbers`: deprecated name of itemNumbers
  - `bodyOrNodeList`: not accepted, a TypeError: the items are given in itemNumbers
- **output**: (type: ObjectIndex) returns index of newly created object

*example*:

```python
  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateMassPoint(referencePosition = [2,0,0],
                           initialVelocity = [2,5,0],
                           mass = 1, gravity = [0,-9.81,0],
                           drawSize = 0.5, color=exu.graphics.color.blue)
  oGround = mbs.AddObject(ObjectGround())
  #add vertical spring
  oSD = mbs.CreateSpringDamper(itemNumbers=[oGround, b0],
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
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`basicTutorial2024.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/basicTutorial2024.py) (Ex), [`camFollowerExample.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/camFollowerExample.py) (Ex), [`chatGPTupdate.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/chatGPTupdate.py) (Ex), [`contactCurveWithLongCurve.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/contactCurveWithLongCurve.py) (Ex), [`springMassFriction.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/springMassFriction.py) (Ex), [`appendSystemTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/appendSystemTest.py) (TM), [`binarySolutionFileTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/binarySolutionFileTest.py) (TM), [`computeItemTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/computeItemTest.py) (TM), ...


(sec-mainsystemextensions-createcartesianspringdamper)=
#### Function: CreateCartesianSpringDamper

[`CreateCartesianSpringDamper(mbs, name = '', itemNumbers = [None, None], localPosition0 = [0.,0.,0.], localPosition1 = [0.,0.,0.], stiffness = [0.,0.,0.], damping = [0.,0.,0.], offset = [0.,0.,0.], springForceUserFunction = 0, show = True, drawSize = -1, color = exudyn.graphics.color.default, bodyNumbers = None, bodyOrNodeList = None) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L852)

- **function description**: helper function to create CartesianSpringDamper connector, using arguments from ObjectConnectorCartesianSpringDamper - NOTE that this function is added to MainSystem via Python function MainSystemCreateCartesianSpringDamper.
- **input**:
  - `mbs`: the MainSystem where items are created
  - `name`: name string for connector; markers get Marker0:name and Marker1:name
  - `itemNumbers`: a list of two items to be connected, each a body (ObjectIndex) at localPosition0/1, a node (NodeIndex) or a marker (MarkerIndex); for a node or a marker, localPosition0/1 is [0,0,0]
  - `localPosition0`: local position (as 3D list or numpy array) on body0, if not a node of marker number
  - `localPosition1`: local position (as 3D list or numpy array) on body1, if not a node of marker number
  - `stiffness`: stiffness coefficients (as 3D list or numpy array)
  - `damping`: damping coefficients (as 3D list or numpy array)
  - `offset`: offset vector (as 3D list or numpy array)
  - `springForceUserFunction`: a user function springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset)->[float,float,float] ; this function replaces the internal connector force computation
  - `show`: if True, connector visualization is drawn
  - `drawSize`: general drawing size of connector
  - `color`: color of connector
  - `bodyNumbers`: deprecated name of itemNumbers
  - `bodyOrNodeList`: not accepted, a TypeError: the items are given in itemNumbers
- **output**: (type: ObjectIndex) returns index of newly created object

*example*:

```python
  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateMassPoint(referencePosition = [7,0,0],
                            mass = 1, gravity = [0,-9.81,0],
                            drawSize = 0.5, color=exu.graphics.color.blue)
  oGround = mbs.AddObject(ObjectGround())
  oSD = mbs.CreateCartesianSpringDamper(itemNumbers=[oGround, b0],
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
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`cartesianSpringDamper.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/cartesianSpringDamper.py) (Ex), [`cartesianSpringDamperUserFunction.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/cartesianSpringDamperUserFunction.py) (Ex), [`chatGPTupdate.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/chatGPTupdate.py) (Ex), [`NGsolveFFRFSlidingJoint.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/NGsolveFFRFSlidingJoint.py) (Ex), [`complexEigenvaluesTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/complexEigenvaluesTest.py) (TM), [`computeODE2AEeigenvaluesTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/computeODE2AEeigenvaluesTest.py) (TM), [`createFunctionsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createFunctionsTest.py) (TM), [`createItemNumbersTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createItemNumbersTest.py) (TM), ...


(sec-mainsystemextensions-createrigidbodyspringdamper)=
#### Function: CreateRigidBodySpringDamper

[`CreateRigidBodySpringDamper(mbs, name = '', itemNumbers = [None, None], localPosition0 = [0.,0.,0.], localPosition1 = [0.,0.,0.], stiffness = np.zeros((6,6)), damping = np.zeros((6,6)), offset = [0.,0.,0.,0.,0.,0.], rotationMatrixJoint = np.eye(3), useGlobalFrame = True, useIntrinsicFormulation = True, springForceTorqueUserFunction = 0, postNewtonStepUserFunction = 0, show = True, drawSize = -1, color = exudyn.graphics.color.default, intrinsicFormulation = None, bodyNumbers = None, bodyOrNodeList = None) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L948)

- **function description**: helper function to create RigidBodySpringDamper connector, using arguments from ObjectConnectorRigidBodySpringDamper, see there for the full documentation - NOTE that this function is added to MainSystem via Python function MainSystemCreateRigidBodySpringDamper.
- **input**:
  - `mbs`: the MainSystem where items are created
  - `name`: name string for connector; markers get Marker0:name and Marker1:name
  - `itemNumbers`: a list of two items to be connected, each a body (ObjectIndex) at localPosition0/1, a rigid node (NodeIndex) or a rigid marker (MarkerIndex); for a node or a marker, localPosition0/1 is [0,0,0]
  - `localPosition0`: local position (as 3D list or numpy array) on body0, if not a node of marker number
  - `localPosition1`: local position (as 3D list or numpy array) on body1, if not a node of marker number
  - `stiffness`: stiffness coefficients (as 6D matrix or numpy array)
  - `damping`: damping coefficients (as 6D matrix or numpy array)
  - `offset`: offset vector (as 6D list or numpy array)
  - `rotationMatrixJoint`: additional rotation matrix; in case  useGlobalFrame=False, it transforms body0/node0 local frame to joint frame; if useGlobalFrame=True, it transforms global frame to joint frame
  - `useGlobalFrame`: if False, the rotationMatrixJoint is defined in the local coordinate system of body0
  - `useIntrinsicFormulation`: if True, uses intrinsic formulation of Maserati and Morandini, which uses matrix logarithm and is independent of order of markers (preferred formulation); otherwise, Tait-Bryan angles are used for computation of torque, see documentation
  - `springForceTorqueUserFunction`: a user function springForceTorqueUserFunction(mbs, t, itemNumber, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1, offset)->[float,float,float, float,float,float] ; this function replaces the internal connector force / torque computation
  - `postNewtonStepUserFunction`: a special user function postNewtonStepUserFunction(mbs, t, Index itemIndex, dataCoordinates, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1, offset)->[PNerror, recommendedStepSize, data[0], data[1], ...] ; for details, see RigidBodySpringDamper for full docu
  - `show`: if True, connector visualization is drawn
  - `drawSize`: general drawing size of connector
  - `color`: color of connector
  - `intrinsicFormulation`: deprecated name of useIntrinsicFormulation
  - `bodyNumbers`: deprecated name of itemNumbers
  - `bodyOrNodeList`: not accepted, a TypeError: the items are given in itemNumbers
- **output**: (type: ObjectIndex) returns index of newly created object

*example*:

```python
  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [8,0,0],
                           initialAngularVelocity = [0,1,2],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                    color=exu.graphics.color.green)])
  oGround = mbs.AddObject(ObjectGround())
  #a bushing at the left end of the bar
  mbs.CreateRigidBodySpringDamper(itemNumbers=[oGround, b0],
                                  localPosition0=[7.5,0,0],
                                  localPosition1=[-0.5,0,0],
                                  stiffness=np.diag([1e4,1e4,1e4,1e3,1e3,1e3]),
                                  damping=np.diag([10,10,10,1,1,1]))
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`bricardMechanism.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/bricardMechanism.py) (TM), [`connectorFrameOutputsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/connectorFrameOutputsTest.py) (TM), [`createItemNumbersTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createItemNumbersTest.py) (TM), [`modelLibrary.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/modelLibrary.py) (TM), [`pythonUserFunctionArgumentsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/pythonUserFunctionArgumentsTest.py) (TM), [`rigidBodySpringDamperIntrinsic.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/rigidBodySpringDamperIntrinsic.py) (TM), [`symbolicUserConnectorsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/symbolicUserConnectorsTest.py) (TM)


(sec-mainsystemextensions-createtorsionalspringdamper)=
#### Function: CreateTorsionalSpringDamper

[`CreateTorsionalSpringDamper(mbs, name = '', itemNumbers = [None, None], position = [0.,0.,0.], axis = [0.,0.,0.], stiffness = 0., damping = 0., offset = 0., velocityOffset = 0., torque = 0., useGlobalFrame = True, springTorqueUserFunction = 0, unlimitedRotations = True, show = True, drawSize = -1, color = exudyn.graphics.color.default, bodyNumbers = None, bodyOrNodeList = None) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L1093)

- **function description**: helper function to create TorsionalSpringDamper connector, using arguments from ObjectConnectorTorsionalSpringDamper, see there for the full documentation - NOTE that this function is added to MainSystem via Python function MainSystemCreateTorsionalSpringDamper.
- **input**:
  - `mbs`: the MainSystem where items are created
  - `name`: name string for connector; markers get Marker0:name and Marker1:name
  - `itemNumbers`: a list of two items to be connected, each a rigid body or a ground object (ObjectIndex) or a rigid marker (MarkerIndex); with a marker, position is [], and the marker sets the position of the spring-damper
  - `position`: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  - `axis`: a 3D vector as list or np.array containing the axis around which the spring acts, either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True)
  - `stiffness`: scalar stiffness of spring
  - `damping`: scalar damping added to spring
  - `offset`: scalar offset, which can be used to realize a P-controlled actuator
  - `velocityOffset`: scalar velocity offset, which can be used to realize a D-controlled actuator
  - `torque`: additional constant torque added to spring-damper, acting between the two bodies
  - `useGlobalFrame`: if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
  - `springTorqueUserFunction`: a user function springTorqueUserFunction(mbs, t, itemNumber, rotation, angularVelocity, stiffness, damping, offset)->float ; this function replaces the internal connector torque computation
  - `unlimitedRotations`: if True, an additional generic data node is added to enable measurement of rotations beyond +/- pi; this also allows the spring to cope with multiple turns.
  - `show`: if True, connector visualization is drawn
  - `drawSize`: general drawing size of connector
  - `color`: color of connector
  - `bodyNumbers`: deprecated name of itemNumbers
  - `bodyOrNodeList`: not accepted, a TypeError: the items are given in itemNumbers
- **output**: (type: ObjectIndex) returns index of newly created object

*example*:

```python
  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [9.5,0,0],
                           initialAngularVelocity = [0,0,2],
                           graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                    color=exu.graphics.color.steelblue)])
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreateRevoluteJoint(itemNumbers=[oGround, b0], position=[9,0,0], axis=[0,0,1],
                          useGlobalFrame=True, axisRadius=0.02, axisLength=0.14)
  #a torsional spring-damper about the axis of the revolute joint
  mbs.CreateTorsionalSpringDamper(itemNumbers=[oGround, b0], position=[9,0,0], axis=[0,0,1],
                                  stiffness=100, damping=1)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`symbolicSystemRigidBodyPendulum.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/symbolicSystemRigidBodyPendulum.py) (Ex), [`createFunctionsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createFunctionsTest.py) (TM), [`createItemNumbersTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createItemNumbersTest.py) (TM), [`symbolicSystemEquationsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/symbolicSystemEquationsTest.py) (TM), [`symbolicSystemItemJacobiansTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/symbolicSystemItemJacobiansTest.py) (TM), [`symbolicSystemVariablesTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/symbolicSystemVariablesTest.py) (TM), [`symbolicUserItemsDrawingTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/symbolicUserItemsDrawingTest.py) (TM), [`symbolicUserRigidBodiesTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/symbolicUserRigidBodiesTest.py) (TM), ...


(sec-mainsystemextensions-createlinearspringdamper)=
#### Function: CreateLinearSpringDamper

[`CreateLinearSpringDamper(mbs, name = '', itemNumbers = [None, None], position = [0.,0.,0.], axis = [1.,0.,0.], stiffness = 0., damping = 0., offset = 0., velocityOffset = 0., force = 0., useGlobalFrame = True, springForceUserFunction = 0, show = True, drawSize = -1, color = exudyn.graphics.color.default) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L1264)

- **function description**: helper function to create a LinearSpringDamper connector, acting along an axis between two rigid bodies, using arguments from ObjectConnectorLinearSpringDamper, see there for the full documentation; usually used together with a prismatic joint - NOTE that this function is added to MainSystem via Python function MainSystemCreateLinearSpringDamper.
- **input**:
  - `mbs`: the MainSystem where items are created
  - `name`: name string for connector; markers get Marker0:name and Marker1:name
  - `itemNumbers`: a list of two items to be connected, each a rigid body or a ground object (ObjectIndex) or a rigid marker (MarkerIndex); with a marker, position is [], and the marker sets the position of the spring-damper
  - `position`: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the spring-damper in reference configuration; else: local position in body0
  - `axis`: a 3D vector as list or np.array containing the axis along which the spring acts, either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True); it is normalized
  - `stiffness`: scalar stiffness of spring
  - `damping`: scalar damping added to spring
  - `offset`: scalar offset of the displacement along the axis, at which the spring force is zero
  - `velocityOffset`: scalar velocity offset, which can be used to realize a D-controlled actuator
  - `force`: additional constant force added to spring-damper, acting between the two bodies along the axis
  - `useGlobalFrame`: if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
  - `springForceUserFunction`: a user function springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset)->float ; this function replaces the internal connector force computation
  - `show`: if True, connector visualization is drawn
  - `drawSize`: general drawing size of connector
  - `color`: color of connector
- **output**: (type: ObjectIndex) returns index of newly created object

*example*:

```python
  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  oGround = mbs.CreateGround()
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=1000, sideLengths=[0.2,0.1,0.1]),
                           referencePosition = [1,0,0], gravity = [-9.81,0,0])
  mbs.CreatePrismaticJoint(itemNumbers=[oGround, b0], position=[1,0,0], axis=[1,0,0])
  mbs.CreateLinearSpringDamper(itemNumbers=[oGround, b0], position=[1,0,0], axis=[1,0,0],
                               stiffness=1e3, damping=10)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`connectorFrameOutputsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/connectorFrameOutputsTest.py) (TM), [`createLinearSpringDamperTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createLinearSpringDamperTest.py) (TM)


(sec-mainsystemextensions-createrevolutejoint)=
#### Function: CreateRevoluteJoint

[`CreateRevoluteJoint(mbs, name = '', itemNumbers = [None, None], position = [], axis = [], useGlobalFrame = True, show = True, axisRadius = 0.1, axisLength = 0.4, color = exudyn.graphics.color.default, bodyNumbers = None, bodyOrNodeList = None) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L1373)

- **function description**: Create revolute joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed - NOTE that this function is added to MainSystem via Python function MainSystemCreateRevoluteJoint.
- **input**:
  - `mbs`: the MainSystem where joint and markers shall be created
  - `name`: name string for joint; markers get Marker0:name and Marker1:name
  - `itemNumbers`: a list of two items to be connected, each a rigid body or a ground object (ObjectIndex) or a rigid marker (MarkerIndex); with a marker, position is [], and the marker sets the position of the joint
  - `position`: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  - `axis`: a 3D vector as list or np.array containing the joint axis either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True)
  - `useGlobalFrame`: if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
  - `show`: if True, connector visualization is drawn
  - `axisRadius`: radius of axis for connector graphical representation
  - `axisLength`: length of axis for connector graphical representation
  - `color`: color of connector
  - `bodyNumbers`: deprecated name of itemNumbers
  - `bodyOrNodeList`: not accepted, a TypeError: the items are given in itemNumbers
- **output**: (type: ObjectIndex) returns index of created joint

*example*:

```python
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
  mbs.CreateRevoluteJoint(itemNumbers=[oGround, b0], position=[2.5,0,0], axis=[0,0,1],
                          useGlobalFrame=True, axisRadius=0.02, axisLength=0.14)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`addRevoluteJoint.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/addRevoluteJoint.py) (Ex), [`bicycleIftommBenchmark.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/bicycleIftommBenchmark.py) (Ex), [`chatGPTupdate.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/chatGPTupdate.py) (Ex), [`chatGPTupdate2.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/chatGPTupdate2.py) (Ex), [`involuteGearGraphics.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/involuteGearGraphics.py) (Ex), [`bricardMechanism.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/bricardMechanism.py) (TM), [`computeItemTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/computeItemTest.py) (TM), [`connectorFrameOutputsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/connectorFrameOutputsTest.py) (TM), ...


(sec-mainsystemextensions-createprismaticjoint)=
#### Function: CreatePrismaticJoint

[`CreatePrismaticJoint(mbs, name = '', itemNumbers = [None, None], position = [], axis = [], useGlobalFrame = True, show = True, axisRadius = 0.1, axisLength = 0.4, color = exudyn.graphics.color.default, bodyNumbers = None, bodyOrNodeList = None) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L1483)

- **function description**: Create prismatic joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed - NOTE that this function is added to MainSystem via Python function MainSystemCreatePrismaticJoint.
- **input**:
  - `mbs`: the MainSystem where joint and markers shall be created
  - `name`: name string for joint; markers get Marker0:name and Marker1:name
  - `itemNumbers`: a list of two items to be connected, each a rigid body or a ground object (ObjectIndex) or a rigid marker (MarkerIndex); with a marker, position is [], and the marker sets the position of the joint
  - `position`: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  - `axis`: a 3D vector as list or np.array containing the joint axis either in local body0 coordinates (useGlobalFrame=False), or in global reference configuration (useGlobalFrame=True)
  - `useGlobalFrame`: if False, the position and axis vectors are defined in the local coordinate system of body0, otherwise in global (reference) coordinates
  - `show`: if True, connector visualization is drawn
  - `axisRadius`: radius of axis for connector graphical representation
  - `axisLength`: length of axis for connector graphical representation
  - `color`: color of connector
  - `bodyNumbers`: deprecated name of itemNumbers
  - `bodyOrNodeList`: not accepted, a TypeError: the items are given in itemNumbers
- **output**: (type: ObjectIndex) returns index of created joint

*example*:

```python
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
  mbs.CreatePrismaticJoint(itemNumbers=[oGround, b0], position=[3.5,0,0], axis=[0,1,0],
                           useGlobalFrame=True, axisRadius=0.02, axisLength=1)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`addPrismaticJoint.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/addPrismaticJoint.py) (Ex), [`chatGPTupdate.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/chatGPTupdate.py) (Ex), [`chatGPTupdate2.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/chatGPTupdate2.py) (Ex), [`involuteGearGraphics.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/involuteGearGraphics.py) (Ex), [`connectorFrameOutputsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/connectorFrameOutputsTest.py) (TM), [`createFunctionsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createFunctionsTest.py) (TM), [`createItemNumbersTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createItemNumbersTest.py) (TM), [`createLinearSpringDamperTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createLinearSpringDamperTest.py) (TM), ...


(sec-mainsystemextensions-createsphericaljoint)=
#### Function: CreateSphericalJoint

[`CreateSphericalJoint(mbs, name = '', itemNumbers = [None, None], position = [], constrainedAxes = [1,1,1], useGlobalFrame = True, show = True, jointRadius = 0.1, color = exudyn.graphics.color.default, bodyNumbers = None, bodyOrNodeList = None) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L1587)

- **function description**: Create spherical joint between two bodies; definition of joint position in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers are automatically computed - NOTE that this function is added to MainSystem via Python function MainSystemCreateSphericalJoint.
- **input**:
  - `mbs`: the MainSystem where joint and markers shall be created
  - `name`: name string for joint; markers get Marker0:name and Marker1:name
  - `itemNumbers`: a list of two items to be connected, each a mass point, a rigid body or a ground object (ObjectIndex) or a position marker (MarkerIndex); with a marker, position is [], and the marker sets the position of the joint
  - `position`: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  - `constrainedAxes`: flags, which determines which (global) translation axes are constrained; each entry may only be 0 (=free) axis or 1 (=constrained axis)
  - `useGlobalFrame`: if False, the point and axis vectors are defined in the local coordinate system of body0
  - `show`: if True, connector visualization is drawn
  - `jointRadius`: radius of sphere for connector graphical representation
  - `color`: color of connector
  - `bodyNumbers`: deprecated name of itemNumbers
  - `bodyOrNodeList`: not accepted, a TypeError: the items are given in itemNumbers
- **output**: (type: ObjectIndex) returns index of created joint

*example*:

```python
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
  mbs.CreateSphericalJoint(itemNumbers=[oGround, b0], position=[5.5,0,0],
                           useGlobalFrame=True, jointRadius=0.06)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`newtonsCradle.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/newtonsCradle.py) (Ex), [`NGsolveCreateFFRFreducedOrder.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/NGsolveCreateFFRFreducedOrder.py) (Ex), [`solutionViewerPreStepUserFunction.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/solutionViewerPreStepUserFunction.py) (Ex), [`ANCFThinPlateTests.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/ANCFThinPlateTests.py) (TM), [`createFunctionsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createFunctionsTest.py) (TM), [`createItemNumbersTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createItemNumbersTest.py) (TM), [`driveTrainTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/driveTrainTest.py) (TM), [`initialAccelerationsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/initialAccelerationsTest.py) (TM), ...


(sec-mainsystemextensions-creategenericjoint)=
#### Function: CreateGenericJoint

[`CreateGenericJoint(mbs, name = '', itemNumbers = [None, None], position = [], rotationMatrixAxes = np.eye(3), constrainedAxes = [1,1,1, 1,1,1], useGlobalFrame = True, offsetUserFunction = 0, offsetUserFunction_t = 0, show = True, axesRadius = 0.1, axesLength = 0.4, color = exudyn.graphics.color.default, bodyNumbers = None, bodyOrNodeList = None) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L1671)

- **function description**: Create generic joint between two bodies; definition of joint position (position) and axes (rotationMatrixAxes) in global coordinates (useGlobalFrame=True) or in local coordinates of body0 (useGlobalFrame=False), where rotationMatrixAxes is an additional rotation to body0; all markers, markerRotation and other quantities are automatically computed - NOTE that this function is added to MainSystem via Python function MainSystemCreateGenericJoint.
- **input**:
  - `mbs`: the MainSystem where joint and markers shall be created
  - `name`: name string for joint; markers get Marker0:name and Marker1:name
  - `itemNumbers`: a list of two items to be connected, each a rigid body or a ground object (ObjectIndex) or a rigid marker (MarkerIndex); with a marker, position is [], and the marker sets the position of the joint
  - `position`: a 3D vector as list or np.array: if useGlobalFrame=True it describes the global position of the joint in reference configuration; else: local position in body0
  - `rotationMatrixAxes`: rotation matrix which defines orientation of constrainedAxes; if useGlobalFrame, this rotation matrix is global, else the rotation matrix is post-multiplied with the rotation of body0, identical with rotationMarker0 in the joint
  - `constrainedAxes`: flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; each entry may only be 0 (=free) axis or 1 (=constrained axis); ALL constrained Axes are defined relative to reference rotation of body0 times rotation0
  - `useGlobalFrame`: if False, the position is defined in the local coordinate system of body0, otherwise it is defined in global coordinates
  - `offsetUserFunction`: a user function offsetUserFunction(mbs, t, itemNumber, offsetUserFunctionParameters)->float ; this function replaces the internal (constant) by a user-defined offset. This allows to realize rheonomic joints and allows kinematic simulation
  - `offsetUserFunction_t`: a user function offsetUserFunction_t(mbs, t, itemNumber, offsetUserFunctionParameters)->float ; this function replaces the internal (constant) by a user-defined offset velocity; this function is used instead of offsetUserFunction, if velocityLevel (index2) time integration
  - `show`: if True, connector visualization is drawn
  - `axesRadius`: radius of axes for connector graphical representation
  - `axesLength`: length of axes for connector graphical representation
  - `color`: color of connector
  - `bodyNumbers`: deprecated name of itemNumbers
  - `bodyOrNodeList`: not accepted, a TypeError: the items are given in itemNumbers
- **output**: (type: ObjectIndex) returns index of created joint

*example*:

```python
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
  mbs.CreateGenericJoint(itemNumbers=[oGround, b0], position=[5.5,0,0],
                         constrainedAxes=[1,1,1, 1,0,0],
                         rotationMatrixAxes=RotationMatrixX(0.125*pi), #tilt axes
                         useGlobalFrame=True, axesRadius=0.02, axesLength=0.2)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`bungeeJump.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/bungeeJump.py) (Ex), [`homogeneousTransformationInterpolation.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/homogeneousTransformationInterpolation.py) (Ex), [`NGsolveFFRFSlidingJoint.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/NGsolveFFRFSlidingJoint.py) (Ex), [`pistonEngine.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/pistonEngine.py) (Ex), [`universalJoint.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/universalJoint.py) (Ex), [`ANCFCableBeamDampingTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/ANCFCableBeamDampingTest.py) (TM), [`ANCFThinPlateRotationTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/ANCFThinPlateRotationTest.py) (TM), [`ANCFThinPlateTests.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/ANCFThinPlateTests.py) (TM), ...


(sec-mainsystemextensions-createdistanceconstraint)=
#### Function: CreateDistanceConstraint

[`CreateDistanceConstraint(mbs, name = '', itemNumbers = [None, None], localPosition0 = [0.,0.,0.], localPosition1 = [0.,0.,0.], distance = None, show = True, drawSize = -1., color = exudyn.graphics.color.default, bodyNumbers = None, bodyOrNodeList = None) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L1786)

- **function description**: Create distance joint between two bodies; definition of joint positions in local coordinates of bodies or nodes; if distance=None, it is computed automatically from reference length; all markers are automatically computed - NOTE that this function is added to MainSystem via Python function MainSystemCreateDistanceConstraint.
- **input**:
  - `mbs`: the MainSystem where joint and markers shall be created
  - `name`: name string for joint; markers get Marker0:name and Marker1:name
  - `itemNumbers`: a list of two items to be connected, each a body (ObjectIndex) at localPosition0/1, a node (NodeIndex) or a marker (MarkerIndex); for a node or a marker, localPosition0/1 is [0,0,0]
  - `localPosition0`: local position (as 3D list or numpy array) on body0, if not a node or marker number
  - `localPosition1`: local position (as 3D list or numpy array) on body1, if not a node or marker number
  - `distance`: if None, distance is computed from reference position of bodies or nodes; if not None, this distance is prescribed between the two positions; if distance = 0, it will create a SphericalJoint as this case is not possible with a DistanceConstraint
  - `show`: if True, connector visualization is drawn
  - `drawSize`: general drawing size of node
  - `color`: color of connector
  - `bodyNumbers`: deprecated name of itemNumbers
  - `bodyOrNodeList`: not accepted, a TypeError: the items are given in itemNumbers
- **output**: (type: ObjectIndex) returns index of created joint

*example*:

```python
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
                           mass=1, drawSize = 0.2)
  n1 = mbs.GetObject(m1)['nodeNumber']
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreateDistanceConstraint(itemNumbers=[oGround, b0],
                               localPosition0 = [6.5,1,0],
                               localPosition1 = [0.5,0,0],
                               distance=None, #automatically computed
                               drawSize=0.06)
  mbs.CreateDistanceConstraint(itemNumbers=[b0, n1],
                               localPosition0 = [-0.5,0,0],
                               localPosition1 = [0.,0.,0.], #must be [0,0,0] for Node
                               distance=None, #automatically computed
                               drawSize=0.06)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`chatGPTupdate.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/chatGPTupdate.py) (Ex), [`chatGPTupdate2.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/chatGPTupdate2.py) (Ex), [`newtonsCradle.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/newtonsCradle.py) (Ex), [`scopeExample.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/scopeExample.py) (Ex), [`constraintDriftTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/constraintDriftTest.py) (TM), [`createFunctionsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createFunctionsTest.py) (TM), [`createItemNumbersTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createItemNumbersTest.py) (TM), [`deleteItemsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/deleteItemsTest.py) (TM), ...


(sec-mainsystemextensions-createcoordinateconstraint)=
#### Function: CreateCoordinateConstraint

[`CreateCoordinateConstraint(mbs, name = '', itemNumbers = [None, None], coordinates = [None, None], offset = 0., factor1 = 1., velocityLevel = False, offsetUserFunction = 0, offsetUserFunction_t = 0, show = True, drawSize = -1., color = exudyn.graphics.color.default, includeReferenceCoordinates = [False, False], factorValue1 = None, bodyNumbers = None, bodyOrNodeList = None) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L1894)

- **function description**: Create coordinate constraint between two bodies or nodes, or one of them and the ground; markers and NodePointGround are automatically created when needed - NOTE that this function is added to MainSystem via Python function MainSystemCreateCoordinateConstraint.
- **input**:
  - `mbs`: the MainSystem where joint and markers shall be created
  - `name`: name string for joint; markers get Marker0:name and Marker1:name
  - `itemNumbers`: a list of the two items whose coordinates are constrained, each a body (ObjectIndex), whose coordinates are those of its nodes in their order, a node (NodeIndex), or None or a ground object for the ground
  - `coordinates`: a list of the two coordinate indices: of the node, or of the body, counted over its nodes in their order; None for the ground
  - `offset`: an fixed offset between the two coordinate values
  - `factor1`: an additional factor multiplied with coordinate value1 used in algebraic equation, to enable (e.g. gear) ratio between coordinates
  - `velocityLevel`: If true: connector constrains velocities (only works for ODE2 coordinates!); offset is used between velocities; if True, the offsetUserFunction_t is considered and offsetUserFunction is ignored
  - `offsetUserFunction`: a Python function which defines the time-dependent offset; see description in CoordinateConstraint
  - `offsetUserFunction_t`: time derivative of offsetUserFunction; needed for velocity level constraints; see description in CoordinateConstraint
  - `show`: if True, connector visualization is drawn
  - `drawSize`: general drawing size of node
  - `color`: color of connector
  - `includeReferenceCoordinates`: a list of two bools, one per item: True constrains the coordinate including its reference value, e.g. the absolute position instead of the displacement (MarkerNodeCoordinate, #990)
  - `factorValue1`: deprecated name of factor1
  - `bodyNumbers`: deprecated name of itemNumbers
  - `bodyOrNodeList`: not accepted, a TypeError: the items are given in itemNumbers
- **output**: (type: ObjectIndex) returns index of created joint

*example*:

```python
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
                           mass=1, drawSize = 0.2)
  mbs.CreateCoordinateConstraint(itemNumbers=[None, b0],
                                 coordinates=[None, 0]) #constrains X-coordinate
  #constrain Y-coordinate of b0 to Z-coordinate of m1:
  mbs.CreateCoordinateConstraint(itemNumbers=[b0, m1],
                                 coordinates=[1, 2])
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`ballBearningModel.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/ballBearningModel.py) (Ex), [`camFollowerExample.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/camFollowerExample.py) (Ex), [`involuteGearGraphics.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/involuteGearGraphics.py) (Ex), [`ballBearingTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/ballBearingTest.py) (TM), [`contactCurveExample.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/contactCurveExample.py) (TM), [`createFunctionsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createFunctionsTest.py) (TM), [`createItemNumbersTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createItemNumbersTest.py) (TM), [`markerNodeCoordinateReferenceTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/markerNodeCoordinateReferenceTest.py) (TM), ...


(sec-mainsystemextensions-createrollingdisc)=
#### Function: CreateRollingDisc

[`CreateRollingDisc(mbs, name = '', bodyNumbers = [None, None], axisPosition = [], axisVector = [1,0,0], discRadius = 0., planePosition = [0,0,0], planeNormal = [0,0,1], constrainedAxes = [1,1,1], activeConnector = True, show = True, discWidth = 0.1, color = exudyn.graphics.color.default) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L2032)

- **function description**: Create an ideal rolling disc joint between wheel rigid body and ground; the disc is infinitely thin and the ground is a perfectly flat plane; the wheel may lift off; definition of joint position and axis in global coordinates (alternatively in wheel (body1) local coordinates) for reference configuration of bodies; all markers and other quantities are automatically computed; some constraint conditions may be deactivated, e.g. to resolve redundancy of constraints for multi-wheel vehicles - NOTE that this function is added to MainSystem via Python function MainSystemCreateRollingDisc.
- **input**:
  - `mbs`: the MainSystem where joint and markers shall be created
  - `name`: name string for joint; markers get Marker0:name and Marker1:name
  - `bodyNumbers`: a list of object numbers for body0=ground and body1=wheel; must be rigid body or ground object
  - `axisPosition`: a 3D vector as list or np.array: position of wheel axis in local body1=wheel coordinates
  - `axisVector`: a 3D vector as list or np.array containing the joint (=wheel) axis in local body1=wheel coordinates
  - `discRadius`: radius of the disc
  - `planePosition`: any 3D position vector of plane in ground object; given as local coordinates in ground object
  - `planeNormal`: 3D normal vector of the rolling (contact) plane on ground; given as local coordinates in ground object
  - `constrainedAxes`: [j0,j1,j2] flags, which determine which constraints are active, in which j0 represents the constraint for lateral motion, j1 longitudinal (forward/backward) motion and j2 represents the normal (contact) direction
  - `activeConnector`: flag to activate or deactivate the joint
  - `show`: if True, connector visualization is drawn
  - `discWidth`: disc with, only used for drawing
  - `color`: color of connector
- **output**: (type: ObjectIndex) returns index of created joint

*example*:

```python
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
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`createFunctionsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createFunctionsTest.py) (TM), [`createRollingDiscTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createRollingDiscTest.py) (TM)


(sec-mainsystemextensions-createrollingdiscpenalty)=
#### Function: CreateRollingDiscPenalty

[`CreateRollingDiscPenalty(mbs, name = '', bodyNumbers = [None, None], axisPosition = [], axisVector = [1,0,0], discRadius = 0., planePosition = [0,0,0], planeNormal = [0,0,1], contactStiffness = 0., contactDamping = 0., dryFriction = [0,0], dryFrictionAngle = 0., dryFrictionProportionalZone = 0., viscousFriction = [0,0], rollingViscousFriction = 0., useLinearProportionalZone = False, activeConnector = True, show = True, discWidth = 0.1, color = exudyn.graphics.color.default, rollingFrictionViscous = None) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L2139)

- **function description**: Create penalty-based rolling disc joint between wheel rigid body and ground; the disc is infinitely thin and the ground is a perfectly flat plane; the wheel may lift off; definition of joint position and axis in global coordinates (alternatively in wheel (body1) local coordinates) for reference configuration of bodies; all markers and other quantities are automatically computed - NOTE that this function is added to MainSystem via Python function MainSystemCreateRollingDiscPenalty.
- **input**:
  - `mbs`: the MainSystem where joint and markers shall be created
  - `name`: name string for joint; markers get Marker0:name and Marker1:name
  - `bodyNumbers`: a list of object numbers for body0=ground and body1=wheel; must be rigid body or ground object
  - `axisPosition`: a 3D vector as list or np.array: position of wheel axis in local body1=wheel coordinates
  - `axisVector`: a 3D vector as list or np.array containing the joint (=wheel) axis in local body1=wheel coordinates
  - `discRadius`: radius of the disc
  - `planePosition`: any 3D position vector of plane in ground object; given as local coordinates in ground object
  - `planeNormal`: 3D normal vector of the rolling (contact) plane on ground; given as local coordinates in ground object
  - `dryFrictionAngle`: angle (radiant) which defines a rotation of the local tangential coordinates dry friction; this allows to model Mecanum wheels with specified roll angle
  - `contactStiffness`: normal contact stiffness
  - `contactDamping`: normal contact damping
  - `dryFriction`: 2D list of friction parameters; dry friction coefficients in local wheel coordinates, where for dryFrictionAngle=0, the first parameter refers to forward direction and the second parameter to lateral direction
  - `viscousFriction`: 2D list of viscous friction coefficients [SI:1/(m/s)] in local wheel coordinates; proportional to slipping velocity, leading to increasing slipping friction force for increasing slipping velocity; directions are same as in dryFriction
  - `dryFrictionProportionalZone`: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations)
  - `rollingViscousFriction`: viscous rolling friction [SI:s/m]: the force acts against the velocity of the trail on ground and is proportional to this velocity and to the contact normal force;
  - `useLinearProportionalZone`: if True, a linear proportional zone is used; the linear zone performs better in implicit time integration as the Jacobian has a constant tangent in the sticking case
  - `activeConnector`: flag to activate or deactivate the connector
  - `show`: if True, connector visualization is drawn
  - `discWidth`: disc with, only used for drawing
  - `color`: color of connector
  - `rollingFrictionViscous`: deprecated name of rollingViscousFriction
- **output**: (type: ObjectIndex) returns index of created joint

*example*:

```python
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
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`createFunctionsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createFunctionsTest.py) (TM), [`createRollingDiscPenaltyTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createRollingDiscPenaltyTest.py) (TM)


(sec-mainsystemextensions-createspherespherecontact)=
#### Function: CreateSphereSphereContact

[`CreateSphereSphereContact(mbs, name = '', itemNumbers = [None, None], localPosition0 = [0.,0.,0.], localPosition1 = [0.,0.,0.], spheresRadii = [-1,-1], isHollowSphere1 = False, dynamicFriction = 0., frictionProportionalZone = 1e-3, contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1, constantPullOffForce = 0, contactPlasticityRatio = 0, adhesionCoefficient = 0, adhesionExponent = 1, restitutionCoefficient = 1, minimumImpactVelocity = 0, impactModel = exu.ContactImpactModel.Linear, dataInitialCoordinates = [0,0,0,0], activeConnector = True, show = False, color = exudyn.graphics.color.default, bodyNumbers = None, bodyOrNodeList = None) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L2275)

- **function description**: Create penalty-based sphere-sphere contact between two rigid bodies, mass points (if friction coefficient is zero) or according nodes; the contact is based on ObjectContactSphereSphere; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems - NOTE that this function is added to MainSystem via Python function MainSystemCreateSphereSphereContact.
- **input**:
  - `mbs`: the MainSystem where joint and markers shall be created
  - `name`: name string for joint; markers get Marker0:name and Marker1:name
  - `itemNumbers`: a list of the two items the spheres are attached to, each a body (ObjectIndex) at localPosition0/1, a node (NodeIndex) or a marker (MarkerIndex); for a node or a marker, localPosition0/1 is [0,0,0]; with friction, a rigid body, node or marker; for a mass point, friction due to rolling is not accounted for
  - `localPosition0`: local position (as 3D list or numpy array) of sphere0 on body0, if not a node number
  - `localPosition1`: local position (as 3D list or numpy array) of sphere1 on body1, if not a node number
  - `spheresRadii`: list containing radius of sphere 0 and radius of sphere 1 [SI:m].
  - `isHollowSphere1`: flag, which determines, if sphere attached to marker 1 (radius 1) is a hollow sphere.
  - `dynamicFriction`: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, Section Module: physics
  - `frictionProportionalZone`: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), Section Module: physics
  - `contactStiffness`: normal contact stiffness
  - `contactDamping`: linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
  - `contactStiffnessExponent`: exponent in normal contact model [SI:1]
  - `constantPullOffForce`: constant adhesion force [SI:N]; Edinburgh Adhesive Elasto-Plastic Model
  - `contactPlasticityRatio`: ratio of contact stiffness for first loading and unloading/reloading [SI:1]; Edinburgh Adhesive Elasto-Plastic Model; see ObjectContactSphereSphere
  - `adhesionCoefficient`: coefficient for adhesion [SI:N/m]; Edinburgh Adhesive Elasto-Plastic Model; set to 0 to deactivate adhesion model
  - `adhesionExponent`: exponent for adhesion coefficient [SI:1]; Edinburgh Adhesive Elasto-Plastic Model
  - `restitutionCoefficient`: coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)
  - `minimumImpactVelocity`: minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)
  - `impactModel`: the impact model, exu.ContactImpactModel: Linear (only the linear contactDamping), HuntCrossley or GonthierCarvalhoMartins, the latter much more accurate regarding the coefficient of restitution in the full range (0, 1]; the integers 0, 1, 2 are accepted as well; in all models, the linear contactDamping is added, if not set to zero (#3327)
  - `dataInitialCoordinates`: a list of four values for initialization of the data node, used for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused
  - `activeConnector`: flag to activate or deactivate the connector
  - `show`: if True, connector visualization is drawn
  - `color`: color of connector
  - `bodyNumbers`: deprecated name of itemNumbers
  - `bodyOrNodeList`: not accepted, a TypeError: the items are given in itemNumbers
- **output**: (type: ObjectIndex) returns index of created joint

*example*:

```python
  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  oGround = mbs.AddObject(ObjectGround())
  b0 = mbs.CreateMassPoint(referencePosition = [11,1,0],
                           initialVelocity = [0.5,0,0],
                           mass = 1, gravity = [0,-9.81,0],
                           drawSize = 0.4, color=exu.graphics.color.blue)
  #a sphere of radius 0.2 falls onto a sphere of radius 0.5 fixed to the ground
  mbs.CreateSphereSphereContact(itemNumbers=[oGround, b0],
                                localPosition0=[11,0,0],
                                spheresRadii=[0.5,0.2],
                                contactStiffness=1e4, contactDamping=10)
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`contactPrimitivesTableTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/contactPrimitivesTableTest.py) (TM), [`createContactSphereSphere.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createContactSphereSphere.py) (TM), [`createItemNumbersTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createItemNumbersTest.py) (TM), [`createSphereQuadContact.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createSphereQuadContact.py) (TM), [`createSphereQuadContact2.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createSphereQuadContact2.py) (TM), [`createSphereTriangleContact.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createSphereTriangleContact.py) (TM), [`restartFileTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/restartFileTest.py) (TM), [`symbolicSolversAdaptiveStepTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/symbolicSolversAdaptiveStepTest.py) (TM), ...


(sec-mainsystemextensions-createspherequadcontact)=
#### Function: CreateSphereQuadContact

[`CreateSphereQuadContact(mbs, name = '', itemNumbers = [None, None], localPosition0 = [0.,0.,0.], sphereRadius = 0, quadPoints = exudyn.Vector3DList([[0,0,0],[1,0,0],[1,1,0],[0,1,0]]), includeEdges = 15, dynamicFriction = 0., frictionProportionalZone = 1e-3, contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1, restitutionCoefficient = 1, minimumImpactVelocity = 0, impactModel = exu.ContactImpactModel.Linear, dataInitialCoordinates = [0,0,0,0], activeConnector = True, localPosition1 = [0.,0.,0.], show = False, color = exudyn.graphics.color.default, radiusSphere = None, bodyNumbers = None, bodyOrNodeList = None) -> dict`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L2428)

- **function description**: Create penalty-based sphere-quad contact between two rigid bodies, mass points or according nodes; the contact is based on two ObjectContactSphereTriangle; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems - NOTE that this function is added to MainSystem via Python function MainSystemCreateSphereQuadContact.
- **input**:
  - `mbs`: the MainSystem where joint and markers shall be created
  - `name`: name string for joint; markers get Marker0:name and Marker1:name
  - `itemNumbers`: the item the sphere is attached to - a body (ObjectIndex) at localPosition0, a node (NodeIndex) or a marker (MarkerIndex), for which localPosition0 is [0,0,0]; with friction, a rigid one - and the body (ObjectIndex) of the quad
  - `localPosition0`: local position (as 3D list or numpy array) of sphere0 on body0, if not a node number
  - `sphereRadius`: radius of sphere 0 [SI:m].
  - `quadPoints`: 4 points as Vector3DList, list or numpy array to define the quad, defined in body1 local coordinates; note that the quad is split into two triangles with point indices [0,1,3] and [1,2,3]
  - `includeEdges`: binary flag, where 1 defines contact with edges 0, 2 with edge 1, 4 with edge 2 and 8 with edge 3; 15 means that contact with all edges is included; edge 0 is the edge between node 0 and node 1, etc.
  - `dynamicFriction`: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, Section Module: physics
  - `frictionProportionalZone`: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), Section Module: physics
  - `contactStiffness`: normal contact stiffness
  - `contactDamping`: linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
  - `contactStiffnessExponent`: exponent in normal contact model [SI:1]
  - `restitutionCoefficient`: coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)
  - `minimumImpactVelocity`: minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)
  - `impactModel`: the impact model, exu.ContactImpactModel: Linear (only the linear contactDamping), HuntCrossley or GonthierCarvalhoMartins, the latter much more accurate regarding the coefficient of restitution in the full range (0, 1]; the integers 0, 1, 2 are accepted as well; in all models, the linear contactDamping is added, if not set to zero (#3327)
  - `dataInitialCoordinates`: a list of four values for initialization of the data node, used for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused
  - `activeConnector`: flag to activate or deactivate the connector
  - `localPosition1`: local position (as 3D list or numpy array) of quad1 on body1; this is usually not needed and adds simply an offset to the quad coordinates
  - `show`: if True, connector visualization is drawn
  - `color`: color of connector
  - `radiusSphere`: deprecated name of sphereRadius
  - `bodyNumbers`: deprecated name of itemNumbers
  - `bodyOrNodeList`: not accepted, a TypeError: the items are given in itemNumbers
- **output**: (type: dict) dictionary containing oContact0 and oContact1 with ObjectIndex of each contact object


Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`rendererNOGLFWexample.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/rendererNOGLFWexample.py) (Ex), [`createItemNumbersTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createItemNumbersTest.py) (TM), [`createSphereQuadContact.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createSphereQuadContact.py) (TM), [`createSphereQuadContact2.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createSphereQuadContact2.py) (TM), [`createSphereTriangleContact.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createSphereTriangleContact.py) (TM)


(sec-mainsystemextensions-createspheretrianglecontact)=
#### Function: CreateSphereTriangleContact

[`CreateSphereTriangleContact(mbs, name = '', itemNumbers = [None, None], localPosition0 = [0.,0.,0.], sphereRadius = 0, trianglePoints = exudyn.Vector3DList([[0,0,0],[1,0,0],[0,1,0]]), includeEdges = 7, dynamicFriction = 0., frictionProportionalZone = 1e-3, contactStiffness = 0., contactDamping = 0., contactStiffnessExponent = 1, restitutionCoefficient = 1, minimumImpactVelocity = 0, impactModel = exu.ContactImpactModel.Linear, dataInitialCoordinates = [0,0,0,0], activeConnector = True, localPosition1 = [0.,0.,0.], infinitePlane = False, show = False, color = exudyn.graphics.color.default, radiusSphere = None, bodyNumbers = None, bodyOrNodeList = None) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L2558)

- **function description**: Create penalty-based sphere-triangle contact between two rigid bodies, mass points or according nodes; the contact is based on ObjectContactSphereTriangle; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems - NOTE that this function is added to MainSystem via Python function MainSystemCreateSphereTriangleContact.
- **input**:
  - `mbs`: the MainSystem where joint and markers shall be created
  - `name`: name string for joint; markers get Marker0:name and Marker1:name
  - `itemNumbers`: the item the sphere is attached to - a body (ObjectIndex) at localPosition0, a node (NodeIndex) or a marker (MarkerIndex), for which localPosition0 is [0,0,0]; with friction, a rigid one - and the body (ObjectIndex) of the triangle
  - `localPosition0`: local position (as 3D list or numpy array) of sphere0 on body0, if not a node number
  - `sphereRadius`: radius of sphere 0 [SI:m].
  - `trianglePoints`: triangle points as Vector3DList, list or numpy array to define the triangle, defined in body1 local coordinates
  - `includeEdges`: binary flag, where 1 defines contact with edges 0, 2 with edge 1 and 4 with edge 2; 7 means that contact with all edges is included; edge 0 is the edge between node 0 and node 1, etc.
  - `dynamicFriction`: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, Section Module: physics
  - `frictionProportionalZone`: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), Section Module: physics
  - `contactStiffness`: normal contact stiffness
  - `contactDamping`: linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
  - `contactStiffnessExponent`: exponent in normal contact model [SI:1]
  - `restitutionCoefficient`: coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)
  - `minimumImpactVelocity`: minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)
  - `impactModel`: the impact model, exu.ContactImpactModel: Linear (only the linear contactDamping), HuntCrossley or GonthierCarvalhoMartins, the latter much more accurate regarding the coefficient of restitution in the full range (0, 1]; the integers 0, 1, 2 are accepted as well; in all models, the linear contactDamping is added, if not set to zero (#3327)
  - `dataInitialCoordinates`: a list of four values for initialization of the data node, used for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused
  - `activeConnector`: flag to activate or deactivate the connector
  - `localPosition1`: local position (as 3D list or numpy array) of triangle1 on body1; this is usually not needed and adds simply an offset to the triangle coordinates
  - `infinitePlane`: if True, the contact is with the plane of the triangle, of any size, as a floor or a wall (#3347)
  - `show`: if True, connector visualization is drawn
  - `color`: color of connector
  - `radiusSphere`: deprecated name of sphereRadius
  - `bodyNumbers`: deprecated name of itemNumbers
  - `bodyOrNodeList`: not accepted, a TypeError: the items are given in itemNumbers
- **output**: (type: ObjectIndex) returns index of created joint


Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`contactPrimitivesTableTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/contactPrimitivesTableTest.py) (TM), [`createItemNumbersTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createItemNumbersTest.py) (TM), [`createPrimitiveContactsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createPrimitiveContactsTest.py) (TM), [`createSphereQuadContact.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createSphereQuadContact.py) (TM), [`createSphereTriangleContact.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createSphereTriangleContact.py) (TM)


(sec-mainsystemextensions-createspherecylindercontact)=
#### Function: CreateSphereCylinderContact

[`CreateSphereCylinderContact(mbs, name = '', itemNumbers = [None, None], localPosition0 = [0.,0.,0.], localPosition1 = [0.,0.,0.], localAxis1 = [0.,0.,1.], sphereRadius = 0., cylinderRadius = 0., cylinderLength = 0., contactStiffness = 0., contactDamping = 0., dynamicFriction = 0., frictionProportionalZone = 1e-3, contactStiffnessExponent = 1, restitutionCoefficient = 1, minimumImpactVelocity = 0, impactModel = exu.ContactImpactModel.Linear, contactForceUserFunction = 0, activeConnector = True, show = False, color = exudyn.graphics.color.default) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L2765)

- **function description**: Create the penalty contact of a sphere and a finite cylinder, ObjectContactSphereCylinder, with its markers and its data node (#3342); for a few contacts, while GeneralContact is made for many bodies - NOTE that this function is added to MainSystem via Python function MainSystemCreateSphereCylinderContact.
- **input**:
  - `mbs`: the MainSystem where the markers, the data node and the contact are created
  - `name`: name string of the contact; the markers get Marker0:name and Marker1:name
  - `itemNumbers`: the items of the sphere and of the cylinder, each a body (ObjectIndex), a node (NodeIndex) or a marker (MarkerIndex), which is used as it is; with friction, a rigid one for the sphere
  - `localPosition0`: the center of the sphere on item 0 [SI:m]; [0,0,0] for a node or a marker
  - `localPosition1`: the center of the cylinder on item 1 [SI:m]; [0,0,0] for a node or a marker
  - `localAxis1`: the axis of the cylinder on item 1, need not have unit length; [0,0,1] for a marker, whose z axis is the axis
  - `sphereRadius`: radius of the sphere [SI:m]
  - `cylinderRadius`: radius of the cylinder [SI:m]
  - `cylinderLength`: length of the cylinder [SI:m], centered at localPosition1
  - `contactStiffness`: normal contact stiffness [SI:N/m]
  - `contactDamping`: linear normal contact damping [SI:N/(m s)]
  - `dynamicFriction`: dynamic friction coefficient of the regularized Coulomb friction
  - `frictionProportionalZone`: velocity [SI:m/s] up to which the friction is proportional to the sliding velocity
  - `contactStiffnessExponent`: exponent of the penetration in the normal contact force
  - `restitutionCoefficient`: coefficient of restitution of the impact model
  - `minimumImpactVelocity`: lower bound of the impact velocity of the impact model [SI:m/s]
  - `impactModel`: exu.ContactImpactModel: Linear, HuntCrossley or GonthierCarvalhoMartins; the integers 0, 1, 2 are accepted as well
  - `contactForceUserFunction`: a Python or symbolic user function replacing the contact law, see ObjectContactSphereCylinder
  - `activeConnector`: flag to activate or deactivate the contact
  - `show`: if True, the contact item is drawn with the settings visualizationSettings.contact
  - `color`: color of the drawn shapes, unless the default
- **output**: (type: ObjectIndex) returns the index of the contact item; its data node is mbs.GetObject(oContact)['nodeNumber']

*example*:

```python
  import exudyn as exu
  from exudyn.utilities import *
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  oGround = mbs.CreateGround()
  #a ball resting on a cylinder lying along x
  oBall = mbs.CreateRigidBody(inertia=InertiaSphere(mass=1, radius=0.1), referencePosition=[0,0,0.3], gravity=[0,0,-9.81])
  mbs.CreateSphereCylinderContact(itemNumbers=[oBall, oGround], localPosition1=[0,0,0], localAxis1=[1,0,0],
                                  sphereRadius=0.1, cylinderRadius=0.2, cylinderLength=1,
                                  contactStiffness=1e5, contactDamping=500)
  mbs.Assemble()
  mbs.SolveDynamic()
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`contactPrimitivesTableTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/contactPrimitivesTableTest.py) (TM), [`createPrimitiveContactsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createPrimitiveContactsTest.py) (TM)


(sec-mainsystemextensions-createcylindercylindercontact)=
#### Function: CreateCylinderCylinderContact

[`CreateCylinderCylinderContact(mbs, name = '', itemNumbers = [None, None], localPosition0 = [0.,0.,0.], localAxis0 = [0.,0.,1.], localPosition1 = [0.,0.,0.], localAxis1 = [0.,0.,1.], cylindersRadii = [0., 0.], cylindersLengths = [0., 0.], isHollowCylinder1 = False, isCapsule0 = False, isCapsule1 = False, contactStiffness = 0., contactDamping = 0., dynamicFriction = 0., frictionProportionalZone = 1e-3, contactStiffnessExponent = 1, restitutionCoefficient = 1, minimumImpactVelocity = 0, impactModel = exu.ContactImpactModel.Linear, contactPlasticityRatio = 0., adhesionCoefficient = 0., adhesionExponent = 1., constantPullOffForce = 0., contactForceUserFunction = 0, activeConnector = True, show = False, color = exudyn.graphics.color.default) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L2829)

- **function description**: Create the penalty contact of two finite cylinders, or of a cylinder inside a hollow one, ObjectContactCylinderCylinder, with its markers and its data node of 4 contact points, 8 with isHollowCylinder1 (#3342) - NOTE that this function is added to MainSystem via Python function MainSystemCreateCylinderCylinderContact.
- **input**:
  - `mbs`: the MainSystem where the markers, the data node and the contact are created
  - `name`: name string of the contact; the markers get Marker0:name and Marker1:name
  - `itemNumbers`: the items of cylinder 0 and cylinder 1, each a body (ObjectIndex), a rigid node (NodeIndex) or a rigid marker (MarkerIndex), which is used as it is
  - `localPosition0`: the center of cylinder 0 on item 0 [SI:m]; [0,0,0] for a node or a marker
  - `localAxis0`: the axis of cylinder 0 on item 0, need not have unit length; [0,0,1] for a marker, whose z axis is the axis
  - `localPosition1`: the center of cylinder 1 on item 1 [SI:m]; [0,0,0] for a node or a marker
  - `localAxis1`: the axis of cylinder 1 on item 1; [0,0,1] for a marker
  - `cylindersRadii`: the radii of cylinder 0 and cylinder 1 [SI:m]; of the bore with isHollowCylinder1
  - `cylindersLengths`: the lengths of cylinder 0 and cylinder 1 [SI:m], each centered at its local position
  - `isHollowCylinder1`: if True, cylinder 0 moves inside cylinder 1, a hollow cylinder open at both ends
  - `isCapsule0`: if True, shape 0 is a capsule, cylindersLengths[0] the length between the centers of its half spheres (#3365)
  - `isCapsule1`: if True, shape 1 is a capsule, as isCapsule0
  - `contactStiffness`: normal contact stiffness of each contact point [SI:N/m]
  - `contactDamping`: linear normal contact damping of each contact point [SI:N/(m s)]
  - `dynamicFriction`: dynamic friction coefficient of the regularized Coulomb friction
  - `frictionProportionalZone`: velocity [SI:m/s] up to which the friction is proportional to the sliding velocity
  - `contactStiffnessExponent`: exponent of the penetration in the normal contact force
  - `restitutionCoefficient`: coefficient of restitution of the impact model
  - `minimumImpactVelocity`: lower bound of the impact velocity of the impact model [SI:m/s]
  - `impactModel`: exu.ContactImpactModel: Linear, HuntCrossley or GonthierCarvalhoMartins; the integers 0, 1, 2 are accepted as well
  - `contactPlasticityRatio`: ratio of the stiffness of first loading and of unloading of the adhesive elasto-plastic model; 0: elastic
  - `adhesionCoefficient`: coefficient of the adhesion of the adhesive elasto-plastic model [SI:N/m]
  - `adhesionExponent`: exponent of the adhesion of the adhesive elasto-plastic model
  - `constantPullOffForce`: constant adhesion force of each contact point of the adhesive elasto-plastic model [SI:N]
  - `contactForceUserFunction`: a Python or symbolic user function replacing the contact law, see ObjectContactCylinderCylinder
  - `activeConnector`: flag to activate or deactivate the contact
  - `show`: if True, the contact item is drawn with the settings visualizationSettings.contact
  - `color`: color of the drawn shapes, unless the default
- **output**: (type: ObjectIndex) returns the index of the contact item; its data node is mbs.GetObject(oContact)['nodeNumber']

*example*:

```python
  import exudyn as exu
  from exudyn.utilities import *
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  oGround = mbs.CreateGround()
  #a shaft along x resting in a fixed ring
  oShaft = mbs.CreateRigidBody(inertia=InertiaCylinder(density=1000, length=0.5, outerRadius=0.05, axis=0),
                               referencePosition=[0,0,0], gravity=[0,0,-9.81])
  mbs.CreateCylinderCylinderContact(itemNumbers=[oShaft, oGround], localAxis0=[1,0,0], localAxis1=[1,0,0],
                                    cylindersRadii=[0.05, 0.06], cylindersLengths=[0.5, 0.1], isHollowCylinder1=True,
                                    contactStiffness=1e5, contactDamping=500)
  mbs.Assemble()
  mbs.SolveDynamic()
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`capsulePrimitiveContactTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/capsulePrimitiveContactTest.py) (TM), [`contactPrimitivesTableTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/contactPrimitivesTableTest.py) (TM), [`createPrimitiveContactsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createPrimitiveContactsTest.py) (TM)


(sec-mainsystemextensions-createcylindertrianglecontact)=
#### Function: CreateCylinderTriangleContact

[`CreateCylinderTriangleContact(mbs, name = '', itemNumbers = [None, None], localPosition0 = [0.,0.,0.], localAxis0 = [0.,0.,1.], cylinderRadius = 0., cylinderLength = 0., trianglePoints = [[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]], infinitePlane = False, localPosition1 = [0.,0.,0.], contactStiffness = 0., contactDamping = 0., dynamicFriction = 0., frictionProportionalZone = 1e-3, contactStiffnessExponent = 1, restitutionCoefficient = 1, minimumImpactVelocity = 0, impactModel = exu.ContactImpactModel.Linear, contactPlasticityRatio = 0., adhesionCoefficient = 0., adhesionExponent = 1., constantPullOffForce = 0., contactForceUserFunction = 0, activeConnector = True, show = False, color = exudyn.graphics.color.default) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L2910)

- **function description**: Create the penalty contact of a finite cylinder and a triangle or its plane, ObjectContactCylinderTriangle, with its markers and its data node of 4 contact points (#3342) - NOTE that this function is added to MainSystem via Python function MainSystemCreateCylinderTriangleContact.
- **input**:
  - `mbs`: the MainSystem where the markers, the data node and the contact are created
  - `name`: name string of the contact; the markers get Marker0:name and Marker1:name
  - `itemNumbers`: the items of the cylinder and of the triangle, each a body (ObjectIndex), a rigid node (NodeIndex) or a rigid marker (MarkerIndex), which is used as it is
  - `localPosition0`: the center of the cylinder on item 0 [SI:m]; [0,0,0] for a node or a marker
  - `localAxis0`: the axis of the cylinder on item 0, need not have unit length; [0,0,1] for a marker, whose z axis is the axis
  - `cylinderRadius`: radius of the cylinder [SI:m]
  - `cylinderLength`: length of the cylinder [SI:m], centered at localPosition0
  - `trianglePoints`: the 3 points of the triangle on item 1 [SI:m], relative to localPosition1, as list, numpy array or Vector3DList; the cylinder is on the side of the normal by the order of the points
  - `infinitePlane`: if True, the contact is with the plane of the triangle, of any size
  - `localPosition1`: the position on item 1 which the triangle points are relative to [SI:m]; [0,0,0] for a node or a marker
  - `contactStiffness`: normal contact stiffness of each contact point [SI:N/m]
  - `contactDamping`: linear normal contact damping of each contact point [SI:N/(m s)]
  - `dynamicFriction`: dynamic friction coefficient of the regularized Coulomb friction
  - `frictionProportionalZone`: velocity [SI:m/s] up to which the friction is proportional to the sliding velocity
  - `contactStiffnessExponent`: exponent of the penetration in the normal contact force
  - `restitutionCoefficient`: coefficient of restitution of the impact model
  - `minimumImpactVelocity`: lower bound of the impact velocity of the impact model [SI:m/s]
  - `impactModel`: exu.ContactImpactModel: Linear, HuntCrossley or GonthierCarvalhoMartins; the integers 0, 1, 2 are accepted as well
  - `contactPlasticityRatio`: ratio of the stiffness of first loading and of unloading of the adhesive elasto-plastic model; 0: elastic
  - `adhesionCoefficient`: coefficient of the adhesion of the adhesive elasto-plastic model [SI:N/m]
  - `adhesionExponent`: exponent of the adhesion of the adhesive elasto-plastic model
  - `constantPullOffForce`: constant adhesion force of each contact point of the adhesive elasto-plastic model [SI:N]
  - `contactForceUserFunction`: a Python or symbolic user function replacing the contact law, see ObjectContactCylinderTriangle
  - `activeConnector`: flag to activate or deactivate the contact
  - `show`: if True, the contact item is drawn with the settings visualizationSettings.contact
  - `color`: color of the drawn shapes, unless the default
- **output**: (type: ObjectIndex) returns the index of the contact item; its data node is mbs.GetObject(oContact)['nodeNumber']

*example*:

```python
  import exudyn as exu
  from exudyn.utilities import *
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  oGround = mbs.CreateGround()
  #a roller along y dropped on the floor, the plane z = 0
  oRoller = mbs.CreateRigidBody(inertia=InertiaCylinder(density=1000, length=0.2, outerRadius=0.1, axis=1),
                                referencePosition=[0,0,0.12], gravity=[0,0,-9.81])
  mbs.CreateCylinderTriangleContact(itemNumbers=[oRoller, oGround], localAxis0=[0,1,0], cylinderRadius=0.1, cylinderLength=0.2,
                                    trianglePoints=[[0,0,0],[1,0,0],[0,1,0]], infinitePlane=True,
                                    contactStiffness=1e5, contactDamping=500)
  mbs.Assemble()
  mbs.SolveDynamic()
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`contactPrimitivesTableTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/contactPrimitivesTableTest.py) (TM), [`createPrimitiveContactsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createPrimitiveContactsTest.py) (TM)


(sec-mainsystemextensions-createconeconecontact)=
#### Function: CreateConeConeContact

[`CreateConeConeContact(mbs, name = '', itemNumbers = [None, None], localPosition0 = [0.,0.,0.], localAxis0 = [0.,0.,1.], localPosition1 = [0.,0.,0.], localAxis1 = [0.,0.,1.], conesRadii0 = [0., 0.], conesRadii1 = [0., 0.], conesLengths = [0., 0.], isHollowCone1 = False, contactStiffness = 0., contactDamping = 0., dynamicFriction = 0., frictionProportionalZone = 1e-3, contactStiffnessExponent = 1, restitutionCoefficient = 1, minimumImpactVelocity = 0, impactModel = exu.ContactImpactModel.Linear, contactPlasticityRatio = 0., adhesionCoefficient = 0., adhesionExponent = 1., constantPullOffForce = 0., contactForceUserFunction = 0, activeConnector = True, show = False, color = exudyn.graphics.color.default) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L2985)

- **function description**: Create the penalty contact of two truncated cones, or of a cone inside a hollow one, ObjectContactConeCone, with its markers and its data node of 4 contact points, 8 with isHollowCone1 (#3342) - NOTE that this function is added to MainSystem via Python function MainSystemCreateConeConeContact.
- **input**:
  - `mbs`: the MainSystem where the markers, the data node and the contact are created
  - `name`: name string of the contact; the markers get Marker0:name and Marker1:name
  - `itemNumbers`: the items of cone 0 and cone 1, each a body (ObjectIndex), a rigid node (NodeIndex) or a rigid marker (MarkerIndex), which is used as it is
  - `localPosition0`: the center of cone 0 on item 0 [SI:m]; [0,0,0] for a node or a marker
  - `localAxis0`: the axis of cone 0 on item 0, from its first to its second radius, need not have unit length; [0,0,1] for a marker, whose z axis is the axis
  - `localPosition1`: the center of cone 1 on item 1 [SI:m]; [0,0,0] for a node or a marker
  - `localAxis1`: the axis of cone 1 on item 1; [0,0,1] for a marker
  - `conesRadii0`: the radii of cone 0 at the start and at the end of its axis [SI:m]; one may be 0, a pointed cone
  - `conesRadii1`: the radii of cone 1 at the start and at the end of its axis [SI:m]; of the bore with isHollowCone1
  - `conesLengths`: the lengths of cone 0 and cone 1 [SI:m], each centered at its local position
  - `isHollowCone1`: if True, cone 0 moves inside cone 1, a hollow cone open at both ends
  - `contactStiffness`: normal contact stiffness of each contact point [SI:N/m]
  - `contactDamping`: linear normal contact damping of each contact point [SI:N/(m s)]
  - `dynamicFriction`: dynamic friction coefficient of the regularized Coulomb friction
  - `frictionProportionalZone`: velocity [SI:m/s] up to which the friction is proportional to the sliding velocity
  - `contactStiffnessExponent`: exponent of the penetration in the normal contact force
  - `restitutionCoefficient`: coefficient of restitution of the impact model
  - `minimumImpactVelocity`: lower bound of the impact velocity of the impact model [SI:m/s]
  - `impactModel`: exu.ContactImpactModel: Linear, HuntCrossley or GonthierCarvalhoMartins; the integers 0, 1, 2 are accepted as well
  - `contactPlasticityRatio`: ratio of the stiffness of first loading and of unloading of the adhesive elasto-plastic model; 0: elastic
  - `adhesionCoefficient`: coefficient of the adhesion of the adhesive elasto-plastic model [SI:N/m]
  - `adhesionExponent`: exponent of the adhesion of the adhesive elasto-plastic model
  - `constantPullOffForce`: constant adhesion force of each contact point of the adhesive elasto-plastic model [SI:N]
  - `contactForceUserFunction`: a Python or symbolic user function replacing the contact law, see ObjectContactConeCone
  - `activeConnector`: flag to activate or deactivate the contact
  - `show`: if True, the contact item is drawn with the settings visualizationSettings.contact
  - `color`: color of the drawn shapes, unless the default
- **output**: (type: ObjectIndex) returns the index of the contact item; its data node is mbs.GetObject(oContact)['nodeNumber']

*example*:

```python
  import exudyn as exu
  from exudyn.utilities import *
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  oGround = mbs.CreateGround()
  #a small cone standing on the top of a fixed cone
  oCone = mbs.CreateRigidBody(inertia=InertiaCylinder(density=1000, length=0.1, outerRadius=0.04, axis=2),
                              referencePosition=[0,0,0.16], gravity=[0,0,-9.81])
  mbs.CreateConeConeContact(itemNumbers=[oCone, oGround], conesRadii0=[0.05, 0.02], conesRadii1=[0.2, 0.1],
                            conesLengths=[0.1, 0.2], contactStiffness=1e5, contactDamping=500)
  mbs.Assemble()
  mbs.SolveDynamic()
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`contactPrimitivesTableTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/contactPrimitivesTableTest.py) (TM), [`createPrimitiveContactsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createPrimitiveContactsTest.py) (TM)


(sec-mainsystemextensions-createconetrianglecontact)=
#### Function: CreateConeTriangleContact

[`CreateConeTriangleContact(mbs, name = '', itemNumbers = [None, None], localPosition0 = [0.,0.,0.], localAxis0 = [0.,0.,1.], coneRadii = [0., 0.], coneLength = 0., trianglePoints = [[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]], infinitePlane = False, localPosition1 = [0.,0.,0.], contactStiffness = 0., contactDamping = 0., dynamicFriction = 0., frictionProportionalZone = 1e-3, contactStiffnessExponent = 1, restitutionCoefficient = 1, minimumImpactVelocity = 0, impactModel = exu.ContactImpactModel.Linear, contactPlasticityRatio = 0., adhesionCoefficient = 0., adhesionExponent = 1., constantPullOffForce = 0., contactForceUserFunction = 0, activeConnector = True, show = False, color = exudyn.graphics.color.default) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L3063)

- **function description**: Create the penalty contact of a truncated cone and a triangle or its plane, ObjectContactConeTriangle, with its markers and its data node of 4 contact points (#3342) - NOTE that this function is added to MainSystem via Python function MainSystemCreateConeTriangleContact.
- **input**:
  - `mbs`: the MainSystem where the markers, the data node and the contact are created
  - `name`: name string of the contact; the markers get Marker0:name and Marker1:name
  - `itemNumbers`: the items of the cone and of the triangle, each a body (ObjectIndex), a rigid node (NodeIndex) or a rigid marker (MarkerIndex), which is used as it is
  - `localPosition0`: the center of the cone on item 0 [SI:m]; [0,0,0] for a node or a marker
  - `localAxis0`: the axis of the cone on item 0, from its first to its second radius, need not have unit length; [0,0,1] for a marker, whose z axis is the axis
  - `coneRadii`: the radii of the cone at the start and at the end of its axis [SI:m]; one may be 0, a pointed cone
  - `coneLength`: length of the cone [SI:m], centered at localPosition0
  - `trianglePoints`: the 3 points of the triangle on item 1 [SI:m], relative to localPosition1, as list, numpy array or Vector3DList; the cone is on the side of the normal by the order of the points
  - `infinitePlane`: if True, the contact is with the plane of the triangle, of any size
  - `localPosition1`: the position on item 1 which the triangle points are relative to [SI:m]; [0,0,0] for a node or a marker
  - `contactStiffness`: normal contact stiffness of each contact point [SI:N/m]
  - `contactDamping`: linear normal contact damping of each contact point [SI:N/(m s)]
  - `dynamicFriction`: dynamic friction coefficient of the regularized Coulomb friction
  - `frictionProportionalZone`: velocity [SI:m/s] up to which the friction is proportional to the sliding velocity
  - `contactStiffnessExponent`: exponent of the penetration in the normal contact force
  - `restitutionCoefficient`: coefficient of restitution of the impact model
  - `minimumImpactVelocity`: lower bound of the impact velocity of the impact model [SI:m/s]
  - `impactModel`: exu.ContactImpactModel: Linear, HuntCrossley or GonthierCarvalhoMartins; the integers 0, 1, 2 are accepted as well
  - `contactPlasticityRatio`: ratio of the stiffness of first loading and of unloading of the adhesive elasto-plastic model; 0: elastic
  - `adhesionCoefficient`: coefficient of the adhesion of the adhesive elasto-plastic model [SI:N/m]
  - `adhesionExponent`: exponent of the adhesion of the adhesive elasto-plastic model
  - `constantPullOffForce`: constant adhesion force of each contact point of the adhesive elasto-plastic model [SI:N]
  - `contactForceUserFunction`: a Python or symbolic user function replacing the contact law, see ObjectContactConeTriangle
  - `activeConnector`: flag to activate or deactivate the contact
  - `show`: if True, the contact item is drawn with the settings visualizationSettings.contact
  - `color`: color of the drawn shapes, unless the default
- **output**: (type: ObjectIndex) returns the index of the contact item; its data node is mbs.GetObject(oContact)['nodeNumber']

*example*:

```python
  import exudyn as exu
  from exudyn.utilities import *
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  oGround = mbs.CreateGround()
  #a cone standing on its base on the floor, the plane z = 0
  oCone = mbs.CreateRigidBody(inertia=InertiaCylinder(density=1000, length=0.2, outerRadius=0.07, axis=2),
                              referencePosition=[0,0,0.11], gravity=[0,0,-9.81])
  mbs.CreateConeTriangleContact(itemNumbers=[oCone, oGround], coneRadii=[0.1, 0.03], coneLength=0.2,
                                trianglePoints=[[0,0,0],[1,0,0],[0,1,0]], infinitePlane=True,
                                contactStiffness=1e5, contactDamping=500)
  mbs.Assemble()
  mbs.SolveDynamic()
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`contactPrimitivesTableTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/contactPrimitivesTableTest.py) (TM), [`createPrimitiveContactsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createPrimitiveContactsTest.py) (TM)


(sec-mainsystemextensions-createconvexconvexcontact)=
#### Function: CreateConvexConvexContact

[`CreateConvexConvexContact(mbs, name = '', itemNumbers = [None, None], points0 = None, triangles0 = None, points1 = None, triangles1 = None, localPosition0 = [0.,0.,0.], localPosition1 = [0.,0.,0.], contactStiffness = 0., contactDamping = 0., dynamicFriction = 0., frictionProportionalZone = 1e-3, contactStiffnessExponent = 1, restitutionCoefficient = 1, minimumImpactVelocity = 0, impactModel = exu.ContactImpactModel.Linear, contactPlasticityRatio = 0., adhesionCoefficient = 0., adhesionExponent = 1., constantPullOffForce = 0., contactForceUserFunction = 0, activeConnector = True, show = False, color = exudyn.graphics.color.default) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L3141)

- **function description**: Create the penalty contact of two convex polytopes, ObjectContactConvexConvex, with its markers and its data node of 8 contact points (#3342); the points and triangles may come from graphics.ToPointsAndTrigs of the graphics of a body - NOTE that this function is added to MainSystem via Python function MainSystemCreateConvexConvexContact.
- **input**:
  - `mbs`: the MainSystem where the markers, the data node and the contact are created
  - `name`: name string of the contact; the markers get Marker0:name and Marker1:name
  - `itemNumbers`: the items of polytope 0 and polytope 1, each a body (ObjectIndex), a rigid node (NodeIndex) or a rigid marker (MarkerIndex), which is used as it is
  - `points0`: the points of polytope 0 relative to localPosition0 on item 0 [SI:m], a list or a numpy array of n x 3; their convex hull is the shape
  - `triangles0`: the triangles of the surface of polytope 0, with outward normals, a list or a numpy array of m x 3 indices of points0
  - `points1`: the points of polytope 1 relative to localPosition1 on item 1 [SI:m]
  - `triangles1`: the triangles of the surface of polytope 1, with outward normals
  - `localPosition0`: the position on item 0 which points0 are relative to [SI:m]; [0,0,0] for a node or a marker
  - `localPosition1`: the position on item 1 which points1 are relative to [SI:m]; [0,0,0] for a node or a marker
  - `contactStiffness`: normal contact stiffness of each contact point [SI:N/m]
  - `contactDamping`: linear normal contact damping of each contact point [SI:N/(m s)]
  - `dynamicFriction`: dynamic friction coefficient of the regularized Coulomb friction
  - `frictionProportionalZone`: velocity [SI:m/s] up to which the friction is proportional to the sliding velocity
  - `contactStiffnessExponent`: exponent of the penetration in the normal contact force
  - `restitutionCoefficient`: coefficient of restitution of the impact model
  - `minimumImpactVelocity`: lower bound of the impact velocity of the impact model [SI:m/s]
  - `impactModel`: exu.ContactImpactModel: Linear, HuntCrossley or GonthierCarvalhoMartins; the integers 0, 1, 2 are accepted as well
  - `contactPlasticityRatio`: ratio of the stiffness of first loading and of unloading of the adhesive elasto-plastic model; 0: elastic
  - `adhesionCoefficient`: coefficient of the adhesion of the adhesive elasto-plastic model [SI:N/m]
  - `adhesionExponent`: exponent of the adhesion of the adhesive elasto-plastic model
  - `constantPullOffForce`: constant adhesion force of each contact point of the adhesive elasto-plastic model [SI:N]
  - `contactForceUserFunction`: a Python or symbolic user function replacing the contact law, see ObjectContactConvexConvex
  - `activeConnector`: flag to activate or deactivate the contact
  - `show`: if True, the contact item is drawn with the settings visualizationSettings.contact
  - `color`: color of the drawn shapes, unless the default
- **output**: (type: ObjectIndex) returns the index of the contact item; its data node is mbs.GetObject(oContact)['nodeNumber']

*example*:

```python
  import exudyn as exu
  from exudyn.utilities import *
  import exudyn.graphics as graphics
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  oGround = mbs.CreateGround()
  #a cube dropped on a fixed slab, both from the points and triangles of their graphics
  cube = graphics.Brick(size=[0.2]*3)
  oCube = mbs.CreateRigidBody(inertia=InertiaCuboid(density=1000, sideLengths=[0.2]*3), referencePosition=[0,0,0.12],
                              gravity=[0,0,-9.81], graphicsDataList=[cube])
  (points0, triangles0) = graphics.ToPointsAndTrigs(cube)
  (points1, triangles1) = graphics.ToPointsAndTrigs(graphics.Brick(centerPoint=[0,0,-0.05], size=[1,1,0.1]))
  mbs.CreateConvexConvexContact(itemNumbers=[oCube, oGround], points0=points0, triangles0=triangles0,
                                points1=points1, triangles1=triangles1, contactStiffness=1e5, contactDamping=500)
  mbs.Assemble()
  mbs.SolveDynamic()
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`contactPrimitivesTableTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/contactPrimitivesTableTest.py) (TM), [`createPrimitiveContactsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createPrimitiveContactsTest.py) (TM)


(sec-mainsystemextensions-createprimitiveconvexcontact)=
#### Function: CreatePrimitiveConvexContact

[`CreatePrimitiveConvexContact(mbs, name = '', itemNumbers = [None, None], primitiveType = exu.ContactPrimitiveType.Sphere, radius = 0., radius1 = 0., length = 0., points1 = None, triangles1 = None, localPosition0 = [0.,0.,0.], localAxis0 = [0.,0.,1.], localPosition1 = [0.,0.,0.], contactStiffness = 0., contactDamping = 0., dynamicFriction = 0., frictionProportionalZone = 1e-3, contactStiffnessExponent = 1, restitutionCoefficient = 1, minimumImpactVelocity = 0, impactModel = exu.ContactImpactModel.Linear, contactPlasticityRatio = 0., adhesionCoefficient = 0., adhesionExponent = 1., constantPullOffForce = 0., contactForceUserFunction = 0, activeConnector = True, show = False, color = exudyn.graphics.color.default) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L3220)

- **function description**: Create the penalty contact of a sphere, capsule, cylinder or truncated cone with a convex polytope, ObjectContactPrimitiveConvex, with its markers and its data node of 4 contact points (#3364). - NOTE that this function is added to MainSystem via Python function MainSystemCreatePrimitiveConvexContact.
- **input**:
  - `mbs`: the MainSystem where the markers, the data node and the contact are created
  - `name`: name of the item; the markers and the data node get names derived from it
  - `itemNumbers`: the two bodies, nodes or markers of the shapes, [primitive, polytope]; a body or node is given a marker
  - `primitiveType`: exu.ContactPrimitiveType: Sphere, Capsule, Cylinder or Cone; the integers 0 to 3 are accepted as well
  - `radius`: the radius of the primitive; of the cone at the start of its axis [SI:m]
  - `radius1`: the radius of the cone at the end of its axis [SI:m]; only used for Cone
  - `length`: the length of the cylinder or the cone, or that of a capsule between the centers of its half spheres [SI:m]
  - `points1`: the points of the polytope in the frame of item 1, an n x 3 list or numpy array, for example from graphics.ToPointsAndTrigs
  - `triangles1`: the triangles of the polytope, an n x 3 integer array of indices into points1, with the normal out of the shape
  - `localPosition0`: the center of the primitive on item 0; [0,0,0] for a marker
  - `localAxis0`: the axis of the primitive on item 0, need not have unit length; [0,0,1] for a marker, whose z axis is the axis
  - `localPosition1`: the offset added to points1
  - `contactStiffness`: normal contact stiffness of each contact point [SI:N/m]
  - `contactDamping`: linear normal contact damping of each contact point [SI:N/(m s)]
  - `dynamicFriction`: dynamic friction coefficient of the regularized Coulomb friction
  - `frictionProportionalZone`: velocity [SI:m/s] up to which the friction is proportional to the sliding velocity
  - `contactStiffnessExponent`: exponent of the penetration in the normal contact force
  - `restitutionCoefficient`: coefficient of restitution of the impact model
  - `minimumImpactVelocity`: lower bound of the impact velocity of the impact model [SI:m/s]
  - `impactModel`: exu.ContactImpactModel: Linear, HuntCrossley or GonthierCarvalhoMartins; the integers 0, 1, 2 are accepted as well
  - `contactPlasticityRatio`: ratio of the stiffness of first loading and of unloading of the adhesive elasto-plastic model; 0: elastic
  - `adhesionCoefficient`: coefficient of the adhesion of the adhesive elasto-plastic model [SI:N/m]
  - `adhesionExponent`: exponent of the adhesion of the adhesive elasto-plastic model
  - `constantPullOffForce`: constant adhesion force of each contact point of the adhesive elasto-plastic model [SI:N]
  - `contactForceUserFunction`: a Python or symbolic user function replacing the contact law, see ObjectContactPrimitiveConvex
  - `activeConnector`: False: no force, the output Distance is still computed
  - `show`: True: the shapes are drawn
  - `color`: the color of the shapes
- **output**: (type: ObjectIndex) returns the index of the contact item; its data node is mbs.GetObject(oContact)['nodeNumber']

*example*:

```python
  mbs.CreatePrimitiveConvexContact(itemNumbers=[oBall, oGround], primitiveType=exu.ContactPrimitiveType.Sphere, radius=0.1,
                                   points1=points, triangles1=triangles, contactStiffness=1e5, contactDamping=500)
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`capsulePrimitiveContactTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/capsulePrimitiveContactTest.py) (TM)


(sec-mainsystemextensions-createkinematictree)=
#### Function: CreateKinematicTree

[`CreateKinematicTree(mbs, name = '', listOfTreeLinks = [], referenceCoordinates = None, initialCoordinates = None, initialCoordinates_t = None, gravity = [0.,0.,0.], baseOffset = [0.,0.,0.], linkForces = None, linkTorques = None, jointForceVector = None, jointPositionOffsetVector = None, jointVelocityOffsetVector = None, forceUserFunction = 0, jointRadius = 0.05, jointWidth = 0.12, colors = exudyn.graphics.color.default, colorsJoints = exudyn.graphics.color.default, baseGraphicsDataList = None, linkRoundness = 0.2, show = True) -> exudyn.ObjectIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L3291)

- **function description**: helper function to create 2D or 3D mass point object and node, using arguments as in NodePoint and MassPoint; uses TreeLink as defined in exudyn.rigidBodyUtilities - NOTE that this function is added to MainSystem via Python function MainSystemCreateKinematicTree.
- **input**:
  - `mbs`: the MainSystem where items are created
  - `name`: name string for object, node is 'Node:'+name
  - `listOfTreeLinks`: list of TreeLink (from exudyn.rigidBodyUtilities) which characterize the KinematicTree
  - `referenceCoordinates`: reference coordinates all kinematic tree coordinates (e.g., joint angles); i.e., configuration where displacements are zero
  - `initialCoordinates`: initial deviation from reference coordinates (= displacements)
  - `initialCoordinates_t`: initial velocities (e.g., of joint angles)
  - `gravity`: gravity vevtor applied to kinematic tree (always a 3D vector, no matter if 2D or 3D mass)
  - `baseOffset`: constant 3D vector representing the origin of the kinematic tree
  - `linkForces`: Vector3DList of forces per link (at joint origin) or None
  - `linkTorques`: Vector3DList of torques per link or None
  - `jointForceVector`: a list or numpy array of scalar forces per joint, representing joint forces (prismatic joint) or joint torques (revolute joint)
  - `jointPositionOffsetVector`: a list or numpy array of scalar set coordinates per joint; use PreStepUserFunction to change values over time
  - `jointVelocityOffsetVector`: a list or numpy array of scalar set velocities per joint; use PreStepUserFunction to change values over time
  - `forceUserFunction`: A Python user function which computes the generalized force vector on RHS with identical action as jointForceVector; for description see ObjectKinematicTree
  - `show`: show kinematic tree
  - `showLinks`: set true, if links shall be shown; if graphicsDataList is empty, a standard drawing for links is used (drawing a cylinder from previous joint or base to next joint; size relative to frame size in KinematicTree visualization settings); else graphicsDataList are used per link; NOTE visualization of joint and COM frames can be modified via visualizationSettings.bodies.kinematicTree
  - `showJoints`: set true, if joints shall be shown; if graphicsDataList is empty, a standard drawing for joints is used (drawing a cylinder for revolute joints; size relative to frame size in KinematicTree visualization settings)
  - `jointRadius`: for generic visualization of joints and links
  - `jointWidth`: for generic visualization of joints and links
  - `colors`: either one general color for kinematic tree, or list with one color per link
  - `colorsJoints`: either one color for all joints or list with one color per joint
  - `baseGraphicsDataList`: graphics for base; if None, it is computed automatically; otherwise a list of graphicsData or empty list
  - `linkRoundness`: for automatic generation of graphics for links, roundness=0 give brick-shape, roundness<1 give transition of brick to ellipsoid and roundness=1 give cylinders
  - `show`: show kinematic tree
- **output**: (type: ObjectIndex) returns kinematic tree object index


Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`kinematicTreeAndMBS.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/kinematicTreeAndMBS.py) (Ex), [`kinematicTreePendulum.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/kinematicTreePendulum.py) (Ex), [`openAIgymNLinkAdvanced.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/openAIgymNLinkAdvanced.py) (Ex), [`openAIgymNLinkContinuous.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/openAIgymNLinkContinuous.py) (Ex), [`robotics/humanRobotInteraction.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/robotics/humanRobotInteraction.py) (Ex), [`createKinematicTreeTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createKinematicTreeTest.py) (TM), [`kinematicTreeAndMBStest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/kinematicTreeAndMBStest.py) (TM), [`kinematicTreeConstraintTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/kinematicTreeConstraintTest.py) (TM), ...


(sec-mainsystemextensions-createffrfreducedorderobject)=
#### Function: CreateFFRFReducedOrderObject

[`CreateFFRFReducedOrderObject(mbs, name, femInterface, referencePosition = [0., 0., 0.], initialVelocity = [0., 0., 0.], referenceRotationMatrix = np.eye(3), initialAngularVelocity = [0., 0., 0.], massProportionalDamping = 0., stiffnessProportionalDamping = 0., gravity = [0., 0., 0.], color = exudyn.graphics.color.defaultFFRF, rigidBodyNodeType = exu.NodeType.RotationEulerParameters, superElementRigidMarkersOffsets = None, superElementRigidMarkersLocalHTs = None, showMarkers = True, verbose = False) -> dict`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L3593)

- **function description**: Create an FFRF reduced order object; the function adds SuperElementRigid markers if boundaries are defined in the given femInterface and thus enables straightforward integration of flexible bodies into a multibody system - NOTE that this function is added to MainSystem via Python function MainSystemCreateFFRFReducedOrderObject.
- **input**:
  - `mbs`: the MainSystem to which the FFRF reduced order object and the SuperElementRigid markers are added
  - `name`: name of the FFRF reduced order object; used to name the created SuperElementRigid markers (name + ':' + boundaryName), the rigid body node ('NodeRigidBody:' + name), and the generic ODE2 node ('NodeGeneric:' + name); if no name is available, set name=None
  - `femInterface`: an instance of EXUDYN's FEMinterface class; this instance must hold at least a position-based mesh and eigenmodes of the system (for model reduction); usually, also boundaries named [boundaryName0, boundaryName1, ...] are defined within the femInterface; if no boundaries are defined, no SuperElementRigid markers are added
  - `referencePosition`: reference position of the floating frame (i.e. of the rigid body node) (always a 3D vector)
  - `initialVelocity`: initial velocity of the floating frame (i.e. of the rigid body node) (always a 3D vector)
  - `referenceRotationMatrix`: reference rotation matrix for the floating frame (i.e. of the rigid body node) (always a 3D matrix)
  - `initialAngularVelocity`: initial angular velocity of the floating frame (i.e. of the rigid body node) (always a 3D vector)
  - `massProportionalDamping`: Rayleigh damping factor for mass proportional damping (multiplied with reduced mass matrix), added to floating frame/modal coordinates only
  - `stiffnessProportionalDamping`: Rayleigh damping factor for stiffness proportional damping (multiplied with reduced stiffness matrix), added to floating frame/modal coordinates only
  - `gravity`: gravity applied to the FFRF reduced order object (always a 3D vector)
  - `color`: color with which the FFRF reduced order object is drawn (if no contour is set in the visualization settings)
  - `rigidBodyNodeType`: node type of the rigid body node of the floating frame: exu.NodeType.RotationEulerParameters, exu.NodeType.RotationRxyz or exu.NodeType.RotationRotationVector, or one of them as string, e.g., 'NodeType.RotationRxyz'
  - `superElementRigidMarkersOffsets`: if not None, adds local offsets to the created SuperElementRigid markers; if N boundaries are defined in the femInterface, a N x 3 list or N x 3 np.array sets an offset for each added marker, for N=1 also a vector with 3 entries; the order of the offsets follows the order in [boundaryName0, boundaryName1, ...] used when setting up the femInterface
  - `superElementRigidMarkersLocalHTs`: if not None, sets the local homogeneous transformations of the created SuperElementRigid markers: the frame of a marker against the frame the marker computes from the mesh nodes; if N boundaries are defined in the femInterface, a list of N transformations, each an exu.HT or a 4x4 matrix given as list of lists or np.array, sets one transformation for each added marker; the order of the transformations follows the order in [boundaryName0, boundaryName1, ...] used when setting up the femInterface; the translation of a transformation is the offset of the marker, so if superElementRigidMarkersOffsets is also given, both must agree
  - `showMarkers`: if True, SuperElementRigid markers are drawn
  - `verbose`: if True, additional information will be printed in the console upon calling the function
- **output**: (type: dict) dictionary mapping each created SuperElementRigid marker name to its marker number, plus an additional entry under 'FFRFReducedOrderObjectDict' containing information about the created FFRF reduced order object
- **author**: Sebastian Weyrer

*example*:

```python
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
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`NGsolveCreateFFRFreducedOrder.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/NGsolveCreateFFRFreducedOrder.py) (Ex), [`NGsolveFFRFSlidingJoint.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/NGsolveFFRFSlidingJoint.py) (Ex), [`createFFRFReducedOrderObjectTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createFFRFReducedOrderObjectTest.py) (TM), [`modelLibrary.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/modelLibrary.py) (TM)


(sec-mainsystemextensions-createforce)=
#### Function: CreateForce

[`CreateForce(mbs, name = '', itemNumber = None, loadVector = [0.,0.,0.], localPosition = [0.,0.,0.], bodyFixed = False, loadVectorUserFunction = 0, show = True, bodyNumber = None) -> exudyn.LoadIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L3808)

- **function description**: helper function to create force applied to a body, a node or a marker - NOTE that this function is added to MainSystem via Python function MainSystemCreateForce.
- **input**:
  - `mbs`: the MainSystem where items are created
  - `name`: name string for object
  - `itemNumber`: the item the force is applied to: a body (ObjectIndex) at localPosition, a node (NodeIndex) or a marker (MarkerIndex), for which localPosition is [0,0,0]
  - `loadVector`: force vector (as 3D list or numpy array)
  - `localPosition`: local position (as 3D list or numpy array) on the body where the force is applied
  - `bodyFixed`: if True, the force is corotated with the body or node; else, the force is global
  - `loadVectorUserFunction`: A Python function f(mbs, t, load)->loadVector which defines the time-dependent load and replaces loadVector in every time step; the arg load is the static loadVector
  - `show`: if True, load is drawn
  - `bodyNumber`: deprecated name of itemNumber
- **output**: (type: LoadIndex) returns load index

*example*:

```python
  import exudyn as exu
  from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  b0=mbs.CreateMassPoint(referencePosition = [0,0,0],
                         initialVelocity = [2,5,0],
                         mass = 1, gravity = [0,-9.81,0],
                         drawSize = 0.5, color=exu.graphics.color.blue)
  f0=mbs.CreateForce(itemNumber=b0, loadVector=[100,0,0],
                     localPosition=[0,0,0])
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`ballBearningModel.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/ballBearningModel.py) (Ex), [`cartesianSpringDamper.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/cartesianSpringDamper.py) (Ex), [`cartesianSpringDamperUserFunction.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/cartesianSpringDamperUserFunction.py) (Ex), [`chatGPTupdate.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/chatGPTupdate.py) (Ex), [`chatGPTupdate2.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/chatGPTupdate2.py) (Ex), [`appendSystemTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/appendSystemTest.py) (TM), [`ballBearingTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/ballBearingTest.py) (TM), [`createFunctionsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createFunctionsTest.py) (TM), ...


(sec-mainsystemextensions-createtorque)=
#### Function: CreateTorque

[`CreateTorque(mbs, name = '', itemNumber = None, loadVector = [0.,0.,0.], localPosition = [0.,0.,0.], bodyFixed = False, loadVectorUserFunction = 0, show = True, bodyNumber = None) -> exudyn.LoadIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L3892)

- **function description**: helper function to create torque applied to a body, a node or a marker - NOTE that this function is added to MainSystem via Python function MainSystemCreateTorque.
- **input**:
  - `mbs`: the MainSystem where items are created
  - `name`: name string for object
  - `itemNumber`: the item the torque is applied to: a body (ObjectIndex) at localPosition, a rigid node (NodeIndex) or a rigid marker (MarkerIndex), for which localPosition is [0,0,0]
  - `loadVector`: torque vector (as 3D list or numpy array)
  - `localPosition`: local position (as 3D list or numpy array) on the body where the torque is applied
  - `bodyFixed`: if True, the torque is corotated with the body or node; else, the torque is global
  - `loadVectorUserFunction`: A Python function f(mbs, t, load)->loadVector which defines the time-dependent load and replaces loadVector in every time step; the arg load is the static loadVector
  - `show`: if True, load is drawn
  - `bodyNumber`: deprecated name of itemNumber
- **output**: (type: LoadIndex) returns load index

*example*:

```python
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
  f0=mbs.CreateTorque(itemNumber=b0, loadVector=[0,100,0])
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values
  simulationSettings.timeIntegration.numberOfSteps = 1000
  simulationSettings.timeIntegration.endTime = 2
  mbs.SolveDynamic(simulationSettings = simulationSettings)
```



Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`ballBearningModel.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/ballBearningModel.py) (Ex), [`chatGPTupdate.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/chatGPTupdate.py) (Ex), [`chatGPTupdate2.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/chatGPTupdate2.py) (Ex), [`ballBearingTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/ballBearingTest.py) (TM), [`createFunctionsTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createFunctionsTest.py) (TM), [`createItemNumbersTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/createItemNumbersTest.py) (TM), [`mainSystemExtensionsTests.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/mainSystemExtensionsTests.py) (TM), [`pickleCopyMbs.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/pickleCopyMbs.py) (TM), ...


(sec-mainsystemextensions-createdistancesensorgeometry)=
#### Function: CreateDistanceSensorGeometry

[`CreateDistanceSensorGeometry(mbs, meshPoints, meshTrigs, rigidBodyMarkerIndex, searchTreeCellSize = [8,8,8]) -> int`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L4049)

- **function description**: Add geometry for distance sensor given by points and triangles (point indices) to mbs; use a rigid body marker where the geometry is put on; Creates a GeneralContact for efficient search on background. If you have several sets of points and trigs, first merge them or add them manually to the contact - NOTE that this function is added to MainSystem via Python function CreateDistanceSensorGeometry.
- **input**:
  - `mbs`: MainSystem where contact is created
  - `meshPoints`: list of points (3D), as returned by graphics.ToPointsAndTrigs()
  - `meshTrigs`: list of trigs (3 node indices each), as returned by graphics.ToPointsAndTrigs()
  - `rigidBodyMarkerIndex`: rigid body marker to which the triangles are fixed on (ground or moving object)
  - `searchTreeCellSize`: size of search tree (X,Y,Z); use larger values in directions where more triangles are located
- **output**: (type: int) returns ngc, which is the number of GeneralContact in mbs, to be used in CreateDistanceSensor(...); keep the gContact as deletion may corrupt data
- **notes**: should be used by CreateDistanceSensor(...) and AddLidar(...) for simple initialization of GeneralContact; old name: DistanceSensorSetupGeometry(...)


Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`robotics/mobileMecanumWheelRobotWithLidar.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/Examples/robotics/mobileMecanumWheelRobotWithLidar.py) (Ex), [`laserScannerTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/laserScannerTest.py) (TM)


(sec-mainsystemextensions-createdistancesensor)=
#### Function: CreateDistanceSensor

[`CreateDistanceSensor(mbs, generalContactIndex, positionOrMarker, dirSensor, minDistance = -1e7, maxDistance = 1e7, cylinderRadius = 0, selectedTypeIndex = exudyn.ContactTypeIndex.IndexEndOfEnumList, storeInternal = False, fileName = '', measureVelocity = False, addGraphicsObject = False, drawDisplaced = True, color = exudyn.graphics.color.red) -> exudyn.SensorIndex`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/exudyn/misc/mainSystemExtensions.py#L4081)

- **function description**: Function to create distance sensor based on GeneralContact in mbs; sensor can be either placed on absolute position or attached to rigid body marker; in case of marker, dirSensor is relative to the marker - NOTE that this function is added to MainSystem via Python function CreateDistanceSensor.
- **input**:
  - `mbs`: the MainSystem where distance sensor is created
  - `generalContactIndex`: the number of the GeneralContact object in mbs; the index of the GeneralContact object which has been added with last AddGeneralContact(...) command is generalContactIndex=mbs.NumberOfGeneralContacts()-1
  - `positionOrMarker`: either a 3D position as list or np.array, or a MarkerIndex with according rigid body marker
  - `dirSensor`: the direction (no need to normalize) along which the distance is measured (must not be normalized); in case of marker, the direction is relative to marker orientation if marker contains orientation (BodyRigid, NodeRigid)
  - `minDistance`: the minimum distance which is accepted; smaller distance will be ignored
  - `maxDistance`: the maximum distance which is accepted; items being at maxDistance or futher are ignored; if no items are found, the function returns maxDistance
  - `cylinderRadius`: in case of spheres (selectedTypeIndex=ContactTypeIndex.IndexSpheresMarkerBased), a cylinder can be used which measures the shortest distance at a certain radius (geometrically interpreted as cylinder)
  - `selectedTypeIndex`: either this type has default value, meaning that all items in GeneralContact are measured, or there is a specific type index, which is the only type that is considered during measurement
  - `storeInternal`: like with any SensorUserFunction, setting to True stores sensor data internally
  - `fileName`: if defined, recorded data of SensorUserFunction is written to specified file
  - `measureVelocity`: if True, the sensor measures additionally the velocity (component 0=distance, component 1=velocity); velocity is the velocity in direction 'dirSensor' and does not account for changes in geometry, thus it may be different from the time derivative of the distance!
  - `addGraphicsObject`: if True, the distance sensor is also visualized graphically in a simplified manner with a red line having the length of dirSensor; NOTE that updates are ONLY performed during computation, not in visualization; for this reason, solution.sensors.writePeriod should be accordingly small
  - `drawDisplaced`: if True, the red line is drawn backwards such that it moves along the measured surface; if False, the beam is fixed to marker or position
  - `color`: optional color for 'laser beam' to be drawn
- **output**: (type: SensorIndex) creates sensor and returns according sensor number of SensorUserFunction
- **notes**: use generalContactIndex = CreateDistanceSensorGeometry(...) before to create GeneralContact module containing geometry; old name: AddDistanceSensor(...)


Relevant Examples (Ex) and TestModels (TM) with weblink to github: [`distanceSensor.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/distanceSensor.py) (TM), [`laserScannerTest.py`](https://github.com/jgerstmayr/EXUDYN/blob/master/python/TestModels/laserScannerTest.py) (TM)

