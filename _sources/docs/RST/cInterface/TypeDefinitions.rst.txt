

.. _sec-cinterface-typedef:

****************
Type definitions
****************

This section defines a couple of structures (C++: enum aka enumeration type), which are used to select, e.g., a configuration type or a variable type. In the background, these types are integer numbers, but for safety, the types should be used as type variables. See this examples:


.. code-block:: python
   :linenos:
   
   #Conversion to integer is possible: 
   x = int(exu.OutputVariableType.Displacement)
   #also conversion from integer: 
   varType = exu.OutputVariableType(8)
   #use in settings:
   SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.StressLocal
   #use outputVariableType in sensor:
   mbs.AddSensor(SensorBody(bodyNumber=rigid, storeInternal=True,
                            outputVariableType=exu.OutputVariableType.Displacement))
   #

.. _sec-outputvariabletype:


OutputVariableType
==================

This section shows the OutputVariableType structure, which is used for selecting output values, e.g. for GetObjectOutput(...) or for selecting variables for contour plot.

Available output variables and the interpreation of the output variable can be found at the object definitions. 
The OutputVariableType does not provide information about the size of the output variable, which can be either scalar or a list (vector). For vector output quantities, the contour plot option offers an additional parameter for selection of the component of the OutputVariableType. The components are usually out of \{0,1,2\}, representing \{x,y,z\} components (e.g., of displacements, velocities, ...), or \{0,1,2,3,4,5\} representing \{xx,yy,zz,yz,xz,xy\} components (e.g., of strain or stress). In order to compute a norm, chose component=-1, which will result in the quadratic norm for other vectors and to a norm specified for stresses (if no norm is defined for an outputVariable, it does not compute anything)


\ The class **OutputVariableType** has the following **functions and structures**:

* | **\_None**:
  | no value; used, e.g., to select no output variable in contour plot
* | **Distance**:
  | e.g., measure distance in spring damper connector
* | **Position**:
  | measure 3D position, e.g., of node or body
* | **Displacement**:
  | measure displacement; usually difference between current position and reference position
* | **DisplacementLocal**:
  | measure local displacement, e.g. in local joint coordinates
* | **Velocity**:
  | measure (translational) velocity of node or object
* | **VelocityLocal**:
  | measure local (translational) velocity, e.g. in local body or joint coordinates
* | **Acceleration**:
  | measure (translational) acceleration of node or object
* | **AccelerationLocal**:
  | measure (translational) acceleration of node or object in local coordinates
* | **RotationMatrix**:
  | measure rotation matrix of rigid body node or object
* | **Rotation**:
  | measure, e.g., scalar rotation of 2D body, Euler angles of a 3D object or rotation within a joint
* | **AngularVelocity**:
  | measure angular velocity of node or object
* | **AngularVelocityLocal**:
  | measure local (body-fixed) angular velocity of node or object
* | **AngularAcceleration**:
  | measure angular acceleration of node or object
* | **AngularAccelerationLocal**:
  | measure angular acceleration of node or object in local coordinates
* | **Coordinates**:
  | measure the coordinates of a node or object; coordinates usually just contain displacements, but not the position values
* | **Coordinates\_t**:
  | measure the time derivative of coordinates (= velocity coordinates) of a node or object
* | **Coordinates\_tt**:
  | measure the second time derivative of coordinates (= acceleration coordinates) of a node or object
* | **SlidingCoordinate**:
  | measure sliding coordinate in sliding joint
* | **Director1**:
  | measure a director (e.g. of a rigid body frame), or a slope vector in local 1 or x-direction
* | **Director2**:
  | measure a director (e.g. of a rigid body frame), or a slope vector in local 2 or y-direction
* | **Director3**:
  | measure a director (e.g. of a rigid body frame), or a slope vector in local 3 or z-direction
* | **Force**:
  | measure global force, e.g., in joint or beam (resultant force), or generalized forces; see description of according object
* | **ForceLocal**:
  | measure local force, e.g., in joint or beam (resultant force)
* | **Torque**:
  | measure torque, e.g., in joint or beam (resultant couple/moment)
* | **TorqueLocal**:
  | measure local torque, e.g., in joint or beam (resultant couple/moment)
* | **StrainLocal**:
  | measure local strain, e.g., axial strain in cross section frame of beam or Green-Lagrange strain
* | **StressLocal**:
  | measure local stress, e.g., axial stress in cross section frame of beam or Second Piola-Kirchoff stress; choosing component==-1 will result in the computation of the Mises stress
* | **CurvatureLocal**:
  | measure local curvature; may be scalar or vectorial: twist and curvature of beam in cross section frame
* | **ConstraintEquation**:
  | evaluates constraint equation (=current deviation or drift of constraint equation)



.. _sec-configurationtype:


ConfigurationType
=================

This section shows the ConfigurationType structure, which is used for selecting a configuration for reading or writing information to the module. Specifically, the ConfigurationType.Current configuration is usually used at the end of a solution process, to obtain result values, or the ConfigurationType.Initial is used to set initial values for a solution process.



\ The class **ConfigurationType** has the following **functions and structures**:

* | **\_None**:
  | no configuration; usually not valid, but may be used, e.g., if no configurationType is required
* | **Initial**:
  | initial configuration prior to static or dynamic solver; is computed during mbs.Assemble() or AssembleInitializeSystemCoordinates()
* | **Current**:
  | current configuration during and at the end of the computation of a step (static or dynamic)
* | **Reference**:
  | configuration used to define deformable bodies (reference configuration for finite elements) or joints (configuration for which some joints are defined)
* | **StartOfStep**:
  | during computation, this refers to the solution at the start of the step = end of last step, to which the solver falls back if convergence fails
* | **Visualization**:
  | this is a state completely de-coupled from computation, used for visualization
* | **EndOfEnumList**:
  | this marks the end of the list, usually not important to the user



.. _sec-itemtype:


ItemType
========

This section shows the ItemType structure, which is used for defining types of indices, e.g., in render window and will be also used in item dictionaries in future.



\ The class **ItemType** has the following **functions and structures**:

* | **\_None**:
  | item has no type
* | **Node**:
  | item or index is of type Node
* | **Object**:
  | item or index is of type Object
* | **Marker**:
  | item or index is of type Marker
* | **Load**:
  | item or index is of type Load
* | **Sensor**:
  | item or index is of type Sensor



.. _sec-nodetype:


NodeType
========

This section shows the NodeType structure, which is used for defining node types for 3D rigid bodies.



\ The class **NodeType** has the following **functions and structures**:

* | **\_None**:
  | node has no type
* | **Ground**:
  | ground node
* | **Position2D**:
  | 2D position node 
* | **Orientation2D**:
  | node with 2D rotation
* | **Point2DSlope1**:
  | 2D node with 1 slope vector
* | **Position**:
  | 3D position node
* | **Orientation**:
  | 3D orientation node
* | **RigidBody**:
  | node that can be used for rigid bodies
* | **RotationEulerParameters**:
  | node with 3D orientations that are modelled with Euler parameters (unit quaternions)
* | **RotationRxyz**:
  | node with 3D orientations that are modelled with Tait-Bryan angles
* | **RotationRotationVector**:
  | node with 3D orientations that are modelled with the rotation vector
* | **LieGroupWithDirectUpdate**:
  | node to be solved with Lie group methods, without data coordinates
* | **GenericODE2**:
  | node with general ODE2 variables
* | **GenericODE1**:
  | node with general ODE1 variables
* | **GenericAE**:
  | node with general algebraic variables
* | **GenericData**:
  | node with general data variables
* | **PointSlope1**:
  | node with 1 slope vector
* | **PointSlope12**:
  | node with 2 slope vectors in x and y direction
* | **PointSlope23**:
  | node with 2 slope vectors in y and z direction



.. _sec-jointtype:


JointType
=========

This section shows the JointType structure, which is used for defining joint types, used in KinematicTree.



\ The class **JointType** has the following **functions and structures**:

* | **\_None**:
  | node has no type
* | **RevoluteX**:
  | revolute joint type with rotation around local X axis
* | **RevoluteY**:
  | revolute joint type with rotation around local Y axis
* | **RevoluteZ**:
  | revolute joint type with rotation around local Z axis
* | **PrismaticX**:
  | prismatic joint type with translation along local X axis
* | **PrismaticY**:
  | prismatic joint type with translation along local Y axis
* | **PrismaticZ**:
  | prismatic joint type with translation along local Z axis



.. _sec-dynamicsolvertype:


DynamicSolverType
=================

This section shows the DynamicSolverType structure, which is used for selecting dynamic solvers for simulation.



\ The class **DynamicSolverType** has the following **functions and structures**:

* | **GeneralizedAlpha**:
  | an implicit solver for index 3 problems; intended to be used for solving directly the index 3 constraints using the spectralRadius sufficiently small (usually 0.5 .. 1)
* | **TrapezoidalIndex2**:
  | an implicit solver for index 3 problems with index2 reduction; uses generalized alpha solver with settings for Newmark with index2 reduction
* | **ExplicitEuler**:
  | an explicit 1st order solver (generally not compatible with constraints)
* | **ExplicitMidpoint**:
  | an explicit 2nd order solver (generally not compatible with constraints)
* | **RK33**:
  | an explicit 3 stage 3rd order Runge-Kutta method, aka "Heun third order"; (generally not compatible with constraints)
* | **RK44**:
  | an explicit 4 stage 4th order Runge-Kutta method, aka "classical Runge Kutta" (generally not compatible with constraints), compatible with Lie group integration and elimination of CoordinateConstraints
* | **RK67**:
  | an explicit 7 stage 6th order Runge-Kutta method, see 'On Runge-Kutta Processes of High Order', J. C. Butcher, J. Austr Math Soc 4, (1964); can be used for very accurate (reference) solutions, but without step size control!
* | **ODE23**:
  | an explicit Runge Kutta method with automatic step size selection with 3rd order of accuracy and 2nd order error estimation, see Bogacki and Shampine, 1989; also known as ODE23 in MATLAB
* | **DOPRI5**:
  | an explicit Runge Kutta method with automatic step size selection with 5th order of accuracy and 4th order error estimation, see  Dormand and Prince, 'A Family of Embedded Runge-Kutta Formulae.', J. Comp. Appl. Math. 6, 1980
* | **DVERK6**:
  | [NOT IMPLEMENTED YET] an explicit Runge Kutta solver of 6th order with 5th order error estimation; includes adaptive step selection



.. _sec-crosssectiontype:


CrossSectionType
================

This section shows the CrossSectionType structure, which is used for defining beam cross section types.



\ The class **CrossSectionType** has the following **functions and structures**:

* | **Polygon**:
  | cross section profile defined by polygon
* | **Circular**:
  | cross section is circle or elliptic



.. _sec-keycode:


KeyCode
=======

This section shows the KeyCode structure, which is used for special key codes in keyPressUserFunction.



\ The class **KeyCode** has the following **functions and structures**:

* | **SPACE**:
  | space key
* | **ENTER**:
  | enter (return) key
* | **TAB**:


* | **BACKSPACE**:


* | **RIGHT**:
  | cursor right
* | **LEFT**:
  | cursor left
* | **DOWN**:
  | cursor down
* | **UP**:
  | cursor up
* | **F1**:
  | function key F1
* | **F2**:
  | function key F2
* | **F3**:
  | function key F3
* | **F4**:
  | function key F4
* | **F5**:
  | function key F5
* | **F6**:
  | function key F6
* | **F7**:
  | function key F7
* | **F8**:
  | function key F8
* | **F9**:
  | function key F9
* | **F10**:
  | function key F10



.. _sec-linearsolvertype:


LinearSolverType
================

This section shows the LinearSolverType structure, which is used for selecting linear solver types, which are dense or sparse solvers.



\ The class **LinearSolverType** has the following **functions and structures**:

* | **\_None**:
  | no value; used, e.g., if no solver is selected
* | **EXUdense**:
  | use dense matrices and according solvers for densly populated matrices (usually the CPU time grows cubically with the number of unknowns)
* | **EigenSparse**:
  | use sparse matrices and according solvers; additional overhead for very small multibody systems; specifically, memory allocation is performed during a factorization process
* | **EigenSparseSymmetric**:
  | use sparse matrices and according solvers; NOTE: this is the symmetric mode, which assumes symmetric system matrices; this is EXPERIMENTAL and should only be used of user knows that the system matrices are (nearly) symmetric; does not work with scaled GeneralizedAlpha matrices; does not work with constraints, as it must be symmetric positive definite
* | **EigenDense**:
  | use Eigen's LU factorization with partial pivoting (faster than EXUdense) or full pivot (if linearSolverSettings.ignoreSingularJacobian=True; is much slower)



.. _sec-contacttypeindex:


ContactTypeIndex
================

This section shows the ContactTypeIndex structure, which is in GeneralContact to select specific contact items, such as spheres, ANCFCable or triangle items.



\ The class **ContactTypeIndex** has the following **functions and structures**:

* | **IndexSpheresMarkerBased**:
  | spheres attached to markers
* | **IndexANCFCable2D**:
  | ANCFCable2D contact items
* | **IndexTrigsRigidBodyBased**:
  | triangles attached to rigid body (or rigid body marker)
* | **IndexEndOfEnumList**:
  | signals end of list


