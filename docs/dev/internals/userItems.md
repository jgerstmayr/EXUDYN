# User items: what each kind gives the solver, and how the user items are built

What an item of each kind gives the solver in C++, what a new item needs, and how the user items - ObjectUserBody,
NodeUser, MarkerBodyUser, ObjectConnectorUser, ObjectJointUser, SensorUserFunction with item numbers - are built on it. How
a user writes them is {ref}`sec-useritems`; the calls of their user functions are {doc}`userFunctions`.

## 1. What an item of each kind gives the solver

The functions of representative items, from their definitions (generated declarations plus hand-written code):

| kind | example | functions | what they are |
|---|---|---|---|
| node, position | NodePoint | 13 | coordinates (reference, initial), `GetPosition`, `GetVelocity`, `GetAcceleration`, `GetPositionJacobian`, output variables |
| node, rigid | NodeRigidBodyEP | 38 | + `GetRotationMatrix`, `GetAngularVelocity(Local)`, `GetRotationJacobian`, `GetG`, `GetGlocal`, `GetG_t`, `GetGlocal_t`, `GetGTv_q`, `GetGlocalTv_q` (derivatives for the Jacobians), the algebraic equation of the Euler parameters |
| body, point | ObjectMassPoint | 22 | `ComputeMassMatrix`, `ComputeODE2LHS`, the access functions for markers (`GetAccessFunctionTypes`, `GetPositionJacobian`, `GetMassWeightedPositionJacobian`), `GetPosition/Velocity/Displacement` at a local position, output variables |
| body, rigid | ObjectRigidBody | 36 | + rotation, angular velocity, `GetRotationJacobian`, `GetKinematicsRigid`, `AddForceTorque` (#2745) |
| marker, position | MarkerBodyPosition | 14 | `ComputeMarkerData` (value, velocity, Jacobian), `ComputeMarkerDataJacobianDerivative`, `AddGeneralizedForce` |
| marker, rigid | MarkerNodeRigid | 16 | + rotation matrix, angular velocity, `GetKinematicsRigid`, `AddGeneralizedForceTorque` |
| connector | ConnectorSpringDamper | 20 | `GetRequestedMarkerType`, `ComputeODE2LHSConnector` (from marker data), `ComputeJacobianODE2Connector`, output variables |
| constraint | JointSpherical | 21 | `GetAlgebraicEquationsSize`, `ComputeAlgebraicEquations`, `ComputeJacobianAE`, reaction forces |
| load | LoadForceVector | 10 | `GetRequestedMarkerType`, `GetLoadVector`, the body-fixed flag |
| sensor | SensorUserFunction | 12 | `GetSensorValues`, file and storage flags |

The kinds are linked by type flags: a node is `Position`, `Orientation`, `RigidBody`, ...; a marker `Position`,
`Orientation`, `Coordinate(s)`, `BodyMass`, ...; a connector or load requests marker types; a body offers access
functions (`TranslationalVelocity_qt`, `AngularVelocity_qt`, `DisplacementMassIntegral_q`), from which the markers on
it are allowed (`CSystem.cpp`, the checks of the marker types).

**What a new body needs**, in C++ or as a user item, besides its equations of motion: no new node if its coordinates
fit an existing one; no new marker if it gives the **body access functions** - position, velocity and position
Jacobian at a local position, and for rigid markers rotation, angular velocity and rotation Jacobian -, as then
MarkerBodyPosition, MarkerBodyRigid and MarkerBodyMass work, and through them every connector, joint and load; no new
load or sensor, as they act through markers and read output variables. The body access functions are the interface: a
user body with them is complete.

## 2. The user items

| item | what it is | its user functions | test models |
|---|---|---|---|
| ObjectUserBody (#2972, #2986) | a body - a finite element or a rigid body - on existing nodes | mass matrix (or `constantMassMatrix`; not needed for a static problem, #3104), force, position, position Jacobian, optionally rotation matrix and rotation Jacobian, mass integral (#3063), output variables (#3057) | userRigidBodyTest.py, userANCFCable2DTest.py, userANCFBeam3DTest.py, userTetrahedronTest.py, userMassIntegralTest.py |
| NodeUser (#2974) | a node of any number of ODE2 coordinates | position, optionally rotation matrix, and their Jacobians (central differences if not given) | userNodeTest.py |
| MarkerBodyUser (#3065, #3103) | a point of several bodies or nodes, for the built-in connectors, joints, loads and sensors | position, with `providedMarkerType` Position \| Orientation the rotation matrix, their Jacobians; no Jacobian derivative | userMarkerTest.py |
| ObjectConnectorUser (#2975, #3102) | a connector of any markers, with access to them | `forceUserFunction(mbs, t, itemNumber, markers, dataCoordinates, parameters)`, one `exudyn.ConnectorMarker` per marker, valid during the call; the data of a NodeGenericData and a `postNewtonStepUserFunction`; its Jacobian numerical | userConnectorTest.py, userConnectorContactTest.py |
| ObjectJointUser (#2976, #3101) | a joint on two rigid markers | `constraintUserFunction` of 1 to 6 equations of the marker positions and rotations and its `jacobianUserFunction`, both required | userJointTest.py |
| SensorUserFunction with `itemNumbers` (#2977) | a sensor of any items | reads the items through `mbs` | sensorUserFunctionItemsTest.py |

No user load (the loads have user functions for their value and act through markers) and no user constraint beyond
ObjectJointUser and ObjectConnectorCoordinateVector. A user item is an item with user function parameters, not a Python
class derived from a C++ class: items stay validated dicts, the functions can be symbolic, pickling and `GetObject`
work as for every item.

**User types** (#2989): OutputVariableType, Node::Type and Marker::Type have `UserType0` to `UserType3` (bits 36-39,
19-22 and 23-26). `exu.special.userTypes.SetName(exu.OutputVariableType.UserType0, 'BendingStrain')` keeps the name in
`src/Main/UserTypes.h` (without pybind11), which the generated string functions write - the header of a sensor file,
the labels of PlotSensor, the messages - and sets it as an alias attribute of the enum class. ObjectUserBody provides
the four user output variables when `outputVariableUserFunction` is given, a conditional output variable of its
definition (`ItemOutputVariable(..., condition=...)`).

## 3. The common pattern

- **Names**: a user function is a parameter `<what>UserFunction`; its def in the definition file carries the
  annotations, from which the `std::function`, the Python Protocol and the argument table of the page are generated.
  Functions with the same signature and meaning share one def.
- **Arguments**: `mbs, t, itemNumber`, then what the function is about (`localPosition`, `outputVariableType`), then
  `qRef, qTotal, q_t` - the reference coordinates, the total coordinates of the configuration asked for (reference plus
  displacement, #3062) and the velocities -, `q_tt` where accelerations are needed, and last `parameters`, the parameter
  vector of the item (#3056). All as read-only numpy arrays; in the reference configuration `qTotal` is `qRef` and the
  velocities are zero; in the visualization, those of the visualization configuration.
- **Results**: a vector or matrix as list or numpy array; a matrix of fixed meaning is checked for its size, a wrong
  size raises a ValueError with the name of the function (`CheckResultSize`).
- **C++** computes what it can from the user functions: velocities from the Jacobians, displacements from the
  reference configuration, the derivative of the projected forces by central differences, the Jacobian of the
  equations numerically. A function that is needed and not given is reported with PyError.
- **Threads**: an item whose user functions are all symbolic is computed in the threads of the parallel loops, one
  with a Python function in the main thread, and so is what reads it (#2978, {ref}`sec-dev-userfunctions-threads`).
  The buffers of the access functions are `static thread_local`. The render thread does not call Python: the item, its
  markers and what depends on them are drawn in the main thread with the graphics user functions
  (`AccessFunctionsCallUserFunctions`, #3003).
- **Output variables** the item provides only with a user function are conditional in the definition.

## 4. Parameters (#3056)

ObjectUserBody and NodeUser have a vector `parameters` of any length, the last argument of all their user functions,
as the stiffness and damping follow the kinematics in the existing user functions. What its entries mean, the user
functions define. `exudyn.advancedUtilities.ParameterLayout([('mass', 1), ('inertia', 6)])` maps it to names:
`ToVector(dict)` checks names and sizes, `ToDict(vector)` works with numpy arrays and symbolic vectors alike. The
layout is not an item parameter: it belongs to the kind of user element, not to the instance, and C++ never calls it.

- **One recording for all items of a kind**: while recording, `parameters` is a named symbolic vector as `q` is, and
  `ToDict` slices it into components - leaves of the recording that read each item's vector at each call. So one
  recording serves every element of a user beam with its own cross section, and `SetObjectParameter(item,
  'parameters', ...)` acts at once (userParametersTest.py).
- **The limit**: a parameter that changes the structure of the computation - the number of Gauss points, the size of a
  matrix, a Python `if` - has no value while recording; it stays a constant of the Python code, and a structural
  variant is a recording of its own.

## 5. The interfaces, and what they leave out (#3056, #3057, #3058, #3062, #3063, #3064)

- **Visualization**: VObjectUserBody keeps `graphicsData` in local coordinates, and every point is mapped with the
  positionUserFunction in the visualization configuration (`EXUvis::AddBodyGraphicsDataMapped`), so a symbolic
  positionUserFunction draws a deformed mesh without Python; the normals of triangles are computed from the mapped
  corners. The graphicsDataUserFunction, in global coordinates, draws anything else; it reads the visualization
  parameters of its item through `mbs.GetObjectParameter(itemNumber, 'V...')` - there is no visualization argument.
- **Against a rigid body or a beam**, the user items have, or lack:

| feature | user items | how it could come, without changing the interfaces |
|---|---|---|
| beam outputs, PotentialEnergy, accelerations, Rotation, directors | through the outputVariableUserFunction, with `q_tt` (#3057) | Acceleration, AngularAcceleration, Rotation, VelocityLocal in C++ from the Jacobians |
| a constant mass matrix | `constantMassMatrix`: computed once and stored (#3058) | - |
| mass integral access, MarkerBodyMass, LoadMassProportional | `massIntegralUserFunction` for `∫ρ J_pos dV`, and `centerOfMass` (#3063) | - |
| an analytic Jacobian | numerical only | an optional jacobianUserFunction, or from the recording |
| algebraic equations, data coordinates, ODE1 | none | through MainSystem functions, later also for symbolic functions |
| drawing with contour colors and tiling | no | the C++ of the item colors the mapped points by its output variables |
| cable markers, GeneralContact on cables | refused: they name the C++ classes | a general shape marker for curves, surfaces and volumes (#2980) |
| node types | NodeUser is always Position and Orientation | the type from the given user functions, the user types |

## 6. The cost

**A call into Python**, `tools/benchmarks/measureUserFunctions.py`: a chain of 100 mass points on coordinate
springs, 2000 steps of explicit RK44, Windows, Python 3.13:

| variant | per step and spring | per call, more than C++ |
|---|---|---|
| C++ spring | 1.00 us | - |
| Python `springForceUserFunction` (floats in, float out) | 1.78 us | 0.16 us |
| symbolic user function | 1.20 us | 0.04 us |
| one ObjectGenericODE2 per mass point | 4.58 us | 0.72 us |
| one ObjectUserBody per mass point, Python force, stored mass matrix | 5.85 us | 0.97 us |
| the same, one symbolic force recorded for all bodies | 1.26 us | 0.06 us |

A call into Python costs 0.2 to 0.7 us, a Python user item of a few vectors about 1 to 2 us per evaluation: a model of
a few thousand Python user items runs, one of millions does not - that is what the symbolic functions and the compiled
items (RG8) are for. A Jacobian by numerical differentiation costs n+1 evaluations per item and Newton step. In 4
threads, 200 user rigid bodies with symbolic functions take 0.31 s instead of 0.77 s with one (#2978).

**The code**: per user function, its declaration in the definition file, one wrapper of about 8 lines in
evaluateUserFunctions.cpp and the call in the item; the std::function, its template instantiation and the symbolic
transfer are generated. ObjectUserBody with its user functions is about 520 hand-written lines of C++ and 300 lines of
definition; the user ANCF cable, beam and tetrahedron needed no C++ at all.

## 7. Decisions

| decision | why |
|---|---|
| the user body is a finite element on existing nodes, with the body access functions (maintainer 2026-10-11) | every marker, connector, load and sensor works on it at once; ObjectGenericODE2 is a superelement, reached only at its mesh nodes |
| the user connector gets its markers as Python objects per call (2026-10-12) | arrays of fixed marker quantities cannot reach a shape marker at varying local coordinates |
| four user types per enumeration (2026-10-12) | enough to start; beyond them a user item reuses a type |
| `qRef, qTotal` instead of displacements (2026-10-22, #3062) | nearly every user body needed the total coordinates; the C++ may move to total coordinates (RG13) |
| `parameters` a vector, its names in a ParameterLayout of the kind (#3056) | one recording serves every item of a kind |
| no visualization argument; graphics data mapped by the positionUserFunction (2026-10-22, #3064) | the graphics function reaches its item through `mbs`; a symbolic body is drawn without Python |
| algebraic equations, data coordinates, ODE1 not now (2026-10-22) | later through MainSystem functions |
