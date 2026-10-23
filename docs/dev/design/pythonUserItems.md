# Python user items: proposal and evaluation

The design of the Python user items (revision2026c group RG7, #2971), written 2026-10-11 and revised with the
maintainer's decisions of 2026-10-11, 2026-10-12, 2026-10-21 and 2026-10-22. Measured from `definitions/itemDefs*.py` (the declared types and
functions of every item) and with [measureUserFunctions.py](measureUserFunctions.py), next to this document (the cost
of a call into Python). The steps are in the plan, group RG7.

## 1. The goal

- **Python user items**: item types whose behaviour a user writes in Python, through a small, fixed interface per
  kind of item. They come **before** the compiled C++ user items (RG8), and they define them: the Python interface of
  a user connector is the interface a compiled connector has to implement.
- **The documentation**: a new section that explains how each kind of item works conceptually - what a node, body,
  marker, connector, constraint, load and sensor give to and take from the solver - and how a user item of that kind
  is set up, with one example per kind.

## 2. What exists

| kind | generic or user part that exists | what it covers |
|---|---|---|
| node | `NodeGenericODE2`, `NodeGenericODE1`, `NodeGenericAE`, `NodeGenericData` | coordinates only: no position, no orientation (types `GenericODE2`, ...); markers can only take its coordinates |
| object (body) | `ObjectGenericODE2`: `forceUserFunction`, `massMatrixUserFunction`, `jacobianUserFunction`; `ObjectGenericODE1`: `rhsUserFunction` | the equations of motion of any coordinates; markers reach it as a superelement (mesh nodes) or through its coordinates (`MarkerObjectODE2Coordinates`) |
| connector | `springForceUserFunction` or `springTorqueUserFunction` of 6 spring-dampers; `springForceTorqueUserFunction` of RigidBodySpringDamper | the force law of a given connector, not its kinematics: the markers and the force direction stay those of the item |
| constraint | `ObjectConnectorCoordinateVector`: `constraintUserFunction`, `jacobianUserFunction` | any equations of the coordinates of two `Coordinates` markers; `offsetUserFunction` of CoordinateConstraint and JointGeneric |
| load | `loadVectorUserFunction` / `loadUserFunction` of all 4 loads | the value of the load over time |
| sensor | `SensorUserFunction` | a value computed from other sensors |
| other | `graphicsDataUserFunction` (Ground, RigidBody, RigidBody2D, GenericODE2), `PreStepUserFunction`, `PostNewtonUserFunction`, `SystemJacobianUserFunction` | drawing; the solver loop |

Altogether 16 nodes, 18 markers, 4 loads, 8 sensors, 51 objects; 34 user-function parameters in 24 items.

## 3. What an item of each kind gives the solver (C++)

The functions of representative items, from their definitions (generated declarations plus hand-written code):

| kind | example | functions | what they are |
|---|---|---|---|
| node, position | NodePoint | 13 | coordinates (reference, initial), `GetPosition`, `GetVelocity`, `GetAcceleration`, `GetPositionJacobian`, output variables |
| node, rigid | NodeRigidBodyEP | 38 | + `GetRotationMatrix`, `GetAngularVelocity(Local)`, `GetRotationJacobian`, `GetG`, `GetGlocal`, `GetG_t`, `GetGlocal_t`, `GetGTv_q`, `GetGlocalTv_q` (derivatives for the Jacobians), algebraic equation of the Euler parameters |
| body, point | ObjectMassPoint | 22 | `ComputeMassMatrix`, `ComputeODE2LHS`, the ACCESS FUNCTIONS for markers (`GetAccessFunctionTypes`, `GetPositionJacobian`, `GetMassWeightedPositionJacobian`), `GetPosition/Velocity/Displacement` at a local position, output variables |
| body, rigid | ObjectRigidBody | 36 | + rotation, angular velocity, `GetRotationJacobian`, `GetKinematicsRigid`, `AddForceTorque` (#2745) |
| marker, position | MarkerBodyPosition | 14 | `ComputeMarkerData` (value, velocity, Jacobian), `ComputeMarkerDataJacobianDerivative`, `AddGeneralizedForce` |
| marker, rigid | MarkerNodeRigid | 16 | + rotation matrix, angular velocity, `GetKinematicsRigid`, `AddGeneralizedForceTorque` |
| connector | ConnectorSpringDamper | 20 | `GetRequestedMarkerType`, `ComputeODE2LHSConnector` (from marker data), `ComputeJacobianODE2Connector`, output variables |
| constraint | JointSpherical | 21 | `GetAlgebraicEquationsSize`, `ComputeAlgebraicEquations`, `ComputeJacobianAE`, reaction forces |
| load | LoadForceVector | 10 | `GetRequestedMarkerType`, `GetLoadVector`, body-fixed flag |
| sensor | SensorUserFunction | 12 | `GetSensorValues`, file and storage flags |

The links between the kinds are declared by type flags: a node is `Position`, `Orientation`, `RigidBody`, ...; a
marker `Position`, `Orientation`, `Coordinate(s)`, `BodyMass`, ...; a connector or load requests marker types; a body
offers access functions (`TranslationalVelocity_qt`, `AngularVelocity_qt`, `DisplacementMassIntegral_q`), from which the
markers on it are allowed (`CSystem.cpp:292-355`).

## 4. What a totally new object needs

If a new kind of body is added - in C++ or as a Python user item - it needs, besides its equations of motion:

- **nodes**: none new if its coordinates fit an existing node (NodeGenericODE2 for any coordinates; position or rigid
  nodes if markers shall attach to them directly). A new node only for new kinematics on the node level.
- **markers**: none new if it provides the access functions of a body - position, velocity and position Jacobian at a
  local position, and for rigid markers rotation, angular velocity and rotation Jacobian. Then MarkerBodyPosition,
  MarkerBodyRigid and MarkerBodyMass work, and through them every connector, joint and load. A new marker only for a
  new kind of attachment (a beam's shape, a surface).
- **loads**: none: loads act through markers.
- **sensors**: none: SensorBody reads the output variables of the body, SensorMarker the markers.
- **graphics, output variables, documentation, a mini example, a test model**.

So the BODY ACCESS FUNCTIONS are the key interface: a user body with them is complete.

## 5. The user items, with the maintainer's decisions (2026-10-11, 2026-10-12)

### 5.1 User body: a finite element first, a rigid body as a check

ObjectGenericODE2 is a SUPERELEMENT: markers reach it only at its mesh nodes (MarkerSuperElementPosition/Rigid), not at
a local position, so it cannot be measured or loaded where a body can. The user body is a new item, **treated as a
finite element**: it takes existing nodes - position nodes, rigid nodes, slope nodes - and gives the body interface,
so MarkerBodyPosition/Rigid/Mass, SensorBody, the loads and every connector work on it at once:

| user function | what it returns | needed for |
|---|---|---|
| `massMatrix(mbs, t, itemNumber, q)` | M (n x n) | every solver; may be constant (flag) |
| `force(mbs, t, itemNumber, q, q_t)` | the generalized forces, `ODE2LHS` | every solver |
| `jacobian(...)`, optional | dF/dq, dF/dq_t | implicit solvers; else numerical |
| `position(mbs, itemNumber, q, localPosition)`, `velocity(... q_t ...)` | position and velocity of a material point | position markers, sensors, loads, drawing |
| `positionJacobian(mbs, itemNumber, q, localPosition)` | d p / d q (3 x n) | position markers: forces into the coordinates |
| optional `rotationMatrix`, `angularVelocityLocal`, `rotationJacobian` at a local position | | rigid markers (beams, rigid bodies) |
| optional `outputVariable(mbs, itemNumber, type, localPosition, q, q_t)` | strains, stresses, ... with the user types of 5.6 | SensorBody, contour colours |

A **user rigid body**, a replica of ObjectRigidBody on a rigid node, is the first check: it gives the same results
as the built-in one. Most users will write finite elements; the user body is the central item of the extension.

### 5.2 User nodes: second

ANCF elements need special nodes (slopes); with a user node and the existing position/rigid interfaces of nodes and
bodies, an element can be written at once. Functions: the number of ODE2 coordinates, `position(q)`,
`velocity(q, q_t)`, `positionJacobian(q)`; for orientation `rotationMatrix(q)`, `angularVelocityLocal(q, q_t)`,
`Glocal(q)`, and the derivatives `Glocal_t`, `d(Glocal^T v)/dq`, numerically if not given. The existing
NodePoint2DSlope1 and NodePointSlope1/12/23 show the pattern; NodePointSlope123 (full gradients, 12 coordinates) does
not exist - a natural first user node (example 3 in section 6).

### 5.3 Markers, and a step for the C++ side

**The user markers are open** (#3065): ObjectConnectorUser reaches any point of a body through its markers, and
ObjectUserBody defines what a local position means, so a user marker on one body adds little; a marker on several nodes
or bodies needs the coordinates of several items in the LTG of the marker - to decide before it is built.

Contact and sliding on flexible bodies need markers that can move along the body: MarkerBodyCable2DShape and
MarkerBodyCable2DCoordinates serve ObjectANCFCable2D only. **Proposed new step (C++, outside RG7, e.g. in RG9)**:
revise the Cable2D markers into a general form for curves (beams), surfaces (plates, shells) and volumes - a marker that
gives the shape of a body, positions at varying local coordinates and their Jacobians - so that contact and sliding
joints work on any flexible body. The user markers (position, orientation, coordinates; on a node or a body) follow the
same interface: value, velocity, Jacobian with respect to the coordinates.

### 5.4 Connector: a user connector that calls the marker functions

**Built** (#2975): `ObjectConnectorUser` with any number of markers of any type; `forceUserFunction(mbs, t, itemNumber,
markers, parameters)` gets one `exudyn.ConnectorMarker` per marker, owned by Python and invalidated after the call. It
reads the marker data (position, velocity, rotation, angular velocity, coordinates) and the marker's body at any local
position, and applies `ApplyForce`, `ApplyTorque`, `ApplyBodyForce(force, localPosition)` and `ApplyCoordinateForces`
through the Jacobians, into the block of the marker in the generalized forces; the Jacobian of the connector is
numerical. userConnectorTest.py: a Cartesian spring-damper and a rigid-body damper equal to the built-in ones, a ball on
an ANCF cable with the closest point searched on every element. The design as written before it was built:

A force-torque function of the relative motion exists already (`springForceTorqueUserFunction` of
ObjectConnectorRigidBodySpringDamper). The new user connector instead gets ACCESS TO ITS MARKERS: the user function
receives, per marker, an object with the functions of the marker - position and velocity (at the marker, or for a
shape marker at a local coordinate), rotation, angular velocity, the Jacobians - and returns the generalized forces
(e.g. `ApplyForce(marker, force)`, `ApplyTorque(marker, torque)`, or the force vectors per marker), optionally the
Jacobian. More overhead per call, but it can do what the data of a marker allow: search the closest point on a cable,
slide along a curve, contact with friction on a surface. Decided 2026-10-12: Python objects per call - a fixed set of
marker quantities passed as arrays cannot reach shape markers at varying local coordinates.

### 5.5 Parallel computation

Items with Python user functions are computed serially (one Python call at a time, the GIL), as the existing user
functions are. Items whose user functions are all symbolic (`exudyn.symbolic`) are evaluated in C++ and can run in
parallel and be drawn directly by the graphics. The symbolic user functions have an awkward interface - the first
design, before some symbolic methods existed - and are revised first in a group of their own (RG12: status, user
function interfaces, dense linear algebra, access to the MainSystem and the items, tests); the symbolic user items
(RG7.8) wait for it.

### 5.6 User types: output variables, node and marker types

A new item needs types of its own: an output variable (a strain of a new element), a node type (the slopes of a user
node), a marker type. The enumerations are bit sets; each has four user types (maintainer 2026-10-12: four are enough
to start; beyond them, a user item reuses an existing type), declared like any value in `definitions/outputVariableTypes.py`
and `definitions/enumTypes.py` (#2989):

| enumeration | built-in bits | UserType0-3 |
|---|---|---|
| OutputVariableType (64 bits) | 0-35 | 36-39 |
| Node::Type (32 bits) | 0-18 | 19-22 |
| Marker::Type (32 bits) | 0-21 | 23-26 (EndOfEnumList 27; AccessFunctionType shares the bits and uses 29-31) |

- **The name**: a pybind enum value cannot be renamed; `exu.special.userTypes.SetName(exu.OutputVariableType.UserType0,
  'BendingStrain')` keeps the name in `src/Main/UserTypes.h` (header only, without pybind11) and
  - the generated string functions (`GetOutputVariableTypeString`, `Node::GetTypeString`, `Marker::GetTypeString`)
    write it, so the header of a sensor file - and so the labels of `PlotSensor` - and the messages say BendingStrain;
  - it is set as an attribute of the enum class, `exu.OutputVariableType.BendingStrain`, the same value as UserType0;
    a dictionary or a sensor given that value has the user type;
  - `str()` of the value writes it (the class gets its own `__str__` the first time a name is set); `.name` stays
    UserType0;
  - a new name replaces the previous one and its alias; a name that is not a Python name or already a name of the
    enumeration is refused; `Reset()` removes all, `GetName()` reads one.
- Which items provide or accept a user type is declared by the user item: ObjectUserBody provides the four user output
  variables when `outputVariableUserFunction` is given (a conditional output variable of the definition,
  `ItemOutputVariable(..., condition=...)`); a user node or marker will declare its node or marker type, so the checks of
  CSystem work unchanged.

### 5.7 User sensor: items as well as sensors

SensorUserFunction takes `sensorNumbers` only, so it reaches other items only through code. Proposal: a second array
`itemNumbers` (one of the two must be empty); the non-empty one is passed to the user function as a list of ints
(`mbs, t, numbers, factors, configuration`), and the user function reads what it needs from the items via mbs - which
is why every user function gets mbs. Which kind of item the numbers are is the user's to know.

### 5.8 Not needed, or existing

- **user load**: not needed - loads have user functions for their value and act through markers; a user marker gives
  new points of action.
- **user constraint**: ObjectConnectorCoordinateVector exists for `Coordinates` markers; the user JOINT extends it to
  position and orientation markers: `constraint(...)` with n equations of the marker quantities, `jacobian(...)`,
  reaction forces; C++ applies the chain rule with the marker Jacobians.

**How a user item is written**: as an item with user-function parameters, like every existing user function, not as a
Python class derived from a C++ class: items stay validated dicts (the C++/Python split), the functions can be symbolic
(5.5), pickling and `GetObject` work as for any item.

### 5.9 The common pattern of a user item (as built with ObjectUserBody, #2972, #2986)

- **Names**: a user function is a parameter `<what>UserFunction` (`massMatrixUserFunction`, `positionUserFunction`);
  its def in the definition file carries the annotations, from which the std::function, the Python Protocol and the
  argument table of the page are derived. Functions with the same signature and meaning share one def.
- **Arguments**: `mbs, t, itemNumber` first, always - the time of the configuration asked for, the number of the item;
  then what the function is about (`localPosition`, `outputVariableType`); then `qRef, qTotal, q_t` - the reference
  coordinates, the total coordinates (reference plus displacement coordinates, #3062) and the velocities of the
  configuration asked for -,
  `q_tt` where accelerations are needed (outputVariableUserFunction), and last `parameters`, the parameter vector of
  the item (#3056). All of them as read-only numpy arrays. What else a function needs it reads through `mbs` and
  `itemNumber`; a symbolic function will get such access through MainSystem functions (RG12.4).
- **Returns**: a vector or a matrix as list or numpy array; a matrix of fixed meaning is checked for its size, and a
  wrong size raises a ValueError with the name of the function.
- **C++**: the item computes what it can from the user functions - velocities from the Jacobians, displacements from the
  reference configuration, the derivative of the projected forces by central differences, the Jacobian of the
  equations numerically. Each call is a wrapper in `src/System/evaluateUserFunctions.cpp` inside
  `UserFunctionExceptionHandling`; a function that is needed and not given is reported with PyError, which also runs in
  the fast module; the item number the user functions get is the object's own number, `CObject::GetObjectNumber`
  (#2995, #3012).
- **Threads**: `HasUserFunction` is true, so the item is computed serially. The renderer thread does not call Python:
  the body, its markers and what depends on them are drawn in the main thread with the graphics user functions
  (`AccessFunctionsCallUserFunctions`, #3003).
- **Output variables** the item provides only with a user function are conditional in the definition.

### 5.10 Parameters of a user item (#3056)

ObjectUserBody and NodeUser have a parameter vector `parameters` of any length (default empty), passed as the last
argument of all their user functions - after the coordinates, as the stiffness and damping follow the kinematics in
the existing user functions (`springForceUserFunction(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping,
force)`). What the entries mean - masses, lengths, stiffnesses, also flags and indices as numbers - the user functions
define.

**The mapping to named quantities** is the Python helper `exudyn.advancedUtilities.ParameterLayout`, a list of named
entries and their sizes, e.g. `ParameterLayout([('mass', 1), ('inertia', 6), ('com', 3)])`: `ToVector(dict)` checks
names and sizes, `ToDict(vector)` works with numpy arrays and symbolic vectors alike and gives a number for size 1 and
a list otherwise. The user defines the layout next to the user functions, uses `ToVector` when adding items and
`ToDict` inside the functions. It is not an item parameter: it would be a Python function that C++ never calls,
stored in the item's dict, in `GetDictionary` and in a saved system, and every item of a kind would hold it again; the
mapping belongs to the **kind** of user element, not to the instance.

**Symbolic functions keep one recording for all items of a kind.** While recording, `parameters` is a named symbolic
vector, as `q` is; `ToDict` slices it into components - `d['mass']` is `parameters[0]`, `d['com']` is
`[parameters[7], parameters[8], parameters[9]]` -, leaves of the recording that read the item's vector at each call:
written inline, as `q[1]`, a read with an index check and no arithmetic. So one recording serves every element of a
user beam with its own cross section and length, and `SetObjectParameter(item, 'parameters', ...)` acts at once,
without recording again (userParametersTest.py). **Python functions** get the vector as a read-only numpy view; `ToDict`
costs a Python call there, so a function that must be fast indexes the vector directly.

**The one limit.** A parameter that changes the **structure** of the computation - the number of Gauss points, the size
of a matrix, a Python `if` - cannot come from the vector of a symbolic function: while recording it has no value
(`int(parameters[3])` raises, #3017). Such values stay constants of the Python code, and a structural variant is a
recording of its own. Bools and indices as doubles are fine where they enter the arithmetic or an `IfThenElse`.

**The cost** of the arguments, measured with measureUserFunctions.py (section 7): a Python force function of
ObjectUserBody, `(mbs, t, itemNumber, qRef, qTotal, q_t, parameters)`, costs 0.33 us per call more than the force function
of ObjectGenericODE2, `(mbs, t, itemNumber, q, q_t)` - two more numpy views and the gathering of the coordinates by the
item; as a symbolic function it costs 0.06 us per call more than C++.

**Later**, if the dialogs shall show named parameters: a layout registered on the Python side for a kind of user
element, read by the dialog.

### 5.11 Interfaces that last: configurations, visualization, what is missing (#3056, #3057, #3058, #3062)

The interfaces of the user functions are chosen so that what comes later does not change them; built and decided with
the maintainer on 2026-10-21 and 2026-10-22, compared with ObjectRigidBody, ObjectANCFCable2D and the geometrically
exact beams.

**A. The configurations in the arguments.** Every user function gets `(mbs, t, itemNumber, [what the function is
about], qRef, qTotal, q_t, parameters)`, the outputVariableUserFunction also `q_tt` before `parameters`. `qTotal` is the
total coordinates of the configuration asked for - reference plus displacement coordinates -, `qRef` the reference
coordinates of the nodes; in the reference configuration `qTotal` is `qRef` and `q_t`, `q_tt` are zero. A rigid body
computes with `qTotal`, a finite element its displacements as `qTotal - qRef` and its reference geometry from `qRef`. The
name says which coordinates they are; a function with long expressions writes `q = qTotal` first. Total coordinates
(maintainer 2026-10-22, #3062, revising #3060): nearly all user bodies so far needed them and computed `qRef + q`, and
the C++ may move to total coordinates for performance (RG13). In the visualization, `qTotal` and `q_t` are those of
the visualization configuration. The reference geometry need not be carried in Python next to the item, and node
numbers are not needed in the parameters: the item has them, and `itemNumber` reaches them.

**B. Parameters of the visualization.** What a drawing needs - the subdivision of a mesh, the cross section drawn,
colors - differs from what the mechanics needs. There is no `visualizationParameters` argument (maintainer 2026-10-22):
the graphicsDataUserFunction, a Python function as it returns graphics data, reads the visualization parameters of its
item through `mbs.GetObjectParameter(itemNumber, 'V...')`, as it reads `'Vshow'`; a vector of visualization
parameters, when it comes, is a parameter of VObjectUserBody reached the same way, and a later mapping function (C)
gets a signature of its own.

**C. Graphics data mapped by the positionUserFunction (#3064).** VObjectUserBody keeps `graphicsData` in local
coordinates, as ObjectRigidBody does, and every point is mapped with the positionUserFunction in the visualization
configuration - which IS the map from a local point to its global position, for a rigid body and for an element whose
local position covers its volume (the 3D ANCF beam, the tetrahedron); C++ loops over the points
(`EXUvis::AddBodyGraphicsDataMapped`), so a symbolic positionUserFunction draws a deformed mesh without Python. The
normals of triangles are computed from the mapped corners, the radii of spheres and circles are kept; `show` switches
the body off. The graphicsDataUserFunction, `(mbs, t, itemNumber, qRef, qTotal, q_t, parameters)` of the visualization
configuration and in global coordinates, keeps its place for anything else; both are drawn. **Later**, where the drawing
map differs from the mechanical one - the 2D cable, whose position covers the axis only, or a scaled deformation - a
separate mapping user function with the signature of the positionUserFunction; and contour colors (D).

**D. Visualization settings in symbolic functions.** A Python function reads every setting through `mbs`; a symbolic
function cannot. Most of what the built-in items do with the settings is done by their C++: contour colors
(`contour.rigidBodiesColored`, `contour.outputVariable` per drawn point of a beam), the subdivision
(`bodies.beams.axialTiling`), numbers (`showNumbers`). With C, the C++ of the user item can do the same - color the
mapped points by `GetOutputVariableBody(contour.outputVariable, localPosition)`, which the user body provides from its
user functions -, so that a symbolic function needs no setting. The graphicsDataUserFunction stays a Python function
that reads the settings through `mbs` (maintainer 2026-10-22); access of symbolic functions to the visualization
settings is a path for later, different from that function, and has no issue.

**E. What ObjectUserBody and NodeUser have, and lack, against a rigid body or a beam:**

| feature | built-ins | user items | how it could come, without changing the interfaces |
|---|---|---|---|
| beam outputs (strain, curvature, stress, force, torque), PotentialEnergy, accelerations, Rotation, directors | beams, rigid body, ANCF | through the outputVariableUserFunction, with `q_tt`: every output variable the body does not compute itself (#3057) | - |
| Acceleration(Local), AngularAcceleration(Local), Rotation, VelocityLocal computed by the item | rigid body, beams | only through the outputVariableUserFunction | in C++ from the Jacobians: `J q_tt` plus the derivative of `J` along `q_t` by differences; Rotation from the rotation matrix |
| constant mass matrix | ANCF, beams (and rigid body with rotation vector) | `constantMassMatrix`: computed once and stored (#3058) | - |
| mass integral access (`DisplacementMassIntegral_q`): `MarkerBodyMass`, `LoadMassProportional` (gravity) | all four | `massIntegralUserFunction` for `∫ρ J_pos dV` and `centerOfMass`, where the marker is drawn (#3063) | - |
| analytic Jacobian | ANCF, beams | numerical only | an optional jacobianUserFunction, later from the symbolic recording |
| algebraic equations, data coordinates (plasticity), ODE1 | rigid body (EP in the node), some elements | none | not now (maintainer 2026-10-22); data and further quantities through MainSystem functions, later also for symbolic functions |
| drawing with contour, tiling, numbers | all | `graphicsData` mapped by the positionUserFunction (#3064), numbers; no contour, no tiling | C and D |
| cable markers (MarkerBodyCable2DShape, ...), GeneralContact cables | ANCF only, by type name | refused | out of scope; these name the C++ classes |
| node types | RigidBody, Point2DSlope1, ... | NodeUser always Position+Orientation, also without rotation | the type from the given user functions, and the user types of #2989 |

**F. The interfaces as they are fixed.** Before the user items reach a release: (1) `qRef, qTotal, q_t, parameters` in
every user function, `qTotal` the total coordinates (A, 5.10; #3056, #3062); (2) `q_tt` in the outputVariableUserFunction (#3057);
(3) no `visualizationParameters` argument - the graphics function reaches the item through `mbs`, a later mapping
function gets a signature of its own (B, C, D); (4) data coordinates, algebraic equations and ODE1 not now, later
through MainSystem functions (E); (5) everything else in E as optional user functions or flags, added without touching
the existing signatures.

## 6. Examples that go beyond the present user functions

Each is a test model AND a check of the concept: it needs the new interface, and it has a result to compare with.

| # | example | user items | what it checks | compared with |
|---|---|---|---|---|
| 1 | rigid body as a user body | user body (rigid interface) | the body interface: mass, forces, rigid markers, SensorBody, loads | ObjectRigidBody, to rounding |
| 2 | ANCF cable 2D as a user finite element, MINIMAL: one integration rule, no extensions, few output variables - as much as shows the problems (maintainer 2026-10-12) | user body on NodePoint2DSlope1 | position and positionJacobian at a local position, MarkerBodyPosition anywhere, a load at mid-span | ObjectANCFCable2D: static deflection, eigenfrequencies |
| 3 | fully parametrized ANCF beam (12 coordinates per node) | user node NodePointSlope123, user body | a new node type, a node user type (5.6), rigid markers on a flexible body | analytic deflection; convergence |
| 4 | linear tetrahedral element in 3D | user body on NodePoint | a solid finite element; a user output variable 'Stress' (5.6) with SensorBody | FEM.py / ObjectFFRF result, analytic bar |
| 5 | sphere rolling on a flexible cable, closest point searched along the cable | user connector with marker access (5.4), shape marker | a connector calling marker functions at varying local coordinates | ObjectContactCircleCable2D in 2D |
| 6 | screw joint (rotation coupled to translation) between two rigid bodies | user joint | equations of marker positions and rotations, reaction forces | analytic: pitch times angle |
| 7 | kinetic energy of a group of bodies | user sensor with itemNumbers (5.7) | items reached via mbs | the sum of SensorBody outputs |
| 8 | examples 1 and 2 with symbolic user functions | symbolic | parallel evaluation and speed (5.5) | the Python version, results and time |

## 7. Measurement: the cost of a call into Python

`measureUserFunctions.py`: a chain of 100 mass points on coordinate springs, 2000 steps of explicit RK44 (5 evaluations
per step, no Jacobian), Windows, Python 3.13, 1.13.63.dev1:

| variant | time | per step and spring | per call, more than C++ |
|---|---|---|---|
| C++ spring | 0.199 s | 1.00 us | - |
| Python `springForceUserFunction` (floats in, float out) | 0.357 s | 1.78 us | **0.16 us** |
| symbolic user function | 0.241 s | 1.20 us | 0.04 us |
| one ObjectGenericODE2 per mass point (numpy array of 1 in and out) | 0.917 s | 4.58 us | **0.72 us** (2 allocations per call) |
| one ObjectGenericODE2 for all 100 coordinates (`-K @ q`) | 0.232 s | 1.16 us | 23 us per call for 100 coordinates |
| one ObjectUserBody per mass point, Python force `(mbs, t, itemNumber, qRef, qTotal, q_t, parameters)`, stored mass matrix (1.13.166.dev1) | 1.17 s | 5.85 us | 0.97 us; 0.33 us more than ObjectGenericODE2 per mass point in the same run |
| the same, one symbolic force recorded for all bodies | 0.251 s | 1.26 us | 0.06 us |

Conclusions:
- A call into Python costs 0.2-0.7 us, more with numpy arrays; a Python user item of a few vectors costs about
  1-2 us per evaluation. A model of a few thousand Python user items runs, a model of millions does not - that is what
  the compiled items (RG8) and the symbolic user functions are for.
- A Jacobian by numerical differentiation costs (n+1) evaluations per item and Newton step; for implicit solvers an
  analytic Jacobian function (or a symbolic user function) matters more than the call itself.
- Symbolic user functions are as fast as C++ within 20 %: where a user item's functions can be symbolic, it should
  accept them.

### 7.1 The code a user function takes

Checked 2026-10-12 (maintainer: user functions affect a lot of code; the user items must stay possible to abandon if
the overhead grows too large). Most of it is generated:
- `UserFunction` occurs 1205 times in 54 hand-written files of src/ and 632 times in 73 generated ones; 23 distinct
  function signatures are instantiated (`PythonUserFunctionsTemplates.h`, generated), and the symbolic transfer of every
  user function is generated as well (`PySymbolicUserFunctionSet.h`, 337 lines).
- Per user function, written by hand: its declaration with signature and documentation in the definition file, one
  evaluation wrapper of about 8 lines (`src/System/evaluateUserFunctions.cpp`: the call inside
  `UserFunctionExceptionHandling`), and the call site in the item's computation.
- So a user body with six user functions costs about 150 hand-written lines of C++ plus its definition; every new
  signature adds one template instantiation, which is build time, not run time. Each step of RG7 measures what it
  added, so that the user items can still be dropped.
- Measured for ObjectUserBody with eight user functions (RG7.3.1, RG7.2): about 520 hand-written lines of C++ - the
  item 315, the renderer cache 97, the wrappers 63, the drawing 20, the consistency check 27 - and 300 lines of
  definition; the user types add 45 lines of header, 100 of binding and the generator lines for the user types and the
  conditional output variables. The ANCF cable and the tetrahedron needed no C++ at all.

## 8. Proposed steps (RG7)

1. **RG7.1** This concept and the measurement (revised with the decisions; for the plan).
2. **RG7.2** The common pattern: naming, the arguments of every user function (`mbs, t, itemNumber`, ...), symbolic
   user functions, how the C++ side calls them; the user types and their names (5.6).
3. **RG7.3** User body as a finite element (5.1); example 1 first (the user rigid body), then example 2 (minimal)
   and example 4.
4. **RG7.4** User nodes (5.2); example 3.
5. **RG7.5** User markers and the user connector with marker access (5.3, 5.4); example 5.
6. **RG7.6** User joint (5.8); example 6.
7. **RG7.7** User sensor with itemNumbers (5.7); example 7.
8. **RG7.8** Symbolic user items in parallel and in the graphics (5.5); example 8 - after the symbolic revision (RG12).
9. **RG7.9** The documentation section "How items work, and user items": the concept per kind, one example per user
   item - done (#2979): docs/manual/userItems.md, which links the general pages of each kind in the item reference.

Outside RG7: **the revision of the Cable2D markers into general markers for curves, surfaces and volumes** (5.3), a
C++ step in RG9; and **the symbolic user functions** (5.5), group RG12.
