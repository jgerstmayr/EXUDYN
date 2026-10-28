# Symbolic user items with the interfaces of the built-in items: design (revision2026c step RG12.55.1, #3111)

`exudyn/misc/symbolicUserItems.py`: user items with symbolic user functions that are created **with the same calls as
the built-in items**, so that a system is built once and runs either with the built-in items or with the user items -
in Exudyn (to compare) and in the symbolic system model (RG12.51.4, the export and the GPU). It comes before the system
model goes deeper. This note lists what is needed, the interface, the extension of ObjectConnectorUser it needs, and
the decisions. It lives while RG12.55 is open (WORKFLOW.md §8.4).

## 1. What the user items can build on (2026-10-26)

| need | exists | symbolic |
|---|---|---|
| a body on existing nodes: mass matrix, forces, position, rotation, their Jacobians, mass integral | ObjectUserBody (#2972) | all its user functions |
| the built-in markers, loads, sensors and connectors on such a body | MarkerBodyPosition, MarkerBodyRigid, MarkerBodyMass through the body access functions | - |
| a constraint of two rigid markers | ObjectJointUser (#2976), constraint and Jacobian by the marker positions and rotations | both |
| a constraint of node coordinates | ObjectConnectorCoordinateVector, constraintUserFunction(mbs, t, itemNumber, q, q_t, velocityLevel) and its Jacobian | both; **no parameters argument** |
| a force between markers | ObjectConnectorUser (#2975) | **no**: its markers are Python objects - section 2 |
| the rotation matrices, G matrices of the node types | exudyn.symbolicUtilities | with exudyn.symbolic |
| one recording for many items of a kind | CreateSymbolicUserFunction with `parameters` (ParameterLayout) | - |

**A connector or joint needs mbs at creation**: the built-in interface names markers or bodies, the user item adds its
markers on them. So the classes of itemInterface - which are only dicts, added by `mbs.AddObject` without access to
mbs - do not fit; the Create functions do (section 3).

## 2. ObjectConnectorUser for symbolic forces (#3119, done)

**Why the ConnectorMarker form cannot be recorded.** A function that gets one `exudyn.ConnectorMarker` per marker and
returns nothing - it reads the markers by methods (`GetPosition()`, ...) and applies the forces by methods
(`ApplyForce(force)`, `ApplyBodyForce(force, localPosition)`, ...) - does not fit a recording, which calls the Python
function once with symbolic arguments and keeps the expressions of its **return value**:

- a ConnectorMarker is not a symbolic argument: its methods return the numbers of the current call, and the recording
  would keep these numbers as constants;
- the forces are side effects of method calls, not a result: nothing is returned that could be recorded;
- the number and the types of the markers are free, and `ApplyBodyForce` at a local position computed in the function
  needs the Jacobian of the body at that point during the call.

**The forceUserFunction of the marker states.** As ObjectJointUser gets the positions and rotation matrices of its
markers and returns the constraint equations, the connector gets the states of its two markers and returns the forces
on them, and applies them through the marker Jacobians, as `ApplyForce` and `ApplyTorque` do,
$\fv_{gen} = -\sum_i (\Jm_{pos,i}\tp \fv_i + \Jm_{rot,i}\tp \mv_i)$ (decision 6):

```python
def UFspringDamper(mbs, t, itemNumber, position0, velocity0, rotationMatrix0, angularVelocity0,
                   position1, velocity1, rotationMatrix1, angularVelocity1, dataCoordinates, parameters):
    ...
    return np.hstack([force0, force1])     #with Orientation requested: [force0, torque0, force1, torque1]
```

- Its arguments and result are vectors and matrices of fixed sizes, so it is recorded by `CreateSymbolicUserFunction`.
- The connector requests Position for it; with `exu.MarkerType.Orientation` in `requestedMarkerType` it gets the
  rotations and angular velocities and returns the torques too, otherwise unit matrices and zeros.
- Assemble refuses it with other than two markers, and together with the markerAccessUserFunction.
- The Jacobian is numerical in Exudyn, as before; in the symbolic system by `EvaluateJacobian` of the recording (#3121).
- `dataCoordinates` and the postNewtonStepUserFunction are as before (#3102); userConnectorContactTest.py computes
  the contact force by the forceUserFunction and switches the contact in the postNewtonStepUserFunction, which reads
  the markers.
- **Drawing**: a `graphicsDataUserFunction(mbs, itemNumber)` of the connector returning a graphics list in global
  coordinates, Python only, as the drawing of ObjectGenericODE2.
- **In the symbolic system** (#3121): a kind of its own per pattern of markers; the marker states of a point node from
  the node coordinates, of the ground from the fixed position; the forces scattered to the coordinates of the markers.
  Markers on bodies and the rotations follow with RG12.51.4.

**The markerAccessUserFunction**: the former ConnectorMarker form under its own name. It is for forces at points that
the function computes itself on a body of many coordinates - `GetBodyPosition(localPosition)` and
`ApplyBodyForce(force, localPosition)`, which need the body Jacobian at that point during the call; its use is the ball
on an ANCF cable whose closest point is searched along the cable (userConnectorTest.py, part 3). Its description says
so, and that it cannot be recorded. ObjectConnectorUser came after the 1.13.0 release (#2975), so the change of names
needed no deprecation.

Why not the alternative of RG12.55.1, a user body without mass on the nodes of both bodies: it would bypass the
markers, compute the marker kinematics again in each connector, need a zero mass matrix, and leave the connector
outside the list of connectors (drawing, outputs, `activeConnector`); the extension keeps the connector a connector.

## 3. The interface: the Create functions, the same signatures

Every item of the list has a Create function of mbs with bodies and local positions instead of markers -
`mbs.CreateMassPoint`, `CreateRigidBody`, `CreateSpringDamper`, `CreateRevoluteJoint`, ... -, and these are what most
models use. **A creator object with the same methods and the same arguments** (decisions 1 and 2; done for ground, mass
point, spring-damper and force, #3125):

```python
import exudyn.misc.symbolicUserItems as sui
creator = sui.UserItemCreator(mbs)        #or creator = mbs: the same script with the built-in items
b0 = creator.CreateMassPoint(referencePosition=[1,0,0], mass=2, gravity=[0,-9.81,0])
creator.CreateSpringDamper(itemNumbers=[creator.CreateGround(), b0], stiffness=100, damping=1)
creator.Record()                          #one symbolic recording per kind, as SymbolicSystem groups them
mbs.Assemble()
```

- The methods add the same nodes, markers and loads as the built-in functions (a NodePoint, a MarkerBodyMass and a
  LoadMassProportional for `gravity`, ...), and an ObjectUserBody, ObjectConnectorUser or ObjectJointUser instead of
  the built-in object; they return the same numbers or dicts (`returnDict`).
- The user functions are Python first, so the system runs in Exudyn as built; `Record()` replaces them by one symbolic
  recording per kind (all mass points one kind, all spring-dampers another, ...), each item with its parameters.
- An argument the user item does not support raises, naming it - never ignored silently. Visualization arguments
  (`graphicsDataList`, `color`, `drawSize`, `show`) are passed to the user body, or to the graphicsDataUserFunction of
  the connector (section 2).
- **A small set first** (decision 2): ground, mass point, spring-damper, force - enough for the explicit solvers and to
  test the interface; the others as the steps below.
- **Done** (#3125): `UserItemCreator` with `CreateGround` (the built-in ground), `CreateMassPoint` (ObjectUserBody on
  a NodePoint with its mass, mass integral, position, rotation and Jacobian functions), `CreateSpringDamper`
  (ObjectConnectorUser, its force with cMath) and `CreateForce` (the built-in load, a Python loadVectorUserFunction
  recorded by `Record`); `items` lists the items of each kind, `recordings` keeps the recordings alive. At the center
  of a user mass point the spring-damper takes a marker on its node instead of the body: the position is the same,
  and SymbolicSystem takes node markers until RG12.51.4 brings body markers. symbolicUserItemsTest.py: the same
  script with `creator = mbs` and with the creator, the Python and the recorded user items equal to the built-in
  items to 7e-16, SolveExplicit on the recorded system to 1.3e-15.
- **Done** (#3128): `CreateRigidBody`, ObjectUserBody on NodeRigidBodyRxyz with the parameters [mass, inertia about
  the node (6), center of mass (3)] - the mass matrix with the coupling of a center of mass off the node and the
  quadratic velocity terms of CObjectRigidBody, written once with exudyn.symbolicUtilities for numbers (cMath) and for
  the recording (exudyn.symbolic) -, and `CreateTorsionalSpringDamper`, ObjectConnectorUser on the MarkerBodyRigid of
  the built-in function, requesting the orientation: the torque about the z-axis of marker 0 of the z Tait-Bryan
  angle of $\LU{0,m0}{\Rot}\tp \LU{0,m1}{\Rot}$ and the relative angular velocity, on marker 0 and back on marker 1, as
  the forces of the spring-damper act on the markers. symbolicUserRigidBodiesTest.py: a chain of three bars, three
  spring-dampers and three torsional spring-dampers per joint, equal to the built-in items to 1.9e-14 (RK44) and
  5.5e-13 (generalized-alpha, whose Newton iterates differ by the numerical Jacobian of the user items).

## 4. The items, what they become, what they support

| built-in (Create) | user item | supported arguments | refused / later |
|---|---|---|---|
| CreateGround | the built-in ObjectGround (no coordinates) | all | - |
| CreateMassPoint | ObjectUserBody on NodePoint: $\Mm = m\Im$, $\fv = 0$, position $\qv + \pLoc$, Jacobian $\Im$, mass integral $m\Im$ | referencePosition, initialDisplacement, initialVelocity, mass, gravity, graphicsDataList, drawSize, color, show | create2D (later: NodePoint2D), physicsMass (deprecated) |
| CreateRigidBody | ObjectUserBody on NodeRigidBodyRxyz: mass matrix and quadratic velocity vector of CObjectRigidBody, by symbolicUtilities; position and rotation of a local point, their Jacobians, the mass integral | referencePosition, referenceRotationMatrix, initialVelocity, initialAngularVelocity, initialDisplacement, initialRotationMatrix, inertia (with center of mass), gravity, nodeType RotationRxyz, graphicsDataList, drawSize, color, show, returnDict, referenceHT, initialHT | nodeType EulerParameters - the default - (the algebraic equation of the node is needed in the system model - later), RotationRotationVector (a Lie group node, which the solvers of the symbolic system do not integrate), graphicsDataUserFunction (other arguments in ObjectUserBody), create2D (later: NodeRigidBody2D) |
| CreateSpringDamper | ObjectConnectorUser on two position markers with the forceUserFunction of the marker states (section 2): $\fv_1 = -\fv_0 = -(k(L - L_0) + d\dot L + f_0)\,\nv$ | itemNumbers, localPosition0/1, referenceLength, stiffness, damping, force, velocityOffset, show, drawSize, color | springForceUserFunction (later: a symbolic one inside the recording), the deprecated bodyNumbers and bodyOrNodeList |
| CreateTorsionalSpringDamper | ObjectConnectorUser on two rigid markers: the torque of the relative rotation about the axis | itemNumbers (bodies or the ground), position, axis, stiffness, damping, offset, velocityOffset, torque, useGlobalFrame, show, drawSize, color | unlimitedRotations, whose default True is refused (needs the data of the turns: pass False), springTorqueUserFunction, markers in itemNumbers, the deprecated bodyNumbers and bodyOrNodeList |
| CreateRevoluteJoint, CreatePrismaticJoint | ObjectJointUser on MarkerBodyRigid of both bodies, the constraint and its Jacobian of the built-in joint, symbolic | itemNumbers, position, axis, useGlobalFrame, show, axisRadius, axisLength, color | - |
| CreateSphericalJoint | ObjectJointUser as above | itemNumbers, position, constrainedAxes, useGlobalFrame, show, jointRadius, color | - |
| CreateDistanceConstraint | ObjectJointUser of one equation, $\|\pv_1 - \pv_0\| - d$ | itemNumbers, localPosition0/1, distance, show, drawSize, color | - |
| CreateSphereSphereContact | ObjectConnectorUser on two position markers: linear contact stiffness and damping, $\fv = (k\,g + d\,\dot g)\,\nv$ while the gap $g < 0$, by `esym.IfThenElse` | itemNumbers, localPosition0/1, spheresRadii, contactStiffness, contactDamping | the impact models, restitution, friction, adhesion, plasticity (they need the data node and the post-Newton step of a symbolic connector), isHollowSphere1 |
| CreateForce, CreateTorque | the built-in LoadForceVector / LoadTorqueVector on a marker of the user body | all but loadVectorUserFunction | - |

For a **good representation of multibody systems** beyond the list: planar mass points and rigid bodies (NodePoint2D,
NodeRigidBody2D - the planar systems are the simplest test of everything), CreateCartesianSpringDamper and
CreateRigidBodySpringDamper (bushings), CreateCoordinateConstraint, a generic joint (`CreateGenericJoint`, constrained
axes), and sensors - which work unchanged, as they read the output variables of the user items.

**The order:** first the extension of ObjectConnectorUser (section 2, RG12.55.6), then CreateGround, CreateMassPoint,
CreateSpringDamper, CreateForce - enough for the explicit solvers of RG12.56 (mass points on spring-dampers, without
constraints); then CreateRigidBody and CreateTorsionalSpringDamper; then the joints, which need the generalized-alpha
solver (RG12.51.3) and the constraints in the system model (RG12.51.4); the contact last.

**The check** of every item: the same script with `creator = mbs` and with `creator = sui.UserItemCreator(mbs)`, the
results equal to the tolerance of the integrator (as userConnectorContactTest.py does for one contact), in a test model
per group.

## 5. Decisions (maintainer 2026-10-26)

1. The Create functions with the same signatures on a creator object - sufficient for now; no item-level classes.
2. `UserItemCreator(mbs).CreateMassPoint(...)`; only a small set of items first, to test.
3. Connectors and contacts by an extension of ObjectConnectorUser, not as user bodies without mass (section 2).
4. (The mass of connector bodies: not needed with 3.)
5. A connector is drawn by a graphicsDataUserFunction, in Python only.
6. The function of the marker states (#3119): two markers with named arguments as ObjectJointUser; returning the forces
   is the form for symbolic user items in general (2026-10-26).
7. It is the `forceUserFunction`; the ConnectorMarker form is kept as `markerAccessUserFunction`, for forces at computed
   points of flexible bodies (2026-10-26).
