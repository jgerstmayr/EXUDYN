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

## 2. ObjectConnectorUser for symbolic forces (#3119)

**The problem.** `forceUserFunction(mbs, t, itemNumber, markers, dataCoordinates, parameters)` gets one
`exudyn.ConnectorMarker` per marker and returns nothing: it reads the marker by methods (`GetPosition()`, ...) and
applies the forces by methods (`ApplyForce(force)`, `ApplyBodyForce(force, localPosition)`, ...). A symbolic user
function is recorded by calling the Python function once with symbolic arguments and keeping the expressions of its
**return value**. Neither part fits:

- a ConnectorMarker is not a symbolic argument: its methods return the numbers of the current call, and the recording
  would keep these numbers as constants;
- the forces are side effects of method calls, not a result: nothing is returned that could be recorded;
- the number and the types of the markers are free, and `ApplyBodyForce` at a local position computed in the function
  needs the Jacobian of the body at that point during the call.

This form stays: it is what makes a contact on a flexible body possible (the closest point searched in the function).

**The solution: a second user function of the marker states, which returns the forces.** As ObjectJointUser gets the
positions and rotation matrices of its markers and returns the constraint equations, the connector gets the states of
its markers and returns the forces on them; the connector applies them through the marker Jacobians, exactly as
`ApplyForce` and `ApplyTorque` do now, $\Qv = -\sum_i (\Jm_{pos,i}\tp \fv_i + \Jm_{rot,i}\tp \mv_i)$. Its arguments
and its result are vectors and matrices of fixed sizes, so it can be recorded. Two forms are possible:

| | (a) two markers, as ObjectJointUser (proposed) | (b) any number of markers, stacked |
|---|---|---|
| signature | `markerForceUserFunction(mbs, t, itemNumber, position0, velocity0, rotationMatrix0, angularVelocity0, position1, velocity1, rotationMatrix1, angularVelocity1, dataCoordinates, parameters)` | `markerForceUserFunction(mbs, t, itemNumber, positions, velocities, rotationMatrices, angularVelocities, dataCoordinates, parameters)`, each of all markers in one vector (3 per marker; 9 per rotation matrix) |
| returns | `[f0, f1]` (6 values) or with orientation `[f0, m0, f1, m1]` (12 values), global | 3 or 6 values per marker, global |
| for | spring-damper, torsional spring-damper, bushings, sphere-sphere contact - every item of the list | items of more than two markers (a cable through pulleys) |
| reads as | the joint, every argument named | index arithmetic in the function |

- `requestedMarkerType` (#3105) decides what the markers provide: `Position` gives the positions and velocities (the
  rotation arguments are unit matrices and zero), `Position | Orientation` also the rotations and angular velocities,
  and the torques in the result.
- Exactly one of `forceUserFunction` and `markerForceUserFunction` is given; Assemble refuses both or none.
- The Jacobian is computed numerically, as ObjectConnectorUser does now; once exudyn.symbolic differentiates vectors
  (#3115), the derivative of the recording by the marker states times the marker Jacobians.
- `dataCoordinates` and the postNewtonStepUserFunction stay as they are (#3102); the post-Newton step of a symbolic
  connector (contact with impact models) is a later step.
- **Drawing**: a `graphicsDataUserFunction(mbs, itemNumber)` of the connector returning a graphics list, Python only -
  a spring drawn as line or spring, a contact as two spheres -, as the drawing of ObjectGenericODE2.
- **In the symbolic system** (RG12.51.4): the connector is a kind of its own, the marker states computed from the
  bodies - a node marker of a point node from the node coordinates, a body marker by the position function of the
  user body -, the forces scattered by the LTG indices of both bodies.

Why not the alternative of RG12.55.1, a user body without mass on the nodes of both bodies: it would bypass the
markers, compute the marker kinematics again in each connector, need a zero mass matrix, and leave the connector
outside the list of connectors (drawing, outputs, `activeConnector`); the extension keeps the connector a connector.

## 3. The interface: the Create functions, the same signatures

Every item of the list has a Create function of mbs with bodies and local positions instead of markers -
`mbs.CreateMassPoint`, `CreateRigidBody`, `CreateSpringDamper`, `CreateRevoluteJoint`, ... -, and these are what most
models use. **A creator object with the same methods and the same arguments** (decisions 1 and 2):

```python
import exudyn.misc.symbolicUserItems as sui
creator = sui.UserItemCreator(mbs)        #or creator = mbs: the same script with the built-in items
b0 = creator.CreateMassPoint(referencePosition=[1,0,0], mass=2, gravity=[0,-9.81,0])
creator.CreateSpringDamper(bodyNumbers=[oGround, b0], stiffness=100, damping=1)
mbs.Assemble()
creator.Record()                          #one symbolic recording per kind, as SymbolicSystem groups them
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

## 4. The items, what they become, what they support

| built-in (Create) | user item | supported arguments | refused / later |
|---|---|---|---|
| CreateGround | the built-in ObjectGround (no coordinates) | all | - |
| CreateMassPoint | ObjectUserBody on NodePoint: $\Mm = m\Im$, $\fv = 0$, position $\qv + \pLoc$, Jacobian $\Im$, mass integral $m\Im$ | referencePosition, initialDisplacement, initialVelocity, mass, gravity, graphicsDataList, drawSize, color, show | create2D (later: NodePoint2D), physicsMass (deprecated) |
| CreateRigidBody | ObjectUserBody on the rigid-body node: mass matrix and quadratic velocity vector of the node type, by symbolicUtilities; position and rotation of a local point | referencePosition, referenceRotationMatrix, initialVelocity, initialAngularVelocity, initialDisplacement, initialRotationMatrix, inertia (with center of mass), gravity, nodeType RotationRotationVector and RotationRxyz, graphicsDataList, referenceHT, initialHT | nodeType EulerParameters (the algebraic equation of the node is needed in the system model - later), create2D (later: NodeRigidBody2D) |
| CreateSpringDamper | ObjectConnectorUser on two position markers with the markerForceUserFunction (section 2): $\fv_1 = -\fv_0 = -(k(L - L_0) + d\dot L + f_0)\,\nv$ | bodyNumbers, localPosition0/1, referenceLength, stiffness, damping, force, velocityOffset, show, drawSize, color | springForceUserFunction (later: a symbolic one inside the recording), bodyOrNodeList with nodes |
| CreateTorsionalSpringDamper | ObjectConnectorUser on two rigid markers: the torque of the relative rotation about the axis | bodyNumbers, position, axis, stiffness, damping, offset, velocityOffset, torque, useGlobalFrame | unlimitedRotations (needs the data of the turns), springTorqueUserFunction |
| CreateRevoluteJoint, CreatePrismaticJoint | ObjectJointUser on MarkerBodyRigid of both bodies, the constraint and its Jacobian of the built-in joint, symbolic | bodyNumbers, position, axis, useGlobalFrame, show, axisRadius, axisLength, color | - |
| CreateSphericalJoint | ObjectJointUser as above | bodyNumbers, position, constrainedAxes, useGlobalFrame, show, jointRadius, color | - |
| CreateDistanceConstraint | ObjectJointUser of one equation, $\|\pv_1 - \pv_0\| - d$ | bodyNumbers, localPosition0/1, distance, show, drawSize, color | - |
| CreateSphereSphereContact | ObjectConnectorUser on two position markers: linear contact stiffness and damping, $\fv = (k\,g + d\,\dot g)\,\nv$ while the gap $g < 0$, by `esym.IfThenElse` | bodyNumbers, localPosition0/1, spheresRadii, contactStiffness, contactDamping | the impact models, restitution, friction, adhesion, plasticity (they need the data node and the post-Newton step of a symbolic connector), isHollowSphere1 |
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

Open, for the maintainer: **the form of the markerForceUserFunction**, (a) two markers with named arguments, as
ObjectJointUser (proposed), or (b) any number of markers, stacked; and its name.
