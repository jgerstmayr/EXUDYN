# Symbolic user items with the interfaces of the built-in items: design (revision2026c step RG12.55.1, #3111)

`exudyn/misc/symbolicUserItems.py`: user items with symbolic user functions that are created **with the same calls as
the built-in items**, so that a system is built once and runs either with the built-in items or with the user items -
in Exudyn (to compare) and in the symbolic system model (RG12.51.4, the export and the GPU). It comes before the system
model goes deeper. This note lists what is needed, proposes the interface and asks the decisions. It lives while
RG12.55 is open (WORKFLOW.md §8.4).

## 1. What the user items can build on (2026-10-26)

| need | exists | symbolic |
|---|---|---|
| a body on existing nodes: mass matrix, forces, position, rotation, their Jacobians, mass integral | ObjectUserBody (#2972) | all its user functions |
| the built-in markers, loads, sensors and connectors on such a body | MarkerBodyPosition, MarkerBodyRigid, MarkerBodyMass through the body access functions | - |
| a constraint of two rigid markers | ObjectJointUser (#2976), constraint and Jacobian by the marker positions and rotations | both |
| a constraint of node coordinates | ObjectConnectorCoordinateVector, constraintUserFunction(mbs, t, itemNumber, q, q_t, velocityLevel) and its Jacobian | both; **no parameters argument** |
| a force between markers | ObjectConnectorUser (#2975) | **no**: its markers are Python objects |
| the rotation matrices, G matrices of the node types | exudyn.symbolicUtilities | with exudyn.symbolic |
| one recording for many items of a kind | CreateSymbolicUserFunction with `parameters` (ParameterLayout) | - |

Two facts decide the design:

- **A force between bodies cannot be a symbolic ObjectConnectorUser.** A spring-damper, a torsional spring-damper and a
  contact are therefore written as an **ObjectUserBody on the nodes of the bodies they connect**, without mass: its
  force function computes the points from the node coordinates and the local positions (its parameters), by the
  kinematics of the node type, and returns the generalized forces $\Jm\tp\fv$. This needs no new C++, and it is exactly
  the form the system model takes: a residual per object on its coordinates. Its mass matrix is zero - a function
  returning zeros, or a flag `noMass` instead (decision 4).
- **A connector or joint needs mbs at creation**: the built-in interface names markers or bodies, the user item needs
  their nodes and node types. So the classes of itemInterface - which are only dicts, added by `mbs.AddObject` without
  access to mbs - fit the bodies, which sit on one node, but not the connectors.

## 2. The interface: the Create functions, the same signatures

Every item of the list has a Create function of mbs with bodies and local positions instead of markers -
`mbs.CreateMassPoint`, `CreateRigidBody`, `CreateSpringDamper`, `CreateRevoluteJoint`, ... -, and these are what most
models use. Proposal: **a creator object with the same methods and the same arguments**:

```python
import exudyn.misc.symbolicUserItems as sui
creator = sui.UserItemCreator(mbs)        #or creator = mbs: the same script with the built-in items
b0 = creator.CreateMassPoint(referencePosition=[1,0,0], mass=2, gravity=[0,-9.81,0])
creator.CreateSpringDamper(bodyNumbers=[oGround, b0], stiffness=100, damping=1)
mbs.Assemble()
creator.Record()                          #one symbolic recording per kind, as SymbolicSystem groups them
```

- The methods add the same nodes, markers and loads as the built-in functions (a NodePoint, a MarkerBodyMass and a
  LoadMassProportional for `gravity`, ...), and an ObjectUserBody or ObjectJointUser instead of the built-in object;
  they return the same numbers or dicts (`returnDict`).
- The user functions are Python first, so the system runs in Exudyn as built; `Record()` replaces them by one symbolic
  recording per kind (all mass points one kind, all spring-dampers between two point nodes another, ...), each item
  with its parameters.
- An argument the user item does not support raises, naming it - never ignored silently. Visualization arguments
  (`graphicsDataList`, `color`, `drawSize`, `show`) are passed to the user body or are drawn as the built-in item
  draws itself, where the user body cannot (a spring as a line: decision 5).
- Item-level classes (`sui.MassPoint(nodeNumber=..., mass=...)` for `mbs.AddObject`) are possible for the bodies, which
  sit on one node, and can follow; for the connectors they would need mbs and are not proposed.

## 3. The items, what they become, what they support

| built-in (Create) | user item | supported arguments | refused / later |
|---|---|---|---|
| CreateGround | the built-in ObjectGround (no coordinates) | all | - |
| CreateMassPoint | ObjectUserBody on NodePoint: $\Mm = m\Im$, $\fv = 0$, position $\qv + \pLoc$, Jacobian $\Im$, mass integral $m\Im$ | referencePosition, initialDisplacement, initialVelocity, mass, gravity, graphicsDataList, drawSize, color, show | create2D (later: NodePoint2D), physicsMass (deprecated) |
| CreateRigidBody | ObjectUserBody on the rigid-body node: mass matrix and quadratic velocity vector of the node type, by symbolicUtilities; position and rotation of a local point | referencePosition, referenceRotationMatrix, initialVelocity, initialAngularVelocity, initialDisplacement, initialRotationMatrix, inertia (with center of mass), gravity, nodeType RotationRotationVector and RotationRxyz, graphicsDataList, referenceHT, initialHT | nodeType EulerParameters (the algebraic equation of the node is needed in the system model - later), create2D (later: NodeRigidBody2D) |
| CreateSpringDamper | ObjectUserBody without mass on the nodes of both bodies: $\fv = (k(L - L_0) + d\dot L + f_0)\,\nv$ at the two points | bodyNumbers, localPosition0/1, referenceLength, stiffness, damping, force, velocityOffset, show, drawSize, color | springForceUserFunction (later: a symbolic one inside the recording), bodyOrNodeList with nodes |
| CreateTorsionalSpringDamper | ObjectUserBody without mass on the two rigid-body nodes: the torque of the relative rotation about the axis, by $\Gm\tp$ | bodyNumbers, position, axis, stiffness, damping, offset, velocityOffset, torque, useGlobalFrame | unlimitedRotations (needs the data of the turns), springTorqueUserFunction |
| CreateRevoluteJoint, CreatePrismaticJoint | ObjectJointUser on MarkerBodyRigid of both bodies, the constraint and its Jacobian of the built-in joint, symbolic | bodyNumbers, position, axis, useGlobalFrame, show, axisRadius, axisLength, color | - |
| CreateSphericalJoint | ObjectJointUser as above | bodyNumbers, position, constrainedAxes, useGlobalFrame, show, jointRadius, color | - |
| CreateDistanceConstraint | ObjectJointUser of one equation, $\|\pv_1 - \pv_0\| - d$ | bodyNumbers, localPosition0/1, distance, show, drawSize, color | - |
| CreateSphereSphereContact | ObjectUserBody without mass on both nodes: linear contact stiffness and damping, $\fv = (k\,g + d\,\dot g)\,\nv$ while the gap $g < 0$, by `esym.IfThenElse` | bodyNumbers, localPosition0/1, spheresRadii, contactStiffness, contactDamping | the impact models, restitution, friction, adhesion, plasticity (they need the data node and the post-Newton step, which a symbolic item has not), isHollowSphere1 |
| CreateForce, CreateTorque | the built-in LoadForceVector / LoadTorqueVector on a marker of the user body | all but loadVectorUserFunction | - |

For a **good representation of multibody systems** beyond the list: planar mass points and rigid bodies (NodePoint2D,
NodeRigidBody2D - the planar systems are the simplest test of everything), CreateCartesianSpringDamper and
CreateRigidBodySpringDamper (bushings), CreateCoordinateConstraint, a generic joint (`CreateGenericJoint`, constrained
axes), and sensors - which work unchanged, as they read the output variables of the user bodies.

**The order:** first CreateGround, CreateMassPoint, CreateSpringDamper, CreateForce - enough for the explicit solvers of
RG12.56 (mass points on spring-dampers, without constraints); then CreateRigidBody and CreateTorsionalSpringDamper;
then the joints, which need the generalized-alpha solver (RG12.51.3) and the constraints in the system model
(RG12.51.4); the contact last.

**The check** of every item: the same script with `creator = mbs` and with `creator = sui.UserItemCreator(mbs)`, the
results equal to the tolerance of the integrator (as userConnectorContactTest.py does for one contact), in a test model
per group.

## 4. Decisions needed

1. The interface: the creator object with the Create methods of the same signatures (proposed) - or item-level classes
   for `mbs.AddObject` for the bodies as well, now.
2. The names: `UserItemCreator(mbs).CreateMassPoint(...)` (proposed), or module functions `sui.CreateMassPoint(mbs, ...)`.
3. Connectors and contacts as user bodies without mass on the nodes of both bodies (proposed; no new C++) - or a new
   symbolic connector item on markers, a C++ extension of ObjectConnectorUser or ObjectJointUser.
4. Their mass: a massMatrixUserFunction that returns zeros (proposed for now), or a flag of ObjectUserBody for an item
   without mass, so that no zero matrix is assembled.
5. The drawing of a connector user body (a line, a spring): through its graphicsDataUserFunction, or not drawn in a
   first version.
