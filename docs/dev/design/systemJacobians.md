# The system Jacobians: design (revision2026c step RG9.11, #3201)

The Jacobian of the implicit solvers and the static solver is assembled by CSystem from four functions,
`JacobianODE2RHS`, `JacobianODE1RHS`, `JacobianAE` and `JacobianODE2Loads`. Each decides per object how its block is
computed, by flags of the object, of its markers and of the settings. Maintainer 2026-11-01: every object provides a
Jacobian - analytically, or locally by automatic or numerical differentiation of its own -, so that the assembly
distinguishes only the system-wide numerical Jacobian and the assembly of the local Jacobians, the latter in parallel
for the objects without Python functions; only ODE2-ODE2, AE-AE and ODE2-AE are optimized for speed, the other blocks
are rare, completed for all couplings and written readable rather than fast; in line with the ODE1-ODE2 couplings of
#3143; CSystem.cpp split first (#3202). This note describes what the code does now, the target, the sub-steps and
what is to be decided.

Related issues: #1203 and #1202 (the analytic ODE2 and AE Jacobians in parallel, revision2026c step RG5.3), #858 (a
cost per item for the split of the tasks), #1548 (the Jacobian of ODE1 loads, RG4.23), #988 (a sparse
`ComputeConstraintJacobianDerivative`), #3107 (the central differences of the user items), #2884 (the joints on the
relative transformation, a revision of the constraint Jacobians themselves, not of their assembly).

## 1. The blocks

The solvers need, with the factors of the integrator,

$$
\Jm = \begin{bmatrix}
  \Jm_{\SO\SO} & \Jm_{\SO\FO} & \Jm_{\SO\AE} \\
  \Jm_{\FO\SO} & \Jm_{\FO\FO} & \Jm_{\FO\AE} \\
  \Jm_{\AE\SO} & \Jm_{\AE\FO} & \Jm_{\AE\AE}
\end{bmatrix},
\qquad
\Jm_{\SO\SO} = a\,\frac{\partial \fv}{\partial \qv} + b\,\frac{\partial \fv}{\partial \dot\qv},
$$

$\fv$ the ODE2 right-hand side, $\Jm_{\SO\AE} = \Cm_{\qv}\tp$ (or $\Cm_{\dot\qv}\tp$ of a velocity-level constraint),
$\Jm_{\AE\SO} = \partial\cv/\partial\qv$ and $\partial\cv/\partial\dot\qv$, and the ODE1 blocks of #3143:
$\Jm_{\SO\FO} = \partial\fv/\partial\yv$, $\Jm_{\FO\SO}$ from the ODE1 right-hand side by $\qv$ and $\dot\qv$,
$\Jm_{\FO\FO}$, $\Jm_{\FO\AE} = -\Cm_{\yv}\tp$ and $\Jm_{\AE\FO} = \Cm_{\yv}$.

| block | rows from | where now | how now |
|---|---|---|---|
| $\Jm_{\SO\SO}$ | bodies, finite elements, connectors | `JacobianODE2RHS` | per object: analytic (`ComputeJacobianODE2_ODE2`), the connector interface (`ComputeJacobianODE2Connector`, automatic differentiation of the force, #2745), the path of the marker data (`ComputeMarkerDataStructure` with Jacobians, `ComputeJacobianODE2_ODE2`, the Jacobian derivative of the markers), or CSystem differentiates the object numerically, with the Lie group nodes apart; serial |
| $\Jm_{\SO\SO}$ | loads | `JacobianODE2Loads` | numerical per load, only with `computeLoadsJacobian`; an ODE1 dependency throws (#1548) |
| $\Jm_{\SO\SO}$ | general contact | `GeneralContact::JacobianODE2LHS` | its own, parallel |
| $\Jm_{\SO\FO}$ | objects with ODE1 dependency | `JacobianODE2RHS` | `ComputeJacobianODE2_ODE1` if available, else numerical, in two code paths (after the analytic and after the numerical ODE2 block) |
| $\Jm_{\FO\FO}$, $\Jm_{\FO\SO}$ | ODE1 objects | `JacobianODE1RHS` | `ComputeJacobianODE1RHS` (all three blocks at once) if available and not a connector, else numerical; serial |
| $\Jm_{\AE\SO}$, $\Jm_{\SO\AE}$, $\Jm_{\AE\FO}$, $\Jm_{\FO\AE}$, $\Jm_{\AE\AE}$ | constraints, bodies with AE, nodes with AE | `JacobianAE` | analytic always - the connector interface for joints (`ComputeJacobianAEConnector`), else the marker data; the transposed block chosen by `UsesVelocityLevel`; nodes (Euler parameters) by a loop of their own; serial |
| all | all | `NumericalJacobianAE`, the `doSystemWideDifferentiation` branches | system-wide numerical: `forAE` (dense only) for the AE rows, `doSystemWideDifferentiation` for the ODE2 and ODE1 rows |

Neglected by default: the derivative of $\Mm(\qv)\ddot\qv$ by $\qv$, the derivative of $\Cm_{\qv}\tp\tlambda$ by $\qv$
(computed only for the option of the generalized-alpha solver that calls `ComputeConstraintJacobianDerivative`), the
Jacobian derivative of a marker without `JacobianDerivativeAvailable` (the connector then falls back to numerical),
and the loads without `computeLoadsJacobian`. The solver chapter of the manual still says that $\Jm_{\SO\FO}$ and
$\Jm_{\FO\SO}$ vanish, which is no longer so since #3143.

The residuals and the mass matrix are assembled in parallel already: `ComputeSystemODE2RHS` over
`listComputeObjectODE2LhsNoUF` with one sparse vector per thread, then the objects with Python user functions
serially (`listComputeObjectODE2LhsUF`); `ComputeMassMatrix` over `objectsBodyWithODE2CoordsNoUF` with sparse triplets
per thread, appended serially. The Jacobians have no parallel path; `multithreadedLowerLimitJacobians` exists unused.

## 2. What is wrong with it

1. **The decision is spread out.** `ComputeObjectJacobianODE2` and the two loops of `JacobianODE2RHS` test the
   available Jacobians, the object type (connector or not), `numDiff.forODE2`, `numDiff.forODE2Connectors` (marked as
   a flag for development to be removed), `jacobianConnectorDerivative` and the marker types; `JacobianODE1RHS` and
   `JacobianAE` repeat the pattern differently. Which way an object takes is known only by reading these.
2. **CSystem differentiates objects.** The numerical Jacobian of an object is written in CSystem, with its Lie group
   variant and its column lists without duplicates (#1424), once for ODE2 and twice for the ODE1 coupling.
3. **Serial.** The analytic Jacobians of finite elements, rigid bodies and connectors could be computed in parallel
   as the residuals are; #1203 and #1202 have waited for it since 2022.
4. **Incomplete couplings.** ODE1 loads (#1548), $\Jm_{\SO\FO}$ only through the ODE2 loop, the transposed AE block
   either of position or of velocity level; each added where a model needed it.
5. **Dependencies outside the local-to-global lists.** A Python user function that reads a sensor, another item or
   the system state depends on coordinates its object does not list. The local Jacobians miss this coupling silently;
   only `doSystemWideDifferentiation` sees it.

## 3. The target

### 3.1 One Jacobian per object

Each object computes the local blocks of its equations by its local coordinates through **one virtual function** of
`CObject`, with the blocks requested and their factors:

```cpp
//! the local Jacobian of the object's equations: the blocks requested in request (ODE2, ODE1, AE rows by the ODE2,
//! ODE2_t, ODE1 and AE coordinates of its local-to-global lists), each scaled by its factor, into jacobian
virtual void ComputeLocalJacobian(const CSystemData& systemData, TemporaryComputationData& temp,
    const JacobianRequest& request, LocalJacobian& jacobian) const;
```

- `LocalJacobian` holds the blocks dense, in the local numbering, or as triplets in local numbering for large finite
  elements; the assembly maps them by the local-to-global lists.
- **The default of the base class** is the numerical differentiation of the object's own residual functions, by its
  local coordinates - what CSystem does now, moved into one place (`CObject`, with the Lie group variant), and used
  by every object that has nothing better.
- **Analytic or automatic differentiation** override it: the finite elements and rigid bodies of today, the
  connectors on the connector interface (the L2 chains of #2745 compute exactly these blocks; the markers provide the
  position and rotation Jacobians and their derivatives), the joints on `ComputeJacobianAEConnector`. The path of the
  marker data stays as the implementation of the connectors not on the interface, inside them.
- **Declared at Assemble**, not tested at every call: `GetJacobianKind()` - analytic, automatic, numerical -,
  `UsesPythonFunctions()` (exists as the UF lists), and the blocks the object has (`GetAvailableJacobians`, kept).
  `PreComputeItemLists` builds from them the lists of §3.2.

The numerical default perturbs the global state vector, as now, so that an object differentiated numerically is
not computed in parallel: two threads perturbing the coordinates of a shared node would see each other's
perturbations. It moves to the parallel list once it is analytic or differentiated automatically. A numerical
differentiation from a local copy of the coordinates would need every object to compute from given coordinates
instead of its nodes; not proposed.

### 3.2 Two ways of assembly

1. **The system-wide numerical Jacobian** (`doSystemWideDifferentiation`, and `forAE` extended to sparse matrices):
   all couplings, including those outside the local-to-global lists; serial; the reference for comparison and the
   way for models with such dependencies.
2. **The assembly of the local Jacobians**:
   - `listJacobianParallel`: analytic or automatic, no Python function - computed in `ParallelFor` with one
     `TemporaryComputationData` and one triplet list per thread, appended serially as for the mass matrix (#1203,
     #1202; the tasks by equal numbers of items, section 9);
   - `listJacobianSerial`: a Python function, the numerical default, and the objects declaring dependencies outside
     their lists - computed serially after the parallel part, as the residuals do.

   An object with dependencies outside its lists gets them by its own declaration or by `doSystemWideDifferentiation`
   (decision J3).

For a dense system matrix the per-thread triplets are added to the dense matrix serially; the cost is the number of
nonzero entries, as for the residuals (below 1 % of the time for 3D bodies at 12 threads).

### 3.3 Fast blocks and the others

- **For speed, parallel:** $\Jm_{\SO\SO}$ of objects and connectors, and the constraint blocks $\Jm_{\AE\SO}$,
  $\Jm_{\SO\AE}$ (its transposed, added by the assembly from the same local block) and $\Jm_{\AE\AE}$.
- **Serial and readable, complete:** $\Jm_{\SO\FO}$, $\Jm_{\FO\SO}$, $\Jm_{\FO\FO}$, $\Jm_{\FO\AE}$, $\Jm_{\AE\FO}$, the
  loads (with the ODE1 coupling, #1548), the nodes with algebraic equations, and the derivative terms of the mass
  matrix and of $\Cm_{\qv}\tp\tlambda$ as options. One loop per kind of row, with the request of §3.1, instead of the
  two code paths of the ODE1 coupling now.
- `GeneralContact` keeps its own parallel Jacobian.

### 3.4 Verification

A test model that builds one small system per item type, computes the local Jacobian of each item and compares it
with the system-wide numerical one for every block (by the Python access of `MainSolverBase.ComputeJacobian*`), and
lists the items whose local Jacobian is numerical. This is the check of every sub-step and makes the classification
of §3.1 visible.

## 4. The split of CSystem.cpp (#3202)

Before the revision, without changing a function:

| file | contents |
|---|---|
| `CSystemAssemble.cpp` | `Assemble`, `CheckSystemIntegrity`, `AssembleCoordinates`, `AssembleLTGLists`, `AssembleObjectLTGLists`, `UpdateUserFunctionsForThreads`, `PreComputeItemLists`, the Lie group lists, `AssembleInitializeSystemCoordinates`, `AssembleSystemInitialize` |
| `CSystem.cpp` | the mass matrix, the ODE2 and ODE1 right-hand sides, loads, algebraic equations, the post-Newton step, the reaction forces, `UpdatePostProcessData`, the per-object helpers |
| `CSystemJacobians.cpp` | `ComputeObjectJacobianODE2`, `JacobianODE2RHS`, `JacobianODE1RHS`, `JacobianODE2Loads`, `NumericalJacobianAE`, `ComputeObjectJacobianAE`, `JacobianAE`, the constraint Jacobian derivative and times vector |

The new files go into `msvc/cppsrc.vcxproj`, from which `sources.json` is generated; the timers registered in the
file move with their functions.

## 5. Sub-steps proposed

1. This design (RG9.11.1).
2. Split CSystem.cpp (#3202).
3. The verification test of §3.4 on the code as it is, so that every later step is compared with it.
4. `ComputeLocalJacobian` with the numerical default in `CObject`, the objects with analytic Jacobians and the
   connector interface on it; `JacobianODE2RHS` and `JacobianODE1RHS` as one loop over the request; same results.
5. The parallel assembly of `listJacobianParallel` (#1203, #1202, #858; replaces RG5.3), measured on the
   performance models.
6. The AE blocks on the same interface, the nodes with algebraic equations included, `forAE` for sparse matrices.
7. The couplings completed serially: the ODE1 loads (#1548, RG4.23), the derivative terms as options; the solver
   chapter of the manual.

## 6. To be decided

- **J1** One virtual `ComputeLocalJacobian` with a request, replacing `ComputeJacobianODE2_ODE2`,
  `ComputeJacobianODE2_ODE1`, `ComputeJacobianODE1RHS` and `ComputeJacobianAE` - or keeping them as the
  implementations behind it. Proposed: keep them behind it first, remove them once every object is on it.
- **J2** The numerical default in `CObject`, serial; parallel only for analytic and automatic Jacobians. Proposed.
- **J3** Dependencies outside the local-to-global lists: a flag of the object (a parameter of the Python user items,
  such as `jacobianIsSystemWide`) that puts the object into the serial list and differentiates it by all coordinates,
  or only `doSystemWideDifferentiation` for the whole system as now. Proposed: the system-wide option as now, and
  the documentation of the user functions saying so; a flag only when a model needs it.
- **J4** `forODE2Connectors`, described as a flag for development to be removed: removed with deprecation when the
  connectors are on the interface.

## 7. Decisions (maintainer 2026-11-01)

- **J1 yes**, with the connectors and joints apart from the other objects:
  - A **connector or joint** always has its Jacobian through the connector interface: the derivative of its force or
    equations (L1) by the kinematics of its markers - by automatic differentiation, or by a *local* numerical
    differentiation of the marker kinematics, not of the system coordinates -, chained with the marker Jacobians
    (L2). Neither touches the system state, so both are computed in parallel; a Python user function in L1 makes the
    connector serial because of Python, not because of the Jacobian.
  - The numerical default of `CObject` - differentiating the object's residual by the system coordinates - is only for
    the objects that are not connectors: bodies, finite elements, ObjectGenericODE2 and the like.
  - **Today**: every joint has its constraint Jacobian analytically (`JacobianAE` has no numerical way except `forAE`).
    The connectors that are still differentiated numerically by CSystem: ObjectConnectorUser, the eight contacts
    (CircleCable2D, ConvexRoll, Coordinate, CurveCircles, FrictionCircleCable2D, SphereSphere, SphereTorus,
    SphereTriangle), ConnectorReevingSystemSprings, ConnectorRollingDiscPenalty (its interface Jacobian exists but is
    not declared), ConnectorHydraulicActuatorSimple, ConnectorCoordinateSpringDamperExt with limit stops or friction,
    ConnectorRigidBodySpringDamper in the intrinsic formulation, and every spring-damper with a Python user function.
    These go onto the interface (automatic, or local numerical of the marker kinematics) in a sub-step of their own.
- **J2 yes**: the numerical default serial, the analytic and automatic Jacobians in parallel.
- **J3 as recommended**: the system-wide numerical Jacobian for dependencies outside the local-to-global lists. To
  keep the Newton convergence for such models, the dependencies would have to be found: through the chain
  (node, object -> node) -> marker -> (load, sensor), and from the user functions - straightforward for a recording,
  which lists what it reads, harder for Python. An evaluation of its own, of low priority.
- **J4**: `forAE`, `forODE2` and `forODE2Connectors` are deprecated and do nothing once the revision has replaced
  their code paths; `doSystemWideDifferentiation` stays, the one numerical comparison. `jacobianConnectorDerivative`
  (on by default) is removed as well if it costs more code than it saves time - measured when the connectors are on
  the interface.
- **Parallel and serial**: if one architecture for all blocks gives simpler code, the ODE1 blocks go into the
  parallel line as well; §3.3 is a default, not a rule.

## 8. The derivative of the marker velocities by the coordinates (evaluation, RG9.11.8.1)

The local numerical Jacobian by the marker kinematics, tried for six connectors, degraded the Newton convergence of
the contacts with friction (log of RG9.11.8). This section evaluates the missing term before RG9.11.8 continues.

### 8.1 The term

A connector's ODE2 rows are $J\tp f$, with the force $f(p, v)$ of the marker kinematics: $p$ the positions and
rotations, $v = J(q)\,\dot q$ the velocities and angular velocities. Its Jacobian is

$$
\frac{\partial (J\tp f)}{\partial \dot q} = J\tp \frac{\partial f}{\partial v} J, \qquad
\frac{\partial (J\tp f)}{\partial q} = J\tp \frac{\partial f}{\partial p} J
  + \left.\frac{\partial (J\tp f)}{\partial q}\right|_{f}
  + J\tp \frac{\partial f}{\partial v} \frac{\partial v}{\partial q} .
$$

- The second term is the Jacobian derivative of the markers, `jacobianConnectorDerivative`.
- The third term, dv/dq, is in neither the automatic Jacobians of the connector interface (#2745) nor the rejected
  local numerical one.
- The numerical differentiation by the system coordinates contains all three.

$\partial v/\partial q = \sum_j \dot q_j\, \partial J_{\cdot j}/\partial q$ is built from the same second derivatives
of the kinematics as the Jacobian derivative: that one contracts them with $f$, this one with $\dot q$. For a position,
$\partial v/\partial q = \dot J$. It is zero where $J$ is constant.

Its size is $\partial f/\partial v$ times a velocity of the moving frame, such as $\omega \times r$:
- a damper $d$ gives $d\,|\omega|\,r$, small against the stiffness;
- regularized friction gives $\mu f_N / v_\mathrm{reg}\,|\omega|\,r$, large against the contact stiffness when
  $v_\mathrm{reg}$ is small - the case of the contacts and of the rolling disc.

### 8.2 Measured on the test suite

All 254 test models were run with the Newton iterations and Jacobians of every static and implicit solve counted; 136
of them are implicit models with connectors.

- **`forODE2Connectors = True`**: every connector is differentiated by the system coordinates, dv/dq included.
  - The counts differ in 13 of the 136. Ten are within 0.1-4 % in both directions (rotatingTableTest 26785 / 26377
    Newton iterations, userConnectorTest 6961 / 7161). mainSystemUserFunctionsTest gets worse, 8000 / 9727, and
    solverFailureMessageTest fails as it is meant to.
  - The one large change is coordinateSpringDamperExt.py, 4780 / 1409. It is a bug of the analytic Jacobian, which
    ignores factor0 and factor1 (#3204), not dv/dq.
  - So the dv/dq that today's automatic and analytic connector Jacobians neglect does not show in the Newton
    convergence of any test model. Those Jacobians cover the spring-dampers, the RigidBodySpringDamper and the
    torsional, Cartesian and linear spring-dampers.
- **`jacobianConnectorDerivative = False`**: the counts differ in 20 models, most by a few iterations.
  - The FFRF models need more iterations: objectFFRFTest 54 / 69, objectFFRFreducedOrderTest 799 / 840,
    objectFFRFTest2 183 / 190, objectFFRFreducedOrderAccelerations 74 / 80. symbolicUserConnectorsTest needs 2309 /
    2372.
  - The Jacobian derivative is therefore worth its code, and its second derivatives are the base of dv/dq (§8.4). It
    stays (J4).
- **The share of the Jacobian in the computation time** of the models with contacts and rolling discs, serial, by the
  timers of the solver. All of these connectors are differentiated numerically by CSystem today:

  | model | Jacobians | share |
  |---|---|---|
  | ConvexContactTest, contactSphereTorusMomentumTest, rollingCoinPenaltyTest | one per iteration | 68-69 % |
  | contactComparisonTest | one per iteration | 50 % |
  | ballBearingTest, symbolicUserContactTest | 531, one per iteration | 41-42 % |
  | carRollingDiscTest | one per iteration | 24 % |
  | createSphereTriangleContact, createSphereQuadContact, sphereTriangleTest2 | 456-834 | 16-19 % |
  | rotatingTableTest, mecanumWheelRollingDiscTest | 71, 39 | 0-1 % |

  The parallel assembly pays where full Newton is used. Modified Newton hides the Jacobian's cost but not its
  quality: without dv/dq, rotatingTableTest needed 678647 instead of 71 Jacobians.

### 8.3 The markers

| marker | dv/dq and dω/dq | how |
|---|---|---|
| MarkerNodeCoordinate, MarkerNodeCoordinates, MarkerObjectODE2Coordinates, MarkerNodeODE1Coordinate | zero | - |
| MarkerNodePosition | zero: the velocity is a velocity coordinate | - |
| MarkerBodyCable2DCoordinates, MarkerBodyCable2DShape, MarkerBodyBeamShape | zero: linear in the coordinates (not for the ALE cable, whose axial velocity moves the point along the slope) | - |
| MarkerBodyMass | no velocity | - |
| MarkerBodyPosition | zero on mass points and on the ANCF elements; nonzero on the rigid bodies and the geometrically exact beams | AD |
| MarkerBodyRigid | nonzero: RigidBody (Euler parameters, Tait-Bryan), RigidBody2D, the ANCF elements with a rotation by their slopes; also the geometrically exact beams and RigidBody on the Lie group node | AD; closed form for the Lie group node; none for the beams |
| MarkerNodeRigid | the rigid nodes (Euler parameters, Tait-Bryan; the Lie group node, whose velocity coordinates are the local angular velocity; 2D: zero) and the slope nodes | closed form per node, AD for the slopes |
| MarkerNodeRotationCoordinate | one component of a node's angular velocity | a row of the node's dω/dq |
| MarkerSuperElementPosition, MarkerSuperElementRigid | the reference frame and the modes of FFRF and ObjectGenericODE2, weighted over mesh nodes | none for now |
| MarkerKinematicTreeRigid | the recursion over the links | none for now; it has no Jacobian derivative either |
| MarkerBodiesRelativeTranslationCoordinate, MarkerBodiesRelativeRotationCoordinate | two bodies | none for now; no Jacobian derivative either |
| MarkerBodyUser | the weighted points of a user body | none for now |

"AD" means the existing nested automatic differentiation of AccessFunctionsAD.
- `JacobianTransposedTimesVectorDerivative` already forms $J$ as nested AD numbers for RigidBody, RigidBody2D, the
  ANCF elements and the slope nodes.
- Contracting the same pass with $\dot q$ instead of $f$ gives $\partial v/\partial q$ and $\partial\omega/\partial q$.
  That is one new function, for every body and node that has the AD access functions.

Every contact and rolling disc of the test suite acts on MarkerBodyRigid of a RigidBody, or on MarkerNodeRigid of an
Euler parameter node, so the AD and the rigid nodes cover them.

### 8.4 The way for RG9.11.8

- **The marker provides dv/dq and dω/dq** in the interface of the connector kinematics:
  - by the AD where the body has the AD access functions;
  - in closed form for the rigid nodes;
  - as zero where the Jacobian is constant.
  - A marker without it returns false. The connector is then differentiated numerically by CSystem, serially, as
    today, so nothing gets worse and the coverage grows marker by marker.
- **The connector requests the term.** It is computed only when requested:
  - The contacts with friction, the rolling disc and the connectors with user functions request it. A user function
    may depend on the velocities in any way: a RigidBodySpringDamper with a user function is numerical by CSystem
    today, so it has the term now and keeps it on the interface.
  - The spring-dampers do not request it (§8.2).
- **The chain** adds $J\tp\,\partial f/\partial v\,\partial v/\partial q$ to the automatic or local numerical Jacobian,
  with $\partial f/\partial v$ from the same evaluation of the force.
- **ConvexRoll** and the contacts on cables may stay numerical by CSystem, serially, for now.
- **Verification**: the Newton counts of the contact models equal those of the numerical Jacobian by CSystem within a
  few iterations, and rotatingTableTest is the sensitive case.

### 8.5 What the code does (RG9.11.8.2, RG9.11.8.3)

- `CMarker::GetVelocity_q(systemData, velocity_q, angularVelocity_q)`: dv/dq and domega/dq, omega global, at fixed
  velocities, in the columns of the marker's Jacobians; false where not provided. MarkerNodeCoordinate and
  MarkerNodePosition give zero (not for NodeUser); MarkerBodyPosition and MarkerBodyRigid ask the body
  (`CObjectBody::GetVelocity_q`: zero for ObjectGround, ObjectMassPoint, ObjectMassPoint2D); ObjectRigidBody,
  ObjectRigidBody2D and MarkerNodeRigid use `RigidBodyNodeVelocity_q` - the 2D node in closed form, Euler parameters
  and Tait-Bryan angles by forward differences of the rotation parameters through the pure functions of RigidBodyMath,
  in the forms the markers give: omega_local = Glocal theta_t, omega = A omega_local, v = A (omega_local x p). The Lie
  group nodes return false.
- `ItemConnectorInterface(markers, jacobian='numerical', velocityDerivative=True)`: the connector's Jacobian is the
  local numerical differentiation of its force by the marker kinematics (`ConnectorJacobianODE2RigidMarkersNumerical`,
  `...CoordinateMarkersNumerical`, the latter by the forces on both markers, so with the factors of a connector), and
  `RequestsVelocityDerivative()` adds J_i^T d(f_i)/d(v_k) [v_q; omega_q]_k. `CObjectConnector::ComputeLocalJacobianODE2`
  collects dv/dq of the markers first; a marker without it leaves the connector to CSystem, serially.
- Declared for ConnectorRollingDiscPenalty, ContactConvexRoll, ContactCoordinate, ContactSphereSphere,
  ContactSphereTorus, ContactSphereTriangle and ConnectorCoordinateSpringDamperExt (#3204: its analytic Jacobian
  without factor0/factor1 is removed). They are no longer deferred in the parallel assembly.
- Found on the way, for Euler parameters off the unit sphere - where the constraint of the Euler parameters is at
  velocity level (`useIndex2Constraints`, rollingCoinPenaltyTest: |e|^2 - 1 of 7e-7 after 5000 steps), and in every
  predictor of the time integration:
  - dv/dq of `G theta_t` differs from the derivative of the forms the markers give; the forms are used.
  - The numerical differentiation perturbed the local angular velocity as `A^T omega` with the perturbed global one;
    `A^T A` differs from the unit matrix by more than a step, so the derivative by omega was wrong by up to 5 times the
    stiffness. Only the increment is added now.
  - The derivative along e itself: section 8.6.
- Newton iterations against the numerical Jacobian by CSystem (all test models, the counts of every solve):
  rotatingTableTest 26785 / 26740, ballBearingTest 3057 / 3057, carRollingDiscTest 584 / 584,
  createSphereQuadContact 50384 / 50386, sphereTriangleTest2 9474 / 9475, ConvexContactTest 4330 / 4361,
  rollingCoinPenaltyTest 9568 / 11351 and contactSphereTorusMomentumTest 12297 / 14487 (both Euler parameters),
  coordinateSpringDamperExt.py 4780 / 1411 (#3204). Without dv/dq the first was 1083889 (RG9.11.8).

### 8.6 The direction along the Euler parameters (RG9.11.8.5)

- The rotation matrix of the Euler parameters is not normalized: `A - I` is quadratic in e, so `dA/de e = 2 (A - I)`
  and `d(A p)/de e = 2 (A - I) p`, also on the unit sphere. The chain of the marker Jacobians has no such direction
  (`G e = 0`, `Glocal e = 0`). The constraint of the Euler parameters (its node's algebraic equation `e^T e - 1`, in the
  blocks of the nodes with algebraic equations) does not remove it from the Newton increments: the predictor of the
  time integration leaves the sphere, so the first increment of every step has a part along e. rollingCoinPenaltyTest
  at index 3 (no drift): the first Newton contraction 3e-3 with the local Jacobian, 4e-5 with the system-wide one,
  12036 against 9654 iterations.
- `CMarker::GetRadialKinematics(systemData, position_r, rotation_r, radial_q)`: the derivatives of the marker's
  position and rotation matrix along e, and the row `radial_q = e^T/(e^T e)` at the Euler parameters that gives the
  part of a change of the coordinates along e; false if the marker has no such direction. MarkerBodyPosition (rotation
  zero), MarkerBodyRigid (by the local rotation of the frame) and MarkerNodeRigid, through
  `CObjectBody::GetRadialKinematics` of ObjectRigidBody and `RigidBodyNodeRadialKinematics`, true for NodeRigidBodyEP.
- The chains of the connector interface add `J_i^T d(f_i, tau_i)/d(eps) radial_q,k`, one more evaluation of the force
  per such marker at the kinematics moved by eps = 1e-7 along e (the global velocity and angular velocity kept): the
  automatic Jacobians on position and rigid markers and the numerical ones on rigid markers.
  `CObjectRigidBody::GetJacobianTransposedTimesVectorDerivative` adds `Glocal^T (p x 2 (A - I)^T f) radial_q`, the part
  of `d(A^T f)/dq` that `A^T fTilde G` lacks.
- The constraints on the connector interface (position level): `dg/d(eps) radial_q,k` - rigid markers from the equations
  at the moved frame, position markers from the automatic derivatives by the positions - into
  `temp.localJacobianAE_ODE2radial`, which AddLocalJacobianAE adds to the rows of the equations only. The reaction forces
  of the residual, `J^T (dg/dp)^T lambda` per marker, have no part along e, so the transposed block keeps C_q without
  it; with it, restartFileTest and userJointTest did not converge.
- The system-wide AE Jacobian (#3206) had both faults: it added `dC/dq^T` as the block of the reaction forces, and with
  the factor 1, as JacobianAE did not pass the factors of the solver. The block is now the reaction forces of the unit
  Lagrange multipliers (`ComputeODE2ProjectedReactionForces`), with the factors of the solver.
- The local Jacobians equal the system-wide one: rollingCoinPenaltyTest 2e-8 of the largest entry (1e-2 before);
  test_systemJacobians.py on Euler parameters for the spring-dampers, the joints and the contacts.
- Newton iterations of all test models, the counts of every solve, against RG9.11.8.3: 1167234 / 1154574 (Jacobians
  468746 / 458690), 25 models fewer and 11 more. Fewer: rollingCoinPenaltyTest 11351 / 9568 (= system-wide),
  contactSphereTorusMomentumTest 14487 / 12283, restartFileTest 6294 / 4374, mainSystemExtensionsTests 18861 / 15446,
  bricardMechanism 6308 / 5969 (Jacobians 18 / 6), sliderCrank3Dbenchmark Jacobians 322 / 39. More, each the same as
  with the system-wide Jacobian: sliderCrank3Dtest 2000 / 3172, carRollingDiscTest 584 / 696, userJointTest Jacobians
  868 / 1285, systemUtilitiesTest Jacobians 27 / 94. The results change by 1e-13 to 3e-6 in 38 test models and 3 mini
  examples (restartFileTest 3e-5, whose step size adapts at the contact).
- Without the direction: markers without GetRadialKinematics on Euler parameter nodes (MarkerSuperElementRigid with
  an Euler parameter reference node, the user markers), connectors and constraints on the path of the marker data,
  and the access functions of the rigid body by automatic differentiation (off by default).

### 8.7 The other connectors (RG9.11.8.4)

- ConnectorHydraulicActuatorSimple: the ODE2 rows on the connector interface of position markers, the force by
  automatic differentiation (ComputeActuatorForce a template, the pressures constants); the block by the pressures
  analytic, `e [-A0, A1]` projected by the marker Jacobians, which `CObject::ComputeJacobianODE2_ODE1(systemData, temp,
  jacobian)` finds in temp.markerTemp after the object's ODE2 Jacobian; the ODE1 rows, the two pressure equations,
  numerical by CSystem as for every connector. The ODE2 rows are assembled in parallel.
- ConnectorRigidBodySpringDamper with useIntrinsicFormulation: the local numerical differentiation of the force by the
  marker kinematics (`ConnectorJacobianODE2RigidMarkersNumerical`), as the logarithm of SE(3) has no automatic
  derivative at the zero relative motion; the other formulation keeps the automatic one.
- Numerical by CSystem, serially (maintainer 2026-11-02): ConnectorReevingSystemSprings, the cable contacts and the
  connectors with Python user functions.
- Newton iterations as before (hydraulicActuatorSimpleTest 962, rigidBodySpringDamperIntrinsic 4115), the results
  within 3e-11.

## 9. The parallel assembly (RG9.11.5)

- The ODE2 rows (RG9.11.5.1, #1203): `listComputeObjectODE2LhsNoUF` in `ParallelFor`, ComputeLocalJacobianODE2 with
  `analyticOnly` and AppendLocalJacobianODE2 per thread; the objects that need a numerical block (deferred) and those
  with Python functions serially.
- The AE rows (RG9.11.5.2, #1202): `objectsConstraintWithAENoUF` and `nodesODE2WithAE` in one `ParallelFor`,
  ComputeObjectJacobianAE or ComputeNodeJacobianAE and AppendLocalJacobianAE per thread (the blocks of
  AddLocalJacobianAE, `LocalJacobianAEBlocks`, as triplets); the bodies with algebraic equations (ObjectGenericDAE,
  which may have Python functions) and the constraints with Python functions serially.
- Both above `parallel.multithreadedLowerLimitJacobians` items (default 20), split by `GetTaskSplit` into
  `taskSplitTasksPerThread` (16) tasks per thread of equal numbers of items, which the task manager hands to the
  threads as they become free.
- A chain of rigid bodies on Euler parameters with alternating revolute and spherical joints, sparse: 2000 joints
  (4000 items with the nodes) 2.44 / 1.26 / 0.65 / 0.34 ms for 1 / 2 / 4 / 8 threads, 90 % of the ideal at 8 threads;
  20 joints 0.03 / 0.01 ms for 1 / 8 threads, no loss at the default limit. The test suite with 4 threads and the limit
  1 passes.
- A split by a cost per item (#858) is not implemented: at 16 tasks per thread taken as threads become free, items of
  different cost even out unless a single task exceeds the share of a thread; `ParallelFor`'s own cost argument only
  switches small loops to serial, which the lower limits do. The ODE2 rows of the rigid bodies on Euler parameters
  are numerical and serial (1.6 ms for the 2000 bodies, against 0.34 ms for their constraints at 8 threads).
