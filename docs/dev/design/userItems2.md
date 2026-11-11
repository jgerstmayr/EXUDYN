# Built-in items as symbolic user items: evaluation (revision2026c step RG14.6.1, #3200)

A design note of open steps: per built-in item, what it computes, which user item reproduces it, what the recording
allows, the effort, and the interface extensions it would need. It leaves the repository when the steps close
(WORKFLOW.md §8.4); what is decided goes into docs/dev/internals/userItems.md, whose terms it uses.

## 1. The request

User items built by `UserItemCreator` (exudyn/symbolics/userItems.py) - ObjectUserBody, ObjectConnectorUser with the
forceUserFunction of the marker states, ObjectJointUser, ObjectGenericDAE, data nodes - that reproduce built-in items,
recorded by `Record()` for the symbolic system and the uniform evaluation on the GPU (group RG14):

- from #3200: ObjectJointGeneric, ObjectConnectorHydraulicActuatorSimple, ObjectConnectorReevingSystemSprings,
  ObjectFFRFreducedOrder;
- the contact primitives sphere-triangle, cylinder-triangle, cylinder-cylinder, cylinder-sphere (docs/dev/internals/contactPrimitives.md);
- ObjectBeamGeometricallyExact (3D);
- ObjectKinematicTree with its markers MarkerKinematicTreeRigid;
- ObjectConnectorRollingDiscPenalty.

Each user item is checked against the built-in item, as symbolicUserItemsCreateExamplesTest.py does for the existing
ones (userItems.md §7).

## 2. What a recording must satisfy

From uniformEvaluation.md §3, §6 and userItems.md §4, the rules for every item below:

- **R1 fixed structure per kind.** Sizes, the number of markers, loops and the branches of Python `if` are fixed when
  the function is recorded; a value that changes them - a number of sheaves, the topology of a tree, the constrained
  axes - is a Python constant of the recording, and each value is a variant of the kind with a recording of its own
  (`_AddUserItem(..., variant=...)`, #3203). Numbers that do not change the structure are `parameters`.
- **R2 branches as selects.** A branch on data is `IfThenElse`, `min`, `max`, `abs`; both branches are computed
  (predication on the GPU). The branch not taken must not produce inf or nan in what is selected: a denominator is
  guarded inside the branch, `IfThenElse(c, x/IfThenElse(c, y, 1.), 0.)`, as `_SphereSphereContactFunctions` does.
- **R3 iterations unrolled.** An iteration runs a fixed number of steps, unrolled at the recording; no exit on
  convergence, no search over a set of variable size. A closed form is preferred where it exists.
- **R4 no exceptions.** A check of the built-in item that raises during the computation (`CHECKandTHROW`, `SysError`)
  is no operation of a recording; it moves into the Create function, at the arguments, or is left out.
- **R5 no mbs inside.** A recording that reads mbs is refused by `SymbolicSystem` (symbolicSystem.md internals §1); a
  user function of the built-in item (an `offsetUserFunction`, a `forceUserFunction`) is taken only if it can be
  recorded, called with symbolic arguments, and refused by `_Refuse` otherwise.
- **R6 Jacobians.** A user body or ObjectGenericDAE has the one-pass derivative of its recordings; a user connector a
  numerical one in C++ and an exact one in `SymbolicSystem` (`_ConnectorJacobian`); a user joint the
  `JointJacobianUserFunction` (#3235). The built-in analytic Jacobians are not reproduced.
- **R7 comparison.** The same formulas in the same order give the bits of Exudyn; a different order, a fixed number of
  iterations instead of a tolerance, or a different Jacobian (other Newton iterates) give agreement to a tolerance.

## 3. The interfaces at hand

| user item | what it gives | what it lacks for the items below |
|---|---|---|
| ObjectUserBody | a body on any existing nodes: mass matrix (or constant), forces, position, rotation, their Jacobians, mass integral, output variables | ODE1 and algebraic coordinates; an access function per part (mesh node, link): the access functions get only `localPosition` |
| ObjectConnectorUser, forceUserFunction | 2 markers by their states `HT0, twist0, HT1, twist1`, a data node of any size, the post-Newton step, recordable | ODE1 coordinates; more than 2 markers (`Assemble` refuses); coordinate markers |
| ObjectConnectorUser, markerAccessUserFunction | any number of markers of any type, forces at points it computes | not recordable (side effects) |
| ObjectJointUser | 1 to 6 equations of the two marker frames, `parameters`, the Jacobian by `JointJacobianUserFunction` | the multipliers in the equations (a free axis as $\lambda_i = 0$) |
| ObjectGenericDAE | ODE2, ODE1, AE and data coordinates of any nodes, symbolic, in `SymbolicSystem` (#3190) | markers: it acts on node coordinates, its kinematics written by hand per node type |
| MarkerBodyUser | a point of several bodies by position and rotation functions with `parameters` (#3065) | Python only: no recording, not assembled by `SymbolicSystem` (to check: whether its functions can be symbolic already) |

## 4. The items

### 4.1 ObjectConnectorRollingDiscPenalty

**Built-in** (`CObjectConnectorRollingDiscPenalty.cpp`, `ComputeContactForces`, `ComputeSlipForce`,
`ComputeConnectorForceRigid`, `PostNewtonStep`). Marker 0 the ground frame, marker 1 the disc, both rigid; a data node
of 3: the slip velocity [lateral, longitudinal] and the gap of the last post-Newton step. With the disc axis
$\mathbf w_1 = \mathbf A_1 \mathbf a$, the plane normal $\mathbf n_0 = \mathbf A_0 \mathbf n$,
$\mathbf w_2 = \mathbf w_1\times\mathbf n_0/|\cdot|$, $\mathbf w_3 = \mathbf w_1\times\mathbf w_2$,
$\mathbf w_l = \mathbf n_0\times\mathbf w_2$: contact point $\mathbf p_C = \mathbf p_1 + r\mathbf w_3 - \mathbf p_0$,
its velocity $\mathbf v_C = \mathbf v_1 + \boldsymbol\omega_1\times r\mathbf w_3 - (\mathbf v_0 + \boldsymbol\omega_0\times\mathbf p_C)$;
while the gap of the data is negative, $f_n = -(k\,\mathbf p_C\cdot\mathbf n_0 + d\,\mathbf v_C\cdot\mathbf n_0)$ and
the slip force $-\varphi |f_n| \mathbf e$ along the slip direction $\mathbf e$ of the data, $\varphi$ the
regularization in `dryFrictionProportionalZone` (linear or quadratic), then the friction matrix
$\mathrm{diag}(\mu_i + \mu_{v,i}|\mathbf v_s|)$, optionally rotated by `dryFrictionAngle`; `rollingViscousFriction`
adds a force against the trail velocity (`RollingDiscTrailVelocity`, #926). The post-Newton step: the error
$|g\,k|$ plus the slip force on a change of the contact, the change of the slip force otherwise; the data set to the
current slip velocity and gap; no recommended step size. The Jacobian by the connector interface (#2745).

**User item**: ObjectConnectorUser on the two rigid markers, `requestedMarkerType` Position | Orientation, a data node
of 3, the forceUserFunction and the postNewtonStepUserFunction of the marker states - the existing interface covers it.

**Approach**: the formulas above with `exudyn.symbolics.kinematics`; the contact and the regularization zone by
`IfThenElse` on the data (R2), the norms of the slip velocities guarded (the built-in gives no friction when the data
slip velocity is 0, which the guard reproduces); `useLinearProportionalZone` and `dryFrictionAngle == 0` Python
constants (variants); the trail velocity closed form. The post-Newton step returns $[\varepsilon, -1, v_{s,l}, v_{s,t}, g]$.

**Limits**: $\mathbf w_1 \parallel \mathbf n_0$ (the disc lying flat) divides by zero, as in the built-in item; a guard
there changes nothing that runs. No comparison of the forces at the switch beyond the tolerance of the iteration.

**Effort LOW.** No extension. First item: it needs nothing new and exercises the data node and the post-Newton step
of a rigid-marker connector on the GPU path.

### 4.2 ObjectJointGeneric

**Built-in** (`CObjectJointGeneric.cpp`, `JointGenericEquations`; `src/System/JointRelativeFrame.h`,
`JointRelativeFrameEquations`). Six algebraic coordinates; the joint frames $\mathbf A_0' = \mathbf A_0\mathbf R_{m0}$
(times the rotation of the `offsetUserFunction`), $\mathbf A_1' = \mathbf A_1\mathbf R_{m1}$; translations: all three
constrained $\mathbf p_1 - \mathbf p_0$ (global), else $\mathbf a_i\cdot(\mathbf p_1-\mathbf p_0)$ for the constrained
axes $\mathbf a_i$ of $\mathbf A_0'$; rotations: three constrained $z_0\cdot y_1,\ z_0\cdot x_1,\ x_0\cdot y_1$ (or the
triple products of `alternativeConstraints`), two the free axis of $\mathbf A_0'$ against the locked axes of
$\mathbf A_1'$, one the two free axes. **A free axis has the equation $\lambda_i = 0$**, so `constrainedAxes` can
change after `Assemble`. The derivatives by the 12 marker variations analytic (`ComputeConstraintEquationsRigidDerivatives`).

**User item**: ObjectJointUser on the two rigid markers with `numberOfConstraints` = the number of constrained axes,
`CreateGenericJoint` of the creator with the markers of `MainSystemCreateGenericJoint`.

**Approach**: the equations of `JointRelativeFrameEquations` for the constrained axes only - the free multipliers of
the built-in item are zero, so the reduced system has the same solution and fewer unknowns. `constrainedAxes` and
`alternativeConstraints` are Python constants: one variant per pattern (in practice a few: revolute, prismatic,
cylindrical, rigid, spherical). $\mathbf R_{m0}$, $\mathbf R_{m1}$ and a constant offset are `parameters`; an
`offsetUserFunction` is recorded with symbolic `t` if it can be (R5). The Jacobian by `JointJacobianUserFunction`.

**Limits**: `constrainedAxes` cannot change after `Assemble` (a new variant would change the number of equations);
the outputs `ForceLocal`, `TorqueLocal` have $n_c$ multipliers instead of 6 and must be mapped for a comparison;
`offsetUserFunction_t` is not needed (the user joint's index-2 form is the Jacobian times the velocities).

**Effort LOW.** No extension. Extension **E4** (the multipliers in the constraintUserFunction) would give the form
$\lambda_i = 0$ and run-time switching; it is not needed for the comparison.

### 4.3 ObjectConnectorHydraulicActuatorSimple

**Built-in** (`CObjectConnectorHydraulicActuatorSimple.cpp`, `ComputeActuatorForce`, `ComputeBulkModulusPerVolume`,
`ComputeValveFlow`, `ComputeODE1RHS`, the Jacobians `ComputeJacobianODE2_ODE1`, `ComputeJacobianODE1RHSConnector`).
Two position markers and a NodeGenericODE1 of the two chamber pressures $p_0, p_1$. Length $L$, direction
$\mathbf e$, stroke $s = L - L_{off}$, $\dot s = \mathbf e\cdot(\mathbf v_1-\mathbf v_0)$; the force
$f = -p_0A_0 + p_1A_1 + d\,\dot s$ along $\mathbf e$; the ODE1 equations

$$\dot p_0 = \frac{K_0}{V_0}\left(-A_0\dot s + Q_0\right),\quad \dot p_1 = \frac{K_1}{V_1}\left(A_1\dot s + Q_1\right),\quad Q_i = o_i Q_n \operatorname{sgn}(\Delta p_i)\sqrt{|\Delta p_i|},$$

$\Delta p_i = p_s - p_i$ for an opening $o_i \ge 0$, else $p_i - p_t$; with `useChamberVolumeChange` the volumes
$V_0 = V_{h0} + A_0 s$, $V_1 = V_{h1} + A_1(s_{max} - s)$ and effective bulk moduli of oil, cylinder and hose. The
valve openings are parameters, set by a preStep function in the examples.

**User item**: two ways.

- **Now, without an extension**: ObjectGenericDAE on the ODE2 nodes of both bodies and the NodeGenericODE1, its
  forceUserFunction and rhsUserFunction written with the marker kinematics of the node types (for a user rigid body
  the position of `_UFrigidBodyPosition`); internals symbolicSystem.md §7 prescribes this for items that couple nodes.
  It bypasses the markers: a body on the ground or a marker of another kind needs its own kinematics.
- **With E1**: ObjectConnectorUser with an ODE1 node: `forceUserFunction` gets the ODE1 coordinates, an
  `ode1RhsUserFunction(mbs, t, itemNumber, HT0, twist0, HT1, twist1, y, dataCoordinates, parameters)` returns
  $\dot{\mathbf y}$; the connector stays a connector on markers.

**Approach**: the formulas above; $o_i \ge 0$ an `IfThenElse` of a parameter (R2); `useChamberVolumeChange` and the
zero bulk moduli Python constants; the openings in `parameters`, changed by `SetObjectParameter(item, 'parameters',
...)` or bound to variables of `mbs.symbolicVariables`.

**Limits**: $\sqrt{|\Delta p|}$ has an infinite derivative at $\Delta p = 0$; the built-in Jacobian bounds it by a
minimum difference ($10^{-8}\max(|p_s|,1)$), the one-pass derivative of a recording does not - the function needs the
same regularization written into it (e.g. $\sqrt{|\Delta p| + \epsilon}$ minus its offset), which changes the result by
$O(\sqrt\epsilon)$; the check of vanishing volumes (R4) goes. Explicit solvers take ODE1 in their stages (SolveExplicit).

**Effort MEDIUM** with E1 (MEDIUM for E1 itself: C++ of ObjectConnectorUser - `GetODE1Size`, `ComputeODE1RHS`, the
Jacobians ODE2_ODE1, ODE1_ODE2 numerically -, the user function def, the composition in `SystemModel`);
LOW-MEDIUM as ObjectGenericDAE. Recommended: with E1, which serves every connector with internal dynamics (an
actuator, LuGre or Bouc-Wen friction, a motor current), none of them built in as a connector today.

### 4.4 ObjectConnectorReevingSystemSprings

**Built-in** (`CObjectConnectorReevingSystemSprings.cpp`, `ComputeReevingGeometry`, `ComputeODE2LHS`,
`ComputeForce`; `src/Linalg/Geometry.h`, `CommonTangentOf2Circles`). $n$ rigid markers (the sheaves, radius $R_i$,
axis $\mathbf a_i$ in the marker frame) and optionally two coordinate markers. Per segment $i$: the tangent points
$\mathbf r_A, \mathbf r_B$ of the circles of sheaves $i$, $i+1$ by a Newton iteration in the two angles (start
$\pi/2$, at most 8 steps, tolerance $10^{-12}$), the free length $|\mathbf p_B + \mathbf r_B - \mathbf p_A - \mathbf
r_A|$; on each inner sheave the wrapped arc $\varphi R$, $\varphi$ by `atan2` from the last tangent point, plus
$2\pi$ if negative. $\dot L$ from the velocities of the tangent points, without the rate of the wrapped arc (a
simplification noted in the code). The force
$F = (EA(L - L_0) + DA(\dot L - \dot L_0))/L_0$, with `regularizationForce` $F_r > 0$ a compression
$F_r\tanh(F/F_r)$; $L_0$ = `referenceLength` plus the coordinate markers times `coordinateFactors` (winches). The
force acts on both sheaves of each segment with the moments $\mathbf r\times\mathbf f$, plus torsional and shear damping.
No data coordinates; the Jacobian numerical.

**User item**: ObjectConnectorUser on $n$ rigid markers - the forceUserFunction takes only 2 (`Assemble` refuses
more); the markerAccessUserFunction takes $n$ but is not recordable. The rope force couples all segments by the total
length, so the item cannot be split into 2-marker connectors.

**Approach with E2** (the forceUserFunction of $n$ markers: lists of frames and twists, $6n$ forces and torques
returned): $n$ a Python constant (a variant per number of sheaves); the Newton iteration unrolled with 8 steps from the
same start (R3) - or the closed form of coplanar sheaves with parallel axes, the common case; the `atan2` correction
and `tanh` by `IfThenElse`; $R_i$, $\mathbf a_i$, $EA$, $DA$, $L_0$ as `parameters`. The coordinate markers have no
frame: open whether E2 passes a coordinate marker as its value and rate (in the translation and velocity of an HT and
a twist), or the winch is a rigid marker of the drum with the turns in a data node, as the torsional spring-damper
counts them (#3218).

**Limits**: 8 steps instead of the tolerance agree to the tolerance where the built-in converges in fewer (R7); the
built-in's check of a failed tangent (R4) goes; the Jacobian of the recording contains the derivative the built-in
$\dot L$ leaves out only if $\dot L$ is taken as $\partial L/\partial\mathbf q\,\dot{\mathbf q}$ - for the comparison
the same simplification is written. In `CommonTangentOf2Circles` the length of $\mathbf n_B$ is taken from $\mathbf
n_A$ (Geometry.h line 521) and the count of the iterations reports a convergence in the 8th step as failure and 8
unconverged steps as success (lines 543, 586): sheaves with non-parallel axes will differ from a correct user item
until that is fixed (an issue of its own, rule 9).

**Effort MEDIUM** (with E2; E2 itself MEDIUM: a second user function def, `Assemble` and the composition of $n$ marker
states in `SystemModel`). E2 also serves belt and chain drives over several pulleys and any user element of a rope or
cable on rigid markers.

### 4.5 Contact primitives

**Built-in** (docs/dev/internals/contactPrimitives.md; `src/Linalg/GeometryConvex.h`; `CObjectContactSphereCylinder.cpp`,
`CObjectContactSphereTriangle.cpp`, `CObjectContactCylinderCylinder.cpp` with `CylinderTrianglePoints`,
`CylinderCylinderPoints`, `FacePoints`; the law `ContactPrimitiveForce.h`, the points `ContactPrimitivePoints.h`).
Distances by GJK and EPA, closed forms where they exist (`DistanceSphereCylinder`, the infinite plane); up to 4
contact points for the cylinders, the number of points chosen by the geometry; a data block of 3 per point (gap,
tangential velocity, impact velocity); the law of `impactModel` and regularized Coulomb friction; a
`contactForceUserFunction` (#3324) for the law, while the geometry stays C++.

**User item**: ObjectConnectorUser on two rigid markers with a data node of 3 per contact point, as the sphere-sphere
contact of the creator (`_SphereSphereContactFunctions`, #3197, #3219). The law and the post-Newton step per point are
those of the sphere-sphere contact, moved into one function of (gap, normal velocity, tangential velocity, data block)
that every contact item of the creator calls - a refactoring, no extension. The post-Newton step sums the errors of
the points and takes the minimum recommended step size (`min`, R2).

| pair | geometry in a recording | effort | recommendation |
|---|---|---|---|
| sphere-cylinder | closed form: the center in the cylinder frame, radial distance $\rho$ and axial $z$; side, cap, rim and inside by selects; the radial direction guarded at $\rho = 0$ (as `Cylinder::SignedDistance`) | LOW | do; check against `DistanceSphereCylinder` to the bit where the formulas agree |
| sphere-triangle | closed form: the closest point of the triangle, the face if the projection is inside (3 edge tests), else the minimum over the 3 clamped edge distances (`min`); `includeEdges` a Python constant | LOW | do; with `infinitePlane` the plane only |
| cylinder-triangle, `infinitePlane` | closed form: the support point against the normal and the points of `FacePoints` (the deepest rim point, the other end of the side line, two rim points at $\pm 120°$), the blend below a tilt of $10^{-3}$ rad as a select; 4 points always, a point above the plane has a positive gap and no force | MEDIUM | do |
| cylinder-triangle, finite | GJK/EPA and the clipping of the points to the triangle (a point beyond moves along its radius to the border) | HIGH | not recordable as built; rim points clipped by selects possible, the closest point of GJK not |
| capsule-capsule (`isCapsule0/1`) | closed form: the closest points of two segments (clamped, with the parallel case as a select), the two ends of the interval along the axis | MEDIUM | do, as the recordable variant of the cylinders |
| cylinder-cylinder | side on side by the segments as above; cap on face, rim on rim, rim on side by GJK/EPA | HIGH | not recommended; a GJK of a fixed number of steps with Johnson's subsimplex by selects is possible in principle, open |

**A mesh of triangles** (a sphere on a surface): one connector per triangle, the triangle points among the
`parameters` - many items of one kind, one recording, the width the uniform evaluation wants. Without a broad phase
every pair is evaluated in every step; for a GPU test that is the intended load, for a model GeneralContact stays the
tool.

**Limits**: the number of points of a configuration varies in the built-in item; in the recording it is the maximum,
with inactive points by their gap (R1, R2). Witness points that jump (cylinder caps) jump in the recording as well.

**Effort**: LOW to MEDIUM for the four recommended pairs; no extension.

### 4.6 ObjectBeamGeometricallyExact (3D)

**Built-in** (`CObjectBeamGeometricallyExact.cpp`). Two rigid-body nodes (any rotation parameters, also
NodeRigidBodyRotVecLG with Lie group integration). The relative motion $\mathbf h = \log_{SE(3)}(\mathbf H_0^{-1}\mathbf
H_1)$ and $\mathbf h_0$ of the reference configuration (`ComputeIncrementalMotion`); the strain
$\boldsymbol\varepsilon = (\mathbf h - \mathbf h_0)/L$, the stress $\mathbf s = \mathbf K\boldsymbol\varepsilon$ with
$\mathbf K = \mathrm{diag}$(axial-shear, torsion-bending stiffnesses); the nodal residuals
$\mathbf r_i = \mathbf T_i\tp\mathbf s$ with $\mathbf T_0 = -\mathbf T_{SE3}^{-1}(-\mathbf h)$,
$\mathbf T_1 = \mathbf T_{SE3}^{-1}(\mathbf h)$, mapped by $\mathbf A_i$ and $\mathbf G_{local,i}\tp$
(`ComputeODE2LHS`). The frame at $x$ is $\mathbf H_0\exp_{SE(3)}(S_1(x)\mathbf h)$ (`GetLocalPositionFrame`), the
velocities and Jacobians the linear interpolation of the nodal ones (`GetPositionJacobian`). Mass matrix
element-consistent (#1273): translational $\rho A L[\frac13,\frac16;\frac16,\frac13]$, rotational
$\mathbf G_i\tp\int S_iS_j\mathbf R\mathbf J\mathbf R\tp\,dx\,\mathbf G_j$ by 2 Gauss points, depending on the
configuration; or lumped by the global flag `exu.special.beams.geometricallyExactLumpedMass`; the quadratic velocity
vector accordingly. The Jacobian analytic for the stiffness with central differences of $\mathbf T_{SE3}^{-1}$.

**User item**: ObjectUserBody on the two rigid nodes, as the user ANCF beam and cable (userANCFBeam3DTest.py,
userANCFCable2DTest.py) but on rigid-body nodes; the markers MarkerBodyPosition and MarkerBodyRigid at a local
position work by the access functions.

**Approach**: `kinematics.HT.LogSE3()` exists symbolically with its series below $\varphi = 0.02$; needed in addition a
symbolic $\mathbf T_{SE3}^{-1}$ with the same series (exudyn/lieGroupBasics.py has it for numbers) and
`SetExpSE3` for the frame at $x$ (exists). $\mathbf h_0$ from `qRef` in the recording, or 6 `parameters` computed at
creation (cheaper). Stiffnesses, $\mathbf J$, $\rho A$, $L$ as `parameters`; lumped or consistent a Python constant.
The positionJacobianUserFunction as the built-in's linear interpolation, so that the velocities agree.

**Limits**: Rxyz and Euler parameter nodes only - the user items refuse Lie group nodes (`CreateRigidBody`), and the
beam is often used with NodeRigidBodyRotVecLG; the comparison runs on Euler parameters. The regular branches of
`LogSE3` and $\mathbf T_{SE3}^{-1}$ divide by $\sin$ and $\varphi^2$, guarded by R2. A mass matrix depending on $\mathbf q$
rules out the stored constant mass matrix; `MassMatrixBlocks` is per element, but two elements share a node, so the
inverse per body does not apply (symbolicSystem.md §10) - an explicit GPU run needs the global solve or the lumped
mass with Euler parameters (block-diagonal per node only if the elements' blocks are assembled per node, open).

**Effort MEDIUM.** No interface extension; the symbolic $\mathbf T_{SE3}^{-1}$ in exudyn.symbolics.kinematics.

### 4.7 ObjectKinematicTree

**Built-in** (`CObjectKinematicTree.cpp`, `ComputeMassMatrixAndODE2LHS`, `AddJointForces`, `ComputeRigidBodyMarkerDataKT`,
`ComputeJacobian`). One NodeGenericODE2 of the $n$ joint coordinates and optionally a rigid base node (#1549). Per link:
joint type (revolute or prismatic about X, Y, Z), parent, the constant joint transformation and offset, mass, center
of mass, inertia. The forces by recursive Newton-Euler: from the base to the leaves the placements
(`RelativePlacement`), velocities and bias accelerations with gravity as an acceleration of the base, the link forces
$\mathbf I\mathbf a + \mathbf v\times^*\mathbf I\mathbf v$; from the leaves to the base the joint forces. The mass
matrix by the composite rigid bodies (`Inertia::AddChild`) and the walk to the root per joint. Added: constant joint
forces, P and D control towards offsets, link forces and torques in global coordinates, a `forceUserFunction(mbs, t,
itemNumber, q, q_t)`. The Jacobian by forward differences of the forces, with $\partial(\mathbf M\ddot{\mathbf q})/
\partial\mathbf q$ (`jacobianMassMatrixDerivative`, #3211). Markers MarkerKinematicTreeRigid: link number and local
position, the frame of the chain and its geometric Jacobian. Sensors SensorKinematicTree.

**User item**: ObjectUserBody on the NodeGenericODE2 (and the base node), its mass matrix the composite-rigid-body
recording and its forces the Newton-Euler recording with $\ddot{\mathbf q} = 0$, minus the joint forces; UserItemCreator
`CreateKinematicTree` with the arguments of `MainSystemCreateKinematicTree`.

**Approach**: the topology - `linkParents`, `jointTypes` - is a Python constant (R1): one variant per tree, the loops
over the links unrolled; per link 22 numbers (mass, center of mass, inertia, joint transformation, offset) and the
control vectors, gravity, base offset and link loads are `parameters`. The forward kinematics of a link frame is the
same code for the markers. Sizes: the forces $O(n)$, the mass matrix $O(n\cdot\text{depth})$ operations - for a robot
of 6 to 7 joints some thousand scalar operations, a few hundred per link.

**Missing: a marker on a link.** The access functions of ObjectUserBody get `localPosition` only. Options:
(a) **E3**, MarkerBodyUser recorded and assembled by `SymbolicSystem`, with the link a Python constant of its variant
(one recording per link of a tree) or selected by a one-hot weight vector among its `parameters` (no branch,
$12n$ operations per marker); (b) an index of a body part in the access functions of ObjectUserBody (a new argument of
5 user function defs, which every user body would carry). Recommended: (a), as it serves ObjectFFRFreducedOrder as well
and leaves the body interface alone.

**Limits**: Lie group base nodes refused (as 4.6); the derivative of the mass matrix is not part of a user body's
Jacobian (userItems.md §2), so the Newton iterates of implicit solvers differ from `jacobianMassMatrixDerivative=True`
(R7); the user's `forceUserFunction` only if recordable (R5); SensorKinematicTree is replaced by SensorMarker on the
user markers; the inverse dynamics of the item is not reproduced. The dense $n\times n$ mass matrix is one block of the
inverse per body for explicit runs on the GPU - one batched solve per tree.

**Effort HIGH.** Valuable: robots and their ensembles (sampling-based planning and control, RG21) are the case the
uniform evaluation fits best. Needs E3.

In `AddJointForces` the forceUserFunction gets `tempVector`, which holds the coordinates only when
`ComputeMassMatrixAndODE2LHS` is called without `qTotal` (line 222 against lines 560-567); the call with `qTotal` (the
Jacobian, line 105) is used only without a forceUserFunction (`GetAvailableJacobians`), so it does not act today.

### 4.8 ObjectFFRFreducedOrder

**Built-in** (`CObjectFFRFreducedOrder.cpp`, `ComputeMassMatrix`, `ComputeODE2LHS`, `ComputeQuadraticVelocityVector`
(#3212), `GetAccessFunctionSuperElement`; definition: the equations of Zwölfer and Gerstmayr). A rigid-body node
(node 0, usually Euler parameters) and a NodeGenericODE2 of the $n_m$ modal coordinates $\boldsymbol\zeta$. The mass
matrix of the blocks $\mathbf M_{tt} = m\mathbf I$, $\mathbf M_{tr}$, $\mathbf M_{tf}$, $\mathbf M_{rr}$, $\mathbf M_{rf}$,
$\mathbf M_{ff}$ from seven constant inertia-like matrices (up to $3n_m\times 3n_m$) and $\boldsymbol\zeta\otimes\mathbf I$,
the reduced stiffness and damping, the quadratic velocity vector; `computeFFRFterms`, user functions for mass and
force. Markers MarkerSuperElementPosition and MarkerSuperElementRigid on weighted mesh nodes,
$\mathbf p = \mathbf p_{ref} + \mathbf A\sum_i w_i(\mathbf x_i + \boldsymbol\Psi_i\boldsymbol\zeta)$; drawing of the deformed
`triangleMesh`; superelement outputs per mesh node.

**User item**: ObjectUserBody on both nodes; mass matrix and forces as recordings of the formulas of the definition.

**Approach**: $n_m$ a Python constant (a variant per mode basis size); the seven matrices either Python constants of
the recording (one recording per body - acceptable, as there are few such bodies) or `parameters`. As parameters,
`ParameterLayout.ToDict` gives a list of components, so a $60\times 60$ matrix ($n_m = 20$) becomes 3600 component
nodes per evaluation; **E5**, a matrix slice of the parameters as one matrix leaf, avoids that, and the sparse matrices
of symbolicSystem.md §12.7 (#3168) would carry the zeros of $\boldsymbol\zeta\otimes\mathbf I$. The markers: the
weighted rows $\sum_i w_i\boldsymbol\Psi_i$ and $\sum_i w_i\mathbf x_i$ are constants per marker, its `parameters`, so
a position marker is $\mathbf p_{ref} + \mathbf A(\bar{\mathbf x} + \bar{\boldsymbol\Psi}\boldsymbol\zeta)$ - a
MarkerBodyUser recorded by **E3**; the rotation of MarkerSuperElementRigid is open (how the built-in averages the
rotations is to be read before). Drawing by a Python graphicsDataUserFunction; the mapping of `graphicsData` by the
positionUserFunction cannot place a mesh vertex, as the function gets `localPosition` and not a mesh node.

**Limits**: the user functions of `exudyn.FEM` (`UFmassFFRFreducedOrder`, `UFforceFFRFreducedOrder`) are numpy code,
not recorded; the superelement outputs per mesh node are not reproduced; Lie group nodes refused; the mass matrix
depends on $\boldsymbol\zeta$ and the rotation, not constant.

**Effort HIGH.** Needs E3; E5 for models beyond a few modes.

## 5. Interface extensions

| | extension | serves | effort |
|---|---|---|---|
| E1 | ODE1 coordinates of ObjectConnectorUser: an ODE1 node, the coordinates in the forceUserFunction, an `ode1RhsUserFunction` of the marker states, in `SymbolicSystem` | HydraulicActuatorSimple; user connectors with internal dynamics | MEDIUM |
| E2 | the forceUserFunction of $n$ markers: lists of frames and twists, $6n$ forces and torques; coordinate markers open | ReevingSystemSprings; drives over several pulleys | MEDIUM |
| E3 | MarkerBodyUser recorded and assembled by `SymbolicSystem` (the marker kinematics composed with the body coordinates, its Jacobians by the one-pass derivative) | KinematicTree (links), FFRFreducedOrder (mesh nodes), any marker between bodies | MEDIUM |
| E4 | the multipliers in the constraintUserFunction of ObjectJointUser | JointGeneric with the form $\lambda_i = 0$ and run-time switching; compliant joints $\mathbf c = -\epsilon\boldsymbol\lambda$ | MEDIUM |
| E5 | matrix slices of `parameters` as matrix leaves (`ParameterLayout` entries with a shape) | FFRFreducedOrder; the inertias of KinematicTree and the beam | LOW-MEDIUM |

No extension, but helpers of exudyn.symbolics: a symbolic $\mathbf T_{SE3}^{-1}$ (4.6); the closed-form closest
points point-triangle, point-cylinder, segment-segment (4.5); one contact law of the user contacts (4.5).

## 6. Summary

| item | user item | extension | effort | recommendation |
|---|---|---|---|---|
| ObjectConnectorRollingDiscPenalty | ObjectConnectorUser, 2 rigid markers, data node 3, post-Newton | none | LOW | first |
| ObjectJointGeneric | ObjectJointUser, the constrained axes only, a variant per pattern | none (E4 optional) | LOW | second |
| contact sphere-cylinder, sphere-triangle | ObjectConnectorUser, data block 3 | none | LOW | with the common contact law |
| contact cylinder-triangle (infinite plane), capsule-capsule | ObjectConnectorUser, 4 points, data 12 | none | MEDIUM | do |
| contact cylinder-triangle (finite), cylinder-cylinder | - | GJK not recordable | HIGH | not now |
| ObjectBeamGeometricallyExact | ObjectUserBody on 2 rigid nodes (EP, Rxyz) | none; symbolic $\mathbf T_{SE3}^{-1}$ | MEDIUM | do |
| ObjectConnectorHydraulicActuatorSimple | ObjectConnectorUser with ODE1 (or ObjectGenericDAE now) | E1 | MEDIUM | with E1 |
| ObjectConnectorReevingSystemSprings | ObjectConnectorUser on $n$ rigid markers | E2 | MEDIUM | after E2, with the tangent fixed |
| ObjectKinematicTree | ObjectUserBody on the joint node (and base), MarkerBodyUser per link | E3 | HIGH | after E3; the most valuable |
| ObjectFFRFreducedOrder | ObjectUserBody on rigid and modal node, MarkerBodyUser per mesh-node set | E3, E5 | HIGH | last |

Order: the items without an extension first (rolling disc, generic joint, the contact law and the four recordable
contact pairs, the beam), then E1 with the hydraulic actuator, E3 with the kinematic tree, E2 with the reeving system,
E5 with the reduced-order body. E3 is the extension with the most value: two of the HIGH items need it.

## 7. Proposed sub-steps (proposals, not plan steps)

1. This evaluation, decided by the maintainer
2. The rolling disc with penalty as user connector, `CreateRollingDiscPenalty` of the creator
3. The generic joint as ObjectJointUser of the constrained axes, `CreateGenericJoint` of the creator
4. One contact law of the user contacts, from the sphere-sphere contact
5. The user contacts sphere-cylinder and sphere-triangle
6. The user contacts cylinder-triangle on the infinite plane and capsule-capsule
7. The symbolic inverse tangent operator of SE(3) in exudyn.symbolics.kinematics
8. The geometrically exact beam as ObjectUserBody on two rigid nodes
9. ODE1 coordinates of ObjectConnectorUser (E1)
10. The hydraulic actuator as user connector with ODE1 coordinates
11. MarkerBodyUser recorded and assembled by SymbolicSystem (E3)
12. The kinematic tree as ObjectUserBody with markers on its links
13. The forceUserFunction of n markers (E2)
14. The reeving system as user connector on n markers
15. Matrix slices of the parameters as matrix leaves (E5)
16. The reduced-order FFRF body as ObjectUserBody with mesh-node markers
17. The multipliers in the constraintUserFunction of ObjectJointUser (E4), if a model needs it
