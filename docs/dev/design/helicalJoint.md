# Helical joint: analysis

A design note of open steps (revision2026c step RG9.14, #3415): what exists, the kinematics, the formulations compared, the
locking options, the decision on one or two joints, parameters, outputs, tests and the sub-steps proposed. It leaves
the repository when the steps close (WORKFLOW.md §8.4).

## 1. What exists

- **The joints on the relative frame** (src/System/JointRelativeFrame.h, #2884): ObjectJointGeneric,
  ObjectJointRevoluteZ and ObjectJointPrismaticX constrain components of the frame of J1 = $(\mathbf{A}_1, \mathbf{p}_1)$
  in J0 = $(\mathbf{A}_0, \mathbf{p}_0)$: translations (global if all three are constrained, else
  $\mathbf{a}_i \cdot (\mathbf{p}_1 - \mathbf{p}_0)$ with the axes $\mathbf{a}_i$ of J0), rotations by scalar products of
  axes (`JointAxesEquation`). `JointRelativeFrameEquations` gives the equations and, on request, their rows in the 12
  variations $[\delta\mathbf{p}_0, \delta\boldsymbol{\theta}_0, \delta\mathbf{p}_1, \delta\boldsymbol{\theta}_1]$ with the
  global rotation increments; `JointRelativeFrameEquations_t` the velocity level as the same rows times the twists of the
  frames. The rows are analytic: `ComputeConstraintEquationsRigidDerivatives` returns true
  (CObjectJointGeneric.cpp, CObjectJointRevoluteZ.cpp, CObjectJointPrismaticX.cpp).
- **The connector interface** (src/System/CObjectConnector.cpp, `ConstraintEquationsRigidMarkers`,
  `ConstraintEquationsDerivativesRigid`, `ConstraintJacobianRigidMarkers`, `ConstraintReactionForcesRigidMarkers`, #2745):
  the equations from the two marker frames; C_q and the reaction forces from the 12-column rows, analytic or by
  automatic differentiation of `ComputeConstraintEquationsRigidDiff` (ObjectJointUser, ObjectJointPrismatic2D); the
  part along the Euler parameters by `RadialJacobianAE` (#3201). Kinds of markers: 'Position', 'Rigid', 'Coordinate'
  (`ItemConnectorInterface`, definitions/definitionTypes.py) - the same kind for both markers.
- **Free axes**: ObjectJointGeneric keeps 6 algebraic equations and sets $\lambda_i = 0$ on a free axis
  (`ComputeJacobianAE_AE`, the AE_AE Jacobian); RevoluteZ and PrismaticX have only their constrained equations
  (`JointConstrainedEquations`).
- **Velocity level**: `velocityLevel` in `ComputeConstraintEquationsRigid`, used with
  `generalizedAlpha.useIndex2Constraints`; for ObjectJointGeneric without the offset of `offsetUserFunction`, and
  `offsetUserFunction_t` raises.
- **The joint frame** is the marker frame; `rotationMarker0/1` are deprecated (#3179), the rotation is the `localHT` of
  the rigid marker. A new joint has no `rotationMarker0/1`.
- **Angles beyond $\pm\pi$**: the joints report the relative rotation by `RotationMatrix2RotXYZ`, in $(-\pi, \pi]$.
  ObjectConnectorTorsionalSpringDamper tracks a continuous angle in the data coordinate of an optional NodeGenericData
  (`nodeNumber`): `ComputeSpringTorque` moves the angle to the branch nearest the data coordinate (the angle at the
  start of the step), `PostNewtonStep` stores it.
- **The screw as a user joint** (python/TestModels/userJointTest.py, #3100): ObjectJointUser with 5 equations along x
  of J0, the coupling written periodic, $h(A_{21}\cos\psi - A_{11}\sin\psi) = h\sin(\varphi - \psi)$ with
  $A = \mathbf{A}_0^T\mathbf{A}_1$ and $\psi = d_x/h$, and its analytic Jacobian; it follows the analytic angle through 2.5 turns at index 3 and 2. The
  form by `atan2` stopped at half a turn, as $d_x - h\,\mathrm{atan2}(\ldots)$ jumps by $2\pi h$ there.

## 2. Kinematics

Notation: J0, J1 the marker frames, $\mathbf{x}_0, \mathbf{y}_0, \mathbf{z}_0$ the axes of J0 (columns of
$\mathbf{A}_0$), likewise for J1; $\mathbf{d} = \mathbf{A}_0^T (\mathbf{p}_1 - \mathbf{p}_0) = [d_x, d_y, d_z]^T$;
$h$ the pitch per radian (the lead is $2\pi h$; $h < 0$ a left-handed thread), $r$ the radius.

**The axis is z of J0**, as in ObjectJointRevoluteZ, ObjectConnectorTorsionalSpringDamper and the cylindrical joint of
ObjectJointGeneric (`constrainedAxes=[1,1,0,1,1,0]`).

### 2.1 The rigid screw (helical pair)

J1 relative to J0 is a screw motion about $\mathbf{z}_0$: $\mathbf{A}_0^T\mathbf{A}_1 = \mathbf{R}_z(\varphi)$,
$\mathbf{d} = [0, 0, h\varphi]^T$. One degree of freedom, five equations:

| equation | meaning | row |
|---|---|---|
| $d_x = 0$, $d_y = 0$ | origin of J1 on the axis | $\pm\mathbf{a}$ on $\delta\mathbf{p}$, $\mathbf{a} \times (\mathbf{p}_1 - \mathbf{p}_0)$ on $\delta\boldsymbol{\theta}_0$ |
| $\mathbf{z}_0 \cdot \mathbf{y}_1 = 0$, $\mathbf{z}_0 \cdot \mathbf{x}_1 = 0$ | no tilt about x, y | `JointAxesEquation` |
| $g_c = d_z - h\tilde\varphi = 0$ | the coupling | see below |

The first four are those of `JointRelativeFrameEquations` for `constrainedAxes=[1,1,0,1,1,0]`; only the coupling is
new.

**The angle.** $c = \mathbf{x}_0 \cdot \mathbf{x}_1$, $s = \mathbf{y}_0 \cdot \mathbf{x}_1$, $\varphi = \mathrm{atan2}(s, c)$,
in $(-\pi, \pi]$. **The turns are in the translation**: on the helix, $\psi = d_z / h$ is the continuous angle. The
coupling takes the branch of $\varphi$ nearest to $\psi$,

$$
\tilde\varphi = \psi + \mathrm{atan2}\big(s\cos\psi - c\sin\psi,\; c\cos\psi + s\sin\psi\big), \qquad
g_c = d_z - h\tilde\varphi = -h\,\mathrm{atan2}\big(\sin(\varphi - \psi), \cos(\varphi - \psi)\big)
$$

and $g_c = d_z$ for $h = 0$. So:

- **No state.** The configuration $(d_z, \mathbf{A}_1)$ decides the turn; the helix is a manifold in which
  $(d_z + 2\pi h, \mathbf{A}_1)$ is the next turn, a valid configuration. A data coordinate of the turns is not needed.
- **One root.** The periodic form $h\sin(\varphi - \psi)$ (#3100) is zero also at $\varphi - \psi = \pi$, a body half a
  turn off with the row of opposite sign; the form by `atan2` of the error has only $\varphi - \psi = 0 \pmod{2\pi}$. At
  the solution both have the same value and row.
- **Its limit**: $g_c$ jumps where the error $\varphi - \psi$ passes $\pm\pi$, i.e. where a Newton iterate has an axial
  mismatch of $\pi h$; with the predictor of a time step the mismatch is of the order of the step error. A static solve
  from an inconsistent start, or a very fine thread, settles on the nearest turn - a valid configuration.
- **$h = 0$** is the revolute joint without a special case in the rows: the term $h\,\delta\tilde\varphi$ vanishes.
- **Units**: $g_c$ is a length; its multiplier $\lambda_c$ is the axial force exactly (the row has 1 on $d_z$), and the
  torque about the axis that the same multiplier transmits is $-h\lambda_c$.

**The row of the coupling**: $\delta g_c = \delta d_z - h\,\delta\varphi$ (the derivative of the branch shift is zero),

$$
\delta d_z = \mathbf{z}_0 \cdot (\delta\mathbf{p}_1 - \delta\mathbf{p}_0) + \big(\mathbf{z}_0 \times (\mathbf{p}_1 - \mathbf{p}_0)\big) \cdot \delta\boldsymbol{\theta}_0, \qquad
\delta\varphi = \frac{c\,\delta s - s\,\delta c}{c^2 + s^2}
$$

with $\delta(\mathbf{a} \cdot \mathbf{b}) = (\mathbf{a} \times \mathbf{b}) \cdot (\delta\boldsymbol{\theta}_0 - \delta\boldsymbol{\theta}_1)$
(`JointAxesEquation`). On the manifold, $\delta\varphi = \mathbf{z}_1 \cdot (\delta\boldsymbol{\theta}_1 - \delta\boldsymbol{\theta}_0)$.
The general form is used, so that the row is the exact derivative off the manifold (Newton). Singular only for
$c = s = 0$, $\mathbf{x}_1 \parallel \mathbf{z}_0$, which the tilt equations exclude; with the tilts free (section 3)
the angle is that of $\mathbf{x}_1$ projected into the $xy$-plane of J0, and the singularity is reachable.

**Velocity level**: the row times the twists, as `JointRelativeFrameEquations_t`: $\dot d_z - h\dot\varphi$, with no
angle at all; the drift of index 2 goes into $d_z - h\tilde\varphi$, the output angle stays continuous (section 4.2).

### 2.2 The radial offset

**With the frame following** (the body turns with the point): the origin of J1 at radius $r$,
$\mathbf{d} = [r\cos\varphi, r\sin\varphi, h\varphi]^T$, $\mathbf{A}_0^T\mathbf{A}_1 = \mathbf{R}_z(\varphi)$. This is
the motion of 2.1 for a marker placed at $\mathbf{p}_1 - r\,\mathbf{x}_1$: a rigid body cannot tell where on it the
marker of a rigid screw sits. $r$ would only re-place marker 1; the circle with the frame following ($h = 0$) is the
revolute joint about an offset axis. **The rigid joint has no radius.**

**With the orientation free** (only the point moves on the helix): a point-on-curve constraint, the point of marker 1 on
$\mathbf{c}(\theta) = [r\cos\theta, r\sin\theta, h\theta]^T$ in J0. One degree of freedom of the point, two equations;
the curve parameter comes from the **position**, not from the rotation:

$$
\rho = \sqrt{d_x^2 + d_y^2}, \quad \theta = \mathrm{atan2}(d_y, d_x), \qquad
g_r = \rho - r = 0, \qquad g_c = d_z - h\tilde\theta = 0
$$

with $\tilde\theta$ the branch of $\theta$ nearest to $d_z/h$ as above ($g_c = d_z$ for $h = 0$: the circle in the plane
$z = 0$ of J0). Rows: $\delta\rho = \mathbf{e}_\rho \cdot \delta\mathbf{d}$,
$\delta\theta = \mathbf{e}_\theta \cdot \delta\mathbf{d} / \rho$, $\delta\mathbf{d}$ as $\delta d_z$ above per component.
The two normals $\mathbf{e}_\rho$ and $\mathbf{e}_z - (h/\rho)\,\mathbf{e}_\theta$ are perpendicular to the tangent
$r\,\mathbf{e}_\theta + h\,\mathbf{e}_z$. **Requires $r > 0$**: at $\rho = 0$ neither $\theta$ nor
$\mathbf{e}_\rho$ is defined; $r = 0$ is the point on the axis, where the rotation and not the position carries the
angle - the rigid joint of 2.1.

So the two cases are **two different formulations**: the curve parameter from the rotation (regular at $r = 0$, no
use for $r$) and from the position (singular at $r = 0$, needs $r$). The orientation of the point joint, when wanted,
is locked by a separate ObjectJointGeneric with `constrainedAxes=[0,0,0,1,1,1]` (circular translation, the coupler of
a parallelogram) or `[0,0,0,1,1,0]` (axis kept parallel, spin free).

## 3. Locking options

### 3.1 The rigid joint

`constrainedAxes` $= [j_x, j_y, j_c, j_{\alpha}, j_{\beta}]$: the radial translations $d_x$, $d_y$, the coupling $g_c$,
the tilt about x ($\mathbf{z}_0 \cdot \mathbf{y}_1$) and about y ($\mathbf{z}_0 \cdot \mathbf{x}_1$). Each tilt has its
own equation - unlike `constrainedAxes[3:6]` of ObjectJointGeneric, where one locked rotation switches to the universal
joint's equation. An axial lock of its own is not needed: the coupling with $h = 0$ is $d_z = 0$, and the coupling
together with an axial lock would also lock the rotation (a rigid joint, which ObjectJointGeneric is).

| `constrainedAxes` | $h \neq 0$ | $h = 0$ | DOF |
|---|---|---|---|
| [1,1,1,1,1] | helical (screw) | revolute = ObjectJointRevoluteZ | 1 |
| [1,1,0,1,1] | cylindrical = ObjectJointGeneric [1,1,0,1,1,0] | the same | 2 |
| [0,0,1,1,1] | screw with the axis parallel-shifted freely in the plane (a floating nut) | planar = ObjectJointGeneric [0,0,1,1,1,0] | 3 |
| [1,1,1,0,0] | the origin of J1 on the axis, $d_z$ coupled to the projected angle; tilts free | spherical = ObjectJointGeneric [1,1,1,0,0,0] | 3 |
| [0,0,0,1,1] | parallel axes only | the same | 4 |

The row with free tilts keeps the singularity of 2.1 ($\mathbf{x}_1 \parallel \mathbf{z}_0$); it is allowed and
documented, as the other combinations of ObjectJointGeneric are.

### 3.2 The point joint

`constrainedAxes` $= [j_r, j_c]$:

| `constrainedAxes` | $h \neq 0$ | $h = 0$ |
|---|---|---|
| [1,1] | point on the helix (bead on a helical wire) | point on the circle of radius $r$ |
| [1,0] | point on the cylinder of radius $r$ | the same |
| [0,1] | point on the helicoid (screw surface) $d_z = h\tilde\theta$ | point in the plane $z = 0$ of J0 |

$[0,1]$ with $h \neq 0$ needs $\rho > 0$ as well; $[0,1]$ with $h = 0$ has no singularity.

## 4. Decision: two joints

**One joint with a mode is not proposed**: the two sets of equations share no row (the angle from the rotation vs.
from the position), $r$ is meaningless in one and required in the other, the free-orientation case needs no
orientation of marker 1 while the screw does, and the options of section 3 differ.

### 4.1 Names

- **ObjectJointHelicalZ**, `pythonShortName='HelicalJointZ'`, type name `JointHelicalZ`: the helical (screw) pair about
  z, named like ObjectJointRevoluteZ / ObjectJointPrismaticX; *helical* is the term of the kinematic pair, *screw* the
  machine element.
- **ObjectJointPointOnHelixZ**, `pythonShortName='PointOnHelixJointZ'`: the point of marker 1 on the helix of
  radius $r$ about z of J0.

### 4.2 ObjectJointHelicalZ

- **Formulation**: (a) the equations of the relative frame for $[d_x, d_y, \mathbf{z}_0\cdot\mathbf{y}_1, \mathbf{z}_0\cdot\mathbf{x}_1]$
  from `JointRelativeFrameEquations`, plus the coupling $g_c$ by `atan2` of the error (2.1). Compared:

  | | form | for | against |
  |---|---|---|---|
  | (a)+(b') | $d_z - h\tilde\varphi$, branch nearest $d_z/h$ | no state, one root, $h = 0$ regular, $\lambda_c$ the axial force | jump at an axial mismatch of $\pi h$ |
  | (b) | $h\sin(\varphi - d_z/h)$ (#3100) | smooth everywhere, no state | a spurious root at half a turn; undefined for $h = 0$ ($d_z/h$) |
  | (c) | $d_z - h\varphi_c$, $\varphi_c$ nearest a data coordinate (as ObjectConnectorTorsionalSpringDamper) | continuous angle also for $h = 0$ | a NodeGenericData, `PostNewtonStep`, at most $\pi$ of rotation per step, a node the user must add |
  | (d) | velocity level only, $\dot d_z - h\dot\varphi$ | no angle | the position drifts with index 3 solvers; the constraint is holonomic, unlike ObjectJointRollingDisc |

  (a)+(b') is recommended; (d) is its velocity level for `useIndex2Constraints`, as for the other joints.
- **Index 3**, rows analytic (`ComputeConstraintEquationsRigidDerivatives` returns true, #2884), the velocity level the
  rows times the twists; on the 'Rigid' connector interface (`*ItemConnectorInterface('Rigid', constraint=True)`),
  `ItemRequestedTypes('Marker', ['Position', 'Orientation'])`.
- **5 algebraic equations always**, $\lambda_i = 0$ on a free one, `ComputeJacobianAE_AE` as ObjectJointGeneric.
- **Parameters**: `markerNumbers`, `pitch` $h$ (per radian, any sign, 0 allowed), `constrainedAxes` (5, default
  [1,1,1,1,1]), `activeConnector`; visualization `show`, `axisRadius`, `axisLength`, `color`. No `rotationMarker0/1`
  (#3179), no radius (2.2), no offset user function in the first version - a driven screw is a torque or a constraint
  on the rotation of the drive; ObjectJointGeneric's user function is index 3 only.
- **Output variables**: `Position`, `Velocity` (marker 0); `DisplacementLocal` $\mathbf{d}$; `VelocityLocal`;
  `Rotation` $[\theta_x, \theta_y, \tilde\varphi]$, the z component the continuous angle for $h \neq 0$, in
  $(-\pi, \pi]$ for $h = 0$ as ObjectJointRevoluteZ; `AngularVelocityLocal`; `ForceLocal`
  $[\lambda_x, \lambda_y, \lambda_c]$ in J0, $\lambda_c$ the axial force; `TorqueLocal`
  $[\lambda_\alpha, \lambda_\beta, -h\lambda_c]$; `HomogeneousTransformation`, `HomogeneousTransformationLocal`;
  `ConstraintEquation`.
- **Checks** (`CheckPreAssembleConsistency`): two rigid markers; `constrainedAxes` of 0 and 1.
- **Drawing**: the axis as two cylinders as ObjectJointRevoluteZ; for $h \neq 0$ the thread as a helix on the axis
  cylinder over `axisLength` (`EXUvis::DrawSpring`, Graphics/VisualizationPrimitives.h, with
  `axisLength`$/(2\pi|h|)$ windings, capped); `connectors.showJointAxes` the two frames.
- **Create function**: `mbs.CreateHelicalJoint(bodyNumbers, position, axis, pitch, useGlobalFrame, ...)` in
  python/exudyn/misc/mainSystemExtensions.py, as `CreateRevoluteJoint` (the markers with the rotation of the axis to z
  in their `localHT`).

### 4.3 ObjectJointPointOnHelixZ

- **Formulation**: $g_r = \rho - r$, $g_c = d_z - h\tilde\theta$ (2.2), index 3, the velocity level by the rows.
  $\lambda_r$ is the radial force; the force on the point is $\lambda_r\mathbf{e}_\rho + \lambda_c(\mathbf{e}_z - (h/\rho)\mathbf{e}_\theta)$.
- **Rows**: analytic is short (2.2); automatic differentiation of `ComputeConstraintEquationsRigidDiff` is the
  alternative with less code (ObjectJointPrismatic2D does so).
- **The markers**: marker 0 needs its frame, marker 1 only a position. The connector interface has one kind for both
  markers; a mixed kind (marker 0 'Rigid', marker 1 'Position') does not exist. The first version takes two rigid
  markers - the rotation columns of marker 1 are zero - which excludes ObjectMassPoint (no rigid marker); the mixed kind
  is a sub-step of its own if a mass point on a helix is wanted.
- **Parameters**: `markerNumbers`, `pitch` $h$, `radius` $r$, `constrainedAxes` (2, default [1,1]), `activeConnector`;
  visualization `show`, `drawSize`, `color`. 2 algebraic equations, $\lambda_i = 0$ on a free one.
- **Output variables**: `Position` (marker 1), `DisplacementLocal`, `VelocityLocal`, `Rotation` the curve parameter
  $\tilde\theta$ (a scalar), `ForceLocal` the force on the point in J0, `ConstraintEquation`.
- **Checks**: $r > 0$ if $j_r = 1$ or ($j_c = 1$ and $h \neq 0$).
- **Drawing**: the helix of radius $r$ over `drawSize` turns about the axis (`DrawSpring`), the circle for $h = 0$
  (`DrawCircle`).

## 5. Tests

- **python/TestModels/helicalJointTest.py** (ObjectJointHelicalZ):
  - the screw of userJointTest.py (torque $M$, axial force $F$, 2.5 turns) against
    $\varphi = (M + Fh)\,t^2 / (2(J_{zz} + mh^2))$, at index 3 and index 2; the axial multiplier against
    $mh\ddot\varphi - F$ (sign by the convention of the multipliers); `Rotation` continuous through the turns;
    the same with $h < 0$;
  - the same model with ObjectJointUser and `UFscrew` (axis x by the marker frame): the same solution to the Newton
    tolerance;
  - $h = 0$ against ObjectJointRevoluteZ; `constrainedAxes=[1,1,0,1,1]` against ObjectJointGeneric
    `[1,1,0,1,1,0]` (the same equations, expected equal to round-off); `[0,0,1,1,1]` with $h = 0$ against the planar
    ObjectJointGeneric;
  - a static solve: an axial spring of stiffness $k$ against the screw under the torque $M$, $\varphi = M/(k h^2)$;
  - the circle with the frame following: a body with marker 1 at radius $r$ placed on the axis, against
    ObjectJointRevoluteZ with the marker at the same point;
  - `activeConnector=False`: $\lambda = 0$; the refusals of `CheckPreAssembleConsistency`.
- **python/TestModels/pointOnHelixJointTest.py** (ObjectJointPointOnHelixZ):
  - a rigid body, its marker at the center of mass, on a helix with gravity along $-\mathbf{z}_0$: orientation free and
    no torque, so $\ddot d_z = -g\,h^2/(r^2 + h^2)$ exactly;
  - $h = 0$, gravity in the plane: the point on the circle against ObjectConnectorDistance of a mass point (the
    pendulum) and, with ObjectJointGeneric `[0,0,0,1,1,1]` added, the circular translation;
  - `constrainedAxes=[1,0]` against a point on a cylinder, `[0,1]` the helicoid; the refusal of $r = 0$.
- **MiniExamples** (generated from the definitions): a nut on a vertical screw falling under gravity, `testResult` the
  angle; a bead on a helical wire, `testResult` $d_z$.

## 6. Sub-steps proposed (RG9.14; RG9.14.1 is this analysis)

1. RG9.14.2 ObjectJointHelicalZ: the definition, the equations with analytic rows and the velocity level, the output
   variables, the drawing, the MiniExample
2. RG9.14.3 helicalJointTest.py: the screw against the analytic solution and ObjectJointUser, the reduced cases against
   ObjectJointRevoluteZ and ObjectJointGeneric
3. RG9.14.4 mbs.CreateHelicalJoint
4. RG9.14.5 ObjectJointPointOnHelixZ on two rigid markers, with pointOnHelixJointTest.py
5. RG9.14.6 The mixed connector interface (marker 0 rigid, marker 1 position) for a mass point on the helix - only if
   wanted
6. RG9.14.7 userJointTest.py compares its screw with ObjectJointHelicalZ; the joints in the manual and in
   revisions.md
