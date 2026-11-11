# The helical joint

ObjectJointHelicalZ inside: the equations, the angle on the turn, the rows, the locking, the outputs and the tests
(#3415, #3421, #3422, #3423); and the point on a helix, which is not built.

## 1. What the joint constrains

The frame of marker 1, J1 = $(\mathbf{A}_1, \mathbf{p}_1)$, moves relative to the frame of marker 0, J0 =
$(\mathbf{A}_0, \mathbf{p}_0)$, on a screw motion about $\mathbf{z}_0$: $\mathbf{A}_0^T\mathbf{A}_1 = \mathbf{R}_z(\varphi)$,
$\mathbf{d} = \mathbf{A}_0^T(\mathbf{p}_1 - \mathbf{p}_0) = [0, 0, h\varphi]^T$, with the pitch $h$ per radian (the lead
is $2\pi h$; $h < 0$ a left-handed thread). One degree of freedom, five equations
(`src/ImplObjects/CObjectJointHelicalZ.cpp`):

| index | equation | meaning |
|---|---|---|
| 0, 1 | $d_x = \mathbf{x}_0 \cdot (\mathbf{p}_1 - \mathbf{p}_0) = 0$, $d_y = 0$ | the origin of J1 on the axis |
| 2 | $g_c = d_z - h\tilde\varphi = 0$ | the coupling |
| 3 | $\mathbf{z}_0 \cdot \mathbf{y}_1 = 0$ | no rotation about $x$ |
| 4 | $\mathbf{z}_0 \cdot \mathbf{x}_1 = 0$ | no rotation about $y$ |

The axis is $z$ of J0, as in ObjectJointRevoluteZ and the cylindrical ObjectJointGeneric. The joint frames are the marker
frames; the joint has no `rotationMarker0/1` (#3179).

## 2. The coupling: the angle on the turn of the axial position

$c = \mathbf{x}_0 \cdot \mathbf{x}_1$, $s = \mathbf{y}_0 \cdot \mathbf{x}_1$, $\varphi = \mathrm{atan2}(s, c)$ in
$(-\pi, \pi]$. The turns are counted by the translation: $\psi = d_z/h$ is the angle of any number of turns that the axial
position requires, and the coupling takes the branch of $\varphi$ nearest to it,

$$
\tilde\varphi = \psi + e, \qquad e = \mathrm{atan2}\big(s\cos\psi - c\sin\psi,\; c\cos\psi + s\sin\psi\big), \qquad
g_c = -h\,e,
$$

`HelicalJointCoupling`; for $h = 0$, $g_c = d_z$ and $\tilde\varphi = \varphi$.

- **No state**: the configuration decides the turn; $(d_z + 2\pi h, \mathbf{A}_1)$ is the next turn, a valid
  configuration. No data node and no post-Newton step are needed.
- **One root**: the error $e$ vanishes only for $\varphi - \psi = 0 \pmod{2\pi}$. The periodic form of the user screw of
  `userJointTest.py`, $h\sin(\varphi - \psi)$ (#3100), also vanishes half a turn off, with the row of opposite sign.
- **The jump**: $g_c$ jumps where $e$ passes $\pm\pi$, i.e. for an axial mismatch of $\pi h$ in a Newton iterate. The
  predictor of a time step keeps the mismatch at the order of the step error; a static solve from an inconsistent
  start settles on the nearest turn.
- **$h = 0$** is the revolute joint without a special case in the rows.
- **Units**: $g_c$ is a length; its multiplier $\lambda_2$ is the axial force; it transmits the torque $-h\lambda_2$
  about the axis.

The alternatives were weighed: the periodic form (a spurious root, undefined for $h = 0$), a continuous angle in a data
coordinate as ObjectConnectorTorsionalSpringDamper keeps it (a node the user adds, a post-Newton step, at most $\pi$ per
step), and the velocity level only (drift with index 3 solvers, though the constraint is holonomic).

## 3. The rows

The rows are analytic in the 12 variations $[\delta\mathbf{p}_0, \delta\boldsymbol{\theta}_0, \delta\mathbf{p}_1,
\delta\boldsymbol{\theta}_1]$ with the global rotation increments, as the joints of `src/System/JointRelativeFrame.h`;
`ComputeConstraintEquationsRigidDerivatives` returns true and the connector interface (#2745) builds $\mathbf{C}_q$ and
the reaction forces from them.

- $\delta(\mathbf{a}\cdot(\mathbf{p}_1 - \mathbf{p}_0)) = \mathbf{a}\cdot(\delta\mathbf{p}_1 - \delta\mathbf{p}_0) + (\mathbf{a} \times (\mathbf{p}_1 - \mathbf{p}_0))\cdot\delta\boldsymbol{\theta}_0$ for $\mathbf{a} = \mathbf{x}_0, \mathbf{y}_0, \mathbf{z}_0$;
- $\delta g_c = \delta d_z - h\,\delta\varphi$ (the branch shift has no derivative), $\delta\varphi = (c\,\delta s - s\,\delta c)/(c^2 + s^2)$
  with $\delta(\mathbf{a}\cdot\mathbf{b}) = (\mathbf{a} \times \mathbf{b})\cdot(\delta\boldsymbol{\theta}_0 - \delta\boldsymbol{\theta}_1)$;
  the general form, so that the row is the exact derivative also off the manifold. It is singular for
  $\mathbf{x}_1 \parallel \mathbf{z}_0$ only, which the tilt equations exclude; with the tilts free the angle is that of
  $\mathbf{x}_1$ projected into the $xy$-plane of J0 and the singularity is reachable - the regular module raises there;
- the tilts by `JointAxesEquation`.

**Velocity level** (`generalizedAlpha.useIndex2Constraints`): the rows times the twists of the frames, as
`JointRelativeFrameEquations_t`; the coupling is $\dot d_z - h\dot\varphi$, without an angle.

## 4. Locking

`constrainedAxes` $= [j_0, \ldots, j_4]$ in the order of section 1; the joint always has 5 algebraic equations, a free one
is $\lambda_i = 0$ (`ComputeJacobianAE_AE`, as ObjectJointGeneric). Each tilt has its own equation. An axial lock of its
own is not needed: the coupling with $h = 0$ is $d_z = 0$, and coupling plus an axial lock would lock the rotation too.

| `constrainedAxes` | $h \neq 0$ | $h = 0$ |
|---|---|---|
| [1,1,1,1,1] | helical joint | revolute joint |
| [1,1,0,1,1] | cylindrical joint | cylindrical joint |
| [0,0,1,1,1] | screw with the axis free to shift in the plane | planar joint |
| [1,1,1,0,0] | the origin of J1 on the axis, $d_z$ coupled to the projected angle | spherical joint |
| [0,0,0,1,1] | the axes kept parallel | the same |

`CheckPreAssembleConsistency` refuses an entry other than 0 and 1 (`src/System/checkPreAssembleConsistenciesObjects.cpp`).

## 5. Why the rigid joint has no radius

A screw curve at the radius $r$ with the frame following the point, $\mathbf{d} = [r\cos\varphi, r\sin\varphi, h\varphi]^T$
with $\mathbf{A}_0^T\mathbf{A}_1 = \mathbf{R}_z(\varphi)$, is the motion of section 1 for a marker placed at
$\mathbf{p}_1 - r\,\mathbf{x}_1$: a rigid body cannot tell where on it the marker of a rigid screw sits. A radius would
only move marker 1; the circle with the frame following ($h = 0$) is the revolute joint about an offset axis.

## 6. Outputs and drawing

- `Rotation`: the Tait-Bryan angles of $\mathbf{A}_0^T\mathbf{A}_1$ for $x$ and $y$, the drift; $z$ is $\tilde\varphi$,
  continuous through the turns for $h \neq 0$, in $(-\pi, \pi]$ for $h = 0$.
- `ForceLocal` $[\lambda_0, \lambda_1, \lambda_2]$ in J0, `TorqueLocal` $[\lambda_3, \lambda_4, -h\lambda_2]$. With the
  convention of the multipliers, $\mathbf{M}\ddot{\mathbf{q}} + \mathbf{C}_q^T\boldsymbol{\lambda} = \mathbf{f}$, a body on the
  screw under the axial force $F$ has $\lambda_2 = F - m h\ddot\varphi$.
- `DisplacementLocal` $\mathbf{d}$, `VelocityLocal`, `AngularVelocityLocal`, the frames, `ConstraintEquation`.
- The drawing: the axis as two cylinders as ObjectJointRevoluteZ, and for $h \neq 0$ the thread as a helix
  (`EXUvis::DrawSpring`) of the lead $2\pi|h|$ over the axis at marker 1, at most 64 windings, `connectors.curveTiling`
  segments per winding.

`mbs.CreateHelicalJoint` (`python/exudyn/misc/mainSystemExtensions.py`) builds the markers with the joint frame as
`CreateRevoluteJoint` does; a marker given in `itemNumbers` must take the rotation in its `localHT`.

## 7. Tests

`python/TestModels/helicalJointTest.py`:

| case | against | agreement |
|---|---|---|
| torque and axial force, 2.5 turns, $h = \pm 0.05$, index 3 and 2 | $\varphi = (M + Fh)t^2/(2(J_{xx} + mh^2))$; $\lambda_2 = F - mh\ddot\varphi$ | 1.2e-3 (the integration error of 500 steps) |
| the same screw | ObjectJointUser with the constraint of `userJointTest.py` | 1.3e-14 |
| $h = 0$; coupling free | ObjectJointRevoluteZ; ObjectJointGeneric [1,1,0,1,1,0] | the same bits |
| static, an axial spring $k$ against the torque | $\varphi = M/(kh^2)$ | 1e-16 |
| `activeConnector=False`; an entry 2 in `constrainedAxes`; a position marker | $\boldsymbol{\lambda} = 0$; refused; refused | |

The MiniExample, a nut falling on a vertical screw, gives $-Fht^2/(2(J_{zz} + mh^2))$ exactly.

## 8. Not built: the point on a helix

A second joint, ObjectJointPointOnHelixZ, was analysed together with the screw and is not built: the point of marker
1 on the helix $\mathbf{c}(\theta) = [r\cos\theta, r\sin\theta, h\theta]^T$ in J0, the orientation free. A motion along a
curve in space, with a screw motion as an option, is to be a general curve joint, defined together with the curve
markers on deformable bodies.

The analysis, kept for that joint:

- **Equations**: the curve parameter from the position, not from the rotation,
  $\rho = \sqrt{d_x^2 + d_y^2}$, $\theta = \mathrm{atan2}(d_y, d_x)$, $g_r = \rho - r = 0$ and $g_c = d_z - h\tilde\theta = 0$
  with $\tilde\theta$ the branch nearest $d_z/h$ as in section 2 ($g_c = d_z$ for $h = 0$, the circle). Rows:
  $\delta\rho = \mathbf{e}_\rho\cdot\delta\mathbf{d}$, $\delta\theta = \mathbf{e}_\theta\cdot\delta\mathbf{d}/\rho$; the two normals
  $\mathbf{e}_\rho$ and $\mathbf{e}_z - (h/\rho)\,\mathbf{e}_\theta$ are perpendicular to the tangent $r\,\mathbf{e}_\theta + h\,\mathbf{e}_z$.
  $\lambda_r$ is the radial force; the force on the point is $\lambda_r\mathbf{e}_\rho + \lambda_c(\mathbf{e}_z - (h/\rho)\mathbf{e}_\theta)$.
- **It is a different formulation from the screw**: the angle from the position is singular at $\rho = 0$ and needs
  $r > 0$; the angle from the rotation is regular at $r = 0$ and has no use for $r$ (section 5). The two share no row,
  hence two joints rather than one with a mode.
- **Locking** `[j_r, j_c]`: [1,1] the point on the helix (on the circle for $h = 0$), [1,0] on the cylinder of radius
  $r$, [0,1] on the helicoid $d_z = h\tilde\theta$ (in the plane $z = 0$ for $h = 0$).
- **Markers**: marker 0 needs its frame, marker 1 only a position; the connector interface has one kind of marker for
  both, so a first version would take two rigid markers, which excludes a mass point.
- **A fixed orientation** of the point body by an added ObjectJointGeneric, `[0,0,0,1,1,1]` (circular translation) or
  `[0,0,0,1,1,0]`.
- **Tests**: a body on a helix under gravity along $-\mathbf{z}_0$ with the orientation free has
  $\ddot d_z = -g\,h^2/(r^2 + h^2)$ exactly; $h = 0$ against ObjectConnectorDistance of a mass point.
