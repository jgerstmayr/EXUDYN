(sec-generalnotation)=
# Notation

The notation is used to explain:

- typical symbols in equations and formulas (e.g., $q$)
- common types used as parameters in items (e.g., `PInt`)
- typical annotation of equations (or parts of it) and symbols (e.g., `ODE2`)

(sec-typesdescriptions)=
## Common types

Common types are especially used in the definition of items.
These types indicate how they need to be set (e.g., a `Vector3D` is set as a list of 3 floats or as a numpy array with 3 floats),
and they usually include some range or size check (e.g., `PReal` is checked for being positive and non-zero):

- `float` $\ldots$ a single-precision floating point number (note: in Python, '`float`' is used also for double precision numbers; in EXUDYN, internally floats are single precision numbers especially for graphics objects and OpenGL)
- `Real` $\ldots$ a double-precision floating point number (note: in Python this is also of type '`float`')
- `UReal` $\ldots$ same as `Real`, but may not be negative
- `PReal` $\ldots$ same as `Real`, but must be positive, non-zero (e.g., step size may never be zero)
- `Index` $\ldots$ deprecated, represents unsined integer, `UInt`
- `Int` $\ldots$ a (signed) integer number, which converts to '`int`' in Python, '`int`' in C++
- `UInt` $\ldots$ an unsigned integer number, which converts to '`int`' in Python
- `PInt` $\ldots$ an positive integer number (> 0), which converts to '`int`' in Python
- `NodeIndex, MarkerIndex, ...` $\ldots$ a special (non-negative) integer type to represent indices of nodes, markers, ...; specifically, an unintentional conversion from one index type to the other is not possible (e.g., to convert `NodeIndex` to `MarkerIndex`); see {ref}`sec-itemindex`
- `String` $\ldots$ a string
- `ArrayIndex` $\ldots$ a list of integer numbers (either list or in some cases `numpy` arrays may be allowed)
- `ArrayNodeIndex` $\ldots$ a list of node indices
- `Bool` $\ldots$ a boolean parameter: either `True` or `False` ('`bool`' in Python)
- `VObjectMassPoint`, `VObjectRigidBody`, `VObjectGround`, etc.  $\ldots$ represents the visualization object of the underlying object; 'V' is put in front of object name
- `BodyGraphicsData` $\ldots$ see {ref}`sec-graphicsdata`
- `Vector2D` $\ldots$ a list or `numpy` array of 2 real numbers
- `Vector3D` $\ldots$ a list or `numpy` array of 3 real numbers
- `Vector'X'D` $\ldots$ a list or `numpy` array of 'X' real numbers
- `Float4` $\ldots$ a list of 4 float numbers
- `Vector` $\ldots$ a list or `numpy` array of real numbers (length given by according object)
- `NumpyVector` $\ldots$ a 1D `numpy` array with real numbers (size given by according object); similar as Vector, but not accepting list
- `Matrix3D` $\ldots$ a list of lists or `numpy` array with $3 \times 3$ real numbers
- `Matrix6D` $\ldots$ a list of lists or `numpy` array with $6 \times 6$ real numbers
- `NumpyMatrix` $\ldots$ a 2D `numpy` array (matrix) with real numbers (size given by according object)
- `NumpyMatrixI` $\ldots$ a 2D `numpy` array (matrix) with integer numbers (size given by according object)
- `MatrixContainer` $\ldots$ a versatile representation for dense and sparse matrices, see {ref}`sec-matrixcontainer`
- `ObjectConnectorSpringDamperSpringForceUserFunction`, ... $\ldots$ a user function: the type is the name of its `Protocol` in `exudyn.itemInterface`, the item and the parameter, against which an editor checks the function; its arguments are in the user function description of the item

Note that for integers, there is also the `exu.InvalidIndex()` which is used to uniquely mark invalid indices, e.g., for default values of node numbers in objects or for other functions, often marked as `invalid (-1)` in the documentation. Currently, the invalid index is set to -1, but it may change in the future!

## States and coordinate attributes

The following subscripts are used to define configurations of a quantity, e.g., for a vector of displacement coordinates $\qv$:

- $\qv\cConfig \ldots$ $\qv$ in any configuration
- $\qv\cRef \ldots$ $\qv$ in reference configuration, e.g., reference coordinates: $\cv\cRef$
- $\qv\cIni \ldots$ $\qv$ in initial configuration, e.g., initial displacements: $\uv\cIni$
- $\qv\cCur \ldots$ $\qv$ in current configuration
- $\qv\cVis \ldots$ $\qv$ in visualization configuration
- $\qv\cSOS \ldots$ $\qv$ in start of step configuration

Note that the reference configuration is not included in other configurations, such that coordinates have to be usually understood relative to the reference configuration, see also {ref}`sec-overview-items-coordinates`.

As written in the introduction, the coordinates are attributed to certain types of equations and therefore, the following attributes are used (usually as subscript, e.g., $\qv_{ODE2}$):

- {ref}`ODE2 <ODE2>`, {ref}`ODE1 <ODE1>`, {ref}`AE <AE>`, Data: these attributes refer to the type of equation or
  coordinate the quantity belongs to (click an abbreviation for its explanation)

Time is usually defined as 'time' or $t$.
The cross product or vector product '$\times$' is often replaced by the skew symmetric matrix using the tilde '$\tilde{\;\;}$' symbol,

$$
  \av \times \bv = \tilde \av \, \bv = -\tilde \bv \, \av \, ,
$$
in which $\tilde{\;\;}$ transforms a vector $\av$ into a skew-symmetric matrix $\tilde \av$.
If the components of $\av$ are defined as $\av = \vrRow{a_0}{a_1}{a_2}\tp$, then the skew-symmetric matrix reads

$$
  \tilde \av = \mr{0}{-a_2}{a_1} {a_2}{0}{-a_0} {-a_1}{a_0}{0} \, .
$$
The inverse operation is denoted as $\vec$, resulting in $\vec(\tilde \av) = \av$.

For the length of a vector we often use the abbreviation

$$
  \Vert \av \Vert = \sqrt{\av^T \av} \, .
$$ (eq-definition-length)

A vector $\av=[x,\, y,\, z]\tp$ can be transformed into a diagonal matrix, e.g.,

$$
  \Am = \diag(\av) = \mr{x}{0}{0} {0}{y}{0} {0}{0}{z}
$$
(sec-symbolsitems)=
## Symbols in item equations

 The following tables contains the common notation
General **coordinates** are:

| **python name (or description)** | **symbol** | **description** |
|---|---|---|
| displacement coordinates ({ref}`ODE2 <ODE2>`) | $\qv = [q_0,\, \ldots,\, q_n]\tp$ | vector of $n$ displacement based coordinates in any configuration; used for second order differential equations |
| rotation coordinates ({ref}`ODE2 <ODE2>`) | $\tpsi = [\psi_0,\, \ldots,\, \psi_\eta]\tp$ | vector of $\eta$ **rotation based coordinates** in any configuration; these coordinates are added to reference rotation parameters to provide the current rotation parameters; used for second order differential equations |
| coordinates ({ref}`ODE1 <ODE1>`) | $\yv = [y_0,\, \ldots,\, y_n]\tp$ | vector of $n$ coordinates for first order ordinary differential equations ({ref}`ODE1 <ODE1>`) in any configuration |
| algebraic coordinates | $\zv = [z_0,\, \ldots,\, z_m]\tp$ | vector of $m$ algebraic coordinates if not Lagrange multipliers in any configuration |
| Lagrange multipliers | $\tlambda = [\lambda_0,\, \ldots,\, \lambda_m]\tp$ | vector of $m$ Lagrange multipliers (=algebraic coordinates) in any configuration |
| data coordinates | $\xv = [x_0,\, \ldots,\, x_l]\tp$ | vector of $l$ data coordinates in any configuration |

The following parameters represent possible **OutputVariable** (list is not complete):

| **python name** | **symbol** | **description** |
|---|---|---|
| Coordinate | $\cv = [c_0,\, \ldots,\, c_n]\tp$ | coordinate vector with $n$ generalized coordinates $c_i$ in any configuration; the letter $c$ is used both for {ref}`ODE1 <ODE1>` and {ref}`ODE2 <ODE2>` coordinates |
| Coordinate_t | $\dot \cv = [c_0,\, \ldots,\, c_n]\tp$ | time derivative of coordinate vector |
| Displacement | $\LU{0}{\uv} = [u_0,\, u_1,\, u_2]\tp$ | global displacement vector with 3 displacement coordinates $u_i$ in any configuration; in 1D or 2D objects, some of there coordinates may be zero |
| Rotation | $[\varphi_0,\,\varphi_1,\,\varphi_2]\tp\cConfig$ | vector with 3 components of the Euler angles in xyz-sequence ($\LU{0b}{\Rot}\cConfig=:\Rot_0(\varphi_0) \cdot \Rot_1(\varphi_1) \cdot \Rot_2(\varphi_2)$), recomputed from rotation matrix |
| Rotation (alt.) | $\ttheta = [\theta_0,\, \ldots,\, \theta_n]\tp$ | vector of **rotation parameters** (e.g., Euler parameters, Tait Bryan angles, ...) with $n$ coordinates $\theta_i$ in any configuration |
| Identity matrix | $\Im = \mr{1}{0}{0} {0}{\ddots}{0} {0}{0}{1}$ | the identity matrix, very often $\Im = \ImThree$, the $3 \times 3$ identity matrix |
| Identity transformation | $\LU{0b}{\ImThree} = \ImThree$ | converts body-fixed into global coordinates, e.g., $\LU{0}{\xv} = \LU{0b}{\ImThree} \LU{b}{\xv}$, thus resulting in $\LU{0}{\xv} = \LU{b}{\xv}$ in this case |
| RotationMatrix | $\LU{0b}{\Rot} = \mr{A_{00}}{A_{01}}{A_{02}} {A_{10}}{A_{11}}{A_{12}} {A_{20}}{A_{21}}{A_{22}}$ | a 3D rotation matrix, which transforms local (e.g., body $b$) to global coordinates (0): $\LU{0}{\xv} = \LU{0b}{\Rot} \LU{b}{\xv}$ |
| RotationMatrixX | $\LU{01}{\Rot_0(\theta_0)} = \mr{1}{0}{0} {0}{\cos(\theta_0)}{-\sin(\theta_0)} {0}{\sin(\theta_0)}{\cos(\theta_0)}$ | rotation matrix for rotation around $X$ axis (axis 0), transforming a vector from frame 1 to frame 0 |
| RotationMatrixY | $\LU{01}{\Rot_1(\theta_1)} = \mr{\cos(\theta_1)}{0}{\sin(\theta_1)} {0}{1}{0} {-\sin(\theta_1)}{0}{\cos(\theta_1)}$ | rotation matrix for rotation around $Y$ axis (axis 1), transforming a vector from frame 1 to frame 0 |
| RotationMatrixZ | $\LU{01}{\Rot_2(\theta_2)} = \mr{\cos(\theta_2)}{-\sin(\theta_2)}{0} {\sin(\theta_2)}{\cos(\theta_2)}{0} {0}{0}{1}$ | rotation matrix for rotation around $Z$ axis (axis 2), transforming a vector from frame 1 to frame 0 |
| Position | $\LU{0}{\pv} = [p_0,\, p_1,\, p_2]\tp$ | global position vector with 3 position coordinates $p_i$ in any configuration |
| Velocity | $\LU{0}{\vv} = \LU{0}{\dot \uv} = [v_0,\, v_1,\, v_2]\tp$ | global velocity vector with 3 displacement coordinates $v_i$ in any configuration |
| AngularVelocity | $\LU{0}{\tomega} = [\omega_0,\, \ldots,\, \omega_2]\tp$ | global angular velocity vector with $3$ coordinates $\omega_i$ in any configuration |
| Acceleration | $\LU{0}{\av} = \LU{0}{\ddot \uv} = [a_0,\, a_1,\, a_2]\tp$ | global acceleration vector with 3 displacement coordinates $a_i$ in any configuration |
| AngularAcceleration | $\LU{0}{\talpha} = \LU{0}{\dot \tomega} = [\alpha_0,\, \ldots,\, \alpha_2]\tp$ | global angular acceleration vector with $3$ coordinates $\alpha_i$ in any configuration |
| VelocityLocal | $\LU{b}{\vv} = [v_0,\, v_1,\, v_2]\tp$ | local (body-fixed) velocity vector with 3 displacement coordinates $v_i$ in any configuration |
| AngularVelocityLocal | $\LU{b}{\tomega} = [\omega_0,\, \ldots,\, \omega_2]\tp$ | local (body-fixed) angular velocity vector with $3$ coordinates $\omega_i$ in any configuration |
| Force | $\LU{0}{\fv} = [f_0,\, \ldots,\, f_2]\tp$ | vector of $3$ force components in global coordinates |
| Torque | $\LU{0}{\ttau} = [\tau_0,\, \ldots,\, \tau_2]\tp$ | vector of $3$ torque components in global coordinates |

The following table collects some typical **input parameters** for nodes, objects and markers:

| **python name** | **symbol** | **description** |
|---|---|---|
| referenceCoordinates | $\cv\cRef = [c_0,\, \ldots,\, c_n]\cRef\tp = [c_{\mathrm{Ref},0},\, \ldots,\, c_{\mathrm{Ref},n}]\cRef\tp$ | $n$ coordinates of reference configuration (can usually be set at initialization of nodes) |
| initialCoordinates | $\cv\cIni$ | initial coordinates with generalized or mixed displacement/rotation quantities (can usually be set at initialization of nodes) |
| reference point | $\pRefG = [r_0,\, r_1,\, r_2]\tp$ | reference point of body, e.g., for rigid bodies or {ref}`FFRF <FFRF>` bodies, in any configuration; NOTE: for ANCF elements, $\pRefG$ is used for the position vector to the beam centerline |
| localPosition | $\pLocB = [\LUR{b}{b}{0},\, \LUR{b}{b}{1},\, \LUR{b}{b}{2}]\tp$ | local (body-fixed) position vector with 3 position coordinates $b_i$ in any configuration, measured relative to reference point; NOTE: for rigid bodies, $\LU{0}{\pv} = \pRefG + \LU{0b}{\Rot} \pLocB$; localPosition is used for definition of body-fixed local position of markers, sensors, COM, etc. |

(sec-equationlhsrhs)=
## LHS-RHS naming conventions in EXUDYN

The general idea of the Exudyn is to have objects, which provide equations ({ref}`ODE2 <ODE2>`, {ref}`ODE1 <ODE1>`, {ref}`AE <AE>`).
The solver then assembles these equations and solves the static or dynamic problem.
The system structure and solver are similar but much more advanced and modular as earlier solvers by the main developer [GerstmayrStangl2004; Gerstmayr2009; GerstmayrEtAl2013].

Functions and variables contain the abbreviations {ref}`LHS <LHS>` and {ref}`RHS <RHS>`, sometimes lower-case, in order
to distinguish if terms are computed at the {ref}`LHS <LHS>` or {ref}`RHS <RHS>`.

The objects have the following {ref}`LHS <LHS>`--{ref}`RHS <RHS>` conventions:

- the acceleration term, e.g., $m \cdot \ddot q$ is always positive on the {ref}`LHS <LHS>`
- objects, connectors, etc., use {ref}`LHS <LHS>` conventions for most terms: mass, stiffness matrix, elastic forces, damping, etc., are computed at {ref}`LHS <LHS>` of the object equation
- object forces are written at the {ref}`RHS <RHS>` of the object equation
- in case of constraint or connector equations, there is no {ref}`LHS <LHS>` or {ref}`RHS <RHS>`, as there is no acceleration term.

Therefore, the computation function evaluates the term as given in the description of the object, adding it to the {ref}`LHS <LHS>`.
Object equations may read, e.g., for one coordinate $q$, mass $m$, damping coefficient $d$, stiffness $k$ and applied force $f$,

$$
  \underbrace{m \cdot \ddot q + d \cdot \dot q + k \cdot q}_{LHS} = \underbrace{f}_{RHS}
$$
In this case, the C++ function `ComputeODE2LHS(const Vector& ode2Lhs)` will compute the term
$d \cdot \dot q + k \cdot q$ with positive sign. Note that the acceleration term $m \cdot \ddot q$ is computed separately, as it
is computed from mass matrix and acceleration.

However, system quantities (e.g. within the solver) are always written on {ref}`RHS <RHS>` (except for the acceleration $\times$ mass matrix and constraint reaction forces, see {eq}`eq-system-eom`):

$$
  \underbrace{M_{sys} \cdot \ddot q_{sys}}_{LHS} = \underbrace{f_{sys}}_{RHS} \,.
$$
In the case of the object equation

$$
  m \cdot \ddot q + d \cdot \dot q + k \cdot q = f \, ,
$$
the {ref}`RHS <RHS>` term becomes $f_{sys} = -(d \cdot \dot q + k \cdot q) + f $ and it is computed by the C++ function `ComputeSystemODE2RHS`.
This means, that during computation, terms which appear at the {ref}`LHS <LHS>` of the object are transferred to the {ref}`RHS <RHS>` of the system equation.
This enables a simpler setup of equations for the solver.

## System assembly

Assembling equations of motion is done within the C++ class `CSystem`, see the file `CSystem.cpp`.
The general idea is to assemble, i.e. to sum up, (parts of) residuals attributed by different objects. The summation process is based on coordinate indices to which the single equations belong to.
Let's assume that we have two simple `ObjectMass1D` objects, with object indices $o0$ and $o1$ and having mass $m_0$ and $m_1$. They are connected to nodes of type `Node1D` $n0$ and $n1$, with global coordinate indices $c0$ and $c1$.
The partial object residuals, which are fully independent equations, read

$$
\begin{aligned}
  m_0 \cdot \ddot q_{c0} &= RHS_{c0} \, , \\
  m_1 \cdot \ddot q_{c1} &= RHS_{c1} \, ,
\end{aligned}
$$
where $RHS_{c0}$ and $RHS_{c1}$ the right-hand-side of the respective equations/coordinates. They represent forces, e.g., from `LoadCoordinate` items (which directly are applied to coordinates of nodes), say $f_{c0}$ and $f_{c1}$, that are in case also summed up on the right hand side.
Let us for now assume that

$$
  RHS_{c0} = f_{c0} \quad \mathrm{and} \quad RHS_{c1} = f_{c1} \, .
$$
Now we add another `ObjectMass1D` object with object index $o2$, having mass $m_2$, but letting the object *again* use node $n0$ with coordinate $c0$.
In this case, the total object residuals read

$$
\begin{aligned}
  (m_0+m_2) \cdot \ddot q_{c0} &= RHS_{c0} \, , \\
  m_1 \cdot \ddot q_{c1} &= RHS_{c1} \, .
\end{aligned}
$$
It is clear, that now the mass in the first equation is increased due to the fact that two objects contribute to the same coordinate. The same would happen, if several loads are applied to the same coordinate.

Finally, if we add a `CoordinateSpringDamper`, assuming a spring $k$ between coordinates $c0$ and $c1$, the {ref}`RHS <RHS>` of equations related to $c0$ and $c1$ is now augmented to

$$
\begin{aligned}
  RHS_{c0} &= f_{c0} + k \cdot (q_{c1} - q_{c0}) \, , \\
  RHS_{c1} &= f_{c1} + k \cdot (q_{c0} - q_{c1}) \, .
\end{aligned}
$$
The system of equation would therefore read

$$
\begin{aligned}
  (m_0+m_2) \cdot \ddot q_{c0} &= f_{c0} + k \cdot (q_{c1} - q_{c0}) \, , \\
  m_1 \cdot \ddot q_{c1}  &= f_{c1} + k \cdot (q_{c0} - q_{c1}) \, .
\end{aligned}
$$
It should be noted, that all (components of) residuals ('equations') are summed up for the according coordinates, and also all contributions to the mass matrix.
Only constraint equations, which are related to Lagrange parameters always get their 'own' Lagrange multipliers, which are automatically assigned by the system and therefore independent for every constraint.

(sec-nomenclatureeom)=
## Nomenclature for system equations of motion and solvers

Using the basic notation for coordinates in {ref}`sec-generalnotation`, we use the following quantities and symbols for equations of motion and solvers:

| **quantity** | **symbol** | **description** |
|---|---|---|
| number of {ref}`ODE2 <ODE2>` coordinates | $n_q$ | {ref}`ODE2 <ODE2>` |
| number of {ref}`ODE1 <ODE1>` coordinates | $n_\FO$ | {ref}`ODE1 <ODE1>` |
| number of {ref}`AE <AE>` coordinates | $m$ | {ref}`AE <AE>` |
| number of system coordinates | $n_{\SYS}$ | number of coordinates of the system equations |
| {ref}`ODE2 <ODE2>` coordinates | $\qv = [q_0,\, \ldots,\, q_{n_q}]\tp$ | {ref}`ODE2 <ODE2>`, displacement-based coordinates (could also be rotation or deformation coordinates) |
| {ref}`ODE2 <ODE2>` velocities | $\vel = \dot \qv = [\dot q_0,\, \ldots,\, \dot q_{n_q}]\tp$ | {ref}`ODE2 <ODE2>` velocity coordinates |
| {ref}`ODE2 <ODE2>` accelerations | $\ddot \qv = [\ddot q_0,\, \ldots,\, \ddot q_{n_q}]\tp$ | {ref}`ODE2 <ODE2>` acceleration coordinates |
| {ref}`ODE1 <ODE1>` coordinates | $\yv = [y_0,\, \ldots,\, y_{n_y}]\tp$ | vector of $n_y$ coordinates for {ref}`ODE1 <ODE1>` |
| {ref}`ODE1 <ODE1>` velocities | $\dot \yv = [\dot y_0,\, \ldots,\, \dot y_{n_y}]\tp$ | vector of $n$ velocities for {ref}`ODE1 <ODE1>` |
| {ref}`ODE2 <ODE2>` Lagrange multipliers | $\tlambda = [\lambda_0,\, \ldots,\, \lambda_m]\tp$ | vector of $m$ Lagrange multipliers (=algebraic coordinates), representing the linear factors (often forces or torques) to fulfill the algebraic equations; for {ref}`ODE1 <ODE1>` and {ref}`ODE2 <ODE2>` coordinates |
| data coordinates | $\xv = [x_0,\, \ldots,\, x_l]\tp$ | vector of $l$ data coordinates in any configuration |
| {ref}`RHS <RHS>` {ref}`ODE2 <ODE2>` | $\fv_\SO\in \Rcal^{n_q}$ | right-hand-side of {ref}`ODE2 <ODE2>` equations; (all terms except mass matrix $\times$ acceleration and joint reaction forces) |
| {ref}`RHS <RHS>` {ref}`ODE1 <ODE1>` | $\fv_\SO\in \Rcal^{n_y}$ | right-hand-side of {ref}`ODE1 <ODE1>` equations |
| {ref}`AE <AE>` | $\gv\in \Rcal^{m}$ | algebraic equations |
| mass matrix | $\Mm\in \Rcal^{n_q \times n_q}$ | mass matrix, only for {ref}`ODE2 <ODE2>` equations |
| (tangent) stiffness matrix | $\Km\in \Rcal^{n_q \times n_q}$ | includes all derivatives of $\fv_\SO$ w.r.t. $\qv$ |
| damping/gyroscopic matrix | $\Dm\in \Rcal^{n_q \times n_q}$ | includes all derivatives of $\fv_\SO$ w.r.t. $\vel$ |
| step size | $h$ | current step size in time integration method |
| residual | $\rv_\SO \in \Rcal^{n_q}$, $\rv_\FO \in \Rcal^{n_y}$, $\rv_\AE \in \Rcal^{m}$ | residuals for each type of coordinates within static/time integration -- depends on method |
| system residual | $\rv\in \Rcal^{n_s}$ | system residual -- depends on method |
| system coordinates | $\txi$ | system coordinates and unknowns for solver; definition depends on solver |
| Jacobian | $\Jm\in \Rcal^{n_s \times n_s}$ | system Jacobian -- depends on method |
