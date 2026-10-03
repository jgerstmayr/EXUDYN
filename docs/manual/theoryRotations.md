# Frames, rotations and coordinate systems

In this section, we introduce frames, which provide reference points and rotations attached to the ground or to bodies, thus providing a bases for coordinate systems.

## Reference points and reference frames

In contrast to the reference coordinates (which may include reference position and rotations), the reference point of a body, marker or joint is available in any configuration (current, initial, etc.).
The reference point and orientation (or reference frame) of a rigid body coincides with the position and orientation of the underlying node.
The local position of a point of the body is expressed relative to the body's reference frame, which may be different from the body's center of mass.
The reference frame is also used for {ref}`FFRF <FFRF>` objects. In most cases, reference points are denoted by $\pRef$.

## Coordinate systems of frames

Many considerations and computations in kinematics can be formulated coordinate-free.
However, ultimately local positions are given by the user in different coordinates from results given in global (world) coordinates.

For this reason, we use left upper indices in symbols to indicate underlying frames of points or vectors, e.g.,
$\LU{0}{\uv}$ represents a displacement vector in the global (world) coordinate system $0$, while
$\LU{m1}{\uv}$ represents the displacement vector in marker 1 ($m1$) coordinates.

Typical coordinate systems (frames) are:

- $\LU{0}{\uv}$ $\ldots$ global or world coordinates
- $\LU{b}{\uv}$ $\ldots$ body-fixed, local coordinates
- $\LU{m0}{\uv}$ $\ldots$ local coordinates of (the body or node of) marker $m0$
- $\LU{m1}{\uv}$ $\ldots$ local coordinates of (the body or node of) marker $m1$
- $\LU{J0}{\uv}$ $\ldots$ local coordinates of joint $0$, related to marker $m0$
- $\LU{J1}{\uv}$ $\ldots$ local coordinates of joint $1$, related to marker $m1$

To transform the local coordinates $\LU{m0}{\uv}$ of marker 0 into global coordinates $\LU{0}{\xv}$, we use the relation

$$
  \LU{0}{\uv} = \LU{0,m0}{\Rot} \LU{m0}{\uv}
$$ (eq-theory-coordinatesystems-rot)

in which $\LU{0,m0}{\Rot}$ is the transformation matrix of (the body or node of) the underlying marker 0.

In particular, note that any transformation, e.g., of the angular velocity in body (b) and world coordinates,

$$
  \LU{0}{\tomega} = \LU{0,b}{\Rot} \LU{b}{\tomega}
$$
only transforms the vector from one frame to the other, but does not change the vector $\tomega$ itself.

## Homogeneous transformations

In order to considere rotations and translations in frames, homogeneous transformations are used.
An arbitrary vector $\LU{i}{\rv}$ in coordinates of frame $\mathcal{F}_i$ can be expressed relative to $\mathcal{F}_j$ with the vector $\LU{j}{\pv_i}$, pointing from $O_j$ to $O_i$ and a rotation matrix according to {eq}`eq-theory-coordinatesystems-rot`:

$$
  \LU{j}{\rv} = \LU{ji}{\Rm} \LU{i}{\rv} + \LU{j}{\pv_i}
$$ (eq-theory-ht-positionrelation)

{eq}`eq-theory-ht-positionrelation` can be written as a linear mapping,

$$
  \vp{\LU{j}{\rv}}{1} = \mp{\LU{ji}{\Rm}}{\LU{j}{\pv_i}}{\Null^T}{1} \vp{\LU{i}{\rv}}{1}, \quad
$$
in which the matrix

$$
  \LU{ji}{\Tm} = \mp{\LU{ji}{\Rm}}{\LU{j}{\pv_i}}{\Null^T}{1}
$$
is referred to as the **homogeneous transformation matrix**, and $\LU{j}{\hat \rv} = [\LU{j}{\rv} \;\; 1]^T$ is the homogeneous vector corresponding to $\LU{j}{\rv}$.

Analogous to the rotation matrix, $\LU{ji}{\Tm}$ has an inverse, which follows as

$$
  \LURU{ji}{\Tm}{}{-1} = \LU{ij}{\Tm} = \mp{\LURU{ji}{\Rm}{}{T}} {-\LURU{ji}{\Rm}{}{T} \LU{j}{\pv_i}}{\Null^T}{1} \vp{\LU{j}{\rv}}{1} \, .
$$
Sequential application of homogeneous transformations simplifies to

$$
  \LU{ki}{\Tm} = \LU{kj}{\Tm} \LU{ji}{\Tm} \, .
$$
We could also actively move vectors, by assuming that the homogeneous transformation matrix $\LU{i0,i1}{\Tm}$ transforms coordinates within the same frame $\mathcal{F}_i$, between two (time) steps 0 and 1,

$$
  \LU{i1}{\hat \rv} = \LU{i1,i0}{\Tm} \LU{i0}{\hat \rv}
$$
which advances the vector $\LU{i0}{\hat \rv}$ in time.

Note that an efficient implementation would only include $\Rm$ and $\pv$, without necessarily computing $4 \times 4$ matrices.

(fig-theory-ht-changeofframe)=
```{figure} /docs/figures/theoryRotationsHTchangeOfFrame.png
:width: 320

Homogeneous transformation from $\mathcal{F}_i$ to $\mathcal{F}_j$ using the relation $\LU{j}{\hat{\mathbf{r}} } = \LU{ji}{\Tm} \LU{i}{\hat{\mathbf{r}} }$
```

All homogeneous transformations form the **Special Euclidean Group SE(3)** -- the motion group -- for describing rigid body movements in 3D. Therefore, homogeneous transformations also meet the criteria for groups (closure, associativity, existence of neutral and inverse elements).

Elementary rotations about the $\ev_z$-axis and translations along the $\ev_x$-axis, for example, are given by:

$$
  \mathbf{Rot}(\ev_z, \theta) = \vfour{\cos \theta ,\; -\sin \theta ,\; 0 ,\; 0}{\sin \theta ,\; \cos \theta ,\; 0 ,\; 0}{0 ,\; 0 ,\; 1 ,\; 0}{0 ,\; 0 ,\; 0 ,\; 1}, \quad
  \mathbf{Trans}(\ev_x, d) = \vfour{1 ,\; 0 ,\; 0 ,\; d}{0 ,\; 1 ,\; 0 ,\; 0}{0 ,\; 0 ,\; 1 ,\; 0}{0 ,\; 0 ,\; 0 ,\; 1}
$$
Arbitrary composite transformations can be constructed from elementary transformations. In general, the following applies:

- Pre-multiplication (**Pre-Multiplication**): Transformation in the global coordinate system
- Post-multiplication (**Post-Multiplication**): Transformation in the rotated coordinate system

By exploiting the tools of so-called Lie groups, rigid body movements can be interpolated using matrix logarithm (`LogSE3(...)`) and matrix exponential function (`ExpSE3(...)`).

## Rotations

In this section, we discuss in particular rotations, as already introduced for coordinate systems and homogeneous transformations.
Rotations can be represented by transformation matrices, leading to a linear transformation

$$
   \LU{0}{\rv} = \LU{0,1}{\Rot} \LU{1}{\rv}
$$
which transforms the vector $\LU{1}{\rv}$ accordingly.
However, rotations are inherently nonlinear. First, rotation parametrizations -- except for the components of the rotation matrix itself -- are highly nonlinear functions of rotation parameters.
Second, subsequent rotations couple in a multiplicative (i.e., nonlinear) way. Therefore rotations have to be treated differently from translations.

### Derivation of the transformation matrix from coordinate transformations

The vector $\av$ can be represented in coordinates of frame $\mathcal{F}_1$ with basis vectors $(\ev_{x1},\,\ev_{y1},\,\ev_{z1})$ and of frame $\mathcal{F}_2$ with basis vectors $(\ev_{x2},\,\ev_{y2},\,\ev_{z2})$,

$$
  \av = \LUR{1}{a}{x} \ev_{x1} + \LUR{1}{a}{y} \ev_{y1} +\LUR{1}{a}{z} \ev_{z1} =
        \LUR{2}{a}{x} \ev_{x2} + \LUR{2}{a}{y} \ev_{y2} +\LUR{2}{a}{z} \ev_{z2} \, .
$$ (eq-theory-rotations-trans1)

Multiplying {eq}`eq-theory-rotations-trans1` with the basis vectors $(\ev_{x1},\,\ev_{y1},\,\ev_{z1})$ consecutively, we obtain three relations,

$$
\begin{aligned}
  \LUR{1}{a}{x} &= \LUR{2}{a}{x} \ev_{x1}^T\ev_{x2} + \LUR{2}{a}{y} \ev_{x1}^T\ev_{y2} +\LUR{2}{a}{z} \ev_{x1}^T\ev_{z2} \, ,\\
  \LUR{1}{a}{y} &= \LUR{2}{a}{x} \ev_{y1}^T\ev_{x2} + \LUR{2}{a}{y} \ev_{y1}^T\ev_{y2} +\LUR{2}{a}{z} \ev_{y1}^T\ev_{z2} \, ,\\
  \LUR{1}{a}{z} &= \LUR{2}{a}{x} \ev_{z1}^T\ev_{x2} + \LUR{2}{a}{y} \ev_{z1}^T\ev_{y2} +\LUR{2}{a}{z} \ev_{z1}^T\ev_{z2} \, .
\end{aligned}
$$
where we apply the orthonormality relationships $\ev^T_{x1} \ev_{x1} = 1$, $\ldots$ as well as $\ev^T_{x1} \ev_{y1} = 0$, $\ev^T_{x1} \ev_{z1} = 0$, $\ldots$ througout.

We can represent the result as linear transformation

$$
  \vr{\LUR{1}{a}{x}}{\LUR{1}{a}{y}}{\LUR{1}{a}{z}} =
  \mr{\ev_{x1}^T\ev_{x2}}{\ev_{x1}^T\ev_{y2}}{\ev_{x1}^T\ev_{z2}}
     {\ev_{y1}^T\ev_{x2}}{\ev_{y1}^T\ev_{y2}}{\ev_{y1}^T\ev_{z2}}
     {\ev_{z1}^T\ev_{x2}}{\ev_{z1}^T\ev_{y2}}{\ev_{z1}^T\ev_{z2}}
  \vr{\LUR{2}{a}{x}}{\LUR{2}{a}{y}}{\LUR{2}{a}{z}}
$$
or in short

$$
  \LU{1}{\av} = \LU{12}{\Rot} \LU{2}{\av} \, .
$$
The **Transformationsmatrix** $\LU{12}{\Rot}$ transforms coordinates of vector $\av$ from $\mathcal{F}_2$ into $\mathcal{F}_1$.
We observe that the columns of $\LU{12}{\Rot}$ contain unit vectors $\ev_{x2}$, $\ev_{y2}$, and $\ev_{z2}$ represented in frame $\mathcal{F}_1$,
and that the rows can be identified as the unit vectors $\ev_{x1}^T$, $\ev_{y1}^T$, and $\ev_{z1}^T$ represented in frame $\mathcal{F}_2$:

$$
  \LU{12}{\Rot} = \left[ \LUR{1}{\ev}{x2} \quad \LUR{1}{\ev}{y2} \quad \LUR{1}{\ev}{z2}\right] =
  \vr{\LURU{2}{\ev}{x1}{\mathrm{T}}}{\LURU{2}{\ev}{y1}{\mathrm{T}}}{\LURU{2}{\ev}{z1}{\mathrm{T}}} \, .
$$
Note that:

- Columns and rows of $\LU{12}{\Rot}$ represent an orthonormal right-hand system.
- Because the determinant $\det(\LU{12}{\Rot})= + 1$, we call $\LU{12}{\Rot}$ to be proper-orthogonal.
- Rotations conserve length, $|\Am \xv| = |\xv|$.
- For the same reason, rotations conserve angles.
- During transformations, we observe that reading left upper indices from left to right, indices are changed only by transformation matrices (which are not transposed): $\LU{2}{\av} = \LU{21}{\Rot} \LU{1}{\av}$

We note the following rules:

$$
\begin{aligned}
  \LU{21}{\Rot} &= \LURU{12}{\Rot}{}{-1} \, , \\
  \LURU{12}{\Rot}{}{-1} &= \LURU{12}{\Rot}{}{T} \, , \\
  \LU{12}{\Rot} \LURU{12}{\Rot}{}{T} &= \Im \, , \\
  \LURU{12}{\Rot}{}{T} \LU{12}{\Rot} &= \Im \, .
\end{aligned}
$$
### Elementary rotations

Elementary rotations are rotations about a single axis, using one of the orthogonal basis vectors.
Consider basis $(\ev_{x1},\,\ev_{y1},\,\ev_{z1})$ and a rotation around axis $\ev_{x1}$ with angle $\varphi_1$ in positive rotation sense, obtaining a rotated frame $(\ev_{x2},\,\ev_{y2},\,\ev_{z2})$, see {ref}`fig-theory-rotations-elementaryx`.
The rotation matrix for this case reads:

$$
  \LU{12}{\Rot} = \left[ \LUR{1}{\ev}{x2} \;\; \LUR{1}{\ev}{y2} \;\; \LUR{1}{\ev}{z2} \right] =
  \mr{1}{0}{0}{0}{c \varphi_1}{-s \varphi_1}{0}{s \varphi_1}{c \varphi_1}\, .
$$
In this case, the coordinates of a vector $\rv$ are transformed according to

$$
  \LU{1}{\rv} = \LU{12}{\Rot} \LU{2}{\rv}
$$
(fig-theory-rotations-elementaryx)=
```{figure} /docs/figures/elementaryRotationX.*
:width: 320

Elementary rotation around axis $\mathbf{ x}_1$.
```

In a second example, a rotation with angle $\varphi_2$ around $\ev_{y2}$ is performed to transform from basis $(\ev_{x2},\,\ev_{y2},\,\ev_{z2})$ into $(\ev_{x3},\,\ev_{y3},\,\ev_{z3})$, see {ref}`fig-theory-rotations-elementaryy`.
The rotation matrix for this case reads:

$$
  \LU{23}{\Rot} = \left[ \LUR{2}{\ev}{x3} \;\; \LUR{2}{\ev}{y3} \;\; \LUR{2}{\ev}{z3} \right] =
  \mr{c \varphi_2}{0}{s \varphi_2}{0}{1}{0}{-s \varphi_2}{0}{c \varphi_2}\, .
$$
(fig-theory-rotations-elementaryy)=
```{figure} /docs/figures/elementaryRotationY.*
:width: 320

Elementary rotation around axis $\mathbf{ y}_2$.
```

Finally, a rotation around $\ev_{z3}$ with angle $\varphi_3$ would give similarly,

$$
  \LU{34}{\Rot} = \left[ \LUR{3}{\ev}{x4} \;\; \LUR{3}{\ev}{y4} \;\; \LUR{3}{\ev}{z4} \right] =
  \mr{c \varphi_3}{-s \varphi_3}{0} {s \varphi_3}{c \varphi_3}{0} {0}{0}{1} \, .
$$
### Linearized rotations

In this case we can interpret the small rotations as a constant angular velocity over small time $\Delta t$, $\boldVarPhi = \Delta t \cdot \tomega$, which results in

$$
  \rv_1^\prime= \rv_1 + \Delta t \cdot (\tomega \times \rv_1) = (\Im + \tilde \boldVarPhi) \rv_1
$$
Leading to the linearized rotation matrix

$$
  \Rot_\mathrm{lin} = \Im - \tilde \boldVarPhi
$$
Alternatively, using linearized rotations $\boldVarPhi = [\varphi_1,\;\varphi_2,\;\varphi_3]\tp$, with $|\boldVarPhi| \ll 1$, we can approximate $\sin \varphi_1$ as:

$$
  \sin \varphi_1 = \varphi_1 - \frac{\varphi_1^3}{3!} + \frac{\varphi_1^5}{5!} - ... \approx \varphi_1
$$
Similarly, we can approximate $\cos \varphi_1 \approx 1$. This immediately gives a linearization for elementary rotations -- where we observe that contributions for each axis can be added.

The axis-angle representation (see later) leads to the same result by linearization of the Rodrigues formula,

$$
  \Rot(\uv, \varphi) \approx \Im + \tilde \uv \varphi, \quad \Rot\tp \approx \Im - \tilde \uv \varphi
$$
in which $\varphi$ represents the infinitesimal angle and $\uv$ is the rotation axis.

**Note**: whenever you would like to work with linearized rotations, or if you do not know the sequence of small rotations, still do not use the linearized formula as it gives immediately rotation matrices without strict orthogonality and determinant of 1. In order to avoid computational problems, use for example `RotationVector2RotationMatrix(phi)` to compute a consistent rotation matrix based on the matrix exponential.

### Active and passive rotations

It is important to distinguish two fundamentally different concepts of rotations.
There is the **active rotation**, which can be represented by a coordinate-free relation,

$$
  \vv^\prime = \Rot \vv
$$ (eq-theory-rotations-active)

in which a vector $\vv$ is actively rotated into a new configuration $\vv^\prime$ using a rotation tensor $\Rot$.
{eq}`eq-theory-rotations-active` can be represented in any coordinate system, but keeping it fixed.

Furthermore, we denote a **passive rotation** as a coordinate transformation, as used many times before,

$$
  \LU{1}{\vv} = \LU{12}{\Rot} \LU{2}{\vv}
$$
in which the vector $\vv$ is not changed at all, but only its coordinates are represented in two different coordinate systems.

While passive rotations (and homogeneous transformations) are used, e.g., to compute current positions of body-attached points through several relative coordinate systems, active rotations can represent the rotations from one time step to the next one.

### Successive rotations

The successive application of rotation matrices is non-commutative. An exception is the planar case, i.e., all rotations occur around the same axis.
Specifically, we see that for successive rotations, the following must be considered:

$$
  \Am_2 \Am_1 \vv \neq \Am_1 \Am_2 \vv \, .
$$
As an example, we consider in Figure {ref}`fig-theory-rotations-successive` the different order of rotations of a block.

(fig-theory-rotations-successive)=
```{figure} /docs/figures/RotationsSequences.*
:width: 500

Successive rotations are not commutative.
```

In {ref}`fig-theory-rotations-successive`a, the block shown is first rotated 90° about the $\mathbf{e}_2$ axis and then 90° about the $\mathbf{e}_3$ axis. In Figure {ref}`fig-theory-rotations-successive`b, the same rotations are applied in reverse order, i.e., the block is first rotated 90° about the $\mathbf{e}_3$ axis and then 90° about the $\mathbf{e}_2$ axis. It is immediately apparent that the resulting orientation of the block is different in both cases.

Finally, it should be noted that there are two different types of successive rotations.
In the first variant, also known as the **single-frame method**, the same reference frame is chosen for each rotation. This variant is best understood in the active rotation of vectors, where these rotations always take place, for example, in the global reference system.

In the second variant, also known as the **multi-frame method**, a new reference frame created by the preceding rotation is used for each rotation. This variant can be illustrated by applications in robotics (e.g., articulated arm robots), where (passive) rotations of the reference systems of the robot's respective arms occur, with each rotation taking place in the reference system of the preceding arm.

## Rotation tensor: axis-angle representation

In the following, we will elaborate the rotation tensor, inherently linked to the axis-angle representation.
Note that the rotation axis times the angle is denoted as rotation vector throughout.

Considering Euler's theorem on rotations, we assume a transformation

$$
  \rv_0 \ra \rv(t)
$$
which purely follows from a rotation. This transformation thus can be represented by a rotation axis $\uv$ and an angle $\varphi(t)$,

$$
  (\uv(t), \, \varphi(t))
$$
both of them given in a coordinate-free manner.
We may therefore conclude

$$
  \rv(t)=\Rot(t) \, \rv_0 \qquad \text{with} \qquad \Rot(t)=\Rot(\uv(t),\varphi(t))
$$
(fig-theory-rotations-angleaxis)=
```{figure} /docs/figures/RotationAxisAngle.*
:width: 260

Rotation of a vector $\mathbf{ r}_0$ by means of the angle-axis tuple $(\mathbf{ u}(t), \, \varphi(t))$.
```

Using {ref}`fig-theory-rotations-angleaxis`, we may now consider relations of the two frames $(\ev_{x0},\,\ev_{y0},\,\ev_{z0})$ and $(\ev_{x1},\,\ev_{y1},\,\ev_{z1})$, solely defined by the angle-axis $(\uv(t), \, \varphi(t))$ relation.

(fig-theory-rotations-axisanglederivation)=
```{figure} /docs/figures/RotationAxisAngleDerivation.*
:width: 320

Relations for derivation of rotation tensor and Rodrigues' formula.
```

According to {ref}`fig-theory-rotations-axisanglederivation`, we may split the vector $\rv$, which has the length $r = \vert \rv \vert$, into

$$
  \rv= c_1 \ev_1 + c_2 \ev_2 + c_3 \ev_3
$$
with the unit vectors given as

$$
  \ev_1  =  \frac{\rv_0 - \uv(\uv^{\mathrm{T}} \rv_0)}{r \sin \delta},  \quad
  \ev_2  =  \frac{\tilde{\uv} \rv_0}{r \sin \delta}, \quad \mathrm{and} \quad
  \ev_3  =   \uv \, .
$$
The coefficients can be calculated as follows,

$$
  c_1 = r \sin \delta \cos \varphi, \quad
  c_2 = r \sin \delta \sin \varphi, \quad \mathrm{and} \quad
  c_3 = r \cos \delta
$$
Putting everything together gives

$$
  \rv = \cos \varphi \, \rv_0 + \sin \varphi \, \tilde{\uv} \, \rv_0 + (1 - \cos \varphi) \, \uv \, (\uv\tp \, \rv_0)
$$
From the relation $\rv = \Rot(\uv ,\varphi) \rv_0$, we can reinterpret the **rotation tensor** $\Rot$

$$
  \Rot(\uv ,\varphi) = \cos \varphi \, \Im + \sin \varphi \, \tilde{\uv} + (1-\cos \varphi) \, \uv \, \uv\tp
$$
With the additional relations

$$
  \uv \, \uv\tp=\tilde\uv \, \tilde\uv +(\uv\tp \uv) \Im \quad \mathrm{and} \quad \uv\tp \uv = 1
$$
we finally obtain

$$
  \Rot(\uv ,\varphi) =  \Im + \sin \varphi \, \tilde{\uv} + (1-\cos \varphi) \,  \tilde{\uv} \, \tilde{\uv}
$$
which is also known as the **Rodrigues formula** for rotations.
While this formula is coordinate-free, it may be represented in any coordinate system, such as

$$
  \LU{0}{\rv}(t)=\LU{0}{\Rot}(t) \, \LUR{0}{\rv}{0} \qquad \text{with} \qquad \LU{0}{\Rot}(t)=\Rot(\LU{0}{\uv}(t),\varphi(t))
$$
### Rotation vector

Using the rotation vector $\vv_\mathrm{rot} = \varphi\cdot \uv$ and $\varphi = |\vv_\mathrm{rot}|$, another representation of the rotation tensor follows as

$$
  \Rot(\uv ,\varphi) =  \Im + \frac{\sin \varphi}{\varphi} \tilde\vv_\mathrm{rot} + \frac{(1-\cos \varphi)}{\varphi^2} \,  \tilde\vv_\mathrm{rot} \, \tilde\vv_\mathrm{rot}
$$
Note, that we inherently assume that $\varphi \in [0,\pi]$.
 In Exudyn, there are the following **functions and items related to the rotation vector** and the axis-angle representation:

- `NodeRigidBodyRotVecLG`: A 3D rigid body node based on rotation vector and Lie group methods; can be used for explicit integration methods, not leading to singularities when integrating with Lie-group methods
- `RotationVector2RotationMatrix`: computes the rotation matrix from a given rotation vector $\vv_\mathrm{rot}$
- `RotationMatrix2RotationVector`: computes the rotation vector $\vv_\mathrm{rot}$ from a given rotation matrix, based on a reconstruction of Euler parameters
- `ComputeRotationAxisFromRotationVector`: computes the rotation axis $\uv$ from the rotation vector by using $\varphi = |\vv_\mathrm{rot}|$

Note the following **properties of the rotation tensor**:

- The axis-angle representations $(\uv, \varphi)$ and $(-\uv, -\varphi)$ represent the same tensor $\Rot(\uv,\varphi)=\Rot(-\uv,-\varphi)$
- The rotation tensor $\Rot$ is orthogonal, i.e., $\Rot\tp \, \Rot = \Em$.
- The inverse rotation tensor $\Rot^{-1} = \Rot\tp$ is represented by the identical axis-angle tuples $(\uv, -\varphi)$ and $(-\uv, \varphi)$, leading to $\rv_0=\Rot(\uv, -\varphi) \rv$.
- We furthermore not the identities: $\Rot(\uv, -\varphi) \equiv \Rot(-\uv, \varphi) \equiv \Rot\tp (\uv, \varphi)$
- The rotation axis is invariant to the rotation: $\Rot(\uv, \varphi) \uv = \uv$

## Euler angles: Tait-Bryan angles

In the following section, we discuss rotation matrices built by Tait-Bryan angles, which are consecutive $xyz$-rotations.
Note that there are many other representations of Euler angles, however, they are not implemented or used in Exudyn.
The output variable `Rotation` gives Tait-Bryan angles, which is why it is important to define their properties.

(fig-theory-rotations-taitbryanangles)=
```{figure} /docs/figures/theoryRotationsTaitBryanAngles.png
:width: 240

Definition of Tait-Bryan angles.
```

 The transformation given by Tait-Bryan angles $(\alpha,\beta,\gamma)$ follows from three successive rotations,

$$
  \LU{0}{\rv} = \LU{01}{\Rot}(\alpha) \, \LU{12}{\Rot}(\beta) \, \LU{23}{\Rot}(\gamma) \, \LU{3}{\rv} \quad
  \mathrm{and} \quad \LU{0}{\rv} = \LU{03}{\Rot}(\alpha,\beta,\gamma) \, \LU{3}{\rv}
$$
The **Tait-Bryan rotation matrix** is thus defined as

$$
  \LU{03}{\Rot}(\alpha,\beta,\gamma) = \mr{\co\beta \,\co\gamma}{-\co\beta\,\si\gamma}{\si\beta} {\co \alpha \,\si \gamma + \si \alpha \,\si \beta \,\co \gamma}{\co \alpha \,\co\gamma-\si\alpha\,\si\beta\,\si\gamma}{-\si\alpha\,\co\beta} {\si\alpha\,\si\gamma-\co\alpha\,\si\beta\,\co\gamma}{\si\alpha\,\co\gamma+\co\alpha\,\si\beta\,\si\gamma}{\co\alpha\,\co\beta}
$$
While not fully obvious from the latter equations, we note that there is a singularity at $|\beta| = \frac{\pi}{2}$, which causes any computations to stall whenever getting close enough to this value.

 It is also possible to reconstruct Tait-Bryan angles from a given rotation matrix.
Assume, we have the rotation matrix $\LU{03}{\Rot}=(A_{ij})$.
There are two solutions for $\beta$ which follow from

$$
  \cos \beta = \pm \sqrt{1-A_{13}^2}, \quad \mathrm{and} \quad \sin \beta = A_{13}
$$
For beta fulfilling $|\beta| \ne \frac{\pi}{2}$, we can uniquely compute $\gamma$ and $\alpha$,

$$
  \cos \gamma = \frac{A_{11}}{\cos \beta}, \quad \sin \gamma = - \frac{A_{12}}{\cos \beta}, \quad
  \cos \alpha = \frac{A_{33}}{\cos \beta}, \quad \mathrm{and} \quad \sin \alpha = - \frac{A_{23}}{\cos \beta} \, .
$$
Note that improvements are possible using the $\mathrm{atan2}()$ function.
We can finally resolve non-uniqueness by restricting $\beta$ within the range $-\frac{\pi}{2} < \beta < \frac{\pi}{2}$.

### Tait-Bryan angles: angular velocity vector

Angular velocities are essential for the formulation of equations of motion of rigid bodies, for constraints and for evaluation.
Therefore, basic relations are shown for Tait-Bryan angles, in particular the velocity transformation matrix $\Gm_{TB}$.

The angular velocity vector $\omega$ can either be computed from the basic relation $\dot \Rot = \tilde \omega \Rot$, which however involves many trigonometric terms, or from partial angular velocities, being part of the co-rotated rotation axes.
With the latter approach, we see that

$$
  \tomega_{30}=\dot{\alpha} \ev_{x0}+\dot{\beta} \ev_{y1}+\dot{\gamma} \ev_{z2} \quad \text{or} \quad
  \tomega_{30}=\underbrace{[\ev_{x0} \quad \ev_{y1} \quad \ev_{z2}]}_{\Gm_{TB}} \vr{\dot{\alpha}}{\dot{\beta}}{\dot{\gamma}}
$$
We can now evaluate this equation in frame $\mathcal{F}_0$ with $\tbeta=[\alpha\;\beta\;\gamma]\tp$, written as

$$
  \LUR{0}{\tomega}{30}=\underbrace{[\LUR{0}{\ev}{x0} \quad \LUR{0}{\ev}{y1} \quad \LUR{0}{\ev}{z2}]}_{\LUR{0}{\Gm}{TB}} \vr{\dot{\alpha}}{\dot{\beta}}{\dot{\gamma}} =
  \LUR{0}{\Gm}{TB}(\tbeta) \dot{\tbeta} \, .
$$
Evaluating the consecutive rotations, we find the respective unit vectors as

$$
  \LUR{0}{\ev}{x0} = \vr{1}{0}{0}, \quad
  \LUR{0}{\ev}{y1} = \vr{0}{\text{c} \alpha}{\text{s} \alpha}, \quad \mathrm{and} \quad
  \LUR{0}{\ev}{z2} = \vr{\text{s} \beta}{-\text{s} \alpha \text{c} \beta}{\text{c} \alpha \text{c} \beta} \, .
$$
which results in the relations for the (global) angular velocity vector

$$
  \vr{\LUR{0}{\omega}{30x}}{\LUR{0}{\omega}{30y}}{\LUR{0}{\omega}{30z}} =
  \mr{1}{0} {\text{s} \beta}
  {0}{\text{c} \alpha} {-\text{s} \alpha \text{c} \beta}
  {0}{\text{s} \alpha} {\text{c} \alpha \text{c} \beta}
  \vr{\dot{\alpha}}{\dot{\beta}}{\dot{\gamma}}
$$ (eq-theory-rotations-taitbryang)

The matrix $\LUR{0}{\Gm}{TB}$ is used to compute partial derivatives of the angular velocity vector w.r.t. rotations.
However, it is also used to project torques and angular velocities into a space given by $\Gm_{TB}^\mathrm{T}$ (which is indeed not equivalent to $\Gm_{TB}^{-1}$, but gives a symmetric mass matrix -- see the equations of motion for the rigid body).

The inverse relation between $\dot \tbeta$ and $\LUR{0}{\tomega}{30}$ gives

$$
  \vr{\dot{\alpha}}{\dot{\beta}}{\dot{\gamma}} = \frac{1}{\text{c} \beta} \mr{\text{c} \beta}{\text{s} \alpha \, \text{s} \beta}{-\text{c} \alpha \, \text{s} \beta}{0}{\text{c} \alpha \, \text{c} \beta}{\text{s} \alpha \text{c} \beta}{0}{-\text{s} \alpha}{\text{c} \alpha}  \vr{\LUR{0}{\omega}{30x}}{\LUR{0}{\omega}{30y}}{\LUR{0}{\omega}{30z}}
$$ (eq-theory-rotations-taitbryanginv)

or in compact form

$$
  \dot{\tbeta} = \LURU{0}{\Gm}{TB}{-1}(\tbeta)  \LUR{0}{\tomega}{30}
$$
We again see that for the case $\vert \beta \vert = \pi/2$ the matrix$\LUR{0}{\Gm}{TB}$ becomes singular, and {eq}`eq-theory-rotations-taitbryanginv` cannot be resolved for $\dot{\tbeta}$!

 Finally, we also provide the relations of $\tomega_{30}$ in body-fixed (local) coordinates $\mathcal{F}_3$,
which are frequently used in the implementation,

$$
  \vr{\LUR{3}{\omega}{30x}}{\LUR{3}{\omega}{30y}}{\LUR{3}{\omega}{30z}} = \mr{\text{c} \beta \, \text{c} \gamma}{\text{s} \gamma}{0}{-\text{c} \beta \, \text{s} \gamma}{\text{c} \gamma}{0}{\text{s} \beta}{0}{1} \vr{\dot{\alpha}}{\dot{\beta}}{\dot{\gamma}} \, ,
$$
which reads in compact form ($\LUR{3}{\Gm}{TB} = \LUR{b}{\Gm}{TB}$),

$$
  \LUR{3}{\tomega}{30} =  \LUR{3}{\Gm}{TB}(\tbeta) \dot{\tbeta}
$$
 In Exudyn, there are the following **functions and items related to Tait-Bryan angles**:

- `NodeRigidBodyRxyz`: A 3D rigid body node based on Tait-Bryan angles
- `RotXYZ2RotationMatrix`: computes the rotation matrix from given Tait-Bryan angles
- `RotationMatrix2RotXYZ`: computes Tait-Bryan angles from a given rotation matrix
- `RotXYZ2G`: computes the matrix $\LU{0}{\Gm}$ relating time derivatives of Tait-Bryan angles to the (global) angular velocity vector
- `RotXYZ2Glocal`: computes the matrix $\LU{b}{\Gm}$ relating time derivatives of Tait-Bryan angles to the (local, body-fixed) angular velocity vector

## Euler parameters and unit quaternions

As one of the most important forms of rotational parameters for multibody systems, the 4 Euler parameters are introduced.
They can be mathematically represented as unit quaternions and provide particularly simple expressions and equations of motion, albeit at the expense of an additional constraint.

The Euler parameters are defined as

$$
  \underline{\pv}=\vp{p_{\mathrm{s}}}{\pv}=\vp{\cos \frac \varphi 2}{\uv \, \sin \frac\varphi 2} \quad \text{bzw.} \quad \underline{\pv}=\vfour{p_{\mathrm{s}}}{p_x}{p_y}{p_z}=\vfour{\cos \frac\varphi 2}{u_x \, \sin \frac\varphi 2}{u_y \, \sin \frac\varphi 2}{u_z \, \sin \frac\varphi 2}
$$ (eq-theory-rotations-eulerparametersdef)

Here, $p_s=\cos \frac\varphi 2$ is the scalar part and $\pv = \uv \, \sin \frac\varphi 2$ is the vector part of the Euler parameters $\underline{\pv}$.
The four Euler parameters $\underline{\pv}$ are subject to the normalization condition

$$
  \phi(\underline{\pv}) \equiv p_s^2+p_x^2+p_y^2+p_z^2-1=0 \quad \mathrm{or} \quad  p_s^2 +\pv\tp \, \pv -1 = 0 \, .
$$
First, the following trigonometric transformations are introduced,

$$
  \cos \varphi = 2 \cos^2\frac{\varphi}{2} -1, \quad \sin \varphi = 2 \sin \frac \varphi 2 \cos \frac \varphi 2, \quad 1-\cos \varphi = 2 \sin^2 \frac{\varphi}{2} \, .
$$
When these are substituted into the rotation tensor using {eq}`eq-theory-rotations-eulerparametersdef`, it follows

$$
  \Rot(\underline{\pv})=(2 p_{\mathrm{s}}^2-1) \, \Em + 2 p_{\mathrm{s}} \, \tilde{\pv} + 2 \, \pv \, \pv\tp \, .
$$
Euler parameters can initially be expressed in the global reference frame $\mathcal{F}_0$,

$$
  \LU{0}{\underline{\pv}}=\vp{p_{\mathrm{s}}}{\LU{0}{\pv}}=\vp{\cos \frac\varphi 2}{\LU{0}{\uv} \, \sin \frac{\varphi} {2}} \, .
$$
Thus, the coordinate representation of the rotation tensor is given by

$$
  \Rot(\LU{0}{\underline{\pv}}) = 2 \LU{0}{\mr{\ps^2+p_x^2-\frac 1 2}{p_x p_y-\ps p_z}{p_x p_z+\ps p_y}
                                {p_x p_y+\ps p_z}{\ps^2+p_y^2-\frac 1 2}{p_y p_z-\ps p_x}
                                {p_x p_z-\ps p_y}{p_y p_z+\ps p_x}{\ps^2+p_z^2-\frac{1}{2} }} = \LU{01}{\Rot}
$$ (eq-theory-rotations-eulerparametersrot)

Note that the axis of rotation is invariant to the rotation, $\LU{0}{\uv} = \LU{1}{\uv}$, thus $\LU{0}{\underline{\pv}} = \LU{1}{\underline{\pv}}$
Due to the ambiguity $\Rot(\underline{\pv}) = \Rot(-\underline{\pv})$, the scalar part of the Euler parameters is usually normalized, i.e., $p_s \ge 0$.

 The careful reader may observe that matrix $\Rot$ in {eq}`eq-theory-rotations-eulerparametersrot` is not identical to the implementation in Exudyn.
This is true, because there is always an alternative formulation for the diagonal terms by adding the normalization condition times a factor.

### Quaternion operations

Calculations with Euler parameters are efficiently performed through the rules of quaternion algebra.
Active perspective as vector rotation: The rotation of vector $\rv_0$ into $\rv(t)$,

$$
  \LU{0}{\rv}(t)=\LU{0}{\Rot}(t) \, \LUR{0}{\rv}{0} \quad \text{mit} \quad \LU{0}{\Rot}(t)= \Rot(\LU{0}{\uv}(t), \varphi (t))
$$ (eq-theory-rotations-quaternionrotation)

is formulated using the quaternion $\LU{0}{\underline{\pv}}=\underline{\pv}(\LU{0}{\uv}, \varphi)$ and its conjugate $\myoverline{\underline{\pv}}$ with the help of the double quaternion product (operator $\circ$),

$$
  \LU{0}{\underline{\rv}}(t) = \LU{0}{\underline{\pv}}(t) \, \circ \, \LUR{0}{\underline{\rv}}{0} \, \circ \, \LU{0}{\underline{\myoverline{\pv}}}(t) \, ,
$$
or in short form,

$$
\vp{0}{\LU{0}{\rv}(t)}=\vp{p_{\mathrm{s}}(t)}{\LU{0}{\pv}(t)} \, \circ \, \vp{0}{\LUR{0}{\rv}{0}} \, \circ \, \vp{p_{\mathrm{s}}(t)}{-\LU{0}{\pv}(t)} \, .
$$
With the multiplication rule for quaternions, the scalar part yields $0=0$ and the vector part the vector rotation {eq}`eq-theory-rotations-quaternionrotation`.
For multiple rotations, we have

$$
  \LUR{0}{\underline{\rv}}{2}=\LUR{0}{\underline{\pv}}{2} \circ \LUR{0}{\underline{\pv}}{1} \circ \LUR{0}{\underline{\rv}}{0} \circ  \LUR{0}{\myoverline{\underline{\pv}}}{1} \circ \LUR{0}{\myoverline{\underline{\pv}}}{2} \, .
$$
Given two quaternions

$$
  \underline{\av} = a_\mathrm{s} + \mathrm{i}\, a_x + \mathrm{j}\, a_y + \mathrm{k}\, a_z, \quad \mathrm{and} \quad \underline{\bv} = b_\mathrm{s} + \mathrm{i}\, b_x + \mathrm{j}\, b_y + \mathrm{k}\, b_z \, ,
$$
we provide two important **rules of quaternion algebra**, which is the sum,

$$
  \underline{\cv} = \underline{\av} + \underline{\bv} \quad \ra \quad
  \vp{c_\mathrm{s}}{\cv} = \vp{a_\mathrm{s}}{\av} + \vp{b_\mathrm{s}}{\bv} = \vp{a_\mathrm{s} + b_\mathrm{s}}{\av + \bv} \, ,
$$
and the multiplication

$$
  \underline{\cv} = \underline{\av} \circ \underline{\bv} \neq \underline{\bv} \circ \underline{\av} \quad \ra \quad
  \vp{c_\mathrm{s}}{\cv} = \vp{a_\mathrm{s}}{\av} \circ \vp{b_\mathrm{s}}{\bv} =
   \vp{a_\mathrm{s} b_\mathrm{s} - \av^\mathrm{T} \bv}{a_\mathrm{s}\bv + b_\mathrm{s}\av + \tilde \av \bv} \, .
$$
### Angular velocity vector

To derive the angular velocity from Euler parameters, there are both the possibilities of deriving the rotation matrix with respect to time and directly deriving the angular velocity from Euler parameters.

 For an infinitesimal rotation $\dd \psi$, the quaternion follows

$$
  \underline{\pv}(\ev, \dd \psi) = \vp{\cos \frac{\dd \psi}{2}}{\ev \sin \frac{\dd\psi}{2}} \approx
    \vp{1}{\ev \frac{\dd\psi}{2}}
$$
from which we conclude that from $\underline{\pv}(\ev, \dd \psi) \, \circ \, \underline{\pv}(t) = \underline{\pv}(t) + \dd\underline{\pv}$ and thus $\dd\underline{\pv} = \vp{0}{\ev \frac{\dd\psi}{2}}$.
Dividing by $\dd t$ and with the angular velocity $\tomega_{10} = \ev \, \dot \psi$, it follows

$$
  \vp{\dot \ps}{\LLdot{0}{\pv}{}} = \frac{1}{2} \vp{0}{\tomega_{10} }  \circ \vp{\ps}{\pv}
$$
or in short form

$$
  \LU{0}{\underline{\dot \pv}} = \frac{1}{2} \underline{\tomega}_{10}  \circ \underline{\pv}
$$
With the quaternion product in matrix notation, it follows

$$
  \vp{\dot \ps}{\LU{0}{\dot\pv}{}} = \frac{1}{2} \mp{\ps}{-\pv^\mathrm{T}}{\pv}{\ps \Em - \tilde \pv} \vp{0}{\tomega_{10} }
$$
or with the matrix $\underline{\Pm}$ it can be written as

$$
  \LU{0}{\underline{\dot \pv}}{} = \frac{1}{2} {\underline{\Pm}}(\pv) \underline{\tomega}_{10}
$$
This form can be represented in global coordinates due to ${\underline{\Pm}}^{-1} = {\underline{\Pm}}\tp$ as

$$
  \vp{0}{\LU{0}{\tomega}_{10} } = 2 \mp{\ps}{\LU{0}{\pv}^\mathrm{T}}{-\LU{0}{\pv}}{\ps \Em + \LU{0}{\tilde \pv}} \, \vp{\dot \ps}{\LLdot{0}{\pv}{}}
$$
Thus, the velocity transformation follows to

$$
  \LUR{0}{\Gm}{EP} = \left[-2\LU{0}{\pv}, \; 2\ps \Em + 2\LU{0}{\tilde \pv}\right]
$$
as a matrix with 3 rows and 4 columns. With $\LUR{0}{\Gm}{EP}$, the angular velocity vector can be conveniently calculated,

$$
  \LU{0}{\tomega}_{10} = \left[-2\LU{0}{\pv}, \; 2\ps \Em + 2\LU{0}{\tilde \pv}\right] \, \LU{0}{\underline{\dot \pv}}
$$
 In Exudyn, there are the following **functions and items related to Euler parameters**:

- `NodeRigidBodyEP`: A 3D rigid body node based on Euler parameters
- `EulerParameters2RotationMatrix`: computes the rotation matrix from given Euler parameters
- `RotationMatrix2EulerParameters`: computes Euler parameters from a given rotation matrix
- `EulerParameters2G`: computes the matrix $\LUR{0}{\Gm}{EP}$ relating time derivatives of Euler parameters to the (global) angular velocity vector
- `EulerParameters2Glocal`: computes the matrix $\LUR{b}{\Gm}{EP}$ relating time derivatives of Euler parameters to the (local, body-fixed) angular velocity vector
