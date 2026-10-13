# The symmetric basis of two slope vectors: concept

The frame of a node with two slope vectors - NodePointSlope12 now, and the other 3D nodes with two slopes - as an
average of both slopes, with its rotation Jacobian and the derivative the implicit solvers need (#2228, revision2026c
step RG11.8.8). The Python reference and its checks are [symmetricBasis.py](symmetricBasis.py), next to this document;
the functions move to `exudyn.rigidBodyUtilities` and, with the derivative, to C++ when the concept is decided.

## 1. What the node computes now

`CNodePointSlope12` (src/ImplNodes/CNodePointSlope12.cpp) has the slope vectors $\av = \rv_{,x}$, $\bv = \rv_{,y}$:

| quantity | computed as |
|---|---|
| rotation matrix | Gram-Schmidt, `EXUmath::OrthogonalBasisFromVectorsXY`: $\ev_1$ along $\av$, $\ev_2$ from $\bv$ |
| angular velocity, rotation Jacobian | least squares: $\tomega$ with $\tomega \times \av \approx \dot\av$, $\tomega \times \bv \approx \dot\bv$ |
| `GetRotationJacobianTTimesVector_q` | not implemented |

The two definitions agree for a rigid motion of the slopes, but not when the slopes deform: in the script the
least-squares $\tomega$ differs from the angular velocity of the Gram-Schmidt frame by up to 130 % (random slopes and
rates). A rigid marker on such a node therefore has a rotation and an angular velocity that do not belong to each
other, and an implicit solver gets no derivative of its rotation Jacobian. The Gram-Schmidt frame also prefers
$\av$: it differs from the symmetric frame by up to 42 degrees in the samples.

## 2. The symmetric basis

With the unit vectors $\hat\av = \av/|\av|$, $\hat\bv = \bv/|\bv|$ and their sum and difference
$\uv = \hat\av + \hat\bv$, $\vv = \hat\av - \hat\bv$, which are orthogonal because $|\hat\av| = |\hat\bv|$:

$$
\ev_1 = \frac{1}{\sqrt 2}\left(\frac{\uv}{|\uv|} + \frac{\vv}{|\vv|}\right), \quad
\ev_2 = \frac{1}{\sqrt 2}\left(\frac{\uv}{|\uv|} - \frac{\vv}{|\vv|}\right), \quad
\ev_3 = \ev_1 \times \ev_2 , \quad \Am = [\ev_1,\, \ev_2,\, \ev_3]
$$

$\ev_1$ and $\ev_2$ lie in the plane of the slopes, at 45 degrees to their bisector. Checked in the script, to
round-off over 1000 random pairs:
- orthonormal and right-handed; symmetric: swapping $\av$, $\bv$ swaps $\ev_1$, $\ev_2$;
- independent of the lengths of $\av$ and $\bv$, so stretching the plate does not rotate the frame;
- for orthogonal slopes it is $[\hat\av, \hat\bv, \nv]$, the frame of the undeformed plate;
- it is the rotation of the polar decomposition of $[\hat\av, \hat\bv, \nv]$ - the frame closest to both slopes -
  computed without an SVD.

Cost: 4 square roots ($|\av|$, $|\bv|$, $|\uv|$, $|\vv|$, the last two also as $\sqrt{2 \pm 2\,\hat\av\cdot\hat\bv}$)
and one cross product - about the cost of Gram-Schmidt. It is defined for non-parallel slopes; parallel slopes are a
degenerate plate in any case.

## 3. The rotation Jacobian

The global angular velocity is $\tomega = \Gm\,[\dot\av;\,\dot\bv]$ with $\Gm \in \Rcal^{3 \times 6}$. From
$\dot\ev_i = \tomega \times \ev_i$,

$$
\tomega = (\ev_3\cdot\dot\ev_2)\,\ev_1 - (\ev_3\cdot\dot\ev_1)\,\ev_2 - (\ev_1\cdot\dot\ev_2)\,\ev_3 ,
$$

and with $\mathrm{d}(\xv/|\xv|) = (\Im - \hat\xv\hat\xv\tp)\,\mathrm{d}\xv/|\xv|$ and $\ev_3\cdot\hat\av =
\ev_3\cdot\hat\bv = \ev_3\cdot\hat\uv = \ev_3\cdot\hat\vv = 0$, $\ev_1\cdot\hat\uv = \ev_1\cdot\hat\vv = 1/\sqrt 2$,
each of the three rows reduces to dot products with $\ev_3$, $\hat\uv$, $\hat\vv$ and the projections of $\hat\uv$,
$\hat\vv$ onto the planes normal to $\hat\av$ and $\hat\bv$ - no 3 x 3 matrix is formed
(`SymmetricBasisRotationJacobian` in the script). Checked against the numerical time derivative of $\Am$, and for a
rigid motion of the slopes it gives the rigid angular velocity, as the least-squares fit and the Gram-Schmidt frame do.
For the node, $\Gm$ is the rotation Jacobian on its slope coordinates (columns 4-9; zero on the position), the local
one is $\Am\tp\Gm$, and `GetAngularVelocity` becomes $\Gm\,\dot\qv$ - consistent with `GetRotationMatrix`.

## 4. The derivative for the implicit solvers

A rigid marker or a torque on the node needs $\partial(\Gm\tp\mv)/\partial\qv$ (6 x 6 on the slopes) -
`GetRotationJacobianTTimesVector_q`. Proposal: write the basis and $\Gm$ once in C++, templated on the scalar type as
`CNodePointSlope12FRot` already is, and get the derivative by `EXUmath::AutoDiff` with 6 directions. An analytic
derivative is possible along the same lines as section 3, but long; the AD version is the reference it would be
checked against, and it is only worth writing if a measurement shows the AD version as a cost.

## 5. Steps, when decided

1. The functions `SymmetricBasis` and `SymmetricBasisRotationJacobian` in C++ (RigidBodyMath, templated), bound to
   `exudyn.cUtilities` and called by `exudyn.rigidBodyUtilities`; the script becomes their test.
2. NodePointSlope12: rotation matrix, angular velocity and rotation Jacobian from them, the derivative by AD; a test
   model with a rigid marker on a deforming plate (consistency of rotation and angular velocity, Newton convergence).
   This changes the frame of the node, and so the results of models that use it - the references are recorded again.
3. The same for NodePointSlope23 and, when it exists, for the other nodes with two slopes.
