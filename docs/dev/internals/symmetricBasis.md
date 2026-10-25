# The symmetric basis of two slope vectors

The frame of a node with two slope vectors as an average of both slopes, with its rotation Jacobian and the derivative
the implicit solvers need (#2228). It is the templates `RigidBodyMath::SymmetricBasis` and
`RigidBodyMath::SymmetricBasisRotationJacobian` (src/Linalg/RigidBodyMath.h), bound as
`exudyn.cUtilities.SymmetricBasis` and `SymmetricBasisRotationJacobian`; the frame, angular velocity, rotation Jacobian
and the derivative of its transpose times a torque of NodePointSlope12, NodePointSlope23, ObjectANCFBeam and
ObjectANCFThinPlate are in `AccessFunctionsAD` (src/Utilities/AccessFunctionsAD.h). The Python reference of the
formulas and their checks is `tools/benchmarks/symmetricBasis.py`; cUtilitiesTest.py tests the C++ functions.

Gram-Schmidt, which the node used before, prefers the first slope - it differs from the symmetric frame by up to 42
degrees - and its least-squares angular velocity did not belong to its rotation when the slopes deform.

## 1. The symmetric basis

With the unit vectors $\hat\av = \av/|\av|$, $\hat\bv = \bv/|\bv|$ and their sum and difference
$\uv = \hat\av + \hat\bv$, $\vv = \hat\av - \hat\bv$, which are orthogonal because $|\hat\av| = |\hat\bv|$:

$$
\ev_1 = \frac{1}{\sqrt 2}\left(\frac{\uv}{|\uv|} + \frac{\vv}{|\vv|}\right), \quad
\ev_2 = \frac{1}{\sqrt 2}\left(\frac{\uv}{|\uv|} - \frac{\vv}{|\vv|}\right), \quad
\ev_3 = \ev_1 \times \ev_2 , \quad \Am = [\ev_1,\, \ev_2,\, \ev_3]
$$

$\ev_1$ and $\ev_2$ lie in the plane of the slopes, at 45 degrees to their bisector. Checked in `tools/benchmarks/symmetricBasis.py`, to
round-off over 1000 random pairs:
- orthonormal and right-handed; symmetric: swapping $\av$, $\bv$ swaps $\ev_1$, $\ev_2$;
- independent of the lengths of $\av$ and $\bv$, so stretching the plate does not rotate the frame;
- for orthogonal slopes it is $[\hat\av, \hat\bv, \nv]$, the frame of the undeformed plate;
- it is the rotation of the polar decomposition of $[\hat\av, \hat\bv, \nv]$ - the frame closest to both slopes -
  computed without an SVD.

Cost: 4 square roots ($|\av|$, $|\bv|$, $|\uv|$, $|\vv|$) against 2 of Gram-Schmidt - 1.6 times its time, see
section 4. It is defined for non-parallel slopes; parallel slopes are a degenerate plate in any case.

## 2. The rotation Jacobian

The global angular velocity is $\tomega = \Gm\,[\dot\av;\,\dot\bv]$ with $\Gm \in \Rcal^{3 \times 6}$. From
$\dot\ev_i = \tomega \times \ev_i$,

$$
\tomega = (\ev_3\cdot\dot\ev_2)\,\ev_1 - (\ev_3\cdot\dot\ev_1)\,\ev_2 - (\ev_1\cdot\dot\ev_2)\,\ev_3 ,
$$

and with $\mathrm{d}(\xv/|\xv|) = (\Im - \hat\xv\hat\xv\tp)\,\mathrm{d}\xv/|\xv|$ and $\ev_3\cdot\hat\av =
\ev_3\cdot\hat\bv = \ev_3\cdot\hat\uv = \ev_3\cdot\hat\vv = 0$, $\ev_1\cdot\hat\uv = \ev_1\cdot\hat\vv = 1/\sqrt 2$,
each of the three rows reduces to dot products with $\ev_3$, $\hat\uv$, $\hat\vv$ and the projections of $\hat\uv$,
$\hat\vv$ onto the planes normal to $\hat\av$ and $\hat\bv$ - no 3 x 3 matrix is formed
(`SymmetricBasisRotationJacobian`). Checked against the numerical time derivative of $\Am$; for a rigid motion of the slopes it gives the rigid angular
velocity.
For the node, $\Gm$ is the rotation Jacobian on its slope coordinates (columns 4-9; zero on the position), the local
one is $\Am\tp\Gm$, and `GetAngularVelocity` is $\Gm\,\dot\qv$ - consistent with `GetRotationMatrix`.

## 3. The derivative for the implicit solvers

A rigid marker or a torque on the node needs $\partial(\Gm\tp\mv)/\partial\qv$ (6 x 6 on the slopes),
`GetRotationJacobianTTimesVector_q`: the basis and $\Gm$ are written once in C++, templated on the scalar type, and the
derivative is computed by `EXUmath::AutoDiff` with 6 directions. An analytic derivative is possible along the lines of
section 2, but long; it is worth writing only if a measurement shows the AD version as a cost.

## 4. The variants, measured

Measured with `tools/benchmarks/symmetricBasisBenchmark.cpp`: MSVC 2022 `/O2
/fp:precise`, Windows x64, the machine of the cMath measurement of RG4.1.3.5 - there `sqrt` took 1.35 ns per evaluation
in a loop over independent arguments, `exp` 1.6 ns, `sin` 2.6 ns. The time per frame is the fastest of 5 runs over
1024 slope pairs - a random rotation, stretched by up to the strain and sheared by up to the angle; the deviation is the
largest entry of the difference to the exact symmetric basis:

| variant | square roots | ns per frame | deviation, strain 0.001 / shear 0.01 | 0.01 / 0.1 | 0.1 / 0.5 |
|---|---|---|---|---|---|
| Gram-Schmidt | 2 | 7.9 | 4.9e-3 | 4.9e-2 | 2.4e-1 |
| symmetric, direct ($\uv$, $\vv$ normalized) | 4 | 13.4 | 0 | 0 | 0 |
| symmetric, combined (below) | 4 | 12.5 | 8e-16 | 7e-16 | 7e-16 |
| polar rotation of $[\av, \bv]$, closed form (below) | 2, 1 division | 9.3 | 3.6e-6 | 3.5e-4 | 1.8e-2 |
| normalize, then 2 Newton-Schulz steps | 2 | 15.1 | 4e-9 | 4e-5 | 2e-2 |
| normalize, then 3 Newton-Schulz steps | 2 | 19.5 | 7e-16 | 1e-9 | 5e-4 |
| normalize, then 4 Newton-Schulz steps | 2 | 24.3 | 7e-16 | 7e-16 | 2e-7 |
| no square root: 3 Newton steps for the inverse lengths from 1, then 3 Newton-Schulz | 0 | 20.6 | 7e-16 | 1e-9 | 5e-4 |
| no square root: 4 + 4 steps | 0 | 27.3 | 7e-16 | 7e-16 | 2e-7 |

**Symmetric, combined.** With $c = \hat\av\cdot\hat\bv$, the lengths $|\uv| = \sqrt{2+2c}$ and $|\vv| = \sqrt{2-2c}$
need no vector; with $k_u = 1/|\uv|$, $k_v = 1/|\vv|$ and $p, m = (k_u \pm k_v)/\sqrt 2$, $\ev_1 = p\,\hat\av +
m\,\hat\bv$, $\ev_2 = m\,\hat\av + p\,\hat\bv$ and $\ev_3 = 2 k_u k_v\,(\hat\av \times \hat\bv)$, as $|\hat\av \times
\hat\bv| = \sin\theta = 1/(2 k_u k_v)$. Fewer operations than the direct form, but the 4 square roots stay: two for the
unit slopes, two for the half angle - $\cos(\theta/2)$ and $\sin(\theta/2)$ need one each, and computing one of them
from $\sin\theta$ needs a square root for $\sin\theta$.

**Polar rotation of $[\av, \bv]$.** The rotation $\Rm$ of $\Fm = \Rm\mathbf{U}$ for $\Fm = [\av, \bv]$ (3 x 2), in closed
form: $\Rm = \Fm\,\Sm^{-1/2}$ with the 2 x 2 matrix $\Sm = \Fm\tp\Fm$, whose square root is $(\Sm + d\,\Im)/t$ with
$d = \sqrt{\det \Sm}$, $t = \sqrt{\mathrm{tr}\,\Sm + 2d}$ - 2 square roots and one division, 1.2 times Gram-Schmidt
(`PolarRotationAB` in symmetricBasis.py). It is symmetric in $\av$ and $\bv$, orthonormal, equal to the symmetric basis for
$|\av| = |\bv|$, and it is the rotation of continuum mechanics: for slopes that are orthonormal in the reference
configuration, $[\av, \bv]$ is the in-plane deformation gradient. It is not independent of the slope lengths:
stretching one of two non-orthogonal slopes turns it by an angle of the order of strain times shear - 3.6e-6 for small
deformations, 1.8e-2 for large ones. For slopes that are not orthonormal in the reference configuration, the
deformation gradient is $[\av, \bv]\,[\av_0, \bv_0]^{-1}$, and the frame is taken relative to the reference frame.

**Iterative schemes are slower here, not faster.** The Newton-Schulz step $\Xm \leftarrow \Xm(3\Im - \Xm\tp\Xm)/2$
converges quadratically to the polar rotation - after an exact normalization to the symmetric basis - but each step
depends on the one before, and a square root costs about as much as one step in a loop over many frames (1.35 ns). For
the usual nearly orthonormal slopes 3 steps reach machine precision, large shear needs more; a fixed number of steps
either wastes time or is not converged, and a test for convergence makes the frame a non-smooth function of the
coordinates, which its derivative cannot follow. The version without any square root is no faster either. They could
pay where square roots are slow; on x86-64 and ARM64 the square root is a pipelined instruction.

**Weighed.** A frame costs 8-13 ns, a Python call of a user function 160 ns, the evaluation of an element
microseconds; the choice is decided by the definition and the derivative more than by these nanoseconds:
- the **symmetric basis, combined** (12.5 ns): independent of the slope lengths, the definition of #2228;
- the **polar rotation of $[\av, \bv]$** (9.3 ns): the rotation of the deformation gradient, close to Gram-Schmidt in
  time, but turned slightly by stretching.

Both have closed-form rotation Jacobians (section 2 for the symmetric basis; the polar rotation along the same lines)
and get the derivative of section 3 by AutoDiff. The symmetric basis, combined, was chosen (maintainer 2026-10-13).
