# Rotation utilities for symbolic user items: analysis and recommendation

The analysis of revision2026c step RG12.47.1 (#3069, maintainer 2026-10-23): a module of rotation and kinematics
functions with the names and arguments of `exudyn.cUtilities` / `exudyn.rigidBodyUtilities` plus a last argument
`functions`, so that a user function written once records as a symbolic function (`functions=exudyn.symbolic`) and
computes with numbers (`functions=exudyn.cMath`, `numpy` or `math`). Measured on 1.13.17x, Windows, Python 3.13.

## 1. What exists

- `exudyn.cUtilities` (C++, numbers only): 30 functions - Skew, Skew2Vec, the conversions of Euler parameters, Tait-Bryan
  angles (RotXYZ), rotation vectors and rotation matrices in both directions, the G matrices and their time derivatives,
  RotationMatrixX/Y/Z, RotationMatrix2D, ExpSO3/LogSO3, orthonormal bases. It does not take symbolic values.
- `exudyn.rigidBodyUtilities` (Python): wraps cUtilities; adds RotationVector2G/GLocal (of the Lie group node:
  G = A, GLocal = I), homogeneous transformations (HT..., T66...), Inertia6D2InertiaTensor and the inertia classes.
- The pattern exists already in userRigidBodyTest.py: `GLocal(functions, rot)`, `GLocal_t`, `RotationMatrix`,
  `MatVec`, `Cross` with `functions` as FIRST argument, written once for `cMath` and `exudyn.symbolic`; checked against
  cUtilities to 6e-17 and recorded as symbolic user functions.

## 2. The math names the four modules share

| name | exudyn.symbolic | exudyn.cMath | math | numpy |
|---|---|---|---|---|
| sin cos tan asin acos atan atan2 sqrt exp log pow sinh cosh tanh | yes | yes | yes | yes |
| abs, sign, min, max | yes | no | no (builtins) | yes |
| pi | **no** | yes | yes | yes |
| floor, round, mod | yes | no | partly | yes |
| cross, dot, norm | no (norm in cMath only) | norm | no | cross, dot |
| IfThenElse | yes | no | no | no |

So the functions can use **sin, cos, tan, asin, acos, atan, atan2, sqrt, exp, log, pow** of `functions` and nothing else;
`pi`, `abs`, cross products and branches need a helper of the module.

## 3. What works and what fails, measured

| case | result |
|---|---|
| a function with `functions=math / cMath / numpy` against cUtilities (RotXYZ2RotationMatrix) | equal (0.0) |
| the same with `functions=exudyn.symbolic`, recorded and evaluated | equal (0.0) |
| `cMath.sin(symbolic.Real)`, `math.sin(symbolic.Real)` | **TypeError** - they take numbers only |
| `numpy.sin(symbolic.Real)` | records `sin(a)`: numpy's functions take symbolic values too; a matrix becomes a numpy array of symbolic Reals, which @ multiplies and the recording converts (#3015, #3020) |
| slicing a symbolic vector `q[3:6]` | a symbolic vector; it adds, subtracts, scales, and `@` multiplies it with a symbolic matrix |
| `esym.IfThenElse(a < 1e-8, esym.Real(1.), a)` | works; with a float branch, `IfThenElse(cond, 1., a)`: **TypeError** - the branches must be symbolic Reals |
| `IfThenElse(b < 1e-8, 1, sin(b)/b)` at b = 0 | 1.0 - the unused branch with 0/0 does not spoil the result |
| a Python `if` on a symbolic value | takes the value at the time of recording, so only that branch is recorded; a branch must be `IfThenElse` |
| `cUtilities.RotationVector2RotationMatrix` with a rotation vector longer than 2 pi | **wrong** (#3073): it reduces the angle by 2 pi but not the vector |
| `symbolic.Vector.cross` | **missing**; `numpy.cross` of object arrays works, or the components written out |
| matrices from lists of mixed Reals and numbers, `@`, `.T`, `m[i, j]` | work |
| cost of a numeric call, RotXYZ2RotationMatrix | cUtilities 0.63 us, math 1.2 us, numpy 2.1 us, cMath 2.4 us (per scalar call into C++) |

## 4. The selected list

Forward, smooth functions of the coordinates - what the mass matrix, the forces, positions and Jacobians of a user body
need:

| group | functions |
|---|---|
| basics | Skew(vector), Inertia6D2InertiaTensor(inertia6D), and a Cross(a, b), which exudyn.symbolic lacks |
| rotations about axes | RotationMatrixX/Y/Z(angleRad), RotationMatrix2D(angleRad) |
| Tait-Bryan angles xyz | RotXYZ2RotationMatrix(rot), RotXYZ2G(rot), RotXYZ2GLocal(rot), RotXYZ2G_t(rot, rot_t), RotXYZ2GLocal_t(rot, rot_t), AngularVelocity2RotXYZ_t(angularVelocity, rotation), RotXYZ2EulerParameters(alpha) |
| Euler parameters | EulerParameters2RotationMatrix(eulerParameters), EulerParameters2G, EulerParameters2GLocal, AngularVelocity2EulerParameters_t(angularVelocity, eulerParameters) |
| rotation vector | RotationVector2RotationMatrix(rotationVector) - the small-angle case by IfThenElse -, RotationVector2G, RotationVector2GLocal (as rigidBodyUtilities: of the Lie group node) |

**Not** in the list: the inverse conversions RotationMatrix2EulerParameters / 2RotXYZ / 2RotZYZ / 2RotationVector and
LogSO3 (their branches select by the largest component or clamp acos - a recording of all branches each time, little
use in a user item), GramSchmidt, ComputeOrthonormalBasis(Vectors), SymmetricBasis (branches on components),
ComputeRotationAxisFromRotationVector (singular at zero) and the HT/T66 functions (4 x 4 and 6 x 6 frames - possible
later, the user items need the rotation matrix and G).

## 5. Recommendation

1. **The module `exudyn.symbolicUtilities`** (python/exudyn/symbolicUtilities.py), next to `exudyn.symbolic` and
   `exudyn.cUtilities`. `exudyn.config` is the module of the runtime settings (`exudyn.config.outputDirectory`, ...), a
   file there would be a module `exudyn.config.symbolicUtilities` beside a settings object - to confirm with the maintainer.
2. **Names and arguments of cUtilities**, plus `functions=exudyn.symbolic` as last argument; each docstring names the
   cUtilities function it equals. Inside, only the shared names of section 2 of `functions`, and three private helpers
   that decide by `functions is exudyn.symbolic`:
   - `_Matrix(functions, rows)`: `exudyn.symbolic.Matrix(rows)` or `np.array(rows)`, `_Vector` likewise - so that the result
     multiplies with `@` in both cases;
   - `_IfThenElse(functions, condition, a, b)`: `exudyn.symbolic.IfThenElse` with both branches made symbolic Reals,
     or a Python conditional for numbers;
   - `_Pi(functions)`: `math.pi` as a number for every module.
   Arguments are sequences indexed by component: lists, numpy arrays, symbolic vectors or their slices.
3. **numpy as `functions`** serves both: it computes with numbers and records with symbolic components, as numpy
   arrays of symbolic Reals that the recording converts. A user function can so use `functions=np` and be the same
   for Python and symbolic - except where a branch is needed (only RotationVector2RotationMatrix in the list), which
   with `functions=np` on symbolic input raises a clear error pointing to `functions=exudyn.symbolic`.
4. **For numbers, cUtilities stays the fast way** (0.63 us against 1.2-2.4 us); the new functions with `math`, `cMath`
   or `numpy` are for checking and for code that shall serve both.
5. **Tested** in a test model `symbolicUtilitiesTest.py`: every function against cUtilities / rigidBodyUtilities at
   random arguments, with `math`, `cMath`, `numpy` and recorded with `exudyn.symbolic` (evaluated), the small-angle
   branch of the rotation vector at 0 and 1e-10, and the refusal of a branch with `functions=np` on symbolic input.
6. **The test scripts then shrink**: userRigidBodyTest.py drops its own GLocal, GLocal_t, RotationMatrix, Skew and
   Cross (about 40 lines) and calls `su.RotXYZ2GLocal(rot, functions=...)` etc. for the Python components and the
   symbolic functions; userANCFBeam3DTest.py and the examples with symbolic rigid bodies alike. Yes - with
   `functions=exudyn.symbolic` (or `numpy`) a symbolic user function is written with the same calls as a Python one.
7. **Missing in exudyn.symbolic, worth an issue each, not needed for the module**: a cross product of symbolic vectors,
   `pi`, and IfThenElse accepting a number as a branch.

## 6. Decisions (maintainer 2026-10-23) and what was built

- **The module is `exudyn.symbolicUtilities`** (python/exudyn/symbolicUtilities.py), with the 21 functions of section 4.
- **The result follows `functions`**: a symbolic matrix or vector for `exudyn.symbolic`, a numpy array of numbers for
  `numpy`, `math` and `exudyn.cMath` (the latter for reproducible bits; for numbers alone cUtilities is faster). So
  numpy serves numbers only; point 3 above, numpy recording symbolic values, is not used.
- The helpers are `_Matrix` and `_Vector`; the branch of RotationVector2RotationMatrix is chosen by `IfThenElse` when the
  angle is symbolic, by a Python `if` otherwise; `_Pi` is not needed by any of the functions.
- Tested in python/TestModels/symbolicUtilitiesTest.py: every function at 5 random arguments with numpy, math and cMath
  and recorded once with exudyn.symbolic, evaluated after SetValue, against cUtilities / rigidBodyUtilities (largest
  difference 6.7e-16), the rotation vector at length 0 and 1e-13 (difference 0).
- userRigidBodyTest.py writes its symbolic user functions with the module: its own GLocal, GLocal_t, RotationMatrix,
  Skew, Cross, MatVec, MatTVec and InertiaTensor are gone (282 to 237 lines); the force and the position, with
  `functions=cMath`, equal the Python user functions bit by bit; the result is unchanged. The other test scripts of the
  user items have no rotation functions of their own.
- Issues: #3070 (symbolic cross product), #3071 (`pi` in exudyn.symbolic), #3072 (IfThenElse with a number), and
  #3073 (cUtilities.RotationVector2RotationMatrix beyond 2 pi), found while testing.
