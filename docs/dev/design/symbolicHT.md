# Homogeneous transformations in exudyn.symbolic: analysis (revision2026c step RG12.69.1, #3174)

`exudyn.HT` is the transformation of the rigid bodies of Exudyn - frames of bodies, markers and joints, the coordinates
of the rigid body nodes, the Lie group maps. A user function written with it runs on numbers only: an `exudyn.HT`
holds 12 doubles and cannot hold an `exudyn.symbolic` expression, so the function cannot be recorded. Maintainer
2026-10-31: HT features directly available in symbolic, similar to `exu.cMath`, which maps a symbolic argument to the
function of `exudyn.symbolic` (#3114); the user functions of connectors and joints with HT0 instead of position0,
rotationMatrix0 (`symbolics.userItems` gets simpler); 6D velocities, with a way to operate with HT; the design decision
whether HT maps at once to linear algebra or gets native operations, and which of the many functions go into
symbolic. This note is the analysis; the decisions are in section 6.

## 1. What exists

| where | what | numbers or symbolic |
|---|---|---|
| `exudyn.HT` (C++, `src/Linalg/HomogeneousTransformation.h`, `PyHomogeneousTransformation.h`, #2780) | rotation A (3x3) and translation p; `H1*H2`, `H*v` (= A v + p), `H@...`, `Inverse`, `Invert`, `Relative` (H^-1 H2 without the inverse), `RotateVector`, `RotateVectorTransposed`; `Set(rotation, translation, eulerParameters, Rxyz, rotationVector)`, `SetRotationX/Y/Z/Axis`, `SetTranslation`, `SetIdentity`; `GetEP`, `GetRxyz`, `GetRotationVector`, `GetCoordinatesEP/Rxyz/RotationVector` and their `Set...`; `RotationAngle`, `RotationAxis`; `LogSE3`, `SetExpSE3`, `LogR3xSO3`, `SetExpR3xSO3`; `InterpolateSO3`, `InterpolateSE3`; `HT44`, `[]`, `HasNoRotation`, copy | numbers |
| `exudyn.rigidBodyUtilities` | the 4x4 numpy forms: `HT0`, `HTtranslate(X/Y/Z)`, `HTrotateX/Y/Z`, `HT2rotationMatrix`, `HT2translation`, `InverseHT`; the 6x6 Plücker transformations `T66...` of the former kinematic tree | numbers |
| `exudyn.cUtilities` | the rotation parameters in C++ (#2968) | numbers |
| `exudyn.symbolics.kinematics` (#3069) | `Skew`, `Cross`, `RotationMatrixX/Y/Z`, `RotXYZ2RotationMatrix`, `EulerParameters2RotationMatrix`, `RotationVector2RotationMatrix` (with the small-angle limits), the G matrices and their time derivatives; a last argument `functions` (exudyn.symbolic, numpy, math, cMath) | both, the same code |
| `exudyn.cMath` (#3114) | `sin`, `cos`, ...: an overload for `exudyn.symbolic.Real` returns the symbolic function | both, by the type of the argument |

The recording of a user function (`CreateSymbolicUserFunction`, advancedUtilities.py) makes a named
`esym.Vector` or `esym.Matrix` per argument from its C++ type (`Vector3D`, `Matrix3D`, ...); the C++ evaluation sets
their values per call. An HT argument would be two of them - a 3x3 matrix and a 3-vector - or one 12-vector.

## 2. The interfaces of the user connectors and joints now

- `ObjectConnectorUser.forceUserFunction(mbs, t, itemNumber, position0, velocity0, rotationMatrix0, angularVelocity0,
  position1, velocity1, rotationMatrix1, angularVelocity1, parameters)` (#3119): global position, velocity, rotation
  matrix (local to global) and global angular velocity of each marker; the unit matrix and zero for a marker without
  orientation.
- `ObjectJointUser.constraintUserFunction(mbs, t, itemNumber, position0, rotationMatrix0, position1, rotationMatrix1,
  parameters)` and the `jacobianUserFunction` with the derivatives by the marker translations and the global rotation
  increments (#3101, #3140).
- `exudyn.symbolics.userItems` writes the joints and the torsional spring-damper on columns of the rotation matrices
  (`_Column(rotationMatrix0, 2)`) and differences of positions, with a helper `_Functions` choosing numbers or
  symbolic.

The same frames as HT: `HT0 = [rotationMatrix0, position0]`; the relative frame of a joint is
`HT0.Relative(HT1) = [A0^T A1, A0^T (p1 - p0)]`, from which the built-in joints take their constraints.

## 3. The two ways

**A - mapped.** A symbolic HT is a pair (rotation `esym.Matrix` 3x3, translation `esym.Vector` 3); each function
produces the existing matrix and vector expressions at once: `H1*H2` gives `(A1@A2, A1@p2 + p1)`, `H*v` gives
`A@v + p`, `Inverse` gives `(A.T, -(A.T@p))`, `Relative` gives `(A1.T@A2, A1.T@(p2 - p1))`, the rotation parameters
come from `symbolics.kinematics`.

- Everything after the recording works unchanged: the table evaluation, the one-pass derivative, `esym.Jacobian`,
  `Optimize`, the export in the three formats, the count, the uniform evaluation of RG14 - none of them learns a new
  kind.
- Small: one class, mostly formulas that exist in `exudyn.HT` and `symbolics.kinematics`.
- The structure is lost in the recording: the export shows matrix products, not HT products. `A1@A2` is a general
  3x3 product (27 multiplications); the orthogonality of A is not used. What `HasNoRotation` saves in C++ - a frame
  without rotation - becomes a unit matrix of constants, which `Optimize` folds component by component only once the
  products are scalar operations (RG12.65.8 makes them codes, not components).
- The maps with branches - `GetRotationVector`, `LogSE3`, `GetEP` (four cases of the largest component),
  `RotationAxis` - become `IfThenElse` of the cases, with the small-angle limits as in
  `RotationVector2RotationMatrix`; the derivative is the one of the branch taken, as for every `IfThenElse`.

**B - native.** A fourth kind of slot, HT (12 values), with codes of its own: compose, transform a point, inverse,
relative, rotate, rotation, translation, from rotation and translation, Exp and Log; each with its rule of the
forward derivative (12 x n components) and of the derivative as nodes, rules in `Optimize` (the identity, an
inverse of an inverse), and its text in each export format.

- Compact recordings and a readable export; fixed-size kernels per code, which suit the GPU; the derivatives of the
  Lie group (by the rotation increments of the tangent space) could later come directly from the codes, as the
  `jacobianUserFunction` of the joints needs them.
- Large: the kind goes through every part of the table - evaluation, both derivatives, `esym.Jacobian`, `Optimize`,
  `GetFlatProgramData`, the numpy and sympy formats (which need an HT of their own), `GetStatistics`, and the scalar
  graph of RG14, which would expand it again into components. The present vector and matrix operations are not yet
  codes of the table (RG12.65.8); B before them would be a native kind beside non-native ones.

**Recommendation: A**, as a class with the method names of `exudyn.HT`, so that a user function written for
`exudyn.HT` takes the symbolic one unchanged (the same names are the interface, as `functions` is for
`symbolics.kinematics`). B, or a few native codes for the products, only if a measurement after RG12.65.8 shows the
HT operations dominating a recording.

**Where the class lives** (to decide):

- A1: a Python class `exudyn.symbolics.kinematics.HT(rotation, translation)`, holding `esym` or numpy values, with
  the methods of section 5; transparent, quick to write, no C++.
- A2: as A1, and `exudyn.HT(...)` returns it when it is given symbolic arguments - an overload of the constructor
  and of `Set`, as `exudyn.cMath` dispatches on `exudyn.symbolic.Real` -, so that user code names `exudyn.HT` only.
  Recommended once A1 works: it is what makes HT "like cMath".

## 4. 6D velocities

A marker has the velocity v of its origin and the angular velocity ω, both global - what `velocity0` and
`angularVelocity0` give now. As one 6D vector, the twist `V = [v, ω]`:

- **The convention**: global components, at the marker origin (not the spatial velocity at the global origin of
  Featherstone, which the `T66` functions of rigidBodyUtilities transform). It regroups what the connectors have now and
  needs no transformation to compare with a sensor.
- **The operations with HT**: the twist of a point fixed in a frame, `v_P = v + ω × (A r)`; the twist seen in the
  frame (`A^T v`, `A^T ω`); the relative twist of two markers `[A0^T (v1 - v0 - ω0 × (p1 - p0)), A0^T (ω1 - ω0)]`,
  which a joint or a bushing in the frame of marker 0 needs; the time derivative of `Relative`. These are functions of
  (HT, twist), written out with `Cross` - the 6x6 adjoint matrix is not formed.
- Naming: `twist0`, or `velocity6D0`; the docstrings say global, at the marker origin.

## 5. Which functions

| group | functions | symbolic |
|---|---|---|
| core | the constructor from rotation and translation, `rotation`, `translation`, `*` (HT, point), `@`, `Inverse`, `Relative`, `RotateVector`, `RotateVectorTransposed`, `SetIdentity`, `SetTranslation`, `SetRotationX/Y/Z`, `SetRotationAxis` (axis and angle symbolic) | yes |
| parameters | `Set(eulerParameters=, Rxyz=, rotationVector=)`, `SetCoordinatesEP/Rxyz/RotationVector`, by `symbolics.kinematics` | yes |
| parameters back | `GetRxyz` (atan2, asin), `GetRotationVector`, `GetEP` (branches), `RotationAngle` (acos of the trace: its derivative is singular at 0 and pi - better from the rotation vector), `RotationAxis` | yes, with the branches of section 3 |
| Lie group | `SetExpSE3`, `LogSE3`, `SetExpR3xSO3`, `LogR3xSO3` | yes, with the small-angle limits |
| 6D | the twist of a point, of the frame, relative (section 4) | yes, new functions on both classes |
| interpolation | `InterpolateSO3`, `InterpolateSE3` | not now: no user function needs them |
| numpy access | `HT44` (an `esym.Matrix` 4x4 is cheap), `[]`, `HasNoRotation`, `Invert` in place, copy | `HT44` yes; the others no |
| 4x4 and 6x6 functions | `HT0`, `HTtranslate`, `HTrotateX`, ... of rigidBodyUtilities; the `T66` functions | no: numpy forms of what the class does; T66 is the former kinematic tree (#3172) |

The duplication with `symbolics.kinematics` stays one-way: the rotation parameters stay there, the HT class calls
them.

## 6. The interfaces with HT

- `ObjectConnectorUser.forceUserFunction(mbs, t, itemNumber, HT0, twist0, HT1, twist1, parameters)` and
  `ObjectJointUser.constraintUserFunction(mbs, t, itemNumber, HT0, HT1, parameters)`, the `jacobianUserFunction`
  likewise; the C++ passes `exudyn.HT` and numpy 6-vectors, the recording the symbolic HT built from the named
  variables it has now (rotation and translation per marker) - the evaluation of a recording does not change. Both
  items were added after 1.13.0 (#2975, #2976) and are not released: changed without deprecation.
- A marker without orientation gives the unit rotation, as now.
- `exudyn.symbolics.userItems`: the joints by `HT0.Relative(HT1)`, the torsional spring-damper by the relative
  rotation vector, the spring-damper by `HT1.translation - HT0.translation`; `_Functions` and `_Column` go.
- Cost of the numeric path: two `exudyn.HT` objects per call instead of two numpy matrices and vectors - measure,
  as the user connectors are called per Newton iteration.

## 7. Decisions, and the steps

1. A (mapped) or B (native) - recommended A.
2. A1 (a Python class) first, A2 (exudyn.HT dispatching) afterwards - recommended.
3. The convention of the 6D velocity: global, at the marker origin - recommended; and its name.
4. The functions of section 5.
5. The new signatures of section 6.

Steps after the decisions: RG12.69.2 the class with its tests against `exudyn.HT` (numbers to 1e-15) and its
derivatives against central differences; RG12.69.3 the 6D functions; RG12.69.4 the interfaces of ObjectConnectorUser
and ObjectJointUser, the recording and `symbolics.userItems`; RG12.69.5 the dispatch of `exudyn.HT` (A2).
