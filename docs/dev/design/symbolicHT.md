# Homogeneous transformations in exudyn.symbolic (revision2026c step RG12.69, #3174)

`exudyn.HT` is the transformation of the rigid bodies of Exudyn - frames of bodies, markers and joints, the coordinates
of the rigid body nodes, the Lie group maps. A user function written with it runs on numbers only: an `exudyn.HT`
holds 12 doubles and cannot hold an `exudyn.symbolic` expression, so the function cannot be recorded. The goal: HT in
symbolic as `exu.cMath` is in symbolic (#3114); the user functions of connectors and joints take one HT and one twist
per marker instead of position, rotation matrix, velocity and angular velocity, so that `symbolics.userItems` gets
simpler. This note is the analysis (RG12.69.1) and the design after the decisions; what is still open is in section 8.

## 1. What exists

| where | what | numbers or symbolic |
|---|---|---|
| `exudyn.HT` (C++, `src/Linalg/HomogeneousTransformation.h`, `PyHomogeneousTransformation.h`, #2780) | rotation A (3x3) and translation p, the properties `rotation`, `translation`; `H1*H2`, `H*v` (= A v + p), `H@...`, `Inverse`, `Invert`, `Relative` (H^-1 H2 without the inverse), `RotateVector`, `RotateVectorTransposed`; `Get`, `Set(rotation, translation, eulerParameters, Rxyz, rotationVector)`, `SetRotationX/Y/Z/Axis`, `SetTranslation`, `SetIdentity`; `GetEP`, `GetRxyz`, `GetRotationVector`, `GetCoordinatesEP/Rxyz/RotationVector` and their `Set...`; `RotationAngle`, `RotationAxis`; `LogSE3`, `SetExpSE3`, `LogR3xSO3`, `SetExpR3xSO3`; `InterpolateSO3`, `InterpolateSE3`; `HT44`, `[]`, `HasNoRotation`, copy | numbers |
| `exudyn.rigidBodyUtilities` | the 4x4 numpy forms: `HT0`, `HTtranslate(X/Y/Z)`, `HTrotateX/Y/Z`, `HT2rotationMatrix`, `HT2translation`, `InverseHT`; the 6x6 Plücker transformations `T66...` | numbers |
| `exudyn.cUtilities` | the rotation parameters in C++ (#2968) | numbers |
| `exudyn.symbolics.kinematics` (#3069) | `Skew`, `Cross`, `RotationMatrixX/Y/Z`, `RotXYZ2RotationMatrix`, `EulerParameters2RotationMatrix`, `RotationVector2RotationMatrix` (with the small-angle limits), the G matrices and their time derivatives; a last argument `functions` (exudyn.symbolic, numpy, math, cMath) | both, the same code |
| `exudyn.cMath` (#3114) | `sin`, `cos`, ...: an overload for `exudyn.symbolic.Real` returns the symbolic function | both, by the type of the argument |

The recording of a user function (`CreateSymbolicUserFunction`, advancedUtilities.py) makes a named `esym.Vector`
or `esym.Matrix` per argument from its C++ type (`Vector3D`, `Matrix3D`, ...); the C++ evaluation sets their values
per call. Since #3176 every operation of a recording is a code of the table of operations, the vector and matrix
operations included, evaluated in one array of the program.

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

## 3. Mapped or native - decided: mapped

**A - mapped (decided).** A symbolic HT is a pair (rotation `esym.Matrix` 3x3, translation `esym.Vector` 3); each
method produces the existing matrix and vector expressions at once: `H1*H2` gives `(A1@A2, A1@p2 + p1)`, `H*v` gives
`A@v + p`, `Inverse` gives `(A.T, -(A.T@p))`, `Relative` gives `(A1.T@A2, A1.T@(p2 - p1))`, the rotation parameters
come from `symbolics.kinematics`.

- Everything after the recording works unchanged: the table evaluation, the one-pass derivative, `esym.Jacobian`,
  `Optimize`, the export in the three formats, the count, the uniform evaluation of RG14 - none of them learns a new
  kind.
- Small: one class, mostly formulas that exist in `exudyn.HT` and `symbolics.kinematics`.
- The structure is lost in the recording: the export shows matrix products, not HT products. `A1@A2` is a general
  3x3 product (27 multiplications); the orthogonality of A is not used. A constant frame without rotation is a unit
  matrix of constants, which `Optimize` folds once the products are scalar.
- The maps with branches - `GetRotationVector`, `LogSE3`, `GetEP` (four cases of the largest component),
  `RotationAxis` - become `IfThenElse` of the cases, with the small-angle limits as in
  `RotationVector2RotationMatrix`; the derivative is the one of the branch taken, as for every `IfThenElse`.

**B - native, kept as the later option.** A kind of slot HT (12 values) with codes of its own - compose, transform,
inverse, relative, Exp and Log -, each with its derivative rule, its rules in `Optimize` and its text in each export
format. Compact recordings, fixed-size kernels for the GPU, the Lie group derivatives directly from the codes; but the
kind goes through every part of the table and every export.

**A can still become B**: the user function sees only the class and its methods. If the methods later emit native
codes instead of matrix products, no user function, no signature and no C++ evaluation of the arguments changes; only
what the class records. The condition is that user code uses the methods and the two properties, not a layout of 12
numbers. B is done only if a measurement shows the HT products dominating a recording.

## 4. The carrier: how HT0 and twist0 reach a recorded function as single objects

The question: the arguments of the user functions must be one HT and one twist per marker; is a symbolic HT object in
C++ needed to carry them? No - the carrier is needed only while recording, which is Python:

1. **While recording.** `CreateSymbolicUserFunction` reads the argument types of the signature. For an argument of
   type HT it makes two named leaves, an `esym.Matrix` (3x3) and an `esym.Vector` (3) - named after the argument, for
   example `HT0_rotation` and `HT0_translation` -, and passes the user function one object
   `kinematics.HT(rotation=..., translation=...)` holding them. For a twist it makes one named `esym.Vector` of 6
   components, `twist0`, which is already a single object (`twist0[0:3]` is v, `twist0[3:6]` is ω). The user function
   computes with `HT0.Relative(HT1)`, `HT0.translation`, ...; what it returns is an ordinary esym expression of the
   leaves.
2. **The recording.** It is the expression graph of the result, whose leaves are the named matrices and vectors. No HT
   is in it: the methods of the class have already been mapped (section 3).
3. **In C++, per call.** The item has the frame of each marker as `HomogeneousTransformation` and its twist; the symbolic
   function copies the 9 + 3 numbers of the frame into the two leaves of the argument and the 6 numbers of the twist
   into its leaf, as it copies a `Matrix3D` and a `Vector3D` now, and evaluates the program. Nothing in C++ needs to know
   that the two leaves were one HT.
4. **On the numeric path** (a Python user function, not recorded) the item passes an `exudyn.HT` and a numpy
   6-vector; the user function calls the same methods on them. One user function serves both, as `functions` does for
   `symbolics.kinematics`.

A C++ class for a symbolic HT would only be needed to record in C++, or for B. A mixed product works without one: the
symbolic class takes an `exudyn.HT` in its products as constants (`HT0 * exu.HT(...)`); for the order
`exu.HT(...) * HT0`, `exudyn.HT.__mul__` and `__matmul__` return `NotImplemented` for an operand that is no HT and no
vector of numbers - now they try to convert it into a vector and raise -, so that Python calls the `__rmul__` of the
symbolic class (a small change in `PyHomogeneousTransformation.h`, in RG12.69.3).

## 5. 6D velocities: the twist (decided)

A marker has the velocity v of its origin and the angular velocity ω, both global - what `velocity0` and
`angularVelocity0` give now. As one 6D vector, the **twist** `twist0 = [v, ω]`, **global components, at the marker
origin** (not the spatial velocity at the global origin of Featherstone, which the `T66` functions transform).

The functions with HT, written out with `Cross`, without the 6x6 adjoint matrix:

- `PointVelocity(HT, twist, localPoint)`: `v + ω × (A r)`, the velocity of a point fixed in the frame;
- `LocalTwist(HT, twist)`: `[A^T v, A^T ω]`, the twist in the components of the frame;
- `RelativeTwist(HT0, twist0, HT1, twist1)`: `[A0^T (v1 - v0 - ω0 × (p1 - p0)), A0^T (ω1 - ω0)]`, the velocity of
  frame 1 relative to frame 0 in the components of frame 0 - what a bushing or a joint in the frame of marker 0 needs;
  the time derivative of `HT0.Relative(HT1)` follows from it.

They are functions of `symbolics.kinematics`, for numbers and esym alike.

## 6. Which functions

**The interface of `exudyn.HT` gets smaller** (proposed: HT was released in 1.13.6 and is little used; removed
without deprecation). These functions are written as well with what stays:

| removed | written as |
|---|---|
| `SetTranslation(p)` | `H.translation = p`, or `exu.HT(translation=p)` |
| `RotateVector(v)` | `H.rotation @ v` |
| `RotateVectorTransposed(v)` | `H.rotation.T @ v` |
| `SetIdentity()` | `exu.HT()` |
| `Get()` | `(H.rotation, H.translation)` |
| `Invert()` (in place) | `H = H.Inverse()` |

Uses in the repository: `RotateVector` and `RotateVectorTransposed` in one file each, the others in none. A frame
without rotation keeps its fast products, as `HT(translation=p)` sets the flag as `SetTranslation` did. `SetRotationX/Y/Z`
stay in `exudyn.HT` (used in seven files), but are not in the symbolic class.

**The symbolic class `symbolics.kinematics.HT`**, in two groups:

| group | functions | for |
|---|---|---|
| minimal (first) | the constructor `HT(rotation, translation)`, `rotation`, `translation`, `*` (HT, point; with an `exudyn.HT` as constant), `@`, `Inverse`, `Relative`; the twist functions of section 5 | the joints and connectors of the Create functions: `CreateSpringDamper`, `CreateCartesianSpringDamper`, `CreateRevoluteJoint`, `CreatePrismaticJoint`, `CreateSphericalJoint`, `CreateGenericJoint`, `CreateDistanceConstraint`, and with the rotation group `CreateTorsionalSpringDamper`, `CreateRigidBodySpringDamper` |
| rotation and Lie group (second) | `GetRotationVector` (the relative rotation of a torsional spring), `GetRxyz` (atan2, asin; the angles of `CreateRigidBodySpringDamper`), `LogSE3`, `SetExpSE3`, `LogR3xSO3`, `SetExpR3xSO3` (with the small-angle limits; Lie group nodes and the intrinsic RigidBodySpringDamper) | the rotational springs; later user nodes |
| not in the class | `SetRotationX/Y/Z`, `SetRotationAxis`, `SetIdentity`, `SetTranslation`, `RotateVector...` (written with the constructor and `rotation`); `InterpolateSO3/SE3` (no user function needs them); `HT44`, `[]`, `HasNoRotation`, `Invert`, copy | - |
| in `symbolics.kinematics` only | the rotation parameters and back: `RotXYZ2RotationMatrix`, `EulerParameters2RotationMatrix`, `RotationVector2RotationMatrix` exist; `GetEP` and `RotationAxis` (branches) and `RotationAngle` (acos of the trace, singular derivative at 0 and pi) only when needed | user nodes with Euler parameters, Rxyz, rotation vectors |

A constant frame of a joint - a marker frame given by an axis and an angle - is an `exudyn.HT` of numbers, built with
`SetRotationAxis` outside the recording and used in the products as constant. The duplication with
`symbolics.kinematics` stays one-way: the rotation parameters stay there, the HT class calls them.

## 7. The interfaces with HT (decided)

- `ObjectConnectorUser.forceUserFunction(mbs, t, itemNumber, HT0, twist0, HT1, twist1, parameters)` and
  `ObjectJointUser.constraintUserFunction(mbs, t, itemNumber, HT0, HT1, parameters)`, the `jacobianUserFunction`
  likewise; the C++ passes `exudyn.HT` and numpy 6-vectors, the recording the symbolic HT of section 4. Both items were
  added after 1.13.0 (#2975, #2976) and are not released: changed without deprecation.
- A marker without orientation gives the unit rotation and zero ω, as now.
- `exudyn.symbolics.userItems`: the joints by `HT0.Relative(HT1)`, the torsional spring-damper by the relative
  rotation vector, the spring-damper by `HT1.translation - HT0.translation`; `_Functions` and `_Column` go.
- Cost of the numeric path: two `exudyn.HT` objects per call instead of two numpy matrices and vectors - measured in
  the step, as the user connectors are called per Newton iteration.

## 8. Decisions, open questions and the steps

Decided (maintainer 2026-10-31): mapped (A), with B as a later internal option; the Python class first; the twist,
global, at the marker origin, named `twist0`, `twist1`; the signatures of section 7; the minimal group first.

Open:

1. The list of section 6 removed from `exudyn.HT` - all six, or which.
2. The names of the leaves of an HT argument in a recording (`HT0_rotation`, `HT0_translation`), which the export
   shows.
3. A2, `exudyn.HT(...)` returning the symbolic class for symbolic arguments: not needed for the user functions
   (section 4); proposed to leave it out unless a model needs `exu.HT(rotation=symbolicMatrix)`.
4. The names of the twist functions (`PointVelocity`, `LocalTwist`, `RelativeTwist`) and whether they are methods of
   the class as well.

Steps:

- RG12.69.2: `exudyn.HT` without the functions of section 6 (after open question 1).
- RG12.69.3: the class `symbolics.kinematics.HT`, minimal group, with its tests against `exudyn.HT` (numbers to
  1e-15) and its derivatives against central differences; the twist functions.
- RG12.69.4: the interfaces of ObjectConnectorUser and ObjectJointUser, the argument type HT and twist in the
  recording, `symbolics.userItems` on them.
- RG12.69.5: the rotation and Lie group group of section 6.
