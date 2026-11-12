# Compile-time sizes for the small linear algebra: evaluation and recommendation (revision2026c step RG5.2)

A design note of revision2026c step RG5.2. This is the evaluation and the recommendation; section 8 adds the profile of `ODE2RHS`. The
measurements are those of [linalgBenchmark.md](linalgBenchmark.md) (RG5.1, `exudyn.special.RunLinalgBenchmark`) and
the solver sub-timers of the performance history (`python/logs/performance/performanceHistory.json`, 1.13.548.dev1,
regular module). No new timing was taken for this note.

## 1. The classes as they are

| class | storage | size known at compile time | size at run time |
|---|---|---|---|
| `ConstSizeMatrixBase<T, dataSize>` (`ConstSizeMatrix.h:44-50`) | `T data[dataSize]` on the stack | the **capacity** `dataSize` only | `numberOfRows`, `numberOfColumns` (members) |
| `ConstSizeVectorBase<T, dataSize>` (`ConstSizeVector.h:44-48`) | `T data[dataSize]` | the capacity | `numberOfItems` (member) |
| `SlimVectorBase<T, dataSize>` (`SlimVector.h:51-54`) | `T data[dataSize]` | **the size** | none; `SetNumberOfItems` only checks (`:187-189`) |
| `MatrixBase<T>` / `ResizableMatrix` (`Matrix.h:59-64`) | `T* data` on the heap, virtual `AllocateMemory`, `ResizeMatrix` (`:169-214`) | nothing | rows, columns, allocation |
| `SlimArray<T, dataSize>` (`Utilities/SlimArray.h:49`) | fixed array of any T | the size | none |

The typedefs (`BasicLinalg.h:34-41`): `Matrix3D = ConstSizeMatrix<9>`, `Matrix4D = <16>`, `Matrix2D = <4>`,
`Matrix6D = <36>`, the float variants alike - each commented "will be changed to SlimMatrix<...>". `Vector3D` etc. are
`SlimVector`s and have their size at compile time. **The matrices never had**: a `Matrix3D` is a 9-element buffer that
may be 3x3, 1x9 or 0x0; a default-constructed one is 0x0 (`ConstSizeMatrix.h:59-60`), 81 declarations rely on that and
are sized by the first assignment.

How the operations are written:

- Element access `operator()(row, column)` = `data[row*numberOfColumns + column]` with `CHECKandTHROW` on both indices
  (`ConstSizeMatrix.h:469-484`): a multiplication by the run-time column count, and two checks in the regular module
  (none in the fast module).
- `+=`, `-=` (`:500-531`) loop over the run-time `numberOfRows*numberOfColumns` or over rows and columns.
- `operator*(ConstSizeMatrix, ConstSizeMatrix)` (`:583-622`): run-time triple loop over `operator()`; since #3418
  `if constexpr (dataSize == 9)` plus a run-time test for 3x3 selects an unrolled 3x3 product.
- `Matrix3D * Vector3D` (`:1037-1059`) and the 2x2 variants (`:985-1035`) are free functions on
  `ConstSizeMatrixBase<T, 9>` / `<T, 4>` with run-time size checks; the 3x3 one is unrolled (#3418).
- The generic kernels of `EXUmath` (`Matrix.h`, e.g. `MultMatrixTransposedVectorTemplate:1160-1183`,
  `MultMatrixTransposedMatrixTemplate`, `MultMatrixMatrix2SubmatrixTemplate`) take any matrix type and loop over the
  run-time sizes with pointers (#3417). They are fast for a run-time size, but no trip count is known to the compiler,
  so nothing is unrolled or kept in registers across the loop.

So the loops have compile-time trip counts only in `SlimVector` (since #3418, L4) and in the special-cased 3x3
products; everything else in `ConstSizeMatrix` runs on run-time sizes.

## 2. Where they are used, and which sizes are really fixed

Uses per directory (word counts, `src/`, 2026-11-12):

| directory | `Matrix3D` | `ConstSizeMatrix<` | of which sized by `maxRotationCoordinates` | `ConstSizeVector<` |
|---|---:|---:|---:|---:|
| ImplObjects | 263 | 142 | 79 | 261 |
| ImplNodes | 48 | 37 | 35 | 25 |
| System | 84 | 18 | 11 | 70 |
| ImplMarkers | 33 | 1 | 1 | 8 |
| Linalg (RigidBodyMath, HT, LieGroups) | 73 | 118 | 47 | 37 |

The most frequent capacities: `ConstSizeMatrix<maxRotationCoordinates * 3>` (= 12; about 180 uses under its various
spellings), `<9>` (39 + the `Matrix3D` typedef), `<12>` (39), `<GMaxSize>` (27), `<16>`, `<36>` (9 each).

Three kinds of use:

1. **Truly fixed, the size never changes**: rotation matrices, skew matrices, inertia tensors (`Matrix3D`, always 3x3),
   the 6x6 matrices of the rigid-body spring-damper and the joints (`Matrix6D`), the homogeneous transformations. Here
   the run-time size is pure overhead.
2. **Fixed per node type, chosen at run time**: the G matrices of the rigid-body nodes are 3x3 (Rxyz, rotation
   vector, Lie group) or 3x4 (Euler parameters), returned through the virtual
   `CNodeRigidBody::GetG/GetGlocal(ConstSizeMatrix<maxRotationCoordinates*nDim3D>&)` (`CNode.h:550-551`,
   `maxRotationCoordinates = 4`, `CNode.h:518`). The rigid body works on them with run-time sizes, e.g.
   `CObjectRigidBody::ComputeODE2LHS` (`CObjectRigidBody.cpp:205-300`: `Glocal`, `forces1` as
   `ConstSizeVector<maxRotationCoordinates>`, `MultMatrixTransposedVectorTemplate`). The size is known inside each node
   class; it becomes a compile-time size only if the object's code is templated on the number of rotation coordinates
   and dispatched once per call (the pattern of `CollectCurrentNodeMarkerData`, hand-written for EP and RotVecLG, #3409).
3. **Truly variable**: the marker Jacobians (`MarkerData::positionJacobian`, `rotationJacobian`, `jacobian` are
   `ResizableMatrix`, `MarkerData.h:37-43`), 3 x n with n = 6, 7, 12, 24 ... by the body; the local Jacobians of the
   connectors; the beams and ANCF elements (`ConstSizeMatrix<nODE2coordinates*...>` with sizes per element class, fixed
   per class). Kind 3 stays run-time sized except inside one element class.

## 3. What the measurements say

From `linalgBenchmark.md` §3.2, §3.3, §3.6 (ns per call, regular / fast; after L1-L6, #3417-#3419):

| operation | Exudyn now | Eigen fixed (3.3.7, MSVC) | prototype (compile-time loops) |
|---|---:|---:|---:|
| `Matrix3D * Vector3D` | 6.5 / 4.0 | 3.5 | - |
| `Matrix3D * Matrix3D` | 7.6 / 8.3 | 41 (`lazyProduct` 7) | 8.2 |
| 6x6 times vector (`ConstSizeMatrix<36>`) | 12.2 / 11.5 | 7.4 / 6.3 | - |
| 6x6 times 6x6 | 92.8 / 91.0 (not changed by L1-L6) | 29.8 / 26.3 | 42.9 |
| J^T K J, 3 x 6 | 59.6 / 55.5 | 558 | 34.8 |
| J^T K J, 3 x 12 | 131 / 192 | 101 | 114 |
| J^T K J, 3 x 24 | 333 / 329 | 301 | 412 |
| `ConstSizeVector +=`, n = 24 | 3.5 / 3.4 | (`Matrix<n,1>` 0.7-0.9 for small n) | 0.7-0.9 |

- After L1-L6 the 3x3 products are as fast as the prototype. The remaining gaps of a compile-time size are: **6x6
  products 2.2 times**, **6x6 times vector 1.7 times**, J^T K J for n = 6 **1.7 times**, short `ConstSizeVector`
  operations **3-4 times**, `Matrix3D * Vector3D` in the regular module 1.6 times (the two run-time checks).
- From n = 12 on the run-time kernels are within 15 % of the prototype, at n = 24 faster. Compile-time sizes pay for
  the smallest sizes only.
- **Eigen's fixed sizes are no option with the vendored 3.3.7 and MSVC**: 3x3 products 5 times, J^T K J at n = 6 16
  times slower than the prototype; 5.0.1 is slower still for small sizes (§3.8 of the benchmark note).

Where the time goes at the level of the solver (sub-timers, 1.13.548.dev1, regular): the rigid-body chains
(`perfLargeMassSpringChain` rigid, n = 1000-20000) spend 73-77 % in `ODE2RHS` and 12-25 % in `massMatrix`;
`perfConnectorInterface` rigid/spring 50-82 % in `ODE2RHS`; `perfAccessFunctionsAD` rigidEP/Rxyz 36-57 % `ODE2RHS`,
8-16 % `massMatrix`. These are the models whose `ODE2RHS` consists of exactly the kernels above: G matrices,
rotation matrices, cross products, marker Jacobians, J^T f. How `ODE2RHS` divides among bodies, markers and
connectors is **not measured**; this decides how much of a kernel gain reaches the solver.

The order of the sums: the test references are bitwise in the regular module (#3418 kept every sum in its order).
Compile-time loops in the same order give the same bits: MSVC does not contract a*b+c, gcc/clang are built with
contraction off (`setup.py:454-457`). A change of the order (e.g. vectorizing a sum across k) moves references.

## 4. Options

**(a) A new class of fixed rows and columns, `SlimMatrix<T, rows, columns>`** (the name the typedefs already
announce): `T data[rows*columns]`, no size members, row-major as `ConstSizeMatrix`, element access
`data[i*columns + j]` with a compile-time `columns`, operators and products as templates over the sizes (the
prototype `LinalgBenchmarkFixedMatrix`, `linalgBenchmark.h:74-111`), `NumberOfRows()`/`NumberOfColumns()` as
`constexpr` so that the `EXUmath` templates keep working with it.
- Pros: the measured 1.7-2.2 times on 6x6 and 3x6, 3-4 times on short vectors; the compiler unrolls and keeps the
  values in registers; no hidden 0x0 state; sizes checked by the compiler instead of at run time.
- Cons: the 81 default-constructed `Matrix3D`s that are sized later must be checked (a `SlimMatrix<3,3>` is always
  3x3); code that tests `NumberOfRows() == 0` on such a matrix changes meaning (`CObjectConnector.cpp:874`,
  `CObjectFFRFreducedOrder.cpp:495` - to check whether they see a `Matrix3D`); interaction with `ConstSizeMatrix` and
  `ResizableMatrix` needs conversions; mixed-type operators multiply (fixed * run-time).
- Compatibility: a converting constructor and assignment from `ConstSizeMatrixBase<T, rows*columns>` (checked in the
  regular module) and a conversion to it let both live together; the typedef `Matrix3D` is switched in one place
  (`BasicLinalg.h`), with a compile switch back to `ConstSizeMatrix<9>` for a measurement and as the fallback - the same
  pattern as the `Vector`/`ResizableVector` switch of RG5.1.2.
- Effort: the class and its tests: medium; the switch of `Matrix3D`/`Matrix6D`: medium (compiler errors show every
  place that needs a size); the G matrices (kind 2): high, per object class.

**(b) Specializations inside `ConstSizeMatrix`** for small sizes, as #3418 did for 3x3 (`if constexpr (dataSize == ...)`
plus a run-time size test). Pros: no caller changes. Cons: still a run-time test per call, the size members stay,
and every new size needs its own branch; for 6x6 and 3x6 it gives part of the gain at best. Effort low per operation.
It is the right tool for the remaining hot 3x3 cases only, which are already done.

**(c) Eigen fixed-size matrices internally.** Rejected by the measurements: slower than the prototype with Eigen 3.3.7
and 5.0.1 under MSVC for every size up to 12 (§3); also it would tie the item code to Eigen's expression templates,
which a human debugging an item in VS2022 does not want to step through.

**(d) Leave as is.** After L1-L6 the 3x3 products and the run-time kernels from n = 12 on are close to the best
measured. The remaining gain is in the 6x6 and 3x6 products and the short `ConstSizeVector`s, i.e. in the rigid-body
connectors, the joints and the G-matrix code.

Impact on the fast module: none in principle - compile-time loops are what AVX2 auto-vectorization needs; the
benchmark shows the prototype equally fast in both modules. Compile time: templates in headers, a few instantiations
(3x3, 3x4, 4x3, 6x6, 3x6, 3x7) - negligible against the item files.

## 5. Recommendation

1. **Measure first where `ODE2RHS` goes** in the rigid-body chains (`perfLargeMassSpringChain` rigid,
   `perfConnectorInterface` rigid): bodies vs. markers vs. connectors vs. load/assembly, with the global timers or a
   profiler run, both modules. This decides whether the kernels of §3 are the cost or the virtual calls and the
   `ResizableMatrix` of `MarkerData` are (then kind 3 dominates and RG5.2 gains little).
2. **Introduce `SlimMatrix<T, rows, columns>`** (option a) in `src/Linalg/SlimMatrix.h`, with the conversions to and from
   `ConstSizeMatrix`, constexpr sizes for the `EXUmath` templates, the products with `SlimVector` and with each other,
   the transposed products, `Vector2SkewMatrix`; extend `RunLinalgBenchmark` by it (it replaces the prototype there).
3. **Switch `Matrix6D` first** (22 uses; the 6x6 products are the largest measured gap, 2.2 times), then `Matrix3D`
   (652 uses) behind a compile switch to the old typedef, checked by the test suite bitwise in the regular module and by
   the references of the fast module.
4. **Then kind 2 in the rigid body** (`CObjectRigidBody::ComputeODE2LHS`, mass matrix, `ComputeJacobianODE2_ODE2`):
   templated on the number of rotation coordinates (3/4), dispatched once by the node type, as #3409 did for the
   marker data - only if step 1 shows the rigid body as a large share.
5. Leave kind 3 (marker Jacobians, beam and ANCF local matrices) run-time sized; their kernels are within 15 % from
   n = 12 on.

Expected: 1.5-2 times on the affected kernels; on the solver level a few percent up to perhaps 10-20 % in the
rigid-body chains, if step 1 confirms their share - an estimate, not measured.

## 6. Proposed sub-steps of RG5.2

| step | content | measure |
|---|---|---|
| RG5.2.1 | the profile of `ODE2RHS` and `massMatrix` in the rigid-body chains: bodies, markers, connectors | sub-timers / profiler, both modules |
| RG5.2.2 | `SlimMatrix<T, rows, columns>` with conversions, products, tests; in `RunLinalgBenchmark` | benchmark against the prototype and Eigen |
| RG5.2.4 | `Matrix6D` as `SlimMatrix<Real,6,6>` | benchmark, perf models, test suite bitwise |
| RG5.2.5 | `Matrix3D` as `SlimMatrix<Real,3,3>`, compile switch back; the 0x0 uses checked | as RG5.2.4 |
| RG5.2.6 | the rigid body templated on the rotation coordinates (only after RG5.2.1 shows the share) | perf models with rigid bodies |

## 7. Open

- The split of `ODE2RHS` in the rigid-body models: section 8.
- Whether any `Matrix3D` is used with another shape than 3x3 or as "unset" 0x0 on purpose; the compiler and the
  regular-module checks of the conversion will find them in RG5.2.5.
- gcc/clang: the benchmark tables are MSVC only (linalgBenchmark.md §7).

## 8. The profile of `ODE2RHS` (RG5.2.1, #3455)

Measured 2026-11-12 with temporary main timers around the parts of `CSystem::ComputeSystemODE2RHS` and of the
connector chain `ConnectorODE2LHSRigidMarkers` (regular module, 1 thread; the instrumentation is not committed). Each
start/stop pair costs about 45 ns, which the timers subtract only in part; the numbers below are corrected by the time of
an empty pair, so they are good to some 10 %. The fast module has no timers; its totals are 5-25 % lower
(solverBaseline.md), with the same kernels.

`perfLargeMassSpringChain`, rigid bodies with `RigidBodySpringDamper`, 1000 bodies, explicit (900 steps; the implicit run
of 450 steps divides alike):

| part of `ODE2RHS` | s | share |
|---|---:|---:|
| connector: the force (`ComputeConnectorForceRigid`) | 0.323 | 41 % |
| connector: the kinematics of both markers (`GetKinematicsRigid`) | 0.206 | 26 % |
| bodies (`CObjectRigidBody::ComputeODE2LHS`) | 0.154 | 19 % |
| connector: the projection of force and torque by the markers (`AddGeneralizedForceTorque`) | 0.070 | 9 % |
| adding the local vectors into the system vector | 0.038 | 5 % |

`ODE2RHS` is 80-85 % of the solver time of this model, the mass matrix 10-19 %. In `perfConnectorInterface` (rigid,
spring) and `perfAccessFunctionsAD` the connectors take 20-50 % of the solver time and the bodies 1-8 %;
`perf3DRigidBodies` spends 23 % in `ODE2RHS` (most of its time is contact and loads).

- Every connector of the performance models goes through the connector interface (#2745); the path through
  `ComputeMarkerDataStructure` and `MarkerData` (kind 3, `ResizableMatrix`) is not taken by any of them. Its cost
  does not need to be considered for RG5.2.
- **The force** of `RigidBodySpringDamper` is `Matrix3D` work: two products with `rotationMarker0/1`, the relative
  rotation `A0^T A1`, four transposed products with vectors, `RotationMatrix2RotXYZ` (atan2, asin), and two 6x6
  matrix-vector products by `operator()` on `ConstSizeMatrix<36>`. The trigonometric functions stay; the rest is kind 1.
- **The kinematics** of a rigid marker on a rigid body: the rotation matrix from the Euler parameters, the angular
  velocity from G (kind 2) - `Matrix3D` and the G matrix of the node.
- **The bodies**: G, its transposed product with the torques, the quadratic velocity term (kind 2).

So 70-75 % of `ODE2RHS` of the rigid chains are kind 1 and kind 2 kernels on 3x3, 3x4 and 6x6 matrices. That
confirms the order of section 5: `SlimMatrix` (RG5.2.2), then `Matrix6D` (the stiffness and damping of the connector,
RG5.2.4) and `Matrix3D` (RG5.2.5); the rigid body templated on its rotation coordinates (RG5.2.6) touches the bodies and
the marker kinematics, together about 45 % of `ODE2RHS`, and stays after RG5.2.5.

## 9. `SlimMatrix` (RG5.2.2, #3456)

`SlimMatrixBase<T, rows, columns>` in `src/Linalg/SlimMatrix.h`, `SlimMatrix<rows, columns>` for `Real`: the data
`T data[rows*columns]` only, row-major; `NumberOfRows()` and `NumberOfColumns()` are `static constexpr`, so the
`EXUmath` templates take it (also as result, `SetNumberOfRowsAndColumns` only checks the size). Operations: element
access with the range checks of the regular module, `+`, `-`, `+=`, `-=`, `*=`, `/=`, the scalar products, the
matrix product of any fitting sizes, matrix times `SlimVectorBase` (also of an automatic-differentiation type) and
vector times matrix, `MultTransposedMatrix` and `MultTransposedVector` without forming the transposed,
`GetTransposed`, `TransposeYourself`, `Trace`, `GetDeterminant` and `GetInverse` (explicit up to 3x3, Gaussian
elimination of `ConstSizeMatrix` above), `SetScalarMatrix`, `SetSkewMatrix`, `SetWithDiadicProduct`,
`SetSubmatrix`, `CopyFrom`, the row and column vectors, and the output of `ConstSizeMatrix`. It converts implicitly
from a `ConstSizeMatrixBase` of at least its size (the shape checked in the regular module) and to
`ConstSizeMatrixBase<T, rows*columns>`.

Every sum runs in the order of the same operation of `ConstSizeMatrix` (start at 0, add over the inner index in
increasing order); the unit tests (`src/Tests/SlimMatrixUnitTests.h`) compare every operation bitwise with
`ConstSizeMatrix` on non-symmetric, non-integer values for 2x2, 3x3, 3x4, 4x3, 4x4 and 6x6.

`RunLinalgBenchmark` (ns per call, regular / fast, 2026-11-12, the same run for all columns):

| operation | `ConstSizeMatrix` / `ResizableMatrix` | `SlimMatrix` | prototype | Eigen fixed |
|---|---:|---:|---:|---:|
| 3x3 times vector | 6.7 / 4.1 | 4.2 / 4.1 | - | 3.6 |
| 3x3 times 3x3 | 8.0 / 8.6 | 8.5 / 7.7 | 8.4 / 8.7 | 44 |
| 6x6 times vector | 16.2 / 11.9 | 9.8 / 10.2 | - | 8.8 |
| 6x6 times 6x6 | 97.9 / 95.0 | 48.2 / 47.4 | 45.2 / 44.9 | 31 |
| J^T f, 3 x 6 | 7.8 / 8.3 | 8.1 / 8.3 | 5.2 | 6.8 |
| J^T K J, 3 x 6 | 60.1 / 58 | 24.6 / 23.3 | 35.7 / 35 | 552 |
| J^T K J, 3 x 7 | 73.1 / 70 | 30.5 / 27.1 | 45.2 / 45 | 773 |
| J^T K J, 3 x 12 | 136 / 169 | 114 / 114 | 117 / 119 | 101 |
| J^T K J, 3 x 24 | 350 / 341 | 278 / 257 | 436 / 437 | 316 |

`SlimMatrix` reaches the prototype or is faster (J^T K J by `MultTransposedMatrix` over rows of the result); the
6x6 product and J^T K J for one or two rigid bodies are 2-2.5 times faster than today, the 3x3 products equal the
unrolled ones of #3418, and in the regular module the range checks of `Matrix3D * Vector3D` are gone (1.6 times).
Eigen's fixed 6x6 product stays faster (31 ns) - its kernel vectorizes over columns, which would change the order of
the sums.

## 10. `Matrix6D` and `Matrix3D` as `SlimMatrix`: the evaluation (RG5.2.4-RG5.2.6, #3458, #3459, #3463)

`Matrix6D` is `SlimMatrix<6, 6>` (#3458). `Matrix3D` is `SlimMatrix<3, 3>` (#3459); `EXUDYN_MATRIX3D_CONSTSIZE`, defined
before `BasicLinalg.h` is included, switches it back to `ConstSizeMatrix<9>`. The number type of a `Matrix3D` in
templated code is `Matrix3DBase<T>` (for example `Matrix3DBase<TReal>` in the connectors with automatic
differentiation); it follows the switch. What the switch needed besides the typedefs:

- the 135 spellings `ConstSizeMatrixBase<T, 9>` and `ConstSizeMatrix<9>` in `src` (16 files) and 3 in the item
  definitions became `Matrix3DBase<T>` and `Matrix3D`; the float matrices of the graphics (`Matrix3DF`, `RTMatrix3D`)
  stay `ConstSizeMatrix`;
- `SlimMatrix` gained the constructor `SlimMatrixBase(bool)` (uninitialized, as `Matrix3D(false)` is used), the
  conversions from and to `std::array<std::array<T, columns>, rows>`, and an **explicit** conversion from a
  `ConstSizeMatrix` of larger capacity - implicit, it made the overloads of `EXUmath::MultMatrixVector` for
  `ConstSizeMatrix<12>` and `Matrix3D` ambiguous for the 3x3 G matrices;
- `ToPython`, `FromPython` (PyConversion.h) and `ToNumPy` (PyCUtilities.h) for `SlimMatrix`.

No default-constructed `Matrix3D` relied on being 0x0, and no test of `NumberOfRows() == 0` met a `Matrix3D`: the test
suite passes with the range checks of the regular module, and **all 309 results are bitwise those of
`ConstSizeMatrix<9>`** (and of the version before `Matrix6D`).

The performance models (`exudev perf`, 2026-11-12, one machine, `SlimMatrix` against the switch to `ConstSizeMatrix<9>`;
regular: the better of two runs each):

| | regular | fast |
|---|---:|---:|
| total of all runs | -1.4 % | -3.1 % |
| `perfLargeMassSpringChain` rigid, 1000-20000 bodies, explicit and implicit | -11 to -14 % | -10 to -12 % |
| `perfConnectorInterface` rigid / spring, explicit | -17 % / -11 % | -16 % / -4 % |
| `perfAccessFunctionsAD` rigid EP / Rxyz, implicit | -10 % / -11 % | -8 % / -9 % |
| models without `Matrix3D` in their hot code (user functions, mass-spring) | within +-3 % | within +-6 %, short runs up to 12 % |

`perf3DRigidBodies` and the contact model run on 4 or 8 threads; their times vary by up to 35 % between runs of the
same module (all their timers, also the factorization, move together), and their results by round-off between runs,
so they do not decide. No model is slower beyond this noise.

**RG5.2.6, not done**: `CObjectRigidBody::ComputeODE2LHS` and `AddForceTorque` with the G products templated on the
number of rotation coordinates (3 or 4) and unchecked pointer loops, the sums in the same order: bitwise equal, but the
rigid models moved by -6 % to +2 % in the regular module, within the noise; the node already gives G in a few
operations (#3409), and the products are a small part of the body. The change was not kept.

**Recommendation for the decision**: make `Matrix3D` as `SlimMatrix<3,3>` final - it is faster in the models that use
it, not slower elsewhere, and bitwise; keep `EXUDYN_MATRIX3D_CONSTSIZE` until the next release has run on Linux and
macOS (gcc and clang were not measured), then remove it with the `ConstSizeMatrixBase<T, 9>` special cases of
`ConstSizeMatrix.h` that only `Matrix3D` used.
