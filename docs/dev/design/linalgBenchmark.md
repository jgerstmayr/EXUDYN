# Linear algebra benchmark: results and recommendations (revision2026c step RG5.1)

A design note of open steps: the measurements of `exudyn.special.RunLinalgBenchmark` (#3410) that steer the open
sub-steps of RG5.1 and RG5.2, the causes found in the code, and the recommendations. It leaves the repository when the
steps close (WORKFLOW.md §8.4); a step that changes `src/Linalg` adds its run before and after here, **always with the
regular and the fast module**.

## 1. What matters: small systems

The linear algebra that costs time is that of small sizes: the local residuals and Jacobians of objects, markers and
connectors, evaluated per item and per step. Long vectors occur in the time integration only (`ResizableVectorParallel`,
the system vectors), the dense matrix times vector in the dense solver only. The benchmark therefore concentrates on
sizes below 500, and on the hand-over of the local Jacobians to Eigen's sparse solvers.

What the computation functions use (a survey of `src/ImplObjects`, `src/ImplMarkers`, `src/ImplNodes`, `src/System`,
`src/Main`):

| where | operations | types |
|---|---|---|
| rigid body, `ComputeODE2LHS`, mass matrix | $\Gm^T(\tomega\times\Jm\tomega)$, 3x3 and 6x6/7x7 blocks | `Matrix3D`, `ConstSizeMatrix<...>`, hand-written for `CNodeRigidBodyEP`/`RotVecLG` (`CollectCurrentNodeMarkerData`, #3409) |
| body markers, `ComputeMarkerData` | position, rotation, $\Jm_{pos}$, $\Jm_{rot}$ (3 x n) | `Matrix3D`, `ResizableMatrix`, `LinkedDataVector` |
| connectors, `ComputeODE2LHS` | $\fv_{gen} = \Jm\tp\fv$ | `MultMatrixTransposedVectorTemplate` into `ResizableVector` |
| connector Jacobians (`CObjectConnector.cpp`, #2884) | $\Jm_i\tp \frac{\partial \fv}{\partial \qv} \Jm_k$, `AutoDiff<12>` of the frame variations | `MultMatrixMatrixTemplate`, `MultMatrixTransposedMatrixTemplate` on `ResizableMatrix`, n = 6 ... 24 |
| beams, ANCF, FFRF | Gauss loops over shape function vectors and their outer products | `ConstSizeVector<n>`, `ConstSizeMatrix<n*m>` with run-time sizes |
| numerical local Jacobian (`CSystemJacobians.cpp`, `ComputeLocalJacobianODE2`) | n evaluations of the local LHS, column differences | `ResizableVector`, `ResizableMatrix` |
| assembly | `AddSubmatrix` of each local Jacobian into the triplets, `FinalizeMatrix` = `setFromTriplets` | `GeneralMatrixEigenSparse`, `SparseTripletVector` |
| dense solver of small systems | `InvertSpecial` (the inverse), then inverse times residual in every Newton step | `GeneralMatrixEXUdense`, `Matrix` |

Counted over `src/ImplObjects`: `CopyFrom` 384, `Matrix3D` 280, `LinkedDataVector` 270, `ConstSizeMatrix<` 138,
`SetNumberOfRowsAndColumns` 100, `MultMatrixVector` 83, `GetTransposed` 65, `MultMatrixTransposedVector` 51,
`MultMatrixMatrix` 24, `AddSubmatrix` 21.

## 2. How to run it

```python
import exudyn as exu
results = exu.special.RunLinalgBenchmark(minimumTime=0.05, numberOfThreads=16)
#list of dicts: group, library, operation, size, threads, nsPerCall, checkSum
```

and the same with `EXUDYN_MODULE=fast` (`exudev build --fast` first). Each operation runs in batches that double until
`minimumTime` has passed. Groups: `vector`, `small`, `jacobian`, `dense`, `assembly`, `autodiff`; libraries: `Exudyn`,
`Eigen` (the reference), `prototype` (a row-major matrix of compile-time size, `LinalgBenchmarkFixedMatrix`, with plain
loops: the evidence for what known sizes gain without changing an Exudyn class). The source is
`src/Linalg/linalgBenchmark.h`; `tools/benchmarks/avx2Benchmark.py` is the counterpart on the level of the solvers.

## 3. Results 2026-11-11

The workstation of the development: AMD Ryzen 9 9950X (16 cores, 32 threads), Windows 11, MSVC 14.44 with the options of
`setup.py`; Exudyn 1.13.546.dev1; Eigen 3.3.7 (`include/Eigen`). `RunLinalgBenchmark(0.05, 16)`. Each cell: **regular
module / fast module** (no range checks, AVX2), ns per call; `u` = us. `RVP` = `ResizableVectorParallel`.

### 3.1 Vectors

| n | `Vector +=` | `RVP +=` | Eigen `+=` | `Vector MultAdd` | `RVP MultAdd` | Eigen MultAdd | `Vector *=` | Eigen `*=` |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3 | 2.4 / 2.2 | 2.3 / 2.7 | 2.3 / 4.2 | 2.3 / 2.2 | 2.4 / 2.5 | 2.3 / 2.3 | 2.4 / 2.3 | 2.4 / 2.4 |
| 6 | 2.5 / 2.4 | 2.6 / 2.6 | 2.3 / 2.3 | 2.8 / 2.4 | 2.8 / 2.6 | 2.4 / 2.3 | 2.5 / 2.3 | 2.4 / 2.4 |
| 12 | 4.0 / 3.3 | 3.2 / 2.4 | 3.0 / 2.3 | 5.2 / 3.5 | 4.4 / 2.5 | 3.5 / 2.3 | 2.9 / 2.9 | 3.9 / 2.4 |
| 24 | 6.5 / 5.3 | 4.6 / 3.0 | 5.0 / 2.8 | 9.8 / 6.5 | 7.6 / 3.3 | 5.5 / 3.7 | 3.8 / 2.9 | 5.7 / 2.5 |
| 48 | 14.5 / 8.9 | 9.4 / 4.8 | 12.0 / 5.1 | 32.1 / 13.0 | 17.5 / 5.5 | 14.1 / 6.5 | 6.5 / 3.0 | 12.0 / 4.9 |
| 100 | 47.9 / 20.0 | 17.5 / 7.3 | 24.2 / 8.7 | 64.0 / 28.9 | 35.4 / 9.3 | 29.5 / 11.8 | 12.2 / 4.8 | 31.8 / 7.0 |
| 200 | 103 / 43.3 | 31.8 / 12.2 | 46.1 / 15.8 | 144 / 72.7 | 66.8 / 16.8 | 60.0 / 22.3 | 21.9 / 7.3 | 42.6 / 12.2 |
| 500 | 223 / 99.0 | 58.8 / 28.7 | 97.9 / 47.1 | 348 / 179 | 140 / 40.5 | 104 / 49.9 | 55.2 / 16.2 | 80.8 / 31.1 |
| 1000 | 246 / 189 | 117 / 118 | 169 / 87.7 | 409 / 362 | 283 / 80.1 | 202 / 151 | 99.4 / 30.8 | 144 / 92.7 |
| 20000 | 5503 / 3984 | 2725 / 1343 | 3380 / 1895 | 8564 / 7674 | 5821 / 1721 | 4055 / 2191 | 1885 / 787 | 2699 / 1456 |
| 200000 | 56.4u / 42.8u | 29.2u / 23.9u | 35.4u / 24.5u | 86.2u / 56.9u | 64.8u / 23.9u | 43.3u / 24.6u | 19.4u / 10.6u | 27.5u / 16.3u |

Sizes known at compile time:

| n | `ConstSizeVector +=` | Eigen `Matrix<n,1> +=` | prototype `fixed +=` |
|---:|---:|---:|---:|
| 6 | 2.5 / 2.4 | 0.9 / 0.9 | 0.8 / 0.7 |
| 12 | 4.6 / 3.7 | 4.3 / 0.9 | 2.3 / 2.2 |
| 24 | 5.6 / 5.1 | 4.6 / 3.2 | 3.4 / 2.3 |

Multithreaded, fast module, 1 / 16 threads:

| n | `RVP +=` | `RVP MultAdd` | `RVP CopyFrom` |
|---:|---:|---:|---:|
| 20000 | 1343 / 3438 | 1721 / 3296 | 1302 / 3203 |
| 200000 | 23.9u / 11.1u | 23.9u / 12.5u | 23.2u / 12.6u |

### 3.2 Small fixed-size types

| operation | Exudyn | Eigen | prototype |
|---|---:|---:|---:|
| `SlimVector<3> +` (`v = v + w*s`) | 5.5 / 5.7 | 0.8 / 0.8 | |
| `SlimVector<3>` cross product | 10.1 / 10.0 | 4.2 / 4.3 | |
| `Matrix3D * Vector3D` | 7.2 / 3.9 | 3.5 / 3.5 | |
| `Matrix3D^T * Vector3D` by `GetTransposed()` | 9.8 / 5.1 | 3.9 / 3.9 | |
| `Matrix3D^T * Vector3D` by `MultMatrixTransposedVectorTemplate` | 7.1 / 7.6 | | |
| `Matrix3D * Matrix3D` | 21.3 / 17.9 | 41.0 / 41.6 (`lazyProduct` 7) | 8.2 / 8.1 |
| 6x6 times vector (`ConstSizeMatrix<36>`) | 25.0 / 18.8 | 7.4 / 6.3 | |
| 6x6 times 6x6 | 92.8 / 91.0 | 29.8 / 26.3 | 42.9 / 43.0 |
| HT * v, HT * HT, HT inverse (rotation) | 8.4, 13.9, 8.7 / 8.2, 14.0, 8.7 | | |

### 3.3 Jacobian kernels of a connector, 3 x n Jacobian

| n | $\Jm\tp\fv$ Exudyn | Eigen dyn. / fixed | prototype | $\Jm\tp\Km\Jm$ Exudyn | Eigen dyn. / fixed | prototype |
|---:|---:|---:|---:|---:|---:|---:|
| 6 | 7.9 / 7.5 | 8.4 / 6.6 | 5.0 | 90.7 / 101 | 422 / 558 | 34.8 |
| 7 | 8.0 / 7.9 | 8.6 / 7.0 | 5.3 | 121 / 131 | 561 / 738 | 43.1 |
| 12 | 10.4 / 14.4 | 9.8 / 9.5 | 7.4 | 298 / 350 | 323 / 101 | 114 |
| 14 | 11.7 / 11.8 | 10.7 / 10.3 | 8.5 | 391 / 442 | 264 / 136 | 152 |
| 24 | 19.3 / 19.1 | 15.0 / 12.5 | 14.5 | 1054 / 1189 | 431 / 301 | 412 |

(Exudyn: regular / fast; Eigen and prototype: regular, the fast module within a few percent.) `AutoDiff<12>`: a dot
product of 3 takes 13.2 ns against 2.5 ns in `Real` - 5 times for 13 values.

### 3.4 Dense: matrix times vector, the dense solver

| n | `Matrix * Vector` | Eigen | `InvertSpecial` | Eigen `PartialPivLU` | solve: inverse * rhs | Eigen LU solve |
|---:|---:|---:|---:|---:|---:|---:|
| 6 | 23.9 / 19.3 | 14.2 / 14.4 | 164 / 148 | 327 / 359 | 23.8 / 20.8 | 65.8 / 68.1 |
| 12 | 115 / 72.8 | 24.4 / 17.2 | 701 / 703 | 945 / 973 | 106 / 72.2 | 147 / 140 |
| 24 | 518 / 366 | 61.2 / 41.1 | 2910 / 2896 | 2614 / 2946 | 518 / 376 | 322 / 295 |
| 48 | 2779 / 2260 | 193 / 120 | 12.0u / 11.5u | 9.1u / 10.6u | 2731 / 2236 | 682 / 620 |
| 100 | 14.5u / 14.3u | 783 / 528 | 54.0u / 53.0u | 48.8u / 62.8u | 14.5u / 14.2u | 1759 / 1464 |
| 200 | 72.6u / 70.7u | 2894 / 1965 | 219u / 216u | 286u / 386u | 73.4u / 71.0u | 5231 / 4200 |
| 500 | 512u / 505u | 18.9u / 14.8u | 1.54 ms / 1.35 ms | 4.79 ms / 5.57 ms | 536u / 496u | 34.4u / 21.5u |

### 3.5 Assembly: local 12 x 12 Jacobians of a chain into the sparse system matrix

| objects | `AddSubmatrix` into triplets | + `FinalizeMatrix` (`setFromTriplets`) | into a known pattern |
|---:|---:|---:|---:|
| 10 | 4135 / 1481 | 9242 / 6612 | 537 / 547 |
| 100 | 34.2u / 14.8u | 87.7u / 66.8u | 5.5u / 5.9u |
| 1000 | 350u / 159u | 1.24 ms / 1.06 ms | 55.9u / 58.3u |

Per object (fast module): 148 ns to append the 144 triplets, 510 ns of `setFromTriplets`, 55 ns to add the 144 values
into a pattern whose positions are known.

### 3.6 After the local recommendations L1-L6 (#3417, #3418, #3419)

The same machine and call, Exudyn 1.13.548.dev1; **before** and **after**, each regular / fast, ns per call. The order of
every sum is the one before: all 438 results of the test suite in the regular module are the same bits (except the one
of #3412), the fast module passes its references.

| operation | n | before | after |
|---|---:|---:|---:|
| `Vector +=` | 24 | 6.5 / 5.3 | 4.4 / 3.6 |
| `Vector +=` | 100 | 47.9 / 20.0 | 12.8 / 6.9 |
| `Vector +=` | 500 | 223 / 99.0 | 57.5 / 26.8 |
| `Vector MultAdd` | 24 | 9.8 / 6.5 | 4.7 / 3.5 |
| `Vector MultAdd` | 100 | 64.0 / 28.9 | 13.1 / 7.1 |
| `Vector MultAdd` | 500 | 348 / 179 | 57.9 / 27.0 |
| `ConstSizeVector +=` | 24 | 5.6 / 5.1 | 3.5 / 3.4 |
| `Matrix * Vector` | 12 | 115 / 72.8 | 29.5 / 29.3 |
| `Matrix * Vector` | 24 | 518 / 366 | 112 / 111 |
| `Matrix * Vector` | 100 | 14.5u / 14.3u | 1.9u / 1.9u |
| `Matrix * Vector` | 500 | 512u / 505u | 79.6u / 80.3u |
| dense solve (inverse * rhs) | 24 | 518 / 376 | 116 / 115 |
| $\Jm\tp\Km\Jm$, `ResizableMatrix` | 6 | 90.7 / 101 | 59.6 / 55.5 |
| $\Jm\tp\Km\Jm$, `ResizableMatrix` | 12 | 298 / 350 | 131 / 192 |
| $\Jm\tp\Km\Jm$, `ResizableMatrix` | 24 | 1.1u / 1.2u | 333 / 329 |
| `ConstSizeMatrix<36> * ConstSizeVector<6>` | 6 | 25.0 / 18.8 | 12.2 / 11.5 |
| `SlimVector<3> +` (`v = v + w*s`) | 3 | 5.5 / 5.7 | 2.8 / 2.7 |
| `SlimVector<3>` cross product | 3 | 10.1 / 10.0 | 4.4 / 4.3 |
| `Matrix3D * Vector3D` | 3 | 7.2 / 3.9 | 6.5 / 4.0 |
| `Matrix3D * Matrix3D` | 3 | 21.3 / 17.9 | 7.6 / 8.3 |
| `AddSubmatrix` 12 x 12 into triplets, 10 objects | 10 | 4.1u / 1.5u | 1.4u / 1.4u |
| `AddSubmatrix` 12 x 12 into triplets, 1000 objects | 1000 | 350u / 159u | 145u / 150u |
| `AddSubmatrix` + `FinalizeMatrix`, 1000 objects | 1000 | 1.24 ms / 1.06 ms | 1.04 ms / 1.08 ms |

Matrix times vector is 0.19 ns per multiply-add from n = 100 on, Eigen 0.05-0.1; `Matrix3D * Matrix3D` is as fast as
the prototype; `AddSubmatrix` is as fast in the regular module as in the fast one; `setFromTriplets` is now 86 % of the
sparse assembly. Single differences of 20-30 % in both directions between two runs of an unchanged kernel (e.g.
`ResizableVectorParallel` at n = 24-100) are the noise of this benchmark.

The performance tests (`exudev perf`, `exudev perf --fast`), total time in s:

| test | regular before | after | fast before | after |
|---|---:|---:|---:|---:|
| perfRigidPendulum.py | 4.73 | 4.59 | 2.81 | 2.65 |
| perfSpringDamperExplicit.py | 4.89 | 4.58 | 3.36 | 2.67 |
| perfSpringDamperUserFunction.py | 2.08 | 2.12 | 1.91 | 1.63 |
| perfUserFunctions.py | 6.44 | 4.98 | 5.87 | 4.11 |
| generalContactSpheresPerf.py | 10.26 | 9.68 | 9.71 | 9.00 |
| perf3DRigidBodies.py | 1.99 | 1.94 | 1.80 | 1.81 |
| perfObjectFFRFreducedOrder.py | 4.47 | 2.97 | 2.51 | 2.13 |
| perfLargeMassSpringChain.py | 11.69 | 10.63 | 10.65 | 9.57 |
| perfConnectorInterface.py | 1.35 | 1.20 | 1.22 | 1.03 |
| perfAccessFunctionsAD.py | 1.15 | 1.00 | 1.03 | 0.92 |
| **total** | **49.06** | **43.69 (-11 %)** | **40.87** | **35.51 (-13 %)** |

The results of all performance tests are unchanged.

### 3.7 The share of FinalizeMatrix in the Newton step (#3420)

`timer.factorization` of the Newton step holds `FinalizeMatrix` (`setFromTriplets`) and the factorization; the two
parts are the global timers *finalize matrix* and *factorize* (`simulationSettings.show.globalTimers`, regular
module). Three sparse models of many small objects (`EigenSparse`, generalized-$\alpha$, 100 steps of 1 ms, Exudyn
1.13.553): a chain of rigid bodies (Euler parameters) on revolute joints, a chain of rigid bodies on
RigidBodySpringDamper, a chain of mass points on spring-dampers; shares of the total solver time, full Newton (a
Jacobian in every iteration) / modified Newton:

| model | n | system size | total s | Jacobian | factorize | finalize |
|---|---:|---:|---:|---:|---:|---:|
| revolute chain | 10 | 130 | 0.012 / 0.007 | 18 % / 1 % | 53 % / 7 % | 5.1 % / 0.6 % |
| revolute chain | 100 | 1300 | 0.122 / 0.061 | 18 % / 3 % | 56 % / 9 % | 4.2 % / 0.6 % |
| revolute chain | 1000 | 13000 | 1.60 / 0.74 | 16 % / 5 % | 58 % / 18 % | 6.8 % / 2.1 % |
| spring chain | 10 | 80 | 0.032 / 0.004 | 66 % / 3 % | 22 % / 1 % | 2.7 % / 0.1 % |
| spring chain | 100 | 800 | 0.358 / 0.031 | 59 % / 3 % | 31 % / 1 % | 2.2 % / 0.1 % |
| spring chain | 1000 | 8000 | 3.66 / 0.30 | 58 % / 3 % | 31 % / 1 % | 3.5 % / 0.1 % |
| mass points | 10 | 30 | 0.0022 / 0.0013 | 23 % / 1 % | 32 % / 1 % | 4.4 % / 0.1 % |
| mass points | 100 | 300 | 0.017 / 0.011 | 32 % / 1 % | 26 % / 1 % | 3.1 % / 0.0 % |
| mass points | 1000 | 3000 | 0.160 / 0.109 | 35 % / 1 % | 21 % / 1 % | 2.0 % / 0.0 % |

`FinalizeMatrix` is 2-7 % of the solver time with full Newton and below 2 % with modified Newton (the default): a
pattern mode of the sparse Jacobian (G1) would gain at most these few percent, against an interface change of
`GeneralMatrix` with a fallback for contact and switched constraints - **not worth it now**. What costs is the
factorization (Eigen's `SparseLU`, 53-58 % of the revolute chain with full Newton) and the Jacobian of the
RigidBodySpringDamper (58-66 % of its chain: about 10 us per connector and Jacobian, against 2 us for a revolute joint).

### 3.8 Eigen 5.0.1 instead of 3.3.7 (#3426)

Eigen 5.0.1 (2025-11-11; requires C++14, MPL2 only since the LGPL parts were removed) in place of `include/Eigen`,
in a scratch copy of the repository, Exudyn 1.13.559, both modules; nothing else changed.

- **It builds unchanged**: no source change, no warning from Eigen (`EIGEN_MPL2_ONLY` is no longer needed but harmless).
- **The test suite**: 323 of 333 test models and all 108 mini examples give the same results, in both modules; 10 differ
  by round-off beyond the tolerance of 5e-14 - beams (ANCFBeamTest, geometricallyExactBeam2DquadraticTest,
  geometricallyExactBeamRightAngleFrame 3e-10 / 6e-10), bricardMechanism, ballBearingTest, ANCFgeneralContactCircle,
  sphereTriangleTest2, createSphereQuadContact2 (1e-13 to 2e-11) and the contact models createSphereQuadContact and
  createSphereTriangleContact (3e-8 / 2e-8, regular / fast). A change of Eigen moves these references.
- **The sparse factorization** (`SparseLU` with COLAMD, per factorization of the models of §3.7, full Newton, regular
  module): the same within the noise - revolute chain 31/317/4221 us (3.3.7) against 32/318/3983 us (5.0.1) for 10/100/1000
  bodies, spring chain 34/561/6866 against 35/583/6956 us, mass points 8/46/328 against 9/46/328 us. The wall time of the
  solver in both modules within -6 % ... +6 %. `setFromTriplets` (assembly, §3.5) the same.
- **Dense LU** (`PartialPivLU`, the solver type `EigenDense`): `compute` in the fast module (AVX2) 1.3-2.9 times faster
  from n = 12 on (n = 500: 5.6 ms to 1.9 ms), in the regular module 1.2-1.5 times faster up to n = 24 and 0.8-0.9 times
  from n = 100 on; `solve` 0.6-0.8 times in both.
- **Vectors** (`VectorXd`): 1.4-2.5 times faster from n = 12 on in the regular module, 1.1-2.7 in the fast module; but
  n = 3 and 6 in the fast module 0.5-0.65 times.
- **Small fixed sizes are slower**, with MSVC: `Matrix3d * Vector3d` 3.4 to 14.8 ns (regular), `Matrix3d^T * Vector3d`
  3.9 to 31-40 ns, `Matrix<6,6> * Matrix<6,1>` 7.8 to 20 ns, $\Jm\tp\fv$ with `Matrix<3,n>` 6-13 to 59-253 ns (10-20
  times, both modules); `Matrix3d * Matrix3d` faster (42 to 36 / 20 ns). Exudyn's items do not use Eigen's fixed sizes,
  so this concerns the benchmark's reference only, not the solver.
- **Known bugs** (gitlab.com/libeigen/eigen, 2026-11-11): the 20 issues fixed in 5.0.1 include an aligned AVX store into
  a 16-byte array (#2982) and an alignment assumption on macOS (#2984), both relevant to the fast module and the macOS
  wheels, both fixed in 5.0.1 - so not 5.0.0. Open and confirmed since 2025-09: `ComplexEigenSolver` eigenvectors that
  overflow (#3230; Exudyn computes eigenvalues only, in `EXUmath::PolynomialRoots`), `SparseQR` (#3231, not used), HIP
  (#3199, not used); open performance issues on partial packets on AVX2 (#3086) and on the threshold of fixed-size
  products (#3119), which match the slower small sizes above. Nothing open concerns `SparseLU`, `SimplicialLDLT`,
  `PartialPivLU`, `FullPivLU` or `setFromTriplets`.

**Result**: 5.0.1 is a drop-in replacement, but it gains nothing where Exudyn spends time (the sparse factorization and
`setFromTriplets`); it gains for the dense solver `EigenDense` in the fast module, and loses on small fixed-size
products, which Exudyn does not use. G6 is therefore not worth a change now; the next occasion is Eigen 5.1 or a need for
the dense solver of larger systems.

### 3.9 MKL PARDISO from Python (#3426)

The question: can the factorization be done by Intel MKL directly from Python, without linking MKL into the C++ (rule 6:
no heavy C++ dependency)? MKL is present in the conda environment `venvExuP313` (`Library/bin/mkl_rt.2.dll`, with `mkl-service`); PARDISO
was called by `ctypes` on the system Jacobians of the models of §3.7, taken by `GetSystemJacobian(sparse=True)` after
100 steps (real unsymmetric, `mtype = 11`, zero-based CSR, `iparm` from `pardisoinit`), against Eigen's `SparseLU` inside
Exudyn and scipy's `splu` (SuperLU, COLAMD). Microseconds per call, regular module, Eigen 3.3.7:

| model | n | system / nonzeros | SparseLU (Exudyn) | SuperLU | PARDISO 1 thread: factorize / solve | PARDISO 4 / 16 threads: factorize | PARDISO analysis | `GetSystemJacobian` |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| revolute chain | 10 | 130 / 644 | 31 | 37 | 12 / 13 | 12 / 13 | 60 | 28 |
| revolute chain | 100 | 1300 / 6674 | 317 | 337 | 75 / 59 | 30 / 63 | 827 | 48 |
| revolute chain | 1000 | 13000 / 66974 | 4221 | 3364 | **fails** | **wrong** | 11158 | 408 |
| spring chain | 10 | 80 / 748 | 34 | 28 | 12 / 12 | 12 / 12 | 54 | 29 |
| spring chain | 100 | 800 / 8170 | 561 | 453 | 94 / 52 | 95 / 95 | 663 | 54 |
| spring chain | 1000 | 8000 / 83244 | 6866 | 5280 | 2446 / 814 | 532 / 282 | 15209 | 451 |
| mass points | 10 | 30 / 116 | 8 | 17 | 8 / 10 | 9 / 9 | 30 | 26 |
| mass points | 100 | 300 / 937 | 46 | 46 | 14 / 16 | 14 / 14 | 74 | 30 |
| mass points | 1000 | 3000 / 5789 | 328 | 315 | 43 / 61 | 25 / 29 | 427 | 49 |

- **Speed**: PARDISO factorizes 2.5-7 times faster than `SparseLU` on one thread when the analysis (ordering, symbolic
  factorization) is reused, and up to 24 times with 16 threads on the largest spring chain. The analysis costs 2-3
  factorizations of `SparseLU`, so the gain needs the pattern kept between the Jacobians - which is G1, rejected in §3.7
  for the assembly alone.
- **Reliability - the decisive point**: PARDISO pivots statically (perturbing small pivots by $10^{-13}$, then iterative
  refinement), not by partial pivoting as `SparseLU` and SuperLU. On the Jacobians of index-3 constraints (zero
  diagonal of the multipliers, the scaling of generalized-$\alpha$) this fails silently: the revolute chain of 100 bodies
  with the default parallel factorization (`iparm[23] = 0`, 8 threads) perturbs 44 pivots and returns a solution with a
  relative residual of $10^3$ and error code 0; only the two-level factorization (`iparm[23] = 10`) solves it. The chain of
  1000 bodies fails with every tried setting (`iparm[23]` 0, 1, 10; perturbation $10^{-13}$, $10^{-8}$, $10^{-6}$; 20
  refinement steps; without scaling and matching): error -15 or relative residuals from $10^{-3}$ to $10^2$, while
  `SparseLU` and SuperLU solve it (SuperLU residual $10^{-7}$). The ODE models (spring chain, mass points) are solved to
  $10^{-16}$ in every setting.
- **The hand-over**: `GetSystemJacobian(sparse=True)` copies the Jacobian into a scipy CSR matrix: 26-54 us up to 1300
  unknowns, 400-450 us at 8000-13000 - as much as the factorization for small systems, which therefore cannot gain.
- **The hooks**: `MainSolverImplicitSecondOrder.SetUserFunctionNewton` replaces the Newton loop by Python, which calls
  `ComputeNewtonResidual`, `ComputeNewtonJacobian`, `GetSystemJacobian` and `GetSystemResidual`. There is **no way back**:
  the increment `newtonSolution` cannot be set from Python (`SolverLocalData` is bound, but no solver gives it out, and
  there is no setter), and `ComputeNewtonUpdate` reads it. A Python Newton would also have to repeat what the C++ Newton
  does (modified Newton, the discontinuous iteration, the step reduction, the solver statistics). So **MKL from Python
  does not work today** without a small C++ addition - a user function of the linear solve, `(J, r) -> x`, or
  `SetNewtonSolution` - and then pays the hand-over and a Python call per iteration.
- **The estimate** where it would gain: the spring chain of 1000 bodies with full Newton spends 198 x 6.9 ms = 1.36 s of
  4.5 s in `SparseLU`; PARDISO with 16 threads, the hand-over and the solve would take about 198 x 1.0 ms = 0.2 s, i.e.
  25 % of the solver time. With modified Newton (the default) the factorization is 1-18 % of the solver time (§3.7), and
  the gain is at most that.

**Result**: MKL PARDISO is fast but not reliable on the Jacobians of index-3 constraints, which are Exudyn's common case;
from Python it needs a C++ hook and loses its gain on small systems to the hand-over. Not now. If a faster sparse
solver is wanted, the candidates are (1) the reuse of the symbolic analysis of `SparseLU` between Jacobians, which
exists as `linearSolver.reuseAnalyzedPattern` (off by default; the pattern is analyzed again when the number of
nonzeros changes, and when the factorization with the old one fails). Measured, regular module, full Newton:

| model | n | factorize: new analysis / reused | wall time of the solver |
|---|---:|---:|---:|
| revolute chain | 100 | 316 / 224 us | 0.117 / 0.098 s (-16 %) |
| revolute chain | 1000 | 3926 / 2806 us | 1.51 / 1.26 s (-17 %) |
| spring chain | 100 | 547 / 549 us | 0.355 / 0.357 s |
| spring chain | 1000 | 5714 / 5664 us | 3.70 / 3.65 s |
| mass points | 100 | 47 / 32 us | 0.017 / 0.016 s |
| mass points | 1000 | 329 / 265 us | 0.159 / 0.153 s (-4 %) |

The factorization is 20-31 % shorter where the pattern stays; the spring chain gains nothing, because its number of
nonzeros changes between the Jacobians (8130 and 8170 at n = 100 in the export: the assembly into triplets leaves out
the entries that are exactly zero, `GeneralMatrixEigenSparse::AddSubmatrix`), so the analysis is repeated. This is the cheapest lever - a setting, no new dependency; whether it can be the default needs
the test suite with it and a pattern that does not depend on numerical zeros. (2) a solver with threshold partial
pivoting and threads, loaded at run time as an optional dependency - UMFPACK or KLU of SuiteSparse, which Eigen
supports in C++ but which are GPL (UMFPACK) and LGPL (KLU), or PARDISO only after its pivoting is shown to work on the
index-3 tests.

## 4. Eigen as the reference

Eigen 3.3.7 compiled by MSVC is not a reference for every small product: `Matrix3d * Matrix3d` takes 41 ns with
`operator*` and 7 ns with `lazyProduct`, and $\Jm\tp\Km\Jm$ for n = 6, 7 takes 420-740 ns on every path, also in a
standalone program with the options of `setup.py`, with and without `/arch:AVX2` (scratch check eigencheck.cpp). For the
products of small fixed sizes the **prototype** - plain loops of compile-time bounds - is the better reference: it is as
fast as or faster than Eigen up to n = 12 and within 40 % at n = 24. Eigen is the reference for the vectors, for
matrix times vector, for the dense LU solve and for products from n = 12 on.

## 5. What the results say

1. **Below 12 elements the call dominates**: every vector operation of 3 or 6 elements takes 2.2-2.8 ns, whatever the
   class or the module; Eigen's `VectorXd` the same. Only compile-time sizes go below it: `Matrix<6,1> +=` and the
   prototype 0.7-0.9 ns, 3 times faster. `ConstSizeVector<n>` carries its size at run time and is as slow as `Vector`.
2. **The range checks are not the cost of the small kernels**: the fast module gains 0-45 % on small vectors and
   `Matrix3D * Vector3D` (7.2 to 3.9 ns), nothing on `Matrix3D * Matrix3D`, `J^T K J` or the 6x6 products. It gains
   2.8 times on `AddSubmatrix` into triplets (4135 to 1481 ns for 10 objects): the checked element access of the local
   matrix and of the triplet array.
3. **The loops of a few Exudyn kernels cost 2-30 times**, in both modules:
   - `MultMatrixVectorTemplate` (`Matrix.h`) adds every term into `result[i]` - through `operator[]` and memory - instead
     of a local sum: 1.45-2 ns per multiply-add against 0.05-0.1 ns of Eigen; 8 times at n = 24, 27 times at n = 500. It is
     the solve of the dense solver in every Newton step (`GeneralMatrixEXUdense::Solve`, inverse times residual) and every
     `MultMatrixVector` of the objects (83 uses).
   - `VectorBase::MultAdd` and `VectorBase::operator+=` (`Vector.h`) read the other vector through its `operator[]` and
     an iterator; the compiler does not vectorize them: `Vector MultAdd` 0.36 ns per element against 0.08 of
     `ResizableVectorParallel`, which has its own loops, and 0.1 of Eigen; `Vector *=` (own data only) is the fastest of
     all.
   - `MultMatrixTransposedMatrixTemplate` runs the 3 rows innermost, strided, through `operator()`: $\Jm\tp\Km\Jm$ 2.6
     times the prototype at n = 6 and 12, which updates rank 1 per row.
   - `Matrix3D` (a `ConstSizeMatrix<9>` with run-time size): `Matrix3D * Matrix3D` 2.2 times the prototype; 6x6 times 6x6
     2.2 times the prototype and 3 times Eigen; `SlimVector<3>` `+` 7 times and the cross product 2.4 times Eigen's
     `Vector3d`, although its size is known - the loops over iterators and the construction from an initializer list.
4. **Assembly**: `setFromTriplets` is 77 % of the sparse assembly (fast module); adding the values into a known
   pattern is 12 times faster than triplets plus `setFromTriplets` (55 against 661 ns per object, 58 us against 1.06 ms
   for 1000 objects per Jacobian). The pattern of the system Jacobian changes only with the structure (assembly,
   contact, a switched joint).
5. **The dense solver of small systems**: `InvertSpecial` is as fast as Eigen's `PartialPivLU` up to n = 200 and 3 times
   faster at 500 (Eigen 3.3.7 / MSVC); the solve by the inverse is fast for n <= 12 and slow from 24 on, because of
   `MultMatrixVectorTemplate` (item 3).
6. **Threads**: 16 threads at the limit of 20000 elements are 2.5 times *slower* than one thread; at 200000 they gain
   2 times. The limit `ResizableVectorParallelThreadingLimit` is too low for 16 threads (RG5.1.2 decides with the merge).
7. **`AutoDiff<12>`** costs 5 times `Real` for 13 values - no gain to be had in the class itself.

## 6. Recommendations

### 6.1 Local - inside the linalg classes, no change of an interface or of a caller

Each one is measured by this benchmark before and after, and checked by the test suite (bitwise or to the tolerance
of the references).

| # | change | where | evidence (fast module) | expected |
|---|---|---|---|---|
| L1 | `MultMatrixVectorTemplate`: sum into a local, write once; pointer access | `Matrix.h` | 0.66-2 ns/term vs Eigen 0.05-0.1 | 5-20 times from n = 12; the dense solve, 83 calls in objects |
| L2 | `VectorBase::MultAdd`, `operator+=`, `operator-=`: loops over the raw pointers of both vectors, as `ResizableVectorParallel` | `Vector.h` | 0.36 vs 0.08 ns/elem | 2-4 times from n = 24 on; `Vector`, `ResizableVector`, `ConstSizeVector` at once |
| L3 | `MultMatrixTransposedMatrixTemplate` (and the Add variant): rank-1 updates row by row, `GetUnsafe`/pointers | `Matrix.h` | 298 vs 114 ns at n = 12 | 2.5 times for the connector Jacobians |
| L4 | `SlimVector<n>` operators `+ - *` and the constructor: plain index loops over `dataSize` (unrolled by the compiler) | `SlimVector.h` | 5.5 vs 0.8 ns | up to 5 times for every `Vector3D` expression |
| L5 | `Matrix3D` (`typedef ConstSizeMatrix<9>`, `BasicLinalg.h`) products: unrolled 3x3 for `Matrix3D * Matrix3D` and `Matrix3D * Vector3D`; `v * A` (= $\Am\tp\vv$, `SlimVector.h`) instead of `A.GetTransposed() * v` in the item code where it is hot | `ConstSizeMatrix.h`, `SlimVector.h` | 21 vs 8 ns; 9.8 vs 3.9 ns | 2-2.5 times |
| L6 | `GeneralMatrixEigenSparse::AddSubmatrix`: the triplets appended from the data pointer of the local matrix, without checked access | `LinearSolver.cpp` | regular 4135 vs fast 1481 ns | 2.8 times in the regular module |

L1-L3 are the ones with the clearest evidence; L4/L5 touch the most-used types and need the most careful check of the
results (the order of the sums must stay where the test references are bitwise).

L1-L3 are implemented by #3417, L4 and the products of L5 by #3418 (the replacement of `A.GetTransposed() * v` by
`v * A` in the item code, 64 places, is not done), L6 by #3419; the results in §3.6.

### 6.2 Global - changes of classes or interfaces, second

| # | change | evidence | note |
|---|---|---|---|
| G1 | keep the pattern of the sparse system Jacobian: per object the positions of its entries in the CSC arrays, computed at the first evaluation; values added directly; triplets and `setFromTriplets` only when the pattern changes | 12 times for the assembly (3.5) | the largest single gain for implicit solvers of many small objects; the solver interface (`GeneralMatrix`) gets a pattern mode; contact and switched constraints need a fallback |
| G2 | compile-time sizes for the local matrices of the items (Matrix of `<rows, columns>` with fixed loops, like the prototype) - RG5.2 | prototype 2-3 times faster than the run-time sized classes for n <= 12 | the classes and the item code change; per item, where the sizes are fixed (rigid bodies, joints, beams) |
| G3 | merge `ResizableVector` into `Vector` (RG5.1.2) | after L2 the difference is gone | only if L2 does not close it |
| G4 | the threading limit per number of threads, or 100000 | 16 threads 2.5 times slower at 20000 | RG5.1.2 |
| G5 | the dense solver by Eigen's LU from n ~ 24 on, or keep the inverse and L1 | after L1 the inverse times rhs is as fast as the LU solve | decide after L1 |
| G6 | Eigen 3.4 (or later) instead of 3.3.7 | 3.3.7/MSVC small products slow (§4) | the vendored headers; a check of the sparse solvers' results first. 5.0.1 evaluated (§3.8): builds unchanged, no gain in the sparse factorization, small fixed sizes slower - not now |
| G7 | an external sparse solver (MKL PARDISO) for the factorization | PARDISO 2.5-24 times faster when the analysis is kept (§3.9) | evaluated (§3.9): fails on index-3 Jacobians, no hook from Python - not now; first `linearSolver.reuseAnalyzedPattern` of `SparseLU`, 20-31 % of the factorization (§3.9) |

### 6.3 Tests that give the evidence for the next decisions

- The benchmark after each of L1-L6 (both modules), and the solver-level effect by `exudev perf` on the performance
  tests with small systems (rigid body chains, the dense solver) - the micro gain must show in the solver timers.
- For G1: the share of `FinalizeMatrix` in the Newton step of a sparse model of many rigid bodies (the solver timers of
  `avx2Benchmark.py`), before deciding the interface.
- For G2: the prototype applied to one item (e.g. the connector Jacobian of `RigidBodySpringDamper`) in a branch,
  timed with the chain of rotVecTiming.py (RG5.1.3).

## 7. Open

- The same tables on Linux (gcc) and macOS (clang).
- The cause of the slow small products of Eigen 3.3.7 with MSVC (§4); with 5.0.1 they are slower still (§3.8).
- The factorization: whether `linearSolver.reuseAnalyzedPattern` can be the default for the sparse solver (§3.9), with
  a pattern that keeps exact zeros, so that it does not change between the Jacobians; the test suite with it.
