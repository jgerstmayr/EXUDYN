# Sparse solvers: MKL PARDISO at run time, the threads of the factorization, the back-substitution

A design note of revision2026c step RG5.9 (#3427, #3428, #3430, #3431, #3432): what the sparse linear solver of the
implicit solvers costs, how an external solver (Intel MKL PARDISO) is used without a build dependency (built), why it
failed on some of Exudyn's Jacobians (studied), how the factorization uses threads while Exudyn's own workers sleep
(built), and how the back-substitution runs in parallel (prototype, experimental). It follows `linalgBenchmark.md` §3.7-§3.9, where the
measurements of the factorization and the first PARDISO results are, and leaves the repository when the steps close
(WORKFLOW.md §8.4).

## 1. Where the time is

The models of `linalgBenchmark.md` §3.7 (chains of rigid bodies on revolute joints and on RigidBodySpringDamper, a
chain of mass points on springs; `EigenSparse`, generalized-$\alpha$, 100 steps), Exudyn 1.13.560, regular module;
shares of the solver time, per call in brackets (scratch solveShare.py):

| model | n | Newton | factorizations | back-substitutions (`newtonIncrement`) | Jacobian | residual (RHS) |
|---|---:|---|---:|---:|---:|---:|
| revolute chain | 100 | modified | 14: 9 % (403 us) | 523: 19 % (22 us) | 3 % | 28 % |
| revolute chain | 100 | full | 199: 59 % (341 us) | 199: 4 % (23 us) | 19 % | 7 % |
| revolute chain | 1000 | modified | 31: 20 % (4.7 ms) | 491: 16 % (244 us) | 5 % | 24 % |
| revolute chain | 1000 | full | 227: 64 % (4.6 ms) | 227: 4 % (286 us) | 16 % | 6 % |
| spring chain | 1000 | modified | 1: 1 % (2.4 ms) | 266: 13 % (149 us) | 3 % | 64 % |
| spring chain | 1000 | full | 198: 34 % (6.3 ms) | 198: 1 % (240 us) | 58 % | 5 % |
| mass points | 1000 | modified | 1: 0.5 % | 210: 5 % (25 us) | 1 % | 92 % |
| mass points | 1000 | full | 100: 23 % (365 us) | 100: 2 % (26 us) | 35 % | 39 % |

With modified Newton (the default) the back-substitution costs as much as or more than the factorization (5-19 % of
the solver time); with full Newton the factorization dominates (23-64 %). A faster factorization pays for full Newton,
for systems whose Jacobian is updated often (contact, large rotations) and for constrained systems.

**The Jacobians carry numerical noise**: of the stored entries, those below $10^{-14}$ of their row's maximum are 46 %
of the revolute chain (30539 of 66974 at n = 1000, the smallest $4\cdot10^{-48}$) and 67 % of the spring chain (the
smallest $10^{-105}$) - the rotation terms that are zero up to round-off. They are part of the pattern, so they cost
fill-in in every factorization, and they come and go between Jacobians, so the pattern changes (the reason why
`reuseAnalyzedPattern` does not help the spring chain, `linalgBenchmark.md` §3.9). Condition numbers (1-norm
estimate): revolute chain $4\cdot10^3$ / $2\cdot10^7$ / $2\cdot10^{11}$ for 10 / 100 / 1000 bodies, spring chain of 1000
bodies $9\cdot10^2$, 1000 independent pendulums $10^1$.

## 2. MKL PARDISO, loaded at run time (built, #3431)

### 2.1 The idea

No MKL in the wheel and none at build time. A user who wants the solver installs MKL into the environment
(`pip install mkl`, or conda, where it often is already). Exudyn's C++ loads the MKL runtime library when the solver
type is selected and takes the functions it needs from it by name. Without MKL, selecting the solver type raises a
`ValueError` that says how to install it; nothing else changes. Eigen's `PardisoSupport` is **not** used: it includes
`mkl.h` and links `mkl_rt` at build time, which would make MKL mandatory for `import exudyn` (rule 6).

### 2.2 Loading

- `src/Linalg/PardisoMKL.h/.cpp`: `namespace PardisoLibrary` with `Load(message)`, loaded once per process under a
  mutex; Windows `LoadLibraryExW` (for a path with `LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR`, so that `mkl_core`, the threading
  layer and `libiomp5md` are found next to it), Linux and macOS `dlopen(RTLD_NOW | RTLD_LOCAL)`; setup.py links `-ldl` on
  Linux.
- Search order: the environment variable `EXUDYN_MKL_LIBRARY` (a full path), then `sys.prefix` and `sys.base_prefix` of
  the running Python (`Library/bin/mkl_rt.2.dll` on Windows, `lib/libmkl_rt.so.2` on Linux - where conda and the `mkl`
  wheel put it), then the system search path (`mkl_rt.2.dll` / `libmkl_rt.so.2`, then the names without `.2`).
- The functions, declared in Exudyn with their C signatures of the LP64 interface (32-bit integers, Exudyn's sparse
  indices are `int`): `pardisoinit`, `pardiso`, `MKL_Set_Num_Threads`, `MKL_Get_Version_String`. No MKL header.
- **Every call of MKL runs on a thread of its own** (`PardisoLibrary::RunOnThread`, a persistent thread, the caller
  waits). Called on the solver's thread, PARDISO of MKL 2025.0 on Windows changed values that the calling function kept
  in registers across the call: the Newton iteration lost its initial residual (it became a denormal number or zero),
  converged badly or not at all - 75 instead of 40 Newton iterations on a chain of 10 mass points, 1103 instead of 198 on
  the chain of 1000 bodies -, while every solve had a residual of $10^{-16}$. Neither its integer arguments (checked
  with guards) nor the floating-point control words changed; the sequential threading layer did the same, the TBB
  layer, which computes on other threads, did not. On a thread of its own the results equal those of `EigenSparse`.
- Python: `exu.special.solver.PardisoInfo()` returns `{'available', 'library', 'version', 'message'}`.

### 2.3 The solver

- **Type**: `LinearSolverType.PardisoMKL`, sparse as `EigenSparse` (the same `GeneralMatrixEigenSparse` assembly,
  triplets and `setFromTriplets`); only the factorization and the solve differ: `GeneralMatrixEigenSparse` holds a
  `PardisoSolver` (one PARDISO handle) when the type is selected (`UsePardiso`, set in `SolverLocalData::SetLinearSolverType`).
- **Matrix**: Eigen's compressed column-major matrix is the compressed row storage of $\Am\tp$, so PARDISO solves with
  the transposed (`iparm[11] = 2`) and gets $\Am\xv = \bv$; zero-based (`iparm[34] = 1`); `mtype = 11` (real
  unsymmetric).
- **The pattern of the analysis is the union of the patterns seen.** Exudyn stores no entry that is exactly zero, and
  such entries come and go between Jacobians (bodies leave a symmetric start, §1); a matrix within the analyzed pattern
  is factorized with zeros in its missing entries, one outside it is analyzed again with the union. The union grows for
  a while: 66 analyses in the first 100 steps of the spring chain.
- **Phases**: analysis (11) when the pattern is not within the analyzed one,
  numerical factorization (22) in `FactorizeNew`, solve (33) in `Solve` (with a copy of the right-hand side, which may be
  the solution vector), release (-1) with the matrix.
- **The check** (the maintainer's requirement that the factorization is shown useful): after every
  `pardisoCheckPeriod`-th factorization of an analysis (the first always) the solve is followed by the relative residual
  $\lVert\Am\xv - \bv\rVert / \lVert\bv\rVert$ with Eigen's matrix (one sparse product). Above
  `pardisoResidualTolerance`, NaN, or an error code of PARDISO, the step is repeated with `SparseLU`, a warning names
  the cause, and the matrix stays with `SparseLU` until the solver is initialized again; `pardisoFallbacks` counts.
- **Settings**, in `exudyn.special.solver` while the type is experimental (the maintainer: a shorter way than
  simulationSettings during testing); they move to `simulationSettings.linearSolver` when it is recommended:

| setting | default | meaning |
|---|---|---|
| `pardisoNumberOfThreads` | 0 | threads of MKL; 0 = `parallel.numberOfThreads` |
| `pardisoCheckPeriod` | 1 | the residual is checked after every n-th factorization of an analysis (the first always); 0 = never |
| `pardisoResidualTolerance` | 1e-6 | relative residual above which the step falls back to `SparseLU`; the wrong solves of the study had $10^{-2}$ and more, the extreme chain has $10^{-7}$ with any solver (§3) |
| `pardisoFallback` | True | False: an error instead of the fallback |
| `pardisoParameters` | `{}` | expert: `{index: value}` written into `iparm` after `pardisoinit` and Exudyn's defaults `iparm[1] = 0` (minimum degree), `iparm[23] = 0` (one-level factorization) (§3) |
| `pardisoFallbacks` | 0 | the number of fallbacks since the import |

- **Test**: `pardisoSolverTest.py` - an ODE and an index-3 chain against `EigenSparse`, the fallback forced by a tolerance
  of 0, `pardisoFallback = False`, and without MKL the error; the same test result with and without MKL.

### 2.4 What it gains (measured)

Exudyn 1.13.563, regular module, generalized-$\alpha$, 100 steps of 1 ms (scratch pardisoRun.py); wall time, CPU time
in brackets for 8 threads:

| model | Newton | EigenSparse 1 thread | EigenSparse 8 threads | PardisoMKL 1 thread | PardisoMKL 8 threads |
|---|---|---:|---:|---:|---:|
| spring chain 1000 | full | 3.50 s | 2.14 s (17 s) | 3.73 s | 2.15 s (32 s) |
| spring chain 1000 | modified | 0.31 s | 0.16 s | 0.46 s | 0.29 s |
| revolute chain 100 | full | 0.17 s | 0.13 s | 0.14 s | 0.12 s |
| revolute chain 100 | modified | 0.088 s | 0.053 s | 0.12 s | 0.12 s |
| revolute chain 1000 | full | 1.59 s | 1.52 s (12 s) | 1.59 s | 1.12 s (17 s) |
| revolute chain 1000 | modified | 0.77 s | 0.55 s | 1.12 s | 0.90 s |

Over 1000 steps with full Newton, when the union pattern has settled (scratch pardisoLong.py): the spring chain of 1000
bodies 36.9 s with EigenSparse on one thread, 18.5 s on 8 threads with sleeping workers (§5), 34.2 s with PardisoMKL on
one thread and 15.3 s on 8 threads; the revolute chain of 1000 bodies 21.7, 18.6, 21.9 and 16.8 s.

PARDISO pays only for full Newton on large systems with several threads; its solve is 2-4 times slower than Eigen's
(§3.9 of `linalgBenchmark.md`), so modified Newton, where the solves dominate, is slower with it. The analyses of the
growing union pattern take much of what the faster factorization gains. **The type stays experimental**; it is a
choice for a user with a large model, full Newton and MKL, to be compared with `EigenSparse` on the model.

## 3. Why PARDISO failed - the study (done, #3430)

Jacobians exported from Exudyn after two steps (full Newton), solved by PARDISO of MKL 2025.0 through ctypes, every call
in its own process, as PARDISO crashed (access violation) in some settings; the relative residual for a random
right-hand side, tolerance $10^{-8}$ (scratch pardisoStudy.py, pardisoMain.py, pardisoFollow.py, pardisoVerify.py). 18
Jacobians in three classes: ODE models (chains of mass points and of rigid bodies on springs, a grid of mass points),
index-3 models (chains of 10-1000 bodies on revolute, spherical and generic joints, 1000 independent pendulums, 30
four-bar loops in a row) and the same chains at index 2 (`useIndex2Constraints`).

| # | hypothesis | result |
|---|---|---|
| H1 | the format (triplets, duplicates, order, the hand-over) | **refuted**: the compressed columns of Eigen passed as the rows of the transposed (`iparm[11] = 2`) give the same residuals as the compressed rows |
| H2 | the scaling | **refuted**: equilibration did not help (`linalgBenchmark.md` §3.9); switching PARDISO's scaling and matching off made it worse (13 of 18) |
| H3 | the conditioning | **confirmed as the limit, not as the cause**: the residual of every solver that works follows the condition - $3\cdot10^{-11}$ at $2\cdot10^7$ (100 revolute bodies), $10^{-7}$ at $2\cdot10^{11}$ (1000 bodies), also for SuperLU; index 2 has the same condition but residuals of $10^{-13}$ to $10^{-11}$ |
| H4 | the numerical noise | **refuted**: dropping the entries below $10^{-14}$ of the row maximum changed nothing that failed, and raised the residuals of the others by one order |
| H5 | the zero diagonal of the multipliers | **refuted**: the weighted matching (`iparm[12] = 1`, the default for `mtype = 11`) moves large entries to the diagonal; index-2 Jacobians behave as index-3 ones |
| H6 | the settings | **the cause**: the METIS ordering (`iparm[1] = 2`, the default) with the parallel or the two-level factorization is wrong or fails on the larger constrained chains - the default on 8 threads 12 of 18, `iparm[23] = 10` error -15, NaN or an access violation on the chains of 1000 bodies, `iparm[23] = 1` wrong on 100 bodies with 8 threads; the minimum degree ordering with the two-level factorization on 8 threads was wrong even on ODE models ($10^{27}$). **The minimum degree ordering with the one-level factorization (`iparm[1] = 0`, `iparm[23] = 0`) solved all 18 on 1, 8 and 16 threads** to the residual of SuperLU; pivot perturbation ($10^{-8}$ ... $10^{-16}$) and iterative refinement (up to 20 steps) changed nothing |

Classes: ODE models and moderately conditioned constrained models are reliable with any setting on one thread; the
chains of more than about 100 constrained bodies need the minimum degree ordering and the one-level factorization,
which are Exudyn's defaults. Its factorization is 3.3 (1 thread) to 4.8 (8 threads) times faster than Eigen's
analysis and factorization of the spring chain of 1000 bodies, and 1.5 to 5 times for the revolute chain; METIS with
the two-level factorization would be another 3-5 times faster where it works, so `pardisoParameters` keeps it
reachable, guarded by the check.

## 4. The back-substitution

The solve with the factors is serial in Eigen's `SparseLU` (forward and backward substitution through the supernodes).
How much of it can run in parallel follows from the dependencies of the triangular solves. Counted on the factors of
SuperLU with COLAMD (the ordering of Eigen's `SparseLU`; scratch levels.py):

| model | n | unknowns | nonzeros of L + U | levels of L / U | unknowns per level |
|---|---:|---:|---:|---:|---:|
| revolute chain | 1000 | 13000 | 126962 | 3563 / 3997 | 3.3 |
| spring chain | 1000 | 8000 | 137525 | 2997 / 3039 | 2.6 |
| mass point chain | 1000 | 3000 | 8590 | 1001 / 1001 | 3.0 |
| 1000 independent pendulums | 1000 | 13000 | 58000 | 4 / 3 | 3250 |
| grid of 32 x 32 mass points | 1024 | 3072 | 17784 | 190 / 190 | 16 |

With these orderings a chain - most mechanisms - has about 3 unknowns per level. **The ordering decides it**: a
nested-dissection ordering (the graph bisected by a separator, the halves ordered first, recursively) makes the halves
independent. Symbolic factors of the pattern of $\Am + \Am\tp$ (static pivoting, as PARDISO), a simple nested
dissection by breadth-first level separators (no METIS) against minimum degree; the speedup bound is the work of the
forward substitution (the nonzeros of L) over its critical path (scratch ndLevels.py):

| model | unknowns | ordering | nonzeros of L | levels | speedup bound |
|---|---:|---|---:|---:|---:|
| revolute chain 1000 | 13000 | minimum degree | 51976 | 2002 | 4.5 |
| | | nested dissection | 105694 | 72 | 60.6 |
| spring chain 1000 | 8000 | minimum degree | 76123 | 4491 | 1.1 |
| | | nested dissection | 138836 | 101 | 23.8 |
| mass point chain 1000 | 3000 | minimum degree | 4295 | 1002 | 2.1 |
| | | nested dissection | 5536 | 40 | 17.9 |
| grid 32 x 32 | 3072 | minimum degree | 7954 | 122 | 3.5 |
| | | nested dissection | 10445 | 83 | 4.2 |
| 300 four-bar loops | 13500 | minimum degree | 28800 | 7 | 1108 |
| 1000 pendulums | 13000 | minimum degree | 30000 | 4 | 2500 |

Nested dissection doubles the fill of the chains (twice the work of the serial solve) but cuts the levels from
1000-4500 to 40-100, with bounds of 18-60; on 8 threads a net gain of 2-3 for the solve of chains is plausible. As the
maintainer pointed out, with a constant step size the Jacobian of a realistic model hardly changes, so modified Newton
solves far more often than it factorizes, and such a gain counts. **Recommended as its own step (RG5.9.7, #3432)**: the
ordering by nested dissection (no METIS), Eigen's `SparseLU` with it as a custom ordering, its factors copied into a
plain format after each factorization, and the substitution by subtrees of the elimination tree on Exudyn's threads;
measured against the serial solve.

### 4.1 The prototype (RG5.9.7, #3432)

Built as `exu.special.solver.parallelSubstitution` (experimental, default False), `src/Linalg/ParallelSubstitution.h/.cpp`:
`SparseLUParallel` is Eigen's `SparseLU` with an ordering functor `NestedDissectionOrdering`; after each factorization
the supernodal L and the U of Eigen are copied into rows (CSR), and the solve runs the tree nodes depth by depth - the
forward substitution from the leaves up, the backward one from the root down, the nodes of one depth in a
`ParallelFor`, a depth with little work serially.

What it took, each found by measuring:
- **Row pivoting** (Eigen pivots partially, threshold 1.0) couples the subtrees: the static-pivoting bound above was
  optimistic. With the scratch study ndTree.py / ndLift.py (SuperLU with the symmetric permutation): the multipliers
  and other unknowns with a zero diagonal must be eliminated after their neighbours - they are **lifted into the
  highest tree node of their neighbours with a nonzero diagonal** and ordered last in it - and the diagonal pivot
  threshold is lowered to 1e-3 (`parallelSubstitutionPivotThreshold`); then no dependency crosses the tree. Lifting
  along neighbours that are lifted themselves carried the first Euler parameters (zero diagonal at the reference
  rotation) of a whole chain into the root; only the nodes of the dissection decide.
- **The pattern** changes from one Jacobian to the next, as exact zeros are dropped: an ordering of the pattern seen
  was crossed by new entries in 77 of 80 factorizations, also on the union of the patterns and on the squared graph.
  The solver hands the linear solver the **coordinates of each object** (`GeneralMatrix::SetStructuralGroups`, from
  the local-to-global indices of ODE2, ODE1 and AE), and the ordering works on the pattern joined with their cliques:
  one ordering per simulation; when the pattern changes, Eigen's analysis only renews its elimination tree with the
  ordering kept. Keeping the zeros in the Jacobian instead made the factorization 1.8 times slower.
- A factor that couples two subtrees anyway is detected after the factorization, solved serially
  (`parallelSubstitutionSerial`), and ordered anew next time; a failed factorization falls back to SparseLU.

Measured (scratch psubBench.py, the fastest of three runs, times in s: total, factorization, solve; the results agree
to 1e-9):

| model | Newton | factorizations / solves | SparseLU 1 thread | SparseLU 8 threads | ND 1 thread | ND 8 threads | ND 16 threads |
|---|---|---|---|---|---|---|---|
| revolute chain 100 | modified | 399 / 4941 | 0.695 (0.143, 0.110) | 0.464 (0.152, 0.114) | 0.675 (0.156, 0.041) | 0.568 (0.254, 0.044) | 0.623 (0.255, 0.083) |
| | full | 2762 / 2762 | 1.699 (1.010, 0.067) | 1.437 (1.029, 0.067) | 1.627 (0.978, 0.024) | 1.869 (1.520, 0.032) | 1.907 (1.561, 0.057) |
| revolute chain 1000 | modified | 80 / 943 | 1.597 (0.359, 0.244) | 0.973 (0.406, 0.262) | 1.538 (0.357, 0.102) | 0.852 (0.434, 0.059) | 0.806 (0.482, 0.059) |
| | full | 527 / 527 | 3.638 (2.264, 0.138) | 2.888 (2.399, 0.142) | 3.495 (2.100, 0.055) | 3.156 (2.783, 0.031) | 3.213 (2.913, 0.037) |
| mass point chain 1000 | modified | 1 / 3577 | 1.634 (0.000, 0.091) | 0.537 (0.001, 0.095) | 1.591 (0.000, 0.055) | 0.468 (0.001, 0.053) | 0.427 (0.001, 0.079) |
| | full | 1410 / 1410 | 2.267 (0.547, 0.041) | 1.111 (0.665, 0.045) | 2.214 (0.511, 0.026) | 1.150 (0.751, 0.022) | 1.009 (0.687, 0.034) |
| mass point chain 10000 | modified | 1 / 197 | 1.114 (0.006, 0.059) | 0.303 (0.007, 0.061) | 1.111 (0.005, 0.029) | 0.275 (0.007, 0.019) | 0.204 (0.007, 0.020) |
| | full | 100 / 100 | 2.014 (0.544, 0.032) | 0.967 (0.580, 0.032) | 1.947 (0.478, 0.015) | 0.832 (0.531, 0.011) | 0.786 (0.576, 0.012) |

What it shows:
- **The solve by rows alone** - one thread, no parallelism - is 1.6-2.8 times faster than the supernodal solve of
  Eigen's SparseLU (the chains have small supernodes, and Exudyn copies the vectors around it). The threads add 0.9-1.7 on
  top: a solve takes 10-60 microseconds, so the parallel loops, the permutations and the serial top of the tree weigh.
  Together: 1.8-4.4 times faster solves than SparseLU on 8 threads.
- **The factorization** with the nested-dissection ordering is as fast as with COLAMD on one thread, but up to 70 %
  slower on 8 threads for the chain of 100 bodies and 7-16 % for 1000 (the deeper dissection and the spinning workers;
  sleeping workers, RG5.9.3, gave back a part).
- In total, on 8 threads against SparseLU on 8 threads: modified Newton on the chains of 1000 and 10000 is 9-13 %
  faster, full Newton on the chain of 10000 mass points 14 %; the chain of 100 bodies and full Newton on the chain of
  1000 bodies are slower. The option stays experimental and off.
- The largest gain that does not depend on the threads is the solve by rows: it would apply to every SparseLU
  factorization with COLAMD as well (a serial substitution with the rows of the factors, about 2 times faster solves
  for modified Newton) - proposed as a next step.

### 4.2 The substitution by rows, the default (RG5.9.9, #3441)

`SparseLURows<Ordering>` (ParallelSubstitution.h) is the base of both: Eigen's `SparseLU` whose `factorize` invalidates
the rows and whose `ExtractRows()` copies L and U into rows after a successful factorization; `SolveRows` is the serial
substitution. `GeneralMatrixEigenSparse` (EigenSparse, COLAMD) uses it if `exu.special.solver.sparseSolveByRows`
(default True). Measured on the 5 performance models and 52 test models with EigenSparse (scratch rowsStudy.py, each in
its own process, the faster of two runs): performance models 22.21 -> 21.68 s (-2.4 %; perf3DRigidBodies 1.40 -> 1.15 s,
-18 %; perfConnectorInterface -5 %), test models 21.23 -> 20.64 s (-2.8 %; up to -11 % for gridGeomExactBeam2D); no
model clearly slower (+4 to +9 % only for models of a few milliseconds). The gain is smaller than for the chains of §4.1,
as most models spend little time in the solve.

## 5. The threads of the factorization (built, #3428)

Exudyn's `ExuThreading::TaskManager` (`src/Utilities/Threading.h/.cpp`, after NGsolve) keeps `numberOfThreads - 1`
workers that wait for the next job by spinning with `std::this_thread::yield()`, also during the factorization, which is
serial in Eigen. MKL's threads cannot be Exudyn's (MKL creates its own OpenMP or TBB threads).

- **Built**: `TaskManager::SuspendWorkers()` sets a flag; a worker in its spin loop then waits on a condition variable
  until `ResumeWorkers()`, the next job (`CreateJob` resumes first, so a job never waits for sleeping workers) or the end
  of the TaskManager. `GeneralMatrixEigenSparse::FactorizeNew` suspends them for a factorization of at least
  `exu.special.solver.suspendWorkersMinimumSize` unknowns (default 1000) if
  `exu.special.solver.suspendWorkersDuringFactorization` (default False), and resumes them when it returns. NGsolve's
  `SuspendWorkers(asleep_usecs)` let them sleep for a fixed time; with the condition variable they sleep exactly as long
  as the factorization, and the cost is the wake-up of the threads in the first job after it.
- **Measured** (the table of §2.4, 8 threads, EigenSparse): full Newton - the spring chain 2.14 s (CPU 17 s) without,
  1.89 s (CPU 6.9 s) with sleeping workers, the revolute chain of 1000 bodies 1.52 s (12 s) and 1.31 s (4.3 s), of 100
  bodies 0.133 s and 0.114 s; modified Newton (few factorizations) - 0.155 / 0.158 s, 0.553 / 0.499 s, 0.053 / 0.059 s,
  within the scatter. With PARDISO on 8 threads it changed little (MKL's own threads spin in OpenMP as well).
- It stays off by default, as the maintainer asked: very small thread tasks pay the wake-up; a setting switches it.
- **Not done** (maintainer): factorizing independent subsystems on Exudyn's threads (RG5.9.5). Realistic models have no
  decoupled components; a user who has them splits them into systems of one SystemContainer, and solving those in
  parallel is RG5.10 (#3433).
- **Persistent threads** (RG5.9.8, #3437), instead of sleeping workers by default (maintainer):
  `exu.special.solver.StartPersistentThreads(n)` starts a task manager that stays; while no computation uses it, it is
  detached from `task_manager`, `GetNumThreads()` is 1 for all other code, and its workers sleep on the condition
  variable above. `TaskManager::AcquireWorkers(n, takeOver, message, ticket)` gives a solver (or the raytracer, or the
  linear algebra benchmark) the persistent workers if `n` agrees, otherwise a task manager of its own, stopping the
  persistent one for that run with a message - `parallel.numberOfThreads` wins; `ReleaseWorkers(acquired, ticket)`
  parks them again or stops its own threads and restarts the persistent ones, and leaves alone threads another
  computation took over (the ticket; #2237). An `atexit` hook stops them before Python ends. Measured (a chain of mass
  points, EigenSparse, scratch persist2.py; per solve): 200 bodies, 1 step - 0.44 / 0.63 / 1.10 ms with threads of its own
  on 4 / 8 / 16 threads, 0.26 / 0.24 / 0.30 ms persistent; 20 steps - 2.03 / 2.11 / 2.49 ms against 1.82 / 1.30 / 1.67 ms;
  2000 bodies, 20 steps - 26.1 / 18.2 / 14.8 ms against 25.0 / 17.6 / 13.7 ms. Starting and stopping the threads costs
  0.2-0.8 ms per solve.

## 6. Steps (RG5.9)

1. RG5.9.1 (#3427, closed): this design and evaluation.
2. RG5.9.2 (#3430, closed): the PARDISO study of §3.
3. RG5.9.3 (#3428, closed): the workers sleep during a factorization (§5).
4. RG5.9.4 (#3431, closed): `LinearSolverType.PardisoMKL` loaded at run time (§2).
5. RG5.9.5 (closed, not done): independent subsystems; RG5.10 (#3433) solves systems of one SystemContainer in parallel.
6. RG5.9.6 (#3434, closed): `reuseAnalyzedPattern` by default - not done: the performance models gain nothing, the test
   models with sparse solvers 1.6 % (at most 13 %).
7. RG5.9.7 (#3432, closed): the parallel forward and backward substitution with a nested-dissection ordering (§4.1).
8. RG5.9.8 (#3437, closed): persistent threads of the TaskManager, started by `exu.special.solver` (§5).
9. RG5.9.9 (#3441, closed): the substitution by the rows of the factors for every SparseLU factorization (§4.2).
