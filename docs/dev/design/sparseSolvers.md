# Sparse solvers: MKL PARDISO at run time, the threads of the factorization, the back-substitution

A design note of open steps (revision2026c step RG5.9, #3427, #3428): what the sparse linear solver of the implicit
solvers costs, how an external solver (Intel MKL PARDISO) can be used without a build dependency, why it failed on some
of Exudyn's Jacobians and how that is studied, how the factorization can use threads while Exudyn's own workers wait,
and whether the back-substitution can run in parallel. It follows `linalgBenchmark.md` §3.7-§3.9, where the
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

## 2. MKL PARDISO, loaded at run time

### 2.1 The idea

No MKL in the wheel and none at build time. A user who wants the solver installs MKL into the environment
(`pip install mkl`, or conda, where it often is already). Exudyn's C++ loads the MKL runtime library when the solver
type is selected and takes three functions from it by name. Without MKL, selecting the solver type raises a clear
error; nothing else changes. Eigen's `PardisoSupport` is **not** used: it includes `mkl.h` and links `mkl_rt` at
build time, which would make MKL mandatory for `import exudyn` (rule 6).

### 2.2 Loading

- `src/Linalg/PardisoMKL.h/.cpp` (new): `class PardisoLibrary` with `static bool Load(std::string& message)`, loaded
  once per process, thread-safe; Windows `LoadLibraryW`, Linux `dlopen(RTLD_NOW | RTLD_LOCAL)`. macOS and ARM have no
  MKL: the type reports *not available on this platform*.
- Search order: the environment variable `EXUDYN_MKL_LIBRARY` (a full path), then the environment of the running Python
  (`sys.prefix`: `Library/bin/mkl_rt.2.dll` on Windows, `lib/libmkl_rt.so.2` on Linux - where conda and the `mkl` wheel
  put it), then the system search path (`mkl_rt.2.dll` / `libmkl_rt.so.2`, then the names without `.2`).
- The functions, declared in Exudyn with their C signatures of the LP64 interface (32-bit integers; Exudyn's sparse
  indices are `int`): `pardisoinit(void* pt[64], const int* mtype, int iparm[64])`, `pardiso(void* pt[64], const int*
  maxfct, const int* mnum, const int* mtype, const int* phase, const int* n, const double* a, const int* ia, const int*
  ja, int* perm, const int* nrhs, int iparm[64], const int* msglvl, double* b, double* x, int* error)`,
  `MKL_Set_Num_Threads(int)`, and `MKL_Get_Version_String(char*, int)` for the information. No MKL header is used.
- Python: `exu.special.PardisoInfo()` returns `{'available': bool, 'library': path, 'version': str, 'message': str}`.

### 2.3 The solver

- **Type**: a new `LinearSolverType.PardisoMKL`, sparse like `EigenSparse` (the same `GeneralMatrixEigenSparse` assembly,
  triplets and `setFromTriplets`), only factorization and solve differ: `GeneralMatrixEigenSparse` gets a member of a small
  class `PardisoSolver` used when the type is selected.
- **Matrix**: Eigen's compressed column-major matrix is passed as it is - its column storage is the row storage (CSR) of
  $\Am\tp$, so PARDISO solves the transposed system (`iparm[11] = 2`) and gets $\Am\xv = \bv$ without a copy; zero-based
  (`iparm[34] = 1`); `mtype = 11` (real unsymmetric). The arrays must be sorted and free of duplicates, which
  `setFromTriplets` guarantees.
- **Phases**: analysis (11) when the pattern changes (the number of nonzeros and a hash of the index arrays, as
  `reuseAnalyzedPattern` but exact), numerical factorization (22) in `FactorizeNew`, solve (33) in `Solve`, release (-1)
  when the matrix is resized or destroyed.
- **The check** (the maintainer's requirement: the factorization must be shown useful): after the first factorization
  of each analysis, and every `checkPeriod`-th one after it, the solve is followed by the residual
  $\lVert\Am\xv - \bv\rVert / \lVert\bv\rVert$ (one sparse matrix-vector product, about the cost of a back-substitution).
  Above `residualTolerance`, or with perturbed pivots (`iparm[13] > 0`) and a residual above the tolerance, the step is
  repeated with Eigen's `SparseLU`, a warning names the cause once, and the matrix stays with `SparseLU` until its
  pattern changes. The number of fallbacks is in the solver statistics.
- **Settings** (`simulationSettings.linearSolver`, new):

| setting | default | meaning |
|---|---|---|
| `solverType` | `EXUdense` | `PardisoMKL` selects the solver |
| `pardiso.numberOfThreads` | 0 | threads of MKL; 0 = Exudyn's `parallel.numberOfThreads` |
| `pardiso.checkPeriod` | 1 | the residual is checked after every n-th factorization of an analysis (the first always); 0 = never |
| `pardiso.residualTolerance` | 1e-8 | relative residual above which the step falls back to `SparseLU` |
| `pardiso.fallback` | True | False: an error instead of the fallback (for the study) |
| `pardiso.parameters` | `[]` | expert: pairs `[index, value]` written into `iparm` after `pardisoinit`; defaults set by Exudyn: `iparm[23] = 10` (the two-level factorization, which solved the revolute chain of 100 bodies that the default one did not), `iparm[11] = 2`, `iparm[34] = 1` |

- **Information**: `iparm[13]` (perturbed pivots), `iparm[6]` (refinement steps), `iparm[16]` (nonzeros of the factors)
  and the version into the solver output at `verboseMode >= 2`.
- **Tests**: a test model that runs only if `PardisoInfo()['available']` - an ODE model and a constrained one against
  `EigenSparse`, the fallback forced by `residualTolerance = 0`; the test suite with the type set globally as a study
  (§3), not as a gate.

### 2.4 What it gains (estimate)

From `linalgBenchmark.md` §3.9: PARDISO's numerical factorization 2.5-7 times faster on one thread, up to 24 times with
16 threads on the largest spring chain, and no copy and no Python call in C++. The spring chain of 1000 bodies with
full Newton: 198 factorizations of 6.3 ms against about 0.3 ms with 16 threads, about 1.2 s of 3.6 s saved (33 %); with
modified Newton almost nothing (one factorization). PARDISO's back-substitution is not faster than Eigen's (814 us on
one thread, 252 us on 16, against 262 us; `linalgBenchmark.md` §3.9).

## 3. Why PARDISO failed - the study

Observed (`linalgBenchmark.md` §3.9): ODE models solved to $10^{-16}$ in every setting; the revolute chain of 100 bodies
wrong with the default parallel factorization (residual $10^3$, error code 0) and right with `iparm[23] = 10`; the chain
of 1000 bodies wrong or failing (error -15) in every setting tried. Hypotheses and how each is tested:

| # | hypothesis | test | first evidence |
|---|---|---|---|
| H1 | the matrix format (triplets, duplicates, unsorted indices, the transfer) | the same CSR solved by SuperLU; a check of the canonical format | SuperLU solves the same arrays; scipy's canonical CSR was passed - unlikely |
| H2 | the scaling of the rows and columns | Ruiz equilibration before PARDISO | tried on the chain of 1000: no improvement - the entries span $10^{-48}$ to 2 because of the noise |
| H3 | the conditioning: static pivoting perturbs pivots by $10^{-13}\lVert\Am\rVert$, which a condition of $10^{11}$ turns into an error of order 1 | the chain for n = 10 ... 1000, the residual over the condition | the chain of 1000 bodies has $2\cdot10^{11}$; SuperLU reaches only $10^{-7}$ - a hard case for any static pivoting |
| H4 | the numerical noise (§1) as pivots or as false structure | drop the entries below $10^{-14}$ of the row maximum, then PARDISO | open |
| H5 | the zero diagonal of the multipliers (saddle point) needs 2x2 pivots | index-2 constraints (`useIndex2Constraints`), and the symmetric indefinite type `mtype = -2` where the Jacobian is symmetric | open |
| H6 | the settings: matching and scaling (`iparm[12]`, `iparm[10]`), perturbation (`iparm[9]`), refinement (`iparm[7]`), `iparm[23]` | a grid over them on the hard cases | partly done, none solved the chain of 1000 |

The study runs on Jacobians exported from models (scratch scripts, no change of Exudyn), in three classes - ODE
models (springs, FFRF, ANCF, penalty contact), index-3 constrained mechanisms of moderate size and conditioning (the
test models with `EigenSparse`), and the extreme chain - and ends with the classes for which PARDISO is reliable with
which settings. The residual check of §2.3 stays in any case: the classes only decide whether the type is recommended.

## 4. The back-substitution

The solve with the factors is serial in Eigen's `SparseLU` (forward and backward substitution through the supernodes).
Whether it can be parallelized follows from the level sets of the triangular factors - the unknowns whose
substitution does not depend on each other. Counted on the factors of SuperLU with COLAMD (the same ordering as
Eigen's `SparseLU`; scratch levels.py):

| model | n | unknowns | nonzeros of L + U | levels of L / U | unknowns per level |
|---|---:|---:|---:|---:|---:|
| revolute chain | 1000 | 13000 | 126962 | 3563 / 3997 | 3.3 |
| spring chain | 1000 | 8000 | 137525 | 2997 / 3039 | 2.6 |
| mass point chain | 1000 | 3000 | 8590 | 1001 / 1001 | 3.0 |
| 1000 independent pendulums | 1000 | 13000 | 58000 | 4 / 3 | 3250 |
| grid of 32 x 32 mass points | 1024 | 3072 | 17784 | 190 / 190 | 16 |

A chain - most mechanisms - has about 3 unknowns per level: there is nothing to parallelize. A grid has 16 per level
over 190 levels; each level needs a synchronization of the threads, and 190 of them - even at a microsecond each, not
measured - take longer than the whole serial solve (25 us for 3000 unknowns). Only independent subsystems parallelize, and for them a
level-scheduled solve is not needed: §5.3 solves each subsystem on its own. Reproducing Eigen's solve by hand would also
mean copying the supernodal factors into a plain format after each factorization. **Not recommended**; the
independent subsystems are the case to take (§5.3).

## 5. The threads of the factorization

### 5.1 Today

Exudyn's `ExuThreading::TaskManager` (`src/Utilities/Threading.h/.cpp`, after NGsolve) keeps `numberOfThreads - 1`
workers that wait for the next job by spinning with `std::this_thread::yield()`. During the factorization, which is
serial in Eigen, they keep their cores busy. `TaskManager::SuspendWorkers` and `ResumeWorkers` exist as empty stubs, and
`FactorizeNew` calls them only behind `LinearSolverSuspendWorkers`, which is `false` (`LinearSolver.cpp`). An external
solver with its own threads (MKL: OpenMP or TBB) would compete with the spinning workers for the cores; the
workers cannot be handed to MKL.

### 5.2 Options

| option | how | cost | for |
|---|---|---|---|
| A: workers sleep | `SuspendWorkers`: the workers leave the spin loop for a `std::condition_variable` (or C++20 `atomic::wait`) after a flag; `ResumeWorkers` sets it back and notifies | the wake-up of the threads, to be measured (tens of microseconds expected); so only around factorizations of more than about 1 ms | any multi-threaded external solver (PARDISO), and the serial `SparseLU` (frees the cores of the machine) |
| B: workers stopped | `ExitTaskManager` / `EnterTaskManager` around the factorization | thread creation, 20-50 us per thread | nothing that A does not do better |
| C: the solver on Exudyn's threads | independent subsystems: the connected components of the Jacobian's graph, each with its own `SparseLU` (or PARDISO with `mkl_sequential`), factorized and solved in a `ParallelFor` | the components once per pattern (a union-find over the triplets); then no overhead | models of many independent parts: particles with springs, many robots or vehicles, pendulum arrays; nothing for one connected mechanism |
| D: MKL's threads as Exudyn's | not possible: MKL creates its own OpenMP or TBB threads | - | - |

### 5.3 Recommendation

A first: it is small (the two stubs, a flag in the worker loop), needed before any multi-threaded external solver, and
measured by the wake-up time and by the factorization of the spring chain with PARDISO and 4-16 threads. C second, as
its own step: it uses the existing threads for both the factorization and the back-substitution, without a new
dependency, for the class of models that §4 shows to be the only one with parallel work in the solve. Contact couples
the components while it is active; the components are then recomputed with the pattern.

## 6. Steps (RG5.9)

1. RG5.9.1 (#3427, closed): this design and evaluation.
2. RG5.9.2: the PARDISO study of §3 on exported Jacobians - the classes and settings for which it is reliable.
3. RG5.9.3 (#3428): option A - the workers sleep during a factorization; the wake-up measured.
4. RG5.9.4: `LinearSolverType.PardisoMKL` loaded at run time (§2), with the residual check and the fallback; after
   RG5.9.2 and RG5.9.3.
5. RG5.9.5 (#3428): option C - independent subsystems factorized and solved in parallel on Exudyn's threads.
6. RG5.9.6: `reuseAnalyzedPattern` by default, with the numerical noise kept out of (or consistently in) the pattern
   (§1; `linalgBenchmark.md` §3.9).
