# Fewer parallel regions per residual

A design note of revision2026c step RG5.14 (#3494): RG5.14.1 measures what a parallel region costs and how many the
solvers start; RG5.14.2 designs how the tasks of a residual run in one region. **Section 6 lists the decisions for the
maintainer.**

Measured 2026-11-13 on Exudyn 1.13.634.dev1, regular module, Windows, MSVC, Python 3.13, AMD Ryzen 9 9950X (16 cores,
32 logical processors, two dies of 8 cores). The models are those of `python/testing/solverBaseline.py`
(solverBaseline.md): GeneralizedAlpha, EigenSparse, modified Newton, the base step size; RK44 for `contactSpheres`. A
time is the smallest of 7 runs unless a median is given. The counts and the cost of an empty region come from a
temporary instrumentation of `TaskManager::CreateJob` (a counter and a clock per call site, keyed by the type of the
lambda); it was not committed.

## 1. The cost of a region

`ExuThreading::ParallelFor` starts a region with `TaskManager::CreateJob`: the main thread writes the job, sets one
flag per worker, takes its own share and waits for every worker's flag. The workers spin between regions
(`std::this_thread::yield`), so a region starts without a wake-up. 20 000 regions back to back, one task per thread,
each task summing `work` numbers:

| threads | empty region [µs] | work 100 | work 1000 | work 10 000 | the same work serially, 10 000 |
|---:|---:|---:|---:|---:|---:|
| 2 | 0.24 | 0.30 | 0.61 | 3.8 | 7.0 |
| 4 | 0.44 | 0.46 | 0.80 | 4.0 | 14.1 |
| 8 | 0.71-0.90 | 0.74 | 1.08 | 4.5 | 28.8 |
| 16 | 3.0-3.3 | 2.8 | 2.7 | 6.3 | 58.7 |

Up to 8 threads a region costs well below a microsecond; at 16 it costs 3 µs, as the threads span both dies. A region
pays from about 1000 operations per thread.

## 2. The regions of the solvers

Per call site and solve (4 threads):

| model | coordinates | items | Newton iterations | residuals | regions per residual | per Jacobian |
|---|---:|---|---:|---:|---|---|
| massOscillator, nPendulum, rigidBodyJoints, rigidBodySpringDampers, sliderCrank3D, kinematicTree, beam3D, FFRFreducedOrder, genericDAE | 2-42 | fewer than 20 per list | - | - | **0** | 0 |
| rigidBodySpringDampers50 | 400 | 50 bodies, 50 spring-dampers | 1655 | 1906 | 5 | 2 |
| rigidBodyJoints50 | 650 | 50 bodies, 50 joints | 1373 | 1624 | 5 | 2 |
| beam3D60 | 372 | 60 beams | 1149 | 1350 | 3 (no lists of constraints with 20 items) | 1 |
| contactSpheres, RK44 | 324 | 108 spheres, GeneralContact | - | 4 per step | 2 + 5 of GeneralContact | - |

- The five regions of an implicit residual (`CSolverImplicitSecondOrder::ComputeNewtonResidual`) are, in this order:
  `ComputeMassMatrix` (only if the mass matrix is not constant), `ComputeSystemODE2RHS` (the objects),
  `ComputeODE2LoadsRHS` (called at its end), `ComputeODE2ProjectedReactionForces` ($\Cm_q\tp \lambda$) and
  `ComputeAlgebraicEquations`. A residual is computed once per Newton iteration and once at the start of each step.
- A Jacobian adds `JacobianODE2RHS` and `JacobianAE` (with modified Newton, rarely).
- An explicit stage has `ComputeSystemODE2RHS` and `ComputeODE2LoadsRHS`; the three loops of `CSolverExplicit.cpp` run
  serially below 100 nodes (`costs = 0`). GeneralContact adds five: data and bounding boxes of spheres, of triangles
  and of ANCF cables, the forces of spheres and of triangles. Two of the five ran on empty lists in `contactSpheres`
  (0.5 µs each).
- **The small models start no region at all**: every list is below `multithreadedLowerLimit*` (20). Their slowdown
  with 4 threads (1.02-1.70 in the solver baseline) is the start and stop of the workers per solve, about 0.15 ms
  with 4 threads and 0.7 ms with 16 (massOscillator 0.35 ms on 1 thread, 0.52 on 4, 1.1 on 16). **Fewer regions do
  not help them**; RG5.14.5 (start no workers when no list reaches its limit) or the persistent workers (#3437) do.

## 3. What the regions cost in a step

Solver timers [ms] (the ODE2RHS timer includes the loads and $\Mm \ddot\qv$):

| model | threads | total | ODE2RHS | massMatrix | reactionForces | AERHS | factorization | integrationFormula |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| rigidBodySpringDampers50 | 1 | 68.8 | 40.3 | 9.0 | 2.5 | 1.7 | 4.1 | 0.6 |
| | 4 | 37.7 | 16.4 | 4.0 | 2.8 | 1.5 | 4.2 | 0.7 |
| | 8 | 31.6 | 11.8 | 3.3 | 2.7 | 1.6 | 4.0 | 0.7 |
| | 16 | 54.9 | 20.0 | 7.8 | 6.8 | 4.2 | 4.8 | 2.0 |
| rigidBodyJoints50 | 1 | 81.8 | 9.5 | 7.7 | 18.7 | 10.6 | 20.6 | 0.7 |
| | 4 | 61.5 | 6.3 | 3.5 | 12.4 | 6.8 | 20.6 | 0.7 |
| | 8 | 54.3 | 6.3 | 3.3 | 8.1 | 4.7 | 20.9 | 0.8 |
| beam3D60 | 1 | 203.7 | 109.7 | 76.0 | 0.3 | 0.2 | 0.9 | 9.6 |
| | 4 | 108.4 | 50.8 | 33.8 | 0.5 | 0.3 | 1.3 | 14.2 |
| | 16 | 68.1 | 26.5 | 15.8 | 0.7 | 0.3 | 1.2 | 16.0 |

The start of the regions of a residual, from section 1, as a share of the time of a residual:

| model | 4 threads | 8 threads | 16 threads |
|---|---:|---:|---:|
| rigidBodySpringDampers50 (19.8 / 16.6 / 28.8 µs per residual) | 5 × 0.44 µs = 11 % | 5 × 0.8 µs = 24 % | 5 × 3 µs = 52 % |
| rigidBodyJoints50 (38 / 33 µs per residual) | 6 % | 12 % | - |
| beam3D60 (80 / 63 / 50 µs per residual) | 2 % | 4 % | 18 % |

One region instead of five saves four starts per residual: about 9 %, 19 % and 42 % of rigidBodySpringDampers50 on 4,
8 and 16 threads, less for the models with more work per item.

**A larger cost is the waiting of the workers.** With every lower limit set to $10^9$ the workers run but no region
starts, so the solver is serial, and still (rigidBodySpringDampers50, median of 7):

| threads | all 32 logical processors | one logical processor per core (affinity 0x55555555) |
|---:|---:|---:|
| 1 | 69.4 ms | 69.4 ms |
| 4 | 105.5 ms (fastest 78) | 74.5 ms |
| 8 | 106.1 ms | 74.7 ms |

A worker spinning on the second logical processor of the main thread's core slows the main thread by up to 50 %, and
it depends on where Windows puts the threads: the same run gives 70 or 105 ms. This hits every serial part - the
factorization, the Newton update, the merging of the per-thread vectors, the user functions - and it does not shrink
with fewer regions. It also makes single measurements of multithreading unreliable on this machine: the solver
baseline's 4-thread ratios contain it. A new sub-step, RG5.14.6, takes it up.

## 4. Design: the tasks of a residual in one region

**What is independent.** The five parts of a residual read the same state (coordinates, velocities, accelerations,
multipliers, time) and write different outputs: the mass matrix, the ODE2 right-hand side (objects and loads), the
projected reaction forces and the algebraic equations. No part reads what another writes within the residual. Each
item already computes its contribution into the scratch data of its thread (`TemporaryComputationData`), which is
merged serially after the region, in the order of the threads; items with Python user functions are computed in the
main thread after the region (#2978). `PreComputeItems()` runs serially before each part and can run once before the
region.

**Option A - one task list, one region (recommended).** A function of `CSystem`, say
`ComputeResidualParts(tempArray, parts, outputs)`, with `parts` the flags of the five parts, builds one index range
over the concatenated lists without user functions - bodies (mass matrix), objects (ODE2), loads, items with
algebraic equations, items with reaction forces - and runs one `ParallelFor` over it; a task finds its part by the
range. Each thread writes to its own buffers: the mass matrix triplets as now, and one sparse vector per output
(today one `sparseVector` is reused part by part), so `TemporaryComputationData` gets two more sparse vectors. After
the region come the serial merges, in the order of today, then the items with user functions, part by part.
- `taskSplit` follows the total number of items; the dynamic load balancing (the default) evens out the different
  costs of the parts.
- The existing functions become calls of `ComputeResidualParts` with one part, so there is one code path; the explicit
  stage calls it with the objects and the loads, the implicit residual with all five.
- On one thread the order of the operations stays as it is, and the results stay bitwise the same. On several threads
  the order of summation already depends on which thread takes which item (load balancing), so the results agree
  within rounding, as today.
- The solver timers `massMatrix`, `ODE2RHS`, `reactionForces` and `AERHS` can no longer separate the parts of a common
  region (decision 3).

**Option B - one region with barriers.** The threads stay in one job through the whole residual and meet at barriers
(an atomic counter) between dependent phases. It allows what A cannot: merging the per-thread vectors in parallel (by
row ranges), the product $\Mm \ddot\qv$ inside, and the dependent phases of GeneralContact (data and boxes, the search
tree, the forces). A barrier costs about what the join of a region costs, items with user functions must wait outside,
and an exception must reach every thread. For the residual it gains little over A (the merges are small: "less than
1 % of the overall time" at 12 threads, as the comment at the merge in `ComputeSystemODE2RHS` says); it is the
candidate for GeneralContact later.

**Option C - the product $\Mm \ddot\qv$ without the global mass matrix**, in either design. The residual needs only
$\Mm \ddot\qv$, but builds the triplets of the global mass matrix in every Newton iteration when the mass matrix is not
constant (beam3D60: 76 of 204 ms on one thread). Each body can multiply its local mass matrix with its own
accelerations inside the task; the global matrix is then built only for a Jacobian. It changes the order of summation
(within rounding) and is worth a measurement of its own.

**The explicit stage.** Its two regions (objects, loads) become one with option A. The five of GeneralContact are
dependent phases; skipping a region whose list is empty saves two of them in `contactSpheres` at once. The update
loops of `CSolverExplicit.cpp` run serially below 100 nodes and are not in the way.

**What changes**: `CSystem.cpp` (the new function; the five functions become callers of it), `CSystem.h`,
`TemporaryComputationData` (the additional sparse vectors), the residual of `CSolverImplicitSecondOrder.cpp`, the
stage of `CSolverExplicit.cpp`, the static solver and the implicit Runge-Kutta solver, which call the same functions, and `MainSolverBase.cpp`, which exposes them to Python. GeneralContact, the
Jacobians and ODE1 stay as they are (maintainer, #3494).

## 5. Expected effect

| change | small models (< 20 items) | rigidBodySpringDampers50, 4 / 8 / 16 threads |
|---|---|---|
| A: one region per residual | none | -9 / -19 / -42 % |
| RG5.14.6: workers not slowing the main thread | the per-solve cost remains | up to -30 % of the serial parts, whenever the threads share a core |
| RG5.14.5: no workers without a region | -0.15 ms per solve with 4 threads, the 1-thread time | none |
| C: $\Mm \ddot\qv$ without the global matrix | none | massMatrix 4.0 ms of 37.7 on 4 threads; beam3D60 33.8 of 108 |

## 6. Decisions for the maintainer

1. **Option A for the residual and the explicit stage (RG5.14.3, RG5.14.4)**, B left for GeneralContact - or B at
   once?
2. **Option C** within RG5.14.3, or as a sub-step of its own, measured separately?
3. **The timers**: the merged region measured by one timer (e.g. `ODE2RHS` covering all parts, the others only their
   serial parts), or the separate timers kept by calling the parts one by one while `displayComputationTime` is set
   (the measured run then differs from the unmeasured one)?
4. **RG5.14.6 first?** The waiting of the workers costs more than the regions for most models with threads; the
   measurement of A is only reliable once it is solved or controlled (affinity during the measurement).
5. **A counter of regions per call site**, as used for this note, kept behind a compile switch for RG5.14.3-.6, or
   left out (it was 60 lines)?
