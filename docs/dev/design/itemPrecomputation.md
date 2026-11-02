# Precomputed item states and thread-local data: design (revision2026c step RG9.11.13.3, #3229)

Maintainer 2026-11-02: the kinematic tree is to compute its kinematics once instead of in every marker; for single
robots, the usual case, that is the preferred way. There is no precomputation step in the solvers, and calling one
only in `CSystem::ComputeSystemODE2RHS` is dangerous, since the mass matrix and other places need the same state. An
iteration counter next to the time, changed whenever the solution vectors change, would be safe at least serially,
with overhead; the numerical differentiation would have to change it for every coordinate, or a switch in mbs would
disable the precomputation while it runs. `UpdateUserFunctionsForThreads()` could be treated alike, possibly once per
iteration. Thread-local data is valuable anyway. This note compares the options and proposes one.

## 1. What exists

- `ObjectKinematicTree` computes the frames of all links, from the base to the leaves, in
  `ComputeTreeTransformations(configuration, velocities, ...)` into its mutable members `jointTransformationsTemp`,
  `jointVelocitiesTemp`, `jointAccelerationsTemp` and `links` (separate members for the visualization). Every call of
  the marker functions (`ComputeRigidBodyMarkerDataKT`, `GetPositionKinematicTree`, ... , and since #3216
  `GetVelocity_qKinematicTree` and the Jacobian derivative) computes all frames again; so do the sensors and the
  output variables. The object's own `ComputeMassMatrixAndODE2LHS` computes them in its own recursion.
- In parallel (`parallel.numberOfThreads` > 1), the residual (`ComputeSystemODE2RHS`), the mass matrix, the algebraic
  equations, the reaction forces and the Jacobians compute the items of the lists without Python functions in
  `ParallelFor`; two connectors on markers of one tree write the same members at the same time (#3229). Since
  RG9.11.14 the parallel ODE2 Jacobian defers such connectors to the serial part; the residual and the AE rows do not.
- `CSystem::UpdateUserFunctionsForThreads()` runs serially at the start of the six system functions that have a
  parallel loop (ComputeMassMatrix, ComputeSystemODE2RHS, ComputeODE2LoadsRHS, ComputeAlgebraicEquations,
  PostNewtonStep, ComputeODE2ProjectedReactionForces): it rebuilds the item lists if a user function was assigned and
  prepares the thread copies of the symbolic user functions once per number of threads; afterwards it costs a
  comparison.
- Thread-local data exists: `TemporaryComputationDataArray` has one `TemporaryComputationData` per thread, with
  `markerTemp[2]` (MarkerTemp: rotation, G, the Jacobians, dv/dq) for the connector interface. The marker functions of
  the marker data path, `CMarker::ComputeMarkerData(cSystemData, computeJacobian, markerData)`, get no temporary data;
  the default `GetKinematicsJacobian...` uses a `TemporaryMarkerDataStructure` on the stack.
- The state is written in many places: the solvers (the vectors of `currentState`, by reference and through
  LinkedDataVectors), the Lie group composition, the numerical differentiation of the system (`x[i] += eps` in
  `NumericalDifferentiation`, `AddNumDiffObject`, `NumericalJacobianAE`, ...), the static solver, the initial
  accelerations, `mbs.systemData.SetODE2Coordinates` and the other setters from Python, and the user functions.
- Measured (RK44, 2000 steps, a chain of revolute links with a CartesianSpringDamper from every link to the ground):
  6 links 0.026 s without the markers / 0.131 s with them, 20 links 0.151 / 1.22 s, 60 links 1.37 / 11.0 s. The markers
  cost 5 to 8 times the tree, O(n^2) for n markers, as each recomputes the tree with its velocities.

## 2. Options

**A. Thread-local temporaries.** The marker functions compute the frames into temporaries of the calling thread
(MarkerTemp, or a buffer per thread in the object) instead of the members. Thread-safe, nothing to invalidate, results
unchanged. It does not reduce the cost: every marker still computes the whole tree. `ComputeMarkerData` and the
`Get...` functions have no temporary data in their signatures, so they need either a thread buffer of the object,
indexed by `ExuThreading::TaskManager::GetThreadId()`, or new arguments.

**B. A state counter.** `CData` gets a counter next to the time, incremented whenever a state vector changes; an object
recomputes its cache when the counter differs from the one it stored. Cheap to check. Every writer of the state must
increment it - the solvers, the numerical differentiation per coordinate, the Lie group update, the Python setters,
user functions that set coordinates; one writer missed gives silently wrong results, and the writers are spread over
the code (section 1). A switch that disables the caches during numerical differentiation removes one class of
writers, but not the others.

**C. A cache validated by its inputs.** The object keeps the frames together with a copy of the coordinates (and
velocities) they were computed from; a marker function compares the current coordinates of the tree's node with the
copy - n numbers for n links, about ten times cheaper than recomputing the frames - and uses the cache if they are
equal, else computes. No writer has to know about the cache, the numerical differentiation included: a changed
coordinate fails the comparison. Thread safety needs that the cache is not written inside a parallel loop.

**D. A precomputation step of the system.** `CSystem::PreComputeItems()`, serial, at the start of each system function
that has a parallel loop - where `UpdateUserFunctionsForThreads()` is called now, which it would contain -, calls
`CObject::PreCompute()` for the items that declare it (a list built in `PreComputeItemLists`). It makes the parallel
part read only. Alone, it is option B with the system functions as the writers' boundary: a caller that changes a
coordinate and then calls an item function directly, outside a system function - the numerical differentiation of an
object or a connector by CSystem (`ComputeObjectODE2LHS` after `x[i] += eps`), the sensors, the output variables,
Python - would read a stale cache.

## 3. Proposed: C and D together, with A as the fallback

- **The cache (C)**: `ObjectKinematicTree` keeps the frames, velocities and accelerations of `Current` with the
  coordinates, velocities and accelerations of its node that they belong to, and a flag which of them are valid. A
  marker, sensor or output function compares and uses the cache.
- **The precomputation (D)**: `CSystem::PreComputeItems()` replaces the calls of `UpdateUserFunctionsForThreads()`
  and calls it; then `PreCompute()` of the items in `cSystemData.listPreComputeObjects` (the trees with markers), which
  validates the cache - recomputes it only if the comparison fails. In the parallel loops that follow, the comparison
  succeeds and the cache is only read.
- **The fallback (A)**: if the comparison fails inside a parallel loop - which the precomputation should prevent, but
  e.g. a user function may change a coordinate - the marker computes into a buffer of its thread instead of the cache,
  correct and only slower. Serially, a failed comparison recomputes the cache, so the numerical differentiation of a
  connector by CSystem gets the perturbed tree.
- The precomputation runs at most once per system function, i.e. a few times per Newton iteration; with an unchanged
  state it costs the comparisons. `UpdateUserFunctionsForThreads()` keeps its own check and is not needed more often.
- No counter in CData and no switch for the numerical differentiation: the comparison makes both unnecessary. A
  counter remains an optimization for later, if the comparisons are measurable (they are O(n) against O(n) frames
  with 3x3 products).
- The object's own `ComputeMassMatrixAndODE2LHS` can take the frames from the cache as well; it is called in the
  serial or the parallel part, but once per tree, after the precomputation.
- Then the deferral of RG9.11.14 is removed, and connectors and constraints on the markers of a tree are computed in
  parallel again.

Expected: with markers on every link, the markers cost O(1) for the kinematics plus O(n) for their Jacobians instead of
O(n) each; the 20-link model near the cost of the tree alone plus the connectors.

## 4. Other items

The same pattern fits other items whose markers recompute a shared state: ObjectFFRFreducedOrder and the
super-element markers (the mode basis times the coordinates for averaged nodes), ObjectGenericODE2 with many mesh
markers, the beams with markers on the same element. They are not changed in this step; the interface (`PreCompute()`
and the list) is general.

## 5. Sub-steps

1. Thread buffers: `ObjectKinematicTree` computes the frames of a marker call into a buffer of the thread (A), the
   members stay for the object's own functions; the deferral removed; the test suite with 4 threads. This alone fixes
   #3229.
2. The cache with its comparison (C) in the tree, used by the markers, sensors and output variables; measured on the
   models of section 1, also with numerical differentiation by CSystem.
3. `CSystem::PreComputeItems()` with `CObject::PreCompute()` and the list (D), containing
   `UpdateUserFunctionsForThreads()`; the tree validates its cache there.
4. The object's own LHS and mass matrix on the cache, if 2 and 3 show a gain.

## 6. To be decided

- Whether 1 (the fix of the race) is done first, alone, as proposed, or only together with 2 and 3.
- Whether the comparison takes the coordinates only (frames) and the velocities only when a velocity is asked for, or
  always both.
- Whether the precomputation is in the scope of this step for other items (section 4), or later.
