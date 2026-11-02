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

## 3. Decided and implemented: C and D together, no thread buffers

Maintainer 2026-11-02: C and D together; if the cache fails inside a parallel loop, an exception, not a fallback. Only a
Python user function can change a coordinate while the system is computed - a symbolic user function cannot, and the
preStepUserFunction and the preNewtonResidualUserFunction run before the step or the residual -; changing a
coordinate of the tree from another object computed earlier is not allowed; such a model is solved with one thread,
which the exception says. So no thread-local cache: the members of the tree are the cache. The comparison takes the
coordinates, the velocities and the accelerations always, as it is cheap and the markers ask for positions and
velocities alike. Only ObjectKinematicTree gets the cache; MarkerSuperElementRigid on ObjectFFRFreducedOrder needs a
better precomputation, not a cache (RG9.11.17.3).

- **The cache (C)**: `CObjectKinematicTree::PreCompute()` keeps the frames, velocities and accelerations of the current
  configuration in `jointTransformationsTemp`, `jointVelocitiesTemp` and `jointAccelerationsTemp`, with the current
  coordinates, velocities and accelerations and the reference coordinates of the node they are computed from in
  `treeCacheCoordinates` (4n numbers); it computes them again only if these differ or `treeCacheValid` is false, which
  `ParametersHaveChanged` (SetObjectParameter, ModifyObject) and Assemble set. The marker functions, the sensors and
  the output variables of the current configuration call it (`GetTreeTransformations`); the other configurations are
  computed into temporaries of the thread, the visualization into its own members as before. The chain of a link for
  the derivatives of the marker Jacobians is a temporary of the thread, no longer `tempArrayIndex`.
- **The precomputation (D)**: `CSystem::PreComputeItems()` replaces the six calls of `UpdateUserFunctionsForThreads()`,
  calls it, and then `PreCompute()` of the objects in `cSystemData.listPreComputeObjects` - those with `HasPreCompute()`
  and a marker on them, built in `PreComputeItemLists`. It is also called before the parallel loops of the ODE2 and AE
  Jacobians and before the Jacobian of the general contacts, which follows the numerical differentiation.
- **Inside a parallel loop** (`ExuThreading::TaskManager::JobIsRunning()`, true while `CreateJob` runs), a failed
  comparison throws an `ExuThreading::Exception`, which the task manager carries to the main thread; `CreateJob` now
  waits for all threads before it throws, so that an exception of a worker thread is not lost either.
- The deferral of RG9.11.14 is removed: connectors on the markers of a tree are computed in parallel again.
- No counter in CData and no switch for the numerical differentiation: a perturbed coordinate fails the comparison.
- The object's own `ComputeMassMatrixAndODE2LHS` keeps its recursion: it needs the placements in the parents and the
  bias accelerations with gravity, not the frames in the base and the accelerations of q_tt, and it runs once per tree
  and evaluation, so the cache would save at most one of its passes.

## 4. Measured

RK44, 2000 steps, a chain of revolute links with a CartesianSpringDamper from every link to the ground (section 1),
before / after: 6 links 0.131 / 0.052 s, 20 links 1.22 / 0.265 s, 60 links 11.0 / 2.13 s; the tree alone 0.026, 0.148,
1.38 s. The markers now cost about as much as the tree, not 5 to 8 times. Generalized-alpha with full Newton,
200 steps, 20 links: 664 iterations, 0.165 s with one thread, 0.126 s with four, the connectors in parallel.

## 5. Sub-steps

1. Thread buffers: not done, decided against (section 3).
2. The cache with its comparison (C), used by the markers, sensors and output variables - done.
3. `CSystem::PreComputeItems()` with `CObject::HasPreCompute()`, `CObject::PreCompute()` and the list (D), containing
   `UpdateUserFunctionsForThreads()` - done.
4. The object's own LHS and mass matrix on the cache: evaluated, no gain (section 3).
