# Symbolic system models: design (revision2026c step RG12.51.1, #3109)

From a model built of items with symbolic user functions, the equations of motion of the whole system as symbolic
programs: to export them to other applications, to evaluate them on a GPU or a specific CPU, and to check them with a
solver written in Python. This note collects what exists, the design, the options and the decisions needed. It lives
while RG12.51 is open (WORKFLOW.md §8.4).

## 1. Goals, and what follows from them

| goal | needs | does not need |
|---|---|---|
| G1 export of the equations of motion to other applications (C, C++, Python, sympy) | the residuals, the mass matrix, the constraints, their Jacobians as code; the sparsity pattern | a solver |
| G2 evaluation on a GPU or a specific CPU, first with Numba | the residuals per object, batched over the objects of one kind; explicit integration | Jacobians, constraints (first stage) |
| G3 a check of G1/G2 against Exudyn | an implicit solver in Python: generalized-alpha, with the settings of `SimulationSettings.timeIntegration` | speed |

The three goals share one representation: **per object a flat program** - already what a recorded user function is -
and **per system the maps from the local to the global coordinates**. The global system is never one big expression:
it is the objects' programs plus the scatter of their results, which is what Exudyn's own assembly does.

## 2. What exists (2026-10-26)

- **Flat programs.** A recorded user function has a flat program (`UserFunction.GetFlatProgramData()`, entry k =
  node k, operands by number, the arguments, variables and results); `esym.GetFlatProgram(expressions, ...)` gives one
  for expressions (#3087). `exudyn/misc/symbolicExport.py` writes it as Python, `symbolic`, `numpy` or `sympy` code and
  as C++ text (`Write`, `ToFunction`, `ToSympy`, #3088); 0.7 M entries per second in Python, 4 M for the C++ text.
- **Optimize** merges equal subexpressions and applies the exact simplification rules (#3091).
- **One recording for many objects.** `CreateSymbolicUserFunction` with `parameters` (`ParameterLayout`) records the
  force of a user body once for all bodies of the kind; the bodies differ in their parameters and their coordinates.
- **The local-to-global maps**: `mbs.systemData.GetObjectLTGODE2/ODE1/AE/Data(objectNumber)` and the node versions.
- **The solver API in Python**: `exu.MainSolverImplicitSecondOrder` and `MainSolverStatic` have `InitializeSolver`,
  `ComputeMassMatrix`, `ComputeODE2RHS`, `ComputeAlgebraicEquations`, `ComputeJacobianODE2RHS`, `ComputeJacobianAE`,
  `GetSystemMassMatrix`, `GetSystemJacobian`, `ComputeNewtonResidual`, `ComputeNewtonJacobian`, and for the output
  `WriteSolutionFileHeader`, `WriteCoordinatesToFile` and `FinishStep`, which writes the solution and the sensors and
  updates the renderer; used by `python/TestModels/manualExplicitIntegrator.py` and `sparseSystemMatricesTest.py`.
  These give the reference for every term of the symbolic model at any state.
- **Derivatives.** `SReal.Diff(var)` is the forward derivative by recursion over the tree (`ExpressionBase::Diff` in
  `src/Linalg/Symbolic.h`), evaluated at the current values: one number per call, no expression, and on a graph with
  shared subexpressions every path is walked again. It is not part of the flat program. **Only scalar expressions and
  scalar named variables have it**: the vector and matrix expressions have no derivative, a component of one raises
  ("operator[]::Diff not implemented"), and a recording cannot be differentiated by a component of its vector
  arguments - section 11.
- **Sparse matrices** inside Exudyn: `MatrixContainer` with triplets, faster than dense from about n > 50. scipy is an
  optional dependency of Exudyn (rule 6), and needed by `exudyn.misc.symbolicSystem`.

## 3. The system model

`SymbolicSystem(mbs)`, after `mbs.Assemble()`, in `exudyn/misc/symbolicSystem.py` (#3110, #3113):

- **Extracted from mbs**, and only if every item is compatible with the symbolic system: an item that is not - a
  built-in body, a Python user function, a recording that reads mbs (an item output), ODE1 coordinates or algebraic
  equations of other items than those of the table below, data coordinates, a load other than those of the table - is refused, with the list of these items. No item is
  approximated or skipped.
- **The data of mbs are reused**: the LTG maps, the reference and initial coordinates, the parameters of the items; the
  system adds nothing that mbs already knows.
- **Only the solve is new.** A computed state is written back into mbs by `SetSystemState(q, q_t, q_tt, t,
  configuration)`: in the Current configuration the sensors and the output functions read it, in the Visualization
  configuration the renderer draws it. A solver of this module writes its output so, through an Exudyn solver object
  that does nothing else: `InitializeSolver` and `WriteSolutionFileHeader` once, and per written step the state and
  `FinishStep`, which writes the solution file and the sensors and updates the renderer, by the settings of
  `simulationSettings.solution` - the SolutionViewer, `PlotSensor` and the stored sensor values then work as after
  `SolveDynamic`.
- **Evaluated by exudyn.symbolic only**: each recording by its `Evaluate`. A conversion of the whole system to numpy, C
  or other formats is an export (G1, RG12.51.5): the flat programs of the kinds written by `symbolicExport`, with the
  LTG tables and parameters as data - another class derived from `SymbolicSystemBase` (section 9), not an option of
  `SymbolicSystem`.

1. **The global coordinates** $\qv$, $\dot\qv$, $\ddot\qv$ (ODE2), $\yv$ (ODE1), $\lambdav$ (AE), $t$, of the sizes of
   `systemData`.
2. **Kinds.** The objects are grouped by their recording: the objects whose user functions are the same recorded
   function form one kind, with a table of their parameters and a table of their LTG indices (n_objects x n_local).
   A kind has one flat program per user function.
3. **The terms**, each as (kind, program, scatter):
   - the residual of the ODE2 equations, per object $\rv_e(\qv_e, \dot\qv_e, \ddot\qv_e, t) = \Mm_e(\qv_e)\,\ddot\qv_e -
     \fv_e(\qv_e, \dot\qv_e, t)$, scattered by the LTG into $\rv$;
   - the mass matrix $\Mm_e$ as local triplets (i, j, program), scattered into a sparse global matrix;
   - the constraints $\gv_k(\qv, t)$ of the joints, and the term $\Cm_q\tp \lambdav$;
   - the loads, as forces through the Jacobians of their markers (#3118).
4. **Sparse throughout.** The global matrices exist only as triplets (row, column, kind, object, local entry); the
   pattern is known from the LTG maps before any evaluation. Dense matrices are formed only on request, for small
   systems and tests.

The residual form is the basic term, as the GPU stage asks for: an object computes $\rv_e$ from its accelerations, and
the mass matrix is then $\partial \rv_e / \partial \ddot\qv_e$ - for G1 it is exported separately, as Exudyn's solvers
use it.

**What can be assembled:**

| item | how |
|---|---|
| ObjectGenericODE2 with constant matrices and symbolic mass and force functions | directly (#3110) |
| ObjectUserBody | its mass matrix and force functions in its coordinates (#3110); the positions of points by its positionUserFunction |
| ObjectConnectorUser with a symbolic forceUserFunction of its marker states (#3119) on position markers of point nodes and of the ground | directly (#3121): the marker states from the node coordinates or the fixed positions, the forces scattered to the coordinates of the markers, the Jacobian by `EvaluateJacobian` by the positions and velocities |
| ObjectConnectorUser on MarkerBodyPosition and MarkerBodyRigid of user bodies and of the ground, with the rotations if it requests Orientation | directly (#3129): the marker states by the recorded position, rotation and Jacobian functions of the body - $\vv = \Jm_{pos}\dot\qv$, $\tomega = \Jm_{rot}\dot\qv$, the rotation of the marker after the one of the body -, the forces and torques by $\Jm_{pos}\tp\fv + \Jm_{rot}\tp\mv$; the Jacobian by `EvaluateJacobian` of the force by the marker states, composed with the derivatives of the states ($\Jm_{pos}$, $\partial\Jm_{pos}/\partial\qv\,\dot\qv$, $\partial\Rot/\partial\qv$, $\Jm_{rot}$, $\partial\Jm_{rot}/\partial\qv\,\dot\qv$), and the derivatives of the Jacobians times the forces |
| ObjectGenericODE1 with its systemMatrix, rhsVector and a symbolic rhsUserFunction; LoadCoordinate on its coordinates | directly (#3144): `ODE1RightHandSide` $\Am\yv + \fv + \fv_{user}(\yv, t)$ per object as CObjectGenericODE1, the objects with the same recording one kind, the loads added; `ODE1Jacobian` exact, $\Am$ and `EvaluateJacobian` of the recording |
| ObjectJointUser, MarkerBodyUser | the constraint is symbolic; the marker kinematics as above, the constraints and $\Cm_{\qv}\tp\tlambda$ (RG12.51.4) |
| LoadForceVector, LoadTorqueVector, LoadCoordinate, LoadMassProportional | directly (#3118): the value constant or by its symbolic user function of the time; on MarkerNodePosition of a point node and on MarkerNodeCoordinate the value at the LTG indices of the marker; on MarkerBodyPosition, MarkerBodyRigid and MarkerBodyMass of an ObjectUserBody $\Jm\tp \vv$ by its symbolic positionJacobian-, rotationJacobian- or massIntegralUserFunction, for `bodyFixed` $\vv$ rotated by its rotationMatrixUserFunction, as CSystem applies loads; the Jacobian by `EvaluateJacobian` of these functions; loads on the ground act on nothing |
| built-in items, Python (not symbolic) user functions, recordings that read mbs | refused, with the list of items; the built-in items need recordings of their own (the AD templates, #3048) - not proposed |

## 4. Evaluation: explicit on the GPU, Numba first

**On the GPU**: one compiled function per kind (Numba), or the whole system by levels as uniform array operations -
the design is [uniformEvaluation.md](uniformEvaluation.md) (group RG14).

**Explicit integration without a mass matrix.** The first GPU stage needs no Jacobians and no constraints, but an
explicit method still needs $\ddot\qv$ from $\Mm\ddot\qv = \fv$. From residuals only, three ways:

| way | cost | when |
|---|---|---|
| a constant mass matrix, factored once (block diagonal for rigid bodies, or lumped) | one solve per stage | constant or lumped mass matrices - most FE and rigid-body models |
| matrix-free: $\Mm\av = \rv(\qv, \dot\qv, \av) - \rv(\qv, \dot\qv, \mathbf{0})$, solved by conjugate gradients | several residual evaluations per stage | a variable SPD mass matrix |
| the local mass matrices, solved per object | small dense solves | objects without shared coordinates |

The first is the default; the matrix-free product keeps "residuals only" exact for the others. The explicit
integrators of scipy (`solve_ivp`) can drive the Numba functions for a check (G3); on the GPU a fixed-step
Runge-Kutta in the kernel loop.

## 5. Jacobians: by differentiating the programs

An implicit solver, the export of the Newton matrix (G1), and the connectors and joints all need derivatives. The
options:

| option | how | cost | what it gives |
|---|---|---|---|
| A central differences | 2n evaluations of the residual | approximate | the default of `SymbolicSystemBase` for a system without derivatives of its own; never for a constraint (#3101) |
| B `Diff` per q[i] and q_t[i] | n calls per output, each a recursion over the tree | slow on shared graphs; needs #3115 | numbers, no program |
| C forward mode over the flat program | per slot a value and n tangents | memory: slots x n, the local memory of the program (#3116) | numbers for all n directions in one pass |
| D symbolic differentiation into new nodes | per node class a derivative from its operands' derivatives, then Optimize | once, at setup | **a flat program of the Jacobian**, exportable like any other |

D is what G1 and G2 need: the Jacobian becomes code, and Optimize removes the duplicates that the product and chain
rules create. Every node class gets a function `DiffFromOperands(operands, operandDerivatives)` that returns new nodes,
next to `NewWithOperands`; differentiating a program with respect to its n inputs is one pass in topological order with
n derivative slots per node, most of them zero - kept sparse, so the cost follows the real dependencies. C is the
runtime form inside C++ for the item Jacobians of RG7.17. D is the subject of RG12.54 (#3108), C of RG12.59 (#3116);
B, which both need for their check, of RG12.58 (#3115).

The marker Jacobians of the joints and connectors (section 3) are derivatives of the marker position and rotation
programs by D - the reason the earlier plan needed sympy.

**Now (#3115):** `SymbolicSystem.ResidualJacobian` is B: object by object `UserFunction.EvaluateJacobian` of the force
and mass matrix recordings by their arguments q and q_t (section 11, S1), $\partial(\Mm\ddot\qv)/\partial\qv$ from the
derivative of the mass matrix, and the constant matrices of ObjectGenericODE2; equal to the central differences of
`SymbolicSystemBase` (A) to their error, 6e-9. A, in the base class, stays the Jacobian of a system without derivatives
of its own and the comparison.

## 6. The implicit solver in Python (G3)

Generalized-alpha (Chung and Hulbert, in the form of Arnold and Brüls that Exudyn uses), index 3 with the constraints
on position level, Newton with the Jacobian

$$\Jm = \beta' \Mm + \gamma' \Dm + \Km, \quad \beta' = \frac{1 - \alpha_m}{h^2 \beta (1 - \alpha_f)}, \quad
\gamma' = \frac{\gamma}{h \beta}$$

with $\Km = \partial \rv / \partial \qv$ and $\Dm = \partial \rv / \partial \dot\qv$, extended by $\Cm_q$ and $\Cm_q\tp$.
It reads `SimulationSettings.timeIntegration`: `numberOfSteps`, `endTime`, `generalizedAlpha.spectralRadius`,
`generalizedAlpha.computeInitialAccelerations`, `newton.relativeTolerance`, `newton.absoluteTolerance`,
`newton.maxIterations`, `newton.useModifiedNewton` - so that the same settings run in Exudyn and in Python. Sparse
linear algebra by scipy.sparse.

**Done** (#3126): `SolveGeneralizedAlpha(system, simulationSettings)` for systems without constraints: the predictor,
the Newton iteration and the update of the algorithmic accelerations of `CSolverImplicitSecondOrder.cpp`, the
Jacobian by `ResidualJacobian(q, q_t, q_tt, t, 1, gamma', beta')` - exact for a SymbolicSystem, with the term
d(M q_tt)/dq that Exudyn leaves out, which changes the iterates, not the solution -, sparse LU by scipy, the
convergence as `CSolverBase::Newton` (relative to the initial residual or absolute, weighted per coordinate), a
constant step size: a Newton iteration that does not converge raises. symbolicGeneralizedAlphaTest.py: equal to
`SolveDynamic` to 1e-10 relative with relativeTolerance 1e-10. The constraints and the index-2 form come with
RG12.51.4.

**The check** in two steps, each a test model:

1. Terms: at random states, $\Mm$, $\rv$, $\gv$ and the Jacobians of the symbolic model against
   `ComputeMassMatrix`, `ComputeODE2RHS`, `ComputeAlgebraicEquations`, `ComputeJacobianODE2RHS`, `ComputeJacobianAE`
   of `MainSolverImplicitSecondOrder` - equal to rounding.
2. Time integration: the Python solver against `mbs.SolveDynamic` with the same settings - the same algorithm, so equal
   to the Newton tolerance.

**Where it goes.** The maintainer suggested `systemUtilities.py`. That module holds the functions that describe a
system; the functions that need a solver - `ComputeLinearizedSystem`, `ComputeODE2Eigenvalues` - are in `solver.py`
(docs/dev/internals/systemUtilities.md §1). The solver works on a `SymbolicSystemBase`, not on `mbs`, so the model and
its solver are together in `exudyn/misc/symbolicSystem.py` (decision 3).

### ODE1 and algebraic coordinates (#3134)

The interface has the three kinds of coordinates of Exudyn, so that the system and its solvers have no gap when items
with ODE1 coordinates and constraints come, and an item may couple all kinds:

$$
\Mm(\qv, t)\,\ddot\qv = \fv(\qv, \dot\qv, t, \yv) - \Cm_{\qv}\tp\tlambda, \qquad
\dot\yv = \gv(\qv, \dot\qv, \yv, t), \qquad \cv(\qv, \dot\qv, \yv, \tlambda, t) = \Null .
$$

`SymbolicSystemBase`: `nODE1`, `nAE`, `InitialStateODE1()`, `Forces(q, q_t, t, y)`, `ODE1RightHandSide(q, q_t, y,
t)`, `AlgebraicEquations(q, q_t, y, lambda_, t, velocityLevel)`, `ReactionForces(q, q_t, y, lambda_, t)` ($\Cm_{\qv}\tp
\tlambda$); `SystemResidual` is the residual of Exudyn's implicit solver, $[\Mm\ddot\qv - \fv + \Cm_{\qv}\tp\tlambda,\;
\gv - \dot\yv,\; \cv]$, and `SystemJacobian` its Jacobian by columns of the Newton unknowns - the ODE2 block by
`ResidualJacobian` (exact for a SymbolicSystem), the rest by central differences. The solvers as
CSolverExplicit and CSolverImplicitSecondOrder:

- `SolveExplicit`: the ODE1 coordinates in the stages of explicit Euler and RK44, their velocities zero at the start;
  algebraic equations refused (Exudyn's explicit solver eliminates only constraints of coordinates on the ground),
  velocity Verlet refuses ODE1 coordinates.
- `SolveGeneralizedAlpha`: the ODE1 coordinates by the trapezoidal rule ($\yv \mathrel{+}= \frac{h}{2}\dot\yv$ in the
  predictor, $\dot\yv = 0$, the update $\yv \mathrel{-}= \Delta$, $\dot\yv \mathrel{-}= \frac{2}{h}\Delta$), their
  velocities zero at the start as in Exudyn; index-3 constraints with the scaling of Exudyn (ODE2 rows times $\beta
  h^2$, algebraic columns divided by it, $\tlambda \mathrel{-}= \Delta/(\beta h^2)$); the initial accelerations of
  $[\Mm\;\Cm_{\qv}\tp; \Cm_{\qv}\;\Null]$ with $-(\Cm_{\qv}\dot\qv)_{\qv}\dot\qv$, the multipliers kept with
  `storeInitialAlgebraicCoordinates`; the convergence over all equations. Without ODE1 and algebraic coordinates the
  path of #3126 is unchanged (sparse). `useIndex2Constraints` is refused.
- `NumpySystem` takes `ode1RightHandSide`, `initialStateODE1`, `constraints`, `constraintJacobian`.
- `SymbolicSystem` takes ObjectJointUser with symbolic constraint and Jacobian functions (#3141): `AlgebraicEquations`
  from the constraint recording with the marker states, `ReactionForces` = $\Cm_{\qv}\tp\tlambda$ from the Jacobian
  recording (by the marker translations and global rotation increments), mapped into the coordinates by the position
  and rotation Jacobians of the markers as the forces of a connector. `SystemJacobian` of `SymbolicSystem` is exact
  (#3145): $\Cm_{\qv}$ is the Jacobian recording times the marker Jacobians, the derivative of
  $\Cm_{\qv}\tp\tlambda$ by $\qv$ comes from the one-pass derivatives of the Jacobian recording by the marker
  positions and rotation matrices and of the body functions, as for the connectors; the ODE1 block is `ODE1Jacobian`.
  For the 33 unknowns of symbolicSystemJointsTest.py `SolveGeneralizedAlpha` takes 0.44 s against 0.059 s of
  SolveDynamic (4.2 s with central differences). Both stop at other Newton iterates - 1.1e-8 apart after 100 steps at
  relativeTolerance 1e-10 - also with the exact blocks and also without the derivative of the reaction forces: the
  difference is in the ODE2 block, where Exudyn leaves out $\partial(\Mm\ddot\qv)/\partial\qv$.
- `SymbolicSystem` takes ObjectGenericODE1 with a symbolic rhsUserFunction and LoadCoordinate on its coordinates
  (#3144). `Equations(algebraicEquations=True, reactionForces=True)` gives the constraint equations of the joints and
  their reaction forces as expressions of q, t and the variable `lambda_`, `GetFlatProgram(results=[...])` writes them
  (#3146); the ODE1 equations are not in `Equations()` yet. The coupling of all kinds in one
  item, ObjectGenericDAE (#3143), also needs Exudyn's internal solvers extended.

## 7. Sub-steps of RG12.51 (as planned after the decisions)

- RG12.51.2 (done, #3110): `SymbolicSystem(mbs)` for ObjectGenericODE2 and ObjectUserBody: kinds, LTG tables, mass
  matrix as triplets or sparse matrix, forces and residual; equal to the solver API.
- RG12.51.7 (done, #3113): `SymbolicSystemBase`, exudyn.symbolic only, `SetSystemState`, `InitialState`,
  `IsMassMatrixConstant`, scipy at the start of the module.
- RG12.56 (done, #3112): explicit Euler, velocity Verlet and RK44 in Python, on the base class (sections 9 and 10),
  `NumpySystem`; RG12.56.4 (done, #3120) the inverse per body of ObjectUserBody in Exudyn.
- RG12.51.9 (done, #3121): ObjectConnectorUser with a symbolic forceUserFunction on position markers of point nodes
  and of the ground.
- RG12.51.3 (done, #3126): `SolveGeneralizedAlpha`, the generalized-alpha and Newmark methods in Python; the
  time-integration check against `SolveDynamic`.
- RG12.55 first: user items with the interfaces of the built-in items (`exudyn/misc/symbolicUserItems.py`).
- RG12.51.8 (done, #3118): the built-in loads, constant or with a symbolic user function.
- RG12.51.4: those user items in the system - rigid bodies, joints, connectors, the constraints; RG12.51.4.1 (done,
  #3129) the connectors on markers of user bodies and of the ground, with the rotations, so that the rigid bodies and
  torsional spring-dampers of UserItemCreator are taken; RG12.51.4.2 (done, #3141) the joints.
- RG12.51.14 ObjectGenericODE1 (#3144), RG12.51.15 the exact constraint blocks (#3145), RG12.51.16 the constraints in
  `Equations()` (#3146) - section 12.
- RG12.51.5: the export (G1): the programs, the sparsity pattern and the scatter as C++ and as Python, and a system
  class of the exported numpy code.
- RG12.51.6: the evaluation by Numba for the GPU - moved to group RG14, [uniformEvaluation.md](uniformEvaluation.md).

And in exudyn.symbolic (section 11): RG12.57 cMath with symbolic arguments (done, #3114), RG12.58 `Diff` of vectors
and matrices (done, #3115), RG12.59 the forward derivative of a flat program (#3116), RG12.60 a recording called with
symbolic arguments (#3117). The evaluation of a whole step on the GPU by levels: [uniformEvaluation.md](uniformEvaluation.md).

## 8. Decisions (maintainer 2026-10-26)

1. Both available: the residual for the implicit integrators, which need only it, and the (sparse) mass matrix, needed
   e.g. to invert it.
2. exudyn.symbolic first; numpy and other formats by the export of the whole system; Numba for the GPU.
3. `exudyn/misc/symbolicSystem.py` for model and solver, for now: special, not for the broad use, of large potential.
4. Option D is the target of RG12.54; before it a Jacobian without a new program, column by column by `Diff`, which
   stays as an option and as the comparison.
5. The system is extracted from mbs and reuses its data; a computed state goes back into mbs for the visualization,
   the sensors and the outputs - only the solve is new.
6. What exudyn.symbolic lacks is added to exudyn.symbolic, not worked around in `SymbolicSystem`.
7. A base class `SymbolicSystemBase`, on which the solvers work without distinguishing the kinds of systems.

## 9. One interface for every solver: SymbolicSystemBase

The solvers of RG12.56 and RG12.51.3 work on the **equations**, not on `mbs` and not on the recordings, so that the
same solver runs a SymbolicSystem, later an exported one (G1), or a system written by hand - the simplest check of a
solver, independent of Exudyn.

The interface is the base class `SymbolicSystemBase` in `exudyn/misc/symbolicSystem.py` (#3113):

| member | what | in the base class |
|---|---|---|
| `nODE2` | the number of ODE2 coordinates | set by the derived class |
| `InitialState()` | `(q0, q_t0)`; the start time is `timeIntegration.startTime` | zeros |
| `Forces(q, q_t, t)` | $\fv$, the right-hand side | computed by the derived class |
| `MassMatrixTriplets(q, q_t, t)` | $\Mm$ as (rows, columns, values) | computed by the derived class (or it overrides `MassMatrix`) |
| `MassMatrix(q, q_t, t, sparse)` | $\Mm$, dense or `scipy.sparse.csr_matrix` | from the triplets |
| `MassMatrixBlocks(q, q_t, t)` | the mass matrices of the bodies, a list of (LTG table, blocks) - for the inverse per body | one block of all coordinates |
| `IsMassMatrixConstant()` | True: the solver factors $\Mm$ once | False |
| `Residual(q, q_t, q_tt, t)` | $\rv = \Mm\ddot\qv - \fv$ | from MassMatrix and Forces |
| `ResidualJacobian(q, q_t, q_tt, t, factor_q, factor_q_t, factor_q_tt, sparse)` | the combined Jacobian | central differences of Residual |

`SymbolicSystem` derives from it: `InitialState()` from the initial configuration of mbs, `IsMassMatrixConstant()`
from `constantMassMatrix` of the user bodies and the constant matrix of ObjectGenericODE2, `Residual` object by object,
and `SetSystemState` for the output (section 3). The kind of evaluation is decided once, by the class of the system:
`SymbolicSystem` by exudyn.symbolic, the exported system (RG12.51.5) by its generated code, a system written by hand
(`NumpySystem(massMatrix, forces, initialState, massMatrixBlocks)`, a constant matrix or a function, and a function) by
its own functions. The solvers use only this interface; the explicit ones only `InitialState`, `Forces`, `MassMatrix`,
`MassMatrixBlocks` and `IsMassMatrixConstant`; `SymbolicSystem` gives one block per object, the kinds as stacks
(objects x n x n).

The result of a solver, the same for all: a dict with `'time'` (n_out), `'q'`, `'q_t'`, `'q_tt'` (n_out x nODE2), at
the steps of `solution.file.writePeriod`, and the statistics (steps, evaluations of the forces, factorizations); for
a `SymbolicSystem` also the solution file and the sensors of mbs (section 3).

## 10. Explicit solvers (RG12.56.1, #3112)

The first solvers, before generalized-alpha: they need no Jacobian and no constraints, so simple user items suffice -
mass points and spring-dampers (RG12.55.2; the spring-damper as ObjectConnectorUser with a symbolic force function,
#3119), and before them an ObjectGenericODE2 or user bodies with symbolic forces. Constant step size
$h = (t_{end} - t_{start})/n$ from `timeIntegration.startTime`, `endTime`, `numberOfSteps`; the method from
`timeIntegration.solverType` (or an argument, as `SolveDynamic(solverType=...)`). `SolveExplicit(system,
simulationSettings, solverType)` in `exudyn/misc/symbolicSystem.py` (#3112):

- **ExplicitEuler**: $\qv_{n+1} = \qv_n + h\,\dot\qv_n$, $\dot\qv_{n+1} = \dot\qv_n + h\,\av_n$, with
  $\av_n = \Mm^{-1}\fv(\qv_n, \dot\qv_n, t_n)$ - the first-order system $[\qv, \dot\qv]$ with the Butcher tableau of one
  stage, as `CSolverExplicit` does.
- **RK44**: the classical 4-stage tableau on $[\qv, \dot\qv]$, the stages as `src/Solver/CSolverExplicit.cpp`
  computes them: $K_i = (\dot\qv, \av)$ at $g_i = u + h \sum_j A_{ij} K_j$, $t + c_i h$, then $u + h \sum_i b_i K_i$.
- **VelocityVerlet**, as `CSolverExplicit` writes it: $\dot\qv_{n+1/2} = \dot\qv_n + \tfrac{h}{2}\av_n$,
  $\qv_{n+1} = \qv_n + h\,\dot\qv_{n+1/2}$, $\av_{n+1} = \Mm^{-1}\fv(\qv_{n+1}, \dot\qv_{n+1/2}, t_{n+1})$,
  $\dot\qv_{n+1} = \dot\qv_{n+1/2} + \tfrac{h}{2}\av_{n+1}$ - the force at the half-step velocity, as in Exudyn; the
  initial acceleration from the initial state.

**The accelerations: the inverse per body.** An explicit integrator is only worth its cost if $\av = \Mm^{-1}\fv$ is
cheap. Solving the system mass matrix in every stage - a sparse LU decomposition, or a dense one - costs as much as
everything else together, does not run in parallel, and is hard to do on a GPU at all. Exudyn's explicit solvers
therefore have `timeIntegration.explicit.computeMassMatrixInversePerBody`: each body inverts its own mass matrix, and
$\av$ is the product of the block-diagonal inverse with $\fv$ - correct as long as no two bodies share a coordinate,
which holds for the single-noded bodies (mass points, rigid bodies). The symbolic system has the blocks by
construction: per kind a stack of the local mass matrices (objects x n x n, `MassMatrixBlocks`) and its LTG table, so
the inverse is one batched inversion per kind (`numpy.linalg.inv` of the stack), and the product one batched
matrix-vector product - the operation a GPU does best, one thread or one warp per object. A constant mass matrix
(`IsMassMatrixConstant()`, `timeIntegration.reuseConstantMassMatrix`) is inverted once, so a step is then only forces
and products. With the setting, `SolveExplicit` checks at the start that the blocks share no coordinate and raises
otherwise; without it, the mass matrix is solved by `scipy.sparse.linalg.splu`, once if constant. On the GPU (G2) the
inverse per body is the only form: the global solve is not done there.

**The settings.** `explicit.computeEndOfStepAccelerations` as in Exudyn (one more evaluation per step, else the
accelerations of the last stage); `explicit.useLieGroupIntegration` with a Lie group node raises, as its formulas are
not implemented here; `automaticStepSize` has no meaning for these three methods, as in Exudyn. The output through
mbs (section 3) runs an Exudyn solver object as explicit Euler without the inverse per body: its initialization
computes and decomposes the mass matrix once by Exudyn, which only costs time.

**The check** (symbolicExplicitSolverTest.py): a net of 4 x 4 mass points in a vertical plane, the two upper corners
fixed, neighbours connected by spring-dampers, under gravity and with an initial velocity out of the plane - the mass
points ObjectUserBody, the spring-dampers ObjectConnectorUser with the forceUserFunction of the marker states (#3119,
#3121), both recorded, written with `cMath.sqrt` (#3114). Each of the three methods, with and without the inverse per
body (#3120), against `mbs.SolveDynamic` with the same solver type, setting and 200 steps: 2.6e-14 relative - the same
formulas; the sensor written through mbs 7e-16; the mass matrix inverted in every stage instead of once equal; the user
items against MassPoint and ObjectConnectorSpringDamper 1e-16; the same net as `NumpySystem` 8e-17; the Jacobian of the
residual by `Diff` against central differences 7e-11 relative to the stiffness. For the record: 200 steps of RK44 take
0.29 s in Python on the symbolic system, 0.12 s as `NumpySystem`, 0.02 s in `SolveDynamic`. The earlier check with
a mass matrix depending on the coordinates is in symbolicSystemTest.py for the terms.

**Sub-steps of RG12.56**: .1 this concept; .2 `NumpySystem` on `SymbolicSystemBase` (the base class itself is
RG12.51.7); .3 the three explicit solvers, their output through mbs, and their test model; .4 the inverse per body of
ObjectUserBody in Exudyn (done, #3120: the user body inverts its local mass matrix).

## 11. What exudyn.symbolic lacks for the system, and the proposals

Building the system met four gaps. Each is a step of its own in exudyn.symbolic; `SymbolicSystem` does not work
around them. S1 and S4 are done, S2 and S3 open.

**S1 `Diff` of vectors and matrices, by the components of vector variables (RG12.58, #3115, done).** `Diff(var)`
existed for scalar expressions and scalar named variables only; a recording has vector and matrix arguments, `q[i]`
was an `operator[]` node without derivative, and the vector and matrix operations had none. As implemented:

- **A seed** `(variable, component)`: a scalar named variable, or component i of a named vector (i = row x columns +
  column of a named matrix). `ExpressionBase::Diff(const DiffSeed&)` replaces `Diff(ExpressionNamedReal*)`; a named
  vector returns the unit vector $\ev_i$ for its own seed, zero otherwise.
- **Vector and matrix nodes get `DiffRef(seed)`**, which returns a reference to a derivative buffer of the node - a
  member next to the evaluation buffer that `EvaluateRef` fills (#3006), sized at the first call and kept. So no
  memory is allocated after the first call and no heap of its own is needed: the memory per node is one more vector
  or matrix of its size. Each operation gets its rule from the values and derivatives of its operands - sum, scalar
  times vector, matrix times vector ($\dot\Am \vv + \Am \dot\vv$), transposed, component product, cross product -, and
  the scalar results of vectors: `v[i]` gives component i of the operand's derivative, `dot` $\dot\uv \cdot \vv +
  \uv \cdot \dot\vv$, `NormL2` $(\vv \cdot \dot\vv) / \|\vv\|$. `IfThenElse` takes the derivative of the branch that
  the condition selects, as for scalars; item outputs raise.
- **In Python**: `Real.Diff(var)` as before, and `Real.Diff(vectorVariable, component)`; `Vector.Diff(var)`,
  `Vector.Diff(vectorVariable, component)`, `Matrix.Diff(var)` and `Matrix.Diff(vectorVariable, component)` return
  numpy arrays; `UserFunction.EvaluateJacobian(argumentName, *arguments)` returns the Jacobian of the result of a
  recording by one of its arguments at the given arguments, column by column: a vector for a scalar result, a matrix
  (result x argument) for a vector result, an array (rows x columns x argument) for a matrix result. A matrix
  variable as seed exists in C++; Python has no named matrix variable to pass yet.
- **What it gives**: the Jacobian of the residual by `Diff` (option B) without any wrapper (section 5), the marker
  Jacobians of the joints and connectors as numbers, and the comparison for C and D. The test,
  symbolicVectorDiffTest.py: 18 vector and matrix expressions by a scalar and by the components of a vector, against
  central differences, 1.9e-10; EvaluateJacobian of a force and a mass matrix recording, 1.1e-10; the refusals.

**S2 the forward derivative of a flat program in one pass (RG12.59, #3116, done).** S1 walks the tree once per seed, n times
for n coordinates. Over the flat program, each slot keeps its value and its n derivatives, computed in the order of the
program from those of its operands - one pass. The memory is slots x n numbers (x the size of a vector or matrix slot):
**one block per program, sized at the first evaluation from the program and kept**, a local memory of the program - so
no allocation per call, and no setting needed, since the size is known from the program and n. In the parallel
evaluation each thread has its block, as each has its values now (#2978). A setting for a fixed maximum size only if a
bound must be known in advance (a GPU kernel); an allocation per call would be simpler but stays in the hot path and is
harder to remove later - not proposed. It is the runtime Jacobian of the symbolic user functions in C++: the position
and rotation Jacobians of NodeUser, ObjectUserBody and MarkerBodyUser, the connector, and a joint without
jacobianUserFunction (RG7.17). After S1, which is its check.
**As implemented**: `FlatProgram::Differentiate(variable, n, scalarVariable)` after `Evaluate()`: the derivatives of every
slot, component after component with n columns, in one block of the program (`derivatives`, offsets per slot from the
sizes of this evaluation), which grows once and is kept; each operation by a rule found once per program from its
`FlatName()` and the kinds of its operands - the arithmetic, the functions sin to atanh, pow (an exponent constant in a
column without the log term, so also for a base <= 0, as the tree's Diff, #3135), atan2, IfThenElse by its condition, the comparisons and
other piecewise constant functions as zero, the dot product, the norm, the components, the vector and matrix
operations; a node without rule (abs, min, max, mod, an item output) by its Diff or DiffRef per component, as before.
`UserFunction.EvaluateJacobian` uses it with the flat evaluation, the tree with `flatEvaluation = False`. Measured on
the rigid body of symbolicUserRigidBodiesTest.py: the Jacobian of the force by qTotal (6 columns) 454 us by the tree,
6.1 us in one pass; of the mass matrix 243 us and 6.6 us; of a connector force by a rotation matrix 29 us and 4.5 us.
With it, `SymbolicSystem.ResidualJacobian` takes the Jacobian of the two recordings of the composed system (up to 200
coordinates; per object above): SolveGeneralizedAlpha on the chain of three rigid bodies, 50 steps, 0.96 s -> 0.027 s,
SolveDynamic with the same recordings 0.039 s.

**S3 a recording called with symbolic arguments (RG12.60, #3117).** A recording evaluates with numbers only. Called
with exudyn.symbolic arguments, it returns a copy of its expressions in which its argument variables are replaced by
the given expressions - the copy that `Optimize` makes by `NewWithOperands`, with the argument nodes mapped. Needed for
one expression per kind ($\Mm(\qv)\,\ddot\qv - \fv(\qv, \dot\qv, t)$ of the two recordings), for the forces of a
connector composed with the marker kinematics of its bodies, and for the export of one program per kind (RG12.51.5).
As numbers, the system needs no S3: it evaluates each recording and composes the results; S3 is for the programs.
**In Python** (#3130): the code of a recording in the format 'symbolic' of exudyn.misc.symbolicExport (`ToFunction`),
called with exudyn.symbolic arguments, records the same graph with the arguments replaced - S3 without C++, at the cost
of recording the graph again; in the format 'sympy' the same with sympy symbols. `SymbolicSystem.Equations(format)`
composes the system of it: the mass matrix, the forces of $\Mm\ddot\qv = \fv$ and, with `massMatrixBlocks`, the
mass matrices of the bodies for the inverse per body, in the variables q, q_t and t (the parameters and reference
coordinates as numbers), with the connectors on their marker kinematics and the loads; `GetFlatProgram(format)` writes
the results asked for (#3132). The forces are the same with and without the inverse per body; the residual is not
offered, as neither solver uses it in this form. Measured: 9 coordinates of a rigid body and a mass point, 474 operations,
composed in 0.01 s (exudyn.symbolic) and 0.15 s (sympy). Whether RG12.60 still needs the copy in C++ by `NewWithOperands`
is open.

**The composed system as two recordings** (#3131). Evaluated object by object, one force evaluation of a chain of 3
rigid bodies with 18 connectors and 4 loads made about 120 calls of `Evaluate`, each about 3 us of call overhead and
Python around it: 0.59 ms, while the arithmetic is a few microseconds. The forces of `Equations()` and the nonzero entries
of its mass matrix, recorded once as functions (mbs, t, itemNumber, q, q_t) of the signature of ObjectGenericODE2
(0.03 s), evaluate by one flat program each: forces 14 us (1591 operations), mass matrix 4.5 us; Exudyn's
`ComputeODE2RHS` with the same user items takes 88 us. `SymbolicSystem(mbs, composeSystem=True)`, the default, uses them
in Forces, MassMatrixTriplets and Residual; MassMatrixBlocks and ResidualJacobian stay per object (the Jacobian of the
composed recording would walk its tree per seed - RG12.59). The mass matrix of up to 200 coordinates is decomposed dense
by scipy.linalg: forming the sparse matrix cost more than the evaluation.

**S4 cMath with symbolic arguments (RG12.57, #3114, done).** `np.sin` of an `esym.Real` records - numpy calls the
method `sin()` of the object -, and the math check of the test models refuses `np.sin` (#2964). The functions of
`exudyn.cMath` take an `esym.Real` as well, by an overload of the compiled function that calls the one of
exudyn.symbolic - all but `cbrt`, `atan2` and `pow` also mixed with a float -, so that one function written with cMath
runs on numbers and is recorded; a float costs nothing more, as its overload comes first.

## 12. Status, analysis and options (2026-10-28)

The symbolic user items and the symbolic system after RG12.51.4.2: what they are, what is open, and the questions of
the maintainer of 2026-10-28 with options and a recommendation each. Nothing of 12.2 to 12.6 is a step yet; the
maintainer decides which become one. What clearly has to be done got a step and an issue: the ODE1 items (RG12.51.14,
#3144), the exact constraint blocks (RG12.51.15, #3145), the constraints in `Equations()` (RG12.51.16, #3146), the
drawing of the joints (RG12.55.7, #3147), the node count (RG12.61, #3142) and ObjectGenericDAE (RG9.10, #3143).

### 12.1 Status

| part | done | open |
|---|---|---|
| user items (`UserItemCreator`) | ground, mass point, rigid body, spring-damper, torsional spring-damper, force, revolute, prismatic, spherical and distance joint - equal to the built-in items to the Newton tolerance | the sphere-sphere contact (RG12.55.5); the joints are not drawn (#3147); free axes of the spherical joint and nodes in the distance constraint are refused |
| `SymbolicSystem` | ObjectUserBody, ObjectGenericODE2, ObjectConnectorUser on point, body and ground markers, ObjectJointUser, ObjectGenericODE1 (#3144), the four loads; two composed recordings (forces, mass matrix); the constraints and reaction forces in `Equations()` (#3146) | data coordinates, the ODE1 equations in `Equations()` |
| solvers in Python | explicit Euler, RK44, velocity Verlet (with the inverse per body), generalized-alpha with ODE1 coordinates and index-3 constraints, all equal to `SolveDynamic` | with joints 7 times slower than `SolveDynamic` with the exact Jacobian (#3145); index 2 refused |
| derivatives | `Diff` of vectors and matrices (#3115), the one pass over a flat program (#3116), pow without the log term (#3135) | the derivative as a new program (RG12.54.2, #3108) |
| export | a recording as Python, numpy, symbolic, sympy and C++ text; `Equations()` and `GetFlatProgram()` of the system | the whole system with its sparsity and scatter (RG12.51.5) |
| GPU | the experiment of RG14.2.1 | RG14.2.2 to RG14.4 |

### 12.2 The system coordinates in exudyn.symbolic

**Now.** Every user function gets the coordinates of its own item - ObjectUserBody `qRef` and `qTotal` of its nodes, a
connector the states of its markers, ObjectGenericODE2 `q` and `q_t` of its nodes -, and `SymbolicSystem` composes them
by the LTG maps. A function that needs the coordinates of other items reads them as item outputs (#2984), which the
Jacobian of its item does not see.

| option | what | for | against |
|---|---|---|---|
| A as now | local arguments per item, composed by the LTG maps | the Jacobian of an item is local, its sparsity known; one recording per kind | a coupling across items needs a connector or an item output |
| B system coordinates as nodes that read mbs | `esym.SystemCoordinate('ODE2', i)`, read at evaluation as the item outputs | any coupling in one line | the dependency is hidden from the LTG map, the sparsity and the Jacobian - the lag of the item outputs, now for every coordinate; the global indices change when the model changes |
| C system variables for the recordings of a system | the named vectors `q`, `q_t`, `y`, `lambda`, `t` (and `p`, 12.4) of a system, and helpers that give the part of a node or item, e.g. `variables.ODE2(node)` | equations written directly in system coordinates - what `Equations()` does internally (`_Backend.Variable`); a controller or a coupling written by hand | for the system only, not inside an item of mbs |

**Recommendation**: C, in `exudyn.misc.symbolicSystem`, as the public form of what `Equations()` does already. Inside
mbs, an item that couples the coordinates of several nodes is ObjectGenericDAE (RG9.10): its node lists give the LTG
map, so the Jacobian sees the coupling. Not B. Data coordinates stay refused: they change in the post-Newton step
(contact, friction), which an equation of the system does not express.

### 12.3 q, qRef and qTotal in symbolicSystem.py

**What it is.** In `SymbolicSystemBase`, its solvers, `Equations()` and the composed recordings, `q` are the ODE2
coordinates of Exudyn's solvers: relative to the reference configuration, what `systemData.GetODE2Coordinates()`
returns and `SetSystemState` writes - the `q` of `solver.md` and of `MainSolver`. The reference values are
`SymbolicSystem.qRef`, read from mbs once. A user body gets `qRef` and `qTotal = qRef + q`, as ObjectUserBody defines
its arguments; ObjectGenericODE2 gets `q`; in `Equations()` the reference values are numbers.

**Is it right?** Yes for the solvers: the state is that of `SolveDynamic`, without conversion, and the total coordinates
are formed where the items need them, as in Exudyn. Three weaknesses: the base class does not say that `q` is relative
(the docstrings of `SymbolicSystem` say "displacements" in some places only); `NumpySystem` has no reference values, so
a solver cannot form `qTotal` for every system; the exported equations contain the reference configuration as numbers,
so a changed reference needs a new export.

**Recommendation**: keep `q` - the name of the solvers - and say it once in the base class: "q: the ODE2 coordinates
relative to the reference configuration, qTotal = qRef + q"; `qRef` a member of the base class (zero in
`NumpySystem`); the reference values an optional variable of `Equations()`, with the parameters of 12.4. Whether user
functions get total coordinates in general is RG13.2.

### 12.4 Further dependencies: the parameters

**Now.** The dependencies are q, q_t, y, lambda and t. The parameters of the items are an argument of the recordings
of ObjectUserBody, ObjectConnectorUser and ObjectJointUser (`parameters`, named by a ParameterLayout), so one recording
serves a kind; `SymbolicSystem` keeps them in the tables of the kinds, and `Equations()` writes them as numbers. The
values of the loads and the matrices of ObjectGenericODE2 are numbers as well. `esym.variables` are global named
variables, read by a recording at every call.

| option | what | gives |
|---|---|---|
| P1 a parameter vector p of the system | every parameter of the items and loads in one vector, with a table (item, name, index) to its slot; p an input of the composed recordings and a variable of `Equations()`; `SetParameters(p)` | the derivative by p by the one-pass derivative (the recordings take `parameters` as an argument already): sensitivities, optimization, identification; a parameterized model evaluated for another p without recording again, also as an ensemble over p (RG14) |
| P2 `esym.variables` | named variables, as now | a few global parameters (gravity, the stiffness of all springs); one global namespace, not per item |
| P3 as now | numbers | nothing more |

**Recommendation**: P1, after the free signature of 12.6 R2 - the composed recordings take the signature of
ObjectGenericODE2, `(mbs, t, itemNumber, q, q_t)`, which has no parameters. The parameters are the `parameters`
vectors, the load values and the matrices of ObjectGenericODE2; not the reference coordinates (12.3), which are an
option of their own.

### 12.5 The names of symbolicSystem.py

`q` stays (12.3). The others against Exudyn's names - Exudyn keeps the right-hand side and the left-hand side apart
for the sign (`notation.md`, *LHS-RHS naming conventions*): an object computes its LHS terms, the system its RHS
$\fv_{sys}$, with $\Mm\ddot\qv$ and the reaction forces on the LHS.

| now | what it computes | proposed | in Exudyn |
|---|---|---|---|
| `Forces(q, q_t, t, y)` | $\fv$, the RHS of the ODE2 equations, the loads included, the reaction forces not | `ODE2RHS` | `MainSolver.ComputeODE2RHS`, `CSystem::ComputeSystemODE2RHS` |
| `ODE1RightHandSide(q, q_t, y, t)` | $\gv$ of $\dot\yv = \gv$ | `ODE1RHS` | `ComputeODE1RHS` |
| `ReactionForces(q, q_t, y, lambda_, t)` | $\Cm_{\qv}\tp\tlambda$, ODE2 only, on the LHS | `ODE2ReactionForcesLHS` | the LHS term of the system equations in `solver.md` |
| (missing) | $\Cm_{\yv}\tp\tlambda$ of the ODE1 equations | `ODE1ReactionTermsLHS` | `solver.md`, the ODE1 rows; needed by ObjectGenericDAE |
| `Residual(q, q_t, q_tt, t, y)` | $\Mm\ddot\qv - \fv$, LHS minus RHS | `ODE2Residual` | |
| `ResidualJacobian(...)` | its Jacobian | `ODE2ResidualJacobian` | |
| `SystemResidual`, `SystemJacobian` | all rows, as the implicit solver | kept | `ComputeNewtonResidual`, `ComputeNewtonJacobian` |
| `AlgebraicEquations` | $\cv$ | kept | `ComputeAlgebraicEquations` |
| `Equations()['forces']` | $\fv$ | `'ODE2RHS'` | |

`SystemResidual` has the ODE2 rows as LHS minus RHS and the ODE1 rows as RHS minus LHS, $\gv - \dot\yv$, as
`CSolverImplicitSecondOrder` computes them ("different from ODE2"), while `solver.md` writes both as LHS minus RHS;
the docstring says so, the manual does not. The module names the constraints $\cv$ and the ODE1 right-hand side $\gv$;
`solver.md` names the constraints $\gv$ and the ODE1 right-hand side $\fv_\FO$; ObjectGenericODE1 names its
coordinates `q`. **Recommendation**: the names of the table, and the letters of `solver.md` in the docstrings. The
module is in no release yet (it came after 1.13.0), so the rename needs no deprecation.

### 12.6 The structure of exudyn.symbolic, checked

**What is good.** A function is recorded by running it once with symbolic arguments, so the same Python function -
with cMath - computes numbers and records; the graph shares subexpressions, and `Optimize` merges them; the flat
program, its export, its one-pass derivative and its parallel evaluation in the threads of the solver are there; one
recording serves all items of a kind through `parameters`. For the known use cases - user functions of items, user
items, the composed system - this is complete.

**What costs.** Facts from the code (2026-10-28):

- **Every operation is written up to eight times**: the tree `Evaluate`/`EvaluateRef`, the tree `Diff`/`DiffRef`,
  `EvaluateFromOperands` of the flat program, its derivative rule (a switch by `FlatName` in
  `FlatProgram::Differentiate`), `ToString` and `ToStringFromOperands`, `NewWithOperands`, the rules of `Optimize`,
  and the writers of `symbolicExport.py`; about 80 expression classes in 9200 lines of C++. The fix of pow (#3135) was
  made twice, in the tree and in the one pass.
- **Values live in the graph**: a named variable holds its value, a vector or matrix node its evaluation and derivative
  buffers; so the graph is state, and the threads need values of their own (#2978).
- **A recording has the signature of an item's user function**, `CreateSymbolicUserFunction(mbs, function, name,
  item)`; the composed system borrows the signature of ObjectGenericODE2 for its two recordings and cannot add the
  parameters (12.4), y or lambda.
- **One namespace of variables**, `esym.variables`, global; the arguments of a recording are named variables of their
  parameter names.
- **Vectors and matrices are nodes** next to their scalar components: comfortable to record, but the derivative, the
  GPU levels (RG14) and the export scalarize them again.

**What would simplify a lot:**

| | change | effect | size |
|---|---|---|---|
| R1 | **one table of operations**: per operation code its name, arity, value, derivative rule, simplification and text, used by the flat evaluation, the derivative, `Optimize`, the export, the count and the uniform evaluation; the tree classes only record and build the program | one place per operation instead of eight; the derivative as a new program (RG12.54.2, option D) is one more column of the table; about half of the C++ | large: a C++ refactoring with a transition, after RG12.54.2 |
| R2 | **recordings with a free signature**: a recording of expressions in declared inputs, e.g. `esym.Function(expressions, inputs)` - `GetFlatProgram(expressions)` nearly does it -; an item user function is such a function whose inputs match its signature | the composed system with its natural inputs (q, q_t, y, lambda, t, p); local names instead of the global namespace; the export and the GPU take any function | small to medium |
| R3 | **values outside the graph**: evaluation frames - the slot arrays of the flat program - per thread or per state | threads and ensembles without copies of the graph; follows from R1 | with R1 |
| R4 | **a scalar program on request**: the vector and matrix nodes expanded into scalar operations, as the experiment of RG14.2.1 does in Python | the GPU levels, the export to plain C, the count per scalar operation | medium, independent |
| R5 | **the count** (RG12.61) | the measure for R1 to R4 | small, a step |

**Recommendation**: R2 first - it unblocks the parameters (12.4) and the system variables (12.2), and it is small; R1
as a planned refactoring once RG12.54.2 has decided the derivative program, which needs exactly the table of R1; R4
when RG14.2.2 needs it; R3 with R1.
