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
  built-in body, a Python user function, a recording that reads mbs (an item output), ODE1 coordinates, algebraic
  equations, data coordinates, and loads until #3118 - is refused, with the list of these items. No item is
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
   - the loads, as forces through the marker Jacobians.
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
| ObjectConnectorUser with the symbolic force function of its marker states (#3119), ObjectJointUser, MarkerBodyUser | the force or constraint is symbolic; the marker positions and rotations as programs, their Jacobians by differentiation (section 5) - no sympy needed |
| LoadForceVector, LoadTorqueVector, LoadCoordinate, LoadMassProportional (#3118) | a constant load vector or a symbolic loadVectorUserFunction; on node markers of point nodes and coordinates a generalized force at the LTG indices of the marker; on body markers by the marker Jacobian |
| built-in items, Python (not symbolic) user functions, recordings that read mbs | refused, with the list of items; the built-in items need recordings of their own (the AD templates, #3048) - not proposed |

## 4. Evaluation: explicit on the GPU, Numba first

**Numba first.** The `numpy` writer of `symbolicExport` gives scalar Python code per program - in the export, not in
`SymbolicSystem`; `numba.njit` compiles it, and a loop over the objects of a kind (`prange`) evaluates the kind for all
objects, reading the coordinates through the LTG table and the parameters from the parameter table - one compiled
function per kind. Numba is optional (rule 6): imported at the point of use, with a clear failure. The same structure
maps to `numba.cuda` or CuPy kernels later: one thread per object, the scatter by atomic adds or by a coloring of the
objects.

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

**Now (#3113):** `SymbolicSystem.ResidualJacobian` is the central differences of `SymbolicSystemBase` (A); B comes back
as its override once `Diff` works on vectors and by the components of a vector argument (#3115). A Jacobian by `Diff`
built through a wrapper of `esym.Real` in Python (#3108) was removed again - section 11 lists what exudyn.symbolic
needs instead.

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

## 7. Sub-steps of RG12.51 (as planned after the decisions)

- RG12.51.2 (done, #3110): `SymbolicSystem(mbs)` for ObjectGenericODE2 and ObjectUserBody: kinds, LTG tables, mass
  matrix as triplets or sparse matrix, forces and residual; equal to the solver API.
- RG12.51.7 (done, #3113): `SymbolicSystemBase`, exudyn.symbolic only, `SetSystemState`, `InitialState`,
  `IsMassMatrixConstant`, scipy at the start of the module.
- RG12.56 (#3112, before RG12.51.3): explicit Euler, velocity Verlet and RK44 in Python, on the base class (sections 9
  and 10).
- RG12.51.3: the generalized-alpha solver in Python; the time-integration check against `SolveDynamic`.
- RG12.55 first: user items with the interfaces of the built-in items (`exudyn/misc/symbolicUserItems.py`).
- RG12.51.8 (#3118): the built-in loads, constant or with a symbolic user function.
- RG12.51.4: those user items in the system - rigid bodies, joints, connectors, the constraints.
- RG12.51.5: the export (G1): the programs, the sparsity pattern and the scatter as C++ and as Python, and a system
  class of the exported numpy code.
- RG12.51.6: the evaluation by Numba for the GPU, an explicit integrator with a constant mass matrix and the
  matrix-free product.

And in exudyn.symbolic (section 11): RG12.57 cMath with symbolic arguments (#3114), RG12.58 `Diff` of vectors and
matrices (#3115), RG12.59 the forward derivative of a flat program (#3116), RG12.60 a recording called with symbolic
arguments (#3117).

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
| `IsMassMatrixConstant()` | True: the solver factors $\Mm$ once | False |
| `Residual(q, q_t, q_tt, t)` | $\rv = \Mm\ddot\qv - \fv$ | from MassMatrix and Forces |
| `ResidualJacobian(q, q_t, q_tt, t, factor_q, factor_q_t, factor_q_tt, sparse)` | the combined Jacobian | central differences of Residual |

`SymbolicSystem` derives from it: `InitialState()` from the initial configuration of mbs, `IsMassMatrixConstant()`
from `constantMassMatrix` of the user bodies and the constant matrix of ObjectGenericODE2, `Residual` object by object,
and `SetSystemState` for the output (section 3). The kind of evaluation is decided once, by the class of the system:
`SymbolicSystem` by exudyn.symbolic, the exported system (RG12.51.5) by its generated code, a system written by hand
(RG12.56.2: `NumpySystem(nODE2, massMatrix, forces, initialState)`, a constant matrix or a function and a function) by
its own functions. The solvers use only this interface; the explicit ones only `InitialState`, `Forces`, `MassMatrix`
and `IsMassMatrixConstant`.

The result of a solver, the same for all: a dict with `'time'` (n_out), `'q'`, `'q_t'`, `'q_tt'` (n_out x nODE2), at
the steps of `solution.solutionWritePeriod`, and the statistics (steps, evaluations of the forces, factorizations); for
a `SymbolicSystem` also the solution file and the sensors of mbs (section 3).

## 10. Explicit solvers (RG12.56.1, #3112)

The first solvers, before generalized-alpha: they need no Jacobian and no constraints, so simple user items suffice -
mass points and spring-dampers (RG12.55.2; the spring-damper as ObjectConnectorUser with a symbolic force function,
#3119), and before them an ObjectGenericODE2 or user bodies with symbolic forces. Constant step size
$h = (t_{end} - t_{start})/n$ from `timeIntegration.startTime`, `endTime`, `numberOfSteps`; the method from
`timeIntegration.explicitIntegration.dynamicSolverType` (or an argument, as `SolveDynamic(solverType=...)`):

- **ExplicitEuler**: $\qv_{n+1} = \qv_n + h\,\dot\qv_n$, $\dot\qv_{n+1} = \dot\qv_n + h\,\av_n$, with
  $\av_n = \Mm^{-1}\fv(\qv_n, \dot\qv_n, t_n)$ - the first-order system $[\qv, \dot\qv]$ with the Butcher tableau of one
  stage, as `CSolverExplicit` does.
- **RK44**: the classical 4-stage tableau on $[\qv, \dot\qv]$, the stages as `src/Solver/CSolverExplicit.cpp`
  computes them: $K_i = (\dot\qv, \av)$ at $g_i = u + h \sum_j A_{ij} K_j$, $t + c_i h$, then $u + h \sum_i b_i K_i$.
- **VelocityVerlet**, as `CSolverExplicit` writes it: $\dot\qv_{n+1/2} = \dot\qv_n + \tfrac{h}{2}\av_n$,
  $\qv_{n+1} = \qv_n + h\,\dot\qv_{n+1/2}$, $\av_{n+1} = \Mm^{-1}\fv(\qv_{n+1}, \dot\qv_{n+1/2}, t_{n+1})$,
  $\dot\qv_{n+1} = \dot\qv_{n+1/2} + \tfrac{h}{2}\av_{n+1}$ - the force at the half-step velocity, as in Exudyn; the
  initial acceleration from the initial state.

**The accelerations.** $\av = \Mm^{-1}\fv$: with `IsMassMatrixConstant()` the matrix is factored once
(`scipy.sparse.linalg.splu`); otherwise solved per stage. `explicitIntegration.computeMassMatrixInversePerBody` has a
counterpart later: a block per kind object, as the kinds know their blocks. The settings that have no meaning here
(`automaticStepSize`, the tolerances, `useLieGroupIntegration`, `eliminateConstraints`) raise if set to a value other
than the default, so that a comparison is not silently different.

**The check** (a test model): a system of symbolic items and the same system written by hand as `NumpySystem`; each of
the three methods against `mbs.SolveDynamic` with `solverType=exu.DynamicSolverType.ExplicitEuler | VelocityVerlet |
RK44`, `automaticStepSize=False` and the same steps: equal to rounding (the same formulas; only the solve of $\Mm$
differs), about 1e-12 relative after 1000 steps; the hand-written system equal to the symbolic one; the sensors of mbs
written by the solver equal to those of `SolveDynamic`. A measurement of the time per step against Exudyn, for the
record only.

**Sub-steps of RG12.56**: .1 this concept; .2 `NumpySystem` on `SymbolicSystemBase` (the base class itself is
RG12.51.7); .3 the three explicit solvers, their output through mbs, and their test model.

## 11. What exudyn.symbolic lacks for the system, and the proposals

Building the system now meets four gaps. Each is a step of its own in exudyn.symbolic; `SymbolicSystem` does not work
around them.

**S1 `Diff` of vectors and matrices, by the components of vector variables (RG12.58, #3115).** Today `Diff(var)`
exists for scalar expressions and scalar named variables. A recording has vector and matrix arguments, `q[i]` is an
`operator[]` node without derivative, and the vector and matrix operations have none. Proposal:

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
- **In Python**: `Real.Diff(var)` as now, and `Real.Diff(vectorVariable, i)`; `Vector.Diff(var[, i])` and
  `Matrix.Diff(var[, i])` return numpy arrays; `UserFunction.EvaluateJacobian(argumentName, *arguments)` returns the
  Jacobian of the result of a recording by one of its vector arguments at the given arguments, column by column.
- **What it gives**: the Jacobian of the residual by `Diff` (option B) without any wrapper, the marker Jacobians of
  the joints and connectors as numbers, and the comparison for C and D. Its test: every operation against central
  differences, as symbolicModuleTest.py does for the scalar functions (#3075).

**S2 the forward derivative of a flat program in one pass (RG12.59, #3116).** S1 walks the tree once per seed, n times
for n coordinates. Over the flat program, each slot keeps its value and its n derivatives, computed in the order of the
program from those of its operands - one pass. The memory is slots x n numbers (x the size of a vector or matrix slot):
**one block per program, sized at the first evaluation from the program and kept**, a local memory of the program - so
no allocation per call, and no setting needed, since the size is known from the program and n. In the parallel
evaluation each thread has its block, as each has its values now (#2978). A setting for a fixed maximum size only if a
bound must be known in advance (a GPU kernel); an allocation per call would be simpler but stays in the hot path and is
harder to remove later - not proposed. It is the runtime Jacobian of the symbolic user functions in C++: the position
and rotation Jacobians of NodeUser, ObjectUserBody and MarkerBodyUser, the connector, and a joint without
jacobianUserFunction (RG7.17). After S1, which is its check.

**S3 a recording called with symbolic arguments (RG12.60, #3117).** A recording evaluates with numbers only. Called
with exudyn.symbolic arguments, it returns a copy of its expressions in which its argument variables are replaced by
the given expressions - the copy that `Optimize` makes by `NewWithOperands`, with the argument nodes mapped. Needed for
one expression per kind ($\Mm(\qv)\,\ddot\qv - \fv(\qv, \dot\qv, t)$ of the two recordings), for the forces of a
connector composed with the marker kinematics of its bodies, and for the export of one program per kind (RG12.51.5).
As numbers, the system needs no S3: it evaluates each recording and composes the results; S3 is for the programs.

**S4 cMath with symbolic arguments (RG12.57, #3114).** `np.sin` of an `esym.Real` records - numpy calls the method
`sin()` of the object -, `cMath.sin` raises, and the math check of the test models refuses `np.sin` (#2964). With S4
the functions of `exudyn.cMath` call those of exudyn.symbolic for a symbolic argument, so that one function written
with cMath runs on numbers and is recorded, also with arguments mixed between numbers and symbolic values.
