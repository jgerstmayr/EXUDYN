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
  `GetSystemMassMatrix`, `GetSystemJacobian`, `ComputeNewtonResidual`, `ComputeNewtonJacobian`, used by
  `python/TestModels/manualExplicitIntegrator.py` and `sparseSystemMatricesTest.py`. These give the reference for every
  term of the symbolic model at any state.
- **Derivatives.** `SReal.Diff(var)` is the forward derivative by recursion over the tree (`ExpressionBase::Diff` in
  `src/Linalg/Symbolic.h`), evaluated at the current values: one number per call, no expression, and on a graph with
  shared subexpressions every path is walked again. It is not part of the flat program.
- **Sparse matrices** inside Exudyn: `MatrixContainer` with triplets, faster than dense from about n > 50. scipy is an
  optional dependency (rule 6).

## 3. The system model

`SymbolicSystem(mbs)`, after `mbs.Assemble()`, in `exudyn/misc/symbolicSystem.py`:

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
| ObjectGenericODE2, ObjectGenericODE1 with symbolic mass, force and Jacobian functions | directly |
| ObjectUserBody, NodeUser | its mass matrix and force functions in its coordinates; the positions of points by its positionUserFunction |
| ObjectJointUser, ObjectConnectorUser, MarkerBodyUser | the constraint or force is symbolic; the marker positions and rotations as programs, their Jacobians by differentiation (section 5) - no sympy needed |
| loads on these bodies | the load vector, by the position Jacobian of the marker |
| built-in items, Python (not symbolic) user functions | refused, with the list of items; the built-in items need recordings of their own (the AD templates, #3048) - not proposed |

## 4. Evaluation: explicit on the GPU, Numba first

**Numba first.** The `numpy` writer gives scalar Python code per program; `numba.njit` compiles it, and a loop over the
objects of a kind (`prange`) evaluates the kind for all objects, reading the coordinates through the LTG table and the
parameters from the parameter table - one compiled function per kind. Numba is optional (rule 6): imported at the
point of use, with a clear failure. The same structure maps to `numba.cuda` or CuPy kernels later: one thread per
object, the scatter by atomic adds or by a coloring of the objects.

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
| A central differences | 2n evaluations of the residual | approximate | not used: nowhere else in Exudyn is a constraint Jacobian numerical (#3101) |
| B `esym.Diff` per q[i] and q_t[i] | n calls per output, each a recursion over the tree | works now; slow on shared graphs | numbers, no program |
| C forward mode over the flat program | per slot a value and n tangents | memory: slots x n, the "local stack" per derivative | numbers for all n directions in one pass |
| D symbolic differentiation into new nodes | per node class a derivative from its operands' derivatives, then Optimize | once, at setup | **a flat program of the Jacobian**, exportable like any other |

D is what G1 and G2 need: the Jacobian becomes code, and Optimize removes the duplicates that the product and chain
rules create. Every node class gets a function `DiffFromOperands(operands, operandDerivatives)` that returns new nodes,
next to `NewWithOperands`; differentiating a program with respect to its n inputs is one pass in topological order with
n derivative slots per node, most of them zero - kept sparse, so the cost follows the real dependencies. C is the
runtime form inside C++ for the item Jacobians of RG7.17. Both are the subject of RG12.54 (#3108); B is the fallback
until then, correct but slow, and acceptable for the Python solver of G3.

The marker Jacobians of the joints and connectors (section 3) are derivatives of the marker position and rotation
programs by D - the reason the earlier plan needed sympy.

## 6. The implicit solver in Python (G3)

Generalized-alpha (Chung and Hulbert, in the form of Arnold and Brüls that Exudyn uses), index 3 with the constraints
on position level, Newton with the Jacobian

$$\Jm = \beta' \Mm + \gamma' \Dm + \Km, \quad \beta' = \frac{1 - \alpha_m}{h^2 \beta (1 - \alpha_f)}, \quad
\gamma' = \frac{\gamma}{h \beta}$$

with $\Km = \partial \rv / \partial \qv$ and $\Dm = \partial \rv / \partial \dot\qv$, extended by $\Cm_q$ and $\Cm_q\tp$.
It reads `SimulationSettings.timeIntegration`: `numberOfSteps`, `endTime`, `generalizedAlpha.spectralRadius`,
`generalizedAlpha.computeInitialAccelerations`, `newton.relativeTolerance`, `newton.absoluteTolerance`,
`newton.maxIterations`, `newton.useModifiedNewton` - so that the same settings run in Exudyn and in Python. Sparse
linear algebra by scipy.sparse when available, dense numpy otherwise.

**The check** in two steps, each a test model:

1. Terms: at random states, $\Mm$, $\rv$, $\gv$ and the Jacobians of the symbolic model against
   `ComputeMassMatrix`, `ComputeODE2RHS`, `ComputeAlgebraicEquations`, `ComputeJacobianODE2RHS`, `ComputeJacobianAE`
   of `MainSolverImplicitSecondOrder` - equal to rounding.
2. Time integration: the Python solver against `mbs.SolveDynamic` with the same settings - the same algorithm, so equal
   to the Newton tolerance.

**Where it goes.** The maintainer suggested `systemUtilities.py`. That module holds the functions that describe a
system; the functions that need a solver - `ComputeLinearizedSystem`, `ComputeODE2Eigenvalues` - are in `solver.py`
(docs/dev/internals/systemUtilities.md §1). The solver works on a `SymbolicSystem`, not on `mbs`, so the proposal is
`exudyn/misc/symbolicSystem.py` for the model and its solver together; see decision 3.

## 7. Sub-steps of RG12.51 (as planned after the decisions)

- RG12.51.2 (done, #3110): `SymbolicSystem(mbs, evaluator)` for ObjectGenericODE2 and ObjectUserBody: kinds, LTG
  tables, mass matrix as triplets or (scipy) sparse matrix, forces and residual; equal to the solver API.
- RG12.51.3: the generalized-alpha solver in Python with the Jacobian column by column by `esym.Diff` (RG12.54.1); the
  time-integration check against `SolveDynamic`.
- RG12.55 first: user items with the interfaces of the built-in items (`exudyn/misc/symbolicUserItems.py`).
- RG12.51.4: those user items in the system - rigid bodies, joints, connectors, loads, the constraints.
- RG12.51.5: the export (G1): the programs, the sparsity pattern and the scatter as C++ and as Python.
- RG12.51.6: the evaluation by Numba for the GPU, an explicit integrator with a constant mass matrix and the
  matrix-free product.

## 8. Decisions (maintainer 2026-10-26)

1. Both available: the residual for the implicit integrators, which need only it, and the (sparse) mass matrix, needed
   e.g. to invert it.
2. exudyn.symbolic and numpy first; Numba for the GPU.
3. `exudyn/misc/symbolicSystem.py` for model and solver, for now: special, not for the broad use, of large potential.
4. Option D is the target of RG12.54; before it a Jacobian without a new program, column by column by `esym.Diff`
   (RG12.54.1), which stays as an option and as the comparison.
