# Symbolic system models: the sections cited by other design notes (#3109)

What the symbolic system is and how it works is in docs/dev/internals/symbolicSystem.md and in the manual, *Symbolic
system and user functions* (#3221). This note keeps only the sections that uniformEvaluation.md and the plan cite, and
the open recommendation 12.3, with their numbers, until the steps that cite them are done.

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
With the table of operations (section 12.7, R1), D is no function per node class but the rule of the table:
`esym.Jacobian(expression, variable)` (#3163) differentiates each operation of the flat program by its rule - the
rules of the one-pass derivative - into new expressions, vector and matrix operations into vector and matrix
operations of the derivatives, then Optimize; C is done (#3116, and by `SymbolicProgram`, #3157). Measured
(symbolicJacobianExpressionsTest.py): a function of 6 components of a 3-vector, 69 operations, has a Jacobian
program of 280 operations, evaluated in 2.7 us against 4.4 us of EvaluateJacobian; equal to it to 1.5e-16, and its
Jacobian of a gradient is the Hessian.

The marker Jacobians of the joints and connectors (section 3) are derivatives of the marker position and rotation
programs by D - the reason the earlier plan needed sympy.

**Now (#3115):** `SymbolicSystem.ODE2ResidualLHSJacobian` is B: object by object `UserFunction.EvaluateJacobian` of the force
and mass matrix recordings by their arguments q and q_t (section 11, S1), $\partial(\Mm\ddot\qv)/\partial\qv$ from the
derivative of the mass matrix, and the constant matrices of ObjectGenericODE2; equal to the central differences of
`SymbolicSystemBase` (A) to their error, 6e-9. A, in the base class, stays the Jacobian of a system without derivatives
of its own and the comparison.

## 10. Explicit solvers (RG12.56.1, #3112)

The first solvers, before generalized-alpha: they need no Jacobian and no constraints, so simple user items suffice -
mass points and spring-dampers (RG12.55.2; the spring-damper as ObjectConnectorUser with a symbolic force function,
#3119), and before them an ObjectGenericODE2 or user bodies with symbolic forces. The step size
$h = (t_{end} - t_{start})/n$, smaller where a post-Newton step rejects a step (#3220), from `timeIntegration.startTime`, `endTime`, `numberOfSteps`; the method from
`timeIntegration.solverType` (or an argument, as `SolveDynamic(solverType=...)`). `SolveExplicit(system,
simulationSettings, solverType)` in `exudyn/symbolics/systemModel.py` (#3112):

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

## 12. Status, analysis and options (2026-10-28)

### 12.3 q, qRef and qTotal in systemModel.py

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
`NumpySystem`); the reference values an optional variable of `Equations()`, as the parameters. Whether user
functions get total coordinates in general is #1910, a performance question.

### 12.7 The table of operations

- **The vector and matrix operations are codes** (RG12.65.8, #3176, done): the 23 classes of vector and matrix
  operations - `+`, `-`, `*`, `@`, `vector@matrix`, `pos`, `neg`, `multiply`, `T`, the vectors and matrices of
  components, dot, `NormL2`, the components, `==` and `!=` of vectors - are rows of the table, found by the kind of
  the result, the name and the kinds of the operands; every operation of a recording has a code, and `Build` raises
  for one without. `ArrayOperation(code, operands, out)` computes them with the arithmetic of their nodes, for
  `SymbolicProgram` and for the constants that `Optimize` folds. The slots are `SlotValue`s - a number, or a pointer,
  rows and columns -; the vectors and matrices of the operations lie in one array of the program, laid out at the
  first evaluation and again when a leaf changes its size (the sizes are checked then, with the messages of the
  nodes); a leaf points into its node. A vector or matrix result is copied out of the array when it is read. The
  rules of the derivatives are the column of the table (`FindDerivativeRule` is gone, `esym.Jacobian` reads the
  table too). The recording (`RecordNode`) is now the only caller of `EvaluateFromOperands`. Measured: the values of
  the recordings of symbolicOnePassJacobianTest.py equal the tree evaluation to the bit in 20 states; a recorded 5x5
  mass matrix of products takes 2.40 us instead of 2.61 us per evaluation, a force of vectors and matrices 2.26 us
  instead of 2.35 us; `tableOperations` of `GetStatistics` counts every operation. What it opens: a thread can
  evaluate the program of another, as no value is kept in a node - the thread copies of the recording (#2978) are
  still made, as the arguments are leaves; the array of the program is what the GPU (RG14) and an export from C++
  read.
- **Proposal for sparse matrices** (RG12.65.7, #3168; maintainer 2026-10-29: needed for objects working with sparse
  matrices, a 20x20 matrix mostly of zeros; not for now). Today such a matrix is an `esym.Matrix` of components whose
  zeros are constants; `A@v` is a dense product of 400 multiplications, and `Optimize` cannot remove the zeros, as
  the product is one node, not scalar operations.
  1. **Recording**: a sparse matrix is declared, not detected - `esym.SparseMatrix(rows, columns, triplets)` with
     `triplets` a list of (row, column, expression), and `esym.Matrix(...).Sparse()`, which keeps the components that
     are not the constant 0. The pattern is fixed at the recording; a component that becomes 0 at run time stays in
     it. A detection in `esym.Matrix` (more than half constant zeros) would change the results of existing recordings
     by the order of the sums - not by default.
  2. **Table** (after RG12.65.8, as codes of the same kind): a slot kind `SparseMatrix` - the pattern (compressed rows:
     the start of each row, the column of each entry) kept once in the step, the values of the nnz entries in the
     memory of the program. Codes: `SparseMatrixVector` (A v), `SparseTransposedVector` (A^T v), `SparseScalar` (s A),
     `SparsePlus` (the union of the patterns, fixed when the program is built), `SparseComponent` (A[i,j], a constant 0
     outside the pattern), `SparseToDense`. A dense matrix times a sparse one stays out until a model needs it.
  3. **Derivatives**: the derivative of A by n variables is nnz x n (only the pattern); d(A v) = (dA) v + A dv
     summed over the pattern; `esym.Jacobian` builds the same as new nodes, the derivative of a sparse matrix being
     sparse with the same pattern.
  4. **Results**: a sum over the pattern leaves out only exact zeros, so A v equals the dense product to the bit
     where no entry outside the pattern is inf or nan - the test compares both.
  5. **Export and system**: triplets (row, column, program of the entry) - what `SymbolicSystem` builds for the mass
     matrices already (`MassMatrix(..., sparse=True)`); numpy format with `scipy.sparse` optional, sympy as
     `SparseMatrix`. A recorded mass matrix or stiffness of an object (FFRF reduced order, shells) then reaches the
     sparse system matrices without passing a dense block.
  6. **Order and size**: after RG12.65.8 (the codes and the memory of the program), about the size of that step;
     the test: a 20x20 band matrix of components, dense against sparse, values and Jacobians to the bit, and the time.
