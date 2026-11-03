# The symbolic system: how it works inside

`exudyn/symbolics/systemModel.py` - `SymbolicSystemBase`, `SymbolicSystem`, `NumpySystem`, `SystemVariables` - and
the solvers of `exudyn/symbolics/solvers.py`. The user side is in the manual, *Symbolic system and user functions*;
the recordings themselves are in [symbolic.md](symbolic.md), the user items in [userItems.md](userItems.md).

## 1. The representation

The equations of a system are **per kind a flat program and per system the maps from the local to the global
coordinates**, as Exudyn's own assembly has them; the global system is never one big expression. The objects whose
user functions are the same recording form one kind, with a table of their parameters and of their LTG indices. The
terms are the residual of the ODE2 equations per object, $\rv_e = \Mm_e(\qv_e)\ddot\qv_e - \fv_e(\qv_e, \dot\qv_e,
t)$, the mass matrix as local triplets, the constraints of the joints with $\Cm_{\qv}\tp\zv$, the ODE1 equations and
the loads through the Jacobians of their markers; the global matrices exist as triplets, their pattern known from the
LTG maps. The goals behind it: the export of the equations (C, C++, Python, sympy), the evaluation on a GPU
(#3137), and a check of both by solvers written in Python.

`SymbolicSystem(mbs)`, after `mbs.Assemble()`:

- **is extracted from mbs**, and only if every item is compatible; an item that is not - a built-in body, a Python
  user function, a recording that reads mbs (an item output), coordinates of other items than those of section 2 - is
  refused with the list of these items, never approximated or skipped;
- **reuses the data of mbs**: the LTG maps, the reference and initial coordinates, the parameters of the items;
- **adds only the solve**: a computed state goes back into mbs by `SetSystemState(q, q_t, q_tt, t, configuration)`.
  A solver of the module writes its output through an Exudyn solver object that does nothing else -
  `InitializeSolver`, `WriteSolutionFileHeader`, and per written step the state and `FinishStep` -, so that the
  solution file, the sensors and the renderer work as after `SolveDynamic`;
- **is evaluated by exudyn.symbolic only**; numpy, C and other formats are an export of the flat programs.

`q` are the ODE2 coordinates of Exudyn's solvers, relative to the reference configuration, `qRef` the reference
values read from mbs once; a user body gets `qRef` and `qTotal = qRef + q`, ObjectGenericODE2 gets `q`. The names
follow Exudyn's (`notation.md`): `ODE2RHS` is $\fv$, `ODE2ReactionForcesLHS` $\Cm_{\qv}\tp\zv$,
`ODE2ResidualLHS` $\Mm\ddot\qv - \fv$; `SystemResidual` has the ODE2 rows as LHS minus RHS and the ODE1 rows as
$\gv - \dot\yv - \Cm_{\yv}\tp\zv$, as `CSolverImplicitSecondOrder` computes them (#3155, #3156).

## 2. What is assembled

| item | how |
|---|---|
| ObjectGenericODE2 with constant matrices and symbolic mass and force functions | directly (#3110) |
| ObjectUserBody | its mass matrix and force recordings in its coordinates; the points by its position recording (#3110) |
| ObjectConnectorUser with a symbolic forceUserFunction of its marker states | on position markers of point nodes and of the ground (#3121), on MarkerBodyPosition and MarkerBodyRigid of user bodies and of the ground, with the rotations if it requests Orientation (#3129): the marker states by the recorded position, rotation and Jacobian functions of the body, the forces by $\Jm_{pos}\tp\fv + \Jm_{rot}\tp\mv$ |
| ObjectJointUser | the constraint recording with the marker states, $\Cm_{\qv}\tp\zv$ by the Jacobian recording and the marker Jacobians (#3141) |
| ObjectGenericODE1 with a symbolic rhsUserFunction, LoadCoordinate on it | `ODE1RHS` $\Am\yv + \fv + \fv_{user}(\yv, t)$, `ODE1Jacobian` exact (#3144) |
| ObjectGenericDAE | its own kind: the forces in `ODE2RHS`, which then needs $\yv$, the ODE1 rows, the constraints and the reaction terms $\Cm_{\qv}\tp\zv$, $\Cm_{\yv}\tp\zv$ (#3190) |
| data coordinates of ObjectConnectorUser and ObjectGenericDAE | their post-Newton recordings, `PostNewton` (#3196) |
| LoadForceVector, LoadTorqueVector, LoadCoordinate, LoadMassProportional | constant or by a symbolic user function of the time, on node markers at their LTG indices, on body markers of a user body by $\Jm\tp\vv$ of its recordings, `bodyFixed` rotated by its rotation recording (#3118) |

## 3. The composed system and the equations

`Equations(format)` composes the system: the code of each recording in the format 'symbolic' of codeExport, called
with exudyn.symbolic arguments, records the graph again with the arguments replaced (#3130) - in the format 'sympy'
with sympy symbols. The connectors are composed with the marker kinematics of their bodies (`_MarkerData`,
`_MarkerState`), the loads added. `SystemVariables(system)` is a `VariableSet` of `q`, `q_t`, `y`, `y_t`, `z` and `t`,
with the marker states as expressions of them (`MarkerPosition`, `MarkerVelocity`, ...) and `Function(result)`
(#3151). `GetFlatProgram(format, results)` writes what is asked for (#3132, #3146).

With `composeSystem=True`, the default, the forces of `Equations()` and the nonzero entries of its mass matrix are
recorded once each as functions of the signature of ObjectGenericODE2 and evaluated by one flat program each (#3131).
Measured on a chain of 3 rigid bodies with 18 connectors and 4 loads: object by object about 120 calls of `Evaluate`,
0.59 ms per force evaluation, as each call costs about 3 us; composed, the forces 14 us (1591 operations), the mass
matrix 4.5 us; Exudyn's `ComputeODE2RHS` with the same user items 88 us. The mass matrix of up to 200 coordinates is
decomposed dense, as forming the sparse matrix cost more than the evaluation.

**The parameters** (#3152, #3159): the parameters of the user items may be expressions of named variables of
`mbs.symbolicVariables` (#3208), kept per item in the `ParameterBindings` of `UserItemCreator`. `Equations()` passes
for each item its list of expressions and numbers as `parameters` instead of the numbers, so that the equations carry
the variables, and its `'parameters'` lists them. The recordings of the evaluation - the composed system and the
functions of the connectors, joints and ObjectGenericDAE constraints for their Jacobians - take the values of the
parameters as their last input `p` (`_ParameterBackend`): the bound expressions are written in the components of `p`
by one `esym.Function` of the bindings in the named variables, called with `p[k]` as arguments. So `SetParameterValues`
sets the variables, writes the numbers into the items (`ParameterBindings.Apply`) and into the tables the system keeps
for the evaluation item by item, and records nothing - 0.04 ms against 6 ms of building the system for
symbolicParameterJacobianTest.py -; `ODE2RHSParameterJacobian` and `MassMatrixParameterJacobian` are one
`EvaluateJacobian('p', ...)` of the composed recordings. The algebraic equations have no derivative by `p`. The recording of a kind stays one recording with `parameters` as its argument, so one serves all items
of the kind in C++. `RigidBodyInertia` keeps symbolic values as numpy arrays of `esym.Real` components (`_InertiaArray`),
and forms its products component by component (`_Scaled`, `_ParallelAxis`), as an operation of a number array with an
`esym.Real` would give an `esym.Matrix`; its checks and its drawing use the present values (`_Numbers`).

## 4. The Jacobians

`ODE2ResidualLHSJacobian` takes the one-pass derivative of the recordings (`EvaluateJacobian`, symbolic.md): of the two
composed recordings up to 200 coordinates, else object by object, $\partial(\Mm\ddot\qv)/\partial\qv$ from the derivative
of the mass matrix - a term Exudyn's solvers leave out, so the Newton iterates differ, not the solution (#3228).
`SystemJacobian` is exact: the connectors and joints by one `esym.Function` each of their generalized forces, or of
their constraints and reaction forces, in the coordinates of the bodies of their markers (`_ItemFunction`,
`_ConnectorJacobian`, `_JointJacobian`, #3166, #3169) - 0.49 ms per evaluation on symbolicSystemItemJacobiansTest.py
(4 rigid bodies, 2 mass points, 5 joints, 4 connectors) instead of 2.8 ms of the chain rule written by hand it
replaced, after 14.5 ms for building the functions -; the ODE1 block by `ODE1Jacobian`. `SymbolicSystemBase` has
central differences, for a system without derivatives of its own, never for a constraint (#3101).
`esym.Jacobian` gives the derivative as new expressions (#3163), for an exported Jacobian.

## 5. One interface for every solver

The solvers work on the equations, not on mbs and not on the recordings, so that the same solver runs a
SymbolicSystem, an exported system or one written by hand (`NumpySystem`):

| member | what | in the base class |
|---|---|---|
| `nODE2`, `nODE1`, `nAE` | the numbers of coordinates | set by the derived class |
| `InitialState()`, `InitialStateODE1()` | `(q0, q_t0)`, `y0` | zeros |
| `ODE2RHS(q, q_t, t, y)`, `ODE1RHS(q, q_t, y, t)` | $\fv$, $\gv$ | the derived class |
| `MassMatrixTriplets`, `MassMatrix(..., sparse)` | $\Mm$ | from the triplets |
| `MassMatrixBlocks(q, q_t, t)` | the mass matrices of the bodies with their LTG tables, for the inverse per body | one block of all coordinates |
| `IsMassMatrixConstant()` | True: factored once | False |
| `AlgebraicEquations`, `ODE2ReactionForcesLHS`, `ODE1ReactionTermsLHS` | $\cv$, $\Cm_{\qv}\tp\zv$, $\Cm_{\yv}\tp\zv$ | zero |
| `ODE2ResidualLHS`, `ODE2ResidualLHSJacobian` | $\rv$ and its combined Jacobian | from the above, central differences |
| `SystemResidual`, `SystemJacobian` | all rows, by the columns of the Newton unknowns | the ODE2 block as above, the rest by differences |
| `InitialStateData`, `PostNewton` | the data coordinates and their post-Newton step | none |

A solver returns a dict with `'time'`, `'q'`, `'q_t'`, `'q_tt'` (and `'y'`, `'y_t'`, `'z'`) at the steps of
`solution.file.writePeriod` and the statistics.

## 6. The solvers

- **SolveExplicit** (#3112): explicit Euler, RK44 and velocity Verlet as `CSolverExplicit` computes them, on
  $[\qv, \dot\qv]$; the ODE1 coordinates in the stages of Euler and RK44; algebraic equations refused. The
  accelerations by the inverse per body with `explicit.computeMassMatrixInversePerBody` - one batched inversion per
  kind of the stacks of `MassMatrixBlocks`, checked that no two blocks share a coordinate -, otherwise by
  `scipy.sparse.linalg.splu`, once for a constant mass matrix. The inverse per body is the form a GPU takes.
- **SolveGeneralizedAlpha** (#3126, #3134): the generalized-alpha method of Arnold and Bruls and Newmark's method as
  `CSolverImplicitSecondOrder`: the predictor, Newton with $\Jm = \beta'\Mm + \gamma'\Dm + \Km$, the ODE1 coordinates by
  the trapezoidal rule, index-3 constraints with the scaling of Exudyn (ODE2 rows times $\beta h^2$, the algebraic
  columns divided by it), the initial accelerations of $[\Mm\;\Cm_{\qv}\tp; \Cm_{\qv}\;\Null]$, the convergence as
  `CSolverBase::Newton`; a Newton iteration that does not converge raises. `useIndex2Constraints` is refused.
- Both read `SimulationSettings`, and with `adaptiveStep` a step rejected by the post-Newton step is repeated with a
  smaller step, which grows again as in `CSolverBase::SolveSteps` (#3220).

Measured: the explicit solvers on a net of 4 x 4 mass points equal `SolveDynamic` to 2.6e-14 relative (the same
formulas), 200 steps of RK44 0.29 s on the symbolic system, 0.12 s as `NumpySystem`, 0.02 s in `SolveDynamic`;
generalized-alpha equal to `SolveDynamic` to the Newton tolerance; with joints (symbolicSystemJointsTest.py, 33
unknowns) 0.44 s against 0.059 s of SolveDynamic.

## 7. Decisions

- The residual for the implicit solvers and the sparse mass matrix, both available; exudyn.symbolic first, other
  formats by the export, Numba and CUDA for the GPU (#3137) (maintainer 2026-10-26).
- The system is extracted from mbs and reuses its data; a state goes back into mbs, only the solve is new.
- What exudyn.symbolic lacks is added there, not worked around in `SymbolicSystem`.
- One base class for every system, on which the solvers work without distinguishing the kinds of systems.
- Model and solvers in `exudyn.symbolics`, not in systemUtilities.py: they work on a `SymbolicSystemBase`, not on mbs.
- Items that couple the coordinates of several nodes inside mbs are ObjectGenericDAE, whose node lists give the LTG
  map, so that the Jacobian sees the coupling; system coordinates read from mbs inside an item (hidden from the LTG
  map) are not offered.
- The parameters are bound in the symbolic layer, where the user items are created; MainSystem and the C++ items stay
  numeric, while the named variables belong to the model, `mbs.symbolicVariables` (#3159, #3208). Bindings stored by
  the Add and Modify functions of MainSystem, or symbolic values held by the C++ items, are not done: MainSystem does
  not know the symbolic layer, and the C++ evaluation needs the numbers anyway.
- What else is to be symbolic - loads, markers, the matrices of the generic objects, the frames of joints and
  connectors - becomes a user item when it is needed, its values its parameters; no other ways for them (maintainer
  2026-11-02, 2026-11-03).
- No forward sensitivities of the solution in the Python solvers: beside adjoint methods, central differences of whole
  simulations are simpler and not much slower; the derivatives of the equations by the parameters are there for them
  (maintainer 2026-11-03).
