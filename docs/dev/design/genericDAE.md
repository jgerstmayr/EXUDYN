# ObjectGenericDAE: design (revision2026c step RG9.10, #3143)

An object on ODE2 nodes, ODE1 nodes and algebraic coordinates of its own - the form of `SymbolicSystemBase`
(exudyn.symbolics.systemModel) as one item, and the generic object of #1104. Maintainer 2026-10-28: good as
suggested, the user functions get mbs and the parameters as the existing ones; 2026-10-29: good to go. This note
gives the equations, the C++ class, what the solvers have and what they lack, and the sub-steps.

## 1. The equations

With the ODE2 coordinates $\qv$ of the ODE2 nodes, the ODE1 coordinates $\yv$ of the ODE1 nodes and the
algebraic coordinates $\tlambda$ of the object - $\zv$ in Python code -, the object adds

$$
\Mm(\qv, t)\,\ddot\qv + \Dm\dot\qv + \Km\qv + \Em\yv + \Cm_{\qv}\tp\tlambda = \fv + \fv_u(\qv, \dot\qv, \yv, t) ,
$$

$$
\dot\yv + \Cm_{\yv}\tp\tlambda = \Am\yv + \Bm\qv + \Bm_t\dot\qv + \gv + \gv_u(\qv, \dot\qv, \yv, t) ,
$$

$$
\Cm_{\qv}\qv + \Cm_{\yv}\yv - \cv_0 + \cv_u(\qv, \dot\qv, \yv, \tlambda, t) = \Null ,
$$

the linear terms by matrices, the nonlinear ones by user functions ($\fv_u$, $\gv_u$, $\cv_u$, and $\Mm$ by a
massMatrixUserFunction instead of the constant mass matrix). The left-hand side and the right-hand side keep
Exudyn's convention (`notation.md`): the object computes its LHS terms, the system its RHS.

## 2. The item

| parameter | symbol | as in |
|---|---|---|
| `nodeNumbersODE2` | the nodes of $\qv$, any nodes with ODE2 coordinates | ObjectGenericODE2 `nodeNumbers` |
| `nodeNumbersODE1` | the NodeGenericODE1 of $\yv$ | ObjectGenericODE1 `nodeNumbers` |
| `numberOfAlgebraicEquations` | the size of $\tlambda$, coordinates of the object | ObjectJointUser `numberOfConstraints` |
| `massMatrix`, `dampingMatrix`, `stiffnessMatrix`, `forceVector` | $\Mm$, $\Dm$, $\Km$, $\fv$ | ObjectGenericODE2 |
| `systemMatrix`, `rhsVector` | $\Am$, $\gv$ | ObjectGenericODE1 |
| `couplingMatrixODE1`, `couplingMatrixODE2`, `couplingMatrixODE2_t` | $\Em$, $\Bm$, $\Bm_t$ | new |
| `constraintMatrixODE2`, `constraintMatrixODE1`, `constraintVector` | $\Cm_{\qv}$, $\Cm_{\yv}$, $\cv_0$ | new |
| `parameters` | a vector of the item, named by a ParameterLayout | ObjectUserBody |

User functions, all with `(mbs, t, itemNumber, q, q_t, y, ..., parameters)` - q the ODE2 coordinates relative to
the reference as in ObjectGenericODE2 (total coordinates wait for RG13.2):

| user function | returns |
|---|---|
| `massMatrixUserFunction(mbs, t, itemNumber, q, q_t, y, parameters)` | $\Mm$, dense or sparse (#3022) |
| `forceUserFunction(mbs, t, itemNumber, q, q_t, y, parameters)` | $\fv_u$ |
| `rhsUserFunction(mbs, t, itemNumber, q, q_t, y, parameters)` | $\gv_u$ |
| `constraintUserFunction(mbs, t, itemNumber, q, q_t, y, z, velocityLevel, parameters)` | $\cv_u$; at velocity level its time derivative |
| `jacobianUserFunction(mbs, t, itemNumber, q, q_t, y, z, factorODE2, factorODE2_t, factorODE1, parameters)` | the local Jacobian of all rows of the object by all its coordinates, with the factors of the solver |
| `graphicsDataUserFunction(mbs, itemNumber)` | the graphics, as ObjectConnectorUser and ObjectJointUser (#3147) |

The C++ class is a body with several nodes, as ObjectGenericODE2 (`CObjectBody`, `ComputeMassMatrix`,
`ComputeODE2LHS`, `ComputeJacobianODE2_ODE2`), with ODE1 coordinates (`GetODE1Size`, `ComputeODE1RHS`) and algebraic
equations of its own (`GetAlgebraicEquationsSize`, `ComputeAlgebraicEquations`, `ComputeJacobianAE` with its four
blocks). Each user function gets a symbolic version (PySymbolicUserFunctionSet), so that a recording is evaluated
in C++, in the threads too (#2978), and SymbolicSystem takes the object.

## 3. What the solvers have, and what they lack (checked in the C++, 2026-10-29)

- **ODE2 rows**: the mass matrix, the LHS, its analytic Jacobian by $\qv$, $\dot\qv$ (`ComputeJacobianODE2_ODE2`)
  and the reaction forces $\Cm_{\qv}\tp\tlambda$ (`ComputeODE2ProjectedReactionForces`) exist. The coupling
  $\partial/\partial\yv$ of the ODE2 rows (`JacobianType::ODE2_ODE1`) is **numerical only**, and only for an object
  without an analytic ODE2 Jacobian: `CSystem::JacobianODE2RHS` differentiates `ComputeODE2LHS` by the ODE1
  coordinates in the branch of the numerical ODE2 Jacobian; an object with `ComputeJacobianODE2_ODE2` gets no
  ODE2-ODE1 block at all.
- **ODE1 rows**: `ComputeODE1RHS` exists; its Jacobian by $\yv$, and by $\qv$, $\dot\qv$ (`ODE1_ODE2`,
  `ODE1_ODE2_t`), is **numerical only** (`CSystem::NumericalJacobianODE1RHS`); CObject has no analytic ODE1 Jacobian.
- **The term $\Cm_{\yv}\tp\tlambda$ in the ODE1 rows is missing**: `CSolverImplicitSecondOrder::ComputeNewtonResidual`
  computes the ODE1 rows as $\gv - \dot\yv$ without it, while `CSystem::JacobianAE` already adds the transposed block
  `AE_ODE1` into the ODE1 rows of the Jacobian; and `ComputeODE2ProjectedReactionForces` raises "not implemented for
  ODE1 jacobian of algebraic equations". No item gives `AE_ODE1` today, so nothing is wrong yet; ObjectGenericDAE would
  be the first.
- **AE rows**: `ComputeAlgebraicEquations` and `ComputeJacobianAE` with the blocks AE_ODE2, AE_ODE2_t, AE_ODE1 and AE_AE
  exist for objects (the bodies with algebraic equations).
- **Explicit solver**: it ignores algebraic equations with a warning, or eliminates CoordinateConstraints
  (`eliminateConstraints`); it evaluates ODE2 and ODE1 rows in each stage. ObjectGenericDAE with $\tlambda$ is for the
  implicit solver; without algebraic equations it runs in both.

## 4. Decisions

1. **The ODE1 Jacobians and the coupling ODE2-ODE1**: (a) new analytic interfaces of CObject - `ComputeJacobianODE1`
   (the ODE1 rows by $\yv$, $\qv$, $\dot\qv$) and the ODE2 rows by $\yv$ - which the jacobianUserFunction or the
   one-pass derivative fill, and CSystem uses them where an object gives them; or (b) the numerical blocks of today
   for the ODE1 rows and the coupling, the analytic ones for ODE2 and AE. #3101 forbids numerical differences for
   constraint Jacobians; (b) keeps them out of the AE rows, but the step text says "never by numerical differences".
   **Recommendation: (a)**, as a sub-step of its own before the item; (b) would be the interim if (a) is postponed.
2. **$\Cm_{\yv}\tp\tlambda$**: added to the ODE1 residual of the implicit solver and to the reaction forces - a
   change of the solver that no present model sees (no item gives AE_ODE1), checked by a model against
   SymbolicSystemBase.SystemResidual, which has the term (#3155).
3. **One jacobianUserFunction of all blocks**, or one per block: one function of all rows by all coordinates, with the
   factors, is what an implicit solver needs and what a recording differentiates in one pass; per block is easier to
   write by hand. **Recommendation: one**, the blocks taken from it by the object.

## 5. Sub-steps

1. This note (RG9.10.1).
2. The solver: $\Cm_{\yv}\tp\tlambda$ in the ODE1 residual and the reaction forces; the analytic ODE1 and ODE2-ODE1
   Jacobians of objects (decision 1).
3. The item with its linear terms and the Python user functions; test models against ObjectGenericODE2,
   ObjectGenericODE1 and a CoordinateConstraint of the same equations.
4. The symbolic versions of the user functions; the Jacobian by the one-pass derivative.
5. SymbolicSystem takes ObjectGenericDAE - its kind, Equations(), the export.
