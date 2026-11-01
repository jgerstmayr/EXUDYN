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
| `nodeNumbersAE` | the NodeGenericAE of $\tlambda$: the assembly gives a body algebraic coordinates through its nodes only | Euler parameter nodes |
| `massMatrix`, `dampingMatrix`, `stiffnessMatrix`, `forceVector` | $\Mm$, $\Dm$, $\Km$, $\fv$ | ObjectGenericODE2 |
| `systemMatrix`, `rhsVector` | $\Am$, $\gv$ | ObjectGenericODE1 |
| `couplingMatrixODE1`, `couplingMatrixODE2`, `couplingMatrixODE2_t` | $\Em$, $\Bm$, $\Bm_t$ | new |
| `constraintMatrixODE2`, `constraintMatrixODE1`, `constraintVector` | $\Cm_{\qv}$, $\Cm_{\yv}$, $\cv_0$ | new |
| `parameters` | a vector of the item, named by a ParameterLayout | ObjectUserBody |

User functions, all with `(mbs, t, itemNumber, q, q_t, y, ..., parameters)` - q the ODE2 coordinates relative to
the reference as in ObjectGenericODE2 (total coordinates wait for RG13.2). `forceUserFunction` and `rhsUserFunction`
exist (RG9.10.3, #3170): a block with one of them is differentiated numerically, the others by the matrices; the
others, whose Jacobian must not be numerical (#3101), come with their symbolic versions and the one-pass derivative
(RG9.10.4):

| user function | returns |
|---|---|
| `massMatrixUserFunction(mbs, t, itemNumber, q, q_t, y, parameters)` | $\Mm$, dense or sparse (#3022) |
| `forceUserFunction(mbs, t, itemNumber, q, q_t, y, parameters)` | $\fv_u$ |
| `rhsUserFunction(mbs, t, itemNumber, q, q_t, y, parameters)` | $\gv_u$ |
| `constraintUserFunction(mbs, t, itemNumber, q, q_t, y, z, velocityLevel, parameters)` | $\cv_u$; at velocity level its time derivative |
| `jacobianUserFunction(mbs, t, itemNumber, q, q_t, y, z, factorODE2, factorODE2_t, factorODE1, parameters)` | the local Jacobian of all rows of the object by all its coordinates, with the factors of the solver |
| `graphicsDataUserFunction(mbs, itemNumber)` | the graphics, as ObjectConnectorUser and ObjectJointUser (#3147) |

The C++ class `CObjectGenericDAE` (src/ImplObjects/CObjectGenericDAE.cpp) is a body with several nodes - the ODE2,
ODE1 and AE nodes in this order -, as ObjectGenericODE2 (`CObjectBody`, `ComputeMassMatrix`, `ComputeODE2LHS`,
`ComputeJacobianODE2_ODE2`, dense), with ODE1 coordinates (`GetODE1Size`, `ComputeODE1RHS`, `ComputeJacobianODE1RHS`,
`ComputeJacobianODE2_ODE1`) and algebraic equations (`GetAlgebraicEquationsSize`, `ComputeAlgebraicEquations`,
`ComputeJacobianAE` with its four blocks). The initial accelerations of the implicit solver keep the ODE1 rows a unit
matrix, without $\Cm_{\yv}$, as SolveGeneralizedAlpha (#3170). Each user function gets a symbolic version (PySymbolicUserFunctionSet), so that a recording is evaluated
in C++, in the threads too (#2978), and SymbolicSystem takes the object.

## 3. What the solvers have (checked in the C++ 2026-10-29, completed by #3167)

- **ODE2 rows**: the mass matrix, the LHS, its analytic Jacobian by $\qv$, $\dot\qv$ (`ComputeJacobianODE2_ODE2`)
  and the reaction forces $\Cm_{\qv}\tp\tlambda$ (`ComputeODE2ProjectedReactionForces`). The coupling
  $\partial/\partial\yv$ of the ODE2 rows (`JacobianType::ODE2_ODE1`) is the object's own `ComputeJacobianODE2_ODE1` if it
  sets `ODE2_ODE1_function`, else numerical - also for an object with an analytic ODE2 Jacobian, which got no coupling
  block before #3167.
- **ODE1 rows**: `ComputeODE1RHS`, and its Jacobian by $\yv$, $\qv$, $\dot\qv$: the object's own
  `ComputeJacobianODE1RHS` if it sets `ODE1_ODE1_function` (and `ODE1_ODE2_function`, `ODE1_ODE2_t_function` for the
  coupling), else numerical (`CSystem::JacobianODE1RHS`). ObjectGenericODE1 gives it without rhsUserFunction: the
  system matrix. Connectors with ODE1 coordinates (ObjectConnectorHydraulicActuatorSimple) stay numerical.
- **The term $\Cm_{\yv}\tp\tlambda$**: the implicit solver subtracts it in the ODE1 rows of its residual
  ($\gv - \dot\yv - \Cm_{\yv}\tp\tlambda$, `ComputeODE1ProjectedReactionTerms`) and `JacobianAE` adds the block
  $-\Cm_{\yv}\tp$ at the ODE1 rows and the AE columns, $\Cm_{\yv}$ at the AE rows and the ODE1 columns, with the factors
  of the solver; the static solver adds $+\Cm_{\yv}\tp\tlambda$, as it does $\Cm_{\qv}\tp\tlambda$ in its ODE2 rows. The
  objects are those with algebraic equations, ODE1 coordinates and `AE_ODE1`; no built-in item has them, ObjectGenericDAE
  is the first, and its test models check this part (RG9.10.3).
- **AE rows**: `ComputeAlgebraicEquations` and `ComputeJacobianAE` with the blocks AE_ODE2, AE_ODE2_t, AE_ODE1 and AE_AE
  for objects (the bodies with algebraic equations).
- **Explicit solver**: it ignores algebraic equations with a warning, or eliminates CoordinateConstraints
  (`eliminateConstraints`); it evaluates ODE2 and ODE1 rows in each stage. ObjectGenericDAE with $\tlambda$ is for the
  implicit solver; without algebraic equations it runs in both.

## 4. Decisions (maintainer 2026-10-29)

1. **The ODE1 Jacobians and the coupling ODE2-ODE1 are analytic**: the new functions `ComputeJacobianODE1RHS` and
   `ComputeJacobianODE2_ODE1` of CObject, called only for an object that sets their flags, so no cost for the others;
   ObjectGenericDAE fills them from its matrices and its jacobianUserFunction, or the one-pass derivative of its
   recordings.
2. **$\Cm_{\yv}\tp\tlambda$** in the ODE1 residual and the Jacobian (section 3), checked against
   SymbolicSystemBase.SystemResidual, which has the term (#3155).
3. **One jacobianUserFunction of all blocks**, with the factors of the solver; the object takes the blocks from it.

## 5. Sub-steps

1. This note (RG9.10.1).
2. The solver: $\Cm_{\yv}\tp\tlambda$ in the ODE1 residual and the reaction forces; the analytic ODE1 and ODE2-ODE1
   Jacobians of objects (decision 1).
3. The item with its linear terms and the Python user functions; test models against ObjectGenericODE2,
   ObjectGenericODE1 and a CoordinateConstraint of the same equations.
4. The symbolic versions of the user functions; the Jacobian by the one-pass derivative.
5. SymbolicSystem takes ObjectGenericDAE - its kind, Equations(), the export (#3190).
6. The massMatrixUserFunction, constraintUserFunction and jacobianUserFunction (#3191): the jacobianUserFunction gives
   the derivatives of the terms of all user functions - the ODE2 rows as $-\fv_u$ of the left-hand side, the ODE1 rows
   as $\gv_u$ of the right-hand side -, and the object calls it with the factors of each block it needs; the derivative
   of the mass matrix is not in the Jacobian, as for the other bodies. A constraint that depends on $\dot\qv$ at position
   level does not converge with generalized-alpha, a compliance $-\epsilon\tlambda$ does (the sign matters).
   SymbolicSystem refuses a massMatrixUserFunction on an object with ODE1 coordinates, as its mass matrix does not
   depend on $\yv$.
