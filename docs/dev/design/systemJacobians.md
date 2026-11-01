# The system Jacobians: design (revision2026c step RG9.11, #3201)

The Jacobian of the implicit solvers and the static solver is assembled by CSystem from four functions,
`JacobianODE2RHS`, `JacobianODE1RHS`, `JacobianAE` and `JacobianODE2Loads`. Each decides per object how its block is
computed, by flags of the object, of its markers and of the settings. Maintainer 2026-11-01: every object provides a
Jacobian - analytically, or locally by automatic or numerical differentiation of its own -, so that the assembly
distinguishes only the system-wide numerical Jacobian and the assembly of the local Jacobians, the latter in parallel
for the objects without Python functions; only ODE2-ODE2, AE-AE and ODE2-AE are optimized for speed, the other blocks
are rare, completed for all couplings and written readable rather than fast; in line with the ODE1-ODE2 couplings of
#3143; CSystem.cpp split first (#3202). This note describes what the code does now, the target, the sub-steps and
what is to be decided.

Related issues: #1203 and #1202 (the analytic ODE2 and AE Jacobians in parallel, revision2026c step RG5.3), #858 (a
cost per item for the split of the tasks), #1548 (the Jacobian of ODE1 loads, RG4.23), #988 (a sparse
`ComputeConstraintJacobianDerivative`), #3107 (the central differences of the user items), #2884 (the joints on the
relative transformation, a revision of the constraint Jacobians themselves, not of their assembly).

## 1. The blocks

The solvers need, with the factors of the integrator,

$$
\Jm = \begin{bmatrix}
  \Jm_{\SO\SO} & \Jm_{\SO\FO} & \Jm_{\SO\AE} \\
  \Jm_{\FO\SO} & \Jm_{\FO\FO} & \Jm_{\FO\AE} \\
  \Jm_{\AE\SO} & \Jm_{\AE\FO} & \Jm_{\AE\AE}
\end{bmatrix},
\qquad
\Jm_{\SO\SO} = a\,\frac{\partial \fv}{\partial \qv} + b\,\frac{\partial \fv}{\partial \dot\qv},
$$

$\fv$ the ODE2 right-hand side, $\Jm_{\SO\AE} = \Cm_{\qv}\tp$ (or $\Cm_{\dot\qv}\tp$ of a velocity-level constraint),
$\Jm_{\AE\SO} = \partial\cv/\partial\qv$ and $\partial\cv/\partial\dot\qv$, and the ODE1 blocks of #3143:
$\Jm_{\SO\FO} = \partial\fv/\partial\yv$, $\Jm_{\FO\SO}$ from the ODE1 right-hand side by $\qv$ and $\dot\qv$,
$\Jm_{\FO\FO}$, $\Jm_{\FO\AE} = -\Cm_{\yv}\tp$ and $\Jm_{\AE\FO} = \Cm_{\yv}$.

| block | rows from | where now | how now |
|---|---|---|---|
| $\Jm_{\SO\SO}$ | bodies, finite elements, connectors | `JacobianODE2RHS` | per object: analytic (`ComputeJacobianODE2_ODE2`), the connector interface (`ComputeJacobianODE2Connector`, automatic differentiation of the force, #2745), the path of the marker data (`ComputeMarkerDataStructure` with Jacobians, `ComputeJacobianODE2_ODE2`, the Jacobian derivative of the markers), or CSystem differentiates the object numerically, with the Lie group nodes apart; serial |
| $\Jm_{\SO\SO}$ | loads | `JacobianODE2Loads` | numerical per load, only with `computeLoadsJacobian`; an ODE1 dependency throws (#1548) |
| $\Jm_{\SO\SO}$ | general contact | `GeneralContact::JacobianODE2LHS` | its own, parallel |
| $\Jm_{\SO\FO}$ | objects with ODE1 dependency | `JacobianODE2RHS` | `ComputeJacobianODE2_ODE1` if available, else numerical, in two code paths (after the analytic and after the numerical ODE2 block) |
| $\Jm_{\FO\FO}$, $\Jm_{\FO\SO}$ | ODE1 objects | `JacobianODE1RHS` | `ComputeJacobianODE1RHS` (all three blocks at once) if available and not a connector, else numerical; serial |
| $\Jm_{\AE\SO}$, $\Jm_{\SO\AE}$, $\Jm_{\AE\FO}$, $\Jm_{\FO\AE}$, $\Jm_{\AE\AE}$ | constraints, bodies with AE, nodes with AE | `JacobianAE` | analytic always - the connector interface for joints (`ComputeJacobianAEConnector`), else the marker data; the transposed block chosen by `UsesVelocityLevel`; nodes (Euler parameters) by a loop of their own; serial |
| all | all | `NumericalJacobianAE`, the `doSystemWideDifferentiation` branches | system-wide numerical: `forAE` (dense only) for the AE rows, `doSystemWideDifferentiation` for the ODE2 and ODE1 rows |

Neglected by default: the derivative of $\Mm(\qv)\ddot\qv$ by $\qv$, the derivative of $\Cm_{\qv}\tp\tlambda$ by $\qv$
(computed only for the option of the generalized-alpha solver that calls `ComputeConstraintJacobianDerivative`), the
Jacobian derivative of a marker without `JacobianDerivativeAvailable` (the connector then falls back to numerical),
and the loads without `computeLoadsJacobian`. The solver chapter of the manual still says that $\Jm_{\SO\FO}$ and
$\Jm_{\FO\SO}$ vanish, which is no longer so since #3143.

The residuals and the mass matrix are assembled in parallel already: `ComputeSystemODE2RHS` over
`listComputeObjectODE2LhsNoUF` with one sparse vector per thread, then the objects with Python user functions
serially (`listComputeObjectODE2LhsUF`); `ComputeMassMatrix` over `objectsBodyWithODE2CoordsNoUF` with sparse triplets
per thread, appended serially. The Jacobians have no parallel path; `multithreadedLowerLimitJacobians` exists unused.

## 2. What is wrong with it

1. **The decision is spread out.** `ComputeObjectJacobianODE2` and the two loops of `JacobianODE2RHS` test the
   available Jacobians, the object type (connector or not), `numDiff.forODE2`, `numDiff.forODE2Connectors` (marked as
   a flag for development to be removed), `jacobianConnectorDerivative` and the marker types; `JacobianODE1RHS` and
   `JacobianAE` repeat the pattern differently. Which way an object takes is known only by reading these.
2. **CSystem differentiates objects.** The numerical Jacobian of an object is written in CSystem, with its Lie group
   variant and its column lists without duplicates (#1424), once for ODE2 and twice for the ODE1 coupling.
3. **Serial.** The analytic Jacobians of finite elements, rigid bodies and connectors could be computed in parallel
   as the residuals are; #1203 and #1202 have waited for it since 2022.
4. **Incomplete couplings.** ODE1 loads (#1548), $\Jm_{\SO\FO}$ only through the ODE2 loop, the transposed AE block
   either of position or of velocity level; each added where a model needed it.
5. **Dependencies outside the local-to-global lists.** A Python user function that reads a sensor, another item or
   the system state depends on coordinates its object does not list. The local Jacobians miss this coupling silently;
   only `doSystemWideDifferentiation` sees it.

## 3. The target

### 3.1 One Jacobian per object

Each object computes the local blocks of its equations by its local coordinates through **one virtual function** of
`CObject`, with the blocks requested and their factors:

```cpp
//! the local Jacobian of the object's equations: the blocks requested in request (ODE2, ODE1, AE rows by the ODE2,
//! ODE2_t, ODE1 and AE coordinates of its local-to-global lists), each scaled by its factor, into jacobian
virtual void ComputeLocalJacobian(const CSystemData& systemData, TemporaryComputationData& temp,
    const JacobianRequest& request, LocalJacobian& jacobian) const;
```

- `LocalJacobian` holds the blocks dense, in the local numbering, or as triplets in local numbering for large finite
  elements; the assembly maps them by the local-to-global lists.
- **The default of the base class** is the numerical differentiation of the object's own residual functions, by its
  local coordinates - what CSystem does now, moved into one place (`CObject`, with the Lie group variant), and used
  by every object that has nothing better.
- **Analytic or automatic differentiation** override it: the finite elements and rigid bodies of today, the
  connectors on the connector interface (the L2 chains of #2745 compute exactly these blocks; the markers provide the
  position and rotation Jacobians and their derivatives), the joints on `ComputeJacobianAEConnector`. The path of the
  marker data stays as the implementation of the connectors not on the interface, inside them.
- **Declared at Assemble**, not tested at every call: `GetJacobianKind()` - analytic, automatic, numerical -,
  `UsesPythonFunctions()` (exists as the UF lists), and the blocks the object has (`GetAvailableJacobians`, kept).
  `PreComputeItemLists` builds from them the lists of §3.2.

The numerical default perturbs the global state vector, as now, so that an object differentiated numerically is
not computed in parallel: two threads perturbing the coordinates of a shared node would see each other's
perturbations. It moves to the parallel list once it is analytic or differentiated automatically. A numerical
differentiation from a local copy of the coordinates would need every object to compute from given coordinates
instead of its nodes; not proposed.

### 3.2 Two ways of assembly

1. **The system-wide numerical Jacobian** (`doSystemWideDifferentiation`, and `forAE` extended to sparse matrices):
   all couplings, including those outside the local-to-global lists; serial; the reference for comparison and the
   way for models with such dependencies.
2. **The assembly of the local Jacobians**:
   - `listJacobianParallel`: analytic or automatic, no Python function - computed in `ParallelFor` with one
     `TemporaryComputationData` and one triplet list per thread, appended serially as for the mass matrix (#1203,
     #1202; the split of the tasks by a cost per object, #858);
   - `listJacobianSerial`: a Python function, the numerical default, and the objects declaring dependencies outside
     their lists - computed serially after the parallel part, as the residuals do.

   An object with dependencies outside its lists gets them by its own declaration or by `doSystemWideDifferentiation`
   (decision J3).

For a dense system matrix the per-thread triplets are added to the dense matrix serially; the cost is the number of
nonzero entries, as for the residuals (below 1 % of the time for 3D bodies at 12 threads).

### 3.3 Fast blocks and the others

- **For speed, parallel:** $\Jm_{\SO\SO}$ of objects and connectors, and the constraint blocks $\Jm_{\AE\SO}$,
  $\Jm_{\SO\AE}$ (its transposed, added by the assembly from the same local block) and $\Jm_{\AE\AE}$.
- **Serial and readable, complete:** $\Jm_{\SO\FO}$, $\Jm_{\FO\SO}$, $\Jm_{\FO\FO}$, $\Jm_{\FO\AE}$, $\Jm_{\AE\FO}$, the
  loads (with the ODE1 coupling, #1548), the nodes with algebraic equations, and the derivative terms of the mass
  matrix and of $\Cm_{\qv}\tp\tlambda$ as options. One loop per kind of row, with the request of §3.1, instead of the
  two code paths of the ODE1 coupling now.
- `GeneralContact` keeps its own parallel Jacobian.

### 3.4 Verification

A test model that builds one small system per item type, computes the local Jacobian of each item and compares it
with the system-wide numerical one for every block (by the Python access of `MainSolverBase.ComputeJacobian*`), and
lists the items whose local Jacobian is numerical. This is the check of every sub-step and makes the classification
of §3.1 visible.

## 4. The split of CSystem.cpp (#3202)

Before the revision, without changing a function:

| file | contents |
|---|---|
| `CSystemAssemble.cpp` | `Assemble`, `CheckSystemIntegrity`, `AssembleCoordinates`, `AssembleLTGLists`, `AssembleObjectLTGLists`, `UpdateUserFunctionsForThreads`, `PreComputeItemLists`, the Lie group lists, `AssembleInitializeSystemCoordinates`, `AssembleSystemInitialize` |
| `CSystem.cpp` | the mass matrix, the ODE2 and ODE1 right-hand sides, loads, algebraic equations, the post-Newton step, the reaction forces, `UpdatePostProcessData`, the per-object helpers |
| `CSystemJacobians.cpp` | `ComputeObjectJacobianODE2`, `JacobianODE2RHS`, `JacobianODE1RHS`, `JacobianODE2Loads`, `NumericalJacobianAE`, `ComputeObjectJacobianAE`, `JacobianAE`, the constraint Jacobian derivative and times vector |

The new files go into `msvc/cppsrc.vcxproj`, from which `sources.json` is generated; the timers registered in the
file move with their functions.

## 5. Sub-steps proposed

1. This design (RG9.11.1).
2. Split CSystem.cpp (#3202).
3. The verification test of §3.4 on the code as it is, so that every later step is compared with it.
4. `ComputeLocalJacobian` with the numerical default in `CObject`, the objects with analytic Jacobians and the
   connector interface on it; `JacobianODE2RHS` and `JacobianODE1RHS` as one loop over the request; same results.
5. The parallel assembly of `listJacobianParallel` (#1203, #1202, #858; replaces RG5.3), measured on the
   performance models.
6. The AE blocks on the same interface, the nodes with algebraic equations included, `forAE` for sparse matrices.
7. The couplings completed serially: the ODE1 loads (#1548, RG4.23), the derivative terms as options; the solver
   chapter of the manual.

## 6. To be decided

- **J1** One virtual `ComputeLocalJacobian` with a request, replacing `ComputeJacobianODE2_ODE2`,
  `ComputeJacobianODE2_ODE1`, `ComputeJacobianODE1RHS` and `ComputeJacobianAE` - or keeping them as the
  implementations behind it. Proposed: keep them behind it first, remove them once every object is on it.
- **J2** The numerical default in `CObject`, serial; parallel only for analytic and automatic Jacobians. Proposed.
- **J3** Dependencies outside the local-to-global lists: a flag of the object (a parameter of the Python user items,
  such as `jacobianIsSystemWide`) that puts the object into the serial list and differentiates it by all coordinates,
  or only `doSystemWideDifferentiation` for the whole system as now. Proposed: the system-wide option as now, and
  the documentation of the user functions saying so; a flag only when a model needs it.
- **J4** `forODE2Connectors`, described as a flag for development to be removed: removed with deprecation when the
  connectors are on the interface.
