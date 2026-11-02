# The road to the CUDA implementation

This page is the plan for evaluating Exudyn's symbolic equations in parallel by CUDA. The plan summarizes what is already there and what is missing. The plan covers Exudyn and the library Paradyn having a CUDA backend. This note collects what exists, the design, the order and the decisions needed. It lives while RG14 is open (WORKFLOW.md §8.4).

## 1. Mission

Compute a **heterogeneous multibody system** - rigid bodies, kinematic trees, flexible bodies, joints and contacts in
one model - **in parallel on a GPU, by CUDA**, with explicit and implicit solvers, for one system and for ensembles of
many states of it. Whatever the mix of items, one evaluation of the system is a fixed sequence of uniform array
operations, the same operation on many values at once. The results agree with Exudyn to a tolerance.

The route starts from what Exudyn has: the equations of a system of symbolic user items, composed by `SymbolicSystem`
as symbolic expressions.

The terminology of this note - flat program, scalar program, level, group, scheduler, launch, ... - is in the README of
the Paradyn repository.

## 2. The pipeline

| stage | what happens | code | Exudyn | Paradyn |
|---|---|---|---|---|
| **recording** | a Python function runs once with symbolic arguments; each operation creates a node of a graph | `exudyn.symbolic`, C++ | x | |
| **flat data** | `GetFlatProgramData()` writes the graph as a list, operands before their operation; a vector or matrix operation is still one entry | C++, read in Python | x | |
| **scalarizing** | every vector and matrix operation is written as scalar operations, into one graph of scalar nodes; a node with the same operation and operands is reused | `Scalarize`, Python | | x |
| **levels** | a scheduler gives each operation its level, by default the length of its longest path from the inputs; the operations of one level are independent of each other | `Levels`, `schedulers`, Python | | x |
| **schedule** | the operations are sorted by level and, within a level, by kind; each group gets the index arrays of its operands | `BuildOneArray`, Python | | x |
| **arrays** | one array holds all values, a row per node and a column per state; each group is one operation over all states | numpy, then CUDA | | x |

The columns Exudyn and Paradyn say where each stage is located. `Scalarize`, `Levels` and `BuildOneArray` have their
source in the library `src/python/paradyn` of the Paradyn repository;
[uniformEvaluationExperiment.py](../design/uniformEvaluationExperiment.py) is their first version, for the operations of
its example only.

| module of `paradyn` | contents |
|---|---|
| `scalarGraph` | `ScalarGraph`, `Scalarize`: a flat program as scalar operations |
| `uniformProgram` | the schedulers `Levels` and `LevelsAsLateAsPossible`, `BuildOneArray`, `UniformProgram`: the quantities of a system as one graph and its schedule |
| `backends` | the evaluation of a batch of states by the schedule: `numpy`, the reference; `cuda`, the kernel of `uniformKernel.cu`; `host`, the same file as loops on the CPU |
| `system` | `UniformSystem`, a `SymbolicSystemBase`, so that `SolveExplicit` of Exudyn runs on it; it keeps the `SymbolicSystem` for `Compare` |
| `solvers` | `SolveExplicitEnsemble`, RK44 of a batch of states, the time loop in Python |
| `experiments` | `Sweep`: the evaluation over all combinations of backends, schedulers, launch modes, precisions and numbers of states, timed and compared with the symbolic system |

**The table of operations and the GPU.** `src/Linalg/SymbolicTable.h` has one row per operation: 65 codes, 40 scalar, 2
of lookup tables, 23 of vectors and matrices. Every operation of a recording has a code. `SymbolicProgram` computes
them in C++ for one state, the vectors and matrices in one array of the program, without values in the nodes (#3176).
This array is not reachable from Python and is not scalarized, so it has no levels; symbolicSystem.md names it as what
the GPU and an export from C++ read, which is not designed yet. The table does not provide the parallelism: the threads
of Exudyn still evaluate a copy of a recording each (#2978). The path of section 2 needs the table when it compiles:

- `esym.Jacobian` (#3163) differentiates each operation by the rule of its row, so the programs of the iteration matrix
  come from it;
- `GetFlatProgramData()` takes the names of the operations from it;
- it lists what `Scalarize` has to cover: `paradyn.scalarGraph` has all 65 codes, each computed as Exudyn computes it, and
  raises a `ValueError` for an operation it does not know.

## 3. A simple example

The function $r = \sin(q_0)\,q_1 + \cos(q_0)\,q_2$, written with vectors:

```python
v = esym.Vector([esym.sin(q[0]), esym.cos(q[0])])
w = esym.Vector([q[1], q[2]])
r = v @ w
```

**Flat data**: the variable `q`, its components, `sin`, `cos`, the two vectors and the dot product, each one entry.

**Scalarized**: the dot product becomes a product per component and a sum, five scalar operations of the inputs `q0`,
`q1`, `q2`:

```
n1 = sin(q0)
n2 = cos(q0)
n3 = n1 * q1
n4 = n2 * q2
n5 = n3 + n4
```

**Levels and schedule**: one group per level and kind.

| level | kind | operations | operands |
|---|---|---|---|
| 1 | `cos` | `n2` | `q0` |
| 1 | `sin` | `n1` | `q0` |
| 2 | `*` | `n3`, `n4` | (`n1`, `q1`), (`n2`, `q2`) |
| 3 | `+` | `n5` | (`n3`, `n4`) |

**Arrays**: `values` has a row per input and operation and a column per state. Each group is one line, here the group
of level 2:

```python
values[start:stop] = np.multiply(values[i0], values[i1])  #i0, i1: the rows of the operands of n3 and n4
```

With 1000 states, this line computes 2000 products. A system of many different items makes the groups wider, and the
deepest item sets the number of levels: the items do not need to be alike, as all operations of a group are of one
kind. On the GPU the line is one launch, in the launch mode `groups`: n operations for m states run n*m threads in parallel, all with
the same instructions, so that a level is parallel over its operations and over the states at once.

## 4. The explicit loop: the first running example

Before the implicit path, the loop is closed once with an explicit solver: a system without algebraic equations, from
the flat programs to the time integration. It shows how the GPU behaves before Jacobians and Newton's method are added.

The example is `src/python/symbolicChainExplicit.py` of the Paradyn repository: the chain of
[uniformEvaluationExperiment.py](../design/uniformEvaluationExperiment.py), rigid bars on spring-dampers and torsional
spring-dampers, a mass point, gravity and a load of the time, built from one function with the built-in items and with
the user items. Joints, Euler parameters and contacts are not part of it.

| step | what | against | status |
|---|---|---|---|
| 1 | the simulation of both systems by `SolveDynamic`, RK44 | each other | done |
| 2 | forces and mass matrices of `SymbolicSystem` and of `UniformSystem` at random states | Exudyn's solver on the built-in items | done |
| 3 | `SolveExplicit` of Exudyn on `UniformSystem`, and an ensemble by RK44 on numpy, the mass matrix of each body solved in a batch | `SolveDynamic` | done |
| 4 | the right-hand side in CUDA: `UniformSystem(system, backend='cuda')`, the time loop still in Python | `Compare`, to a tolerance | written, to be run on the GPU server |
| 5 | the explicit solver in CUDA: RK44, the batched solve per body and the state update on the GPU, the groups launched from C++ for several steps per call, the data copied back only at output steps | step 3, to a tolerance | open |
| 6 | the measurements below | - | open |

Measured on numpy (1.13.309, Windows, one core, 200 steps of RK44, 100 states; run of 2026-11-02):

| | 1 bar (9 coordinates) | 10 bars (63 coordinates) |
|---|---|---|
| scalar operations, levels | 523, 29 | 7732, 33 |
| array operations per evaluation | 74 | 97 |
| largest relative difference to Exudyn: forces, mass matrix | 6.9e-17, 2.9e-18 | 1.1e-16, 5.8e-18 |
| member 0 against `SolveDynamic` of the built-in items | 6.7e-16 | 2.1e-15 |
| 100 states by RK44 | 0.25 to 0.35 s | 3.4 to 3.7 s |
| 1 state by `SolveDynamic` of the built-in items | 0.005 to 0.007 s | 0.04 to 0.06 s |

For 10 bars, the evaluation is 3.2 ms of the 4.0 ms per stage, the batched solve and the rest the other 0.8 ms; so the
arrays cost 32 us per state. 100 states take about 70 times one `SolveDynamic` for 10 bars and about 50 times for 1 bar,
so on one core the arrays are already 1.4 to 2 times faster than 100 separate runs.

The CUDA backend runs the schedule: `uniformKernel.cu` is compiled once by nvcc, independent of the system, and the
schedule is its data, so that a system, a scheduler or a launch mode needs no compilation. The number of parallel threads
is the number of operations of a launch times the number of states, so one large system and an ensemble run by the same
code. Step 4 copies the states per call, step 5 removes this. The backend `host` runs the same file as loops on the CPU,
a check without a GPU; lookup tables have no code in the kernel yet. Neither backend has been compiled yet.

The scheduler decides what can run together, the launch mode how it is sent to the GPU; both are independent options, to
be varied in experiments, also with new ones:

| launch mode | one launch computes | launches per evaluation, 10 bars | |
|---|---|---|---|
| `groups` | one group | 97 | no divergence; the most commands from the CPU |
| `levels` | all groups of a level, each thread selecting its operation | 33 | threads of different operations diverge in a warp, at the borders of the groups, which are sorted by kind |
| a CUDA graph | planned: all launches recorded once, started by one call | 1 call | the groups stay free of divergence; recorded again for a new schedule; to be checked against the CUDA version of the server |

The data stays on the GPU between launches in every mode; the difference is the number of commands from the CPU.

Step 6 measures by `Sweep`, over the size of the ensemble, for the schedulers, the launch modes and in double and single
precision: the size from which the GPU beats numpy and `SolveDynamic`, the cost of a launch against its operations, the
share of the evaluation against the batched solve, and the memory.

## 5. The implicit path: what exists

The ODE2 rows of the iteration matrix of generalized-alpha, for the residual
$\mathbf{r} = \mathbf{M}\ddot{\mathbf{q}} - \mathbf{f} + \mathbf{C}_{\mathbf{q}}^T\boldsymbol{\lambda}$, are
$\mathbf{K}_t + \gamma'\,\mathbf{D}_t + \beta'\,\mathbf{M}$, with the tangent stiffness and damping matrices
$\mathbf{K}_t$, $\mathbf{D}_t$ and with $\mathbf{C}_{\mathbf{q}}$ and $\mathbf{C}_{\mathbf{q}}^T$ as coupling blocks.

**Part 1: what Exudyn's solver needs.**

| term | as numbers at one state | as a program |
|---|---|---|
| $\mathbf{M}$, $\mathbf{f}$, $\mathbf{c}$, $\mathbf{C}_{\mathbf{q}}^T\boldsymbol{\lambda}$ | yes | yes, `SymbolicSystem.Equations()`: `massMatrix`, `ODE2RHS`, `algebraicEquations`, `ODE2ReactionForcesLHS` |
| $\mathbf{K}_t = -\partial\mathbf{f}/\partial\mathbf{q}$, $\mathbf{D}_t = -\partial\mathbf{f}/\partial\dot{\mathbf{q}}$ | yes, `ODE2ResidualLHSJacobian` with `q_tt = 0` | not yet |
| $\mathbf{C}_{\mathbf{q}}$ | yes, `SymbolicSystem.SystemJacobian` | not yet |

In Exudyn (`ComputeNewtonJacobian`, `CSystem::JacobianODE2RHS` in `CSystemJacobians.cpp`, assembled from local Jacobians
in parallel), $\mathbf{K}_t$ and $\mathbf{D}_t$ are the derivatives, automatic, analytic or numerical, of the forces of
the bodies and connectors, the derivative of the marker Jacobians included, without the terms of part 2. A program of
$-\partial\mathbf{f}/\partial\mathbf{q}$ of the composed $\mathbf{f}$ contains the last two rows of part 2 by itself.

**Part 2: what the symbolic solver includes and Exudyn's solver does not.**

**Warning:** `SolveGeneralizedAlpha` on a `SymbolicSystem` adds these terms to $\mathbf{K}_t$, so that
$\mathbf{K}_t = \partial\mathbf{r}/\partial\mathbf{q}$ exactly. Its Newton iterates therefore differ from those of
`SolveDynamic` and from those of programs of part 1 alone; the solutions agree to the Newton tolerance only. The
iteration matrix decides how fast Newton converges, not what it converges to.

| term | as numbers at one state | as a program | in Exudyn's solver |
|---|---|---|---|
| $\partial(\mathbf{M}\ddot{\mathbf{q}})/\partial\mathbf{q}$ | yes, `ODE2ResidualLHSJacobian` | not yet | left out |
| $\partial(\mathbf{C}_{\mathbf{q}}^T\boldsymbol{\lambda})/\partial\mathbf{q}$ | yes, `SymbolicSystem.SystemJacobian` | not yet | left out by default (RG9.11.7 plans it as an option) |
| the derivative of the loads in $\mathbf{f}$ | yes, `ODE2ResidualLHSJacobian` | not yet, comes with $\mathbf{K}_t$ of the composed $\mathbf{f}$ | left out by default, `timeIntegration.computeLoadsJacobian = 0` |
| $\mathbf{J}^T(\partial\mathbf{f}_c/\partial\mathbf{v})(\partial\mathbf{v}/\partial\mathbf{q})$ of a connector force $\mathbf{f}_c$ | yes, `ODE2ResidualLHSJacobian` | not yet, comes with $\mathbf{K}_t$ of the composed $\mathbf{f}$ | left out for the spring-dampers, where it shows in no test model; included for contacts, rolling discs and connectors with user functions |

**Part 3: what the initial accelerations need.** They solve
$\begin{bmatrix} \mathbf{M} & \mathbf{C}_{\mathbf{q}}^T \\ \mathbf{C}_{\mathbf{q}} & \mathbf{0} \end{bmatrix}
\begin{bmatrix} \ddot{\mathbf{q}} \\ \boldsymbol{\lambda} \end{bmatrix} =
\begin{bmatrix} \mathbf{f} \\ -(\mathbf{C}_{\mathbf{q}}\dot{\mathbf{q}})_{\mathbf{q}}\dot{\mathbf{q}} \end{bmatrix}$.

| term | as numbers at one state | as a program |
|---|---|---|
| $\mathbf{M}$, $\mathbf{f}$ | yes | yes, part 1 |
| $\mathbf{C}_{\mathbf{q}}$ | central differences in `_InitialAccelerations` | not yet, the program of part 1 |
| $(\mathbf{C}_{\mathbf{q}}\dot{\mathbf{q}})_{\mathbf{q}}\dot{\mathbf{q}}$ | second difference along $\dot{\mathbf{q}}$ in `_InitialAccelerations` | not yet, `esym.Jacobian` of the vector $\mathbf{C}_{\mathbf{q}}\dot{\mathbf{q}}$ by `q`, times $\dot{\mathbf{q}}$ |

Without these programs, the initial accelerations and multipliers have to be computed in Python before the solver
starts; that is a non-symbolic solve in any case.

`SolveGeneralizedAlpha` works on the numbers: `SystemJacobian`, in Python, gets them from the one-pass derivative in
C++, at the current state. That cannot move to a GPU, which needs the derivatives as scalar programs. `esym.Jacobian` gives them as
expressions of a Real or Vector, so $\partial(\mathbf{M}\ddot{\mathbf{q}})/\partial\mathbf{q}$ of part 2 takes
$\mathbf{M}\ddot{\mathbf{q}}$ as a vector with a named `q_tt`. `SymbolicSystem` uses it only for the reaction terms of ObjectGenericDAE; `Equations()` gives no Jacobians, and the size of such programs for a composed system is not measured.

Further open points of the implicit path:

- `SolveGeneralizedAlpha` computes one state: ensembles need batched LU decompositions and a convergence per member;
- a system with data coordinates, such as a contact, needs an outer loop over the post-Newton step per member, with the
  inputs `x` and `x0` and the program `Equations(postNewton=True)`;
- `SymbolicSystem` has ODE2-ODE1 couplings for ObjectGenericDAE only and refuses ObjectConnectorHydraulicActuatorSimple.

## 6. The order

**The checks.** Each stage of the compile path has to pass these, in this order, 1 and 2 at random states:

1. the quantities: the scalar graph against `MassMatrix`, `ODE2RHS`, `AlgebraicEquations` and `ODE2ReactionForcesLHS`;
2. the iteration matrix: $\mathbf{K}_t$ and $\mathbf{D}_t$ against `ODE2ResidualLHSJacobian` with `q_tt = 0`, one factor
   at a time, and $\mathbf{C}_{\mathbf{q}}$ against its block of `SystemJacobian`;
3. the time integration: generalized-alpha on the scalar graph against `SolveGeneralizedAlpha` and `SolveDynamic`.

**The stages.**

| stage | work | check | plan |
|---|---|---|---|
| 1 | **the explicit loop** of section 4, on numpy and then on the GPU, in Paradyn | section 4 | RG14.3, written for CuPy |
| 2 | the library `paradyn`, on numpy: done, with its tests in `src/python/tests`; every operation against Exudyn, except `==` and `!=` of vectors, which Python does not reach | 1 | RG14.2.2, written for `exudyn.misc` |
| 3 | the programs of section 5, parts 1 and 3, from `Equations()`; the first two rows of part 2 only if the Newton iterations ask for them | 2 | no step yet |
| 4 | `UniformSystem` with algebraic equations, so that `SolveGeneralizedAlpha` runs on it unchanged, on the pendulum | 3 | no step yet |
| 5 | MarkerBodyUser in `SymbolicSystem`, markers in the `itemNumbers` of joints and connectors | user item against built-in item | no step yet |
| 6 | **kinematic tree**: ObjectUserBody on a NodeGenericODE2, the recursion of `CObjectKinematicTree` recorded, a marker on its links | as stage 5 | no step yet |
| 7 | **Euler parameters**: the rigid body with their formulas, the algebraic equation of the node in `SymbolicSystem`; implicit only | as stage 5 | no step yet |
| 8 | **FFRF**: ObjectUserBody on the rigid and the generic node, the interface markers closed-form, for both node types | as stage 5 | RG14.6, low priority |
| 9 | **contacts**: sphere-sphere exists in `UserItemCreator`, with a data node and a recorded post-Newton step (#3187, #3197); on the scalar graph the comparisons, the inputs `x`, `x0` and the post-Newton loop, on fixed pairs | as stage 5, to a tolerance | no step yet |
| 10 | batched generalized-alpha for ensembles, on numpy | 3, per member | no step yet |
| 11 | CUDA for the demonstrators, in the Paradyn repository first, in double and single precision | against numpy, to a tolerance | RG14.3; RG14.5 gives the CPU reference from exported code |

Done since this plan was written: `CreateTorque`, `CreateCartesianSpringDamper`, `CreateRigidBodySpringDamper`,
`CreateCoordinateConstraint` (RG12.55) and the data coordinates in `SymbolicSystem` and its solvers (RG12.66).

Sums are chains, so that the results keep the bits of Exudyn. A chain has a depth of its number of terms, so sums as
balanced trees, of a depth of the logarithm of their terms, may be needed in the future where long sums occur, as in
FFRF; they give up equality to the bit with Exudyn. Stage 7 comes before stage 8 as `CreateFFRFReducedOrderObject` uses
Euler parameters by default.

**To settle with the maintainer.**

- What "the array of the program is what the GPU reads" (symbolicSystem.md) means for section 2: the array has no levels
  and serves one state.
- FFRF is low priority in RG14.6 ("not for now"), and the kinematic tree has no step, while the demonstrators need both.
- A step for stages 3, 4 and 10, the implicit uniform evaluation; RG14 has none.
- RG14.2.2, RG14.3 and the location in Paradyn: RG14.2.2 puts the compile path into `exudyn.misc`, and RG14.3 is written
  for CuPy inside Exudyn.

**Decisions.**

- The explicit loop is closed first, on the GPU; the implicit path follows.
- CUDA is tested first in the Paradyn repository; what serves every user moves to Exudyn afterwards.
- First the right-hand side in CUDA, then the explicit solver in CUDA, so that all runs in CUDA for several steps;
  synchronization only at output steps. The GPU tests run on a server with an H100.
- The GPU runs the schedule, so that the operations of a level run in parallel; schedulers and launch modes are
  independent options of the library, to be varied in experiments.
- The accuracies are compared: double and single precision, each to its own tolerance.
- The visualization of a symbolic or GPU result runs through the system of built-in items with the same coordinates, so
  that a flexible body keeps its contour colors, which the user items do not have.
- Recording and flat data are part of Exudyn; scalarizing, levels, schedule and arrays are part of the Paradyn
  repository, with the demonstrators and their visualization.

**The demonstrators** are built twice from one script, with `creator = mbs` and with
`creator = UserItemCreator(mbs)`, and compared through five levels: built-in items with `SolveDynamic`, user items with
`SolveDynamic`, `SymbolicSystem` with the Python solvers, the uniform evaluation on numpy, and CUDA. The speed is measured
against `SolveDynamic` with 1 and with N threads, as Exudyn assembles its ODE2 and constraint Jacobians in parallel
(#1203, #1202).
