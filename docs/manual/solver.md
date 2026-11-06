(sec-solvers)=
# Solvers and system equations

This chapter describes the equations of motion that the system assembles from its items, their Jacobian, and the
solvers that integrate them: the explicit and implicit dynamic solvers, the static solver and the optimizers.

## Solvers in Exudyn

The user has a couple of basic solvers available in Exudyn , see {numref}`fig-available-solvers`:

- `mbs.SolveStatic(...)`: compute static solution for given problem (may also be used to compute kinematic behavior by prescribing joint motion)
- `mbs.SolveDynamic(...)`: time integration of equations of motion, by one of the solvers of {ref}`sec-solver-overview`
- `mbs.ComputeLinearizedSystem(...)`: computes the linearized system of equations and returns mass, stiffness, damping matrices
- `mbs.ComputeODE2Eigenvalues(...)`: computes the eigenvalues of the linearized system of equations; only possible if no algebraic constraints in system; uses scipy to compute eigenvalues

(fig-available-solvers)=
```{figure} /docs/figures/solversAvailableSolvers.*
:width: 650

Basic and advanced solvers in Exudyn ; advanced solvers build upon any basic solver to perform more sophisticated operations
```

There are advanced solvers, like in `exudyn.processing`:

- **Optimization**:
  - `GeneticOptimization(...)`: find optimum for given set of parameter ranges using genetic optimization; works in parallel
  - `Minimize(...)`: find optimum with `scipy.optimize.minimize(...)`

- `ParameterVariation(...)`: compute a series of simulations for given set(s) of parameters; works in parallel
- `ComputeSensitivities(...)`: compute sensitivities for certain parameters; works in parallel

The advanced methods are build upon the basic solvers and essentially run single simulations in the background, see the according examples.

The basic solvers need a `MainSystem`, usually denoted as `mbs`, to be solved. Furthermore, a couple of options are usually to be given, which are explained shortly:

- `simulationSettings`: This is a big structure, containing all solver options; note that only the according options for `staticSolver` or `timeIntegration` are used. Look at the detailed description of these options in {ref}`sec-simulationsettingsmain`. These settings influence the output rate and output quantity of the solution, solver reporting, accuracy, solver type, etc. Specifically, the `verboseMode` may be increased (2-4) to see the behavior of the solver and intermediate quantities.
- `solverType`: Only for `mbs.SolveDynamic(...)`: This is a simpler access to the solverType given in the internal structure of
  - `timeIntegration.generalizedAlpha` and
  - `simulationSettings.timeIntegration.solverType`.

The function `mbs.SolveDynamic(...)` sets the according variables internally. For available solver types, see the description of `exudyn.DynamicSolverType` in {ref}`sec-dynamicsolvertype`.

- `storeSolver`: if `True`, the solver is stored in `mbs.sys['staticSolver']` or `mbs.sys['dynamicSolver']` and also solver settings are stored in `mbs.sys['simulationSettings']`. After the solver has finished, `mbs.sys['staticSolver']` can be used to retrieve additional information on convergence, system matrices, etc. (see the solver structure).
- `showHints`: This shows a lot of possible solutions in case of no convergence
- `showCausingItems`: This shows a potential causing item if the linear solver failed; the item number is computed from the coordinate number that caused problems (e.g., a row that became zero during factorization); note that this item may not be the real cause in your problem

(sec-solver-overview)=
### Overview of the dynamic solvers

`mbs.SolveDynamic(..., solverType=exu.DynamicSolverType.<name>)` selects the solver. The orders are those of the
global error with a constant step size, for the ODE2 coordinates $\qv$ and velocities $\dot \qv$ and for the
accelerations and Lagrange multipliers $\tlambda$ of a constrained system; {ref}`sec-solver-convergence` shows them
measured.

| solver | kind | order $\qv$, $\dot \qv$ / $\tlambda$ | constraints | numerical damping | step size |
|---|---|---|---|---|---|
| `GeneralizedAlpha` | implicit, Newmark-type | 2 / 2 | index 3 | by `spectralRadius` | constant (reduced on failure) |
| `TrapezoidalIndex2` | implicit, Newmark-type | 2 / 2 | index 2, the positions drift | none | constant |
| `TrapezoidalGGL` | implicit Runge-Kutta | 2 / 2 | index 3 with GGL stabilization | none | constant or automatic |
| `LobattoIIIA4` | implicit Runge-Kutta | 4 / 2 | positions and velocities at the end of the step | none | constant or automatic |
| `LobattoIIIC4GGL` | implicit Runge-Kutta | 4 / 2 | positions and velocities at each stage (GGL) | L-stable | constant or automatic |
| `Radau3` | implicit Runge-Kutta | 3 / 2 | positions and velocities at the end of the step | L-stable | constant or automatic |
| `ExplicitEuler`, `ExplicitMidpoint`, `RK33`, `RK44`, `RK67` | explicit Runge-Kutta | 1, 2, 3, 4, 6 | `CoordinateConstraint` to the ground only | none | constant |
| `ODE23`, `DOPRI5` | explicit, embedded | 3, 5 | as above | none | automatic |
| `VelocityVerlet` | explicit | 2 | as above | none | constant |

The implicit solvers need the Jacobian of the system and solve a nonlinear system per step; they take steps far
above the stability limit of the explicit solvers and handle constraints. The ODE1 coordinates are integrated by
the trapezoidal rule in the Newmark-type solvers and by the Runge-Kutta method itself in the implicit Runge-Kutta
solvers.

(sec-systemequations)=
## The system equations

(sec-systemequationsofmotion)=
### System equations of motion

The system equations of motion in Exudyn follow the notations of {ref}`sec-nomenclatureeom` and are represented as

$$
\begin{aligned}
  \Mm \ddot \qv + \frac{\partial \gv}{\partial \qv^\mathrm{T}} \tlambda_q +
                  \frac{\partial \gv}{\partial \dot \qv^\mathrm{T}} \tlambda_{\dot q}
                  & = &\fv_\SO(\qv, \dot \qv, \yv, t) \\
  \dot \yv + \frac{\partial \gv}{\partial \yv^\mathrm{T}} \tlambda & = &\fv_\FO(\qv, \dot \qv, \yv, t) \\
  \gv(\qv, \dot \qv, \yv, \tlambda, t) &= 0 \, .
\end{aligned}
$$ (eq-system-eom)

Here, we introduce different Lagrange multipliers $\tlambda_q$ and $\tlambda_{\dot q}$ which have equal sizes as $\tlambda$, while those $\lambda_i$ which belong to holonomic constraints, are included in $\tlambda_q$ and $\tlambda_i$ belonging to non-holonomic constraints, are included in $\tlambda_{\dot q}$, whereas other components in $\tlambda_q$ or $\tlambda_{\dot q}$ are zero.

It may help to know that for linear mechanical the term $\fv_\SO$ becomes

$$
  \fv^{lin}_\SO = \fv_a - \Km \qv - \Dm \dot \qv
$$
in which $\fv^a$ represents applied forces and stiffness matrix $\Km$ and damping matrix $\Dm$ become part of the system Jacobian for time integration.

The system assembles these equations from its items:

- **bodies and finite elements** add their mass matrices to $\Mm$ and their forces to $\fv_\SO$; ObjectGenericODE1 and
  ObjectGenericDAE add ODE1 equations to $\fv_\FO$;
- **connectors** add their forces to $\fv_\SO$, through the Jacobians of their markers; a connector with ODE1
  coordinates, such as ConnectorHydraulicActuatorSimple, adds its equations to $\fv_\FO$;
- **constraints** (joints) and the Euler parameter nodes add their algebraic equations to $\gv$; the reaction forces
  $\Cm_{\qv}\tp\tlambda$ and, for objects with ODE1 coordinates in their constraints such as ObjectGenericDAE,
  $\Cm_{\yv}\tp\tlambda$ follow from the Jacobian of $\gv$;
- **loads** act on $\fv_\SO$, a LoadCoordinate on a MarkerNodeODE1Coordinate on $\fv_\FO$.

The residuals and the mass matrix are assembled in parallel with `parallel.numberOfThreads` threads; the items with
Python user functions are computed serially after the others.

(sec-systemjacobian)=
### The system Jacobian

The implicit solvers and the static solver need the Jacobian of the equations {eq}`eq-system-eom` by all coordinates,

$$
  \Jm = \mr{\Jm_{\SO\SO}}{\Jm_{\SO\FO}}{\Jm_{\SO\AE}}
           {\Jm_{\FO\SO}}{\Jm_{\FO\FO}}{\Jm_{\FO\AE}}
           {\Jm_{\AE\SO}}{\Jm_{\AE\FO}}{\Jm_{\AE\AE}} ,
  \qquad
  \Jm_{\SO\SO} = a\,\frac{\partial \fv_\SO}{\partial \qv} + b\,\frac{\partial \fv_\SO}{\partial \dot\qv} ,
$$
with the factors $a$, $b$ of the integrator. $\Jm_{\SO\AE} = \Cm_{\qv}\tp$ (or $\Cm_{\dot\qv}\tp$ of a constraint at
velocity level), $\Jm_{\AE\SO}$ the derivative of $\gv$ by $\qv$ and $\dot\qv$, $\Jm_{\FO\AE} = -\Cm_{\yv}\tp$ and
$\Jm_{\AE\FO} = \Cm_{\yv}$.

**Each item provides its own local Jacobian**, by its local coordinates, and the system adds it at the coordinates of
the item:

- most bodies and finite elements compute it analytically or by automatic differentiation; ObjectKinematicTree by
  differences of its inverse dynamics, with the derivative of $\Mm(\qv)\ddot\qv$ (#3211); ObjectGenericODE2,
  ObjectGenericODE1 and ObjectGenericDAE by their matrices, and by the jacobianUserFunction or the derivative of their
  symbolic user functions;
- **connectors** differentiate their force by the kinematics of their markers - by automatic differentiation, or by a
  numerical differentiation of the force by the marker positions, rotations and velocities, as the contacts and the
  rolling disc do -, chained with the Jacobians of the markers. The contacts with friction and the rolling disc also
  take the derivative of the marker velocities by the coordinates. Differentiating by the marker kinematics instead
  of the system coordinates leaves the system state untouched, so that connectors are computed in parallel;
- **constraints** have their Jacobians analytically, as have the pressure equations of ConnectorHydraulicActuatorSimple
  (#3214);
- the other items are **differentiated numerically by the system**, by their own coordinates - the rotation
  coordinates of the Lie group nodes by the composition of the rotation -: ObjectFFRF, ObjectFFRFreducedOrder with a
  forceUserFunction, on Lie group nodes or with `numericalJacobian`, ObjectKinematicTree with a floating base or a
  forceUserFunction, ObjectANCFBeam,
  ObjectALEANCFCable2D, ObjectUserBody, the connectors with Python user functions (ObjectConnectorUser and the
  spring-dampers with a user function), ConnectorReevingSystemSprings, the contacts on cables and curves, and
  connectors on markers that lack the derivatives the connector needs (MarkerSuperElementRigid, user markers,
  MarkerKinematicTreeRigid on a tree with a floating base).

**Parallel and serial**: with `parallel.numberOfThreads` > 1 and at least `parallel.multithreadedLowerLimitJacobians`
items, the analytic and automatic Jacobians of the ODE2 rows and of the constraints of the items without Python user
functions are computed in parallel; the numerical ones, those of items with Python user functions and the couplings
with ODE1 coordinates are computed serially. GeneralContact computes its own Jacobian in parallel.

**Options** of `newton.numericalDifferentiation` (see {ref}`sec-numericaldifferentiationsettings`):

- `doSystemWideDifferentiation`: the whole Jacobian by numerical differentiation of the system residual by every
  coordinate, serially. It includes all couplings, also those outside the coordinates of an item - a user function
  that reads a sensor, another item or the state of the system -, which the local Jacobians miss (a spring-damper
  whose user function reads the position of another body needs two instead of one Newton iteration per step without
  it); and it serves to check the local Jacobians;
- `jacobianConnectorDerivative`: the derivative of the marker Jacobians in the Jacobians of the connectors, on by
  default; without it, models with flexible bodies may need more Newton iterations;
- `relativeEpsilon` and `minimumCoordinateSize`: the step of the numerical differentiation.

**Loads** are in the Jacobian with `computeLoadsJacobian` of `timeIntegration` or `staticSolver`: 1 for the
derivatives by the coordinates, 2 also by the ODE2 velocities. A load is differentiated numerically by the coordinates
of its marker and by those declared with `mbs.systemData.AddODE2LoadDependencies` and `AddODE1LoadDependencies`, as
for a load whose user function reads other coordinates, e.g. a controller.

**Not in the Jacobian**: the derivative of $\Mm(\qv)\ddot\qv$ by $\qv$, also not in the system-wide numerical one;
it is zero for bodies with a constant mass matrix and changes the Newton iterations, not the solution. The derivative
of the reaction forces $\Cm_{\qv}\tp\tlambda$ by $\qv$ and $\dot\qv$ is added with
`newton.numericalDifferentiation.constraintReactionDerivative = True`, by a numerical differentiation of the reaction
forces of each constraint by its own coordinates, serially; it is off by default, as it seldom saves iterations and
costs one evaluation of the constraint Jacobian per coordinate (#3230).

For rigid bodies on Euler parameters, the Jacobians of the connectors and joints contain the derivative along the
Euler parameters on which the unnormalized rotation matrix depends, as the predictor of a time step leaves the unit
sphere of the parameters.

The Python interface of the solvers gives the Jacobian and its parts, e.g. `ComputeJacobianODE2RHS`,
`ComputeJacobianODE1RHS` and `ComputeJacobianAE` of a solver (see {ref}`sec-mainsolverimplicitsecondorder`).

## General solver structure

The description of solvers in this section follows the nomenclature given in {ref}`sec-generalnotation`.
Both in the static as well as in the dynamic case, the solvers run in a loop to solve a nonlinear system of (differential and/or algebraic) equations over a given time or load interval. Explicit solvers only perform a factorization of the mass matrix, but the `Newton` loop, see {numref}`fig-solver-newton-iteration`, is replaced by an explicit computation of the time step according to a given Runge-Kutta tableau.

In case of an implicit time integration, {numref}`fig-solver-time-integration` shows the basic loops for the solution process. The inner loops are shown in {numref}`fig-solver-solve-steps` and {numref}`fig-solver-discontinuous-iteration`.
The static solver behaves very similar, while no velocities or accelerations need to be solved and time is replaced by load steps.

Settings for the solver substructures, like timer, output, iterations, etc., are described in Sections {ref}`sec-csolvertimer` -- {ref}`sec-solveroutputdata`.
The description of interfaces for solvers starts in {ref}`sec-mainsolverstatic`.

(fig-solver-time-integration)=
```{figure} /docs/figures/solverTimeIntegration.*
:width: 350

Basic solver flow chart for SolveSystem(). This flow chart is the same for static solver and for time integration.
```

(fig-solver-initialize-solver)=
```{figure} /docs/figures/solverInitializeSolver.*
:width: 400

Basic solver flow chart for function InitializeSolver().
```

(fig-solver-solve-steps)=
```{figure} /docs/figures/solverSolveSteps.*
:width: 550

Flow chart for SolveSteps(), which is the inner loop of the solver.
```

(fig-solver-discontinuous-iteration)=
```{figure} /docs/figures/solverDiscontinuousIteration.*
:width: 550

Solver flow chart for DiscontinuousIteration(), which is run for every solved step inside the static/dynamic solvers. If the DiscontinuousIteration() returns False, SolveSteps() will try to reduce the step size.
```

(fig-solver-newton-iteration)=
```{figure} /docs/figures/solverNewton.*
:height: 880

Solver flow chart for Newton(), which is run inside the DiscontinuousIteration(). The shown case is valid for residualMode = 0.
```

(sec-explicitsolver)=
## Explicit solvers

Explicit solvers are in general only applicable for systems without constraints (i.e., no joints!). However, some solvers accept simple `CoordinateConstraint`, e.g., fixing coordinates to the ground.
Nevertheless, for constraint-free systems, e.g., with penalty constraints, can be solved for very high order and with great efficiency.
A list of explicit solvers is available, see {ref}`sec-dynamicsolvertype`, for an overview of all implicit and explicit solvers.

The solution vector $\txi$ (denoted as $y$ in the literature [Hairer1987]), which is defined as

$$
  \txi = [\qv\tp \;\; \dot \qv\tp \;\; \yv\tp ]\tp
$$
and which includes {ref}`ODE2 <ODE2>` coordinates and velocities and {ref}`ODE1 <ODE1>` coordinates. All coordinates are computed without reference values.

The {ref}`ODE1 <ODE1>` and {ref}`ODE2 <ODE2>` equations of {eq}`eq-systemeom`, with $\tlambda=0$, are written in explicit form and converted to first order equations,

$$
\begin{aligned}
  \dot \qv &= \vel \\
  \dot \vel & = &\Mm^{-1} \fv_\SO(\qv, \vel, t) \\
  \dot \yv & = &\fv_\FO(\yv, t) \\
\end{aligned}
$$ (eq-systemeom)

The system first order differential equations for explicit solvers thus read

$$
  \dot \txi = \fv_e (\txi, t)
$$
(sec-rungekuttamethod)=
### Explicit Runge-Kutta method

Explicit time integration methods seek the solution $\txi_{t+h}$ at time $t+h$ for given initial value $\txi_{t}$ (at the beginning of one step $t$ or at the beginning of the simulation, $t=0$),

$$
  \txi_{t+h} = \txi_{t} + \Delta \txi\, .
$$
For any given Runge-Kutta method, the integration of one step with step size $h$ is performed by an approximation

$$
  \Delta \txi = \int _{t}^{t+h}\fv_e(\tau ,\txi(\tau ))d\tau \approx h\left[b_{1} \fv_e(t,\txi(t))+b_{2} \fv_e(t+c_{2} h,\txi(t+c_{2} h))+ \ldots +b_{s} \fv_e(t+\txi_{s} h,u(t+\txi_{s} h))\right]
$$ (s-stage-quadrature)

in which $t + c_{i}h$ is the time for stage $i$ and $b_i$ the according weight given in the integration formula.
Stages are within one step (therefor called one-step-methods), where $c_i=0$ represents the beginning of the step and $c_i=1$ the end.
Note that $c_{1}= 0$ for explicit integration formulas.

The unknown solution vectors $\txi$ at the stages are abbreviated by

$$
  \gv_{i} \approx \txi(t+c_{i} h)
$$
and computed by explicit integration (quadrature) formulas of lower order ($g_i$ not to be mixed up with algebraic equations!),

$$
  \begin{array}{l}
  {\gv_{1} =\txi_t} \\
  {\gv_{2} =\txi_t+ha_{21} \fv_e(t,\gv_{1} )} \\
  {\gv_{3} =\txi_t+h\left[a_{31} \fv_e(t,\gv_{1} )+a_{32} \fv_e(t+c_{2} h,\gv_{2} )\right]} \\
  {{\rm \; \; \; \; \; \; }\vdots } \\
  {\gv_{s} =\txi_t+h\left[a_{s1} \fv_e(t,\gv_{1} )+a_{s2} \fv_e(t+c_{2} h,\gv_{2} )+ \ldots +a_{s,s-1} \fv_e(t+c_{s-1} h,\gv_{s-1} )\right]} \end{array}
$$ (eq-expl-rk-stages)

After all vectors $\gv_i$ have been consecutively evaluated, the step is updated by {eq}`s-stage-quadrature`.

For exemplary tableaus of explicit and implicit Runge-Kutta methods, see the references of the
method in `exudyn.utilities` and the standard literature on Runge-Kutta schemes.

### Automatic step size control

Advanced solvers, such as `ODE23` and `DOPRI5`, include automatic step size control (activated with
`timeIntegration.automaticStepSize = True` in simulationSettings).

We estimate the error of a time step with current step size $h$ by
using an embedded Runge-Kutta formula, which includes two approximations {eq}`s-stage-quadrature` of order $p$ and $\hat p = p-1$, which is obtained by using two different integration formulas with common coefficients $c_i$, but two sets of weights $b_i$ and $\hat b_i$, leading to two approximations $\txi$ and $\hat \txi$. These so-called embedded Runge-Kutta formulas are widely used, for details see Hairer et al. [Hairer1987].

The according apporximations $\txi$ and $\hat \txi$ are used to estimate an error

$$
  e_j=|\xi_j- \hat \xi_j|
$$
for every component $j$ of the solution vector $\txi$.
A scaling is used for every component of the solution vector, evaluating at the beginning ($0$) and end ($1$) of the time step:

$$
  s_j = a_{tol} + r_{tol} \cdot \mathrm{max}(|\xi_{0j}|, |\xi_{1j}|)
$$
Then the relative, scaled, scalar error for the step, which needs to fulfill $err \le 1$, is computed as

$$
  err = \sqrt{\frac 1 n \sum_{j=1}^n \left( \frac{\xi_{1j} - \hat \xi_{1j}}{s_j} \right)^2}
$$
A step with $err \le 1$ is accepted, otherwise it is repeated at once with a smaller step size. The step sizes come
from the PI controller of Gustafsson, which also serves the implicit Runge-Kutta solvers: after an accepted step, the
next one reads

$$
  h_{new} = \mathrm{min}\left(h_{max}, h \cdot f_{maxInc}, \mathrm{max}\left(h_{min}, f_{sfty} \cdot h \cdot
  \left(\frac{1}{err} \right)^{0.7/(q+1)} err_{prev}^{0.4/(q+1)} \right) \right)
$$
with $err_{prev}$ the error of the previous accepted step. The first accepted step and a rejected step use the
elementary rule $f_{sfty} \cdot h_{opt}$ with

$$
  h_{opt} = h \cdot \left(\frac{1}{err} \right)^{(1/(q+1))} ,
$$
and after a rejection the step size does not grow. The maximum step size is $h_{max} = \frac{t_{end} - t_{start}}{n_{steps}}$,
the minimum step size $h_{min}$ is given in the `timeIntegration` `simulationSettings`; a step at $h_{min}$ is accepted
with a warning. The factor $f_{maxInc}$ limits the increase of the current step size $h$, the factor $f_{sfty}$ is a
safety factor for limiting the chosen step size relative to the optimal one in order to avoid frequent step
rejections.
For more details, see Hairer et al. [Hairer1987].

### Stability limit

Note that there are hard limitations for every explicit integration method regarding the step size. Especially for stiff systems (basically with high stiffness parameters and small masses, but also with restrictions to damping), the **step size** $h$ **has an upper limit**: $h < h_{lim}$. Above that limit the method is inherently unstable, which needs to be considered both for constant and automatic step size selection.

### Explicit Lie group integrators

All explicit solvers including the automatic step size solvers (DOPRI5, ODE23) have been equiped with Lie group integration functionality, see Holzinger et al. [HolzingerArnoldGerst2023].

Basically, the integration formulas, see {ref}`sec-rungekuttamethod` are extended for special rotation parameters.
Lie group integration is currently only available for `NodeRigidBodyRotVecLG` used in `ObjectRigidBody` (3D rigid body).
`FFRFreducedOrder` will be extended to such nodes in the near future.
To get Lie group integrators running with rigid body models, all 3D node types need to be set to `NodeRigidBodyRotVecLG` and
set `explicit.useLieGroupIntegration = True`.

### Constraints with explicit solvers

Explicit solvers generally do not solve for algebraic constraints, except for very simple `CoordinateConstraint`.
All connectors having the additional `type=Constraint`, see the according object in {ref}`sec-item-objectconnectorspringdamper`ff.,
are in general not solvable by explicit solvers.
Currently, only `CoordinateConstraint` with one coordinate fixed to ground can be accounted for,
if `explicit.eliminateConstraints == True`.
However, this offers the great flexibility to compute finite elements (imported meshes or ANCF beams) to be (partially) fixed to ground.
A `CoordinateConstraint` that fixes a coordinate with index $j$ to ground leads to the simple algebraic {ref}`ODE2 <ODE2>` equation

$$
  g_j(\qv) = 0 \quad \Leftrightarrow \quad  q_j = \bar q_j(t)
$$
with $\bar q_j$ the value its equation `factor1*value1 - value0 - offset = 0` gives - 0 without an offset -, which the
explicit solvers set in every stage and at the end of every step, its time derivative, by `offsetUserFunction_t`, as
$\dot q_j$, and $\ddot q_j = 0$ (#3240). So the Runge-Kutta methods keep their order with eliminated coordinates. A
`CoordinateConstraint` with `velocityLevel=True` is not eliminated and raises.

NOTE that, if `explicit.eliminateConstraints == False`, constraints are ignored by the explicit solver (and all algebraic variables are set to zero). This may be wanted (e.g. to investigate the free motion of bodies), but in general leads to wrong and meaningless solution.

(sec-implicittrapezoidalsolver)=
## Implicit trapezoidal rule-based, Newmark and Generalized-alpha solver

This solver represents a class of solvers, which are -- in the undamped case -- based on the implicit trapezoidal rule (in the view of Runge-Kutta methods). The interpolation of the quantities for one step includes the start and the end value of the time step, thus being called trapezoidal integration rule. In some special cases in Newmark's method [Newmark1959], the interpolation might only depend on the start value or the end value.

For now, all implemented solvers can be viewed as a generalization of Newmark's method, but there are called differently in the solver interfaces

- **Implicit trapezoidal rule** (Newmark with $\beta = \frac 1 4$ and $\gamma = \frac 1 2$)
- **Newmark's method** [Newmark1959]
- **Generalized**-$\alpha$ **method** ($=$ generalized Newmark method with additional parameters), see Chung and Hulbert [Chung1993] for the original method and Arnold and Brüls [Arnold2007] for the application to multibody system dynamics.

### Newmark and Generalized-alpha method

Newmark's method has two parameters $\beta$ and $\gamma$.
The main ideas are given in the following.
First, displacements and velocities are linearly interpolated using the accelerations $\ddot \qv$ of the beginning of the time step (subindex '0') and the end of the time step (subindex 'T').
The $2^\mathrm{nd}$ order differential equations displacements and velocities and for $1^\mathrm{st}$ order differential equations coordinates are given by (definition of $\aalg$ will become clear later):

$$
\begin{aligned}
  \qv_T & = &      \qv_0 + h \dot \qv_0 + h^2 (\frac 1 2 -\beta) \aalg_0 + h^2 \beta \aalg_T\\
  \dot \qv_T & = & \dot \qv_0 + h (1-\gamma) \aalg_0 + h\gamma \aalg_T\\
  \yv_T & = & \yv_0 + h (1-\gamma_\FO) \vel^0_\FO + h\gamma_\FO \vel^T_\FO
\end{aligned}
$$ (eq-newmark-interpolation)

Hereafter, the system equations are solved at the end of the time step ($T$) for the unknown accelerations as well as for $1^\mathrm{st}$ order differential equations and algebraic equations coordinates.

 Remarks:

- The system of equations may be solved for accelerations $\ddot \qv$, but also for displacements $\qv$ or even velocities as unknowns while the remaining quantities are reconstructed from {eq}`eq-newmark-interpolation`. In case of displacements as unknowns, a scaling of the Jacobian is necessary, see later.
- For consistency reasons, one may set $\gamma_\FO = \gamma$, but **currently we use** $\gamma_T = \frac 1 2$, leading to no numerical damping for {ref}`ODE1 <ODE1>` variables $\yv$.
- In the extension to the so-called generalized-$\alpha$ method [Chung1993], algorithmic accelerations $\aalg$ are used in {eq}`eq-newmark-interpolation`.
- Algorithmic accelerations are no longer equivalent to the time derivatives of displacements, $\aalg \neq \ddot \qv$; thus, both sets of variables are used independently. In case of Newmark or the implicit trapezoidal rule just use $\aalg = \ddot \qv$.
- Implicit solvers are also available with Lie groups, if according rigid body nodes (`NodeRigidBodyRotVecLG`) are used, for theory see Holzinger et al. [HolzingerArnoldGerst2023]. The implicit Runge-Kutta solvers (`TrapezoidalGGL`, `LobattoIIIA4`, `LobattoIIIC4GGL`, `Radau3`) compose the rotation of stage $i$ with the increment $\Delta_i = h \sum_j a_{ij} \Tm^{-1}(\Delta_j) \tomega_j$ of Runge-Kutta-Munthe-Kaas, $\tomega_j$ the angular velocities of stage $j$ and $\Tm$ the tangent operator, by a fixed-point iteration; with $\Tm = \Im$ the order would be 2 (#3290).

For generalized-$\alpha$, the algorithmic accelerations $\aalg$ are computed from the recurrence relation

$$
   (1-\alpha_m)\av_T + \alpha_m \av_0 = (1-\alpha_f) \ddot \uv_T + \alpha_f \ddot \uv_0
$$
which can be resolved for the unknown $\av_T$,

$$
  \av_T = \frac{(1-\alpha_f) \ddot \uv_T + \alpha_f \ddot \uv_0 - \alpha_m \av_0}{(1-\alpha_m)}
$$
For the first step, one can simply use $\aalg_0 = \ddot \qv_0$.

(sec-parametersgeneralizedalpha)=
### Parameter selection for Generalized-alpha

Compared to alternative implicit integration methods (including the Newmark method), the generalized-$\alpha$ integrator's parameters break down to one single parameter $\rho_\infty$, which allows to chose numerical damping in a practical way.

Based on a simple single DOF mass-spring-damper model [Bauchau2011], having the eigen frequency $\omega = 2\pi f$ with frequency $f$ and period $T=1/f$, the spectral radius $\rho$ for the integrator defines the amount of damping for a given step size $h$ related to $T$, thus using the dimensionless step size $\bar h=h/T$.

In {numref}`fig-spectralradius` the spectral radius is shown versus $\bar h$ for various spectral radii at infinity $\rho_\infty$.
Here, $\rho_\infty$ specifies the numerical damping of very time step for large step sizes (or very high frequencies). An amount of $\rho_\infty=0.9$ means that high frequency parts of the system (($\bar h \gg 1$); high compared to the step rate) are damped to $90\%$ in every step, reducing an initial value $1$ to $2.66e-5$ after 100 steps, which is already much larger than usual physical damping in many cases.

Furthermore, low frequency parts of the system ($\bar h \ll 1$) receive almost no numerical damping, see again {numref}`fig-spectralradius`.
Exemplarily, consider $\rho$ a low frequency situation with different $\rho_\infty$:

- $\rho(\bar h=0.01, \rho_\infty=0.9) = 1 - 1.13\cdot 10^{-9}$
- $\rho(\bar h=0.01, \rho_\infty=0.6) = 1 - 1.22\cdot 10^{-7}$

which shows that numerical damping is very low for moderately small step sizes (100 steps for one oscillation).

Obviously, $\rho_\infty$ does not have a large influence for very high or low frequencies in the system as long as it is $\neq 1$ and we could even use $\rho_\infty=0$.
Regarding differential algebraic equations (DAEs), $\rho_\infty<1$ allows to integrate index 3 DAEs. Typically a value of $\rho_\infty=0.7$ leads to a stable integration, but values depend on the structure of the multibody system.

Once having chosen $\rho_\infty$, all other parameters follow automatically [Chung1993], regarding the $\alpha$s

$$
  \alpha_m = \frac{2 \rho_\infty - 1}{\rho_\infty + 1}, \quad
  \alpha_f = \frac{\rho_\infty}{\rho_\infty + 1}
$$
and Newmarks's parameters,

$$
  \gamma = \frac{1}{2} - \alpha_m + \alpha_f, \quad
  \beta = \frac{1}{4}(1- \alpha_m + \alpha_f)^2
$$
(fig-spectralradius)=
```{figure} /docs/figures/spectralRadiusZeta0.*
:width: 400

Spectral radius for generalized-$\alpha$ method depending on dimensionless step size $\bar h=h/T$, in which $T$ is the period of an equivalent single DOF mass-spring-damper system.
```

### Newton iteration

Thus, the residuals at the end of the time step ($T$) read (put all terms to {ref}`LHS <LHS>`):

$$
\begin{aligned}
  \rv^\GA_\SO &= \Mm \ddot \qv_T + \frac{\partial \gv}{\partial \qv^\mathrm{T}} \tlambda_T - \fv_\SO(\qv_T, \dot \qv_T, t) = 0\\
  \rv^\GA_\FO &= \dot \yv_T + \frac{\partial \gv}{\partial \yv^\mathrm{T}} \tlambda_T - \fv_\FO(\yv_T, t) = 0\\
  \rv^\GA_\AE &= \gv(\qv_T, \dot \qv_T, \yv_T, \tlambda_T, t) = 0
\end{aligned}
$$ (eq-generalizedalphares)

We consider two options for $2^\mathrm{nd}$ order differential equations: (A) solve for unknown accelerations $\acc_T$,  or (B) for unknown displacements $\qv_T$.

#### (A) Solve for unknown accelerations

The unknowns for the Newton method then are

$$
  \txi^\GA_{k+1} = \vr{\acc_T}{\yv_T}{\tlambda_T}
$$ (eq-newton-unknowns1)

and at the beginning of the step, we have

$$
  \txi^\GA_{k} = \vr{\acc_0}{\yv_0}{\tlambda_0}
$$ (eq-newton-unknowns2)

For the Newton method, we need to compute an update for the unknowns of {eq}`eq-newton-unknowns1`, using the previous residual $\rv_{k}$ and the inverse of the Jacobian $\Jm_{k}$ of Newton iteration $k$,

$$
  \txi^\GA_{k+1} = \txi^\GA_{k} - \Jm^{-1} \left( \txi^\GA_{k} \right) \cdot \rv^\GA \left( \txi^\GA_{k} \right)
$$
The Jacobian has the following $3 \times 3$ structure,

$$
  \Jm = \mr{\Jm_{\SO\SO}}{\Jm_{\SO\FO}}{\Jm_{\SO\AE}}
           {\Jm_{\FO\SO}}{\Jm_{\FO\FO}}{\Jm_{\FO\AE}}
           {\Jm_{\AE\SO}}{\Jm_{\AE\FO}}{\Jm_{\AE\AE}}
$$
whose blocks the items provide as described in {ref}`sec-systemjacobian`; the couplings $\Jm_{\SO\FO}$ and
$\Jm_{\FO\SO}$ of {ref}`ODE2 <ODE2>` and {ref}`ODE1 <ODE1>` coordinates come from the objects and loads that couple them.

With respect to the unknowns of the Newton method, the terms read:

$$
\begin{aligned}
  \Jm_{\SO\SO}&=\frac{\partial \rv^\GA_\SO}{\partial \acc}
               = \frac{\partial \rv^\GA_\SO}{\partial \qv} \frac{\partial \qv}{\partial \acc}
                 + \frac{\partial \rv^\GA_\SO}{\partial \dot \qv} \frac{\partial \dot \qv}{\partial \acc}
               = h^2 \beta \Km + h \gamma \Dm
 \\
  \Jm_{\SO\AE}&=\frac{\partial \rv^\GA_\SO}{\partial \tlambda}
               = \frac{\partial \gv}{\partial \qv} \quad (\mbox{or } \frac{\partial \gv}{\partial \dot \qv} \mbox{ for constraints at velocity level)} \\
  \Jm_{\FO\FO}&=\frac{\partial \rv^\GA_\FO}{\partial \yv} \\
  \Jm_{\AE\SO}&=\frac{\partial \rv^\GA_\AE}{\partial \acc}
               = \frac{\partial \gv}{\partial \acc}
               = \frac{\partial \gv}{\partial \qv} \frac{\partial \qv}{\partial \acc} +
                 \frac{\partial \gv}{\partial \dot \qv} \frac{\partial \dot \qv}{\partial \acc}
               = h^2 \beta \frac{\partial \gv}{\partial \qv}
                 + h \gamma \frac{\partial \gv}{\partial \dot \qv}
 \\
  \Jm_{\AE\FO}&=\frac{\partial \rv^\GA_\AE}{\partial \yv} \\
  \Jm_{\AE\AE}&=\frac{\partial \rv^\GA_\AE}{\partial \tlambda}
               = \frac{\partial \gv}{\partial \tlambda}
\end{aligned}
$$
Note that the derivative of the mass matrix term in $\Jm_{\SO\SO}$ is **neglected** - except in the Jacobian of
ObjectKinematicTree (`jacobianMassMatrixDerivative`, #3211) -, and the derivative of the constraint Jacobian term unless `constraintReactionDerivative` is set; both are usually of minor influence. The first is zero for a constant mass matrix; three rigid bodies on Tait-Bryan angles spinning fast take 614 instead of 417 Newton iterations in 200 steps with full Newton, the same solution (test model `symbolicMassMatrixDerivativeTest.py`, #3228). Jacobians of state-dependent loads are included with `computeLoadsJacobian` in static or time integration solvers, see {ref}`sec-systemjacobian`.

Once an update $\qv^\mathrm{Newton}_{k+1}$ has been computed, the interpolation formulas {eq}`eq-newmark-interpolation` need to be evaluated before the next residual and Jacobian can be computed.

#### (B) Solve for unknown displacements

This approach is similar to the previous approach and follows exactly the algorithm given by Arnold and Brüls [Arnold2007], however, extended for {ref}`ODE1 <ODE1>` variables, which are integrated by the (undamped) trapezoidal rule. The trapezoidal rule starts from the derivatives
of the ODE1 coordinates at the initial state, their right-hand side minus the reaction terms of the initial algebraic
coordinates, which keeps the ODE1 coordinates at order 2 (#3258).
Documentation will be added lateron.

### Initial accelerations

For the solvers based on the implicit trapezoidal rule, initial accelerations are necessary in order to significantly increase the accuracy
of the first time step.
For this reason, the constraints $\gv(\qv_0, \dot \qv_0, \yv_0, \tlambda_0, t) = 0$ in {eq}`eq-system-eom` are differentiated w.r.t. time,

$$
  \dot \gv(\qv_0, \dot \qv_0, \yv_0, \tlambda_0, t) =
  \frac{\partial \gv}{\partial \qv} \dot \qv_0 +
  \frac{\partial \gv}{\partial \dot \qv}\ddot \qv_0 +
  \frac{\partial \gv}{\partial \yv} \dot \yv_0 +
  \frac{\partial \gv}{\partial \tlambda} \dot \tlambda +
  \frac{\partial \gv}{\partial t} = 0 \, .
$$ (eq-initialaccelerationsvel)

Currently, we assume $\frac{\partial \gv}{\partial \tlambda} = 0$ for all further derivations on initial accelerations.
For velocity level constraints, {eq}`eq-initialaccelerationsvel` is used to extract initial accelerations $\ddot \qv_0$,

$$
  \frac{\partial \gv}{\partial \dot \qv}\ddot \qv_0 =
    -\frac{\partial \gv}{\partial \qv} \dot \qv_0
    -\frac{\partial \gv}{\partial \yv} \dot \yv_0
    -  \frac{\partial \gv}{\partial t} \, .
$$
Finally, the equations for the computation of the initial accelerations read for velocity level constraints,
note that $\yv_{init}$ are the nodal initial values for $\yv$,

$$
  \mr{\Mm}{\Null}{\frac{\partial \gv}{\partial \dot \qv^\mathrm{T}}}
     {\Null}{\Im}{\Null}
     {\frac{\partial \gv}{\partial \dot \qv}}{\Null}{\Null}
     \vr{\ddot \qv_0}{\yv_0}{\tlambda_0}
   = \vr{\fv_\SO(\qv_T, \dot \qv_T, t)}{\yv_{init}}
        {-\frac{\partial \gv}{\partial \qv} \dot \qv_0-\frac{\partial \gv}{\partial \yv} \dot \yv_0 - \frac{\partial \gv}{\partial t}}  \, ,
$$ (eq-initialaccelerationsvelb)

The {ref}`ODE1 <ODE1>` term $\frac{\partial \gv}{\partial \yv} = 0$ is not used yet in constraints.

For position level constraints, we assume $\frac{\partial \gv}{\partial \dot \qv} = 0$ and $\frac{\partial \gv}{\partial \yv} = 0$ in {eq}`eq-initialaccelerationsvel` and perform a second derivation w.r.t. time,

$$
  \ddot \gv(\qv_0, \dot \qv_0, \yv_0, \tlambda_0, t) =
  \frac{\partial^2 \gv}{\partial \qv^2} \dot \qv_0^2 +
  2 \frac{\partial^2 \gv}{\partial \qv \partial t} \dot \qv_0 +
  \frac{\partial \gv}{\partial \qv} \ddot \qv_0 +
  \frac{\partial^2 \gv}{\partial t^2} = 0 \, .
$$ (eq-initialaccelerationspos)

For position level constraints, {eq}`eq-initialaccelerationspos` is used to extract initial accelerations $\ddot \qv_0$,

$$
  \frac{\partial \gv}{\partial \qv} \ddot \qv_0 =
  - 2 \frac{\partial^2 \gv}{\partial \qv \partial t} \dot \qv_0
  - \frac{\partial^2 \gv}{\partial \qv^2} \dot \qv_0^2
  - \frac{\partial^2 \gv}{\partial t^2} \, .
$$
Finally, the equations for the computation of the initial accelerations for position level constraints read

$$
  \mr{\Mm}{\Null}{\frac{\partial \gv}{\partial \qv^\mathrm{T}}}
     {\Null}{\Im}{\Null}
     {\frac{\partial \gv}{\partial \qv} }{\Null}{\Null}
     \vr{\ddot \qv_0}{\yv_0}{\tlambda_0}
   = \vr{\fv_\SO(\qv_T, \dot \qv_T, t)}{\yv_{init}}
        {- 2 \frac{\partial^2 \gv}{\partial \qv \partial t} \dot \qv_0 - \frac{\partial^2 \gv}{\partial \qv^2} \dot \qv_0^2  - \frac{\partial^2 \gv}{\partial t^2}}  \, ,
$$ (eq-initialaccelerationsposb)

The linear system of equations, either {eq}`eq-initialaccelerationsvelb` or {eq}`eq-initialaccelerationsposb`, is solved prior to an implicit time integration if

- `simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True`,

which is the default value.

The term $\frac{\partial^2 \gv}{\partial \qv^2} \dot \qv_0^2$ is the second derivative of the constraints along the
motion with constant velocities: a central difference of fourth order of $\frac{\partial \gv}{\partial \qv} \dot \qv_0$
at the positions moved by $\pm \varepsilon \dot \qv_0$ and $\pm 2 \varepsilon \dot \qv_0$. The rotation of a Lie group
node is moved by the composition with its angular velocity, and the constraint Jacobian of Euler parameters includes
its part along the parameters (#3289).

The other terms come from the residuals of the constraints along the motion and in time,
$\mathbf{G}(\sigma, \tau) = \gv(\qv_0 + \sigma \dot \qv_0, \dot \qv_0, t_0 + \tau)$, by central differences of fourth order:
at velocity level $\frac{\partial \gv}{\partial \qv} \dot \qv_0 + \frac{\partial \gv}{\partial t} = \mathbf{G}_\sigma + \mathbf{G}_\tau$,
at position level $2 \frac{\partial^2 \gv}{\partial \qv \partial t} \dot \qv_0 + \frac{\partial^2 \gv}{\partial t^2} =
2 \mathbf{G}_{\sigma\tau} + \mathbf{G}_{\tau\tau}$ (#303, #574). The step in $\tau$ is $10^{-3}$ of the simulated time, at most $10^{-3}$;
the user functions of the constraints, such as an `offsetUserFunction`, are therefore also evaluated up to two such steps
before the start time. A constraint that does not depend on the time gives the same residuals at every $\tau$, and its
terms by the time are zero.

(sec-solver-initialconstraints)=
### Consistent initial conditions

The initial positions $\qv_0$ and velocities $\dot \qv_0$ shall fulfill the constraints: $\gv(\qv_0) = \Null$ for
position-level constraints, and the velocity-level form $\gv_v(\qv_0, \dot \qv_0) = \Null$ - the time derivative
$\frac{\partial \gv}{\partial \qv} \dot \qv_0$ of a position-level constraint, or the equation of a velocity-level
constraint such as a rolling disc or a prescribed velocity. If they do not, the first steps of an implicit time
integration enforce them by a jump: the constraint forces become large and oscillate, and the Newton method may need
smaller steps. A typical cause is an initial velocity given to one body of a mechanism but not to the bodies connected
to it, or a constraint that prescribes a velocity different from the initial one.

At the start of the time integration, the implicit solver evaluates both levels and warns if the largest residual
exceeds the tolerance, naming the item:

- `simulationSettings.timeIntegration.generalizedAlpha.checkInitialConstraints = True` (default) checks and warns;
  `False` switches the check and the warning off;
- `simulationSettings.timeIntegration.generalizedAlpha.initialConstraintsTolerance = 1e-8` (default) is the tolerance,
  relative to 1 plus the largest initial coordinate, respectively velocity coordinate.

The residuals are in the output of the solver, `initialConstraintsResidualPosition` and
`initialConstraintsResidualVelocity` (-1 if not computed), see `mbs.sys['dynamicSolver'].output` after
`mbs.SolveDynamic(..., storeSolver=True)`; `exudyn.special.solver.initialConstraintsViolations` counts the time
integrations that started from inconsistent initial conditions. If a constraint does not provide its velocity-level
equations, the check says so and the simulation continues.

With

- `simulationSettings.timeIntegration.generalizedAlpha.projectInitialVelocities = True`

the solver projects the initial velocities onto the constraints: it adds the change $\Delta \dot \qv$ with the least
kinetic energy that makes the velocity-level residual zero,

$$
  \mp{\Mm}{\frac{\partial \gv}{\partial \qv^\mathrm{T}}}{\frac{\partial \gv}{\partial \qv}}{\Null}
  \vp{\Delta \dot \qv}{\tmu} = \vp{\Null}{-\gv_v(\qv_0, \dot \qv_0)} \, , \quad
  \dot \qv_0 \leftarrow \dot \qv_0 + \Delta \dot \qv \, ,
$$ (eq-projectinitialvelocities)

with the matrix of the initial accelerations, for which $\frac{\partial \gv}{\partial \dot \qv}$ takes the place of
$\frac{\partial \gv}{\partial \qv}$ for velocity-level constraints. The initial positions are not corrected; they must
fulfill the position-level constraints, which the check reports.

(sec-solver-constraintdrift)=
### Drift of the constraints

The solver fulfills the constraints at the level it is given: the default generalized-alpha method the position-level
constraints $\gv(\qv) = \Null$ in every step, to the Newton tolerance; with velocity-level (index 2) constraints -
`generalizedAlpha.useIndex2Constraints = True`, or the solver `TrapezoidalIndex2` - their time derivative only, so that
the positions drift away from $\gv(\qv) = \Null$, typically growing with time and with the step size. The drift of the
Euler parameter norm of a rigid body node belongs to it.

With `simulationSettings.timeIntegration.generalizedAlpha.checkConstraintDrift = True` (default False), the solver
evaluates the position-level constraints every `constraintDriftCheckPeriod` steps (default 20); the equations of
velocity-level constraints - a rolling disc, a `CoordinateConstraint` with `velocityLevel = True` - do not drift and are
left out:

- above 0.001 `constraintDriftTolerance` it warns once, naming the item, and reports the largest drift at the end of the
  run;
- above `constraintDriftTolerance` (default $10^{-5}$, in the units of the constraint equations) it stops with an
  exception: reduce the step size, use the index 3 formulation, or raise the tolerance;
- the largest drift, its item and its time are in the output of the solver, `constraintDriftMaximum`,
  `constraintDriftItem`, `constraintDriftTime`.

`mbs.SolveDynamic(..., solverType = exu.DynamicSolverType.TrapezoidalIndex2)` switches the check on for its run, unless
`index2CheckConstraints = False` is given.

The drift in the current state, e.g., in a user function: `mbs.ComputeConstraintResiduals()` returns the residuals of
all algebraic equations of the system and the item of each, `velocityLevel = True` those at velocity level;
`mbs.ComputeItem(item, exu.ComputeItemType.ConstraintDrift)` those of one constraint or node; every constraint has the
output variable `ConstraintEquation`; and `SystemConstraintDrift` of `exudyn.systemUtilities` gives the largest drift
and the largest residual at velocity level and records them with a sensor, see {ref}`sec-systemquantities`:

```python
sDrift = SystemConstraintDrift(mbs).AddSensor()     #after mbs.Assemble()
mbs.Assemble()
```

(sec-solver-implicitrungekutta)=
## Implicit Runge-Kutta solvers

The solvers `TrapezoidalGGL`, `LobattoIIIA4`, `LobattoIIIC4GGL` and `Radau3` are implicit Runge-Kutta methods for
the constrained equations of motion {eq}`eq-system-eom`, in one solver class with the method given by its tableau
(#3261). They share the Newton iteration over all stages, the error estimates and the step size controller of
{ref}`sec-explicitsolver`.

### Stage equations

A method of $s$ stages with the coefficients $a_{ij}$, $c_i$ - stiffly accurate, the last stage being the end of the
step - has the stage accelerations $\mathbf{A}_i$ as unknowns, from which the stage velocities and positions follow,

$$
\begin{aligned}
  \mathbf{V}_i &= \dot \qv_0 + h \sum_j a_{ij} \mathbf{A}_j , \\
  \mathbf{Q}_i &= \qv_0 + h c_i \dot \qv_0 + h^2 \sum_j (\Am^2)_{ij} \mathbf{A}_j ,
\end{aligned}
$$
and at each implicit stage the equations of motion hold,
$\Mm(\mathbf{Q}_i) \mathbf{A}_i = \fv_\SO(\mathbf{Q}_i, \mathbf{V}_i, t_0 + c_i h) - \Cm_{q}(\mathbf{Q}_i)\tp \tlambda_i$. The end of the step is
$\qv_1 = \mathbf{Q}_s$, $\dot \qv_1 = \mathbf{V}_s$. The ODE1 coordinates have stage derivatives of their own, the data coordinates
are updated by the post-Newton step as in the other solvers. The tableaus:

- `LobattoIIIA4`: Lobatto IIIA with $c = (0, \frac 1 2, 1)$; the first stage is the start of the step, so two stages
  are implicit; $\qv_1 = \qv_0 + \frac h 6 (\mathbf{V}_1 + 4 \mathbf{V}_2 + \mathbf{V}_3)$ is Simpson's rule. Order 4, no numerical damping.
- `LobattoIIIC4GGL`: Lobatto IIIC with the same $c$, all three stages implicit; L-stable, it damps fast modes.
- `Radau3`: Radau IIA with $c = (\frac 1 3, 1)$, two implicit stages; order 3, L-stable, it also damps the slow motion
  slightly.
- `TrapezoidalGGL`: the trapezoidal rule, $\dot \qv_1 = \dot \qv_0 + \frac h 2 (\ddot \qv_0 + \ddot \qv_1)$ and
  $\qv_1 = \qv_0 + \frac h 2 (\dot \qv_0 + \dot \qv_1) - \Cm_q\tp \tmu$; order 2 in all quantities, no damping.

### Constraints

An index 3 solver must fix the positions and the velocities on the constraints; where it does so decides the order
of the velocities and multipliers:

- `LobattoIIIA4` and `Radau3` impose $\gv(\qv_1, t_1) = \Null$ and the velocity constraint
  $\Cm_q \dot \qv_1 + \gv_t = \Null$ at the end of the step, with a multiplier $\tlambda_i$ per implicit stage. Both
  hold to the Newton tolerance, nothing drifts, and the velocities keep the order of the positions; the
  multipliers and accelerations converge with order 2.
- `TrapezoidalGGL` and `LobattoIIIC4GGL` use the stabilization of Gear, Gupta and Leimkuhler (GGL): both constraint
  levels at each implicit stage, the positions moved onto $\gv = \Null$ by a second multiplier $\tmu_i$ along
  $\Cm_q\tp$.

The velocity constraint needs nothing from the items beyond the Jacobian $\Cm_q$ of the reaction forces: it is
$\Cm_q \dot \qv + \gv_t$, with $\gv_t$ the derivative of a constraint by the time. A constraint at velocity level - a
rolling disc, a `CoordinateConstraint` with `velocityLevel = True` - enters with its own equation at the implicit
stages. The unknowns per step are $2 n + 2 m$ for `LobattoIIIA4` and `Radau3`, $3 n + 6 m$ for `LobattoIIIC4GGL`,
$n$ coordinates and $m$ constraints. Lie group nodes are treated as in {ref}`sec-implicittrapezoidalsolver`, the
rotations of the stages by Runge-Kutta-Munthe-Kaas (#3290).

The unknowns are the stage accelerations, and a position constraint changes with $h^2$ times them; its equation is
therefore divided by $h^2$ in the Newton iteration. A position constraint is evaluated only to the rounding of the
coordinates, which this turns into increments of the accelerations of about $\varepsilon |\qv| / h^2$ in every
iteration; with the Jacobian of the end of the step at all stages they leave a residual of the equations of motion
that grows as the step shrinks. A residual of a position constraint below $16 \varepsilon \max_j |q_{\mathrm{ref},j} +
q_j|$, $\varepsilon$ the machine precision, therefore counts as zero, and the Newton iteration reaches tolerances of
$10^{-12}$ also with steps of $10^{-5}$ (#3307).

### Error estimates and automatic step size

With `timeIntegration.automaticStepSize = True` the step size follows the controller of {ref}`sec-explicitsolver`,
with the tolerances `absoluteTolerance` and `relativeTolerance` and the error norm of the explicit solvers. The
error of a step comes from the converged stages, without a further solve; `timeIntegration.errorEstimate` selects it
(#3287):

- `'oneStep'`: the trapezoidal rule of the first and the last stage against the solution of the method,
  $\ev_q = \qv_1 - (\qv_0 + \frac h 2 (\mathbf{V}_1 + \mathbf{V}_s))$, and the same for the velocities with the stage
  accelerations. It estimates the local error of the trapezoidal rule, order 3. For `TrapezoidalGGL`, which is the
  trapezoidal rule, the estimate is $\ev_q = \frac{h^2}{12} (\ddot \qv_1 - \ddot \qv_0)$ for the positions, and for
  the velocities the second difference of the mean accelerations $(\dot \qv_1 - \dot \qv_0)/h$ of the last three
  steps (the end accelerations alternate from step to step with constraints, their means do not).
- `'twoSteps'`: the quadrature of the method against the integral of the polynomial through the stage velocities of
  the previous and of this step - for the Lobatto methods five values, for `Radau3` four. It estimates the local
  error of the method itself, of the order of the method plus one (for Lobatto with equal steps
  $\ev_q = \frac h {180} (\mathbf{V}_{-2} - 4 \mathbf{V}_{-1} + 6 \mathbf{V}_0 - 4 \mathbf{V}_1 + \mathbf{V}_2)$, Simpson's error); the first step, without a
  previous one, uses the one-step estimate. The stage accelerations of a constrained system are of order 2 only, so
  the velocities enter the error only without constraints.
- `'default'`: `twoSteps` for the Runge-Kutta methods, `oneStep` for `TrapezoidalGGL`.

The stages of one step alone cannot estimate an error of order 4: three stage values determine a quadratic, which
Simpson's rule integrates exactly. With `'oneStep'` the Lobatto methods therefore take about the steps of the
trapezoidal rule and end with errors far below the tolerance; `'twoSteps'` takes 4 to 23 times fewer steps for the
same tolerance.

```python
simulationSettings.timeIntegration.automaticStepSize = True
simulationSettings.timeIntegration.absoluteTolerance = 1e-6
simulationSettings.timeIntegration.relativeTolerance = 1e-6
mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.LobattoIIIA4)
```

(sec-solver-convergence)=
## Convergence and efficiency of the dynamic solvers

The figures of this section are computed by `python/testing/solverConvergenceFigures.py` on the small models of
`python/TestModels/modelLibrary.py` (#3305). Each solver runs with the steps $2 h_0, h_0, \ldots, h_0/16$, where
$h_0$ is the step of the model's fast run. The error is taken at the end time per group of coordinates - the ODE2
coordinates, their velocities, the ODE1 coordinates and the algebraic coordinates (the Lagrange multipliers) - as the L2 norm of the
difference to a reference solution, divided by the L2 norm of the group in the reference. The reference is
`LobattoIIIC4GGL` (`RK67` without constraints) at $h_0/128$, extrapolated by Richardson. The Newton tolerances are
`relativeTolerance` $10^{-10}$ and `absoluteTolerance` $10^{-10}$, which set the floor of about $10^{-12}$ that the
methods of order 4 reach. The CPU time is that of `SolveDynamic` on one core, the smallest of three runs; for models
this small it includes a share of fixed costs.

(fig-solver-convergence-mean)=
```{figure} /docs/figures/solverStudy/convergence_mean.png
:width: 100%

The mean convergence of the implicit solvers over the models of the following figures: at each relative step
$h/h_0$ the geometric mean of the relative errors of all models that have the group; a mean is left out where a
solver failed on one of the models (`TrapezoidalIndex2` at the larger steps)
```

{numref}`fig-solver-convergence-mean` shows the orders of {ref}`sec-solver-overview`: 2 for the Newmark-type
solvers and `TrapezoidalGGL`, 3 for `Radau3` and 4 for the Lobatto solvers in the ODE2 coordinates, their velocities
and the ODE1 coordinates; the
Lagrange multipliers converge with order 2 in all solvers, with errors 10 to 100 times smaller in the Lobatto and
Radau solvers than in generalized-alpha. The orders measured between $h_0$ and $h_0/4$ on the single models are 1.95 to
2.0 for generalized-alpha, 3.99 to 4.37 for `LobattoIIIA4` and 2.87 to 3.01 for `Radau3`, the same within 0.1 for the
velocities - except on the spatial slider crank, whose errors of the Runge-Kutta solvers reach the floor.

(fig-solver-efficiency-mean)=
```{figure} /docs/figures/solverStudy/efficiency_mean.png
:width: 70%

Mean efficiency of the implicit solvers: the geometric mean over the models of the relative error of the ODE2
coordinates against that of the CPU time, relative to generalized-alpha with the step $h_0$; the points are the
steps $2 h_0 \ldots h_0/16$
```

A step costs about 2 (`TrapezoidalGGL`), 3 (`LobattoIIIA4`, `Radau3`) and 9 (`LobattoIIIC4GGL`) times a step of
generalized-alpha. The higher order pays off nevertheless, {numref}`fig-solver-efficiency-mean`: at 1.6 times the
CPU time of generalized-alpha with $h_0$, `LobattoIIIA4` with $2 h_0$ has a 600 times smaller error; below a
relative error of about $10^{-4}$ it is the most efficient of these solvers on these models. Generalized-alpha remains the choice where the accuracy of
the order 2 suffices, where its numerical damping is wanted, and for models with contact, friction or other
discontinuities, whose errors do not follow the order of a method anyway.

(fig-solver-efficiency-models)=
```{figure} /docs/figures/solverStudy/efficiency_models.png
:width: 100%

Error against CPU time on four models, with constant steps (solid) and with automatic step size for the
tolerances $10^{-3} \ldots 10^{-8}$ (dashed); the explicit solvers only on the kinematic tree, which has no
constraints
```

With automatic step size, {numref}`fig-solver-efficiency-models`, the Runge-Kutta solvers with the estimate
`twoSteps` lie on the curves of their constant steps or below, where the motion allows larger steps; `TrapezoidalGGL`
lies on its own curve. Without constraints, the explicit `DOPRI5` is the most efficient solver as long as the step is
not limited by stability, as on the kinematic tree.

(fig-solver-convergence-explicit)=
```{figure} /docs/figures/solverStudy/convergence_explicit.png
:width: 80%

The explicit solvers `RK44` and `RK67` with constant steps on the models without constraints, against
generalized-alpha and `LobattoIIIA4`: orders 4 and 6 down to the floor of the reference
```

The errors of the single models, the groups of coordinates in subplots:

```{figure} /docs/figures/solverStudy/convergence_nPendulum.png
:width: 80%

A pendulum of four mass points on distance constraints
```

```{figure} /docs/figures/solverStudy/convergence_rigidBodyJoints.png
:width: 80%

A chain of three rigid bodies (Euler parameters) on revolute joints
```

```{figure} /docs/figures/solverStudy/convergence_rigidBodySpringDampers.png
:width: 80%

The same chain on rigid body spring-dampers; the algebraic coordinates are those of the Euler parameters
```

```{figure} /docs/figures/solverStudy/convergence_sliderCrank3D.png
:width: 80%

The spatial slider-crank of the IFToMM benchmark with Lie group nodes; the errors of the Runge-Kutta solvers reach
the floor of $10^{-12}$ at the smaller steps
```

```{figure} /docs/figures/solverStudy/convergence_kinematicTree.png
:width: 60%

A kinematic tree of three links with PD control, without constraints
```

```{figure} /docs/figures/solverStudy/convergence_FFRFreducedOrder.png
:width: 80%

A flexible pendulum, ObjectFFRFreducedOrder with four modes, on a revolute joint
```

```{figure} /docs/figures/solverStudy/convergence_genericDAE.png
:width: 100%

ObjectGenericDAE: a pendulum in Cartesian coordinates with a first order lag on an ODE1 coordinate; the ODE1
coordinate converges with the order of the solver
```

## Optimization and parameter variation

The real benefit of powerful multi-body simulation emerges only if combined with modern but also simple analysis and evaluation methods.
Therefore, Exudyn has been integrated into the Python language, which offers a virtually unlimited number of methods of post-processing, evaluation and optimization.
In this section, two methods that are directly integrated into Exudyn are revisited.

Both write their results to the file given as `resultsFile` while they run, and such a run is
usually long. `python -m exudyn monitor --last` shows that file as it grows, see
{ref}`sec-resultsmonitor`.

(sec-parametervariation)=
### Parameter variation

Parameter variation is one of the simplest tools to evaluate the dependency of the solution of a problem on certain parameters. This usually requires the computation of an objective (goal, result) value for a single computation (e.g, some error norm, maximum vibration amplitude, maximum stress, maximum deflection, etc.) for every computation. Furthermore, it needs to be run for a set of parameters, e.g., using a `for` loop.
While this could be done manually in Exudyn , it is recommended to use built-in functions, which simplify evaluation and postprocessing and directly enable parallelization.
The according function `ParameterVariation(...)`, see {ref}`sec-processing-parametervariation`, performs a set of multi-dimensional parameter variations using a dictionary that describes the variation of parameters. See also `parameterVariationExample.py` in the `Examples` folder for a simple example showing a 2D parameter variation. The function `ParameterVariation(...)` requires the `multiprocessing` Python module which enables simple multi-threaded parallelism and has been tested for up to 80 cores on the LEO4 supercomputer at the University of Innsbruck, achieving a speedup of 50 as compared to a serial computation.

(sec-optimization)=
### Genetic optimization

In engineering, we often need to find a set of unknown, independent parameters $\xv \in \Rcal^n$, $\xv$ being denoted as design variables and $\Rcal^n$ as design space. Sometimes, the design space is further subjected to constraints $\gv(\xv)=0$ as well as inequalities $\hv(x) \le 0$, which are not considered here. For simple solutions for constrained optimization problems using penalty methods, see the introductory literature [Kiusalaas2013].

Optimization problems are written in general in the form

$$
  \min\limits_{\xv} f(\xv), \quad \xv \in \Rcal^n \, ,
$$
where $f(\xv)$ denotes the *objective function* (=*fitness function*). If we would like to maximize a function $\bar f(\xv)$, simply set $f(\xv)=-\bar f(\xv)$.

In engineering, the optimization problem could seek model parameters, e.g., the geometric dimensions and inertia parameters of a slider crank mechanism, in order to achieve smallest possible forces at the supports.
Another example is the identification of unknown physical parameters, such as stiffness, damping of friction. This can be achieved by comparing measurement and simulation data (e.g., accelerations measured at relevant parts of a machine). Lets assume that $\epsilon(t)$ is an error computed in every time step of a computation, then we can set the objective (=fitness) function, e.g., as

$$
  f(x) = \frac 1 T \sqrt{\int_{t=0}^T \epsilon(t)^2 dt}
$$
as the integral over the error $\epsilon$ between measurement and simulation data.
In general, a parameter variation would be sufficient to compute sufficient computations for all combinations within the design space, however, a 3D design space with 100 variations into every direction (e.g., varying the unknown damping coefficient between 1 and 100, etc.) would already require 1000.000 computations, which in an ideal case of 1 second/computation leads to almost 2 weeks of computation time.

As an alternative stochastic methods can be use to compute only the objective function for a smaller set of randomly generated design variables, which usually show regions with better parameters (lower $f$) in scatter plots.

**Genetic algorithms**[Goldberg1989; Whitley1994] can significantly reduce the necessary amount of objective function evaluations in order to perform the optimization. Genetic identification algorithms have been already successfully applied to multibody system dynamics[Eder2014].

The general structure of a (canonical) genetic algorithm is depicted in {numref}`fig-geneticoptimization`.

(fig-geneticoptimization)=
```{figure} /docs/figures/geneticOptimization.*
:width: 300

Basic solver flow chart genetic algorithm / optimization.
```

For details, see the cited literature. Here, we focus on the implementation of the function
`GeneticOptimization(...)`, see {ref}`sec-processing-geneticoptimization`.
The initial population (step 1) is created with `initialPopulationSize` individuals with uniformly distributed random design variables $[\xv_0, \ldots, \xv_{n_{pi}-1}]$  ($\xv_i=[x_{i0}, x_{i1}, \ldots]$ being a set of genes, with single genes $x_{i0}$, $x_{i1}$, ...) in the search space, which is given in the dictionary `parameters`. Herafter (steps 2-6), we iteratively process a population for a certain `numberOfGenerations` generations.

In step 3, the surviving individuals $S_s$ with best fitness (smallest value from evaluation of `objectiveFunction`) are selected and considered further in the optimization. If the `distanceFactor` is used, the surviving individuals must be located within a certain distance (measured relative to the range of the search space) to all other surviving individuals. This option guarantees the search within several local minima, while a conventional search often converges to one single minimum.
Crossover (step 4)  is performed using a crossover of all available parameters of two randomly selected parents when generating children from the surviving individuals. The crossover of genes is performed only for a part of the new population, defined by `crossoverAmount`.

Finally, in step 5, we apply mutation to all genes, which extends the search to the surrounding design space of the individuals created by crossover. The mutation could be performed by means of certain distribution functions in order to focus on the currently best search regions. However, in the current implementation of `GeneticOptimization(...)` we simply use a uniform random variable to distribute the genes over a certain percentage of the design space, which is reduced in every generation defined by the `rangeReductionFactor` $r_r$. This allows us to restrict further search to a smaller subregion of the design space and in general allows a reduction of search space by means of $r_r^{n_g}$. In the ideal case, using sufficiently large population sizes and being lucky with the found random values, a range reduction factor $r_r=0.7$ reduces the search space by a factor of $100$ after every 13 generations, allowing to obtain 4 digits of accuracy for design variables after 26 generations for suitable optimization problems.

It should be noted that still this optimization method is based on random values and thus may fail occasionally for any problem case. In order to get reproducible results, set `randomizerInitialization` to any integer value (simply: 0) in order to get identical results for repeated runs. Setting the latter variable guarantees that the Python (numpy) randomizer creates the same series of random values for initial population, mutation, etc.
