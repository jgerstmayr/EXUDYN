(sec-solvers)=
# Solvers

This section provides and overview on most important solvers. Explicit and implicit dynamic solvers, as well as optimizers are described in more detail.

## Solvers in Exudyn

The user has a couple of basic solvers available in Exudyn , see {numref}`fig-available-solvers`:

- `mbs.SolveStatic(...)`: compute static solution for given problem (may also be used to compute kinematic behavior by prescribing joint motion)
- `mbs.SolveDynamic(...)`: time integration of equations of motion
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

### System equations of motion

The system equations of motion in Exudyn follow the notations of {ref}`sec-nomenclatureeom` and are represented as

$$
\begin{aligned}
  \Mm \ddot \qv + \frac{\partial \gv}{\partial \qv^\mathrm{T}} \tlambda_q +
                  \frac{\partial \gv}{\partial \dot \qv^\mathrm{T}} \tlambda_{\dot q}
                  & = &\fv_\SO(\qv, \dot \qv, t) \\
  \dot \yv + \frac{\partial \gv}{\partial \yv^\mathrm{T}} \tlambda & = &\fv_\FO(\yv, t) \\
  \gv(\qv, \dot \qv, \yv, \tlambda, t) &= 0 \, .
\end{aligned}
$$ (eq-system-eom)

Here, we introduce different Lagrange multipliers $\tlambda_q$ and $\tlambda_{\dot q}$ which have equal sizes as $\tlambda$, while those $\lambda_i$ which belong to holonomic constraints, are included in $\tlambda_q$ and $\tlambda_i$ belonging to non-holonomic constraints, are included in $\tlambda_{\dot q}$, whereas other components in $\tlambda_q$ or $\tlambda_{\dot q}$ are zero.

It may help to know that for linear mechanical the term $\fv_\SO$ becomes

$$
  \fv^{lin}_\SO = \fv_a - \Km \qv - \Dm \dot \qv
$$
in which $\fv^a$ represents applied forces and stiffness matrix $\Km$ and damping matrix $\Dm$ become part of the system Jacobian for time integration.

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
The optimal step size then reads

$$
  h_{opt} = h \cdot \left(\frac{1}{err} \right)^{(1/(q+1))}
$$
Currently we use the suggested step size as

$$
  h_{new} = \mathrm{min}\left(h_{max}, \mathrm{min}\left(h \cdot f_{maxInc},  \mathrm{max}(h_{min}, f_{sfty} \cdot h_{opt}) \right) \right)
$$
With the maximum step size $h_{max} = \frac{t_{start} - t_{end}}{n_{steps}}$ and the minimum step size $h_{min}$, given in the `timeIntegration`
`simulationSettings`.
The factor $f_{maxInc}$ limits the increase of the current step size $h$, the factor $f_{sfty}$ is a safety factor for limiting the chosen step size relative to the optimal one in order to avoid frequent step rejections.
If $h_{new} \le h$, the current step is accepted, otherwise the step is recomputed with $h_{new}$.
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
  g_j(\qv) = 0 \quad \Leftrightarrow \quad  q_j = 0
$$
which can be solved by the implemented explicit solvers by just setting $q_j = 0$ previously to every computation and $\dot q_j = 0$ after every {ref}`RHS <RHS>` evaluation.

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
- Implicit solvers are also available with Lie groups, if according rigid body nodes (`NodeRigidBodyRotVecLG`) are used, for theory see Holzinger et al. [HolzingerArnoldGerst2023].

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
      = \mr{\Jm_{\SO\SO}}{\Null}{\Jm_{\SO\AE}}
           {\Null}{\Jm_{\FO\FO}}{\Jm_{\FO\AE}}
           {\Jm_{\AE\SO}}{\Jm_{\AE\FO}}{\Jm_{\AE\AE}}
$$
in which we consider $\Jm_{\FO\SO}$ and $\Jm_{\SO\FO}$ to vanish in the current implementations, which means that coupling of {ref}`ODE1 <ODE1>` and {ref}`ODE2 <ODE2>` coordinates is only possible due to algebraic equations.

The remaining terms in the Jacobian are currently (or by default settings) evaluated as:

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
Note that some parts of the Jacobian are **neglected**, such as mass matrix and constraint Jacobian terms in $\Jm_{\SO\SO}$, which are usually of minor influence. Furthermore, Jacobians for state-dependent loads are neglected except for system-wide numerical Jacobians or if `computeLoadsJacobian` in static or time integration solvers is set True.

Once an update $\qv^\mathrm{Newton}_{k+1}$ has been computed, the interpolation formulas {eq}`eq-newmark-interpolation` need to be evaluated before the next residual and Jacobian can be computed.

#### (B) Solve for unknown displacements

This approach is similar to the previous approach and follows exactly the algorithm given by Arnold and Brüls [Arnold2007], however, extended for {ref}`ODE1 <ODE1>` variables, which are integrated by the (undamped) trapezoidal rule.
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

The term $\frac{\partial \gv}{\partial t}$ can only occur in case of user functions and therefore currently not implemented, and the {ref}`ODE1 <ODE1>` term $\frac{\partial \gv}{\partial \yv} = 0$ is not used yet in constraints.

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
