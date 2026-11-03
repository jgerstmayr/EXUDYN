# Advanced solvers: evaluation (revision2026c group RG20)

A design note of open steps: the maintainer's request as given, then what exists, the evaluation per step, the open
questions and the sub-steps. It leaves the repository when the steps close (WORKFLOW.md §8.4).

## 1. The request (maintainer 2026-11-04)

A new group "Advanced solvers" (requested as RG16, which revision2026b has taken; it is RG20), with this design note and no issues before the evaluation (except step 5):

1. **GGL and a variable step size with the symbolic solvers**: `DynamicSolver.TrapezoidalGGL`, the index-3 solver based
   on the trapezoidal rule, with an adaptive step size by a local error estimator (probably based on the accelerations
   or some numerical approximation).
2. **`DynamicSolver.Lobatto4`** for the symbolic solvers: the 4th order Lobatto integrator (the trapezoidal rule with an
   additional stage at the midpoint); slower, as it doubles the size of the system, but much more accurate (4th order,
   directly stable for index 3, no tricks needed). The error estimator could be the 2nd order (trapezoidal) solution,
   or something smarter from the theory.
3. **An adjoint solver** for optimization, designed for the symbolic system: first choose whether generalized-alpha or
   TrapezoidalGGL (not adaptive) is better suited as basic solver; the adjoint solver takes a subset of the parameters
   of the symbolic system; it provides the gradient, an optimizer on top performs the optimization with a symbolic
   objective function. With examples of what could be optimized (minimum deformation, min(max(deviation from a given
   trajectory)), optimal control, minimum time to a goal, ...). Lowest priority: the step exists but is not proposed
   to be done (no issue); the ultimate test of the symbolic sensitivities by symbolic parameters later.
4. **Output interpolated within the step** for all old and new solvers, with the proper interpolation formula:
   `solution.file.writePeriod` and `solution.sensors.writePeriod` trigger the refinement; an interpolation function of
   a computed step, then the writers; always the correct write period, also in time. Shifts the results of the test
   suite.
5. **Explicit solvers with constraints**: the elimination of coordinate constraints of the C++ solver - is it in the
   symbolic solver? It would go in line with it. Euler parameters with explicit solvers by the well-known projection
   after each step; the Euler parameter constraints then do not raise in explicit integrators, with a possible drift
   of momentum and energy. An issue and a step, the analysis as first sub-step.

Further: check whether the names of steps 1 and 2 are appropriate; check what else is missing for the solvers; each
first sub-step does the analysis and a recommendation; the adaptive solvers are to be integrated into the C++ core later
(a cleanup of the C++ solvers first, then the implementations), so steps 1 and 2 are also a test and evaluation before
the internal solvers; a further step evaluates the internal solver structure (files, classes, Python bindings), to be
improved, revised, completed and made ready for the two new adaptive solvers in C++.

## 2. What exists

- **Python solvers on the symbolic system** (exudyn/symbolics/solvers.py, 537 lines): `SolveGeneralizedAlpha`
  (generalized-alpha and Newmark as CSolverImplicitSecondOrder, index 3 with Exudyn's scaling, ODE1 by the trapezoidal
  rule, initial accelerations from [M C_q^T; C_q 0]) and `SolveExplicit` (explicit Euler, RK44, velocity Verlet). Both
  on `SymbolicSystemBase`: `SystemResidual`, `SystemJacobian`, `MassMatrix`, `ODE2RHS`, `AlgebraicEquations` - with
  `velocityLevel` - and the exact Jacobians. Step size: constant, `_StepSize` reduces only on a rejection of the
  post-Newton step (`adaptiveStep`, #3220); no error control. `SolveExplicit` refuses algebraic equations.
- **C++ solvers** (src/Solver, 7200 lines): `CSolverBase` (2494 lines: initialization, the step loop, Newton,
  discontinuous iteration, output, restart), `CSolverImplicitSecondOrderTimeInt` (generalized-alpha, Newmark,
  TrapezoidalIndex2 as Newmark with index-2 reduction), `CSolverExplicitTimeInt` (Euler, midpoint, RK33, RK44, RK67,
  ODE23 and DOPRI5 with automatic step size and absolute/relative tolerance, velocity Verlet; Lie group integration),
  `CSolverStatic`; `MainSolverBase` and its Python binding; exudyn/solver.py `SolveDynamic` dispatches by
  `DynamicSolverType`.
- **Output**: `CSolverBase::FinishStep` writes the solution and the sensors when `output.lastSolutionWritten +=
  writePeriod` is passed - at the step times, not at the write times; with an adaptive step the written times are
  irregular. The Python solvers record at the steps of `solution.file.writePeriod` (as multiples of the step).
- **Explicit with constraints**: `explicit.eliminateConstraints` (C++): CoordinateConstraint objects on the ground are
  eliminated by setting the constrained coordinates and their derivatives to zero in each stage; any other constraint
  (also a coordinate constraint between two nodes) raises, and nAE != 0 without elimination warns and ignores the
  constraints. The symbolic `SolveExplicit` refuses every algebraic equation. Euler parameter nodes add an algebraic
  equation (the norm) and are refused by the explicit solvers; the rotation vector Lie group node is the explicit
  alternative.

## 3. Evaluation

### 3.1 TrapezoidalGGL with an adaptive step (step 1)

- **The method**: the trapezoidal rule (= the 2-stage Lobatto IIIA method) for q, v = q_t and the Gear-Gupta-Leimkuhler
  (GGL) stabilization: the position constraints g(q) = 0 and the velocity constraints G(q) v = 0 both enforced, with a
  second multiplier mu in q_t = v - G^T mu. The pure trapezoidal rule on index-3 constraints is unstable (it is the
  limit of generalized-alpha with rho_inf = 1, which Exudyn avoids), with GGL it is stable and of order 2 in q and v,
  the multipliers of order 1 to 2. Exudyn's `TrapezoidalIndex2` (Newmark with index-2 reduction) is related but
  drifts on position level, which GGL removes.
- **Unknowns per step**: q, v, lambda, mu (2 n + 2 m), or by eliminating v with the trapezoidal relation n + 2 m; the
  Newton matrix [M/h... ] reuses `SystemJacobian` and the constraint Jacobian C_q, both exact in the symbolic system.
- **Error estimator**: the trapezoidal rule has the local error -h^3/12 q'''; with q''' ~ (a_{n+1} - a_n)/h the
  estimate e ~ h^2/12 |a_{n+1} - a_n| per component, weighted by the absolute and relative tolerance; or the
  difference to a second-order explicit predictor (Milne's device). Both cost no extra evaluation; the acceleration difference is the standard choice and robust for mechanical systems; step size
  control h_new = h (tol/err)^(1/3) with a safety factor and limits, rejection when err > tol.
- **The name**: `TrapezoidalGGL` is appropriate (Lobatto IIIA-2 with GGL is common in the literature; "TrapezoidalGGL"
  is clearer for users). The adaptivity as `automaticStepSize`, as for ODE23/DOPRI5, with the tolerances of
  `timeIntegration` (the existing absolute and relative tolerance of the explicit solvers, or new ones).

### 3.2 Lobatto (step 2)

- **The method**: the 3-stage Lobatto IIIA method (stages at 0, h/2, h) is of order 4; for constrained mechanical
  systems the partitioned Lobatto IIIA-IIIB pair (Jay) or Lobatto IIIA with the constraints enforced at the stages
  (index-3 at the stages, GGL or the velocity constraints at the end) keeps order 4 for q and v. Lobatto IIIA alone
  on index-3 systems is not stable without the projection of the velocities - the claim "directly stable for index 3,
  no tricks" holds for the IIIA-IIIB pair (SPARK methods), not for IIIA alone; to be checked in the implementation,
  starting from Jay's formulation.
- **Cost**: the end of a step is the start of the next (FSAL), so the new stages are the midpoint and the end: 2
  stages, twice the unknowns of the trapezoidal rule - as the request says.
- **Error estimator**: the embedded trapezoidal solution from the same end values (order 2) gives an estimate of the
  order-2 error, which over-estimates the order-4 error and makes the steps too small; better: the stage values give a
  cubic (Hermite) solution with an error of order 4, and the difference of the order-4 solution to the Hermite
  extrapolation of the previous step - or the classical estimate from the difference of the Lobatto IIIA and IIIC
  solutions of the same stages. Recommended: start with the embedded trapezoidal estimate and a step size rule of
  exponent 1/3 (safe), and compare with a reference.
- **The name**: `Lobatto4` says the order but not the variant; recommended `LobattoIIIA3` (3 stages, order 4) or
  `LobattoGGL` if GGL is used for the constraints. To be decided (Q2).

### 3.3 Adjoint sensitivities (step 3, not proposed to be done)

- **Basic solver**: the discrete adjoint of a one-step method is simplest for the trapezoidal rule (GGL, constant
  step): the step map depends only on (q_n, v_n) and (q_{n+1}, v_{n+1}), the adjoint system is the transposed Newton
  matrix of the step, which the symbolic system has exactly. Generalized-alpha carries the acceleration-like variable
  a as a state, so its adjoint has more states and its initialization (consistent accelerations) enters; recommended:
  TrapezoidalGGL with constant step.
- **What it needs**: the forward solution stored (or checkpointed); per step the derivatives dr/dq, dr/dv (the Newton
  matrix) and dr/dp - `ODE2RHSParameterJacobian`, `MassMatrixParameterJacobian` (#3159) - and of the constraints by p
  (not available yet: the algebraic equations have no derivative by p); the objective J = Phi(q_N) + sum_n h L(q_n,
  v_n, p) as a symbolic function, its derivatives by esym.Jacobian.
- **Examples of optimizations**: minimum deformation or maximum stiffness of a mechanism (spring and inertia
  parameters, p of the user items); the tracking of a given trajectory, sum of squares (smooth) - min(max(deviation))
  needs a reformulation (epigraph or p-norms) for gradient methods; the identification of parameters from measured
  sensor data; optimal control with the control as parameters of a spline in time (the coefficients as p); minimum
  time to a goal by scaling the time (t = p_T s) with the end condition as constraint; minimum energy of a motion.
- **Optimizer on top**: scipy.optimize (L-BFGS-B, SLSQP) with the adjoint gradient - an optional dependency, as the
  project rules allow.

### 3.4 Output within the step (step 4)

- **Interpolation**: for the implicit second-order solvers the cubic Hermite interpolation of q from q, v at both ends
  (order 3, consistent with q and v), v by its derivative, and a linear one for the accelerations, the multipliers and
  the ODE1 coordinates; for the explicit RK methods the same Hermite cubic (DOPRI5 has a dense output of order 4,
  which could be used later). With constant steps and a write period that is a multiple of the step the results are
  unchanged; otherwise the written times become the exact multiples of the write period.
- **Writers**: the state of the system (`currentState`) is set to the interpolated values for the writers and the
  sensors, then restored - the sensors evaluate their items at that state, so items with internal history (data
  coordinates) take the end of the step; written as it is.
- **Test suite**: models with `writePeriod` not a multiple of the step change their output times (the reference
  values of models that read the last sensor value stay, as the last step is written as before).

### 3.5 Explicit solvers with constraints (step 5, #3238)

- **Coordinate constraints in the symbolic SolveExplicit**: the elimination of the C++ solver (constrained coordinates
  of CoordinateConstraints to the ground fixed) is not there - SolveExplicit refuses algebraic equations. In line with
  it: eliminate the coordinates of ObjectGenericDAE constraints that fix a coordinate (CreateCoordinateConstraint to
  the ground), or more general a linear constraint C q = c by a null-space projection of the accelerations per step.
- **Euler parameters**: the explicit stages integrate the four parameters as ODE2 coordinates without their algebraic
  equation, then the parameters are normalized after each step (projection onto |p| = 1) and their velocities
  projected onto p^T p_t = 0. The constraint does not raise then; energy and momentum drift slightly, of the order of
  the error of the method. The rotation vector Lie group node stays the recommended explicit node.

### 3.6 What else is missing for the solvers

- A BDF method (variable order, as for stiff DAEs) - not needed while generalized-alpha serves.
- Energy-momentum conserving schemes for long-time simulations of free rigid bodies - niche.
- A common step size controller (PI controller) for all adaptive solvers in C++ and Python, instead of the separate
  rules of ODE23/DOPRI5 and the post-Newton reduction.
- Sensitivities of static solutions (the derivative of the static equilibrium by p) - simple with the Jacobians by p.
- The exact restart of adaptive solvers (the step size in the restart file).

### 3.7 The C++ solver structure (a step of its own)

To be evaluated in its step: CSolverBase holds both the step loop and the Newton method and the output (2494 lines);
the implicit and explicit solvers each keep their own data; a new solver - TrapezoidalGGL, Lobatto - needs the step
loop with rejection and a step controller in the base class, the extended unknowns (mu, the stages) in the data
structures, and its own Newton residual and Jacobian. A first look recommends: the output (solution, sensors,
restart) out of CSolverBase into a class of its own, which also takes the interpolation of step 4; a step size
controller class shared by all adaptive solvers; the Newton method as a class of its own on residual and Jacobian
callbacks.

## 4. Open questions (to be decided by the maintainer)

1. TrapezoidalGGL: the error estimate by the acceleration difference (recommended) or by a predictor?
2. The name of step 2: `Lobatto4`, `LobattoIIIA3` or `LobattoGGL`, depending on the variant (IIIA with GGL, or the
   IIIA-IIIB pair)?
3. The tolerances of the adaptive implicit solvers: the existing `timeIntegration.absoluteTolerance/relativeTolerance`
   of the explicit solvers, or new fields for the implicit ones?
4. Step 4: the interpolation in C++ first (the test suite shifts once), or in the Python solvers first?
5. Step 5: in the symbolic SolveExplicit only the elimination of coordinates fixed to the ground (as C++), or the
   projection of general linear constraints?
6. Step 5, Euler parameters: in C++ and in the Python solvers, or the Python solvers first?

## 5. Steps proposed (group RG20)

- RG20.1 TrapezoidalGGL with an adaptive step in the symbolic solvers: .1 analysis (this note), .2 the constant-step
  method against generalized-alpha, .3 the error estimate and the step control, measured on a pendulum and a slider
  crank against a fine reference.
- RG20.2 Lobatto (name to be decided) in the symbolic solvers: .1 analysis, .2 the method, .3 the error estimate.
- RG20.3 Adjoint sensitivities on the symbolic system: .1 analysis (this note), .2 not proposed now.
- RG20.4 Output interpolated within the step: .1 analysis, .2 C++, .3 Python solvers.
- RG20.5 Explicit solvers with constraints (issue): .1 analysis, .2 the elimination of coordinate constraints in
  SolveExplicit, .3 Euler parameters by projection.
- RG20.6 Evaluate the structure of the C++ solvers (files, classes, Python bindings) and prepare it for adaptive implicit
  solvers.
- RG20.7 TrapezoidalGGL and Lobatto in C++, after RG20.6.
