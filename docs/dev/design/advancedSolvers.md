# Advanced solvers: evaluation (revision2026c group RG20)

A design note of open steps: the maintainer's request as given, then what exists, the evaluation per step, the
decisions and the sub-steps. It leaves the repository when the steps close (WORKFLOW.md §8.4).

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

## 2. What exists (at the start of RG20, and what RG20 added)

- **Python solvers on the symbolic system** (exudyn/symbolics/solvers.py): `SolveGeneralizedAlpha` (generalized-alpha
  and Newmark as CSolverImplicitSecondOrder, index 3 with Exudyn's scaling) and `SolveExplicit` (explicit Euler,
  RK44, velocity Verlet), on `SymbolicSystemBase` (`SystemResidual`, `SystemJacobian`, `MassMatrix`, `ODE2RHS`,
  `AlgebraicEquations`, exact Jacobians for a SymbolicSystem). RG20 added: the output at the exact write times,
  interpolated within the step (#3241); the elimination of fixed coordinates in `SolveExplicit` (#3238).
- **exudyn/symbolics/advancedSolvers.py** (new in RG20): `SolveTrapezoidalGGL` (#3242, #3243) and
  `SolveImplicitRungeKutta` with `LobattoIIIA4`, `LobattoIIIC4-GGL`, `Radau3` (#3244, #3246, #3247); one stage solver
  `_SolveRungeKutta` for stiffly accurate tableaus with the constraints at the end of the step or with GGL at the
  stages; the step size controller `_ErrorStepSize` (PI, RG20.8.1).
- **C++ solvers** (src/Solver, 7200 lines): `CSolverBase` (initialization, the step loop, Newton, discontinuous
  iteration, output, restart), `CSolverImplicitSecondOrderTimeInt` (generalized-alpha, Newmark, TrapezoidalIndex2),
  `CSolverExplicitTimeInt` (Euler to RK67, ODE23 and DOPRI5 with automatic step size, velocity Verlet, Lie group
  integration), `CSolverStatic`; unchanged by RG20 so far. Found: RK44 with eliminated constraints is of order 1
  (#3240); the error norm of ODE23/DOPRI5 lacks absolute values (#3245).

## 3. Evaluation

Notation: n coordinates q, velocities v = q_t, accelerations a = q_tt, m constraints c(q, t) = 0 with C_q = dc/dq,
multipliers z (lambda); M a = f(q, v, t) - C_q^T z.

### 3.1 TrapezoidalGGL (step 1, done)

- **The method** (`SolveTrapezoidalGGL`):

      v1 = v0 + h/2 (a0 + a1),      q1 = q0 + h/2 (v0 + v1) - C_q(q1)^T mu
      M(q1) a1 = f(q1, v1, t1) - C_q(q1)^T z1,      c(q1, t1) = 0,      C_q(q1) v1 + c_t = 0

  Newton for (a1, z1, mu); order 2 in q, v, a, z; no damping; the constraints hold on both levels.
- **Error estimate** (decision 1, #3243): the local errors of the trapezoidal rule, -h^3/12 q_ttt and -h^3/12 v_ttt,

      e_q = h^2/12 (a1 - a0)                                      (q_ttt ~ (a1 - a0)/h)
      e_v = h^3/12 a_tt,  a_tt = 2/(tau1 + tau2) ((m0 - m1)/tau1 - (m1 - m2)/tau2)

  with the mean accelerations m = (v1 - v0)/h of the last three steps (tau the distances of their midpoints): the end
  accelerations alternate from step to step with constraints (the trapezoidal rule does not damp that mode), their
  means do not. The norm of the explicit solvers, error = rms(e/(atol + rtol max(|x0|, |x1|))) over q and v.
- **Controller** (RG20.8.1): PI of Gustafsson, h_next = h safety (1/err)^(0.7/k) err_prev^(0.4/k), k = 3; a step of
  err > 1 repeated with h safety (1/err)^(1/k), no growth right after a rejection.

### 3.2 The 3-stage Lobatto methods and Radau3 (step 2)

- **The stage equations** (`_SolveRungeKutta`, a stiffly accurate tableau (A, b = last row, c)):

      V_i = v0 + h sum_j A_ij A_j,      Q_i = q0 + h c_i v0 + h^2 sum_j (A^2)_ij A_j
      M(Q_i) A_i = f(Q_i, V_i, t0 + c_i h) - C_q(Q_i)^T Z_i          (each implicit stage)
      q1 = Q_s,  v1 = V_s,  a1 = A_s,  z1 = Z_s

  Lobatto IIIA (c = 0, 1/2, 1): A = [[0, 0, 0], [5/24, 1/3, -1/24], [1/6, 2/3, 1/6]], the first stage the start;
  Lobatto IIIC: A = [[1/6, -1/3, 1/6], [1/6, 5/12, -1/12], [1/6, 2/3, 1/6]], all three stages implicit, L-stable;
  Radau IIA (c = 1/3, 1): A = [[5/12, -1/12], [3/4, 1/4]], order 3, L-stable. b = (1/6, 2/3, 1/6) for both Lobatto
  methods: Simpson's rule, q1 = q0 + h/6 (V_1 + 4 V_2 + V_3).
- **The constraints, measured** (pendulum, slider crank; orders of q / v / a, z):

  | where | method | q | v | a, z | velocity constraint |
  |---|---|---|---|---|---|
  | end: c(q1) = 0 and C_q v1 + c_t = 0, Z_i per stage after the start, Z_1 = z0 | IIIA (`LobattoIIIA4`) | 4 | 4 | 2 | 1e-15 |
  | | IIIC, IIIA-IIIB | 4 | 4 | 2-3 | 1e-15 |
  | | Radau IIA (`Radau3`) | 3 | 3 | 2 | 1e-15 |
  | end: c(q1) = 0 only, one Z for the step | IIIA, IIIB, IIIC, IIIA-IIIB | 4 | 2 | 1-2 | drifts (O(h^2)) |
  | GGL at each implicit stage: c(Q_i) = 0, C_q V_i = 0, Q_i - C_q^T mu_i | IIIA, IIIC (`LobattoIIIC4-GGL`) | 4 | 4 | 2 | 1e-15 |
  | | Radau IIA | 3 | 3 | 2 | 1e-15 |
  | c(Q_i) = 0 at each implicit stage only, Z_i per stage (index 3) | IIIC | 4 | 2 | 1 | 4e-4 (O(h^2)) |
  | | Radau IIA | 3 | 2 | 1 | 6e-4 |
  | | IIIA | 2 | 2 | no convergence | 1e-4 |
  | the same, v projected onto C_q v + c_t = 0 after each step | IIIC | 4 | 4 | 1 | 1e-17 |
  | | Radau IIA | 3 | 3 | 1 | 1e-17 |
  | | IIIA | 3 | 3 | 1 | 1e-17 |
  | end: c(q1) = 0 only, one Z for all implicit stages | IIIC, Radau IIA | 4, 3 | 2 | 1 | 4e-4 |
  | | IIIA | Newton fails | | | |

  (The velocity constraints of the last column: the pendulum with 100 steps on 1 s.)

  **Index 3 at the stages alone** (maintainer's question 2026-11-05: c(Q_i) = 0 at the two implicit stages of Radau
  IIA or Lobatto, without velocity constraints, would need no velocity constraints of the user items). Measured with
  the stage solver, pendulum 25/50/100 steps, slider crank 16/32 steps: it converges for Radau IIA and Lobatto IIIC,
  with the positions of the full order, but the velocities lose one (Radau) or two (Lobatto) orders and the
  multipliers one - slider crank with 32 steps, Radau IIA: the velocity error 2.7e-3 against 6.9e-6 with the
  constraints at the end, the multipliers 6e-2 against 2e-3. This is the theory (Hairer, Lubich, Roche 1989; Hairer,
  Wanner II, VII.4): in q_t = v, M v_t = f - C_q^T z, c(q) = 0 the velocities are the index-2 and the multipliers the
  index-3 components, of the global orders 2s - 1, s and s - 1 for an s-stage Radau IIA method (s = 2: 3, 2, 1), and
  no stage equation fixes the velocity constraint, which drifts by O(h^2). Lobatto IIIA does not converge at all in
  a and z: its first stage is the end of the previous step, and with |R(inf)| = 1 the errors of the algebraic
  components are not damped. A projection of v after each step onto the velocity constraint gives back the order of
  v, not that of z. One multiplier shared by the stages is no better and fails for IIIA.

  **The velocity constraint needs nothing from the items** beyond an index-3 solver: it is C_q v + c_t, with the
  Jacobian C_q that every constraint has for its reaction forces C_q^T z, and c_t, the partial derivative of the
  position constraint by time (zero unless the constraint depends on t; numerical by two evaluations at t +- eps in
  the Python solvers). No velocity-level implementation of a constraint (the `velocityLevel` variants of the C++
  joints) is used. Its Jacobian has the term H = (C_q v)_q, the derivative of C_q along v; without it - C_q only -
  the orders and errors are the same, and Newton takes 3 % (pendulum, 292 against 283 iterations) to 27 % (slider
  crank, 84 against 66) more iterations. So the constraints at the end of the step, which keep all orders, need only
  c, C_q and c_t, the same as an index-3 solver; they stay the form of `LobattoIIIA4` and `Radau3`.

  The unknowns per step (n coordinates, m constraints; maintainer's question 2026-11-04):
  - LobattoIIIA4: 2 n + 2 m. The first stage is the start of the step (its row of A is zero, A_1 = a0), so the
    stage accelerations A_2, A_3 are unknown (2 n). It is not GGL: no mu. But it has two multipliers, Z_2 and Z_3,
    because there are two constraint equations at the end, c(q1) = 0 and C_q v1 + c_t = 0 (2 m); with one Z for the
    step (2 n + m), only c(q1) = 0 can be imposed - v of order 2 and a drifting velocity constraint (the table).
  - Radau3: 2 n + 2 m, the same: two implicit stages, the same two end equations.
  - LobattoIIIC4-GGL: 3 n + 6 m. The first row of IIIC is not zero, so the first stage is implicit too (Q_1 and V_1
    differ from q0 and v0): 3 stages of A_i (3 n), each with Z_i and mu_i for c(Q_i) = 0 and C_q V_i + c_t = 0
    (6 m). 2 n + 4 m would be GGL on two implicit stages, i.e. IIIA-GGL (dropped 2026-11-04).

  A fast mode with
  h omega = 70 (a stiff spring): LobattoIIIC4-GGL and Radau3 damp it (Radau3 also damps the slow motion slightly),
  LobattoIIIA4 does not converge there and keeps the energy to 1e-4 at h omega = 14; IIIC-end gained energy.
- **Error estimates, the formulas.** The trapezoidal solutions from the stages of the same step - evaluated from the
  converged stages, without a solve and without the constraints (maintainer's proposal):

      one step:       T1 = q0 + h/2 (V_1 + V_3)
      two substeps:   T2 = q0 + h/4 (V_1 + 2 V_2 + V_3)
      Lobatto:        S  = q0 + h/6 (V_1 + 4 V_2 + V_3) = (4 T2 - T1)/3      (Simpson is the Richardson extrapolation)

      S - T1 = -h/3 D,   S - T2 = -h/12 D,   D = V_1 - 2 V_2 + V_3 ~ h^2/4 q_ttt

  (the same for v with the accelerations A_i). Both are the same second difference D, an estimate of the local
  error -h^3/12 q_ttt of the trapezoidal rule, not of the error -h^5/2880 q^(5) of Simpson's rule. This is general:
  an estimate must vanish where the method is exact, for V a polynomial of degree 2 - and the only combination of
  three values V_1, V_2, V_3 that vanishes for all quadratics is zero. **No estimate of order 4 comes from the
  stages of one step alone**; a fourth (and fifth) value is needed. RG20.2.3 used S - T1 for q and v, with the
  exponent of h^3; it controlled the step as the trapezoidal rule would - the error of Lobatto4 ended far below the
  tolerance (slider crank at 1e-6: 68 steps, 4e-10, where 16 constant steps give 1e-8).

  *The maintainer's idea, worked out* (2026-11-04): one trapezoidal step has the error h^3 E, two substeps
  2 (h/2)^3 E = h^3 E/4, E = -q_ttt/12 - both are evaluations of the formulas, no solve, as proposed. With the same
  constant E the two give E, and the extrapolation that removes it, (4 T2 - T1)/3, is Simpson's rule itself, i.e.
  exactly q1 of IIIA: q1 - S = 0 identically. The error of the order-4 method is the next term, h^5 q^(5)/2880, a
  different constant that three values of one step cannot see (they determine a quadratic, which Simpson integrates
  exactly). So the trapezoidal evaluations estimate the trapezoidal error only.
- **The error estimate of the order of the method** (RG20.2.5, #3250): the values of the previous step supply what
  the stages of one step lack. The polynomial p through the stage velocities of the previous and this step (for the
  Lobatto methods the five values at t0 - h_p, t0 - h_p/2, t0, t0 + h/2, t1, degree 4; for Radau3 the four at
  t0 - 2 h_p/3, t0, t0 + h/3, t1, degree 3) is integrated over the step and compared with the quadrature of the method:

      e_q = h sum_j b_j V_j - integral_t0^t1 p dt         (Lobatto, equal steps: e_q = h/180 (V_-2 - 4 V_-1 + 6 V_0 - 4 V_1 + V_2),
                                                            Simpson's error h^5/2880 v'''')
      e_v = the same with the stage accelerations A_j, only without constraints

  (`_QuadratureError`: the weights of the integral of the interpolating polynomial from its Vandermonde system, so
  unequal steps and Radau need no case of their own). No solve; the first step, without a previous one, by the
  trapezoidal rule T1 with order 3; then order len(points) in the controller. The accelerations of a constrained
  system are of order 2 at the stages, which spoils their fourth difference: there, only e_q enters the error norm.
- **Measured** (RG20.2.5; the true local error by a reference of 64 steps of the method from the same start - the
  step-doubling idea, used only as a reference, not in the method), estimate / true local error, median over a
  run with constant steps (N = 25, 50, 100, 200 on 1 s):

  | model, method | true e_q, N = 25 / 200 | trapezoid T1, q | e_q (this) | e_v (this) | e_v from the 5th difference of V |
  |---|---|---|---|---|---|
  | oscillator, IIIA | 4e-7 / 1e-11 | 850 ... 44000 (1/h^2) | 0.33 (all N) | 0.8 ... 1.06 | 2.4 ... 15 (1/h) |
  | pendulum, IIIA | 3e-7 / 8e-12 | 1200 ... 73000 | 0.58 (all N) | 12 ... 690 (1/h^2) | 1.7 ... 7.6 |
  | oscillator, Radau3 | 6e-6 / 2e-9 | | 0.54 ... 0.61 | | |
  | pendulum, Radau3 | 2e-5 / 4e-9 | | 0.77 ... 0.86 | | |

  The automatic step size with it (tolerances 1e-4, 1e-6, 1e-8 on 2 s; steps, global error of q):

  | model, method | trapezoid T1 (before) | this (#3250) |
  |---|---|---|
  | oscillator, IIIA | 109 / 3e-7, 518 / 9e-10, 2414 / 2e-12 | 26 / 2e-4, 59 / 6e-6, 139 / 2e-7 |
  | pendulum, IIIA | 128 / 2e-7, 598 / 5e-10, 2753 / 1e-11 | 24 / 7e-4, 52 / 2e-5, 120 / 4e-7 |
  | pendulum, IIIC4-GGL | 106 / 2e-6, 507 / 5e-9, 2365 / 2e-11 | 27 / 3e-4, 80 / 4e-6, 243 / 5e-8 |
  | pendulum, Radau3 | 103 / 1e-6, 485 / 4e-8, 2305 (51 rejected) / 6e-10 | 37 / 3e-4, 109 / 4e-6, 338 / 2e-7 |

  4 to 23 times fewer steps; the steps grow with the tolerance as tol^(-1/(p+1)); the global error is that of a
  control of the local error per step, the local errors of about the tolerance adding up over the steps. Slider
  crank at 1e-6: 12 steps, 7e-7 (68 steps, 4e-10 before).
- **Other candidates, not taken**: Lobatto IIIA against IIIC on the same step (a second solve); a defect of the
  Hermite polynomial of both ends at the midpoint (O(h^4), one order short); step doubling as the estimate (three
  solves per step) - it serves as the reference above.
- **The names** (decision 2026-11-04, third answers): `LobattoIIIA4` - the simplest form that solves index 3 without
  tricks -, `LobattoIIIC4-GGL` and `Radau3`, the methods of `SolveImplicitRungeKutta`; HOTINT presumably had
  LobattoIIIA4 or IIIC4 with the constraints at the end.

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

## 4. Decisions

Maintainer 2026-11-04 (first answers):

1. **TrapezoidalGGL**: the error estimate by the acceleration difference.
2. **Lobatto**: named by its order; IIIC with the constraints at all stages expected for index 3, compared with IIIA
   and GGL (RG20.2.2); Radau3 kept in mind.
3. **Tolerances**: `timeIntegration.absoluteTolerance` and `relativeTolerance` for all adaptive solvers.
4. **Output within the step**: the Python solvers first (RG20.4.3), then C++ (RG20.4.2).
5. **Explicit elimination**: as in C++, coordinates fixed to the ground (RG20.5.2).
6. **Euler parameters**: the Python solvers first; NodeRigidBodyEP into `SymbolicSystem` first (RG20.5.3).
7. **Python first everywhere**: the new solvers in a module of their own; C++ when settled.
8. **A common step size controller**: RG20.8.

Second answers: models of changing time scales - a whipping chain of about 10 elements and a contact sphere; the
velocity error and the PI controller; then: the chain from horizontal; the constraints at the end of the step, as
in HOTINT, without GGL; the sensitivity of a motion near an unstable position is documented only; the embedded
trapezoidal estimate first.

Third answers: `LobattoIIIA4` (constraints at the end), `LobattoIIIC4-GGL` and `Radau3` are the three methods,
IIIC-end and IIIA-GGL dropped; the formulas of the error estimates in this file and a better estimate (section 3.2).

Decided (2026-11-05): LobattoIIIA4 (the constraints at the end), LobattoIIIC4-GGL and Radau3 go into C++ next to
TrapezoidalGGL, with the automatic step size for TrapezoidalGGL and one Lobatto method; the other variants of section
3.2 are not implemented; no Python interface of the phases of the new solvers (cppSolverStructure.md section 8).

Fifth question (2026-11-05): index 3 at the stages alone, without velocity constraints - measured (section 3.2):
the positions of the full order, the velocities and the multipliers not; the end constraints kept, as they need
only c, C_q and c_t.

Fourth answers: step doubling is no method of its own, only the reference of the true local error for the
measurement; the estimate from the formulas evaluated without a solve - done with the stages of the previous step
(section 3.2, #3250); the count of the unknowns explained in section 3.2.

## 5. Results

- **TrapezoidalGGL, automatic step** (#3243): a bouncing ball (Hertz contact) needs a tenth of the steps of a
  constant step for a tenth of the error; a chain of 10 mass points falling from horizontal and whipping, as many as a
  constant step (231 steps 2.1e-4, 200 constant steps 3.7e-4); from nearly upright (80 degrees) the errors of the
  unstable start grow, the controlled step is worse than a constant one - documented. PI controller: 1 to 12
  rejections instead of up to 65.
- **Lobatto and Radau** (#3244, #3246, #3247): the table of section 3.2; the slider crank in 0.5 s with
  LobattoIIIA4 against 1.4 s with LobattoIIIC4-GGL; Lobatto4 with 32 steps 1e-8 where TrapezoidalGGL with 64 steps
  has 4e-5. The error estimate of the order of the method (#3250): 0.3 to 0.9 times the true local error at all step
  sizes, 4 to 23 times fewer steps than with the trapezoidal estimate (section 3.2). Both estimates selectable in
  both solvers by `errorEstimate` (#3253): for TrapezoidalGGL the two-step one gives the same steps (both of the
  order of the method); ODE1 and data coordinates in SolveImplicitRungeKutta. For C++, the two-step estimate needs the
  stage velocities of the previous step kept by the solver.
- **Output within the step** (#3241): no result of the test suite changed; the interpolated output of RK44 converges
  with order 4.
- **SolveExplicit with fixed coordinates** (#3238), the C++ findings #3240 and #3245.

## 6. Steps (group RG20)

| step | status | what |
|---|---|---|
| RG20.1 TrapezoidalGGL | done | constant and automatic steps, PI controller (#3242, #3243) |
| RG20.2 Lobatto4 | done | the three methods (#3244, #3246, #3247); the error estimate of the order of the method (#3250) |
| RG20.3 Adjoint sensitivities | .1 done, .2 not proposed | |
| RG20.4 Output within the step | .3 done (Python, #3241), .2 open (C++) | |
| RG20.5 Explicit solvers with constraints | done | fixed coordinates (#3238), Euler parameters, the C++ stages (#3240) |
| RG20.6 The structure of the C++ solvers | .1 done (design cppSolverStructure.md), .2 open | the analysis; the recommendations after the symbolic solvers |
| RG20.7 TrapezoidalGGL and the Runge-Kutta methods in C++ | open | after RG20.6 |
| RG20.8 Common step size controller | .1 done (Python), .2 open (C++, #3245) | |
