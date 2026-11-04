(sec-symbolicsolvers)=
# Symbolic solvers

`exudyn.symbolics.solvers` integrates a symbolic system (a `SymbolicSystem` or a `NumpySystem`) with the algorithms of
Exudyn's C++ solvers, written in Python with numpy and scipy:

- `SolveExplicit(system, simulationSettings, solverType)`: explicit Euler, RK44 and velocity Verlet, the accelerations
  by the inverse mass matrix of each body or by the decomposition of the mass matrix; algebraic equations that fix
  one coordinate each - a coordinate constraint to the ground, as `UserItemCreator.CreateCoordinateConstraint` makes
  it - are eliminated with `timeIntegration.explicit.eliminateConstraints`, as Exudyn's explicit solver eliminates
  CoordinateConstraints to the ground; the coordinate keeps the value of its equation, also in the stages, and any
  other algebraic equation is refused (#3238); rigid bodies on Euler parameters are integrated with their constraint
  on acceleration level, and the parameters are projected onto the unit norm and their velocities onto its tangent
  after each step, so that the norm holds to rounding - energy and momentum drift in the order of the error of the
  method (#3238);
- `SolveGeneralizedAlpha(system, simulationSettings)`: the generalized-alpha method and the Newmark method, with ODE1
  coordinates and index 3 constraints, the Newton method with the exact Jacobian of the system.

`exudyn.symbolics.advancedSolvers` has solvers that Exudyn's C++ core does not have:

- `SolveTrapezoidalGGL(system, simulationSettings)`: the trapezoidal rule with the stabilization of Gear, Gupta and
  Leimkuhler (GGL): the position and the velocity constraints both hold in every step, a second multiplier moves the
  positions back onto the constraints. It is of order 2 in the coordinates, velocities, accelerations and Lagrange
  multipliers and has no numerical damping. With `timeIntegration.automaticStepSize` (the default) the step size
  follows the local errors of the coordinates and velocities, estimated from the accelerations - the coordinates
  by $h^2/12\,(\ddot{\mathbf q}_1 - \ddot{\mathbf q}_0)$, the velocities by the second difference of the mean
  accelerations of the last three steps -, with `timeIntegration.absoluteTolerance` and `relativeTolerance`, as the
  explicit solvers DOPRI5 and ODE23, and a PI controller, which rejects few steps (#3243). A motion of changing time
  scales - a ball bouncing on the ground - needs a tenth of the steps of a constant step size for the same error, a
  chain falling from horizontal and whipping as many. The tolerances are those of one step: where a motion is
  sensitive to small changes - a chain starting near its upright position - the global error grows far beyond
  them, as no local estimate sees the sensitivity; tighter tolerances help. The algebraic equations must depend on the
  coordinates and the time only (#3242).
- `SolveImplicitRungeKutta(system, simulationSettings, method)`: implicit Runge-Kutta methods with two implicit
  stages (#3244, #3246, #3247). `'LobattoIIIA4'` (the default), Lobatto IIIA of order 4, enforces the constraints and
  the velocity constraints at the end of the step only, with a multiplier for each stage after the start: the
  simplest form that solves index-3 constraints directly; it has no numerical damping and keeps the energy.
  `'LobattoIIIC4-GGL'`, Lobatto IIIC of order 4, enforces them at each stage, with a multiplier moving the positions
  onto the constraints; it damps fast modes, also where the step is 70 times longer than 1/omega of the mode.
  `'Radau3'`, Radau IIA of order 3, has the constraints as `'LobattoIIIA4'` and damps as well. All are of order 2 in
  the accelerations and multipliers. With `timeIntegration.automaticStepSize` (the default) the step size follows an
  estimate of the local error of the order of the method, with the PI controller of `SolveTrapezoidalGGL`: the
  quadrature of the method against the polynomial through the stage velocities of the previous and this step - for
  the Lobatto methods Simpson's error from the fourth difference of five values -, without an extra solve (#3250);
  the accelerations enter it only without constraints. The local errors are then of the size of the tolerances, and
  the steps grow by the order of the method as the tolerances are lowered.

All of them take the `SimulationSettings` of `mbs.SolveDynamic`: the steps, the solver parameters, the Newton tolerances, the
discontinuous iteration of the data coordinates and, with `timeIntegration.adaptiveStep`, the step size that a
post-Newton step recommends, reduced and grown again as Exudyn's solvers do (see {ref}`sec-solvers`). For a
`SymbolicSystem`, every step is written into its `mbs`: the solution file, the sensors and the renderer show the
solution as after `mbs.SolveDynamic`, and the solution viewer shows the solution file.

The result, the solution file and the sensors are written at the exact times of `solution.file.writePeriod` and
`solution.sensors.writePeriod`, and at the end: a write time within a step takes the state interpolated between the
two ends of the step, the coordinates by the cubic Hermite polynomial of the coordinates and velocities, the
velocities by its derivative, the accelerations, ODE1 and algebraic coordinates linearly, the data coordinates of the
end of the step (#3241). A write period that is a multiple of the step writes the states of the steps as computed.

The main use is a system of which the symbolic system is an exact replicate: it is solved as numpy code - or, once
exported, compiled - while the visualization runs in Exudyn. With the same settings, the solution equals that of
`mbs.SolveDynamic` up to the tolerances of the Newton method; a Newton iteration that does not converge raises instead
of reducing the step. The functions are described in {ref}`sec-module-symbolics.solvers`.
