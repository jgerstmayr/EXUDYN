(sec-symbolicsolvers)=
# Symbolic solvers

`exudyn.symbolics.solvers` integrates a symbolic system (a `SymbolicSystem` or a `NumpySystem`) with the algorithms of
Exudyn's C++ solvers, written in Python with numpy and scipy:

- `SolveExplicit(system, simulationSettings, solverType)`: explicit Euler, RK44 and velocity Verlet, the accelerations
  by the inverse mass matrix of each body or by the decomposition of the mass matrix; algebraic equations that fix
  one coordinate each - a coordinate constraint to the ground, as `UserItemCreator.CreateCoordinateConstraint` makes
  it - are eliminated with `timeIntegration.explicit.eliminateConstraints`, as Exudyn's explicit solver eliminates
  CoordinateConstraints to the ground; the coordinate keeps the value of its equation, also in the stages, and any
  other algebraic equation is refused (#3238);
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
  explicit solvers DOPRI5 and ODE23, and a PI controller, which rejects few steps (#3243). The tolerances are those of
  one step: the error of a motion that is sensitive to small changes - a chain falling from an unstable position -
  grows beyond them, while a motion of changing time scales - a ball bouncing on the ground - needs a tenth of the
  steps of a constant step size for the same error. The algebraic equations must depend on the coordinates and the
  time only (#3242).
- `SolveLobatto4(system, simulationSettings, variant)`: the 3-stage Lobatto methods of order 4 with constant steps
  (#3244), with the constraints, the velocity constraints and a multiplier moving the positions onto the constraints
  at each implicit stage: `'IIIC-GGL'` (the default), Lobatto IIIC, which damps fast modes, and `'IIIA-GGL'`,
  Lobatto IIIA, which has one stage less, keeps the energy and has smaller errors of the accelerations, but does not
  converge where a step is much larger than a fast period; both are of order 4 in the coordinates and velocities and
  of order 2 in the accelerations and multipliers. `'IIIC'`, with the constraints at all stages only, is of order 2 in
  the velocities and does not keep the velocity constraints.

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
