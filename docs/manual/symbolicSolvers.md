(sec-symbolicsolvers)=
# Symbolic solvers

`exudyn.symbolics.solvers` integrates a symbolic system (a `SymbolicSystem` or a `NumpySystem`) with the algorithms of
Exudyn's C++ solvers, written in Python with numpy and scipy:

- `SolveExplicit(system, simulationSettings, solverType)`: explicit Euler, RK44 and velocity Verlet, the accelerations
  by the inverse mass matrix of each body or by the decomposition of the mass matrix, without algebraic equations;
- `SolveGeneralizedAlpha(system, simulationSettings)`: the generalized-alpha method and the Newmark method, with ODE1
  coordinates and index 3 constraints, the Newton method with the exact Jacobian of the system.

Both take the `SimulationSettings` of `mbs.SolveDynamic`: the steps, the solver parameters, the Newton tolerances, the
discontinuous iteration of the data coordinates and, with `timeIntegration.adaptiveStep`, the step size that a
post-Newton step recommends, reduced and grown again as Exudyn's solvers do (see {ref}`sec-solvers`). For a
`SymbolicSystem`, every step is written into its `mbs`: the solution file, the sensors and the renderer show the
solution as after `mbs.SolveDynamic`, and the solution viewer shows the solution file.

The main use is a system of which the symbolic system is an exact replicate: it is solved as numpy code - or, once
exported, compiled - while the visualization runs in Exudyn. With the same settings, the solution equals that of
`mbs.SolveDynamic` up to the tolerances of the Newton method; a Newton iteration that does not converge raises instead
of reducing the step. The functions are described in {ref}`sec-module-symbolics.solvers`.
