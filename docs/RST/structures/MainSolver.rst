


.. _sec-mainsolverstatic:

MainSolverStatic
----------------

PyBind interface (trampoline) class for static solver. With this interface, the static solver and its substructures can be accessed via Python. NOTE that except from SolveSystem(...), these functions are only intended for experienced users and they need to be handled with care, as unexpected crashes may happen if used inappropriate. Furthermore, the functions have a lot of overhead (performance much lower than internal solver) due to Python interfaces, and should thus be used for small systems. To access the solver in Python, write: 

 
 \ ``solver = MainSolverStatic()``\  

 
and hereafter you can access all data and functions via 'solver'.

MainSolverStatic has the following items:

* | **conv** [type = SolverConvergenceData]:
  | all information about tolerances, errors and residua
* | **it** [type = SolverIterationData]:
  | all information about iterations (steps, discontinuous iteration, newton,...)
* | **newton** [type = NewtonSettings]:
  | copy of newton settings from timeint or staticSolver
* | **output** [type = SolverOutputData]:
  | output modes and timers for exporting solver information and solution
* | **timer** [type = CSolverTimer]:
  | timer which measures the CPU time of solver sub functions
* | **loadStepGeometricFactor** [type = Real]:
  | multiplicative load step factor; this factor is computed from loadStepGeometric parameters in SolveSystem(...)



.. _sec-mainsolverimplicitsecondorder:

MainSolverImplicitSecondOrder
-----------------------------

PyBind interface (trampoline) class for dynamic implicit solver. Note that this solver includes the classical Newmark method (set useNewmark True; with option of index 2 reduction) as well as the generalized-alpha method. With the interface, the dynamic implicit solver and its substructures can be accessed via Python. NOTE that except from SolveSystem(...), these functions are only intended for experienced users and they need to be handled with care, as unexpected crashes may happen if used inappropriate. Furthermore, the functions have a lot of overhead (still fast, but performance much lower than internal solver) due to Python interfaces, and should thus be used for small systems. To access the solver in Python, write: 

 
 \ ``solver = MainSolverImplicitSecondOrder()``\  

 
and hereafter you can access all data and functions via 'solver'.
 In this solver, user functions are possible to extend the solver at certain parts, while keeping the overal C++ performance. User functions, which are added with SetUserFunction...(...), have the arguments (MainSolver, MainSystem, simulationSettings), except for ComputeNewtonUpdate which adds the initial flag as an additional argument and ComputeNewtonResidual, which returns the scalar residual.

MainSolverImplicitSecondOrder has the following items:

* | **conv** [type = SolverConvergenceData]:
  | all information about tolerances, errors and residua
* | **it** [type = SolverIterationData]:
  | all information about iterations (steps, discontinuous iteration, newton,...)
* | **newton** [type = NewtonSettings]:
  | copy of newton settings from timeint or staticSolver
* | **output** [type = SolverOutputData]:
  | output modes and timers for exporting solver information and solution
* | **timer** [type = CSolverTimer]:
  | timer which measures the CPU time of solver sub functions; note that solver structures can only be written indirectly, e.g.,  timer=dynamicSolver.timer; timer.useTimer = False; dynamicSolver.timer=timer; however, dynamicSolver.timer.useTimer cannot be written.
* | **alphaF** [type = Real]:
  | copy of parameter in timeIntegration.generalizedAlpha
* | **alphaM** [type = Real]:
  | copy of parameter in timeIntegration.generalizedAlpha
* | **factJacAlgorithmic** [type = Real]:
  | locally computed parameter from generalizedAlpha parameters
* | **newmarkBeta** [type = Real]:
  | copy of parameter in timeIntegration.generalizedAlpha
* | **newmarkGamma** [type = Real]:
  | copy of parameter in timeIntegration.generalizedAlpha
* | **spectralRadius** [type = Real]:
  | copy of parameter in timeIntegration.generalizedAlpha



.. _sec-mainsolverexplicit:

MainSolverExplicit
------------------

PyBind interface (trampoline) class for dynamic explicit solver. Note that this solver includes the 1st order explicit Euler scheme and the 4th order Runge-Kutta scheme with 5th order error estimation (DOPRI5). With the interface, the solver and its substructures can be accessed via Python. NOTE that except from SolveSystem(...), these functions are only intended for experienced users and they need to be handled with care, as unexpected crashes may happen if used inappropriate. Furthermore, the functions have a lot of overhead (still fast, but performance much lower than internal solver) due to Python interfaces, and should thus be used for small systems. To access the solver in Python, write 

 
 \ ``solver = MainSolverExplicit()``\  

 
and hereafter you can access all data and functions via 'solver'.
 In this solver, no user functions are possible, but you can use SolverImplicitSecondOrder instead (turning off Newton gives explicit scheme ...).

MainSolverExplicit has the following items:

* | **conv** [type = SolverConvergenceData]:
  | all information about tolerances, errors and residua
* | **it** [type = SolverIterationData]:
  | all information about iterations (steps, discontinuous iteration, newton,...)
* | **output** [type = SolverOutputData]:
  | output modes and timers for exporting solver information and solution
* | **timer** [type = CSolverTimer]:
  | timer which measures the CPU time of solver sub functions

