


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
* | **CheckInitialized(mainSystem)** [return type = bool]:
  | check if MainSolver and MainSystem are correctly initialized ==> otherwise raise SysError
* | **ComputeAlgebraicEquations(mainSystem, velocityLevel=False)** [return type = void]:
  | compute the algebraic equations in systemResidual in range(nODE2+nODE1, nODE2+nODE1+nAE)
* | **ComputeJacobianAE(mainSystem, scalarFactor\_ODE2=1., scalarFactor\_ODE2\_t=0., scalarFactor\_ODE1=1., velocityLevel=False)** [return type = void]:
  | add jacobian of algebraic equations (multiplied with factor) to systemJacobian in cSolver; the scalarFactors are scaling the derivatives w.r.t. \ :ref:`ODE2 <ODE2>`\  coordinates, ODE2_t (velocity) coordinates and ODE1 coordinates; if velocityLevel == true, the constraints are evaluated at velocity level; the scalar factors scalarFactor_ODE2=0 and scalarFactor_ODE2 are used for the same ODE2 block in the jacobian
* | **ComputeJacobianODE1RHS(mainSystem, scalarFactor\_ODE2=1., scalarFactor\_ODE2\_t=0., scalarFactor\_ODE1=1.)** [return type = void]:
  | ADD jacobian of ODE1RHS (multiplied with factors for ODE2 and ODE1 coordinates) to the according rows (nODE2:nODE2+nODE1) of the exising systemJacobian in cSolver; it requires a prior call to ComputeJacobianODE2RHS(...); the scalar factors scalarFactor_ODE2=0 and scalarFactor_ODE2 are used for the same ODE2 block in the jacobian
* | **ComputeJacobianODE2RHS(mainSystem, scalarFactor\_ODE2=1., scalarFactor\_ODE2\_t=0., scalarFactor\_ODE1=1., computeLoadsJacobian=0)** [return type = void]:
  | set systemJacobian to zero, size = (nODE2+nODE1+nAE) x (nODE2+nODE1+nAE), and add jacobian (multiplied with factors for ODE2 and ODE1 coordinates) of ODE2RHS to systemJacobian in cSolver; using (scalarFactor_ODE2=-1,scalarFactor_ODE2=0) gives the stiffness matrix (=derivatives of ODE2 coords) in the nODE2 x nODE2 part, while using (scalarFactor_ODE2=0,scalarFactor_ODE2=-1) gives the damping matrix (= derivatives of ODE2 velocity coordinates) in the same part; a superposition of these two parts makes sense for implicit solvers; if , Index computeLoadsJacobian=0, loads are not considered in the Jacobian computation; for , Index computeLoadsJacobian=1 the ODE2 and ODE1 derivatives of loads are included and for , Index computeLoadsJacobian=2, also the ODE2_t dependencies are added
* | **ComputeLoadFactor(simulationSettings)** [return type = Real]:
  | for static solver, this is a factor in interval [0,1]; MUST be overwritten
* | **ComputeMassMatrix(mainSystem, scalarFactor=1.)** [return type = void]:
  | compute systemMassMatrix (multiplied with factor) in cSolver and return mass nODE2 x nODE2 matrix
* | **ComputeNewtonJacobian(mainSystem, simulationSettings)** [return type = void]:
  | compute jacobian for newton method of given solver method; store result in systemJacobian
* | **ComputeNewtonResidual(mainSystem, simulationSettings)** [return type = Real]:
  | compute residual for Newton method (e.g. static or time step); store residual vector in systemResidual and return scalar residual (specific computation may depend on solver types)
* | **ComputeNewtonUpdate(mainSystem, simulationSettings, initial=True)** [return type = void]:
  | compute update for currentState from newtonSolution (decrement from residual and jacobian); if initial, this is for the initial update with newtonSolution=0
* | **ComputeODE2RHS(mainSystem)** [return type = void]:
  | compute the RHS of \ :ref:`ODE2 <ODE2>`\  equations in systemResidual in range(0,nODE2)
* | **DiscontinuousIteration(mainSystem, simulationSettings)** [return type = bool]:
  | perform discontinuousIteration for static step / time step; CALLS ComputeNewtonResidual
* | **FinalizeSolver(mainSystem, simulationSettings)** [return type = void]:
  | write concluding information (timer statistics, messages) and close files
* | **FinishStep(mainSystem, simulationSettings)** [return type = void]:
  | finish static step / time step; write output of results to file
* | **GetAEsize()** [return type = Index]:
  | number of algebraic equations in solver
* | **GetDataSize()** [return type = Index]:
  | number of data (history) variables in solver
* | **GetErrorString()** [return type = std::string]:
  | return error string if solver has not been successful
* | **GetNewtonSolution()** [return type = NumpyVector]:
  | get locally stored / last computed solution (=increment) of Newton
* | **GetODE1size()** [return type = Index]:
  | number of \ :ref:`ODE1 <ODE1>`\  equations in solver (not yet implemented)
* | **GetODE2size()** [return type = Index]:
  | number of \ :ref:`ODE2 <ODE2>`\  equations in solver
* | **GetSimulationEndTime(simulationSettings)** [return type = Real]:
  | compute simulation end time (depends on static or time integration solver)
* | **GetSolverName()** [return type = std::string]:
  | get solver name - needed for output file header and visualization window
* | **GetSystemJacobian()** [return type = NumpyMatrix]:
  | get locally stored / last computed system jacobian of solver
* | **GetSystemMassMatrix()** [return type = NumpyMatrix]:
  | get locally stored / last computed mass matrix of solver
* | **GetSystemResidual()** [return type = NumpyVector]:
  | get locally stored / last computed system residual
* | **HasAutomaticStepSizeControl(mainSystem, simulationSettings)** [return type = bool]:
  | return true, if solver supports automatic stepsize control, otherwise false
* | **IncreaseStepSize(mainSystem, simulationSettings)** [return type = void]:
  | increase step size if convergence is good
* | **InitializeSolver(mainSystem, simulationSettings)** [return type = bool]:
  | initialize solverSpecific,data,it,conv; set/compute initial conditions (solver-specific!); initialize output files
* | **InitializeSolverData(mainSystem, simulationSettings)** [return type = void]:
  | initialize all data,it,conv; called from InitializeSolver()
* | **InitializeSolverInitialConditions(mainSystem, simulationSettings)** [return type = void]:
  | set/compute initial conditions (solver-specific!); called from InitializeSolver()
* | **InitializeSolverOutput(mainSystem, simulationSettings)** [return type = void]:
  | initialize output files; called from InitializeSolver()
* | **InitializeSolverPreChecks(mainSystem, simulationSettings)** [return type = bool]:
  | check if system is solvable; initialize dense/sparse computation modes
* | **InitializeStep(mainSystem, simulationSettings)** [return type = void]:
  | initialize static step / time step; Python-functions; do some outputs, checks, etc.
* | **IsStaticSolver()** [return type = bool]:
  | return true, if static solver; needs to be overwritten in derived class
* | **IsVerboseCheck(level)** [return type = bool]:
  | return true, if file or console output is at or above the given level
* | **loadStepGeometricFactor** [type = Real]:
  | multiplicative load step factor; this factor is computed from loadStepGeometric parameters in SolveSystem(...)
* | **Newton(mainSystem, simulationSettings)** [return type = bool]:
  | perform Newton method for given solver method
* | **PostInitializeSolverSpecific(mainSystem, simulationSettings)** [return type = void]:
  | post-initialize for solver specific tasks; called at the end of InitializeSolver
* | **PreInitializeSolverSpecific(mainSystem, simulationSettings)** [return type = void]:
  | pre-initialize for solver specific tasks; called at beginning of InitializeSolver, right after Solver data reset
* | **ReduceStepSize(mainSystem, simulationSettings, severity)** [return type = bool]:
  | reduce step size (1..normal, 2..severe problems); return true, if reduction was successful
* | **SetSystemJacobian(systemJacobian)** [return type = void]:
  | set locally stored system jacobian of solver; must have size nODE2+nODE1+nAE
* | **SetSystemMassMatrix(systemMassMatrix)** [return type = void]:
  | set locally stored mass matrix of solver; must have size nODE2+nODE1+nAE
* | **SetSystemResidual(systemResidual)** [return type = void]:
  | set locally stored system residual; must have size nODE2+nODE1+nAE
* | **SolveSteps(mainSystem, simulationSettings)** [return type = bool]:
  | main solver part: calls multiple InitializeStep(...)/ DiscontinuousIteration(...)/ FinishStep(...); do step reduction if necessary; return true if success, false else
* | **SolveSystem(mainSystem, simulationSettings)** [return type = bool]:
  | solve System: InitializeSolver, SolveSteps, FinalizeSolver
* | **UpdateCurrentTime(mainSystem, simulationSettings)** [return type = void]:
  | update currentTime (and load factor); MUST be overwritten in special solver class
* | **VerboseWrite(level, str)** [return type = void]:
  | write to console and/or file in case of level
* | **WriteCoordinatesToFile(mainSystem, simulationSettings)** [return type = void]:
  | write unique coordinates solution file
* | **WriteSolutionFileHeader(mainSystem, simulationSettings)** [return type = void]:
  | write unique file header, depending on static/ dynamic simulation



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
* | **CheckInitialized(mainSystem)** [return type = bool]:
  | check if MainSolver and MainSystem are correctly initialized ==> otherwise raise SysError
* | **ComputeAlgebraicEquations(mainSystem, velocityLevel=False)** [return type = void]:
  | compute the algebraic equations in systemResidual in range(nODE2+nODE1, nODE2+nODE1+nAE)
* | **ComputeJacobianAE(mainSystem, scalarFactor\_ODE2=1., scalarFactor\_ODE2\_t=0., scalarFactor\_ODE1=1., velocityLevel=False)** [return type = void]:
  | add jacobian of algebraic equations (multiplied with factor) to systemJacobian in cSolver; the scalarFactors are scaling the derivatives w.r.t. \ :ref:`ODE2 <ODE2>`\  coordinates, ODE2_t (velocity) coordinates and ODE1 coordinates; if velocityLevel == true, the constraints are evaluated at velocity level; the scalar factors scalarFactor_ODE2=0 and scalarFactor_ODE2 are used for the same ODE2 block in the jacobian
* | **ComputeJacobianODE1RHS(mainSystem, scalarFactor\_ODE2=1., scalarFactor\_ODE2\_t=0., scalarFactor\_ODE1=1.)** [return type = void]:
  | ADD jacobian of ODE1RHS (multiplied with factors for ODE2 and ODE1 coordinates) to the according rows (nODE2:nODE2+nODE1) of the exising systemJacobian in cSolver; it requires a prior call to ComputeJacobianODE2RHS(...); the scalar factors scalarFactor_ODE2=0 and scalarFactor_ODE2 are used for the same ODE2 block in the jacobian
* | **ComputeJacobianODE2RHS(mainSystem, scalarFactor\_ODE2=1., scalarFactor\_ODE2\_t=0., scalarFactor\_ODE1=1., computeLoadsJacobian=0)** [return type = void]:
  | set systemJacobian to zero, size = (nODE2+nODE1+nAE) x (nODE2+nODE1+nAE), and add jacobian (multiplied with factors for ODE2 and ODE1 coordinates) of ODE2RHS to systemJacobian in cSolver; using (scalarFactor_ODE2=-1,scalarFactor_ODE2=0) gives the stiffness matrix (=derivatives of ODE2 coords) in the nODE2 x nODE2 part, while using (scalarFactor_ODE2=0,scalarFactor_ODE2=-1) gives the damping matrix (= derivatives of ODE2 velocity coordinates) in the same part; a superposition of these two parts makes sense for implicit solvers; if , Index computeLoadsJacobian=0, loads are not considered in the Jacobian computation; for , Index computeLoadsJacobian=1 the ODE2 and ODE1 derivatives of loads are included and for , Index computeLoadsJacobian=2, also the ODE2_t dependencies are added
* | **ComputeLoadFactor(simulationSettings)** [return type = Real]:
  | for static solver, this is a factor in interval [0,1]; MUST be overwritten
* | **ComputeMassMatrix(mainSystem, scalarFactor=1.)** [return type = void]:
  | compute systemMassMatrix (multiplied with factor) in cSolver and return mass nODE2 x nODE2 matrix
* | **ComputeNewtonJacobian(mainSystem, simulationSettings)** [return type = void]:
  | compute jacobian for newton method of given solver method; store result in systemJacobian
* | **ComputeNewtonResidual(mainSystem, simulationSettings)** [return type = Real]:
  | compute residual for Newton method (e.g. static or time step); store residual vector in systemResidual and return scalar residual (specific computation may depend on solver types)
* | **ComputeNewtonUpdate(mainSystem, simulationSettings, initial=True)** [return type = void]:
  | compute update for currentState from newtonSolution (decrement from residual and jacobian); if initial, this is for the initial update with newtonSolution=0
* | **ComputeODE1RHS(mainSystem)** [return type = void]:
  | compute the RHS of \ :ref:`ODE1 <ODE1>`\  equations in systemResidual in range(0,nODE1)
* | **ComputeODE2RHS(mainSystem)** [return type = void]:
  | compute the RHS of \ :ref:`ODE2 <ODE2>`\  equations in systemResidual in range(0,nODE2)
* | **DiscontinuousIteration(mainSystem, simulationSettings)** [return type = bool]:
  | perform discontinuousIteration for static step / time step; CALLS ComputeNewtonResidual
* | **factJacAlgorithmic** [type = Real]:
  | locally computed parameter from generalizedAlpha parameters
* | **FinalizeSolver(mainSystem, simulationSettings)** [return type = void]:
  | write concluding information (timer statistics, messages) and close files
* | **FinishStep(mainSystem, simulationSettings)** [return type = void]:
  | finish static step / time step; write output of results to file
* | **GetAAlgorithmic()** [return type = NumpyVector]:
  | get locally stored / last computed algorithmic accelerations
* | **GetAEsize()** [return type = Index]:
  | number of algebraic equations in solver
* | **GetDataSize()** [return type = Index]:
  | number of data (history) variables in solver
* | **GetErrorString()** [return type = std::string]:
  | return error string if solver has not been successful
* | **GetNewtonSolution()** [return type = NumpyVector]:
  | get locally stored / last computed solution (=increment) of Newton
* | **GetODE1size()** [return type = Index]:
  | number of \ :ref:`ODE1 <ODE1>`\  equations in solver (not yet implemented)
* | **GetODE2size()** [return type = Index]:
  | number of \ :ref:`ODE2 <ODE2>`\  equations in solver
* | **GetSimulationEndTime(simulationSettings)** [return type = Real]:
  | compute simulation end time (depends on static or time integration solver)
* | **GetSolverName()** [return type = std::string]:
  | get solver name - needed for output file header and visualization window
* | **GetStartOfStepStateAAlgorithmic()** [return type = NumpyVector]:
  | get locally stored / last computed algorithmic accelerations at start of step
* | **GetSystemJacobian()** [return type = NumpyMatrix]:
  | get locally stored / last computed system jacobian of solver
* | **GetSystemMassMatrix()** [return type = NumpyMatrix]:
  | get locally stored / last computed mass matrix of solver
* | **GetSystemResidual()** [return type = NumpyVector]:
  | get locally stored / last computed system residual
* | **HasAutomaticStepSizeControl(mainSystem, simulationSettings)** [return type = bool]:
  | return true, if solver supports automatic stepsize control, otherwise false
* | **IncreaseStepSize(mainSystem, simulationSettings)** [return type = void]:
  | increase step size if convergence is good
* | **InitializeSolver(mainSystem, simulationSettings)** [return type = bool]:
  | initialize solverSpecific,data,it,conv; set/compute initial conditions (solver-specific!); initialize output files
* | **InitializeSolverData(mainSystem, simulationSettings)** [return type = void]:
  | initialize all data,it,conv; called from InitializeSolver()
* | **InitializeSolverInitialConditions(mainSystem, simulationSettings)** [return type = void]:
  | set/compute initial conditions (solver-specific!); called from InitializeSolver()
* | **InitializeSolverOutput(mainSystem, simulationSettings)** [return type = void]:
  | initialize output files; called from InitializeSolver()
* | **InitializeSolverPreChecks(mainSystem, simulationSettings)** [return type = bool]:
  | check if system is solvable; initialize dense/sparse computation modes
* | **InitializeStep(mainSystem, simulationSettings)** [return type = void]:
  | initialize static step / time step; Python-functions; do some outputs, checks, etc.
* | **IsStaticSolver()** [return type = bool]:
  | return true, if static solver; needs to be overwritten in derived class
* | **IsVerboseCheck(level)** [return type = bool]:
  | return true, if file or console output is at or above the given level
* | **newmarkBeta** [type = Real]:
  | copy of parameter in timeIntegration.generalizedAlpha
* | **newmarkGamma** [type = Real]:
  | copy of parameter in timeIntegration.generalizedAlpha
* | **Newton(mainSystem, simulationSettings)** [return type = bool]:
  | perform Newton method for given solver method
* | **PostInitializeSolverSpecific(mainSystem, simulationSettings)** [return type = void]:
  | post-initialize for solver specific tasks; called at the end of InitializeSolver
* | **PostNewton(mainSystem, simulationSettings)** [return type = Real]:
  | call PostNewton for all relevant objects (contact, friction, ... iterations); returns error for discontinuous iteration
* | **PreInitializeSolverSpecific(mainSystem, simulationSettings)** [return type = void]:
  | pre-initialize for solver specific tasks; called at beginning of InitializeSolver, right after Solver data reset
* | **ReduceStepSize(mainSystem, simulationSettings, severity)** [return type = bool]:
  | reduce step size (1..normal, 2..severe problems); return true, if reduction was successful
* | **SetSystemJacobian(systemJacobian)** [return type = void]:
  | set locally stored system jacobian of solver; must have size nODE2+nODE1+nAE
* | **SetSystemMassMatrix(systemMassMatrix)** [return type = void]:
  | set locally stored mass matrix of solver; must have size nODE2+nODE1+nAE
* | **SetSystemResidual(systemResidual)** [return type = void]:
  | set locally stored system residual; must have size nODE2+nODE1+nAE
* | **SetUserFunctionComputeNewtonJacobian(mainSystem, userFunction)** [return type = void]:
  | set user function
* | **SetUserFunctionComputeNewtonResidual(mainSystem, userFunction)** [return type = void]:
  | set user function
* | **SetUserFunctionComputeNewtonUpdate(mainSystem, userFunction)** [return type = void]:
  | set user function
* | **SetUserFunctionDiscontinuousIteration(mainSystem, userFunction)** [return type = void]:
  | set user function
* | **SetUserFunctionFinishStep(mainSystem, userFunction)** [return type = void]:
  | set user function
* | **SetUserFunctionInitializeStep(mainSystem, userFunction)** [return type = void]:
  | set user function
* | **SetUserFunctionNewton(mainSystem, userFunction)** [return type = void]:
  | set user function
* | **SetUserFunctionPostNewton(mainSystem, userFunction)** [return type = void]:
  | set user function
* | **SetUserFunctionUpdateCurrentTime(mainSystem, userFunction)** [return type = void]:
  | set user function
* | **SolveSteps(mainSystem, simulationSettings)** [return type = bool]:
  | main solver part: calls multiple InitializeStep(...)/ DiscontinuousIteration(...)/ FinishStep(...); do step reduction if necessary; return true if success, false else
* | **SolveSystem(mainSystem, simulationSettings)** [return type = bool]:
  | solve System: InitializeSolver, SolveSteps, FinalizeSolver
* | **spectralRadius** [type = Real]:
  | copy of parameter in timeIntegration.generalizedAlpha
* | **UpdateCurrentTime(mainSystem, simulationSettings)** [return type = void]:
  | update currentTime (and load factor); MUST be overwritten in special solver class
* | **VerboseWrite(level, str)** [return type = void]:
  | write to console and/or file in case of level
* | **WriteCoordinatesToFile(mainSystem, simulationSettings)** [return type = void]:
  | write unique coordinates solution file
* | **WriteSolutionFileHeader(mainSystem, simulationSettings)** [return type = void]:
  | write unique file header, depending on static/ dynamic simulation



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
* | **ComputeLoadFactor(simulationSettings)** [return type = Real]:
  | for static solver, this is a factor in interval [0,1]; MUST be overwritten
* | **ComputeMassMatrix(mainSystem, scalarFactor=1.)** [return type = void]:
  | compute systemMassMatrix (multiplied with factor) in cSolver and return mass matrix
* | **ComputeNewtonJacobian(mainSystem, simulationSettings)** [return type = void]:
  | compute jacobian for newton method of given solver method; store result in systemJacobian
* | **ComputeNewtonResidual(mainSystem, simulationSettings)** [return type = Real]:
  | compute residual for Newton method (e.g. static or time step); store residual vector in systemResidual and return scalar residual (specific computation may depend on solver types)
* | **ComputeNewtonUpdate(mainSystem, simulationSettings, initial=True)** [return type = void]:
  | compute update for currentState from newtonSolution (decrement from residual and jacobian); if initial, this is for the initial update with newtonSolution=0
* | **ComputeODE1RHS(mainSystem)** [return type = void]:
  | compute the RHS of \ :ref:`ODE1 <ODE1>`\  equations in systemResidual in range(0,nODE1)
* | **ComputeODE2RHS(mainSystem)** [return type = void]:
  | compute the RHS of \ :ref:`ODE2 <ODE2>`\  equations in systemResidual in range(0,nODE2)
* | **DiscontinuousIteration(mainSystem, simulationSettings)** [return type = bool]:
  | perform discontinuousIteration for static step / time step; CALLS ComputeNewtonResidual
* | **FinalizeSolver(mainSystem, simulationSettings)** [return type = void]:
  | write concluding information (timer statistics, messages) and close files
* | **FinishStep(mainSystem, simulationSettings)** [return type = void]:
  | finish static step / time step; write output of results to file
* | **GetAEsize()** [return type = Index]:
  | number of algebraic equations in solver
* | **GetDataSize()** [return type = Index]:
  | number of data (history) variables in solver
* | **GetErrorString()** [return type = std::string]:
  | return error string if solver has not been successful
* | **GetMethodOrder()** [return type = Index]:
  | return order of method (higher value in methods with automatic step size, e.g., DOPRI5=5)
* | **GetNumberOfStages()** [return type = Index]:
  | return number of stages in current method
* | **GetODE1size()** [return type = Index]:
  | number of \ :ref:`ODE1 <ODE1>`\  equations in solver (not yet implemented)
* | **GetODE2size()** [return type = Index]:
  | number of \ :ref:`ODE2 <ODE2>`\  equations in solver
* | **GetSimulationEndTime(simulationSettings)** [return type = Real]:
  | compute simulation end time (depends on static or time integration solver)
* | **GetSolverName()** [return type = std::string]:
  | get solver name - needed for output file header and visualization window
* | **GetSystemMassMatrix()** [return type = NumpyMatrix]:
  | get locally stored / last computed mass matrix of solver
* | **GetSystemResidual()** [return type = NumpyVector]:
  | get locally stored / last computed system residual
* | **HasAutomaticStepSizeControl(mainSystem, simulationSettings)** [return type = bool]:
  | return true, if solver supports automatic stepsize control, otherwise false
* | **IncreaseStepSize(mainSystem, simulationSettings)** [return type = void]:
  | increase step size if convergence is good
* | **InitializeSolver(mainSystem, simulationSettings)** [return type = bool]:
  | initialize solverSpecific,data,it,conv; set/compute initial conditions (solver-specific!); initialize output files
* | **InitializeSolverData(mainSystem, simulationSettings)** [return type = void]:
  | initialize all data,it,conv; called from InitializeSolver()
* | **InitializeSolverInitialConditions(mainSystem, simulationSettings)** [return type = void]:
  | set/compute initial conditions (solver-specific!); called from InitializeSolver()
* | **InitializeSolverOutput(mainSystem, simulationSettings)** [return type = void]:
  | initialize output files; called from InitializeSolver()
* | **InitializeSolverPreChecks(mainSystem, simulationSettings)** [return type = bool]:
  | check if system is solvable; initialize dense/sparse computation modes
* | **InitializeStep(mainSystem, simulationSettings)** [return type = void]:
  | initialize static step / time step; Python-functions; do some outputs, checks, etc.
* | **IsStaticSolver()** [return type = bool]:
  | return true, if static solver; needs to be overwritten in derived class
* | **IsVerboseCheck(level)** [return type = bool]:
  | return true, if file or console output is at or above the given level
* | **Newton(mainSystem, simulationSettings)** [return type = bool]:
  | perform Newton method for given solver method
* | **PostInitializeSolverSpecific(mainSystem, simulationSettings)** [return type = void]:
  | post-initialize for solver specific tasks; called at the end of InitializeSolver
* | **PreInitializeSolverSpecific(mainSystem, simulationSettings)** [return type = void]:
  | pre-initialize for solver specific tasks; called at beginning of InitializeSolver, right after Solver data reset
* | **ReduceStepSize(mainSystem, simulationSettings, severity)** [return type = bool]:
  | reduce step size (1..normal, 2..severe problems); return true, if reduction was successful
* | **SetSystemMassMatrix(systemMassMatrix)** [return type = void]:
  | set locally stored mass matrix of solver; must have size nODE2+nODE1+nAE
* | **SetSystemResidual(systemResidual)** [return type = void]:
  | set locally stored system residual; must have size nODE2+nODE1+nAE
* | **SolveSteps(mainSystem, simulationSettings)** [return type = bool]:
  | main solver part: calls multiple InitializeStep(...)/ DiscontinuousIteration(...)/ FinishStep(...); do step reduction if necessary; return true if success, false else
* | **SolveSystem(mainSystem, simulationSettings)** [return type = bool]:
  | solve System: InitializeSolver, SolveSteps, FinalizeSolver
* | **UpdateCurrentTime(mainSystem, simulationSettings)** [return type = void]:
  | update currentTime (and load factor); MUST be overwritten in special solver class
* | **VerboseWrite(level, str)** [return type = void]:
  | write to console and/or file in case of level
* | **WriteCoordinatesToFile(mainSystem, simulationSettings)** [return type = void]:
  | write unique coordinates solution file
* | **WriteSolutionFileHeader(mainSystem, simulationSettings)** [return type = void]:
  | write unique file header, depending on static/ dynamic simulation

