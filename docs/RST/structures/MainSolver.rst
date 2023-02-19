


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
* | **CheckInitialized(...)** [type = bool, default = mainSystem]:
  | check if MainSolver and MainSystem are correctly initialized ==> otherwise raise SysError
* | **ComputeAlgebraicEquations(...)** [type = \tabnewline void, default = mainSystem, velocityLevel=false]:
  | compute the algebraic equations in systemResidual in range(nODE2+nODE1, nODE2+nODE1+nAE)
* | **ComputeJacobianAE(...)** [type = void, default = mainSystem, scalarFactor\_ODE2=1., scalarFactor\_ODE2\_t=0., scalarFactor\_ODE1=1., velocityLevel=false]:
  | add jacobian of algebraic equations (multiplied with factor) to systemJacobian in cSolver; the scalarFactors are scaling the derivatives w.r.t. ODE2 coordinates, ODE2_t (velocity) coordinates and ODE1 coordinates; if velocityLevel == true, the constraints are evaluated at velocity level; the scalar factors scalarFactor_ODE2=0 and scalarFactor_ODE2 are used for the same ODE2 block in the jacobian
* | **ComputeJacobianODE1RHS(...)** [type = void, default = mainSystem, scalarFactor\_ODE2=1., scalarFactor\_ODE2\_t=0., scalarFactor\_ODE1=1.]:
  | ADD jacobian of ODE1RHS (multiplied with factors for ODE2 and ODE1 coordinates) to the according rows (nODE2:nODE2+nODE1) of the exising systemJacobian in cSolver; it requires a prior call to ComputeJacobianODE2RHS(...); the scalar factors scalarFactor_ODE2=0 and scalarFactor_ODE2 are used for the same ODE2 block in the jacobian
* | **ComputeJacobianODE2RHS(...)** [type = void, default = mainSystem, scalarFactor\_ODE2=1., scalarFactor\_ODE2\_t=0., scalarFactor\_ODE1=1.]:
  | set systemJacobian to zero, size = (nODE2+nODE1+nAE) x (nODE2+nODE1+nAE), and add jacobian (multiplied with factors for ODE2 and ODE1 coordinates) of ODE2RHS to systemJacobian in cSolver; using (scalarFactor_ODE2=-1,scalarFactor_ODE2=0) gives the stiffness matrix (=derivatives of ODE2 coords) in the nODE2 x nODE2 part, while using (scalarFactor_ODE2=0,scalarFactor_ODE2=-1) gives the damping matrix (= derivatives of ODE2 velocity coordinates) in the same part; a superposition of these two parts makes sense for implicit solvers
* | **ComputeLoadFactor(...)** [type = Real, default = simulationSettings]:
  | for static solver, this is a factor in interval [0,1]; MUST be overwritten
* | **ComputeMassMatrix(...)** [type = void, default = mainSystem, scalarFactor=1.]:
  | compute systemMassMatrix (multiplied with factor) in cSolver and return mass nODE2 x nODE2 matrix
* | **ComputeNewtonJacobian(...)** [type = void, default = mainSystem, simulationSettings]:
  | compute jacobian for newton method of given solver method; store result in systemJacobian
* | **ComputeNewtonResidual(...)** [type = Real, default = mainSystem, simulationSettings]:
  | compute residual for Newton method (e.g. static or time step); store residual vector in systemResidual and return scalar residual (specific computation may depend on solver types)
* | **ComputeNewtonUpdate(...)** [type = void, default = mainSystem, simulationSettings, initial=true]:
  | compute update for currentState from newtonSolution (decrement from residual and jacobian); if initial, this is for the initial update with newtonSolution=0
* | **ComputeODE2RHS(...)** [type = void, default = mainSystem]:
  | compute the RHS of ODE2 equations in systemResidual in range(0,nODE2)
* | **DiscontinuousIteration(...)** [type = bool, default = mainSystem, simulationSettings]:
  | perform discontinuousIteration for static step / time step; CALLS ComputeNewtonResidual
* | **FinalizeSolver(...)** [type = void, default = mainSystem, simulationSettings]:
  | write concluding information (timer statistics, messages) and close files
* | **FinishStep(...)** [type = void, default = mainSystem, simulationSettings]:
  | finish static step / time step; write output of results to file
* | **GetAEsize()** [type = Index]:
  | number of algebraic equations in solver
* | **GetDataSize()** [type = Index]:
  | number of data (history) variables in solver
* | **GetNewtonSolution()** [type = NumpyVector]:
  | get locally stored / last computed solution (=increment) of Newton
* | **GetODE1size()** [type = Index]:
  | number of ODE1 equations in solver (not yet implemented)
* | **GetODE2size()** [type = Index]:
  | number of ODE2 equations in solver
* | **GetSimulationEndTime(...)** [type = Real, default = simulationSettings]:
  | compute simulation end time (depends on static or time integration solver)
* | **GetSolverName()** [type = std::string]:
  | get solver name - needed for output file header and visualization window
* | **GetSystemJacobian()** [type = NumpyMatrix]:
  | get locally stored / last computed system jacobian of solver
* | **GetSystemMassMatrix()** [type = NumpyMatrix]:
  | get locally stored / last computed mass matrix of solver
* | **GetSystemResidual()** [type = NumpyVector]:
  | get locally stored / last computed system residual
* | **HasAutomaticStepSizeControl(...)** [type = \tabnewline bool, default = mainSystem, simulationSettings]:
  | return true, if solver supports automatic stepsize control, otherwise false
* | **IncreaseStepSize(...)** [type = void, default = mainSystem, simulationSettings]:
  | increase step size if convergence is good
* | **InitializeSolver(...)** [type = bool, default = mainSystem, simulationSettings]:
  | initialize solverSpecific,data,it,conv; set/compute initial conditions (solver-specific!); initialize output files
* | **InitializeSolverData(...)** [type = void, default = mainSystem, simulationSettings]:
  | initialize all data,it,conv; called from InitializeSolver()
* | **InitializeSolverInitialConditions(...)** [type = \tabnewline void, default = mainSystem, simulationSettings]:
  | set/compute initial conditions (solver-specific!); called from InitializeSolver()
* | **InitializeSolverOutput(...)** [type = void, default = mainSystem, simulationSettings]:
  | initialize output files; called from InitializeSolver()
* | **InitializeSolverPreChecks(...)** [type = \tabnewline bool, default = mainSystem, simulationSettings]:
  | check if system is solvable; initialize dense/sparse computation modes
* | **InitializeStep(...)** [type = void, default = mainSystem, simulationSettings]:
  | initialize static step / time step; Python-functions; do some outputs, checks, etc.
* | **IsStaticSolver()** [type = bool]:
  | return true, if static solver; needs to be overwritten in derived class
* | **IsVerboseCheck(...)** [type = bool, default = level]:
  | return true, if file or console output is at or above the given level
* | **loadStepGeometricFactor** [type = Real]:
  | multiplicative load step factor; this factor is computed from loadStepGeometric parameters in SolveSystem(...)
* | **Newton(...)** [type = bool, default = mainSystem, simulationSettings]:
  | perform Newton method for given solver method
* | **PostInitializeSolverSpecific(...)** [type = \tabnewline void, default = mainSystem, simulationSettings]:
  | post-initialize for solver specific tasks; called at the end of InitializeSolver
* | **PreInitializeSolverSpecific(...)** [type = \tabnewline void, default = mainSystem, simulationSettings]:
  | pre-initialize for solver specific tasks; called at beginning of InitializeSolver, right after Solver data reset
* | **ReduceStepSize(...)** [type = bool, default = mainSystem, simulationSettings, severity]:
  | reduce step size (1..normal, 2..severe problems); return true, if reduction was successful
* | **SetSystemJacobian(...)** [type = void, default = systemJacobian]:
  | set locally stored system jacobian of solver; must have size nODE2+nODE1+nAE
* | **SetSystemMassMatrix(...)** [type = void, default = systemMassMatrix]:
  | set locally stored mass matrix of solver; must have size nODE2+nODE1+nAE
* | **SetSystemResidual(...)** [type = void, default = systemResidual]:
  | set locally stored system residual; must have size nODE2+nODE1+nAE
* | **SolveSteps(...)** [type = bool, default = mainSystem, simulationSettings]:
  | main solver part: calls multiple InitializeStep(...)/ DiscontinuousIteration(...)/ FinishStep(...); do step reduction if necessary; return true if success, false else
* | **SolveSystem(...)** [type = bool, default = mainSystem, simulationSettings]:
  | solve System: InitializeSolver, SolveSteps, FinalizeSolver
* | **UpdateCurrentTime(...)** [type = void, default = mainSystem, simulationSettings]:
  | update currentTime (and load factor); MUST be overwritten in special solver class
* | **VerboseWrite(...)** [type = void, default = level, str]:
  | write to console and/or file in case of level
* | **WriteCoordinatesToFile(...)** [type = void, default = mainSystem, simulationSettings]:
  | write unique coordinates solution file
* | **WriteSolutionFileHeader(...)** [type = void, default = mainSystem, simulationSettings]:
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
* | **CheckInitialized(...)** [type = bool, default = mainSystem]:
  | check if MainSolver and MainSystem are correctly initialized ==> otherwise raise SysError
* | **ComputeAlgebraicEquations(...)** [type = \tabnewline void, default = mainSystem, velocityLevel=false]:
  | compute the algebraic equations in systemResidual in range(nODE2+nODE1, nODE2+nODE1+nAE)
* | **ComputeJacobianAE(...)** [type = void, default = mainSystem, scalarFactor\_ODE2=1., scalarFactor\_ODE2\_t=0., scalarFactor\_ODE1=1., velocityLevel=false]:
  | add jacobian of algebraic equations (multiplied with factor) to systemJacobian in cSolver; the scalarFactors are scaling the derivatives w.r.t. ODE2 coordinates, ODE2_t (velocity) coordinates and ODE1 coordinates; if velocityLevel == true, the constraints are evaluated at velocity level; the scalar factors scalarFactor_ODE2=0 and scalarFactor_ODE2 are used for the same ODE2 block in the jacobian
* | **ComputeJacobianODE1RHS(...)** [type = void, default = mainSystem, scalarFactor\_ODE2=1., scalarFactor\_ODE2\_t=0., scalarFactor\_ODE1=1.]:
  | ADD jacobian of ODE1RHS (multiplied with factors for ODE2 and ODE1 coordinates) to the according rows (nODE2:nODE2+nODE1) of the exising systemJacobian in cSolver; it requires a prior call to ComputeJacobianODE2RHS(...); the scalar factors scalarFactor_ODE2=0 and scalarFactor_ODE2 are used for the same ODE2 block in the jacobian
* | **ComputeJacobianODE2RHS(...)** [type = void, default = mainSystem, scalarFactor\_ODE2=1., scalarFactor\_ODE2\_t=0., scalarFactor\_ODE1=1.]:
  | set systemJacobian to zero, size = (nODE2+nODE1+nAE) x (nODE2+nODE1+nAE), and add jacobian (multiplied with factors for ODE2 and ODE1 coordinates) of ODE2RHS to systemJacobian in cSolver; using (scalarFactor_ODE2=-1,scalarFactor_ODE2=0) gives the stiffness matrix (=derivatives of ODE2 coords) in the nODE2 x nODE2 part, while using (scalarFactor_ODE2=0,scalarFactor_ODE2=-1) gives the damping matrix (= derivatives of ODE2 velocity coordinates) in the same part; a superposition of these two parts makes sense for implicit solvers
* | **ComputeLoadFactor(...)** [type = Real, default = simulationSettings]:
  | for static solver, this is a factor in interval [0,1]; MUST be overwritten
* | **ComputeMassMatrix(...)** [type = void, default = mainSystem, scalarFactor=1.]:
  | compute systemMassMatrix (multiplied with factor) in cSolver and return mass nODE2 x nODE2 matrix
* | **ComputeNewtonJacobian(...)** [type = void, default = mainSystem, simulationSettings]:
  | compute jacobian for newton method of given solver method; store result in systemJacobian
* | **ComputeNewtonResidual(...)** [type = Real, default = mainSystem, simulationSettings]:
  | compute residual for Newton method (e.g. static or time step); store residual vector in systemResidual and return scalar residual (specific computation may depend on solver types)
* | **ComputeNewtonUpdate(...)** [type = void, default = mainSystem, simulationSettings, initial=true]:
  | compute update for currentState from newtonSolution (decrement from residual and jacobian); if initial, this is for the initial update with newtonSolution=0
* | **ComputeODE1RHS(...)** [type = void, default = mainSystem]:
  | compute the RHS of ODE1 equations in systemResidual in range(0,nODE1)
* | **ComputeODE2RHS(...)** [type = void, default = mainSystem]:
  | compute the RHS of ODE2 equations in systemResidual in range(0,nODE2)
* | **DiscontinuousIteration(...)** [type = bool, default = mainSystem, simulationSettings]:
  | perform discontinuousIteration for static step / time step; CALLS ComputeNewtonResidual
* | **factJacAlgorithmic** [type = Real]:
  | locally computed parameter from generalizedAlpha parameters
* | **FinalizeSolver(...)** [type = void, default = mainSystem, simulationSettings]:
  | write concluding information (timer statistics, messages) and close files
* | **FinishStep(...)** [type = void, default = mainSystem, simulationSettings]:
  | finish static step / time step; write output of results to file
* | **GetAAlgorithmic()** [type = NumpyVector]:
  | get locally stored / last computed algorithmic accelerations
* | **GetAEsize()** [type = Index]:
  | number of algebraic equations in solver
* | **GetDataSize()** [type = Index]:
  | number of data (history) variables in solver
* | **GetNewtonSolution()** [type = NumpyVector]:
  | get locally stored / last computed solution (=increment) of Newton
* | **GetODE1size()** [type = Index]:
  | number of ODE1 equations in solver (not yet implemented)
* | **GetODE2size()** [type = Index]:
  | number of ODE2 equations in solver
* | **GetSimulationEndTime(...)** [type = Real, default = simulationSettings]:
  | compute simulation end time (depends on static or time integration solver)
* | **GetSolverName()** [type = std::string]:
  | get solver name - needed for output file header and visualization window
* | **GetStartOfStepStateAAlgorithmic()** [type = \tabnewline NumpyVector]:
  | get locally stored / last computed algorithmic accelerations at start of step
* | **GetSystemJacobian()** [type = NumpyMatrix]:
  | get locally stored / last computed system jacobian of solver
* | **GetSystemMassMatrix()** [type = NumpyMatrix]:
  | get locally stored / last computed mass matrix of solver
* | **GetSystemResidual()** [type = NumpyVector]:
  | get locally stored / last computed system residual
* | **HasAutomaticStepSizeControl(...)** [type = \tabnewline bool, default = mainSystem, simulationSettings]:
  | return true, if solver supports automatic stepsize control, otherwise false
* | **IncreaseStepSize(...)** [type = void, default = mainSystem, simulationSettings]:
  | increase step size if convergence is good
* | **InitializeSolver(...)** [type = bool, default = mainSystem, simulationSettings]:
  | initialize solverSpecific,data,it,conv; set/compute initial conditions (solver-specific!); initialize output files
* | **InitializeSolverData(...)** [type = void, default = mainSystem, simulationSettings]:
  | initialize all data,it,conv; called from InitializeSolver()
* | **InitializeSolverInitialConditions(...)** [type = \tabnewline void, default = mainSystem, simulationSettings]:
  | set/compute initial conditions (solver-specific!); called from InitializeSolver()
* | **InitializeSolverOutput(...)** [type = void, default = mainSystem, simulationSettings]:
  | initialize output files; called from InitializeSolver()
* | **InitializeSolverPreChecks(...)** [type = \tabnewline bool, default = mainSystem, simulationSettings]:
  | check if system is solvable; initialize dense/sparse computation modes
* | **InitializeStep(...)** [type = void, default = mainSystem, simulationSettings]:
  | initialize static step / time step; Python-functions; do some outputs, checks, etc.
* | **IsStaticSolver()** [type = bool]:
  | return true, if static solver; needs to be overwritten in derived class
* | **IsVerboseCheck(...)** [type = bool, default = level]:
  | return true, if file or console output is at or above the given level
* | **newmarkBeta** [type = Real]:
  | copy of parameter in timeIntegration.generalizedAlpha
* | **newmarkGamma** [type = Real]:
  | copy of parameter in timeIntegration.generalizedAlpha
* | **Newton(...)** [type = bool, default = mainSystem, simulationSettings]:
  | perform Newton method for given solver method
* | **PostInitializeSolverSpecific(...)** [type = \tabnewline void, default = mainSystem, simulationSettings]:
  | post-initialize for solver specific tasks; called at the end of InitializeSolver
* | **PostNewton(...)** [type = Real, default = mainSystem, simulationSettings]:
  | call PostNewton for all relevant objects (contact, friction, ... iterations); returns error for discontinuous iteration
* | **PreInitializeSolverSpecific(...)** [type = \tabnewline void, default = mainSystem, simulationSettings]:
  | pre-initialize for solver specific tasks; called at beginning of InitializeSolver, right after Solver data reset
* | **ReduceStepSize(...)** [type = bool, default = mainSystem, simulationSettings, severity]:
  | reduce step size (1..normal, 2..severe problems); return true, if reduction was successful
* | **SetSystemJacobian(...)** [type = void, default = systemJacobian]:
  | set locally stored system jacobian of solver; must have size nODE2+nODE1+nAE
* | **SetSystemMassMatrix(...)** [type = void, default = systemMassMatrix]:
  | set locally stored mass matrix of solver; must have size nODE2+nODE1+nAE
* | **SetSystemResidual(...)** [type = void, default = systemResidual]:
  | set locally stored system residual; must have size nODE2+nODE1+nAE
* | **SetUserFunctionComputeNewtonJacobian(...)** [type = \tabnewline void, default = mainSystem, userFunction]:
  | set user function
* | **SetUserFunctionComputeNewtonResidual(...)** [type = \tabnewline void, default = mainSystem, userFunction]:
  | set user function
* | **SetUserFunctionComputeNewtonUpdate(...)** [type = \tabnewline void, default = mainSystem, userFunction]:
  | set user function
* | **SetUserFunctionDiscontinuousIteration(...)** [type = \tabnewline void, default = mainSystem, userFunction]:
  | set user function
* | **SetUserFunctionFinishStep(...)** [type = \tabnewline void, default = mainSystem, userFunction]:
  | set user function
* | **SetUserFunctionInitializeStep(...)** [type = \tabnewline void, default = mainSystem, userFunction]:
  | set user function
* | **SetUserFunctionNewton(...)** [type = void, default = mainSystem, userFunction]:
  | set user function
* | **SetUserFunctionPostNewton(...)** [type = \tabnewline void, default = mainSystem, userFunction]:
  | set user function
* | **SetUserFunctionUpdateCurrentTime(...)** [type = \tabnewline void, default = mainSystem, userFunction]:
  | set user function
* | **SolveSteps(...)** [type = bool, default = mainSystem, simulationSettings]:
  | main solver part: calls multiple InitializeStep(...)/ DiscontinuousIteration(...)/ FinishStep(...); do step reduction if necessary; return true if success, false else
* | **SolveSystem(...)** [type = bool, default = mainSystem, simulationSettings]:
  | solve System: InitializeSolver, SolveSteps, FinalizeSolver
* | **spectralRadius** [type = Real]:
  | copy of parameter in timeIntegration.generalizedAlpha
* | **UpdateCurrentTime(...)** [type = void, default = mainSystem, simulationSettings]:
  | update currentTime (and load factor); MUST be overwritten in special solver class
* | **VerboseWrite(...)** [type = void, default = level, str]:
  | write to console and/or file in case of level
* | **WriteCoordinatesToFile(...)** [type = void, default = mainSystem, simulationSettings]:
  | write unique coordinates solution file
* | **WriteSolutionFileHeader(...)** [type = void, default = mainSystem, simulationSettings]:
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
* | **ComputeLoadFactor(...)** [type = Real, default = simulationSettings]:
  | for static solver, this is a factor in interval [0,1]; MUST be overwritten
* | **ComputeMassMatrix(...)** [type = void, default = mainSystem, scalarFactor=1.]:
  | compute systemMassMatrix (multiplied with factor) in cSolver and return mass matrix
* | **ComputeNewtonJacobian(...)** [type = void, default = mainSystem, simulationSettings]:
  | compute jacobian for newton method of given solver method; store result in systemJacobian
* | **ComputeNewtonResidual(...)** [type = Real, default = mainSystem, simulationSettings]:
  | compute residual for Newton method (e.g. static or time step); store residual vector in systemResidual and return scalar residual (specific computation may depend on solver types)
* | **ComputeNewtonUpdate(...)** [type = void, default = mainSystem, simulationSettings, initial=true]:
  | compute update for currentState from newtonSolution (decrement from residual and jacobian); if initial, this is for the initial update with newtonSolution=0
* | **ComputeODE1RHS(...)** [type = void, default = mainSystem]:
  | compute the RHS of ODE1 equations in systemResidual in range(0,nODE1)
* | **ComputeODE2RHS(...)** [type = void, default = mainSystem]:
  | compute the RHS of ODE2 equations in systemResidual in range(0,nODE2)
* | **DiscontinuousIteration(...)** [type = bool, default = mainSystem, simulationSettings]:
  | perform discontinuousIteration for static step / time step; CALLS ComputeNewtonResidual
* | **FinalizeSolver(...)** [type = void, default = mainSystem, simulationSettings]:
  | write concluding information (timer statistics, messages) and close files
* | **FinishStep(...)** [type = void, default = mainSystem, simulationSettings]:
  | finish static step / time step; write output of results to file
* | **GetAEsize()** [type = Index]:
  | number of algebraic equations in solver
* | **GetDataSize()** [type = Index]:
  | number of data (history) variables in solver
* | **GetMethodOrder()** [type = Index]:
  | return order of method (higher value in methods with automatic step size, e.g., DOPRI5=5)
* | **GetNumberOfStages()** [type = Index]:
  | return number of stages in current method
* | **GetODE1size()** [type = Index]:
  | number of ODE1 equations in solver (not yet implemented)
* | **GetODE2size()** [type = Index]:
  | number of ODE2 equations in solver
* | **GetSimulationEndTime(...)** [type = Real, default = simulationSettings]:
  | compute simulation end time (depends on static or time integration solver)
* | **GetSolverName()** [type = std::string]:
  | get solver name - needed for output file header and visualization window
* | **GetSystemMassMatrix()** [type = NumpyMatrix]:
  | get locally stored / last computed mass matrix of solver
* | **GetSystemResidual()** [type = NumpyVector]:
  | get locally stored / last computed system residual
* | **HasAutomaticStepSizeControl(...)** [type = \tabnewline bool, default = mainSystem, simulationSettings]:
  | return true, if solver supports automatic stepsize control, otherwise false
* | **IncreaseStepSize(...)** [type = void, default = mainSystem, simulationSettings]:
  | increase step size if convergence is good
* | **InitializeSolver(...)** [type = bool, default = mainSystem, simulationSettings]:
  | initialize solverSpecific,data,it,conv; set/compute initial conditions (solver-specific!); initialize output files
* | **InitializeSolverData(...)** [type = void, default = mainSystem, simulationSettings]:
  | initialize all data,it,conv; called from InitializeSolver()
* | **InitializeSolverInitialConditions(...)** [type = \tabnewline void, default = mainSystem, simulationSettings]:
  | set/compute initial conditions (solver-specific!); called from InitializeSolver()
* | **InitializeSolverOutput(...)** [type = void, default = mainSystem, simulationSettings]:
  | initialize output files; called from InitializeSolver()
* | **InitializeSolverPreChecks(...)** [type = \tabnewline bool, default = mainSystem, simulationSettings]:
  | check if system is solvable; initialize dense/sparse computation modes
* | **InitializeStep(...)** [type = void, default = mainSystem, simulationSettings]:
  | initialize static step / time step; Python-functions; do some outputs, checks, etc.
* | **IsStaticSolver()** [type = bool]:
  | return true, if static solver; needs to be overwritten in derived class
* | **IsVerboseCheck(...)** [type = bool, default = level]:
  | return true, if file or console output is at or above the given level
* | **Newton(...)** [type = bool, default = mainSystem, simulationSettings]:
  | perform Newton method for given solver method
* | **PostInitializeSolverSpecific(...)** [type = \tabnewline void, default = mainSystem, simulationSettings]:
  | post-initialize for solver specific tasks; called at the end of InitializeSolver
* | **PreInitializeSolverSpecific(...)** [type = \tabnewline void, default = mainSystem, simulationSettings]:
  | pre-initialize for solver specific tasks; called at beginning of InitializeSolver, right after Solver data reset
* | **ReduceStepSize(...)** [type = bool, default = mainSystem, simulationSettings, severity]:
  | reduce step size (1..normal, 2..severe problems); return true, if reduction was successful
* | **SetSystemMassMatrix(...)** [type = void, default = systemMassMatrix]:
  | set locally stored mass matrix of solver; must have size nODE2+nODE1+nAE
* | **SetSystemResidual(...)** [type = void, default = systemResidual]:
  | set locally stored system residual; must have size nODE2+nODE1+nAE
* | **SolveSteps(...)** [type = bool, default = mainSystem, simulationSettings]:
  | main solver part: calls multiple InitializeStep(...)/ DiscontinuousIteration(...)/ FinishStep(...); do step reduction if necessary; return true if success, false else
* | **SolveSystem(...)** [type = bool, default = mainSystem, simulationSettings]:
  | solve System: InitializeSolver, SolveSteps, FinalizeSolver
* | **UpdateCurrentTime(...)** [type = void, default = mainSystem, simulationSettings]:
  | update currentTime (and load factor); MUST be overwritten in special solver class
* | **VerboseWrite(...)** [type = void, default = level, str]:
  | write to console and/or file in case of level
* | **WriteCoordinatesToFile(...)** [type = void, default = mainSystem, simulationSettings]:
  | write unique coordinates solution file
* | **WriteSolutionFileHeader(...)** [type = void, default = mainSystem, simulationSettings]:
  | write unique file header, depending on static/ dynamic simulation

