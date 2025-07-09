


.. _sec-solversubstructures:


********************
Solver substructures
********************

This section includes structures contained in the solver, which can be accessed via the Python interface during solution or for building a customized solver in Python.
There is plenty of possibilities to interact with the solvers, being it the extraction of data at the end (such as .it or .conv), computation of mass matrix or system matrices, and finally the modification of solver structures (which may have effect or not). In any case, there is no full description for all these methods and the user must always consider the according C++ function to verify the desired behavior.


.. _sec-csolvertimer:

CSolverTimer
------------

Structure for timing in solver. Each Real variable is used to measure the CPU time which certain parts of the solver need. This structure is only active if the code is not compiled with the __FAST_EXUDYN_LINALG option and if displayComputationTime is set True. Timings will only be filled, if useTimer is True.

CSolverTimer has the following items:

* | **AERHS** [type = Real, default = 0.]:
  | time for residual evaluation of algebraic equations right-hand-side
* | **errorEstimator** [type = Real, default = 0.]:
  | for explicit solvers, additional evaluation
* | **factorization** [type = Real, default = 0.]:
  | solve or inverse
* | **integrationFormula** [type = Real, default = 0.]:
  | time spent for evaluation of integration formulas
* | **jacobianAE** [type = Real, default = 0.]:
  | jacobian of algebraic equations (not counted in sum)
* | **jacobianODE1** [type = Real, default = 0.]:
  | jacobian w.r.t. coordinates of \ :ref:`ODE1 <ODE1>`\  equations (not counted in sum)
* | **jacobianODE2** [type = Real, default = 0.]:
  | jacobian w.r.t. coordinates of \ :ref:`ODE2 <ODE2>`\  equations (not counted in sum)
* | **jacobianODE2\_t** [type = Real, default = 0.]:
  | jacobian w.r.t. coordinates_t of \ :ref:`ODE2 <ODE2>`\  equations (not counted in sum)
* | **massMatrix** [type = Real, default = 0.]:
  | mass matrix computation
* | **newtonIncrement** [type = Real, default = 0.]:
  | Jac\ :math:`^{-1}`\  * RHS; backsubstitution
* | **ODE1RHS** [type = Real, default = 0.]:
  | time for residual evaluation of \ :ref:`ODE1 <ODE1>`\  right-hand-side
* | **ODE2RHS** [type = Real, default = 0.]:
  | time for residual evaluation of \ :ref:`ODE2 <ODE2>`\  right-hand-side
* | **overhead** [type = Real, default = 0.]:
  | overhead, such as initialization, copying and some matrix-vector multiplication
* | **postNewton** [type = Real, default = 0.]:
  | discontinuous iteration / PostNewtonStep
* | **python** [type = Real, default = 0.]:
  | time spent for Python functions
* | **reactionForces** [type = Real, default = 0.]:
  | CqT * lambda
* | **realtimeIdleCPU** [type = Real, default = 0.]:
  | time waited for next frame to compute and draw if simulateInRealtime is True
* | **Reset(useSolverTimer)** [return type = void]:
  | reset solver timings to initial state by assigning default values; useSolverTimer sets the useTimer flag
* | **StartTimer(value)** [return type = void]:
  | start timer function for a given variable; subtracts current CPU time from value
* | **StopTimer(value)** [return type = void]:
  | stop timer function for a given variable; adds current CPU time to value
* | **Sum()** [return type = Real]:
  | compute sum of all timers (except for those counted multiple, e.g., jacobians
* | **ToString()** [return type = String]:
  | converts the current timings to a string
* | **total** [type = Real, default = 0.]:
  | total time measured between start and end of computation (static/dynamics)
* | **totalJacobian** [type = Real, default = 0.]:
  | time for all jacobian computations
* | **useTimer** [type = bool, default = True]:
  | flag to decide, whether the timer is used (true) or not
* | **visualization** [type = Real, default = 0.]:
  | time spent for visualization in computation thread
* | **writeSolution** [type = Real, default = 0.]:
  | time for writing solution




.. _sec-solveriterationdata:

SolverIterationData
-------------------

Solver internal structure for counters, steps, step size, time, etc.; solution vectors, residuals, etc. are SolverLocalData. The given default values are overwritten by the simulationSettings when initializing the solver.

SolverIterationData has the following items:

* | **adaptiveStep** [type = bool, default = True]:
  | True: the step size may be reduced if step fails; no automatic stepsize control
* | **automaticStepSize** [type = bool, default = True]:
  | True: if timeIntegration.automaticStepSize == True AND chosen integrators supports automatic step size control (e.g., DOPRI5); False: constant step size used (step may be reduced if adaptiveStep=True)
* | **automaticStepSizeError** [type = Real, default = 0]:
  | estimated error (relative to atol + rtol*solution) of last step; must be \ :math:`\le 1`\   for a step to be accepted
* | **currentStepIndex** [type = Index, default = 0]:
  | current step index; \ :math:`i`\ 
* | **currentStepSize** [type = Real, default = 0.]:
  | stepSize of current step
* | **currentTime** [type = Real, default = 0.]:
  | holds the current simulation time, copy of state.current.time; interval is [startTime,tEnd]; in static solver, duration is loadStepDuration
* | **discontinuousIteration** [type = Index, default = 0]:
  | number of current discontinuous iteration
* | **discontinuousIterationsCount** [type = Index, default = 0]:
  | count total number of discontinuous iterations (min. 1 per step)
* | **endTime** [type = Real, default = 0.]:
  | end time of static/dynamic solver
* | **initialStepSize** [type = Real, default = 1e-6]:
  | initial stepSize for dynamic solver; only used, if automaticStepSize is activated
* | **lastStepSize** [type = Real, default = 0.]:
  | stepSize suggested from last step or by initial step size; only used, if automaticStepSize is activated
* | **maxStepSize** [type = Real, default = 0.]:
  | constant or maximum stepSize
* | **minStepSize** [type = Real, default = 0.]:
  | minimum stepSize for static/dynamic solver; only used, if automaticStepSize is activated
* | **newtonJacobiCount** [type = Index, default = 0]:
  | count total Newton jacobian computations
* | **newtonSteps** [type = Index, default = 0]:
  | number of current newton steps
* | **newtonStepsCount** [type = Index, default = 0]:
  | count total Newton steps
* | **numberOfSteps** [type = Index, default = 0]:
  | number of time steps (if fixed size); \ :math:`n`\ 
* | **recommendedStepSize** [type = Real, default = -1.]:
  | recommended step size \ :math:`h\_{recom}`\  after PostNewton(...): \ :math:`h\_{recom} < 0`\ : no recommendation, \ :math:`h\_{recom}==0`\ : use minimum step size, \ :math:`h\_{recom}>0`\ : use specific step size, if no smaller size requested by other reason
* | **rejectedAutomaticStepSizeSteps** [type = Index, default = 0]:
  | count the number of rejected steps in case of automatic step size control (rejected steps are repeated with smaller step size)
* | **rejectedModifiedNewtonSteps** [type = Index, default = 0]:
  | count the number of rejected modified Newton steps (switch to full Newton)
* | **startTime** [type = Real, default = 0.]:
  | time at beginning of time integration
* | **ToString()** [return type = String]:
  | convert iteration statistics to string; used for displayStatistics option



.. _sec-solverconvergencedata:

SolverConvergenceData
---------------------

Solver internal structure for convergence information: residua, iteration loop errors and error flags. For detailed behavior of these flags, visit the source code!. 

SolverConvergenceData has the following items:

* | **contractivity** [type = Real, default = 0.]:
  | Newton contractivity = geometric decay of error in every step
* | **discontinuousIterationError** [type = Real, default = 0.]:
  | error of discontinuous iterations (contact, friction, ...) outside of Newton iteration
* | **discontinuousIterationSuccessful** [type = bool, default = True]:
  | true, if last discontinuous iteration had success (failure may be recovered by adaptive step)
* | **errorCoordinateFactor** [type = Real, default = 1.]:
  | factor may include the number of system coordinates to reduce the residual
* | **InitializeData()** [return type = void]:
  | initialize SolverConvergenceData by assigning default values
* | **jacobianUpdateRequested** [type = bool, default = True]:
  | true, if a jacobian update is requested in modified Newton (determined in previous step)
* | **lastResidual** [type = Real, default = 0.]:
  | last Newton residual to determine contractivity
* | **linearSolverCausingRow** [type = Index, default = -1]:
  | -1 if successful, 0 ... n-1, the system equation (=coordinate) index which may have caused the problem, at which the linear solver failed
* | **linearSolverFailed** [type = bool, default = False]:
  | true, if linear solver failed to factorize
* | **massMatrixNotInvertible** [type = bool, default = False]:
  | true, if mass matrix is not invertable during initialization or solution (explicit solver)
* | **newtonConverged** [type = bool, default = False]:
  | true, if Newton has (finally) converged
* | **newtonSolutionDiverged** [type = bool, default = False]:
  | true, if Newton diverged (may be recovered)
* | **residual** [type = Real, default = 0.]:
  | current Newton residual
* | **stepReductionFailed** [type = bool, default = False]:
  | true, if iterations over time/static steps failed (finally, cannot be recovered)
* | **stopNewton** [type = bool, default = False]:
  | set true by Newton, if Newton was stopped, e.g., because of exceeding iterations or linear solver failed



.. _sec-solveroutputdata:

SolverOutputData
----------------

Solver internal structure for output modes, output timers and counters.

SolverOutputData has the following items:

* | **cpuLastTimePrinted** [type = Real, default = 0.]:
  | CPU time when output has been printed last time
* | **cpuStartTime** [type = Real, default = 0.]:
  | CPU start time of computation (starts counting at computation of initial conditions)
* | **finishedSuccessfully** [type = bool, default = False]:
  | flag is false until solver functions SolveSteps)...) or SolveSystem(...) finished successfully (can be used as external trigger)
* | **initializationSuccessful** [type = bool, default = False]:
  | flag is set during call to InitializeSolver(...); reasons for failure are multiple, either inconsistent solver settings are used, files cannot be written (file locked), or initial conditions could not be computed 
* | **InitializeData()** [return type = void]:
  | initialize SolverOutputData by assigning default values
* | **lastDiscontinuousIterationsCount** [type = Index, default = 0]:
  | discontinuous iterations count when written to console (or file) last time
* | **lastImageRecorded** [type = Real, default = 0.]:
  | simulation time when last image has been recorded
* | **lastNewtonJacobiCount** [type = Index, default = 0]:
  | jacobian update count when written to console (or file) last time
* | **lastNewtonStepsCount** [type = Index, default = 0]:
  | newton steps count when written to console (or file) last time
* | **lastSensorsWritten** [type = Real, default = 0.]:
  | simulation time when last sensors have been written
* | **lastSolutionWritten** [type = Real, default = 0.]:
  | simulation time when last solution has been written
* | **lastVerboseStepIndex** [type = Index, default = 0]:
  | step index when last time written to console (or file)
* | **multiThreadingMode** [type = Index, default = 0]:
  | multithreading mode that has been used: 0=None (serial), 1=multithreading, 2=multithreading with load balancing; (modes new since 2025-06, V1.9.198)
* | **numberOfThreadsUsed** [type = Index, default = 1]:
  | number of threads that have been used in simulation
* | **stepInformation** [type = Index, default = 0]:
  | this is a copy of the solvers stepInformation used for console output
* | **verboseMode** [type = Index, default = 0]:
  | this is a copy of the solvers verboseMode used for console output
* | **verboseModeFile** [type = Index, default = 0]:
  | this is a copy of the solvers verboseModeFile used for file
* | **writeToSolutionFile** [type = bool, default = False]:
  | if false, no solution file is generated and no file is written
* | **writeToSolverFile** [type = bool, default = False]:
  | if false, no solver output file is generated and no file is written


