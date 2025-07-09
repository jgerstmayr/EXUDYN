


.. _sec-simulationsettingsmain:


*******************
Simulation settings
*******************

This section includes hierarchical structures for simulation settings, e.g., time integration, static solver, Newton iteration and solution file export.


.. _sec-solutionsettings:

SolutionSettings
----------------

General settings for exporting the solution (results) of a simulation.

SolutionSettings has the following items:

* | **appendToFile** [type = bool, default = False]:
  | \ ``simulationSettings.solutionSettings.appendToFile``\ 
  | flag (true/false); if true, solution and solverInformation is appended to existing file (otherwise created); in BINARY mode, files are always replaced and this parameter is ineffective!
* | **binarySolutionFile** [type = bool, default = False]:
  | \ ``simulationSettings.solutionSettings.binarySolutionFile``\ 
  | if true, the solution file is written in binary format for improved speed and smaller file sizes; setting outputPrecision >= 8 uses double (8 bytes), otherwise float (4 bytes) is used; note that appendToFile is ineffective and files are always replaced without asking! If not provided, file ending will read .sol in case of binary files and .txt in case of text files
* | **coordinatesSolutionFileName** [type = FileName, default = 'coordinatesSolution']:
  | \ ``simulationSettings.solutionSettings.coordinatesSolutionFileName``\ 
  | filename and (relative) path of solution file (coordinatesSolutionFile) containing all multibody system coordinates versus time; directory will be created if it does not exist; character encoding of string is up to your filesystem, but for compatibility, it is recommended to use letters, numbers and '_' only; filename ending will be added automatically if not provided: .txt in case of text mode and .sol in case of binary solution files (binarySolutionFile=True)
* | **exportAccelerations** [type = bool, default = True]:
  | \ ``simulationSettings.solutionSettings.exportAccelerations``\ 
  | add \ :ref:`ODE2 <ODE2>`\  accelerations to solution file (coordinatesSolutionFile)
* | **exportAlgebraicCoordinates** [type = bool, default = True]:
  | \ ``simulationSettings.solutionSettings.exportAlgebraicCoordinates``\ 
  | add algebraicCoordinates (=Lagrange multipliers) to solution file (coordinatesSolutionFile)
* | **exportDataCoordinates** [type = bool, default = True]:
  | \ ``simulationSettings.solutionSettings.exportDataCoordinates``\ 
  | add DataCoordinates to solution file (coordinatesSolutionFile)
* | **exportODE1Velocities** [type = bool, default = True]:
  | \ ``simulationSettings.solutionSettings.exportODE1Velocities``\ 
  | add coordinatesODE1_t to solution file (coordinatesSolutionFile)
* | **exportVelocities** [type = bool, default = True]:
  | \ ``simulationSettings.solutionSettings.exportVelocities``\ 
  | add \ :ref:`ODE2 <ODE2>`\  velocities to solution file (coordinatesSolutionFile)
* | **flushFilesDOF** [type = PInt, default = 10000]:
  | \ ``simulationSettings.solutionSettings.flushFilesDOF``\ 
  | number of DOF, above which solution file (coordinatesSolutionFile) buffers are always flushed, irrespectively of whether flushFilesImmediately is set True or False (see also flushFilesImmediately); for larger files, writing takes so much time that flushing does not add considerable time
* | **flushFilesImmediately** [type = bool, default = False]:
  | \ ``simulationSettings.solutionSettings.flushFilesImmediately``\ 
  | flush file buffers after every solution period written (coordinatesSolutionFile and sensor files); if set False, the output is written through a buffer, which is highly efficient, but during simulation, files may be always in an incomplete state; if set True, this may add a large amount of CPU time as the process waits until files are really written to hard disc (especially for simulation of small scale systems, writing 10.000s of time steps; at least 5us per step/file, depending on hardware)
* | **outputPrecision** [type = UInt, default = 10]:
  | \ ``simulationSettings.solutionSettings.outputPrecision``\ 
  | precision for floating point numbers written to solution and sensor files
* | **recordImagesInterval** [type = Real, default = -1.]:
  | \ ``simulationSettings.solutionSettings.recordImagesInterval``\ 
  | record frames (images) during solving: amount of time to wait until next image (frame) is recorded; set recordImages = -1. if no images shall be recorded; set, e.g., recordImages = 0.01 to record an image every 10 milliseconds (requires that the time steps / load steps are sufficiently small!); for file names, etc., see VisualizationSettings.exportImages
* | **restartFileName** [type = FileName, default = 'restartFile.txt']:
  | \ ``simulationSettings.solutionSettings.restartFileName``\ 
  | filename and (relative) path of text file for storing solution after every restartWritePeriod if writeRestartFile=True; backup file is created with ending .bck, which should be used if restart file is crashed; use Python utility function InitializeFromRestartFile(...) to consistently restart
* | **restartWritePeriod** [type = UReal, default = 0.01]:
  | \ ``simulationSettings.solutionSettings.restartWritePeriod``\ 
  | time span (period), determines how often the restart file is updated; this should be often enough to enable restart without too much loss of data; too low values may influence performance
* | **sensorsAppendToFile** [type = bool, default = False]:
  | \ ``simulationSettings.solutionSettings.sensorsAppendToFile``\ 
  | flag (true/false); if true, sensor output is appended to existing file (otherwise created) or in case of internal storage, it is appended to existing currently stored data; this allows storing sensor values over different simulations
* | **sensorsStoreAndWriteFiles** [type = bool, default = True]:
  | \ ``simulationSettings.solutionSettings.sensorsStoreAndWriteFiles``\ 
  | flag (true/false); if false, no sensor files will be created and no sensor data will be stored; this may be advantageous for benchmarking as well as for special solvers which should not overwrite existing results (e.g. ComputeODE2Eigenvalues); settings this value to False may cause problems if sensors are required to perform operations which are needed e.g. in UserSensors as input of loads, etc.
* | **sensorsWriteFileFooter** [type = bool, default = False]:
  | \ ``simulationSettings.solutionSettings.sensorsWriteFileFooter``\ 
  | flag (true/false); if true, file footer is written for sensor output (turn off, e.g. for multiple runs of time integration)
* | **sensorsWriteFileHeader** [type = bool, default = True]:
  | \ ``simulationSettings.solutionSettings.sensorsWriteFileHeader``\ 
  | flag (true/false); if true, file header is written for sensor output (turn off, e.g. for multiple runs of time integration)
* | **sensorsWritePeriod** [type = UReal, default = 0.01]:
  | \ ``simulationSettings.solutionSettings.sensorsWritePeriod``\ 
  | time span (period), determines how often the sensor output is written to file or internal storage during a simulation
* | **solutionInformation** [type = String, default = '']:
  | \ ``simulationSettings.solutionSettings.solutionInformation``\ 
  | special information added to header of solution file (e.g. parameters and settings, modes, ...); character encoding my be UTF-8, restricted to characters in Section :ref:`sec-utf8`\ , but for compatibility, it is recommended to use ASCII characters only (95 characters, see wiki)
* | **solutionWritePeriod** [type = UReal, default = 0.01]:
  | \ ``simulationSettings.solutionSettings.solutionWritePeriod``\ 
  | time span (period), determines how often the solution file (coordinatesSolutionFile) is written during a simulation
* | **solverInformationFileName** [type = FileName, default = 'solverInformation.txt']:
  | \ ``simulationSettings.solutionSettings.solverInformationFileName``\ 
  | filename and (relative) path of text file showing detailed information during solving; detail level according to yourSolver.verboseModeFile; if solutionSettings.appendToFile is true, the information is appended in every solution step; directory will be created if it does not exist; character encoding of string is up to your filesystem, but for compatibility, it is recommended to use letters, numbers and '_' only
* | **writeFileFooter** [type = bool, default = True]:
  | \ ``simulationSettings.solutionSettings.writeFileFooter``\ 
  | flag (true/false); if true, information at end of simulation is written: convergence, total solution time, statistics
* | **writeFileHeader** [type = bool, default = True]:
  | \ ``simulationSettings.solutionSettings.writeFileHeader``\ 
  | flag (true/false); if true, file header is written (turn off, e.g. for multiple runs of time integration)
* | **writeInitialValues** [type = bool, default = True]:
  | \ ``simulationSettings.solutionSettings.writeInitialValues``\ 
  | flag (true/false); if true, initial values are exported for the start time; applies to coordinatesSolution and sensor files; this may not be wanted in the append file mode if the initial values are identical to the final values of a previous computation
* | **writeRestartFile** [type = bool, default = False]:
  | \ ``simulationSettings.solutionSettings.writeRestartFile``\ 
  | flag (true/false), which determines if restart file is written regularly, see restartFileName for details
* | **writeSolutionToFile** [type = bool, default = True]:
  | \ ``simulationSettings.solutionSettings.writeSolutionToFile``\ 
  | flag (true/false), which determines if (global) solution vector is written to the solution file (coordinatesSolutionFile); standard quantities that are written are: solution is written as displacements and coordinatesODE1; for additional coordinates in the solution file, see the options below



.. _sec-numericaldifferentiationsettings:

NumericalDifferentiationSettings
--------------------------------

Settings for numerical differentiation of a function (needed for computation of numerical jacobian e.g. in implizit integration).

NumericalDifferentiationSettings has the following items:

* | **addReferenceCoordinatesToEpsilon** [type = bool, default = False]:
  | \ ``simulationSettings.timeIntegration.newton.numericalDifferentiation.addReferenceCoordinatesToEpsilon``\ , \ ``simulationSettings.staticSolver.newton.numericalDifferentiation.addReferenceCoordinatesToEpsilon``\ 
  | True: for the size estimation of the differentiation parameter, the reference coordinate \ :math:`q^{Ref}\_i`\  is added to \ :ref:`ODE2 <ODE2>`\  coordinates --> see; False: only the current coordinate is used for size estimation of the differentiation parameter
* | **doSystemWideDifferentiation** [type = bool, default = False]:
  | \ ``simulationSettings.timeIntegration.newton.numericalDifferentiation.doSystemWideDifferentiation``\ , \ ``simulationSettings.staticSolver.newton.numericalDifferentiation.doSystemWideDifferentiation``\ 
  | True: system wide differentiation (e.g. all \ :ref:`ODE2 <ODE2>`\  equations w.r.t. all \ :ref:`ODE2 <ODE2>`\  coordinates); False: only local (object) differentiation
* | **forAE** [type = bool, default = False]:
  | \ ``simulationSettings.timeIntegration.newton.numericalDifferentiation.forAE``\ , \ ``simulationSettings.staticSolver.newton.numericalDifferentiation.forAE``\ 
  | flag (true/false); false = perform direct computation of jacobian for algebraic equations (AE), true = use numerical differentiation; as there must always exist an analytical implemented jacobian for AE, 'true' should only be used for verification
* | **forODE2** [type = bool, default = False]:
  | \ ``simulationSettings.timeIntegration.newton.numericalDifferentiation.forODE2``\ , \ ``simulationSettings.staticSolver.newton.numericalDifferentiation.forODE2``\ 
  | flag (true/false); false = perform direct computation (e.g., using autodiff) of jacobian for ODE2 equations, true = use numerical differentiation; numerical differentiation is less efficient and may lead to numerical problems, but may smoothen problems of analytical derivatives; sometimes the analytical derivative may neglect terms
* | **forODE2connectors** [type = bool, default = False]:
  | \ ``simulationSettings.timeIntegration.newton.numericalDifferentiation.forODE2connectors``\ , \ ``simulationSettings.staticSolver.newton.numericalDifferentiation.forODE2connectors``\ 
  | flag (true/false); false: if also forODE2==false, perform direct computation of jacobian for ODE2 terms for connectors; else: use numerical differentiation; NOTE: THIS FLAG IS FOR DEVELOPMENT AND WILL BE ERASED IN FUTURE
* | **jacobianConnectorDerivative** [type = bool, default = True]:
  | \ ``simulationSettings.timeIntegration.newton.numericalDifferentiation.jacobianConnectorDerivative``\ , \ ``simulationSettings.staticSolver.newton.numericalDifferentiation.jacobianConnectorDerivative``\ 
  | True: for analytic Jacobians of connectors, the Jacobian derivative is computed, causing additional CPU costs and not beeing available for all connectors or markers (thus switching to numerical differentiation); False: Jacobian derivative is neglected in analytic Jacobians (but included in numerical Jacobians), which often has only minor influence on convergence
* | **minimumCoordinateSize** [type = UReal, default = 1e-2]:
  | \ ``simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize``\ , \ ``simulationSettings.staticSolver.newton.numericalDifferentiation.minimumCoordinateSize``\ 
  | minimum size of coordinates in relative differentiation parameter
* | **relativeEpsilon** [type = UReal, default = 1e-7]:
  | \ ``simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon``\ , \ ``simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon``\ 
  | relative differentiation parameter epsilon; the numerical differentiation parameter \ :math:`\varepsilon`\  follows from the formula (\ :math:`\varepsilon = \varepsilon\_\mathrm{relative}*max(q\_{min}, |q\_i + [q^{Ref}\_i]|)`\ , with \ :math:`\varepsilon\_\mathrm{relative}`\ =relativeEpsilon, \ :math:`q\_{min} =`\ minimumCoordinateSize, \ :math:`q\_i`\  is the current coordinate which is differentiated, and \ :math:`qRef\_i`\  is the reference coordinate of the current coordinate



.. _sec-discontinuoussettings:

DiscontinuousSettings
---------------------

Settings for discontinuous iterations, as in contact, friction, plasticity and general switching phenomena.

DiscontinuousSettings has the following items:

* | **ignoreMaxIterations** [type = bool, default = True]:
  | \ ``simulationSettings.timeIntegration.discontinuous.ignoreMaxIterations``\ , \ ``simulationSettings.staticSolver.discontinuous.ignoreMaxIterations``\ 
  | continue solver if maximum number of discontinuous (post Newton) iterations is reached (ignore tolerance)
* | **iterationTolerance** [type = UReal, default = 1]:
  | \ ``simulationSettings.timeIntegration.discontinuous.iterationTolerance``\ , \ ``simulationSettings.staticSolver.discontinuous.iterationTolerance``\ 
  | absolute tolerance for discontinuous (post Newton) iterations; the errors represent absolute residuals and can be quite high
* | **maxIterations** [type = UInt, default = 5]:
  | \ ``simulationSettings.timeIntegration.discontinuous.maxIterations``\ , \ ``simulationSettings.staticSolver.discontinuous.maxIterations``\ 
  | maximum number of discontinuous (post Newton) iterations
* | **useRecommendedStepSize** [type = bool, default = True]:
  | \ ``simulationSettings.timeIntegration.discontinuous.useRecommendedStepSize``\ , \ ``simulationSettings.staticSolver.discontinuous.useRecommendedStepSize``\ 
  | some objects (contact-related) provide a recommendedStepSize; if True, this recommendation is used, but may lead to very small step sizes and solver could fail if restrictions are too hard; set to False to ignore this recommendation



.. _sec-newtonsettings:

NewtonSettings
--------------

Settings for Newton method used in static or dynamic simulation.

NewtonSettings has the following items:

* | **numericalDifferentiation** [type = NumericalDifferentiationSettings]:
  | \ ``simulationSettings.timeIntegration.newton.numericalDifferentiation``\ , \ ``simulationSettings.staticSolver.newton.numericalDifferentiation``\ 
  | numerical differentiation parameters for numerical jacobian (e.g. Newton in static solver or implicit time integration)
* | **absoluteTolerance** [type = UReal, default = 1e-10]:
  | \ ``simulationSettings.timeIntegration.newton.absoluteTolerance``\ , \ ``simulationSettings.staticSolver.newton.absoluteTolerance``\ 
  | absolute tolerance of residual for Newton (needed e.g. if residual is fulfilled right at beginning); condition: sqrt(q*q)/numberOfCoordinates <= absoluteTolerance
* | **adaptInitialResidual** [type = bool, default = True]:
  | \ ``simulationSettings.timeIntegration.newton.adaptInitialResidual``\ , \ ``simulationSettings.staticSolver.newton.adaptInitialResidual``\ 
  | flag (true/false); false = standard; True: if initialResidual is very small (or zero), it may increase significantely in the first Newton iteration; to achieve relativeTolerance, the initialResidual will by updated by a higher residual within the first Newton iteration
* | **maximumSolutionNorm** [type = UReal, default = 1e38]:
  | \ ``simulationSettings.timeIntegration.newton.maximumSolutionNorm``\ , \ ``simulationSettings.staticSolver.newton.maximumSolutionNorm``\ 
  | this is the maximum allowed value for solutionU.L2NormSquared() which is the square of the square norm (i.e., value=\ :math:`u\_1^2`\ +\ :math:`u\_2^2`\ +...), and solutionV/A...; if the norm of solution vectors is larger, Newton method is stopped; the default value is chosen such that it would still work for single precision numbers (float)
* | **maxIterations** [type = UInt, default = 25]:
  | \ ``simulationSettings.timeIntegration.newton.maxIterations``\ , \ ``simulationSettings.staticSolver.newton.maxIterations``\ 
  | maximum number of iterations (including modified + restart Newton iterations); after that total number of iterations, the static/dynamic solver refines the step size or stops with an error
* | **maxModifiedNewtonIterations** [type = UInt, default = 8]:
  | \ ``simulationSettings.timeIntegration.newton.maxModifiedNewtonIterations``\ , \ ``simulationSettings.staticSolver.newton.maxModifiedNewtonIterations``\ 
  | maximum number of iterations for modified Newton (without Jacobian update); after that number of iterations, the modified Newton method gets a jacobian update and is further iterated
* | **maxModifiedNewtonRestartIterations** [type = UInt, default = 7]:
  | \ ``simulationSettings.timeIntegration.newton.maxModifiedNewtonRestartIterations``\ , \ ``simulationSettings.staticSolver.newton.maxModifiedNewtonRestartIterations``\ 
  | maximum number of iterations for modified Newton after a Jacobian update; after that number of iterations, the full Newton method is started for this step
* | **modifiedNewtonContractivity** [type = PReal, default = 0.5]:
  | \ ``simulationSettings.timeIntegration.newton.modifiedNewtonContractivity``\ , \ ``simulationSettings.staticSolver.newton.modifiedNewtonContractivity``\ 
  | maximum contractivity (=reduction of error in every Newton iteration) accepted by modified Newton; if contractivity is greater, a Jacobian update is computed
* | **modifiedNewtonJacUpdatePerStep** [type = bool, default = False]:
  | \ ``simulationSettings.timeIntegration.newton.modifiedNewtonJacUpdatePerStep``\ , \ ``simulationSettings.staticSolver.newton.modifiedNewtonJacUpdatePerStep``\ 
  | True: compute Jacobian at every time step (or static step), but not in every Newton iteration (except for bad convergence ==> switch to full Newton)
* | **newtonResidualMode** [type = UInt, default = 0]:
  | \ ``simulationSettings.timeIntegration.newton.newtonResidualMode``\ , \ ``simulationSettings.staticSolver.newton.newtonResidualMode``\ 
  | 0 ... use residual for computation of error (standard); 1 ... use \ :ref:`ODE2 <ODE2>`\  and \ :ref:`ODE1 <ODE1>`\  newton increment for error (set relTol and absTol to same values!) ==> may be advantageous if residual is zero, e.g., in kinematic analysis; TAKE CARE with this flag
* | **relativeTolerance** [type = UReal, default = 1e-8]:
  | \ ``simulationSettings.timeIntegration.newton.relativeTolerance``\ , \ ``simulationSettings.staticSolver.newton.relativeTolerance``\ 
  | relative tolerance of residual for Newton (general goal of Newton is to decrease the residual by this factor)
* | **useModifiedNewton** [type = bool, default = False]:
  | \ ``simulationSettings.timeIntegration.newton.useModifiedNewton``\ , \ ``simulationSettings.staticSolver.newton.useModifiedNewton``\ 
  | True: compute Jacobian only at first call to solver; the Jacobian (and its factorizations) is not computed in each Newton iteration, even not in every (time integration) step; False: Jacobian (and factorization) is computed in every Newton iteration (default, but may be costly)
* | **useNewtonSolver** [type = bool, default = True]:
  | \ ``simulationSettings.timeIntegration.newton.useNewtonSolver``\ , \ ``simulationSettings.staticSolver.newton.useNewtonSolver``\ 
  | flag (true/false); false = linear computation, true = use Newton solver for nonlinear solution
* | **weightTolerancePerCoordinate** [type = bool, default = False]:
  | \ ``simulationSettings.timeIntegration.newton.weightTolerancePerCoordinate``\ , \ ``simulationSettings.staticSolver.newton.weightTolerancePerCoordinate``\ 
  | flag (true/false); false = compute error as L2-Norm of residual; true = compute error as (L2-Norm of residual) / (sqrt(number of coordinates)), which can help to use common tolerance independent of system size



.. _sec-generalizedalphasettings:

GeneralizedAlphaSettings
------------------------

Settings for generalized-alpha, implicit trapezoidal or Newmark time integration methods.

GeneralizedAlphaSettings has the following items:

* | **computeInitialAccelerations** [type = bool, default = True]:
  | \ ``simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations``\ 
  | True: compute initial accelerations from system EOM in acceleration form; NOTE that initial accelerations that are following from user functions in constraints are not considered for now! False: use zero accelerations
* | **lieGroupAddTangentOperator** [type = bool, default = True]:
  | \ ``simulationSettings.timeIntegration.generalizedAlpha.lieGroupAddTangentOperator``\ 
  | True: for Lie group nodes, in case that lieGroupSimplifiedKinematicRelations=True, the integrator adds the tangent operator for stiffness and constraint matrices, for improved Newton convergence; not available for sparse matrix mode (EigenSparse)
* | **lieGroupSimplifiedKinematicRelations** [type = bool, default = False]:
  | \ ``simulationSettings.timeIntegration.generalizedAlpha.lieGroupSimplifiedKinematicRelations``\ 
  | True: for Lie group nodes, the integrator uses the original kinematic relations of the Bruls and Cardona 2010 paper
* | **newmarkBeta** [type = UReal, default = 0.25]:
  | \ ``simulationSettings.timeIntegration.generalizedAlpha.newmarkBeta``\ 
  | value beta for Newmark method; default value beta = \ :math:`\frac 1 4`\  corresponds to (undamped) trapezoidal rule
* | **newmarkGamma** [type = UReal, default = 0.5]:
  | \ ``simulationSettings.timeIntegration.generalizedAlpha.newmarkGamma``\ 
  | value gamma for Newmark method; default value gamma = \ :math:`\frac 1 2`\  corresponds to (undamped) trapezoidal rule
* | **resetAccelerations** [type = bool, default = False]:
  | \ ``simulationSettings.timeIntegration.generalizedAlpha.resetAccelerations``\ 
  | this flag only affects if computeInitialAccelerations=False: if resetAccelerations=True, accelerations are set zero in the solver function InitializeSolverInitialConditions; this may be unwanted in case of repeatedly called SolveSteps() and in cases where solutions shall be prolonged from previous computations
* | **spectralRadius** [type = UReal, default = 0.9]:
  | \ ``simulationSettings.timeIntegration.generalizedAlpha.spectralRadius``\ 
  | spectral radius for Generalized-alpha solver; set this value to 1 for no damping or to 0 < spectralRadius < 1 for damping of high-frequency dynamics; for position-level constraints (index 3), spectralRadius must be < 1
* | **useIndex2Constraints** [type = bool, default = False]:
  | \ ``simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints``\ 
  | set useIndex2Constraints = true in order to use index2 (velocity level constraints) formulation
* | **useNewmark** [type = bool, default = False]:
  | \ ``simulationSettings.timeIntegration.generalizedAlpha.useNewmark``\ 
  | if true, use Newmark method with beta and gamma instead of generalized-Alpha



.. _sec-explicitintegrationsettings:

ExplicitIntegrationSettings
---------------------------

Settings for explicit solvers, like Explicit Euler, RK44, ODE23, DOPRI5 and others. The settings may significantely influence performance.

ExplicitIntegrationSettings has the following items:

* | **computeEndOfStepAccelerations** [type = bool, default = True]:
  | \ ``simulationSettings.timeIntegration.explicitIntegration.computeEndOfStepAccelerations``\ 
  | accelerations are computed at stages of the explicit integration scheme; if the user needs accelerations at the end of a step, this flag needs to be activated; if True, this causes a second call to the RHS of the equations, which may DOUBLE COMPUTATIONAL COSTS for one-step-methods; if False, the accelerations are re-used from the last stage, being slightly different
* | **computeMassMatrixInversePerBody** [type = bool, default = False]:
  | \ ``simulationSettings.timeIntegration.explicitIntegration.computeMassMatrixInversePerBody``\ 
  | If true, the solver assumes the bodies to be independent and computes the inverse of the mass matrix for all bodies independently; this may lead to WRONG RESULTS, if bodies share nodes, e.g., two MassPoint objects put on the same node or a beam with a mass point attached at a shared node; however, it may speed up explicit time integration for large systems significantly (multi-threaded)
* | **dynamicSolverType** [type = DynamicSolverType, default = DynamicSolverType::DOPRI5]:
  | \ ``simulationSettings.timeIntegration.explicitIntegration.dynamicSolverType``\ 
  | selection of explicit solver type (DOPRI5, ExplicitEuler, ExplicitMidpoint, RK44, RK67, VelocityVerlet, ...), for detailed description see DynamicSolverType, Section :ref:`sec-dynamicsolvertype`\ , but only referring to explicit solvers.
* | **eliminateConstraints** [type = bool, default = True]:
  | \ ``simulationSettings.timeIntegration.explicitIntegration.eliminateConstraints``\ 
  | True: make explicit solver work for simple CoordinateConstraints, which are eliminated for ground constraints (e.g. fixed nodes in finite element models). False: incompatible constraints are ignored (BE CAREFUL)!
* | **useLieGroupIntegration** [type = bool, default = True]:
  | \ ``simulationSettings.timeIntegration.explicitIntegration.useLieGroupIntegration``\ 
  | True: use Lie group integration for rigid body nodes; must be turned on for Lie group nodes (without data coordinates) to work properly; does not work for nodes with data coordinates!



.. _sec-timeintegrationsettings:

TimeIntegrationSettings
-----------------------

General parameters used in time integration; specific parameters are provided in the according solver settings, e.g. for generalizedAlpha.

TimeIntegrationSettings has the following items:

* | **discontinuous** [type = DiscontinuousSettings]:
  | \ ``simulationSettings.timeIntegration.discontinuous``\ 
  | parameters for treatment of discontinuities
* | **explicitIntegration** [type = ExplicitIntegrationSettings]:
  | \ ``simulationSettings.timeIntegration.explicitIntegration``\ 
  | special parameters for explicit time integration
* | **generalizedAlpha** [type = GeneralizedAlphaSettings]:
  | \ ``simulationSettings.timeIntegration.generalizedAlpha``\ 
  | parameters for generalized-alpha, implicit trapezoidal rule or Newmark (options only apply for these methods)
* | **newton** [type = NewtonSettings]:
  | \ ``simulationSettings.timeIntegration.newton``\ 
  | parameters for Newton method; used for implicit time integration methods only
* | **absoluteTolerance** [type = UReal, default = 1e-8]:
  | \ ``simulationSettings.timeIntegration.absoluteTolerance``\ 
  | \ :math:`a\_{tol}`\ : if automaticStepSize=True, absolute tolerance for the error control; must fulfill \ :math:`a\_{tol} > 0`\ ; see Section :ref:`sec-explicitsolver`\ 
* | **adaptiveStep** [type = bool, default = True]:
  | \ ``simulationSettings.timeIntegration.adaptiveStep``\ 
  | True: the step size may be reduced if step fails; no automatic stepsize control
* | **adaptiveStepDecrease** [type = UReal, default = 0.5]:
  | \ ``simulationSettings.timeIntegration.adaptiveStepDecrease``\ 
  | Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors
* | **adaptiveStepIncrease** [type = UReal, default = 2]:
  | \ ``simulationSettings.timeIntegration.adaptiveStepIncrease``\ 
  | Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors
* | **adaptiveStepRecoveryIterations** [type = UInt, default = 7]:
  | \ ``simulationSettings.timeIntegration.adaptiveStepRecoveryIterations``\ 
  | Number of max. (Newton iterations + discontinuous iterations) at which a step increase is considered; in order to immediately increase steps after reduction, chose a high value
* | **adaptiveStepRecoverySteps** [type = UInt, default = 10]:
  | \ ``simulationSettings.timeIntegration.adaptiveStepRecoverySteps``\ 
  | Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors
* | **automaticStepSize** [type = bool, default = True]:
  | \ ``simulationSettings.timeIntegration.automaticStepSize``\ 
  | True: for specific integrators with error control (e.g., DOPRI5), compute automatic step size based on error estimation; False: constant step size (step may be reduced if adaptiveStep=True); the maximum stepSize reads \ :math:`h = h\_{max} = \frac{t\_{end} - t\_{start}}{n\_{steps}}`\ 
* | **computeLoadsJacobian** [type = UInt, default = 0]:
  | \ ``simulationSettings.timeIntegration.computeLoadsJacobian``\ 
  | 0:  jacobian of loads not considered (may lead to slow convergence or Newton failure); 1: in case of implicit integrators, compute (numerical) Jacobian of ODE2 and ODE1 coordinates for loads, causing additional computational costs; this is advantageous in cases where loads are related nonlinearly to coordinates; 2: also compute ODE2_t dependencies for jacobian; note that computeLoadsJacobian has no effect in case of doSystemWideDifferentiation, as this anyway includes all load dependencies
* | **endTime** [type = UReal, default = 1]:
  | \ ``simulationSettings.timeIntegration.endTime``\ 
  | \ :math:`t\_{end}`\ : end time of time integration
* | **initialStepSize** [type = UReal, default = 0]:
  | \ ``simulationSettings.timeIntegration.initialStepSize``\ 
  | \ :math:`h\_{init}`\ : if automaticStepSize=True, initial step size; if initialStepSize==0, max. stepSize, which is (endTime-startTime)/numberOfSteps, is used as initial guess; a good choice of initialStepSize may help the solver to start up faster.
* | **minimumStepSize** [type = PReal, default = 1e-8]:
  | \ ``simulationSettings.timeIntegration.minimumStepSize``\ 
  | \ :math:`h\_{min}`\ : if automaticStepSize=True or adaptiveStep=True: lower limit of time step size, before integrator stops with adaptiveStep; lower limit of automaticStepSize control (continues but raises warning)
* | **numberOfSteps** [type = PReal, default = 100]:
  | \ ``simulationSettings.timeIntegration.numberOfSteps``\ 
  | \ :math:`n\_{steps}`\ : number of steps in time integration; (maximum) stepSize \ :math:`h`\  is computed from \ :math:`h = \frac{t\_{end} - t\_{start}}{n\_{steps}}`\ ; for automatic stepsize control, this stepSize is the maximum steps size, \ :math:`h\_{max} = h`\ ; numberOfSteps can be a float-point type, but must be close to an integer (relative tolerance \ :math:`100\cdot\varepsilon`\ ) as it is silently rounded to int
* | **realtimeFactor** [type = PReal, default = 1]:
  | \ ``simulationSettings.timeIntegration.realtimeFactor``\ 
  | if simulateInRealtime=True, this factor is used to make the simulation slower than realtime (factor < 1) or faster than realtime (factor > 1)
* | **realtimeWaitMicroseconds** [type = PInt, default = 1000]:
  | \ ``simulationSettings.timeIntegration.realtimeWaitMicroseconds``\ 
  | if simulateInRealtime=True, a loop runs which waits realtimeWaitMicroseconds until checking again if the realtime is reached; using larger values leads to less CPU usage but less accurate realtime accuracy; smaller values (< 1000) increase CPU usage but improve realtime accuracy
* | **relativeTolerance** [type = UReal, default = 1e-8]:
  | \ ``simulationSettings.timeIntegration.relativeTolerance``\ 
  | \ :math:`r\_{tol}`\ : if automaticStepSize=True, relative tolerance for the error control; must fulfill \ :math:`r\_{tol} \ge 0`\ ; see Section :ref:`sec-explicitsolver`\ 
* | **reuseConstantMassMatrix** [type = bool, default = True]:
  | \ ``simulationSettings.timeIntegration.reuseConstantMassMatrix``\ 
  | True: does not recompute constant mass matrices (e.g. of some finite elements, mass points, etc.); if False, it always recomputes the mass matrix (e.g. needed, if user changes mass parameters via Python)
* | **simulateInRealtime** [type = bool, default = False]:
  | \ ``simulationSettings.timeIntegration.simulateInRealtime``\ 
  | True: simulate in realtime; the solver waits for computation of the next step until the CPU time reached the simulation time; if the simulation is slower than realtime, it simply continues
* | **startTime** [type = UReal, default = 0]:
  | \ ``simulationSettings.timeIntegration.startTime``\ 
  | \ :math:`t\_{start}`\ : start time of time integration (usually set to zero)
* | **stepInformation** [type = UInt, default = 67]:
  | \ ``simulationSettings.timeIntegration.stepInformation``\ 
  | add up the following binary flags: 0 ... show only step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step or period, 4 ... show Newton jacobians (jac) per step or period, 8 ... show discontinuous iterations (Dit) per step or period, 16 ... show step size (dt), 32 ... show CPU time spent; 64 ... show adaptive step reduction warnings; 128 ... show step increase information; 1024 ... show every time step; time is usually shown in fractions of seconds (s), hours (h), or days
* | **stepSizeMaxIncrease** [type = UReal, default = 2]:
  | \ ``simulationSettings.timeIntegration.stepSizeMaxIncrease``\ 
  | \ :math:`f\_{maxInc}`\ : if automaticStepSize=True, maximum increase of step size per step, see Section :ref:`sec-explicitsolver`\ ; make this factor smaller (but \ :math:`> 1`\ ) if too many rejected steps
* | **stepSizeSafety** [type = UReal, default = 0.90]:
  | \ ``simulationSettings.timeIntegration.stepSizeSafety``\ 
  | \ :math:`r\_{sfty}`\ : if automaticStepSize=True, a safety factor added to estimated optimal step size, in order to prevent from many rejected steps, see Section :ref:`sec-explicitsolver`\ . Make this factor smaller if many steps are rejected.
* | **verboseMode** [type = UInt, default = 0]:
  | \ ``simulationSettings.timeIntegration.verboseMode``\ 
  | 0 ... no output, 1 ... show short step information every 2 seconds (every 30 seconds after 1 hour CPU time), 2 ... show every step information, 3 ... show also solution vector, 4 ... show also mass matrix and jacobian (implicit methods), 5 ... show also Jacobian inverse (implicit methods)
* | **verboseModeFile** [type = UInt, default = 0]:
  | \ ``simulationSettings.timeIntegration.verboseModeFile``\ 
  | same behaviour as verboseMode, but outputs all solver information to file



.. _sec-staticsolversettings:

StaticSolverSettings
--------------------

Settings for static solver linear or nonlinear (Newton).

StaticSolverSettings has the following items:

* | **discontinuous** [type = DiscontinuousSettings]:
  | \ ``simulationSettings.staticSolver.discontinuous``\ 
  | parameters for treatment of discontinuities
* | **newton** [type = NewtonSettings]:
  | \ ``simulationSettings.staticSolver.newton``\ 
  | parameters for Newton method (e.g. in static solver or time integration)
* | **adaptiveStep** [type = bool, default = True]:
  | \ ``simulationSettings.staticSolver.adaptiveStep``\ 
  | True: use step reduction if step fails; False: fixed step size
* | **adaptiveStepDecrease** [type = UReal, default = 0.25]:
  | \ ``simulationSettings.staticSolver.adaptiveStepDecrease``\ 
  | Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors
* | **adaptiveStepIncrease** [type = UReal, default = 2]:
  | \ ``simulationSettings.staticSolver.adaptiveStepIncrease``\ 
  | Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors
* | **adaptiveStepRecoveryIterations** [type = UInt, default = 7]:
  | \ ``simulationSettings.staticSolver.adaptiveStepRecoveryIterations``\ 
  | Number of max. (Newton iterations + discontinuous iterations) at which a step increase is considered; in order to immediately increase steps after reduction, chose a high value
* | **adaptiveStepRecoverySteps** [type = UInt, default = 4]:
  | \ ``simulationSettings.staticSolver.adaptiveStepRecoverySteps``\ 
  | Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors
* | **computeLoadsJacobian** [type = bool, default = True]:
  | \ ``simulationSettings.staticSolver.computeLoadsJacobian``\ 
  | True: compute (currently numerical) Jacobian for loads, causing additional computational costs; this is advantageous in cases where loads are related nonlinearly to coordinates; False: jacobian of loads not considered (may lead to slow convergence or Newton failure); note that computeLoadsJacobian has no effect in case of doSystemWideDifferentiation, as this anyway includes all load dependencies
* | **constrainODE1coordinates** [type = bool, default = True]:
  | \ ``simulationSettings.staticSolver.constrainODE1coordinates``\ 
  | True: ODE1coordinates are constrained to initial values; False: undefined behavior, currently not supported
* | **loadStepDuration** [type = PReal, default = 1]:
  | \ ``simulationSettings.staticSolver.loadStepDuration``\ 
  | quasi-time for all load steps (added to current time in load steps)
* | **loadStepGeometric** [type = bool, default = False]:
  | \ ``simulationSettings.staticSolver.loadStepGeometric``\ 
  | if loadStepGeometric=false, the load steps are incremental (arithmetic series, e.g. 0.1,0.2,0.3,...); if true, the load steps are increased in a geometric series, e.g. for \ :math:`n=8`\  numberOfLoadSteps and \ :math:`d = 1000`\  loadStepGeometricRange, it follows: \ :math:`1000^{1/8}/1000=0.00237`\ , \ :math:`1000^{2/8}/1000=0.00562`\ , \ :math:`1000^{3/8}/1000=0.0133`\ , ..., \ :math:`1000^{7/8}/1000=0.422`\ , \ :math:`1000^{8/8}/1000=1`\ 
* | **loadStepGeometricRange** [type = PReal, default = 1000]:
  | \ ``simulationSettings.staticSolver.loadStepGeometricRange``\ 
  | if loadStepGeometric=true, the load steps are increased in a geometric series, see loadStepGeometric
* | **loadStepStart** [type = UReal, default = 0]:
  | \ ``simulationSettings.staticSolver.loadStepStart``\ 
  | a quasi time, which can be used for the output (first column) as well as for time-dependent forces; quasi-time is increased in every step i by loadStepDuration/numberOfLoadSteps; loadStepTime = loadStepStart + i*loadStepDuration/numberOfLoadSteps, but loadStepStart untouched ==> increment by user
* | **minimumStepSize** [type = PReal, default = 1e-8]:
  | \ ``simulationSettings.staticSolver.minimumStepSize``\ 
  | lower limit of step size, before nonlinear solver stops
* | **numberOfLoadSteps** [type = PInt, default = 1]:
  | \ ``simulationSettings.staticSolver.numberOfLoadSteps``\ 
  | number of load steps; if numberOfLoadSteps=1, no load steps are used and full forces are applied at once
* | **stabilizerODE2term** [type = UReal, default = 0]:
  | \ ``simulationSettings.staticSolver.stabilizerODE2term``\ 
  | add mass-proportional stabilizer term in \ :ref:`ODE2 <ODE2>`\  part of jacobian for stabilization (scaled ), e.g. of badly conditioned problems; the diagnoal terms are scaled with \ :math:`stabilizer = (1-loadStepFactor^2)`\ , and go to zero at the end of all load steps: \ :math:`loadStepFactor=1`\  -> \ :math:`stabilizer = 0`\ 
* | **stepInformation** [type = UInt, default = 67]:
  | \ ``simulationSettings.staticSolver.stepInformation``\ 
  | add up the following binary flags: 0 ... show only step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step or period, 4 ... show Newton jacobians (jac) per step or period, 8 ... show discontinuous iterations (Dit) per step or period, 16 ... show step size (dt), 32 ... show CPU time spent; 64 ... show adaptive step reduction warnings; 128 ... show step increase information; 1024 ... show every time step; time is usually shown in fractions of seconds (s), hours (h), or days
* | **useLoadFactor** [type = bool, default = True]:
  | \ ``simulationSettings.staticSolver.useLoadFactor``\ 
  | True: compute a load factor \ :math:`\in [0,1]`\  from static step time; all loads are scaled by the load factor; False: loads are always scaled with 1 -- use this option if time dependent loads use a userFunction
* | **verboseMode** [type = UInt, default = 1]:
  | \ ``simulationSettings.staticSolver.verboseMode``\ 
  | 0 ... no output, 1 ... show errors and load steps, 2 ... show short Newton step information (error), 3 ... show also solution vector, 4 ... show also jacobian, 5 ... show also Jacobian inverse
* | **verboseModeFile** [type = UInt, default = 0]:
  | \ ``simulationSettings.staticSolver.verboseModeFile``\ 
  | same behaviour as verboseMode, but outputs all solver information to file



.. _sec-linearsolversettings:

LinearSolverSettings
--------------------

Settings for linear solver, both dense and sparse (Eigen).

LinearSolverSettings has the following items:

* | **ignoreSingularJacobian** [type = bool, default = False]:
  | \ ``simulationSettings.linearSolverSettings.ignoreSingularJacobian``\ 
  | [ONLY implemented for dense, Eigen matrix mode] False: standard way, fails if jacobian is singular; True: use Eigen's FullPivLU (thus only works with LinearSolverType.EigenDense) which handles over- and underdetermined systems; can often resolve redundant constraints, but MAY ALSO LEAD TO ERRONEOUS RESULTS!
* | **pivotThreshold** [type = UReal, default = 0]:
  | \ ``simulationSettings.linearSolverSettings.pivotThreshold``\ 
  | [ONLY available for EXUdense and EigenDense (FullPivot) solver] threshold for dense linear solver, can be used to detect close to singular solutions, setting this to, e.g., 1e-12; solver then reports on equations that are causing close to singularity
* | **reuseAnalyzedPattern** [type = bool, default = False]:
  | \ ``simulationSettings.linearSolverSettings.reuseAnalyzedPattern``\ 
  | [ONLY available for sparse matrices] True: the Eigen SparseLU solver offers the possibility to reuse an analyzed pattern of a previous factorization; this may reduce total factorization time by a factor of 2 or 3, depending on the matrix type; however, if the matrix patterns heavily change between computations, this may even slow down performance; this flag is set for SparseMatrices in InitializeSolverData(...) and should be handled with care!
* | **showCausingItems** [type = bool, default = True]:
  | \ ``simulationSettings.linearSolverSettings.showCausingItems``\ 
  | False: no output, if solver fails; True: if redundant equations appear, they are resolved such that according solution variables are set to zero; in case of redundant constraints, this may help, but it may lead to erroneous behaviour; for static problems, this may suppress static motion or resolve problems in case of instabilities, but should in general be considered with care!



.. _sec-parallel:

Parallel
--------

Settings for linear solver, both dense and sparse (Eigen).

Parallel has the following items:

* | **multithreadedLLimitJacobians** [type = PInt, default = 20]:
  | \ ``simulationSettings.parallel.multithreadedLLimitJacobians``\ 
  | compute jacobians (ODE2, AE, ...) multi-threaded; this is the limit number of according objects from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
* | **multithreadedLLimitLoads** [type = PInt, default = 20]:
  | \ ``simulationSettings.parallel.multithreadedLLimitLoads``\ 
  | compute loads multi-threaded; this is the limit number of loads from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
* | **multithreadedLLimitMassMatrices** [type = PInt, default = 20]:
  | \ ``simulationSettings.parallel.multithreadedLLimitMassMatrices``\ 
  | compute bodies mass matrices multi-threaded; this is the limit number of bodies from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
* | **multithreadedLLimitResiduals** [type = PInt, default = 20]:
  | \ ``simulationSettings.parallel.multithreadedLLimitResiduals``\ 
  | compute RHS vectors, AE, and reaction forces multi-threaded; this is the limit number of objects from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)
* | **numberOfThreads** [type = PInt, default = 1]:
  | \ ``simulationSettings.parallel.numberOfThreads``\ 
  | number of threads used for parallel computation (1 == scalar processing); do not use more threads than available threads (in most cases it is good to restrict to the number of cores); currently, only one solver can be started with multithreading; if you use several mbs in parallel (co-simulation), you should use serial computing
* | **taskSplitMinItems** [type = PInt, default = 50]:
  | \ ``simulationSettings.parallel.taskSplitMinItems``\ 
  | number of items from which on the tasks are split into subtasks (which slightly increases threading performance; this may be critical for smaller number of objects, should be roughly between 50 and 5000; flag is copied into MainSystem internal flag at InitializeSolverData(...)
* | **taskSplitTasksPerThread** [type = PInt, default = 16]:
  | \ ``simulationSettings.parallel.taskSplitTasksPerThread``\ 
  | this is the number of subtasks that every thread receives; minimum is 1, the maximum should not be larger than 100; this factor is 1 as long as the taskSplitMinItems is not reached; flag is copied into MainSystem internal flag at InitializeSolverData(...)
* | **useLoadBalancing** [type = bool, default = True]:
  | \ ``simulationSettings.parallel.useLoadBalancing``\ 
  | if True, parallel computation uses load balancing, which may give better performance in case of non-equilibrated loads; (mobile) Intel CPUs may perform better without load balancing; this flag is coupled to exudyn.special.solver.multiThreadingLoadBalancing (overwritten when solver starts with multithreading)



.. _sec-simulationsettings:

SimulationSettings
------------------

General Settings for simulation; according settings for solution and solvers are given in subitems of this structure. 

SimulationSettings has the following items:

* | **linearSolverSettings** [type = LinearSolverSettings]:
  | \ ``.simulationSettings.linearSolverSettings``\ 
  | linear solver parameters (used for dense and sparse solvers)
* | **parallel** [type = Parallel]:
  | \ ``.simulationSettings.parallel``\ 
  | parameters for vectorized and parallelized (multi-threaded) computations
* | **solutionSettings** [type = SolutionSettings]:
  | \ ``.simulationSettings.solutionSettings``\ 
  | settings for solution files
* | **staticSolver** [type = StaticSolverSettings]:
  | \ ``.simulationSettings.staticSolver``\ 
  | static solver parameters
* | **timeIntegration** [type = TimeIntegrationSettings]:
  | \ ``.simulationSettings.timeIntegration``\ 
  | time integration parameters
* | **cleanUpMemory** [type = bool, default = False]:
  | \ ``.simulationSettings.cleanUpMemory``\ 
  | True: solvers will free memory at exit (recommended for large systems); False: keep allocated memory for repeated computations to increase performance
* | **displayComputationTime** [type = bool, default = False]:
  | \ ``.simulationSettings.displayComputationTime``\ 
  | display computation time statistics at end of solving
* | **displayGlobalTimers** [type = bool, default = True]:
  | \ ``.simulationSettings.displayGlobalTimers``\ 
  | display global timer statistics at end of solving (e.g., for contact, but also for internal timings during development)
* | **displayStatistics** [type = bool, default = False]:
  | \ ``.simulationSettings.displayStatistics``\ 
  | display general computation information at end of time step (steps, iterations, function calls, step rejections, ...
* | **linearSolverType** [type = LinearSolverType, default = LinearSolverType::EXUdense]:
  | \ ``.simulationSettings.linearSolverType``\ 
  | selection of numerical linear solver: exu.LinearSolverType.EXUdense (dense matrix inverse), exu.LinearSolverType.EigenSparse (sparse matrix LU-factorization), ... (enumeration type)
* | **outputPrecision** [type = UInt, default = 6]:
  | \ ``.simulationSettings.outputPrecision``\ 
  | precision for floating point numbers written to console; e.g. values written by solver
* | **pauseAfterEachStep** [type = bool, default = False]:
  | \ ``.simulationSettings.pauseAfterEachStep``\ 
  | pause after every time step or static load step(user press SPACE)

