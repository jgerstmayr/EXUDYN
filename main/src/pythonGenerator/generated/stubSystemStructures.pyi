
#This is the stub file for system structures, such as SimulationSettings and VisualizationSettings
#This file will greatly improve autocompletion


#information for BeamSection
class BeamSection:
    """Data structure for definition of 2D and 3D beam (cross) section mechanical properties. The beam has local coordinates, in which :math:`X` represents the beam centerline (beam axis) coordinate, being the neutral fiber w.r.t. bending; :math:`Y` and :math:`Z` are the local cross section coordinates. Note that most elements do not accept all parameters, which results in an error if those parameters (e.g., stiffness parameters) are non-zero."""
    dampingMatrix: ArrayLike
    r"""[SI:Nsm:math:`^2`, Nsm and Ns (mixed)] sectional linear damping matrix related to :math:`\vp{{}^{c}{\nv}}{{}^{c}{\mv}} = {}^{c}{\Dm} \vp{{}^{c}{\tepsDot}}{{}^{c}{\tkappaDot}}`; note that this damping models is highly simplified and usually, it cannot be derived from material parameters; however, it can be used to adjust model damping to observed damping behavior. Set with list of lists or numpy array."""
    inertia: ArrayLike
    r"""[SI:kg:math:`\,`m:math:`^2`] sectional inertia for shear-deformable beams. Set with list of lists or numpy array."""
    massPerLength: float
    """[SI:kg/m] mass per unit length of the beam."""
    stiffnessMatrix: ArrayLike
    r"""[SI:Nm:math:`^2`, Nm and N (mixed)] sectional stiffness matrix related to :math:`\vp{{}^{c}{\nv}}{{}^{c}{\mv}} = {}^{c}{\Cm} \vp{{}^{c}{\teps}}{{}^{c}{\tkappa}}` with sectional normal force :math:`{}^{c}{\nv}`, torque :math:`{}^{c}{\mv}`, strain :math:`{}^{c}{\teps}` and curvature :math:`{}^{c}{\tkappa}`, all quantities expressed in the cross section frame :math:`c`. Set with list of lists or numpy array."""

#information for BeamSectionGeometry
class BeamSectionGeometry:
    """Data structure for definition of 2D and 3D beam (cross) section geometrical properties. Used for visualization and contact."""
    crossSectionRadiusY: float
    """[SI:m] :math:`Y` radius for circular cross section."""
    crossSectionRadiusZ: float
    """[SI:m] :math:`Z` radius for circular cross section."""
    crossSectionType: CrossSectionType
    """Type of cross section: Polygon, Circular, etc."""
    polygonalPoints: Vector2DList
    """[SI: (m,m) ] list of polygonal (:math:`Y,Z`) points in local beam cross section coordinates, defined in positive rotation direction."""

#information for SolutionSettings
class SolutionSettings:
    """General settings for exporting the solution (results) of a simulation."""
    appendToFile: bool
    """flag (true/false); if true, solution and solverInformation is appended to existing file (otherwise created); in BINARY mode, files are always replaced and this parameter is ineffective!"""
    binarySolutionFile: bool
    """if true, the solution file is written in binary format for improved speed and smaller file sizes; setting outputPrecision >= 8 uses double (8 bytes), otherwise float (4 bytes) is used; note that appendToFile is ineffective and files are always replaced without asking! If not provided, file ending will read .sol in case of binary files and .txt in case of text files."""
    coordinatesSolutionFileName: str
    """filename and (relative) path of solution file (coordinatesSolutionFile) containing all multibody system coordinates versus time; directory will be created if it does not exist; character encoding of string is up to your filesystem, but for compatibility, it is recommended to use letters, numbers and '_' only; filename ending will be added automatically if not provided: .txt in case of text mode and .sol in case of binary solution files (binarySolutionFile=True)."""
    exportAccelerations: bool
    """add ODE2 accelerations to solution file (coordinatesSolutionFile)."""
    exportAlgebraicCoordinates: bool
    """add algebraicCoordinates (=Lagrange multipliers) to solution file (coordinatesSolutionFile)."""
    exportDataCoordinates: bool
    """add DataCoordinates to solution file (coordinatesSolutionFile)."""
    exportODE1Velocities: bool
    """add coordinatesODE1_t to solution file (coordinatesSolutionFile)."""
    exportVelocities: bool
    """add ODE2 velocities to solution file (coordinatesSolutionFile)."""
    flushFilesDOF: int
    """number of DOF, above which solution file (coordinatesSolutionFile) buffers are always flushed, irrespectively of whether flushFilesImmediately is set True or False (see also flushFilesImmediately); for larger files, writing takes so much time that flushing does not add considerable time."""
    flushFilesImmediately: bool
    """flush file buffers after every solution period written (coordinatesSolutionFile and sensor files); if set False, the output is written through a buffer, which is highly efficient, but during simulation, files may be always in an incomplete state; if set True, this may add a large amount of CPU time as the process waits until files are really written to hard disc (especially for simulation of small scale systems, writing 10.000s of time steps; at least 5us per step/file, depending on hardware)."""
    outputPrecision: int
    """precision for floating point numbers written to solution and sensor files."""
    recordImagesInterval: float
    """record frames of the main view in the renderer (images) during solving: amount of time to wait until next image (frame) is recorded; set recordImages = -1. if no images shall be recorded; set, e.g., recordImages = 0.01 to record an image every 10 milliseconds (requires that the time steps / load steps are sufficiently small!); for file names, etc., see VisualizationSettings.exportImages; note that only the main view (0) can be saved in this way, while for multiple views, you have to aquire data via renderer.RedrawAndGetImage()."""
    restartFileName: str
    """filename and (relative) path of text file for storing solution after every restartWritePeriod if writeRestartFile=True; backup file is created with ending .bck, which should be used if restart file is crashed; use Python utility function InitializeFromRestartFile(...) to consistently restart."""
    restartWritePeriod: float
    """time span (period), determines how often the restart file is updated; this should be often enough to enable restart without too much loss of data; too low values may influence performance."""
    sensorsAppendToFile: bool
    """flag (true/false); if true, sensor output is appended to existing file (otherwise created) or in case of internal storage, it is appended to existing currently stored data; this allows storing sensor values over different simulations."""
    sensorsStoreAndWriteFiles: bool
    """flag (true/false); if false, no sensor files will be created and no sensor data will be stored; this may be advantageous for benchmarking as well as for special solvers which should not overwrite existing results (e.g. ComputeODE2Eigenvalues); settings this value to False may cause problems if sensors are required to perform operations which are needed e.g. in UserSensors as input of loads, etc."""
    sensorsWriteFileFooter: bool
    """flag (true/false); if true, file footer is written for sensor output (turn off, e.g. for multiple runs of time integration)."""
    sensorsWriteFileHeader: bool
    """flag (true/false); if true, file header is written for sensor output (turn off, e.g. for multiple runs of time integration)."""
    sensorsWritePeriod: float
    """time span (period), determines how often the sensor output is written to file or internal storage during a simulation."""
    solutionInformation: str
    """special information added to header of solution file (e.g. parameters and settings, modes, ...); character encoding my be UTF-8, restricted to characters in theDoc.pdf, but for compatibility, it is recommended to use ASCII characters only (95 characters, see wiki)."""
    solutionWritePeriod: float
    """time span (period), determines how often the solution file (coordinatesSolutionFile) is written during a simulation."""
    solverInformationFileName: str
    """filename and (relative) path of text file showing detailed information during solving; detail level according to yourSolver.verboseModeFile; if solutionSettings.appendToFile is true, the information is appended in every solution step; directory will be created if it does not exist; character encoding of string is up to your filesystem, but for compatibility, it is recommended to use letters, numbers and '_' only."""
    writeFileFooter: bool
    """flag (true/false); if true, information at end of simulation is written: convergence, total solution time, statistics."""
    writeFileHeader: bool
    """flag (true/false); if true, file header is written (turn off, e.g. for multiple runs of time integration)."""
    writeInitialValues: bool
    """flag (true/false); if true, initial values are exported for the start time; applies to coordinatesSolution and sensor files; this may not be wanted in the append file mode if the initial values are identical to the final values of a previous computation."""
    writeRestartFile: bool
    """flag (true/false), which determines if restart file is written regularly, see restartFileName for details."""
    writeSolutionToFile: bool
    """flag (true/false), which determines if (global) solution vector is written to the solution file (coordinatesSolutionFile); standard quantities that are written are: solution is written as displacements and coordinatesODE1; for additional coordinates in the solution file, see the options below."""

#information for NumericalDifferentiationSettings
class NumericalDifferentiationSettings:
    """Settings for numerical differentiation of a function (needed for computation of numerical jacobian e.g. in implizit integration)."""
    addReferenceCoordinatesToEpsilon: bool
    """True: for the size estimation of the differentiation parameter, the reference coordinate :math:`q^{Ref}_i` is added to ODE2 coordinates --> see; False: only the current coordinate is used for size estimation of the differentiation parameter."""
    doSystemWideDifferentiation: bool
    """True: system wide differentiation (e.g. all ODE2 equations w.r.t. all ODE2 coordinates); False: only local (object) differentiation."""
    forAE: bool
    """flag (true/false); false = perform direct computation of jacobian for algebraic equations (AE), true = use numerical differentiation; as there must always exist an analytical implemented jacobian for AE, 'true' should only be used for verification."""
    forODE2: bool
    """flag (true/false); false = perform direct computation (e.g., using autodiff) of jacobian for ODE2 equations, true = use numerical differentiation; numerical differentiation is less efficient and may lead to numerical problems, but may smoothen problems of analytical derivatives; sometimes the analytical derivative may neglect terms."""
    forODE2connectors: bool
    """flag (true/false); false: if also forODE2==false, perform direct computation of jacobian for ODE2 terms for connectors; else: use numerical differentiation; NOTE: THIS FLAG IS FOR DEVELOPMENT AND WILL BE ERASED IN FUTURE."""
    jacobianConnectorDerivative: bool
    """True: for analytic Jacobians of connectors, the Jacobian derivative is computed, causing additional CPU costs and not beeing available for all connectors or markers (thus switching to numerical differentiation); False: Jacobian derivative is neglected in analytic Jacobians (but included in numerical Jacobians), which often has only minor influence on convergence."""
    minimumCoordinateSize: float
    """minimum size of coordinates in relative differentiation parameter."""
    relativeEpsilon: float
    r"""relative differentiation parameter epsilon; the numerical differentiation parameter :math:`\varepsilon` follows from the formula (:math:`\varepsilon = \varepsilon_\mathrm{relative}*max(q_{min}, |q_i + [q^{Ref}_i]|)`, with :math:`\varepsilon_\mathrm{relative}`=relativeEpsilon, :math:`q_{min} = `minimumCoordinateSize, :math:`q_i` is the current coordinate which is differentiated, and :math:`qRef_i` is the reference coordinate of the current coordinate."""

#information for DiscontinuousSettings
class DiscontinuousSettings:
    """Settings for discontinuous iterations, as in contact, friction, plasticity and general switching phenomena."""
    ignoreMaxIterations: bool
    """continue solver if maximum number of discontinuous (post Newton) iterations is reached (ignore tolerance)."""
    iterationTolerance: float
    """absolute tolerance for discontinuous (post Newton) iterations; the errors represent absolute residuals and can be quite high."""
    maxIterations: int
    """maximum number of discontinuous (post Newton) iterations."""
    useRecommendedStepSize: bool
    """some objects (contact-related) provide a recommendedStepSize; if True, this recommendation is used, but may lead to very small step sizes and solver could fail if restrictions are too hard; set to False to ignore this recommendation."""

#information for NewtonSettings
class NewtonSettings:
    """Settings for Newton method used in static or dynamic simulation."""
    numericalDifferentiation: NumericalDifferentiationSettings
    """numerical differentiation parameters for numerical jacobian (e.g. Newton in static solver or implicit time integration)."""
    absoluteTolerance: float
    """absolute tolerance of residual for Newton (needed e.g. if residual is fulfilled right at beginning); condition: sqrt(q*q)/numberOfCoordinates <= absoluteTolerance."""
    adaptInitialResidual: bool
    """flag (true/false); false = standard; True: if initialResidual is very small (or zero), it may increase significantely in the first Newton iteration; to achieve relativeTolerance, the initialResidual will by updated by a higher residual within the first Newton iteration."""
    maximumSolutionNorm: float
    """this is the maximum allowed value for solutionU.L2NormSquared() which is the square of the square norm (i.e., value=:math:`u_1^2`+:math:`u_2^2`+...), and solutionV/A...; if the norm of solution vectors is larger, Newton method is stopped; the default value is chosen such that it would still work for single precision numbers (float)."""
    maxIterations: int
    """maximum number of iterations (including modified + restart Newton iterations); after that total number of iterations, the static/dynamic solver refines the step size or stops with an error."""
    maxModifiedNewtonIterations: int
    """maximum number of iterations for modified Newton (without Jacobian update); after that number of iterations, the modified Newton method gets a jacobian update and is further iterated."""
    maxModifiedNewtonRestartIterations: int
    """maximum number of iterations for modified Newton after a Jacobian update; after that number of iterations, the full Newton method is started for this step."""
    modifiedNewtonContractivity: float
    """maximum contractivity (=reduction of error in every Newton iteration) accepted by modified Newton; if contractivity is greater, a Jacobian update is computed."""
    modifiedNewtonJacUpdatePerStep: bool
    """True: compute Jacobian at every time step (or static step), but not in every Newton iteration (except for bad convergence ==> switch to full Newton)."""
    newtonResidualMode: int
    """0 ... use residual for computation of error (standard); 1 ... use ODE2 and ODE1 newton increment for error (set relTol and absTol to same values!) ==> may be advantageous if residual is zero, e.g., in kinematic analysis; TAKE CARE with this flag."""
    relativeTolerance: float
    """relative tolerance of residual for Newton (general goal of Newton is to decrease the residual by this factor)."""
    useModifiedNewton: bool
    """True: compute Jacobian only at first call to solver; the Jacobian (and its factorizations) is not computed in each Newton iteration, even not in every (time integration) step; False: Jacobian (and factorization) is computed in every Newton iteration (default, but may be costly)."""
    useNewtonSolver: bool
    """flag (true/false); false = linear computation, true = use Newton solver for nonlinear solution."""
    weightTolerancePerCoordinate: bool
    """flag (true/false); false = compute error as L2-Norm of residual; true = compute error as (L2-Norm of residual) / (sqrt(number of coordinates)), which can help to use common tolerance independent of system size."""

#information for GeneralizedAlphaSettings
class GeneralizedAlphaSettings:
    """Settings for generalized-alpha, implicit trapezoidal or Newmark time integration methods."""
    computeInitialAccelerations: bool
    """True: compute initial accelerations from system EOM in acceleration form; NOTE that initial accelerations that are following from user functions in constraints are not considered for now! False: use zero accelerations."""
    lieGroupAddTangentOperator: bool
    """True: for Lie group nodes, in case that lieGroupSimplifiedKinematicRelations=True, the integrator adds the tangent operator for stiffness and constraint matrices, for improved Newton convergence; not available for sparse matrix mode (EigenSparse)."""
    lieGroupSimplifiedKinematicRelations: bool
    """True: for Lie group nodes, the integrator uses the original kinematic relations of the Bruls and Cardona 2010 paper; False (recommended): higher accuracy as proposed in paper by Holzinger, Arnold, Gerstmayr, sigma-modified Lie group generalized alpha methods for constrained multibody systems, 2025 (to be sumitted)."""
    newmarkBeta: float
    r"""value beta for Newmark method; default value beta = :math:`\frac 1 4` corresponds to (undamped) trapezoidal rule."""
    newmarkGamma: float
    r"""value gamma for Newmark method; default value gamma = :math:`\frac 1 2` corresponds to (undamped) trapezoidal rule."""
    resetAccelerations: bool
    """this flag only affects if computeInitialAccelerations=False: if resetAccelerations=True, accelerations are set zero in the solver function InitializeSolverInitialConditions; this may be unwanted in case of repeatedly called SolveSteps() and in cases where solutions shall be prolonged from previous computations."""
    spectralRadius: float
    """spectral radius for Generalized-alpha solver; set this value to 1 for no damping or to 0 < spectralRadius < 1 for damping of high-frequency dynamics; for position-level constraints (index 3), spectralRadius must be < 1."""
    storeInitialAlgebraicCoordinates: bool
    """True: IF computeInitialAccelerations=True, store initial algebraic coordinates (usually the Lagrange multipliers) in the initial coordinates vector (and thus in the first line of the coordinates solution file); for further details on limitations, see computeInitialAccelerations."""
    useIndex2Constraints: bool
    """set useIndex2Constraints = true in order to use index2 (velocity level constraints) formulation."""
    useNewmark: bool
    """if true, use Newmark method with beta and gamma instead of generalized-Alpha."""

#information for ExplicitIntegrationSettings
class ExplicitIntegrationSettings:
    """Settings for explicit solvers, like Explicit Euler, RK44, ODE23, DOPRI5 and others. The settings may significantely influence performance."""
    computeEndOfStepAccelerations: bool
    """accelerations are computed at stages of the explicit integration scheme; if the user needs accelerations at the end of a step, this flag needs to be activated; if True, this causes a second call to the RHS of the equations, which may DOUBLE COMPUTATIONAL COSTS for one-step-methods; if False, the accelerations are re-used from the last stage, being slightly different."""
    computeMassMatrixInversePerBody: bool
    """If true, the solver assumes the bodies to be independent and computes the inverse of the mass matrix for all bodies independently; this may lead to WRONG RESULTS, if bodies share nodes, e.g., two MassPoint objects put on the same node or a beam with a mass point attached at a shared node; however, it may speed up explicit time integration for large systems significantly (multi-threaded)."""
    dynamicSolverType: DynamicSolverType
    """selection of explicit solver type (DOPRI5, ExplicitEuler, ExplicitMidpoint, RK44, RK67, VelocityVerlet, ...), for detailed description see DynamicSolverType, theDoc.pdf, but only referring to explicit solvers."""
    eliminateConstraints: bool
    """True: make explicit solver work for simple CoordinateConstraints, which are eliminated for ground constraints (e.g. fixed nodes in finite element models). False: incompatible constraints are ignored (BE CAREFUL)!"""
    useLieGroupIntegration: bool
    """True: use Lie group integration for rigid body nodes; must be turned on for Lie group nodes (without data coordinates) to work properly; does not work for nodes with data coordinates!"""

#information for TimeIntegrationSettings
class TimeIntegrationSettings:
    """General parameters used in time integration; specific parameters are provided in the according solver settings, e.g. for generalizedAlpha."""
    discontinuous: DiscontinuousSettings
    """parameters for treatment of discontinuities."""
    explicitIntegration: ExplicitIntegrationSettings
    """special parameters for explicit time integration."""
    generalizedAlpha: GeneralizedAlphaSettings
    """parameters for generalized-alpha, implicit trapezoidal rule or Newmark (options only apply for these methods)."""
    newton: NewtonSettings
    """parameters for Newton method; used for implicit time integration methods only."""
    absoluteTolerance: float
    """: if automaticStepSize=True, absolute tolerance for the error control; must fulfill :math:`a_{tol} > 0`; see theDoc.pdf."""
    adaptiveStep: bool
    """True: the step size may be reduced if step fails; no automatic stepsize control."""
    adaptiveStepDecrease: float
    """Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors."""
    adaptiveStepIncrease: float
    """Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors."""
    adaptiveStepRecoveryIterations: int
    """Number of max. (Newton iterations + discontinuous iterations) at which a step increase is considered; in order to immediately increase steps after reduction, chose a high value."""
    adaptiveStepRecoverySteps: int
    """Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors."""
    automaticStepSize: bool
    r"""True: for specific integrators with error control (e.g., DOPRI5), compute automatic step size based on error estimation; False: constant step size (step may be reduced if adaptiveStep=True); the maximum stepSize reads :math:`h = h_{max} = \frac{t_{end} - t_{start}}{n_{steps}}`."""
    computeLoadsJacobian: int
    """0:  jacobian of loads not considered (may lead to slow convergence or Newton failure); 1: in case of implicit integrators, compute (numerical) Jacobian of ODE2 and ODE1 coordinates for loads, causing additional computational costs; this is advantageous in cases where loads are related nonlinearly to coordinates; 2: also compute ODE2_t dependencies for jacobian; note that computeLoadsJacobian has no effect in case of doSystemWideDifferentiation, as this anyway includes all load dependencies."""
    endTime: float
    """: end time of time integration."""
    initialStepSize: float
    """: if automaticStepSize=True, initial step size; if initialStepSize==0, max. stepSize, which is (endTime-startTime)/numberOfSteps, is used as initial guess; a good choice of initialStepSize may help the solver to start up faster."""
    minimumStepSize: float
    """: if automaticStepSize=True or adaptiveStep=True: lower limit of time step size, before integrator stops with adaptiveStep; lower limit of automaticStepSize control (continues but raises warning)."""
    numberOfSteps: float
    r""": number of steps in time integration; (maximum) stepSize :math:`h` is computed from :math:`h = \frac{t_{end} - t_{start}}{n_{steps}}`; for automatic stepsize control, this stepSize is the maximum steps size, :math:`h_{max} = h`; numberOfSteps can also be a float type, but must be close to an integer (relative tolerance :math:`100\cdot\varepsilon`) as it is silently rounded to int."""
    realtimeFactor: float
    """if simulateInRealtime=True, this factor is used to make the simulation slower than realtime (factor < 1) or faster than realtime (factor > 1)."""
    realtimeWaitMicroseconds: int
    """if simulateInRealtime=True, a loop runs which waits realtimeWaitMicroseconds until checking again if the realtime is reached; using larger values leads to less CPU usage but less accurate realtime accuracy; smaller values (< 1000) increase CPU usage but improve realtime accuracy."""
    relativeTolerance: float
    r""": if automaticStepSize=True, relative tolerance for the error control; must fulfill :math:`r_{tol} \ge 0`; see theDoc.pdf."""
    reuseConstantMassMatrix: bool
    """True: does not recompute constant mass matrices (e.g. of some finite elements, mass points, etc.); if False, it always recomputes the mass matrix (e.g. needed, if user changes mass parameters via Python)."""
    simulateInRealtime: bool
    """True: simulate in realtime; the solver waits for computation of the next step until the CPU time reached the simulation time; if the simulation is slower than realtime, it simply continues."""
    startTime: float
    """: start time of time integration (usually set to zero)."""
    stepInformation: int
    """add up the following binary flags: 0 ... show only step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step or period, 4 ... show Newton jacobians (jac) per step or period, 8 ... show discontinuous iterations (Dit) per step or period, 16 ... show step size (dt), 32 ... show CPU time spent; 64 ... show adaptive step reduction warnings; 128 ... show step increase information; 1024 ... show every time step; time is usually shown in fractions of seconds (s), hours (h), or days."""
    stepSizeMaxIncrease: float
    """: if automaticStepSize=True, maximum increase of step size per step, see theDoc.pdf; make this factor smaller (but :math:`> 1`) if too many rejected steps."""
    stepSizeSafety: float
    """: if automaticStepSize=True, a safety factor added to estimated optimal step size, in order to prevent from many rejected steps, see theDoc.pdf. Make this factor smaller if many steps are rejected."""
    verboseMode: int
    """0 ... no output, 1 ... show short step information every 2 seconds (every 30 seconds after 1 hour CPU time), 2 ... show every step information, 3 ... show also solution vector, 4 ... show also mass matrix and jacobian (implicit methods), 5 ... show also Jacobian inverse (implicit methods)."""
    verboseModeFile: int
    """same behaviour as verboseMode, but outputs all solver information to file."""

#information for StaticSolverSettings
class StaticSolverSettings:
    """Settings for static solver linear or nonlinear (Newton)."""
    discontinuous: DiscontinuousSettings
    """parameters for treatment of discontinuities."""
    newton: NewtonSettings
    """parameters for Newton method (e.g. in static solver or time integration)."""
    adaptiveStep: bool
    """True: use step reduction if step fails; False: fixed step size."""
    adaptiveStepDecrease: float
    """Multiplicative factor (MUST BE: 0 < factor < 1) for step size to decrese due to discontinuousIteration or Newton errors."""
    adaptiveStepIncrease: float
    """Multiplicative factor (MUST BE > 1) for step size to increase after previous step reduction due to discontinuousIteration or Newton errors."""
    adaptiveStepRecoveryIterations: int
    """Number of max. (Newton iterations + discontinuous iterations) at which a step increase is considered; in order to immediately increase steps after reduction, chose a high value."""
    adaptiveStepRecoverySteps: int
    """Number of steps needed after which steps will be increased after previous step reduction due to discontinuousIteration or Newton errors."""
    computeLoadsJacobian: bool
    """True: compute (currently numerical) Jacobian for loads, causing additional computational costs; this is advantageous in cases where loads are related nonlinearly to coordinates; False: jacobian of loads not considered (may lead to slow convergence or Newton failure); note that computeLoadsJacobian has no effect in case of doSystemWideDifferentiation, as this anyway includes all load dependencies."""
    constrainODE1coordinates: bool
    """True: ODE1coordinates are constrained to initial values; False: undefined behavior, currently not supported."""
    loadStepDuration: float
    """quasi-time for all load steps (added to current time in load steps)."""
    loadStepGeometric: bool
    """if loadStepGeometric=false, the load steps are incremental (arithmetic series, e.g. 0.1,0.2,0.3,...); if true, the load steps are increased in a geometric series, e.g. for :math:`n=8` numberOfLoadSteps and :math:`d = 1000` loadStepGeometricRange, it follows: :math:`1000^{1/8}/1000=0.00237`, :math:`1000^{2/8}/1000=0.00562`, :math:`1000^{3/8}/1000=0.0133`, ..., :math:`1000^{7/8}/1000=0.422`, :math:`1000^{8/8}/1000=1`."""
    loadStepGeometricRange: float
    """if loadStepGeometric=true, the load steps are increased in a geometric series, see loadStepGeometric."""
    loadStepStart: float
    """a quasi time, which can be used for the output (first column) as well as for time-dependent forces; quasi-time is increased in every step i by loadStepDuration/numberOfLoadSteps; loadStepTime = loadStepStart + i*loadStepDuration/numberOfLoadSteps, but loadStepStart untouched ==> increment by user."""
    minimumStepSize: float
    """lower limit of step size, before nonlinear solver stops."""
    numberOfLoadSteps: int
    """number of load steps; if numberOfLoadSteps=1, no load steps are used and full forces are applied at once."""
    stabilizerODE2term: float
    """add mass-proportional stabilizer term in ODE2 part of jacobian for stabilization (scaled ), e.g. of badly conditioned problems; the diagnoal terms are scaled with :math:`stabilizer = (1-loadStepFactor^2)`, and go to zero at the end of all load steps: :math:`loadStepFactor=1` -> :math:`stabilizer = 0`."""
    stepInformation: int
    """add up the following binary flags: 0 ... show only step time, 1 ... show time to go, 2 ... show newton iterations (Nit) per step or period, 4 ... show Newton jacobians (jac) per step or period, 8 ... show discontinuous iterations (Dit) per step or period, 16 ... show step size (dt), 32 ... show CPU time spent; 64 ... show adaptive step reduction warnings; 128 ... show step increase information; 1024 ... show every time step; time is usually shown in fractions of seconds (s), hours (h), or days."""
    useLoadFactor: bool
    r"""True: compute a load factor :math:`\in [0,1]` from static step time; all loads are scaled by the load factor; False: loads are always scaled with 1 -- use this option if time dependent loads use a userFunction."""
    verboseMode: int
    """0 ... no output, 1 ... show errors and load steps, 2 ... show short Newton step information (error), 3 ... show also solution vector, 4 ... show also jacobian, 5 ... show also Jacobian inverse."""
    verboseModeFile: int
    """same behaviour as verboseMode, but outputs all solver information to file."""

#information for LinearSolverSettings
class LinearSolverSettings:
    """Settings for linear solver, both dense and sparse (Eigen)."""
    ignoreSingularJacobian: bool
    """[ONLY implemented for dense, Eigen matrix mode] False: standard way, fails if jacobian is singular; True: use Eigen's FullPivLU (thus only works with LinearSolverType.EigenDense) which handles over- and underdetermined systems; can often resolve redundant constraints, but MAY ALSO LEAD TO ERRONEOUS RESULTS!"""
    pivotThreshold: float
    """[ONLY available for EXUdense and EigenDense (FullPivot) solver] threshold for dense linear solver, can be used to detect close to singular solutions, setting this to, e.g., 1e-12; solver then reports on equations that are causing close to singularity."""
    reuseAnalyzedPattern: bool
    """[ONLY available for sparse matrices] True: the Eigen SparseLU solver offers the possibility to reuse an analyzed pattern of a previous factorization; this may reduce total factorization time by a factor of 2 or 3, depending on the matrix type; however, if the matrix patterns heavily change between computations, this may even slow down performance; this flag is set for SparseMatrices in InitializeSolverData(...) and should be handled with care!"""
    showCausingItems: bool
    """False: no output, if solver fails; True: if redundant equations appear, they are resolved such that according solution variables are set to zero; in case of redundant constraints, this may help, but it may lead to erroneous behaviour; for static problems, this may suppress static motion or resolve problems in case of instabilities, but should in general be considered with care!"""

#information for Parallel
class Parallel:
    """Settings for linear solver, both dense and sparse (Eigen)."""
    multithreadedLLimitJacobians: int
    """compute jacobians (ODE2, AE, ...) multi-threaded; this is the limit number of according objects from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)."""
    multithreadedLLimitLoads: int
    """compute loads multi-threaded; this is the limit number of loads from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)."""
    multithreadedLLimitMassMatrices: int
    """compute bodies mass matrices multi-threaded; this is the limit number of bodies from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)."""
    multithreadedLLimitResiduals: int
    """compute RHS vectors, AE, and reaction forces multi-threaded; this is the limit number of objects from which on parallelization is used; flag is copied into MainSystem internal flag at InitializeSolverData(...)."""
    numberOfThreads: int
    """number of threads used for parallel computation (1 == scalar processing); do not use more threads than available threads (in most cases it is good to restrict to the number of cores); currently, only one solver can be started with multithreading; if you use several mbs in parallel (co-simulation), you should use serial computing."""
    taskSplitMinItems: int
    """number of items from which on the tasks are split into subtasks (which slightly increases threading performance; this may be critical for smaller number of objects, should be roughly between 50 and 5000; flag is copied into MainSystem internal flag at InitializeSolverData(...)."""
    taskSplitTasksPerThread: int
    """this is the number of subtasks that every thread receives; minimum is 1, the maximum should not be larger than 100; this factor is 1 as long as the taskSplitMinItems is not reached; flag is copied into MainSystem internal flag at InitializeSolverData(...)."""
    useLoadBalancing: bool
    """if True, parallel computation uses load balancing, which may give better performance in case of non-equilibrated loads; (mobile) Intel CPUs may perform better without load balancing; this flag is coupled to exudyn.special.solver.multiThreadingLoadBalancing (overwritten when solver starts with multithreading)."""

#information for SimulationSettings
class SimulationSettings:
    """General Settings for simulation; according settings for solution and solvers are given in subitems of this structure."""
    linearSolverSettings: LinearSolverSettings
    """linear solver parameters (used for dense and sparse solvers)."""
    parallel: Parallel
    """parameters for vectorized and parallelized (multi-threaded) computations."""
    solutionSettings: SolutionSettings
    """settings for solution files."""
    staticSolver: StaticSolverSettings
    """static solver parameters."""
    timeIntegration: TimeIntegrationSettings
    """time integration parameters."""
    cleanUpMemory: bool
    """True: solvers will free memory at exit (recommended for large systems); False: keep allocated memory for repeated computations to increase performance."""
    displayComputationTime: bool
    """display computation time statistics at end of solving."""
    displayGlobalTimers: bool
    """display global timer statistics at end of solving (e.g., for contact, but also for internal timings during development)."""
    displayStatistics: bool
    """display general computation information at end of time step (steps, iterations, function calls, step rejections, ..."""
    linearSolverType: LinearSolverType
    """selection of numerical linear solver: exu.LinearSolverType.EXUdense (dense matrix inverse), exu.LinearSolverType.EigenSparse (sparse matrix LU-factorization), ... (enumeration type)."""
    outputPrecision: int
    """precision for floating point numbers written to console; e.g. values written by solver."""
    pauseAfterEachStep: bool
    """pause after every time step or static load step(user press SPACE)."""

#information for VSettingsGeneral
class VSettingsGeneral:
    """General settings for visualization that influence all windows, default values, autofit, multithreading, etc."""
    autoFitScene: bool
    """automatically fit scene within startup after SC.renderer.Start()."""
    axesTiling: int
    """global number of segments for drawing cylinders for axes and cones for arrows (reduce this number, e.g. to 4, if many axes are drawn)."""
    backgroundColor: Tuple[float,float,float,float]
    """red, green, blue and alpha values for background color of render window (white=[1,1,1,1]; black = [0,0,0,1])."""
    backgroundColorBottom: Tuple[float,float,float,float]
    """red, green, blue and alpha values for bottom background color in case that useGradientBackground = True."""
    boundingBoxZoomAllFactor: float
    """factor on boundingBox for zoom all (without minimum offset)."""
    boundingBoxZoomAllOffset: float
    """minimum offset to bounding box of scene in window - width or height, whatever is smaller; adjust for very small or large scenes; may be negative."""
    circleTiling: int
    """global number of segments for circles; if smaller than 2, 2 segments are used (flat)."""
    coordinateSystemSize: float
    """size of coordinate system relative to font size."""
    cylinderTiling: int
    """global number of segments for cylinders; if smaller than 2, 2 segments are used (flat)."""
    graphicsUpdateInterval: float
    """interval of graphics update during simulation in seconds; 0.1 = 10 frames per second; low numbers might slow down computation speed."""
    limitWindowToScreenSize: bool
    """True: size for render window of respective view is limited to screen size; False: larger window sizes (e.g. for rendering) allowed according to renderWindowSize."""
    linuxDisplayScaleFactor: float
    """Scaling factor for linux, which cannot determined from system by now; adjust this value to scale dialog fonts and renderer fonts."""
    minSceneSize: float
    """minimum scene size for initial scene size and for autoFitScene, to avoid division by zero; SET GREATER THAN ZERO."""
    pointSize: float
    """global point size (absolute)."""
    reallyQuitTimeLimit: float
    """number of seconds after which user is asked a security question before stopping simulation and closing renderer; set to 0 in order to always get asked; set to 1e10 to (nearly) never get asked."""
    rendererPrecision: int
    """precision of general floating point numbers shown in render window: total number of digits used  (max. 16)."""
    rendererStartupTimeout: int
    """OpenGL render windows startup timeout in ms (change might be necessary if CPU is very slow)."""
    renderWindowString: str
    """string shown in render window (use this, e.g., for debugging, etc.; written below EXUDYN, similar to solutionInformation in SimulationSettings.solutionSettings)."""
    showHelpOnStartup: int
    """seconds to show help message on startup (0=deactivate)."""
    showSolutionInformation: bool
    """true = show solution information (from simulationSettings.solution)."""
    showSolverInformation: bool
    """true = solver name and further information shown in render window."""
    showSolverTime: bool
    """true = solver current time shown in render window."""
    sphereTiling: int
    """global number of segments for spheres; if smaller than 2, 2 segments are used (flat)."""
    textAlwaysInFront: bool
    """if true, text for item numbers and other item-related text is drawn in front; this may be unwanted in case that you only with to see numbers of objects in front; currently does not work with perspective."""
    textColor: Tuple[float,float,float,float]
    """general text color (default); used for system texts in render window."""
    textHasBackground: bool
    """if true, text for item numbers and other item-related text have a background (depending on text color), allowing for better visibility if many numbers are shown; the text itself is black; therefore, dark background colors are ignored and shown as white."""
    textOffsetFactor: float
    """This is an additional out of plane offset for item texts (node number, etc.); the factor is relative to the maximum scene size and is only used, if textAlwaysInFront=False; this factor allows to draw text, e.g., in front of nodes."""
    threadSafeGraphicsUpdate: bool
    """true = updating of visualization is threadsafe, but slower for complicated models; deactivate this to speed up computation, but activate for generation of animations; may be improved in future by adding a safe visualizationUpdate state."""
    useBitmapText: bool
    """if true, texts are displayed using pre-defined bitmaps for the text; may increase the complexity of your scene, e.g., if many (>10000) node numbers shown."""
    useGradientBackground: bool
    """true = use vertical gradient for background;."""
    useMultiThreadedRendering: bool
    """true = rendering is done in separate thread; false = no separate thread, which may be more stable but has lagging interaction for large models (do not interact with models during simulation); you MUST set this parameter BEFORE call to SC.renderer.Start(); MAC OS: uses always false, because MAC OS does not support multi threaded GLFW."""
    useWindowsDisplayScaleFactor: bool
    """the Windows display scaling (monitor scaling; content scaling) factor is used for increased visibility of texts on high resolution displays; based on GLFW glfwGetWindowContentScale; deactivated on linux compilation as it leads to crashes (adjust textSize manually!)."""
    zoomAllUseBoundingBox: bool
    """if true, use exact scene bounding box (but not including texts) for zoom; does not include perspective effects!"""

#information for VSettingsContourAdvanced
class VSettingsContourAdvanced:
    """Advanced settings for contour plots."""
    colorBarPrecision: int
    """precision of floating point values shown in color bar; total number of digits used (max. 16)."""
    colorBarTiling: int
    """number of tiles (segements) shown in the colorbar for the contour plot."""
    contourColor0: Tuple[float,float,float,float]
    """RGBA color for relative value 0 used for contour plot; alpha is ignored."""
    contourColor1: Tuple[float,float,float,float]
    """RGBA color for relative value 0.25 used for contour plot; alpha is ignored."""
    contourColor2: Tuple[float,float,float,float]
    """RGBA color for relative value 0.25 used for contour plot; alpha is ignored."""
    contourColor3: Tuple[float,float,float,float]
    """RGBA color for relative value 0.25 used for contour plot; alpha is ignored."""
    contourColor4: Tuple[float,float,float,float]
    """RGBA color for relative value 0.25 used for contour plot; alpha is ignored."""
    contourColorMax: Tuple[float,float,float,float]
    """RGBA color if relative value in contour plot is larger than 1 (if automaticRange=False); alpha is ignored."""
    contourColorMin: Tuple[float,float,float,float]
    """RGBA color if relative value in contour plot is smaller than 0 (if automaticRange=False); alpha is ignored."""
    showColorBar: bool
    """show the colour bar with minimum and maximum values for the contour plot."""

#information for VSettingsContour
class VSettingsContour:
    """Settings for contour plots; use these options to visualize field data, such as displacements, stresses, strains, etc. for bodies, nodes and finite elements."""
    advanced: VSettingsContourAdvanced
    """advanced settings for contour."""
    alphaTransparency: float
    """default value for contour alpha transparency (RGB color computed from contour value)."""
    automaticRange: bool
    """if true, the contour plot value range is chosen automatically to the maximum range."""
    maxValue: float
    """maximum value for contour plot; set manually, if automaticRange == False."""
    minValue: float
    """minimum value for contour plot; set manually, if automaticRange == False."""
    nodesColored: bool
    """if true, the contour color is also applied to nodes (except mesh nodes), otherwise node drawing is not influenced by contour settings."""
    outputVariable: OutputVariableType
    """selected contour plot output variable type; select OutputVariableType._None to deactivate contour plotting."""
    outputVariableComponent: int
    """select the component of the chosen output variable; e.g., for displacements, 3 components are available: 0 == x, 1 == y, 2 == z component; for stresses, 6 components are available, see OutputVariableType description; to draw the norm of a outputVariable, set component to -1; if a certain component is not available by certain objects or nodes, no value is drawn (using default color)."""
    reduceRange: bool
    """if true, the contour plot value range is also reduced; better for static computation; in dynamic computation set this option to false, it can reduce visualization artifacts; you should also set minVal to max(float) and maxVal to min(float)."""
    rigidBodiesColored: bool
    """if true, the contour color is also applied to triangular faces of rigid bodies and mass points, otherwise the rigid body drawing are not influenced by contour settings; for general rigid bodies (except for ObjectGround), Position, Displacement, DisplacementLocal(=0), Velocity, VelocityLocal, AngularVelocity, and AngularVelocityLocal are available; may slow down visualization!"""

#information for VSettingsNodes
class VSettingsNodes:
    """Visualization settings for nodes."""
    basisSize: float
    """size of basis for nodes."""
    defaultColor: Tuple[float,float,float,float]
    """default RGBA color for nodes; 4th value is alpha-transparency."""
    defaultSize: float
    """global node size; if -1.f, node size is relative to openGL.initialMaxSceneSize."""
    drawNodesAsPoint: bool
    """simplified/faster drawing of nodes; uses general->pointSize as drawing size; if drawNodesAsPoint==True, the basis of the node will be drawn with lines."""
    show: bool
    """flag to decide, whether the nodes are shown."""
    showBasis: bool
    """show basis (three axes) of coordinate system in 3D nodes."""
    showNodalSlopes: int
    """draw nodal slope vectors, e.g. in ANCF beam finite elements."""
    showNumbers: bool
    """flag to decide, whether the node number is shown."""
    tiling: int
    """tiling for node if drawn as sphere; used to lower the amount of triangles to draw each node; if drawn as circle, this value is multiplied with 4."""

#information for VSettingsBeams
class VSettingsBeams:
    """Visualization settings for beam finite elements."""
    axialTiling: int
    """number of segments to discretise the beams axis."""
    crossSectionFilled: bool
    """if implemented for element, cross section is drawn as solid (filled) instead of wire-frame; NOTE: some quantities may not be interpolated correctly over cross section in visualization; equivalent to drawSolid of shells."""
    crossSectionTiling: int
    """number of quads drawn over height of beam, if drawn as flat objects; leads to higher accuracy of components drawn over beam height or with, but also to larger CPU costs for drawing."""
    drawVertical: bool
    """draw contour plot outputVariables 'vertical' along beam height; contour.outputVariable must be set accordingly."""
    drawVerticalColor: Tuple[float,float,float,float]
    """color for outputVariable to be drawn along cross section (vertically)."""
    drawVerticalFactor: float
    """factor for outputVariable to be drawn along cross section (vertically)."""
    drawVerticalLines: bool
    """draw additional vertical lines for better visibility."""
    drawVerticalOffset: float
    """offset for vertical drawn lines; offset is added before multiplication with drawVerticalFactor."""
    drawVerticalValues: bool
    """show values at vertical lines; note that these numbers are interpolated values and may be different from values evaluated directly at this point!"""
    reducedAxialInterploation: bool
    """if True, the interpolation along the beam axis may be lower than the beam element order; this may, however, show more consistent values than a full interpolation, e.g. for strains or forces."""

#information for VSettingsShells
class VSettingsShells:
    """Visualization settings for plate/shell finite elements."""
    drawSolid: bool
    """if true: to draw plates/shells as 3D objects; false: only the element surface is drawn; equivalent to crossSectionFilled in beams."""
    thicknessFactor: float
    """a factor multiplied with the thickness of shells/plates only for visualization (e.g. to make some effects more visible)."""

#information for VSettingsKinematicTree
class VSettingsKinematicTree:
    """Visualization settings for kinematic trees."""
    frameSize: float
    """size of COM and joint frames."""
    showCOMframes: bool
    """if True, a frame is attached to every center of mass."""
    showFramesNumbers: bool
    """if True, numbers are drawn for joint frames (O[i]J[j]) and COM frames (O[i]COM[j]) for object [i] and local joint [j]."""
    showJointFrames: bool
    """if True, a frame is attached to the origin of every joint frame."""

#information for VSettingsBodies
class VSettingsBodies:
    """Visualization settings for bodies."""
    beams: VSettingsBeams
    """visualization settings for beams (e.g. ANCFCable or other beam elements)."""
    kinematicTree: VSettingsKinematicTree
    """visualization settings for kinematic tree."""
    shells: VSettingsShells
    """visualization settings for plates and shells."""
    defaultColor: Tuple[float,float,float,float]
    """default RGBA color for bodies; 4th value is alpha-transparency."""
    defaultSize: Tuple[float,float,float]
    """global body size of xyz-cube."""
    deformationScaleFactor: float
    """global deformation scale factor; also applies to nodes, if drawn; currently only used for scaled drawing of (linear) finite elements in FFRF and FFRFreducedOrder objects."""
    show: bool
    """flag to decide, whether the bodies are shown."""
    showNumbers: bool
    """flag to decide, whether the body(=object) number is shown."""

#information for VSettingsConnectors
class VSettingsConnectors:
    """Visualization settings for connectors."""
    contactPointsDefaultSize: float
    """DEPRECATED: do not use! global contact points size; if -1.f, connector size is relative to maxSceneSize."""
    defaultColor: Tuple[float,float,float,float]
    """default RGBA color for connectors; 4th value is alpha-transparency."""
    defaultSize: float
    """global connector size; if -1.f, connector size is relative to maxSceneSize."""
    jointAxesLength: float
    """global joint axes length."""
    jointAxesRadius: float
    """global joint axes radius."""
    show: bool
    """flag to decide, whether the connectors are shown."""
    showContact: bool
    """flag to decide, whether contact points, lines, etc. are shown for special cable-circle contacts; for spheres, triangles, tori, see visualizationSettings.contact."""
    showJointAxes: bool
    """flag to decide, whether contact joint axes of 3D joints are shown."""
    showNumbers: bool
    """flag to decide, whether the connector(=object) number is shown."""
    springNumberOfWindings: int
    """number of windings for springs drawn as helical spring."""

#information for VSettingsMarkers
class VSettingsMarkers:
    """Visualization settings for markers."""
    defaultColor: Tuple[float,float,float,float]
    """default RGBA color for markers; 4th value is alpha-transparency."""
    defaultSize: float
    """global marker size; if -1.f, marker size is relative to maxSceneSize."""
    drawSimplified: bool
    """draw markers with simplified symbols."""
    show: bool
    """flag to decide, whether the markers are shown."""
    showNumbers: bool
    """flag to decide, whether the marker numbers are shown."""

#information for VSettingsLoads
class VSettingsLoads:
    """Visualization settings for loads."""
    defaultColor: Tuple[float,float,float,float]
    """default RGBA color for loads; 4th value is alpha-transparency."""
    defaultRadius: float
    """global radius of load axis if drawn in 3D."""
    defaultSize: float
    """global load size; if -1.f, load size is relative to maxSceneSize."""
    drawSimplified: bool
    """draw markers with simplified symbols."""
    drawWithUserFunction: bool
    """draw loads like force vectors time dependent; make sure that fixedLoadSize=false, while otherwise only the direction will change; user functions can only be drawn, if they are either symbolic or for Python user functions if useMultiThreadedRendering=False."""
    fixedLoadSize: bool
    """if true, the load is drawn with a fixed vector length in direction of the load vector, independently of the load size."""
    loadSizeFactor: float
    """if fixedLoadSize=false, then this scaling factor is used to draw the load vector."""
    show: bool
    """flag to decide, whether the loads are shown."""
    showNumbers: bool
    """flag to decide, whether the load numbers are shown."""

#information for VSettingsTraces
class VSettingsTraces:
    """Visualization settings for traces of sensors. Note that a large number of time points (influenced by simulationSettings.solutionSettings.sensorsWritePeriod) may lead to slow graphics."""
    lineWidth: float
    """line width for traces."""
    listOfPositionSensors: List[int]
    """list of position sensors which can be shown as trace inside render window if sensors have storeInternal=True; if this list is empty and showPositionTrace=True, then all available sensors are shown."""
    listOfTriadSensors: List[int]
    """list of sensors of with OutputVariableType RotationMatrix; this non-empty list needs to coincide in length with the listOfPositionSensors to be shown if showTriads=True; the triad is drawn at the related position."""
    listOfVectorSensors: List[int]
    """list of sensors with 3D vector quantities; this non-empty list needs to coincide in length with the listOfPositionSensors to be shown if showVectors=True; the vector quantity is drawn relative to the related position."""
    positionsShowEvery: int
    """integer value i; out of available sensor data, show every i-th position."""
    sensorsMbsNumber: int
    """number of main system which is used to for sensor lists; if only 1 mbs is in the SystemContainer, use 0; if there are several mbs, it needs to specify the number."""
    showCurrent: bool
    """show current trace position (and especially vector quantity) related to current visualization state; this only works in solution viewer if sensor values are stored at time grid points of the solution file (up to a precision of 1e-10) and may therefore be temporarily unavailable."""
    showFuture: bool
    """show trace future to current visualization state if already computed (e.g. in SolutionViewer)."""
    showPast: bool
    """show trace previous to current visualization state."""
    showPositionTrace: bool
    """show position trace of all position sensors if listOfPositionSensors=[] or of specified sensors; sensors need to activate storeInternal=True."""
    showTriads: bool
    """if True, show basis vectors from rotation matrices provided by sensors."""
    showVectors: bool
    """if True, show vector quantities according to description in showPositionTrace."""
    timeSpan: float
    """maximum trace time span of past or future trace; given in seconds of simulation time; if zero, it is unused."""
    traceColors: List[float]
    """RGBA float values for traces in one array; using 6x4 values gives different colors for 6 traces; in case of triads, the 0/1/2-axes are drawn in red, green, and blue."""
    triadSize: float
    """length of triad axes if shown."""
    triadsShowEvery: int
    """integer value i; out of available sensor data, show every i-th triad."""
    vectorScaling: float
    """scaling of vector quantities; if, e.g., loads, this factor has to be adjusted significantly."""
    vectorsShowEvery: int
    """integer value i; out of available sensor data, show every i-th vector."""

#information for VSettingsSensors
class VSettingsSensors:
    """Visualization settings for sensors."""
    traces: VSettingsTraces
    """settings for showing (position/triad) sensor traces and vector plots in the render window."""
    defaultColor: Tuple[float,float,float,float]
    """default RGBA color for sensors; 4th value is alpha-transparency."""
    defaultSize: float
    """global sensor size; if -1.f, sensor size is relative to maxSceneSize."""
    drawSimplified: bool
    """draw sensors with simplified symbols."""
    show: bool
    """flag to decide, whether the sensors are shown."""
    showNumbers: bool
    """flag to decide, whether the sensor numbers are shown."""

#information for VSettingsContact
class VSettingsContact:
    """Global visualization settings for GeneralContact. This allows to easily switch on/off during visualization; also used for contact objects, such as ObjectContactSphereSphere or ObjectContactSphereTriangle."""
    colorBoundingBoxes: Tuple[float,float,float,float]
    """RGBA color for boudnding boxes, see showBoundingBoxes."""
    colorSearchTree: Tuple[float,float,float,float]
    """RGBA color for search tree, see showSearchTree."""
    colorSpheres: Tuple[float,float,float,float]
    """RGBA color for contact spheres, see showSpheres."""
    colorTori: Tuple[float,float,float,float]
    """RGBA color for contact tori, see showTori."""
    colorTriangles: Tuple[float,float,float,float]
    """RGBA color for contact triangles, see showTriangles."""
    contactForcesFactor: float
    """factor used for scaling of contact forces is showContactForces=True."""
    contactPointsDefaultSize: float
    """global contact points size; if -1.f, connector size is relative to maxSceneSize; used for some contacts, e.g., in ContactFrictionCircle."""
    showBoundingBoxes: bool
    """show computed bounding boxes of all GeneralContacts; Warning: avoid for large number of contact objects!"""
    showContactForces: bool
    """if True, contact forces are drawn for certain contact models."""
    showContactForcesValues: bool
    """if True and showContactForces=True, numerical values for  contact forces are shown at certain points."""
    showSearchTree: bool
    """show outer box of search tree for all GeneralContacts."""
    showSearchTreeCells: bool
    """show all cells of search tree; empty cells have colorSearchTree, cells with contact objects have higher red value; Warning: avoid for large number of search tree cells!"""
    showSpheres: bool
    """show contact spheres (SpheresWithMarker, ...)."""
    showTori: bool
    """show each contact torus."""
    showTriangles: bool
    """show contact triangles (TrianglesRigidBodyBased, ...)."""
    tilingCurves: int
    """tiling for nonlinear/polynomial curves; higher values give smoother curves."""
    tilingSpheres: int
    """tiling for spheres; higher values give smoother spheres, but may lead to lower frame rates."""

#information for VSettingsCamera
class VSettingsCamera:
    """Settings for camera like perspective, marker tracking, clipping plane, etc. Note that some options may also be found in openGL settings."""
    cameraPosition: Tuple[float,float,float]
    """if modelCentricView=True: offset to camera position in model view (and, if used, relative to tracked marker - instead of a tracked marker position, you could also just change the camera position in camera-centric views); camera rotation follows modelRotation in renderState."""
    clippingPlaneDistance: float
    """distance of clipping plane on normal vector; see also clippingPlaneNormal and openGL.advanced.clippingPlaneColor."""
    clippingPlaneNormal: Tuple[float,float,float]
    """normal vector of clipping plane, e.g. [0,0,1] to set a xy-clipping plane; the clipped half-space is in direction of the normal; use [0,0,0] to deactivate clipping plane; Note that clipping is mainly made for triangles in order to visualize hidden objects and currently it only fully clips triangles, but does not exactly cut them; see also clippingPlaneDistance and openGL.advanced.clippingPlaneColor."""
    modelCentricView: bool
    """True: rotations and translations are applied to model, while camera stays far enough away from the model and always captures the whole model (everything is in front of camera plane); False: camera moves and rotates while model stays in physical space; only geometry in front of camera is visible; note that the behavior of trackMarker changes with modelCentricView and some features are not available in case of modelCentricView=False."""
    nearFarPlaneOffset: Tuple[float,float,float]
    """the three values are [nearPlaneOffset, farPlaneOffset, flag]; if flag=0, the offsets are ignored and computed automatically, using x = 2 * maxSceneSize * zMaxSceneFactor, setting near plane to -x and far plane to +x in case of modelCentricView=True and setting near plane to 0.01 (minimal offset to eye point) and far plane to +x if modelCentricView=False; if flag=1, the near and far plane values are just overwritten; note that positive values for near plane make objects in front of the camera invisible while negative values make objects behind the camera plane visible; in case of camera-centric view, the eyepoint can be shifted backwards using cameraPosition accordingly."""
    perspective: float
    """parameter prescribes amount of perspective (0=no perspective=orthographic projection; positive values increase perspective; feasible values are 0.001 (little perspective) ... 1 (extreme: 5), where larger values are possible but should be used with care; NOTE that the relation to the common field of view (FOV) angle alpha, with alpha=90°, is given by perspective = tan(alpha/2) = 1; mouse coordinates (F3) can not be shown with perspective>0."""
    trackMarker: int
    """if valid marker index is provided and marker provides position (and orientation), the centerpoint of the scene follows the marker (and orientation); depends on trackMarkerPosition and trackMarkerOrientation; by default, only position is tracked."""
    trackMarkerMbsNumber: int
    """number of main system which is used to track marker; if only 1 mbs is in the SystemContainer, use 0; if there are several mbs, it needs to specify the number."""
    trackMarkerOrientation: Tuple[float,float,float]
    """choose which orientation axes (x,y,z) are tracked; currently can only be all zero or all one."""
    trackMarkerPosition: Tuple[float,float,float]
    """choose which coordinates or marker are tracked (x,y,z)."""
    useRaytracer: bool
    """True: use (software) raytracer for this view; False: use standard OpenGL renderer."""

#information for VSettingsScene
class VSettingsScene:
    """Settings change scene representation (show edges, show faces, global transparency), adding world basis, etc., in particular settings that are individual to each view. Note that some scene settings that are global to all views may be found in general and in openGL settings."""
    drawCoordinateSystem: int
    """0 = no coordinate system shown, 1 = draw lines with text, 2 = draw arrows, 3 = draw arrows with text."""
    drawWorldBasis: bool
    """true = draw world basis coordinate system at (0,0,0)."""
    facesTransparent: bool
    """True: show faces transparent independent of transparency (A)-value in color of objects; allow to show otherwise hidden node/marker/object numbers."""
    showFaceEdges: bool
    """True: show edges of triangles; using the options showFaces=false and showFaceEdges=true gives are wire frame representation."""
    showFaces: bool
    """True: show faces of triangles, etc.; using the options showFaces=false and showFaceEdges=true gives are wireframe representation."""
    showLines: bool
    """True: show lines (other lines than face and mesh edges)."""
    showMeshEdges: bool
    """True: show edges of finite elements; independent of showFaceEdges."""
    showMeshFaces: bool
    """True: show faces of finite elements; independent of showFaces."""
    worldBasisSize: float
    """size of world basis coordinate system."""

#information for VSettingsWindow
class VSettingsWindow:
    """Settings for window that are individual to each view; in particular initial size, and behavior. Note that some of the settings are only used during creation of the window."""
    alwaysOnTop: bool
    """True: render window of respective view will be always on top of all other windows."""
    globalFontSize: float
    """general text font size (roughly measured in pixels); if useWindowsDisplayScaleFactor=True, the the textSize is multplied with the windows display scaling (monitor scaling; content scaling) factor for larger texts on on high resolution displays; for bitmap fonts, the maximum size of any font (standard/large/huge) is limited to 256 (which is not recommended, especially if you do not have a powerful graphics card)."""
    lockModelView: bool
    """True: all movements (with mouse/keys), rotations, zoom are disabled; the view is either based on initial values (or on the current state) ==> initial zoom, rotation and center point need to be adjusted, approx. 0.4*maxSceneSize is a good value."""
    maximize: bool
    """True: render window of respective view will be maximized at startup."""
    renderWindowSize: Tuple[int,int]
    """initial size of render window of respective view for specific view in pixels for."""
    showComputationInfo: bool
    """true = show (hide) all computation information including Exudyn and version."""
    showMouseCoordinates: bool
    """True: show OpenGL coordinates and distance to last left mouse button pressed position in renderer status message; switched on/off with key 'F3'; only works for axis-aligned ortho-projections."""
    showRenderStateInfo: bool
    """True: show renderer.state infos regarding zoom, offset and rotation in renderer status message; switched on/off with 'CTRL-F3'."""
    showWindow: bool
    """True: render window of respective view is shown when created; False: window will be iconified when created (e.g. if you are starting multiple computations automatically)."""

#information for VSettingsView
class VSettingsView:
    """Settings for view including camera, scene, window, and advanced options to setup a view or view window."""
    camera: VSettingsCamera
    """settings for camera like perspective, marker tracking or clipping plane."""
    scene: VSettingsScene
    """settings which change scene representation, showing edges, faces or world basis."""
    window: VSettingsWindow
    """visualization settings for window that are individual to each view."""

#information for VSettingsWindowDeprecated
class VSettingsWindowDeprecated:
    """OpenGL Window and interaction settings for visualization; handle changes with care, as they might lead to unexpected results or crashes."""

#information for VSettingsDialogs
class VSettingsDialogs:
    """Settings related to dialogs (e.g., visualization settings dialog)."""
    alphaTransparency: float
    """alpha-transparency of dialogs; recommended range 0.7 (very transparent) - 1 (not transparent at all)."""
    alwaysTopmost: bool
    """True: dialogs are always topmost (otherwise, they are sometimes hidden)."""
    fontScalingMacOS: float
    """font scaling value for MacOS systems (on Windows, system display scaling is used)."""
    multiThreadedDialogs: bool
    """True: During dialogs, the OpenGL render windows will still get updates of changes in dialogs, etc., which may cause problems on some platforms or for some (complicated) models; False: changes of dialogs will take effect when dialogs are closed."""
    openTreeView: bool
    """True: all sub-trees of the visusalization dialog are opened when opening the dialog; False: only some sub-trees are opened."""

#information for VSettingsMaterial
class VSettingsMaterial:
    """Settings for rendering materials, in particular for the Raytracer (may be available also in the OpenGL renderer in the future). This material (widely follows Phong model) can be either accessed via SC.renderer.materials or directly in visualizationSettings.raytracer.material0, material1, etc.; note that the default values shown in the documentation only reflect material0 but not all 10 default materials."""
    alpha: float
    """alpha-transparency, same as in alpha channel in RGBA colors; 1=opaque, 0=fully transparent; leads to extra rendering costs per transparent pixel."""
    baseColor: Tuple[float,float,float]
    """RGB default material color if face color has R-color channel -1."""
    emission: Tuple[float,float,float]
    """RGB emissive material color (enlightened material)."""
    ior: float
    """index of refraction for transparent materials (1=no refraction), >1 represents refraction."""
    name: str
    """material name for easier handling."""
    reflectivity: float
    """controls reflectivity of material; 0=no reflections (rough, e.g. rubber), 1=fully reflective (mirror); this leads to large extra rendering costs per visible reflective pixel."""
    shininess: float
    """controls shininess of specular component of lights; values < 5 is not very shiny, while > 50 is very shiny."""
    specular: Tuple[float,float,float]
    """RGB specular material color."""

#information for VSettingsRaytracerAdvanced
class VSettingsRaytracerAdvanced:
    """Advanced settings for raytracer."""
    backgroundColorReflections: Tuple[float,float,float,float]
    """scene RGBA color for background that is hit by reflection material; while openGL.backgroundColor is used for rays that do not hit an object, this background may - if black or white - not be a suitable color for computing reflections; this is generally needed, as our scenes are usually not inside a closed geometry (like inside a room); this color is also used if maxReflectionDepth is reached."""
    searchTreeFactor: int
    """This factor can be used to increase the number of search tree bins, which can improve performance in case of inequilibrated scense; range=1..128."""
    shadowScalingFactor: int
    """if lightRadiusVariations>1, this defines the downscaling factor of the shadow map, where 2 means that the resolution is 2 times smaller than the image resolution; additionally, multisampling is not used for shadow map computation if shadowScalingFactor>0, thus reducing the computational effort for shadow computation also in case of 1; range=0..16; larger values cause significant artifacts at shadow boundaries."""
    shadowSmoothingSteps: int
    """if lightRadiusVariations>1, this defines the number of smoothing steps at the low-resolution shadow map; smoothing reduces shadow artifacts caused by smaller values of lightRadiusVariations; range=0..32; smoothing  steps may cause artifacts at shadow boundaries; only works for directional lights with position (e.g. 4th component in light0Position should be 1)."""
    showText: bool
    """True: show any kind of status text, node numbers, object numbers, etc. (depending on settings); False: do not show any text in raytracer, independently of settings."""
    tilesPerThread: int
    """Total number of sub-tiles per thread, used to evenly distribute rendering load to threads."""
    zBiasLines: float
    """offset for lines to draw in front of faces; relative to scene radius."""

#information for VSettingsRaytracer
class VSettingsRaytracer:
    """Settings for raytracer (software renderer) which can be used as alternative to classic OpenGL rendering; this option may be erased in future in favor of a modern GPU rendering. To activate the raytracer, simply switch the enable flag to True. The raytracer uses CPU-based rendering and is therefore comparably slow (may take seconds to render one frame). Thus, take care with the window dimension (start with small window size like 400 x 300) and use openGL.multiSampling=1. Note that many parameters are used from openGL settings, like backgroundColor, lineWidth, multiSampling, shadow (only on/off), and lights. See the options to improve appearance and performance."""
    advanced: VSettingsRaytracerAdvanced
    """advanced settings for raytracer."""
    material0: VSettingsMaterial
    """settings for material0."""
    material1: VSettingsMaterial
    """settings for material1."""
    material2: VSettingsMaterial
    """settings for material2."""
    material3: VSettingsMaterial
    """settings for material3."""
    material4: VSettingsMaterial
    """settings for material4."""
    material5: VSettingsMaterial
    """settings for material5."""
    material6: VSettingsMaterial
    """settings for material6."""
    material7: VSettingsMaterial
    """settings for material7."""
    material8: VSettingsMaterial
    """settings for material8."""
    material9: VSettingsMaterial
    """settings for material9."""
    globalFogColor: Tuple[float,float,float,float]
    """scene RGBA fog color."""
    globalFogDensity: float
    """global fog density; fog is deactivated if fogDensity=0, otherwise it is a density relative to scene max size; as it is relative, the factor has to be relatively high to be visible (usually >1)."""
    imageSizeFactor: int
    """Special size factor (1-16) to allow drawing with smaller resolution (faster); use this for long rendering times for adjustments, etc."""
    keepWindowActive: bool
    """Special flag, handle with care; True: sends some glfw functions to keep window reactive for long render times (>2 seconds); otherwise, the rendering may not finish due to timeout."""
    lightRadiusVariations: int
    """if lightRadiusVariations>1, this defines the number of positions that are used to compute the effect of distributed lights (larger is slower but better quality); range=1..256; avoid squares of integers; good values: 1 (hard shadow boundaries), 6, 13, 20, 31, 72, 130, 240; for lower values, use shadowSmoothingSteps=2..8."""
    maxReflectionDepth: int
    """Maximum number of reflections computed for one ray (note that for each transparent face passed, the reflection depth is reduced by 1); maximum is 32 (but should not be more than 2-4 usually!)."""
    maxTransparencyDepth: int
    """Maximum number of transparent faces that can be passed (note that for each reflection, the transparency depth is reduced by 1); maximum is 32 (but should not be more than 2-4 usually!)."""
    multiSampling: int
    """Multi-sampling used for rendering of faces, lines and text; increases image quality along edges (lines, etc.) but INCREASES rendering costs dramatically (multiSampling=3 => 3x3=9 times slower); also used for shadow if shadowScalingFactor=0; values only accepted in range [1..4]."""
    numberOfThreads: int
    """Number of CPU-threads (max: 256) used for software rendering (should be approx. the number of available threads)."""
    verbose: int
    """1: print out some debug information on rendering, in particular rendering timings and counter; 2 and higher: advanced debug information."""

#information for VSettingsOpenGLAdvanced
class VSettingsOpenGLAdvanced:
    """Advanced settings for openGL."""
    clippingPlaneColor: Tuple[float,float,float,float]
    """RGBA color for clipping plane; if alpha-channel is 0, the cutting plane is not drawn; if alpha-channel is 1, the clippingPlaneColor is used; if alpha-channel is 2, the color of the object interior is used as clipping plane color (which may look strange in case of object-in-object); see also view.camera for clipping plane options."""
    depthSorting: bool
    """True (slower): sort triangles by Z-depth to remove transparency artifacts: only works if triangles do not intersect or come close (you may like to refine triangle meshes); False: no depth-sort (faster)."""
    enableLighting: bool
    """generally enable lighting (otherwise, colors of objects are used); OpenGL: glEnable(GL_LIGHTING)."""
    faceNormalsColor: Tuple[float,float,float,float]
    """global RGBA color for face normals."""
    initialCenterPoint: Tuple[float,float,float]
    """centerpoint of scene (3D) at renderer startup; overwritten if autoFitScene = True; only used in case that modelCentricView=True."""
    initialMaxSceneSize: float
    """initial maximum scene size (auto: diagonal of cube with maximum scene coordinates); used for 'zoom all' functionality and for visibility of objects; overwritten if autoFitScene = True."""
    initialModelRotation: ArrayLike
    """initial model rotation matrix for OpenGl; in python use e.g.: initialModelRotation=[[1,0,0],[0,1,0],[0,0,1]]; only used in case that modelCentricView=True."""
    initialZoom: float
    """initial zoom of scene; overwritten/ignored if autoFitScene = True."""
    lightModelLocalViewer: bool
    """True: the camera origin is used to compute shininess effects (more realistic); maps to OpenGL glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,...)."""
    lightModelTwoSide: bool
    """enlighten also backside of object; may cause problems on some graphics cards and lead to slower performance; maps to OpenGL glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,...)."""
    lineSmooth: bool
    """draw lines smooth."""
    polygonOffset: float
    """general polygon offset for polygons, except for shadows; use this parameter to draw polygons behind lines to reduce artifacts for very large or small models."""
    shadeModelSmooth: bool
    """True: turn on smoothing for shaders, which uses vertex normals to smooth surfaces."""
    shadowPolygonOffset: float
    """some special drawing parameter for shadows which should be handled with care; defines some offset needed by openGL to avoid aritfacts for shadows and depends on maxSceneSize; this value may need to be reduced for larger models in order to achieve more accurate shadows, it may be needed to be increased for thin bodies."""
    showBoundingBox: bool
    """show scene bounding box (red), as available in renderState.boundingBox; NOTE that the bounding box is only updated with ZoomAll or at startup; this is a debug flag and it may show reasongs for strange ZoomAll behavior, as ZoomAll should zoom to the bounding box; does only work for perspective=0."""
    textLineSmooth: bool
    """draw lines for representation of text smooth."""
    textLineWidth: float
    """width of lines used for representation of text."""
    vertexNormalsColor: Tuple[float,float,float,float]
    """global RGBA color for vertex normals."""

#information for VSettingsLight
class VSettingsLight:
    """Settings for lights."""
    constantAttenuation: float
    """constant attenuation coefficient of GL_LIGHT[0,1,2,3], this is a constant factor that attenuates the light source; attenuation factor = 1/(kc +kl*d + kq*d*d); (kc,kl,kq)=(1,0,0) means no attenuation; only used for lights, where last component of light position is 1."""
    diffuse: float
    """diffuse value of GL_LIGHT[0,1,2,3]."""
    enable: bool
    """turn on/off light."""
    lightRadius: float
    """only used by raytracers: radius of light used to compute smooth shadows (approximated by raytracer.lightRadiusVariations); if lightRadiusVariations>1, this value defines the radius of the light, converting point lights into distributed lights (slower)."""
    linearAttenuation: float
    """linear attenuation coefficient of GL_LIGHT[0,1,2,3], this is a linear factor for attenuation of the light source with distance."""
    position: Tuple[float,float,float,float]
    """4D position vector of GL_LIGHT[0,1,2,3]; 4th value should be 0 for directional lights that are (almost) infinitely far away, like the sun, but 1 for position-based lights (and for attenuation factor being calculated); light0 is also used for shadows, so you need to adjust this position to be located at a reasonable location; the openGL renderer uses shadow volumes and approximates directional lights by enlarging the direction to 200 times maxSceneSize, while the raytracer uses the correct direction; see opengl manuals."""
    quadraticAttenuation: float
    """quadratic attenuation coefficient of GL_LIGHT[0,1,2,3], this is a quadratic factor for attenuation of the light source with distance."""
    shadow: float
    r"""in OpenGL renderer, the shadow parameter :math:`\in [0 ... 1]` prescribes amount of shadow of light [0,1,2,3] that is added to the scene, using light position (or only direction), accumulating for each light; if this parameter is different from 0, rendering of triangles becomes approx. 5 times more expensive, so take care in case of complex scenes; for complex object, such as spheres with fine resolution or for particle systems, the present approach has limitations and leads to artifacts and unrealistic shadows; for raytracer, shadow is included by a physics-based model for each light if shadow>0, accumulating effects of each light source."""
    specular: float
    """specular value of GL_LIGHT[0,1,2,3]."""
    useCameraFrame: bool
    """set False to set light positions and directions relative to model frame; True: lights are in camera frame, not following the visual transformations; this was True up to Exudyn 1.9.174."""

#information for VSettingsOpenGL
class VSettingsOpenGL:
    """OpenGL settings for 2D and 3D rendering - with many settings also used for raytracer. For further details and backgrounds also see OpenGL 1.3 functionality on the web."""
    advanced: VSettingsOpenGLAdvanced
    """advanced settings for openGL."""
    light0: VSettingsLight
    """settings for light0 and shadow."""
    light1: VSettingsLight
    """settings for light1 and shadow."""
    light2: VSettingsLight
    """settings for light2 and shadow."""
    light3: VSettingsLight
    """settings for light3 and shadow."""
    drawFaceNormals: bool
    """draws triangle normals, e.g. at center of triangles; used for debugging of faces."""
    drawNormalsLength: float
    """length of normals; used for debugging."""
    drawVertexNormals: bool
    """draws vertex normals; used for debugging."""
    faceEdgesColor: Tuple[float,float,float,float]
    """global RGBA color for face edges."""
    faceTransparencyGlobal: float
    """in case that facesTransparent=True this represents the max alpha-transparency."""
    lightModelAmbient: Tuple[float,float,float,float]
    """global ambient light (needed for faces that are close to orthogonal to light or faces in shadow region); maps to OpenGL glLightModeli(GL_LIGHT_MODEL_AMBIENT,[r,g,b,a]); also used by raytracer."""
    lineWidth: float
    """width of lines used for representation of lines, circles, points, etc."""
    materialShininess: float
    """shininess of material."""
    materialSpecular: Tuple[float,float,float,float]
    """RGBA specular color of material."""
    multiSampling: int
    """NOTE: this parameter must be set before starting renderer; later changes are not affecting visualization; multi sampling turned off (<=1) or turned on to given values (2, 3, 4, 8 or 16); increases the graphics buffers and might crash due to graphics card memory limitations; only works if supported by hardware; if it does not work, try to change 3D graphics hardware settings!"""
    zMaxSceneFactor: float
    """factor multiplied with maxSceneSize to avoid clipping of modelview; larger values reduce clipping of near or far objects, but may lead to artifacts (so-called Z-fighting)."""

#information for VSettingsExportImages
class VSettingsExportImages:
    """Functionality to export images of view0 to files (PNG or TGA format) which can be used to create animations; in order to activate image recording during the solution process, set SolutionSettings.recordImagesInterval accordingly."""
    heightAlignment: int
    """alignment of exported image height; using a value of 2 helps to reduce problems with video conversion (additional horizontal lines are lost)."""
    saveImageAsTextCircles: bool
    """export circles in save image (only in TXT format)."""
    saveImageAsTextLines: bool
    """export lines in save image (only in TXT format)."""
    saveImageAsTextTexts: bool
    """export text in save image (only in TXT format)."""
    saveImageAsTextTriangles: bool
    """export triangles in save image (only in TXT format)."""
    saveImageFileCounter: int
    """current value of the counter which is used to consecutively save frames (images) with consecutive numbers."""
    saveImageFileName: str
    """filename (without extension!) and (relative) path for image file(s) with consecutive numbering (e.g., frame0000.png, frame0001.png,...); ; directory will be created if it does not exist."""
    saveImageFormat: str
    """format for exporting figures: currently only PNG, TGA and TXT available; while PNG and TGA represent the according image file formats, the TXT format results in a text file containing the 3D graphics data information as lists of lines, triangles, etc; PNG is not available for Ubuntu18.04 (check  use TGA has highest compatibility with all platforms."""
    saveImageSingleFile: bool
    """True: only save single files with given filename, not adding numbering; False: add numbering to files, see saveImageFileName."""
    saveImageTimeOut: int
    """timeout in milliseconds for saving a frame as image to disk; this is the amount of time waited for redrawing; increase for very complex scenes."""
    widthAlignment: int
    """alignment of exported image width; using a value of 4 helps to reduce problems with video conversion (additional vertical lines are lost)."""

#information for VSettingsOpenVR
class VSettingsOpenVR:
    """Functionality to interact openVR; requires special hardware or software emulator, see steam / openVR descriptions."""
    actionManifestFileName: str
    """This string must contain a string representing a valid absolute path to a vr_actions.json manifest, which describes all HMD, tracker, etc. devices as given by openVR."""
    enable: bool
    """True: openVR enabled (if compiled with according flag and installed openVR)."""
    logLevel: int
    """integer value setting log level of openVR: -1 (no output), 0 (error), 1 (warning), 2 (info), 3 (debug); increase log level to get more output."""
    showCompanionWindow: bool
    """True: openVR will show companion window containing left and right eye view."""

#information for VSettingsInteractiveAdvanced
class VSettingsInteractiveAdvanced:
    """Advanced settings for interactive."""
    highlightColor: Tuple[float,float,float,float]
    """RGBA color for highlighted item; 4th value is alpha-transparency."""
    highlightOtherColor: Tuple[float,float,float,float]
    """RGBA color for other items (which are not highlighted); 4th value is alpha-transparency."""
    joystickScaleRotation: float
    """rotation scaling factor for joystick input."""
    joystickScaleTranslation: float
    """translation scaling factor for joystick input."""
    keypressRotationStep: float
    """rotation increment per keypress in degree (full rotation = 360 degree)."""
    keypressTranslationStep: float
    """translation increment per keypress relative to window size."""
    mouseMoveRotationFactor: float
    """rotation increment per 1 pixel mouse movement in degree."""
    pauseWithSpacebar: bool
    """True: during simulation, space bar can be pressed to pause simulation."""
    selectionHighlights: bool
    """True: enable mouse click to highlights item (default: red)."""
    selectionLeftMouse: bool
    """True: enable left mouse click on items to show basic information."""
    selectionLeftMouseItemTypes: int
    """binary flags (1,2,4,8,16) for (Node,Object,Marker,Load,Sensor) that are identified with left mouse click selection."""
    selectionRightMouse: bool
    """True: enable right mouse click on items to show dictionary (read only!)."""
    selectionRightMouseGraphicsData: bool
    """True: right mouse click on items also shows GraphicsData information for inspectation (may sometimes be very large and may not fit into dialog for large graphics objects!)."""
    zoomStepFactor: float
    """change of zoom per keypress (keypad +/-) or mouse wheel increment."""

#information for VSettingsInteractive
class VSettingsInteractive:
    """Functionality to interact with render window; includes special rotation and zoom factors, item-highlighting, marker tracking, item selection and keyPressUserFunction."""
    advanced: VSettingsInteractiveAdvanced
    """advanced interactive visualization settings."""
    openVR: VSettingsOpenVR
    """openVR visualization settings."""
    autoRotateModelView: bool
    """True: rotate model view with autorotation."""
    autoRotationVelocity: Tuple[float,float,float]
    """Angular velocity vector for auto-rotation of scene (only visualization view is rotated, not the model itself!)."""
    highlightItemIndex: int
    """index of item that shall be highlighted (e.g., to find item which cauess problems); if set -1, no item is highlighted."""
    highlightItemType: ItemType
    """item type (Node, Object, ...) that shall be highlighted (e.g., to find item which cauess problems)."""
    highlightMbsNumber: int
    """index of main system (mbs) for which the item shall be highlighted; number is related to the ID in SystemContainer (first mbs = 0, second = 1, ...)."""
    ignoreKeys: bool
    """True: ignore keyboard input except escape and 'F2' keys; used for interactive mode, e.g., to perform kinematic analysis; This flag can be switched with key 'F2'; if ignoreKeys=True, then keyPressUserFunction can be used!"""
    keyPressUserFunction: Any
    """add a Python function f(key, action, mods) here, which is called every time a key is pressed; set this parameter to 0 (int) in order to deactivate it; the user function is only called if interactive.ignoreKeys=True; function shall return true, if key has been processed; Example: ; def f(key, action, mods):; phantom{XXX} print('key=',key);; use chr(key) to convert key codes [32 ...96] to ascii; special key codes (>256) are provided in the exudyn.KeyCode enumeration type; key action needs to be checked (0=released, 1=pressed, 2=repeated); mods provide information (binary) for SHIFT (1), CTRL (2), ALT (4), Super keys (8), CAPSLOCK (16)."""
    logMouseCoordinates: bool
    """True: if showMouseCoordinates=True, also log mouse coordinates (transformed to model coordinates); only works for axis-aligned ortho-projections and shows the coordinates of the current plane."""
    useJoystickInput: bool
    """True: read joystick input (use 6-axis joystick with lowest ID found when starting renderer window) and interpret as (x,y,z) position and (rotx, roty, rotz) rotation: as available from 3Dconnexion space mouse and maybe others as well; set to False, if external joystick makes problems ..."""

#information for VisualizationSettings
class VisualizationSettings:
    """Top structure for all visualization settings in Exudyn."""
    bodies: VSettingsBodies
    """body visualization settings."""
    connectors: VSettingsConnectors
    """connector visualization settings."""
    contact: VSettingsContact
    """contact visualization settings."""
    contour: VSettingsContour
    """contour plot visualization settings."""
    dialogs: VSettingsDialogs
    """dialogs settings."""
    exportImages: VSettingsExportImages
    """settings for exporting (saving) images to files in order to create animations."""
    general: VSettingsGeneral
    """general visualization settings."""
    interactive: VSettingsInteractive
    """Settings for interaction with renderer."""
    loads: VSettingsLoads
    """load visualization settings."""
    markers: VSettingsMarkers
    """marker visualization settings."""
    nodes: VSettingsNodes
    """node visualization settings."""
    openGL: VSettingsOpenGL
    """OpenGL rendering settings."""
    raytracer: VSettingsRaytracer
    """Raytracer settings (builds on OpenGL rendering settings)."""
    sensors: VSettingsSensors
    """sensor visualization settings."""
    view0: VSettingsView
    """Settings for main view 0."""
    view1: VSettingsView
    """Settings for sub-view 1."""
    view2: VSettingsView
    """Settings for sub-view 2."""
    view3: VSettingsView
    """Settings for sub-view 3."""

#information for CSolverTimer
class CSolverTimer:
    """Structure for timing in solver. Each Real variable is used to measure the CPU time which certain parts of the solver need. This structure is only active if the code is not compiled with the __FAST_EXUDYN_LINALG option and if displayComputationTime is set True. Timings will only be filled, if useTimer is True."""
    AERHS: float
    """time for residual evaluation of algebraic equations right-hand-side."""
    errorEstimator: float
    """for explicit solvers, additional evaluation."""
    factorization: float
    """solve or inverse."""
    integrationFormula: float
    """time spent for evaluation of integration formulas."""
    jacobianAE: float
    """jacobian of algebraic equations (not counted in sum)."""
    jacobianODE1: float
    """jacobian w.r.t. coordinates of ODE1 equations (not counted in sum)."""
    jacobianODE2: float
    """jacobian w.r.t. coordinates of ODE2 equations (not counted in sum)."""
    jacobianODE2_t: float
    """jacobian w.r.t. coordinates_t of ODE2 equations (not counted in sum)."""
    massMatrix: float
    """mass matrix computation."""
    newtonIncrement: float
    """Jac:math:`^{-1}` * RHS; backsubstitution."""
    ODE1RHS: float
    """time for residual evaluation of ODE1 right-hand-side."""
    ODE2RHS: float
    """time for residual evaluation of ODE2 right-hand-side."""
    overhead: float
    """overhead, such as initialization, copying and some matrix-vector multiplication."""
    postNewton: float
    """discontinuous iteration / PostNewtonStep."""
    python: float
    """time spent for Python functions."""
    reactionForces: float
    """CqT * lambda."""
    realtimeIdleCPU: float
    """time waited for next frame to compute and draw if simulateInRealtime is True."""
    @overload
    def Reset(useSolverTimer) -> None: ...
    @overload
    def StartTimer(value) -> None: ...
    @overload
    def StopTimer(value) -> None: ...
    @overload
    def Sum() -> float: ...
    @overload
    def ToString() -> str: ...
    total: float
    """total time measured between start and end of computation (static/dynamics)."""
    totalJacobian: float
    """time for all jacobian computations."""
    useTimer: bool
    """flag to decide, whether the timer is used (true) or not."""
    visualization: float
    """time spent for visualization in computation thread."""
    writeSolution: float
    """time for writing solution."""

#information for SolverIterationData
class SolverIterationData:
    """Solver internal structure for counters, steps, step size, time, etc.; solution vectors, residuals, etc. are SolverLocalData. The given default values are overwritten by the simulationSettings when initializing the solver."""
    adaptiveStep: bool
    """True: the step size may be reduced if step fails; no automatic stepsize control."""
    automaticStepSize: bool
    """True: if timeIntegration.automaticStepSize == True AND chosen integrators supports automatic step size control (e.g., DOPRI5); False: constant step size used (step may be reduced if adaptiveStep=True)."""
    automaticStepSizeError: float
    r"""estimated error (relative to atol + rtol*solution) of last step; must be :math:`\le 1`  for a step to be accepted."""
    currentStepIndex: int
    """current step index; :math:`i`."""
    currentStepSize: float
    """stepSize of current step."""
    currentTime: float
    """holds the current simulation time, copy of state.current.time; interval is [startTime,tEnd]; in static solver, duration is loadStepDuration."""
    discontinuousIteration: int
    """number of current discontinuous iteration."""
    discontinuousIterationsCount: int
    """count total number of discontinuous iterations (min. 1 per step)."""
    endTime: float
    """end time of static/dynamic solver."""
    initialStepSize: float
    """initial stepSize for dynamic solver; only used, if automaticStepSize is activated."""
    lastStepSize: float
    """stepSize suggested from last step or by initial step size; only used, if automaticStepSize is activated."""
    maxStepSize: float
    """constant or maximum stepSize."""
    minStepSize: float
    """minimum stepSize for static/dynamic solver; only used, if automaticStepSize is activated."""
    newtonJacobiCount: int
    """count total Newton jacobian computations."""
    newtonSteps: int
    """number of current newton steps."""
    newtonStepsCount: int
    """count total Newton steps."""
    numberOfSteps: int
    """number of time steps (if fixed size); :math:`n`."""
    recommendedStepSize: float
    """recommended step size :math:`h_{recom}` after PostNewton(...): :math:`h_{recom} < 0`: no recommendation, :math:`h_{recom}==0`: use minimum step size, :math:`h_{recom}>0`: use specific step size, if no smaller size requested by other reason."""
    rejectedAutomaticStepSizeSteps: int
    """count the number of rejected steps in case of automatic step size control (rejected steps are repeated with smaller step size)."""
    rejectedModifiedNewtonSteps: int
    """count the number of rejected modified Newton steps (switch to full Newton)."""
    startTime: float
    """time at beginning of time integration."""
    @overload
    def ToString() -> str: ...

#information for SolverConvergenceData
class SolverConvergenceData:
    """Solver internal structure for convergence information: residua, iteration loop errors and error flags. For detailed behavior of these flags, visit the source code!"""
    contractivity: float
    """Newton contractivity = geometric decay of error in every step."""
    discontinuousIterationError: float
    """error of discontinuous iterations (contact, friction, ...) outside of Newton iteration."""
    discontinuousIterationSuccessful: bool
    """true, if last discontinuous iteration had success (failure may be recovered by adaptive step)."""
    errorCoordinateFactor: float
    """factor may include the number of system coordinates to reduce the residual."""
    @overload
    def InitializeData() -> None: ...
    jacobianUpdateRequested: bool
    """true, if a jacobian update is requested in modified Newton (determined in previous step)."""
    lastResidual: float
    """last Newton residual to determine contractivity."""
    linearSolverCausingRow: int
    """-1 if successful, 0 ... n-1, the system equation (=coordinate) index which may have caused the problem, at which the linear solver failed."""
    linearSolverFailed: bool
    """true, if linear solver failed to factorize."""
    massMatrixNotInvertible: bool
    """true, if mass matrix is not invertable during initialization or solution (explicit solver)."""
    newtonConverged: bool
    """true, if Newton has (finally) converged."""
    newtonSolutionDiverged: bool
    """true, if Newton diverged (may be recovered)."""
    residual: float
    """current Newton residual."""
    stepReductionFailed: bool
    """true, if iterations over time/static steps failed (finally, cannot be recovered)."""
    stopNewton: bool
    """set true by Newton, if Newton was stopped, e.g., because of exceeding iterations or linear solver failed."""

#information for SolverOutputData
class SolverOutputData:
    """Solver internal structure for output modes, output timers and counters."""
    cpuLastTimePrinted: float
    """CPU time when output has been printed last time."""
    cpuSolverStartTime: float
    """CPU start time of main solver (not including initial conditions); cpuSolverStartTime-cpuStartTime gives time for initialization."""
    cpuStartTime: float
    """CPU start time of computation (starts counting at computation of initial conditions)."""
    finishedSuccessfully: bool
    """flag is false until solver functions SolveSteps)...) or SolveSystem(...) finished successfully (can be used as external trigger)."""
    initializationSuccessful: bool
    """flag is set during call to InitializeSolver(...); reasons for failure are multiple, either inconsistent solver settings are used, files cannot be written (file locked), or initial conditions could not be computed."""
    @overload
    def InitializeData() -> None: ...
    lastDiscontinuousIterationsCount: int
    """discontinuous iterations count when written to console (or file) last time."""
    lastImageRecorded: float
    """simulation time when last image has been recorded."""
    lastNewtonJacobiCount: int
    """jacobian update count when written to console (or file) last time."""
    lastNewtonStepsCount: int
    """newton steps count when written to console (or file) last time."""
    lastSensorsWritten: float
    """simulation time when last sensors have been written."""
    lastSolutionWritten: float
    """simulation time when last solution has been written."""
    lastVerboseStepIndex: int
    """step index when last time written to console (or file)."""
    multiThreadingMode: int
    """multithreading mode that has been used: 0=None (serial), 1=multithreading, 2=multithreading with load balancing; (modes new since 2025-06, V1.9.198)."""
    numberOfThreadsUsed: int
    """number of threads that have been used in simulation."""
    simulationStoppedByUser: bool
    """flag (initialized false) is set true when user stops the simulation (press Q, Escape, etc.)."""
    simulationStoppedByUserFunction: bool
    """flag (initialized false) is set true when a user function (PreStep, PostNewton, etc.) sends termination signal."""
    simulationTimeout: bool
    """flag (initialized false) is set true when exudyn.special.solver.timeout is reached (and timeout is >= 0)."""
    stepInformation: int
    """this is a copy of the solvers stepInformation used for console output."""
    verboseMode: int
    """this is a copy of the solvers verboseMode used for console output."""
    verboseModeFile: int
    """this is a copy of the solvers verboseModeFile used for file."""
    writeToSolutionFile: bool
    """if false, no solution file is generated and no file is written."""
    writeToSolverFile: bool
    """if false, no solver output file is generated and no file is written."""

#information for MainSolverStatic
class MainSolverStatic:
    """PyBind interface (trampoline) class for static solver. With this interface, the static solver and its substructures can be accessed via Python. NOTE that except from SolveSystem(...), these functions are only intended for experienced users and they need to be handled with care, as unexpected crashes may happen if used inappropriate. Furthermore, the functions have a lot of overhead (performance much lower than internal solver) due to Python interfaces, and should thus be used for small systems. To access the solver in Python, write:  ; ``solver = MainSolverStatic()``  ; and hereafter you can access all data and functions via 'solver'."""
    conv: SolverConvergenceData
    """all information about tolerances, errors and residua."""
    it: SolverIterationData
    """all information about iterations (steps, discontinuous iteration, newton,...)."""
    newton: NewtonSettings
    """copy of newton settings from timeint or staticSolver."""
    output: SolverOutputData
    """output modes and timers for exporting solver information and solution."""
    timer: CSolverTimer
    """timer which measures the CPU time of solver sub functions."""
    @overload
    def CheckInitialized(mainSystem) -> bool: ...
    @overload
    def ComputeAlgebraicEquations(mainSystem, velocityLevel=False) -> None: ...
    @overload
    def ComputeJacobianAE(mainSystem, scalarFactor_ODE2=1., scalarFactor_ODE2_t=0., scalarFactor_ODE1=1., velocityLevel=False) -> None: ...
    @overload
    def ComputeJacobianODE1RHS(mainSystem, scalarFactor_ODE2=1., scalarFactor_ODE2_t=0., scalarFactor_ODE1=1.) -> None: ...
    @overload
    def ComputeJacobianODE2RHS(mainSystem, scalarFactor_ODE2=1., scalarFactor_ODE2_t=0., scalarFactor_ODE1=1., computeLoadsJacobian=0) -> None: ...
    @overload
    def ComputeLoadFactor(simulationSettings) -> float: ...
    @overload
    def ComputeMassMatrix(mainSystem, scalarFactor=1.) -> None: ...
    @overload
    def ComputeNewtonJacobian(mainSystem, simulationSettings) -> None: ...
    @overload
    def ComputeNewtonResidual(mainSystem, simulationSettings) -> float: ...
    @overload
    def ComputeNewtonUpdate(mainSystem, simulationSettings, initial=True) -> None: ...
    @overload
    def ComputeODE2RHS(mainSystem) -> None: ...
    @overload
    def DiscontinuousIteration(mainSystem, simulationSettings) -> bool: ...
    @overload
    def FinalizeSolver(mainSystem, simulationSettings) -> None: ...
    @overload
    def FinishStep(mainSystem, simulationSettings) -> None: ...
    @overload
    def GetAEsize() -> int: ...
    @overload
    def GetDataSize() -> int: ...
    @overload
    def GetErrorString() -> str: ...
    @overload
    def GetNewtonSolution() -> ArrayLike: ...
    @overload
    def GetODE1size() -> int: ...
    @overload
    def GetODE2size() -> int: ...
    @overload
    def GetSimulationEndTime(simulationSettings) -> float: ...
    @overload
    def GetSolverName() -> str: ...
    @overload
    def GetSystemJacobian() -> ArrayLike: ...
    @overload
    def GetSystemMassMatrix() -> ArrayLike: ...
    @overload
    def GetSystemResidual() -> ArrayLike: ...
    @overload
    def HasAutomaticStepSizeControl(mainSystem, simulationSettings) -> bool: ...
    @overload
    def IncreaseStepSize(mainSystem, simulationSettings) -> None: ...
    @overload
    def InitializeSolver(mainSystem, simulationSettings) -> bool: ...
    @overload
    def InitializeSolverData(mainSystem, simulationSettings) -> None: ...
    @overload
    def InitializeSolverInitialConditions(mainSystem, simulationSettings) -> None: ...
    @overload
    def InitializeSolverOutput(mainSystem, simulationSettings) -> None: ...
    @overload
    def InitializeSolverPreChecks(mainSystem, simulationSettings) -> bool: ...
    @overload
    def InitializeStep(mainSystem, simulationSettings) -> None: ...
    @overload
    def IsStaticSolver() -> bool: ...
    @overload
    def IsVerboseCheck(level) -> bool: ...
    loadStepGeometricFactor: float
    """multiplicative load step factor; this factor is computed from loadStepGeometric parameters in SolveSystem(...)."""
    @overload
    def Newton(mainSystem, simulationSettings) -> bool: ...
    @overload
    def PostInitializeSolverSpecific(mainSystem, simulationSettings) -> None: ...
    @overload
    def PreInitializeSolverSpecific(mainSystem, simulationSettings) -> None: ...
    @overload
    def ReduceStepSize(mainSystem, simulationSettings, severity) -> bool: ...
    @overload
    def SetSystemJacobian(systemJacobian) -> None: ...
    @overload
    def SetSystemMassMatrix(systemMassMatrix) -> None: ...
    @overload
    def SetSystemResidual(systemResidual) -> None: ...
    @overload
    def SolveSteps(mainSystem, simulationSettings) -> bool: ...
    @overload
    def SolveSystem(mainSystem, simulationSettings) -> bool: ...
    @overload
    def UpdateCurrentTime(mainSystem, simulationSettings) -> None: ...
    @overload
    def VerboseWrite(level, str) -> None: ...
    @overload
    def WriteCoordinatesToFile(mainSystem, simulationSettings) -> None: ...
    @overload
    def WriteSolutionFileHeader(mainSystem, simulationSettings) -> None: ...

#information for MainSolverImplicitSecondOrder
class MainSolverImplicitSecondOrder:
    """PyBind interface (trampoline) class for dynamic implicit solver. Note that this solver includes the classical Newmark method (set useNewmark True; with option of index 2 reduction) as well as the generalized-alpha method. With the interface, the dynamic implicit solver and its substructures can be accessed via Python. NOTE that except from SolveSystem(...), these functions are only intended for experienced users and they need to be handled with care, as unexpected crashes may happen if used inappropriate. Furthermore, the functions have a lot of overhead (still fast, but performance much lower than internal solver) due to Python interfaces, and should thus be used for small systems. To access the solver in Python, write:  ; ``solver = MainSolverImplicitSecondOrder()``  ; and hereafter you can access all data and functions via 'solver'.; In this solver, user functions are possible to extend the solver at certain parts, while keeping the overal C++ performance. User functions, which are added with SetUserFunction...(...), have the arguments (MainSolver, MainSystem, simulationSettings), except for ComputeNewtonUpdate which adds the initial flag as an additional argument and ComputeNewtonResidual, which returns the scalar residual."""
    conv: SolverConvergenceData
    """all information about tolerances, errors and residua."""
    it: SolverIterationData
    """all information about iterations (steps, discontinuous iteration, newton,...)."""
    newton: NewtonSettings
    """copy of newton settings from timeint or staticSolver."""
    output: SolverOutputData
    """output modes and timers for exporting solver information and solution."""
    timer: CSolverTimer
    """timer which measures the CPU time of solver sub functions; note that solver structures can only be written indirectly, e.g.,  timer=dynamicSolver.timer; timer.useTimer = False; dynamicSolver.timer=timer; however, dynamicSolver.timer.useTimer cannot be written."""
    alphaF: float
    """copy of parameter in timeIntegration.generalizedAlpha."""
    alphaM: float
    """copy of parameter in timeIntegration.generalizedAlpha."""
    @overload
    def CheckInitialized(mainSystem) -> bool: ...
    @overload
    def ComputeAlgebraicEquations(mainSystem, velocityLevel=False) -> None: ...
    @overload
    def ComputeJacobianAE(mainSystem, scalarFactor_ODE2=1., scalarFactor_ODE2_t=0., scalarFactor_ODE1=1., velocityLevel=False) -> None: ...
    @overload
    def ComputeJacobianODE1RHS(mainSystem, scalarFactor_ODE2=1., scalarFactor_ODE2_t=0., scalarFactor_ODE1=1.) -> None: ...
    @overload
    def ComputeJacobianODE2RHS(mainSystem, scalarFactor_ODE2=1., scalarFactor_ODE2_t=0., scalarFactor_ODE1=1., computeLoadsJacobian=0) -> None: ...
    @overload
    def ComputeLoadFactor(simulationSettings) -> float: ...
    @overload
    def ComputeMassMatrix(mainSystem, scalarFactor=1.) -> None: ...
    @overload
    def ComputeNewtonJacobian(mainSystem, simulationSettings) -> None: ...
    @overload
    def ComputeNewtonResidual(mainSystem, simulationSettings) -> float: ...
    @overload
    def ComputeNewtonUpdate(mainSystem, simulationSettings, initial=True) -> None: ...
    @overload
    def ComputeODE1RHS(mainSystem) -> None: ...
    @overload
    def ComputeODE2RHS(mainSystem) -> None: ...
    @overload
    def DiscontinuousIteration(mainSystem, simulationSettings) -> bool: ...
    factJacAlgorithmic: float
    """locally computed parameter from generalizedAlpha parameters."""
    @overload
    def FinalizeSolver(mainSystem, simulationSettings) -> None: ...
    @overload
    def FinishStep(mainSystem, simulationSettings) -> None: ...
    @overload
    def GetAAlgorithmic() -> ArrayLike: ...
    @overload
    def GetAEsize() -> int: ...
    @overload
    def GetDataSize() -> int: ...
    @overload
    def GetErrorString() -> str: ...
    @overload
    def GetNewtonSolution() -> ArrayLike: ...
    @overload
    def GetODE1size() -> int: ...
    @overload
    def GetODE2size() -> int: ...
    @overload
    def GetSimulationEndTime(simulationSettings) -> float: ...
    @overload
    def GetSolverName() -> str: ...
    @overload
    def GetStartOfStepStateAAlgorithmic() -> ArrayLike: ...
    @overload
    def GetSystemJacobian() -> ArrayLike: ...
    @overload
    def GetSystemMassMatrix() -> ArrayLike: ...
    @overload
    def GetSystemResidual() -> ArrayLike: ...
    @overload
    def HasAutomaticStepSizeControl(mainSystem, simulationSettings) -> bool: ...
    @overload
    def IncreaseStepSize(mainSystem, simulationSettings) -> None: ...
    @overload
    def InitializeSolver(mainSystem, simulationSettings) -> bool: ...
    @overload
    def InitializeSolverData(mainSystem, simulationSettings) -> None: ...
    @overload
    def InitializeSolverInitialConditions(mainSystem, simulationSettings) -> None: ...
    @overload
    def InitializeSolverOutput(mainSystem, simulationSettings) -> None: ...
    @overload
    def InitializeSolverPreChecks(mainSystem, simulationSettings) -> bool: ...
    @overload
    def InitializeStep(mainSystem, simulationSettings) -> None: ...
    @overload
    def IsStaticSolver() -> bool: ...
    @overload
    def IsVerboseCheck(level) -> bool: ...
    newmarkBeta: float
    """copy of parameter in timeIntegration.generalizedAlpha."""
    newmarkGamma: float
    """copy of parameter in timeIntegration.generalizedAlpha."""
    @overload
    def Newton(mainSystem, simulationSettings) -> bool: ...
    @overload
    def PostInitializeSolverSpecific(mainSystem, simulationSettings) -> None: ...
    @overload
    def PostNewton(mainSystem, simulationSettings) -> float: ...
    @overload
    def PreInitializeSolverSpecific(mainSystem, simulationSettings) -> None: ...
    @overload
    def ReduceStepSize(mainSystem, simulationSettings, severity) -> bool: ...
    @overload
    def SetSystemJacobian(systemJacobian) -> None: ...
    @overload
    def SetSystemMassMatrix(systemMassMatrix) -> None: ...
    @overload
    def SetSystemResidual(systemResidual) -> None: ...
    @overload
    def SetUserFunctionComputeNewtonJacobian(mainSystem, userFunction) -> None: ...
    @overload
    def SetUserFunctionComputeNewtonResidual(mainSystem, userFunction) -> None: ...
    @overload
    def SetUserFunctionComputeNewtonUpdate(mainSystem, userFunction) -> None: ...
    @overload
    def SetUserFunctionDiscontinuousIteration(mainSystem, userFunction) -> None: ...
    @overload
    def SetUserFunctionFinishStep(mainSystem, userFunction) -> None: ...
    @overload
    def SetUserFunctionInitializeStep(mainSystem, userFunction) -> None: ...
    @overload
    def SetUserFunctionNewton(mainSystem, userFunction) -> None: ...
    @overload
    def SetUserFunctionPostNewton(mainSystem, userFunction) -> None: ...
    @overload
    def SetUserFunctionUpdateCurrentTime(mainSystem, userFunction) -> None: ...
    @overload
    def SolveSteps(mainSystem, simulationSettings) -> bool: ...
    @overload
    def SolveSystem(mainSystem, simulationSettings) -> bool: ...
    spectralRadius: float
    """copy of parameter in timeIntegration.generalizedAlpha."""
    @overload
    def UpdateCurrentTime(mainSystem, simulationSettings) -> None: ...
    @overload
    def VerboseWrite(level, str) -> None: ...
    @overload
    def WriteCoordinatesToFile(mainSystem, simulationSettings) -> None: ...
    @overload
    def WriteSolutionFileHeader(mainSystem, simulationSettings) -> None: ...

#information for MainSolverExplicit
class MainSolverExplicit:
    """PyBind interface (trampoline) class for dynamic explicit solver. Note that this solver includes the 1st order explicit Euler scheme and the 4th order Runge-Kutta scheme with 5th order error estimation (DOPRI5). With the interface, the solver and its substructures can be accessed via Python. NOTE that except from SolveSystem(...), these functions are only intended for experienced users and they need to be handled with care, as unexpected crashes may happen if used inappropriate. Furthermore, the functions have a lot of overhead (still fast, but performance much lower than internal solver) due to Python interfaces, and should thus be used for small systems. To access the solver in Python, write  ; ``solver = MainSolverExplicit()``  ; and hereafter you can access all data and functions via 'solver'.; In this solver, no user functions are possible, but you can use SolverImplicitSecondOrder instead (turning off Newton gives explicit scheme ...)."""
    conv: SolverConvergenceData
    """all information about tolerances, errors and residua."""
    it: SolverIterationData
    """all information about iterations (steps, discontinuous iteration, newton,...)."""
    output: SolverOutputData
    """output modes and timers for exporting solver information and solution."""
    timer: CSolverTimer
    """timer which measures the CPU time of solver sub functions."""
    @overload
    def ComputeLoadFactor(simulationSettings) -> float: ...
    @overload
    def ComputeMassMatrix(mainSystem, scalarFactor=1.) -> None: ...
    @overload
    def ComputeNewtonJacobian(mainSystem, simulationSettings) -> None: ...
    @overload
    def ComputeNewtonResidual(mainSystem, simulationSettings) -> float: ...
    @overload
    def ComputeNewtonUpdate(mainSystem, simulationSettings, initial=True) -> None: ...
    @overload
    def ComputeODE1RHS(mainSystem) -> None: ...
    @overload
    def ComputeODE2RHS(mainSystem) -> None: ...
    @overload
    def DiscontinuousIteration(mainSystem, simulationSettings) -> bool: ...
    @overload
    def FinalizeSolver(mainSystem, simulationSettings) -> None: ...
    @overload
    def FinishStep(mainSystem, simulationSettings) -> None: ...
    @overload
    def GetAEsize() -> int: ...
    @overload
    def GetDataSize() -> int: ...
    @overload
    def GetErrorString() -> str: ...
    @overload
    def GetMethodOrder() -> int: ...
    @overload
    def GetNumberOfStages() -> int: ...
    @overload
    def GetODE1size() -> int: ...
    @overload
    def GetODE2size() -> int: ...
    @overload
    def GetSimulationEndTime(simulationSettings) -> float: ...
    @overload
    def GetSolverName() -> str: ...
    @overload
    def GetSystemMassMatrix() -> ArrayLike: ...
    @overload
    def GetSystemResidual() -> ArrayLike: ...
    @overload
    def HasAutomaticStepSizeControl(mainSystem, simulationSettings) -> bool: ...
    @overload
    def IncreaseStepSize(mainSystem, simulationSettings) -> None: ...
    @overload
    def InitializeSolver(mainSystem, simulationSettings) -> bool: ...
    @overload
    def InitializeSolverData(mainSystem, simulationSettings) -> None: ...
    @overload
    def InitializeSolverInitialConditions(mainSystem, simulationSettings) -> None: ...
    @overload
    def InitializeSolverOutput(mainSystem, simulationSettings) -> None: ...
    @overload
    def InitializeSolverPreChecks(mainSystem, simulationSettings) -> bool: ...
    @overload
    def InitializeStep(mainSystem, simulationSettings) -> None: ...
    @overload
    def IsStaticSolver() -> bool: ...
    @overload
    def IsVerboseCheck(level) -> bool: ...
    @overload
    def Newton(mainSystem, simulationSettings) -> bool: ...
    @overload
    def PostInitializeSolverSpecific(mainSystem, simulationSettings) -> None: ...
    @overload
    def PreInitializeSolverSpecific(mainSystem, simulationSettings) -> None: ...
    @overload
    def ReduceStepSize(mainSystem, simulationSettings, severity) -> bool: ...
    @overload
    def SetSystemMassMatrix(systemMassMatrix) -> None: ...
    @overload
    def SetSystemResidual(systemResidual) -> None: ...
    @overload
    def SolveSteps(mainSystem, simulationSettings) -> bool: ...
    @overload
    def SolveSystem(mainSystem, simulationSettings) -> bool: ...
    @overload
    def UpdateCurrentTime(mainSystem, simulationSettings) -> None: ...
    @overload
    def VerboseWrite(level, str) -> None: ...
    @overload
    def WriteCoordinatesToFile(mainSystem, simulationSettings) -> None: ...
    @overload
    def WriteSolutionFileHeader(mainSystem, simulationSettings) -> None: ...
