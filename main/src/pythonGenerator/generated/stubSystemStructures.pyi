
#This is the stub file for system structures, such as SimulationSettings and VisualizationSettings
#This file will greatly improve autocompletion


#information for BeamSection
class BeamSection:
    dampingMatrix: ArrayLike
    inertia: ArrayLike
    massPerLength: float
    stiffnessMatrix: ArrayLike

#information for BeamSectionGeometry
class BeamSectionGeometry:
    crossSectionRadiusY: float
    crossSectionRadiusZ: float
    crossSectionType: CrossSectionType
    polygonalPoints: Vector2DList

#information for SolutionSettings
class SolutionSettings:
    appendToFile: bool
    binarySolutionFile: bool
    coordinatesSolutionFileName: str
    exportAccelerations: bool
    exportAlgebraicCoordinates: bool
    exportDataCoordinates: bool
    exportODE1Velocities: bool
    exportVelocities: bool
    flushFilesDOF: int
    flushFilesImmediately: bool
    outputPrecision: int
    recordImagesInterval: float
    restartFileName: str
    restartWritePeriod: float
    sensorsAppendToFile: bool
    sensorsStoreAndWriteFiles: bool
    sensorsWriteFileFooter: bool
    sensorsWriteFileHeader: bool
    sensorsWritePeriod: float
    solutionInformation: str
    solutionWritePeriod: float
    solverInformationFileName: str
    writeFileFooter: bool
    writeFileHeader: bool
    writeInitialValues: bool
    writeRestartFile: bool
    writeSolutionToFile: bool

#information for NumericalDifferentiationSettings
class NumericalDifferentiationSettings:
    addReferenceCoordinatesToEpsilon: bool
    doSystemWideDifferentiation: bool
    forAE: bool
    forODE2: bool
    forODE2connectors: bool
    jacobianConnectorDerivative: bool
    minimumCoordinateSize: float
    relativeEpsilon: float

#information for DiscontinuousSettings
class DiscontinuousSettings:
    ignoreMaxIterations: bool
    iterationTolerance: float
    maxIterations: int

#information for NewtonSettings
class NewtonSettings:
    numericalDifferentiation: NumericalDifferentiationSettings
    absoluteTolerance: float
    adaptInitialResidual: bool
    maximumSolutionNorm: float
    maxIterations: int
    maxModifiedNewtonIterations: int
    maxModifiedNewtonRestartIterations: int
    modifiedNewtonContractivity: float
    modifiedNewtonJacUpdatePerStep: bool
    newtonResidualMode: int
    relativeTolerance: float
    useModifiedNewton: bool
    useNewtonSolver: bool
    weightTolerancePerCoordinate: bool

#information for GeneralizedAlphaSettings
class GeneralizedAlphaSettings:
    computeInitialAccelerations: bool
    lieGroupAddTangentOperator: bool
    newmarkBeta: float
    newmarkGamma: float
    resetAccelerations: bool
    spectralRadius: float
    useIndex2Constraints: bool
    useNewmark: bool

#information for ExplicitIntegrationSettings
class ExplicitIntegrationSettings:
    computeEndOfStepAccelerations: bool
    computeMassMatrixInversePerBody: bool
    dynamicSolverType: DynamicSolverType
    eliminateConstraints: bool
    useLieGroupIntegration: bool

#information for TimeIntegrationSettings
class TimeIntegrationSettings:
    discontinuous: DiscontinuousSettings
    explicitIntegration: ExplicitIntegrationSettings
    generalizedAlpha: GeneralizedAlphaSettings
    newton: NewtonSettings
    absoluteTolerance: float
    adaptiveStep: bool
    adaptiveStepDecrease: float
    adaptiveStepIncrease: float
    adaptiveStepRecoveryIterations: int
    adaptiveStepRecoverySteps: int
    automaticStepSize: bool
    computeLoadsJacobian: int
    endTime: float
    initialStepSize: float
    minimumStepSize: float
    numberOfSteps: int
    realtimeFactor: float
    realtimeWaitMicroseconds: int
    relativeTolerance: float
    reuseConstantMassMatrix: bool
    simulateInRealtime: bool
    startTime: float
    stepInformation: int
    stepSizeMaxIncrease: float
    stepSizeSafety: float
    verboseMode: int
    verboseModeFile: int

#information for StaticSolverSettings
class StaticSolverSettings:
    discontinuous: DiscontinuousSettings
    newton: NewtonSettings
    adaptiveStep: bool
    adaptiveStepDecrease: float
    adaptiveStepIncrease: float
    adaptiveStepRecoveryIterations: int
    adaptiveStepRecoverySteps: int
    computeLoadsJacobian: bool
    loadStepDuration: float
    loadStepGeometric: bool
    loadStepGeometricRange: float
    loadStepStart: float
    minimumStepSize: float
    numberOfLoadSteps: int
    stabilizerODE2term: float
    stepInformation: int
    useLoadFactor: bool
    verboseMode: int
    verboseModeFile: int

#information for LinearSolverSettings
class LinearSolverSettings:
    ignoreSingularJacobian: bool
    pivotThreshold: float
    reuseAnalyzedPattern: bool
    showCausingItems: bool

#information for Parallel
class Parallel:
    multithreadedLLimitJacobians: int
    multithreadedLLimitLoads: int
    multithreadedLLimitMassMatrices: int
    multithreadedLLimitResiduals: int
    numberOfThreads: int
    taskSplitMinItems: int
    taskSplitTasksPerThread: int

#information for SimulationSettings
class SimulationSettings:
    linearSolverSettings: LinearSolverSettings
    parallel: Parallel
    solutionSettings: SolutionSettings
    staticSolver: StaticSolverSettings
    timeIntegration: TimeIntegrationSettings
    cleanUpMemory: bool
    displayComputationTime: bool
    displayGlobalTimers: bool
    displayStatistics: bool
    linearSolverType: LinearSolverType
    outputPrecision: int
    pauseAfterEachStep: bool

#information for VSettingsGeneral
class VSettingsGeneral:
    autoFitScene: bool
    axesTiling: int
    backgroundColor: Tuple[float,float,float,float]
    backgroundColorBottom: Tuple[float,float,float,float]
    circleTiling: int
    coordinateSystemSize: float
    cylinderTiling: int
    drawCoordinateSystem: bool
    drawWorldBasis: bool
    graphicsUpdateInterval: float
    linuxDisplayScaleFactor: float
    minSceneSize: float
    pointSize: float
    rendererPrecision: int
    renderWindowString: str
    showComputationInfo: bool
    showHelpOnStartup: int
    showSolutionInformation: bool
    showSolverInformation: bool
    showSolverTime: bool
    sphereTiling: int
    textAlwaysInFront: bool
    textColor: Tuple[float,float,float,float]
    textHasBackground: bool
    textOffsetFactor: float
    textSize: float
    threadSafeGraphicsUpdate: bool
    useBitmapText: bool
    useGradientBackground: bool
    useMultiThreadedRendering: bool
    useWindowsDisplayScaleFactor: bool
    worldBasisSize: float

#information for VSettingsContour
class VSettingsContour:
    automaticRange: bool
    colorBarPrecision: int
    colorBarTiling: int
    maxValue: float
    minValue: float
    nodesColored: bool
    outputVariable: OutputVariableType
    outputVariableComponent: int
    reduceRange: bool
    rigidBodiesColored: bool
    showColorBar: bool

#information for VSettingsNodes
class VSettingsNodes:
    basisSize: float
    defaultColor: Tuple[float,float,float,float]
    defaultSize: float
    drawNodesAsPoint: bool
    show: bool
    showBasis: bool
    showNodalSlopes: int
    showNumbers: bool
    tiling: int

#information for VSettingsBeams
class VSettingsBeams:
    axialTiling: int
    crossSectionFilled: bool
    crossSectionTiling: int
    drawVertical: bool
    drawVerticalColor: Tuple[float,float,float,float]
    drawVerticalFactor: float
    drawVerticalLines: bool
    drawVerticalOffset: float
    drawVerticalValues: bool
    reducedAxialInterploation: bool

#information for VSettingsKinematicTree
class VSettingsKinematicTree:
    frameSize: float
    showCOMframes: bool
    showFramesNumbers: bool
    showJointFrames: bool

#information for VSettingsBodies
class VSettingsBodies:
    beams: VSettingsBeams
    kinematicTree: VSettingsKinematicTree
    defaultColor: Tuple[float,float,float,float]
    defaultSize: Tuple[float,float,float]
    deformationScaleFactor: float
    show: bool
    showNumbers: bool

#information for VSettingsConnectors
class VSettingsConnectors:
    contactPointsDefaultSize: float
    defaultColor: Tuple[float,float,float,float]
    defaultSize: float
    jointAxesLength: float
    jointAxesRadius: float
    show: bool
    showContact: bool
    showJointAxes: bool
    showNumbers: bool
    springNumberOfWindings: int

#information for VSettingsMarkers
class VSettingsMarkers:
    defaultColor: Tuple[float,float,float,float]
    defaultSize: float
    drawSimplified: bool
    show: bool
    showNumbers: bool

#information for VSettingsLoads
class VSettingsLoads:
    defaultColor: Tuple[float,float,float,float]
    defaultRadius: float
    defaultSize: float
    drawSimplified: bool
    fixedLoadSize: bool
    loadSizeFactor: float
    show: bool
    showNumbers: bool

#information for VSettingsSensors
class VSettingsSensors:
    defaultColor: Tuple[float,float,float,float]
    defaultSize: float
    drawSimplified: bool
    show: bool
    showNumbers: bool

#information for VSettingsContact
class VSettingsContact:
    colorBoundingBoxes: Tuple[float,float,float,float]
    colorSearchTree: Tuple[float,float,float,float]
    contactForcesFactor: float
    contactPointsDefaultSize: float
    showBoundingBoxes: bool
    showContactForces: bool
    showContactForcesValues: bool
    showSearchTree: bool
    showSearchTreeCells: bool

#information for VSettingsWindow
class VSettingsWindow:
    alwaysOnTop: bool
    ignoreKeys: bool
    keyPressUserFunction: Any
    limitWindowToScreenSize: bool
    maximize: bool
    reallyQuitTimeLimit: float
    renderWindowSize: Tuple[int,int]
    @overload
    def ResetKeyPressUserFunction() -> None: ...
    showMouseCoordinates: bool
    showWindow: bool
    startupTimeout: int

#information for VSettingsDialogs
class VSettingsDialogs:
    alphaTransparency: float
    alwaysTopmost: bool
    fontScalingMacOS: float
    multiThreadedDialogs: bool
    openTreeView: bool

#information for VSettingsOpenGL
class VSettingsOpenGL:
    drawFaceNormals: bool
    drawNormalsLength: float
    drawVertexNormals: bool
    enableLight0: bool
    enableLight1: bool
    enableLighting: bool
    faceEdgesColor: Tuple[float,float,float,float]
    facesTransparent: bool
    initialCenterPoint: Tuple[float,float,float]
    initialMaxSceneSize: float
    initialModelRotation: ArrayLike
    initialZoom: float
    light0ambient: float
    light0constantAttenuation: float
    light0diffuse: float
    light0linearAttenuation: float
    light0position: Tuple[float,float,float,float]
    light0quadraticAttenuation: float
    light0specular: float
    light1ambient: float
    light1constantAttenuation: float
    light1diffuse: float
    light1linearAttenuation: float
    light1position: Tuple[float,float,float,float]
    light1quadraticAttenuation: float
    light1specular: float
    lightModelAmbient: Tuple[float,float,float,float]
    lightModelLocalViewer: bool
    lightModelTwoSide: bool
    lineSmooth: bool
    lineWidth: float
    materialAmbientAndDiffuse: Tuple[float,float,float,float]
    materialShininess: float
    materialSpecular: Tuple[float,float,float,float]
    multiSampling: int
    perspective: float
    polygonOffset: float
    shadeModelSmooth: bool
    shadow: float
    shadowPolygonOffset: float
    showFaceEdges: bool
    showFaces: bool
    showLines: bool
    showMeshEdges: bool
    showMeshFaces: bool
    textLineSmooth: bool
    textLineWidth: float

#information for VSettingsExportImages
class VSettingsExportImages:
    heightAlignment: int
    saveImageAsTextCircles: bool
    saveImageAsTextLines: bool
    saveImageAsTextTexts: bool
    saveImageAsTextTriangles: bool
    saveImageFileCounter: int
    saveImageFileName: str
    saveImageFormat: str
    saveImageSingleFile: bool
    saveImageTimeOut: int
    widthAlignment: int

#information for VSettingsOpenVR
class VSettingsOpenVR:
    actionManifestFileName: str
    enable: bool
    logLevel: int
    showCompanionWindow: bool

#information for VSettingsInteractive
class VSettingsInteractive:
    openVR: VSettingsOpenVR
    highlightColor: Tuple[float,float,float,float]
    highlightItemIndex: int
    highlightItemType: ItemType
    highlightMbsNumber: int
    highlightOtherColor: Tuple[float,float,float,float]
    joystickScaleRotation: float
    joystickScaleTranslation: float
    keypressRotationStep: float
    keypressTranslationStep: float
    lockModelView: bool
    mouseMoveRotationFactor: float
    pauseWithSpacebar: bool
    selectionHighlights: bool
    selectionLeftMouse: bool
    selectionRightMouse: bool
    selectionRightMouseGraphicsData: bool
    trackMarker: int
    trackMarkerMbsNumber: int
    trackMarkerOrientation: Tuple[float,float,float]
    trackMarkerPosition: Tuple[float,float,float]
    useJoystickInput: bool
    zoomStepFactor: float

#information for VisualizationSettings
class VisualizationSettings:
    bodies: VSettingsBodies
    connectors: VSettingsConnectors
    contact: VSettingsContact
    contour: VSettingsContour
    dialogs: VSettingsDialogs
    exportImages: VSettingsExportImages
    general: VSettingsGeneral
    interactive: VSettingsInteractive
    loads: VSettingsLoads
    markers: VSettingsMarkers
    nodes: VSettingsNodes
    openGL: VSettingsOpenGL
    sensors: VSettingsSensors
    window: VSettingsWindow

#information for CSolverTimer
class CSolverTimer:
    AERHS: float
    errorEstimator: float
    factorization: float
    integrationFormula: float
    jacobianAE: float
    jacobianODE1: float
    jacobianODE2: float
    jacobianODE2_t: float
    massMatrix: float
    newtonIncrement: float
    ODE1RHS: float
    ODE2RHS: float
    overhead: float
    postNewton: float
    python: float
    reactionForces: float
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
    totalJacobian: float
    useTimer: bool
    visualization: float
    writeSolution: float

#information for SolverIterationData
class SolverIterationData:
    adaptiveStep: bool
    automaticStepSize: bool
    automaticStepSizeError: float
    currentStepIndex: int
    currentStepSize: float
    currentTime: float
    discontinuousIteration: int
    discontinuousIterationsCount: int
    endTime: float
    initialStepSize: float
    lastStepSize: float
    maxStepSize: float
    minStepSize: float
    newtonJacobiCount: int
    newtonSteps: int
    newtonStepsCount: int
    numberOfSteps: int
    recommendedStepSize: float
    rejectedAutomaticStepSizeSteps: int
    rejectedModifiedNewtonSteps: int
    startTime: float
    @overload
    def ToString() -> str: ...

#information for SolverConvergenceData
class SolverConvergenceData:
    contractivity: float
    discontinuousIterationError: float
    discontinuousIterationSuccessful: bool
    errorCoordinateFactor: float
    @overload
    def InitializeData() -> None: ...
    jacobianUpdateRequested: bool
    lastResidual: float
    linearSolverCausingRow: int
    linearSolverFailed: bool
    massMatrixNotInvertible: bool
    newtonConverged: bool
    newtonSolutionDiverged: bool
    residual: float
    stepReductionFailed: bool
    stopNewton: bool

#information for SolverOutputData
class SolverOutputData:
    cpuLastTimePrinted: float
    cpuStartTime: float
    finishedSuccessfully: bool
    initializationSuccessful: bool
    @overload
    def InitializeData() -> None: ...
    lastDiscontinuousIterationsCount: int
    lastImageRecorded: float
    lastNewtonJacobiCount: int
    lastNewtonStepsCount: int
    lastSensorsWritten: float
    lastSolutionWritten: float
    lastVerboseStepIndex: int
    multiThreadingMode: int
    numberOfThreadsUsed: int
    stepInformation: int
    verboseMode: int
    verboseModeFile: int
    writeToSolutionFile: bool
    writeToSolverFile: bool

#information for MainSolverStatic
class MainSolverStatic:
    conv: SolverConvergenceData
    it: SolverIterationData
    newton: NewtonSettings
    output: SolverOutputData
    timer: CSolverTimer
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
    conv: SolverConvergenceData
    it: SolverIterationData
    newton: NewtonSettings
    output: SolverOutputData
    timer: CSolverTimer
    alphaF: float
    alphaM: float
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
    newmarkGamma: float
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
    conv: SolverConvergenceData
    it: SolverIterationData
    output: SolverOutputData
    timer: CSolverTimer
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
