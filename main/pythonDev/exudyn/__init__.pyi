#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is the EXUDYN stub file initialization
#
# Author:   Johannes Gerstmayr
# Date:     2023-05-09
#
# Notes:    Under development; see https://peps.python.org/pep-0484/#stub-files
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#from typing import Dict, List, Optional

from typing import (
    Annotated, 
    Any,
    # ByteString,
    # Callable,
    # Container,
    Callable,
    Dict,
    # Generic,
    # IO,
    # Iterable,
    # Iterator,
    List,
    Literal,
    # Mapping,
    # NoReturn,
    # Optional,
    overload,
    # Sequence,
    # Sized,
    # SupportsComplex,
    # SupportsFloat,
    # SupportsInt,
    # Text,
    Tuple, #for Tuple[int, int]
    # Type,
    TypeVar,
    Union,
)
from numpy.typing import ArrayLike, NDArray
from enum import Enum
import numpy as np


#DType = TypeVar("DType", bound=np.generic)
# NPreal3D = Annotated[NDArray[float], Literal[3]]
# NPint3D = Annotated[NDArray[int], Literal[3]]

#type variables for size in numpy arrays
T1 = TypeVar("T1", bound=int)
T2 = TypeVar("T2", bound=int)

# Dimension types represented as typles
Shape = Tuple
Shape1D = Shape[T1]
Shape2D = Shape[T1, T2]
# Shape3D = Shape[T1, T2, T3]
# ShapeND = Shape[T1, ...]
# ShapeNDType = TypeVar("ShapeNDType", bound=ShapeND)

#LENGTH = Literal[2]
#NDArray[Shape2D[3,3], np.float64]


import exudyn
from exudyn import (ObjectIndex, NodeIndex, MarkerIndex, LoadIndex, SensorIndex,
                    # MainSystem, SystemContainer,
                    # Vector2DList, Vector3DList, Vector6DList,
                    # MatrixContainer, 
                    # VisualizationSettings, #could be erased in future
                    )
from exudyn.graphicsDataUtilities import color4red, color4default


# class MainSystem:
#     @overload
#     def CreateMassPoint(self,#node quantities
#                                 referenceCoordinates = [0.,0.,0.],
#                                 initialCoordinates = [0.,0.,0.],
#                                 initialVelocities = [0.,0.,0.],
#                                 #object quantities:
#                                 physicsMass=0,
#                                 gravity = [0.,0.,0.],
#                                 graphicsDataList = [],
#                                 #graphics, mixed
#                                 drawSize = -1,
#                                 color =  [-1.,-1.,-1.,-1.],
#                                 show = True, #if graphicsDataList is empty, node is shown, otherwise body is shown
#                                 name = '',   #both for node and object
#                                 create2D = False, #NodePoint2D, MassPoint2D
#                                 returnDict = False, #if True, returns dictionary of all data
#                                 ) -> Union[ObjectIndex, dict]: ...
    
#     @overload
#     def CreateSpringDamper(self,
#                                   bodyOrNode0:int, bodyOrNode1:int,
#                                   localPosition0: list = [0.,0.,0.],
#                                   localPosition1: list = [0.,0.,0.],
#                                   #
#                                   referenceLength = None, 
#                                   stiffness: float = 0., damping: float = 0., force: float = 0.,
#                                   velocityOffset: float = 0., 
#                                   show=True, drawSize: float=-1, color=[-1.,-1.,-1.,-1.],
#                                   ) -> ObjectIndex: ...

#     @overload
#     def CreateRevoluteJoint(mbs, bodyNumber0:ObjectIndex, bodyNumber1:ObjectIndex, 
#                                   position:[float,float,float], 
#                                   axis:[float,float,float], useGlobalFrame=True, 
#                                   show=True, axisRadius=0.1, axisLength=0.4,
#                                   color=[-1.,-1.,-1.,-1.]) -> [ObjectIndex, MarkerIndex, MarkerIndex]: ...

#     @overload
#     def CreatePrismaticJoint(mbs, bodyNumber0:ObjectIndex, bodyNumber1:ObjectIndex, 
#                                   position:[float,float,float], 
#                                   axis:[float,float,float], useGlobalFrame=True, 
#                                   show=True, axisRadius=0.1, axisLength=0.4,
#                                   color=[-1.,-1.,-1.,-1.]) -> [ObjectIndex, MarkerIndex, MarkerIndex]: ...

#     @overload
#     def CreateGenericJoint(mbs, bodyNumber0:ObjectIndex, bodyNumber1:ObjectIndex, 
#                                   position:[float,float,float], rotation0=np.eye(3),
#                                   constrainedAxes=[1,1,1, 1,1,1], useGlobalFrame=True, 
#                                   show=True, axesRadius=0.1, axesLength=0.4,
#                                   color=[-1.,-1.,-1.,-1.]) -> [ObjectIndex, MarkerIndex, MarkerIndex]: ...
    
    # @overload
    # def AddMassPoint(self, referenceCoordinates: list = [0.,0.,0.],
    #                            initialCoordinates: list = [0.,0.,0.],
    #                            initialVelocities: list = [0.,0.,0.],
    #                            physicsMass: float=0,
    #                            gravity: list = [0.,0.,0.],
    #                            graphicsDataList: list = []) -> exudyn.ObjectIndex: ...

class OutputVariableType(Enum):
    _None = int
    Distance = int
    Position = int
    Displacement = int
    DisplacementLocal = int
    Velocity = int
    VelocityLocal = int
    Acceleration = int
    AccelerationLocal = int
    RotationMatrix = int
    Rotation = int
    AngularVelocity = int
    AngularVelocityLocal = int
    AngularAcceleration = int
    AngularAccelerationLocal = int
    Coordinates = int
    Coordinates_t = int
    Coordinates_tt = int
    SlidingCoordinate = int
    Director1 = int
    Director2 = int
    Director3 = int
    Force = int
    ForceLocal = int
    Torque = int
    TorqueLocal = int
    StrainLocal = int
    StressLocal = int
    CurvatureLocal = int
    ConstraintEquation = int

class ConfigurationType(Enum):
    _None = int
    Initial = int
    Current = int
    Reference = int
    StartOfStep = int
    Visualization = int
    EndOfEnumList = int

class ItemType(Enum):
    _None = int
    Node = int
    Object = int
    Marker = int
    Load = int
    Sensor = int

class NodeType(Enum):
    _None = int
    Ground = int
    Position2D = int
    Orientation2D = int
    Point2DSlope1 = int
    Position = int
    Orientation = int
    RigidBody = int
    RotationEulerParameters = int
    RotationRxyz = int
    RotationRotationVector = int
    LieGroupWithDirectUpdate = int
    LieGroupWithDataCoordinates = int
    GenericODE2 = int
    GenericODE1 = int
    GenericAE = int
    GenericData = int
    Point3DSlope1 = int
    Point3DSlope23 = int

class JointType(Enum):
    _None = int
    RevoluteX = int
    RevoluteY = int
    RevoluteZ = int
    PrismaticX = int
    PrismaticY = int
    PrismaticZ = int

class DynamicSolverType(Enum):
    GeneralizedAlpha = int
    TrapezoidalIndex2 = int
    ExplicitEuler = int
    ExplicitMidpoint = int
    RK33 = int
    RK44 = int
    RK67 = int
    ODE23 = int
    DOPRI5 = int
    DVERK6 = int

class CrossSectionType(Enum):
    Polygon = int
    Circular = int

class KeyCode(Enum):
    SPACE = int
    ENTER = int
    TAB = int
    BACKSPACE = int
    RIGHT = int
    LEFT = int
    DOWN = int
    UP = int
    F1 = int
    F2 = int
    F3 = int
    F4 = int
    F5 = int
    F6 = int
    F7 = int
    F8 = int
    F9 = int
    F10 = int

class LinearSolverType(Enum):
    _None = int
    EXUdense = int
    EigenSparse = int
    EigenSparseSymmetric = int

class ContactTypeIndex(Enum):
    IndexSpheresMarkerBased = int
    IndexANCFCable2D = int
    IndexTrigsRigidBodyBased = int
    IndexEndOfEnumList = int

#stub information for class MatrixContainer functions
class MatrixContainer:
    @overload
    def SetWithDenseMatrix(self, pyArray: ArrayLike, useDenseMatrix=False) -> None: ...
    @overload
    def SetWithSparseMatrixCSR(self, numberOfRowsInit: int, numberOfColumnsInit: int, pyArrayCSR: ArrayLike, useDenseMatrix=True) -> None: ...
    @overload
    def GetPythonObject(self) -> Union[dict,ArrayLike]: ...
    @overload
    def Convert2DenseMatrix(self) -> ArrayLike: ...
    @overload
    def UseDenseMatrix(self) -> bool: ...

#stub information for class Vector3DList functions
class Vector3DList:
    @overload
    def Append(self, pyArray: [float,float,float]) -> None: ...
    @overload
    def GetPythonObject(self) -> List[[float,float,float]]: ...

#stub information for class Vector2DList functions
class Vector2DList:
    @overload
    def Append(self, pyArray: [float,float]) -> None: ...
    @overload
    def GetPythonObject(self) -> List[[float,float]]: ...

#stub information for class Vector6DList functions
class Vector6DList:
    @overload
    def Append(self, pyArray: [float,float,float,float,float,float]) -> None: ...
    @overload
    def GetPythonObject(self) -> List[[float,float,float,float,float,float]]: ...

#stub information for class Matrix3DList functions
class Matrix3DList:
    @overload
    def Append(self, pyArray: NDArray[Shape2D[3,3], float]) -> None: ...
    @overload
    def GetPythonObject(self) -> List[NDArray[Shape2D[3,3], float]]: ...

#stub information for class Matrix6DList functions
class Matrix6DList:
    @overload
    def Append(self, pyArray: NDArray[Shape2D[6,6], float]) -> None: ...
    @overload
    def GetPythonObject(self) -> List[NDArray[Shape2D[6,6], float]]: ...

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
    ignoreRedundantConstraints: bool
    ignoreSingularJacobian: bool
    pivotTreshold: float
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
    renderWindowSize: Tuple[int,int]
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

#information for SolverConvergenceData
class SolverConvergenceData:
    contractivity: float
    discontinuousIterationError: float
    discontinuousIterationSuccessful: bool
    errorCoordinateFactor: float
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
    loadStepGeometricFactor: float

#information for MainSolverImplicitSecondOrder
class MainSolverImplicitSecondOrder:
    conv: SolverConvergenceData
    it: SolverIterationData
    newton: NewtonSettings
    output: SolverOutputData
    timer: CSolverTimer
    alphaF: float
    alphaM: float
    factJacAlgorithmic: float
    newmarkBeta: float
    newmarkGamma: float
    spectralRadius: float

#information for MainSolverExplicit
class MainSolverExplicit:
    conv: SolverConvergenceData
    it: SolverIterationData
    output: SolverOutputData
    timer: CSolverTimer

#stub information for class VisuGeneralContact functions
class VisuGeneralContact:
    @overload
    def Reset(self) -> None: ...

#stub information for class GeneralContact functions
class GeneralContact:
    @overload
    def GetPythonObject(self) -> dict: ...
    @overload
    def Reset(self, freeMemory=True) -> None: ...
    isActive:bool
    verboseMode:int
    visualization:VisuGeneralContact
    resetSearchTreeInterval:int
    sphereSphereContact:bool
    sphereSphereFrictionRecycle:bool
    minRelDistanceSpheresTriangles:float
    frictionProportionalZone:float
    frictionVelocityPenalty:float
    excludeOverlappingTrigSphereContacts:bool
    excludeDuplicatedTrigSphereContactPoints:bool
    ancfCableUseExactMethod:bool
    ancfCableNumberOfContactSegments:int
    ancfCableMeasuringSegments:int
    @overload
    def SetFrictionPairings(self, frictionPairings: ArrayLike) -> None: ...
    @overload
    def SetFrictionProportionalZone(self, frictionProportionalZone: float) -> None: ...
    @overload
    def SetSearchTreeCellSize(self, numberOfCells: [int,int,int]) -> None: ...
    @overload
    def SetSearchTreeBox(self, pMin: [float,float,float], pMax: [float,float,float]) -> None: ...
    @overload
    def AddSphereWithMarker(self, markerIndex: MarkerIndex, radius: float, contactStiffness: float, contactDamping: float, frictionMaterialIndex: int) -> int: ...
    @overload
    def AddANCFCable(self, objectIndex: ObjectIndex, halfHeight: float, contactStiffness: float, contactDamping: float, frictionMaterialIndex: int) -> int: ...
    @overload
    def AddTrianglesRigidBodyBased(self, rigidBodyMarkerIndex: MarkerIndex, contactStiffness: float, contactDamping: float, frictionMaterialIndex: int, pointList: List[[float,float,float]], triangleList: List[[int,int,int]]) -> int: ...
    @overload
    def GetItemsInBox(self, pMin: [float,float,float], pMax: [float,float,float]) -> Union[dict,bool]: ...
    @overload
    def GetMarkerBasedSphere(self, localIndex: int) -> dict: ...
    @overload
    def ShortestDistanceAlongLine(self, pStart: [float,float,float], direction: [float,float,float], minDistance: float, maxDistance: float, asDictionary: bool, cylinderRadius: float, typeIndex: ContactTypeIndex) -> Union[dict,float]: ...
    @overload
    def UpdateContacts(self, mainSystem: MainSystem) -> None: ...
    @overload
    def GetActiveContacts(self, typeIndex: ContactTypeIndex, itemIndex: int) -> List[int]: ...

#stub information for class SystemData functions
class SystemData:
    @overload
    def NumberOfLoads(self) -> int: ...
    @overload
    def NumberOfMarkers(self) -> int: ...
    @overload
    def NumberOfNodes(self) -> int: ...
    @overload
    def NumberOfObjects(self) -> int: ...
    @overload
    def NumberOfSensors(self) -> int: ...
    @overload
    def ODE2Size(self, configurationType: ConfigurationType) -> int: ...
    @overload
    def ODE1Size(self, configurationType=ConfigurationType.Current) -> int: ...
    @overload
    def AEsize(self, configurationType=ConfigurationType.Current) -> int: ...
    @overload
    def DataSize(self, configurationType=ConfigurationType.Current) -> int: ...
    @overload
    def SystemSize(self, configurationType=ConfigurationType.Current) -> int: ...
    @overload
    def GetTime(self, configurationType=ConfigurationType.Current) -> float: ...
    @overload
    def SetTime(self, newTime: float, configurationType: ConfigurationType) -> None: ...
    @overload
    def AddODE2LoadDependencies(self, loadNumber: float, globalODE2coordinates: List[int]) -> None: ...
    @overload
    def Info(self) -> None: ...
    @overload
    def InfoLTG(self) -> None: ...
    @overload
    def GetODE2Coordinates(self, configuration: ConfigurationType) -> List[float]: ...
    @overload
    def SetODE2Coordinates(self, coordinates: List[float], configuration: ConfigurationType) -> List[float]: ...
    @overload
    def GetODE2Coordinates_t(self, configuration: ConfigurationType) -> List[float]: ...
    @overload
    def SetODE2Coordinates_t(self, coordinates: List[float], configuration: ConfigurationType) -> List[float]: ...
    @overload
    def GetODE2Coordinates_tt(self, configuration: ConfigurationType) -> List[float]: ...
    @overload
    def SetODE2Coordinates_tt(self, coordinates: List[float], configuration: ConfigurationType) -> List[float]: ...
    @overload
    def GetODE1Coordinates(self, configuration: ConfigurationType) -> List[float]: ...
    @overload
    def SetODE1Coordinates(self, coordinates: List[float], configuration: ConfigurationType) -> List[float]: ...
    @overload
    def GetODE1Coordinates_t(self, configuration: ConfigurationType) -> List[float]: ...
    @overload
    def SetODE1Coordinates_t(self, coordinates: List[float], configuration: ConfigurationType) -> List[float]: ...
    @overload
    def GetAECoordinates(self, configuration: ConfigurationType) -> List[float]: ...
    @overload
    def SetAECoordinates(self, coordinates: List[float], configuration: ConfigurationType) -> List[float]: ...
    @overload
    def GetDataCoordinates(self, configuration: ConfigurationType) -> List[float]: ...
    @overload
    def SetDataCoordinates(self, coordinates: List[float], configuration: ConfigurationType) -> List[float]: ...
    @overload
    def GetSystemState(self, configuration: ConfigurationType) -> List[List[float]]: ...
    @overload
    def SetSystemState(self, systemStateList: List[List[float]], configuration: ConfigurationType) -> List[float]: ...
    @overload
    def GetObjectLTGODE2(self, objectNumber: int) -> List[int]: ...
    @overload
    def GetObjectLTGODE1(self, objectNumber: int) -> List[int]: ...
    @overload
    def GetObjectLTGAE(self, objectNumber: int) -> List[int]: ...
    @overload
    def GetObjectLTGData(self, objectNumber: int) -> List[int]: ...
    @overload
    def GetNodeLTGODE2(self, nodeNumber: int) -> List[int]: ...
    @overload
    def GetNodeLTGODE1(self, nodeNumber: int) -> List[int]: ...
    @overload
    def GetNodeLTGAE(self, nodeNumber: int) -> List[int]: ...
    @overload
    def GetNodeLTGData(self, nodeNumber: int) -> List[int]: ...

#stub information for class MainSystem functions
class MainSystem:
    @overload
    def Assemble(self) -> None: ...
    @overload
    def AssembleCoordinates(self) -> None: ...
    @overload
    def AssembleLTGLists(self) -> None: ...
    @overload
    def AssembleInitializeSystemCoordinates(self) -> None: ...
    @overload
    def AssembleSystemInitialize(self) -> None: ...
    @overload
    def Reset(self) -> None: ...
    @overload
    def GetSystemContainer(self) -> SystemContainer: ...
    @overload
    def WaitForUserToContinue(self, printMessage=True) -> None: ...
    @overload
    def SendRedrawSignal(self) -> None: ...
    @overload
    def GetRenderEngineStopFlag(self) -> bool: ...
    @overload
    def SetRenderEngineStopFlag(self, stopFlag: bool) -> None: ...
    @overload
    def ActivateRendering(self, flag=True) -> None: ...
    @overload
    def SetPreStepUserFunction(self, value: Callable[[MainSystem, float],bool]) -> None: ...
    @overload
    def SetPostNewtonUserFunction(self, value: Callable[[MainSystem, float],[float,float]]) -> None: ...
    @overload
    def AddGeneralContact(self) -> GeneralContact: ...
    @overload
    def GetGeneralContact(self, generalContactNumber: int) -> GeneralContact: ...
    @overload
    def DeleteGeneralContact(self, generalContactNumber: int) -> None: ...
    @overload
    def NumberOfGeneralContacts(self) -> int: ...
    systemIsConsistent:bool
    interactiveMode:bool
    variables:dict
    sys:dict
    solverSignalJacobianUpdate:bool
    systemData:SystemData
    @overload
    def AddNode(self, pyObject: dict) -> NodeIndex: ...
    @overload
    def GetNodeNumber(self, nodeName: str) -> NodeIndex: ...
    @overload
    def GetNode(self, nodeNumber: NodeIndex) -> dict: ...
    @overload
    def ModifyNode(self, nodeNumber: NodeIndex, nodeDict: dict) -> None: ...
    @overload
    def GetNodeDefaults(self, typeName: str) -> dict: ...
    @overload
    def GetNodeOutput(self, nodeNumber: NodeIndex, variableType: OutputVariableType, configuration: ConfigurationType) -> List[float]: ...
    @overload
    def GetNodeODE2Index(self, nodeNumber: NodeIndex) -> int: ...
    @overload
    def GetNodeODE1Index(self, nodeNumber: NodeIndex) -> int: ...
    @overload
    def GetNodeAEIndex(self, nodeNumber: NodeIndex) -> int: ...
    @overload
    def GetNodeParameter(self, nodeNumber: NodeIndex, parameterName: str) -> Any: ...
    @overload
    def SetNodeParameter(self, nodeNumber: NodeIndex, parameterName: str, value: Any) -> None: ...
    @overload
    def AddObject(self, pyObject: dict) -> ObjectIndex: ...
    @overload
    def GetObjectNumber(self, objectName: str) -> ObjectIndex: ...
    @overload
    def GetObject(self, objectNumber: ObjectIndex, addGraphicsData=False) -> dict: ...
    @overload
    def ModifyObject(self, objectNumber: ObjectIndex, objectDict: dict) -> None: ...
    @overload
    def GetObjectDefaults(self, typeName: str) -> dict: ...
    @overload
    def GetObjectOutput(self, objectNumber: ObjectIndex, variableType: OutputVariableType, configuration: ConfigurationType) -> List[float]: ...
    @overload
    def GetObjectOutputBody(self, objectNumber: ObjectIndex, variableType: OutputVariableType, localPosition: [float,float,float], configuration: ConfigurationType) -> List[float]: ...
    @overload
    def GetObjectOutputSuperElement(self, objectNumber: ObjectIndex, variableType: OutputVariableType, meshNodeNumber: int, configuration: ConfigurationType) -> List[float]: ...
    @overload
    def GetObjectParameter(self, objectNumber: ObjectIndex, parameterName: str) -> Any: ...
    @overload
    def SetObjectParameter(self, objectNumber: ObjectIndex, parameterName: str, value: Any) -> None: ...
    @overload
    def AddMarker(self, pyObject: dict) -> MarkerIndex: ...
    @overload
    def GetMarkerNumber(self, markerName: str) -> MarkerIndex: ...
    @overload
    def GetMarker(self, markerNumber: MarkerIndex) -> dict: ...
    @overload
    def ModifyMarker(self, markerNumber: MarkerIndex, markerDict: dict) -> None: ...
    @overload
    def GetMarkerDefaults(self, typeName: str) -> dict: ...
    @overload
    def GetMarkerParameter(self, markerNumber: MarkerIndex, parameterName: str) -> Any: ...
    @overload
    def SetMarkerParameter(self, markerNumber: MarkerIndex, parameterName: str, value: Any) -> None: ...
    @overload
    def GetMarkerOutput(self, markerNumber: MarkerIndex, variableType: OutputVariableType, configuration: ConfigurationType) -> List[float]: ...
    @overload
    def AddLoad(self, pyObject: dict) -> LoadIndex: ...
    @overload
    def GetLoadNumber(self, loadName: str) -> LoadIndex: ...
    @overload
    def GetLoad(self, loadNumber: LoadIndex) -> dict: ...
    @overload
    def ModifyLoad(self, loadNumber: LoadIndex, loadDict: dict) -> None: ...
    @overload
    def GetLoadDefaults(self, typeName: str) -> dict: ...
    @overload
    def GetLoadValues(self, loadNumber: LoadIndex) -> List[float]: ...
    @overload
    def GetLoadParameter(self, loadNumber: LoadIndex, parameterName: str) -> Any: ...
    @overload
    def SetLoadParameter(self, loadNumber: LoadIndex, parameterName: str, value: Any) -> None: ...
    @overload
    def AddSensor(self, pyObject: dict) -> SensorIndex: ...
    @overload
    def GetSensorNumber(self, sensorName: str) -> SensorIndex: ...
    @overload
    def GetSensor(self, sensorNumber: SensorIndex) -> dict: ...
    @overload
    def ModifySensor(self, sensorNumber: SensorIndex, sensorDict: dict) -> None: ...
    @overload
    def GetSensorDefaults(self, typeName: str) -> dict: ...
    @overload
    def GetSensorValues(self, sensorNumber: SensorIndex, configuration: ConfigurationType) -> List[float]: ...
    @overload
    def GetSensorStoredData(self, sensorNumber: SensorIndex) -> ArrayLike: ...
    @overload
    def GetSensorParameter(self, sensorNumber: SensorIndex, parameterName: str) -> Any: ...
    @overload
    def SetSensorParameter(self, sensorNumber: SensorIndex, parameterName: str, value: Any) -> None: ...

#stub information for class SystemContainer functions
class SystemContainer:
    @overload
    def Reset(self) -> None: ...
    @overload
    def AddSystem(self) -> MainSystem: ...
    @overload
    def NumberOfSystems(self) -> int: ...
    @overload
    def GetSystem(self, systemNumber: int) -> MainSystem: ...
    visualizationSettings:VisualizationSettings
    @overload
    def GetRenderState(self) -> dict: ...
    @overload
    def SetRenderState(self, renderState: dict) -> None: ...
    @overload
    def RedrawAndSaveImage(self) -> None: ...
    @overload
    def WaitForRenderEngineStopFlag(self) -> bool: ...
    @overload
    def RenderEngineZoomAll(self) -> None: ...
    @overload
    def AttachToRenderEngine(self) -> bool: ...
    @overload
    def DetachFromRenderEngine(self) -> bool: ...
    @overload
    def SendRedrawSignal(self) -> None: ...
    @overload
    def GetCurrentMouseCoordinates(self, useOpenGLcoordinates=False) -> [float,float]: ...

#stub information for exudyn module functions
@overload
def GetVersionString(addDetails=False) -> str: ...
@overload
def Help() -> None: ...
@overload
def RequireVersion(requiredVersionString: str) -> None: ...
@overload
def StartRenderer(verbose=0) -> bool: ...
@overload
def IsRendererActive() -> bool: ...
@overload
def DoRendererIdleTasks(waitSeconds=0) -> None: ...
@overload
def SolveStatic(mbs: MainSystem, simulationSettings: SimulationSettings, updateInitialValues=False, storeSolver=True) -> bool: ...
@overload
def SolveDynamic(mbs: MainSystem, simulationSettings: SimulationSettings, solverType: DynamicSolverType, updateInitialValues=False, storeSolver=True) -> bool: ...
@overload
def ComputeODE2Eigenvalues(mbs: MainSystem, simulationSettings: SimulationSettings, useSparseSolver=False, numberOfEigenvalues=-1, setInitialValues=True, convert2Frequencies=False) -> bool: ...
@overload
def SetOutputPrecision(numberOfDigits: int) -> None: ...
@overload
def SetLinalgOutputFormatPython(flagPythonFormat: bool) -> None: ...
@overload
def SetWriteToConsole(flag: bool) -> None: ...
@overload
def SetWriteToFile(filename: str, flagWriteToFile=True, flagAppend=False) -> None: ...
@overload
def SetPrintDelayMilliSeconds(delayMilliSeconds: int) -> None: ...
@overload
def Print(*args: Any) -> None: ...
@overload
def SuppressWarnings(flag: bool) -> None: ...
@overload
def InfoStat(writeOutput=True) -> List[int]: ...
@overload
def Go() -> None: ...
@overload
def InvalidIndex() -> int: ...
variables:dict
sys:dict

class MainSystem:
    @overload
    def SolutionViewer(self, solution=[], rowIncrement=1, timeout=0.04, runOnStart=True, runMode=2, fontSize=12, title='', checkRenderEngineStopFlag=True) -> None: ...

    @overload
    def CreateMassPoint(self, name='', referenceCoordinates=[0.,0.,0.], initialCoordinates=[0.,0.,0.], initialVelocities=[0.,0.,0.], physicsMass=0, gravity=[0.,0.,0.], graphicsDataList=[], drawSize=-1, color=[-1.,-1.,-1.,-1.], show=True, create2D=False, returnDict=False) -> Union[dict, ObjectIndex]: ...

    @overload
    def CreateRigidBody(self, name='', referencePosition=[0.,0.,0.], referenceRotationMatrix=np.eye(3), initialVelocity=[0.,0.,0.], initialAngularVelocity=[0.,0.,0.], initialDisplacement=None, initialRotationMatrix=None, inertia=None, gravity=[0.,0.,0.], nodeType=exudyn.NodeType.RotationEulerParameters, graphicsDataList=[], drawSize=-1, color=[-1.,-1.,-1.,-1.], show=True, create2D=False, returnDict=False) -> Union[dict, ObjectIndex]: ...

    @overload
    def CreateSpringDamper(self, name='', bodyOrNodeList=[None, None], localPosition0=[0.,0.,0.], localPosition1=[0.,0.,0.], referenceLength=None, stiffness=0., damping=0., force=0., velocityOffset=0., show=True, drawSize=-1, color=color4default) -> ObjectIndex: ...

    @overload
    def CreateCartesianSpringDamper(self, name='', bodyOrNodeList=[None, None], localPosition0=[0.,0.,0.], localPosition1=[0.,0.,0.], stiffness=[0.,0.,0.], damping=[0.,0.,0.], offset=[0.,0.,0.], show=True, drawSize=-1, color=color4default) -> ObjectIndex: ...

    @overload
    def CreateRevoluteJoint(self, name='', bodyNumbers=[None, None], position=[], axis=[], useGlobalFrame=True, show=True, axisRadius=0.1, axisLength=0.4, color=color4default) -> [ObjectIndex, MarkerIndex, MarkerIndex]: ...

    @overload
    def CreatePrismaticJoint(self, name='', bodyNumbers=[None, None], position=[], axis=[], useGlobalFrame=True, show=True, axisRadius=0.1, axisLength=0.4, color=color4default) -> [ObjectIndex, MarkerIndex, MarkerIndex]: ...

    @overload
    def CreateSphericalJoint(self, name='', bodyNumbers=[None, None], position=[], constrainedAxes=[1,1,1], useGlobalFrame=True, show=True, jointRadius=0.1, color=color4default) -> [ObjectIndex, MarkerIndex, MarkerIndex]: ...

    @overload
    def CreateGenericJoint(self, name='', bodyNumbers=[None, None], position=[], rotationMatrixAxes=np.eye(3), constrainedAxes=[1,1,1, 1,1,1], useGlobalFrame=True, show=True, axesRadius=0.1, axesLength=0.4, color=color4default) -> [ObjectIndex, MarkerIndex, MarkerIndex]: ...

    @overload
    def CreateDistanceConstraint(self, name='', bodyOrNodeList=[None, None], localPosition0=[0.,0.,0.], localPosition1=[0.,0.,0.], distance=None, show=True, drawSize=-1., color=color4default) -> [ObjectIndex, MarkerIndex, MarkerIndex]: ...

    @overload
    def PlotSensor(self, sensorNumbers=[], components=0, xLabel='time (s)', yLabel=None, labels=[], colorCodeOffset=0, newFigure=True, closeAll=False, componentsX=[], title='', figureName='', fontSize=16, colors=[], lineStyles=[], lineWidths=[], markerStyles=[], markerSizes=[], markerDensity=0.08, rangeX=[], rangeY=[], majorTicksX=10, majorTicksY=10, offsets=[], factors=[], subPlot=[], sizeInches=[6.4,4.8], fileName='', useXYZcomponents=True, **kwargs) -> [Any, Any, Any, Any]: ...

    @overload
    def SolveStatic(self, simulationSettings=exudyn.SimulationSettings(), updateInitialValues=False, storeSolver=True, showHints=False, showCausingItems=True) -> bool: ...

    @overload
    def SolveDynamic(self, simulationSettings=exudyn.SimulationSettings(), solverType=exudyn.DynamicSolverType.GeneralizedAlpha, updateInitialValues=False, storeSolver=True, showHints=False, showCausingItems=True) -> bool: ...

    @overload
    def ComputeLinearizedSystem(self, simulationSettings=exudyn.SimulationSettings(), useSparseSolver=False) -> [ArrayLike, ArrayLike, ArrayLike]: ...

    @overload
    def ComputeODE2Eigenvalues(self, simulationSettings=exudyn.SimulationSettings(), useSparseSolver=False, numberOfEigenvalues=0, constrainedCoordinates=[], convert2Frequencies=False, useAbsoluteValues=True) -> [ArrayLike, ArrayLike]: ...

    @overload
    def ComputeSystemDegreeOfFreedom(self, simulationSettings=exudyn.SimulationSettings(), threshold=1e-12, verbose=False, useSVD=False) -> List[int]: ...

    @overload
    def CreateDistanceSensorGeometry(self, meshPoints, meshTrigs, rigidBodyMarkerIndex, searchTreeCellSize=[8,8,8]) -> int: ...

    @overload
    def CreateDistanceSensor(self, generalContactIndex, positionOrMarker, dirSensor, minDistance=-1e7, maxDistance=1e7, cylinderRadius=0, selectedTypeIndex=exudyn.ContactTypeIndex.IndexEndOfEnumList, storeInternal=False, fileName='', measureVelocity=False, addGraphicsObject=False, drawDisplaced=True, color=color4red) -> SensorIndex: ...

    @overload
    def DrawSystemGraph(self, showLoads=True, showSensors=True, useItemNames=False, useItemTypes=False, addItemTypeNames=True, multiLine=True, fontSizeFactor=1., layoutDistanceFactor=3., layoutIterations=100, showLegend=True) -> [Any, Any, Any]: ...

