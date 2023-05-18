
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
