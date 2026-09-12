
class OutputVariableType(Enum):
    """The enumeration type  OutputVariableType is used for selecting output values, e.g., for GetObjectOutput(...) or for selecting variables for contour plot.

Available output variables and the interpreation of the output variable can be found at the object definitions.

    The OutputVariableType does not provide information about the size of the output variable, which can be either scalar or a list (vector)
    For vector output quantities, the contour plot option offers an additional parameter for selection of the component of the OutputVariableType
    The components are usually out of {0,1,2}, representing {x,y,z} components (e.g., of displacements, velocities, ...), or {0,1,2,3,4,5} representing {xx,yy,zz,yz,xz,xy} components (e.g., of strain or stress)
    In order to compute a norm, chose component=-1, which will result in the quadratic norm for other vectors and to a norm specified for stresses (if no norm is defined for an outputVariable, it does not compute anything)
    """
    _None = int
    """no value; used, e.g., to select no output variable in contour plot"""
    Distance = int
    """e.g., measure distance in spring damper connector"""
    Position = int
    """measure 3D position, e.g., of node or body"""
    Displacement = int
    """measure displacement; usually difference between current position and reference position"""
    DisplacementLocal = int
    """measure local displacement, e.g., in local joint coordinates"""
    Velocity = int
    """measure (translational) velocity of node or object"""
    VelocityLocal = int
    """measure local (translational) velocity, e.g., in local body or joint coordinates"""
    Acceleration = int
    """measure (translational) acceleration of node or object"""
    AccelerationLocal = int
    """measure (translational) acceleration of node or object in local coordinates"""
    RotationMatrix = int
    """measure rotation matrix of rigid body node or object"""
    Rotation = int
    """measure, e.g., scalar rotation of 2D body, Euler angles of a 3D object or rotation within a joint"""
    AngularVelocity = int
    """measure angular velocity of node or object"""
    AngularVelocityLocal = int
    """measure local (body-fixed) angular velocity of node or object"""
    AngularAcceleration = int
    """measure angular acceleration of node or object"""
    AngularAccelerationLocal = int
    """measure angular acceleration of node or object in local coordinates"""
    CoordinatesTotal = int
    """measure the total coordinates (including reference configuration) of a node or object; otherwise the same as Coordinates"""
    Coordinates = int
    """measure the coordinates of a node or object; coordinates just contain displacements, but not the reference (position or rotation) values - see also definition of respective nodes or objects"""
    Coordinates_t = int
    """measure the time derivative of coordinates (= velocity coordinates) of a node or object"""
    Coordinates_tt = int
    """measure the second time derivative of coordinates (= acceleration coordinates) of a node or object"""
    SlidingCoordinate = int
    """measure sliding coordinate in sliding joint"""
    Director1 = int
    """measure a director (e.g., of a rigid body frame), or a slope vector in local 1 or x-direction"""
    Director2 = int
    """measure a director (e.g., of a rigid body frame), or a slope vector in local 2 or y-direction"""
    Director3 = int
    """measure a director (e.g., of a rigid body frame), or a slope vector in local 3 or z-direction"""
    Force = int
    """measure global force, e.g., in joint or beam (resultant force), or generalized forces; see description of according object"""
    ForceLocal = int
    """measure local force, e.g., in joint or beam (resultant force)"""
    Torque = int
    """measure torque, e.g., in joint or beam (resultant couple/moment)"""
    TorqueLocal = int
    """measure local torque, e.g., in joint or beam (resultant couple/moment)"""
    StrainLocal = int
    """measure local strain, e.g., axial strain in cross section frame of beam or Green-Lagrange strain"""
    StressLocal = int
    """measure local stress, e.g., axial stress in cross section frame of beam or Second Piola-Kirchoff stress; choosing component==-1 will result in the computation of the Mises stress"""
    CurvatureLocal = int
    """measure local curvature; may be scalar or vectorial: twist and curvature of beam in cross section frame"""
    ConstraintEquation = int
    """evaluates constraint equation (=current deviation or drift of constraint equation)"""

class ConfigurationType(Enum):
    """The enumeration type  ConfigurationType is used for selecting a configuration for reading or writing information to the module.

    Specifically, the ConfigurationType.Current configuration is usually used at the end of a solution process, to obtain result values, or the ConfigurationType.Initial is used to set initial values for a solution process.
    """
    _None = int
    """no configuration; usually not valid, but may be used, e.g., if no configurationType is required"""
    Initial = int
    """initial configuration prior to static or dynamic solver; is computed during mbs.Assemble() or AssembleInitializeSystemCoordinates()"""
    Current = int
    """current configuration during and at the end of the computation of a step (static or dynamic)"""
    Reference = int
    """configuration used to define deformable bodies (reference configuration for finite elements) or joints (configuration for which some joints are defined)"""
    StartOfStep = int
    """during computation, this refers to the solution at the start of the step = end of last step, to which the solver falls back if convergence fails"""
    Visualization = int
    """this is a state completely de-coupled from computation, used for visualization"""
    EndOfEnumList = int
    """this marks the end of the list, usually not important to the user"""

class ItemType(Enum):
    """The enumeration type  ItemType is used for defining types of indices, e.g., in render window and will be also used in item dictionaries in future."""
    _None = int
    """item has no type"""
    Node = int
    """item or index is of type Node"""
    Object = int
    """item or index is of type Object"""
    Marker = int
    """item or index is of type Marker"""
    Load = int
    """item or index is of type Load"""
    Sensor = int
    """item or index is of type Sensor"""

class NodeType(Enum):
    """The enumeration type  NodeType is used for defining node types for 3D rigid bodies."""
    _None = int
    """node has no type"""
    Ground = int
    """ground node"""
    Position2D = int
    """2D position node"""
    Orientation2D = int
    """node with 2D rotation"""
    Point2DSlope1 = int
    """2D node with 1 slope vector"""
    Position = int
    """3D position node"""
    Orientation = int
    """3D orientation node"""
    RigidBody = int
    """node that can be used for rigid bodies"""
    RotationEulerParameters = int
    """node with 3D orientations that are modelled with Euler parameters (unit quaternions)"""
    RotationRxyz = int
    """node with 3D orientations that are modelled with Tait-Bryan angles"""
    RotationRotationVector = int
    """node with 3D orientations that are modelled with the rotation vector"""
    LieGroupWithDirectUpdate = int
    """node to be solved with Lie group methods, without data coordinates"""
    GenericODE2 = int
    """node with general ODE2 variables"""
    GenericODE1 = int
    """node with general ODE1 variables"""
    GenericAE = int
    """node with general algebraic variables"""
    GenericData = int
    """node with general data variables"""
    PointSlope1 = int
    """node with 1 slope vector"""
    PointSlope12 = int
    """node with 2 slope vectors in x and y direction"""
    PointSlope23 = int
    """node with 2 slope vectors in y and z direction"""

class JointType(Enum):
    """The enumeration type  JointType is used for defining joint types, used in KinematicTree."""
    _None = int
    """node has no type"""
    RevoluteX = int
    """revolute joint type with rotation around local X axis"""
    RevoluteY = int
    """revolute joint type with rotation around local Y axis"""
    RevoluteZ = int
    """revolute joint type with rotation around local Z axis"""
    PrismaticX = int
    """prismatic joint type with translation along local X axis"""
    PrismaticY = int
    """prismatic joint type with translation along local Y axis"""
    PrismaticZ = int
    """prismatic joint type with translation along local Z axis"""

class DynamicSolverType(Enum):
    """The enumeration type  DynamicSolverType is used for selecting dynamic solvers for simulation."""
    GeneralizedAlpha = int
    """an implicit solver for index 3 problems; intended to be used for solving directly the index 3 constraints using the spectralRadius sufficiently small (usually 0.5 .. 1)"""
    TrapezoidalIndex2 = int
    """an implicit solver for index 3 problems with index2 reduction; uses generalized alpha solver with settings for Newmark with index2 reduction"""
    ExplicitEuler = int
    """an explicit 1st order solver (generally not compatible with constraints)"""
    ExplicitMidpoint = int
    """an explicit 2nd order solver (generally not compatible with constraints)"""
    RK33 = int
    """an explicit 3 stage 3rd order Runge-Kutta method, aka 'Heun third order'; (generally not compatible with constraints)"""
    RK44 = int
    """an explicit 4 stage 4th order Runge-Kutta method, aka 'classical Runge Kutta' (generally not compatible with constraints), compatible with Lie group integration and elimination of CoordinateConstraints"""
    RK67 = int
    """an explicit 7 stage 6th order Runge-Kutta method, see 'On Runge-Kutta Processes of High Order', J. C. Butcher, J. Austr Math Soc 4, (1964); can be used for very accurate (reference) solutions, but without step size control!"""
    ODE23 = int
    """an explicit Runge Kutta method with automatic step size selection with 3rd order of accuracy and 2nd order error estimation, see Bogacki and Shampine, 1989; also known as ODE23 in MATLAB"""
    DOPRI5 = int
    """an explicit Runge Kutta method with automatic step size selection with 5th order of accuracy and 4th order error estimation, see  Dormand and Prince, 'A Family of Embedded Runge-Kutta Formulae.', J. Comp. Appl. Math. 6, 1980"""
    DVERK6 = int
    """[NOT IMPLEMENTED YET] an explicit Runge Kutta solver of 6th order with 5th order error estimation; includes adaptive step selection"""
    VelocityVerlet = int
    """[TEST phase] a special explicit time integration scheme, the 'velocity Verlet' method (similar to leap frog method), with second order accuracy for conservative second order differential equations, often used for particle dynamics and contact; implementation uses Explicit Euler for ODE1 equations"""

class CrossSectionType(Enum):
    """The enumeration type  CrossSectionType is used for defining beam cross section types."""
    Polygon = int
    """cross section profile defined by polygon"""
    Circular = int
    """cross section is circle or elliptic"""

class KeyCode(Enum):
    """The enumeration type  KeyCode is used for special key codes in keyPressUserFunction."""
    SPACE = int
    """space key"""
    ENTER = int
    """enter (return) key"""
    TAB = int
    """"""
    BACKSPACE = int
    """"""
    RIGHT = int
    """cursor right"""
    LEFT = int
    """cursor left"""
    DOWN = int
    """cursor down"""
    UP = int
    """cursor up"""
    F1 = int
    """function key F1"""
    F2 = int
    """function key F2"""
    F3 = int
    """function key F3"""
    F4 = int
    """function key F4"""
    F5 = int
    """function key F5"""
    F6 = int
    """function key F6"""
    F7 = int
    """function key F7"""
    F8 = int
    """function key F8"""
    F9 = int
    """function key F9"""
    F10 = int
    """function key F10"""

class LinearSolverType(Enum):
    """The enumeration type  LinearSolverType is used for selecting linear solver types, which are dense or sparse solvers."""
    _None = int
    """no value; used, e.g., if no solver is selected"""
    EXUdense = int
    """use dense matrices and according solvers for densly populated matrices (usually the CPU time grows cubically with the number of unknowns)"""
    EigenSparse = int
    """use sparse matrices and according solvers; additional overhead for very small multibody systems; specifically, memory allocation is performed during a factorization process"""
    EigenSparseSymmetric = int
    """use sparse matrices and according solvers; NOTE: this is the symmetric mode, which assumes symmetric system matrices; this is EXPERIMENTAL and should only be used of user knows that the system matrices are (nearly) symmetric; does not work with scaled GeneralizedAlpha matrices; does not work with constraints, as it must be symmetric positive definite"""
    EigenDense = int
    """use Eigen's LU factorization with partial pivoting (faster than EXUdense) or full pivot (if linearSolverSettings.ignoreSingularJacobian=True; is much slower, but can resolve overdetermined and underdetermined problems!)"""

class ContactTypeIndex(Enum):
    """The enumeration type  ContactTypeIndex is used in GeneralContact to select specific contact items, such as spheres, ANCFCable or triangle items."""
    IndexSpheresMarkerBased = int
    """spheres attached to markers"""
    IndexANCFCable2D = int
    """ANCFCable2D contact items"""
    IndexTrigsRigidBodyBased = int
    """triangles attached to rigid body (or rigid body marker)"""
    IndexEndOfEnumList = int
    """signals end of list"""

#stub information for class MatrixContainer functions
class MatrixContainer:
    """The MatrixContainer is a versatile representation for dense and sparse matrices.

    NOTE: if the MatrixContainer is constructed from a numpy array or a list of lists, both representing a dense matrix, it will go into dense mode; if it is initialized with a scipy sparse csr matrix, it will go into sparse mode
    Examples:
    """
    @overload
    def Initialize(self, numberOfRows: int, numberOfColumns: int, useDenseMatrix: bool=True) -> None: 
        """Initialize MatrixContainer with number of rows and columns and set dense/sparse mode."""
        ...
    @overload
    def SetWithDenseMatrix(self, pyArray, useDenseMatrix=False, factor=1.) -> None: 
        """Set MatrixContainer with dense numpy array of size (n x m); array (=matrix) contains values and matrix size information; if useDenseMatrix=True, matrix will be stored internally as dense matrix, otherwise it will be converted and stored as sparse matrix (which may speed up computations for larger problems); pyArray is multiplied with given factor."""
        ...
    @overload
    def SetWithSparseMatrix(self, sparseMatrix: Any, numberOfRows: int=exudyn.InvalidIndex(), numberOfColumns: int=exudyn.InvalidIndex(), useDenseMatrix: bool=False, factor: float=1.) -> None: 
        """Set with scipy sparse csr_matrix (NOT: csc_matrix!) or with internal sparse triplet format (denoted as CSR): 'sparseMatrix' either contains a scipy matrix create with csr_matrix or a list of lists of sparse triplets (row, col, value) or the list of lists converted into numpy array; numberOfRowsInit and numberOfColumnsInit denote the size of the matrices, which are ignored in case of a scipy sparse matrix; if useDenseMatrix=True, matrix will be converted and stored internally as dense matrix, otherwise it will be stored as sparse matrix triplets; the values of sparseMatrix are multiplied with the given factor before storing."""
        ...
    @overload
    def AddSparseMatrix(self, sparseMatrix: Any, factor: float=1.) -> None: 
        """Add scipy sparse csr_matrix with factor to already initilized MatrixContainer; sparseMatrix must contain according scipy csr format, otherwise the behavior is undefined! This function allows to efficiently add submatrices to the MatrixContainer."""
        ...
    @overload
    def GetPythonObject(self) -> Union[dict,ArrayLike]: 
        """Convert MatrixContainer to numpy array (dense) or dictionary (sparse): containing nr.
        
        of rows, nr. of columns, numpy matrix with sparse triplets
        """
        ...
    @overload
    def Convert2DenseMatrix(self) -> ArrayLike: 
        """Convert MatrixContainer to dense numpy array (SLOW and may fail for too large sparse matrices)."""
        ...
    @overload
    def UseDenseMatrix(self) -> bool: 
        """Returns True if dense matrix is used, otherwise False."""
        ...
    @overload
    def SetAllZero(self) -> None: 
        """Set all values to zero; dense mode: set all matrix entries to zero (slow); sparse mode: set number of triplets to zero (fast)."""
        ...
    @overload
    def SetWithSparseMatrixCSR(self, numberOfRowsInit: int, numberOfColumnsInit: int, pyArrayCSR: Any, useDenseMatrix: bool=False, factor: float=1.) -> None: 
        """DEPRECATED: set with sparse CSR matrix format: numpy array 'pyArrayCSR' contains sparse triplet (row, col, value) per row; numberOfRows and numberOfColumns given extra; if useDenseMatrix=True, matrix will be converted and stored internally as dense matrix, otherwise it will be stored as sparse matrix; the values of pyArrayCSR are multiplied by the given factor."""
        ...

#stub information for class GraphicsMaterialList functions
class GraphicsMaterialList:
    """The GraphicsMaterialList contains the list of materials (material properties) for visualization; currently, only the raytracer uses materials.

    Materials can be accessed via the variable materials in renderer of SystemContainer.
    """
    @overload
    def Reset(self) -> None: 
        """Reset materials to 10 default materials."""
        ...
    @overload
    def Append(self, material: Any) -> int: 
        """Add single material as dict or VSettingsMaterial to list; returns index of newly added material."""
        ...
    @overload
    def New(self) -> VSettingsMaterial: 
        """Get new default material, which can be modified or appended to materials list."""
        ...
    @overload
    def Set(self, indexOrName: int, material: Any) -> None: 
        """Set material with index 'materialIndex' as dict or VSettingsMaterial."""
        ...
    @overload
    def Get(self, indexOrName: int) -> None: 
        """Get material with index 'materialIndex' as VSettingsMaterial."""
        ...
    @overload
    def GetDict(self, indexOrName: int) -> None: 
        """Get material with index 'materialIndex' as dict."""
        ...

#stub information for class Vector3DList functions
class Vector3DList:
    """The Vector3DList is used to represent lists of 3D vectors.

    This is used to transfer such lists from Python to C++
      Usage:
    bi
      item Create empty ``Vector3DList`` with ``x = Vector3DList()`` 
      item Create ``Vector3DList`` with list of numpy arrays:``x = Vector3DList([ numpy.array([1.,2.,3.]), numpy.array([4.,5.,6.]) ])``
      item Create ``Vector3DList`` with list of lists ``x = Vector3DList([[1.,2.,3.], [4.,5.,6.]])``
      item Append item: ``x.Append([0.,2.,4.])``
      item Convert into list of numpy arrays: ``x.GetPythonObject()``
    ei
    """
    @overload
    def Append(self, pyArray: [float,float,float]) -> None: 
        """Add single array or list to Vector3DList; array or list must have appropriate dimension!."""
        ...
    @overload
    def GetPythonObject(self) -> List[[float,float,float]]: 
        """Convert Vector3DList into (copied) list of numpy arrays."""
        ...

#stub information for class Vector2DList functions
class Vector2DList:
    """The Vector2DList is used to represent lists of 2D vectors.

    This is used to transfer such lists from Python to C++
      Usage: bi
      item Create empty ``Vector2DList`` with ``x = Vector2DList()`` 
      item Create ``Vector2DList`` with list of numpy arrays:``x = Vector2DList([ numpy.array([1.,2.]), numpy.array([4.,5.]) ])``
      item Create ``Vector2DList`` with list of lists ``x = Vector2DList([[1.,2.], [4.,5.]])``
      item Append item: ``x.Append([0.,2.])``
      item Convert into list of numpy arrays: ``x.GetPythonObject()``
      item similar to Vector3DList !
    ei
    """
    @overload
    def Append(self, pyArray: [float,float]) -> None: 
        """Add single array or list to Vector2DList; array or list must have appropriate dimension!."""
        ...
    @overload
    def GetPythonObject(self) -> List[[float,float]]: 
        """Convert Vector2DList into (copied) list of numpy arrays."""
        ...

#stub information for class Vector6DList functions
class Vector6DList:
    """The Vector6DList is used to represent lists of 6D vectors.

    This is used to transfer such lists from Python to C++
      Usage: bi
      item Create empty ``Vector6DList`` with ``x = Vector6DList()`` 
      item Convert into list of numpy arrays: ``x.GetPythonObject()``
      item similar to Vector3DList !
    ei
    """
    @overload
    def Append(self, pyArray: [float,float,float,float,float,float]) -> None: 
        """Add single array or list to Vector6DList; array or list must have appropriate dimension!."""
        ...
    @overload
    def GetPythonObject(self) -> List[[float,float,float,float,float,float]]: 
        """Convert Vector6DList into (copied) list of numpy arrays."""
        ...

#stub information for class Matrix3DList functions
class Matrix3DList:
    """The Matrix3DList is used to represent lists of 3D Matrices.

    
    This is used to transfer such lists from Python to C++
      Usage: bi
      item Create empty ``Matrix3DList`` with ``x = Matrix3DList()`` 
      item Create ``Matrix3DList`` with list of numpy arrays:``x = Matrix3DList([ numpy.eye(3), numpy.array([[1.,2.,3.],[4.,5.,6.],[7.,8.,9.]]) ])``
      item Create ``Matrix3DList`` with one matrix ``x = Matrix3DList(13.*numpy.eye(3))`` 
      item Append item: ``x.Append(numpy.eye(3))``
      item Convert into list of numpy arrays: ``x.GetPythonObject()``
      item similar to Vector3DList !
    ei
    """
    @overload
    def Append(self, pyArray: NDArray[Shape2D[3,3], float]) -> None: 
        """Add single 3D array or list of lists to Matrix3DList; array or lists must have appropriate dimension!."""
        ...
    @overload
    def GetPythonObject(self) -> List[NDArray[Shape2D[3,3], float]]: 
        """Convert Matrix3DList into (copied) list of 3x3 numpy arrays."""
        ...

#stub information for class Matrix6DList functions
class Matrix6DList:
    """The Matrix6DList is used to represent lists of 6D Matrices.

    
    This is used to transfer such lists from Python to C++
      Usage: bi
      item Create empty ``Matrix6DList`` with ``x = Matrix6DList()`` 
      item Create ``Matrix6DList`` with list of numpy arrays:``x = Matrix6DList([ numpy.eye(6), 2*numpy.eye(6) ])``
      item Append item: ``x.Append(numpy.eye(6))``
      item Convert into list of numpy arrays: ``x.GetPythonObject()``
      item similar to Matrix3DList !
    ei
    """
    @overload
    def Append(self, pyArray: NDArray[Shape2D[6,6], float]) -> None: 
        """Add single 6D array or list of lists to Matrix6DList; array or lists must have appropriate dimension!."""
        ...
    @overload
    def GetPythonObject(self) -> List[NDArray[Shape2D[6,6], float]]: 
        """Convert Matrix6DList into (copied) list of 6x6 numpy arrays."""
        ...
