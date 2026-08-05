

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



T1 = TypeVar("T1", bound=int)
T2 = TypeVar("T2", bound=int)

Shape = Tuple
Shape1D = Shape[T1]
Shape2D = Shape[T1, T2]



import exudyn
from exudyn import (ObjectIndex, NodeIndex, MarkerIndex, LoadIndex, SensorIndex,
                    VSettingsMaterial)





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



@overload
def Help() -> None: 
    """Show basic help information."""
    ...
@overload
def RequireVersion(requiredVersionString: str) -> None: 
    r"""Checks if the installed version is according to the required version.
    
    Major, micro and minor version must agree the required level. This function is defined in the \texttt{__init__.py} file
    
    Examples:
        exu.RequireVersion("1.0.31")
    """
    ...
@overload
def SetWriteToFile(filename: str, flagWriteToFile=True, flagAppend=False, flagFlushAlways=False) -> None: 
    r"""Set flag to write (True) or not write to console; default value of flagWriteToFile = False; flagAppend appends output to file, if set True; in order to finalize the file, write \texttt{exu.SetWriteToFile('', False)} to close the output file; in case of flagFlushAlways=True, file will be finalized immediately in every print command, but may be slower;.
    
    Examples:
        exudyn.config.printToConsole = False #no output to console
        exu.SetWriteToFile(filename='testOutput.log', flagWriteToFile=True, flagAppend=False, flagFlushAlways=False)
        exu.Print('print this to file')
        exu.SetWriteToFile('', False) #terminate writing to file which closes the file
    """
    ...
@overload
def Print() -> None: 
    """This allows printing via exudyn with similar syntax as in Python print(args) except for keyword arguments: exu.Print('test=',42,sep=' ',end='',flush=True); allows to redirect all output to file given by SetWriteToFile(...); does not print to console in case that exudyn.config.printToConsole eis set to False."""
    ...
@overload
def InvalidIndex() -> int: 
    """This function provides the invalid index, which may depend on the kind of 32-bit, 64-bit signed or unsigned integer; e.g., node index or item index in list; currently, the InvalidIndex() gives -1, but it may be changed in future versions, therefore you should use this function."""
    ...
__version__:str
"""contains the current version of the Exudyn package."""
variables:dict
"""this dictionary may be used by the user to store exudyn-wide data in order to avoid global Python variables; usage: exu.variables['myvar'] = 42; can be used in particular to exchange data between different mbs or between packages by importing exudyn.variables wherever needed."""
sys:dict
"""this dictionary is used and reserved by the system, e.g., for testsuite, graphics or system function to store module-wide data in order to avoid global Python variables; the variable exu.sys['renderState'] contains the last render state after SC.renderer.Stop() and can be used for subsequent simulations."""


class OutputVariableType(Enum):
    """The enumeration type  OutputVariableType is used for selecting output values, e.g., for GetObjectOutput(...) or for selecting variables for contour plot.


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


class VSettingsShells:
    """Visualization settings for plate/shell finite elements."""
    drawSolid: bool
    """if true: to draw plates/shells as 3D objects; false: only the element surface is drawn; equivalent to crossSectionFilled in beams."""
    thicknessFactor: float
    """a factor multiplied with the thickness of shells/plates only for visualization (e.g. to make some effects more visible)."""


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


class VSettingsView:
    """Settings for view including camera, scene, window, and advanced options to setup a view or view window."""
    camera: VSettingsCamera
    """settings for camera like perspective, marker tracking or clipping plane."""
    scene: VSettingsScene
    """settings which change scene representation, showing edges, faces or world basis."""
    window: VSettingsWindow
    """visualization settings for window that are individual to each view."""


class VSettingsWindowDeprecated:
    """OpenGL Window and interaction settings for visualization; handle changes with care, as they might lead to unexpected results or crashes."""


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


class VisuGeneralContact:
    """This structure may contains some visualization parameters in future.

    Currently, all visualization settings are controlled via SC.visualizationSettings
    """
    @overload
    def Reset(self) -> None: 
        """Reset visualization parameters to default values."""
        ...


class GeneralContact:
    """Structure to define general and highly efficient contact functionality in multibody systems, allowing millions of particles, using search trees and parallelized contact computations, mainly intended for explicit solvers."""
    @overload
    def GetPythonObject(self) -> dict: 
        """Convert member variables of GeneralContact into dictionary; use this for debug only!."""
        ...
    @overload
    def Reset(self, freeMemory=True) -> None: 
        """Remove all contact objects and reset contact parameters."""
        ...
    isActive:bool
    """default = True (compute contact); if isActive=False, no contact computation is performed for this contact set."""
    verboseMode:int
    """default = 0; verboseMode = 1 or higher outputs useful information on the contact creation and computation."""
    visualization:VisuGeneralContact
    """access visualization data structure."""
    resetSearchTreeInterval:int
    """(default=10000) number of search tree updates (contact computation steps) after which the search tree cells are re-created; this costs some time, will free memory in cells that are not needed any more."""
    sphereSphereContact:bool
    """activate/deactivate contact between spheres."""
    sphereSphereFrictionRecycle:bool
    """False: compute static friction force based on tangential velocity; True: recycle friction from previous PostNewton step, which greatly improves convergence, but may lead to unphysical artifacts; will be solved in future by step reduction."""
    minRelDistanceSpheresTriangles:float
    """(default=1e-10) tolerance (relative to sphere radiues) below which the contact between triangles and spheres is ignored; used for spheres directly attached to triangles."""
    frictionProportionalZone:float
    r"""(default=0.001) velocity :math:`v_{\mu,reg}` upon which the dry friction coefficient is interpolated linearly (regularized friction model); must be greater 0; very small values cause oscillations in friction force."""
    excludeOverlappingTrigSphereContacts:bool
    """(default=True) for consistent, closed meshes, we can exclude overlapping contact triangles (which would cause holes if mesh is overlapping and not consistent!!!)."""
    excludeDuplicatedTrigSphereContactPoints:bool
    """(default=False) run additional checks for double contacts at edges or vertices, being more accurate but can cause additional costs if many contacts."""
    computeExactStaticTriangleBins:bool
    """(default=True) if True, search tree bins are computed exactly for static triangles while if False, it uses the overall (=very inaccurate) AABB of each triangle in the search tree."""
    computeContactForces:bool
    """(default=False) if True, additional system vector is computed which contains all contact force and torque contributions. In order to recover forces on a single rigid body, the respective LTG-vector has to be used and forces need to be extracted from this system vector; may slow down computations."""
    ancfCableUseExactMethod:bool
    """(default=True) if True, uses exact computation of intersection of 3rd order polynomials and contacting circles."""
    ancfCableNumberOfContactSegments:int
    """(default=1) number of segments to be used in case that ancfCableUseExactMethod=False; maximum number of segments=3."""
    ancfCableMeasuringSegments:int
    """(default=20) number of segments used to approximate geometry for ANCFCable2D elements for measuring with ShortestDistanceAlongLine; with 20 segments the relative error due to approximation as compared to 10 segments usually stays below 1e-8."""
    parallelTaskSplit:int
    """(default=12) general number of tasks per thread (min)."""
    parallelTaskSplitBoundingBoxes:int
    """(default=48) number of tasks per thread for bounding box computations."""
    parallelTaskSplitThreshold:int
    """(default=12) general threshold below which only one task per thread is used."""
    parallelTaskSplitBoundingBoxesThreshold:int
    """(default=400) threshold below which only one task per thread is used, for bounding box computations."""
    @overload
    def SetFrictionPairings(self, frictionPairings: ArrayLike) -> None: 
        """Set Coulomb friction coefficients for pairings of materials (e.g., use material 0,1, then the entries (0,1) and (1,0) define the friction coefficients for this pairing); matrix should be symmetric!.
        
        Examples:
            #set 3 surface friction types, all being 0.1:
            gContact.SetFrictionPairings(0.1*np.ones((3,3)));
        """
        ...
    @overload
    def SetFrictionProportionalZone(self, frictionProportionalZone: float) -> None: 
        """Regularization for friction (m/s); used for all contacts."""
        ...
    @overload
    def SetSearchTreeCellSize(self, numberOfCells: [int,int,int]) -> None: 
        """Set number of cells of search tree (boxed search) in x, y and z direction.
        
        Examples:
            gContact.SetSearchTreeInitSize([10,10,10])
        """
        ...
    @overload
    def SetSearchTreeBox(self, pMin: [float,float,float], pMax: [float,float,float]) -> None: 
        """Set geometric dimensions of searchTreeBox (point with minimum coordinates and point with maximum coordinates); if this box becomes smaller than the effective contact objects, contact computations may slow down significantly.
        
        Examples:
            gContact.SetSearchTreeBox(pMin=[-1,-1,-1],
                pMax=[1,1,1])
        """
        ...
    @overload
    def AddSphereWithMarker(self, markerIndex: MarkerIndex, radius: float, contactStiffness: float, contactDamping: float, frictionMaterialIndex: int) -> int: 
        """Add contact object using a marker (Position or Rigid), radius and contact/friction parameters and return localIndex of the contact item in GeneralContact; frictionMaterialIndex refers to frictionPairings in GeneralContact; contact is possible between spheres (circles in 2D) (if intraSphereContact = True), spheres and triangles and between sphere (=circle) and ANCFCable2D; contactStiffness is computed as serial spring between contacting objects, while damping is computed as a parallel damper."""
        ...
    @overload
    def AddANCFCable(self, objectIndex: ObjectIndex, halfHeight: float, contactStiffness: float, contactDamping: float, frictionMaterialIndex: int) -> int: 
        """Add contact object for an ANCF cable element, using the objectIndex of the cable element and the cable's half height as an additional distance to contacting objects (currently not causing additional torque in case of friction), and return localIndex of the contact item in GeneralContact; currently only contact with spheres (circles in 2D) possible; contact computed using exact geometry of elements, finding max 3 intersecting contact regions."""
        ...
    @overload
    def AddTrianglesRigidBodyBased(self, rigidBodyMarkerIndex: MarkerIndex, contactStiffness: float, contactDamping: float, frictionMaterialIndex: int, pointList: List[[float,float,float]], triangleList: List[[int,int,int]], staticTriangles: bool=False) -> int: 
        """Add contact object using a rigidBodyMarker (of a body), contact/friction parameters, a list of points (as 3D numpy arrays or lists; coordinates relative to rigidBodyMarker) and a list of triangles (3 indices as numpy array or list) according to a mesh attached to the rigidBodyMarker; the flag staticTriangles=True can be used to inform the contact solver that these triangles are static (fixed in space); note that static triangles have to be added before dynamic triangles; function returns starting local index of trigsRigidBodyBased at which the triangles are stored; mesh can be produced with GraphicsData2TrigsAndPoints(...); contact is possible between sphere (circle) and Triangle but yet not between triangle and triangle; frictionMaterialIndex refers to frictionPairings in GeneralContact; contactStiffness is computed as serial spring between contacting objects, while damping is computed as a parallel damper (otherwise the smaller damper would always dominate); the triangle normal must point outwards, with the normal of a triangle given with local points (p0,p1,p2) defined as n=(p1-p0) x (p2-p0), see function ComputeTriangleNormal(...)."""
        ...
    @overload
    def GetItemsInBox(self, pMin: [float,float,float], pMax: [float,float,float]) -> Union[dict,bool]: 
        """Get all items in box defined by minimum coordinates given in pMin and maximum coordinates given by pMax, accepting 3D lists or numpy arrays; in case that no objects are found, False is returned; otherwise, a dictionary is returned, containing numpy arrays with indices of obtained MarkerBasedSpheres, TrigsRigidBodyBased, ANCFCable2D, ...; the indices refer to the local index in GeneralContact which can be evaluated e.g., by GetMarkerBasedSphere(localIndex).
        
        Examples:
            gContact.GetItemsInBox(pMin=[0,1,1],
                pMax=[2,3,2])
        """
        ...
    @overload
    def GetSphereMarkerBased(self, localIndex: int, addData: bool=False) -> dict: 
        """Get dictionary with current position, orientation, velocity, angular velocity as computed in last contact iteration; if addData=True, adds stored data of contact element, such as radius, markerIndex and contact parameters; localIndex is the internal index of contact element, as returned e.g., from GetItemsInBox."""
        ...
    @overload
    def SetSphereMarkerBased(self, localIndex: int, contactStiffness: float=-1., contactDamping: float=-1., radius: float=-1., frictionMaterialIndex: int=-1) -> None: 
        """Set data of marker based sphere with localIndex (as internally stored) with given arguments; arguments that are < 0 (default) imply that current values are not overwritten."""
        ...
    @overload
    def GetTriangleRigidBodyBased(self, localIndex: int) -> dict: 
        """Get dictionary with rigid body index, local position of triangle vertices (nodes) and triangle normal; NOTE: the mesh added to contact is different from this structure, as it contains nodes and connectivity lists; the triangle index corresponds to the order as triangles are added to GeneralContact."""
        ...
    @overload
    def SetTriangleRigidBodyBased(self, localIndex: int, points: NDArray[Shape2D[3,3], float], contactRigidBodyIndex: int=-1) -> None: 
        """Set data of marker based sphere with localIndex (triangle index); points are provided as 3x3 numpy array, with point coordinates in rows; contactRigidBodyIndex<0 indicates no change of the current index (and changing this index should be handled with care)."""
        ...
    @overload
    def ShortestDistanceAlongLine(self, pStart: [float,float,float]=[0,0,0], direction: [float,float,float]=[1,0,0], minDistance: float=-1e-7, maxDistance: float=1e7, asDictionary: bool=False, cylinderRadius: float=0, typeIndex: ContactTypeIndex=ContactTypeIndex.IndexEndOfEnumList) -> Union[dict,float]: 
        """Find shortest distance to contact objects in GeneralContact along line with pStart (given as 3D list or numpy array) and direction (as 3D list or numpy array with no need to be normalized); the function returns the distance which is >= minDistance and < maxDistance; in case of beam elements, it measures the distance to the beam centerline; the distance is measured from pStart along given direction and can also be negative; if no item is found along line, the maxDistance is returned; if asDictionary=False, the result is a float, while otherwise details are returned as dictionary (including distance, velocityAlongLine (which is the object velocity in given direction and may be different from the time derivative of the distance; works similar to a laser Doppler vibrometer - LDV), itemIndex and itemType in GeneralContact); the cylinderRadius, if not equal to 0, will be used for spheres to find closest sphere along cylinder with given point and direction; the typeIndex can be set to a specific contact type, e.g., which are searched for (otherwise all objects are considered)."""
        ...
    @overload
    def UpdateContacts(self, mainSystem: "MainSystem") -> None: 
        """Update contact sets, e.g., if no contact is simulated (isActive=False) but user functions need up-to-date contact states for GetItemsInBox(...) or for GetActiveContacts(...).
        
        Examples:
            gContact.UpdateContacts(mbs)
        """
        ...
    @overload
    def GetActiveContacts(self, typeIndex: ContactTypeIndex, itemIndex: int) -> List[int]: 
        """Get list of global item numbers which are in contact with itemIndex of type typeIndex in case that the global itemIndex is smaller than the abs value of the contact pair index; a negative sign indicates that the contacting (spheres) is in Coloumb friction, a positive sign indicates a regularized friction region; in case of itemIndex==-1, it will return the list of numbers of active contacts per item for the contact type; for interpretation of global contact indices, see gContact.GetPythonObject() and documentation; requires either implicit contact computation or UpdateContacts(...) needs to be called prior to this function.
        
        Examples:
            #if explicit solver is used, we first need to update contacts:
            gContact.UpdateContacts(mbs)
            #obtain active contacts of marker based sphere 42:
            gList = gContact.GetActiveContacts(exu.ContactTypeIndex.IndexSpheresMarkerBased, 42)
        """
        ...
    @overload
    def GetSystemODE2RhsContactForces(self, copy: bool=False) -> List[float]: 
        """Get numpy array of system vector containing contribution of contact forces to system ODE2 Rhs vector; if copy=False, it will give direct (reference) access to the internal vector (note: modifications to this vector do not influence simulation!), however, which may cause problems if the system size changes or simulation is restarted; if copy=True, the vector is copied (time consuming); contributions to single objects may be extracted by checking the according LTG-array of according objects (such as rigid bodies); the contact forces vector is computed in each contact iteration;."""
        ...


class SystemData:
    """A data structure of a MainSystem which mainly allows access to states and details of items (objects, nodes, loads, etc.).

    In particular access is given to system coordinates in all configurations, object and node coordinates, as well as to local-to-global (LTG) coordinate indices
    Here, ODE2 represents second order differential equations (and coordinates), ODE1 for first order ODEs, AE represents algebraic equations, and Data is used for data (=history) variables that represent contact states or plastic deformation which is no classical state
    The SystemData structure allows advanced access to this data, which HAS TO BE USED WITH CARE, as unexpected results and system crash might happen.
    """
    @overload
    def NumberOfLoads(self) -> int: 
        """Return number of loads in system.
        
        Examples:
            print(mbs.systemData.NumberOfLoads())
        """
        ...
    @overload
    def NumberOfMarkers(self) -> int: 
        """Return number of markers in system.
        
        Examples:
            print(mbs.systemData.NumberOfMarkers())
        """
        ...
    @overload
    def NumberOfNodes(self) -> int: 
        """Return number of nodes in system.
        
        Examples:
            print(mbs.systemData.NumberOfNodes())
        """
        ...
    @overload
    def NumberOfObjects(self) -> int: 
        """Return number of objects in system.
        
        Examples:
            print(mbs.systemData.NumberOfObjects())
        """
        ...
    @overload
    def NumberOfSensors(self) -> int: 
        """Return number of sensors in system.
        
        Examples:
            print(mbs.systemData.NumberOfSensors())
        """
        ...
    @overload
    def ODE2Size(self, configurationType: ConfigurationType=ConfigurationType.Current) -> int: 
        """Get size of ODE2 coordinate vector for given configuration (only works correctly after mbs.Assemble() ).
        
        Examples:
            print('ODE2 size=',mbs.systemData.ODE2Size())
        """
        ...
    @overload
    def ODE1Size(self, configurationType: ConfigurationType=ConfigurationType.Current) -> int: 
        """Get size of ODE1 coordinate vector for given configuration (only works correctly after mbs.Assemble() ).
        
        Examples:
            print('ODE1 size=',mbs.systemData.ODE1Size())
        """
        ...
    @overload
    def AEsize(self, configurationType: ConfigurationType=ConfigurationType.Current) -> int: 
        """Get size of AE coordinate vector for given configuration (only works correctly after mbs.Assemble() ).
        
        Examples:
            print('AE size=',mbs.systemData.AEsize())
        """
        ...
    @overload
    def DataSize(self, configurationType: ConfigurationType=ConfigurationType.Current) -> int: 
        """Get size of Data coordinate vector for given configuration (only works correctly after mbs.Assemble() ).
        
        Examples:
            print('Data size=',mbs.systemData.DataSize())
        """
        ...
    @overload
    def SystemSize(self, configurationType: ConfigurationType=ConfigurationType.Current) -> int: 
        """Get size of System coordinate vector for given configuration (only works correctly after mbs.Assemble() ).
        
        Examples:
            print('System size=',mbs.systemData.SystemSize())
        """
        ...
    @overload
    def GetTime(self, configurationType: ConfigurationType=ConfigurationType.Current) -> float: 
        """Get configuration dependent time.
        
        Examples:
            mbs.systemData.GetTime(exu.ConfigurationType.Initial)
        """
        ...
    @overload
    def SetTime(self, newTime: float, configurationType: ConfigurationType=ConfigurationType.Current) -> None: 
        """Set configuration dependent time; use this access with care, e.g., in user-defined solvers.
        
        Examples:
            mbs.systemData.SetTime(10., exu.ConfigurationType.Initial)
        """
        ...
    @overload
    def AddODE2LoadDependencies(self, loadNumber: float, globalODE2coordinates: List[int]) -> None: 
        """Advanced function for adding special dependencies of loads onto ODE2 coordinates, taking a list / numpy array of global ODE2 coordinates; this function needs to be called after Assemble() and needs to contain global ODE2 coordinate indices; this list only affects implicit or static solvers if timeIntegration.computeLoadsJacobian or staticSolver.computeLoadsJacobian is set to 1 (ODE2) or 2 (ODE2 and ODE2_t dependencies); if set, it may greatly improve convergence if loads with user functions depend on some system states, such as in a load with feedback control loop; the additional dependencies are not required, if doSystemWideDifferentiation=True, however the latter option being much less efficient.
        
        For more details, consider the file doublePendulum2DControl.py in the examples directory.
        
        Examples:
            mbs.systemData.AddODE2LoadDependencies(0,[0,1,2])
            #add dependency of load 5 onto node 2 coordinates:
            nodeLTG2 = mbs.systemData.GetNodeLTGODE2(2)
            mbs.systemData.AddODE2LoadDependencies(5,nodeLTG2)
        """
        ...
    @overload
    def Info(self) -> None: 
        """Print detailed information on every item; for short information use print(mbs).
        
        Examples:
            mbs.systemData.Info()
        """
        ...
    @overload
    def InfoLTG(self) -> None: 
        """Print LTG information of objects and load dependencies.
        
        Examples:
            mbs.systemData.InfoLTG()
        """
        ...
    @overload
    def GetODE2CoordinatesTotal(self, configuration: ConfigurationType=ConfigurationType.Current) -> List[float]: 
        """Get ODE2 system coordinates (displacements/rotation) including reference values for given configuration (default: exu.Configuration.Current); in case of exu.ConfigurationType.Reference, it only includes reference values once and is identical to GetODE2Coordinates; note that faster access to coordinates is possibly with GetODE2Coordinates(copy=False), which is not possible with GetODE2CoordinatesTotal !.
        
        Examples:
            uTotal = mbs.systemData.GetODE2CoordinatesTotal()
            #this is equivalent to:
            uTotal=mbs.systemData.GetODE2Coordinates()+mbs.systemData.GetODE2Coordinates(exu.ConfigurationType.Reference)
        """
        ...
    @overload
    def GetODE2Coordinates(self, configuration: ConfigurationType=ConfigurationType.Current, copy: bool=True) -> List[float]: 
        """Get ODE2 system coordinates (displacements/rotations) for given configuration (default: exu.Configuration.Current).
        
        Examples:
            uCurrent = mbs.systemData.GetODE2Coordinates()
        """
        ...
    @overload
    def SetODE2Coordinates(self, coordinates: List[float], configuration: ConfigurationType=ConfigurationType.Current) -> None: 
        """Set ODE2 system coordinates (displacements/rotations) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!.
        
        Examples:
            mbs.systemData.SetODE2Coordinates(uCurrent)
        """
        ...
    @overload
    def GetODE2Coordinates_t(self, configuration: ConfigurationType=ConfigurationType.Current, copy: bool=True) -> List[float]: 
        """Get ODE2 system coordinates (velocities) for given configuration (default: exu.Configuration.Current).
        
        Examples:
            vCurrent = mbs.systemData.GetODE2Coordinates_t()
        """
        ...
    @overload
    def SetODE2Coordinates_t(self, coordinates: List[float], configuration: ConfigurationType=ConfigurationType.Current) -> None: 
        """Set ODE2 system coordinates (velocities) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!.
        
        Examples:
            mbs.systemData.SetODE2Coordinates_t(vCurrent)
        """
        ...
    @overload
    def GetODE2Coordinates_tt(self, configuration: ConfigurationType=ConfigurationType.Current, copy: bool=True) -> List[float]: 
        """Get ODE2 system coordinates (accelerations) for given configuration (default: exu.Configuration.Current).
        
        Examples:
            vCurrent = mbs.systemData.GetODE2Coordinates_tt()
        """
        ...
    @overload
    def SetODE2Coordinates_tt(self, coordinates: List[float], configuration: ConfigurationType=ConfigurationType.Current) -> None: 
        """Set ODE2 system coordinates (accelerations) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!.
        
        Examples:
            mbs.systemData.SetODE2Coordinates_tt(aCurrent)
        """
        ...
    @overload
    def GetODE1Coordinates(self, configuration: ConfigurationType=ConfigurationType.Current, copy: bool=True) -> List[float]: 
        """Get ODE1 system coordinates (displacements) for given configuration (default: exu.Configuration.Current).
        
        Examples:
            qCurrent = mbs.systemData.GetODE1Coordinates()
        """
        ...
    @overload
    def SetODE1Coordinates(self, coordinates: List[float], configuration: ConfigurationType=ConfigurationType.Current) -> None: 
        """Set ODE1 system coordinates (velocities) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!.
        
        Examples:
            mbs.systemData.SetODE1Coordinates_t(qCurrent)
        """
        ...
    @overload
    def GetODE1Coordinates_t(self, configuration: ConfigurationType=ConfigurationType.Current, copy: bool=True) -> List[float]: 
        """Get ODE1 system coordinates (velocities) for given configuration (default: exu.Configuration.Current).
        
        Examples:
            qCurrent = mbs.systemData.GetODE1Coordinates_t()
        """
        ...
    @overload
    def SetODE1Coordinates_t(self, coordinates: List[float], configuration: ConfigurationType=ConfigurationType.Current) -> None: 
        """Set ODE1 system coordinates (displacements) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!.
        
        Examples:
            mbs.systemData.SetODE1Coordinates(qCurrent)
        """
        ...
    @overload
    def GetAECoordinates(self, configuration: ConfigurationType=ConfigurationType.Current, copy: bool=True) -> List[float]: 
        """Get algebraic equations (AE) system coordinates for given configuration (default: exu.Configuration.Current).
        
        Examples:
            lambdaCurrent = mbs.systemData.GetAECoordinates()
        """
        ...
    @overload
    def SetAECoordinates(self, coordinates: List[float], configuration: ConfigurationType=ConfigurationType.Current) -> None: 
        """Set algebraic equations (AE) system coordinates for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!.
        
        Examples:
            mbs.systemData.SetAECoordinates(lambdaCurrent)
        """
        ...
    @overload
    def GetDataCoordinates(self, configuration: ConfigurationType=ConfigurationType.Current, copy: bool=True) -> List[float]: 
        """Get system data coordinates for given configuration (default: exu.Configuration.Current).
        
        Examples:
            dataCurrent = mbs.systemData.GetDataCoordinates()
        """
        ...
    @overload
    def SetDataCoordinates(self, coordinates: List[float], configuration: ConfigurationType=ConfigurationType.Current) -> None: 
        """Set system data coordinates for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!.
        
        Examples:
            mbs.systemData.SetDataCoordinates(dataCurrent)
        """
        ...
    @overload
    def GetSystemState(self, configuration: ConfigurationType=ConfigurationType.Current) -> List[List[float]]: 
        """Get system state for given configuration (default: exu.Configuration.Current); state vectors do not include the non-state derivatives ODE1_t and ODE2_tt and the time; function is copying data - not highly efficient; format of pyList: [ODE2Coords, ODE2Coords_t, ODE1Coords, AEcoords, dataCoords].
        
        Examples:
            sysStateList = mbs.systemData.GetSystemState()
        """
        ...
    @overload
    def SetSystemState(self, systemStateList: List[List[float]], configuration: ConfigurationType=ConfigurationType.Current) -> None: 
        """Set system data coordinates for given configuration (default: exu.Configuration.Current); invalid list of vectors / vector size may lead to system crash; write access to state vectors (but not the non-state derivatives ODE1_t and ODE2_tt and the time); function is copying data - not highly efficient; format of pyList: [ODE2Coords, ODE2Coords_t, ODE1Coords, AEcoords, dataCoords].
        
        Examples:
            mbs.systemData.SetSystemState(sysStateList, configuration = exu.ConfigurationType.Initial)
        """
        ...
    @overload
    def GetSystemStateDict(self, configuration: ConfigurationType=ConfigurationType.Current, reference: bool=False) -> Dict[List[float]]: 
        """Get dictionary with copies of (or references to) system states for given configuration (default: exu.Configuration.Current), with at least the following quantities: ODE1Coords, ODE1Coords_t, ODE2Coords, ODE2Coords_t, ODE2Coords_tt, AECoords, dataCoords; we can obtain copies OR references to vectors without copying, meaning that these vectors then have read-write properties and have to be treated carefully! The dictionary's contents are subject to changes in the future; if reference=False, data is copied.
        
        Examples:
            d = mbs.systemData.GetSystemStateDict()
        """
        ...
    @overload
    def GetObjectLTGODE2(self, objectNumber: int) -> List[int]: 
        """Get object local-to-global coordinate mapping (list of global coordinate indices) for ODE2 coordinates; only available after Assemble().
        
        Examples:
            ltgObject4 = mbs.systemData.GetObjectLTGODE2(4)
        """
        ...
    @overload
    def GetObjectLTGODE1(self, objectNumber: int) -> List[int]: 
        """Get object local-to-global coordinate mapping (list of global coordinate indices) for ODE1 coordinates; only available after Assemble().
        
        Examples:
            ltgObject4 = mbs.systemData.GetObjectLTGODE1(4)
        """
        ...
    @overload
    def GetObjectLTGAE(self, objectNumber: int) -> List[int]: 
        """Get object local-to-global coordinate mapping (list of global coordinate indices) for algebraic equations (AE) coordinates; only available after Assemble().
        
        Examples:
            ltgObject4 = mbs.systemData.GetObjectLTGAE(4)
        """
        ...
    @overload
    def GetObjectLTGData(self, objectNumber: int) -> List[int]: 
        """Get object local-to-global coordinate mapping (list of global coordinate indices) for data coordinates; only available after Assemble().
        
        Examples:
            ltgObject4 = mbs.systemData.GetObjectLTGData(4)
        """
        ...
    @overload
    def GetNodeLTGODE2(self, nodeNumber: int) -> List[int]: 
        """Get node local-to-global coordinate mapping (list of global coordinate indices) for ODE2 coordinates; only available after Assemble().
        
        Examples:
            ltgNode4 = mbs.systemData.GetNodeLTGODE2(4)
        """
        ...
    @overload
    def GetNodeLTGODE1(self, nodeNumber: int) -> List[int]: 
        """Get node local-to-global coordinate mapping (list of global coordinate indices) for ODE1 coordinates; only available after Assemble().
        
        Examples:
            ltgNode4 = mbs.systemData.GetNodeLTGODE1(4)
        """
        ...
    @overload
    def GetNodeLTGAE(self, nodeNumber: int) -> List[int]: 
        """Get node local-to-global coordinate mapping (list of global coordinate indices) for AE coordinates; only available after Assemble().
        
        Examples:
            ltgNode4 = mbs.systemData.GetNodeLTGAE(4)
        """
        ...
    @overload
    def GetNodeLTGData(self, nodeNumber: int) -> List[int]: 
        """Get node local-to-global coordinate mapping (list of global coordinate indices) for Data coordinates; only available after Assemble().
        
        Examples:
            ltgNode4 = mbs.systemData.GetNodeLTGData(4)
        """
        ...


class MainSystem:
    """MainSystem is the class which defines a (multibody) system and it's instance if usually called ``mbs``.

    Interactions with the system are done via MainSystem, either through, e.g., ``mbs.AddObject(...)`` or with create functions, such as ``mbs.CreateRigidBody(...)``; States are accessible via ``mbs.systemData``
    The MainSystem shall only be created from a SystemContainer ``SC`` using ``SC.AddSystem()``; do not use ``exu.MainSystem()``, as the latter one would not be linked to a SystemContainer
    Having already a valid ``mbs``, you may use ``SC.Append(mbs).``
    """
    @overload
    def Assemble(self) -> None: 
        """Assemble items (nodes, bodies, markers, loads, ...) of multibody system; Calls CheckSystemIntegrity(...), AssembleCoordinates(), AssembleLTGLists(), AssembleInitializeSystemCoordinates(), and AssembleSystemInitialize()."""
        ...
    @overload
    def AssembleCoordinates(self) -> None: 
        """Assemble coordinates: assign computational coordinates to nodes and constraints (algebraic variables)."""
        ...
    @overload
    def AssembleLTGLists(self) -> None: 
        r"""Build \ac{LTG} coordinate lists for objects (used to build global ODE2RHS, MassMatrix, etc.
        
        vectors and matrices) and store special object lists (body, connector, constraint, ...)
        """
        ...
    @overload
    def AssembleInitializeSystemCoordinates(self) -> None: 
        """Initialize all system-wide coordinates based on initial values given in nodes."""
        ...
    @overload
    def AssembleSystemInitialize(self) -> None: 
        """Initialize some system data, e.g., generalContact objects (searchTree, etc.)."""
        ...
    @overload
    def Reset(self) -> None: 
        """Reset all lists of items (nodes, bodies, markers, loads, ...) and temporary vectors; deallocate memory."""
        ...
    @overload
    def GetSystemContainer(self) -> "SystemContainer": 
        """Return the systemContainer where the mainSystem (mbs) was created."""
        ...
    @overload
    def SendRedrawSignal(self) -> None: 
        """This function is used to send a signal to the renderer that the scene shall be redrawn because the visualization state has been updated."""
        ...
    @overload
    def GetRenderEngineStopFlag(self) -> bool: 
        """Get the current stop simulation flag; True=user wants to stop simulation."""
        ...
    @overload
    def SetRenderEngineStopFlag(self, stopFlag: bool) -> None: 
        """Set the current stop simulation flag; set to False, in order to continue a previously user-interrupted simulation."""
        ...
    @overload
    def ActivateRendering(self, flag: bool=True) -> None: 
        """Activate (flag=True) or deactivate (flag=False) rendering for this system."""
        ...
    @overload
    def SetPreStepUserFunction(self, value: Callable[["MainSystem", float],bool]) -> None: 
        """Sets a user function PreStepUserFunction(mbs, t) executed at beginning of every computation step; in normal case return True; return False to stop simulation after current step; set to 0 (integer) in order to erase user function.
        
        Note that the time t in the args is already the end of the step, which allows to compute forces consistently with trapezoidal integrators; for higher order Runge-Kutta methods, step time will be available only in object-user functions. The PreStepUserFunction is recommended e.g., for prescribing forces or set values of actuators
        
        Examples:
            def PreStepUserFunction(mbs, t):
                print(mbs.systemData.NumberOfNodes())
                if(t>1):
                    return False
                return True
            mbs.SetPreStepUserFunction(PreStepUserFunction)
        """
        ...
    @overload
    def GetPreStepUserFunction(self, asDict: bool=False) -> Callable[["MainSystem", float],bool]: 
        """Returns the preStepUserFunction."""
        ...
    @overload
    def SetPostStepUserFunction(self, value: Callable[["MainSystem", float],bool]) -> None: 
        """Sets a user function PostStepUserFunction(mbs, t) executed at end of every computation step; in normal case return True; return False to stop simulation after current step; set to 0 (integer) in order to erase user function.
        
        The difference to PreStepUserFunction, the PostStepUserFunction is called after the step has been computed, AFTER the discontinuous iterations, just BEFORE writing solution file, sensors and visualization. This allows to change or evaluate results before they are stored (e.g., do some projection).
        
        Examples:
            def PostStepUserFunction(mbs, t):
                print(mbs.systemData.NumberOfNodes())
                if(t>1):
                    return False
                return True
            mbs.SetPostStepUserFunction(PostStepUserFunction)
        """
        ...
    @overload
    def GetPostStepUserFunction(self, asDict: bool=False) -> Callable[["MainSystem", float],bool]: 
        """Returns the postStepUserFunction."""
        ...
    @overload
    def SetPostNewtonUserFunction(self, value: Callable[["MainSystem", float],[float,float]]) -> None: 
        """Sets a user function PostNewtonUserFunction(mbs, t) executed after successful Newton iteration in implicit or static solvers and after step update of explicit solvers, but BEFORE PostNewton functions are called by the solver; function returns list [discontinuousError, recommendedStepSize], containing a error of the PostNewtonStep, which is compared to [solver].discontinuous.iterationTolerance.
        
        The recommendedStepSize shall be negative, if no recommendation is given, 0 in order to enforce minimum step size or a specific value to which the current step size will be reduced and the step will be repeated; use this function, e.g., to reduce step size after impact or change of data variables; set to 0 (integer) in order to erase user function. Similar described by Flores and Ambrosio, https://doi.org/10.1007/s11044-010-9209-8
        
        Examples:
            def PostNewtonUserFunction(mbs, t):
                if(t>1):
                    return [0, 1e-6]
                return [0,0]
            mbs.SetPostNewtonUserFunction(PostNewtonUserFunction)
        """
        ...
    @overload
    def GetPostNewtonUserFunction(self, asDict: bool=False) -> Callable[["MainSystem", float],bool]: 
        """Returns the postNewtonUserFunction."""
        ...
    @overload
    def SetPreNewtonResidualUserFunction(self, value: Callable[["MainSystem", float, int, int],None]) -> None: 
        """Sets a user function PreNewtonResidualUserFunction(mbs, t, newtonIt, discontinuousIt) executed prior to computation of the Newton residual in implicit or static solvers.
        
        This function returns nothing. The arguments newtonIt and discontinuousIt may be used to distinguish if the call is done at the beginning of a discontinuous iteration (newtonIt=0) or during Newton iterations (newtonIt>0). The typical use case would be to modify objects or loads in every iteration. Note that this user function is not called during Jacobian computation. If needed, the jacobian can be modified with the user function set by SetSystemJacobianUserFunction.
        
        Examples:
            def PreNewtonResidualUserFunction(mbs, t, newtonIt, discontinuousIt):
                print("t=",t,", newtonIt=",newtonIt,", discIt=",discontinuousIt)
            mbs.SetPreNewtonResidualUserFunction(PreNewtonResidualUserFunction)
        """
        ...
    @overload
    def GetPreNewtonResidualUserFunction(self, asDict: bool=False) -> Callable[["MainSystem", float, int, int],None]: 
        """Returns the preNewtonResidualUserFunction."""
        ...
    @overload
    def SetSystemJacobianUserFunction(self, value: Callable[["MainSystem", float, float, float, float],Any]) -> None: 
        """Sets a user function SystemJacobianUserFunction(mbs, t, factorODE2, factorODE2_t, factorODE1) executed after computation of the Newton jacobian of a static solver or an implicit timeintegrator; The function shall return additional terms for the jacobian at RHS, e.g., related to dependencies that are added by the user in the PreNewtonResidualUserFunction; RHS means that for a spring with stiffness K, the jacobian would be -K as it is computed for the RHS, see the RHS-LHS convention.
        
        If you like to completely replace the jacobian, consider using the solver's user function SetUserFunctionComputeNewtonJacobian which can be used to replace the jacobian computation; the factors factorODE2, factorODE2_t, factorODE1 must be multiplied with quantities related to ODE2 coordinates (like stiffness terms), ODE2_t velocity coordinates (like damping terms) and ODE1 quantities. The functions returns a MatrixContainer, for which the sparse format is recommended for efficiency reasons.
        
        Examples:
            def SystemJacobianUserFunction(mbs, t, factorODE2, factorODE2_t, factorODE1):
                return MatrixContainer([[factorODE2*10,0],[0,0]])
            mbs.SetSystemJacobianUserFunction(SystemJacobianUserFunction)
        """
        ...
    @overload
    def GetSystemJacobianUserFunction(self, asDict: bool=False) -> Callable[["MainSystem", float, float, float, float],Any]: 
        """Returns the systemJacobianUserFunction."""
        ...
    @overload
    def AddGeneralContact(self) -> GeneralContact: 
        """Add a new general contact, used to enable efficient contact computation between objects (nodes or markers)."""
        ...
    @overload
    def GetGeneralContact(self, generalContactNumber: int) -> GeneralContact: 
        """Get read/write access to GeneralContact with index generalContactNumber stored in mbs; Examples shows how to access the GeneralContact object added with last AddGeneralContact() command:.
        
        Examples:
            gc=mbs.GetGeneralContact(mbs.NumberOfGeneralContacts()-1)
        """
        ...
    @overload
    def DeleteGeneralContact(self, generalContactNumber: int) -> None: 
        """Delete GeneralContact with index generalContactNumber in mbs; other general contacts are resorted (index changes!)."""
        ...
    @overload
    def NumberOfGeneralContacts(self) -> int: 
        """Return number of GeneralContact objects in mbs."""
        ...
    @overload
    def GetAvailableFactoryItems(self) -> dict: 
        """Get all available items to be added (nodes, objects, etc.); this is useful in particular in case of additional user elements to check if they are available; the available items are returned as dictionary, containing lists of strings for Node, Object, etc."""
        ...
    @overload
    def GetDictionary(self) -> dict: 
        """[UNDER DEVELOPMENT]: return the dictionary of the system data (todo: and state), e.g., to copy the system or for pickling."""
        ...
    @overload
    def SetDictionary(self, systemDict: dict) -> None: 
        """[UNDER DEVELOPMENT]: set system data (todo: and state) from given dictionary; used for pickling."""
        ...
    systemIsConsistent:bool
    """this flag is used by solvers to decide, whether the system is in a solvable state; this flag is set to False as long as Assemble() has not been called; any modification to the system, such as Add...(), Modify...(), etc. will set the flag to False again; this flag can be modified (set to True), if a change of e.g.~an object (change of stiffness) or load (change of force) keeps the system consistent, but would normally lead to systemIsConsistent=False."""
    interactiveMode:bool
    """set this flag to True in order to invoke a Assemble() command in every system modification, e.g., AddNode, AddObject, ModifyNode, ...; this helps that the system can be visualized in interactive mode."""
    variables:dict
    """this dictionary may be used by the user to store model-specific data, in order to avoid global Python variables in complex models; mbs.variables['myvar'] = 42."""
    sys:dict
    """this dictionary is used by exudyn Python libraries, e.g., solvers, to avoid global Python variables."""
    solverSignalJacobianUpdate:bool
    """this flag is used by solvers to decide, whether the jacobian should be updated; at beginning of simulation and after jacobian computation, this flag is set automatically to False; use this flag to indicate system changes, e.g., during time integration."""
    systemData:SystemData
    """Access to SystemData structure; enables access to number of nodes, objects, ... and to (current, initial, reference, ...) state variables (ODE2, AE, Data,...)."""
    @overload
    def AddNode(self, pyObject: Any) -> NodeIndex: 
        """Add a node with nodeDefinition from Python node class; returns (global) node index (type NodeIndex) of newly added node; use int(nodeIndex) to convert to int, if needed (but not recommended in order not to mix up index types of nodes, objects, markers, ...).
        
        Examples:
            item = Rigid2D( referenceCoordinates= [1,0.5,0], initialVelocities= [10,0,0])
            mbs.AddNode(item)
            nodeDict = {'nodeType': 'Point',
            'referenceCoordinates': [1.0, 0.0, 0.0],
            'initialCoordinates': [0.0, 2.0, 0.0],
            'name': 'example node'}
            mbs.AddNode(nodeDict)
        """
        ...
    @overload
    def DeleteNode(self, nodeNumber, suppressWarnings=False) -> None: 
        """Delete the node with nodeNumber in MainSystem; consistently renames nodes according to their new node numbers; adapts node numbers in sensors and in markers; items using deleted nodeNumber obtain invalid nodeNumber.
        
        Examples:
            mbs.DeleteNode(nodeNumber=42)
        """
        ...
    @overload
    def GetNodeNumber(self, nodeName: str) -> NodeIndex: 
        """Get node's number by name (string).
        
        Examples:
            n = mbs.GetNodeNumber('example node')
        """
        ...
    @overload
    def GetNode(self, nodeNumber: NodeIndex) -> dict: 
        """Get node's dictionary by node number (type NodeIndex).
        
        Examples:
            nodeDict = mbs.GetNode(0)
        """
        ...
    @overload
    def ModifyNode(self, nodeNumber: NodeIndex, nodeDict: dict) -> None: 
        """Modify node's dictionary by node number (type NodeIndex).
        
        Examples:
            mbs.ModifyNode(nodeNumber, nodeDict)
        """
        ...
    @overload
    def GetNodeDefaults(self, typeName: str) -> dict: 
        """Get node's default values for a certain nodeType as (dictionary).
        
        Examples:
            nodeType = 'Point'
            nodeDict = mbs.GetNodeDefaults(nodeType)
        """
        ...
    @overload
    def GetNodeOutput(self, nodeNumber: NodeIndex, variableType: OutputVariableType, configuration: ConfigurationType=ConfigurationType.Current) -> List[float]: 
        """Get the ouput of the node specified with the OutputVariableType; output may be scalar or array (e.g., displacement vector).
        
        Examples:
            mbs.GetNodeOutput(nodeNumber=0, variableType=exu.OutputVariableType.Displacement)
        """
        ...
    @overload
    def GetNodeODE2Index(self, nodeNumber: NodeIndex) -> int: 
        """Get index in the global ODE2 coordinate vector for the first node coordinate of the specified node.
        
        Examples:
            mbs.GetNodeODE2Index(nodeNumber=0)
        """
        ...
    @overload
    def GetNodeODE1Index(self, nodeNumber: NodeIndex) -> int: 
        """Get index in the global ODE1 coordinate vector for the first node coordinate of the specified node.
        
        Examples:
            mbs.GetNodeODE1Index(nodeNumber=0)
        """
        ...
    @overload
    def GetNodeAEIndex(self, nodeNumber: NodeIndex) -> int: 
        """Get index in the global AE coordinate vector for the first node coordinate of the specified node.
        
        Examples:
            mbs.GetNodeAEIndex(nodeNumber=0)
        """
        ...
    @overload
    def GetNodeParameter(self, nodeNumber: NodeIndex, parameterName: str) -> Any: 
        """Get nodes's parameter from node number (type NodeIndex) and parameterName; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix.
        
        Examples:
            mbs.GetNodeParameter(0, 'referenceCoordinates')
        """
        ...
    @overload
    def SetNodeParameter(self, nodeNumber: NodeIndex, parameterName: str, value: Any) -> None: 
        """Set parameter 'parameterName' of node with node number (type NodeIndex) to value; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix.
        
        Examples:
            mbs.SetNodeParameter(0, 'Vshow', True)
        """
        ...
    @overload
    def AddObject(self, pyObject: Any) -> ObjectIndex: 
        """Add an object with objectDefinition from Python object class; returns (global) object number (type ObjectIndex) of newly added object.
        
        Examples:
            item = MassPoint(name='heavy object', nodeNumber=0, physicsMass=100)
            mbs.AddObject(item)
            objectDict = {'objectType': 'MassPoint',
            'physicsMass': 10,
            'nodeNumber': 0,
            'name': 'example object'}
            mbs.AddObject(objectDict)
        """
        ...
    @overload
    def DeleteObject(self, objectNumber: ObjectIndex, deleteDependentItems: bool=True, suppressWarnings: bool=False) -> None: 
        """Delete the object with objectNumber in MainSystem; consistently renames objects according to their new object numbers; adapts object numbers in sensors and in markers; items using deleted objectNumber obtain invalid objectNumber; with the option deleteDependentItems (default=True) the function also delete nodes and markers which are used by the object.
        
        Examples:
            mbs.DeleteObject(objectNumber=42)
        """
        ...
    @overload
    def GetObjectNumber(self, objectName: str) -> ObjectIndex: 
        """Get object's number by name (string).
        
        Examples:
            n = mbs.GetObjectNumber('heavy object')
        """
        ...
    @overload
    def GetObject(self, objectNumber: ObjectIndex, addGraphicsData: bool=False) -> dict: 
        """Get object's dictionary by object number (type ObjectIndex); NOTE: visualization parameters have a prefix 'V'; in order to also get graphicsData written, use addGraphicsData=True (which is by default False, as it would spoil the information).
        
        Examples:
            objectDict = mbs.GetObject(0)
        """
        ...
    @overload
    def ModifyObject(self, objectNumber: ObjectIndex, objectDict: dict) -> None: 
        """Modify object's dictionary by object number (type ObjectIndex); NOTE: visualization parameters have a prefix 'V'.
        
        Examples:
            mbs.ModifyObject(objectNumber, objectDict)
        """
        ...
    @overload
    def GetObjectDefaults(self, typeName: str) -> dict: 
        """Get object's default values for a certain objectType as (dictionary).
        
        Examples:
            objectType = 'MassPoint'
            objectDict = mbs.GetObjectDefaults(objectType)
        """
        ...
    @overload
    def GetObjectOutput(self, objectNumber: ObjectIndex, variableType: OutputVariableType, configuration: ConfigurationType=ConfigurationType.Current) -> List[float]: 
        """Get object's current output variable from object number (type ObjectIndex) and OutputVariableType; for connectors, it can only be computed for exu.ConfigurationType.Current configuration!."""
        ...
    @overload
    def GetObjectOutputBody(self, objectNumber: ObjectIndex, variableType: OutputVariableType, localPosition: [float,float,float]=[0,0,0], configuration: ConfigurationType=ConfigurationType.Current) -> List[float]: 
        """Get body's output variable from object number (type ObjectIndex) and OutputVariableType, using the localPosition as defined in the body, and as used in MarkerBody and SensorBody.
        
        Examples:
            u = mbs.GetObjectOutputBody(objectNumber = 1, variableType = exu.OutputVariableType.Position, localPosition=[1,0,0], configuration = exu.ConfigurationType.Initial)
        """
        ...
    @overload
    def GetObjectOutputSuperElement(self, objectNumber: ObjectIndex, variableType: OutputVariableType, meshNodeNumber: int, configuration: ConfigurationType=ConfigurationType.Current) -> List[float]: 
        """Get output variable from mesh node number of object with type SuperElement (GenericODE2, FFRF, FFRFreduced - CMS) with specific OutputVariableType; the meshNodeNumber is the object's local node number, not the global node number!.
        
        Examples:
            u = mbs.GetObjectOutputSuperElement(objectNumber = 1, variableType = exu.OutputVariableType.Position, meshNodeNumber = 12, configuration = exu.ConfigurationType.Initial)
        """
        ...
    @overload
    def GetObjectParameter(self, objectNumber: ObjectIndex, parameterName: str) -> Any: 
        """Get objects's parameter from object number (type ObjectIndex) and parameterName; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix; NOTE that BodyGraphicsData cannot be get or set, use dictionary access instead.
        
        Examples:
            mbs.GetObjectParameter(objectNumber = 0, parameterName = 'nodeNumber')
        """
        ...
    @overload
    def SetObjectParameter(self, objectNumber: ObjectIndex, parameterName: str, value: Any) -> None: 
        """Set parameter 'parameterName' of object with object number (type ObjectIndex) to value;; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix; NOTE that BodyGraphicsData cannot be get or set, use dictionary access instead.
        
        Examples:
            mbs.SetObjectParameter(objectNumber = 0, parameterName = 'Vshow', value=True)
        """
        ...
    @overload
    def AddMarker(self, pyObject: Any) -> MarkerIndex: 
        """Add a marker with markerDefinition from Python marker class; returns (global) marker number (type MarkerIndex) of newly added marker.
        
        Examples:
            item = MarkerNodePosition(name='my marker',nodeNumber=1)
            mbs.AddMarker(item)
            markerDict = {'markerType': 'NodePosition',
              'nodeNumber': 0,
              'name': 'position0'}
            mbs.AddMarker(markerDict)
        """
        ...
    @overload
    def DeleteMarker(self, markerNumber: MarkerIndex, suppressWarnings: bool=False) -> None: 
        """Delete the marker with markerNumber in MainSystem; consistently renames markers according to their new marker numbers; adapts marker numbers in objects, loads and sensors; items using deleted markerNumber obtain invalid markerNumber.
        
        Examples:
            mbs.DeleteMarker(markerNumber=42)
        """
        ...
    @overload
    def GetMarkerNumber(self, markerName: str) -> MarkerIndex: 
        """Get marker's number by name (string).
        
        Examples:
            n = mbs.GetMarkerNumber('my marker')
        """
        ...
    @overload
    def GetMarker(self, markerNumber: MarkerIndex) -> dict: 
        """Get marker's dictionary by index.
        
        Examples:
            markerDict = mbs.GetMarker(0)
        """
        ...
    @overload
    def ModifyMarker(self, markerNumber: MarkerIndex, markerDict: dict) -> None: 
        """Modify marker's dictionary by index.
        
        Examples:
            mbs.ModifyMarker(markerNumber, markerDict)
        """
        ...
    @overload
    def GetMarkerDefaults(self, typeName: str) -> dict: 
        """Get marker's default values for a certain markerType as (dictionary).
        
        Examples:
            markerType = 'NodePosition'
            markerDict = mbs.GetMarkerDefaults(markerType)
        """
        ...
    @overload
    def GetMarkerParameter(self, markerNumber: MarkerIndex, parameterName: str) -> Any: 
        """Get markers's parameter from markerNumber and parameterName; parameter names can be found for the specific items in the reference manual."""
        ...
    @overload
    def SetMarkerParameter(self, markerNumber: MarkerIndex, parameterName: str, value: Any) -> None: 
        """Set parameter 'parameterName' of marker with markerNumber to value; parameter names can be found for the specific items in the reference manual."""
        ...
    @overload
    def GetMarkerOutput(self, markerNumber: MarkerIndex, variableType: OutputVariableType, configuration: ConfigurationType=ConfigurationType.Current) -> List[float]: 
        """Get the ouput of the marker specified with the OutputVariableType; currently only provides Displacement, Position and Velocity for position based markers, and RotationMatrix, Rotation and AngularVelocity(Local) for markers providing orientation; Coordinates and Coordinates_t available for coordinate markers.
        
        Examples:
            mbs.GetMarkerOutput(markerNumber=0, variableType=exu.OutputVariableType.Position)
        """
        ...
    @overload
    def AddLoad(self, pyObject: Any) -> LoadIndex: 
        """Add a load with loadDefinition from Python load class; returns (global) load number (type LoadIndex) of newly added load.
        
        Examples:
            item = mbs.AddLoad(LoadForceVector(loadVector=[1,0,0], markerNumber=0, name='heavy load'))
            mbs.AddLoad(item)
            loadDict = {'loadType': 'ForceVector',
              'markerNumber': 0,
              'loadVector': [1.0, 0.0, 0.0],
              'name': 'heavy load'}
            mbs.AddLoad(loadDict)
        """
        ...
    @overload
    def DeleteLoad(self, loadNumber: LoadIndex, deleteDependentMarkers: bool=True, suppressWarnings: bool=False) -> None: 
        """Delete the load with loadNumber in MainSystem; consistently renames loads according to their new load numbers; deleteDependentMarkers (default=True) also deletes the corresponding marker.
        
        Examples:
            mbs.DeleteLoad(loadNumber=42)
        """
        ...
    @overload
    def GetLoadNumber(self, loadName: str) -> LoadIndex: 
        """Get load's number by name (string).
        
        Examples:
            n = mbs.GetLoadNumber('heavy load')
        """
        ...
    @overload
    def GetLoad(self, loadNumber: LoadIndex) -> dict: 
        """Get load's dictionary by index.
        
        Examples:
            loadDict = mbs.GetLoad(0)
        """
        ...
    @overload
    def ModifyLoad(self, loadNumber: LoadIndex, loadDict: dict) -> None: 
        """Modify load's dictionary by index.
        
        Examples:
            mbs.ModifyLoad(loadNumber, loadDict)
        """
        ...
    @overload
    def GetLoadDefaults(self, typeName: str) -> dict: 
        """Get load's default values for a certain loadType as (dictionary).
        
        Examples:
            loadType = 'ForceVector'
            loadDict = mbs.GetLoadDefaults(loadType)
        """
        ...
    @overload
    def GetLoadValues(self, loadNumber: LoadIndex) -> List[float]: 
        """Get current load values, specifically if user-defined loads are used; can be scalar or vector-valued return value."""
        ...
    @overload
    def GetLoadParameter(self, loadNumber: LoadIndex, parameterName: str) -> Any: 
        """Get loads's parameter from loadNumber and parameterName; parameter names can be found for the specific items in the reference manual."""
        ...
    @overload
    def SetLoadParameter(self, loadNumber: LoadIndex, parameterName: str, value: Any) -> None: 
        """Set parameter 'parameterName' of load with loadNumber to value; parameter names can be found for the specific items in the reference manual."""
        ...
    @overload
    def AddSensor(self, pyObject: Any) -> SensorIndex: 
        """Add a sensor with sensor definition from Python sensor class; returns (global) sensor number (type SensorIndex) of newly added sensor.
        
        Examples:
            item = mbs.AddSensor(SensorNode(sensorType= exu.SensorType.Node, nodeNumber=0, name='test sensor'))
            mbs.AddSensor(item)
            sensorDict = {'sensorType': 'Node',
              'nodeNumber': 0,
              'fileName': 'sensor.txt',
              'name': 'test sensor'}
            mbs.AddSensor(sensorDict)
        """
        ...
    @overload
    def DeleteSensor(self, sensorNumber, suppressWarnings=False) -> None: 
        """Delete the marker with sensorNumber in MainSystem; consistently renames sensors according to their new sensor numbers; adapts sensor numbers in sensors; items using deleted sensorNumber obtain invalid sensorNumber.
        
        Examples:
            mbs.DeleteSensor(sensorNumber=42)
        """
        ...
    @overload
    def GetSensorNumber(self, sensorName: str) -> SensorIndex: 
        """Get sensor's number by name (string).
        
        Examples:
            n = mbs.GetSensorNumber('test sensor')
        """
        ...
    @overload
    def GetSensor(self, sensorNumber: SensorIndex) -> dict: 
        """Get sensor's dictionary by index.
        
        Examples:
            sensorDict = mbs.GetSensor(0)
        """
        ...
    @overload
    def ModifySensor(self, sensorNumber: SensorIndex, sensorDict: dict) -> None: 
        """Modify sensor's dictionary by index.
        
        Examples:
            mbs.ModifySensor(sensorNumber, sensorDict)
        """
        ...
    @overload
    def GetSensorDefaults(self, typeName: str) -> dict: 
        """Get sensor's default values for a certain sensorType as (dictionary).
        
        Examples:
            sensorType = 'Node'
            sensorDict = mbs.GetSensorDefaults(sensorType)
        """
        ...
    @overload
    def GetSensorValues(self, sensorNumber: SensorIndex, configuration: ConfigurationType=ConfigurationType.Current) -> List[float]: 
        """Get sensors's values for configuration; can be a scalar or vector-valued return value!."""
        ...
    @overload
    def GetSensorStoredData(self, sensorNumber: SensorIndex) -> ArrayLike: 
        """Get sensors's internally stored data as matrix (all time points stored); rows are containing time and sensor values as obtained by sensor (e.g., time, and x, y, and z value of position)."""
        ...
    @overload
    def GetSensorParameter(self, sensorNumber: SensorIndex, parameterName: str) -> Any: 
        """Get sensors's parameter from sensorNumber and parameterName; parameter names can be found for the specific items in the reference manual."""
        ...
    @overload
    def SetSensorParameter(self, sensorNumber: SensorIndex, parameterName: str, value: Any) -> None: 
        """Set parameter 'parameterName' of sensor with sensorNumber to value; parameter names can be found for the specific items in the reference manual."""
        ...

    @overload
    def SolutionViewer(self, solution=None, rowIncrement=1, timeout=0.04, runOnStart=True, runMode=2, fontSize=12, title='', checkRenderEngineStopFlag=True) -> None: 
        """open interactive dialog and visulation (animate) solution loaded with LoadSolutionFile(...); Change slider 'Increment' to change the automatic increment of time frames; Change mode between continuous run, one cycle (fits perfect for animation recording) or 'Static' (to change Solution steps manually with the mouse); update period also lets you change the speed of animation; Press Run / Stop button to start/stop interactive mode (updating of grpahics)."""
    ...

    @overload
    def CreateGround(self, name='', referencePosition=[0.,0.,0.], referenceRotationMatrix=np.eye(3), graphicsDataList=[], graphicsDataUserFunction=0, show=True) -> ObjectIndex: 
        """helper function to create a ground object, using arguments of ObjectGround; this function is mainly added for consistency with other mainSystemExtensions."""
    ...

    @overload
    def CreateMassPoint(self, name='', referencePosition=[0.,0.,0.], initialDisplacement=[0.,0.,0.], initialVelocity=[0.,0.,0.], physicsMass=0, gravity=[0.,0.,0.], graphicsDataList=[], drawSize=-1, color=[-1.,-1.,-1.,-1.], show=True, create2D=False, returnDict=False) -> Union[dict, ObjectIndex]: 
        """helper function to create 2D or 3D mass point object and node, using arguments as in NodePoint and MassPoint."""
    ...

    @overload
    def CreateRigidBody(self, name='', referencePosition=[0.,0.,0.], referenceRotationMatrix=np.eye(3), initialVelocity=[0.,0.,0.], initialAngularVelocity=[0.,0.,0.], initialDisplacement=None, initialRotationMatrix=None, inertia=None, gravity=[0.,0.,0.], nodeType=exudyn.NodeType.RotationEulerParameters, graphicsDataList=[], graphicsDataUserFunction=0, drawSize=-1, color=[-1.,-1.,-1.,-1.], show=True, create2D=False, returnDict=False) -> Union[dict, ObjectIndex]: 
        """helper function to create 3D (or 2D) rigid body object and node; all quantities are global (angular velocity, etc.); use this function to easily create a rigid body; graphics can be directly obtained from inertia object, e.g. in case of cylindrical or cuboid shape."""
    ...

    @overload
    def CreateSpringDamper(self, name='', bodyNumbers=[None, None], localPosition0=[0.,0.,0.], localPosition1=[0.,0.,0.], referenceLength=None, stiffness=0., damping=0., force=0., velocityOffset=0., springForceUserFunction=0, bodyOrNodeList=[None, None], bodyList=[None, None], show=True, drawSize=-1, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """helper function to create SpringDamper connector, using arguments from ObjectConnectorSpringDamper; similar interface as CreateDistanceConstraint(...), see there for for further information."""
    ...

    @overload
    def CreateCartesianSpringDamper(self, name='', bodyNumbers=[None, None], localPosition0=[0.,0.,0.], localPosition1=[0.,0.,0.], stiffness=[0.,0.,0.], damping=[0.,0.,0.], offset=[0.,0.,0.], springForceUserFunction=0, bodyOrNodeList=[None, None], bodyList=[None, None], show=True, drawSize=-1, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """helper function to create CartesianSpringDamper connector, using arguments from ObjectConnectorCartesianSpringDamper."""
    ...

    @overload
    def CreateRigidBodySpringDamper(self, name='', bodyNumbers=[None, None], localPosition0=[0.,0.,0.], localPosition1=[0.,0.,0.], stiffness=np.zeros((6,6)), damping=np.zeros((6,6)), offset=[0.,0.,0.,0.,0.,0.], rotationMatrixJoint=np.eye(3), useGlobalFrame=True, intrinsicFormulation=True, springForceTorqueUserFunction=0, postNewtonStepUserFunction=0, bodyOrNodeList=[None, None], bodyList=[None, None], show=True, drawSize=-1, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """helper function to create RigidBodySpringDamper connector, using arguments from ObjectConnectorRigidBodySpringDamper, see there for the full documentation."""
    ...

    @overload
    def CreateTorsionalSpringDamper(self, name='', bodyNumbers=[None, None], position=[0.,0.,0.], axis=[0.,0.,0.], stiffness=0., damping=0., offset=0., velocityOffset=0., torque=0., useGlobalFrame=True, springTorqueUserFunction=0, unlimitedRotations=True, show=True, drawSize=-1, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """helper function to create TorsionalSpringDamper connector, using arguments from ObjectConnectorTorsionalSpringDamper, see there for the full documentation."""
    ...

    @overload
    def CreateRevoluteJoint(self, name='', bodyNumbers=[None, None], position=[], axis=[], useGlobalFrame=True, show=True, axisRadius=0.1, axisLength=0.4, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create revolute joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed."""
    ...

    @overload
    def CreatePrismaticJoint(self, name='', bodyNumbers=[None, None], position=[], axis=[], useGlobalFrame=True, show=True, axisRadius=0.1, axisLength=0.4, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create prismatic joint between two bodies; definition of joint position and axis in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers, markerRotation and other quantities are automatically computed."""
    ...

    @overload
    def CreateSphericalJoint(self, name='', bodyNumbers=[None, None], position=[], constrainedAxes=[1,1,1], useGlobalFrame=True, show=True, jointRadius=0.1, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create spherical joint between two bodies; definition of joint position in global coordinates (alternatively in body0 local coordinates) for reference configuration of bodies; all markers are automatically computed."""
    ...

    @overload
    def CreateGenericJoint(self, name='', bodyNumbers=[None, None], position=[], rotationMatrixAxes=np.eye(3), constrainedAxes=[1,1,1, 1,1,1], useGlobalFrame=True, offsetUserFunction=0, offsetUserFunction_t=0, show=True, axesRadius=0.1, axesLength=0.4, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create generic joint between two bodies; definition of joint position (position) and axes (rotationMatrixAxes) in global coordinates (useGlobalFrame=True) or in local coordinates of body0 (useGlobalFrame=False), where rotationMatrixAxes is an additional rotation to body0; all markers, markerRotation and other quantities are automatically computed."""
    ...

    @overload
    def CreateDistanceConstraint(self, name='', bodyNumbers=[None, None], localPosition0=[0.,0.,0.], localPosition1=[0.,0.,0.], distance=None, bodyOrNodeList=[None, None], bodyList=[None, None], show=True, drawSize=-1., color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create distance joint between two bodies; definition of joint positions in local coordinates of bodies or nodes; if distance=None, it is computed automatically from reference length; all markers are automatically computed."""
    ...

    @overload
    def CreateCoordinateConstraint(self, name='', bodyNumbers=[None, None], coordinates=[None, None], offset=0., factorValue1=1., velocityLevel=False, offsetUserFunction=0, offsetUserFunction_t=0, show=True, drawSize=-1., color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create coordinate constraint for two bodies, or body on ground; markers and NodePointGround are automatically created when needed."""
    ...

    @overload
    def CreateRollingDisc(self, name='', bodyNumbers=[None, None], axisPosition=[], axisVector=[1,0,0], discRadius=0., planePosition=[0,0,0], planeNormal=[0,0,1], constrainedAxes=[1,1,1], activeConnector=True, show=True, discWidth=0.1, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create an ideal rolling disc joint between wheel rigid body and ground; the disc is infinitely thin and the ground is a perfectly flat plane; the wheel may lift off; definition of joint position and axis in global coordinates (alternatively in wheel (body1) local coordinates) for reference configuration of bodies; all markers and other quantities are automatically computed; some constraint conditions may be deactivated, e.g. to resolve redundancy of constraints for multi-wheel vehicles."""
    ...

    @overload
    def CreateRollingDiscPenalty(self, name='', bodyNumbers=[None, None], axisPosition=[], axisVector=[1,0,0], discRadius=0., planePosition=[0,0,0], planeNormal=[0,0,1], contactStiffness=0., contactDamping=0., dryFriction=[0,0], dryFrictionAngle=0., dryFrictionProportionalZone=0., viscousFriction=[0,0], rollingFrictionViscous=0., useLinearProportionalZone=False, activeConnector=True, show=True, discWidth=0.1, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create penalty-based rolling disc joint between wheel rigid body and ground; the disc is infinitely thin and the ground is a perfectly flat plane; the wheel may lift off; definition of joint position and axis in global coordinates (alternatively in wheel (body1) local coordinates) for reference configuration of bodies; all markers and other quantities are automatically computed."""
    ...

    @overload
    def CreateSphereSphereContact(self, name='', bodyNumbers=[None, None], localPosition0=[0.,0.,0.], localPosition1=[0.,0.,0.], spheresRadii=[-1,-1], isHollowSphere1=False, dynamicFriction=0., frictionProportionalZone=1e-3, contactStiffness=0., contactDamping=0., contactStiffnessExponent=1, constantPullOffForce=0, contactPlasticityRatio=0, adhesionCoefficient=0, adhesionExponent=1, restitutionCoefficient=1, minimumImpactVelocity=0, impactModel=0, dataInitialCoordinates=[0,0,0,0], activeConnector=True, bodyOrNodeList=[None, None], show=False, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create penalty-based sphere-sphere contact between two rigid bodies, mass points (if friction coefficient is zero) or according nodes; the contact is based on ObjectContactSphereSphere; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems."""
    ...

    @overload
    def CreateSphereQuadContact(self, name='', bodyNumbers=[None, None], localPosition0=[0.,0.,0.], radiusSphere=0, quadPoints=exudyn.Vector3DList([[0,0,0],[1,0,0],[1,1,0],[0,1,0]]), includeEdges=15, dynamicFriction=0., frictionProportionalZone=1e-3, contactStiffness=0., contactDamping=0., contactStiffnessExponent=1, restitutionCoefficient=1, minimumImpactVelocity=0, impactModel=0, dataInitialCoordinates=[0,0,0,0], activeConnector=True, bodyOrNodeList=[None, None], localPosition1=[0.,0.,0.], show=False, color=exudyn.graphics.color.default) -> dict: 
        """Create penalty-based sphere-quad contact between two rigid bodies, mass points or according nodes; the contact is based on two ObjectContactSphereTriangle; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems."""
    ...

    @overload
    def CreateSphereTriangleContact(self, name='', bodyNumbers=[None, None], localPosition0=[0.,0.,0.], radiusSphere=0, trianglePoints=exudyn.Vector3DList([[0,0,0],[1,0,0],[0,1,0]]), includeEdges=7, dynamicFriction=0., frictionProportionalZone=1e-3, contactStiffness=0., contactDamping=0., contactStiffnessExponent=1, restitutionCoefficient=1, minimumImpactVelocity=0, impactModel=0, dataInitialCoordinates=[0,0,0,0], activeConnector=True, bodyOrNodeList=[None, None], localPosition1=[0.,0.,0.], show=False, color=exudyn.graphics.color.default) -> ObjectIndex: 
        """Create penalty-based sphere-triangle contact between two rigid bodies, mass points or according nodes; the contact is based on ObjectContactSphereTriangle; note that this approach is only intended to be used for small number of contact objects, while GeneralContact shall be used for large scale systems."""
    ...

    @overload
    def CreateKinematicTree(self, name='', listOfTreeLinks=[], referenceCoordinates=None, initialCoordinates=None, initialCoordinates_t=None, gravity=[0.,0.,0.], baseOffset=[0.,0.,0.], linkForces=None, linkTorques=None, jointForceVector=None, jointPositionOffsetVector=None, jointVelocityOffsetVector=None, forceUserFunction=0, jointRadius=0.05, jointWidth=0.12, colors=exudyn.graphics.color.default, colorsJoints=exudyn.graphics.color.default, baseGraphicsDataList=None, linkRoundness=0.2, show=True) -> ObjectIndex: 
        """helper function to create 2D or 3D mass point object and node, using arguments as in NodePoint and MassPoint; uses TreeLink as defined in exudyn.rigidBodyUtilities."""
    ...

    @overload
    def CreateFFRFReducedOrderObject(self, name, femInterface, referencePosition=[0., 0., 0.], initialVelocity=[0., 0., 0.], referenceRotationMatrix=np.eye(3), initialAngularVelocity=[0., 0., 0.], massProportionalDamping=0., stiffnessProportionalDamping=0., gravity=[0., 0., 0.], color=exudyn.graphics.color.defaultFFRF, superElementRigidMarkersOffsets=None, showMarkers=True, verbose=False) -> dict: 
        """Create an FFRF reduced order object; the function adds SuperElementRigid markers if boundaries are defined in the given femInterface and thus enables straightforward integration of flexible bodies into a multibody system."""
    ...

    @overload
    def CreateForce(self, name='', bodyNumber=None, loadVector=[0.,0.,0.], localPosition=[0.,0.,0.], bodyFixed=False, loadVectorUserFunction=0, show=True) -> LoadIndex: 
        """helper function to create force applied to given body."""
    ...

    @overload
    def CreateTorque(self, name='', bodyNumber=None, loadVector=[0.,0.,0.], localPosition=[0.,0.,0.], bodyFixed=False, loadVectorUserFunction=0, show=True) -> LoadIndex: 
        """helper function to create torque applied to given body."""
    ...

    @overload
    def PlotSensor(self, sensorNumbers=[], components=0, xLabel='time (s)', yLabel=None, labels=[], colorCodeOffset=0, newFigure=True, closeAll=False, componentsX=[], title='', figureName='', fontSize=16, colors=[], lineStyles=[], lineWidths=[], markerStyles=[], markerSizes=[], markerDensity=0.08, rangeX=[], rangeY=[], majorTicksX=10, majorTicksY=10, offsets=[], factors=[], subPlot=[], sizeInches=[6.4,4.8], fileName='', useXYZcomponents=True, legendArgs=None, **kwargs) -> [Any, Any, Any, Any]: 
        """Helper function for direct and easy visualization of sensor outputs, without need for loading text files, etc.; PlotSensor can be used to simply plot, e.g., the measured x-Position over time in a figure. PlotSensor provides an interface to matplotlib (which needs to be installed). Default values of many function arguments can be changed using the exudyn.plot function PlotSensorDefaults(), see there for usage."""
    ...

    @overload
    def SolveStatic(self, simulationSettings=None, updateInitialValues=False, storeSolver=True, showHints=False, showCausingItems=True, autoAssemble=True) -> bool: 
        """solves the static mbs problem using simulationSettings; check theDoc.pdf for MainSolverStatic for further details of the static solver; this function is also available in exudyn (using exudyn.SolveStatic(...))."""
    ...

    @overload
    def SolveDynamic(self, simulationSettings=None, solverType=exudyn.DynamicSolverType.GeneralizedAlpha, updateInitialValues=False, storeSolver=True, showHints=False, showCausingItems=True, autoAssemble=True) -> bool: 
        """solves the dynamic mbs problem using simulationSettings and solver type; check theDoc.pdf for MainSolverImplicitSecondOrder for further details of the dynamic solver; this function is also available in exudyn (using exudyn.SolveDynamic(...))."""
    ...

    @overload
    def ComputeLinearizedSystem(self, simulationSettings=None, projectIntoConstraintNullspace=False, singularValuesTolerance=1e-12, returnConstraintJacobian=False, returnConstraintNullspace=False, autoAssemble=True) -> [ArrayLike, ArrayLike, ArrayLike]: 
        """compute linearized system of equations for ODE2 part of mbs, not considering the effects of algebraic constraints; for computation of eigenvalues and advanced computation with constrained systems, see ComputeODE2Eigenvalues; the current implementation is also able to project into the constrained space, however, this currently does not generally work with non-holonomic systems."""
    ...

    @overload
    def ComputeODE2Eigenvalues(self, simulationSettings=None, useSparseSolver=False, numberOfEigenvalues=0, constrainedCoordinates=[], convert2Frequencies=False, useAbsoluteValues=True, computeComplexEigenvalues=False, ignoreAlgebraicEquations=False, singularValuesTolerance=1e-12, autoAssemble=True) -> [ArrayLike, ArrayLike]: 
        """compute eigenvalues for unconstrained ODE2 part of mbs, which represent the square of the eigenfrequencies (in radiant) of the undamped system; the computation may include constraints in case that ignoreAlgebraicEquations=False (however, this currently does not generally work with non-holonomic systems); for algebraic constraints, however, a dense singular value decomposition of the constraint jacobian is used for the nullspace projection; the computation is done for the initial values of the mbs, independently of previous computations. If you would like to use the current state for the eigenvalue computation, you need to copy the current state to the initial state (using GetSystemState, SetSystemState, see theDoc.pdf); note that mass and stiffness matrices are computed in dense mode so far, while eigenvalues are computed according to useSparseSolver."""
    ...

    @overload
    def ComputeSystemDegreeOfFreedom(self, simulationSettings=None, threshold=1e-12, verbose=False, useSVD=False, autoAssemble=True) -> dict: 
        """compute system DOF numerically, considering Gr{'u}bler-Kutzbach formula as well as redundant constraints; uses numpy matrix rank or singular value decomposition of scipy (useSVD=True)."""
    ...

    @overload
    def CreateDistanceSensorGeometry(self, meshPoints, meshTrigs, rigidBodyMarkerIndex, searchTreeCellSize=[8,8,8]) -> int: 
        """Add geometry for distance sensor given by points and triangles (point indices) to mbs; use a rigid body marker where the geometry is put on;; Creates a GeneralContact for efficient search on background. If you have several sets of points and trigs, first merge them or add them manually to the contact."""
    ...

    @overload
    def CreateDistanceSensor(self, generalContactIndex, positionOrMarker, dirSensor, minDistance=-1e7, maxDistance=1e7, cylinderRadius=0, selectedTypeIndex=exudyn.ContactTypeIndex.IndexEndOfEnumList, storeInternal=False, fileName='', measureVelocity=False, addGraphicsObject=False, drawDisplaced=True, color=exudyn.graphics.color.red) -> SensorIndex: 
        """Function to create distance sensor based on GeneralContact in mbs; sensor can be either placed on absolute position or attached to rigid body marker; in case of marker, dirSensor is relative to the marker."""
    ...

    @overload
    def DrawSystemGraph(self, showLoads=True, showSensors=True, useItemNames=False, useItemTypes=False, addItemTypeNames=True, multiLine=True, fontSizeFactor=1., layoutDistanceFactor=3., layoutIterations=100, showLegend=True, tightLayout=True, showGraph=True, addItemData=False, addAnnotations=False) -> [Any, Any, Any]: 
        """helper function which draws system graph of a MainSystem (mbs); several options let adjust the appearance of the graph; the graph visualization uses randomizer, which results in different graphs after every run!"""
    ...


class Renderer:
    """The Renderer is the substructure of SystemContainer that collects rendering and visualization interaction, in particular starting and stopping the renderer, image retrieval and materials.

    Rendering is done for a single SystemContainer, which may include several MainSystems
    Note that visualizationSettings are directly accessible from the SystemContainer.
    """
    @overload
    def Start(self, verbose=0) -> bool: 
        """Start OpenGL rendering engine (in separate thread) for visualization of rigid or flexible multibody system; use verbose=1 to output information during OpenGL window creation; verbose=2 produces more output and verbose=3 gives a debug level; some of the information will only be seen in windows command (powershell) windows or linux shell, but not inside iPython of e.g., Spyder."""
        ...
    @overload
    def Stop(self) -> None: 
        """Stop OpenGL rendering engine; uses timeout in multithreading."""
        ...
    @overload
    def IsActive(self) -> bool: 
        """Returns True if GLFW renderer is available and running; otherwise False."""
        ...
    @overload
    def Attach(self) -> bool: 
        """Links the SystemContainer to the render engine, such that the changes in the graphics structure drawn upon updates, etc.; done automatically on creation of SystemContainer; return False, if no renderer exists (e.g., compiled without GLFW) or cannot be linked (if other SystemContainer already linked)."""
        ...
    @overload
    def Detach(self) -> bool: 
        """DEPRECATED; Releases the SystemContainer from the render engine; return True if successfully released, False if no GLFW available or detaching failed."""
        ...
    @overload
    def DoIdleTasks(self, waitSeconds: float=-1., printPauseMessage: bool=True) -> bool: 
        """Interrupt further computation until user input (Space, 'Q', Escape-key), representing a PAUSE function; this command runs a loop in the background to have active response of the render window, e.g., to open the visualization dialog or use the right-mouse-button; replaces former SC.WaitForRenderEngineStopFlag() and mbs.WaitForUserToContinue(); call this function in order to interact with Renderer window; use waitSeconds in order to run this idle tasks while animating a model (e.g., waitSeconds=0.04), use waitSeconds=0 without waiting, or use waitSeconds=-1 (default) to wait until window is closed; NOTE: may also first initialize renderState from visualizationSettings (if renderer is inactive).
        
        Examples:
            SC.renderer.DoIdleTasks()
        """
        ...
    @overload
    def EnableView(self, viewID: int, createWindow: bool=True) -> None: 
        """Enables a specified view (1,2 or 3) additionally to the main view (0); a window is created if createWindow=True, otherwise, the view is only enabled to use it with the raytracer; NOTE: additional views cause slightly more workload for the renderer which is why they are disabled by default; failure to create the view usually results in an exception; to check whether the view exists, check the according RenderState regarding viewEnabled and windowOpen."""
        ...
    @overload
    def DisableView(self, viewID: int) -> None: 
        """Disables a specified view (1,2 or 3) and (if created) closes the according window; NOTE: in case of renderer.Stop(), all views are closed; NOTE: if you still aim to use the view without the window, you have to enable the view again EnableView(..., createWindow=False)."""
        ...
    @overload
    def ZoomAll(self, computeMaxScene: bool=True, viewID: int=0) -> None: 
        """Send zoom all signal, which will perform zoom all at next redraw request; if renderer is inactive (renderer.IsActive()=0), it will perform computations for renderState, thus at the next RedrawAndGetImage() having the full view as with the OpenGL renderer; NOTE: in case of OpenGL, call ZoomAll() after renderer.Start(); NOTE: may also first initialize renderState from visualizationSettings (if renderer is inactive)."""
        ...
    @overload
    def SetModelView(self, zoom: float=0, rotationVector: [float,float,float]=[0,0,0], centerPoint: [float,float,float]=[0,0,0], viewID: int=0) -> None: 
        """Function to adjusts the current view in renderState; rotationVector and centerPoint transform the modelView while zoom equals the visible scene height; rotationVector is the axis of rotation times the angle in radiant; if zoom=0 then zoom will be computed automatically like in autoFitScene with openGL; use this function in particular for raytracing before RedrawAndGetImage() or with regular OpenGL after renderer.Start(); you can also store the renderState and write the full renderState alternatively; NOTE: may also first initialize renderState from visualizationSettings (if renderer is inactive).
        
        Examples:
            SC.renderer.SetModelView(10,[0,0,pi],[2.5,0,0])
            image=SC.renderer.RedrawAndGetImage()
        """
        ...
    @overload
    def RedrawAndSaveImage(self, viewID: int=0) -> None: 
        """Redraw openGL scene and save image (command waits until process is finished); uses the current rendering engine (OpenGL or raytracer)."""
        ...
    @overload
    def RedrawAndGetImage(self, useRaytracer: bool=False, viewID: int=0) -> NDArray[np.uint8]: 
        """Redraw scene and return image in numpy-format, containing a 3-dimensional array with 3 matrices of RGB channels; the shape is according to (height,width,3) where height and width represent the window pixels defined in visualizationSettings.window.renderWindowSize (Note: in case that raytracer.imageSizeFactor>1 the retrieved image size is smaller by the imageSizeFactor!); command waits until process is finished; if useRaytracer=False, the openGL render is used and the render window needs to be opened before; if useRaytracer=True, the software raytracer is used which runs completely without GLFW and OpenGL (e.g., on a supercomputer); NOTE: in case of useRaytracer=True the displayScaling factor is only available if the OpenGL renderer has been opened once; otherwise e.g., SC.renderer.SetState({'displayScaling':1.5}) has to be used to adjust it; NOTE: may also first initialize renderState from visualizationSettings (if renderer is inactive).
        
        Examples:
            import matplotlib.pyplot as plt
            image=SC.renderer.RedrawAndGetImage()
            plt.imshow(image)
            plt.axis('off')
            plt.show()
        """
        ...
    @overload
    def GetState(self, viewID: int=0) -> dict: 
        """Get dictionary with current render state (openGL zoom, modelview, etc.).
        
        Examples:
            SC = exu.SystemContainer()
            renderState = SC.renderer.GetState()
            print(renderState['zoom'])
        """
        ...
    @overload
    def SetState(self, renderState: dict, waitForRendererFullStartup: bool=True, viewID: int=0) -> None: 
        """Set current render state (openGL zoom, modelview, etc.) with given dictionary; usually, this dictionary has been obtained with GetRenderState; waitForRendererFullStartup is used to wait at startup for the first frame to be drawn (and zoom all to be set), but be be set False in case of performance issues; NOTE: before setting available state values, this function may also first initialize renderState from visualizationSettings (if renderer is inactive).
        
        Examples:
            SC = exu.SystemContainer()
            SC.renderer.SetState(renderState)
        """
        ...
    @overload
    def GetMouseCoordinates(self, useOpenGLcoordinates: bool=False, viewID: int=0) -> [float,float]: 
        """Get current mouse coordinates as list [x, y]; x and y being floats, as returned by GLFW, measured from top left corner of window; use GetCurrentMouseCoordinates(useOpenGLcoordinates=True) to obtain OpenGLcoordinates of projected plane."""
        ...
    @overload
    def GetItemSelection(self, resetSelection: bool=True, viewID: int=0) -> [int,int,int,float]: 
        """Get selected item in render state; option to reset selected item afterwards; item is selected in render window by clicking left mouse button; returns [mbs number, ItemType, ItemIndex, depth] where depth is the Z-depth in the current view; note that only items of the categories activated in visualizationSettings.interactive.selectionLeftMouseItemTypes are returned; NOTE: if itemType == 0, no item has been selected."""
        ...
    @overload
    def ResetState(self) -> None: 
        """Reset renderState in all views to default values using current visualizationSettings; usually this does not have to be called!."""
        ...
    @overload
    def SendRedrawSignal(self) -> None: 
        """This function is used to send a signal to the renderer that all MainSystems (mbs) shall be redrawn."""
        ...
    @overload
    def GetRenderCount(self) -> bool: 
        """Returns the number of rendered OpenGL images; can be used to determine if image has been drawn by comparing to previous counter; also shows that first image has been drawn (needed for zoom all)."""
        ...
    materials:GraphicsMaterialList
    """GraphicsMaterialList used for raytracer (possibly for OpenGL in future); list can be accessed with [] operator, reset and extended. Note that after Reset() there are at least 10 materials available, which are copied from visualizationSettings.raytracer.materials which are synced continuously."""


class SystemContainer:
    """The SystemContainer is the top level of structures in Exudyn.

    The container holds all (multibody) systems of type ``MainSystem`` and the link to OpenGL renderers and raytracers (every SystemContainer has an independent rendering, while all MainSystems are rendered together).Via the MainSystems it thus contains all computational data
    A SystemContainer is created by ``SC = exu.SystemContainer()``, understanding ``exu.SystemContainer`` as a state machine where MainSystems are added and renderer state machines are processed, similar to the behavior of other Python packages
    Usually, only one container shall be used, while multiple containers are possible -- e.g., for reasons of significantly different behavior (drawing, etc.)
    The SystemContainer contains ``visualizationSettings`` to adjust all kinds of visualization appearance, windows and interactions.
    """
    @overload
    def Reset(self) -> None: 
        """Delete all multibody systems and reset SystemContainer (including graphics); this also releases SystemContainer from the renderer, which requires SC.renderer.Attach() to be called in order to reconnect to rendering; a safer way is to delete the current SystemContainer and create a new one (SC=SystemContainer() )."""
        ...
    @overload
    def AddSystem(self) -> MainSystem: 
        """Add a new computational system."""
        ...
    @overload
    def Append(self, mainSystem: MainSystem) -> int: 
        """Append an exsiting computational system to the system container; returns the number of MainSystem in system container."""
        ...
    @overload
    def NumberOfSystems(self) -> int: 
        """Obtain number of multibody systems available in system container."""
        ...
    @overload
    def GetSystem(self, systemNumber: int) -> MainSystem: 
        """Obtain multibody systems with index from system container."""
        ...
    visualizationSettings:VisualizationSettings
    """this structure is read/writeable and contains visualization settings, which are immediately applied to the rendering window. ; ; EXAMPLE:; ; SC = exu.SystemContainer(); ; SC.visualizationSettings.autoFitScene=False."""
    @overload
    def GetDictionary(self) -> dict: 
        """[UNDER DEVELOPMENT]: return the dictionary of the system container data, e.g., to copy the system or for pickling."""
        ...
    @overload
    def SetDictionary(self, systemDict: dict) -> None: 
        """[UNDER DEVELOPMENT]: set system container data from given dictionary; used for pickling."""
        ...
    renderer:Renderer
    """The substructure in SystemContainer responsible for rendering (except visualizationSettings)."""
    visualizationSettings:VisualizationSettings
    """Structure representing the settings for renderer; for details of visualizationSettings see Section Structures and Settings."""

