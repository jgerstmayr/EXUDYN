
#stub information for class VisuGeneralContact functions
class VisuGeneralContact:
    """This structure may contains some visualization parameters in future.

    Currently, all visualization settings are controlled via SC.visualizationSettings
    """
    @overload
    def Reset(self) -> None: 
        """Reset visualization parameters to default values."""
        ...

#stub information for class GeneralContact functions
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

#stub information for class SystemData functions
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

#stub information for class MainSystem functions
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

#stub information for class Renderer functions
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

#stub information for class SystemContainer functions
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

#stub information for exudyn module functions
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
