#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN definition file
#
# Details:  the GeneralContact and VisuGeneralContact classes.
#           The calls are recorded by PybindInterface (pybindTypes.py) and replayed by
#           tools/generators/pybindEmitter.py into pybind_manual_classes.h, the stub fragments and
#           the Python-C++ interface documentation.
#
#           DESCRIPTIONS: read definitions/README.md, section "Writing a
#           description", before writing or changing one - what the text may
#           contain, and how it is checked.
#
# Author:   Johannes Gerstmayr
# Date:     2018-05-18 (created in autoGeneratePyBindings.py), 2026-09-14 (moved to definitions/)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from pybindTypes import *

pb = PybindInterface()



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++













#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
pb.CreateNewRSTfile('GeneralContact')
classStr = 'PyGeneralContact'
pyClassStr = 'GeneralContact'
pb.DefPyStartClass(classStr, pyClassStr, 
                    'Structure to define general and highly efficient contact functionality in multibody systems, allowing millions of particles, using search trees and parallelized contact computations, mainly intended for explicit solvers. ',
                    labelName='sec:GeneralContact',
                    forbidPythonConstructor=True)

pb.AddDocu(
            'For further explanations and theoretical backgrounds, see [](#seccontacttheory). '+
            'Internally, the contacts are stored with global indices, which are in the following list: '+
            '[numberOfSpheresMarkerBased, numberOfANCFCable2D, numberOfTrigsRigidBodyBased], see also'+
            'the output of GetPythonObject().')

pb.AddDocuCodeBlock(code="""
#...
#code snippet, must be placed anywhere before mbs.Assemble()
#Add GeneralContact to mbs:
gContact = mbs.AddGeneralContact()
#Add contact elements, e.g.:
gContact.AddSphereWithMarker(...) #use appropriate arguments
gContact.SetFrictionPairings(...) #set friction pairings and adjust searchTree if needed.
""")

pb.DefStartTable(pyClassStr)

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                                description="convert member variables of GeneralContact into dictionary; use this for debug only!",
                                returnType='dict',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='Reset', cName='Reset', 
                                argList=['freeMemory'],
                                defaultArgs=['True'],
                                description="remove all contact objects and reset contact parameters",
                                returnType='None',
                                )

pb.CppCode('        .def_readwrite("isActive", &PyGeneralContact::isActive, py::return_value_policy::reference)\n') 
pb.DefDataAccess('isActive','default = True (compute contact); if isActive=False, no contact computation is performed for this contact set ',
                       dataType='bool',
                       )

pb.CppCode('        .def_readwrite("verboseMode", &PyGeneralContact::verboseMode, py::return_value_policy::reference)\n') 
pb.DefDataAccess('verboseMode','default = 0; verboseMode = 1 or higher outputs useful information on the contact creation and computation ',
                       dataType='int',
                       )

pb.CppCode('        .def_readwrite("visualization", &PyGeneralContact::visualization, py::return_value_policy::reference)\n') 
pb.DefDataAccess('visualization','access visualization data structure ',
                       dataType='VisuGeneralContact',
                       )

pb.CppCode('        .def_property("resetSearchTreeInterval", &PyGeneralContact::GetResetSearchTreeInterval, &PyGeneralContact::SetResetSearchTreeInterval)\n') 
pb.DefDataAccess('resetSearchTreeInterval','(default=10000) number of search tree updates (contact computation steps) after which the search tree cells are re-created; this costs some time, will free memory in cells that are not needed any more ',
                       dataType='int',
                       )

pb.CppCode('        .def_property("sphereSphereContact", &PyGeneralContact::GetSphereSphereContact, &PyGeneralContact::SetSphereSphereContact)\n') 
pb.DefDataAccess('sphereSphereContact','activate/deactivate contact between spheres ',
                       dataType='bool',
                       )

pb.CppCode('        .def_property("sphereSphereFrictionRecycle", &PyGeneralContact::GetSphereSphereFrictionRecycle, &PyGeneralContact::SetSphereSphereFrictionRecycle)\n') 
pb.DefDataAccess('sphereSphereFrictionRecycle','False: compute static friction force based on tangential velocity; True: recycle friction from previous PostNewton step, which greatly improves convergence, but may lead to unphysical artifacts; will be solved in future by step reduction ',
                       dataType='bool',
                       )

pb.CppCode('        .def_property("minRelDistanceSpheresTriangles", &PyGeneralContact::GetMinRelDistanceSpheresTriangles, &PyGeneralContact::SetMinRelDistanceSpheresTriangles)\n') 
pb.DefDataAccess('minRelDistanceSpheresTriangles','(default=1e-10) tolerance (relative to sphere radiues) below which the contact between triangles and spheres is ignored; used for spheres directly attached to triangles ',
                       dataType='float',
                       )

pb.CppCode('        .def_property("frictionProportionalZone", &PyGeneralContact::GetFrictionProportionalZone, &PyGeneralContact::SetFrictionProportionalZone)\n') 
pb.DefDataAccess('frictionProportionalZone',r"""(default=0.001) velocity $v_{\mu,reg}$ upon which the dry friction coefficient is interpolated linearly (regularized friction model); must be greater 0; very small values cause oscillations in friction force """,
                       dataType='float',
                       )

# pb.DefDataAccess('frictionVelocityPenalty','(default=1e3) regularization factor for friction [N/(m$^2 \cdot$m/s) ];$k_{\mu,reg}$, multiplied with tangential velocity to compute friciton force as long as it is smaller than $\mu$ times contact force; large values cause oscillations in friction force ',
#                        dataType='float',
#                        )

pb.CppCode('        .def_property("excludeOverlappingTrigSphereContacts", &PyGeneralContact::GetExcludeOverlappingTrigSphereContacts, &PyGeneralContact::SetExcludeOverlappingTrigSphereContacts)\n') 
pb.DefDataAccess('excludeOverlappingTrigSphereContacts','(default=True) for consistent, closed meshes, we can exclude overlapping contact triangles (which would cause holes if mesh is overlapping and not consistent!!!) ',
                       dataType='bool',
                       )

pb.CppCode('        .def_property("excludeDuplicatedTrigSphereContactPoints", &PyGeneralContact::GetExcludeDuplicatedTrigSphereContactPoints, &PyGeneralContact::SetExcludeDuplicatedTrigSphereContactPoints)\n') 
pb.DefDataAccess('excludeDuplicatedTrigSphereContactPoints','(default=False) run additional checks for double contacts at edges or vertices, being more accurate but can cause additional costs if many contacts ',
                       dataType='bool',
                       )
pb.CppCode('        .def_property("computeExactStaticTriangleBins", &PyGeneralContact::GetComputeExactStaticTriangleBins, &PyGeneralContact::SetComputeExactStaticTriangleBins)\n') 
pb.DefDataAccess('computeExactStaticTriangleBins','(default=True) if True, search tree bins are computed exactly for static triangles while if False, it uses the overall (=very inaccurate) AABB of each triangle in the search tree',
                       dataType='bool',
                       )

pb.CppCode('        .def_property("computeContactForces", &PyGeneralContact::GetComputeContactForces, &PyGeneralContact::SetComputeContactForces)\n') 
pb.DefDataAccess('computeContactForces','(default=False) if True, additional system vector is computed which contains all contact force and torque contributions. In order to recover forces on a single rigid body, the respective LTG-vector has to be used and forces need to be extracted from this system vector; may slow down computations.',
                       dataType='bool',
                       )

pb.CppCode('        .def_property("ancfCableUseExactMethod", &PyGeneralContact::GetAncfCableUseExactMethod, &PyGeneralContact::SetAncfCableUseExactMethod)\n') 
pb.DefDataAccess('ancfCableUseExactMethod','(default=True) if True, uses exact computation of intersection of 3rd order polynomials and contacting circles ',
                       dataType='bool',
                       )

pb.CppCode('        .def_property("ancfCableNumberOfContactSegments", &PyGeneralContact::GetAncfCableNumberOfContactSegments, &PyGeneralContact::SetAncfCableNumberOfContactSegments)\n') 
pb.DefDataAccess('ancfCableNumberOfContactSegments','(default=1) number of segments to be used in case that ancfCableUseExactMethod=False; maximum number of segments=3 ',
                       dataType='int',
                       )

pb.CppCode('        .def_property("ancfCableMeasuringSegments", &PyGeneralContact::GetAncfCableMeasuringSegments, &PyGeneralContact::SetAncfCableMeasuringSegments)\n') 
pb.DefDataAccess('ancfCableMeasuringSegments','(default=20) number of segments used to approximate geometry for ANCFCable2D elements for measuring with ShortestDistanceAlongLine; with 20 segments the relative error due to approximation as compared to 10 segments usually stays below 1e-8 ',
                       dataType='int',
                       )
#+++++++++++++++++
#parallel:
pb.CppCode('        .def_property("parallelTaskSplit", &PyGeneralContact::GetParallelTaskSplit, &PyGeneralContact::SetParallelTaskSplit)\n') 
pb.DefDataAccess('parallelTaskSplit','(default=12) general number of tasks per thread (min)',
                       dataType='int',
                       )
pb.CppCode('        .def_property("parallelTaskSplitBoundingBoxes", &PyGeneralContact::GetParallelTaskSplitBoundingBoxes, &PyGeneralContact::SetParallelTaskSplitBoundingBoxes)\n') 
pb.DefDataAccess('parallelTaskSplitBoundingBoxes','(default=48) number of tasks per thread for bounding box computations',
                       dataType='int',
                       )
pb.CppCode('        .def_property("parallelTaskSplitThreshold", &PyGeneralContact::GetParallelTaskSplitThreshold, &PyGeneralContact::SetParallelTaskSplitThreshold)\n') 
pb.DefDataAccess('parallelTaskSplitThreshold','(default=12) general threshold below which only one task per thread is used',
                       dataType='int',
                       )
pb.CppCode('        .def_property("parallelTaskSplitBoundingBoxesThreshold", &PyGeneralContact::GetParallelTaskSplitBoundingBoxesThreshold, &PyGeneralContact::SetParallelTaskSplitBoundingBoxesThreshold)\n') 
pb.DefDataAccess('parallelTaskSplitBoundingBoxesThreshold','(default=400) threshold below which only one task per thread is used, for bounding box computations',
                       dataType='int',
                       )
#+++++++++++++++++


# pb.DefPyFunctionAccess(cClass=classStr, pyName='FinalizeContact', cName='PyFinalizeContact', 
#                                 argList=['mainSystem','searchTreeSize','frictionPairingsInit','searchTreeBoxMin','searchTreeBoxMax'],
#                                 defaultArgs=['','','', '(std::vector<Real>)Vector3D( EXUstd::MAXREAL )','(std::vector<Real>)Vector3D( EXUstd::LOWESTREAL )'],
#                                 description="WILL CHANGE IN FUTURE: Call this function after mbs.Assemble(); precompute some contact arrays (mainSystem needed) and set up necessary parameters for contact: friction, SearchTree, etc.; done after all contacts have been added; function performs checks; empty box will autocompute size!")
pb.DefPyFunctionAccess(cClass=classStr, pyName='SetFrictionPairings', cName='SetFrictionPairings', 
                        argList=['frictionPairings'],
                        example=r"""#set 3 surface friction types, all being 0.1:\\gContact.SetFrictionPairings(0.1*np.ones((3,3)));""",
                        description="set Coulomb friction coefficients for pairings of materials (e.g., use material 0,1, then the entries (0,1) and (1,0) define the friction coefficients for this pairing); matrix should be symmetric!",
                        argTypes=['ArrayLike'],
                        returnType='None',
                        )

#this could be removed, accessed by variable directly:
pb.DefPyFunctionAccess(cClass=classStr, pyName='SetFrictionProportionalZone', cName='SetFrictionProportionalZone', 
                        argList=['frictionProportionalZone'],
                        description="regularization for friction (m/s); used for all contacts",
                        argTypes=['float'],
                        returnType='None',
                        )
                                   
pb.DefPyFunctionAccess(cClass=classStr, pyName='SetSearchTreeCellSize', cName='SetSearchTreeCellSize', 
                        argList=['numberOfCells'],
                        example='gContact.SetSearchTreeInitSize([10,10,10])',
                        description="set number of cells of search tree (boxed search) in x, y and z direction",
                        argTypes=['[int,int,int]'],
                        returnType='None',
                        )
                                                   
pb.DefPyFunctionAccess(cClass=classStr, pyName='SetSearchTreeBox', cName='SetSearchTreeBox', 
                        argList=['pMin','pMax'],
                        example=r"""gContact.SetSearchTreeBox(pMin=[-1,-1,-1],\\ \TAB pMax=[1,1,1])""",
                        description="set geometric dimensions of searchTreeBox (point with minimum coordinates and point with maximum coordinates); if this box becomes smaller than the effective contact objects, contact computations may slow down significantly",
                        argTypes=[vector3D,vector3D],
                        returnType='None',
                       )
                                              
pb.DefPyFunctionAccess(cClass=classStr, pyName='AddSphereWithMarker', cName='AddSphereWithMarker', 
                        argList=['markerIndex','radius','contactStiffness','contactDamping','frictionMaterialIndex'],
                        description="add contact object using a marker (Position or Rigid), radius and contact/friction parameters and return localIndex of the contact item in GeneralContact; frictionMaterialIndex refers to frictionPairings in GeneralContact; contact is possible between spheres (circles in 2D) (if intraSphereContact = True), spheres and triangles and between sphere (=circle) and ANCFCable2D; contactStiffness is computed as serial spring between contacting objects, while damping is computed as a parallel damper",
                        argTypes=['MarkerIndex','float','float','float','int'],
                        returnType='int',
                        )
                                              
pb.DefPyFunctionAccess(cClass=classStr, pyName='AddANCFCable', cName='AddANCFCable', 
                        argList=['objectIndex','halfHeight','contactStiffness','contactDamping','frictionMaterialIndex'],
                        description="add contact object for an ANCF cable element, using the objectIndex of the cable element and the cable's half height as an additional distance to contacting objects (currently not causing additional torque in case of friction), and return localIndex of the contact item in GeneralContact; currently only contact with spheres (circles in 2D) possible; contact computed using exact geometry of elements, finding max 3 intersecting contact regions",
                        argTypes=['ObjectIndex','float','float','float','int'],
                        returnType='int',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='AddTrianglesRigidBodyBased', cName='PyAddTrianglesRigidBodyBased', 
                        argList=['rigidBodyMarkerIndex','contactStiffness','contactDamping','frictionMaterialIndex','pointList','triangleList','staticTriangles'],
                        description="add contact object using a rigidBodyMarker (of a body), contact/friction parameters, a list of points (as 3D numpy arrays or lists; coordinates relative to rigidBodyMarker) and a list of triangles (3 indices as numpy array or list) according to a mesh attached to the rigidBodyMarker; the flag staticTriangles=True can be used to inform the contact solver that these triangles are static (fixed in space); note that static triangles have to be added before dynamic triangles; function returns starting local index of trigsRigidBodyBased at which the triangles are stored; mesh can be produced with GraphicsData2TrigsAndPoints(...); contact is possible between sphere (circle) and Triangle but yet not between triangle and triangle; frictionMaterialIndex refers to frictionPairings in GeneralContact; contactStiffness is computed as serial spring between contacting objects, while damping is computed as a parallel damper (otherwise the smaller damper would always dominate); the triangle normal must point outwards, with the normal of a triangle given with local points (p0,p1,p2) defined as n=(p1-p0) x (p2-p0), see function ComputeTriangleNormal(...)",
                        argTypes=['MarkerIndex','float','float','int','List[[float,float,float]]','List[[int,int,int]]','bool'],
                        defaultArgs=['','','','','','','False'],
                        returnType='int',
                        )

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#access functions:
pb.DefPyFunctionAccess(cClass=classStr, pyName='GetItemsInBox', cName='PyGetItemsInBox', 
                        argList=['pMin','pMax'],
                        example=r"""gContact.GetItemsInBox(pMin=[0,1,1],\\ \TAB pMax=[2,3,2])""",
                        description="Get all items in box defined by minimum coordinates given in pMin and maximum coordinates given by pMax, accepting 3D lists or numpy arrays; in case that no objects are found, False is returned; otherwise, a dictionary is returned, containing numpy arrays with indices of obtained MarkerBasedSpheres, TrigsRigidBodyBased, ANCFCable2D, ...; the indices refer to the local index in GeneralContact which can be evaluated e.g., by GetMarkerBasedSphere(localIndex)",
                        argTypes=[vector3D,vector3D],
                        returnType='Union[dict,bool]',
                        )

#++++++++++++++++++++++++++++++++++++++++++++++++++++

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetSphereMarkerBased', cName='PyGetSphereMarkerBased', 
                        argList=['localIndex','addData'],
                        description="Get dictionary with current position, orientation, velocity, angular velocity as computed in last contact iteration; if addData=True, adds stored data of contact element, such as radius, markerIndex and contact parameters; localIndex is the internal index of contact element, as returned e.g., from GetItemsInBox",
                        argTypes=['int','bool'],
                        defaultArgs=['','False'],
                        returnType='dict',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetSphereMarkerBased', cName='PySetSphereMarkerBased', 
                        argList=['localIndex','contactStiffness','contactDamping','radius','frictionMaterialIndex'],
                        description="Set data of marker based sphere with localIndex (as internally stored) with given arguments; arguments that are < 0 (default) imply that current values are not overwritten",
                        argTypes=['int','float','float','float','int'],
                        defaultArgs=['','-1.','-1.','-1.','-1'],
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetTriangleRigidBodyBased', cName='PyGetTriangleRigidBodyBased', 
                        argList=['localIndex'],
                        description="Get dictionary with rigid body index, local position of triangle vertices (nodes) and triangle normal; NOTE: the mesh added to contact is different from this structure, as it contains nodes and connectivity lists; the triangle index corresponds to the order as triangles are added to GeneralContact",
                        argTypes=['int'],
                        defaultArgs=[''],
                        returnType='dict',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetTriangleRigidBodyBased', cName='PySetTriangleRigidBodyBased', 
                        argList=['localIndex','points','contactRigidBodyIndex'],
                        description="Set data of marker based sphere with localIndex (triangle index); points are provided as 3x3 numpy array, with point coordinates in rows; contactRigidBodyIndex<0 indicates no change of the current index (and changing this index should be handled with care)",
                        argTypes=['int',matrix3D,'int'],
                        defaultArgs=['','','-1'],
                        returnType='None',
                        )

#++++++++++++++++++++++++++++++++++++++++++++++++++++

pb.DefPyFunctionAccess(cClass=classStr, pyName='ShortestDistanceAlongLine', cName='PyShortestDistanceAlongLine', 
                        argList=['pStart','direction','minDistance','maxDistance','asDictionary','cylinderRadius','typeIndex'],
                        defaultArgs=['(std::vector<Real>)Vector3D({0,0,0})','(std::vector<Real>)Vector3D({1,0,0})','-1e-7','1e7','False','0','Contact::IndexEndOfEnumList'],
                        description="Find shortest distance to contact objects in GeneralContact along line with pStart (given as 3D list or numpy array) and direction (as 3D list or numpy array with no need to be normalized); the function returns the distance which is >= minDistance and < maxDistance; in case of beam elements, it measures the distance to the beam centerline; the distance is measured from pStart along given direction and can also be negative; if no item is found along line, the maxDistance is returned; if asDictionary=False, the result is a float, while otherwise details are returned as dictionary (including distance, velocityAlongLine (which is the object velocity in given direction and may be different from the time derivative of the distance; works similar to a laser Doppler vibrometer - LDV), itemIndex and itemType in GeneralContact); the cylinderRadius, if not equal to 0, will be used for spheres to find closest sphere along cylinder with given point and direction; the typeIndex can be set to a specific contact type, e.g., which are searched for (otherwise all objects are considered)",
                        argTypes=[vector3D,vector3D,'float','float','bool','float','ContactTypeIndex'],
                        returnType='Union[dict,float]',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='UpdateContacts', cName='PyUpdateContacts', 
                        argList=['mainSystem'],
                        example='gContact.UpdateContacts(mbs)',
                        description="Update contact sets, e.g., if no contact is simulated (isActive=False) but user functions need up-to-date contact states for GetItemsInBox(...) or for GetActiveContacts(...)",
                        argTypes=['"MainSystem"'], #MainSystem undefined at this point
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetActiveContacts', cName='PyGetActiveContacts', 
                        argList=['typeIndex', 'itemIndex'],
                        example=r"""#if explicit solver is used, we first need to update contacts:\\gContact.UpdateContacts(mbs)\\#obtain active contacts of marker based sphere 42:\\gList = gContact.GetActiveContacts(exu.ContactTypeIndex.IndexSpheresMarkerBased, 42)""",
                        description="Get list of global item numbers which are in contact with itemIndex of type typeIndex in case that the global itemIndex is smaller than the abs value of the contact pair index; a negative sign indicates that the contacting (spheres) is in Coloumb friction, a positive sign indicates a regularized friction region; in case of itemIndex==-1, it will return the list of numbers of active contacts per item for the contact type; for interpretation of global contact indices, see gContact.GetPythonObject() and documentation; requires either implicit contact computation or UpdateContacts(...) needs to be called prior to this function",
                        argTypes=['ContactTypeIndex','int'],
                        returnType='List[int]',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetSystemODE2RhsContactForces', cName='PyGetSystemODE2RhsContactForces', 
                        argList=['copy'],
                        description="Get numpy array of system vector containing contribution of contact forces to system ODE2 Rhs vector; if copy=False, it will give direct (reference) access to the internal vector (note: modifications to this vector do not influence simulation!), however, which may cause problems if the system size changes or simulation is restarted; if copy=True, the vector is copied (time consuming); contributions to single objects may be extracted by checking the according LTG-array of according objects (such as rigid bodies); the contact forces vector is computed in each contact iteration;",
                        argTypes=['bool'],
                        defaultArgs=['False'],
                        returnType=returnedArray,
                        )

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                                              
pb.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', cName='[](const PyGeneralContact &item) {\n            return EXUstd::ToString(item); }', 
                        description="return the string representation of the GeneralContact, containing basic information and statistics",
                        isLambdaFunction = True)


#++++++++++++++++
pb.DefPyFinishClass('GeneralContact')

pb.EndStubSection()


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for VisuGeneralContact
classStr = 'VisuGeneralContact'
pyClassStr = 'VisuGeneralContact'
pb.DefPyStartClass(classStr, pyClassStr, 'This structure may contains some visualization parameters in future. '+
                    'Currently, all visualization settings are controlled via SC.visualizationSettings', 
                    subSection=True, labelName='sec:GeneralContact:visualization')

pb.DefStartTable(pyClassStr)

pb.DefPyFunctionAccess(cClass=classStr, pyName='Reset', cName='Reset', 
                        description="reset visualization parameters to default values",
                        returnType='None',
                        )

# pb.DefDataAccess('spheresMarkerBasedDraw','default = False; if True, markerBased spheres are drawn with given resolution and color ')

# pb.DefDataAccess('spheresMarkerBasedResolution','default = 4; integer value for number of triangles per circumference of markerBased spheres; higher values leading to smoother spheres but higher graphics costs ')

# pb.DefDataAccess('spheresMarkerBasedColor','vector with 4 floats (Float4) for color of markerBased spheres ')

#++++++++++++++++
pb.DefPyFinishClass('GeneralContact')
pb.EndStubSection()
