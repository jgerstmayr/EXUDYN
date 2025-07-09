
.. _sec-generalcontact:


**************
GeneralContact
**************




Structure to define general and highly efficient contact functionality in multibody systems\ (Note that GeneralContact is still developed, use with care.). For further explanations and theoretical backgrounds, see Section :ref:`seccontacttheory`\ . Internally, the contacts are stored with global indices, which are in the following list: [numberOfSpheresMarkerBased, numberOfANCFCable2D, numberOfTrigsRigidBodyBased], see alsothe output of GetPythonObject().

.. code-block:: python
   :linenos:
   
   #...
   #code snippet, must be placed anywhere before mbs.Assemble()
   #Add GeneralContact to mbs:
   gContact = mbs.AddGeneralContact()
   #Add contact elements, e.g.:
   gContact.AddSphereWithMarker(...) #use appropriate arguments
   gContact.SetFrictionPairings(...) #set friction pairings and adjust searchTree if needed.

\ The class **GeneralContact** has the following **functions and structures**:

* | **GetPythonObject**\ (): 
  | convert member variables of GeneralContact into dictionary; use this for debug only!
* | **Reset**\ (\ *freeMemory*\  = True): 
  | remove all contact objects and reset contact parameters
* | **isActive**:
  | default = True (compute contact); if isActive=False, no contact computation is performed for this contact set 
* | **verboseMode**:
  | default = 0; verboseMode = 1 or higher outputs useful information on the contact creation and computation 
* | **visualization**:
  | access visualization data structure 
* | **resetSearchTreeInterval**:
  | (default=10000) number of search tree updates (contact computation steps) after which the search tree cells are re-created; this costs some time, will free memory in cells that are not needed any more 
* | **sphereSphereContact**:
  | activate/deactivate contact between spheres 
* | **sphereSphereFrictionRecycle**:
  | False: compute static friction force based on tangential velocity; True: recycle friction from previous PostNewton step, which greatly improves convergence, but may lead to unphysical artifacts; will be solved in future by step reduction 
* | **minRelDistanceSpheresTriangles**:
  | (default=1e-10) tolerance (relative to sphere radiues) below which the contact between triangles and spheres is ignored; used for spheres directly attached to triangles 
* | **frictionProportionalZone**:
  | (default=0.001) velocity \ :math:`v_{\mu,reg}`\  upon which the dry friction coefficient is interpolated linearly (regularized friction model); must be greater 0; very small values cause oscillations in friction force 
* | **excludeOverlappingTrigSphereContacts**:
  | (default=True) for consistent, closed meshes, we can exclude overlapping contact triangles (which would cause holes if mesh is overlapping and not consistent!!!) 
* | **excludeDuplicatedTrigSphereContactPoints**:
  | (default=False) run additional checks for double contacts at edges or vertices, being more accurate but can cause additional costs if many contacts 
* | **computeExactStaticTriangleBins**:
  | (default=True) if True, search tree bins are computed exactly for static triangles while if False, it uses the overall (=very inaccurate) AABB of each triangle in the search tree
* | **computeContactForces**:
  | (default=False) if True, additional system vector is computed which contains all contact force and torque contributions. In order to recover forces on a single rigid body, the respective LTG-vector has to be used and forces need to be extracted from this system vector; may slow down computations.
* | **ancfCableUseExactMethod**:
  | (default=True) if True, uses exact computation of intersection of 3rd order polynomials and contacting circles 
* | **ancfCableNumberOfContactSegments**:
  | (default=1) number of segments to be used in case that ancfCableUseExactMethod=False; maximum number of segments=3 
* | **ancfCableMeasuringSegments**:
  | (default=20) number of segments used to approximate geometry for ANCFCable2D elements for measuring with ShortestDistanceAlongLine; with 20 segments the relative error due to approximation as compared to 10 segments usually stays below 1e-8 
* | **parallelTaskSplit**:
  | (default=12) general number of tasks per thread (min)
* | **parallelTaskSplitBoundingBoxes**:
  | (default=48) number of tasks per thread for bounding box computations
* | **parallelTaskSplitThreshold**:
  | (default=12) general threshold below which only one task per thread is used
* | **parallelTaskSplitBoundingBoxesThreshold**:
  | (default=400) threshold below which only one task per thread is used, for bounding box computations
* | **SetFrictionPairings**\ (\ *frictionPairings*\ ): 
  | set Coulomb friction coefficients for pairings of materials (e.g., use material 0,1, then the entries (0,1) and (1,0) define the friction coefficients for this pairing); matrix should be symmetric!
  | *Example*:

  .. code-block:: python

     #set 3 surface friction types, all being 0.1:
     gContact.SetFrictionPairings(0.1*np.ones((3,3)));

* | **SetFrictionProportionalZone**\ (\ *frictionProportionalZone*\ ): 
  | regularization for friction (m/s); used for all contacts
* | **SetSearchTreeCellSize**\ (\ *numberOfCells*\ ): 
  | set number of cells of search tree (boxed search) in x, y and z direction
  | *Example*:

  .. code-block:: python

     gContact.SetSearchTreeInitSize([10,10,10])

* | **SetSearchTreeBox**\ (\ *pMin*\ , \ *pMax*\ ): 
  | set geometric dimensions of searchTreeBox (point with minimum coordinates and point with maximum coordinates); if this box becomes smaller than the effective contact objects, contact computations may slow down significantly
  | *Example*:

  .. code-block:: python

     gContact.SetSearchTreeBox(pMin=[-1,-1,-1],
         pMax=[1,1,1])

* | **AddSphereWithMarker**\ (\ *markerIndex*\ , \ *radius*\ , \ *contactStiffness*\ , \ *contactDamping*\ , \ *frictionMaterialIndex*\ ): 
  | add contact object using a marker (Position or Rigid), radius and contact/friction parameters and return localIndex of the contact item in GeneralContact; frictionMaterialIndex refers to frictionPairings in GeneralContact; contact is possible between spheres (circles in 2D) (if intraSphereContact = True), spheres and triangles and between sphere (=circle) and ANCFCable2D; contactStiffness is computed as serial spring between contacting objects, while damping is computed as a parallel damper
* | **AddANCFCable**\ (\ *objectIndex*\ , \ *halfHeight*\ , \ *contactStiffness*\ , \ *contactDamping*\ , \ *frictionMaterialIndex*\ ): 
  | add contact object for an ANCF cable element, using the objectIndex of the cable element and the cable's half height as an additional distance to contacting objects (currently not causing additional torque in case of friction), and return localIndex of the contact item in GeneralContact; currently only contact with spheres (circles in 2D) possible; contact computed using exact geometry of elements, finding max 3 intersecting contact regions
* | **AddTrianglesRigidBodyBased**\ (\ *rigidBodyMarkerIndex*\ , \ *contactStiffness*\ , \ *contactDamping*\ , \ *frictionMaterialIndex*\ , \ *pointList*\ , \ *triangleList*\ , \ *staticTriangles*\  = False): 
  | add contact object using a rigidBodyMarker (of a body), contact/friction parameters, a list of points (as 3D numpy arrays or lists; coordinates relative to rigidBodyMarker) and a list of triangles (3 indices as numpy array or list) according to a mesh attached to the rigidBodyMarker; the flag staticTriangles=True can be used to inform the contact solver that these triangles are static (fixed in space); note that static triangles have to be added before dynamic triangles; function returns starting local index of trigsRigidBodyBased at which the triangles are stored; mesh can be produced with GraphicsData2TrigsAndPoints(...); contact is possible between sphere (circle) and Triangle but yet not between triangle and triangle; frictionMaterialIndex refers to frictionPairings in GeneralContact; contactStiffness is computed as serial spring between contacting objects, while damping is computed as a parallel damper (otherwise the smaller damper would always dominate); the triangle normal must point outwards, with the normal of a triangle given with local points (p0,p1,p2) defined as n=(p1-p0) x (p2-p0), see function ComputeTriangleNormal(...)
* | **GetItemsInBox**\ (\ *pMin*\ , \ *pMax*\ ): 
  | Get all items in box defined by minimum coordinates given in pMin and maximum coordinates given by pMax, accepting 3D lists or numpy arrays; in case that no objects are found, False is returned; otherwise, a dictionary is returned, containing numpy arrays with indices of obtained MarkerBasedSpheres, TrigsRigidBodyBased, ANCFCable2D, ...; the indices refer to the local index in GeneralContact which can be evaluated e.g. by GetMarkerBasedSphere(localIndex)
  | *Example*:

  .. code-block:: python

     gContact.GetItemsInBox(pMin=[0,1,1],
         pMax=[2,3,2])

* | **GetSphereMarkerBased**\ (\ *localIndex*\ , \ *addData*\  = False): 
  | Get dictionary with current position, orientation, velocity, angular velocity as computed in last contact iteration; if addData=True, adds stored data of contact element, such as radius, markerIndex and contact parameters; localIndex is the internal index of contact element, as returned e.g. from GetItemsInBox
* | **SetSphereMarkerBased**\ (\ *localIndex*\ , \ *contactStiffness*\  = -1., \ *contactDamping*\  = -1., \ *radius*\  = -1., \ *frictionMaterialIndex*\  = -1): 
  | Set data of marker based sphere with localIndex (as internally stored) with given arguments; arguments that are < 0 (default) imply that current values are not overwritten
* | **GetTriangleRigidBodyBased**\ (\ *localIndex*\ ): 
  | Get dictionary with rigid body index, local position of triangle vertices (nodes) and triangle normal; NOTE: the mesh added to contact is different from this structure, as it contains nodes and connectivity lists; the triangle index corresponds to the order as triangles are added to GeneralContact
* | **SetTriangleRigidBodyBased**\ (\ *localIndex*\ , \ *points*\ , \ *contactRigidBodyIndex*\  = -1): 
  | Set data of marker based sphere with localIndex (triangle index); points are provided as 3x3 numpy array, with point coordinates in rows; contactRigidBodyIndex<0 indicates no change of the current index (and changing this index should be handled with care)
* | **ShortestDistanceAlongLine**\ (\ *pStart*\  = [0,0,0], \ *direction*\  = [1,0,0], \ *minDistance*\  = -1e-7, \ *maxDistance*\  = 1e7, \ *asDictionary*\  = False, \ *cylinderRadius*\  = 0, \ *typeIndex*\  = Contact.IndexEndOfEnumList): 
  | Find shortest distance to contact objects in GeneralContact along line with pStart (given as 3D list or numpy array) and direction (as 3D list or numpy array with no need to be normalized); the function returns the distance which is >= minDistance and < maxDistance; in case of beam elements, it measures the distance to the beam centerline; the distance is measured from pStart along given direction and can also be negative; if no item is found along line, the maxDistance is returned; if asDictionary=False, the result is a float, while otherwise details are returned as dictionary (including distance, velocityAlongLine (which is the object velocity in given direction and may be different from the time derivative of the distance; works similar to a laser Doppler vibrometer - LDV), itemIndex and itemType in GeneralContact); the cylinderRadius, if not equal to 0, will be used for spheres to find closest sphere along cylinder with given point and direction; the typeIndex can be set to a specific contact type, e.g., which are searched for (otherwise all objects are considered)
* | **UpdateContacts**\ (\ *mainSystem*\ ): 
  | Update contact sets, e.g. if no contact is simulated (isActive=False) but user functions need up-to-date contact states for GetItemsInBox(...) or for GetActiveContacts(...)
  | *Example*:

  .. code-block:: python

     gContact.UpdateContacts(mbs)

* | **GetActiveContacts**\ (\ *typeIndex*\ , \ *itemIndex*\ ): 
  | Get list of global item numbers which are in contact with itemIndex of type typeIndex in case that the global itemIndex is smaller than the abs value of the contact pair index; a negative sign indicates that the contacting (spheres) is in Coloumb friction, a positive sign indicates a regularized friction region; in case of itemIndex==-1, it will return the list of numbers of active contacts per item for the contact type; for interpretation of global contact indices, see gContact.GetPythonObject() and documentation; requires either implicit contact computation or UpdateContacts(...) needs to be called prior to this function
  | *Example*:

  .. code-block:: python

     #if explicit solver is used, we first need to update contacts:
     gContact.UpdateContacts(mbs)
     #obtain active contacts of marker based sphere 42:
     gList = gContact.GetActiveContacts(exu.ContactTypeIndex.IndexSpheresMarkerBased, 42)

* | **GetSystemODE2RhsContactForces**\ (\ *copy*\  = False): 
  | Get numpy array of system vector containing contribution of contact forces to system ODE2 Rhs vector; if copy=False, it will give direct (reference) access to the internal vector (note: modifications to this vector do not influence simulation!), however, which may cause problems if the system size changes or simulation is restarted; if copy=True, the vector is copied (time consuming); contributions to single objects may be extracted by checking the according LTG-array of according objects (such as rigid bodies); the contact forces vector is computed in each contact iteration;
* | **\_\_repr\_\_()**\ : 
  | return the string representation of the GeneralContact, containing basic information and statistics




.. _sec-generalcontact-visualization:


VisuGeneralContact
==================

This structure may contains some visualization parameters in future. Currently, all visualization settings are controlled via SC.visualizationSettings

\ The class **VisuGeneralContact** has the following **functions and structures**:

* | **Reset**\ (): 
  | reset visualization parameters to default values



