/** ***********************************************************************************************
* @class	    CContact
* @brief		Class for general contact computation
* @details		Details:
*
* @author		Gerstmayr Johannes
* @date			2021-10-23 (generated)
* @pre			...
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
************************************************************************************************ */
#ifndef CCONTACT__H
#define CCONTACT__H


#define USE_GENERAL_CONTACT
#ifdef USE_GENERAL_CONTACT

#define USE_STATICDYNAMIC_SEARCHTREE //for higher performance

#include "Linalg/BasicLinalg.h"		//includes basic classes, all basic arrays and vectors
#include "Utilities/AdvancedStuff.h"
#include "Linalg/SearchTree.h"
#include "Main/TemporaryComputationData.h"

#include "Objects/CObjectANCFCable2DBase.h"

//use define because otherwise not correctly applied to templates in linux
#define DANCFmaxCoordinates 9
#define DANCFshapeCoordinates 8
#define DANCFcirclePolynomialDegree 6
#define DANCFselectedSegmentsLength 16 //min 3 for exact method

#ifdef USE_STATICDYNAMIC_SEARCHTREE 
typedef StaticDynamicSearchTree ContactSearchTree;
#else
typedef SearchTree ContactSearchTree;
#endif


class CSystem;
//class TemporaryComputationData;
class MarkerData;


//type definitions for iteration
namespace Contact {
	enum Type {
		_None = 0, //marks that no type is used
		//type of attachement:
		MarkerBased = 1 << 0,
		NodeBased = 1 << 1,
		ObjectBased = 1 << 2,
		RigidBodyAttached = 1 << 3, //attached to rigid body (using marker)
		//type of contact element:
		Sphere = 1 << 4,			//sphere (circle) attached e.g. to Marker
		ANCFCable2D = 1 << 5,		//very special contact, only accepting ANCFCable2D elements (using cubic spline)
		Line2D = 1 << 6,			//line (x/y line for 2D contact, represent planes with z in [-infty,+infty]
		Triangle = 1 << 7
	};

	//get type string for some debugging and Python functionality
	inline STDstring GetTypeString(Type var)
	{
		STDstring t; //empty string
		if (var == Contact::_None) { t = "_None/Undefined"; }
		if (var & MarkerBased) { t += "MarkerBased"; }
		if (var & NodeBased) { t += "NodeBased"; }
		if (var & ObjectBased) { t += "ObjectBased"; }
		if (var & RigidBodyAttached) { t += "RigidBodyAttached"; }

		if (var & Sphere) { t += "Sphere"; }
		if (var & ANCFCable2D) { t += "ANCFCable2D"; }
		if (var & Line2D) { t += "Line2D"; }
		if (var & Triangle) { t += "Triangle"; }

		if (t.length() == 0) { CHECKandTHROWstring("Contact::GetTypeString(...) called for invalid type!"); }

		return t;
	}

	//this type maps to some arrays:
	enum TypeIndex {
		IndexSpheresMarkerBased = 0,
		IndexANCFCable2D = 1,
		IndexTrigsRigidBodyBased = 2,
		IndexEndOfEnumList = 3
	};

	inline STDstring GetTypeIndexString(TypeIndex var)
	{
		STDstring t; //empty string
		if (var == IndexSpheresMarkerBased) { t = "SpheresMarkerBased"; }
		else if (var == IndexANCFCable2D) { t = "ANCFCable2D"; }
		else if (var == IndexTrigsRigidBodyBased) { t = "TrigsRigidBodyBased"; }
		else { CHECKandTHROWstring("Contact::GetTypeIndexString(...) called for invalid type!"); }

		return t;
	}

};

//short-term contacts:
//spheresMarkerBased (does this differ from spheres?? planar case?)
//ANCFCable2D (does this differ from spheres?? planar case?)

//SpheresMarkerBased
//LinesMarkerBased (x/y lines for 2D contact, represent planes with z in [-infty,+infty]
//SpheresNodeBased
//SpheresRigidBodyAttached
//TrianglesRigidBodyAttached

//contact pairs:
//SpheresMarkerBased <-> SpheresMarkerBased
//SpheresMarkerBased <-> LinesMarkerBased
//SpheresMarkerBased <-> ANCFCable2D
//TrianglesRigidBodyAttached <-> SpheresMarkerBased
//
//SpheresNodeBased <-> SpheresNodeBased
//SpheresNodeBased <-> SpheresMarkerBased
//SpheresRigidBodyAttached <-> TrianglesRigidBodyAttached
//TrianglesRigidBodyAttached <-> SpheresNodeBased

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//here we have some base classes (without virtual functions!) for rigid body data, etc.
//! temporary data for contact computation
class ContactRigidBodyData //also used for circles! (as markerdata is anyway 3D!)
{
public:
	Vector3D position;			//!< global position of reference point
	Matrix3D orientation;		//!< transforms local to global coordinates
	Vector3D velocity;			//!< global velocity
	Vector3D angularVelocity;   //!< local (body-fixed) angular velocity
};

class ContactANCFCable2DData
{
public:
	static Index constexpr ANCFmaxCoordinates = 9;
	static Index constexpr ANCFshapeCoordinates = 8;		//!< coordinates used to compute shape of ANCF elements
	static Index constexpr ANCFcirclePolynomialDegree = 6;  //!< degree of polynomial in contact with circle
	Real L; //reference length of ANCF element
	bool isALE;
	ConstSizeVector<ANCFmaxCoordinates> coordinates;
	ConstSizeVector<ANCFmaxCoordinates> coordinates_t;
};


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! data structures for contact definition
class ContactSpheresMarkerBased : public ContactRigidBodyData
{
public:
	Index markerIndex;		//index to Marker in MainSystem
	Real contactStiffness;	//normal contact stiffness
	Real contactDamping;	//normal contact damping
	Real radius;
	Index frictionMaterialIndex; // 0 is default
};

//! contact with ANCFCable2D and ALEANCFCable2D
class ContactANCFCable2D : public ContactANCFCable2DData
{
public:
	Index objectIndex;		//index to ANCFCable2D object in MainSystem
	Real contactStiffness;	//normal contact stiffness
	Real contactDamping;	//normal contact damping
	Real halfHeight;		//distance of center line of beam to contact surface (friction behaves slightly different as compared to increasing the circle by halfHeight)
	Index frictionMaterialIndex; // 0 is default
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! rigid body is marker based, used for trianglemesh-on-rigid-body and spheres-on-rigid-body
class ContactRigidBodyMarkerBased : public ContactRigidBodyData
{
public:
	Index markerIndex;		//index to Marker in MainSystem
	Real contactStiffness;	//normal contact stiffness
	Real contactDamping;	//normal contact damping
	Index frictionMaterialIndex; // 0 is default
};

//! this triangle is fixed to a rigid body, given by a marker
class ContactTriangleRigidBodyBased
{
public:
	Index contactRigidBodyIndex;	//!< index of underlying rigidBodyMarkerBased in GeneralContact
	std::array<Vector3D, 3> points; //!< relative position on rigid body; points directly stored here, because computed locally (memory access!)
	Vector3D normal;				//!< local normal of triangle, computed from points during adding of triangle
};

//! temporary data at classification of triangle contact
class ContactTrianglesRigidBodyBasedTemp
{
public:
	Vector3D pointInPlane;
	Vector3D planeNormal;	//!< stored plane normal; ONLY computed if inside=true
	Index rigidID;			//!< local rigid ID
	Index trigGID;			//!< global (contact) ID
	Index inside;			//!< 1 = inside, 0 = outside, 2 = on edge (which may be on another edge as well)
};

class VisualizationSettings;
class GeneralContact;

//! data structure for visualization of GeneralContact
class VisuGeneralContact
{
public:
	//put here drawing options for search tree or bounding boxes
	//not used for now, because options in visualizationSettings

	VisuGeneralContact() { Reset(); }

	void Reset() //not called by PyGeneralContact!!!
	{
	}

	//! visualization function to draw some contact information (bounding box, search tree, active contacts etc.)
	//! only possible to draw during computation!
	void DrawContacts(const GeneralContact& gContact, const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem);
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class GeneralContactSettings
{
public:
	Index3 searchTreeSizeInit;						//!< initialization for search tree sizes
	Vector3D searchTreeBoxMinInit;					//!< initialization for searchTree box
	Vector3D searchTreeBoxMaxInit;					//!< initialization for searchTree box
	Index resetSearchTreeInterval;					//!< number of iterations after which the search tree is re-created

	bool sphereSphereContact;						//!< if false, contact between spheres is deactivated
	bool sphereSphereFrictionRecycle;				//!< if true, static friction force is recycled from previous PostNewton step, which greatly improves convergence but may behave unphysically
	Real frictionProportionalZone;					//!< regularization for friction (m/s); global for all contacts
	Real minRelDistanceSpheresTriangles;			//!< minimum relative distance between spheres and triangles, below that there is no contact computation
	bool excludeOverlappingTrigSphereContacts;		//!< for consistent, closed meshes, we can exclude duplicate contacts
	bool excludeDuplicatedTrigSphereContactPoints;  //!< run additional checks for double contacts at edges or vertices, being more accurate but can cause additional costs if many contacts
	bool computeExactStaticTriangleBins;			//!< if True, search tree bins are computed exactly for static triangles while if False, it uses the overall (=very inaccurate) AABB of each triangle in the search tree

	bool computeContactForces;						//!< if true, contribution of contact forces to system vector is evaluated

	Real tolEquivalentPoints;						//!< tolerance distance of projected points that are considered to be equivalent; 
	Real tolEquivalentPointsSquared;				//!< squared tolerance distance of projected points, considered to be equivalent;

	bool ancfCableUseExactMethod;					//!< if true, uses exact computation of intersection of 3rd order polynomials and contacting circles
	Index ancfCableNumberOfContactSegments;			//!< if ancfCableUseExactMethod=false, then this specifies the number of contact segments for ANCF element
	Index ancfCableMeasuringSegments;			    //!< number of segments used to approximate geometry for ANCFCable2D elements for measuring distance
	//++++++++++++++++++++++++++++++++
	//contact behaviour
	Matrix frictionPairings;						//!< contains pairing coefficients between two materials; if rows=columns=1, it only uses one pairing for all materials

	Index parallelTaskSplit;						//!< general number of tasks per thread (min)
	Index parallelTaskSplitBoundingBoxes;           //!< number of tasks per thread for bounding box computations
	Index parallelTaskSplitThreshold;               //!< general threshold below which only one task per thread is used
	Index parallelTaskSplitBoundingBoxesThreshold;  //!< threshold below which only one task per thread is used, for bounding box computations
public:
	GeneralContactSettings() { Reset(); }
	void Reset()
	{
		searchTreeSizeInit = Index3({ 10, 10, 10 }); //surely not optimal, but better than {1,1,1}
		searchTreeBoxMinInit = Vector3D(EXUstd::MAXREAL);
		searchTreeBoxMaxInit = Vector3D(EXUstd::LOWESTREAL);
		resetSearchTreeInterval = 10000;

		sphereSphereContact = true;
		sphereSphereFrictionRecycle = false;
		frictionProportionalZone = 0.001;

		minRelDistanceSpheresTriangles = 1e-10;
		excludeOverlappingTrigSphereContacts = true;
		excludeDuplicatedTrigSphereContactPoints = false; //cheaper method using reduction of stiffness preferred
		computeExactStaticTriangleBins = true;

		computeContactForces = false;

		tolEquivalentPoints = 1e-13;
		tolEquivalentPointsSquared = EXUstd::Square(tolEquivalentPoints);

		ancfCableUseExactMethod = true;
		ancfCableNumberOfContactSegments = 1;
		ancfCableMeasuringSegments = 20;

		frictionPairings = Matrix();

		//parallel
		parallelTaskSplit = 12;				//split min into 12*nThreads tasks; in case of threshold*32 => will use 144*nThreads
		parallelTaskSplitThreshold = 48;	//split above 48*nThreads tasks
		parallelTaskSplitBoundingBoxes = 12;//in case of threshold*8 => will use 96
		parallelTaskSplitBoundingBoxesThreshold = 400;
		//DELETE:
		// original for marker-based spheres:
		//Index taskSplit = nThreads; //shall be multiple of number of treads (Default=nThreads), but better 8*nThreads or larger for large problems
		//if ((Index)nItems >= 32 * nThreads)
		//{
		//	if ((Index)nItems < 128 * nThreads) { taskSplit = 16 * nThreads; }
		//	else { taskSplit = 128 * nThreads; }
		//}

	}
};


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! class for general contact computations; mixing many different contact objects (but not all combinations of contact objects are considered!)
class GeneralContact
{
public: //make public in order to directly access from Python
	bool isActive;						//!< if false, no contact computation is performed
	Index verboseMode;					//!< if >0, it outputs a couple of information on contact creation and computation
	bool contactIsFinalized;			//!< this flag is only true if contact is finalized and no further items added
	VisuGeneralContact visualization;	//!< data structure for visualization

	GeneralContactSettings settings; //!< settings for GeneralContact, easy to be modified via Python

protected:

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//these indices define the order of contact object lists in the globalIndex
	//**ICI individual contact implementation
	//!!THIS ORDER NEEDS TO BE OBEYED THROUGHOUT!!
	static const Index spheresMarkerBasedIndex = 0;
	static const Index ancfCable2DIndex = 1;
	static const Index trigsRigidBodyBasedIndex = 2;  //for bounding boxes and contact computation
	static const Index rigidBodyMarkerBasedIndex = 2; //for jacobians

	//define all lists for contact search, with unique global index
	//the following lists represent the ORDER at which they are mapped to a gloal contact index!!!
	//**ICI individual contact implementation
	ResizableArray<ContactSpheresMarkerBased> spheresMarkerBased;		//!< these are spheres (circles) with center point [and orientation] defined by Marker; this allows to attach contact to mass points, rigid bodies and deformable bodies
	ResizableArray<ContactANCFCable2D> ancfCable2D;						//!< these are ANCFCable2D splines [undergoing contact with spheres (circles)]
	ResizableArray<ContactTriangleRigidBodyBased> trigsRigidBodyBased;	//!< these are triangles attached to rigid body [undergoing contact with spheres (circles)]
	Index trigsRigidBodyBasedDynamicStartIndex;							//!< start index at which dynamic triangles start (others are not changing)
	bool staticContactObjectsInitialized;								//!< flag to determine if static objects have been initialized for search tree, etc.


	//this is only a base list, not directly evaluated for contacts
	ResizableArray<ContactRigidBodyMarkerBased> rigidBodyMarkerBased;	//!< these are rigid bodies, underlying the triangles

	//this is not nice but helps to avoid copy-paste error for new contacts:
	//**ICI individual contact implementation
	#define CallOnAllContacts(memberFunctionCall, memberFunctionArg, externCall)\
			externCall(spheresMarkerBased.memberFunctionCall(memberFunctionArg));\
			externCall(ancfCable2D.memberFunctionCall(memberFunctionArg));\
			externCall(trigsRigidBodyBased.memberFunctionCall(memberFunctionArg));\
			externCall(rigidBodyMarkerBased.memberFunctionCall(memberFunctionArg));\

	////this is not nice but helps to avoid copy-paste error for new contacts:
	////**ICI individual contact implementation
	//#define EvaluateOnAllContacts(memberFunctionCall, memberFunctionArg, variableName, variableFunction)\
	//		variableName.variableFunction(spheresMarkerBased.memberFunctionCall(memberFunctionArg));\
	//		variableName.variableFunction(ancfCable2D.memberFunctionCall(memberFunctionArg));\
	//		variableName.variableFunction(trigsRigidBodyBased.memberFunctionCall(memberFunctionArg));\

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



	//++++++++++++++++++++++++++++++++++++++++++++++++
	Index maxFrictionMaterialIndex;									//!< set during initialization to check whether all indices are valid
	bool initializeData;											//!< flag is set to initialize some MarkerData in first run after FinalizeContact
	//++++++++++++++++++++++++++++++++++++++++++++++++
	//! updated at every contact computation:
	//! bounding boxes, created prior to contact search:
	ContactSearchTree searchTree;									//!< search tree containing all contact objects

	Index searchTreeUpdateCounter;									//!< number of search tree updates; used to reset tree after specific number of updates

	ArrayIndex globalContactIndexOffsets;							//!< offsets corresponding to contact lists (allBoundingBoxes;allActiveContacts)
	//the followoing lists follow the globalContactIndices:
	ResizableArray<Box3D> allBoundingBoxes;							//!< contains all bounding boxes, with global contact object index
	ResizableArray<ArrayIndex*> allActiveContacts;					//!< contains other (negative) global index, if contact is active; sign: positive in linear zone, negative in saturated friction 
	ResizableArray<ResizableArray<Vector3D>*> allActiveContactsVector;	//!< syncronized with allActiveContacts; ONLY available in case of friction! additional vector data for active contacts; represents e.g. friction given by PostNewtonMethod, to avoid change of sign during Newton

	//the following lists follow the globalJacobianIndices:
	ArrayIndex globalJacobianIndexOffsets;							//!< offsets corresponding to jacobians (is different e.g. for rigidBodyMarkerBased and triangles)
	ResizableArray<ResizableMatrix*> allPositionJacobians;			//!< d(pos)/dq; storage for precomputed jacobians of contacts OR rigid bodies
	ResizableArray<ResizableMatrix*> allRotationJacobians;			//!< d(omega)/dq_t; storage for precomputed jacobians of contacts OR rigid bodies
	ResizableArray<ArrayIndex*> allLTGs;							//!< LTG mapping for every jacobian (nullptr if unused)

	//! the following lists need to be stored per thread:
	ResizableArray<ArrayIndex*> addedObjects;						//!< array used when retreiving contact objects, storing which objects already have been added
	ResizableArray<ResizableArray<bool>*> addedObjectsFlags;		//!< stores true/false if object has been added; this allows fast detection if objects have been added

	ResizableArray<ResizableArray<ContactTrianglesRigidBodyBasedTemp>*> foundTrianglesRigidBodyBased; //!< array used to classify found triangles
	ResizableArray<ArrayIndex*> foundPlanesTrianglesRigidBodyBased;			//!< indices of plane triangles; used to exclude edges coinciding with planes
	ResizableArray<ArrayIndex*> foundEdgesTrianglesRigidBodyBased;			//!< indices of edge triangles; used to exclude edges with same projected points
	//

    //used by functions called directly via Python interface
    TemporaryComputationDataArray externFunctionsTempArray;			//!< will be moved to CSystem or Solver

	ResizableVector systemODE2RhsContactForces;						//!< vector contains contribution of contact forces in system ODE2Rhs vector after contact computation

public:
	GeneralContact()
	{
		Reset(false); //in fact, only necessary to initialize some variables; arrays are anyway initialized
		settings.Reset();
		//pout << "searchTreeBox init=" << settings.searchTreeBoxMinInit << "-" << settings.searchTreeBoxMaxInit << "\n";
	}

	~GeneralContact()
	{
		Reset(true);
	}

	//! reset all contact arrays, free memory if requested
	void Reset(bool freeMemory = true);

	const ContactSearchTree& GetSearchTree() const { return searchTree; }
	const ResizableArray<Box3D>& GetAllBoundingBoxes() const { return allBoundingBoxes; }

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! convert global contact index / bounding box index into local item index (e.g. markerBasedSpheres) and type
	void GlobalToLocalItemAndTypeIndex(Index globalIndex, Index& localIndex, Contact::TypeIndex& itemType);

	//! helper function that is called for every contact set
	template<typename T>
	void ComputeMaximumFrictionIndex(const ResizableArray<T>& contactObjects)
	{
		for (const T& item : contactObjects)
		{
			maxFrictionMaterialIndex = EXUstd::Maximum(maxFrictionMaterialIndex, item.frictionMaterialIndex);
		}
	}

	inline void ActiveContact2IndexRegularizedFriction(Index activeContactIndex, Index& indexUnsigned, bool& frictionRegularizedRegion)
	{
		if (activeContactIndex >= 0) { indexUnsigned = activeContactIndex; frictionRegularizedRegion = true; return; }
		frictionRegularizedRegion = false;
		indexUnsigned = -activeContactIndex;
	}

	inline Index IndexRegularizedFriction2ActiveContact(Index index, bool frictionRegularizedRegion)
	{
		if (frictionRegularizedRegion) {return index;}
		else { return -index; }
	}

	//! perform operations in case that number of threads have been changed or initialize arrays
	void SetNumberOfThreads(Index nThreads);

	//! compute task split for bounding boxes
	inline Index TaskSplitBoundingBoxes(Index nThreads, Index nItems) const
	{
		Index taskSplit = nThreads; //shall be multiple of number of treads (Default=nThreads), but better 8*nThreads or larger for large problems
		if (nItems > settings.parallelTaskSplitBoundingBoxesThreshold * nThreads) { 
			taskSplit = settings.parallelTaskSplitBoundingBoxes * nThreads;
			if (nItems > settings.parallelTaskSplitBoundingBoxesThreshold * nThreads * 16) { taskSplit *= 8; }
		}
		return taskSplit;
	}

	//! compute task split for general computations
	inline Index TaskSplit(Index nThreads, Index nItems) const
	{
		Index taskSplit = nThreads; //shall be multiple of number of treads (Default=nThreads), but better 8*nThreads or larger for large problems
		if (nItems > settings.parallelTaskSplitThreshold * nThreads) {
			taskSplit = settings.parallelTaskSplit * nThreads;
			if (nItems > settings.parallelTaskSplitThreshold * nThreads * 32) { taskSplit *= 12; }
		}
		return taskSplit;
	}

	//! get total number of contact objects; only available after FinalizeContact!
	Index TotalContactObjects() const { return globalContactIndexOffsets.Last(); }

	//! get total number of jacobians; only available after FinalizeContact!
	Index TotalJacobians() const { return globalJacobianIndexOffsets.Last(); }

	//**ICI individual contact implementation
	const ResizableArray<ContactSpheresMarkerBased>& GetSpheresMarkerBased() const { return spheresMarkerBased; }
	const ResizableArray<ContactANCFCable2D>& GetANCFCable2D() const { return ancfCable2D; }
	const ResizableArray<ContactTriangleRigidBodyBased>& TrigsRigidBodyBased() const { return trigsRigidBodyBased; }
	const ResizableArray<ContactRigidBodyMarkerBased>& RigidBodyMarkerBased() const { return rigidBodyMarkerBased; }

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//FUNCTIONS FOR CONTACT INITIALIZATION:

	//! add contact object using a marker (Position or Rigid), radius and contact/friction parameters; return local index in spheresMarkerBased
	//contact is possible between spheres (circles) (if intraSphereContact = true) and between sphere (circle) and ANCFCable2D
	Index AddSphereWithMarker(Index markerIndex, Real radius, Real contactStiffness, Real contactDamping, Index frictionMaterialIndex);

	//! add contact object for ANCFCable element; currently only possible for ANCFCable2D elements; return local index in ancfCable2D
	//contact is possible between sphere (circle) and ANCFCable2D
	Index AddANCFCable(Index objectIndex, Real halfHeight, Real contactStiffness, Real contactDamping, Index frictionMaterialIndex);

	//! add contact object for Triangles attached to rigidBodyMarker; return starting index of trigsRigidBodyBased
	//! contact is possible between sphere (circle) and Triangle but yet not between triangle and triangle!
	//! staticTriangles can be used to put triangles only to a static searchtree which is not updated during computation
	Index AddTrianglesRigidBodyBased(Index rigidBodyMarkerIndex, Real contactStiffness, Real contactDamping, 
		Index frictionMaterialIndex, ResizableArray<Vector3D> pointList, ResizableArray<Index3> triangleList, bool staticTriangles = false);

	//! set up necessary parameters for contact: friction, SearchTree, etc.; automatically done in mbs.Assemble()
	//! at this point, it will also be checked if something is wrong (illegal pairings or frictionMaterial coeffs, etc.)
	//! empty searchtree box will autocompute size!
	void FinalizeContact(const CSystem& cSystem);

		//! set up necessary parameters for contact: friction, SearchTree, etc.; done after all contacts have been added
	//! at this point, can also be checked if something is wrong (illegal pairings or frictionMaterial coeffs, etc.)
	//! empty box will autocompute size!
	//void FinalizeContact(const CSystem& cSystem, Index3 searchTreeSize, const Matrix& frictionPairingsInit,
	//		Vector3D searchTreeBoxMin = Vector3D(EXUstd::MAXREAL), Vector3D searchTreeBoxMax = Vector3D(EXUstd::LOWESTREAL));

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//FUNCTIONS FOR CONTACT COMPUTATION:
	
	//! unified function to compute contact forces, in particular for unification to compute derivatives for jacobians
	template<Index opMode, typename TReal>
	TReal ComputeContactForces(TReal gap, const SlimVectorBase<TReal, 3>& deltaP0,
		TReal deltaVnormal, const SlimVectorBase<TReal, 3>& deltaVji,
		TReal kContact, TReal dContact, TReal dryFriction, bool frictionRegularizedRegion, const SlimVectorBase<TReal, 3>& storedFFriction,
		SlimVectorBase<TReal, 3>& fVec, SlimVectorBase<TReal, 3>& fFriction);

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//bounding boxes:
	//! compute temporary data and bounding boxes; addToSearchTree is used to only compute bounding boxes, but not adding to searchtree
	void ComputeContactDataAndBoundingBoxes(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, 
		bool updateBoundingBoxes, bool addToSearchTree = true);

	//**ICI individual contact implementation
	//! specific Data and bounding box computations:
	void ComputeDataAndBBmarkerBasedSpheres(const CSystemData& systemData, TemporaryComputationDataArray& tempArray,
		Index nThreads, bool updateBoundingBoxes);
	//! compute bounding boxes for ANCFCable2D (no data necessary)
	void ComputeDataAndBBancfCable2D(const CSystemData& systemData, TemporaryComputationDataArray& tempArray,
		Index nThreads, bool updateBoundingBoxes);
	//! compute Data for rigidBodyMarkerBased and bounding boxes for trigsRigidBodyBased
	void ComputeDataAndBBtrigsRigidBodyBased(const CSystemData& systemData, TemporaryComputationDataArray& tempArray,
		Index nThreads, bool updateBoundingBoxes);

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	enum ComputeContactMode { CCactiveSets = 1 << 0, CCode2rhsFull = 1 << 1, CCode2rhsFromActiveSets = 1 << 2 };
	//! compute 1) active sets (PostNewtonStep), 2) ODE2RHS (fully) or 3) ODE2RHS from active sets
	template<Index opMode>
	void ComputeContact(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, Vector& systemODE2Rhs);//, Real& maxError);//! in case of active sets, maxError returns maximum penetration

	//**ICI individual contact implementation
	//! specific contact computations:
	template<Index opMode>
	void ComputeContactMarkerBasedSpheres(TemporaryComputationDataArray& tempArray, Index nThreads);

	template<Index opMode>
	void ComputeContactANCFCable2D(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, Index nThreads);

	template<Index opMode>
	void ComputeContactTrigsRigidBodyBased(TemporaryComputationDataArray& tempArray, Index nThreads);

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! compute contact forces and add to global system vector
	void ComputeODE2RHS(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, Vector& systemODE2Rhs);

	//! compute LHS jacobian of ODE2RHS w.r.t. ODE2 and ODE2_t quantities; 
	//! multiply (before added to jacobianGM) ODE2 with factorODE2 and ODE2_t with factorODE2_t
	//! only sparse option available
	void JacobianODE2LHS(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, const NumericalDifferentiationSettings& numDiff,
		GeneralMatrix& jacobianGM, Real factorODE2, Real factorODE2_t);

	//! helper function to compute jacobian of contact forces for given segments on ANCF cable, using numerical integration
	void ComputeContactJacobianANCFcableCircleContact(Index gi, Index gj, TemporaryComputationData& tempData, 
		Real factorODE2, Real factorODE2_t, const CObject* cObject,
		const ConstSizeVector<DANCFmaxCoordinates>& q, const ConstSizeVector<DANCFmaxCoordinates>& q_t,
		Real L, Real halfHeight, const Vector2D& circlePos, const ContactSpheresMarkerBased& sphere,
		Vector2D& segment, Real stiffness, Real damping, Real dryFriction, bool frictionRegularizedRegion);


	//! PostNewtonStep: update active sets for contact
	//! recommended step size \f$h_{recom}\f$ after PostNewton(...): \f$h_{recom} < 0\f$: no recommendation, \f$h_{recom}==0\f$: use minimum step size, \f$h_{recom}>0\f$: use specific step size, if no smaller size requested by other reason
	Real PostNewtonStep(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, Real& recommendedStepSize);

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//access / output functions
		
	//! return all items in sub-arrays and total count
	Index GetItemsInBox(const Box3D& box,
		ArrayIndex& arrayMarkerBasedSpheres,
		ArrayIndex& arrayTrigsRigidBodyBased,
		ArrayIndex& arrayANCFCable2D);

	//! measure shortest distance to object along line with start point and direction; return false, if no item found inside given min/max distance
	//! only points accepted in range [minDistance, maxDistance] including min, but excluding maxDistance
	//! the cylinderRadius, if not equal to 0, will be used for spheres to find closest sphere along cylinder with given point and direction
	//! search will only be done for selectedTypeIndex, if it is not equal to Contact::IndexEndOfEnumList
	//! returns also velocity, which is the velocity of the object in direction, not necessarily the time derivative of the distance
	bool ShortestDistanceAlongLine(const Vector3D& pStart, const Vector3D& direction, Real minDistance, Real maxDistance,
		Index& foundLocalIndex, Contact::TypeIndex& foundTypeIndex, Real& foundDistance, Real& foundVelocityAlongLine,
		Real cylinderRadius=0, Contact::TypeIndex selectedTypeIndex = Contact::IndexEndOfEnumList);

    //! update contact interactions, e.g. for ShortestDistanceAlongLine or for getting items in box
    void UpdateContacts(const CSystem& cSystem);

    //! get contact interactions of itemIndex of type selectedTypeIndex, e.g. IndexSpheresMarkerBased with index 2
	//! returns list of contacts, with global indices OR in case of itemIndex == -1, it will return all items with active contacts of the contact type (0=first contact type item)
    void GetActiveContacts(Contact::TypeIndex selectedTypeIndex, Index itemIndex, ArrayIndex*& foundContacts);

};
#endif //USE_GENERAL_CONTACT

#endif
