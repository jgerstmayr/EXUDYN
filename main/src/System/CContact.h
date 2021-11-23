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

#include "Linalg/BasicLinalg.h"		//includes basic classes, all basic arrays and vectors
#include "Linalg/SearchTree.h"
#include "Main/TemporaryComputationData.h"

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
};

//! structure to store global contact state (only stored for contact object with smaller index
class ActiveContact
{
public:
	Index globalIndex;	//!< global index of contact object
	bool isStick;		//!< friction: true, if sticking, false if slipping 
	
	ActiveContact(Index globalIndexInit, bool isStickInit)
	{
		globalIndexInit = globalIndexInit;
		isStick = isStickInit;
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
//! temporary data for contact computation
class ContactSphereData //also used for circles! (as markerdata is anyway 3D!)
{
public:
	Vector3D position;
	Matrix3D orientation;
	Vector3D velocity;
	Vector3D angularVelocity;
};

class ContactANCFCable2DData
{
public:
	static Index constexpr ContactANCFCable2DmaxSize = 9;
	ConstSizeVector<ContactANCFCable2DmaxSize> coordinates;
	ConstSizeVector<ContactANCFCable2DmaxSize> coordinates_t;
};


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! data structures for contact definition
class ContactSpheresMarkerBased: public ContactSphereData
{
public:
	Index markerIndex;		//index to Marker in MainSystem
	Real contactStiffness;	//normal contact stiffness
	Real contactDamping;	//normal contact damping
	Real radius;
	Index frictionMaterialIndex; // 0 is default
};

class ContactANCFCable2D: ContactANCFCable2DData
{
public:
	Index objectIndex;		//index to ANCFCable2D object in MainSystem
	//for efficiency reasons: ANCFCable2D do not allow contact pairing with each other, thus contactStiffness must be included in circles (reduced computational costs 1/(1/c1+1/c2))
	//Real contactStiffness;	//normal contact stiffness
	//Real contactDamping;	//normal contact damping
	Real halfHeight;		//distance of center line of beam to contact surface (friction behaves slightly different as compared to increasing the circle by halfHeight)
	Index frictionMaterialIndex; // 0 is default
};

//class ContactSphereMarkerBased: public ContactSphereData
//{
//public:
//	Index markerIndex;
//	Real contactStiffness;
//	Real contactDamping;	//normal contact damping
//	Real radius;
//	Index frictionMaterialIndex; // 0 is default
//};

//Define empty argument
#define NOARG 

//! data structure for visualization of GeneralContact
class VisuGeneralContact
{
public:
	//put here drawing options for search tree or bounding boxes
	//not used for now, because drawing done by objects!
	//bool spheresMarkerBasedDraw;		//!< draw contact spheres
	//Index spheresMarkerBasedResolution; //!< resolution for drawing of spheres; use values between 3 (low resolution) and 100 (very high resolution, not recommended for > 5000 spheres)
	//Float4 spheresMarkerBasedColor;		//!< 

	VisuGeneralContact() { Reset(); }

	void Reset() //not called by PyGeneralContact!!!
	{
		//spheresMarkerBasedDraw = false;
		//spheresMarkerBasedResolution = 4;
		//spheresMarkerBasedColor = Float4({ 0.6f,0.6f,0.6f,1.f });
	}

};


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! class for general contact computations; mixing many different contact objects (but not all combinations of contact objects are considered!)
class GeneralContact
{
public: //make public in order to directly access from Python
	bool isActive;													//!< if false, no contact computation is performed
	Index verboseMode;												//!< if >0, it outputs a couple of information on contact creation and computation
	VisuGeneralContact visualization; //data structure for visualization
protected:

//#define CallOnAllContactsFunction(functionName, memberFunctionCall)\
//	void functionName() {\
//		spheresMarkerBased.memberFunctionCall();\
//		ancfCable2D.memberFunctionCall();\
//		spheresMarkerBased.memberFunctionCall();\
//	}
	//these indices define the order of contact object lists in the globalIndex
	//!!THIS ORDER NEEDS TO BE OBEYED THROUGHOUT!!
	static const Index spheresMarkerBasedIndex = 0;
	static const Index ancfCable2DIndex = 1;
	static const Index linesMarkerBasedIndex = 2;
	
//this is not nice but helps to avoid copy-paste error for new contacts:
#define CallOnAllContacts(memberFunctionCall, memberFunctionArg, externCall)\
		externCall(spheresMarkerBased.memberFunctionCall(memberFunctionArg));\
		externCall(ancfCable2D.memberFunctionCall(memberFunctionArg));\

//this is not nice but helps to avoid copy-paste error for new contacts:
#define EvaluateOnAllContacts(memberFunctionCall, memberFunctionArg, variableName, variableFunction)\
		variableName.variableFunction(spheresMarkerBased.memberFunctionCall(memberFunctionArg));\
		variableName.variableFunction(ancfCable2D.memberFunctionCall(memberFunctionArg));\

	//++++++++++++++++++++++++++++++++++++++++++++++++
	//define all lists for contact search, with unique global index
	//the following lists represent the ORDER at which they are mapped to a gloal contact index!!!
	ResizableArray<ContactSpheresMarkerBased> spheresMarkerBased;	//!< these are spheres (circles) with center point [and orientation] defined by Marker; this allows to attach contact to mass points, rigid bodies and deformable bodies
	ResizableArray<ContactANCFCable2D> ancfCable2D;					//!< these are ANCFCable2D splines [undergoing contact with spheres (circles)]


	bool intraSpheresContact;										//!< if false, contact between spheres is deactivated

	//++++++++++++++++++++++++++++++++++++++++++++++++
	//! initialized when contact definition finished:
	ArrayIndex globalContactIndexOffsets;							//!< offsets corresponding to contact lists above
	Matrix frictionPairings;										//!< contains pairing coefficients between two materials; if rows=columns=1, it only uses one pairing for all materials
	Index maxFrictionMaterialIndex;									//!< set during initialization to check whether all indices are valid
	ResizableArray<ArrayIndex*> allLTGs;							//!< LTG mapping for every contact object (nullptr if unused)

	//++++++++++++++++++++++++++++++++++++++++++++++++
	//! updated at every contact computation:
	//! bounding boxes, created prior to contact search:
	SearchTree searchTree;											//!< search tree containing all contact objects
	ResizableArray<Box3D> allBoundingBoxes;							//!< contains all bounding boxes, with global contact object index
	ResizableArray<ArrayIndex*> allActiveContacts;					//!< according to global contact objects: contains other global index, if contact is active

	//storage for precomputed jacobians of some contacts
	ResizableArray<ResizableMatrix*> allPositionJacobians;			//!< d(pos)/dq
	ResizableArray<ResizableMatrix*> allRotationJacobians;			//!< d(omega)/dq_t


	//TemporaryComputationDataArray tempArray; //will be moved to CSystem or Solver
	//! the following lists need to be stored per thread:
	ResizableArray<ArrayIndex*> addedObjects;					//!< array used when retreiving contact objects, storing which objects already have been added
	ResizableArray<ResizableArray<bool>*> addedObjectsFlags;		//!< stores true/false if object has been added; this allows fast detection if objects have been added
	//

public:
	GeneralContact()
	{
		Reset(false); //in fact, only necessary to initialize some variables; arrays are anyway initialized
	}

	~GeneralContact()
	{
		Reset(true);
	}

	//! reset all contact arrays, free memory if requested
	void Reset(bool freeMemory = true) 
	{
		visualization.Reset();
		intraSpheresContact = true;
		isActive = true;
		verboseMode = 0;

		if (freeMemory)
		{
			searchTree.Flush();
			allBoundingBoxes.Flush();
			globalContactIndexOffsets.Flush();

			CallOnAllContacts(Flush, NOARG, NOARG);
			for (Index i = 0; i < allActiveContacts.NumberOfItems(); i++)
			{
				if (allActiveContacts[i] != nullptr)
				{
					delete allActiveContacts[i];
				}
			}
			allActiveContacts.Flush();

			for (Index i = 0; i < allPositionJacobians.NumberOfItems(); i++)
			{
				if (allPositionJacobians[i] != nullptr)
				{
					delete allPositionJacobians[i];
				}
			}
			allPositionJacobians.Flush();

			for (Index i = 0; i < allRotationJacobians.NumberOfItems(); i++)
			{
				if (allPositionJacobians[i] != nullptr)
				{
					delete allRotationJacobians[i];
				}
			}
			allRotationJacobians.Flush();

			for (Index i = 0; i < allLTGs.NumberOfItems(); i++)
			{
				if (allLTGs[i] != nullptr)
				{
					delete allLTGs[i];
				}
			}
			allLTGs.Flush();

			SetNumberOfThreads(0); //erases all thread-related arrays
		}
		else
		{
			allActiveContacts.SetNumberOfItems(0);
			allBoundingBoxes.SetNumberOfItems(0);
			searchTree.ClearItems();
			globalContactIndexOffsets.SetNumberOfItems(0);

			CallOnAllContacts(SetNumberOfItems,0, NOARG);
		}
	}

	//! helper function that is called for every contact set
	template<typename T>
	void ComputeMaximumFrictionIndex(const ResizableArray<T>& contactObjects)
	{
		for (const T& item : contactObjects)
		{
			maxFrictionMaterialIndex = EXUstd::Maximum(maxFrictionMaterialIndex, item.frictionMaterialIndex);
		}
	}

	//! perform operations in case that number of threads have been changed or initialize arrays
	void SetNumberOfThreads(Index nThreads);

	//! get total number of contact objects; only available after FinalizeContact!
	Index TotalContactObjects() const { return globalContactIndexOffsets.Last(); }

	const ResizableArray<ContactSpheresMarkerBased>& GetSpheresMarkerBased() { return spheresMarkerBased; }
	const ResizableArray<ContactANCFCable2D>& GetANCFCable2D() { return ancfCable2D; }
	

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//FUNCTIONS FOR CONTACT INITIALIZATION:

	//! add contact object using a marker (Position or Rigid), radius and contact/friction parameters; 
	//contact is possible between spheres (circles) (if intraSphereContact = true) and between sphere (circle) and ANCFCable2D
	void AddSphereWithMarker(Index markerIndex, Real radius, Real contactStiffness, Real contactDamping, Index frictionMaterialIndex);

	//! set up necessary parameters for contact: friction, SearchTree, etc.; done after all contacts have been added
	//! at this point, can also be checked if something is wrong (illegal pairings or frictionMaterial coeffs, etc.)
	//! empty box will autocompute size!
	void FinalizeContact(const CSystem& cSystem, Index3 searchTreeSize, const Matrix& frictionPairingsInit,
		Vector3D searchTreeBoxMin = Vector3D(EXUstd::MAXREAL), Vector3D searchTreeBoxMax = Vector3D(EXUstd::LOWESTREAL));

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//FUNCTIONS FOR CONTACT COMPUTATION:

	//! compute temporary data and bounding boxes; addToSearchTree is used to only compute bounding boxes, but not adding to searchtree
	template<bool updateBoundingBoxes>
	void ComputeContactDataAndBoundingBoxes(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, bool addToSearchTree = true);

	//! update search tree with previously computed bounding boxes
	void UpdateSearchTree()
	{
		searchTree.ClearItems();
		Index globalIndex = 0; //
		for (const Box3D& box : allBoundingBoxes)
		{
			if (!box.Empty()) { searchTree.AddItem(box, globalIndex); } //check needed?
			globalIndex++;
		}
	}

	enum ComputeContactMode { CCactiveSets = 1 << 0, CCode2rhsFull = 1 << 1, CCode2rhsFromActiveSets = 1 << 2 };
	//! compute 1) active sets (PostNewtonStep), 2) ODE2RHS (fully) or 3) ODE2RHS from active sets ?or 4) jacobian
	template<Index opMode>
	void ComputeContact(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, Vector& systemODE2Rhs);//, Real& maxError);//! in case of active sets, maxError returns maximum penetration

	//! compute contact forces and add to global system vector
	void ComputeODE2RHS(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, Vector& systemODE2Rhs);

	//! compute jacobian of ODE2RHS w.r.t. ODE2 and ODE2_t quantities; 
	//! multiply (before added to jacobianGM) ODE2 with factorODE2 and ODE2_t with factorODE2_t
	//! only sparse option available
	void JacobianODE2RHS(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, const NumericalDifferentiationSettings& numDiff,
		GeneralMatrix& jacobianGM, Real factorODE2, Real factorODE2_t);

	//! PostNewtonStep: update active sets for contact
	//! recommended step size \f$h_{recom}\f$ after PostNewton(...): \f$h_{recom} < 0\f$: no recommendation, \f$h_{recom}==0\f$: use minimum step size, \f$h_{recom}>0\f$: use specific step size, if no smaller size requested by other reason
	Real PostNewtonStep(const CSystem& cSystem, TemporaryComputationDataArray& tempArray, Real& recommendedStepSize);

};
#endif //USE_GENERAL_CONTACT

#endif
