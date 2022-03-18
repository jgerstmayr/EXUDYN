/** ***********************************************************************************************
* @class        CSystemData
* @brief        All data defining the structure of the system; this is the place where objects really is live (link to this data if necessary)
*
* @author       Gerstmayr Johannes
* @date         2018-05-18 (generated)
* @date         2019-04-25 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
				- email: johannes.gerstmayr@uibk.ac.at
				- weblink: https://github.com/jgerstmayr/EXUDYN
				
************************************************************************************************ */
#ifndef CSYSTEMDATA__H
#define CSYSTEMDATA__H

//#include <ostream>

#include "Linalg/BasicLinalg.h"		//includes Vector.h
#include "Utilities/ObjectContainer.h"	

#include "Main/OutputVariable.h" 
#include "Main/CData.h"		//includes ReleaseAssert.h and BasicDefinitions.h
#include "Main/MainSystemBase.h"	//no further dependencies


#include "System/CMaterial.h"			//includes ReleaseAssert.h 
#include "System/CObjectBody.h"			//includes OutputVariable.h and CObject.h
#include "System/CNode.h"				//includes ReleaseAssert.h, BasicDefinitions.h, ResizeableArray.h, LinkedDataVector.h
#include "System/CMarker.h"		//needs nodes and bodies
#include "System/CLoad.h"				//needs markers
#include "System/CSensor.h"				//needs sensors
#include "System/CObjectConnector.h"	//includes OutputVariable.h and CObject.h

class CSystemData //
{
protected: //
	CData cData;                                    //!< computational data for all configurations (current, initial, etc.); this data is available in CNode
	MainSystemBase* mainSystemBacklink;             //!< backlink to MainSystem, but do not use generally!
	ResizableArray<CObject*> cObjects;              //!< container for computational objects
	ResizableArray<CNode*> cNodes;                  //!< container for computational nodes
	ResizableArray<CMaterial*> cMaterials;          //!< container for computational materials
	ResizableArray<CMarker*> cMarkers;              //!< container for computational markers
	ResizableArray<CLoad*> cLoads;                  //!< container for computational loads
	ResizableArray<CSensor*> cSensors;               //!< container for computational sensors

	ObjectContainer<ArrayIndex> localToGlobalODE2;  //!< CObject local to global ODE2 (Second order ODEs) coordinate indices transformation
	ObjectContainer<ArrayIndex> localToGlobalODE1;  //!< CObject local to global ODE1 (first order ODEs) coordinate indices transformation
	ObjectContainer<ArrayIndex> localToGlobalAE;    //!< CObject local to global AE (algebraic variables) coordinate indices transformation
	ObjectContainer<ArrayIndex> localToGlobalData;  //!< CObject local to global Data coordinate indices transformation

	Index numberOfCoordinatesODE2;                  //!< global number of ODE2 coordinates (sum of all node ODE2 coordinates); must be synchronous to NumberOfItems in SystemState Vectors
	Index numberOfCoordinatesODE1;                  //!< global number of ODE1 coordinates (sum of all node ODE1 coordinates); must be synchronous to NumberOfItems in SystemState Vectors
	Index numberOfCoordinatesAE;                    //!< global number of AE coordinates (sum of all node AE coordinates); must be synchronous to NumberOfItems in SystemState Vectors
	Index numberOfCoordinatesData;                  //!< global number of Data variables/coordinates (sum of all node Data variables); must be synchronous to NumberOfItems in SystemState Vectors

public:
	//use lists that are directly accessible for now; performance?
	ResizableArray<Index> objectsBodyWithODE2Coords;//!< list of objects that are bodies with ODE2 coordinates (e.g., no ground objects)
	ResizableArray<Index> listComputeObjectODE2Lhs;		//!< list of objects that need to evaluate ComputeObjectODE2Lhs (ALL)
	ResizableArray<Index> listComputeObjectODE2LhsNoUF;	//!< list of objects that need to evaluate ComputeObjectODE2Lhs, but have no user function
	ResizableArray<Index> listComputeObjectODE2LhsUF;	//!< list of objects that need to evaluate ComputeObjectODE2Lhs with user functions
	ResizableArray<Index> listComputeObjectODE1Rhs;		//!< list of objects that need to evaluate ComputeObjectODE1Rhs
	ResizableArray<Index> listDiscontinuousIteration;	//!< list of objects that need discontinuous iteration (PostNewtonStep, PostDiscontinuousIteration)
	ResizableArray<Index> listOfLoads;					//!< list of loads without user functions (can be processes multithreaded)
	ResizableArray<Index> listOfLoadsUF;				//!< list of loads WITH user functions (must be processed serially)

	ResizableArray<Index> objectsBodyWithAE;			//!< list of objects that are bodies and have AE
	ResizableArray<Index> nodesODE2WithAE;					//!< list of nodes that have AE (Euler parameters)
	ResizableArray<Index> objectsConstraintWithAE;		//!< list of objects that are constraints and have AE
	ResizableArray<Index> objectsWithAlgebraicEquations;//!< list of objects that have algebraic equations (AE)
	ResizableArray<Index> listObjectProjectedReactionForcesODE2;//!< list of objects that produce projected reaction forces for constraints

public: //

	//bool isODE2RHSjacobianComputation;

	// access functions
	//! clone object; specifically for copying instances of derived class, for automatic memory management e.g. in ObjectContainer
	CSystemData* GetClone() const { return new CSystemData(*this); }

	//! Specific destructor do deallocate data (allocated in MainSystem/ObjectFactory)
	~CSystemData() { Reset(); }

	void Reset()
	{
		cData = CData();
		//mainSystemBacklink = nullptr; //should not be reset, as the back link is still needed!!!

		for (auto item : cLoads) { delete item; }
		for (auto item : cMarkers) { delete item; }
		for (auto item : cMaterials) { delete item; }
		for (auto item : cNodes) { delete item; }
		for (auto item : cObjects) { delete item; }
		for (auto item : cSensors) { delete item; }

		cLoads.Flush();
		cMarkers.Flush();
		cMaterials.Flush();
		cNodes.Flush();
		cObjects.Flush();
		cSensors.Flush();

		numberOfCoordinatesODE2 = 0;
		numberOfCoordinatesODE1 = 0;
		numberOfCoordinatesAE = 0;
		numberOfCoordinatesData = 0;

		localToGlobalODE2.Flush();
		localToGlobalODE1.Flush();
		localToGlobalAE.Flush();
		localToGlobalData.Flush();

		objectsBodyWithODE2Coords.Flush();
		listComputeObjectODE2Lhs.Flush();
		listComputeObjectODE2LhsUF.Flush();
		listComputeObjectODE1Rhs.Flush();
		listDiscontinuousIteration.Flush();

		objectsBodyWithAE.Flush();
		nodesODE2WithAE.Flush();
		objectsConstraintWithAE.Flush();
		objectsWithAlgebraicEquations.Flush();
		listObjectProjectedReactionForcesODE2.Flush();

	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! Write (Reference) access to:computational data for all configurations (current, initial, etc.); this data is available in CNode
	CData& GetCData() { return cData; }
	//! Read (Reference) access to:computational data for all configurations (current, initial, etc.); this data is available in CNode
	const CData& GetCData() const { return cData; }

	//! Write (Reference) access to: MainSystem
	MainSystemBase& GetMainSystemBacklink() { return *mainSystemBacklink; }
	void SetMainSystemBacklink(MainSystemBase* mainSystemBacklinkInit) { mainSystemBacklink = mainSystemBacklinkInit; }
	//! Read (Reference) access to:computational data for all configurations (current, initial, etc.); this data is available in CNode
	const MainSystemBase& GetMainSystemBacklink() const { return *mainSystemBacklink; }

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! Write (Reference) access to:container for computational objects
	ResizableArray<CObject*>& GetCObjects() { return cObjects; }
	//! Read (Reference) access to:container for computational objects
	const ResizableArray<CObject*>& GetCObjects() const { return cObjects; }

	//! Write (Reference) access to:container for computational objects
	CObjectBody& GetCObjectBody(Index objectIndex) { CHECKandTHROW(((Index)cObjects[objectIndex]->GetType() & (Index)CObjectType::Body) != 0,"SystemData::GetObjectBody(...): object is not a body"); return *(CObjectBody*)(cObjects[objectIndex]); }
	//! Read (Reference) access to:container for computational objects
	const CObjectBody& GetCObjectBody(Index objectIndex) const { CHECKandTHROW(((Index)cObjects[objectIndex]->GetType() & (Index)CObjectType::Body) != 0, "SystemData::GetObjectBody(...) const: object is not a body"); return *(CObjectBody*)(cObjects[objectIndex]); }

	//! Write (Reference) access to:container for computational nodes
	ResizableArray<CNode*>& GetCNodes() { return cNodes; }
	//! Read (Reference) access to:container for computational nodes
	const ResizableArray<CNode*>& GetCNodes() const { return cNodes; }
	//! Write (Reference) access to:container for computational node
	CNode& GetCNode(Index itemIndex) { return *(cNodes[itemIndex]); }
	//! Read (Reference) access to:container for computational node
	const CNode& GetCNode(Index itemIndex) const { return *(cNodes[itemIndex]); }

	//! Write (Reference) access to:container for computational materials
	ResizableArray<CMaterial*>& GetCMaterials() { return cMaterials; }
	//! Read (Reference) access to:container for computational materials
	const ResizableArray<CMaterial*>& GetCMaterials() const { return cMaterials; }

	//! Write (Reference) access to:container for computational markers
	ResizableArray<CMarker*>& GetCMarkers() { return cMarkers; }
	//! Read (Reference) access to:container for computational markers
	const ResizableArray<CMarker*>& GetCMarkers() const { return cMarkers; }
	//! Write (Reference) access to:container for computational marker
	CMarker& GetCMarker(Index itemIndex) { return *(cMarkers[itemIndex]); }
	//! Read (Reference) access to:container for computational marker
	const CMarker& GetCMarker(Index itemIndex) const { return *(cMarkers[itemIndex]); }

	//! Write (Reference) access to:container for computational loads
	ResizableArray<CLoad*>& GetCLoads() { return cLoads; }
	//! Read (Reference) access to:container for computational loads
	const ResizableArray<CLoad*>& GetCLoads() const { return cLoads; }

	//! Write (Reference) access to:container for computational sensors
	ResizableArray<CSensor*>& GetCSensors() { return cSensors; }
	//! Read (Reference) access to:container for computational sensors
	const ResizableArray<CSensor*>& GetCSensors() const { return cSensors; }

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! Write (Reference) access to:CObject local to global ODE2 (Second order ODEs) coordinate indices transformation
	ObjectContainer<ArrayIndex>& GetLocalToGlobalODE2() { return localToGlobalODE2; }
	//! Read (Reference) access to:CObject local to global ODE2 (Second order ODEs) coordinate indices transformation
	const ObjectContainer<ArrayIndex>& GetLocalToGlobalODE2() const { return localToGlobalODE2; }

	//! Write (Reference) access to:CObject local to global ODE1 (first order ODEs) coordinate indices transformation
	ObjectContainer<ArrayIndex>& GetLocalToGlobalODE1() { return localToGlobalODE1; }
	//! Read (Reference) access to:CObject local to global ODE1 (first order ODEs) coordinate indices transformation
	const ObjectContainer<ArrayIndex>& GetLocalToGlobalODE1() const { return localToGlobalODE1; }

	//! Write (Reference) access to:CObject local to global AE (algebraic variables) coordinate indices transformation
	ObjectContainer<ArrayIndex>& GetLocalToGlobalAE() { return localToGlobalAE; }
	//! Read (Reference) access to:CObject local to global AE (algebraic variables) coordinate indices transformation
	const ObjectContainer<ArrayIndex>& GetLocalToGlobalAE() const { return localToGlobalAE; }

	//! Write (Reference) access to:CObject local to global Data variable indices transformation
	ObjectContainer<ArrayIndex>& GetLocalToGlobalData() { return localToGlobalData; }
	//! Read (Reference) access to:CObject local to global Data variable indices transformation
	const ObjectContainer<ArrayIndex>& GetLocalToGlobalData() const { return localToGlobalData; }

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//no access functions during development; test preformance
	////! Write (Reference) access 
	//ResizableArray<Index>& GetListComputeObjectODE2Lhs() { return listComputeObjectODE2Lhs; }
	////! Read (Reference) access
	//const ResizableArray<Index>& GetListComputeObjectODE2Lhs() const { return listComputeObjectODE2Lhs; }
	


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! Number of coordinates which determines the number of unknowns in system of equations and the matrix sizes
	void GetNumberOfComputationCoordinates(Index& nODE2, Index& nODE1, Index& nAE, Index& nData) const 
	{ nODE2 = numberOfCoordinatesODE2; nODE1 = numberOfCoordinatesODE1; nAE = numberOfCoordinatesAE; nData = numberOfCoordinatesData;	}

	//! Number of coordinates which determines the number of unknowns in system of equations and the matrix sizes
	Index GetNumberOfComputationCoordinates() const { return numberOfCoordinatesODE2 + numberOfCoordinatesODE1 + numberOfCoordinatesAE; }

	//! Write (Reference) access to:global number of ODE2 coordinates (sum of all node ODE2 coordinates)
	Index& GetNumberOfCoordinatesODE2() { return numberOfCoordinatesODE2; }
	//! Read (Reference) access to:global number of ODE2 coordinates (sum of all node ODE2 coordinates)
	const Index& GetNumberOfCoordinatesODE2() const { return numberOfCoordinatesODE2; }

	//! Write (Reference) access to:global number of ODE1 coordinates (sum of all node ODE1 coordinates)
	Index& GetNumberOfCoordinatesODE1() { return numberOfCoordinatesODE1; }
	//! Read (Reference) access to:global number of ODE1 coordinates (sum of all node ODE1 coordinates)
	const Index& GetNumberOfCoordinatesODE1() const { return numberOfCoordinatesODE1; }

	//! Write (Reference) access to:global number of AE coordinates (sum of all node AE coordinates)
	Index& GetNumberOfCoordinatesAE() { return numberOfCoordinatesAE; }
	//! Read (Reference) access to:global number of AE coordinates (sum of all node AE coordinates)
	const Index& GetNumberOfCoordinatesAE() const { return numberOfCoordinatesAE; }

	//! Write (Reference) access to:global number of Data variable (sum of all node Data variable)
	Index& GetNumberOfCoordinatesData() { return numberOfCoordinatesData; }
	//! Read (Reference) access to:global number of Data variable (sum of all node Data variable)
	const Index& GetNumberOfCoordinatesData() const { return numberOfCoordinatesData; }

	//! compute ODE2 ltg indices for marker (which either composes the ltg of a connector using two markers, or may be used e.g. for markers in CContact)
	void ComputeMarkerODE2LTGarray(Index markerNumber, ArrayIndex& ltgListODE2, bool resetFlag = true) const;

	//! compute ODE1+Data ltg indices for marker (which either composes the ltg of a connector using two markers, or may be used e.g. for markers in CContact)
	void ComputeMarkerODE1DataLTGarray(Index markerNumber, ArrayIndex& ltgListODE1, ArrayIndex& ltgListData, bool resetFlag = true) const;

	//! compute MarkerDataStructure for a given connector (using its markers); used in ComputeSystemODE2RHS, GetOutputVariableConnector, etc.; implemented in CSystem.cpp
	void ComputeMarkerDataStructure(const CObjectConnector* connector, bool computeJacobian, MarkerDataStructure& markerDataStructure) const;

	//! compute MarkerDataStructure for computation of Connector Jacobians (no AE, diff of Jacobian needed)
	//! jacobian derivative times constant vector v, e.g.: d(Jpos.T @ v)/dq
	void ComputeMarkerDataStructureJacobianODE2(const CObjectConnector* connector, const Vector& v, MarkerDataStructure& markerDataStructure) const;

	void Print(std::ostream& os) const
	{
		os << "CSystemData";
		os << "  cData = " << cData << "\n";
		os << "  cObjects = " << cObjects << "\n";
		os << "  cNodes = " << cNodes << "\n";
		os << "  cMaterials = " << cMaterials << "\n";
		os << "  cMarkers = " << cMarkers << "\n";
		os << "  cLoads = " << cLoads << "\n";
		os << "  cSensors = " << cSensors << "\n";
		os << "  localToGlobalODE2 = " << localToGlobalODE2 << "\n";
		os << "  localToGlobalODE1 = " << localToGlobalODE1 << "\n";
		os << "  localToGlobalAE = " << localToGlobalAE << "\n";
		os << "  localToGlobalData = " << localToGlobalData << "\n";
		os << "  numberOfCoordinatesODE2 = " << numberOfCoordinatesODE2 << "\n";
		os << "  numberOfCoordinatesODE1 = " << numberOfCoordinatesODE1 << "\n";
		os << "  numberOfCoordinatesAE = " << numberOfCoordinatesAE << "\n";
		os << "  numberOfCoordinatesData = " << numberOfCoordinatesData << "\n";
		os << "\n";
	}

	friend std::ostream& operator<<(std::ostream& os, const CSystemData& object)
	{
		object.Print(os);
		return os;
	}

};

//! compute ltg indices for marker (which either composes the ltg of a connector using two markers, or may be used e.g. for markers in CContact)
inline void CSystemData::ComputeMarkerODE2LTGarray(Index markerNumber, ArrayIndex& ltgListODE2, bool resetFlag) const
{
	if (resetFlag) { ltgListODE2.SetNumberOfItems(0); }

	//moved here from CSystem:

	//pout << "build LTG for " << objectIndex << " (=connector), marker " << markerNumber << "\n";
	CMarker* marker = GetCMarkers()[markerNumber];
	if (marker->GetType() & Marker::Object) //was before::Object
	{
		Index objectNumber = marker->GetObjectNumber();
		const CObject& object = *(GetCObjects()[objectNumber]);

		//object2 can't be a connector, so must have nodes
		for (Index j = 0; j < object.GetNumberOfNodes(); j++)
		{
			const CNode* node = object.GetCNode(j);
			//pout << "  node ODE2=" << node->GetNumberOfODE2Coordinates() << "\n";
			if (node->GetNumberOfODE2Coordinates())
			{
				Index gIndex = node->GetGlobalODE2CoordinateIndex();
				for (Index i = 0; i < node->GetNumberOfODE2Coordinates(); i++)
				{
					ltgListODE2.Append(gIndex + i);
				}
			}
		}
	}
	if (marker->GetType() & Marker::Node) //marker can be object + node ==> sliding joing
	{
		Index nodeNumber = marker->GetNodeNumber();
		CNode* node = GetCNodes()[nodeNumber];

		if (node->GetNumberOfODE2Coordinates())
		{
			Index gIndex = node->GetGlobalODE2CoordinateIndex();
			for (Index i = 0; i < node->GetNumberOfODE2Coordinates(); i++)
			{
				ltgListODE2.Append(gIndex + i);
			}
		}
	}
	else if (!(marker->GetType() & Marker::Node) && !(marker->GetType() & Marker::Object))
	{
		pout << "ComputeMarkerODE2LTGarray: ERROR: invalid MarkerType: not implemented in CSystem::AssembleLTGLists\n";
	}

}

//! compute ODE1+Data ltg indices for marker (which either composes the ltg of a connector using two markers, or may be used e.g. for markers in CContact)
inline void CSystemData::ComputeMarkerODE1DataLTGarray(Index markerNumber, ArrayIndex& ltgListODE1, ArrayIndex& ltgListData, bool resetFlag) const
{
	CMarker* marker = GetCMarkers()[markerNumber];
	if (marker->GetType() & Marker::Object) //was before::Object
	{
		Index objectNumber = marker->GetObjectNumber();
		const CObject& object = *(GetCObjects()[objectNumber]);

		//pout << "  nNodes=" << object.GetNumberOfNodes() << "\n";

		//object2 can't be a connector, so must have nodes
		for (Index j = 0; j < object.GetNumberOfNodes(); j++)
		{
			const CNode* node = object.GetCNode(j);
			//pout << "  node ODE2=" << node->GetNumberOfODE2Coordinates() << "\n";
			if (node->GetNumberOfODE1Coordinates())
			{
				Index gIndex = node->GetGlobalODE1CoordinateIndex();
				for (Index i = 0; i < node->GetNumberOfODE1Coordinates(); i++)
				{
					ltgListODE1.Append(gIndex + i);
				}
			}
			//exclude AE-coordinates, because markers should not act on algebraic coordinates (e.g. rigid body nodes with Euler parameters)
			//if (node->GetNumberOfAECoordinates())
			//{
			//	Index gIndex = node->GetGlobalAECoordinateIndex();
			//	for (Index i = 0; i < node->GetNumberOfAECoordinates(); i++)
			//	{
			//		ltgListAE.Append(gIndex + i);
			//	}
			//}
			if (node->GetNumberOfDataCoordinates())
			{
				Index gIndex = node->GetGlobalDataCoordinateIndex();
				for (Index i = 0; i < node->GetNumberOfDataCoordinates(); i++)
				{
					ltgListData.Append(gIndex + i);
				}
			}
		}
	}
	if (marker->GetType() & Marker::Node) //marker can be object + node ==> sliding joing
	{
		Index nodeNumber = marker->GetNodeNumber();
		CNode* node = GetCNodes()[nodeNumber];

		if (node->GetNumberOfODE1Coordinates())
		{
			Index gIndex = node->GetGlobalODE1CoordinateIndex();
			for (Index i = 0; i < node->GetNumberOfODE1Coordinates(); i++)
			{
				ltgListODE1.Append(gIndex + i);
			}
		}
		//exclude AE-coordinates, because markers should not act on algebraic coordinates (e.g. rigid body nodes with Euler parameters)
		//if (node->GetNumberOfAECoordinates())
		//{
		//	Index gIndex = node->GetGlobalAECoordinateIndex();
		//	for (Index i = 0; i < node->GetNumberOfAECoordinates(); i++)
		//	{
		//		ltgListAE.Append(gIndex + i);
		//	}
		//}
		if (node->GetNumberOfDataCoordinates())
		{
			Index gIndex = node->GetGlobalDataCoordinateIndex();
			for (Index i = 0; i < node->GetNumberOfDataCoordinates(); i++)
			{
				ltgListData.Append(gIndex + i);
			}
		}
	}
	else if (!(marker->GetType() & Marker::Node) && !(marker->GetType() & Marker::Object))
	{
		pout << "ComputeMarkerODE1DataLTGarray: ERROR: invalid MarkerType: not implemented in CSystem::AssembleLTGLists\n";
	}


}


//!markerdata computed in CSystemData because needed for sensors
//!synchronize with ComputeMarkerDataStructureJacobianODE2 function!
inline void CSystemData::ComputeMarkerDataStructure(const CObjectConnector* connector, bool computeJacobian, MarkerDataStructure& markerDataStructure) const
{
	const ArrayIndex& markerNumbers = connector->GetMarkerNumbers();
	Index nMarkers = connector->GetMarkerNumbers().NumberOfItems();
	if (nMarkers != 2) { CHECKandTHROWstring("CSystemData::ComputeMarkerDataStructure(...): Number of connector markers != 2 not implemented"); }
	markerDataStructure.SetTime(GetCData().currentState.GetTime());

	if ((Index)connector->GetType() & (Index)CObjectType::Constraint)
	{
		const CObjectConstraint* constraint = (CObjectConstraint*)connector;
		Index AEindex = constraint->GetGlobalAECoordinateIndex();
		Index nAEcoords = constraint->GetAlgebraicEquationsSize();
		markerDataStructure.GetLagrangeMultipliers().LinkDataTo(GetCData().currentState.AECoords, AEindex, nAEcoords);
	}
	for (Index k = 0; k < nMarkers; k++)
	{
		GetCMarkers()[markerNumbers[k]]->ComputeMarkerData(*this, computeJacobian, markerDataStructure.GetMarkerData(k));
	}
}

////!synchronize with ComputeMarkerDataStructure function!
////!compute jacobian derivative times constant vector v, e.g.: d(Jpos.T @ v)/dq
//inline void CSystemData::ComputeMarkerDataStructureJacobianODE2(const CObjectConnector* connector, const Vector& v, MarkerDataStructure& markerDataStructure) const
//{
//	const ArrayIndex& markerNumbers = connector->GetMarkerNumbers();
//	Index nMarkers = connector->GetMarkerNumbers().NumberOfItems();
//	markerDataStructure.SetTime(GetCData().currentState.GetTime()); //only needed for user functions, but kept for future
//
//	for (Index k = 0; k < nMarkers; k++)
//	{
//		GetCMarkers()[markerNumbers[k]]->ComputeMarkerDataJacobianDerivative(*this, v, markerDataStructure.GetMarkerData(k));
//	}
//}




#endif
