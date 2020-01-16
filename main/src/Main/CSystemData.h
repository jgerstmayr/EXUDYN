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
				- weblink: missing
				
************************************************************************************************ */
#pragma once

//#include <ostream>

#include "Linalg/BasicLinalg.h"		//includes Vector.h
#include "Utilities/ObjectContainer.h"	

#include "Main/OutputVariable.h" 
#include "Main/CData.h"		//includes ReleaseAssert.h and BasicDefinitions.h

#include "System/CMaterial.h"			//includes ReleaseAssert.h 
#include "System/CObjectBody.h"			//includes OutputVariable.h and CObject.h
#include "System/CNode.h"				//includes ReleaseAssert.h, BasicDefinitions.h, ResizeableArray.h, LinkedDataVector.h
#include "System/CMarker.h"		//needs nodes and bodies
#include "System/CLoad.h"				//needs markers
#include "System/CObjectConnector.h"	//includes OutputVariable.h and CObject.h




class CSystemData //
{
protected: //
	CData cData;                                    //!< computational data for all configurations (current, initial, etc.); this data is available in CNode
	ResizableArray<CObject*> cObjects;              //!< container for computational objects
	ResizableArray<CNode*> cNodes;                  //!< container for computational nodes
	ResizableArray<CMaterial*> cMaterials;          //!< container for computational materials
	ResizableArray<CMarker*> cMarkers;              //!< container for computational markers
	ResizableArray<CLoad*> cLoads;                  //!< container for computational loads
	ObjectContainer<ArrayIndex> localToGlobalODE2;  //!< CObject local to global ODE2 (Second order ODEs) coordinate indices transformation
	ObjectContainer<ArrayIndex> localToGlobalODE1;  //!< CObject local to global ODE1 (first order ODEs) coordinate indices transformation
	ObjectContainer<ArrayIndex> localToGlobalAE;    //!< CObject local to global AE (algebraic variables) coordinate indices transformation
	ObjectContainer<ArrayIndex> localToGlobalData;  //!< CObject local to global Data coordinate indices transformation
	Index numberOfCoordinatesODE2;                  //!< global number of ODE2 coordinates (sum of all node ODE2 coordinates); must be synchronous to NumberOfItems in SystemState Vectors
	Index numberOfCoordinatesODE1;                  //!< global number of ODE1 coordinates (sum of all node ODE1 coordinates); must be synchronous to NumberOfItems in SystemState Vectors
	Index numberOfCoordinatesAE;                    //!< global number of AE coordinates (sum of all node AE coordinates); must be synchronous to NumberOfItems in SystemState Vectors
	Index numberOfCoordinatesData;                  //!< global number of Data variables/coordinates (sum of all node Data variables); must be synchronous to NumberOfItems in SystemState Vectors

public: //

	bool isODE2RHSjacobianComputation;

	// access functions
	//! clone object; specifically for copying instances of derived class, for automatic memory management e.g. in ObjectContainer
	CSystemData* GetClone() const { return new CSystemData(*this); }

	//! Specific destructor do deallocate data (allocated in MainSystem/ObjectFactory)
	virtual ~CSystemData() { Reset(); }

	void Reset()
	{
		cData = CData();

		for (auto item : cLoads) { delete item; }
		for (auto item : cMarkers) { delete item; }
		for (auto item : cMaterials) { delete item; }
		for (auto item : cNodes) { delete item; }
		for (auto item : cObjects) { delete item; }

		cLoads.Flush();
		cMarkers.Flush();
		cMaterials.Flush();
		cNodes.Flush();
		cObjects.Flush();

		numberOfCoordinatesODE2 = 0;
		numberOfCoordinatesODE1 = 0;
		numberOfCoordinatesAE = 0;
		numberOfCoordinatesData = 0;

		localToGlobalODE2.Flush();
		localToGlobalODE1.Flush();
		localToGlobalAE.Flush();
		localToGlobalData.Flush();

	}


	//! Write (Reference) access to:computational data for all configurations (current, initial, etc.); this data is available in CNode
	CData& GetCData() { return cData; }
	//! Read (Reference) access to:computational data for all configurations (current, initial, etc.); this data is available in CNode
	const CData& GetCData() const { return cData; }
	//void SetCData(const CData& cDataInit) { cData = cDataInit; }

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

	virtual void Print(std::ostream& os) const
	{
		os << "CSystemData";
		os << "  cData = " << cData << "\n";
		os << "  cObjects = " << cObjects << "\n";
		os << "  cNodes = " << cNodes << "\n";
		os << "  cMaterials = " << cMaterials << "\n";
		os << "  cMarkers = " << cMarkers << "\n";
		os << "  cLoads = " << cLoads << "\n";
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


