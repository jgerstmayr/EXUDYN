/** ***********************************************************************************************
* @class        MarkerData
* @brief        A data structure used to transferdata between markers and connectors/loads during computation
*
* @author       Gerstmayr Johannes
* @date         2018-06-13 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#ifndef MARKERDATA__H
#define MARKERDATA__H

#include "Utilities/ReleaseAssert.h"
#include <initializer_list>
#include "Utilities/BasicDefinitions.h" //defines Real
#include "Utilities/ResizableArray.h" 
#include "Linalg/ResizableMatrix.h" 


//! a structure, which contains temporary data from markers (position, orientation, velocity, Jacobians, ...)
class MarkerData
{
public:
	Vector3D position;			//position of marker
	Vector3D velocity;			//velocity of marker
	Matrix3D orientation;//rotation matrix describing orientation of marker
	//ResizableMatrix orientation;//rotation matrix describing orientation of marker
	Vector3D angularVelocityLocal;	//local (=body fixed) angular velocity of marker

	ResizableMatrix positionJacobian;	//d(pos)/dq
	ResizableMatrix rotationJacobian;	//d(omega)/dq_t
	ResizableMatrix jacobian;	//general jacobian, e.g. for coordinate marker; ?d(Rotv123)/dq for rigid bodies?

	//ResizableMatrix positionJacobianDerivative;	//d(d(pos)/dq*v)/dq //depends on connector configuration!!!
	//ResizableMatrix rotationJacobianDerivative;	//d(d(omega)/dq_t*v)/dq //depends on connector configuration!!!
	ResizableMatrix jacobianDerivative;	//general d(jacobian*v)/dq; has always (size of q) x (size of q)

	//removed and replaced by vectorValue! Real value;					//general value, e.g. for coordinate marker
	//removed and replaced by vectorValue_t! Real value_t;				//general value at velocity level, e.g. for coordinate marker
	ResizableVector vectorValue;//general vector value, e.g. for ANCF shape marker; changed from Vector to ResizableVector to avoid memory allocation
	ResizableVector vectorValue_t;		//general vector value at velocity level, e.g. for ANCF shape marker; changed from Vector to ResizableVector to avoid memory allocation

	bool velocityAvailable;		//used for value/value_t, vectorValue/vectorValue_t, position/velocity, ... to determine, if velocities are available

	//! helper function for e.g. CMarkerBodyCable2DCoordinates:
	const Real& GetHelper() const { return angularVelocityLocal[0]; }
	Real& GetHelper() { return angularVelocityLocal[0]; }
	const Real& GetHelper2() const { return angularVelocityLocal[1]; }
	Real& GetHelper2() { return angularVelocityLocal[1]; }
	const ResizableMatrix& GetHelperMatrix() const { return jacobianDerivative; }
	ResizableMatrix& GetHelperMatrix() { return jacobianDerivative; }
};

//this class contains several MarkerData structures ==> derive from this class for special connectors
class MarkerDataStructure
{
public:
	static const Index staticNumberOfMarkerData = 2; //! this is the absolute minimum of MarkerData; some functions access first markerData without checks (as it is always possible)
private:
	MarkerData markerDataStatic[staticNumberOfMarkerData]; //!< for 0-2 markerData, otherwise use dynamic array !
	MarkerData* markerData;					//!< dynamic marker data
	Index numberOfMarkerData;				//!< currently operated marker data size; to check inconsistencies
	Index allocatedMarkerDataSize;			//!< size of allocated data; either 0 or larger than staticNumberOfMarkerData
	
	//further transmitted data:
	Real t;									//!< add time for user functions or time in constraints, because they do not have nodes
	LinkedDataVector lagrangeMultipliers;	//!< for constraint equation evaluation; WORKAROUND, in order not to access system coordinates in ComputeAlgebraicEquations

public:
	//! initialize with static markerdata; works as in old case with static data; no memory allocation
	MarkerDataStructure()
	{
		//pout << "MarkerDataStructure()\n";
		//this is the standard:
		allocatedMarkerDataSize = 0;
		ResetMarkerData();
	};

	//! copy constructor; needed, because linking of markerData needs to be assigned to copied data!
	MarkerDataStructure(const MarkerDataStructure& data)
	{
		CopyFrom(data);
	}

	//! copy data from other object, do not delete local data
	void CopyFrom(const MarkerDataStructure& data)
	{
		for (Index i = 0; i < staticNumberOfMarkerData; i++)
		{
			markerDataStatic[i] = data.markerDataStatic[i];
		}

		numberOfMarkerData = data.numberOfMarkerData;
		allocatedMarkerDataSize = data.allocatedMarkerDataSize;
		t = data.t;
		lagrangeMultipliers = data.lagrangeMultipliers;

		if (allocatedMarkerDataSize == 0)
		{
			markerData = &markerDataStatic[0];
		}
		else
		{
			markerData = new MarkerData[allocatedMarkerDataSize](); //() adds initialization (needed for ResizableVector, etc.)
			for (Index i = 0; i < allocatedMarkerDataSize; i++) //initialize data
			{
				markerData[i] = data.markerData[i];
			}
		}
	}

	//! remove move constructor as this needs to be designed specially
	MarkerDataStructure(MarkerDataStructure&& other) = delete;

	//! special assignment operator as with copy constructor
	MarkerDataStructure& operator=(const MarkerDataStructure& data)
	{
		if (this == &data) { return *this; }

		ResetMarkerData();
		CopyFrom(data);
		return *this;
	}
	//! reset function if reset for temporary data needed
	void ResetMarkerData()
	{
		if (allocatedMarkerDataSize)
		{
			delete[] markerData;
			allocatedMarkerDataSize = 0;
		}
		markerData = &markerDataStatic[0];
		numberOfMarkerData = staticNumberOfMarkerData;
	}

	//! adjust memory to larger number of MarkerData if needed
	void SetNumberOfMarkerData(Index size)
	{
		//pout << "SetNumberOfMarkerData: " << size << "()\n";
		//CHECKandTHROW(size == 2, "MarkerDataStructure::SetNumberOfMarkerData called with size != 2");
		if (size > staticNumberOfMarkerData && size > allocatedMarkerDataSize)
		{
			//in this case, we need (possibly more) allocated data
			if (allocatedMarkerDataSize)
			{
				delete[] markerData;
			}
			markerData = new MarkerData[size](); //() adds initialization (needed for ResizableVector, etc.)
			//for (Index i = 0; i < size; i++) //initialize data
			//{
			//	markerData[i] = MarkerData();
			//}
			allocatedMarkerDataSize = size;
		}

		numberOfMarkerData = size;
	}

	virtual ~MarkerDataStructure()
	{
		if (allocatedMarkerDataSize)
		{
			delete[] markerData;
		}
	}

	static Index GetMaxNumberOfMarkerData() { return EXUstd::MAXINDEX; }
	
	//! get number of marker Data structures ==> for conventional connectors it is 2, but could be different for complex joints (e.g. sliding joint)
	Index GetNumberOfMarkerData() const { return numberOfMarkerData; }

	//! read access to markerData
	const MarkerData& GetMarkerData(const Index& i) const { 
		CHECKandTHROW(i < numberOfMarkerData, "GetMarkerData const: invalid index");
		return markerData[i]; 
	}
	//! write access to markerData
	MarkerData& GetMarkerData(const Index& i) { 
		CHECKandTHROW(i < numberOfMarkerData, "GetMarkerData: invalid index");
		return markerData[i];
	}

	//! write access to time t
	void SetTime(Real time) { t = time; }
	//! write access to time t
	Real GetTime() const { return t; }

	//! read access to lagrangeMultipliers
	const LinkedDataVector& GetLagrangeMultipliers() const { return lagrangeMultipliers; }
	//! write access to lagrangeMultipliers
	LinkedDataVector& GetLagrangeMultipliers() { return lagrangeMultipliers; }

	////! read access to connectorForceJac
	//const ResizableVector& GetConnectorForceJac() const { return connectorForceJac; }
	////! write access to connectorForceJac
	//ResizableVector& GetConnectorForceJac() { return connectorForceJac; }
};


//DELETE:
//OLD const-sized MarkerData:
//this class contains several MarkerData structures ==> derive from this class for special connectors
//class MarkerDataStructure
//{
//public:
//	//static const Index numberOfMarkerData = 2;
//	Index GetMaxNumberOfMarkerData() const { return 2; }
//private:
//	MarkerData markerData[numberOfMarkerData]; //for 0-2 markerData, otherwise use dynamic array !
//	Real t; //!< add time for user functions or time in constraints, because they do not have nodes
//	LinkedDataVector lagrangeMultipliers; //for constraint equation evaluation; WORKAROUND, in order not to access system coordinates in ComputeAlgebraicEquations
//	//ResizableVector connectorForceJac; //for computation of connector jacobian; WORKAROUND; needs to be filled by connector on ComputeODE2LHS
//
//public:
//	//! get number of marker Data structures ==> for conventional connectors it is 2, but could be different for complex joints (e.g. sliding joint)
//	Index GetNumberOfMarkerData() const { return numberOfMarkerData; }
//	//! read access to markerData
//	const MarkerData& GetMarkerData(const Index& i) const { return markerData[i]; }
//	//! write access to markerData
//	MarkerData& GetMarkerData(const Index& i) { return markerData[i]; }
//
//	//! write access to time t
//	void SetTime(Real time) { t = time; }
//	//! write access to time t
//	Real GetTime() const { return t; }
//
//	//! read access to lagrangeMultipliers
//	const LinkedDataVector& GetLagrangeMultipliers() const { return lagrangeMultipliers; }
//	//! write access to lagrangeMultipliers
//	LinkedDataVector& GetLagrangeMultipliers() { return lagrangeMultipliers; }
//
//	////! read access to connectorForceJac
//	//const ResizableVector& GetConnectorForceJac() const { return connectorForceJac; }
//	////! write access to connectorForceJac
//	//ResizableVector& GetConnectorForceJac() { return connectorForceJac; }
//};


#endif
