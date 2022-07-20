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
	static const Index numberOfMarkerData = 2;
private:
	MarkerData markerData[numberOfMarkerData];
	Real t; //!< add time for user functions or time in constraints, because they do not have nodes
	LinkedDataVector lagrangeMultipliers; //for constraint equation evaluation; WORKAROUND, in order not to access system coordinates in ComputeAlgebraicEquations
	//ResizableVector connectorForceJac; //for computation of connector jacobian; WORKAROUND; needs to be filled by connector on ComputeODE2LHS

public:
	//! get number of marker Data structures ==> for conventional connectors it is 2, but could be different for complex joints (e.g. sliding joint)
	Index GetNumberOfMarkerData() const { return numberOfMarkerData; }
	//! read access to markerData
	const MarkerData& GetMarkerData(const Index& i) const { return markerData[i]; }
	//! write access to markerData
	MarkerData& GetMarkerData(const Index& i) { return markerData[i]; }

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

#endif
