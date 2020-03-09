/** ***********************************************************************************************
* @class	    CObjectConnector
* @brief		Base class for connectors, which can add equations based on two objects/nodes
* @details		Details:
* 				- base class for connectors, which can add equations based on two objects/nodes;
*				- the interface to objects/nodes are marker; 
*				- specifically, connectors can be spring/dampers (penalty constraint), or real constraints 
*				- for constraints, use the derived CObjectConstraint class
*
* @author		Gerstmayr Johannes
* @date			2018-05-17 (generated)
* @pre			...
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
* 				
*
* *** Example code ***
*
************************************************************************************************ */
#pragma once


#include "Main/MarkerData.h" 

namespace PostNewtonFlags {
	enum Type {
		_None = 0,
		UpdateLTGLists = 1 << 0	//!< signals the solver that a change was such that the localToGlobal index list must be updated for the object
	};
}


class CMarker;

class CObjectConnector : public CObject
{
protected:
    //marker numbers available in spezialized object! ResizableArray<CMarker*> markers;

public:
    //! get an exact clone of *this, must be implemented in all derived classes! Necessary for better handling in ObjectContainer
    virtual CObjectConnector* GetClone() const { return new CObjectConnector(*this); }
    virtual const char* GetName() const { return "CObjectConnector"; }

    virtual void Print(std::ostream& os) const {
        os << "CObjectConnector:";
        CObject::Print(os);
    }

	//! connectors are attached to markers; this function must be overwritten in derived class
	virtual const ArrayIndex& GetMarkerNumbers() const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::GetMarkerNumbers"); ArrayIndex* v = new ArrayIndex(0); return *v; }

	//! connector may have nodes (data coordinates)
	virtual Index GetNumberOfNodes() const override { return 0; }
		
	virtual Marker::Type GetRequestedMarkerType() const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::RequestedMarkerType"); return Marker::_None; }

    virtual CObjectType GetCObjectType() const { return CObjectType::Connector; }

	//! Return true, if connector does not use algebraic equations ==> springs, damper, etc.
	virtual bool IsPenaltyConnector() const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::IsPenaltyConnector"); return false; }

	//! Return true, if connector is time dependent (default is false); used for userFunctions in constraints, e.g., to drive offsets, etc.
	virtual bool IsTimeDependent() const { return false; }

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//specific Connector/Marker functions!
	//! compute right-hand-side (RHS) of second order ordinary differential equations (ODE) to 'ode2rhs'; provides time t for user functions
	virtual void ComputeODE2RHS(Vector& ode2Rhs, const MarkerDataStructure& markerData) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeODE2RHS"); }

	//! compute algebraic equations to 'algebraicEquations', which has dimension GetAlgebraicEquationsSize(); q are the system coordinates
	virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, bool useIndex2 = false) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeAlgebraicEquations"); }

	//! compute derivative of right-hand-side (RHS) w.r.t q of second order ordinary differential equations (ODE) [optional w.r.t. ODE2_t variables as well, if flag ODE2_ODE2_t_function set in GetAvailableJacobians()]; jacobian [and jacobianODE2_t] has dimension GetODE2Size() x GetODE2Size(); this is the local tangent stiffness matrix;
	virtual void ComputeJacobianODE2_ODE2(ResizableMatrix& jacobian, ResizableMatrix& jacobian_ODE2_t, const MarkerDataStructure& markerData) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeODE2RHSJacobian"); }

	//! compute derivative of algebraic equations w.r.t. ODE2 in jacobian [and w.r.t. ODE2_t coordinates in jacobian_t if flag ODE2_t_AE_function is set] [and w.r.t. AE coordinates if flag AE_AE_function is set in GetAvailableJacobians()]; jacobian[_t] has dimension GetAlgebraicEquationsSize() x (GetODE2Size() + GetODE1Size() [+GetAlgebraicEquationsSize()]); q are the system coordinates; markerData provides according marker information to compute jacobians
	virtual void ComputeJacobianAE(ResizableMatrix& jacobian, ResizableMatrix& jacobian_t, ResizableMatrix& jacobian_AE, const MarkerDataStructure& markerData, Real t) const { CHECKandTHROWstring("ERROR: illegal call to CObject::ComputeJacobianAE"); }

	//! get output variable 'variableType' in (vector) value; for connectors, marker information must be provided as in ComputeODE2RHS (e.g. to compute distance)
	virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Vector& value) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::GetOutputVariableConnector(...)"); }

	////! compute derivative of right-hand-side (RHS) w.r.t q of second order ordinary differential equations (ODE) to 'ode2rhs', which has dimension GetODE2Size() x GetODE2Size(); this is the tangent stiffness matrix; q are the system coordinates
	//virtual void ComputeODE2RHSJacobian(Matrix& jac, const MarkerDataStructure& markerData) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeODE2RHSJacobian"); }

	////! compute derivative of right-hand-side (RHS) w.r.t qt (velocities) of second order ordinary differential equations (ODE) to 'ode2rhs', which has dimension GetODE2Size() x GetODE2Size(); this is the damping / gyroscopic matrix; q are the system coordinates
	//virtual void ComputeODE2RHSVelocitiesJacobian(Matrix& jac, const MarkerDataStructure& markerData) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeODE2RHSVelocitiesJacobian"); }

	////! compute derivative of algebraic equations w.r.t. all ODE1+ODE2 coordinates and if GetAvailableJacobians&JacobianType::AE_AE_compute also w.r.t. AE coordinates; dimension of jacobian is therefore GetAlgebraicEquationsSize() x (marker0+marker1)(GetODE1Size()+GetODE2Size() [+GetAlgbraicEquationsSize()]); q are the marker0/1 coordinates ==> stored in ltgODE2 of connector
	//virtual void ComputeJacobianAE(ResizableMatrix& jacobian, const MarkerDataStructure& markerData) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeJacobianAE"); }

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//discontinuous iteration (contact, friction, plasticity, ...) ==> put into SpecialObjectFunctionsContainer?
	//! flag to be set for connectors, which use DiscontinuousIteration
	virtual bool HasDiscontinuousIteration() const { return false; }

	//! function to initialize discontinuous iterations prior to simulation (e.g. reset contact conditions, etc.)
	//virtual void InitializeDiscontinuousIteration() {  } ==> use initial conditions of data coordinates?

	//! function called after Newton method; returns a residual error (force); gets marker data for current configuration to achieve the correct behavior
	virtual Real PostNewtonStep(const MarkerDataStructure& markerDataCurrent, PostNewtonFlags::Type& flags) { return 0; };
	//virtual Real PostNewtonStep(const MarkerDataStructure& markerDataStartOfStep, const MarkerDataStructure& markerDataCurrent) { return 0; };

	//! function called after discontinuous iterations have been completed for one step (e.g. to finalize history variables and set initial values for next step)
	virtual void PostDiscontinuousIterationStep() {};

private:
	//the following functions are not available for connectors, because they need the markerData structure
	//they could be called via the base class, therefore exceptions are thrown!
	virtual void ComputeJacobianODE2_ODE2(Matrix& jacobian, Matrix& jacobian_ODE2_t) const final { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeJacobianODE2_ODE2"); }; //!< this is the non-connector function, which is not available for connectors!
	virtual void ComputeJacobianAE(Matrix& jacobian, ResizableMatrix& jacobian_t, Matrix& jacobian_AE) const final { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeJacobianAE"); }; //!< this is the non-connector function, which is not available for connectors!
	virtual void GetOutputVariable(OutputVariableType variableType, Vector& value) const final { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::GetOutputVariable"); } //!< this is the non-connector function, which is not available for connectors!

};




//! constraints have algebraic variables, directly obtained from system; they do not need nodes
//!  most functions are duplicates from CObjectConnector
class CObjectConstraint : public CObjectConnector
{
protected:
	Index globalAECoordinateIndex;                //!< refers to the place in the global AE coordinate vector; Lagrange multipliers are not obtained via nodes

public:
	//! get an exact clone of *this, must be implemented in all derived classes! Necessary for better handling in ObjectContainer
	virtual CObjectConnector* GetClone() const { return new CObjectConnector(*this); }
	virtual const char* GetName() const { return "CObjectConnector"; }

	virtual void Print(std::ostream& os) const {
		os << "CObjectConstraint:";
		CObject::Print(os);
	}

	virtual CObjectType GetCObjectType() const { return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint); }
	
	//! Return true, if connector does not use algebraic equations ==> springs, damper, etc.
	virtual bool IsPenaltyConnector() const { return false; }

	//! Return true, if constraint implements velocity level equations (e.g. for Index reduction)
	virtual bool HasVelocityEquations() const { return false; }

	//! Return true, if constraint currently is formulated at velocity level (e.g. coordinate constraint ==> this information is needed for correct jacobian computation)
	virtual bool UsesVelocityLevel() const { return false; }

	//! get global starting index (index of first algebraic variable) at global algebraic coordinates vector in CData
	virtual Index GetGlobalAECoordinateIndex() const { return globalAECoordinateIndex; }
	//! set global starting index (index of first algebraic variable) at global algebraic coordinates vector in CData
	virtual void SetGlobalAECoordinateIndex(Index globalIndex) { globalAECoordinateIndex = globalIndex; }

	//! get current algebraic coordinate with local index
	//  implementation in CObjectBody.cpp
	virtual Real GetCurrentAEcoordinate(Index localIndex) const;

	////! Compute time derivative of algebraic equations to vector 'algebraicEquations', which has dimension GetAlgebraicEquationsSize()
	//virtual void ComputeAlgebraicEquations_t(Vector& algebraicEquations, const MarkerDataStructure& markerData) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeAlgebraicEquations_t"); }


};


