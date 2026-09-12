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
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
************************************************************************************************ */
#ifndef COBJECTCONNECTOR__H
#define COBJECTCONNECTOR__H


#include "Main/MarkerData.h" 

namespace PostNewtonFlags {
	enum Type {
		_None = 0,
		UpdateLTGLists = 1 << 0,	//!< signals the solver that a change was such that the localToGlobal index list must be updated for the object
		UpdateJacobian = 1 << 1,	//!< signals the solver that jacobian shall be updated
	};
}

class CMarker;

class CObjectConnector : public CObject
{
protected:
    //marker numbers available in spezialized object! ResizableArray<CMarker*> markers;

private:
	//the following functions are not available for connectors, because they need the markerData structure
	//they could be called via the base class, therefore exceptions are thrown!
	using CObject::ComputeODE2LHS; //avoid warnings in gcc
	using CObject::ComputeODE1RHS; //avoid warnings in gcc
	using CObject::ComputeJacobianODE2_ODE2;
	using CObject::ComputeJacobianAE;
	using CObject::GetOutputVariable;
	virtual void ComputeODE2LHS(Vector& ode2Lhs, Index objectNumber) const final { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeODE2LHS"); }//!< this is the non-connector function, which is not available for connectors!
	virtual void ComputeODE1RHS(Vector& ode1Rhs, Index objectNumber) const final { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeODE1LHS"); }//!< this is the non-connector function, which is not available for connectors!
	virtual void ComputeJacobianODE2_ODE2(EXUmath::MatrixContainer& jacobianODE2, JacobianTemp& temp, Real factorODE2, Real factorODE2_t, Index objectNumber, const ArrayIndex& ltg) const final { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeJacobianODE2_ODE2"); }//!< this is the non-connector function, which is not available for connectors!
	virtual void ComputeJacobianAE(ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE) const final { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeJacobianAE"); }//!< this is the non-connector function, which is not available for connectors!
	virtual void GetOutputVariable(OutputVariableType variableType, Vector& value, ConfigurationType configuration, Index objectNumber) const final { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::GetOutputVariable"); } //!< this is the non-connector function, which is not available for connectors!

public:
    //! get an exact clone of *this, must be implemented in all derived classes! Necessary for better handling in ObjectContainer
    virtual CObjectConnector* GetClone() const { return new CObjectConnector(*this); }
    virtual const char* GetName() const { return "CObjectConnector"; }

    virtual void Print(std::ostream& os) const {
        os << "CObjectConnector:";
        CObject::Print(os);
    }

	//! marker numbers for connectors (read); this function must be overwritten in derived class
	virtual const ArrayIndex& GetMarkerNumbers() const { CHECKandTHROWstring("ERROR: illegal call to const CObjectConnector::GetMarkerNumbers"); ArrayIndex* v = new ArrayIndex(0); return *v; }
	//! marker numbers for connectors (write); this function must be overwritten in derived class
	virtual ArrayIndex& GetMarkerNumbers() { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::GetMarkerNumbers"); ArrayIndex* v = new ArrayIndex(0); return *v; }

	//! connector may have nodes (data coordinates)
	virtual Index GetNumberOfNodes() const override { return 0; }

	//! requested marker types (or Marker::_None if mixed)
	virtual Marker::Type GetRequestedMarkerType() const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::RequestedMarkerType"); return Marker::_None; }

	//! requested number of markers for PreAssemble checks; default = 2
	virtual Index RequestedNumberOfMarkers() const { return 2; }

    virtual CObjectType GetCObjectType() const { return CObjectType::Connector; }

	//! Return true, if connector does not use algebraic equations ==> springs, damper, etc.
	virtual bool IsPenaltyConnector() const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::IsPenaltyConnector"); return false; }

	//! Return true, if connector is time dependent (default is false); used for userFunctions in constraints, e.g., to drive offsets, etc.
	virtual bool IsTimeDependent() const { return false; }

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//specific Connector/Marker functions!
	//! compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs' for CONNECTORS; provides objectNumber for user functions
	virtual void ComputeODE2LHS(Vector& ode2Lhs, const MarkerDataStructure& markerData, Index objectNumber) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeODE2LHS"); }

	//! compute right-hand-side (RHS) of first order ordinary differential equations (ODE) to 'ode1Rhs' for CONNECTORS, which has dimension GetODE1Size(); objectNumber contains items own number for user functions
	virtual void ComputeODE1RHS(Vector& ode1Rhs, const MarkerDataStructure& markerData, Index objectNumber) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeODE1LHS"); }

	//! compute algebraic equations to 'algebraicEquations', which has dimension GetAlgebraicEquationsSize(); q are the system coordinates
	virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool useIndex2 = false) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeAlgebraicEquations"); }

	//! compute derivative of right-hand-side (LHS) w.r.t q of second order ordinary differential equations (ODE) [optional w.r.t. ODE2_t variables as well, if flag ODE2_ODE2_t_function set in GetAvailableJacobians()]; jacobian [and jacobianODE2_t] has dimension GetODE2Size() x GetODE2Size(); this is the local tangent stiffness matrix;
	//OLD: virtual void ComputeJacobianODE2_ODE2(ResizableMatrix& jacobian, ResizableMatrix& jacobian_ODE2_t, const MarkerDataStructure& markerData) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeODE2LHSJacobian"); }
	virtual void ComputeJacobianODE2_ODE2(EXUmath::MatrixContainer& jacobianODE2, JacobianTemp& temp, Real factorODE2, Real factorODE2_t,
		Index objectNumber, const ArrayIndex& ltg, const MarkerDataStructure& markerData) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeJacobianODE2_ODE2"); }

	//! compute 'force' which is used for computation of derivative of jacobian; for position connectors this is a force, 
	//! while for position+rotation connectors, this is a generalized 6D force; used only in combination with ComputeJacobianODE2_ODE2
	virtual void ComputeJacobianForce6D(const MarkerDataStructure& markerData, Index objectNumber, Vector6D& force6D) const {
		CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeJacobianForce6D");
	}

	//! function to compute jacobian for connectors having a simple structure with a local jacobian 
	//! using K=d(F)/(dq), D=d(F)/(dq_t) ==> localJac = factorODE2*K + factorODE_t*D
	//! localJac is modified inside this function!
	//! jacobianODE2 is computed using the marker jacobians and the jacobianDerivative stored in markerData
	//! dense mode is used here; if activeConnector=false, jacobian becomes a zeros matrix
	//! works for coordinate connectors (isCoordinateConnector=true) or position/orientation markers (pure position if hasRotationJacobian=false)
	virtual void ComputeJacobianODE2_ODE2generic(ResizableMatrix& localJac, EXUmath::MatrixContainer& jacobianODE2, JacobianTemp& temp,
		Real factorODE2, Real factorODE2_t, Index objectNumber, const MarkerDataStructure& markerData, bool activeConnector, 
		bool isCoordinateConnector, bool hasRotationJacobian) const;

	//! compute derivative of algebraic equations w.r.t. ODE2 in jacobian [and w.r.t. ODE2_t coordinates in jacobian_t if flag ODE2_t_AE_function is set] [and w.r.t. AE coordinates if flag AE_AE_function is set in GetAvailableJacobians()]; jacobian[_t] has dimension GetAlgebraicEquationsSize() x (GetODE2Size() + GetODE1Size() [+GetAlgebraicEquationsSize()]); q are the system coordinates; markerData provides according marker information to compute jacobians
	virtual void ComputeJacobianAE(ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE, const MarkerDataStructure& markerData, Real t, Index itemIndex) const { CHECKandTHROWstring("ERROR: illegal call to CObject::ComputeJacobianAE"); }

	//! get output variable 'variableType' in (vector) value; for connectors, marker information must be provided as in ComputeODE2LHS (e.g. to compute distance)
	virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Index itemIndex, Vector& value) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::GetOutputVariableConnector(...)"); }

	////! compute derivative of right-hand-side (LHS) w.r.t q of second order ordinary differential equations (ODE) to 'ode2Lhs', which has dimension GetODE2Size() x GetODE2Size(); this is the tangent stiffness matrix; q are the system coordinates
	//virtual void ComputeODE2LHSJacobian(Matrix& jac, const MarkerDataStructure& markerData) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeODE2LHSJacobian"); }

	////! compute derivative of right-hand-side (LHS) w.r.t qt (velocities) of second order ordinary differential equations (ODE) to 'ode2Lhs', which has dimension GetODE2Size() x GetODE2Size(); this is the damping / gyroscopic matrix; q are the system coordinates
	//virtual void ComputeODE2LHSVelocitiesJacobian(Matrix& jac, const MarkerDataStructure& markerData) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeODE2LHSVelocitiesJacobian"); }

	////! compute derivative of algebraic equations w.r.t. all ODE1+ODE2 coordinates and if GetAvailableJacobians&JacobianType::AE_AE_compute also w.r.t. AE coordinates; dimension of jacobian is therefore GetAlgebraicEquationsSize() x (marker0+marker1)(GetODE1Size()+GetODE2Size() [+GetAlgbraicEquationsSize()]); q are the marker0/1 coordinates ==> stored in ltgODE2 of connector
	//virtual void ComputeJacobianAE(ResizableMatrix& jacobian, const MarkerDataStructure& markerData) const { CHECKandTHROWstring("ERROR: illegal call to CObjectConnector::ComputeJacobianAE"); }

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//discontinuous iteration (contact, friction, plasticity, ...) ==> put into SpecialObjectFunctionsContainer?
	//! flag to be set for connectors, which use DiscontinuousIteration
	virtual bool HasDiscontinuousIteration() const { return false; }

	//! function to initialize discontinuous iterations prior to simulation (e.g. reset contact conditions, etc.)
	//virtual void InitializeDiscontinuousIteration() {  } ==> use initial conditions of data coordinates?

	//! function called after Newton method; returns a residual error (force); input: marker data for current configuration to achieve the correct behavior; flags are modified, e.g. to recompute jacobian; recommendedStepSize = [< 0 for no recommendation, 0 for min step size, > 0 for reduction of step size to given value]
	virtual Real PostNewtonStep(const MarkerDataStructure& markerDataCurrent, Index itemIndex, PostNewtonFlags::Type& flags, Real& recommendedStepSize) { return 0; };
	//virtual Real PostNewtonStep(const MarkerDataStructure& markerDataStartOfStep, const MarkerDataStructure& markerDataCurrent) { return 0; };

	//! function called after discontinuous iterations have been completed for one step (e.g. to finalize history variables and set initial values for next step)
	virtual void PostDiscontinuousIterationStep() {};

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

	//NOT USED ANY MORE ==> UsesVelocityLevel//! Return true, if constraint implements velocity level equations (e.g. for Index reduction)
	//virtual bool HasVelocityEquations() const { return false; }

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


#endif
