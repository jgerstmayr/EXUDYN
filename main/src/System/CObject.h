/** ***********************************************************************************************
* @class	    CObject
* @brief		Computational objects; objects obtain their coordinates from nodes or they are linked to objects via markers
* @details		Details:
 				- ...
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

#include "Utilities/ReleaseAssert.h"
#include <initializer_list>
#include "Utilities/BasicDefinitions.h" //defines Real

#include "Utilities/ResizableArray.h"	//includes SlimArray.h and BasicFunctions.h
#include "Linalg/Matrix.h" 
#include "Linalg/LinkedDataVector.h"	//includes SlimVector.h and Vector.h
//#include "Utilities/ObjectContainer.h"	
#include "Main/OutputVariable.h" 

#include "System/CNode.h" 


//! Connectors link two bodies (e.g. by means of spring), constraints are Lagrange-multiplier based, FiniteElement additionally has shape functions, etc.
//! Body: contains its own node(s); finiteElement: connects several nodes; connectors: based on markers
enum class CObjectType {
	None = 0, //marks that no type is used
	Ground = 1 << 1,		//!< Ground (is also a Body) can connect to other bodies, but has not DOF
	Constraint = 1 << 2,	//!< Lagrange-multiplier based, leads to algebraic equations
	Connector = 1 << 3,		//!< Connects two objects (usually bodies); can be a spring-damper, but also a constraint
	Body = 1 << 4,			//!< has mass; measure position and velocity; forces can be applied
	SingleNoded = 1 << 5,	//!< does not connect nodes ==> limited nodal connectivity
	MultiNoded = 1 << 6,	//!< connects nodes (e.g. spring-damper or finite element)
	FiniteElement = 1 << 7, //!< object is flexible, nodal interpolation
	EndOfEnumList = 1 << 8  //!< KEEP THIS AS THE (2^i) MAXIMUM OF THE ENUM LIST!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//keep these lists synchronized with PybindModule.cpp lists
};


namespace JacobianType {
//! used mainly to show which jacobians are available analytically in objects; can be combined binary to see, which jacobian is available
	enum Type {
		None = 0,				//marks that no type is available
		ODE2_ODE2 = 1 << 1,		//derivative of ODE2 equations with respect to ODE2 variables
		ODE2_ODE2_t = 1 << 2,	//derivative of ODE2 equations with respect to ODE2_t (velocity) variables
		ODE1_ODE1 = 1 << 3,		//derivative of ODE1 equations with respect to ODE1 variables
		AE_ODE2 = 1 << 4,		//derivative of AE (algebraic) equations with respect to ODE2 variables
		AE_ODE2_t = 1 << 5,		//derivative of AE (algebraic) equations with respect to ODE2_t (velocity) variables
		AE_ODE1 = 1 << 6,		//derivative of AE (algebraic) equations with respect to ODE1 variables
		AE_AE = 1 << 7,			//derivative of AE (algebraic) equations with respect to AE variables
		//
		ODE2_ODE2_function = 1 << 8,	//function available for derivative of ODE2 equations with respect to ODE2 variables
		ODE2_ODE2_t_function = 1 << 9,	//function available for derivative of ODE2 equations with respect to ODE2_t (velocity) variables
		ODE1_ODE1_function = 1 << 10,	//function available for derivative of ODE1 equations with respect to ODE1 variables
		AE_ODE2_function = 1 << 11,		//function available for derivative of AE (algebraic) equations with respect to ODE2 variables
		AE_ODE2_t_function = 1 << 12,	//function available for derivative of AE (algebraic) equations with respect to ODE2_t (velocity) variables
		AE_ODE1_function = 1 << 13,		//function available for derivative of AE (algebraic) equations with respect to ODE1 variables
		AE_AE_function = 1 << 14,		//function available for derivative of AE (algebraic) equations with respect to AE variables
	};
}

class CObject;
class CSystemData;

//! this is a class belonging to CObject, which can be used to derive object-specific special functions, to be called under certain events (e.g. start of time step)
class SpecialObjectFunctionsContainer
{
private:

public:
    //==>special functionality added via a pointer to object
    virtual void StartTimeStep(CObject*) {}; //function is called when computation of time step is started
    virtual void EndTimeStep(CObject*) {}; //function is called when computation of time step is finished
    virtual void SimulationFinished(CObject*) {}; //function is called after computation has finished (e.g. in order to free memory, write results, close connections, etc.)
    virtual void SimulationStarted(CObject*) {}; //function is called before computation has been started (e.g. in order to allocate memory, open connections, etc.)
    //virtual Real PostNewtonStep(CObject*) { return 0; }; //moved to object ...
    //virtual void PostprocessingStep(CObject*) {};
};

class CObject
{
protected:
    SpecialObjectFunctionsContainer* SpecialObjectFunctions() { return nullptr; }

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // LOCAL VARIABLES
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//remove this in future: CObject functions should get nodal coordinates (pos+vel+acc) per node
	CSystemData* cSystemData; //needs to be initialized during Object Factory

public:
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // CONSTRUCTOR, DESTRUCTOR, INITIALIZATION
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    CObject() {DefaultInitialization();}

    //! sets object back to default state
    virtual void DefaultInitialization() {cSystemData = 0;};

    //! get an exact clone of *this, must be implemented in all derived classes! Necessary for better handling in ObjectContainer
    virtual CObject* GetClone() const {return new CObject(*this);}

    virtual void Print(std::ostream& os) const {
        os << "CObject(";
        os << ")";
    }

	//! access to CSystemData; only const functions shall be called
	//  - used to provide computational data and nodeNumbers to CNode* information
	//  - use this access rarely, as it introduces deeper coupling of objects into system!
	CSystemData* GetCSystemData() const { return cSystemData; }
	void SetCSystemData(CSystemData* cSystemDataInit) { cSystemData = cSystemDataInit; }

	//! function which is called, if parameters have changed; this should e.g. tell if precomputed values need to be re-computed; is called in MainObject::SetWithDictionary(...)
	virtual void ParametersHaveChanged() {} //default: do nothing

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // SYSTEM FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	//! distinguish between different body types for management in CSystem
	virtual CObjectType GetType() const { CHECKandTHROWstring("ERROR: illegal call to CObject::GetType"); return CObjectType::None; }

	//! Return true, if object will be computed (used to deactivate objects/contacts without computational overhead)
	virtual bool IsActive() const { return true; }

	//! get number of computational coordinates (without data variables)
    virtual Index GetNumberOfCoordinates() const { return 2*GetODE2Size()+ GetODE1Size() + GetAlgebraicEquationsSize(); } //completely depends on nodes
    virtual Index GetODE1Size() const { return 0; } //completely depends on nodes
    virtual Index GetODE2Size() const { return 0; } //completely depends on nodes
    virtual Index GetAlgebraicEquationsSize() const { return 0; } //depends on constaint type; may depend on node (e.g. Euler parameter constraint)
    virtual Index GetDataVariablesSize() const { return 0; } //usually depends on object (e.g. finite element / plasticity)

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// nodes access; provided in final class!
	// for connectors: only used for data coordinates!!

	//! local to global node number transformation
	virtual Index GetNodeNumber(Index localIndex) const
	{
		CHECKandTHROWstring("ERROR: illegal call to CObject::GetNodeNumber");
		return EXUstd::InvalidIndex;
	}
	//! number of nodes
	virtual Index GetNumberOfNodes() const { return 0; }
	//before: {	CHECKandTHROWstring("ERROR: illegal call to CObject::GetNumberOfNodes"); return EXUstd::InvalidIndex;	}

	//access to CNode
	virtual CNode*& GetCNode(Index localIndex);

	virtual CNode* GetCNode(Index localIndex) const;

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Computation FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //! compute right-hand-side (RHS) of second order ordinary differential equations (ODE) to 'ode2rhs'
    virtual void ComputeODE2RHS(Vector& ode2Rhs) const { CHECKandTHROWstring("ERROR: illegal call to CObject::ComputeODE2RHS"); }

    //! compute right-hand-side (RHS) of first order ordinary differential equations (ODE) to 'ode1rhs', which has dimension GetODE1Size(); q are the system coordinates
    //virtual void ComputeODE1RHS(Vector& ode1Rhs, const Vector& q) {}
    //! compute derivative of right-hand-side (RHS) w.r.t q of second order ordinary differential equations (ODE) to 'ode2rhs', which has dimension GetODE1Size() x GetODE1Size(); this is the tangent (stiffness) matrix; q are the system coordinates
    //virtual void ComputeODE1RHS_q(Matrix& ode2Rhs, const Vector& q) {}

    //! compute algebraic equations to 'algebraicEquations', which has dimension GetAlgebraicEquationsSize(); q are the system coordinates
    virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, bool useIndex2 = false) const { CHECKandTHROWstring("ERROR: illegal call to CObject::ComputeAlgebraicEquations"); }

	//! return the available jacobian types (can be combined with 2^i enum flags); default: no jacobians ==> computed numerically
	virtual JacobianType::Type GetAvailableJacobians() const { return JacobianType::None; }

    //! compute derivative of right-hand-side (RHS) w.r.t q of second order ordinary differential equations (ODE) [optional w.r.t. ODE2_t variables as well, if flag ODE2_ODE2_t_function set in GetAvailableJacobians()]; jacobian [and jacobianODE2_t] has dimension GetODE2Size() x GetODE2Size(); this is the local tangent stiffness matrix;
    virtual void ComputeJacobianODE2_ODE2(ResizableMatrix& jacobian, ResizableMatrix& jacobian_ODE2_t) const { CHECKandTHROWstring("ERROR: illegal call to CObject::ComputeODE2RHSJacobian"); }

    //! compute derivative of algebraic equations w.r.t. ODE2 in jacobian [and w.r.t. ODE2_t coordinates in jacobian_t if flag ODE2_t_AE_function is set] [and w.r.t. AE coordinates if flag AE_AE_function is set in GetAvailableJacobians()]; jacobian[_t] has dimension GetAlgebraicEquationsSize() x (GetODE2Size() + GetODE1Size() [+GetAlgebraicEquationsSize()]); q are the system coordinates
    virtual void ComputeJacobianAE(ResizableMatrix& jacobian, ResizableMatrix& jacobian_t, ResizableMatrix& jacobian_AE) const { CHECKandTHROWstring("ERROR: illegal call to CObject::ComputeJacobianAE"); }

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // ACCESS FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! get available access function types for forces and constraints (action of lagrange multipliers)
    virtual AccessFunctionType GetAccessFunctionTypes() const { CHECKandTHROWstring("ERROR: illegal call to CObject::GetAccessFunctionTypes"); return AccessFunctionType::None; }
	//! get available output variable types for constraints and for sensors
	virtual OutputVariableType GetOutputVariableTypes() const { CHECKandTHROWstring("ERROR: illegal call to CObject::GetOutputVariableTypes"); return OutputVariableType::None; }

	//! get access function 'accessType' in (matrix) value
	virtual void GetAccessFunction(AccessFunctionType accessType, Matrix& value) const { CHECKandTHROWstring("ERROR: illegal call to CObject::GetAccessFunction"); }
	//! get output variable 'variableType' in (vector) value
	virtual void GetOutputVariable(OutputVariableType variableType, Vector& value) const { CHECKandTHROWstring("ERROR: illegal call to CObject::GetOutputVariable"); } //configuration not needed in general objects!

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // OTHER FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! do initialization before object-node-list is created
    virtual void PreAssemble() { } 
	//! do initialization after global assembly has been finished and object-node-list is created
    virtual void PostAssemble() { }


};

//! the ostream operator<< is only defined in base class and calls the Print(...) method of the derived class; this calls the correct method also in the ResizableArray<CObject*>
inline std::ostream& operator<<(std::ostream& os, const CObject& object) {
    object.Print(os);
    return os;
}



