/** ***********************************************************************************************
* @class        CObjectANCFCable2DBase
* @brief        Basic class for ANCFCable2D elements
*
* @author       Gerstmayr Johannes
* @date         2019-07-22 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#ifndef COBJECTANCFCABLE2DBASE__H
#define COBJECTANCFCABLE2DBASE__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

class CObjectANCFCable2DBase: public CObjectBody
{
protected:
	static const Index nODE2Coordinates = 8; //!< fixed size of coordinates used e.g. for ConstSizeVectors
	mutable bool massMatrixComputed; //!< flag which shows that mass matrix has been computed; will be set to false at time when parameters are set
	mutable ConstSizeMatrix<nODE2Coordinates*nODE2Coordinates> precomputedMassMatrix; //!< if massMatrixComputed=true, this contains the (constant) mass matrix for faster computation

public:

	//! access function to parameters; must be overwritten
	virtual Real GetLength() const { CHECKandTHROWcond(false); return 0; }

	//! access function to parameters; must be overwritten
	virtual Real GetMassPerLength() const { CHECKandTHROWcond(false); return 0; }

	//! access function to parameters; must be overwritten
	virtual void GetMaterialParameters(Real& physicsBendingStiffness, Real& physicsAxialStiffness, Real& physicsBendingDamping, Real& physicsAxialDamping,
		Real& physicsReferenceAxialStrain, Real& physicsReferenceCurvature) const {
		CHECKandTHROWcond(false); }

	//! access to parameters.useReducedOrderIntegration of derived class
	virtual bool UseReducedOrderIntegration() const { return false; }

	//!  Computational function: compute mass matrix
    virtual void ComputeMassMatrix(Matrix& massMatrix) const override;


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//TEMPLATED FUNCTIONS
	template<class TReal>
	void ComputeODE2LHStemplate(VectorBase<TReal>& ode2Lhs, const ConstSizeVectorBase<TReal, nODE2Coordinates>& qANCF, const ConstSizeVectorBase<TReal, nODE2Coordinates>& qANCF_t) const;

	//!  map element coordinates (position or veloctiy level) given by nodal vectors q0 and q1 onto compressed shape function vector to compute position, etc.
	template<class TReal>
	SlimVectorBase<TReal, 2> MapCoordinates(const Vector4D& SV, const ConstSizeVectorBase<TReal, nODE2Coordinates>& qANCF) const
	{
		SlimVectorBase<TReal, 2> v;
		v[0] = 0;
		v[1] = 0;
		for (Index i = 0; i < 4; i++)
		{
			v[0] += SV[i] * qANCF[2 * i];
			v[1] += SV[i] * qANCF[2 * i + 1];
		}
		return v;
	}
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	//!  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to "ode2Lhs"
    virtual void ComputeODE2LHS(Vector& ode2Lhs) const override;

    ////!  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    //virtual JacobianType::Type GetAvailableJacobians() const override
    //{
    //    return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t + JacobianType::ODE2_ODE2_function + JacobianType::ODE2_ODE2_t_function);
    //}

	//! compute derivative of left-hand-side (LHS) w.r.t q of second order ordinary differential equations (ODE) [optional w.r.t. ODE2_t variables as well, if flag ODE2_ODE2_t_function set in GetAvailableJacobians()]; jacobian [and jacobianODE2_t] has dimension GetODE2Size() x GetODE2Size(); this is the local tangent stiffness matrix;
	virtual void ComputeJacobianODE2_ODE2(ResizableMatrix& jacobian, ResizableMatrix& jacobian_ODE2_t) const;

    //!  Flags to determine, which access (forces, moments, connectors, ...) to object are possible
    virtual AccessFunctionType GetAccessFunctionTypes() const override;

    //!  Flags to determine, which output variables are available (displacment, velocity, stress, ...)
    virtual OutputVariableType GetOutputVariableTypes() const override;

    //!  provide Jacobian at localPosition in "value" according to object access
    virtual void GetAccessFunctionBody(AccessFunctionType accessType, const Vector3D& localPosition, Matrix& value) const override;

    //!  provide according output variable in "value"
    virtual void GetOutputVariableBody(OutputVariableType variableType, const Vector3D& localPosition, ConfigurationType configuration, Vector& value) const override;

    //!  return the (global) position of "localPosition" according to configuration type
    virtual Vector3D GetPosition(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //!  return the (global) position of "localPosition" according to configuration type
    virtual Vector3D GetDisplacement(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //!  return the (global) velocity of "localPosition" according to configuration type
    virtual Vector3D GetVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //!  return configuration dependent rotation matrix of node; returns always a 3D Matrix, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
    virtual Matrix3D GetRotationMatrix(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

	//!  return configuration dependent angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
	virtual Vector3D GetAngularVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

	//!  return configuration dependent angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
	virtual Vector3D GetAngularVelocityLocal(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override
	{ return GetAngularVelocity(localPosition, configuration); }

	virtual Vector3D GetLocalCenterOfMass() const { return Vector3D({ 0., 0., 0. }); }

	//!  Get type of object, e.g. to categorize and distinguish during assembly and computation
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded);
    }

    //!  map element coordinates (position or veloctiy level) given by nodal vectors q0 and q1 onto compressed shape function vector to compute position, etc.
    static Vector2D MapCoordinates(const Vector4D& SV, const LinkedDataVector& q0, const LinkedDataVector& q1);

	
	//!  get compressed shape function vector \f$\Sm_v\f$, depending local position \f$x \in [0,L]\f$
    static Vector4D ComputeShapeFunctions(Real x, Real L);

    //!  get first derivative of compressed shape function vector \f$\frac{\partial \Sm_v}{\partial x}\f$, depending local position \f$x \in [0,L]\f$
    static Vector4D ComputeShapeFunctions_x(Real x, Real L);

    //!  get second derivative of compressed shape function vector \f$\frac{\partial^2 \Sm_v}{\partial^2 x}\f$, depending local position \f$x \in [0,L]\f$
    static Vector4D ComputeShapeFunctions_xx(Real x, Real L);

    //!  Compute node coordinates in current configuration including reference coordinates
    void ComputeCurrentNodeCoordinates(ConstSizeVector<4>& qNode0, ConstSizeVector<4>& qNode1) const;

    //!  Compute node velocity coordinates in current configuration
    void ComputeCurrentNodeVelocities(ConstSizeVector<4>& qNode0, ConstSizeVector<4>& qNode1) const;

    //!  Compute object (finite element) coordinates in current configuration including reference coordinates
    void ComputeCurrentObjectCoordinates(ConstSizeVector<8>& qANCF) const;

    //!  Compute object (finite element) velocities in current configuration
    void ComputeCurrentObjectVelocities(ConstSizeVector<8>& qANCF_t) const;

	//!  compute the slope vector at a certain position, for given configuration
	Vector2D ComputeSlopeVector(Real x, ConfigurationType configuration) const;

	//!  compute the derivative of slope vector w.r.t. x (== r_xx) at a certain position, for given configuration
	Vector2D ComputeSlopeVector_x(Real x, ConfigurationType configuration) const;

	//!  compute the axial strain at a certain axial position, for given configuration
	Real ComputeAxialStrain(Real x, ConfigurationType configuration) const;

	//!  compute the (bending) curvature at a certain axial position, for given configuration
	Real ComputeCurvature(Real x, ConfigurationType configuration) const;

	//!  compute time derivative of the slope vector at a certain position, for given configuration
	Vector2D ComputeSlopeVector_t(Real x, ConfigurationType configuration) const;

	//!  compute time derivative of the x-derivative of slope vector w.r.t. x (== r_xx) at a certain position, for given configuration
	Vector2D ComputeSlopeVector_xt(Real x, ConfigurationType configuration) const;

	//!  compute time derivative of the axial strain at a certain axial position, for given configuration
	Real ComputeAxialStrain_t(Real x, ConfigurationType configuration) const;

	//!  compute time derivative of the (bending) curvature at a certain axial position, for given configuration
	Real ComputeCurvature_t(Real x, ConfigurationType configuration) const;

};


#endif
