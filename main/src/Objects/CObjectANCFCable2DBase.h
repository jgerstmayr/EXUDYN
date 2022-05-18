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
	static const Index nODE2coordinates = 8; //!< fixed size of coordinates used e.g. for ConstSizeVectors
	mutable bool massMatrixComputed; //!< flag which shows that mass matrix has been computed; will be set to false at time when parameters are set
	mutable ConstSizeMatrix<nODE2coordinates*nODE2coordinates> precomputedMassMatrix; //!< if massMatrixComputed=true, this contains the (constant) mass matrix for faster computation

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
	virtual Index UseReducedOrderIntegration() const { return 0; }

	//! access to parameters.strainIsRelativeToReference of derived class
	virtual Real StrainIsRelativeToReference() const { return 0.; }

	//!  Computational function: compute mass matrix
	virtual void ComputeMassMatrix(EXUmath::MatrixContainer& massMatrixC, const ArrayIndex& ltg, Index objectNumber) const override;

	//!  precompute mass terms if it has not been done yet
	virtual void PreComputeMassTerms() const;


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//TEMPLATED FUNCTIONS
	template<class TReal>
	void ComputeODE2LHStemplate(VectorBase<TReal>& ode2Lhs, const ConstSizeVectorBase<TReal, nODE2coordinates>& qANCF, const ConstSizeVectorBase<TReal, nODE2coordinates>& qANCF_t) const;

	//!  map element coordinates (position or veloctiy level) given by nodal vectors q0 and q1 onto compressed shape function vector to compute position, etc.
	template<class TReal>
	SlimVectorBase<TReal, 2> MapCoordinates(const Vector4D& SV, const ConstSizeVectorBase<TReal, nODE2coordinates>& qANCF) const
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
    virtual void ComputeODE2LHS(Vector& ode2Lhs, Index objectNumber) const override;

    ////!  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    //virtual JacobianType::Type GetAvailableJacobians() const override
    //{
    //    return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t + JacobianType::ODE2_ODE2_function + JacobianType::ODE2_ODE2_t_function);
    //}

	//OLD: compute derivative of left-hand-side (LHS) w.r.t q of second order ordinary differential equations (ODE) [optional w.r.t. ODE2_t variables as well, if flag ODE2_ODE2_t_function set in GetAvailableJacobians()]; jacobian [and jacobianODE2_t] has dimension GetODE2Size() x GetODE2Size(); this is the local tangent stiffness matrix;
	//virtual void ComputeJacobianODE2_ODE2(ResizableMatrix& jacobian, ResizableMatrix& jacobian_ODE2_t) const;
	
	//! compute derivative of left-hand-side (LHS) w.r.t q of second order ordinary differential equations (ODE) 
	//! combined computation w.r.t. ODE2 and ODE2\_t variables jacobian has dimension GetODE2Size() x GetODE2Size(); this is the local tangent stiffness matrix;
	virtual void ComputeJacobianODE2_ODE2(EXUmath::MatrixContainer& jacobianODE2, JacobianTemp& temp, Real factorODE2, Real factorODE2_t,
		Index objectNumber, const ArrayIndex& ltg) const;

    //!  Flags to determine, which access (forces, moments, connectors, ...) to object are possible
    virtual AccessFunctionType GetAccessFunctionTypes() const override;

    //!  Flags to determine, which output variables are available (displacment, velocity, stress, ...)
    virtual OutputVariableType GetOutputVariableTypes() const override;

    //!  provide Jacobian at localPosition in "value" according to object access
    virtual void GetAccessFunctionBody(AccessFunctionType accessType, const Vector3D& localPosition, Matrix& value) const override;

    //!  provide according output variable in "value"
    virtual void GetOutputVariableBody(OutputVariableType variableType, const Vector3D& localPosition, ConfigurationType configuration, Vector& value, Index objectNumber) const override;

    //!  return the (global) position of "localPosition" according to configuration type
    virtual Vector3D GetPosition(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //!  return the (global) position of "localPosition" according to configuration type
    virtual Vector3D GetDisplacement(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

	//!  return the (global) velocity of "localPosition" according to configuration type
	virtual Vector3D GetVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

	//!  return the (global) acceleration of "localPosition" according to configuration type
	virtual Vector3D GetAcceleration(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const;

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

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//new functions needed for contact:

	//! compute polynomial coefficients for 3rd order ANCF polynomial; store in Vector cx and cy
	//! the position vector is then defined as: r(x) = Vector2D({ cx[0] + cx[1] * x + cx[2] * x^2 + cx[3] * x^3, cy[0] + cy[1] * x + cy[2] * x^2 + cy[3] * x^3 })
	template<class TCoordinateVector>
	static void ComputePolynomialCoeffs(const TCoordinateVector& q, Real L, ConstSizeVector<4>& cx, ConstSizeVector<4>& cy)
	{
		Real divL = 1 / L;
		Real divL2 = EXUstd::Square(divL);
		Real divL3 = divL * divL2;

		cx[0] = q[0];
		cx[1] = q[2];
		cx[2] = -((3 * q[0] - 3 * q[4] + 2 * L* q[2] + L * q[6]) * divL2);
		cx[3] = (2 * q[0] - 2 * q[4] + L * (q[2] + q[6])) * divL3;

		cy[0] = q[1];
		cy[1] = q[3];
		cy[2] = -((3 * q[1] - 3 * q[5] + 2 * L* q[3] + L * q[7]) * divL2);
		cy[3] = (2 * q[1] - 2 * q[5] + L * (q[3] + q[7])) * divL3;
	}

	//! compute derivative of polynomial coefficients for 3rd order ANCF polynomials; store in Vectors cx and cy
	template<class TCoordinateVector>
	static void ComputePolynomialCoeffs_x(const TCoordinateVector& q, Real L,
		ConstSizeVector<3>& cx, ConstSizeVector<3>& cy)
	{
		Real divL = 1 / L;
		Real divL2 = EXUstd::Square(divL);
		Real divL3 = divL * divL2;

		cx[0] = q[2];
		cy[0] = q[3];

		cx[1] = -((2 * (3 * q[0] - 3 * q[4] + L * (2 * q[2] + q[6]))) * divL2);
		cy[1] = -((2 * (3 * q[1] - 3 * q[5] + L * (2 * q[3] + q[7]))) * divL2);

		cx[2] = (6 * q[0] - 6 * q[4] + 3 * L* (q[2] + q[6])) * divL3;
		cy[2] = (6 * q[1] - 6 * q[5] + 3 * L* (q[3] + q[7])) * divL3;

	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//!  return configuration dependent angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
	virtual Vector3D GetAngularAcceleration(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const;

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
