/** ***********************************************************************************************
* @class        CObjectANCFThinPlateParameters
* @brief        Parameter class for CObjectANCFThinPlate
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-04-06  19:55:07 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTANCFTHINPLATEPARAMETERS__H
#define COBJECTANCFTHINPLATEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CObjectANCFThinPlateParameters
class CObjectANCFThinPlateParameters // AUTO: 
{
public: // AUTO: 
    Vector physicsThickness;                      //!< AUTO:  [SI:m] thickness of plate either provided as scalar or as vector (4 values, same order as local element node numbers) values that are linearly interpolated from nodal values; dimensionality must agree between thickness, strainCoefficients and curvatureCoefficients
    Real physicsDensity;                          //!< AUTO:  [SI:kg/m\f$^3\f$] density of the plate, possibly averaged over thickness
    Real physicsMassProportionalDamping;          //!< AUTO: mass-proportional damping coefficient \f$\alpha\f$ [SI:1/s]; adds massmatrix proportional damping forces \f$\fv_d = \alpha \Mm \dot{\qv}\f$
    Matrix3DList physicsStrainCoefficients;       //!< AUTO:  [SI:N/m] stiffness coefficients related to inplane normal and shear strains, integrated over height of the plate; either given as 3D Matrix (numpy array), or a list of 3D matrices at each nodal point, see thickness; dimensionality must agree between thickness, strainCoefficients and curvatureCoefficients
    Matrix3DList physicsCurvatureCoefficients;    //!< AUTO:  [SI:Nm] stiffness coefficients related to curvatures, integrated over height of the plate; either given as 3D Matrix (numpy array), or a list of 3D matrices at each nodal point, see thickness; dimensionality must agree between thickness, strainCoefficients and curvatureCoefficients
    Real strainIsRelativeToReference;             //!< AUTO:  if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration serves as a reference geometry; allows also values between 0. and 1. to perform a transition during static computation
    Vector4D slopesScalingX;                      //!< AUTO: scaling of x-slopes at each element node; flat elements: half of the side length of the element; curved: optimal values such that curved geometry is best approximated; if negative (default) values are used, length is computed from node distances.
    Vector4D slopesScalingY;                      //!< AUTO: scaling of y-slopes at each element node; flat elements: half of the side length of the element; curved: optimal values such that curved geometry is best approximated; if negative (default) values are used, length is computed from node distances.
    Index4 nodeNumbers;                           //!< AUTO: 4 NodePointSlope12 node numbers, with local (xi,eta) coordinates as [(-1,-1),(1,-1),(1,1),(-1,1)]
    Index useReducedOrderIntegration;             //!< AUTO: 0/false: use highest Gauss integration for virtual work of strains
    //! AUTO: default constructor with parameter initialization
    CObjectANCFThinPlateParameters()
    {
        physicsThickness = Vector();
        physicsDensity = 0.;
        physicsMassProportionalDamping = 0.;
        physicsStrainCoefficients = Matrix3DList();
        physicsCurvatureCoefficients = Matrix3DList();
        strainIsRelativeToReference = 1.;
        slopesScalingX = Vector4D({-1.,-1.,-1.,-1.});
        slopesScalingY = Vector4D({-1.,-1.,-1.,-1.});
        nodeNumbers = Index4({EXUstd::InvalidIndex, EXUstd::InvalidIndex, EXUstd::InvalidIndex, EXUstd::InvalidIndex});
        useReducedOrderIntegration = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectANCFThinPlate
* @brief        OBJECT UNDER CONSTRUCTION: A 3D thin Kirchhoff plate finite element based on the absolute nodal coordinate formulation, using 4 nodes of type NodePointSlope12. The geometry as well as (deformed and distorted) reference configuration is given by the nodes. The localPosition follows unit-coordinates in the range [-1,1] for X, Y and Z coordinates; the thickness of the plate is h; This element is under construction.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

//! AUTO: CObjectANCFThinPlate
class CObjectANCFThinPlate: public CObjectBody // AUTO: 
{
protected: // AUTO: 
    static constexpr Index nODE2coordinates = 36; //!< fixed size of coordinates used e.g. for ConstSizeVectors
    mutable bool massMatrixComputed; //!< flag which shows that mass matrix has been computed; will be set to false at time when parameters are set
    mutable ConstSizeMatrix<nODE2coordinates*nODE2coordinates> precomputedMassMatrix; //!< if massMatrixComputed=true, this contains the (constant) mass matrix for faster computation
    CObjectANCFThinPlateParameters parameters; //! AUTO: contains all parameters for CObjectANCFThinPlate

public: // AUTO: 
    static constexpr Index nNodes = 4; //!< number of nodes
    static constexpr Index nSF = 12; //!< number of shape functions
    static constexpr Index nnc = 9; //!< number of node coordinates

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectANCFThinPlateParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectANCFThinPlateParameters& GetParameters() const { return parameters; }

    //! AUTO:  Computational function: compute mass matrix
    virtual void ComputeMassMatrix(EXUmath::MatrixContainer& massMatrixC, const ArrayIndex& ltg, Index objectNumber, bool computeInverse=false) const override;

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs, Index objectNumber) const override;

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    template<class TReal> void ComputeODE2LHStemplate(VectorBase<TReal>& ode2Lhs, const ConstSizeVectorBase<TReal, nODE2coordinates>& qANCFtotal, const ConstSizeVectorBase<TReal, nODE2coordinates>& qANCF_t) const;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t + JacobianType::ODE2_ODE2_function + JacobianType::ODE2_ODE2_t_function);
    }

    //! AUTO:  Flags to determine, which access (forces, moments, connectors, ...) to object are possible
    virtual AccessFunctionType GetAccessFunctionTypes() const override;

    //! AUTO:  provide Jacobian at localPosition in 'value' according to object access
    virtual void GetAccessFunctionBody(AccessFunctionType accessType, const Vector3D& localPosition, Matrix& value) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableBody(OutputVariableType variableType, const Vector3D& localPosition, ConfigurationType configuration, Vector& value, Index objectNumber) const override;

    //! AUTO:  return the (global) position of 'localPosition' according to configuration type
    virtual Vector3D GetPosition(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) displacement of 'localPosition' according to configuration type
    virtual Vector3D GetDisplacement(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) velocity of 'localPosition' according to configuration type
    virtual Vector3D GetVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) acceleration of 'localPosition' according to configuration type
    Vector3D GetAcceleration(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  return configuration dependent angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
    virtual Vector3D GetAngularVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the local position of the center of mass, needed for equations of motion and for massProportionalLoad
    virtual Vector3D GetLocalCenterOfMass() const override
    {
        return Vector3D({0.,0.,0.});
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        CHECKandTHROW(localIndex <= 3, __EXUDYN_invalid_local_node1);
        return parameters.nodeNumbers[localIndex];
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual void SetNodeNumber(Index localIndex, Index nodeNumber) override
    {
        parameters.nodeNumbers[localIndex]=nodeNumber;
    }

    //! AUTO:  number of nodes; needed for every object
    virtual Index GetNumberOfNodes() const override
    {
        return 4;
    }

    //! AUTO:  number of \hac{ODE2} coordinates; needed for object?
    virtual Index GetODE2Size() const override
    {
        return nODE2coordinates;
    }

    //! AUTO:  Get type of object, e.g. to categorize and distinguish during assembly and computation
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded);
    }

    //! AUTO:  return true if object has time and coordinate independent (=constant) mass matrix
    virtual bool HasConstantMassMatrix() const override
    {
        return true;
    }

    //! AUTO:  This function is called upon change of parameters
    virtual void ParametersHaveChanged() override;

    //! AUTO:  map element coordinates (position or veloctiy level) given by nodal vectors q0, ..., q3 onto shape function vector to compute position, etc.
    template<class TReal> SlimVectorBase<TReal, 3> MapCoordinates(const Vector12D& sf, const ConstSizeVectorBase<TReal, nODE2coordinates>& q) const;

    //! AUTO:  compute strains and curvatures relative to reference configuration
    template<class TReal> void ComputeKinematics(Real xi, Real eta, const ConstSizeVectorBase<Real, nODE2coordinates>& qANCFref, const ConstSizeVectorBase<TReal, nODE2coordinates>& qANCFtotal, SlimVectorBase<TReal, 3>& eps, SlimVectorBase<TReal, 3>& kappa) const;

    //! AUTO:  compute element energy integrating over Gauss points
    template<class TReal> TReal ComputeElementEnergy(const ConstSizeVectorBase<Real, nODE2coordinates>& qANCFref, const ConstSizeVectorBase<TReal, nODE2coordinates>& qANCFtotal) const;

    //! AUTO:  scale shape functions accordingly (only reference configuration!)
    void ScaleShapeFunctions(Vector12D& sf) const;

    //! AUTO:  get compressed shape function vector \f$\Sm_v\f$, depending on local position \f$[\xi, \eta] \in [-1,1] \times [-1,1]\f$ (in unit coordinates)
    void ComputeShapeFunctions(Real xi, Real eta, Vector12D& sf, bool scaled=true) const;

    //! AUTO:  get first derivatives of compressed shape function vector \f$\Sm_v\f$, depending on local position \f$[\xi, \eta] \in [-1,1] \times [-1,1]\f$ (in unit coordinates)
    void ComputeShapeFunctions_xy(Real xi, Real eta, Vector12D& sf_x, Vector12D& sf_y, bool scaled=true) const;

    //! AUTO:  get second derivatives of compressed shape function vector \f$\Sm_v\f$, depending on local position \f$[\xi, \eta] \in [-1,1] \times [-1,1]\f$ (in unit coordinates)
    void ComputeShapeFunctions_xxyy(Real xi, Real eta, Vector12D& sf_xx, Vector12D& sf_yy, Vector12D& sf_xy, bool scaled=true) const;

    //! AUTO:  Compute object (finite element) coordinates in reference configuration
    void ComputeReferenceObjectCoordinates(ConstSizeVector<nODE2coordinates>& qANCF) const;

    //! AUTO:  Compute object (finite element) coordinates in current configuration including reference coordinates
    void ComputeCurrentTotalObjectCoordinates(ConstSizeVector<nODE2coordinates>& qANCF) const;

    //! AUTO:  Compute object (finite element) coordinates in given configuration
    void ComputeObjectCoordinates(ConstSizeVector<nODE2coordinates>& qANCF, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  Compute object (finite element) velocities in given configuration
    void ComputeObjectVelocities(ConstSizeVector<nODE2coordinates>& qANCF_t, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  Compute object (finite element) accelerations in given configuration
    void ComputeObjectAccelerations(ConstSizeVector<nODE2coordinates>& qANCF_tt, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  return the (global) normal at 'localPosition' according to configuration type
    Vector3D GetNormal(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  compute the (global) slope vectors at 'localPosition' according to configuration type
    void GetSlopes(const Vector3D& localPosition, Vector3D& slopeX, Vector3D& slopeY, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  compute 2x2 element jacobian matrix; uses inplane orthogonal basis vectors
    template<class TReal> ConstSizeMatrixBase<TReal, 4> GetElementJacobian(const SlimVectorBase<TReal, 3>& r_xi_ref, const SlimVectorBase<TReal, 3>& r_eta_ref, const SlimVectorBase<TReal, 3>& n0) const;

    //! AUTO:  compute element jacobian for transformation of unit element derivatives to global derivatives
    Real GetElementJacobian() const;

    //! AUTO:  precompute mass terms if it has not been done yet
    void PreComputeMassTerms() const;

    //! AUTO:  compute thickness from local unit coordinates
    Real ComputeThicknessAtPoint(Real xi, Real eta) const;

    //! AUTO:  compute strain coefficient matrix from local unit coordinates
    Matrix3D ComputeStrainCoefficientsAtPoint(Real xi, Real eta) const;

    //! AUTO:  compute curvature coefficient matrix from local unit coordinates
    Matrix3D ComputeCurvatureCoefficientsAtPoint(Real xi, Real eta) const;

    //! AUTO:  Computational function: compute jacobian (dense or sparse mode, see parent CObject function)
    virtual void ComputeJacobianODE2_ODE2(EXUmath::MatrixContainer& jacobianODE2, JacobianTemp& temp, Real factorODE2, Real factorODE2_t, Index objectNumber, const ArrayIndex& ltg) const override;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index64)OutputVariableType::Position +
            (Index64)OutputVariableType::Displacement +
            (Index64)OutputVariableType::Velocity +
            (Index64)OutputVariableType::Director1 +
            (Index64)OutputVariableType::Director2 +
            (Index64)OutputVariableType::StrainLocal +
            (Index64)OutputVariableType::CurvatureLocal +
            (Index64)OutputVariableType::ForceLocal +
            (Index64)OutputVariableType::TorqueLocal +
            (Index64)OutputVariableType::StressLocal +
            (Index64)OutputVariableType::Acceleration );
    }

};



#endif //#ifdef include once...
