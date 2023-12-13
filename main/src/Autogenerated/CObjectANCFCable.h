/** ***********************************************************************************************
* @class        CObjectANCFCableParameters
* @brief        Parameter class for CObjectANCFCable
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2023-10-16  00:07:33 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTANCFCABLEPARAMETERS__H
#define COBJECTANCFCABLEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CObjectANCFCableParameters
class CObjectANCFCableParameters // AUTO: 
{
public: // AUTO: 
    Real physicsLength;                           //!< AUTO:  [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives \f$\rho A L\f$; must be positive
    Real physicsMassPerLength;                    //!< AUTO:  [SI:kg/m] mass per length of beam
    Real physicsBendingStiffness;                 //!< AUTO:  [SI:Nm\f$^2\f$] bending stiffness of beam; the bending moment is \f$m = EI (\kappa - \kappa_0)\f$, in which \f$\kappa\f$ is the material measure of curvature
    Real physicsAxialStiffness;                   //!< AUTO:  [SI:N] axial stiffness of beam; the axial force is \f$f_{ax} = EA (\varepsilon -\varepsilon_0)\f$, in which \f$\varepsilon = |\rv^\prime|-1\f$ is the axial strain
    Real physicsBendingDamping;                   //!< AUTO:  [SI:Nm\f$^2\f$/s] bending damping of beam ; the additional virtual work due to damping is \f$\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx\f$
    Real physicsAxialDamping;                     //!< AUTO:  [SI:N/s] axial damping of beam; the additional virtual work due to damping is \f$\delta W_{\dot\varepsilon} = \int_0^L \dot \varepsilon \delta \varepsilon dx\f$
    Real physicsReferenceAxialStrain;             //!< AUTO:  [SI:1] reference axial strain of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference axial strain value
    Real strainIsRelativeToReference;             //!< AUTO:  if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration plus the values of \f$\varepsilon_0\f$ and \f$\kappa_0\f$ serve as a reference geometry; allows also values between 0. and 1.
    Index2 nodeNumbers;                           //!< AUTO: two node numbers ANCF cable element
    Index useReducedOrderIntegration;             //!< AUTO: 0/false: use Gauss order 9 integration for virtual work of axial forces, order 5 for virtual work of bending moments; 1/true: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments
    //! AUTO: default constructor with parameter initialization
    CObjectANCFCableParameters()
    {
        physicsLength = 0.;
        physicsMassPerLength = 0.;
        physicsBendingStiffness = 0.;
        physicsAxialStiffness = 0.;
        physicsBendingDamping = 0.;
        physicsAxialDamping = 0.;
        physicsReferenceAxialStrain = 0.;
        strainIsRelativeToReference = 0.;
        nodeNumbers = Index2({EXUstd::InvalidIndex, EXUstd::InvalidIndex});
        useReducedOrderIntegration = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectANCFCable
* @brief        A 3D cable finite element using 2 nodes of type NodePointSlope1. The localPosition of the beam with length \f$L\f$=physicsLength and height \f$h\f$ ranges in \f$X\f$-direction in range \f$[0, L]\f$ and in \f$Y\f$-direction in range \f$[-h/2,h/2]\f$ (which is in fact not needed in the \hac{EOM}). For description see ObjectANCFCable2D, which is almost identical to 3D case. Note that this element does not include torsion, therfore a torque cannot be applied along the local x-axis.
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

//! AUTO: CObjectANCFCable
class CObjectANCFCable: public CObjectBody // AUTO: 
{
protected: // AUTO: 
    static constexpr Index nODE2coordinates = 12; //!< fixed size of coordinates used e.g. for ConstSizeVectors
    mutable bool massMatrixComputed; //!< flag which shows that mass matrix has been computed; will be set to false at time when parameters are set
    mutable ConstSizeMatrix<nODE2coordinates*nODE2coordinates> precomputedMassMatrix; //!< if massMatrixComputed=true, this contains the (constant) mass matrix for faster computation
    CObjectANCFCableParameters parameters; //! AUTO: contains all parameters for CObjectANCFCable

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectANCFCableParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectANCFCableParameters& GetParameters() const { return parameters; }

    //! AUTO:  access to individual element paramters for base class functions
    Real GetLength() const
    {
        return parameters.physicsLength;
    }

    //! AUTO:  Computational function: compute mass matrix
    virtual void ComputeMassMatrix(EXUmath::MatrixContainer& massMatrixC, const ArrayIndex& ltg, Index objectNumber, bool computeInverse=false) const override;

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs, Index objectNumber) const override;

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    template<class TReal, Index ancfSize> void ComputeODE2LHStemplate(VectorBase<TReal>& ode2Lhs, const ConstSizeVectorBase<TReal, ancfSize>& qANCF, const ConstSizeVectorBase<TReal, ancfSize>& qANCF_t) const;

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
        return Vector3D({0.5*parameters.physicsLength,0.,0.});
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        CHECKandTHROW(localIndex <= 1, __EXUDYN_invalid_local_node1);
        return parameters.nodeNumbers[localIndex];
    }

    //! AUTO:  number of nodes; needed for every object
    virtual Index GetNumberOfNodes() const override
    {
        return 2;
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

    //! AUTO:  This flag is reset upon change of parameters; says that mass matrix (future: other pre-computed values) need to be recomputed
    virtual void ParametersHaveChanged() override
    {
        massMatrixComputed = false;
    }

    //! AUTO:  map element coordinates (position or veloctiy level) given by nodal vectors q0 and q1 onto compressed shape function vector to compute position, etc.
    static Vector3D MapCoordinates(const Vector4D& SV, const LinkedDataVector& q0, const LinkedDataVector& q1);

    //! AUTO:  get compressed shape function vector \f$\Sm_v\f$, depending local position \f$x \in [0,L]\f$
    static Vector4D ComputeShapeFunctions(Real x, Real L);

    //! AUTO:  get first derivative of compressed shape function vector \f$\frac{\partial \Sm_v}{\partial x}\f$, depending local position \f$x \in [0,L]\f$
    static Vector4D ComputeShapeFunctions_x(Real x, Real L);

    //! AUTO:  get second derivative of compressed shape function vector \f$\frac{\partial^2 \Sm_v}{\partial^2 x}\f$, depending local position \f$x \in [0,L]\f$
    static Vector4D ComputeShapeFunctions_xx(Real x, Real L);

    //! AUTO:  get third derivative of compressed shape function vector \f$\frac{\partial^3 \Sm_v}{\partial^3 x}\f$, depending local position \f$x \in [0,L]\f$
    static Vector4D ComputeShapeFunctions_xxx(Real x, Real L);

    //! AUTO:  Compute node coordinates in current configuration including reference coordinates
    void ComputeCurrentNodeCoordinates(ConstSizeVector<6>& qNode0, ConstSizeVector<6>& qNode1) const;

    //! AUTO:  Compute node velocity coordinates in current configuration
    void ComputeCurrentNodeVelocities(ConstSizeVector<6>& qNode0, ConstSizeVector<6>& qNode1) const;

    //! AUTO:  Compute object (finite element) coordinates in current configuration including reference coordinates
    void ComputeCurrentObjectCoordinates(ConstSizeVector<nODE2coordinates>& qANCF) const;

    //! AUTO:  Compute object (finite element) velocities in current configuration
    void ComputeCurrentObjectVelocities(ConstSizeVector<nODE2coordinates>& qANCF_t) const;

    //! AUTO:  compute the slope vector at a certain position, for given configuration
    Vector3D ComputeSlopeVector(Real x, ConfigurationType configuration) const;

    //! AUTO:  compute the d(slope)/dt vector at a certain position, for given configuration
    Vector3D ComputeSlopeVector_t(Real x, ConfigurationType configuration) const;

    //! AUTO:  compute the d(slope)/dx vector at a certain position, for given configuration
    Vector3D ComputeSlopeVector_x(Real x, ConfigurationType configuration) const;

    //! AUTO:  compute the d(slope)/dxdt vector at a certain position, for given configuration
    Vector3D ComputeSlopeVector_xt(Real x, ConfigurationType configuration) const;

    //! AUTO:  precompute mass terms if it has not been done yet
    void PreComputeMassTerms() const;

    //! AUTO:  Computational function: compute jacobian (dense or sparse mode, see parent CObject function)
    virtual void ComputeJacobianODE2_ODE2(EXUmath::MatrixContainer& jacobianODE2, JacobianTemp& temp, Real factorODE2, Real factorODE2_t, Index objectNumber, const ArrayIndex& ltg) const override;

    //! AUTO:  compute scalar axial strain at a certain position, for given configuration
    Real ComputeAxialStrain(Real x, ConfigurationType configuration) const;

    //! AUTO:  compute scalar time derivative of axial strain at a certain position, for given configuration
    Real ComputeAxialStrain_t(Real x, ConfigurationType configuration) const;

    //! AUTO:  compute vectorial curvature at a certain position, for given configuration
    Vector3D ComputeCurvature(Real x, ConfigurationType configuration) const;

    //! AUTO:  compute time derivative of vectorial curvature at a certain position, for given configuration
    Vector3D ComputeCurvature_t(Real x, ConfigurationType configuration) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Position +
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::Director1 +
            (Index)OutputVariableType::StrainLocal +
            (Index)OutputVariableType::CurvatureLocal +
            (Index)OutputVariableType::ForceLocal +
            (Index)OutputVariableType::TorqueLocal +
            (Index)OutputVariableType::Acceleration );
    }

};



#endif //#ifdef include once...
