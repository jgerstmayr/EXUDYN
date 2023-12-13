/** ***********************************************************************************************
* @class        CObjectANCFThinPlateParameters
* @brief        Parameter class for CObjectANCFThinPlate
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2023-10-17  22:36:49 (last modified)
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
    Real physicsHeight;                           //!< AUTO:  [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives \f$\rho A L\f$; must be positive
    Matrix3D physicsStrainCoefficients;           //!< AUTO:  [SI:N/m] stiffness coefficients related to inplane normal and shear strains, integrated over height of the plate
    Matrix3D physicsCurvatureCoefficients;        //!< AUTO:  [SI:Nm] stiffness coefficients related to curvatures, integrated over height of the plate
    Real strainIsRelativeToReference;             //!< AUTO:  if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration serves as a reference geometry; allows also values between 0. and 1. to perform a transition during static computation
    Index4 nodeNumbers;                           //!< AUTO: 4 NodePointSlope12 node numbers
    Index useReducedOrderIntegration;             //!< AUTO: 0/false: use highest Gauss integration for virtual work of strains, order 5 for bending moments; 1/true: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments
    //! AUTO: default constructor with parameter initialization
    CObjectANCFThinPlateParameters()
    {
        physicsHeight = 0.;
        physicsStrainCoefficients = EXUmath::unitMatrix3D;
        physicsCurvatureCoefficients = EXUmath::unitMatrix3D;
        strainIsRelativeToReference = 0.;
        nodeNumbers = Index4({EXUstd::InvalidIndex, EXUstd::InvalidIndex, EXUstd::InvalidIndex, EXUstd::InvalidIndex});
        useReducedOrderIntegration = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectANCFThinPlate
* @brief        A 3D thin Kirchhoff plate finite element based on the absolute nodal coordinate formulation, using 4 nodes of type NodePointSlope12. The geometry as well as (deformed and distorted) reference configuration is given by the nodes. The localPosition follows unit-coordinates in the range [-1,1] for X, Y and Z coordinates; the thickness of the plate is h; This element is under construction.
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
    static constexpr Index nNodes = 4; //!< number of nodes
    static constexpr Index nnc = 9; //!< number of node coordinates
    static constexpr Index nODE2coordinates = 36; //!< fixed size of coordinates used e.g. for ConstSizeVectors
    mutable bool massMatrixComputed; //!< flag which shows that mass matrix has been computed; will be set to false at time when parameters are set
    mutable ConstSizeMatrix<nODE2coordinates*nODE2coordinates> precomputedMassMatrix; //!< if massMatrixComputed=true, this contains the (constant) mass matrix for faster computation
    CObjectANCFThinPlateParameters parameters; //! AUTO: contains all parameters for CObjectANCFThinPlate

public: // AUTO: 

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

    //! AUTO:  Compute node coordinates in current configuration including reference coordinates
    void ComputeCurrentNodeCoordinates(ConstSizeVector<6>& qNode0, ConstSizeVector<6>& qNode1) const;

    //! AUTO:  Compute node velocity coordinates in current configuration
    void ComputeCurrentNodeVelocities(ConstSizeVector<6>& qNode0, ConstSizeVector<6>& qNode1) const;

    //! AUTO:  Compute object (finite element) coordinates in current configuration including reference coordinates
    void ComputeCurrentObjectCoordinates(ConstSizeVector<nODE2coordinates>& qANCF) const;

    //! AUTO:  Compute object (finite element) velocities in current configuration
    void ComputeCurrentObjectVelocities(ConstSizeVector<nODE2coordinates>& qANCF_t) const;

    //! AUTO:  precompute mass terms if it has not been done yet
    void PreComputeMassTerms() const;

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
