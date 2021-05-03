/** ***********************************************************************************************
* @class        CObjectANCFCable2DParameters
* @brief        Parameter class for CObjectANCFCable2D
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-03-25  16:05:11 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTANCFCABLE2DPARAMETERS__H
#define COBJECTANCFCABLE2DPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include "Objects/CObjectANCFCable2DBase.h"

//! AUTO: Parameters for class CObjectANCFCable2DParameters
class CObjectANCFCable2DParameters // AUTO: 
{
public: // AUTO: 
    Real physicsLength;                           //!< AUTO:  [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives \f$\rho A L\f$; must be positive
    Real physicsMassPerLength;                    //!< AUTO:  [SI:kg/m] mass per length of beam
    Real physicsBendingStiffness;                 //!< AUTO:  [SI:Nm\f$^2\f$] bending stiffness of beam; the bending moment is \f$m = EI (\kappa - \kappa_0)\f$, in which \f$\kappa\f$ is the material measure of curvature
    Real physicsAxialStiffness;                   //!< AUTO:  [SI:N] axial stiffness of beam; the axial force is \f$f_{ax} = EA (\varepsilon -\varepsilon_0)\f$, in which \f$\varepsilon = |\rv^\prime|-1\f$ is the axial strain
    Real physicsBendingDamping;                   //!< AUTO:  [SI:Nm\f$^2\f$/s] bending damping of beam ; the additional virtual work due to damping is \f$\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx\f$
    Real physicsAxialDamping;                     //!< AUTO:  [SI:N/s] axial stiffness of beam; the additional virtual work due to damping is \f$\delta W_{\dot\varepsilon} = \int_0^L \dot \varepsilon \delta \varepsilon dx\f$
    Real physicsReferenceAxialStrain;             //!< AUTO:  [SI:1] reference axial strain of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference axial strain value
    Real physicsReferenceCurvature;               //!< AUTO:  [SI:1/m] reference curvature of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference curvature value
    Index2 nodeNumbers;                           //!< AUTO: two node numbers ANCF cable element
    bool useReducedOrderIntegration;              //!< AUTO: false: use Gauss order 9 integration for virtual work of axial forces, order 5 for virtual work of bending moments; true: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments
    //! AUTO: default constructor with parameter initialization
    CObjectANCFCable2DParameters()
    {
        physicsLength = 0.;
        physicsMassPerLength = 0.;
        physicsBendingStiffness = 0.;
        physicsAxialStiffness = 0.;
        physicsBendingDamping = 0.;
        physicsAxialDamping = 0.;
        physicsReferenceAxialStrain = 0.;
        physicsReferenceCurvature = 0.;
        nodeNumbers = Index2({EXUstd::InvalidIndex, EXUstd::InvalidIndex});
        useReducedOrderIntegration = false;
    };
};


/** ***********************************************************************************************
* @class        CObjectANCFCable2D
* @brief        A 2D cable finite element using 2 nodes of type NodePoint2DSlope1. The beam with length \f$L\f$=physicsLength uses a localPosition\f$\in [0, L]\f$.
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

//! AUTO: CObjectANCFCable2D
class CObjectANCFCable2D: public CObjectANCFCable2DBase // AUTO: 
{
protected: // AUTO: 
    CObjectANCFCable2DParameters parameters; //! AUTO: contains all parameters for CObjectANCFCable2D

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectANCFCable2DParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectANCFCable2DParameters& GetParameters() const { return parameters; }

    //! AUTO:  access to individual element paramters for base class functions
    virtual Real GetLength() const override
    {
        return parameters.physicsLength;
    }

    //! AUTO:  access to individual element paramters for base class functions
    virtual Real GetMassPerLength() const override
    {
        return parameters.physicsMassPerLength;
    }

    //! AUTO:  access to individual element paramters for base class functions
    virtual void GetMaterialParameters(Real& physicsBendingStiffness, Real& physicsAxialStiffness, Real& physicsBendingDamping, Real& physicsAxialDamping, Real& physicsReferenceAxialStrain, Real& physicsReferenceCurvature) const override
    {
        physicsBendingStiffness = parameters.physicsBendingStiffness; physicsAxialStiffness = parameters.physicsAxialStiffness; physicsBendingDamping = parameters.physicsBendingDamping; physicsAxialDamping = parameters.physicsAxialDamping; physicsReferenceAxialStrain = parameters.physicsReferenceAxialStrain; physicsReferenceCurvature = parameters.physicsReferenceCurvature;
    }

    //! AUTO:  access to useReducedOrderIntegration from derived class
    virtual bool UseReducedOrderIntegration() const override
    {
        return parameters.useReducedOrderIntegration;
    }

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t + JacobianType::ODE2_ODE2_function + JacobianType::ODE2_ODE2_t_function);
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        release_assert(localIndex <= 1);
        return parameters.nodeNumbers[localIndex];
    }

    //! AUTO:  number of nodes; needed for every object
    virtual Index GetNumberOfNodes() const override
    {
        return 2;
    }

    //! AUTO:  number of ODE2 coordinates; needed for object?
    virtual Index GetODE2Size() const override
    {
        return nODE2Coordinates;
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
            (Index)OutputVariableType::TorqueLocal );
    }

};



#endif //#ifdef include once...
