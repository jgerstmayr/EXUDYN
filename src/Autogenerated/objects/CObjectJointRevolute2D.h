/** ***********************************************************************************************
* @class        CObjectJointRevolute2DParameters
* @brief        Parameter class for CObjectJointRevolute2D
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-02  08:26:07 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef COBJECTJOINTREVOLUTE2DPARAMETERS__H
#define COBJECTJOINTREVOLUTE2DPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CObjectJointRevolute2DParameters
class CObjectJointRevolute2DParameters // AUTO:
{
public: // AUTO:
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectJointRevolute2DParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectJointRevolute2D
* @brief        A revolute joint in 2D; constrains the absolute 2D position of two points given by PointMarkers or RigidMarkers
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

//! AUTO: CObjectJointRevolute2D
class CObjectJointRevolute2D: public CObjectConstraint // AUTO:
{
protected: // AUTO:
    CObjectJointRevolute2DParameters parameters; //! AUTO: contains all parameters for CObjectJointRevolute2D

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectJointRevolute2DParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectJointRevolute2DParameters& GetParameters() const { return parameters; }

    //! AUTO:  default (read) function to return Marker numbers
    virtual const ArrayIndex& GetMarkerNumbers() const override
    {
        return parameters.markerNumbers;
    }

    //! AUTO:  default (write) function to return Marker numbers
    virtual ArrayIndex& GetMarkerNumbers() override
    {
        return parameters.markerNumbers;
    }

    //! AUTO:  true if the connector uses a penalty formulation; false if the constraint uses Lagrange multipliers
    virtual bool IsPenaltyConnector() const override
    {
        return false;
    }

    //! AUTO:  Computational function: compute algebraic equations and write residual into 'algebraicEquations'; velocityLevel: equation provided at velocity level
    virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false) const override;

    //! AUTO:  compute derivative of algebraic equations w.r.t. ABRV:ODE2, ABRV:ODE2 time derivatives, ABRV:ODE1 and ABRV:AE coordinates in jacobian [flags ODE2_t_AE_function, AE_AE_function, etc. need to be set in GetAvailableJacobians()]; jacobianODE2[_t] has dimension GetAlgebraicEquationsSize() x GetODE2Size() ; q are the system coordinates; markerData provides according marker information to compute jacobians
    virtual void ComputeJacobianAE(ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE, const MarkerDataStructure& markerData, Real t, Index itemIndex) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override;

    //! AUTO:  the equations on the connector interface of position markers (#2745)
    virtual bool ComputeAlgebraicEquationsConnector(const CSystemData& systemData, TemporaryComputationData& temp, Index objectNumber, bool velocityLevel, Vector& localAE) const override
    {
        if (!OnConnectorInterface()) { return false; } ConstraintEquationsPositionMarkers(systemData, temp, *this, objectNumber, velocityLevel, localAE); return true;
    }

    //! AUTO:  C_q by automatic differentiation of the equations (#2745)
    virtual bool ComputeJacobianAEConnector(const CSystemData& systemData, TemporaryComputationData& temp, Index objectNumber, ResizableMatrix& jacobianAE_ODE2) const override
    {
        if (!OnConnectorInterface()) { return false; } ConstraintJacobianPositionMarkers(systemData, temp, *this, objectNumber, jacobianAE_ODE2); return true;
    }

    //! AUTO:  C_q^T lambda per marker, without forming C_q (#2745)
    virtual bool ComputeReactionForcesConnector(const CSystemData& systemData, TemporaryComputationData& temp, Index objectNumber, const Vector& reactionForces, Vector& localODE2) const override
    {
        if (!OnConnectorInterface()) { return false; } ConstraintReactionForcesPositionMarkers(systemData, temp, *this, objectNumber, reactionForces, localODE2); return true;
    }

    //! AUTO:  the algebraic equations from the kinematics of the two markers (#2745)
    virtual void ComputeConstraintEquationsPosition(const MarkerPosition<Real>* markers, const LinkedDataVector& lambda, Real t, Index itemIndex, bool velocityLevel, ConstSizeVector<maxConstraintEquations>& equations) const override;

    //! AUTO:  the same equations with automatic differentiation, for C_q (#2745)
    virtual void ComputeConstraintEquationsPositionDiff(const MarkerPosition<DRealPositionMarkers>* markers, const LinkedDataVector& lambda, Real t, Index itemIndex, ConstSizeVectorBase<DRealPositionMarkers, maxConstraintEquations>& equations) const override;

    //! AUTO:  the equations of the constraint, for Real and AutoDiff (#2745)
    template<class TReal> void ComputeConstraintEquationsTemplate(const MarkerPosition<TReal>* markers, const LinkedDataVector& lambda, bool velocityLevel, ConstSizeVectorBase<TReal, maxConstraintEquations>& equations) const;

    //! AUTO:  Flags to determine, which output variables are available (displacment, velocity, stress, ...)
    virtual OutputVariableType GetOutputVariableTypes() const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Index itemIndex, Vector& value) const override;

    //! AUTO:  provide requested markerType for connector
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return Marker::Position;
    }

    //! AUTO:  return object type (for node treatment in computation)
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);
    }

    //! AUTO:  number of algebraic equations; independent of node/body coordinates
    virtual Index GetAlgebraicEquationsSize() const override
    {
        return 2;
    }

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

};



#endif //#ifdef include once...
