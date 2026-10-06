/** ***********************************************************************************************
* @class        CObjectJointPrismatic2DParameters
* @brief        Parameter class for CObjectJointPrismatic2D
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-06  07:03:31 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef COBJECTJOINTPRISMATIC2DPARAMETERS__H
#define COBJECTJOINTPRISMATIC2DPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CObjectJointPrismatic2DParameters
class CObjectJointPrismatic2DParameters // AUTO:
{
public: // AUTO:
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    Vector3D axisMarker0;                         //!< AUTO: direction of prismatic axis, given as a 3D vector in Marker0 frame
    Vector3D normalMarker1;                       //!< AUTO: direction of normal to prismatic axis, given as a 3D vector in Marker1 frame
    bool constrainRotation;                       //!< AUTO: flag, which determines, if the connector also constrains the relative rotation of the two objects; if set to false, the constraint will keep an algebraic equation set equal zero
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectJointPrismatic2DParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        axisMarker0 = Vector3D({1.,0.,0.});
        normalMarker1 = Vector3D({0.,1.,0.});
        constrainRotation = true;
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectJointPrismatic2D
* @brief        A prismatic joint in 2D; allows the relative motion of two bodies, using two RigidMarkers.
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

//! AUTO: CObjectJointPrismatic2D
class CObjectJointPrismatic2D: public CObjectConstraint // AUTO:
{
protected: // AUTO:
    CObjectJointPrismatic2DParameters parameters; //! AUTO: contains all parameters for CObjectJointPrismatic2D

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectJointPrismatic2DParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectJointPrismatic2DParameters& GetParameters() const { return parameters; }

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

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override;

    //! AUTO:  the equations on the connector interface of rigid markers (#2745)
    virtual bool ComputeAlgebraicEquationsConnector(const CSystemData& systemData, TemporaryComputationData& temp, Index objectNumber, bool velocityLevel, Vector& localAE) const override
    {
        if (!OnConnectorInterface()) { return false; } ConstraintEquationsRigidMarkers(systemData, temp, *this, objectNumber, velocityLevel, localAE); return true;
    }

    //! AUTO:  C_q by automatic differentiation of the equations (#2745)
    virtual bool ComputeJacobianAEConnector(const CSystemData& systemData, TemporaryComputationData& temp, Index objectNumber, ResizableMatrix& jacobianAE_ODE2) const override
    {
        if (!OnConnectorInterface()) { return false; } ConstraintJacobianRigidMarkers(systemData, temp, *this, objectNumber, jacobianAE_ODE2); return true;
    }

    //! AUTO:  C_q^T lambda per marker, without forming C_q (#2745)
    virtual bool ComputeReactionForcesConnector(const CSystemData& systemData, TemporaryComputationData& temp, Index objectNumber, const Vector& reactionForces, Vector& localODE2) const override
    {
        if (!OnConnectorInterface()) { return false; } ConstraintReactionForcesRigidMarkers(systemData, temp, *this, objectNumber, reactionForces, localODE2); return true;
    }

    //! AUTO:  the equations of the inactive constraint, lambda = 0; the active one computes on the connector interface (#2745)
    virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false) const override
    {
        CHECKandTHROW(!IsActive(), "ComputeAlgebraicEquations: an active constraint computes on the connector interface"); algebraicEquations.CopyFrom(markerData.GetLagrangeMultipliers());
    }

    //! AUTO:  the Jacobian of the inactive constraint, d(lambda)/d(lambda); the active one computes on the connector interface (#2745)
    virtual void ComputeJacobianAE(ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE, const MarkerDataStructure& markerData, Real t, Index itemIndex) const override
    {
        CHECKandTHROW(!IsActive(), "ComputeJacobianAE: an active constraint computes on the connector interface"); jacobian_AE.SetScalarMatrix(GetAlgebraicEquationsSize(), 1.);
    }

    //! AUTO:  the algebraic equations from the kinematics of the two markers (#2745)
    virtual void ComputeConstraintEquationsRigid(const MarkerRigid<Real>* markers, const LinkedDataVector& lambda, Real t, Index itemIndex, bool velocityLevel, ConstSizeVector<maxConstraintEquations>& equations) const override;

    //! AUTO:  the same equations with automatic differentiation, for C_q (#2745)
    virtual void ComputeConstraintEquationsRigidDiff(const MarkerRigid<DRealRigidMarkers>* markers, const LinkedDataVector& lambda, Real t, Index itemIndex, ConstSizeVectorBase<DRealRigidMarkers, maxConstraintEquations>& equations) const override;

    //! AUTO:  the equations of the joint, for Real and AutoDiff (#2745)
    template<class TReal> void ComputeConstraintEquationsTemplate(const MarkerRigid<TReal>* markers, const LinkedDataVector& lambda, bool velocityLevel, ConstSizeVectorBase<TReal, maxConstraintEquations>& equations) const;

    //! AUTO:  the derivative of the equation of a free rotation by its Lagrange multiplier (#2745)
    virtual void ComputeJacobianAE_AE(ResizableMatrix& jacobian_AE) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Index itemIndex, Vector& value) const override;

    //! AUTO:  provide requested markerType for connector
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return (Marker::Type)((Index)Marker::Position + (Index)Marker::Orientation);
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

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index64)OutputVariableType::Distance +
            (Index64)OutputVariableType::Rotation );
    }

};



#endif //#ifdef include once...
