/** ***********************************************************************************************
* @class        CObjectJointHelicalZParameters
* @brief        Parameter class for CObjectJointHelicalZ
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-11-11  08:46:20 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef COBJECTJOINTHELICALZPARAMETERS__H
#define COBJECTJOINTHELICALZPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectJointHelicalZParameters
class CObjectJointHelicalZParameters // AUTO:
{
public: // AUTO:
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    Real pitch;                                   //!< AUTO: the axial translation per radian of rotation (the lead of the thread is \f$2\pi h\f$); positive for a right-handed thread, negative for a left-handed one, zero for the revolute joint
    ArrayIndex constrainedAxes;                   //!< AUTO: flags, which determine which equations are active: the translations along \f$x\f$ and \f$y\f$ of J0 (0, 1), the coupling of the axial translation and the rotation (2), the rotations about \f$x\f$ and \f$y\f$ (3, 4); 0=free, 1=constrained
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectJointHelicalZParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        pitch = 0.;
        constrainedAxes = ArrayIndex({1,1,1,1,1});
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectJointHelicalZ
* @brief        A helical (screw) joint in 3D: the frame of marker 1 moves on a screw motion about the \f$z\f$-axis of marker 0, a rotation by the angle \f$\varphi\f$ together with the translation \f$h\varphi\f$ along the axis, with the pitch \f$h\f$ per radian. The joint coordinate system is the frame of the markers; a rotation of it is given to the markers as their localHT. Each of the five constraints can be switched off by `constrainedAxes`; with the pitch zero it is the revolute joint. For easier definition of the joint, use `mbs.CreateHelicalJoint(...)` for two rigid bodies (or ground).
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

//! AUTO: CObjectJointHelicalZ
class CObjectJointHelicalZ: public CObjectConstraint // AUTO:
{
protected: // AUTO:
    static constexpr Index nConstraints = 5;
    CObjectJointHelicalZParameters parameters; //! AUTO: contains all parameters for CObjectJointHelicalZ

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectJointHelicalZParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectJointHelicalZParameters& GetParameters() const { return parameters; }

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
    virtual bool ComputeAlgebraicEquationsConnector(const CSystemData& systemData, TemporaryComputationData& temp, bool velocityLevel, Vector& localAE) const override
    {
        if (!OnConnectorInterface()) { return false; } ConstraintEquationsRigidMarkers(systemData, temp, *this, velocityLevel, localAE); return true;
    }

    //! AUTO:  C_q by automatic differentiation of the equations (#2745)
    virtual bool ComputeJacobianAEConnector(const CSystemData& systemData, TemporaryComputationData& temp, ResizableMatrix& jacobianAE_ODE2) const override
    {
        if (!OnConnectorInterface()) { return false; } ConstraintJacobianRigidMarkers(systemData, temp, *this, jacobianAE_ODE2); return true;
    }

    //! AUTO:  C_q^T lambda per marker, without forming C_q (#2745)
    virtual bool ComputeReactionForcesConnector(const CSystemData& systemData, TemporaryComputationData& temp, const Vector& reactionForces, Vector& localODE2) const override
    {
        if (!OnConnectorInterface()) { return false; } ConstraintReactionForcesRigidMarkers(systemData, temp, *this, reactionForces, localODE2); return true;
    }

    //! AUTO:  the equations of the inactive constraint, lambda = 0; the active one computes on the connector interface (#2745)
    virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, bool velocityLevel = false) const override
    {
        CHECKandTHROW(!IsActive(), "ComputeAlgebraicEquations: an active constraint computes on the connector interface"); algebraicEquations.CopyFrom(markerData.GetLagrangeMultipliers());
    }

    //! AUTO:  the Jacobian of the inactive constraint, d(lambda)/d(lambda); the active one computes on the connector interface (#2745)
    virtual void ComputeJacobianAE(ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE, const MarkerDataStructure& markerData, Real t) const override
    {
        CHECKandTHROW(!IsActive(), "ComputeJacobianAE: an active constraint computes on the connector interface"); jacobian_AE.SetScalarMatrix(GetAlgebraicEquationsSize(), 1.);
    }

    //! AUTO:  the algebraic equations from the kinematics of the two markers (#3421)
    virtual void ComputeConstraintEquationsRigid(FlatArray<const MarkerRigid<Real>> markers, const LinkedDataVector& lambda, Real t, bool velocityLevel, ConstSizeVector<maxConstraintEquations>& equations) const override;

    //! AUTO:  the equations with their analytic derivatives by the variations of the marker frames, for C_q (#3421)
    virtual bool ComputeConstraintEquationsRigidDerivatives(FlatArray<const MarkerRigid<Real>> markers, const LinkedDataVector& lambda, Real t, ConstSizeVector<maxConstraintEquations>& equations, RigidMarkersDerivatives& derivatives) const override;

    //! AUTO:  the derivative of the equations of the free axes by the Lagrange multipliers
    virtual void ComputeJacobianAE_AE(ResizableMatrix& jacobian_AE) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Vector& value) const override;

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
        return nConstraints;
    }

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index64)OutputVariableType::Position +
            (Index64)OutputVariableType::Velocity +
            (Index64)OutputVariableType::DisplacementLocal +
            (Index64)OutputVariableType::VelocityLocal +
            (Index64)OutputVariableType::Rotation +
            (Index64)OutputVariableType::AngularVelocityLocal +
            (Index64)OutputVariableType::ForceLocal +
            (Index64)OutputVariableType::TorqueLocal +
            (Index64)OutputVariableType::HomogeneousTransformation +
            (Index64)OutputVariableType::HomogeneousTransformationLocal +
            (Index64)OutputVariableType::ConstraintEquation );
    }

};


//! AUTO: the member functions that the declarations of CObjectJointHelicalZ require (#2866)
#include <type_traits>
static_assert(!std::is_same<decltype(&CObjectJointHelicalZ::GetOutputVariableConnector), decltype(&CObjectConnector::GetOutputVariableConnector)>::value, "CObjectJointHelicalZ declares output variables and must implement GetOutputVariableConnector; the default of CObjectConnector raises");

#endif //#ifdef include once...
