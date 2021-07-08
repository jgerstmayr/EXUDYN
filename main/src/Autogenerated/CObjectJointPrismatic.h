/** ***********************************************************************************************
* @class        CObjectJointPrismaticParameters
* @brief        Parameter class for CObjectJointPrismatic
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-07-01  11:19:35 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTJOINTPRISMATICPARAMETERS__H
#define COBJECTJOINTPRISMATICPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectJointPrismaticParameters
class CObjectJointPrismaticParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    Matrix3D rotationMarker0;                     //!< AUTO: local rotation matrix for marker \f$m0\f$; translation and rotation axes for marker \f$m0\f$ are defined in the local body coordinate system and additionally transformed by rotationMarker0
    Matrix3D rotationMarker1;                     //!< AUTO: local rotation matrix for marker \f$m1\f$; translation and rotation axes for marker \f$m1\f$ are defined in the local body coordinate system and additionally transformed by rotationMarker1
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temorarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectJointPrismaticParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        rotationMarker0 = EXUmath::unitMatrix3D;
        rotationMarker1 = EXUmath::unitMatrix3D;
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectJointPrismatic
* @brief        A prismatic joint in 3D; constrains the relative rotation of two rigid body markers and relative motion w.r.t. the joint \f$y\f$ and \f$z\f$ axes, allowing a relative motion along the joint \f$x\f$ axis. An additional local rotation (rotationMarker) can be used to transform the markers' coordinate systems into the joint coordinate system. For easier definition of the joint, use the exudyn.rigidbodyUtilities function AddPrismaticJoint(...) for two rigid bodies (or ground).
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

//! AUTO: CObjectJointPrismatic
class CObjectJointPrismatic: public CObjectConstraint // AUTO: 
{
protected: // AUTO: 
    static constexpr Index nConstraints = 5;
    CObjectJointPrismaticParameters parameters; //! AUTO: contains all parameters for CObjectJointPrismatic

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectJointPrismaticParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectJointPrismaticParameters& GetParameters() const { return parameters; }

    //! AUTO:  default function to return Marker numbers
    virtual const ArrayIndex& GetMarkerNumbers() const override
    {
        return parameters.markerNumbers;
    }

    //! AUTO:  constraints uses Lagrance multiplier formulation
    virtual bool IsPenaltyConnector() const override
    {
        return false;
    }

    //! AUTO:  Computational function: compute algebraic equations and write residual into 'algebraicEquations'; velocityLevel: equation provided at velocity level
    virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false) const override;

    //! AUTO:  compute derivative of algebraic equations w.r.t. ODE2, ODE2_t, ODE1 and AE coordinates in jacobian [flags ODE2_t_AE_function, AE_AE_function, etc. need to be set in GetAvailableJacobians()]; jacobianODE2[_t] has dimension GetAlgebraicEquationsSize() x GetODE2Size() ; q are the system coordinates; markerData provides according marker information to compute jacobians
    virtual void ComputeJacobianAE(ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE, const MarkerDataStructure& markerData, Real t, Index itemIndex) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags; available jacobians is switched depending on velocity level and on activeConnector condition
    virtual JacobianType::Type GetAvailableJacobians() const override;

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
        return 5;
    }

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionOffset(Vector6D& offset, const MainSystemBase& mainSystem, Real t, Index itemIndex) const;

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionOffset_t(Vector6D& offset, const MainSystemBase& mainSystem, Real t, Index itemIndex) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Position +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::DisplacementLocal +
            (Index)OutputVariableType::VelocityLocal +
            (Index)OutputVariableType::Rotation +
            (Index)OutputVariableType::AngularVelocityLocal +
            (Index)OutputVariableType::ForceLocal +
            (Index)OutputVariableType::TorqueLocal );
    }

};



#endif //#ifdef include once...
