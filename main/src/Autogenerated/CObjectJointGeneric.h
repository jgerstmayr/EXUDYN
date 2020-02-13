/** ***********************************************************************************************
* @class        CObjectJointGenericParameters
* @brief        Parameter class for CObjectJointGeneric
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-02-12  16:38:34 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

#include <functional> //! AUTO: needed for std::function

//! AUTO: Parameters for class CObjectJointGenericParameters
class CObjectJointGenericParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    ArrayIndex constrainedAxes;                   //!< AUTO: flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; 0=free, 1=constrained
    Matrix3D rotationMarker0;                     //!< AUTO: local rotation matrix for marker 0; translation and rotation axes for marker0 are defined in the local body coordinate system and additionally transformed by rotationMarker0
    Matrix3D rotationMarker1;                     //!< AUTO: local rotation matrix for marker 1; translation and rotation axes for marker1 are defined in the local body coordinate system and additionally transformed by rotationMarker1
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temorarily) a connector or constraint
    Vector6D forceTorqueUserFunctionParameters;   //!< AUTO: vector of 6 parameters for joint"s forceTorqueUserFunction
    Vector6D offsetUserFunctionParameters;        //!< AUTO: vector of 6 parameters for joint"s offsetUserFunction
    std::function<StdVector6D(Real,StdVector6D)> forceTorqueUserFunction;//!< AUTO: A python function which defines the time-dependent force (indices 0,1,2) and torque (indices 3,4,5) joint coordinates with parameters (t, forceTorqueUserFunctionParameters); the offset represents the current value of the object; it is highly RECOMMENDED to use sufficiently smooth functions, having consistent initial offsets with initial configuration of bodies, zero or compatible initial offset-velocity, and no accelerations; Example for python function: def f(t, forceTorqueUserFunctionParameters): return [forceTorqueUserFunctionParameters[0]*(1 - np.cos(t*10*2*np.pi)), 0,0,0,0,0]
    std::function<StdVector6D(Real,StdVector6D)> offsetUserFunction;//!< AUTO: A python function which defines the time-dependent (fixed) offset of translation (indices 0,1,2) and rotation (indices 3,4,5) joint coordinates with parameters (t, offsetUserFunctionParameters); the offset represents the current value of the object; it is highly RECOMMENDED to use sufficiently smooth functions, having consistent initial offsets with initial configuration of bodies, zero or compatible initial offset-velocity, and no accelerations; Example for python function: def f(t, offsetUserFunctionParameters): return [offsetUserFunctionParameters[0]*(1 - np.cos(t*10*2*np.pi)), 0,0,0,0,0]
    std::function<StdVector6D(Real,StdVector6D)> offsetUserFunction_t;//!< AUTO: time derivative of offsetUserFunction using the same parameters; needed for "velocityLevel=True", or for index2 time integration and for computation of initial accelerations in SecondOrderImplicit integrators
    //! AUTO: default constructor with parameter initialization
    CObjectJointGenericParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        constrainedAxes = ArrayIndex({1,1,1,1,1,1});
        rotationMarker0 = EXUmath::unitMatrix3D;
        rotationMarker1 = EXUmath::unitMatrix3D;
        activeConnector = true;
        forceTorqueUserFunctionParameters = Vector6D({0.,0.,0.,0.,0.,0.});
        offsetUserFunctionParameters = Vector6D({0.,0.,0.,0.,0.,0.});
        forceTorqueUserFunction = 0;
        offsetUserFunction = 0;
        offsetUserFunction_t = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectJointGeneric
* @brief        A generic joint in 3D; constrains components of the absolute position and rotations of two points given by PointMarkers or RigidMarkers; an additional local rotation can be used to define three rotation axes and/or sliding axes
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

//! AUTO: CObjectJointGeneric
class CObjectJointGeneric: public CObjectConstraint // AUTO: 
{
protected: // AUTO: 
    static const Index nConstraints = 6;
    CObjectJointGenericParameters parameters; //! AUTO: contains all parameters for CObjectJointGeneric

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectJointGenericParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectJointGenericParameters& GetParameters() const { return parameters; }

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

    //! AUTO:  constraint also implements velocity level equations
    virtual bool HasVelocityEquations() const override
    {
        return true;
    }

    //! AUTO:  Computational function: compute algebraic equations and write residual into "algebraicEquations"; velocityLevel: equation provided at velocity level
    virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, bool velocityLevel = false) const override;

    //! AUTO:  compute derivative of algebraic equations w.r.t. ODE2 in jacobian [and w.r.t. ODE2_t coordinates in jacobian_t if flag ODE2_t_AE_function is set] [and w.r.t. AE coordinates if flag AE_AE_function is set in GetAvailableJacobians()]; jacobian[_t] has dimension GetAlgebraicEquationsSize() x (GetODE2Size() + GetODE1Size() [+GetAlgebraicEquationsSize()]); q are the system coordinates; markerData provides according marker information to compute jacobians
    virtual void ComputeJacobianAE(ResizableMatrix& jacobian, ResizableMatrix& jacobian_t, ResizableMatrix& jacobian_AE, const MarkerDataStructure& markerData, Real t) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags; available jacobians is switched depending on velocity level and on activeConnector condition
    virtual JacobianType::Type GetAvailableJacobians() const override;

    //! AUTO:  Flags to determine, which output variables are available (displacment, velocity, stress, ...)
    virtual OutputVariableType GetOutputVariableTypes() const override;

    //! AUTO:  provide according output variable in "value"
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
        return 6;
    }

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

};


