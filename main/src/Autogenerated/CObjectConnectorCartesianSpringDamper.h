/** ***********************************************************************************************
* @class        CObjectConnectorCartesianSpringDamperParameters
* @brief        Parameter class for CObjectConnectorCartesianSpringDamper
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-07-20  12:33:23 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTCONNECTORCARTESIANSPRINGDAMPERPARAMETERS__H
#define COBJECTCONNECTORCARTESIANSPRINGDAMPERPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

#include <functional> //! AUTO: needed for std::function

//! AUTO: Parameters for class CObjectConnectorCartesianSpringDamperParameters
class CObjectConnectorCartesianSpringDamperParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    Vector3D stiffness;                           //!< AUTO: stiffness [SI:N/m] of springs; act against relative displacements in 0, 1, and 2-direction
    Vector3D damping;                             //!< AUTO: damping [SI:N/(m s)] of dampers; act against relative velocities in 0, 1, and 2-direction
    Vector3D offset;                              //!< AUTO: offset between two springs
    std::function<StdVector(Real, StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdVector3D)> springForceUserFunction;//!< AUTO: A python function which computes the 3D force vector between the two marker points, if activeConnector=True;  The function takes the relative displacement (3D) vector (m1.position-m0.position, etc.) and the relative velocity vector (3D), the spring striffness vector 3D, damping and offset parameter vectors (3D): f(time, displacement, velocity, stiffness, damping, offset); Example for python function: def f(t, u, v, k, d, offset): return [u[0]*k[0],u[1]*k[1],u[2]*k[2]]
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temorarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectConnectorCartesianSpringDamperParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        stiffness = Vector3D({0.,0.,0.});
        damping = Vector3D({0.,0.,0.});
        offset = Vector3D({0.,0.,0.});
        springForceUserFunction = 0;
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectConnectorCartesianSpringDamper
* @brief        An 3D spring-damper element acting accordingly in three (global) directions (x,y,z) which connects to position-based markers.
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

//! AUTO: CObjectConnectorCartesianSpringDamper
class CObjectConnectorCartesianSpringDamper: public CObjectConnector // AUTO: 
{
protected: // AUTO: 
    CObjectConnectorCartesianSpringDamperParameters parameters; //! AUTO: contains all parameters for CObjectConnectorCartesianSpringDamper

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectConnectorCartesianSpringDamperParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectConnectorCartesianSpringDamperParameters& GetParameters() const { return parameters; }

    //! AUTO:  default function to return Marker numbers
    virtual const ArrayIndex& GetMarkerNumbers() const override
    {
        return parameters.markerNumbers;
    }

    //! AUTO:  connector uses penalty formulation
    virtual bool IsPenaltyConnector() const override
    {
        return true;
    }

    //! AUTO:  Computational function: compute right-hand-side (RHS) of second order ordinary differential equations (ODE) to 'ode2rhs'
    virtual void ComputeODE2RHS(Vector& ode2Rhs, const MarkerDataStructure& markerData) const override;

    //! AUTO:  Computational function: compute Jacobian of ODE2 RHS equations w.r.t. ODE coordinates (jacobian) and if JacobianType::ODE2_ODE2_t flag is set in GetAvailableJacobians() compute jacobian w.r.t. ODE_t coordinates
    virtual void ComputeJacobianODE2_ODE2(ResizableMatrix& jacobian, ResizableMatrix& jacobian_ODE2_t, const MarkerDataStructure& markerData) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Vector& value) const override;

    //! AUTO:  provide requested markerType for connector
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return Marker::Position;
    }

    //! AUTO:  return object type (for node treatment in computation)
    virtual CObjectType GetType() const override
    {
        return CObjectType::Connector;
    }

    //! AUTO:  compute spring damper force helper function
    void ComputeSpringForce(const MarkerDataStructure& markerData, const CObjectConnectorCartesianSpringDamperParameters& parameters, Vector3D& vPos, Vector3D& vVel, Vector3D& fVec) const;

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Distance +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::Force );
    }

};



#endif //#ifdef include once...
