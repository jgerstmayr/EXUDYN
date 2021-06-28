/** ***********************************************************************************************
* @class        CObjectConnectorCartesianSpringDamperParameters
* @brief        Parameter class for CObjectConnectorCartesianSpringDamper
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-06-27  17:36:10 (last modfied)
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
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectConnectorCartesianSpringDamperParameters
class CObjectConnectorCartesianSpringDamperParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    Vector3D stiffness;                           //!< AUTO: stiffness [SI:N/m] of springs; act against relative displacements in 0, 1, and 2-direction
    Vector3D damping;                             //!< AUTO: damping [SI:N/(m s)] of dampers; act against relative velocities in 0, 1, and 2-direction
    Vector3D offset;                              //!< AUTO: offset between two springs
    std::function<StdVector(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdVector3D)> springForceUserFunction;//!< AUTO: A python function which computes the 3D force vector between the two marker points, if activeConnector=True; see description below
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
#include "System/ItemIndices.h"

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

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs, const MarkerDataStructure& markerData, Index objectNumber) const override;

    //! AUTO:  Computational function: compute Jacobian of ODE2 LHS equations w.r.t. ODE coordinates (jacobian) and if JacobianType::ODE2_ODE2_t flag is set in GetAvailableJacobians() compute jacobian w.r.t. ODE_t coordinates
    virtual void ComputeJacobianODE2_ODE2(ResizableMatrix& jacobian, ResizableMatrix& jacobian_ODE2_t, const MarkerDataStructure& markerData) const override;

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
        return CObjectType::Connector;
    }

    //! AUTO:  compute spring damper force helper function
    void ComputeSpringForce(const MarkerDataStructure& markerData, Index itemIndex, Vector3D& vPos, Vector3D& vVel, Vector3D& fVec) const;

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionForce(Vector3D& force, const MainSystemBase& mainSystem, Real t, Index itemIndex, Vector3D& vPos, Vector3D& vVel) const;

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
