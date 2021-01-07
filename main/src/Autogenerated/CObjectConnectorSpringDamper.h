/** ***********************************************************************************************
* @class        CObjectConnectorSpringDamperParameters
* @brief        Parameter class for CObjectConnectorSpringDamper
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-01-05  12:27:48 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTCONNECTORSPRINGDAMPERPARAMETERS__H
#define COBJECTCONNECTORSPRINGDAMPERPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectConnectorSpringDamperParameters
class CObjectConnectorSpringDamperParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    Real referenceLength;                         //!< AUTO: reference length [SI:m] of spring
    Real stiffness;                               //!< AUTO: stiffness [SI:N/m] of spring; acts against (length-initialLength)
    Real damping;                                 //!< AUTO: damping [SI:N/(m s)] of damper; acts against d/dt(length)
    Real force;                                   //!< AUTO: added constant force [SI:N] of spring; scalar force; f=1 is equivalent to reducing initialLength by 1/stiffness; f > 0: tension; f < 0: compression; can be used to model actuator force
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temorarily) a connector or constraint
    std::function<Real(const MainSystem&,Real,Real,Real,Real,Real,Real)> springForceUserFunction;//!< AUTO: A python function which defines the spring force with parameters; the python function will only be evaluated, if activeConnector is true, otherwise the SpringDamper is inactive; see description below
    //! AUTO: default constructor with parameter initialization
    CObjectConnectorSpringDamperParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        referenceLength = 0.;
        stiffness = 0.;
        damping = 0.;
        force = 0.;
        activeConnector = true;
        springForceUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectConnectorSpringDamper
* @brief        An simple spring-damper element with additional force; connects to position-based markers.
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

//! AUTO: CObjectConnectorSpringDamper
class CObjectConnectorSpringDamper: public CObjectConnector // AUTO: 
{
protected: // AUTO: 
    CObjectConnectorSpringDamperParameters parameters; //! AUTO: contains all parameters for CObjectConnectorSpringDamper

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectConnectorSpringDamperParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectConnectorSpringDamperParameters& GetParameters() const { return parameters; }

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
    virtual void ComputeODE2LHS(Vector& ode2Lhs, const MarkerDataStructure& markerData) const override;

    //! AUTO:  Computational function: compute Jacobian of ODE2 LHS equations w.r.t. ODE coordinates (jacobian) and if JacobianType::ODE2_ODE2_t flag is set in GetAvailableJacobians() compute jacobian w.r.t. ODE_t coordinates
    virtual void ComputeJacobianODE2_ODE2(ResizableMatrix& jacobian, ResizableMatrix& jacobian_ODE2_t, const MarkerDataStructure& markerData) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::_None);
    }

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

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

    //! AUTO:  compute connector force and further properties (relative position, etc.) for unique functionality and output
    void ComputeConnectorProperties(const MarkerDataStructure& markerData, Vector3D& relPos, Vector3D& relVel, Real& force, Vector3D& forceDirection) const;

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionForce(Real& force, const MainSystemBase& mainSystem, Real t, Real deltaL, Real deltaL_t) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Distance +
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::Force );
    }

};



#endif //#ifdef include once...
