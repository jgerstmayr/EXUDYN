/** ***********************************************************************************************
* @class        CObjectConnectorCoordinateSpringDamperParameters
* @brief        Parameter class for CObjectConnectorCoordinateSpringDamper
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2019-12-18  22:06:39 (last modfied)
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

//! AUTO: Parameters for class CObjectConnectorCoordinateSpringDamperParameters
class CObjectConnectorCoordinateSpringDamperParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    Real stiffness;                               //!< AUTO: stiffness [SI:N/m] of spring; acts against relative value of coordinates
    Real damping;                                 //!< AUTO: damping [SI:N/(m s)] of damper; acts against relative velocity of coordinates
    Real offset;                                  //!< AUTO: offset between two coordinates (reference length of springs), see equation
    Real dryFriction;                             //!< AUTO: dry friction coefficient against relative velocity
    Real dryFrictionProportionalZone;             //!< AUTO: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations)
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temorarily) a connector or constraint
    std::function<Real(Real,Real,Real,Real,Real,Real,Real)> springForceUserFunction;//!< AUTO: A python function which defines the spring force with parameters (deltaL, deltaL\f$_t\f$, Real stiffness, Real damping, Real offset, Real dryFriction, Real dryFrictionProportionalZone); the parameters are provided to the function using the current values of the SpringDamper object; note that \f$u=(m1.coordinate - m0.coordinate)\f$, not including the offset; The python function will only be evaluated, if activeConnector is true, otherwise the SpringDamper is inactive; Example for python function: def f(u, v, k, d, offset, mu, muProp): return k*u + d*v + F0
    //! AUTO: default constructor with parameter initialization
    CObjectConnectorCoordinateSpringDamperParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        stiffness = 0.;
        damping = 0.;
        offset = 0.;
        dryFriction = 0.;
        dryFrictionProportionalZone = 0.;
        activeConnector = true;
        springForceUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectConnectorCoordinateSpringDamper
* @brief        A 1D (scalar) spring-damper element acting on single ODE2 coordinates; connects to coordinate-based markers; NOTE that the coordinate markers only measure the coordinate (=displacement), but the reference position is not included as compared to position-based markers!; the spring-damper can also act on rotational coordinates; the resulting force in the spring-damper reads (\f$m0 = marker[0]\f$ and \f$m1 = marker[1]\f$): \f[ f = (m1.coordinate - m0.coordinate - offset)\cdot stiffness + (m1.coordinate_t - m0.coordinate_t)\cdot damping \f] If dry (Coulomb) friction is non-zero, an additional term \f[ \text{sign}(m1.coordinate_t - m0.coordinate_t)\cdot dryFriction \f] is added to the force \f$f\f$.
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

//! AUTO: CObjectConnectorCoordinateSpringDamper
class CObjectConnectorCoordinateSpringDamper: public CObjectConnector // AUTO: 
{
protected: // AUTO: 
    CObjectConnectorCoordinateSpringDamperParameters parameters; //! AUTO: contains all parameters for CObjectConnectorCoordinateSpringDamper

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectConnectorCoordinateSpringDamperParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectConnectorCoordinateSpringDamperParameters& GetParameters() const { return parameters; }

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

    //! AUTO:  Computational function: compute right-hand-side (RHS) of second order ordinary differential equations (ODE) to "ode2rhs"
    virtual void ComputeODE2RHS(Vector& ode2Rhs, const MarkerDataStructure& markerData) const override;

    //! AUTO:  Computational function: compute Jacobian of ODE2 RHS equations w.r.t. ODE coordinates (jacobian) and if JacobianType::ODE2_ODE2_t flag is set in GetAvailableJacobians() compute jacobian w.r.t. ODE_t coordinates
    virtual void ComputeJacobianODE2_ODE2(ResizableMatrix& jacobian, ResizableMatrix& jacobian_ODE2_t, const MarkerDataStructure& markerData) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::ODE2_ODE2+JacobianType::ODE2_ODE2_t);
    }

    //! AUTO:  Flags to determine, which output variables are available (displacment, velocity, stress, ...)
    virtual OutputVariableType GetOutputVariableTypes() const override;

    //! AUTO:  provide according output variable in "value"
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Vector& value) const override;

    //! AUTO:  provide requested markerType for connector
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return Marker::Coordinate;
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

};


