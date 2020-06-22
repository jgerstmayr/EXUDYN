/** ***********************************************************************************************
* @class        CObjectConnectorCoordinateParameters
* @brief        Parameter class for CObjectConnectorCoordinate
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-06-19  16:27:41 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

#include <functional> //! AUTO: needed for std::function

//! AUTO: Parameters for class CObjectConnectorCoordinateParameters
class CObjectConnectorCoordinateParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    Real offset;                                  //!< AUTO: An offset between the two values
    Real factorValue1;                            //!< AUTO: An additional factor multiplied with value1 used in algebraic equation
    bool velocityLevel;                           //!< AUTO: If true: connector constrains velocities (only works for ODE2 coordinates!); offset is used between velocities; in this case, the offsetUserFunction\_t is considered and offsetUserFunction is ignored
    std::function<Real(Real,Real)> offsetUserFunction;//!< AUTO: A python function which defines the time-dependent offset; it is highly RECOMMENDED to use sufficiently smooth functions, having consistent initial offsets with initial configuration of bodies, zero or compatible initial offset-velocity, and no accelerations; Example for python function: def UF(t, l\_offset): return l\_offset*(1-np.cos(t*10*2*np.pi))
    std::function<Real(Real,Real)> offsetUserFunction_t;//!< AUTO: time derivative of offsetUserFunction; needed for 'velocityLevel=True', or for index2 time integration and for computation of initial accelerations in SecondOrderImplicit integrators
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temorarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectConnectorCoordinateParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        offset = 0.;
        factorValue1 = 1.;
        velocityLevel = false;
        offsetUserFunction = 0;
        offsetUserFunction_t = 0;
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectConnectorCoordinate
* @brief        A coordinate constraint which constrains two (scalar) coordinates of Marker[Node|Body]Coordinates attached to nodes or bodies. The constraint acts directly on coordinates, but does not include reference values, e.g., of nodal values.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

//! AUTO: CObjectConnectorCoordinate
class CObjectConnectorCoordinate: public CObjectConstraint // AUTO: 
{
protected: // AUTO: 
    CObjectConnectorCoordinateParameters parameters; //! AUTO: contains all parameters for CObjectConnectorCoordinate

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectConnectorCoordinateParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectConnectorCoordinateParameters& GetParameters() const { return parameters; }

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

    //! AUTO:  connector is time dependent if user functions are defined
    virtual bool IsTimeDependent() const override
    {
        return (parameters.offsetUserFunction != 0 || parameters.offsetUserFunction_t != 0);
    }

    //! AUTO:  Return true, if constraint currently is formulated at velocity level (e.g. coordinate constraint ==> this information is needed for correct jacobian computation)
    virtual bool UsesVelocityLevel() const override
    {
        return parameters.velocityLevel;
    }

    //! AUTO:  Computational function: compute algebraic equations and write residual into 'algebraicEquations'; velocityLevel: equation provided at velocity level
    virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, bool velocityLevel = false) const override;

    //! AUTO:  compute derivative of algebraic equations w.r.t. ODE2 in jacobian [and w.r.t. ODE2_t coordinates in jacobian_t if flag ODE2_t_AE_function is set] [and w.r.t. AE coordinates if flag AE_AE_function is set in GetAvailableJacobians()]; jacobian[_t] has dimension GetAlgebraicEquationsSize() x (GetODE2Size() + GetODE1Size() [+GetAlgebraicEquationsSize()]); q are the system coordinates; markerData provides according marker information to compute jacobians
    virtual void ComputeJacobianAE(ResizableMatrix& jacobian, ResizableMatrix& jacobian_t, ResizableMatrix& jacobian_AE, const MarkerDataStructure& markerData, Real t) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags; available jacobians is switched depending on velocity level and on activeConnector condition
    virtual JacobianType::Type GetAvailableJacobians() const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Vector& value) const override;

    //! AUTO:  provide requested markerType for connector
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return Marker::Coordinate;
    }

    //! AUTO:  return object type (for node treatment in computation)
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);
    }

    //! AUTO:  number of algebraic equations; independent of node/body coordinates
    virtual Index GetAlgebraicEquationsSize() const override
    {
        return 1;
    }

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::ConstraintEquation +
            (Index)OutputVariableType::Force );
    }

};


