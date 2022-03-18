/** ***********************************************************************************************
* @class        CObjectConnectorCoordinateParameters
* @brief        Parameter class for CObjectConnectorCoordinate
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2022-03-01  20:14:20 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTCONNECTORCOORDINATEPARAMETERS__H
#define COBJECTCONNECTORCOORDINATEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectConnectorCoordinateParameters
class CObjectConnectorCoordinateParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    Real offset;                                  //!< AUTO: An offset between the two values
    Real factorValue1;                            //!< AUTO: An additional factor multiplied with value1 used in algebraic equation
    bool velocityLevel;                           //!< AUTO: If true: connector constrains velocities (only works for \hac{ODE2} coordinates!); offset is used between velocities; in this case, the offsetUserFunction\_t is considered and offsetUserFunction is ignored
    std::function<Real(const MainSystem&,Real,Index,Real)> offsetUserFunction;//!< AUTO: A Python function which defines the time-dependent offset; see description below
    std::function<Real(const MainSystem&,Real,Index,Real)> offsetUserFunction_t;//!< AUTO: time derivative of offsetUserFunction; needed for velocity level constraints; see description below
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
* @brief        A coordinate constraint which constrains two (scalar) coordinates of Marker[Node|Body]Coordinates attached to nodes or bodies. The constraint acts directly on coordinates, but does not include reference values, e.g., of nodal values. This constraint is computationally efficient and should be used to constrain nodal coordinates.
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

    //! AUTO:  return true, if object has a computation user function
    virtual bool HasUserFunction() const override
    {
        return (parameters.offsetUserFunction!=0) || (parameters.offsetUserFunction_t!=0);
    }

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
    virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false) const override;

    //! AUTO:  compute derivative of algebraic equations w.r.t. \hac{ODE2}, \hac{ODE2t}, \hac{ODE1} and \hac{AE} coordinates in jacobian [flags ODE2_t_AE_function, AE_AE_function, etc. need to be set in GetAvailableJacobians()]; jacobianODE2[_t] has dimension GetAlgebraicEquationsSize() x GetODE2Size() ; q are the system coordinates; markerData provides according marker information to compute jacobians
    virtual void ComputeJacobianAE(ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE, const MarkerDataStructure& markerData, Real t, Index itemIndex) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags; available jacobians is switched depending on velocity level and on activeConnector condition
    virtual JacobianType::Type GetAvailableJacobians() const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Index itemIndex, Vector& value) const override;

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

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionOffset(Real& offset, const MainSystemBase& mainSystem, Real t, Index itemIndex) const;

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionOffset_t(Real& offset, const MainSystemBase& mainSystem, Real t, Index itemIndex) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::ConstraintEquation +
            (Index)OutputVariableType::Force );
    }

};



#endif //#ifdef include once...
