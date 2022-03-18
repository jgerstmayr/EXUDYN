/** ***********************************************************************************************
* @class        CObjectConnectorGravityParameters
* @brief        Parameter class for CObjectConnectorGravity
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2022-01-30  18:19:34 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTCONNECTORGRAVITYPARAMETERS__H
#define COBJECTCONNECTORGRAVITYPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CObjectConnectorGravityParameters
class CObjectConnectorGravityParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    Real gravitationalConstant;                   //!< AUTO: gravitational constant [SI:m\f$^3\f$kg\f$^{-1}\f$s\f$^{-2}\f$)]; while not recommended, a negative constant gan represent a repulsive force
    Real mass0;                                   //!< AUTO: mass [SI:kg] of object attached to marker \f$m0\f$
    Real mass1;                                   //!< AUTO: mass [SI:kg] of object attached to marker \f$m1\f$
    Real minDistanceRegularization;               //!< AUTO: distance [SI:m] at which a regularization is added in order to avoid singularities, if objects come close
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temorarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectConnectorGravityParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        gravitationalConstant = 6.67430e-11;
        mass0 = 0.;
        mass1 = 0.;
        minDistanceRegularization = 0.;
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectConnectorGravity
* @brief        A connector for additing forces due to gravitational fields beween two bodies, which can be used for aerospace and small-scale astronomical problems; DO NOT USE this connector for adding gravitational forces (loads), which should be using LoadMassProportional, which is acting global and always in the same direction.
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

//! AUTO: CObjectConnectorGravity
class CObjectConnectorGravity: public CObjectConnector // AUTO: 
{
protected: // AUTO: 
    CObjectConnectorGravityParameters parameters; //! AUTO: contains all parameters for CObjectConnectorGravity

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectConnectorGravityParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectConnectorGravityParameters& GetParameters() const { return parameters; }

    //! AUTO:  return true, if object has a computation user function
    virtual bool HasUserFunction() const override
    {
        return false;
    }

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

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override;

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

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

    //! AUTO:  compute connector force and further properties (relative position, etc.) for unique functionality and output
    void ComputeConnectorProperties(const MarkerDataStructure& markerData, Index itemIndex, Vector3D& relPos,Real& force, Vector3D& forceDirection) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Distance +
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Force );
    }

};



#endif //#ifdef include once...
