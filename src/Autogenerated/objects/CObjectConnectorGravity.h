/** ***********************************************************************************************
* @class        CObjectConnectorGravityParameters
* @brief        Parameter class for CObjectConnectorGravity
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-02  08:26:07 (last modified)
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
    Real mass0;                                   //!< AUTO: must be >= 0; mass [SI:kg] of object attached to marker \f$m0\f$
    Real mass1;                                   //!< AUTO: must be >= 0; mass [SI:kg] of object attached to marker \f$m1\f$
    Real minDistanceRegularization;               //!< AUTO: must be >= 0; distance [SI:m] at which a regularization is added in order to avoid singularities, if objects come close
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectConnectorGravityParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        gravitationalConstant = 6.6743e-11;
        mass0 = 0.;
        mass1 = 0.;
        minDistanceRegularization = 0.;
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectConnectorGravity
* @brief        A connector for additing forces due to gravitational fields beween two bodies, which can be used for aerospace and small-scale astronomical problems. NOTE: DO NOT USE this connector for adding gravitational forces (loads), which should be using LoadMassProportional, which is acting global and always in the same direction.
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

    //! AUTO:  the right-hand side on the connector interface of position markers (#2745)
    virtual bool ComputeODE2LHSConnector(const CSystemData& systemData, TemporaryComputationData& temp, Vector& localODE2Lhs, Index objectNumber) const override
    {
        ConnectorODE2LHSPositionMarkers(systemData, temp, *this, localODE2Lhs, objectNumber); return true;
    }

    //! AUTO:  the Jacobian by automatic differentiation of the force (#2745)
    virtual bool ComputeJacobianODE2Connector(const CSystemData& systemData, TemporaryComputationData& temp, Real factorODE2, Real factorODE2_t, Index objectNumber, bool jacobianDerivativeNonZero) const override
    {
        ConnectorJacobianODE2PositionMarkers(systemData, temp, *this, factorODE2, factorODE2_t, objectNumber, jacobianDerivativeNonZero); return true;
    }

    //! AUTO:  the force on marker 1 from the kinematics of the two markers (#2745)
    virtual void ComputeConnectorForcePosition(const MarkerPosition<Real>* markers, Real t, Index itemIndex, Vector3D& force) const override;

    //! AUTO:  the same force with automatic differentiation, for the Jacobian (#2745)
    virtual void ComputeConnectorForcePositionDiff(const MarkerPosition<DRealPositionMarkers>* markers, Real t, Index itemIndex, SlimVectorBase<DRealPositionMarkers, 3>& force) const override;

    //! AUTO:  the physics of the connector, shared by the legacy path, the new one, its Jacobian and the output variables (#2745)
    template<class TReal> void ComputeGravityForce(const SlimVectorBase<TReal, 3>& position0, const SlimVectorBase<TReal, 3>& position1, SlimVectorBase<TReal, 3>& relPos, TReal& force, SlimVectorBase<TReal, 3>& forceDirection) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index64)OutputVariableType::Distance +
            (Index64)OutputVariableType::Displacement +
            (Index64)OutputVariableType::Force +
            (Index64)OutputVariableType::PotentialEnergy );
    }

};



#endif //#ifdef include once...
