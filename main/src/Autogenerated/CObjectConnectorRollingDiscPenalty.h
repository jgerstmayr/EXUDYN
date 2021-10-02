/** ***********************************************************************************************
* @class        CObjectConnectorRollingDiscPenaltyParameters
* @brief        Parameter class for CObjectConnectorRollingDiscPenalty
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-09-28  19:19:58 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTCONNECTORROLLINGDISCPENALTYPARAMETERS__H
#define COBJECTCONNECTORROLLINGDISCPENALTYPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CObjectConnectorRollingDiscPenaltyParameters
class CObjectConnectorRollingDiscPenaltyParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector; \f$m0\f$ represents the ground, which can undergo translations but not rotations, and \f$m1\f$ represents the rolling body, which has its reference point (=local position [0,0,0]) at the disc center point
    Index nodeNumber;                             //!< AUTO: node number of a NodeGenericData (size=3) for 3 dataCoordinates, needed for discontinuous iteration (friction and contact)
    Real dryFrictionAngle;                        //!< AUTO: angle [SI:1 (rad)] which defines a rotation of the local tangential coordinates dry friction; this allows to model Mecanum wheels with specified roll angle
    Real contactStiffness;                        //!< AUTO: normal contact stiffness [SI:N/m]
    Real contactDamping;                          //!< AUTO: normal contact damping [SI:N/(m s)]
    Vector2D dryFriction;                         //!< AUTO: dry friction coefficients [SI:1] in local marker 1 joint \f$J1\f$ coordinates; if \f$\alpha_t==0\f$, lateral direction \f$l=x\f$ and forward direction \f$f=y\f$; assuming a normal force \f$f_n\f$, the local friction force can be computed as \f$\LU{J1}{\vp{f_{t,x}}{f_{t,y}}} = \vp{\mu_x f_n}{\mu_y f_n}\f$
    Real dryFrictionProportionalZone;             //!< AUTO: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations)
    Real rollingFrictionViscous;                  //!< AUTO: rolling friction [SI:1], which acts against the velocity of the trail on ground and leads to a force proportional to the contact normal force; currently, only implemented for disc axis parallel to ground!
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temorarily) a connector or constraint
    Real discRadius;                              //!< AUTO: defines the disc radius
    Vector3D planeNormal;                         //!< AUTO: normal to the contact / rolling plane (ground); Currently, this is not co-rotating with the ground body, but will do so in the future
    //! AUTO: default constructor with parameter initialization
    CObjectConnectorRollingDiscPenaltyParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        nodeNumber = EXUstd::InvalidIndex;
        dryFrictionAngle = 0.;
        contactStiffness = 0.;
        contactDamping = 0.;
        dryFriction = Vector2D({0,0});
        dryFrictionProportionalZone = 0.;
        rollingFrictionViscous = 0.;
        activeConnector = true;
        discRadius = 0;
        planeNormal = Vector3D({0,0,1});
    };
};


/** ***********************************************************************************************
* @class        CObjectConnectorRollingDiscPenalty
* @brief        A (flexible) connector representing a rolling rigid disc (marker 1) on a flat surface (marker 0, ground body, not moving) in global \f$x\f$-\f$y\f$ plane. The connector is based on a penalty formulation and adds friction and slipping. The contraints works for discs as long as the disc axis and the plane normal vector are not parallel. Parameters may need to be adjusted for better convergence (e.g., dryFrictionProportionalZone). The formulation is still under development and needs further testing. Note that the rolling body must have the reference point at the center of the disc.
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

//! AUTO: CObjectConnectorRollingDiscPenalty
class CObjectConnectorRollingDiscPenalty: public CObjectConnector // AUTO: 
{
protected: // AUTO: 
    static constexpr Index nDataVariables = 3; //number of data variables for tangential and normal contact
    CObjectConnectorRollingDiscPenaltyParameters parameters; //! AUTO: contains all parameters for CObjectConnectorRollingDiscPenalty

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectConnectorRollingDiscPenaltyParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectConnectorRollingDiscPenaltyParameters& GetParameters() const { return parameters; }

    //! AUTO:  default function to return Marker numbers
    virtual const ArrayIndex& GetMarkerNumbers() const override
    {
        return parameters.markerNumbers;
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        release_assert(localIndex == 0);
        return parameters.nodeNumber;
    }

    //! AUTO:  number of nodes; needed for every object
    virtual Index GetNumberOfNodes() const override
    {
        return 1;
    }

    //! AUTO:  data (history) variable simplifies contact switching for implicit time integration and Newton method
    virtual Index GetDataVariablesSize() const override
    {
        return nDataVariables;
    }

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);
    }

    //! AUTO:  flag to be set for connectors, which use DiscontinuousIteration
    virtual bool HasDiscontinuousIteration() const override
    {
        return true;
    }

    //! AUTO:  function called after Newton method; returns a residual error (force)
    virtual Real PostNewtonStep(const MarkerDataStructure& markerDataCurrent, Index itemIndex, PostNewtonFlags::Type& flags, Real& recommendedStepSize) override;

    //! AUTO:  function called after discontinuous iterations have been completed for one step (e.g. to finalize history variables and set initial values for next step)
    virtual void PostDiscontinuousIterationStep() override;

    //! AUTO:  connector uses penalty formulation
    virtual bool IsPenaltyConnector() const override
    {
        return true;
    }

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs, const MarkerDataStructure& markerData, Index objectNumber) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Index itemIndex, Vector& value) const override;

    //! AUTO:  compute contact kinematics and contact forces
    void ComputeContactForces(const MarkerDataStructure& markerData, const CObjectConnectorRollingDiscPenaltyParameters& parameters, bool computeCurrent, Vector3D& pC, Vector3D& vC, Vector3D& wLateral, Vector3D& w2, Vector3D& w3, Vector3D& fContact, Vector2D& localSlipVelocity) const;

    //! AUTO:  compute slip force vector for specific states
    Vector2D ComputeSlipForce(const CObjectConnectorRollingDiscPenaltyParameters& parameters, 	const Vector2D& localSlipVelocity, const Vector2D& dataLocalSlipVelocity, Real contactForce) const;

    //! AUTO:  provide requested markerType for connector
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return (Marker::Type)((Index)Marker::Position + (Index)Marker::Orientation);
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

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Position +
            (Index)OutputVariableType::VelocityLocal +
            (Index)OutputVariableType::ForceLocal );
    }

};



#endif //#ifdef include once...
