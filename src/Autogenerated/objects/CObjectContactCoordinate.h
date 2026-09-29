/** ***********************************************************************************************
* @class        CObjectContactCoordinateParameters
* @brief        Parameter class for CObjectContactCoordinate
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-09-29  21:13:53 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef COBJECTCONTACTCOORDINATEPARAMETERS__H
#define COBJECTCONTACTCOORDINATEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CObjectContactCoordinateParameters
class CObjectContactCoordinateParameters // AUTO:
{
public: // AUTO:
    ArrayIndex markerNumbers;                     //!< AUTO: markers define contact gap
    Index nodeNumber;                             //!< AUTO: node number of a NodeGenericData with 1 data coordinate, the gap of the last discontinuous iteration (active set strategy), and a second one, the last impact velocity, if impactModel is not 0
    Real contactStiffness;                        //!< AUTO: must be >= 0; contact (penalty) stiffness [SI:N/m]; acts only upon penetration
    Real contactDamping;                          //!< AUTO: must be >= 0; contact damping [SI:N/(m s)]; acts only upon penetration
    Real contactStiffnessExponent;                //!< AUTO: must be > 0; exponent in the contact law [SI:1], as in ObjectContactSphereSphere; 1 is linear
    Real restitutionCoefficient;                  //!< AUTO: must be > 0; coefficient of restitution [SI:1], used by impactModel 1 and 2; must be > 0
    Real minimumImpactVelocity;                   //!< AUTO: must be >= 0; lower bound [SI:m/s] of the impact velocity in the impact models; a larger damping at low impact velocities and in permanent contact
    Index impactModel;                            //!< AUTO: must be >= 0;  impact model, as in ObjectContactSphereSphere: 0) linear damping only; 1) Hunt-Crossley; 2) Gonthier et al. / Carvalho-Martins; contactDamping is added in all of them
    Real offset;                                  //!< AUTO: offset [SI:m] of contact
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectContactCoordinateParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        nodeNumber = EXUstd::InvalidIndex;
        contactStiffness = 0.;
        contactDamping = 0.;
        contactStiffnessExponent = 1.;
        restitutionCoefficient = 1.;
        minimumImpactVelocity = 0.;
        impactModel = 0;
        offset = 0.;
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectContactCoordinate
* @brief        A penalty-based contact condition for one coordinate: a force upon penetration of the gap between the coordinates of two markers, with the contact law of ObjectContactSphereSphere - linear by default, with a stiffness exponent and impact models; the contact state is kept in a data node (active set strategy).
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

//! AUTO: CObjectContactCoordinate
class CObjectContactCoordinate: public CObjectConnector // AUTO:
{
protected: // AUTO:
    CObjectContactCoordinateParameters parameters; //! AUTO: contains all parameters for CObjectContactCoordinate

public: // AUTO:
    static constexpr Index dataIndexImpactVelocity = 1; //!< index in the data node of the last impact velocity (#2750)

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectContactCoordinateParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectContactCoordinateParameters& GetParameters() const { return parameters; }

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

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual void SetNodeNumber(Index localIndex, Index nodeNumber) override
    {
        parameters.nodeNumber=nodeNumber;
    }

    //! AUTO:  number of nodes; needed for every object; can depend on the configuration
    virtual Index GetNumberOfNodes() const override
    {
        return 1;
    }

    //! AUTO:  needed in order to create ltg-lists for data variable of connector
    virtual Index GetDataVariablesSize() const override
    {
        return (parameters.impactModel != 0) ? 2 : 1;
    }

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

    //! AUTO:  compute gap for given MarkerData --> done for different configurations (current, start of step, ...)
    Real ComputeGap(const MarkerDataStructure& markerData) const;

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs, const MarkerDataStructure& markerData, Index objectNumber) const override;

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

    //! AUTO:  true if the connector uses a penalty formulation; false if the constraint uses Lagrange multipliers
    virtual bool IsPenaltyConnector() const override
    {
        return true;
    }

    //! AUTO:  Flags to determine, which output variables are available (displacment, velocity, stress, ...)
    virtual OutputVariableType GetOutputVariableTypes() const override;

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
        return CObjectType::Connector;
    }

};



#endif //#ifdef include once...
