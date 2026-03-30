/** ***********************************************************************************************
* @class        CObjectContactCircleCircleParameters
* @brief        Parameter class for CObjectContactCircleCircle
*
* @author       Custom Implementation
* @date         2019-07-01 (generated)
* @date         2025-10-22  09:47:43 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTCONTACTCIRCLECIRCLEPARAMETERS__H
#define COBJECTCONTACTCIRCLECIRCLEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CObjectContactCircleCircleParameters
class CObjectContactCircleCircleParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers representing the two circle centers
    Index nodeNumber;                             //!< AUTO: node number of a NodeGenericData with 6 dataCoordinates, storing gap history and friction information
    Real radius1;                                 //!< AUTO: radius of circle attached to marker \f$m0\f$ [SI:m]; negative values denote concave contact
    Real radius2;                                 //!< AUTO: radius of circle attached to marker \f$m1\f$ [SI:m]; negative values denote concave contact
    Real contactStiffness;                        //!< AUTO: normal contact stiffness [SI:N/m]
    Real contactDamping;                          //!< AUTO: linear normal contact damping [SI:N/(m s)]
    Real frictionCoefficient;                     //!< AUTO: Coulomb friction coefficient μ [-]
    Real frictionVelocityPenalty;                 //!< AUTO: velocity-dependent friction penalty [SI:N/(m/s)]
    Real frictionProportionalZone;                //!< AUTO: regularization zone for friction [SI:m/s]
    Real frictionStiffness;                       //!< AUTO: sticking friction stiffness for Bristle model [SI:N/m]; 0=disabled
    bool activeConnector;                         //!< AUTO: flag to (temporarily) deactivate the connector
    //! AUTO: default constructor with parameter initialization
    CObjectContactCircleCircleParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        nodeNumber = EXUstd::InvalidIndex;
        radius1 = 1.;
        radius2 = 1.;
        contactStiffness = 0.;
        contactDamping = 0.;
        frictionCoefficient = 0.;
        frictionVelocityPenalty = 0.;
        frictionProportionalZone = 0.001;
        frictionStiffness = 0.;
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectContactCircleCircle
* @brief        A penalty-based 2D connector enforcing contact between two circles defined by position markers. The second circle may be concave by providing a negative radius, enabling contact with grooves or raceways.
*
* @author       Custom Implementation
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

//! AUTO: CObjectContactCircleCircle
class CObjectContactCircleCircle: public CObjectConnector // AUTO: 
{
protected: // AUTO: 
    static constexpr Index nDataVariables = 8; //gap, gap velocity, normal force, friction x/y, contact state, stick pos x/y
    static constexpr Index dataIndexGap = 0; //!< index in data node representing gap
    static constexpr Index dataIndexGapVelocity = 1; //!< index in data node representing gap rate
    static constexpr Index dataIndexContactForce = 2; //!< index in data node representing normal contact force magnitude
    static constexpr Index dataIndexFrictionForceX = 3; //!< index in data node representing tangential friction force x-component
    static constexpr Index dataIndexFrictionForceY = 4; //!< index in data node representing tangential friction force y-component
    static constexpr Index dataIndexContactState = 5; //!< index in data node representing binary contact state
    static constexpr Index dataIndexStickPosX = 6; //!< index for Bristle model stick position X
    static constexpr Index dataIndexStickPosY = 7; //!< index for Bristle model stick position Y
    mutable Real gap = 0.;
    mutable Real gapVelocity = 0.;
    mutable Real contactForce = 0.;
    mutable Vector3D contactNormal = Vector3D({1.,0.,0.});
    mutable Vector3D contactPoint = Vector3D({0.,0.,0.});
    mutable Vector3D frictionForce = Vector3D({0.,0.,0.});
    mutable Real tangentialVelocity = 0.;         //!< relative tangential velocity [SI:m/s]
    CObjectContactCircleCircleParameters parameters; //! AUTO: contains all parameters for CObjectContactCircleCircle

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectContactCircleCircleParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectContactCircleCircleParameters& GetParameters() const { return parameters; }

    //! AUTO:  default (read) function to return marker numbers
    virtual const ArrayIndex& GetMarkerNumbers() const override
    {
        return parameters.markerNumbers;
    }

    //! AUTO:  default (write) function to return marker numbers
    virtual ArrayIndex& GetMarkerNumbers() override
    {
        return parameters.markerNumbers;
    }

    //! AUTO:  number of markers required for this connector
    virtual Index RequestedNumberOfMarkers() const override
    {
        return 2;
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;
    }

    //! AUTO:  Set global node number (with local node index)
    virtual void SetNodeNumber(Index localIndex, Index nodeNumber) override
    {
        parameters.nodeNumber=nodeNumber;
    }

    //! AUTO:  number of nodes; needed for every object
    virtual Index GetNumberOfNodes() const override
    {
        return 1;
    }

    //! AUTO:  size of associated data node
    virtual Index GetDataVariablesSize() const override
    {
        return nDataVariables;
    }

    //! AUTO:  connector uses discontinuous iteration
    virtual bool HasDiscontinuousIteration() const override
    {
        return true;
    }

    //! AUTO:  function called after Newton method; returns a residual error (force)
    virtual Real PostNewtonStep(const MarkerDataStructure& markerDataCurrent, Index itemIndex, PostNewtonFlags::Type& flags, Real& recommendedStepSize) override;

    //! AUTO:  function called after discontinuous iterations have been completed for one step
    virtual void PostDiscontinuousIterationStep() override;

    //! AUTO:  connector uses penalty formulation
    virtual bool IsPenaltyConnector() const override
    {
        return true;
    }

    //! AUTO:  compute left-hand-side (LHS) of ODE2 equations
    virtual void ComputeODE2LHS(Vector& ode2Lhs, const MarkerDataStructure& markerData, Index objectNumber) const override;

    //! AUTO:  return available Jacobians
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);
    }

    //! AUTO:  provide output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Index itemIndex, Vector& value) const override;

    //! AUTO:  compute gap, forces, and store helper quantities
    void ComputeContactProperties(const MarkerDataStructure& markerData, Index itemIndex, LinkedDataVector& data, bool useDataStates) const;

    //! AUTO:  requested marker type for both markers
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return Marker::Position;
    }

    //! AUTO:  return object type
    virtual CObjectType GetType() const override
    {
        return CObjectType::Connector;
    }

    //! AUTO:  return if connector is active
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::DisplacementLocal +
            (Index)OutputVariableType::VelocityLocal +
            (Index)OutputVariableType::ForceLocal );
    }

};



#endif //#ifdef include once...
