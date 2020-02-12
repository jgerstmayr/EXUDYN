/** ***********************************************************************************************
* @class        CObjectJointALEMoving2DParameters
* @brief        Parameter class for CObjectJointALEMoving2D
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-02-10  21:17:19 (last modfied)
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


//! AUTO: Parameters for class CObjectJointALEMoving2DParameters
class CObjectJointALEMoving2DParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: marker0: position-marker of mass point or rigid body; marker1: updated marker to Cable2D element, where the sliding joint currently is attached to; must be initialized with an appropriate (global) marker number according to the starting position of the sliding object; this marker changes with time (PostNewtonStep)
    ArrayIndex slidingMarkerNumbers;              //!< AUTO: these markers are used to update marker1, if the sliding position exceeds the current cable"s range; the markers must be sorted such that marker(i) at x=cable.length is equal to marker(i+1) at x=0
    Vector slidingMarkerOffsets;                  //!< AUTO: this list contains the offsets of every sliding object (given by slidingMarkerNumbers) w.r.t. to the initial position (0): marker0: offset=0, marker1: offset=Length(cable0), marker2: offset=Length(cable0)+Length(cable1), ...
    Real slidingOffset;                           //!< AUTO: offset [SI:m] used set the sliding position relative to the chosen Eulerian (NodeGenericODE2) coordinate; the following relation is used: \f$slidingPosition = posALE + slidingOffset\f$
    ArrayIndex nodeNumbers;                       //!< AUTO: node numbers of: [0] NodeGenericData for 1 dataCoordinate showing the according marker number which is currently active; [1] of the GenericNodeODE2 of the ALE sliding coordinate
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temorarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectJointALEMoving2DParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        slidingMarkerNumbers = ArrayIndex();
        slidingMarkerOffsets = Vector();
        slidingOffset = 0.;
        nodeNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectJointALEMoving2D
* @brief        A specialized axially moving joint (without rotation) in 2D between a ALE Cable2D (marker1) and a position-based marker (marker0); the data coordinate [0] provides the current index in slidingMarkerNumbers, and the ODE2 coordinate [0] provides the (given) moving coordinate in the cable element; the algebraic variables are \f[ \qv_{AE}=[\lambda_x\;\; \lambda_y]^T \f], in which \f$\lambda_x\f$ and \f$\lambda_y\f$ are the Lagrange multipliers for the position constraint of the moving joint; the data coordinate is \f[ \qv_{Data} = [i_{marker}]^T \f] in which \f$i_{marker}\f$ is the current local index to the slidingMarkerNumber list.
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

//! AUTO: CObjectJointALEMoving2D
class CObjectJointALEMoving2D: public CObjectConstraint // AUTO: 
{
protected: // AUTO: 
    CObjectJointALEMoving2DParameters parameters; //! AUTO: contains all parameters for CObjectJointALEMoving2D

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectJointALEMoving2DParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectJointALEMoving2DParameters& GetParameters() const { return parameters; }

    //! AUTO:  default function to return Marker numbers
    virtual const ArrayIndex& GetMarkerNumbers() const override
    {
        return parameters.markerNumbers;
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        release_assert(localIndex <= 1);
        return parameters.nodeNumbers[localIndex];
    }

    //! AUTO:  number of nodes; needed for every object
    virtual Index GetNumberOfNodes() const override
    {
        return 2;
    }

    //! AUTO:  data variables: [0] showing the current (local) index in slidingMarkerNumber list --> providing the cable element active in sliding; coordinate [1] stores the previous sliding coordinate
    virtual Index GetDataVariablesSize() const override
    {
        return 1;
    }

    //! AUTO:  constraints uses Lagrance multiplier formulation
    virtual bool IsPenaltyConnector() const override
    {
        return false;
    }

    //! AUTO:  constraint also implements velocity level equations
    virtual bool HasVelocityEquations() const override
    {
        return false;
    }

    //! AUTO:  Computational function: compute algebraic equations and write residual into "algebraicEquations"; velocityLevel: equation provided at velocity level
    virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, bool velocityLevel = false) const override;

    //! AUTO:  compute derivative of algebraic equations w.r.t. ODE2 in jacobian [and w.r.t. ODE2_t coordinates in jacobian_t if flag ODE2_t_AE_function is set] [and w.r.t. AE coordinates if flag AE_AE_function is set in GetAvailableJacobians()]; jacobian[_t] has dimension GetAlgebraicEquationsSize() x (GetODE2Size() + GetODE1Size() [+GetAlgebraicEquationsSize()]); q are the system coordinates; markerData provides according marker information to compute jacobians
    virtual void ComputeJacobianAE(ResizableMatrix& jacobian, ResizableMatrix& jacobian_t, ResizableMatrix& jacobian_AE, const MarkerDataStructure& markerData, Real t) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::AE_ODE2 + JacobianType::AE_ODE2_function + JacobianType::AE_AE + JacobianType::AE_AE_function);
    }

    //! AUTO:  flag to be set for connectors, which use DiscontinuousIteration
    virtual bool HasDiscontinuousIteration() const override
    {
        return true;
    }

    //! AUTO:  function called after Newton method; returns a residual error (force)
    virtual Real PostNewtonStep(const MarkerDataStructure& markerDataCurrent, PostNewtonFlags::Type& flags) override;

    //! AUTO:  function called after discontinuous iterations have been completed for one step (e.g. to finalize history variables and set initial values for next step)
    virtual void PostDiscontinuousIterationStep() override;

    //! AUTO:  provide according output variable in "value"
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Vector& value) const override;

    //! AUTO:  provide requested markerType for connector; for different markerTypes in marker0/1 => set to ::None
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return Marker::None;
    }

    //! AUTO:  return object type (for node treatment in computation)
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);
    }

    //! AUTO:  q0=forceX of sliding joint, q1=forceY of sliding joint
    virtual Index GetAlgebraicEquationsSize() const override
    {
        return 2;
    }

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

    //! AUTO:  compute the (local) sliding coordinate within the current cable element; this is calculated from (globalSlidingCoordinate - slidingMarkerOffset) of the cable
    Real ComputeLocalSlidingCoordinate() const;

    //! AUTO:  compute the (local=global) sliding velocity, which is equivalent to the ALE velocity!
    Real ComputeLocalSlidingCoordinate_t() const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Position +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::SlidingCoordinate +
            (Index)OutputVariableType::Coordinates +
            (Index)OutputVariableType::Coordinates_t +
            (Index)OutputVariableType::Force );
    }

};


