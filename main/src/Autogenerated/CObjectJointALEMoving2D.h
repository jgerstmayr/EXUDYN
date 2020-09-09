/** ***********************************************************************************************
* @class        CObjectJointALEMoving2DParameters
* @brief        Parameter class for CObjectJointALEMoving2D
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-09-08  18:14:40 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTJOINTALEMOVING2DPARAMETERS__H
#define COBJECTJOINTALEMOVING2DPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CObjectJointALEMoving2DParameters
class CObjectJointALEMoving2DParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: marker m0: position-marker of mass point or rigid body; marker m1: updated marker to ANCF Cable2D element, where the sliding joint currently is attached to; must be initialized with an appropriate (global) marker number according to the starting position of the sliding object; this marker changes with time (PostNewtonStep)
    ArrayIndex slidingMarkerNumbers;              //!< AUTO: a list of sn (global) marker numbers which are are used to update marker1
    Vector slidingMarkerOffsets;                  //!< AUTO: this list contains the offsets of every sliding object (given by slidingMarkerNumbers) w.r.t. to the initial position (0): marker0: offset=0, marker1: offset=Length(cable0), marker2: offset=Length(cable0)+Length(cable1), ...
    Real slidingOffset;                           //!< AUTO: sliding offset list [SI:m]: a list of sn scalar offsets, which represent the (reference arc) length of all previous sliding cable elements
    ArrayIndex nodeNumbers;                       //!< AUTO: node number of NodeGenericData (GD) with one data coordinate and of NodeGenericODE2 (ALE) with one ODE2 coordinate
    bool usePenaltyFormulation;                   //!< AUTO: flag, which determines, if the connector is formulated with penalty, but still using algebraic equations (IsPenaltyConnector() still false)
    Real penaltyStiffness;                        //!< AUTO: penalty stiffness [SI:N/m] used if usePenaltyFormulation=True
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temorarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectJointALEMoving2DParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        slidingMarkerNumbers = ArrayIndex();
        slidingMarkerOffsets = Vector();
        slidingOffset = 0.;
        nodeNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        usePenaltyFormulation = false;
        penaltyStiffness = 0.;
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectJointALEMoving2D
* @brief        A specialized axially moving joint (without rotation) in 2D between a ALE Cable2D (marker1) and a position-based marker (marker0); ALE=Arbitrary Lagrangian Eulerian; the data coordinate x[0] provides the current index in slidingMarkerNumbers, and the ODE2 coordinate q[0] provides the (given) moving coordinate in the cable element.
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

    //! AUTO:  Computational function: compute algebraic equations and write residual into 'algebraicEquations'; velocityLevel: equation provided at velocity level
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

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Vector& value) const override;

    //! AUTO:  provide requested markerType for connector; for different markerTypes in marker0/1 => set to ::\_None
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return Marker::_None;
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



#endif //#ifdef include once...
