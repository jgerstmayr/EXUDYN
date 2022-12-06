/** ***********************************************************************************************
* @class        CObjectContactCircleCable2DParameters
* @brief        Parameter class for CObjectContactCircleCable2D
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2022-12-01  20:24:37 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTCONTACTCIRCLECABLE2DPARAMETERS__H
#define COBJECTCONTACTCIRCLECABLE2DPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

constexpr Index CObjectContactCircleCable2DmaxNumberOfSegments = 12; //maximum number of contact segments

//! AUTO: Parameters for class CObjectContactCircleCable2DParameters
class CObjectContactCircleCable2DParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: markers define contact gap
    Index nodeNumber;                             //!< AUTO: node number of a NodeGenericData for nSegments dataCoordinates (used for active set strategy ==> hold the gap of the last discontinuous iteration and the friction state)
    Index numberOfContactSegments;                //!< AUTO: number of linear contact segments to determine contact; each segment is a line and is associated to a data (history) variable; must be same as in according marker
    Real contactStiffness;                        //!< AUTO: contact (penalty) stiffness [SI:N/m/(contact segment)]; the stiffness is per contact segment; specific contact forces (per length) \f$f_N\f$ act in contact normal direction only upon penetration
    Real contactDamping;                          //!< AUTO: contact damping [SI:N/(m s)/(contact segment)]; the damping is per contact segment; acts in contact normal direction only upon penetration
    Real circleRadius;                            //!< AUTO: radius [SI:m] of contact circle
    Real offset;                                  //!< AUTO: offset [SI:m] of contact, e.g. to include thickness of cable element
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectContactCircleCable2DParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        nodeNumber = EXUstd::InvalidIndex;
        numberOfContactSegments = 3;
        contactStiffness = 0.;
        contactDamping = 0.;
        circleRadius = 0.;
        offset = 0.;
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectContactCircleCable2D
* @brief        A very specialized penalty-based contact condition between a 2D circle (=marker0, any Position-marker) on a body and an ANCFCable2DShape (=marker1, Marker: BodyCable2DShape), in xy-plane; a node NodeGenericData is required with the number of cordinates according to the number of contact segments; the contact gap \f$g\f$ is integrated (piecewise linear) along the cable and circle; the contact force \f$f_c\f$ is zero for \f$gap>0\f$ and otherwise computed from \f$f_c = g*contactStiffness + \dot g*contactDamping\f$; during Newton iterations, the contact force is actived only, if \f$dataCoordinate[0] <= 0\f$; dataCoordinate is set equal to gap in nonlinear iterations, but not modified in Newton iterations.
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

//! AUTO: CObjectContactCircleCable2D
class CObjectContactCircleCable2D: public CObjectConnector // AUTO: 
{
protected: // AUTO: 
    CObjectContactCircleCable2DParameters parameters; //! AUTO: contains all parameters for CObjectContactCircleCable2D

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectContactCircleCable2DParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectContactCircleCable2DParameters& GetParameters() const { return parameters; }

    //! AUTO:  default function to return Marker numbers
    virtual const ArrayIndex& GetMarkerNumbers() const override
    {
        return parameters.markerNumbers;
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;
    }

    //! AUTO:  number of nodes; needed for every object
    virtual Index GetNumberOfNodes() const override
    {
        return 1;
    }

    //! AUTO:  Needs a data variable for every contact segment (tells if this segment is in contact or not)
    virtual Index GetDataVariablesSize() const override
    {
        return parameters.numberOfContactSegments;
    }

    //! AUTO:  compute gap for given MarkerData; done for every contact point (numberOfSegments+1) --> in order to decide contact state for every segment; in case of positive gap, the area is distance*segment_length
    void ComputeGap(const MarkerDataStructure& markerData, ConstSizeVector<CObjectContactCircleCable2DmaxNumberOfSegments>& gapPerSegment, ConstSizeVector<CObjectContactCircleCable2DmaxNumberOfSegments>& referenceCoordinatePerSegment, ConstSizeVector<CObjectContactCircleCable2DmaxNumberOfSegments>& xDirectionGap, ConstSizeVector<CObjectContactCircleCable2DmaxNumberOfSegments>& yDirectionGap) const;

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

    //! AUTO:  connector uses penalty formulation
    virtual bool IsPenaltyConnector() const override
    {
        return true;
    }

    //! AUTO:  Flags to determine, which output variables are available (displacment, velocity, stress, ...)
    virtual OutputVariableType GetOutputVariableTypes() const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Index itemIndex, Vector& value) const override;

    //! AUTO:  provide requested markerType for connector; for different markerTypes in marker0/1 => set to ::\_None
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return Marker::_None;
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

    //! AUTO:  return if contact is active-->avoids computation of ODE2LHS, speeds up computation
    bool IsContactActive() const;

};



#endif //#ifdef include once...
