/** ***********************************************************************************************
* @class        CObjectContactFrictionCircleCable2DParameters
* @brief        Parameter class for CObjectContactFrictionCircleCable2D
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-08-11  21:25:32 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTCONTACTFRICTIONCIRCLECABLE2DPARAMETERS__H
#define COBJECTCONTACTFRICTIONCIRCLECABLE2DPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"


//! AUTO: Parameters for class CObjectContactFrictionCircleCable2DParameters
class CObjectContactFrictionCircleCable2DParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: markers define contact gap
    Index nodeNumber;                             //!< AUTO: node number of a NodeGenericData with 3 \f$\times\f$ nSegments dataCoordinates (used for active set strategy ==> hold the gap of the last discontinuous iteration and the friction state)
    Index numberOfContactSegments;                //!< AUTO: number of linear contact segments to determine contact; each segment is a line and is associated to a data (history) variable; must be same as in according marker
    Real contactStiffness;                        //!< AUTO: contact (penalty) stiffness [SI:N/m/(contact segment)]; the stiffness is per contact segment; specific contact forces (per length) \f$f_N\f$ act in contact normal direction only upon penetration
    Real contactDamping;                          //!< AUTO: contact damping [SI:N/(m s)/(contact segment)]; the damping is per contact segment; acts in contact normal direction only upon penetration
    Real frictionVelocityPenalty;                 //!< AUTO: velocity dependent penalty coefficient for friction [SI:N/(m s)/(contact segment)]; the coefficient causes tangential (contact) forces against relative tangential velocities in the contact area
    Real frictionStiffness;                       //!< AUTO: CURRENTLY NOT IMPLEMENTED: displacement dependent penalty/stiffness coefficient for friction [SI:N/m/(contact segment)]; the coefficient causes tangential (contact) forces against relative tangential displacements in the contact area
    Real frictionCoefficient;                     //!< AUTO: friction coefficient \f$\mu\f$ [SI: 1]; tangential specific friction forces (per length) \f$f_T\f$ must fulfill the condition \f$f_T \le \mu f_N\f$
    Real circleRadius;                            //!< AUTO: radius [SI:m] of contact circle
    Real offset;                                  //!< AUTO: offset [SI:m] of contact, e.g. to include thickness of cable element
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temorarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectContactFrictionCircleCable2DParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        nodeNumber = EXUstd::InvalidIndex;
        numberOfContactSegments = 3;
        contactStiffness = 0.;
        contactDamping = 0.;
        frictionVelocityPenalty = 0.;
        frictionStiffness = 0.;
        frictionCoefficient = 0.;
        circleRadius = 0.;
        offset = 0.;
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectContactFrictionCircleCable2D
* @brief        A very specialized penalty-based contact/friction condition between a 2D circle in the local x/y plane (=marker0, a Rigid-Body Marker) on a body and an ANCFCable2DShape (=marker1, Marker: BodyCable2DShape), in xy-plane; a node NodeGenericData is required with 3\f$\times\f$(number of contact segments) -- containing per segment: [contact gap, stick/slip (stick=1), last friction position]; the contact gap \f$g\f$ is integrated (piecewise linear) along the cable and circle; the contact force \f$f_c\f$ is zero for \f$gap>0\f$ and otherwise computed from \f$f_c = g*contactStiffness + \dot g*contactDamping\f$; during Newton iterations, the contact force is actived only, if \f$dataCoordinate[0] <= 0\f$; dataCoordinate is set equal to gap in nonlinear iterations, but not modified in Newton iterations.
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

//! AUTO: CObjectContactFrictionCircleCable2D
class CObjectContactFrictionCircleCable2D: public CObjectConnector // AUTO: 
{
protected: // AUTO: 
    CObjectContactFrictionCircleCable2DParameters parameters; //! AUTO: contains all parameters for CObjectContactFrictionCircleCable2D

public: // AUTO: 
    static constexpr Index maxNumberOfSegments = 12; //maximum number of contact segments

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectContactFrictionCircleCable2DParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectContactFrictionCircleCable2DParameters& GetParameters() const { return parameters; }

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

    //! AUTO:  Needs a data variable for every contact segment (tells if this segment is in contact or not), every friction condition (stick = 1, slip = 0), and the last sticking position in tangential direction in terms of an angle \f$\varphi\f$ in the local circle coordinates (\f$\varphi = 0\f$, if the vector to the contact position is aligned with the x-axis)
    virtual Index GetDataVariablesSize() const override
    {
        return 3*parameters.numberOfContactSegments;
    }

    //! AUTO:  compute gap for given MarkerData; done for every contact point (numberOfSegments+1) --> in order to decide contact state for every segment; in case of positive gap, the area is distance*segment_length
    void ComputeGap(const MarkerDataStructure& markerData, ConstSizeVector<maxNumberOfSegments>& gapPerSegment, ConstSizeVector<maxNumberOfSegments>& referenceCoordinatePerSegment, ConstSizeVector<maxNumberOfSegments>& xDirectionGap, ConstSizeVector<maxNumberOfSegments>& yDirectionGap) const;

    //! AUTO:  Computational function: compute right-hand-side (RHS) of second order ordinary differential equations (ODE) to 'ode2rhs'
    virtual void ComputeODE2RHS(Vector& ode2Rhs, const MarkerDataStructure& markerData) const override;

    //! AUTO:  Computational function: compute Jacobian of ODE2 RHS equations w.r.t. ODE coordinates (jacobian) and if JacobianType::ODE2_ODE2_t flag is set in GetAvailableJacobians() compute jacobian w.r.t. ODE_t coordinates
    virtual void ComputeJacobianODE2_ODE2(ResizableMatrix& jacobian, ResizableMatrix& jacobian_ODE2_t, const MarkerDataStructure& markerData) const override;

    //! AUTO:  flag to be set for connectors, which use DiscontinuousIteration
    virtual bool HasDiscontinuousIteration() const override
    {
        return true;
    }

    //! AUTO:  function called after Newton method; returns a residual error (force)
    virtual Real PostNewtonStep(const MarkerDataStructure& markerDataCurrent, PostNewtonFlags::Type& flags) override;

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
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Vector& value) const override;

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
    virtual bool IsActive() const override;

};



#endif //#ifdef include once...
