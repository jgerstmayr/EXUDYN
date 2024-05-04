/** ***********************************************************************************************
* @class        CObjectJointRollingDiscParameters
* @brief        Parameter class for CObjectJointRollingDisc
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2024-04-29  08:41:26 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTJOINTROLLINGDISCPARAMETERS__H
#define COBJECTJOINTROLLINGDISCPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CObjectJointRollingDiscParameters
class CObjectJointRollingDiscParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector; \f$m0\f$ represents the ground and \f$m1\f$ represents the rolling body, which has its reference point (=local position [0,0,0]) at the disc center point
    ArrayIndex constrainedAxes;                   //!< AUTO: flag, which determines which constraints are active, in which \f$j_0,j_1\f$ represent the tangential motion and \f$j_2\f$ represents the normal (contact) direction
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    Real discRadius;                              //!< AUTO: defines the disc radius
    Vector3D discAxis;                            //!< AUTO: axis of disc defined in marker \f$m1\f$ frame
    Vector3D planeNormal;                         //!< AUTO: normal to the contact / rolling plane defined in marker \f$m0\f$ coordinates
    //! AUTO: default constructor with parameter initialization
    CObjectJointRollingDiscParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        constrainedAxes = ArrayIndex({1,1,1});
        activeConnector = true;
        discRadius = 0;
        discAxis = Vector3D({1,0,0});
        planeNormal = Vector3D({0,0,1});
    };
};


/** ***********************************************************************************************
* @class        CObjectJointRollingDisc
* @brief        A joint representing a rolling rigid disc (marker 1) on a flat surface (marker 0, ground body) in global \f$x\f$-\f$y\f$ plane. The contraint is based on an idealized rolling formulation with no slip. The contraints works for discs as long as the disc axis and the plane normal vector are not parallel. It must be assured that the disc has contact to ground in the initial configuration (adjust z-position of body accordingly). The ground body can be a rigid body which is moving. In this case, the flat surface is assumed to be in the \f$x\f$-\f$y\f$-plane at \f$z=0\f$. Note that the rolling body must have the reference point at the center of the disc. NOTE: the cases of normal other than \f$z\f$-direction, wheel axis other than \f$x\f$-axis and moving ground body needs to be tested further, check your results!
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

//! AUTO: CObjectJointRollingDisc
class CObjectJointRollingDisc: public CObjectConstraint // AUTO: 
{
protected: // AUTO: 
    static constexpr Index nConstraints = 3;
    CObjectJointRollingDiscParameters parameters; //! AUTO: contains all parameters for CObjectJointRollingDisc

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectJointRollingDiscParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectJointRollingDiscParameters& GetParameters() const { return parameters; }

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

    //! AUTO:  Computational function: compute algebraic equations and write residual into 'algebraicEquations'; velocityLevel: equation provided at velocity level
    virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false) const override;

    //! AUTO:  compute derivative of algebraic equations w.r.t. \hac{ODE2}, \hac{ODE2t}, \hac{ODE1} and \hac{AE} coordinates in jacobian [flags ODE2_t_AE_function, AE_AE_function, etc. need to be set in GetAvailableJacobians()]; jacobianODE2[_t] has dimension GetAlgebraicEquationsSize() x GetODE2Size() ; q are the system coordinates; markerData provides according marker information to compute jacobians
    virtual void ComputeJacobianAE(ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE, const MarkerDataStructure& markerData, Real t, Index itemIndex) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags; available jacobians is switched depending on velocity level and on activeConnector condition
    virtual JacobianType::Type GetAvailableJacobians() const override;

    //! AUTO:  Return true, if constraint currently is formulated at velocity level (e.g. coordinate constraint ==> this information is needed for correct jacobian computation)
    virtual bool UsesVelocityLevel() const override
    {
        return true;
    }

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Index itemIndex, Vector& value) const override;

    //! AUTO:  provide requested markerType for connector
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return (Marker::Type)((Index)Marker::Position + (Index)Marker::Orientation);
    }

    //! AUTO:  return object type (for node treatment in computation)
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);
    }

    //! AUTO:  number of algebraic equations; independent of node/body coordinates
    virtual Index GetAlgebraicEquationsSize() const override
    {
        return 3;
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
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::ForceLocal +
            (Index)OutputVariableType::RotationMatrix );
    }

};



#endif //#ifdef include once...
