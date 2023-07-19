/** ***********************************************************************************************
* @class        CObjectConnectorReevingSystemSpringsParameters
* @brief        Parameter class for CObjectConnectorReevingSystemSprings
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2023-07-12  16:03:38 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTCONNECTORREEVINGSYSTEMSPRINGSPARAMETERS__H
#define COBJECTCONNECTORREEVINGSYSTEMSPRINGSPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectConnectorReevingSystemSpringsParameters
class CObjectConnectorReevingSystemSpringsParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of position or rigid body markers used in reeving system and optional two coordinate markers (\f$m_{c0}, \, m_{c1}\f$); the first marker \f$m_0\f$ and the last rigid body marker \f$m_{nr-1}\f$ represent the ends of the rope and are directly connected to a position; the markers \f$m_1, \, \ldots, \, m_{nr-2}\f$ can be connected to sheaves, for which a radius and an axis can be prescribed. The coordinate markers are optional and represent prescribed length at the rope ends (marker \f$m_{c0}\f$ is added length at start, marker \f$m_{c1}\f$ is added length at end of the rope in the reeving system)
    bool hasCoordinateMarkers;                    //!< AUTO: flag, which determines, the list of markers (markerNumbers) contains two coordinate markers at the end of the list, representing the prescribed change of length at both ends
    Vector2D coordinateFactors;                   //!< AUTO: factors which are multiplied with the values of coordinate markers; this can be used, e.g., to change directions or to transform rotations (revolutions of a sheave) into change of length
    Real stiffnessPerLength;                      //!< AUTO: stiffness per length [SI:N/m/m] of rope; in case of cross section \f$A\f$ and Young's modulus \f$E\f$, this parameter results in \f$E\cdot A\f$; the effective stiffness of the reeving system is computed as \f$EA/L\f$ in which \f$L\f$ is the current length of the rope
    Real dampingPerLength;                        //!< AUTO: axial damping per length [SI:N/(m/s)/m] of rope; the effective damping coefficient of the reeving system is computed as \f$DA/L\f$ in which \f$L\f$ is the current length of the rope
    Real dampingTorsional;                        //!< AUTO: torsional damping [SI:Nms] between sheaves; this effect can damp rotations around the rope axis, pairwise between sheaves; this parameter is experimental
    Real dampingShear;                            //!< AUTO: damping of shear motion [SI:Ns] between sheaves; this effect can damp motion perpendicular to the rope between each pair of sheaves; this parameter is experimental
    Real regularizationForce;                     //!< AUTO: small regularization force [SI:N] in order to avoid large compressive forces; this regularization force can either be \f$<0\f$ (using a linear tension/compression spring model) or \f$>0\f$, which restricts forces in the rope to be always \f$\ge -F_{reg}\f$. Note that smaller forces lead to problems in implicit integrators and smaller time steps. For explicit integrators, this force can be chosen close to zero.
    Real referenceLength;                         //!< AUTO: reference length for computation of roped force
    Vector3DList sheavesAxes;                     //!< AUTO: list of local vectors axes of sheaves; vectors refer to rigid body markers given in list of markerNumbers; first and last axes are ignored, as they represent the attachment of the rope ends
    Vector sheavesRadii;                          //!< AUTO: radius for each sheave, related to list of markerNumbers and list of sheaveAxes; first and last radii must always be zero.
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectConnectorReevingSystemSpringsParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        hasCoordinateMarkers = false;
        coordinateFactors = Vector2D({1,1});
        stiffnessPerLength = 0.;
        dampingPerLength = 0.;
        dampingTorsional = 0.;
        dampingShear = 0.;
        regularizationForce = 0.1;
        referenceLength = 0.;
        sheavesAxes = Vector3DList();
        sheavesRadii = Vector();
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectConnectorReevingSystemSprings
* @brief        A rD reeving system defined by a list of torque-free and friction-free sheaves or points that are connected with one rope (modelled as massless spring). NOTE that the spring can undergo tension AND compression (in order to avoid compression, use a PreStepUserFunction to turn off stiffness and damping in this case!). The force is assumed to be constant all over the rope. The sheaves or connection points are defined by \f$nr\f$ rigid body markers \f$[m_0, \, m_1, \, \ldots, \, m_{nr-1}]\f$. At both ends of the rope there may be a prescribed motion coupled to a coordinate marker each, given by \f$m_{c0}\f$ and \f$m_{c1}\f$ .
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

//! AUTO: CObjectConnectorReevingSystemSprings
class CObjectConnectorReevingSystemSprings: public CObjectConnector // AUTO: 
{
protected: // AUTO: 
    CObjectConnectorReevingSystemSpringsParameters parameters; //! AUTO: contains all parameters for CObjectConnectorReevingSystemSprings
    mutable Vector3DList tempPositionsList;       //!< AUTO: temporary list of vectors representing the rope local positions

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    CObjectConnectorReevingSystemSprings()
    {
        tempPositionsList = Vector3DList();
    };

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectConnectorReevingSystemSpringsParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectConnectorReevingSystemSpringsParameters& GetParameters() const { return parameters; }

    //! AUTO:  Write (Reference) access to:temporary list of vectors representing the rope local positions
    void SetTempPositionsList(const Vector3DList& value) { tempPositionsList = value; }
    //! AUTO:  Read (Reference) access to:temporary list of vectors representing the rope local positions
    const Vector3DList& GetTempPositionsList() const { return tempPositionsList; }
    //! AUTO:  Read (Reference) access to:temporary list of vectors representing the rope local positions
    Vector3DList& GetTempPositionsList() { return tempPositionsList; }

    //! AUTO:  default function to return Marker numbers
    virtual const ArrayIndex& GetMarkerNumbers() const override
    {
        return parameters.markerNumbers;
    }

    //! AUTO:  number of markers; 0 means no requirements
    virtual Index RequestedNumberOfMarkers() const override
    {
        return 0;
    }

    //! AUTO:  return true, if object has a computation user function
    virtual bool HasUserFunction() const override
    {
        return false;
    }

    //! AUTO:  connector uses penalty formulation
    virtual bool IsPenaltyConnector() const override
    {
        return true;
    }

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs, const MarkerDataStructure& markerData, Index objectNumber) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);
    }

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Index itemIndex, Vector& value) const override;

    //! AUTO:  provide requested markerType for connector
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

    //! AUTO:  compute reeving geometry based on tempPositionsList, length, time derivative of length
    void ComputeReevingGeometry(const MarkerDataStructure& markerData, Index itemIndex, Vector3DList& positionsList, Real& actualLength, Real& actualLength_t, bool storePositions) const;

    //! AUTO:  compute force in reeving system (including damping)
    Real ComputeForce(Real L, Real L0, Real L_t, Real L0_t, Real EA, Real DA) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Distance +
            (Index)OutputVariableType::VelocityLocal +
            (Index)OutputVariableType::ForceLocal );
    }

};



#endif //#ifdef include once...
