/** ***********************************************************************************************
* @class        CObjectConnectorHydraulicActuatorSimpleParameters
* @brief        Parameter class for CObjectConnectorHydraulicActuatorSimple
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2023-05-02  16:50:45 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTCONNECTORHYDRAULICACTUATORSIMPLEPARAMETERS__H
#define COBJECTCONNECTORHYDRAULICACTUATORSIMPLEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectConnectorHydraulicActuatorSimpleParameters
class CObjectConnectorHydraulicActuatorSimpleParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    ArrayIndex nodeNumbers;                       //!< AUTO: currently a list with one node number of NodeGenericODE1 for 2 hydraulic pressures (reference values for this node must be zero); data node may be added in future for switching
    Real offsetLength;                            //!< AUTO: offset length [SI:m] of cylinder, representing minimal distance between the two bushings at stroke=0
    Real strokeLength;                            //!< AUTO: stroke length [SI:m] of cylinder, representing maximum extension relative to \f$L_o\f$; the measured distance between the markers is \f$L_s+L_o\f$
    Real chamberCrossSection0;                    //!< AUTO: cross section [SI:m\f$^2\f$] of chamber (inner cylinder) at piston head (nut) side (0)
    Real chamberCrossSection1;                    //!< AUTO: cross section [SI:m\f$^2\f$] of chamber at piston rod side (1); usually smaller than chamberCrossSection0
    Real hoseVolume0;                             //!< AUTO: hose volume [SI:m\f$^3\f$] at piston head (nut) side (0); as the effective bulk modulus would go to infinity at stroke length zero, the hose volume must be greater than zero
    Real hoseVolume1;                             //!< AUTO: hose volume [SI:m\f$^3\f$] at piston rod side (1); as the effective bulk modulus would go to infinity at max. stroke length, the hose volume must be greater than zero
    Real valveOpening0;                           //!< AUTO: relative opening of valve \f$[-1 \ldots 1]\f$ [SI:1] at piston head (nut) side (0); positive value is valve opening towards system pressure, negative value is valve opening towards tank pressure; zero means closed valve
    Real valveOpening1;                           //!< AUTO: relative opening of valve \f$[-1 \ldots 1]\f$ [SI:1] at piston rod side (1); positive value is valve opening towards system pressure, negative value is valve opening towards tank pressure; zero means closed valve
    Real actuatorDamping;                         //!< AUTO: damping [SI:N/(m\f$\,\f$s)] of hydraulic actuator (against actuator axial velocity)
    Real oilBulkModulus;                          //!< AUTO: bulk modulus of oil [SI:N/(m\f$^2\f$)]
    Real cylinderBulkModulus;                     //!< AUTO: bulk modulus of cylinder [SI:N/(m\f$^2\f$)]; in fact, this is value represents the effect of the cylinder stiffness on the effective bulk modulus
    Real hoseBulkModulus;                         //!< AUTO: bulk modulus of hose [SI:N/(m\f$^2\f$)]; in fact, this is value represents the effect of the hose stiffness on the effective bulk modulus
    Real nominalFlow;                             //!< AUTO: nominal flow of oil through valve [SI:m\f$^3\f$/s]
    Real systemPressure;                          //!< AUTO: system pressure [SI:N/(m\f$^2\f$)]
    Real tankPressure;                            //!< AUTO: tank pressure [SI:N/(m\f$^2\f$)]
    bool useChamberVolumeChange;                  //!< AUTO: if True, the pressure build up equations include the change of oil stiffness due to change of chamber volume
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectConnectorHydraulicActuatorSimpleParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        nodeNumbers = ArrayIndex();
        offsetLength = 0.;
        strokeLength = 0.;
        chamberCrossSection0 = 0.;
        chamberCrossSection1 = 0.;
        hoseVolume0 = 0.;
        hoseVolume1 = 0.;
        valveOpening0 = 0.;
        valveOpening1 = 0.;
        actuatorDamping = 0.;
        oilBulkModulus = 0.;
        cylinderBulkModulus = 0.;
        hoseBulkModulus = 0.;
        nominalFlow = 0.;
        systemPressure = 0.;
        tankPressure = 0.;
        useChamberVolumeChange = false;
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectConnectorHydraulicActuatorSimple
* @brief        A basic hydraulic actuator with pressure build up equations. The actuator follows a valve input value, which results in a in- or outflow of fluid depending on the pressure difference. Valve values can be prescribed by user functions (not yet available) or with the MainSystem PreStepUserFunction(...).
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

//! AUTO: CObjectConnectorHydraulicActuatorSimple
class CObjectConnectorHydraulicActuatorSimple: public CObjectConnector // AUTO: 
{
protected: // AUTO: 
    CObjectConnectorHydraulicActuatorSimpleParameters parameters; //! AUTO: contains all parameters for CObjectConnectorHydraulicActuatorSimple

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectConnectorHydraulicActuatorSimpleParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectConnectorHydraulicActuatorSimpleParameters& GetParameters() const { return parameters; }

    //! AUTO:  default function to return Marker numbers
    virtual const ArrayIndex& GetMarkerNumbers() const override
    {
        return parameters.markerNumbers;
    }

    //! AUTO:  connector uses penalty formulation
    virtual bool IsPenaltyConnector() const override
    {
        return true;
    }

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs, const MarkerDataStructure& markerData, Index objectNumber) const override;

    //! AUTO:  Computational function: compute right-hand-side (RHS) of first order ordinary differential equations (ODE) to 'ode1Rhs'
    virtual void ComputeODE1RHS(Vector& ode1Rhs, const MarkerDataStructure& markerData, Index objectNumber) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Index itemIndex, Vector& value) const override;

    //! AUTO:  provide requested markerType for connector
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return Marker::Position;
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        return parameters.nodeNumbers[localIndex];
    }

    //! AUTO:  number of nodes; needed for every object
    virtual Index GetNumberOfNodes() const override
    {
        return parameters.nodeNumbers.NumberOfItems();
    }

    //! AUTO:  number of \hac{ODE1} coordinates; needed for object?
    virtual Index GetODE1Size() const override;

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
    void ComputeConnectorProperties(const MarkerDataStructure& markerData, Index itemIndex, Vector3D& relPos, Vector3D& relVel, Real& linearVelocity, Real& force, Vector3D& forceDirection) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Distance +
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::VelocityLocal +
            (Index)OutputVariableType::Force );
    }

};



#endif //#ifdef include once...
