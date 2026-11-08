/** ***********************************************************************************************
* @class        CObjectJournalBearingParameters
* @brief        Parameter class for CObjectJournalBearing
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-11-08  22:55:36 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef COBJECTJOURNALBEARINGPARAMETERS__H
#define COBJECTJOURNALBEARINGPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
namespace py = pybind11;            //! AUTO: "py" used throughout in code

//! AUTO: Parameters for class CObjectJournalBearingParameters
class CObjectJournalBearingParameters // AUTO:
{
public: // AUTO:
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers: the center of the journal (marker 0) and the center of the bearing (marker 1), whose local z axis is the axis of the bearing
    Real journalRadius;                           //!< AUTO: must be > 0; radius of the journal [SI:m]
    Real bearingRadius;                           //!< AUTO: must be > 0; radius of the bearing, larger than journalRadius; the clearance is \f$c = R_B - R_J\f$ [SI:m]
    Real bearingLength;                           //!< AUTO: must be > 0; length of the bearing along its axis [SI:m]
    Real viscosity;                               //!< AUTO: must be >= 0; dynamic viscosity of the lubricant [SI:Pa s]
    JournalBearingModel bearingModel;             //!< AUTO: the model of the film, see JournalBearingModel: Short (Ocvirk) or Long (Sommerfeld); the integers 0, 1 are accepted as well
    Real maximumEccentricityRatio;                //!< AUTO: must be > 0; the largest eccentricity ratio \f$e/c\f$ the film force is evaluated at, below 1, where it is singular [SI:1]
    Real contactStiffness;                        //!< AUTO: must be >= 0; stiffness of the contact where the eccentricity exceeds the clearance [SI:N/m]
    Real contactDamping;                          //!< AUTO: must be >= 0; damping of the contact where the eccentricity exceeds the clearance [SI:N/(m s)]
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    PythonUserFunctionBase< std::function<StdVector3D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,Real,StdVector3D)> > forceUserFunction;//!< AUTO: A Python function which replaces the force of the bearing model and of the contact and returns the force of the film on marker 0, the journal; see description below (#3335)
    //! AUTO: default constructor with parameter initialization
    CObjectJournalBearingParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        journalRadius = 0.;
        bearingRadius = 0.;
        bearingLength = 0.;
        viscosity = 0.;
        bearingModel = JournalBearingModel::Short;
        maximumEccentricityRatio = 0.99;
        contactStiffness = 0.;
        contactDamping = 0.;
        activeConnector = true;
        forceUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectJournalBearing
* @brief        A lubricated journal bearing, a revolute joint with clearance between a journal (marker 0) and a bearing (marker 1), whose axis is the local z axis of marker 1: the hydrodynamic force of the oil film by the model of the infinitely short or the infinitely long bearing, and a penalty contact where the eccentricity exceeds the clearance; a user function for further models.
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

//! AUTO: CObjectJournalBearing
class CObjectJournalBearing: public CObjectConnector // AUTO:
{
protected: // AUTO:
    CObjectJournalBearingParameters parameters; //! AUTO: contains all parameters for CObjectJournalBearing

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectJournalBearingParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectJournalBearingParameters& GetParameters() const { return parameters; }

    //! AUTO: the user functions of the item, for the parallel computation (#2978)
    virtual void GetUserFunctions(ResizableArray<const UserFunctionBase*>& list) const override
    {
        list.Append(&parameters.forceUserFunction);
    }

    //! AUTO:  return true, if object has a computation user function
    virtual bool HasUserFunction() const override
    {
        return (parameters.forceUserFunction!=0);
    }

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionForce(Vector3D& force, const MainSystemBase& mainSystem, Real t, const Vector3D& eccentricity, const Vector3D& eccentricity_t, Real angularVelocity, const Vector3D& axis) const;

    //! AUTO:  the eccentricity, its rate, the radial direction and the force of the film and the contact on marker 0
    void ComputeBearingForce(const MarkerDataStructure& markerData, Vector3D& eccentricity, Vector3D& eccentricity_t, Vector3D& radial, Vector3D& force) const;

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

    //! AUTO:  true if the connector uses a penalty formulation; false if the constraint uses Lagrange multipliers
    virtual bool IsPenaltyConnector() const override
    {
        return true;
    }

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t + JacobianType::ODE2_ODE2_function + JacobianType::ODE2_ODE2_t_function);
    }

    //! AUTO:  the right-hand side on the connector interface of rigid markers (#2745)
    virtual bool ComputeODE2LHSConnector(const CSystemData& systemData, TemporaryComputationData& temp, Vector& localODE2Lhs) const override
    {
        ConnectorODE2LHSRigidMarkers(systemData, temp, *this, localODE2Lhs); return true;
    }

    //! AUTO:  the Jacobian by local numerical differentiation of the force by the marker kinematics (#3201)
    virtual bool ComputeJacobianODE2Connector(const CSystemData& systemData, TemporaryComputationData& temp, Real factorODE2, Real factorODE2_t, bool jacobianDerivativeNonZero) const override
    {
        ConnectorJacobianODE2RigidMarkersNumerical(systemData, temp, *this, factorODE2, factorODE2_t, jacobianDerivativeNonZero); return true;
    }

    //! AUTO:  its Jacobian contains dv/dq and domega/dq of the markers (#3201)
    virtual bool RequestsVelocityDerivative() const override
    {
        return true;
    }

    //! AUTO:  the force and torque on each marker from the kinematics of the two markers (#2745)
    virtual void ComputeConnectorForceRigid(FlatArray<const MarkerRigid<Real>> markers, Real t, FlatArray<Vector3D> forces, FlatArray<Vector3D> torques) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Vector& value) const override;

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
            (Index64)OutputVariableType::Displacement +
            (Index64)OutputVariableType::Velocity +
            (Index64)OutputVariableType::Distance +
            (Index64)OutputVariableType::Director1 +
            (Index64)OutputVariableType::Force );
    }

};



#endif //#ifdef include once...
