/** ***********************************************************************************************
* @class        CObjectConnectorLinearSpringDamperParameters
* @brief        Parameter class for CObjectConnectorLinearSpringDamper
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-06  09:17:10 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef COBJECTCONNECTORLINEARSPRINGDAMPERPARAMETERS__H
#define COBJECTCONNECTORLINEARSPRINGDAMPERPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
namespace py = pybind11;            //! AUTO: "py" used throughout in code
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectConnectorLinearSpringDamperParameters
class CObjectConnectorLinearSpringDamperParameters // AUTO:
{
public: // AUTO:
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    Real stiffness;                               //!< AUTO: torsional stiffness [SI:Nm/rad] against relative rotation
    Real damping;                                 //!< AUTO: torsional damping [SI:Nm/(rad/s)]
    Vector3D axisMarker0;                         //!< AUTO: local axis of spring-damper in marker 0 coordinates; this axis will co-move with marker \f$m0\f$; if marker m0 is attached to ground, the spring-damper represents linear equations
    Real offset;                                  //!< AUTO: translational offset considered in the spring force calculation (this can be used as position control input!)
    Real velocityOffset;                          //!< AUTO: velocity offset considered in the damper force calculation (this can be used as velocity control input!)
    Real force;                                   //!< AUTO: additional constant force [SI:Nm] added to spring-damper; this can be used to prescribe a force between the two attached bodies (e.g., for actuation and control)
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    PythonUserFunctionBase< std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real)> > springForceUserFunction;//!< AUTO: A Python function which computes the scalar force between the two rigid body markers along axisMarker0 in \f$m0\f$ coordinates, if activeConnector=True; see description below
    //! AUTO: default constructor with parameter initialization
    CObjectConnectorLinearSpringDamperParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        stiffness = 0.;
        damping = 0.;
        axisMarker0 = Vector3D({1,0,0});
        offset = 0.;
        velocityOffset = 0.;
        force = 0.;
        activeConnector = true;
        springForceUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectConnectorLinearSpringDamper
* @brief        An linear spring-damper element acting on relative translations along given axis of local joint0 coordinate system. It connects to position and orientation-based markers; the linear spring-damper is intended to act within prismatic joints or in situations where only one translational axis is free; if the two markers rotate relative to each other, the spring-damper will always act in the local joint0 coordinate system.
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

//! AUTO: CObjectConnectorLinearSpringDamper
class CObjectConnectorLinearSpringDamper: public CObjectConnector // AUTO:
{
protected: // AUTO:
    CObjectConnectorLinearSpringDamperParameters parameters; //! AUTO: contains all parameters for CObjectConnectorLinearSpringDamper

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectConnectorLinearSpringDamperParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectConnectorLinearSpringDamperParameters& GetParameters() const { return parameters; }

    //! AUTO:  no PotentialEnergy while a user function defines the force (#2202)
    virtual bool PotentialEnergyAvailable() const override
    {
        return !parameters.springForceUserFunction;
    }

    //! AUTO:  return true, if object has a computation user function
    virtual bool HasUserFunction() const override
    {
        return (parameters.springForceUserFunction!=0);
    }

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
    virtual JacobianType::Type GetAvailableJacobians() const override;

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
        return CObjectType::Connector;
    }

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

    //! AUTO:  the physics of the connector, for Real and AutoDiff (#2745)
    template<class TReal> void ComputeSpringForce(const MarkerRigid<TReal>* markers, Real t, Index itemIndex, ConstSizeMatrixBase<TReal, 9>& A0, TReal& displacement, TReal& velocity, TReal& force) const;

    //! AUTO:  the right-hand side on the connector interface of rigid markers (#2745)
    virtual bool ComputeODE2LHSConnector(const CSystemData& systemData, TemporaryComputationData& temp, Vector& localODE2Lhs, Index objectNumber) const override
    {
        ConnectorODE2LHSRigidMarkers(systemData, temp, *this, localODE2Lhs, objectNumber); return true;
    }

    //! AUTO:  the Jacobian by automatic differentiation of the force (#2745)
    virtual bool ComputeJacobianODE2Connector(const CSystemData& systemData, TemporaryComputationData& temp, Real factorODE2, Real factorODE2_t, Index objectNumber, bool jacobianDerivativeNonZero) const override
    {
        ConnectorJacobianODE2RigidMarkers(systemData, temp, *this, factorODE2, factorODE2_t, objectNumber, jacobianDerivativeNonZero); return true;
    }

    //! AUTO:  the force and torque on each marker from the kinematics of the two markers (#2745)
    virtual void ComputeConnectorForceRigid(const MarkerRigid<Real>* markers, Real t, Index itemIndex, Vector3D* forces, Vector3D* torques) const override;

    //! AUTO:  the same forces and torques with automatic differentiation, for the Jacobian (#2745)
    virtual void ComputeConnectorForceRigidDiff(const MarkerRigid<DRealRigidMarkers>* markers, Real t, Index itemIndex, SlimVectorBase<DRealRigidMarkers, 3>* forces, SlimVectorBase<DRealRigidMarkers, 3>* torques) const override;

    //! AUTO:  the forces and torques of both, Real and AutoDiff (#2745)
    template<class TReal> void ComputeConnectorForceRigidTemplate(const MarkerRigid<TReal>* markers, Real t, Index itemIndex, SlimVectorBase<TReal, 3>* forces, SlimVectorBase<TReal, 3>* torques) const;

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionForce(Real& force, const MainSystemBase& mainSystem, Real t, Index itemIndex, Real displacement, Real velocity) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index64)OutputVariableType::DisplacementLocal +
            (Index64)OutputVariableType::Displacement +
            (Index64)OutputVariableType::HomogeneousTransformationLocal +
            (Index64)OutputVariableType::VelocityLocal +
            (Index64)OutputVariableType::ForceLocal +
            (Index64)OutputVariableType::PotentialEnergy );
    }

};



#endif //#ifdef include once...
