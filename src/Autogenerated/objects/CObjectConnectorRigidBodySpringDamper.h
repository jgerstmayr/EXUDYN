/** ***********************************************************************************************
* @class        CObjectConnectorRigidBodySpringDamperParameters
* @brief        Parameter class for CObjectConnectorRigidBodySpringDamper
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-03  10:53:12 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef COBJECTCONNECTORRIGIDBODYSPRINGDAMPERPARAMETERS__H
#define COBJECTCONNECTORRIGIDBODYSPRINGDAMPERPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
namespace py = pybind11;            //! AUTO: "py" used throughout in code
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectConnectorRigidBodySpringDamperParameters
class CObjectConnectorRigidBodySpringDamperParameters // AUTO:
{
public: // AUTO:
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    Index nodeNumber;                             //!< AUTO: node number of a NodeGenericData (size depends on application) for dataCoordinates for user functions (e.g., implementing contact/friction user function)
    Matrix6D stiffness;                           //!< AUTO: stiffness [SI:N/m or Nm/rad] of translational, torsional and coupled springs; act against relative displacements in x, y, and z-direction as well as the relative angles (calculated as Euler angles); in the simplest case, the first 3 diagonal values correspond to the local stiffness in x,y,z direction and the last 3 diagonal values correspond to the rotational stiffness around x,y and z axis
    Matrix6D damping;                             //!< AUTO: damping [SI:N/(m/s) or Nm/(rad/s)] of translational, torsional and coupled dampers; very similar to stiffness, however, the rotational velocity is computed from the angular velocity vector
    Matrix3D rotationMarker0;                     //!< AUTO: local rotation matrix for marker 0; stiffness, damping, etc. components are measured in local coordinates relative to rotationMarker0; **deprecated** (removed in 2031): give the rotation to marker 0 as its localHT
    Matrix3D rotationMarker1;                     //!< AUTO: local rotation matrix for marker 1; stiffness, damping, etc. components are measured in local coordinates relative to rotationMarker1; **deprecated** (removed in 2031): give the rotation to marker 1 as its localHT
    Vector6D offset;                              //!< AUTO: translational and rotational offset considered in the spring force calculation
    bool intrinsicFormulation;                    //!< AUTO: if True, the joint uses the intrinsic formulation, which is independent on order of markers, using a mid-point and mid-rotation for evaluation and application of connector forces and torques; this uses a Lie group formulation; in this case, the force/torque vector is computed from the stiffness matrix times the 6-vector of the SE3 matrix logarithm between the two marker positions/rotations, see the equations
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    PythonUserFunctionBase< std::function<StdVector6D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)> > springForceTorqueUserFunction;//!< AUTO: A Python function which computes the 6D force-torque vector (3D force + 3D torque) between the two rigid body markers, if activeConnector=True; see description below
    PythonUserFunctionBase< std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)> > postNewtonStepUserFunction;//!< AUTO: A Python function which computes the error of the PostNewtonStep; see description below
    //! AUTO: default constructor with parameter initialization
    CObjectConnectorRigidBodySpringDamperParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        nodeNumber = EXUstd::InvalidIndex;
        stiffness = Matrix6D(6,6,0.);
        damping = Matrix6D(6,6,0.);
        rotationMarker0 = EXUmath::unitMatrix3D;
        rotationMarker1 = EXUmath::unitMatrix3D;
        offset = Vector6D({0.,0.,0.,0.,0.,0.});
        intrinsicFormulation = false;
        activeConnector = true;
        springForceTorqueUserFunction = 0;
        postNewtonStepUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectConnectorRigidBodySpringDamper
* @brief        An 3D spring-damper element acting on relative displacements and relative rotations of two rigid body (position+orientation) markers. It represents a penalty-based rigid joint (or prismatic, revolute, etc.)
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

//! AUTO: CObjectConnectorRigidBodySpringDamper
class CObjectConnectorRigidBodySpringDamper: public CObjectConnector // AUTO:
{
protected: // AUTO:
    CObjectConnectorRigidBodySpringDamperParameters parameters; //! AUTO: contains all parameters for CObjectConnectorRigidBodySpringDamper

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectConnectorRigidBodySpringDamperParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectConnectorRigidBodySpringDamperParameters& GetParameters() const { return parameters; }

    //! AUTO:  no PotentialEnergy while a user function defines the force (#2202)
    virtual bool PotentialEnergyAvailable() const override
    {
        return !parameters.springForceTorqueUserFunction;
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

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual void SetNodeNumber(Index localIndex, Index nodeNumber) override
    {
        parameters.nodeNumber=nodeNumber;
    }

    //! AUTO:  number of nodes; needed for every object; can depend on the configuration
    virtual Index GetNumberOfNodes() const override
    {
        return (parameters.postNewtonStepUserFunction!=0);
    }

    //! AUTO:  return true, if object has a computation user function
    virtual bool HasUserFunction() const override
    {
        return (parameters.springForceTorqueUserFunction!=0);
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

    //! AUTO:  flag to be set for connectors, which use DiscontinuousIteration
    virtual bool HasDiscontinuousIteration() const override
    {
        return (parameters.postNewtonStepUserFunction!=0);
    }

    //! AUTO:  function called after Newton method; returns a residual error (force)
    virtual Real PostNewtonStep(const MarkerDataStructure& markerDataCurrent, Index itemIndex, PostNewtonFlags::Type& flags, Real& recommendedStepSize) override;

    //! AUTO:  function called after discontinuous iterations have been completed for one step (e.g. to finalize history variables and set initial values for next step)
    virtual void PostDiscontinuousIterationStep() override
    {

    }

    //! AUTO:  the physics of the connector, for Real and AutoDiff (#2745)
    template<class TReal> void ComputeSpringForceTorque(const MarkerRigid<TReal>* markers, Real t, Index itemIndex, ConstSizeMatrixBase<TReal, 9>& Ajoint, SlimVectorBase<TReal, 3>& vLocPos, SlimVectorBase<TReal, 3>& vLocVel, SlimVectorBase<TReal, 3>& vLocRot, SlimVectorBase<TReal, 3>& vLocAngVel, SlimVectorBase<TReal, 6>& fLocVec6D, bool computeForceTorque=true) const;

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
    void EvaluateUserFunctionForce(Vector6D& fLocVec6D, const MainSystemBase& mainSystem, Real t, Index itemIndex, Vector6D& uLoc6D, Vector6D& vLoc6D) const;

    //! AUTO:  call to post Newton step user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionPostNewtonStep(Vector& returnValue, const MainSystemBase& mainSystem, Real t, Index itemIndex, Vector& dataCoordinates, Vector6D& uLoc6D, Vector6D& vLoc6D) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index64)OutputVariableType::DisplacementLocal +
            (Index64)OutputVariableType::VelocityLocal +
            (Index64)OutputVariableType::Rotation +
            (Index64)OutputVariableType::AngularVelocityLocal +
            (Index64)OutputVariableType::ForceLocal +
            (Index64)OutputVariableType::TorqueLocal +
            (Index64)OutputVariableType::PotentialEnergy );
    }

};



#endif //#ifdef include once...
