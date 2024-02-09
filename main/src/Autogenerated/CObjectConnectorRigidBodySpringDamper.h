/** ***********************************************************************************************
* @class        CObjectConnectorRigidBodySpringDamperParameters
* @brief        Parameter class for CObjectConnectorRigidBodySpringDamper
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2024-02-02  20:40:00 (last modified)
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
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectConnectorRigidBodySpringDamperParameters
class CObjectConnectorRigidBodySpringDamperParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    Index nodeNumber;                             //!< AUTO: node number of a NodeGenericData (size depends on application) for dataCoordinates for user functions (e.g., implementing contact/friction user function)
    Matrix6D stiffness;                           //!< AUTO: stiffness [SI:N/m or Nm/rad] of translational, torsional and coupled springs; act against relative displacements in x, y, and z-direction as well as the relative angles (calculated as Euler angles); in the simplest case, the first 3 diagonal values correspond to the local stiffness in x,y,z direction and the last 3 diagonal values correspond to the rotational stiffness around x,y and z axis
    Matrix6D damping;                             //!< AUTO: damping [SI:N/(m/s) or Nm/(rad/s)] of translational, torsional and coupled dampers; very similar to stiffness, however, the rotational velocity is computed from the angular velocity vector
    Matrix3D rotationMarker0;                     //!< AUTO: local rotation matrix for marker 0; stiffness, damping, etc. components are measured in local coordinates relative to rotationMarker0
    Matrix3D rotationMarker1;                     //!< AUTO: local rotation matrix for marker 1; stiffness, damping, etc. components are measured in local coordinates relative to rotationMarker1
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
* @brief        An 3D spring-damper element acting on relative displacements and relative rotations of two rigid body (position+orientation) markers; connects to (position+orientation)-based markers and represents a penalty-based rigid joint (or prismatic, revolute, etc.)
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
        return (parameters.postNewtonStepUserFunction!=0);
    }

    //! AUTO:  return true, if object has a computation user function
    virtual bool HasUserFunction() const override
    {
        return (parameters.springForceTorqueUserFunction!=0);
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

    //! AUTO:  compute spring damper force-torque helper function
    void ComputeSpringForceTorque(const MarkerDataStructure& markerData, Index itemIndex, Matrix3D& Ajoint, Vector3D& vLocPos, Vector3D& vLocVel, Vector3D& vLocRot, Vector3D& vLocAngVel, Vector6D& fLocVec6D) const;

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionForce(Vector6D& fLocVec6D, const MainSystemBase& mainSystem, Real t, Index itemIndex, Vector6D& uLoc6D, Vector6D& vLoc6D) const;

    //! AUTO:  call to post Newton step user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionPostNewtonStep(Vector& returnValue, const MainSystemBase& mainSystem, Real t, Index itemIndex, Vector& dataCoordinates, Vector6D& uLoc6D, Vector6D& vLoc6D) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::DisplacementLocal +
            (Index)OutputVariableType::VelocityLocal +
            (Index)OutputVariableType::Rotation +
            (Index)OutputVariableType::AngularVelocityLocal +
            (Index)OutputVariableType::ForceLocal +
            (Index)OutputVariableType::TorqueLocal );
    }

};



#endif //#ifdef include once...
