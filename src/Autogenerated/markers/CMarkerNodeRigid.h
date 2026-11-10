/** ***********************************************************************************************
* @class        CMarkerNodeRigidParameters
* @brief        Parameter class for CMarkerNodeRigid
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-11-10  21:43:04 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef CMARKERNODERIGIDPARAMETERS__H
#define CMARKERNODERIGIDPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CMarkerNodeRigidParameters
class CMarkerNodeRigidParameters // AUTO:
{
public: // AUTO:
    Index nodeNumber;                             //!< AUTO: node number to which marker is attached to
    HomogeneousTransformation localHT;            //!< AUTO: the frame of the marker in the node frame, as homogeneous transformation: its rotation turns the marker frame against the node; its translation must be zero for now; a 4x4 matrix, its 16 values row by row or an exu.HT; None: the node frame
    //! AUTO: default constructor with parameter initialization
    CMarkerNodeRigidParameters()
    {
        nodeNumber = EXUstd::InvalidIndex;
        localHT = HomogeneousTransformation();
    };
};


/** ***********************************************************************************************
* @class        CMarkerNodeRigid
* @brief        A rigid-body (position+orientation) node-marker attached to a rigid-body node. It provides position and orientation (rotation), as well as the according derivatives. It can be used for most connectors, joints or loads where either position, position and orientation, or orientation are required.
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

//! AUTO: CMarkerNodeRigid
class CMarkerNodeRigid: public CMarker // AUTO:
{
protected: // AUTO:
    CMarkerNodeRigidParameters parameters; //! AUTO: contains all parameters for CMarkerNodeRigid

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerNodeRigidParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerNodeRigidParameters& GetParameters() const { return parameters; }

    //! AUTO:  general access to node number
    virtual Index GetNodeNumber() const override
    {
        return parameters.nodeNumber;
    }

    //! AUTO:  change bodyNumber
    virtual void SetNodeNumber(Index nodeNumber) override
    {
        parameters.nodeNumber = nodeNumber;
    }

    //! AUTO:  return marker type (for node treatment in computation)
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)((Index)Marker::Node + (Index)Marker::Position + (Index)Marker::Orientation + (Index)Marker::JacobianDerivativeAvailable + (Index)Marker::JacobianDerivativeNonZero);
    }

    //! AUTO:  return dimension of connector, which an attached connector would have; for coordinate markers, it gives the number of coordinates used by the marker
    virtual Index GetDimension(const CSystemData& cSystemData) const override
    {
        return 3;
    }

    //! AUTO:  return position of marker
    virtual void GetPosition(const CSystemData& cSystemData, Vector3D& position, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return velocity of marker
    virtual void GetVelocity(const CSystemData& cSystemData, Vector3D& velocity, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent rotation matrix of node; returns always a 3D Matrix
    virtual void GetRotationMatrix(const CSystemData& cSystemData, Matrix3D& rotationMatrix, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent angular velocity of node; returns always a 3D Vector
    virtual void GetAngularVelocity(const CSystemData& cSystemData, Vector3D& angularVelocity, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector
    virtual void GetAngularVelocityLocal(const CSystemData& cSystemData, Vector3D& angularVelocity, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  Compute marker data (e.g. position and positionJacobian) for a marker
    virtual void ComputeMarkerData(const CSystemData& cSystemData, bool computeJacobian, MarkerData& markerData) const override;

    //! AUTO:  fill in according data for derivative of jacobian times vector v6D, e.g.: d(Jpos.T @ v6D[0:3])/dq; v6D represents 3 force components and 3 torque components in global coordinates!
    virtual void ComputeMarkerDataJacobianDerivative(const CSystemData& cSystemData, const Vector6D& v6D, MarkerData& markerData) const override;

    //! AUTO:  dv/dq and domega/dq of the marker at fixed velocities (#3201); false if not provided
    virtual bool GetVelocity_q(const CSystemData& cSystemData, Matrix& velocity_q, Matrix& angularVelocity_q) const override;

    //! AUTO:  the derivative of the position and rotation matrix of the marker along the Euler parameters, and the row of that direction (#3201); false if there is none
    virtual bool GetRadialKinematics(const CSystemData& cSystemData, Vector3D& position_r, Matrix3D& rotation_r, Vector& radial_q) const override;

    //! AUTO:  frame and velocities of the node, without the Jacobians; a rigid body node keeps its rotation and G matrices in temp for AddGeneralizedForceTorque, other nodes go through the marker data (#2745)
    virtual Index GetKinematicsRigid(const CSystemData& cSystemData, MarkerRigid<Real>& kinematics, MarkerTemp& temp) const override;

    //! AUTO:  add J_pos^T force + J_rot^T torque to ode2Lhs, the coordinates of the node (#2745)
    virtual void AddGeneralizedForceTorque(const CSystemData& cSystemData, const Vector3D& force, const Vector3D& torque, MarkerTemp& temp, LinkedDataVector& ode2Lhs) const override;

};


//! AUTO: the member functions that the declarations of CMarkerNodeRigid require (#2866)
#include <type_traits>
static_assert(!std::is_same<decltype(&CMarkerNodeRigid::GetPosition), decltype(&CMarker::GetPosition)>::value, "CMarkerNodeRigid declares Position and must implement GetPosition; the default of CMarker raises");
static_assert(!std::is_same<decltype(&CMarkerNodeRigid::GetVelocity), decltype(&CMarker::GetVelocity)>::value, "CMarkerNodeRigid declares Position and must implement GetVelocity; the default of CMarker raises");
static_assert(!std::is_same<decltype(&CMarkerNodeRigid::ComputeMarkerData), decltype(&CMarker::ComputeMarkerData)>::value, "CMarkerNodeRigid declares Position and must implement ComputeMarkerData; the default of CMarker raises");
static_assert(!std::is_same<decltype(&CMarkerNodeRigid::GetRotationMatrix), decltype(&CMarker::GetRotationMatrix)>::value, "CMarkerNodeRigid declares Orientation and must implement GetRotationMatrix; the default of CMarker raises");
static_assert(!std::is_same<decltype(&CMarkerNodeRigid::GetAngularVelocity), decltype(&CMarker::GetAngularVelocity)>::value, "CMarkerNodeRigid declares Orientation and must implement GetAngularVelocity; the default of CMarker raises");
static_assert(!std::is_same<decltype(&CMarkerNodeRigid::GetAngularVelocityLocal), decltype(&CMarker::GetAngularVelocityLocal)>::value, "CMarkerNodeRigid declares Orientation and must implement GetAngularVelocityLocal; the default of CMarker raises");
static_assert(!std::is_same<decltype(&CMarkerNodeRigid::ComputeMarkerDataJacobianDerivative), decltype(&CMarker::ComputeMarkerDataJacobianDerivative)>::value, "CMarkerNodeRigid declares JacobianDerivativeAvailable and must implement ComputeMarkerDataJacobianDerivative; the default of CMarker raises");

#endif //#ifdef include once...
