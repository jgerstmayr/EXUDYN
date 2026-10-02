/** ***********************************************************************************************
* @class        CMarkerBodyRigidParameters
* @brief        Parameter class for CMarkerBodyRigid
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-03  00:33:15 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef CMARKERBODYRIGIDPARAMETERS__H
#define CMARKERBODYRIGIDPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CMarkerBodyRigidParameters
class CMarkerBodyRigidParameters // AUTO:
{
public: // AUTO:
    Index bodyNumber;                             //!< AUTO: body number to which marker is attached to
    HomogeneousTransformation localHT;            //!< AUTO: the frame of the marker in the body frame, as homogeneous transformation: its translation is localPosition, its rotation \f$\LU{bm}{\Rot}\f$ turns the marker frame against the body; a 4x4 matrix, its 16 values row by row or an exu.HT; None: not given; given together with localPosition, both must agree
    //! AUTO: default constructor with parameter initialization
    CMarkerBodyRigidParameters()
    {
        bodyNumber = EXUstd::InvalidIndex;
        localHT = HomogeneousTransformation();
    };
};


/** ***********************************************************************************************
* @class        CMarkerBodyRigid
* @brief        A rigid-body (position+orientation) body-marker attached to a local (body-fixed) position \f$\pLocB = [b_0,\; b_1,\; b_2]\f$ (\f$x\f$, \f$y\f$, and \f$z\f$ coordinates) of the body. It provides position and orientation (rotation), as well as the according derivatives. It can be used for most connectors, joints or loads where either position, position and orientation, or orientation are required.
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

//! AUTO: CMarkerBodyRigid
class CMarkerBodyRigid: public CMarker // AUTO:
{
protected: // AUTO:
    CMarkerBodyRigidParameters parameters; //! AUTO: contains all parameters for CMarkerBodyRigid

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerBodyRigidParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerBodyRigidParameters& GetParameters() const { return parameters; }

    //! AUTO:  general access to object number
    virtual Index GetObjectNumber(Index localIndex = 0) const override
    {
        return parameters.bodyNumber;
    }

    //! AUTO:  change bodyNumber
    virtual void SetObjectNumber(Index objectNumber, Index localIndex = 0) override
    {
        parameters.bodyNumber = objectNumber;
    }

    //! AUTO:  general access to object number
    virtual Index GetNumberOfObjects() const override
    {
        return 1;
    }

    //! AUTO:  return marker type (for body treatment in computation)
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)((Index)Marker::Body + (Index)Marker::Object + (Index)Marker::Position + (Index)Marker::Orientation + (Index)Marker::JacobianDerivativeAvailable + (Index)Marker::JacobianDerivativeNonZero);
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

    //! AUTO:  frame and velocities through the body, without the Jacobians; the body keeps in temp what it needs for AddGeneralizedForceTorque (#2745)
    virtual Index GetKinematicsRigid(const CSystemData& cSystemData, MarkerRigid<Real>& kinematics, MarkerTemp& temp) const override;

    //! AUTO:  add J_pos^T force + J_rot^T torque at the local position to ode2Lhs, through the body (#2745)
    virtual void AddGeneralizedForceTorque(const CSystemData& cSystemData, const Vector3D& force, const Vector3D& torque, MarkerTemp& temp, LinkedDataVector& ode2Lhs) const override;

    //! AUTO:  the local position on the body, for the check of IsValidLocalPosition at Assemble() (#2744)
    virtual bool GetLocalPosition(Vector3D& localPosition) const override
    {
        localPosition = parameters.localHT.GetTranslation(); return true;
    }

};



#endif //#ifdef include once...
