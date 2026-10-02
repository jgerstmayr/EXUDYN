/** ***********************************************************************************************
* @class        CMarkerKinematicTreeRigidParameters
* @brief        Parameter class for CMarkerKinematicTreeRigid
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-03  00:32:23 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef CMARKERKINEMATICTREERIGIDPARAMETERS__H
#define CMARKERKINEMATICTREERIGIDPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CMarkerKinematicTreeRigidParameters
class CMarkerKinematicTreeRigidParameters // AUTO:
{
public: // AUTO:
    Index objectNumber;                           //!< AUTO: body number to which marker is attached to
    Index linkNumber;                             //!< AUTO: must be >= 0; number of link in KinematicTree to which marker is attached to
    HomogeneousTransformation localHT;            //!< AUTO: the frame of the marker in the link frame, as homogeneous transformation: its translation is localPosition, its rotation turns the marker frame against the link; a 4x4 matrix, its 16 values row by row or an exu.HT; None: not given; given together with localPosition, both must agree
    //! AUTO: default constructor with parameter initialization
    CMarkerKinematicTreeRigidParameters()
    {
        objectNumber = EXUstd::InvalidIndex;
        linkNumber = EXUstd::InvalidIndex;
        localHT = HomogeneousTransformation();
    };
};


/** ***********************************************************************************************
* @class        CMarkerKinematicTreeRigid
* @brief        A position and orientation (rigid-body) marker attached to a kinematic tree. The marker is attached to the ObjectKinematicTree object and additionally needs a link number as well as a local position, similar to the SensorKinematicTree. The marker allows to attach loads (LoadForceVector and LoadTorqueVector) at arbitrary links or position. It also allows to attach connectors (e.g., spring dampers or actuators) to the kinematic tree. Finally, joint constraints can be attached, which allows for realization of closed loop structures. NOTE, however, that it is less efficient to attach many markers to a kinematic tree, therefor for forces or joint control use the structures available in kinematic tree whenever possible.
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

//! AUTO: CMarkerKinematicTreeRigid
class CMarkerKinematicTreeRigid: public CMarker // AUTO:
{
protected: // AUTO:
    CMarkerKinematicTreeRigidParameters parameters; //! AUTO: contains all parameters for CMarkerKinematicTreeRigid

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerKinematicTreeRigidParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerKinematicTreeRigidParameters& GetParameters() const { return parameters; }

    //! AUTO:  general access to object number
    virtual Index GetObjectNumber(Index localIndex = 0) const override
    {
        return parameters.objectNumber;
    }

    //! AUTO:  change bodyNumber
    virtual void SetObjectNumber(Index objectNumber, Index localIndex = 0) override
    {
        parameters.objectNumber = objectNumber;
    }

    //! AUTO:  general access to object number
    virtual Index GetNumberOfObjects() const override
    {
        return 1;
    }

    //! AUTO:  return marker type (for node treatment in computation)
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)((Index)Marker::Body + (Index)Marker::Object + (Index)Marker::Position + (Index)Marker::Orientation + (Index)Marker::KinematicTree);
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

};



#endif //#ifdef include once...
