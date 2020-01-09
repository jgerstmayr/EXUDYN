/** ***********************************************************************************************
* @class        CMarkerBodyRigidParameters
* @brief        Parameter class for CMarkerBodyRigid
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2019-11-12  21:51:18 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"


//! AUTO: Parameters for class CMarkerBodyRigidParameters
class CMarkerBodyRigidParameters // AUTO: 
{
public: // AUTO: 
    Index bodyNumber;                             //!< AUTO: body number to which marker is attached to
    Vector3D localPosition;                       //!< AUTO: local body position of marker; e.g. local (body-fixed) position where force is applied to
    bool bodyFixed;                               //!< AUTO: if bodyFixed is true, the force/sensor is using body-fixed coordinates (orientation); otherwise, it uses global coordinates
    //! AUTO: default constructor with parameter initialization
    CMarkerBodyRigidParameters()
    {
        bodyNumber = EXUstd::InvalidIndex;
        localPosition = Vector3D({0.,0.,0.});
        bodyFixed = false;
    };
};


/** ***********************************************************************************************
* @class        CMarkerBodyRigid
* @brief        A rigid-body (position+orientation) body-marker attached to local position (x,y,z) of the body.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

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
    virtual Index GetObjectNumber() const override
    {
        return parameters.bodyNumber;
    }

    //! AUTO:  return marker type (for node treatment in computation)
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)(Marker::Body + Marker::Object + Marker::Position + Marker::Orientation);
    }

    //! AUTO:  return dimension of connector, which an attached connector would have
    virtual Index GetDimension() const override
    {
        return 3;
    }

    //! AUTO:  return configuration dependent rotation matrix of node; returns always a 3D Matrix
    virtual void GetRotationMatrix(const CSystemData& cSystemData, Matrix3D& rotationMatrix, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent angular velocity of node; returns always a 3D Vector
    virtual void GetAngularVelocity(const CSystemData& cSystemData, Vector3D& angularVelocity, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent local (=body fixed) angular velocity of node; returns always a 3D Vector
    virtual void GetAngularVelocityLocal(const CSystemData& cSystemData, Vector3D& angularVelocity, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return position of marker
    virtual void GetPosition(const CSystemData& cSystemData, Vector3D& position, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return velocity of marker
    virtual void GetVelocity(const CSystemData& cSystemData, Vector3D& velocity, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  Compute marker data (e.g. position and positionJacobian) for a marker
    virtual void ComputeMarkerData(const CSystemData& cSystemData, bool computeJacobian, MarkerData& markerData) const override;

};


