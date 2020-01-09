/** ***********************************************************************************************
* @class        CMarkerBodyPositionParameters
* @brief        Parameter class for CMarkerBodyPosition
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


//! AUTO: Parameters for class CMarkerBodyPositionParameters
class CMarkerBodyPositionParameters // AUTO: 
{
public: // AUTO: 
    Index bodyNumber;                             //!< AUTO: body number to which marker is attached to
    Vector3D localPosition;                       //!< AUTO: local body position of marker; e.g. local (body-fixed) position where force is applied to
    bool bodyFixed;                               //!< AUTO: if bodyFixed is true, the force/sensor is using body-fixed coordinates (orientation); otherwise, it uses global coordinates
    //! AUTO: default constructor with parameter initialization
    CMarkerBodyPositionParameters()
    {
        bodyNumber = EXUstd::InvalidIndex;
        localPosition = Vector3D({0.,0.,0.});
        bodyFixed = false;
    };
};


/** ***********************************************************************************************
* @class        CMarkerBodyPosition
* @brief        A position body-marker attached to local position (x,y,z) of the body.
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

//! AUTO: CMarkerBodyPosition
class CMarkerBodyPosition: public CMarker // AUTO: 
{
protected: // AUTO: 
    CMarkerBodyPositionParameters parameters; //! AUTO: contains all parameters for CMarkerBodyPosition

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerBodyPositionParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerBodyPositionParameters& GetParameters() const { return parameters; }

    //! AUTO:  general access to object number
    virtual Index GetObjectNumber() const override
    {
        return parameters.bodyNumber;
    }

    //! AUTO:  return marker type (for node treatment in computation)
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)(Marker::Body + Marker::Object + Marker::Position);
    }

    //! AUTO:  return dimension of connector, which an attached connector would have
    virtual Index GetDimension() const override
    {
        return 3;
    }

    //! AUTO:  return position of marker
    virtual void GetPosition(const CSystemData& cSystemData, Vector3D& position, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return velocity of marker
    virtual void GetVelocity(const CSystemData& cSystemData, Vector3D& velocity, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  Compute marker data (e.g. position and positionJacobian) for a marker
    virtual void ComputeMarkerData(const CSystemData& cSystemData, bool computeJacobian, MarkerData& markerData) const override;

};


