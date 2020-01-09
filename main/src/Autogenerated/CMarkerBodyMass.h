/** ***********************************************************************************************
* @class        CMarkerBodyMassParameters
* @brief        Parameter class for CMarkerBodyMass
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


//! AUTO: Parameters for class CMarkerBodyMassParameters
class CMarkerBodyMassParameters // AUTO: 
{
public: // AUTO: 
    Index bodyNumber;                             //!< AUTO: body number to which marker is attached to
    //! AUTO: default constructor with parameter initialization
    CMarkerBodyMassParameters()
    {
        bodyNumber = EXUstd::InvalidIndex;
    };
};


/** ***********************************************************************************************
* @class        CMarkerBodyMass
* @brief        A marker attached to the body mass; use this marker to apply a body-load (e.g. gravitational force).
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

//! AUTO: CMarkerBodyMass
class CMarkerBodyMass: public CMarker // AUTO: 
{
protected: // AUTO: 
    CMarkerBodyMassParameters parameters; //! AUTO: contains all parameters for CMarkerBodyMass

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerBodyMassParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerBodyMassParameters& GetParameters() const { return parameters; }

    //! AUTO:  general access to object number
    virtual Index GetObjectNumber() const override
    {
        return parameters.bodyNumber;
    }

    //! AUTO:  return marker type (for node treatment in computation)
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)(Marker::Body + Marker::Object + Marker::BodyMass);
    }

    //! AUTO:  return dimension of connector, which an attached connector would have
    virtual Index GetDimension() const override
    {
        return 3;
    }

    //! AUTO:  return position of marker at local position (0,0,0) of the body
    virtual void GetPosition(const CSystemData& cSystemData, Vector3D& position, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  Compute marker data (e.g. position and positionJacobian) for a marker
    virtual void ComputeMarkerData(const CSystemData& cSystemData, bool computeJacobian, MarkerData& markerData) const override;

};


