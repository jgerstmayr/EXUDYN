/** ***********************************************************************************************
* @class        CMarkerBodyCable2DShapeParameters
* @brief        Parameter class for CMarkerBodyCable2DShape
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-06-01  20:10:12 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"


//! AUTO: Parameters for class CMarkerBodyCable2DShapeParameters
class CMarkerBodyCable2DShapeParameters // AUTO: 
{
public: // AUTO: 
    Index bodyNumber;                             //!< AUTO: body number to which marker is attached to
    Index numberOfSegments;                       //!< AUTO: number of number of segments; each segment is a line and is associated to a data (history) variable; must be same as in according contact element
    //! AUTO: default constructor with parameter initialization
    CMarkerBodyCable2DShapeParameters()
    {
        bodyNumber = EXUstd::InvalidIndex;
        numberOfSegments = 3;
    };
};


/** ***********************************************************************************************
* @class        CMarkerBodyCable2DShape
* @brief        A special Marker attached to a 2D ANCF beam finite element with cubic interpolation and 8 coordinates.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

//! AUTO: CMarkerBodyCable2DShape
class CMarkerBodyCable2DShape: public CMarker // AUTO: 
{
protected: // AUTO: 
    static const Index maxNumberOfSegments = 12; //maximum number of contact segments
    CMarkerBodyCable2DShapeParameters parameters; //! AUTO: contains all parameters for CMarkerBodyCable2DShape

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerBodyCable2DShapeParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerBodyCable2DShapeParameters& GetParameters() const { return parameters; }

    //! AUTO:  general access to object number
    virtual Index GetObjectNumber() const override
    {
        return parameters.bodyNumber;
    }

    //! AUTO:  return marker type (for node treatment in computation)
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)(Marker::Body + Marker::Object + Marker::Coordinate);
    }

    //! AUTO:  return dimension of connector, which an attached connector would have; for coordinate markers, it gives the number of coordinates used by the marker
    virtual Index GetDimension(const CSystemData& cSystemData) const override
    {
        return 2;
    }

    //! AUTO:  return position of marker -> axis-midpoint of ANCF cable
    virtual void GetPosition(const CSystemData& cSystemData, Vector3D& position, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  Compute marker data (e.g. position and positionJacobian) for a marker
    virtual void ComputeMarkerData(const CSystemData& cSystemData, bool computeJacobian, MarkerData& markerData) const override;

};


