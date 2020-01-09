/** ***********************************************************************************************
* @class        CMarkerBodyCable2DCoordinatesParameters
* @brief        Parameter class for CMarkerBodyCable2DCoordinates
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


//! AUTO: Parameters for class CMarkerBodyCable2DCoordinatesParameters
class CMarkerBodyCable2DCoordinatesParameters // AUTO: 
{
public: // AUTO: 
    Index bodyNumber;                             //!< AUTO: body number to which marker is attached to
    //! AUTO: default constructor with parameter initialization
    CMarkerBodyCable2DCoordinatesParameters()
    {
        bodyNumber = EXUstd::InvalidIndex;
    };
};


/** ***********************************************************************************************
* @class        CMarkerBodyCable2DCoordinates
* @brief        A special Marker attached to the coordinates of a 2D ANCF beam finite element with cubic interpolation.
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

//! AUTO: CMarkerBodyCable2DCoordinates
class CMarkerBodyCable2DCoordinates: public CMarker // AUTO: 
{
protected: // AUTO: 
    CMarkerBodyCable2DCoordinatesParameters parameters; //! AUTO: contains all parameters for CMarkerBodyCable2DCoordinates

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerBodyCable2DCoordinatesParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerBodyCable2DCoordinatesParameters& GetParameters() const { return parameters; }

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

    //! AUTO:  return dimension of connector, which an attached connector would have
    virtual Index GetDimension() const override
    {
        return 2;
    }

    //! AUTO:  return position of marker -> axis-midpoint of ANCF cable
    virtual void GetPosition(const CSystemData& cSystemData, Vector3D& position, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  Compute marker data (e.g. position and positionJacobian) for a marker
    virtual void ComputeMarkerData(const CSystemData& cSystemData, bool computeJacobian, MarkerData& markerData) const override;

};


