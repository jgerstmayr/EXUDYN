/** ***********************************************************************************************
* @class        CMarkerNodeRotationCoordinateParameters
* @brief        Parameter class for CMarkerNodeRotationCoordinate
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-05-28  00:49:17 (last modfied)
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


//! AUTO: Parameters for class CMarkerNodeRotationCoordinateParameters
class CMarkerNodeRotationCoordinateParameters // AUTO: 
{
public: // AUTO: 
    Index nodeNumber;                             //!< AUTO: node number to which marker is attached to
    Index rotationCoordinate;                     //!< AUTO: rotation coordinate: 0=x, 1=y, 2=z
    //! AUTO: default constructor with parameter initialization
    CMarkerNodeRotationCoordinateParameters()
    {
        nodeNumber = EXUstd::InvalidIndex;
        rotationCoordinate = EXUstd::InvalidIndex;
    };
};


/** ***********************************************************************************************
* @class        CMarkerNodeRotationCoordinate
* @brief        A node-Marker attached to a a node containing rotation; the Marker measures a rotation coordinate (Tait-Bryan angles) or angular velocities on the velocity level
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

//! AUTO: CMarkerNodeRotationCoordinate
class CMarkerNodeRotationCoordinate: public CMarker // AUTO: 
{
protected: // AUTO: 
    CMarkerNodeRotationCoordinateParameters parameters; //! AUTO: contains all parameters for CMarkerNodeRotationCoordinate

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerNodeRotationCoordinateParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerNodeRotationCoordinateParameters& GetParameters() const { return parameters; }

    //! AUTO:  access to node number
    virtual Index GetNodeNumber() const override
    {
        return parameters.nodeNumber;
    }

    //! AUTO:  access to coordinate index
    Index GetRotationCoordinateNumber() const
    {
        return parameters.rotationCoordinate;
    }

    //! AUTO:  return marker type (for node treatment in computation)
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)(Marker::Node + Marker::Coordinate + Marker::Orientation);
    }

    //! AUTO:  return dimension of connector, which an attached connector would have; for coordinate markers, it gives the number of coordinates used by the marker
    virtual Index GetDimension(const CSystemData& cSystemData) const override
    {
        return 1;
    }

    //! AUTO:  return position of marker
    virtual void GetPosition(const CSystemData& cSystemData, Vector3D& position, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  Compute marker data (e.g. position and positionJacobian) for a marker
    virtual void ComputeMarkerData(const CSystemData& cSystemData, bool computeJacobian, MarkerData& markerData) const override;

};


