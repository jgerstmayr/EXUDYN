/** ***********************************************************************************************
* @class        CMarkerBodyCable2DShapeParameters
* @brief        Parameter class for CMarkerBodyCable2DShape
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2025-06-29  16:12:53 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CMARKERBODYCABLE2DSHAPEPARAMETERS__H
#define CMARKERBODYCABLE2DSHAPEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CMarkerBodyCable2DShapeParameters
class CMarkerBodyCable2DShapeParameters // AUTO: 
{
public: // AUTO: 
    Index bodyNumber;                             //!< AUTO: body number to which marker is attached to
    Index numberOfSegments;                       //!< AUTO: number of number of segments; each segment is a line and is associated to a data (history) variable; must be same as in according contact element
    Real verticalOffset;                          //!< AUTO: vertical offset from beam axis in positive (local) Y-direction; this offset accounts for consistent computation of positions and velocities at the surface of the beam
    //! AUTO: default constructor with parameter initialization
    CMarkerBodyCable2DShapeParameters()
    {
        bodyNumber = EXUstd::InvalidIndex;
        numberOfSegments = 3;
        verticalOffset = 0.;
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

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

//! AUTO: CMarkerBodyCable2DShape
class CMarkerBodyCable2DShape: public CMarker // AUTO: 
{
protected: // AUTO: 
    static constexpr Index maxNumberOfSegments = 12; //maximum number of contact segments
    CMarkerBodyCable2DShapeParameters parameters; //! AUTO: contains all parameters for CMarkerBodyCable2DShape

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerBodyCable2DShapeParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerBodyCable2DShapeParameters& GetParameters() const { return parameters; }

    //! AUTO:  general access to object number
    virtual Index GetObjectNumber(Index localIndex = 0) const override
    {
        return parameters.bodyNumber;
    }

    //! AUTO:  change bodyNumber
    virtual void SetObjectNumber(Index bodyNumber, Index localIndex = 0) override
    {
        parameters.bodyNumber = bodyNumber;
    }

    //! AUTO:  general access to object number
    virtual Index GetNumberOfObjects() const override
    {
        return 1;
    }

    //! AUTO:  return marker type (for node treatment in computation)
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)(Marker::Body + Marker::Object + Marker::Coordinate + Marker::JacobianDerivativeAvailable);
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

    //! AUTO:  fill in according data for derivative of jacobian times vector v6D, e.g.: d(Jpos.T @ v6D[0:3])/dq; v6D represents 3 force components and 3 torque components in global coordinates!
    virtual void ComputeMarkerDataJacobianDerivative(const CSystemData& cSystemData, const Vector6D& v6D, MarkerData& markerData) const override;

};



#endif //#ifdef include once...
