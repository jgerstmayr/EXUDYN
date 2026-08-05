/** ***********************************************************************************************
* @class        CMarkerBodyBeamShapeParameters
* @brief        Parameter class for CMarkerBodyBeamShape
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-03-02  09:09:18 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CMARKERBODYBEAMSHAPEPARAMETERS__H
#define CMARKERBODYBEAMSHAPEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CMarkerBodyBeamShapeParameters
class CMarkerBodyBeamShapeParameters // AUTO: 
{
public: // AUTO: 
    Index bodyNumber;                             //!< AUTO: body number to which marker is attached to (beam type)
    //! AUTO: default constructor with parameter initialization
    CMarkerBodyBeamShapeParameters()
    {
        bodyNumber = EXUstd::InvalidIndex;
    };
};


/** ***********************************************************************************************
* @class        CMarkerBodyBeamShape
* @brief        A special Marker attached to a 3D beam finite element which provides at least position and tangent to the beam axis.
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

//! AUTO: CMarkerBodyBeamShape
class CMarkerBodyBeamShape: public CMarker // AUTO: 
{
protected: // AUTO: 
    CMarkerBodyBeamShapeParameters parameters; //! AUTO: contains all parameters for CMarkerBodyBeamShape

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerBodyBeamShapeParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerBodyBeamShapeParameters& GetParameters() const { return parameters; }

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

    //! AUTO:  general access to local object number
    virtual Index GetNumberOfObjects() const override
    {
        return 1;
    }

    //! AUTO:  return marker type
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)(Marker::Body + Marker::Object + Marker::Beam3DShape + Marker::JacobianDerivativeAvailable);
    }

    //! AUTO:  return dimension of connector, which an attached connector would have; for coordinate markers, it gives the number of coordinates used by the marker
    virtual Index GetDimension(const CSystemData& cSystemData) const override
    {
        return 3;
    }

    //! AUTO:  return position of marker -> axis-midpoint of beam element; mostly for drawing
    virtual void GetPosition(const CSystemData& cSystemData, Vector3D& position, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  Compute marker data (e.g. position and positionJacobian, etc.) for a marker
    virtual void ComputeMarkerData(const CSystemData& cSystemData, bool computeJacobian, MarkerData& markerData) const override;

    //! AUTO:  fill in according data for derivative of jacobian times vector v6D, e.g.: d(Jpos.T @ v6D[0:3])/dq; v6D represents 3 force components and 3 torque components in global coordinates!
    virtual void ComputeMarkerDataJacobianDerivative(const CSystemData& cSystemData, const Vector6D& v6D, MarkerData& markerData) const override;

    //! AUTO:  Compute all data for sliding joint computations
    static void ComputeSlidingJointData(Real xBeam, Real lBeam, const ResizableVector& totalCoordinates, const ResizableVector& totalCoordinates_t, Vector3D& position, Vector3D& slopeVector, Vector3D& slopeVector_x, bool& beamHasTorsion, ConfigurationType configuration=ConfigurationType::Current);

};



#endif //#ifdef include once...
