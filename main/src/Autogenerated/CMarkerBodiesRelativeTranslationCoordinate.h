/** ***********************************************************************************************
* @class        CMarkerBodiesRelativeTranslationCoordinateParameters
* @brief        Parameter class for CMarkerBodiesRelativeTranslationCoordinate
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-02-05  22:16:39 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CMARKERBODIESRELATIVETRANSLATIONCOORDINATEPARAMETERS__H
#define CMARKERBODIESRELATIVETRANSLATIONCOORDINATEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CMarkerBodiesRelativeTranslationCoordinateParameters
class CMarkerBodiesRelativeTranslationCoordinateParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex bodyNumbers;                       //!< AUTO: list of body numbers for which relative coordinate is computed
    Vector3D localPosition0;                      //!< AUTO: local position on body 0; i.e. local (body-fixed) position where position is measured and force is applied to
    Vector3D localPosition1;                      //!< AUTO: local position on body 1; i.e. local (body-fixed) position where position is measured and force is applied to
    Vector3D axis0;                               //!< AUTO: axis defined in body 0, along which the relative translation is measured
    Real offset;                                  //!< AUTO: translation offset [SI:m] subtracted from the translation; can be used to change the zero position
    //! AUTO: default constructor with parameter initialization
    CMarkerBodiesRelativeTranslationCoordinateParameters()
    {
        bodyNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        localPosition0 = Vector3D({0.,0.,0.});
        localPosition1 = Vector3D({0.,0.,0.});
        axis0 = Vector3D({1.,0.,0.});
        offset = 0.;
    };
};


/** ***********************************************************************************************
* @class        CMarkerBodiesRelativeTranslationCoordinate
* @brief        A coordinate-based Marker attached to two rigid bodies or beams which computes the relative translation between the bodies according to the given axis. This marker can be used together with coordinate-based constraints and connectors (e.g., CoordinateSpringDamper and CoordinateConstraint). NOTE: it is assumed that the two bodies can only move along the given axis (e.g., constrained by a prismatic joint) -- otherwise results may be unexpected. NOTE: this approach is not compatible with FFRF-based flexible bodies and currently requires and intermediate rigid body.
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

//! AUTO: CMarkerBodiesRelativeTranslationCoordinate
class CMarkerBodiesRelativeTranslationCoordinate: public CMarker // AUTO: 
{
protected: // AUTO: 
    CMarkerBodiesRelativeTranslationCoordinateParameters parameters; //! AUTO: contains all parameters for CMarkerBodiesRelativeTranslationCoordinate

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerBodiesRelativeTranslationCoordinateParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerBodiesRelativeTranslationCoordinateParameters& GetParameters() const { return parameters; }

    //! AUTO:  general access to object number
    virtual Index GetObjectNumber(Index localIndex = 0) const override
    {
        return parameters.bodyNumbers[localIndex];
    }

    //! AUTO:  change bodyNumber
    virtual void SetObjectNumber(Index objectNumber, Index localIndex = 0) override
    {
        parameters.bodyNumbers[localIndex] = objectNumber;
    }

    //! AUTO:  general access to object number
    virtual Index GetNumberOfObjects() const override
    {
        return 2;
    }

    //! AUTO:  return marker type (for body treatment in computation)
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)(Marker::Body + Marker::Object + Marker::Coordinate + Marker::Position + Marker::Orientation
        //+ Marker::JacobianDerivativeAvailable + Marker::JacobianDerivativeNonZero//neglected for now
            );
    }

    //! AUTO:  return dimension of connector, which an attached connector would have; for coordinate markers, it gives the number of coordinates used by the marker
    virtual Index GetDimension(const CSystemData& cSystemData) const override
    {
        return 1;
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
