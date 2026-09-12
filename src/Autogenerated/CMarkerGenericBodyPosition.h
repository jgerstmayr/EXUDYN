/** ***********************************************************************************************
* @class        CMarkerGenericBodyPositionParameters
* @brief        Parameter class for CMarkerGenericBodyPosition
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-04-10  10:39:20 (last modfied)
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


//! AUTO: Parameters for class CMarkerGenericBodyPositionParameters
class CMarkerGenericBodyPositionParameters // AUTO: 
{
public: // AUTO: 
    Index bodyNumber;                             //!< AUTO: body number to which marker is attached to
    ArrayIndex nodeNumbers;                       //!< AUTO: local node numbers of body which are used to compute the body-fixed marker position; the related nodes must provide 3D position information, such as NodePoint, NodePoint2D, NodeRigidBody[..]; in order to retrieve the global node number, the generic body needs to convert local into global node numbers
    Vector weightingFactors;                      //!< AUTO: weighting factors per node to compute the final local position
    bool useFirstNodeAsReferenceFrame;            //!< AUTO: if true, the first node of the body is used to transform the nodal coordinates from local (body-fixed) to global coordinates, which MUST provide position and orientation information; this is according to the floating frame of reference formulation (ffrf)
    //! AUTO: default constructor with parameter initialization
    CMarkerGenericBodyPositionParameters()
    {
        bodyNumber = EXUstd::InvalidIndex;
        nodeNumbers = ArrayIndex();
        weightingFactors = Vector();
        useFirstNodeAsReferenceFrame = false;
    };
};


/** ***********************************************************************************************
* @class        CMarkerGenericBodyPosition
* @brief        A position marker attached to a generic, discretized body, such as GenericODE2 or bodies modelled with the floating frame of reference formulation. The nodes of the body must provide position information. For a list of \f$n\f$ local node numbers, referencing to node points \f$\LU{b}{\pv_i}\f$ and weights \f$w_i\f$, the body-fixed marker position \f$\LU{b}{\pv_m}\f$ results in \f$\LU{b}{\pv_m} = \sum_{i=0}^{n-1}w_i \cdot \LU{b}{\pv_i}\f$. If the flag \texttt{useFirstNodeAsReferenceFrame} = \texttt{False}, then it follows that \f$\LU{0}{\pv_m} = \LU{b}{\pv_m}\f$. Otherwise \f$\LU{0}{\pv_m} = \LU{0b}{\Rot} \LU{b}{\pv_m}\f$, in which \f$\LU{0b}{\Rot}\f$ is the rotation matrix provided by the first node of the body, which also must provide orientation information.
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

//! AUTO: CMarkerGenericBodyPosition
class CMarkerGenericBodyPosition: public CMarker // AUTO: 
{
protected: // AUTO: 
    CMarkerGenericBodyPositionParameters parameters; //! AUTO: contains all parameters for CMarkerGenericBodyPosition

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerGenericBodyPositionParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerGenericBodyPositionParameters& GetParameters() const { return parameters; }

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


