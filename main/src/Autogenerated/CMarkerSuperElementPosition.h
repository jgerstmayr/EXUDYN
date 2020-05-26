/** ***********************************************************************************************
* @class        CMarkerSuperElementPositionParameters
* @brief        Parameter class for CMarkerSuperElementPosition
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-05-25  00:47:35 (last modfied)
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


//! AUTO: Parameters for class CMarkerSuperElementPositionParameters
class CMarkerSuperElementPositionParameters // AUTO: 
{
public: // AUTO: 
    Index bodyNumber;                             //!< AUTO: body number to which marker is attached to
    ArrayIndex meshNodeNumbers;                   //!< AUTO: mesh node numbers of superelement which are used to compute the body-fixed marker position; the related nodes must provide 3D position information, such as NodePoint, NodePoint2D, NodeRigidBody[..]; in order to retrieve the global node number, the generic body needs to convert local into global node numbers
    Vector weightingFactors;                      //!< AUTO: weighting factors per node to compute the final local position
    //! AUTO: default constructor with parameter initialization
    CMarkerSuperElementPositionParameters()
    {
        bodyNumber = EXUstd::InvalidIndex;
        meshNodeNumbers = ArrayIndex();
        weightingFactors = Vector();
    };
};


/** ***********************************************************************************************
* @class        CMarkerSuperElementPosition
* @brief        A position marker attached to a SuperElement, such as ObjectFFRF, ObjectGenericODE2 and ObjectFFRFreducedOrder (for which it is inefficient!!!). The marker acts on the mesh nodes, not on the underlying nodes of the object. For a list of \f$n\f$ mesh node numbers, referencing to mesh node points \f$\LU{b}{\pv_i}\f$ and weights \f$w_i\f$, the body-fixed marker position \f$\LU{b}{\pv_m}\f$ results in \f$\LU{b}{\pv_m} = \sum_{i=0}^{n-1}w_i \cdot \LU{b}{\pv_i}\f$. EXAMPLE for single node marker on body 4, mesh node 10: MarkerSuperElementPosition(bodyNumber=4, meshNodeNumber=[10], weightingFactors=[1])
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

//! AUTO: CMarkerSuperElementPosition
class CMarkerSuperElementPosition: public CMarker // AUTO: 
{
protected: // AUTO: 
    CMarkerSuperElementPositionParameters parameters; //! AUTO: contains all parameters for CMarkerSuperElementPosition

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerSuperElementPositionParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerSuperElementPositionParameters& GetParameters() const { return parameters; }

    //! AUTO:  general access to object number
    virtual Index GetObjectNumber() const override
    {
        return parameters.bodyNumber;
    }

    //! AUTO:  return marker type (for node treatment in computation)
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)(Marker::Body + Marker::Object + Marker::Position + Marker::SuperElement);
    }

    //! AUTO:  return dimension of connector, which an attached connector would have; for coordinate markers, it gives the number of coordinates used by the marker
    virtual Index GetDimension(const CSystemData& cSystemData) const override
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


