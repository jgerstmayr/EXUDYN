/** ***********************************************************************************************
* @class        CMarkerNodePositionParameters
* @brief        Parameter class for CMarkerNodePosition
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-09-08  18:14:40 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CMARKERNODEPOSITIONPARAMETERS__H
#define CMARKERNODEPOSITIONPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CMarkerNodePositionParameters
class CMarkerNodePositionParameters // AUTO: 
{
public: // AUTO: 
    Index nodeNumber;                             //!< AUTO: node number to which marker is attached to
    //! AUTO: default constructor with parameter initialization
    CMarkerNodePositionParameters()
    {
        nodeNumber = EXUstd::InvalidIndex;
    };
};


/** ***********************************************************************************************
* @class        CMarkerNodePosition
* @brief        A node-Marker attached to a position-based node.
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

//! AUTO: CMarkerNodePosition
class CMarkerNodePosition: public CMarker // AUTO: 
{
protected: // AUTO: 
    CMarkerNodePositionParameters parameters; //! AUTO: contains all parameters for CMarkerNodePosition

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerNodePositionParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerNodePositionParameters& GetParameters() const { return parameters; }

    //! AUTO:  general access to node number
    virtual Index GetNodeNumber() const override
    {
        return parameters.nodeNumber;
    }

    //! AUTO:  return marker type (for node treatment in computation)
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)(Marker::Node + Marker::Position);
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



#endif //#ifdef include once...
