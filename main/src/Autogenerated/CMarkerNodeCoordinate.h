/** ***********************************************************************************************
* @class        CMarkerNodeCoordinateParameters
* @brief        Parameter class for CMarkerNodeCoordinate
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-12-22  19:35:18 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CMARKERNODECOORDINATEPARAMETERS__H
#define CMARKERNODECOORDINATEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CMarkerNodeCoordinateParameters
class CMarkerNodeCoordinateParameters // AUTO: 
{
public: // AUTO: 
    Index nodeNumber;                             //!< AUTO: node number to which marker is attached to
    Index coordinate;                             //!< AUTO: coordinate of node to which marker is attached to
    //! AUTO: default constructor with parameter initialization
    CMarkerNodeCoordinateParameters()
    {
        nodeNumber = EXUstd::InvalidIndex;
        coordinate = EXUstd::InvalidIndex;
    };
};


/** ***********************************************************************************************
* @class        CMarkerNodeCoordinate
* @brief        A node-Marker attached to a \hac{ODE2} coordinate of a node; this marker allows to connect a coordinate-based constraint or connector to a nodal coordinate (also NodeGround); for \hac{ODE1} coordinates use MarkerNodeODE1Coordinate.
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

//! AUTO: CMarkerNodeCoordinate
class CMarkerNodeCoordinate: public CMarker // AUTO: 
{
protected: // AUTO: 
    CMarkerNodeCoordinateParameters parameters; //! AUTO: contains all parameters for CMarkerNodeCoordinate

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerNodeCoordinateParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerNodeCoordinateParameters& GetParameters() const { return parameters; }

    //! AUTO:  access to node number
    virtual Index GetNodeNumber() const override
    {
        return parameters.nodeNumber;
    }

    //! AUTO:  access to coordinate index
    virtual Index GetCoordinateNumber() const override
    {
        return parameters.coordinate;
    }

    //! AUTO:  return marker type (for node treatment in computation)
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)(Marker::Node + Marker::Coordinate + Marker::JacobianDerivativeAvailable);
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

    //! AUTO:  fill in according data for derivative of jacobian times vector v6D, e.g.: d(Jpos.T @ v6D[0:3])/dq; v6D represents 3 force components and 3 torque components in global coordinates!
    virtual void ComputeMarkerDataJacobianDerivative(const CSystemData& cSystemData, const Vector6D& v6D, MarkerData& markerData) const override;

};



#endif //#ifdef include once...
