/** ***********************************************************************************************
* @class        CMarkerNodeCoordinatesParameters
* @brief        Parameter class for CMarkerNodeCoordinates
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-11-15  23:04:42 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CMARKERNODECOORDINATESPARAMETERS__H
#define CMARKERNODECOORDINATESPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CMarkerNodeCoordinatesParameters
class CMarkerNodeCoordinatesParameters // AUTO: 
{
public: // AUTO: 
    Index nodeNumber;                             //!< AUTO: node number to which marker is attached to
    //! AUTO: default constructor with parameter initialization
    CMarkerNodeCoordinatesParameters()
    {
        nodeNumber = EXUstd::InvalidIndex;
    };
};


/** ***********************************************************************************************
* @class        CMarkerNodeCoordinates
* @brief        A node-Marker attached to all \hac{ODE2} coordinates of a node; IN CONTRAST DOT MarkerNodeCoordinate, the marker coordinates INCLUDE the reference values!; this marker allows connecting a coordinate-based constraint or connector to a nodal coordinate (also NodeGround); for \hac{ODE1} coordinates use MarkerNodeODE1Coordinates (under development).
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

//! AUTO: CMarkerNodeCoordinates
class CMarkerNodeCoordinates: public CMarker // AUTO: 
{
protected: // AUTO: 
    CMarkerNodeCoordinatesParameters parameters; //! AUTO: contains all parameters for CMarkerNodeCoordinates

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerNodeCoordinatesParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerNodeCoordinatesParameters& GetParameters() const { return parameters; }

    //! AUTO:  access to node number
    virtual Index GetNodeNumber() const override
    {
        return parameters.nodeNumber;
    }

    //! AUTO:  return marker type (for node treatment in computation)
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)(Marker::Node + Marker::Coordinates + Marker::JacobianDerivativeAvailable);
    }

    //! AUTO:  return dimension of connector, which an attached connector would have; for coordinate markers, it gives the number of coordinates used by the marker
    virtual Index GetDimension(const CSystemData& cSystemData) const override
    {
        return cSystemData.GetCNodes()[parameters.nodeNumber]->GetNumberOfODE2Coordinates();
    }

    //! AUTO:  return position of marker
    virtual void GetPosition(const CSystemData& cSystemData, Vector3D& position, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  Compute marker data (e.g. position and positionJacobian) for a marker
    virtual void ComputeMarkerData(const CSystemData& cSystemData, bool computeJacobian, MarkerData& markerData) const override;

    //! AUTO:  fill in according data for derivative of jacobian times vector v, e.g.: d(Jpos.T @ v)/dq
    virtual void ComputeMarkerDataJacobianDerivative(const CSystemData& cSystemData, const Vector& v, MarkerData& markerData) const override;

};



#endif //#ifdef include once...
