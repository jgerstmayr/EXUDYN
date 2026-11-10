/** ***********************************************************************************************
* @class        CMarkerNodeCoordinateParameters
* @brief        Parameter class for CMarkerNodeCoordinate
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-11-10  21:43:04 (last modified)
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
    Index coordinate;                             //!< AUTO: must be >= 0; coordinate of node to which marker is attached to
    bool includeReferenceCoordinate;              //!< AUTO: if True, the value of the marker is the coordinate plus its reference value, e.g. the absolute position of a NodePoint coordinate, as for the coordinate of a NodePointGround; if False, the coordinate alone, as the displacement of a NodePoint; the velocity and the Jacobian are the same in both cases (#990)
    //! AUTO: default constructor with parameter initialization
    CMarkerNodeCoordinateParameters()
    {
        nodeNumber = EXUstd::InvalidIndex;
        coordinate = EXUstd::InvalidIndex;
        includeReferenceCoordinate = false;
    };
};


/** ***********************************************************************************************
* @class        CMarkerNodeCoordinate
* @brief        A node-Marker attached to a ABRV:ODE2 coordinate of a node; this marker allows to connect a coordinate-based constraint or connector to a nodal coordinate (also NodeGround); for ABRV:ODE1 coordinates use `MarkerNodeODE1Coordinate`.
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

    //! AUTO:  change bodyNumber
    virtual void SetNodeNumber(Index nodeNumber) override
    {
        parameters.nodeNumber = nodeNumber;
    }

    //! AUTO:  access to coordinate index
    virtual Index GetCoordinateNumber() const override
    {
        return parameters.coordinate;
    }

    //! AUTO:  return marker type (for node treatment in computation)
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)((Index)Marker::Node + (Index)Marker::Coordinate + (Index)Marker::JacobianDerivativeAvailable);
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

    //! AUTO:  dv/dq and domega/dq of the marker at fixed velocities (#3201); false if not provided
    virtual bool GetVelocity_q(const CSystemData& cSystemData, Matrix& velocity_q, Matrix& angularVelocity_q) const override;

    //! AUTO:  the coordinate and its velocity, read from the node, without the marker data (#2745)
    virtual Index GetKinematicsCoordinate(const CSystemData& cSystemData, MarkerCoordinate<Real>& kinematics, MarkerTemp& temp) const override;

    //! AUTO:  add the force to the coordinate (#2745)
    virtual void AddGeneralizedForceCoordinate(const CSystemData& cSystemData, Real force, MarkerTemp& temp, LinkedDataVector& ode2Lhs) const override;

};


//! AUTO: the member functions that the declarations of CMarkerNodeCoordinate require (#2866)
#include <type_traits>
static_assert(!std::is_same<decltype(&CMarkerNodeCoordinate::ComputeMarkerData), decltype(&CMarker::ComputeMarkerData)>::value, "CMarkerNodeCoordinate declares Coordinate and must implement ComputeMarkerData; the default of CMarker raises");
static_assert(!std::is_same<decltype(&CMarkerNodeCoordinate::ComputeMarkerDataJacobianDerivative), decltype(&CMarker::ComputeMarkerDataJacobianDerivative)>::value, "CMarkerNodeCoordinate declares JacobianDerivativeAvailable and must implement ComputeMarkerDataJacobianDerivative; the default of CMarker raises");

#endif //#ifdef include once...
