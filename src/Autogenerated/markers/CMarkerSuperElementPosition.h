/** ***********************************************************************************************
* @class        CMarkerSuperElementPositionParameters
* @brief        Parameter class for CMarkerSuperElementPosition
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-02  19:23:07 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef CMARKERSUPERELEMENTPOSITIONPARAMETERS__H
#define CMARKERSUPERELEMENTPOSITIONPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CMarkerSuperElementPositionParameters
class CMarkerSuperElementPositionParameters // AUTO:
{
public: // AUTO:
    Index bodyNumber;                             //!< AUTO: body number to which marker is attached to
    ArrayIndex meshNodeNumbers;                   //!< AUTO: a list of \f$n_m\f$ mesh node numbers of superelement (=interface nodes) which are used to compute the body-fixed marker position; the related nodes must provide 3D position information, such as NodePoint, NodePoint2D, NodeRigidBody[..]; in order to retrieve the global node number, the generic body needs to convert local into global node numbers
    Vector weightingFactors;                      //!< AUTO: a list of \f$n_m\f$ weighting factors per node to compute the final local position; the sum of these weights shall be 1, such that a summation of all nodal positions times weights gives the average position of the marker
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
* @brief        A position marker attached to a SuperElement, such as ObjectFFRF, ObjectGenericODE2 and ObjectFFRFreducedOrder (for which it is in its current implementation inefficient for large number of meshNodeNumbers). The marker acts on the mesh (interface) nodes, not on the underlying nodes of the object.
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
    virtual Index GetObjectNumber(Index localIndex = 0) const override
    {
        return parameters.bodyNumber;
    }

    //! AUTO:  change bodyNumber
    virtual void SetObjectNumber(Index objectNumber, Index localIndex = 0) override
    {
        parameters.bodyNumber = objectNumber;
    }

    //! AUTO:  general access to object number
    virtual Index GetNumberOfObjects() const override
    {
        return 1;
    }

    //! AUTO:  return marker type (for node treatment in computation)
    virtual Marker::Type GetType() const override
    {
        return (Marker::Type)((Index)Marker::Body + (Index)Marker::Object + (Index)Marker::Position + (Index)Marker::SuperElement);
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

    //! AUTO:  position and velocity, and the position Jacobian into temp, without the marker data (#2745)
    virtual void GetKinematicsJacobianPosition(const CSystemData& cSystemData, MarkerPosition<Real>& kinematics, MarkerTemp& temp) const override;

    //! AUTO:  number of ODE2 coordinates of the superelement, without forming the Jacobian (#2745)
    virtual Index GetODE2Size(const CSystemData& cSystemData, MarkerTemp& temp) const override;

    //! AUTO:  add J_pos^T force to ode2Lhs; the Jacobian formed here, so that the kinematics alone do not form it (#2745)
    virtual void AddGeneralizedForce(const CSystemData& cSystemData, const Vector3D& force, MarkerTemp& temp, LinkedDataVector& ode2Lhs) const override;

};



#endif //#ifdef include once...
