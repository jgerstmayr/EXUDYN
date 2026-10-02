/** ***********************************************************************************************
* @class        CNodePointSlope12Parameters
* @brief        Parameter class for CNodePointSlope12
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-02  22:10:14 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef CNODEPOINTSLOPE12PARAMETERS__H
#define CNODEPOINTSLOPE12PARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CNodePointSlope12Parameters
class CNodePointSlope12Parameters // AUTO:
{
public: // AUTO:
    Vector9D referenceCoordinates;                //!< AUTO: reference coordinates (x-pos,y-pos,z-pos; x-slopeX, y-slopeX, z-slopeX; x-slopeY, y-slopeY, z-slopeY) of node; global position of node without displacement
    //! AUTO: default constructor with parameter initialization
    CNodePointSlope12Parameters()
    {
        referenceCoordinates = Vector9D({0.,0.,0.,1.,0.,0.,0.,1.,0.});
    };
};


/** ***********************************************************************************************
* @class        CNodePointSlope12
* @brief        A 3D point/slope vector node for thin ANCF (absolute nodal coordinate formulation) plate elements, with 3 position and 2 \f$\times\f$ 3 slope coordinates, all ABRV:ODE2; the slope vectors are the derivatives of the position with respect to the two in-plane coordinates of the plate.
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

//! AUTO: CNodePointSlope12
class CNodePointSlope12: public CNodeODE2 // AUTO:
{
protected: // AUTO:
    CNodePointSlope12Parameters parameters; //! AUTO: contains all parameters for CNodePointSlope12

public: // AUTO:
    static constexpr Index nODE2coordinates = 9;//AUTO: number of coordinates, used for fixed-size templates

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CNodePointSlope12Parameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CNodePointSlope12Parameters& GetParameters() const { return parameters; }

    //! AUTO:  return number of second order diff. eq. coordinates
    virtual Index GetNumberOfODE2Coordinates() const override
    {
        return 9;
    }

    //! AUTO:  return node type (for node treatment in computation)
    virtual Node::Type GetType() const override
    {
        return (Node::Type)((Index)Node::Position + (Index)Node::Orientation + (Index)Node::PointSlope12);
    }

    //! AUTO:  return configuration dependent position of node; returns always a 3D Vector
    virtual Vector3D GetPosition(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent velocity of node; returns always a 3D Vector
    virtual Vector3D GetVelocity(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent acceleration of node
    virtual Vector3D GetAcceleration(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent rotation matrix of node; returns always a 3D matrix
    virtual Matrix3D GetRotationMatrix(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent time derivative of rotation matrix of node
    Matrix3D GetRotationMatrix_t(ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector
    virtual Vector3D GetAngularVelocity(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent local (=body-fixed) angular velocity of node; in 2D case, this is the same as the global angular velocity; returns always a 3D Vector
    virtual Vector3D GetAngularVelocityLocal(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  provide position jacobian \f$\Jm_P\f$ of node; derivative of global 3D position with respect to all nodal coordiantes; action of force: \f$\Qm_f = \Jm_P^T \fv\f$; zero-size matrix for ground node (no action)
    virtual void GetPositionJacobian(Matrix& value) const override;

    //! AUTO:  provide 'rotation' jacobian \f$\Jm_R\f$ of node; derivative of global 3D angular velocity vector with respect to all velocity coordinates ('G-matrix'); action of torque \f$\mv\f$: \f$\Qm_m = \Jm_R^T \mv\f$
    virtual void GetRotationJacobian(Matrix& value) const override;

    //! AUTO:  provide derivative w.r.t. coordinates of rotation Jacobian times vector; for current configuration
    virtual void GetRotationJacobianTTimesVector_q(const Vector3D& vector, Matrix& jacobian_q) const override;

    //! AUTO:  return internally stored reference coordinates of node
    virtual LinkedDataVector GetReferenceCoordinateVector() const override
    {
        return parameters.referenceCoordinates;
    }

    //! AUTO:  provide according output variable in 'value'; used e.g. for postprocessing and sensors
    virtual void GetOutputVariable(OutputVariableType variableType, ConfigurationType configuration, Vector& value) const override;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index64)OutputVariableType::Position +
            (Index64)OutputVariableType::Displacement +
            (Index64)OutputVariableType::Velocity +
            (Index64)OutputVariableType::Acceleration +
            (Index64)OutputVariableType::CoordinatesTotal +
            (Index64)OutputVariableType::Coordinates +
            (Index64)OutputVariableType::Coordinates_t +
            (Index64)OutputVariableType::Coordinates_tt +
            (Index64)OutputVariableType::RotationMatrix +
            (Index64)OutputVariableType::HomogeneousTransformation +
            (Index64)OutputVariableType::Rotation +
            (Index64)OutputVariableType::AngularVelocity +
            (Index64)OutputVariableType::AngularVelocityLocal );
    }

};



#endif //#ifdef include once...
