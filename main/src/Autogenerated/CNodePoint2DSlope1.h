/** ***********************************************************************************************
* @class        CNodePoint2DSlope1Parameters
* @brief        Parameter class for CNodePoint2DSlope1
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-01-25  01:33:20 (last modfied)
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


//! AUTO: Parameters for class CNodePoint2DSlope1Parameters
class CNodePoint2DSlope1Parameters // AUTO: 
{
public: // AUTO: 
    Vector4D referenceCoordinates;                //!< AUTO: reference coordinates (x-pos,y-pos; x-slopex, y-slopex) of node; global position of node without displacement
    //! AUTO: default constructor with parameter initialization
    CNodePoint2DSlope1Parameters()
    {
        referenceCoordinates = Vector4D({0.,0.,1.,0.});
    };
};


/** ***********************************************************************************************
* @class        CNodePoint2DSlope1
* @brief        A 2D point/slope vector node for planar Bernoulli-Euler ANCF (absolute nodal coordinate formulation) beam elements; the node has 4 displacement degrees of freedom (2 for displacement of point node and 2 for the slope vector 'slopex'); all coordinates lead to second order differential equations; the slope vector defines the directional derivative w.r.t the local axial (x) coordinate, denoted as \f$()^\prime\f$; in straight configuration aligned at the global x-axis, the slope vector reads \f$\rv^\prime=[r_x^\prime\;\;r_y^\prime]^T=[1\;\;0]^T\f$.
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

//! AUTO: CNodePoint2DSlope1
class CNodePoint2DSlope1: public CNodeODE2 // AUTO: 
{
protected: // AUTO: 
    CNodePoint2DSlope1Parameters parameters; //! AUTO: contains all parameters for CNodePoint2DSlope1

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CNodePoint2DSlope1Parameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CNodePoint2DSlope1Parameters& GetParameters() const { return parameters; }

    //! AUTO:  return number of second order diff. eq. coordinates
    virtual Index GetNumberOfODE2Coordinates() const override
    {
        return 4;
    }

    //! AUTO:  return node type (for node treatment in computation)
    virtual Node::Type GetType() const override
    {
        return (Node::Type)(Node::Position2D + Node::Orientation2D + Node::Point2DSlope1);
    }

    //! AUTO:  return configuration dependent position of node; returns always a 3D Vector
    virtual Vector3D GetPosition(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent velocity of node; returns always a 3D Vector
    virtual Vector3D GetVelocity(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent rotation matrix of node; the slope vector \f$\rv^\prime = [1,0]\f$ is defines as zero angle (\f$\varphi = 0\f$), leading to a matrix \f$\Am = \mr{\cos\varphi}{-\sin\varphi}{0} {\sin\varphi}{\cos\varphi}{0} {0}{0}{1}\f$; the function always computes a 3D Matrix
    virtual Matrix3D GetRotationMatrix(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent local (=body fixed) angular velocity of node; returns always a 3D Vector
    virtual Vector3D GetAngularVelocity(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent angular velocity of node; in 2D case, this is the same as the global angular velocity; returns always a 3D Vector
    virtual Vector3D GetAngularVelocityLocal(ConfigurationType configuration = ConfigurationType::Current) const override
    {
        return GetAngularVelocity(configuration);
    }

    //! AUTO:  provide position jacobian \f$\Jm_P\f$ of node; derivative of 3D position with respect to 4 coordinates ux,uy and x/y "displacements" of slopex; action of force: \f$\Qm_f = \Jm_P^T \fv\f$
    virtual void GetPositionJacobian(Matrix& value) const override;

    //! AUTO:  provide "rotation" jacobian \f$\Jm_R\f$ of node; derivative of 3D angular velocity with respect to 4 velocity coordinates ux,uy and x/y "displacements" of slopex; action of torque: \f$\Qm_m = \Jm_R^T \mv\f$
    virtual void GetRotationJacobian(Matrix& value) const override;

    //! AUTO:  return internally stored reference coordinates of node
    virtual LinkedDataVector GetReferenceCoordinateVector() const override
    {
        return parameters.referenceCoordinates;
    }

    //! AUTO:  provide according output variable in "value"; used e.g. for postprocessing and sensors
    virtual void GetOutputVariable(OutputVariableType variableType, ConfigurationType configuration, Vector& value) const override;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Position +
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::Coordinates +
            (Index)OutputVariableType::Coordinates_t );
    }

};


