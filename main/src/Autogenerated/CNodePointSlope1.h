/** ***********************************************************************************************
* @class        CNodePointSlope1Parameters
* @brief        Parameter class for CNodePointSlope1
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2024-10-26  18:53:42 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CNODEPOINTSLOPE1PARAMETERS__H
#define CNODEPOINTSLOPE1PARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CNodePointSlope1Parameters
class CNodePointSlope1Parameters // AUTO: 
{
public: // AUTO: 
    Vector6D referenceCoordinates;                //!< AUTO: reference coordinates (x-pos,y-pos,z-pos; x-slopex, y-slopex, z-slopex) of node; global position of node without displacement
    //! AUTO: default constructor with parameter initialization
    CNodePointSlope1Parameters()
    {
        referenceCoordinates = Vector6D({0.,0.,0.,1.,0.,0.});
    };
};


/** ***********************************************************************************************
* @class        CNodePointSlope1
* @brief        A 3D point/slope vector node for spatial Bernoulli-Euler ANCF (absolute nodal coordinate formulation) beam elements; the node has 6 displacement degrees of freedom (3 for displacement of point node and 3 for the slope vector 'slopex'); all coordinates lead to second order differential equations; the slope vector defines the directional derivative w.r.t the local axial (x) coordinate, denoted as \f$()^\prime\f$; in straight configuration aligned at the global x-axis, the slope vector reads \f$\rv^\prime=[r_x^\prime\;\;r_y^\prime\;\;r_z^\prime]^T=[1\;\;0]^T\f$.
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

//! AUTO: CNodePointSlope1
class CNodePointSlope1: public CNodeODE2 // AUTO: 
{
protected: // AUTO: 
    CNodePointSlope1Parameters parameters; //! AUTO: contains all parameters for CNodePointSlope1

public: // AUTO: 
    static constexpr Index nODE2coordinates = 6;//AUTO: number of coordinates, used for fixed-size templates

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CNodePointSlope1Parameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CNodePointSlope1Parameters& GetParameters() const { return parameters; }

    //! AUTO:  return number of second order diff. eq. coordinates
    virtual Index GetNumberOfODE2Coordinates() const override
    {
        return 6;
    }

    //! AUTO:  return node type (for node treatment in computation)
    virtual Node::Type GetType() const override
    {
        return (Node::Type)(Node::Position + Node::PointSlope1);
    }

    //! AUTO:  return configuration dependent position of node; returns always a 3D Vector
    virtual Vector3D GetPosition(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent velocity of node; returns always a 3D Vector
    virtual Vector3D GetVelocity(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent acceleration of node
    virtual Vector3D GetAcceleration(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent rotation matrix of node; the slope vector \f$\rv^\prime = [1,0]\f$ is defines as zero angle (\f$\varphi = 0\f$), leading to a matrix \f$\Am = \mr{\cos\varphi}{-\sin\varphi}{0} {\sin\varphi}{\cos\varphi}{0} {0}{0}{1}\f$; the function always computes a 3D Matrix
    virtual Matrix3D GetRotationMatrix(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  provide position jacobian \f$\Jm_P\f$ of node; derivative of 3D position with respect to 4 coordinates ux,uy and x/y 'displacements' of slopex; action of force: \f$\Qm_f = \Jm_P^T \fv\f$
    virtual void GetPositionJacobian(Matrix& value) const override;

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
            (Index)OutputVariableType::Position +
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::Acceleration +
            (Index)OutputVariableType::CoordinatesTotal +
            (Index)OutputVariableType::Coordinates +
            (Index)OutputVariableType::Coordinates_t +
            (Index)OutputVariableType::Coordinates_tt );
    }

};



#endif //#ifdef include once...
