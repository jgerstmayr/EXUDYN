/** ***********************************************************************************************
* @class        CNodePoint3DSlope23Parameters
* @brief        Parameter class for CNodePoint3DSlope23
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2022-05-12  23:26:18 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef CNODEPOINT3DSLOPE23PARAMETERS__H
#define CNODEPOINT3DSLOPE23PARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CNodePoint3DSlope23Parameters
class CNodePoint3DSlope23Parameters // AUTO: 
{
public: // AUTO: 
    Vector9D referenceCoordinates;                //!< AUTO: reference coordinates (x-pos,y-pos,z-pos; x-slopey, y-slopey, z-slopey; x-slopez, y-slopez, z-slopez) of node; global position of node without displacement
    //! AUTO: default constructor with parameter initialization
    CNodePoint3DSlope23Parameters()
    {
        referenceCoordinates = Vector9D({0.,0.,0.,1.,0.,0.,1.,0.,0.});
    };
};


/** ***********************************************************************************************
* @class        CNodePoint3DSlope23
* @brief        A 3D point/slope vector node for spatial, shear and cross-section deformable ANCF (absolute nodal coordinate formulation) beam elements; the node has 9 ODE2 degrees of freedom (3 for displacement of point node and 2 \f$\times\f$ 3 for the slope vectors 'slopey' and 'slopez'); all coordinates lead to second order differential equations; the slope vector defines the directional derivative w.r.t the local axial (x) coordinate, denoted as \f$()^\prime\f$; in straight configuration aligned at the global x-axis, the slopey vector reads \f$\rv_y^\prime=[0\;\;1\;\;0]^T\f$ and slopez gets \f$\rv_z^\prime=[0\;\;0\;\;1]^T\f$.
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

//! AUTO: CNodePoint3DSlope23
class CNodePoint3DSlope23: public CNodeODE2 // AUTO: 
{
protected: // AUTO: 
    CNodePoint3DSlope23Parameters parameters; //! AUTO: contains all parameters for CNodePoint3DSlope23

public: // AUTO: 
    static constexpr Index nODE2coordinates = 9;//AUTO: number of coordinates, used for fixed-size templates

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CNodePoint3DSlope23Parameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CNodePoint3DSlope23Parameters& GetParameters() const { return parameters; }

    //! AUTO:  return number of second order diff. eq. coordinates
    virtual Index GetNumberOfODE2Coordinates() const override
    {
        return 9;
    }

    //! AUTO:  return node type (for node treatment in computation)
    virtual Node::Type GetType() const override
    {
        return (Node::Type)(Node::Position + Node::Orientation + Node::Point3DSlope23);
    }

    //! AUTO:  return configuration dependent position of node; returns always a 3D Vector
    virtual Vector3D GetPosition(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent velocity of node; returns always a 3D Vector
    virtual Vector3D GetVelocity(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent rotation matrix of node
    virtual Matrix3D GetRotationMatrix(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent time derivative of rotation matrix of node
    Matrix3D GetRotationMatrix_t(ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector
    virtual Vector3D GetAngularVelocity(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent angular velocity of node; in 2D case, this is the same as the global angular velocity; returns always a 3D Vector
    virtual Vector3D GetAngularVelocityLocal(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  provide position jacobian \f$\Jm_P\f$ of node; derivative of 3D position with respect to 4 coordinates ux,uy and x/y 'displacements' of slopex; action of force: \f$\Qm_f = \Jm_P^T \fv\f$
    virtual void GetPositionJacobian(Matrix& value) const override;

    //! AUTO:  provide 'rotation' jacobian \f$\Jm_R\f$ of node; derivative of 3D angular velocity with respect to 4 velocity coordinates ux,uy and x/y 'displacements' of slopex; action of torque: \f$\Qm_m = \Jm_R^T \mv\f$
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
            (Index)OutputVariableType::Position +
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::Coordinates +
            (Index)OutputVariableType::Coordinates_t +
            (Index)OutputVariableType::Coordinates_tt );
    }

};



#endif //#ifdef include once...
