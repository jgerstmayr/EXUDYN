/** ***********************************************************************************************
* @class        CNodeRigidBodyRotVecLGParameters
* @brief        Parameter class for CNodeRigidBodyRotVecLG
*
* @author       Gerstmayr Johannes, Holzinger Stefan
* @date         2019-07-01 (generated)
* @date         2026-11-10  21:43:04 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef CNODERIGIDBODYROTVECLGPARAMETERS__H
#define CNODERIGIDBODYROTVECLGPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CNodeRigidBodyRotVecLGParameters
class CNodeRigidBodyRotVecLGParameters // AUTO:
{
public: // AUTO:
    Vector6D referenceCoordinates;                //!< AUTO: reference coordinates (position and rotation vector \f$\tnu\f$) of node ==> e.g. ref. coordinates for finite elements or reference position of rigid body (e.g. for definition of joints)
    //! AUTO: default constructor with parameter initialization
    CNodeRigidBodyRotVecLGParameters()
    {
        referenceCoordinates = Vector6D({0.,0.,0., 0.,0.,0.});
    };
};


/** ***********************************************************************************************
* @class        CNodeRigidBodyRotVecLG
* @brief        A 3D rigid body node based on rotation vector and Lie group methods for rigid bodies. The node has 3 displacement coordinates and three rotation coordinates and can be used in combination with explicit Lie Group time integration methods and the implicit generalized-alpha solver; the static solver cannot update its rotation and refuses it (#1493).
*
* @author       Gerstmayr Johannes, Holzinger Stefan
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

//! AUTO: CNodeRigidBodyRotVecLG
class CNodeRigidBodyRotVecLG: public CNodeRigidBody // AUTO:
{
protected: // AUTO:
    static constexpr Index nRotationCoordinates = 3;
    static constexpr Index nDisplacementCoordinates = 3;
    CNodeRigidBodyRotVecLGParameters parameters; //! AUTO: contains all parameters for CNodeRigidBodyRotVecLG

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CNodeRigidBodyRotVecLGParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CNodeRigidBodyRotVecLGParameters& GetParameters() const { return parameters; }

    //! AUTO:  return number of second order diff. eq. coordinates
    virtual Index GetNumberOfODE2Coordinates() const override
    {
        return 6;
    }

    //! AUTO:  return number of displacement coordinates
    virtual Index GetNumberOfDisplacementCoordinates() const override
    {
        return nDisplacementCoordinates;
    }

    //! AUTO:  return number of rotation coordinates
    virtual Index GetNumberOfRotationCoordinates() const override
    {
        return nRotationCoordinates;
    }

    //! AUTO:  return node type (for node treatment in computation)
    virtual Node::Type GetType() const override
    {
        return (Node::Type)((Index)Node::Position + (Index)Node::Orientation + (Index)Node::RigidBody + (Index)Node::RotationRotationVector + (Index)Node::LieGroupWithDirectUpdate);
    }

    //! AUTO:  return node group, which is special because of algebraic equations
    virtual CNodeGroup GetNodeGroup() const override
    {
        return CNodeGroup::ODE2variables;
    }

    //! AUTO:  return configuration dependent position of node; returns always a 3D Vector
    virtual Vector3D GetPosition(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent velocity of node; returns always a 3D Vector
    virtual Vector3D GetVelocity(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent acceleration of node
    virtual Vector3D GetAcceleration(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent rotation matrix of node; returns always a 3D matrix
    virtual Matrix3D GetRotationMatrix(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent angular velocity of node; returns always a 3D Vector
    virtual Vector3D GetAngularVelocity(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent local (=body-fixed) angular velocity of node; in 2D case, this is the same as the global angular velocity; returns always a 3D Vector
    virtual Vector3D GetAngularVelocityLocal(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent angular acceleration of node
    virtual Vector3D GetAngularAcceleration(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  provide position jacobian \f$\Jm_P\f$ of node; derivative of global 3D position with respect to all nodal coordiantes; action of force: \f$\Qm_f = \Jm_P^T \fv\f$; zero-size matrix for ground node (no action)
    virtual void GetPositionJacobian(Matrix& value) const override;

    //! AUTO:  provide 'rotation' jacobian \f$\Jm_R\f$ of node; derivative of global 3D angular velocity vector with respect to all velocity coordinates ('G-matrix'); action of torque \f$\mv\f$: \f$\Qm_m = \Jm_R^T \mv\f$
    virtual void GetRotationJacobian(Matrix& value) const override;

    //! AUTO:  provide derivative w.r.t. coordinates of rotation Jacobian times vector; for current configuration
    virtual void GetRotationJacobianTTimesVector_q(const Vector3D& vector, Matrix& jacobian_q) const override;

    //! AUTO:  static function to compute d(G^T*v)/dq for rotation vector (Glocal = I, G = RotationMatrix); using autodiff
    static Matrix3D RotationVectorGTv_q(const CSVector4D& rotParameters, const Vector3D& v3D);

    //! AUTO:  provide nodal values efficiently for rigid body computation
    virtual void CollectCurrentNodeData1(ConstSizeMatrix<maxRotationCoordinates * nDim3D>& Glocal, Vector3D& angularVelocityLocal) const override;

    //! AUTO:  obtain G matrices, position, velocity, rotation matrix A (local to global), local angular velocity
    virtual void CollectCurrentNodeMarkerData(ConstSizeMatrix<maxRotationCoordinates * nDim3D>& Glocal, ConstSizeMatrix<maxRotationCoordinates * nDim3D>& G, Vector3D& pos, Vector3D& vel, Matrix3D& A, Vector3D& angularVelocityLocal) const override;

    //! AUTO:  return internally stored reference coordinates of node
    virtual LinkedDataVector GetReferenceCoordinateVector() const override
    {
        return parameters.referenceCoordinates;
    }

    //! AUTO:  provide according output variable in 'value'; used e.g. for postprocessing and sensors
    virtual void GetOutputVariable(OutputVariableType variableType, ConfigurationType configuration, Vector& value) const override;

    //! AUTO:  Compute vector to of 4 Euler Parameters from reference and configuration coordinates
    virtual ConstSizeVector<maxRotationCoordinates> GetRotationParameters(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  Compute vector to time derivative of 4 Euler Parameters in given configuration
    virtual LinkedDataVector GetRotationParameters_t(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  Compute G matrix (=diff(angularVelocity, velocityParameters)) for given configuration
    virtual void GetG(ConstSizeMatrix<maxRotationCoordinates * nDim3D>& matrix, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  Compute local G matrix for given configuration
    virtual void GetGlocal(ConstSizeMatrix<maxRotationCoordinates * nDim3D>& matrix, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  Compute G matrix (=diff(angularVelocity, velocityParameters)) for given configuration
    virtual void GetG_t(ConstSizeMatrix<maxRotationCoordinates * nDim3D>& matrix, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  Compute local G matrix for given configuration
    virtual void GetGlocal_t(ConstSizeMatrix<maxRotationCoordinates * nDim3D>& matrix, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  compute d(G^T*v)/dq for any set of parameters; needed for jacobians
    virtual void GetGTv_q(const Vector3D& v, ConstSizeMatrix<maxRotationCoordinates * maxRotationCoordinates>& matrix, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  compute d(Glocal^T*v)/dq for any set of parameters; needed for jacobians
    virtual void GetGlocalTv_q(const Vector3D& v, ConstSizeMatrix<maxRotationCoordinates * maxRotationCoordinates>& matrix, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  apply composition rule for all nodal coordinates
    virtual void CompositionRule(const LinkedDataVector& currentPosition, const LinkedDataVector& currentOrientation, const Vector6D& incrementalMotion, LinkedDataVector& newPosition, LinkedDataVector& newOrientation) const override;

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
            (Index64)OutputVariableType::RotationMatrix +
            (Index64)OutputVariableType::HomogeneousTransformation +
            (Index64)OutputVariableType::Rotation +
            (Index64)OutputVariableType::AngularVelocity +
            (Index64)OutputVariableType::AngularVelocityLocal );
    }

};


//! AUTO: the member functions that the declarations of CNodeRigidBodyRotVecLG require (#2866)
#include <type_traits>
static_assert(!std::is_same<decltype(&CNodeRigidBodyRotVecLG::GetOutputVariable), decltype(&CNode::GetOutputVariable)>::value, "CNodeRigidBodyRotVecLG declares output variables and must implement GetOutputVariable; the default of CNode raises");
static_assert(!std::is_same<decltype(&CNodeRigidBodyRotVecLG::GetPosition), decltype(&CNodeODE2::GetPosition)>::value, "CNodeRigidBodyRotVecLG declares Position and must implement GetPosition; the default of CNodeODE2 raises");
static_assert(!std::is_same<decltype(&CNodeRigidBodyRotVecLG::GetVelocity), decltype(&CNodeODE2::GetVelocity)>::value, "CNodeRigidBodyRotVecLG declares Position and must implement GetVelocity; the default of CNodeODE2 raises");
static_assert(!std::is_same<decltype(&CNodeRigidBodyRotVecLG::GetPositionJacobian), decltype(&CNodeODE2::GetPositionJacobian)>::value, "CNodeRigidBodyRotVecLG declares Position and must implement GetPositionJacobian; the default of CNodeODE2 raises");
static_assert(!std::is_same<decltype(&CNodeRigidBodyRotVecLG::GetRotationMatrix), decltype(&CNodeODE2::GetRotationMatrix)>::value, "CNodeRigidBodyRotVecLG declares Orientation and must implement GetRotationMatrix; the default of CNodeODE2 raises");
static_assert(!std::is_same<decltype(&CNodeRigidBodyRotVecLG::GetAngularVelocity), decltype(&CNodeODE2::GetAngularVelocity)>::value, "CNodeRigidBodyRotVecLG declares Orientation and must implement GetAngularVelocity; the default of CNodeODE2 raises");
static_assert(!std::is_same<decltype(&CNodeRigidBodyRotVecLG::GetAngularVelocityLocal), decltype(&CNodeODE2::GetAngularVelocityLocal)>::value, "CNodeRigidBodyRotVecLG declares Orientation and must implement GetAngularVelocityLocal; the default of CNodeODE2 raises");
static_assert(!std::is_same<decltype(&CNodeRigidBodyRotVecLG::GetRotationJacobian), decltype(&CNodeODE2::GetRotationJacobian)>::value, "CNodeRigidBodyRotVecLG declares Orientation and must implement GetRotationJacobian; the default of CNodeODE2 raises");
static_assert(!std::is_same<decltype(&CNodeRigidBodyRotVecLG::GetNumberOfRotationCoordinates), decltype(&CNodeRigidBody::GetNumberOfRotationCoordinates)>::value, "CNodeRigidBodyRotVecLG declares RigidBody and must implement GetNumberOfRotationCoordinates; the default of CNodeRigidBody raises");
static_assert(!std::is_same<decltype(&CNodeRigidBodyRotVecLG::GetG), decltype(&CNodeRigidBody::GetG)>::value, "CNodeRigidBodyRotVecLG declares RigidBody and must implement GetG; the default of CNodeRigidBody raises");
static_assert(!std::is_same<decltype(&CNodeRigidBodyRotVecLG::GetGlocal), decltype(&CNodeRigidBody::GetGlocal)>::value, "CNodeRigidBodyRotVecLG declares RigidBody and must implement GetGlocal; the default of CNodeRigidBody raises");
static_assert(!std::is_same<decltype(&CNodeRigidBodyRotVecLG::GetG_t), decltype(&CNodeRigidBody::GetG_t)>::value, "CNodeRigidBodyRotVecLG declares RigidBody and must implement GetG_t; the default of CNodeRigidBody raises");
static_assert(!std::is_same<decltype(&CNodeRigidBodyRotVecLG::GetGlocal_t), decltype(&CNodeRigidBody::GetGlocal_t)>::value, "CNodeRigidBodyRotVecLG declares RigidBody and must implement GetGlocal_t; the default of CNodeRigidBody raises");

#endif //#ifdef include once...
