/** ***********************************************************************************************
* @class        CObjectRigidBody2DParameters
* @brief        Parameter class for CObjectRigidBody2D
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-01-24  13:27:00 (last modfied)
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


//! AUTO: Parameters for class CObjectRigidBody2DParameters
class CObjectRigidBody2DParameters // AUTO: 
{
public: // AUTO: 
    Real physicsMass;                             //!< AUTO: mass [SI:kg] of mass point
    Real physicsInertia;                          //!< AUTO: inertia [SI:kgm\f$^2\f$] of rigid body w.r.t. center of mass
    Index nodeNumber;                             //!< AUTO: node number for 2D rigid body node
    //! AUTO: default constructor with parameter initialization
    CObjectRigidBody2DParameters()
    {
        physicsMass = 0.;
        physicsInertia = 0.;
        nodeNumber = EXUstd::InvalidIndex;
    };
};


/** ***********************************************************************************************
* @class        CObjectRigidBody2D
* @brief        A 2D rigid body which is attached to a rigid body 2D node. Equations of motion with the displacements \f$[u_x\;\; u_y]^T\f$ of the center of mass and the rotation \f$\varphi\f$ (positive rotation around z-axis), the mass \f$m\f$, inertia around z-axis \f$J\f$ and the residual of all forces and moments \f$[R_x\;\; R_y\;\; R_\varphi]^T\f$ are given as \f[ \vr{m \cdot \ddot u_x}{m \cdot \ddot u_y}{J \varphi} = \vr{R_x}{R_y}{R_\varphi}.\f]
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

//! AUTO: CObjectRigidBody2D
class CObjectRigidBody2D: public CObjectBody // AUTO: 
{
protected: // AUTO: 
    static const Index nODE2Coordinates = 3;
    CObjectRigidBody2DParameters parameters; //! AUTO: contains all parameters for CObjectRigidBody2D

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectRigidBody2DParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectRigidBody2DParameters& GetParameters() const { return parameters; }

    //! AUTO:  Computational function: compute mass matrix
    virtual void ComputeMassMatrix(Matrix& massMatrix) const override;

    //! AUTO:  Computational function: compute right-hand-side (RHS) of second order ordinary differential equations (ODE) to "ode2rhs"
    virtual void ComputeODE2RHS(Vector& ode2Rhs) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return JacobianType::None;
    }

    //! AUTO:  Flags to determine, which access (forces, moments, connectors, ...) to object are possible
    virtual AccessFunctionType GetAccessFunctionTypes() const override;

    //! AUTO:  provide Jacobian at localPosition in "value" according to object access
    virtual void GetAccessFunctionBody(AccessFunctionType accessType, const Vector3D& localPosition, Matrix& value) const override;

    //! AUTO:  provide according output variable in "value"
    virtual void GetOutputVariableBody(OutputVariableType variableType, const Vector3D& localPosition, ConfigurationType configuration, Vector& value) const override;

    //! AUTO:  return the (global) position of "localPosition" according to configuration type
    virtual Vector3D GetPosition(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) position of "localPosition" according to configuration type
    virtual Vector3D GetDisplacement(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) velocity of "localPosition" according to configuration type
    virtual Vector3D GetVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent rotation matrix of node; returns always a 3D Matrix, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
    virtual Matrix3D GetRotationMatrix(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
    virtual Vector3D GetAngularVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent local (=body fixed) angular velocity of node, which is the same as the global angular velocity vector in 2D; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
    virtual Vector3D GetAngularVelocityLocal(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override
    {
        return GetAngularVelocity(localPosition, configuration);
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        release_assert(localIndex == 0);
        return parameters.nodeNumber;
    }

    //! AUTO:  number of nodes; needed for every object
    virtual Index GetNumberOfNodes() const override
    {
        return 1;
    }

    //! AUTO:  number of ODE2 coordinates; needed for object?
    virtual Index GetODE2Size() const override
    {
        return nODE2Coordinates;
    }

    //! AUTO:  Get type of object, e.g. to categorize and distinguish during assembly and computation
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::SingleNoded);
    }

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Position +
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::Rotation +
            (Index)OutputVariableType::AngularVelocity +
            (Index)OutputVariableType::RotationMatrix );
    }

};


