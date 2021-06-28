/** ***********************************************************************************************
* @class        CObjectRotationalMass1DParameters
* @brief        Parameter class for CObjectRotationalMass1D
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-06-25  13:31:26 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTROTATIONALMASS1DPARAMETERS__H
#define COBJECTROTATIONALMASS1DPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CObjectRotationalMass1DParameters
class CObjectRotationalMass1DParameters // AUTO: 
{
public: // AUTO: 
    Real physicsInertia;                          //!< AUTO: inertia components [SI:kgm\f$^2\f$] of rotor / rotational mass
    Index nodeNumber;                             //!< AUTO: node number (type NodeIndex) of Node1D, providing rotation coordinate \f$\psi_0 = c_0\f$
    Vector3D referencePosition;                   //!< AUTO: a constant reference position, used to assign joint constraints accordingly and for drawing
    Matrix3D referenceRotation;                   //!< AUTO: an intermediate rotation matrix, which transforms the 1D coordinate into 3D, see description
    //! AUTO: default constructor with parameter initialization
    CObjectRotationalMass1DParameters()
    {
        physicsInertia = 0.;
        nodeNumber = EXUstd::InvalidIndex;
        referencePosition = Vector3D({0.,0.,0.});
        referenceRotation = EXUmath::unitMatrix3D;
    };
};


/** ***********************************************************************************************
* @class        CObjectRotationalMass1D
* @brief        A 1D rotational inertia (mass) which is attached to Node1D.
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

//! AUTO: CObjectRotationalMass1D
class CObjectRotationalMass1D: public CObjectBody // AUTO: 
{
protected: // AUTO: 
    CObjectRotationalMass1DParameters parameters; //! AUTO: contains all parameters for CObjectRotationalMass1D

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectRotationalMass1DParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectRotationalMass1DParameters& GetParameters() const { return parameters; }

    //! AUTO:  Computational function: compute mass matrix
    virtual void ComputeMassMatrix(Matrix& massMatrix, Index objectNumber) const override;

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs, Index objectNumber) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return JacobianType::_None;
    }

    //! AUTO:  Flags to determine, which access (forces, moments, connectors, ...) to object are possible
    virtual AccessFunctionType GetAccessFunctionTypes() const override;

    //! AUTO:  provide Jacobian at localPosition in 'value' according to object access
    virtual void GetAccessFunctionBody(AccessFunctionType accessType, const Vector3D& localPosition, Matrix& value) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableBody(OutputVariableType variableType, const Vector3D& localPosition, ConfigurationType configuration, Vector& value, Index objectNumber) const override;

    //! AUTO:  return the rotation angle (reference+current) according to configuration type
    Real GetRotationAngle(ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  return the (global) position of 'localPosition' according to configuration type
    virtual Vector3D GetPosition(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) position of 'localPosition' according to configuration type
    virtual Vector3D GetDisplacement(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) velocity of 'localPosition' according to configuration type
    virtual Vector3D GetVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent rotation matrix of node; returns always a 3D Matrix, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
    virtual Matrix3D GetRotationMatrix(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
    virtual Vector3D GetAngularVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
    virtual Vector3D GetAngularVelocityLocal(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the local position of the center of mass, needed for equations of motion and for massProportionalLoad
    virtual Vector3D GetLocalCenterOfMass() const override
    {
        return Vector3D({0.,0.,0.});
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
        return 1;
    }

    //! AUTO:  Get type of object, e.g. to categorize and distinguish during assembly and computation
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::SingleNoded);
    }

    //! AUTO:  return true if object has time and coordinate independent (=constant) mass matrix
    virtual bool HasConstantMassMatrix() const override
    {
        return true;
    }

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Position +
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::RotationMatrix +
            (Index)OutputVariableType::Rotation +
            (Index)OutputVariableType::AngularVelocity +
            (Index)OutputVariableType::AngularVelocityLocal );
    }

};



#endif //#ifdef include once...
