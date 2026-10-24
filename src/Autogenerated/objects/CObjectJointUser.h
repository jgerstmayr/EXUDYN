/** ***********************************************************************************************
* @class        CObjectJointUserParameters
* @brief        Parameter class for CObjectJointUser
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-24  22:03:01 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef COBJECTJOINTUSERPARAMETERS__H
#define COBJECTJOINTUSERPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
namespace py = pybind11;            //! AUTO: "py" used throughout in code
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectJointUserParameters
class CObjectJointUserParameters // AUTO:
{
public: // AUTO:
    ArrayIndex markerNumbers;                     //!< AUTO: two rigid markers, whose frames the constraintUserFunction gets
    Index numberOfConstraints;                    //!< AUTO: must be > 0; the number of constraint equations, 1 to 6
    Vector parameters;                            //!< AUTO: the parameters of the joint, of any length, passed to the user functions; see exudyn.advancedUtilities.ParameterLayout
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    PythonUserFunctionBase< std::function<StdVector(const MainSystem&,Real,Index,StdVector3D,StdMatrix3D,StdVector3D,StdMatrix3D,const StdVector&)> > constraintUserFunction;//!< AUTO: A Python user function which computes the constraint equations from the frames of the two markers; needed for an active joint
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Real,Index,StdVector3D,StdMatrix3D,StdVector3D,StdMatrix3D,const StdVector&)> > jacobianUserFunction;//!< AUTO: A Python user function which computes the derivative of the constraint equations by the translations and rotation increments of the markers; optional, computed by central differences if not given
    //! AUTO: default constructor with parameter initialization
    CObjectJointUserParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        numberOfConstraints = 1;
        parameters = Vector();
        activeConnector = true;
        constraintUserFunction = 0;
        jacobianUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectJointUser
* @brief        A joint on two rigid markers whose constraint equations are a Python user function of the positions and rotation matrices of the markers. It has 1 to 6 equations, e.g. for a screw joint, a coupling of rotation and translation or a joint whose axes depend on the motion; the reaction forces and torques follow from the Lagrange multipliers, and the Jacobian is computed numerically if it is not given.
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

//! AUTO: CObjectJointUser
class CObjectJointUser: public CObjectConstraint // AUTO:
{
protected: // AUTO:
    CObjectJointUserParameters parameters; //! AUTO: contains all parameters for CObjectJointUser

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectJointUserParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectJointUserParameters& GetParameters() const { return parameters; }

    //! AUTO:  return true, if object has a computation user function
    virtual bool HasUserFunction() const override
    {
        return true;
    }

    //! AUTO:  default (read) function to return Marker numbers
    virtual const ArrayIndex& GetMarkerNumbers() const override
    {
        return parameters.markerNumbers;
    }

    //! AUTO:  default (write) function to return Marker numbers
    virtual ArrayIndex& GetMarkerNumbers() override
    {
        return parameters.markerNumbers;
    }

    //! AUTO:  true if the connector uses a penalty formulation; false if the constraint uses Lagrange multipliers
    virtual bool IsPenaltyConnector() const override
    {
        return false;
    }

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return parameters.activeConnector ? (JacobianType::Type)(JacobianType::AE_ODE2 + JacobianType::AE_ODE2_function) : (JacobianType::Type)(JacobianType::AE_AE + JacobianType::AE_AE_function);
    }

    //! AUTO:  the equations on the connector interface of rigid markers (#2745)
    virtual bool ComputeAlgebraicEquationsConnector(const CSystemData& systemData, TemporaryComputationData& temp, bool velocityLevel, Vector& localAE) const override
    {
        if (!OnConnectorInterface()) { return false; } ConstraintEquationsRigidMarkers(systemData, temp, *this, velocityLevel, localAE); return true;
    }

    //! AUTO:  C_q by automatic differentiation of the equations (#2745)
    virtual bool ComputeJacobianAEConnector(const CSystemData& systemData, TemporaryComputationData& temp, ResizableMatrix& jacobianAE_ODE2) const override
    {
        if (!OnConnectorInterface()) { return false; } ConstraintJacobianRigidMarkers(systemData, temp, *this, jacobianAE_ODE2); return true;
    }

    //! AUTO:  C_q^T lambda per marker, without forming C_q (#2745)
    virtual bool ComputeReactionForcesConnector(const CSystemData& systemData, TemporaryComputationData& temp, const Vector& reactionForces, Vector& localODE2) const override
    {
        if (!OnConnectorInterface()) { return false; } ConstraintReactionForcesRigidMarkers(systemData, temp, *this, reactionForces, localODE2); return true;
    }

    //! AUTO:  the equations of the inactive constraint, lambda = 0; the active one computes on the connector interface (#2745)
    virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, bool velocityLevel = false) const override
    {
        CHECKandTHROW(!IsActive(), "ComputeAlgebraicEquations: an active constraint computes on the connector interface"); algebraicEquations.CopyFrom(markerData.GetLagrangeMultipliers());
    }

    //! AUTO:  the Jacobian of the inactive constraint, d(lambda)/d(lambda); the active one computes on the connector interface (#2745)
    virtual void ComputeJacobianAE(ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE, const MarkerDataStructure& markerData, Real t) const override
    {
        CHECKandTHROW(!IsActive(), "ComputeJacobianAE: an active constraint computes on the connector interface"); jacobian_AE.SetScalarMatrix(GetAlgebraicEquationsSize(), 1.);
    }

    //! AUTO:  the equations of the constraintUserFunction, at velocity level by the Jacobian (#2976)
    virtual void ComputeConstraintEquationsRigid(const MarkerRigid<Real>* markers, const LinkedDataVector& lambda, Real t, bool velocityLevel, ConstSizeVector<maxConstraintEquations>& equations) const override;

    //! AUTO:  the equations with their derivatives by the 12 increments of the markers, from the jacobianUserFunction or by central differences (#2976)
    virtual void ComputeConstraintEquationsRigidDiff(const MarkerRigid<DRealRigidMarkers>* markers, const LinkedDataVector& lambda, Real t, ConstSizeVectorBase<DRealRigidMarkers, maxConstraintEquations>& equations) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Vector& value) const override;

    //! AUTO:  provide requested markerType for connector
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return (Marker::Type)((Index)Marker::Position + (Index)Marker::Orientation);
    }

    //! AUTO:  return object type (for node treatment in computation)
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);
    }

    //! AUTO:  number of algebraic equations; independent of node/body coordinates
    virtual Index GetAlgebraicEquationsSize() const override
    {
        return parameters.numberOfConstraints;
    }

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

    //! AUTO:  call of constraintUserFunction, with the size checked; implemented in evaluateUserFunctions.cpp
    void EvaluateUserFunctionConstraint(Vector& equations, Real t, const Vector3D& position0, const Matrix3D& rotation0, const Vector3D& position1, const Matrix3D& rotation1) const;

    //! AUTO:  call of jacobianUserFunction, with the size checked; implemented in evaluateUserFunctions.cpp
    void EvaluateUserFunctionJacobian(Matrix& jacobian, Real t, const Vector3D& position0, const Matrix3D& rotation0, const Vector3D& position1, const Matrix3D& rotation1) const;

    //! AUTO:  dc/d[p0, theta0, p1, theta1], n_c x 12: the jacobianUserFunction or central differences of the constraintUserFunction (#2976)
    void ComputeJacobianIncrements(Matrix& jacobian, Real t, const Vector3D* positions, const Matrix3D* rotations) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index64)OutputVariableType::Position +
            (Index64)OutputVariableType::Displacement +
            (Index64)OutputVariableType::Force +
            (Index64)OutputVariableType::ConstraintEquation );
    }

};



#endif //#ifdef include once...
