/** ***********************************************************************************************
* @class        CObjectUserBodyParameters
* @brief        Parameter class for CObjectUserBody
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-22  15:51:08 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef COBJECTUSERBODYPARAMETERS__H
#define COBJECTUSERBODYPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
namespace py = pybind11;            //! AUTO: "py" used throughout in code
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectUserBodyParameters
class CObjectUserBodyParameters // AUTO:
{
public: // AUTO:
    ArrayIndex nodeNumbers;                       //!< AUTO: the nodes whose ABRV:ODE2 coordinates are the coordinates of the body, in this order
    Vector parameters;                            //!< AUTO: the parameters of the body - masses, lengths, stiffnesses -, passed to every user function; their meaning is defined by the user functions, see exudyn.advancedUtilities.ParameterLayout; empty by default
    bool constantMassMatrix;                      //!< AUTO: True: the mass matrix does not depend on the coordinates and the time; massMatrixUserFunction is called once, its result is kept and reused - computed again after a change of a parameter of the body or Assemble -, and the solvers treat the mass matrix as constant
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > massMatrixUserFunction;//!< AUTO: A Python user function which computes the mass matrix; it must be given
    PythonUserFunctionBase< std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > forceUserFunction;//!< AUTO: A Python user function which computes the generalized forces on the right-hand side; zero if not given
    PythonUserFunctionBase< std::function<StdVector3D(const MainSystem&,Real,Index,StdVector3D,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > positionUserFunction;//!< AUTO: A Python user function which computes the global position of the point at localPosition
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Real,Index,StdVector3D,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > positionJacobianUserFunction;//!< AUTO: A Python user function which computes the position Jacobian at localPosition
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Real,Index,StdVector3D,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > rotationMatrixUserFunction;//!< AUTO: A Python user function which computes the rotation matrix at localPosition; optional, for rigid markers
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Real,Index,StdVector3D,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > rotationJacobianUserFunction;//!< AUTO: A Python user function which computes the rotation Jacobian (of the global angular velocity) at localPosition; optional, for rigid markers
    PythonUserFunctionBase< std::function<StdVector(const MainSystem&,Real,Index,OutputVariableType,StdVector3D,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > outputVariableUserFunction;//!< AUTO: A Python user function which computes the output variables at localPosition that the body does not compute itself, e.g. strains, energy, accelerations and UserType0 to UserType3; optional
    //! AUTO: default constructor with parameter initialization
    CObjectUserBodyParameters()
    {
        nodeNumbers = ArrayIndex();
        parameters = Vector();
        constantMassMatrix = false;
        massMatrixUserFunction = 0;
        forceUserFunction = 0;
        positionUserFunction = 0;
        positionJacobianUserFunction = 0;
        rotationMatrixUserFunction = 0;
        rotationJacobianUserFunction = 0;
        outputVariableUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectUserBody
* @brief        A body whose equations of motion and whose position, and optionally orientation, at a local position are computed by Python user functions: a user finite element or a user rigid body on existing nodes. As it provides the access functions of a body, the body markers (MarkerBodyPosition, MarkerBodyRigid), SensorBody, the loads on these markers and every connector work on it.
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

//! AUTO: CObjectUserBody
class CObjectUserBody: public CObjectBody // AUTO:
{
protected: // AUTO:
    //! a user function that returns a matrix of 3 rows, with its name for the messages and the number of columns (#2995)
    template<class TUserFunction>
    void EvaluateMatrixUserFunction(const TUserFunction& function, const char* name, Index columns, Matrix& value, Real t,
        const Vector3D& localPosition, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t) const;
    //! the coordinates the user functions get in a configuration: the reference ones, the total ones and their velocities (#3056)
    void GatherUserFunctionCoordinates(ResizableConstVector& qRef, ResizableConstVector& q, ResizableConstVector& q_t,
        ConfigurationType configuration) const;
    //! the acceleration coordinates of a configuration, zero in the reference configuration (#3056)
    void GatherAccelerations(ResizableConstVector& q_tt, ConfigurationType configuration) const;
    //! the result of a user function has the size the body needs, or a PyError names the user function and the sizes (#2995)
    void CheckResultSize(const char* name, Index rows, Index columns, Index requiredRows, Index requiredColumns) const;
    //! a Jacobian times the velocity coordinates, in a configuration (#2995)
    Vector3D JacobianTimesVelocity(bool rotation, const Vector3D& localPosition, ConfigurationType configuration) const;
    //! the matrices and vectors of the computations above, which keep their memory between the calls (#3046); each is
    //! written by a copy after a user function returned, so that a user function that asks the body again does not
    //! change what its caller reads
    mutable ResizableMatrix bufferJacobian;
    mutable ResizableMatrix bufferRotation;
    mutable ResizableVector bufferPlus;
    mutable ResizableVector bufferMinus;
    //! the mass matrix of constantMassMatrix, in the local coordinates of the body: computed once, as in the ANCF
    //! elements, and again after a change of a parameter or Assemble (#3058, approved storage of a value, D18)
    mutable bool massMatrixComputed = false;
    mutable EXUmath::MatrixContainer storedMassMatrix;
    CObjectUserBodyParameters parameters; //! AUTO: contains all parameters for CObjectUserBody

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectUserBodyParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectUserBodyParameters& GetParameters() const { return parameters; }

    //! AUTO:  no PotentialEnergy: the user function defines the forces
    virtual bool PotentialEnergyAvailable() const override
    {
        return false;
    }

    //! AUTO:  the user functions are called serially
    virtual bool HasUserFunction() const override
    {
        return true;
    }

    //! AUTO:  Computational function: compute mass matrix
    virtual void ComputeMassMatrix(EXUmath::MatrixContainer& massMatrixC, const ArrayIndex& ltg, bool computeInverse=false) const override;

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);
    }

    //! AUTO:  Flags to determine, which access (forces, moments, connectors, ...) to object are possible
    virtual AccessFunctionType GetAccessFunctionTypes() const override
    {
        return (AccessFunctionType)((Index)AccessFunctionType::TranslationalVelocity_qt + (Index)AccessFunctionType::AngularVelocity_qt + (Index)AccessFunctionType::JacobianTtimesVector_q);
    }

    //! AUTO:  the position Jacobian d(v)/d(q_t) at localPosition, 3 x n
    virtual void GetPositionJacobian(const Vector3D& localPosition, Matrix& value) const override;

    //! AUTO:  the rotation Jacobian d(omega)/d(q_t), omega global, 3 x n
    virtual void GetRotationJacobian(const Vector3D& localPosition, Matrix& value) const override;

    //! AUTO:  d(J_pos^T force + J_rot^T torque)/dq at localPosition, n x n; false if it is zero
    virtual bool GetJacobianTransposedTimesVectorDerivative(const Vector3D& localPosition, const Vector6D& forceTorque, Matrix& value) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableBody(OutputVariableType variableType, const Vector3D& localPosition, ConfigurationType configuration, Vector& value) const override;

    //! AUTO:  return the (global) position of 'localPosition' according to configuration type
    virtual Vector3D GetPosition(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) displacement of 'localPosition' according to configuration type
    virtual Vector3D GetDisplacement(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) velocity of 'localPosition' according to configuration type
    virtual Vector3D GetVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent rotation matrix of node; returns always a 3D matrix
    virtual Matrix3D GetRotationMatrix(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
    virtual Vector3D GetAngularVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent local (=body-fixed) angular velocity of node; in 2D case, this is the same as the global angular velocity; returns always a 3D Vector
    virtual Vector3D GetAngularVelocityLocal(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the local position of the center of mass, needed for equations of motion and for massProportionalLoad
    virtual Vector3D GetLocalCenterOfMass() const override
    {
        return Vector3D({0.,0.,0.});
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        return parameters.nodeNumbers[localIndex];
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual void SetNodeNumber(Index localIndex, Index nodeNumber) override
    {
        parameters.nodeNumbers[localIndex]=nodeNumber;
    }

    //! AUTO:  number of nodes; needed for every object; can depend on the configuration
    virtual Index GetNumberOfNodes() const override
    {
        return parameters.nodeNumbers.NumberOfItems();
    }

    //! AUTO:  number of ABRV:ODE2 coordinates; needed for object?
    virtual Index GetODE2Size() const override;

    //! AUTO:  Get type of object, e.g. to categorize and distinguish during assembly and computation
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded);
    }

    //! AUTO:  return true if object has time and coordinate independent (=constant) mass matrix
    virtual bool HasConstantMassMatrix() const override
    {
        return parameters.constantMassMatrix;
    }

    //! AUTO:  a changed parameter computes the constant mass matrix again (#3058)
    virtual void ParametersHaveChanged() override
    {
        massMatrixComputed = false;
    }

    //! AUTO:  Assemble computes the constant mass matrix again (#3058)
    virtual void PreAssemble() override
    {
        massMatrixComputed = false;
    }

    //! AUTO:  false in another thread than the main thread, which then gets no position (#3003); implemented in evaluateUserFunctions.cpp
    bool PythonCallPossible() const;

    //! AUTO:  the body, its markers and what depends on them are drawn in the main thread (#3003)
    virtual bool AccessFunctionsCallUserFunctions() const override
    {
        return true;
    }

    //! AUTO:  call of positionUserFunction; implemented in evaluateUserFunctions.cpp
    void EvaluatePosition(Vector3D& position, Real t, const Vector3D& localPosition, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t) const;

    //! AUTO:  call of positionJacobianUserFunction, with the size checked; implemented in evaluateUserFunctions.cpp
    void EvaluatePositionJacobian(Matrix& value, Real t, const Vector3D& localPosition, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t) const;

    //! AUTO:  call of rotationMatrixUserFunction, with the size checked; implemented in evaluateUserFunctions.cpp
    void EvaluateRotationMatrix(Matrix3D& value, Real t, const Vector3D& localPosition, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t) const;

    //! AUTO:  call of rotationJacobianUserFunction, with the size checked; implemented in evaluateUserFunctions.cpp
    void EvaluateRotationJacobian(Matrix& value, Real t, const Vector3D& localPosition, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t) const;

    //! AUTO:  call of outputVariableUserFunction; implemented in evaluateUserFunctions.cpp
    void EvaluateUserFunctionOutputVariable(Vector& value, Real t, OutputVariableType variableType, const Vector3D& localPosition, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t, const Vector& coordinates_tt) const;

    //! AUTO:  call of forceUserFunction, with the size checked; implemented in evaluateUserFunctions.cpp
    void EvaluateUserFunctionForce(Vector& force, Real t, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t) const;

    //! AUTO:  call of massMatrixUserFunction; implemented in evaluateUserFunctions.cpp
    void EvaluateUserFunctionMassMatrix(EXUmath::MatrixContainer& massMatrix, Real t, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t, const ArrayIndex& ltg) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index64)OutputVariableType::Position +
            (Index64)OutputVariableType::Displacement +
            (Index64)OutputVariableType::Velocity +
            (Index64)OutputVariableType::RotationMatrix +
            (Index64)OutputVariableType::HomogeneousTransformation +
            (Index64)OutputVariableType::AngularVelocity +
            (Index64)OutputVariableType::AngularVelocityLocal +
            (Index64)OutputVariableType::Coordinates +
            (Index64)OutputVariableType::Coordinates_t +
            (Index64)OutputVariableType::KineticEnergy +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::DisplacementLocal : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::VelocityLocal : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::Acceleration : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::AccelerationLocal : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::Rotation : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::AngularAcceleration : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::AngularAccelerationLocal : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::Director1 : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::Director2 : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::Director3 : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::Force : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::ForceLocal : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::Torque : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::TorqueLocal : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::StrainLocal : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::StressLocal : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::CurvatureLocal : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::PotentialEnergy : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::UserType0 : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::UserType1 : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::UserType2 : (Index64)0) +
            ((parameters.outputVariableUserFunction) ? (Index64)OutputVariableType::UserType3 : (Index64)0) );
    }

};



#endif //#ifdef include once...
