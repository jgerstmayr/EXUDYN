/** ***********************************************************************************************
* @class        CNodeUserParameters
* @brief        Parameter class for CNodeUser
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-22  21:06:35 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef CNODEUSERPARAMETERS__H
#define CNODEUSERPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
namespace py = pybind11;            //! AUTO: "py" used throughout in code
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CNodeUserParameters
class CNodeUserParameters // AUTO:
{
public: // AUTO:
    Index numberOfODE2Coordinates;                //!< AUTO: must be > 0; number of ABRV:ODE2 coordinates
    Vector referenceCoordinates;                  //!< AUTO: reference coordinates of the node; must have numberOfODE2Coordinates components
    Vector parameters;                            //!< AUTO: the parameters of the node, passed to every user function; their meaning is defined by the user functions, see exudyn.advancedUtilities.ParameterLayout; empty by default
    PythonUserFunctionBase< std::function<StdVector3D(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > positionUserFunction;//!< AUTO: A Python user function which computes the global position of the node; it must be given
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > positionJacobianUserFunction;//!< AUTO: A Python user function which computes the position Jacobian; by central differences of the position if not given
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > rotationMatrixUserFunction;//!< AUTO: A Python user function which computes the rotation matrix of the node; optional, for rigid markers
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > rotationJacobianUserFunction;//!< AUTO: A Python user function which computes the rotation Jacobian (of the global angular velocity); by central differences of the rotation matrix if not given
    //! AUTO: default constructor with parameter initialization
    CNodeUserParameters()
    {
        numberOfODE2Coordinates = 0;
        referenceCoordinates = Vector();
        parameters = Vector();
        positionUserFunction = 0;
        positionJacobianUserFunction = 0;
        rotationMatrixUserFunction = 0;
        rotationJacobianUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CNodeUser
* @brief        A node with any number of ABRV:ODE2 coordinates, whose position, and optionally orientation, are computed from its coordinates by Python user functions: a node for elements whose nodes are not in Exudyn, such as an ANCF node with full gradients. The node markers (MarkerNodePosition, MarkerNodeRigid, MarkerNodeCoordinate), SensorNode, loads on these markers and every connector work on it.
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

//! AUTO: CNodeUser
class CNodeUser: public CNodeODE2 // AUTO:
{
protected: // AUTO:
    CSystemData* cSystemData = nullptr; //!< for the MainSystem and the time of the user functions
    //! the displacement coordinates of a configuration, zero in the reference configuration (#2974)
    void Displacements(ResizableConstVector& q, ConfigurationType configuration) const;
    //! the number of the node in the system, for the user functions
    Index GetNodeNumber() const;
    //! false in another thread than the main thread
    bool PythonCallPossible() const;
    //! a matrix user function, its size checked
    template<class TUserFunction>
    void EvaluateMatrixUserFunction(const TUserFunction& function, const char* name, Index rows, Index columns, Matrix& value, Real t,
        const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t) const;
    //! the velocity coordinates of a configuration, zero in the reference configuration (#3056)
    void Velocities(ResizableConstVector& q_t, ConfigurationType configuration) const;
    //! the matrix of the rotation user function, which keeps its memory between the calls (#3049)
    mutable ResizableMatrix bufferRotation;
    CNodeUserParameters parameters; //! AUTO: contains all parameters for CNodeUser

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CNodeUserParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CNodeUserParameters& GetParameters() const { return parameters; }

    //! AUTO:  return number of second order diff. eq. coordinates
    virtual Index GetNumberOfODE2Coordinates() const override
    {
        return parameters.numberOfODE2Coordinates;
    }

    //! AUTO:  return node type (for node treatment in computation); the orientation needs rotationMatrixUserFunction
    virtual Node::Type GetType() const override
    {
        return (Node::Type)((Index)Node::Position + (Index)Node::Orientation);
    }

    //! AUTO:  return configuration dependent position of node; returns always a 3D Vector
    virtual Vector3D GetPosition(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent velocity of node; returns always a 3D Vector
    virtual Vector3D GetVelocity(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent acceleration of node
    virtual Vector3D GetAcceleration(ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent rotation matrix of node; returns always a 3D matrix
    virtual Matrix3D GetRotationMatrix(ConfigurationType configuration = ConfigurationType::Current) const override;

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

    //! AUTO:  the system of the node, for the MainSystem and the time of the user functions (#2974)
    virtual void SetCSystemData(CSystemData* cSystemDataInit) override
    {
        cSystemData = cSystemDataInit;
    }

    //! AUTO:  the node, its markers and what depends on them are drawn in the main thread (#3003)
    virtual bool AccessFunctionsCallUserFunctions() const override
    {
        return true;
    }

    //! AUTO:  call of positionUserFunction; implemented in evaluateUserFunctions.cpp
    void EvaluatePosition(Vector3D& position, Real t, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t) const;

    //! AUTO:  call of positionJacobianUserFunction, or central differences of the position; implemented in evaluateUserFunctions.cpp
    void EvaluatePositionJacobian(Matrix& value, Real t, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t) const;

    //! AUTO:  call of rotationMatrixUserFunction; implemented in evaluateUserFunctions.cpp
    void EvaluateRotationMatrix(Matrix3D& value, Real t, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t) const;

    //! AUTO:  call of rotationJacobianUserFunction, or central differences of the rotation matrix; implemented in evaluateUserFunctions.cpp
    void EvaluateRotationJacobian(Matrix& value, Real t, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index64)OutputVariableType::Position +
            (Index64)OutputVariableType::Displacement +
            (Index64)OutputVariableType::Velocity +
            (Index64)OutputVariableType::Acceleration +
            ((parameters.rotationMatrixUserFunction) ? (Index64)OutputVariableType::RotationMatrix : (Index64)0) +
            ((parameters.rotationMatrixUserFunction) ? (Index64)OutputVariableType::HomogeneousTransformation : (Index64)0) +
            ((parameters.rotationMatrixUserFunction) ? (Index64)OutputVariableType::AngularVelocity : (Index64)0) +
            ((parameters.rotationMatrixUserFunction) ? (Index64)OutputVariableType::AngularVelocityLocal : (Index64)0) +
            (Index64)OutputVariableType::CoordinatesTotal +
            (Index64)OutputVariableType::Coordinates +
            (Index64)OutputVariableType::Coordinates_t +
            (Index64)OutputVariableType::Coordinates_tt );
    }

};



#endif //#ifdef include once...
