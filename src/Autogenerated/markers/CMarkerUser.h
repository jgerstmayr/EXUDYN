/** ***********************************************************************************************
* @class        CMarkerUserParameters
* @brief        Parameter class for CMarkerUser
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-25  14:35:54 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef CMARKERUSERPARAMETERS__H
#define CMARKERUSERPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
namespace py = pybind11;            //! AUTO: "py" used throughout in code
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CMarkerUserParameters
class CMarkerUserParameters // AUTO:
{
public: // AUTO:
    ArrayIndex bodyNumbers;                       //!< AUTO: the bodies whose coordinates the user functions get, at least one
    Vector parameters;                            //!< AUTO: the parameters of the marker, of any length, passed to the user functions; see exudyn.advancedUtilities.ParameterLayout
    PythonUserFunctionBase< std::function<StdVector3D(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > positionUserFunction;//!< AUTO: A Python user function which computes the global position of the marker; it must be given
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > positionJacobianUserFunction;//!< AUTO: A Python user function which computes the position Jacobian; by central differences of the position if not given
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > rotationMatrixUserFunction;//!< AUTO: A Python user function which computes the rotation matrix of the marker; optional, for rigid markers
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > rotationJacobianUserFunction;//!< AUTO: A Python user function which computes the rotation Jacobian (of the global angular velocity); by central differences of the rotation matrix if not given
    //! AUTO: default constructor with parameter initialization
    CMarkerUserParameters()
    {
        bodyNumbers = ArrayIndex();
        parameters = Vector();
        positionUserFunction = 0;
        positionJacobianUserFunction = 0;
        rotationMatrixUserFunction = 0;
        rotationJacobianUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CMarkerUser
* @brief        A marker on any number of bodies whose position, and optionally rotation, are computed by Python user functions from the coordinates of the nodes of these bodies, e.g. a point between several bodies or nodes, or a point of a mesh. Loads, connectors, joints and sensors use it as a position or rigid marker; the Jacobians are computed by central differences if they are not given.
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

//! AUTO: CMarkerUser
class CMarkerUser: public CMarker // AUTO:
{
protected: // AUTO:
    //! the coordinates of the nodes of the bodies, body by body: reference, total and velocity coordinates of a configuration
    void GatherCoordinates(const CSystemData& cSystemData, ConfigurationType configuration, Vector& qRef, Vector& q, Vector& q_t) const;
    //! the number of the marker in the system, for the user functions
    Index GetMarkerNumber(const CSystemData& cSystemData) const;
    //! a matrix user function, its size checked
    template<class TUserFunction>
    void EvaluateMatrixUserFunction(const CSystemData& cSystemData, const TUserFunction& function, const char* name, Index rows, Index columns,
        Matrix& value, Real t, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t) const;
    CMarkerUserParameters parameters; //! AUTO: contains all parameters for CMarkerUser

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CMarkerUserParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CMarkerUserParameters& GetParameters() const { return parameters; }

    //! AUTO: the user functions of the item, for the parallel computation (#2978)
    virtual void GetUserFunctions(ResizableArray<const UserFunctionBase*>& list) const override
    {
        list.Append(&parameters.positionUserFunction);
        list.Append(&parameters.positionJacobianUserFunction);
        list.Append(&parameters.rotationMatrixUserFunction);
        list.Append(&parameters.rotationJacobianUserFunction);
    }

    //! AUTO:  general access to object number
    virtual Index GetObjectNumber(Index localIndex = 0) const override
    {
        return parameters.bodyNumbers[localIndex];
    }

    //! AUTO:  change bodyNumber
    virtual void SetObjectNumber(Index objectNumber, Index localIndex = 0) override
    {
        parameters.bodyNumbers[localIndex] = objectNumber;
    }

    //! AUTO:  general access to object number
    virtual Index GetNumberOfObjects() const override
    {
        return parameters.bodyNumbers.NumberOfItems();
    }

    //! AUTO:  Orientation only with a rotationMatrixUserFunction, so that a joint or torque on a marker without it is refused at Assemble
    virtual Marker::Type GetType() const override
    {
        return parameters.rotationMatrixUserFunction ? (Marker::Type)(Marker::Body + Marker::Object + Marker::Position + Marker::Orientation) : (Marker::Type)(Marker::Body + Marker::Object + Marker::Position);
    }

    //! AUTO:  return dimension of connector, which an attached connector would have; for coordinate markers, it gives the number of coordinates used by the marker
    virtual Index GetDimension(const CSystemData& cSystemData) const override
    {
        return 3;
    }

    //! AUTO:  return position of marker
    virtual void GetPosition(const CSystemData& cSystemData, Vector3D& position, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return velocity of marker
    virtual void GetVelocity(const CSystemData& cSystemData, Vector3D& velocity, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent rotation matrix of node; returns always a 3D matrix
    virtual void GetRotationMatrix(const CSystemData& cSystemData, Matrix3D& rotationMatrix, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent angular velocity of node; returns always a 3D Vector
    virtual void GetAngularVelocity(const CSystemData& cSystemData, Vector3D& angularVelocity, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent local (=body-fixed) angular velocity of node; in 2D case, this is the same as the global angular velocity; returns always a 3D Vector
    virtual void GetAngularVelocityLocal(const CSystemData& cSystemData, Vector3D& angularVelocity, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  Compute marker data (e.g. position and positionJacobian) for a marker
    virtual void ComputeMarkerData(const CSystemData& cSystemData, bool computeJacobian, MarkerData& markerData) const override;

    //! AUTO:  the marker and what depends on it are drawn in the main thread (#3003)
    virtual bool AccessFunctionsCallUserFunctions() const override
    {
        return true;
    }

    //! AUTO:  call of positionUserFunction; implemented in evaluateUserFunctions.cpp
    void EvaluatePosition(const CSystemData& cSystemData, Vector3D& position, Real t, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t) const;

    //! AUTO:  call of positionJacobianUserFunction, or central differences of the position; implemented in evaluateUserFunctions.cpp
    void EvaluatePositionJacobian(const CSystemData& cSystemData, Matrix& value, Real t, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t) const;

    //! AUTO:  call of rotationMatrixUserFunction; implemented in evaluateUserFunctions.cpp
    void EvaluateRotationMatrix(const CSystemData& cSystemData, Matrix3D& value, Real t, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t) const;

    //! AUTO:  call of rotationJacobianUserFunction, or central differences of the rotation matrix; implemented in evaluateUserFunctions.cpp
    void EvaluateRotationJacobian(const CSystemData& cSystemData, Matrix& value, Real t, const Vector& qRef, const Vector& coordinates, const Vector& coordinates_t) const;

};



#endif //#ifdef include once...
