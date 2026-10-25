/** ***********************************************************************************************
* @class        CObjectConnectorUserParameters
* @brief        Parameter class for CObjectConnectorUser
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

#ifndef COBJECTCONNECTORUSERPARAMETERS__H
#define COBJECTCONNECTORUSERPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
namespace py = pybind11;            //! AUTO: "py" used throughout in code
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectConnectorUserParameters
class CObjectConnectorUserParameters // AUTO:
{
public: // AUTO:
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers of any type, at least one; the user function gets them in this order
    Vector parameters;                            //!< AUTO: the parameters of the connector, of any length, passed to the user function; see exudyn.advancedUtilities.ParameterLayout
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector
    PythonUserFunctionBase< std::function<void(const MainSystem&,Real,Index,py::object,const StdVector&)> > forceUserFunction;//!< AUTO: A Python user function which computes the forces of the connector and applies them through its markers; without it, the connector applies no forces
    //! AUTO: default constructor with parameter initialization
    CObjectConnectorUserParameters()
    {
        markerNumbers = ArrayIndex();
        parameters = Vector();
        activeConnector = true;
        forceUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectConnectorUser
* @brief        A connector whose forces are computed by a Python user function with access to its markers: for each marker it reads the position, velocity, rotation and coordinates of the marker, and for a body marker the positions and velocities anywhere on the body, and it applies forces and torques through the marker - at the marker or at any local position of the marker's body. So a contact can search the closest point on a flexible body and act there. It links any number of markers of any type; its Jacobian is computed numerically.
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

//! AUTO: CObjectConnectorUser
class CObjectConnectorUser: public CObjectConnector // AUTO:
{
protected: // AUTO:
    CObjectConnectorUserParameters parameters; //! AUTO: contains all parameters for CObjectConnectorUser

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectConnectorUserParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectConnectorUserParameters& GetParameters() const { return parameters; }

    //! AUTO: the user functions of the item, for the parallel computation (#2978)
    virtual void GetUserFunctions(ResizableArray<const UserFunctionBase*>& list) const override
    {
        list.Append(&parameters.forceUserFunction);
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

    //! AUTO:  number of markers; 0 means no requirements
    virtual Index RequestedNumberOfMarkers() const override
    {
        return 0;
    }

    //! AUTO:  return true, if object has a computation user function
    virtual bool HasUserFunction() const override
    {
        return true;
    }

    //! AUTO:  true if the connector uses a penalty formulation; false if the constraint uses Lagrange multipliers
    virtual bool IsPenaltyConnector() const override
    {
        return true;
    }

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs, const MarkerDataStructure& markerData) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);
    }

    //! AUTO:  provide requested markerType for connector
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return Marker::_None;
    }

    //! AUTO:  Get type of object, e.g. to decide for loads, markers, ... which type of computation is needed
    virtual CObjectType GetType() const override
    {
        return CObjectType::Connector;
    }

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

    //! AUTO:  no output variables: what the connector computes is in its user function
    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return OutputVariableType::_None;
    }

    //! AUTO:  call of forceUserFunction with the ConnectorMarkers of the markers, each writing into its block of ode2Lhs (#2975); implemented in evaluateUserFunctions.cpp
    void EvaluateUserFunctionForce(Vector& ode2Lhs, const MarkerDataStructure& markerData, const ArrayIndex& blockSizes) const;

};



#endif //#ifdef include once...
