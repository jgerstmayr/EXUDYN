/** ***********************************************************************************************
* @class        CObjectConnectorUserParameters
* @brief        Parameter class for CObjectConnectorUser
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-11-01  00:21:53 (last modified)
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
    ArrayIndex markerNumbers;                     //!< AUTO: the markers: two Position markers for the forceUserFunction; at least one of any type for the markerAccessUserFunction; the user functions get them in this order
    Marker::Type requestedMarkerType;             //!< AUTO: the types every marker must provide, checked at Assemble, e.g. exu.MarkerType.Position, or exu.MarkerType.Position | exu.MarkerType.Orientation for torques; exu.MarkerType._None: markers of any type (#3105)
    Index nodeNumber;                             //!< AUTO: node number of a NodeGenericData, whose data coordinates the user functions get, e.g. the state of a contact; optional, needed for a postNewtonStepUserFunction (#3102)
    Vector parameters;                            //!< AUTO: the parameters of the connector, of any length, passed to the user function; see exudyn.advancedUtilities.ParameterLayout
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector
    PythonUserFunctionBase< std::function<StdVector(const MainSystem&,Real,Index,const HomogeneousTransformation&,const Twist&,const HomogeneousTransformation&,const Twist&,const StdVector&,const StdVector&)> > forceUserFunction;//!< AUTO: A Python or symbolic user function which returns the forces (and torques) on the two markers from their states (#3119); without it and without markerAccessUserFunction, the connector applies no forces
    PythonUserFunctionBase< std::function<void(const MainSystem&,Real,Index,py::object,const StdVector&,const StdVector&)> > markerAccessUserFunction;//!< AUTO: A Python user function which reads the markers and applies forces through them, also at points of their bodies it computes, e.g. for a contact on a flexible body; instead of forceUserFunction, not recordable (#2975)
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Real,Index,py::object,const StdVector&,const StdVector&,const StdVector&)> > postNewtonStepUserFunction;//!< AUTO: A Python user function which updates the data coordinates of the data node after each Newton iteration and returns them with the error of the switch, for contact, friction and other discontinuous states; needs nodeNumber (#3102)
    //! AUTO: default constructor with parameter initialization
    CObjectConnectorUserParameters()
    {
        markerNumbers = ArrayIndex();
        requestedMarkerType = Marker::_None;
        nodeNumber = EXUstd::InvalidIndex;
        parameters = Vector();
        activeConnector = true;
        forceUserFunction = 0;
        markerAccessUserFunction = 0;
        postNewtonStepUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectConnectorUser
* @brief        A connector of two markers whose forces are a user function of the frames and twists of the markers - their positions, rotations, velocities and angular velocities -: it returns the forces and torques on the markers, which the connector applies through the marker Jacobians - a spring-damper, a bushing, a contact of two points -, also as symbolic user function. For forces at points of a body that the function computes itself, as a contact searching the closest point on a flexible body, a second user function reads any number of markers of any type and applies forces through them. A NodeGenericData keeps a state, which a postNewtonStepUserFunction updates, as for contact or friction; its Jacobian is computed numerically.
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
        list.Append(&parameters.markerAccessUserFunction);
        list.Append(&parameters.postNewtonStepUserFunction);
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

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual void SetNodeNumber(Index localIndex, Index nodeNumber) override
    {
        parameters.nodeNumber=nodeNumber;
    }

    //! AUTO:  number of nodes; needed for every object; can depend on the configuration
    virtual Index GetNumberOfNodes() const override
    {
        return (parameters.nodeNumber != EXUstd::InvalidIndex);
    }

    //! AUTO:  flag to be set for connectors, which use DiscontinuousIteration
    virtual bool HasDiscontinuousIteration() const override
    {
        return (parameters.postNewtonStepUserFunction!=0);
    }

    //! AUTO:  function called after Newton method; returns a residual error (force)
    virtual Real PostNewtonStep(const MarkerDataStructure& markerDataCurrent, PostNewtonFlags::Type& flags, Real& recommendedStepSize) override;

    //! AUTO:  function called after discontinuous iterations have been completed for one step (e.g. to finalize history variables and set initial values for next step)
    virtual void PostDiscontinuousIterationStep() override
    {

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

    //! AUTO:  the requestedMarkerType of the parameters (#3105), with Position for a forceUserFunction (#3119)
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return (Marker::Type)(parameters.requestedMarkerType | (parameters.forceUserFunction != 0 ? Marker::Position : Marker::_None));
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

    //! AUTO:  call of forceUserFunction with the states of the two markers; the returned forces and torques by the marker Jacobians into the blocks of ode2Lhs (#3119); implemented in evaluateUserFunctions.cpp
    void EvaluateUserFunctionForce(Vector& ode2Lhs, const MarkerDataStructure& markerData, const ArrayIndex& blockSizes) const;

    //! AUTO:  call of markerAccessUserFunction with the ConnectorMarkers of the markers, each writing into its block of ode2Lhs (#2975); implemented in evaluateUserFunctions.cpp
    void EvaluateUserFunctionMarkerAccess(Vector& ode2Lhs, const MarkerDataStructure& markerData, const ArrayIndex& blockSizes) const;

    //! AUTO:  call of postNewtonStepUserFunction with ConnectorMarkers that only read; writes the new data coordinates and returns the error (#3102); implemented in evaluateUserFunctions.cpp
    Real EvaluateUserFunctionPostNewtonStep(const MarkerDataStructure& markerData, Vector& dataCoordinates, Real& recommendedStepSize) const;

    //! AUTO:  the data coordinates of the data node, empty without one (#3102); implemented in evaluateUserFunctions.cpp
    void GetDataCoordinates(ConfigurationType configuration, StdVector& data) const;

};



#endif //#ifdef include once...
