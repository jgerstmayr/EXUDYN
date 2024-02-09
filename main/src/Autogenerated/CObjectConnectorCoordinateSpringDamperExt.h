/** ***********************************************************************************************
* @class        CObjectConnectorCoordinateSpringDamperExtParameters
* @brief        Parameter class for CObjectConnectorCoordinateSpringDamperExt
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2024-02-02  20:40:01 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTCONNECTORCOORDINATESPRINGDAMPEREXTPARAMETERS__H
#define COBJECTCONNECTORCOORDINATESPRINGDAMPEREXTPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectConnectorCoordinateSpringDamperExtParameters
class CObjectConnectorCoordinateSpringDamperExtParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    Index nodeNumber;                             //!< AUTO: node number of a NodeGenericData for 3 data coordinates (friction mode, last sticking position, limit stop state), see description for details; must exist in case of bristle friction model or limit stops
    Real stiffness;                               //!< AUTO: stiffness [SI:N/m] of spring; acts against relative value of coordinates
    Real damping;                                 //!< AUTO: damping [SI:N/(m s)] of damper; acts against relative velocity of coordinates
    Real offset;                                  //!< AUTO: offset between two coordinates (reference length of springs), see equation; it can be used to represent the pre-scribed drive coordinate
    Real velocityOffset;                          //!< AUTO: offset between two coordinates; used to model D-control of a drive, where damping is not acting against prescribed velocity
    Real factor0;                                 //!< AUTO: marker 0 coordinate is multiplied with factor0
    Real factor1;                                 //!< AUTO: marker 1 coordinate is multiplied with factor1
    Real fDynamicFriction;                        //!< AUTO: dynamic (viscous) friction force [SI:N] against relative velocity when sliding; assuming a normal force \f$f_N\f$, the friction force can be interpreted as \f$f_\mu = \mu f_N\f$
    Real fStaticFrictionOffset;                   //!< AUTO: static (dry) friction offset force [SI:N]; assuming a normal force \f$f_N\f$, the friction force is limited by \f$f_\mu \le (\mu_{so} + \mu_d) f_N = f_{\mu_d} + f_{\mu_{so}}\f$
    Real stickingStiffness;                       //!< AUTO: stiffness of bristles in sticking case  [SI:N/m]
    Real stickingDamping;                         //!< AUTO: damping of bristles in sticking case  [SI:N/(m/s)]
    Real exponentialDecayStatic;                  //!< AUTO: relative velocity for exponential decay of static friction offset force [SI:m/s] against relative velocity; at \f$\Delta v = v_\mathrm{exp}\f$, the static friction offset force is reduced to 36.8\%
    Real fViscousFriction;                        //!< AUTO: viscous friction force part [SI:N/(m s)], acting against relative velocity in sliding case
    Real frictionProportionalZone;                //!< AUTO: if non-zero, a regularized Stribeck model is used, regularizing friction force around zero velocity - leading to zero friction force in case of zero velocity; this does not require a data node at all; if zero, the bristle model is used, which requires a data node which contains previous friction state and last sticking position
    Real limitStopsUpper;                         //!< AUTO: upper (maximum) value [SI:m] of coordinate before limit is activated; defined relative to the two marker coordinates
    Real limitStopsLower;                         //!< AUTO: lower (minimum) value [SI:m] of coordinate before limit is activated; defined relative to the two marker coordinates
    Real limitStopsStiffness;                     //!< AUTO: stiffness [SI:N/m] of limit stop (contact stiffness); following a linear contact model
    Real limitStopsDamping;                       //!< AUTO: damping [SI:N/(m/s)] of limit stop (contact damping); following a linear contact model
    bool useLimitStops;                           //!< AUTO: if True, limit stops are considered and parameters must be set accordingly; furthermore, the NodeGenericData must have 3 data coordinates
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    PythonUserFunctionBase< std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real)> > springForceUserFunction;//!< AUTO: A Python function which defines the spring force with 8 parameters, see equations section / see description below
    //! AUTO: default constructor with parameter initialization
    CObjectConnectorCoordinateSpringDamperExtParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        nodeNumber = EXUstd::InvalidIndex;
        stiffness = 0.;
        damping = 0.;
        offset = 0.;
        velocityOffset = 0.;
        factor0 = 1.;
        factor1 = 1.;
        fDynamicFriction = 0.;
        fStaticFrictionOffset = 0.;
        stickingStiffness = 0.;
        stickingDamping = 0.;
        exponentialDecayStatic = 1.e-3;
        fViscousFriction = 0.;
        frictionProportionalZone = 0.;
        limitStopsUpper = 0.;
        limitStopsLower = 0.;
        limitStopsStiffness = 0.;
        limitStopsDamping = 0.;
        useLimitStops = false;
        activeConnector = true;
        springForceUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectConnectorCoordinateSpringDamperExt
* @brief        A 1D (scalar) spring-damper element acting on single \hac{ODE2} coordinates; same as ObjectConnectorCoordinateSpringDamper but with extended features, such as limit stop and improved friction; has different user function interface and additional data node as compared to ObjectConnectorCoordinateSpringDamper, but otherwise behaves very similar. The CoordinateSpringDamperExt is very useful for a single axis of a robot or similar machine modelled with a KinematicTree, as it can add friction and limits based on physical properties. It is highly recommended, to use the bristle model for friction with frictionProportionalZone=0 in case of implicit integrators (GeneralizedAlpha) as it converges better.
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

//! AUTO: CObjectConnectorCoordinateSpringDamperExt
class CObjectConnectorCoordinateSpringDamperExt: public CObjectConnector // AUTO: 
{
protected: // AUTO: 
    CObjectConnectorCoordinateSpringDamperExtParameters parameters; //! AUTO: contains all parameters for CObjectConnectorCoordinateSpringDamperExt

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectConnectorCoordinateSpringDamperExtParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectConnectorCoordinateSpringDamperExtParameters& GetParameters() const { return parameters; }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;
    }

    //! AUTO:  number of nodes depending on configuration; needed for every object
    virtual Index GetNumberOfNodes() const override
    {
        return (parameters.nodeNumber==EXUstd::InvalidIndex) ? 0 : 1;
    }

    //! AUTO:  needed in order to create ltg-lists for data variable of connector
    virtual Index GetDataVariablesSize() const override
    {
        return 3;
    }

    //! AUTO:  return true, if object has a computation user function
    virtual bool HasUserFunction() const override
    {
        return (parameters.springForceUserFunction!=0);
    }

    //! AUTO:  default function to return Marker numbers
    virtual const ArrayIndex& GetMarkerNumbers() const override
    {
        return parameters.markerNumbers;
    }

    //! AUTO:  connector uses penalty formulation
    virtual bool IsPenaltyConnector() const override
    {
        return true;
    }

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs, const MarkerDataStructure& markerData, Index objectNumber) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override;

    //! AUTO:  Computational function: compute Jacobian of \hac{ODE2} \ac{LHS} equations w.r.t. ODE2 coordinates and ODE2 velocities; write either dense local jacobian into dense matrix of MatrixContainer or ADD sparse triplets INCLUDING ltg mapping to sparse matrix of MatrixContainer
    virtual void ComputeJacobianODE2_ODE2(EXUmath::MatrixContainer& jacobianODE2, JacobianTemp& temp, Real factorODE2, Real factorODE2_t, Index objectNumber, const ArrayIndex& ltg, const MarkerDataStructure& markerData) const override;

    //! AUTO:  flag to be set for connectors, which use DiscontinuousIteration
    virtual bool HasDiscontinuousIteration() const override
    {
        return (( (parameters.fDynamicFriction != 0 || parameters.fStaticFrictionOffset != 0) && parameters.frictionProportionalZone == 0) || parameters.useLimitStops);
    }

    //! AUTO:  function called after Newton method; returns a residual error (force)
    virtual Real PostNewtonStep(const MarkerDataStructure& markerDataCurrent, Index itemIndex, PostNewtonFlags::Type& flags, Real& recommendedStepSize) override;

    //! AUTO:  function called after discontinuous iterations have been completed for one step (e.g. to finalize history variables and set initial values for next step)
    virtual void PostDiscontinuousIterationStep() override;

    //! AUTO:  compute global 6D force and torque which is used for computation of derivative of jacobian; used only in combination with ComputeJacobianODE2_ODE2
    virtual void ComputeJacobianForce6D(const MarkerDataStructure& markerData, Index objectNumber, Vector6D& force6D) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Index itemIndex, Vector& value) const override;

    //! AUTO:  provide requested markerType for connector
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return Marker::Coordinate;
    }

    //! AUTO:  return object type (for node treatment in computation)
    virtual CObjectType GetType() const override
    {
        return CObjectType::Connector;
    }

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

    //! AUTO:  compute spring damper force helper function
    void ComputeSpringForce(const MarkerDataStructure& markerData, Index itemIndex, Real& relPos, Real& relVel, Real& force) const;

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionForce(Real& force, const MainSystemBase& mainSystem, Real t, Index itemIndex, Real relPos, Real relVel) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::Force );
    }

};



#endif //#ifdef include once...
