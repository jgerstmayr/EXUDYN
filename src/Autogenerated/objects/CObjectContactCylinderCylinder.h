/** ***********************************************************************************************
* @class        CObjectContactCylinderCylinderParameters
* @brief        Parameter class for CObjectContactCylinderCylinder
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-11-07  15:39:40 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef COBJECTCONTACTCYLINDERCYLINDERPARAMETERS__H
#define COBJECTCONTACTCYLINDERCYLINDERPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
namespace py = pybind11;            //! AUTO: "py" used throughout in code

//! AUTO: Parameters for class CObjectContactCylinderCylinderParameters
class CObjectContactCylinderCylinderParameters // AUTO:
{
public: // AUTO:
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers: the centers of cylinder 0 and cylinder 1, the local z axis of each marker the axis of its cylinder
    Index nodeNumber;                             //!< AUTO: node number of a NodeGenericData with numberOfDataCoordinates = 12, needed for the discontinuous iteration (friction and contact): a block of 3 for each of the up to 4 contact points \f$i\f$ holds the values of the last PostNewton iteration: data[3i] the gap, data[3i+1] the norm of the tangential velocity (stick or slip), data[3i+2] the impact velocity
    Vector2D cylindersRadii;                      //!< AUTO: the radii of cylinder 0 and cylinder 1, both > 0 [SI:m]
    Vector2D cylindersLengths;                    //!< AUTO: the lengths of cylinder 0 and cylinder 1 along the local z axes of their markers, from \f$-L_i/2\f$ to \f$L_i/2\f$, both > 0 [SI:m]
    Real dynamicFriction;                         //!< AUTO: must be >= 0; dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, [](#sec-module-physics)
    Real frictionProportionalZone;                //!< AUTO: must be >= 0; limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), [](#sec-module-physics)
    Real contactStiffness;                        //!< AUTO: must be >= 0; normal contact stiffness [SI:N/m] (units in case that \f$n_\mathrm{exp}=1\f$)
    Real contactDamping;                          //!< AUTO: must be >= 0; linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
    Real contactStiffnessExponent;                //!< AUTO: must be > 0; exponent in normal contact model [SI:1]
    Real restitutionCoefficient;                  //!< AUTO: must be > 0; coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)
    Real minimumImpactVelocity;                   //!< AUTO: must be >= 0; minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)
    ContactImpactModel impactModel;               //!< AUTO: the impact model, see ContactImpactModel: Linear (only the linear contactDamping), HuntCrossley or GonthierCarvalhoMartins, the latter much more accurate regarding the coefficient of restitution in the full range (0, 1]; the integers 0, 1, 2 are accepted as well; in all models, the linear contactDamping is added, if not set to zero (#3327)
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    PythonUserFunctionBase< std::function<StdVector3D(const MainSystem&,Real,Index,Real,Real,StdVector3D,StdVector3D,Real)> > contactForceUserFunction;//!< AUTO: A Python function which replaces the contact law and returns the contact force on marker 0, while the item computes the geometry and the contact state; the contact parameters of the item are then not used; see description below (#3324)
    //! AUTO: default constructor with parameter initialization
    CObjectContactCylinderCylinderParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        nodeNumber = EXUstd::InvalidIndex;
        cylindersRadii = Vector2D({0.,0.});
        cylindersLengths = Vector2D({0.,0.});
        dynamicFriction = 0.;
        frictionProportionalZone = 0.001;
        contactStiffness = 0.;
        contactDamping = 0.;
        contactStiffnessExponent = 1.;
        restitutionCoefficient = 1.;
        minimumImpactVelocity = 0.;
        impactModel = ContactImpactModel::Linear;
        activeConnector = true;
        contactForceUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectContactCylinderCylinder
* @brief        A penalty contact between two finite cylinders, each centered at its marker along the local z axis of the marker, with their sides, caps and rims; up to 4 contact points carry a line or a face of contact. Contact law and friction as for ObjectContactSphereTriangle.
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

//! AUTO: CObjectContactCylinderCylinder
class CObjectContactCylinderCylinder: public CObjectConnector // AUTO:
{
protected: // AUTO:
    CObjectContactCylinderCylinderParameters parameters; //! AUTO: contains all parameters for CObjectContactCylinderCylinder

public: // AUTO:
    static constexpr Index nDataVariables = 12; //!< a block of 3 per contact point (4): gap, tangential velocity, impact velocity

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectContactCylinderCylinderParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectContactCylinderCylinderParameters& GetParameters() const { return parameters; }

    //! AUTO: the user functions of the item, for the parallel computation (#2978)
    virtual void GetUserFunctions(ResizableArray<const UserFunctionBase*>& list) const override
    {
        list.Append(&parameters.contactForceUserFunction);
    }

    //! AUTO:  return true, if object has a computation user function
    virtual bool HasUserFunction() const override
    {
        return (parameters.contactForceUserFunction!=0);
    }

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionForce(Vector3D& force, const MainSystemBase& mainSystem, Real t, Real gap, Real gap_t, const Vector3D& normal, const Vector3D& tangentialVelocity, Real impactVelocity) const;

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
        return 1;
    }

    //! AUTO:  data (history) variable simplifies contact switching for implicit time integration and Newton method
    virtual Index GetDataVariablesSize() const override
    {
        return nDataVariables;
    }

    //! AUTO:  flag to be set for connectors, which use DiscontinuousIteration
    virtual bool HasDiscontinuousIteration() const override
    {
        return true;
    }

    //! AUTO:  function called after Newton method; returns a residual error (force)
    virtual Real PostNewtonStep(const MarkerDataStructure& markerDataCurrent, PostNewtonFlags::Type& flags, Real& recommendedStepSize) override;

    //! AUTO:  function called after discontinuous iterations have been completed for one step (e.g. to finalize history variables and set initial values for next step)
    virtual void PostDiscontinuousIterationStep() override;

    //! AUTO:  true if the connector uses a penalty formulation; false if the constraint uses Lagrange multipliers
    virtual bool IsPenaltyConnector() const override
    {
        return true;
    }

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t + JacobianType::ODE2_ODE2_function + JacobianType::ODE2_ODE2_t_function);
    }

    //! AUTO:  the right-hand side on the connector interface of rigid markers (#2745)
    virtual bool ComputeODE2LHSConnector(const CSystemData& systemData, TemporaryComputationData& temp, Vector& localODE2Lhs) const override
    {
        ConnectorODE2LHSRigidMarkers(systemData, temp, *this, localODE2Lhs); return true;
    }

    //! AUTO:  the Jacobian by local numerical differentiation of the force by the marker kinematics (#3201)
    virtual bool ComputeJacobianODE2Connector(const CSystemData& systemData, TemporaryComputationData& temp, Real factorODE2, Real factorODE2_t, bool jacobianDerivativeNonZero) const override
    {
        ConnectorJacobianODE2RigidMarkersNumerical(systemData, temp, *this, factorODE2, factorODE2_t, jacobianDerivativeNonZero); return true;
    }

    //! AUTO:  its Jacobian contains dv/dq and domega/dq of the markers (#3201)
    virtual bool RequestsVelocityDerivative() const override
    {
        return true;
    }

    //! AUTO:  the force and torque on each marker from the kinematics of the two markers (#2745)
    virtual void ComputeConnectorForceRigid(const MarkerRigid<Real>* markers, Real t, Vector3D* forces, Vector3D* torques) const override;

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
        return CObjectType::Connector;
    }

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index64)OutputVariableType::Distance +
            (Index64)OutputVariableType::Position +
            (Index64)OutputVariableType::Displacement +
            (Index64)OutputVariableType::Velocity +
            (Index64)OutputVariableType::DisplacementLocal +
            (Index64)OutputVariableType::Director1 +
            (Index64)OutputVariableType::Force +
            (Index64)OutputVariableType::Torque );
    }

};



#endif //#ifdef include once...
