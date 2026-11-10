/** ***********************************************************************************************
* @class        CObjectContactSphereCylinderParameters
* @brief        Parameter class for CObjectContactSphereCylinder
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-11-10  08:34:50 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef COBJECTCONTACTSPHERECYLINDERPARAMETERS__H
#define COBJECTCONTACTSPHERECYLINDERPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
namespace py = pybind11;            //! AUTO: "py" used throughout in code

//! AUTO: Parameters for class CObjectContactSphereCylinderParameters
class CObjectContactSphereCylinderParameters // AUTO:
{
public: // AUTO:
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers: the center of the sphere (marker 0) and the center of the cylinder (marker 1), whose local z axis is the axis of the cylinder
    Index nodeNumber;                             //!< AUTO: node number of a NodeGenericData with numberOfDataCoordinates = 3, needed for the discontinuous iteration (friction and contact); the data variables hold the values of the last PostNewton iteration: data[0] the gap, data[1] the norm of the tangential velocity (stick or slip), data[2] the impact velocity
    Real sphereRadius;                            //!< AUTO: must be > 0; radius of the sphere [SI:m]
    Real cylinderRadius;                          //!< AUTO: must be > 0; radius of the cylinder [SI:m]
    Real cylinderLength;                          //!< AUTO: must be > 0; length of the cylinder along the local z axis of marker 1, from \f$-L/2\f$ to \f$L/2\f$ [SI:m]
    Real dynamicFriction;                         //!< AUTO: must be >= 0; dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, [](#sec-module-physics)
    Real frictionProportionalZone;                //!< AUTO: must be >= 0; limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), [](#sec-module-physics)
    Real contactStiffness;                        //!< AUTO: must be >= 0; normal contact stiffness [SI:N/m] (units in case that \f$n_\mathrm{exp}=1\f$)
    Real contactDamping;                          //!< AUTO: must be >= 0; linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior; see [](#sec-contact-damping) for the negative contact force it gives at the end of a contact
    Real contactStiffnessExponent;                //!< AUTO: must be > 0; exponent in normal contact model [SI:1]
    Real restitutionCoefficient;                  //!< AUTO: must be > 0; coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)
    Real minimumImpactVelocity;                   //!< AUTO: must be >= 0; minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)
    ContactImpactModel impactModel;               //!< AUTO: the impact model, see ContactImpactModel: Linear (only the linear contactDamping), HuntCrossley or GonthierCarvalhoMartins, the latter much more accurate regarding the coefficient of restitution in the full range (0, 1]; the integers 0, 1, 2 are accepted as well; in all models, the linear contactDamping is added, if not set to zero (#3327)
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    PythonUserFunctionBase< std::function<StdVector3D(const MainSystem&,Real,Index,Real,Real,StdVector3D,StdVector3D,Real)> > contactForceUserFunction;//!< AUTO: A Python function which replaces the contact law and returns the contact force on marker 0, while the item computes the geometry and the contact state; the contact parameters of the item are then not used, except dynamicFriction > 0 for the rotation of marker 0; see description below (#3324)
    //! AUTO: default constructor with parameter initialization
    CObjectContactSphereCylinderParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        nodeNumber = EXUstd::InvalidIndex;
        sphereRadius = 0.;
        cylinderRadius = 0.;
        cylinderLength = 0.;
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
* @class        CObjectContactSphereCylinder
* @brief        A penalty contact between a sphere (marker 0) and a finite cylinder (marker 1), with its side, its caps and its rims; the cylinder is centered at marker 1 along its local z axis. Contact law and friction as for ObjectContactSphereTriangle.
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

//! AUTO: CObjectContactSphereCylinder
class CObjectContactSphereCylinder: public CObjectConnector // AUTO:
{
protected: // AUTO:
    static constexpr Index nDataVariables = 3; //number of data variables for tangential and normal contact
    static constexpr Index dataIndexGap = 0; //!< index in data node representing gap
    static constexpr Index dataIndexVtangent = 1; //!< index in data node representing tangent velocity
    static constexpr Index dataIndexImpactVel = 2; //!< index in data node representing last impact velocity
    CObjectContactSphereCylinderParameters parameters; //! AUTO: contains all parameters for CObjectContactSphereCylinder

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectContactSphereCylinderParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectContactSphereCylinderParameters& GetParameters() const { return parameters; }

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
    virtual void ComputeConnectorForceRigid(FlatArray<const MarkerRigid<Real>> markers, Real t, FlatArray<Vector3D> forces, FlatArray<Vector3D> torques) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Vector& value) const override;

    //! AUTO:  the contact kinematics and forces: the gap, the vector from the sphere center to the closest point of the cylinder, the relative velocity there, the force on marker 1 and the normal
    void ComputeConnectorProperties(const MarkerDataStructure& markerData, const LinkedDataVector& data, Real& frictionCoeff, Real& gap, Vector3D& deltaP, Vector3D& deltaV, Vector3D& fVec, Vector3D& fFriction, Vector3D& n0, bool contactFromData = true) const;

    //! AUTO:  provide requested markerType for connector
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return (Marker::Type)((Index)Marker::Position + (parameters.dynamicFriction != 0)*(Index)Marker::Orientation);
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
