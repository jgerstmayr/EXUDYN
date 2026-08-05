/** ***********************************************************************************************
* @class        CObjectContactSphereTorusParameters
* @brief        Parameter class for CObjectContactSphereTorus
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-02-05  22:13:58 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTCONTACTSPHERETORUSPARAMETERS__H
#define COBJECTCONTACTSPHERETORUSPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CObjectContactSphereTorusParameters
class CObjectContactSphereTorusParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers representing centers of sphere (marker 0) and center of torus (marker 1)
    Index nodeNumber;                             //!< AUTO: node number of a NodeGenericData with numberOfDataCoordinates = 4 dataCoordinates, needed for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the  gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused.
    Real radiusSphere;                            //!< AUTO:  radius of sphere [SI:m]
    Real torusMajorRadius;                        //!< AUTO:  major radius of torus [SI:m], representing center of rotated circle
    Real torusMinorRadius;                        //!< AUTO:  minor radius of torus [SI:m], representing radius of circle of ring
    Vector3D torusAxis;                           //!< AUTO: Vector containing rotation axis of torus; must be a unit vector.
    Real dynamicFriction;                         //!< AUTO: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, \refSection{sec:module:physics}
    Real frictionProportionalZone;                //!< AUTO: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), \refSection{sec:module:physics}
    Real contactStiffness;                        //!< AUTO: normal contact stiffness [SI:N/m] (units in case that \f$n_\mathrm{exp}=1\f$)
    Real contactDamping;                          //!< AUTO: linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
    Real contactStiffnessExponent;                //!< AUTO: exponent in normal contact model [SI:1]
    Real restitutionCoefficient;                  //!< AUTO: coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)
    Real minimumImpactVelocity;                   //!< AUTO: minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)
    Index impactModel;                            //!< AUTO:  number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectContactSphereTorusParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        nodeNumber = EXUstd::InvalidIndex;
        radiusSphere = 0.;
        torusMajorRadius = 0.;
        torusMinorRadius = 0.;
        torusAxis = Vector3D({0,0,0});
        dynamicFriction = 0.;
        frictionProportionalZone = 1e-3;
        contactStiffness = 0.;
        contactDamping = 0.;
        contactStiffnessExponent = 1.;
        restitutionCoefficient = 1.;
        minimumImpactVelocity = 0.;
        impactModel = 0;
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectContactSphereTorus
* @brief        A simple contact connector between a sphere (marker0) and a torus (marker1). The sphere is assumed to be placed inside of the torus (outer contact of sphere with torus currently not implemented!).
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

//! AUTO: CObjectContactSphereTorus
class CObjectContactSphereTorus: public CObjectConnector // AUTO: 
{
protected: // AUTO: 
    static constexpr Index nDataVariables = 4; //number of data variables for tangential and normal contact
    static constexpr Index dataIndexGap = 0; //!< index in data node representing gap
    static constexpr Index dataIndexVtangent = 1; //!< index in data node representing tangent velocity
    static constexpr Index dataIndexImpactVel = 2; //!< index in data node representing last impact velocity
    static constexpr Index dataIndexDeltaPlastic = 3; //!< index in data node representing plastic deformation, according to elasto-plastic adhesion model
    CObjectContactSphereTorusParameters parameters; //! AUTO: contains all parameters for CObjectContactSphereTorus

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectContactSphereTorusParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectContactSphereTorusParameters& GetParameters() const { return parameters; }

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

    //! AUTO:  number of nodes; needed for every object
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
    virtual Real PostNewtonStep(const MarkerDataStructure& markerDataCurrent, Index itemIndex, PostNewtonFlags::Type& flags, Real& recommendedStepSize) override;

    //! AUTO:  function called after discontinuous iterations have been completed for one step (e.g. to finalize history variables and set initial values for next step)
    virtual void PostDiscontinuousIterationStep() override;

    //! AUTO:  connector uses penalty formulation
    virtual bool IsPenaltyConnector() const override
    {
        return true;
    }

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs, const MarkerDataStructure& markerData, Index objectNumber) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);
    }

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Index itemIndex, Vector& value) const override;

    //! AUTO:  unique function to compute contact forces
    template<typename TReal> TReal ComputeContactForces(TReal gap, const SlimVectorBase<TReal, 3>& n0, TReal deltaVnormal, const SlimVectorBase<TReal, 3>& deltaVji, TReal dryFriction, bool frictionRegularizedRegion, SlimVectorBase<TReal, 3>& fVec, SlimVectorBase<TReal, 3>& fFriction, bool forceFrictionMode = true) const;

    //! AUTO:  main function to compute contact kinematics and forces
    void ComputeConnectorProperties(const MarkerDataStructure& markerData, Index itemIndex, const LinkedDataVector& data, Real& frictionCoeff, Real& gap, Vector3D& deltaP, Vector3D& deltaV, Vector3D& pCircle1, Vector3D& contactPoint, Vector3D& fVec, Vector3D& fFriction, Vector3D& n0, bool contactFromData = true) const;

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
            (Index64)OutputVariableType::Position +
            (Index64)OutputVariableType::Displacement +
            (Index64)OutputVariableType::DisplacementLocal +
            (Index64)OutputVariableType::Director1 +
            (Index64)OutputVariableType::Director2 +
            (Index64)OutputVariableType::Director3 +
            (Index64)OutputVariableType::Force +
            (Index64)OutputVariableType::Torque );
    }

};



#endif //#ifdef include once...
