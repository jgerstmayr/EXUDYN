/** ***********************************************************************************************
* @class        CObjectContactConvexRollParameters
* @brief        Parameter class for CObjectContactConvexRoll
*
* @author       Manzl Peter
* @date         2019-07-01 (generated)
* @date         2021-12-22  08:13:07 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTCONTACTCONVEXROLLPARAMETERS__H
#define COBJECTCONTACTCONVEXROLLPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

constexpr Index CObjectContactConvexRollMaxPolynomialCoefficients = 20; //maximum number of polynomial coefficients, polynomial order needs to be n-1
constexpr Index CObjectContactConvexRollMaxIterationsContact = 20; // maximum number of iterations t find roots of polynomial for contact
constexpr Index CObjectContactConvexRollNEvalConvexityCheck = 1000; // number of equidistant sample points to check convexity of given polynomial at assembly time.

//! AUTO: Parameters for class CObjectContactConvexRollParameters
class CObjectContactConvexRollParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector; \f$m0\f$ represents the ground, which can undergo translations but not rotations, and \f$m1\f$ represents the rolling body, which has its reference point (=local position [0,0,0]) at the roll's center point
    Index nodeNumber;                             //!< AUTO: node number of a NodeGenericData (size=3) for 3 dataCoordinates, needed for discontinuous iteration (friction and contact)
    Real contactStiffness;                        //!< AUTO: normal contact stiffness [SI:N/m]
    Real contactDamping;                          //!< AUTO: normal contact damping [SI:N/(m s)]
    Real dynamicFriction;                         //!< AUTO: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, \refSection{sec:module:physics}
    Real staticFrictionOffset;                    //!< AUTO: static friction offset for friction model (static friction = dynamic friction + static offset), see StribeckFunction in exudyn.physics, \refSection{sec:module:physics}
    Real viscousFriction;                         //!< AUTO: viscous friction coefficient (velocity dependent part) for friction model, see StribeckFunction in exudyn.physics, \refSection{sec:module:physics}
    Real exponentialDecayStatic;                  //!< AUTO: exponential decay of static friction offset (must not be zero!), see StribeckFunction in exudyn.physics (named expVel there!), \refSection{sec:module:physics}
    Real frictionProportionalZone;                //!< AUTO: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), \refSection{sec:module:physics}
    Real rollLength;                              //!< AUTO: roll length [m], symmetric w.r.t.\ centerpoint
    Vector coefficientsHull;                      //!< AUTO: a vector of polynomial coefficients, which provides the polynomial of the CONVEX hull of the roll; \f$\mathrm{hull}(x) = k_0 x^{n_p-1} + k x^{n_p-2} + \ldots + k_{n_p-2} x  + k_{n_p-1}\f$
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temorarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectContactConvexRollParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        nodeNumber = EXUstd::InvalidIndex;
        contactStiffness = 0.;
        contactDamping = 0.;
        dynamicFriction = 0.;
        staticFrictionOffset = 0.;
        viscousFriction = 0.;
        exponentialDecayStatic = 1e-3;
        frictionProportionalZone = 1e-3;
        rollLength = 0.;
        coefficientsHull =  Vector();
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectContactConvexRoll
* @brief        A contact connector representing a convex roll (marker 1) on a flat surface (marker 0, ground body, not moving) in global \f$x\f$-\f$y\f$ plane. The connector is similar to ObjectConnectorRollingDiscPenalty, but includes a (strictly) convex shape of the roll defined by a polynomial. It is based on a penalty formulation and adds friction and slipping. The formulation is still under development and needs further testing. Note that the rolling body must have the reference point at the center of the disc.
*
* @author       Manzl Peter
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

//! AUTO: CObjectContactConvexRoll
class CObjectContactConvexRoll: public CObjectConnector // AUTO: 
{
protected: // AUTO: 
    static constexpr Index nDataVariables = 3; //number of data variables for tangential and normal contact
    mutable bool objectIsInitialized; //!< flag which shows that polynomials have not been computed
    CObjectContactConvexRollParameters parameters; //! AUTO: contains all parameters for CObjectContactConvexRoll
    mutable Vector coefficientsHullDerivative;    //!< AUTO: polynomial coefficients of the polynomial \f$\mathrm{hull}^\prime(x)\f$
    mutable Vector coefficientsHullDDerivative;   //!< AUTO: second derivative of the hull polynomial.
    Real rBoundingSphere;                         //!< AUTO: The  radius of the bounding sphere for the contact pre-check, calculated from the polynomial coefficients of the hull
    Vector3D pContact;                            //!< AUTO: The  current potential contact point. Contact occures if pContact[2] < 0. 

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    CObjectContactConvexRoll()
    {
        coefficientsHullDerivative = Vector();
        coefficientsHullDDerivative = Vector();
        rBoundingSphere = 0;
        pContact = Vector3D({0,0,0});
    };

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectContactConvexRollParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectContactConvexRollParameters& GetParameters() const { return parameters; }

    //! AUTO:  Write (Reference) access to:\f$\kv^\prime \in \Rcal^{n_p}\f$polynomial coefficients of the polynomial \f$\mathrm{hull}^\prime(x)\f$
    void SetCoefficientsHullDerivative(const Vector& value) { coefficientsHullDerivative = value; }
    //! AUTO:  Read (Reference) access to:\f$\kv^\prime \in \Rcal^{n_p}\f$polynomial coefficients of the polynomial \f$\mathrm{hull}^\prime(x)\f$
    const Vector& GetCoefficientsHullDerivative() const { return coefficientsHullDerivative; }
    //! AUTO:  Read (Reference) access to:\f$\kv^\prime \in \Rcal^{n_p}\f$polynomial coefficients of the polynomial \f$\mathrm{hull}^\prime(x)\f$
    Vector& GetCoefficientsHullDerivative() { return coefficientsHullDerivative; }

    //! AUTO:  Write (Reference) access to:second derivative of the hull polynomial.
    void SetCoefficientsHullDDerivative(const Vector& value) { coefficientsHullDDerivative = value; }
    //! AUTO:  Read (Reference) access to:second derivative of the hull polynomial.
    const Vector& GetCoefficientsHullDDerivative() const { return coefficientsHullDDerivative; }
    //! AUTO:  Read (Reference) access to:second derivative of the hull polynomial.
    Vector& GetCoefficientsHullDDerivative() { return coefficientsHullDDerivative; }

    //! AUTO:  Write (Reference) access to:The  radius of the bounding sphere for the contact pre-check, calculated from the polynomial coefficients of the hull
    void SetRBoundingSphere(const Real& value) { rBoundingSphere = value; }
    //! AUTO:  Read (Reference) access to:The  radius of the bounding sphere for the contact pre-check, calculated from the polynomial coefficients of the hull
    const Real& GetRBoundingSphere() const { return rBoundingSphere; }
    //! AUTO:  Read (Reference) access to:The  radius of the bounding sphere for the contact pre-check, calculated from the polynomial coefficients of the hull
    Real& GetRBoundingSphere() { return rBoundingSphere; }

    //! AUTO:  Write (Reference) access to:The  current potential contact point. Contact occures if pContact[2] < 0. 
    void SetPContact(const Vector3D& value) { pContact = value; }
    //! AUTO:  Read (Reference) access to:The  current potential contact point. Contact occures if pContact[2] < 0. 
    const Vector3D& GetPContact() const { return pContact; }
    //! AUTO:  Read (Reference) access to:The  current potential contact point. Contact occures if pContact[2] < 0. 
    Vector3D& GetPContact() { return pContact; }

    //! AUTO:  default function to return Marker numbers
    virtual const ArrayIndex& GetMarkerNumbers() const override
    {
        return parameters.markerNumbers;
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        release_assert(localIndex == 0);
        return parameters.nodeNumber;
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

    //! AUTO:  compute contact kinematics and contact forces; allowSwitching set false for Newton
    void ComputeContactForces(const MarkerDataStructure& markerData, const CObjectContactConvexRollParameters& parameters, Vector3D& pC, Vector3D& vC, Vector3D& fContact, Vector3D& mContact, bool allowSwitching) const;

    //! AUTO:  initialize parameters for contact check
    void InitializeObject(const CObjectContactConvexRollParameters& parameters);

    //! AUTO:  Check Convexity of the given polynomial before execution
    bool CheckConvexityOfPolynomial(const CObjectContactConvexRollParameters& parameters);

    //! AUTO:  Check if one of the bounding spheres at the end of the roller is in contact with the ground
    bool PreContactCheckRoller(const Matrix3D& Rotm, const Vector3D& displacement, Real lRoller, Real R, Vector3D& pC) const;

    //! AUTO:  Find the point of the roller closest the ground, contact occures when return[2] < 0  
    Vector3D FindContactPoint(const Matrix3D& Rotm, const Vector& poly, Real lRoller) const;

    //! AUTO:  PolynomialRollXOFAngle: calculate the x-Value of the polynomial matching the slope of the contact
    Real PolynomialRollXOfAngle(const Vector& poly, const Vector& dpoly, Real lRoller, Real angy) const;

    //! AUTO:  This flag is reset upon change of parameters; says that the vector of coordinate indices has changed
    virtual void ParametersHaveChanged() override
    {
        objectIsInitialized = false;
    }

    //! AUTO:  operations done after Assemble()
    virtual void PostAssemble() override
    {
        InitializeObject(this->parameters);
    }

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
            (Index)OutputVariableType::Position +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::Force +
            (Index)OutputVariableType::Torque );
    }

};



#endif //#ifdef include once...
