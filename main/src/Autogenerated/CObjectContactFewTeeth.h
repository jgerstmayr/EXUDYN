/** ***********************************************************************************************
* @class        CObjectContactFewTeethParameters
* @brief        Parameter class for CObjectContactFewTeeth
*
* @author       GPT
* @date         2025-11-06
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
*                - email: johannes.gerstmayr@uibk.ac.at
*                - weblink: https://github.com/jgerstmayr/EXUDYN
*                
************************************************************************************************ */

#ifndef COBJECTCONTACTFEWTEETHPARAMETERS__H
#define COBJECTCONTACTFEWTEETHPARAMETERS__H

#include <ostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

//! Analytic few-teeth flank descriptor used to evaluate involute curves on-the-fly
struct FewTeethAnalyticProfile
{
    bool isInner = false;              //!< true -> inner ring flank, false -> outer gear flank
    bool replicateAllTeeth = false;    //!< true -> evaluate all teeth by looping over tooth index
    Index numberOfTeeth = 0;           //!< teeth count of associated gear
    Index singleToothIndex = 0;        //!< used when replicateAllTeeth=false
    Real module = 0.;                  //!< module (meters)
    Real alpha = 0.;                   //!< pressure angle (radians)
    Real haStar = 0.;                  //!< addendum coefficient
    Real cStar = 0.;                   //!< clearance coefficient
    Real addendumModification = 0.;    //!< x-coefficient
    Real deltaY = 0.;                  //!< delta_y shift (only relevant for inner gear)
    Real baseRotation = 0.;            //!< additional rotation applied before tooth pitch rotation
    Real flankMirror = 1.;             //!< +1 for right flank, -1 for left flank (mirror about y-axis)
    bool reverseParam = false;         //!< true -> parameter runs from tip to root (mirrors sample ordering)

    mutable bool derivedValid = false;
    mutable Real baseRadius = 0.;
    mutable Real baseCircleRadius = 0.;
    mutable Real radiusMin = 0.;
    mutable Real radiusMax = 0.;
    mutable Real radiusSpan = 0.;
    mutable Real phiPitch = 0.;
    mutable Real thetaBase = 0.;
    mutable Real rotationPerTooth = 0.;

    void InvalidateDerived() const { derivedValid = false; }

    //! recompute cached values when parameters changed
    void UpdateDerived() const
    {
        if (derivedValid) { return; }

        if (numberOfTeeth <= 0 || module <= 0.)
        {
            baseRadius = 0.;
            baseCircleRadius = 0.;
            radiusMin = 0.;
            radiusMax = 0.;
            radiusSpan = 0.;
            phiPitch = 0.;
            thetaBase = 0.;
            rotationPerTooth = 0.;
            derivedValid = true;
            return;
        }

        const Real r = 0.5 * module * (Real)numberOfTeeth;
        baseRadius = r;
        baseCircleRadius = r * cos(alpha);
        thetaBase = tan(alpha) - alpha;
        rotationPerTooth = (numberOfTeeth > 0) ? (2. * EXUstd::pi / (Real)numberOfTeeth) : 0.;

        if (isInner)
        {
            const Real ra = r - module * (haStar - addendumModification + deltaY);
            const Real rf = r + module * (haStar + cStar + addendumModification);
            radiusMin = std::max(ra, baseCircleRadius);
            radiusMax = rf;
            const Real s = EXUstd::pi * module / 2.0 - 2.0 * addendumModification * module * tan(alpha);
            phiPitch = (r != 0.) ? (s / (2.0 * r)) : 0.;
        }
        else
        {
            const Real rf = r - module * (haStar + cStar - addendumModification);
            const Real ra = r + module * (haStar + addendumModification);
            radiusMin = std::max(baseCircleRadius, rf);
            radiusMax = ra;
            const Real s = EXUstd::pi * module / 2.0 + 2.0 * addendumModification * module * tan(alpha);
            phiPitch = (r != 0.) ? (s / (2.0 * r)) : 0.;
        }

        radiusSpan = radiusMax - radiusMin;
        if (radiusSpan <= 0.) { radiusSpan = 1e-12; }
        derivedValid = true;
    }

    //! rotation for a specific tooth index (wraps automatically)
    Real RotationForTooth(Index toothIndex) const
    {
        UpdateDerived();
        if (numberOfTeeth <= 0) { return baseRotation; }

        Index idx = toothIndex;
        if (!replicateAllTeeth)
        {
            idx = singleToothIndex;
        }
        if (idx == EXUstd::InvalidIndex) { idx = 0; }
        if (numberOfTeeth > 0)
        {
            idx = ((idx % numberOfTeeth) + numberOfTeeth) % numberOfTeeth;
        }
        return baseRotation + rotationPerTooth * (Real)idx;
    }
};

//! Parameters for class CObjectContactFewTeeth
class CObjectContactFewTeethParameters
{
public:
    ArrayIndex markerNumbers;               //!< list of 2 markers; marker 0 -> outer gear body, marker 1 -> inner gear body
    Index nodeNumber;                       //!< node number of a NodeGenericData with data variables storing gap and active segment indices
    Real contactStiffness;                  //!< normal contact stiffness [SI:N/m]
    Real contactDamping;                    //!< linear normal contact damping [SI:N*s/m]
    Real contactReferenceDistance;          //!< reference distance at which gap is zero [SI:m]
    Real tipProbeLength;                    //!< tip-probe segment length used in fallback detection [SI:m]
    bool activeConnector;                   //!< flag to deactivate connector
    FewTeethAnalyticProfile outerAnalytic;  //!< analytic parameterization for outer gear flank
    FewTeethAnalyticProfile innerAnalytic;  //!< analytic parameterization for inner gear flank
    std::vector<Real> stiffnessOuter;       //!< outer gear tooth stiffness table [SI:N/m], uniformly distributed along flank parameter [0,1] (structure stiffness, without contact)
    std::vector<Real> stiffnessInner;       //!< inner gear tooth stiffness table [SI:N/m], uniformly distributed along flank parameter [0,1] (structure stiffness, without contact)
    Real faceWidth;                         //!< gear face width for contact stiffness calculation [SI:m]
    Real elasticModulus;                    //!< elastic modulus for contact stiffness calculation [SI:Pa]
    Real poissonRatio;                      //!< Poisson's ratio for contact stiffness calculation [-]
    Real frictionCoefficient;               //!< dry friction coefficient μ [-]
    Real frictionVelocityPenalty;           //!< velocity-dependent friction penalty [SI:N/(m/s)]
    Real frictionProportionalZone;          //!< regularization zone for friction [SI:m/s]
    Real frictionStiffness;                 //!< sticking friction stiffness for Bristle model [SI:N/m]; 0=disabled

    //! default constructor with parameter initialization
    CObjectContactFewTeethParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        nodeNumber = EXUstd::InvalidIndex;
        contactStiffness = 1e9;
        contactDamping = 1e3;
        contactReferenceDistance = 0.;
        tipProbeLength = 0.005;  // default 5mm
        activeConnector = true;
        faceWidth = 0.01;         // default 10mm
        elasticModulus = 2.1e11;  // default steel 210 GPa
        poissonRatio = 0.3;       // default steel
        frictionCoefficient = 0.;           // default: no friction
        frictionVelocityPenalty = 0.;       // default: no velocity penalty
        frictionProportionalZone = 0.001;   // default: 1mm/s regularization zone
        frictionStiffness = 0.;             // default: no Bristle friction
        // stiffnessOuter and stiffnessInner are empty by default
    }
};

/** ***********************************************************************************************
* @class        CObjectContactFewTeeth
 * @brief        Penalty-based planar contact using analytic gear flanks (few-teeth difference transmission).
*
************************************************************************************************ */

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class CObjectContactFewTeeth : public CObjectConnector
{
protected:
    CObjectContactFewTeethParameters parameters;

    mutable Real gapValue;
    mutable Real gapVelocity;
    mutable Real normalForce;
    mutable Vector3D contactPointOuter;
    mutable Vector3D contactPointInner;
    mutable Vector3D contactNormal;
    mutable Index lastOuterSegment;
    mutable Index lastInnerSegment;
    mutable Real lastOuterParameter;
    mutable Real lastInnerParameter;
    mutable Real lastRhoOuter;           //!< curvature radius of outer gear at contact point [SI:m]
    mutable Real lastRhoInner;           //!< curvature radius of inner gear at contact point [SI:m]
    mutable Real frictionForce;          //!< computed friction force magnitude [SI:N]
    mutable Vector3D tangentDirection;   //!< tangent direction at contact point
    mutable Real tangentialVelocity;     //!< relative tangential velocity [SI:m/s]

public:
    static constexpr Index nDataVariables = 9;
    static constexpr Index dataIndexGap = 0;
    static constexpr Index dataIndexGapVelocity = 1;
    static constexpr Index dataIndexContactForce = 2;
    static constexpr Index dataIndexOuterSegment = 3;
    static constexpr Index dataIndexInnerSegment = 4;
    static constexpr Index dataIndexOuterParam = 5;
    static constexpr Index dataIndexInnerParam = 6;
    static constexpr Index dataIndexStickPosX = 7; //!< Bristle model stick position X
    static constexpr Index dataIndexStickPosY = 8; //!< Bristle model stick position Y

    CObjectContactFewTeeth()
    {
        gapValue = 0.;
        gapVelocity = 0.;
        normalForce = 0.;
        contactPointOuter = Vector3D({0.,0.,0.});
        contactPointInner = Vector3D({0.,0.,0.});
        contactNormal = Vector3D({0.,0.,1.});
        lastOuterSegment = EXUstd::InvalidIndex;
        lastInnerSegment = EXUstd::InvalidIndex;
        lastOuterParameter = 0.;
        lastInnerParameter = 0.;
        lastRhoOuter = 0.;
        lastRhoInner = 0.;
        frictionForce = 0.;
        tangentDirection = Vector3D({1.,0.,0.});
        tangentialVelocity = 0.;
    }

    // access functions
    virtual CObjectContactFewTeethParameters& GetParameters() { return parameters; }
    virtual const CObjectContactFewTeethParameters& GetParameters() const { return parameters; }

    virtual const ArrayIndex& GetMarkerNumbers() const override { return parameters.markerNumbers; }
    virtual ArrayIndex& GetMarkerNumbers() override { return parameters.markerNumbers; }

    virtual Index RequestedNumberOfMarkers() const override { return 2; }

    virtual Index GetNodeNumber(Index localIndex) const override
    {
        CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;
    }

    virtual void SetNodeNumber(Index localIndex, Index nodeNumber) override
    {
        CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        parameters.nodeNumber = nodeNumber;
    }

    virtual Index GetNumberOfNodes() const override { return 1; }
    virtual Index GetDataVariablesSize() const override { return nDataVariables; }
    virtual bool HasDiscontinuousIteration() const override { return true; }
    virtual bool IsPenaltyConnector() const override { return true; }

    virtual void ComputeODE2LHS(Vector& ode2Lhs, const MarkerDataStructure& markerData, Index objectNumber) const override;

    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);
    }

    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Index itemIndex, Vector& value) const override;

    virtual Real PostNewtonStep(const MarkerDataStructure& markerDataCurrent, Index itemIndex, PostNewtonFlags::Type& flags, Real& recommendedStepSize) override;
    virtual void PostDiscontinuousIterationStep() override;

    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return (Marker::Type)((Index)Marker::Position + (Index)Marker::Orientation);
    }

    virtual CObjectType GetType() const override { return CObjectType::Connector; }
    virtual bool IsActive() const override { return parameters.activeConnector; }

    void ComputeContactKinematics(const MarkerDataStructure& markerData, Index itemIndex, LinkedDataVector& data, bool useDataStates) const;
    bool ComputeAnalyticContact(const MarkerDataStructure& markerData, LinkedDataVector& data, bool useDataStates) const;
};

#endif
