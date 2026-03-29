/** ***********************************************************************************************
* @class        CObjectContactExternalInvoluteParameters
* @brief        Parameter class for CObjectContactExternalInvolute
*
* @author       GPT
* @date         2025-12-17
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
*                - email: johannes.gerstmayr@uibk.ac.at
*                - weblink: https://github.com/jgerstmayr/EXUDYN
*                
************************************************************************************************ */

#ifndef COBJECTCONTACTEXTERNALINVOLUTE__H
#define COBJECTCONTACTEXTERNALINVOLUTE__H

#include <ostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

//! Analytic involute gear flank descriptor for external gears
struct ExternalInvoluteProfile
{
    Index numberOfTeeth = 0;           //!< teeth count of gear
    Real module = 0.;                  //!< module (meters)
    Real alpha = 0.;                   //!< pressure angle (radians), typically 20°
    Real haStar = 1.0;                 //!< addendum coefficient (standard = 1.0)
    Real cStar = 0.25;                 //!< clearance coefficient (standard = 0.25)
    Real addendumModification = 0.;    //!< profile shift coefficient (x)
    Real baseRotation = 0.;            //!< additional rotation applied before tooth pitch rotation
    Real flankMirror = 1.;             //!< +1 for right flank, -1 for left flank (mirror about y-axis)
    bool reverseParam = false;         //!< true -> parameter runs from tip to root
    bool replicateAllTeeth = true;     //!< if true, iterate all teeth; if false, only use singleToothIndex
    Index singleToothIndex = 0;        //!< tooth index to use when replicateAllTeeth=false

    mutable bool derivedValid = false;
    mutable Real pitchRadius = 0.;     //!< pitch circle radius r = m*z/2
    mutable Real baseCircleRadius = 0.;//!< base circle radius rb = r*cos(alpha)
    mutable Real rootRadius = 0.;      //!< root circle radius rf
    mutable Real tipRadius = 0.;       //!< tip circle radius ra
    mutable Real radiusMin = 0.;       //!< minimum evaluation radius (max of base circle and root circle)
    mutable Real radiusMax = 0.;       //!< maximum evaluation radius (tip circle)
    mutable Real radiusSpan = 0.;      //!< radiusMax - radiusMin
    mutable Real toothThickAngle = 0.; //!< half tooth thickness angle at pitch circle
    mutable Real thetaBase = 0.;       //!< involute angle at base circle: tan(alpha) - alpha
    mutable Real rotationPerTooth = 0.;//!< angular pitch: 2*pi/z

    void InvalidateDerived() const { derivedValid = false; }

    //! recompute cached values when parameters changed
    void UpdateDerived() const
    {
        if (derivedValid) { return; }

        if (numberOfTeeth <= 0 || module <= 0.)
        {
            pitchRadius = 0.;
            baseCircleRadius = 0.;
            rootRadius = 0.;
            tipRadius = 0.;
            radiusMin = 0.;
            radiusMax = 0.;
            radiusSpan = 0.;
            toothThickAngle = 0.;
            thetaBase = 0.;
            rotationPerTooth = 0.;
            derivedValid = true;
            return;
        }

        // Standard external gear geometry
        pitchRadius = 0.5 * module * (Real)numberOfTeeth;
        baseCircleRadius = pitchRadius * cos(alpha);
        thetaBase = tan(alpha) - alpha;
        rotationPerTooth = (numberOfTeeth > 0) ? (2. * EXUstd::pi / (Real)numberOfTeeth) : 0.;

        // External gear: root below pitch, tip above pitch
        rootRadius = pitchRadius - module * (haStar + cStar - addendumModification);
        tipRadius = pitchRadius + module * (haStar + addendumModification);

        // Involute only valid above base circle
        radiusMin = std::max(baseCircleRadius, rootRadius);
        radiusMax = tipRadius;
        radiusSpan = radiusMax - radiusMin;
        if (radiusSpan <= 0.) { radiusSpan = 1e-12; }

        // Tooth thickness angle at pitch circle
        const Real s = EXUstd::pi * module / 2.0 + 2.0 * addendumModification * module * tan(alpha);
        toothThickAngle = (pitchRadius > 0.) ? (s / (2.0 * pitchRadius)) : 0.;

        derivedValid = true;
    }

    //! rotation for a specific tooth index (wraps automatically)
    Real RotationForTooth(Index toothIndex) const
    {
        UpdateDerived();
        if (numberOfTeeth <= 0) { return baseRotation; }

        Index idx = toothIndex;
        if (idx == EXUstd::InvalidIndex) { idx = 0; }
        if (numberOfTeeth > 0)
        {
            idx = ((idx % numberOfTeeth) + numberOfTeeth) % numberOfTeeth;
        }
        return baseRotation + rotationPerTooth * (Real)idx;
    }
};

//! Parameters for class CObjectContactExternalInvolute
class CObjectContactExternalInvoluteParameters
{
public:
    ArrayIndex markerNumbers;               //!< list of 2 markers; marker 0 -> gear 1 body, marker 1 -> gear 2 body
    Index nodeNumber;                       //!< node number of a NodeGenericData with data variables storing gap and active segment indices
    Real contactDamping;                    //!< linear normal contact damping [SI:N*s/m]
    Real tipProbeLength;                    //!< tip-probe segment length used in contact detection [SI:m]
    bool activeConnector;                   //!< flag to deactivate connector
    ExternalInvoluteProfile gear1Analytic;  //!< analytic parameterization for gear 1 flank
    ExternalInvoluteProfile gear2Analytic;  //!< analytic parameterization for gear 2 flank
    std::vector<Real> stiffnessGear1;       //!< gear 1 tooth stiffness table [SI:N/m], uniformly distributed along flank parameter [0,1]
    std::vector<Real> stiffnessGear2;       //!< gear 2 tooth stiffness table [SI:N/m], uniformly distributed along flank parameter [0,1]
    Real faceWidth;                         //!< gear face width for contact stiffness calculation [SI:m]
    Real elasticModulus;                    //!< elastic modulus for contact stiffness calculation [SI:Pa]
    Real poissonRatio;                      //!< Poisson's ratio for contact stiffness calculation [-]
    Real frictionCoefficient;               //!< dry friction coefficient μ [-]
    Real frictionVelocityPenalty;           //!< velocity-dependent friction penalty [SI:N/(m/s)]
    Real frictionProportionalZone;          //!< regularization zone for friction [SI:m/s]
    Real frictionStiffness;                 //!< sticking friction stiffness for Bristle model [SI:N/m]; 0=disabled

    //! default constructor with parameter initialization
    CObjectContactExternalInvoluteParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        nodeNumber = EXUstd::InvalidIndex;
        contactDamping = 1e3;
        tipProbeLength = 0.005;  // default 5mm
        activeConnector = true;
        faceWidth = 0.01;         // default 10mm
        elasticModulus = 2.1e11;  // default steel 210 GPa
        poissonRatio = 0.3;       // default steel
        frictionCoefficient = 0.;           // default: no friction
        frictionVelocityPenalty = 0.;       // default: no velocity penalty
        frictionProportionalZone = 0.001;   // default: 1mm/s regularization zone
        frictionStiffness = 0.;             // default: no Bristle friction
    }
};

/** ***********************************************************************************************
* @class        CObjectContactExternalInvolute
 * @brief        Penalty-based planar contact for external involute gear pairs.
*
************************************************************************************************ */

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class CObjectContactExternalInvolute : public CObjectConnector
{
protected:
    CObjectContactExternalInvoluteParameters parameters;

    mutable Real gapValue;
    mutable Real gapVelocity;
    mutable Real normalForce;
    mutable Vector3D contactPointGear1;
    mutable Vector3D contactPointGear2;
    mutable Vector3D contactNormal;
    mutable Index lastGear1Tooth;
    mutable Index lastGear2Tooth;
    mutable Real lastGear1Param;
    mutable Real lastGear2Param;
    mutable Real lastRhoGear1;           //!< curvature radius of gear 1 at contact point [SI:m]
    mutable Real lastRhoGear2;           //!< curvature radius of gear 2 at contact point [SI:m]
    mutable Real frictionForce;          //!< computed friction force magnitude [SI:N]
    mutable Vector3D tangentDirection;   //!< tangent direction at contact point
    mutable Real tangentialVelocity;     //!< relative tangential velocity [SI:m/s]
    mutable bool lastFlankIsLeft1;       //!< true if gear1 contact was on left flank
    mutable bool lastFlankIsLeft2;       //!< true if gear2 contact was on left flank

public:
    static constexpr Index nDataVariables = 11;
    static constexpr Index dataIndexGap = 0;
    static constexpr Index dataIndexGapVelocity = 1;
    static constexpr Index dataIndexContactForce = 2;
    static constexpr Index dataIndexGear1Tooth = 3;
    static constexpr Index dataIndexGear2Tooth = 4;
    static constexpr Index dataIndexGear1Param = 5;
    static constexpr Index dataIndexGear2Param = 6;
    static constexpr Index dataIndexStickPosX = 7;  //!< Bristle model stick position X
    static constexpr Index dataIndexStickPosY = 8;  //!< Bristle model stick position Y
    static constexpr Index dataIndexFlankLeft1 = 9; //!< gear1 flank side (0=right, 1=left)
    static constexpr Index dataIndexFlankLeft2 = 10;//!< gear2 flank side (0=right, 1=left)

    CObjectContactExternalInvolute()
    {
        gapValue = 0.;
        gapVelocity = 0.;
        normalForce = 0.;
        contactPointGear1 = Vector3D({0.,0.,0.});
        contactPointGear2 = Vector3D({0.,0.,0.});
        contactNormal = Vector3D({0.,0.,1.});
        lastGear1Tooth = EXUstd::InvalidIndex;
        lastGear2Tooth = EXUstd::InvalidIndex;
        lastGear1Param = 0.;
        lastGear2Param = 0.;
        lastRhoGear1 = 0.;
        lastRhoGear2 = 0.;
        frictionForce = 0.;
        tangentDirection = Vector3D({1.,0.,0.});
        tangentialVelocity = 0.;
        lastFlankIsLeft1 = false;
        lastFlankIsLeft2 = false;
    }

    // access functions
    virtual CObjectContactExternalInvoluteParameters& GetParameters() { return parameters; }
    virtual const CObjectContactExternalInvoluteParameters& GetParameters() const { return parameters; }

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
    bool ComputeExternalGearContact(const MarkerDataStructure& markerData, LinkedDataVector& data, bool useDataStates) const;
};

#endif
