/** ***********************************************************************************************
* @class        CObjectContactCurveCirclesParameters
* @brief        Parameter class for CObjectContactCurveCircles
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-01-07  23:36:17 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTCONTACTCURVECIRCLESPARAMETERS__H
#define COBJECTCONTACTCURVECIRCLESPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include "Pymodules/PyMatrixContainer.h"//for data matrices
constexpr Index CObjectContactCurveCirclesMaxConstSize = 100; //maximum number of markers upon which arrays do not require memory allocation

//! AUTO: Parameters for class CObjectContactCurveCirclesParameters
class CObjectContactCurveCirclesParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of \f$n_c+1\f$ markers; marker \f$m0\f$ represents the marker carrying the curve; all other markers represent centers of \f$n_c\f$ circles, used in connector
    Index nodeNumber;                             //!< AUTO: node number of a NodeGenericData with nDataVariablesPerSegment dataCoordinates per segment, needed for discontinuous iteration; data variables contain values from last PostNewton iteration: data[0+3*i] is the circle number, data[1+3*i] is the gap, data[2+3*i] is the tangential velocity (and thus contains information if it is stick or slip)
    Vector circlesRadii;                          //!< AUTO: Vector containing radii of \f$n_c\f$ circles [SI:m]; number according to size of markerNumbers-1
    PyMatrixContainer segmentsData;               //!< AUTO: matrix containing a set of two planar point coordinates in each row, representing segments attached to marker \f$m0\f$ and undergoing contact with the circles; for segment \f$s0\f$ row 0 reads \f$[p_{0x,s0},\,p_{0y,s0},\,p_{1x,s0},\,p_{1y,s0}]\f$; note that the segments must be ordered such that going from \f$\pv_0\f$ to \f$\pv_1\f$, the exterior lies on the right (positive) side. MatrixContainer has to be provided in dense mode!
    PyMatrixContainer polynomialData;             //!< AUTO: matrix containing coefficients for special polynomial enhancements of the linear segments; each row contains coefficients for polynomials for the according segment, prescribing slopes at beginning and end of segment as well as curvature at beginning and end of segment; slopes and curvatures are defined in a local x/y coordinate system where x is the segment axis (start: x=0; x-axis points towards end point) and the segment normal is in y-direction; MatrixContainer has to be provided in dense mode!
    Matrix3D rotationMarker0;                     //!< AUTO: local rotation matrix for marker 0; used to rotate marker coordinates such that the curve lies in the \f$x-y\f$-plane
    Real dynamicFriction;                         //!< AUTO: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, \refSection{sec:module:physics}
    Real frictionProportionalZone;                //!< AUTO: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), \refSection{sec:module:physics}
    Real contactStiffness;                        //!< AUTO: normal contact stiffness [SI:N/(m*m)]
    Real contactDamping;                          //!< AUTO: linear normal contact damping [SI:N/(m s)]; this damping is a simplification of real contact dissipation and should be used with care.
    Index contactModel;                           //!< AUTO: number of contact model: 0) linear model for stiffness and damping, only proportional to penetration; contact force is computed from \f$l_\mathrm{seg}\left(p \cdot  \cdot k_c + \dot p \cdot d_c \right)\f$ as long as \f$p>0\f$; while this is numerically more stable, it gives jumps in forces when sliding over contact geometry 1) contact force proportional to integral over penetration area of circle with segments, giving a smoother contact force when sliding over geometry;
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectContactCurveCirclesParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        nodeNumber = EXUstd::InvalidIndex;
        circlesRadii = Vector();
        segmentsData = PyMatrixContainer();
        polynomialData = PyMatrixContainer();
        rotationMarker0 = EXUmath::unitMatrix3D;
        dynamicFriction = 0.;
        frictionProportionalZone = 1e-3;
        contactStiffness = 0.;
        contactDamping = 0.;
        contactModel = 0;
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectContactCurveCircles
* @brief        A contact model between a curve defined by piecewise segments and a set of circles. The 2D curve may corotate in 3D with the underlying marker and also defines the plane of action for the circles. [REQUIRES FURTHER TESTING; friction not yet available]
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

//! AUTO: CObjectContactCurveCircles
class CObjectContactCurveCircles: public CObjectConnector // AUTO: 
{
protected: // AUTO: 
    CObjectContactCurveCirclesParameters parameters; //! AUTO: contains all parameters for CObjectContactCurveCircles
    mutable Vector gapPerSegment;                 //!< AUTO: temporary vector for computed gap
    mutable Vector gapPerSegment_t;               //!< AUTO: temporary vector for computed gap velocity
    mutable Vector segmentsForceLocalX;           //!< AUTO: temporary vector for contact force per segment in local X-direction
    mutable Vector segmentsForceLocalY;           //!< AUTO: temporary vector for contact force per segment in local Y-direction

public: // AUTO: 
    static constexpr Index nDataVariablesPerSegment = 3; //number of data variables per circle marker
    static constexpr Index dataIndexCircle = 0; //!< index in data node (per segment) representing circle number
    static constexpr Index dataIndexGap = 1; //!< index in data node (per segment) representing gap
    static constexpr Index dataIndexVtangent = 2; //!< index in data node (per segment) representing tangent velocity
    //! AUTO: default constructor with parameter initialization
    CObjectContactCurveCircles()
    {
        gapPerSegment = Vector();
        gapPerSegment_t = Vector();
        segmentsForceLocalX = Vector();
        segmentsForceLocalY = Vector();
    };

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectContactCurveCirclesParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectContactCurveCirclesParameters& GetParameters() const { return parameters; }

    //! AUTO:  Write (Reference) access to:temporary vector for computed gap
    void SetGapPerSegment(const Vector& value) { gapPerSegment = value; }
    //! AUTO:  Read (Reference) access to:temporary vector for computed gap
    const Vector& GetGapPerSegment() const { return gapPerSegment; }
    //! AUTO:  Read (Reference) access to:temporary vector for computed gap
    Vector& GetGapPerSegment() { return gapPerSegment; }

    //! AUTO:  Write (Reference) access to:temporary vector for computed gap velocity
    void SetGapPerSegment_t(const Vector& value) { gapPerSegment_t = value; }
    //! AUTO:  Read (Reference) access to:temporary vector for computed gap velocity
    const Vector& GetGapPerSegment_t() const { return gapPerSegment_t; }
    //! AUTO:  Read (Reference) access to:temporary vector for computed gap velocity
    Vector& GetGapPerSegment_t() { return gapPerSegment_t; }

    //! AUTO:  Write (Reference) access to:temporary vector for contact force per segment in local X-direction
    void SetSegmentsForceLocalX(const Vector& value) { segmentsForceLocalX = value; }
    //! AUTO:  Read (Reference) access to:temporary vector for contact force per segment in local X-direction
    const Vector& GetSegmentsForceLocalX() const { return segmentsForceLocalX; }
    //! AUTO:  Read (Reference) access to:temporary vector for contact force per segment in local X-direction
    Vector& GetSegmentsForceLocalX() { return segmentsForceLocalX; }

    //! AUTO:  Write (Reference) access to:temporary vector for contact force per segment in local Y-direction
    void SetSegmentsForceLocalY(const Vector& value) { segmentsForceLocalY = value; }
    //! AUTO:  Read (Reference) access to:temporary vector for contact force per segment in local Y-direction
    const Vector& GetSegmentsForceLocalY() const { return segmentsForceLocalY; }
    //! AUTO:  Read (Reference) access to:temporary vector for contact force per segment in local Y-direction
    Vector& GetSegmentsForceLocalY() { return segmentsForceLocalY; }

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

    //! AUTO:  number of markers; 0 means no requirements (standard would be 2; would fail in pre-checks!)
    virtual Index RequestedNumberOfMarkers() const override
    {
        return 0;
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

    //! AUTO:  data variables in total
    virtual Index GetDataVariablesSize() const override
    {
        return nDataVariablesPerSegment*GetNumberOfSegments();
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

    //! AUTO:  returns number of circles, needed by other functions
    Index GetNumberOfCircles() const
    {
        return parameters.markerNumbers.NumberOfItems()-1;
    }

    //! AUTO:  number of segments determined by segmentsData
    Index GetNumberOfSegments() const
    {
        return parameters.segmentsData.NumberOfRows();
    }

    //! AUTO:  main function to compute contact kinematics and forces
    void ComputeConnectorProperties(const MarkerDataStructure& markerData, Index itemIndex, LinkedDataVector& data, bool useDataStates, Vector2D& forceMarker0, Real& torqueMarker0, Vector& gapPerSegment, Vector& gapPerSegment_t, Vector& segmentsForceLocalX, Vector& segmentsForceLocalY) const;

    //! AUTO:  compute sum of polynomials at x, with segment length c, segment number and polyCoeffs; implemented for 0, 2 or 4 coefficients
    Real ComputePolynomials(Real x, Real c, Index segNum, const ResizableMatrix& polyCoeffs) const;

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
            (Index64)OutputVariableType::DisplacementLocal +
            (Index64)OutputVariableType::VelocityLocal +
            (Index64)OutputVariableType::ForceLocal );
    }

};



#endif //#ifdef include once...
