/** ***********************************************************************************************
* @class        CObjectContactCurveCirclesParameters
* @brief        Parameter class for CObjectContactCurveCircles
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2024-11-12  13:45:53 (last modified)
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
    PyMatrixContainer polynomialData;             //!< AUTO: matrix containing coefficients for polynomial enhancements of the linear segments; each row contains polynomial coefficients for the according segment; the polynomial coefficients may contain quadratic, cubic, etc. coefficients, while constant and linear coefficients are automatically selected such that the end points of the polynomial match the segment's endpoints; the local coordinate \f$x\f$ of the polynomial runs from 0 to 1 and positive values represent concave geometries (enlarging the curve). MatrixContainer has to be provided in dense mode!
    Matrix3D rotationMarker0;                     //!< AUTO: local rotation matrix for marker 0; used to rotate marker coordinates such that the curve lies in the \f$x-y\f$-plane
    Real dynamicFriction;                         //!< AUTO: dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, \refSection{sec:module:physics}
    Real frictionProportionalZone;                //!< AUTO: limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), \refSection{sec:module:physics}
    Real contactStiffness;                        //!< AUTO: normal contact stiffness [SI:N/m] (units in case that \f$n_\mathrm{exp}=1\f$)
    Real contactDamping;                          //!< AUTO: linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
    Index contactModel;                           //!< AUTO: number of contact model: 0) linear model for stiffness and damping, only proportional to penetration; 1) model taking contact geometry, in particular curvature of circle and curve into account, giving nonlinear normal force model
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
* @brief        [UNDER CONSTRUCTION] A contact model between a curve defined by piecewise segments and a set of circles. The 2D curve may corotate in 3D with the underlying marker and also defines the plane of action for the circles. Note that there is a limit of 100 circle markes above which computation becomes slower as it requires memory allocation.
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
    static constexpr Index nDataVariablesPerSegment = 3; //number of data variables per circle marker
    static constexpr Index dataIndexCircle = 0; //!< index in data node (per segment) representing circle number
    static constexpr Index dataIndexGap = 1; //!< index in data node (per segment) representing gap
    static constexpr Index dataIndexVtangent = 2; //!< index in data node (per segment) representing tangent velocity
    CObjectContactCurveCirclesParameters parameters; //! AUTO: contains all parameters for CObjectContactCurveCircles

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectContactCurveCirclesParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectContactCurveCirclesParameters& GetParameters() const { return parameters; }

    //! AUTO:  default function to return Marker numbers
    virtual const ArrayIndex& GetMarkerNumbers() const override
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
    void ComputeConnectorProperties(const MarkerDataStructure& markerData, Index itemIndex, LinkedDataVector& data, bool useDataStates, Vector2D& forceMarker0, Real& torqueMarker0, ResizableConstVectorBase<Real, CObjectContactCurveCirclesMaxConstSize>& gapPerSegment, ResizableConstVectorBase<Real, CObjectContactCurveCirclesMaxConstSize>& gapPerSegment_t, ResizableConstVectorBase<Real, CObjectContactCurveCirclesMaxConstSize>& segmentsForceLocalX, ResizableConstVectorBase<Real, CObjectContactCurveCirclesMaxConstSize>& segmentsForceLocalY) const;

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
            (Index)OutputVariableType::DisplacementLocal +
            (Index)OutputVariableType::VelocityLocal +
            (Index)OutputVariableType::ForceLocal );
    }

};



#endif //#ifdef include once...
