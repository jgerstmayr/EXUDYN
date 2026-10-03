/** ***********************************************************************************************
* @class        CObjectJointGenericParameters
* @brief        Parameter class for CObjectJointGeneric
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-10-03  11:06:22 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef COBJECTJOINTGENERICPARAMETERS__H
#define COBJECTJOINTGENERICPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
namespace py = pybind11;            //! AUTO: "py" used throughout in code
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectJointGenericParameters
class CObjectJointGenericParameters // AUTO:
{
public: // AUTO:
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    ArrayIndex constrainedAxes;                   //!< AUTO: flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; for \f$j_i\f$, two values are possible: 0=free axis, 1=constrained axis
    Matrix3D rotationMarker0;                     //!< AUTO: local rotation matrix for marker \f$m0\f$; translation and rotation axes for marker \f$m0\f$ are defined in the local body coordinate system and additionally transformed by rotationMarker0; **deprecated** (removed in 2031): give the rotation to marker 0 as its localHT
    Matrix3D rotationMarker1;                     //!< AUTO: local rotation matrix for marker \f$m1\f$; translation and rotation axes for marker \f$m1\f$ are defined in the local body coordinate system and additionally transformed by rotationMarker1; **deprecated** (removed in 2031): give the rotation to marker 1 as its localHT
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
    Vector6D offsetUserFunctionParameters;        //!< AUTO: vector of 6 parameters for joint's offsetUserFunction
    PythonUserFunctionBase< std::function<StdVector6D(const MainSystem&,Real,Index,StdVector6D)> > offsetUserFunction;//!< AUTO: A Python function which defines the time-dependent (fixed) offset of translation (indices 0,1,2) and rotation (indices 3,4,5) joint coordinates with parameters (mbs, t, offsetUserFunctionParameters)
    PythonUserFunctionBase< std::function<StdVector6D(const MainSystem&,Real,Index,StdVector6D)> > offsetUserFunction_t;//!< AUTO: (NOT IMPLEMENTED YET)time derivative of offsetUserFunction using the same parameters
    bool alternativeConstraints;                  //!< AUTO: this is an experimental flag, may change in future: if uses alternative contraint equations for rotations, currently in case of 3 locked rotations: \f$\LU{0}{\tv}_{x0}\tp (\LU{0}{\tv}_{y1} \times \LU{0}{\tv}_{z0})\f$, \f$\LU{0}{\tv}_{y0}\tp (\LU{0}{\tv}_{z1} \times \LU{0}{\tv}_{x0})\f$, \f$\LU{0}{\tv}_{z0}\tp (\LU{0}{\tv}_{x1} \times \LU{0}{\tv}_{y0})\f$; this avoids 180° flips of the standard configuration in static computations, but leads to different values in Lagrange multipliers
    //! AUTO: default constructor with parameter initialization
    CObjectJointGenericParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        constrainedAxes = ArrayIndex({1,1,1,1,1,1});
        rotationMarker0 = EXUmath::unitMatrix3D;
        rotationMarker1 = EXUmath::unitMatrix3D;
        activeConnector = true;
        offsetUserFunctionParameters = Vector6D({0.,0.,0.,0.,0.,0.});
        offsetUserFunction = 0;
        offsetUserFunction_t = 0;
        alternativeConstraints = false;
    };
};


/** ***********************************************************************************************
* @class        CObjectJointGeneric
* @brief        A generic joint in 3D; constrains components of the absolute position and rotations of two points given by PointMarkers or RigidMarkers. The three rotation axes and sliding axes are those of the markers' frames; a rotation of these frames is given to the markers as their localHT.

```{image} /docs/figures/UniversalJoint.png
:width: 400
```

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

//! AUTO: CObjectJointGeneric
class CObjectJointGeneric: public CObjectConstraint // AUTO:
{
protected: // AUTO:
    static constexpr Index nConstraints = 6;
    CObjectJointGenericParameters parameters; //! AUTO: contains all parameters for CObjectJointGeneric

public: // AUTO:

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectJointGenericParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectJointGenericParameters& GetParameters() const { return parameters; }

    //! AUTO:  return true, if object has a computation user function
    virtual bool HasUserFunction() const override
    {
        return (parameters.offsetUserFunction!=0) || (parameters.offsetUserFunction_t!=0);
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

    //! AUTO:  true if the connector uses a penalty formulation; false if the constraint uses Lagrange multipliers
    virtual bool IsPenaltyConnector() const override
    {
        return false;
    }

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override;

    //! AUTO:  the equations on the connector interface of rigid markers (#2745)
    virtual bool ComputeAlgebraicEquationsConnector(const CSystemData& systemData, TemporaryComputationData& temp, Index objectNumber, bool velocityLevel, Vector& localAE) const override
    {
        if (!OnConnectorInterface()) { return false; } ConstraintEquationsRigidMarkers(systemData, temp, *this, objectNumber, velocityLevel, localAE); return true;
    }

    //! AUTO:  C_q by automatic differentiation of the equations (#2745)
    virtual bool ComputeJacobianAEConnector(const CSystemData& systemData, TemporaryComputationData& temp, Index objectNumber, ResizableMatrix& jacobianAE_ODE2) const override
    {
        if (!OnConnectorInterface()) { return false; } ConstraintJacobianRigidMarkers(systemData, temp, *this, objectNumber, jacobianAE_ODE2); return true;
    }

    //! AUTO:  C_q^T lambda per marker, without forming C_q (#2745)
    virtual bool ComputeReactionForcesConnector(const CSystemData& systemData, TemporaryComputationData& temp, Index objectNumber, const Vector& reactionForces, Vector& localODE2) const override
    {
        if (!OnConnectorInterface()) { return false; } ConstraintReactionForcesRigidMarkers(systemData, temp, *this, objectNumber, reactionForces, localODE2); return true;
    }

    //! AUTO:  the equations of the inactive constraint, lambda = 0; the active one computes on the connector interface (#2745)
    virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false) const override
    {
        CHECKandTHROW(!IsActive(), "ComputeAlgebraicEquations: an active constraint computes on the connector interface"); algebraicEquations.CopyFrom(markerData.GetLagrangeMultipliers());
    }

    //! AUTO:  the Jacobian of the inactive constraint, d(lambda)/d(lambda); the active one computes on the connector interface (#2745)
    virtual void ComputeJacobianAE(ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE, const MarkerDataStructure& markerData, Real t, Index itemIndex) const override
    {
        CHECKandTHROW(!IsActive(), "ComputeJacobianAE: an active constraint computes on the connector interface"); jacobian_AE.SetScalarMatrix(GetAlgebraicEquationsSize(), 1.);
    }

    //! AUTO:  the algebraic equations from the kinematics of the two markers (#2745)
    virtual void ComputeConstraintEquationsRigid(const MarkerRigid<Real>* markers, const LinkedDataVector& lambda, Real t, Index itemIndex, bool velocityLevel, ConstSizeVector<maxConstraintEquations>& equations) const override;

    //! AUTO:  the same equations with automatic differentiation, for C_q (#2745)
    virtual void ComputeConstraintEquationsRigidDiff(const MarkerRigid<DRealRigidMarkers>* markers, const LinkedDataVector& lambda, Real t, Index itemIndex, ConstSizeVectorBase<DRealRigidMarkers, maxConstraintEquations>& equations) const override;

    //! AUTO:  the equations of the joint, for Real and AutoDiff (#2745)
    template<class TReal> void ComputeConstraintEquationsTemplate(const MarkerRigid<TReal>* markers, const LinkedDataVector& lambda, Real t, Index itemIndex, bool velocityLevel, ConstSizeVectorBase<TReal, maxConstraintEquations>& equations) const;

    //! AUTO:  the derivative of the equations of the free axes by the Lagrange multipliers (#2745)
    virtual void ComputeJacobianAE_AE(ResizableMatrix& jacobian_AE) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Index itemIndex, Vector& value) const override;

    //! AUTO:  provide requested markerType for connector
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return (Marker::Type)((Index)Marker::Position + (Index)Marker::Orientation);
    }

    //! AUTO:  return object type (for node treatment in computation)
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);
    }

    //! AUTO:  number of algebraic equations; independent of node/body coordinates
    virtual Index GetAlgebraicEquationsSize() const override
    {
        return 6;
    }

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionOffset(Vector6D& offset, const MainSystemBase& mainSystem, Real t, Index itemIndex) const;

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionOffset_t(Vector6D& offset, const MainSystemBase& mainSystem, Real t, Index itemIndex) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index64)OutputVariableType::Position +
            (Index64)OutputVariableType::Velocity +
            (Index64)OutputVariableType::DisplacementLocal +
            (Index64)OutputVariableType::VelocityLocal +
            (Index64)OutputVariableType::Rotation +
            (Index64)OutputVariableType::AngularVelocityLocal +
            (Index64)OutputVariableType::ForceLocal +
            (Index64)OutputVariableType::TorqueLocal );
    }

};



#endif //#ifdef include once...
