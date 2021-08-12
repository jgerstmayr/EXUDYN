/** ***********************************************************************************************
* @class        CObjectConnectorCoordinateVectorParameters
* @brief        Parameter class for CObjectConnectorCoordinateVector
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-08-11  16:20:59 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTCONNECTORCOORDINATEVECTORPARAMETERS__H
#define COBJECTCONNECTORCOORDINATEVECTORPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CObjectConnectorCoordinateVectorParameters
class CObjectConnectorCoordinateVectorParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex markerNumbers;                     //!< AUTO: list of markers used in connector
    Matrix scalingMarker0;                        //!< AUTO: linear scaling matrix for coordinate vector of marker 0; matrix provided in python numpy format
    Matrix scalingMarker1;                        //!< AUTO: linear scaling matrix for coordinate vector of marker 1; matrix provided in python numpy format
    Matrix quadraticTermMarker0;                  //!< AUTO: quadratic scaling matrix for coordinate vector of marker 0; matrix provided in python numpy format
    Matrix quadraticTermMarker1;                  //!< AUTO: quadratic scaling matrix for coordinate vector of marker 1; matrix provided in python numpy format
    Vector offset;                                //!< AUTO: offset added to constraint equation; only active, if no userFunction is defined
    bool velocityLevel;                           //!< AUTO: If true: connector constrains velocities (only works for \hac{ODE2} coordinates!); offset is used between velocities; in this case, the offsetUserFunction\_t is considered and offsetUserFunction is ignored
    bool activeConnector;                         //!< AUTO: flag, which determines, if the connector is active; used to deactivate (temorarily) a connector or constraint
    //! AUTO: default constructor with parameter initialization
    CObjectConnectorCoordinateVectorParameters()
    {
        markerNumbers = ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex });
        scalingMarker0 = Matrix();
        scalingMarker1 = Matrix();
        quadraticTermMarker0 = Matrix();
        quadraticTermMarker1 = Matrix();
        offset = Vector();
        velocityLevel = false;
        activeConnector = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectConnectorCoordinateVector
* @brief        A constraint which constrains the coordinate vectors of two markers Marker[Node|Object|Body]Coordinates attached to nodes or bodies. The marker uses the objects LTG-lists to build the according coordinate mappings.
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

//! AUTO: CObjectConnectorCoordinateVector
class CObjectConnectorCoordinateVector: public CObjectConstraint // AUTO: 
{
protected: // AUTO: 
    CObjectConnectorCoordinateVectorParameters parameters; //! AUTO: contains all parameters for CObjectConnectorCoordinateVector

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectConnectorCoordinateVectorParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectConnectorCoordinateVectorParameters& GetParameters() const { return parameters; }

    //! AUTO:  default function to return Marker numbers
    virtual const ArrayIndex& GetMarkerNumbers() const override
    {
        return parameters.markerNumbers;
    }

    //! AUTO:  constraints uses Lagrance multiplier formulation
    virtual bool IsPenaltyConnector() const override
    {
        return false;
    }

    //! AUTO:  connector is time dependent if user functions are defined
    virtual bool IsTimeDependent() const override
    {
        return false;
    }

    //! AUTO:  Return true, if constraint currently is formulated at velocity level (e.g. coordinate constraint ==> this information is needed for correct jacobian computation)
    virtual bool UsesVelocityLevel() const override
    {
        return parameters.velocityLevel;
    }

    //! AUTO:  Computational function: compute algebraic equations and write residual into 'algebraicEquations'; velocityLevel: equation provided at velocity level
    virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t,  Index itemIndex, bool velocityLevel = false) const override;

    //! AUTO:  compute derivative of algebraic equations w.r.t. \hac{ODE2}, \hac{ODE2t}, \hac{ODE1} and \hac{AE} coordinates in jacobian [flags ODE2_t_AE_function, AE_AE_function, etc. need to be set in GetAvailableJacobians()]; jacobianODE2[_t] has dimension GetAlgebraicEquationsSize() x GetODE2Size() ; q are the system coordinates; markerData provides according marker information to compute jacobians
    virtual void ComputeJacobianAE(ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE, const MarkerDataStructure& markerData, Real t, Index itemIndex) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags; available jacobians is switched depending on velocity level and on activeConnector condition
    virtual JacobianType::Type GetAvailableJacobians() const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData, Index itemIndex, Vector& value) const override;

    //! AUTO:  provide requested markerType for connector
    virtual Marker::Type GetRequestedMarkerType() const override
    {
        return Marker::Coordinates;
    }

    //! AUTO:  return object type (for node treatment in computation)
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);
    }

    //! AUTO:  number of algebraic equations; independent of node/body coordinates
    virtual Index GetAlgebraicEquationsSize() const override;

    //! AUTO:  return if connector is active-->speeds up computation
    virtual bool IsActive() const override
    {
        return parameters.activeConnector;
    }

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::ConstraintEquation +
            (Index)OutputVariableType::Force );
    }

};



#endif //#ifdef include once...
