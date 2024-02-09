/** ***********************************************************************************************
* @class        CObjectGenericODE1Parameters
* @brief        Parameter class for CObjectGenericODE1
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2024-02-03  15:27:06 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTGENERICODE1PARAMETERS__H
#define COBJECTGENERICODE1PARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
#include <pybind11/numpy.h>//for NumpyMatrix
#include <pybind11/stl.h>//for NumpyMatrix
#include <pybind11/pybind11.h>
typedef py::array_t<Real> NumpyMatrix; 
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectGenericODE1Parameters
class CObjectGenericODE1Parameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex nodeNumbers;                       //!< AUTO: node numbers which provide the coordinates for the object (consecutively as provided in this list)
    Matrix systemMatrix;                          //!< AUTO: system matrix (state space matrix) of first order ODE
    Vector rhsVector;                             //!< AUTO: a constant rhs vector (e.g., for constant input)
    PythonUserFunctionBase< std::function<StdVector(const MainSystem&,Real,Index,StdVector)> > rhsUserFunction;//!< AUTO: A Python user function which computes the right-hand-side (rhs) of the first order ODE; see description below
    ArrayIndex coordinateIndexPerNode;            //!< AUTO: this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed
    //! AUTO: default constructor with parameter initialization
    CObjectGenericODE1Parameters()
    {
        nodeNumbers = ArrayIndex();
        systemMatrix = Matrix();
        rhsVector = Vector();
        rhsUserFunction = 0;
        coordinateIndexPerNode = ArrayIndex();
    };
};


/** ***********************************************************************************************
* @class        CObjectGenericODE1
* @brief        A system of \f$n\f$ \acf{ODE1}, having a system matrix, a rhs vector, but mostly it will use a user function to describe special \hac{ODE1} systems. It is based on NodeGenericODE1 nodes. NOTE that all matrices, vectors, etc. must have the same dimensions \f$n\f$ or \f$(n \times n)\f$, or they must be empty \f$(0 \times 0)\f$, using [] in Python.
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

//! AUTO: CObjectGenericODE1
class CObjectGenericODE1: public CObject // AUTO: 
{
protected: // AUTO: 
    CObjectGenericODE1Parameters parameters; //! AUTO: contains all parameters for CObjectGenericODE1
    mutable Vector tempCoordinates;               //!< AUTO: temporary vector containing coordinates
    mutable Vector tempCoordinates_t;             //!< AUTO: temporary vector containing velocity coordinates

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    CObjectGenericODE1()
    {
        tempCoordinates = Vector();
        tempCoordinates_t = Vector();
    };

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectGenericODE1Parameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectGenericODE1Parameters& GetParameters() const { return parameters; }

    //! AUTO:  Write (Reference) access to:\f$\cv_{temp} \in \Rcal^{n}\f$temporary vector containing coordinates
    void SetTempCoordinates(const Vector& value) { tempCoordinates = value; }
    //! AUTO:  Read (Reference) access to:\f$\cv_{temp} \in \Rcal^{n}\f$temporary vector containing coordinates
    const Vector& GetTempCoordinates() const { return tempCoordinates; }
    //! AUTO:  Read (Reference) access to:\f$\cv_{temp} \in \Rcal^{n}\f$temporary vector containing coordinates
    Vector& GetTempCoordinates() { return tempCoordinates; }

    //! AUTO:  Write (Reference) access to:\f$\dot \cv_{temp} \in \Rcal^{n}\f$temporary vector containing velocity coordinates
    void SetTempCoordinates_t(const Vector& value) { tempCoordinates_t = value; }
    //! AUTO:  Read (Reference) access to:\f$\dot \cv_{temp} \in \Rcal^{n}\f$temporary vector containing velocity coordinates
    const Vector& GetTempCoordinates_t() const { return tempCoordinates_t; }
    //! AUTO:  Read (Reference) access to:\f$\dot \cv_{temp} \in \Rcal^{n}\f$temporary vector containing velocity coordinates
    Vector& GetTempCoordinates_t() { return tempCoordinates_t; }

    //! AUTO:  return true, if object has a computation user function
    virtual bool HasUserFunction() const override
    {
        return (parameters.rhsUserFunction!=0);
    }

    //! AUTO:  Computational function: compute right-hand-side (RHS) of first order ordinary differential equations (ODE) to 'ode1Rhs'
    virtual void ComputeODE1RHS(Vector& ode1Rhs, Index objectNumber) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::ODE1_ODE1);
    }

    //! AUTO:  Flags to determine, which access (forces, moments, connectors, ...) to object are possible
    virtual AccessFunctionType GetAccessFunctionTypes() const override;

    //! AUTO:  provide Jacobian at localPosition in 'value' according to object access
    virtual void GetAccessFunction(AccessFunctionType accessType, Matrix& value) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariable(OutputVariableType variableType, Vector& value, ConfigurationType configuration, Index objectNumber) const override;

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        return parameters.nodeNumbers[localIndex];
    }

    //! AUTO:  number of nodes; needed for every object
    virtual Index GetNumberOfNodes() const override
    {
        return parameters.nodeNumbers.NumberOfItems();
    }

    //! AUTO:  number of \hac{ODE1} coordinates; needed for object?
    virtual Index GetODE1Size() const override;

    //! AUTO:  Get type of object, e.g. to categorize and distinguish during assembly and computation
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::MultiNoded);
    }

    //! AUTO:  This flag is reset upon change of parameters; says that the vector of coordinate indices has changed
    virtual void ParametersHaveChanged() override
    {
        InitializeCoordinateIndices();
    }

    //! AUTO:  read access to coordinate index array
    Index GetLocalODE1CoordinateIndexPerNode(Index localNode) const
    {
        return parameters.coordinateIndexPerNode[localNode];
    }

    //! AUTO:  compute object coordinates composed from all nodal coordinates; does not include reference coordinates
    void ComputeObjectCoordinates(Vector& coordinates, Vector& coordinates_t, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  compute object coordinates composed from all nodal coordinates; does not include reference coordinates
    void ComputeObjectCoordinates(Vector& coordinates, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  initialize coordinateIndexPerNode array
    void InitializeCoordinateIndices();

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionRHS(Vector& rhs, const MainSystemBase& mainSystem, Real t, Index objectNumber, const StdVector& coordinates) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Coordinates +
            (Index)OutputVariableType::Coordinates_t );
    }

};



#endif //#ifdef include once...
