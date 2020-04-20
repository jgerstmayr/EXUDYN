/** ***********************************************************************************************
* @class        CObjectGenericODE2Parameters
* @brief        Parameter class for CObjectGenericODE2
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-04-10  10:39:20 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

#include <functional> //! AUTO: needed for std::function
#include <pybind11/numpy.h>//for NumpyMatrix
#include <pybind11/stl.h>//for NumpyMatrix
#include <pybind11/pybind11.h>
typedef py::array_t<Real> NumpyMatrix; 

//! AUTO: Parameters for class CObjectGenericODE2Parameters
class CObjectGenericODE2Parameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex nodeNumbers;                       //!< AUTO: node numbers which provide the coordinates for the object (consecutively as provided in this list)
    Matrix massMatrix;                            //!< AUTO: mass matrix of object in python numpy format
    Matrix stiffnessMatrix;                       //!< AUTO: stiffness matrix of object in python numpy format
    Matrix dampingMatrix;                         //!< AUTO: damping matrix of object in python numpy format
    Vector forceVector;                           //!< AUTO: generalized force vector added to RHS
    std::function<StdVector(Real, StdVector,StdVector)> forceUserFunction;//!< AUTO: A python user function which computes the generalized user force vector for the ODE2 equations; The function takes the time, coordinates q (without reference values) and coordinate velocities q\_t; Example for python function with numpy stiffness matrix K: def f(t, q, q\_t): return np.dot(K, q)
    std::function<NumpyMatrix(Real, StdVector,StdVector)> massMatrixUserFunction;//!< AUTO: A python user function which computes the mass matrix instead of the constant mass matrix; The function takes the time, coordinates q (without reference values) and coordinate velocities q\_t; Example (academic) for python function with numpy stiffness matrix M: def f(t, q, q\_t): return (q[0]+1)*M
    ArrayIndex coordinateIndexPerNode;            //!< AUTO: this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed
    bool useFirstNodeAsReferenceFrame;            //!< AUTO: set true, if first node (\f$n_0\f$) is used as floating reference frame; all other nodes are interpreted relative to the reference frame; used to implement FFRF (floating frame of reference formulation); NOTE that in this case, nodes \f$[n_1,\,\ldots,\,n_n]\tp\f$ are still drawn without the reference frame
    //! AUTO: default constructor with parameter initialization
    CObjectGenericODE2Parameters()
    {
        nodeNumbers = ArrayIndex();
        massMatrix = Matrix();
        stiffnessMatrix = Matrix();
        dampingMatrix = Matrix();
        forceVector = Vector();
        forceUserFunction = 0;
        massMatrixUserFunction = 0;
        coordinateIndexPerNode = ArrayIndex();
        useFirstNodeAsReferenceFrame = false;
    };
};


/** ***********************************************************************************************
* @class        CObjectGenericODE2
* @brief        A system of \f$n\f$ second order ordinary differential equations (ODE2), having a mass matrix, damping/gyroscopic matrix, stiffness matrix and generalized forces. It can combine generic nodes, or node points. User functions can be used to compute mass matrix and generalized forces depending on given coordinates. NOTE that all matrices, vectors, etc. must have the same dimensions \f$n\f$ or \f$(n \times n)\f$, or they must be empty \f$(0 \times 0)\f$, except for the mass matrix which always needs to have dimensions \f$(n \times n)\f$.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

//! AUTO: CObjectGenericODE2
class CObjectGenericODE2: public CObjectBody // AUTO: 
{
protected: // AUTO: 
    CObjectGenericODE2Parameters parameters; //! AUTO: contains all parameters for CObjectGenericODE2

public: // AUTO: 
    static const Index ffrfNodeNumber = 0; //floating frame of reference (ffrf) node number in body, if useFirstNodeAsReferenceFrame=True

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectGenericODE2Parameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectGenericODE2Parameters& GetParameters() const { return parameters; }

    //! AUTO:  Computational function: compute mass matrix
    virtual void ComputeMassMatrix(Matrix& massMatrix) const override;

    //! AUTO:  Computational function: compute right-hand-side (RHS) of second order ordinary differential equations (ODE) to 'ode2rhs'
    virtual void ComputeODE2RHS(Vector& ode2Rhs) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return JacobianType::_None;
    }

    //! AUTO:  Flags to determine, which access (forces, moments, connectors, ...) to object are possible
    virtual AccessFunctionType GetAccessFunctionTypes() const override;

    //! AUTO:  provide Jacobian at localPosition in 'value' according to object access
    virtual void GetAccessFunctionBody(AccessFunctionType accessType, const Vector3D& localPosition, Matrix& value) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableBody(OutputVariableType variableType, const Vector3D& localPosition, ConfigurationType configuration, Vector& value) const override;

    //! AUTO:  return the (global) position of 'localPosition' according to configuration type
    virtual Vector3D GetPosition(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) position of 'localPosition' according to configuration type
    virtual Vector3D GetDisplacement(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) velocity of 'localPosition' according to configuration type
    virtual Vector3D GetVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

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

    //! AUTO:  number of ODE2 coordinates; needed for object?
    virtual Index GetODE2Size() const override;

    //! AUTO:  Get type of object, e.g. to categorize and distinguish during assembly and computation
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded);
    }

    //! AUTO:  This flag is reset upon change of parameters; says that the vector of coordinate indices has changed
    virtual void ParametersHaveChanged() override
    {
        InitializeCoordinateIndices();
    }

    //! AUTO:  read access to coordinate index array
    virtual Index GetLocalODE2CoordinateIndexPerNode(Index localNode) const override
    {
        return parameters.coordinateIndexPerNode[localNode];
    }

    //! AUTO:  compute object coordinates composed from all nodal coordinates; does not include reference coordinates
    void ComputeObjectCoordinates(Vector& coordinates, Vector& coordinates_t, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  initialize coordinateIndexPerNode array
    void InitializeCoordinateIndices();

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Coordinates +
            (Index)OutputVariableType::Coordinates_t +
            (Index)OutputVariableType::Force );
    }

};


