/** ***********************************************************************************************
* @class        CObjectGenericODE2Parameters
* @brief        Parameter class for CObjectGenericODE2
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-11-30  20:42:27 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTGENERICODE2PARAMETERS__H
#define COBJECTGENERICODE2PARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

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
    std::function<StdVector(Real, StdVector,StdVector)> forceUserFunction;//!< AUTO: A python user function which computes the generalized user force vector for the ODE2 equations; see description below
    std::function<NumpyMatrix(Real, StdVector,StdVector)> massMatrixUserFunction;//!< AUTO: A python user function which computes the mass matrix instead of the constant mass matrix; see description below
    ArrayIndex coordinateIndexPerNode;            //!< AUTO: this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed
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
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

//! AUTO: CObjectGenericODE2
class CObjectGenericODE2: public CObjectSuperElement // AUTO: 
{
protected: // AUTO: 
    CObjectGenericODE2Parameters parameters; //! AUTO: contains all parameters for CObjectGenericODE2

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectGenericODE2Parameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectGenericODE2Parameters& GetParameters() const { return parameters; }

    //! AUTO:  Computational function: compute mass matrix
    virtual void ComputeMassMatrix(Matrix& massMatrix) const override;

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs) const override;

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

    //! AUTO:  return the local position of the center of mass, used for massProportionalLoad, which may NOT be appropriate for GenericODE2
    virtual Vector3D GetLocalCenterOfMass() const override
    {
        return Vector3D({0.,0.,0.});
    }

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
        return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded + (Index)CObjectType::SuperElement);
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

    //! AUTO:  return true, if object has reference frame; return according LOCAL node number
    virtual bool HasReferenceFrame(Index& localReferenceFrameNode) const override
    {
        localReferenceFrameNode = 0; return false;
    }

    //! AUTO:  return the number of mesh nodes, which is 1 less than the number of nodes if referenceFrame is used
    virtual Index GetNumberOfMeshNodes() const override
    {
        return GetNumberOfNodes();
    }

    //! AUTO:  return the (local) position of a mesh node according to configuration type; use Configuration.Reference to access the mesh reference position; meshNodeNumber is the local node number of the (underlying) mesh
    virtual Vector3D GetMeshNodeLocalPosition(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (local) velocity of a mesh node according to configuration type; meshNodeNumber is the local node number of the (underlying) mesh
    virtual Vector3D GetMeshNodeLocalVelocity(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) position of a mesh node according to configuration type; this is the node position transformed by the motion of the reference frame; meshNodeNumber is the local node number of the (underlying) mesh
    virtual Vector3D GetMeshNodePosition(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) velocity of a mesh node according to configuration type; this is the node position transformed by the motion of the reference frame; meshNodeNumber is the local node number of the (underlying) mesh
    virtual Vector3D GetMeshNodeVelocity(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  compute Jacobian with weightingMatrix (WM) and/or meshNodeNumbers, which define how the SuperElement mesh nodes or coordinates are transformed to a global position; for details see CObjectSuperElement header file
    virtual void GetAccessFunctionSuperElement(AccessFunctionType accessType, const Matrix& weightingMatrix, const ArrayIndex& meshNodeNumbers, Matrix& value) const override;

    //! AUTO:  get extended output variable types for multi-nodal objects with mesh nodes; some objects have meshNode-dependent OutputVariableTypes
    virtual OutputVariableType GetOutputVariableTypesSuperElement(Index meshNodeNumber) const override;

    //! AUTO:  get extended output variables for multi-nodal objects with mesh nodes
    virtual void GetOutputVariableSuperElement(OutputVariableType variableType, Index meshNodeNumber, ConfigurationType configuration, Vector& value) const override;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Coordinates +
            (Index)OutputVariableType::Coordinates_t +
            (Index)OutputVariableType::Force );
    }

};



#endif //#ifdef include once...
