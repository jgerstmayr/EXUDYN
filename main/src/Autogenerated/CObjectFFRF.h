/** ***********************************************************************************************
* @class        CObjectFFRFParameters
* @brief        Parameter class for CObjectFFRF
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-06-25  13:31:26 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTFFRFPARAMETERS__H
#define COBJECTFFRFPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include <pybind11/numpy.h>//for NumpyMatrix
#include <pybind11/stl.h>//for NumpyMatrix
#include <pybind11/pybind11.h>
typedef py::array_t<Real> NumpyMatrix; 
#include "Pymodules/PyMatrixContainer.h"//for some FFRF matrices
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectFFRFParameters
class CObjectFFRFParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex nodeNumbers;                       //!< AUTO: node numbers which provide the coordinates for the object (consecutively as provided in this list); the \f$(n_\mathrm{nf}+1)\f$ nodes represent the nodes of the FE mesh (except for node 0); the global nodal position needs to be reconstructed from the rigid-body motion of the reference frame
    PyMatrixContainer massMatrixFF;               //!< AUTO: body-fixed and ONLY flexible coordinates part of mass matrix of object given in python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format
    PyMatrixContainer stiffnessMatrixFF;          //!< AUTO: body-fixed and ONLY flexible coordinates part of stiffness matrix of object in python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format
    PyMatrixContainer dampingMatrixFF;            //!< AUTO: body-fixed and ONLY flexible coordinates part of damping matrix of object in python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format
    Vector forceVector;                           //!< AUTO: generalized, force vector added to RHS; the rigid body part \f$\fv_r\f$ is directly applied to rigid body coordinates while the flexible part \f$\fv\indf\f$ is transformed from global to local coordinates; note that this force vector only allows to add gravity forces for bodies with COM at the origin of the reference frame
    std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector)> forceUserFunction;//!< AUTO: A python user function which computes the generalized user force vector for the ODE2 equations; note the different coordinate systems for rigid body and flexible part; The function args are mbs, time, objectNumber, coordinates q (without reference values) and coordinate velocities q\_t; see description below
    std::function<NumpyMatrix(const MainSystem&,Real,Index,StdVector,StdVector)> massMatrixUserFunction;//!< AUTO: A python user function which computes the TOTAL mass matrix (including reference node) and adds the local constant mass matrix; note the different coordinate systems as described in the FFRF mass matrix; see description below
    bool computeFFRFterms;                        //!< AUTO: flag decides whether the standard FFRF terms are computed; use this flag for user-defined definition of FFRF terms in mass matrix and quadratic velocity vector
    //! AUTO: default constructor with parameter initialization
    CObjectFFRFParameters()
    {
        nodeNumbers = ArrayIndex();
        massMatrixFF = PyMatrixContainer();
        stiffnessMatrixFF = PyMatrixContainer();
        dampingMatrixFF = PyMatrixContainer();
        forceVector = Vector();
        forceUserFunction = 0;
        massMatrixUserFunction = 0;
        computeFFRFterms = true;
    };
};


/** ***********************************************************************************************
* @class        CObjectFFRF
* @brief        This object is used to represent equations modelled by the floating frame of reference formulation (FFRF). It contains a RigidBodyNode (always node 0) and a list of other nodes representing the finite element nodes used in the FFRF. Note that temporary matrices and vectors are subject of change in future.
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

//! AUTO: CObjectFFRF
class CObjectFFRF: public CObjectSuperElement // AUTO: 
{
protected: // AUTO: 
    CObjectFFRFParameters parameters; //! AUTO: contains all parameters for CObjectFFRF
    ArrayIndex coordinateIndexPerNode;            //!< AUTO: this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed
    bool objectIsInitialized;                     //!< AUTO: ALWAYS set to False! flag used to correctly initialize all FFRF matrices; as soon as this flag is False, internal (constant) FFRF matrices are recomputed during Assemble()
    Real physicsMass;                             //!< AUTO: total mass [SI:kg] of FFRF object, auto-computed from mass matrix \f$\LU{b}{\Mm}\f$
    Matrix3D physicsInertia;                      //!< AUTO: inertia tensor [SI:kgm\f$^2\f$] of rigid body w.r.t. to the reference point of the body, auto-computed from the mass matrix \f$\LU{b}{\Mm}\f$
    Vector3D physicsCenterOfMass;                 //!< AUTO: local position of center of mass (COM); auto-computed from mass matrix \f$\LU{b}{\Mm}\f$
    Matrix PHItTM;                                //!< AUTO: projector matrix; may be removed in future
    Vector referencePositions;                    //!< AUTO: vector containing the reference positions of all flexible nodes
    mutable Vector tempVector;                    //!< AUTO: temporary vector
    mutable Vector tempCoordinates;               //!< AUTO: temporary vector containing coordinates
    mutable Vector tempCoordinates_t;             //!< AUTO: temporary vector containing velocity coordinates
    mutable Matrix tempRefPosSkew;                //!< AUTO: temporary matrix with skew symmetric local (deformed) node positions
    mutable Matrix tempVelSkew;                   //!< AUTO: temporary matrix with skew symmetric local node velocities
    mutable ResizableMatrix tempMatrix;           //!< AUTO: temporary matrix
    mutable ResizableMatrix tempMatrix2;          //!< AUTO: other temporary matrix

public: // AUTO: 
    static constexpr Index ffrfNodeDim = 3; //dimension of nodes (=displacement coordinates per node)
    static constexpr Index rigidBodyNodeNumber  = 0; //number of rigid body node (usually = 0)
    //! AUTO: default constructor with parameter initialization
    CObjectFFRF()
    {
        coordinateIndexPerNode = ArrayIndex();
        objectIsInitialized = false;
        physicsMass = 0.;
        physicsInertia = EXUmath::unitMatrix3D;
        physicsCenterOfMass = Vector3D({0.,0.,0.});
        PHItTM = Matrix();
        referencePositions = Vector();
        tempVector = Vector();
        tempCoordinates = Vector();
        tempCoordinates_t = Vector();
        tempRefPosSkew = Matrix();
        tempVelSkew = Matrix();
        tempMatrix = ResizableMatrix();
        tempMatrix2 = ResizableMatrix();
    };

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectFFRFParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectFFRFParameters& GetParameters() const { return parameters; }

    //! AUTO:  Write (Reference) access to:this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed
    void SetCoordinateIndexPerNode(const ArrayIndex& value) { coordinateIndexPerNode = value; }
    //! AUTO:  Read (Reference) access to:this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed
    const ArrayIndex& GetCoordinateIndexPerNode() const { return coordinateIndexPerNode; }
    //! AUTO:  Read (Reference) access to:this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed
    ArrayIndex& GetCoordinateIndexPerNode() { return coordinateIndexPerNode; }

    //! AUTO:  Write (Reference) access to:ALWAYS set to False! flag used to correctly initialize all FFRF matrices; as soon as this flag is False, internal (constant) FFRF matrices are recomputed during Assemble()
    void SetObjectIsInitialized(const bool& value) { objectIsInitialized = value; }
    //! AUTO:  Read (Reference) access to:ALWAYS set to False! flag used to correctly initialize all FFRF matrices; as soon as this flag is False, internal (constant) FFRF matrices are recomputed during Assemble()
    const bool& GetObjectIsInitialized() const { return objectIsInitialized; }
    //! AUTO:  Read (Reference) access to:ALWAYS set to False! flag used to correctly initialize all FFRF matrices; as soon as this flag is False, internal (constant) FFRF matrices are recomputed during Assemble()
    bool& GetObjectIsInitialized() { return objectIsInitialized; }

    //! AUTO:  Write (Reference) access to:\f$m\f$total mass [SI:kg] of FFRF object, auto-computed from mass matrix \f$\LU{b}{\Mm}\f$
    void SetPhysicsMass(const Real& value) { physicsMass = value; }
    //! AUTO:  Read (Reference) access to:\f$m\f$total mass [SI:kg] of FFRF object, auto-computed from mass matrix \f$\LU{b}{\Mm}\f$
    const Real& GetPhysicsMass() const { return physicsMass; }
    //! AUTO:  Read (Reference) access to:\f$m\f$total mass [SI:kg] of FFRF object, auto-computed from mass matrix \f$\LU{b}{\Mm}\f$
    Real& GetPhysicsMass() { return physicsMass; }

    //! AUTO:  Write (Reference) access to:\f$J_r \in \Rcal^{3 \times 3}\f$inertia tensor [SI:kgm\f$^2\f$] of rigid body w.r.t. to the reference point of the body, auto-computed from the mass matrix \f$\LU{b}{\Mm}\f$
    void SetPhysicsInertia(const Matrix3D& value) { physicsInertia = value; }
    //! AUTO:  Read (Reference) access to:\f$J_r \in \Rcal^{3 \times 3}\f$inertia tensor [SI:kgm\f$^2\f$] of rigid body w.r.t. to the reference point of the body, auto-computed from the mass matrix \f$\LU{b}{\Mm}\f$
    const Matrix3D& GetPhysicsInertia() const { return physicsInertia; }
    //! AUTO:  Read (Reference) access to:\f$J_r \in \Rcal^{3 \times 3}\f$inertia tensor [SI:kgm\f$^2\f$] of rigid body w.r.t. to the reference point of the body, auto-computed from the mass matrix \f$\LU{b}{\Mm}\f$
    Matrix3D& GetPhysicsInertia() { return physicsInertia; }

    //! AUTO:  Write (Reference) access to:\f$\LU{b}{\pv}_{COM}\f$local position of center of mass (COM); auto-computed from mass matrix \f$\LU{b}{\Mm}\f$
    void SetPhysicsCenterOfMass(const Vector3D& value) { physicsCenterOfMass = value; }
    //! AUTO:  Read (Reference) access to:\f$\LU{b}{\pv}_{COM}\f$local position of center of mass (COM); auto-computed from mass matrix \f$\LU{b}{\Mm}\f$
    const Vector3D& GetPhysicsCenterOfMass() const { return physicsCenterOfMass; }
    //! AUTO:  Read (Reference) access to:\f$\LU{b}{\pv}_{COM}\f$local position of center of mass (COM); auto-computed from mass matrix \f$\LU{b}{\Mm}\f$
    Vector3D& GetPhysicsCenterOfMass() { return physicsCenterOfMass; }

    //! AUTO:  Write (Reference) access to:\f$\tPhi\indt\tp \in \Rcal^{n\indf \times 3}\f$projector matrix; may be removed in future
    void SetPHItTM(const Matrix& value) { PHItTM = value; }
    //! AUTO:  Read (Reference) access to:\f$\tPhi\indt\tp \in \Rcal^{n\indf \times 3}\f$projector matrix; may be removed in future
    const Matrix& GetPHItTM() const { return PHItTM; }
    //! AUTO:  Read (Reference) access to:\f$\tPhi\indt\tp \in \Rcal^{n\indf \times 3}\f$projector matrix; may be removed in future
    Matrix& GetPHItTM() { return PHItTM; }

    //! AUTO:  Write (Reference) access to:\f$\xv\cRef \in \Rcal^{n\indf}\f$vector containing the reference positions of all flexible nodes
    void SetReferencePositions(const Vector& value) { referencePositions = value; }
    //! AUTO:  Read (Reference) access to:\f$\xv\cRef \in \Rcal^{n\indf}\f$vector containing the reference positions of all flexible nodes
    const Vector& GetReferencePositions() const { return referencePositions; }
    //! AUTO:  Read (Reference) access to:\f$\xv\cRef \in \Rcal^{n\indf}\f$vector containing the reference positions of all flexible nodes
    Vector& GetReferencePositions() { return referencePositions; }

    //! AUTO:  Write (Reference) access to:\f$\vv_{temp} \in \Rcal^{n\indf}\f$temporary vector
    void SetTempVector(const Vector& value) { tempVector = value; }
    //! AUTO:  Read (Reference) access to:\f$\vv_{temp} \in \Rcal^{n\indf}\f$temporary vector
    const Vector& GetTempVector() const { return tempVector; }
    //! AUTO:  Read (Reference) access to:\f$\vv_{temp} \in \Rcal^{n\indf}\f$temporary vector
    Vector& GetTempVector() { return tempVector; }

    //! AUTO:  Write (Reference) access to:\f$\cv_{temp} \in \Rcal^{n\indf}\f$temporary vector containing coordinates
    void SetTempCoordinates(const Vector& value) { tempCoordinates = value; }
    //! AUTO:  Read (Reference) access to:\f$\cv_{temp} \in \Rcal^{n\indf}\f$temporary vector containing coordinates
    const Vector& GetTempCoordinates() const { return tempCoordinates; }
    //! AUTO:  Read (Reference) access to:\f$\cv_{temp} \in \Rcal^{n\indf}\f$temporary vector containing coordinates
    Vector& GetTempCoordinates() { return tempCoordinates; }

    //! AUTO:  Write (Reference) access to:\f$\dot \cv_{temp} \in \Rcal^{n\indf}\f$temporary vector containing velocity coordinates
    void SetTempCoordinates_t(const Vector& value) { tempCoordinates_t = value; }
    //! AUTO:  Read (Reference) access to:\f$\dot \cv_{temp} \in \Rcal^{n\indf}\f$temporary vector containing velocity coordinates
    const Vector& GetTempCoordinates_t() const { return tempCoordinates_t; }
    //! AUTO:  Read (Reference) access to:\f$\dot \cv_{temp} \in \Rcal^{n\indf}\f$temporary vector containing velocity coordinates
    Vector& GetTempCoordinates_t() { return tempCoordinates_t; }

    //! AUTO:  Write (Reference) access to:\f$\tilde\pv\indf \in \Rcal^{n\indf \times 3}\f$temporary matrix with skew symmetric local (deformed) node positions
    void SetTempRefPosSkew(const Matrix& value) { tempRefPosSkew = value; }
    //! AUTO:  Read (Reference) access to:\f$\tilde\pv\indf \in \Rcal^{n\indf \times 3}\f$temporary matrix with skew symmetric local (deformed) node positions
    const Matrix& GetTempRefPosSkew() const { return tempRefPosSkew; }
    //! AUTO:  Read (Reference) access to:\f$\tilde\pv\indf \in \Rcal^{n\indf \times 3}\f$temporary matrix with skew symmetric local (deformed) node positions
    Matrix& GetTempRefPosSkew() { return tempRefPosSkew; }

    //! AUTO:  Write (Reference) access to:\f$\dot{\tilde\cv}\indf \in \Rcal^{n\indf \times 3}\f$temporary matrix with skew symmetric local node velocities
    void SetTempVelSkew(const Matrix& value) { tempVelSkew = value; }
    //! AUTO:  Read (Reference) access to:\f$\dot{\tilde\cv}\indf \in \Rcal^{n\indf \times 3}\f$temporary matrix with skew symmetric local node velocities
    const Matrix& GetTempVelSkew() const { return tempVelSkew; }
    //! AUTO:  Read (Reference) access to:\f$\dot{\tilde\cv}\indf \in \Rcal^{n\indf \times 3}\f$temporary matrix with skew symmetric local node velocities
    Matrix& GetTempVelSkew() { return tempVelSkew; }

    //! AUTO:  Write (Reference) access to:\f$\Xm_{temp} \in \Rcal^{n\indf \times 3}\f$temporary matrix
    void SetTempMatrix(const ResizableMatrix& value) { tempMatrix = value; }
    //! AUTO:  Read (Reference) access to:\f$\Xm_{temp} \in \Rcal^{n\indf \times 3}\f$temporary matrix
    const ResizableMatrix& GetTempMatrix() const { return tempMatrix; }
    //! AUTO:  Read (Reference) access to:\f$\Xm_{temp} \in \Rcal^{n\indf \times 3}\f$temporary matrix
    ResizableMatrix& GetTempMatrix() { return tempMatrix; }

    //! AUTO:  Write (Reference) access to:\f$\Xm_{temp2} \in \Rcal^{n\indf \times 4}\f$other temporary matrix
    void SetTempMatrix2(const ResizableMatrix& value) { tempMatrix2 = value; }
    //! AUTO:  Read (Reference) access to:\f$\Xm_{temp2} \in \Rcal^{n\indf \times 4}\f$other temporary matrix
    const ResizableMatrix& GetTempMatrix2() const { return tempMatrix2; }
    //! AUTO:  Read (Reference) access to:\f$\Xm_{temp2} \in \Rcal^{n\indf \times 4}\f$other temporary matrix
    ResizableMatrix& GetTempMatrix2() { return tempMatrix2; }

    //! AUTO:  Computational function: compute mass matrix
    virtual void ComputeMassMatrix(Matrix& massMatrix, Index objectNumber) const override;

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs, Index objectNumber) const override;

    //! AUTO:  Compute algebraic equations part of rigid body
    virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, bool useIndex2 = false) const override;

    //! AUTO:  Compute jacobians of algebraic equations part of rigid body w.r.t. ODE2, ODE2_t, ODE1, AE
    virtual void ComputeJacobianAE(ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::AE_ODE2 + JacobianType::AE_ODE2_function);
    }

    //! AUTO:  Flags to determine, which access (forces, moments, connectors, ...) to object are possible
    virtual AccessFunctionType GetAccessFunctionTypes() const override;

    //! AUTO:  provide Jacobian at localPosition in 'value' according to object access
    virtual void GetAccessFunctionBody(AccessFunctionType accessType, const Vector3D& localPosition, Matrix& value) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableBody(OutputVariableType variableType, const Vector3D& localPosition, ConfigurationType configuration, Vector& value, Index objectNumber) const override;

    //! AUTO:  return the (global) position of 'localPosition' according to configuration type
    virtual Vector3D GetPosition(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) position of 'localPosition' according to configuration type
    virtual Vector3D GetDisplacement(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) velocity of 'localPosition' according to configuration type
    virtual Vector3D GetVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent rotation matrix of node; returns always a 3D Matrix, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
    virtual Matrix3D GetRotationMatrix(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
    virtual Vector3D GetAngularVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
    virtual Vector3D GetAngularVelocityLocal(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the local position of the center of mass, needed for massProportionalLoad; this is only the reference-frame part!
    virtual Vector3D GetLocalCenterOfMass() const override
    {
        return physicsCenterOfMass;
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

    //! AUTO:  number of AE coordinates; depends on node
    virtual Index GetAlgebraicEquationsSize() const override;

    //! AUTO:  Get type of object, e.g. to categorize and distinguish during assembly and computation
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded + (Index)CObjectType::SuperElement);
    }

    //! AUTO:  return true if object has time and coordinate independent (=constant) mass matrix
    virtual bool HasConstantMassMatrix() const override
    {
        return false;
    }

    //! AUTO:  This flag is reset upon change of parameters; says that the vector of coordinate indices has changed
    virtual void ParametersHaveChanged() override
    {
        objectIsInitialized = false;
    }

    //! AUTO:  operations done after Assemble()
    virtual void PostAssemble() override
    {
        InitializeObject();
    }

    //! AUTO:  read access to coordinate index array
    virtual Index GetLocalODE2CoordinateIndexPerNode(Index localNode) const override
    {
        return coordinateIndexPerNode[localNode];
    }

    //! AUTO:  compute object coordinates composed from all nodal coordinates; does not include reference coordinates
    void ComputeObjectCoordinates(Vector& coordinates, Vector& coordinates_t, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  compute object acceleration coordinates composed from all nodal coordinates
    void ComputeObjectCoordinates_tt(Vector& coordinates_tt, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  initialize coordinateIndexPerNode array
    void InitializeObject();

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionForce(Vector& force, const MainSystemBase& mainSystem, Real t, Index objectNumber, const StdVector& coordinates, const StdVector& coordinates_t) const;

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionMassMatrix(Matrix& massMatrix, const MainSystemBase& mainSystem, Real t, Index objectNumber, const StdVector& coordinates, const StdVector& coordinates_t) const;

    //! AUTO:  always true, because ObjectFFRF; return according LOCAL node number
    virtual bool HasReferenceFrame(Index& localReferenceFrameNode) const override
    {
        localReferenceFrameNode = rigidBodyNodeNumber; return true;
    }

    //! AUTO:  return the number of mesh nodes, which is 1 less than the number of nodes (but different in other SuperElements)
    virtual Index GetNumberOfMeshNodes() const override
    {
        return GetNumberOfNodes()-1;
    }

    //! AUTO:  return the (local) position of a mesh node according to configuration type; use Configuration.Reference to access the mesh reference position; meshNodeNumber is the local node number of the (underlying) mesh
    virtual Vector3D GetMeshNodeLocalPosition(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (local) velocity of a mesh node according to configuration type; meshNodeNumber is the local node number of the (underlying) mesh
    virtual Vector3D GetMeshNodeLocalVelocity(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (local) acceleration of a mesh node according to configuration type; meshNodeNumber is the local node number of the (underlying) mesh
    virtual Vector3D GetMeshNodeLocalAcceleration(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) position of a mesh node according to configuration type; this is the node position transformed by the motion of the reference frame; meshNodeNumber is the local node number of the (underlying) mesh
    virtual Vector3D GetMeshNodePosition(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) velocity of a mesh node according to configuration type; this is the node position transformed by the motion of the reference frame; meshNodeNumber is the local node number of the (underlying) mesh
    virtual Vector3D GetMeshNodeVelocity(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) acceleration of a mesh node according to configuration type; this is the node position transformed by the motion of the reference frame; meshNodeNumber is the local node number of the (underlying) mesh
    virtual Vector3D GetMeshNodeAcceleration(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const override;

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
            (Index)OutputVariableType::Coordinates_tt +
            (Index)OutputVariableType::Force );
    }

};



#endif //#ifdef include once...
