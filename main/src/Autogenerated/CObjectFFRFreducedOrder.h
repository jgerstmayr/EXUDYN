/** ***********************************************************************************************
* @class        CObjectFFRFreducedOrderParameters
* @brief        Parameter class for CObjectFFRFreducedOrder
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2021-03-30  14:53:06 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTFFRFREDUCEDORDERPARAMETERS__H
#define COBJECTFFRFREDUCEDORDERPARAMETERS__H

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

//! AUTO: Parameters for class CObjectFFRFreducedOrderParameters
class CObjectFFRFreducedOrderParameters // AUTO: 
{
public: // AUTO: 
    ArrayIndex nodeNumbers;                       //!< AUTO: node numbers of rigid body node and NodeGenericODE2 for modal coordinates; the global nodal position needs to be reconstructed from the rigid-body motion of the reference frame, the modal coordinates and the mode basis
    PyMatrixContainer massMatrixReduced;          //!< AUTO: body-fixed and ONLY flexible coordinates part of reduced mass matrix; provided as MatrixContainer(sparse/dense matrix)
    PyMatrixContainer stiffnessMatrixReduced;     //!< AUTO: body-fixed and ONLY flexible coordinates part of reduced stiffness matrix; provided as MatrixContainer(sparse/dense matrix)
    PyMatrixContainer dampingMatrixReduced;       //!< AUTO: body-fixed and ONLY flexible coordinates part of reduced damping matrix; provided as MatrixContainer(sparse/dense matrix)
    std::function<StdVector(const MainSystem&,Real,StdVector,StdVector)> forceUserFunction;//!< AUTO: A python user function which computes the generalized user force vector for the ODE2 equations; see description below
    std::function<NumpyMatrix(const MainSystem&,Real,StdVector,StdVector)> massMatrixUserFunction;//!< AUTO: A python user function which computes the TOTAL mass matrix (including reference node) and adds the local constant mass matrix; see description below
    bool computeFFRFterms;                        //!< AUTO: flag decides whether the standard FFRF/CMS terms are computed; use this flag for user-defined definition of FFRF terms in mass matrix and quadratic velocity vector
    Matrix modeBasis;                             //!< AUTO: mode basis, which transforms reduced coordinates to (full) nodal coordinates, written as a single vector \f$[u_{x,n_0},\,u_{y,n_0},\,u_{z,n_0},\,\ldots,\,u_{x,n_n},\,u_{y,n_n},\,u_{z,n_n}]\tp\f$
    Matrix outputVariableModeBasis;               //!< AUTO: mode basis, which transforms reduced coordinates to output variables per mode and per node; \f$s_{OV}\f$ is the size of the output variable, e.g., 6 for stress modes (\f$S_{xx},...,S_{xy}\f$)
    OutputVariableType outputVariableTypeModeBasis;//!< AUTO: this must be the output variable type of the outputVariableModeBasis, e.g. exu.OutputVariableType.Stress
    Vector referencePositions;                    //!< AUTO: vector containing the reference positions of all flexible nodes, needed for graphics
    Matrix mPsiTildePsi;                          //!< AUTO: special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface
    Matrix mPsiTildePsiTilde;                     //!< AUTO: special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface
    Matrix mPhitTPsi;                             //!< AUTO: special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface
    Matrix mPhitTPsiTilde;                        //!< AUTO: special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface
    Matrix mXRefTildePsi;                         //!< AUTO: special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface
    Matrix mXRefTildePsiTilde;                    //!< AUTO: special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface
    //! AUTO: default constructor with parameter initialization
    CObjectFFRFreducedOrderParameters()
    {
        nodeNumbers = ArrayIndex();
        massMatrixReduced = PyMatrixContainer();
        stiffnessMatrixReduced = PyMatrixContainer();
        dampingMatrixReduced = PyMatrixContainer();
        forceUserFunction = 0;
        massMatrixUserFunction = 0;
        computeFFRFterms = true;
        modeBasis = Matrix();
        outputVariableModeBasis = Matrix();
        outputVariableTypeModeBasis = OutputVariableType::_None;
        referencePositions = Vector();
        mPsiTildePsi = Matrix();
        mPsiTildePsiTilde = Matrix();
        mPhitTPsi = Matrix();
        mPhitTPsiTilde = Matrix();
        mXRefTildePsi = Matrix();
        mXRefTildePsiTilde = Matrix();
    };
};


/** ***********************************************************************************************
* @class        CObjectFFRFreducedOrder
* @brief        This object is used to represent modally reduced flexible bodies using the floating frame of reference formulation (FFRF) and the component mode synthesis (CMS). It can be used to model real-life mechanical systems imported from finite element codes or python tools such as NETGEN/NGsolve, see the \texttt{FEMinterface} in \refSection{sec:FEM:FEMinterface:__init__}. It contains a RigidBodyNode (always node 0) and a NodeGenericODE2 representing the modal coordinates. Currently, equations must be defined within user functions, which are available in the FEM module, see class \texttt{ObjectFFRFreducedOrderInterface}, especially the user functions \texttt{UFmassFFRFreducedOrder} and \texttt{UFforceFFRFreducedOrder}, \refSection{sec:FEM:ObjectFFRFreducedOrderInterface:AddObjectFFRFreducedOrderWithUserFunctions}.
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

//! AUTO: CObjectFFRFreducedOrder
class CObjectFFRFreducedOrder: public CObjectSuperElement // AUTO: 
{
protected: // AUTO: 
    CObjectFFRFreducedOrderParameters parameters; //! AUTO: contains all parameters for CObjectFFRFreducedOrder
    bool objectIsInitialized;                     //!< AUTO: ALWAYS set to False! flag used to correctly initialize all FFRF matrices; as soon as this flag is False, some internal (constant) FFRF matrices are recomputed during Assemble()
    Real physicsMass;                             //!< AUTO: total mass [SI:kg] of FFRFreducedOrder object
    Matrix3D physicsInertia;                      //!< AUTO: inertia tensor [SI:kgm\f$^2\f$] of rigid body w.r.t. to the reference point of the body
    Vector3D physicsCenterOfMass;                 //!< AUTO: local position of center of mass (COM)
    Matrix3D physicsCenterOfMassTilde;            //!< AUTO: tilde matrix from local position of COM; autocomputed during initialization
    mutable Vector tempUserFunctionForce;         //!< AUTO: temporary vector for UF force
    mutable ResizableVector tempCoordinates;      //!< AUTO: temporary vector containing coordinates
    mutable ResizableVector tempCoordinates_t;    //!< AUTO: temporary vector containing velocity coordinates
    mutable ResizableMatrix tempKronZetaI;        //!< AUTO: temporary coordinate dependent matrix
    mutable ResizableMatrix tempKronZetaI_t;      //!< AUTO: temporary coordinate dependent matrix
    mutable ResizableMatrix tempKronIZetaOmegaT;  //!< AUTO: temporary coordinate dependent matrix
    mutable ResizableMatrix tempMatrix;           //!< AUTO: temporary matrix at several parts of computation MassMatrix, ODE2Lhs
    mutable ResizableMatrix tempMatrix2;          //!< AUTO: second temporary matrix at several parts of computation MassMatrix, ODE2Lhs
    mutable ResizableVector tempVector;           //!< AUTO: temporary vector at computation of ODE2Lhs
    mutable ResizableVector tempVector2;          //!< AUTO: second temporary vector at computation of ODE2Lhs

public: // AUTO: 
    static constexpr Index ffrfNodeDim = 3; //dimension of nodes (=displacement coordinates per node)
    static constexpr Index rigidBodyNodeNumber = 0; //node number of rigid body node (usually = 0)
    static constexpr Index genericNodeNumber = 1;//node number for modal coordinates
    //! AUTO: default constructor with parameter initialization
    CObjectFFRFreducedOrder()
    {
        objectIsInitialized = false;
        physicsMass = 0.;
        physicsInertia = EXUmath::unitMatrix3D;
        physicsCenterOfMass = Vector3D({0.,0.,0.});
        physicsCenterOfMassTilde = EXUmath::zeroMatrix3D;
        tempUserFunctionForce = Vector();
        tempCoordinates = ResizableVector();
        tempCoordinates_t = ResizableVector();
        tempKronZetaI = ResizableMatrix();
        tempKronZetaI_t = ResizableMatrix();
        tempKronIZetaOmegaT = ResizableMatrix();
        tempMatrix = ResizableMatrix();
        tempMatrix2 = ResizableMatrix();
        tempVector = ResizableVector();
        tempVector2 = ResizableVector();
    };

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectFFRFreducedOrderParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectFFRFreducedOrderParameters& GetParameters() const { return parameters; }

    //! AUTO:  Write (Reference) access to:ALWAYS set to False! flag used to correctly initialize all FFRF matrices; as soon as this flag is False, some internal (constant) FFRF matrices are recomputed during Assemble()
    void SetObjectIsInitialized(const bool& value) { objectIsInitialized = value; }
    //! AUTO:  Read (Reference) access to:ALWAYS set to False! flag used to correctly initialize all FFRF matrices; as soon as this flag is False, some internal (constant) FFRF matrices are recomputed during Assemble()
    const bool& GetObjectIsInitialized() const { return objectIsInitialized; }
    //! AUTO:  Read (Reference) access to:ALWAYS set to False! flag used to correctly initialize all FFRF matrices; as soon as this flag is False, some internal (constant) FFRF matrices are recomputed during Assemble()
    bool& GetObjectIsInitialized() { return objectIsInitialized; }

    //! AUTO:  Write (Reference) access to:\f$m\f$total mass [SI:kg] of FFRFreducedOrder object
    void SetPhysicsMass(const Real& value) { physicsMass = value; }
    //! AUTO:  Read (Reference) access to:\f$m\f$total mass [SI:kg] of FFRFreducedOrder object
    const Real& GetPhysicsMass() const { return physicsMass; }
    //! AUTO:  Read (Reference) access to:\f$m\f$total mass [SI:kg] of FFRFreducedOrder object
    Real& GetPhysicsMass() { return physicsMass; }

    //! AUTO:  Write (Reference) access to:\f$\Jm_r \in \Rcal^{3 \times 3}\f$inertia tensor [SI:kgm\f$^2\f$] of rigid body w.r.t. to the reference point of the body
    void SetPhysicsInertia(const Matrix3D& value) { physicsInertia = value; }
    //! AUTO:  Read (Reference) access to:\f$\Jm_r \in \Rcal^{3 \times 3}\f$inertia tensor [SI:kgm\f$^2\f$] of rigid body w.r.t. to the reference point of the body
    const Matrix3D& GetPhysicsInertia() const { return physicsInertia; }
    //! AUTO:  Read (Reference) access to:\f$\Jm_r \in \Rcal^{3 \times 3}\f$inertia tensor [SI:kgm\f$^2\f$] of rigid body w.r.t. to the reference point of the body
    Matrix3D& GetPhysicsInertia() { return physicsInertia; }

    //! AUTO:  Write (Reference) access to:\f$\LU{b}{\pv}_{COM}\f$local position of center of mass (COM)
    void SetPhysicsCenterOfMass(const Vector3D& value) { physicsCenterOfMass = value; }
    //! AUTO:  Read (Reference) access to:\f$\LU{b}{\pv}_{COM}\f$local position of center of mass (COM)
    const Vector3D& GetPhysicsCenterOfMass() const { return physicsCenterOfMass; }
    //! AUTO:  Read (Reference) access to:\f$\LU{b}{\pv}_{COM}\f$local position of center of mass (COM)
    Vector3D& GetPhysicsCenterOfMass() { return physicsCenterOfMass; }

    //! AUTO:  Write (Reference) access to:\f$\LU{b}{\tilde \pv}_{COM}\f$tilde matrix from local position of COM; autocomputed during initialization
    void SetPhysicsCenterOfMassTilde(const Matrix3D& value) { physicsCenterOfMassTilde = value; }
    //! AUTO:  Read (Reference) access to:\f$\LU{b}{\tilde \pv}_{COM}\f$tilde matrix from local position of COM; autocomputed during initialization
    const Matrix3D& GetPhysicsCenterOfMassTilde() const { return physicsCenterOfMassTilde; }
    //! AUTO:  Read (Reference) access to:\f$\LU{b}{\tilde \pv}_{COM}\f$tilde matrix from local position of COM; autocomputed during initialization
    Matrix3D& GetPhysicsCenterOfMassTilde() { return physicsCenterOfMassTilde; }

    //! AUTO:  Write (Reference) access to:\f$\fv_{temp} \in \Rcal^{n_{ODE2}}\f$temporary vector for UF force
    void SetTempUserFunctionForce(const Vector& value) { tempUserFunctionForce = value; }
    //! AUTO:  Read (Reference) access to:\f$\fv_{temp} \in \Rcal^{n_{ODE2}}\f$temporary vector for UF force
    const Vector& GetTempUserFunctionForce() const { return tempUserFunctionForce; }
    //! AUTO:  Read (Reference) access to:\f$\fv_{temp} \in \Rcal^{n_{ODE2}}\f$temporary vector for UF force
    Vector& GetTempUserFunctionForce() { return tempUserFunctionForce; }

    //! AUTO:  Write (Reference) access to:\f$\pv_{temp} \in \Rcal^{n\indf}\f$temporary vector containing coordinates
    void SetTempCoordinates(const ResizableVector& value) { tempCoordinates = value; }
    //! AUTO:  Read (Reference) access to:\f$\pv_{temp} \in \Rcal^{n\indf}\f$temporary vector containing coordinates
    const ResizableVector& GetTempCoordinates() const { return tempCoordinates; }
    //! AUTO:  Read (Reference) access to:\f$\pv_{temp} \in \Rcal^{n\indf}\f$temporary vector containing coordinates
    ResizableVector& GetTempCoordinates() { return tempCoordinates; }

    //! AUTO:  Write (Reference) access to:\f$\dot \pv_{temp} \in \Rcal^{n\indf}\f$temporary vector containing velocity coordinates
    void SetTempCoordinates_t(const ResizableVector& value) { tempCoordinates_t = value; }
    //! AUTO:  Read (Reference) access to:\f$\dot \pv_{temp} \in \Rcal^{n\indf}\f$temporary vector containing velocity coordinates
    const ResizableVector& GetTempCoordinates_t() const { return tempCoordinates_t; }
    //! AUTO:  Read (Reference) access to:\f$\dot \pv_{temp} \in \Rcal^{n\indf}\f$temporary vector containing velocity coordinates
    ResizableVector& GetTempCoordinates_t() { return tempCoordinates_t; }

    //! AUTO:  Write (Reference) access to:\f$(\tzeta \otimes \Im) \in \Rcal^{n\indf \times 3}\f$temporary coordinate dependent matrix
    void SetTempKronZetaI(const ResizableMatrix& value) { tempKronZetaI = value; }
    //! AUTO:  Read (Reference) access to:\f$(\tzeta \otimes \Im) \in \Rcal^{n\indf \times 3}\f$temporary coordinate dependent matrix
    const ResizableMatrix& GetTempKronZetaI() const { return tempKronZetaI; }
    //! AUTO:  Read (Reference) access to:\f$(\tzeta \otimes \Im) \in \Rcal^{n\indf \times 3}\f$temporary coordinate dependent matrix
    ResizableMatrix& GetTempKronZetaI() { return tempKronZetaI; }

    //! AUTO:  Write (Reference) access to:\f$(\tzeta \otimes \Im) \in \Rcal^{n\indf \times 3}\f$temporary coordinate dependent matrix
    void SetTempKronZetaI_t(const ResizableMatrix& value) { tempKronZetaI_t = value; }
    //! AUTO:  Read (Reference) access to:\f$(\tzeta \otimes \Im) \in \Rcal^{n\indf \times 3}\f$temporary coordinate dependent matrix
    const ResizableMatrix& GetTempKronZetaI_t() const { return tempKronZetaI_t; }
    //! AUTO:  Read (Reference) access to:\f$(\tzeta \otimes \Im) \in \Rcal^{n\indf \times 3}\f$temporary coordinate dependent matrix
    ResizableMatrix& GetTempKronZetaI_t() { return tempKronZetaI_t; }

    //! AUTO:  Write (Reference) access to:\f$(\Im_zeta \otimes \tomega)^T \in \Rcal^{n\indf \times 3 n\indf}\f$temporary coordinate dependent matrix
    void SetTempKronIZetaOmegaT(const ResizableMatrix& value) { tempKronIZetaOmegaT = value; }
    //! AUTO:  Read (Reference) access to:\f$(\Im_zeta \otimes \tomega)^T \in \Rcal^{n\indf \times 3 n\indf}\f$temporary coordinate dependent matrix
    const ResizableMatrix& GetTempKronIZetaOmegaT() const { return tempKronIZetaOmegaT; }
    //! AUTO:  Read (Reference) access to:\f$(\Im_zeta \otimes \tomega)^T \in \Rcal^{n\indf \times 3 n\indf}\f$temporary coordinate dependent matrix
    ResizableMatrix& GetTempKronIZetaOmegaT() { return tempKronIZetaOmegaT; }

    //! AUTO:  Write (Reference) access to:\f$\Xm_{temp}\f$temporary matrix at several parts of computation MassMatrix, ODE2Lhs
    void SetTempMatrix(const ResizableMatrix& value) { tempMatrix = value; }
    //! AUTO:  Read (Reference) access to:\f$\Xm_{temp}\f$temporary matrix at several parts of computation MassMatrix, ODE2Lhs
    const ResizableMatrix& GetTempMatrix() const { return tempMatrix; }
    //! AUTO:  Read (Reference) access to:\f$\Xm_{temp}\f$temporary matrix at several parts of computation MassMatrix, ODE2Lhs
    ResizableMatrix& GetTempMatrix() { return tempMatrix; }

    //! AUTO:  Write (Reference) access to:\f$\Xm_{temp2}\f$second temporary matrix at several parts of computation MassMatrix, ODE2Lhs
    void SetTempMatrix2(const ResizableMatrix& value) { tempMatrix2 = value; }
    //! AUTO:  Read (Reference) access to:\f$\Xm_{temp2}\f$second temporary matrix at several parts of computation MassMatrix, ODE2Lhs
    const ResizableMatrix& GetTempMatrix2() const { return tempMatrix2; }
    //! AUTO:  Read (Reference) access to:\f$\Xm_{temp2}\f$second temporary matrix at several parts of computation MassMatrix, ODE2Lhs
    ResizableMatrix& GetTempMatrix2() { return tempMatrix2; }

    //! AUTO:  Write (Reference) access to:\f$\vv_{temp}\f$temporary vector at computation of ODE2Lhs
    void SetTempVector(const ResizableVector& value) { tempVector = value; }
    //! AUTO:  Read (Reference) access to:\f$\vv_{temp}\f$temporary vector at computation of ODE2Lhs
    const ResizableVector& GetTempVector() const { return tempVector; }
    //! AUTO:  Read (Reference) access to:\f$\vv_{temp}\f$temporary vector at computation of ODE2Lhs
    ResizableVector& GetTempVector() { return tempVector; }

    //! AUTO:  Write (Reference) access to:\f$\vv_{temp2}\f$second temporary vector at computation of ODE2Lhs
    void SetTempVector2(const ResizableVector& value) { tempVector2 = value; }
    //! AUTO:  Read (Reference) access to:\f$\vv_{temp2}\f$second temporary vector at computation of ODE2Lhs
    const ResizableVector& GetTempVector2() const { return tempVector2; }
    //! AUTO:  Read (Reference) access to:\f$\vv_{temp2}\f$second temporary vector at computation of ODE2Lhs
    ResizableVector& GetTempVector2() { return tempVector2; }

    //! AUTO:  Computational function: compute mass matrix
    virtual void ComputeMassMatrix(Matrix& massMatrix) const override;

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs) const override;

    //! AUTO:  Compute algebraic equations part of rigid body
    virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, bool useIndex2 = false) const override;

    //! AUTO:  Compute jacobians of algebraic equations part of rigid body w.r.t. ODE2, ODE2_t, ODE1, AE
    virtual void ComputeJacobianAE(ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override;

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

    //! AUTO:  compute object coordinates composed from all nodal coordinates; does not include reference coordinates
    void ComputeObjectCoordinates(Vector& coordinates, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  compute object velocity coordinates composed from all nodal coordinates
    void ComputeObjectCoordinates_t(Vector& coordinates_t, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  initialize FFRFreducedOrder matrices
    void InitializeObject();

    //! AUTO:  compute coordinates for nodeNumber (without reference coordinates) from modeBasis (=multiplication of according part of mode Basis with modal coordinates)
    Vector3D GetMeshNodeCoordinates(Index nodeNumber, const Vector& coordinates) const;

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionForce(Vector& force, const MainSystemBase& mainSystem, Real t, const StdVector& coordinates, const StdVector& coordinates_t) const;

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionMassMatrix(Matrix& massMatrix, const MainSystemBase& mainSystem, Real t, const StdVector& coordinates, const StdVector& coordinates_t) const;

    //! AUTO:  always true, because FFRF-based object; return according LOCAL node number
    virtual bool HasReferenceFrame(Index& localReferenceFrameNode) const override
    {
        localReferenceFrameNode = rigidBodyNodeNumber; return true;
    }

    //! AUTO:  return the number of mesh nodes, which is given according to the node reference positions
    virtual Index GetNumberOfMeshNodes() const override
    {
        return parameters.referencePositions.NumberOfItems()/3;
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
            (Index)OutputVariableType::Force +
            (Index)OutputVariableType::StressLocal +
            (Index)OutputVariableType::StrainLocal );
    }

};



#endif //#ifdef include once...
