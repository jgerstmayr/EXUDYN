/** ***********************************************************************************************
* @class        CObjectKinematicTreeParameters
* @brief        Parameter class for CObjectKinematicTree
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2022-07-04  11:58:23 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTKINEMATICTREEPARAMETERS__H
#define COBJECTKINEMATICTREEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Linalg/KinematicsBasics.h"//for transformations
#include "Pymodules/PyMatrixVector.h"//for some matrix and vector lists
#include "Main/OutputVariable.h"
#include <pybind11/numpy.h>//for NumpyMatrix
#include <pybind11/stl.h>//for NumpyMatrix
#include <pybind11/pybind11.h>
#include "Pymodules/PyMatrixVector.h"//for some matrix and vector lists
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectKinematicTreeParameters
class CObjectKinematicTreeParameters // AUTO: 
{
public: // AUTO: 
    Index nodeNumber;                             //!< AUTO: node number (type NodeIndex) of GenericODE2 node containing the coordinates for the kinematic tree; \f$n\f$ being the number of minimum coordinates
    Vector3D gravity;                             //!< AUTO: gravity vector in inertial coordinates; used to simply apply gravity as LoadMassProportional is not available for KinematicTree
    Vector3D baseOffset;                          //!< AUTO: offset vector for base, in global coordinates
    JointTypeList jointTypes;                     //!< AUTO: joint types of kinematic Tree joints; must be always set
    ArrayIndex linkParents;                       //!< AUTO: index of parent joint/link; if no parent exists, the value is \f$-1\f$; by default, \f$p_0=-1\f$ because the \f$i\f$th parent index must always fulfill \f$p_i<i\f$; must be always set
    Matrix3DList jointTransformations;            //!< AUTO: list of constant joint transformations from parent joint coordinates \f$p_0\f$ to this joint coordinates \f$j_0\f$; if no parent exists (\f$-1\f$), the base coordinate system \f$0\f$ is used; must be always set
    Vector3DList jointOffsets;                    //!< AUTO: list of constant joint offsets from parent joint to this joint; \f$p_0\f$, \f$p_1\f$, \f$\ldots\f$ denote the parent coordinate systems; if no parent exists (\f$-1\f$), the base coordinate system \f$0\f$ is used; must be always set
    Matrix3DList linkInertiasCOM;                 //!< AUTO: list of link inertia tensors w.r.t.\ \ac{COM} in joint/link \f$j_i\f$ coordinates; must be always set
    Vector3DList linkCOMs;                        //!< AUTO: list of vectors for center of mass (COM) in joint/link \f$j_i\f$ coordinates; must be always set
    Vector linkMasses;                            //!< AUTO: masses of links; must be always set
    Vector3DList linkForces;                      //!< AUTO: list of 3D force vectors per link in global coordinates acting on joint frame origin; use force-torque couple to realize off-origin forces; defaults to empty list \f$[]\f$, adding no forces
    Vector3DList linkTorques;                     //!< AUTO: list of 3D torque vectors per link in global coordinates; defaults to empty list \f$[]\f$, adding no torques
    Vector jointForceVector;                      //!< AUTO: generalized force vector per coordinate added to RHS of EOM; represents a torque around the axis of rotation in revolute joints and a force in prismatic joints; for a revolute joint \f$i\f$, the torque \f$f[i]\f$ acts positive (w.r.t.\ rotation axis) on link \f$i\f$ and negative on parent link \f$p_i\f$; must be either empty list/array \f$[]\f$ (default) or have size \f$n\f$
    Vector jointPositionOffsetVector;             //!< AUTO: offset for joint coordinates used in P(D) control; acts in positive joint direction similar to jointForceVector; should be modified, e.g., in preStepUserFunction; must be either empty list/array \f$[]\f$ (default) or have size \f$n\f$
    Vector jointVelocityOffsetVector;             //!< AUTO: velocity offset for joint coordinates used in (P)D control; acts in positive joint direction similar to jointForceVector; should be modified, e.g., in preStepUserFunction; must be either empty list/array \f$[]\f$ (default) or have size \f$n\f$
    Vector jointPControlVector;                   //!< AUTO: proportional (P) control values per joint (multiplied with position error between joint value and offset \f$\uv_o\f$); note that more complicated control laws must be implemented with user functions; must be either empty list/array \f$[]\f$ (default) or have size \f$n\f$
    Vector jointDControlVector;                   //!< AUTO: derivative (D) control values per joint (multiplied with velocity error between joint velocity and velocity offset \f$\vv_o\f$); note that more complicated control laws must be implemented with user functions; must be either empty list/array \f$[]\f$ (default) or have size \f$n\f$
    std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector)> forceUserFunction;//!< AUTO: A Python user function which computes the generalized force vector on RHS with identical action as jointForceVector; see description below
    //! AUTO: default constructor with parameter initialization
    CObjectKinematicTreeParameters()
    {
        nodeNumber = EXUstd::InvalidIndex;
        gravity = Vector3D({0.,0.,0.});
        baseOffset = Vector3D({0.,0.,0.});
        jointTypes = JointTypeList();
        linkParents = ArrayIndex();
        jointTransformations = Matrix3DList();
        jointOffsets = Vector3DList();
        linkInertiasCOM = Matrix3DList();
        linkCOMs = Vector3DList();
        linkMasses = Vector();
        linkForces = Vector3DList();
        linkTorques = Vector3DList();
        jointForceVector = Vector();
        jointPositionOffsetVector = Vector();
        jointVelocityOffsetVector = Vector();
        jointPControlVector = Vector();
        jointDControlVector = Vector();
        forceUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectKinematicTree
* @brief        A special object to represent open kinematic trees using minimum coordinate formulation (NOT FULLY TESTED!). The kinematic tree is defined by lists of joint types, parents, inertia parameters (w.r.t. COM), etc.\ per link (body) and given joint (pre) transformations from the previous joint. Every joint / link is defined by the position and orientation of the previous joint and a coordinate transformation (incl.\ translation) from the previous link's to this link's joint coordinates. The joint can be combined with a marker, which allows to attach connectors as well as joints to represent closed loop mechanisms. Efficient models can be created by using tree structures in combination with constraints and very long chains should be avoided and replaced by (smaller) jointed chains if possible. The class Robot from exudyn.robotics can also be used to create kinematic trees, which are then exported as KinematicTree or as redundant multibody system. Use specialized settings in VisualizationSettings.bodies.kinematicTree for showing joint frames and other properties.
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

//! AUTO: CObjectKinematicTree
class CObjectKinematicTree: public CObjectSuperElement // AUTO: 
{
protected: // AUTO: 
    CObjectKinematicTreeParameters parameters; //! AUTO: contains all parameters for CObjectKinematicTree
    mutable ResizableVector tempVector;           //!< AUTO: temporary vector during computation of mass and ODE2LHS
    mutable ResizableVector tempVector2;          //!< AUTO: second temporary vector during computation of mass and ODE2LHS
    mutable Transformations66List jointTransformationsTemp;//!< AUTO: temporary list containing transformations (Pluecker transforms) per joint
    mutable Transformations66List linkInertiasT66;//!< AUTO: temporary list link inertias as Pluecker transforms per link
    mutable Vector6DList motionSubspaces;         //!< AUTO: temporary list containing 6D motion subspaces per joint
    mutable Transformations66List jointTempT66;   //!< AUTO: temporary list containing 66 transformations per joint
    mutable Vector6DList jointVelocities;         //!< AUTO: temporary list containing 6D velocities per joint
    mutable Vector6DList jointAccelerations;      //!< AUTO: temporary list containing 6D accelerations per joint
    mutable Vector6DList jointForces;             //!< AUTO: temporary list containing 6D torques/forces per joint/link

public: // AUTO: 
    static constexpr Index noParent = -1;//AUTO: number which defines that this link has no parent
    //! AUTO: default constructor with parameter initialization
    CObjectKinematicTree()
    {
        tempVector = ResizableVector();
        tempVector2 = ResizableVector();
        jointTransformationsTemp = Transformations66List();
        linkInertiasT66 = Transformations66List();
        motionSubspaces = Vector6DList();
        jointTempT66 = Transformations66List();
        jointVelocities = Vector6DList();
        jointAccelerations = Vector6DList();
        jointForces = Vector6DList();
    };

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectKinematicTreeParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectKinematicTreeParameters& GetParameters() const { return parameters; }

    //! AUTO:  Write (Reference) access to:temporary vector during computation of mass and ODE2LHS
    void SetTempVector(const ResizableVector& value) { tempVector = value; }
    //! AUTO:  Read (Reference) access to:temporary vector during computation of mass and ODE2LHS
    const ResizableVector& GetTempVector() const { return tempVector; }
    //! AUTO:  Read (Reference) access to:temporary vector during computation of mass and ODE2LHS
    ResizableVector& GetTempVector() { return tempVector; }

    //! AUTO:  Write (Reference) access to:second temporary vector during computation of mass and ODE2LHS
    void SetTempVector2(const ResizableVector& value) { tempVector2 = value; }
    //! AUTO:  Read (Reference) access to:second temporary vector during computation of mass and ODE2LHS
    const ResizableVector& GetTempVector2() const { return tempVector2; }
    //! AUTO:  Read (Reference) access to:second temporary vector during computation of mass and ODE2LHS
    ResizableVector& GetTempVector2() { return tempVector2; }

    //! AUTO:  Write (Reference) access to:\f$\Xm \in \Rcal^{n \times (6 \times 6)}\f$temporary list containing transformations (Pluecker transforms) per joint
    void SetJointTransformationsTemp(const Transformations66List& value) { jointTransformationsTemp = value; }
    //! AUTO:  Read (Reference) access to:\f$\Xm \in \Rcal^{n \times (6 \times 6)}\f$temporary list containing transformations (Pluecker transforms) per joint
    const Transformations66List& GetJointTransformationsTemp() const { return jointTransformationsTemp; }
    //! AUTO:  Read (Reference) access to:\f$\Xm \in \Rcal^{n \times (6 \times 6)}\f$temporary list containing transformations (Pluecker transforms) per joint
    Transformations66List& GetJointTransformationsTemp() { return jointTransformationsTemp; }

    //! AUTO:  Write (Reference) access to:\f$\Jm_{66} \in \Rcal^{n \times (6 \times 6)}\f$temporary list link inertias as Pluecker transforms per link
    void SetLinkInertiasT66(const Transformations66List& value) { linkInertiasT66 = value; }
    //! AUTO:  Read (Reference) access to:\f$\Jm_{66} \in \Rcal^{n \times (6 \times 6)}\f$temporary list link inertias as Pluecker transforms per link
    const Transformations66List& GetLinkInertiasT66() const { return linkInertiasT66; }
    //! AUTO:  Read (Reference) access to:\f$\Jm_{66} \in \Rcal^{n \times (6 \times 6)}\f$temporary list link inertias as Pluecker transforms per link
    Transformations66List& GetLinkInertiasT66() { return linkInertiasT66; }

    //! AUTO:  Write (Reference) access to:\f$\Mm\Sm \in \Rcal^{n \times 6}\f$temporary list containing 6D motion subspaces per joint
    void SetMotionSubspaces(const Vector6DList& value) { motionSubspaces = value; }
    //! AUTO:  Read (Reference) access to:\f$\Mm\Sm \in \Rcal^{n \times 6}\f$temporary list containing 6D motion subspaces per joint
    const Vector6DList& GetMotionSubspaces() const { return motionSubspaces; }
    //! AUTO:  Read (Reference) access to:\f$\Mm\Sm \in \Rcal^{n \times 6}\f$temporary list containing 6D motion subspaces per joint
    Vector6DList& GetMotionSubspaces() { return motionSubspaces; }

    //! AUTO:  Write (Reference) access to:\f$\Xm_j \in \Rcal^{n \times 6}\f$temporary list containing 66 transformations per joint
    void SetJointTempT66(const Transformations66List& value) { jointTempT66 = value; }
    //! AUTO:  Read (Reference) access to:\f$\Xm_j \in \Rcal^{n \times 6}\f$temporary list containing 66 transformations per joint
    const Transformations66List& GetJointTempT66() const { return jointTempT66; }
    //! AUTO:  Read (Reference) access to:\f$\Xm_j \in \Rcal^{n \times 6}\f$temporary list containing 66 transformations per joint
    Transformations66List& GetJointTempT66() { return jointTempT66; }

    //! AUTO:  Write (Reference) access to:\f$\Vm_j \in \Rcal^{n \times 6}\f$temporary list containing 6D velocities per joint
    void SetJointVelocities(const Vector6DList& value) { jointVelocities = value; }
    //! AUTO:  Read (Reference) access to:\f$\Vm_j \in \Rcal^{n \times 6}\f$temporary list containing 6D velocities per joint
    const Vector6DList& GetJointVelocities() const { return jointVelocities; }
    //! AUTO:  Read (Reference) access to:\f$\Vm_j \in \Rcal^{n \times 6}\f$temporary list containing 6D velocities per joint
    Vector6DList& GetJointVelocities() { return jointVelocities; }

    //! AUTO:  Write (Reference) access to:\f$\Am_j \in \Rcal^{n \times 6}\f$temporary list containing 6D accelerations per joint
    void SetJointAccelerations(const Vector6DList& value) { jointAccelerations = value; }
    //! AUTO:  Read (Reference) access to:\f$\Am_j \in \Rcal^{n \times 6}\f$temporary list containing 6D accelerations per joint
    const Vector6DList& GetJointAccelerations() const { return jointAccelerations; }
    //! AUTO:  Read (Reference) access to:\f$\Am_j \in \Rcal^{n \times 6}\f$temporary list containing 6D accelerations per joint
    Vector6DList& GetJointAccelerations() { return jointAccelerations; }

    //! AUTO:  Write (Reference) access to:\f$\Fm_j \in \Rcal^{n \times 6}\f$temporary list containing 6D torques/forces per joint/link
    void SetJointForces(const Vector6DList& value) { jointForces = value; }
    //! AUTO:  Read (Reference) access to:\f$\Fm_j \in \Rcal^{n \times 6}\f$temporary list containing 6D torques/forces per joint/link
    const Vector6DList& GetJointForces() const { return jointForces; }
    //! AUTO:  Read (Reference) access to:\f$\Fm_j \in \Rcal^{n \times 6}\f$temporary list containing 6D torques/forces per joint/link
    Vector6DList& GetJointForces() { return jointForces; }

    //! AUTO:  return true, if object has a computation user function
    virtual bool HasUserFunction() const override
    {
        return (parameters.forceUserFunction!=0);
    }

    //! AUTO:  Computational function: compute mass matrix
    virtual void ComputeMassMatrix(EXUmath::MatrixContainer& massMatrixC, const ArrayIndex& ltg, Index objectNumber) const override;

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs, Index objectNumber) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override;

    //! AUTO:  Flags to determine, which access (forces, moments, connectors, ...) to object are possible
    virtual AccessFunctionType GetAccessFunctionTypes() const override;

    //! AUTO:  provide Jacobian at localPosition in 'value' according to object access
    virtual void GetAccessFunctionBody(AccessFunctionType accessType, const Vector3D& localPosition, Matrix& value) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariable(OutputVariableType variableType, Vector& value, ConfigurationType configuration, Index objectNumber) const override;

    //! AUTO:  return the (global) position of 'localPosition' according to configuration type
    virtual Vector3D GetPosition(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) position of 'localPosition' according to configuration type
    virtual Vector3D GetDisplacement(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) velocity of 'localPosition' according to configuration type
    virtual Vector3D GetVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the local position of the center of mass, used for massProportionalLoad, which may NOT be appropriate for GenericODE2
    virtual Vector3D GetLocalCenterOfMass() const override;

    //! AUTO:  return the (global) position of 'localPosition' of linkNumber according to configuration type
    Vector3D GetPositionKinematicTree(const Vector3D& localPosition, Index linkNumber, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  return the rotation matrix of of linkNumber according to configuration type
    Matrix3D GetRotationMatrixKinematicTree(Index linkNumber, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  return the (global) velocity of 'localPosition' and linkNumber according to configuration type
    Vector3D GetVelocityKinematicTree(const Vector3D& localPosition, Index linkNumber, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  return the (global) angular velocity of linkNumber according to configuration type
    Vector3D GetAngularVelocityKinematicTree(Index linkNumber, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  return the (local) angular velocity of linkNumber according to configuration type
    Vector3D GetAngularVelocityLocalKinematicTree(Index linkNumber, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  return the (global) acceleration of 'localPosition' and linkNumber according to configuration type
    Vector3D GetAccelerationKinematicTree(const Vector3D& localPosition, Index linkNumber, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  return the (global) angular acceleration of linkNumber according to configuration type
    Vector3D GetAngularAccelerationKinematicTree(Index linkNumber, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        return parameters.nodeNumber;
    }

    //! AUTO:  number of nodes; needed for every object
    virtual Index GetNumberOfNodes() const override
    {
        return 1;
    }

    //! AUTO:  number of \hac{ODE2} coordinates
    virtual Index GetODE2Size() const override
    {
        return parameters.jointTransformations.NumberOfItems();
    }

    //! AUTO:  Get type of object, e.g. to categorize and distinguish during assembly and computation
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded + (Index)CObjectType::SuperElement + (Index)CObjectType::KinematicTree);
    }

    //! AUTO:  return true if object has time and coordinate independent (=constant) mass matrix
    virtual bool HasConstantMassMatrix() const override
    {
        return false;
    }

    //! AUTO:  This flag is reset upon change of parameters; says that the vector of coordinate indices has changed
    virtual void ParametersHaveChanged() override
    {
        ;
    }

    //! AUTO:  number of links used in computation functions for kinematic tree
    Index NumberOfLinks() const
    {
        return parameters.jointTransformations.NumberOfItems();
    }

    //! AUTO:  call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places
    void EvaluateUserFunctionForce(Vector& force, const MainSystemBase& mainSystem, Real t, Index objectNumber, const StdVector& coordinates, const StdVector& coordinates_t) const;

    //! AUTO:  compute negative 6D gravity to be used in Pluecker transforms
    void GetNegativeGravity6D(Vector6D& gravity6D) const;

    //! AUTO:  compute joint transformation T and motion subspace MS for jointType and joint value q
    void JointTransformMotionSubspace66(Joint::Type jointType, Real q, Transformation66& T, Vector6D& MS) const;

    //! AUTO:  compute list of Pluecker transformations Xup, 6D velocities and 6D acceleration terms (not joint accelerations) per joint
    void ComputeTreeTransformations(ConfigurationType configuration, bool computeVelocitiesAccelerations, bool computeAbsoluteTransformations, Transformations66List& Xup, Vector6DList& V, Vector6DList& A) const;

    //! AUTO:  compute mass matrix if computeMass = true and compute ODE2LHS vector if computeMass=false
    void ComputeMassMatrixAndODE2LHS(EXUmath::MatrixContainer* massMatrixC, const ArrayIndex* ltg, Vector* ode2Lhs, Index objectNumber, bool computeMass) const;

    //! AUTO:  function which adds 3D torques/forces per joint to Fvp
    void AddExternalForces6D(const Transformations66List& Xup, Vector6DList& Fvp) const;

    //! AUTO:  return true, if object has reference frame; return according LOCAL node number
    virtual bool HasReferenceFrame(Index& localReferenceFrameNode) const override
    {
        localReferenceFrameNode = 0; return false;
    }

    //! AUTO:  return the number of mesh nodes; these are virtual nodes per link, emulating rigid bodies recomputed from kinematic tree
    virtual Index GetNumberOfMeshNodes() const override
    {
        return parameters.jointTransformations.NumberOfItems();
    }

    //! AUTO:  return the (global) position of a mesh node according to configuration type; this is the node position transformed by the motion of the reference frame; meshNodeNumber is the local node number of the (underlying) mesh
    virtual Vector3D GetMeshNodePosition(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) velocity of a mesh node according to configuration type; this is the node position transformed by the motion of the reference frame; meshNodeNumber is the local node number of the (underlying) mesh
    virtual Vector3D GetMeshNodeVelocity(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  compute Jacobian with weightingMatrix (WM) and/or meshNodeNumbers, which define how the SuperElement mesh nodes or coordinates are transformed to a global position; for details see CObjectSuperElement header file
    virtual void GetAccessFunctionSuperElement(AccessFunctionType accessType, const Matrix& weightingMatrix, const ArrayIndex& meshNodeNumbers, Matrix& value) const override;

    //! AUTO:  get extended output variable types for multi-nodal objects with mesh nodes; some objects have meshNode-dependent OutputVariableTypes
    virtual OutputVariableType GetOutputVariableTypesSuperElement(Index meshNodeNumber) const override;

    //! AUTO:  get extended output variables for multi-nodal objects with mesh nodes
    void GetOutputVariableKinematicTree(OutputVariableType variableType, const Vector3D& localPosition, Index linkNumber, ConfigurationType configuration, Vector& value) const;

    //! AUTO:  accelerator function for faster computation of MarkerData for rigid bodies/joints
    void ComputeRigidBodyMarkerDataKT(const Vector3D& localPosition, Index linkNumber, bool computeJacobian, MarkerData& markerData) const;

    //! AUTO:  compute rot+pos jacobian of (global) position at linkNumber, using pre-computed joint transformations
    void ComputeJacobian(Index linkNumber, const Vector3D& position, const Transformations66List& jointTransformations, ResizableMatrix& positionJacobian, ResizableMatrix& rotationJacobian) const;

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
