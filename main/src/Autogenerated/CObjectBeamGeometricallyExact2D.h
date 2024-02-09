/** ***********************************************************************************************
* @class        CObjectBeamGeometricallyExact2DParameters
* @brief        Parameter class for CObjectBeamGeometricallyExact2D
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2024-01-28  18:16:35 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef COBJECTBEAMGEOMETRICALLYEXACT2DPARAMETERS__H
#define COBJECTBEAMGEOMETRICALLYEXACT2DPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"


//! AUTO: Parameters for class CObjectBeamGeometricallyExact2DParameters
class CObjectBeamGeometricallyExact2DParameters // AUTO: 
{
public: // AUTO: 
    Index2 nodeNumbers;                           //!< AUTO: two node numbers for beam element
    Real physicsLength;                           //!< AUTO:  [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives \f$\rho A L\f$; must be positive
    Real physicsMassPerLength;                    //!< AUTO:  [SI:kg/m] mass per length of beam
    Real physicsCrossSectionInertia;              //!< AUTO:  [SI:kg m] cross section mass moment of inertia; inertia acting against rotation of cross section
    Real physicsBendingStiffness;                 //!< AUTO:  [SI:Nm\f$^2\f$] bending stiffness of beam; the bending moment is \f$m = EI (\kappa - \kappa_0)\f$, in which \f$\kappa\f$ is the material measure of curvature
    Real physicsAxialStiffness;                   //!< AUTO:  [SI:N] axial stiffness of beam; the axial force is \f$f_{ax} = EA (\varepsilon -\varepsilon_0)\f$, in which \f$\varepsilon\f$ is the axial strain
    Real physicsShearStiffness;                   //!< AUTO:  [SI:N] effective shear stiffness of beam, including stiffness correction
    Real physicsBendingDamping;                   //!< AUTO:  [SI:Nm\f$^2\f$/s] viscous damping of bending deformation; the additional virtual work due to damping is \f$\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx\f$
    Real physicsAxialDamping;                     //!< AUTO:  [SI:N/s] viscous damping of axial deformation
    Real physicsShearDamping;                     //!< AUTO:  [SI:N/s] viscous damping of shear deformation
    Real physicsReferenceCurvature;               //!< AUTO:  [SI:1/m] reference curvature of beam (pre-deformation) of beam
    bool includeReferenceRotations;               //!< AUTO: if True, rotations at nodes consider reference rotations, which are used for the computation of bending strains (this means that a pre-curved beam is stress-free); if False, the reference rotation of the cross section is orthogonal to the direction between the reference position of the end nodes. This allows to easily share nodes among several beams with different cross section orientation.
    //! AUTO: default constructor with parameter initialization
    CObjectBeamGeometricallyExact2DParameters()
    {
        nodeNumbers = Index2({EXUstd::InvalidIndex, EXUstd::InvalidIndex});
        physicsLength = 0.;
        physicsMassPerLength = 0.;
        physicsCrossSectionInertia = 0.;
        physicsBendingStiffness = 0.;
        physicsAxialStiffness = 0.;
        physicsShearStiffness = 0.;
        physicsBendingDamping = 0.;
        physicsAxialDamping = 0.;
        physicsShearDamping = 0.;
        physicsReferenceCurvature = 0.;
        includeReferenceRotations = false;
    };
};


/** ***********************************************************************************************
* @class        CObjectBeamGeometricallyExact2D
* @brief        A 2D geometrically exact beam finite element, currently using 2 nodes of type NodeRigidBody2D; FURTHER TESTS REQUIRED. Note that the orientation of the nodes need to follow the cross section orientation in case that includeReferenceRotations=True; e.g., an angle 0 represents the cross section aligned with the \f$y\f$-axis, while and angle \f$\pi/2\f$ means that the cross section points in negative \f$x\f$-direction. Pre-curvature can be included with physicsReferenceCurvature and axial pre-stress can be considered by using a physicsLength different from the reference configuration of the nodes. The localPosition of the beam with length \f$L\f$=physicsLength and height \f$h\f$ ranges in \f$X\f$-direction in range \f$[-L/2, L/2]\f$ and in \f$Y\f$-direction in range \f$[-h/2,h/2]\f$ (which is in fact not needed in the \hac{EOM}).
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

//! AUTO: CObjectBeamGeometricallyExact2D
class CObjectBeamGeometricallyExact2D: public CObjectBody // AUTO: 
{
protected: // AUTO: 
    static constexpr Index nODE2coordinates = 6; //!< fixed size of coordinates used e.g. for ConstSizeVectors
    mutable bool massMatrixComputed; //!< flag which shows that mass matrix has been computed; will be set to false at time when parameters are set
    mutable ConstSizeMatrix<nODE2coordinates*nODE2coordinates> precomputedMassMatrix; //!< if massMatrixComputed=true, this contains the (constant) mass matrix for faster computation
    CObjectBeamGeometricallyExact2DParameters parameters; //! AUTO: contains all parameters for CObjectBeamGeometricallyExact2D

public: // AUTO: 

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectBeamGeometricallyExact2DParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectBeamGeometricallyExact2DParameters& GetParameters() const { return parameters; }

    //! AUTO:  Computational function: compute mass matrix
    virtual void ComputeMassMatrix(EXUmath::MatrixContainer& massMatrixC, const ArrayIndex& ltg, Index objectNumber, bool computeInverse=false) const override;

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs, Index objectNumber) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override
    {
        return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);
    }

    //! AUTO:  Flags to determine, which access (forces, moments, connectors, ...) to object are possible
    virtual AccessFunctionType GetAccessFunctionTypes() const override;

    //! AUTO:  provide Jacobian at localPosition in 'value' according to object access
    virtual void GetAccessFunctionBody(AccessFunctionType accessType, const Vector3D& localPosition, Matrix& value) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableBody(OutputVariableType variableType, const Vector3D& localPosition, ConfigurationType configuration, Vector& value, Index objectNumber) const override;

    //! AUTO:  return the (global) position of 'localPosition' according to configuration type
    virtual Vector3D GetPosition(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) displacement of 'localPosition' according to configuration type
    virtual Vector3D GetDisplacement(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the (global) velocity of 'localPosition' according to configuration type
    virtual Vector3D GetVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent rotation matrix of beam; returns always a 3D Matrix, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
    virtual Matrix3D GetRotationMatrix(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return configuration dependent rotation of beam (Tait-Bryan angles); returns 3D Vector with z-component
    Real GetRotation(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  return configuration dependent angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect
    virtual Vector3D GetAngularVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const override;

    //! AUTO:  return the local position of the center of mass, needed for equations of motion and for massProportionalLoad
    virtual Vector3D GetLocalCenterOfMass() const override
    {
        return Vector3D({0.,0.,0.});
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override
    {
        CHECKandTHROW(localIndex <= 1, __EXUDYN_invalid_local_node1);
        return parameters.nodeNumbers[localIndex];
    }

    //! AUTO:  number of nodes; needed for every object
    virtual Index GetNumberOfNodes() const override
    {
        return 2;
    }

    //! AUTO:  number of \hac{ODE2} coordinates; needed for object?
    virtual Index GetODE2Size() const override
    {
        return nODE2coordinates;
    }

    //! AUTO:  Get type of object, e.g. to categorize and distinguish during assembly and computation
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded);
    }

    //! AUTO:  return true if object has time and coordinate independent (=constant) mass matrix
    virtual bool HasConstantMassMatrix() const override
    {
        return true;
    }

    //! AUTO:  This flag is reset upon change of parameters; says that mass matrix (future: other pre-computed values) need to be recomputed
    virtual void ParametersHaveChanged() override
    {
        massMatrixComputed = false;
    }

    //! AUTO:  map element coordinates (position or velocity level) given by nodal vectors q0 and q1 onto compressed shape function vector to compute position, etc.; if SV=SV(x), it returns Vector of coordinates at certain position x: [p0,p1,theta0]
    Vector3D MapCoordinates(const Vector2D& SV, const LinkedDataVector& q0, const LinkedDataVector& q1) const;

    //! AUTO:  get compressed shape function vector \f$\Sm_v\f$, depending local position \f$x \in [0,L]\f$
    Vector2D ComputeShapeFunctions(Real x) const;

    //! AUTO:  get first derivative of compressed shape function vector \f$\frac{\partial \Sm_v}{\partial x}\f$, depending local position \f$x \in [0,L]\f$
    Vector2D ComputeShapeFunctions_x(Real x) const;

    //! AUTO:  compute rotation matrix from angle theta
    Matrix2D GetRotationMatrix2D(Real theta) const;

    //! AUTO:  compute strains and variation of strains for given interpolated derivatives of displacement u1_x, u2_x, angle theta (incl. reference config.!), shape vector SV and shape vector derivatives SV_x and slope vector in reference configuration
    void ComputeGeneralizedStrains(Real x, Real& theta, Vector2D& SV, Vector2D& SV_x, Real& gamma1, Real& gamma2, Real& theta_x, Real& gamma1_t, Real& gamma2_t, Real& theta_xt, CSVector6D& deltaGamma1, CSVector6D& deltaGamma2) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index)OutputVariableType::Position +
            (Index)OutputVariableType::Displacement +
            (Index)OutputVariableType::Velocity +
            (Index)OutputVariableType::Rotation +
            (Index)OutputVariableType::StrainLocal +
            (Index)OutputVariableType::CurvatureLocal +
            (Index)OutputVariableType::ForceLocal +
            (Index)OutputVariableType::TorqueLocal );
    }

};



#endif //#ifdef include once...
