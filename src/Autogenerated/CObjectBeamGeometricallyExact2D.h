/** ***********************************************************************************************
* @class        CObjectBeamGeometricallyExact2DParameters
* @brief        Parameter class for CObjectBeamGeometricallyExact2D
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-01-09  01:47:29 (last modified)
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
    ArrayIndex nodeNumbers;                       //!< AUTO: two node numbers for beam element
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
    bool includeReferenceRotations;               //!< AUTO: if True, rotation of the cross section at the nodes includes node reference rotations (within referenceCoordinates of NodeRigidBody2D), which are used for the computation of bending strains (this means that a pre-curved beam is stress-free); if False, the reference rotation of the cross section is orthogonal to the reference slope vector. This allows to easily share nodes among several beams with different reference cross section orientation (i.e., only the change of rotation counts).
    //! AUTO: default constructor with parameter initialization
    CObjectBeamGeometricallyExact2DParameters()
    {
        nodeNumbers = ArrayIndex();
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
* @brief        A 2D geometrically exact beam finite element, using 2 or 3 nodes of type NodeRigidBody2D. Note that the orientation of the nodes need to follow the cross section orientation in case that includeReferenceRotations=True; e.g., an angle 0 represents the cross section aligned with the \f$y\f$-axis, while and angle \f$\pi/2\f$ means that the cross section points in negative \f$x\f$-direction. Pre-curvature can be included with physicsReferenceCurvature and axial pre-stress can be considered by using a physicsLength different from the reference configuration of the nodes. The localPosition of the beam with length \f$L\f$=physicsLength and height \f$h\f$ ranges in \f$X\f$-direction in range \f$[-L/2, L/2]\f$ and in \f$Y\f$-direction in range \f$[-h/2,h/2]\f$ (which is in fact not needed in the \hac{EOM}).
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
    static constexpr Index maxNNodes = 3; //!< max number of nodes
    static constexpr Index maxODE2coordinates = 9; //!< max size of coordinates used e.g. for ConstSizeVectors
    mutable bool massMatrixComputed; //!< flag which shows that mass matrix has been computed; will be set to false at time when parameters are set
    mutable ConstSizeMatrix<maxODE2coordinates*maxODE2coordinates> precomputedMassMatrix; //!< if massMatrixComputed=true, this contains the (constant) mass matrix for faster computation
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

    //! AUTO:  templated function to enable automatic differentiation
    template<class TReal> void ComputeODE2LHStemplate(VectorBase<TReal>& ode2Lhs, const ConstSizeVectorBase<TReal, maxODE2coordinates>& qBeamTotal, const ConstSizeVectorBase<TReal, maxODE2coordinates>& qBeam_t, const ConstSizeVectorBase<Real, maxODE2coordinates>& qBeamRef, Index objectNumber) const;

    //! AUTO:  Computational function: compute jacobian (dense or sparse mode, see parent CObject function)
    virtual void ComputeJacobianODE2_ODE2(EXUmath::MatrixContainer& jacobianODE2, JacobianTemp& temp, Real factorODE2, Real factorODE2_t, Index objectNumber, const ArrayIndex& ltg) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override;

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
        CHECKandTHROW(localIndex < parameters.nodeNumbers.NumberOfItems(), __EXUDYN_invalid_local_node0);
        return parameters.nodeNumbers[localIndex];
    }

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual void SetNodeNumber(Index localIndex, Index nodeNumber) override
    {
        parameters.nodeNumbers[localIndex]=nodeNumber;
    }

    //! AUTO:  number of nodes; needed for every object
    virtual Index GetNumberOfNodes() const override
    {
        return parameters.nodeNumbers.NumberOfItems();
    }

    //! AUTO:  number of \hac{ODE2} coordinates
    virtual Index GetODE2Size() const override
    {
        return parameters.nodeNumbers.NumberOfItems()*3;
    }

    //! AUTO:  Linear=2 node element, Quadratic (!Linear)=3 node element
    bool IsLinear() const
    {
        return parameters.nodeNumbers.NumberOfItems() == 2;
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

    //! AUTO:  compute object coordinates for configuration
    void ComputeCurrentCoordinates(ConstSizeVectorBase<Real, maxODE2coordinates>& qBeamTotal, ConstSizeVectorBase<Real, maxODE2coordinates>& qBeam_t, ConstSizeVectorBase<Real, maxODE2coordinates>& qBeamRef, ConfigurationType configuration) const;

    //! AUTO:  templated map of element coordinate vector to  [u0,u1,theta0]
    template<class TReal, Index nODE2> SlimVectorBase<TReal, 3> MapCoordinates(const ConstSizeVector<maxNNodes>& SV, const ConstSizeVectorBase<TReal, nODE2>& qBeam) const;

    //! AUTO:  map element coordinates (position or velocity level) given by nodal vectors q0 and q1 onto compressed shape function vector to compute position, etc.; if SV=SV(x), it returns Vector of coordinates at certain position x: [p0,p1,theta0]
    Vector3D MapCoordinatesLinear(const ConstSizeVector<maxNNodes>& SV, const LinkedDataVector& q0, const LinkedDataVector& q1) const;

    //! AUTO:  map element coordinates for 3-node element
    Vector3D MapCoordinatesQuadratic(const ConstSizeVector<maxNNodes>& SV, const LinkedDataVector& q0, const LinkedDataVector& q1, const LinkedDataVector& q2) const;

    //! AUTO:  get compressed shape function vector \f$\Sm_v\f$, depending local position \f$x \in [0,L]\f$
    ConstSizeVector<maxNNodes> ComputeShapeFunctions(Real x) const;

    //! AUTO:  get first derivative of compressed shape function vector \f$\frac{\partial \Sm_v}{\partial x}\f$, depending local position \f$x \in [0,L]\f$
    ConstSizeVector<maxNNodes> ComputeShapeFunctions_x(Real x) const;

    //! AUTO:  compute rotation matrix from angle theta
    Matrix2D GetRotationMatrix2D(Real theta) const;

    //! AUTO:  compute strains and variation of strains for given interpolated derivatives of displacement u1_x, u2_x, angle theta (incl. reference config.!), shape vector SV and shape vector derivatives SV_x and slope vector in reference configuration
    template<class TReal> void ComputeGeneralizedStrains(Real x, TReal& theta, const ConstSizeVectorBase<TReal, maxODE2coordinates>& qBeamTotal, const ConstSizeVectorBase<TReal, maxODE2coordinates>& qBeam_t, const ConstSizeVectorBase<Real, maxODE2coordinates>& qBeamRef, ConstSizeVectorBase<Real,maxNNodes>& SV, ConstSizeVectorBase<Real, maxNNodes>& SV_x, TReal& gamma1, TReal& gamma2, TReal& theta_x, TReal& gamma1_t, TReal& gamma2_t, TReal& theta_xt, ConstSizeVectorBase<TReal, maxODE2coordinates>& deltaGamma1, ConstSizeVectorBase<TReal, maxODE2coordinates>& deltaGamma2) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index64)OutputVariableType::Position +
            (Index64)OutputVariableType::Displacement +
            (Index64)OutputVariableType::Velocity +
            (Index64)OutputVariableType::Rotation +
            (Index64)OutputVariableType::StrainLocal +
            (Index64)OutputVariableType::CurvatureLocal +
            (Index64)OutputVariableType::ForceLocal +
            (Index64)OutputVariableType::TorqueLocal );
    }

};



#endif //#ifdef include once...
