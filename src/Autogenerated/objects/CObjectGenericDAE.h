/** ***********************************************************************************************
* @class        CObjectGenericDAEParameters
* @brief        Parameter class for CObjectGenericDAE
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2026-11-10  21:43:04 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#ifndef COBJECTGENERICDAEPARAMETERS__H
#define COBJECTGENERICDAEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <functional> //! AUTO: needed for std::function
#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11
namespace py = pybind11;            //! AUTO: "py" used throughout in code
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h

//! AUTO: Parameters for class CObjectGenericDAEParameters
class CObjectGenericDAEParameters // AUTO:
{
public: // AUTO:
    ArrayIndex nodeNumbersODE2;                   //!< AUTO: the nodes of the ABRV:ODE2 coordinates \f$\qv\f$, any nodes with ABRV:ODE2 coordinates
    ArrayIndex nodeNumbersODE1;                   //!< AUTO: the NodeGenericODE1 of the ABRV:ODE1 coordinates \f$\yv\f$
    ArrayIndex nodeNumbersAE;                     //!< AUTO: the NodeGenericAE of the algebraic coordinates \f$\tlambda\f$; their number of coordinates is the number of algebraic equations
    ArrayIndex nodeNumbersData;                   //!< AUTO: the NodeGenericData of the data coordinates \f$\xv\f$, which the user functions get and the postNewtonStepUserFunction updates (#3195)
    Matrix massMatrix;                            //!< AUTO: the mass matrix; empty: zero, for an object that constrains the coordinates of other items (#3193)
    Matrix dampingMatrix;                         //!< AUTO: the damping matrix
    Matrix stiffnessMatrix;                       //!< AUTO: the stiffness matrix
    Vector forceVector;                           //!< AUTO: the constant generalized forces
    Matrix couplingMatrixODE1;                    //!< AUTO: the term of the ABRV:ODE1 coordinates in the ABRV:ODE2 equations
    Matrix systemMatrix;                          //!< AUTO: the system matrix of the ABRV:ODE1 equations
    Vector rhsVector;                             //!< AUTO: the constant right-hand side of the ABRV:ODE1 equations
    Matrix couplingMatrixODE2;                    //!< AUTO: the term of the ABRV:ODE2 coordinates in the ABRV:ODE1 equations
    Matrix couplingMatrixODE2_t;                  //!< AUTO: the term of the ABRV:ODE2 velocities in the ABRV:ODE1 equations
    Matrix constraintMatrixODE2;                  //!< AUTO: the ABRV:ODE2 coordinates in the algebraic equations
    Matrix constraintMatrixODE1;                  //!< AUTO: the ABRV:ODE1 coordinates in the algebraic equations
    Vector constraintVector;                      //!< AUTO: the constant term of the algebraic equations
    Vector parameters;                            //!< AUTO: the parameters of the object, passed to every user function; see exudyn.advancedUtilities.ParameterLayout; empty by default
    PythonUserFunctionBase< std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > forceUserFunction;//!< AUTO: A Python user function which computes the nonlinear generalized forces of the ABRV:ODE2 equations
    PythonUserFunctionBase< std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > rhsUserFunction;//!< AUTO: A Python user function which computes the nonlinear term of the right-hand side of the ABRV:ODE1 equations
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > massMatrixUserFunction;//!< AUTO: A Python user function which computes the mass matrix instead of the constant massMatrix (#3191)
    PythonUserFunctionBase< std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&,bool,const StdVector&)> > constraintUserFunction;//!< AUTO: A Python user function which computes the nonlinear term of the algebraic equations (#3191)
    PythonUserFunctionBase< std::function<py::object(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&,Real,Real,Real,const StdVector&)> > jacobianUserFunction;//!< AUTO: A Python user function which computes the local Jacobian of the terms of all user functions, see the Jacobians below (#3191)
    PythonUserFunctionBase< std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> > postNewtonStepUserFunction;//!< AUTO: A Python user function which returns the error of the switch, a recommended step size and the new data coordinates after each Newton iteration; needs nodeNumbersData (#3195)
    //! AUTO: default constructor with parameter initialization
    CObjectGenericDAEParameters()
    {
        nodeNumbersODE2 = ArrayIndex();
        nodeNumbersODE1 = ArrayIndex();
        nodeNumbersAE = ArrayIndex();
        nodeNumbersData = ArrayIndex();
        massMatrix = Matrix();
        dampingMatrix = Matrix();
        stiffnessMatrix = Matrix();
        forceVector = Vector();
        couplingMatrixODE1 = Matrix();
        systemMatrix = Matrix();
        rhsVector = Vector();
        couplingMatrixODE2 = Matrix();
        couplingMatrixODE2_t = Matrix();
        constraintMatrixODE2 = Matrix();
        constraintMatrixODE1 = Matrix();
        constraintVector = Vector();
        parameters = Vector();
        forceUserFunction = 0;
        rhsUserFunction = 0;
        massMatrixUserFunction = 0;
        constraintUserFunction = 0;
        jacobianUserFunction = 0;
        postNewtonStepUserFunction = 0;
    };
};


/** ***********************************************************************************************
* @class        CObjectGenericDAE
* @brief        A system of ABRV:ODE2, ABRV:ODE1 and algebraic equations on the coordinates of its nodes: ABRV:ODE2 nodes, NodeGenericODE1 and NodeGenericAE, whose coordinates are the Lagrange multipliers \f$\tlambda\f$ of its algebraic equations. The linear terms are given by matrices, the nonlinear ones by user functions. NOTE that every matrix or vector must have the dimensions of the coordinates it multiplies, or be empty, using [] in Python.
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

//! AUTO: CObjectGenericDAE
class CObjectGenericDAE: public CObjectBody // AUTO:
{
protected: // AUTO:
    CObjectGenericDAEParameters parameters; //! AUTO: contains all parameters for CObjectGenericDAE
    mutable Vector tempCoordinates;               //!< AUTO: the ABRV:ODE2 coordinates, kept between the calls
    mutable Vector tempCoordinates_t;             //!< AUTO: the ABRV:ODE2 velocities, kept between the calls
    mutable Vector tempCoordinatesODE1;           //!< AUTO: the ABRV:ODE1 coordinates, kept between the calls
    mutable Vector tempCoordinatesAE;             //!< AUTO: the algebraic coordinates, kept between the calls
    mutable Vector tempCoordinatesData;           //!< AUTO: the data coordinates, kept between the calls
    mutable Matrix tempJacobian;                  //!< AUTO: the local Jacobian of the jacobianUserFunction, kept between the calls
    mutable ResizableVector tempUserFunctionResult;//!< AUTO: the result of a user function, kept between the calls

public: // AUTO:
    //! AUTO: default constructor with parameter initialization
    CObjectGenericDAE()
    {
        tempCoordinates = Vector();
        tempCoordinates_t = Vector();
        tempCoordinatesODE1 = Vector();
        tempCoordinatesAE = Vector();
        tempCoordinatesData = Vector();
        tempJacobian = Matrix();
        tempUserFunctionResult = ResizableVector();
    };

    // AUTO: access functions
    //! AUTO: Write (Reference) access to parameters
    virtual CObjectGenericDAEParameters& GetParameters() { return parameters; }
    //! AUTO: Read access to parameters
    virtual const CObjectGenericDAEParameters& GetParameters() const { return parameters; }

    //! AUTO: the user functions of the item, for the parallel computation (#2978)
    virtual void GetUserFunctions(ResizableArray<const UserFunctionBase*>& list) const override
    {
        list.Append(&parameters.forceUserFunction);
        list.Append(&parameters.rhsUserFunction);
        list.Append(&parameters.massMatrixUserFunction);
        list.Append(&parameters.constraintUserFunction);
        list.Append(&parameters.jacobianUserFunction);
        list.Append(&parameters.postNewtonStepUserFunction);
    }

    //! AUTO:  Write (Reference) access to:\f$\qv_{temp}\f$the ABRV:ODE2 coordinates, kept between the calls
    void SetTempCoordinates(const Vector& value) { tempCoordinates = value; }
    //! AUTO:  Read (Reference) access to:\f$\qv_{temp}\f$the ABRV:ODE2 coordinates, kept between the calls
    const Vector& GetTempCoordinates() const { return tempCoordinates; }
    //! AUTO:  Read (Reference) access to:\f$\qv_{temp}\f$the ABRV:ODE2 coordinates, kept between the calls
    Vector& GetTempCoordinates() { return tempCoordinates; }

    //! AUTO:  Write (Reference) access to:\f$\dot\qv_{temp}\f$the ABRV:ODE2 velocities, kept between the calls
    void SetTempCoordinates_t(const Vector& value) { tempCoordinates_t = value; }
    //! AUTO:  Read (Reference) access to:\f$\dot\qv_{temp}\f$the ABRV:ODE2 velocities, kept between the calls
    const Vector& GetTempCoordinates_t() const { return tempCoordinates_t; }
    //! AUTO:  Read (Reference) access to:\f$\dot\qv_{temp}\f$the ABRV:ODE2 velocities, kept between the calls
    Vector& GetTempCoordinates_t() { return tempCoordinates_t; }

    //! AUTO:  Write (Reference) access to:\f$\yv_{temp}\f$the ABRV:ODE1 coordinates, kept between the calls
    void SetTempCoordinatesODE1(const Vector& value) { tempCoordinatesODE1 = value; }
    //! AUTO:  Read (Reference) access to:\f$\yv_{temp}\f$the ABRV:ODE1 coordinates, kept between the calls
    const Vector& GetTempCoordinatesODE1() const { return tempCoordinatesODE1; }
    //! AUTO:  Read (Reference) access to:\f$\yv_{temp}\f$the ABRV:ODE1 coordinates, kept between the calls
    Vector& GetTempCoordinatesODE1() { return tempCoordinatesODE1; }

    //! AUTO:  Write (Reference) access to:\f$\tlambda_{temp}\f$the algebraic coordinates, kept between the calls
    void SetTempCoordinatesAE(const Vector& value) { tempCoordinatesAE = value; }
    //! AUTO:  Read (Reference) access to:\f$\tlambda_{temp}\f$the algebraic coordinates, kept between the calls
    const Vector& GetTempCoordinatesAE() const { return tempCoordinatesAE; }
    //! AUTO:  Read (Reference) access to:\f$\tlambda_{temp}\f$the algebraic coordinates, kept between the calls
    Vector& GetTempCoordinatesAE() { return tempCoordinatesAE; }

    //! AUTO:  Write (Reference) access to:\f$\xv_{temp}\f$the data coordinates, kept between the calls
    void SetTempCoordinatesData(const Vector& value) { tempCoordinatesData = value; }
    //! AUTO:  Read (Reference) access to:\f$\xv_{temp}\f$the data coordinates, kept between the calls
    const Vector& GetTempCoordinatesData() const { return tempCoordinatesData; }
    //! AUTO:  Read (Reference) access to:\f$\xv_{temp}\f$the data coordinates, kept between the calls
    Vector& GetTempCoordinatesData() { return tempCoordinatesData; }

    //! AUTO:  Write (Reference) access to:the local Jacobian of the jacobianUserFunction, kept between the calls
    void SetTempJacobian(const Matrix& value) { tempJacobian = value; }
    //! AUTO:  Read (Reference) access to:the local Jacobian of the jacobianUserFunction, kept between the calls
    const Matrix& GetTempJacobian() const { return tempJacobian; }
    //! AUTO:  Read (Reference) access to:the local Jacobian of the jacobianUserFunction, kept between the calls
    Matrix& GetTempJacobian() { return tempJacobian; }

    //! AUTO:  Write (Reference) access to:the result of a user function, kept between the calls
    void SetTempUserFunctionResult(const ResizableVector& value) { tempUserFunctionResult = value; }
    //! AUTO:  Read (Reference) access to:the result of a user function, kept between the calls
    const ResizableVector& GetTempUserFunctionResult() const { return tempUserFunctionResult; }
    //! AUTO:  Read (Reference) access to:the result of a user function, kept between the calls
    ResizableVector& GetTempUserFunctionResult() { return tempUserFunctionResult; }

    //! AUTO:  true if a Python user function is given; the object is then computed in the main thread
    virtual bool HasUserFunction() const override
    {
        return (parameters.forceUserFunction!=0) || (parameters.rhsUserFunction!=0) || (parameters.massMatrixUserFunction!=0) || (parameters.constraintUserFunction!=0) || (parameters.jacobianUserFunction!=0) || (parameters.postNewtonStepUserFunction!=0);
    }

    //! AUTO:  Computational function: compute mass matrix
    virtual void ComputeMassMatrix(EXUmath::MatrixContainer& massMatrixC, const ArrayIndex& ltg, bool computeInverse=false) const override;

    //! AUTO:  Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'
    virtual void ComputeODE2LHS(Vector& ode2Lhs) const override;

    //! AUTO:  Computational function: compute jacobian (dense or sparse mode, see parent CObject function)
    virtual void ComputeJacobianODE2_ODE2(EXUmath::MatrixContainer& jacobianODE2, JacobianTemp& temp, Real factorODE2, Real factorODE2_t, const ArrayIndex& ltg) const override;

    //! AUTO:  the Jacobian of the ABRV:ODE2 ABRV:LHS by the ABRV:ODE1 coordinates, the coupling matrix (#3143)
    virtual void ComputeJacobianODE2_ODE1(const CSystemData& systemData, TemporaryComputationData& temp, ResizableMatrix& jacobian) const override;

    //! AUTO:  the right-hand side of the ABRV:ODE1 equations
    virtual void ComputeODE1RHS(Vector& ode1Rhs) const override;

    //! AUTO:  Computational function: the analytic Jacobian of the ABRV:ODE1 ABRV:RHS by the local ABRV:ODE1 coordinates, and by the ABRV:ODE2 coordinates and velocities if the object depends on them (#3143)
    virtual void ComputeJacobianODE1RHS(ResizableMatrix& jacobianODE1, ResizableMatrix& jacobianODE2, ResizableMatrix& jacobianODE2_t) const override;

    //! AUTO:  Compute algebraic equations part of rigid body
    virtual void ComputeAlgebraicEquations(Vector& algebraicEquations, bool useIndex2 = false) const override;

    //! AUTO:  Compute jacobians of algebraic equations part of rigid body w.r.t. ABRV:ODE2, ABRV:ODE2 time derivatives, ABRV:ODE1, ABRV:AE
    virtual void ComputeJacobianAE(ResizableMatrix& jacobian_ODE2, ResizableMatrix& jacobian_ODE2_t, ResizableMatrix& jacobian_ODE1, ResizableMatrix& jacobian_AE) const override;

    //! AUTO:  return the available jacobian dependencies and the jacobians which are available as a function; if jacobian dependencies exist but are not available as a function, it is computed numerically; can be combined with 2^i enum flags
    virtual JacobianType::Type GetAvailableJacobians() const override;

    //! AUTO:  Flags to determine, which access (forces, moments, connectors, ...) to object are possible
    virtual AccessFunctionType GetAccessFunctionTypes() const override
    {
        return AccessFunctionType::_None;
    }

    //! AUTO:  provide Jacobian at localPosition in 'value' according to object access
    virtual void GetAccessFunction(AccessFunctionType accessType, Matrix& value) const override;

    //! AUTO:  provide according output variable in 'value'
    virtual void GetOutputVariableBody(OutputVariableType variableType, const Vector3D& localPosition, ConfigurationType configuration, Vector& value) const override;

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual Index GetNodeNumber(Index localIndex) const override;

    //! AUTO:  Get global node number (with local node index); needed for every object ==> does local mapping
    virtual void SetNodeNumber(Index localIndex, Index nodeNumber) override;

    //! AUTO:  number of nodes; needed for every object; can depend on the configuration
    virtual Index GetNumberOfNodes() const override
    {
        return parameters.nodeNumbersODE2.NumberOfItems() + parameters.nodeNumbersODE1.NumberOfItems() + parameters.nodeNumbersAE.NumberOfItems() + parameters.nodeNumbersData.NumberOfItems();
    }

    //! AUTO:  number of ABRV:ODE2 coordinates; needed for object?
    virtual Index GetODE2Size() const override;

    //! AUTO:  number of ABRV:ODE1 coordinates; needed for object?
    virtual Index GetODE1Size() const override;

    //! AUTO:  number of algebraic equations; independent of node/body coordinates
    virtual Index GetAlgebraicEquationsSize() const override;

    //! AUTO:  a body of several nodes, so that its mass matrix, its ABRV:ODE2 equations and its algebraic equations are assembled
    virtual CObjectType GetType() const override
    {
        return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded);
    }

    //! AUTO:  return true if object has time and coordinate independent (=constant) mass matrix
    virtual bool HasConstantMassMatrix() const override
    {
        return !parameters.massMatrixUserFunction;
    }

    //! AUTO:  the ABRV:ODE2 coordinates and velocities and the ABRV:ODE1 coordinates of the nodes, without reference values
    void ComputeObjectCoordinates(Vector& q, Vector& q_t, Vector& y, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  call of the force or the rhs user function with the coordinates of tempCoordinates, the size checked; implemented in evaluateUserFunctions.cpp
    void EvaluateUserFunction(const PythonUserFunctionBase< std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> >& userFunction, const char* name, Index size, Vector& result) const;

    //! AUTO:  factor times the derivative of the recorded force or rhs user function by its argument of index argument (2: q, 3: q_t, 4: y) at tempCoordinates, added to jacobian, by the one-pass derivative (#3189); implemented in evaluateUserFunctions.cpp
    void AddUserFunctionJacobian(const PythonUserFunctionBase< std::function<StdVector(const MainSystem&,Real,Index,const StdVector&,const StdVector&,const StdVector&,const StdVector&,const StdVector&)> >& userFunction, const char* name, Index argument, Real factor, ResizableMatrix& jacobian) const;

    //! AUTO:  the coordinates of the user functions in tempCoordinates, tempCoordinates_t, tempCoordinatesODE1, tempCoordinatesAE and tempCoordinatesData (#3191, #3195)
    void ComputeUserFunctionCoordinates() const;

    //! AUTO:  the data coordinates of the data nodes (#3195)
    void ComputeDataCoordinates(Vector& x, ConfigurationType configuration = ConfigurationType::Current) const;

    //! AUTO:  true with a postNewtonStepUserFunction, which the system calls after each Newton iteration (#3195)
    virtual bool HasPostNewtonStepBody() const override
    {
        return (parameters.postNewtonStepUserFunction!=0);
    }

    //! AUTO:  the postNewtonStepUserFunction: stores the new data coordinates, returns the error and the recommended step size (#3195); implemented in evaluateUserFunctions.cpp
    virtual Real PostNewtonStepBody(Real& recommendedStepSize) override;

    //! AUTO:  the massMatrixUserFunction at tempCoordinates into massMatrix (#3191); implemented in evaluateUserFunctions.cpp
    void EvaluateMassMatrixUserFunction(EXUmath::MatrixContainer& massMatrix, const ArrayIndex& ltg) const;

    //! AUTO:  the constraintUserFunction at tempCoordinates and tempCoordinatesAE, the size checked (#3191); implemented in evaluateUserFunctions.cpp
    void EvaluateConstraintUserFunction(bool velocityLevel, Vector& result) const;

    //! AUTO:  the derivative of the recorded constraintUserFunction by its argument of index argument (2: q, 3: q_t, 4: y, 5: z) at tempCoordinates and tempCoordinatesAE, added to jacobian, by the one-pass derivative (#3191); implemented in evaluateUserFunctions.cpp
    void AddConstraintUserFunctionJacobian(Index argument, ResizableMatrix& jacobian) const;

    //! AUTO:  the jacobianUserFunction at tempCoordinates and tempCoordinatesAE into tempJacobian, the size checked (#3191); implemented in evaluateUserFunctions.cpp
    void EvaluateJacobianUserFunction(Real factorODE2, Real factorODE2_t, Real factorODE1) const;

    virtual OutputVariableType GetOutputVariableTypes() const override
    {
        return (OutputVariableType)(
            (Index64)OutputVariableType::Coordinates +
            (Index64)OutputVariableType::Coordinates_t );
    }

};


//! AUTO: the member functions that the declarations of CObjectGenericDAE require (#2866)
#include <type_traits>
static_assert(!std::is_same<decltype(&CObjectGenericDAE::GetOutputVariableBody), decltype(&CObjectBody::GetOutputVariableBody)>::value, "CObjectGenericDAE declares output variables and must implement GetOutputVariableBody; the default of CObjectBody raises");

#endif //#ifdef include once...
