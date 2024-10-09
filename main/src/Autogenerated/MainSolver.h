/** ***********************************************************************************************
* @class        MainSolverStatic
* @brief        PyBind interface (trampoline) class for static solver. With this interface, the static solver and its substructures can be accessed via Python. NOTE that except from SolveSystem(...), these functions are only intended for experienced users and they need to be handled with care, as unexpected crashes may happen if used inappropriate. Furthermore, the functions have a lot of overhead (performance much lower than internal solver) due to Python interfaces, and should thus be used for small systems. To access the solver in Python, write: \\ 
 \texttt{solver = MainSolverStatic()} \\ 
and hereafter you can access all data and functions via 'solver'.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2024-08-07 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/

#ifndef MAINSOLVER__H
#define MAINSOLVER__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

class MainSolverStatic: public MainSolverBase // AUTO: 
{
public: // AUTO: 
  CSolverStatic cSolver;                          //!< AUTO: link to C++ CSolver, not accessible from Python
  Index4 initializedSystemSizes;                  //!< AUTO: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 
  bool isInitialized;                             //!< AUTO: variable is used to see, if system is initialized ==> avoid crashes; DO not change these variables: can easily lead to crash! 


public: // AUTO: 

  // AUTO: access functions
  //! AUTO: for static solver, this is a factor in interval [0,1]; MUST be overwritten
  Real ComputeLoadFactor(const SimulationSettings& simulationSettings) {
    return cSolver.ComputeLoadFactor(simulationSettings);
  }

  //! AUTO: const access to cSolver
  virtual const CSolverBase& GetCSolver() const override {
    return cSolver;
  }

  //! AUTO: reference access to cSolver
  virtual CSolverBase& GetCSolver() override {
    return cSolver;
  }

  //! AUTO: Set function (needed in pybind) for: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 
  void PySetInitializedSystemSizes(const std::array<Index,4>& initializedSystemSizesInit) { initializedSystemSizes = initializedSystemSizesInit; }
  //! AUTO: Read (Copy) access to: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 
  std::array<Index,4> PyGetInitializedSystemSizes() const { return std::array<Index,4>(initializedSystemSizes); }

  //! AUTO: Set function (needed in pybind) for: multiplicative load step factor; this factor is computed from loadStepGeometric parameters in SolveSystem(...)
  void PySetLoadStepGeometricFactor(const Real& loadStepGeometricFactorInit) { cSolver.loadStepGeometricFactor = loadStepGeometricFactorInit; }
  //! AUTO: Read (Copy) access to: multiplicative load step factor; this factor is computed from loadStepGeometric parameters in SolveSystem(...)
  Real PyGetLoadStepGeometricFactor() const { return Real(cSolver.loadStepGeometricFactor); }

  //! AUTO: constructor, in order to set valid state (settings not initialized at beginning)
   MainSolverStatic() {
    isInitialized=false;
  }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "MainSolverStatic" << ":\n";
    os << ":"; 
    MainSolverBase::Print(os);
    os << "  cSolver = " << cSolver << "\n";
    os << "  initializedSystemSizes = " << initializedSystemSizes << "\n";
    os << "  isInitialized = " << isInitialized << "\n";
    os << "  cSolver.loadStepGeometricFactor = " << cSolver.loadStepGeometricFactor << "\n";
    os << "\n";
  }

};


/** ***********************************************************************************************
* @class        MainSolverImplicitSecondOrder
* @brief        PyBind interface (trampoline) class for dynamic implicit solver. Note that this solver includes the classical Newmark method (set useNewmark True; with option of index 2 reduction) as well as the generalized-alpha method. With the interface, the dynamic implicit solver and its substructures can be accessed via Python. NOTE that except from SolveSystem(...), these functions are only intended for experienced users and they need to be handled with care, as unexpected crashes may happen if used inappropriate. Furthermore, the functions have a lot of overhead (still fast, but performance much lower than internal solver) due to Python interfaces, and should thus be used for small systems. To access the solver in Python, write: \\ 
 \texttt{solver = MainSolverImplicitSecondOrder()} \\ 
and hereafter you can access all data and functions via 'solver'.
 In this solver, user functions are possible to extend the solver at certain parts, while keeping the overal C++ performance. User functions, which are added with SetUserFunction...(...), have the arguments (MainSolver, MainSystem, simulationSettings), except for ComputeNewtonUpdate which adds the initial flag as an additional argument and ComputeNewtonResidual, which returns the scalar residual.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2024-08-07 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

class MainSolverImplicitSecondOrder: public MainSolverBase // AUTO: 
{
public: // AUTO: 
  CSolverImplicitSecondOrderTimeIntUserFunction cSolver;//!< AUTO: link to C++ CSolver, not accessible from Python
  Index4 initializedSystemSizes;                  //!< AUTO: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 
  bool isInitialized;                             //!< AUTO: variable is used to see, if system is initialized ==> avoid crashes; DO not change these variables: can easily lead to crash! 


public: // AUTO: 
  //! AUTO: default constructor with parameter initialization
  MainSolverImplicitSecondOrder(): MainSolverBase()
  {
    isInitialized = false;
  };

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: copy of parameter in timeIntegration.generalizedAlpha
  void PySetAlphaF(const Real& alphaFInit) { GetCSolverImplicitSecondOrder().alphaF = alphaFInit; }
  //! AUTO: Read (Copy) access to: copy of parameter in timeIntegration.generalizedAlpha
  Real PyGetAlphaF() const { return Real(GetCSolverImplicitSecondOrder().alphaF); }

  //! AUTO: Set function (needed in pybind) for: copy of parameter in timeIntegration.generalizedAlpha
  void PySetAlphaM(const Real& alphaMInit) { GetCSolverImplicitSecondOrder().alphaM = alphaMInit; }
  //! AUTO: Read (Copy) access to: copy of parameter in timeIntegration.generalizedAlpha
  Real PyGetAlphaM() const { return Real(GetCSolverImplicitSecondOrder().alphaM); }

  //! AUTO: for static solver, this is a factor in interval [0,1]; MUST be overwritten
  Real ComputeLoadFactor(const SimulationSettings& simulationSettings) {
    return GetCSolverImplicitSecondOrder().ComputeLoadFactor(simulationSettings);
  }

  //! AUTO: Set function (needed in pybind) for: locally computed parameter from generalizedAlpha parameters
  void PySetFactJacAlgorithmic(const Real& factJacAlgorithmicInit) { GetCSolverImplicitSecondOrder().factJacAlgorithmic = factJacAlgorithmicInit; }
  //! AUTO: Read (Copy) access to: locally computed parameter from generalizedAlpha parameters
  Real PyGetFactJacAlgorithmic() const { return Real(GetCSolverImplicitSecondOrder().factJacAlgorithmic); }

  //! AUTO: get locally stored / last computed algorithmic accelerations
  py::array_t<Real> GetAAlgorithmic();
  //! AUTO: const access to cSolver
  virtual const CSolverBase& GetCSolver() const override {
    return GetCSolverImplicitSecondOrder();
  }

  //! AUTO: reference access to cSolver
  virtual CSolverBase& GetCSolver() override {
    return GetCSolverImplicitSecondOrder();
  }

  //! AUTO: const access to cSolver
  const CSolverImplicitSecondOrderTimeIntUserFunction& GetCSolverImplicitSecondOrder() const {
    return cSolver;
  }

  //! AUTO: reference access to cSolver
  CSolverImplicitSecondOrderTimeIntUserFunction& GetCSolverImplicitSecondOrder() {
    return cSolver;
  }

  //! AUTO: get locally stored / last computed algorithmic accelerations at start of step
  py::array_t<Real> GetStartOfStepStateAAlgorithmic();
  //! AUTO: Set function (needed in pybind) for: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 
  void PySetInitializedSystemSizes(const std::array<Index,4>& initializedSystemSizesInit) { initializedSystemSizes = initializedSystemSizesInit; }
  //! AUTO: Read (Copy) access to: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 
  std::array<Index,4> PyGetInitializedSystemSizes() const { return std::array<Index,4>(initializedSystemSizes); }

  //! AUTO: Set function (needed in pybind) for: copy of parameter in timeIntegration.generalizedAlpha
  void PySetNewmarkBeta(const Real& newmarkBetaInit) { GetCSolverImplicitSecondOrder().newmarkBeta = newmarkBetaInit; }
  //! AUTO: Read (Copy) access to: copy of parameter in timeIntegration.generalizedAlpha
  Real PyGetNewmarkBeta() const { return Real(GetCSolverImplicitSecondOrder().newmarkBeta); }

  //! AUTO: Set function (needed in pybind) for: copy of parameter in timeIntegration.generalizedAlpha
  void PySetNewmarkGamma(const Real& newmarkGammaInit) { GetCSolverImplicitSecondOrder().newmarkGamma = newmarkGammaInit; }
  //! AUTO: Read (Copy) access to: copy of parameter in timeIntegration.generalizedAlpha
  Real PyGetNewmarkGamma() const { return Real(GetCSolverImplicitSecondOrder().newmarkGamma); }

  //! AUTO: set user function
  void SetUserFunctionComputeNewtonJacobian(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunction& userFunction) {
    GetCSolverImplicitSecondOrder().SetUserFunctionComputeNewtonJacobian(this, &mainSystem, userFunction);
  }

  //! AUTO: set user function
  void SetUserFunctionComputeNewtonResidual(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunctionReal& userFunction) {
    GetCSolverImplicitSecondOrder().SetUserFunctionComputeNewtonResidual(this, &mainSystem, userFunction);
  }

  //! AUTO: set user function
  void SetUserFunctionComputeNewtonUpdate(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunctionBool& userFunction) {
    GetCSolverImplicitSecondOrder().SetUserFunctionComputeNewtonUpdate(this, &mainSystem, userFunction);
  }

  //! AUTO: set user function
  void SetUserFunctionDiscontinuousIteration(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunction& userFunction) {
    GetCSolverImplicitSecondOrder().SetUserFunctionDiscontinuousIteration(this, &mainSystem, userFunction);
  }

  //! AUTO: set user function
  void SetUserFunctionFinishStep(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunction& userFunction) {
    GetCSolverImplicitSecondOrder().SetUserFunctionFinishStep(this, &mainSystem, userFunction);
  }

  //! AUTO: set user function
  void SetUserFunctionInitializeStep(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunction& userFunction) {
    GetCSolverImplicitSecondOrder().SetUserFunctionInitializeStep(this, &mainSystem, userFunction);
  }

  //! AUTO: set user function
  void SetUserFunctionNewton(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunction& userFunction) {
    GetCSolverImplicitSecondOrder().SetUserFunctionNewton(this, &mainSystem, userFunction);
  }

  //! AUTO: set user function
  void SetUserFunctionPostNewton(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunctionReal& userFunction) {
    GetCSolverImplicitSecondOrder().SetUserFunctionPostNewton(this, &mainSystem, userFunction);
  }

  //! AUTO: set user function
  void SetUserFunctionUpdateCurrentTime(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunction& userFunction) {
    GetCSolverImplicitSecondOrder().SetUserFunctionUpdateCurrentTime(this, &mainSystem, userFunction);
  }

  //! AUTO: Set function (needed in pybind) for: copy of parameter in timeIntegration.generalizedAlpha
  void PySetSpectralRadius(const Real& spectralRadiusInit) { GetCSolverImplicitSecondOrder().spectralRadius = spectralRadiusInit; }
  //! AUTO: Read (Copy) access to: copy of parameter in timeIntegration.generalizedAlpha
  Real PyGetSpectralRadius() const { return Real(GetCSolverImplicitSecondOrder().spectralRadius); }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "MainSolverImplicitSecondOrder" << ":\n";
    os << ":"; 
    MainSolverBase::Print(os);
    os << "  GetCSolverImplicitSecondOrder().alphaF = " << GetCSolverImplicitSecondOrder().alphaF << "\n";
    os << "  GetCSolverImplicitSecondOrder().alphaM = " << GetCSolverImplicitSecondOrder().alphaM << "\n";
    os << "  GetCSolverImplicitSecondOrder().factJacAlgorithmic = " << GetCSolverImplicitSecondOrder().factJacAlgorithmic << "\n";
    os << "  initializedSystemSizes = " << initializedSystemSizes << "\n";
    os << "  isInitialized = " << isInitialized << "\n";
    os << "  GetCSolverImplicitSecondOrder().newmarkBeta = " << GetCSolverImplicitSecondOrder().newmarkBeta << "\n";
    os << "  GetCSolverImplicitSecondOrder().newmarkGamma = " << GetCSolverImplicitSecondOrder().newmarkGamma << "\n";
    os << "  GetCSolverImplicitSecondOrder().spectralRadius = " << GetCSolverImplicitSecondOrder().spectralRadius << "\n";
    os << "\n";
  }

};


/** ***********************************************************************************************
* @class        MainSolverExplicit
* @brief        PyBind interface (trampoline) class for dynamic explicit solver. Note that this solver includes the 1st order explicit Euler scheme and the 4th order Runge-Kutta scheme with 5th order error estimation (DOPRI5). With the interface, the solver and its substructures can be accessed via Python. NOTE that except from SolveSystem(...), these functions are only intended for experienced users and they need to be handled with care, as unexpected crashes may happen if used inappropriate. Furthermore, the functions have a lot of overhead (still fast, but performance much lower than internal solver) due to Python interfaces, and should thus be used for small systems. To access the solver in Python, write \\ 
 \texttt{solver = MainSolverExplicit()} \\ 
and hereafter you can access all data and functions via 'solver'.
 In this solver, no user functions are possible, but you can use SolverImplicitSecondOrder instead (turning off Newton gives explicit scheme ...).
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2024-08-07 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

class MainSolverExplicit: public MainSolverBase // AUTO: 
{
public: // AUTO: 
  CSolverExplicitTimeInt cSolver;                 //!< AUTO: link to C++ CSolver, not accessible from Python
  Index4 initializedSystemSizes;                  //!< AUTO: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 
  bool isInitialized;                             //!< AUTO: variable is used to see, if system is initialized ==> avoid crashes; DO not change these variables: can easily lead to crash! 


public: // AUTO: 

  // AUTO: access functions
  //! AUTO: for static solver, this is a factor in interval [0,1]; MUST be overwritten
  Real ComputeLoadFactor(const SimulationSettings& simulationSettings) {
    return cSolver.ComputeLoadFactor(simulationSettings);
  }

  //! AUTO: const access to cSolver
  virtual const CSolverBase& GetCSolver() const override {
    return cSolver;
  }

  //! AUTO: reference access to cSolver
  virtual CSolverBase& GetCSolver() override {
    return cSolver;
  }

  //! AUTO: return order of method (higher value in methods with automatic step size, e.g., DOPRI5=5)
  Index GetMethodOrder() const {
    return cSolver.rk.orderMethod;
  }

  //! AUTO: return number of stages in current method
  Index GetNumberOfStages() const {
    return cSolver.nStages;
  }

  //! AUTO: Set function (needed in pybind) for: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 
  void PySetInitializedSystemSizes(const std::array<Index,4>& initializedSystemSizesInit) { initializedSystemSizes = initializedSystemSizesInit; }
  //! AUTO: Read (Copy) access to: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 
  std::array<Index,4> PyGetInitializedSystemSizes() const { return std::array<Index,4>(initializedSystemSizes); }

  //! AUTO: constructor, in order to set valid state (settings not initialized at beginning)
   MainSolverExplicit();
  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "MainSolverExplicit" << ":\n";
    os << ":"; 
    MainSolverBase::Print(os);
    os << "  cSolver = " << cSolver << "\n";
    os << "  initializedSystemSizes = " << initializedSystemSizes << "\n";
    os << "  isInitialized = " << isInitialized << "\n";
    os << "\n";
  }

};



#endif //#ifdef include once...
