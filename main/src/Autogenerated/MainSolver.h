/** ***********************************************************************************************
* @class        MainSolverStatic
* @brief        PyBind interface (trampoline) class for static solver. With this interface, the static solver and its substructures can be accessed via python. NOTE that except from SolveSystem(...), these functions are only intended for experienced users and they need to be handled with care, as unexpected crashes may happen if used inappropriate. Furthermore, the functions have a lot of overhead (performance much lower than internal solver) due to python interfaces, and should thus be used for small systems. To access the solver in python, write: \bi
 \item[] solver = MainSolverStatic() 
\ei
 and hereafter you can access all data and functions via 'solver'.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-02-09 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

class MainSolverStatic: public MainSolverBase // AUTO: 
{
public: // AUTO: 
  CSolverStatic cSolver;                          //!< AUTO: link to C++ CSolver, not accessible from Python
  bool isInitialized;                             //!< AUTO: variable is used to see, if system is initialized ==> avoid crashes; DO not change these variables: can easily lead to crash! 
  Index4 initializedSystemSizes;                  //!< AUTO: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 


public: // AUTO: 

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: multiplicative load step factor; this factor is computed from loadStepGeometric parameters in SolveSystem(...)
  void PySetLoadStepGeometricFactor(const Real& loadStepGeometricFactorInit) { cSolver.loadStepGeometricFactor = loadStepGeometricFactorInit; }
  //! AUTO: Read (Copy) access to: multiplicative load step factor; this factor is computed from loadStepGeometric parameters in SolveSystem(...)
  Real PyGetLoadStepGeometricFactor() const { return (Real)(cSolver.loadStepGeometricFactor); }

  //! AUTO: Set function (needed in pybind) for: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 
  void PySetInitializedSystemSizes(const std::array<Index,4>& initializedSystemSizesInit) { initializedSystemSizes = initializedSystemSizesInit; }
  //! AUTO: Read (Copy) access to: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 
  std::array<Index,4> PyGetInitializedSystemSizes() const { return (std::array<Index,4>)(initializedSystemSizes); }

  //! AUTO: constructor, in order to set valid state (settings not initialized at beginning)
   MainSolverStatic() {
    isInitialized = false;
  }

  //! AUTO: const access to cSolver
  virtual const CSolverBase& GetCSolver() const override {
    return cSolver;
  }

  //! AUTO: reference access to cSolver
  virtual CSolverBase& GetCSolver() override {
    return cSolver;
  }

  //! AUTO: for static solver, this is a factor in interval [0,1]; MUST be overwritten
  Real ComputeLoadFactor(const SimulationSettings& simulationSettings) {
    return cSolver.ComputeLoadFactor(simulationSettings);
  }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "MainSolverStatic" << ":\n";
    os << ":"; 
    MainSolverBase::Print(os);
    os << "  cSolver = " << cSolver << "\n";
    os << "  cSolver.loadStepGeometricFactor = " << cSolver.loadStepGeometricFactor << "\n";
    os << "  isInitialized = " << isInitialized << "\n";
    os << "  initializedSystemSizes = " << initializedSystemSizes << "\n";
    os << "\n";
  }

};


/** ***********************************************************************************************
* @class        MainSolverImplicitSecondOrder
* @brief        PyBind interface (trampoline) class for dynamic implicit solver. Note that this solver includes the classical Newmark method (set useNewmark True; with option of index 2 reduction) as well as the generalized-alpha method. With the interface, the dynamic implicit solver and its substructures can be accessed via python. NOTE that except from SolveSystem(...), these functions are only intended for experienced users and they need to be handled with care, as unexpected crashes may happen if used inappropriate. Furthermore, the functions have a lot of overhead (performance much lower than internal solver) due to python interfaces, and should thus be used for small systems. To access the solver in python, write \bi
 \item[] solver = MainSolverImplicitSecondOrder() 
\ei
 and hereafter you can access all data and functions via 'solver'.
 In this solver, user functions are possible to extend the solver at certain parts, while keeping the overal C++ performance.
*
* @author       AUTO: Gerstmayr Johannes
* @date         AUTO: 2019-07-01 (generated)
* @date         AUTO: 2020-02-09 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ **/
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

class MainSolverImplicitSecondOrder: public MainSolverBase // AUTO: 
{
public: // AUTO: 
  CSolverImplicitSecondOrderTimeIntUserFunction cSolver;//!< AUTO: link to C++ CSolver, not accessible from Python
  bool isInitialized;                             //!< AUTO: variable is used to see, if system is initialized ==> avoid crashes; DO not change these variables: can easily lead to crash! 
  Index4 initializedSystemSizes;                  //!< AUTO: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 


public: // AUTO: 

  // AUTO: access functions
  //! AUTO: Set function (needed in pybind) for: copy of parameter in timeIntegration.generalizedAlpha
  void PySetNewmarkBeta(const Real& newmarkBetaInit) { cSolver.newmarkBeta = newmarkBetaInit; }
  //! AUTO: Read (Copy) access to: copy of parameter in timeIntegration.generalizedAlpha
  Real PyGetNewmarkBeta() const { return (Real)(cSolver.newmarkBeta); }

  //! AUTO: Set function (needed in pybind) for: copy of parameter in timeIntegration.generalizedAlpha
  void PySetNewmarkGamma(const Real& newmarkGammaInit) { cSolver.newmarkGamma = newmarkGammaInit; }
  //! AUTO: Read (Copy) access to: copy of parameter in timeIntegration.generalizedAlpha
  Real PyGetNewmarkGamma() const { return (Real)(cSolver.newmarkGamma); }

  //! AUTO: Set function (needed in pybind) for: copy of parameter in timeIntegration.generalizedAlpha
  void PySetAlphaM(const Real& alphaMInit) { cSolver.alphaM = alphaMInit; }
  //! AUTO: Read (Copy) access to: copy of parameter in timeIntegration.generalizedAlpha
  Real PyGetAlphaM() const { return (Real)(cSolver.alphaM); }

  //! AUTO: Set function (needed in pybind) for: copy of parameter in timeIntegration.generalizedAlpha
  void PySetAlphaF(const Real& alphaFInit) { cSolver.alphaF = alphaFInit; }
  //! AUTO: Read (Copy) access to: copy of parameter in timeIntegration.generalizedAlpha
  Real PyGetAlphaF() const { return (Real)(cSolver.alphaF); }

  //! AUTO: Set function (needed in pybind) for: copy of parameter in timeIntegration.generalizedAlpha
  void PySetSpectralRadius(const Real& spectralRadiusInit) { cSolver.spectralRadius = spectralRadiusInit; }
  //! AUTO: Read (Copy) access to: copy of parameter in timeIntegration.generalizedAlpha
  Real PyGetSpectralRadius() const { return (Real)(cSolver.spectralRadius); }

  //! AUTO: Set function (needed in pybind) for: locally computed parameter from generalizedAlpha parameters
  void PySetFactJacAlgorithmic(const Real& factJacAlgorithmicInit) { cSolver.factJacAlgorithmic = factJacAlgorithmicInit; }
  //! AUTO: Read (Copy) access to: locally computed parameter from generalizedAlpha parameters
  Real PyGetFactJacAlgorithmic() const { return (Real)(cSolver.factJacAlgorithmic); }

  //! AUTO: Set function (needed in pybind) for: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 
  void PySetInitializedSystemSizes(const std::array<Index,4>& initializedSystemSizesInit) { initializedSystemSizes = initializedSystemSizesInit; }
  //! AUTO: Read (Copy) access to: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 
  std::array<Index,4> PyGetInitializedSystemSizes() const { return (std::array<Index,4>)(initializedSystemSizes); }

  //! AUTO: constructor, in order to set valid state (settings not initialized at beginning)
   MainSolverImplicitSecondOrder();
  //! AUTO: const access to cSolver
  virtual const CSolverBase& GetCSolver() const override {
    return cSolver;
  }

  //! AUTO: reference access to cSolver
  virtual CSolverBase& GetCSolver() override {
    return cSolver;
  }

  //! AUTO: for static solver, this is a factor in interval [0,1]; MUST be overwritten
  Real ComputeLoadFactor(const SimulationSettings& simulationSettings) {
    return cSolver.ComputeLoadFactor(simulationSettings);
  }

  //! AUTO: get locally stored / last computed algorithmic accelerations
  py::array_t<Real> GetAAlgorithmic();
  //! AUTO: get locally stored / last computed algorithmic accelerations at start of step
  py::array_t<Real> GetStartOfStepStateAAlgorithmic();
  //! AUTO: set user function
  void SetUserFunctionUpdateCurrentTime(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunction& userFunction) {
    cSolver.SetUserFunctionUpdateCurrentTime(this, &mainSystem, userFunction);
  }

  //! AUTO: set user function
  void SetUserFunctionInitializeStep(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunction& userFunction) {
    cSolver.SetUserFunctionInitializeStep(this, &mainSystem, userFunction);
  }

  //! AUTO: set user function
  void SetUserFunctionFinishStep(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunction& userFunction) {
    cSolver.SetUserFunctionFinishStep(this, &mainSystem, userFunction);
  }

  //! AUTO: set user function
  void SetUserFunctionDiscontinuousIteration(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunction& userFunction) {
    cSolver.SetUserFunctionDiscontinuousIteration(this, &mainSystem, userFunction);
  }

  //! AUTO: set user function
  void SetUserFunctionNewton(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunction& userFunction) {
    cSolver.SetUserFunctionNewton(this, &mainSystem, userFunction);
  }

  //! AUTO: set user function
  void SetUserFunctionComputeNewtonUpdate(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunction& userFunction) {
    cSolver.SetUserFunctionComputeNewtonUpdate(this, &mainSystem, userFunction);
  }

  //! AUTO: set user function
  void SetUserFunctionComputeNewtonResidual(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunction& userFunction) {
    cSolver.SetUserFunctionComputeNewtonResidual(this, &mainSystem, userFunction);
  }

  //! AUTO: set user function
  void SetUserFunctionComputeNewtonJacobian(MainSystem& mainSystem, const MainSolverImplicitSecondOrderUserFunction& userFunction) {
    cSolver.SetUserFunctionComputeNewtonJacobian(this, &mainSystem, userFunction);
  }

  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)
  virtual void Print(std::ostream& os) const
  {
    os << "MainSolverImplicitSecondOrder" << ":\n";
    os << ":"; 
    MainSolverBase::Print(os);
    os << "  cSolver = " << cSolver << "\n";
    os << "  cSolver.newmarkBeta = " << cSolver.newmarkBeta << "\n";
    os << "  cSolver.newmarkGamma = " << cSolver.newmarkGamma << "\n";
    os << "  cSolver.alphaM = " << cSolver.alphaM << "\n";
    os << "  cSolver.alphaF = " << cSolver.alphaF << "\n";
    os << "  cSolver.spectralRadius = " << cSolver.spectralRadius << "\n";
    os << "  cSolver.factJacAlgorithmic = " << cSolver.factJacAlgorithmic << "\n";
    os << "  isInitialized = " << isInitialized << "\n";
    os << "  initializedSystemSizes = " << initializedSystemSizes << "\n";
    os << "\n";
  }

};


