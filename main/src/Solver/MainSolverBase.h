/** ***********************************************************************************************
* @class        MainSolverBase
* @brief        PyBind interface (trampoline) class for CSolverStatic. With this interface, the static solver and its substructures can be accessed via python. NOTE that except from SolveSystem(...), these functions are only intended for experienced users and they need to be handled with care, as unexpected crashes may happen if used inappropriate. Furthermore, the functions have a lot of overhead (performance much lower than internal solver) due to python interfaces, and should thus be used for small systems.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-01-07 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ **/
#ifndef MAINSOLVERBASE__H
#define MAINSOLVERBASE__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Main/OutputVariable.h"
#include "Linalg/BasicLinalg.h"

#include <pybind11/pybind11.h> //for integrated python connectivity (==>put functionality into separate file ...!!!)
#include <pybind11/eval.h>
#include <pybind11/numpy.h>       //interface to numpy
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>       //interface to numpy
#include <pybind11/functional.h> //for function handling ... otherwise gives a python error (no compilation error in C++ !)
//
#include "Linalg/BasicLinalg.h"
#include "Linalg/LinearSolver.h"

#include "Linalg/Geometry.h"

#include "Main/MainSystem.h"

#include "Pymodules/PybindUtilities.h"

//#include "Solver/TimeIntegrationSolver.h"
#include "Main/SystemContainer.h"

#include "Main/MainSystemContainer.h"

#include "Solver/CSolverImplicitSecondOrder.h"
#include "Solver/CSolverExplicit.h"
#include "Solver/CSolverStatic.h"


class MainSolverBase // 
{
public: // 
	bool isInitialized;                             //!< variable is used to see, if system is initialized ==> avoid crashes; DO not change these variables: can easily lead to crash! 
	Index4 initializedSystemSizes;                  //!< index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 


public: // 

	//! constructor, in order to set valid state (settings not initialized at beginning)
	MainSolverBase() {
		isInitialized = false;
	}
	virtual ~MainSolverBase() {} //added for correct deletion of derived classes

	//+++++++++++++++++++++++++++++++++++++++++++++++
	// must be overwritten:
	//! access to CSolver
	virtual const CSolverBase& GetCSolver() const;
	virtual CSolverBase& GetCSolver();
	//+++++++++++++++++++++++++++++++++++++++++++++++

	// access functions
	//! Set function (needed in pybind) for: timer which measures the CPU time of solver sub functions
	virtual void PySetTimer(const CSolverTimer& timerInit) { GetCSolver().timer = timerInit; }
	//! Read (Copy) access to: timer which measures the CPU time of solver sub functions
	//virtual const CSolverTimer& PyGetTimer() const { return (GetCSolver().timer); }
	virtual const CSolverTimer& PyGetTimer() const { return GetCSolver().timer; }

	//! Set function (needed in pybind) for: all information about iterations (steps, discontinuous iteration, newton,...)
	virtual void PySetIt(const SolverIterationData& itInit) { GetCSolver().it = itInit; }
	//! Read (Copy) access to: all information about iterations (steps, discontinuous iteration, newton,...)
    virtual const SolverIterationData& PyGetIt() const { return GetCSolver().it; } //2023-02-25: change to const ...& return value to allow write access

	//! Set function (needed in pybind) for: all information about tolerances, errors and residua
	virtual void PySetConv(const SolverConvergenceData& convInit) { GetCSolver().conv = convInit; }
	//! Read (Copy) access to: all information about tolerances, errors and residua
	virtual const SolverConvergenceData& PyGetConv() const { return GetCSolver().conv; }

	//! Set function (needed in pybind) for: output modes and timers for exporting solver information and solution
	virtual void PySetOutput(const SolverOutputData& outputInit) { GetCSolver().output = outputInit; }
	//! Read (Copy) access to: output modes and timers for exporting solver information and solution
	virtual const SolverOutputData& PyGetOutput() const { return GetCSolver().output; }

	//! Set function (needed in pybind) for: copy of newton settings from timeint or staticSolver
	virtual void PySetNewton(const NewtonSettings& newtonInit) { GetCSolver().newton = newtonInit; }
	//! Read (Copy) access to: copy of newton settings from timeint or staticSolver
	virtual const NewtonSettings& PyGetNewton() const { return GetCSolver().newton; }

	//only for static solver:
	////! Set function (needed in pybind) for: multiplicative load step factor; this factor is computed from loadStepGeometric parameters in SolveSystem(...)
	//virtual void PySetLoadStepGeometricFactor(const Real& loadStepGeometricFactorInit) { GetCSolver().loadStepGeometricFactor = loadStepGeometricFactorInit; }
	////! Read (Copy) access to: multiplicative load step factor; this factor is computed from loadStepGeometric parameters in SolveSystem(...)
	//virtual Real PyGetLoadStepGeometricFactor() const { return (Real)(GetCSolver().loadStepGeometricFactor); }

	//! Set function (needed in pybind) for: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 
	virtual void PySetInitializedSystemSizes(const std::array<Index, 4>& initializedSystemSizesInit) { initializedSystemSizes = initializedSystemSizesInit; }
	//! Read (Copy) access to: index-array contains 4 integers: nODE2, nODE1, nAE and nData of initialization: this guaranties, that no function is called with wrong system sizes; DO not change these variables: can easily lead to crash! 
	virtual std::array<Index, 4> PyGetInitializedSystemSizes() const { return (std::array<Index, 4>)(initializedSystemSizes); }

	//! check if MainSolver and MainSystem are correctly initialized ==> otherwise raise SysError
	virtual bool CheckInitialized(const MainSystem& mainSystem) const;

	//! set initialized flags flags true (called from cSolver.initializeSolver)
	virtual void InitializeCheckInitialized(const MainSystem& mainSystem);

    //! get solver name - needed for output file header and visualization window
    virtual std::string GetSolverName() const
    {
        return GetCSolver().GetSolverName();
    }

    //! get solver error message after SolveSteps(...) or SolveSystem(...)
    virtual std::string GetErrorString() const
    {
        return GetCSolver().GetErrorString();
    }

    //! return true, if static solver; needs to be overwritten in derived class
	virtual bool IsStaticSolver() const 
	{
		return GetCSolver().IsStaticSolver();
	}

	//! compute simulation end time (depends on static or time integration solver)
	virtual Real GetSimulationEndTime(const SimulationSettings& simulationSettings) const 
	{
		return GetCSolver().GetSimulationEndTime(simulationSettings);
	}

	//! reduce step size (1..normal, 2..severe problems); return true, if reduction was successful
	virtual bool ReduceStepSize(MainSystem& mainSystem, const SimulationSettings& simulationSettings, Index severity) 
	{
		CheckInitialized(mainSystem); return GetCSolver().ReduceStepSize(*(mainSystem.cSystem), simulationSettings, severity);
	}

	//! increase step size if convergence is good
	virtual void IncreaseStepSize(MainSystem& mainSystem, const SimulationSettings& simulationSettings) 
	{
		CheckInitialized(mainSystem); GetCSolver().IncreaseStepSize(*(mainSystem.cSystem), simulationSettings);
	}

	//! increase step size if convergence is good
	virtual bool HasAutomaticStepSizeControl(MainSystem& mainSystem, const SimulationSettings& simulationSettings) const
	{
		CheckInitialized(mainSystem); 
		return GetCSolver().HasAutomaticStepSizeControl();
	}

	//only for static solver:
	////! for static solver, this is a factor in interval [0,1]; MUST be overwritten
	//virtual Real ComputeLoadFactor(const SimulationSettings& simulationSettings) {
	//	return GetCSolver().ComputeLoadFactor(simulationSettings);
	//}

	//! initialize solverSpecific,data,it,conv; set/compute initial conditions (solver-specific!); initialize output files
	virtual bool InitializeSolver(MainSystem& mainSystem, const SimulationSettings& simulationSettings)
	{
		InitializeCheckInitialized(mainSystem); //in order to allow user functions within InitializeSolver()
		bool returnValue = GetCSolver().InitializeSolver(*(mainSystem.cSystem), simulationSettings);
		if (!returnValue) { isInitialized = false; }

		return returnValue;
	}

	//! pre-initialize for solver specific tasks; called at beginning of InitializeSolver, right after Solver data reset
	virtual void PreInitializeSolverSpecific(MainSystem& mainSystem, const SimulationSettings& simulationSettings) 
	{
		CheckInitialized(mainSystem); GetCSolver().PreInitializeSolverSpecific(*(mainSystem.cSystem), simulationSettings);
	}

	//! initialize output files; called from InitializeSolver()
	virtual void InitializeSolverOutput(MainSystem& mainSystem, const SimulationSettings& simulationSettings)
	{
		CheckInitialized(mainSystem); GetCSolver().InitializeSolverOutput(*(mainSystem.cSystem), simulationSettings);
	}

	//! check if system is solvable; initialize dense/sparse computation modes
	virtual bool InitializeSolverPreChecks(MainSystem& mainSystem, const SimulationSettings& simulationSettings) 
	{
		CheckInitialized(mainSystem); return GetCSolver().InitializeSolverPreChecks(*(mainSystem.cSystem), simulationSettings);
	}

	//! initialize all data,it,conv; called from InitializeSolver()
	virtual void InitializeSolverData(MainSystem& mainSystem, const SimulationSettings& simulationSettings)
	{
		CheckInitialized(mainSystem); GetCSolver().InitializeSolverData(*(mainSystem.cSystem), simulationSettings);
	}

	//! set/compute initial conditions (solver-specific!); called from InitializeSolver()
	virtual void InitializeSolverInitialConditions(MainSystem& mainSystem, const SimulationSettings& simulationSettings) 
	{
		CheckInitialized(mainSystem); GetCSolver().InitializeSolverInitialConditions(*(mainSystem.cSystem), simulationSettings);
	}

	//! post-initialize for solver specific tasks; called at the end of InitializeSolver
	virtual void PostInitializeSolverSpecific(MainSystem& mainSystem, const SimulationSettings& simulationSettings) {
		CheckInitialized(mainSystem); GetCSolver().PostInitializeSolverSpecific(*(mainSystem.cSystem), simulationSettings);
	}

	//! solve System: InitializeSolver, SolveSteps, FinalizeSolver
	virtual bool SolveSystem(MainSystem& mainSystem, const SimulationSettings& simulationSettings) 
	{
		InitializeCheckInitialized(mainSystem); //needs to be called, as otherwise, user functions will fail!
		return GetCSolver().SolveSystem(*(mainSystem.cSystem), simulationSettings);
	}

	//! write concluding information (timer statistics, messages) and close files
	virtual void FinalizeSolver(MainSystem& mainSystem, const SimulationSettings& simulationSettings) 
	{
		CheckInitialized(mainSystem); GetCSolver().FinalizeSolver(*(mainSystem.cSystem), simulationSettings);
	}

	//! main solver part: calls multiple InitializeStep(...)/ DiscontinuousIteration(...)/ FinishStep(...); do step reduction if necessary; return true if success, false else
	virtual bool SolveSteps(MainSystem& mainSystem, const SimulationSettings& simulationSettings) 
	{
		CheckInitialized(mainSystem); return GetCSolver().SolveSteps(*(mainSystem.cSystem), simulationSettings);
	}

	//! update currentTime (and load factor); MUST be overwritten in special solver class
	virtual void UpdateCurrentTime(MainSystem& mainSystem, const SimulationSettings& simulationSettings) 
	{
		CheckInitialized(mainSystem); GetCSolver().UpdateCurrentTime(*(mainSystem.cSystem), simulationSettings);
	}

	//! initialize static step / time step; python-functions; do some outputs, checks, etc.
	virtual void InitializeStep(MainSystem& mainSystem, const SimulationSettings& simulationSettings)
	{
		CheckInitialized(mainSystem); GetCSolver().InitializeStep(*(mainSystem.cSystem), simulationSettings);
	}

	//! finish static step / time step; write output of results to file; writeSolution not available here as compared to CSolverBase.h!
	virtual void FinishStep(MainSystem& mainSystem, const SimulationSettings& simulationSettings)
	{
		CheckInitialized(mainSystem); GetCSolver().FinishStep(*(mainSystem.cSystem), simulationSettings);
	}

	//! perform discontinuousIteration for static step / time step; CALLS ComputeNewtonResidual
	virtual bool DiscontinuousIteration(MainSystem& mainSystem, const SimulationSettings& simulationSettings)
	{
		CheckInitialized(mainSystem); return GetCSolver().DiscontinuousIteration(*(mainSystem.cSystem), simulationSettings);
	}

	//! perform Newton method for given solver method
	virtual bool Newton(MainSystem& mainSystem, const SimulationSettings& simulationSettings)
	{
		CheckInitialized(mainSystem); return GetCSolver().Newton(*(mainSystem.cSystem), simulationSettings);
	}

	//! perform PostNewton method for given solver method
	virtual Real PostNewton(MainSystem& mainSystem, const SimulationSettings& simulationSettings)
	{
		CheckInitialized(mainSystem); return GetCSolver().PostNewton(*(mainSystem.cSystem), simulationSettings);
	}

	//! compute residual for Newton method (e.g. static or time step); store result vector in systemResidual and return scalar residual
	virtual Real ComputeNewtonResidual(MainSystem& mainSystem, const SimulationSettings& simulationSettings)
	{
		CheckInitialized(mainSystem); return GetCSolver().ComputeNewtonResidual(*(mainSystem.cSystem), simulationSettings);
	}

	//! compute update for currentState from newtonSolution (decrement from residual and jacobian)
	virtual void ComputeNewtonUpdate(MainSystem& mainSystem, const SimulationSettings& simulationSettings, bool initial = false)
	{
		CheckInitialized(mainSystem); GetCSolver().ComputeNewtonUpdate(*(mainSystem.cSystem), simulationSettings, initial);
	}

	//! compute jacobian for newton method of given solver method; store result in systemJacobian
	virtual void ComputeNewtonJacobian(MainSystem& mainSystem, const SimulationSettings& simulationSettings) 
	{
		CheckInitialized(mainSystem); GetCSolver().ComputeNewtonJacobian(*(mainSystem.cSystem), simulationSettings);
	}

	//! write unique file header, depending on static/ dynamic simulation
	virtual void WriteSolutionFileHeader(MainSystem& mainSystem, const SimulationSettings& simulationSettings) 
	{
		CheckInitialized(mainSystem); GetCSolver().WriteSolutionFileHeader(*(mainSystem.cSystem), simulationSettings);
	}

	//! write unique coordinates solution file
	virtual void WriteCoordinatesToFile(MainSystem& mainSystem, const SimulationSettings& simulationSettings) 
	{
		CheckInitialized(mainSystem); GetCSolver().WriteCoordinatesToFile(*(mainSystem.cSystem), simulationSettings);
	}

	//! return true, if file or console output is at or above the given level
	virtual bool IsVerboseCheck(Index level) const 
	{
		return GetCSolver().IsVerboseCheck(level);
	}

	//! write to console and/or file in case of level
	virtual void VerboseWrite(Index level, const std::string& str) 
	{
		GetCSolver().VerboseWrite(level, str);
	}

	//! number of ODE2 equations in solver
	virtual Index GetODE2size() const 
	{
		return GetCSolver().data.nODE2;
	}

	//! number of ODE1 equations in solver (not yet implemented)
	virtual Index GetODE1size() const 
	{
		return GetCSolver().data.nODE1;
	}

	//! number of algebraic equations in solver
	virtual Index GetAEsize() const 
	{
		return GetCSolver().data.nAE;
	}

	//! number of data (history) variables in solver
	virtual Index GetDataSize() const 
	{
		return GetCSolver().data.nData;
	}

	//! get locally stored / last computed system jacobian of solver
	virtual py::array_t<Real> GetSystemJacobian() const;
	//! get locally stored / last computed mass matrix of solver
	virtual py::array_t<Real> GetSystemMassMatrix() const;
	//! get locally stored / last computed system residual
	virtual py::array_t<Real> GetSystemResidual() const;
	//! get locally stored / last computed solution (=increment) of Newton
	virtual py::array_t<Real> GetNewtonSolution() const;
	//! set locally stored system jacobian of solver; must have size nODE2+nODE1+nAE
	virtual void SetSystemJacobian(const py::array_t<Real>& systemJacobian);
	//! set locally stored mass matrix of solver; must have size nODE2+nODE1+nAE
	virtual void SetSystemMassMatrix(const py::array_t<Real>& systemMassMatrix);
	//! set locally stored system residual; must have size nODE2+nODE1+nAE
	virtual void SetSystemResidual(const Vector& systemResidual);
	//! compute systemMassMatrix (multiplied with factor) in cSolver and return mass matrix
	virtual void ComputeMassMatrix(MainSystem& mainSystem/*, const SimulationSettings& simulationSettings*/, Real scalarFactor = 1.);

	//! set systemJacobian to zero and add jacobian (multiplied with factor) of ODE2RHS to systemJacobian in cSolver
	virtual void ComputeJacobianODE2RHS(MainSystem& mainSystem, Real scalarFactor_ODE2 = 1., Real scalarFactor_ODE2_t = 0., Real scalarFactor_ODE1 = 1.,
        Index computeLoadsJacobian = 0);

	//not needed:
	////! add jacobian of ODE2RHS_t (multiplied with factor) to systemJacobian in cSolver
	//virtual void ComputeJacobianODE2RHS_t(MainSystem& mainSystem/*, const SimulationSettings& simulationSettings*/, Real scalarFactor = 1.);

	//! add jacobian (multiplied with factor) of ODE1RHS to systemJacobian in cSolver
	virtual void ComputeJacobianODE1RHS(MainSystem& mainSystem, Real scalarFactor_ODE2 = 1., Real scalarFactor_ODE2_t = 0., Real scalarFactor_ODE1 = 1.);

	//! add jacobian of algebraic equations (multiplied with factor) to systemJacobian in cSolver; the scalarFactors are scaling the derivatives w.r.t. ODE2 coordinates and w.r.t. ODE2_t (velocity) coordinates; if velocityLevel == true, the constraints are evaluated at velocity level
	virtual void ComputeJacobianAE(MainSystem& mainSystem,
		Real scalarFactor_ODE2 = 1., Real scalarFactor_ODE2_t = 0., Real scalarFactor_ODE1 = 1., bool velocityLevel = false);

	//! compute the RHS of ODE2 equations in systemResidual in range(0,nODE2)
	virtual void ComputeODE2RHS(MainSystem& mainSystem/*, const SimulationSettings& simulationSettings*/);
	//! compute the RHS of ODE1 equations in systemResidual in range(nODE2,nODE2+nODE1)
	virtual void ComputeODE1RHS(MainSystem& mainSystem/*, const SimulationSettings& simulationSettings*/);
	//! compute the algebraic equations in systemResidual in range(nODE2+nODE1, nODE2+nODE1+nAE)
	virtual void ComputeAlgebraicEquations(MainSystem& mainSystem/*, const SimulationSettings& simulationSettings*/, bool velocityLevel = false);
	//! print function used in ostream operator (print is virtual and can thus be overloaded)
	virtual void Print(std::ostream& os) const
	{
		os << "MainSolverBase" << ":\n";
		os << "  cSolver = " << GetCSolver() << "\n";
		os << "  isInitialized = " << isInitialized << "\n";
		os << "  initializedSystemSizes = " << initializedSystemSizes << "\n";
		os << "\n";
	}

	friend std::ostream& operator<<(std::ostream& os, const MainSolverBase& object)
	{
		object.Print(os);
		return os;
	}

};

#endif
