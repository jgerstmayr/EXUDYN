/** ***********************************************************************************************
* @class		CSolverStatic
* @brief		This is the new solver base class for static and time integration solution
* @details		Details:
*				Solver uses load steps, discontinuous iterations and Newton's method to solve nonlinear static problems based 
*				on the multibody system given; 
*				NOTE: system may not be kinematic, otherwise it cannot be solved statically
*
*
* @author		Gerstmayr Johannes
* @date			2019-12-16 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef CSOLVERSTATIC__H
#define CSOLVERSTATIC__H

#include "Solver/CSolverBase.h" 

//! this is the new static solver
class CSolverStatic : public CSolverBase
{
public:
	//==> now all parameters directly linked to simulationSettings! copy of parameters from StaticSolverSettings (may not be changed during solution)
	//Real loadStepDuration;                          //!< quasi-time for all load steps (added to current time in load steps)
	//bool loadStepGeometric;                         //!< if loadStepGeometric=false, the load steps are incremental (arithmetic series, e.g. 0.1,0.2,0.3,...); if true, the load steps are increased in a geometric series, e.g. for \f$n=8\f$ numberOfLoadSteps and \f$d = 1000\f$ loadStepGeometricRange, it follows: \f$1000^{1/8}/1000=0.00237\f$, \f$1000^{2/8}/1000=0.00562\f$, \f$1000^{3/8}/1000=0.0133\f$, ..., \f$1000^{7/8}/1000=0.422\f$, \f$1000^{8/8}/1000=1\f$
	//Real loadStepGeometricRange;                    //!< if loadStepGeometric=true, the load steps are increased in a geometric series, see loadStepGeometric
	//Real stabilizerODE2term;                        //!< add mass-proportional stabilizer term in ODE2 part of jacobian for stabilization (scaled ), e.g. of badly conditioned problems; the diagnoal terms are scaled with \f$stabilizer = (1-loadStepFactor^2)\f$, and go to zero at the end of all load steps: \f$loadStepFactor=1\f$ -> \f$stabilizer = 0\f$
	//Index verboseMode;                              //!< 0 ... no output, 1 ... show errors and load steps, 2 ... show short Newton step information (error), 3 ... show also solution vector, 4 ... show also jacobian, 5 ... show also Jacobian inverse
	//Index verboseModeFile;                          //!< same behaviour as verboseMode, but outputs all solver information to file

	Real loadStepGeometricFactor;					//!< //multiplicative load step factor

public:

	//! return true, if static solver; needs to be overwritten in derived class
	virtual bool IsStaticSolver() const override { return true; }

	//! get solver name - needed for output file header and visualization window
	virtual const STDstring GetSolverName() const override { return "nonlinear static solver"; }

	virtual Real ComputeLoadFactor(const SimulationSettings& simulationSettings) const override 
	{ 
		return (it.currentTime - it.startTime) / simulationSettings.staticSolver.loadStepDuration; 
	}

	//! reduce step size (1..normal, 2..severe problems); return true, if reduction was successful
	virtual bool ReduceStepSize(CSystem& computationalSystem, const SimulationSettings& simulationSettings, 
		Index severity, Real suggestedStepSize = -1.) override;

	//! increase step size if convergence is good
	virtual void IncreaseStepSize(CSystem& computationalSystem, const SimulationSettings& simulationSettings,
		Real suggestedStepSize = -1.) override
	{
		it.currentStepSize = EXUstd::Minimum(it.maxStepSize, simulationSettings.staticSolver.adaptiveStepIncrease*it.currentStepSize);
	}

	//! update currentTime (and load factor); MUST be overwritten in special solver class
	virtual void UpdateCurrentTime(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	////! pre-initialize for solver specific tasks; called at beginning of InitializeSolver, right after Solver data reset
	//virtual void PreInitializeSolverSpecific(CSystem& computationalSystem, const SimulationSettings& simulationSettings);

	//! post-initialize for solver specific tasks; called at the end of InitializeSolver
	virtual void PostInitializeSolverSpecific(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! compute residual for Newton method (e.g. static or time step); store result vector in systemResidual and return scalar residual
	virtual Real ComputeNewtonResidual(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! compute update for currentState from newtonSolution (decrement from residual and jacobian)
	virtual void ComputeNewtonUpdate(CSystem& computationalSystem, const SimulationSettings& simulationSettings, bool initial = false) override;

	//! compute jacobian for newton method of given solver method; store result in systemJacobian
	virtual void ComputeNewtonJacobian(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

};

#endif
