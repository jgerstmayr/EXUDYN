/** ***********************************************************************************************
* @class		CSolverImplicitSecondOrderNew
* @brief		This is the new (2021) implicit second order solver 
* @details		Details:
* 				- solves a dynamic system with constraints
*               - either use Newmark-based formulas with index-2 reduction, or use generalized-alpha with index 3 constraints
*               - in the final version it should include GGL stabilization, Brüls/Arnold implementation of gen-alpha and Lie group integration
*				- step is solved by nonlinear iteration and Newton's method for accelerations
*
* @author		Gerstmayr Johannes
* @date			2021-01-27 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef CSOLVERIMPLICITSECONDORDERTIMEINTNEW__H
#define CSOLVERIMPLICITSECONDORDERTIMEINTNEW__H

#include "Solver/CSolverBase.h" 

//! this is the new general implicit second order time integration solver
class CSolverImplicitSecondOrderTimeIntNew : public CSolverImplicitSecondOrderTimeIntUserFunction
{
public: //made public for access via pybind
	//copy of parameters from integration scheme (cannot be changed during integration!)
	Real newmarkBeta;
	Real newmarkGamma;
	Real alphaM;
	Real alphaF;
	Real spectralRadius;
	Real factJacAlgorithmic;

	bool useScaling; //scaling for ODE2 and AE part to ensure good conditioning of Jacobian


	//bool useIndex2Constraints; ==> directly linked to simulationSettings
public:

	//! return true, if static solver; needs to be overwritten in derived class
	virtual bool IsStaticSolver() const override { return false; }

	//! get solver name - needed for output file header and visualization window
	virtual const STDstring GetSolverName() const override { return "implicit second order time integration"; }

	//! reduce step size (0 .. with given step size, 1..normal, 2..severe problems); return true, if reduction was successful
	virtual bool ReduceStepSize(CSystem& computationalSystem, const SimulationSettings& simulationSettings,
		Index severity, Real suggestedStepSize = -1.) override;

	//! increase step size if convergence is good; if suggestedStepSize == -1, a solver-specific factor will be used
	virtual void IncreaseStepSize(CSystem& computationalSystem, const SimulationSettings& simulationSettings,
		Real suggestedStepSize = -1.) override
	{
		it.currentStepSize = EXUstd::Minimum(it.maxStepSize, 2.*it.currentStepSize);
	}

	//! pre-initialize for solver specific tasks; called at beginning of InitializeSolver, right after Solver data reset
	virtual void PreInitializeSolverSpecific(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! post-initialize for solver specific tasks; called at the end of InitializeSolver
	virtual void PostInitializeSolverSpecific(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! set/compute initial conditions (solver-specific!); called from InitializeSolver()
	virtual void InitializeSolverInitialConditions(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! update currentTime (and load factor); MUST be overwritten in special solver class
	virtual void UpdateCurrentTime(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! compute residual for Newton method (e.g. static or time step); store result vector in systemResidual and return scalar residual
	virtual Real ComputeNewtonResidual(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! compute update for currentState from newtonSolution (decrement from residual and jacobian)
	virtual void ComputeNewtonUpdate(CSystem& computationalSystem, const SimulationSettings& simulationSettings, bool initial = false) override;

	//! compute jacobian for newton method of given solver method; store result in systemJacobian
	virtual void ComputeNewtonJacobian(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! call Newton from CSolverBase and finally update algorithmic accelerations
	virtual bool Newton(CSystem& computationalSystem, const SimulationSettings& simulationSettings);


};

#endif
