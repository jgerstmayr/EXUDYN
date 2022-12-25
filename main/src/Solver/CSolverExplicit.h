/** ***********************************************************************************************
* @class		CSolverExplicitTimeInt
* @brief		This is the new implicit second order solver (incl. generalized alpha)
* @details		Details:
* 				- solves a dynamic system with constraints
*               - either use Newmark-based formulas with index-2 reduction, or use generalized-alpha with index 3 constraints
*				- step is solved by nonlinear iteration and Newton's method for accelerations
*				- step reduction is not implemented yet, as well as the initial accelerations are not computed accordingly
*
* @author		Gerstmayr Johannes
* @date			2021-01-21 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef CSOLVEREXPLICITTIMEINT__H
#define CSOLVEREXPLICITTIMEINT__H

#include "Solver/CSolverBase.h" 
#include "Linalg/ResizableVectorParallel.h"	 

static constexpr Index cSolverExplicitTimeIntMaxStages = 7; //for RK67

//! simple container for all data related to RK45 integration
class RKdata
{
public:

	//stage temporary data, Ki=f(t,gi)
	ResizableVectorParallel stageDerivODE2[cSolverExplicitTimeIntMaxStages]; //!< temporary k-vector for RK stages
	//ResizableVectorParallel stageDerivLieODE2[cSolverExplicitTimeIntMaxStages]; //!< temporary K-vector for Lie groups for RK stages
	ResizableVectorParallel stageDerivODE2_t[cSolverExplicitTimeIntMaxStages]; //!< temporary k-vector for RK stages
	ResizableVectorParallel stageDerivODE1[cSolverExplicitTimeIntMaxStages]; //!< temporary k-vector for RK stages
	ResizableVectorParallel solutionSecondApproxODE2; //second approximation for error estimator
	ResizableVectorParallel solutionSecondApproxODE2_t; //second approximation for error estimator
	ResizableVectorParallel solutionSecondApproxODE1; //second approximation for error estimator
	ResizableVectorParallel startOfStepODE2;   //temporary vector for stage computation and final evaluation
	ResizableVectorParallel startOfStepODE2_t; //temporary vector for stage computation and final evaluation
	ResizableVectorParallel startOfStepODE1;   //temporary vector for stage computation and final evaluation

	Matrix A;		//RK tableau, A in Butcher tableau
	Vector time;	//RK relative stage times, c in Butcher tableau
	Vector weight;	//rk tableau stage weights, b in Butcher tableau, for evaluation step
	Vector weightEE;	//rk tableau stage weights, b' in Butcher tableau, for error estimation
	Index orderMethod; //order of the method; for embedded methods with orders p and p+1, this variable holds p+1, the higher value of the order, e.g., 5 in DOPRI5
	bool hasStepSizeControl;
};

//! this is the new general implicit second order time integration solver
//! includes trapezoidal rule, Newmark and generalized alpha
class CSolverExplicitTimeInt: public CSolverBase
{
public: //made public for access via pybind
	static constexpr Index maxStages = cSolverExplicitTimeIntMaxStages;
	//copy of parameters from integration scheme (cannot be changed during integration!)
	DynamicSolverType dynamicSolverType; //must be explicit
	bool eliminateConstraints;
	ArrayIndex constrainedODE2Coordinates;

	bool hasConstantMassMatrix; //!< avoid recomputation of mass matrix
	bool computeMassMatrixInversePerBody;

	bool minStepSizeWarned; //!< set true, if already warned because of reaching minimum step due to error control

	Index nStages;	//!< number of active stages in explicit integrator

	RKdata rk;		//data for RungeKutta scheme / Butcher tableau

	//++++++++++++++++++++++++++++++
	//Lie groups:
	bool useLieGroupIntegration;
	ResizableArray<Index> lieGroupDataNodes; //filled with lie group node indices during initialization; ONLY if useLieGroupIntegration=true
	ResizableArray<Index> nonLieODE2Coordinates; //filled with ODE2 coordinates, for which no Lie group integration is used; ONLY if useLieGroupIntegration=true

public:

	//! return true, if static solver; needs to be overwritten in derived class
	virtual bool IsStaticSolver() const override { return false; }

	//! get solver name - needed for output file header and visualization window
	virtual const STDstring GetSolverName() const override 
	{ 
		return "explicit time integration (" + EXUstd::ToString(dynamicSolverType) + ")";

		//switch (dynamicSolverType)
		//{
		//case DynamicSolverType::ExplicitEuler: return "explicit Euler time integration"; break;
		//case DynamicSolverType::RK45: return "explicit RK45 time integration"; break;
		//default: return "ERROR: undefined explicit solver type";
		//}
	}

	//! pre-initialize for solver specific tasks; called at beginning of InitializeSolver, right after Solver data reset
	virtual void PreInitializeSolverSpecific(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	////! set/compute initial conditions (solver-specific!); called from InitializeSolver()
	//virtual void InitializeSolverInitialConditions(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! post-initialize for solver specific tasks; called at the end of InitializeSolver
	virtual void PostInitializeSolverSpecific(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! initialize all data,it,conv; called from InitializeSolver()
	virtual void InitializeSolverData(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! reduce step size (0 .. with given step size, 1..normal, 2..severe problems); return true, if reduction was successful
	virtual bool ReduceStepSize(CSystem& computationalSystem, const SimulationSettings& simulationSettings,
		Index severity, Real suggestedStepSize = -1.) override;

	//! increase step size if convergence is good; if suggestedStepSize == -1, a solver-specific factor will be used
	virtual void IncreaseStepSize(CSystem& computationalSystem, const SimulationSettings& simulationSettings,
		Real suggestedStepSize = -1.) override;

	//! return true, if solver supports automatic stepsize control
	virtual bool HasAutomaticStepSizeControl() const;

	//! update currentTime (and load factor); MUST be overwritten in special solver class
	virtual void UpdateCurrentTime(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! replace Newton of solver by explicit integration scheme!
	virtual bool Newton(CSystem& computationalSystem, const SimulationSettings& simulationSettings);

	//NOT NEEDED, because no Newton performed:
	////! compute residual for Newton method (e.g. static or time step); store result vector in systemResidual and return scalar residual
	//virtual Real ComputeNewtonResidual(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	////! compute update for currentState from newtonSolution (decrement from residual and jacobian)
	//virtual void ComputeNewtonUpdate(CSystem& computationalSystem, const SimulationSettings& simulationSettings, bool initial = true) override;

	////! compute jacobian for newton method of given solver method; store result in systemJacobian
	//virtual void ComputeNewtonJacobian(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++              HELPER FUNCTIONS FOR EXPLICIT INTEGRATOR                 +++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! helper function that computes q_tt = M^-1*ODE2RHS for given q_t and q; return true, if successful
	bool ComputeODE2Acceleration(CSystem& computationalSystem, const SimulationSettings& simulationSettings, Vector& ode2Rhs,
		Vector& ode2Acceleration, GeneralMatrix* massMatrix);

	//! helper function to compute Butcher tableau in rkData for given integrator and return number of stages; 
	//! set hasStepSizeControl=true in rkData, if integrator supports automaticStepSize control
	Index ComputeButcherTableau(DynamicSolverType dynamicSolverType, RKdata& rkData);

	//++++++++++++++++++++++++++++++
	//! precompute list of coordinates (constraints) that are eliminated
	void PrecomputeConstraintElimination(CSystem& computationalSystem, const SimulationSettings& simulationSettings);

	//! set all ODE2 coordinates given in cODE2 in solODE2 to zero; used for displacements and velocities
	void EliminateCoordinateConstraints(CSystem& computationalSystem, const ArrayIndex& cODE2, Vector& solODE2);

	//++++++++++++++++++++++++++++++
	//! precompute non-Lie group coordinates and Lie-group node indices
	void PrecomputeLieGroupStructures(CSystem& computationalSystem, const SimulationSettings& simulationSettings);

	//! update stage coordinates for regular ODE2 coordinates and Lie group nodes
	void UpdateODE2StageCoordinatesLieGroup(CSystem& computationalSystem, ResizableVectorParallel& solutionODE2,
		Real stepSize, Index i);

	//! compute displacement stage increment for standard coordinates (stageDerivODE2) and for Lie group nodes (stageDerivLieODE2)
	void LieGroupComputeKstage(CSystem& computationalSystem, const ResizableVectorParallel& solutionODE2_t,
		ResizableVectorParallel& stageDerivODE2, ResizableVectorParallel& stageDerivLieODE2, Real stepSize, Index i);

	//! compute final integration formula for Runge-Kutta method, including Lie group nodes; 
	//! also applicable to compute secondApproximation with different weights
	void LieGroupODE2StepEvaluation(CSystem& computationalSystem, ResizableVectorParallel& solutionODE2, Real stepSize, const Vector& weights);

};


#endif
