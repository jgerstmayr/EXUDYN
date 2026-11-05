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
#ifndef CSOLVERIMPLICITSECONDORDERTIMEINT__H
#define CSOLVERIMPLICITSECONDORDERTIMEINT__H

#include "Solver/CSolverBase.h" 
#include "Solver/CSolverImplicitRungeKutta.h" //TrapezoidalGGL, LobattoIIIA4, LobattoIIIC4GGL, Radau3 (#3261)

//! this is the new general implicit second order time integration solver
class CSolverImplicitSecondOrderTimeInt : public CSolverBase
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
	bool hasConstantMassMatrix; //!< avoid recomputation of mass matrix, if constant

	bool useLieGroupIntegration;			//!< only used internally to speed up in case of no Lie group nodes
	bool constraintDriftWarned;				//!< the drift of the constraints has exceeded the level of the warning in this run (#565)
	//++++++++++++++++++++++++++++++
	//Lie groups:
	//ResizableArray<Index> lieGroupDataNodes;			//filled with Lie group node indices with data coordinates during initialization; ONLY if useLieGroupIntegration=true
	ResizableArray<Index> lieGroupDirectUpdateNodes;	//filled with Lie group node indices with direct updates during initialization; ONLY if useLieGroupIntegration=true
	//UNUSED: //ResizableArray<Index> nonLieODE2Coordinates;		//filled with ODE2 coordinates, for which no Lie group integration is used; ONLY if useLieGroupIntegration=true
	ResizableVectorParallel lieGroupDirectUpdateNewtonSolution; //this is the previous newton solution in case of direct update
	CSolverImplicitRungeKutta rungeKutta;	//!< the implicit Runge-Kutta methods; active if timeIntegration.solverType is one of them (#3261)
public:

	//! return true, if static solver; needs to be overwritten in derived class
	virtual bool IsStaticSolver() const override { return false; }

	//! get solver name - needed for output file header and visualization window
	virtual const STDstring GetSolverName() const override
	{
		if (rungeKutta.active) { return STDstring("implicit Runge-Kutta time integration (") + rungeKutta.method.name + ")"; }
		return "implicit second order time integration";
	}

	//! the implicit Runge-Kutta methods control their step size (#3261)
	virtual bool HasAutomaticStepSizeControl() const override { return rungeKutta.active; }
	virtual Real GetNextStepSize() const override { return rungeKutta.active ? rungeKutta.nextStepSize : -1.; }
	virtual void SetNextStepSize(Real stepSize) override { if (rungeKutta.active) { rungeKutta.nextStepSize = stepSize; } }

	//! the Newton iteration of generalized-alpha, or the step of an implicit Runge-Kutta method (#3261)
	virtual bool Newton(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! reduce step size (0 .. with given step size, 1..normal, 2..severe problems); return true, if reduction was successful
	virtual bool ReduceStepSize(CSystem& computationalSystem, const SimulationSettings& simulationSettings,
		Index severity, Real suggestedStepSize = -1.) override;

	//! increase step size if convergence is good; if suggestedStepSize == -1, a solver-specific factor will be used
	virtual void IncreaseStepSize(CSystem& computationalSystem, const SimulationSettings& simulationSettings,
		Real suggestedStepSize = -1.) override;

	//! pre-initialize for solver specific tasks; called at beginning of InitializeSolver, right after Solver data reset
	virtual void PreInitializeSolverSpecific(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! post-initialize for solver specific tasks; called at the end of InitializeSolver
	virtual void PostInitializeSolverSpecific(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! set/compute initial conditions (solver-specific!); called from InitializeSolver()
	virtual void InitializeSolverInitialConditions(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! assemble and factorize the matrix [M C_q^T; C_q 0] of the initial state, with a unit diagonal for ODE1 coordinates;
	//! used for the initial accelerations and the projection of the initial velocities; returns the result of FactorizeNew
	//! (-1: success) (#2234)
	Index FactorizeInitialSystemJacobian(CSystem& computationalSystem, const SimulationSettings& simulationSettings);

	//! check the initial positions and velocities against the constraints, warn and, if requested, project the velocities
	//! onto the constraints (#2234); returns true if it left the matrix of FactorizeInitialSystemJacobian factorized
	bool CheckInitialConstraints(CSystem& computationalSystem, const SimulationSettings& simulationSettings);

	//! the largest absolute entry of v, and its index
	Real MaxAbsAndIndex(const Vector& v, Index& index) const;

	//! the item whose algebraic equation aeIndex is, as "object 5" or "node 3" (#2234)
	STDstring AlgebraicEquationItem(const CSystemData& systemData, Index aeIndex) const;

	//! the residual of the algebraic equations at position or velocity level and its largest absolute value, with the
	//! equation in index; at position level, the equations of velocity-level constraints are zero, as they give their
	//! velocity equations at both levels (#2234, #565)
	Real ConstraintResidual(CSystem& computationalSystem, bool velocityLevel, Vector& residual, Index& index);

	//! the drift of the position-level constraints, every constraintDriftCheckPeriod steps: the largest value in the output,
	//! a warning once above 0.001 constraintDriftTolerance, an exception above constraintDriftTolerance (#565)
	void CheckConstraintDrift(CSystem& computationalSystem, const SimulationSettings& simulationSettings);

	//! the check of the drift, then the output of the step
	virtual void FinishStep(CSystem& computationalSystem, const SimulationSettings& simulationSettings, bool writeSolution = true) override;

	//! the largest drift of the run, if it was warned about, then the finalization of the base class
	virtual void FinalizeSolver(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! update currentTime (and load factor); MUST be overwritten in special solver class
	virtual void UpdateCurrentTime(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! compute residual for Newton method (e.g. static or time step); store result vector in systemResidual and return scalar residual
	virtual Real ComputeNewtonResidual(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! compute update for currentState from newtonSolution (decrement from residual and jacobian)
	virtual void ComputeNewtonUpdate(CSystem& computationalSystem, const SimulationSettings& simulationSettings, bool initial = false) override;

	//! compute jacobian for newton method of given solver method; store result in systemJacobian
	virtual void ComputeNewtonJacobian(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! finalize algorithmic accelerations in generalized alpha method
	virtual void FinalizeNewton(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	////! apply composition rule to currentODE2 o incrementODE2 for given set of nodes
	//virtual void UpdateDataCoordinatesLieGroupIntegrator(CSystem& computationalSystem, const ArrayIndex& lieGroupNodes, 
	//	const ResizableVectorParallel& currentODE2, const ResizableVectorParallel& incrementODE2, ResizableVectorParallel& compositionODE2);

	//! apply composition rule to currentODE2 o incrementODE2 for given set of nodes
	virtual void CompositionRuleCoordinatesLieGroupIntegrator(CSystem& computationalSystem, const ArrayIndex& lieGroupNodes, 
		const ResizableVectorParallel& currentODE2, const ResizableVectorParallel& incrementODE2, ResizableVectorParallel& compositionODE2);

	//! set Lie group data coordinates 0, set Lie group previous Newton increment 0
	virtual void ResetCoordinatesLieGroupNodes(CSystem& computationalSystem, ResizableVectorParallel& previousNewtonSolution);

	//! set solutionODE2 coordinates of Lie group direct update nodes to previous Newton solution
	virtual void SetPreviousNewtonSolutionLieGroupDirectUpdateNodes(CSystem& computationalSystem, 
		const ResizableVectorParallel& previousNewtonSolution, ResizableVectorParallel& solutionODE2);

	//! apply tangent operator T to matrix A: A = A*T; T defined for Lie group nodes
	virtual void LieGroupNodesApplyTangentOperator(CSystem& computationalSystem, 
		ResizableVectorParallel& globalIncrementalSolutionODE2, const ArrayIndex& lieGroupNodes, Index startRow, GeneralMatrix& A);
};

class MainSolverImplicitSecondOrder;
typedef std::function<bool(MainSolverImplicitSecondOrder&, MainSystem&, const SimulationSettings&)> MainSolverImplicitSecondOrderUserFunction;
typedef std::function<bool(MainSolverImplicitSecondOrder&, MainSystem&, const SimulationSettings&, bool)> MainSolverImplicitSecondOrderUserFunctionBool;
typedef std::function<Real(MainSolverImplicitSecondOrder&, MainSystem&, const SimulationSettings&)> MainSolverImplicitSecondOrderUserFunctionReal;

//! second order integrator with user functions capability
class CSolverImplicitSecondOrderTimeIntUserFunction : public CSolverImplicitSecondOrderTimeInt
{
private:
	MainSolverImplicitSecondOrderUserFunction userFunctionPreInitializeSolverSpecific;//!< User function to override Newton()
	MainSolverImplicitSecondOrderUserFunction userFunctionInitializeSolverInitialConditions;//!< User function to override Newton()
	MainSolverImplicitSecondOrderUserFunction userFunctionPostInitializeSolverSpecific;//!< User function to override Newton()
	MainSolverImplicitSecondOrderUserFunction userFunctionUpdateCurrentTime;//!< User function to override UpdateCurrentTime()
	MainSolverImplicitSecondOrderUserFunction userFunctionInitializeStep;//!< User function to override InitializeStep()
	MainSolverImplicitSecondOrderUserFunction userFunctionFinishStep;//!< User function to override FinishStep()
	MainSolverImplicitSecondOrderUserFunction userFunctionDiscontinuousIteration;//!< User function to override DiscontinuousIteration()
	MainSolverImplicitSecondOrderUserFunction userFunctionNewton;//!< User function to override Newton()
	MainSolverImplicitSecondOrderUserFunctionBool userFunctionComputeNewtonUpdate;//!< User function to override ComputeNewtonUpdate()
	MainSolverImplicitSecondOrderUserFunctionReal userFunctionComputeNewtonResidual;//!< User function to override ComputeNewtonResidual()
	MainSolverImplicitSecondOrderUserFunction userFunctionComputeNewtonJacobian;//!< User function to override ComputeNewtonJacobian()
	MainSolverImplicitSecondOrderUserFunctionReal userFunctionPostNewton;//!< User function to override PostNewton()

	MainSolverImplicitSecondOrder* mainSolver; //!< pointer to main solver, needed in user function; this is dangerous, but cannot be avoided!
	MainSystem* mainSystem; //!< pointer to main solver, needed in user function; this is dangerous, but cannot be avoided!

public:
	CSolverImplicitSecondOrderTimeIntUserFunction()
	{
		userFunctionPreInitializeSolverSpecific = 0;
		userFunctionInitializeSolverInitialConditions = 0;
		userFunctionPostInitializeSolverSpecific = 0;

		userFunctionUpdateCurrentTime = 0;
		userFunctionInitializeStep = 0;
		userFunctionFinishStep = 0;
		userFunctionDiscontinuousIteration = 0;
		userFunctionNewton = 0;
		userFunctionComputeNewtonUpdate = 0;
		userFunctionComputeNewtonResidual = 0;
		userFunctionComputeNewtonJacobian = 0;
		userFunctionPostNewton = 0;

		mainSolver = nullptr;
		mainSystem = nullptr;
	}

	virtual void SetUserFunctionUpdateCurrentTime(MainSolverImplicitSecondOrder* mainSolverInit, MainSystem* mainSystemInit, const MainSolverImplicitSecondOrderUserFunction& uf)
	{
		mainSolver = mainSolverInit; mainSystem = mainSystemInit; userFunctionUpdateCurrentTime = uf;
	}
	virtual void SetUserFunctionInitializeStep(MainSolverImplicitSecondOrder* mainSolverInit, MainSystem* mainSystemInit, const MainSolverImplicitSecondOrderUserFunction& uf)
	{
		mainSolver = mainSolverInit; mainSystem = mainSystemInit; userFunctionInitializeStep = uf;
	}
	virtual void SetUserFunctionFinishStep(MainSolverImplicitSecondOrder* mainSolverInit, MainSystem* mainSystemInit, const MainSolverImplicitSecondOrderUserFunction& uf)
	{
		mainSolver = mainSolverInit; mainSystem = mainSystemInit; userFunctionFinishStep = uf;
	}
	virtual void SetUserFunctionDiscontinuousIteration(MainSolverImplicitSecondOrder* mainSolverInit, MainSystem* mainSystemInit, const MainSolverImplicitSecondOrderUserFunction& uf)
	{
		mainSolver = mainSolverInit; mainSystem = mainSystemInit; userFunctionDiscontinuousIteration = uf;
	}
	virtual void SetUserFunctionNewton(MainSolverImplicitSecondOrder* mainSolverInit, MainSystem* mainSystemInit, const MainSolverImplicitSecondOrderUserFunction& uf)
	{
		mainSolver = mainSolverInit; mainSystem = mainSystemInit; userFunctionNewton = uf;
	}
	virtual void SetUserFunctionComputeNewtonUpdate(MainSolverImplicitSecondOrder* mainSolverInit, MainSystem* mainSystemInit, const MainSolverImplicitSecondOrderUserFunctionBool& uf)
	{
		mainSolver = mainSolverInit; mainSystem = mainSystemInit; userFunctionComputeNewtonUpdate = uf;
	}
	virtual void SetUserFunctionComputeNewtonResidual(MainSolverImplicitSecondOrder* mainSolverInit, MainSystem* mainSystemInit, const MainSolverImplicitSecondOrderUserFunctionReal& uf)
	{
		mainSolver = mainSolverInit; mainSystem = mainSystemInit; userFunctionComputeNewtonResidual = uf;
	}
	virtual void SetUserFunctionComputeNewtonJacobian(MainSolverImplicitSecondOrder* mainSolverInit, MainSystem* mainSystemInit, const MainSolverImplicitSecondOrderUserFunction& uf)
	{
		mainSolver = mainSolverInit; mainSystem = mainSystemInit; userFunctionComputeNewtonJacobian = uf;
	}
	virtual void SetUserFunctionPostNewton(MainSolverImplicitSecondOrder* mainSolverInit, MainSystem* mainSystemInit, const MainSolverImplicitSecondOrderUserFunctionReal& uf)
	{
		mainSolver = mainSolverInit; mainSystem = mainSystemInit; userFunctionPostNewton = uf;
	}

	//! update currentTime (and load factor); MUST be overwritten in special solver class
	virtual void UpdateCurrentTime(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! initialize static step / time step; python-functions; do some outputs, checks, etc.
	virtual void InitializeStep(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! finish static step / time step; write output of results to file; writeSolution will switch on/off writing solution and sensor outputs
	virtual void FinishStep(CSystem& computationalSystem, const SimulationSettings& simulationSettings, bool writeSolution = true) override;

	//! perform discontinuousIteration for static step / time step; CALLS ComputeNewtonResidual 
	virtual bool DiscontinuousIteration(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! perform Newton method for given solver method
	virtual bool Newton(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! compute residual for Newton method (e.g. static or time step); store result vector in systemResidual and return scalar residual
	virtual Real ComputeNewtonResidual(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! compute update for currentState from newtonSolution (decrement from residual and jacobian)
	virtual void ComputeNewtonUpdate(CSystem& computationalSystem, const SimulationSettings& simulationSettings, bool initial = false) override;

	//! compute jacobian for newton method of given solver method; store result in systemJacobian
	virtual void ComputeNewtonJacobian(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;

	//! perform PostNewton method for given solver method
	virtual Real PostNewton(CSystem& computationalSystem, const SimulationSettings& simulationSettings) override;
};



#endif
