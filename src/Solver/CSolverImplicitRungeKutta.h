/** ***********************************************************************************************
* @class		CSolverImplicitRungeKutta
* @brief		The implicit Runge-Kutta solvers for index-3 constraints: TrapezoidalGGL, LobattoIIIA4, LobattoIIIC4GGL,
*				Radau3 (#3261)
* @details		Details:
*				- one algorithm for the four methods, a stiffly accurate implicit Runge-Kutta method given by its
*				  tableau (A, c) and the place of its constraints: 'GGL', c = 0 and the velocity constraints at each
*				  implicit stage with the positions moved onto the constraints by a multiplier mu; 'end', the two at
*				  the end of the step only, with a multiplier per implicit stage
*				- a component of CSolverImplicitSecondOrderTimeInt, which runs Step() in place of the Newton
*				  iteration of generalized-alpha when timeIntegration.solverType is one of the four; the step loop,
*				  the discontinuous iteration, the initial accelerations, the output and the restart are those of
*				  the implicit solver
*				- unknowns per implicit stage: the accelerations and the ODE1 velocities, the multipliers (and mu
*				  with GGL); the Newton iteration with the Jacobian of the system at the last stage, the term
*				  (C_q v)_q left out; the ODE2 rows scaled by h^2, the multipliers by 1/h^2
*				- velocity-level constraints and constraints whose equation is their multiplier (inactive ones):
*				  their equation at the implicit stages (GGL, with mu = 0), or at the last two stages (end)
*				- Lie group nodes: the rotation of a stage by the composition of the start of the step with the
*				  increment of the stage, as generalized-alpha does
*				- automatic step size with the error estimate of two steps (the Runge-Kutta methods) or of one step
*				  (TrapezoidalGGL), the PI controller of StepSizeController
*				- the Python version is exudyn.symbolics.advancedSolvers (SolveTrapezoidalGGL, SolveImplicitRungeKutta)
*
* @author		Gerstmayr Johannes
* @date			2026-11-05
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#ifndef CSOLVERIMPLICITRUNGEKUTTA__H
#define CSOLVERIMPLICITRUNGEKUTTA__H

#include "Linalg/LinearSolver.h"
#include "Solver/StepSizeController.h"

class CSystem;
class CSolverImplicitSecondOrderTimeInt;

//! the tableau of a method: s stages, A (s x s, row by row), c; ggl: the constraints at each implicit stage, else at the end
class ImplicitRungeKuttaMethod
{
public:
	static constexpr Index maxStages = 3;
	const char* name = "";
	Index s = 0;
	Real A[maxStages][maxStages] = {};
	Real c[maxStages] = {};
	bool ggl = false;
	Index order = 0;
};

//! the triplets of one block of a system matrix, in local indices
class IRKBlock
{
public:
	std::vector<Index> rows, columns;
	std::vector<Real> values;
	void Clear() { rows.clear(); columns.clear(); values.clear(); }
	void Add(Index i, Index j, Real v) { rows.push_back(i); columns.push_back(j); values.push_back(v); }
};

class CSolverImplicitRungeKutta
{
public:
	bool active = false;
	ImplicitRungeKuttaMethod method;
	bool errorEstimateTwoSteps = false;
	StepSizeController controller;
	Real nextStepSize = -1.;			//!< the step the error control proposes for the next step; -1: none
	bool rejectedByError = false;		//!< the last Step was rejected by the error estimate, the step size already set
	Index rejectedStepsError = 0;
	bool minStepSizeWarned = false;

	//the method
	ArrayIndex implicitStages;			//!< the stages with a row of A not zero
	Real A2[ImplicitRungeKuttaMethod::maxStages][ImplicitRungeKuttaMethod::maxStages];
	Index nUnknowns = 0;
	ArrayIndex offA, offY, offZ, offMu;	//!< per implicit stage the offsets of its unknowns; offMu only with GGL

	//the constraints
	ArrayIndex velocityLevelRow;		//!< 1 for the rows of velocity-level constraints
	ArrayIndex specialRow;				//!< 1 for the rows of velocity-level constraints and of equations of the multiplier
	ArrayIndex lieGroupNodes;

	//the stages
	Vector stageA[ImplicitRungeKuttaMethod::maxStages], stageYd[ImplicitRungeKuttaMethod::maxStages];
	Vector stageZ[ImplicitRungeKuttaMethod::maxStages], stageMu[ImplicitRungeKuttaMethod::maxStages];
	Vector Q[ImplicitRungeKuttaMethod::maxStages], V[ImplicitRungeKuttaMethod::maxStages], Y[ImplicitRungeKuttaMethod::maxStages];
	Vector qIterate[ImplicitRungeKuttaMethod::maxStages];
	Vector q0, v0, a0, y0, yd0, z0;
	Vector residual, increment, temp2, tempF, temp1, tempAE, tempAEv, tempAEm;
	Vector rowScale;					//!< the scaling factors of the rows of the residual (h^2, h, 1)

	//the previous accepted step, for the error estimate of two steps
	bool hasPrevious = false;
	Real previousT0 = 0., previousH = 0.;
	Vector previousV[ImplicitRungeKuttaMethod::maxStages], previousA[ImplicitRungeKuttaMethod::maxStages];
	Vector candidateV[ImplicitRungeKuttaMethod::maxStages], candidateA[ImplicitRungeKuttaMethod::maxStages];
	Real candidateT0 = 0., candidateH = 0.;
	//TrapezoidalGGL: the mean accelerations (v1 - v0)/h of the last two accepted steps, for the velocity error (#3287)
	Vector meanA1, meanA2, candidateMeanA;
	Real meanH1 = 0., meanH2 = 0.;
	Index numberOfMeans = 0;

	//the matrices
	GeneralMatrixEXUdense jqDense, jvDense, stageDense;
	GeneralMatrixEigenSparse jqSparse, jvSparse, stageSparse;
	GeneralMatrix* jq = nullptr;
	GeneralMatrix* jv = nullptr;
	GeneralMatrix* stageMatrix = nullptr;
	IRKBlock blockK, blockKy, blockCt, blockGq, blockGy, blockCyt, blockCq, blockCy, blockCaa, blockD, blockGv, blockCv, blockM;
	std::vector<Index> tripletRows, tripletColumns;
	std::vector<Real> tripletValues;
	//the work arrays of the stage Jacobian, kept between steps (#3287)
	std::vector<Index> entryRows, entryColumns;		//!< the entries of a system Jacobian
	std::vector<Real> entryValues;
	SparseTripletVector stageTriplets;
	std::vector<Index> rightStart, rightColumns;	//!< AddProduct: the right block by rows (compressed)
	std::vector<Real> rightValues;
	std::vector<std::pair<int64_t, Real>> productEntries; //!< AddProduct: (row * nRight + column, value), sorted

	//! the method of a solver type, false if it is none of the four
	static bool GetMethod(DynamicSolverType type, ImplicitRungeKuttaMethod& method);

	//! set up the method, the unknowns and the matrices; refuses what the solver cannot do
	void Initialize(CSolverImplicitSecondOrderTimeInt& solver, CSystem& computationalSystem, const SimulationSettings& simulationSettings);

	//! the step from the start of the step state to it.currentTime; false if Newton did not converge or the error
	//! estimate rejected the step (rejectedByError, the new step size in it.currentStepSize)
	bool Step(CSolverImplicitSecondOrderTimeInt& solver, CSystem& computationalSystem, const SimulationSettings& simulationSettings);

	//! the step is accepted (after the discontinuous iteration): its stages become the previous step
	void AcceptStep();

private:
	void SetStageState(CSystem& computationalSystem, Index i, Real t0, Real h, bool iterate = false);
	void ComputeStages(CSolverImplicitSecondOrderTimeInt& solver, CSystem& computationalSystem, Real t0, Real h);
	Real ComputeResidual(CSolverImplicitSecondOrderTimeInt& solver, CSystem& computationalSystem, const SimulationSettings& simulationSettings, Real t0, Real h);
	void ComputeJacobian(CSolverImplicitSecondOrderTimeInt& solver, CSystem& computationalSystem, const SimulationSettings& simulationSettings, Real t0, Real h);
	void ComputeRowScale(const CSolverImplicitSecondOrderTimeInt& solver, Real h);
	void AddBlock(const IRKBlock& block, Index rowOffset, Index columnOffset, Real factor, Index rowSelect = -1);
	void AddProduct(const IRKBlock& left, const IRKBlock& right, Index rowOffset, Index columnOffset, Real factor, Index nRight);
	Real ErrorEstimate(CSolverImplicitSecondOrderTimeInt& solver, const SimulationSettings& simulationSettings, Real t0, Real h, Real& order);
};

#endif
