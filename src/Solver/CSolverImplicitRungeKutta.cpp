/** ***********************************************************************************************
* @brief		Implementation of CSolverImplicitRungeKutta, the implicit Runge-Kutta solvers TrapezoidalGGL,
*				LobattoIIIA4, LobattoIIIC4GGL, Radau3 (#3261) and Radau5 (#3318); see the header
*
* @author		Gerstmayr Johannes
* @date			2026-11-05
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */

#include <algorithm>
#include <map>
#include "Utilities/ExceptionsTemplates.h"
#include "Linalg/BasicLinalg.h"
#include "Linalg/RigidBodyMath.h"
#include "Main/CSystem.h"
#include "Solver/CSolverImplicitSecondOrder.h"

//! the five methods; the Python version is _rungeKuttaTableaus and implicitRungeKuttaMethods of exudyn.symbolics.advancedSolvers
bool CSolverImplicitRungeKutta::GetMethod(DynamicSolverType type, ImplicitRungeKuttaMethod& m)
{
	m = ImplicitRungeKuttaMethod();
	if (type == DynamicSolverType::TrapezoidalGGL)
	{
		m.name = "TrapezoidalGGL"; m.s = 2; m.ggl = true; m.order = 2;
		m.A[1][0] = 0.5; m.A[1][1] = 0.5;
		m.c[0] = 0.; m.c[1] = 1.;
	}
	else if (type == DynamicSolverType::LobattoIIIA4 || type == DynamicSolverType::LobattoIIIC4GGL)
	{
		m.s = 3; m.order = 4;
		m.c[0] = 0.; m.c[1] = 0.5; m.c[2] = 1.;
		const Real A3[3][3] = { {0., 0., 0.}, {5. / 24., 1. / 3., -1. / 24.}, {1. / 6., 2. / 3., 1. / 6.} };
		const Real C3[3][3] = { {1. / 6., -1. / 3., 1. / 6.}, {1. / 6., 5. / 12., -1. / 12.}, {1. / 6., 2. / 3., 1. / 6.} };
		m.ggl = (type == DynamicSolverType::LobattoIIIC4GGL);
		m.name = m.ggl ? "LobattoIIIC4GGL" : "LobattoIIIA4";
		for (Index i = 0; i < 3; i++) { for (Index j = 0; j < 3; j++) { m.A[i][j] = m.ggl ? C3[i][j] : A3[i][j]; } }
	}
	else if (type == DynamicSolverType::Radau3)
	{
		m.name = "Radau3"; m.s = 2; m.order = 3;
		m.A[0][0] = 5. / 12.; m.A[0][1] = -1. / 12.; m.A[1][0] = 0.75; m.A[1][1] = 0.25;
		m.c[0] = 1. / 3.; m.c[1] = 1.;
	}
	else if (type == DynamicSolverType::Radau5) //Radau IIA with 3 stages (#3318)
	{
		m.name = "Radau5"; m.s = 3; m.order = 5;
		const Real r6 = sqrt(6.);
		const Real A3[3][3] = { {(88. - 7. * r6) / 360., (296. - 169. * r6) / 1800., (-2. + 3. * r6) / 225.},
			{(296. + 169. * r6) / 1800., (88. + 7. * r6) / 360., (-2. - 3. * r6) / 225.},
			{(16. - r6) / 36., (16. + r6) / 36., 1. / 9.} };
		for (Index i = 0; i < 3; i++) { for (Index j = 0; j < 3; j++) { m.A[i][j] = A3[i][j]; } }
		m.c[0] = (4. - r6) / 10.; m.c[1] = (4. + r6) / 10.; m.c[2] = 1.;
	}
	else { return false; }
	return true;
}

void CSolverImplicitRungeKutta::Initialize(CSolverImplicitSecondOrderTimeInt& solver, CSystem& computationalSystem, const SimulationSettings& simulationSettings)
{
	const TimeIntegrationSettings& timeint = simulationSettings.timeIntegration;
	active = GetMethod(timeint.solverType, method);
	if (!active) { return; }
	const Index s = method.s;
	//the error estimate of the automatic step size (#3287)
	if (timeint.errorEstimate == "default") { errorEstimateTwoSteps = (timeint.solverType != DynamicSolverType::TrapezoidalGGL); }
	else if (timeint.errorEstimate == "twoSteps") { errorEstimateTwoSteps = true; }
	else if (timeint.errorEstimate == "oneStep") { errorEstimateTwoSteps = false; }
	else
	{
		PyError(STDstring(method.name) + ": timeIntegration.errorEstimate must be 'default', 'oneStep' or 'twoSteps', not '"
			+ timeint.errorEstimate + "'", PyErrorType::valueError);
	}
	for (Index i = 0; i < s; i++)
	{
		for (Index j = 0; j < s; j++)
		{
			A2[i][j] = 0.;
			for (Index k = 0; k < s; k++) { A2[i][j] += method.A[i][k] * method.A[k][j]; }
		}
	}
	implicitStages.SetNumberOfItems(0);
	for (Index i = 0; i < s; i++)
	{
		bool isImplicit = false;
		for (Index j = 0; j < s; j++) { if (method.A[i][j] != 0.) { isImplicit = true; } }
		if (isImplicit) { implicitStages.Append(i); }
	}

	const Index n2 = solver.data.nODE2, n1 = solver.data.nODE1, na = solver.data.nAE;
	Index nImplicit = implicitStages.NumberOfItems();
	offA.SetNumberOfItems(nImplicit); offY.SetNumberOfItems(nImplicit); offZ.SetNumberOfItems(nImplicit); offMu.SetNumberOfItems(nImplicit);
	Index index = 0;
	for (Index k = 0; k < nImplicit; k++)
	{
		offA[k] = index; offY[k] = index + n2;
		index += n2 + n1;
		if (method.ggl) { offZ[k] = index; offMu[k] = index + na; index += 2 * na; }
	}
	if (!method.ggl) { for (Index k = 0; k < nImplicit; k++) { offZ[k] = index; index += na; } }
	nUnknowns = index;

	//the rows of velocity-level constraints
	CSystemData& systemData = computationalSystem.GetSystemData();
	velocityLevelRow.SetNumberOfItems(na);
	velocityLevelRow.SetAll(0);
	for (Index i = 0; i < systemData.GetCObjects().NumberOfItems(); i++)
	{
		const CObject* object = systemData.GetCObjects()[i];
		if (((Index)object->GetType() & (Index)CObjectType::Constraint) && ((const CObjectConstraint*)object)->UsesVelocityLevel())
		{
			for (Index row : systemData.GetLocalToGlobalAE()[i]) { velocityLevelRow[row] = 1; }
		}
	}
	specialRow.CopyFrom(velocityLevelRow);

	//Lie group nodes: their rotation composed with the increment of a stage
	lieGroupNodes.SetNumberOfItems(0);
	for (Index i = 0; i < systemData.GetCNodes().NumberOfItems(); i++)
	{
		if (EXUstd::IsOfType(systemData.GetCNodes()[i]->GetType(), Node::LieGroupWithDirectUpdate)) { lieGroupNodes.Append(i); }
	}

	for (Index i = 0; i < s; i++)
	{
		stageA[i].SetNumberOfItems(n2); stageYd[i].SetNumberOfItems(n1); stageZ[i].SetNumberOfItems(na); stageMu[i].SetNumberOfItems(na);
		Q[i].SetNumberOfItems(n2); V[i].SetNumberOfItems(n2); Y[i].SetNumberOfItems(n1); qIterate[i].SetNumberOfItems(n2);
	}
	residual.SetNumberOfItems(nUnknowns); increment.SetNumberOfItems(nUnknowns);
	temp2.SetNumberOfItems(n2); tempF.SetNumberOfItems(n2); temp1.SetNumberOfItems(n1);
	tempAE.SetNumberOfItems(na); tempAEv.SetNumberOfItems(na); tempAEm.SetNumberOfItems(na);
	meanA1.SetNumberOfItems(n2); meanA2.SetNumberOfItems(n2); candidateMeanA.SetNumberOfItems(n2);

	bool dense = EXUstd::IsOfType(LinearSolverType::Dense, simulationSettings.linearSolver.solverType);
	jq = dense ? (GeneralMatrix*)&jqDense : (GeneralMatrix*)&jqSparse;
	jv = dense ? (GeneralMatrix*)&jvDense : (GeneralMatrix*)&jvSparse;
	stageMatrix = dense ? (GeneralMatrix*)&stageDense : (GeneralMatrix*)&stageSparse;
#ifdef USE_EIGEN_DENSE_SOLVER
	if (dense) { stageDense.UseEigenSolverType() = ((GeneralMatrixEXUdense*)solver.data.systemJacobian)->UseEigenSolverType(); }
#endif
	stageMatrix->PivotThreshold() = solver.data.systemJacobian->PivotThreshold(); //as the solver's matrix
	if (!dense) { stageSparse.SetReuseAnalyzedPattern(simulationSettings.linearSolver.reuseAnalyzedPattern); }
	Index nSys = n2 + n1 + na;
	jq->SetNumberOfRowsAndColumns(nSys, nSys);
	jv->SetNumberOfRowsAndColumns(nSys, nSys);
	stageMatrix->SetNumberOfRowsAndColumns(nUnknowns, nUnknowns);

	controller.Initialize(timeint.stepSizeSafety, timeint.stepSizeMaxIncrease, solver.it.minStepSize, solver.it.maxStepSize);
	nextStepSize = -1.;
	rejectedByError = false;
	rejectedStepsError = 0;
	hasPrevious = false;
	candidateH = 0.;
	numberOfMeans = 0;
	minStepSizeWarned = false;
}

//! the state of stage i (iterate: the positions of the previous iterate, for C_q^T mu) into the current state
void CSolverImplicitRungeKutta::SetStageState(CSystem& computationalSystem, Index i, Real t0, Real h, bool iterate)
{
	CSystemState& state = computationalSystem.GetSystemData().GetCData().currentState;
	state.ODE2Coords.CopyFrom(iterate ? qIterate[i] : Q[i]);
	state.ODE2Coords_t.CopyFrom(V[i]);
	state.ODE2Coords_tt.CopyFrom(stageA[i]);
	state.ODE1Coords.CopyFrom(Y[i]);
	state.ODE1Coords_t.CopyFrom(stageYd[i]);
	state.AECoords.CopyFrom(stageZ[i]);
	state.time = t0 + h * method.c[i];
}

//! the stage positions, velocities and ODE1 coordinates from the stage unknowns; with GGL the positions minus C_q^T mu
void CSolverImplicitRungeKutta::ComputeStages(CSolverImplicitSecondOrderTimeInt& solver, CSystem& computationalSystem, Real t0, Real h)
{
	const Index s = method.s;
	for (Index i = 0; i < s; i++)
	{
		V[i].CopyFrom(v0);
		temp2.CopyFrom(v0);
		temp2 *= h * method.c[i]; //the increment of the positions
		Y[i].CopyFrom(y0);
		for (Index j = 0; j < s; j++)
		{
			if (method.A[i][j] != 0.) { V[i].MultAdd(h * method.A[i][j], stageA[j]); Y[i].MultAdd(h * method.A[i][j], stageYd[j]); }
			if (A2[i][j] != 0.) { temp2.MultAdd(h * h * A2[i][j], stageA[j]); }
		}
		Q[i].CopyFrom(q0);
		Q[i] += temp2;
	}
	//Lie group nodes: the rotation of stage i by the composition of the start of the step with the increment
	//Delta_i = h sum_j A_ij TExpSO3Inv(Delta_j) omega_j, omega_j the angular velocities of stage j (Runge-Kutta-Munthe-Kaas,
	//as the explicit solvers), by a fixed-point iteration from Delta_i = h sum_j A_ij omega_j, the increment of the
	//other coordinates, which limits the order to 2 (#3290)
	const Index maxFixedPointIterations = 10;
	for (Index k : lieGroupNodes)
	{
		const CNodeRigidBody& node = (const CNodeRigidBody&)(computationalSystem.GetSystemData().GetCNode(k));
		Index nPos = node.GetNumberOfDisplacementCoordinates();
		Index nRot = node.GetNumberOfRotationCoordinates();
		Index off = node.GetGlobalODE2CoordinateIndex();
		Vector3D delta[ImplicitRungeKuttaMethod::maxStages], w[ImplicitRungeKuttaMethod::maxStages];
		for (Index i = 0; i < s; i++) { delta[i] = Vector3D(Q[i], off + nPos) - Vector3D(q0, off + nPos); }
		for (Index iteration = 0; iteration < maxFixedPointIterations; iteration++)
		{
			for (Index j = 0; j < s; j++) { w[j] = EXUlie::TExpSO3Inv(delta[j]) * Vector3D(V[j], off + nPos); }
			Real change = 0.;
			for (Index i = 0; i < s; i++)
			{
				Vector3D newDelta(0.);
				for (Index j = 0; j < s; j++) { if (method.A[i][j] != 0.) { newDelta += (h * method.A[i][j]) * w[j]; } }
				for (Index m = 0; m < 3; m++) { change = EXUstd::Maximum(change, fabs(newDelta[m] - delta[i][m])); }
				delta[i] = newDelta;
			}
			if (change <= 1e-15) { break; }
		}
		for (Index i = 0; i < s; i++)
		{
			LinkedDataVector position0(q0, off, nPos), rotation0(q0, off + nPos, nRot);
			LinkedDataVector position(Q[i], off, nPos), rotation(Q[i], off + nPos, nRot);
			Vector6D incrementalMotion;
			for (Index m = 0; m < nPos; m++) { incrementalMotion[m] = Q[i][off + m] - q0[off + m]; }
			for (Index m = 0; m < 3; m++) { incrementalMotion[nPos + m] = delta[i][m]; }
			node.CompositionRule(position0, rotation0, incrementalMotion, position, rotation);
		}
	}
	if (method.ggl && solver.data.nAE)
	{
		for (Index i : implicitStages)
		{
			SetStageState(computationalSystem, i, t0, h, true);
			temp2.SetAll(0.);
			computationalSystem.ComputeODE2ProjectedReactionForces(solver.data.tempCompDataArray, stageMu[i], temp2);
			Q[i] -= temp2;
		}
	}
}

Real CSolverImplicitRungeKutta::ComputeResidual(CSolverImplicitSecondOrderTimeInt& solver, CSystem& computationalSystem,
	const SimulationSettings& simulationSettings, Real t0, Real h)
{
	const Index n2 = solver.data.nODE2, n1 = solver.data.nODE1, na = solver.data.nAE;
	const Index s = method.s;
	ComputeStages(solver, computationalSystem, t0, h);
	residual.SetAll(0.);
	for (Index k = 0; k < implicitStages.NumberOfItems(); k++)
	{
		Index i = implicitStages[k];
		SetStageState(computationalSystem, i, t0, h);
		if (!solver.hasConstantMassMatrix)
		{
			solver.data.systemMassMatrix->SetAllZero();
			computationalSystem.ComputeMassMatrix(solver.data.tempCompDataArray, *(solver.data.systemMassMatrix));
		}
		if (n2)
		{
			computationalSystem.ComputeSystemODE2RHS(solver.data.tempCompDataArray, tempF);
			solver.data.systemMassMatrix->MultMatrixVector(stageA[i], temp2);
			temp2 -= tempF;
			computationalSystem.ComputeODE2ProjectedReactionForces(solver.data.tempCompDataArray, stageZ[i], temp2);
			for (Index j = 0; j < n2; j++) { residual[offA[k] + j] = temp2[j]; }
		}
		if (n1)
		{
			computationalSystem.ComputeSystemODE1RHS(solver.data.tempCompData, temp1);
			temp1 -= stageYd[i];
			computationalSystem.ComputeODE1ProjectedReactionTerms(solver.data.tempCompDataArray, stageZ[i], temp1, -1.);
			for (Index j = 0; j < n1; j++) { residual[offY[k] + j] = temp1[j]; }
		}
		if (method.ggl && na)
		{
			computationalSystem.ComputeAlgebraicEquations(solver.data.tempCompDataArray, tempAE, false);
			computationalSystem.ComputeAlgebraicEquations(solver.data.tempCompDataArray, tempAEv, true);
			for (Index j = 0; j < na; j++)
			{
				residual[offZ[k] + j] = (specialRow[j] == 0 && fabs(tempAE[j]) <= constraintRoundoff) ? 0. : tempAE[j];
				residual[offMu[k] + j] = specialRow[j] ? stageMu[i][j] : tempAEv[j];
			}
		}
	}
	if (!method.ggl && na) //the constraints at the end; the special rows at the last two stages
	{
		Index m = implicitStages[0];
		SetStageState(computationalSystem, m, t0, h);
		computationalSystem.ComputeAlgebraicEquations(solver.data.tempCompDataArray, tempAEm, false);
		SetStageState(computationalSystem, s - 1, t0, h);
		computationalSystem.ComputeAlgebraicEquations(solver.data.tempCompDataArray, tempAE, false);
		computationalSystem.ComputeAlgebraicEquations(solver.data.tempCompDataArray, tempAEv, true);
		for (Index j = 0; j < na; j++)
		{
			residual[offZ[0] + j] = specialRow[j] ? tempAEm[j] : ((fabs(tempAE[j]) <= constraintRoundoff) ? 0. : tempAE[j]);
			residual[offZ[1] + j] = tempAEv[j];
		}
		if (implicitStages.NumberOfItems() == 3) //Radau5: the velocity constraints at the middle implicit stage, with its multipliers - order 5 in positions and velocities; the position constraints there gave order 4, at all stages order 3 in the velocities (#3318)
		{
			SetStageState(computationalSystem, implicitStages[1], t0, h);
			computationalSystem.ComputeAlgebraicEquations(solver.data.tempCompDataArray, tempAEm, false);
			computationalSystem.ComputeAlgebraicEquations(solver.data.tempCompDataArray, tempAEv, true);
			for (Index j = 0; j < na; j++)
			{
				residual[offZ[2] + j] = specialRow[j] ? tempAEm[j] : tempAEv[j];
			}
		}
	}
	if (solver.IsVerboseCheck(3)) //the largest residual of each block of unknowns
	{
		auto MaxAbs = [this](Index offset, Index n) { Real m = 0.; for (Index j = 0; j < n; j++) { m = EXUstd::Maximum(m, fabs(residual[offset + j])); } return m; };
		STDstring blocks;
		for (Index k = 0; k < implicitStages.NumberOfItems(); k++)
		{
			blocks += " stage " + EXUstd::ToString(implicitStages[k]) + ": ODE2 " + EXUstd::ToString(MaxAbs(offA[k], n2)) + ", ODE1 " + EXUstd::ToString(MaxAbs(offY[k], n1));
			blocks += ", |z| " + EXUstd::ToString(stageZ[implicitStages[k]].GetL2Norm()) + ", |a| " + EXUstd::ToString(stageA[implicitStages[k]].GetL2Norm());
			if (method.ggl) { blocks += ", AE " + EXUstd::ToString(MaxAbs(offZ[k], na)) + ", mu " + EXUstd::ToString(MaxAbs(offMu[k], na)); }
		}
		if (!method.ggl && na) { blocks += "; end: AE " + EXUstd::ToString(MaxAbs(offZ[0], na)) + ", velocity AE " + EXUstd::ToString(MaxAbs(offZ[1], na)); }
		solver.VerboseWrite(3, "    residual (largest per block):" + blocks + "\n");
	}
	Real sum = 0.;
	for (Index j = 0; j < nUnknowns; j++)
	{
		sum += EXUstd::Square(residual[j]);
		residual[j] *= rowScale[j];
	}
	return sqrt(sum);
}

//! the factors of the rows of the residual for the step size h, for a matrix of entries of order 1: the ODE2 and ODE1
//! equations 1, the constraints 1/h^2, the velocity constraints and velocity-level constraints 1/h, the equations of
//! a multiplier 1, the equations mu = 0 1/h^2 (mu is h^2 times its unknown)
void CSolverImplicitRungeKutta::ComputeRowScale(const CSolverImplicitSecondOrderTimeInt& solver, Real h)
{
	const Index na = solver.data.nAE;
	rowScale.SetNumberOfItems(nUnknowns);
	rowScale.SetAll(1.);
	auto RowScaleAE = [this, h](Index j, bool velocityEquation)
	{
		if (specialRow[j] == 2) { return 1.; } //an equation of the multiplier
		return (velocityEquation || specialRow[j] == 1) ? 1. / h : 1. / (h * h);
	};
	for (Index k = 0; k < implicitStages.NumberOfItems(); k++)
	{
		if (method.ggl)
		{
			for (Index j = 0; j < na; j++)
			{
				rowScale[offZ[k] + j] = RowScaleAE(j, false);
				rowScale[offMu[k] + j] = specialRow[j] ? 1. / (h * h) : 1. / h;
			}
		}
	}
	if (!method.ggl && na)
	{
		for (Index j = 0; j < na; j++)
		{
			rowScale[offZ[0] + j] = RowScaleAE(j, false);
			rowScale[offZ[1] + j] = RowScaleAE(j, true);
		}
	}
}

//! add a block of triplets, scaled, at an offset; rowSelect 0, 1, 2: only the rows j with specialRow[j] == rowSelect
void CSolverImplicitRungeKutta::AddBlock(const IRKBlock& block, Index rowOffset, Index columnOffset, Real factor, Index rowSelect)
{
	if (factor == 0.) { return; }
	for (size_t k = 0; k < block.values.size(); k++)
	{
		if (rowSelect == -1 || specialRow[block.rows[k]] == rowSelect)
		{
			tripletRows.push_back(block.rows[k] + rowOffset);
			tripletColumns.push_back(block.columns[k] + columnOffset);
			tripletValues.push_back(factor * block.values[k]);
		}
	}
}

//! add left times right^T... : left (rows x n) times right (n x nRight, given as its triplets), the columns j of the
//! product with specialRow[j] left out (their mu is zero)
void CSolverImplicitRungeKutta::AddProduct(const IRKBlock& left, const IRKBlock& right, Index rowOffset, Index columnOffset, Real factor, Index nRight)
{
	if (factor == 0.) { return; }
	//the right block by rows, compressed, in the work arrays of the solver (#3287)
	Index nRows = 0;
	for (Index row : right.rows) { nRows = EXUstd::Maximum(nRows, row + 1); }
	rightStart.assign(nRows + 1, 0);
	for (size_t k = 0; k < right.values.size(); k++) { if (!specialRow[right.columns[k]]) { rightStart[right.rows[k] + 1]++; } }
	for (Index i = 0; i < nRows; i++) { rightStart[i + 1] += rightStart[i]; }
	rightColumns.resize(rightStart[nRows]);
	rightValues.resize(rightStart[nRows]);
	for (size_t k = 0; k < right.values.size(); k++)
	{
		if (!specialRow[right.columns[k]])
		{
			Index position = rightStart[right.rows[k]]++;
			rightColumns[position] = right.columns[k];
			rightValues[position] = right.values[k];
		}
	}
	for (Index i = nRows; i > 0; i--) { rightStart[i] = rightStart[i - 1]; } //back to the starts of the rows
	rightStart[0] = 0;

	//the products, summed per entry after sorting
	productEntries.clear();
	for (size_t k = 0; k < left.values.size(); k++)
	{
		Index row = left.columns[k];
		if (row >= nRows) { continue; }
		for (Index position = rightStart[row]; position < rightStart[row + 1]; position++)
		{
			productEntries.push_back({ (int64_t)left.rows[k] * (int64_t)nRight + rightColumns[position], left.values[k] * rightValues[position] });
		}
	}
	std::sort(productEntries.begin(), productEntries.end(), [](const std::pair<int64_t, Real>& a, const std::pair<int64_t, Real>& b) { return a.first < b.first; });
	for (size_t k = 0; k < productEntries.size(); )
	{
		int64_t key = productEntries[k].first;
		Real value = 0.;
		for (; k < productEntries.size() && productEntries[k].first == key; k++) { value += productEntries[k].second; }
		tripletRows.push_back((Index)(key / nRight) + rowOffset);
		tripletColumns.push_back((Index)(key % nRight) + columnOffset);
		tripletValues.push_back(factor * value);
	}
}

//! the columns of the rotation coordinates of Lie group nodes in the entries of jq times the tangent operator T(Delta)
//! of the increment Delta of the last stage: the rotation of a stage is the composition of the start of the step with
//! Delta, which depends on the stage accelerations through T, as generalized-alpha with lieGroupAddTangentOperator
void CSolverImplicitRungeKutta::ApplyTangentOperator(CSystem& computationalSystem, Index nRows, Real h)
{
	const Index s = method.s;
	std::vector<Index>& rows = entryRows;
	std::vector<Index>& columns = entryColumns;
	std::vector<Real>& values = entryValues;
	//the entries in the rotation columns, moved out of the list and sorted by node and row
	lieGroupEntries.clear();
	for (Index slot = 0; slot < lieGroupNodes.NumberOfItems(); slot++)
	{
		const CNodeRigidBody& node = (const CNodeRigidBody&)(computationalSystem.GetSystemData().GetCNode(lieGroupNodes[slot]));
		Index first = node.GetGlobalODE2CoordinateIndex() + node.GetNumberOfDisplacementCoordinates();
		for (size_t k = 0; k < values.size(); k++)
		{
			if (columns[k] >= first && columns[k] < first + 3 && values[k] != 0.)
			{
				lieGroupEntries.push_back({ (int64_t)slot * (int64_t)nRows + rows[k], { columns[k] - first, values[k] } });
				values[k] = 0.;
			}
		}
	}
	std::sort(lieGroupEntries.begin(), lieGroupEntries.end(),
		[](const std::pair<int64_t, std::pair<Index, Real>>& a, const std::pair<int64_t, std::pair<Index, Real>>& b) { return a.first < b.first; });

	Index lastSlot = -1;
	Matrix3D T;
	Index first = 0;
	for (size_t k = 0; k < lieGroupEntries.size(); )
	{
		int64_t key = lieGroupEntries[k].first;
		Index slot = (Index)(key / nRows);
		Index row = (Index)(key % nRows);
		if (slot != lastSlot)
		{
			//the rotation increment of the last stage: h c v0 + h^2 sum_j A^2 a_j
			const CNodeRigidBody& node = (const CNodeRigidBody&)(computationalSystem.GetSystemData().GetCNode(lieGroupNodes[slot]));
			first = node.GetGlobalODE2CoordinateIndex() + node.GetNumberOfDisplacementCoordinates();
			Vector3D increment;
			for (Index m = 0; m < 3; m++)
			{
				increment[m] = h * method.c[s - 1] * v0[first + m];
				for (Index j = 0; j < s; j++) { increment[m] += h * h * A2[s - 1][j] * stageA[j][first + m]; }
			}
			T = EXUlie::TExpSO3(increment);
			lastSlot = slot;
		}
		Vector3D rowValues(0.);
		for (; k < lieGroupEntries.size() && lieGroupEntries[k].first == key; k++)
		{
			rowValues[lieGroupEntries[k].second.first] += lieGroupEntries[k].second.second;
		}
		for (Index j = 0; j < 3; j++)
		{
			Real value = rowValues[0] * T(0, j) + rowValues[1] * T(1, j) + rowValues[2] * T(2, j);
			if (value != 0.) { rows.push_back(row); columns.push_back(first + j); values.push_back(value); }
		}
	}
}

//! the stage Jacobian from the Jacobians of the system at the last stage, by the coefficients of A and A^2
void CSolverImplicitRungeKutta::ComputeJacobian(CSolverImplicitSecondOrderTimeInt& solver, CSystem& computationalSystem,
	const SimulationSettings& simulationSettings, Real t0, Real h)
{
	const TimeIntegrationSettings& timeint = simulationSettings.timeIntegration;
	const Index n2 = solver.data.nODE2, n1 = solver.data.nODE1, na = solver.data.nAE;
	const Index s = method.s;
	const Index startAE = n2 + n1;
	ComputeStages(solver, computationalSystem, t0, h); //the stage states of the current iterate
	SetStageState(computationalSystem, s - 1, t0, h);
	if (!solver.hasConstantMassMatrix)
	{
		solver.data.systemMassMatrix->SetAllZero();
		computationalSystem.ComputeMassMatrix(solver.data.tempCompDataArray, *(solver.data.systemMassMatrix));
	}
	jq->SetAllZero();
	jv->SetAllZero();
	computationalSystem.JacobianODE2RHS(solver.data.tempCompDataArray, timeint.newton.numericalDifferentiation, *jq, -1., 0., -1., timeint.computeLoadsJacobian);
	computationalSystem.JacobianODE1RHS(solver.data.tempCompDataArray, timeint.newton.numericalDifferentiation, *jq, 1., 0., 1., timeint.computeLoadsJacobian);
	computationalSystem.JacobianODE2RHS(solver.data.tempCompDataArray, timeint.newton.numericalDifferentiation, *jv, 0., -1., 0., timeint.computeLoadsJacobian);
	computationalSystem.JacobianODE1RHS(solver.data.tempCompDataArray, timeint.newton.numericalDifferentiation, *jv, 0., 1., 0., timeint.computeLoadsJacobian);
	if (na)
	{
		computationalSystem.JacobianAE(solver.data.tempCompDataArray, timeint.newton, *jq, 1., 0., 1., false, 1., -1., 1.);
		computationalSystem.JacobianAE(solver.data.tempCompDataArray, timeint.newton, *jv, 0., 1., 0., false, 0., 0., 0.);
	}
	jq->FinalizeMatrix();
	jv->FinalizeMatrix();

	//the blocks
	for (IRKBlock* b : { &blockK, &blockKy, &blockCt, &blockGq, &blockGy, &blockCyt, &blockCq, &blockCy, &blockCaa, &blockD, &blockGv, &blockCv, &blockM }) { b->Clear(); }
	auto Range = [n2, n1](Index i) { return (i < n2) ? 0 : ((i < n2 + n1) ? 1 : 2); };
	auto Local = [n2, startAE](Index i) { return (i < n2) ? i : ((i < startAE) ? i - n2 : i - startAE); };
	std::vector<Index>& rows = entryRows; //the work arrays of the solver (#3287)
	std::vector<Index>& columns = entryColumns;
	std::vector<Real>& values = entryValues;
	jq->GetSparseEntries(rows, columns, values);
	if (lieGroupNodes.NumberOfItems()) { ApplyTangentOperator(computationalSystem, n2 + n1 + na, h); }
	IRKBlock* qBlocks[3][3] = { {&blockK, &blockKy, &blockCt}, {&blockGq, &blockGy, &blockCyt}, {&blockCq, &blockCy, &blockCaa} };
	specialRow.CopyFrom(velocityLevelRow);
	for (size_t k = 0; k < values.size(); k++)
	{
		if (values[k] == 0.) { continue; }
		qBlocks[Range(rows[k])][Range(columns[k])]->Add(Local(rows[k]), Local(columns[k]), values[k]);
		if (Range(rows[k]) == 2 && Range(columns[k]) == 2 && !specialRow[Local(rows[k])]) { specialRow[Local(rows[k])] = 2; } //an equation of the multiplier
	}
	jv->GetSparseEntries(rows, columns, values);
	IRKBlock* vBlocks[3] = { &blockD, &blockGv, &blockCv };
	for (size_t k = 0; k < values.size(); k++)
	{
		if (values[k] != 0. && Range(columns[k]) == 0) { vBlocks[Range(rows[k])]->Add(Local(rows[k]), columns[k], values[k]); }
	}
	if (solver.IsVerboseCheck(3)) { solver.VerboseWrite(3, "    special rows of the constraints (1 velocity level, 2 equation of the multiplier) = " + EXUstd::ToString(specialRow) + "\n"); }
	solver.data.systemMassMatrix->GetSparseEntries(rows, columns, values);
	for (size_t k = 0; k < values.size(); k++) { if (values[k] != 0.) { blockM.Add(rows[k], columns[k], values[k]); } }

	//the stage matrix: the derivatives of the equations, each row times its factor of rowScale; mu is h^2 times its unknown
	ComputeRowScale(solver, h);
	tripletRows.clear(); tripletColumns.clear(); tripletValues.clear();
	const Real h2 = h * h;
	const Index nImplicit = implicitStages.NumberOfItems();
	for (Index k = 0; k < nImplicit; k++)
	{
		Index i = implicitStages[k];
		for (Index l = 0; l < nImplicit; l++)
		{
			Index j = implicitStages[l];
			Real a = method.A[i][j], a2 = A2[i][j];
			if (i == j) { AddBlock(blockM, offA[k], offA[l], 1.); }
			AddBlock(blockK, offA[k], offA[l], h2 * a2);
			AddBlock(blockD, offA[k], offA[l], h * a);
			AddBlock(blockKy, offA[k], offY[l], h * a);
			AddBlock(blockGq, offY[k], offA[l], h2 * a2);
			AddBlock(blockGv, offY[k], offA[l], h * a);
			AddBlock(blockGy, offY[k], offY[l], h * a);
			if (method.ggl && na)
			{
				AddBlock(blockCq, offZ[k], offA[l], h2 * a2);
				AddBlock(blockCv, offZ[k], offA[l], h * a);
				AddBlock(blockCy, offZ[k], offY[l], h * a);
				AddBlock(blockCq, offMu[k], offA[l], h * a, 0); //C_q V + c_t
			}
		}
		for (Index j = 0; j < n1; j++) { tripletRows.push_back(offY[k] + j); tripletColumns.push_back(offY[k] + j); tripletValues.push_back(-1.); }
		if (na)
		{
			AddBlock(blockCt, offA[k], offZ[k], 1.);
			AddBlock(blockCyt, offY[k], offZ[k], 1.);
			if (method.ggl)
			{
				AddBlock(blockCaa, offZ[k], offZ[k], 1.);
				AddProduct(blockK, blockCt, offA[k], offMu[k], -h2, na); //Q_i - C_q^T mu_i
				AddProduct(blockGq, blockCt, offY[k], offMu[k], -h2, na);
				AddProduct(blockCq, blockCt, offZ[k], offMu[k], -h2, na);
				for (Index j = 0; j < na; j++)
				{
					if (specialRow[j]) { tripletRows.push_back(offMu[k] + j); tripletColumns.push_back(offMu[k] + j); tripletValues.push_back(h2); } //mu = h^2 times its unknown
				}
			}
		}
	}
	if (!method.ggl && na) //the end: rows offZ[0] (positions; special rows at stage m) and offZ[1] (velocities)
	{
		Index m = implicitStages[0];
		for (Index l = 0; l < nImplicit; l++)
		{
			Index j = implicitStages[l];
			AddBlock(blockCq, offZ[0], offA[l], h2 * A2[s - 1][j], 0);
			AddBlock(blockCy, offZ[0], offY[l], h * method.A[s - 1][j], 0);
			for (Index c = 1; c <= 2; c++)
			{
				AddBlock(blockCq, offZ[0], offA[l], h2 * A2[m][j], c);
				AddBlock(blockCv, offZ[0], offA[l], h * method.A[m][j], c);
				AddBlock(blockCy, offZ[0], offY[l], h * method.A[m][j], c);
				AddBlock(blockCq, offZ[1], offA[l], h2 * A2[s - 1][j], c);
				AddBlock(blockCv, offZ[1], offA[l], h * method.A[s - 1][j], c);
				AddBlock(blockCy, offZ[1], offY[l], h * method.A[s - 1][j], c);
			}
			AddBlock(blockCq, offZ[1], offA[l], h * method.A[s - 1][j], 0);
		}
		for (Index c = 1; c <= 2; c++)
		{
			AddBlock(blockCaa, offZ[0], offZ[0], 1., c);
			AddBlock(blockCaa, offZ[1], offZ[nImplicit - 1], 1., c);
		}
		if (nImplicit == 3) //Radau5: the rows offZ[2], the velocity constraints at the middle implicit stage, the special rows there (#3318)
		{
			const Index i2 = implicitStages[1];
			for (Index l = 0; l < nImplicit; l++)
			{
				Index j = implicitStages[l];
				AddBlock(blockCq, offZ[2], offA[l], h * method.A[i2][j], 0); //C_q V_i2
				for (Index c = 1; c <= 2; c++)
				{
					AddBlock(blockCq, offZ[2], offA[l], h2 * A2[i2][j], c);
					AddBlock(blockCy, offZ[2], offY[l], h * method.A[i2][j], c);
					AddBlock(blockCv, offZ[2], offA[l], h * method.A[i2][j], c);
				}
			}
			for (Index c = 1; c <= 2; c++) { AddBlock(blockCaa, offZ[2], offZ[1], 1., c); }
		}
	}
	for (size_t k = 0; k < tripletValues.size(); k++) { tripletValues[k] *= rowScale[tripletRows[k]]; }
	stageMatrix->SetAllZero();
	stageTriplets.SetNumberOfItems(0);
	for (size_t k = 0; k < tripletValues.size(); k++) { stageTriplets.Append(EXUmath::Triplet(tripletRows[k], tripletColumns[k], tripletValues[k])); }
	stageMatrix->AddSparseTriplets(stageTriplets);
	stageMatrix->FinalizeMatrix();
	if (solver.IsVerboseCheck(3)) { solver.VerboseWrite(3, "    stage Jacobian = " + EXUstd::ToString(stageMatrix->GetEXUdenseMatrix()) + "\n"); }
}

//! the error estimate of the step in the norm of the tolerances, and the order of the local error it estimates
Real CSolverImplicitRungeKutta::ErrorEstimate(CSolverImplicitSecondOrderTimeInt& solver, const SimulationSettings& simulationSettings,
	Real t0, Real h, Real& order)
{
	const Index n2 = solver.data.nODE2;
	const Index s = method.s;
	const Real atol = simulationSettings.timeIntegration.absoluteTolerance;
	const Real rtol = simulationSettings.timeIntegration.relativeTolerance;
	Real sum = 0.;
	Index count = 0;
	auto Add = [&](Real error, Real value0, Real value1)
	{
		Real scale = atol + rtol * EXUstd::Maximum(fabs(value0), fabs(value1));
		sum += EXUstd::Square(error / scale);
		count++;
	};
	//TrapezoidalGGL: the velocity error h^3/12 a_tt, a_tt by the second difference of the mean accelerations (v1 - v0)/h
	//of this and the last two steps, which do not alternate as the accelerations do with constraints; as SolveTrapezoidalGGL (#3287)
	const bool trapezoidal = (s == 2 && method.c[0] == 0. && method.ggl);
	const bool twoSteps = hasPrevious && errorEstimateTwoSteps;
	if (trapezoidal && (!twoSteps || solver.data.nAE) && numberOfMeans == 2)
	{
		Real tau1 = 0.5 * (h + meanH1), tau2 = 0.5 * (meanH1 + meanH2);
		for (Index j = 0; j < n2; j++)
		{
			Real mean = (V[s - 1][j] - v0[j]) / h;
			Real a_tt = (2. / (tau1 + tau2)) * ((mean - meanA1[j]) / tau1 - (meanA1[j] - meanA2[j]) / tau2);
			Add((h * h * h / 12.) * a_tt, v0[j], V[s - 1][j]);
		}
	}
	if (trapezoidal && !twoSteps)
	{
		//the position error h^3/12 q_ttt = h^2/12 (a1 - a0) of the trapezoidal rule (#3287)
		for (Index j = 0; j < n2; j++) { Add((h * h / 12.) * (stageA[s - 1][j] - stageA[0][j]), q0[j], Q[s - 1][j]); }
		order = 3.;
	}
	else if (!twoSteps)
	{
		//against the trapezoidal rule of the first (or the start) and the last stage, an error of order h^3
		const Vector& vFirst = (method.c[0] == 0.) ? V[0] : v0;
		const Vector& aFirst = (method.c[0] == 0.) ? stageA[0] : a0;
		for (Index j = 0; j < n2; j++)
		{
			Add(Q[s - 1][j] - (q0[j] + 0.5 * h * (vFirst[j] + V[s - 1][j])), q0[j], Q[s - 1][j]);
			Add(V[s - 1][j] - (v0[j] + 0.5 * h * (aFirst[j] + stageA[s - 1][j])), v0[j], V[s - 1][j]);
		}
		order = 3.;
	}
	else
	{
		//the quadrature of the method against the polynomial through the stage velocities of the previous and this
		//step, of the order of the method (#3250); the accelerations only without constraints
		Real tau[2 * ImplicitRungeKuttaMethod::maxStages];
		const Vector* pointV[2 * ImplicitRungeKuttaMethod::maxStages];
		const Vector* pointA[2 * ImplicitRungeKuttaMethod::maxStages];
		Index nPoints = 0;
		for (Index j = 0; j < s; j++)
		{
			Real t = (previousT0 + previousH * method.c[j] - t0) / h;
			bool older = true;
			for (Index k = 0; k < s; k++) { if (fabs(t - method.c[k]) <= 1e-9) { older = false; } }
			if (older) { tau[nPoints] = t; pointV[nPoints] = &previousV[j]; pointA[nPoints] = &previousA[j]; nPoints++; }
		}
		for (Index j = 0; j < s; j++) { tau[nPoints] = method.c[j]; pointV[nPoints] = &V[j]; pointA[nPoints] = &stageA[j]; nPoints++; }
		//the weights of the integral over [0, 1] of the polynomial through the points: V^T w = (1, 1/2, 1/3, ...)
		Real matrix[2 * ImplicitRungeKuttaMethod::maxStages][2 * ImplicitRungeKuttaMethod::maxStages + 1];
		for (Index r = 0; r < nPoints; r++)
		{
			for (Index c = 0; c < nPoints; c++) { Real power = 1.; for (Index e = 0; e < r; e++) { power *= tau[c]; } matrix[r][c] = power; }
			matrix[r][nPoints] = 1. / (Real)(r + 1);
		}
		for (Index c = 0; c < nPoints; c++) //Gauss elimination with pivoting
		{
			Index pivot = c;
			for (Index r = c + 1; r < nPoints; r++) { if (fabs(matrix[r][c]) > fabs(matrix[pivot][c])) { pivot = r; } }
			for (Index k = 0; k <= nPoints; k++) { std::swap(matrix[c][k], matrix[pivot][k]); }
			for (Index r = 0; r < nPoints; r++)
			{
				if (r == c) { continue; }
				Real f = matrix[r][c] / matrix[c][c];
				for (Index k = c; k <= nPoints; k++) { matrix[r][k] -= f * matrix[c][k]; }
			}
		}
		Real weight[2 * ImplicitRungeKuttaMethod::maxStages];
		for (Index r = 0; r < nPoints; r++) { weight[r] = matrix[r][nPoints] / matrix[r][r]; }
		bool withAccelerations = (solver.data.nAE == 0);
		for (Index j = 0; j < n2; j++)
		{
			Real errorQ = 0., errorV = 0.;
			for (Index k = 0; k < s; k++) { errorQ += method.A[s - 1][k] * V[k][j]; errorV += method.A[s - 1][k] * stageA[k][j]; }
			for (Index p = 0; p < nPoints; p++) { errorQ -= weight[p] * (*pointV[p])[j]; errorV -= weight[p] * (*pointA[p])[j]; }
			Add(h * errorQ, q0[j], Q[s - 1][j]);
			if (withAccelerations) { Add(h * errorV, v0[j], V[s - 1][j]); }
		}
		order = (Real)nPoints;
	}
	return count ? sqrt(sum / count) : 0.;
}

//! verbose 4: the stage Jacobian against the finite differences of the residual, the largest difference per block of
//! rows and columns (A: accelerations, Y: ODE1 velocities, Z: multipliers, M: mu of GGL; stage index)
//! the columns of Euler parameters differ by the derivative along the parameters themselves, which the Jacobian by G
//! leaves out and the constraint of the Euler parameters fixes; at small angles such entries have the opposite sign (#3308)
void CSolverImplicitRungeKutta::CheckJacobian(CSolverImplicitSecondOrderTimeInt& solver, CSystem& computationalSystem,
	const SimulationSettings& simulationSettings, Real t0, Real h)
{
	const Index n2 = solver.data.nODE2, n1 = solver.data.nODE1, na = solver.data.nAE;
	ResizableMatrix J = stageMatrix->GetEXUdenseMatrix();
	ComputeResidual(solver, computationalSystem, simulationSettings, t0, h);
	Vector r0 = residual;
	auto Unknown = [&](Index u) -> Real&
	{
		for (Index k = 0; k < implicitStages.NumberOfItems(); k++)
		{
			Index i = implicitStages[k];
			if (u >= offA[k] && u < offA[k] + n2) { return stageA[i][u - offA[k]]; }
			if (u >= offY[k] && u < offY[k] + n1) { return stageYd[i][u - offY[k]]; }
			if (u >= offZ[k] && u < offZ[k] + na) { return stageZ[i][u - offZ[k]]; }
			if (method.ggl && u >= offMu[k] && u < offMu[k] + na) { return stageMu[i][u - offMu[k]]; }
		}
		CHECKandTHROWstring("CheckJacobian: unknown");
		return stageA[0][0];
	};
	auto Block = [&](Index u) -> STDstring
	{
		for (Index k = 0; k < implicitStages.NumberOfItems(); k++)
		{
			STDstring stage = EXUstd::ToString(implicitStages[k]);
			if (u >= offA[k] && u < offA[k] + n2) { return "A" + stage + "[" + EXUstd::ToString(u - offA[k]) + "]"; }
			if (u >= offY[k] && u < offY[k] + n1) { return "Y" + stage + "[" + EXUstd::ToString(u - offY[k]) + "]"; }
			if (u >= offZ[k] && u < offZ[k] + na) { return "Z" + stage + "[" + EXUstd::ToString(u - offZ[k]) + "]"; }
			if (method.ggl && u >= offMu[k] && u < offMu[k] + na) { return "M" + stage + "[" + EXUstd::ToString(u - offMu[k]) + "]"; }
		}
		return "?";
	};
	std::map<STDstring, std::pair<Real, STDstring>> largest; //block pair -> (difference, entry)
	for (Index c = 0; c < nUnknowns; c++)
	{
		Real& x = Unknown(c);
		Real x0 = x;
		bool isMu = false;
		for (Index k = 0; k < implicitStages.NumberOfItems(); k++) { if (method.ggl && c >= offMu[k] && c < offMu[k] + na) { isMu = true; } }
		Real scale = isMu ? h * h : 1.; //mu is h^2 times its unknown
		Real eps = 1e-3 * EXUstd::Maximum(1., fabs(x0 / scale)); //large: the constraints at the end change by h^2 times it
		x = x0 + eps * scale;
		ComputeResidual(solver, computationalSystem, simulationSettings, t0, h);
		x = x0;
		for (Index r = 0; r < nUnknowns; r++)
		{
			Real numeric = (residual[r] - r0[r]) / eps;
			Real difference = fabs(numeric - J(r, c));
			STDstring rowBlock = Block(r), columnBlock = Block(c);
			STDstring key = rowBlock.substr(0, 2) + "/" + columnBlock.substr(0, 2);
			if (difference > largest[key].first)
			{
				largest[key] = { difference, rowBlock + "/" + columnBlock + ": numeric " + EXUstd::ToString(numeric) + ", Jacobian " + EXUstd::ToString(J(r, c)) };
			}
		}
	}
	ComputeResidual(solver, computationalSystem, simulationSettings, t0, h);
	for (const auto& entry : largest)
	{
		if (entry.second.first > 1e-6) { solver.VerboseWrite(4, "    Jacobian check " + entry.first + ": largest difference " + EXUstd::ToString(entry.second.first) + " at " + entry.second.second + "\n"); }
	}
}

bool CSolverImplicitRungeKutta::Step(CSolverImplicitSecondOrderTimeInt& solver, CSystem& computationalSystem, const SimulationSettings& simulationSettings)
{
	const NewtonSettings& newton = simulationSettings.timeIntegration.newton;
	const Index s = method.s;
	CSystemData& systemData = computationalSystem.GetSystemData();
	const CSystemState& start = systemData.GetCData().startOfStepState;
	CSystemState& current = systemData.GetCData().currentState;
	const Real h = solver.it.currentStepSize;
	const Real t1 = solver.it.currentTime;
	const Real t0 = t1 - h;
	rejectedByError = false;
	solver.conv.newtonConverged = false;
	solver.conv.newtonSolutionDiverged = false;
	solver.conv.linearSolverFailed = false;

	q0.CopyFrom(start.ODE2Coords); v0.CopyFrom(start.ODE2Coords_t); a0.CopyFrom(start.ODE2Coords_tt);
	y0.CopyFrom(start.ODE1Coords); yd0.CopyFrom(start.ODE1Coords_t); z0.CopyFrom(start.AECoords);
	for (Index i = 0; i < s; i++)
	{
		stageA[i].CopyFrom(a0); stageYd[i].CopyFrom(yd0); stageZ[i].CopyFrom(z0); stageMu[i].SetAll(0.);
		qIterate[i].CopyFrom(q0);
		qIterate[i].MultAdd(h * method.c[i], v0);
		qIterate[i].MultAdd(0.5 * EXUstd::Square(h * method.c[i]), a0);
	}

	//a position constraint is evaluated to the rounding of the coordinates, which the rows of 1/h^2 turn into increments of
	//the accelerations of eps |q| / h^2 per iteration; the Newton iteration does not follow them (#3307)
	const Vector& qRef = systemData.GetCData().referenceState.ODE2Coords;
	Real qMax = 1.;
	for (Index j = 0; j < q0.NumberOfItems(); j++) { qMax = EXUstd::Maximum(qMax, fabs(qRef[j] + q0[j])); }
	constraintRoundoff = 16. * std::numeric_limits<Real>::epsilon() * qMax;

	Real errorFactor = (newton.weightTolerancePerCoordinate && nUnknowns) ? sqrt((Real)nUnknowns) : 1.;
	ComputeRowScale(solver, h);
	bool converged = false;
	bool factorized = false;
	Real initialResidual = -1.;
	Real residualNorm = 0.;
	for (Index iteration = 0; iteration < newton.maxIterations; iteration++)
	{
		if (!factorized || !newton.useModifiedNewton)
		{
			ComputeJacobian(solver, computationalSystem, simulationSettings, t0, h);
			solver.it.newtonJacobiCount++;
			if (solver.IsVerboseCheck(4)) { CheckJacobian(solver, computationalSystem, simulationSettings, t0, h); } //before the factorization
			if (stageMatrix->FactorizeNew() != -1) { solver.conv.linearSolverFailed = true; break; } //singular: as not converged
			factorized = true;
			ComputeRowScale(solver, h); //the special rows of the multipliers are known now
		}
		residualNorm = ComputeResidual(solver, computationalSystem, simulationSettings, t0, h);
		if (!std::isfinite(residualNorm)) { solver.conv.newtonSolutionDiverged = true; break; }
		if (initialResidual < 0.) { initialResidual = EXUstd::Maximum(residualNorm / errorFactor, 2. * newton.absoluteTolerance); }
		stageMatrix->Solve(residual, increment);
		if (method.ggl) { for (Index i : implicitStages) { qIterate[i].CopyFrom(Q[i]); } }
		const Index n2 = solver.data.nODE2, n1 = solver.data.nODE1, na = solver.data.nAE;
		for (Index k = 0; k < implicitStages.NumberOfItems(); k++)
		{
			Index i = implicitStages[k];
			for (Index j = 0; j < n2; j++) { stageA[i][j] -= increment[offA[k] + j]; }
			for (Index j = 0; j < n1; j++) { stageYd[i][j] -= increment[offY[k] + j]; }
			for (Index j = 0; j < na; j++) { stageZ[i][j] -= increment[offZ[k] + j]; }
			if (method.ggl) { for (Index j = 0; j < na; j++) { stageMu[i][j] -= (h * h) * increment[offMu[k] + j]; } }
		}
		solver.it.newtonSteps++;
		solver.it.newtonStepsCount++;
		residualNorm = ComputeResidual(solver, computationalSystem, simulationSettings, t0, h) / errorFactor;
		if (solver.IsVerboseCheck(2)) { solver.VerboseWrite(2, "    " + STDstring(method.name) + " Newton iteration " + EXUstd::ToString(iteration + 1) + ": residual = " + EXUstd::ToString(residualNorm) + " (initial " + EXUstd::ToString(initialResidual) + ")\n"); }
		if (residualNorm / initialResidual <= newton.relativeTolerance || residualNorm <= newton.absoluteTolerance)
		{
			converged = true;
			break;
		}
	}
	solver.conv.residual = residualNorm;
	if (!converged)
	{
		if (solver.IsVerboseCheck(1))
		{
			solver.VerboseWrite(1, "  " + STDstring(method.name) + ": Newton did not converge at t = " + EXUstd::ToString(t1) + ", residual = "
				+ EXUstd::ToString(residualNorm) + " (initial " + EXUstd::ToString(initialResidual) + ")\n");
		}
		current.time = t1;
		return false;
	}
	solver.conv.newtonConverged = true;

	//stiffly accurate: the end of the step is the last stage
	ComputeStages(solver, computationalSystem, t0, h);
	SetStageState(computationalSystem, s - 1, t0, h);
	current.time = t1;

	if (solver.it.automaticStepSize)
	{
		Real order = 3.;
		Real error = ErrorEstimate(solver, simulationSettings, t0, h, order);
		solver.it.automaticStepSizeError = error;
		Real hNext;
		bool minimumReached;
		if (!controller.Control(error, h, order, hNext, minimumReached))
		{
			rejectedByError = true;
			rejectedStepsError++;
			solver.it.rejectedAutomaticStepSizeSteps++;
			solver.it.currentStepSize = hNext;
			nextStepSize = -1.;
			return false;
		}
		if (minimumReached && !minStepSizeWarned)
		{
			PyWarning(STDstring(method.name) + ": t=" + EXUstd::ToString(t0) + "; the automatic step size reached timeIntegration.minimumStepSize; the error is above the tolerances");
			minStepSizeWarned = true;
		}
		nextStepSize = hNext;
	}
	//the stages of this step, which becomes the previous one when it is accepted
	candidateT0 = t0;
	candidateH = h;
	for (Index j = 0; j < solver.data.nODE2; j++) { candidateMeanA[j] = (V[s - 1][j] - v0[j]) / h; }
	for (Index j = 0; j < s; j++) { candidateV[j].CopyFrom(V[j]); candidateA[j].CopyFrom(stageA[j]); }
	return true;
}

void CSolverImplicitRungeKutta::AcceptStep()
{
	if (candidateH <= 0.) { return; } //the initial state
	previousT0 = candidateT0;
	previousH = candidateH;
	meanA2.CopyFrom(meanA1); //the mean accelerations of the last two steps (#3287)
	meanA1.CopyFrom(candidateMeanA);
	meanH2 = meanH1;
	meanH1 = candidateH;
	numberOfMeans = EXUstd::Minimum(numberOfMeans + 1, (Index)2);
	for (Index j = 0; j < method.s; j++) { previousV[j].CopyFrom(candidateV[j]); previousA[j].CopyFrom(candidateA[j]); }
	hasPrevious = true;
	candidateH = 0.;
}
