/** ***********************************************************************************************
* @file			LinearSolverUnitTests.h
* @brief		Unit tests for LinearSolver.h: the dense and the sparse (Eigen) system matrix
*				behind the one GeneralMatrix interface.
* @details		Details:
*				- the same shape as MatrixContainer in R5.4.1, and the same kind of test: one
*				  system is solved by all FOUR variants - EXUdense, Eigen PartialPivLU, Eigen
*				  FullPivLU and EigenSparse - and they must agree. The check is the property
*				  A*x == rhs, not a recorded solution vector, so it survives a rewrite.
*				- where they do NOT agree is at least as important: only EXUdense reports a singular
*				  matrix by its causing row, the two Eigen dense paths report success and return a
*				  least-squares answer - which is what ignoreSingularJacobian is FOR, see the case -
*				  and EXUdense overwrites its own matrix with the INVERSE while factorizing.
*				- negative tests use the throwing paths only. Misusing the dense matrix produces a
*				  SysError, which prints, continues and sets the process-global
*				  globalPyRuntimeErrorFlag - a unit test must not leave that behind.
*
* @author		Gerstmayr Johannes
* @date			2026-09-17 (#2479)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
************************************************************************************************ */
#ifndef LINEARSOLVERUNITTESTS__H
#define LINEARSOLVERUNITTESTS__H

//not reached by UnitTestBase.cpp otherwise; GeneralMatrix lives in the GLOBAL namespace
#include "Linalg/LinearSolver.h"

//a system that is well conditioned and not symmetric, so that a transposed index shows up
inline Matrix LinearSolverTestMatrix()
{
	return Matrix(4, 4, { 4.,  1.,  0.,  2.,
						  1.,  5.,  2.,  0.,
						  0.,  2.,  6.,  1.,
						  2.,  0.,  1.,  3. });
}

inline Vector LinearSolverTestRhs() { return Vector({ 1., -2., 3., 0.5 }); }

//! fill 'matrix' with 'values' the way a solver does, and leave it factorized and ready to solve
inline Index LinearSolverBuildAndFactorize(GeneralMatrix& matrix, const Matrix& values)
{
	matrix.SetNumberOfRowsAndColumns(values.NumberOfRows(), values.NumberOfColumns());
	matrix.SetAllZero();
	matrix.SetMatrix(values);
	matrix.FinalizeMatrix();
	return matrix.FactorizeNew();
}

//! the residual of a solution, computed with plain linear algebra rather than with the solver
inline Real LinearSolverResidual(const Matrix& values, const Vector& solution, const Vector& rhs)
{
	Real maximum = 0.;
	for (Index i = 0; i < values.NumberOfRows(); i++)
	{
		Real sum = 0.;
		for (Index j = 0; j < values.NumberOfColumns(); j++) { sum += values(i, j) * solution[j]; }
		maximum = EXUstd::Maximum(maximum, fabs(sum - rhs[i]));
	}
	return maximum;
}

const lest::test linearSolver_specific_test[] =
{

	CASE("LinearSolver: all four variants solve the same system")
	{
		//EXUdense, Eigen PartialPivLU, Eigen FullPivLU and EigenSparse; the property is A*x == rhs
		const Matrix values = LinearSolverTestMatrix();
		const Vector rhs = LinearSolverTestRhs();
		const Index size = values.NumberOfRows();

		Vector solutions[4];
		for (Index mode = 0; mode <= 2; mode++)
		{
			GeneralMatrixEXUdense dense;
			dense.UseEigenSolverType() = mode;
			EXPECT(LinearSolverBuildAndFactorize(dense, values) == -1); //-1 means success
			EXPECT(dense.IsMatrixIsFactorized());

			//the Eigen paths map the result vector by its CURRENT size, so the caller must size it
			solutions[mode] = Vector(size);
			solutions[mode].SetAll(0.);
			dense.Solve(rhs, solutions[mode]);
			EXPECT(LinearSolverResidual(values, solutions[mode], rhs) < 1e-13);
		}

		GeneralMatrixEigenSparse sparse;
		EXPECT(LinearSolverBuildAndFactorize(sparse, values) == -1);
		solutions[3] = Vector(size);
		solutions[3].SetAll(0.);
		sparse.Solve(rhs, solutions[3]);
		EXPECT(LinearSolverResidual(values, solutions[3], rhs) < 1e-13);

		//and the four answers are the same answer
		for (Index mode = 1; mode <= 3; mode++)
		{
			for (Index i = 0; i < size; i++)
			{
				EXPECT(fabs(solutions[mode][i] - solutions[0][i]) < 1e-12);
			}
		}
	},

	CASE("LinearSolver: the identity solves to the right hand side itself")
	{
		//the simplest case there is, and the one that catches a transposed or shifted index
		const Vector rhs = LinearSolverTestRhs();
		const Index size = rhs.NumberOfItems();

		GeneralMatrixEXUdense dense;
		dense.SetNumberOfRowsAndColumns(size, size);
		dense.SetAllZero();
		dense.AddDiagonalMatrix(1., size);
		dense.FinalizeMatrix();
		EXPECT(dense.FactorizeNew() == -1);

		Vector solution(size);
		solution.SetAll(0.);
		dense.Solve(rhs, solution);
		for (Index i = 0; i < size; i++) { EXPECT(fabs(solution[i] - rhs[i]) < 1e-14); }

		GeneralMatrixEigenSparse sparse;
		sparse.SetNumberOfRowsAndColumns(size, size);
		sparse.SetAllZero();
		sparse.AddDiagonalMatrix(1., size);
		sparse.FinalizeMatrix();
		EXPECT(sparse.FactorizeNew() == -1);

		Vector sparseSolution(size);
		sparseSolution.SetAll(0.);
		sparse.Solve(rhs, sparseSolution);
		for (Index i = 0; i < size; i++) { EXPECT(fabs(sparseSolution[i] - rhs[i]) < 1e-14); }
	},

	CASE("LinearSolver: dense and sparse hold the SAME matrix, built the same way")
	{
		//before any factorization, both must hand out the same dense matrix - this is what makes
		//the two interchangeable for the solver
		const Matrix values = LinearSolverTestMatrix();
		const Index size = values.NumberOfRows();

		GeneralMatrixEXUdense dense;
		dense.SetNumberOfRowsAndColumns(size, size);
		dense.SetAllZero();
		dense.AddSubmatrixWithFactor(values, 1.);
		dense.FinalizeMatrix();

		GeneralMatrixEigenSparse sparse;
		sparse.SetNumberOfRowsAndColumns(size, size);
		sparse.SetAllZero();
		sparse.AddSubmatrixWithFactor(values, 1.);
		sparse.FinalizeMatrix();

		EXPECT(dense.NumberOfRows() == sparse.NumberOfRows());
		EXPECT(dense.NumberOfColumns() == sparse.NumberOfColumns());

		ResizableMatrix denseMatrix = dense.GetEXUdenseMatrix();
		ResizableMatrix sparseMatrix = sparse.GetEXUdenseMatrix();
		for (Index i = 0; i < size; i++)
		{
			for (Index j = 0; j < size; j++)
			{
				EXPECT(denseMatrix(i, j) == values(i, j));
				EXPECT(sparseMatrix(i, j) == values(i, j));
			}
		}

		//adding twice adds twice, in both modes: the triplets accumulate, they do not replace
		dense.AddSubmatrixWithFactor(values, 1.);
		sparse.SetMatrixBuiltFromTriplets(false); //triplet mode is required for a further Add
		sparse.AddSubmatrixWithFactor(values, 1.);
		sparse.FinalizeMatrix();
		EXPECT(dense.GetEXUdenseMatrix()(0, 0) == 2.*values(0, 0));
		EXPECT(sparse.GetEXUdenseMatrix()(0, 0) == 2.*values(0, 0));
	},

	CASE("LinearSolver: EXUdense reports WHICH row made the matrix singular")
	{
		//a zero row is exactly singular, and the default PivotThreshold() of 0. catches exactly
		//that: 'fabs(pivot) <= 0.' is true only for a pivot that is exactly zero
		Matrix values = LinearSolverTestMatrix();
		for (Index j = 0; j < values.NumberOfColumns(); j++) { values(2, j) = 0.; } //row 2 is dead

		GeneralMatrixEXUdense dense;
		dense.UseEigenSolverType() = 0;
		const Index returnValue = LinearSolverBuildAndFactorize(dense, values);

		EXPECT(returnValue != -1);                    //not -1: the factorization failed
		EXPECT(returnValue >= 0);
		EXPECT(returnValue < values.NumberOfRows());  //and it names a row
		EXPECT(!dense.IsMatrixIsFactorized());        //the flag stays false, so Solve is refused
	},

	CASE("LinearSolver: the two Eigen DENSE paths cannot report a singular matrix at all")
	{
		//this is DELIBERATE, and the case exists so that it cannot change silently: FullPivLU is the
		//linearSolverSettings.ignoreSingularJacobian=True path, whose documented purpose is to
		//resolve over- and underdetermined systems and redundant constraints by least squares
		//("in this case, we could report errors, but we do not want to"), and PartialPivLU offers no
		//invertibility check at all - a property of Eigen, not a decision of Exudyn
		Matrix values = LinearSolverTestMatrix();
		for (Index j = 0; j < values.NumberOfColumns(); j++) { values(2, j) = 0.; }

		for (Index mode = 1; mode <= 2; mode++)
		{
			GeneralMatrixEXUdense dense;
			dense.UseEigenSolverType() = mode;
			EXPECT(LinearSolverBuildAndFactorize(dense, values) == -1); //reports SUCCESS
			EXPECT(dense.IsMatrixIsFactorized());
		}

		//the sparse path DOES report the failure, and says 'causing row unknown' by returning
		//NumberOfRows(): Eigen's SparseLU reports a status, not a row. Until #2482 it returned that
		//status minus one, so the solver printed "causing system equation number = 0" every time
		GeneralMatrixEigenSparse sparse;
		const Index sparseReturn = LinearSolverBuildAndFactorize(sparse, values);
		EXPECT(sparseReturn != -1);
		EXPECT(sparseReturn == sparse.NumberOfRows()); //the documented 'no row known' answer
		EXPECT(!sparse.IsMatrixIsFactorized());
	},

	CASE("LinearSolver: EXUdense replaces its own matrix by the INVERSE while factorizing")
	{
		//a trap for anyone reading the matrix back after a solve: InvertSpecial() writes the
		//inverse into the same storage. Only mode 0 does this; the Eigen paths keep A
		const Matrix values = LinearSolverTestMatrix();
		const Index size = values.NumberOfRows();

		GeneralMatrixEXUdense dense;
		dense.UseEigenSolverType() = 0;
		EXPECT(LinearSolverBuildAndFactorize(dense, values) == -1);

		ResizableMatrix afterFactorization = dense.GetEXUdenseMatrix();
		//it is the inverse: A * A^-1 must be the identity
		for (Index i = 0; i < size; i++)
		{
			for (Index j = 0; j < size; j++)
			{
				Real sum = 0.;
				for (Index k = 0; k < size; k++) { sum += values(i, k) * afterFactorization(k, j); }
				EXPECT(fabs(sum - (i == j ? 1. : 0.)) < 1e-13);
			}
		}

		//the Eigen path keeps the matrix it was given
		GeneralMatrixEXUdense eigenDense;
		eigenDense.UseEigenSolverType() = 1;
		EXPECT(LinearSolverBuildAndFactorize(eigenDense, values) == -1);
		EXPECT(eigenDense.GetEXUdenseMatrix()(0, 0) == values(0, 0));
	},

	CASE("LinearSolver: the products agree between the two modes")
	{
		const Matrix values = LinearSolverTestMatrix();
		const Index size = values.NumberOfRows();
		Vector x({ 1., 2., 3., 4. });

		GeneralMatrixEXUdense dense;
		dense.SetNumberOfRowsAndColumns(size, size);
		dense.SetAllZero();
		dense.SetMatrix(values);
		dense.FinalizeMatrix();

		GeneralMatrixEigenSparse sparse;
		sparse.SetNumberOfRowsAndColumns(size, size);
		sparse.SetAllZero();
		sparse.SetMatrix(values);
		sparse.FinalizeMatrix();

		//MultMatrixVector SIZES the result, in both modes
		Vector denseResult, sparseResult;
		dense.MultMatrixVector(x, denseResult);
		sparse.MultMatrixVector(x, sparseResult);
		EXPECT(denseResult.NumberOfItems() == size);
		EXPECT(sparseResult.NumberOfItems() == size);

		for (Index i = 0; i < size; i++)
		{
			Real reference = 0.;
			for (Index j = 0; j < size; j++) { reference += values(i, j) * x[j]; }
			EXPECT(fabs(denseResult[i] - reference) < 1e-14);
			EXPECT(fabs(sparseResult[i] - reference) < 1e-14);
		}

		//MultMatrixVectorAdd accumulates and REQUIRES the caller to have sized the result
		Vector accumulated = denseResult;
		dense.MultMatrixVectorAdd(x, accumulated);
		Vector accumulatedSparse = sparseResult;
		sparse.MultMatrixVectorAdd(x, accumulatedSparse);
		for (Index i = 0; i < size; i++)
		{
			EXPECT(fabs(accumulated[i] - 2.*denseResult[i]) < 1e-14);
			EXPECT(fabs(accumulatedSparse[i] - accumulated[i]) < 1e-13);
		}
	},

	CASE("LinearSolver: the sparse matrix refuses to solve before it is factorized")
	{
		//the sparse path throws, which a test can assert on; the dense path only prints a SysError
		//and sets a process-global flag, so it is deliberately NOT exercised here
		const Matrix values = LinearSolverTestMatrix();
		const Index size = values.NumberOfRows();

		GeneralMatrixEigenSparse sparse;
		sparse.SetNumberOfRowsAndColumns(size, size);
		sparse.SetAllZero();
		sparse.SetMatrix(values);
		sparse.FinalizeMatrix();
		EXPECT(!sparse.IsMatrixIsFactorized());

		Vector rhs = LinearSolverTestRhs();
		Vector solution(size);
		EXPECT_THROWS(sparse.Solve(rhs, solution));

		//and it also refuses to factorize a matrix that was never built from its triplets
		GeneralMatrixEigenSparse unbuilt;
		unbuilt.SetNumberOfRowsAndColumns(size, size);
		unbuilt.SetAllZero();
		unbuilt.SetMatrix(values); //no FinalizeMatrix()
		EXPECT_THROWS(unbuilt.FactorizeNew());
	},

	CASE("LinearSolver: the sparse matrix keeps its OWN size, SetMatrix does not set it")
	{
		//the dense matrix takes its size from the matrix it is given; the sparse one only holds
		//triplets, so its size is whatever SetNumberOfRowsAndColumns said - and every product and
		//factorization depends on that
		const Matrix values = LinearSolverTestMatrix();
		const Index size = values.NumberOfRows();

		GeneralMatrixEXUdense dense;
		dense.SetMatrix(values); //no SetNumberOfRowsAndColumns
		EXPECT(dense.NumberOfRows() == size);
		EXPECT(dense.NumberOfColumns() == size);

		GeneralMatrixEigenSparse sparse;
		sparse.SetMatrix(values); //the triplets are there ...
		EXPECT(sparse.NumberOfRows() == 0);    //... but the matrix says 0 x 0
		EXPECT(sparse.NumberOfColumns() == 0);

		sparse.SetNumberOfRowsAndColumns(size, size);
		EXPECT(sparse.NumberOfRows() == size);
		sparse.FinalizeMatrix();
		EXPECT(sparse.GetEXUdenseMatrix()(1, 1) == values(1, 1)); //and now it is the matrix again
	},

	CASE("LinearSolver: the type each matrix reports")
	{
		//SetLinearSolverType in the solver dispatches on these, so they are part of the interface
		GeneralMatrixEXUdense dense;
		dense.UseEigenSolverType() = 0;
		EXPECT(dense.GetSystemMatrixType() == LinearSolverType::EXUdense);
		dense.UseEigenSolverType() = 1;
		EXPECT(dense.GetSystemMatrixType() == LinearSolverType::EigenDense);

		GeneralMatrixEigenSparse sparse;
		EXPECT(sparse.GetSystemMatrixType() == LinearSolverType::EigenSparse);
		EXPECT(!sparse.IsSymmetric());
		EXPECT(!dense.IsSymmetric()); //the dense matrix never claims to be symmetric

#ifdef USE_SYMMETRIC_SOLVER
		//only a non-release build has the symmetric solver; elsewhere AssumeSymmetric(true) throws
		sparse.AssumeSymmetric(true);
		EXPECT(sparse.IsSymmetric());
		EXPECT(sparse.GetSystemMatrixType() == LinearSolverType::EigenSparseSymmetric);
		sparse.AssumeSymmetric(false);
#else
		EXPECT_THROWS(sparse.AssumeSymmetric(true));
#endif
	},

};

#endif //include header once
