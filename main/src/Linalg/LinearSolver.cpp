/** ***********************************************************************************************
* @brief		implementation for linear solver matrix classes
*
* @author		Gerstmayr Johannes
* @date			2019-12-29 (generated)
* @date			2019-12-29 (last modified)
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#ifdef _MSC_VER
#pragma warning(disable : 4996) //warning deprecated of Eigen2020
#endif

//BasicLinalg provides consistent includes for BasicDefinitions, arrays, vectors and matrices
#include "Linalg/LinearSolver.h"	
#include "Utilities/TimerStructure.h" //for local CPU time measurement


//! factorize matrix (invert, SparseLU, etc.); -1=success
Index GeneralMatrixEXUdense::FactorizeNew(bool ignoreRedundantEquation, Index redundantEquationsStart)
{
	static ResizableMatrix m;
	static ArrayIndex rows;
	Real pivotTreshold = 0;
	Index  rv = matrix.InvertSpecial(m, rows, ignoreRedundantEquation, redundantEquationsStart, pivotTreshold);
	if (rv == -1) 
	{ 
		SetMatrixIsFactorized(true); 
	}
	else 
	{ 
		SetMatrixIsFactorized(false); 
	}
	return rv;
	//bool rv = !matrix.Invert();
	//if (!rv)
	//{
	//	SetMatrixIsFactorized(true);
	//	return -1;
	//}
	//else
	//{
	//	SetMatrixIsFactorized(false);
	//	return 0;
	//}
}



#ifdef USE_EIGEN_SPARSE_SOLVER

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! set all matrix items to zero (in dense matrix, all entries are set 0, in sparse matrix, the vector of items is erased)
void GeneralMatrixEigenSparse::SetAllZero()
{
	SetMatrixIsFactorized(false);
	SetMatrixBuiltFromTriplets(false);

	triplets.resize(0); //this removes all entries!
	matrix.setZero();	//flush the Eigen sparse matrix
}

//! reset matrices and free memory
void GeneralMatrixEigenSparse::Reset()
{
	//no easy way to do free memory: create simplistic problem to reduce memory
	SetAllZero();
	SetNumberOfRowsAndColumns(1, 1);
	triplets.push_back(EigenTriplet(0, 0, 1.));
	FinalizeMatrix();
	FactorizeNew(); //now solver should be reset to much smaller size ==> test!

	SetAllZero();
	triplets.shrink_to_fit(); //this erases the data if it has zero entries
	matrix.data().squeeze();
}

//! multiply either triplets or matrix entries with factor
void GeneralMatrixEigenSparse::MultiplyWithFactor(Real factor)
{
	SetMatrixIsFactorized(false);
	if (IsMatrixBuiltFromTriplets())
	{
		matrix *= factor;
	}
	else //work on triplets
	{
		for (auto& item : triplets)
		{
			item = EigenTriplet(item.row(), item.col(), factor*item.value());
		}
	}

}

//! set the matrix with a dense matrix; do not use this function for computational tasks, as it will drop performance significantly
void GeneralMatrixEigenSparse::SetMatrix(const Matrix& otherMatrix)
{
	SetMatrixIsFactorized(false);
	SetMatrixBuiltFromTriplets(false);

	triplets.resize(0); //this removes all entries!
	matrix.setZero();	//flush the Eigen sparse matrix

	for (Index i = 0; i < otherMatrix.NumberOfRows(); i++)
	{
		for (Index j = 0; j < otherMatrix.NumberOfColumns(); j++)
		{
			Real value = otherMatrix(i, j);
			if (value != 0.)
			{
				triplets.push_back(EigenTriplet((StorageIndex)i, (StorageIndex)j, value));
			}
		}
	}

}

//! add a diagonal (or unit) matrix located at a certain (rowOffset, columnOffset) position
void GeneralMatrixEigenSparse::AddDiagonalMatrix(Real diagValue, Index numberOfRowsColumns, Index rowOffset, Index columnOffset)
{
	CHECKandTHROW(!IsMatrixBuiltFromTriplets(), "GeneralMatrixEigenSparse::AddDiagonalMatrix(...): only possible in triplet mode!");

	SetMatrixIsFactorized(false);
	if (diagValue != 0.)
	{
		for (Index i = 0; i < numberOfRowsColumns; i++)
		{
			triplets.push_back(EigenTriplet((StorageIndex)(rowOffset + i), (StorageIndex)(columnOffset + i), diagValue));
		}
	}
}


//! add (possibly) smaller factor*Matrix to this matrix, transforming the row indices of the submatrix with LTGrows and the column indices with LTGcolumns; 
//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
//! the offsets are with respect to the indices calculated from the LTGrows/columns transformation
void GeneralMatrixEigenSparse::AddSubmatrix(const Matrix& submatrix, Real factor, const ArrayIndex& LTGrows, const ArrayIndex& LTGcolumns, Index rowOffset, Index columnOffset)
{
	//only allowed in triplet mode:
	CHECKandTHROW(!IsMatrixBuiltFromTriplets(), "GeneralMatrixEigenSparse::AddSubmatrix(const Matrix&, const ArrayIndex& LTGrows, const ArrayIndex& LTGcolumns, ...): only possible in triplet mode!");

	if (factor == 1. && rowOffset == 0 && columnOffset == 0)
	{
		for (Index i = 0; i < submatrix.NumberOfRows(); i++)
		{
			for (Index j = 0; j < submatrix.NumberOfColumns(); j++)
			{
				Real value = submatrix(i, j);
				if (value != 0.)
				{
					triplets.push_back(EigenTriplet((StorageIndex)LTGrows[i], (StorageIndex)LTGcolumns[j], value));
				}
			}
		}
	}
	else
	{
		for (Index i = 0; i < submatrix.NumberOfRows(); i++)
		{
			for (Index j = 0; j < submatrix.NumberOfColumns(); j++)
			{
				Real value = submatrix(i, j);
				if (value != 0.)
				{
					triplets.push_back(EigenTriplet((StorageIndex)(LTGrows[i] + rowOffset), (StorageIndex)(LTGcolumns[j] + columnOffset), factor*value));
				}
			}
		}
	}
}

//! add (possibly) smaller factor*Transposed(Matrix) to this matrix, transforming the row indices of the submatrix with LTGrows and the column indices with LTGcolumns; 
//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
//! the offsets are with respect to the indices calculated from the LTGrows/columns transformation
void GeneralMatrixEigenSparse::AddSubmatrixTransposed(const Matrix& submatrix, Real factor, const ArrayIndex& LTGrows, const ArrayIndex& LTGcolumns, Index rowOffset, Index columnOffset)
{
	//only allowed in triplet mode:
	CHECKandTHROW(!IsMatrixBuiltFromTriplets(), "GeneralMatrixEigenSparse::AddSubmatrix(const Matrix&, const ArrayIndex& LTGrows, const ArrayIndex& LTGcolumns, ...): only possible in triplet mode!");

	if (factor == 1. && rowOffset == 0 && columnOffset == 0)
	{
		for (Index j = 0; j < submatrix.NumberOfRows(); j++)
		{
			for (Index i = 0; i < submatrix.NumberOfColumns(); i++)
			{
				Real value = submatrix(j, i);
				if (value != 0.)
				{
					triplets.push_back(EigenTriplet((StorageIndex)LTGrows[i], (StorageIndex)LTGcolumns[j], value));
				}
			}
		}
	}
	else
	{
		for (Index j = 0; j < submatrix.NumberOfRows(); j++)
		{
			for (Index i = 0; i < submatrix.NumberOfColumns(); i++)
			{
				Real value = submatrix(j, i);
				if (value != 0.)
				{
					triplets.push_back(EigenTriplet((StorageIndex)(LTGrows[i] + rowOffset), (StorageIndex)(LTGcolumns[j] + columnOffset), factor*value));
				}
			}
		}
	}
}

//! add (possibly) smaller factor*Matrix to this matrix; 
//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
void GeneralMatrixEigenSparse::AddSubmatrixWithFactor(const Matrix& submatrix, Real factor, Index rowOffset, Index columnOffset)
{
	//only allowed in triplet mode:
	CHECKandTHROW(!IsMatrixBuiltFromTriplets(), "GeneralMatrixEigenSparse::AddSubmatrixWithFactor(...): only possible in triplet mode!");
	for (Index i = 0; i < submatrix.NumberOfRows(); i++)
	{
		for (Index j = 0; j < submatrix.NumberOfColumns(); j++)
		{
			Real value = submatrix(i, j);
			if (value != 0.)
			{
				triplets.push_back(EigenTriplet((StorageIndex)(i + rowOffset), (StorageIndex)(j + columnOffset), factor*value));
			}
		}
	}

}

//! add (possibly) smaller factor*Transposed(Matrix) to this matrix; 
//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
void GeneralMatrixEigenSparse::AddSubmatrixTransposedWithFactor(const Matrix& submatrix, Real factor, Index rowOffset, Index columnOffset)
{
	//only allowed in triplet mode:
	CHECKandTHROW(!IsMatrixBuiltFromTriplets(), "GeneralMatrixEigenSparse::AddSubmatrixTransposedWithFactor(...): only possible in triplet mode!");
	for (Index j = 0; j < submatrix.NumberOfRows(); j++)
	{
		for (Index i = 0; i < submatrix.NumberOfColumns(); i++)
		{
			Real value = submatrix(j, i);
			if (value != 0.)
			{
				triplets.push_back(EigenTriplet((StorageIndex)(i + rowOffset), (StorageIndex)(j + columnOffset), factor*value));
			}
		}
	}
}


//! add possibly GeneralMatrix to this matrix; in case of sparse matrices, only the triplets (row,col,value) are added
//! matrix types of submatrix and *this must be same
//! operations must be both in triplet mode!
void GeneralMatrixEigenSparse::AddSubmatrix(const GeneralMatrix& submatrix, Index rowOffset, Index columnOffset)
{
	CHECKandTHROW((GetSystemMatrixType() == submatrix.GetSystemMatrixType()), "GeneralMatrixEigenSparse::AddSubmatrix: invalid SystemMatrixType!");
	CHECKandTHROW(!IsMatrixBuiltFromTriplets(), "GeneralMatrixEigenSparse::AddSubmatrix(const GeneralMatrix&, ...): matrix must be in triplet mode !");

	const GeneralMatrixEigenSparse& m = (const GeneralMatrixEigenSparse&)submatrix;
	CHECKandTHROW(!m.IsMatrixBuiltFromTriplets(), "GeneralMatrixEigenSparse::AddSubmatrix(const GeneralMatrix&, ...): matrix must be in triplet mode !");

	SetMatrixIsFactorized(false);

	if ((rowOffset != 0) || (columnOffset != 0))
	{
		for (const EigenTriplet& item : m.GetEigenTriplets())
		{
			if (item.value() != 0.)
			{
				triplets.push_back(EigenTriplet(item.row() + (StorageIndex)rowOffset, item.col() + (StorageIndex)columnOffset, item.value()));
			}
		}
	}
	else //faster mode, no offsets ...
	{
		for (const EigenTriplet& item : m.GetEigenTriplets())
		{
			if (item.value() != 0.)
			{
				triplets.push_back(item); //in this case, just add the items of submatrix
			}
		}
	}
}

//! add column vector 'vec' at 'column'; used to add a couple of entries during jacobian computation; filters zeros in sparse mode
void GeneralMatrixEigenSparse::AddColumnVector(Index column, const Vector& vec, Index rowOffset)
{
	CHECKandTHROW(!IsMatrixBuiltFromTriplets(), "GeneralMatrixEigenSparse::AddColumnVector(...): matrix must be in triplet mode !");

	if (!rowOffset)
	{
		for (Index i = 0; i < vec.NumberOfItems(); i++) //i = row
		{
			Real value = vec[i];
			if (value != 0.)
			{
				triplets.push_back(EigenTriplet((StorageIndex)i, (StorageIndex)column, value));
			}
		}
	}
	else
	{
		for (Index i = 0; i < vec.NumberOfItems(); i++) //i = row
		{
			Real value = vec[i];
			if (value != 0.)
			{
				triplets.push_back(EigenTriplet((StorageIndex)(i+rowOffset), (StorageIndex)column, value));
			}
		}
	}
}

//! After filling the matrix, it is finalized for further operations (matrix*vector, factorization, ...)
void GeneralMatrixEigenSparse::FinalizeMatrix()
{
	SetMatrixIsFactorized(false);

	if (matrix.nonZeros() != 0) { matrix.setZero(); } //this should be already done in matrix.resize - could be omitted ...?
	matrix.resize(NumberOfRows(), NumberOfColumns());
	matrix.setFromTriplets(triplets.begin(), triplets.end()); //sums up duplicates by default... (@TODO: what happens in EigenSparseMatrix::setFromTriplets(...) with (+1) + (-1) ? )

	SetMatrixBuiltFromTriplets(); //now the sparse matrix is finally set and ready for multiplication and factorization
}

//Index TSeigenFactorize;
//TimerStructureRegistrator TSReigenFactorize("eigenFactorize", TSeigenFactorize, globalTimers);
//Index TSeigenAnalyzePattern;
//TimerStructureRegistrator TSReigenAnalyzePattern("eigenAnalyzePattern", TSeigenAnalyzePattern, globalTimers);

//! factorize matrix (invert, SparseLU, etc.); -1=success
Index GeneralMatrixEigenSparse::FactorizeNew(bool ignoreRedundantEquation, Index redundantEquationsStart)
{
	CHECKandTHROW(IsMatrixBuiltFromTriplets(), "GeneralMatrixEigenSparse::Factorize(): matrix must be built before factorization!");

	//STARTGLOBALTIMER(TSeigenAnalyzePattern);
	solver.analyzePattern(matrix);
	//STOPGLOBALTIMER(TSeigenAnalyzePattern);

	// Compute the numerical factorization 
	//STARTGLOBALTIMER(TSeigenFactorize);
	solver.factorize(matrix);
	//STOPGLOBALTIMER(TSeigenFactorize);

	//0: successful factorization
	//if info = i, and i is
	//<= A->ncol : U(i, i) is exactly zero.The factorization has been completed, but the factor U is exactly singular, and division by zero will occur if it is used to solve a system of equations.
	//> A->ncol: number of bytes allocated when memory allocation failure occurred, plus A->ncol.If lwork = -1, it is the estimated amount of space needed, plus A->ncol.
	Index rv = solver.info();
	if (!rv) { SetMatrixIsFactorized(true); return -1; }
	else if (rv <= NumberOfRows()) { return rv - 1;  } //causing row
	else { return NumberOfRows(); } //undefined error
}

//! multiply matrix with vector: solution = A*x
//! this leads to memory allocation in case that the matrix is built from triplets
void GeneralMatrixEigenSparse::MultMatrixVector(const Vector& x, Vector& solution)
{
	if (IsMatrixIsFactorized()) { SysError("GeneralMatrixEigenSparse::MultMatrixVector(...): matrix is already factorized ==> use Solve(...)!"); }
	Index nRows = NumberOfRows();
	Index nColumns = NumberOfColumns();
	solution.SetNumberOfItems(nRows);

	if (IsMatrixBuiltFromTriplets())
	{
		//this function could be optimized, by accessing directly the non zero entries of the sparse matrix:
		//Eigen::Map<Eigen::VectorXd> xEigen(x.GetDataPointer(), n);
		//Eigen::Map<Eigen::VectorXd> solutionEigen(solution.GetDataPointer(), n);

		//the following way invokes memory allocation
		Eigen::VectorXd xEigen(nColumns);
		Eigen::VectorXd solutionEigen(nRows);
		for (Index i = 0; i < nColumns; i++)
		{
			xEigen[i] = x[i];
		}
		solutionEigen = matrix * xEigen;

		for (Index i = 0; i < nRows; i++)
		{
			solution[i] = solutionEigen[i];
		}
	}
	else //work on triplets; no memory allocation
	{
		solution.SetAll(0.);

		for (const auto& item : triplets)
		{
			solution[item.row()] += x[item.col()] * item.value();
		}
	}
}

//! multiply matrix with vector and add to solution: solution += A*x
//! this leads to memory allocation in case that the matrix is built from triplets
void GeneralMatrixEigenSparse::MultMatrixVectorAdd(const Vector& x, Vector& solution)
{
	if (IsMatrixIsFactorized()) { SysError("GeneralMatrixEigenSparse::MultMatrixVector(...): matrix is already factorized ==> use Solve(...)!"); }
	Index nRows = NumberOfRows();
	Index nColumns = NumberOfColumns();
	CHECKandTHROW(solution.NumberOfItems() == nRows, "GeneralMatrixEigenSparse::MultMatrixVectorAdd(...): matrix number of rows must be equal to size of solution vector");

	if (IsMatrixBuiltFromTriplets())
	{
		//this function could be optimized, by accessing directly the non zero entries of the sparse matrix:
		//Eigen::Map<Eigen::VectorXd> xEigen(x.GetDataPointer(), n);
		//Eigen::Map<Eigen::VectorXd> solutionEigen(solution.GetDataPointer(), n);

		//the following way invokes memory allocation
		Eigen::VectorXd xEigen(nColumns);
		Eigen::VectorXd solutionEigen(nRows);
		for (Index i = 0; i < nColumns; i++)
		{
			xEigen[i] = x[i];
		}
		solutionEigen = matrix * xEigen;

		for (Index i = 0; i < nRows; i++)
		{
			solution[i] += solutionEigen[i];
		}
	}
	else //work on triplets; no memory allocation
	{
		for (const auto& item : triplets)
		{
			solution[item.row()] += x[item.col()] * item.value();
		}
	}
}

//! multiply transposed(matrix) with vector: solution = A^T*x
//! this leads to memory allocation in case that the matrix is built from triplets
void GeneralMatrixEigenSparse::MultMatrixTransposedVector(const Vector& x, Vector& solution)
{
	if (IsMatrixIsFactorized()) { SysError("GeneralMatrixEigenSparse::MultMatrixTransposedVector(...): matrix is already factorized ==> use Solve(...)!"); }
	Index nRows = NumberOfRows();
	//Index nColumns = NumberOfColumns();

	if (IsMatrixBuiltFromTriplets())
	{
		SysError("GeneralMatrixEigenSparse::MultMatrixTransposedVector(...): currently only possible in triplet mode!");
	}
	else //work on triplets; 
	{
		solution.SetNumberOfItems(nRows);
		solution.SetAll(0.);

		for (const auto& item : triplets)
		{
			solution[item.col()] += x[item.row()] * item.value();
		}
	}
}

//! after factorization of matrix (=A), solve provides a solution vector (=x) for A*x = rhs ==> soluation = A^{-1}*rhs
void GeneralMatrixEigenSparse::Solve(const Vector& rhs, Vector& solution)
{
	CHECKandTHROW(IsMatrixIsFactorized(), "GeneralMatrixEigenSparse::Solve( ...): matrix is not factorized!");

	//will only work for Real==double!!! ==> make type check!
	Real test=0;
	Real* testPtr = &test;
	double* testDouble = static_cast<double*>(testPtr); // this will not compile in case of Real==float, for which the following functions need to be changed (use VectorXf)!

	//n must be same as number of columns:
	Index n = NumberOfRows()+(Index)(*testDouble);  //add (Index)(*testDouble) to suppress gcc warning
	Eigen::VectorXd b = Eigen::Map<Eigen::VectorXd>(rhs.GetDataPointer(), n);

	//Eigen::VectorXd b(n);
	//for (Index i = 0; i < n; i++)
	//{
	//	b[i] = rhs[i];
	//}

	//static Eigen::VectorXd x;
	solution.SetNumberOfItems(n);
	//Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd>(solution.GetDataPointer(), n);

	Eigen::VectorXd x;
	x = solver.solve(b);

	for (Index i = 0; i < n; i++)
	{
		solution[i] = x[i];
	}
}

//! return a dense matrix from any other matrix: requires a copy - SLOW!
ResizableMatrix GeneralMatrixEigenSparse::GetEXUdenseMatrix() const
{
	ResizableMatrix denseMatrix(NumberOfRows(), NumberOfColumns());
	denseMatrix.SetAll(0.);

	if (IsMatrixBuiltFromTriplets())
	{
		//this is very slow for dense matrices (possibly order nCols^2*nRows), but convenient for tests!
		for (Index i = 0; i < NumberOfRows(); i++)
		{
			for (Index j = 0; j < NumberOfColumns(); j++)
			{
				denseMatrix(i, j) = matrix.coeff(i, j);
			}
		}
	}
	else //add all triplets to matrix
	{
		for (auto& item : triplets)
		{
			denseMatrix(item.row(), item.col()) += item.value();
		}
	}
	return denseMatrix;
}

#endif //eigen sparse solver
