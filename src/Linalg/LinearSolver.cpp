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
#include "Utilities/AdvancedStuff.h"
#include "Utilities/TimerStructure.h" //for local CPU time measurement

#include "Linalg/MatrixContainer.h"	
#include "Utilities/Parallel.h" //for local CPU time measurement

#include "Main/Experimental.h"
extern PyExperimental pyExperimental; //!this class can be accessed from outside, but also from every other file where this is imported

//! factorize matrix (invert, SparseLU, etc.); -1=success; >=0: causing row
Index GeneralMatrixEXUdense::FactorizeNew() //bool ignoreRedundantEquation, Index redundantEquationsStart)
{
    Index  rv;

    if (useEigenSolverType == 0)
    {
        static ResizableMatrix m; //temporary matrix!
        static ArrayIndex rows;
        //rv = matrix.InvertSpecial(m, rows, ignoreRedundantEquation, redundantEquationsStart, pivotThreshold);
        rv = matrix.InvertSpecial(m, rows, false, 0, PivotThreshold());
    }
    else
    {
#ifdef USE_EIGEN_DENSE_SOLVER
        rv = -1; //always gives factorization, both in over- as well as underdetermined case

        //pout << "factorize Eigen\n";
        Eigen::Map<Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > mEigen(matrix.GetDataPointer(), 
            matrix.NumberOfRows(), matrix.NumberOfColumns());
        //lu = m.fullPivLu();
        //eigenLU = Eigen::FullPivLU<Eigen::MatrixXd>(mEigen);

        if (useEigenSolverType == eigenSolverTypeFullPivotLU)
        {
			if (PivotThreshold() == 0)
			{
				eigenLUfullPivot.setThreshold(Eigen::Default);
			}
			else
			{
				eigenLUfullPivot.setThreshold(PivotThreshold());
			}
            eigenLUfullPivot.compute(mEigen);   //full pivoting LU-factorization (very stable and resolves singular Jacobian)
            //in this case, we could report errors, but we do not want to
            //if (!eigenLUfullPivot.isInvertible())
            //{
            //    rv = matrix.NumberOfRows(); //this means that the solver failed, but no causing row known
            //}
            if (pyExperimental.eigenFullPivotLUsolverDebugLevel >= 1)
            {
                pout << "eigenFullPivotLU dim=(" << matrix.NumberOfRows() << "," << matrix.NumberOfColumns() << ")\n";
                pout << "eigenFullPivotLU rank=" << eigenLUfullPivot.rank() << "\n";
                pout << "eigenFullPivotLU isInvertible=" << eigenLUfullPivot.isInvertible() << "\n";
                pout << "eigenFullPivotLU nonzeroPivots=" << eigenLUfullPivot.nonzeroPivots() << "\n";
            }
        }
        else
        {
            //NOTE: according to Eigen's homepage, there is no possibility to check for invertability!
            eigenLU.compute(mEigen);            //regular LU-factorization (fast)
        }


#else
        CHECKandTHROWstring("GeneralMatrixEXUdense::Factorize: code not compiled with Eigen; use other solver");
#endif
    }
    if (rv == -1)
    {
        SetMatrixIsFactorized(true);
    }
    else
    {
        SetMatrixIsFactorized(false);
}
    return rv;
}

//!after factorization of matrix (=A), solve provides a solution vector (=x) for A*x = rhs ==> soluation = A^{-1}*rhs
void GeneralMatrixEXUdense::Solve(const Vector& rhs, Vector& solution)
{
    if (!IsMatrixIsFactorized()) { SysError("GeneralMatrixEXUdense::Solve(...): matrix is not factorized!"); }

    if (!useEigenSolverType)
    {
        EXUmath::MultMatrixVector(matrix, rhs, solution);
    }
    else
    {
#ifdef USE_EIGEN_DENSE_SOLVER
        Eigen::Map<Eigen::VectorXd> rhsEigen(rhs.GetDataPointer(), rhs.NumberOfItems());
        Eigen::Map<Eigen::VectorXd> solutionEigen(solution.GetDataPointer(), solution.NumberOfItems());

        if (useEigenSolverType == eigenSolverTypeFullPivotLU)
        {
            solutionEigen = eigenLUfullPivot.solve(rhsEigen);
        }
        else
        {
            solutionEigen = eigenLU.solve(rhsEigen);
        }

#endif
    }
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

	triplets.SetNumberOfItems(0); //this removes all entries!

//#ifdef useMatrixContainerTriplets
//	triplets.SetNumberOfItems(0); //this removes all entries!
//#else
//	triplets.resize(0); //this removes all entries!
//#endif
	matrix.setZero();	//flush the Eigen sparse matrix
}

//! reset matrices and free memory
void GeneralMatrixEigenSparse::Reset()
{
	SetAllZero();
	triplets.Flush();
	analyzedPatternLastNNZ = 0;
//#ifdef useMatrixContainerTriplets
//	SetAllZero();
//	triplets.Flush();
//#else
//	//no easy way to do free memory: create simplistic problem to reduce memory
//	SetAllZero();
//	SetNumberOfRowsAndColumns(1, 1);
//	triplets.push_back(SparseTriplet(0, 0, 1.));
//	FinalizeMatrix();
//	FactorizeNew(); //now solver should be reset to much smaller size ==> test!
//
//	SetAllZero();
//	triplets.shrink_to_fit(); //this erases the data if it has zero entries
//	matrix.data().squeeze();
//#endif
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
			item = SparseTriplet(item.row(), item.col(), factor*item.value());
		}
	}

}

//! set the matrix with a dense matrix; do not use this function for computational tasks, as it will drop performance significantly
void GeneralMatrixEigenSparse::SetMatrix(const Matrix& otherMatrix)
{
	SetMatrixIsFactorized(false);
	SetMatrixBuiltFromTriplets(false);

	triplets.SetNumberOfItems(0); //this removes all entries!
//#ifdef useMatrixContainerTriplets
//	triplets.SetNumberOfItems(0); //this removes all entries!
//#else
//	triplets.resize(0); //this removes all entries!
//#endif
	matrix.setZero();	//flush the Eigen sparse matrix

	for (Index i = 0; i < otherMatrix.NumberOfRows(); i++)
	{
		for (Index j = 0; j < otherMatrix.NumberOfColumns(); j++)
		{
			Real value = otherMatrix(i, j);
			if (value != 0.)
			{
				triplets.AppendPure(SparseTriplet((StorageIndex)i, (StorageIndex)j, value));
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
			triplets.AppendPure(SparseTriplet((StorageIndex)(rowOffset + i), (StorageIndex)(columnOffset + i), diagValue));
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
					triplets.AppendPure(SparseTriplet((StorageIndex)LTGrows[i], (StorageIndex)LTGcolumns[j], value));
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
					triplets.AppendPure(SparseTriplet((StorageIndex)(LTGrows[i] + rowOffset), (StorageIndex)(LTGcolumns[j] + columnOffset), factor*value));
				}
			}
		}
	}
}

//! add sparse triplets to dense or sparse matrix
void GeneralMatrixEigenSparse::AddSparseTriplets(const SparseTripletVector& otherTriplets)
{
	//only allowed in triplet mode:
	CHECKandTHROW(!IsMatrixBuiltFromTriplets(), "GeneralMatrixEigenSparse::AddSparseTriplets(...): only possible in triplet mode!");

	for (const SparseTriplet& triplet : otherTriplets)
	{
		triplets.AppendPure(triplet);
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
					triplets.AppendPure(SparseTriplet((StorageIndex)LTGrows[i], (StorageIndex)LTGcolumns[j], value));
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
					triplets.AppendPure(SparseTriplet((StorageIndex)(LTGrows[i] + rowOffset), (StorageIndex)(LTGcolumns[j] + columnOffset), factor*value));
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
				triplets.AppendPure(SparseTriplet((StorageIndex)(i + rowOffset), (StorageIndex)(j + columnOffset), factor*value));
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
				triplets.AppendPure(SparseTriplet((StorageIndex)(i + rowOffset), (StorageIndex)(j + columnOffset), factor*value));
			}
		}
	}
}


//! add possibly smaller GeneralMatrix to this matrix; in case of sparse matrices, only the triplets (row,col,value) are added
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
		for (const SparseTriplet& item : m.GetSparseTriplets())
		{
			if (item.value() != 0.)
			{
				triplets.AppendPure(SparseTriplet(item.row() + (StorageIndex)rowOffset, item.col() + (StorageIndex)columnOffset, item.value()));
			}
		}
	}
	else //faster mode, no offsets ...
	{
		for (const SparseTriplet& item : m.GetSparseTriplets())
		{
			if (item.value() != 0.)
			{
				triplets.AppendPure(item); //in this case, just add the items of submatrix
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
				triplets.AppendPure(SparseTriplet((StorageIndex)i, (StorageIndex)column, value));
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
				triplets.AppendPure(SparseTriplet((StorageIndex)(i+rowOffset), (StorageIndex)column, value));
			}
		}
	}
}

//! add factor*(vectorAdd-vectorSub) at 'column'; used for numerical jacobians
void GeneralMatrixEigenSparse::AddColumnVectorDiff(Index column, const Vector& vectorAdd, const Vector& vectorSub, Real factor, Index rowOffset)
{
    Index vectorLength = vectorAdd.NumberOfItems();
    CHECKandTHROW(!IsMatrixBuiltFromTriplets(), "GeneralMatrixEigenSparse::AddColumnVectorDiff(...): matrix must be in triplet mode !");
    CHECKandTHROW(vectorLength == vectorSub.NumberOfItems(), "GeneralMatrixEigenSparse::AddColumnVectorDiff: vectors must have equal length");

    if (!rowOffset)
    {
        for (Index i = 0; i < vectorLength; i++) //i = row
        {
            Real value = factor * (vectorAdd[i] - vectorSub[i]);
            if (value != 0.)
            {
                triplets.AppendPure(SparseTriplet((StorageIndex)i, (StorageIndex)column, value));
            }
        }
    }
    else
    {
        for (Index i = 0; i < vectorLength; i++) //i = row
        {
            Real value = factor * (vectorAdd[i] - vectorSub[i]);
            if (value != 0.)
            {
                triplets.AppendPure(SparseTriplet((StorageIndex)(i + rowOffset), (StorageIndex)column, value));
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
	
	////+++++++++++++++++++++++++++++++++++++++
	////EXUmath::SparseTripletMatrix stm;
	//ResizableArray<EXUmath::Triplet> sparseTriplets;
	////pout << "here\n";
	//for (auto t : triplets)
	//{
	//	sparseTriplets.Append(EXUmath::Triplet(t.row(), t.col(), t.value()));
	//}
	//matrix.setFromTriplets(sparseTriplets.begin(), sparseTriplets.end()); //sums up duplicates by default... (@TODO: what happens in EigenSparseMatrix::setFromTriplets(...) with (+1) + (-1) ? )
	////+++++++++++++++++++++++++++++++++++++++

	matrix.setFromTriplets(triplets.begin(), triplets.end()); //sums up duplicates by default... (@TODO: what happens in EigenSparseMatrix::setFromTriplets(...) with (+1) + (-1) ? )

	SetMatrixBuiltFromTriplets(); //now the sparse matrix is finally set and ready for multiplication and factorization
}

//Index TSeigenFactorize;
//TimerStructureRegistrator TSReigenFactorize("eigenFactorize", TSeigenFactorize, globalTimers);
//Index TSeigenAnalyzePattern;
//TimerStructureRegistrator TSReigenAnalyzePattern("eigenAnalyzePattern", TSeigenAnalyzePattern, globalTimers);

#define LinearSolverSuspendWorkers false //maybe this gives better results for large systems, but needs to be adjusted
#define LinearSolverSuspendWorkersTimeUS 20 //could not find good value; large values raise effort of other functions (mostly subsequent computation of mass matrix)

//! factorize matrix (invert, SparseLU, etc.); -1=success
Index GeneralMatrixEigenSparse::FactorizeNew() //bool ignoreRedundantEquation, Index redundantEquationsStart)
{
	CHECKandTHROW(IsMatrixBuiltFromTriplets(), "GeneralMatrixEigenSparse::Factorize(): matrix must be built before factorization!");
	Index nThreads = ExuThreading::TaskManager::GetNumThreads();
	if (LinearSolverSuspendWorkers && nThreads > 1) { ExuThreading::TaskManager::SuspendWorkers(LinearSolverSuspendWorkersTimeUS); }

	Index rv = 0;
	if (!IsSymmetric())
	{
		bool reuseFailed = false;
		if (GetReuseAnalyzedPattern())
		{
			if (analyzedPatternLastNNZ != matrix.nonZeros())
			{
				solver.analyzePattern(matrix);
			}

			// Compute the numerical factorization 
			solver.factorize(matrix);
			rv = solver.info();
		}
		//if factorize failed with reuced pattern, repeat step!
		if (!GetReuseAnalyzedPattern() || (analyzedPatternLastNNZ == matrix.nonZeros() && rv > 0))
		{
			reuseFailed = true;
			solver.analyzePattern(matrix);
			solver.factorize(matrix);
		}

		if (LinearSolverSuspendWorkers && nThreads > 1) { ExuThreading::TaskManager::ResumeWorkers(); }

		//0: successful factorization
		//if info = i, and i is
		//<= A->ncol : U(i, i) is exactly zero.The factorization has been completed, but the factor U is exactly singular, and division by zero will occur if it is used to solve a system of equations.
		//> A->ncol: number of bytes allocated when memory allocation failure occurred, plus A->ncol.If lwork = -1, it is the estimated amount of space needed, plus A->ncol.
		rv = solver.info();
		if (!rv)  //success
		{ 
			if (reuseFailed) { analyzedPatternLastNNZ = 0; } //for next iteration, do not repeat once if failed
			else { analyzedPatternLastNNZ = (Index)matrix.nonZeros(); }
			SetMatrixIsFactorized(true); 
			return -1; 
		}
		analyzedPatternLastNNZ = 0;
		if (rv <= NumberOfRows()) { return rv - 1; } //causing row
		else { return NumberOfRows(); } //undefined error

	}
	else
	{
		//STARTGLOBALTIMER(TSeigenAnalyzePattern);
		solverSymmetric.analyzePattern(matrix);
		//STOPGLOBALTIMER(TSeigenAnalyzePattern);

		// Compute the numerical factorization 
		//STARTGLOBALTIMER(TSeigenFactorize);
		solverSymmetric.factorize(matrix);
		//STOPGLOBALTIMER(TSeigenFactorize);

		//0: successful factorization; 1: numerical issue
		rv = solverSymmetric.info();
		if (rv == 0) { SetMatrixIsFactorized(true); return -1; }
		else { return NumberOfRows(); } //undefined error; symmetric solver does not determine causing row!
	}
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

//Index TStimer1;
//TimerStructureRegistrator TSRtimer1("timer1", TStimer1, globalTimers);

//! after factorization of matrix (=A), solve provides a solution vector (=x) for A*x = rhs ==> soluation = A^{-1}*rhs
void GeneralMatrixEigenSparse::Solve(const Vector& rhs, Vector& solution)
{
	CHECKandTHROW(IsMatrixIsFactorized(), "GeneralMatrixEigenSparse::Solve( ...): matrix is not factorized!");

	//Index nThreads = ExuThreading::TaskManager::GetNumThreads();
	//if (LinearSolverSuspendWorkers && nThreads > 1) { ExuThreading::task_manager->SuspendWorkers(1); }

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
	if (!IsSymmetric())
	{
		x = solver.solve(b);
	}
	else
	{
		x = solverSymmetric.solve(b);
	}


	//STARTGLOBALTIMER(TStimer1);
	for (Index i = 0; i < n; i++)
	{
		solution[i] = x[i];
	}

	//if (LinearSolverSuspendWorkers && nThreads > 1) { ExuThreading::task_manager->ResumeWorkers(); }

	//STOPGLOBALTIMER(TStimer1);
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
