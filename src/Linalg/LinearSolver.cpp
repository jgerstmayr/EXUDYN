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
extern PySpecial pySpecial;           //!special.solver: the sleeping workers (#3428) and MKL PARDISO (#3431)
#include "Linalg/PardisoMKL.h"

//! factorize matrix (invert, SparseLU, etc.); -1=success; >=0: causing row
Index GeneralMatrixEXUdense::FactorizeNew() //bool ignoreRedundantEquation, Index redundantEquationsStart)
{
    Index  rv;

    if (useEigenSolverType == 0)
    {
        rv = matrix.InvertSpecial(inverseTemporary, rowSwapsTemporary, false, 0, PivotThreshold()); //members, not static (#1614)
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
	if (pardisoSolver) { pardisoSolver->Release(); }
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


//! append the triplets of factor*submatrix (transposed: factor*submatrix^T) to triplets, from the data pointers;
//! row i of the result goes to rowIndices[i]+rowOffset (rowIndices == nullptr: i+rowOffset), columns likewise;
//! zeros are skipped; the order of the triplets is the one of the loops before (#3419)
template<bool transposed>
inline void AppendSubmatrixTriplets(SparseTripletVector& triplets, const Matrix& submatrix, Real factor,
	const Index* rowIndices, const Index* columnIndices, Index rowOffset, Index columnOffset)
{
	const Index rows = submatrix.NumberOfRows();
	const Index columns = submatrix.NumberOfColumns();
	const Real* values = submatrix.GetDataPointer();
	triplets.EnlargeMaxNumberOfItemsTo(triplets.NumberOfItems() + rows * columns);

	for (Index i = 0; i < rows; i++)
	{
		const Real* row = &values[i*columns];
		for (Index j = 0; j < columns; j++)
		{
			const Real value = row[j];
			if (value != 0.)
			{
				//the row of the submatrix is the column of the result for transposed
				const Index resultRow = transposed ? j : i;
				const Index resultColumn = transposed ? i : j;
				const Index globalRow = (rowIndices ? rowIndices[resultRow] : resultRow) + rowOffset;
				const Index globalColumn = (columnIndices ? columnIndices[resultColumn] : resultColumn) + columnOffset;
				triplets.AppendPure(SparseTriplet((StorageIndex)globalRow, (StorageIndex)globalColumn, factor*value));
			}
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
	CHECKandTHROW(LTGrows.NumberOfItems() >= submatrix.NumberOfRows() && LTGcolumns.NumberOfItems() >= submatrix.NumberOfColumns(),
		"GeneralMatrixEigenSparse::AddSubmatrix(const Matrix&, const ArrayIndex& LTGrows, const ArrayIndex& LTGcolumns, ...): LTGrows or LTGcolumns too short");

	AppendSubmatrixTriplets<false>(triplets, submatrix, factor, LTGrows.GetDataPointer(), LTGcolumns.GetDataPointer(), rowOffset, columnOffset);
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
	CHECKandTHROW(!IsMatrixBuiltFromTriplets(), "GeneralMatrixEigenSparse::AddSubmatrixTransposed(const Matrix&, const ArrayIndex& LTGrows, const ArrayIndex& LTGcolumns, ...): only possible in triplet mode!");
	CHECKandTHROW(LTGrows.NumberOfItems() >= submatrix.NumberOfColumns() && LTGcolumns.NumberOfItems() >= submatrix.NumberOfRows(),
		"GeneralMatrixEigenSparse::AddSubmatrixTransposed(const Matrix&, const ArrayIndex& LTGrows, const ArrayIndex& LTGcolumns, ...): LTGrows or LTGcolumns too short");

	AppendSubmatrixTriplets<true>(triplets, submatrix, factor, LTGrows.GetDataPointer(), LTGcolumns.GetDataPointer(), rowOffset, columnOffset);
}

//! add (possibly) smaller factor*Matrix to this matrix; 
//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
void GeneralMatrixEigenSparse::AddSubmatrixWithFactor(const Matrix& submatrix, Real factor, Index rowOffset, Index columnOffset)
{
	//only allowed in triplet mode:
	CHECKandTHROW(!IsMatrixBuiltFromTriplets(), "GeneralMatrixEigenSparse::AddSubmatrixWithFactor(...): only possible in triplet mode!");
	AppendSubmatrixTriplets<false>(triplets, submatrix, factor, nullptr, nullptr, rowOffset, columnOffset);
}

//! add (possibly) smaller factor*Transposed(Matrix) to this matrix; 
//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
void GeneralMatrixEigenSparse::AddSubmatrixTransposedWithFactor(const Matrix& submatrix, Real factor, Index rowOffset, Index columnOffset)
{
	//only allowed in triplet mode:
	CHECKandTHROW(!IsMatrixBuiltFromTriplets(), "GeneralMatrixEigenSparse::AddSubmatrixTransposedWithFactor(...): only possible in triplet mode!");
	AppendSubmatrixTriplets<true>(triplets, submatrix, factor, nullptr, nullptr, rowOffset, columnOffset);
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

	if (dropThreshold > 0.) { DropSmallEntries(); }

	SetMatrixBuiltFromTriplets(); //now the sparse matrix is finally set and ready for multiplication and factorization
}

//! leave out the entries whose magnitude is at most dropThreshold times the smaller of the largest magnitudes in their
//! row and in their column - round-off of rotations and numerical derivatives, which costs fill-in in the
//! factorization (#3448)
void GeneralMatrixEigenSparse::DropSmallEntries()
{
	rowMaximum.SetNumberOfItems(NumberOfRows());
	columnMaximum.SetNumberOfItems(NumberOfColumns());
	rowMaximum.SetAll(0.);
	columnMaximum.SetAll(0.);
	for (Index column = 0; column < matrix.outerSize(); column++)
	{
		for (EigenSparseMatrix::InnerIterator it(matrix, column); it; ++it)
		{
			const Real value = fabs(it.value());
			if (value > rowMaximum[(Index)it.row()]) { rowMaximum[(Index)it.row()] = value; }
			if (value > columnMaximum[column]) { columnMaximum[column] = value; }
		}
	}
	matrix.prune([this](Index row, Index column, Real value)
	{
		return fabs(value) > dropThreshold * EXUstd::Minimum(rowMaximum[row], columnMaximum[column]);
	});
}

//Index TSeigenFactorize;
//TimerStructureRegistrator TSReigenFactorize("eigenFactorize", TSeigenFactorize, globalTimers);
//Index TSeigenAnalyzePattern;
//TimerStructureRegistrator TSReigenAnalyzePattern("eigenAnalyzePattern", TSeigenAnalyzePattern, globalTimers);

//! the workers of the TaskManager sleep while a factorization of at least special.solver.suspendWorkersMinimumSize
//! unknowns runs, if special.solver.suspendWorkersDuringFactorization; they are woken when it ends (#3428)
class SuspendWorkersDuringFactorization
{
	bool suspended;
public:
	SuspendWorkersDuringFactorization(Index size)
	{
		suspended = pySpecial.solver.suspendWorkersDuringFactorization && ExuThreading::task_manager &&
			ExuThreading::TaskManager::GetNumThreads() > 1 && size >= pySpecial.solver.suspendWorkersMinimumSize;
		if (suspended) { ExuThreading::TaskManager::SuspendWorkers(); }
	}
	~SuspendWorkersDuringFactorization()
	{
		if (suspended) { ExuThreading::TaskManager::ResumeWorkers(); }
	}
};

void GeneralMatrixEigenSparse::PardisoFallBack(const std::string& reason)
{
	if (!pySpecial.solver.pardisoFallback) //PyError, not CHECKandTHROW, which the fast module does not have
	{
		PyError("LinearSolverType.PardisoMKL: " + reason + "; no fallback to SparseLU, as exudyn.special.solver.pardisoFallback is False");
	}
	pySpecial.solver.pardisoFallbacks++;
	PyWarning("LinearSolverType.PardisoMKL: " + reason + "; this simulation continues with SparseLU (as EigenSparse)");
	pardisoFallenBack = true;
	pardisoCheckNextSolve = false;
	analyzedPatternLastNNZ = 0;
}

bool GeneralMatrixEigenSparse::FactorizePardiso()
{
	if (!pardisoSolver) { pardisoSolver = std::make_shared<PardisoSolver>(); }
	matrix.makeCompressed(); //PARDISO takes the compressed columns of Eigen as they are
	Index numberOfThreads = pySpecial.solver.pardisoNumberOfThreads > 0 ? pySpecial.solver.pardisoNumberOfThreads :
		EXUstd::Maximum(ExuThreading::TaskManager::GetNumThreads(), (Index)1);
	int error = pardisoSolver->Factorize((int)NumberOfRows(), matrix.outerIndexPtr(), matrix.innerIndexPtr(), matrix.valuePtr(),
		(int)numberOfThreads, pySpecial.solver.pardisoParameters);
	if (pardisoSolver->PatternChanged()) { pardisoFactorizations = 0; }
	if (error != 0)
	{
		PardisoFallBack("the factorization failed with the error " + EXUstd::ToString(error) + " of PARDISO");
		return false;
	}
	//the residual is checked after the first factorization of an analysis and after every checkPeriod-th one
	Index period = pySpecial.solver.pardisoCheckPeriod;
	pardisoCheckNextSolve = period > 0 && (pardisoFactorizations % period) == 0;
	pardisoFactorizations++;
	return true;
}

void GeneralMatrixEigenSparse::SetStructuralGroups(const std::vector<std::vector<int>>& groups)
{
	if (!solverParallel) { solverParallel = std::make_shared<SparseLUParallel>(); }
	solverParallel->SetStructuralGroups(groups);
}

//! factorize matrix (invert, SparseLU, etc.); -1=success
Index GeneralMatrixEigenSparse::FactorizeNew() //bool ignoreRedundantEquation, Index redundantEquationsStart)
{
	CHECKandTHROW(IsMatrixBuiltFromTriplets(), "GeneralMatrixEigenSparse::Factorize(): matrix must be built before factorization!");
	SuspendWorkersDuringFactorization suspendWorkers(NumberOfRows()); //resumes the workers on every return (#3428)

	if (PardisoActive() && FactorizePardiso())
	{
		SetMatrixIsFactorized(true);
		return -1;
	}

	parallelSubstitutionActive = false;
	if (!IsSymmetric() && pySpecial.solver.parallelSubstitution) //experimental (#3432)
	{
		if (!solverParallel) { solverParallel = std::make_shared<SparseLUParallel>(); }
		matrix.makeCompressed();
		int numberOfThreads = (int)EXUstd::Maximum(ExuThreading::TaskManager::GetNumThreads(), (Index)1);
		if (solverParallel->AnalyzeAndFactorize(matrix, numberOfThreads, pySpecial.solver.parallelSubstitutionPivotThreshold) == 0)
		{
			if (!solverParallel->Prepared()) { pySpecial.solver.parallelSubstitutionSerial++; }
			parallelSubstitutionActive = true;
			analyzedPatternLastNNZ = 0;
			SetMatrixIsFactorized(true);
			return -1;
		}
		pySpecial.solver.parallelSubstitutionSerial++; //failed with the small pivot threshold: SparseLU as usual
	}

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

		//Eigen::ComputationInfo: 0=Success, 1=NumericalIssue, 2=NoConvergence, 3=InvalidInput.
		//NOTE: this is a STATUS, not a row. The comment that stood here described SuperLU's 'info'
		//(where a value <= ncol was the column with the zero pivot) and the code returned info-1 as
		//if it were a causing row - which made the solver report "causing system equation number
		//= 0" for every singular sparse system, whichever row was actually singular (#2482).
		//Eigen's SparseLU does not tell which row it was, so say 'unknown' instead of inventing one
		rv = solver.info();
		if (!rv)  //success
		{
			if (pySpecial.solver.sparseSolveByRows) { solver.ExtractRows(); } //the substitution by rows (#3441)
			if (reuseFailed) { analyzedPatternLastNNZ = 0; } //for next iteration, do not repeat once if failed
			else { analyzedPatternLastNNZ = (Index)matrix.nonZeros(); }
			SetMatrixIsFactorized(true);
			return -1;
		}
		analyzedPatternLastNNZ = 0;
		return NumberOfRows(); //failed, causing row unknown: the caller prints no row for this value

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

	if (PardisoActive())
	{
		Index n = NumberOfRows();
		solution.SetNumberOfItems(n);
		int error = pardisoSolver->Solve(rhs.GetDataPointer(), solution.GetDataPointer());
		std::string reason;
		if (error != 0) { reason = "the solve failed with the error " + EXUstd::ToString(error) + " of PARDISO"; }
		else if (pardisoCheckNextSolve)
		{
			pardisoCheckNextSolve = false;
			Eigen::Map<const Eigen::VectorXd> b(pardisoSolver->LastRhs(), n); //rhs may be overwritten, if it is the solution
			Eigen::Map<const Eigen::VectorXd> x(solution.GetDataPointer(), n);
			Real residual = (matrix * x - b).norm();
			Real bNorm = b.norm();
			if (!(residual <= pySpecial.solver.pardisoResidualTolerance * bNorm)) //also catches NaN
			{
				reason = "the relative residual of the solve, " + EXUstd::ToString(bNorm != 0. ? residual / bNorm : residual) +
					", is above exudyn.special.solver.pardisoResidualTolerance (perturbed pivots: " +
					EXUstd::ToString(pardisoSolver->PerturbedPivots()) + ")";
			}
		}
		if (reason.empty()) { return; }

		PardisoFallBack(reason);
		solver.analyzePattern(matrix);
		solver.factorize(matrix);
		if (solver.info() != 0) { SysError("LinearSolverType.PardisoMKL: the factorization by SparseLU after the fallback failed as well", PyErrorType::solverError); }
		//the right-hand side kept by PARDISO, as rhs may be the solution vector, which PARDISO has overwritten
		Eigen::VectorXd x = solver.solve(Eigen::Map<const Eigen::VectorXd>(pardisoSolver->LastRhs(), n));
		for (Index i = 0; i < n; i++) { solution[i] = x[i]; }
		return;
	}

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
	if (parallelSubstitutionActive) //(#3432)
	{
		if (solverParallel->Prepared())
		{
			solverParallel->SolveParallel(rhs.GetDataPointer(), solution.GetDataPointer(), (int)n);
			return;
		}
		if (solverParallel->RowsReady()) //a factor couples subtrees: the serial substitution by rows
		{
			solverParallel->SolveRows(rhs.GetDataPointer(), solution.GetDataPointer(), (int)n);
			return;
		}
		x = solverParallel->solve(b);
	}
	else if (!IsSymmetric())
	{
		if (solver.RowsReady()) //(#3441)
		{
			solver.SolveRows(rhs.GetDataPointer(), solution.GetDataPointer(), (int)n);
			return;
		}
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

	//STOPGLOBALTIMER(TStimer1);
}

//! return a dense matrix from any other matrix: requires a copy - SLOW!
void GeneralMatrixEigenSparse::GetSparseEntries(std::vector<Index>& rows, std::vector<Index>& columns,
	std::vector<Real>& values) const
{
	rows.clear(); columns.clear(); values.clear();
	if (IsMatrixBuiltFromTriplets()) //the matrix, as MultiplyWithFactor changes it and not the triplets
	{
		for (Index k = 0; k < (Index)matrix.outerSize(); k++)
		{
			for (EigenSparseMatrix::InnerIterator it(matrix, k); it; ++it)
			{
				rows.push_back((Index)it.row()); columns.push_back((Index)it.col()); values.push_back(it.value());
			}
		}
	}
	else
	{
		for (const auto& item : triplets)
		{
			rows.push_back((Index)item.row()); columns.push_back((Index)item.col()); values.push_back(item.value());
		}
	}
}

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
