/** ***********************************************************************************************
* @brief		Include files and data strcutures linalg solvers (link to Eigen);
*				this file includes defines for activation of eigen sparse solvers (increases drastically number of include files and dependencies, compile time and size of executable!)
*
* @author		Gerstmayr Johannes
* @date			2019-11-09 (generated)
* @date			2019-11-09 (last modified)
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#ifndef LINEARSOLVER__H
#define LINEARSOLVER__H

//BasicLinalg provides consistent includes for BasicDefinitions, arrays, vectors and matrices
#include "Linalg/BasicLinalg.h"	
#include "Main/OutputVariable.h"
#include "Linalg/MatrixContainer.h"	
#include <vector> //for eigen triplets
#include <iostream>

#define USE_EIGEN

#ifdef EXUDYN_RELEASE
//for tests with symmetric solver:
#define EIGEN_MPL2_ONLY //add this line at the place of 'USE_EIGEN'; it guaranties to raise a compilation error, if non-MPL2 licensed parts of Eigen are used (see also LICENSE.txt file and COPYING.README of Eigen)
#else
#define USE_SYMMETRIC_SOLVER
#endif


#ifdef USE_EIGEN
    #define USE_EIGEN_SPARSE_SOLVER
    #define USE_EIGEN_DENSE_SOLVER
    #include "../Eigen/Sparse"
	#include "../Eigen/Dense"
	//#include "Eigen/SuperLUSupport"
	#include "../Eigen/SparseLU"
#ifdef USE_SYMMETRIC_SOLVER
	#include "../Eigen/SparseCholesky"
#endif
	//#include <Eigen/Core>
//#ifdef EIGEN_HAS_OPENMP
//    #include <omp.h> //for eigen omp support
//#endif // EIGEN_HAS_OPENMP

//#define useMatrixContainerTriplets
//#ifdef useMatrixContainerTriplets
//	typedef EXUmath::Triplet SparseTriplet;						//! this is a simple (row,col,value) structure for sparse matrix non zero entries
//	typedef ResizableArray<SparseTriplet> SparseTripletVector;	//! this vector stores (dynamically!) the triplets
//#else
//	typedef Eigen::Triplet<Real> SparseTriplet;				//! this is a simple (row,col,value) structure for sparse matrix non zero entries
//	typedef std::vector<SparseTriplet> SparseTripletVector;		//! this vector stores (dynamically!) the triplets
//#endif
	typedef Eigen::SparseMatrix<Real> EigenSparseMatrix;		//! declares a column-major sparse matrix type of double
	typedef Eigen::SparseMatrix<Real>::StorageIndex StorageIndex;	//! conversion to Index necessary
#endif


//without Eigen: 47.4 (49.3) seconds compile time on Surface Pro 5: 2019-11-09 (2019-11-16)
//with    Eigen(no usage yet): 63.4 (53.6 without Eigen) seconds compile time on Surface Pro 5: 2019-11-16
//with    Eigen(no usage yet): 19.3 (15.4 without Eigen) seconds compile time on I9 / 14 core: 2019-11-18


typedef std::complex<Real> RealC;
//eigenvalue solver based on Eigen
namespace EXUmath {
	//! compute [size] eigenvalues of polynomial with [size+1] coefficients
	template<Index size>
	void PolynomialRoots(const ConstSizeVector<size+1>& coeffs,
		ConstSizeVectorBase<std::complex<Real>, size>& complexRoots)
	{
		typedef Eigen::Matrix<Eigen::dcomplex, size, size> EigMatrixC;
		EigMatrixC A;
		A.setZero();
		CHECKandTHROW(coeffs[size] != 0., "PolynomialRoots: highest coefficient may not be zero!");

		//A = [[0, 0, 0, 0, 0, -c0 / c6], 
		//	[1, 0, 0, 0, 0, -c1 / c6], 
		//	[0, 1, 0, 0, 0, -c2 / c6], 
		//	[0, 0, 1, 0, 0, -c3 / c6], 
		//	[0, 0, 0, 1, 0, -c4 / c6], 
		//	[0, 0, 0, 0, 1, -c5 / c6]]

		Real invCoeffMax = 1. / coeffs[size];
		for (Index i = 0; i < size; i++)
		{
			A(i, size - 1) = RealC(-coeffs[i] * invCoeffMax, 0);
		}
		for (Index i = 1; i < size; i++)
		{
			A(i, i - 1) = RealC(1., 0.);
		}

		Eigen::ComplexEigenSolver<EigMatrixC> ces;
		ces.compute(A, false); //17microsecs for 6x6 matrix!
		//pout << "The eigenvalues of A are:" << endl << ces.eigenvalues() << endl;

		for (Index i = 0; i < size; i++)
		{
			complexRoots[i] = ces.eigenvalues()[i];
		}
	}
};


//! container for storage of different system matrix formats; this shall grant storage-independent access and manipulation; do not access individual entries of matrix directly (may be slow)
class GeneralMatrix
{
private:
	bool matrixIsFactorized; //!< flag, which is only true, if the matrix has been previously factorized without any changes
    Real pivotThreshold;

public:
	virtual ~GeneralMatrix() {} //added for correct deletion of derived classes
	//! information on storage type
	virtual LinearSolverType GetSystemMatrixType() const = 0;
	virtual bool IsMatrixIsFactorized() const { return matrixIsFactorized; };
	virtual void SetMatrixIsFactorized(bool flag) { matrixIsFactorized = flag; };

    virtual const Real& PivotThreshold() const { return pivotThreshold; };
    virtual Real& PivotThreshold() { return pivotThreshold; };

	//! functions only used for Sparse mode; no problem, if called for dense matrix
	virtual void AssumeSymmetric(bool flag = true) {};
	virtual bool IsSymmetric() const { return false; }
	virtual void SetReuseAnalyzedPattern(bool flag) { }
	virtual bool GetReuseAnalyzedPattern() const { return false; }

	//don't do the following; use casting!
	////! get (read) matrix as dense exudyn Matrix; this function should be used rarly, as it disables the compatibility to other matrix formats
	//virtual const Matrix& GetMatrixEXUdense() const { CHECKandTHROWstring("GeneralMatrix::GetMatrixEXUdense const: invalid call"); return EXUmath::unitMatrix3D; }
	////! get (write) matrix as dense exudyn Matrix; also in this case, solvability may be lost; 
	////! this function should be used rarly, as it disables the compatibility to other matrix formats
	////! however, we never know what else is done with the matrix afterwards ...
	//virtual Matrix& GetMatrixEXUdense() { CHECKandTHROWstring("GeneralMatrix::GetMatrixEXUdense const: invalid call"); return EXUmath::unitMatrix3D; }

	//helper functions for matrix:
	virtual void SetNumberOfRowsAndColumns(Index numberOfRowsInit, Index numberOfColumnsInit) = 0;
	
	//! get number of columns
	virtual Index NumberOfRows() const = 0;
	//! get number of rows
	virtual Index NumberOfColumns() const = 0;

	//! set all matrix items to zero (in dense matrix, all entries are set 0, in sparse matrix, the vector of items is erased)
	virtual void SetAllZero() = 0;
	//! multiply matrix entries with factor
	virtual void MultiplyWithFactor(Real factor) = 0;

	//! set the matrix with a dense matrix
	virtual void SetMatrix(const Matrix& otherMatrix) = 0;

	//! add a diagonal (or unit) matrix located at a certain (rowOffset, columnOffset) position
	virtual void AddDiagonalMatrix(Real diagValue, Index numberOfRowsColumns, Index rowOffset = 0, Index columnOffset = 0) = 0;

	//! add (possibly) smaller factor*Matrix to this matrix, transforming the row indices of the submatrix with LTGrows and the column indices with LTGcolumns; 
	//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
	//! the offsets are with respect to the indices calculated from the LTGrows/columns transformation
	virtual void AddSubmatrix(const Matrix& submatrix, Real factor, const ArrayIndex& LTGrows, const ArrayIndex& LTGcolumns, Index rowOffset = 0, Index columnOffset = 0) = 0;

	//! add sparse triplets to dense or sparse matrix
	virtual void AddSparseTriplets(const SparseTripletVector& otherTriplets) = 0;

	//! add (possibly) smaller factor*Transposed(Matrix) to this matrix, transforming the row indices of the submatrix with LTGrows and the column indices with LTGcolumns; 
	//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
	//! the offsets are with respect to the indices calculated from the LTGrows/columns transformation
	virtual void AddSubmatrixTransposed(const Matrix& submatrix, Real factor, const ArrayIndex& LTGrows, const ArrayIndex& LTGcolumns, Index rowOffset = 0, Index columnOffset = 0) = 0;

	//! add possibly smaller GeneralMatrix (with same type as *this !) to *this matrix; in case of sparse matrices, only the triplets of GeneralMatrixEigenSparse are added
	virtual void AddSubmatrix(const GeneralMatrix& submatrix, Index rowOffset = 0, Index columnOffset = 0) = 0;

	//! add (possibly) smaller factor*Matrix to this matrix; 
	//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
	virtual void AddSubmatrixWithFactor(const Matrix& submatrix, Real factor, Index rowOffset = 0, Index columnOffset = 0) = 0;

	//! add (possibly) smaller factor*Transposed(Matrix) to this matrix; 
	//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
	virtual void AddSubmatrixTransposedWithFactor(const Matrix& submatrix, Real factor, Index rowOffset = 0, Index columnOffset = 0) = 0;

	//! add column 'vector' at 'column'; used to add a couple of entries during jacobian computation; filters zeros in sparse mode
	virtual void AddColumnVector(Index column, const Vector& vector, Index rowOffset = 0) = 0;

    //! add factor*(vectorAdd-vectorSub) at 'column'; used for numerical jacobians
    virtual void AddColumnVectorDiff(Index column, const Vector& vectorAdd, const Vector& vectorSub, Real factor, Index rowOffset = 0) = 0;

	//! After filling the matrix, it is finalized for further operations (matrix*vector, factorization, ...)
	//! In the dense matrix case, nothing needs to be done; in the sparse matrix case, the elements of the matrix are filled into the sparse matrix
	virtual void FinalizeMatrix() = 0;

	//! factorize matrix (invert, SparseLU, etc.)
	//! bool ignoreRedundantEquations, Index redundantEquationsStart, Real pivotThreshold
	//! return -1 if success, causing index otherwise
    virtual Index FactorizeNew() = 0; // bool ignoreRedundantEquation = false, Index redundantEquationsStart = 0) = 0;

	//! after factorization of matrix (=A), solve provides a solution vector (=x) for A*x = rhs ==> soluation = A^{-1}*rhs
	virtual void Solve(const Vector& rhs, Vector& solution) = 0;

	//! multiply matrix with vector: solution = A*x
	virtual void MultMatrixVector(const Vector& x, Vector& solution) = 0;

	//! multiply matrix with vector and add to solution: solution += A*x
	virtual void MultMatrixVectorAdd(const Vector& x, Vector& solution) = 0;

	//! multiply transposed(matrix) with vector: solution = A^T*x
	virtual void MultMatrixTransposedVector(const Vector& x, Vector& solution) = 0;

	//! return a dense matrix from any other matrix: requires a copy - SLOW!
	virtual ResizableMatrix GetEXUdenseMatrix() const = 0;

	//! function overwritten in derived class to print matrix
	virtual void PrintMatrix(std::ostream& os) const = 0;

	//! ostream for general matrix; for output and for debug purposes
	friend std::ostream& operator<<(std::ostream& os, const GeneralMatrix& matrix) 
	{ 
		matrix.PrintMatrix(os);
		return os; 
	};

};


//! specialization of GeneralMatrix to exudyn dense matrix
class GeneralMatrixEXUdense : public GeneralMatrix
{
private:
    Index useEigenSolverType;                               //!< use Eigen solver
    ResizableMatrix matrix;                                 //!< internal dense matrix storage:
#ifdef USE_EIGEN_DENSE_SOLVER
    Eigen::FullPivLU<Eigen::MatrixXd> eigenLUfullPivot;     //!< Full Pivot LU factorization using Eigen
    Eigen::PartialPivLU<Eigen::MatrixXd> eigenLU;           //!< LU factorization using Eigen
    static Index constexpr eigenSolverTypeLU = 1;
    static Index constexpr eigenSolverTypeFullPivotLU = 2;
    bool eigenUsesFullPivot;                                //!< set by last call of LU factorization to remember type of solver
#endif
public:
    GeneralMatrixEXUdense() { SetMatrixIsFactorized(false); UseEigenSolverType() = 0; PivotThreshold() = 0.; }

	//! information on storage type
	virtual LinearSolverType GetSystemMatrixType() const { return (useEigenSolverType == 0) ? LinearSolverType::EXUdense : LinearSolverType::EigenDense; };

	//! get (read) matrix as dense exudyn Matrix; this function should be used rarly, as it disables the compatibility to other matrix formats
	const ResizableMatrix& GetMatrixEXUdense() const { return matrix; }
	//! get (write) matrix as dense exudyn Matrix; also in this case, solvability may be lost; 
	//! this function should be used rarly, as it disables the compatibility to other matrix formats
	//! however, we never know what else is done with the matrix afterwards ...
	ResizableMatrix& GetMatrixEXUdense() { SetMatrixIsFactorized(false); return matrix; }

    //! access function to special solver setting
    const Index& UseEigenSolverType() const { return useEigenSolverType; }
    Index& UseEigenSolverType() { return useEigenSolverType; }

	//helper functions for matrix:
	virtual void SetNumberOfRowsAndColumns(Index numberOfRowsInit, Index numberOfColumnsInit)
	{
		SetMatrixIsFactorized(false);
		matrix.SetNumberOfRowsAndColumns(numberOfRowsInit, numberOfColumnsInit);
	}

	//! get number of columns
	virtual Index NumberOfRows() const { return matrix.NumberOfRows(); }
	//! get number of rows
	virtual Index NumberOfColumns() const { return matrix.NumberOfColumns(); }

	//! set all matrix items to zero (in dense matrix, all entries are set 0, in sparse matrix, the vector of items is erased)
	virtual void SetAllZero()
	{
		SetMatrixIsFactorized(false);
		matrix.SetAll(0.);
	}
	//! multiply matrix entries with factor
	virtual void MultiplyWithFactor(Real factor)
	{
		SetMatrixIsFactorized(false);
		matrix *= factor;
	}
	//! set the matrix with a dense matrix
	virtual void SetMatrix(const Matrix& otherMatrix)
	{
		SetMatrixIsFactorized(false);
		matrix.CopyFrom(otherMatrix);
	}

	//! add a diagonal (or unit) matrix located at a certain (rowOffset, columnOffset) position
	virtual void AddDiagonalMatrix(Real diagValue, Index numberOfRowsColumns, Index rowOffset = 0, Index columnOffset = 0)
	{
		SetMatrixIsFactorized(false);
		for (Index i = 0; i < numberOfRowsColumns; i++)
		{
			matrix(rowOffset + i, columnOffset + i) += diagValue;
		}
	}

	//! add (possibly) smaller factor*Matrix to this matrix, transforming the row indices of the submatrix with LTGrows and the column indices with LTGcolumns; 
	//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
	//! the offsets are with respect to the indices calculated from the LTGrows/columns transformation
	virtual void AddSubmatrix(const Matrix& submatrix, Real factor, const ArrayIndex& LTGrows, const ArrayIndex& LTGcolumns, Index rowOffset = 0, Index columnOffset = 0)
	{
		SetMatrixIsFactorized(false);
		matrix.AddSubmatrix(submatrix, factor, LTGrows, LTGcolumns, rowOffset, columnOffset);
	}

	//! add sparse triplets to dense matrix
	virtual void AddSparseTriplets(const SparseTripletVector& otherTriplets)
	{
		SetMatrixIsFactorized(false);
		for (const SparseTriplet& triplet : otherTriplets)
		{
			matrix(triplet.row(), triplet.col()) += triplet.value();
		}
	}

	//! add (possibly) smaller factor*Transposed(Matrix) to this matrix, transforming the row indices of the submatrix with LTGrows and the column indices with LTGcolumns; 
	//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
	//! the offsets are with respect to the indices calculated from the LTGrows/columns transformation
	virtual void AddSubmatrixTransposed(const Matrix& submatrix, Real factor, const ArrayIndex& LTGrows, const ArrayIndex& LTGcolumns, Index rowOffset = 0, Index columnOffset = 0)
	{
		SetMatrixIsFactorized(false);
		matrix.AddSubmatrixTransposed(submatrix, factor, LTGrows, LTGcolumns, rowOffset, columnOffset);
	}

	//! add (possibly) smaller factor*Matrix to this matrix; 
	//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
	virtual void AddSubmatrixWithFactor(const Matrix& submatrix, Real factor, Index rowOffset = 0, Index columnOffset = 0)
	{
		SetMatrixIsFactorized(false);
		matrix.AddSubmatrixWithFactor(submatrix, factor, rowOffset, columnOffset);
	}

	//! add (possibly) smaller factor*Transposed(Matrix) to this matrix; 
	//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
	virtual void AddSubmatrixTransposedWithFactor(const Matrix& submatrix, Real factor, Index rowOffset = 0, Index columnOffset = 0)
	{
		SetMatrixIsFactorized(false);
		matrix.AddSubmatrixTransposedWithFactor(submatrix, factor, rowOffset, columnOffset);
	}


	//! add possibly smaller matrix to this matrix; in case of sparse matrices, only the triplets (row,col,value) are added
	virtual void AddSubmatrix(const GeneralMatrix& submatrix, Index rowOffset = 0, Index columnOffset = 0)
	{
		CHECKandTHROW((GetSystemMatrixType() == submatrix.GetSystemMatrixType()), "GeneralMatrixEXUdense::AddSubmatrix: invalid SystemMatrixType!");
		SetMatrixIsFactorized(false);
		const GeneralMatrixEXUdense& m = (const GeneralMatrixEXUdense&)submatrix;
		matrix.AddSubmatrix(m.GetMatrixEXUdense(), rowOffset, columnOffset);
	}


	//! add column 'vector' at 'column'; used to add a couple of entries during jacobian computation; filters zeros in sparse mode
	virtual void AddColumnVector(Index column, const Vector& vector, Index rowOffset = 0)
	{
        matrix.AddColumnVector(column, vector, rowOffset);
		//if (!rowOffset)
		//{
		//	for (Index i = 0; i < vec.NumberOfItems(); i++) //i = row
		//	{
		//		matrix(i, column) += vec[i];
		//	}
		//}
		//else
		//{
		//	for (Index i = 0; i < vec.NumberOfItems(); i++) //i = row
		//	{
		//		matrix(i + rowOffset, column) += vec[i];
		//	}
		//}
	}

    //! add factor*(vectorAdd-vectorSub) at 'column'; used for numerical jacobians
    virtual void AddColumnVectorDiff(Index column, const Vector& vectorAdd, const Vector& vectorSub, Real factor, Index rowOffset = 0)
    {
            matrix.AddColumnVectorDiff(column, vectorAdd, vectorSub, factor, rowOffset);
    }

	//! After filling the matrix, it is finalized for further operations (matrix*vector, factorization, ...)
	virtual void FinalizeMatrix() 
	{
		SetMatrixIsFactorized(false);
	}

	//! factorize matrix (invert, SparseLU, etc.);
	//! return -1 if success, causing index otherwise
    virtual Index FactorizeNew(); //removed 2023-06-11: switched to Eigen:FullPivLU; bool ignoreRedundantEquation = false, Index redundantEquationsStart = 0);

	//! multiply matrix with vector: solution = A*x
	virtual void MultMatrixVector(const Vector& x, Vector& solution)
	{
		if (IsMatrixIsFactorized()) { SysError("GeneralMatrixEXUdense::MultMatrixVector(...): matrix is already factorized ==> use Solve(...)!"); }
		EXUmath::MultMatrixVector(matrix, x, solution);
	}

	//! multiply matrix with vector and add to solution: solution += A*x
	virtual void MultMatrixVectorAdd(const Vector& x, Vector& solution)
	{
		if (IsMatrixIsFactorized()) { SysError("GeneralMatrixEXUdense::MultMatrixVectorAdd(...): matrix is already factorized ==> use Solve(...)!"); }
		EXUmath::MultMatrixVectorAdd(matrix, x, solution);
	}

	//! multiply transposed(matrix) with vector: solution = A^T*x
	virtual void MultMatrixTransposedVector(const Vector& x, Vector& solution)
	{
		if (IsMatrixIsFactorized()) { SysError("GeneralMatrixEXUdense::MultMatrixTransposedVector(...): matrix is already factorized ==> use Solve(...)!"); }
		EXUmath::MultMatrixTransposedVector(matrix, x, solution);
	}

	//!after factorization of matrix (=A), solve provides a solution vector (=x) for A*x = rhs ==> soluation = A^{-1}*rhs
    virtual void Solve(const Vector& rhs, Vector& solution);

	//! return a dense matrix from any other matrix: requires a copy - SLOW!
	virtual ResizableMatrix GetEXUdenseMatrix() const
	{
		return matrix;
	}

	//! function to print matrix
	virtual void PrintMatrix(std::ostream& os) const
	{
		os << matrix;
	}
};

//! specialization of GeneralMatrix to Eigen sparse matrix
//! there are three stages: 
//! 1) The matrix is defined by Eigen Triplets; the triplets might contain duplicates of entries, e.g. item (2,3) might be filled twice
//! 2) The EigenSparseMatrix is built from the triplets; it can be used e.g. to perform matrix-vector multiplication (MultMatrixVector) (matrixBuiltFromTriplets = true)
//! 3) The EigenSparseMatrix is solved (Factorize(...) ) with the 'solver' and can be used to solve with a rhs (matrixIsFactorized = true)
#ifdef USE_EIGEN_SPARSE_SOLVER
class GeneralMatrixEigenSparse : public GeneralMatrix
{
private:
	bool isSymmetric;       //!< mode to assume symmetric matrices with faster solver
	bool matrixBuiltFromTriplets;	//!< flag is set true as soon as the matrix is built from triplets; false, as soon as the triplets are modified
	Index numberOfRows;				//!< as the triplet structure does not provide this information, it must be stored separately
	Index numberOfColumns;			//!< as the triplet structure does not provide this information, it must be stored separately

	Index analyzedPatternLastNNZ;   //!< number of non-zeros of last analyzedPattern computation
	bool reuseAnalyzedPattern;      //!< True: the Eigen SparseLU solver offers the possibility to reuse an analyzed pattern of a previous factorization; this may reduce total factorization time by a factor of 2 or 3, depending on the matrix type; 
	//data for Eigen sparse matrix storage:
#ifdef USE_EIGEN_SPARSE_SOLVER
	EigenSparseMatrix matrix;	 //this is the sparse matrix built from triplets
	SparseTripletVector triplets; //this contains a redundant set of matrix entries
	Eigen::SparseLU<Eigen::SparseMatrix<Real>, Eigen::COLAMDOrdering<int> >   solver; //this is the solver for the matrix
#ifdef USE_SYMMETRIC_SOLVER
	Eigen::SimplicialLDLT<Eigen::SparseMatrix<Real>, 1>   solverSymmetric; //this is the symmetric solver for the matrix
	//Eigen::SimplicialLLT<Eigen::SparseMatrix<Real>, 0>   solverSymmetric; //this is the symmetric solver for the matrix
#else
	Eigen::SparseLU<Eigen::SparseMatrix<Real>, Eigen::COLAMDOrdering<int> >   solverSymmetric; //replacement, for consistent functionality
#endif

#endif

public:
	GeneralMatrixEigenSparse() 
	{ 
        SetMatrixIsFactorized(false);
        PivotThreshold() = 0.;

        isSymmetric = false;
		SetMatrixBuiltFromTriplets(false);
		numberOfRows = 0;
		numberOfColumns = 0;
		analyzedPatternLastNNZ = 0;
		reuseAnalyzedPattern = false;
	}

	//! information on storage type
	virtual LinearSolverType GetSystemMatrixType() const 
	{
		if (!isSymmetric) { return LinearSolverType::EigenSparse; }
		else { return LinearSolverType::EigenSparseSymmetric; }
	};

	//! flag which help to check if invalid matrix operations are performed:
	virtual void SetMatrixBuiltFromTriplets(bool flag = true) { matrixBuiltFromTriplets = flag; }
	virtual bool IsMatrixBuiltFromTriplets() const { return matrixBuiltFromTriplets; }

	//! flag to check if is symmetric:
	virtual void AssumeSymmetric(bool flag = true) 
	{ 
#if !defined(USE_SYMMETRIC_SOLVER)
		CHECKandTHROW(flag == false, "LinearSolver::EigenSparseSymmetric: (yet) not available; use EigenSparse");
#endif
		isSymmetric = flag; 
	}
	virtual bool IsSymmetric() const { return isSymmetric; }

	//! reuse pattern for faster computation [experimental]
	virtual void SetReuseAnalyzedPattern(bool flag) { reuseAnalyzedPattern = flag; }
	virtual bool GetReuseAnalyzedPattern() const { return reuseAnalyzedPattern; }

	//! get (read) matrix as dense exudyn Matrix
	const SparseTripletVector& GetSparseTriplets() const { return triplets; }

	//! get (write) matrix as dense exudyn Matrix; also in this case, solvability may be lost; sparse matrix is invalid
	//! however, we never know what else is done with the matrix afterwards ...
	SparseTripletVector& GetSparseTriplets() { 
		SetMatrixIsFactorized(false); 
		SetMatrixBuiltFromTriplets(false);  
		return triplets; 
	}

	//helper functions for matrix:
	virtual void SetNumberOfRowsAndColumns(Index numberOfRowsInit, Index numberOfColumnsInit)
	{
		SetMatrixIsFactorized(false);
		SetMatrixBuiltFromTriplets(false);

		numberOfRows = numberOfRowsInit;
		numberOfColumns = numberOfColumnsInit;
	}

	//! get number of columns
	virtual Index NumberOfRows() const { return numberOfRows; }
	//! get number of rows
	virtual Index NumberOfColumns() const { return numberOfColumns; }

	//! set all matrix items to zero (in dense matrix, all entries are set 0, in sparse matrix, the vector of items is erased)
	virtual void SetAllZero();

	//! reset matrices and free memory
	virtual void Reset();

	//! multiply either triplets or matrix entries with factor
	virtual void MultiplyWithFactor(Real factor);
	
	//! set the matrix with a dense matrix; do not use this function for computational tasks, as it will drop performance significantly
	virtual void SetMatrix(const Matrix& otherMatrix);

	//! add a diagonal (or unit) matrix located at a certain (rowOffset, columnOffset) position
	virtual void AddDiagonalMatrix(Real diagValue, Index numberOfRowsColumns, Index rowOffset = 0, Index columnOffset = 0);

	//! add (possibly) smaller factor*Matrix to this matrix, transforming the row indices of the submatrix with LTGrows and the column indices with LTGcolumns; 
	//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
	//! the offsets are with respect to the indices calculated from the LTGrows/columns transformation
	virtual void AddSubmatrix(const Matrix& submatrix, Real factor, const ArrayIndex& LTGrows, const ArrayIndex& LTGcolumns, Index rowOffset = 0, Index columnOffset = 0);

	//! add sparse triplets to sparse matrix
	virtual void AddSparseTriplets(const SparseTripletVector& otherTriplets);

	//! add (possibly) smaller factor*Transposed(Matrix) to this matrix, transforming the row indices of the submatrix with LTGrows and the column indices with LTGcolumns; 
	//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
	//! the offsets are with respect to the indices calculated from the LTGrows/columns transformation
	virtual void AddSubmatrixTransposed(const Matrix& submatrix, Real factor, const ArrayIndex& LTGrows, const ArrayIndex& LTGcolumns, Index rowOffset = 0, Index columnOffset = 0);

	//! add (possibly) smaller factor*Matrix to this matrix; 
	//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
	virtual void AddSubmatrixWithFactor(const Matrix& submatrix, Real factor, Index rowOffset = 0, Index columnOffset = 0);

	//! add (possibly) smaller factor*Transposed(Matrix) to this matrix; 
	//! in case of sparse matrices, only non-zero values are considered for the triplets (row,col,value)
	virtual void AddSubmatrixTransposedWithFactor(const Matrix& submatrix, Real factor, Index rowOffset = 0, Index columnOffset = 0);

	//! add possibly GeneralMatrix to this matrix; in case of sparse matrices, only the triplets (row,col,value) are added
	//! matrix types of submatrix and *this must be same
	//! operations must be both in triplet mode!
	virtual void AddSubmatrix(const GeneralMatrix& submatrix, Index rowOffset = 0, Index columnOffset = 0);

	//! add column vector 'vec' at 'column'; used to add a couple of entries during jacobian computation; filters zeros in sparse mode
	virtual void AddColumnVector(Index column, const Vector& vec, Index rowOffset = 0);

    //! add factor*(vectorAdd-vectorSub) at 'column'; used for numerical jacobians
    virtual void AddColumnVectorDiff(Index column, const Vector& vectorAdd, const Vector& vectorSub, Real factor, Index rowOffset = 0);
    
    //! After filling the matrix, it is finalized for further operations (matrix*vector, factorization, ...)
	virtual void FinalizeMatrix();

	//! factorize matrix (invert, SparseLU, etc.);
	//! return -1 if success, causing index otherwise
    virtual Index FactorizeNew(); // bool ignoreRedundantEquation = false, Index redundantEquationsStart = 0);

	//! multiply matrix with vector: solution = A*x
	//! this leads to memory allocation in case that the matrix is built from triplets
	virtual void MultMatrixVector(const Vector& x, Vector& solution);

	//! multiply matrix with vector and add to solution: solution += A*x
	//! this leads to memory allocation in case that the matrix is built from triplets
	virtual void MultMatrixVectorAdd(const Vector& x, Vector& solution);

	//! multiply transposed(matrix) with vector: solution = A^T*x
	//! this leads to memory allocation in case that the matrix is built from triplets
	virtual void MultMatrixTransposedVector(const Vector& x, Vector& solution);

	//! after factorization of matrix (=A), solve provides a solution vector (=x) for A*x = rhs ==> soluation = A^{-1}*rhs
	virtual void Solve(const Vector& rhs, Vector& solution);

	//! return a dense matrix from any other matrix: requires a copy - SLOW!
	virtual ResizableMatrix GetEXUdenseMatrix() const;

	//! function to print matrix
	virtual void PrintMatrix(std::ostream& os) const
	{
		os << GetEXUdenseMatrix();
	}
};

#else
class GeneralMatrixEigenSparse : public GeneralMatrixEXUdense
{
public:
	GeneralMatrixEigenSparse() { CHECKandTHROWstring("GeneralMatrixEigenSparse:: called when Eigen was deactivated!"); }

	//! information on storage type
	virtual LinearSolverType GetSystemMatrixType() const { return LinearSolverType::EigenSparse; };

};

#endif

#endif //include once
