/** ***********************************************************************************************
* @class		MatrixContainer
* @brief		A container for dense and sparse matrices for interfacing with objects that offer both matrix modes
*
* @author		Gerstmayr Johannes
* @date			2020-05-11 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */

#ifndef MATRIXCONTAINER__H
#define MATRIXCONTAINER__H

#include "Linalg/BasicLinalg.h"	


namespace EXUmath {

	//!Triplets for simple sparse matrix
	//!very much like Eigen, to be compatible in future!
	class Triplet
	{
	public:
		//Triplet() : m_row(0), m_col(0), m_value(0) {}
		Triplet() {}

		Triplet(const Index& row, const Index& col, const Real& value = 0) : m_row(row), m_col(col), m_value(value) {}

		const Index& row() const { return m_row; }
		const Index& col() const { return m_col; }
		const Real& value() const { return m_value; }

		//! in addition to Eigen, we also allow write access to value!
		Real& value() { return m_value; }
	private: //change to protected if derived class wanted (and also set destructor to virtual ...)
		Index m_row, m_col;
		Real m_value;
	};

	//! simple sparse matrix container for simplistic operations
	class SparseTripletMatrix
	{
	private: //change to protected if derived class wanted (and also set destructor to virtual ...)
		ResizableArray<Triplet> sparseTriplets;
		Index numberOfRows;
		Index numberOfColumns;
	public:
		SparseTripletMatrix() : numberOfRows(0), numberOfColumns(0) {}
		SparseTripletMatrix(Index numberOfRowsInit, Index numberOfColumnsInit, const ResizableArray<Triplet>& sparseTripletsInit)
		{
			for (const Triplet& triplet : sparseTripletsInit)
			{
				sparseTriplets.Append(triplet);
			}
		}
		//! set number of rows and columns
		void SetNumberOfRowsAndColumns(Index numberOfRowsInit, Index numberOfColumnsInit)
		{
			numberOfRows = numberOfRowsInit;
			numberOfColumns = numberOfColumnsInit;
		}

		//! const access to triplet list ==> can be used in Eigen::SparseMatrix
		const ResizableArray<Triplet>& GetTriplets() const { return sparseTriplets; }

		//! for swap with GeneralMatrix
		ResizableArray<Triplet>& GetTriplets() { return sparseTriplets; }

		//! add triplet
		void AddTriplet(const Triplet& triplet) { sparseTriplets.Append(triplet); }

		//! get number of columns
		Index NumberOfRows() const { return numberOfRows; }
		//! get number of rows
		Index NumberOfColumns() const { return numberOfColumns; }

		//! set all matrix items to zero (in dense matrix, all entries are set 0, in sparse matrix, the vector of items is erased)
		void SetAllZero() { SetNumberOfRowsAndColumns(0, 0); sparseTriplets.SetNumberOfItems(0); }

		//! reset matrices and free memory
		void Reset() { SetNumberOfRowsAndColumns(0, 0); sparseTriplets.SetMaxNumberOfItems(0); };

		//! multiply either triplets or matrix entries with factor
		void MultiplyWithFactor(Real factor)
		{
			for (auto& item : sparseTriplets)
			{
				item.value() *= factor;
			}
		}

		////! set the matrix with a dense matrix; do not use this function for computational tasks, as it will drop performance significantly
		//void SetMatrix(const Matrix& otherMatrix);

		//! multiply matrix with vector: solution = A*x
		//! this leads to memory allocation in case that the matrix is built from triplets
		void MultMatrixVector(const Vector& x, Vector& solution) const
		{
			solution.SetAll(0.); //! because some values may not be touched, others may be written several times ...

			for (const auto& item : sparseTriplets)
			{
				solution[item.row()] += x[item.col()] * item.value(); //must be "+=", becaues several values may be added!!!!
			}
		}

		//! multiply matrix with vector and add to solution: solution += A*x
		//! this leads to memory allocation in case that the matrix is built from triplets
		void MultMatrixVectorAdd(const Vector& x, Vector& solution) const
		{
			for (const auto& item : sparseTriplets)
			{
				solution[item.row()] += x[item.col()] * item.value(); 
			}
		}

		////! multiply transposed(matrix) with vector: solution = A^T*x
		////! this leads to memory allocation in case that the matrix is built from triplets
		//virtual void MultMatrixTransposedVector(const Vector& x, Vector& solution);
		
		//! multiply matrix with vector: solution = *this * matrix
		void MultMatrixDenseMatrix(const Matrix& matrix, Matrix& solution) const
		{
			CHECKandTHROW(NumberOfColumns() == matrix.NumberOfRows(), "SparseTripletMatrix::MultMatrixDenseMatrix: inconsistent matrices!");
			solution.SetNumberOfRowsAndColumns(NumberOfRows(), matrix.NumberOfColumns());
			solution.SetAll(0);

			Index mColumns = matrix.NumberOfColumns();
			for (const auto& item : sparseTriplets)
			{
				for (Index i = 0; i < mColumns; i++)
				{
					solution(item.row(), i) += item.value() * matrix(item.col(), i);
				}
			}
		}
		
		//! multiply matrix with vector: solution = matrix^T * *this
		void MultDenseMatrixTransposedMatrix(const Matrix& matrix, Matrix& solution) const
		{
			CHECKandTHROW(NumberOfRows() == matrix.NumberOfRows(), "SparseTripletMatrix::MultDenseMatrixTransposedMatrix: inconsistent matrices!");
			solution.SetNumberOfRowsAndColumns(matrix.NumberOfColumns(), NumberOfColumns());
			solution.SetAll(0);

			Index mColumns = matrix.NumberOfColumns();
			for (const auto& item : sparseTriplets)
			{
				for (Index i = 0; i < mColumns; i++)
				{
					solution(i, item.col()) += item.value() * matrix(item.row(), i);
				}
			}
		}

		//! add triplets to given dense matrix, using row and column offset and factor
		void AddToDenseMatrix(Matrix& denseMatrix, Index rowOffset = 0, Index colOffset = 0, Real factor = 1.) const
		{
			if (rowOffset == 0 and colOffset == 0 and factor == 1.)
			{
				for (auto& item : sparseTriplets)
				{
					denseMatrix(item.row(), item.col()) += item.value();
				}
			}
			else
			{
				for (auto& item : sparseTriplets)
				{
					denseMatrix(item.row()+rowOffset, item.col()+colOffset) += factor*item.value();
				}
			}
		}

		//! slow function which returns triplets as matrix
		Matrix GetTripletsAsMatrix() const
		{
			Matrix triplets(sparseTriplets.NumberOfItems(), 3);
			Index cnt = 0;
			for (const Triplet& item : sparseTriplets)
			{
				triplets(cnt, 0) = (Real)item.row();
				triplets(cnt, 1) = (Real)item.col();
				triplets(cnt, 2) = item.value();
				cnt++;
			}
			return triplets;
		}

		//! return a dense matrix from any other matrix: requires a copy - SLOW!
		ResizableMatrix GetEXUdenseMatrix() const
		{
			ResizableMatrix denseMatrix; // (NumberOfRows(), NumberOfColumns());
			Convert2DenseMatrix(denseMatrix);
			//denseMatrix.SetAll(0.);

			//for (auto& item : sparseTriplets)
			//{
			//	denseMatrix(item.row(), item.col()) += item.value();
			//}
			return denseMatrix;
		}

		//! convert to dense matrix without copying (but may create large matrices!)
		void Convert2DenseMatrix(ResizableMatrix& denseMatrix) const
		{
			denseMatrix.SetNumberOfRowsAndColumns(NumberOfRows(), NumberOfColumns());
			denseMatrix.SetAll(0.);

			for (auto& item : sparseTriplets)
			{
				denseMatrix(item.row(), item.col()) += item.value();
			}
		}

		//! function to print matrix
		void PrintMatrix(std::ostream& os) const
		{
			os << GetEXUdenseMatrix();
		}
	};

	//! simple dense/sparse matrix container for simplistic operations; MatrixContainer can be used as interface for both sparse and dense matrices
	class MatrixContainer
	{
	protected: //leave access for derived python class
		ResizableMatrix denseMatrix;
		SparseTripletMatrix sparseTripletMatrix;
		bool useDenseMatrix;

	public:
		MatrixContainer() { useDenseMatrix = true; } //default
		MatrixContainer(const Matrix& matrix) { denseMatrix = matrix; useDenseMatrix = true; }
		MatrixContainer(const SparseTripletMatrix& matrix) { sparseTripletMatrix = matrix; useDenseMatrix = false; }

		~MatrixContainer() {} //added for correct deletion of derived classes

		//! returns true, if matrix container uses dense matrix mode
		bool UseDenseMatrix() const { return useDenseMatrix; }

		//! get number of columns
		Index NumberOfRows() const { 
			if (useDenseMatrix) { return denseMatrix.NumberOfRows(); } 
			else { return sparseTripletMatrix.NumberOfRows(); } }

		//! get number of rows
		Index NumberOfColumns() const {
			if (useDenseMatrix) { return denseMatrix.NumberOfColumns(); } 
			else { return sparseTripletMatrix.NumberOfColumns(); } 
		}

		//! copy dense matrix or add triplets from other dense/sparse matrix converting rows/columns according to ltg-mapping; used in mass matrix user functions
		void CopyOrAddTriplets(const MatrixContainer& other, const ArrayIndex& ltg)
		{
			if (other.UseDenseMatrix())
			{
				SetUseDenseMatrix(true);
				denseMatrix = other.GetInternalDenseMatrix();
			}
			else
			{
				SetUseDenseMatrix(false);
				ResizableArray<Triplet>& triplets = GetInternalSparseTripletMatrix().GetTriplets();
				for (const Triplet& item : other.GetInternalSparseTripletMatrix().GetTriplets())
				{
					triplets.AppendPure(Triplet(ltg[item.row()], ltg[item.col()], item.value()));
				}
			}
		}

		//! copy dense matrix or add triplets from other dense/sparse matrix converting rows/columns according to ltg-mapping; used in mass matrix user functions
		void CopyOrAddTripletsWithFactor(const MatrixContainer& other, const ArrayIndex& ltg, Real factor)
		{
			if (other.UseDenseMatrix())
			{
				SetUseDenseMatrix(true);
				denseMatrix = other.GetInternalDenseMatrix();
				denseMatrix *= factor;
			}
			else
			{
				SetUseDenseMatrix(false);
				if (factor != 0.)
				{
					ResizableArray<Triplet>& triplets = GetInternalSparseTripletMatrix().GetTriplets();
					for (const Triplet& item : other.GetInternalSparseTripletMatrix().GetTriplets())
					{
						triplets.AppendPure(Triplet(ltg[item.row()], ltg[item.col()], factor*item.value()));
					}
				}
			}
		}

		//! set all matrix items to zero (in dense matrix, all entries are set 0, in sparse matrix, the vector of items is erased)
		void SetAllZero() {
			if (useDenseMatrix) { denseMatrix.SetAll(0); }
			else { sparseTripletMatrix.SetAllZero(); }
		}

		//! set both matrices to zero (in dense matrix, all entries are set 0, in sparse matrix, the vector of items is erased)
		void SetAllMatricesZero() {
			denseMatrix.SetAll(0);
			sparseTripletMatrix.SetAllZero();
		}

		//! reset matrices and free memory
		void Reset() {
			if (useDenseMatrix) { denseMatrix = ResizableMatrix(); }
			else { sparseTripletMatrix.Reset(); }
		};

		//! note that a switch to the other matrix may lead to undefined state ...
		void SetUseDenseMatrix(bool useDenseMatrixInit = true) {
			useDenseMatrix = useDenseMatrixInit;
		};

		//! multiply either triplets or matrix entries with factor
		void MultiplyWithFactor(Real factor)
		{
			if (useDenseMatrix) { denseMatrix *= factor; }
			else { sparseTripletMatrix.MultiplyWithFactor(factor); }
		}

		//! multiply matrix with vector: solution = A*x
		void MultMatrixVector(const Vector& x, Vector& solution) const
		{
			if (useDenseMatrix) { MultMatrixVectorTemplate<ResizableMatrix, Vector, Vector>(denseMatrix, x, solution); }
			else { sparseTripletMatrix.MultMatrixVector(x, solution); }
		}

		//! multiply matrix with vector and add to solution: solution += A*x
		void MultMatrixVectorAdd(const Vector& x, Vector& solution) const
		{
			if (useDenseMatrix) { MultMatrixVectorAddTemplate<ResizableMatrix, Vector, Vector>(denseMatrix, x, solution); }
			else { sparseTripletMatrix.MultMatrixVectorAdd(x, solution); }
		}

		////! multiply transposed(matrix) with vector: solution = A^T*x
		//virtual void MultMatrixTransposedVector(const Vector& x, Vector& solution) const;

		//! multiply matrixContainer with matrix: solution = *this * matrix
		void MultMatrixDenseMatrix(const Matrix& matrix, Matrix& solution) const
		{
			if (useDenseMatrix) { MultMatrixMatrixTemplate<ResizableMatrix, Matrix, Matrix>(denseMatrix, matrix, solution); }
			else { sparseTripletMatrix.MultMatrixDenseMatrix(matrix, solution); }
		}

		//! multiply matrixContainer with matrix: solution = matrix^T * *this
		void MultDenseMatrixTransposedMatrix(const Matrix& matrix, Matrix& solution) const
		{
			if (useDenseMatrix) { MultMatrixTransposedMatrixTemplate<Matrix, ResizableMatrix, Matrix>(matrix, denseMatrix, solution); }
			else { sparseTripletMatrix.MultDenseMatrixTransposedMatrix(matrix, solution); }
		}

		//! return a dense matrix from any other matrix: requires a copy - SLOW!
		ResizableMatrix GetEXUdenseMatrix() const
		{
			if (useDenseMatrix) { return denseMatrix; }
			else { return sparseTripletMatrix.GetEXUdenseMatrix(); }
		}

		//! this function fails in sparse matrix mode!
		const ResizableMatrix& GetInternalDenseMatrix() const
		{
			CHECKandTHROW(useDenseMatrix, "MatrixContainer::GetInternalDenseMatrix failed");
			return denseMatrix;
		}

		//! this function fails in sparse matrix mode!
		ResizableMatrix& GetInternalDenseMatrix()
		{
			CHECKandTHROW(useDenseMatrix, "MatrixContainer::GetInternalDenseMatrix failed");
			return denseMatrix;
		}

		//! this function fails in dense matrix mode! implementation is slow!
		const SparseTripletMatrix& GetInternalSparseTripletMatrix() const
		{
			if (useDenseMatrix) { CHECKandTHROWstring("MatrixContainer::GetInternalSparseTripletMatrix (const) failed"); return sparseTripletMatrix; }
			else { return sparseTripletMatrix; }
		}

		//! this function fails in dense matrix mode! implementation is slow!
		SparseTripletMatrix& GetInternalSparseTripletMatrix()
		{
			if (useDenseMatrix) { CHECKandTHROWstring("MatrixContainer::GetInternalSparseTripletMatrix failed"); return sparseTripletMatrix; }
			else { return sparseTripletMatrix; }
		}

		//! this function fails in dense matrix mode! implementation is slow!
		Matrix GetInternalSparseTripletsAsMatrix() const
		{
			if (useDenseMatrix) { CHECKandTHROWstring("MatrixContainer::GetInternalSparseTripletsAsMatrix failed"); return sparseTripletMatrix.GetTripletsAsMatrix(); }
			else { return sparseTripletMatrix.GetTripletsAsMatrix(); }
		}

		//! function to print matrix
		void PrintMatrix(std::ostream& os) const
		{
			os << GetEXUdenseMatrix();
		}

	};

}

typedef EXUmath::Triplet SparseTriplet;						//! this is a simple (row,col,value) structure for sparse matrix non zero entries
typedef ResizableArray<SparseTriplet> SparseTripletVector;	//! this vector stores (dynamically!) the triplets


#endif
