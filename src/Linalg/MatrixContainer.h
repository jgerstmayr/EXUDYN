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


		//! set both matrices to zero (in dense matrix, cols=rows=0; in sparse matrix, the vector of items is erased)
		void ClearAllMatrices() {
			//denseMatrix.SetAll(0); //this would be very slow if repeatedly called
			denseMatrix.Flush();
			sparseTripletMatrix.SetAllZero();
		}

		//! reset matrices and free memory
		void Reset() {
			if (useDenseMatrix) { denseMatrix = ResizableMatrix(); }
			else { sparseTripletMatrix.Reset(); }
		};

		//! set all entries to zero, but do not destroy memory or type of matrices
		void SetAllZero()
		{
			if (useDenseMatrix)
			{
				denseMatrix.SetAll(0.);
			}
			else
			{
				sparseTripletMatrix.SetAllZero();
			}
		}


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


#endif
