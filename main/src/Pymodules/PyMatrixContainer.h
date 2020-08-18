/** ***********************************************************************************************
* @class		PyMatrixContainer
* @brief		Pybind11 interface to MatrixContainer
*
* @author		Gerstmayr Johannes
* @date			2020-05-11 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
* 				
*
************************************************************************************************ */
#ifndef PYMATRIXCONTAINER__H
#define PYMATRIXCONTAINER__H

#include "Linalg/MatrixContainer.h"	

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>       //interface to numpy
#include <pybind11/buffer_info.h> //passing reference to matrix to numpy
namespace py = pybind11;            //! namespace 'py' used throughout in code
#include "Pymodules/PybindUtilities.h"

//! simple dense/sparse matrix container for simplistic operations; MatrixContainer can be used as interface for both sparse and dense matrices
class PyMatrixContainer: public EXUmath::MatrixContainer
{

public:
	//! create empty (dense) container
	PyMatrixContainer():MatrixContainer() {}
	
	//! initialize container with py::array_t or with emtpy list (default value)
	PyMatrixContainer(const py::object& matrix);

	//! set with dense numpy array; array (=matrix) contains values and size information
	void SetWithDenseMatrix(const py::array_t<Real>& pyArray, bool useDenseMatrixInit = true);

	//! set with sparse CSR matrix format: numpy array contains in every row [row, col, value]; numberOfRows and numberOfColumns given extra
	void SetWithSparseMatrixCSR(Index numberOfRowsInit, Index numberOfColumnsInit, const py::array_t<Real>& pyArray, bool useDenseMatrixInit = false);

	//!convert MatrixContainer to numpy array (dense) or dictionary (sparse): containing #rows, #columns, numpy matrix with triplets
	py::object GetPythonObject()
	{
		if (UseDenseMatrix())
		{
			return py::array_t<Real>(std::vector<std::ptrdiff_t>{(int)NumberOfRows(), (int)NumberOfColumns()}, GetInternalDenseMatrix().GetDataPointer());
		}
		else
		{
			//this function currently is very slow!
			auto d = py::dict();
			Matrix tripletMatrix = GetInternalSparseTripletsAsMatrix();
			py::array_t<Real> triplets = py::array_t<Real>(std::vector<std::ptrdiff_t>{(int)tripletMatrix.NumberOfRows(), (int)tripletMatrix.NumberOfColumns()}, tripletMatrix.GetDataPointer());
			d["numberOfRows"] = NumberOfRows();
			d["numberOfColumns"] = NumberOfColumns();
			d["triplets"] = triplets;
			return d;
		}
	}

	////! get number of columns
	//Index NumberOfRows() const {}

	////! get number of rows
	//Index NumberOfColumns() const {	}

	////! set all matrix items to zero (in dense matrix, all entries are set 0, in sparse matrix, the vector of items is erased)
	//void SetAllZero() {	}

	////! reset matrices and free memory
	//void Reset() {};

	////! multiply either triplets or matrix entries with factor
	//void MultiplyWithFactor(Real factor)
	//{
	//	if (useDenseMatrix) { denseMatrix *= factor; }
	//	else { sparseTripletMatrix.MultiplyWithFactor(factor); }
	//}

	//////! set the matrix with a dense matrix; do not use this function for computational tasks, as it will drop performance significantly
	////void SetMatrix(const Matrix& otherMatrix);

	////! multiply matrix with vector: solution = A*x
	////! this leads to memory allocation in case that the matrix is built from triplets
	//void MultMatrixVector(const Vector& x, Vector& solution)
	//{
	//	if (useDenseMatrix) { MultMatrixVectorTemplate<ResizableMatrix, Vector, Vector>(denseMatrix, x, solution); }
	//	else { sparseTripletMatrix.MultMatrixVector(x, solution); }
	//}

	////! multiply matrix with vector and add to solution: solution += A*x
	////! this leads to memory allocation in case that the matrix is built from triplets
	//void MultMatrixVectorAdd(const Vector& x, Vector& solution)
	//{
	//	if (useDenseMatrix) { MultMatrixVectorAddTemplate<ResizableMatrix, Vector, Vector>(denseMatrix, x, solution); }
	//	else { sparseTripletMatrix.MultMatrixVectorAdd(x, solution); }
	//}

	//////! multiply transposed(matrix) with vector: solution = A^T*x
	//////! this leads to memory allocation in case that the matrix is built from triplets
	////virtual void MultMatrixTransposedVector(const Vector& x, Vector& solution);

	////! return a dense matrix from any other matrix: requires a copy - SLOW!
	//ResizableMatrix GetEXUdenseMatrix() const
	//{
	//	if (useDenseMatrix) { return denseMatrix; }
	//	else { return sparseTripletMatrix.GetEXUdenseMatrix(); }
	//}

	////! function to print matrix
	//void PrintMatrix(std::ostream& os) const
	//{
	//	os << GetEXUdenseMatrix();
	//}

};

namespace EPyUtils {
	//numpy conversions
	inline bool SetPyMatrixContainerSafely(const py::dict& d, const char* itemName, PyMatrixContainer& destination)
	{
		if (d.contains(itemName))
		{
			py::object other = d[itemName]; //this is necessary to make isinstance work

			destination  = PyMatrixContainer(other);
			return true;
		}
		PyError(STDstring("ERROR: failed to convert '") + itemName + "' into MatrixContainer; dictionary:\n" + EXUstd::ToString(d));
		return false;
	}
	inline bool SetPyMatrixContainerSafely(const py::object& value, PyMatrixContainer& destination)
	{
		destination = PyMatrixContainer(value);
		return true;
	}


} //namespace EPyUtils

#endif
