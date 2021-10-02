/** ***********************************************************************************************
* @class		PyMatrixContainer
* @brief		Pybind11 interface to MatrixContainer
*
* @author		Gerstmayr Johannes
* @date			2020-05-11 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
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
	//?remove default constructor to enable conversion from py::object in constructor?
	PyMatrixContainer():MatrixContainer() {}
	
	//! initialize container with py::array_t or with emtpy list (default value)
	PyMatrixContainer(const py::object& matrix);
	//PyMatrixContainer(const py::object& matrix = py::list());

	//PyMatrixContainer(const py::array_t<Real>& pyArray);

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

	//! return a dense matrix from any other matrix: requires a copy - SLOW!
	py::array_t<Real> Convert2DenseMatrix() const
	{
		return EPyUtils::Matrix2NumPy(GetEXUdenseMatrix());
	}


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
