/** ***********************************************************************************************
* @brief		implementation for PyMatrixContainer
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

#include "PyMatrixContainer.h"	
#include "PybindUtilities.h"

////! initialize container with py::array_t or with emtpy list (default value)
//PyMatrixContainer::PyMatrixContainer(const py::array_t<Real>& pyArray)
//{
//	useDenseMatrix = true;
//	denseMatrix = EPyUtils::NumPy2Matrix(pyArray);
//}
//

//! initialize container with py::array_t or with emtpy list (default value)
PyMatrixContainer::PyMatrixContainer(const py::object& matrix)
{
	//pout << "PyMatrixContainer::PyMatrixContainer:\n";
	//py::print(matrix);
	if (py::isinstance<PyMatrixContainer>(matrix))
	{
		//pout << "works2: PyMatrixContainer::PyMatrixContainer:\n";
		*this = py::cast<PyMatrixContainer>(matrix);
		//pout << "  denseFlag=" << useDenseMatrix << "\n";
		//pout << "  matrix=" << GetEXUdenseMatrix() << "\n";
	}
	else if (py::isinstance<py::list>(matrix)) //process list, which is default in python constructor
	{
		//std::vector<Real> stdlist = py::cast<std::vector<Real>>(matrix); 

		//pout << "  is list\n";
		py::list pylist = py::cast<py::list>(matrix); 
		//pout << "list.size()==" << pylist.size() << "\n";
		
		if  (pylist.size() == 0)
		{
			useDenseMatrix = true;
			denseMatrix = ResizableMatrix();
		}
		else 
		{
			bool isInitialized = false;
			Index nRows = (Index)pylist.size();
			Index nCols = -1;
			Index iRow = 0;
			for (auto item : pylist)
			{
				py::print(item);
				Index iCol = 0;
				if (!py::isinstance<py::list>(item)) //process list, which is default in python constructor
				{
					CHECKandTHROWstring("MatrixContainer: list must be either empty or list of lists");
				}
				py::list pylist2 = py::cast<py::list>(item);

				if (!isInitialized)
				{
					nCols = (Index)pylist2.size();
					denseMatrix = ResizableMatrix(nRows, nCols);
					isInitialized = true;
				}
				else if (nCols != pylist2.size())
				{
					CHECKandTHROWstring("MatrixContainer: list of lists: number of floats must be same in all sub-lists");
				}

				for (auto value : pylist2)
				{
					denseMatrix(iRow, iCol) = py::cast<Real>(value);
					iCol++;
				}
				iRow++;
			}
		}
	}
	else if (py::isinstance<py::array>(matrix)) //process empty list, which is default in python constructor
	{
		useDenseMatrix = true;
		denseMatrix = EPyUtils::NumPy2Matrix(py::cast<py::array_t<Real>>(matrix));
	}
	else
	{
		CHECKandTHROWstring("MatrixContainer: can only initialize with empty list [], list of lists or with 2D numpy array");
	}
	//pout << "PyMatrixContainer::PyMatrixContainer:READY\n";

}

//! set with dense numpy array; array (=matrix) contains values and size information
void PyMatrixContainer::SetWithDenseMatrix(const py::array_t<Real>& pyArray, bool useDenseMatrixInit)
{
	if (useDenseMatrixInit)
	{
		useDenseMatrix = true;
		denseMatrix = EPyUtils::NumPy2Matrix(pyArray);
	}
	else
	{
		useDenseMatrix = false;
		if (pyArray.size() == 0) //process empty arrays, which leads to empty matrix, but has no dimension 2
		{
			sparseTripletMatrix.SetAllZero(); //empty matrix
		}
		else if (pyArray.ndim() == 2)
		{
			auto mat = pyArray.unchecked<2>();
			Index nrows = (Index)mat.shape(0);
			Index ncols = (Index)mat.shape(1);

			sparseTripletMatrix.SetNumberOfRowsAndColumns(nrows, ncols);
			for (Index i = 0; i < nrows; i++)
			{
				for (Index j = 0; j < ncols; j++)
				{
					if (mat(i, j) != 0.)
					{
						sparseTripletMatrix.AddTriplet(EXUmath::Triplet(i, j, mat(i, j)));
					}
				}
			}
		}
		else { CHECKandTHROWstring("MatrixContainer::SetWithDenseMatrix: illegal array format!"); }
	}
}

//! set with sparse CSR matrix format: numpy array contains in every row [row, col, value]; numberOfRows and numberOfColumns given extra
void PyMatrixContainer::SetWithSparseMatrixCSR(Index numberOfRowsInit, Index numberOfColumnsInit, const py::array_t<Real>& pyArray, bool useDenseMatrixInit)
{
	useDenseMatrix = useDenseMatrixInit;
	if (pyArray.size() != 0)
	{
		if (pyArray.ndim() == 2)
		{
			auto mat = pyArray.unchecked<2>();
			Index nrows = (Index)mat.shape(0);
			Index ncols = (Index)mat.shape(1);

			if (ncols != 3)
			{
				CHECKandTHROWstring("MatrixContainer::SetWithSparseMatrixCSR: array must have 3 columns: row, column and value!");
			}

			if (useDenseMatrixInit)
			{
				denseMatrix.SetNumberOfRowsAndColumns(numberOfRowsInit, numberOfColumnsInit);
				denseMatrix.SetAll(0.);

				for (Index i = 0; i < nrows; i++)
				{
					denseMatrix((Index)mat(i, 0), (Index)mat(i, 1)) += mat(i, 2); //use += in case that indices are duplicated
				}
			}
			else
			{
				sparseTripletMatrix.SetNumberOfRowsAndColumns(numberOfRowsInit, numberOfColumnsInit);
				sparseTripletMatrix.SetAllZero(); //empty matrix

				for (Index i = 0; i < nrows; i++)
				{
					sparseTripletMatrix.AddTriplet(EXUmath::Triplet((Index)mat(i, 0), (Index)mat(i, 1), mat(i, 2)));
				}
			}
		}
		else { CHECKandTHROWstring("MatrixContainer::SetWithSparseMatrixCSR: illegal array format!"); }
	}
}
