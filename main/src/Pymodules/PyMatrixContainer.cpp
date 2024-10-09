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
#include <pybind11/eigen.h>			//interface to eigen, to include scipy matrix interface
#include "Utilities/ExceptionsTemplates.h" //for exceptions in solver steps

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
	
	//seems that hasattr("shape") also detects attr("_shape"), and attr("shape") works for attr("_shape") !
	//pout << "has attr shape:" << py::hasattr(matrix, "shape") << "\n";
	//pout << "has attr _shape:" << py::hasattr(matrix, "_shape") << "\n";

    if (matrix.is_none())
    {
        useDenseMatrix = true;
        denseMatrix = ResizableMatrix();
    }
	else if (py::isinstance<PyMatrixContainer>(matrix))
	{
		//pout << "works2: PyMatrixContainer::PyMatrixContainer:\n";
		*this = py::cast<PyMatrixContainer>(matrix);
	}
	else if (py::isinstance<py::list>(matrix)) //process list, which is default in python constructor
	{
		py::list pylist = py::cast<py::list>(matrix); 
		useDenseMatrix = true;

		if  (pylist.size() == 0)
		{
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
				//py::print(item);
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
				else if (nCols != (Index)pylist2.size())
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
			//pout << "denseMatrix=" << denseMatrix << "\n";
		}
	}
	else if (IsScipySparseMatrix(matrix)) //do this before checking for py::array, as this may change in scipy in the future ...
	{
		//should be scipy sparse matrix
		py::tuple shape = matrix.attr("shape").cast<py::tuple>();
		Index numberOfRows = shape[0].cast<Index>();
		Index numberOfColumns = shape[1].cast<Index>();

		Initialize(numberOfRows, numberOfColumns, false); //useDenseMatrix = false; => sparse mode!
		AddSparseMatrixBase(matrix);
	}
	else if (py::isinstance<py::array>(matrix)) //process numpy array
	{
		//pout << "array\n";
		useDenseMatrix = true;
		denseMatrix = EPyUtils::NumPy2Matrix(py::cast<py::array_t<Real>>(matrix));
	}
	else
	{
		CHECKandTHROWstring("MatrixContainer: can only initialize with None, empty list [], list of lists, 2D numpy array, or scipy csr_matrix");
	}
	//pout << "PyMatrixContainer::PyMatrixContainer:READY\n";

}

//! set with dense numpy array; array (=matrix) contains values and size information
void PyMatrixContainer::SetWithDenseMatrix(const py::array_t<Real>& pyArray, bool useDenseMatrixInit, Real factor)
{
	if (useDenseMatrixInit)
	{
		useDenseMatrix = true;
		denseMatrix = EPyUtils::NumPy2Matrix(pyArray);
		if (factor != 1.) { denseMatrix *= factor; }
	}
	else
	{
		useDenseMatrix = false;
		if (pyArray.size() == 0) //process empty arrays, which leads to empty matrix, but has no dimension too
		{
			sparseTripletMatrix.SetAllZero(); //empty matrix
			sparseTripletMatrix.SetNumberOfRowsAndColumns(0, 0);
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
						sparseTripletMatrix.AddTriplet(EXUmath::Triplet(i, j, factor * mat(i, j)));
					}
				}
			}
		}
		else { CHECKandTHROWstring("MatrixContainer::SetWithDenseMatrix: illegal array format!"); }
	}
}

//! DEPRECATED: set with sparse CSR matrix format: numpy array contains in every row [row, col, value]; numberOfRows and numberOfColumns given extra
void PyMatrixContainer::SetWithSparseMatrixCSR(Index numberOfRowsInit, Index numberOfColumnsInit, const py::object& pyArray,
	bool useDenseMatrixInit, Real factor)
{
	GenericExceptionHandling([&]
	{
		if (py::isinstance<py::list>(pyArray) //process list, which is default in python constructor
			|| py::isinstance<py::array>(pyArray)) //process numpy array
		{
			SetOrAddSparseMatrixCSRBase(numberOfRowsInit, numberOfColumnsInit, py::cast<py::array>(pyArray), useDenseMatrixInit, false, factor);
		}
		else
		{
			CHECKandTHROWstring("MatrixContainer::SetWithSparseMatrixCSR: reading sparse matrix failed: invalid format");
		}
	}, "MatrixContainer::SetWithSparseMatrixCSR failed: it is very likely that pyArray does not have appropriate sparse triplet format");
}

//! set with sparse CSR matrix format: numpy array contains in every row [row, col, value]; numberOfRows and numberOfColumns given extra
void PyMatrixContainer::SetWithSparseMatrix(const py::object& sparseMatrix, Index numberOfRowsInit, Index numberOfColumnsInit,
	bool useDenseMatrixInit, Real factor)
{
	GenericExceptionHandling([&]
		{
			if (IsScipySparseMatrix(sparseMatrix))
			{
				if (!py::hasattr(sparseMatrix, "shape")) {
					CHECKandTHROWstring("MatrixContainer::SetWithSparseMatrix: reading scipy sparse matrix failed: did not detect shape");
				}
				py::tuple shape = sparseMatrix.attr("shape").cast<py::tuple>();
				Index numColumns = shape[1].cast<Index>();
				Index numRows = shape[0].cast<Index>();

				if (numberOfRowsInit == EXUstd::InvalidIndex || numberOfColumnsInit == EXUstd::InvalidIndex) 
				{ 
					CHECKandTHROW(numberOfRowsInit == numberOfColumnsInit, "MatrixContainer::SetWithSparseMatrix: rows / columns must be either both valid columns and rows or both having the exu.InvalidIndex().");
					//in case that both indices are invalid, we take the scipy - shape for initialization
					numberOfRowsInit = numRows;
					numberOfColumnsInit = numColumns;

				}
				else
				{ 
					CHECKandTHROW(numberOfRowsInit >= numRows && numberOfColumnsInit >= numColumns, "SetWithSparseMatrix: numberOfRows and numberOfColumns must be either default values (invalid index), or >= the dimensions of sparseMatrix");
				}

				Initialize(numberOfRowsInit, numberOfColumnsInit, useDenseMatrixInit);
				AddSparseMatrixBase(sparseMatrix, factor);
			}
			else if (py::isinstance<py::list>(sparseMatrix) //process list, which is default in python constructor
				|| py::isinstance<py::array>(sparseMatrix)) //process numpy array
			{
				SetOrAddSparseMatrixCSRBase(numberOfRowsInit, numberOfColumnsInit, py::cast<py::array>(sparseMatrix), useDenseMatrixInit, false, factor);
			}
			else { CHECKandTHROWstring("MatrixContainer::SetWithSparseMatrix: invalid matrix format!"); }

		}, "MatrixContainer::SetWithSparseMatrix failed: it is very likely that sparseMatrix is not a scipy csr_matrix or sparse triplet format");
}

//! set with sparse CSR matrix format: numpy array contains in every row [row, col, value]; numberOfRows and numberOfColumns given extra
void PyMatrixContainer::SetOrAddSparseMatrixCSRBase(Index numberOfRowsInit, Index numberOfColumnsInit, const py::array_t<Real>& pyArray, 
	bool useDenseMatrixInit, bool addMatrix, Real factor)
{
	if (!addMatrix) { useDenseMatrix = useDenseMatrixInit; }

	if (pyArray.size() != 0)
	{
		if (pyArray.ndim() == 2)
		{
			auto mat = pyArray.unchecked<2>();
			Index nrows = (Index)mat.shape(0);
			Index ncols = (Index)mat.shape(1);

			if (ncols != 3)
			{
				CHECKandTHROWstring("MatrixContainer::SetWithSparseMatrix: in case of triplets, array must have 3 columns: row, column and value!");
			}

			if (useDenseMatrix)
			{
				if (!addMatrix)
				{
					denseMatrix.SetNumberOfRowsAndColumns(numberOfRowsInit, numberOfColumnsInit);
					denseMatrix.SetAll(0.);
				}

				for (Index i = 0; i < nrows; i++)
				{
					denseMatrix((Index)mat(i, 0), (Index)mat(i, 1)) += factor * mat(i, 2); //use += in case that indices are duplicated
				}
			}
			else
			{
				if (!addMatrix)
				{
					sparseTripletMatrix.Reset(); //empty matrix, but also rows and columns...
					sparseTripletMatrix.SetMaxNumberOfItems(nrows); //not needed, but reduces memory defragmentation
					sparseTripletMatrix.SetNumberOfRowsAndColumns(numberOfRowsInit, numberOfColumnsInit);
				}

				for (Index i = 0; i < nrows; i++)
				{
					sparseTripletMatrix.AddTriplet(EXUmath::Triplet((Index)mat(i, 0), (Index)mat(i, 1), factor * mat(i, 2)));
				}
			}
		}
		else { CHECKandTHROWstring("MatrixContainer::SetWithSparseMatrix: illegal array format!"); }
	}
	else 
	{ 
		if (!addMatrix) //if only added, nowthing has to be done
		{
			if (useDenseMatrix)
			{
				if (numberOfColumnsInit == 0 && numberOfRowsInit == 0)
				{
					denseMatrix.SetNumberOfRowsAndColumns(0, 0);
				}
				else
				{
					CHECKandTHROWstring("MatrixContainer::SetWithSparseMatrix: when useDenseMatrix=true, array can only be empty if number of columns=rows=0!");
				}
			}
			else
			{
				//defines a certain matrix size, but with no triplets
				sparseTripletMatrix.SetNumberOfRowsAndColumns(numberOfRowsInit, numberOfColumnsInit);
				sparseTripletMatrix.SetAllZero(); //empty matrix
			}
		}
	}
}

//! add (ONLY) scipy sparse matrix multiplied with given factor; matrix must already be initialized
void PyMatrixContainer::AddSparseMatrixBase(const py::object& sparseMatrix, Real factor)
{
	CHECKandTHROW(IsScipySparseMatrix(sparseMatrix), "MatrixContainer::SetWithSparseMatrix or AddSparseMatrix: reading scipy sparse matrix failed, possibly due to wrong format");

	// Get the 'indptr', 'indices', and 'data' attributes
	py::array_t<Index> indptr = sparseMatrix.attr("indptr").cast<py::array_t<Index>>();
	py::array_t<Index> indices = sparseMatrix.attr("indices").cast<py::array_t<Index>>();
	py::array_t<Real> data = sparseMatrix.attr("data").cast<py::array_t<Real>>();

	// Compute the number of rows and non-zero elements
	Index num_rows = indptr.shape(0) - 1;  // 'indptr' length is 'num_rows + 1'
	Index nnz = (Index)data.size();            // Number of non-zero elements

	// Access the raw data
	auto indptr_ptr = indptr.unchecked<1>();
	auto indices_ptr = indices.unchecked<1>();
	auto data_ptr = data.unchecked<1>();

	// FUTURE: Reserve space in the vectors

	if (useDenseMatrix)
	{
		for (int row = 0; row < num_rows; ++row)
		{
			for (int idx = indptr_ptr(row); idx < indptr_ptr(row + 1); ++idx)
			{

				denseMatrix(row, indices_ptr(idx)) += factor * data_ptr(idx);
			}
		}
	}
	else
	{	//sparse:
		for (int row = 0; row < num_rows; ++row)
		{
			for (int idx = indptr_ptr(row); idx < indptr_ptr(row + 1); ++idx)
			{
				sparseTripletMatrix.AddTriplet(EXUmath::Triplet(row, indices_ptr(idx), factor * data_ptr(idx)));
			}
		}
	}
}

//! add with sparse triplets or scipy sparse matrix multiplied with given factor; matrix must already be initialized
void PyMatrixContainer::AddSparseMatrix(const py::object& sparseMatrix, Real factor)
{
	GenericExceptionHandling([&]
	{
		if (IsScipySparseMatrix(sparseMatrix))
		{
			AddSparseMatrixBase(sparseMatrix, factor);
		}
		else if (py::isinstance<py::list>(sparseMatrix) //process list, which is default in python constructor
			|| py::isinstance<py::array>(sparseMatrix)) //process numpy array
		{
			//to same as SetWithSparseMatrixCSR but without initialization (add new superfunction, which can do both set+add!
			SetOrAddSparseMatrixCSRBase(0, 0, py::cast<py::array>(sparseMatrix), false, true, factor); //check if this also works for lists of lists ...
		}
		else { CHECKandTHROWstring("MatrixContainer::AddSparseMatrix did not detect a valid sparse matrix format"); }
	}, "MatrixContainer::AddSparseMatrix failed: it is very likely that sparseMatrix is not a scipy csr matrix and has an inappropriate format");

}
