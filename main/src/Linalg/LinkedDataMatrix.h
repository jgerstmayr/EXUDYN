/** ***********************************************************************************************
* @class		LinkedDataMatrix
* @brief		A matrix derived from Matrix for math operations with no own data (only linked to data of other objects)
* @details		Details:
                    - a matrix of Real entries (Real/float);
                    - use SlimMatrix for tiny matrices with known size
                    - use ResizableMatrix to allow a matrix to allocate more data than currently needed (no memory allocation when matrix size changes)
*
* @author		Gerstmayr Johannes
* @date			2020-05-15 (generated)
* @pre			Indizes of []-operator run from 0 to dataSize-1;
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
* 				
*
* *** Example code ***
*
* @code{.cpp}
* Matrix matrix(2,2,{1,2,3,4}); //matrix [[1,2],[3,4]]
* LinkedDataMatrix linkedMatrix(1, 2, m, 1, 0);	//linkedMatrix with 1 row and 2 columns, linked to row 1, column 0 of matrix, containing [3,4]
* @endcode
************************************************************************************************ */
#pragma once

#include <initializer_list> //for initializer_list in constructor
#include <ostream>          //ostream for matrix output as text
#include <iostream>         //! for cout @todo remove cout from class matrix ==> add Error handling

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Utilities/BasicFunctions.h"
#include "Linalg/Matrix.h"

template<typename T>
class LinkedDataMatrixBase: public MatrixBase<T>
{
public:
	//Constructors, Destructor
	//!!keep these constructors, because derived function Init() may not be called from parent class Matrix!!!

	//! Default constructor: no memory allocation yet; matrix of dimension 0 x 0
	LinkedDataMatrixBase() : MatrixBase() {}

	//! link to existing matrix
	LinkedDataMatrixBase(const MatrixBase<T>& m)
	{
		this->numberOfRows = m.numberOfRows;
		this->numberOfColumns = m.numberOfColumns;
		this->data = m.data;
	}

	//! create matrix with dimensions numberOfRowsInit x numberOfColumnsInit; initialize items with values stored at pointer T*; this method can potentially link to invalid data!
	//! can also link to vectors, matrices of other formats, etc.
	LinkedDataMatrixBase(const T* dataPointer, Index numberOfRowsInit, Index numberOfColumnsInit)
	{
		CHECKandTHROW((numberOfRowsInit >= 0 && numberOfColumnsInit >= 0),
			"LinkedDataMatrix::LinkedDataMatrix(Index, Index, T): invalid parameters");

		this->numberOfRows = numberOfRowsInit;
		this->numberOfColumns = numberOfColumnsInit;
		this->data = const_cast<T*>(dataPointer); //necessary, because otherwise cannot be used for const objects ==> do not abuse this feature!
	}

	//! link to a sub-matrix; only works, for full rows
	LinkedDataMatrixBase(const MatrixBase<T>& matrix, Index startRows, Index numberOfRowsLinked)
	{
		CHECKandTHROW((startRows >= 0 && startRows < matrix.NumberOfRows() && 
			numberOfRowsLinked + startRows <= matrix.NumberOfRows() && //use <= because this position is after the last element
			numberOfRowsLinked >= 0),
			"LinkedDataMatrix::LinkedDataMatrix(MatrixBase<T>, Index, Index): invalid parameters");
		//this->data = matrix.data[startRows*matrix.NumberOfColumns()];
		//this->data = &(matrix(startRows, 0));
		T* matrixData = const_cast<T*>(matrix.GetDataPointer());
		this->data = &(matrixData[startRows*matrix.NumberOfColumns()]);
		this->numberOfRows = numberOfRowsLinked;
		this->numberOfColumns = matrix.NumberOfColumns();
	}

	////! link to a sub-matrix; only works, for full rows
	//LinkedDataMatrixBase(const ResizableMatrixBase<T>& matrix, Index startRows, Index numberOfRowsLinked)
	//{
	//	CHECKandTHROW((startRows >= 0 && startRows < matrix.numberOfRows && numberOfRowsLinked + startRows <= matrix.numberOfRows && numberOfRowsLinked >= 0),
	//		"LinkedDataMatrix::LinkedDataMatrix(ResizableMatrixBase<T>, Index, Index): invalid parameters");
	//	this->data = matrix.data[startRows*matrix.numberOfColumns];
	//	this->numberOfRows = numberOfRowsLinked;
	//	this->numberOfColumns = matrix.numberOfColumns;
	//}

	//destructor: avoid that data is deleted
	virtual ~LinkedDataMatrixBase() 
	{
		this->data = nullptr; //because destructor ~MatrixBase<T>  AND  MatrixBase<T>::FreeMemory() are called hereafter ==> this will cause ~MatrixBase<T> not to delete anything!
		this->numberOfRows = 0;
		this->numberOfColumns = 0;
	}
	

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// BASIC FUNCTIONS
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

protected:
	//! allocate memory if numberOfRealsInit!=0; set data to allocated array of Reals or to nullptr; return false if failed
	virtual bool AllocateMemory(Index numberOfRowsInit, Index numberOfColumnsInit) override
	{
		CHECKandTHROWstring("LinkedDataMatrixBase<>::AllocateMemory not allowed");
		return true;
	}

	//! free memory if data!=nullptr
	virtual void FreeMemory() override {}

	//! if new size (rows*cols) fits into allocatedSize ==> reshape matrix; else: delete data if data != nullptr; Set new size of matrix; for external access, use 'SetNumberOfRowsAndColumns' to modify size of matrix
	virtual void ResizeMatrix(Index numberOfRowsInit, Index numberOfColumnsInit) override
	{
		CHECKandTHROW(numberOfRowsInit == this->NumberOfRows() && numberOfColumnsInit == this->NumberOfColumns(), "LinkedDataMatrixBase<>::ResizeMatrix: cannot change size of matrix");
	}


public:

	//operators+(Matrix,Matrix), etc. not implemented (as compared to ConstMatrix), because LinkedDataMatrix is intended for operations, where no copy is performed
	
	//! add matrix to *this matrix (for each component); both matrices must have same size; FAST / no memory allocation
	LinkedDataMatrixBase& operator+= (const MatrixBase<T>& matrix)
	{
		CHECKandTHROW((this->NumberOfRows() == matrix.NumberOfRows() && this->NumberOfColumns() == matrix.NumberOfColumns()), "LinkedDataMatrixBase::operator+=: incompatible number of rows and/or columns");
		Index cnt = 0;
		for (auto item : matrix) { this->data[cnt++] += item; }
		return *this;
	}

	//! add matrix from *this matrix (for each component); both matrices must have same size; FAST / no memory allocation
	LinkedDataMatrixBase<T>& operator-= (const MatrixBase<T>& matrix)
	{
		CHECKandTHROW((this->NumberOfRows() == matrix.NumberOfRows() && this->NumberOfColumns() == matrix.NumberOfColumns()), "LinkedDataMatrixBase::operator-=: incompatible number of rows and/or columns");
		Index cnt = 0;
		for (auto item : matrix) { this->data[cnt++] -= item; }
		return *this;
	}

	//! scalar multiply matrix *this with scalar (for each component); FAST / no memory allocation
	LinkedDataMatrixBase<T>& operator*= (T scalar)
	{
		for (auto &item : *this) { item *= scalar; }
		return *this;
	}


};

typedef LinkedDataMatrixBase<Real> LinkedDataMatrix;
typedef LinkedDataMatrixBase<float> LinkedDataMatrixF;

