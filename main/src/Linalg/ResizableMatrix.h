/** ***********************************************************************************************
* @class		ResizableMatrix
* @brief		A matrix derived from Matrix for math operations with allocated size and current size (independant); memory allocation
* @details		Details:
                    - a matrix of Real entries (Real/float);
                    - use SlimMatrix for tiny matrices with known size
                    - use LinkedDataMatrix to link data to a (part of a) matrix (without memory allocation)
                    - use ResizableMatrix to allow a matrix to allocate more data than currently needed (no memory allocation when matrix size changes)
*
* @author		Gerstmayr Johannes
* @date			1997-05-15 (generated)
* @date			2019-05-13 (last modified)
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
* ResizableMatrix m(5, 5);			//create a matrix with 25 Real items, allocated 50 entries
* m.SetNumberOfRowsAndColumns(4,4); //no memory allocated!
* m.SetNumberOfRowsAndColumns(7,4); //needs 28 items ==> memory allocated!
* @endcode
************************************************************************************************ */
#ifndef RESIZABLEMATRIX__H
#define RESIZABLEMATRIX__H

#include <initializer_list> //for initializer_list in constructor
#include <ostream>          //ostream for matrix output as text
#include <iostream>         //! for cout @todo remove cout from class matrix ==> add Error handling

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Utilities/BasicFunctions.h"
#include "Linalg/Matrix.h"

template<typename T>
class ResizableMatrixBase: public MatrixBase<T>
{
private:
	Index allocatedSize; //!< number of allocated Reals
public:

	//Constructors, Destructor
	//!!keep these constructors, because derived function Init() may not be called from parent class Matrix!!!

	//! Default constructor: no memory allocation; matrix of dimension 0 x 0
	ResizableMatrixBase()
	{
		Init();
	}

	//! create matrix with dimensions numberOfRowsInit x numberOfColumnsInit; data is not initialized; if allocatedSizeInit is valid==>allocate spezific size of Matrix
	ResizableMatrixBase(Index numberOfRowsInit, Index numberOfColumnsInit)
	{
		CHECKandTHROW((numberOfRowsInit >= 0 && numberOfColumnsInit >= 0),
			"ResizableMatrix::ResizableMatrix(Index, Index): invalid parameters");

		Init();
		ResizeMatrix(numberOfRowsInit, numberOfColumnsInit);
	}

	//! create matrix with dimensions numberOfRowsInit x numberOfColumnsInit; initialize items with 'initializationValue'
	ResizableMatrixBase(Index numberOfRowsInit, Index numberOfColumnsInit, T initializationValue)
	{
		CHECKandTHROW((numberOfRowsInit >= 0 && numberOfColumnsInit >= 0),
			"ResizableMatrix::ResizableMatrix(Index, Index, T): invalid parameters");

		Init();
		ResizeMatrix(numberOfRowsInit, numberOfColumnsInit);

		for (auto &item : *this) {
			item = initializationValue;
		}
	}

	//! create matrix with dimensions numberOfRowsInit x numberOfColumnsInit; initialize data with initializer list
	ResizableMatrixBase(Index numberOfRowsInit, Index numberOfColumnsInit, std::initializer_list<T> listOfReals)
	{
		CHECKandTHROW((numberOfRowsInit >= 0 && numberOfColumnsInit >= 0 &&
			numberOfRowsInit*numberOfColumnsInit == listOfReals.size()),
			"ResizableMatrix::ResizableMatrix(Index, Index, initializer_list): inconsistent size of initializer_list");

		Init();
		ResizeMatrix(numberOfRowsInit, numberOfColumnsInit);

		Index cnt = 0;
		for (auto value : listOfReals) {
			this->data[cnt++] = value; //use this-> to access member variables, because of templated base class
		}
	}

	//! copy constructor
	ResizableMatrixBase(const ResizableMatrixBase<T>& matrix)
	{
		Init();
		ResizeMatrix(matrix.NumberOfRows(), matrix.NumberOfColumns());

		Index cnt = 0;
		for (auto value : matrix) { this->data[cnt++] = value; } //use this-> to access member variables, because of templated base class
	}

	//! copy constructor, for type conversion from pure Matrix
	ResizableMatrixBase(const MatrixBase<T>& matrix)
	{
		Init();
		ResizeMatrix(matrix.NumberOfRows(), matrix.NumberOfColumns());

		Index cnt = 0;
		for (auto value : matrix) { this->data[cnt++] = value; }//use this-> to access member variables, because of templated base class
	}

	//destructor: not needed; called from Matrix ...
	//virtual ~ResizableMatrix()
	

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// BASIC FUNCTIONS
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

protected:
	//! allocate memory if numberOfRealsInit!=0; set data to allocated array of Reals or to nullptr; return false if failed
	virtual bool AllocateMemory(Index numberOfRowsInit, Index numberOfColumnsInit) override
	{
		allocatedSize = numberOfRowsInit * numberOfColumnsInit;
		return MatrixBase<T>::AllocateMemory(numberOfRowsInit, numberOfColumnsInit);
	}

	//! free memory if data!=nullptr
	virtual void FreeMemory() override
	{
		allocatedSize = 0;
		MatrixBase<T>::FreeMemory();
	}

	//! initialize matrix
	virtual void Init() override
	{
		this->numberOfRows = 0;		//use this-> to access member variables, because of templated base class
		this->numberOfColumns = 0;
		allocatedSize = 0;
		this->data = nullptr;
	};

	//! if new size (rows*cols) fits into allocatedSize ==> reshape matrix; else: delete data if data != nullptr; Set new size of matrix; for external access, use 'SetNumberOfRowsAndColumns' to modify size of matrix
	virtual void ResizeMatrix(Index numberOfRowsInit, Index numberOfColumnsInit) override
	{
		if (numberOfRowsInit*numberOfColumnsInit <= allocatedSize)
		{
			this->numberOfRows = numberOfRowsInit;		//use this-> to access member variables, because of templated base class
			this->numberOfColumns = numberOfColumnsInit;
			//SetNumberOfRowsAndColumns(numberOfRowsInit, numberOfColumnsInit);
		}
		else
		{
			FreeMemory();

			this->numberOfRows = numberOfRowsInit;
			this->numberOfColumns = numberOfColumnsInit;

			AllocateMemory(numberOfRowsInit, numberOfColumnsInit);
		}
	}


public:

	//operators+(Matrix,Matrix), etc. not implemented (as compared to ConstMatrix), because ResizeableMatrix is intended for operations, where no copy is performed
	
	//! add matrix to *this matrix (for each component); both matrices must have same size; FAST / no memory allocation
	ResizableMatrixBase& operator+= (const ResizableMatrixBase& matrix)
	{
		CHECKandTHROW((this->NumberOfRows() == matrix.NumberOfRows() && this->NumberOfColumns() == matrix.NumberOfColumns()), "Matrix::operator+=: incompatible number of rows and/or columns");
		Index cnt = 0;
		for (auto item : matrix) { this->data[cnt++] += item; }
		return *this;
	}

	//! add matrix from *this matrix (for each component); both matrices must have same size; FAST / no memory allocation
	ResizableMatrixBase<T>& operator-= (const ResizableMatrixBase<T>& matrix)
	{
		CHECKandTHROW((this->NumberOfRows() == matrix.NumberOfRows() && this->NumberOfColumns() == matrix.NumberOfColumns()), "Matrix::operator-=: incompatible number of rows and/or columns");
		Index cnt = 0;
		for (auto item : matrix) { this->data[cnt++] -= item; }
		return *this;
	}

	//! scalar multiply matrix *this with scalar (for each component); FAST / no memory allocation
	ResizableMatrixBase<T>& operator*= (T scalar)
	{
		for (auto &item : *this) { item *= scalar; }
		return *this;
	}


};

typedef ResizableMatrixBase<Real> ResizableMatrix;
typedef ResizableMatrixBase<float> ResizableMatrixF;

#endif
