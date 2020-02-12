/** ***********************************************************************************************
* @class		MatrixBase
* @brief		A base matrix for math operations with memory allocated on heap; a change of size leads to new/delete
* @details		Details:
					- a matrix of Real entries (Real/float);
					- each constructor Matrix(rows, columns), operator=, operator+, operator-, etc. performs dynamic memory allocation; change of Matrix size requires memory allocation
					- for operation on large matrix data
					- use SlimMatrix for many matrices
					- use ConstSizeMatrix for few short matrices allocated on stack; use for temporary matrices in computation
					- use LinkedDataMatrix to link data to a (part of a) matrix (without memory allocation)
					- use ResizableMatrix to allow a matrix to allocate more data than currently needed (no memory allocation when matrix size changes)
*
* @author		Gerstmayr Johannes
* @date			1997-05-15 (generated)
* @date			2018-05-05 (last modified)
* @pre			Indizes of []-operator run from 0 to dataSize-1;
* 				Use Matrix for large matrix sizes; uses dynamic allocation (slow)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
* 				
*
* *** Example code ***
*
* @code{.cpp}
* Matrix m(100, 100);			//create a matrix with 100 x 100 Real entries; uses dynamic allocation
* for (Index i = 0; i < 100; i++) {
*   for (Index j = 0; j < 100; j++) {
*     m(i,j) = i*j;
* }
* Matrix m2 = m1;				//assign m1 to m2
* m1 += m2;					    //add m2 to m1
* cout << v1 << "\n";			//write m1 to cout
* @endcode
************************************************************************************************ */
#pragma once

#include <initializer_list> //for initializer_list in constructor
#include <ostream>          //ostream for matrix output as text
#include <iostream>         //for cout @todo remove cout from class matrix ==> add Error handling
#include <math.h>           //for pow

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

#ifdef __EXUDYN_RUNTIME_CHECKS__
extern Index matrix_new_counts; //global counter of item allocations; is increased every time a new is called
extern Index matrix_delete_counts; //global counter of item deallocations; is increased every time a delete is called
#endif

//! templated base matrix, which is used as Matrix (Real) or MatrixF (float) - for graphics
template<typename T>
class MatrixBase
{
protected:

	T* data;
	Index numberOfRows;
	Index numberOfColumns;

public:

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// CONSTRUCTOR, DESTRUCTOR
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! Default constructor: no memory allocation; matrix of dimension 0 x 0
	MatrixBase()
	{
		Init();
	};

	//! create matrix with dimensions numberOfRowsInit x numberOfColumnsInit; data is not initialized
	MatrixBase(Index numberOfRowsInit, Index numberOfColumnsInit)
	{
		//CHECKandTHROW((numberOfRowsInit >= 0 && numberOfColumnsInit >= 0) && "Matrix::Matrix(Index, Index): invalid parameters"); //unsigned int always >= 0

		Init();
		ResizeMatrix(numberOfRowsInit, numberOfColumnsInit);
	}
	//! @todo disable copy/move constructors/operators: NotCopyableOrMovable(NotCopyableOrMovable&&) = delete;
	//! NotCopyableOrMovable& operator=(NotCopyableOrMovable&&) = delete;
	//! Copyable(const Copyable& rhs) = default;
	//! Copyable& operator=(const Copyable& rhs) = default;

	//! create matrix with dimensions numberOfRowsInit x numberOfColumnsInit; initialize items with 'initializationValue'
	MatrixBase(Index numberOfRowsInit, Index numberOfColumnsInit, T initializationValue)
	{
		//CHECKandTHROW((numberOfRowsInit >= 0 && numberOfColumnsInit >= 0) && "Matrix::Matrix(Index, Index, T): invalid parameters"); //unsigned int always >= 0
		if (initializationValue != 0) { PyWarning("MatrixBase: initializationValue != 0"); }

		Init();
		ResizeMatrix(numberOfRowsInit, numberOfColumnsInit);

		for (auto &item : *this) {
			item = initializationValue;
		}
	}

	//! create matrix with dimensions numberOfRowsInit x numberOfColumnsInit; initialize data with initializer list
	MatrixBase(Index numberOfRowsInit, Index numberOfColumnsInit, std::initializer_list<T> listOfReals)
	{
		//CHECKandTHROW((numberOfRowsInit >= 0 && numberOfColumnsInit >= 0 &&  //unsigned int always >= 0
		CHECKandTHROW((numberOfRowsInit*numberOfColumnsInit == listOfReals.size()),
			"Matrix::Matrix(Index, Index, initializer_list): inconsistent size of initializer_list");

		Init();
		ResizeMatrix(numberOfRowsInit, numberOfColumnsInit);

		Index cnt = 0;
		for (auto value : listOfReals) {
			data[cnt++] = value;
		}
	}

	//! copy constructor, based on iterators
	MatrixBase(const MatrixBase<T>& matrix)
	{
		Init();
		ResizeMatrix(matrix.NumberOfRows(), matrix.NumberOfColumns());

		Index cnt = 0;
		for (auto value : matrix) { data[cnt++] = value; }
	}

	//! @todo Matrix: add constructor with SlimMatrix
	//Matrix(const SlimMatrix matrix);

	virtual ~MatrixBase<T>()
	{
		if (data != nullptr) //not necessary
		{
			delete[] data;
			data = nullptr;     //do not set to zero==>causes runtime error to detect multiple deletes!
#ifdef __EXUDYN_RUNTIME_CHECKS__
			matrix_delete_counts++;
#endif
		}
	};


	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// BASIC FUNCTIONS
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

protected:
	////! allocate memory if numberOfRealsInit!=0; set data to allocated array of Reals or to nullptr; return false if failed
	//virtual bool AllocateMemory(Index numberOfRowsInit, Index numberOfColumnsInit);

	////! free memory if data!=nullptr
	//virtual void FreeMemory();

	//! allocate memory if numberOfRealsInit!=0; set data to allocated array of Reals or to nullptr; return false if failed
	virtual bool AllocateMemory(Index numberOfRowsInit, Index numberOfColumnsInit)
	{
		if (numberOfRowsInit*numberOfColumnsInit == 0) { data = nullptr; }
		else
		{
			try {
				data = new T[numberOfRowsInit*numberOfColumnsInit];
			}
			catch (const std::bad_alloc& e) {
				pout << "Allocation failed: " << e.what() << '\n';
				pout << "requested memory = " << (8. * numberOfRowsInit*numberOfColumnsInit) / pow(2, 20) << " MB, rows = " << numberOfRowsInit << ", columns = " << numberOfColumnsInit << "\n";
				
				CHECKandTHROWstring("MatrixBase::Allocation failed");
				return false; //no success
			}
#ifdef __EXUDYN_RUNTIME_CHECKS__
			matrix_new_counts++; //only counted if try succeeded
#endif
		}
		return true;
	}

	//! free memory if data!=nullptr
	virtual void FreeMemory()
	{
		if (data != nullptr)
		{
			delete[] data;
			data = nullptr;
#ifdef __EXUDYN_RUNTIME_CHECKS__
			matrix_delete_counts++;
#endif
		}
	}


	//! initialize matrix ==> call constructor of own class
	virtual void Init()
	{
		numberOfRows = 0;
		numberOfColumns = 0;
		data = NULL;
	};

	//! delete data if data != nullptr; Set new size of matrix; for external access, use 'SetNumberOfRowsAndColumns' to modify size of matrix
	virtual void ResizeMatrix(Index numberOfRowsInit, Index numberOfColumnsInit)
	{
		FreeMemory();

		numberOfRows = numberOfRowsInit;
		numberOfColumns = numberOfColumnsInit;

		AllocateMemory(numberOfRowsInit, numberOfColumnsInit);
	}

public:

	virtual T* begin() const { return data; }							            //!< C++11 std::begin() for iterators; iterator range is always the currently used numberOfItems.
	//virtual const T* begin() const { return data; }				                //!< C++11 std::begin() for iterators, const version needed for ==, +=, etc.; iterator range is always the currently used numberOfItems.
	virtual T* end() const { return &data[numberOfRows*numberOfColumns]; }			//!< C++11 std::end() for iterators; iterator range is always the currently used numberOfItems.
	//virtual const T* end() const { return &data[numberOfRows*numberOfColumns]; }	//!< C++11 std::end() for iterators, const version needed for ==, +=, etc.; iterator range is always the currently used numberOfItems.

	Index NumberOfRows() const { return numberOfRows; };                            //!< number of columns (currently used)
	Index NumberOfColumns() const { return numberOfColumns; };                      //!< number of rows (currently used)
	T* GetDataPointer() const { return data; }									//!< return pointer to first data containing T numbers; const needed for LinkedDataVectors.
	virtual bool IsConstSizeMatrix() const { return false; }						//!< for derived classes: determine, if matrix has constant size

	//! Set rows and columns sizes (also used in derived classes)
	virtual void SetNumberOfRowsAndColumns(Index numberOfRowsInit, Index numberOfColumnsInit)
	{
		/*CHECKandTHROW((numberOfRowsInit >= 0 && numberOfColumnsInit >= 0), "Matrix::SetNumberOfRowsAndColumns(Index, Index): invalid parameters");*/
		if (numberOfRows != numberOfRowsInit || numberOfColumns != numberOfColumnsInit || data == NULL)
		{
			ResizeMatrix(numberOfRowsInit, numberOfColumnsInit);
		}
	}

	//! copy a submatrix from matrix, ranging rows[matrixStartRow,matrixEndRow] and columns[matrixEndColumns]; CopyFrom(m, 0,1,0,1) copy the item m(0,0)
	void CopyFrom(const MatrixBase& matrix, Index matrixStartRow, Index matrixStartColumn, Index matrixEndRow, Index matrixEndColumn)
	{
		CHECKandTHROW((/*matrixStartRow >= 0 && matrixStartColumn >= 0) &&*/
			matrixEndRow >= matrixStartRow && matrixEndColumn >= matrixStartColumn),
			"Matrix::CopyFrom(...): invalid parameters");
		ResizeMatrix(matrixEndRow - matrixStartRow, matrixEndColumn - matrixStartColumn);
		Index i = 0;
		for (Index row = matrixStartRow; row < matrixEndRow; row++)
		{
			for (Index column = matrixStartColumn; column < matrixEndColumn; column++)
			{
				data[i++] = matrix(row, column);
			}
		}
	}

	//! make a real copy of matrix
	template<class TMatrix>
	void CopyFrom(const TMatrix& matrix)
	//void CopyFrom(const MatrixBase& matrix)
	{
		ResizeMatrix(matrix.NumberOfRows(), matrix.NumberOfColumns());
		
		Index cnt = 0;
		for (auto value : matrix) { data[cnt++] = (T)value; }
	}

	//! set all items to given value.
	void SetAll(T value)
	{
		for (auto &item : *this) {
			item = value;
		}
	}

	//! set matrix with dimensions numberOfRowsInit x numberOfColumnsInit; initialize data with initializer list
	void SetMatrix(Index numberOfRowsInit, Index numberOfColumnsInit, std::initializer_list<T> listOfTs)
	{
		CHECKandTHROW((/*numberOfRowsInit >= 0 && numberOfColumnsInit >= 0 &&*/
			numberOfRowsInit*numberOfColumnsInit == listOfTs.size()),
			"Matrix::SetMatrix(Index, Index, initializer_list): inconsistent size of initializer_list");
		ResizeMatrix(numberOfRowsInit, numberOfColumnsInit);

		Index cnt = 0;
		for (auto value : listOfTs) {
			data[cnt++] = value;
		}
	}

	//Set Matrix with components 'value' in the diagonal and zero elsewhere
	void SetScalarMatrix(Index rowsColumns, T value)
	{
		//CHECKandTHROW(rowsColumns >= 0 && "Matrix::SetScalarMatrix(Index, T): invalid parameters!");
		SetNumberOfRowsAndColumns(rowsColumns, rowsColumns); //JG2019-05-13: changed from ResizeMatrix(...)

		for (Index i = 0; i < rowsColumns; i++)
		{
			for (Index j = 0; j < rowsColumns; j++)
			{
				if (i != j) (*this)(i, j) = 0;
				else (*this)(i, j) = value;
			}
		}
	}


	//! const (read) access of item with index 'i'; items run in range[0, numberOfRows*numberOfColumns]
	const T& GetItem(Index index) const
	{
		//CHECKandTHROW((index >= 0) && "Matrix::GetItem(Index) const: index < 0");
		CHECKandTHROW((index < numberOfRows*numberOfColumns), "Matrix::GetItem(Index) const: index >= numberOfRows*numberOfColumns");
		return data[index];
	}

	//! by reference (write) access of item with index 'i'; does not automatically increase array (compatibility with SlimArray<>)
	T& GetItem(Index index)
	{
		CHECKandTHROW((index < numberOfRows*numberOfColumns), "Matrix::GetItem(Index): index >= numberOfRows*numberOfColumns");
		return data[index];
	}

	//Referencing access-operator on element using row- and column-values
	T& GetItem(Index row, Index column)
	{
		CHECKandTHROW((row < numberOfRows), "Matrix::GetItem()(Index, Index): request of invalid row");
		CHECKandTHROW((column < numberOfColumns), "Matrix::GetItem()(Index, Index): request of invalid column");

		return data[row*numberOfColumns + column];
	}


	//Referencing constant access-operator on element using row- and column-values, WARNING: ZERO-BASED (DIFFERENT TO HOTINT1)
	const T& GetItem(Index row, Index column) const
	{
		CHECKandTHROW((row < numberOfRows), "Matrix::GetItem()(Index, Index) const: request of invalid row");
		CHECKandTHROW((column < numberOfColumns), "Matrix::GetItem()(Index, Index) const: request of invalid column");

		return data[row*numberOfColumns + column];
	};

	//void CopyFrom(const Matrix& m, Index row1, Index col1, Index row2, Index col2, const IVector& r);

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// OPERATORS
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! get pointer to row with Index 'row'; allows matrix access with [row][column] style (column is C-based array and does not include range check/assertion!!!)
	T* operator[](Index row)
	{
		CHECKandTHROW(/*(row >= 0) && */(row < numberOfRows), "Matrix::operator[](Index): request of invalid row");

		return &(data[row*numberOfColumns]);
	}

	T* operator[](Index row) const
	{
		CHECKandTHROW(/*(row >= 0) && */(row < numberOfRows), "Matrix::operator[](Index) const: request of invalid row");

		return &(data[row*numberOfColumns]);
	}

	//Referencing access-operator on element using row- and column-values
	T& operator()(Index row, Index column)
	{
		CHECKandTHROW(/*(row >= 0) && */(row < numberOfRows), "Matrix::operator()(Index, Index): request of invalid row");
		CHECKandTHROW(/*(column >= 0) && */(column < numberOfColumns), "Matrix::operator()(Index, Index): request of invalid column");

		return data[row*numberOfColumns + column];
	}


	//Referencing constant access-operator on element using row- and column-values, WARNING: ZERO-BASED (DIFFERENT TO HOTINT1)
	const T& operator()(Index row, Index column) const
	{
		CHECKandTHROW(/*(row >= 0) && */(row < numberOfRows), "Matrix::operator()(Index, Index) const: request of invalid row");
		CHECKandTHROW(/*(column >= 0) && */(column < numberOfColumns), "Matrix::operator()(Index, Index) const: request of invalid column");

		return data[row*numberOfColumns + column];
	};

	//Assignment-operator
	MatrixBase<T>& operator= (const MatrixBase<T>& matrix)
	{
		if (this == &matrix) { return *this; }

		ResizeMatrix(matrix.NumberOfRows(), matrix.NumberOfColumns());

		Index cnt = 0;
		for (auto value : matrix) { data[cnt++] = value; }

		return *this;
	}

	//! comparison operator, component-wise compare; MATRIX DIMENSIONS MUST BE SAME; returns true, if all components are equal
	bool operator== (const MatrixBase<T>& matrix) const
	{
		CHECKandTHROW((NumberOfRows() == matrix.NumberOfRows() && NumberOfColumns() == matrix.NumberOfColumns()), "Matrix::operator==: incompatible number of rows and/or columns");
		Index cnt = 0;
		for (const auto item : matrix)
		{
			if (item != data[cnt++]) { return false; }
		}
		return true;
	}

	//! add matrix to *this matrix (for each component); both matrices must have same size; FAST / no memory allocation
	MatrixBase<T>& operator+= (const MatrixBase<T>& matrix)
	{
		CHECKandTHROW((NumberOfRows() == matrix.NumberOfRows() && NumberOfColumns() == matrix.NumberOfColumns()), "Matrix::operator+=: incompatible number of rows and/or columns");
		Index cnt = 0;
		for (auto item : matrix) { data[cnt++] += item; }
		return *this;
	}

	//! add matrix from *this matrix (for each component); both matrices must have same size; FAST / no memory allocation
	MatrixBase<T>& operator-= (const MatrixBase<T>& matrix)
	{
		CHECKandTHROW((NumberOfRows() == matrix.NumberOfRows() && NumberOfColumns() == matrix.NumberOfColumns()), "Matrix::operator-=: incompatible number of rows and/or columns");
		Index cnt = 0;
		for (auto item : matrix) { data[cnt++] -= item; }
		return *this;
	}

	//! scalar multiply matrix *this with scalar (for each component); FAST / no memory allocation
	MatrixBase<T>& operator*= (T scalar)
	{
		for (auto &item : *this) { item *= scalar; }
		return *this;
	}

	//! scalar divide matrix *this with scalar (for each component); no check against division by zero; FAST / no memory allocation
	MatrixBase<T>& operator/= (T scalar)
	{
		for (auto &item : *this) { item /= scalar; }
		return *this;
	}

	//! add two matrices m1 and m2 (for each component); creates new Matrix / memory allocation (MAY BE SLOW)
	friend MatrixBase<T> operator+ (const MatrixBase<T>& m1, const MatrixBase<T>& m2)
	{
		CHECKandTHROW(m1.NumberOfColumns() == m2.NumberOfColumns() && m1.NumberOfRows() == m2.NumberOfRows(),
			"operator+(Matrix,Matrix): Size mismatch");

		MatrixBase<T> result(m1.NumberOfRows(), m1.NumberOfColumns());
		Index cnt = 0;
		for (auto &item : result)
		{
			item = m1.GetItem(cnt) + m2.GetItem(cnt);
			cnt++;
		}
		return result;
	}

	//! subtract matrix m2 from m1 (for each component); creates new Matrix / memory allocation (MAY BE SLOW)
	friend MatrixBase<T> operator- (const MatrixBase<T>& m1, const MatrixBase<T>& m2)
	{
		CHECKandTHROW(m1.NumberOfColumns() == m2.NumberOfColumns() && m1.NumberOfRows() == m2.NumberOfRows(),
			"operator-(Matrix,Matrix): Size mismatch");

		MatrixBase<T> result(m1.NumberOfRows(), m1.NumberOfColumns());
		Index cnt = 0;
		for (auto &item : result)
		{
			item = m1.GetItem(cnt) - m2.GetItem(cnt);
			cnt++;
		}
		return result;
	}

	//! multiply matrix m1*m2 (matrix multiplication); algorithm has order O(n^3); creates new Matrix / memory allocation (MAY BE SLOW)
	friend MatrixBase<T> operator* (const MatrixBase<T>& m1, const MatrixBase<T>& m2)
	{
		CHECKandTHROW(m1.NumberOfColumns() == m2.NumberOfRows(),
			"operator*(Matrix,Matrix): Size mismatch");

		MatrixBase<T> result(m1.NumberOfRows(), m2.NumberOfColumns());

		for (Index i = 0; i < m2.NumberOfColumns(); i++)
		{
			for (Index j = 0; j < m1.NumberOfRows(); j++)
			{
				T value = 0;
				for (Index k = 0; k < m1.NumberOfColumns(); k++)
				{
					value += m1(j, k)*m2(k, i);
				}
				result(j, i) = value;
			}
		}
		return result;
	}

	//! multiply matrix with scalar value; creates new Matrix / memory allocation (MAY BE SLOW)
	friend MatrixBase<T> operator* (const MatrixBase<T>& matrix, const T& value)
	{
		MatrixBase<T> result = matrix;
		result *= value;
		return result;
	}

	//! multiply scalar value with matrix; creates new Matrix / memory allocation (MAY BE SLOW)
	friend MatrixBase<T> operator* (const T& value, const MatrixBase<T>& matrix)
	{
		MatrixBase<T> result = matrix;
		result *= value;
		return result;
	}

	//friend Matrix operator* (const Matrix& mat, const T& val);
	//friend VectorBase<T> operator* (const VectorBase<T>& v, const Matrix& m);
	//friend VectorBase<T> operator* (const Matrix& matrix, const VectorBase<T>& vector);

	//! matrix*vector multiplication with given result vector; invokes memory allocation
	//! for versions without memory allocation, use EXUmath::MultMatrixVector(...)
	//template<class TVector>
	friend VectorBase<T> operator* (const MatrixBase<T>& matrix, const VectorBase<T> & vector)
	{
		CHECKandTHROW(matrix.NumberOfColumns() == vector.NumberOfItems(),
			"operator*(Matrix,TVector): Size mismatch");

		VectorBase<T> result(matrix.NumberOfRows());

		for (Index i = 0; i < result.NumberOfItems(); i++)
		{
			T resultRow = 0;
			for (Index j = 0; j < vector.NumberOfItems(); j++)
			{
				resultRow += matrix(i, j)*vector[j];
			}
			result[i] = resultRow;
		}
		return result;
	}


	//! @brief Output operator << generates ostream "[m[0][0] m[0][1] ... m[0][m]; ... m[n][m]]" for a matrix m;
	//! the FORMAT IS DIFFERENT TO HOTINT1
	//! @todo check Vector/Matrix ostream output format to be compatible with Python / NumPy
	friend std::ostream& operator<<(std::ostream& os, const MatrixBase<T>& matrix)
	//template<typename T>
	//inline std::ostream& operator<<(std::ostream& os, const MatrixBase<T>& matrix)
	{
		if (linalgPrintUsePythonFormat) 
		{
			os << "[";
			for (Index row = 0; row < matrix.NumberOfRows(); row++)
			{
				os << "[";
				for (Index column = 0; column < matrix.NumberOfColumns(); column++)
				{
					os << matrix(row, column);
					if (column != matrix.NumberOfColumns() - 1) { os << ","; }
				}
				os << "]";
				if (row != matrix.NumberOfRows() - 1) { os << ","; } 
			}
			os << "]";
		}
		else
		{
			os << "[";
			for (Index row = 0; row < matrix.NumberOfRows(); row++)
			{
				for (Index column = 0; column < matrix.NumberOfColumns(); column++)
				{
					os << matrix(row, column);
					if (column != matrix.NumberOfColumns() - 1) { os << " "; }
				}
				if (row != matrix.NumberOfRows() - 1) { os << "; "; } //';' and '[]' as a compromise between Python and MATLAB format
			}
			os << "]";
		}
		return os;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// EXTENDED FUNCTIONS
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! computes and returns the transposed of *this (does not change *this); memory allocation (MAY BE SLOW)
	MatrixBase<T> GetTransposed() const
	{
		MatrixBase<T> result(numberOfColumns, numberOfRows);

		for (Index i = 0; i < numberOfRows; i++) {
			for (Index j = 0; j < numberOfColumns; j++) {
				result(j, i) = (*this)(i, j);
			}
		}
		return result;
	}

	//! transposes *this; currently only works for square matrix
	//! @todo check efficient implementation of tranpose for non-square matrices
	void TransposeYourself()
	{
		CHECKandTHROW(IsSquare(), "Matrix::GetTransposed: matrix must be square!");

		for (Index i = 0; i < numberOfRows; i++) {
			for (Index j = 0; j < i; j++) { //operates only on lower left triangular matrix
				EXUstd::Swap((*this)(i, j), (*this)(j, i));
			}
		}
	}

	//! return whether *this is a square matrix (numberOfRows == numberOfColumns)
	bool IsSquare() const { return numberOfRows == numberOfColumns; }

	//! get column vector as a SlimVector (no memory allocation, but final size needs to be known at compile time)
	//! use e.g.: Vector3D v = GetColumnVector<3>(i);
	template<Index columnSize>
	SlimVectorBase<T, columnSize> GetColumnVector(Index column) const
	{
		CHECKandTHROW(this->numberOfRows == columnSize,
			"ConstSizeMatrixBase::GetColumnVector(...): size mismatch");
		CHECKandTHROW(column <= this->numberOfColumns,
			"ConstSizeMatrixBase::GetColumnVector(...): illegal column");

		SlimVectorBase<T, columnSize> result;

		for (Index i = 0; i < this->numberOfRows; i++)
		{
			result[i] = (*this)(i, column);
		}
		return result;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// SOLVER
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! Compute matrix inverse (internal function, needs memory allocation and may be slower than external functions - Eigen, etc.)
	//  NOT Threadsafe!
	bool Invert();

	//! Solve System of Equations with right hand side 'fv' and solution 'q'; needs memory allocation and may be slower than external functions, e.g. Eigen
	//  NOT Threadsafe!
	bool Solve(const VectorBase<T>& rhs, VectorBase<T>& q);
		
	//Adds row factor*'fromRow' to row 'toRow'
	void AddRowToRowWithFactor(SignedIndex fromRow, SignedIndex toRow, T factor)
	{
		CHECKandTHROW((toRow < (SignedIndex)numberOfRows) && (fromRow < (SignedIndex)numberOfRows), "Matrix::AddRowToRowWithFactor(SignedIndex,SignedIndex, T): invalid toRow");

		SignedIndex j = (fromRow - toRow)*(SignedIndex)numberOfColumns;  //offset
		SignedIndex end = (toRow+1) * (SignedIndex)numberOfColumns-1;
		for (SignedIndex i = toRow* (SignedIndex)numberOfColumns; i <= end; i++)
		{
			data[i] += factor * data[j + i];
		}
	}

	//Adds row factor*'fromRow' to row 'toRow'; consider only columns 'fromCol' ... 'toCol'
	void AddRowToRowWithFactor(SignedIndex fromRow, SignedIndex toRow, T factor, SignedIndex fromCol, SignedIndex toCol)
	{
		CHECKandTHROW((toRow < (SignedIndex)numberOfRows)&& (fromRow < (SignedIndex)numberOfRows), "Matrix::AddRowToRowWithFactor: invalid toRow");

		SignedIndex j = (fromRow - toRow)*(SignedIndex)numberOfColumns; //offset
		SignedIndex end = toRow* (SignedIndex)numberOfColumns + toCol;
		for (SignedIndex i = toRow* (SignedIndex)numberOfColumns + fromCol; i <= end; i++)
		{
			data[i] += factor * data[j + i];
		}
	}

	//Multiplies the row 'row' with 'value'
	void MultiplyRow(Index row, T value)
	{
		CHECKandTHROW(row < numberOfRows, "Matrix::MultiplyRow: invalid row");

		for (Index i = row*numberOfColumns; i < (row+1) * numberOfColumns; i++) { data[i] *= value; }
	}

	//! Swaps the rows 'row1' and 'row2'
	void SwapRows(Index row1, Index row2)
	{
		if (row1 == row2) { return; }
		CHECKandTHROW((row1 < numberOfRows) && (row2 < numberOfRows), "Matrix::SwapRows: invalid row");

		for (Index i = 0; i < numberOfColumns; i++) 
		{ 
			EXUstd::Swap((*this)(row1, i), (*this)(row2, i)); 
		}
	}

	//! Swaps the columns 'column1' and 'column2'
	void SwapColumns(Index columns1, Index columns2)
	{
		if (columns1 == columns2) { return; }
		CHECKandTHROW((columns1 < numberOfColumns) && (columns2 < numberOfColumns), "Matrix::SwapColumns: invalid column");

		for (Index i = 0; i < numberOfRows; i++) 
		{ 
			EXUstd::Swap((*this)(i, columns1), (*this)(i, columns2)); 
		}
	}

	//! Add matrix m at indices given in ArrayIndex x ArrayIndex to *this
	void AddMatrix(const ArrayIndex& rowref, const ArrayIndex& colref, const MatrixBase<T>& m)
	{
		Index i, j;
		Index k;
		for (i = 0; i < m.numberOfRows; i++)
		{
			k = rowref[i]*numberOfColumns;
			for (j = 0; j < m.numberOfColumns; j++)
			{
				data[k + colref[j]] += m(i, j);
			}
		}
	}

	//! Add submatrix 'sm' with possibly smaller size than this at (*this) 'row' and 'column'
	void AddSubmatrix(const MatrixBase<T>& sm, Index row = 0, Index column = 0)
	{
		CHECKandTHROW(row + sm.NumberOfRows() <= NumberOfRows() && column + sm.NumberOfColumns() <= NumberOfColumns(), "Matrix::AddSubmatrix size mismatch");

		for (Index i = 0; i < sm.numberOfRows; i++)
		{
			for (Index j = 0; j < sm.numberOfColumns; j++)
			{
				data[(i + row)*numberOfColumns + column + j] += sm(i, j);
			}
		}
	}

	//! Add submatrix (factor * sm) with possibly smaller size than *this matrix
	//! the destination rows and columns of the submatrix (sm) relative to row and column are given in LTGrows and LTGcolumns
	void AddSubmatrix(const MatrixBase<T>& sm, Real factor, const ResizableArray<Index>& LTGrows, const ResizableArray<Index>& LTGcolumns, Index row = 0, Index column = 0)
	{
		CHECKandTHROW(row + sm.NumberOfRows() <= NumberOfRows() && column + sm.NumberOfColumns() <= NumberOfColumns(), "Matrix::AddSubmatrix(2) size mismatch");

		for (Index i = 0; i < sm.numberOfRows; i++)
		{
			for (Index j = 0; j < sm.numberOfColumns; j++)
			{
				data[(LTGrows[i] + row)*numberOfColumns + column + LTGcolumns[j]] += factor * sm(i, j);
				//jacobian(nODE2 + ltgAE[ii], ltgODE2[jj]) += factorAE_ODE2 * temp.localJacobianAE(ii, jj); //depends, if velocity or position level is used //factorVelocityLevel
			}
		}
	}

	//! Add transposed submatrix (factor * sm) with possibly smaller size than *this matrix
	//! the destination rows and columns of the submatrix (sm) relative to row and column are given in LTGrows and LTGcolumns
	void AddSubmatrixTransposed(const MatrixBase<T>& sm, Real factor, const ResizableArray<Index>& LTGrows, const ResizableArray<Index>& LTGcolumns, Index row = 0, Index column = 0)
	{
		for (Index j = 0; j < sm.numberOfRows; j++)
		{
			for (Index i = 0; i < sm.numberOfColumns; i++)
			{
				data[(LTGrows[i] + row)*numberOfColumns + column + LTGcolumns[j]] += factor * sm(j, i);
				//jacobian(nODE2 + ltgAE[ii], ltgODE2[jj]) += factorAE_ODE2 * temp.localJacobianAE(ii, jj); //depends, if velocity or position level is used //factorVelocityLevel
			}
		}
	}

	//! Set submatrix 'sm'*factor with possibly smaller size than this at (*this) 'row' and 'column'
	void SetSubmatrix(const MatrixBase<T>& sm, Index row = 0, Index column = 0, Real factor = 1.)
	{
		CHECKandTHROW(row + sm.NumberOfRows() <= NumberOfRows() && column + sm.NumberOfColumns() <= NumberOfColumns(), "Matrix::SetSubmatrix size mismatch");

		for (Index i = 0; i < sm.numberOfRows; i++)
		{
			for (Index j = 0; j < sm.numberOfColumns; j++)
			{
				data[(i + row)*numberOfColumns + column + j] = factor * sm(i, j);
			}
		}
	}

	//! Get submatrix at certain row/column with numberOfRows/numberOfColumnsGet taken; performs COPY and may be SLOW
	MatrixBase<T> GetSubmatrix(Index startRow, Index startColumn,
		Index numberOfRowsGet, Index numberOfColumnsGet)
	{
		CHECKandTHROW(startRow + numberOfRowsGet <= NumberOfRows() && startColumn + numberOfColumnsGet <= NumberOfColumns(), "Matrix::GetSubmatrix index mismatch");

		MatrixBase<T> sm(numberOfRowsGet, numberOfColumnsGet);
		for (Index i = startRow; i < startRow + numberOfRowsGet; i++)
		{
			for (Index j = startColumn; j < startColumn + numberOfColumnsGet; j++)
			{
				sm(i-startRow,j-startColumn) = data[i*numberOfColumns + j];
			}
		}
		return sm;
	}

	//add transposed submatrix 'sm' with possibly smaller size than this at (*this) 'row' and 'column'
	void AddTransposedSubmatrix(const MatrixBase<T>& sm, Index row = 0, Index column = 0)
	{
		CHECKandTHROW(row + sm.NumberOfColumns() <= NumberOfRows() && column + sm.NumberOfRows() <= NumberOfColumns(), "Matrix::AddSubmatrix size mismatch");

		for (Index i = 0; i < sm.numberOfColumns; i++)
		{
			for (Index j = 0; j < sm.numberOfRows; j++)
			{
				data[(i + row)*numberOfColumns + column + j] += sm(j, i);
			}
		}
	}


	//! Returns the maximum-norm (largest absolute value in matrix)
	T MaxNorm() const;


	////Returns Determinate of matrix
	//T Det() const;

	////Returns the minimum-norm (smallest value in matrix)
	//T MinNorm() const;

	////Returns the quadratic-norm
	//T Norm2() const;

	////Returns true if matrix is symmetric
	//Index IsSymmetric();
	//Index IsSymmetric(T eps); //version with small tolerance epsilon
	//void MakeSymmetric(); //makes a matrix really symmetric (if there are small errors)
};

typedef MatrixBase<Real> Matrix;
typedef MatrixBase<float> MatrixF;


namespace EXUmath {

	//implement the following functions as templates within namespace EXUmath::MultMatrixVector(...), ...
	//! matrix*vector multiplication with given result vector (does not invoke memory allocation if result vector has appropriate size)
	template<class TMatrix, class TVector, class TVectorResult>
	inline void MultMatrixVectorTemplate(const TMatrix& matrix, const TVector& vector, TVectorResult& result)
	{
		CHECKandTHROW(matrix.NumberOfColumns() == vector.NumberOfItems(),
			"Hmath::MultMatrixVector(matrix,vector,result,T): Size mismatch");

		result.SetNumberOfItems(matrix.NumberOfRows());

		auto* mm = matrix.GetDataPointer();
		const auto* vv = vector.GetDataPointer();
		Index resultLength = result.NumberOfItems();
		Index vectorLength = vector.NumberOfItems();

		for (Index i = 0; i < resultLength; i++)
		{
			result[i] = 0;
			auto* mr = &mm[i*matrix.NumberOfColumns()];
			for (Index j = 0; j < vectorLength; j++)
			{
				result[i] += mr[j] * vv[j];
			}
		}
	}

	//implement the following functions as templates within namespace EXUmath::MultMatrixVector(...), ...
	//! matrix.GetTranspose()*vector multiplication with given result vector (does not invoke memory allocation if result vector has appropriate size)
	template<class TMatrix, class TVector, class TVectorResult>
	inline void MultMatrixTransposedVectorTemplate(const TMatrix& matrix, const TVector& vector, TVectorResult& result)
	{
		CHECKandTHROW(matrix.NumberOfRows() == vector.NumberOfItems(),
			"Hmath::MultMatrixTransposedVectorTemplate(matrix,vector,result): Size mismatch");

		result.SetNumberOfItems(matrix.NumberOfColumns());

		Real* mm = matrix.GetDataPointer();
		const Real* vv = vector.GetDataPointer();
		Index resultLength = result.NumberOfItems();
		Index vectorLength = vector.NumberOfItems();

		for (Index i = 0; i < resultLength; i++)
		{
			Real val = 0;
			Real* mr = &mm[i];
			for (Index j = 0; j < vectorLength; j++)
			{
				val += *mr * vv[j];
				mr += resultLength;
			}
			result[i] = val;
		}
	}

	//! implement the following functions as templates within namespace EXUmath::MultMatrixVector(...), ...
	//! ADD matrix.GetTranspose()*vector multiplication to given result vector (does not invoke memory allocation if result vector has appropriate size)
	//! result vector needs to have already appropriate size
	template<class TMatrix, class TVector, class TVectorResult>
	inline void MultMatrixTransposedVectorAddTemplate(const TMatrix& matrix, const TVector& vector, TVectorResult& result)
	{
		CHECKandTHROW(matrix.NumberOfRows() == vector.NumberOfItems(),
			"Hmath::MultMatrixTransposedVectorAddTemplate(matrix,vector,result): Size mismatch");

		CHECKandTHROW(matrix.NumberOfColumns() == result.NumberOfItems(),
			"Hmath::MultMatrixTransposedVectorAddTemplate(matrix,vector,result): Size mismatch");

		Real* mm = matrix.GetDataPointer();
		const Real* vv = vector.GetDataPointer();
		Index resultLength = result.NumberOfItems();
		Index vectorLength = vector.NumberOfItems();

		for (Index i = 0; i < resultLength; i++)
		{
			Real val = 0;
			Real* mr = &mm[i];
			for (Index j = 0; j < vectorLength; j++)
			{
				val += *mr * vv[j];
				mr += resultLength;
			}
			result[i] += val;
		}
	}

	//! generic matrix*matrix multiplication template
	template<class TMatrix1, class TMatrix2, class TMatrixResult>
	inline void MultMatrixMatrixTemplate(const TMatrix1& m1, const TMatrix2& m2, TMatrixResult& result)
	{
		CHECKandTHROW(m1.NumberOfColumns() == m2.NumberOfRows(),
			"MultMatrixMatrixTemplate(TMatrix1,TMatrix2,TMatrixResult): Size mismatch");

		result.SetNumberOfRowsAndColumns(m1.NumberOfRows(), m2.NumberOfColumns());

		for (Index i = 0; i < m2.NumberOfColumns(); i++)
		{
			for (Index j = 0; j < m1.NumberOfRows(); j++)
			{
				//auto value = (decltype(result(0, 0)))0.; //in this way, we can determine double or float! ==> TEST
				Real value = 0.; //in this way, we can determine double or float! ==> TEST
				for (Index k = 0; k < m1.NumberOfColumns(); k++)
				{
					value += m1(j, k)*m2(k, i);
				}
				result(j, i) = value;
			}
		}
	}

	//! generic transposed(matrix)*matrix multiplication template
	template<class TMatrix1, class TMatrix2, class TMatrixResult>
	inline void MultMatrixTransposedMatrixTemplate(const TMatrix1& m1, const TMatrix2& m2, TMatrixResult& result)
	{
		CHECKandTHROW(m1.NumberOfRows() == m2.NumberOfRows(),
			"MultMatrixTransposedMatrixTemplate(TMatrix1,TMatrix2,TMatrixResult): Size mismatch");

		result.SetNumberOfRowsAndColumns(m1.NumberOfColumns(), m2.NumberOfColumns());

		for (Index i = 0; i < m2.NumberOfColumns(); i++)
		{
			for (Index j = 0; j < m1.NumberOfColumns(); j++)
			{
				//auto value = (decltype(result(0, 0)))0.; //in this way, we can determine double or float! ==> TEST
				Real value = 0.; //in this way, we can determine double or float! ==> TEST
				for (Index k = 0; k < m1.NumberOfRows(); k++)
				{
					value += m1(k, j)*m2(k, i);
				}
				result(j, i) = value;
			}
		}
	}

	void MatrixTests();
} //namespace EXUmath
