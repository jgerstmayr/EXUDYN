/** ***********************************************************************************************
* @class		SlimMatrixBase
* @brief		A matrix of fixed rows and columns, known at compile time, with its data in the object (#3456)
* @details		Details:
                    - the size is a template parameter: no size members, no hidden 0 x 0 state, the loops have
                      compile-time trip counts, which the compiler unrolls and keeps in registers;
                    - row-major as ConstSizeMatrix; every sum runs in the order of the same operation of
                      ConstSizeMatrix, so that a matrix switched from ConstSizeMatrix gives the same bits;
                    - converts to and from ConstSizeMatrixBase<T, rows*columns> (the sizes checked in the regular
                      module), so that both classes can be used side by side;
                    - SlimMatrix<rows, columns> for Real, SlimMatrixF<rows, columns> for float;
                    - use ConstSizeMatrix where the size is only known at run time
                      (docs/dev/design/linalgFixedSize.md)
*
* @author		Gerstmayr Johannes
* @date			2026-11-12 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
* *** Example code ***
*
* @code{.cpp}
* SlimMatrix<3, 3> A(3, 3, { 1., 0., 0.,  0., 0., -1.,  0., 1., 0. });
* SlimMatrix<3, 4> G;				//data not initialized
* G.SetAll(0.);
* SlimMatrix<3, 4> AG = A * G;		//3x3 * 3x4, sizes checked by the compiler
* Vector3D v = A * Vector3D({ 1., 2., 3. });
* ConstSizeMatrix<9> B = A;			//conversion to ConstSizeMatrix
* @endcode
************************************************************************************************ */
#ifndef SLIMMATRIX__H
#define SLIMMATRIX__H

#include <initializer_list>
#include <ostream>
#include <array>
#include <type_traits>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Linalg/SlimVector.h"
#include "Linalg/ConstSizeMatrix.h"

extern bool linalgPrintUsePythonFormat; //!< true: use python format for output of vectors and matrices; false: use matlab format

template <typename T, Index rows, Index columns>
class SlimMatrixBase
{
	static_assert(rows > 0 && columns > 0, "SlimMatrixBase: rows and columns must be positive");
private:
	T data[rows * columns];

public:
	//Constructors, Destructor

	//! a rows x columns matrix; the data is not initialized
	SlimMatrixBase() = default;
	SlimMatrixBase(SlimMatrixBase const& other) = default;
	SlimMatrixBase& operator=(SlimMatrixBase const& other) = default;
	SlimMatrixBase(SlimMatrixBase&& other) = default;
	SlimMatrixBase& operator=(SlimMatrixBase&& other) = default;
	~SlimMatrixBase() = default;

	//! as ConstSizeMatrixBase(bool): the data is not initialized
	explicit SlimMatrixBase(bool) {}

	//! as ConstSizeMatrixBase(numberOfRows, numberOfColumns): the sizes must be rows and columns; data not initialized
	SlimMatrixBase(Index numberOfRowsInit, Index numberOfColumnsInit)
	{
		CHECKandTHROW(numberOfRowsInit == rows && numberOfColumnsInit == columns, "SlimMatrixBase(Index, Index): the size differs from the size of the type");
	}

	//! as ConstSizeMatrixBase(numberOfRows, numberOfColumns, value): every item set to initializationValue
	SlimMatrixBase(Index numberOfRowsInit, Index numberOfColumnsInit, T initializationValue)
	{
		CHECKandTHROW(numberOfRowsInit == rows && numberOfColumnsInit == columns, "SlimMatrixBase(Index, Index, T): the size differs from the size of the type");
		SetAll(initializationValue);
	}

	//! the items row by row from an initializer list
	SlimMatrixBase(Index numberOfRowsInit, Index numberOfColumnsInit, std::initializer_list<T> listOfItems)
	{
		CHECKandTHROW(numberOfRowsInit == rows && numberOfColumnsInit == columns && (Index)listOfItems.size() == rows * columns,
			"SlimMatrixBase(Index, Index, initializer_list): the size differs from the size of the type");
		Index cnt = 0;
		for (auto value : listOfItems) { data[cnt++] = value; }
	}

	//! from a ConstSizeMatrix of the same capacity, implicitly; the shape is checked in the regular module
	SlimMatrixBase(const ConstSizeMatrixBase<T, rows * columns>& matrix)
	{
		CopyFromConstSizeMatrix(matrix);
	}

	//! from a ConstSizeMatrix of larger capacity, e.g. a G matrix of maxRotationCoordinates*3, only explicitly, so that
	//! overloads for ConstSizeMatrix and SlimMatrix stay unambiguous
	template<Index dataSize, typename std::enable_if<(dataSize > rows * columns), int>::type = 0>
	explicit SlimMatrixBase(const ConstSizeMatrixBase<T, dataSize>& matrix)
	{
		CopyFromConstSizeMatrix(matrix);
	}

	template<Index dataSize>
	void CopyFromConstSizeMatrix(const ConstSizeMatrixBase<T, dataSize>& matrix)
	{
		static_assert(dataSize >= rows * columns, "SlimMatrixBase(ConstSizeMatrixBase): its dataSize is smaller than the matrix");
		CHECKandTHROW(matrix.NumberOfRows() == rows && matrix.NumberOfColumns() == columns, "SlimMatrixBase(ConstSizeMatrixBase): the size differs");
		const T* source = matrix.GetDataPointer();
		for (Index i = 0; i < rows * columns; i++) { data[i] = source[i]; }
	}

	//! from std::array, rows of columns, as the Python interface gives it
	SlimMatrixBase(const std::array<std::array<T, columns>, rows>& matrix)
	{
		for (Index i = 0; i < rows; i++)
		{
			for (Index j = 0; j < columns; j++) { data[i * columns + j] = matrix[i][j]; }
		}
	}

	//! to std::array, rows of columns, as the Python interface takes it
	operator std::array<std::array<T, columns>, rows>() const
	{
		std::array<std::array<T, columns>, rows> matrix;
		for (Index i = 0; i < rows; i++)
		{
			for (Index j = 0; j < columns; j++) { matrix[i][j] = data[i * columns + j]; }
		}
		return matrix;
	}

	//! to a ConstSizeMatrix of the same capacity
	operator ConstSizeMatrixBase<T, rows * columns>() const
	{
		ConstSizeMatrixBase<T, rows * columns> result(rows, columns);
		T* target = result.GetDataPointer();
		for (Index i = 0; i < rows * columns; i++) { target[i] = data[i]; }
		return result;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// BASIC FUNCTIONS
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	static constexpr Index NumberOfRows() { return rows; }
	static constexpr Index NumberOfColumns() { return columns; }
	static constexpr Index NumberOfItems() { return rows * columns; }
	static constexpr Index MaxDataSize() { return rows * columns; }
	static constexpr bool IsSquare() { return rows == columns; }

	const T* begin() const { return &data[0]; }
	T* begin() { return &data[0]; }
	const T* end() const { return &data[rows * columns]; }
	T* end() { return &data[rows * columns]; }
	const T* GetDataPointer() const { return &data[0]; }
	T* GetDataPointer() { return &data[0]; }

	//! as ConstSizeMatrix: only checks that the size is the one of the type
	void SetNumberOfRowsAndColumns(Index numberOfRowsInit, Index numberOfColumnsInit)
	{
		CHECKandTHROW(numberOfRowsInit == rows && numberOfColumnsInit == columns, "SlimMatrixBase::SetNumberOfRowsAndColumns: the size differs from the size of the type");
	}

	void SetAll(T value)
	{
		for (Index i = 0; i < rows * columns; i++) { data[i] = value; }
	}

	//! the items row by row from an initializer list
	void SetMatrix(Index numberOfRowsInit, Index numberOfColumnsInit, std::initializer_list<T> listOfItems)
	{
		CHECKandTHROW(numberOfRowsInit == rows && numberOfColumnsInit == columns && (Index)listOfItems.size() == rows * columns,
			"SlimMatrixBase::SetMatrix: the size differs from the size of the type");
		Index cnt = 0;
		for (auto value : listOfItems) { data[cnt++] = value; }
	}

	//! value on the diagonal, 0 elsewhere; square matrices
	void SetScalarMatrix(Index rowsColumns, T value)
	{
		static_assert(rows == columns, "SlimMatrixBase::SetScalarMatrix: only for square matrices");
		CHECKandTHROW(rowsColumns == rows, "SlimMatrixBase::SetScalarMatrix: the size differs from the size of the type");
		for (Index i = 0; i < rows * columns; i++) { data[i] = 0; }
		for (Index j = 0; j < rows; j++) { data[j * columns + j] = value; }
	}

	//! the skew matrix of v, such that SkewMatrix(v) * w = v x w
	void SetSkewMatrix(const SlimVectorBase<T, 3>& v)
	{
		static_assert(rows == 3 && columns == 3, "SlimMatrixBase::SetSkewMatrix: only for 3x3 matrices");
		data[0] = 0.;    data[1] = -v[2]; data[2] = v[1];
		data[3] = v[2];  data[4] = 0;     data[5] = -v[0];
		data[6] = -v[1]; data[7] = v[0];  data[8] = 0;
	}

	//! vectorA * vectorB^T
	template<class TVectorA, class TVectorB>
	void SetWithDiadicProduct(const TVectorA& vectorA, const TVectorB& vectorB)
	{
		CHECKandTHROW(vectorA.NumberOfItems() == rows && vectorB.NumberOfItems() == columns, "SlimMatrixBase::SetWithDiadicProduct: the sizes differ");
		for (Index i = 0; i < rows; i++)
		{
			for (Index j = 0; j < columns; j++) { data[i * columns + j] = vectorA.GetUnsafe(i) * vectorB.GetUnsafe(j); }
		}
	}

	//! copy matrix sm into *this at row, column
	template <class TMatrix>
	void SetSubmatrix(const TMatrix& sm, Index row = 0, Index column = 0)
	{
		CHECKandTHROW(row + sm.NumberOfRows() <= rows && column + sm.NumberOfColumns() <= columns, "SlimMatrixBase::SetSubmatrix: size mismatch");
		for (Index i = 0; i < sm.NumberOfRows(); i++)
		{
			for (Index j = 0; j < sm.NumberOfColumns(); j++) { data[(i + row) * columns + column + j] = sm.GetUnsafe(i, j); }
		}
	}

	//! copy any matrix of the same size
	template<class TMatrix>
	void CopyFrom(const TMatrix& matrix)
	{
		CHECKandTHROW(matrix.NumberOfRows() == rows && matrix.NumberOfColumns() == columns, "SlimMatrixBase::CopyFrom: the size differs");
		for (Index i = 0; i < rows; i++)
		{
			for (Index j = 0; j < columns; j++) { data[i * columns + j] = (T)matrix.GetUnsafe(i, j); }
		}
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// ACCESS
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	T& operator()(Index row, Index column)
	{
		CHECKandTHROW(row < rows, "SlimMatrixBase::operator()(Index, Index): request of invalid row");
		CHECKandTHROW(column < columns, "SlimMatrixBase::operator()(Index, Index): request of invalid column");
		return data[row * columns + column];
	}

	const T& operator()(Index row, Index column) const
	{
		CHECKandTHROW(row < rows, "SlimMatrixBase::operator()(Index, Index) const: request of invalid row");
		CHECKandTHROW(column < columns, "SlimMatrixBase::operator()(Index, Index) const: request of invalid column");
		return data[row * columns + column];
	}

	//! pointer to the row; [row][column] style, the column unchecked
	T* operator[](Index row)
	{
		CHECKandTHROW(row < rows, "SlimMatrixBase::operator[](Index): request of invalid row");
		return &data[row * columns];
	}

	const T* operator[](Index row) const
	{
		CHECKandTHROW(row < rows, "SlimMatrixBase::operator[](Index) const: request of invalid row");
		return &data[row * columns];
	}

	T& GetItem(Index row, Index column) { return (*this)(row, column); }
	const T& GetItem(Index row, Index column) const { return (*this)(row, column); }
	T& GetUnsafe(Index row, Index column) { return data[row * columns + column]; }
	const T& GetUnsafe(Index row, Index column) const { return data[row * columns + column]; }

	//! the item with the row-major index
	T& GetItem(Index index)
	{
		CHECKandTHROW(index < rows * columns, "SlimMatrixBase::GetItem(Index): invalid index");
		return data[index];
	}

	const T& GetItem(Index index) const
	{
		CHECKandTHROW(index < rows * columns, "SlimMatrixBase::GetItem(Index) const: invalid index");
		return data[index];
	}

	T& GetItemUnsafe(Index index) { return data[index]; }
	const T& GetItemUnsafe(Index index) const { return data[index]; }

	template<Index columnSize = rows>
	SlimVectorBase<T, columnSize> GetColumnVector(Index column) const
	{
		static_assert(columnSize == rows, "SlimMatrixBase::GetColumnVector: the size of the vector differs from the rows");
		CHECKandTHROW(column < columns, "SlimMatrixBase::GetColumnVector: illegal column");
		SlimVectorBase<T, rows> result;
		for (Index i = 0; i < rows; i++) { result[i] = data[i * columns + column]; }
		return result;
	}

	template<Index columnSize = rows>
	void SetColumnVector(Index column, const SlimVectorBase<T, columnSize>& columnVector)
	{
		static_assert(columnSize == rows, "SlimMatrixBase::SetColumnVector: the size of the vector differs from the rows");
		CHECKandTHROW(column < columns, "SlimMatrixBase::SetColumnVector: illegal column");
		for (Index i = 0; i < rows; i++) { data[i * columns + column] = columnVector[i]; }
	}

	template<Index rowSize = columns>
	SlimVectorBase<T, rowSize> GetRowVector(Index row) const
	{
		static_assert(rowSize == columns, "SlimMatrixBase::GetRowVector: the size of the vector differs from the columns");
		CHECKandTHROW(row < rows, "SlimMatrixBase::GetRowVector: illegal row");
		SlimVectorBase<T, columns> result;
		for (Index i = 0; i < columns; i++) { result[i] = data[row * columns + i]; }
		return result;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// OPERATORS
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! true if all items are equal
	bool operator==(const SlimMatrixBase& matrix) const
	{
		for (Index i = 0; i < rows * columns; i++) { if (data[i] != matrix.data[i]) { return false; } }
		return true;
	}

	SlimMatrixBase& operator+=(const SlimMatrixBase& matrix)
	{
		for (Index i = 0; i < rows * columns; i++) { data[i] += matrix.data[i]; }
		return *this;
	}

	SlimMatrixBase& operator-=(const SlimMatrixBase& matrix)
	{
		for (Index i = 0; i < rows * columns; i++) { data[i] -= matrix.data[i]; }
		return *this;
	}

	SlimMatrixBase& operator*=(T scalar)
	{
		for (Index i = 0; i < rows * columns; i++) { data[i] *= scalar; }
		return *this;
	}

	//! no check against division by zero
	SlimMatrixBase& operator/=(T scalar)
	{
		for (Index i = 0; i < rows * columns; i++) { data[i] /= scalar; }
		return *this;
	}

	friend SlimMatrixBase operator+(const SlimMatrixBase& m1, const SlimMatrixBase& m2)
	{
		SlimMatrixBase result;
		for (Index i = 0; i < rows * columns; i++) { result.data[i] = m1.data[i] + m2.data[i]; }
		return result;
	}

	friend SlimMatrixBase operator-(const SlimMatrixBase& m1, const SlimMatrixBase& m2)
	{
		SlimMatrixBase result;
		for (Index i = 0; i < rows * columns; i++) { result.data[i] = m1.data[i] - m2.data[i]; }
		return result;
	}

	friend SlimMatrixBase operator*(const SlimMatrixBase& matrix, const T& value)
	{
		SlimMatrixBase result;
		for (Index i = 0; i < rows * columns; i++) { result.data[i] = matrix.data[i] * value; }
		return result;
	}

	friend SlimMatrixBase operator*(const T& value, const SlimMatrixBase& matrix)
	{
		SlimMatrixBase result;
		for (Index i = 0; i < rows * columns; i++) { result.data[i] = matrix.data[i] * value; }
		return result;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// EXTENDED FUNCTIONS
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	SlimMatrixBase<T, columns, rows> GetTransposed() const
	{
		SlimMatrixBase<T, columns, rows> result;
		for (Index i = 0; i < rows; i++)
		{
			for (Index j = 0; j < columns; j++) { result.GetUnsafe(j, i) = data[i * columns + j]; }
		}
		return result;
	}

	//! transposes *this; square matrices
	void TransposeYourself()
	{
		static_assert(rows == columns, "SlimMatrixBase::TransposeYourself: only for square matrices");
		for (Index i = 0; i < rows; i++)
		{
			for (Index j = 0; j < i; j++) { EXUstd::Swap(data[i * columns + j], data[j * columns + i]); }
		}
	}

	T Trace() const
	{
		static_assert(rows == columns, "SlimMatrixBase::Trace: only for square matrices");
		T trace = 0.;
		for (Index i = 0; i < rows; i++) { trace += data[i * columns + i]; }
		return trace;
	}

	//! the determinant of matrices up to 3x3, as ConstSizeMatrixBase::GetDeterminant
	T GetDeterminant() const
	{
		static_assert(rows == columns && rows <= 3, "SlimMatrixBase::GetDeterminant: only for square matrices up to 3x3");
		const SlimMatrixBase& m = *this;
		if constexpr (rows == 1) { return m(0, 0); }
		else if constexpr (rows == 2) { return m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0); }
		else
		{
			return m(0, 0) * (m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2)) -
				m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
				m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
		}
	}

	//! the inverse, as ConstSizeMatrixBase::GetInverse: explicit up to 3x3, Gaussian elimination above
	SlimMatrixBase GetInverse() const
	{
		static_assert(rows == columns, "SlimMatrixBase::GetInverse: only for square matrices");
		const SlimMatrixBase& m = *this;
		SlimMatrixBase result;
		if constexpr (rows == 1)
		{
			CHECKandTHROW(m(0, 0) != 0, "Matrix1D::GetInverse: matrix is singular");
			result(0, 0) = (T)1. / m(0, 0);
		}
		else if constexpr (rows == 2)
		{
			T det = (m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0));
			CHECKandTHROW(det != 0, "Matrix2D::GetInverse: matrix is singular");
			T invdet = (T)1 / det;
			result(0, 0) = invdet * m(1, 1);
			result(0, 1) = -invdet * m(0, 1);
			result(1, 0) = -invdet * m(1, 0);
			result(1, 1) = invdet * m(0, 0);
		}
		else if constexpr (rows == 3)
		{
			T det = m(0, 0) * (m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2)) - m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) + m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
			CHECKandTHROW(det != 0, "Matrix3D::GetInverse: matrix is singular");
			T invdet = (T)1 / det;
			result(0, 0) = invdet * (m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2));
			result(0, 1) = invdet * (m(0, 2) * m(2, 1) - m(0, 1) * m(2, 2));
			result(0, 2) = invdet * (m(0, 1) * m(1, 2) - m(0, 2) * m(1, 1));
			result(1, 0) = invdet * (m(1, 2) * m(2, 0) - m(1, 0) * m(2, 2));
			result(1, 1) = invdet * (m(0, 0) * m(2, 2) - m(0, 2) * m(2, 0));
			result(1, 2) = invdet * (m(1, 0) * m(0, 2) - m(0, 0) * m(1, 2));
			result(2, 0) = invdet * (m(1, 0) * m(2, 1) - m(2, 0) * m(1, 1));
			result(2, 1) = invdet * (m(2, 0) * m(0, 1) - m(0, 0) * m(2, 1));
			result(2, 2) = invdet * (m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1));
		}
		else
		{
			ConstSizeMatrixBase<T, rows * columns> copyMatrix = *this;
			ConstSizeMatrixBase<T, rows * columns> inverse;
			bool success = copyMatrix.ComputeInverse(inverse);
			if (!success) { CHECKandTHROWstring("SlimMatrixBase::GetInverse: matrix is singular"); }
			result = inverse;
		}
		return result;
	}

	//! as ConstSizeMatrix: "[a b; c d]", or "[[a,b],[c,d]]" with linalgPrintUsePythonFormat
	friend std::ostream& operator<<(std::ostream& os, const SlimMatrixBase& matrix)
	{
		os << ConstSizeMatrixBase<T, rows * columns>(matrix);
		return os;
	}
};

//! m1 * m2, the sizes checked by the compiler; each sum over k in increasing order, as ConstSizeMatrix
template<typename T, Index rows, Index inner, Index columns>
inline SlimMatrixBase<T, rows, columns> operator*(const SlimMatrixBase<T, rows, inner>& m1, const SlimMatrixBase<T, inner, columns>& m2)
{
	SlimMatrixBase<T, rows, columns> result;
	const T* a = m1.GetDataPointer();
	const T* b = m2.GetDataPointer();
	T* r = result.GetDataPointer();
	for (Index i = 0; i < rows; i++)
	{
		for (Index j = 0; j < columns; j++)
		{
			T value = 0;
			for (Index k = 0; k < inner; k++) { value += a[i * inner + k] * b[k * columns + j]; }
			r[i * columns + j] = value;
		}
	}
	return result;
}

//! m1^T * m2 without forming the transposed; the same bits as m1.GetTransposed() * m2
template<typename T, Index inner, Index rows, Index columns>
inline SlimMatrixBase<T, rows, columns> MultTransposedMatrix(const SlimMatrixBase<T, inner, rows>& m1, const SlimMatrixBase<T, inner, columns>& m2)
{
	SlimMatrixBase<T, rows, columns> result;
	const T* a = m1.GetDataPointer();
	const T* b = m2.GetDataPointer();
	T* r = result.GetDataPointer();
	for (Index i = 0; i < rows; i++)
	{
		for (Index j = 0; j < columns; j++)
		{
			T value = 0;
			for (Index k = 0; k < inner; k++) { value += a[k * rows + i] * b[k * columns + j]; }
			r[i * columns + j] = value;
		}
	}
	return result;
}

//! matrix * vector; the vector may be of another type T2 (e.g. automatic differentiation), as for ConstSizeMatrix
template<typename T, typename T2, Index rows, Index columns>
inline SlimVectorBase<T, rows> operator*(const SlimMatrixBase<T, rows, columns>& matrix, const SlimVectorBase<T2, columns>& vector)
{
	SlimVectorBase<T, rows> result;
	const T* m = matrix.GetDataPointer();
	const T2* v = vector.GetDataPointer();
	T* r = result.GetDataPointer();
	for (Index i = 0; i < rows; i++)
	{
		T value = 0;
		for (Index j = 0; j < columns; j++) { value += m[i * columns + j] * v[j]; }
		r[i] = value;
	}
	return result;
}

//! vector^T * matrix, returned as a vector
template<typename T, typename T2, Index rows, Index columns>
inline SlimVectorBase<T, columns> operator*(const SlimVectorBase<T2, rows>& vector, const SlimMatrixBase<T, rows, columns>& matrix)
{
	SlimVectorBase<T, columns> result;
	const T* m = matrix.GetDataPointer();
	const T2* v = vector.GetDataPointer();
	T* r = result.GetDataPointer();
	for (Index i = 0; i < columns; i++)
	{
		T value = 0;
		for (Index j = 0; j < rows; j++) { value += v[j] * m[j * columns + i]; }
		r[i] = value;
	}
	return result;
}

//! matrix^T * vector without forming the transposed; the same bits as vector * matrix
template<typename T, typename T2, Index rows, Index columns>
inline SlimVectorBase<T, columns> MultTransposedVector(const SlimMatrixBase<T, rows, columns>& matrix, const SlimVectorBase<T2, rows>& vector)
{
	return vector * matrix;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Transposed(A) * x and Transposed(A) * B without forming the transposed (#3479): Transposed() returns a view, which
//only operator* takes, in the same statement; no other operator accepts it, so a misuse does not compile. The products
//are MultTransposedVector and MultTransposedMatrix, the same bits as A.GetTransposed() * x.

//! a view of the transposed of the matrix of an lvalue; it refers to the matrix
template<class TMatrix>
class TransposedView
{
public:
	explicit TransposedView(const TMatrix& matrixInit) : matrix(matrixInit) {}
	const TMatrix& GetMatrix() const { return matrix; }
private:
	const TMatrix& matrix;
};

//! the transposed of the matrix of an rvalue, e.g. Transposed(node->GetRotationMatrix()): it keeps the (not transposed)
//! matrix, so that the view cannot refer to a temporary that is gone
template<class TMatrix>
class TransposedCopy
{
public:
	explicit TransposedCopy(TMatrix&& matrixInit) : matrix(std::move(matrixInit)) {}
	const TMatrix& GetMatrix() const { return matrix; }
private:
	TMatrix matrix;
};

//! Transposed(A) of an lvalue A: a view, no copy
template<class TMatrix>
inline TransposedView<TMatrix> Transposed(const TMatrix& matrix) { return TransposedView<TMatrix>(matrix); }

//! Transposed(A) of an rvalue A: the matrix is kept
template<class TMatrix, typename std::enable_if<!std::is_lvalue_reference<TMatrix>::value, int>::type = 0>
inline TransposedCopy<TMatrix> Transposed(TMatrix&& matrix) { return TransposedCopy<TMatrix>(std::move(matrix)); }

//! A^T * v
template<typename T, typename T2, Index rows, Index columns>
inline SlimVectorBase<T, columns> operator*(const TransposedView<SlimMatrixBase<T, rows, columns>>& transposed, const SlimVectorBase<T2, rows>& vector)
{
	return MultTransposedVector(transposed.GetMatrix(), vector);
}
template<typename T, typename T2, Index rows, Index columns>
inline SlimVectorBase<T, columns> operator*(const TransposedCopy<SlimMatrixBase<T, rows, columns>>& transposed, const SlimVectorBase<T2, rows>& vector)
{
	return MultTransposedVector(transposed.GetMatrix(), vector);
}

//! A^T * B
template<typename T, Index inner, Index rows, Index columns>
inline SlimMatrixBase<T, rows, columns> operator*(const TransposedView<SlimMatrixBase<T, inner, rows>>& transposed, const SlimMatrixBase<T, inner, columns>& m2)
{
	return MultTransposedMatrix(transposed.GetMatrix(), m2);
}
template<typename T, Index inner, Index rows, Index columns>
inline SlimMatrixBase<T, rows, columns> operator*(const TransposedCopy<SlimMatrixBase<T, inner, rows>>& transposed, const SlimMatrixBase<T, inner, columns>& m2)
{
	return MultTransposedMatrix(transposed.GetMatrix(), m2);
}

template<Index rows, Index columns>
using SlimMatrix = SlimMatrixBase<Real, rows, columns>;

template<Index rows, Index columns>
using SlimMatrixF = SlimMatrixBase<float, rows, columns>;

#endif
