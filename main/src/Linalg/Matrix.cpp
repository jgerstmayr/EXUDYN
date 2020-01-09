/** ***********************************************************************************************
* @brief		Implementation for class Matrix
* @details		Details:
*
* @author		Gerstmayr Johannes
* @date			1997-05-15 (generated)
* @date			2019-05-17 (last modified)
* @pre			Indizes of []-operator run from 0 to dataSize-1;
* 				Use Matrix for large matrix sizes; uses dynamic allocation (slow)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
* 				
*
************************************************************************************************ */

//#include <initializer_list> //for initializer_list in constructor
//#include <ostream>          //ostream for matrix output as text
//#include <iostream>         //! for cout @todo remove cout from class matrix ==> add Error handling
//
//#include "Utilities/ReleaseAssert.h"
//#include "Utilities/BasicDefinitions.h"
//#include "Utilities/BasicFunctions.h"
//#include "Linalg/ConstSizeVector.h"
//#include "Linalg/ResizableVector.h"
//#include "Linalg/Matrix.h"
//#include "Linalg/ResizableMatrix.h"

#include "Linalg/BasicLinalg.h" //for Resizable Vector

//template<typename T>
//inline MatrixBase<T>::~MatrixBase()
//{
//	if (data != nullptr) //not necessary
//	{
//		delete[] data;
//		data = nullptr;     //do not set to zero==>causes runtime error to detect multiple deletes!
//#ifdef __EXUDYN_RUNTIME_CHECKS__
//		matrix_delete_counts++;
//#endif
//	}
//};


////! allocate memory if numberOfRealsInit!=0; set data to allocated array of Reals or to nullptr; return false if failed
//template<typename T>
//bool MatrixBase<T>::AllocateMemory(Index numberOfRowsInit, Index numberOfColumnsInit)
//{
//	if (numberOfRowsInit*numberOfColumnsInit == 0) { data = nullptr; }
//	else
//	{
//		try {
//			data = new T[numberOfRowsInit*numberOfColumnsInit];
//		}
//		catch (const std::bad_alloc& e) {
//			std::cout << "Allocation failed: " << e.what() << '\n';
//			std::cout << "requested memory = " << (8. * numberOfRowsInit*numberOfColumnsInit) / pow(2, 20) << " MB, rows = " << numberOfRowsInit << ", columns = " << numberOfColumnsInit << "\n";
//			release_assert(0 && "MatrixBase<T>::AllocateMemory");
//			return false; //no success
//		}
//#ifdef __EXUDYN_RUNTIME_CHECKS__
//		matrix_new_counts++; //only counted if try succeeded
//#endif
//	}
//	return true;
//}
//
////! free memory if data!=nullptr
//template<typename T>
//void MatrixBase<T>::FreeMemory()
//{
//	if (data != nullptr)
//	{
//		delete[] data;
//		data = nullptr;
//#ifdef __EXUDYN_RUNTIME_CHECKS__
//		matrix_delete_counts++;
//#endif
//	}
//}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// SOLVER
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! Compute matrix inverse (internal function, needs memory allocation and may be slower than external functions - Eigen, etc.)
//  NOT Threadsafe!
template<typename T>
bool MatrixBase<T>::Invert()
{
	if (numberOfRows*numberOfColumns == 0) return true; //no need to invert; but this is no error!

	release_assert(numberOfColumns == numberOfRows && data != NULL);

	static ResizableMatrix m; //memory allocation only once, if size does not change; not THREAD safe!

	//Insert identity-matrix on left-hand-side
	m.SetScalarMatrix(numberOfRows, 1.); //set unit matrix

	T mij;
	Index i, j, k;
	Index maxj = 0;

	// Solve lower triangular Matrix
	for (j = 0; j < numberOfRows; j++)
	{
		//pout << "j=" << j << ":\n";
		T pivot = fabs(GetItem(j, j));
		Index pivotpos = j;
		for (k = j + 1; k < numberOfRows; k++)
		{
			if (fabs(GetItem(k, j)) > pivot) { pivotpos = k; pivot = fabs(GetItem(k, j)); }
		}
		if (pivot == 0)
		{
			//SysError(STDstring("Matrix::Invert: problems with column ") + EXUstd::ToString(j) + "\n");
			return false;
		}
		//pout << "  pivot = " << pivotpos << "\n";

		maxj = EXUstd::Maximum(pivotpos, maxj);

		m.SwapRows(pivotpos, j);
		SwapRows(pivotpos, j);
		m.MultiplyRow(j, 1. / GetItem(j, j));
		MultiplyRow(j, 1. / GetItem(j, j));

		//pout << "  minv=" << m << "\n";
		//pout << "  m=   " << *this << "\n";

		for (i = j + 1; i < numberOfColumns; i++)
		{
			mij = GetItem(i, j);
			if (mij != 0.)
			{
				AddRowToRowWithFactor(j, i, -mij, j, numberOfColumns - 1); //j..numberOfRows
				m.AddRowToRowWithFactor(j, i, -mij, 0, maxj); //1..j
			}
		}
		//pout << "  minv=" << m << "\n";
		//pout << "  m=   " << *this << "\n";

	}

	//backsubstitution ==> for inverse, this takes most of the time
	for (j = numberOfRows - 1; j > 0; j--)
	{
		for (i = 0; i <= j - 1; i++)
		{
			mij = GetItem(i, j);
			if (mij != 0)
			{
				m.AddRowToRowWithFactor(j, i, -mij); //1..numberOfRows
			}
		}
	}

	CopyFrom(m); //now write temporary matrix into *this

	return true;
}

template<typename T>
bool MatrixBase<T>::Solve(const VectorBase<T>& rhs, VectorBase<T>& q)
{
	release_assert(0 && "Matrix::Solve needs to be tested!\n");
	return false;
	/*		release_assert(rhs.NumberOfItems() == numberOfRows && numberOfColumns == numberOfRows && numberOfColumns*numberOfRows != 0 && data != NULL);
			static ResizableMatrix m; //memory allocated only once; not THREAD safe!
			static ResizableVector f;

			m = *this;
			f.CopyFrom(rhs);

			if (f.NumberOfItems() == 1)
			{
				if (data[0] == 0) { return 0; }
				q[0] = f[0] / data[0];
				return true;
			}

			Index i, j, k;
			// Solve lower triangular matrix
			for (j = 0; j < numberOfColumns; j++)
			{
				double pivot = fabs(m(j, j));
				Index pivotpos = j;
				for (k = j; k < numberOfRows; k++)
				{
					if (fabs(m(k, j)) > pivot) { pivotpos = k; pivot = fabs(m(k, j)); }
				}
				if (pivot == 0)
				{
					//SysError(STDstring("Matrix::Solve: problems with column ") + EXUstd::ToString(j) + "\n");
					return false;
				}

				m.SwapRows(pivotpos, j);
				EXUstd::Swap(f[pivotpos], f[j]);

				f[j] *= 1 / m(j, j);		//must be place before MulRow(j,1/m(j,j))
				m.MultiplyRow(j, 1 / m(j, j));

				for (i = j; i < numberOfRows; i++)
				{
					if (m(i, j) != 0)
					{
						f[i] += -m(i, j)*f[j];
						m.AddRowToRowWithFactor(j, i, -m(i, j));
					}
				}
			}

			for (j = numberOfRows; j > 0; j--)
			{
				q[j-1] = f[j-1];
				double sum = 0;
				for (Index i = j-1 ; i < numberOfRows; i++)
				{
					sum += m(j-1, i)*q[i];
				}
				q[j-1] -= sum;
			}

			return true;
			*/
}

//! Returns the maximum-norm (largest absolute value in matrix)
template<typename T>
T MatrixBase<T>::MaxNorm() const
{
	T value = 0;

	for (auto item : *this) {
		if (fabs(item) > value) { value = fabs(item); };
	}

	return value;
}

//! perform general matrix inverse tests; put this into test suite lateron
namespace EXUmath {
	void MatrixTests()
	{
		Real maxError; //error per test
		Real sumError = 0.; //sum of errors of all tests

		Matrix m1(1, 1, { 7 });
		Matrix m1Inv(1, 1, { 0.1428571428571428 });
		m1.Invert();
		maxError = (m1 - m1Inv).MaxNorm(); sumError += maxError;
		if (maxError > 1e-14) { pout << "MatrixTest inverse failed for m1 \n"; }

		Matrix m2(2, 2, { 2,0,0,3 });
		Matrix m2Inv(2, 2, { 0.5,-0,-0,0.3333333333333333 });
		m2.Invert();
		maxError = (m2 - m2Inv).MaxNorm(); sumError += maxError;
		if (maxError > 1e-14) { pout << "MatrixTest inverse failed for m2 \n"; }

		Matrix m3(2, 2, { 0,2,3,0 });
		Matrix m3Inv(2, 2, { -0,0.3333333333333333,0.5,0 });
		m3.Invert();
		maxError = (m3 - m3Inv).MaxNorm(); sumError += maxError;
		if (maxError > 1e-14) { pout << "MatrixTest inverse failed for m3 \n"; }

		Matrix m4(2, 2, { 0,2,3,1 });
		Matrix m4Inv(2, 2, { -0.1666666666666667,0.3333333333333333,0.5,0 });
		m4.Invert();
		maxError = (m4 - m4Inv).MaxNorm(); sumError += maxError;
		if (maxError > 1e-14) { pout << "MatrixTest inverse failed for m4 \n"; }

		Matrix m5(2, 2, { 2,0,3,1 });
		Matrix m5Inv(2, 2, { 0.5,0,-1.5,1 });
		m5.Invert();
		maxError = (m5 - m5Inv).MaxNorm(); sumError += maxError;
		if (maxError > 1e-14) { pout << "MatrixTest inverse failed for m5 \n"; }

		Matrix m6(2, 2, { 2.1,1.1,3.4,0 });
		Matrix m6Inv(2, 2, { -0,0.2941176470588235,0.9090909090909091,-0.5614973262032086 });
		m6.Invert();
		maxError = (m6 - m6Inv).MaxNorm(); sumError += maxError;
		if (maxError > 1e-14) { pout << "MatrixTest inverse failed for m6 \n"; }

		Matrix m7(3, 3, { 2,1,0,3,1,0,1,2,3 });
		Matrix m7Inv(3, 3, { -0.9999999999999999,0.9999999999999999,5.551115123125781e-18,3,-2,3.33066907387547e-17,-1.666666666666667,0.9999999999999999,0.3333333333333333 });
		m7.Invert();
		maxError = (m7 - m7Inv).MaxNorm(); sumError += maxError;
		if (maxError > 1e-14) { pout << "MatrixTest inverse failed for m7 \n"; }

		Matrix m8(3, 3, { 1,2,0,3,0,0,1,40,3 });
		Matrix m8Inv(3, 3, { 0,0.3333333333333333,-0,0.5,-0.1666666666666667,0,-6.666666666666666,2.111111111111111,0.3333333333333333 });
		m8.Invert();
		maxError = (m8 - m8Inv).MaxNorm(); sumError += maxError;
		if (maxError > 1e-14) { pout << "MatrixTest inverse failed for m8 \n"; }

		Matrix m9(3, 3, { 0,5,0,3,1,0,1,2,3 });
		Matrix m9Inv(3, 3, { -0.06666666666666667,0.3333333333333333,-0,0.2,0,-0,-0.1111111111111111,-0.1111111111111111,0.3333333333333333 });
		m9.Invert();
		maxError = (m9 - m9Inv).MaxNorm(); sumError += maxError;
		if (maxError > 1e-14) { pout << "MatrixTest inverse failed for m9 \n"; }

		Matrix m10(3, 3, { 0,0,7,3,1,0,1,2,3 });
		Matrix m10Inv(3, 3, { 0.0857142857142857,0.4,-0.2,-0.2571428571428571,-0.2,0.6,0.1428571428571428,0,0 });
		m10.Invert();
		maxError = (m10 - m10Inv).MaxNorm(); sumError += maxError;
		if (maxError > 1e-14) { pout << "MatrixTest inverse failed for m10 \n"; }

		Matrix m11(3, 3, { 1,2,4,0,0,7,2,1,3 });
		Matrix m11Inv(3, 3, { -0.3333333333333333,-0.09523809523809525,0.6666666666666666,0.6666666666666666,-0.2380952380952381,-0.3333333333333333,0,0.1428571428571428,0 });
		m11.Invert();
		maxError = (m11 - m11Inv).MaxNorm(); sumError += maxError;
		if (maxError > 1e-14) { pout << "MatrixTest inverse failed for m11 \n"; }

		Matrix m12(4, 4, { 1,2,4,7,4,0,7,9,2,2,1,3,7,8,1,1 });
		Matrix m12Inv(4, 4, { -0.3703703703703703,0.1555555555555556,0.3999999999999999,-0.00740740740740739,0.2962962962962963,-0.1444444444444445,-0.3,0.1259259259259259,0.2592592592592593,0.1111111111111111,-0.9999999999999999,0.1851851851851851,-0.03703703703703706,-0.04444444444444443,0.6,-0.1407407407407407 });
		m12.Invert();
		maxError = (m12 - m12Inv).MaxNorm(); sumError += maxError;
		if (maxError > 1e-14) { pout << "MatrixTest inverse failed for m12 \n"; }

		pout << "Matrix tests error=" << sumError << "\n\n";
	}
} //namespace EXUmath

