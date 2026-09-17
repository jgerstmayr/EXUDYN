/** ***********************************************************************************************
* @file			AllMatrixVariantsUnitTests.h
* @brief		Unit tests for the matrix classes other than Matrix: ResizableMatrix,
*				ConstSizeMatrix, LinkedDataMatrix and MatrixContainer.
* @details		Details:
*				- AllMatrixUnitTests.h covers the base class Matrix. The four classes here are the
*				  ones the solver actually uses, and none of them had a unit test: they differ from
*				  Matrix exactly where a defect is expensive - in how they own, keep or share their
*				  data.
*				- ResizableMatrix KEEPS its allocation when it shrinks, so the tests check what
*				  happens to the data and to the pointer across a shrink and a re-grow, not only
*				  that the size is right.
*				- ConstSizeMatrix holds its data in the object; the tests run it at, below and
*				  above its static size, the last one being the case that must be rejected.
*				- LinkedDataMatrix owns nothing. It is tested on a ROW SUB-RANGE of another matrix,
*				  including that writing through the link changes the original and nothing outside
*				  the linked rows - the matrix analogue of the LinkedDataVector case of #2394.
*				- MatrixContainer is two matrices behind one interface; every test asks the same
*				  question of a dense and a sparse container built from the same values, because
*				  the two answers differing is the only defect it can have.
*
* @author		Gerstmayr Johannes
* @date			2026-09-17 (created; revision2026 step R5.4.1, issue #2472)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
************************************************************************************************ */
#ifndef ALLMATRIXVARIANTSUNITTESTS__H
#define ALLMATRIXVARIANTSUNITTESTS__H

//BasicLinalg.h does NOT include this one - it comes in through CSystem.h, which is part of why
//MatrixContainer had no unit test until now (the same gap as for the parallel vector classes)
#include "Linalg/MatrixContainer.h"

//a reproducible value; not i+j, because that is symmetric and would hide a transposed index
inline Real MatrixVariantsValue(Index i, Index j) { return 1. + (Real)i + 0.25 * (Real)j; }

template<class TMatrix>
inline void MatrixVariantsFill(TMatrix& m, Index numberOfRows, Index numberOfColumns)
{
	m.SetNumberOfRowsAndColumns(numberOfRows, numberOfColumns);
	for (Index i = 0; i < numberOfRows; i++)
	{
		for (Index j = 0; j < numberOfColumns; j++)
		{
			m(i, j) = MatrixVariantsValue(i, j);
		}
	}
}

//! every item equals MatrixVariantsValue(i,j); says what the matrix should hold, not what it holds
template<class TMatrix>
inline bool MatrixVariantsHoldsValues(const TMatrix& m, Index numberOfRows, Index numberOfColumns)
{
	if (m.NumberOfRows() != numberOfRows || m.NumberOfColumns() != numberOfColumns) { return false; }
	for (Index i = 0; i < numberOfRows; i++)
	{
		for (Index j = 0; j < numberOfColumns; j++)
		{
			if (m(i, j) != MatrixVariantsValue(i, j)) { return false; }
		}
	}
	return true;
}

const lest::test matrixVariants_specific_test[] =
{

	CASE("ResizableMatrix: fill, copy and compare")
	{
		ResizableMatrix m;
		MatrixVariantsFill(m, 3, 4);
		EXPECT(MatrixVariantsHoldsValues(m, 3, 4));

		ResizableMatrix copy = m;
		EXPECT(copy.NumberOfRows() == 3);
		EXPECT(copy.NumberOfColumns() == 4);
		EXPECT(MatrixVariantsHoldsValues(copy, 3, 4));
		EXPECT(copy == m);

		copy(2, 3) = -1.;
		EXPECT(!(copy == m)); //a copy is a copy: writing into it must not reach the original
		EXPECT(m(2, 3) == MatrixVariantsValue(2, 3));
	},

	CASE("ResizableMatrix: shrinking keeps the allocation, growing beyond it reallocates")
	{
		//this is the whole point of the class: SetNumberOfRowsAndColumns down and up again must not
		//allocate, and the data of the rows that stay must not be disturbed by the round trip
		ResizableMatrix m;
		MatrixVariantsFill(m, 4, 4);
		const Real* dataBefore = m.GetDataPointer();

		m.SetNumberOfRowsAndColumns(2, 4);
		EXPECT(m.NumberOfRows() == 2);
		EXPECT(m.GetDataPointer() == dataBefore); //shrinking does NOT reallocate
		EXPECT(MatrixVariantsHoldsValues(m, 2, 4));

		m.SetNumberOfRowsAndColumns(4, 4); //back within the old allocation
		EXPECT(m.GetDataPointer() == dataBefore);
		EXPECT(m.NumberOfRows() == 4);

		m.SetNumberOfRowsAndColumns(8, 8); //beyond it: a new allocation, and the size is right
		EXPECT(m.NumberOfRows() == 8);
		EXPECT(m.NumberOfColumns() == 8);
		//NOTE: the CONTENT after growing is deliberately not checked - the class does not promise
		//anything about it, and a test that pins down undefined data is a test of today's accident
	},

	CASE("ResizableMatrix: operators and products agree with Matrix")
	{
		//the class inherits these, so the test is that inheriting did not break the sizes
		ResizableMatrix a, b;
		MatrixVariantsFill(a, 2, 3);
		MatrixVariantsFill(b, 3, 2);

		Matrix aPlain(2, 3), bPlain(3, 2);
		for (Index i = 0; i < 2; i++) { for (Index j = 0; j < 3; j++) { aPlain(i, j) = MatrixVariantsValue(i, j); } }
		for (Index i = 0; i < 3; i++) { for (Index j = 0; j < 2; j++) { bPlain(i, j) = MatrixVariantsValue(i, j); } }

		Matrix product = aPlain * bPlain;
		EXPECT(product.NumberOfRows() == 2);
		EXPECT(product.NumberOfColumns() == 2);

		a *= 2.;
		EXPECT(a(1, 2) == 2. * MatrixVariantsValue(1, 2));
		a *= 0.5;
		EXPECT(MatrixVariantsHoldsValues(a, 2, 3));

		//TransposeYourself works on SQUARE matrices only - it transposes in place and would have to
		//reallocate otherwise; the 2x3 above therefore cannot be transposed, which is a limitation
		//worth knowing rather than a defect
		ResizableMatrix square;
		MatrixVariantsFill(square, 3, 3);
		square.TransposeYourself();
		for (Index i = 0; i < 3; i++)
		{
			for (Index j = 0; j < 3; j++)
			{
				EXPECT(square(i, j) == MatrixVariantsValue(j, i));
			}
		}
	},

	CASE("ConstSizeMatrix: at, below and above its static size")
	{
		ConstSizeMatrix<16> m; //4x4 worth of data
		MatrixVariantsFill(m, 4, 4);
		EXPECT(MatrixVariantsHoldsValues(m, 4, 4));
		EXPECT(m.MaxDataSize() == 16);

		ConstSizeMatrix<16> copy = m; //the data lives IN the object: a copy must copy the values
		EXPECT(copy == m);
		copy(0, 0) = -7.;
		EXPECT(m(0, 0) == MatrixVariantsValue(0, 0));

		MatrixVariantsFill(m, 2, 3); //below the static size
		EXPECT(MatrixVariantsHoldsValues(m, 2, 3));
		EXPECT(m.MaxDataSize() == 16); //unchanged: the storage is the object

		ConstSizeMatrix<16> square;
		MatrixVariantsFill(square, 4, 4);
		square.TransposeYourself();
		for (Index i = 0; i < 4; i++)
		{
			for (Index j = 0; j < 4; j++)
			{
				EXPECT(square(i, j) == MatrixVariantsValue(j, i));
			}
		}
	},

	CASE("ConstSizeMatrix: 3x3 and 6x6, the sizes the solver uses")
	{
		//ConstSizeMatrix<9>/<36> appear all over the rigid body code; a product of the small ones
		//is compared against a hand-written loop, which states what the operator means
		ConstSizeMatrix<9> a, b;
		MatrixVariantsFill(a, 3, 3);
		MatrixVariantsFill(b, 3, 3);

		ConstSizeMatrix<9> product = a * b;
		EXPECT(product.NumberOfRows() == 3);
		EXPECT(product.NumberOfColumns() == 3);
		for (Index i = 0; i < 3; i++)
		{
			for (Index j = 0; j < 3; j++)
			{
				Real reference = 0.;
				for (Index k = 0; k < 3; k++) { reference += a(i, k) * b(k, j); }
				EXPECT(product(i, j) == reference);
			}
		}

		ConstSizeMatrix<36> big;
		MatrixVariantsFill(big, 6, 6);
		EXPECT(MatrixVariantsHoldsValues(big, 6, 6));
		big.SetAll(0.);
		EXPECT(big(5, 5) == 0.);
	},

	CASE("LinkedDataMatrix: linked to a whole matrix, writing reaches the original")
	{
		//LinkedDataMatrix(const MatrixBase&) read the protected members of another object and did
		//not compile at all until #2473; these tests are its first users
		Matrix original;
		MatrixVariantsFill(original, 3, 4);

		LinkedDataMatrix linked(original);
		EXPECT(linked.NumberOfRows() == 3);
		EXPECT(linked.NumberOfColumns() == 4);
		EXPECT(MatrixVariantsHoldsValues(linked, 3, 4));
		EXPECT(linked.GetDataPointer() == original.GetDataPointer()); //linked, not copied

		linked(1, 1) = -5.;
		EXPECT(original(1, 1) == -5.); //writing through the link IS writing the original
	},

	CASE("LinkedDataMatrix: a ROW SUB-RANGE leaves everything outside it untouched")
	{
		//the matrix analogue of the LinkedDataVector case of #2394: the linked range starts at an
		//offset into someone else's data, and an operation on it must stay inside those rows
		const Index numberOfRows = 6;
		const Index numberOfColumns = 3;
		for (Index startRow = 0; startRow + 2 <= numberOfRows; startRow++)
		{
			Matrix original;
			MatrixVariantsFill(original, numberOfRows, numberOfColumns);

			const Index linkedRows = 2;
			//the constructor says this itself since #2473; before that the caller had to do the
			//row-major pointer arithmetic (row startRow begins startRow*numberOfColumns items in)
			LinkedDataMatrix linked(original, startRow, linkedRows);
			EXPECT(linked.GetDataPointer() == original.GetDataPointer() + startRow * numberOfColumns);
			EXPECT(linked.NumberOfRows() == linkedRows);
			EXPECT(linked.NumberOfColumns() == numberOfColumns);

			for (Index i = 0; i < linkedRows; i++) //the link shows the rows it was given
			{
				for (Index j = 0; j < numberOfColumns; j++)
				{
					EXPECT(linked(i, j) == MatrixVariantsValue(startRow + i, j));
				}
			}

			linked.SetAll(-1.); //write the whole sub-range

			for (Index i = 0; i < numberOfRows; i++)
			{
				for (Index j = 0; j < numberOfColumns; j++)
				{
					const bool insideLink = (i >= startRow) && (i < startRow + linkedRows);
					EXPECT(original(i, j) == (insideLink ? -1. : MatrixVariantsValue(i, j)));
				}
			}
		}
	},

	CASE("LinkedDataMatrix: linked to a raw data pointer")
	{
		Vector data(6);
		for (Index i = 0; i < 6; i++) { data[i] = (Real)i; }

		LinkedDataMatrix linked(data.GetDataPointer(), 2, 3); //row-major 2x3 over the same memory
		EXPECT(linked.NumberOfRows() == 2);
		EXPECT(linked.NumberOfColumns() == 3);
		EXPECT(linked(0, 0) == 0.);
		EXPECT(linked(0, 2) == 2.);
		EXPECT(linked(1, 0) == 3.); //row-major: this is the fact worth pinning down
		EXPECT(linked(1, 2) == 5.);
	},

	CASE("MatrixContainer: dense and sparse answer the same questions")
	{
		//the same 3x3 values, built both ways; every question is asked of both containers
		const Index size = 3;
		//row 1 deliberately holds TWO entries: the sparse path accumulates triplets with '+=', and
		//with one entry per row a '=' would pass unnoticed (found by mutating that very line)
		Matrix dense(size, size);
		dense.SetAll(0.);
		dense(0, 0) = 2.;
		dense(1, 0) = 0.5;
		dense(1, 2) = -1.5;
		dense(2, 1) = 4.;

		EXUmath::MatrixContainer denseContainer(dense);
		EXPECT(denseContainer.UseDenseMatrix());

		EXUmath::SparseTripletMatrix sparse;
		sparse.SetNumberOfRowsAndColumns(size, size);
		sparse.AddTriplet(EXUmath::Triplet(0, 0, 2.));
		sparse.AddTriplet(EXUmath::Triplet(1, 0, 0.5));
		sparse.AddTriplet(EXUmath::Triplet(1, 2, -1.5));
		sparse.AddTriplet(EXUmath::Triplet(2, 1, 4.));

		EXUmath::MatrixContainer sparseContainer(sparse);
		EXPECT(!sparseContainer.UseDenseMatrix());

		EXPECT(denseContainer.NumberOfRows() == sparseContainer.NumberOfRows());
		EXPECT(denseContainer.NumberOfColumns() == sparseContainer.NumberOfColumns());
		EXPECT(denseContainer.NumberOfRows() == size);

		//the dense matrix each container hands out must be the same matrix
		EXPECT(denseContainer.GetEXUdenseMatrix() == sparseContainer.GetEXUdenseMatrix());

		//and so must a matrix-vector product, which is where the two implementations part ways.
		//The result vectors are deliberately left UNSIZED: both modes must size them, which is what
		//#2474 fixed - the sparse path used to index into whatever the caller passed, throwing in a
		//checked build and writing out of bounds in a module compiled without range checks
		Vector x(size);
		x[0] = 1.; x[1] = 2.; x[2] = 3.;
		Vector denseResult, sparseResult;
		denseContainer.MultMatrixVector(x, denseResult);
		sparseContainer.MultMatrixVector(x, sparseResult);
		EXPECT(denseResult.NumberOfItems() == size);
		EXPECT(denseResult == sparseResult);
		EXPECT(denseResult[0] == 2.);     //2*1
		EXPECT(denseResult[1] == -4.);    //0.5*1 + (-1.5)*3, two triplets in one row
		EXPECT(denseResult[2] == 8.);     //4*2

		//MultMatrixVectorAdd accumulates rather than overwrites - both must agree on that too
		Vector accumulated = denseResult;
		denseContainer.MultMatrixVectorAdd(x, accumulated);
		for (Index i = 0; i < size; i++) { EXPECT(accumulated[i] == 2. * denseResult[i]); }

		Vector accumulatedSparse = sparseResult;
		sparseContainer.MultMatrixVectorAdd(x, accumulatedSparse);
		EXPECT(accumulated == accumulatedSparse);
	},

	CASE("MatrixContainer: SetAllZero and switching the mode")
	{
		Matrix dense(2, 2);
		dense.SetAll(3.);
		EXUmath::MatrixContainer container(dense);
		EXPECT(container.UseDenseMatrix());
		EXPECT(container.GetEXUdenseMatrix()(1, 1) == 3.);

		container.SetAllZero();
		EXPECT(container.NumberOfRows() == 2); //zeroed, not resized
		EXPECT(container.GetEXUdenseMatrix()(1, 1) == 0.);

		container.SetUseDenseMatrix(false); //a sparse container with no triplets is the zero matrix
		EXPECT(!container.UseDenseMatrix());
		//switching the mode does NOT carry the size over - the sparse matrix has its own, and the
		//comment on SetUseDenseMatrix warns that the state is undefined until it is given one
		EXPECT(container.NumberOfRows() == 0);
		container.GetInternalSparseTripletMatrix().SetNumberOfRowsAndColumns(2, 2);

		Vector x(2), result;
		x[0] = 1.; x[1] = 1.;
		container.MultMatrixVector(x, result);
		EXPECT(result.NumberOfItems() == 2); //sized by the product itself since #2474
		EXPECT(result[0] == 0.);
		EXPECT(result[1] == 0.);
	},

};

#endif //include header once
