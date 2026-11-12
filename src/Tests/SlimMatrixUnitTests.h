/** ***********************************************************************************************
* @file			SlimMatrixUnitTests.h
* @brief		Unit tests for SlimMatrix, the matrix of fixed rows and columns (#3456)
* @details		Details:
*				- every operation is compared with the same operation of ConstSizeMatrix on the same values,
*				  BITWISE: SlimMatrix promises the same order of every sum, so that switching Matrix3D or
*				  Matrix6D to it leaves the results of the test suite unchanged;
*				- the sizes are those of the solver: 3x3, 3x4 and 4x3 (the G matrices of Euler parameters),
*				  6x6, and the 3 x n of the Jacobians;
*				- the values are not symmetric and not integers, so that a transposed index or a changed
*				  order of a sum shows.
*
* @author		Gerstmayr Johannes
* @date			2026-11-12 (#3456)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
************************************************************************************************ */
#ifndef SLIMMATRIXUNITTESTS__H
#define SLIMMATRIXUNITTESTS__H

//a value whose sums are not exact, so that the order of a sum shows in the last bits
inline Real SlimMatrixTestValue(Index i, Index j, Real seed) { return std::sin(seed + 1.3 * (Real)i + 0.7 * (Real)j) / 3.; }

//! the same values into a SlimMatrix and a ConstSizeMatrix
template<Index rows, Index columns>
inline void SlimMatrixTestFill(SlimMatrix<rows, columns>& slim, ConstSizeMatrix<rows * columns>& constSize, Real seed)
{
	constSize.SetNumberOfRowsAndColumns(rows, columns);
	for (Index i = 0; i < rows; i++)
	{
		for (Index j = 0; j < columns; j++) { slim(i, j) = constSize(i, j) = SlimMatrixTestValue(i, j, seed); }
	}
}

//! bitwise equal items and the same size
template<Index rows, Index columns, Index dataSize>
inline bool SlimMatrixTestSame(const SlimMatrix<rows, columns>& slim, const ConstSizeMatrix<dataSize>& constSize)
{
	if (constSize.NumberOfRows() != rows || constSize.NumberOfColumns() != columns) { return false; }
	for (Index i = 0; i < rows; i++)
	{
		for (Index j = 0; j < columns; j++) { if (slim(i, j) != constSize(i, j)) { return false; } }
	}
	return true;
}

template<Index n>
inline bool SlimVectorTestSame(const SlimVector<n>& v, const SlimVector<n>& w)
{
	for (Index i = 0; i < n; i++) { if (v[i] != w[i]) { return false; } }
	return true;
}

const lest::test slimMatrix_specific_test[] =
{
	CASE("SlimMatrix: the size is the type's, and the conversions to and from ConstSizeMatrix")
	{
		SlimMatrix<3, 4> A;
		ConstSizeMatrix<12> cA;
		SlimMatrixTestFill(A, cA, 0.1);
		EXPECT(A.NumberOfRows() == 3);
		EXPECT(A.NumberOfColumns() == 4);
		EXPECT(SlimMatrixTestSame(A, cA));

		ConstSizeMatrix<12> converted = A;
		EXPECT(SlimMatrixTestSame(A, converted));
		SlimMatrix<3, 4> back = converted;
		EXPECT(back == A);

		//from a ConstSizeMatrix of larger capacity, as the G matrices of maxRotationCoordinates*3
		ConstSizeMatrix<16> larger(3, 4);
		for (Index i = 0; i < 12; i++) { larger.GetDataPointer()[i] = cA.GetDataPointer()[i]; }
		SlimMatrix<3, 4> fromLarger(larger); //explicit from a larger capacity
		EXPECT(fromLarger == A);

#ifndef __FAST_EXUDYN_LINALG
		ConstSizeMatrix<12> wrongShape(4, 3);
		typedef SlimMatrix<3, 4> SlimMatrix34;
		typedef SlimMatrix<3, 3> SlimMatrix33;
		EXPECT_THROWS(SlimMatrix34 wrong = wrongShape);
		EXPECT_THROWS(A(3, 0));
		EXPECT_THROWS(A(0, 4));
		EXPECT_THROWS(A.SetNumberOfRowsAndColumns(4, 3));
		EXPECT_THROWS(SlimMatrix33 wrongList(3, 3, { 1., 2. }));
#endif
		A.SetNumberOfRowsAndColumns(3, 4); //its own size is accepted
		EXPECT(A == fromLarger);
	},

	CASE("SlimMatrix: products bitwise as ConstSizeMatrix - 3x3, 3x4 * 4x3, 6x6")
	{
		SlimMatrix<3, 3> A, B;
		ConstSizeMatrix<9> cA, cB;
		SlimMatrixTestFill(A, cA, 0.2);
		SlimMatrixTestFill(B, cB, 1.7);
		EXPECT(SlimMatrixTestSame(A * B, cA * cB));
		EXPECT(SlimMatrixTestSame(A.GetTransposed() * B, cA.GetTransposed() * cB));
		EXPECT(MultTransposedMatrix(A, B) == A.GetTransposed() * B);

		SlimMatrix<3, 4> G;
		SlimMatrix<4, 3> H;
		ConstSizeMatrix<12> cG, cH;
		SlimMatrixTestFill(G, cG, 0.4);
		SlimMatrixTestFill(H, cH, 2.1);
		ConstSizeMatrix<9> cGH;
		EXUmath::MultMatrixMatrix(cG, cH, cGH);
		EXPECT(SlimMatrixTestSame(G * H, cGH));
		EXPECT(SlimMatrixTestSame(G.GetTransposed(), cG.GetTransposed()));
		ConstSizeMatrix<12> cAG;
		ConstSizeMatrix<16> cGtAG;
		EXUmath::MultMatrixMatrix(cA, cG, cAG);
		EXUmath::MultMatrixMatrix(cG.GetTransposed(), cAG, cGtAG);
		EXPECT(SlimMatrixTestSame(MultTransposedMatrix(G, A * G), cGtAG));
		SlimMatrix<4, 4> viaTemplate; //the EXUmath templates take a SlimMatrix as result
		EXUmath::MultMatrixMatrix(G.GetTransposed(), A * G, viaTemplate);
		EXPECT(viaTemplate == MultTransposedMatrix(G, A * G));

		SlimMatrix<6, 6> M, N;
		ConstSizeMatrix<36> cM, cN;
		SlimMatrixTestFill(M, cM, 0.9);
		SlimMatrixTestFill(N, cN, -0.3);
		EXPECT(SlimMatrixTestSame(M * N, cM * cN));
		EXPECT(SlimMatrixTestSame(M + N, cM + cN));
		EXPECT(SlimMatrixTestSame(M - N, cM - cN));
		EXPECT(SlimMatrixTestSame(0.3 * M, 0.3 * cM));
		EXPECT(SlimMatrixTestSame(M * 0.3, cM * 0.3));
		M += N;
		cM += cN;
		EXPECT(SlimMatrixTestSame(M, cM));
		M -= N;
		cM -= cN;
		EXPECT(SlimMatrixTestSame(M, cM));
		M *= 1.1;
		cM *= 1.1;
		EXPECT(SlimMatrixTestSame(M, cM));
	},

	CASE("SlimMatrix: matrix times vector bitwise as ConstSizeMatrix - 3x3, 6x6, and the transposed")
	{
		SlimMatrix<3, 3> A;
		ConstSizeMatrix<9> cA;
		SlimMatrixTestFill(A, cA, 0.5);
		Vector3D v({ 0.3, -0.21, 0.77 });
		EXPECT(SlimVectorTestSame(A * v, cA * v));
		EXPECT(SlimVectorTestSame(v * A, v * cA));
		EXPECT(SlimVectorTestSame(MultTransposedVector(A, v), cA.GetTransposed() * v));

		SlimMatrix<6, 6> M;
		ConstSizeMatrix<36> cM;
		SlimMatrixTestFill(M, cM, 1.3);
		Vector6D w({ 0.1, -0.2, 0.33, 0.47, -0.51, 0.6 });
		EXPECT(SlimVectorTestSame(M * w, cM * w));
		EXPECT(SlimVectorTestSame(w * M, w * cM));

		//3 x 4 times a vector of 4: the G matrix of Euler parameters times their velocities
		SlimMatrix<3, 4> G;
		ConstSizeMatrix<12> cG;
		SlimMatrixTestFill(G, cG, -0.8);
		SlimVector<4> e({ 0.5, 0.5, -0.5, 0.5 });
		ConstSizeVector<3> cGe;
		EXUmath::MultMatrixVector(cG, e, cGe);
		Vector3D Ge = G * e;
		EXPECT((Ge[0] == cGe[0] && Ge[1] == cGe[1] && Ge[2] == cGe[2]));
	},

	CASE("Transposed(A) * x and Transposed(A) * B bitwise as A.GetTransposed() * x - 3x3, 3x4, 6x6, of an rvalue (#3479)")
	{
		SlimMatrix<3, 3> A, B;
		ConstSizeMatrix<9> cA, cB;
		SlimMatrixTestFill(A, cA, 0.35);
		SlimMatrixTestFill(B, cB, -1.2);
		Vector3D v({ 0.3, -0.21, 0.77 });
		EXPECT(SlimVectorTestSame(Transposed(A) * v, A.GetTransposed() * v));
		EXPECT(SlimVectorTestSame(Transposed(A) * v, cA.GetTransposed() * v));
		EXPECT(Transposed(A) * B == A.GetTransposed() * B);
		EXPECT(SlimMatrixTestSame(Transposed(A) * B, cA.GetTransposed() * cB));
		EXPECT(SlimVectorTestSame(Transposed(A * B) * v, (A * B).GetTransposed() * v)); //an rvalue: the matrix is kept

		SlimMatrix<3, 4> G;
		ConstSizeMatrix<12> cG;
		SlimMatrixTestFill(G, cG, 0.45);
		SlimVector<4> gtv = Transposed(G) * v;
		SlimVector<4> gtvReference = G.GetTransposed() * v;
		EXPECT((gtv[0] == gtvReference[0] && gtv[1] == gtvReference[1] && gtv[2] == gtvReference[2] && gtv[3] == gtvReference[3]));
		EXPECT(Transposed(G) * A == G.GetTransposed() * A);

		SlimMatrix<6, 6> M, N;
		ConstSizeMatrix<36> cM, cN;
		SlimMatrixTestFill(M, cM, 0.9);
		SlimMatrixTestFill(N, cN, 0.15);
		Vector6D w({ 0.1, -0.2, 0.33, 0.47, -0.51, 0.6 });
		EXPECT(SlimVectorTestSame(Transposed(M) * w, M.GetTransposed() * w));
		EXPECT(SlimVectorTestSame(Transposed(M) * w, cM.GetTransposed() * w));
		EXPECT(Transposed(M) * N == M.GetTransposed() * N);
	},

	CASE("EXUmath products with ResultMode: add and subtract bitwise as r + p and r - p, the block as the product (#3481)")
	{
		const Index n = 7;
		ResizableMatrix J(6, n), K(6, 6), A(6, 4);
		Vector v(6), w(n), r0(6), q0(n);
		for (Index i = 0; i < 6; i++)
		{
			for (Index j = 0; j < n; j++) { J(i, j) = SlimMatrixTestValue(i, j, 0.3); }
			for (Index j = 0; j < 6; j++) { K(i, j) = SlimMatrixTestValue(i, j, 1.1); }
			for (Index j = 0; j < 4; j++) { A(i, j) = SlimMatrixTestValue(i, j, -0.6); }
			v[i] = SlimMatrixTestValue(i, 0, 2.2);
			r0[i] = SlimMatrixTestValue(i, 1, 0.8);
		}
		for (Index j = 0; j < n; j++) { w[j] = SlimMatrixTestValue(j, 2, 0.4); q0[j] = SlimMatrixTestValue(j, 3, 0.9); }

		//matrix * vector
		Vector p, r;
		EXUmath::MultMatrixVector(K, v, p);
		r = r0; EXUmath::MultMatrixVectorAdd(K, v, r);
		bool same = true;
		for (Index i = 0; i < 6; i++) { if (r[i] != r0[i] + p[i]) { same = false; } }
		r = r0; EXUmath::MultMatrixVector<EXUmath::ResultMode::subtract>(K, v, r);
		for (Index i = 0; i < 6; i++) { if (r[i] != r0[i] - p[i]) { same = false; } }
		EXPECT(same);

		//matrix^T * vector
		Vector pt, rt;
		EXUmath::MultMatrixTransposedVector(J, v, pt);
		rt = q0; EXUmath::MultMatrixTransposedVectorAdd(J, v, rt);
		same = true;
		for (Index i = 0; i < n; i++) { if (rt[i] != q0[i] + pt[i]) { same = false; } }
		EXPECT(same);

		//m1^T * m2, set and add, and the block product of the connectors: J^T K, then (J^T K) J into a block
		ResizableMatrix JtK, JtKadd(n, 6), JtKJ, block(2 * n, 2 * n);
		EXUmath::MultMatrixTransposedMatrix(J, K, JtK);
		JtKadd.SetAll(0.25);
		EXUmath::MultMatrixTransposedMatrixAdd(J, K, JtKadd);
		EXUmath::MultMatrixMatrix(JtK, J, JtKJ);
		block.SetAll(-1.);
		EXUmath::MultMatrixMatrixToSubmatrix(JtK, J, block, n, 0);
		ResizableMatrix blockAdd(block);
		EXUmath::MultMatrixMatrixToSubmatrix<EXUmath::ResultMode::add>(JtK, J, blockAdd, 0, n);
		same = true;
		for (Index i = 0; i < n; i++)
		{
			for (Index j = 0; j < 6; j++) { if (JtKadd(i, j) != 0.25 + JtK(i, j)) { same = false; } }
			for (Index j = 0; j < n; j++)
			{
				if (block(n + i, j) != JtKJ(i, j)) { same = false; }
				if (block(i, j) != -1. || block(i, n + j) != -1.) { same = false; } //outside the block untouched
				if (blockAdd(i, n + j) != -1. + JtKJ(i, j)) { same = false; }
			}
		}
		EXPECT(same);

		//m1 * m2 add, and the sizes are checked in the add modes
		ResizableMatrix KA, KAadd(6, 4);
		EXUmath::MultMatrixMatrix(K, A, KA);
		KAadd.SetAll(0.5);
		EXUmath::MultMatrixMatrix<EXUmath::ResultMode::add>(K, A, KAadd);
		same = true;
		for (Index i = 0; i < 6; i++) { for (Index j = 0; j < 4; j++) { if (KAadd(i, j) != 0.5 + KA(i, j)) { same = false; } } }
		EXPECT(same);
		ResizableMatrix wrongSize(3, 3);
		EXPECT_THROWS(EXUmath::MultMatrixMatrix<EXUmath::ResultMode::add>(K, A, wrongSize));
		EXPECT_THROWS(EXUmath::MultMatrixMatrixToSubmatrix(JtK, J, block, 2 * n - 1, 0));
	},

	CASE("SlimVector::MultAdd bitwise as += scalar * v (#3484)")
	{
		Vector6D a({ 0.1, -0.2, 0.33, 0.47, -0.51, 0.6 });
		Vector6D b({ 1.3, 0.7, -0.11, 0.29, 2.1, -0.37 });
		Vector6D viaOperator = a;
		viaOperator += 0.37 * b;
		Vector6D viaMultAdd = a;
		viaMultAdd.MultAdd(0.37, b);
		EXPECT(SlimVectorTestSame(viaMultAdd, viaOperator));
	},

	CASE("SlimMatrix: inverse, determinant, trace, skew matrix, transposing, rows and columns")
	{
		SlimMatrix<3, 3> A;
		ConstSizeMatrix<9> cA;
		SlimMatrixTestFill(A, cA, 0.6);
		EXPECT(SlimMatrixTestSame(A.GetInverse(), cA.GetInverse()));
		EXPECT(A.GetDeterminant() == cA.GetDeterminant());
		EXPECT(A.Trace() == cA.Trace());

		SlimMatrix<2, 2> B;
		ConstSizeMatrix<4> cB;
		SlimMatrixTestFill(B, cB, 0.6);
		EXPECT(SlimMatrixTestSame(B.GetInverse(), cB.GetInverse()));
		EXPECT(B.GetDeterminant() == cB.GetDeterminant());

		SlimMatrix<4, 4> C;
		ConstSizeMatrix<16> cC;
		SlimMatrixTestFill(C, cC, 0.25);
		for (Index i = 0; i < 4; i++) { C(i, i) += 2.; cC(i, i) += 2.; } //well conditioned
		EXPECT(SlimMatrixTestSame(C.GetInverse(), cC.GetInverse()));

		Vector3D v({ 0.3, -0.21, 0.77 });
		SlimMatrix<3, 3> skew;
		ConstSizeMatrix<9> cSkew;
		skew.SetSkewMatrix(v);
		cSkew.SetSkewMatrix(v);
		EXPECT(SlimMatrixTestSame(skew, cSkew));
		EXPECT(SlimVectorTestSame(skew * Vector3D({ 1., 2., 3. }), v.CrossProduct(Vector3D({ 1., 2., 3. }))));

		SlimMatrix<3, 3> unit;
		unit.SetScalarMatrix(3, 1.);
		EXPECT(SlimMatrixTestSame(unit * A, cA));

		SlimMatrix<3, 3> transposed = A;
		transposed.TransposeYourself();
		EXPECT(transposed == A.GetTransposed());

		Vector3D column = A.GetColumnVector(1);
		Vector3D row = A.GetRowVector(2);
		EXPECT(SlimVectorTestSame(column, cA.GetColumnVector(1)));
		EXPECT(SlimVectorTestSame(row, cA.GetRowVector<3>(2)));
		A.SetColumnVector(0, v);
		EXPECT((A(0, 0) == v[0] && A(1, 0) == v[1] && A(2, 0) == v[2]));

		SlimMatrix<3, 4> diadic;
		diadic.SetWithDiadicProduct(v, SlimVector<4>({ 1., -1., 2., 0.5 }));
		EXPECT(diadic(2, 2) == v[2] * 2.);
	},
};

#endif
