/** ***********************************************************************************************
* @file			linalgBenchmark.h
* @brief		The linear algebra micro-benchmark that every build carries: exudyn.special.RunLinalgBenchmark()
* @details		Details:
*				- times the operations of the computation functions of objects, markers and connectors and of the
*				  assembly, with the compiler options of the build (regular or fast module), each with Eigen as the
*				  reference where Eigen has the operation:
*				  - group 'vector': Vector, ConstSizeVector and ResizableVectorParallel add, scale, MultAdd over sizes
*				    3 to 500 and across ResizableVectorParallelThreadingLimit, single- and multithreaded
*				  - group 'small': Matrix3D, ConstSizeMatrix 6x6, HomogeneousTransformation, SlimVector
*				  - group 'jacobian': J^T f and J^T K J for a 3 x n Jacobian, n = 6 ... 24, as the connectors do
*				  - group 'dense': Matrix * Vector, and the factorization and solve of the dense solver (InvertSpecial)
*				  - group 'assembly': local 12 x 12 Jacobians of a chain into the sparse triplets, FinalizeMatrix,
*				    and the same into a sparse matrix of known pattern
*				  - group 'autodiff': AutoDiff<12> products as in the analytic connector Jacobians
*				  - library 'prototype': a matrix of compile-time size, as evidence for the gain of known sizes
*				- each operation runs until minimumTime has passed and is reported in ns per call; a check sum keeps the
*				  compiler from removing the work
*				- tools/benchmarks/avx2Benchmark.py is the counterpart on the level of the solvers
*
* @author		Gerstmayr Johannes
* @date			2026-11-11 (#3410)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
*
************************************************************************************************ */
#ifndef LINALGBENCHMARK__H
#define LINALGBENCHMARK__H

#include <chrono>
#include <string>
#include <vector>
#include "Linalg/BasicLinalg.h"
#include "Linalg/ResizableVectorParallel.h"
#include "Linalg/HomogeneousTransformation.h"
#include "Linalg/LinearSolver.h"			//Eigen (dense and sparse) and the system matrices
#include "Utilities/AutomaticDifferentiation.h"
#include "Utilities/Threading.h"

//! one result of the benchmark: group, library (Exudyn, Eigen, prototype), operation, size, threads, ns per call
struct LinalgBenchmarkResult
{
	std::string group;
	std::string library;
	std::string operation;
	Index size;
	Index threads;
	Real nsPerCall;
	Real checkSum;
};

//! run operation (which returns a number for the check sum) repeatedly for at least minimumTime seconds; ns per call
template<class TFunction>
inline LinalgBenchmarkResult LinalgBenchmarkTime(const std::string& group, const std::string& library, const std::string& operation,
	Index size, Index threads, Real minimumTime, TFunction operation_)
{
	typedef std::chrono::steady_clock Clock;
	Real checkSum = 0.;
	Index calls = 0;
	Index batch = 1;
	auto t0 = Clock::now();
	Real elapsed = 0.;
	while (elapsed < minimumTime)
	{
		for (Index i = 0; i < batch; i++) { checkSum += operation_(); }
		calls += batch;
		batch *= 2; //few reads of the clock for fast operations
		elapsed = std::chrono::duration<Real>(Clock::now() - t0).count();
	}
	return LinalgBenchmarkResult({ group, library, operation, size, threads, 1e9 * elapsed / (Real)calls, checkSum });
}

//! a row-major matrix of compile-time size: the prototype that measures what known sizes gain (loops unrolled, no checks)
template<Index rows, Index columns>
struct LinalgBenchmarkFixedMatrix
{
	Real data[rows * columns];
	Real& operator()(Index i, Index j) { return data[i * columns + j]; }
	const Real& operator()(Index i, Index j) const { return data[i * columns + j]; }
};

//! result = m1 * m2, sizes known at compile time
template<Index rows, Index inner, Index columns>
inline void LinalgBenchmarkFixedMult(const LinalgBenchmarkFixedMatrix<rows, inner>& m1, const LinalgBenchmarkFixedMatrix<inner, columns>& m2,
	LinalgBenchmarkFixedMatrix<rows, columns>& result)
{
	for (Index i = 0; i < rows; i++)
	{
		for (Index j = 0; j < columns; j++)
		{
			Real value = 0.;
			for (Index k = 0; k < inner; k++) { value += m1(i, k) * m2(k, j); }
			result(i, j) = value;
		}
	}
}

//! result = m1^T * m2, sizes known at compile time
template<Index inner, Index rows, Index columns>
inline void LinalgBenchmarkFixedMultTransposed(const LinalgBenchmarkFixedMatrix<inner, rows>& m1, const LinalgBenchmarkFixedMatrix<inner, columns>& m2,
	LinalgBenchmarkFixedMatrix<rows, columns>& result)
{
	for (Index i = 0; i < rows; i++) { for (Index j = 0; j < columns; j++) { result(i, j) = 0.; } }
	for (Index k = 0; k < inner; k++)
	{
		for (Index i = 0; i < rows; i++)
		{
			const Real a = m1(k, i);
			for (Index j = 0; j < columns; j++) { result(i, j) += a * m2(k, j); }
		}
	}
}

//! the Jacobian kernels J^T f and J^T K J of a connector on a 3 x n Jacobian, n known at compile time: Exudyn, Eigen, prototype
template<Index n>
inline void LinalgBenchmarkJacobian(std::vector<LinalgBenchmarkResult>& results, Real minimumTime)
{
	const std::string group = "jacobian";
	ResizableMatrix J(3, n), KJ, JKJ;
	Matrix3D K(3, 3, { 2., 0.1, 0., 0.1, 3., 0.2, 0., 0.2, 4. });
	Eigen::MatrixXd Jd(3, n), Kd(3, 3), JKJd(n, n);
	Eigen::Matrix<Real, 3, n> Jf;
	Eigen::Matrix<Real, 3, 3> Kf;
	Eigen::Matrix<Real, n, n> JKJf;
	LinalgBenchmarkFixedMatrix<3, n> Jp;
	LinalgBenchmarkFixedMatrix<3, 3> Kp;
	LinalgBenchmarkFixedMatrix<3, n> KJp;
	LinalgBenchmarkFixedMatrix<n, n> JKJp;
	for (Index i = 0; i < 3; i++)
	{
		for (Index j = 0; j < n; j++)
		{
			Real value = (i == j % 3) ? 1. : 0.01 * (Real)(i + j);
			J(i, j) = value; Jd(i, j) = value; Jf(i, j) = value; Jp(i, j) = value;
		}
		for (Index j = 0; j < 3; j++) { Kd(i, j) = K(i, j); Kf(i, j) = K(i, j); Kp(i, j) = K(i, j); }
	}
	Vector3D f({ 0.3, -0.2, 0.5 });
	ResizableVector r;
	Eigen::Vector3d fd(0.3, -0.2, 0.5);
	Eigen::VectorXd rd(n);
	Eigen::Matrix<Real, n, 1> rf;
	Real rp[n];

	results.push_back(LinalgBenchmarkTime(group, "Exudyn", "J^T f, ResizableMatrix 3 x n", n, 1, minimumTime, [&]()
		{ EXUmath::MultMatrixTransposedVectorTemplate(J, f, r); f[0] += 1e-12 * r[n - 1]; return r[0]; }));
	results.push_back(LinalgBenchmarkTime(group, "Eigen", "J^T f, MatrixXd 3 x n", n, 1, minimumTime, [&]()
		{ rd.noalias() = Jd.transpose() * fd; fd[0] += 1e-12 * rd[n - 1]; return rd[0]; }));
	results.push_back(LinalgBenchmarkTime(group, "Eigen", "J^T f, Matrix<3,n>", n, 1, minimumTime, [&]()
		{ rf.noalias() = Jf.transpose() * fd; fd[0] += 1e-12 * rf[n - 1]; return rf[0]; }));
	results.push_back(LinalgBenchmarkTime(group, "prototype", "J^T f, fixed 3 x n", n, 1, minimumTime, [&]()
		{
			for (Index j = 0; j < n; j++) { rp[j] = Jp(0, j) * f[0] + Jp(1, j) * f[1] + Jp(2, j) * f[2]; }
			f[0] += 1e-12 * rp[n - 1]; return rp[0];
		}));

	results.push_back(LinalgBenchmarkTime(group, "Exudyn", "J^T K J, ResizableMatrix", n, 1, minimumTime, [&]()
		{
			EXUmath::MultMatrixMatrixTemplate(K, J, KJ);
			EXUmath::MultMatrixTransposedMatrixTemplate(J, KJ, JKJ);
			K(0, 0) += 1e-12 * JKJ(n - 1, n - 1); return JKJ(0, 0);
		}));
	results.push_back(LinalgBenchmarkTime(group, "Eigen", "J^T K J, MatrixXd", n, 1, minimumTime, [&]()
		{ JKJd.noalias() = Jd.transpose() * (Kd * Jd); Kd(0, 0) += 1e-12 * JKJd(n - 1, n - 1); return JKJd(0, 0); }));
	results.push_back(LinalgBenchmarkTime(group, "Eigen", "J^T K J, Matrix<3,n>", n, 1, minimumTime, [&]()
		{ JKJf.noalias() = Jf.transpose() * (Kf * Jf); Kf(0, 0) += 1e-12 * JKJf(n - 1, n - 1); return JKJf(0, 0); }));
	results.push_back(LinalgBenchmarkTime(group, "prototype", "J^T K J, fixed", n, 1, minimumTime, [&]()
		{
			LinalgBenchmarkFixedMult(Kp, Jp, KJp);
			LinalgBenchmarkFixedMultTransposed(Jp, KJp, JKJp);
			Kp(0, 0) += 1e-12 * JKJp(n - 1, n - 1); return JKJp(0, 0);
		}));
}

//! the vector operations of size n known at compile time: ConstSizeVector against Eigen and the prototype
template<Index n>
inline void LinalgBenchmarkFixedVector(std::vector<LinalgBenchmarkResult>& results, Real minimumTime)
{
	ConstSizeVector<n> a, b;
	Eigen::Matrix<Real, n, 1> ea, eb;
	Real pa[n], pb[n];
	for (Index i = 0; i < n; i++) { a[i] = ea[i] = pa[i] = 1e-3 * (Real)(i % 7); b[i] = eb[i] = pb[i] = 1e-3 * (Real)(i % 5); }
	results.push_back(LinalgBenchmarkTime("vector", "Exudyn", "ConstSizeVector +=", n, 1, minimumTime, [&]() { a += b; return a[0]; }));
	results.push_back(LinalgBenchmarkTime("vector", "Eigen", "Matrix<n,1> +=", n, 1, minimumTime, [&]() { ea += eb; return ea[0]; }));
	results.push_back(LinalgBenchmarkTime("vector", "prototype", "fixed +=", n, 1, minimumTime, [&]()
		{ for (Index i = 0; i < n; i++) { pa[i] += pb[i]; } return pa[0]; }));
}

//! the benchmark; numberOfThreads > 1 adds the multithreaded ResizableVectorParallel runs
inline std::vector<LinalgBenchmarkResult> RunLinalgBenchmark(Real minimumTime, Index numberOfThreads)
{
	std::vector<LinalgBenchmarkResult> results;

	//+++ vectors over a sweep of sizes: small ones as in the objects, the long ones of the time integration
	Index limit = ResizableVectorParallelThreadingLimit;
	std::vector<Index> sizes = { 3, 6, 12, 24, 48, 100, 200, 500, 1000, limit, 10 * limit };
	for (Index n : sizes)
	{
		Vector a(n), b(n);
		for (Index i = 0; i < n; i++) { a[i] = 1e-3 * (Real)(i % 7); b[i] = 1e-3 * (Real)(i % 5); }
		results.push_back(LinalgBenchmarkTime("vector", "Exudyn", "Vector +=", n, 1, minimumTime, [&]() { a += b; return a[0]; }));
		results.push_back(LinalgBenchmarkTime("vector", "Exudyn", "Vector *= scalar", n, 1, minimumTime, [&]() { a *= 0.999; return a[n - 1]; }));
		results.push_back(LinalgBenchmarkTime("vector", "Exudyn", "Vector MultAdd", n, 1, minimumTime, [&]() { a.MultAdd(1e-3, b); return a[0]; }));

		Eigen::VectorXd ea(n), eb(n);
		for (Index i = 0; i < n; i++) { ea[i] = a[i]; eb[i] = b[i]; }
		results.push_back(LinalgBenchmarkTime("vector", "Eigen", "VectorXd +=", n, 1, minimumTime, [&]() { ea += eb; return ea[0]; }));
		results.push_back(LinalgBenchmarkTime("vector", "Eigen", "VectorXd *= scalar", n, 1, minimumTime, [&]() { ea *= 0.999; return ea[n - 1]; }));
		results.push_back(LinalgBenchmarkTime("vector", "Eigen", "VectorXd MultAdd", n, 1, minimumTime, [&]() { ea += 1e-3 * eb; return ea[0]; }));

		//ResizableVectorParallel: multithreaded above the limit, if the threads run
		std::vector<Index> threadList = { 1 };
		if (numberOfThreads > 1 && n >= limit) { threadList.push_back(numberOfThreads); }
		for (Index threads : threadList)
		{
			ExuThreading::WorkersAcquired acquired = ExuThreading::workersNotAcquired;
			Index ticket = 0;
			if (threads > 1) //the persistent workers if their number agrees (#3437)
			{
				STDstring message;
				acquired = ExuThreading::TaskManager::AcquireWorkers(threads, false, message, ticket);
			}
			Index threadsUsed = ExuThreading::TaskManager::GetNumThreads();
			ResizableVectorParallel c(n), d(n);
			for (Index i = 0; i < n; i++) { c[i] = 1e-3 * (Real)(i % 7); d[i] = 1e-3 * (Real)(i % 5); }
			results.push_back(LinalgBenchmarkTime("vector", "Exudyn", "ResizableVectorParallel +=", n, threadsUsed, minimumTime, [&]() { c += d; return c[0]; }));
			results.push_back(LinalgBenchmarkTime("vector", "Exudyn", "ResizableVectorParallel MultAdd", n, threadsUsed, minimumTime, [&]() { c.MultAdd(1e-3, d); return c[0]; }));
			results.push_back(LinalgBenchmarkTime("vector", "Exudyn", "ResizableVectorParallel CopyFrom", n, threadsUsed, minimumTime, [&]() { c.CopyFrom(d); return c[n - 1]; }));
			if (threads > 1) { ExuThreading::TaskManager::ReleaseWorkers(acquired, ticket); }
		}
	}
	LinalgBenchmarkFixedVector<6>(results, minimumTime);
	LinalgBenchmarkFixedVector<12>(results, minimumTime);
	LinalgBenchmarkFixedVector<24>(results, minimumTime);

	//+++ small fixed-size types
	{
		const std::string group = "small";
		Vector3D v({ 0.3, -0.2, 0.7 }), w({ -0.1, 0.5, 0.2 });
		Matrix3D A(3, 3, { 0.36, 0.48, -0.8, -0.8, 0.6, 0., 0.48, 0.64, 0.6 });
		Matrix3D B = A.GetTransposed();
		Eigen::Vector3d ev(0.3, -0.2, 0.7), ew(-0.1, 0.5, 0.2);
		Eigen::Matrix3d eA, eB;
		LinalgBenchmarkFixedMatrix<3, 3> pA, pB, pC;
		for (Index i = 0; i < 3; i++) { for (Index j = 0; j < 3; j++) { eA(i, j) = pA(i, j) = A(i, j); eB(i, j) = pB(i, j) = B(i, j); } }
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "SlimVector<3> +", 3, 1, minimumTime, [&]() { v = v + w * 1e-6; return v[0]; }));
		results.push_back(LinalgBenchmarkTime(group, "Eigen", "Vector3d +", 3, 1, minimumTime, [&]() { ev = ev + ew * 1e-6; return ev[0]; }));
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "SlimVector<3> cross product", 3, 1, minimumTime, [&]() { v = v.CrossProduct(w) * 0.5 + w; return v[1]; }));
		results.push_back(LinalgBenchmarkTime(group, "Eigen", "Vector3d cross product", 3, 1, minimumTime, [&]() { ev = ev.cross(ew) * 0.5 + ew; return ev[1]; }));
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "Matrix3D * Vector3D", 3, 1, minimumTime, [&]() { v = A * v; return v[2]; }));
		results.push_back(LinalgBenchmarkTime(group, "Eigen", "Matrix3d * Vector3d", 3, 1, minimumTime, [&]() { ev = eA * ev; return ev[2]; }));
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "Matrix3D^T * Vector3D, GetTransposed", 3, 1, minimumTime, [&]() { v = A.GetTransposed() * v; return v[2]; }));
		ConstSizeVector<3> v3({ 0.3, -0.2, 0.7 }), r3;
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "Matrix3D^T * Vector3D, MultMatrixTransposedVectorTemplate", 3, 1, minimumTime, [&]()
			{ EXUmath::MultMatrixTransposedVectorTemplate(A, v3, r3); v3 = r3; return v3[2]; }));
		results.push_back(LinalgBenchmarkTime(group, "Eigen", "Matrix3d^T * Vector3d", 3, 1, minimumTime, [&]() { ev = eA.transpose() * ev; return ev[2]; }));
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "Matrix3D * Matrix3D", 3, 1, minimumTime, [&]() { B = A * B; return B(0, 1); }));
		results.push_back(LinalgBenchmarkTime(group, "Eigen", "Matrix3d * Matrix3d", 3, 1, minimumTime, [&]() { eB = eA * eB; return eB(0, 1); }));
		results.push_back(LinalgBenchmarkTime(group, "prototype", "fixed 3x3 * 3x3", 3, 1, minimumTime, [&]() { LinalgBenchmarkFixedMult(pA, pB, pC); pB = pC; return pB(0, 1); }));

		ConstSizeMatrix<36> M6(6, 6), R6(6, 6);
		ConstSizeVector<6> v6, r6;
		Eigen::Matrix<Real, 6, 6> eM6, eR6;
		Eigen::Matrix<Real, 6, 1> ev6, er6;
		LinalgBenchmarkFixedMatrix<6, 6> pM6, pR6;
		for (Index i = 0; i < 6; i++)
		{
			v6[i] = ev6[i] = 0.1 * (Real)i;
			for (Index j = 0; j < 6; j++) { M6(i, j) = eM6(i, j) = pM6(i, j) = (i == j) ? 0.9 : 0.01 * (Real)(i - j); }
		}
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "ConstSizeMatrix<36> * ConstSizeVector<6>", 6, 1, minimumTime, [&]() { EXUmath::MultMatrixVectorTemplate(M6, v6, r6); v6 = r6; return v6[3]; }));
		results.push_back(LinalgBenchmarkTime(group, "Eigen", "Matrix<6,6> * Matrix<6,1>", 6, 1, minimumTime, [&]() { er6.noalias() = eM6 * ev6; ev6 = er6; return ev6[3]; }));
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "ConstSizeMatrix<36> * ConstSizeMatrix<36>", 6, 1, minimumTime, [&]() { EXUmath::MultMatrixMatrixTemplate(M6, M6, R6); R6 *= 0.5; M6 = R6; return M6(1, 2); }));
		results.push_back(LinalgBenchmarkTime(group, "Eigen", "Matrix<6,6> * Matrix<6,6>", 6, 1, minimumTime, [&]() { eR6.noalias() = eM6 * eM6; eM6 = 0.5 * eR6; return eM6(1, 2); }));
		results.push_back(LinalgBenchmarkTime(group, "prototype", "fixed 6x6 * 6x6", 6, 1, minimumTime, [&]()
			{ LinalgBenchmarkFixedMult(pM6, pM6, pR6); for (Index i = 0; i < 36; i++) { pM6.data[i] = 0.5 * pR6.data[i]; } return pM6(1, 2); }));

		//homogeneous transformations: with rotation and translation only (the no-rotation flag, #3405)
		HomogeneousTransformation H1(A, v), H2(B, w), T1, T2;
		T1.SetTranslation(v);
		T2.SetTranslation(w);
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "HT * v, rotation", 3, 1, minimumTime, [&]() { w = H1 * w; return w[0]; }));
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "HT * HT, rotation", 3, 1, minimumTime, [&]() { H2 = H1 * H2; return H2.GetTranslation()[0]; }));
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "HT inverse, rotation", 3, 1, minimumTime, [&]() { H2 = H2.GetInverse(); return H2.GetTranslation()[1]; }));
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "HT inverse, translation", 3, 1, minimumTime, [&]() { T2 = T2.GetInverse(); return T2.GetTranslation()[1]; }));
	}

	//+++ the Jacobian kernels of the connectors: n = 6 (two 3D points), 7 (EP body), 12 (two rigid bodies), 14 (two EP bodies), 24 (two beam nodes)
	LinalgBenchmarkJacobian<6>(results, minimumTime);
	LinalgBenchmarkJacobian<7>(results, minimumTime);
	LinalgBenchmarkJacobian<12>(results, minimumTime);
	LinalgBenchmarkJacobian<14>(results, minimumTime);
	LinalgBenchmarkJacobian<24>(results, minimumTime);

	//+++ AutoDiff<12>: the analytic connector Jacobians (#2884) differentiate by the 12 variations of the two frames
	{
		typedef EXUmath::AutoDiff<12, Real> DReal12;
		DReal12 x[3], y[3];
		for (Index i = 0; i < 3; i++)
		{
			x[i] = 0.1 * (Real)(i + 1); x[i].DValue((int)i) = 1.;
			y[i] = 0.2 * (Real)(i + 1); y[i].DValue((int)(i + 6)) = 1.;
		}
		Real xr[3] = { 0.1, 0.2, 0.3 }, yr[3] = { 0.2, 0.4, 0.6 };
		results.push_back(LinalgBenchmarkTime("autodiff", "Exudyn", "AutoDiff<12> dot product of 3", 12, 1, minimumTime, [&]()
			{ DReal12 s = x[0] * y[0] + x[1] * y[1] + x[2] * y[2]; x[0] += 1e-12 * s; return s.Value() + s.DValue(11); }));
		results.push_back(LinalgBenchmarkTime("autodiff", "Exudyn", "Real dot product of 3", 1, 1, minimumTime, [&]()
			{ Real s = xr[0] * yr[0] + xr[1] * yr[1] + xr[2] * yr[2]; xr[0] += 1e-12 * s; return s; }));
	}

	//+++ dense: matrix times vector, and the factorization and solve of the dense solver of small systems
	for (Index n : { 6, 12, 24, 48, 100, 200, 500 })
	{
		const std::string group = "dense";
		Matrix K(n, n);
		Vector x(n), y(n);
		Eigen::MatrixXd eK(n, n);
		Eigen::VectorXd ex(n), ey(n);
		for (Index i = 0; i < n; i++)
		{
			x[i] = ex[i] = 1e-3 * (Real)(i % 3);
			for (Index j = 0; j < n; j++) { K(i, j) = eK(i, j) = (i == j) ? 4. : ((std::abs(i - j) < 7) ? -0.3 + 1e-3 * (Real)((i * 7 + j) % 11) : 0.); }
		}
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "Matrix * Vector", n, 1, minimumTime, [&]() { EXUmath::MultMatrixVector(K, x, y); x[0] = y[0] * 1e-3; return y[n - 1]; }));
		results.push_back(LinalgBenchmarkTime(group, "Eigen", "MatrixXd * VectorXd", n, 1, minimumTime, [&]() { ey.noalias() = eK * ex; ex[0] = ey[0] * 1e-3; return ey[n - 1]; }));

		GeneralMatrixEXUdense dense;
		dense.SetMatrix(K);
		Vector rhs(n), solution(n);
		for (Index i = 0; i < n; i++) { rhs[i] = 1e-2 * (Real)(i % 5); }
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "dense factorize (InvertSpecial)", n, 1, minimumTime, [&]()
			{ dense.SetMatrix(K); dense.FactorizeNew(); return 0.; }));
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "dense solve (inverse * rhs)", n, 1, minimumTime, [&]()
			{ dense.Solve(rhs, solution); rhs[0] += 1e-12 * solution[0]; return solution[n - 1]; }));
		Eigen::PartialPivLU<Eigen::MatrixXd> lu(n);
		Eigen::VectorXd erhs(n), esolution(n);
		for (Index i = 0; i < n; i++) { erhs[i] = rhs[i]; }
		results.push_back(LinalgBenchmarkTime(group, "Eigen", "PartialPivLU compute", n, 1, minimumTime, [&]() { lu.compute(eK); return lu.matrixLU()(0, 0); }));
		results.push_back(LinalgBenchmarkTime(group, "Eigen", "PartialPivLU solve", n, 1, minimumTime, [&]()
			{ esolution = lu.solve(erhs); erhs[0] += 1e-12 * esolution[0]; return esolution[n - 1]; }));
	}

	//+++ assembly: the local 12 x 12 Jacobians of a chain of objects on 6 coordinates each into the system matrix
	for (Index nObjects : { 10, 100, 1000 })
	{
		const std::string group = "assembly";
		Index nLocal = 12;
		Index n = 6 * (nObjects + 1);
		ResizableMatrix local(nLocal, nLocal);
		for (Index i = 0; i < nLocal; i++) { for (Index j = 0; j < nLocal; j++) { local(i, j) = (i == j) ? 2. : 0.01 * (Real)(i + j + 1); } }
		std::vector<ArrayIndex> ltg(nObjects);
		for (Index k = 0; k < nObjects; k++)
		{
			ltg[k].SetNumberOfItems(nLocal);
			for (Index i = 0; i < nLocal; i++) { ltg[k][i] = 6 * k + i; }
		}
		GeneralMatrixEigenSparse sparse;
		sparse.SetNumberOfRowsAndColumns(n, n);
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "AddSubmatrix 12x12 into triplets, per call all objects", nObjects, 1, minimumTime, [&]()
			{
				sparse.SetAllZero();
				for (Index k = 0; k < nObjects; k++) { sparse.AddSubmatrix(local, 1., ltg[k], ltg[k]); }
				return local(0, 0);
			}));
		results.push_back(LinalgBenchmarkTime(group, "Exudyn", "AddSubmatrix + FinalizeMatrix (setFromTriplets)", nObjects, 1, minimumTime, [&]()
			{
				sparse.SetAllZero();
				for (Index k = 0; k < nObjects; k++) { sparse.AddSubmatrix(local, 1., ltg[k], ltg[k]); }
				sparse.FinalizeMatrix();
				return local(0, 0);
			}));

		//the same sums into a sparse matrix of known pattern: the positions of the entries computed once
		Eigen::SparseMatrix<Real> pattern(n, n);
		std::vector<Eigen::Triplet<Real>> triplets;
		for (Index k = 0; k < nObjects; k++)
		{
			for (Index i = 0; i < nLocal; i++) { for (Index j = 0; j < nLocal; j++) { triplets.push_back(Eigen::Triplet<Real>((int)ltg[k][i], (int)ltg[k][j], 1.)); } }
		}
		pattern.setFromTriplets(triplets.begin(), triplets.end());
		std::vector<Index> positions(nObjects * nLocal * nLocal);
		for (Index k = 0; k < nObjects; k++)
		{
			for (Index i = 0; i < nLocal; i++)
			{
				for (Index j = 0; j < nLocal; j++)
				{
					positions[(k * nLocal + i) * nLocal + j] = (Index)(&pattern.coeffRef((int)ltg[k][i], (int)ltg[k][j]) - pattern.valuePtr());
				}
			}
		}
		results.push_back(LinalgBenchmarkTime(group, "Eigen", "add 12x12 into a known pattern, per call all objects", nObjects, 1, minimumTime, [&]()
			{
				Real* values = pattern.valuePtr();
				std::fill(values, values + pattern.nonZeros(), 0.);
				const Index* position = positions.data();
				const Real* localData = local.GetDataPointer();
				for (Index k = 0; k < nObjects; k++)
				{
					for (Index i = 0; i < nLocal * nLocal; i++) { values[*position++] += localData[i]; }
				}
				return values[0];
			}));
	}
	return results;
}

#endif
