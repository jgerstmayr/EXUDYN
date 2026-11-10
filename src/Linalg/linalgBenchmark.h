/** ***********************************************************************************************
* @file			linalgBenchmark.h
* @brief		The linear algebra micro-benchmark that every build carries: exudyn.special.RunLinalgBenchmark()
* @details		Details:
*				- times the operations the solvers spend their time in, with the compiler options of the build:
*				  Vector add, scale and MultAdd over a sweep of sizes across ResizableVectorParallelThreadingLimit,
*				  ResizableVectorParallel single- and multithreaded, the products of SlimVector, Matrix3D,
*				  ConstSizeMatrix and HomogeneousTransformation, and the product of a Matrix with a Vector
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
#include "Utilities/Threading.h"

//! one result of the benchmark: the operation, the size of the vectors or matrices, the threads, ns per call
struct LinalgBenchmarkResult
{
	std::string operation;
	Index size;
	Index threads;
	Real nsPerCall;
	Real checkSum;
};

//! run operation (which returns a number for the check sum) repeatedly for at least minimumTime seconds; ns per call
template<class TFunction>
inline LinalgBenchmarkResult LinalgBenchmarkTime(const std::string& operation, Index size, Index threads, Real minimumTime, TFunction operation_)
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
	return LinalgBenchmarkResult({ operation, size, threads, 1e9 * elapsed / (Real)calls, checkSum });
}

//! the benchmark; numberOfThreads > 1 adds the multithreaded ResizableVectorParallel runs
inline std::vector<LinalgBenchmarkResult> RunLinalgBenchmark(Real minimumTime, Index numberOfThreads)
{
	std::vector<LinalgBenchmarkResult> results;

	//+++ vectors over a sweep of sizes, across the limit of multithreading
	Index limit = ResizableVectorParallelThreadingLimit;
	std::vector<Index> sizes = { 3, 10, 100, 1000, 10000, limit, 10 * limit, 50 * limit };
	for (Index n : sizes)
	{
		Vector a(n), b(n);
		for (Index i = 0; i < n; i++) { a[i] = 1e-3 * (Real)(i % 7); b[i] = 1e-3 * (Real)(i % 5); }
		results.push_back(LinalgBenchmarkTime("Vector +=", n, 1, minimumTime, [&]() { a += b; return a[0]; }));
		results.push_back(LinalgBenchmarkTime("Vector *= scalar", n, 1, minimumTime, [&]() { a *= 0.999; return a[n - 1]; }));
		results.push_back(LinalgBenchmarkTime("Vector MultAdd", n, 1, minimumTime, [&]() { a.MultAdd(1e-3, b); return a[0]; }));

		//ResizableVectorParallel: multithreaded above the limit, if the threads run
		std::vector<Index> threadList = { 1 };
		if (numberOfThreads > 1 && n >= limit) { threadList.push_back(numberOfThreads); }
		for (Index threads : threadList)
		{
			if (threads > 1)
			{
				ExuThreading::TaskManager::SetNumThreads(threads);
				ExuThreading::EnterTaskManager();
			}
			Index threadsUsed = ExuThreading::TaskManager::GetNumThreads();
			ResizableVectorParallel c(n), d(n);
			for (Index i = 0; i < n; i++) { c[i] = 1e-3 * (Real)(i % 7); d[i] = 1e-3 * (Real)(i % 5); }
			results.push_back(LinalgBenchmarkTime("ResizableVectorParallel +=", n, threadsUsed, minimumTime, [&]() { c += d; return c[0]; }));
			results.push_back(LinalgBenchmarkTime("ResizableVectorParallel MultAdd", n, threadsUsed, minimumTime, [&]() { c.MultAdd(1e-3, d); return c[0]; }));
			results.push_back(LinalgBenchmarkTime("ResizableVectorParallel CopyFrom", n, threadsUsed, minimumTime, [&]() { c.CopyFrom(d); return c[n - 1]; }));
			if (threads > 1)
			{
				ExuThreading::ExitTaskManager(threads);
				ExuThreading::TaskManager::SetNumThreads(1);
			}
		}
	}

	//+++ small fixed-size types
	Vector3D v({ 0.3, -0.2, 0.7 }), w({ -0.1, 0.5, 0.2 });
	Matrix3D A(3, 3, { 0.36, 0.48, -0.8, -0.8, 0.6, 0., 0.48, 0.64, 0.6 });
	Matrix3D B = A.GetTransposed();
	results.push_back(LinalgBenchmarkTime("SlimVector<3> +", 3, 1, minimumTime, [&]() { v = v + w * 1e-6; return v[0]; }));
	results.push_back(LinalgBenchmarkTime("SlimVector<3> cross product", 3, 1, minimumTime, [&]() { v = v.CrossProduct(w) * 0.5 + w; return v[1]; }));
	results.push_back(LinalgBenchmarkTime("Matrix3D * Vector3D", 3, 1, minimumTime, [&]() { v = A * v; return v[2]; }));
	results.push_back(LinalgBenchmarkTime("Matrix3D * Matrix3D", 3, 1, minimumTime, [&]() { B = A * B; return B(0, 1); }));

	ConstSizeMatrix<36> M6(6, 6);
	ConstSizeVector<6> v6;
	for (Index i = 0; i < 6; i++) { v6[i] = 0.1 * (Real)i; for (Index j = 0; j < 6; j++) { M6(i, j) = (i == j) ? 0.9 : 0.01 * (Real)(i - j); } }
	ConstSizeVector<6> r6;
	results.push_back(LinalgBenchmarkTime("ConstSizeMatrix<36> * ConstSizeVector<6>", 6, 1, minimumTime, [&]() { EXUmath::MultMatrixVectorTemplate(M6, v6, r6); v6 = r6; return v6[3]; }));
	results.push_back(LinalgBenchmarkTime("ConstSizeMatrix<36> * ConstSizeMatrix<36>", 6, 1, minimumTime, [&]() { M6 = M6 * M6; M6 *= 0.5; return M6(1, 2); }));

	//+++ homogeneous transformations: with rotation and translation only (the no-rotation flag, #3405)
	HomogeneousTransformation H1(A, v), H2(B, w), T1, T2;
	T1.SetTranslation(v);
	T2.SetTranslation(w);
	results.push_back(LinalgBenchmarkTime("HT * v, rotation", 3, 1, minimumTime, [&]() { w = H1 * w; return w[0]; }));
	results.push_back(LinalgBenchmarkTime("HT * v, translation", 3, 1, minimumTime, [&]() { w = T1 * w; w *= 0.5; return w[0]; }));
	results.push_back(LinalgBenchmarkTime("HT * HT, rotation", 3, 1, minimumTime, [&]() { H2 = H1 * H2; return H2.GetTranslation()[0]; }));
	results.push_back(LinalgBenchmarkTime("HT * HT, translation", 3, 1, minimumTime, [&]() { T2 = T1 * T2; T2.GetTranslation() *= 0.5; return T2.GetTranslation()[0]; }));
	results.push_back(LinalgBenchmarkTime("HT inverse, rotation", 3, 1, minimumTime, [&]() { H2 = H2.GetInverse(); return H2.GetTranslation()[1]; }));
	results.push_back(LinalgBenchmarkTime("HT inverse, translation", 3, 1, minimumTime, [&]() { T2 = T2.GetInverse(); return T2.GetTranslation()[1]; }));

	//+++ dense matrix times vector
	for (Index n : { 10, 100, 1000 })
	{
		Matrix K(n, n);
		Vector x(n), y(n);
		for (Index i = 0; i < n; i++) { x[i] = 1e-3 * (Real)(i % 3); for (Index j = 0; j < n; j++) { K(i, j) = (i == j) ? 1. : 1e-4; } }
		results.push_back(LinalgBenchmarkTime("Matrix * Vector", n, 1, minimumTime, [&]() { EXUmath::MultMatrixVector(K, x, y); x[0] = y[0] * 1e-3; return y[n - 1]; }));
	}
	return results;
}

#endif
