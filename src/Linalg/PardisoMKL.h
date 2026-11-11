/** ***********************************************************************************************
* @brief		Intel MKL PARDISO as sparse direct solver, loaded at run time from the MKL that the user installed
*               (pip install mkl, or conda); no MKL header, no build dependency, nothing in the wheel (#3431)
* @details		Details:
*               - PardisoLibrary loads mkl_rt once per process and takes pardisoinit, pardiso, MKL_Set_Num_Threads and
*                 MKL_Get_Version_String from it by name; search order: the environment variable EXUDYN_MKL_LIBRARY,
*                 the environment of the running Python (sys.prefix, sys.base_prefix), the system search path
*               - PardisoSolver keeps one PARDISO handle for one matrix: the analysis when the pattern changes, the
*                 numerical factorization, the solve; the compressed column-major matrix of Eigen is passed as it is,
*                 as the compressed rows of the transposed, and solved with the transposed (iparm[11] = 2)
*               - design: docs/dev/design/sparseSolvers.md section 2
*
* @author		Gerstmayr Johannes
* @date			2026-11-11 (generated)
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#ifndef PARDISOMKL__H
#define PARDISOMKL__H

#include <string>
#include <vector>
#include <map>
#include <functional>
#include "Utilities/BasicDefinitions.h"

//! the MKL runtime library, loaded once per process (#3431)
namespace PardisoLibrary
{
	//! load mkl_rt if not yet loaded; returns true if PARDISO is available, otherwise false and the reason in message
	bool Load(std::string& message);
	//! the path or name from which mkl_rt was loaded, empty if not loaded
	std::string LibraryPath();
	//! the version string of MKL, empty if not loaded
	std::string Version();
	//! run a call of MKL on the thread of MKL and wait for it: PARDISO of MKL 2025.0 on Windows changed values that the
	//! calling function kept in registers across the call - the Newton iteration lost its initial residual -, which a
	//! call on another thread cannot do (#3431)
	void RunOnThread(const std::function<void()>& function);
};

//! one PARDISO handle for one real unsymmetric matrix in compressed column storage (#3431)
class PardisoSolver
{
public:
	PardisoSolver();
	~PardisoSolver();
	PardisoSolver(const PardisoSolver&) = delete;
	PardisoSolver& operator=(const PardisoSolver&) = delete;

	//! analysis if the pattern is not within the analyzed one, then the numerical factorization of the n x n matrix in compressed
	//! column storage (Eigen's outer index, inner index and values, zero-based, sorted, compressed);
	//! returns PARDISO's error code, 0 on success
	int Factorize(int n, const int* columnStarts, const int* rowIndices, const double* values,
		int numberOfThreads, const std::map<Index, Index>& parameters);

	//! solve A*x = b with the last factorization; returns PARDISO's error code
	int Solve(const double* rhs, double* solution);

	//! release the memory of PARDISO and the pattern; the next Factorize does the analysis
	void Release();

	//! true if the last Factorize did the analysis (a pattern not within the analyzed one)
	bool PatternChanged() const { return patternChanged; }
	//! number of perturbed pivots of the last factorization (iparm[13])
	int PerturbedPivots() const { return iparm[13]; }
	//! the right-hand side of the last Solve, kept as rhs and solution may be the same vector
	const double* LastRhs() const { return rhsCopy.data(); }
	//! number of iterative refinement steps of the last solve (iparm[6])
	int RefinementSteps() const { return iparm[6]; }

private:
	int Call(int phase, const int* columnStarts, const int* rowIndices, const double* values, double* rhs, double* solution, int nrhs = 1);
	//! release the memory of PARDISO, keep the pattern
	void ReleaseHandle();
	//! the values of the matrix in the analyzed pattern, zero where the matrix has no entry; false if it has one outside it
	bool MapToPattern(int nInit, const int* matrixColumnStarts, const int* matrixRowIndices, const double* matrixValues);
	//! the pattern becomes the union of the analyzed one (if of the same size) and that of the matrix, with its values
	void UnitePattern(int nInit, const int* matrixColumnStarts, const int* matrixRowIndices, const double* matrixValues);

	void* pt[64];		//!< PARDISO's internal handle
	int iparm[64];		//!< PARDISO's parameters
	int n;				//!< size of the analyzed matrix
	bool analyzed;		//!< true if pt holds an analysis
	bool patternChanged;
	std::vector<int> columnStarts;	//!< the analyzed pattern: the union of the patterns factorized since the analysis
	std::vector<int> rowIndices;
	std::vector<double> values;		//!< the values of the last factorization in the analyzed pattern
	std::vector<double> rhsCopy;	//!< the right-hand side of the last solve
	std::vector<double> work;		//!< x of the analysis and the factorization
	std::map<Index, Index> storedParameters; //!< the parameters of the analysis
};

#endif
