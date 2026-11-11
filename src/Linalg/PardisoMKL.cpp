/** ***********************************************************************************************
* @brief		Intel MKL PARDISO loaded at run time (#3431); see PardisoMKL.h
*
* @author		Gerstmayr Johannes
* @date			2026-11-11 (generated)
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#include "Utilities/BasicDefinitions.h"
#include <pybind11/pybind11.h>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <functional>
#include <cstdlib>
#include <cstring>
#include "Linalg/PardisoMKL.h"

namespace py = pybind11;

#ifdef _WIN32
	#ifndef WIN32_LEAN_AND_MEAN
		#define WIN32_LEAN_AND_MEAN
	#endif
	#ifndef NOMINMAX
		#define NOMINMAX
	#endif
	#include <windows.h>
#else
	#include <dlfcn.h>
#endif

//the C interface of MKL (LP64: 32-bit integers), declared here instead of including mkl.h
typedef void (*PardisoInitFunction)(void* pt, const int* mtype, int* iparm);
typedef void (*PardisoFunction)(void* pt, const int* maxfct, const int* mnum, const int* mtype, const int* phase,
	const int* n, const void* a, const int* ia, const int* ja, int* perm, const int* nrhs, int* iparm,
	const int* msglvl, void* b, void* x, int* error);
typedef void (*SetNumThreadsFunction)(int numberOfThreads);
typedef void (*GetVersionStringFunction)(char* buffer, int length);

namespace PardisoLibrary
{
	static std::mutex loadMutex;
	static bool loadTried = false;
	static bool loaded = false;
	static std::string loadMessage;
	static std::string libraryPath;
	static std::string version;
	PardisoInitFunction pardisoinit = nullptr;
	PardisoFunction pardiso = nullptr;
	SetNumThreadsFunction setNumThreads = nullptr;

	//! the candidates of the search order: the environment variable, the environment of Python, the system search path
	static std::vector<std::string> Candidates()
	{
		std::vector<std::string> candidates;
		const char* variable = std::getenv("EXUDYN_MKL_LIBRARY");
		if (variable && std::strlen(variable)) { candidates.push_back(variable); }

#if defined(_WIN32)
		std::vector<std::string> inPrefix = { "/Library/bin/mkl_rt.2.dll", "/Library/bin/mkl_rt.dll" };
		std::vector<std::string> names = { "mkl_rt.2.dll", "mkl_rt.dll" };
#elif defined(__APPLE__)
		std::vector<std::string> inPrefix = { "/lib/libmkl_rt.2.dylib", "/lib/libmkl_rt.dylib" };
		std::vector<std::string> names = { "libmkl_rt.2.dylib", "libmkl_rt.dylib" };
#else
		std::vector<std::string> inPrefix = { "/lib/libmkl_rt.so.2", "/lib/libmkl_rt.so" };
		std::vector<std::string> names = { "libmkl_rt.so.2", "libmkl_rt.so" };
#endif
		try
		{
			py::gil_scoped_acquire gil;
			py::module_ sys = py::module_::import("sys");
			std::vector<std::string> prefixes = { py::str(sys.attr("prefix")), py::str(sys.attr("base_prefix")) };
			if (prefixes[1] == prefixes[0]) { prefixes.pop_back(); }
			for (const std::string& prefix : prefixes)
			{
				for (const std::string& name : inPrefix) { candidates.push_back(prefix + name); }
			}
		}
		catch (...) {} //without Python (unit tests), only the variable and the system search path

		for (const std::string& name : names) { candidates.push_back(name); }
		return candidates;
	}

#if defined(_WIN32)
	static void* OpenLibrary(const std::string& name)
	{
		int size = MultiByteToWideChar(CP_UTF8, 0, name.c_str(), -1, nullptr, 0);
		std::wstring wideName(size, L'\0');
		MultiByteToWideChar(CP_UTF8, 0, name.c_str(), -1, &wideName[0], size);
		bool isPath = name.find('/') != std::string::npos || name.find('\\') != std::string::npos;
		//a path: the dependent libraries of MKL (mkl_core, the threading layer, libiomp5md) are searched in its directory
		DWORD flags = isPath ? (LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_DEFAULT_DIRS) : 0;
		return (void*)LoadLibraryExW(wideName.c_str(), nullptr, flags);
	}
	static void* Symbol(void* library, const char* name) { return (void*)GetProcAddress((HMODULE)library, name); }
#else
	static void* OpenLibrary(const std::string& name) { return dlopen(name.c_str(), RTLD_NOW | RTLD_LOCAL); }
	static void* Symbol(void* library, const char* name) { return dlsym(library, name); }
#endif

	bool Load(std::string& message)
	{
		std::lock_guard<std::mutex> lock(loadMutex);
		if (!loadTried)
		{
			loadTried = true;
			std::vector<std::string> candidates = Candidates();
			std::string tried;
			for (const std::string& candidate : candidates)
			{
				void* library = OpenLibrary(candidate);
				if (!library) { tried += "\n  " + candidate; continue; }
				pardisoinit = (PardisoInitFunction)Symbol(library, "pardisoinit");
				pardiso = (PardisoFunction)Symbol(library, "pardiso");
				setNumThreads = (SetNumThreadsFunction)Symbol(library, "MKL_Set_Num_Threads");
				GetVersionStringFunction getVersion = (GetVersionStringFunction)Symbol(library, "MKL_Get_Version_String");
				if (!pardisoinit || !pardiso || !setNumThreads)
				{
					tried += "\n  " + candidate + " (loaded, but without pardisoinit, pardiso or MKL_Set_Num_Threads)";
					continue;
				}
				if (getVersion)
				{
					char buffer[256] = { 0 };
					getVersion(buffer, 255);
					version = buffer;
				}
				libraryPath = candidate;
				loaded = true;
				break;
			}
			if (!loaded)
			{
				loadMessage = "Intel MKL PARDISO is not available: the MKL runtime library mkl_rt was not found; tried:" + tried +
					"\ninstall MKL into the Python environment (pip install mkl, or conda install mkl) or set the environment "
					"variable EXUDYN_MKL_LIBRARY to the full path of mkl_rt; MKL exists for x86-64 only";
			}
		}
		message = loadMessage;
		return loaded;
	}

	std::string LibraryPath() { return libraryPath; }
	std::string Version() { return version; }

	//! the state of the thread of MKL; allocated once and never freed, so that no destructor runs while the thread waits
	struct ThreadState
	{
		std::mutex callMutex;		//!< one call at a time, also from several Python threads
		std::mutex taskMutex;
		std::condition_variable taskCondition;
		const std::function<void()>* task = nullptr;
		bool taskDone = false;
		bool started = false;
	};
	static ThreadState* threadState = new ThreadState();

	void RunOnThread(const std::function<void()>& function)
	{
		ThreadState& state = *threadState;
		std::lock_guard<std::mutex> callLock(state.callMutex);
		std::unique_lock<std::mutex> lock(state.taskMutex);
		if (!state.started)
		{
			state.started = true;
			std::thread([]() {
				ThreadState& state = *threadState;
				std::unique_lock<std::mutex> lock(state.taskMutex);
				while (true)
				{
					state.taskCondition.wait(lock, [&state] { return state.task != nullptr; });
					(*state.task)();
					state.task = nullptr;
					state.taskDone = true;
					state.taskCondition.notify_all();
				}
				}).detach();
		}
		state.task = &function;
		state.taskDone = false;
		state.taskCondition.notify_all();
		state.taskCondition.wait(lock, [&state] { return state.taskDone; });
	}
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
static const int pardisoMatrixType = 11; //real and unsymmetric

PardisoSolver::PardisoSolver()
{
	for (Index i = 0; i < 64; i++) { pt[i] = nullptr; iparm[i] = 0; }
	n = 0;
	analyzed = false;
	patternChanged = false;
}

PardisoSolver::~PardisoSolver()
{
	Release();
}

int PardisoSolver::Call(int phase, const int* columnStarts, const int* rowIndices, const double* values, double* rhs, double* solution, int nrhs)
{
	int maxfct = 1, mnum = 1, msglvl = 0, error = 0;
	PardisoLibrary::RunOnThread([&]() {
		PardisoLibrary::pardiso(pt, &maxfct, &mnum, &pardisoMatrixType, &phase, &n, values, columnStarts, rowIndices,
			nullptr, &nrhs, iparm, &msglvl, rhs, solution, &error);
		});
	return error;
}

void PardisoSolver::ReleaseHandle()
{
	if (analyzed && PardisoLibrary::pardiso)
	{
		//the arrays of the analysis and b, x of the size of the matrix: no phase may get a pointer to a single number
		rhsCopy.assign(n, 0.);
		work.assign(n, 0.);
		Call(-1, columnStarts.data(), rowIndices.data(), values.data(), rhsCopy.data(), work.data());
	}
	analyzed = false;
}

void PardisoSolver::Release()
{
	ReleaseHandle();
	columnStarts.clear();
	rowIndices.clear();
	values.clear();
}

bool PardisoSolver::MapToPattern(int nInit, const int* matrixColumnStarts, const int* matrixRowIndices, const double* matrixValues)
{
	if (nInit != n || (int)columnStarts.size() != n + 1) { return false; }
	values.assign(rowIndices.size(), 0.);
	for (int j = 0; j < n; j++)
	{
		int k = columnStarts[j];
		int kEnd = columnStarts[j + 1];
		for (int i = matrixColumnStarts[j]; i < matrixColumnStarts[j + 1]; i++)
		{
			while (k < kEnd && rowIndices[k] < matrixRowIndices[i]) { k++; }
			if (k == kEnd || rowIndices[k] != matrixRowIndices[i]) { return false; }
			values[k++] = matrixValues[i];
		}
	}
	return true;
}

void PardisoSolver::UnitePattern(int nInit, const int* matrixColumnStarts, const int* matrixRowIndices, const double* matrixValues)
{
	bool keep = (nInit == n && (int)columnStarts.size() == n + 1); //the old pattern is kept in the union
	std::vector<int> oldStarts, oldRows;
	if (keep) { oldStarts.swap(columnStarts); oldRows.swap(rowIndices); }
	n = nInit;
	columnStarts.assign(n + 1, 0);
	rowIndices.clear();
	values.clear();
	rowIndices.reserve(matrixColumnStarts[n] + (keep ? oldRows.size() / 8 : 0));
	values.reserve(rowIndices.capacity());
	for (int j = 0; j < n; j++)
	{
		int i = matrixColumnStarts[j], iEnd = matrixColumnStarts[j + 1];
		int k = keep ? oldStarts[j] : 0, kEnd = keep ? oldStarts[j + 1] : 0;
		while (i < iEnd || k < kEnd)
		{
			if (k == kEnd || (i < iEnd && matrixRowIndices[i] < oldRows[k])) { rowIndices.push_back(matrixRowIndices[i]); values.push_back(matrixValues[i]); i++; }
			else if (i == iEnd || oldRows[k] < matrixRowIndices[i]) { rowIndices.push_back(oldRows[k]); values.push_back(0.); k++; }
			else { rowIndices.push_back(matrixRowIndices[i]); values.push_back(matrixValues[i]); i++; k++; }
		}
		columnStarts[j + 1] = (int)rowIndices.size();
	}
}

int PardisoSolver::Factorize(int nInit, const int* matrixColumnStarts, const int* matrixRowIndices, const double* matrixValues,
	int numberOfThreads, const std::map<Index, Index>& parameters)
{
	std::string message;
	if (!PardisoLibrary::Load(message)) { PyError(message, PyErrorType::valueError); } //also in the fast module, which has no CHECKandTHROW

	//entries that are exactly zero are not stored by Exudyn and come and go between Jacobians; the analysis is kept for the
	//union of all patterns seen, and a matrix within it is factorized with zeros in the missing entries
	if (parameters != storedParameters || nInit != n) { Release(); }
	patternChanged = !analyzed || !MapToPattern(nInit, matrixColumnStarts, matrixRowIndices, matrixValues);

	if (numberOfThreads > 0) { PardisoLibrary::RunOnThread([numberOfThreads]() { PardisoLibrary::setNumThreads(numberOfThreads); }); }

	if (patternChanged)
	{
		ReleaseHandle();
		UnitePattern(nInit, matrixColumnStarts, matrixRowIndices, matrixValues);
		for (Index i = 0; i < 64; i++) { pt[i] = nullptr; iparm[i] = 0; }
		PardisoLibrary::RunOnThread([this]() { PardisoLibrary::pardisoinit(pt, &pardisoMatrixType, iparm); });
		//the defaults of Exudyn (sparseSolvers.md section 3): minimum degree ordering and the one-level factorization were
		//reliable on all Jacobians studied; METIS with the two-level factorization failed or crashed on the chain of 1000 bodies
		iparm[1] = 0;	//ordering: minimum degree
		iparm[23] = 0;	//one-level factorization
		for (const auto& item : parameters)
		{
			if (item.first < 0 || item.first >= 64) { PyError("exudyn.special.solver.pardisoParameters: the index must be in 0 ... 63, got " + std::to_string(item.first), PyErrorType::valueError); }
			iparm[item.first] = (int)item.second;
		}
		iparm[11] = 2;	//the compressed columns of A are the compressed rows of A^T: solve with the transposed
		iparm[34] = 1;	//zero-based indices
		storedParameters = parameters;
		rhsCopy.assign(n, 0.);
		work.assign(n, 0.);

		int error = Call(11, columnStarts.data(), rowIndices.data(), values.data(), rhsCopy.data(), work.data());
		if (error != 0) { Release(); return error; }
		analyzed = true;
	}
	//b and x of the size of the matrix also in the analysis and the factorization: PARDISO may write into them
	rhsCopy.assign(n, 0.);
	work.assign(n, 0.);
	return Call(22, columnStarts.data(), rowIndices.data(), values.data(), rhsCopy.data(), work.data());
}

int PardisoSolver::Solve(const double* rhs, double* solution)
{
	if (!analyzed) { SysError("PardisoSolver::Solve: the matrix is not factorized"); }
	rhsCopy.assign(rhs, rhs + n); //rhs and solution may be the same vector, which PARDISO does not allow
	return Call(33, columnStarts.data(), rowIndices.data(), values.data(), rhsCopy.data(), solution);
}
