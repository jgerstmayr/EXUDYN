/** ***********************************************************************************************
* @brief		The transcendental functions of the math library by name, evaluated the way the C++ code of Exudyn calls
*				them; used by exudyn.experimental.EvaluateMathFunction to compare the math libraries of the platforms
*				(#2947): glibc, the MSVC UCRT and Apple's libm round some results differently in the last bit
*
* @author		Gerstmayr Johannes
* @date			2026-10-10 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
************************************************************************************************ */
#ifndef MATHFUNCTIONS__H
#define MATHFUNCTIONS__H

#include <cmath>
#include <chrono>
#include <string>
#include <vector>
#include <type_traits>
#include "Utilities/BasicDefinitions.h"

//! a loop the compiler shall not vectorize: one call per value, as the code calls the functions at one point;
//! MSVC would call its vector math library in a vectorized loop, which rounds differently
#if defined(_MSC_VER)
#define EXU_LOOP_NO_VECTOR __pragma(loop(no_vector))
#elif defined(__clang__)
#define EXU_LOOP_NO_VECTOR _Pragma("clang loop vectorize(disable)")
#else
#define EXU_LOOP_NO_VECTOR //gcc does not vectorize calls of the math library without -ffast-math
#endif

namespace EXUmath {

	//! the functions of musl libc in PortableMath.cpp: the same bits on every platform, as they use only +, -, *, /
	//! and sqrt without contraction into a fused multiply-add (#2948)
	namespace Portable {
		double Sin(double x);
		double Cos(double x);
		double Tan(double x);
		double Asin(double x);
		double Acos(double x);
		double Atan(double x);
		double Atan2(double y, double x);
		double Exp(double x);
		double Log(double x);
		double Pow(double x, double y);
		double Sinh(double x);
		double Cosh(double x);
		double Tanh(double x);
		double Cbrt(double x);
	} //namespace Portable

	//! THE TRANSCENDENTAL FUNCTIONS OF EXUDYN: the C++ code calls EXUmath::Sin, ... instead of std::sin, ..., so that
	//! one switch decides where they come from (#2949): the C++ library of the platform, or with EXUDYN_PORTABLE_MATH
	//! (BasicDefinitions.h) the functions of Portable, which give the same bits on every platform. A double argument
	//! goes to these; any other type - AutoDiff, a symbolic expression - to its own overload, found by argument
	//! dependent lookup; fmod and sqrt are exact or correctly rounded everywhere and stay std::fmod, std::sqrt
#ifdef EXUDYN_PORTABLE_MATH
#define EXU_MATH_FUNCTION1(NAME, STDNAME) inline double NAME(double x) { return Portable::NAME(x); }
#define EXU_MATH_FUNCTION2(NAME, STDNAME) inline double NAME(double x, double y) { return Portable::NAME(x, y); }
#else
#define EXU_MATH_FUNCTION1(NAME, STDNAME) inline double NAME(double x) { return std::STDNAME(x); }
#define EXU_MATH_FUNCTION2(NAME, STDNAME) inline double NAME(double x, double y) { return std::STDNAME(x, y); }
#endif
#define EXU_MATH_GENERIC1(NAME, STDNAME) template<class T, typename std::enable_if<!std::is_arithmetic<T>::value, int>::type = 0> 	inline auto NAME(const T& x) { using std::STDNAME; return STDNAME(x); }
#define EXU_MATH_GENERIC2(NAME, STDNAME) template<class T1, class T2, typename std::enable_if<!std::is_arithmetic<T1>::value || 	!std::is_arithmetic<T2>::value, int>::type = 0> inline auto NAME(const T1& x, const T2& y) { using std::STDNAME; return STDNAME(x, y); }

	EXU_MATH_FUNCTION1(Sin, sin)		EXU_MATH_GENERIC1(Sin, sin)
	EXU_MATH_FUNCTION1(Cos, cos)		EXU_MATH_GENERIC1(Cos, cos)
	EXU_MATH_FUNCTION1(Tan, tan)		EXU_MATH_GENERIC1(Tan, tan)
	EXU_MATH_FUNCTION1(Asin, asin)		EXU_MATH_GENERIC1(Asin, asin)
	EXU_MATH_FUNCTION1(Acos, acos)		EXU_MATH_GENERIC1(Acos, acos)
	EXU_MATH_FUNCTION1(Atan, atan)		EXU_MATH_GENERIC1(Atan, atan)
	EXU_MATH_FUNCTION2(Atan2, atan2)	EXU_MATH_GENERIC2(Atan2, atan2)
	EXU_MATH_FUNCTION1(Exp, exp)		EXU_MATH_GENERIC1(Exp, exp)
	EXU_MATH_FUNCTION1(Log, log)		EXU_MATH_GENERIC1(Log, log)
	EXU_MATH_FUNCTION2(Pow, pow)		EXU_MATH_GENERIC2(Pow, pow)
	EXU_MATH_FUNCTION1(Sinh, sinh)		EXU_MATH_GENERIC1(Sinh, sinh)
	EXU_MATH_FUNCTION1(Cosh, cosh)		EXU_MATH_GENERIC1(Cosh, cosh)
	EXU_MATH_FUNCTION1(Tanh, tanh)		EXU_MATH_GENERIC1(Tanh, tanh)
	EXU_MATH_FUNCTION1(Cbrt, cbrt)		EXU_MATH_GENERIC1(Cbrt, cbrt)
#undef EXU_MATH_FUNCTION1
#undef EXU_MATH_FUNCTION2
#undef EXU_MATH_GENERIC1
#undef EXU_MATH_GENERIC2

	//! the names EvaluateMathFunction knows; atan2, pow and fmod take two arguments; sqrt is correctly rounded on every
	//! platform (IEEE 754) and is the control
	inline const std::vector<std::string>& MathFunctionNames()
	{
		static const std::vector<std::string> names = { "sin", "cos", "tan", "asin", "acos", "atan", "atan2", "exp", "log",
			"pow", "sinh", "cosh", "tanh", "cbrt", "fmod", "sqrt" };
		return names;
	}

	//! the implementations EvaluateMathFunction knows: "std" = the C++ library, one call per value; "stdLoop" = the same in
	//! a loop the compiler may vectorize; "stdNoFMA3" = "std" with the FMA3 code path of the MSVC UCRT switched off
	//! (Windows x64 only: the UCRT chooses its implementation by the CPU, _set_FMA3_enable); "portable" = the functions
	//! of namespace Portable, with std::fmod and std::sqrt, which are exact or correctly rounded everywhere (#2948)
	inline const std::vector<std::string>& MathFunctionImplementations()
	{
		static const std::vector<std::string> names = { "std", "stdLoop", "portable"
#if defined(_MSC_VER) && defined(_M_X64)
			, "stdNoFMA3"
#endif
		};
		return names;
	}

	namespace MathFunctionsInternal {
		template<class F> inline void LoopScalar(F f, const std::vector<Real>& x, std::vector<Real>& result)
		{
			const size_t n = x.size();
			EXU_LOOP_NO_VECTOR
			for (size_t i = 0; i < n; i++) { result[i] = f(x[i]); }
		}
		template<class F> inline void LoopScalar2(F f, const std::vector<Real>& x, const std::vector<Real>& y, std::vector<Real>& result)
		{
			const size_t n = x.size();
			EXU_LOOP_NO_VECTOR
			for (size_t i = 0; i < n; i++) { result[i] = f(x[i], y[i]); }
		}
		template<class F> inline void LoopPlain(F f, const std::vector<Real>& x, std::vector<Real>& result)
		{
			const size_t n = x.size();
			for (size_t i = 0; i < n; i++) { result[i] = f(x[i]); }
		}
		template<class F> inline void LoopPlain2(F f, const std::vector<Real>& x, const std::vector<Real>& y, std::vector<Real>& result)
		{
			const size_t n = x.size();
			for (size_t i = 0; i < n; i++) { result[i] = f(x[i], y[i]); }
		}

		//! evaluate with the library; scalar = one call per value, not vectorized
		inline bool EvaluateStd(const std::string& name, const std::vector<Real>& x, const std::vector<Real>& y,
			std::vector<Real>& result, bool scalar)
		{
#define EXU_MATH_CASE1(NAME, CALL) if (name == NAME) { auto f = [](Real a) { return CALL; }; \
			if (scalar) { LoopScalar(f, x, result); } else { LoopPlain(f, x, result); } return true; }
#define EXU_MATH_CASE2(NAME, CALL) if (name == NAME) { auto f = [](Real a, Real b) { return CALL; }; \
			if (scalar) { LoopScalar2(f, x, y, result); } else { LoopPlain2(f, x, y, result); } return true; }
			EXU_MATH_CASE1("sin", std::sin(a))
			EXU_MATH_CASE1("cos", std::cos(a))
			EXU_MATH_CASE1("tan", std::tan(a))
			EXU_MATH_CASE1("asin", std::asin(a))
			EXU_MATH_CASE1("acos", std::acos(a))
			EXU_MATH_CASE1("atan", std::atan(a))
			EXU_MATH_CASE2("atan2", std::atan2(a, b))
			EXU_MATH_CASE1("exp", std::exp(a))
			EXU_MATH_CASE1("log", std::log(a))
			EXU_MATH_CASE2("pow", std::pow(a, b))
			EXU_MATH_CASE1("sinh", std::sinh(a))
			EXU_MATH_CASE1("cosh", std::cosh(a))
			EXU_MATH_CASE1("tanh", std::tanh(a))
			EXU_MATH_CASE1("cbrt", std::cbrt(a))
			EXU_MATH_CASE2("fmod", std::fmod(a, b))
			EXU_MATH_CASE1("sqrt", std::sqrt(a))
#undef EXU_MATH_CASE1
#undef EXU_MATH_CASE2
			return false;
		}

		//! evaluate with the portable functions, one call per value
		inline bool EvaluatePortable(const std::string& name, const std::vector<Real>& x, const std::vector<Real>& y,
			std::vector<Real>& result)
		{
#define EXU_MATH_CASE1(NAME, CALL) if (name == NAME) { LoopScalar([](Real a) { return CALL; }, x, result); return true; }
#define EXU_MATH_CASE2(NAME, CALL) if (name == NAME) { LoopScalar2([](Real a, Real b) { return CALL; }, x, y, result); return true; }
			EXU_MATH_CASE1("sin", Portable::Sin(a))
			EXU_MATH_CASE1("cos", Portable::Cos(a))
			EXU_MATH_CASE1("tan", Portable::Tan(a))
			EXU_MATH_CASE1("asin", Portable::Asin(a))
			EXU_MATH_CASE1("acos", Portable::Acos(a))
			EXU_MATH_CASE1("atan", Portable::Atan(a))
			EXU_MATH_CASE2("atan2", Portable::Atan2(a, b))
			EXU_MATH_CASE1("exp", Portable::Exp(a))
			EXU_MATH_CASE1("log", Portable::Log(a))
			EXU_MATH_CASE2("pow", Portable::Pow(a, b))
			EXU_MATH_CASE1("sinh", Portable::Sinh(a))
			EXU_MATH_CASE1("cosh", Portable::Cosh(a))
			EXU_MATH_CASE1("tanh", Portable::Tanh(a))
			EXU_MATH_CASE1("cbrt", Portable::Cbrt(a))
			EXU_MATH_CASE2("fmod", std::fmod(a, b))
			EXU_MATH_CASE1("sqrt", std::sqrt(a))
#undef EXU_MATH_CASE1
#undef EXU_MATH_CASE2
			return false;
		}
	} //namespace MathFunctionsInternal

	//! evaluate the function 'name' (MathFunctionNames) with the implementation (MathFunctionImplementations) for every
	//! x[i] - and y[i] for atan2, pow, fmod - into result, 'repetitions' times; returns the time per evaluation in
	//! seconds, or a negative value for an unknown name or implementation and for y not of the size of x
	inline Real EvaluateMathFunction(const std::string& name, const std::vector<Real>& x, const std::vector<Real>& y,
		const std::string& implementation, Index repetitions, std::vector<Real>& result)
	{
		const bool twoArguments = (name == "atan2" || name == "pow" || name == "fmod");
		if (twoArguments && y.size() != x.size()) { return -1; }
		result.assign(x.size(), 0.);
		if (repetitions < 1) { repetitions = 1; }

		bool scalar = true;
		const bool portable = (implementation == "portable");
		if (implementation == "stdLoop") { scalar = false; }
		else if (implementation != "std" && implementation != "stdNoFMA3" && !portable) { return -1; }
#if defined(_MSC_VER) && defined(_M_X64)
		const int fma3Enabled = _get_FMA3_enable();
		if (implementation == "stdNoFMA3") { _set_FMA3_enable(0); }
#else
		if (implementation == "stdNoFMA3") { return -1; }
#endif

		bool known = true;
		auto start = std::chrono::steady_clock::now();
		for (Index i = 0; i < repetitions && known; i++)
		{
			known = portable ? MathFunctionsInternal::EvaluatePortable(name, x, y, result)
				: MathFunctionsInternal::EvaluateStd(name, x, y, result, scalar);
		}
		auto end = std::chrono::steady_clock::now();

#if defined(_MSC_VER) && defined(_M_X64)
		_set_FMA3_enable(fma3Enabled);
#endif
		if (!known) { return -1; }
		Real seconds = std::chrono::duration<Real>(end - start).count();
		return x.size() ? seconds / ((Real)x.size() * (Real)repetitions) : 0.;
	}

} //namespace EXUmath

#endif
