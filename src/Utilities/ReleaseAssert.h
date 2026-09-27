/** ***********************************************************************************************
* @file			ReleaseAssert.h
* @brief		Enable asserts in release mode; which show more information on runtime errors in release mode
* @details		Details:
                - helps to detect index and memory allocation errors for large models
*
* @author		Gerstmayr Johannes
* @date			2010-10-01 (created)
* @date			2018-04-30 (update, Exudyn)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef EXUDYNEXCEPTIONS__H
#define EXUDYNEXCEPTIONS__H

#include <assert.h>
#include <exception>
#include <stdexcept>

//now defined in preprocessor of Release / ReleaseFast
//#define __FAST_EXUDYN_LINALG //use this to avoid any range checks in linalg; TEST: with __FAST_EXUDYN_LINALG: 2.3s time integration of contact problem, without: 2.9s

//gcc cannot call std::exception() ==> use runtime_error
#ifdef _MSC_VER
//#define EXUexception std::exception
#define EXUexception std::runtime_error
#else
#define EXUexception std::runtime_error
#endif

//#define __FAST_EXUDYN_LINALG //defined as preprocessor flags

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//THE KINDS OF ERROR EXUDYN REPORTS (#2516)
//One C++ class per exception class of the exudyn Python module; PybindModule.cpp registers the
//Python side, where each of them derives BOTH from exudyn.ExudynError and from the built-in that
//fits - so 'except exudyn.ExudynError' catches everything Exudyn raises, while an existing
//'except RuntimeError' or 'except IndexError' keeps working unchanged.
//
//They all derive from EXUexception, which is std::runtime_error (above). That is deliberate: every
//existing catch site keeps catching them. It is also the trap of #2432 - a
//'catch (const ExudynError&)' must come BEFORE any 'catch (const EXUexception&)' in the same try
//block, or the base catch takes it first and the type is flattened back to RuntimeError.
//
//Which class a check raises is a property of the CHECK, not of the helper it is written with: the
//same macro states a user's index mistake in one place and an Exudyn invariant in the next.
//
//=> A CLASS ADDED, REMOVED OR RENAMED HERE MUST BE ADDED, REMOVED OR RENAMED IN THE
//   REGISTRATION BLOCK OF src/Pymodules/PybindModule.cpp AS WELL. Only that direction needs
//   saying: a class registered there without one here does not compile, while a class added
//   here and NOT registered there compiles perfectly and arrives in Python as a plain
//   RuntimeError - pybind11 translates it with its built-in std::runtime_error rule, and
//   nothing reports that the new class exists only in C++.
class ExudynError : public EXUexception            //!< the root; never raised directly
{
public:
	//EXUexception is a MACRO for std::runtime_error, so the inheriting-constructor form
	//'using EXUexception::EXUexception' would expand to a doubly qualified name and not compile
	explicit ExudynError(const std::string& message) : EXUexception(message) {}
	explicit ExudynError(const char* message) : EXUexception(message) {}
};

class ExudynModelError : public ExudynError        //!< an illegal model: wrong combination, illegal setting
{
public: using ExudynError::ExudynError;
};

class ExudynSolverError : public ExudynError       //!< the solver cannot continue: singular matrix, no convergence, divergence
{
public: using ExudynError::ExudynError;
};

class ExudynInternalError : public ExudynError     //!< an Exudyn bug; the message is what a developer needs to see in a user's log
{
public: using ExudynError::ExudynError;
};

class ExudynNotImplementedError : public ExudynError //!< the feature or the combination does not exist (yet); not a mistake and not a bug
{
public: using ExudynError::ExudynError;
};

class ExudynIndexError : public ExudynError        //!< an index outside its range
{
public: using ExudynError::ExudynError;
};

class ExudynValueError : public ExudynError        //!< right kind of value, wrong value: size, shape, range
{
public: using ExudynError::ExudynError;
};

class ExudynTypeError : public ExudynError         //!< the object cannot be that parameter at all
{
public: using ExudynError::ExudynError;
};

class ExudynArithmeticError : public ExudynError   //!< division by zero, sqrt of a negative number, ...
{
public: using ExudynError::ExudynError;
};
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef __FAST_EXUDYN_LINALG
	#define __PYTHON_USERFUNCTION_CATCH__  //performs try/catch in all python user functions
	#define __EXUDYN_RUNTIME_CHECKS__  //performs several runtime checks, which slows down performance in release or debug mode

	//!check if _checkExpression is true; if no, trow std::exception(_exceptionMessage); _exceptionMessage will be a const char*, e.g. "VectorBase::operator[]: invalid index"
	//!linalg matrix/vector access functions, memory allocation, array classes and solvers will throw exceptions if the errors are not recoverable
	//!this, as a consequence leads to a pybind exception translated to python; the message will be visible in python; for __FAST_EXUDYN_LINALG, no checks are performed

	//The LAST argument is optional and names the exception class the check raises (#2521):
	//    CHECKandTHROW(index < n, "...")                       ExudynInternalError: an EXUDYN BUG
	//    CHECKandTHROW(index < n, "...", ExudynIndexError)     a user's index mistake, IndexError in Python
	//
	//THE DEFAULT SAYS SOMETHING (#2528, the last move of the mapping).
	//Until every user-facing call site had been read and given a class, the untyped form threw a
	//bare EXUexception, which is std::runtime_error and means nothing. It now throws
	//ExudynInternalError, whose whole meaning is "please report this" - so a check WITHOUT a class
	//is a statement: this condition is an Exudyn invariant and a user cannot break it from Python.
	//That types 1100 sites in Linalg, Utilities and the base-class stubs of System without touching
	//one of them. ExudynInternalError derives from RuntimeError, so every existing
	//"except RuntimeError" keeps working; nothing in Exudyn raises a bare RuntimeError any more.
	//
	//=> If a check IS reachable from Python, give it a class. Leaving it out now labels a user's
	//   mistake an Exudyn bug, which is worse than the untyped state it replaces.
	//The class belongs to the CHECK, not to the helper: the same macro states a user's mistake in
	//one place and an Exudyn invariant in the next, and the measurement of #2520 says CHECKandTHROW
	//is 27% user-facing. Putting the type on the helper would therefore have been wrong in roughly
	//240 places; putting it here costs one token per site.
	//EXU_EXPAND is needed because MSVC's traditional preprocessor passes __VA_ARGS__ as ONE token
	//to the selector macro unless the result is expanded again.
	#define EXU_EXPAND(_x) _x
	#define EXU_SELECT_3RD(_1,_2,_3,_name,...) _name
	#define EXU_SELECT_2ND(_1,_2,_name,...) _name

	#define CHECKandTHROW_2(_checkExpression,_exceptionMessage) ((_checkExpression) ? 0 : throw ExudynInternalError(_exceptionMessage))
	#define CHECKandTHROW_3(_checkExpression,_exceptionMessage,_exceptionClass) ((_checkExpression) ? 0 : throw _exceptionClass(_exceptionMessage))
	#define CHECKandTHROW(...) EXU_EXPAND(EXU_SELECT_3RD(__VA_ARGS__, CHECKandTHROW_3, CHECKandTHROW_2, )(__VA_ARGS__))

	//no message at all, so nothing but an internal error can be meant by it
	#define CHECKandTHROWcond(_checkExpression) ((_checkExpression) ? 0 : throw ExudynInternalError("unexpected EXUDYN internal error"))

	//always throw:
	#define CHECKandTHROWstring_1(_exceptionMessage) (throw ExudynInternalError(_exceptionMessage))
	#define CHECKandTHROWstring_2(_exceptionMessage,_exceptionClass) (throw _exceptionClass(_exceptionMessage))
	#define CHECKandTHROWstring(...) EXU_EXPAND(EXU_SELECT_2ND(__VA_ARGS__, CHECKandTHROWstring_2, CHECKandTHROWstring_1, )(__VA_ARGS__))
#else
	//no checks in __FAST_EXUDYN_LINALG mode
	#define CHECKandTHROW(...)
	#define CHECKandTHROWcond(_checkExpression)
	#define CHECKandTHROWstring(...)
#endif

//add some macro to define unused variable, not trowing compiler warning, especially in case of __FAST_EXUDYN_LINALG where some variables will not be used any more
#define __UNUSED(x) ((void)(true ? 0 : (x)))

#define __EXUDYN_invalid_local_node0 "Object:GetNodeNumber: invalid call to local node number" //workaround to avoid string in object definition file
#define __EXUDYN_invalid_local_node "Object:GetNodeNumber: invalid local node number > 0" //workaround to avoid string in object definition file
#define __EXUDYN_invalid_local_node1 "Object:GetNodeNumber: invalid local node number > 1" //workaround to avoid string in object definition file
#define __EXUDYN_invalid_local_node2 "Object:GetNodeNumber: invalid local node number > 2" //workaround to avoid string in object definition file

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//a specific flag _MYDEBUG is used as the common _NDEBUG flag does not work in Visual Studio
//use following statements according to msdn.microsoft in order to detect memory leaks and show line number/file where first new to leaked memory has been called
//works only, if dbg_new is used instead of all 'new' commands!
#ifdef _MYDEBUG
#define dbg_new new ( _NORMAL_BLOCK , __FILE__ , __LINE__ )
#undef NDEBUG
#else
#define dbg_new new
#ifndef NDEBUG
#define NDEBUG //used to avoid range checks e.g. in Eigen
#endif
#endif
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif //EXUDYNEXCEPTIONS__H
