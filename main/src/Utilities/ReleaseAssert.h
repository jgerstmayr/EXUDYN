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
* 				- weblink: missing
* 				
*
************************************************************************************************ */
#pragma once

#include <assert.h>

//now defined in preprocessor of Release / ReleaseFast
//#define __FAST_EXUDYN_LINALG //use this to avoid any range checks in linalg; TEST: with __FAST_EXUDYN_LINALG: 2.3s time integration of contact problem, without: 2.9s

#ifndef __FAST_EXUDYN_LINALG
	#define __ASSERT_IN_RELEASE_MODE__ //slows down release, but faster than debug mode (for debugging large scale problems)
	#define __EXUDYN_RUNTIME_CHECKS__  //performs several runtime checks, which slows down performance in release or debug mode

	//!check if _checkExpression is true; if no, trow std::exception(_exceptionMessage); _exceptionMessage will be a const char*, e.g. "VectorBase::operator[]: invalid index"
	//!linalg matrix/vector access functions, memory allocation, array classes and solvers will throw exceptions if the errors are not recoverable
	//!this, as a consequence leads to a pybind exception translated to python; the message will be visible in python; for __FAST_EXUDYN_LINALG, no checks are performed
	#define CHECKandTHROW(_checkExpression,_exceptionMessage) ((_checkExpression) ? 0 : throw std::exception(_exceptionMessage))
	#define CHECKandTHROWcond(_checkExpression) ((_checkExpression) ? 0 : throw std::exception("unexpected EXUDYN internal error"))
	//always throw:
	#define CHECKandTHROWstring(_exceptionMessage) (throw std::exception(_exceptionMessage))
#else
	//no checks in __FAST_EXUDYN_LINALG mode
	#define CHECKandTHROW(_checkExpression,_exceptionMessage)
	#define CHECKandTHROWcond(_checkExpression)
	#define CHECKandTHROWstring(_exceptionMessage)
#endif

//#define CHECKandTHROW(_checkExpression,_exceptionMessage) ((_checkExpression) ? 0 : {throw std::exception(_exceptionMessage);}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//a specific flag _MYDEBUG is used as the common _NDEBUG flag does not work in Visual Studio
//use following statements according to msdn.microsoft in order to detect memory leaks and show line number/file where first new to leaked memory has been called
//works only, if dbg_new is used instead of all 'new' commands!
#ifdef _MYDEBUG
	#define dbg_new new ( _NORMAL_BLOCK , __FILE__ , __LINE__ )
	#undef NDEBUG
#else
	#define dbg_new new
	#define NDEBUG //used to avoid range checks e.g. in Eigen
#endif
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#undef release_assert

#ifndef  __ASSERT_IN_RELEASE_MODE__

	#ifndef _MYDEBUG //otherwise, assert(...) does something in release mode ...
		//use CHECKandTHROW instead!!!
		#define release_assert(_Expression) 
	#else
		//use CHECKandTHROW instead!!!
#define release_assert(_Expression) (assert(_Expression))
	#endif
#else  /*__ASSERT_IN_RELEASE_MODE__*/

	#ifdef _MYDEBUG
	//DEBUG:
		//use CHECKandTHROW instead!!!
		#define release_assert(_Expression) (assert(_Expression))
	#else  
	//RELEASE:
		#ifdef _WIN32
			#ifdef  __cplusplus
			extern "C" {
			#endif

				_CRTIMP void __cdecl _wassert(__in_z const wchar_t * _Message, __in_z const wchar_t *_File, __in unsigned _Line);

			#ifdef  __cplusplus
			}
			#endif


			//use CHECKandTHROW instead!!!
			#define release_assert(_Expression) (void)( (!!(_Expression)) || (_wassert(_CRT_WIDE(#_Expression), _CRT_WIDE(__FILE__), __LINE__), 0) )

		#else
			//use CHECKandTHROW instead!!!
			#define release_assert(_Expression) 
		#endif // _WIN32

	#endif  /* _MYDEBUG */

#endif  /* __ASSERT_IN_RELEASE_MODE__ */
