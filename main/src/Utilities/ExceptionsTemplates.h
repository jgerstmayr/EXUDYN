/** ***********************************************************************************************
* @file			ExceptionTemplates.h
* @brief		This file contains templates and functions for simple handling of exceptions
*
* @author		Gerstmayr Johannes
* @date			2020-04-25 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef EXCEPTIONTEMPLATES__H
#define EXCEPTIONTEMPLATES__H

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h" //defines Real
#include <pybind11/pybind11.h>
namespace py = pybind11;            //! namespace 'py' used throughout in code

//#undef __PYTHON_USERFUNCTION_CATCH__

template <typename Tfunction>
//void UserFunctionExceptionHandling(Tfunction&& f, STDstring functionName)
void UserFunctionExceptionHandling(Tfunction&& f, const char* functionName)
{
#ifdef __PYTHON_USERFUNCTION_CATCH__
	try
	{
		f();
	}
	//mostly catches python errors:
	catch (const pybind11::error_already_set& ex)
	{
		SysError("Error in python USER FUNCTION '" + STDstring(functionName) + "' (referred line number my be wrong!):\n" + STDstring(ex.what()) + "; check your python code!");
		throw(ex); //avoid multiple exceptions trown again (don't know why!)!
	}

	catch (const EXUexception& ex)
	{
		SysError("Internal error in python in USER FUNCTION '" + STDstring(functionName) + "' (referred line number my be wrong!):\n" + STDstring(ex.what()) + "; check your python code!");
		throw(ex); //avoid multiple exceptions trown again (don't know why!)!
	}
	catch (...) //any other exception
	{
		SysError("Unknown error in python USER FUNCTION '" + STDstring(functionName) + "' (referred line number my be wrong!): check your python code!");
	}
#else
	f();
#endif
}

//! specific template to handle exceptions catched during solver steps
template <typename Tfunction>
//void SolverExceptionHandling(Tfunction&& f, STDstring functionName)
void SolverExceptionHandling(Tfunction&& f, const char* functionName)
{
#ifdef __PYTHON_USERFUNCTION_CATCH__
	try
	{
		f();
	}
	//mostly catches python errors:
	catch (const pybind11::error_already_set& ex)
	{
		SysError("Error in solver function '" + STDstring(functionName) + "' originating from python code (referred line number my be wrong!):\n" + STDstring(ex.what()) + "; check your python code!");
		throw(ex); //avoid multiple exceptions trown again (don't know why!)!
	}
	catch (const EXUexception& ex)
	{
		SysError("EXUDYN raised internal error in '" + STDstring(functionName) + "':\n" + STDstring(ex.what()));
		throw(ex); //avoid multiple exceptions trown again (don't know why!)!
	}
	catch (...) //any other exception
	{
		SysError("Unexpected exception during '" + STDstring(functionName) + "'");
	}
#else
	f();
#endif
}

#endif
