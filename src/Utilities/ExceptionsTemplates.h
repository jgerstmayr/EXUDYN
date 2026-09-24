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

//THE CAUSE OF AN EXUDYN EXCEPTION (#2537).
//When a user's Python function raises, the handlers below report it as an Exudyn exception and the
//original used to survive only as words inside the message. These three carry it as an OBJECT from
//the handler that caught it to the pybind boundary, where the translator chains it with
//py::raise_from - so 'except ModelError as e: e.__cause__' IS the user's ZeroDivisionError, with
//its traceback. Defined in Stdoutput.cpp; a raw PyObject* and not a py::object, because a
//thread_local py::object destructor needs the GIL at thread exit.
void SetPendingExceptionCause(PyObject* value);  //!< store a new reference to the exception that caused what is about to be thrown
void ClearPendingExceptionCause();               //!< drop it; the translator does this whenever it runs
PyObject* PendingExceptionCause();               //!< borrowed reference, or nullptr

//#undef __PYTHON_USERFUNCTION_CATCH__
//ignore exceptions in visualization
template <typename Tfunction>
void VisualizationExceptionHandling(Tfunction&& f)
{
	try
	{
		f();
	}
	catch (...) //any other exception
	{
		;
	}
}


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
		//the user's own Python failed inside their own function. That is a MODEL error - a user
		//function is part of the model - and naming it as one is what keeps the solver from
		//reporting it a second time as an internal Exudyn error (#2524)
		SetPendingExceptionCause(ex.value().ptr()); //the real exception travels on as __cause__ (#2537)
		PyError("Error in Python USER FUNCTION '" + STDstring(functionName) + "':\n" + STDstring(ex.what()) + "; check your Python code!",
			PyErrorType::modelError);
	}

	catch (const EXUexception& ex)
	{
		PyError("Internal error in Python in USER FUNCTION '" + STDstring(functionName) + "' (referred line number my be wrong!):\n" + STDstring(ex.what()) + "; check your Python code!");
		//not needed due to change of SysError: throw(ex); //avoid multiple exceptions trown again (don't know why!)!
	}
	catch (...) //any other exception
	{
		PyError("Unknown error in Python USER FUNCTION '" + STDstring(functionName) + "' (referred line number my be wrong!): check your Python code!");
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
		SetPendingExceptionCause(ex.value().ptr()); //(#2537)
		PyError("Error in solver function '" + STDstring(functionName) + "' originating from Python code:\n" + STDstring(ex.what()) + "; check your Python code!",
			PyErrorType::modelError);
	}
	//ALREADY REPORTED, and with a type that says what it is: a user function that raised, a
	//parameter error, a solver failure. Reporting it again as "EXUDYN raised internal error" labels
	//a user's mistake an Exudyn bug - which is exactly what happened to every user function error
	//until #2524. These must come before catch(EXUexception): both derive from it (info fact 29)
	//
	//StopRendererOnError() is what the SysError below used to do on the way past (#2531,
	//revision2026 step R6.3.11). Passing the exception through skipped it, so every site typed by
	//step R6.3.6 silently stopped shutting the renderer down - the behaviour came to depend on how
	//far the typing had got. The flag belongs HERE, at the solver boundary: the renderer stalls
	//anyway once an exception reaches the solver, while an error raised while a model is being
	//built must not take the window down.
	catch (const py::builtin_exception&)
	{
		StopRendererOnError();
		throw;
	}
	catch (const ExudynError&)
	{
		StopRendererOnError();
		throw;
	}
	catch (const EXUexception& ex)
	{
		SysError("EXUDYN raised internal error in '" + STDstring(functionName) + "':\n" + STDstring(ex.what()));
	}
	catch (...) //any other exception
	{
		SysError("Unexpected exception during '" + STDstring(functionName) + "'");
	}
#else
	f();
#endif
}

//! generic handling of exceptions for SetSafely and other data transmission
template <typename Tfunction>
void GenericExceptionHandling(Tfunction&& f, const char* placeOfException)
{
	try
	{
		f();
	}
	//mostly catches python errors:
	catch (const pybind11::error_already_set& ex)
	{
		SetPendingExceptionCause(ex.value().ptr()); //(#2537)
		PyError("Error in '" + STDstring(placeOfException) + "' (referred line number my be wrong!):\n" + STDstring(ex.what()) + "; check your Python code!");
		//not needed due to change of SysError: throw(ex); //avoid multiple exceptions trown again (don't know why!)!
	}

	//a parameter error that named its Python exception type keeps it; its message was
	//already printed where it was raised (#2432).
	//NOTE: this MUST come before catch(EXUexception), which is std::runtime_error and would
	//otherwise catch it first (ReleaseAssert.h:30) - and the same holds for the Exudyn exception
	//classes, which derive from EXUexception for exactly that reason (#2516)
	catch (const py::builtin_exception&)
	{
		throw;
	}
	catch (const ExudynError&)
	{
		throw;
	}
	catch (const EXUexception& ex)
	{
		PyError("Internal error in '" + STDstring(placeOfException) + "' (referred line number my be wrong!):\n" + STDstring(ex.what()) + "; check your Python code!");
		//not needed due to change of SysError: throw(ex); //avoid multiple exceptions trown again (don't know why!)!
	}
	catch (...) //any other exception
	{
		PyError("Unknown error in '" + STDstring(placeOfException) + "' (referred line number my be wrong!): check your Python code!");
	}
}



#endif
