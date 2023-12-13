/** ***********************************************************************************************
* @brief		Implementation file for automatically created interfaces to structures (settings)
* @details		Details:
				- saves compile time; see also PybindModule.cpp
*
* @author		Gerstmayr Johannes
* @date			2023-11-22
* @pre			...
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */

//
// pybind11 includes
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>       //interface to numpy
#include <pybind11/buffer_info.h> //passing reference to matrix to numpy
#include <pybind11/embed.h>       //scoped interpreter
//does not work globally: #include <pybind11/iostream.h> //used to redirect cout:  py::scoped_ostream_redirect output;
#include <pybind11/cast.h> //for arguments
#include <pybind11/functional.h> //for function handling ... otherwise gives a python error (no compilation error in C++ !)
namespace py = pybind11;
using namespace pybind11::literals; //brings in the '_a' literals; e.g. for short arguments definition

// includes needed for glfw test example
#define NOMINMAX //needs to be placed on top (before windows.h)! Otherwise std::min/max will cause error msg!
#include <cmath>

//#include "Linalg/BasicLinalg.h"
//
//// glfw testclass. This  includes the glfw test example
#include "Graphics/GlfwClient.h"


#include "System/versionCpp.h"
//
#include "Main/MainSystem.h"
//
#include "Pymodules/PyMatrixContainer.h"
#include "Pymodules/PyMatrixVector.h"
#include "Pymodules/PyGeneralContact.h"
//
#include "Main/SystemContainer.h"
#include "Main/MainSystemContainer.h"
//


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! this function returns the Python version for which Exudyn is compiled (even micro version, which may be different from Python interpreter!)
//! returns e.g. "3.9.0"
STDstring GetExudynPythonVersionString()
{
	STDstring str = EXUstd::ToString(PY_MAJOR_VERSION) + '.' + EXUstd::ToString(PY_MINOR_VERSION) + '.' + EXUstd::ToString(PY_MICRO_VERSION);
	return str;
}

//! this function is available outside PybindModule.cpp and returns version + additional information
STDstring GetExudynBuildVersionString(bool addDetails)
{
	STDstring str = STDstring(EXUstd::exudynVersion);
#ifndef EXUDYN_RELEASE
	str += "(pre-release)";
#pragma message("====================================")
#pragma message("EXUDYN not compiled in release mode!")
#pragma message("====================================")
#endif
	if (addDetails)
	{
		str += "; Python" + GetExudynPythonVersionString();
		str += "; " + EXUstd::GetPlatformString();
	}
#ifdef __FAST_EXUDYN_LINALG
	if (addDetails)
	{
		str += "[FAST]"; //changed from "[NO RANGE CHECKS]"
	}
	//#pragma message("====================================")
#pragma message("** EXUDYN using __FAST_EXUDYN_LINALG without range checks! **")
//#pragma message("====================================")
#endif
	return str;
}

//! retrieve current version as m.attr is not passed trough package
py::str PyGetVersionString(bool addDetails = false)
{
	return GetExudynBuildVersionString(addDetails);
}

//! this function is available outside PybindModule.cpp and returns version + additional information
void PyHelp()
{
	pout << "This is the exudyn Python module.\n";
	pout << "For basic help, visit the github page and start reading: https://github.com/jgerstmayr/EXUDYN \n";
	pout << "For tutorials, visit https://github.com/jgerstmayr/EXUDYN#tutorial \n";
	pout << "For quick demos, just write exudyn.Demo1() and exudyn.Demo2() \n";
	pout << "For many examples and test models, see https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev ; consider clone or .zip the repository\n";
	pout << "For advanced information, read theDoc: https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf \n";
	pout << "Good luck and have fun!\n";
	pout << "(C) 2018-2023 University of Innsbruck\n\n";
}

//! Definition of Invalid Index; to be used in Python to check whether a function returned a valid index (e.g. AddObject(...))
Index GetInvalidIndex() { return EXUstd::InvalidIndex; }

//! set flag to write (true) or not write to console; default = true
void PySetWriteToConsole(bool flag) { outputBuffer.SetWriteToConsole(flag); }

//! set flag to write (true) or not write to console; default = false
void PySetWriteToFile(STDstring filename, bool flagWriteToFile, bool flagAppend)
{
	outputBuffer.SetWriteToFile(filename, flagWriteToFile, flagAppend);
}

extern bool suppressWarnings; //!< global flag to suppress warnings
//! set flag to suppress (=true) or enable (=false) warnings
void PySuppressWarnings(bool flag)
{
	suppressWarnings = flag;
}

//! print function with line feed
void PyPrint(py::args args)
{
	for (auto item : args)
	{
		pout << item << " ";
	}
	pout << "\n";
}
//! add some delay (in milliSeconds) to printing to console, in order to let Spyder process the output; default = 0
void PySetPrintDelayMilliSeconds(Index delayMilliSeconds)
{
	outputBuffer.SetDelayMilliSeconds(delayMilliSeconds);
}

#ifdef __EXUDYN_RUNTIME_CHECKS__
extern Index array_new_counts;		//global counter of item allocations; is increased every time a new is called
extern Index array_delete_counts;	//global counter of item deallocations; is increased every time a delete is called
extern Index vector_new_counts;	//global counter of item allocations; is increased every time a new is called
extern Index vector_delete_counts; //global counter of item deallocations; is increased every time a delete is called
extern Index matrix_new_counts;	//global counter of item allocations; is increased every time a new is called
extern Index matrix_delete_counts; //global counter of item deallocations; is increased every time a delete is called
extern Index linkedDataVectorCast_counts; //global counter for unwanted type conversion from LinkedDataVector to Vector
#endif


//Print some (Debug) infos: linalg, threads, computational efficiency, etc.
py::list PythonInfoStat(bool writeOutput = true)
{
	py::list list;
#ifdef __EXUDYN_RUNTIME_CHECKS__
	if (writeOutput)
	{
		pout << "Linalg stats:\n";
		pout << "  array_new_counts:   " << array_new_counts << "\n";
		pout << "  array_delete_counts:" << array_delete_counts << "\n";

		pout << "  vector_new_counts:   " << vector_new_counts << "\n";
		pout << "  vector_delete_counts:" << vector_delete_counts << "\n";

		pout << "  matrix_new_counts:   " << matrix_new_counts << "\n";
		pout << "  matrix_delete_counts:" << matrix_delete_counts << "\n";

		pout << "  linkedDataVec_counts:" << linkedDataVectorCast_counts << "\n";
	}
	list.append(array_new_counts);
	list.append(array_delete_counts);
	list.append(vector_new_counts);
	list.append(vector_delete_counts);
	list.append(matrix_new_counts);
	list.append(matrix_delete_counts);
	list.append(linkedDataVectorCast_counts);
#else
	if (writeOutput)
	{
		pout << "Linalg stats deactivated (needs re-compile)\n";
	}
	list.append(0); //used by Static/DynamicSolver, therefore list needs to be created
	list.append(0);
	list.append(0);
	list.append(0);
	list.append(0);
	list.append(0);
	list.append(0);
#endif
	return list;


}

//! Set the precision for floating point numbers written to console; this is reset after a simulation is started by according simulation settings
void PySetOutputPrecision(Index precision)
{
	std::cout.precision(precision);
	pout.precision(precision);
}

extern bool linalgPrintUsePythonFormat; //!< true: use python format for output of vectors and matrices; false: use matlab format

//! true: use python format for output of vectors and matrices; false: use matlab format
void PySetLinalgOutputFormatPython(bool flagPythonFormat)
{
	linalgPrintUsePythonFormat = flagPythonFormat;
}

//! start glfw renderer; return true if successful
bool PyStartOpenGLRenderer(Index verbose = true)
{
#ifdef USE_GLFW_GRAPHICS
#if defined(__EXUDYN__APPLE__)
	//on APPLE, tkinter must be imported before start of OpenGL - workaround for BUG, #1339
	STDstring str = "";
	str += "try:\n";
	str += "    import tkinter as tk\n";
	str += "    rootTk = tk.Tk()\n";
	str += "    rootTk.withdraw()\n";
	str += "except:\n";
	str += "    pass\n"; //no error at this point as tkinter may just not be available for no-glfw use

	py::object scope = py::module::import("__main__").attr("__dict__"); //use this to enable access to mbs and other variables of global scope within test models suite
	py::exec(str.c_str(), scope);

#endif
	return glfwRenderer.SetupRenderer(verbose);
#else
	PyWarning("StartRenderer(): has no effect as GLFW_GRAPHICS is deactivated in your exudyn module (needs recompile or another version)");
	return false;
#endif
}

//! start glfw renderer; return true if successful
void PyStopOpenGLRenderer()
{
#ifdef USE_GLFW_GRAPHICS
	glfwRenderer.StopRenderer();
#else
	PyWarning("StopRenderer(): has no effect as GLFW_GRAPHICS is deactivated in your exudyn module (needs recompile or another version)");
#endif
}

//! start glfw renderer; return true if successful
bool PyIsRendererActive()
{
#ifdef USE_GLFW_GRAPHICS
	return glfwRenderer.IsGlfwInitAndRendererActive();
#else
	return false;
#endif
}

//! run renderer idle for certain amount of time; use this for single-threaded, interactive animations
void PyDoRendererIdleTasks(Real waitSeconds)
{
#ifdef USE_GLFW_GRAPHICS
	glfwRenderer.DoRendererIdleTasks(waitSeconds);
#else
	PyWarning("DoRendererIdleTasks(): has no effect as GLFW_GRAPHICS is deactivated in your exudyn module (needs recompile or another version)");
#endif
}

//! simple startup of exudyn module for debug, etc.
void PythonGo()
{
	py::exec(R"(
import exudyn
systemContainer = exudyn.SystemContainer()
mbs = systemContainer.AddSystem()
    )");
	pout << "main variables:\n systemContainer=exudyn.SystemContainer()\n mbs = systemContainer.AddSystem()\n";
	//pout << "ready to go\n";
}



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//some low level functions linked to exudyn
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "Main/Experimental.h"
PyExperimental pyExperimental;	//! for experimental things, not to be used by common user
PySpecial pySpecial;			//! special features; affects exudyn globally; treat with care




//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void Init_Pybind_manual_classes(py::module& m) {
	py::dict exudynVariables; //!< global dictionary which can be used by the user to store local variables
	py::dict exudynSystemVariables; //!< global dictionary which is used by system functions to store local variables

	//use _Experimental, because __Experimental (__) has special meaning in Python and may lead to different behavior
	py::class_<PyExperimental>(m, "Experimental", "Experimental features, not intended for regular users") //use _Experimental to distinguish from Experimental() function
		.def(py::init<>())
		//+++++++++++++++++++++++++++++++++++++++++++
		//.def_readwrite("useEigenFullPivotLUsolver", &Experimental::useEigenFullPivotLUsolver)//, "switch to special solver")
		.def_readwrite("eigenFullPivotLUsolverDebugLevel", &PyExperimental::eigenFullPivotLUsolverDebugLevel)//, "debug level for solver")

		//representation:
		.def("__repr__", [](const PyExperimental& item) {
		return STDstring(EXUstd::ToString(item));
			}, "return the string representation of Experimental class")
		;

	//Python version of SpecialSolver class
	py::class_<PySpecialSolver>(m, "SpecialSolver", "SpecialSolver features, to be handled with care")
		.def(py::init<>())
		//+++++++++++++++++++++++++++++++++++++++++++
		//multiThreadingType = MultiThreadingType::LoadBalancing;
		.def_readwrite("timeout", &PySpecialSolver::timeout)
		.def_readwrite("throwErrorWithCtrlC", &PySpecialSolver::throwErrorWithCtrlC)

		//representation:
		.def("__repr__", [](const PyExperimental& item) {
		return STDstring(EXUstd::ToString(item));
			}, "return the string representation of SpecialSolver class")
		;

	//Python version of Special class
	py::class_<PySpecial>(m, "Special", "Special features, to be handled with care")
		.def(py::init<>())
		//+++++++++++++++++++++++++++++++++++++++++++
		.def_readwrite("solver", &PySpecial::solver)

#ifdef PERFORM_UNIT_TESTS
			.def("RunCppUnitTests", &PySpecial::SpecialRunUnitTests, "Run C++ unit tests and return int with 'number of fails' (0 if all tests passed); reportOnPass=True also outputs the passed tests; printOutput prints according output to console",
				py::arg("reportOnPass") = false, py::arg("printOutput") = true)
#endif

		//representation:
		.def("__repr__", [](const PySpecial& item) {
		return STDstring(EXUstd::ToString(item));
			}, "return the string representation of Special class")
		;

			//enum class MultiThreadingType {
			//	MicroThreading = 0,
			//	LoadBalancing = 1,
			//};


	//moved here in order to be able to store current renderState in exudynSystemVariables
	//m.def("StopOpenGLRenderer", &GetVector, "GetVector");
	m.def("StopRenderer", [exudynSystemVariables]() {
		try
		{
			PyStopOpenGLRenderer();
#ifdef USE_GLFW_GRAPHICS
			py::dict d = MainSystemContainer::RenderState2PyDict(glfwRenderer.GetRenderState());
			exudynSystemVariables["renderState"] = d;
			//pout << "stop renderer\n";
#endif
		}
		catch (const EXUexception& ex)
		{
			SysError("EXUDYN raised internal error in StopOpenGLRenderer:\n" + STDstring(ex.what()) + "\n");
		}
		catch (...) //any other exception
		{
			SysError("Unexpected exception during StopOpenGLRenderer!\n");
		}

		}, "Stop the openGL renderer and write current renderState to exudyn.sys['renderState']");


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	#include "Autogenerated/pybind_manual_classes.h"


}





