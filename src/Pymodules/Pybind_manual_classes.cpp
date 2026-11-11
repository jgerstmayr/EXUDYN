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
#include "Pymodules/PyHomogeneousTransformation.h" //exudyn.HT (#2780)
#include "Pymodules/PyConnectorMarker.h" //exudyn.ConnectorMarker (#2975)
#include "Pymodules/PyGeneralContact.h"
//
#include "Main/SystemContainer.h"
#include "Main/MainSystemContainer.h"
#include "Graphics/IconBar.h"
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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// the following functions are kept in exudyn main module (not in exudyn.config):

//! this function is available outside PybindModule.cpp and returns version + additional information
void PyHelp()
{
	pout << "This is the exudyn Python module.\n";
	pout << "For basic help, visit the github page and start reading: https://github.com/jgerstmayr/EXUDYN \n";
	pout << "For more information and tutorials, visit https://exudyn.readthedocs.io/en/stable \n";
	pout << "For quick demos, just write exudyn.demos.Demo1() and exudyn.demos.Demo2() \n";
	pout << "For many examples and test models, see https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev ; consider clone or .zip the repository\n";
	pout << "For advanced information, read theDoc: https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf \n";
	pout << "Good luck and have fun!\n";
	pout << "(C) 2018-2025 University of Innsbruck\n\n";
}

//! Definition of Invalid Index; to be used in Python to check whether a function returned a valid index (e.g. AddObject(...))
Index GetInvalidIndex() { return EXUstd::InvalidIndex; }

//! set flag to write (true) or not write to console; default = true
void PySetWriteToConsole(bool flag) 
{ 
	PyDeprecated("functions", "exudyn.SetWriteToConsole", "exudyn.SetWriteToConsole(): function is deprecated; use set exudyn.config.printToConsole instead");
	outputBuffer.SetWriteToConsole(flag);
}

//! set flag to write (true) or not write to console; default = false
void PySetWriteToFile(STDstring filename, bool flagWriteToFile, bool flagAppend, bool flagFlushAlways)
{
	outputBuffer.SetWriteToFile(filename, flagWriteToFile, flagAppend, flagFlushAlways);
}

//! print function with line feed; this allows to either stream to console or to redirect to file, following settings in pout.
void PyPrint(py::args args, py::kwargs kwargs) {
	// Extract keyword arguments with defaults
	std::string sep = kwargs.contains("sep") ? kwargs["sep"].cast<std::string>() : " ";
	std::string end = kwargs.contains("end") ? kwargs["end"].cast<std::string>() : "\n";
	bool flush = kwargs.contains("flush") ? kwargs["flush"].cast<bool>() : false;

	// Print all positional arguments with separator
	bool first = true;
	for (auto item : args) 
	{
		if (!first) { pout << sep; }
		first = false;
		pout << item;
	}

	pout << end;

	if (flush || end!="\n") { outputBuffer.overflowFlush(0, flush, end != "\n"); } //does not print 0, but does flush or clears buffer if there is no "\n" at end of string
}

//void PyPrint(py::args args)
//{
//	for (auto item : args)
//	{
//		pout << item << " ";
//	}
//	pout << "\n";
//}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//the following functions/structures move to exudyn.config:

//! retrieve current version as m.attr is not passed trough package
py::str PyGetVersionString(bool addDetails = false)
{
	PyDeprecated("functions", "exudyn.GetVersionString", "exudyn.GetVersionString(): function is deprecated; use set exudyn.config.Version() instead");
	return GetExudynBuildVersionString(addDetails);
}

extern bool suppressWarnings; //!< global flag to suppress warnings
//! set flag to suppress (=true) or enable (=false) warnings
void PySuppressWarningsOld(bool flag)
{
	PyDeprecated("functions", "exudyn.SuppressWarnings", "exudyn.SuppressWarnings(): function is deprecated; use set exudyn.config.suppressWarnings instead");
	suppressWarnings = flag;
}


//! add some delay (in milliSeconds) to printing to console, in order to let Spyder process the output; default = 0
void PySetPrintDelayMilliSeconds(Index delayMilliSeconds)
{
	PyDeprecated("functions", "exudyn.SetPrintDelayMilliSeconds", "SetPrintDelayMilliSeconds(): function is deprecated; use set exudyn.config.printDelayMilliSeconds instead");
	outputBuffer.SetDelayMilliSeconds(delayMilliSeconds);
}

//! Set the precision for floating point numbers written to console; this is reset after a simulation is started by according simulation settings
void PySetOutputPrecision(Index precision)
{
	std::cout.precision(precision);
	pout.precision(precision);
}

void PySetOutputPrecisionOld(Index precision)
{
	PyDeprecated("functions", "exudyn.SetOutputPrecision", "SetOutputPrecision(): function is deprecated; use set exudyn.config.precision instead");
	PySetOutputPrecision(precision);
}

//! Set the precision for floating point numbers written to console; this is reset after a simulation is started by according simulation settings
Index PyGetOutputPrecision()
{
	return (Index)pout.precision();
}

extern bool linalgPrintUsePythonFormat; //!< true: use python format for output of vectors and matrices; false: use matlab format

//! true: use python format for output of vectors and matrices; false: use matlab format
void PySetLinalgOutputFormatPython(bool flagPythonFormat)
{
	PyDeprecated("functions", "exudyn.SetLinalgOutputFormatPython", "SetLinalgOutputFormatPython(): function is deprecated; use set exudyn.config.linalgOutputFormatPython instead");
	linalgPrintUsePythonFormat = flagPythonFormat;
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

py::list PythonInfoStatOld(bool writeOutput = true)
{
	PyDeprecated("functions", "exudyn.InfoStat", "exudyn.InfoStat(): function is deprecated; use set exudyn.special.InfoStat() instead");
	return PythonInfoStat(writeOutput);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//! start glfw renderer; return true if successful
bool PyStartOpenGLRenderer(Index verbose = true, bool deprecationWarning = false)
{
	if (deprecationWarning) { PyDeprecated("functions", "exudyn.StartRenderer", "exudyn.StartRenderer(): function is deprecated; for SystemContainer SC use set SC.renderer.Start() instead"); }
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
	return glfwRenderer.StartRenderer(verbose);
#else
	PyWarning("SC.renderer.Start(): has no effect as GLFW_GRAPHICS is deactivated in your exudyn module (needs recompile or another version)");
	return false;
#endif
}

//! stop glfw renderer; return true if successful
void PyStopOpenGLRenderer(bool deprecationWarning = false)
{
	if (deprecationWarning) { PyDeprecated("functions", "exudyn.StopRenderer", "exudyn.StopRenderer(): function is deprecated; for SystemContainer SC use set SC.renderer.Stop() instead"); }

#ifdef USE_GLFW_GRAPHICS
	try
	{
		glfwRenderer.StopRenderer();
		py::module exudynModule = py::module::import("exudyn");
		for (Index viewID = 0; viewID < glfwRenderer.GetRenderViews()->NumberOfViews(); viewID++)
		{
			if (glfwRenderer.GetRenderViews()->IsWindowOpen(viewID))
			{
				py::dict d = MainSystemContainer::RenderState2PyDict(*glfwRenderer.GetRenderViews()->State(viewID));
				if (viewID == 0) { exudynModule.attr("sys")["renderState"] = d; }
				else { exudynModule.attr("sys")[(STDstring("renderState")+EXUstd::ToString(viewID)).c_str()] = d; }
			}
		}
	}
	catch (const EXUexception& ex)
	{
		SysError("EXUDYN raised internal error in renderer.Stop():\n" + STDstring(ex.what()) + "\n");
	}
	catch (...) //any other exception
	{
		SysError("Unexpected exception during renderer.Stop()!\n");
	}
#else
		PyWarning("SC.renderer.Stop(): has no effect as GLFW_GRAPHICS is deactivated in your exudyn module (needs recompile or another version)");
#endif
}

//! start glfw renderer; return true if successful
void PyOpenViewWindow(Index viewID)
{
#ifdef USE_GLFW_GRAPHICS
	glfwRenderer.GetRenderViews()->SetWindowShouldBeCreated(viewID, true);
	//glfwRenderer.CreateViewWindow(viewID); //this can only be called from GLFWClient thread
#endif //no alternative, as already checked in caller if renderer is active!
}

//! start glfw renderer; return true if successful
void PyCloseViewWindow(Index viewID)
{
#ifdef USE_GLFW_GRAPHICS
	glfwRenderer.CloseViewWindow(viewID);
#endif //no alternative, as already checked in caller if renderer is active!
}


//! start glfw renderer; return true if successful
bool PyIsRendererActive(bool deprecationWarning = false)
{
	if (deprecationWarning) { PyDeprecated("functions", "exudyn.IsRendererActive", "exudyn.IsRendererActive(): function is deprecated; for SystemContainer SC use set SC.renderer.IsActive() instead"); }
#ifdef USE_GLFW_GRAPHICS
	return glfwRenderer.IsGlfwInitAndRendererActive();
#else
	return false;
#endif
}

//! wait until the first frame of the renderer has been drawn
Index PyGetRendererUpdateCount()
{
#ifdef USE_GLFW_GRAPHICS
	return GlfwRenderer::GetRendererTasksCount();
#else
	return 0;
#endif
}



//! run renderer idle for certain amount of time; use this for single-threaded, interactive animations
void PyDoRendererIdleTasks(Real waitSeconds, bool deprecationWarning = false)
{
	if (deprecationWarning) { PyDeprecated("functions", "exudyn.DoRendererIdleTasks", "exudyn.DoRendererIdleTasks(): function is deprecated; for SystemContainer SC use set SC.renderer.DoIdleTasks() instead"); }
#ifdef USE_GLFW_GRAPHICS
	glfwRenderer.DoRendererIdleTasks(waitSeconds);
#else
	PyWarning("DoRendererIdleTasks(): has no effect as GLFW_GRAPHICS is deactivated in your exudyn module (needs recompile or another version)");
#endif
}

//removed:
//! simple startup of exudyn module for debug, etc.
//void PythonGo()
//{
//	py::exec(R"(
//import exudyn
//systemContainer = exudyn.SystemContainer()
//mbs = systemContainer.AddSystem()
//    )");
//	pout << "main variables:\n systemContainer=exudyn.SystemContainer()\n mbs = systemContainer.AddSystem()\n";
//	//pout << "ready to go\n";
//}



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//some low level functions linked to exudyn
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "Main/Experimental.h"
#include "Linalg/PardisoMKL.h"	//exudyn.special.solver.PardisoInfo (#3431)
#include "Utilities/MathFunctions.h"	//exudyn.experimental.EvaluateMathFunction (#2947)
#include "Pymodules/PyCUtilities.h"	//exudyn.cUtilities (#2960)
#include "Pymodules/PyKinematicChain.h"	//exudyn.cUtilities.KinematicChain (#3259)
#include "Pymodules/PyConvexShape.h"	//exudyn.cUtilities.ConvexShape and Distance (#3317)
#include "Pymodules/PyCollisionModel.h"	//exudyn.cUtilities.CollisionModel (#3380)
#include "Linalg/Symbolic.h"	//exudyn.cMath with a symbolic argument (#3114)
#include "Linalg/linalgBenchmark.h"	//exudyn.special.RunLinalgBenchmark (#3410)
PyExperimental pyExperimental;	//! for experimental things, not to be used by common user
PySpecial pySpecial;			//! special features; affects exudyn globally; treat with care
bool EXUstd::ParameterRangeChecksActive() { return pySpecial.exceptions.parameterRangeChecks; }

//! the one dictionary behind exudyn.special.overrideSettings (#2679); see PybindUtilities.h.
//! Allocated on the first access, which happens during module initialization while the interpreter
//! and the GIL are there, and never freed: releasing a Python reference after finalization crashes
py::dict& EPyUtils::OverrideSettings()
{
	static py::dict* overrideSettings = new py::dict();
	return *overrideSettings;
}

#include "Main/Config.h"
ExudynConfig pyConfig;				//! unified config for exudyn, avoid bloating main scope

//! the dictionary interface of exudyn.config and its defaults (#2685); see PybindUtilities.h.
//! The settings are listed ONCE, here, and the three read-only ones are marked, so that a new
//! setting of exudyn.config is added in one place and reaches the dictionary, the defaults and
//! everything that compares against them
namespace
{
	//{name, writable}
	const std::vector<std::pair<const char*, bool>> configSettings = {
		{"outputPrecision", true},
		{"suppressWarnings", true},
		{"outputDirectory", true},
		{"linalgOutputFormatPython", true},
		{"printDelayMilliSeconds", true},
		{"printFlushAlways", true},
		{"printToConsole", true},
		{"printToFile", false},
		{"printFileName", false},
		{"printToFileAppend", false},
		};
}

py::dict& EPyUtils::ConfigDefaults()
{
	static py::dict* configDefaults = new py::dict();	//never freed, as OverrideSettings above
	return *configDefaults;
}

py::dict EPyUtils::ConfigDictionary(const ExudynConfig& config)
{
	py::dict values;
	py::object item = py::cast(&config);
	for (const auto& setting : configSettings)
	{
		values[setting.first] = item.attr(setting.first);
	}
	return values;
}

void EPyUtils::SetConfigFromDictionary(ExudynConfig& config, const py::dict& values)
{
	py::object item = py::cast(&config);
	for (const auto& setting : configSettings)
	{
		if (!setting.second) { continue; }				//printToFile and friends only report
		if (values.contains(setting.first))
		{
			item.attr(setting.first) = values[setting.first];
		}
	}
}



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//the user types (#2989)

//! the Python class of each kind of user type
static py::object UserTypeClass(Index kind)
{
	if (kind == UserTypes::OutputVariable) { return py::type::of<OutputVariableType>(); }
	if (kind == UserTypes::Node) { return py::type::of<Node::Type>(); }
	return py::type::of<Marker::Type>();
}

//! true if value is UserType0 to UserType3 of OutputVariableType, NodeType or MarkerType, with its kind and index
static bool UserTypeOf(const py::object& value, Index& kind, Index& index)
{
	Index64 bits;
	Index64 userType0;
	if (py::isinstance<OutputVariableType>(value)) { kind = UserTypes::OutputVariable; bits = (Index64)py::cast<OutputVariableType>(value); userType0 = (Index64)OutputVariableType::UserType0; }
	else if (py::isinstance<Node::Type>(value)) { kind = UserTypes::Node; bits = (Index64)py::cast<Node::Type>(value); userType0 = (Index64)Node::UserType0; }
	else if (py::isinstance<Marker::Type>(value)) { kind = UserTypes::Marker; bits = (Index64)py::cast<Marker::Type>(value); userType0 = (Index64)Marker::UserType0; }
	else { return false; }
	for (index = 0; index < UserTypes::numberOfUserTypes; index++)
	{
		if (bits == (userType0 << index)) { return true; }
	}
	return false;
}

//! str() of the enumeration writes the name of a user type; set once per class, the first time a name is set
static void SetUserTypeStr(const py::object& typeClass)
{
	if (py::hasattr(typeClass, "_userTypeStr")) { return; }
	py::object original = typeClass.attr("__str__");
	STDstring className = py::cast<STDstring>(typeClass.attr("__name__"));
	typeClass.attr("__str__") = py::cpp_function([original, className](const py::object& self) -> py::object {
		Index kind, index;
		if (UserTypeOf(self, kind, index)) { return py::str(className + "." + UserTypes::Name(kind, index)); }
		return original(self);
		}, py::is_method(typeClass));
	typeClass.attr("_userTypeStr") = true;
}

//! give a user type a name, which is also an alias of the value in its Python class
static void SetUserTypeName(const py::object& userType, const STDstring& name)
{
	Index kind, index;
	if (!UserTypeOf(userType, kind, index))
	{
		PyError("special.userTypes.SetName: userType must be UserType0 to UserType3 of OutputVariableType, NodeType or MarkerType", PyErrorType::valueError);
	}
	py::object typeClass = UserTypeClass(kind);
	STDstring& registered = UserTypes::RegisteredName(kind, index);
	if (!py::cast<bool>(py::str(name).attr("isidentifier")()))
	{
		PyError("special.userTypes.SetName: '" + name + "' is not a Python name", PyErrorType::valueError);
	}
	if (name != registered && py::hasattr(typeClass, name.c_str()))
	{
		PyError("special.userTypes.SetName: '" + name + "' is already a name of " + py::cast<STDstring>(typeClass.attr("__name__")), PyErrorType::valueError);
	}
	if (!registered.empty() && registered != name) { py::delattr(typeClass, registered.c_str()); }
	py::setattr(typeClass, name.c_str(), userType);
	registered = name;
	SetUserTypeStr(typeClass);
}

//! remove all names of the user types and their aliases
static void ResetUserTypeNames()
{
	for (Index kind = 0; kind < UserTypes::numberOfKinds; kind++)
	{
		for (Index index = 0; index < UserTypes::numberOfUserTypes; index++)
		{
			STDstring& registered = UserTypes::RegisteredName(kind, index);
			if (!registered.empty())
			{
				py::object typeClass = UserTypeClass(kind);
				if (py::hasattr(typeClass, registered.c_str())) { py::delattr(typeClass, registered.c_str()); }
				registered.clear();
			}
		}
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void Init_Pybind_manual_classes(py::module& m) {
	py::dict exudynVariables; //!< global dictionary which can be used by the user to store local variables
	py::dict exudynSystemVariables; //!< global dictionary which is used by system functions to store local variables

	//interface to exudyn.config
	py::class_<ExudynConfig>(m, "Config", "global config, including special settings for output and printing behavior")
		.def(py::init<>())
		//+++++++++++++++++++++++++++++++++++++++++++
		.def_property("outputPrecision", &ExudynConfig::GetOutputPrecision, &ExudynConfig::SetOutputPrecision)
		.def_property("suppressWarnings", &ExudynConfig::GetSuppressWarnings, &ExudynConfig::SetSuppressWarnings)
		.def_property("outputDirectory", &ExudynConfig::GetOutputDirectory, &ExudynConfig::SetOutputDirectory)
		.def_property("linalgOutputFormatPython", &ExudynConfig::GetLinalgPrintUsePythonFormat, &ExudynConfig::SetLinalgPrintUsePythonFormat)

		.def_property("printDelayMilliSeconds", &ExudynConfig::GetPrintDelayMilliSeconds, &ExudynConfig::SetPrintDelayMilliSeconds)
		.def_property("printFlushAlways", &ExudynConfig::GetFlushAlways, &ExudynConfig::SetFlushAlways)
		.def_property("printToConsole", &ExudynConfig::GetWriteToConsole, &ExudynConfig::SetWriteToConsole)
		.def_property_readonly("printToFile", &ExudynConfig::GetWriteToFile)
		.def_property_readonly("printFileName", &ExudynConfig::GetFileName)
		.def_property_readonly("printToFileAppend", &ExudynConfig::GetWriteAppend)


		.def("Version", &ExudynConfig::Version, "Get Exudyn built version as string (if addDetails=True, adds more information on compilation Python version, platform, etc.)", py::arg("addDetails") = false)

		.def("GetDictionary", [](const ExudynConfig& config) { return EPyUtils::ConfigDictionary(config); },
			"all settings of exudyn.config as a dictionary, as a settings structure gives them; printToFile, printFileName and printToFileAppend are in it but only report, so they are ignored by SetDictionary")
		.def("SetDictionary", [](ExudynConfig& config, const py::dict& values) { EPyUtils::SetConfigFromDictionary(config, values); },
			"set the settings named in the dictionary and leave the others; a name exudyn.config does not have is ignored", py::arg("values"))
		.def("GetDefaults", [](const ExudynConfig&) { return EPyUtils::ConfigDefaults(); },
			"the defaults of exudyn.config, taken when the module was imported and before any setting or override could change one: what Exudyn starts with. This is what tells a stored setting from one a user never touched")

		//representation:
		.def("__repr__", [](const ExudynConfig& item) {
		return STDstring(EXUstd::ToString(item));
			}, "return the string representation of Config class")
		;



	//THE DEFAULTS OF exudyn.config, TAKEN NOW (#2685): the class is registered, and no user code
	//has run yet - __init__.py applies the override settings and reads the environment variables
	//after the module is imported. They cannot be constructed on demand the way a settings
	//structure's defaults are, because every getter of ExudynConfig reads a GLOBAL: a second Config
	//reports the current values, so what Exudyn starts with has to be taken while it still is.
	EPyUtils::ConfigDefaults().clear();
	EPyUtils::ConfigDefaults().attr("update")(EPyUtils::ConfigDictionary(pyConfig));

	//use _Experimental, because __Experimental (__) has special meaning in Python and may lead to different behavior
	py::class_<PyExperimental>(m, "Experimental", "Experimental features, not intended for regular users") //use _Experimental to distinguish from Experimental() function
		.def(py::init<>())
		//+++++++++++++++++++++++++++++++++++++++++++
		//.def_readwrite("useEigenFullPivotLUsolver", &Experimental::useEigenFullPivotLUsolver)//, "switch to special solver")
		.def_readwrite("eigenFullPivotLUsolverDebugLevel", &PyExperimental::eigenFullPivotLUsolverDebugLevel)//, "debug level for solver")
		.def_readwrite("markerSuperElementRigidTexpSO3", &PyExperimental::markerSuperElementRigidTexpSO3)//, "debug level for solver")
		//the math library as the C++ code calls it, to compare the platforms (#2947)
		.def("EvaluateMathFunction", [](const PyExperimental&, const std::string& name, const std::vector<Real>& x,
			const std::vector<Real>& y, const std::string& implementation, Index repetitions) {
				std::vector<Real> result;
				Real seconds = EXUmath::EvaluateMathFunction(name, x, y, implementation, repetitions, result);
				if (seconds < 0)
				{
					auto Join = [](const std::vector<std::string>& names) { std::string s; for (auto& n : names) { s += (s.size() ? ", " : "") + n; } return s; };
					PyError("EvaluateMathFunction: unknown function '" + name + "' (" + Join(EXUmath::MathFunctionNames())
						+ ") or implementation '" + implementation + "' (" + Join(EXUmath::MathFunctionImplementations())
						+ "), or y not of the size of x for atan2, pow, fmod", PyErrorType::valueError);
				}
				return py::make_tuple(py::array_t<Real>(result.size(), result.data()), seconds);
			}, py::arg("name"), py::arg("x"), py::arg("y") = std::vector<Real>(), py::arg("implementation") = "std",
			py::arg("repetitions") = 1)

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
		.def_readwrite("multiThreadingLoadBalancing", &PySpecialSolver::multiThreadingLoadBalancing)
		.def_readwrite("initialConstraintsViolations", &PySpecialSolver::initialConstraintsViolations)
		.def_readwrite("suspendWorkersDuringFactorization", &PySpecialSolver::suspendWorkersDuringFactorization)
		.def_readwrite("suspendWorkersMinimumSize", &PySpecialSolver::suspendWorkersMinimumSize)
		.def_readwrite("pardisoNumberOfThreads", &PySpecialSolver::pardisoNumberOfThreads)
		.def_readwrite("pardisoCheckPeriod", &PySpecialSolver::pardisoCheckPeriod)
		.def_readwrite("pardisoResidualTolerance", &PySpecialSolver::pardisoResidualTolerance)
		.def_readwrite("pardisoFallback", &PySpecialSolver::pardisoFallback)
		.def_readwrite("pardisoParameters", &PySpecialSolver::pardisoParameters)
		.def_readwrite("pardisoFallbacks", &PySpecialSolver::pardisoFallbacks)
		.def_static("PardisoInfo", []() {
			std::string message;
			bool available = PardisoLibrary::Load(message);
			py::dict d;
			d["available"] = available;
			d["library"] = PardisoLibrary::LibraryPath();
			d["version"] = PardisoLibrary::Version();
			d["message"] = message;
			return d;
			}, "whether Intel MKL PARDISO (LinearSolverType.PardisoMKL) is available: a dict with available, library, version and message (#3431)")
		.def_static("StartPersistentThreads", [](Index numberOfThreads) {
			if (numberOfThreads < 2) { PyError("exudyn.special.solver.StartPersistentThreads: numberOfThreads must be at least 2, got " + EXUstd::ToString(numberOfThreads) + "; StopPersistentThreads() stops them", PyErrorType::valueError); }
			if (!ExuThreading::TaskManager::SetPersistentWorkers(numberOfThreads)) { PyError("exudyn.special.solver.StartPersistentThreads: not possible while a solver or the raytracer uses threads"); }
			static bool stopAtExit = false; //the workers wait on a condition variable, which must not die with them at exit
			if (!stopAtExit)
			{
				py::module_::import("atexit").attr("register")(py::cpp_function([]() { ExuThreading::TaskManager::SetPersistentWorkers(0); }));
				stopAtExit = true;
			}
			}, "start numberOfThreads persistent threads (main thread included) for the solvers (#3437)", py::arg("numberOfThreads"))
		.def_static("StopPersistentThreads", []() {
			if (!ExuThreading::TaskManager::SetPersistentWorkers(0)) { PyError("exudyn.special.solver.StopPersistentThreads: not possible while a solver or the raytracer uses threads"); }
			}, "stop the persistent threads of StartPersistentThreads (#3437)")
		.def_static("PersistentThreads", []() { return ExuThreading::TaskManager::PersistentWorkers(); },
			"the number of persistent threads, main thread included; 0: none (#3437)")

		//representation:
		.def("__repr__", [](const PySpecialSolver& item) {
		return STDstring(EXUstd::ToString(item));
			}, "return the string representation of SpecialSolver class")
		;

	//Python version of SpecialSolver class
	py::class_<PySpecialExceptions>(m, "SpecialExceptions", "SpecialExceptions features, to be handled with care")
		.def(py::init<>())
		//+++++++++++++++++++++++++++++++++++++++++++
		.def_readwrite("dictionaryNonCopyable", &PySpecialExceptions::dictionaryNonCopyable)
		.def_readwrite("dictionaryVersionMismatch", &PySpecialExceptions::dictionaryVersionMismatch)
		.def_readwrite("parameterRangeChecks", &PySpecialExceptions::parameterRangeChecks)
		.def_readwrite("writeErrorsToLogFile", &PySpecialExceptions::writeErrorsToLogFile)

		//representation:
		.def("__repr__", [](const PySpecialExceptions& item) {
		return STDstring(EXUstd::ToString(item));
			}, "return the string representation of SpecialExceptions class")
		;

	//Python version of SpecialUserInterface class
	py::class_<PySpecialUserInterface>(m, "SpecialUserInterface", "SpecialUserInterface flags, to be handled with care")
		.def(py::init<>())
		//+++++++++++++++++++++++++++++++++++++++++++
		.def_readwrite("suppressRenderer", &PySpecialUserInterface::suppressRenderer)
		.def_readwrite("suppressSolutionViewer", &PySpecialUserInterface::suppressSolutionViewer)
		.def_readwrite("suppressPlots", &PySpecialUserInterface::suppressPlots)
		.def_readwrite("suppressDialogs", &PySpecialUserInterface::suppressDialogs)
		.def("SuppressAll", &PySpecialUserInterface::SuppressAll, "set all four suppress flags at once; a run either wants windows or does not",
			py::arg("flag") = true)

		//representation:
		.def("__repr__", [](const PySpecialUserInterface& item) {
		return STDstring(EXUstd::ToString(item));
			}, "return the string representation of SpecialUserInterface class")
		;

	//Python version of SpecialBeams class
	py::class_<PySpecialBeams>(m, "SpecialBeams", "switches of the beam elements, for tests and comparisons")
		.def(py::init<>())
		.def_readwrite("geometricallyExactLumpedMass", &PySpecialBeams::geometricallyExactLumpedMass)
		.def("__repr__", [](const PySpecialBeams& item) {
		return STDstring(EXUstd::ToString(item));
			}, "return the string representation of SpecialBeams class")
		;

	//Python version of SpecialDeprecations class
	py::class_<PySpecialDeprecations>(m, "SpecialDeprecations", "how a deprecated name is reported: once per session or on every use, and recorded in exudyn.sys['deprecationUse']")
		.def(py::init<>())
		.def_readwrite("warnOnce", &PySpecialDeprecations::warnOnce)
		.def_readwrite("recordUse", &PySpecialDeprecations::recordUse)
		.def_readwrite("userFunctionListArguments", &PySpecialDeprecations::userFunctionListArguments,
			"if True, Python user functions get their vector arguments (q, q_t, ...) as lists, as before version 1.13.104, instead of read-only numpy arrays; deprecated, removed in 2031 (#3008)")
		.def_readwrite("symbolicStarProduct", &PySpecialDeprecations::symbolicStarProduct,
			"if True, * of two exudyn.symbolic vectors is their scalar product and * with a symbolic matrix the matrix product, as before version 1.13.115; False (=default): it raises a TypeError, as numpy multiplies the components - use dot() or @ for the products and multiply() for the product of the components; deprecated, removed when * becomes the product of the components (#3015, #3016)")
		.def_readwrite("symbolicVariablesWithoutScope", &PySpecialDeprecations::symbolicVariablesWithoutScope,
			"if True, a named exudyn.symbolic.Real, Vector or Matrix is a variable of no scope, so that two of one name are two variables, and a symbolic user function reads any named variable, as before version 1.13.330; False (=default): esym.Real(name, value, scope) gives the variable of that name in its scope, esym.variables or mbs.symbolicVariables, and a symbolic user function reads only those of mbs.symbolicVariables; deprecated, removed in a later version (#3208)")
		.def("Reset", &PySpecialDeprecations::Reset, "forget which names were warned about in this session, so that each one warns once more")
		.def("Warn", [](const PySpecialDeprecations&, const STDstring& source, const STDstring& name, const STDstring& message, int stackLevel) {
			PyDeprecated(source, name, message, stackLevel);
			}, "report the use of a deprecated name from Python: a DeprecationWarning, once per session unless warnOnce is False, and a count in exudyn.sys['deprecationUse'][source][name]; stackLevel=2 attributes it to the caller of the function that calls Warn",
			py::arg("source"), py::arg("name"), py::arg("message"), py::arg("stackLevel") = 2)
		.def("__repr__", [](const PySpecialDeprecations& item) {
		return STDstring(EXUstd::ToString(item));
			}, "return the string representation of SpecialDeprecations class")
		;

	//Python version of SpecialUserTypes class (#2989)
	py::class_<PySpecialUserTypes>(m, "SpecialUserTypes", "the names of the user types UserType0-3 of OutputVariableType, NodeType and MarkerType")
		.def(py::init<>())
		.def("SetName", [](const PySpecialUserTypes&, const py::object& userType, const STDstring& name) { SetUserTypeName(userType, name); },
			"give userType, UserType0 to UserType3 of OutputVariableType, NodeType or MarkerType, a name: it is written wherever Exudyn writes the type (sensor files, messages, str()), and it becomes an alias of the value, e.g. exudyn.OutputVariableType.BendingStrain; a new name replaces the previous one",
			py::arg("userType"), py::arg("name"))
		.def("GetName", [](const PySpecialUserTypes&, const py::object& userType) {
			Index kind, index;
			if (!UserTypeOf(userType, kind, index)) { PyError("special.userTypes.GetName: userType must be UserType0 to UserType3 of OutputVariableType, NodeType or MarkerType", PyErrorType::valueError); }
			return STDstring(UserTypes::Name(kind, index));
			}, "the name of userType: the name that was set, or UserType0, ...", py::arg("userType"))
		.def("Reset", [](const PySpecialUserTypes&) { ResetUserTypeNames(); },
			"remove all names and aliases of the user types")
		.def("__repr__", [](const PySpecialUserTypes& item) {
		return STDstring(EXUstd::ToString(item));
			}, "return the string representation of SpecialUserTypes class")
		;

	//the texture store without a SystemContainer, for the textures of exudyn.graphics, e.g. CheckerBoard (#3036);
	//the user adds textures with SC.renderer.AddTexture
	m.def("_AddTexture", [](const STDstring& name, const py::object& image, TextureInterpretation interpretation,
		Index flags) { PyAddTexture(name, image, interpretation, flags, true); },
		"internal: add a texture; names starting with 'exudyn.' are allowed", py::arg("name"), py::arg("image"),
		py::arg("interpretation"), py::arg("flags")); //no defaults: the enums are registered after this
	//a texture as a dict and back, e.g. for the dict of a Robot (#3268)
	m.def("_GetTexture", &PyGetTexture, "internal: a texture as a dict, as renderer.GetTexture", py::arg("name"));
	m.def("_SetTexture", &PySetTexture, "internal: add a texture from a dict of renderer.GetTexture; names starting with 'exudyn.' are allowed", py::arg("textureDict"));

	//the images of the icons of the icon bar, for their test (#3026): a list of [name, uint8 array (64, 64, 4)], first row at the top
	m.def("_IconBarImages", []() {
		py::list result;
		std::vector<unsigned char> rgba;
		for (Index icon = 0; icon < IconBar::NumberOfIcons(); icon++)
		{
			IconBar::DrawIcon(icon, rgba);
			const py::ssize_t n = IconBar::iconPixels;
			py::array_t<uint8_t> image({ n, n, (py::ssize_t)4 });
			auto pixels = image.mutable_unchecked<3>();
			for (py::ssize_t j = 0; j < n; j++) { for (py::ssize_t i = 0; i < n; i++) { for (py::ssize_t k = 0; k < 4; k++) {
				pixels(n - 1 - j, i, k) = rgba[(size_t)((j*n + i)*4 + k)]; } } }
			result.append(py::make_tuple(STDstring(IconBar::Name(icon)), image));
		}
		return result;
		}, "internal: the images of the icons of the icon bar");

	//Python version of Special class
	py::class_<PySpecial>(m, "Special", "Special features, to be handled with care")
		.def(py::init<>())
		//+++++++++++++++++++++++++++++++++++++++++++
		.def_readwrite("solver", &PySpecial::solver)
		.def_readwrite("exceptions", &PySpecial::exceptions)
		.def_readwrite("userInterface", &PySpecial::userInterface)
		.def_readwrite("beams", &PySpecial::beams)
		.def_readwrite("deprecations", &PySpecial::deprecations)
		.def_readwrite("userTypes", &PySpecial::userTypes)
		.def_property_readonly("currentRendererSystemContainer", [](const PySpecial&) -> py::object {
			//py::cast of the POINTER returns the Python object that already wraps it, so a script
			//gets the very SystemContainer it created; None when no renderer is attached
			if (MainSystemContainer::currentRendererContainer == nullptr) { return py::none(); }
			return py::cast(MainSystemContainer::currentRendererContainer);
			},
			"the SystemContainer the renderer is attached to, or None; GLFW can hold one at a time, so this is module-wide. It is set when a SystemContainer attaches to the render engine and cleared when it detaches or is destroyed, and it is what the dialogs of exudyn.misc.GUI ask for; not intended for regular users")
		.def_property_readonly("overrideSettings", [](const PySpecial&) { return EPyUtils::OverrideSettings(); },
			"the settings read from ~/.exudyn/config.json by 'import exudyn', as a dictionary with one key per section ('config', 'visualizationSettings', 'dialogs', 'resultsMonitor'); it is empty unless a user has stored something, and EXUDYN_NO_USER_SETTINGS=1 keeps it empty; the same dictionary is read by the C++ side; use exudyn.misc.overrideSettings for reading and writing the file")
		.def_static("RunLinalgBenchmark", [](Real minimumTime, Index numberOfThreads) {
			py::list list;
			for (const LinalgBenchmarkResult& r : RunLinalgBenchmark(minimumTime, numberOfThreads))
			{
				py::dict d;
				d["group"] = r.group;
				d["library"] = r.library;
				d["operation"] = r.operation;
				d["size"] = r.size;
				d["threads"] = r.threads;
				d["nsPerCall"] = r.nsPerCall;
				d["checkSum"] = r.checkSum;
				list.append(d);
			}
			return list;
			}, "the linear algebra micro-benchmark of this build (#3410): the time per call in ns of the operations of the computation functions of objects, markers and connectors and of the assembly, with Eigen as the reference: vector add, scale and MultAdd over sizes 3 to 500 and across the threading limit (group vector), Matrix3D, 6 x 6 and transformations (small), J^T f and J^T K J of a 3 x n Jacobian (jacobian), matrix times vector and the dense factorization and solve (dense), local Jacobians into the sparse triplets (assembly), AutoDiff<12> (autodiff), and a matrix of compile-time size as prototype; returns a list of dictionaries with group, library, operation, size, threads, nsPerCall and checkSum; each operation runs for at least minimumTime seconds; numberOfThreads > 1 adds the multithreaded vectors above the threading limit",
			py::arg("minimumTime") = 0.05, py::arg("numberOfThreads") = 1)
		.def_static("InfoStat", &PythonInfoStat, "Retrieve list of global information on memory allocation and other counts as list:[array_new_counts, array_delete_counts, vector_new_counts, vector_delete_counts, matrix_new_counts, matrix_delete_counts, linkedDataVectorCast_counts]; May be extended in future; if writeOutput==True, it additionally prints the statistics; counts for new vectors and matrices should not depend on numberOfSteps, except for some objects such as ObjectGenericODE2 and for (sensor) output to files; Not available if code is compiled with __FAST_EXUDYN_LINALG flag", py::arg("writeOutput") = true)

#ifdef PERFORM_UNIT_TESTS
			.def("RunCppUnitTests", &PySpecial::SpecialRunUnitTests, "Run C++ unit tests and return int with 'number of fails' (0 if all tests passed); reportOnPass=True also outputs the passed tests; printOutput prints according output to console",
				py::arg("reportOnPass") = false, py::arg("printOutput") = true)
#endif

		//representation:
		.def("__repr__", [](const PySpecial& item) {
		return STDstring(EXUstd::ToString(item)) + "overrideSettings: "
			+ EXUstd::ToString((Index)EPyUtils::OverrideSettings().size()) + " section(s)\n";
			}, "return the string representation of Special class")
		;

			//enum class MultiThreadingType {
			//	MicroThreading = 0,
			//	LoadBalancing = 1,
			//};


	////moved here in order to be able to store current renderState in exudynSystemVariables
	////m.def("StopOpenGLRenderer", &GetVector, "GetVector");
	//m.def("StopRenderer", [exudynSystemVariables]() {
	//		PyStopOpenGLRenderer();

	//	}, "Stop the openGL renderer and write current renderState to exudyn.sys['renderState']");


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	#include "Autogenerated/pybind_manual_classes.h"


}





