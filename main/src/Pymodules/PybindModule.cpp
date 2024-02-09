/** ***********************************************************************************************
* @brief		Implementation file for linking to python; includes several automatically generated header files inside of module (kind of hack)
* @details		Details:
				- main module linked to python (python interface)
*
* @author		Stefan Holzinger / Gerstmayr Johannes
* @date			2019-03-29 
* @pre			...
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
************************************************************************************************ */

//++++++++++++++++++++++++++
//for signal, catch CTRL-C in console:
//#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <array>
#include <vector>
#include <signal.h>
//++++++++++++++++++++++++++

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

#include "System/versionCpp.h"
#include "Main/MainSystem.h"
//#include "Pymodules/PybindUtilities.h" //for RenderState conversions
#include "Main/SystemContainer.h"
#include "Main/MainSystemContainer.h"

//for special test functions:
#include "Pymodules/PybindTests.h"
#include "Pymodules/PybindUtilities.h"

//#pragma message("==========================")
#ifdef use_AVX2
#pragma message("** compiled with AVX2 **")
#elif defined(use_AVX512)
#pragma message("** compiled with AVX512 **")
#else
#pragma message("** compiled without AVX **")
#endif

//check some platform / architecture or compiler specific things at which is compiled and define globally used flags:
#if defined(__EXUDYN__APPLEM1__)
#pragma message("*** compiled for MacOS (ARM M1) ***")
#elif defined(__EXUDYN__APPLE__)
#pragma message("*** compiled for MacOS (x86) ***")
#elif defined(__EXUDYN__WINDOWS__)
	#if defined(_WIN32) && !defined(_WIN64) //_WIN32 also defined in 64 bits mode!
	#pragma message("*** compiled for Windows _x86 (32bits) ***") //this works for VS2017
    #elif defined(_WIN64)
	#pragma message("*** compiled for Windows _x86 (64bits) ***") //this works for VS2017
	#else
	#pragma message("*** compiled for Windows _x86 ***")
	#endif
#elif defined(__EXUDYN__LINUX__ARM__)
#pragma message("*** compiled for linux / ARM CPU ***")
#elif defined(__EXUDYN__LINUX__x86__)
#pragma message("*** compiled for linux / x86 CPU ***")
#else
#pragma message("*** WARNING: NO KNOWN PLATFORM DETECTED!!! ***")
#endif


//definition of further module functions; could improve parallel compile time (on many cores)
void Init_Pybind_modules(py::module&);
void Init_Pybind_manual_classes(py::module&);
void Init_Pybind_Symbolic(py::module&);


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// write access to system variables dictionary inside exudyn module
void PyWriteToSysDictionary(const STDstring& key, py::object item)
{
    py::module exudynModule = py::module::import("exudyn");
    exudynModule.attr("sys")[key.c_str()] = item;
}

// write access to system variables dictionary inside exudyn module
Real PyReadRealFromSysDictionary(const STDstring& key)
{
    py::module exudynModule = py::module::import("exudyn");
    return py::cast<Real>(exudynModule.attr("sys")[key.c_str()]);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//// Define the function to be called when ctrl-c (SIGINT) is sent to process
//void signal_callback_handler(int signum) 
//{
//	if (signum == SIGINT)
//	{
//		//std::cout << "Caught signal " << signum << "\n";
//		//std::cout << "Process interrupted by user (CTRL+C)!\n";
//		pout << "Process interrupted by user (CTRL+C)!\n";
//		// Terminate program
//		exit(signum);
//	}
//}
//
////enable CTRL-C in DOS windows
//class MySignal 
//{
//public:
//	MySignal()
//	{
//		// Register signal and signal handler
//		signal(SIGINT, signal_callback_handler);
//	}
//};
//
//MySignal registerSignal;


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#ifdef __FAST_EXUDYN_LINALG
PYBIND11_MODULE(exudynCPPfast, m) {
	m.doc() = "EXUDYN binding Python<->C++\n This is the 'fast' version without range/memory/whatsoever checks and uses /fp:fast compiler options!\n -> usage:\nSC=exu.SystemContainer()\nmbs=SC.AddSystem()\n see theDoc.pdf for tutorials, interface description and further information"; // module docstring
#pragma message("***** pybind: building exudynCPPfast module *****")
#elif defined(__EXUDYN_COMPILE_NOAVX)
PYBIND11_MODULE(exudynCPPnoAVX, m) {
	m.doc() = "EXUDYN binding Python<->C++\n This is the version without AVX(2) compiler options for high CPU compatibility!\n -> usage:\nSC=exu.SystemContainer()\nmbs=SC.AddSystem()\n see theDoc.pdf for tutorials, interface description and further information"; // module docstring
#pragma message("***** pybind: building exudynCPPnoAVX module *****")
#else
PYBIND11_MODULE(exudynCPP, m) {
	m.doc() = "EXUDYN binding Python<->C++\n -> usage:\nSC=exu.SystemContainer()\nmbs=SC.AddSystem()\n see theDoc.pdf for tutorials, interface description and further information"; // module docstring
//#pragma message("***** pybind: building exudynCPP module *****")
#endif
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//variables linked to exudyn
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//py::dict exudynVariables; //!< global dictionary which can be used by the user to store local variables
	//py::dict exudynSystemVariables; //!< global dictionary which is used by system functions to store local variables


	// Create a submodule named "symbolic"
	auto symbolic = m.def_submodule("symbolic", "A submodule for symbolic operations and mathematical expression trees for user functions and future symbolic operations in exudyn");
	Init_Pybind_Symbolic(symbolic);

	// Add functions, classes, etc. to the submodule
	// For example:
	// symbolic.def("some_function", &some_function, "A function in the symbolic submodule");

	//no effect:
	//m.def("__repr__", [](const PyModuleDef&) {
	//	return STDstring("This is the exudyn module; see theDoc.pdf on https://github.com/jgerstmayr/EXUDYN tutorials how to create multibody models");
	//}, "return the string representation of module");



#ifndef EXUDYN_RELEASE
	m.def("Test", &PyTest, "internal test, do not use");
	//m.def("GetTestSD", &GetTestSD, "test user function");
#endif


#ifdef _MYDEBUG //only in debug mode!
	m.def("CreateTestSystem", &CreateTestSystem, "Create test system with systemNumber and optional arguments; will create the system depending on arg1 and arg2", py::arg("systemNumber")=0, py::arg("arg0") = 0, py::arg("arg1") = 0);
#endif

	py::enum_<AccessFunctionType>(m, "AccessFunctionType")
		.value("_None", AccessFunctionType::_None)
		.value("TranslationalVelocity_qt", AccessFunctionType::TranslationalVelocity_qt)
		.value("AngularVelocity_qt", AccessFunctionType::AngularVelocity_qt)
		.value("Coordinate_q", AccessFunctionType::Coordinate_q)
		.value("DisplacementLineIntegral_q", AccessFunctionType::DisplacementLineIntegral_q)
		.value("DisplacementSurfaceIntegral_q", AccessFunctionType::DisplacementSurfaceIntegral_q)
		.value("DisplacementVolumeIntegral_q", AccessFunctionType::DisplacementVolumeIntegral_q)
		.value("DisplacementMassIntegral_q", AccessFunctionType::DisplacementMassIntegral_q)
		.value("DisplacementSurfaceNormalIntegral_q", AccessFunctionType::DisplacementSurfaceNormalIntegral_q)
		//.value("Rotv1v2v3", AccessFunctionType::Rotv1v2v3_q)
		//.value("EndOfEnumList", AccessFunctionType::EndOfEnumList)
		;// .export_values();

	py::enum_<CObjectType>(m, "ObjectType")
		.value("_None", CObjectType::_None)
		.value("Ground", CObjectType::Ground)
		.value("Constraint", CObjectType::Constraint)
		.value("Connector", CObjectType::Connector)
		.value("Body", CObjectType::Body)
		.value("SingleNoded", CObjectType::SingleNoded)
		.value("MultiNoded", CObjectType::MultiNoded)
		.value("FiniteElement", CObjectType::FiniteElement)
		.value("SuperElement", CObjectType::SuperElement)
		.value("EndOfEnumList", CObjectType::EndOfEnumList)
		;//.export_values();



	py::enum_<CNodeGroup>(m, "NodeGroup")
		.value("_None", CNodeGroup::_None)
		.value("ODE1variables", CNodeGroup::ODE1variables)
		.value("ODE2variables", CNodeGroup::ODE2variables)
		.value("AEvariables", CNodeGroup::AEvariables)
		.value("DataVariables", CNodeGroup::DataVariables)
		;// .export_values();


	//#include "Autogenerated/Pybind_modules.h"
	Init_Pybind_modules(m);

	Init_Pybind_manual_classes(m);

	//++++++++++++++++++++++++++++++++++++++++

	//#include "Autogenerated/pybind_manual_classes.h"

	////++++++++++++++++++++++++++++++++++++++++
	//DELETE:
	//py::class_<MainSystemBase>(m, "MainSystemBase", "MainSystemBase: equal to MainSystem")
	//	.def(py::init<>())
	//	;
	////used to avoid including MainSystem in many places (but very slow!)
	//py::implicitly_convertible<MainSystemBase, MainSystem>();

	//+++++++++++++++++++++++++++++++++++++++++++
	//item indices:
	py::class_<NodeIndex>(m, "NodeIndex", "NodeIndex: index which may only be used for nodes")
		.def(py::init<>())
		.def(py::init<Index>())
		//+++++++++++++++++++++++++++++++++++++++++++
		//private: .def_readwrite("index", &NodeIndex::index)
		.def("GetTypeString", &NodeIndex::GetTypeString,
			"get type string for identification in Python")
		.def("GetIndex", &NodeIndex::GetIndex,
			"get index converted to index / int")
		.def("SetIndex", &NodeIndex::SetIndex,
			"set index with index / int")
		.def("__int__", [](const NodeIndex &item) {
		return item.GetIndex();
		}, "return the integer representation of the index")
		.def("__index__", [](const NodeIndex &item) {
			return item.GetIndex();
		}, "return the integer representation of the index; consistent with Python 3.8+")
		//add operators:
		.def(py::self + int())
		.def(int() + py::self)
		.def(py::self - int())
		.def(int() - py::self)
		.def(int() * py::self)
		.def(py::self * int())
		.def(-py::self)
		//pickle:
		.def(py::pickle(
			[](const NodeIndex& self) {
				return py::make_tuple(self.GetIndex());
			},
			[](const py::tuple& t) {
				CHECKandTHROW(t.size() == 1, "NodeIndex: loading data with pickle received invalid data structure!");
				NodeIndex self(py::cast<Index>(t[0]));
				return self;
			}))
		//representation:
		.def("__repr__", [](const NodeIndex &item) {
			return STDstring(EXUstd::ToString(item.GetIndex()));
		}, "return the string representation of the index, which can be, e.g., printed")
		;

	py::class_<ObjectIndex>(m, "ObjectIndex", "ObjectIndex: index which may only be used for objects")
		.def(py::init<>())
		.def(py::init<Index>())
		//+++++++++++++++++++++++++++++++++++++++++++
		//private: .def_readwrite("index", &ObjectIndex::index)
		.def("GetTypeString", &ObjectIndex::GetTypeString,
			"get type string for identification in Python")
		.def("GetIndex", &ObjectIndex::GetIndex,
			"get index converted to index / int")
		.def("SetIndex", &ObjectIndex::SetIndex,
			"set index with index / int")
		.def("__int__", [](const ObjectIndex &item) {
			return item.GetIndex();
			}, "return the integer representation of the index")
		.def("__index__", [](const ObjectIndex &item) {
				return item.GetIndex();
			}, "return the integer representation of the index; consistent with Python 3.8+")
		//add operators:
		.def(py::self + int())
		.def(int() + py::self)
		.def(py::self - int())
		.def(int() - py::self)
		.def(int() * py::self)
		.def(py::self * int())
		.def(-py::self)
		//pickle:
		.def(py::pickle(
			[](const ObjectIndex& self) {
				return py::make_tuple(self.GetIndex());
			},
			[](const py::tuple& t) {
				CHECKandTHROW(t.size() == 1, "ObjectIndex: loading data with pickle received invalid data structure!");
				ObjectIndex self(py::cast<Index>(t[0]));
				return self;
			}))
		//representation:
		.def("__repr__", [](const ObjectIndex &item) {
			return STDstring(EXUstd::ToString(item.GetIndex()));
		}, "return the string representation of the index, which can be, e.g., printed")
			;

	py::class_<MarkerIndex>(m, "MarkerIndex", "MarkerIndex: index which may only be used for markers")
		.def(py::init<>())
		.def(py::init<Index>())
		//+++++++++++++++++++++++++++++++++++++++++++
		//private: .def_readwrite("index", &MarkerIndex::index)
		.def("GetTypeString", &MarkerIndex::GetTypeString,
			"get type string for identification in Python")
		.def("GetIndex", &MarkerIndex::GetIndex,
			"get index converted to index / int")
		.def("SetIndex", &MarkerIndex::SetIndex,
			"set index with index / int")
		.def("__int__", [](const MarkerIndex &item) {
			return item.GetIndex();
		}, "return the integer representation of the index")
		.def("__index__", [](const MarkerIndex &item) {
			return item.GetIndex();
		}, "return the integer representation of the index; consistent with Python 3.8+")
			//add operators:
		.def(py::self + int())
		.def(int() + py::self)
		.def(py::self - int())
		.def(int() - py::self)
		.def(int() * py::self)
		.def(py::self * int())
		.def(-py::self)
		//pickle:
		.def(py::pickle(
			[](const MarkerIndex& self) {
				return py::make_tuple(self.GetIndex());
			},
			[](const py::tuple& t) {
				CHECKandTHROW(t.size() == 1, "MarkerIndex: loading data with pickle received invalid data structure!");
				MarkerIndex self(py::cast<Index>(t[0]));
				return self;
			}))
		//representation:
		.def("__repr__", [](const MarkerIndex &item) {
			return STDstring(EXUstd::ToString(item.GetIndex()));
		}, "return the string representation of the index, which can be, e.g., printed")
		;

	py::class_<LoadIndex>(m, "LoadIndex", "LoadIndex: index which may only be used for loads")
		.def(py::init<>())
		.def(py::init<Index>())
		//+++++++++++++++++++++++++++++++++++++++++++
		//private: .def_readwrite("index", &LoadIndex::index)
		.def("GetTypeString", &LoadIndex::GetTypeString,
			"get type string for identification in Python")
		.def("GetIndex", &LoadIndex::GetIndex,
			"get index converted to index / int")
		.def("SetIndex", &LoadIndex::SetIndex,
			"set index with index / int")
		.def("__int__", [](const LoadIndex &item) {
			return item.GetIndex();
		}, "return the integer representation of the index")
		.def("__index__", [](const LoadIndex &item) {
			return item.GetIndex();
		}, "return the integer representation of the index; consistent with Python 3.8+")
			//add operators:
		.def(py::self + int())
		.def(int() + py::self)
		.def(py::self - int())
		.def(int() - py::self)
		.def(int() * py::self)
		.def(py::self * int())
		.def(-py::self)
		//pickle:
		.def(py::pickle(
			[](const LoadIndex& self) {
				return py::make_tuple(self.GetIndex());
			},
			[](const py::tuple& t) {
				CHECKandTHROW(t.size() == 1, "LoadIndex: loading data with pickle received invalid data structure!");
				LoadIndex self(py::cast<Index>(t[0]));
				return self;
			}))
		//representation:
		.def("__repr__", [](const LoadIndex &item) {
			return STDstring(EXUstd::ToString(item.GetIndex()));
		}, "return the string representation of the index, which can be, e.g., printed")
		;

	py::class_<SensorIndex>(m, "SensorIndex", "SensorIndex: index which may only be used for sensors")
		.def(py::init<>())
		.def(py::init<Index>())
		//+++++++++++++++++++++++++++++++++++++++++++
		//private: .def_readwrite("index", &SensorIndex::index)
		.def("GetTypeString", &SensorIndex::GetTypeString,
			"get type string for identification in Python")
		.def("GetIndex", &SensorIndex::GetIndex,
			"get index converted to index / int")
		.def("SetIndex", &SensorIndex::SetIndex,
			"set index with index / int")
		.def("__int__", [](const SensorIndex &item) {
			return item.GetIndex();
		}, "return the integer representation of the index")
		.def("__index__", [](const SensorIndex &item) {
			return item.GetIndex();
		}, "return the integer representation of the index; consistent with Python 3.8+")
		//add operators:
		.def(py::self + int())
		.def(int() + py::self)
		.def(py::self - int())
		.def(int() - py::self)
		.def(int() * py::self)
		.def(py::self * int())
		.def(-py::self)
		//pickle:
		.def(py::pickle(
			[](const SensorIndex& self) {
				return py::make_tuple(self.GetIndex());
			},
			[](const py::tuple& t) {
				CHECKandTHROW(t.size() == 1, "SensorIndex: loading data with pickle received invalid data structure!");
				SensorIndex self(py::cast<Index>(t[0]));
				return self;
			}))
		//representation:
		.def("__repr__", [](const SensorIndex &item) {
			return STDstring(EXUstd::ToString(item.GetIndex()));
		}, "return the string representation of the index, which can be, e.g., printed")
		;
        
	////++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//NOT needed, should always convert to std::vector<NodeIndex>
	//py::class_<ArrayNodeIndex>(m, "ArrayNodeIndex", "ArrayNodeIndex: array of indices which may only be used for nodes")
	//	.def(py::init<>())
	//	.def(py::init<std::vector<Index>>())
	//	.def(py::init<std::vector<NodeIndex>>())
	//	//+++++++++++++++++++++++++++++++++++++++++++
	//	.def("GetTypeString", &ArrayNodeIndex::GetTypeString,
	//		"get type string for identification in Python")
	//	.def("GetArrayIndex", &ArrayNodeIndex::GetArrayIndex,
	//		"get index converted to index / int")
	//	.def("SetArrayIndex", &ArrayNodeIndex::SetArrayIndex,
	//		"set index with index / int")
	//	//.def("__list__", [](const ArrayNodeIndex &item) {
	//	//	return item.GetArrayIndex();
	//	//}, "return the integer representation of the index array")
	//	.def("__repr__", [](const ArrayNodeIndex &item) {
	//		return STDstring(EXUstd::ToString(ArrayIndex(item.GetArrayIndex())));
	//	}, "return the string representation of the index array, which can be, e.g., printed")
	//	;


	//+++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++
	//MAINSYSTEMCONTAINER
	py::class_<MainSystemContainer>(m, "SystemContainer", "SystemContainer: Contains a set of (multibody) systems")
		.def(py::init<>())
		//+++++++++++++++++++++++++++++++++++++++++++
		//System functions:
		.def("AddSystem", &MainSystemContainer::AddMainSystem, "add a new computational system", py::return_value_policy::reference)

		.def("AppendSystem", &MainSystemContainer::AppendMainSystem, "append an existing computational system")

		.def_property("visualizationSettings", &MainSystemContainer::PyGetVisualizationSettings, &MainSystemContainer::PySetVisualizationSettings)//, py::return_value_policy::reference)

		.def("GetRenderState", &MainSystemContainer::PyGetRenderState, "Get dictionary with current render state (openGL zoom, modelview, etc.)")

		.def("SetRenderState", &MainSystemContainer::PySetRenderState, "Set current render state (openGL zoom, modelview, etc.) with given dictionary; usually, this dictionary has been obtained with GetRenderState")

		.def("WaitForRenderEngineStopFlag", &MainSystemContainer::WaitForRenderEngineStopFlag, "Wait for user to stop render engine (CTRL+Q)")

		.def("RenderEngineZoomAll", &MainSystemContainer::PyZoomAll, "Send zoom all signal, which will perform zoom all at next redraw request")
		
		.def("RedrawAndSaveImage", &MainSystemContainer::RedrawAndSaveImage, "Redraw openGL scene and save image (command waits until process is finished)")

		.def("AttachToRenderEngine", &MainSystemContainer::AttachToRenderEngine, "Links the SystemContainer to the render engine, such that the changes in the graphics structure drawn upon updates, etc.; done automatically on creation of SystemContainer; return False, if no renderer exists (e.g., compiled without GLFW) or cannot be linked (if other SystemContainer already linked)")

		.def("DetachFromRenderEngine", &MainSystemContainer::DetachFromRenderEngine, "Releases the SystemContainer from the render engine; return True if successfully released, False if no GLFW available or detaching failed")

		.def("SendRedrawSignal", &MainSystemContainer::SendRedrawSignal, "This function is used to send a signal to the renderer that all MainSystems (mbs) shall be redrawn")

		.def("GetCurrentMouseCoordinates", &MainSystemContainer::PyGetCurrentMouseCoordinates, "Get current mouse coordinates as list [x, y]; x and y being floats, as returned by GLFW, measured from top left corner of window; use GetCurrentMouseCoordinates(True) to obtain OpenGLcoordinates of projected plane", py::arg("useOpenGLcoordinates") = true)

		.def("Reset", &MainSystemContainer::Reset, "delete all systems and reset SystemContainer (including graphics)") 

		.def("NumberOfSystems", [](const MainSystemContainer& msc) {return msc.GetMainSystems().NumberOfItems(); }, "get number of MainSystems")

		.def("GetSystem", &MainSystemContainer::GetMainSystem, "Get main system i from system container", py::return_value_policy::reference) //added reference options as otherwise system is copied

		.def("GetDictionary", &MainSystemContainer::GetDictionary, "Get dictionary which represents system container; for pickle and copy")

		.def("SetDictionary", &MainSystemContainer::SetDictionary, "Set class with dictionary which represents system container; for pickle and copy")

		.def(py::pickle(
			[](const MainSystemContainer& self) {
				return py::make_tuple(self.GetDictionary());
			},
			[](const py::tuple& t) {
				CHECKandTHROW(t.size() == 1, "MainSystem: loading data with pickle received invalid data structure!");
				MainSystemContainer* self = new MainSystemContainer();
				self->SetDictionary(py::cast<py::dict>(t[0]));
				return self;
			}))

		.def("__repr__", [](const MainSystemContainer &item) {
			STDstring str = "SystemContainer:\n";

			for (Index i = 0; i < item.GetMainSystems().NumberOfItems(); i++)
			{
				str += "System " + EXUstd::ToString(i) + ": <systemData:\n";
				str += item.GetMainSystems()[i]->GetMainSystemData().PyInfoSummary() + ">\n";
			}
			return str;
		}, "return the representation of the systemContainer, which can be, e.g., printed")

		;

	m.attr("__version__") = EXUstd::exudynVersion;
}





