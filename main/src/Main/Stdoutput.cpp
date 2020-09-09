/** ***********************************************************************************************
* @file         stdoutput.cpp
* @brief
* @details		Details: externals which provide directives for output, error and warning messages
*				Here, the redirection goes to Python stream
*
* @author		Gerstmayr Johannes
* @date			2019-04-02 (generated)
* @pre			...
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
* 				
*
************************************************************************************************ */


//#include "Main/stdoutput.h"
#include "Utilities/BasicDefinitions.h" //includes stdoutput.h
#include "Utilities/BasicFunctions.h"	//includes stdoutput.h
#include <chrono> //sleep_for()
#include <fstream>    

#include <pybind11/pybind11.h>
#include <pybind11/eval.h>
#include <thread>
//#include <pybind11/stl.h>
//#include <pybind11/stl_bind.h>
//#include <pybind11/operators.h>
//#include <pybind11/numpy.h>
//does not work globally: #include <pybind11/iostream.h> //used to redirect cout:  py::scoped_ostream_redirect output;
//#include <pybind11/cast.h> //for arguments
//#include <pybind11/functional.h> //for functions
#include <atomic> //for output buffer semaphore

#include "Utilities/TimerStructure.h"

namespace py = pybind11;

//comment the following line, if C++17 or stdc++fs library are not available on your system!


//CHECK wheter predefined macros indicate that std::...::filesystem is available: __cpp_lib_filesystem and __cpp_lib_experimental_filesystem
#ifdef __cpp_lib_filesystem
	//VS2017, gcc 8.0, etc:
	#include <filesystem> //requires C++17 with filesystem implemented; linker needs "-lstdc++fs" on linux
	namespace filesystemNamespace = std::filesystem;
	#define USE_AUTOCREATE_DIRECTORIES
#else
	//for UBUNTU18.04 GCC version 7.5.0 does not implement std::filesystem and also does not have __cpp_lib_filesystem macro
	//works for GCC and VS2017
	#ifdef __has_include 
		#if __has_include (<filesystem>)
			#include <filesystem>
			#define USE_AUTOCREATE_DIRECTORIES
			namespace filesystemNamespace = std::filesystem;
		#elif __has_include (<experimental/filesystem>)
			#include <experimental/filesystem>
			#define USE_AUTOCREATE_DIRECTORIES
			namespace filesystemNamespace = std::experimental::filesystem;
		#endif
	#endif
#endif

//! check if directory of whole path+filename exists; return false, if fails
//! this function requires C++17 std libraries
//! works with local path
bool CheckPathAndCreateDirectories(const STDstring& pathAndFileName)
{
	bool returnValue = true;

#ifdef USE_AUTOCREATE_DIRECTORIES
	char key1 = '\\';
	char key2 = '/';

	std::size_t pos = std::string::npos;
	auto found1 = pathAndFileName.rfind(key1);
	auto found2 = pathAndFileName.rfind(key2);
	if (found1 != std::string::npos)
	{
		pos = found1;
	}
	if (found2 != std::string::npos)
	{
		//only use '/' key, if it is the last key
		if (pos == std::string::npos || pos < found2)
		{
			pos = found2;
		}
	}

	//now create dictionary
	if (pos != std::string::npos)
	{
		STDstring pathStr = pathAndFileName.substr(0, pos);
		returnValue = filesystemNamespace::create_directories(pathStr);
	}
#endif

	return returnValue;
}








//global variable for timers:
TimerStructure globalTimers;

//these two variables become global
OutputBuffer outputBuffer; //this is my customized output buffer, which can redirect the output stream;
std::ostream pout(&outputBuffer);  // link ostream pout to buffer; pout behaves the same as std::cout

bool globalPyRuntimeErrorFlag = false; //this flag is set true as soon as a PyError or SysError is raised; this causes to shut down secondary processes, such as graphics, etc.
bool deactivateGlobalPyRuntimeErrorFlag = false; //this flag is set true as soon as functions are called e.g. from command windows, which allow errors without shutting down the renderer
std::atomic_flag outputBufferAtomicFlag;   //!< flag, which is used to lock access to outputBuffer

std::atomic_flag queuedPythonExecutableCodeAtomicFlag;  //!< flag for executable python code
STDstring queuedPythonExecutableCodeStr;					//!< this string contains python code which shall be executed

//! used to print to python; string is temporary stored and written as soon as '\n' is detected
int OutputBuffer::overflow(int c)
{
	outputBufferAtomicFlag.test_and_set(std::memory_order_acquire); //lock outputBuffer
	if ((char)c != '\n') {
		buf.push_back((char)c);
	}
	else 
	{
		if (writeToConsole)
		{
			py::print(buf);
			if (waitMilliSeconds) {
				std::this_thread::sleep_for(std::chrono::milliseconds(waitMilliSeconds)); //add this to enable Spyder to print messages
			}
		}
		if (writeToFile)
		{
			file << buf << "\n"; //add "\n" as compared to py::print, which already adds end line command
		}

		buf.clear();
	}
	//py::print((char)c); //this would be much slower as each character needs to be processed with py::print
	outputBufferAtomicFlag.clear(std::memory_order_release); //clear outputBuffer
	return c;
}

void OutputBuffer::SetWriteToFile(STDstring filename, bool flagWriteToFile, bool flagAppend)
{
	if (writeToFile) //if file is already open, close it!
	{
		file.close();
	}
	if (flagWriteToFile)        //now open file with new file name
	{
		CheckPathAndCreateDirectories(filename);

		if (flagAppend) 
		{ 
			file.open(filename, std::ofstream::app); 
		}
		else 
		{ 
			file.open(filename, std::ofstream::out); 
		}
	}
	writeToFile = flagWriteToFile;
}


void PyGetCurrentFileInformation(std::string& fileName, Index& lineNumber) //!< retrieve current parsed file information from python (for error/warning messages...)
{
	py::module inspect = py::module::import("inspect");
	py::object currentFrame = inspect.attr("currentframe")();
	lineNumber = int(py::int_(currentFrame.attr("f_lineno")));

	//python usage: fn = inspect.getframeinfo(inspect.currentframe()).filename
	py::object frameInfo = inspect.attr("getframeinfo")(currentFrame);
	fileName = std::string(py::str(frameInfo.attr("filename")));

	//py::print(std::string("info has been called from: ") + fileName + std::string(" at line ") + EXUstd::ToString(line));
}

//!< prints a formated error message (+log file, etc.); 'error_msg' shall only contain the error information, do not write "Python ERROR: ..." or similar
void PyError(std::string error_msg)
{
	std::ofstream dummy; //ofstream which is not active
	PyError(error_msg, dummy);
}

//!< prints a formated error message (+log file, etc.); 'error_msg' shall only contain the error information, do not write "Python ERROR: ..." or similar
//! additional output to file
void PyError(std::string error_msg, std::ofstream& file) 
{
	if (!deactivateGlobalPyRuntimeErrorFlag) { globalPyRuntimeErrorFlag = true; } //stop graphics, etc.
	STDstring fileName;
	Index lineNumber;
	PyGetCurrentFileInformation(fileName, lineNumber);

	pout << "\n=========================================\n";
	pout << "User ERROR [file '" << fileName << "', line " << lineNumber << "]: \n";
	pout << error_msg << "\n";
	pout << "=========================================\n\n";

	if (file.is_open())
	{
		file << "\nUser ERROR [file '" << fileName << "', line " << lineNumber << "]: \n";
		file << error_msg << "\n\n";
		file << "Exudyn: parsing of python file terminated due to python (user) error\n\n";
		file << "********************************************************************\n\n";
	}
	PyErr_SetString(PyExc_RuntimeError, "Exudyn: parsing of python file terminated due to python (user) error");
	//this kills kernel in spyder: CHECKandTHROWstring("Exudyn: parsing of python file terminated due to python (user) error!");
}

//!< prints a formated error message (+log file, etc.); 'error_msg' shall only contain the error information, do not write "Python ERROR: ..." or similar
void SysError(std::string error_msg)
{
	std::ofstream dummy; //ofstream which is not active
	SysError(error_msg, dummy);
}

//! prints a formated error message (+log file, etc.); 'error_msg' shall only contain the error information, do not write "Python ERROR: ..." or similar
//! additional output to file
void SysError(std::string error_msg, std::ofstream& file) 
{
	if (!deactivateGlobalPyRuntimeErrorFlag) { globalPyRuntimeErrorFlag = true; }//stop graphics, etc.
	globalPyRuntimeErrorFlag = true; //stop graphics, etc.
	STDstring fileName;
	Index lineNumber;
	PyGetCurrentFileInformation(fileName, lineNumber);

	pout << "\n=========================================\n";
	pout << "SYSTEM ERROR [file '" << fileName << "', line " << lineNumber << "]: \n";
	pout << error_msg << "\n";
	pout << "=========================================\n\n";

	if (file.is_open())
	{
		file << "\nSYSTEM ERROR [file '" << fileName << "', line " << lineNumber << "]: \n";
		file << error_msg << "\n\n";
		file << "Exudyn: parsing of python file terminated due to system error\n\n";
		file << "********************************************************************\n\n";
	}
	PyErr_SetString(PyExc_RuntimeError, "Exudyn: parsing of python file terminated due to system error");
}

//!< prints a formated warning message (+log file, etc.); 'warning_msg' shall only contain the warning information, do not write "Python WARNING: ..." or similar
void PyWarning(std::string warning_msg) 
{
	std::ofstream dummy; //ofstream which is not active
	PyWarning(warning_msg, dummy);
}

//! prints a formated warning message (+log file, etc.); 'warning_msg' shall only contain the warning information, do not write "Python WARNING: ..." or similar
//! additional output to file
void PyWarning(std::string warning_msg, std::ofstream& file)
{
	STDstring fileName;
	Index lineNumber;
	PyGetCurrentFileInformation(fileName, lineNumber);

	pout << "\nPython WARNING [file '" << fileName << "', line " << lineNumber << "]: \n";
	pout << warning_msg << "\n\n";

	if (file.is_open())
	{
		file << "\nPython WARNING [file '" << fileName << "', line " << lineNumber << "]: \n";
		file << warning_msg << "\n\n";
	}
}

//! put executable string into queue, which is called from main thread
void PyQueueExecutableString(STDstring str) //call python function and execute string as python code
{
	queuedPythonExecutableCodeAtomicFlag.test_and_set(std::memory_order_acquire); //lock queuedPythonExecutableCodeStr
	queuedPythonExecutableCodeStr += '\n' + str; //for safety add a "\n", as the last command may include spaces, tabs, ... at the end
	queuedPythonExecutableCodeAtomicFlag.clear(std::memory_order_release); //clear queuedPythonExecutableCodeStr
}

void PyProcessExecuteQueue() //call python function and execute string as python code
{
	queuedPythonExecutableCodeAtomicFlag.test_and_set(std::memory_order_acquire); //lock queuedPythonExecutableCodeStr
	if (queuedPythonExecutableCodeStr.size())
	{
		STDstring execStr = queuedPythonExecutableCodeStr;
		queuedPythonExecutableCodeStr.clear();
		queuedPythonExecutableCodeAtomicFlag.clear(std::memory_order_release); //clear queuedPythonExecutableCodeStr
		deactivateGlobalPyRuntimeErrorFlag = true; //errors will not crash the render window

		try //catch exceptions; user may want to continue after a illegal python command 
		{
			py::object scope = py::module::import("__main__").attr("__dict__"); //use this to enable access to mbs and other variables of global scope within test models suite
			py::exec(execStr.c_str(), scope);
		}
		//mostly catches python errors:
		catch (const pybind11::error_already_set& ex)
		{
			PyWarning("Error when executing '" + STDstring(execStr) + "':\n" + STDstring(ex.what()) + "\n; maybe a module is missing!");
			deactivateGlobalPyRuntimeErrorFlag = false;
			throw; //avoid multiple exceptions trown again (don't know why!)!
			//throw(ex); //avoid multiple exceptions trown again (don't know why!)!
		}
		catch (const EXUexception& ex)
		{
			PyWarning("Error when executing '" + STDstring(execStr) + "':\n" + STDstring(ex.what()) + "\n; maybe a module is missing!!");
			deactivateGlobalPyRuntimeErrorFlag = false;
			throw; //avoid multiple exceptions trown again (don't know why!)!
			//throw(ex); //avoid multiple exceptions trown again (don't know why!)!
		}
		catch (...) //any other exception
		{
			PyWarning("Error when executing '" + STDstring(execStr) + "'\nmaybe a module is missing and check your python code!!");
		}
		deactivateGlobalPyRuntimeErrorFlag = false; 
	}
	else
	{
		queuedPythonExecutableCodeAtomicFlag.clear(std::memory_order_release); //clear queuedPythonExecutableCodeStr
	}
}

////this solution does not work!
//int OutputBuffer::sync() 
//{
//	//redirect stream "this->str()" to this buffer: 
//	//this could go to cout or any other output (e.g. Python)
//	//std::cout << this->str();
//	py::print(this->str());
//	py::print("test");
//	return 0;
//}

