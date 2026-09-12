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
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */


//#include "Main/stdoutput.h"
#include "Utilities/BasicDefinitions.h" //includes stdoutput.h
#include "Utilities/BasicFunctions.h"	//includes stdoutput.h
#include "Utilities/AdvancedStuff.h"
#include "Utilities/ResizableArray.h"	
#include <chrono> //sleep_for()
#include <fstream>    

#include <pybind11/pybind11.h>
#include <pybind11/eval.h>
#include <thread>
#include <pybind11/stl.h>
//#include <pybind11/stl_bind.h>
//#include <pybind11/operators.h>
//#include <pybind11/numpy.h>
//does not work globally: #include <pybind11/iostream.h> //used to redirect cout:  py::scoped_ostream_redirect output;
//#include <pybind11/cast.h> //for arguments
#include <pybind11/functional.h> //for functions
#include <atomic> //for output buffer semaphore

#include "Utilities/TimerStructure.h"

namespace py = pybind11;
using namespace pybind11::literals; //brings in the '_a' literals; e.g. for short arguments definition

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
		returnValue = !filesystemNamespace::create_directories(pathStr);
	}
#endif

	return returnValue;
}







//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//global variable for timers:
std::vector<Real>* globalTimersCounters = nullptr; //global scalar variables get initialized with zero, this works on all platforms ... (arrays are not initialized ...)!
std::vector<const char*>* globalTimersCounterNames = nullptr;
TimerStructure globalTimers(4.5e-08); //offset added to correct measurements (i9: 4.521e-08); may lead to negative timings!

//! initialize timers at first call to RegisterTimer, whatever library is doing that (unordered! depends on compiler / Windows/Linux/...)
void TimerStructure::Initialize()
{
	if (globalTimersCounters == nullptr)
	{
		globalTimersCounters = new std::vector<Real>();
	}
	if (globalTimersCounterNames == nullptr)
	{
		globalTimersCounterNames = new std::vector<const char*>();
	}
}
//! create a new timer; name must be a static name (must exist until end of timer) or dynamically allocated string, may not be deleted
Index TimerStructure::RegisterTimer(const char* name)
{
	Initialize(); //called upon every registration; this is needed, because it is unclear, which function is called first!
	Index n = (Index)globalTimersCounters->size();
	globalTimersCounters->push_back(0.);
	globalTimersCounterNames->push_back(name);
	return n;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//these two variables become global
OutputBuffer outputBuffer; //this is my customized output buffer, which can redirect the output stream;
std::ostream pout(&outputBuffer);  // link ostream pout to buffer; pout behaves the same as std::cout

bool globalPyRuntimeErrorFlag = false; //this flag is set true as soon as a PyError or SysError is raised; this causes to shut down secondary processes, such as graphics, etc.
bool deactivateGlobalPyRuntimeErrorFlag = false; //this flag is set true as soon as functions are called e.g. from command windows, which allow errors without shutting down the renderer
std::atomic_flag outputBufferAtomicFlag = ATOMIC_FLAG_INIT;   //!< flag, which is used to lock access to outputBuffer

//! used to print to python; string is temporary stored and written as soon as '\n' is detected
int OutputBuffer::overflow(int c)
{
	return overflowFlush(c);
}

//if flushOnly or clearBuffer, c is ignored! in case of clearBuffer, the buffer is printed like having "\n" in the string
int OutputBuffer::overflowFlush(int c, bool flushOnly, bool clearBuffer)
{
	EXUstd::WaitAndLockSemaphoreIgnore(outputBufferAtomicFlag); //lock outputBuffer
	if ((char)c != '\n' && !flushOnly && !clearBuffer) {
		buf.push_back((char)c);
	}
	else 
	{
		if (!suspendWriting)
		{
			if (!flushOnly && !clearBuffer) { buf.push_back('\n'); }
			if (visualizationBuffer.size())
			{
				for (char visChar: visualizationBuffer)
				{
					buf.push_back(visChar);
				}
				visualizationBuffer.clear(); //erases memory, same as visualizationBuffer = ""
			}

			if (buf.size())
			{
				if (writeToConsole)
				{
					//if python raised already an error, exudyn.Print() will not work, because py::print() still has the exception error_already_set
					// ==> therefore add try and catch to print, such that subsequent print commands work again
					try
					{
						py::print(buf, "end"_a = "", "flush"_a = (writeFlushAlways || flushOnly));
					}
					catch (py::error_already_set& eas)
					{
						// Discard the Python error: for future print commands?
						eas.discard_as_unraisable(__func__); //prints long message ...
						//py::print(buf); 
						py::print(buf, "end"_a = "");//try again to print, which should work now
					}

					if (waitMilliSeconds) {
						std::this_thread::sleep_for(std::chrono::milliseconds(waitMilliSeconds)); //add this to enable Spyder to print messages
					}
				}
				if (writeToFile)
				{
					//file << buf << "\n"; //add "\n" as compared to py::print, which already adds end line command
					file << buf;
					if (writeFlushAlways || flushOnly)        //now open file with new file name
					{
						file.flush();
					}
				}
				buf.clear();
			}
		}
		else
		{
			buf.push_back((char)c);
		}
	}
	//py::print((char)c); //this would be much slower as each character needs to be processed with py::print
	EXUstd::ReleaseSemaphore(outputBufferAtomicFlag); //clear outputBuffer
	return c;
}

//! function which allows to write asynchronuously during visualization thread; requires lateron call of pout in main thread (to clear buffer!)
void OutputBuffer::WriteVisualization(const STDstring& string)
{
	EXUstd::WaitAndLockSemaphoreIgnore(outputBufferAtomicFlag); //lock outputBuffer
	visualizationBuffer += string;
	EXUstd::ReleaseSemaphore(outputBufferAtomicFlag); //clear outputBuffer
}

void OutputBuffer::SetWriteToFile(STDstring filename, bool flagWriteToFile, bool flagAppend, bool flagFlushAlways)
{
	writeToFile = flagWriteToFile;
	writeAppend = flagAppend;
	writeFlushAlways = flagFlushAlways;
	writeFilename = filename;

	if (writeToFile) //if file is already open, close it!
	{
		file.close();
	}
	if (flagWriteToFile)        //now open file with new file name
	{
		CheckPathAndCreateDirectories(filename);

		if (writeAppend)
		{ 
			file.open(writeFilename, std::ofstream::app);
		}
		else 
		{ 
			file.open(writeFilename, std::ofstream::out);
		}
	}
}


void PyGetCurrentFileInformation(std::string& fileName, Index& lineNumber) //!< retrieve current parsed file information from python (for error/warning messages...)
{
	try
	{
		py::module inspect = py::module::import("inspect");
		py::object currentFrame = inspect.attr("currentframe")();
		lineNumber = int(py::int_(currentFrame.attr("f_lineno")));

		//python usage: fn = inspect.getframeinfo(inspect.currentframe()).filename
		py::object frameInfo = inspect.attr("getframeinfo")(currentFrame);
		fileName = std::string(py::str(frameInfo.attr("filename")));

		//py::print(std::string("info has been called from: ") + fileName + std::string(" at line ") + EXUstd::ToString(line));
	}
	catch (...) //any other exception
	{
		fileName = "unknown file";
		lineNumber = 0;
	}
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
		file << "Exudyn: parsing of Python file terminated due to python (user) error\n\n";
		file << "********************************************************************\n\n";
	}
	//PyErr_SetString(PyExc_RuntimeError, "Exudyn: parsing of python file terminated due to python (user) error");
	throw std::runtime_error("Exudyn: parsing of Python file terminated due to Python (user) error");
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
	//globalPyRuntimeErrorFlag = true; //stop graphics, etc.

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
	throw std::runtime_error("Exudyn: parsing of Python file terminated due to system error");
	//PyErr_SetString(PyExc_RuntimeError, "Exudyn: parsing of python file terminated due to system error");
}

//!< prints a formated warning message (+log file, etc.); 'warning_msg' shall only contain the warning information, do not write "Python WARNING: ..." or similar
void PyWarning(std::string warning_msg) 
{
	std::ofstream dummy; //ofstream which is not active
	PyWarning(warning_msg, dummy);
}

bool suppressWarnings = false; //!< global flag to suppress warnings
							  //! prints a formated warning message (+log file, etc.); 'warning_msg' shall only contain the warning information, do not write "Python WARNING: ..." or similar
//! additional output to file
void PyWarning(std::string warning_msg, std::ofstream& file)
{
	if (!suppressWarnings)
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
}

