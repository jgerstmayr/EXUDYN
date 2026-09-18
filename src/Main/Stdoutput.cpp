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

//! see Stdoutput.h; set from Python as exudyn.config.outputDirectory (#2418)
STDstring outputDirectory = "";

STDstring ResolveOutputFileName(const STDstring& fileName)
{
	if (outputDirectory.empty() || fileName.empty()) { return fileName; }

	//absolute paths: '/...', '\...', '\\server\...' and 'C:\...'; checked here and not before the
	//run, so that the error names the file that is actually opened
	bool isAbsolute = (fileName[0] == '/' || fileName[0] == '\\' ||
		(fileName.size() > 1 && fileName[1] == ':'));
	if (isAbsolute)
	{
		throw EXUexception(STDstring("exudyn.config.outputDirectory is set to '") + outputDirectory +
			"', but the file name '" + fileName + "' is an absolute path; use a relative file name "
			"or reset exudyn.config.outputDirectory = ''");
	}

	char last = outputDirectory[outputDirectory.size() - 1];
	if (last == '/' || last == '\\') { return outputDirectory + fileName; }
	return outputDirectory + "/" + fileName;
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

//! write text to the log file and NOT to the console (#2530). An error block belongs in the file of a
//! long unattended run, where it is the only record; on the console the exception says the same thing
//! already, and printing it there means a CAUGHT exception still floods the terminal.
void OutputBuffer::WriteToFileOnly(const std::string& text)
{
	if (!writeToFile) { return; }

	//whatever is half-written must reach the file first, or the block lands in the middle of a line;
	//overflowFlush takes the semaphore itself, so it may not be held here
	overflowFlush(EOF, true);

	EXUstd::WaitAndLockSemaphoreIgnore(outputBufferAtomicFlag); //lock outputBuffer
	if (file.is_open())
	{
		file << text;
		file.flush(); //an error is exactly what a truncated log loses
	}
	EXUstd::ReleaseSemaphore(outputBufferAtomicFlag); //clear outputBuffer
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
	//exudyn.Print writes an output of the run as well, so it follows outputDirectory (#2454)
	writeFilename = ResolveOutputFileName(filename);

	if (writeToFile) //if file is already open, close it!
	{
		file.close();
	}
	if (flagWriteToFile)        //now open file with new file name
	{
		CheckPathAndCreateDirectories(writeFilename);

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


//! the directory of the shipped exudyn package, asked once; used to decide which Python frame is
//! the USER's (#2524, revision2026 step R6.3.5)
const std::string& ExudynPackageDirectory()
{
	static std::string directory = []() -> std::string
	{
		try
		{
			std::string file = py::cast<std::string>(py::module::import("exudyn").attr("__file__"));
			for (char& character : file) { if (character == '\\') { character = '/'; } }
			for (char& character : file) { character = (char)tolower((unsigned char)character); }
			size_t position = file.find_last_of('/');
			return position == std::string::npos ? std::string() : file.substr(0, position + 1);
		}
		catch (...) { return std::string(); }
	}();

	return directory;
}

void PyGetCurrentFileInformation(std::string& fileName, Index& lineNumber) //!< retrieve current parsed file information from python (for error/warning messages...)
{
	try
	{
		py::module inspect = py::module::import("inspect");
		py::object frame = inspect.attr("currentframe")();

		//WHY currentframe() AND NOT THE C++ STACK: this is the only way to learn which line of
		//PYTHON is executing, and inside a user function - springForceUserFunction and its kin -
		//that is the only line worth naming. Nothing on the C++ side knows it.
		//WHY THE WALK: the innermost frame is not always the user's. During mbs.SolveDynamic() it is
		//exudyn/solver.py, so every error raised from a solver run used to name the solver wrapper
		//and a line the user has never seen (#2524). So walk outwards to the first frame that is not
		//inside the shipped package - and if every frame is inside it, keep the innermost one, which
		//is the best answer available.
		//getframeinfo() is deliberately NOT used: it scans sys.modules and READS THE SOURCE FILE,
		//which cost 9 s in a model that provokes 38000 errors (#2423), and f_code.co_filename says
		//the same thing.
		const std::string& packageDirectory = ExudynPackageDirectory();
		py::object chosen = frame;
		py::object current = frame;

		while (!current.is_none())
		{
			std::string name = py::cast<std::string>(current.attr("f_code").attr("co_filename"));
			std::string normalized = name;
			for (char& character : normalized) { if (character == '\\') { character = '/'; } }
			for (char& character : normalized) { character = (char)tolower((unsigned char)character); }

			if (packageDirectory.empty() || normalized.compare(0, packageDirectory.size(), packageDirectory) != 0)
			{
				chosen = current;
				break;
			}
			current = current.attr("f_back");
		}

		fileName = py::cast<std::string>(chosen.attr("f_code").attr("co_filename"));
		lineNumber = int(py::int_(chosen.attr("f_lineno")));
	}
	catch (...) //any other exception
	{
		fileName = "unknown file";
		lineNumber = 0;
	}
}

//! the ONE place that turns a PyErrorType into a throw (#2521, revision2026 step R6.3.3). The
//! Exudyn classes are in ReleaseAssert.h and reach Python as the classes registered in
//! PybindModule.cpp; py::type_error and py::value_error are pybind11 builtins and become the plain
//! Python TypeError/ValueError. [[noreturn]] so that every caller of it ends the same way.
[[noreturn]] void ThrowPyErrorType(PyErrorType errorType, const char* message)
{
	switch (errorType)
	{
	//These were py::type_error and py::value_error - pybind11 builtins, which become the PLAIN
	//Python TypeError and ValueError - from step R6.7 until step R6.3.6 (#2528). They now throw the
	//Exudyn classes, which derive from those built-ins AND from exudyn.ExudynError: an existing
	//"except TypeError" keeps working and "except exudyn.ExudynError" starts working. That is the
	//maintainer's decision of 2026-09-18, "wrap everything as ExudynError".
	case PyErrorType::typeError:           throw ExudynTypeError(message);
	case PyErrorType::valueError:          throw ExudynValueError(message);
	case PyErrorType::modelError:          throw ExudynModelError(message);
	case PyErrorType::solverError:         throw ExudynSolverError(message);
	case PyErrorType::internalError:       throw ExudynInternalError(message);
	case PyErrorType::notImplementedError: throw ExudynNotImplementedError(message);
	case PyErrorType::indexError:          throw ExudynIndexError(message);
	case PyErrorType::arithmeticError:     throw ExudynArithmeticError(message);
	default:                               throw std::runtime_error(message);
	}
}

//! THE CAUSE OF AN EXUDYN EXCEPTION (#2537, revision2026 step R6.3.8); declared in
//! ExceptionsTemplates.h next to the handlers that fill it. thread_local because two threads can be
//! inside Exudyn at once (the renderer calls Python of its own), and a raw PyObject* rather than a
//! py::object because a thread_local py::object destructor would need the GIL at thread exit.
static thread_local PyObject* pendingExceptionCause = nullptr;

void SetPendingExceptionCause(PyObject* value)
{
	Py_XINCREF(value);
	Py_XDECREF(pendingExceptionCause);
	pendingExceptionCause = value;
}

void ClearPendingExceptionCause()
{
	Py_XDECREF(pendingExceptionCause);
	pendingExceptionCause = nullptr;
}

PyObject* PendingExceptionCause()
{
	return pendingExceptionCause;
}

//! The renderer reads globalPyRuntimeErrorFlag in five places (GlfwClient.cpp): it stops the render
//! loop and, more importantly, keeps the render thread from calling into Python while Python is in
//! an error state. This is the ONE place that knows the rule, so that every site which decides
//! "this error ends the run" says so the same way (#2531, revision2026 step R6.3.11).
//! deactivateGlobalPyRuntimeErrorFlag is set by rendererPythonInterface.cpp around calls the
//! renderer itself makes into Python, where an error may not take the window down.
void StopRendererOnError()
{
	if (!deactivateGlobalPyRuntimeErrorFlag) { globalPyRuntimeErrorFlag = true; }
}

//! the message an exception carries: what went wrong, and where the user's Python was (#2527)
std::string ErrorMessageWithLocation(const std::string& message, const std::string& fileName, Index lineNumber)
{
	if (fileName == "unknown file" || fileName.empty()) { return message; }

	return message + " [Python file '" + fileName + "', line " + EXUstd::ToString(lineNumber) + "]";
}

//! the block a log file records. Both channels - the pout log file and an explicit ofstream such as
//! the solver file - write exactly this text, so a run cannot be reconstructed differently depending
//! on which file is read (#2530). It replaces the old file-only sentence "Exudyn: parsing of Python
//! file terminated due to python (user) error", which was the same fixed line that step R6.3.5 took
//! out of the exception for saying nothing.
std::string ErrorMessageBlock(const char* heading, const std::string& message,
	const std::string& fileName, Index lineNumber)
{
	return STDstring("\n=========================================\n")
		+ heading + " [file '" + fileName + "', line " + EXUstd::ToString(lineNumber) + "]: \n"
		+ message + "\n"
		+ "=========================================\n\n";
}

//!< prints a formated error message (+log file, etc.); 'error_msg' shall only contain the error information, do not write "Python ERROR: ..." or similar
void PyError(std::string error_msg, PyErrorType errorType)
{
	std::ofstream dummy; //ofstream which is not active
	PyError(error_msg, dummy, errorType);
}

//!< prints a formated error message (+log file, etc.); 'error_msg' shall only contain the error information, do not write "Python ERROR: ..." or similar
//! additional output to file
void PyError(std::string error_msg, std::ofstream& file, PyErrorType errorType) 
{
	StopRendererOnError(); //stop graphics, etc.
	STDstring fileName;
	Index lineNumber;
	PyGetCurrentFileInformation(fileName, lineNumber);

	//NOT to the console (#2530, revision2026 step R6.3.10): the exception thrown below carries the
	//same message and the same location, so the console would say it twice - and a CAUGHT exception
	//would still say it, which is what floods the terminal of a GUI or a parameter variation that
	//handles its own errors. The log file is a different matter: on a long unattended run nothing
	//else records that this happened.
	STDstring block = ErrorMessageBlock("User ERROR", error_msg, fileName, lineNumber);
	outputBuffer.WriteToFileOnly(block);

	if (file.is_open())
	{
		file << block;
	}
	//WHAT IS THROWN CARRIES THE DETAIL (#2527, revision2026 step R6.3.5). Until now it was the fixed
	//sentence "Exudyn: parsing of Python file terminated due to Python (user) error", identical for
	//a bad item number, a string written into a number and a missing marker; the explanation was
	//printed above and then dropped, so str(exception) told a user nothing and an except block that
	//logs the message logged nothing. The location goes with it, because a caught exception is often
	//all that survives of a run.
	ThrowPyErrorType(errorType, ErrorMessageWithLocation(error_msg, fileName, lineNumber).c_str());
}

//!< prints a formated error message (+log file, etc.); 'error_msg' shall only contain the error information, do not write "Python ERROR: ..." or similar
void SysError(std::string error_msg, PyErrorType errorType)
{
	std::ofstream dummy; //ofstream which is not active
	SysError(error_msg, dummy, errorType);
}

//! prints a formated error message (+log file, etc.); 'error_msg' shall only contain the error information, do not write "Python ERROR: ..." or similar
//! additional output to file
void SysError(std::string error_msg, std::ofstream& file, PyErrorType errorType)
{
	StopRendererOnError(); //stop graphics, etc.

	STDstring fileName;
	Index lineNumber;
	PyGetCurrentFileInformation(fileName, lineNumber);

	//file only, for the reasons written at the same place in PyError (#2530)
	STDstring block = ErrorMessageBlock("SYSTEM ERROR", error_msg, fileName, lineNumber);
	outputBuffer.WriteToFileOnly(block);

	if (file.is_open())
	{
		file << block;
	}
	//an Exudyn invariant broke: exudyn.InternalError, which IS a RuntimeError, so an existing
	//"except RuntimeError" keeps catching it while the type now says "please report this" (#2521).
	//The message goes with it: an internal error that reaches a developer as a fixed sentence is a
	//bug report with the evidence removed (#2527)
	ThrowPyErrorType(errorType, ErrorMessageWithLocation(error_msg, fileName, lineNumber).c_str());
}

//!< prints a formated warning message (+log file, etc.); 'warning_msg' shall only contain the warning information, do not write "Python WARNING: ..." or similar
void PyWarning(std::string warning_msg)
{
	std::ofstream dummy; //ofstream which is not active
	PyWarning(warning_msg, dummy);
}

//! a deprecation is neither an error nor a line of output: it is a statement about the user's code
//! that the user must be able to act on (#2522, revision2026 step R6.3.4). Python has the machinery
//! for it, and a printed line has none of it:
//!   - "-W error::DeprecationWarning" turns every one of them into an exception, which is how a
//!     user finds them all before an Exudyn release removes the old name;
//!   - warnings.filterwarnings() silences one of them without silencing the others;
//!   - the same site reports ONCE instead of on every call - a deprecated setting read inside a
//!     time-step loop used to print thousands of identical lines.
//! PyErr_WarnEx does the per-location bookkeeping itself, from the Python frame, so unlike
//! PyWarning this does not call PyGetCurrentFileInformation (#2423) and costs nothing per call.
//! The GIL is held: every call site is a pybind-bound getter, setter or function.
extern bool suppressWarnings;   //defined below, next to PyWarning, which is its other reader

void PyDeprecated(std::string message)
{
	if (suppressWarnings) { return; } //an explicit request for silence is honoured here as well

	if (PyErr_WarnEx(PyExc_DeprecationWarning, message.c_str(), 1) != 0)
	{
		//the user turned this warning into an error; the Python exception is already set
		throw py::error_already_set();
	}
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

