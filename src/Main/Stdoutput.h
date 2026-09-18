/** ***********************************************************************************************
* @file         stdoutput.h
* @brief
* @details		Details: externals which provide directives for output, error and warning messages
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



#include <sstream>      // std::stringbuf
#include <iostream>     // std::cout, std::ostream
//#include <iosfwd>		//forward declaration of ofstream; hopefully takes less compile time than fstream ... as this file is included in every .cpp file!!!
#include <fstream>      // needed for outputbuffer write to file ...
#include <functional> //! AUTO: needed for std::function
//#include <atomic> //for output buffer semaphore

//! buffer which enables output to python and/or to file
class OutputBuffer : public std::stringbuf //uses solution of so:redirect-stdcout-to-a-custom-writer
{
private:
	std::string buf;     //this buffer is used until end of line is detected
	std::string visualizationBuffer;     //this buffer is used in visualization thread => does not call Python functions!
	bool suspendWriting; //this flag is used to suspend writing via Python, e.g., during parallel computation
	bool writeToFile;    //redirect all output to file
	bool writeFlushAlways; //flush immediately after write; used to simplify readout
	bool writeAppend;    //append to file
	bool writeToConsole; //redirect all output to console
	std::string writeFilename;//filename for re-open during flush
	std::ofstream file;  //this is the file for redirecting all output
	Index waitMilliSeconds; //wait this amount of milliseconds in order that spyder can print messages
public:
	OutputBuffer() 
	{ 
		setbuf(0, 0); //this leads to an overflow in any access to stringbuf!
		suspendWriting = false;
		writeToFile = false;
		writeToConsole = true;
		waitMilliSeconds = 0;
		writeFlushAlways = false;
		writeAppend = false;
	} 
	//! override overflow in std::stringbuf; alternative: virtual int sync(); ==> this solution does not work!
	virtual int overflow(int c = EOF);

	//! special overflow with option to flush, ignoring char
	virtual int overflowFlush(int c = EOF, bool flushOnly=false, bool clearBuffer=false);

	//! function which allows to write asynchronuously during visualization thread; requires lateron call of pout in main thread (to clear buffer!)
	virtual void WriteVisualization(const STDstring& string);

	//! set delay added to writing in order to resolve problems of some ipython consoles
	virtual void SetDelayMilliSeconds(Index delayMilliSeconds) { waitMilliSeconds = delayMilliSeconds; }
	virtual Index GetDelayMilliSeconds() const { return waitMilliSeconds; }

	//! activate/deactivate writing to file
	virtual void SetWriteToFile(STDstring filename, bool flagWriteToFile = true, bool flagAppend = false, bool flagFlushAlways = false);
	virtual void SetFlushAlways(bool flag) { writeFlushAlways = flag; }
	virtual bool GetFlushAlways() const { return writeFlushAlways; }
	//this cannot be changed on the fly: virtual void SetWriteAppend(bool flag) { writeAppend = flag; }
	virtual bool GetWriteAppend() const { return writeAppend; }
	//this cannot be changed on the fly: virtual void SetWriteToFile(bool flag) { writeToFile = flag; }
	virtual bool GetWriteToFile() const { return writeToFile; }
	virtual std::string GetFileName() const { return writeFilename; }
	virtual void SetWriteToConsole(bool flag) { writeToConsole = flag; }
	virtual bool GetWriteToConsole() const { return writeToConsole; }

	//! write text to the log file and NOT to the console; does nothing if no log file is open (#2530)
	virtual void WriteToFileOnly(const std::string& text);

	//! suspend writing to console/file with flag=true; needs to be set to false, otherwise writing to console is fully stopped
	virtual void SetSuspendWriting(bool flag) { suspendWriting = flag; }
};

//! which Python exception an error becomes. Everything was a RuntimeError until revision2026 step
//! R6.7; a wrong TYPE and a wrong VALUE are told apart since (#2432), and the Exudyn exception
//! classes of #2516 are named here as well (#2521, step R6.3.3). The classes themselves are in
//! ReleaseAssert.h; this enum exists because PyError and SysError are compiled functions and
//! cannot be templated on the class the way the CHECKandTHROW macros are.
enum class PyErrorType
{
	runtimeError,       //!< the default, and what every call site outside PyConversion.h still uses
	typeError,          //!< the object cannot be this parameter at all: None, a string for a number, an index of the wrong kind
	valueError,         //!< the kind is right, the value is not: out of range, wrong size, a placeholder left in place
	modelError,         //!< the model is illegal: a combination that cannot work, a setting that contradicts another
	solverError,        //!< the solver cannot continue: singular matrix, no convergence, divergence
	internalError,      //!< an Exudyn invariant broke; the message is for a developer reading a user's log
	notImplementedError,//!< the feature or the combination does not exist; neither a mistake nor a bug
	indexError,         //!< an index outside its range
	arithmeticError     //!< division by zero, sqrt of a negative number
};

[[noreturn]] void ThrowPyErrorType(PyErrorType errorType, const char* message); //!< the one place that turns a PyErrorType into a throw (#2521); the classes are in ReleaseAssert.h

std::string ErrorMessageWithLocation(const std::string& message, const std::string& fileName, Index lineNumber); //!< what an exception carries: the detail and the user's Python location (#2527)

std::string ErrorMessageBlock(const char* heading, const std::string& message, const std::string& fileName, Index lineNumber); //!< the block a log file records; every channel writes exactly this text (#2530)

void SysError(std::string error_msg, PyErrorType errorType = PyErrorType::internalError); //!< prints a formated system (internal) error message (+log file, etc.); 'error_msg' shall only contain the error information, do not write "ERROR: ..." or similar; errorType selects the Python exception and defaults to what SysError means (#2521)

void PyError(std::string error_msg, PyErrorType errorType = PyErrorType::runtimeError); //!< prints a formated python error message (+log file, etc.); 'error_msg' shall only contain the error information, do not write "Python ERROR: ..." or similar; errorType selects the Python exception (#2432)

void PyWarning(std::string warning_msg); //!< prints a formated python warning message (+log file, etc.); 'warning_msg' shall only contain the warning information, do not write "Python WARNING: ..." or similar

void StopRendererOnError(); //!< raise globalPyRuntimeErrorFlag, which shuts the renderer down, unless the renderer itself asked for errors to be survivable (#2531)

void PyDeprecated(std::string message); //!< raises a Python DeprecationWarning: filterable, promotable with -W error::DeprecationWarning, and reported once per source location instead of on every call (#2522)

//NOTE there is no PyError/SysError overload taking an ofstream any more (#2538, revision2026 step
//R6.8): an error that ends a solver run is written to the solver file by CSolverBase::SolveSystem,
//which catches it where the file is known. That covers every helper, including the 1100+ macro
//sites that could never pass a file. PyWarning keeps the overload, because a warning throws
//nothing and that catch can never see it.
void PyWarning(std::string warning_msg, std::ofstream& file); //!< prints a formated python warning message (+log file, etc.); 'warning_msg' shall only contain the warning information, do not write "Python WARNING: ..." or similar; additionally writes to file if file.is_open()=true


void PyGetCurrentFileInformation(std::string& fileName, Index& lineNumber); //!< retrieve current parsed file information from python (for error/warning messages...)

//********************************
extern std::ostream pout;  //!< provide a output stream (e.g. for Python); remove the following line if linkage to Python is not needed!
extern OutputBuffer outputBuffer;  //!< link outputBuffer to change options
//alternatively use:
//#define pout std::cout
//********************************

//! check if directory of whole path+filename exists; return false, if fails
//! this function requires C++17 std libraries
//! works with local path
bool CheckPathAndCreateDirectories(const STDstring& pathAndFileName);

extern STDstring outputDirectory; //!< global directory prepended to written files; exudyn.config.outputDirectory (#2418)

//! prepend outputDirectory to fileName (solution, solver information, sensor and image files);
//! raises an exception if fileName is an absolute path while outputDirectory is set, because the
//! two would contradict each other; returns fileName unchanged if outputDirectory is empty
STDstring ResolveOutputFileName(const STDstring& fileName);
