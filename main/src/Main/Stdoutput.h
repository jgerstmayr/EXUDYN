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
	bool writeToConsole; //redirect all output to console
	std::ofstream file;  //this is the filename for redirecting all output
	Index waitMilliSeconds; //wait this amount of milliseconds in order that spyder can print messages
public:
	OutputBuffer() 
	{ 
		setbuf(0, 0); //this leads to an overflow in any access to stringbuf!
		writeToFile = false;
		writeToConsole = true;
		waitMilliSeconds = 0;
	} 
	//! virtual int sync(); //this solution does not work!
	virtual int overflow(int c = EOF);

	//! function which allows to write asynchronuously during visualization thread; requires lateron call of pout in main thread (to clear buffer!)
	virtual void WriteVisualization(const STDstring& string);

	//! set delay added to writing in order to resolve problems of some ipython consoles
	virtual void SetDelayMilliSeconds(Index delayMilliSeconds) { waitMilliSeconds = delayMilliSeconds; }

	//! activate/deactivate writing to console
	virtual void SetWriteToConsole(bool flag) { writeToConsole = flag; }

	//! activate/deactivate writing to file
	virtual void SetWriteToFile(STDstring filename, bool flagWriteToFile = true, bool flagAppend = false);

	//! suspend writing to console/file with flag=true; needs to be set to false, otherwise writing to console is fully stopped
	virtual void SetSuspendWriting(bool flag) { suspendWriting = flag; }
};

void SysError(std::string error_msg); //!< prints a formated system (inernal) error message (+log file, etc.); 'error_msg' shall only contain the error information, do not write "ERROR: ..." or similar

void PyError(std::string error_msg); //!< prints a formated python error message (+log file, etc.); 'error_msg' shall only contain the error information, do not write "Python ERROR: ..." or similar

void PyWarning(std::string warning_msg); //!< prints a formated python warning message (+log file, etc.); 'warning_msg' shall only contain the warning information, do not write "Python WARNING: ..." or similar

void SysError(std::string error_msg, std::ofstream& file); //!< prints a formated system (inernal) error message (+log file, etc.); 'error_msg' shall only contain the error information, do not write "ERROR: ..." or similar; additionally writes to file if file.is_open()=true

void PyError(std::string error_msg, std::ofstream& file); //!< prints a formated python error message (+log file, etc.); 'error_msg' shall only contain the error information, do not write "Python ERROR: ..." or similar; additionally writes to file if file.is_open()=true

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
