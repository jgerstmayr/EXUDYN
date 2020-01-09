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
#include <thread>
//#include <pybind11/stl.h>
//#include <pybind11/stl_bind.h>
//#include <pybind11/operators.h>
//#include <pybind11/numpy.h>
//does not work globally: #include <pybind11/iostream.h> //used to redirect cout:  py::scoped_ostream_redirect output;
//#include <pybind11/cast.h> //for arguments
//#include <pybind11/functional.h> //for functions
namespace py = pybind11;

//these two variables become global
OutputBuffer outputBuffer; //this is my customized output buffer, which can redirect the output stream;
std::ostream pout(&outputBuffer);  // link ostream pout to buffer; pout behaves the same as std::cout



//! used to print to python; string is temporary stored and written as soon as '\n' is detected
int OutputBuffer::overflow(int c)
{
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
		if (flagAppend) { file.open(filename, std::ofstream::app); }
		else { file.open(filename, std::ofstream::out); }
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
	STDstring fileName;
	Index lineNumber;
	PyGetCurrentFileInformation(fileName, lineNumber);

	pout << "\nUser ERROR [file '" << fileName << "', line " << lineNumber << "]: \n";
	pout << error_msg << "\n\n";
	PyErr_SetString(PyExc_RuntimeError, "Exudyn: parsing of python file terminated due to python (user) error");

	if (file.is_open())
	{
		file << "\nUser ERROR [file '" << fileName << "', line " << lineNumber << "]: \n";
		file << error_msg << "\n\n";
		file << "Exudyn: parsing of python file terminated due to python (user) error\n\n";
		file << "********************************************************************\n\n";
	}
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
	STDstring fileName;
	Index lineNumber;
	PyGetCurrentFileInformation(fileName, lineNumber);

	pout << "\nSYSTEM ERROR [file '" << fileName << "', line " << lineNumber << "]: \n";
	pout << error_msg << "\n\n";
	PyErr_SetString(PyExc_RuntimeError, "Exudyn: parsing of python file terminated due to system error");

	if (file.is_open())
	{
		file << "\nSYSTEM ERROR [file '" << fileName << "', line " << lineNumber << "]: \n";
		file << error_msg << "\n\n";
		file << "Exudyn: parsing of python file terminated due to system error\n\n";
		file << "********************************************************************\n\n";
	}
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

