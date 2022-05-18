/** ***********************************************************************************************
* @class		UnitTestBase
* @brief		UnitTestBase class handles Exudyn unit tests
* @details		Details:
                - unit tests using lest.hpp throw/catch mechanism
                - autoregistration of tests allows integration of tests into class defintion files
                - Perform...Tests functions return an integer value > 0 if fails occured
                - Flags for Perform...Tests are provided in enum class UnitTestFlags
                - output is appended to STDstring::testOutput

* @author		Gerstmayr Johannes
* @date			2018-05-02 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef UNITTESTBASE__H
#define UNITTESTBASE__H

#include "Utilities/BasicDefinitions.h"

//#define PERFORM_UNIT_TESTS //set with compiler flags in setup.py, but can be switched here for testing

#ifdef PERFORM_UNIT_TESTS
//run all unit tests; return 0 on success, otherwise the number of fails
extern Index RunUnitTests(bool reportOnPass, bool printOutput);

class UnitTestBase
{
private:
    STDstring testOutput;
public:
    //! perform all tests and return number of detected fails (real number of failed tests might be higher, because consecutive failures are not reported in sections.
    int PerformVectorAndArrayTests(int flags);

    STDstring& GetOutputString() { return testOutput; }
};


/** ***********************************************************************************************
* @class		UnitTestFlags
* @brief		Flags to handle behaviour of unit tests in class UnitTestBase.
* @warning      Flags are enumerated as 2^n, n being a consecutive number; this allows to combine flags.
* @author		Gerstmayr Johannes
* @date			2 May 2018
************************************************************************************************ */
namespace UnitTestFlags {
    enum {
        reportOnPass = 1,   //!< lest reports on pass (otherwise only failed tests are reported)
        reportSections = 2, //!< lest reports on sections
		writeToPout = 4,     //!< write report into cout
		writeToFile = 8     //!< write report into file
	};
};

#endif

#endif //include header once
