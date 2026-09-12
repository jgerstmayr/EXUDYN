/** ***********************************************************************************************
* @file			PybindTests.h
* @brief		This file contains test functions for pybind11 
* @details		Details:
* 				- put tests here, which are useful for later developements
*
* @author		Gerstmayr Johannes
* @date			2019-05-02 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef PYBINDTESTS__H
#define PYBINDTESTS__H

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//PYBIND TESTS

#include "Linalg/BasicLinalg.h"

extern void PyTest();
//extern void PythonAlive();
extern void CreateTestSystem(Index systemNumber, Index arg0, Index arg1);



#endif
