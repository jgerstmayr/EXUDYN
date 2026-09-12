/** ***********************************************************************************************
* @file			BasicFunctions.cpp
* @brief		This file contains helper functions, not definitions and not traits;
* @details		Details:
* 				- Helper functions for manipulating arrays, vectors, etc.
*
* @author		Gerstmayr Johannes
* @date			2019-05-15 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */

#include <chrono> //for high resolution clock

#include "Utilities/BasicDefinitions.h" //includes stdoutput.h
#include "Utilities/BasicFunctions.h"
#include "Utilities/AdvancedStuff.h"

//! namespace EXUstd = Exudyn standard functions
namespace EXUstd {


	auto referenceTimeChrono = std::chrono::high_resolution_clock::now(); //initialize this variable at program start
	double timerOffset = SetTimerOffset(); //offset to correct effects of GetTimeInSeconds() function

	//get current time since program start in seconds; resolution in nanoseconds; due to offset, this function can produce negative values!
	double GetTimeInSeconds()
	{
		return 1e-9*(double)(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - referenceTimeChrono).count());
	}

	//this calculates an average offset for the function GetTimeInSeconds() and sets it into a global variable 
	double SetTimerOffset()
	{
		double average = 0;
		double nCount = 10000;
		for (Index i = 1; i <= nCount; i++)
		{
			double t1 = GetTimeInSeconds();
			double t2 = GetTimeInSeconds();

			average += t2 - t1;
		}

		timerOffset = average / nCount;
		return timerOffset;
	}

} //namespace EXUstd