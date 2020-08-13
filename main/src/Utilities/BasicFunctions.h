/** ***********************************************************************************************
* @file			BasicFunctions.h
* @brief		This file contains helper functions, not definitions and not traits;
* @details		Details:
* 				- Helper functions for manipulating arrays, vectors, etc.
*
* @author		Gerstmayr Johannes
* @date			2018-04-30 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
* 				
*
************************************************************************************************ */
#ifndef BASICFUNCTIONS__H
#define BASICFUNCTIONS__H

//for chrono / datetime:
#include <iostream>
#include <chrono>
#include <ctime>

#include <cmath> //(math.h is not available in CLANG)
#include <sstream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h" //defines Real
#include "Utilities/BasicTraits.h"



//! namespace EXUstd = Exudyn standard functions
namespace EXUstd {

    //! checks whether a number is a valid real number: NAN and not INFINITE
    inline int IsValidReal(Real x)
    {
        return std::isfinite(x);
    }

	//! checks whether a number is within a valid range [0,n]; returns true if 0 <= index < n, otherwise false
	inline int IndexIsInRange(Index index, Index rangeBegin, Index rangeEnd)
	{
		return (rangeBegin <= index) && (index < rangeEnd);
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//template functions need to be implemented in header!!!

    //! swaps two functions; could also use std::swap function
    template <class T>
    inline void Swap(T & a, T & b)
    {
        T temp = a;
        a = b;
        b = temp;
    }

    //! compute sign function (gives 0 for a==0, -1 for a < 0 and +1 for a > 0)
    template <class T>
    inline int Sgn(T a)
    {
        if (a > 0) return 1;
        if (a < 0) return -1;
        return 0;
    }

    //! compute square of an object (e.g. Real or int)
    template <class T>
    inline T Square(T a)
    {
        return a * a;
    }

    //! compute cube of an object (e.g. Real or int)
    template <class T>
    inline T Cube(T a)
    {
        return a * a * a;
    }

	//! templated maximum value of two values a and b
	template <class T>
	inline T Maximum(const T& a, const T& b)
	{
		return (a > b) ? a : b;
	}

	//! templated minimum value of two values a and b
	template <class T>
	inline T Minimum(const T& a, const T& b)
	{
		return (a < b) ? a : b;
	}

	//! helper to linearly interpolate Real values:
	inline Real LinearInterpolate(Real value1, Real value2, Real xMin, Real xMax, Real x)
	{
		if (xMax == xMin) { return 0.5 * (value1 + value2); } //return average value if no distance ...
		Real fact1 = (xMax - x) / (xMax - xMin);
		Real fact2 = (x - xMin) / (xMax - xMin);
		return fact1 * value1 + fact2 * value2;
	}

	//! helper to linearly interpolate Real values:
	inline float LinearInterpolate(float value1, float value2, float xMin, float xMax, float x)
	{
		if (xMax == xMin) { return 0.5f * (value1 + value2); } //return average value if no distance ...
		float fact1 = (xMax - x) / (xMax - xMin);
		float fact2 = (x - xMin) / (xMax - xMin);
		return fact1 * value1 + fact2 * value2;
	}


    // @brief Sorts an array x(1..x.Length()) into ascending numerical order by Shell’s method (diminishing increment sort).
    // 'array' is replaced on output by its sorted rearrangement. 
    // needed member functions of array: operator[], Index NumberOfItems()
    // needed capability of items: operator>, copy constructor (operator=)
	//template <class ArrayClass>
	//void QuickSort(ArrayClass& array)
	//{
	//	Index len = array.NumberOfItems();
	//	Index i, j, inc;
	//	if (!len) { return; } //exit if array has zero length (next line would fail otherwise!)

	//	auto item = array[0];

	//	inc = 1; //Determine the starting increment.
	//	do
	//	{
	//		inc *= 3;
	//		inc++;
	//	} while (inc <= len); //<len ?

	//	do
	//	{ //Loop over the partial sorts.
	//		inc /= 3;
	//		for (i = inc; i < len; i++)
	//		{ //Outer loop of straight insertion.
	//			item = array[i];
	//			j = i;
	//			while (array[j - inc] > item)
	//			{ //Inner loop of straight insertion.
	//				array[j] = array[j - inc];
	//				j -= inc;
	//				if (j < inc) break;
	//			}
	//			array[j] = item;
	//		}
	//	} while (inc > 1);
	//}

	//! template function to allow string conversion for objects having stream operator
	template<class T>
	STDstring ToString(const T& streamableObject)
	{
		std::ostringstream sstream;
		sstream << streamableObject;
		return sstream.str();
	}

	template<class T>
	inline STDstring Num2String(const T& number)
	{
		return std::to_string(number);
	}

	//! Get current date and time string
	inline std::string GetDateTimeString() {
		std::time_t clockNow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

		//std::string s(19, '\0'); //allocate sufficient characters (5+3+3+3+3+2=19 needed)
		const size_t n = 20;
		char s[n]; //allocate sufficient characters (5+3+3+3+3+2+1=20 needed)
		std::strftime(&s[0], n, "%Y-%m-%d,%H:%M:%S", localtime(&clockNow));
		//std::strftime(&s[0], s.size(), "%Y-%m-%d,%H:%M:%S", localtime_s(&clockNow)); //newer, but difficult
		return std::string(s);
	}

	//! template function to find an object index within a ResizableArray with objects having a "const STDstring::GetName()" function
	//  if name is not found, EXUstd::InvalidIndex is returned
	template<class T>
	Index GetIndexByName(const T& objectArray, const std::string& name)
	{
		Index index = 0;
		for (auto value : objectArray)
		{
			if (value->GetName() == name)
			{
				return index;
			}
			index++;
		}
		return InvalidIndex;
	}

	//! template function for boolean types to check whether the requested type (typeRequested) is available (typeAvailable)
	template<class T>
	inline bool IsOfType(T typeAvailable, T typeRequested)
	{
		return ((Index)typeAvailable & (Index)typeRequested) == (Index)typeRequested;
	}

	//! template function for boolean types to check whether the requested type (typeRequested) is available (typeAvailable) and if it is not None
	template<class T>
	inline bool IsOfTypeAndNotNone(T typeAvailable, T typeRequested)
	{
		return (((Index)typeAvailable & (Index)typeRequested) == (Index)typeRequested) && (Index)typeRequested != 0;
	}

	//get current time since program start in seconds; resolution in nanoseconds; due to offset, this function can produce negative values!
	double GetTimeInSeconds();

	//this calculates an average offset for the function GetTimeInSeconds() and sets it into a global variable 
	double SetTimerOffset();

} //namespace EXUstd

#endif