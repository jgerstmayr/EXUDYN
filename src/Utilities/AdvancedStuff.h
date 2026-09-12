/** ***********************************************************************************************
* @file			AdvancedStuff.h
* @brief		advanced classes and functions, not needed in every file
* @details		no details yet
*
* @author		Gerstmayr Johannes
* @date			2025-05-08 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef ADVANCEDSTUFF__H
#define ADVANCEDSTUFF__H

//usually include this file after BasicFunctions.h or BasicLinalg.h !

#include <chrono>
#include <ctime>
#include <atomic> //std::atomic
#include <iterator> //for Reverse

//! namespace EXUstd = Exudyn standard functions
namespace EXUstd {

	//! helper to linearly interpolate Real values:
	template<class TVector>
	TVector LinearInterpolateVector(TVector value1, TVector value2, Real xMin, Real xMax, Real x)
	{
		if (xMax == xMin) { return 0.5 * (value1 + value2); } //return average value if no distance ...
		Real fact1 = (xMax - x) / (xMax - xMin);
		Real fact2 = (x - xMin) / (xMax - xMin);
		return fact1 * value1 + fact2 * value2;
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


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//a function to wait until flag is available, then reserve flag
	inline void WaitAndLockSemaphore(std::atomic_flag& flag)
	{
		while (flag.test_and_set(std::memory_order_acquire)) {}; //wait for atomic flag to be available
	}

	//a function to release locked flag
	inline void ReleaseSemaphore(std::atomic_flag& flag)
	{
		flag.clear(std::memory_order_release);
	}

	//!for testing only
	inline void WaitAndLockSemaphoreIgnore(std::atomic_flag& flag)
	{
		; //do nothing, to check if semaphore is needed or causing deadlock
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//time and timer
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

	//get current time since program start in seconds; resolution in nanoseconds; due to offset, this function can produce negative values!
	double GetTimeInSeconds();

	//this calculates an average offset for the function GetTimeInSeconds() and sets it into a global variable 
	double SetTimerOffset();
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	//! compute total size of array of arrays
	template <class T>
	inline Index ArrayOfArraysTotalCount(const T& arrayOfArrays)
	{
		Index cnt = 0;
		for (const auto a : arrayOfArrays)
		{
			cnt += a->NumberOfItems();
		}
		return cnt;
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//a reverse iterator for arrays, etc.:

	template <typename T>
	class ReverseWrapper {
	public:
		explicit ReverseWrapper(T& iterable) : iterable_(iterable) {}

		auto begin() const { return std::make_reverse_iterator(iterable_.end()); }
		auto end() const { return std::make_reverse_iterator(iterable_.begin()); }

	private:
		T& iterable_;
	};

	//usage for reversed iteration: for (auto item: EXUstd::Reverse(vector)) {}
	template <typename T>
	ReverseWrapper<T> Reverse(T& iterable) {
		return ReverseWrapper<T>(iterable);
	}
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//!sorting with values and indices
	template <class IndexArray, class ValueArray>
	void QuickSortIndexed(IndexArray& indices, ValueArray& values, bool reverse = false)
	{
		Index len = values.NumberOfItems();
		if (len == 0) return;

		if (indices.NumberOfItems() == 0) { return; }
		CHECKandTHROW(indices.NumberOfItems() == values.NumberOfItems(), "QuickSortIndexed: numbers of indices and values must be equal");

		Index i, j, inc;
		auto val = values[0];
		auto idx = indices[0];

		inc = 1;
		do {
			inc = inc * 3 + 1;
		} while (inc <= len);

		do {
			inc /= 3;
			for (i = inc; i < len; ++i) {
				val = values[i];
				idx = indices[i];
				j = i;

				// Adjust comparison based on sort direction
				if (!reverse) {
					while (j >= inc && values[j - inc] > val) {
						values[j] = values[j - inc];
						indices[j] = indices[j - inc];
						j -= inc;
					}
				}
				else {
					while (j >= inc && values[j - inc] < val) {
						values[j] = values[j - inc];
						indices[j] = indices[j - inc];
						j -= inc;
					}
				}

				values[j] = val;
				indices[j] = idx;
			}
		} while (inc > 1);
	}

} //namespace EXUstd

#endif //ADVANCEDSTUFF__H