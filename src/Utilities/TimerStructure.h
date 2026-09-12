/** ***********************************************************************************************
* @file			TimerStructure.h
* @brief		This file contains a timer class, which can be used to measure function timings at any place in the code
*
* @author		Gerstmayr Johannes
* @date			2020-03-01 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef TIMERSTRUCTURE__H
#define TIMERSTRUCTURE__H

//for chrono / datetime:
#include "Utilities/BasicFunctions.h" //defines Real
#include "Utilities/AdvancedStuff.h"
#include <vector>

//! use macros to completely avoid timings in fast mode
#ifndef EXUDYN_RELEASE
	//timer registration does not work in debug mode (and is not needed!)
	#ifndef _MYDEBUG
		#define USEGLOBALTIMERS
	#endif
#endif


#ifdef __FAST_EXUDYN_LINALG
	#define STARTGLOBALTIMER(_Expression) 
	#define STOPGLOBALTIMER(_Expression) 
	#define STARTGLOBALTIMERmain(_Expression) 
	#define STOPGLOBALTIMERmain(_Expression) 
#else
    //main timers are always kept!
	#define STARTGLOBALTIMERmain(_Expression) (globalTimers.StartTimer(_Expression))
	#define STOPGLOBALTIMERmain(_Expression) (globalTimers.StopTimer(_Expression))
	#ifdef USEGLOBALTIMERS
		#define STARTGLOBALTIMER(_Expression) (globalTimers.StartTimer(_Expression))
		#define STOPGLOBALTIMER(_Expression) (globalTimers.StopTimer(_Expression))
	#else
		#define STARTGLOBALTIMER(_Expression) 
		#define STOPGLOBALTIMER(_Expression) 
	#endif
#endif


class TimerStructure;
extern TimerStructure globalTimers;
extern std::vector<Real>* globalTimersCounters;
extern std::vector<const char*>* globalTimersCounterNames;

//!special timer structure to measure time spent at certain parts of code
//! put a global variable of this class somewhere to allow micro-timing of functions at any place
class TimerStructure
{
private:
	//this class has two coupled lists, which do the work
	//std::vector<Real> counters;
	//std::vector<const char*> counterNames;
	Real offsetSecondsPerCall;
public:
	//! default constructor; DO NOT call Initialize, as TimerStructure may be initialized LATER than first timer is registered!
	TimerStructure() { offsetSecondsPerCall = 0; };

	//! initialize structure with given (measured offset per call, to obtain more accurate measurements)
	//  DO NOT call Initialize, as TimerStructure may be initialized LATER than first timer is registered!
	TimerStructure(Real offsetSecondsPerCallInit) { offsetSecondsPerCall = offsetSecondsPerCallInit; };
	
	//! initialize timers at first call to RegisterTimer, whatever library is doing that (unordered! depends on compiler / Windows/Linux/...)
	void Initialize();

	//! create a new timer; name must be a static name (must exist until end of timer) or dynamically allocated string, may not be deleted
	Index RegisterTimer(const char* name);

	//!get current value of a timer
	Real GetTiming(Index counterIndex) const { return (*globalTimersCounters)[counterIndex]; }

	//!get current value of a timer
	const char* GetTimerName(Index counterIndex) const { return (*globalTimersCounterNames)[counterIndex]; }

	//!get current value of a timer
	Index NumberOfTimers() const { return (Index)globalTimersCounters->size(); }

	//!reset timers( e.g. before starting simulation):
	void Reset()
	{
		for (auto& item : (*globalTimersCounters)) { item = 0; }
	}

	//! set counter to specific value
	void SetCounter(Index counterIndex, Real value) { (*globalTimersCounters)[counterIndex] = value; }

	//! get counter value
	Real GetCounter(Index counterIndex) { return (*globalTimersCounters)[counterIndex]; }

	//! start measurement
	void StartTimer(Index counterIndex)
	{
		(*globalTimersCounters)[counterIndex] -= EXUstd::GetTimeInSeconds();
	}

	//! stop measurement
	void StopTimer(Index counterIndex)
	{
		(*globalTimersCounters)[counterIndex] += EXUstd::GetTimeInSeconds() - offsetSecondsPerCall;
	}

	//! print current timers into string; if total time is provided, it may also print times relative to total time
	STDstring ToString(bool printRelative=false, Real totalTime=0) const 
	{
		if (!globalTimersCounters->size()) { return ""; }

		std::ostringstream ostr;
		for (Index i = 0; i < (Index)globalTimersCounters->size(); i++)
		{
			if ((*globalTimersCounters)[i] != 0.) //exclude timers that are exactly zero:
			{
				ostr.precision(5); //reduced precision for nicer output...
				ostr << "  " << (*globalTimersCounterNames)[i] << " = " << (*globalTimersCounters)[i] << "s";
				if (totalTime != 0 && printRelative) {
					ostr.precision(3); //reduced precision for nicer output...
					ostr << " (" << (*globalTimersCounters)[i] / totalTime * 100. << "%)";
				}
				ostr << "\n";
			}
		}
		return ostr.str();
	}
};


class TimerStructureRegistrator
{
public:
	//! register a timer by creating instance with this constructor:
	TimerStructureRegistrator(const char* timerName, Index& timerNumber, TimerStructure& globalTimerStructure, bool addAlways=false)
	{
	#ifdef USEGLOBALTIMERS
		timerNumber = globalTimerStructure.RegisterTimer(timerName);
	#else
		if (addAlways)
		{
			timerNumber = globalTimerStructure.RegisterTimer(timerName);
		}
	#endif
	}
};

#endif
