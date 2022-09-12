/** ***********************************************************************************************
* @file			Threading.h
* @brief		This file mainly contains a library for small-scale parallelization ("micro-threading")
*               Multithreading takes effect for >1000 clock cycles (or even less);
*               thanks to Joachim Schöberl!!!
* @details		Details:
*               This file adapts / duplicates parts of NGsolve: https://github.com/NGSolve/ngsolve ; see also https://ngsolve.org/ 
*
* @author		Gerstmayr Johannes
* @date			2022-01-13 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */

#define MICROTHREADING__H
#ifdef MICROTHREADING__H

//partially copied from NGsolve project:
#include <exception>
#include <atomic>
#include <mutex>
#include "Utilities/BasicDefinitions.h"
#include "Utilities/ResizableArray.h"

namespace MicroThreading {
	typedef Index SizeType;
	typedef SizeType TotalCosts;
	extern class TaskManager * task_manager;

	class Exception : public std::exception
	{
		/// a verbal description of the exception
		std::string m_what;
	public:
		// Exception ();
		/// string s describes the exception
		Exception(const std::string & s);
		/// string s describes the exception
		Exception(const char * s);
		///
		virtual ~Exception();

		/// append string to description
		Exception & Append(const std::string & s);
		/// append string to description
		Exception & Append(const char * s);

		/// verbal description of exception
		const std::string & What() const { return m_what; }

		/// implement virtual function of std::exception
		virtual const char* what() const noexcept override { return m_what.c_str(); }
	};

	inline Exception :: ~Exception()
	{
		;
	}

	inline Exception & Exception::Append(const std::string & s)
	{
		m_what += s;
		return *this;
	}

	inline Exception & Exception::Append(const char * s)
	{
		m_what += s;
		return *this;
	}

	template <typename T>
	inline Exception & operator<< (Exception & ex, T data)
	{
		ex.Append(data);
		return ex;
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	template <typename TIndex>
	class ArrayRangeIterator
	{
		TIndex ind;
	public:
		inline ArrayRangeIterator(TIndex ai) : ind(ai) { ; }
		inline ArrayRangeIterator operator++ (Index) { return ind++; }
		inline ArrayRangeIterator operator++ () { return ++ind; }
		inline TIndex operator*() const { return ind; }
		inline TIndex Index() { return ind; }
		inline operator TIndex () const { return ind; }
		inline bool operator != (ArrayRangeIterator d2) { return ind != d2.ind; }
		inline bool operator == (ArrayRangeIterator d2) { return ind == d2.ind; }
	};

	//range with iterator for elements in range; will be used to iterate loop in task
	template <typename TIndex>
	class RangeBase
	{
		TIndex first, next;
	public:
		inline RangeBase() { ; }
		inline RangeBase(TIndex n) : first(0), next(n) { ; }
		inline RangeBase(TIndex f, TIndex n) : first(f), next(n) { ; }
		template <typename T2>
		inline RangeBase(RangeBase<T2> r2) : first(r2.First()), next(r2.Next()) { ; }
		inline TIndex First() const { return first; }
		inline TIndex Next() const { return next; }
		inline TIndex Size() const { return next - first; }
		inline TIndex operator[] (TIndex i) const { return first + i; }
		inline bool Contains(TIndex i) const { return ((i >= first) && (i < next)); }

		ArrayRangeIterator<TIndex> begin() const { return first; }
		ArrayRangeIterator<TIndex> end() const { return next; }

		RangeBase Split(SizeType nr, Index tot) const
		{
			TIndex diff = next - first;
			return RangeBase(first + nr * diff / tot,
				first + (nr + 1) * diff / tot);
		}
		// inline operator IntRange () const { return IntRange(first,next); }
	};
	typedef RangeBase<Index> RangeIndex;

	class TaskInfo
	{
	public:
		Index task_nr;
		Index ntasks;

		Index thread_nr;
		Index nthreads;

	};


	//template <typename TFUNC>
	//inline void ParallelFor(NGSsizeType n, TFUNC f, Index tasks_per_thread = 1, NGSsizeType costs = 1000)
	//{
	//	for (Index i = 0; i < n; i++)
	//	{
	//		f(i);
	//	}
	//}
	//inline Index  EnterTaskManager() { return 1; };
	//inline void ExitTaskManager(Index num_threads) {};

	class TaskManager
	{
		//static const Index maxNumberOfThreads = 128; //limit by arrays
		//static std::atomic<bool> isRunning; //this flag is used to stop all additional threads by isRunning=false
		static bool isRunning; //atomic not needed; slightly faster for permanent checks of this variable; this flag is used to stop all additional threads by isRunning=false
		std::atomic<Index> active_workers; //counts the additional threads running (excl. main thread!); used to wait until all threads started/stopped
		//atomic<Index> workers_on_node[4];   // max nodes

		std::atomic<bool> newJob; //this flag is used to initiate new job at all threads
		std::atomic<bool> restartLoops; //this flag is used to restart waiting loop in all threads
		//bool newJob; //this flag is used to initiate new job at all threads
		//bool restartLoops; //this flag is used to restart waiting loop in all threads

		static Index num_threads;
		static Index max_threads;
		static thread_local Index thread_id; //this is the thread ID locally accessible in thread ...

		//this is written on CreateJob:
		static const std::function<void(TaskInfo&)> * func;

		//std::atomic<Index> ntasks; //number of tasks / equal to number of threads
		//std::atomic<Index> completed_tasks;
		Exception * ex;

		ResizableArray<std::atomic<int>*> sync; //is initialized when started up

	public:

		TaskManager()
		{
			//done during initialization:
			//num_threads = 1; //already set before
			//max_threads = std::thread::hardware_concurrency(); //this is the number provided by hardware
			isRunning = true;
			active_workers = 0;
			newJob = false;
			restartLoops = false;

		}
		~TaskManager() 
		{ 
			if (isRunning) { StopWorkers(); }
		};


		void StartWorkers();
		void StopWorkers();

		void SuspendWorkers(Index asleep_usecs = 1000)
		{
			//sleep_usecs = asleep_usecs;
			//sleep = true;
		}
		void ResumeWorkers() {} // { sleep = false; }

		static void SetNumThreads(Index numThreadsInit) 
		{ 
			CHECKandTHROW(!isRunning, "SetNumThreads: may only be called when threads are not running");
			num_threads = numThreadsInit;
		}
		static Index GetNumThreads() { return num_threads; }

		static Index GetThreadId() { return task_manager ? task_manager->thread_id : 0; }

		void CreateJob(const std::function<void(TaskInfo&)> & afunc,
			Index numberOfJobTasks = task_manager->GetNumThreads());

		void Loop(Index threadID);

	};

	Index EnterTaskManager();
	//! if we exit taskmanager with value != 0, work is stopped and task manager removed
	void ExitTaskManager(Index num_threads);

	//! ParallelFor with range
	template <typename TR, typename TFUNC>
	inline void ParallelFor(RangeBase<TR> r, TFUNC f,
		int tasksTimesThreads = task_manager ? task_manager->GetNumThreads() : 0,
		TotalCosts costs = 1000)
	{
		//CHECKandTHROW(tasksTimesThreads == task_manager->GetNumThreads(), "in TinyThread, tasksTimesThreads must be equal to number of threads");
		tasksTimesThreads = task_manager->GetNumThreads(); //overwrite for compatibility with ngs lib
		if (task_manager && costs >= 1000)
		{
			task_manager->CreateJob
			([r, f](TaskInfo & ti)
			{
				auto myrange = r.Split(ti.task_nr, ti.ntasks);
				for (auto i : myrange) f(i);
			},
				tasksTimesThreads);
		}
		else
		{
			for (auto i : r) { f(i); }
		}
	}

	//! ParallelFor with simple index, ranges from 0 ... n-1
	template <typename ...Args>
	inline void ParallelFor(size_t n, Args...args)
	{
		ParallelFor(RangeIndex(n), args...);
	}


};
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif //MICROTHREADING__H
