/** ***********************************************************************************************
* @file			Threading.h
* @brief		This file mainly contains a library for small-scale parallelization ("micro-threading")
*               Multithreading takes effect for tasks>1000 clock cycles (or even less);
*               thanks to Joachim Schöberl!!!
* @details		Details:
*               This file adapts / duplicates parts of NGsolve: https://github.com/NGSolve/ngsolve ; see also https://ngsolve.org/ 
*
* @author		Gerstmayr Johannes
* @date			2022-01-13 (created); 2025-06-16 (extended)
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
#include <condition_variable>
#include "Utilities/BasicDefinitions.h"
#include "Utilities/ResizableArray.h"


namespace ExuThreading {

	typedef Index SizeType;
	typedef SizeType TotalCosts;
	extern class TaskManager* task_manager;

	class Exception : public std::exception
	{
		/// a verbal description of the exception
		std::string m_what;
	public:
		// Exception ();
		/// string s describes the exception
		Exception(const std::string& s);
		/// string s describes the exception
		Exception(const char* s);
		///
		virtual ~Exception();

		/// append string to description
		Exception& Append(const std::string& s);
		/// append string to description
		Exception& Append(const char* s);

		/// verbal description of exception
		const std::string& What() const { return m_what; }

		/// implement virtual function of std::exception
		virtual const char* what() const noexcept override { return m_what.c_str(); }
	};

	inline Exception :: ~Exception()
	{
		;
	}

	inline Exception& Exception::Append(const std::string& s)
	{
		m_what += s;
		return *this;
	}

	inline Exception& Exception::Append(const char* s)
	{
		m_what += s;
		return *this;
	}

	template <typename T>
	inline Exception& operator<< (Exception& ex, T data)
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

	//padding to have sync variables in different cache lines
	struct PaddedAtomicInt {
		//alignas(64) std::atomic_int value; // Align to 64 bytes
		//char padding[64 - sizeof(std::atomic_int)]; // Add padding to fill the cache line
		std::atomic_int value;
	};

	//! how a computation got its threads from TaskManager::AcquireWorkers (#3437)
	enum WorkersAcquired { workersNotAcquired = -2, workersBusy = -1, workersSerial = 0, workersPersistent = 1, workersOwn = 2 };

	class TaskManager
	{
		static std::mutex poolMutex;				//!< guards task_manager, parkedManager and persistentNumberOfThreads (#3437)
		static TaskManager* parkedManager;			//!< the persistent workers while no computation uses them; they sleep (#3437)
		static Index persistentNumberOfThreads;		//!< the threads of the persistent workers, main thread included; 0: none (#3437)
		static Index ticketCounter;					//!< counts AcquireWorkers (#3437)
		static Index attachedTicket;				//!< the ticket of the computation that uses task_manager (#3437)
		static void StartPersistentLocked();
		static void ParkLocked();
		static void StopParkedLocked();

		static std::atomic<bool> isRunning;
		static std::atomic<bool> jobIsRunning; //!< true while CreateJob runs a job in the threads (#3229)
		static std::atomic<bool> workersSuspended; //!< true: the workers wait on suspendCondition instead of spinning (#3428)
		static std::mutex suspendMutex;
		static std::condition_variable suspendCondition;
		std::atomic<Index> active_workers;
		std::atomic<Index> numberOfTasks;  // number of tasks in current job

		static Index num_threads;
		static Index max_threads;
		static thread_local Index thread_id;

		static const std::function<void(TaskInfo&)>* func;

		Exception* ex;
		//ResizableArray<std::atomic_int*> sync;
		ResizableArray<PaddedAtomicInt*> sync;

		std::atomic<Index> next_task;  // Shared task index for dynamic load balancing
		static bool loadBalancing;    // Load balancing flag

		std::atomic<Index> completedThreads; // total tasks completed by all threads (except main thread)

	public:

		TaskManager()
		{
			isRunning.store(true, std::memory_order_relaxed);
			active_workers = 0;
			//newJob = false;
			//restartLoops = false;
			next_task.store(0);  // Initialize shared task index
		}

		~TaskManager()
		{
			if (isRunning.load(std::memory_order_relaxed) )
			{
				StopWorkers();
			}
		}

		void StartWorkers();
		void StopWorkers();
		//! the workers stop spinning and wait on a condition variable until ResumeWorkers or the next job, e.g. while a
		//! serial or externally threaded factorization runs (#3428); the first job after it pays the wake-up
		static void SuspendWorkers() { workersSuspended.store(true, std::memory_order_seq_cst); }
		//! wake up the workers suspended by SuspendWorkers (#3428)
		static void ResumeWorkers();
		static bool WorkersSuspended() { return workersSuspended.load(std::memory_order_relaxed); }

		static bool IsRunning() { return isRunning.load(std::memory_order_relaxed); }

		//! start numberOfThreads > 1 persistent workers, which sleep while no computation uses them; 0 or 1 stops them;
		//! returns false and changes nothing while a computation uses the threads (#3437)
		static bool SetPersistentWorkers(Index numberOfThreads);
		//! the threads of the persistent workers, main thread included; 0: none (#3437)
		static Index PersistentWorkers() { return persistentNumberOfThreads; }
		//! take the threads for a parallel computation with numberOfThreads: the persistent workers if their number agrees,
		//! otherwise a task manager of its own (the persistent workers stop until ReleaseWorkers, and message says so);
		//! if threads are in use: workersBusy, or with takeOver they are stopped (message says so); the result goes to
		//! ReleaseWorkers (#3437)
		static WorkersAcquired AcquireWorkers(Index numberOfThreads, bool takeOver, STDstring& message, Index& ticket);
		//! give back the threads of AcquireWorkers with its ticket: persistent workers sleep again, a task manager of its
		//! own stops; threads taken over by another computation in the meantime are left alone (#2237, #3437)
		static void ReleaseWorkers(WorkersAcquired acquired, Index ticket);
		//! true inside a parallel loop, in every thread taking part (#3229)
		static bool JobIsRunning() { return jobIsRunning.load(std::memory_order_relaxed); }
		static void SetNumThreads(Index numThreadsInit)
		{
			CHECKandTHROW(!isRunning.load(std::memory_order_relaxed), "SetNumThreads: may only be called if threads are not running");
			num_threads = numThreadsInit;
		}
		static Index GetNumThreads() { return num_threads; }

		static void SetLoadBalancing(bool flag)
		{
			CHECKandTHROW(!isRunning.load(std::memory_order_relaxed), "SetLoadBalancing: may only be called if threads are not running");
			loadBalancing = flag;
		}
		static bool GetLoadBalancing() { return loadBalancing; }


		inline static Index GetThreadId() { return task_manager ? task_manager->thread_id : 0; }

		void CreateJob(const std::function<void(TaskInfo&)>& afunc, Index numberOfJobTasks = task_manager->GetNumThreads());

		inline void Loop(Index threadID);


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
		//DELETE: tasksTimesThreads = task_manager->GetNumThreads(); //overwrite for compatibility with ngs lib
		if (task_manager && costs >= 1000)
		{
			task_manager->CreateJob
			([r, f](TaskInfo& ti)
				{
					auto myrange = r.Split(ti.task_nr, ti.ntasks);
					for (auto i : myrange) { f(i); }
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
		ParallelFor(RangeIndex((Index)n), args...);
	}


};


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif //MICROTHREADING__H
