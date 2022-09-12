/** ***********************************************************************************************
* @file			Threading.cpp
* @brief		Implementation of micro-multithreading tools, taken and adapted from NGsolve project; thanks to Joachim Schöberl!!!
* @details		Details:
*               This file adapts / duplicates parts of NGsolve: https://github.com/NGSolve/ngsolve ; see also https://ngsolve.org/ 
*
* @author		Gerstmayr Johannes
* @date			2022-07-11 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */

#include "Utilities/BasicDefinitions.h"
#include "Linalg/Use_avx.h"
#include "Linalg/Vector.h" 
#include "Linalg/SlimVector.h" 
#include "Utilities/Parallel.h"

//#define MEMORY_ORDER_TINYTHREAD std::memory_order_acq_rel

#define MEMORY_ORDER_TINYTHREAD std::memory_order_seq_cst
//#define MEMORY_ORDER_NEWJOB std::memory_order_relaxed 
//#define MEMORY_ORDER_NEWJOB std::memory_order_seq_cst
//#define MEMORY_ORDER_NEWJOB std::memory_order_acq_rel //only works if applied always in correct order

#define MEMORY_ORDER_SYNCLOAD std::memory_order_relaxed //on intel same as memory_order_seq_cst?
#define MEMORY_ORDER_SYNCSTORE std::memory_order_seq_cst

//#define USETHREADFENCE
//#define PRINTDEBUGINFORMATION

//#define USE_BINARY_OR_SYNC

//static objects
namespace MicroThreading
{
	TaskManager * task_manager = nullptr;
	int TaskManager::num_threads = 1;
	int TaskManager::max_threads = std::thread::hardware_concurrency();
	//std::atomic<bool> TaskManager::isRunning = false;
	bool TaskManager::isRunning = false;

	//bool TaskManager::isRunning = false;
	thread_local int TaskManager::thread_id;

	const std::function<void(TaskInfo&)> * TaskManager::func;

	static std::mutex copyex_mutex;
	static std::mutex printexception_mutex;

};


//! a very slim and efficient multithreading approach for small loops
namespace MicroThreading
{
	inline Exception::Exception(const std::string & s)
		: m_what(s)
	{
		{
			std::lock_guard<std::mutex> guard(printexception_mutex);
		}
	}

	inline Exception::Exception(const char * s)
		: m_what(s)
	{
		{
			std::lock_guard<std::mutex> guard(printexception_mutex);
		}
	}


	void TaskManager::CreateJob(const std::function<void(TaskInfo&)> & afunc,
		Index numberOfJobTasks) //numberOfJobTasks == numberOfThreads
	{
#ifdef PRINTDEBUGINFORMATION
		std::cout << "main thread: create job\n";
#endif
		func = &afunc;

		ex = nullptr;

		TaskInfo ti;
		ti.nthreads = GetNumThreads();
		ti.ntasks = ti.nthreads; //here both are same!
		ti.thread_nr = 0;
		ti.task_nr = ti.thread_nr; //here both are same!

#ifdef PRINTDEBUGINFORMATION
		std::cout << "main thread: send starting sync\n";
#endif
#ifdef USE_BINARY_OR_SYNC
		sync[0]->store(0, MEMORY_ORDER_SYNCSTORE);//sync==0 means that job is ready to be computed
#else
		for (Index i = 1; i < sync.NumberOfItems(); i++)
		{
			sync[i]->store(0, MEMORY_ORDER_SYNCSTORE);//sync==0 means that job is ready to be computed
		}
#endif
#ifdef PRINTDEBUGINFORMATION
		std::cout << "main thread: compute job\n";
#endif
		try
		{
			//just complete own task (expands into range!)
			(*func)(ti);

		}
		catch (Exception e)
		{
			{
				std::lock_guard<std::mutex> guard(copyex_mutex);
				if (ex) { delete ex; }
				ex = new Exception(e);
			}
		}

		//throw exception after work
		if (ex)
			throw Exception(*ex);

#ifdef PRINTDEBUGINFORMATION
		std::cout << " main thread: wait for signal finished\n";
#endif
		//! wait until other threads are ready
		//==> use loop from 1 .. nThreads-1
#ifdef USE_BINARY_OR_SYNC
		while (sync[0]->load(MEMORY_ORDER_SYNCLOAD) != (1 << ti.nthreads) - 2) { ; }//wait until all sync set!
#else
		for (Index i = 1; i < sync.NumberOfItems(); i++)
		{
			while (!sync[i]->load(MEMORY_ORDER_SYNCLOAD)) { ; }//wait until sync is 1
#ifdef PRINTDEBUGINFORMATION
			std::cout << " main thread: task " << i << " signal finished\n";
#endif
		}
#endif
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	Index EnterTaskManager()
	{
		if (task_manager)
		{
			// task manager already running
			return 0;
		}

		task_manager = new TaskManager();

#ifndef WIN32
		// master has maximal priority !
		int policy;
		struct sched_param param;
		pthread_getschedparam(pthread_self(), &policy, &param);
		param.sched_priority = sched_get_priority_max(policy);
		pthread_setschedparam(pthread_self(), policy, &param);
#endif // WIN32

		task_manager->StartWorkers();

		ParallelFor(Range(100), [&](Index i) {; });    // startup
		return task_manager->GetNumThreads();
	}


	//! if we exit taskmanager with value != 0, work is stopped and task manager removed
	void ExitTaskManager(Index num_threads)
	{
		if (task_manager && num_threads > 0)
		{
			task_manager->StopWorkers();
			delete task_manager;
			task_manager = nullptr;
		}
	}

	inline void TaskManager::StartWorkers()
	{
		isRunning = true;
		restartLoops = false; //for safety, should be already initialized
		newJob = false; //for safety, should be already initialized

#ifdef PRINTDEBUGINFORMATION
		std::cout << "start " << num_threads << " workers ****\n";
#endif

		sync.SetNumberOfItems(num_threads);
		sync[0] = new std::atomic<Index>(0); //sync[0] in fact not needed!

		//! start (num_threads-1) additional threads (+ main thread)
		for (Index i = 1; i < num_threads; i++)
		{
			std::thread([this, i]() { this->Loop(i); }).detach();
		}

		while (active_workers < num_threads - 1)
			;
	}

	inline void TaskManager::StopWorkers()
	{
		isRunning = false;

		//wait for other threads to stop
		while (active_workers) { ; }

		if (task_manager && num_threads > 0 && sync.NumberOfItems() > 0) //otherwise, do not delete sync!
		{
			delete sync[0]; //free memory for sync
			sync.SetNumberOfItems(0);
		}
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	inline void TaskManager::Loop(Index threadID)
	{
		thread_id = threadID;
#ifdef PRINTDEBUGINFORMATION
		std::cout << "  start Loop in thread" << thread_id << "\n";
#endif

#ifdef USE_BINARY_OR_SYNC
		*sync[0] |= 1 << thread_id;
#else
		sync[thread_id] = new std::atomic<Index>(1);
#endif

		TaskInfo ti;
		ti.nthreads = GetNumThreads();
		ti.ntasks = ti.nthreads; //here both are same!
		ti.thread_nr = thread_id;
		ti.task_nr = ti.thread_nr; //here both are same!


		active_workers++; //register that thread is running; after sync=0 !
		bool stop = !isRunning;// .load(MEMORY_ORDER_SYNCLOAD);
		while (!stop)
		{
			//wait for new job; HOT WAIT!:
			//wait until main thread switches sync to 0, then job is available
#ifdef USE_BINARY_OR_SYNC
			while ((*sync[0] & (1 << thread_id)) != 0 && !stop)
#else
			while (sync[thread_id]->load(MEMORY_ORDER_SYNCLOAD) && !stop)
#endif
			{
				stop = !isRunning;// .load(std::memory_order_relaxed); //this is not urgent, as it is only performed at end of many computations
			}

			if (stop)
			{ 
				break; 
			}

#ifdef PRINTDEBUGINFORMATION
			std::cout << "  thread" << thread_id << ": compute task\n";
#endif
			//now complete task
			try
			{
				//just complete own task (expands into range!)
				(*func)(ti);

			}
			catch (Exception e)
			{
				{
					std::lock_guard<std::mutex> guard(copyex_mutex);
					if (ex) { delete ex; }
					ex = new Exception(e);
				}
			}

#ifdef PRINTDEBUGINFORMATION
			std::cout << "  thread" << thread_id << ": send signal finished\n";
#endif
#ifdef USE_BINARY_OR_SYNC
			*sync[0] |= 1 << thread_id;//->store(0, MEMORY_ORDER_SYNCSTORE);//sync==0 means that job is ready to be computed
#else
			sync[thread_id]->store(1, MEMORY_ORDER_SYNCSTORE); // , memory_order_release); //main thread receives message that job is done!
#endif
		}


		//finish loops!
#if !defined(USE_BINARY_OR_SYNC)
		delete sync[thread_id];
#endif
		active_workers--;
	}
};

//with MicroThreading (OLD):
//AVXsize = 4
//AVXRealShift = 2
//vector operations needed = 0.303403, GFlops = 2.6366, result = 2.09011e+10
//factor threads = 1, vector size = 201
//parallel vector operations needed = 0.50916, GFlops = 0.394768, error = 0.00223267
//factor threads = 1, vector size = 401
//parallel vector operations needed = 0.260245, GFlops = 0.770427, error = 0.00157917
//factor threads = 1, vector size = 801
//parallel vector operations needed = 0.140613, GFlops = 1.42412, error = 0.00111658
//factor threads = 1, vector size = 1601
//parallel vector operations needed = 0.0891604, GFlops = 2.24455, error = 0.000789559
//factor threads = 1, vector size = 3201
//parallel vector operations needed = 0.0509337, GFlops = 3.9279, error = 0.000558309
//factor threads = 1, vector size = 6401
//parallel vector operations needed = 0.0485383, GFlops = 4.1211, error = 0.000394734
//factor threads = 1, vector size = 12801
//parallel vector operations needed = 0.0344423, GFlops = 5.80727, error = 0.000279094
//factor threads = 1, vector size = 25601
//parallel vector operations needed = 0.0413064, GFlops = 4.84174, error = 0.000197333
//factor threads = 1, vector size = 51201
//parallel vector operations needed = 0.0268043, GFlops = 7.46116, error = 0.000139527
//factor threads = 1, vector size = 102401
//parallel vector operations needed = 0.0427369, GFlops = 4.67954, error = 9.86343e-05
//factor threads = 1, vector size = 204801
//parallel vector operations needed = 0.0379905, GFlops = 5.26147, error = 6.94446e-05
//factor threads = 1, vector size = 409601
//parallel vector operations needed = 0.0494997, GFlops = 4.03811, error = 4.89937e-05
//factor threads = 1, vector size = 819201
//parallel vector operations needed = 0.149659, GFlops = 1.33561, error = 3.50732e-05
//factor threads = 1, vector size = 1638401
//parallel vector operations needed = 0.199998, GFlops = 0.999434, error = 2.48448e-05

//with MicroThreading (OLD):
//AVXsize = 4
//AVXRealShift = 2
//vector operations needed = 0.29037, GFlops = 2.75494, result = 2.09011e+10
//factor threads = 1, vector size = 801
//parallel vector operations needed = 0.600197, GFlops = 1.33456, error = 0.0178655
//factor threads = 1, vector size = 1601
//parallel vector operations needed = 0.338942, GFlops = 2.36176, error = 0.0126328
//factor threads = 1, vector size = 3201
//parallel vector operations needed = 0.21171, GFlops = 3.77994, error = 0.0089327
//factor threads = 1, vector size = 6401
//parallel vector operations needed = 0.163372, GFlops = 4.89757, error = 0.0063165
//factor threads = 1, vector size = 12801
//parallel vector operations needed = 0.183468, GFlops = 4.36077, error = 0.00446646
//factor threads = 1, vector size = 25601
//parallel vector operations needed = 0.15794, GFlops = 5.06541, error = 0.00315788
//factor threads = 1, vector size = 51201
//parallel vector operations needed = 0.140163, GFlops = 5.70776, error = 0.00223276
//factor threads = 1, vector size = 102401
//parallel vector operations needed = 0.161744, GFlops = 4.94583, error = 0.00157866
//factor threads = 1, vector size = 204801
//parallel vector operations needed = 0.154861, GFlops = 5.16562, error = 0.00111622
//factor threads = 1, vector size = 409601
//parallel vector operations needed = 0.177923, GFlops = 4.49606, error = 0.000789076
//factor threads = 1, vector size = 819201
//parallel vector operations needed = 0.557107, GFlops = 1.43516, error = 0.000555557
//factor threads = 1, vector size = 1638401
//parallel vector operations needed = 0.75907, GFlops = 1.05331, error = 0.00039195





//AVX2 with ngstd::TaskManager:
//AVXsize = 4
//AVXRealShift = 2
//vector operations needed = 0.296289, GFlops = 2.69991, result = 2.09011e+10
//factor threads = 1, vector size = 201
//parallel vector operations needed = 0.737914, GFlops = 0.27239, error = 0.00223267
//factor threads = 1, vector size = 401
//parallel vector operations needed = 0.375357, GFlops = 0.534158, error = 0.00157917
//factor threads = 1, vector size = 801
//parallel vector operations needed = 0.237826, GFlops = 0.842002, error = 0.00111658
//factor threads = 1, vector size = 1601
//parallel vector operations needed = 0.16496, GFlops = 1.21318, error = 0.000789559
//factor threads = 1, vector size = 3201
//parallel vector operations needed = 0.117977, GFlops = 1.69577, error = 0.000558309
//factor threads = 1, vector size = 6401
//parallel vector operations needed = 0.0837133, GFlops = 2.38948, error = 0.000394734
//factor threads = 1, vector size = 12801
//parallel vector operations needed = 0.062855, GFlops = 3.18218, error = 0.000279094
//factor threads = 1, vector size = 25601
//parallel vector operations needed = 0.0548018, GFlops = 3.64942, error = 0.000197333
//factor threads = 1, vector size = 51201
//parallel vector operations needed = 0.0455601, GFlops = 4.38961, error = 0.000139527
//factor threads = 1, vector size = 102401
//parallel vector operations needed = 0.0458407, GFlops = 4.3627, error = 9.86343e-05
//factor threads = 1, vector size = 204801
//parallel vector operations needed = 0.0444333, GFlops = 4.49856, error = 6.94446e-05
//factor threads = 1, vector size = 409601
//parallel vector operations needed = 0.0506009, GFlops = 3.95023, error = 4.89937e-05
//factor threads = 1, vector size = 819201
//parallel vector operations needed = 0.14133, GFlops = 1.41432, error = 3.50732e-05
//factor threads = 1, vector size = 1638401
//parallel vector operations needed = 0.194314, GFlops = 1.02867, error = 2.48448e-05








//non-AVX with ngstd::TaskManager:
//vector operations needed = 1.82035, GFlops = 1.09979, result = 3.65422e+09
//	factor threads = 1, vector size = 1001
//	parallel vector operations needed = 3.59688, GFlops = 0.556594, result = 3.65422e+09
//	factor threads = 1, vector size = 2001
//	parallel vector operations needed = 2.66248, GFlops = 0.751556, result = 5.16591e+09
//	factor threads = 1, vector size = 4001
//	parallel vector operations needed = 1.73088, GFlops = 1.15577, result = 7.30434e+09
//	factor threads = 1, vector size = 8001
//	parallel vector operations needed = 1.22093, GFlops = 1.63829, result = 1.03289e+10
//	factor threads = 1, vector size = 16001
//	parallel vector operations needed = 1.01305, GFlops = 1.97436, result = 1.46066e+10
//	factor threads = 1, vector size = 32001
//	parallel vector operations needed = 0.906638, GFlops = 2.20602, result = 2.06564e+10
//	factor threads = 1, vector size = 64001
//	parallel vector operations needed = 0.848137, GFlops = 2.35815, result = 2.92122e+10
//	factor threads = 1, vector size = 128001
//	parallel vector operations needed = 1.08405, GFlops = 1.84494, result = 4.13121e+10
//	factor threads = 1, vector size = 256001
//	parallel vector operations needed = 0.894263, GFlops = 2.23634, result = 5.84202e+10
//	factor threads = 1, vector size = 512001
//	parallel vector operations needed = 1.0294, GFlops = 1.94275, result = 8.26185e+10
//	factor threads = 1, vector size = 1024001
//	parallel vector operations needed = 2.11977, GFlops = 0.943439, result = 1.1684e+11
//	factor threads = 1, vector size = 2048001
//	parallel vector operations needed = 2.21317, GFlops = 0.903159, result = 1.65152e+11
//	factor threads = 1, vector size = 4096001
//	parallel vector operations needed = 2.18703, GFlops = 0.913958, result = 2.3356e+11
//	factor threads = 1, vector size = 8192001
//	parallel vector operations needed = 2.24689, GFlops = 0.889607, result = 3.30304e+11
//	factor threads = 1, vector size = 16384001
//	parallel vector operations needed = 2.21898, GFlops = 0.900795, result = 4.67121e+11

	//pure AVX results, when TaskManager is not running:
	//vector operations needed = 1.51717, GFlops = 1.31837, result = 1.15479e+10
	//	factor threads = 1, vector size = 10001
	//	parallel vector operations needed = 0.469449, GFlops = 4.26074, result = 1.15479e+10
	//	factor threads = 1, vector size = 20001
	//	parallel vector operations needed = 0.605504, GFlops = 3.3032, result = 1.63305e+10
	//	factor threads = 1, vector size = 40001
	//	parallel vector operations needed = 0.647455, GFlops = 3.0891, result = 2.30944e+10
	//	factor threads = 1, vector size = 80001
	//	parallel vector operations needed = 0.6713, GFlops = 2.97933, result = 3.26602e+10
	//	factor threads = 1, vector size = 160001
	//	parallel vector operations needed = 0.663785, GFlops = 3.01304, result = 4.61882e+10
	//	factor threads = 1, vector size = 320001
	//	parallel vector operations needed = 0.704739, GFlops = 2.83794, result = 6.53199e+10
	//	factor threads = 1, vector size = 640001
	//	parallel vector operations needed = 1.50589, GFlops = 1.32812, result = 9.23762e+10
	//	factor threads = 1, vector size = 1280001
	//	parallel vector operations needed = 1.99124, GFlops = 1.00408, result = 1.30598e+11
	//	factor threads = 1, vector size = 2560001
	//	parallel vector operations needed = 2.2307, GFlops = 0.896292, result = 1.84693e+11
	//	factor threads = 1, vector size = 5120001
	//	parallel vector operations needed = 2.34449, GFlops = 0.851701, result = 2.60861e+11
	//	factor threads = 1, vector size = 10240001
	//	parallel vector operations needed = 2.45914, GFlops = 0.811991, result = 3.68913e+11
	//	factor threads = 1, vector size = 20480001
	//	parallel vector operations needed = 2.38116, GFlops = 0.834284, result = 5.19046e+11
	//	Drücken Sie eine beliebige Taste . . .

	//pure AVX results, when TaskManager is running, but ParallelFor not used:
		//vector operations needed = 1.47524, GFlops = 1.35584, result = 1.15479e+10
		//	factor threads = 1, vector size = 10001
		//	parallel vector operations needed = 0.568947, GFlops = 3.51562, result = 1.15479e+10
		//	factor threads = 1, vector size = 20001
		//	parallel vector operations needed = 0.933088, GFlops = 2.14353, result = 1.63305e+10
		//	factor threads = 1, vector size = 40001
		//	parallel vector operations needed = 0.926404, GFlops = 2.15894, result = 2.30944e+10
		//	factor threads = 1, vector size = 80001
		//	parallel vector operations needed = 0.963877, GFlops = 2.07498, result = 3.26602e+10
		//	factor threads = 1, vector size = 160001
		//	parallel vector operations needed = 0.798813, GFlops = 2.50373, result = 4.61882e+10
		//	factor threads = 1, vector size = 320001
		//	parallel vector operations needed = 1.01789, GFlops = 1.96485, result = 6.53199e+10
		//	factor threads = 1, vector size = 640001
		//	parallel vector operations needed = 1.92901, GFlops = 1.0368, result = 9.23762e+10
		//	factor threads = 1, vector size = 1280001
		//	parallel vector operations needed = 2.2352, GFlops = 0.894489, result = 1.30598e+11
		//	factor threads = 1, vector size = 2560001
		//	parallel vector operations needed = 2.39387, GFlops = 0.835199, result = 1.84693e+11
		//	factor threads = 1, vector size = 5120001
		//	parallel vector operations needed = 2.50272, GFlops = 0.797853, result = 2.60861e+11
		//	factor threads = 1, vector size = 10240001
		//	parallel vector operations needed = 2.63581, GFlops = 0.757566, result = 3.68913e+11
		//	factor threads = 1, vector size = 20480001
		//	parallel vector operations needed = 2.64285, GFlops = 0.751673, result = 5.19046e+11

		//factorThreads = 1: (with AVX2)
		//vector operations needed = 1.61493, GFlops = 1.23857, result = 1.15479e+10
		//	factor threads = 1, vector size = 10001
		//	parallel vector operations needed = 0.627708, GFlops = 3.18651, result = 1.15479e+10
		//	factor threads = 1, vector size = 20001
		//	parallel vector operations needed = 0.521897, GFlops = 3.83236, result = 1.63305e+10
		//	factor threads = 1, vector size = 40001
		//	parallel vector operations needed = 0.444163, GFlops = 4.50297, result = 2.30944e+10
		//	factor threads = 1, vector size = 80001
		//	parallel vector operations needed = 0.413542, GFlops = 4.83633, result = 3.26602e+10
		//	factor threads = 1, vector size = 160001
		//	parallel vector operations needed = 0.404306, GFlops = 4.94677, result = 4.61882e+10
		//	factor threads = 1, vector size = 320001
		//	parallel vector operations needed = 0.428432, GFlops = 4.6682, result = 6.53199e+10
		//	factor threads = 1, vector size = 640001
		//	parallel vector operations needed = 1.10868, GFlops = 1.80395, result = 9.23762e+10
		//	factor threads = 1, vector size = 1280001
		//	parallel vector operations needed = 1.78428, GFlops = 1.12054, result = 1.30598e+11
		//	factor threads = 1, vector size = 2560001
		//	parallel vector operations needed = 2.19958, GFlops = 0.908976, result = 1.84693e+11
		//	factor threads = 1, vector size = 5120001
		//	parallel vector operations needed = 2.23269, GFlops = 0.894349, result = 2.60861e+11
		//	factor threads = 1, vector size = 10240001
		//	parallel vector operations needed = 2.28973, GFlops = 0.872068, result = 3.68913e+11
		//	factor threads = 1, vector size = 20480001
		//	parallel vector operations needed = 2.27436, GFlops = 0.873458, result = 5.19046e+11
//factorThreads = 4: (with AVX2)
		//vector operations needed = 1.54754, GFlops = 1.29251, result = 1.15479e+10
		//	factor threads = 4, vector size = 10001
		//	parallel vector operations needed = 0.708955, GFlops = 2.82134, result = 1.15479e+10
		//	factor threads = 4, vector size = 20001
		//	parallel vector operations needed = 0.563439, GFlops = 3.54981, result = 1.63305e+10
		//	factor threads = 4, vector size = 40001
		//	parallel vector operations needed = 0.42293, GFlops = 4.72903, result = 2.30944e+10
		//	factor threads = 4, vector size = 80001
		//	parallel vector operations needed = 0.372774, GFlops = 5.36525, result = 3.26602e+10
		//	factor threads = 4, vector size = 160001
		//	parallel vector operations needed = 0.349975, GFlops = 5.71474, result = 4.61882e+10
		//	factor threads = 4, vector size = 320001
		//	parallel vector operations needed = 0.348699, GFlops = 5.73563, result = 6.53199e+10
		//	factor threads = 4, vector size = 640001
		//	parallel vector operations needed = 1.16103, GFlops = 1.72261, result = 9.23762e+10
		//	factor threads = 4, vector size = 1280001
		//	parallel vector operations needed = 1.88657, GFlops = 1.05979, result = 1.30598e+11
		//	factor threads = 4, vector size = 2560001
		//	parallel vector operations needed = 2.15939, GFlops = 0.925892, result = 1.84693e+11
		//	factor threads = 4, vector size = 5120001
		//	parallel vector operations needed = 2.24028, GFlops = 0.891317, result = 2.60861e+11
		//	factor threads = 4, vector size = 10240001
		//	parallel vector operations needed = 2.31076, GFlops = 0.864133, result = 3.68913e+11
		//	factor threads = 4, vector size = 20480001
		//	parallel vector operations needed = 2.22821, GFlops = 0.891549, result = 5.19046e+11
//factorThreads = 8: (with AVX2)
		//vector operations needed = 1.6235, GFlops = 1.23203, result = 1.15479e+10
		//	factor threads = 8, vector size = 10001
		//	parallel vector operations needed = 0.924971, GFlops = 2.16245, result = 1.15479e+10
		//	factor threads = 8, vector size = 20001
		//	parallel vector operations needed = 0.64933, GFlops = 3.08025, result = 1.63305e+10
		//	factor threads = 8, vector size = 40001
		//	parallel vector operations needed = 0.488869, GFlops = 4.09118, result = 2.30944e+10
		//	factor threads = 8, vector size = 80001
		//	parallel vector operations needed = 0.386593, GFlops = 5.17346, result = 3.26602e+10
		//	factor threads = 8, vector size = 160001
		//	parallel vector operations needed = 0.344094, GFlops = 5.81241, result = 4.61882e+10
		//	factor threads = 8, vector size = 320001
		//	parallel vector operations needed = 0.362734, GFlops = 5.5137, result = 6.53199e+10
		//	factor threads = 8, vector size = 640001
		//	parallel vector operations needed = 1.07776, GFlops = 1.8557, result = 9.23762e+10
		//	factor threads = 8, vector size = 1280001
		//	parallel vector operations needed = 1.76621, GFlops = 1.13201, result = 1.30598e+11
		//	factor threads = 8, vector size = 2560001
		//	parallel vector operations needed = 2.06247, GFlops = 0.969401, result = 1.84693e+11
		//	factor threads = 8, vector size = 5120001
		//	parallel vector operations needed = 2.16103, GFlops = 0.924003, result = 2.60861e+11
		//	factor threads = 8, vector size = 10240001
		//	parallel vector operations needed = 2.19312, GFlops = 0.910483, result = 3.68913e+11
		//	factor threads = 8, vector size = 20480001
		//	parallel vector operations needed = 2.20869, GFlops = 0.899427, result = 5.19046e+11
//factorThreads = 16: (with AVX2)
		//vector operations needed = 1.63806, GFlops = 1.22108, result = 1.15479e+10
		//	factor threads = 16, vector size = 10001
		//	parallel vector operations needed = 1.15682, GFlops = 1.72906, result = 1.15479e+10
		//	factor threads = 16, vector size = 20001
		//	parallel vector operations needed = 0.870734, GFlops = 2.29703, result = 1.63305e+10
		//	factor threads = 16, vector size = 40001
		//	parallel vector operations needed = 0.584415, GFlops = 3.42231, result = 2.30944e+10
		//	factor threads = 16, vector size = 80001
		//	parallel vector operations needed = 0.46117, GFlops = 4.33685, result = 3.26602e+10
		//	factor threads = 16, vector size = 160001
		//	parallel vector operations needed = 0.378161, GFlops = 5.28878, result = 4.61882e+10
		//	factor threads = 16, vector size = 320001
		//	parallel vector operations needed = 0.380567, GFlops = 5.25534, result = 6.53199e+10
		//	factor threads = 16, vector size = 640001
		//	parallel vector operations needed = 1.01258, GFlops = 1.97515, result = 9.23762e+10
		//	factor threads = 16, vector size = 1280001
		//	parallel vector operations needed = 1.77347, GFlops = 1.12737, result = 1.30598e+11
		//	factor threads = 16, vector size = 2560001
		//	parallel vector operations needed = 2.07719, GFlops = 0.962531, result = 1.84693e+11
		//	factor threads = 16, vector size = 5120001
		//	parallel vector operations needed = 2.17081, GFlops = 0.919843, result = 2.60861e+11
		//	factor threads = 16, vector size = 10240001
		//	parallel vector operations needed = 2.19029, GFlops = 0.911662, result = 3.68913e+11
		//	factor threads = 16, vector size = 20480001
		//	parallel vector operations needed = 2.20516, GFlops = 0.900868, result = 5.19046e+11






////! a very slim and efficient multithreading approach for small loops
//namespace MicroThreading
//{
//	void TaskManager::CreateJob(const std::function<void(TaskInfo&)> & afunc,
//		Index numberOfJobTasks) //numberOfJobTasks == numberOfThreads
//	{
//#ifdef PRINTDEBUGINFORMATION
//		std::cout << "main thread: create job\n";
//#endif
//		func = &afunc;
//
//		ex = nullptr;
//		//COULD BE removed for main thread!
//		//sync[0]->store(1); // , memory_order_release); //needed as it is checked in the end; 
//
//		//restartLoops = false;
//		//restartLoops.store(false, MEMORY_ORDER_TINYTHREAD);
//		newJob.store(true, MEMORY_ORDER_TINYTHREAD);
//
//		//ntasks.store(numberOfJobTasks); // , memory_order_relaxed);
//
//		TaskInfo ti;
//		ti.nthreads = GetNumThreads();
//		ti.ntasks = ti.nthreads; //here both are same!
//		ti.thread_nr = 0;
//		ti.task_nr = ti.thread_nr; //here both are same!
//
//#ifdef PRINTDEBUGINFORMATION
//		std::cout << "main thread: compute job\n";
//#endif
//		try
//		{
//			//just complete own task (expands into range!)
//			(*func)(ti);
//
//		}
//		catch (Exception e)
//		{
//			{
//				std::lock_guard<std::mutex> guard(copyex_mutex);
//				if (ex) { delete ex; }
//				ex = new Exception(e);
//			}
//		}
//
//		//throw exception after work
//		if (ex)
//			throw Exception(*ex);
//
//		//! wait until other threads are ready
//		//==> use loop from 1 .. nThreads-1
//		for (Index i = 1; i < sync.NumberOfItems(); i++)
//		{
//			while (!sync[i]->load(std::memory_order_relaxed)) { ; }//wait until sync is 1
//#ifdef PRINTDEBUGINFORMATION
//			std::cout << " main thread: task " << i << " signal finished\n";
//#endif
//		}
//
//#ifdef PRINTDEBUGINFORMATION
//		std::cout << "main thread: reset job\n";
//#endif
//		newJob.store(false, MEMORY_ORDER_TINYTHREAD);
//		/*newJob = false;*/
//
//		for (Index i = 1; i < sync.NumberOfItems(); i++)
//		{
//			sync[i]->store(0, std::memory_order_relaxed);
//		}
//#ifdef PRINTDEBUGINFORMATION
//		std::cout << "main thread: restart loops\n";
//#endif
//		//restartLoops.store(true, MEMORY_ORDER_TINYTHREAD);
//		//restartLoops = true; //now loops can go into waiting loop
//	}
//
//	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	Index EnterTaskManager()
//	{
//		if (task_manager)
//		{
//			// task manager already running
//			return 0;
//		}
//
//		task_manager = new TaskManager();
//
//#ifndef WIN32
//		// master has maximal priority !
//		int policy;
//		struct sched_param param;
//		pthread_getschedparam(pthread_self(), &policy, &param);
//		param.sched_priority = sched_get_priority_max(policy);
//		pthread_setschedparam(pthread_self(), policy, &param);
//#endif // WIN32
//
//		task_manager->StartWorkers();
//
//		ParallelFor(Range(100), [&](Index i) {; });    // startup
//		return task_manager->GetNumThreads();
//	}
//
//
//	//! if we exit taskmanager with value != 0, work is stopped and task manager removed
//	void ExitTaskManager(Index num_threads)
//	{
//		if (task_manager && num_threads > 0)
//		{
//			task_manager->StopWorkers();
//			delete task_manager;
//			task_manager = nullptr;
//		}
//	}
//
//	inline void TaskManager::StartWorkers()
//	{
//		isRunning = true;
//		restartLoops = false; //for safety, should be already initialized
//		newJob = false; //for safety, should be already initialized
//
//#ifdef PRINTDEBUGINFORMATION
//		std::cout << "start " << num_threads << " workers ****\n";
//#endif
//
//		sync.SetNumberOfItems(num_threads);
//		sync[0] = new std::atomic<Index>(0); //unused for now, but does not cost anything
//
//		//! start (num_threads-1) additional threads (+ main thread)
//		for (Index i = 1; i < num_threads; i++)
//		{
//			std::thread([this, i]() { this->Loop(i); }).detach();
//		}
//
//		while (active_workers < num_threads - 1)
//			;
//	}
//
//	inline void TaskManager::StopWorkers()
//	{
//		isRunning = false;
//
//		//wait for other threads to stop
//		while (active_workers) { ; }
//
//		if (task_manager && num_threads > 0 && sync.NumberOfItems() > 0) //otherwise, do not delete sync!
//		{
//			delete sync[0]; //free memory for sync
//			sync.SetNumberOfItems(0);
//		}
//	}
//
//	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	inline void TaskManager::Loop(Index threadID)
//	{
//		thread_id = threadID;
//#ifdef PRINTDEBUGINFORMATION
//		std::cout << "  start Loop in thread" << thread_id << "\n";
//#endif
//
//		sync[thread_id] = new std::atomic<Index>(0);
//
//		TaskInfo ti;
//		ti.nthreads = GetNumThreads();
//		ti.ntasks = ti.nthreads; //here both are same!
//		ti.thread_nr = thread_id;
//		ti.task_nr = ti.thread_nr; //here both are same!
//
//
//		active_workers++; //register that thread is running; after sync=0 !
//		bool stop = !isRunning.load(MEMORY_ORDER_TINYTHREAD);
//		while (!stop)
//		{
//			//wait for new job; HOT WAIT!:
//			//while (!newJob && isRunning)
//			while (!newJob.load(MEMORY_ORDER_TINYTHREAD) && !stop)
//			{
//				stop = !isRunning.load(std::memory_order_relaxed);
//			}
//
//			if (stop)
//			{
//				break;
//			}
//
//#ifdef PRINTDEBUGINFORMATION
//			std::cout << "  thread" << thread_id << ": compute task\n";
//#endif
//			//now complete task
//			try
//			{
//				//just complete own task (expands into range!)
//				(*func)(ti);
//
//			}
//			catch (Exception e)
//			{
//				{
//					std::lock_guard<std::mutex> guard(copyex_mutex);
//					if (ex) { delete ex; }
//					ex = new Exception(e);
//				}
//			}
//
//#ifdef PRINTDEBUGINFORMATION
//			std::cout << "  thread" << thread_id << ": send signal finished\n";
//#endif
//			sync[thread_id]->store(1, MEMORY_ORDER_TINYTHREAD); // , memory_order_release); //main thread receives message that job is done!
//
//
//			//wait until new loop is released
//			while (sync[thread_id]->load(MEMORY_ORDER_TINYTHREAD)/* && isRunning.load(MEMORY_ORDER_TINYTHREAD)*/) { ; }
//
//#ifdef PRINTDEBUGINFORMATION
//			std::cout << "  thread" << thread_id << ": restart\n";
//#endif
//		}
//
//
//		//finish loops!
//		delete sync[thread_id];
//		//workers_on_node[mynode]--;
//		active_workers--;
//	}
//};
//
