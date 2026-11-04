/** ***********************************************************************************************
* @class        PostProcessData
* @brief		
* @details		Details:
 				shared between CSystem and VisualizationSystem
*
* @author		Gerstmayr Johannes
* @date			2021-05-07 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
************************************************************************************************ */
#ifndef POSTPROCESSDATA__H
#define POSTPROCESSDATA__H

#include <atomic>
#include <thread>

class MainSystem; 
class VisualizationSystem; //for backlink to VisualizationSystem for PythonUserFunctions for drawing


//! @brief class that contains relevant data for postprocessing:
//! - postProcessState
//! - flag to show that results have been updated
class PostProcessData
{
private:
	std::string solverMessage;			//!< additional solver message shown in renderer window regarding solver and state; this variable is private, because it cannot be accessed safely in multithreading
	std::string solutionMessage;		//!< additional solution information shown in renderer window; this variable is private, because it cannot be accessed safely in multithreading
	CSystemState visualizationStateUpdate; //!< proposed update for visualizationState; will be used as soon as current rendering is finished
	bool visualizationStateUpdateAvailable;//!< if this is true, a visualizationState update is available

public:
	std::atomic_flag accessState;		//!< flag, which is locked / released to access data
	std::atomic_flag accessMessage;		//!< flag, which is locked / released to access messages
	bool postProcessDataReady;			//!< signals, that data can be plotted (CSystem must be consistent, state is a current state of the CSystem, ...); usually same as CSystem::systemIsConsistent
	uint64_t updateCounter;				//!< updateCounter is increased upon every update of state; can be used to judge graphics update; for 1 billion steps/second counter goes for 585 years before overflow
	uint64_t recordImageCounter;		//!< updateCounter is increased upon every update of state; can be used to judge graphics update; for 1 billion steps/second counter goes for 585 years before overflow
	bool simulationFinished;			//!< shows that computation has been finished ==> visualize last step
	bool stopSimulation;				//!< renderer or GUI sends signal that simulation shall be interrupted
	bool forceQuitSimulation;			//!< flag is set if user closes simulation window (but not if Escape pressed)
	bool simulationPaused;				//!< true: simulation sends renderer or GUI that simulation is paused, waiting for user-input (SPACE)
	Real visualizationTime;				//!< this value is set as soon as the computation date is generated from visualization; needed to synchronize text message and visualization state
	bool systemHasChanged;				//!< systemHasChanged is used to signal GLFWclient to compute new maxSceneSize and center

	VisualizationSystem* visualizationSystem; //!< this backlink ist used for: user functions, WaitForUserToContinue and for MainSystem backlink (but use rarely!!!)
	bool requestUserFunctionDrawing;	//!< if this flag is set, user functions request drawing update from computation thread
	bool userFunctionOutsideMainThreadReported = false; //!< a call of a user function from the render thread was reported (#3003)
	std::atomic_flag requestUserFunctionDrawingAtomicFlag;  //!< flag for user function drawing in python
	//! the lock of the items of the system (#3239): taken by the main thread while it resets, adds, deletes or modifies
	//! items or assembles, by the render thread while it reads them; not released by Reset()
	std::atomic_flag accessSystem;
	std::atomic<std::thread::id> accessSystemOwner; //!< the thread holding accessSystem, so that a nested lock passes
	bool inUserFunctionDrawing = false; //!< the main thread computes the user function drawings, which the render thread waits for


	PostProcessData()
	{
		Reset();
		accessSystem.clear(std::memory_order_release);
		accessSystemOwner.store(std::thread::id());
	}

	//! lock the items of the system (#3239); false if this thread holds the lock already, or computes the user function
	//! drawings the render thread waits for (nothing to release then); with serveDrawing - the main thread - the user
	//! function drawings the render thread requests while it holds the lock are computed while waiting
	bool LockSystemAccess(bool serveDrawing);
	//! release the lock of LockSystemAccess, if it returned true
	void ReleaseSystemAccess(bool locked);

	//! reset data according to MainSystem.Reset()
	void Reset();

	//! access to private visualizationStateUpdate, may be changed in future
	const CSystemState& GetVisualizationStateUpdate() const { return visualizationStateUpdate; }
	//! access to private visualizationStateUpdate, may be changed in future
	CSystemState& GetVisualizationStateUpdate() { return visualizationStateUpdate; }
	//! access to private visualizationStateUpdateAvailable
	bool GetVisualizationStateUpdateAvailable() const { return visualizationStateUpdateAvailable; }
	void SetVisualizationStateUpdateAvailable(bool flag) { visualizationStateUpdateAvailable = flag; }

	//! this function is used to only send a signal that the scene shall be redrawn because the visualization state has been updated
	void SendRedrawSignal();

	//! send flag to GUI / renderer which signals that simulation is interrupted until user interaction
	void WaitForUserToContinue(bool printMessage=true);

	//! uses some backlinks for solver to find out if visualization is running (could also be checked directly via GLFWrenderer global variable)
	bool VisualizationIsRunning() const; 

	//! set a visualization message into openGL window
	void SetSolverMessage(const std::string& solverMessageInit);

	void SetSolutionMessage(const std::string& solutionMessageInit);

	//! get the current solver message string
	std::string GetSolverMessage();

	//! get the current solution message string
	std::string GetSolutionMessage();

	void ProcessUserFunctionDrawing();

};

//! the lock of the items of a system for a scope (#3239)
class SystemAccessGuard
{
	PostProcessData& postProcessData;
	bool locked;
public:
	SystemAccessGuard(PostProcessData& data, bool serveDrawing, bool active = true) : postProcessData(data),
		locked(active && data.LockSystemAccess(serveDrawing)) {}
	~SystemAccessGuard() { postProcessData.ReleaseSystemAccess(locked); }
	SystemAccessGuard(const SystemAccessGuard&) = delete;
	SystemAccessGuard& operator=(const SystemAccessGuard&) = delete;
};


#endif
