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
	uint64_t recordImageCounter;				//!< updateCounter is increased upon every update of state; can be used to judge graphics update; for 1 billion steps/second counter goes for 585 years before overflow
	bool simulationFinished;			//!< shows that computation has been finished ==> visualize last step
	bool stopSimulation;				//!< renderer or GUI sends signal that simulation shall be interrupted
	bool forceQuitSimulation;			//!< flag is set if user closes simulation window (but not if Escape pressed)
	bool simulationPaused;				//!< true: simulation sends renderer or GUI that simulation is paused, waiting for user-input (SPACE)
	Real visualizationTime;				//!< this value is set as soon as the computation date is generated from visualization; needed to synchronize text message and visualization state
	bool systemHasChanged;				//!< systemHasChanged is used to signal GLFWclient to compute new maxSceneSize and center

	VisualizationSystem* visualizationSystem; //!< this backlink ist used for: user functions, WaitForUserToContinue and for MainSystem backlink (but use rarely!!!)
	bool requestUserFunctionDrawing;	//!< if this flag is set, user functions request drawing update from computation thread
	std::atomic_flag requestUserFunctionDrawingAtomicFlag;  //!< flag for user function drawing in python


	PostProcessData()
	{
		Reset();
	}

	//! reset data according to MainSystem.Reset()
	void Reset()
	{
		postProcessDataReady = false;
		updateCounter = 1;				// must be larger than recordImageCounter, in order to avoid hang up in first UpdatePostProcessData(...) in CSystem
		recordImageCounter = 0;
		simulationFinished = false;
		stopSimulation = false;
		forceQuitSimulation = false;
		simulationPaused = false;
		visualizationTime = 0;
		systemHasChanged = true; // used to compute maxSceneSize at beginning

		visualizationStateUpdateAvailable = false;
		requestUserFunctionDrawing = false;

		solverMessage = "";
		solutionMessage = "";

		EXUstd::ReleaseSemaphore(accessState);
		EXUstd::ReleaseSemaphore(requestUserFunctionDrawingAtomicFlag);
		EXUstd::ReleaseSemaphore(accessMessage);
	}

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
	void SetSolverMessage(const std::string& solverMessageInit)
	{
		//add separate semaphore for PostProcessData.accessMessages
		EXUstd::WaitAndLockSemaphore(accessMessage); //lock PostProcessData
		solverMessage = solverMessageInit;
		EXUstd::ReleaseSemaphore(accessMessage); //clear PostProcessData
	}

	void SetSolutionMessage(const std::string& solutionMessageInit)
	{
		EXUstd::WaitAndLockSemaphore(accessMessage); //lock PostProcessData
		solutionMessage = solutionMessageInit;
		EXUstd::ReleaseSemaphore(accessMessage); //clear PostProcessData
	}

	//! get the current solver message string
	std::string GetSolverMessage()
	{
		EXUstd::WaitAndLockSemaphore(accessMessage); //lock PostProcessData
		std::string message = solverMessage; 		//copy message
		EXUstd::ReleaseSemaphore(accessMessage); //clear PostProcessData
		return message; //now safely return message
	}

	//! get the current solution message string
	std::string GetSolutionMessage()
	{
		EXUstd::WaitAndLockSemaphore(accessMessage); //lock PostProcessData
		std::string message = solutionMessage; 		//copy message
		EXUstd::ReleaseSemaphore(accessMessage); //clear PostProcessData
		return message; //now safely return message
	}

	void ProcessUserFunctionDrawing();

};


#endif
