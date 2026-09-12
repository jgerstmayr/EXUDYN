/** ***********************************************************************************************
* @file         rendererPythonInterface.h
* @brief		Provides an interface between renderer thread and python
* @details		All Python functions MUST be called in the main thread; 
*				thus, tasks need to be transferred from renderer to python thread
*
* @author		Gerstmayr Johannes
* @date			2021-05-07 (generated)
* @pre			...
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */
#ifndef RENDERERPYTHONINTERFACE__H
#define RENDERERPYTHONINTERFACE__H


#include "Utilities/BasicDefinitions.h" //includes stdoutput.h
#include "Utilities/BasicFunctions.h"	//includes stdoutput.h
#include "Utilities/ResizableArray.h"	
#include <atomic> //for output buffer semaphore

//! special enum transferred from renderer to python interface
namespace ProcessID
{
	enum Type {
		_None = 0,
		ShowVisualizationSettingsDialog = 1,
		ShowHelpDialog = 2,
		ShowPythonCommandDialog = 3,
		ShowRightMouseSelectionDialog = 4,
        AskYesNo = 5
	};
	//! transform type into string (e.g. for error messages); this is slow and cannot be used during computation!
	inline STDstring GetTypeString(Type var)
	{
		switch (var)
		{
		case _None: return "_None"; break;
		case ShowVisualizationSettingsDialog: return "ShowVisualizationSettingsDialog"; break;
		case ShowHelpDialog: return "ShowHelpDialog"; break;
		case ShowPythonCommandDialog: return "ShowPythonCommandDialog"; break;
        case ShowRightMouseSelectionDialog: return "ShowRightMouseSelectionDialog"; break;
        case AskYesNo: return "AskYesNo"; break;
        default: return "ProcessID::unknown";
		}
	}
}

//! lock renderer callbacks during critical operations 
void PySetRendererCallbackLock(bool flag);

//! get state of callback lock
bool PyGetRendererCallbackLock();

//! lock renderer Python command execution (e.g. opening of dialogs) during critical operations 
void PySetRendererPythonCommandLock(bool flag);

//! get state of Python command execution lock
bool PyGetRendererPythonCommandLock();

//the following is a workaround, because here we do not have simple access to visualizationSettings
//! set state of multithreaded dialog (interaction with renderer during settings dialogs)
void PySetRendererMultiThreadedDialogs(bool flag);

//! get state of multithreaded dialog (interaction with renderer during settings dialogs)
bool PyGetRendererMultiThreadedDialogs();

//! check CTRL+"C" signals
bool PyCheckSignals();

//! this throws an exception for which a (Python) error has already been set, e.g. due to CTRL+"C"
void PyThrowErrorAlreadySet();


//! get/set result of PyProcess action
Index PyProcessGetResult();
void PyProcessSetResult(Index value);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//FILL QUEUES

//! put process ID into queue, which is then called from main (Python) thread
void PyQueuePythonProcess(ProcessID::Type processID, Index info=-1);

//! put executable string into queue, which is then called from main (Python) thread
void PyQueueExecutableString(STDstring str); //call python function and execute string as python code

//! put executable key codes into queue, which are the processed in main (Python) thread
void PyQueueKeyPressed(int key, int action, int mods); //call python user function
//void PyQueueKeyPressed(int key, int action, int mods, std::function<bool(int, int, int)> keyPressUserFunctionInit); //call python user function

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//PROCESS QUEUES

//! function to be called from main (python) thread to process different queues
void PyProcessExecuteQueue(); 

//! process waiting queue: keys
void PyProcessPythonProcessQueue();

//! process waiting queue: strings
void PyProcessExecutableStringQueue(); //call python function, execute string as python code and run ProcessID processes

//! process waiting queue: ProcessID
void PyProcessRendererKeyQueue();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! specific process called:
void PyProcessShowVisualizationSettingsDialog();
void PyProcessShowHelpDialog();
void PyProcessShowPythonCommandDialog();
void PyProcessShowRightMouseSelectionDialog(Index itemID);
void PyProcessAskQuit();

//! execute str as Python commands; lock Renderer during this time to prevent from further commands, Python conflicts and crashing!
//! this function MAY BE ONLY CALLED FROM PYTHON THREAD, NOT from GLFW!!:
void PyProcessExecuteStringAsPython(const STDstring& str, bool lockRendererCallbacks=true, bool lockPythonCommands=true);


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! perform idle tasks for single-threaded renderer
void RendererDoSingleThreadedIdleTasks(Real waitSeconds = 0.);
//! check if renderer is single-threaded
bool RendererIsSingleThreadedOrNotRunning();


#endif //RENDERERPYTHONINTERFACE__H
