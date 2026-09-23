/** ***********************************************************************************************
* @file         rendererPythonInterface.cpp
* @brief		Provides implementation for interaction between renderer, Python and MainSystem(Container)
* @details		All Python functions MUST be called in the main thread;
*				
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


#include "Main/rendererPythonInterface.h"

#include <pybind11/pybind11.h>
#include <pybind11/eval.h>
#include <thread>
#include <pybind11/stl.h>
//#include <pybind11/stl_bind.h>
//#include <pybind11/operators.h>
//#include <pybind11/numpy.h>
//does not work globally: #include <pybind11/iostream.h> //used to redirect cout:  py::scoped_ostream_redirect output;
//#include <pybind11/cast.h> //for arguments
#include <pybind11/functional.h> //for functions
#include <atomic> //for output buffer semaphore
#include "Graphics/GlfwClient.h"
namespace py = pybind11;

#include "Main/Experimental.h"
extern PySpecial pySpecial;			//! special features; affects exudyn globally; treat with care

#ifdef USE_GLFW_GRAPHICS
GlfwRenderer& GetGlfwRenderer() { return glfwRenderer; }
extern Real PyReadRealFromSysDictionary(const STDstring& key);
extern void PyWriteToSysDictionary(const STDstring& key, py::object item);
#endif // USE_GLFW_GRAPHICS



namespace py = pybind11;

extern bool deactivateGlobalPyRuntimeErrorFlag;

const int queuedPythonProcessIDlistLength = 2;					//!< amount of entries
//these are global variables, as they are accessed from GLFW and from main part
std::atomic_flag queuedPythonProcessAtomicFlag = ATOMIC_FLAG_INIT;//!< flag for queued processID
ResizableArray<SlimArray<int, queuedPythonProcessIDlistLength>>  queuedPythonProcessIDlist;	//!< this queued (processID, processInformation)
bool rendererCallbackLock = false;								//!< callbacks deactivated as long as Python dialogs open (avoid crashes)
bool rendererPythonCommandLock = false;							//!< callbacks deactivated as long as Python dialogs open (avoid crashes)
bool rendererMultiThreadedDialogs = true;						//!< renderer stays interactive during rendering (immediate apply of changes, e.g., visualizationSettings)
Index processResult = 0;                                        //!< result of PyProcess (if available)

Index PyProcessGetResult() { return processResult; }
void PyProcessSetResult(Index value) { processResult = value; }


std::atomic_flag queuedPythonExecutableCodeAtomicFlag = ATOMIC_FLAG_INIT;			//!< flag for executable python code (String)
STDstring queuedPythonExecutableCodeStr;						//!< this string contains (accumulated) python code which shall be executed

std::atomic_flag queuedRendererKeyListAtomicFlag = ATOMIC_FLAG_INIT;	//!< flag for queuedRendererKeyList
ResizableArray<SlimArray<int, 3>> queuedRendererKeyList;	//!< this list contains keys that are transferred to python
std::function<int(int, int, int)> keyPressUserFunction = 0; //!< must be set by GLFW, before that nothing is done; should not be changed too often, as it is not stored in list

//! lock renderer callbacks during critical operations 
void PySetRendererCallbackLock(bool flag) { rendererCallbackLock = flag; }

//! get state of callback lock
bool PyGetRendererCallbackLock() { return rendererCallbackLock; }

//! lock renderer callbacks during critical operations 
void PySetRendererPythonCommandLock(bool flag) { rendererPythonCommandLock = flag; }

//! get state of callback lock
bool PyGetRendererPythonCommandLock() { return rendererPythonCommandLock; }

//! set state of multithreaded dialog (interaction with renderer during settings dialogs)
void PySetRendererMultiThreadedDialogs(bool flag) { rendererMultiThreadedDialogs = flag; }

//! get state of multithreaded dialog (interaction with renderer during settings dialogs)
bool PyGetRendererMultiThreadedDialogs() { return rendererMultiThreadedDialogs; }

//! check CTRL+"C" signals
bool PyCheckSignals()
{
	return (PyErr_CheckSignals() != 0);
}


//! this throws an exception for which a (Python) error has already been set, e.g. due to CTRL+"C"
void PyThrowErrorAlreadySet()
{
	if (pySpecial.solver.throwErrorWithCtrlC)
	{
		//pout << "raised PyThrowErrorAlreadySet\n" << std::flush;
		throw py::error_already_set();
	}
	//else if (pySpecial.solver.throwErrorWithCtrlC == 1)
	//{
	//	throw py::error_already_set();
	//}
	//else if (pySpecial.solver.throwErrorWithCtrlC == 2)
	//{
	//	try {
	//		throw py::error_already_set();
	//	}
	//	catch (py::error_already_set& eas) {
	//		eas.discard_as_unraisable(__func__); //remove unraisable error
	//	}
	//	CHECKandTHROWstring("Simulation stopped with CTRL-C");
	//}
	//else if (pySpecial.solver.throwErrorWithCtrlC == 3)
	//{
	//	//works in console
	//}
}

//! put process ID into queue, which is then called from main (Python) thread
void PyQueuePythonProcess(ProcessID::Type processID, Index info)
{
	EXUstd::WaitAndLockSemaphore(queuedPythonProcessAtomicFlag);
	queuedPythonProcessIDlist.Append(SlimArray<int, queuedPythonProcessIDlistLength>({ processID, info}));
	EXUstd::ReleaseSemaphore(queuedPythonProcessAtomicFlag); 

    //do not process here: will not work on Apple, as it is done inside key callback function
	//if (RendererIsSingleThreadedOrNotRunning()) { PyProcessPythonProcessQueue(); PyProcessExecutableStringQueue();  } //immediately process queue...+executable string for right-mouse-button
}

//! put executable string into queue, which is then called from main (Python) thread
void PyQueueExecutableString(STDstring str) //call python function and execute string as python code
{
	EXUstd::WaitAndLockSemaphore(queuedPythonExecutableCodeAtomicFlag); //lock queuedPythonExecutableCodeStr
	queuedPythonExecutableCodeStr += '\n' + str; //for safety add a "\n", as the last command may include spaces, tabs, ... at the end
	EXUstd::ReleaseSemaphore(queuedPythonExecutableCodeAtomicFlag); //clear queuedPythonExecutableCodeStr

	//if (RendererIsSingleThreadedOrNotRunning()) { PyProcessExecutableStringQueue(); } //immediately process queue...
}

//! put executable key codes into queue, which are the processed in main (Python) thread
void PyQueueKeyPressed(int key, int action, int mods)
//void PyQueueKeyPressed(int key, int action, int mods, std::function<bool(int, int, int)> keyPressUserFunctionInit) //call python user function
{
	EXUstd::WaitAndLockSemaphore(queuedRendererKeyListAtomicFlag); //lock queuedRendererKeyListAtomicFlag
	queuedRendererKeyList.Append(SlimArray<int, 3>({ key, action, mods }));
	EXUstd::ReleaseSemaphore(queuedRendererKeyListAtomicFlag); //clear queuedRendererKeyListAtomicFlag

	//if (RendererIsSingleThreadedOrNotRunning()) { PyProcessRendererKeyQueue(); } //immediately process queue...
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//function to execute regularly the queues
void PyProcessExecuteQueue() //call python function and execute string as python code
{
	PyProcessPythonProcessQueue();

	PyProcessExecutableStringQueue();

	PyProcessRendererKeyQueue();
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! process waiting queue: ProcessID
void PyProcessPythonProcessQueue()
{
	EXUstd::WaitAndLockSemaphore(queuedPythonProcessAtomicFlag); //lock queuedPythonExecutableCodeStr
	if (queuedPythonProcessIDlist.NumberOfItems() != 0)
	{
		//EXUstd::WaitAndLockSemaphore(graphicsUpdateAtomicFlag); //lock queuedRendererKeyListAtomicFlag
		ProcessID::Type processID = (ProcessID::Type)(queuedPythonProcessIDlist[0][0]);
		Index processInfo = queuedPythonProcessIDlist[0][1];
		queuedPythonProcessIDlist.Remove(0); //remove first index from list

		EXUstd::ReleaseSemaphore(queuedPythonProcessAtomicFlag); //clear queuedPythonExecutableCodeStr
		deactivateGlobalPyRuntimeErrorFlag = true; //errors will not crash the render window

		try //catch exceptions; user may want to continue after a illegal python command 
		{
			switch (processID)
			{
			case ProcessID::_None:
				break;
			case ProcessID::ShowVisualizationSettingsDialog:
				PyProcessShowVisualizationSettingsDialog();  break;
			case ProcessID::ShowHelpDialog:
				PyProcessShowHelpDialog(); break;
			case ProcessID::ShowPythonCommandDialog:
				PyProcessShowPythonCommandDialog();  break;
			case ProcessID::ShowRightMouseSelectionDialog:
				PyProcessShowRightMouseSelectionDialog(processInfo);  break;
            case ProcessID::AskYesNo:
                PyProcessAskQuit(); break;
            default:
				break;
			}
		}
		//mostly catches python errors:
		catch (pybind11::error_already_set& ex)
		{
            PyProcessSetResult(-2); //error
			PyWarning("Error when executing process " + ProcessID::GetTypeString(processID) + +"':\n" + STDstring(ex.what()) + "\n; maybe a module is missing!");
			deactivateGlobalPyRuntimeErrorFlag = false;
            // Discard the Python error using Python APIs, using the C++ magic
            // variable __func__. Python already knows the type and value and of the
            // exception object.
            ex.discard_as_unraisable(__func__); //see if this works and avoids further exceptions
			//throw; //avoid multiple exceptions trown again 
		}
		catch (const EXUexception& ex)
		{
            PyProcessSetResult(-2); //error
            //EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag); //clear 
			PyWarning("Error when executing process " + ProcessID::GetTypeString(processID) +
				":\n" + STDstring(ex.what()) + "\n; maybe a module is missing!!");
			deactivateGlobalPyRuntimeErrorFlag = false;
			throw; //avoid multiple exceptions trown again 
			//throw(ex); //avoid multiple exceptions trown again 
		}
		catch (...) //any other exception
		{
            PyProcessSetResult(-2); //error
            //EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag); //clear 
			PyWarning("Error when executing process " + ProcessID::GetTypeString(processID) + "\nmaybe a module is missing and check your Python code!!");
		}
		//EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag); 
		deactivateGlobalPyRuntimeErrorFlag = false;
	}
	else
	{
		EXUstd::ReleaseSemaphore(queuedPythonProcessAtomicFlag); //clear queuedPythonExecutableCodeStr
	}
}

//! process waiting queue: strings
void PyProcessExecutableStringQueue()
{
	EXUstd::WaitAndLockSemaphore(queuedPythonExecutableCodeAtomicFlag); //lock queuedPythonExecutableCodeStr
	if (queuedPythonExecutableCodeStr.size())
	{
		//EXUstd::WaitAndLockSemaphore(graphicsUpdateAtomicFlag); //lock queuedRendererKeyListAtomicFlag

		STDstring execStr = queuedPythonExecutableCodeStr;
		queuedPythonExecutableCodeStr.clear();

		EXUstd::ReleaseSemaphore(queuedPythonExecutableCodeAtomicFlag); //clear queuedPythonExecutableCodeStr
		deactivateGlobalPyRuntimeErrorFlag = true; //errors will not crash the render window

		try //catch exceptions; user may want to continue after a illegal python command 
		{
			py::object scope = py::module::import("__main__").attr("__dict__"); //use this to enable access to mbs and other variables of global scope within test models suite
			py::exec(execStr.c_str(), scope);
		}
		//mostly catches python errors:
		catch (const pybind11::error_already_set& ex)
		{
			//EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag); 
			PyWarning("Error when executing '" + STDstring(execStr) + "':\n" + STDstring(ex.what()) + "\n; maybe a module is missing!");
			deactivateGlobalPyRuntimeErrorFlag = false;
			throw; //avoid multiple exceptions trown again; see notes in pybind11: Any Python error must be thrown or cleared, or Python/pybind11 will be left in an invalid state
		}
		catch (const EXUexception& ex)
		{
			//EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag); //clear 
			PyWarning("Error when executing '" + STDstring(execStr) + "':\n" + STDstring(ex.what()) + "\n; maybe a module is missing!!");
			deactivateGlobalPyRuntimeErrorFlag = false;
			throw; //avoid multiple exceptions trown again see notes in pybind11: Any Python error must be thrown or cleared, or Python/pybind11 will be left in an invalid state
		}
		catch (...) //any other exception
		{
			//EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag); //clear 
			PyWarning("Error when executing '" + STDstring(execStr) + "'\nmaybe a module is missing and check your Python code!!");
		}
		//EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag); 
		deactivateGlobalPyRuntimeErrorFlag = false;
	}
	else
	{
		EXUstd::ReleaseSemaphore(queuedPythonExecutableCodeAtomicFlag); //clear queuedPythonExecutableCodeStr
	}

}

//! process waiting queue: keys
void PyProcessRendererKeyQueue()
{
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//process pressed keys:
	EXUstd::WaitAndLockSemaphore(queuedRendererKeyListAtomicFlag); //lock queuedPythonExecutableCodeStr
	if (queuedRendererKeyList.NumberOfItems() != 0)
	{
		//EXUstd::WaitAndLockSemaphore(graphicsUpdateAtomicFlag); //lock queuedRendererKeyListAtomicFlag
		ResizableArray<SlimArray<int, 3>> keyList = queuedRendererKeyList; //immediately copy list for small interaction with graphics part
		//std::cout << "keylist=" << keyList << "\n";
		bool glfwInitialized = false;
#ifdef USE_GLFW_GRAPHICS
		glfwInitialized = GetGlfwRenderer().IsGlfwInitAndRendererActive();
#endif //USE_GLFW_GRAPHICS
		if (glfwInitialized) //otherwise makes no sense ...! ==> ignore
		{
#ifdef USE_GLFW_GRAPHICS
			//keyPressUserFunction = keyPressUserFunctionInit;
			std::function<int(int, int, int)> localKeyPressUserFunction = GetGlfwRenderer().GetKeyPressUserFunction();
			queuedRendererKeyList.SetNumberOfItems(0); //clear list

			EXUstd::ReleaseSemaphore(queuedRendererKeyListAtomicFlag); //clear queuedPythonExecutableCodeStr

			deactivateGlobalPyRuntimeErrorFlag = true; //errors will not crash the render window

			if (localKeyPressUserFunction) //check if function is available!
			{
				for (auto key : keyList)
				{
					//std::cout << "call key=" << key << "\n";
					try //catch exceptions; user may want to continue after a illegal python command 
					{
						//bool rv = //rv not used right now, because it is received at a time where it is too late for graphics
						localKeyPressUserFunction(key[0], key[1], key[2]);
					}
					//mostly catches python errors:
					catch (const pybind11::error_already_set& ex)
					{
						//EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag);
						PyWarning("Error when executing key press function with key " + EXUstd::ToString(key) + "':\n" + STDstring(ex.what()) + "\n; check function parameters!");
						deactivateGlobalPyRuntimeErrorFlag = false;
						throw; //avoid multiple exceptions trown again (don't know why!)!
					}
					catch (const EXUexception& ex)
					{
						//EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag);
						PyWarning("Error when executing key press function with key " + EXUstd::ToString(key) + "':\n" + STDstring(ex.what()) + "\n; check function parameters!");
						deactivateGlobalPyRuntimeErrorFlag = false;
						throw; //avoid multiple exceptions trown again (don't know why!)!
						//throw(ex); //avoid multiple exceptions trown again (don't know why!)!
					}
					catch (...) //any other exception
					{
						//EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag);
						PyWarning("Error when executing key press function with key " + EXUstd::ToString(key) + "\n; check function parameters!");
					}
					//EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag);
				}
			}
			deactivateGlobalPyRuntimeErrorFlag = false;
#endif //USE_GLFW_GRAPHICS
		}
		else
		{
			//remove waiting items if renderer not running
			queuedRendererKeyList.SetNumberOfItems(0); //clear list
			EXUstd::ReleaseSemaphore(queuedRendererKeyListAtomicFlag); //clear queuedPythonExecutableCodeStr
		}
		//std::cout << "key process finished\n";
	}
	else
	{
		EXUstd::ReleaseSemaphore(queuedRendererKeyListAtomicFlag); //clear queuedPythonExecutableCodeStr
	}

}




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void PyProcessShowVisualizationSettingsDialog()
{
#ifdef USE_GLFW_GRAPHICS
    //open window to execute a python command ... 
    std::string str = R"PY(
try:
    import exudyn.misc.GUI   #this may fail if tkinter is missing
    try:
        exudyn.misc.GUI.ShowVisualizationSettingsDialog()
    except Exception as exceptionVariable:
        print("edit dialog for visualizationSettings failed")
        print(exceptionVariable) #not necessary, but can help to identify reason
except ImportError:
    print("edit dialog for visualizationSettings failed: cannot import exudyn.misc.GUI / tkinter; tkinter probably missing")
)PY";
    PyProcessExecuteStringAsPython(str, !PyGetRendererMultiThreadedDialogs(), true);
#endif // USE_GLFW_GRAPHICS
}



void PyProcessShowHelpDialog()
{
#ifdef USE_GLFW_GRAPHICS

    std::string str = R"PY(
try:
    import exudyn.misc.GUI   #this may fail if tkinter is missing
    try:
        exudyn.misc.GUI.ShowHelpDialog()
    except Exception as exceptionVariable:
        print("help dialog failed")
        print(exceptionVariable) #not necessary, but can help to identify reason
except ImportError:
    print("help dialog failed: cannot import exudyn.misc.GUI / tkinter; tkinter probably missing")
)PY";
    PyProcessExecuteStringAsPython(str, !PyGetRendererMultiThreadedDialogs(), true);
#endif // USE_GLFW_GRAPHICS

}



void PyProcessShowPythonCommandDialog()
{
#ifdef USE_GLFW_GRAPHICS

    std::string str = R"PY(
try:
    import exudyn.misc.GUI   #this may fail if tkinter is missing
    try:
        exudyn.misc.GUI.ShowPythonCommandDialog()
    except Exception as exceptionVariable:
        print("command window failed")
        print(exceptionVariable) #not necessary, but can help to identify reason
except ImportError:
    print("command window failed: cannot import exudyn.misc.GUI / tkinter; tkinter probably missing")
)PY";
    PyProcessExecuteStringAsPython(str, !PyGetRendererMultiThreadedDialogs(), true);
#endif // USE_GLFW_GRAPHICS

}

void PyProcessShowRightMouseSelectionDialog(Index itemID)
{
#ifdef USE_GLFW_GRAPHICS //only works with renderer active
    GetGlfwRenderer().PySetRendererSelectionDict(itemID);
    STDstring str = R"PY(
try:
    import exudyn.misc.GUI   #this may fail if tkinter is missing
    try:
        exudyn.misc.GUI.ShowRightMouseSelectionDialog()
    except Exception as exceptionVariable:
        print("showing of dictionary failed")
        print(exceptionVariable) #not necessary, but can help to identify reason
except ImportError:
    print("showing of dictionary failed: cannot import exudyn.misc.GUI / tkinter; tkinter probably missing")
)PY";
    PyProcessExecuteStringAsPython(str, !PyGetRendererMultiThreadedDialogs(), true);
#endif // USE_GLFW_GRAPHICS

}

void PyProcessAskQuit()
{
#ifdef USE_GLFW_GRAPHICS
    PyProcessSetResult(1);

    try
    {
        PyWriteToSysDictionary("quitResponse", py::cast((int)1) );

        std::string str = R"PY(
try:
    import exudyn.misc.GUI   #this may fail if tkinter is missing
    exudyn.misc.GUI.AskQuitDialog()
except Exception:
    pass #if fails, user shall not be notified
)PY";
        PyProcessExecuteStringAsPython(str, !PyGetRendererMultiThreadedDialogs(), true);
        PyProcessSetResult((Index)PyReadRealFromSysDictionary("quitResponse"));
    }
    catch (pybind11::error_already_set& ex)
    {
        ex.discard_as_unraisable(__func__); //see if this works and avoids further exceptions
        PyProcessSetResult(-2); //error
        pout << "to quit in long running simulations without tkinter, press Q twice!";
    }
    catch (...) //any other exception
    {
        PyProcessSetResult(-2); //error
    }

    if (PyProcessGetResult() == 1) { PyProcessSetResult(-2); } //this indicates that an exception occurred (tkinter not available, ...)
#endif // USE_GLFW_GRAPHICS
}



void PyProcessExecuteStringAsPython(const STDstring& str, bool lockRendererCallbacks, bool lockPythonCommands)
{
    py::object scope = py::module::import("__main__").attr("__dict__"); //use this to enable access to mbs and other variables of global scope within test models suite
    PySetRendererCallbackLock(lockRendererCallbacks);
    PySetRendererPythonCommandLock(lockPythonCommands);
    py::exec(str.c_str(), scope);
    PySetRendererCallbackLock(false);
    PySetRendererPythonCommandLock(false);
}

//! check if renderer is single-threaded or not running
bool RendererIsSingleThreadedOrNotRunning()
{
#ifdef USE_GLFW_GRAPHICS //only works with renderer active
    if (!GetGlfwRenderer().UseMultiThreadedRendering() || !GetGlfwRenderer().IsGlfwInitAndRendererActive())
    {
		return true;
	}
	return false;
#else
	return true; //in case that no GLFW used, renderer behaves always like single-threaded
#endif // USE_GLFW_GRAPHICS
}

//! perform idle tasksfor single-threaded renderer
void RendererDoSingleThreadedIdleTasks(Real waitSeconds)
{
	//this may only be done from main thread; used for PrintDelayed:
	outputBuffer.overflowFlush(0, false, true); //clear buffer, in particular visualization buffer!
#ifdef USE_GLFW_GRAPHICS //only works with renderer active
	if (GetGlfwRenderer().IsGlfwInitAndRendererActive() && !GetGlfwRenderer().UseMultiThreadedRendering())
	{
		GetGlfwRenderer().DoRendererIdleTasks(waitSeconds);
	}
#endif // USE_GLFW_GRAPHICS
}

