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


#ifdef USE_GLFW_GRAPHICS
GlfwRenderer& GetGlfwRenderer() { return glfwRenderer; }
#endif // USE_GLFW_GRAPHICS

namespace py = pybind11;

extern bool deactivateGlobalPyRuntimeErrorFlag;

const int queuedPythonProcessIDlistLength = 2;					//!< amount of entries
//these are global variables, as they are accessed from GLFW and from main part
std::atomic_flag queuedPythonProcessAtomicFlag = ATOMIC_FLAG_INIT;//!< flag for queued processID
ResizableArray<SlimArray<int, queuedPythonProcessIDlistLength>>  queuedPythonProcessIDlist;	//!< this queued (processID, processInformation)
bool rendererCallbackLock = false;								//!< callbacks deactivated as long as Python dialogs open (avoid crashes)

std::atomic_flag queuedPythonExecutableCodeAtomicFlag = ATOMIC_FLAG_INIT;			//!< flag for executable python code (String)
STDstring queuedPythonExecutableCodeStr;						//!< this string contains (accumulated) python code which shall be executed

std::atomic_flag queuedRendererKeyListAtomicFlag = ATOMIC_FLAG_INIT;	//!< flag for queuedRendererKeyList
ResizableArray<SlimArray<int, 3>> queuedRendererKeyList;	//!< this list contains keys that are transferred to python
std::function<int(int, int, int)> keyPressUserFunction = 0; //!< must be set by GLFW, before that nothing is done; should not be changed too often, as it is not stored in list

//! lock renderer callbacks during critical operations 
void PySetRendererCallbackLock(bool flag)
{
	rendererCallbackLock = flag;
}

//! get state of callback lock
bool PyGetRendererCallbackLock() { return rendererCallbackLock; }


//! put process ID into queue, which is then called from main (Python) thread
void PyQueuePythonProcess(ProcessID::Type processID, Index info)
{
	EXUstd::WaitAndLockSemaphore(queuedPythonProcessAtomicFlag);
	queuedPythonProcessIDlist.Append(SlimArray<int, queuedPythonProcessIDlistLength>({ processID, info}));
	EXUstd::ReleaseSemaphore(queuedPythonProcessAtomicFlag); 

	if (RendererIsSingleThreadedOrNotRunning()) { PyProcessPythonProcessQueue(); PyProcessExecutableStringQueue();  } //immediately process queue...+executable string for right-mouse-button
}

//! put executable string into queue, which is then called from main (Python) thread
void PyQueueExecutableString(STDstring str) //call python function and execute string as python code
{
	EXUstd::WaitAndLockSemaphore(queuedPythonExecutableCodeAtomicFlag); //lock queuedPythonExecutableCodeStr
	queuedPythonExecutableCodeStr += '\n' + str; //for safety add a "\n", as the last command may include spaces, tabs, ... at the end
	EXUstd::ReleaseSemaphore(queuedPythonExecutableCodeAtomicFlag); //clear queuedPythonExecutableCodeStr

	if (RendererIsSingleThreadedOrNotRunning()) { PyProcessExecutableStringQueue(); } //immediately process queue...
}

//! put executable key codes into queue, which are the processed in main (Python) thread
void PyQueueKeyPressed(int key, int action, int mods)
//void PyQueueKeyPressed(int key, int action, int mods, std::function<bool(int, int, int)> keyPressUserFunctionInit) //call python user function
{
	EXUstd::WaitAndLockSemaphore(queuedRendererKeyListAtomicFlag); //lock queuedRendererKeyListAtomicFlag
	queuedRendererKeyList.Append(SlimArray<int, 3>({ key, action, mods }));
	EXUstd::ReleaseSemaphore(queuedRendererKeyListAtomicFlag); //clear queuedRendererKeyListAtomicFlag

	if (RendererIsSingleThreadedOrNotRunning()) { PyProcessRendererKeyQueue(); } //immediately process queue...
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
			default:
				break;
			}
		}
		//mostly catches python errors:
		catch (const pybind11::error_already_set& ex)
		{
			PyWarning("Error when executing process " + ProcessID::GetTypeString(processID) + +"':\n" + STDstring(ex.what()) + "\n; maybe a module is missing!");
			deactivateGlobalPyRuntimeErrorFlag = false;
			throw; //avoid multiple exceptions trown again (don't know why!)!
		}
		catch (const EXUexception& ex)
		{
			//EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag); //clear 
			PyWarning("Error when executing process " + ProcessID::GetTypeString(processID) +
				":\n" + STDstring(ex.what()) + "\n; maybe a module is missing!!");
			deactivateGlobalPyRuntimeErrorFlag = false;
			throw; //avoid multiple exceptions trown again (don't know why!)!
			//throw(ex); //avoid multiple exceptions trown again (don't know why!)!
		}
		catch (...) //any other exception
		{
			//EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag); //clear 
			PyWarning("Error when executing process " + ProcessID::GetTypeString(processID) + "\nmaybe a module is missing and check your python code!!");
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
			throw; //avoid multiple exceptions trown again (don't know why!)!
			//throw(ex); //avoid multiple exceptions trown again (don't know why!)!
		}
		catch (const EXUexception& ex)
		{
			//EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag); //clear 
			PyWarning("Error when executing '" + STDstring(execStr) + "':\n" + STDstring(ex.what()) + "\n; maybe a module is missing!!");
			deactivateGlobalPyRuntimeErrorFlag = false;
			throw; //avoid multiple exceptions trown again (don't know why!)!
			//throw(ex); //avoid multiple exceptions trown again (don't know why!)!
		}
		catch (...) //any other exception
		{
			//EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag); //clear 
			PyWarning("Error when executing '" + STDstring(execStr) + "'\nmaybe a module is missing and check your python code!!");
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
		if (GetGlfwRenderer().WindowIsInitialized()) //otherwise makes no sense ...! ==> ignore
		{
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
						bool rv = localKeyPressUserFunction(key[0], key[1], key[2]);
						//rv not used right now, because it is received at a time where it is too late for graphics
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
	//open window to execute a python command ... 
	std::string str = R"(
import exudyn
import numpy as np
import exudyn.GUI
try:
    if 'currentRendererSystemContainer' not in exudyn.sys: 
        print('ERROR: problems with SystemContainer, probably not attached yet to renderer')
    else:
        guiSC = exudyn.sys['currentRendererSystemContainer']
        if guiSC != 0: #this would mean that renderer is detached
            vis=guiSC.visualizationSettings.GetDictionaryWithTypeInfo()
            guiSC.visualizationSettings.SetDictionary(exudyn.GUI.EditDictionaryWithTypeInfo(vis, exu, 'Visualization Settings'))
except:
    print("edit dialog for visualizationSettings failed")
)";
	PyProcessExecuteStringAsPython(str);
}



void PyProcessShowHelpDialog()
{
	std::string str = R"(import tkinter as tk
root = tk.Tk()
root.attributes("-topmost", True) #puts window topmost (permanent)
root.title("Help on keyboard commands and mouse")
root.lift() #window has focus
root.bind("<Escape>", lambda x: root.destroy())
root.focus_force() #window has focus
scrollW = tk.Scrollbar(root)
textW = tk.Text(root, height = 30, width = 90)
textW.focus_set()
scrollW.pack(side = tk.RIGHT, fill = tk.Y)
textW.pack(side = tk.LEFT, fill = tk.Y)
scrollW.config(command = textW.yview)
textW.config(yscrollcommand = scrollW.set)
msg = """
Mouse action:
left mouse button     ... hold and drag: move model
left mouse button     ... click: select item
right mouse button    ... hold and drag: rotate model
right mouse button    ... click: open edit dialog (if activated in visualizationSettings)
mouse wheel           ... zoom
======================
Key(s) action:
1,2,3,4 or 5          ... visualization update speed (0.02, 0.1=default, 0.5, 2, 
                          100 seconds)
'.' or KEYPAD '+'     ... zoom in
',' or KEYPAD '-'     ... zoom out
CTRL+1                ... set view to 1/2-plane
SHIFT+CTRL+1          ... set view to 1/2-plane (viewed from behind)
CTRL+2                ... set view to 1/3-plane
SHIFT+CTRL+2          ... set view to 1/3-plane (viewed from behind)
CTRL+3,4,5,6          ... other views (with optional SHIFT key)
CURSOR UP, DOWN, etc. ... move scene (use CTRL for small movements)
KEYPAD 2/8,4/6,1/9    ... rotate scene about 1,2 or 3-axis (use CTRL for small rotations)
F2                    ... ignore all keyboard input, except for KeyPress user function, 
                          F2 and escape keys
F3                    ... show mouse coordinates
A      ... zoom all
C      ... show/hide connectors
CTRL+C ... show/hide connector numbers
B      ... show/hide bodies
CTRL+B ... show/hide body numbers
L      ... show/hide loads
CTRL+L ... show/hide load numbers
M      ... show/hide markers
CTRL+M ... show/hide marker numbers
N      ... show/hide nodes
CTRL+N ... show/hide node numbers
S      ... show/hide sensors
CTRL+S ... show/hide sensor numbers
T      ... switch between faces transparent/ faces transparent + edges / only face edges / full faces with edges / only faces
Q      ... stop simulation
X      ... execute command; dialog may appear behind the visualization window! may crash!
V      ... visualization settings; dialog may appear behind the visualization window!
ESCAPE ... close render window
SPACE ... continue simulation
"""
textW.insert(tk.END, msg)
tk.mainloop()
)";
	PyProcessExecuteStringAsPython(str);

}

void PyProcessShowPythonCommandDialog()
{
	//open window to execute a python command ... 
	//trys to catch errors made by user in this window
	//std::string str =
	std::string str = R"(
import exudyn
import tkinter as tk
from tkinter.scrolledtext import ScrolledText

commandString = ''
commandSet = False
singleCommandMainwin = tk.Tk()
singleCommandMainwin.focus_force() #window has focus
#singleCommandMainwin.lift() #brings it to front of other
singleCommandMainwin.attributes("-topmost", True) #puts window topmost (permanent)
#singleCommandMainwin.attributes("-topmost", False)#keeps window topmost, but not permanent
singleCommandMainwin.bind("<Escape>", lambda x: singleCommandMainwin.destroy())

def OnSingleCommandReturn(event): #set command string, but do not execute
    commandString = singleCommandEntry.get()
    print(commandString) #printout the command
    #exec(singleCommandEntry.get(), globals()) #OLD version, does not print return value!
    try:
        exec(f"""locals()['tempEXUDYNexecute'] = {commandString}""", globals(), locals())
        if locals()['tempEXUDYNexecute']!=None:
            print(locals()['tempEXUDYNexecute'])
        singleCommandMainwin.destroy()
    except:
        print("Execution of command failed. check your code!")

tk.Label(singleCommandMainwin, text="Single command (press return to execute):", justify=tk.LEFT).grid(row=0, column=0)
singleCommandEntry = tk.Entry(singleCommandMainwin, width=70);
singleCommandEntry.grid(row=1, column=0)
singleCommandEntry.bind('<Return>',OnSingleCommandReturn)
singleCommandMainwin.mainloop()
)";
	PyProcessExecuteStringAsPython(str);
}

void PyProcessShowRightMouseSelectionDialog(Index itemID)
{
#ifdef USE_GLFW_GRAPHICS //only works with renderer active
	GetGlfwRenderer().PySetRendererSelectionDict(itemID);
	STDstring strName = "edit item";
	STDstring str = "import exudyn\n";
	str += "import numpy as np\n";
	str += "import exudyn.GUI\n";
	//str += "d=exudyn.GetInternalSelectionDict()\n";
	str += "d=exudyn.sys['currentRendererSelectionDict']\n";
	str += "try:\n";
	str += "    strName = 'properties of <' + d['name'] + '>'\n";
	str += "    exudyn.GUI.EditDictionary(d,False,dialogName=strName)\n";
	str += "except:\n";
	str += "    print('showing of dictionary failed')\n";
	PyProcessExecuteStringAsPython(str);
#endif // USE_GLFW_GRAPHICS

}


void PyProcessExecuteStringAsPython(const STDstring& str)
{
	py::object scope = py::module::import("__main__").attr("__dict__"); //use this to enable access to mbs and other variables of global scope within test models suite
	PySetRendererCallbackLock(true);
	py::exec(str.c_str(), scope);
	PySetRendererCallbackLock(false);
}

//! check if renderer is single-threaded or not running
bool RendererIsSingleThreadedOrNotRunning()
{
#ifdef USE_GLFW_GRAPHICS //only works with renderer active
	if (!GetGlfwRenderer().UseMultiThreadedRendering() || !GetGlfwRenderer().WindowIsInitialized())
	{
		return true;
	}
	return false;
#else
	return true; //in case that no GLFW used, renderer behaves always like single-threaded
#endif // USE_GLFW_GRAPHICS
}

//! perform idle tasksfor single-threaded renderer
void RendererDoSingleThreadedIdleTasks()
{
#ifdef USE_GLFW_GRAPHICS //only works with renderer active
	if (GetGlfwRenderer().WindowIsInitialized() && !GetGlfwRenderer().UseMultiThreadedRendering())
	{
		GetGlfwRenderer().DoRendererIdleTasks(0);
	}
#endif // USE_GLFW_GRAPHICS
}
