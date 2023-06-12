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
    std::string str = R"(
import exudyn
import numpy as np
try:
    import exudyn.GUI #this may also fail because of tkinter
    try:
        guiSC = exudyn.GUI.GetRendererSystemContainer()
        if guiSC == None:
            print('ERROR: problems with SystemContainer, probably not attached yet to renderer')
        else:
            exudyn.GUI.EditDictionaryWithTypeInfo(guiSC.visualizationSettings, exudyn, 'Visualization Settings') 
    except Exception as exceptionVariable:
        print("edit dialog for visualizationSettings failed")
        print(exceptionVariable) #not necessary, but can help to identify reason
except:
    print("visualizationSettings dialog failed: cannot import exudyn.GUI / tkinter; tkinter probably missing")

)";
    PyProcessExecuteStringAsPython(str, !PyGetRendererMultiThreadedDialogs(), true);
#endif // USE_GLFW_GRAPHICS
}



void PyProcessShowHelpDialog()
{
#ifdef USE_GLFW_GRAPHICS

    float alphaTransparency = GetGlfwRenderer().GetVisualizationSettings()->dialogs.alphaTransparency;
    std::string str = R"(
import tkinter as tk
import exudyn
from exudyn.GUI import GetTkRootAndNewWindow

[root, tkWindow, tkRuns] = GetTkRootAndNewWindow()

)";
    if (GetGlfwRenderer().GetVisualizationSettings()->dialogs.alwaysTopmost)
    {
        str += "tkWindow.attributes('-topmost', True) #puts window topmost (permanent)\n";
    }
    if (alphaTransparency < 1.f)
    {
        str += "tkWindow.attributes('-alpha'," + EXUstd::ToString(alphaTransparency) + ") #transparency\n";
    }
    str += R"(
tkWindow.title("Help on keyboard commands and mouse")
tkWindow.lift() #window has focus
tkWindow.bind("<Escape>", lambda x: tkWindow.destroy())
tkWindow.focus_force() #window has focus
scrollW = tk.Scrollbar(tkWindow)
#resize grid columns/rows if window is resized:
tkWindow.grid_columnconfigure(0, weight=1)
tkWindow.grid_rowconfigure(0, weight=1)

textW = tk.Text(tkWindow, height = 30, width = 90, background='gray98')
textW.focus_set()
textW.grid(row=0, column=0, padx=10, pady=10, sticky=tk.NSEW)
scrollW.grid(row=0, column=1, pady=10, sticky=tk.NSEW)
scrollW.config(command = textW.yview)
textW.config(yscrollcommand = scrollW.set)
msg = """Mouse action:
left mouse button     ... hold and drag: move model
left mouse button     ... click: select item (deactivated if mouse coordinates shown)
right mouse button    ... hold and drag: rotate model
right mouse button    ... click: open edit dialog (if activated in visualizationSettings)
mouse wheel           ... zoom
======================
Key(s) action:
1,2,3,4 or 5          ... visualization update speed (0.02, 0.1=default, 0.5, 2, 
                          100 seconds)
'.' or KEYPAD '+'     ... zoom in (with optional CTRL key for small zoom)
',' or KEYPAD '-'     ... zoom out (with optional CTRL key for small zoom)
CTRL+1                ... set view to 1/2-plane
SHIFT+CTRL+1          ... set view to 1/2-plane (viewed from behind)
CTRL+2                ... set view to 1/3-plane
SHIFT+CTRL+2          ... set view to 1/3-plane (viewed from behind)
CTRL+3,4,5,6          ... other views (with optional SHIFT key)
CURSOR UP, DOWN, etc. ... move scene (use CTRL for small movements, 
                          SHIFT for rotations (ALT for z-axis))
KEYPAD 2/8,4/6,1/9    ... rotate scene about 1,2 or 3-axis (use CTRL for small rotations)
F2                    ... ignore all keyboard input, except for KeyPress user function, 
                          F2 and escape keys
F3                    ... show mouse coordinates
Q      ... stop current solver and proceed to next simulation (or end of file); 
           after window.reallyQuitTimeLimit (default:900) seconds a safety dialog opens
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
O      ... change center of rotation to current center of the window (affects only 
           current plane coordinates; rotate model to ajust other coordinates)
T      ... switch between faces transparent/ faces transparent + edges / 
           only face edges / full faces with edges / only faces
X      ... execute command; dialog may appear in background! may crash simulation!
V      ... visualization settings; dialog may appear behind the visualization window!
ESCAPE ... close render window and stop all simulations (same as close window button); 
           after window.reallyQuitTimeLimit seconds a dialog opens for safety
SPACE  ... continue simulation
"""
textW.insert(tk.END, msg)
textW.configure(state='disabled') #unable to edit
if tkRuns:
    root.wait_window(tkWindow)
else:
    tk.mainloop()
)";
    PyProcessExecuteStringAsPython(str, !PyGetRendererMultiThreadedDialogs(), true);
#endif // USE_GLFW_GRAPHICS

}

void PyProcessShowPythonCommandDialog()
{
#ifdef USE_GLFW_GRAPHICS

    //open window to execute a python command ... 
    float alphaTransparency = GetGlfwRenderer().GetVisualizationSettings()->dialogs.alphaTransparency;
    std::string str = R"(
import exudyn
import tkinter as tk
import traceback #for exception printing
from tkinter import ttk
from tkinter import scrolledtext
from exudyn.GUI import GetTkRootAndNewWindow

[root, tkWindow, tkRuns] = GetTkRootAndNewWindow()
commandString = ''
tkWindow.title("Exudyn command window")
)";
    if (GetGlfwRenderer().GetVisualizationSettings()->dialogs.alwaysTopmost)
    {
        str += "tkWindow.attributes('-topmost', True) #puts window topmost (permanent)\n";
    }
    if (alphaTransparency < 1.f)
    {
        str += "tkWindow.attributes('-alpha'," + EXUstd::ToString(alphaTransparency) + ") #transparency\n";
    }
    str += R"(
#resize grid columns/rows if window is resized:
tkWindow.grid_columnconfigure(0, weight=1)
tkWindow.grid_rowconfigure(1, weight=1)
#tkWindow.grid_rowconfigure(3, weight=1)

description ='Enter Python command which operates in global scope of you Python model;\n'
description+='Evaluate or CHANGE your current model (parameters) during simulation;\n'
description+='Press CRTL+RETURN to execute, escape to close:'

label = tk.Label(tkWindow, text=description, justify=tk.LEFT, 
                 relief=tk.SUNKEN, background='gray94')
label.grid(row=0, column=0, padx=15, pady=(15,0), sticky='W')

text_area = scrolledtext.ScrolledText(root, wrap=tk.WORD,
                                      width=60, height=8,
                                      #font=("Times New Roman", 15)
                                      )
#configure tab size:
font = tk.font.Font(font=text_area['font'])
tab_size = font.measure(' '*4) #in pixels
text_area.config(tabs=tab_size)

#++++++++++++++++++++++++++++++++
#read command string and execute
globs=None
def OnRunCode(event): 
    global text_area
    global globs

    commandString = text_area.get('1.0', tk.END)
    print('command window execute:\n',commandString.strip(),sep='') #printout the command
    print('output:')

    if commandString.strip() == '': #empty command causes exception
        return
    commandString = commandString.replace('\t',' '*4) #tabs may cause problems

    try:
        exec(commandString, globals(), locals())
        #old version: for single line, it prints out the result
        # exec(f"""locals()['tempEXUDYNexecute'] = {commandString}""", globals(), locals())
        # if locals()['tempEXUDYNexecute']!=None:
        #     print(locals()['tempEXUDYNexecute'])
    # except:
    except:
        print("Execution of command failed; error:")
        #traceback.print_exc()
        globs = traceback.format_exc()
        lines = globs.split('\n')
        for s in lines:
            s = s.replace('  File "<string>", ','')
            if (('File "' not in s) and 
                ('exec(commandString, globals(), locals())' not in s) ):
                print(s)
    if event!=None:
        return "break" #prevent from passing Return key to text ...

#run code
def OnClose(event):
    global tkWindow
    tkWindow.destroy() 

#++++++++++++++++++++++++++++++++
text_area.grid(row=1, column=0, pady=15,padx=10,sticky=tk.NSEW)
text_area.bind('<Control-Return>',OnRunCode)
text_area.bind('<Escape>',OnClose)
tkWindow.bind('<Escape>',OnClose)

#++++++++++++++++++++++++++++++++
frame = tk.Frame(tkWindow)
runButton = tk.Button(frame, text = "    Run code    ", command = lambda: OnRunCode(None))
closeButton = tk.Button(frame, text = "    Close    ", command = lambda: OnClose(None))

frame.grid(row=2, column=0, padx=15, pady=(0,15), sticky='', columnspan=3)
runButton.grid(row=0, column=0, padx=80, sticky='')
closeButton.grid(row=0, column=1, padx=80, sticky='')

#show some examples:
examples = 'helpful examples:\n'
examples+= 'show overall info of mbs:\n'
examples+= 'print(mbs)\n'
examples+= '#change current dynamic solver end time:\n'
examples+= "mbs.sys['dynamicSolver'].it.endTime=10 \n"
examples+= '#change verbose mode of dynamic solver:\n'
examples+= "mbs.sys['dynamicSolver'].output.verboseMode=1\n"
examples+= '#stop file writing:\n'
examples+= "mbs.sys['dynamicSolver'].output.writeToSolutionFile=False\n"
examples+= '#print values of sensor 0:\n'
examples+= "print(mbs.GetSensorValues(0))\n"
examples+= '#pause after each step:\n'
examples+= "simulationSettings.pauseAfterEachStep=True\n"
examples+= '\n#==>BUT changing simulationSettings is dangerous!'

textExample = scrolledtext.ScrolledText(root, wrap=tk.WORD,
                                      width=60, height=examples.count('\n')+1,
                                      background='gray94',
                                      )
textExample.grid(row=3, column=0, padx=15, pady=(0,15), sticky=tk.NSEW)
textExample.insert(tk.END, examples)
textExample.configure(state='disabled') #unable to edit

# placing cursor in text area
text_area.focus_set()
tkWindow.focus_force() #window has focus

if tkRuns:
    root.wait_window(tkWindow)
else:
    tk.mainloop()
)";
    PyProcessExecuteStringAsPython(str, !PyGetRendererMultiThreadedDialogs(), true);
#endif // USE_GLFW_GRAPHICS
}

//OLD, single line:
//void PyProcessShowPythonCommandDialog()
//{
//#ifdef USE_GLFW_GRAPHICS
//
//    //open window to execute a python command ... 
//    float alphaTransparency = GetGlfwRenderer().GetVisualizationSettings()->dialogs.alphaTransparency;
//    std::string str = R"(
//import exudyn
//import tkinter as tk
//from exudyn.GUI import GetTkRootAndNewWindow
//
//[root, tkWindow, tkRuns] = GetTkRootAndNewWindow()
//commandString = ''
//commandSet = False
//tkWindow.focus_force() #window has focus
//tkWindow.title("Exudyn command window")
//)";
//    if (GetGlfwRenderer().GetVisualizationSettings()->dialogs.alwaysTopmost)
//    {
//        str += "tkWindow.attributes('-topmost', True) #puts window topmost (permanent)\n";
//    }
//    if (alphaTransparency < 1.f)
//    {
//        str += "tkWindow.attributes('-alpha'," + EXUstd::ToString(alphaTransparency) + ") #transparency\n";
//    }
//    str += R"(
//tkWindow.bind("<Escape>", lambda x: tkWindow.destroy())
//
//def OnSingleCommandReturn(event): #set command string, but do not execute
//    commandString = singleCommandEntry.get()
//    print(commandString) #printout the command
//    #exec(singleCommandEntry.get(), globals()) #OLD version, does not print return value!
//    try:
//        exec(f"""locals()['tempEXUDYNexecute'] = {commandString}""", globals(), locals())
//        if locals()['tempEXUDYNexecute']!=None:
//            print(locals()['tempEXUDYNexecute'])
//        tkWindow.destroy()
//    except:
//        print("Execution of command failed. check your code!")
//
//label = tk.Label(tkWindow, text="Enter Python command which operates in global scope of you Python model]\nEvaluate you current model or change parameters\npress return to execute:", justify=tk.LEFT) #.grid(row=0, column=0)
//label.pack(pady=10,padx=(10,40))
//singleCommandEntry = tk.Entry(tkWindow, width=70);
//#singleCommandEntry.grid(row=1, column=0)
//singleCommandEntry.bind('<Return>',OnSingleCommandReturn)
//singleCommandEntry.pack(pady=15,padx=20)
//
//if tkRuns:
//    root.wait_window(tkWindow)
//else:
//    tk.mainloop()
//)";
//    PyProcessExecuteStringAsPython(str, !PyGetRendererMultiThreadedDialogs(), true);
//#endif // USE_GLFW_GRAPHICS
//}
//
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
    PyProcessExecuteStringAsPython(str, !PyGetRendererMultiThreadedDialogs(), true);
#endif // USE_GLFW_GRAPHICS

}

void PyProcessAskQuit()
{
#ifdef USE_GLFW_GRAPHICS
    PyProcessSetResult(1);

    try
    {
        //open window to execute a python command ... 
        float alphaTransparency = GetGlfwRenderer().GetVisualizationSettings()->dialogs.alphaTransparency;
        PyWriteToSysDictionary("quitResponse", py::cast((int)1) );

        std::string str = R"(
try:
    import exudyn
    import tkinter as tk
    from exudyn.GUI import GetTkRootAndNewWindow

    response = False #if user just shuts window

    [root, tkWindow, tkRuns] = GetTkRootAndNewWindow()
    tkWindow.attributes('-topmost', True) #puts window topmost(permanent)\n";
    tkWindow.bind("<Escape>", lambda x : tkWindow.destroy())
    tkWindow.title("WARNING - long running simulation!")

    def QuitResponse(clickResponse) :
        global tkWindow
        global response
        response = clickResponse
        tkWindow.destroy()

    label = tk.Label(tkWindow, text = "Do you really want to stop simulation and close renderer?", justify = tk.LEFT)
    yes_button = tk.Button(tkWindow, text = "        Yes        ", command = lambda: QuitResponse(True))
    no_button = tk.Button(tkWindow, text = "        No        ", command = lambda: QuitResponse(False))

    label.grid(row=0, column=0, pady=(20,0),padx=50,columnspan=5)
    yes_button.grid(row=1, column=1, pady=20)
    no_button.grid(row=1, column=3, pady=20)

    tkWindow.focus_force() #window has focus

    if tkRuns:
        root.wait_window(tkWindow)
    else:
        tk.mainloop()

    #response ready
    exudyn.sys['quitResponse'] = response+2 #2=do not quit, 3=quit
except:
    pass #if fails, user shall not be notified
)";
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
void RendererDoSingleThreadedIdleTasks()
{
#ifdef USE_GLFW_GRAPHICS //only works with renderer active
	if (GetGlfwRenderer().IsGlfwInitAndRendererActive() && !GetGlfwRenderer().UseMultiThreadedRendering())
	{
		GetGlfwRenderer().DoRendererIdleTasks(0);
	}
#endif // USE_GLFW_GRAPHICS
}
