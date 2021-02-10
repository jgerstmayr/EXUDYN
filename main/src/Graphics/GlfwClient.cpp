/** ***********************************************************************************************
* @brief        Implementation of GlfwClient
*
* @author       Gerstmayr Johannes
* @date         2019-05-24 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Utilities/SlimArray.h"
//#include <string>

//void testtest()
//{
//	std::cout << "test";
//}
//
#ifdef USE_GLFW_GRAPHICS
using namespace std::string_literals; // enables s-suffix for std::string literals

#include <ostream>
//#include <stdlib.h> //only works in MSVC for initialization with std::vector
#include <array>
#include <vector>
#include <stdio.h>
#include <fstream> //for image save to file

#include <chrono> //sleep_for()
//#pragma comment(lib, "opengl32")
//#pragma comment(lib, "glu32")
//#include <gl/gl.h>
//#include <gl/glu.h>
////#define GLFW_INCLUDE_ES3 //open gl ES version
//#define GLFW_INCLUDE_GLEXT
//#include <GLFW/glfw3.h>


#include "Graphics/characterBitmap.h"
#include "Graphics/GlfwClient.h"
#include "System/versionCpp.h"


extern bool globalPyRuntimeErrorFlag; //stored in Stdoutput.cpp; this flag is set true as soon as a PyError or SysError is raised; this causes to shut down secondary processes, such as graphics, etc.
//extern bool deactivateGlobalPyRuntimeErrorFlag; //stored in Stdoutput.cpp; this flag is set true as soon as functions are called e.g. from command windows, which allow errors without shutting down the renderer
#define rendererOut std::cout //defines the type of output for renderer: pout could be problematic because of parallel threads; std::cout does not work in Spyder


GlfwRenderer glfwRenderer;

//++++++++++++++++++++++++++++++++++++++++++
//define static variables:
bool GlfwRenderer::rendererActive = false;
bool GlfwRenderer::stopRenderer = false;
Index GlfwRenderer::rendererError = 0;
GLFWwindow* GlfwRenderer::window = nullptr;
RenderState* GlfwRenderer::state;
RenderStateMachine GlfwRenderer::stateMachine;
std::thread GlfwRenderer::rendererThread;
//uint64_t GlfwRenderer::visualizationCounter = 0;
Index GlfwRenderer::firstRun = 0; //zoom all in first run
std::atomic_flag GlfwRenderer::renderFunctionRunning = ATOMIC_FLAG_INIT;  //!< semaphore to check if Render(...)  function is currently running (prevent from calling twice); initialized with clear state

BitmapFont GlfwRenderer::bitmapFont;				//!< bitmap font for regular texts, initialized upon start of renderer
float GlfwRenderer::fontScale;						//!< monitor scaling factor from windows, to scale fonts
#ifndef USE_TEXTURED_BITMAP_FONTS
BitmapFont GlfwRenderer::bitmapFontSmall;			//!< bitmap font for small texts, initialized upon start of renderer
BitmapFont GlfwRenderer::bitmapFontLarge;			//!< bitmap font for large texts, initialized upon start of renderer
BitmapFont GlfwRenderer::bitmapFontHuge;			//!< bitmap font for huge texts, initialized upon start of renderer
#else
GLuint GlfwRenderer::textureNumberRGBbitmap[256];	//!< store texture number for our bitmap font
GLuint GlfwRenderer::bitmapFontListBase;			//!< starting index for GLlists for font bitmap textured quads
ResizableArray<GLubyte> GlfwRenderer::charBuffer;	//!< buffer for converstion of UTF8 into internal unicode-like format
#endif

ResizableArray<GraphicsData*>* GlfwRenderer::graphicsDataList = nullptr;
//GraphicsData* GlfwRenderer::data = nullptr;
VisualizationSettings* GlfwRenderer::visSettings = nullptr;
VisualizationSystemContainerBase* GlfwRenderer::basicVisualizationSystemContainer = nullptr;
//++++++++++++++++++++++++++++++++++++++++++


GlfwRenderer::GlfwRenderer()
{
	rendererActive = false;
	graphicsDataList = nullptr;
	window = nullptr;

	stateMachine.leftMousePressed = false;
	stateMachine.rightMousePressed = false;
	stateMachine.shiftPressed = false;
	stateMachine.ctrlPressed = false;
	stateMachine.mode = RendererMode::_None;			//!< determines the state of any action

	stateMachine.mousePositionX = 0;	//!< last mouse position used for move and zoom
	stateMachine.mousePositionY = 0;	//!< last mouse position used for move and zoom
	stateMachine.lastMousePressedX = 0;	//!< last left mouse button position pressed
	stateMachine.lastMousePressedY = 0;

	fontScale = 1; //initialized if needed before bitmap initialization
	//renderState state cannot be initialized here, because it will be linked later to visualizationSystemContainer
};

void GlfwRenderer::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
	{
		basicVisualizationSystemContainer->StopSimulation();
		
		//this leads to problems when closing:
		//std::this_thread::sleep_for(std::chrono::milliseconds(200)); //give thread time to finish the stop simulation command

		glfwSetWindowShouldClose(window, GL_TRUE);

		return; //don't process keys or call user function
	}

	//switch ignore keys functionality
	if (key == GLFW_KEY_F2 && action == GLFW_PRESS)
	{
		visSettings->window.ignoreKeys = !visSettings->window.ignoreKeys;
		rendererOut << "ignore keys mode switched to " << visSettings->window.ignoreKeys << "\n";
	}

	//do this first, as key may still have time to complete action
	if (visSettings->window.ignoreKeys || !(key == GLFW_KEY_Q && action == GLFW_PRESS && mods == 0))
	{
		PyQueueKeyPressed(key, action, mods, visSettings->window.keyPressUserFunction); //call python user function
	}

	//+++++++++++++++++++++++++++++++++++++++++++++
	//check if regular keys are ignored:
	if (!visSettings->window.ignoreKeys)
	{
		//keycode to quit simulation:
		if (key == GLFW_KEY_Q && action == GLFW_PRESS && mods == 0)
		{
			basicVisualizationSystemContainer->StopSimulation();
		}

		//keycode to continue paused simulation:
		if ((key == GLFW_KEY_SPACE && action == GLFW_PRESS && mods == 0) ||
			(key == GLFW_KEY_SPACE && action == GLFW_REPEAT)) //changed shift to repeat 
			//(key == GLFW_KEY_SPACE && mods == GLFW_MOD_SHIFT))
		{
			basicVisualizationSystemContainer->ContinueSimulation();
		}

		//switch ignore keys functionality
		if (key == GLFW_KEY_F3 && action == GLFW_PRESS)
		{
			visSettings->window.showMouseCoordinates = !visSettings->window.showMouseCoordinates;
		}

		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		//visualization update keys:
		if (key == GLFW_KEY_1 && action == GLFW_PRESS && mods == 0)
		{
			visSettings->general.graphicsUpdateInterval = 0.02f;
			rendererOut << "Visualization update: 20ms\n";
		}

		if (key == GLFW_KEY_2 && action == GLFW_PRESS && mods == 0)
		{
			visSettings->general.graphicsUpdateInterval = 0.2f;
			rendererOut << "Visualization update: 200ms\n";
		}

		if (key == GLFW_KEY_3 && action == GLFW_PRESS && mods == 0)
		{
			visSettings->general.graphicsUpdateInterval = 1.f;
			rendererOut << "Visualization update: 1s\n";
		}

		if (key == GLFW_KEY_4 && action == GLFW_PRESS && mods == 0)
		{
			visSettings->general.graphicsUpdateInterval = 10.f;
			rendererOut << "Visualization update: 10s\n";
		}

		if (key == GLFW_KEY_5 && action == GLFW_PRESS && mods == 0)
		{
			visSettings->general.graphicsUpdateInterval = 100.f;
			rendererOut << "Visualization update: 100s\n";
		}

		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
		//process keys for showing nodes, bodies, ...
		if (key == GLFW_KEY_N && action == GLFW_PRESS && mods != GLFW_MOD_CONTROL)
		{
			visSettings->nodes.show = !visSettings->nodes.show; UpdateGraphicsDataNow();
			rendererOut << "show nodes: " << visSettings->nodes.show << "\n";
		}
		if (key == GLFW_KEY_B && action == GLFW_PRESS && mods != GLFW_MOD_CONTROL)
		{
			visSettings->bodies.show = !visSettings->bodies.show; UpdateGraphicsDataNow();
			rendererOut << "show bodies: " << visSettings->bodies.show << "\n";
		}
		if (key == GLFW_KEY_C && action == GLFW_PRESS && mods != GLFW_MOD_CONTROL)
		{
			visSettings->connectors.show = !visSettings->connectors.show; UpdateGraphicsDataNow();
			rendererOut << "show connectors: " << visSettings->connectors.show << "\n";
		}
		if (key == GLFW_KEY_M && action == GLFW_PRESS && mods != GLFW_MOD_CONTROL)
		{
			visSettings->markers.show = !visSettings->markers.show; UpdateGraphicsDataNow();
			rendererOut << "show markers: " << visSettings->markers.show << "\n";
		}
		if (key == GLFW_KEY_L && action == GLFW_PRESS && mods != GLFW_MOD_CONTROL)
		{
			visSettings->loads.show = !visSettings->loads.show; UpdateGraphicsDataNow();
			rendererOut << "show loads: " << visSettings->loads.show << "\n";
		}
		if (key == GLFW_KEY_S && action == GLFW_PRESS && mods != GLFW_MOD_CONTROL)
		{
			visSettings->sensors.show = !visSettings->sensors.show; UpdateGraphicsDataNow();
			rendererOut << "show sensors: " << visSettings->sensors.show << "\n";
		}
		//show node, object, ... numbers:
		if (key == GLFW_KEY_N && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			visSettings->nodes.showNumbers = !visSettings->nodes.showNumbers; UpdateGraphicsDataNow();
			rendererOut << "show node numbers: " << visSettings->nodes.showNumbers << "\n";
		}
		if (key == GLFW_KEY_B && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			visSettings->bodies.showNumbers = !visSettings->bodies.showNumbers; UpdateGraphicsDataNow();
			rendererOut << "show body numbers: " << visSettings->bodies.showNumbers << "\n";
		}
		if (key == GLFW_KEY_C && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			visSettings->connectors.showNumbers = !visSettings->connectors.showNumbers; UpdateGraphicsDataNow();
			rendererOut << "show connector numbers: " << visSettings->connectors.showNumbers << "\n";
		}
		if (key == GLFW_KEY_M && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			visSettings->markers.showNumbers = !visSettings->markers.showNumbers; UpdateGraphicsDataNow();
			rendererOut << "show marker numbers: " << visSettings->markers.showNumbers << "\n";
		}
		if (key == GLFW_KEY_L && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			visSettings->loads.showNumbers = !visSettings->loads.showNumbers; UpdateGraphicsDataNow();
			rendererOut << "show load numbers: " << visSettings->loads.showNumbers << "\n";
		}
		if (key == GLFW_KEY_S && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			visSettings->sensors.showNumbers = !visSettings->sensors.showNumbers; UpdateGraphicsDataNow();
			rendererOut << "show sensor numbers: " << visSettings->sensors.showNumbers << "\n";
		}
		if (key == GLFW_KEY_T && action == GLFW_PRESS && mods != GLFW_MOD_CONTROL)
		{
			visSettings->openGL.facesTransparent = !visSettings->openGL.facesTransparent; UpdateGraphicsDataNow();
			rendererOut << "all faces transparent: " << visSettings->openGL.facesTransparent << "\n";
		}
		if (key == GLFW_KEY_X && action == GLFW_PRESS)
		{
			//open window to execute a python command ... 
			//trys to catch errors made by user in this window
			//std::string str =
			std::string str = R"(
import tkinter as tk
from tkinter.scrolledtext import ScrolledText

commandString = ''
commandSet = False
singleCommandMainwin = tk.Tk()
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
			PyQueueExecutableString(str);
			UpdateGraphicsDataNow();
		}
		//visualization settings dialog
		if (key == GLFW_KEY_V && action == GLFW_PRESS)
		{
			//open window to execute a python command ... 
			std::string str = R"(
import exudyn.GUI
vis=SC.visualizationSettings.GetDictionaryWithTypeInfo()
#if 'keyPressUserFunction' in vis['window']: #removed from Get/SetDictionary(...)
#    del vis['window']['keyPressUserFunction'] #not possible to edit
SC.visualizationSettings.SetDictionary(exudyn.GUI.EditDictionaryWithTypeInfo(vis, exu, 'Visualization Settings'))
)";
			PyQueueExecutableString(str);
			UpdateGraphicsDataNow();
		}
		//help key
		if (key == GLFW_KEY_H && action == GLFW_PRESS)
		{
			//open window to execute a python command ... (THREADSAFE???)
			std::string str = R"(import tkinter as tk
root = tk.Tk()
root.title("Help on keyboard commands and mouse")
scrollW = tk.Scrollbar(root)
textW = tk.Text(root, height = 30, width = 90)
scrollW.pack(side = tk.RIGHT, fill = tk.Y)
textW.pack(side = tk.LEFT, fill = tk.Y)
scrollW.config(command = textW.yview)
textW.config(yscrollcommand = scrollW.set)
msg = """
Mouse action:
left mouse button     ... move model
right mouse button    ... rotate model
mouse wheel           ... zoom
======================
Key(s) action:
1,2,3,4 or 5          ... visualization update speed
'.' or KEYPAD '+'     ... zoom in
',' or KEYPAD '-'     ... zoom out
CTRL+1                ... set view to 1/2-plane
SHIFT+CTRL+1          ... set view to 1/2-plane (viewed from behind)
CTRL+2                ... set view to 1/3-plane
SHIFT+CTRL+2          ... set view to 1/3-plane (viewed from behind)
CTRL+3,4,5,6          ... other views (with optional SHIFT key)
CURSOR UP, DOWN, etc. ... move scene (use CTRL for small movements)
KEYPAD 2/8,4/6,1/9    ... rotate scene about 1,2 or 3-axis (use CTRL for small rotations)
F2                    ... ignore all keyboard input, except for KeyPress user function, F2 and escape
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
T      ... make all faces transparent
Q      ... stop simulation
X      ... execute command; dialog may appear behind the visualization window! may crash!
V      ... visualization settings; dialog may appear behind the visualization window!
ESCAPE ... close render window
SPACE ... continue simulation
"""
textW.insert(tk.END, msg)
tk.mainloop()
)";
			PyQueueExecutableString(str);
			UpdateGraphicsDataNow();
		}

		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
		//process keys for move, rotate, zoom
		float rotStep = visSettings->window.keypressRotationStep; //degrees
		float transStep = visSettings->window.keypressTranslationStep * state->zoom; //degrees
		if (mods == GLFW_MOD_CONTROL) //only for rotStep and transStep
		{
			rotStep *= 0.1f;
			transStep *= 0.1f;
		}

		float zoomStep = visSettings->window.zoomStepFactor;
		Float3 incRot({ 0.f,0.f,0.f });
		if (key == GLFW_KEY_KP_2 && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[0] = rotStep; }
		if (key == GLFW_KEY_KP_8 && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[0] = -rotStep; }
		if (key == GLFW_KEY_KP_4 && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[1] = rotStep; }
		if (key == GLFW_KEY_KP_6 && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[1] = -rotStep; }
		if (key == GLFW_KEY_KP_7 && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[2] = rotStep; }
		if (key == GLFW_KEY_KP_9 && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[2] = -rotStep; }

		if (incRot[0] + incRot[1] + incRot[2] != 0.f)
		{
			glMatrixMode(GL_MODELVIEW);
			//glLoadIdentity();	//start with identity
			glLoadMatrixf(state->modelRotation.GetDataPointer()); //load previous rotation
			glRotatef(incRot[0], 1.f, 0.f, 0.f); //apply "incremental" rotation around x
			glRotatef(incRot[1], 0.f, 1.f, 0.f); //apply "incremental" rotation around y
			glRotatef(incRot[2], 0.f, 0.f, 1.f); //apply "incremental" rotation around z
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
		}


		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		//standard view:
		//if ((key == GLFW_KEY_KP_0 || key == GLFW_KEY_0) && action == GLFW_PRESS) //reset all rotations
		//{
		//	glMatrixMode(GL_MODELVIEW);
		//	glLoadIdentity();	//start with identity
		//	glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
		//	rendererOut << "Reset OpenGL modelview\n";
		//}

		//change view:
		if (key == GLFW_KEY_1 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			//glRotated(180, 0.0, 1.0, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			rendererOut << "View 1: 1-2-plane\n";
		}

		if (key == GLFW_KEY_2 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(-90, 1.0, 0.0, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			rendererOut << "View 2: 1-3-plane\n";
		}

		if (key == GLFW_KEY_3 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(-90, 0.0, 1.0, 0.0);
			glRotated(-90, 1.0, 0.0, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			rendererOut << "View 3: 2-3-plane\n";
		}

		if (key == GLFW_KEY_1 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(180, 0.0, 1.0, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			rendererOut << "View 1: 1-2-plane mirrored about vertical axis\n";
		}

		if (key == GLFW_KEY_2 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(-90, 1.0, 0.0, 0.0);
			glRotated(180, 0.0, 0.0, 1.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			rendererOut << "View 2: 1-3-plane mirrored about vertical axis\n";
		}

		if (key == GLFW_KEY_3 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(-90, 0.0, 1.0, 0.0);
			glRotated(-90, 1.0, 0.0, 0.0);
			glRotated(180, 0.0, 0.0, 1.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			rendererOut << "View 3: 2-3-plane mirrored about vertical axis\n";
		}

		//second group of views:
		if (key == GLFW_KEY_4 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(180, 0.0, 1.0, 0.0);
			glRotated(90, 0.0, 0.0, 1.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			rendererOut << "View 1: 2-1-plane\n";
		}

		if (key == GLFW_KEY_5 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(90, 0.0, 1.0, 0.0);
			glRotated(90, 0.0, 0.0, 1.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			rendererOut << "View 2: 3-1-plane\n";
		}

		if (key == GLFW_KEY_6 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(90, 0.0, 1.0, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			rendererOut << "View 3: 3-2-plane\n";
		}

		if (key == GLFW_KEY_4 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(90, 0.0, 0.0, 1.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			rendererOut << "View 1: 2-1-plane mirrored about vertical axis\n";
		}

		if (key == GLFW_KEY_5 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(-90, 1.0, 0.0, 0.0);
			glRotated(-90, 0.0, 1.0, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			rendererOut << "View 2: 3-1-plane mirrored about vertical axis\n";
		}

		if (key == GLFW_KEY_6 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(-90, 0.0, 1.0, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			rendererOut << "View 3: 3-2-plane mirrored about vertical axis\n";
		}

		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


		if (key == GLFW_KEY_UP && (action == GLFW_PRESS || action == GLFW_REPEAT)) { state->centerPoint[1] -= transStep; }
		if (key == GLFW_KEY_DOWN && (action == GLFW_PRESS || action == GLFW_REPEAT)) { state->centerPoint[1] += transStep; }
		if (key == GLFW_KEY_LEFT && (action == GLFW_PRESS || action == GLFW_REPEAT)) { state->centerPoint[0] += transStep; }
		if (key == GLFW_KEY_RIGHT && (action == GLFW_PRESS || action == GLFW_REPEAT)) { state->centerPoint[0] -= transStep; }

		if ((key == GLFW_KEY_KP_SUBTRACT || key == GLFW_KEY_COMMA) && (action == GLFW_PRESS || action == GLFW_REPEAT))
		{
			if (mods == GLFW_MOD_CONTROL)
			{
				state->zoom *= pow(zoomStep, 0.1f);
			} //small zoom step
			else { state->zoom *= zoomStep; }
		}
		if ((key == GLFW_KEY_KP_ADD || key == GLFW_KEY_PERIOD) && (action == GLFW_PRESS || action == GLFW_REPEAT))
		{
			if (mods == GLFW_MOD_CONTROL)
			{
				state->zoom /= pow(zoomStep, 0.1f);
			} //small zoom step
			else { state->zoom /= zoomStep;; }
		}

		if (key == GLFW_KEY_A && action == GLFW_PRESS) { ZoomAll(); UpdateGraphicsDataNow(); }
	}
}

void GlfwRenderer::ZoomAll()
{
	//pout << "zoom all\n";
	//pout << "graphicsDataList=" << graphicsDataList << "\n";
	//max scene size from current line data:
	//Float3 pmax({ -1e30f,-1e30f,-1e30f });
	//Float3 pmin({ 1e30f,1e30f,1e30f });

	Float3 pmax({ -1e30f,-1e30f,-1e30f });
	Float3 pmin({ 1e30f,1e30f,1e30f });

	if (graphicsDataList != nullptr && state != nullptr && visSettings != nullptr)
	{
		for (auto data : *graphicsDataList)
		{
			for (auto item : data->glLines)
			{
				for (Index i = 0; i < 3; i++)
				{
					pmax[i] = EXUstd::Maximum(item.point1[i], pmax[i]);
					pmin[i] = EXUstd::Minimum(item.point1[i], pmin[i]);
					pmax[i] = EXUstd::Maximum(item.point2[i], pmax[i]);
					pmin[i] = EXUstd::Minimum(item.point2[i], pmin[i]);
				}
			}
			for (auto item : data->glTexts)
			{
				for (Index i = 0; i < 3; i++)
				{
					pmax[i] = EXUstd::Maximum(item.point[i], pmax[i]);
					pmin[i] = EXUstd::Minimum(item.point[i], pmin[i]);
				}
			}
			for (auto item : data->glPoints)
			{
				for (Index i = 0; i < 3; i++)
				{
					pmax[i] = EXUstd::Maximum(item.point[i], pmax[i]);
					pmin[i] = EXUstd::Minimum(item.point[i], pmin[i]);
				}
			}
			for (auto item : data->glCirclesXY)
			{
				for (Index i = 0; i < 3; i++)
				{
					pmax[i] = EXUstd::Maximum(item.point[i] + item.radius, pmax[i]);
					pmin[i] = EXUstd::Minimum(item.point[i] - item.radius, pmin[i]);
				}
			}
			for (auto item : data->glTriangles)
			{
				for (auto point : item.points)
				{
					for (Index i = 0; i < 3; i++)
					{
						pmax[i] = EXUstd::Maximum(point[i], pmax[i]);
						pmin[i] = EXUstd::Minimum(point[i], pmin[i]);
					}
				}
			}
		}

		Float3 center = 0.5f*(pmin + pmax);

		float maxSceneSize = (pmax - pmin).GetL2Norm();
		if (maxSceneSize < visSettings->general.minSceneSize) { maxSceneSize = visSettings->general.minSceneSize; }

		if (graphicsDataList->NumberOfItems() == 0 ||
			((*graphicsDataList)[0]->glCirclesXY.NumberOfItems() == 0 && (*graphicsDataList)[0]->glLines.NumberOfItems() == 0
				&& (*graphicsDataList)[0]->glPoints.NumberOfItems() == 0 && (*graphicsDataList)[0]->glTexts.NumberOfItems() == 0
			&& (*graphicsDataList)[0]->glTriangles.NumberOfItems() == 0))
		{
			maxSceneSize = 1;
			center = Float3({ 0,0,0 });
		}

		//rendererOut << "Zoom all\n";
		//rendererOut << "maxScenesize=" << maxSceneSize << "\n";
		//rendererOut << "center=" << center << "\n";

		state->zoom = 0.4f*maxSceneSize;
		state->centerPoint = center;
		//state->zoom = 0.5f*state->maxSceneSize;
		//state->centerPoint = state->sceneCenterPoint; //computed in VisualizationSystem::UpdateMaximumSceneCoordinates

		UpdateGraphicsDataNow(); //remove from here; just put here for testing 
	}
}

void GlfwRenderer::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	//rendererOut << "scroll: x=" << xoffset << ", y=" << yoffset << "\n";
	float zoomStep = visSettings->window.zoomStepFactor;

	if (yoffset > 0) { state->zoom /= zoomStep * (float)yoffset; }
	if (yoffset < 0) { state->zoom *= zoomStep * (float)(-yoffset); }

	//rendererOut << "zoom=" << state->zoom << "\n";

}

void GlfwRenderer::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//STATE MACHINE MOUSE MOVE
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
	{
		//rendererOut << "mouse button left pressed\n";
		stateMachine.leftMousePressed = true;

		stateMachine.lastMousePressedX = stateMachine.mousePositionX;
		stateMachine.lastMousePressedY = stateMachine.mousePositionY; //now see if the mouse moves, then switch to move mode!

		state->mouseLeftPressed = true;
	}
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
	{
		stateMachine.leftMousePressed = false;
		state->mouseLeftPressed = false;
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//STATE MACHINE ROTATE
	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS && !stateMachine.leftMousePressed)
	{
		stateMachine.rightMousePressed = true;
		stateMachine.lastMousePressedX = stateMachine.mousePositionX;
		stateMachine.lastMousePressedY = stateMachine.mousePositionY; //now see if the mouse moves, then switch to move mode!
	}
	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE)
	{
		stateMachine.rightMousePressed = false;
	}

	//glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED); //unlimited cursor position (also outside of window) - might get negative coordinates
	//glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
	{
		state->mouseRightPressed = true;
	}
	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE)
	{
		state->mouseRightPressed = false;
	}

	if (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS)
	{
		state->mouseMiddlePressed = true;
	}
	if (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_RELEASE)
	{
		state->mouseMiddlePressed = false;
	}

}

void GlfwRenderer::cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
	//rendererOut << "mouse cursor: x=" << xpos << ", y=" << ypos << "\n";
	stateMachine.mousePositionX = xpos;
	stateMachine.mousePositionY = ypos;

	float height = (float)state->currentWindowSize[1];
	float factor = 2.f*state->zoom / height;

	state->mouseCoordinates = Vector2D({ xpos, ypos });
	state->openGLcoordinates = factor * Vector2D({ xpos - 0.5*state->currentWindowSize[0], -1.*(ypos - 0.5*state->currentWindowSize[1]) }) +
		Vector2D({ (double)state->centerPoint[0], (double)state->centerPoint[1] });

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//MOUSE MOVE state machine:
	//check if one should switch to mouse move mode:
	double minMove = 2;
	if (stateMachine.leftMousePressed && stateMachine.mode == RendererMode::_None)
	{
		if (fabs(stateMachine.lastMousePressedX - xpos) >= minMove || fabs(stateMachine.lastMousePressedY - ypos) >= minMove)
		{ 
			stateMachine.mode = RendererMode::Move;
			stateMachine.storedCenterPointX = state->centerPoint[0];
			stateMachine.storedCenterPointY = state->centerPoint[1];
		}
	}

	if (stateMachine.mode == RendererMode::Move)
	{
		if (stateMachine.leftMousePressed)
		{
			state->centerPoint[0] = stateMachine.storedCenterPointX - (float)(xpos - stateMachine.lastMousePressedX) * factor;
			state->centerPoint[1] = stateMachine.storedCenterPointY + (float)(ypos - stateMachine.lastMousePressedY) * factor;
		}
		else { stateMachine.mode = RendererMode::_None; } //finish move operation if button is released!
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//ROTATE state machine:
	//check if one should switch to mouse move mode:
	minMove = 2; //for rotation
	if (stateMachine.rightMousePressed && stateMachine.mode == RendererMode::_None)
	{
		if (fabs(stateMachine.lastMousePressedX - xpos) >= minMove || fabs(stateMachine.lastMousePressedY - ypos) >= minMove)
		{
			stateMachine.mode = RendererMode::Rotate;
			stateMachine.storedModelRotation = state->modelRotation; //now store the current rotation of the modelview
		}
	}

	if (stateMachine.mode == RendererMode::Rotate)
	{
		if (stateMachine.rightMousePressed)
		{
			//rotation shall be proportional to pixels
			float deltaX = (float)(xpos - stateMachine.lastMousePressedX);
			float deltaY = (float)(ypos - stateMachine.lastMousePressedY);
			float rotationFactor = visSettings->window.mouseMoveRotationFactor;

			//use OpenGL transformation to compute incremental rotation

			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotatef(deltaX*rotationFactor, 0.f, 1.f, 0.f); //apply "incremental" rotation around x
			glRotatef(deltaY*rotationFactor, 1.f, 0.f, 0.f); //apply "incremental" rotation around y
			glMultMatrixf(stateMachine.storedModelRotation.GetDataPointer()); //load previous rotation
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering

			////this mode always works locally (does not allow rotation around local z-axis!):
			//glMatrixMode(GL_MODELVIEW);
			////glLoadIdentity();	//start with identity
			//glLoadMatrixf(stateMachine.storedModelRotation.GetDataPointer()); //load previous rotation
			//glRotatef(deltaX*rotationFactor, 0.f, 1.f, 0.f); //apply "incremental" rotation around x
			//glRotatef(deltaY*rotationFactor, 1.f, 0.f, 0.f); //apply "incremental" rotation around y
			//glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering

		}
		else { stateMachine.mode = RendererMode::_None; } //finish move operation if button is released!
	}


}


bool GlfwRenderer::SetupRenderer(bool verbose)
{
	//glfwCreateThread();
	//auto th = new std::thread(GlfwRenderer::StartThread);
	globalPyRuntimeErrorFlag = false; //if previous renderer crashed, this allows to relase this error even if the old renderer is still running

	if (rendererActive)//check that renderer is not already running and that link to SystemContainer exists
	{
		PyWarning("OpenGL renderer already active");
		return false;
	}
	else if (basicVisualizationSystemContainer != nullptr) //check that renderer is not already running and that link to SystemContainer exists
	{
		basicVisualizationSystemContainer->UpdateMaximumSceneCoordinates(); //this is done to make OpenGL zoom and maxSceneCoordinates work

		rendererError = 0; 

		if (verbose) { pout << "Setup OpenGL renderer ...\n"; }
		if (rendererThread.joinable()) //thread is still running from previous call ...
		{
			rendererThread.join();
			//rendererThread.~thread(); //this would make the thread unusable?
		}

		rendererThread = std::thread(GlfwRenderer::InitCreateWindow);
		Index timeOut = visSettings->window.startupTimeout / 10;

		Index i = 0;
		while(i++ < timeOut && !(rendererActive || rendererError > 0)) //wait 5 seconds for thread to answer; usually 150ms in Release and 500ms in debug mode
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		if (verbose) { pout << "waited for " << i * 10 << " milliseconds \n"; }
		if (rendererActive)
		{
			if (verbose) { pout << "OpenGL renderer started!\n"; return true; }
			basicVisualizationSystemContainer->SetVisualizationIsRunning(true); //render engine runs, graphicsupdate shall be done
		}
		else { 
			basicVisualizationSystemContainer->SetVisualizationIsRunning(false); //render engine did not start
			if (rendererError == 1)
			{
				SysError("Start of OpenGL renderer failed: glfwInit() failed");
			}
			else if (rendererError == 2)
			{
				SysError("Start of OpenGL renderer failed: glfwCreateWindow() failed");
			}
			else { SysError("Start of OpenGL renderer failed: timeout"); }
			return false;
		}
	}
	else
	{
		PyError("No SystemContainer has been. Renderer cannot be started without SystemContainer.");
		return false;
	}
	return false; //not needed, but to suppress warnings
}

//! stop the renderer engine and its thread
void GlfwRenderer::StopRenderer()
{
	if (window)
	{
		basicVisualizationSystemContainer->SetVisualizationIsRunning(false); //no further WaitForUserToContinue or GraphicsDataUpdates

		stopRenderer = true;
		glfwSetWindowShouldClose(window, 1);
		Index timeOut = 100; //2020-12-09: changed to 1000ms, because window can hang very long; visSettings->window.startupTimeout / 10;

		Index i = 0;
		while (i++ < timeOut && rendererActive) //wait 5 seconds for thread to answer
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		if (rendererActive) { SysError("OpenGL Renderer could not be stopped safely\n"); }
		//else { pout << "Renderer Stopped\n"; }

		//glfwDestroyWindow(window); //this is done in GLFW thread ...?
		//not necessary: glfwTerminate(); //test if this helps; should not be needed

		//delete window; //will not work? VS2017 reports warning that destructor will not be called, since window is only a struct
		window = nullptr; //this is used to identify if window has already been generated

		//after this command, this thread is terminated! ==> nothing will be done any more
		if (rendererThread.joinable()) //thread is still running from previous call ...
		{
			//pout << "join thread ...\n";
			rendererThread.join();
			//pout << "thread joined\n";
			//not necessary: rendererThread.~thread(); //check if this is necessary/right ==> will not be called after .joint() ...
		}
	}
}

void GlfwRenderer::InitCreateWindow()
{

	glfwSetErrorCallback(error_callback);
	if (!glfwInit())
	{
		rendererError = 1;
		exit(EXIT_FAILURE);
	}
	
	if (visSettings->openGL.multiSampling == 2 || visSettings->openGL.multiSampling == 4 || visSettings->openGL.multiSampling == 8 || visSettings->openGL.multiSampling == 16) //only 4 is possible right now ... otherwise no multisampling
	{
		glfwWindowHint(GLFW_SAMPLES, (int)visSettings->openGL.multiSampling); //multisampling=4, means 4 times larger buffers! but leads to smoother graphics
		glEnable(GL_MULTISAMPLE); //usually activated by default, but better to have it anyway
	}

	if (visSettings->window.alwaysOnTop)
	{
		glfwWindowHint(GLFW_FLOATING, GLFW_TRUE); //GLFW_FLOATING (default: GLFW_FALSE)  specifies whether the windowed mode window will be floating above other regular windows, also called topmost or always - on - top.This is intended primarily for debugging purposes and cannot be used to implement proper full screen windows.Possible values are GLFW_TRUE and GLFW_FALSE.
	}
	//glfwWindowHint(GLFW_FOCUSED, GLFW_TRUE); //(default: GLFW_TRUE) specifies whether the windowed mode window will be given input focus when created. Possible values are GLFW_TRUE and GLFW_FALSE.
	//GLFW_FOCUS_ON_SHOW (default: GLFW_TRUE) specifies whether the window will be given input focus when glfwShowWindow is called. Possible values are GLFW_TRUE and GLFW_FALSE
	//GLFW_SCALE_TO_MONITOR (default: GLFW_FALSE) specified whether the window content area should be resized based on the monitor content scale of any monitor it is placed on. This includes the initial placement when the window is created. Possible values are GLFW_TRUE and GLFW_FALSE.

	//window = glfwCreateWindow(visSettings->openGLWindowSize[0], visSettings->openGLWindowSize[1], "Exudyn OpenGL window", NULL, NULL);
	int sizex = (int)state->currentWindowSize[0];
	int sizey = (int)state->currentWindowSize[1];

	const int minWidth = 2; //avoid zero size
	const int minHeight = 2; //avoid zero size
	const int maxWidth = 2 * 8192; //limit upper size to 16K, 16:9 for now ...
	const int maxHeight = 2 * 4608; //limit upper size to 16K, 16:9 for now ...

	if (sizex < minWidth) { sizex = minWidth; }//limit lower size: negative numbers or zero could make problems ...
	if (sizey < minHeight) { sizey = minHeight; }

	if (sizex > maxWidth) { sizex = maxWidth; }
	if (sizey > maxHeight) { sizey = maxHeight; }

	window = glfwCreateWindow(sizex, sizey, "Exudyn OpenGL window", NULL, NULL);

	if (!window)
	{
		rendererError = 2;
		glfwTerminate();
		SysError("GLFWRenderer::InitCreateWindow: Render window could not be created");
		exit(EXIT_FAILURE);
	}
	//allow for very small windows, but never get 0 ...; x-size anyway limited due to windows buttons
	glfwSetWindowSizeLimits(window, 2, 2, maxWidth, maxHeight);

	rendererActive = true; //this is still threadsafe, because main thread waits for this signal!

	firstRun = 0; //zoom all on startup of window
	//+++++++++++++++++++++++++++++++++
	//set keyback functions
	glfwSetKeyCallback(window, key_callback);			//keyboard input
	glfwSetScrollCallback(window, scroll_callback);		//mouse wheel input
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetCursorPosCallback(window, cursor_position_callback);
	glfwSetWindowCloseCallback(window, window_close_callback);

	glfwSetWindowRefreshCallback(window, Render);
	glfwMakeContextCurrent(window);

	//+++++++++++++++++
	//initialize opengl
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);

	//std::cout << "OpenGL version=" << glGetString(GL_VERSION) << "\n";

	//+++++++++++++++++
	//determine the windows scale; TODO: add callback to redraw if monitor is changed: glfwSetWindowContentScaleCallback(...)
	float xWindowScale, yWindowScale;
	glfwGetWindowContentScale(window, &xWindowScale, &yWindowScale);
	fontScale = 0.5f*(xWindowScale + yWindowScale); //simplified for now!
	if (!visSettings->general.useWindowsMonitorScaleFactor) { fontScale = 1; }

	guint fontSize = (guint)(visSettings->general.textSize*fontScale);

	InitFontBitmap(fontSize);

	//+++++++++++++++++++++++++++++++++
	//depending on flags, do some changes to window
	if (visSettings->window.showWindow)
	{
		glfwShowWindow(window); //show the window when created ... should by anyway done, but did not work in Spyder so far
	}
	else
	{
		glfwIconifyWindow(window); //iconify window
	}
	if (visSettings->window.maximize)
	{
		glfwMaximizeWindow(window);
	}
	RunLoop();
}


void GlfwRenderer::RunLoop()
{
	//this is the OpenGL thread main loop
	while (rendererActive && !glfwWindowShouldClose(window) && 
		!stopRenderer && !globalPyRuntimeErrorFlag)
	{
		basicVisualizationSystemContainer->UpdateGraphicsData();
		if (basicVisualizationSystemContainer->GetAndResetZoomAllRequest()) { ZoomAll(); }
		Render(window);
		SaveImage(); //in case of flag, save frame to image file
		glfwWaitEventsTimeout((double)(visSettings->general.graphicsUpdateInterval)); //wait x seconds for next event
	}
	if (globalPyRuntimeErrorFlag)
	{
		rendererOut << "render window stopped because of error\n";
	}
	basicVisualizationSystemContainer->StopSimulation(); //if user waits for termination of render engine, it tells that window is closed

	glfwDestroyWindow(window);
	window = nullptr;
	rendererActive = false; //for new startup of renderer
	glfwTerminate(); //move to destructor
	stopRenderer = false;	//if stopped by user

	DeleteFonts();
}

void GlfwRenderer::Render(GLFWwindow* window) //GLFWwindow* needed in argument, because of glfwSetWindowRefreshCallback
{
	renderFunctionRunning.test_and_set(std::memory_order_acquire); //lock Render(...) function, no second call possible

	//rendererOut << "Render\n";
	float ratio;
	int width, height;

	glfwGetFramebufferSize(window, &width, &height);

	//rendererOut << "current window: width=" << width << ", height=" << height << "\n";
	state->currentWindowSize[0] = width;
	state->currentWindowSize[1] = height;

	ratio = (float)width;
	if (height != 0)
	{
		ratio = width / (float)height;
	}

	GLfloat zoom = state->zoom;
	
	//determine the windows scale; TODO: add callback to redraw if monitor is changed: glfwSetWindowContentScaleCallback(...)
	//float xWindowScale, yWindowScale;
	//glfwGetWindowContentScale(window, &xWindowScale, &yWindowScale);
	//fontScale = 0.5f*(xWindowScale + yWindowScale); //simplified for now!

	////do not use font scaling with bitmaps (they are internally scaled ...)
	//if (!visSettings->general.useWindowsMonitorScaleFactor/* || visSettings->general.useBitmapText*/) { fontScale = 1; }

	float fontSize = visSettings->general.textSize * fontScale; //use this size for fonts throughout

	glViewport(0, 0, width, height);
	//std::cout << "h=" << height << ", w=" << width << "\n";

	//original 2020-12-05: SetGLLights(); //must be very early, before anything to draw

	//get available line width range:
	//GLfloat LineRange[2];
	//glGetFloatv(GL_LINE_WIDTH_RANGE, LineRange);

	Float4 bg = visSettings->general.backgroundColor;
	glClearColor(bg[0], bg[1], bg[2], bg[3]); //(float red, float green, float blue, float alpha);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//glDisable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	GLdouble zFactor = 100.; //original:100
	glOrtho(-ratio * zoom, ratio*zoom, -zoom, zoom, -zFactor*2.*state->maxSceneSize, zFactor * 2.*state->maxSceneSize); //https: //www.khronos.org/opengl/wiki/Viewing_and_Transformations#How_do_I_implement_a_zoom_operation.3F
	//original (flipped?): 
	//glOrtho(-ratio * zoom, ratio*zoom, -1.f*zoom, 1.f*zoom, zFactor*2.*state->maxSceneSize, -zFactor * 2.*state->maxSceneSize); //https: //www.khronos.org/opengl/wiki/Viewing_and_Transformations#How_do_I_implement_a_zoom_operation.3F
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//create special background, looking more professional ...
	if (visSettings->general.useGradientBackground)
	{
		Float4 bg2 = visSettings->general.backgroundColorBottom;
		glDisable(GL_DEPTH_TEST);
		glBegin(GL_QUADS);
		//red color
		glColor3f(bg2[0], bg2[1], bg2[2]);
		float ax = zoom * ratio;
		float ay = zoom;
		glVertex2f(-ax, -ay);
		glVertex2f(ax, -ay);
		//blue color
		glColor3f(bg[0], bg[1], bg[2]);
		glVertex2f(ax, ay);
		glVertex2f(-ax, ay);
		glEnd();
		glEnable(GL_DEPTH_TEST);
	}

	//put here, will be fixed light seen from camera:
	SetGLLights(); //moved here 2020-12-05; light should now be rotation independent!

	glTranslated(-state->centerPoint[0], -state->centerPoint[1], 0.f); 

	glMultMatrixf(state->modelRotation.GetDataPointer());
	//glRotatef(state->rotations[2], 0.f, 0.f, 1.f);//((float)glfwGetTime() * 50.f, 0.f, 0.f, 1.f);
	//glRotatef(state->rotations[1], 0.f, 1.f, 0.f);
	//glRotatef(state->rotations[0], 1.f, 0.f, 0.f);

	//put here, will rotate with model-view:
	//SetGLLights(); //moved here 2020-12-05; light should now be rotation independent!

	RenderGraphicsData(fontScale);

	//glPushMatrix(); //store current matrix
	//glPopMatrix(); //restore matrix
	const int textIndentPixels = 10;
	const float verticalPixelOffset = (1.5f*fontSize*fontLargeFactor); //vertical offset of computation info
	Float4 textColor = visSettings->general.textColor;

	Index computationMessageNumberOfLines = 0; //compute offset for contour plot
	if (visSettings->general.showComputationInfo || visSettings->window.showMouseCoordinates) //draw coordinate system
	{
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		//float factor = 0.35f*zoom * 2.6f;
		//glTranslated(-factor*ratio*1.05, factor, 0.f); //old ; DELETE
		glDepthMask(GL_FALSE); //draw lines always in front

		//float scale = 2.f*fontSize*zoom / ((float)height);
		float hOff = 0.95f*(float)zFactor * 2.f*state->maxSceneSize; //draw in front; NEEDED since glDepthMask(GL_FALSE) ?


		if (visSettings->general.showComputationInfo)
		{
			Float2 pInfo = PixelToVertexCoordinates((float)textIndentPixels, 
				(float)(height) - (float)verticalPixelOffset); //fixed position, very top left window position
			Float3 poff({ pInfo[0], pInfo[1], hOff });
			Float2 pInfo2 = PixelToVertexCoordinates((float)textIndentPixels, 
				(float)(height) - fontSize*(1.f*fontLargeFactor) - (float)verticalPixelOffset); //fixed position, very top left window position
			Float3 poff2({ pInfo2[0], pInfo2[1], hOff });

			DrawString("EXUDYN", fontSize*fontLargeFactor, poff, textColor);
			
			std::string message = basicVisualizationSystemContainer->GetComputationMessage(visSettings->general.showSolverInformation,
				visSettings->general.showSolutionInformation, visSettings->general.showSolverTime);
			for (char c : message)
			{
				if (c == '\n') { computationMessageNumberOfLines++; }
			}
			DrawString(message.c_str(), fontSize, poff2, textColor);

			//+++++++++++++++++++
			//print version:
			Float2 pInfo3 = PixelToVertexCoordinates((float)(width-fontSize*fontSmallFactor*13), 5.f); //fixed position, very bottom right window position
			Float3 poff3({ pInfo3[0], pInfo3[1], hOff });
			DrawString((STDstring("version ")+EXUstd::exudynVersion).c_str(), fontSize*fontSmallFactor, poff3, textColor);
		}

		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		if (visSettings->window.showMouseCoordinates)
		{
			Vector2D mp = state->mouseCoordinates;
			Real xpos = mp[0];
			Real ypos = mp[1];

			float height = (float)state->currentWindowSize[1];
			float factor = 2.f*state->zoom / height;

			Vector2D ploc = factor * Vector2D({ xpos - 0.5*state->currentWindowSize[0], -1.*(ypos - 0.5*state->currentWindowSize[1]) });
			Vector2D cp({(double)state->centerPoint[0], (double)state->centerPoint[1] });

			Vector2D lastPressedCoords = factor * Vector2D({ stateMachine.lastMousePressedX - 0.5*state->currentWindowSize[0],
				-1.*(stateMachine.lastMousePressedY - 0.5*state->currentWindowSize[1]) }) + cp;

			char glx[16];
			char gly[16];
			sprintf(glx, "%7.3g", state->openGLcoordinates[0]);
			sprintf(gly, "%7.3g", state->openGLcoordinates[1]);
			char lpx[16];
			char lpy[16];
			sprintf(lpx, "%7.3g", lastPressedCoords[0]);
			sprintf(lpy, "%7.3g", lastPressedCoords[1]);
			char dist[16];
			sprintf(dist,"%7.3g", (state->openGLcoordinates - lastPressedCoords).GetL2Norm());

			STDstring mouseStr = STDstring("mouse=(") + glx + "," + gly + ")" +
				", last=(" + lpx + "," + lpy + "), dist=" + dist;
				//", cp=(" + EXUstd::ToString(cp[0]) + "," + EXUstd::ToString(cp[1]) + ")"+
				//", ploc=(" + EXUstd::ToString(ploc[0]) + "," + EXUstd::ToString(ploc[1]) + ")";

			//Float3 poff({ 0.f*zoom,-1.88f*zoom, hOff }); //old, using vertex coordinates
			Float2 pMouse = PixelToVertexCoordinates((float)textIndentPixels, 5); //fixed position, very bottom left window position
			Float3 poff({ pMouse[0], pMouse[1], hOff });

			DrawString(mouseStr.c_str(), fontSize*fontSmallFactor, poff, textColor);
		}
		glDepthMask(GL_TRUE); //draw lines always in front
	}

	if (visSettings->contour.showColorBar && visSettings->contour.outputVariable != OutputVariableType::_None) //draw coordinate system
	{
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		//float factor = 0.35f*zoom * 2.6f;
		//glTranslated(-factor * ratio*1.05, factor, 0.f); //old: now use pixel coordinates

		glDepthMask(GL_FALSE); //draw lines always in front
		float hOff = 0.9f*(float)zFactor * 2.f*state->maxSceneSize;   //quads in front
		float hOff2 = 0.95f*(float)zFactor * 2.f*state->maxSceneSize; //lines in front

		float scale = 2.f*fontSize*zoom / ((float)height);
		Float2 pContourBar = PixelToVertexCoordinates((float)textIndentPixels, 
			(float)height - (float)verticalPixelOffset - 
			(3.f+(float)computationMessageNumberOfLines)*(float)fontSize*1.54f); //factor 1.2f because space is slightly larger than fontSize for every line ... WHY?
		Float3 p0({ pContourBar[0], pContourBar[1],hOff2 }); //offset
		//Float3 p0({ 0.f,-2 * d,hOff2 }); //old, using vertex coordinates
		//float d = 0.05f*zoom; //old, using vertex coordinates
		float d = scale*1.6f;

		float minVal = 0;
		float maxVal = 1;
		if (graphicsDataList)
		{
			if (graphicsDataList->NumberOfItems() > 1) { pout << "WARNING: contour plot color bar only works for one single system\n"; }

			minVal = graphicsDataList->GetItem(0)->GetContourCurrentMinValue();
			maxVal = graphicsDataList->GetItem(0)->GetContourCurrentMaxValue();
		}
		//DrawString(basicVisualizationSystemContainer->GetComputationMessage().c_str(), scale, poff, textColor);
		STDstring contourStr = STDstring("contour plot: ")+GetOutputVariableTypeString(visSettings->contour.outputVariable) + "\ncomponent=" + EXUstd::ToString(visSettings->contour.outputVariableComponent) +
			"\nmin=" + EXUstd::ToString(minVal) + ",max=" + EXUstd::ToString(maxVal);
		DrawString(contourStr.c_str(), fontSize, p0, textColor);
		p0 += Float3({0.f,-3.2f*scale,0.f});


		//now draw boxes for contour plot colors and add texts
		float n = (float)visSettings->contour.colorBarTiling;
		float range = maxVal - minVal;
		for (float i = 0; i < n; i++)
		{
			float value = i / n * range + minVal;
			const float sizeX = 1.25f*d;
			const float sizeY = d;
			const float sizeY2 = d; //avoid spaces between fields
			//const float sizeX = 0.06f*zoom; //old, based on vertex coordinates
			//const float sizeY = 0.05f*zoom * n / 12.f;
			//const float sizeY2 = 0.05f*zoom * n / 12.f; //avoid spaces between fields

			bool drawFacesContourPlot = true;
			Float4 color0 = VisualizationSystemContainerBase::ColorBarColor(minVal, maxVal, value);

			if (drawFacesContourPlot)
			{
				Float4 color0 = VisualizationSystemContainerBase::ColorBarColor(minVal, maxVal, value);
				glBegin(GL_TRIANGLES);
				glColor3f(color0[0], color0[1], color0[2]);
				glVertex3f(p0[0], p0[1], hOff);
				glVertex3f(p0[0] + sizeX, p0[1], hOff);
				glVertex3f(p0[0] + sizeX, p0[1] - sizeY2, hOff);

				glVertex3f(p0[0], p0[1], hOff);
				glVertex3f(p0[0] + sizeX, p0[1] - sizeY2, hOff);
				glVertex3f(p0[0], p0[1] - sizeY2, hOff);
				glEnd();
			}
			if (visSettings->openGL.lineSmooth) { glEnable(GL_LINE_SMOOTH); }
			glLineWidth(visSettings->openGL.lineWidth);
			if (drawFacesContourPlot) { color0 = Float4({ 0.1f,0.1f,0.1f,1.f }); }
			glBegin(GL_LINE_STRIP);
			glColor3f(color0[0], color0[1], color0[2]);
			if (i != 0) { glVertex3f(p0[0] + sizeX, p0[1] - sizeY2, hOff2); }
			glVertex3f(p0[0], p0[1] - sizeY2, hOff2);
			glVertex3f(p0[0], p0[1], hOff2);
			glVertex3f(p0[0] + sizeX, p0[1], hOff2);
			glVertex3f(p0[0] + sizeX, p0[1] - sizeY2, hOff2);
			glEnd();
			if (visSettings->openGL.lineSmooth) { glDisable(GL_LINE_SMOOTH); }

			char str[20];
			std::sprintf(str, "% .2g", value);
			DrawString(str, fontSize*fontSmallFactor, p0 + Float3({ 1.2f*sizeX,-0.8f*sizeY,0}), textColor);

			p0 += Float3({ 0.f,-sizeY,0.f });

		}
		//		glBegin(GL_TRIANGLES);
		//		glColor3f(1.f, 0.f, 0.f);
		//		glVertex3f(-0.6f+i, -0.4f+j, 0.f);
		//		glColor3f(0.f, 1.f, 0.f);
		//		glVertex3f(0.6f+i, -0.4f+j, 0.f);
		//		glColor3f(0.f, 0.f, 1.f);
		//		glVertex3f(0.f+i, 0.6f+j, 0.f);
		//		glEnd();
		glDepthMask(GL_TRUE);
	}

	if (visSettings->general.drawCoordinateSystem) //draw coordinate system
	{
		//float d = zoom * 0.1;
		float scale = 2.f*fontSize*zoom / ((float)height); //text size in vertex coordinates
		//float d = visSettings->general.coordinateSystemSize*zoom;
		float d = 1.2f*visSettings->general.coordinateSystemSize*scale;
		//float a = d * 0.05f; //distance of text

		//2020-12-05: no hOff, as it makes problems showing the coordinate system! float hOff = 0.95f*(float)zFactor * 2.f*state->maxSceneSize; //draw in front
		//Float3 p0({ 0.f,0.f,hOff }); //
		Float3 p0({ 0.f,0.f,0 });
		Float3 v1({ d,  0.f,0.f });
		Float3 v2({ 0.f,  d,0.f });
		Float3 v3({ 0.f,0.f,  d }); //check why z-coordinates are flipped ...
		Float3 p1 = p0 + v1;
		Float3 p2 = p0 + v2;
		Float3 p3 = p0 + v3;

		//rendererOut << "zoom=" << zoom << "\n";

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		//float factor = -0.35f*zoom * 2.5f;
		//glOrtho(0.0f, ratio, 1, 0.0f, 0.0f, 1.0f);
		//float factor = -0.35f;

		Float2 pPix = PixelToVertexCoordinates(10 + visSettings->general.coordinateSystemSize*fontSize, 
			10 + (visSettings->general.coordinateSystemSize+0.f)*fontSize); //+1.f because of possible status line at bottom
		glTranslated(pPix[0], pPix[1], 0.f);

		//glTranslated(factor*ratio, factor, 0.f);
		glMultMatrixf(state->modelRotation.GetDataPointer());
		glDepthMask(GL_FALSE);

		glLineWidth(visSettings->openGL.lineWidth);
		if (visSettings->openGL.lineSmooth) { glEnable(GL_LINE_SMOOTH); }
		glBegin(GL_LINES);
		glColor3f(0.f, 0.f, 0.f);
		glVertex3f(p0[0], p0[1], p0[2]);
		glVertex3f(p1[0], p1[1], p1[2]);
		glVertex3f(p0[0], p0[1], p0[2]);
		glVertex3f(p2[0], p2[1], p2[2]);
		glVertex3f(p0[0], p0[1], p0[2]);
		glVertex3f(p3[0], p3[1], p3[2]);
		glEnd();
		if (visSettings->openGL.lineSmooth) { glDisable(GL_LINE_SMOOTH); }

		const char* X1 = "X(0)";
		const char* X2 = "Y(1)";
		const char* X3 = "Z(2)";

		Float16 m = state->modelRotation;
		Float16 matTp({m[0],m[4],m[ 8],m[12],
					   m[1],m[5],m[ 9],m[13],
					   m[2],m[6],m[10],m[14],
					   m[3],m[7],m[11],m[15]});

		Float3 poff({ 0.4f*scale, 0.4f*scale,0.f }); //small offset from axes

		glPushMatrix(); //store current matrix -> before rotation
		glTranslated(p1[0], p1[1], p1[2]);
		glMultMatrixf(matTp.GetDataPointer());
		glLineWidth(0.25f);
		DrawString(X1, fontSize, poff, textColor);
		glPopMatrix(); //restore matrix

		glPushMatrix(); //store current matrix -> before rotation
		glTranslated(p2[0], p2[1], p2[2]);
		glMultMatrixf(matTp.GetDataPointer());
		glLineWidth(0.5f);
		DrawString(X2, fontSize, poff, textColor);
		glPopMatrix(); //restore matrix

		glPushMatrix(); //store current matrix -> before rotation
		glTranslated(p3[0], p3[1], p3[2]);
		glMultMatrixf(matTp.GetDataPointer());
		glLineWidth(1.f);
		DrawString(X3, fontSize, poff, textColor);
		glPopMatrix(); //restore matrix

		glDepthMask(GL_TRUE);
	}



	glfwSwapBuffers(window);

	//rendererOut << "Render ready\n";

	firstRun++;
	//if (firstRun == 10) { ZoomAll(); }

	if ((firstRun *  visSettings->general.graphicsUpdateInterval) < 1. && visSettings->general.autoFitScene) { ZoomAll(); }

	//++++++++++++++++++++++++++++++++++++++++++
	renderFunctionRunning.clear(std::memory_order_release); //clear PostProcessData
}

void GlfwRenderer::SaveImage()
{
	//at this time, the scene must have been rendered (called directly from render loop after Render() )
	if (basicVisualizationSystemContainer->SaveImageRequest())
	{
		STDstring filename = visSettings->exportImages.saveImageFileName;

		if (!visSettings->exportImages.saveImageSingleFile)
		{
			char num[100];
			sprintf(num, "%05d", (int)visSettings->exportImages.saveImageFileCounter);

			filename += num;
			visSettings->exportImages.saveImageFileCounter++; //this changes the settings, because it should always contain the current value for consecutive simulations
		}
		filename += ".tga"; //image format ending

		SaveSceneToFile(filename);

		basicVisualizationSystemContainer->SaveImageFinished();
	}
}

void GlfwRenderer::SaveSceneToFile(const STDstring& filename)
{
	Index windowWidth = state->currentWindowSize[0];
	Index windowHeight = state->currentWindowSize[1];

	Index numberOfPixels = windowWidth * windowHeight * 3;
	ResizableArray<char> pixelBuffer(numberOfPixels);
	pixelBuffer.SetNumberOfItems(numberOfPixels);

	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadBuffer(GL_FRONT);
	glReadPixels(0, 0, (GLsizei)windowWidth, (GLsizei)windowHeight, GL_BGR_EXT, GL_UNSIGNED_BYTE, pixelBuffer.GetDataPointer());

	std::ofstream imageFile;
	CheckPathAndCreateDirectories(filename);
	imageFile.open(filename, std::ofstream::out | std::ofstream::binary);
	if (!imageFile.is_open()) //failed to open file ...  e.g. invalid file name
	{
		PyWarning(STDstring("GlfwRenderer::SaveSceneToFile: Failed to open image file '") + filename + "'");
	}
	else
	{
		short header[] = { 0, 2, 0, 0, 0, 0, (short)windowWidth, (short)windowHeight, 24 }; //file header for .tga (targa) images
		char* charHeader = (char*)(&header);

		imageFile.write(charHeader, sizeof(header));
		imageFile.write(pixelBuffer.GetDataPointer(), numberOfPixels);

		imageFile.close();
	}


}


void GlfwRenderer::RenderGraphicsData(float fontScale)
{
	if (graphicsDataList)
	{
		for (auto data : *graphicsDataList)
		{
			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//DRAW POINTS
			//GLfloat lineWidth = 0.5f; //has no action so far
			GLfloat d = visSettings->general.pointSize; //point drawing parameter --> put into settings!
			glLineWidth(visSettings->openGL.lineWidth);
			if (visSettings->openGL.lineSmooth) { glEnable(GL_LINE_SMOOTH); }

			for (const GLPoint& item : data->glPoints)
			{
				glBegin(GL_LINES);
				glColor4f(item.color[0], item.color[1], item.color[2], item.color[3]);

				//plot point as 3D cross
				glVertex3f(item.point[0] + d, item.point[1], item.point[2]);
				glVertex3f(item.point[0] - d, item.point[1], item.point[2]);
				glVertex3f(item.point[0], item.point[1] + d, item.point[2]);
				glVertex3f(item.point[0], item.point[1] - d, item.point[2]);
				glVertex3f(item.point[0], item.point[1], item.point[2] + d);
				glVertex3f(item.point[0], item.point[1], item.point[2] - d);

				glEnd();
			}

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//DRAW CIRCLES
			//draw a circle in xy-plane
			for (const GLCircleXY& item : data->glCirclesXY)
			{
				glBegin(GL_LINE_STRIP); //list of single points to define lines
				glColor4f(item.color[0], item.color[1], item.color[2], item.color[3]);

				const Float3& p = item.point;
				GLfloat r = item.radius;
				float nSeg = (float)item.numberOfSegments;
				if (nSeg == 0.f) { nSeg = (float)visSettings->general.circleTiling; }

				for (float i = 0; i <= nSeg; i += 2.f*EXUstd::pi_f / nSeg)
				{
					glVertex3f(p[0] + r * sin(i), p[1] + r * cos(i), p[2]);
				}

				glEnd(); //GL_LINE_STRIP
			}

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//DRAW LINES
			for (const GLLine& item : data->glLines)
			{
				glBegin(GL_LINES);
				glColor4f(item.color1[0], item.color1[1], item.color1[2], item.color1[3]);
				glVertex3f(item.point1[0], item.point1[1], item.point1[2]);
				glColor4f(item.color2[0], item.color2[1], item.color2[2], item.color2[3]);
				glVertex3f(item.point2[0], item.point2[1], item.point2[2]);
				glEnd();
			}

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//DRAW TRIANGLES
			if (visSettings->openGL.showFaceEdges)
			{
				for (const GLTriangle& trig : data->glTriangles)
				{ //draw lines
					glColor4f(0.2f, 0.2f, 0.2f, 1.f);
					for (Index i = 0; i < 3; i++)
					{
						Index j = i + 1;
						if (j >= 3) { j = 0; }
						glBegin(GL_LINES);
						const Float3& p = trig.points[i];
						glVertex3f(p[0], p[1], p[2]);

						const Float3& p1 = trig.points[j];
						glVertex3f(p1[0], p1[1], p1[2]);
						glEnd();
					}
				}
			}

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//DRAW TEXT (before triangles, in order to make texts visible in case of transparency
			if (visSettings->openGL.lineSmooth) { glDisable(GL_LINE_SMOOTH); }

			float textheight = visSettings->general.textSize;
			float scaleFactor = 2.f * state->zoom / ((float)state->currentWindowSize[1]); //factor, which gives approximately 1pt textsize

			Float16 m = state->modelRotation;

			Float16 matTp({ m[0],m[4],m[8],m[12], //transpose of modelRotation
						   m[1],m[5],m[9],m[13],
						   m[2],m[6],m[10],m[14],
						   m[3],m[7],m[11],m[15] });

			float textFontSize;

			for (const GLText& t : data->glTexts)
			{
				//delete: float scale = textheight * scaleFactor;
				//delete: if (t.size != 0.f) { scale = t.size * scaleFactor; }
				textFontSize = textheight * fontScale;
				if (t.size != 0.f) { textFontSize = t.size * fontScale; }

				float offx = t.offsetX * scaleFactor * textFontSize;
				float offy = t.offsetY * scaleFactor * textFontSize;
				//draw strings without applying the rotation:
				glPushMatrix(); //store current matrix -> before rotation
				glTranslated(t.point[0], t.point[1], t.point[2]);
				glMultMatrixf(matTp.GetDataPointer());
				DrawString(t.text, textFontSize, Float3({ offx,offy,0.f }), t.color);
				//delete: DrawString(t.text, fontScale*scale, Float3({ offx,offy,0.f }), t.color);
				glPopMatrix(); //restore matrix
			}
			//glPopMatrix(); //restore matrix

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//DRAW TRIANGLES
			if (visSettings->openGL.showFaces)
			{
				if (visSettings->openGL.enableLighting) { glEnable(GL_LIGHTING); } //only enabled when drawing triangle faces
				glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
				if (!visSettings->openGL.facesTransparent)
				{
					for (const GLTriangle& trig : data->glTriangles)
					{ //draw faces
						glBegin(GL_TRIANGLES);
						for (Index i = 0; i < 3; i++)
						{
							glColor4fv(trig.colors[i].GetDataPointer());
							glNormal3fv(trig.normals[i].GetDataPointer());
							glVertex3fv(trig.points[i].GetDataPointer());
						}
						glEnd();
					}
				}
				else //for global transparency of faces; slower
				{
					const float transparencyLimit = 0.4f; //use at least this transparency
					for (const GLTriangle& trig : data->glTriangles)
					{ //draw faces
						glBegin(GL_TRIANGLES);
						for (Index i = 0; i < 3; i++)
						{
							Float4 col = trig.colors[i];
							if (col[3] > transparencyLimit) { col[3] = transparencyLimit; }
							glColor4fv(col.GetDataPointer());
							glNormal3fv(trig.normals[i].GetDataPointer());
							glVertex3fv(trig.points[i].GetDataPointer());
						}
						glEnd();
					}
				}
				if (visSettings->openGL.enableLighting) { glDisable(GL_LIGHTING); } //only enabled when drawing triangle faces
				//glDisable(GL_LIGHTING);
			}

			//draw normals
			if (visSettings->openGL.drawFaceNormals)
			{
				float len = visSettings->openGL.drawNormalsLength;
				for (const GLTriangle& trig : data->glTriangles)
				{
					Float3 midPoint = { 0,0,0 };
					for (Index i = 0; i < 3; i++)
					{
						midPoint += trig.points[i];
					}
					midPoint *= 1.f / 3.f;
					glColor4f(0.2f, 0.2f, 0.2f, 1.f);
					glBegin(GL_LINES);
					const Float3& p = midPoint;
					glVertex3f(p[0], p[1], p[2]);
					Float3 p1 = midPoint + len * trig.normals[0];
					glVertex3f(p1[0], p1[1], p1[2]);
					glEnd();
				}
			}

			if (visSettings->openGL.drawVertexNormals)
			{
				float len = visSettings->openGL.drawNormalsLength;
				for (const GLTriangle& trig : data->glTriangles)
				{
					for (Index i = 0; i < 3; i++)
					{
						glBegin(GL_LINES);
						const Float3& p = trig.points[i];
						glVertex3f(p[0], p[1], p[2]);

						Float3 p1 = trig.points[i] + len * trig.normals[i];
						glVertex3f(p1[0], p1[1], p1[2]);
						glEnd();
					}
				}
			}

		}
	}
}


#endif //USE_GLFW_GRAPHICS
