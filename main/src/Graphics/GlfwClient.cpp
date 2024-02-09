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

#include "Graphics/GlfwClient.h"
#include "Utilities/SlimArray.h"
#include "Linalg/RigidBodyMath.h"



#ifdef USE_GLFW_GRAPHICS

#define GlfwRendererUsePNG //deactivate this flag for compatibility; switches to .TGA image output

//needs to be tested!!!
//#if defined(__EXUDYN__APPLE__)
//#undef GlfwRendererUsePNG 
//#endif 

//we need to exclude Python36 (in fact Ubuntu18.04, where glfw is not available with stb_image_write.h
#if (defined(__EXUDYN__LINUX__) && defined(__EXUDYN__PYTHON36))
#undef GlfwRendererUsePNG 
#endif

//GlfwRendererUsePNG only makes sense if GLFW_GRAPHICS is available
#ifdef GlfwRendererUsePNG
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "deps/stb_image_write.h" //for save image as .PNG
#endif

#ifdef __EXUDYN_USE_OPENVR 
#include "Graphics/OpenVRinterface.h"
extern OpenVRinterface glfwOpenVRinterface;
#endif //__EXUDYN_USE_OPENVR 


using namespace std::string_literals; // enables s-suffix for std::string literals

//if this flag is set, the GLFW thread will be detached (which may be advantageous is stoprenderer is not called); otherwise it is a joinable thread
#define detachGLFWthread 

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


//#include "Graphics/characterBitmap.h"
#include "Graphics/GlfwClient.h"
#include "System/versionCpp.h"


extern bool globalPyRuntimeErrorFlag; //stored in Stdoutput.cpp; this flag is set true as soon as a PyError or SysError is raised; this causes to shut down secondary processes, such as graphics, etc.
//extern bool deactivateGlobalPyRuntimeErrorFlag; //stored in Stdoutput.cpp; this flag is set true as soon as functions are called e.g. from command windows, which allow errors without shutting down the renderer
//use PrintDelayed(...) or ShowMessage(...) instead #define rendererOut std::cout //defines the type of output for renderer: pout could be problematic because of parallel threads; std::cout does not work in Spyder


//+++++++++++++++++++++++++++++++++++++
//#undef __EXUDYN_USE_OPENVR
#ifdef __EXUDYN_USE_OPENVR
#include "Graphics/OpenVRinterface.h"
OpenVRinterface openVRinterface;
//extern void InitializeOpenVR(GLFWwindow* window, GlfwRenderer* glfwRenderer);
//extern void RenderOpenVR(GLFWwindow* window, GlfwRenderer* glfwRenderer);
//extern void getTrackedDevicePoseMatrices(std::vector<Matrix4DF> &Controller, std::vector<Matrix4DF> &Tracker);
#endif
//+++++++++++++++++++++++++++++++++++++




GlfwRenderer glfwRenderer;

//++++++++++++++++++++++++++++++++++++++++++
//define static variables:
bool GlfwRenderer::rendererActive = false;
bool GlfwRenderer::stopRenderer = false;
bool GlfwRenderer::useMultiThreadedRendering = false;
Real GlfwRenderer::lastGraphicsUpdate = 0.;
Real GlfwRenderer::lastEventUpdate = 0.;	
Real GlfwRenderer::rendererStartTime = 0.;
Real GlfwRenderer::lastTryCloseWindow = 0.;
bool GlfwRenderer::callBackSignal = false;

Index GlfwRenderer::rendererError = 0;
GLFWwindow* GlfwRenderer::window = nullptr;
RenderState* GlfwRenderer::state;
RenderStateMachine GlfwRenderer::stateMachine;
std::thread GlfwRenderer::rendererThread;
Index GlfwRenderer::verboseRenderer = 0;         //0=False, 1=True (main output), 2=more info, 3=debug
//DELETE: Index GlfwRenderer::firstRun = 0; //zoom all in first run
std::atomic_flag GlfwRenderer::renderFunctionRunning = ATOMIC_FLAG_INIT;  //!< semaphore to check if Render(...)  function is currently running (prevent from calling twice); initialized with clear state
std::atomic_flag GlfwRenderer::showMessageSemaphore = ATOMIC_FLAG_INIT;   //!< semaphore for ShowMessage

BitmapFont GlfwRenderer::bitmapFont;				//!< bitmap font for regular texts, initialized upon start of renderer
//moved to render state; float GlfwRenderer::fontScale;						//!< monitor scaling factor from windows, to scale fonts
#ifndef USE_TEXTURED_BITMAP_FONTS
BitmapFont GlfwRenderer::bitmapFontSmall;			//!< bitmap font for small texts, initialized upon start of renderer
BitmapFont GlfwRenderer::bitmapFontLarge;			//!< bitmap font for large texts, initialized upon start of renderer
BitmapFont GlfwRenderer::bitmapFontHuge;			//!< bitmap font for huge texts, initialized upon start of renderer
#else
GLuint GlfwRenderer::textureNumberRGBbitmap[256*NUMBER_OF_TEXTUREFONT_LISTS];	//!< store texture number for our bitmap font; in ultimate case, there are 2*nCharacter lists
GLuint GlfwRenderer::bitmapFontListBase;			//!< starting index for GLlists for font bitmap textured quads
ResizableArray<GLubyte> GlfwRenderer::charBuffer;	//!< buffer for converstion of UTF8 into internal unicode-like format
#endif
bool GlfwRenderer::depthMask;                   //!< state of glDepthMask (except for fonts)
GLdouble GlfwRenderer::zFactor;                 //!< for clipping in ortho projection
GLuint GlfwRenderer::spheresListBase;			//!< starting index for GLlists for spheres


ResizableArray<GraphicsData*>* GlfwRenderer::graphicsDataList = nullptr;
//GraphicsData* GlfwRenderer::data = nullptr;
VisualizationSettings* GlfwRenderer::visSettings = nullptr;
VisualizationSystemContainerBase* GlfwRenderer::basicVisualizationSystemContainer = nullptr;
//++++++++++++++++++++++++++++++++++++++++++
Vector3DList GlfwRenderer::sensorTracePositions;
Vector3DList GlfwRenderer::sensorTraceVectors; //synchronized with triads
Matrix3DList GlfwRenderer::sensorTraceTriads;  //synchronized with vectors
Vector GlfwRenderer::sensorTraceValues; //temporary storage for current sensor data
//++++++++++++++++++++++++++++++++++++++++++


GlfwRenderer::GlfwRenderer()
{
	//take care, this may not be initialized:
	rendererActive = false;
	graphicsDataList = nullptr;
	window = nullptr;
	state = nullptr;
    depthMask = false;
    zFactor = 100.; //z-factor to avoid clipping; original:100; could be set via options ...

	//moved to renderState: fontScale = 1; //initialized if needed before bitmap initialization
	//renderState state cannot be initialized here, because it will be linked later to visualizationSystemContainer

	ResetStateMachine();
};


void GlfwRenderer::ResetStateMachine()
{
	stateMachine.leftMousePressed = false;
	stateMachine.rightMousePressed = false;
	stateMachine.shiftPressed = false;
	stateMachine.ctrlPressed = false;
	stateMachine.mode = RendererMode::_None;			//!< determines the state of any action

	stateMachine.mousePositionX = 0;	//!< last mouse position used for move and zoom
	stateMachine.mousePositionY = 0;	//!< last mouse position used for move and zoom
	stateMachine.lastMousePressedX = 0;	//!< last left mouse button position pressed
	stateMachine.lastMousePressedY = 0;

	//initialize highlight item:
	stateMachine.highlightIndex = invalidIndex;
	stateMachine.highlightType = ItemType::_None;
	stateMachine.highlightMbsNumber = 0;
	stateMachine.highlightTimeout = 0.;

	stateMachine.selectionMouseCoordinates = Vector2D({ 0.,0. });
}

//! add status message, e.g., if button is pressed
void GlfwRenderer::ShowMessage(const STDstring& str, Real timeout)
{
	EXUstd::WaitAndLockSemaphore(showMessageSemaphore); 
	stateMachine.rendererMessage = str;
	if (timeout != 0)
	{
		stateMachine.renderMessageTimeout = EXUstd::GetTimeInSeconds() + timeout;
	}
	else
	{
		stateMachine.renderMessageTimeout = 0;
	}
	EXUstd::ReleaseSemaphore(showMessageSemaphore); 
}

void GlfwRenderer::window_close_callback(GLFWwindow* window)
{
	if (PyGetRendererCallbackLock()) { 
        glfwSetWindowShouldClose(window, GL_FALSE);
        return;
    }

    bool reallyQuit = true;
    //PrintDelayed("window_close_callback");
    //PrintDelayed("tstart=" + EXUstd::ToString(rendererStartTime) +
    //    ", tcurrent=" + EXUstd::ToString(EXUstd::GetTimeInSeconds()));
    //PrintDelayed("quittime=" + EXUstd::ToString(visSettings->window.reallyQuitTimeLimit) +
    //    ", lastTryCloseWindow=" + EXUstd::ToString(lastTryCloseWindow));

    const Real timeoutCloseWindow = 8;
    if ((EXUstd::GetTimeInSeconds() - rendererStartTime > visSettings->window.reallyQuitTimeLimit) &&
        (EXUstd::GetTimeInSeconds() - lastTryCloseWindow > 2)) //hardcoded, 2 seconds
    {
        reallyQuit = false;
        ShowMessage("To really close window, click twice on icon", timeoutCloseWindow);
        if (verboseRenderer) { PrintDelayed("Long running simulation: requires second click to close window!"); }
        glfwSetWindowShouldClose(window, GL_FALSE);
    }

    lastTryCloseWindow = EXUstd::GetTimeInSeconds();
    if (reallyQuit)
    {
        ShowMessage("closing renderer ..."); //in regular cases this is not visible
        basicVisualizationSystemContainer->StopSimulation();		//stop solver if running
        basicVisualizationSystemContainer->ForceQuitSimulation();	//if solver is not running, also tell that it shall be shut down if started

        glfwSetWindowShouldClose(window, GL_FALSE);
        stopRenderer = true;
    }
}

void GlfwRenderer::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (PyGetRendererCallbackLock()) { return; }
	SetCallBackSignal();
	//if (graphicsUpdateAtomicFlag.test_and_set(std::memory_order_acquire)) { return; } //ignore keys if currently in use

	//EXUstd::WaitAndLockSemaphore(graphicsUpdateAtomicFlag);

	const Real timeoutShowItem = 2; //seconds

	if ((key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) ||
        (key == GLFW_KEY_Q && action == GLFW_PRESS && mods == 0))
	{
        bool reallyQuit = true;
        if (EXUstd::GetTimeInSeconds() - rendererStartTime > visSettings->window.reallyQuitTimeLimit)
        {
            if (verboseRenderer) { PrintDelayed("Long running simulation: requires additional action to quit!"); }
            //result=-2: exception occured
            //result=-1: undefined quit response
            //result= 0: default result value: unused HERE
            //result= 1: tkinter dialog opened
            //result= 2: do not quit
            //result= 3: quit

            PyProcessSetResult(-1); //initialize as undefined
            PyQueuePythonProcess(ProcessID::AskYesNo,0);

            if (!useMultiThreadedRendering) //if not multithreaded, we have to call the queue
            {
                PyProcessExecuteQueue(); //if still some elements open in queue; MAY ONLY BE DONE IN SINGLE-THREADED MODE
                PyProcessExecuteQueue(); //if still some elements open in queue; MAY ONLY BE DONE IN SINGLE-THREADED MODE
            }
            //PrintDelayed("wait for askyesno process to be released");

            //now wait if there is a response (at least tkinter dialog has opened ...)
            if (useMultiThreadedRendering) //if not multithreaded, queue already processed and result is there
            {
                Index timeOut = 100;
                Index i = 0;
                while ((i++ < timeOut) && (PyProcessGetResult() == -1))
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                if (verboseRenderer) { PrintDelayed("QUIT renderer: waited for " + EXUstd::ToString( i * 20) + " milliseconds \n"); }
                //PrintDelayed("wait for askyesno process to be finished");

                while (PyProcessGetResult() == 1) //this signals that tkinter dialog opened; now wait what happens; result gets -2 in case of exception
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                if (verboseRenderer) { PrintDelayed("  Quit function=" + EXUstd::ToString(PyProcessGetResult())); }
            }
            //PrintDelayed("evaluate:");
            if (PyProcessGetResult() <= -1) //no tkinter dialog opened or crashed
            {
                //PrintDelayed("no tkinter; result=" + EXUstd::ToString(PyProcessGetResult()) );
                const Real timeOutMessage = 8;
                if (EXUstd::GetTimeInSeconds() - lastTryCloseWindow > 1)
                {
                    ShowMessage("To really close window, press Q or Escape second time!", timeOutMessage);
                    lastTryCloseWindow = EXUstd::GetTimeInSeconds();
                    PyProcessSetResult(2); //do not quit
                }
                else
                {
                    PyProcessSetResult(3); //second trial ... now quit
                }
            }
            if (verboseRenderer) { PrintDelayed("Quit action code=" + EXUstd::ToString(PyProcessGetResult())); }
            reallyQuit = (PyProcessGetResult() == 3);
            PyProcessSetResult(0); //default
        }
        if (reallyQuit)
        {
            basicVisualizationSystemContainer->StopSimulation();		//stop solver if running
            if (key == GLFW_KEY_ESCAPE) //escape does more than Q
            {
                basicVisualizationSystemContainer->ForceQuitSimulation();	//if solver is not running, also tell that it shall be shut down if started

                stopRenderer = true;
                //glfwSetWindowShouldClose(window, GL_TRUE);

                return; //don't process keys or call user function
            }
        }
	}

	//switch ignore keys functionality
	if (key == GLFW_KEY_F2 && action == GLFW_PRESS)
	{
		visSettings->window.ignoreKeys = !visSettings->window.ignoreKeys;
		//rendererOut << "ignore keys mode switched to " << visSettings->window.ignoreKeys << "\n";
		//PyQueueExecutableString("print('ignore keys mode switched to " + EXUstd::ToString(visSettings->window.ignoreKeys) + "')\n");
		ShowMessage("ignore keys mode switched " + OnOffFromBool(visSettings->window.ignoreKeys), timeoutShowItem);
	}

	//do this first, as key may still have time to complete action
	if (visSettings->window.ignoreKeys) //2023-06-03: makes no sense: || !(key == GLFW_KEY_Q && action == GLFW_PRESS && mods == 0))
	{
		//keyPressUserFunction uses the pybind interface and thus causes crashes when set or copied (Python thread!):
		PyQueueKeyPressed(key, action, mods); // visSettings->window.keyPressUserFunction); //call python user function
	}

	//+++++++++++++++++++++++++++++++++++++++++++++
	//check if regular keys are ignored:
	if (!visSettings->window.ignoreKeys)
	{
		//keycode to quit simulation:
		//if (key == GLFW_KEY_Q && action == GLFW_PRESS && mods == 0)
		//{
		//	basicVisualizationSystemContainer->StopSimulation();
		//}

		//keycode to continue paused simulation or to pause:
		if ((key == GLFW_KEY_SPACE && action == GLFW_PRESS && mods == 0) ||
			(key == GLFW_KEY_SPACE && action == GLFW_REPEAT)) //changed shift to repeat 
		{
            if (visSettings->interactive.pauseWithSpacebar)
            {
                ShowMessage("SPACE pressed: switch pause on/off", timeoutShowItem);
                basicVisualizationSystemContainer->SwitchPauseSimulation();
            }
            else //only continue, but no pause:
            {
                basicVisualizationSystemContainer->ContinueSimulation();
            }
        }

		//switch ignore keys functionality
		if (key == GLFW_KEY_F3 && action == GLFW_PRESS)
		{
			visSettings->window.showMouseCoordinates = !visSettings->window.showMouseCoordinates;
			stateMachine.rendererMessage = "";
		}

		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		//visualization update keys:
		if (key == GLFW_KEY_1 && action == GLFW_PRESS && mods == 0)
		{
			visSettings->general.graphicsUpdateInterval = 0.02f;
			ShowMessage("Visualization update: 20ms", timeoutShowItem);
		}

		if (key == GLFW_KEY_2 && action == GLFW_PRESS && mods == 0)
		{
			visSettings->general.graphicsUpdateInterval = 0.1f;
			ShowMessage("Visualization update: 100ms", timeoutShowItem);
		}

		if (key == GLFW_KEY_3 && action == GLFW_PRESS && mods == 0)
		{
			visSettings->general.graphicsUpdateInterval = 0.5f;
			ShowMessage("Visualization update: 0.5s", timeoutShowItem);
		}

		if (key == GLFW_KEY_4 && action == GLFW_PRESS && mods == 0)
		{
			visSettings->general.graphicsUpdateInterval = 2.f;
			ShowMessage("Visualization update: 2s", timeoutShowItem);
		}

		if (key == GLFW_KEY_5 && action == GLFW_PRESS && mods == 0)
		{
			visSettings->general.graphicsUpdateInterval = 100.f;
			ShowMessage("Visualization update: 100s", timeoutShowItem);
		}

		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
		//process keys for showing nodes, bodies, ...
		if (key == GLFW_KEY_N && action == GLFW_PRESS && mods != GLFW_MOD_CONTROL)
		{
			visSettings->nodes.show = !visSettings->nodes.show; UpdateGraphicsDataNow();
			ShowMessage("show nodes: "+ OnOffFromBool(visSettings->nodes.show), timeoutShowItem);
		}
		if (key == GLFW_KEY_B && action == GLFW_PRESS && mods != GLFW_MOD_CONTROL)
		{
			visSettings->bodies.show = !visSettings->bodies.show; UpdateGraphicsDataNow();
			ShowMessage("show bodies: " + OnOffFromBool(visSettings->bodies.show), timeoutShowItem);
		}
		if (key == GLFW_KEY_C && action == GLFW_PRESS && mods != GLFW_MOD_CONTROL)
		{
			visSettings->connectors.show = !visSettings->connectors.show; UpdateGraphicsDataNow();
			ShowMessage("show connectors: " + OnOffFromBool(visSettings->connectors.show), timeoutShowItem);
		}
		if (key == GLFW_KEY_M && action == GLFW_PRESS && mods != GLFW_MOD_CONTROL)
		{
			visSettings->markers.show = !visSettings->markers.show; UpdateGraphicsDataNow();
			ShowMessage("show markers: " + OnOffFromBool(visSettings->markers.show), timeoutShowItem);
		}
		if (key == GLFW_KEY_L && action == GLFW_PRESS && mods != GLFW_MOD_CONTROL)
		{
			visSettings->loads.show = !visSettings->loads.show; UpdateGraphicsDataNow();
			ShowMessage("show loads: " + OnOffFromBool(visSettings->loads.show), timeoutShowItem);
		}
		if (key == GLFW_KEY_S && action == GLFW_PRESS && mods != GLFW_MOD_CONTROL)
		{
			visSettings->sensors.show = !visSettings->sensors.show; UpdateGraphicsDataNow();
			ShowMessage("show sensors: " + OnOffFromBool(visSettings->sensors.show), timeoutShowItem);
		}
		//show node, object, ... numbers:
		if (key == GLFW_KEY_N && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			visSettings->nodes.showNumbers = !visSettings->nodes.showNumbers; UpdateGraphicsDataNow();
			if (visSettings->nodes.showNumbers) { visSettings->nodes.showNumbers = true; }
			ShowMessage("show node numbers: " + OnOffFromBool(visSettings->nodes.showNumbers), timeoutShowItem);
		}
		if (key == GLFW_KEY_B && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			visSettings->bodies.showNumbers = !visSettings->bodies.showNumbers; UpdateGraphicsDataNow();
			if (visSettings->bodies.showNumbers) { visSettings->bodies.show = true; }
			ShowMessage("show body numbers: " + OnOffFromBool(visSettings->bodies.showNumbers), timeoutShowItem);
		}
		if (key == GLFW_KEY_C && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			visSettings->connectors.showNumbers = !visSettings->connectors.showNumbers; UpdateGraphicsDataNow();
			if (visSettings->connectors.showNumbers) { visSettings->connectors.show = true; }
			ShowMessage("show connector numbers: " + OnOffFromBool(visSettings->connectors.showNumbers), timeoutShowItem);
		}
		if (key == GLFW_KEY_M && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			visSettings->markers.showNumbers = !visSettings->markers.showNumbers; UpdateGraphicsDataNow();
			if (visSettings->markers.showNumbers) { visSettings->markers.show = true; }
			ShowMessage("show markers numbers: " + OnOffFromBool(visSettings->markers.showNumbers), timeoutShowItem);
		}
		if (key == GLFW_KEY_L && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			visSettings->loads.showNumbers = !visSettings->loads.showNumbers; UpdateGraphicsDataNow();
			if (visSettings->loads.showNumbers) { visSettings->loads.show = true; }
			ShowMessage("show loads numbers: " + OnOffFromBool(visSettings->loads.showNumbers), timeoutShowItem);
		}
		if (key == GLFW_KEY_S && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			visSettings->sensors.showNumbers = !visSettings->sensors.showNumbers; UpdateGraphicsDataNow();
			if (visSettings->sensors.showNumbers) { visSettings->sensors.show = true; }
			ShowMessage("show sensor numbers: " + OnOffFromBool(visSettings->sensors.showNumbers), timeoutShowItem);
		}
		if (key == GLFW_KEY_T && action == GLFW_PRESS && mods != GLFW_MOD_CONTROL)
		{
			//OLD: visSettings->openGL.facesTransparent = !visSettings->openGL.facesTransparent;
			//switch between faces transparent + edges / faces transparent / only face edges / full faces with edges / only faces
			if (!visSettings->openGL.facesTransparent && visSettings->openGL.showFaces && !visSettings->openGL.showFaceEdges && visSettings->openGL.showMeshFaces && visSettings->openGL.showMeshEdges)
			{
				visSettings->openGL.facesTransparent = false;
				visSettings->openGL.showFaces = true;
				visSettings->openGL.showFaceEdges = false;
				visSettings->openGL.showMeshFaces = true;
				visSettings->openGL.showMeshEdges = false;
			}
			else if (!visSettings->openGL.facesTransparent && visSettings->openGL.showFaces && !visSettings->openGL.showFaceEdges && visSettings->openGL.showMeshFaces && !visSettings->openGL.showMeshEdges)
			{
				visSettings->openGL.facesTransparent = true;
				visSettings->openGL.showFaces = true;
				visSettings->openGL.showFaceEdges = true;
				visSettings->openGL.showMeshFaces = true;
				visSettings->openGL.showMeshEdges = true;
			}
			else if (visSettings->openGL.facesTransparent && visSettings->openGL.showFaces && visSettings->openGL.showFaceEdges && visSettings->openGL.showMeshFaces && visSettings->openGL.showMeshEdges)
			{
				visSettings->openGL.facesTransparent = true;
				visSettings->openGL.showFaces = true;
				visSettings->openGL.showFaceEdges = false;
				visSettings->openGL.showMeshFaces = true;
				visSettings->openGL.showMeshEdges = true;
			}
			else if (visSettings->openGL.facesTransparent && visSettings->openGL.showFaces && !visSettings->openGL.showFaceEdges && visSettings->openGL.showMeshFaces && visSettings->openGL.showMeshEdges)
			{
				visSettings->openGL.facesTransparent = false;
				visSettings->openGL.showFaces = true;
				visSettings->openGL.showFaceEdges = true;
				visSettings->openGL.showMeshFaces = true;
				visSettings->openGL.showMeshEdges = true;
			}
			else if (!visSettings->openGL.facesTransparent && visSettings->openGL.showFaces && visSettings->openGL.showFaceEdges && visSettings->openGL.showMeshFaces && visSettings->openGL.showMeshEdges)
			{
				visSettings->openGL.facesTransparent = false;
				visSettings->openGL.showFaces = false;
				visSettings->openGL.showFaceEdges = true;
				visSettings->openGL.showMeshFaces = false;
				visSettings->openGL.showMeshEdges = true;
			}
			else if (!visSettings->openGL.facesTransparent && !visSettings->openGL.showFaces && visSettings->openGL.showFaceEdges && !visSettings->openGL.showMeshFaces && visSettings->openGL.showMeshEdges)
			{
				visSettings->openGL.facesTransparent = false;
				visSettings->openGL.showFaces = false;
				visSettings->openGL.showFaceEdges = true;
				visSettings->openGL.showMeshFaces = true;
				visSettings->openGL.showMeshEdges = true;
			}
			//else if (!visSettings->openGL.facesTransparent && !visSettings->openGL.showFaces && visSettings->openGL.showFaceEdges && visSettings->openGL.showMeshEdges && visSettings->openGL.showMeshFaces)
			//{
			//	visSettings->openGL.facesTransparent = false;
			//	visSettings->openGL.showFaces = true;
			//	visSettings->openGL.showFaceEdges = false;
			//	visSettings->openGL.showMeshFaces = true;
			//	visSettings->openGL.showMeshEdges = true;
			//}
			else
			{
				visSettings->openGL.facesTransparent = false;
				visSettings->openGL.showFaces = true;
				visSettings->openGL.showFaceEdges = false;
				visSettings->openGL.showMeshFaces = true;
				visSettings->openGL.showMeshEdges = true;
			}
			
			UpdateGraphicsDataNow();
			ShowMessage("faces transparent=" + OnOffFromBool(visSettings->openGL.facesTransparent) +
				", faces=" + OnOffFromBool(visSettings->openGL.showFaces) +
				", face edges=" + OnOffFromBool(visSettings->openGL.showFaceEdges) +
				", mesh faces=" + OnOffFromBool(visSettings->openGL.showMeshFaces) +
				", mesh edges=" + OnOffFromBool(visSettings->openGL.showMeshEdges)
				, timeoutShowItem);
		}
		if (key == GLFW_KEY_X && action == GLFW_PRESS)
		{
			if (PyGetRendererPythonCommandLock())
			{
				ShowMessage("execute command not possible; other dialog already running", 5);
			}
			else
			{
				ShowMessage("execute command ... (see other window)", 2);
				UpdateGraphicsDataNow();
				Render(window);
				//queue process and execute as soon as possible in Python (main) thread
				PySetRendererMultiThreadedDialogs(visSettings->dialogs.multiThreadedDialogs);
				PyQueuePythonProcess(ProcessID::ShowPythonCommandDialog);
			}
		}
		//visualization settings dialog
		if (key == GLFW_KEY_V && action == GLFW_PRESS)
		{
			if (PyGetRendererPythonCommandLock())
			{
				ShowMessage("edit VisualizationSettings not possible; other dialog already running", 5);
			}
			else
			{
				ShowMessage("edit VisualizationSettings (see other window)", 2);
				UpdateGraphicsDataNow();
				Render(window);
				//queue process and execute as soon as possible in Python (main) thread
				PySetRendererMultiThreadedDialogs(visSettings->dialogs.multiThreadedDialogs);
				PyQueuePythonProcess(ProcessID::ShowVisualizationSettingsDialog);
				UpdateGraphicsDataNow();
			}
		}
		//help key
		if (key == GLFW_KEY_H && action == GLFW_PRESS)
		{
			if (PyGetRendererPythonCommandLock())
			{
				ShowMessage("show help information not possible; other dialog already running", 5);
			}
			else
			{
				ShowMessage("show help information (see other window)", 2);
				UpdateGraphicsDataNow();
				Render(window);
				//queue process and execute as soon as possible in Python (main) thread
				PySetRendererMultiThreadedDialogs(visSettings->dialogs.multiThreadedDialogs);
				PyQueuePythonProcess(ProcessID::ShowHelpDialog);
			}
		}

		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
		//process keys for move, rotate, zoom
		float rotStep = visSettings->interactive.keypressRotationStep; //degrees
		float transStep = visSettings->interactive.keypressTranslationStep * state->zoom; //degrees
		if ((mods & GLFW_MOD_CONTROL) != 0) //only for rotStep and transStep
		{
			rotStep *= 0.1f;
			transStep *= 0.1f;
		}

		float zoomStep = visSettings->interactive.zoomStepFactor;
		Float3 incRot({ 0.f,0.f,0.f });
		bool hasShift = (mods & GLFW_MOD_SHIFT) != 0;
		bool hasAlt = (mods & GLFW_MOD_ALT) != 0;

		if (key == GLFW_KEY_KP_2 && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[0] = rotStep; }
		if (key == GLFW_KEY_KP_8 && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[0] = -rotStep; }
		if (key == GLFW_KEY_KP_4 && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[1] = rotStep; }
		if (key == GLFW_KEY_KP_6 && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[1] = -rotStep; }
		if (key == GLFW_KEY_KP_7 && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[2] = rotStep; }
		if (key == GLFW_KEY_KP_9 && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[2] = -rotStep; }

		if (hasShift && key == GLFW_KEY_UP && (action == GLFW_PRESS || action == GLFW_REPEAT))	  { incRot[0] = rotStep; }
		if (hasShift && key == GLFW_KEY_DOWN && (action == GLFW_PRESS || action == GLFW_REPEAT))  { incRot[0] = -rotStep; }
		if (hasShift && key == GLFW_KEY_LEFT && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[1] = rotStep; }
		if (hasShift && key == GLFW_KEY_RIGHT && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[1] = -rotStep; }
		if (hasAlt && key == GLFW_KEY_LEFT && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[2] = rotStep; }
		if (hasAlt && key == GLFW_KEY_RIGHT && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[2] = -rotStep; }

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
		//}

		//change view:
		if (key == GLFW_KEY_1 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			//glRotated(180, 0.0, 1.0, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			ShowMessage("View 1: 1-2-plane", timeoutShowItem);
		}

		if (key == GLFW_KEY_2 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(-90, 1.0, 0.0, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			ShowMessage("View 2: 1-3-plane", timeoutShowItem);
		}

		if (key == GLFW_KEY_3 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(-90, 0.0, 1.0, 0.0);
			glRotated(-90, 1.0, 0.0, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			ShowMessage("View 3: 2-3-plane", timeoutShowItem);
		}

		if (key == GLFW_KEY_1 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(180, 0.0, 1.0, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			ShowMessage("View 1: 1-2-plane mirrored about vertical axis", timeoutShowItem);
		}

		if (key == GLFW_KEY_2 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(-90, 1.0, 0.0, 0.0);
			glRotated(180, 0.0, 0.0, 1.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			ShowMessage("View 2: 1-3-plane mirrored about vertical axis", timeoutShowItem);
		}

		if (key == GLFW_KEY_3 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(-90, 0.0, 1.0, 0.0);
			glRotated(-90, 1.0, 0.0, 0.0);
			glRotated(180, 0.0, 0.0, 1.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			ShowMessage("View 3: 2-3-plane mirrored about vertical axis", timeoutShowItem);
		}

		//second group of views:
		if (key == GLFW_KEY_4 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(180, 0.0, 1.0, 0.0);
			glRotated(90, 0.0, 0.0, 1.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			ShowMessage("View 4: 2-1-plane", timeoutShowItem);
		}

		if (key == GLFW_KEY_5 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(90, 0.0, 1.0, 0.0);
			glRotated(90, 0.0, 0.0, 1.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			ShowMessage("View 5: 3-1-plane", timeoutShowItem);
		}

		if (key == GLFW_KEY_6 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(90, 0.0, 1.0, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			ShowMessage("View 6: 3-2-plane", timeoutShowItem);
		}

		if (key == GLFW_KEY_4 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(90, 0.0, 0.0, 1.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			ShowMessage("View 4: 2-1-plane mirrored about vertical axis", timeoutShowItem);
		}

		if (key == GLFW_KEY_5 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(-90, 1.0, 0.0, 0.0);
			glRotated(-90, 0.0, 1.0, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			ShowMessage("View 5: 3-1-plane mirrored about vertical axis", timeoutShowItem);
		}

		if (key == GLFW_KEY_6 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotated(-90, 0.0, 1.0, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
			ShowMessage("View 6: 3-2-plane mirrored about vertical axis", timeoutShowItem);
		}
		
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
		if (!hasShift && !hasAlt && key == GLFW_KEY_UP && (action == GLFW_PRESS || action == GLFW_REPEAT)) { state->centerPoint[1] -= transStep; }
		if (!hasShift && !hasAlt && key == GLFW_KEY_DOWN && (action == GLFW_PRESS || action == GLFW_REPEAT)) { state->centerPoint[1] += transStep; }
		if (!hasShift && !hasAlt && key == GLFW_KEY_LEFT && (action == GLFW_PRESS || action == GLFW_REPEAT)) { state->centerPoint[0] += transStep; }
		if (!hasShift && !hasAlt && key == GLFW_KEY_RIGHT && (action == GLFW_PRESS || action == GLFW_REPEAT)) { state->centerPoint[0] -= transStep; }

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

		if (key == GLFW_KEY_A && action == GLFW_PRESS) 
		{ 
			ZoomAll(); 
		}
		if (key == GLFW_KEY_O && action == GLFW_PRESS)
		{
			const float* A = state->modelRotation.GetDataPointer();
			Matrix3DF rotationMV(3, 3, { A[0], A[1], A[2],  A[4], A[5], A[6],  A[8], A[9], A[10] });

			//Float3 p = state->rotationCenterPoint * rotationMV + (state->centerPoint - state->rotationCenterPoint);
			//state->rotationCenterPoint = state->centerPoint;// rotationMV * state->rotationCenterPoint;
			Float3 pOld = state->rotationCenterPoint * rotationMV + state->centerPoint;
			state->rotationCenterPoint = state->rotationCenterPoint + rotationMV * state->centerPoint;
			state->centerPoint = pOld - state->rotationCenterPoint * rotationMV;

			//PrintDelayed("new centerpoint=" + EXUstd::ToString(state->rotationCenterPoint));
		}
	}
	//EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag);
}

void GlfwRenderer::ComputeMaxSceneSize(float& maxSceneSize, Float3& center)
{
	//max scene size from current line data:
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
			for (auto item : data->glSpheres)
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

		center = 0.5f*(pmin + pmax);

		maxSceneSize = (pmax - pmin).GetL2Norm();
		if (maxSceneSize < visSettings->general.minSceneSize) { maxSceneSize = visSettings->general.minSceneSize; }

		if (graphicsDataList->NumberOfItems() == 0 ||
			((*graphicsDataList)[0]->glCirclesXY.NumberOfItems() == 0 && (*graphicsDataList)[0]->glLines.NumberOfItems() == 0
				&& (*graphicsDataList)[0]->glSpheres.NumberOfItems() == 0 && (*graphicsDataList)[0]->glTexts.NumberOfItems() == 0
				&& (*graphicsDataList)[0]->glTriangles.NumberOfItems() == 0))
		{
			maxSceneSize = 1;
			center = Float3({ 0,0,0 });
		}
	}
}

//! at beginning with autofitScene and when manually requested
void GlfwRenderer::ZoomAll(bool updateGraphicsData, bool computeMaxScene, bool render)
{
	if (updateGraphicsData) { UpdateGraphicsDataNow(); }
	if (computeMaxScene) { ComputeMaxSceneSize(state->maxSceneSize, state->centerPoint); }

	state->zoom = 0.4f*state->maxSceneSize;

	if (render) { Render(window); }
}

void GlfwRenderer::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	if (PyGetRendererCallbackLock()) { return; }
	SetCallBackSignal();
	//rendererOut << "scroll: x=" << xoffset << ", y=" << yoffset << "\n";
	float zoomStep = visSettings->interactive.zoomStepFactor;

	//if (yoffset > 0) { state->zoom /= zoomStep * (float)yoffset; }
	//if (yoffset < 0) { state->zoom *= zoomStep * (float)(-yoffset); }
	if (yoffset*(double)zoomStep > 0) { state->zoom /= pow(fabs(zoomStep), fabs((float)yoffset)); }
	if (yoffset*(double)zoomStep < 0) { state->zoom *= pow(fabs(zoomStep), fabs((float)yoffset)); }
	//rendererOut << "zoom=" << state->zoom << "\n";
}

void GlfwRenderer::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (PyGetRendererCallbackLock()) { return; }
	SetCallBackSignal();
	//EXUstd::WaitAndLockSemaphore(graphicsUpdateAtomicFlag);

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
		//check, if it was a regular mouse press without moving
		if (stateMachine.lastMousePressedX == stateMachine.mousePositionX &&
			stateMachine.lastMousePressedY == stateMachine.mousePositionY &&
			visSettings->interactive.selectionLeftMouse &&
			!visSettings->window.showMouseCoordinates) //but not done if mouse coordinates shown
		{
			//rendererOut << "mouse pressed!\n";
			//MouseSelect(window, stateMachine.mousePositionX, stateMachine.mousePositionY);
			stateMachine.selectionMouseCoordinates = state->mouseCoordinates;

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			Index itemID;
			//ItemType itemType;
			//Index mbsNumber;
			MouseSelect(window,
				(Index)stateMachine.selectionMouseCoordinates[0],
				(Index)stateMachine.selectionMouseCoordinates[1],
				itemID);
		}

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
		//check, if it was a regular mouse press without moving
		if (stateMachine.lastMousePressedX == stateMachine.mousePositionX &&
			stateMachine.lastMousePressedY == stateMachine.mousePositionY &&
			visSettings->interactive.selectionRightMouse &&
			!PyGetRendererPythonCommandLock())
		{
			//rendererOut << "mouse pressed!\n";
			//MouseSelect(window, stateMachine.mousePositionX, stateMachine.mousePositionY);
			stateMachine.selectionMouseCoordinates = state->mouseCoordinates;

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			Index itemID;
			bool success = MouseSelect(window,
				(Index)stateMachine.selectionMouseCoordinates[0],
				(Index)stateMachine.selectionMouseCoordinates[1],
				itemID);

			if (success)
			{
				ShowMessage("show item properties (see other window)", 2);
				UpdateGraphicsDataNow();
				Render(window);
				//queue process and execute as soon as possible in Python (main) thread
				PySetRendererMultiThreadedDialogs(visSettings->dialogs.multiThreadedDialogs);
				PyQueuePythonProcess(ProcessID::ShowRightMouseSelectionDialog, itemID);
				//PyQueueExecutableString(STDstring("print('+++++++++++++++++++++++++++++++++++')\n") + "print(" + strDict + ")\n");
			}
		}

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
	//EXUstd::ReleaseSemaphore(graphicsUpdateAtomicFlag);

}

//Index cnt = 0;
void GlfwRenderer::cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (PyGetRendererCallbackLock()) { return; }
	SetCallBackSignal();
	//rendererOut << "mouse cursor: x=" << xpos << ", y=" << ypos << "\n";
	stateMachine.mousePositionX = xpos;
	stateMachine.mousePositionY = ypos;

	////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//ShowMessage("cnt ="+EXUstd::ToString(cnt++), 5);
	////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


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
			float rotationFactor = visSettings->interactive.mouseMoveRotationFactor;

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

//! return true, if joystick available and updated values are available; if joystickNumber==invalidIndex, chose a joystick; 
//! if joystickNumber!=invalidIndex, it uses the fixed joystick until end of Renderer
bool GlfwRenderer::GetJoystickValues(Vector3D& position, Vector3D& rotation, Index& joystickNumber)
{
	bool initFirst = false; //if initialized first, also reset stateMachine
	if (joystickNumber == invalidIndex)
	{
		//check if joystick available
		for (Index i = 0; i <= GLFW_JOYSTICK_LAST - GLFW_JOYSTICK_1; i++)
		{
			if (glfwJoystickPresent(GLFW_JOYSTICK_1 + i))
			{
				int count;
				//const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1 + i, &count);
				glfwGetJoystickAxes(GLFW_JOYSTICK_1 + i, &count);
				if (count == 6)
				{
					initFirst = true;
					joystickNumber = i;
					ShowMessage("found 6-axis joystick with ID " + EXUstd::ToString(i)+"; using for translation/rotation input", 5);
					break;
				}
			}
		}
	}

	if (joystickNumber >= 0)
	{
		int count;
		const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1 + joystickNumber, &count);
		if (count == 6) //ignore all other joysticks!
		{
			for (Index j = 0; j < 3; j++)
			{
				position[j] = axes[j];
				rotation[j] = axes[j + 3];
			}
		}
		if (initFirst)
		{
			stateMachine.storedJoystickPosition = position;
			stateMachine.storedJoystickRotation = rotation;
		}
		return true;
	}
	return false;
}

//! read joystick values; if changed, send refresh signal for graphics
void GlfwRenderer::ProcessJoystick()
{
	if (visSettings->interactive.useJoystickInput && 
		stateMachine.mode == RendererMode::_None && //only if no other move/zoom action ongoing!
		GetJoystickValues(state->joystickPosition, state->joystickRotation, state->joystickAvailable))
	{
		Vector3D diffPos = state->joystickPosition - stateMachine.storedJoystickPosition;
		Vector3D diffRot = state->joystickRotation - stateMachine.storedJoystickRotation;
		stateMachine.storedJoystickPosition = state->joystickPosition;
		stateMachine.storedJoystickRotation = state->joystickRotation;

		//ShowMessage("joystick =" + EXUstd::ToString(diffPos) + EXUstd::ToString(diffRot));
		if (!(diffPos == 0. && diffRot == 0.))
		{
			SetCallBackSignal();
		}
		if (!(diffPos == 0.))
		{
			float fact = 2.f*state->zoom * visSettings->interactive.joystickScaleTranslation; //add more weight to translation in plane
			state->centerPoint[0] -= fact * (float)diffPos[0];
			state->centerPoint[1] += fact * (float)diffPos[1];

			state->zoom *= (1.f + visSettings->interactive.joystickScaleTranslation*(float)diffPos[2]);
			//ShowMessage("move: " + EXUstd::ToString(state->centerPoint[0]) +"," + EXUstd::ToString(state->centerPoint[1]));
		}
		if (!(diffRot == 0.))
		{
			diffRot *= visSettings->interactive.joystickScaleRotation;

			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();	//start with identity
			glRotatef((float)diffRot[0], 1.f, 0.f, 0.f); //apply "incremental" rotation around x
			glRotatef(-(float)diffRot[1], 0.f, 1.f, 0.f); //apply "incremental" rotation around y
			glRotatef(-(float)diffRot[2], 0.f, 0.f, 1.f); //apply "incremental" rotation around z
			glMultMatrixf(state->modelRotation.GetDataPointer());
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
		}
	}
}



//! zoom in to mouse position (x,y), used to render that area lateron (replacement for gluPickMatrix(...)
void GlfwRenderer::SetViewOnMouseCursor(GLdouble x, GLdouble y, GLdouble delX, GLdouble delY, GLint viewport[4])
{
	if (delX <= 0 || delY <= 0) 
	{ 
		CHECKandTHROWstring("SetViewOnMouseCursor: not allowed with delX<=0 or delY<=0");
		return; 
	}

	/* Translate and scale the picked region to the entire window */
	glTranslated((viewport[2] - 2 * (x - viewport[0])) / delX,
		(viewport[3] - 2 * (y - viewport[1])) / delY, 0);
	glScaled(viewport[2] / delX, viewport[3] / delY, 1.0);
}

//! function to evaluate selection of items, show message, return dictionary string
bool GlfwRenderer::MouseSelect(GLFWwindow* window, Index mouseX, Index mouseY, Index& itemID)
{
	//if (verboseRenderer) { std::cout << "Mouse select" << std::flush; }
	
	MouseSelectOpenGL(window,
		(Index)stateMachine.selectionMouseCoordinates[0],
		(Index)stateMachine.selectionMouseCoordinates[1],
		itemID);

	const Real timeOutHighlightItem = 0.5; //just short to exactly see object
	ItemID2IndexType(itemID, stateMachine.highlightIndex, stateMachine.highlightType, stateMachine.highlightMbsNumber);

	//if (verboseRenderer) { std::cout << "  select=" << EXUstd::ToString(itemID) << std::flush; }

	//PrintDelayed("itemID=" + EXUstd::ToString(itemID));

	if (stateMachine.highlightType != ItemType::_None && stateMachine.highlightIndex != invalidIndex)
	{
		stateMachine.highlightTimeout = EXUstd::GetTimeInSeconds() + timeOutHighlightItem; //5 seconds timeout

		STDstring itemTypeName;
		STDstring itemName;
		//STDstring itemInfo;
		bool rv = GetItemInformation(itemID, itemTypeName, itemName);// , itemInfo);

		if (rv)
		{
			ShowMessage("Selected item: " + itemTypeName +
				//"type = " + EXUstd::ToString(itemType) + 
				", index = " + EXUstd::ToString(stateMachine.highlightIndex) + " (" + itemName + ")", 0);
		}
		return true;
	}
	else
	{
		ShowMessage("no item selected", 2);
		return false;
	}
}

//! function to evaluate selection of items
void GlfwRenderer::MouseSelectOpenGL(GLFWwindow* window, Index mouseX, Index mouseY, Index& itemID)
{
	//++++++++++++++++++++++++++++++++++++++++
	//put into separate function, for Render(...)
	int width, height;

	//if (verboseRenderer) { std::cout << "  MouseSelectOpenGL" << std::flush; }
	glfwGetFramebufferSize(window, &width, &height);

	//rendererOut << "current window: width=" << width << ", height=" << height << "\n";
	state->currentWindowSize[0] = width;
	state->currentWindowSize[1] = height;

	float ratio = (float)width;
	if (height != 0) { ratio = width / (float)height; }

	//++++++++++++++++++++++++++++++++++++++++

	//if (verboseRenderer) { std::cout << "  glSelectBuffer" << std::flush; }
	const Index selectBufferSize = 10000; //size for number of objects picked at same time
	GLuint selectBuffer[selectBufferSize];
	glSelectBuffer(selectBufferSize, selectBuffer);

	//if (verboseRenderer) { std::cout << "  glRenderMode" << std::flush; }
	glRenderMode(GL_SELECT);

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	//rendererOut << "viewport=" << viewport[0] << ", " << viewport[1] << ", " << viewport[2] << ", " << viewport[3] << "\n";
	//rendererOut << "mouse=" << mouseX << ", " << mouseY << "\n";

	float backgroundColor = 0.f;
	glClearColor(backgroundColor, backgroundColor, backgroundColor, 1.0f);
	glStencilMask(~0); //make sure that all stencil bits are cleared
	//glDisable(GL_SCISSOR_TEST);
	glClearStencil(0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	//if (verboseRenderer) { std::cout << "  glMatrixMode" << std::flush; }
	//++++++++++++++++++++++++++++++++++++++++
	//from Render(...) 
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	GLdouble selectArea = 3; //3 seems to work properly; size of the select area (could be larger than 1 pixel to average)
	SetViewOnMouseCursor(mouseX, viewport[3] - mouseY, selectArea*ratio, selectArea, viewport); //add ratio to make area non-distorted?q

	//++++++++++++++++++++++++++++++++++++++++
	float zoom;
	SetProjection(width, height, ratio, zoom); //set zoom, perspective, ...; may not work for larger perspective

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	SetModelRotationTranslation();

	//glTranslated(-state->centerPoint[0], -state->centerPoint[1], 0.f);
	//glMultMatrixf(state->modelRotation.GetDataPointer());
	//++++++++++++++++++++++++++++++++++++++++

	//add one name in hierarchie and then draw scene with names
	//if (verboseRenderer) { std::cout << "  glInitNames" << std::flush; }
	glInitNames();
	//if (verboseRenderer) { std::cout << "  glPushName" << std::flush; }
	glPushName(1);

	const bool selectionMode = true;
	//if (verboseRenderer) { std::cout << "  RenderGraphicsData" << std::flush; }
	RenderGraphicsData(selectionMode); //render scene with names
	//glCallList (filledlist);

	glPopName();
	//++++++++++++++++++++++++++++++++++++++++
	//if (verboseRenderer) { std::cout << "  glPopMatrix" << std::flush; }
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	//if (verboseRenderer) { std::cout << "  glRenderMode" << std::flush; }
	Index numberOfItemsFound = glRenderMode(GL_RENDER);

	//++++++++++++++++++++++++++++++++++++++++
	//evaluate items:
	//rendererOut << "number of found items = " << numberOfItemsFound << "\n";
	//if (verboseRenderer) { std::cout << "  evaluate" << std::flush; }

	Index  itemIDnearest = 0;
	GLuint minimalDepth = 0; //clip other items that are closer
	for (Index i = 0; i < numberOfItemsFound; i++)
	{
		GLuint currentIdemID = selectBuffer[4 * i + 3];
		GLuint curdepth = selectBuffer[4 * i + 1];

		if (currentIdemID != 0 && (curdepth < minimalDepth || !itemIDnearest))
		{
			minimalDepth = curdepth;
			itemIDnearest = currentIdemID;
		}
	}

	itemID = itemIDnearest;
	//ItemID2IndexType(itemIDnearest, itemIndex, itemType, mbsNumber); //itemType==_None, if no item found
	//rendererOut << "selected item: " << itemIndex << ", type=" << itemType << "\n";
}


bool GlfwRenderer::SetupRenderer(Index verbose)
{
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//initializat and detect running renderer
	verboseRenderer = verbose;

	lastGraphicsUpdate = EXUstd::GetTimeInSeconds() - 1000.; //do some update at beginning
	lastEventUpdate = lastGraphicsUpdate;
    rendererStartTime = EXUstd::GetTimeInSeconds();
    lastTryCloseWindow = rendererStartTime - 1000; //

	//glfwCreateThread(); //does not work properly ...
	//auto th = new std::thread(GlfwRenderer::StartThread);

	globalPyRuntimeErrorFlag = false; //if previous renderer crashed, this allows to relase this error even if the old renderer is still running

	if (rendererActive)//check that renderer is not already running and that link to SystemContainer exists
	{
		PyWarning("OpenGL renderer already active");
		return false;
	}
	else if (basicVisualizationSystemContainer != nullptr) //check that renderer is not already running and that link to SystemContainer exists
	{
		PySetRendererCallbackLock(false); //reset callback lock if still set from earlier run (for safety ...)
		PySetRendererPythonCommandLock(false); //reset command callback lock if still set from earlier run (for safety ...)

		basicVisualizationSystemContainer->InitializeView(); //initializes renderState; this is done to make OpenGL zoom and maxSceneCoordinates work
		basicVisualizationSystemContainer->SetComputeMaxSceneRequest(true); //computes maxSceneCoordinates for perspective and shadow
		if (visSettings->general.autoFitScene) 
		{ 
			basicVisualizationSystemContainer->SetZoomAllRequest(true); 
			//pout << "set zoom all\n";
		}
		basicVisualizationSystemContainer->ForceQuitSimulation(false);	//reset flag if set from earlier simulations
		basicVisualizationSystemContainer->StopSimulation(false);		//reset flag if set from earlier simulations; the SC and all mbs stop flags are set true only at exit of renderer, therefore must be reset when restarting!

		rendererError = 0; 

		if (rendererThread.joinable()) //thread is still running from previous call ...
		{
			rendererThread.join();
			//rendererThread.~thread(); //this would make the thread unusable?
		}

		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		//now we startup new thread
		if (verboseRenderer) { pout << "Setup OpenGL renderer ...\n"; } //still thread-safe

		useMultiThreadedRendering = visSettings->general.useMultiThreadedRendering;
#if defined(__EXUDYN__APPLE__)
		useMultiThreadedRendering = false;
		visSettings->general.useMultiThreadedRendering = false; //make sure that this is also set in visualization settings
#endif

		if (visSettings->general.showHelpOnStartup > 0) {
			ShowMessage("press H for help on keyboard and mouse functionality", visSettings->general.showHelpOnStartup);
		}

		if (useMultiThreadedRendering)
		{
			rendererThread = std::thread(GlfwRenderer::InitCreateWindow);
			Index timeOut = visSettings->window.startupTimeout / 10;

			Index i = 0;
			while (i++ < timeOut && !(rendererActive || rendererError > 0)) //wait 5 seconds for thread to answer; usually 150ms in Release and 500ms in debug mode
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}
			if (verboseRenderer) { pout << "waited for " << i * 10 << " milliseconds \n"; }
			if (rendererActive)
			{
				if (verboseRenderer) { pout << "OpenGL renderer started!\n"; }
				UpdateGraphicsDataNow(); //in case of previous run, otherwise old state will be shown
				//not needed as called via backlink to GLFWrenderer: basicVisualizationSystemContainer->SetVisualizationIsRunning(true); //render engine runs, graphicsupdate shall be done
				return true;
			}
			else
			{
				//not needed as called via backlink to GLFWrenderer: basicVisualizationSystemContainer->SetVisualizationIsRunning(false); //render engine did not start
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
			//do all initialization for start of renderer
			GlfwRenderer::InitCreateWindow();
			if (rendererActive)
			{
				if (verboseRenderer) { pout << "Single-threaded OpenGL renderer started!\n"; }
				UpdateGraphicsDataNow(); //in case of previous run, otherwise old state will be shown
				return true;
			}
			else
			{
				if (rendererError == 1)
				{
					SysError("Start of Single-threaded OpenGL renderer failed: glfwInit() failed");
				}
				else if (rendererError == 2)
				{
					SysError("Start of Single-threaded OpenGL renderer failed: glfwCreateWindow() failed");
				}
				else { SysError("Start of Single-threaded OpenGL renderer failed"); }
				return false;
			}
		}
	}
	else
	{
		PyError("No SystemContainer has been attached to renderer (or it has been detached). Renderer cannot be started without SystemContainer.");
		return false;
	}
	return false; //not needed, but to suppress warnings
}

//! stop the renderer engine and its thread
void GlfwRenderer::StopRenderer()
{
	if (window)
	{
		//not needed as called via backlink to GLFWrenderer : basicVisualizationSystemContainer->SetVisualizationIsRunning(false); //no further WaitForUserToContinue or GraphicsDataUpdates

		stopRenderer = true;
		glfwSetWindowShouldClose(window, GL_TRUE);

		if (useMultiThreadedRendering)
		{
			Index timeOut = 100; //2020-12-09: changed to 10*100ms, because window can hang very long; visSettings->window.startupTimeout / 10;

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
				if (verboseRenderer) { outputBuffer.WriteVisualization("StopRenderer(): second thread join main thread ...\n"); }

				rendererThread.join();
				if (verboseRenderer) { outputBuffer.WriteVisualization("  ... joined\n"); }
				//not necessary: rendererThread.~thread(); //check if this is necessary/right ==> will not be called after .joint() ...
			}
		}
		else
		{
			FinishRunLoop(); //shut down
		}
	}
	else
	{
		if (useMultiThreadedRendering)
		{
			if (rendererThread.joinable()) //thread is still running from previous call ...
			{
				if (verboseRenderer) { outputBuffer.WriteVisualization("StopRenderer(): window already closed; now second thread join main thread ...\n"); }
				//pout << "join thread ...\n";
				rendererThread.join();
				if (verboseRenderer) { outputBuffer.WriteVisualization("  ... joined\n"); }
			}
		}
	}
}


void GlfwRenderer::SetContentScaling(float xScale, float yScale)
{
	float fontScaleOld = GetFontScaling();
	if (visSettings->general.useWindowsDisplayScaleFactor)
	{
		SetFontScaling(0.5f*(xScale + yScale) ); //simplified for now!
	} else {SetFontScaling(1); }

	if (GetFontScaling() != fontScaleOld)
	{
		ShowMessage("Font size adjusted to monitor scaling", 3.);
	}
}

void GlfwRenderer::SetFontScaling(float scaling)
{
    if (state != nullptr)
    {
#if defined(__EXUDYN__LINUX__)
        state->displayScaling = scaling * visSettings->general.linuxDisplayScaleFactor;
#else
        state->displayScaling = scaling;
#endif
    }
}

void GlfwRenderer::InitCreateWindow()
{
	try
	{
		//this is now running in separate thread, can only output to rendererOut.
		if (verboseRenderer) { PrintDelayed("InitCreateWindow"); }
		glfwSetErrorCallback(error_callback);
		if (!glfwInit())
		{
			if (verboseRenderer) { PrintDelayed("glfwInit failed"); }

			rendererError = 1;
			exit(EXIT_FAILURE);
		}


#ifndef __EXUDYN__APPLE__
		if (visSettings->openGL.multiSampling == 2 || visSettings->openGL.multiSampling == 4 || visSettings->openGL.multiSampling == 8 || visSettings->openGL.multiSampling == 16) //only 4 is possible right now ... otherwise no multisampling
		{
			glfwWindowHint(GLFW_SAMPLES, (int)visSettings->openGL.multiSampling); //multisampling=4, means 4 times larger buffers! but leads to smoother graphics
			glEnable(GL_MULTISAMPLE); //usually activated by default, but better to have it anyway
			if (verboseRenderer) { PrintDelayed("enable GL_MULTISAMPLE"); }
		}
#endif

		if (visSettings->window.alwaysOnTop)
		{
			glfwWindowHint(GLFW_FLOATING, GLFW_TRUE); //GLFW_FLOATING (default: GLFW_FALSE)  specifies whether the windowed mode window will be floating above other regular windows, also called topmost or always - on - top.This is intended primarily for debugging purposes and cannot be used to implement proper full screen windows.Possible values are GLFW_TRUE and GLFW_FALSE.
			if (verboseRenderer) { PrintDelayed("enable GLFW_FLOATING"); }
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
			if (useMultiThreadedRendering)
			{
				PrintDelayed("GLFWRenderer::InitCreateWindow: Render window could not be created");
				exit(EXIT_FAILURE); //for task
			}
			else
			{
				SysError("GLFWRenderer::InitCreateWindow: Render window could not be created");
				return;
			}
		}
		else
		{
			if (verboseRenderer) { PrintDelayed("glfwCreateWindow(...) successful"); }
		}

		//allow for very small windows, but never get 0 ...; x-size anyway limited due to windows buttons
		glfwSetWindowSizeLimits(window, 2, 2, maxWidth, maxHeight);
        if (!visSettings->window.limitWindowToScreenSize)
        {
            glfwSetWindowSize(window, sizex, sizey); //this call ensures that window size is really the requested size; tested on windows
            //outputBuffer.WriteVisualization("window size=" + EXUstd::ToString(sizex) + ", " + EXUstd::ToString(sizey));
        }

		if (verboseRenderer) { PrintDelayed("glfwSetWindowSizeLimits(...) successful"); }

		//+++++++++++++++++++++++++++++++++
		//set keyback functions
		glfwSetKeyCallback(window, key_callback);			//keyboard input
		glfwSetScrollCallback(window, scroll_callback);		//mouse wheel input
		glfwSetMouseButtonCallback(window, mouse_button_callback);
		glfwSetCursorPosCallback(window, cursor_position_callback);
		if (verboseRenderer) { PrintDelayed("mouse and key callbacks successful"); }

		glfwSetWindowCloseCallback(window, window_close_callback);
		glfwSetWindowRefreshCallback(window, Render);
#if !defined(__EXUDYN__LINUX__)
		glfwSetWindowContentScaleCallback(window, window_content_scale_callback);
#endif
		if (verboseRenderer) { PrintDelayed("window callbacks successful"); }

		glfwMakeContextCurrent(window);
		if (verboseRenderer) { PrintDelayed("glfwMakeContextCurrent(...) successful"); }

		//+++++++++++++++++++++++++++++++++
		//joystick
		state->joystickAvailable = invalidIndex; //this causes to search for new joystick and, if fourn, initialize stateMachine!
		ResetStateMachine();

#ifdef __EXUDYN_USE_OPENVR
		if (verboseRenderer) { PrintDelayed("Initialize OpenVR"); }
		if (visSettings->interactive.openVR.enable) 
		{ 
			glfwOpenVRinterface.SetLogLevel(visSettings->interactive.openVR.logLevel);
			state->openVRstate.isActivated = glfwOpenVRinterface.InitOpenVR(&glfwRenderer);
		}
#endif

		//+++++++++++++++++
		//initialize opengl
		glClearDepth(1.0f);
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_NORMALIZE);

		//std::cout << "OpenGL version=" << glGetString(GL_VERSION) << "\n";

		//+++++++++++++++++
		//determine the windows scale; TODO: add callback to redraw if monitor is changed: glfwSetWindowContentScaleCallback(...)
		float xWindowScale = 1;
		float yWindowScale = 1;
#if !defined(__EXUDYN__LINUX__) //glfwGetWindowContentScale() crashes on Ubuntu18.04 and 20.04 compilation
		glfwGetWindowContentScale(window, &xWindowScale, &yWindowScale);
#endif
		SetContentScaling(xWindowScale, yWindowScale); 
		guint fontSize = (guint)(visSettings->general.textSize * GetFontScaling()); //use this size for fonts throughout
		InitFontBitmap(fontSize); //fontSize only used in old bitmap mode!
		if (verboseRenderer) { PrintDelayed("InitFontBitmap(...) successful"); }

		InitGLlists();
		if (verboseRenderer) { PrintDelayed("InitGLlists(...) successful"); }

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

		//+++++++++++++++++++++++++++++++++
		//do this just before RunLoop, all initialization finished ...
		//DELETE: firstRun = 0; //zoom all on startup of window
		rendererActive = true; //this is still threadsafe, because main thread waits for this signal!
		//+++++++++++++++++++++++++++++++++

	}
	catch (const std::exception& e) // reference to the base of a polymorphic object
	{
		outputBuffer.WriteVisualization(STDstring("Exception in GLFW::InitCreateWindow:\n") + e.what() + "\n");
		return;
	}
	catch (...)
	{
		outputBuffer.WriteVisualization("Unknown Exception in GLFW::InitCreateWindow\n");
		return;
	}

	//exceptions starting from here are catched in RunLoop before FinishRunLoop (which stops thread...)
	if (useMultiThreadedRendering)
	{
		if (verboseRenderer) { PrintDelayed("InitCreateWindow finished: Starting renderer loop"); }
		RunLoop();
	}
	else
	{
		if (verboseRenderer) { PrintDelayed("InitCreateWindow finished: Ready to update window using DoRendererIdleTasks(...)"); }
	}
}

void GlfwRenderer::RunLoop()
{
	try
	{
		while (rendererActive && !glfwWindowShouldClose(window) &&
			!stopRenderer && !globalPyRuntimeErrorFlag)
		{
			DoRendererTasks();
		}
	}
	catch (const std::exception& e) // reference to the base of a polymorphic object
	{
		outputBuffer.WriteVisualization(STDstring("Exception in Renderer loop:\n") + e.what() + "\nclosing renderer.\n");
		//std::cout << "** Exception in Renderer loop:\n" << e.what() << "\nclosing renderer.\n";
	}
	catch (...)
	{

		outputBuffer.WriteVisualization("Unknown Exception in Renderer loop\nclosing renderer.\n");
		//std::cout << "** Unknown Exception in Renderer loop\nclosing renderer.\n";
	}
	EXUstd::ReleaseSemaphore(renderFunctionRunning); //in case that it crashed after lock
	EXUstd::ReleaseSemaphore(showMessageSemaphore); //in case that it crashed after lock
	for (auto data : *graphicsDataList) //lock prevents from closing renderer consistently!
	{
		data->ClearLock();
	}

	FinishRunLoop();
}

void GlfwRenderer::DoRendererTasks(bool graphicsUpdateAndRender)
{
	Real updateInterval = (Real)(visSettings->general.graphicsUpdateInterval);
	Real time = EXUstd::GetTimeInSeconds();

	if (!useMultiThreadedRendering) //do this before rendering ...
	{
		if (time >= lastEventUpdate + 0.01) //should be very responsive - 100Hz is ok
		{
			glfwPollEvents(); //do not wait, just do tasks if they are there
			lastEventUpdate = time;
			PyProcessExecuteQueue(); //if still some elements open in queue; MAY ONLY BE DONE IN SINGLE-THREADED MODE
			ProcessJoystick();
        }
	}


	if (useMultiThreadedRendering || 
		(time >= lastGraphicsUpdate + updateInterval) || 
		GetCallBackSignal() ||
		graphicsUpdateAndRender)
	{
		basicVisualizationSystemContainer->UpdateGraphicsData();
		bool maxSceneComputed = false;
		if (basicVisualizationSystemContainer->GetComputeMaxSceneRequest()) 
		{ 
			ComputeMaxSceneSize(state->maxSceneSize, state->centerPoint); 
			maxSceneComputed = true;
			basicVisualizationSystemContainer->SetComputeMaxSceneRequest(false);

		}
		if (basicVisualizationSystemContainer->GetAndResetZoomAllRequest()) {
			ZoomAll(false, !maxSceneComputed, false); 
		}
		Render(window);
		SaveImage(); //in case of flag, save frame to image file
#ifdef __EXUDYN_USE_OPENVR
		if (glfwOpenVRinterface.IsActivated())
		{
			glfwOpenVRinterface.RenderAndUpdateDevices();
		}
#endif
		lastGraphicsUpdate = time;
		SetCallBackSignal(false);
	}

	if (useMultiThreadedRendering)
	{
		glfwWaitEventsTimeout((double)updateInterval); //wait x seconds for next event
		ProcessJoystick();
#ifdef __EXUDYN_USE_OPENVR
        if (glfwOpenVRinterface.IsActivated())
        {
            //in future, this should be a GlfwRenderer function, which transmits data to renderState
            glfwOpenVRinterface.GetState(state->openVRstate);
            //OpenVRparameters p; //currently without any functionality
            //glfwOpenVRinterface.SetDataAndParameters(p);
        }
#endif
    }

}

void GlfwRenderer::FinishRunLoop()
{
	if (verboseRenderer) { outputBuffer.WriteVisualization("Finish renderer loop ...\n"); }

	if (globalPyRuntimeErrorFlag)
	{
		PrintDelayed("render window stopped because of error");
	}
	if (basicVisualizationSystemContainer)
	{
		basicVisualizationSystemContainer->StopSimulation(); //if user waits for termination of render engine, it tells that window is closed
	}

	if (window)
	{
#ifdef __EXUDYN_USE_OPENVR
        glfwOpenVRinterface.ShutDown();
#endif
        glfwDestroyWindow(window); //should be called from main thread, but also works this way!
		window = nullptr;
	}
	rendererActive = false; //for new startup of renderer
	stopRenderer = false;	//if stopped by user
	glfwTerminate();		//should be called from main thread, but also works this way!

	DeleteFonts();
	if (verboseRenderer) { outputBuffer.WriteVisualization("  ... renderer loop finished\n"); }
}

//! run renderer idle for certain amount of time; use this for single-threaded, interactive animations; waitSeconds==-1 waits forever
void GlfwRenderer::DoRendererIdleTasks(Real waitSeconds, bool graphicsUpdateAndRender)
{
	Real time = EXUstd::GetTimeInSeconds();
	bool continueTask = true;
	if (IsGlfwInitAndRendererActive()) //in case that renderer is not running, the following should not be processed (MacOS!)
	{
		while (rendererActive &&
			!glfwWindowShouldClose(window) &&
			!stopRenderer &&
			!globalPyRuntimeErrorFlag &&
			continueTask)
		{
			if (!useMultiThreadedRendering)
			{
				DoRendererTasks(graphicsUpdateAndRender);
			}
			else
			{
				basicVisualizationSystemContainer->DoIdleOperations(); //this calls the Python functions, which is ok, because DoRendererIdleTasks() called from Python!
			}

            if (waitSeconds != -1. && EXUstd::GetTimeInSeconds() > time + waitSeconds)
			{
				continueTask = false;
			}
			else
			{
				//wait small amount of time, not fully blocking CPU ==> only done, if called directly with DoRendererIdleTasks(100)
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}
		}

		if (!(rendererActive &&
			!glfwWindowShouldClose(window) &&
			!stopRenderer &&
			!globalPyRuntimeErrorFlag))
		{
			FinishRunLoop();
		}
	}
}

//load GL_PROJECTION and set according to zoom, perspective, etc.
void GlfwRenderer::SetProjection(int width, int height, float ratio, float& zoom)
{
	if (visSettings->interactive.lockModelView)
	{
		zoom = visSettings->openGL.initialZoom;
		state->zoom = zoom;
	}
	else
	{
		zoom = state->zoom;
	}

	const Matrix4DF& P = state->projectionMatrix;
	if (P(0, 0) == 1.f && P(1, 1) == 1.f && P(2, 2) == 1.f && P(3, 3) == 1.f) //in this case, no projection has been provided
	{
		if (visSettings->openGL.perspective <= 0)
		{
			glOrtho(-ratio * zoom, ratio*zoom, -zoom, zoom, -zFactor * 2.*state->maxSceneSize, zFactor * 2.*state->maxSceneSize); //https: //www.khronos.org/opengl/wiki/Viewing_and_Transformations#How_do_I_implement_a_zoom_operation.3F
			//void glOrtho(GLdouble left,GLdouble right,GLdouble bottom,GLdouble top,GLdouble nearVal,GLdouble farVal);
		}
		else
		{
			//visSettings->openGL.perspective
			float fact = 1.f / (float)visSettings->openGL.perspective;
			float factZoom = 0.5f; //correction, to show same zoom level as orthographic projection

			float zNear = state->maxSceneSize*fact;
			float zFar = state->maxSceneSize * (4.f*fact + 2.f);
			float right = ratio * zoom*factZoom;
			float left = -right;
			float top = zoom * factZoom;
			float bottom = -zoom * factZoom;
			glFrustum(left, right, bottom, top, zNear, zFar);
			glTranslatef(0.f, 0.f, -2 * fact * state->maxSceneSize);
		}
	}
	else //openVR
	{
		if (state->projectionInfo == 0) //for companion window
		{
			glOrtho(-ratio * zoom, ratio*zoom, -zoom, zoom, -zFactor * 2.*state->maxSceneSize, zFactor * 2.*state->maxSceneSize); //https: //www.khronos.org/opengl/wiki/Viewing_and_Transformations#How_do_I_implement_a_zoom_operation.3F
			glMultMatrixf(state->projectionMatrix.GetDataPointer());
		}
		else if (state->projectionInfo == 1) //load matrix
		{
			glLoadMatrixf(state->projectionMatrix.GetDataPointer());
		}
		else if (state->projectionInfo == 2) //apply matrix after glOrtho
		{
			//additional projection has been provided and is added after glOrtho
			glOrtho(-ratio * zoom, ratio*zoom, -zoom, zoom, -zFactor * 2.*state->maxSceneSize, zFactor * 2.*state->maxSceneSize); //https: //www.khronos.org/opengl/wiki/Viewing_and_Transformations#How_do_I_implement_a_zoom_operation.3F
			//glOrtho(-ratio , ratio, -1, 1, 0.1, 30); //https: //www.khronos.org/opengl/wiki/Viewing_and_Transformations#How_do_I_implement_a_zoom_operation.3F
			//glLoadMatrixf(state->projectionMatrix.GetDataPointer());
			//glTranslatef(0.f, 0.f, -state->maxSceneSize);
			glMultMatrixf(state->projectionMatrix.GetDataPointer());
		}

	}
}

//! set model view rotation and translation, unified for Render and mouse select
void GlfwRenderer::SetModelRotationTranslation()
{
	//++++++++++++++++++++++++++++++++++++++++++++++++++++
	//model rotation and translation, include rotation center point
	Matrix4DF A;
	Matrix3DF rotationMV;
	Float3 translationMV;
	if (!visSettings->interactive.lockModelView)
	{
		A = state->modelRotation; //copy, do not update model rotation, which is still modifiable
		rotationMV = EXUmath::Matrix4DtoMatrix3D(A);
		translationMV = state->rotationCenterPoint * rotationMV + state->centerPoint;
	}
	else
	{
		A = state->modelRotation; //copy, do not update model rotation, which is still modifiable
		for (Index i = 0; i < 3; i++) //overwrite initial rotation
		{
			for (Index j = 0; j < 3; j++)
			{
				A(i, j) = visSettings->openGL.initialModelRotation[i][j];
			}
		}
		rotationMV = EXUmath::Matrix4DtoMatrix3D(A);
		translationMV = visSettings->openGL.initialCenterPoint;
	}

	//update center point if tracked by marker
	if (visSettings->interactive.trackMarker != -1)
	{
		Vector3D markerPosition;
		Matrix3D markerOrientation;
		bool hasPosition;
		bool hasOrientation;
		basicVisualizationSystemContainer->GetMarkerPositionOrientation(visSettings->interactive.trackMarker,
			visSettings->interactive.trackMarkerMbsNumber, markerPosition, markerOrientation, hasPosition, hasOrientation);


		if (hasOrientation && visSettings->interactive.trackMarkerOrientation.SumAbs() == 3.f)
		{ //track orientation as well!
			Matrix3DF markerOrientation3DF;
			markerOrientation3DF.CopyFrom(markerOrientation);
			rotationMV = markerOrientation3DF * rotationMV; //superimposed initial rotation

			//map rotation matrix to part of 16 components in A
			for (Index i = 0; i < 3; i++)
			{
				for (Index j = 0; j < 3; j++)
				{
					A(i, j) = rotationMV(i, j);
				}
			}
		}
		if (hasPosition)
		{  //track position
			markerPosition[0] *= (Real)visSettings->interactive.trackMarkerPosition[0];
			markerPosition[1] *= (Real)visSettings->interactive.trackMarkerPosition[1];
			markerPosition[2] *= (Real)visSettings->interactive.trackMarkerPosition[2];

			Float3 markerPosition3DF;
			markerPosition3DF.CopyFrom(markerPosition);
			translationMV += markerPosition3DF * rotationMV;
		}

	}

	glTranslatef(-translationMV[0], -translationMV[1], 0.f);
	//glTranslatef(0.f,0.f,-5.f); //hack openvr

	//glMultMatrixf(state->modelRotation.GetDataPointer()); //OLD
	glMultMatrixf(A.GetDataPointer());
}

Real phiact = 0;
void GlfwRenderer::Render(GLFWwindow* window) //GLFWwindow* needed in argument, because of glfwSetWindowRefreshCallback
{
	if (PyGetRendererCallbackLock()) { return; }
	EXUstd::WaitAndLockSemaphore(renderFunctionRunning); //lock Render(...) function, no second call possible

	int width, height;
	GetWindowSize(width, height);
	//rendererOut << "current window: width=" << width << ", height=" << height << "\n";

	float ratio, zoom;
	Render3Dobjects(width, height, ratio, zoom);
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	float fontSize = visSettings->general.textSize * GetFontScaling(); //use this size for fonts throughout

	//do this always, e.g. in openVR case or if projection is modified:
	//for texts, axes, etc.: draw without perspective
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-ratio * zoom, ratio*zoom, -zoom, zoom, -zFactor * 2.*state->maxSceneSize, zFactor * 2.*state->maxSceneSize); //https: //www.khronos.org/opengl/wiki/Viewing_and_Transformations#How_do_I_implement_a_zoom_operation.3F

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
		SetGLdepthMask(GL_FALSE); //draw system information and coordinate system always in front

		//float scale = 2.f*fontSize*zoom / ((float)height);
		float hOff = 0.95f*(float)zFactor * 2.f*state->maxSceneSize; //draw in front; NEEDED since SetGLdepthMask(GL_FALSE) ?


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
			if (visSettings->general.renderWindowString.size() != 0)
			{
				message = visSettings->general.renderWindowString + '\n' + message;
			}
			for (char c : message)
			{
				if (c == '\n') { computationMessageNumberOfLines++; }
			}
			DrawString(message.c_str(), fontSize, poff2, textColor);

			//+++++++++++++++++++
			//print version:
			Float2 pInfo3 = PixelToVertexCoordinates((float)(width-fontSize*fontSmallFactor*15), 5.f); //fixed position, very bottom right window position
			Float3 poff3({ pInfo3[0], pInfo3[1], hOff });
			DrawString((STDstring("version ")+EXUstd::exudynVersion).c_str(), fontSize*fontSmallFactor, poff3, textColor);
		}

		Float2 pStatus = PixelToVertexCoordinates((float)textIndentPixels, 5); //fixed position, very bottom left window position
		Float3 poff({ pStatus[0], pStatus[1], hOff });

		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		if (stateMachine.rendererMessage.size() != 0)
		{
			DrawString(stateMachine.rendererMessage.c_str(), fontSize*fontSmallFactor, poff, textColor);
			if (stateMachine.renderMessageTimeout != 0. && stateMachine.renderMessageTimeout < EXUstd::GetTimeInSeconds()) 
			{ 
				stateMachine.rendererMessage = ""; 
			}
		}
		else if (visSettings->window.showMouseCoordinates)
		{
			//Vector2D mp = state->mouseCoordinates; //not showing mouse coordinates
			//Real xpos = mp[0];
			//Real ypos = mp[1];

			float height = (float)state->currentWindowSize[1];
			float factor = 2.f*state->zoom / height;

			//Vector2D ploc = factor * Vector2D({ xpos - 0.5*state->currentWindowSize[0], -1.*(ypos - 0.5*state->currentWindowSize[1]) });
			Vector2D cp({(double)state->centerPoint[0], (double)state->centerPoint[1] });

			Vector2D lastPressedCoords = factor * Vector2D({ stateMachine.lastMousePressedX - 0.5*state->currentWindowSize[0],
				-1.*(stateMachine.lastMousePressedY - 0.5*state->currentWindowSize[1]) }) + cp;

			const Index nCharMax = 24;
			Index precision = EXUstd::Maximum(0, EXUstd::Minimum(visSettings->general.rendererPrecision, 16));
			char glx[nCharMax];
			char gly[nCharMax];
			snprintf(glx, nCharMax, "%7.*g", precision, state->openGLcoordinates[0]);
			snprintf(gly, nCharMax, "%7.*g", precision, state->openGLcoordinates[1]);
			char lpx[nCharMax];
			char lpy[nCharMax];
			snprintf(lpx, nCharMax, "%7.*g", precision, lastPressedCoords[0]);
			snprintf(lpy, nCharMax, "%7.*g", precision, lastPressedCoords[1]);
			char dist[nCharMax];
			snprintf(dist,nCharMax, "%7.*g", precision, (state->openGLcoordinates - lastPressedCoords).GetL2Norm());

			STDstring mouseStr = STDstring("mouse=(") + glx + "," + gly + ")" +
				", last=(" + lpx + "," + lpy + "), dist=" + dist;
				//", cp=(" + EXUstd::ToString(cp[0]) + "," + EXUstd::ToString(cp[1]) + ")"+
				//", ploc=(" + EXUstd::ToString(ploc[0]) + "," + EXUstd::ToString(ploc[1]) + ")";

			//Float3 poff({ 0.f*zoom,-1.88f*zoom, hOff }); //old, using vertex coordinates
			DrawString(mouseStr.c_str(), fontSize*fontSmallFactor, poff, textColor);
		} 
        SetGLdepthMask(GL_TRUE); //draw lines always in front
	}

	if (visSettings->contour.showColorBar && visSettings->contour.outputVariable != OutputVariableType::_None) //draw coordinate system
	{
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		//float factor = 0.35f*zoom * 2.6f;
		//glTranslated(-factor * ratio*1.05, factor, 0.f); //old: now use pixel coordinates

        SetGLdepthMask(GL_FALSE); //draw lines always in front
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
			if (graphicsDataList->NumberOfItems() > 1) 
			{
				ShowMessage("WARNING: contour plot color bar only works for one single system");
			}

			minVal = graphicsDataList->GetItem(0)->GetContourCurrentMinValue();
			maxVal = graphicsDataList->GetItem(0)->GetContourCurrentMaxValue();
		}
		//DrawString(basicVisualizationSystemContainer->GetComputationMessage().c_str(), scale, poff, textColor);
		STDstring contourStr = STDstring("contour plot: ")+GetOutputVariableTypeString(visSettings->contour.outputVariable) +
			"\ncomponent=" + EXUstd::ToString(visSettings->contour.outputVariableComponent) +
			"\nmin=" + EXUstd::Num2String(minVal, visSettings->contour.colorBarPrecision) + ",max=" + EXUstd::Num2String(maxVal, visSettings->contour.colorBarPrecision);
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

			//const Index nCharMax = 24;
			//char str[nCharMax];
			//std::snprintf(str, nCharMax, "% .*g", visSettings->contour.colorBarPrecision, value);
			DrawString(EXUstd::Num2String(value, visSettings->contour.colorBarPrecision).c_str(), fontSize*fontSmallFactor, p0 + Float3({ 1.2f*sizeX,-0.8f*sizeY,0}), textColor);

			p0 += Float3({ 0.f,-sizeY,0.f });

		}
        SetGLdepthMask(GL_TRUE);
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

		Float2 pPix = PixelToVertexCoordinates(10 + visSettings->general.coordinateSystemSize*fontSize, 
			10 + (visSettings->general.coordinateSystemSize+0.f)*fontSize); //+1.f because of possible status line at bottom
		glTranslated(pPix[0], pPix[1], 0.f);

		//glTranslated(factor*ratio, factor, 0.f);
		if (!visSettings->interactive.lockModelView)
		{
			glMultMatrixf(state->modelRotation.GetDataPointer());
		}
        SetGLdepthMask(GL_FALSE);

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

		Matrix4DF matTp;
		if (!visSettings->interactive.lockModelView)
		{
			matTp = state->modelRotation.GetTransposed();
		}
		else
		{
			matTp.SetScalarMatrix(4, 1.f);
		}
		//Matrix4DF m = state->modelRotation;
		//Float16 matTp({m[0],m[4],m[ 8],m[12],
		//			   m[1],m[5],m[ 9],m[13],
		//			   m[2],m[6],m[10],m[14],
		//			   m[3],m[7],m[11],m[15]});

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

        SetGLdepthMask(GL_TRUE);
	}



	glfwSwapBuffers(window);

	//++++++++++++++++++++++++++++++++++++++++++
	//renderFunctionRunning.clear(std::memory_order_release); //clear PostProcessData
	EXUstd::ReleaseSemaphore(renderFunctionRunning);

}

void GlfwRenderer::Render3Dobjects(int screenWidth, int screenHeight, float& screenRatio, float& zoom)
{
	state->currentWindowSize[0] = screenWidth;
	state->currentWindowSize[1] = screenHeight;

	glViewport(0, 0, screenWidth, screenHeight);

	Float4 bg = visSettings->general.backgroundColor;
	glClearColor(bg[0], bg[1], bg[2], bg[3]); //(float red, float green, float blue, float alpha);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glStencilMask(~0); //make sure that all stencil bits are cleared
	glClearStencil(0);

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//main render process
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	screenRatio = (float)screenWidth;
	if (screenHeight != 0) { screenRatio = screenWidth / (float)screenHeight; }
	SetProjection(screenWidth, screenHeight, screenRatio, zoom); //set zoom, perspective, ...

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	AddGradientBackground(zoom, screenRatio);
	//put here; light fixed relative to camera:
	SetGLLights(); //moved here 2020-12-05; light should now be rotation independent!

	SetModelRotationTranslation();

    RenderSensorTraces();
    RenderGraphicsData();




}

void GlfwRenderer::AddGradientBackground(float zoom , float ratio)
{
	//create special gradient background
	if (visSettings->general.useGradientBackground)
	{
		Float4 bg = visSettings->general.backgroundColor;
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

#ifdef GlfwRendererUsePNG
		bool pngAvailable = true;
#else
		bool pngAvailable = false;
#endif
		if (visSettings->exportImages.saveImageFormat == "PNG" && pngAvailable)
		{
			filename += ".png"; //image format ending
		}
		else if (visSettings->exportImages.saveImageFormat == "TXT")
		{
			filename += ".txt"; //this is the (internal) text format; used then to postprocess in Python
		}
		else if (visSettings->exportImages.saveImageFormat == "TGA" || !pngAvailable)
		{
			filename += ".tga"; //image format ending
		}
		else
		{
			PrintDelayed("SaveImage ERROR: illegal format; check documentation for exportImages; no file written");
			//SaveSceneToFile will do nothing
		}

		SaveSceneToFile(filename);

		basicVisualizationSystemContainer->SaveImageFinished();
	}
}

void GlfwRenderer::SaveSceneToFile(const STDstring& filename)
{
#ifdef GlfwRendererUsePNG
	bool pngAvailable = true;
#else
	bool pngAvailable = false;
#endif

	if (visSettings->exportImages.saveImageFormat == "PNG" && pngAvailable)
	{
#ifdef GlfwRendererUsePNG
		Index windowWidth = state->currentWindowSize[0]; //this is the size at which the renderer created buffer last time ...
		Index windowHeight = state->currentWindowSize[1];

		Index widthAlignment = visSettings->exportImages.widthAlignment; //width widthAlignment: 1,2,4 or 8
		Index heightAlignment = visSettings->exportImages.heightAlignment; //width widthAlignment: 1,2,4 or 8
		if (widthAlignment != 1 && widthAlignment != 2 && widthAlignment != 4 && widthAlignment != 8)
		{
			widthAlignment = 4;
			PrintDelayed("SaveImage ERROR: exportImages.widthAlignment illegal: must be 1, 2, 4 or 8; defaulting to 4");
		}
		if (heightAlignment != 1 && heightAlignment != 2 && heightAlignment != 4 && heightAlignment != 8)
		{
			heightAlignment = 2;
			PrintDelayed("SaveImage ERROR: exportImages.heightAlignment illegal: must be 1, 2, 4 or 8; defaulting to 2");
		}

		windowWidth = widthAlignment * (Index)(windowWidth / widthAlignment); //make multiple of 4 to align with most animation converter ...

		Index nrChannels = 3;
		const Index strideAlignment = 1; //safer to use 1; otherwise, uncomment line below next! seems not to affect performance!
		Index stride = nrChannels * windowWidth; //must be div by strideAlignment!
		//stride += (stride % strideAlignment) ? (strideAlignment - stride % strideAlignment) : 0; 

		Index numberOfPixels = windowHeight * stride;
		ResizableArray<char> pixelBuffer(numberOfPixels);
		pixelBuffer.SetNumberOfItems(numberOfPixels);


		glPixelStorei(GL_PACK_ALIGNMENT, strideAlignment);
		glReadBuffer(GL_FRONT);
		glReadPixels(0, 0, (GLsizei)windowWidth, (GLsizei)windowHeight, GL_RGB, GL_UNSIGNED_BYTE, pixelBuffer.GetDataPointer());
		//glReadPixels(0, 0, (GLsizei)windowWidth, (GLsizei)windowHeight, GL_BGR_EXT, GL_UNSIGNED_BYTE, pixelBuffer.GetDataPointer());

		ResizableArray<char> pixelBufferFlip(numberOfPixels);
		pixelBufferFlip.SetNumberOfItems(numberOfPixels);

		//FLIP
		//not available in GLFW:
		//stbi_flip_vertically_on_write(true); //as otherwise would be upside-down!
		for (Index i = 0; i < windowHeight; i++)
		{
			for (Index j = 0; j < stride; j++)
			{
				pixelBufferFlip[(windowHeight - i - 1)*stride + j] = pixelBuffer[i*stride + j];
			}
		}

		std::ofstream imageFile;
		CheckPathAndCreateDirectories(filename);

		windowHeight = heightAlignment * (Index)(windowHeight / heightAlignment);
		stbi_write_png(filename.c_str(), windowWidth, windowHeight, nrChannels, pixelBufferFlip.GetDataPointer(), stride);
#endif
	}
	else if (visSettings->exportImages.saveImageFormat == "TXT")
	{
		//export text
		std::ofstream imageFile;

		std::ios_base::openmode fileMode = std::ofstream::out; //int does not work in linux!

		//if (solutionSettings.binarySolutionFile) { fileMode = std::ofstream::binary; } //no append right now!

		//if (solutionSettings.appendToFile) { file.solutionFile.open(solutionFileName, std::ofstream::app); }
		//else { file.solutionFile.open(solutionFileName, std::ofstream::out); }
		bool checkPath = CheckPathAndCreateDirectories(filename);

		if (checkPath)
		{
			imageFile.open(filename, fileMode);
		}

		if (!imageFile.is_open()) //failed to open file ...  e.g. invalid file name
		{
			SysError(STDstring("failed to open image file '") + filename + "'; check path and file name");
		}
		imageFile.precision(8); //more accuracy is not available from float values!
		imageFile << "#Exudyn text image export file\n";
		imageFile << "# export of lines, triangles, texts, etc.\n";
		imageFile << "# \n";

		if (visSettings->exportImages.saveImageAsTextLines)
		{
			imageFile << "# SECTION LINES (consisting of X0, Y0, Z0, X1, Y1, Z1, ...  coordinates for 3D line points)\n";
			//circles are currently transformed into lines
			for (auto data : *graphicsDataList)
			{
				for (const GLCircleXY& item : data->glCirclesXY)
				{
					bool isFirst = true;
					imageFile << "#COLOR\n";
					imageFile << item.color[0] << ", " << item.color[1] << ", " << item.color[2] << ", " << item.color[3] << "\n";

					imageFile << "#LINE\n";
					const Float3& pItem = item.point;
					float r = item.radius;

					float nSeg = (float)item.numberOfSegments;
					if (nSeg == 0.f) { nSeg = (float)visSettings->general.circleTiling; }

					//for (float i = 0; i <= nSeg; i += 2.f*EXUstd::pi_f / nSeg)
					for (float i = 0; i <= 2.f*EXUstd::pi_f + 1e-5; i += 2.f*EXUstd::pi_f / nSeg)
					{
						Float3 p({ pItem[0] + r * sin(i), pItem[1] + r * cos(i), pItem[2] });
						if (!isFirst) { imageFile << ", "; }
						else { isFirst = false; }
						imageFile << p[0] << ", " << p[1] << ", " << p[2];
					}
					if (!isFirst) { imageFile << "\n"; }
				}

				//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
				//DRAW LINES
				for (const GLLine& item : data->glLines)
				{
					imageFile << "#COLOR\n";
					imageFile << item.color1[0] << ", " << item.color1[1] << ", " << item.color1[2] << ", " << item.color1[3] << "\n";
					//second color item.color2 ignored!

					imageFile << "#LINE\n";
					imageFile << item.point1[0] << ", " << item.point1[1] << ", " << item.point1[2] << ", ";
					imageFile << item.point2[0] << ", " << item.point2[1] << ", " << item.point2[2] << "\n";
				}
			}
		}

		if (visSettings->exportImages.saveImageAsTextTriangles)
		{
			imageFile << "# SECTION TRIANGLES (consisting of X0, Y0, Z0, X1, Y1, Z1, X2, Y2, Z2  coordinates for 3D triangle points)\n";
			for (auto data : *graphicsDataList)
			{
				for (const GLTriangle& trig : data->glTriangles)
				{ //draw lines
					imageFile << "#COLOR\n";
					const Float4& color = trig.colors[0]; //other colors ignored!
					imageFile << color[0] << ", " << color[1] << ", " << color[2] << ", " << color[3] << "\n";

					imageFile << "#TRIANGLE\n";
					for (Index i = 0; i < 3; i++)
					{
						Index j = i + 1;
						if (j >= 3) { j = 0; }
						const Float3& p0 = trig.points[i];
						//const Float3& p1 = trig.points[j];
						imageFile << p0[0] << ", " << p0[1] << ", " << p0[2];
						if (i != 2) { imageFile << ", "; }
						else { imageFile << "\n"; }
					}
				}
			}
		}
		if (visSettings->exportImages.saveImageAsTextTexts)
		{
			PrintDelayed("SageImage: Text export not yet implemented!");
		}
		imageFile << "#END\n"; //for safety add file end

		//FINALLY: close
		imageFile.close();
	}
	else if (visSettings->exportImages.saveImageFormat == "TGA" || !pngAvailable) //for all remaining scenarios
	{
		Index windowWidth = state->currentWindowSize[0];
		Index windowHeight = state->currentWindowSize[1];
		windowWidth = 4 * (int)(windowWidth / 4); //make multiple of 4 to align with most animation converter ...

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
			//not thread/Python safe: PyWarning(STDstring("GlfwRenderer::SaveSceneToFile: Failed to open image file '") + filename + "'");
			PrintDelayed("GlfwRenderer::SaveSceneToFile: Failed to open image file <" + filename + ">");
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
	//else : ignored
}

//! Render particulary the text of multibody system; selectionMode==true adds names
void GlfwRenderer::RenderGraphicsDataText(GraphicsData* data, Index lastItemID, bool highlight, Index highlightID, Float4 highlightColor2, Float4 otherColor2, bool selectionMode)
{
    if (visSettings->openGL.lineSmooth) { glDisable(GL_LINE_SMOOTH); }

    float textheight = visSettings->general.textSize;
    float scaleFactor = 2.f * state->zoom / ((float)state->currentWindowSize[1]); //factor, which gives approximately 1pt textsize

    //Float16 m = state->modelRotation;
    //Float16 matTp({ m[0],m[4],m[8],m[12], //transpose of modelRotation
    //               m[1],m[5],m[9],m[13],
    //               m[2],m[6],m[10],m[14],
    //               m[3],m[7],m[11],m[15] });

	Matrix4DF matTp = state->modelRotation.GetTransposed();

	float textFontSize;
    //SetGLdepthMask(GL_FALSE);
    float offz = 0.f; //positive values make it more visible than other objects!
    if (visSettings->general.textAlwaysInFront)
    {
        if (visSettings->openGL.perspective > 0)
        {
            offz = visSettings->general.textOffsetFactor*state->maxSceneSize; //larger value does not work for perspective!
            //offz = 1.05f* (1.f / (float)visSettings->openGL.perspective * state->maxSceneSize); //the bracket term is the near plane which we need to surpass
        }
        else
        {
            offz = 0.95f*(float)zFactor * 2.f*state->maxSceneSize;
        }
    }
    else
    {
        offz = visSettings->general.textOffsetFactor*state->maxSceneSize; //is always positive
    }
    bool transparent = !visSettings->general.textHasBackground;

    for (const GLText& t : data->glTexts)
    {
        if (selectionMode) { if (t.itemID != lastItemID) { glLoadName(t.itemID); lastItemID = t.itemID; } }
        //delete: float scale = textheight * scaleFactor;
        //delete: if (t.size != 0.f) { scale = t.size * scaleFactor; }
        textFontSize = textheight * GetFontScaling();
        if (t.size != 0.f) { textFontSize = t.size * GetFontScaling(); }

        float offx = t.offsetX * scaleFactor * textFontSize;
        float offy = t.offsetY * scaleFactor * textFontSize;
        //draw strings without applying the rotation:
        glPushMatrix(); //store current matrix -> before rotation
        glTranslated(t.point[0], t.point[1], t.point[2]);
        glMultMatrixf(matTp.GetDataPointer());

        if (!highlight)
        {
            DrawString(t.text, textFontSize, Float3({ offx,offy,offz }), t.color, transparent);
        }
        else
        {
            Float4 color = otherColor2;
            if (t.itemID == highlightID) { color = highlightColor2; }

            DrawString(t.text, textFontSize, Float3({ offx,offy,offz }), color, transparent);
        }


        glPopMatrix(); //restore matrix
    }

}

//Index cnt0 = 0;

//draw sensor traces
//NOTE: this function could be moved to VisualizationSystem to create this data only once (but what happens for animations?)?
void GlfwRenderer::RenderSensorTraces()
{
    if (visSettings->sensors.traces.showPositionTrace || 
        visSettings->sensors.traces.showVectors || 
        visSettings->sensors.traces.showTriads)
    {
        float factOffset = 1.f*state->maxSceneSize;
        if (state->zoom != 0.f) { factOffset *= 1.f / state->zoom; }

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glPolygonOffset(visSettings->openGL.polygonOffset*factOffset, visSettings->openGL.polygonOffset*factOffset); //
        glEnable(GL_POLYGON_OFFSET_FILL);
        glDisable(GL_POLYGON_OFFSET_LINE);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        if (visSettings->openGL.lineSmooth) { glEnable(GL_LINE_SMOOTH); }

        const ArrayIndex& positionSensors = visSettings->sensors.traces.listOfPositionSensors;
        const ArrayIndex& vectorSensors = visSettings->sensors.traces.listOfVectorSensors;
        const ArrayIndex& triadSensors = visSettings->sensors.traces.listOfTriadSensors;

        glLineWidth(visSettings->sensors.traces.lineWidth);

        Index positionsShowEvery = EXUstd::Maximum(1,visSettings->sensors.traces.positionsShowEvery); //max in order to avoid crash in case of 0 or negative numbers
        Index vectorsShowEvery = EXUstd::Maximum(1, visSettings->sensors.traces.vectorsShowEvery);
        Index triadsShowEvery = EXUstd::Maximum(1, visSettings->sensors.traces.triadsShowEvery);

        const ArrayFloat& edgeColors = visSettings->sensors.traces.traceColors;

        bool showVectors = visSettings->sensors.traces.showVectors && (positionSensors.NumberOfItems() == vectorSensors.NumberOfItems());
        bool showTriads = visSettings->sensors.traces.showTriads && (positionSensors.NumberOfItems() == triadSensors.NumberOfItems());
        
        Real vectorScaling = (Real)visSettings->sensors.traces.vectorScaling;
        float triadSize = visSettings->sensors.traces.triadSize;
        // if no sensors, do other approach: 
        //while list with stop criteria
        //GetSensorsPositionsVectorsLists returns true if further sensors available
        //std::cout << "ST" << showTriads << ", PL" << positionSensors.NumberOfItems()
        //    << ", TL" << triadSensors.NumberOfItems() << "\n";


        Index i = 0;
        bool returnValue = true;
        while ((positionSensors.NumberOfItems() > 0 && i < positionSensors.NumberOfItems()) || (positionSensors.NumberOfItems() == 0 && returnValue) )
        {
            Float4 edgeColor({ 0.,0.,0.,1. }); //Default
            if (edgeColors.NumberOfItems() >= (i + 1) * 4)
            {
                for (Index j = 0; j < 4; j++)
                {
                    edgeColor[j] = edgeColors[i * 4 + j];
                }
            }

            Index positionSensorIndex = i;
            Index vectorSensorIndex = -1;
            Index triadSensorIndex = -1;
            if (i < positionSensors.NumberOfItems()) { positionSensorIndex = positionSensors[i]; }
            if (i < vectorSensors.NumberOfItems() && showVectors) { vectorSensorIndex = vectorSensors[i]; }
            if (i < triadSensors.NumberOfItems() && showTriads) { triadSensorIndex = triadSensors[i]; }

            //get sensor data
            returnValue = basicVisualizationSystemContainer->GetSensorsPositionsVectorsLists(visSettings->sensors.traces.sensorsMbsNumber, positionSensorIndex,
                vectorSensorIndex, triadSensorIndex, sensorTracePositions, sensorTraceVectors, sensorTraceTriads, sensorTraceValues,
                visSettings->sensors.traces);

            if (visSettings->sensors.traces.showPositionTrace)// && sensorTracePositions.NumberOfItems() > 1)
            {
                glBegin(GL_LINE_STRIP); //list of single points to define lines
                glColor4f(edgeColor[0], edgeColor[1], edgeColor[2], edgeColor[3]);

                for (Index j = 0; j < sensorTracePositions.NumberOfItems(); j++)
                {
                    if (j % positionsShowEvery == 0 || j == sensorTracePositions.NumberOfItems() - 1)
                    {
                        const Vector3D& p = sensorTracePositions[j];
                        glVertex3f((float)p[0], (float)p[1], (float)p[2]);
                    }
                }
                glEnd(); //GL_LINE_STRIP
            }

            if ((visSettings->sensors.traces.showVectors && sensorTraceVectors.NumberOfItems() != 0) ||
                (visSettings->sensors.traces.showTriads && sensorTraceTriads.NumberOfItems() != 0) )
            {
                glBegin(GL_LINES);
                //if (visSettings->openGL.enableLighting) { glEnable(GL_LIGHTING); } //only enabled when drawing triangle faces
                //glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
                for (Index j = 0; j < sensorTracePositions.NumberOfItems(); j++)
                {
                    if (j < sensorTraceVectors.NumberOfItems())
                    {
                        if (j % vectorsShowEvery == 0 || j == sensorTraceVectors.NumberOfItems() - 1)
                        {
                            const Vector3D& p = sensorTracePositions[j];
                            const Vector3D& v = sensorTraceVectors[j];
                            glColor4f(edgeColor[0], edgeColor[1], edgeColor[2], edgeColor[3]);
                            glVertex3f((float)p[0], (float)p[1], (float)p[2]);
                            glVertex3f((float)(p[0] + vectorScaling * v[0]),
                                (float)(p[1] + vectorScaling * v[1]),
                                (float)(p[2] + vectorScaling * v[2]));
                        }
                    }
                    if (j < sensorTraceTriads.NumberOfItems())
                    {
                        if (j % triadsShowEvery == 0 || j == sensorTraceTriads.NumberOfItems() - 1)
                        {
                            float f = triadSize;
                            Float3 p({ (float)sensorTracePositions[j][0],
                                        (float)sensorTracePositions[j][1],
                                        (float)sensorTracePositions[j][2] });
                            const Matrix3D& m = sensorTraceTriads[j];
                            //glColor4f(edgeColor[0], edgeColor[1], edgeColor[2], edgeColor[3]); //set back to trace color!
                            glColor4f(1.f, 0, 0, edgeColor[3]);
                            glVertex3f(p[0], p[1], p[2]);
                            glVertex3f(p[0] + f * (float)m(0, 0), p[1] + f * (float)m(1, 0), p[2] + f * (float)m(2, 0));

                            glColor4f(0, 1.f, 0, edgeColor[3]);
                            glVertex3f(p[0], p[1], p[2]);
                            glVertex3f(p[0] + f * (float)m(0, 1), p[1] + f * (float)m(1, 1), p[2] + f * (float)m(2, 1));

                            glColor4f(0, 0, 1.f, edgeColor[3]);
                            glVertex3f(p[0], p[1], p[2]);
                            glVertex3f(p[0] + f * (float)m(0, 2), p[1] + f * (float)m(1, 2), p[2] + f * (float)m(2, 2));
                        }
                    }
                }
                glEnd(); //GL_LINES

                //if (visSettings->openGL.enableLighting) { glDisable(GL_LIGHTING); } //only enabled when drawing triangle faces
            }

            i++;
        }
        if (visSettings->openGL.lineSmooth) { glDisable(GL_LINE_SMOOTH); }

    }
}

void GlfwRenderer::RenderGraphicsData(bool selectionMode)
{
	if (graphicsDataList)
	{
		//use this to draw coplanar lines in front polygons
		//this seems to be affected by zoom size: glPolygonOffset(visSettings->openGL.polygonOffset * state->maxSceneSize, visSettings->openGL.polygonOffset * state->maxSceneSize); //
		float factOffset = 1.f*state->maxSceneSize;
		if (state->zoom != 0.f) { factOffset *=  1.f / state->zoom; }

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glPolygonOffset(visSettings->openGL.polygonOffset*factOffset, visSettings->openGL.polygonOffset*factOffset); //
		glEnable(GL_POLYGON_OFFSET_FILL);
		glDisable(GL_POLYGON_OFFSET_LINE);
		//glDisable(GL_POLYGON_OFFSET_FILL);
		//glEnable(GL_POLYGON_OFFSET_LINE);

		Index lastItemID = itemIDinvalidValue;
		if (selectionMode)
		{
			glLoadName(invalidIndex); //to have some name in it
		}
		//check if item shall be highlighted:
		bool highlight = false;
		Float4 highlightColor = visSettings->interactive.highlightColor;
		Float4 otherColor = visSettings->interactive.highlightOtherColor;
		Float4 highlightColor2 = visSettings->interactive.highlightColor; //for text and lines
		Float4 otherColor2 = visSettings->interactive.highlightOtherColor; //for text and lines

		//Index highlightIndex;
		//ItemType highlightType;
		//Index highlightMbsNumber;
		Index highlightIndex = visSettings->interactive.highlightItemIndex;
		ItemType highlightType = visSettings->interactive.highlightItemType;
		Index highlightMbsNumber = visSettings->interactive.highlightMbsNumber;
		if (visSettings->interactive.selectionHighlights && stateMachine.highlightIndex != invalidIndex)
		{
			//if selected with mouse, temporarily use this:
			if (stateMachine.highlightTimeout != 0. && stateMachine.highlightTimeout < EXUstd::GetTimeInSeconds())
			{
				stateMachine.highlightIndex = invalidIndex; //from now on, no further highlighting
			}
			else
			{
				highlightIndex = stateMachine.highlightIndex;
				highlightType = stateMachine.highlightType;
				highlightMbsNumber = stateMachine.highlightMbsNumber;
			}
		}

		Index highlightID = Index2ItemID(highlightIndex, highlightType, highlightMbsNumber);
		if (highlightIndex >= 0 && highlightType != ItemType::_None)
		{
			highlight = true;
			highlightColor2 = Float4({ EXUstd::Minimum(1.f,highlightColor[0] * 1.2f),
				EXUstd::Minimum(1.f,highlightColor[1] * 1.2f),
				EXUstd::Minimum(1.f,highlightColor[2] * 1.2f),
				highlightColor[3] });
			otherColor2 = Float4({ otherColor[0] * 0.5f, otherColor[1] * 0.5f, otherColor[2] * 0.5f,
				EXUstd::Minimum(0.8f,otherColor[3] * 1.5f) });
		}

		for (auto data : *graphicsDataList)
		{
            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            //DRAW TEXT (before triangles, in order to make texts visible in case of transparency
            if (visSettings->openGL.facesTransparent)
            {
                RenderGraphicsDataText(data, lastItemID, highlight, highlightID, highlightColor2, otherColor2, selectionMode);
            }
            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//DRAW POINTS before triangles (nodes shown in transparent scenes)

			GLfloat d = visSettings->general.pointSize; //point drawing parameter --> put into settings!
			glLineWidth(visSettings->openGL.lineWidth);
			if (visSettings->openGL.lineSmooth) { glEnable(GL_LINE_SMOOTH); }

			if (visSettings->openGL.showFaces)
			{
				if (visSettings->openGL.enableLighting) { glEnable(GL_LIGHTING); } //only enabled when drawing triangle faces
				glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			}

			for (const GLSphere& item : data->glSpheres)
			{
				if (selectionMode) { if (item.itemID != lastItemID) { glLoadName(item.itemID); lastItemID = item.itemID; } }

				if (!visSettings->openGL.showFaces || item.resolution < 1 || item.radius <= 0.f)
				{
					glBegin(GL_LINES);
					if (!highlight)
					{
						glColor4f(item.color[0], item.color[1], item.color[2], item.color[3]);
					}
					else
					{
						if (item.itemID != highlightID) { glColor4fv(otherColor2.GetDataPointer()); }
						else { glColor4fv(highlightColor2.GetDataPointer()); }
					}
					//plot point as 3D cross
					glVertex3f(item.point[0] + d, item.point[1], item.point[2]);
					glVertex3f(item.point[0] - d, item.point[1], item.point[2]);
					glVertex3f(item.point[0], item.point[1] + d, item.point[2]);
					glVertex3f(item.point[0], item.point[1] - d, item.point[2]);
					glVertex3f(item.point[0], item.point[1], item.point[2] + d);
					glVertex3f(item.point[0], item.point[1], item.point[2] - d);

					glEnd();
				}
				else
				{
					//use GLlists based spheres
					if (!highlight)
					{
						glColor4f(item.color[0], item.color[1], item.color[2], item.color[3]);
					}
					else
					{
						if (item.itemID != highlightID) { glColor4fv(otherColor2.GetDataPointer()); }
						else { glColor4fv(highlightColor2.GetDataPointer()); }
					}

					glPushMatrix();
					glTranslatef(item.point[0], item.point[1], item.point[2]);
					glScalef(item.radius, item.radius, item.radius);

					//glListBase(spheresListBase + EXUstd::Minimum(item.resolution, maxSpheresLists-1); //assign base of string list, 32 MUST be smallest value
					glCallList(spheresListBase + EXUstd::Minimum(item.resolution, maxSpheresLists - 1));
					glPopMatrix();
				}
			}
			if (visSettings->openGL.showFaces) //now turn off lighting for lines and texts
			{
				if (visSettings->openGL.enableLighting) { glDisable(GL_LIGHTING); } //only enabled when drawing triangle faces
			}

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//DRAW TRIANGLES
			if (visSettings->openGL.showFaces || visSettings->openGL.showMeshFaces)
			{
				if (visSettings->openGL.enableLighting) { glEnable(GL_LIGHTING); } //only enabled when drawing triangle faces
				glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
				//glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
				if (highlight)
				{
					for (const GLTriangle& trig : data->glTriangles)
					{ //draw faces
						if (selectionMode) { if (trig.itemID != lastItemID) { glLoadName(trig.itemID); lastItemID = trig.itemID; } }
						if ((visSettings->openGL.showFaces && !trig.isFiniteElement)
							|| (visSettings->openGL.showMeshFaces && trig.isFiniteElement))
						{
							glBegin(GL_TRIANGLES);
							for (Index i = 0; i < 3; i++)
							{
								if (trig.itemID != highlightID) { glColor4fv(otherColor.GetDataPointer()); }
								else { glColor4fv(highlightColor.GetDataPointer()); }
								glNormal3fv(trig.normals[i].GetDataPointer());
								glVertex3fv(trig.points[i].GetDataPointer());
							}
							glEnd();
						}
					}
				}
				else if (!visSettings->openGL.facesTransparent)
				{
					for (const GLTriangle& trig : data->glTriangles)
					{ //draw faces
						if (selectionMode) { if (trig.itemID != lastItemID) { glLoadName(trig.itemID); lastItemID = trig.itemID; } }
						if ((visSettings->openGL.showFaces && !trig.isFiniteElement)
							|| (visSettings->openGL.showMeshFaces && trig.isFiniteElement))
						{
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
					if (!selectionMode && visSettings->openGL.shadow != 0)
					{
						DrawTrianglesWithShadow(data);
					}
				}
				else //for global transparency of faces; slower
				{
					const float transparencyLimit = 0.4f; //use at least this transparency
					for (const GLTriangle& trig : data->glTriangles)
					{ //draw faces
						if (selectionMode) { if (trig.itemID != lastItemID) { glLoadName(trig.itemID); lastItemID = trig.itemID; } }
						if ((visSettings->openGL.showFaces && !trig.isFiniteElement)
							|| (visSettings->openGL.showMeshFaces && trig.isFiniteElement))
						{
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
				}
				if (visSettings->openGL.enableLighting) { glDisable(GL_LIGHTING); } //only enabled when drawing triangle faces
			}


			//++++++++++++++++++++++++++++++++++++++++++++++
			//draw lines at end of rendering: lines shown nicely on top of faces with polygon offset
			//++++++++++++++++++++++++++++++++++++++++++++++

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//DRAW CIRCLES
			//draw a circle in xy-plane
			for (const GLCircleXY& item : data->glCirclesXY)
			{
				if (selectionMode) { if (item.itemID != lastItemID) { glLoadName(item.itemID); lastItemID = item.itemID; } }
				glBegin(GL_LINE_STRIP); //list of single points to define lines
				if (!highlight)
				{
					glColor4f(item.color[0], item.color[1], item.color[2], item.color[3]);
				}
				else
				{
					if (item.itemID != highlightID) { glColor4fv(otherColor2.GetDataPointer()); }
					else { glColor4fv(highlightColor2.GetDataPointer()); }
				}

				const Float3& p = item.point;
				GLfloat r = item.radius;
				float nSeg = (float)item.numberOfSegments;
				if (nSeg == 0.f) { nSeg = (float)visSettings->general.circleTiling; }

				//for (float i = 0; i <= nSeg; i += 2.f*EXUstd::pi_f / nSeg)
				for (float i = 0; i <= 2.f*EXUstd::pi_f + 1e-5; i += 2.f*EXUstd::pi_f / nSeg)
				{
					glVertex3f(p[0] + r * sin(i), p[1] + r * cos(i), p[2]);
				}

				glEnd(); //GL_LINE_STRIP
			}


			//draw normals
			if (visSettings->openGL.drawFaceNormals)
			{
				Float4 edgeColor = visSettings->openGL.faceEdgesColor;
				float len = visSettings->openGL.drawNormalsLength;
				for (const GLTriangle& trig : data->glTriangles)
				{
					if (selectionMode) { if (trig.itemID != lastItemID) { glLoadName(trig.itemID); lastItemID = trig.itemID; } }
					Float3 midPoint = { 0,0,0 };
					for (Index i = 0; i < 3; i++)
					{
						midPoint += trig.points[i];
					}
					midPoint *= 1.f / 3.f;
					glColor4f(edgeColor[0], edgeColor[1], edgeColor[2], edgeColor[3]);
					//glColor4f(0.2f, 0.2f, 0.2f, 1.f);
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
					if (selectionMode) { if (trig.itemID != lastItemID) { glLoadName(trig.itemID); lastItemID = trig.itemID; } }
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

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//DRAW LINES
			if (visSettings->openGL.showLines)
			{
				if (!highlight)
				{
					for (const GLLine& item : data->glLines)
					{
						if (selectionMode) { if (item.itemID != lastItemID) { glLoadName(item.itemID); lastItemID = item.itemID; } }
						glBegin(GL_LINES);
						glColor4f(item.color1[0], item.color1[1], item.color1[2], item.color1[3]);
						glVertex3f(item.point1[0], item.point1[1], item.point1[2]);
						glColor4f(item.color2[0], item.color2[1], item.color2[2], item.color2[3]);
						glVertex3f(item.point2[0], item.point2[1], item.point2[2]);
						glEnd();
					}
				}
				else
				{
					for (const GLLine& item : data->glLines)
					{
						if (selectionMode) { if (item.itemID != lastItemID) { glLoadName(item.itemID); lastItemID = item.itemID; } }
						glBegin(GL_LINES);
						if (item.itemID != highlightID) { glColor4fv(otherColor2.GetDataPointer()); }
						else { glColor4fv(highlightColor2.GetDataPointer()); }
						glVertex3f(item.point1[0], item.point1[1], item.point1[2]);

						if (item.itemID != highlightID) { glColor4fv(otherColor2.GetDataPointer()); }
						else { glColor4fv(highlightColor2.GetDataPointer()); }
						glVertex3f(item.point2[0], item.point2[1], item.point2[2]);
						glEnd();
					}
				}
			}


			
			//DRAW TRIANGLES MESH
			if (visSettings->openGL.showFaceEdges || visSettings->openGL.showMeshEdges)
			{
				//glEnable(GL_POLYGON_OFFSET_LINE);

				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
				Float4 edgeColor = visSettings->openGL.faceEdgesColor;

				//if (visSettings->openGL.enableLighting) { glEnable(GL_LIGHTING); } //only enabled when drawing triangle faces
				//glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
				//glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
				for (const GLTriangle& trig : data->glTriangles)
				{ //draw faces edges
					if (selectionMode) { if (trig.itemID != lastItemID) { glLoadName(trig.itemID); lastItemID = trig.itemID; } }
					if ((visSettings->openGL.showFaceEdges && !trig.isFiniteElement)
						|| (visSettings->openGL.showMeshEdges && trig.isFiniteElement))
					{
						if (!highlight)
						{
							glColor4f(edgeColor[0], edgeColor[1], edgeColor[2], edgeColor[3]);
						}
						else
						{
							if (trig.itemID != highlightID) { glColor4fv(otherColor2.GetDataPointer()); }
							else { glColor4fv(highlightColor2.GetDataPointer()); }
						}
						glBegin(GL_TRIANGLES);
						for (Index i = 0; i < 3; i++)
						{
							glNormal3fv(trig.normals[i].GetDataPointer());
							glVertex3fv(trig.points[i].GetDataPointer());
						}
						glEnd();
					}
				}
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				//glDisable(GL_POLYGON_OFFSET_LINE);
			}

            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            //DRAW TEXT finally to be in front of everything if no transparency is activated
            if (!visSettings->openGL.facesTransparent)
            {
                if (visSettings->general.textAlwaysInFront)
                {
                    glDepthMask(GL_FALSE); //draw system information and coordinate system always in front
                }

                
                RenderGraphicsDataText(data, lastItemID, highlight, highlightID, highlightColor2, otherColor2, selectionMode);
                if (GetGLdepthMask())
                {
                    glDepthMask(GL_TRUE); //switch back to original state
                }
            }
            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


		} //for (auto data : *graphicsDataList)
		if (selectionMode)
		{
			glLoadName(invalidIndex); //to have some name in
		}



	} //if graphicsDataList
}

//draw stenciled shadow volume
//following concepts of https://github.com/joshb/shadowvolumes
void RenderTriangleShadowVolume(const GLTriangle& trig, const Float3& lightPos, float maxDist, float shadow)
{
	const bool computeNormals = false;
	//glColor4f(0.f, 0.f, 0.f, shadow);//shadow

	//check if triangle normal is looking in direction of light (otherwise no shadow is produced
	if (EXUmath::ComputeTriangleNormal(trig.points[0], trig.points[1], trig.points[2]) * (lightPos - trig.points[0]) > 0)
	{
		if (computeNormals) { glNormal3fv(EXUmath::ComputeTriangleNormal(trig.points[0], trig.points[1], trig.points[2]).GetDataPointer()); }
		glBegin(GL_TRIANGLES);
		for (Index i = 0; i < 3; i++)
		{
			glVertex3fv(trig.points[i].GetDataPointer());
		}
		std::array< Float3, 3> farPoints;

		if (computeNormals) { glNormal3fv(EXUmath::ComputeTriangleNormal(trig.points[2], trig.points[1], trig.points[0]).GetDataPointer()); }
		for (Index i = 2; i >= 0; i--)
		{
			Float3 vecDist = trig.points[i] - lightPos;
			//Float3 vecDist = - lightPos;
			//float dist = vecDist.SumAbs()*0.577f; //cheaper than norm, but needs safety factor sqrt(3)
			float dist = vecDist.GetL2Norm(); //SumAbs()*0.577 would be cheaper and also on safe side
			if (dist != 0)
			{
				vecDist *= maxDist / dist; //scale up to maximum distance in scene, will cover all objects
			}

			farPoints[i] = trig.points[i] + vecDist;
			glVertex3fv(farPoints[i].GetDataPointer());
		}
		glEnd();

		glBegin(GL_QUADS);
		for (Index i = 0; i < 3; i++)
		{
			Index iNext = (i + 1) % 3;
			if (computeNormals) { glNormal3fv(EXUmath::ComputeTriangleNormal(trig.points[i], farPoints[i], farPoints[iNext]).GetDataPointer()); }
			glVertex3fv(trig.points[i].GetDataPointer());
			glVertex3fv(farPoints[i].GetDataPointer());
			glVertex3fv(farPoints[iNext].GetDataPointer());
			glVertex3fv(trig.points[iNext].GetDataPointer());
		}
		glEnd();
	}

}

//draw full plane over screen, containing shadow mask
void DrawShadowPlane(float shadow)
{
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, 1, 1, 0, 0, 1);
	glDisable(GL_DEPTH_TEST);

	glColor4f(0.0f, 0.0f, 0.0f, shadow);
	glBegin(GL_QUADS);
	glVertex2i(0, 0);
	glVertex2i(0, 1);
	glVertex2i(1, 1);
	glVertex2i(1, 0);
	glEnd();

	glEnable(GL_DEPTH_TEST);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}


void GlfwRenderer::DrawTrianglesWithShadow(GraphicsData* data)
{
	//render triangle shadow using stencils
	const Float4& lp = visSettings->openGL.light0position;
	Float3 lightPos({ lp[0], lp[1], lp[2] });
	float maxDist = state->maxSceneSize*1.5f; 
	float shadow = EXUstd::Minimum(visSettings->openGL.shadow, 1.f);

	if (false)
	{
		//test to show shadow volumes
		glColor4f(0.6f, 0.3f, 0.3f, 1);
		for (const GLTriangle& trig : data->glTriangles)
		{ //draw faces
			if ((visSettings->openGL.showFaces && !trig.isFiniteElement)
				|| (visSettings->openGL.showMeshFaces && trig.isFiniteElement))
			{
				RenderTriangleShadowVolume(trig, lightPos, maxDist, shadow);
			}
		}
	}
	else
	{
		//add shadow now:

		glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        SetGLdepthMask(GL_FALSE);
		//glDepthFunc(GL_ALWAYS); //added ...?
		glEnable(GL_CULL_FACE);
		glEnable(GL_STENCIL_TEST);
		glEnable(GL_POLYGON_OFFSET_FILL);

		float factOffset = 1.f*state->maxSceneSize;
		if (state->zoom != 0.f) { factOffset *= 1.f / state->zoom; }
		//here, we must use the original offset and add a shadow offset ...
		glPolygonOffset(visSettings->openGL.polygonOffset*factOffset, visSettings->openGL.polygonOffset*factOffset + visSettings->openGL.shadowPolygonOffset * state->maxSceneSize);

		glCullFace(GL_FRONT);
		glStencilFunc(GL_ALWAYS, 0x0, 0xff);
		glStencilOp(GL_KEEP, GL_INCR_WRAP, GL_KEEP);
		//glStencilOp(GL_KEEP, GL_INCR, GL_KEEP); //fails at 255 faces
		for (const GLTriangle& trig : data->glTriangles)
		{ //draw faces
			if ((visSettings->openGL.showFaces && !trig.isFiniteElement)
				|| (visSettings->openGL.showMeshFaces && trig.isFiniteElement))
			{
				RenderTriangleShadowVolume(trig, lightPos, maxDist, shadow);
			}
		}
		glCullFace(GL_BACK);
		glStencilFunc(GL_ALWAYS, 0x0, 0xff);
		glStencilOp(GL_KEEP, GL_DECR_WRAP, GL_KEEP);
		//glStencilOp(GL_KEEP, GL_DECR, GL_KEEP); //fails at 255 faces
		for (const GLTriangle& trig : data->glTriangles)
		{ //draw faces
			if ((visSettings->openGL.showFaces && !trig.isFiniteElement)
				|| (visSettings->openGL.showMeshFaces && trig.isFiniteElement))
			{
				RenderTriangleShadowVolume(trig, lightPos, maxDist, shadow);
			}
		}

		glDisable(GL_POLYGON_OFFSET_FILL);
		glDisable(GL_CULL_FACE);
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        SetGLdepthMask(GL_TRUE);

		glStencilFunc(GL_NOTEQUAL, 0x0, 0xff);
		glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);

		DrawShadowPlane(shadow);

		glDisable(GL_STENCIL_TEST);

		glPolygonOffset(visSettings->openGL.polygonOffset*factOffset, visSettings->openGL.polygonOffset*factOffset); //
		glEnable(GL_POLYGON_OFFSET_FILL);
		glDisable(GL_POLYGON_OFFSET_LINE);

	}
}

#endif //USE_GLFW_GRAPHICS
