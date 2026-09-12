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
#include "Linalg/Geometry.h"
#include "Linalg/BoundingBox.h"

#include "Graphics/Raytracing.h"


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

//if this flag is set, the GLFW thread will be detached (which may be advantageous if stoprenderer is not called); otherwise it is a joinable thread
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
bool GlfwRenderer::callBackRefreshSignal = false;
Index GlfwRenderer::rendererTasksCount = 0;

Real GlfwRenderer::lastTimeAutoRotate = 0;
bool GlfwRenderer::doAutoRotate = false;

Index GlfwRenderer::rendererError = 0;

//GLFWwindow* GlfwRenderer::window = nullptr;
//GLFWwindow* GlfwRenderer::subWindow = nullptr;
//RenderState* GlfwRenderer::state;

RenderViewList GlfwRenderer::renderViews; //!< link to GLFWwindow, RenderState and more


RenderStateMachine GlfwRenderer::stateMachine;
std::thread GlfwRenderer::rendererThread;
Index GlfwRenderer::verboseRenderer = 0;         //0=False, 1=True (main output), 2=more info, 3=debug
//DELETE: Index GlfwRenderer::firstRun = 0; //zoom all in first run
std::atomic_flag GlfwRenderer::renderFunctionRunning = ATOMIC_FLAG_INIT;  //!< semaphore to check if Render(...)  function is currently running (prevent from calling twice); initialized with clear state
std::atomic_flag GlfwRenderer::showMessageSemaphore = ATOMIC_FLAG_INIT;   //!< semaphore for ShowMessage

BitmapFont GlfwRenderer::bitmapFont;				//!< bitmap font for regular texts, initialized upon start of renderer

GLuint GlfwRenderer::textureNumberRGBbitmap[256*NUMBER_OF_TEXTUREFONT_LISTS];	//!< store texture number for our bitmap font; in ultimate case, there are 2*nCharacter lists
GLuint GlfwRenderer::bitmapFontListBase;			//!< starting index for GLlists for font bitmap textured quads
ResizableArray<GLubyte> GlfwRenderer::charBuffer;	//!< buffer for converstion of UTF8 into internal unicode-like format

bool GlfwRenderer::depthMask;                   //!< state of glDepthMask (except for fonts)
GLuint GlfwRenderer::spheresListBase;			//!< starting index for GLlists for spheres

GraphicsData GlfwRenderer::graphicsDataStatic;	//!< static GraphicsData objects (info, Exudyn, etc.)

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
    depthMask = false;

	ResetStateMachine();
};


void GlfwRenderer::ResetStateMachine()
{
	stateMachine.Reset();
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

//! called when according key has been pressed to close window, etc.
void GlfwRenderer::window_close_callback(GLFWwindow* window)
{
	Index viewID = renderViews.GetViewID(window);
	if (viewID == mainViewID) //no action on other windows, as they will be closed in DoRendererTasks()
	{
		if (PyGetRendererCallbackLock()) {
			glfwSetWindowShouldClose(window, GL_FALSE);
			return;
		}

		bool reallyQuit = true;

		const Real timeoutCloseWindow = 8;
		if ((EXUstd::GetTimeInSeconds() - rendererStartTime > visSettings->general.reallyQuitTimeLimit) &&
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
}

void GlfwRenderer::window_content_scale_callback(GLFWwindow* window, float xscale, float yscale) 
{ 
	Index viewID = renderViews.GetViewID(window);
	SetContentScaling(viewID, xscale, yscale); 
}


void GlfwRenderer::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	Index viewID = renderViews.GetViewID(window);
	RenderState* state = renderViews.State(viewID);
	VSettingsView& settingsView = GetSettingsView(viewID, *visSettings);

	if (PyGetRendererCallbackLock()) { return; }
	SetCallBackRefreshSignal();
	//if (graphicsUpdateAtomicFlag.test_and_set(std::memory_order_acquire)) { return; } //ignore keys if currently in use

	//EXUstd::WaitAndLockSemaphore(graphicsUpdateAtomicFlag);

	const Real timeoutShowItem = 2; //seconds

	//Q does not close the window, but stops simulation! ESCAPE stops renderer
	if ((key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) ||
        (key == GLFW_KEY_Q && action == GLFW_PRESS && mods == 0))
	{
		if (viewID == mainViewID)
		{
			bool reallyQuit = true;
			//check long-time runs as key affects all windows (in fact the simulation):
			if ((EXUstd::GetTimeInSeconds() - rendererStartTime > visSettings->general.reallyQuitTimeLimit))
			{
				if (verboseRenderer) { PrintDelayed("Long running simulation: requires additional action to quit!"); }
				//result=-2: exception occured
				//result=-1: undefined quit response
				//result= 0: default result value: unused HERE
				//result= 1: tkinter dialog opened
				//result= 2: do not quit
				//result= 3: quit

				PyProcessSetResult(-1); //initialize as undefined
				PyQueuePythonProcess(ProcessID::AskYesNo, 0);

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
					if (verboseRenderer) { PrintDelayed("QUIT renderer: waited for " + EXUstd::ToString(i * 20) + " milliseconds \n"); }
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
					//glfwSetWindowShouldClose(window, GL_TRUE); //will be done later

					return; //don't process keys or call user function
				}
			}
		}
		else
		{
			glfwSetWindowShouldClose(window, GL_TRUE); //notify that window shall be closed for this view; same as closing window itself
			return; //don't process keys or call user function for this window
		}
	}

	//switch ignore keys functionality
	if (key == GLFW_KEY_F2 && action == GLFW_PRESS && mods == 0)
	{
		visSettings->interactive.ignoreKeys = !visSettings->interactive.ignoreKeys;
		//rendererOut << "ignore keys mode switched to " << visSettings->interactive.ignoreKeys << "\n";
		//PyQueueExecutableString("print('ignore keys mode switched to " + EXUstd::ToString(visSettings->interactive.ignoreKeys) + "')\n");
		ShowMessage("ignore keys mode switched " + OnOffFromBool(visSettings->interactive.ignoreKeys), timeoutShowItem);
	}

	//do this first, as key may still have time to complete action
	if (visSettings->interactive.ignoreKeys) //2023-06-03: makes no sense: || !(key == GLFW_KEY_Q && action == GLFW_PRESS && mods == 0))
	{
		//keyPressUserFunction uses the pybind interface and thus causes crashes when set or copied (Python thread!):
		PyQueueKeyPressed(key, action, mods); // visSettings->interactive.keyPressUserFunction); //call python user function
	}

	//+++++++++++++++++++++++++++++++++++++++++++++
	//check if regular keys are ignored:
	if (!visSettings->interactive.ignoreKeys)
	{
		//keycode to quit simulation:
		//if (key == GLFW_KEY_Q && action == GLFW_PRESS && mods == 0)
		//{
		//	basicVisualizationSystemContainer->StopSimulation();
		//}

		//keycode to continue paused simulation or to pause:
		if (((key == GLFW_KEY_SPACE && action == GLFW_PRESS) ||
			(key == GLFW_KEY_SPACE && action == GLFW_REPEAT)) && mods == 0) //changed shift to repeat 
		{
            if (visSettings->interactive.advanced.pauseWithSpacebar)
            {
                ShowMessage("SPACE pressed: switch pause on/off", timeoutShowItem);
                basicVisualizationSystemContainer->SwitchPauseSimulation();
            }
            else //only continue, but no pause:
            {
                basicVisualizationSystemContainer->ContinueSimulation();
            }
        }

		//switch show mouse coordinates
		if (key == GLFW_KEY_F3 && action == GLFW_PRESS && mods == 0)
		{
			//glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN); hide cursor over window
			settingsView.window.showMouseCoordinates = !settingsView.window.showMouseCoordinates;
			stateMachine.rendererMessage = "";
		}
		if (key == GLFW_KEY_F3 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			settingsView.window.showRenderStateInfo = !settingsView.window.showRenderStateInfo;

			if (settingsView.window.showRenderStateInfo) //if switched on:
			{
				//without trackMarker, as user wants to know how to set view relative to marker
				Float3 translationMV = state->GetTranslationF();

				STDstring str = "Set current view: SC.renderer.SetModelView(zoom=" + EXUstd::ToString(state->zoom, 7) + ",rotationVector=";
				//without trackMarker, as user wants to know how to set view relative to marker
				str += EXUstd::ToString(state->GetRotationVector(basicVisualizationSystemContainer, settingsView, false), 7) + ",centerPoint=";
				str += EXUstd::ToString(translationMV, 7) + ")";
				PrintDelayed(str);
			}
			stateMachine.rendererMessage = "";
		}
		if (key == GLFW_KEY_R && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			settingsView.camera.useRaytracer = !settingsView.camera.useRaytracer;
			ShowMessage(STDstring("CTRL-R pressed: switched raytracing ")+OnOffFromBool(settingsView.camera.useRaytracer)+" for view "+EXUstd::ToString(viewID),
				timeoutShowItem);
		}
		if (key == GLFW_KEY_R && action == GLFW_PRESS && mods == 0)
		{
			visSettings->interactive.autoRotateModelView = !visSettings->interactive.autoRotateModelView;
			ShowMessage(STDstring("autorotation ") + OnOffFromBool(visSettings->interactive.autoRotateModelView),
				timeoutShowItem);
		}

		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
		//process keys for showing nodes, bodies, ...
		if (key == GLFW_KEY_N && action == GLFW_PRESS && mods == 0)
		{
			visSettings->nodes.show = !visSettings->nodes.show; UpdateGraphicsDataNow();
			ShowMessage("show nodes: "+ OnOffFromBool(visSettings->nodes.show), timeoutShowItem);
		}
		if (key == GLFW_KEY_B && action == GLFW_PRESS && mods == 0)
		{
			visSettings->bodies.show = !visSettings->bodies.show; UpdateGraphicsDataNow();
			ShowMessage("show bodies: " + OnOffFromBool(visSettings->bodies.show), timeoutShowItem);
		}
		if (key == GLFW_KEY_C && action == GLFW_PRESS && mods == 0)
		{
			visSettings->connectors.show = !visSettings->connectors.show; UpdateGraphicsDataNow();
			ShowMessage("show connectors: " + OnOffFromBool(visSettings->connectors.show), timeoutShowItem);
		}
		if (key == GLFW_KEY_M && action == GLFW_PRESS && mods == 0)
		{
			visSettings->markers.show = !visSettings->markers.show; UpdateGraphicsDataNow();
			ShowMessage("show markers: " + OnOffFromBool(visSettings->markers.show), timeoutShowItem);
		}
		if (key == GLFW_KEY_L && action == GLFW_PRESS && mods == 0)
		{
			visSettings->loads.show = !visSettings->loads.show; UpdateGraphicsDataNow();
			ShowMessage("show loads: " + OnOffFromBool(visSettings->loads.show), timeoutShowItem);
		}
		if (key == GLFW_KEY_S && action == GLFW_PRESS && mods == 0)
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
		if (key == GLFW_KEY_T && action == GLFW_PRESS && mods == 0)
		{
			//OLD: visSettings->openGL.facesTransparent = !visSettings->openGL.facesTransparent;
			//switch between faces transparent + edges / faces transparent / only face edges / full faces with edges / only faces
			if (!settingsView.scene.facesTransparent && settingsView.scene.showFaces && !settingsView.scene.showFaceEdges && settingsView.scene.showMeshFaces && settingsView.scene.showMeshEdges)
			{
				settingsView.scene.facesTransparent = false;
				settingsView.scene.showFaces = true;
				settingsView.scene.showFaceEdges = false;
				settingsView.scene.showMeshFaces = true;
				settingsView.scene.showMeshEdges = false;
			}
			else if (!settingsView.scene.facesTransparent && settingsView.scene.showFaces && !settingsView.scene.showFaceEdges && settingsView.scene.showMeshFaces && !settingsView.scene.showMeshEdges)
			{
				settingsView.scene.facesTransparent = true;
				settingsView.scene.showFaces = true;
				settingsView.scene.showFaceEdges = true;
				settingsView.scene.showMeshFaces = true;
				settingsView.scene.showMeshEdges = true;
			}
			else if (settingsView.scene.facesTransparent && settingsView.scene.showFaces && settingsView.scene.showFaceEdges && settingsView.scene.showMeshFaces && settingsView.scene.showMeshEdges)
			{
				settingsView.scene.facesTransparent = true;
				settingsView.scene.showFaces = true;
				settingsView.scene.showFaceEdges = false;
				settingsView.scene.showMeshFaces = true;
				settingsView.scene.showMeshEdges = true;
			}
			else if (settingsView.scene.facesTransparent && settingsView.scene.showFaces && !settingsView.scene.showFaceEdges && settingsView.scene.showMeshFaces && settingsView.scene.showMeshEdges)
			{
				settingsView.scene.facesTransparent = false;
				settingsView.scene.showFaces = true;
				settingsView.scene.showFaceEdges = true;
				settingsView.scene.showMeshFaces = true;
				settingsView.scene.showMeshEdges = true;
			}
			else if (!settingsView.scene.facesTransparent && settingsView.scene.showFaces && settingsView.scene.showFaceEdges && settingsView.scene.showMeshFaces && settingsView.scene.showMeshEdges)
			{
				settingsView.scene.facesTransparent = false;
				settingsView.scene.showFaces = false;
				settingsView.scene.showFaceEdges = true;
				settingsView.scene.showMeshFaces = false;
				settingsView.scene.showMeshEdges = true;
			}
			else if (!settingsView.scene.facesTransparent && !settingsView.scene.showFaces && settingsView.scene.showFaceEdges && !settingsView.scene.showMeshFaces && settingsView.scene.showMeshEdges)
			{
				settingsView.scene.facesTransparent = false;
				settingsView.scene.showFaces = false;
				settingsView.scene.showFaceEdges = true;
				settingsView.scene.showMeshFaces = true;
				settingsView.scene.showMeshEdges = true;
			}
			else
			{
				settingsView.scene.facesTransparent = false;
				settingsView.scene.showFaces = true;
				settingsView.scene.showFaceEdges = false;
				settingsView.scene.showMeshFaces = true;
				settingsView.scene.showMeshEdges = true;
			}
			
			UpdateGraphicsDataNow();
			ShowMessage("faces transparent=" + OnOffFromBool(settingsView.scene.facesTransparent) +
				", faces=" + OnOffFromBool(settingsView.scene.showFaces) +
				", face edges=" + OnOffFromBool(settingsView.scene.showFaceEdges) +
				", mesh faces=" + OnOffFromBool(settingsView.scene.showMeshFaces) +
				", mesh edges=" + OnOffFromBool(settingsView.scene.showMeshEdges)
				, timeoutShowItem);
		}
		if (key == GLFW_KEY_X && action == GLFW_PRESS && mods == 0)
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
		if (key == GLFW_KEY_V && action == GLFW_PRESS && mods == 0)
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

		//add further view, if available:
		if (key == GLFW_KEY_V && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
		{
			for (Index viewID = 1; viewID < renderViews.NumberOfViews(); viewID++)
			{
				if (!renderViews.IsValidWindow(viewID))
				{
					renderViews.SetWindowShouldBeCreated(viewID, true);
					PrintDelayed(STDstring("Creating window for view " + EXUstd::ToString(viewID)));
					break;
				}
			}
		}
		//help key
		if (key == GLFW_KEY_H && action == GLFW_PRESS && mods == 0)
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


		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// operations that change zoom, rotation, view:
		// process keys for move, rotate, zoom
		if (!settingsView.window.lockModelView)
		{
			HomogeneousTransformationF previousModelView;
			previousModelView.SetHT44(state->modelRotation.GetTransposed());
			HomogeneousTransformationF newModelView = previousModelView;
			HomogeneousTransformationF tempRot;

			bool keyPressed = false;

			float rotStep = visSettings->interactive.advanced.keypressRotationStep; //degrees
			float transStep = visSettings->interactive.advanced.keypressTranslationStep * state->zoom; //degrees
			if ((mods & GLFW_MOD_CONTROL) != 0) //only for rotStep and transStep
			{
				rotStep *= 0.1f;
				transStep *= 0.1f;
			}

			float zoomStep = visSettings->interactive.advanced.zoomStepFactor;
			Float3 incRot({ 0.f,0.f,0.f });
			bool hasShift = (mods & GLFW_MOD_SHIFT) != 0;
			bool hasAlt = (mods & GLFW_MOD_ALT) != 0;

			if (key == GLFW_KEY_KP_2 && (action == GLFW_PRESS || action == GLFW_REPEAT) && mods == 0) { incRot[0] = rotStep; keyPressed=true;}
			if (key == GLFW_KEY_KP_8 && (action == GLFW_PRESS || action == GLFW_REPEAT) && mods == 0) { incRot[0] = -rotStep; keyPressed=true;}
			if (key == GLFW_KEY_KP_4 && (action == GLFW_PRESS || action == GLFW_REPEAT) && mods == 0) { incRot[1] = rotStep; keyPressed=true;}
			if (key == GLFW_KEY_KP_6 && (action == GLFW_PRESS || action == GLFW_REPEAT) && mods == 0) { incRot[1] = -rotStep; keyPressed=true;}
			if (key == GLFW_KEY_KP_7 && (action == GLFW_PRESS || action == GLFW_REPEAT) && mods == 0) { incRot[2] = rotStep; keyPressed=true;}
			if (key == GLFW_KEY_KP_9 && (action == GLFW_PRESS || action == GLFW_REPEAT) && mods == 0) { incRot[2] = -rotStep; keyPressed=true;}

			if (hasShift && key == GLFW_KEY_UP && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[0] = rotStep; keyPressed=true;}
			if (hasShift && key == GLFW_KEY_DOWN && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[0] = -rotStep; keyPressed=true;}
			if (hasShift && key == GLFW_KEY_LEFT && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[1] = rotStep; keyPressed=true;}
			if (hasShift && key == GLFW_KEY_RIGHT && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[1] = -rotStep; keyPressed=true;}
			if (hasAlt && key == GLFW_KEY_LEFT && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[2] = rotStep; keyPressed=true;}
			if (hasAlt && key == GLFW_KEY_RIGHT && (action == GLFW_PRESS || action == GLFW_REPEAT)) { incRot[2] = -rotStep; keyPressed=true;}

			if (incRot[0] + incRot[1] + incRot[2] != 0.f)
			{
				//glMatrixMode(GL_MODELVIEW);
				//glLoadMatrixf(state->modelRotation.GetDataPointer()); //load previous rotation
				//glRotatef(incRot[0], 1.f, 0.f, 0.f); //apply "incremental" rotation around x
				//glRotatef(incRot[1], 0.f, 1.f, 0.f); //apply "incremental" rotation around y
				//glRotatef(incRot[2], 0.f, 0.f, 1.f); //apply "incremental" rotation around z
				//glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
				newModelView.SetRotation(EXUstd::degree2radF * incRot);
				newModelView = newModelView*previousModelView; //first multiply with newModelView to keep key-rotations aligned with screen axes
			}

			//change view:
			if (key == GLFW_KEY_1 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
			{
				//glMatrixMode(GL_MODELVIEW);
				//glLoadIdentity();	//start with identity
				//glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
				newModelView.SetIdentity();

				ShowMessage("View 1: 1-2-plane", timeoutShowItem);
				keyPressed = true;
			}

			if (key == GLFW_KEY_2 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
			{
				//glMatrixMode(GL_MODELVIEW);
				//glLoadIdentity();	//start with identity
				//glRotated(-90, 1.0, 0.0, 0.0);
				//glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering

				newModelView.SetRotation(EXUstd::degree2radF * Float3({-90,0,0}));
				ShowMessage("View 2: 1-3-plane", timeoutShowItem);
				keyPressed = true;
			}

			if (key == GLFW_KEY_3 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
			{
				//glMatrixMode(GL_MODELVIEW);
				//glLoadIdentity();	//start with identity
				//glRotated(-90, 0.0, 1.0, 0.0);
				//glRotated(-90, 1.0, 0.0, 0.0);
				//glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering

				newModelView.SetRotation(EXUstd::degree2radF * Float3({ 0,-90,0 }));
				tempRot.SetRotation(EXUstd::degree2radF * Float3({ -90,0,0 }));
				newModelView = newModelView * tempRot;
				ShowMessage("View 3: 2-3-plane", timeoutShowItem);
				keyPressed = true;
			}

			if (key == GLFW_KEY_1 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
			{
				//glMatrixMode(GL_MODELVIEW);
				//glLoadIdentity();	//start with identity
				//glRotated(180, 0.0, 1.0, 0.0);
				//glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering

				newModelView.SetRotation(EXUstd::degree2radF * Float3({ 0, 180 ,0 }));
				ShowMessage("View 1: 1-2-plane mirrored about vertical axis", timeoutShowItem);
				keyPressed = true;
			}

			if (key == GLFW_KEY_2 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
			{
				//glMatrixMode(GL_MODELVIEW);
				//glLoadIdentity();	//start with identity
				//glRotated(-90, 1.0, 0.0, 0.0);
				//glRotated(180, 0.0, 0.0, 1.0);
				//glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering

				newModelView.SetRotation(EXUstd::degree2radF * Float3({ -90,0,0 }));
				tempRot.SetRotation(EXUstd::degree2radF * Float3({ 0,0,180 }));
				newModelView = newModelView * tempRot;
				ShowMessage("View 2: 1-3-plane mirrored about vertical axis", timeoutShowItem);
				keyPressed = true;
			}

			if (key == GLFW_KEY_3 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
			{
				//glMatrixMode(GL_MODELVIEW);
				//glLoadIdentity();	//start with identity
				//glRotated(-90, 0.0, 1.0, 0.0);
				//glRotated(-90, 1.0, 0.0, 0.0);
				//glRotated(180, 0.0, 0.0, 1.0);
				//glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering

				newModelView.SetRotation(EXUstd::degree2radF * Float3({ 0,-90,0 }));
				tempRot.SetRotation(EXUstd::degree2radF * Float3({ -90,0,0 }));
				newModelView = newModelView * tempRot;
				tempRot.SetRotation(EXUstd::degree2radF * Float3({ 0,0,180 }));
				newModelView = newModelView * tempRot;
				ShowMessage("View 3: 2-3-plane mirrored about vertical axis", timeoutShowItem);
				keyPressed = true;
			}

			//second group of views:
			if (key == GLFW_KEY_4 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
			{
				//glMatrixMode(GL_MODELVIEW);
				//glLoadIdentity();	//start with identity
				//glRotated(180, 0.0, 1.0, 0.0);
				//glRotated(90, 0.0, 0.0, 1.0);
				//glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
				newModelView.SetRotation(EXUstd::degree2radF * Float3({ 0,180,0 }));
				tempRot.SetRotation(EXUstd::degree2radF * Float3({ 0,0,90 }));
				newModelView = newModelView * tempRot;

				ShowMessage("View 4: 2-1-plane", timeoutShowItem);
				keyPressed = true;
			}

			if (key == GLFW_KEY_5 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
			{
				//glMatrixMode(GL_MODELVIEW);
				//glLoadIdentity();	//start with identity
				//glRotated(90, 0.0, 1.0, 0.0);
				//glRotated(90, 0.0, 0.0, 1.0);
				//glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering

				newModelView.SetRotation(EXUstd::degree2radF * Float3({ 0,90.f,0 }));
				tempRot.SetRotation(EXUstd::degree2radF* Float3({ 0,0,90.f }));
				newModelView = newModelView * tempRot;

				ShowMessage("View 5: 3-1-plane", timeoutShowItem);
				keyPressed = true;
			}

			if (key == GLFW_KEY_6 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
			{
				//glMatrixMode(GL_MODELVIEW);
				//glLoadIdentity();	//start with identity
				//glRotated(90, 0.0, 1.0, 0.0);
				//glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
				newModelView.SetRotation(EXUstd::degree2radF* Float3({ 0, 90 ,0 }));

				ShowMessage("View 6: 3-2-plane", timeoutShowItem);
				keyPressed = true;
			}

			if (key == GLFW_KEY_4 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
			{
				//glMatrixMode(GL_MODELVIEW);
				//glLoadIdentity();	//start with identity
				//glRotated(90, 0.0, 0.0, 1.0);
				//glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
				newModelView.SetRotation(EXUstd::degree2radF* Float3({ 0, 0, 90 }));

				ShowMessage("View 4: 2-1-plane mirrored about vertical axis", timeoutShowItem);
				keyPressed = true;
			}

			if (key == GLFW_KEY_5 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
			{
				//glMatrixMode(GL_MODELVIEW);
				//glLoadIdentity();	//start with identity
				//glRotated(-90, 1.0, 0.0, 0.0);
				//glRotated(-90, 0.0, 1.0, 0.0);
				//glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering

				newModelView.SetRotation(EXUstd::degree2radF* Float3({ -90,0,0 }));
				tempRot.SetRotation(EXUstd::degree2radF* Float3({ 0,-90,0 }));
				newModelView = newModelView * tempRot;

				ShowMessage("View 5: 3-1-plane mirrored about vertical axis", timeoutShowItem);
				keyPressed = true;
			}

			if (key == GLFW_KEY_6 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL + GLFW_MOD_SHIFT)
			{
				//glMatrixMode(GL_MODELVIEW);
				//glLoadIdentity();	//start with identity
				//glRotated(-90, 0.0, 1.0, 0.0);
				//glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
				newModelView.SetRotation(EXUstd::degree2radF* Float3({ 0, -90 ,0 }));

				ShowMessage("View 6: 3-2-plane mirrored about vertical axis", timeoutShowItem);
				keyPressed = true;
			}

			if (key == GLFW_KEY_7 && action == GLFW_PRESS && mods == GLFW_MOD_CONTROL)
			{
				//glMatrixMode(GL_MODELVIEW);
				//glLoadIdentity();	//start with identity
				//glRotated(52, -0.82, 0.25, 0.51487863); //gives a nice view on x-y plane
				//glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering

				newModelView.SetRotation((52 * EXUstd::degree2radF) * Float3({ -0.82f, 0.25f, 0.51487863f }));

				ShowMessage("View 7: 3D view 1", timeoutShowItem);
				keyPressed = true;
			}
			//++++++++++++++++++++++++
			//now update rotations in modelRotation:
			state->modelRotation = newModelView.GetHT44().GetTransposed();
			//update centerPoint accordingly (looks like rotation around rotationCenterPoint):
			state->CenterPointUpdateFromRotationChange(previousModelView.GetRotation(), newModelView.GetRotation(), false);//false=use row-major

			//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

			if (!hasShift && !hasAlt && key == GLFW_KEY_UP && (action == GLFW_PRESS || action == GLFW_REPEAT)) { state->centerPoint[1] -= transStep; keyPressed=true;}
			if (!hasShift && !hasAlt && key == GLFW_KEY_DOWN && (action == GLFW_PRESS || action == GLFW_REPEAT)) { state->centerPoint[1] += transStep; keyPressed=true;}
			if (!hasShift && !hasAlt && key == GLFW_KEY_LEFT && (action == GLFW_PRESS || action == GLFW_REPEAT)) { state->centerPoint[0] += transStep; keyPressed=true;}
			if (!hasShift && !hasAlt && key == GLFW_KEY_RIGHT && (action == GLFW_PRESS || action == GLFW_REPEAT)) { state->centerPoint[0] -= transStep; keyPressed=true;}

			if ((key == GLFW_KEY_KP_SUBTRACT || key == GLFW_KEY_COMMA) && (action == GLFW_PRESS || action == GLFW_REPEAT) && (mods == 0 || mods == GLFW_MOD_CONTROL))
			{
				if (mods == GLFW_MOD_CONTROL)
				{
					state->zoom *= pow(zoomStep, 0.1f);
				} //small zoom step
				else { state->zoom *= zoomStep; }
				keyPressed = true;
			}
			if ((key == GLFW_KEY_KP_ADD || key == GLFW_KEY_PERIOD) && (action == GLFW_PRESS || action == GLFW_REPEAT) && (mods == 0 || mods == GLFW_MOD_CONTROL))
			{
				if (mods == GLFW_MOD_CONTROL && zoomStep > 0)
				{
					state->zoom /= pow(zoomStep, 0.1f);
				} //small zoom step
				else { state->zoom /= zoomStep; }
				keyPressed = true;
			}

			if (key == GLFW_KEY_A && action == GLFW_PRESS && mods == 0)
			{
				ZoomAll(viewID);
				keyPressed = true;
			}
			if (key == GLFW_KEY_O && action == GLFW_PRESS && mods == 0)
			{
				const float* A = state->modelRotation.GetDataPointer();
				Matrix3DF rotationMV(3, 3, { A[0], A[1], A[2],  A[4], A[5], A[6],  A[8], A[9], A[10] });

				//DELETE: Float3 pOld = state->rotationCenterPoint * rotationMV + state->centerPoint;
				//we only update rotation center in sceen plane, but keep "Z" coordinate unchanged
				Float3 oldRotationCenterProjected = state->rotationCenterPoint * rotationMV;
				oldRotationCenterProjected[0] = 0; //this part is updated!
				oldRotationCenterProjected[1] = 0; //this part is updated!
				state->rotationCenterPoint = rotationMV * (state->centerPoint + oldRotationCenterProjected);

				Index precision = visSettings->general.rendererPrecision;
				ShowMessage(STDstring("Set rotationCenterPoint to current center: p=[")+
					EXUstd::Num2String(state->rotationCenterPoint[0], precision) + "," +
					EXUstd::Num2String(state->rotationCenterPoint[1], precision) + "," +
					EXUstd::Num2String(state->rotationCenterPoint[2], precision) +
					"]", timeoutShowItem);
				keyPressed = true;
			}
			if (keyPressed) //do not show mouse coordinates/last point; it is invalid
			{ 
				stateMachine.hasLastMousePressed = 0; 
				if (settingsView.window.showMouseCoordinates)
				{
					ShowMessage("show mouse coordinates turned of due to change of view", timeoutShowItem);
				}
				settingsView.window.showMouseCoordinates = false;
			} 
		}
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	}
}

//! Zoom all for OpenGL renderer
void GlfwRenderer::ZoomAll(Index viewID, bool updateGraphicsData, bool computeMaxScene, bool render)
{
	RenderState* state = renderViews.State(viewID);
	GLFWwindow* window = renderViews.GetWindow(viewID);

	if (updateGraphicsData) { UpdateGraphicsDataNow(); }
	if (computeMaxScene) 
	{ 
		state->ComputeMaxSceneSize(*visSettings, graphicsDataList);
	}
	stateMachine.hasLastMousePressed = 0;
	state->ComputeZoomAll(*visSettings);

	if (render) { Render(window); }
}

void GlfwRenderer::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	Index viewID = renderViews.GetViewID(window);
	RenderState* state = renderViews.State(viewID);

	if (PyGetRendererCallbackLock() || GetSettingsView(viewID, *visSettings).window.lockModelView) { return; }
	SetCallBackRefreshSignal();
	//rendererOut << "scroll: x=" << xoffset << ", y=" << yoffset << "\n";
	stateMachine.hasLastMousePressed = 0;

	bool ctrl = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
		glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;

	float zoomStep = visSettings->interactive.advanced.zoomStepFactor;
	if (ctrl) { zoomStep = pow(fabs(zoomStep), 0.1f); } //small zoom step

	if (yoffset*(double)zoomStep > 0) { state->zoom /= pow(fabs(zoomStep), fabs((float)yoffset)); }
	if (yoffset*(double)zoomStep < 0) { state->zoom *= pow(fabs(zoomStep), fabs((float)yoffset)); }
}

void GlfwRenderer::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	Index viewID = renderViews.GetViewID(window);
	RenderState* state = renderViews.State(viewID);

	if (PyGetRendererCallbackLock()) { return; }
	SetCallBackRefreshSignal();
	//EXUstd::WaitAndLockSemaphore(graphicsUpdateAtomicFlag);

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//STATE MACHINE MOUSE MOVE
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
	{
		bool isOrthoAxisAligned = (basicVisualizationSystemContainer->IsAxisAlignedView(*state) && GetSettingsView(viewID, *visSettings).camera.perspective == 0);

		if (isOrthoAxisAligned)
		{
			if (GetSettingsView(viewID, *visSettings).window.showMouseCoordinates)
			{
				Vector2D lastMouseOpenGL = basicVisualizationSystemContainer->PixelsToOpenGLCoordinates2D(
					stateMachine.lastMousePressedX, stateMachine.lastMousePressedY, *state);
				double dist = (state->openGLcoordinates - lastMouseOpenGL).GetL2Norm();
				Index prec = 4;
				STDstring s = "p" + EXUstd::ToString(int(stateMachine.hasLastMousePressed)) + //print index for better measuring
					": pos=[" + EXUstd::Num2String(state->openGLcoordinates[0], prec, true) + 
					"," + EXUstd::Num2String(state->openGLcoordinates[1], prec, true) + "]";
				if (stateMachine.hasLastMousePressed)
				{
					s += ", lastPos=[" + EXUstd::Num2String(lastMouseOpenGL[0], prec, true) + "," + EXUstd::Num2String(lastMouseOpenGL[1], prec, true) +
						"], dist=" + EXUstd::Num2String(dist, prec, false);
					stateMachine.hasLastMousePressed++; //increment as long as it is active!
				}
				else { stateMachine.hasLastMousePressed = 1; }
				if (visSettings->interactive.logMouseCoordinates) { PrintDelayed(s); }
			}
		}

		stateMachine.lastMousePressedX = stateMachine.mousePositionX;
		stateMachine.lastMousePressedY = stateMachine.mousePositionY; //now see if the mouse moves, then switch to move mode!
		stateMachine.mouseLeftPressed = true;
		state->mouseLeftPressed = true;
	}
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
	{
		//check, if it was a regular mouse press without moving
		if (stateMachine.lastMousePressedX == stateMachine.mousePositionX &&
			stateMachine.lastMousePressedY == stateMachine.mousePositionY &&
			visSettings->interactive.advanced.selectionLeftMouse &&
			!GetSettingsView(viewID, *visSettings).window.showMouseCoordinates) //but not done if mouse coordinates shown
		{
			//rendererOut << "mouse pressed!\n";
			//MouseSelect(window, stateMachine.mousePositionX, stateMachine.mousePositionY);
			stateMachine.selectionMouseCoordinates = state->mouseCoordinates;

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			Index itemID;
			//ItemType itemType;
			//Index mbsNumber;
			MouseSelect(viewID,
				(Index)stateMachine.selectionMouseCoordinates[0],
				(Index)stateMachine.selectionMouseCoordinates[1],
				itemID);
		}

		stateMachine.mouseLeftPressed = false;
		state->mouseLeftPressed = false;
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS && !stateMachine.mouseLeftPressed)
	{
		stateMachine.mouseRightPressed = true;
		stateMachine.lastMousePressedX = stateMachine.mousePositionX;
		stateMachine.lastMousePressedY = stateMachine.mousePositionY; //now see if the mouse moves, then switch to move mode!
	}
	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE)
	{
		//check, if it was a regular mouse press without moving
		if (stateMachine.lastMousePressedX == stateMachine.mousePositionX &&
			stateMachine.lastMousePressedY == stateMachine.mousePositionY &&
			visSettings->interactive.advanced.selectionRightMouse &&
			!PyGetRendererPythonCommandLock())
		{
			//rendererOut << "mouse pressed!\n";
			//MouseSelect(window, stateMachine.mousePositionX, stateMachine.mousePositionY);
			stateMachine.selectionMouseCoordinates = state->mouseCoordinates;

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			Index itemID;
			bool success = MouseSelect(viewID,
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

		stateMachine.mouseRightPressed = false;
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
	Index viewID = renderViews.GetViewID(window);
	RenderState* state = renderViews.State(viewID);

	if (PyGetRendererCallbackLock()) { return; }
	SetCallBackRefreshSignal();
	//rendererOut << "mouse cursor: x=" << xpos << ", y=" << ypos << "\n";
	stateMachine.mousePositionX = xpos;
	stateMachine.mousePositionY = ypos;

	////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//ShowMessage("cnt ="+EXUstd::ToString(cnt++), 5);
	////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	float height = (float)state->currentWindowSize[1];
	float factor = 2.f*state->zoom / height;

	state->mouseCoordinates = Vector2D({ xpos, ypos });
	state->openGLcoordinates = basicVisualizationSystemContainer->PixelsToOpenGLCoordinates2D(xpos, ypos, *state);
		//factor * Vector2D({ xpos - 0.5*state->currentWindowSize[0], -1.*(ypos - 0.5*state->currentWindowSize[1]) }) +
		//Vector2D({ (double)state->centerPoint[0], (double)state->centerPoint[1] });

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//MOUSE MOVE state machine:
	//check if one should switch to mouse move mode:
	double minMove = 2;
	if (stateMachine.mouseLeftPressed && stateMachine.mode == RendererMode::_None)
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
		stateMachine.hasLastMousePressed = 0; //do not show last point; it is invalid

		if (stateMachine.mouseLeftPressed && !GetSettingsView(viewID, *visSettings).window.lockModelView)
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
	if (stateMachine.mouseRightPressed && stateMachine.mode == RendererMode::_None)
	{
		stateMachine.hasLastMousePressed = 0; //do not show last point; it is invalid
		if (fabs(stateMachine.lastMousePressedX - xpos) >= minMove || fabs(stateMachine.lastMousePressedY - ypos) >= minMove)
		{
			stateMachine.mode = RendererMode::Rotate;
			stateMachine.storedModelRotation = state->modelRotation; //now store the current rotation of the modelview
		}
	}

	if (stateMachine.mode == RendererMode::Rotate)
	{
		stateMachine.hasLastMousePressed = 0; //do not show last point; it is invalid
		if (stateMachine.mouseRightPressed && !GetSettingsView(viewID, *visSettings).window.lockModelView)
		{
			//rotation shall be proportional to pixels
			float deltaX = (float)(xpos - stateMachine.lastMousePressedX);
			float deltaY = (float)(ypos - stateMachine.lastMousePressedY);
			float rotationFactor = visSettings->interactive.advanced.mouseMoveRotationFactor;

			//apply incremental rotation
			HomogeneousTransformationF previousModelView;
			previousModelView.SetHT44(stateMachine.storedModelRotation.GetTransposed());
			HomogeneousTransformationF newModelView;
			newModelView.SetRotation(EXUstd::degree2radF * Float3({ deltaY * rotationFactor, deltaX * rotationFactor, 0}) );
			newModelView *= previousModelView;

			//now store rotation in modelView and update centerPoint:
			//compute rotationCenterPoint update: only compute changes from previous modelRotation, 
			//  rest is already updated in centerPoint!!!
			previousModelView.SetHT44(state->modelRotation.GetTransposed());

			state->modelRotation = newModelView.GetHT44().GetTransposed(); //only contains rotation
			
			state->CenterPointUpdateFromRotationChange(previousModelView.GetRotation(), newModelView.GetRotation(), false); //false=use row-major

		}
		else { stateMachine.mode = RendererMode::_None; } //finish move operation if button is released!
	}
}

//! return true, if joystick available and updated values are available; if joystickNumber==EXUstd::InvalidIndex, chose a joystick; 
//! if joystickNumber!=EXUstd::InvalidIndex, it uses the fixed joystick until end of Renderer
bool GlfwRenderer::GetJoystickValues(Vector3D& position, Vector3D& rotation, Index& joystickNumber)
{
	bool initFirst = false; //if initialized first, also reset stateMachine
	if (joystickNumber == EXUstd::InvalidIndex)
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

void GlfwRenderer::DoAutoRotation(Index viewID)
{
	RenderState* state = renderViews.State(viewID);

	if (visSettings->interactive.autoRotateModelView && !doAutoRotate)
	{
		lastTimeAutoRotate = EXUstd::GetTimeInSeconds();
	}
	doAutoRotate = visSettings->interactive.autoRotateModelView;
	Float3 rot = visSettings->interactive.autoRotationVelocity;

	Real dt = EXUstd::GetTimeInSeconds() - lastTimeAutoRotate;
	//std::cout << "rotate, dt=" << dt << ", autorot=" << doAutoRotate << "\n";
	if (doAutoRotate && dt > 0.001)
	{
		lastTimeAutoRotate = EXUstd::GetTimeInSeconds();
		Matrix3DF incRot = EXUlie::ExpSO3((float)dt * rot);
		Matrix4DF transform;
		transform.SetScalarMatrix(4, 1.f);
		transform.SetSubmatrix(incRot, 0, 0);
		state->modelRotation = transform * state->modelRotation;
	}
}

//! read joystick values; if changed, send refresh signal for graphics
void GlfwRenderer::ProcessJoystick()
{
	//joysticks only affect opened windows which have focus
	ForEachWindowOpen([&](Index viewID)
		{
			RenderState* state = renderViews.State(viewID);
			GLFWwindow* window = renderViews.GetWindow(viewID);
			if (glfwGetWindowAttrib(window, GLFW_FOCUSED))
			{

				if (visSettings->interactive.useJoystickInput &&
					stateMachine.mode == RendererMode::_None && //only if no other move/zoom action ongoing!
					GetJoystickValues(state->joystickPosition, state->joystickRotation, state->joystickAvailable))
				{
					Vector3D diffPos = state->joystickPosition - stateMachine.storedJoystickPosition;
					Vector3D diffRot = state->joystickRotation - stateMachine.storedJoystickRotation;
					stateMachine.storedJoystickPosition = state->joystickPosition;
					stateMachine.storedJoystickRotation = state->joystickRotation;

					if (GetSettingsView(viewID, *visSettings).window.lockModelView) { return; }

					if (!(diffPos == 0. && diffRot == 0.))
					{
						SetCallBackRefreshSignal();
					}
					if (!(diffPos == 0.))
					{
						float fact = 2.f * state->zoom * visSettings->interactive.advanced.joystickScaleTranslation; //add more weight to translation in plane
						state->centerPoint[0] -= fact * (float)diffPos[0];
						state->centerPoint[1] += fact * (float)diffPos[1];

						state->zoom *= (1.f + visSettings->interactive.advanced.joystickScaleTranslation * (float)diffPos[2]);
						//ShowMessage("move: " + EXUstd::ToString(state->centerPoint[0]) +"," + EXUstd::ToString(state->centerPoint[1]));
					}
					if (!(diffRot == 0.))
					{
						diffRot *= visSettings->interactive.advanced.joystickScaleRotation;

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
		}
	);
}



//! zoom in to mouse position (x,y), used to render that area lateron (replacement for gluPickMatrix(...)
void GlfwRenderer::SetViewOnMouseCursor(GLdouble x, GLdouble y, GLdouble delX, GLdouble delY, GLint viewport[4])
{
	if (delX <= 0 || delY <= 0) 
	{ 
		ShowMessage("SetViewOnMouseCursor: not allowed with delX<=0 or delY<=0", 5);
		//CHECKandTHROWstring("SetViewOnMouseCursor: not allowed with delX<=0 or delY<=0"); //avoid exceptions in render thread
		return; 
	}

	/* Translate and scale the picked region to the entire window */
	glTranslated((viewport[2] - 2 * (x - viewport[0])) / delX,
		(viewport[3] - 2 * (y - viewport[1])) / delY, 0);
	glScaled(viewport[2] / delX, viewport[3] / delY, 1.0);
}

//! function to evaluate selection of items, show message, return dictionary string
bool GlfwRenderer::MouseSelect(Index viewID, Index mouseX, Index mouseY, Index& itemID)
{
	RenderState* state = renderViews.State(viewID);

	//if (verboseRenderer) { std::cout << "Mouse select" << std::flush; }
	float zDepth;
	MouseSelectOpenGL(viewID,
		(Index)stateMachine.selectionMouseCoordinates[0],
		(Index)stateMachine.selectionMouseCoordinates[1],
		itemID, zDepth);

	const Real timeOutHighlightItem = 0.5; //just short to exactly see object
	ItemID2IndexType(itemID, stateMachine.highlightIndex, stateMachine.highlightType, stateMachine.highlightMbsNumber);

	state->mouseSelectionMbsNumber = stateMachine.highlightMbsNumber;
	state->mouseSelectionItemType = stateMachine.highlightType;
	state->mouseSelectionItemID = stateMachine.highlightIndex;
	state->mouseSelectionZdepth = zDepth;

	//if (verboseRenderer) { std::cout << "  select=" << EXUstd::ToString(itemID) << std::flush; }

	//PrintDelayed("itemID=" + EXUstd::ToString(itemID));

	if (stateMachine.highlightType != ItemType::_None && stateMachine.highlightIndex != EXUstd::InvalidIndex)
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
void GlfwRenderer::MouseSelectOpenGL(Index viewID, Index mouseX, Index mouseY, Index& itemID, float& zDepth)
{
	RenderState* state = renderViews.State(viewID);
	GLFWwindow* window = renderViews.GetWindow(viewID);

	//++++++++++++++++++++++++++++++++++++++++
	//put into separate function, for Render(...)
	int width, height;

	//if (verboseRenderer) { std::cout << "  MouseSelectOpenGL" << std::flush; }
	glfwGetFramebufferSize(window, &width, &height);

	//rendererOut << "current window: width=" << width << ", height=" << height << "\n";
	state->currentWindowSize[0] = width;
	state->currentWindowSize[1] = height;

	SetRenderStateScreenSize(viewID, width, height);
	float ratio = height != 0 ? width / (float)height : (float)width;
	float zoom = state->zoom;

	//++++++++++++++++++++++++++++++++++++++++

	//if (verboseRenderer) { std::cout << "  glSelectBuffer" << std::flush; }
	const Index selectBufferSize = 1000; //size for number of objects picked at same time
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
	SetProjection(viewID, width, height, ratio, zoom); //set zoom, perspective, ...; may not work for larger perspective

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	SetModelRotationTranslation(viewID);

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
	RenderGraphicsData(viewID, selectionMode); //render scene with names
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
	Index tempItemIndex;
	Index tempItemMbsNumber;
	ItemType tempItemType;
	Index itemTypeIndex;
	for (Index i = 0; i < numberOfItemsFound; i++)
	{
		GLuint currentItemID = selectBuffer[4 * i + 3];
		GLuint curdepth = selectBuffer[4 * i + 1];
		
		ItemID2IndexType(currentItemID, tempItemIndex, tempItemType, tempItemMbsNumber);
		itemTypeIndex = (Index)tempItemType;
		itemTypeIndex = itemTypeIndex == 0 ? 0 : 1 << (itemTypeIndex-1); //convert into binary flags

		if (currentItemID != 0
			&& (curdepth < minimalDepth || !itemIDnearest)
			&& EXUstd::IsOfTypeAndNotNone(visSettings->interactive.advanced.selectionLeftMouseItemTypes, itemTypeIndex)
			)
		{
			minimalDepth = curdepth;
			itemIDnearest = currentItemID;
		}
	}

	itemID = itemIDnearest;
	zDepth = (float)minimalDepth/(float)(4294967295); //2^32-1
	//ItemID2IndexType(itemIDnearest, itemIndex, itemType, mbsNumber); //itemType==_None, if no item found
	//rendererOut << "selected item: " << itemIndex << ", type=" << itemType << "\n";
}

//! start renderer
//! called from main Python thread
bool GlfwRenderer::StartRenderer(Index verbose)
{
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//initializat and detect running renderer
	verboseRenderer = verbose;
	rendererTasksCount = 0;

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

		// initializes renderState - now with settings already given; 
		// this is done to make OpenGL zoom and maxSceneCoordinates work; 
		// MUST always be done due to conformity with earlier functionality 
		// otherwise, would not reset model rotation, etc.; 
		// if user likes to preserve renderstate->copy before and set after start!
		basicVisualizationSystemContainer->InitializeRenderState(true); 

		//*** further initialization ***
		basicVisualizationSystemContainer->SetComputeMaxSceneRequest(true); //computes maxSceneCoordinates for perspective and shadow
		for (Index viewID = 0; viewID < renderViews.NumberOfViews(); ++viewID)
		{
			basicVisualizationSystemContainer->SetZoomAllRequest(viewID, visSettings->general.autoFitScene);
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
			Index timeOut = visSettings->general.rendererStartupTimeout / 10;

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
					//here we are still in the main Python thread, so we can call SysError
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
//! called from main Python thread
void GlfwRenderer::StopRenderer()
{
	if (renderViews.GetWindow(0, false)) //only first window needed
	{
		stopRenderer = true;
		glfwSetWindowShouldClose(renderViews.GetWindow(mainViewID, false), GL_TRUE); //only main window is ok

		if (useMultiThreadedRendering)
		{
			Index timeOut = 100; //1 second (can also hang longer ...)
			bool raytracerEnabled = false;
			ForEachEnabledView([&](Index viewID) {
				if (GetSettingsView(viewID, *visSettings).camera.useRaytracer) { raytracerEnabled = true; }
				});
			if (raytracerEnabled) {timeOut = 2000;} //2026-01-03: increased to 20s for raytracer, which should capture most scenarios

			Index i = 0;
			while (i++ < timeOut && rendererActive) //wait 5 seconds for thread to answer
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}

			if (rendererActive) { SysError("OpenGL Renderer could not be stopped safely\n"); }

			//glfwDestroyWindow(window); //HAS TO BE CALLED FROM GLFWClient THREAD
			//not necessary: glfwTerminate(); //test if this helps; should not be needed

			renderViews.SetWindow(0, nullptr); //this is used to identify if window has already been generated

			//after this command, this thread is terminated! ==> nothing will be done any more
			if (rendererThread.joinable()) //thread is still running from previous call ...
			{
				if (verboseRenderer) { outputBuffer.WriteVisualization("renderer.Stop(): second thread join main thread ...\n"); }

				rendererThread.join();
				if (verboseRenderer) { outputBuffer.WriteVisualization("  ... joined\n"); }
				//not necessary: rendererThread.~thread(); //check if this is necessary/right ==> will not be called after .joint() ...
			}
		}
		else
		{
			FinishRunLoop(); //shut down manually
		}
	}
	else
	{
		if (useMultiThreadedRendering)
		{
			if (rendererThread.joinable()) //thread is still running from previous call ...
			{
				if (verboseRenderer) { outputBuffer.WriteVisualization("renderer.Stop(): window already closed; now second thread join main thread ...\n"); }
				//pout << "join thread ...\n";
				rendererThread.join();
				if (verboseRenderer) { outputBuffer.WriteVisualization("  ... joined\n"); }
			}
		}
	}
}

//! Initializes and starts viewID=0, 1, 2, ...; HAS TO BE CALLED FROM GLFWClient THREAD
bool GlfwRenderer::CreateViewWindow(Index viewID)
{
	//NOTE: view 0 may only be called from InitCreateWindow!!!

#ifndef __EXUDYN__APPLE__
	if (visSettings->openGL.multiSampling >= 2 || visSettings->openGL.multiSampling <= 4 || visSettings->openGL.multiSampling == 8 || visSettings->openGL.multiSampling == 16) //only 4 is possible right now ... otherwise no multisampling
	{
		glfwWindowHint(GLFW_SAMPLES, (int)visSettings->openGL.multiSampling); //multisampling=4, means 4 times larger buffers! but leads to smoother graphics
		glEnable(GL_MULTISAMPLE); //usually activated by default, but better to have it anyway
		if (verboseRenderer) { PrintDelayed("enable GL_MULTISAMPLE"); }
	}
#endif

	if (GetSettingsView(viewID, *visSettings).window.alwaysOnTop)
	{
		glfwWindowHint(GLFW_FLOATING, GLFW_TRUE); //GLFW_FLOATING (default: GLFW_FALSE)  specifies whether the windowed mode window will be floating above other regular windows, also called topmost or always - on - top.This is intended primarily for debugging purposes and cannot be used to implement proper full screen windows.Possible values are GLFW_TRUE and GLFW_FALSE.
		if (verboseRenderer) { PrintDelayed("enable GLFW_FLOATING"); }
	}
	//glfwWindowHint(GLFW_FOCUSED, GLFW_TRUE); //(default: GLFW_TRUE) specifies whether the windowed mode window will be given input focus when created. Possible values are GLFW_TRUE and GLFW_FALSE.
	//GLFW_FOCUS_ON_SHOW (default: GLFW_TRUE) specifies whether the window will be given input focus when glfwShowWindow is called. Possible values are GLFW_TRUE and GLFW_FALSE
	//GLFW_SCALE_TO_MONITOR (default: GLFW_FALSE) specified whether the window content area should be resized based on the monitor content scale of any monitor it is placed on. This includes the initial placement when the window is created. Possible values are GLFW_TRUE and GLFW_FALSE.

	const int minWidth = 2; //avoid zero size
	const int minHeight = 2; //avoid zero size
	const int maxWidth = 2 * 8192; //limit upper size to 16K, 16:9 for now ...
	const int maxHeight = 2 * 4608; //limit upper size to 16K, 16:9 for now ...

	int sizex = EXUstd::Clamp((int)renderViews.State(viewID)->currentWindowSize[0], minWidth, maxWidth);
	int sizey = EXUstd::Clamp((int)renderViews.State(viewID)->currentWindowSize[1], minHeight, maxHeight);

	STDstring title = (viewID == mainViewID) ? "Exudyn OpenGL Main View 0" : STDstring("Exudyn View ") + EXUstd::ToString(viewID);

	GLFWwindow* parentWindow = (viewID == mainViewID) ? NULL : renderViews.GetWindow(mainViewID);
	GLFWwindow* window = glfwCreateWindow(sizex, sizey, title.c_str(), NULL, parentWindow);

	if (!window)
	{
		PrintDelayed(STDstring("GLFWRenderer::CreateViewWindow: window could not be created for view ")+EXUstd::ToString(viewID));
		renderViews.SetWindow(viewID, NULL);
		renderViews.State(viewID)->windowOpen = false;
		renderViews.State(viewID)->viewEnabled = false;
		return false;
	}
	else
	{
		if (verboseRenderer) { PrintDelayed(STDstring("glfwCreateWindow(...) successful for view ") + EXUstd::ToString(viewID)); }
		renderViews.SetWindow(viewID, window);
		renderViews.State(viewID)->windowOpen = true;
		renderViews.State(viewID)->viewEnabled = true;
	}


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// determine the windows scale; 
	float xWindowScale = 1;
	float yWindowScale = 1;
#if !defined(__EXUDYN__LINUX__) //glfwGetWindowContentScale() crashes on Ubuntu18.04 and 20.04 compilation
	glfwGetWindowContentScale(window, &xWindowScale, &yWindowScale);
#endif
	SetContentScaling(viewID, xWindowScale, yWindowScale); //must be done before initialization of fonts

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//allow for very small windows, but never get 0 ...; x-size anyway limited due to windows buttons
	glfwSetWindowSizeLimits(window, 2, 2, maxWidth, maxHeight);
	if (!visSettings->general.limitWindowToScreenSize)
	{
		glfwSetWindowSize(window, sizex, sizey); //this call ensures that window size is really the requested size; tested on windows
		//outputBuffer.WriteVisualization("window size=" + EXUstd::ToString(sizex) + ", " + EXUstd::ToString(sizey));
	}

	if (verboseRenderer) { PrintDelayed("glfwSetWindowSizeLimits(...) successful"); }

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	//Needed here (otherwise no overlay text ...):
	glfwMakeContextCurrent(window); //seems to be required to activate content upon window creation ...?
	if (verboseRenderer) { PrintDelayed("glfwMakeContextCurrent(...) successful"); }

	//+++++++++++++++++
	//initialization of important global OpenGL parameters once every view:
	glClearDepth(1.0f); //depth cleared to far plane
	glEnable(GL_DEPTH_TEST); //enables depth, which is default to draw objects behind others
	glEnable(GL_NORMALIZE);  //for normals that are not normalized ... adjusts them to avoid strange light effects (performance issue, but not for modern GPUs)
	//+++++++++++++++++

	//+++++++++++++++++++++++++++++++++
	//joystick
	renderViews.State(viewID)->joystickAvailable = EXUstd::InvalidIndex; //this causes to search for new joystick and, if found, initialize stateMachine!
	ResetStateMachine(); //on each new view, we reset state machine!

	return true;
}

//! Stops and removes a viewID=1,2, ...; MAY BE CALLED FROM ANY THREAD
void GlfwRenderer::CloseViewWindow(Index viewID)
{
	//close only if 
	if (renderViews.GetWindow(viewID, false))
	{
		stopRenderer = true;
		glfwSetWindowShouldClose(renderViews.GetWindow(viewID, false), GL_TRUE); //causes window to be closed
	}
	if (renderViews.IsEnabledView(viewID))
	{
		renderViews.SetEnabledView(viewID, false); //this causes the view to get inactive
	}
}



void GlfwRenderer::SetContentScaling(Index viewID, float xScale, float yScale)
{
	float fontScaleOld = GetFontScaling(viewID);
	if (visSettings->general.useWindowsDisplayScaleFactor)
	{
		SetFontScaling(viewID, 0.5f*(xScale + yScale) ); //simplified for now!
	} else {SetFontScaling(viewID, 1); }

	if (GetFontScaling(viewID) != fontScaleOld)
	{
		ShowMessage(STDstring("Font size adjusted to monitor scaling for view ") + EXUstd::ToString(viewID), 3.);
	}
}

float GlfwRenderer::GetFontScaling(Index viewID)
{
	if (renderViews.State(viewID, false) != nullptr)
	{
		return renderViews.State(viewID, true)->displayScaling;
	}
	else { return 1; }
}

void GlfwRenderer::SetFontScaling(Index viewID, float scaling)
{
    if (renderViews.State(viewID,false) != nullptr)
    {
#if defined(__EXUDYN__LINUX__)
		renderViews.State(viewID, false)->displayScaling = scaling * visSettings->general.linuxDisplayScaleFactor;
#else
		renderViews.State(viewID, false)->displayScaling = scaling;
#endif
    }
}

//! this is the create function for window0/view0
void GlfwRenderer::InitCreateWindow()
{
	try
	{
		Index viewID = mainViewID;
		
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// common GLFW startup

		//this is now running in separate thread, can only output to rendererOut.
		if (verboseRenderer) { PrintDelayed("InitCreateWindow"); }
		glfwSetErrorCallback(error_callback);

		if (!glfwInit())
		{
			if (verboseRenderer) { PrintDelayed("glfwInit failed"); }

			rendererError = 1;
			exit(EXIT_FAILURE);
		}

		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//special behavior for main view:
		if (!CreateViewWindow(viewID))
		{
			rendererError = 2;
			glfwTerminate();
			if (useMultiThreadedRendering)
			{
				exit(EXIT_FAILURE); //stop task
			}
			CHECKandTHROWstring("GLFWRenderer::InitCreateWindow: Render window could not be created"); //SysError requires Python and crashes!
		}
		GLFWwindow* window = renderViews.GetWindow(viewID, false);



#ifdef __EXUDYN_USE_OPENVR
		if (verboseRenderer) { PrintDelayed("Initialize OpenVR"); }
		if (visSettings->interactive.openVR.enable) 
		{ 
			glfwOpenVRinterface.SetLogLevel(visSettings->interactive.openVR.logLevel);
			state->openVRstate.isActivated = glfwOpenVRinterface.InitOpenVR(&glfwRenderer);
		}
#endif

		guint fontSize = (guint)(GetSettingsView(viewID, *visSettings).window.globalFontSize * GetFontScaling(viewID)); //use this size for fonts throughout
		Real timeFont = EXUstd::GetTimeInSeconds();
		InitFontBitmap(fontSize); //fontSize only used in old bitmap mode!
		timeFont = EXUstd::GetTimeInSeconds() - timeFont;
		if (verboseRenderer) { PrintDelayed("InitFontBitmap(...) successful (took "+EXUstd::ToString(std::round(timeFont*1000.)/1000.)+" seconds)"); }

		InitGLlists();
		if (verboseRenderer) { PrintDelayed("InitGLlists(...) successful"); }

		//+++++++++++++++++++++++++++++++++
		//depending on flags, do some changes to window
		if (GetSettingsView(viewID, *visSettings).window.showWindow)
		{
			glfwShowWindow(window); //show the window when created ... should by anyway done, but did not work in Spyder so far
		}
		else
		{
			glfwIconifyWindow(window); //iconify window
		}
		if (GetSettingsView(viewID, *visSettings).window.maximize)
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
		if (verboseRenderer) { PrintDelayed("InitCreateWindow finished: Ready to update window using renderer.DoIdleTasks(...)"); }
	}
}

void GlfwRenderer::RunLoop()
{
	try
	{
		while (rendererActive && 
			!glfwWindowShouldClose(renderViews.GetWindow(mainViewID)) && //loop ends when main window is closed ...
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
			ProcessJoystick(); //done per view
        }
	}

	//in any case multithreaded or not:
	for (Index viewID = 1; viewID < renderViews.NumberOfViews(); viewID++)
	{
		if (renderViews.IsValidWindow(viewID) && glfwWindowShouldClose(renderViews.GetWindow(viewID)))
		{
			glfwDestroyWindow(renderViews.GetWindow(viewID)); //should be called from main thread, but also works this way!
			renderViews.SetWindow(viewID, nullptr);
			renderViews.State(viewID)->windowOpen = false;
			//now we still can use the view, similarly to main view after renderer.Stop(); DON'T SET: renderViews.State(viewID)->viewEnabled = false;
			if (verboseRenderer) { PrintDelayed(STDstring("Close window of view ")+EXUstd::ToString(viewID)); }
		}
		if (renderViews.GetWindowShouldBeCreated(viewID))
		{
			if (renderViews.IsValidWindow(viewID))
			{
				PrintDelayed(STDstring("Received GetWindowShouldBeCreated signal while window already exists for view ") + EXUstd::ToString(viewID));
			}
			else
			{
				bool success = CreateViewWindow(viewID); //CreateViewWindow already does the error-message
				glfwRenderer.GetRenderViews()->SetWindowShouldBeCreated(viewID, false);
				if (verboseRenderer && success) { PrintDelayed(STDstring("Created window for view ") + EXUstd::ToString(viewID)); }
			}
		}
	}

	if (useMultiThreadedRendering ||
		(time >= lastGraphicsUpdate + updateInterval) ||
		GetCallBackRefreshSignal() ||
		graphicsUpdateAndRender)
	{
		basicVisualizationSystemContainer->UpdateGraphicsData();
		bool maxSceneComputed = false;
		if (basicVisualizationSystemContainer->GetComputeMaxSceneRequest())
		{
			ForEachEnabledView([&](Index viewID) {
				RenderState* state = renderViews.State(viewID);
				state->ComputeMaxSceneSize(*visSettings, graphicsDataList);
				});

			maxSceneComputed = true;
			basicVisualizationSystemContainer->SetComputeMaxSceneRequest(false);
		}
		ForEachEnabledView([&](Index viewID) {
				if (basicVisualizationSystemContainer->GetAndResetZoomAllRequest(viewID))
				{
					ZoomAll(viewID, false, !maxSceneComputed, false);
				}
			});
		DoAutoRotation(mainViewID); //currently available only for main view
		
		ForEachWindowOpen([&](Index viewID) {
			GLFWwindow* window = renderViews.GetWindow(viewID);
			Render(window);
			SaveImage(viewID); //in case of flag, save frame to image file
			});

#ifdef __EXUDYN_USE_OPENVR
		if (glfwOpenVRinterface.IsActivated())
		{
			glfwOpenVRinterface.RenderAndUpdateDevices();
		}
#endif
		lastGraphicsUpdate = time;
		SetCallBackRefreshSignal(false);
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
	rendererTasksCount++;
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

	if (renderViews.HasValidWindows())
	{
#ifdef __EXUDYN_USE_OPENVR
        glfwOpenVRinterface.ShutDown();
#endif
		//! first close sub-views
		for (Index viewID = 1; viewID < renderViews.NumberOfViews(); viewID++)
		{
			if (renderViews.IsValidWindow(viewID))
			{
				glfwDestroyWindow(renderViews.GetWindow(viewID)); //should be called from main thread, but also works this way!
				renderViews.SetWindow(viewID, nullptr);
			}
		}
		glfwDestroyWindow(renderViews.GetWindow(mainViewID)); //HAS TO BE CALLED FROM GLFWClient THREAD
		renderViews.SetWindow(mainViewID, nullptr);
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
			!glfwWindowShouldClose(renderViews.GetWindow(mainViewID)) && //only check for main window
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
				basicVisualizationSystemContainer->DoSingleIdleOperation(); //this calls the Python functions, which is ok, because DoRendererIdleTasks() called from Python!
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
			!glfwWindowShouldClose(renderViews.GetWindow(mainViewID)) && //only check for main window
			!stopRenderer &&
			!globalPyRuntimeErrorFlag))
		{
			FinishRunLoop();
		}
	}
}

//load GL_PROJECTION and set according to zoom, perspective, etc.
void GlfwRenderer::SetProjection(Index viewID, int width, int height, float ratio, float zoom)
{
	RenderState* state = renderViews.State(viewID);
	const VSettingsView& settingsView = GetSettingsView(viewID, *visSettings);

	const Matrix4DF& P = state->projectionMatrix;
	float maxSize = state->maxSceneSize;
	float perspective = settingsView.camera.perspective;
	float zFactor = visSettings->openGL.zMaxSceneFactor;

	if (P(0, 0) == 1.f && P(1, 1) == 1.f && P(2, 2) == 1.f && P(3, 3) == 1.f) //in this case, no projection has been provided
	{
		Float3 nearFar = settingsView.camera.nearFarPlaneOffset;
		float zNear = -zFactor * 2.f * maxSize;
		float zFar = zFactor * 2.f * maxSize;
		if (perspective <= 0)
		{
			if (nearFar[2] == 1.f) { zNear = nearFar[0]; zFar = nearFar[1]; }
			//https: //www.khronos.org/opengl/wiki/Viewing_and_Transformations#How_do_I_implement_a_zoom_operation.3F
			//void glOrtho(GLdouble left,GLdouble right,GLdouble bottom,GLdouble top,GLdouble nearVal,GLdouble farVal);
			glOrtho(- ratio * zoom, ratio * zoom, -zoom, zoom, zNear, zFar);
		}
		else
		{
			float zNearMin = state->GetProjectionNearMin();
			//perspective
			float distance = EXUstd::Maximum(1e-6f, zoom / perspective);

			//zNear must be > 0 for glFrustum
			//choose a value that ensures the whole scene is visible
			float zNear;
			float zFar;
			float scale;

			if (settingsView.camera.modelCentricView) 
			{
				//model centric: zNear pushes out to surround the model
				zNear = distance - (zFactor * maxSize);
				zFar = distance + (zFactor * maxSize);
				if (nearFar[2] == 1.f) { zNear = nearFar[0] + distance; zFar = nearFar[1] + distance; }

				//safety cap to avoid inversion and avoid very small values
				//for camera centric view, if we reach zNearMin, we can increase zoom!
				if (zNear < zNearMin) { zNear = zNearMin; }

				scale = zNear / distance;
			}
			else 
			{
				//camera-centric: near plane is the camera lens. 
				//must be strictly > 0 for glFrustum to make sense:
				zNear = zNearMin;
				zFar = (zFactor * maxSize);
				if (nearFar[2] == 1.f) { zNear = nearFar[0]; zFar = nearFar[1]; }

				//safety cap to avoid inversion and avoid very small values
				//for camera centric view, if we reach zNearMin, we can increase zoom!
				if (zNear < zNearMin) { zNear = zNearMin; }

				scale = zNear * perspective;
			}


			//scale the Ortho-bounds to the Near Plane
			//relation: frustumValue = orthoValue * (zNear / distance)
			float left = -ratio * zoom * scale;
			float right = ratio * zoom * scale;
			float bottom = -zoom * scale;
			float top = zoom * scale;

			glFrustum(left, right, bottom, top, zNear, zFar);

			// apply Z-offset pushback only for the model centric view.
			if (settingsView.camera.modelCentricView) 
			{
				glTranslatef(0.f, 0.f, -distance);
			}
		}
	}
	else //openVR
	{
		if (state->projectionInfo == 0) //for companion window
		{
			glOrtho(-ratio * zoom, ratio*zoom, -zoom, zoom, -zFactor * 2.*maxSize, zFactor * 2.*maxSize); //https: //www.khronos.org/opengl/wiki/Viewing_and_Transformations#How_do_I_implement_a_zoom_operation.3F
			glMultMatrixf(state->projectionMatrix.GetDataPointer());
		}
		else if (state->projectionInfo == 1) //load matrix
		{
			glLoadMatrixf(state->projectionMatrix.GetDataPointer());
		}
		else if (state->projectionInfo == 2) //apply matrix after glOrtho
		{
			//additional projection has been provided and is added after glOrtho
			glOrtho(-ratio * zoom, ratio*zoom, -zoom, zoom, -zFactor * 2.*maxSize, zFactor * 2.*maxSize); //https: //www.khronos.org/opengl/wiki/Viewing_and_Transformations#How_do_I_implement_a_zoom_operation.3F
			//glOrtho(-ratio , ratio, -1, 1, 0.1, 30); //https: //www.khronos.org/opengl/wiki/Viewing_and_Transformations#How_do_I_implement_a_zoom_operation.3F
			//glLoadMatrixf(state->projectionMatrix.GetDataPointer());
			//glTranslatef(0.f, 0.f, -maxSize);
			glMultMatrixf(state->projectionMatrix.GetDataPointer());
		}

	}
}

//! set model view rotation and translation, unified for Render and mouse select
void GlfwRenderer::SetModelRotationTranslation(Index viewID)
{
	RenderState* state = renderViews.State(viewID);
	const VSettingsView& settingsView = GetSettingsView(viewID, *visSettings);

	//++++++++++++++++++++++++++++++++++++++++++++++++++++
	//model rotation and translation, include rotation center point
	Matrix4DF A;
	Matrix3DF rotationMV;
	Float3 translationMV;

	state->GetRotationTranslationFWithMarker(rotationMV, translationMV, 
		basicVisualizationSystemContainer, settingsView);

	A = state->modelRotation; //copy; only modify in case of trackMarker
	//map rotation matrix to part of 16 components in A
	A.SetSubmatrix(rotationMV, 0, 0);
	//for (Index i = 0; i < 3; i++)
	//{
	//	for (Index j = 0; j < 3; j++)
	//	{
	//		A(i, j) = rotationMV(i, j);
	//	}
	//}

	//we update the current transformation, but we do not update state->modelRotation / centerPoint 
	// (only done with view transformations using mouse/keys)
	glTranslatef(-translationMV[0], -translationMV[1], -translationMV[2]);
	glMultMatrixf(A.GetDataPointer());

	////inverse:
	//glTranslatef(translationMV[0], translationMV[1], translationMV[2]);
	//glMultMatrixf(A.GetTransposed().GetDataPointer());
}

void GlfwRenderer::Render(GLFWwindow* window) //GLFWwindow* needed in argument, because of glfwSetWindowRefreshCallback
{
	Index viewID = renderViews.GetViewID(window);
	RenderState* state = renderViews.State(viewID);

	if (PyGetRendererCallbackLock()) { return; }
	EXUstd::WaitAndLockSemaphore(renderFunctionRunning); //lock Render(...) function, no second call possible

	//+++++++++++++++++
	//activate openGL context
	glfwMakeContextCurrent(window);

	int width, height;

	GetWindowSize(window, width, height);
	SetRenderStateScreenSize(viewID, width, height);
	height = height ? height : 1;
	float ratio = width / (float)height;
	float zoom = state->zoom;

	//float fontSizeScaled = visSettings->general.textSize * GetFontScaling(); //use this size for fonts throughout

	if (GetSettingsView(viewID, *visSettings).camera.useRaytracer)
	{
		glfwSetWindowRefreshCallback(window, NULL); //resolve problems with timeouts
		//RenderStateMachine stateMachine; //if called from outside GlfwClient
		//stateMachine.Reset();
		raytracer.SoftwareRenderer(viewID, basicVisualizationSystemContainer, stateMachine);

		glfwSetWindowRefreshCallback(window, Render); //put it back

		//this is needed for drawing texts; 
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	}
	else
	{
		Render3Dobjects(viewID, width, height, ratio, zoom);

		//do this always, e.g. in openVR case or if projection is modified:
		//for texts, axes, etc.: draw without perspective
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-ratio * zoom, ratio * zoom, -zoom, zoom, 
			-visSettings->openGL.zMaxSceneFactor * 2.f * state->maxSceneSize, 
			visSettings->openGL.zMaxSceneFactor * 2.f * state->maxSceneSize); //https: //www.khronos.org/opengl/wiki/Viewing_and_Transformations#How_do_I_implement_a_zoom_operation.3F

		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		if (GetSettingsView(viewID, *visSettings).window.showComputationInfo ||
			GetSettingsView(viewID, *visSettings).window.showMouseCoordinates ||
			GetSettingsView(viewID, *visSettings).window.showRenderStateInfo ||
			(visSettings->contour.advanced.showColorBar && visSettings->contour.outputVariable != OutputVariableType::_None) ||
			GetSettingsView(viewID, *visSettings).scene.drawCoordinateSystem)
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			SetGLdepthMask(GL_FALSE); //draw system information and coordinate system always in front

			//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//new approach 2025-12 with static objects that are pre-computed
			GetGraphicsDataStatic(viewID, basicVisualizationSystemContainer, graphicsDataStatic, stateMachine, true);

			float offsetZ = 0.95f * visSettings->openGL.zMaxSceneFactor * 2.f * state->maxSceneSize; //absolute offset for lines and triangles
			bool transparentText = true;
			float offsetZrelative = 0.95f * visSettings->openGL.zMaxSceneFactor * 2.f; //NEW: offset relative to maxSceneSize (mult. inside DrawString)
			for (const GLText& text : graphicsDataStatic.glTexts)
			{
				float textFontSizeScaled = text.fontSize ? text.fontSize * state->displayScaling : GetSettingsView(viewID, *visSettings).window.globalFontSize * state->displayScaling;
				//float textFontSize = text.fontSize ? text.fontSize : visSettings->general.textSize;
				//std::cout << "text:" << text.text << ":"  << text.point << "\n";
				DrawString(viewID, text.text, textFontSizeScaled, text.point,
					Float3({ text.offsetX, text.offsetY, offsetZrelative }),
					text.color,
					transparentText);
			}

			glLineWidth(visSettings->openGL.lineWidth);
			if (visSettings->openGL.advanced.lineSmooth) { glEnable(GL_LINE_SMOOTH); }
			SetGLdepthMask(GL_TRUE);
			for (const GLLine& item : graphicsDataStatic.glLines)
			{
				glBegin(GL_LINES);
				glColor4f(item.color1[0], item.color1[1], item.color1[2], item.color1[3]);
				glVertex3f(item.point1[0], item.point1[1], item.point1[2] + offsetZ);
				glColor4f(item.color2[0], item.color2[1], item.color2[2], item.color2[3]);
				glVertex3f(item.point2[0], item.point2[1], item.point2[2] + offsetZ);
				glEnd();
			}

			//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//light settings for 3D object overlay (coordinate system, etc.)
			//triangles: static unshaded and static shaded
			glShadeModel(GL_SMOOTH);

			float a1 = 0.f; //turned off because now using global ambient light: GL_LIGHT_MODEL_AMBIENT
			float d1 = 0.9f;
			float s1 = 0.5f;
			float ambientLight0[] = { a1, a1, a1, 1.0f };
			float diffuseLight0[] = { d1, d1, d1, 1.0f };
			float specularLight0[] = { s1, s1, s1, 1.0f };
			float light0position[] = { 0.f,0.f,-10.f, 0.0f };
			float materialSpecularAmbient[] = { 0.7f,0.7f,0.7f,1.f };

			// Assign created components to GL_LIGHT0
			//only glLightfv works properly, not glLightf ...:
			glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight0);
			glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight0);
			glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight0);
			glLightfv(GL_LIGHT0, GL_POSITION, light0position);

			glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 1.f);
			glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0);
			glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0);
			glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, true);
			glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, true);

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//MATERIAL and Light model:
			glEnable(GL_COLOR_MATERIAL); //otherwise colors are grey
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 32.f);
			glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, materialSpecularAmbient);
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, materialSpecularAmbient);

			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);


			glEnable(GL_LIGHT0);
			glDisable(GL_LIGHT1);
			glDisable(GL_LIGHT2);
			glDisable(GL_LIGHT3);


			//arrows would require light in camera frame, position-independent (0,0,1,0) ?
			//add second static flag to enable lighting for such objects!
			bool lightsOn = false;
			if (visSettings->openGL.advanced.enableLighting) { glDisable(GL_LIGHTING); }
			for (const GLTriangle& item : graphicsDataStatic.glTriangles)
			{
				bool isShaded = item.itemID <= itemIDstaticObjectShaded;
				float localOffset = (item.itemID == itemIDstaticObjectWithoutZoff) ? 0.f : offsetZ; //these objects are not on top!
				if (visSettings->openGL.advanced.enableLighting)
				{
					if (isShaded && !lightsOn) { glEnable(GL_LIGHTING); lightsOn = true; }
					else if (!isShaded && lightsOn) { glDisable(GL_LIGHTING); lightsOn = false; }
				}

				glBegin(GL_TRIANGLES);
				glColor4f(item.colors[0][0], item.colors[0][1], item.colors[0][2], item.colors[0][3]);
				if (isShaded) { glNormal3fv(item.normals[0].GetDataPointer()); }
				glVertex3f(item.points[0][0], item.points[0][1], item.points[0][2] + localOffset);

				glColor4f(item.colors[1][0], item.colors[1][1], item.colors[1][2], item.colors[2][3]);
				if (isShaded) { glNormal3fv(item.normals[1].GetDataPointer()); }
				glVertex3f(item.points[1][0], item.points[1][1], item.points[1][2] + localOffset);

				glColor4f(item.colors[2][0], item.colors[2][1], item.colors[2][2], item.colors[2][3]);
				if (isShaded) { glNormal3fv(item.normals[2].GetDataPointer()); }
				glVertex3f(item.points[2][0], item.points[2][1], item.points[2][2] + localOffset);
				glEnd();
			}
			if (visSettings->openGL.advanced.enableLighting) { glDisable(GL_LIGHTING); }
			//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		}
		if (graphicsDataList)
		{
			if (graphicsDataList->NumberOfItems() > 1 &&
				visSettings->contour.advanced.showColorBar && visSettings->contour.outputVariable != OutputVariableType::_None)
			{
				ShowMessage("WARNING: contour plot color bar only works for one single MainSystem (mbs)");
			}
		}
	}
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//this only has to be done in GlfwClient
	if (stateMachine.renderMessageTimeout != 0. &&
		stateMachine.renderMessageTimeout < EXUstd::GetTimeInSeconds() &&
		stateMachine.rendererMessage.size())
	{
		stateMachine.rendererMessage = "";
	}

	glfwSwapBuffers(window);

	//++++++++++++++++++++++++++++++++++++++++++
	EXUstd::ReleaseSemaphore(renderFunctionRunning);

}

void GlfwRenderer::SetRenderStateScreenSize(Index viewID, int screenWidth, int screenHeight)
{
	RenderState* state = renderViews.State(viewID);
	state->currentWindowSize[0] = screenWidth;
	state->currentWindowSize[1] = screenHeight;
}

void GlfwRenderer::Render3Dobjects(Index viewID, int screenWidth, int screenHeight, float screenRatio, float zoom)
{
	RenderState* state = renderViews.State(viewID);

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

	SetProjection(viewID, screenWidth, screenHeight, screenRatio, zoom); //set zoom, perspective, ...

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glEnable(GL_DEPTH_TEST); //should be always enabled for correct depth
	glEnable(GL_BLEND);

	AddGradientBackground(zoom, screenRatio); //draw first, as it is drawn without depth test at the very background
	SetGLLights(viewID);

	SetModelRotationTranslation(viewID);

    RenderSensorTraces(viewID);
    RenderGraphicsData(viewID);
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




void GlfwRenderer::SaveImage(Index viewID)
{
	//at this time, the scene must have been rendered (called directly from render loop after Render() )
	if (basicVisualizationSystemContainer->SaveImageRequest(viewID))
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

		SaveSceneToFile(viewID, filename);

		basicVisualizationSystemContainer->SaveImageFinished(viewID);
	}
}

void GlfwRenderer::SaveSceneToFile(Index viewID, const STDstring& filename)
{
	RenderState* state = renderViews.State(viewID);

	GLFWwindow* window = renderViews.GetWindow(viewID);
	glfwMakeContextCurrent(window); //in order to read from correct openGL data

#ifdef GlfwRendererUsePNG
	bool pngAvailable = true;
#else
	bool pngAvailable = false;
#endif

	Index windowWidth = state->currentWindowSize[0]; //this is the size at which the renderer created buffer last time ...
	Index windowHeight = state->currentWindowSize[1];

	Index widthAlignment = visSettings->exportImages.widthAlignment; //width widthAlignment: 1,2,4 or 8
	Index heightAlignment = visSettings->exportImages.heightAlignment; //width widthAlignment: 1,2,4 or 8
	Index nrChannels = 3;
	Index stride = nrChannels * windowWidth;

	if (basicVisualizationSystemContainer->SaveImageAsData(viewID) || (visSettings->exportImages.saveImageFormat == "PNG" && pngAvailable) )
	{

		if (basicVisualizationSystemContainer->SaveImageAsData(viewID))
		{
			widthAlignment = 1;
			heightAlignment = 1;
		}
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
		widthAlignment = EXUstd::Clamp(widthAlignment, 1, 32); //exclude zero

		windowWidth = widthAlignment * (Index)(windowWidth / widthAlignment); //make multiple of 4 to align with most animation converter ...

		const Index strideAlignment = 1; //safer to use 1; otherwise, uncomment line below next! seems not to affect performance!
		stride = nrChannels * windowWidth; //must be div by strideAlignment!

		Index numberOfPixels = windowHeight * stride;
		ResizableArray<uint8_t> pixelBuffer(numberOfPixels); //until 1.10.11 this was char !
		pixelBuffer.SetNumberOfItems(numberOfPixels);

		glPixelStorei(GL_PACK_ALIGNMENT, strideAlignment);
		glReadBuffer(GL_FRONT);
		glReadPixels(0, 0, (GLsizei)windowWidth, (GLsizei)windowHeight, GL_RGB, GL_UNSIGNED_BYTE, pixelBuffer.GetDataPointer());

		//pixelBufferFlip writes to buffer in visualizationSystemContainer to be able to retrieve data
		ResizableArray<uint8_t>* pixelBufferFlip = basicVisualizationSystemContainer->ImageData(viewID);
		pixelBufferFlip->SetNumberOfItems(numberOfPixels);

		SlimVectorBase<Index, 2>* imageSize = basicVisualizationSystemContainer->GetImageDataSize(viewID);
		(*imageSize)[0] = windowWidth; //only needed for SaveImageAsData
		(*imageSize)[1] = windowHeight;

		//FLIP
		for (Index i = 0; i < windowHeight; i++)
		{
			for (Index j = 0; j < stride; j++)
			{
				(*pixelBufferFlip)[(windowHeight - i - 1) * stride + j] = pixelBuffer[i * stride + j];
			}
		}
		//==> image now in pixelBufferFlip / visualizationSystemContainer.imageData for returning
	}

	if (basicVisualizationSystemContainer->SaveImageAsData(viewID))
	{
		return;
	}

	if (visSettings->exportImages.saveImageFormat == "PNG" && pngAvailable)
	{
#ifdef GlfwRendererUsePNG
		ResizableArray<uint8_t>* pixelBufferFlip = basicVisualizationSystemContainer->ImageData(viewID);

		std::ofstream imageFile;
		CheckPathAndCreateDirectories(filename);

		windowHeight = heightAlignment * (Index)(windowHeight / heightAlignment);
		stbi_write_png(filename.c_str(), windowWidth, windowHeight, nrChannels, pixelBufferFlip->GetDataPointer(), stride);
		pixelBufferFlip->Flush(); //not stored to preserve earlier functionality
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
			CHECKandTHROWstring((STDstring("failed to open image file '") + filename + "'; check path and file name").c_str());
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
void GlfwRenderer::RenderGraphicsDataText(Index viewID, GraphicsData* data, Index lastItemID, 
	bool highlight, Index highlightID, Float4 highlightColor2, Float4 otherColor2, bool selectionMode)
{
	RenderState* state = renderViews.State(viewID);
	
	if (visSettings->openGL.advanced.lineSmooth) { glDisable(GL_LINE_SMOOTH); }

    float textheight = GetSettingsView(viewID, *visSettings).window.globalFontSize;
    //float scaleFactor = 2.f * state->zoom / EXUstd::Maximum(1.f, (float)state->currentWindowSize[1]); //factor, which gives approximately 1pt textsize

	Matrix4DF modelviewRotation = state->modelRotation;
	Matrix4DF modelviewRotationTp = modelviewRotation.GetTransposed();

	float textFontSize;
    //SetGLdepthMask(GL_FALSE);
    float offsetZ = 0.f; //positive values make it more visible than other objects!
	if (visSettings->general.textAlwaysInFront && GetSettingsView(viewID, *visSettings).camera.perspective == 0) //large offset does not work for perspective
	{
		offsetZ = visSettings->openGL.zMaxSceneFactor * 1.f; //relative to maxSceneSize; factor 1.f is used to stay far away from limit (for large scenes!)
	}
    else
    {
        offsetZ = visSettings->general.textOffsetFactor;
    }
    bool transparent = !visSettings->general.textHasBackground;

	glPushMatrix(); //store current matrix -> before rotation
	glMultMatrixf(modelviewRotationTp.GetDataPointer()); //rotate back -> rotation not applied to text! translation is done by OpenGL MODELVIEW

    for (const GLText& text : data->glTexts)
    {
        if (selectionMode) { if (text.itemID != lastItemID) { glLoadName(text.itemID); lastItemID = text.itemID; } }
        textFontSize = text.fontSize ? text.fontSize * GetFontScaling(viewID) : textheight * GetFontScaling(viewID);

		Float3 pos;
		TransformVertex(text.point, modelviewRotation, pos); //add rotation to position of text, but do not rotate text

        if (!highlight)
        {
            DrawString(viewID, text.text, textFontSize, pos, Float3({ text.offsetX, text.offsetY, offsetZ }), text.color, transparent);
        }
        else
        {
            Float4 color = otherColor2;
            if (text.itemID == highlightID) { color = highlightColor2; }

            DrawString(viewID, text.text, textFontSize, pos, Float3({ text.offsetX, text.offsetY, offsetZ }), color, transparent);
        }
    }
	glPopMatrix(); //restore matrix

}

//Index cnt0 = 0;

//draw sensor traces
//NOTE: this function could be moved to VisualizationSystem to create this data only once (but what happens for animations?)?
void GlfwRenderer::RenderSensorTraces(Index viewID)
{
	RenderState* state = renderViews.State(viewID);
	
	if (visSettings->sensors.traces.showPositionTrace ||
        visSettings->sensors.traces.showVectors || 
        visSettings->sensors.traces.showTriads)
    {
        float factOffset = 1.f*state->maxSceneSize;
        if (state->zoom != 0.f) { factOffset *= 1.f / state->zoom; }

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glPolygonOffset(visSettings->openGL.advanced.polygonOffset*factOffset, visSettings->openGL.advanced.polygonOffset*factOffset); //
        glEnable(GL_POLYGON_OFFSET_FILL);
        glDisable(GL_POLYGON_OFFSET_LINE);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        if (visSettings->openGL.advanced.lineSmooth) { glEnable(GL_LINE_SMOOTH); }

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
        if (visSettings->openGL.advanced.lineSmooth) { glDisable(GL_LINE_SMOOTH); }

    }
}




void GlfwRenderer::RenderGraphicsData(Index viewID, bool selectionMode)
{
	RenderState* state = renderViews.State(viewID);
	GLFWwindow* window = renderViews.GetWindow(viewID);
	const VSettingsView& settingsView = GetSettingsView(viewID, *visSettings);

	if (graphicsDataList)
	{
		bool useClipping = !(GetSettingsView(viewID, *visSettings).camera.clippingPlaneNormal == 0);
		if (useClipping && fabs(GetSettingsView(viewID, *visSettings).camera.clippingPlaneNormal.GetL2Norm() - 1.f) > 1e-5)
		{   //add some warning to show user that this normal vector does not work
			ShowMessage("Warning: clipping plane normal length != 1; normalizing clippingPlaneNormal", 5);
			//useClipping = false;
			GetSettingsView(viewID, *visSettings).camera.clippingPlaneNormal.NormalizeSafe();
		}
		//use this to draw coplanar lines in front polygons
		//this seems to be affected by zoom size: glPolygonOffset(visSettings->openGL.polygonOffset * state->maxSceneSize, visSettings->openGL.polygonOffset * state->maxSceneSize); //
		float factOffset = 1.f*state->maxSceneSize;
		if (state->zoom != 0.f) { factOffset *=  1.f / state->zoom; }

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glPolygonOffset(visSettings->openGL.advanced.polygonOffset*factOffset, visSettings->openGL.advanced.polygonOffset*factOffset); //
		glEnable(GL_POLYGON_OFFSET_FILL);
		glDisable(GL_POLYGON_OFFSET_LINE);
		//glDisable(GL_POLYGON_OFFSET_FILL);
		//glEnable(GL_POLYGON_OFFSET_LINE);

		Index lastItemID = itemIDinvalidValue;
		if (selectionMode)
		{
			glLoadName(EXUstd::InvalidIndex); //to have some name in it
		}
		//check if item shall be highlighted:
		bool highlight = false;
		Float4 highlightColor = visSettings->interactive.advanced.highlightColor;
		Float4 otherColor = visSettings->interactive.advanced.highlightOtherColor;
		Float4 highlightColor2 = visSettings->interactive.advanced.highlightColor; //for text and lines
		Float4 otherColor2 = visSettings->interactive.advanced.highlightOtherColor; //for text and lines

		//Index highlightIndex;
		//ItemType highlightType;
		//Index highlightMbsNumber;
		Index highlightIndex = visSettings->interactive.highlightItemIndex;
		ItemType highlightType = visSettings->interactive.highlightItemType;
		Index highlightMbsNumber = visSettings->interactive.highlightMbsNumber;
		if (visSettings->interactive.advanced.selectionHighlights && stateMachine.highlightIndex != EXUstd::InvalidIndex)
		{
			//if selected with mouse, temporarily use this:
			if (stateMachine.highlightTimeout != 0. && stateMachine.highlightTimeout < EXUstd::GetTimeInSeconds())
			{
				stateMachine.highlightIndex = EXUstd::InvalidIndex; //from now on, no further highlighting
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
            if (settingsView.scene.facesTransparent)
            {
                RenderGraphicsDataText(viewID, data, lastItemID, highlight, highlightID, highlightColor2, otherColor2, selectionMode);
            }
            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//DRAW POINTS before triangles (nodes shown in transparent scenes)

			glLineWidth(visSettings->openGL.lineWidth);
			if (visSettings->openGL.advanced.lineSmooth) { glEnable(GL_LINE_SMOOTH); }

			if (settingsView.scene.showFaces)
			{
				if (visSettings->openGL.advanced.enableLighting) { glEnable(GL_LIGHTING); } //only enabled when drawing triangle faces
				glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			}

			for (const GLSphere& item : data->glSpheres)
			{
				if (selectionMode) { if (item.itemID != lastItemID) { glLoadName(item.itemID); lastItemID = item.itemID; } }

				if (useClipping) { if (IsClipped(viewID, item.point)) { continue; } }

				DrawSphere(item, highlight, highlightID, otherColor2, highlightColor2, settingsView.scene.showFaces);
			}
			if (settingsView.scene.showFaces) //now turn off lighting for lines and texts
			{
				if (visSettings->openGL.advanced.enableLighting) { glDisable(GL_LIGHTING); } //only enabled when drawing triangle faces
			}

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//DRAW TRIANGLES
			if (settingsView.scene.showFaces || settingsView.scene.showMeshFaces)
			{
				if (visSettings->openGL.advanced.enableLighting) { glEnable(GL_LIGHTING); }
				glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

				if (highlight) {
					DrawTriangles<true, false, false>(data->glTriangles,
						highlightColor, otherColor, selectionMode, lastItemID, highlightID, useClipping, window);
				}
				else if (!settingsView.scene.facesTransparent) 
				{
					DrawTriangles<false, false, false>(data->glTriangles,
						highlightColor, otherColor, selectionMode, lastItemID, highlightID, useClipping, window);
					if (!selectionMode) // DrawTrianglesWithShadow will do nothing if lights have no shadow; //&& visSettings->openGL.shadow != 0)
					{
						DrawTrianglesWithShadow(viewID, data);
					}
				}
				else {
					DrawTriangles<false, true, false>(data->glTriangles,
						highlightColor, otherColor, selectionMode, lastItemID, highlightID, useClipping, window);
				}

				if (visSettings->openGL.advanced.enableLighting) { glDisable(GL_LIGHTING); }
			}


			//++++++++++++++++++++++++++++++++++++++++++++++
			//draw lines at end of rendering: lines shown nicely on top of faces with polygon offset
			//++++++++++++++++++++++++++++++++++++++++++++++

			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			//DRAW CIRCLES
			//draw a circle in xy-plane
			for (const GLCircleXY& item : data->glCirclesXY)
			{
				if (useClipping)
				{
					if (IsClipped(viewID, item.point)) { continue; }
				}
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
				float len = visSettings->openGL.drawNormalsLength;
				Float4 edgeColor = visSettings->openGL.advanced.faceNormalsColor;
				glColor4f(edgeColor[0] + 0.5f, edgeColor[1], edgeColor[2], edgeColor[3]);
				for (const GLTriangle& trig : data->glTriangles)
				{
					if (useClipping)
					{
						if (IsClipped(viewID, trig.points[0]) && IsClipped(viewID, trig.points[1]) && IsClipped(viewID, trig.points[2])) { continue; }
					}
					if (selectionMode) { if (trig.itemID != lastItemID) { glLoadName(trig.itemID); lastItemID = trig.itemID; } }
					Float3 midPoint = { 0,0,0 };
					for (Index i = 0; i < 3; i++)
					{
						midPoint += trig.points[i];
					}
					midPoint *= 1.f / 3.f;
					glBegin(GL_LINES);
					const Float3& p = midPoint;
					glVertex3f(p[0], p[1], p[2]);
					Float3 p1 = midPoint + len * EGeometry::ComputeTriangleNormalTemplate<float, std::array<Float3, 3>>(trig.points); //OLD: trig.normals[0];
					glVertex3f(p1[0], p1[1], p1[2]);
					glEnd();
				}
			}

			if (visSettings->openGL.drawVertexNormals)
			{
				float len = visSettings->openGL.drawNormalsLength;
				Float4 edgeColor = visSettings->openGL.advanced.vertexNormalsColor;
				glColor4f(edgeColor[0], edgeColor[1], edgeColor[2]+0.5f, edgeColor[3]);
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
			if (settingsView.scene.showLines)
			{
				if (!highlight)
				{
					for (const GLLine& item : data->glLines)
					{
						if (useClipping)
						{
							if (IsClipped(viewID, item.point1) && IsClipped(viewID, item.point2)) { continue; }
						}
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
						if (useClipping)
						{
							if (IsClipped(viewID, item.point1) || IsClipped(viewID, item.point2)) { continue; }
						}
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
			if (settingsView.scene.showFaceEdges || settingsView.scene.showMeshEdges)
			{
				if (highlight)
				{
					DrawTriangles<true, false, true>(data->glTriangles,
						highlightColor, otherColor, selectionMode, lastItemID, highlightID, useClipping, window);
				}
				else
				{
					DrawTriangles<false, false, true>(data->glTriangles,
						highlightColor, otherColor, selectionMode, lastItemID, highlightID, useClipping, window);
				}
			}

            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            //DRAW TEXT finally to be in front of everything if no transparency is activated
            if (!settingsView.scene.facesTransparent)
            {
                if (visSettings->general.textAlwaysInFront)
                {
                    glDepthMask(GL_FALSE); //draw system information and coordinate system always in front
                }

                
                RenderGraphicsDataText(viewID, data, lastItemID, highlight, highlightID, highlightColor2, otherColor2, selectionMode);
                if (GetGLdepthMask())
                {
                    glDepthMask(GL_TRUE); //switch back to original state
                }
            }
            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


		} //for (auto data : *graphicsDataList)
		if (selectionMode)
		{
			glLoadName(EXUstd::InvalidIndex); //to have some name in
		}



	} //if graphicsDataList
}



void GlfwRenderer::DrawSphere(const GLSphere& item, bool highlight, Index highlightID, const Float4& otherColor2, const Float4& highlightColor2, bool showFaces)
{
	if (!showFaces || item.resolution < 1 || item.radius <= 0.f)
	{
		GLfloat d = visSettings->general.pointSize; //point drawing parameter --> put into settings!
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






//following concepts of https://github.com/joshb/shadowvolumes
//draw stenciled shadow volume
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


void GlfwRenderer::DrawTrianglesWithShadow(Index viewID, GraphicsData* data)
{
	RenderState* state = renderViews.State(viewID);
	const VSettingsView& settingsView = GetSettingsView(viewID, *visSettings);

	float maxDist = state->maxSceneSize * 1.5f;

	float factOffset = 1.f * state->maxSceneSize;
	if (state->zoom != 0.f) { factOffset *= 1.f / state->zoom; }

	bool hasShadow = false;
	// =========== LOOP OVER LIGHTS ===========
	for (Index lightID = 0; lightID < MAX_LIGHTS_GLFW; lightID++)
	{
		const VSettingsLight& settingsLight = GetSettingsLight(lightID, *visSettings);

		float shadow = EXUstd::Clamp(settingsLight.shadow, 0.f, 1.f);
		if (!settingsLight.enable || shadow == 0) { continue; }
		hasShadow = true;

		glClear(GL_STENCIL_BUFFER_BIT);

		//render triangle shadow using stencils
		const Float4& lp = settingsLight.position;
		Float3 lightPos({ lp[0], lp[1], lp[2] });
		bool useLightDir = lp[3] == 0.f;
		if (useLightDir)
		{
			float lightPosNorm = lightPos.GetL2Norm();
			if (lightPosNorm > 0.f)
			{
				lightPos *= 200.f * state->maxSceneSize / lightPosNorm; //scale light position to 200*maxScene to get position-independent effects
			}
		}
		if (settingsLight.useCameraFrame)
		{
			//because lights are added in modelView-coordinates, 
			// camera-fixed lights have to be transformed!
			Matrix3DF rotationMV;
			Float3 translationMV;
			state->GetRotationTranslationFWithMarker(rotationMV, translationMV,
				basicVisualizationSystemContainer, settingsView, false);
			lightPos = lightPos * rotationMV + (1.f-(float)useLightDir) * (translationMV * rotationMV); //light is rotated/translated back to camera view
			//lightPos = rotationMV * lightPos - useLightDir * (rotationMV*translationMV); //light is rotated/translated back to camera view
		}

		//create shadow volumes:
		glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
		SetGLdepthMask(GL_FALSE);

		glEnable(GL_CULL_FACE);
		glEnable(GL_STENCIL_TEST);
		glEnable(GL_POLYGON_OFFSET_FILL);

		//here, we must use the original offset and add a shadow offset ...
		glPolygonOffset(visSettings->openGL.advanced.polygonOffset * factOffset,
			visSettings->openGL.advanced.polygonOffset * factOffset + 
			visSettings->openGL.advanced.shadowPolygonOffset * state->maxSceneSize);

		glCullFace(GL_FRONT);
		glStencilFunc(GL_ALWAYS, 0x0, 0xff);
		glStencilOp(GL_KEEP, GL_INCR_WRAP, GL_KEEP); //INCR_WRAP/DECR_WRAP works for > 255 triangles
		
		for (const GLTriangle& trig : data->glTriangles)
		{ //draw faces
			if ((settingsView.scene.showFaces && !trig.isFiniteElement)
				|| (settingsView.scene.showMeshFaces && trig.isFiniteElement))
			{
				RenderTriangleShadowVolume(trig, lightPos, maxDist, shadow);
			}
		}
		glCullFace(GL_BACK);
		glStencilFunc(GL_ALWAYS, 0x0, 0xff);
		glStencilOp(GL_KEEP, GL_DECR_WRAP, GL_KEEP);
		for (const GLTriangle& trig : data->glTriangles)
		{ //draw faces
			if ((settingsView.scene.showFaces && !trig.isFiniteElement)
				|| (settingsView.scene.showMeshFaces && trig.isFiniteElement))
			{
				RenderTriangleShadowVolume(trig, lightPos, maxDist, shadow);
			}
		}

		glDisable(GL_POLYGON_OFFSET_FILL);
		glDisable(GL_CULL_FACE);
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

		glStencilFunc(GL_NOTEQUAL, 0x0, 0xff);
		glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);

		DrawShadowPlane(shadow);

		glDisable(GL_STENCIL_TEST);
		// Enable blending, therefore shadows from multiple lights accumulate (darken)
		//glEnable(GL_BLEND); //blend in order to overlay effects of multiple shadows
		//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		//DrawShadowPlane(shadow);

		//glDisable(GL_STENCIL_TEST);
		//glDisable(GL_BLEND); //needed for texts
	}

	if (hasShadow)
	{
		//set back values ...
		SetGLdepthMask(GL_TRUE);
		glPolygonOffset(visSettings->openGL.advanced.polygonOffset * factOffset,
			visSettings->openGL.advanced.polygonOffset * factOffset); //
		glEnable(GL_POLYGON_OFFSET_FILL);

		glEnable(GL_BLEND); //needed fo texts
	}
}

#endif //USE_GLFW_GRAPHICS




