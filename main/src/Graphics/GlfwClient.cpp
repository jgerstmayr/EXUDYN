/** ***********************************************************************************************
* @brief        Implementation of GlfwClient
*
* @author       Gerstmayr Johannes
* @date         2019-05-24 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: missing
                
************************************************************************************************ */
#pragma once

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#ifdef USE_GLFW_GRAPHICS

#include <ostream>
#include <stdlib.h>
#include <stdio.h>
#include <fstream> //for image save to file

#include <chrono> //sleep_for()
//#pragma comment(lib, "opengl32")
//#pragma comment(lib, "glu32")
//#include <gl/gl.h>
//#include <gl/glu.h>

//#define GLFW_INCLUDE_ES3 //open gl ES version
#include <GLFW/glfw3.h>


#include "Graphics/GlfwClient.h"

extern bool globalPyRuntimeErrorFlag; //stored in Stdoutput.cpp; this flag is set true as soon as a PyError or SysError is raised; this causes to shut down secondary processes, such as graphics, etc.
#define rendererOut std::cout //defines the type of output for renderer: pout could be problematic because of parallel threads; std::cout does not work in Spyder


GlfwRenderer glfwRenderer;

//++++++++++++++++++++++++++++++++++++++++++
//define static variables:
bool GlfwRenderer::rendererActive = false;
bool GlfwRenderer::stopRenderer = false;
Index GlfwRenderer::rendererError = 0;
GLFWwindow* GlfwRenderer::window = nullptr;
RendererState* GlfwRenderer::state;
RendererStateMachine GlfwRenderer::stateMachine;
std::thread GlfwRenderer::rendererThread;
//uint64_t GlfwRenderer::visualizationCounter = 0;

ResizableArray<GraphicsData*>* GlfwRenderer::graphicsDataList = nullptr;
//GraphicsData* GlfwRenderer::data = nullptr;
VisualizationSettings* GlfwRenderer::visSettings = nullptr;
VisualizationSystemContainerBase* GlfwRenderer::basicVisualizationSystemContainer = nullptr;
//++++++++++++++++++++++++++++++++++++++++++


GlfwRenderer::GlfwRenderer()
{
	rendererActive = false;
	
	stateMachine.leftMousePressed = false;
	stateMachine.rightMousePressed = false;
	stateMachine.shiftPressed = false;
	stateMachine.ctrlPressed = false;
	stateMachine.mode = RendererMode::None;			//!< determines the state of any action

	stateMachine.mousePositionX = 0;	//!< last mouse position used for move and zoom
	stateMachine.mousePositionY = 0;	//!< last mouse position used for move and zoom
	stateMachine.lastMousePressedX = 0;	//!< last left mouse button position pressed
	stateMachine.lastMousePressedY = 0;

};

void GlfwRenderer::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
	{
		basicVisualizationSystemContainer->StopSimulation();
		std::this_thread::sleep_for(std::chrono::milliseconds(200)); //give thread time to finish the stop simulation command

		glfwSetWindowShouldClose(window, GL_TRUE);
	}
	
	//keycode to quit simulation:
	if (key == GLFW_KEY_Q && action == GLFW_PRESS && mods == 0)
	{

		basicVisualizationSystemContainer->StopSimulation();
	}

	//keycode to continue paused simulation:
	if ((key == GLFW_KEY_SPACE && action == GLFW_PRESS && mods == 0) || (key == GLFW_KEY_SPACE && mods == GLFW_MOD_SHIFT))
	{
		basicVisualizationSystemContainer->ContinueSimulation();
	}

	if (key == GLFW_KEY_1 && action == GLFW_PRESS)
	{
		visSettings->general.graphicsUpdateInterval = 0.02f;
		rendererOut << "Visualization update: 20ms\n";
	}

	if (key == GLFW_KEY_2 && action == GLFW_PRESS)
	{
		visSettings->general.graphicsUpdateInterval = 0.2f;
		rendererOut << "Visualization update: 200ms\n";
	}

	if (key == GLFW_KEY_3 && action == GLFW_PRESS)
	{
		visSettings->general.graphicsUpdateInterval = 1.f;
		rendererOut << "Visualization update: 1s\n";
	}

	if (key == GLFW_KEY_4 && action == GLFW_PRESS)
	{
		visSettings->general.graphicsUpdateInterval = 10.f;
		rendererOut << "Visualization update: 10s\n";
	}

	if (key == GLFW_KEY_5 && action == GLFW_PRESS)
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
	if (key == GLFW_KEY_X && action == GLFW_PRESS)
	{
		//open window to execute a python command ... (THREADSAFE???)
		std::string str = R"(
import tkinter as tk
from tkinter.scrolledtext import ScrolledText

singleCommandMainwin = tk.Tk()
def OnSingleCommandReturn(event):
    exec(singleCommandEntry.get(), globals())
    singleCommandMainwin.destroy()

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
		std::string str = "import exudynGUI\nvis=SC.visualizationSettings.GetDictionaryWithTypeInfo()\nSC.visualizationSettings.SetDictionary(exudynGUI.EditDictionaryWithTypeInfo(vis, exu, 'Visualization Settings'))";
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
textW = tk.Text(root, height = 30, width = 60)
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
0 or KEYPAD '0'       ... reset rotation
CURSOR UP, DOWN, etc. ... move scene
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
Q      ... stop simulation
X      ... execute command
V      ... visualization settings
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
	float transStep = visSettings->window.keypressRotationStep * state->zoom; //degrees
	float zoomStep = visSettings->window.zoomStepFactor;
	Float3 incRot({ 0.f,0.f,0.f });
	if (key == GLFW_KEY_KP_2 && action == GLFW_PRESS) { incRot[0] = rotStep; }
	if (key == GLFW_KEY_KP_8 && action == GLFW_PRESS) { incRot[0] = -rotStep; }
	if (key == GLFW_KEY_KP_4 && action == GLFW_PRESS) { incRot[1] = rotStep; }
	if (key == GLFW_KEY_KP_6 && action == GLFW_PRESS) { incRot[1] = -rotStep; }
	if (key == GLFW_KEY_KP_7 && action == GLFW_PRESS) { incRot[2] = rotStep; }
	if (key == GLFW_KEY_KP_9 && action == GLFW_PRESS) { incRot[2] = -rotStep; }

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

	if ((key == GLFW_KEY_KP_0 || key == GLFW_KEY_0) && action == GLFW_PRESS) //reset all rotations
	{
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();	//start with identity
		glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering
		rendererOut << "Reset OpenGL modelview\n";
	}



	if (key == GLFW_KEY_UP && action == GLFW_PRESS) { state->centerPoint[1] -= transStep; }
	if (key == GLFW_KEY_DOWN && action == GLFW_PRESS) { state->centerPoint[1] += transStep; }
	if (key == GLFW_KEY_LEFT && action == GLFW_PRESS) { state->centerPoint[0] += transStep; }
	if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS) { state->centerPoint[0] -= transStep; }
	
	if ((key == GLFW_KEY_KP_SUBTRACT || key == GLFW_KEY_COMMA) && action == GLFW_PRESS) 
	{ 
		if (mods == GLFW_MOD_CONTROL)
		{ state->zoom *= pow(zoomStep,0.1f); } //small zoom step
		else { state->zoom *= zoomStep; }
	}
	if ((key == GLFW_KEY_KP_ADD || key == GLFW_KEY_PERIOD) && action == GLFW_PRESS) 
	{ 
		if (mods == GLFW_MOD_CONTROL)
		{
			state->zoom /= pow(zoomStep, 0.1f);
		} //small zoom step
		else { state->zoom /= zoomStep;; }
	}

	if (key == GLFW_KEY_A && action == GLFW_PRESS) { ZoomAll(); UpdateGraphicsDataNow(); }

}

void GlfwRenderer::ZoomAll()
{
	//max scene size from current line data:
	//Float3 pmax({ -1e30f,-1e30f,-1e30f });
	//Float3 pmin({ 1e30f,1e30f,1e30f });

	Float3 pmax({ -1e30f,-1e30f,-1e30f });
	Float3 pmin({ 1e30f,1e30f,1e30f });

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
			for (auto point: item.points)
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
			&& (*graphicsDataList)[0]->glPoints.NumberOfItems() == 0 && (*graphicsDataList)[0]->glTexts.NumberOfItems() == 0)
			&& (*graphicsDataList)[0]->glTriangles.NumberOfItems() == 0)
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
		//if (stateMachine.mode != RendererMode::None)
		//{
		stateMachine.lastMousePressedX = stateMachine.mousePositionX;
		stateMachine.lastMousePressedY = stateMachine.mousePositionY; //now see if the mouse moves, then switch to move mode!
		//}
	}
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
	{
		stateMachine.leftMousePressed = false;
		//rendererOut << "mouse button left released\n";
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//STATE MACHINE ROTATE
	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS && !stateMachine.leftMousePressed)
	{
		//rendererOut << "mouse button left pressed\n";
		stateMachine.rightMousePressed = true;
		stateMachine.lastMousePressedX = stateMachine.mousePositionX;
		stateMachine.lastMousePressedY = stateMachine.mousePositionY; //now see if the mouse moves, then switch to move mode!
	}
	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE)
	{
		stateMachine.rightMousePressed = false;
		//rendererOut << "mouse button left released\n";
	}

	//glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED); //unlimited cursor position (also outside of window) - might get negative coordinates
	//glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

}

void GlfwRenderer::cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
	//rendererOut << "mouse cursor: x=" << xpos << ", y=" << ypos << "\n";
	stateMachine.mousePositionX = xpos;
	stateMachine.mousePositionY = ypos;

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//MOUSE MOVE state machine:
	//check if one should switch to mouse move mode:
	double minMove = 2;
	if (stateMachine.leftMousePressed && stateMachine.mode == RendererMode::None)
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
			float height = (float)state->currentWindowSize[1];
			float factor = 2.f*state->zoom / height;
			state->centerPoint[0] = stateMachine.storedCenterPointX - (float)(xpos - stateMachine.lastMousePressedX) * factor;
			state->centerPoint[1] = stateMachine.storedCenterPointY + (float)(ypos - stateMachine.lastMousePressedY) * factor;
		}
		else { stateMachine.mode = RendererMode::None; } //finish move operation if button is released!
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//ROTATE state machine:
	//check if one should switch to mouse move mode:
	minMove = 2; //for rotation
	if (stateMachine.rightMousePressed && stateMachine.mode == RendererMode::None)
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
			//glLoadIdentity();	//start with identity
			glLoadMatrixf(stateMachine.storedModelRotation.GetDataPointer()); //load previous rotation
			glRotatef(deltaX*rotationFactor, 0.f, 1.f, 0.f); //apply "incremental" rotation around x
			glRotatef(deltaY*rotationFactor, 1.f, 0.f, 0.f); //apply "incremental" rotation around y
			glGetFloatv(GL_MODELVIEW_MATRIX, state->modelRotation.GetDataPointer()); //store rotation in modelRotation, applied in model rendering

		}
		else { stateMachine.mode = RendererMode::None; } //finish move operation if button is released!
	}


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//ZOOM SELECT state machine:

}


bool GlfwRenderer::SetupRenderer()
{
	//glfwCreateThread();
	//auto th = new std::thread(GlfwRenderer::StartThread);
	globalPyRuntimeErrorFlag = false; //if previous renderer crashed, this allows to relase this error even if the old renderer is still running
	if (!rendererActive)
	{
		basicVisualizationSystemContainer->UpdateMaximumSceneCoordinates(); //this is done to make OpenGL zoom and maxSceneCoordinates work

		rendererError = 0; 

		pout << "Setup OpenGL renderer ...\n";
		if (rendererThread.joinable()) //thread is still running from previous call ...
		{
			rendererThread.join();
			//rendererThread.~thread(); //this would make the thread unusable?
		}
		rendererThread = std::thread(GlfwRenderer::InitCreateWindow);
		Index timeOut = visSettings->window.startupTimeout / 10;

		Index i = 0;
		while(i++ < timeOut && !rendererActive || rendererError > 0) //wait 5 seconds for thread to answer; usually 150ms in Release and 500ms in debug mode
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		pout << "waited for " << i * 10 << " milliseconds \n";
		if (rendererActive) { pout << "OpenGL renderer started!\n"; return true; }
		else { 
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
		SysError("OpenGL renderer already active");
		return false;
	}

}

//! stop the renderer engine and its thread; @todo StopRenderer currently also stops also main thread (python)
void GlfwRenderer::StopRenderer()
{
	if (window)
	{
		stopRenderer = true;
		glfwSetWindowShouldClose(window, 1);
		Index timeOut = visSettings->window.startupTimeout / 10;

		Index i = 0;
		while (i++ < timeOut && rendererActive) //wait 5 seconds for thread to answer
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		if (rendererActive) { SysError("OpenGL Renderer could not be stopped safely\n"); }
		//else { pout << "Renderer Stopped\n"; }

		glfwDestroyWindow(window);
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
	if (sizex < 20) { sizex = 20; }//limit lower size: negative numbers or zero could make problems ...
	if (sizey < 20) { sizey = 20; }

	if (sizex > 8160) { sizex = 8160; } //limit upper size for now ...
	if (sizey > 4320) { sizey = 4320; }

	window = glfwCreateWindow(sizex, sizey, "Exudyn OpenGL window", NULL, NULL);

	if (!window)
	{
		rendererError = 2;
		glfwTerminate();
		SysError("GLFWRenderer::InitCreateWindow: Render window could not be created");
		exit(EXIT_FAILURE);
	}

	rendererActive = true; //this is still threadsafe, because main thread waits for this signal!

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
	//+++++++++++++++++

	//+++++++++++++++++++++++++++++++++
	//depending on flags, do some changes to window
	
	if (visSettings->window.showWindow)
	{
		glfwShowWindow(window); //show the window when created ... should by anyway done, but did not work in Spyder so far
		//glfwFocusWindow(window); //show window and give focus to window; DANGEROUS, if user does not want this; will by anyway done automatically
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
		if (basicVisualizationSystemContainer->GetAndResetZoomAllRequest()) { ZoomAll(); ZoomAll();}
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
	glfwTerminate(); //move to destructor
	rendererActive = false; //for new startup of renderer
	stopRenderer = false;	//if stopped by user

}

void GlfwRenderer::SetGLLights()
{
	if (visSettings->openGL.shadeModelSmooth) { glShadeModel(GL_SMOOTH); }
	else { glShadeModel(GL_FLAT); }

	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	Float4 bg = visSettings->general.backgroundColor;
	glClearColor(bg[0], bg[1], bg[2], bg[3]); //(float red, float green, float blue, float alpha);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// the text should not be affected by the lighting effects
	glDisable(GL_LIGHTING);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();


	//+++++++++++++++++++++++++++++++++++
	//light; see https://www.glprogramming.com/red/chapter05.html#name4
	;
	//GLfloat mat_specular[] = { 0.7f, 0.7f, 0.7f, 1.0f };
	//GLfloat mat_shininess = 10.0f;
	//GLfloat light_position[] = { 2.0f, 3.0f, 4.0f, 0.0f };
	//GLfloat light_position2[] = { 100.0f, 100.0f, 100.0f, 0.0f };
	//GLfloat light0_ambient[] = { 0.3f, 0.3f, 0.3f};
	//GLfloat light0_diffuse[] = { 0.3f, 0.3f, 0.3f };
	//GLfloat light0_specular[] = { 0.3f, 0.3f, 0.3f };

	glLightf(GL_LIGHT0, GL_AMBIENT, visSettings->openGL.light0ambient);
	glLightf(GL_LIGHT0, GL_DIFFUSE, visSettings->openGL.light0diffuse);
	glLightf(GL_LIGHT0, GL_SPECULAR, visSettings->openGL.light0specular);
	glLightfv(GL_LIGHT0, GL_POSITION, visSettings->openGL.light0position.GetDataPointer());
	if (visSettings->openGL.enableLight0) { glEnable(GL_LIGHT0); }
	else { glDisable(GL_LIGHT0); }

	glLightf(GL_LIGHT1, GL_AMBIENT, visSettings->openGL.light1ambient);
	glLightf(GL_LIGHT1, GL_DIFFUSE, visSettings->openGL.light1diffuse);
	glLightf(GL_LIGHT1, GL_SPECULAR, visSettings->openGL.light1specular);
	glLightfv(GL_LIGHT1, GL_POSITION, visSettings->openGL.light1position.GetDataPointer());
	if (visSettings->openGL.enableLight0) { glEnable(GL_LIGHT1); }
	else { glDisable(GL_LIGHT1); }

	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, visSettings->openGL.materialShininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, visSettings->openGL.materialSpecular.GetDataPointer());
	glEnable(GL_COLOR_MATERIAL);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 0);
	GLint localViewer = 0;
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, localViewer);

	//glCullFace(GL_FRONT); //only view one side of faces
	//glEnable(GL_CULL_FACE);
	glDisable(GL_CULL_FACE);

	//+++++++++++++++++++++++++++++++++++
	glPopMatrix();

	glEnable(GL_LIGHTING);
}


Index firstRun = 0; //zoom all in first run
void GlfwRenderer::Render(GLFWwindow* window) //GLFWwindow* needed in argument, because of glfwSetWindowRefreshCallback
{
	//rendererOut << "Render\n";
	float ratio;
	int width, height;

	glfwGetFramebufferSize(window, &width, &height);

	//rendererOut << "current window: width=" << width << ", height=" << height << "\n";
	state->currentWindowSize[0] = width;
	state->currentWindowSize[1] = height;

	ratio = width / (float)height;

	GLfloat zoom = state->zoom;

	glViewport(0, 0, width, height);
	//glEnable(0x809D);// GL_MULTISAMPLE is not defined, but 0x809D does the job; settings?
	SetGLLights(); //must be very early, before anything to draw

	//get available line width range:
	//GLfloat LineRange[2];
	//glGetFloatv(GL_LINE_WIDTH_RANGE, LineRange);
	//rendererOut << "Minimum Line Width " << LineRange[0] << " – ";		//gives 0.5 run with VS2017
	//rendererOut << "Maximum Line Width " << LineRange[1] << std::endl;	//gives 10  run with VS2017

	glDisable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	GLdouble zFactor = 100.; //original:100
	glOrtho(-ratio * zoom, ratio*zoom, -zoom, zoom, -zFactor*2.*state->maxSceneSize, zFactor * 2.*state->maxSceneSize); //https: //www.khronos.org/opengl/wiki/Viewing_and_Transformations#How_do_I_implement_a_zoom_operation.3F
	//original (flipped?): 
	//glOrtho(-ratio * zoom, ratio*zoom, -1.f*zoom, 1.f*zoom, zFactor*2.*state->maxSceneSize, -zFactor * 2.*state->maxSceneSize); //https: //www.khronos.org/opengl/wiki/Viewing_and_Transformations#How_do_I_implement_a_zoom_operation.3F
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glTranslated(-state->centerPoint[0], -state->centerPoint[1], 0.f); 

	glMultMatrixf(state->modelRotation.GetDataPointer());
	//glRotatef(state->rotations[2], 0.f, 0.f, 1.f);//((float)glfwGetTime() * 50.f, 0.f, 0.f, 1.f);
	//glRotatef(state->rotations[1], 0.f, 1.f, 0.f);
	//glRotatef(state->rotations[0], 1.f, 0.f, 0.f);

	RenderGraphicsData();
	//glPushMatrix(); //store current matrix
	//glPopMatrix(); //restore matrix
	Float4 textColor({ 0.2f,0.2f,0.2f,1.f }); //chosen text color (add to settings?)

	if (visSettings->general.showComputationInfo) //draw coordinate system
	{
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		float factor = 0.35f*zoom * 2.6f;
		glTranslated(-factor*ratio*1.05, factor, 0.f);

		float scale = 2.f*visSettings->general.textSize*zoom / ((float)height);
		Float3 poff({ 0.f,0.05f*zoom,0.f }); //offset

		DrawString(basicVisualizationSystemContainer->GetComputationMessage().c_str(), scale, poff, textColor);
	}

	if (visSettings->contour.showColorBar && visSettings->contour.outputVariable != OutputVariableType::None) //draw coordinate system
	{
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		float factor = 0.35f*zoom * 2.6f;
		glTranslated(-factor * ratio*1.05, factor, 0.f);

		float d = 0.05f*zoom;

		float scale = 2.f*visSettings->general.textSize*zoom / ((float)height);
		Float3 p0({ 0.f,-2*d,0.f }); //offset

		if (graphicsDataList->NumberOfItems() > 1) { pout << "WARNING: contour plot color bar only works for one single system\n"; }

		float minVal = graphicsDataList->GetItem(0)->GetContourCurrentMinValue();
		float maxVal = graphicsDataList->GetItem(0)->GetContourCurrentMaxValue();

		//DrawString(basicVisualizationSystemContainer->GetComputationMessage().c_str(), scale, poff, textColor);
		STDstring contourStr = STDstring(GetOutputVariableTypeString(visSettings->contour.outputVariable)) + "(" + EXUstd::ToString(visSettings->contour.outputVariableComponent) + ")" +
			"\nmin=" + EXUstd::ToString(minVal) + ",max=" + EXUstd::ToString(maxVal);
		DrawString(contourStr.c_str(), scale, p0, textColor);
		p0 += Float3({0.f,-2*d,0.f});


		//now draw boxes for contour plot colors and add texts
		float n = (float)visSettings->contour.colorBarTiling;
		float range = maxVal - minVal;
		for (float i = 0; i < n; i++)
		{
			float value = i / n * range + minVal;
			const float sizeX = 0.06f*zoom;
			const float sizeY = 0.05f*zoom * n/12.f;

			if (visSettings->openGL.lineSmooth) { glEnable(GL_LINE_SMOOTH); }
			glLineWidth(visSettings->openGL.lineWidth);

			glBegin(GL_LINE_STRIP);
			Float4 color0 = VisualizationSystemContainerBase::ColorBarColor(minVal, maxVal, value);
			glColor3f(color0[0], color0[1], color0[2]);
			glVertex3f(p0[0], p0[1], 0.);
			glVertex3f(p0[0] + sizeX, p0[1], 0.);
			glVertex3f(p0[0] + sizeX, p0[1] - sizeY, 0.);
			glVertex3f(p0[0], p0[1] - sizeY, 0.);
			glVertex3f(p0[0], p0[1], 0.);
			glEnd();
			if (visSettings->openGL.lineSmooth) { glDisable(GL_LINE_SMOOTH); }

			char str[20];
			std::sprintf(str, "% .2g", value);
			DrawString(str, scale*0.8f, p0 + Float3({ 1.2f*sizeX,-0.8f*sizeY,0. }), textColor);
			//DrawString(EXUstd::ToString(value).c_str(), scale*0.8, p0 + Float3({ 1.2f*sizeX,-0.8f*sizeY,0. }), textColor);

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
	}

	if (visSettings->general.drawCoordinateSystem) //draw coordinate system
	{
		//float d = zoom * 0.1;
		float d = visSettings->general.coordinateSystemSize*zoom;
		//float a = d * 0.05f; //distance of text

		Float3 p0({ 0.f,0.f,0.f });
		Float3 v1({ d,  0.f,0.f });
		Float3 v2({ 0.f,  d,0.f });
		Float3 v3({ 0.f,0.f,  d }); //check why z-coordinates are flipped ...
		Float3 p1 = p0 + v1;
		Float3 p2 = p0 + v2;
		Float3 p3 = p0 + v3;

		//rendererOut << "zoom=" << zoom << "\n";

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		float factor = -0.35f*zoom * 2.5f;
		//glOrtho(0.0f, ratio, 1, 0.0f, 0.0f, 1.0f);
		//float factor = -0.35f;

		glTranslated(factor*ratio, factor, 0.f);
		glMultMatrixf(state->modelRotation.GetDataPointer());

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

		const char* X1 = "X1";
		const char* X2 = "X2";
		const char* X3 = "X3";

		float scale = 2.f*visSettings->general.textSize*zoom/((float)height);

		Float16 m = state->modelRotation;
		Float16 matTp({m[0],m[4],m[ 8],m[12],
					   m[1],m[5],m[ 9],m[13],
					   m[2],m[6],m[10],m[14],
					   m[3],m[7],m[11],m[15]});
		Float3 poff({ 0.005f,0.005f,0.f }); //small offset from axes

		glPushMatrix(); //store current matrix -> before rotation
		glTranslated(p1[0], p1[1], p1[2]);
		glMultMatrixf(matTp.GetDataPointer());
		glLineWidth(0.25f);
		DrawString(X1, scale, poff, textColor);
		glPopMatrix(); //restore matrix

		glPushMatrix(); //store current matrix -> before rotation
		glTranslated(p2[0], p2[1], p2[2]);
		glMultMatrixf(matTp.GetDataPointer());
		glLineWidth(0.5f);
		DrawString(X2, scale, poff, textColor);
		glPopMatrix(); //restore matrix

		glPushMatrix(); //store current matrix -> before rotation
		glTranslated(p3[0], p3[1], p3[2]);
		glMultMatrixf(matTp.GetDataPointer());
		glLineWidth(1.f);
		DrawString(X3, scale, poff, textColor);
		glPopMatrix(); //restore matrix

	}



	glfwSwapBuffers(window);

	//rendererOut << "Render ready\n";

	firstRun++;
	//if (firstRun == 10) { ZoomAll(); }

	if ((firstRun *  visSettings->general.graphicsUpdateInterval) < 1. && visSettings->general.autoFitScene) { ZoomAll(); }
	
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


void GlfwRenderer::RenderGraphicsData()
{
	for (auto data : *graphicsDataList)
	{
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		//DRAW POINTS
		GLfloat lineWidth = 0.5f; //has no action so far
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

		if (visSettings->openGL.showFaces)
		{
			glEnable(GL_LIGHTING);
			for (const GLTriangle& trig : data->glTriangles)
			{ //draw faces
				//glColor4f(0.2f, 0.2f, 0.9f, 1.f);
				glBegin(GL_TRIANGLES);
				for (Index i = 0; i < 3; i++)
				{
					glColor4fv(trig.colors[i].GetDataPointer());
					glNormal3fv(trig.normals[i].GetDataPointer());
					glVertex3fv(trig.points[i].GetDataPointer());
				}
				glEnd();
			}
			glDisable(GL_LIGHTING);
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
				Float3 p1 = midPoint + len*trig.normals[0];
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

					Float3 p1 = trig.points[i] + len*trig.normals[i];
					glVertex3f(p1[0], p1[1], p1[2]);
					glEnd();
				}
			}
		}

		if (visSettings->openGL.lineSmooth) { glDisable(GL_LINE_SMOOTH); }

		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		//DRAW TEXT
		//float scale = 0.025f; //scaling of text
		float textheight = visSettings->general.textSize;
		float scaleFactor = 2.f * state->zoom / ((float)state->currentWindowSize[1]); //factor, which gives approximately 1pt textsize
		//float scale = 2.f*textheight * state->zoom / ((float)state->currentWindowSize[1]);

		Float16 m = state->modelRotation;

		Float16 matTp({ m[0],m[4],m[8],m[12], //transpose of modelRotation
					   m[1],m[5],m[9],m[13],
					   m[2],m[6],m[10],m[14],
					   m[3],m[7],m[11],m[15] });

		//if not called from modelview, use the following transformations
		//glMatrixMode(GL_MODELVIEW);
		//glPushMatrix(); //store current matrix -> before rotation
		//glLoadIdentity();
		//glTranslated(-state->centerPoint[0], -state->centerPoint[1], 0.f);
		//glMultMatrixf(state->modelRotation.GetDataPointer());

		//Float3 p0({ 0.f,0.f,0.f }); //texts are drawn at position 0,0,0 ==> everything else done by tranformations

		for (const GLText& t : data->glTexts)
		{
			float scale = textheight * scaleFactor;
			if (t.size != 0.f) { scale = t.size * scaleFactor; }

			float offx = t.offsetX * scale;
			float offy = t.offsetX * scale;
			//draw strings without applying the rotation:
			glPushMatrix(); //store current matrix -> before rotation
			glTranslated(t.point[0], t.point[1], t.point[2]);
			glMultMatrixf(matTp.GetDataPointer());
			DrawString(t.text, scale, Float3({ offx,offy,0.f }), t.color);
			glPopMatrix(); //restore matrix
		}
		//glPopMatrix(); //restore matrix
	}
}















#endif //USE_GLFW_GRAPHICS


