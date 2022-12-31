/** ***********************************************************************************************
* @brief		implemenation for MainSystemContainer 
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
* *** Example code ***
*
************************************************************************************************ */

//#include <chrono> //sleep_for()
//#include <thread>

#include "Main/MainSystem.h"
#include "Pymodules/PybindUtilities.h" //for RenderState conversions
#include "Main/SystemContainer.h"
#include "Main/MainSystemContainer.h"



extern void PyWriteToSysDictionary(const STDstring& key, py::object item);

//can be also used outside MainSystemContainer
py::dict MainSystemContainer::RenderState2PyDict(const RenderState& state)
{
	auto d = py::dict();
	d["centerPoint"] = (const std::vector<float>)state.centerPoint;
	d["rotationCenterPoint"] = (const std::vector<float>)state.rotationCenterPoint;
	d["maxSceneSize"] = state.maxSceneSize;
	d["zoom"] = state.zoom;
	d["currentWindowSize"] = (const std::vector<Index>)state.currentWindowSize;
	d["displayScaling"] = state.displayScaling;

	//current orientation:
	const Float16& A = state.modelRotation;
	//Float9 A33({ A[0], A[1], A[2],  A[4], A[5], A[6],  A[8], A[9], A[10] }); //convert 4x4 into 3x3 matrix as position part of A is [0,0,0]
	//d["modelRotation"] = EXUmath::SlimVectorF9ToStdArray33F(A33);

	//Matrix3DF rotMatrix(3, 3, { A[0], A[1], A[2],  A[4], A[5], A[6],  A[8], A[9], A[10] });
		
	//++++++++++++++++++++++++++++++++++++++++++++
	//old, with numpy, gives problems in output ("array( ...", "dtype=float32")
	//auto rot = EPyUtils::MatrixF2NumPy(rotMatrix);
		
	//++++++++++++++++++++++++++++++++++++++++++++
	//new, list of lists:
	std::array<float, 3> A1 = { A[0], A[1], A[2]  };
	std::array<float, 3> A2 = { A[4], A[5], A[6]  };
	std::array<float, 3> A3 = { A[8], A[9], A[10] };

	py::list rot;
	rot.append(A1);
	rot.append(A2);
	rot.append(A3);

	d["modelRotation"] = rot;

	d["mouseCoordinates"] = (std::array<Real, 2>)(state.mouseCoordinates);
	d["openGLcoordinates"] = (std::array<Real, 2>)(state.openGLcoordinates);

	//for space mouse (3D position + 3D rotation); read ONLY!
	d["joystickPosition"] = (std::array<Real, 3>)(state.joystickPosition);
	d["joystickRotation"] = (std::array<Real, 3>)(state.joystickRotation);
	d["joystickAvailable"] = state.joystickAvailable;

	return d;
}

//! return current render state to a dictionary; can be used afterwards for initilization of modelview matrix
py::dict MainSystemContainer::PyGetRenderState() const
{
	const RenderState& state = visualizationSystems.renderState;

	return RenderState2PyDict(state);
}
//! set current render state with a dictionary
void MainSystemContainer::PySetRenderState(py::dict renderState)
{
	try
	{
		RenderState& state = visualizationSystems.renderState;
			
		//Vector3D centerPoint;
		EPyUtils::SetSlimVectorTemplateSafely<float, 3>(renderState["centerPoint"], state.centerPoint);
		if (renderState.contains("rotationCenterPoint"))
		{
			EPyUtils::SetSlimVectorTemplateSafely<float, 3>(renderState["rotationCenterPoint"], state.rotationCenterPoint);
		}
		state.maxSceneSize = py::cast<float>(renderState["maxSceneSize"]);
		state.zoom = py::cast<float>(renderState["zoom"]);

		Vector2D windowSize;
		EPyUtils::SetVector2DSafely(renderState["currentWindowSize"], windowSize); //no effect when changing
		state.currentWindowSize[0] = (Index)windowSize[0];
		state.currentWindowSize[1] = (Index)windowSize[1];

		//check if all parts of modelRotation (translation part) shall be modified?
		Float16& A = state.modelRotation;
		Matrix3D R;
		EPyUtils::SetNumpyMatrixSafely(renderState["modelRotation"], R);
		A[0] = (float)R(0, 0);
		A[1] = (float)R(0, 1);
		A[2] = (float)R(0, 2);
		A[4] = (float)R(1, 0);
		A[5] = (float)R(1, 1);
		A[6] = (float)R(1, 2);
		A[8] = (float)R(2, 0);
		A[9] = (float)R(2, 1);
		A[10]= (float)R(2, 2);
	}
	catch (const EXUexception& ex)
	{
		SysError("EXUDYN raised internal error in SetRenderState(...):\n" + STDstring(ex.what()) + "\nCheck dictionary format.\n");
	}
	catch (...) //any other exception
	{
		SysError("Unexpected exception during SetRenderState(...)! Check dictionary format.\n");
	}
}

//! this function links the VisualizationSystem to renderer; returns true if renderer exists/running
bool MainSystemContainer::AttachToRenderEngineInternal(bool warnNoRenderer)
{
	bool rv = visualizationSystems.AttachToRenderEngine(warnNoRenderer); //raise warning
	if (rv)
	{
		py::module exudynModule = py::module::import("exudyn");
		exudynModule.attr("sys")["currentRendererSystemContainer"] = this; //use pointer, otherwise SC is copied ...?
		//PyWriteToSysDictionary("currentRendererSystemContainer", *this);
		return true;
	}
	return false;
}

//! this function releases the VisualizationSystem from the render engine;
bool MainSystemContainer::DetachFromRenderEngineInternal(bool warnNoRenderer)
{
	py::module exudynModule = py::module::import("exudyn");
	exudynModule.attr("sys")["currentRendererSystemContainer"] = 0;
	return visualizationSystems.DetachFromRenderEngine(&visualizationSystems, warnNoRenderer);
}


py::list MainSystemContainer::PyGetCurrentMouseCoordinates(bool useOpenGLcoordinates) const
{
	if (!useOpenGLcoordinates) 
	{ 
		return py::cast((std::array<Real, 2>)(visualizationSystems.renderState.mouseCoordinates)); 
	}
	else 
	{ 
		return py::cast((std::array<Real, 2>)(visualizationSystems.renderState.openGLcoordinates));
	}
}
//*********************************************************************
//object factory functions

//!Add a MainSystem (and its according CSystem) to the system container
MainSystem& MainSystemContainer::AddMainSystem()
{
	//pout << "MainSystemContainer::AddMainSystem ...\n";
	CSystem* cSystem = new CSystem();
	GetCSystems().Append(cSystem);

	MainSystem* mainSystem = new MainSystem();
	cSystem->GetSystemData().SetMainSystemBacklink(mainSystem);
	mainSystem->mainSystemData.SetCSystemData(&(cSystem->GetSystemData()));
	mainSystem->cSystem = cSystem;
	mainSystem->LinkToVisualizationSystem(); //links the system to be rendered in OpenGL
	visualizationSystems.Append(&mainSystem->GetVisualizationSystem());
	GetMainSystems().Append(mainSystem);
	mainSystem->SetInteractiveMode(false);
	mainSystem->SetMainSystemIndex(GetCSystems().NumberOfItems()-1); //index will not change hereafter
	mainSystem->SetMainSystemContainer(this);
	
	return *mainSystem;
}

//! delete all MainSystems, detach render engine from main systems and and, delete all VisualizationSystems
void MainSystemContainer::Reset()
{
	//pout << "MainSystemContainer::Reset()" << "\n";
	visualizationSystems.DetachFromRenderEngine(&visualizationSystems);
	//pout << "MainSystemContainer::Reset():1" << "\n";
	visualizationSystems.Reset(); //this takes care that no invalid pointers to some VisualizationSystem are left
	for (auto item : mainSystems)
	{
		item->UnlinkVisualizationSystem();
		item->Reset();

		//now done in MainSystem: delete item->cSystem; //allocated in AddMainSystem; C-Items are deleted in SystemData.Reset()
		delete item; //allocated in AddMainSystem; MainItems are deleted in MainSystemData.Reset()
	}
	mainSystems.Flush();
	//pout << "MainSystemContainer::Reset() finished" << "\n";
	//visualizationSystems are already deleted by "delete item" above: DON'T DO THAT; visualizationSystems.Reset();
}

//! return reference to a MainSystem
MainSystem& MainSystemContainer::GetMainSystem(Index systemNumber)
{
	if (systemNumber < mainSystems.NumberOfItems())
	{
		return *(GetMainSystems()[systemNumber]);
	}
	else
	{
		PyError(STDstring("GetMainSystem: Cannot access system ") + EXUstd::ToString(systemNumber) +
			" (number of systems = " + EXUstd::ToString(mainSystems.NumberOfItems()) + "); added and returned a new system");
		return AddMainSystem();
	}
}

//! send redraw to all MainSystems
void MainSystemContainer::SendRedrawSignal()
{
	for (auto item : mainSystems)
	{
		//not necessary, because anyway checked in UpdateGraphicsData ...:
		//if (item->GetCSystem()->IsSystemConsistent()) //otherwise, redraw is impossible ...
		item->GetCSystem()->GetPostProcessData()->SendRedrawSignal();
	}
}

