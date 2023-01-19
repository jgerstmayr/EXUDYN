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
	d["centerPoint"] = EPyUtils::SlimVector2NumPy(Vector3D({ state.centerPoint[0],state.centerPoint[1],state.centerPoint[2] }) );
	d["rotationCenterPoint"] = EPyUtils::SlimVector2NumPy(Vector3D({ state.rotationCenterPoint[0],state.rotationCenterPoint[1],state.rotationCenterPoint[2] }) );
	d["maxSceneSize"] = state.maxSceneSize;
	d["zoom"] = state.zoom;
	d["currentWindowSize"] = EPyUtils::SlimArrayIndex2NumPy(state.currentWindowSize);
	d["displayScaling"] = state.displayScaling;
	//++++++++++++++++++++++++++++++++++++++++++++
	//current orientation:
	Matrix3D m3D(3,3); //this matrix is smaller than the stored matrix!
	for (Index i = 0; i < 3; i++)
	{
		for (Index j = 0; j < 3; j++)
		{
			m3D(i, j) = (Real)state.modelRotation(i, j);
		}
	}

	auto rotMatrix = EPyUtils::Matrix2NumPyTemplate(m3D);
	d["modelRotation"] = rotMatrix;

	//++++++++++++++++++++++++++++++++++++++++++++
	//current projection matrix:
	Matrix4D m4D;
	m4D.CopyFrom(state.projectionMatrix);
	auto projectionMatrix = EPyUtils::Matrix2NumPyTemplate(m4D); //converts to double
	d["projectionMatrix"] = projectionMatrix;
	//++++++++++++++++++++++++++++++++++++++++++++

	d["mouseCoordinates"] = EPyUtils::SlimVector2NumPy(state.mouseCoordinates);
	d["openGLcoordinates"] = EPyUtils::SlimVector2NumPy(state.openGLcoordinates);

	//for space mouse (3D position + 3D rotation); read ONLY!
	d["joystickPosition"] = EPyUtils::SlimVector2NumPy(state.joystickPosition);
	d["joystickRotation"] = EPyUtils::SlimVector2NumPy(state.joystickRotation);
	d["joystickAvailable"] = state.joystickAvailable;

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//OpenVR
	if (state.openVRstate.isActivated) //add openVR only if enabled (which also means compiled ...)
	{
		auto VR = py::dict();

		Matrix4D m4D;
		m4D.CopyFrom(state.openVRstate.HMDpose); VR["HMDpose"] = EPyUtils::Matrix2NumPyTemplate(m4D);
		m4D.CopyFrom(state.openVRstate.projectionLeft); VR["projectionLeft"] = EPyUtils::Matrix2NumPyTemplate(m4D);
		m4D.CopyFrom(state.openVRstate.eyePosLeft); VR["eyePosLeft"] = EPyUtils::Matrix2NumPyTemplate(m4D);
		m4D.CopyFrom(state.openVRstate.projectionRight); VR["projectionRight"] = EPyUtils::Matrix2NumPyTemplate(m4D);
		m4D.CopyFrom(state.openVRstate.eyePosRight); VR["eyePosRight"] = EPyUtils::Matrix2NumPyTemplate(m4D);

		auto controllerPoseList = py::list();
		for (auto mat : state.openVRstate.controllerPoses)
		{
			m4D.CopyFrom(mat); controllerPoseList.append(EPyUtils::Matrix2NumPyTemplate(m4D));
		}
		VR["controllerPoses"] = controllerPoseList;

		auto trackerPoseList = py::list();
		for (auto mat : state.openVRstate.trackerPoses)
		{
			m4D.CopyFrom(mat); trackerPoseList.append(EPyUtils::Matrix2NumPyTemplate(m4D));
		}
		VR["trackerPoses"] = trackerPoseList;

		d["openVR"] = VR;
		//TODO: controllerActions missing
	}
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	return d;


	//OLD STYLE, not returning numpy arrays:
	//auto d = py::dict();
	//d["centerPoint"] = (const std::vector<float>)state.centerPoint;
	//d["rotationCenterPoint"] = (const std::vector<float>)state.rotationCenterPoint;
	//d["maxSceneSize"] = state.maxSceneSize;
	//d["zoom"] = state.zoom;
	//d["currentWindowSize"] = (const std::vector<Index>)state.currentWindowSize;
	//d["displayScaling"] = state.displayScaling;

	//++++++++++++++++++++++++++++++++++++++++++++
	////current orientation:
	//const float* A = state.modelRotation.GetDataPointer();
	//	
	//new, list of lists:
	//std::array<float, 3> A1 = { A[0], A[1], A[2]  };
	//std::array<float, 3> A2 = { A[4], A[5], A[6]  };
	//std::array<float, 3> A3 = { A[8], A[9], A[10] };

	//py::list rotMatrix;
	//rotMatrix.append(A1);
	//rotMatrix.append(A2);
	//rotMatrix.append(A3);
	//d["modelRotation"] = rotMatrix;

	////++++++++++++++++++++++++++++++++++++++++++++
	////current projection matrix:
	//m.CopyFrom(state.projectionMatrix);
	//auto projectionMatrix = EPyUtils::Matrix2NumPyTemplate(m); //converts to double
	//d["projectionMatrix"] = projectionMatrix;
	////++++++++++++++++++++++++++++++++++++++++++++

	//d["mouseCoordinates"] = (std::array<Real, 2>)(state.mouseCoordinates);
	//d["openGLcoordinates"] = (std::array<Real, 2>)(state.openGLcoordinates);

	////for space mouse (3D position + 3D rotation); read ONLY!
	//d["joystickPosition"] = (std::array<Real, 3>)(state.joystickPosition);
	//d["joystickRotation"] = (std::array<Real, 3>)(state.joystickRotation);
	//d["joystickAvailable"] = state.joystickAvailable;


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
			
		if (renderState.contains("centerPoint"))
		{
			EPyUtils::SetSlimVectorTemplateSafely<float, 3>(renderState["centerPoint"], state.centerPoint); //conversion to float works ...
		}
		if (renderState.contains("rotationCenterPoint"))
		{
			EPyUtils::SetSlimVectorTemplateSafely<float, 3>(renderState["rotationCenterPoint"], state.rotationCenterPoint);
		}
		if (renderState.contains("maxSceneSize")) { state.maxSceneSize = py::cast<float>(renderState["maxSceneSize"]); }
		if (renderState.contains("zoom")) { state.zoom = py::cast<float>(renderState["zoom"]); }

		if (renderState.contains("currentWindowSize"))
		{
			Vector2D windowSize;
			EPyUtils::SetVector2DSafely(renderState["currentWindowSize"], windowSize); //no effect when changing; maybe changes in future
			state.currentWindowSize[0] = (Index)windowSize[0];
			state.currentWindowSize[1] = (Index)windowSize[1];
		}
		if (renderState.contains("modelRotation"))
		{
			//check if all parts of modelRotation (translation part) shall be modified?
			Matrix4DF& A = state.modelRotation;
			Matrix3D R; 
			EPyUtils::SetNumpyMatrixSafely(renderState["modelRotation"], R);
			//map rotation matrix to part of 16 components in A; other components untouched!
			for (Index i = 0; i < 3; i++)
			{
				for (Index j = 0; j < 3; j++)
				{
					A(i, j) = (float)R(i, j);
				}
			}
		}

		//++++++++++++++++++++++++++++++++++++++++++++
		//current projection matrix:
		if (renderState.contains("projectionMatrix"))
		{
			Matrix4D m;
			EPyUtils::SetNumpyMatrixSafely(renderState["projectionMatrix"], m);
			state.projectionMatrix.CopyFrom(m);
		}
		//++++++++++++++++++++++++++++++++++++++++++++

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

