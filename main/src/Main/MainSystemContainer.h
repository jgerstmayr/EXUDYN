/** ***********************************************************************************************
* @class        MainSystemContainer
* @brief		MainSystemContainer contains a list of MainSystems for managing by Python
*
* @author		Gerstmayr Johannes
* @date			2019-05-02 (generated)
* @pre			...
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
* 				
*
* *** Example code ***
*
************************************************************************************************ */
#pragma once

//! This is the extension of SystemContainer, which contains all objects needed for management and in the Python world.
//  The System objects have their corresponding python object, which contains a pointer to the system (main) objects.
//  Thus, any object list in SystemContainer is doubled by its Main correspondent
class MainSystemContainer : public SystemContainer
{
protected:
	ResizableArray<MainSystem*> mainSystems;			//!< contains one or a set of complete multibody/finite element systems
	VisualizationSystemContainer visualizationSystems;  //!< contains all linking to visualization
	//MainSolverContainer solvers;                      //!< contains a structure with all solver-relevant structures (dynamic, static, etc.)

public:
	MainSystemContainer()
	{

	}

	//! Write (Reference) access to:contains one or a set of complete multibody/finite element systems
	ResizableArray<MainSystem*>& GetMainSystems() { return mainSystems; }
	//! Read (Reference) access to:contains one or a set of complete multibody/finite element systems
	const ResizableArray<MainSystem*>& GetMainSystems() const { return mainSystems; }

	const VisualizationSettings& PyGetVisualizationSettings() const { return visualizationSystems.settings; }
	void PySetVisualizationSettings(const VisualizationSettings& visualizationSettings) { visualizationSystems.settings = visualizationSettings; }

	//! this function waits for the stop flag in the render engine;
	bool WaitForRenderEngineStopFlag() { return visualizationSystems.WaitForRenderEngineStopFlag(); }

	void PyZoomAll() { visualizationSystems.zoomAllRequest = true; }

	void RedrawAndSaveImage() { visualizationSystems.RedrawAndSaveImage(); }

	//! return current render state to a dictionary; can be used afterwards for initilization of modelview matrix
	py::dict PyGetRenderState() const
	{
		auto d = py::dict();
		const RendererState& state = visualizationSystems.rendererState;

		d["centerPoint"] = (const std::vector<float>)state.centerPoint;
		d["maxSceneSize"] = state.maxSceneSize;
		d["zoom"] = state.zoom;
		d["currentWindowSize"] = (const std::vector<Index>)state.currentWindowSize;
		
		//current orientation:
		const Float16& A = state.modelRotation;
		//Float9 A33({ A[0], A[1], A[2],  A[4], A[5], A[6],  A[8], A[9], A[10] }); //convert 4x4 into 3x3 matrix as position part of A is [0,0,0]
		//d["modelRotation"] = EXUmath::SlimVectorF9ToStdArray33F(A33);
		
		Matrix3DF rotMatrix(3, 3, { A[0], A[1], A[2],  A[4], A[5], A[6],  A[8], A[9], A[10] });
		d["modelRotation"] = EPyUtils::MatrixF2NumPy(rotMatrix);
		return d;
	}
	//! set current render state with a dictionary
	void PySetRenderState(py::dict renderState)
	{
		try
		{
			RendererState& state = visualizationSystems.rendererState;
			
			Vector3D centerPoint;
			EPyUtils::SetVectorTemplateSafely<float,3>(renderState["centerPoint"], state.centerPoint);

			state.maxSceneSize = py::cast<float>(renderState["maxSceneSize"]);
			state.zoom = py::cast<float>(renderState["zoom"]);

			Vector2D windowSize;
			EPyUtils::SetVector2DSafely(renderState["currentWindowSize"], windowSize);
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
		catch (const std::exception& ex)
		{
			SysError("EXUDYN raised internal error in SetRenderState(...):\n" + STDstring(ex.what()) + "\nCheck dictionary format.\n");
		}
		catch (...) //any other exception
		{
			SysError("Unexpected exception during SetRenderState(...)! Check dictionary format.\n");
		}
	}
	//*********************************************************************
	//object factory functions

	//!Add a MainSystem (and its according CSystem) to the system container
	MainSystem& AddMainSystem()
	{
		//pout << "MainSystemContainer::AddMainSystem ...\n";
		CSystem* cSystem = new CSystem();
		GetCSystems().Append(cSystem);

		MainSystem* mainSystem = new MainSystem();
		mainSystem->mainSystemData.SetCSystemData(&(cSystem->GetSystemData()));
		mainSystem->cSystem = cSystem;
		mainSystem->LinkToRenderEngine(); //links the system to be rendered in OpenGL
		visualizationSystems.Append(&mainSystem->GetVisualizationSystem());
		GetMainSystems().Append(mainSystem);
		mainSystem->SetInteractiveMode(false);

		return *mainSystem;
	}

	//! delete all MainSystems, detach render engine from main systems and and, delete all VisualizationSystems
	void Reset()
	{
		for (auto item : mainSystems)
		{
			item->DetachRenderEngine();
			item->Reset();
			delete item->cSystem; //allocated in AddMainSystem; C-Items are deleted in SystemData.Reset()
			delete item; //allocated in AddMainSystem; MainItems are deleted in MainSystemData.Reset()
		}
		mainSystems.Flush();
		visualizationSystems.DetachRenderEngine();
		visualizationSystems.Reset();
	}

	//! return reference to a MainSystem
	MainSystem& GetMainSystem(Index systemNumber)
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

};
