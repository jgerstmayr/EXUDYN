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
	py::dict PyGetRenderState()
	{
		auto d = py::dict();
		const RendererState& state = visualizationSystems.rendererState;

		d["centerPoint"] = (const std::vector<float>)state.centerPoint;
		d["maxSceneSize"] = state.maxSceneSize;
		d["zoom"] = state.zoom;
		
		d["currentWindowSize"] = (const std::vector<Index>)state.currentWindowSize;
		
		//current orientation:
		const Float16& A = state.modelRotation;
		Float9 A33({ A[0], A[1], A[2],  A[4], A[5], A[6],  A[8], A[9], A[10] }); //convert 4x4 into 3x3 matrix as position part of A is [0,0,0]
		d["modelRotation"] = EXUmath::SlimVectorF9ToStdArray33F(A33);


		//d["openGLModelViewMatrix"] = EXUmath::SlimVectorF16ToStdArray44F(state.openGLModelViewMatrix);
		//d["openGLProjection"] = EXUmath::SlimVectorF16ToStdArray44F(state.openGLProjection);

		return d;

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

		return *mainSystem;
	}

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

};
