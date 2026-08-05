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
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
************************************************************************************************ */
#ifndef MAINSYSTEMCONTAINER__H
#define MAINSYSTEMCONTAINER__H

//! a container for handling the materials
//! substructure of MainRenderer
//! preserving consistency with visualizationSettings material0-material9
class MainGraphicsMaterialList
{
private:
	static const Index minNumberOfMaterials = 10;
	MainSystemContainer* mainSystemContainer;	//!< access back via main system
	//std::vector<VSettingsMaterial> data;			//!< moved to VisualizationSystemContainer
public:
	static MainGraphicsMaterialList* ForbidConstructor()
	{
		CHECKandTHROWstring("constructor GraphicsMaterialList() may not be called. It is automatically created inside Renderer and other usage of this class is not possible.");
		return new MainGraphicsMaterialList(); //this is never called
	}
	void Initialize(MainSystemContainer* backlink) 
	{
		mainSystemContainer = backlink;
		Reset(); //create default materials
	}
	//- read / write access with[i]

	//! reset with default materials
	void Reset();

	//! helper
	Index IndexOrName2Index(py::object indexOrName) const;

	VSettingsMaterial NewMaterial() const { return VSettingsMaterial(); };

	//! read/write access
	const VSettingsMaterial& GetMaterial(py::object indexOrName) const;
	//don't allow reference access, as sync with visSettings would be lost!: VSettingsMaterial& GetMaterial(py::object indexOrName);
	py::dict PyGetMaterialDict(py::object indexOrName) const;

	//! set material i
	void SetMaterial(Index i, const VSettingsMaterial& material);

	//!access with dictionary or SettingsMaterial
	void PySetMaterial(py::object indexOrName, py::object material);

	//! copying from/to visualizationSettings
	void CopyFromVisSettings();
	void CopyToVisSettings() const;

	//! get available number of materials
	Index NumberOfItems() const;

	//! append material
	//void Append(const VSettingsMaterial& material);
	Index PyAppend(py::object material);

	virtual void Print(std::ostream& os) const;

	friend std::ostream& operator<<(std::ostream& os, const MainGraphicsMaterialList& object)
	{
		object.Print(os);
		return os;
	}

};

//! substructure for rendering tasks with backlink to MainSystemContainer; 
//! serves as unified access to rendering in Exudyn
class MainRenderer
{
private:
	MainSystemContainer* mainSystemContainer;
	mutable Index warnRendererCount;			//!< this is used to limit warnings during transition to new renderer functionality
public:
	//public access to alleviate pybind interface:
	MainGraphicsMaterialList materials;			//!< also accessible with Pybind11

	static MainRenderer* ForbidConstructor()
	{
		CHECKandTHROWstring("constructor Renderer() may not be called. It is automatically created inside SystemContainer and other usage of this class is not possible.");
		return new MainRenderer(); //this is never called
	}

	void Initialize(MainSystemContainer* backlink) 
	{
		mainSystemContainer = backlink; 
		warnRendererCount = 0;
		materials.Initialize(backlink);
	}

	void DeprecationWarning(const STDstring& oldFunctionName, const STDstring& newFunctionName) const
	{
		if (warnRendererCount < 3)
		{
			warnRendererCount++;
			PyWarning("The call to SystemContainer function " + oldFunctionName + 
				" is deprecated. For SystemContainer SC use SC.renderer." + newFunctionName + " instead!\n");
		}
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//these functions are directly mapped to GlfwRenderer:
	//! start render engine
	bool Start(Index verbose = true);
	//! stop render engine
	void Stop();
	//! check if render engine is activated
	bool IsActive() const;
	//! attach render engine to SystemContainer (automatically done for current (=newest) SystemContainer)
	bool Attach();
	//! detach render engine from SystemContainer
	bool Detach();

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//functions that previously existed in MainSystemContainer
	//! generic function to wait for continue, stop or just process tasks
	//! if -1, it waits for continue/stop; otherwise wait milliseconds (0=no wait)
	bool DoIdleTasks(Real waitSeconds = -1., bool printPauseMessage = true);

	//! send zoom all request for next scene redraw:
	void ZoomAll(bool computeMaxScene = true, Index viewID = VisualizationSystemContainer::mainViewID);

	//! set model view with zoom for window height, (rotationVector & centerPoint) for model view
	void SetModelView(float zoom, const std::vector<Real>& rotationVector, const std::vector<Real>& centerPoint,
		Index viewID = VisualizationSystemContainer::mainViewID);

	//! add subview which is either only activated (for raytracing), or activated and visible
	void EnableView(Index viewID = VisualizationSystemContainer::mainViewID, bool visible = true);

	//! remove/close subview
	void DisableView(Index viewID = VisualizationSystemContainer::mainViewID);

	//! redraw current view and save image
	void RedrawAndSaveImage(Index viewID);

	//! redraw current view and get image
	py::array_t<uint8_t> RedrawAndGetImage(bool useRaytracer = false, Index viewID = VisualizationSystemContainer::mainViewID);

	//! get render state dictionary
	py::dict GetState(Index viewID = VisualizationSystemContainer::mainViewID) const;

	//! set render state dictionary
	void SetState(py::dict renderState, bool waitForRendererFullStartup = false,
		Index viewID = VisualizationSystemContainer::mainViewID);

	//! get OpenGL coordinates as list, faster than using render state
	py::list GetMouseCoordinates(bool useOpenGLcoordinates = false, Index viewID = VisualizationSystemContainer::mainViewID) const;

	//! get infor about item selection and reset selection if flag is set true
	py::list GetItemSelection(bool resetSelection = true, Index viewID = VisualizationSystemContainer::mainViewID);

	//! renderer functionality, to allow user to reset the RenderState using visulizationSettings
	void ResetState();

	//! send redraw signal for renderer (e.g. if no simulaiton runs, but graphicsData has changed)
	void SendRedrawSignal();

	//! retrieve number of redraws, can be used to see whether renderer is fully started and first image drawn (zoom all)
	Index GetRedrawCount() const;

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! check if renderer is active; if not, write error message
	void RendererInActiveError(const char* functionName) const
	{
		if (!IsActive())
		{
			PyError(STDstring("Renderer function '") + functionName + "' has been called, but renderer was not active (call renderer.Start() first!)");
		}
	}

	//! check if renderer is active; if not, write error message
	//! implementation after definition of MainSystemContainer
	void ViewDisabledError(Index viewID, const char* functionName) const;
};

//! This is the extension of SystemContainer, which contains all objects needed for management and in the Python world.
//  The System objects have their corresponding python object, which contains a pointer to the system (main) objects.
//  Thus, any object list in SystemContainer is doubled by its Main correspondent
class MainSystemContainer // : public SystemContainer
{
protected:
	ResizableArray<MainSystem*> mainSystems;			//!< contains one or a set of complete multibody/finite element systems
	VisualizationSystemContainer visualizationSystems;  //!< contains all linking to visualization
	//should be after visualizationSystems due to initialization
	MainRenderer renderer;								//!< substructure mainly for Python: in Python this represents the substructure which handles all rendering tasks
public:
	MainSystemContainer()
	{
		renderer.Initialize(this);
		AttachToRenderEngineInternal(); //also sets exu.sys['currentRendererSystemContainer']
	}

	//! delete all systems (otherwise they could do illegal operations)
	virtual ~MainSystemContainer()
	{
		Reset(); //delete operator needs to delete all systems (otherwise they could do illegal operations)
	}

	//! function for getting all data and state; for pickling
	py::dict GetDictionary() const;
	//! function for setting all data from dict; for pickling
	void SetDictionary(const py::dict& d);



	//! Write (Reference) access to:contains one or a set of complete multibody/finite element systems
	ResizableArray<MainSystem*>& GetMainSystems() { return mainSystems; }
	//! Read (Reference) access to:contains one or a set of complete multibody/finite element systems
	const ResizableArray<MainSystem*>& GetMainSystems() const { return mainSystems; }

	const VisualizationSettings& PyGetVisualizationSettings() const { return visualizationSystems.settings; }
	void PySetVisualizationSettings(const VisualizationSettings& visualizationSettings) { visualizationSystems.settings = visualizationSettings; }

	const MainRenderer& PyGetRenderer() const { return renderer; }
	void PySetRenderer(const MainRenderer& rendererInit) { renderer = rendererInit; }

	const VisualizationSystemContainer& GetVisualizationSystemContainer() const { return visualizationSystems; }
	VisualizationSystemContainer& GetVisualizationSystemContainer() { return visualizationSystems; }

	//! this function links the VisualizationSystemContainer to a render engine, such that the changes in the graphics structure drawn upon updates, etc.
	bool AttachToRenderEngine();
	
	//! internal function with option for warning
	bool AttachToRenderEngineInternal(bool warnNoRenderer = false);

	//! this function releases the VisualizationSystemContainer from the render engine;
	bool DetachFromRenderEngine();

	//! internal function with option for warning
	bool DetachFromRenderEngineInternal(bool warnNoRenderer = false);

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! DEPRECATED, DO NOT USE! USE renderer functions
	//! this function waits for the stop flag in the render engine;
	bool WaitForRenderEngineStopFlag();
	
	//! DEPRECATED
	//! send renderer zoom all request
	void PyZoomAll();
	
	//! DEPRECATED
	//! redraw current view and save image
	void RedrawAndSaveImage();

	//can be also used outside MainSystemContainer
	static py::dict RenderState2PyDict(const RenderState& state);

	//! DEPRECATED
	//! return current render state to a dictionary; can be used afterwards for initilization of modelview matrix
	py::dict PyGetRenderState() const;

	//! DEPRECATED
	//! set current render state with a dictionary
	void PySetRenderState(py::dict renderState, bool waitForRendererFullStartup=false);

	//! DEPRECATED
	//! get OpenGL coordinates as list, faster than using render state
	py::list PyGetCurrentMouseCoordinates(bool useOpenGLcoordinates = false) const;

	//! DEPRECATED
	//! send redraw signal for renderer (e.g. if no simulaiton runs, but graphicsData has changed)
	void SendRedrawSignal();

	//*********************************************************************
	//object factory functions

	//!Add a MainSystem (and its according CSystem) to the system container
	MainSystem& AddMainSystem();

	//! append an existing mainSystem, which is already initialized to SystemContainer
	Index AppendMainSystem(MainSystem& mainSystem);

	//! delete all MainSystems, detach render engine from main systems and and, delete all VisualizationSystems
	void Reset();

	//! return reference to a MainSystem
	MainSystem& GetMainSystem(Index systemNumber);
};

inline void MainRenderer::ViewDisabledError(Index viewID, const char* functionName) const
{
	VisualizationSystemContainer& VSC = mainSystemContainer->GetVisualizationSystemContainer();
	if (!VSC.GetRenderViewData(viewID).renderState.viewEnabled)
	{
		PyError(STDstring("Renderer function '") + functionName + "' has been called for view " + EXUstd::ToString(viewID) + ", view is not enabled (call renderer.EnableView(...) first)");
	}
}


#endif
