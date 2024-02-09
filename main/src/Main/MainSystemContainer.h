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

//! This is the extension of SystemContainer, which contains all objects needed for management and in the Python world.
//  The System objects have their corresponding python object, which contains a pointer to the system (main) objects.
//  Thus, any object list in SystemContainer is doubled by its Main correspondent
class MainSystemContainer // : public SystemContainer
{
protected:
	ResizableArray<MainSystem*> mainSystems;			//!< contains one or a set of complete multibody/finite element systems
	VisualizationSystemContainer visualizationSystems;  //!< contains all linking to visualization
	//MainSolverContainer solvers;                      //!< contains a structure with all solver-relevant structures (dynamic, static, etc.)

public:
	MainSystemContainer()
	{
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

	const VisualizationSystemContainer& GetVisualizationSystemContainer() const { return visualizationSystems; }
	VisualizationSystemContainer& GetVisualizationSystemContainer() { return visualizationSystems; }

	//! this function waits for the stop flag in the render engine;
	bool WaitForRenderEngineStopFlag() { return visualizationSystems.WaitForRenderEngineStopFlag(); }

	//! this function links the VisualizationSystemContainer to a render engine, such that the changes in the graphics structure drawn upon updates, etc.
	bool AttachToRenderEngine() { return AttachToRenderEngineInternal(true); }; //only warn in this case!

	//! internal function with option for warning
	bool AttachToRenderEngineInternal(bool warnNoRenderer = false);

	//! this function releases the VisualizationSystemContainer from the render engine;
	bool DetachFromRenderEngine() { return DetachFromRenderEngineInternal(true); } //only warn in this case

	//! internal function with option for warning
	bool DetachFromRenderEngineInternal(bool warnNoRenderer = false);

	void PyZoomAll() { visualizationSystems.zoomAllRequest = true; }

	void RedrawAndSaveImage() { visualizationSystems.RedrawAndSaveImage(); }

	//can be also used outside MainSystemContainer
	static py::dict RenderState2PyDict(const RenderState& state);
	//{
	//	auto d = py::dict();
	//	d["centerPoint"] = (const std::vector<float>)state.centerPoint;
	//	d["maxSceneSize"] = state.maxSceneSize;
	//	d["zoom"] = state.zoom;
	//	d["currentWindowSize"] = (const std::vector<Index>)state.currentWindowSize;

	//	//current orientation:
	//	const Float16& A = state.modelRotation;
	//	//Float9 A33({ A[0], A[1], A[2],  A[4], A[5], A[6],  A[8], A[9], A[10] }); //convert 4x4 into 3x3 matrix as position part of A is [0,0,0]
	//	//d["modelRotation"] = EXUmath::SlimVectorF9ToStdArray33F(A33);

	//	Matrix3DF rotMatrix(3, 3, { A[0], A[1], A[2],  A[4], A[5], A[6],  A[8], A[9], A[10] });
	//	
	//	//++++++++++++++++++++++++++++++++++++++++++++
	//	//old, with numpy, gives problems in output ("array( ...", "dtype=float32")
	//	//auto rot = EPyUtils::MatrixF2NumPy(rotMatrix);
	//	
	//	//++++++++++++++++++++++++++++++++++++++++++++
	//	//new, list of lists:
	//	std::array<float, 3> A1 = { A[0], A[1], A[2]  };
	//	std::array<float, 3> A2 = { A[4], A[5], A[6]  };
	//	std::array<float, 3> A3 = { A[8], A[9], A[10] };

	//	py::list rot;
	//	rot.append(A1);
	//	rot.append(A2);
	//	rot.append(A3);

	//	d["modelRotation"] = rot;

	//	d["mouseCoordinates"] = (std::array<Real, 2>)(state.mouseCoordinates);
	//	d["openGLcoordinates"] = (std::array<Real, 2>)(state.openGLcoordinates);

	//	//now in stateMachine:
	//	//d["selectionMouseCoordinates"] = (std::array<Real, 2>)(state.selectionMouseCoordinates);
	//	//d["selectionModeOn"] = state.selectMode;

	//	return d;
	//}

	//! return current render state to a dictionary; can be used afterwards for initilization of modelview matrix
	py::dict PyGetRenderState() const;

	//! set current render state with a dictionary
	void PySetRenderState(py::dict renderState);

	py::list PyGetCurrentMouseCoordinates(bool useOpenGLcoordinates = false) const;

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

	void SendRedrawSignal();

	//delete:
	//virtual void Print(std::ostream& os) const
	//{
	//	os << "SystemContainer" << ":\n";
	//	os << "  cSystems = " << mainSystems << "\n";
	//	os << "\n";
	//}

	//friend std::ostream& operator<<(std::ostream& os, const MainSystemContainer& object)
	//{
	//	object.Print(os);
	//	return os;
	//}

};

#endif
