/** ***********************************************************************************************
* @class        VisualizationSystemBase
* @brief		A base class for visualization system for common interface to different graphics renderers
* @details		Details:
 				- a visualization system, containing data and functions for visualization
*
* @author		Gerstmayr Johannes
* @date			2019-05-24 (generated)
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
#ifndef VISUALIZATIONSYSTEMBASE__H
#define VISUALIZATIONSYSTEMBASE__H

class MainSystem;

//! A generic visualization class, which is used by Renderer class GLFW and Visualization
//! Used to send signals from Renderer class GLFW, e.g. for update of graphics data
class VisualizationSystemContainerBase
{
public:
	//! OpenGL renderer sends message that graphics shall be updated
	virtual void UpdateGraphicsData() = 0;		//!< renderer reports to simulation to update the graphics data
	virtual void InitializeView() = 0;	//!< renderer reports to update the maximum scene coordinates (on initialization)
	virtual void StopSimulation() = 0;			//!< renderer reports to simulation that simulation shall be interrupted
	virtual void ForceQuitSimulation(bool flag = true) = 0;		//!< renderer reports that render window is closed and simulation shall be shut down
	virtual void ContinueSimulation() = 0;		//!< renderer reports to simulation that simulation can be continued
	virtual void UpdateGraphicsDataNow() = 0;	//! renderer signals to update the graphics data, e.g. if settings have changed
	//virtual void SetVisualizationIsRunning(bool flag = true) = 0;	//! renderer signals that visualizationIsRunning flag should be set to "flag"
	virtual void SaveImageFinished() = 0;		//! renderer signals that frame has been grabed and saved
	virtual bool SaveImageRequest() = 0;		//! signal renderer that a frame shall be recorded
	virtual bool GetAndResetZoomAllRequest() = 0;//! get zoom all request and reset to false
	virtual void SetComputeMaxSceneRequest(bool flag) = 0;
	virtual bool GetComputeMaxSceneRequest() = 0;

	virtual void GetMarkerPositionOrientation(Index markerNumber, Index mbsNumber, Vector3D& position, Matrix3D& orientation, bool& hasPosition, bool& hasOrientation) = 0;

	virtual std::string GetComputationMessage(bool solverInformation = true,
		bool solutionInformation = true, bool solverTime = true) = 0; //! any multi-line text message from computation to be shown in renderer (e.g. time, solver, ...)
	virtual MainSystem* GetMainSystemBacklink(Index iSystem) = 0; //! REMOVE: get backlink of ith main system (0 if not existing), temporary for selection
	virtual Index NumberOFMainSystemsBacklink() const = 0; //! REMOVE: get backlink to number of main systems, temporary for selection
	virtual bool DoIdleOperations() = 0; //!< this function does any idle operations (execute some python commands) and returns false if stop flag in the render engine, otherwise true;
	virtual void SetZoomAllRequest(bool flag) = 0; //!< request ZoomAll

	virtual ~VisualizationSystemContainerBase() {} //added for correct deletion of derived classes

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//HELPER FUNCTIONS

	//! compute RGBA-color for given value within a min/max range
	static Float4 ColorBarColor(float minVal, float maxVal, float value)
	{
		//float v = EXUstd::LinearInterpolate(0.f, 1.f, minVal, maxVal, value);
		float v = 0;
		if (maxVal != minVal) { v = (value - minVal) / (maxVal - minVal); }

		if (v < 0.f) { return Float4({ 0.1f, 0.1f, 0.1f, 1.f }); }
		else if (v < 0.25f) { return Float4({ EXUstd::LinearInterpolate(0.1f,0.1f,0.00f,0.25f, v),EXUstd::LinearInterpolate(0.1f,0.9f,0.00f,0.25f, v),EXUstd::LinearInterpolate(0.9f,0.9f,0.00f,0.25f, v), 1.f }); }
		else if (v < 0.50f) { return Float4({ EXUstd::LinearInterpolate(0.1f,0.1f,0.25f,0.50f, v),EXUstd::LinearInterpolate(0.9f,0.9f,0.25f,0.50f, v),EXUstd::LinearInterpolate(0.9f,0.1f,0.25f,0.50f, v), 1.f }); }
		else if (v < 0.75f) { return Float4({ EXUstd::LinearInterpolate(0.1f,0.9f,0.50f,0.75f, v),EXUstd::LinearInterpolate(0.9f,0.9f,0.50f,0.75f, v),EXUstd::LinearInterpolate(0.1f,0.1f,0.50f,0.75f, v), 1.f }); }
		else if (v <= 1.00f) { return Float4({ EXUstd::LinearInterpolate(0.9f,0.9f,0.75f,1.00f, v),EXUstd::LinearInterpolate(0.9f,0.1f,0.75f,1.00f, v),EXUstd::LinearInterpolate(0.1f,0.1f,0.75f,1.00f, v), 1.f }); }

		return Float4({ 0.9f, 0.9f, 0.9f, 1.f });
		//{ 0.1, 0.1, 0.9 }, //previous values
		//{ 0.1, 0.9, 0.9 },
		//{ 0.1, 0.9, 0.1 },
		//{ 0.9, 0.9, 0.1 },
		//{ 0.9, 0.1, 0.1 },
	}
};

class OpenVRaction
{
public:
	Float2 trackpad;
	float trigger;
	bool button1;
	bool button2;
	bool button3;
	bool button4;

	void Init()
	{
		trackpad = Float2({ 0, 0 });
		trigger = 0;
		button1 = false;
		button2 = false;
		button3 = false;
		button4 = false;
	}
};

//for interaction with openVR
class OpenVRState
{
public:
	void Initialize(bool setActivated) 
	{
		isActivated = setActivated;
		controllerPoses.Flush();
		controllerActions.Flush();
		trackerPoses.Flush();

		HMDpose.SetScalarMatrix(4, 1.f);
		projectionLeft.SetScalarMatrix(4, 1.f); 
		eyePosLeft.SetScalarMatrix(4, 1.f); 
		projectionRight.SetScalarMatrix(4, 1.f); 
		eyePosRight.SetScalarMatrix(4, 1.f);
	}

	bool isActivated;										//!< flag is used to enable output with GetRenderState() only if enabled
	ResizableArray<Matrix4DF> controllerPoses;		//!< stores current poses of openVR controllers
	ResizableArray<OpenVRaction> controllerActions;	//!< stored for according controllers (same size as poses)
	ResizableArray<Matrix4DF> trackerPoses;			//!< stores current poses of openVR trackers

	//homogeneous transformations used in openVR / HMD (head mounted display):
	Matrix4DF HMDpose;
	Matrix4DF projectionLeft, eyePosLeft, projectionRight, eyePosRight;
}; 

//MOVE to python generated class!!!
//! rendering state to be controlled via pybind
class RenderState
{
public:
	//GLfloat modelview[16];		//!< current model view matrix
	Float3 centerPoint;				//!< offset of scene in x, y and z direction; initialized by user, then by InitializeView and hereafter changed in OpenGL renderer by ZoomAll (z not used, because it would bring objects out of near/far plane)
	Float3 rotationCenterPoint;	    //!< additional offset for point around which the model view is rotated; standard=[0,0,0]

	float maxSceneSize;				//!< size given e.g. by initial state of system
	float zoom;						//!< this is a factor for zoom
	float displayScaling;			//!< value as retrieved from GLFW glfwGetWindowContentScale

	Index2 currentWindowSize;		//!< current window size in pixel; used to transform mouse movements to OpenGL coordinates; x=width, y=height
	Matrix4DF modelRotation;			//!< rotation used for incremental rotation with mouse / right mouse button
	//DELETE Float16 openGLModelViewMatrix;	//!< modelview matrix as used in openGL
	Matrix4DF projectionMatrix;		//!< projection matrix as used in openGL
	
	Vector2D mouseCoordinates;		//!current mouse coordinates as obtained from GLFW
	Vector2D openGLcoordinates;		//!current mouse coordinates projected in current model view plane (x/y)
	bool mouseLeftPressed;			//!current left mouse button as obtained from GLFW
	bool mouseRightPressed;			//!current right mouse button as obtained from GLFW
	bool mouseMiddlePressed;		//!current middle mouse button as obtained from GLFW

	//for space mouse (3D position + 3D rotation):
	Vector3D joystickPosition;		//!< stored position of joystick, if available
	Vector3D joystickRotation;		//!< stored rotation of joystick, if available
	Index joystickAvailable;		//!< -1 if no joystick available, otherwise the index of the available joystick

	OpenVRState openVRstate;		//!< contains all data exchanced with openVR; this is always available, even if not compiled with openVR
};

#endif
