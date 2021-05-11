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
	virtual void UpdateMaximumSceneCoordinates() = 0;	//!< renderer reports to update the maximum scene coordinates (on initialization)
	virtual void StopSimulation() = 0;			//!< renderer reports to simulation that simulation shall be interrupted
	virtual void ContinueSimulation() = 0;		//!< renderer reports to simulation that simulation can be continued
	virtual void UpdateGraphicsDataNow() = 0;	//! renderer signals to update the graphics data, e.g. if settings have changed
	//virtual void SetVisualizationIsRunning(bool flag = true) = 0;	//! renderer signals that visualizationIsRunning flag should be set to "flag"
	virtual void SaveImageFinished() = 0;		//! renderer signals that frame has been grabed and saved
	virtual bool SaveImageRequest() = 0;		//! signal renderer that a frame shall be recorded
	virtual bool GetAndResetZoomAllRequest() = 0;//! get zoom all request and reset to false
	virtual std::string GetComputationMessage(bool solverInformation = true, 
		bool solutionInformation = true, bool solverTime = true) = 0; //! any multi-line text message from computation to be shown in renderer (e.g. time, solver, ...)
	virtual MainSystem* GetMainSystemBacklink(Index iSystem) = 0; //! REMOVE: get backlink of ith main system (0 if not existing), temporary for selection
	virtual Index NumberOFMainSystemsBacklink() const = 0; //! REMOVE: get backlink to number of main systems, temporary for selection
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

//MOVE to python generated class!!!
//! rendering state to be controlled via pybind
class RenderState
{
public:
	//GLfloat modelview[16]; //current model view matrix
	Float3 centerPoint;		//offset of scene in x, y and z direction; initialized by user, then by UpdateMaximumSceneCoordinates and hereafter changed in OpenGL renderer by ZoomAll (z not used, because it would bring objects out of near/far plane)
	float maxSceneSize;		//size given e.g. by initial state of system
	float zoom;				//this is a factor for zoom

	Index2 currentWindowSize;	//!< current window size in pixel; used to transform mouse movements to OpenGL coordinates; x=width, y=height
	Float16 modelRotation;		//!< rotation used for incremental rotation with mouse / right mouse button
	Float16 openGLModelViewMatrix;	//!< modelview matrix as used in openGL
	Float16 openGLProjection;		//!< projection matrix as used in openGL
	
	Vector2D mouseCoordinates; //!current mouse coordinates as obtained from GLFW
	Vector2D openGLcoordinates; //!current mouse coordinates projected in current model view plane (x/y)
	bool mouseLeftPressed;     //!current left mouse button as obtained from GLFW
	bool mouseRightPressed;    //!current right mouse button as obtained from GLFW
	bool mouseMiddlePressed;   //!current middle mouse button as obtained from GLFW

};

#endif
