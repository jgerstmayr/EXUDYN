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

//MOVE to python generated class!!!

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

//! for a given viewID, get respective settings structure
inline const VSettingsView& GetSettingsView(Index viewID, const VisualizationSettings& visualizationSettings)
{
	CHECKandTHROW(MAX_VIEWS_GLFW == 4, "GetSettingsView: global number of views must be 4");

	switch (viewID)
	{
	case 0: return visualizationSettings.view0;
	case 1: return visualizationSettings.view1;
	case 2: return visualizationSettings.view2;
	case 3: return visualizationSettings.view3;
	default: {
		CHECKandTHROWstring("GetSettingsView: Invalid viewID");
	}
	}
	return visualizationSettings.view0;
}

//! for a given viewID, get respective settings structure
inline const VSettingsLight& GetSettingsLight(Index lightID, const VisualizationSettings& visualizationSettings)
{
	CHECKandTHROW(MAX_LIGHTS_GLFW == 4, "GetSettingsLight: global number of lights must be 4");

	switch (lightID)
	{
	case 0: return visualizationSettings.openGL.light0;
	case 2: return visualizationSettings.openGL.light2;
	case 3: return visualizationSettings.openGL.light3;
	case 1: return visualizationSettings.openGL.light1;
	default: {
		CHECKandTHROWstring("GetSettingsLight: Invalid lightID");
	}
	}
	return visualizationSettings.openGL.light0;
}

//! for a given viewID, get respective settings structure
inline VSettingsView& GetSettingsView(Index viewID, VisualizationSettings& visualizationSettings)
{
	CHECKandTHROW(MAX_VIEWS_GLFW == 4, "GetSettingsView: global number of views must be 4");

	switch (viewID)
	{
	case 0: return visualizationSettings.view0;
	case 1: return visualizationSettings.view1;
	case 2: return visualizationSettings.view2;
	case 3: return visualizationSettings.view3;
	default: {
		CHECKandTHROWstring("GetSettingsView: Invalid viewID");
	}
	}
	return visualizationSettings.view0;
}

class VisualizationSystemContainerBase;

//! rendering state to be controlled via pybind
class RenderState
{
public:
	bool validInitialization;       //!< RenderState is initialized provisionally with default visualizationSettings at creation of SystemContainer; for raytracer, it is initialized with current SC.visualizationSettings at first call to renderer function
	bool viewEnabled;				//!< view enabled, not necessarily open window; user can activate view without window open
	bool windowOpen;				//!< view enabled and GLFW window running;

	//GLfloat modelview[16];		//!< current model view matrix
	Float3 centerPoint;				//!< offset of scene in x and y direction (z omitted!); initialized by user, then by InitializeRenderState and hereafter changed in OpenGL renderer by ZoomAll (z not used, because it would bring objects out of near/far plane)
	Float3 rotationCenterPoint;	    //!< additional offset for point around which the model view is rotated; standard=[0,0,0]

	Box3DF boundingBox;				//!< bounding box for scene in screen coordinates
	float maxSceneSize;				//!< size given e.g. by initial state of system
	float zoom;						//!< this is a factor for zoom
	float displayScaling;			//!< value as retrieved from GLFW glfwGetWindowContentScale

	Index2 currentWindowSize;		//!< current window size in pixel; used to transform mouse movements to OpenGL coordinates; x=width, y=height
	Matrix4DF modelRotation;			//!< rotation used for incremental rotation with mouse / right mouse button
	//DELETE Float16 openGLModelViewMatrix;	//!< modelview matrix as used in openGL
	Matrix4DF projectionMatrix;		//!< projection matrix as used in openGL
	Index projectionInfo;			//!< some additional information on how to apply projection; may be erased in future, just for trial

	Vector2D mouseCoordinates;		//!< current mouse coordinates as obtained from GLFW
	Vector2D openGLcoordinates;		//!< current mouse coordinates projected in current model view plane (x/y)
	bool mouseLeftPressed;			//!< current left mouse button as obtained from GLFW
	bool mouseRightPressed;			//!< current right mouse button as obtained from GLFW
	bool mouseMiddlePressed;		//!< current middle mouse button as obtained from GLFW

	//for item selection:
	Index mouseSelectionMbsNumber;  //!< mbs number of last selected item
	ItemType mouseSelectionItemType;//!< item type of last selected item (None if no selection)
	Index mouseSelectionItemID;		//!< item ID of last selected item
	float mouseSelectionZdepth;		//!< Z-depth of last selected item

	//for space mouse (3D position + 3D rotation):
	Vector3D joystickPosition;		//!< stored position of joystick, if available
	Vector3D joystickRotation;		//!< stored rotation of joystick, if available
	Index joystickAvailable;		//!< -1 if no joystick available, otherwise the index of the available joystick

	OpenVRState openVRstate;		//!< contains all data exchanced with openVR; this is always available, even if not compiled with openVR

	//! minimal distance from screen plane for projection
	constexpr float GetProjectionNearMin() const { return 0.1f; }

	//! increment centerPoint to compensate motion due to rotation with rotationCenterPoint != 0
	void CenterPointUpdateFromRotationChange(const Matrix3DF& previousRotation, const Matrix3DF& newRotation,
		bool useColumnMajor = true)
	{
		if (rotationCenterPoint.GetL2NormSquared() != 0)
		{
			//update centerPoint accordingly (looks like rotation around rotationCenterPoint):
			if (useColumnMajor)
			{
				centerPoint += previousRotation.GetTransposed() * (-rotationCenterPoint) +
					newRotation.GetTransposed() * rotationCenterPoint;
			}
			else
			{
				centerPoint += previousRotation * (-rotationCenterPoint) + newRotation * rotationCenterPoint;
			}
			centerPoint[2] = 0;
		}
	}

	Vector3D GetRotationVector(VisualizationSystemContainerBase* VSC, const VSettingsView& settingsView,
		bool includeTrackMarker = true)
	{
		Matrix3DF rotationMV;
		Float3 translationMV;
		if (includeTrackMarker) { GetRotationTranslationFWithMarker(rotationMV, translationMV, VSC, settingsView); }
		else { GetRotationTranslationF(rotationMV, translationMV); }

		//compute angle and rotation vector:
		//use transposed due to transposed meaning in OpenGL
		Matrix3D R;
		R.CopyFrom(rotationMV);
		Real ep0;
		Vector3D n;
		RigidBodyMath::RotationMatrix2EP(R.GetTransposed(), ep0, n[0], n[1], n[2]);
		Real norm = n.GetL2Norm();

		Real phi = 2. * atan2(norm, ep0);
		if (norm != 0.) { n = (1. / norm) * n; }
		//==> rotationVector = phi * n;
		return phi * n;

		//this computes the rotation part back again:
		//Matrix3D modelRotation = EXUlie::ExpSO3(-rotationVector); //transposed!

	}

	//! compute model rotation and translation for camera-frame; uses either column-major (OpenGL) or row-major format (standard HT)
	void GetRotationTranslationFWithMarker(Matrix3DF& rotationMV, Float3& translationMV,
		VisualizationSystemContainerBase* VSC, const VSettingsView& settingsView,
		bool useColumnMajor = true);

	//! compute rotation and translation as used in ModelView for rendering
	//! uses modelRotation (column-major, transposed HomogeneousTransformation)
	void GetRotationTranslationF(Matrix3DF& rotationMV, Float3& translationMV, bool useColumnMajor = true)
	{ 
		rotationMV = EXUmath::Matrix4DtoMatrix3D(modelRotation);
		if (!useColumnMajor)
		{
			rotationMV.TransposeYourself();
		}
		translationMV = centerPoint;
		translationMV[2] = 0.f; //we do not want z-coordinate due to rotationCenterPoint
	}

	//! compute translation as used in ModelView for rendering
	Float3 GetTranslationF()
	{
		Float3 translationMV;
		translationMV = centerPoint;
		translationMV[2] = 0.f; //we do not want z-coordinate due to rotationCenterPoint
		return translationMV;
	}

	//! compute rotation matrix as used in ModelView for rendering
	//! uses modelRotation (column-major, transposed HomogeneousTransformation)
	Matrix3DF GetRotation3DF(bool useColumnMajor = true)
	{
		if (useColumnMajor)
		{
			return EXUmath::Matrix4DtoMatrix3D(modelRotation);
		}
		else
		{
			return EXUmath::Matrix4DtoMatrix3D(modelRotation.GetTransposed());
		}
	}

	// ++++++++++++++++++++++++++++++++++++++++++
	void ComputeMaxSceneSize(const VisualizationSettings& visSettings, ResizableArray<GraphicsData*>* graphicsDataList)
	{
		float minSceneSize = visSettings.general.minSceneSize;

		Matrix3DF rotationMV = GetRotation3DF(); //without trackMarker, as this should not be done when trackMarker is active?
		Float3 translationMV(0.f); //don't add translation, as the center point is computed from scene dimensions!

		Box3DF box = GraphicsData::ComputeMaxScene(graphicsDataList, rotationMV, translationMV, minSceneSize, 1.f);

		//add basis dimensions to box!
		for (Index viewID = 0; viewID < MAX_VIEWS_GLFW; viewID++)
		{
			const VSettingsView& view = GetSettingsView(viewID, visSettings);
			if (view.scene.drawWorldBasis)
			{
				float a = view.scene.worldBasisSize * 0.0375f;  //additional size for longer arrow
				float b = view.scene.worldBasisSize + a;
				box.Add(-translationMV); //shift due to translation of model;
				box.Add(rotationMV.GetTransposed() * Float3({ b,-a,-a }) - translationMV);
				box.Add(rotationMV.GetTransposed() * Float3({ -a,b,-a }) - translationMV);
				box.Add(rotationMV.GetTransposed() * Float3({ -a,-a,b }) - translationMV);
			}
		}


		maxSceneSize = 2.f * box.Radius();
		if (false) //=> moved to GlfwRenderer::ZoomAll; removed as it translates scene just with computation of max scene
		{
			centerPoint = box.Center(); //why done?
			centerPoint[2] = 0; //do not shift Z-coordinate, as this causes many problems and it is not needed for ZoomAll() !
		}
		boundingBox = box;
	}


	//! ZoomAll compute actions: optionally update graphics
	void ComputeZoomAll(const VisualizationSettings& visSettings)
	{
		float bbFactor = visSettings.general.boundingBoxZoomAllFactor;
		float bbOffset = visSettings.general.boundingBoxZoomAllOffset;

		//center point shifts scene in x/y plane
		centerPoint = Float3({ boundingBox.Center()[0], boundingBox.Center()[1], 0.f }); //do not use Z-coordinate!

		if (!visSettings.general.zoomAllUseBoundingBox)
		{
			//this->zoom = 0.5f * 0.4f * this->maxSceneSize; //original until 2026-01-06 (1.10.52)
			float factor = bbFactor * this->maxSceneSize;
			if (this->maxSceneSize + bbOffset > factor)
			{
				factor = this->maxSceneSize + bbOffset;
			}
			this->zoom = 0.5f * factor / 1.41421f; //this is slightly larger zoom-value than original
		}
		else
		{
			//compute ratio for comparison with bounding box
			Index width = this->currentWindowSize[0];
			Index height = this->currentWindowSize[1];
			float ratio = height != 0 ? width / (float)height : (float)width;

			const Box3DF& box = this->boundingBox;

			float factorX = bbFactor * box.SizeX();
			if (box.SizeX() + bbOffset > factorX)
			{
				factorX = box.SizeX() + bbOffset;
			}

			float factorY = bbFactor * box.SizeY();
			if (box.SizeY() + bbOffset > factorY)
			{
				factorY = box.SizeY() + bbOffset;
			}

			if (factorY * ratio > factorX)
			{   //Y-size is the relevant quantity:
				this->zoom = 0.5f * factorY;
			}
			else
			{   //X-size is the relevant quantity:
				this->zoom = 0.5f * factorX / ratio;
			}
		}
		//std::cout << "ZoomAll zoom=" << this->zoom << "\n";
	}

};

//! A generic visualization class, which is used by Renderer class GLFW and Visualization
//! Used to send signals from Renderer class GLFW, e.g. for update of graphics data
class VisualizationSystemContainerBase
{
public:
	//! OpenGL renderer sends message that graphics shall be updated
	virtual void UpdateGraphicsData() = 0;		//!< renderer reports to simulation to update the graphics data
	virtual void InitializeRenderState(bool validInitialization, bool initializeOnlyIfInvalid = false) = 0;	//!< Initialize RenderState (when renderer is started, etc.) using current visualizationSettings; validInitialization shall be false at instantiation where VisualizationSettings contains default values; initializeOnlyIfInvalid: only initialize if no valid previous initialization was done

	virtual void StopSimulation(bool flag = true) = 0;			//!< renderer reports to simulation that simulation shall be interrupted
	virtual void ForceQuitSimulation(bool flag = true) = 0;		//!< renderer reports that render window is closed and simulation shall be shut down
    virtual void ContinueSimulation() = 0;		//!< renderer reports to simulation that simulation can be continued
    virtual void SwitchPauseSimulation() = 0;		//!< renderer reports to simulation that paused shall be switched
    virtual void UpdateGraphicsDataNow() = 0;	//! renderer signals to update the graphics data, e.g. if settings have changed
	//virtual void SetVisualizationIsRunning(bool flag = true) = 0;	//! renderer signals that visualizationIsRunning flag should be set to "flag"
	virtual void SaveImageFinished(Index viewID) = 0;		//! renderer signals that frame has been grabed and saved
	virtual bool SaveImageRequest(Index viewID) = 0;		//! signal renderer that a frame shall be recorded
	virtual bool SaveImageAsData(Index viewID) = 0;			//! signal renderer that a frame shall be stored in data
	virtual ResizableArray<uint8_t>* ImageData(Index viewID) = 0; //! storage for image data if frame is retrieved as data
	virtual SlimVectorBase<Index, 2>* GetImageDataSize(Index viewID) = 0; //! width and height of image in ImageData()
	virtual bool GetAndResetZoomAllRequest(Index viewID) = 0;//! get zoom all request and reset to false
	virtual void SetZoomAllRequest(Index viewID, bool flag) = 0; //!< request ZoomAll
	virtual void SetComputeMaxSceneRequest(bool flag) = 0;
	virtual bool GetComputeMaxSceneRequest() = 0;

    //get marker position and orientation
	virtual void GetMarkerPositionOrientation(Index markerNumber, Index mbsNumber, Vector3D& position, Matrix3D& orientation, bool& hasPosition, bool& hasOrientation) = 0;
    //get sensor data list (if available); return true if further sensors available
    virtual bool GetSensorsPositionsVectorsLists(Index mbsNumber, Index positionSensorIndex, Index vectorSensorIndex, Index triadSensorIndex,
        Vector3DList& sensorTracePositions, Vector3DList& sensorTraceVectors, Matrix3DList& sensorTraceTriads, Vector sensorTraceValues, 
        const VSettingsTraces& traces) = 0;

	virtual std::vector<VSettingsMaterial>* GetGraphicsMaterialList() = 0;
	virtual void CopyMaterialsFromVisualizationSettings() = 0;

	//! added to directly access this data from GlfwClient
	virtual ResizableArray<GraphicsData*>& GetGraphicsDataList() = 0;
	virtual VisualizationSettings& GetVisualizationSettings() = 0;
	virtual RenderState& GetRenderState(Index viewID) = 0;


	virtual std::string GetComputationMessage(bool solverInformation = true,
		bool solutionInformation = true, bool solverTime = true) = 0; //! any multi-line text message from computation to be shown in renderer (e.g. time, solver, ...)
	virtual MainSystem* GetMainSystemBacklink(Index iSystem) = 0; //! REMOVE: get backlink of ith main system (0 if not existing), temporary for selection
	virtual Index NumberOFMainSystemsBacklink() const = 0; //! REMOVE: get backlink to number of main systems, temporary for selection
	virtual bool DoSingleIdleOperation() = 0; //!< this function does any idle operations (execute some python commands) and returns false if stop flag in the render engine, otherwise true;

	virtual ~VisualizationSystemContainerBase() {} //added for correct deletion of derived classes

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//HELPER FUNCTIONS

	//! compute RGBA-color for given value within a min/max range
	virtual Float4 ColorBarColor(float minVal, float maxVal, float value, float alpha = 1.f) = 0;

	//!transform pixel coordinates (from bottom/left, [0..width-1, 0..height-1]) into GL_MODELVIEW vertex coordinates
	//!works with 	glMatrixMode(GL_MODELVIEW);glLoadIdentity();
	Float2 PixelsToModelViewCoordinates2D(float x, float y, Index widthPixels, Index heightPixels, float zoom)
	{
		if (widthPixels == 0 || heightPixels == 0) { return Float2({ 0, 0 }); }
		float ratio = widthPixels / (float)heightPixels + 2e-7f; // for round off errors

		//x,y in pixels; origin is bottom/left
		float vx = zoom * ratio * (-1.f + 2.f * x / widthPixels);
		float vy = zoom * (-1.f + 2.f * y / heightPixels);
		return Float2({ vx, vy });
	}
	//!transform pixel coordinates (from bottom/left, [0..width-1, 0..height-1]) into GL_MODELVIEW vertex coordinates
	//!works with 	glMatrixMode(GL_MODELVIEW);glLoadIdentity();
	Float3 PixelsToModelViewCoordinates(float x, float y, Index widthPixels, Index heightPixels, float zoom, float offsetZ = 0)
	{
		Float2 p = PixelsToModelViewCoordinates2D(x, y, widthPixels, heightPixels, zoom);
		return Float3({ p[0], p[1], offsetZ });
	}
	//!transform GL_MODELVIEW vertex coordinates into pixel coordinates (from bottom/left, [0..width-1, 0..height-1]) 
	//!works with 	glMatrixMode(GL_MODELVIEW);glLoadIdentity();
	Float2 ModelViewCoordinatesToPixels(const Float2& pos2D, Index widthPixels, Index heightPixels, float zoom)
	{
		if (widthPixels == 0 || heightPixels == 0 || zoom == 0) { return Float2({ 0, 0 }); }
		float ratio = widthPixels / (float)heightPixels + 2e-7f; // for round off errors

		float vx = pos2D[0];
		float vy = pos2D[1];

		//x,y in pixels; origin is bottom/left
		float x = widthPixels * (vx / (zoom * ratio) + 1.f) / 2.f;
		float y = heightPixels * (vy / zoom + 1.f) / 2.f;
		return Float2({ x, y });
	}

	//!transform pixel coordinates (x, y) (from bottom/left, [0..width-1, 0..height-1]) into OpenGL (3D) coordinates
	//!in a certain ortho-projection (x-y plane, x-z plane, etc.)
	Vector2D PixelsToOpenGLCoordinates2D(double x, double y, const RenderState& state)
	{
		double height = (double)state.currentWindowSize[1];
		if (height == 0.) { height = 1.; }//for safety
		double factor = 2. * state.zoom / height;

		return factor * Vector2D({ x - 0.5 * state.currentWindowSize[0],-(y - 0.5 * state.currentWindowSize[1]) }) 
			   + Vector2D({ (double)state.centerPoint[0], (double)state.centerPoint[1] });
	}

	//! check if view is axis-aligned, e.g., x-y, x-z, z-y, etc.
	bool IsAxisAlignedView(const RenderState& state)
	{
		const Matrix4DF& A = state.modelRotation;
		for (int i = 0; i < 3; ++i) {
			//sum the absolute values of the components of the basis vector
			float rowSum = fabs(A(0, i)) + fabs(A(1, i)) + fabs(A(2, i));

			if (fabs(rowSum - 1.0f) > 1e-5) { return false; }
		}
		return true;
	}
};

inline void RenderState::GetRotationTranslationFWithMarker(Matrix3DF& rotationMV, Float3& translationMV,
	VisualizationSystemContainerBase* VSC, const VSettingsView& settingsView,
	bool useColumnMajor)
{
	GetRotationTranslationF(rotationMV, translationMV); //? include useColumnMajor ?

	if (!settingsView.camera.modelCentricView)
	{
		translationMV += rotationMV * settingsView.camera.cameraPosition; //in marker-frame; without camera-rotation...
	}

	//update center point if tracked by marker
	if (settingsView.camera.trackMarker != -1)
	{
		Vector3D markerPosition;
		Matrix3D markerOrientation;
		bool hasPosition;
		bool hasOrientation;
		VSC->GetMarkerPositionOrientation(settingsView.camera.trackMarker,
			settingsView.camera.trackMarkerMbsNumber, markerPosition, markerOrientation, hasPosition, hasOrientation);

		if (hasOrientation && settingsView.camera.trackMarkerOrientation.SumAbs() == 3.f)
		{ //track orientation as well!
			Matrix3DF markerOrientation3DF;
			markerOrientation3DF.CopyFrom(markerOrientation);
			//superimposed initial rotation, same for model and camera-centric:
			rotationMV = markerOrientation3DF * rotationMV;
		}

		if (hasPosition)
		{  //track only selected position coordinates:
			markerPosition[0] *= (Real)settingsView.camera.trackMarkerPosition[0];
			markerPosition[1] *= (Real)settingsView.camera.trackMarkerPosition[1];
			markerPosition[2] *= (Real)settingsView.camera.trackMarkerPosition[2];

			Float3 markerPosition3DF;
			markerPosition3DF.CopyFrom(markerPosition);
			//convert model-centric marker to camera-frame:
			translationMV += markerPosition3DF * rotationMV;
		}
	}

	if (!useColumnMajor)
	{
		rotationMV.TransposeYourself();
	}

}


#endif
