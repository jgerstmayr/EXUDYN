/** ***********************************************************************************************
* @class        GlfwClient
* @brief        Hub to glfw class for 3D visualization using OpenGL
*
* @author       Gerstmayr Johannes
* @date         2019-05-24 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#ifndef GLFWCLIENT__H
#define GLFWCLIENT__H

#include "Graphics/GlfwClientBase.h" //part which does not require GLFW and OpenGL

#include "Main/rendererPythonInterface.h"

#ifdef USE_GLFW_GRAPHICS

#define GLFW_INCLUDE_GLEXT
//#define GL_GLEXT_PROTOTYPES OpenGL3.2
#include <GLFW/glfw3.h>

//DELETE: #define USE_TEXTURED_BITMAP_FONTS //!< textured based fonts with glLists are standard with no alternative
#define NUMBER_OF_TEXTUREFONT_LISTS 2 //1 is standard; 2 are created to switch between transparent and font with background

#include "Graphics/GlfwClientText.h" //link to external library; include only if copyright is appropriate
#include "Graphics/GlfwClientBitmapText.h"

//currently done in preprocessor flags; check that openvr_api.dll is included in exudynCPP.pyd directory
//#define __EXUDYN_USE_OPENVR //done with preprocessor flags in VisualStudio or in setup.py
//#if !defined(__EXUDYN__APPLE__) && !defined(__EXUDYN__LINUX__ARM__) //would not work with APPLE
class Raytracer;

//! a class that contains all information on window, RenderState, etc. used for a single window/view in GLFWClient
class RenderView
{
private:
	GLFWwindow* window;			//!< the window pointer after creation; otherwise nullptr
	RenderState* renderState;   //!< the link to the renderState in VSC; otherwise nullptr
	bool windowShouldBeCreated;	//!< true, if window should be opened at next possible event

public:
	void Reset() { window = nullptr; renderState = nullptr; windowShouldBeCreated = false; }
	//! check if window is active; note: in this case, state is also active!
	bool IsValidWindow() const { return window != nullptr; }
	//! checkif state is active; window might not be active
	bool IsValidState() const { return renderState != nullptr; }
	//! checkif state is active; window might not be active
	bool GetWindowShouldBeCreated() const { return windowShouldBeCreated; }
	void SetWindowShouldBeCreated(bool flag) { windowShouldBeCreated = flag; }

	const GLFWwindow* GetWindow(bool raiseError = true) const
	{
		if (raiseError && window == nullptr) { CHECKandTHROWstring("RenderView::GetWindow const: not yet set; initialize renderer first (renderer.Start())"); }
		return window;
	}
	GLFWwindow* GetWindow(bool raiseError = true)
	{
		if (raiseError && window == nullptr) { CHECKandTHROWstring("RenderView::GetWindow: not yet set; initialize renderer first (renderer.Start())"); }
		return window;
	}
	void SetWindow(GLFWwindow* windowInit)
	{ 
		window = windowInit;
	}

	const RenderState* GetRenderState(bool raiseError = true) const
	{
		if (raiseError && renderState == nullptr) { CHECKandTHROWstring("RenderView::RenderState const: not yet set; initialize renderer first (renderer.Start())"); }
		return renderState;
	}
	RenderState* GetRenderState(bool raiseError = true)
	{
		if (raiseError && renderState == nullptr) { CHECKandTHROWstring("RenderView::RenderState: not yet set; initialize renderer first (renderer.Start())"); }
		return renderState;
	}
	void SetRenderState(RenderState* renderStateInit)
	{ 
		renderState = renderStateInit;
	}
};

class RenderViewList
{
private:
	std::array<RenderView, MAX_VIEWS_GLFW> renderViewArray; //!< link to GLFWwindow, RenderState and more
public:
	RenderViewList()
	{
		for (RenderView& rv : renderViewArray) { rv.Reset(); }
	}

	Index NumberOfViews() const { return (Index)renderViewArray.size(); }

	void Reset() { for (RenderView& rv : renderViewArray) { rv.Reset(); } }

	const RenderView& GetRenderView(Index viewID) const { return renderViewArray[viewID]; }
	RenderView& GetRenderView(Index viewID) { return renderViewArray[viewID]; }

	//! check if window is active; note: in this case, state is also active!
	bool IsValidWindow(Index viewID) const { return renderViewArray[viewID].IsValidWindow(); }
	//! check for any valid window (e.g. to be closed)
	bool HasValidWindows() const
	{
		for (const RenderView& rv : renderViewArray) { if (rv.IsValidWindow()) { return true; }; }
		return false;
	}

	//! checkif state is active; window might not be active
	bool IsValidState(Index viewID) const { return renderViewArray[viewID].IsValidState(); }

	//! check if view is enabled by user => additional operations to be done for view (max scene size, etc.)
	bool IsEnabledView(Index viewID) const 
	{ 
		return renderViewArray[viewID].IsValidState() && renderViewArray[viewID].GetRenderState()->viewEnabled;
	}

	//! check if view is enabled by user => additional operations to be done for view (max scene size, etc.)
	void SetEnabledView(Index viewID, bool enable)
	{
		CHECKandTHROW(renderViewArray[viewID].IsValidState(), "RenderViewList::SetEnabledView: renderstate invalid!");
		renderViewArray[viewID].GetRenderState()->viewEnabled = enable;
	}

	//! check if view is currenly opened and enabled by user 
	//! => additional operations to be done for view (max scene size, etc.)
	//! may be different from IsValidWindow in future?
	bool IsWindowOpen(Index viewID) const
	{
		return (renderViewArray[viewID].IsValidWindow() //this is synchronized with RenderState viewEnabled & windowOpen
			//renderViewArray[viewID].GetRenderState()->viewEnabled &&
			//renderViewArray[viewID].GetRenderState()->windowOpen
			);
	}

	bool GetWindowShouldBeCreated(Index viewID) const 
	{ 
		bool flag = renderViewArray[viewID].GetWindowShouldBeCreated();
		CHECKandTHROW(!(flag && IsWindowOpen(viewID)), "RenderViewList::GetWindowShouldBeCreated: inconsistent flags");
		CHECKandTHROW(viewID != 0, "RenderViewList::GetWindowShouldBeCreated: not possible for main view 0");
		return flag;
	}

	void SetWindowShouldBeCreated(Index viewID, bool flag) 
	{ 
		CHECKandTHROW(viewID != 0, "RenderViewList::SetWindowShouldBeCreated: not possible for main view 0");
		renderViewArray[viewID].SetWindowShouldBeCreated(flag);
	}

	const RenderState* State(Index viewID, bool raiseError = true) const
	{
		CHECKandTHROW(viewID < MAX_VIEWS_GLFW, "GetRenderState const: illegal viewID");
		return renderViewArray[viewID].GetRenderState(raiseError);
	}

	RenderState* State(Index viewID, bool raiseError = true)
	{
		CHECKandTHROW(viewID < MAX_VIEWS_GLFW, "GetRenderState: illegal viewID");
		return renderViewArray[viewID].GetRenderState(raiseError);
	}

	void SetRenderState(Index viewID, RenderState* renderState)
	{
		CHECKandTHROW(viewID < MAX_VIEWS_GLFW, "SetRenderState: illegal viewID");
		renderViewArray[viewID].SetRenderState(renderState);
	}

	//get viewID from window; this is not very efficient, but we have only 1-4 (future maybe 8) windows ...
	Index GetViewID(GLFWwindow* window)
	{
		for (Index i=0; i < (Index)renderViewArray.size(); i++)
		{ 
			if (renderViewArray[i].GetWindow() == window)
			{
				return i;
			}
		}
		//at this point, we cannot do more! Python (PyError) would crash at this point:
		CHECKandTHROWstring("RenderViewList::GetViewID: received unknown window");
		return 0;
	}

	const GLFWwindow* GetWindow(Index viewID, bool raiseError = true) const 
	{ 
		return renderViewArray[viewID].GetWindow(raiseError); 
	}
	GLFWwindow* GetWindow(Index viewID, bool raiseError = true) 
	{ 
		return renderViewArray[viewID].GetWindow(raiseError); 
	}
	//! set window
	void SetWindow(Index viewID, GLFWwindow* window) 
	{ 
		renderViewArray[viewID].SetWindow(window); 
	}
	bool WindowHasFocus(Index viewID) //has to be checked for joystick!!!
	{
		if (IsValidWindow(viewID))
		{
			return glfwGetWindowAttrib(GetWindow(viewID), GLFW_FOCUSED);
		}
		return false;
	}

};


//! this is the rendering module for displaying 3D model data
class GlfwRenderer
{
private:
	//static RenderState state;
	static bool rendererActive;			//!< signal that shows that renderer is active
	static bool stopRenderer;			//!< signal that shows that renderer should quit
	static bool useMultiThreadedRendering;		//!< according to visualizationSettings.general; always false for MACOS (__EXUDYN__APPLE__)
    static Real lastGraphicsUpdate;		//!< time of last graphics update
    static Real lastTryCloseWindow;		//!< time of last trial to close window (for security closing)
    static Real lastEventUpdate;		//!< time of last event polling
    static Real rendererStartTime;      //!< time when renderer was started; used to check if quit requires security question
	static bool callBackRefreshSignal;			//!< for single threaded applications, react if callback is sent=> update graphics immediately
	static Index rendererTasksCount;		//!< this counts is increased as soon as DoRendererTasks() has finished, and is reset on startup; can be used to check if first frame has been drawn

	static Real lastTimeAutoRotate;     //!< time when last autorotation was applied (EXUstd::GetTimeInSeconds)
	static bool doAutoRotate;			//!< internal flag, knowing if autorotation was already activated or not

	static constexpr Index mainViewID = 0;		//!< main view ID, used for special operations only applicable to main view (or during initialization)
	static RenderViewList renderViews; //!< link to GLFWwindow, RenderState and more

	static RenderStateMachine stateMachine; //!< all variables (mouse, keyboard, ...) used for state machine (zoom, zoom-view, move, ...)
	static std::thread rendererThread;	//!< std::thread variable for rendererThread
	static Index rendererError;			//!< 0 ... no error, 1 ... glfwInit() failed, 2 ... glfwCreateWindow failed, 3 ... other error
	static Index verboseRenderer;        //!< initialized in StartRenderer(bool verbose): output helpful information
	//static Index firstRun; //zoom all in first run
	static std::atomic_flag renderFunctionRunning;  //!< semaphore to check if Render(...)  function is currently running (prevent from calling twice)
	static std::atomic_flag showMessageSemaphore;   //!< semaphore to prevent calling ShowMessage twice


	//+++++++++++++++++++++++++++++++++++++++++
	static BitmapFont bitmapFont;				//!< bitmap font for regular texts and for textured fonts, initialized upon start of renderer
	static float fontScale;						//!< monitor scaling factor from windows, to scale fonts

	//+++ for textures with glLists +++:
    static bool depthMask;                      //!< used for font drawing; current state of depth mask

public:
	static GLuint textureNumberRGBbitmap[256*NUMBER_OF_TEXTUREFONT_LISTS];	//!< store texture numbers for every character of bitmap font
private:
    static GLuint bitmapFontListBase;			//!< starting index for GLlists for font bitmap textured quads
	static ResizableArray<GLubyte> charBuffer;	//!< buffer for converstion of UTF8 into internal unicode-like format

	static GLuint spheresListBase;				//!< starting index for GLlists for spheres
	static constexpr Index maxSpheresLists = 8; //!< max. number of GLlists for spheres (with resolution 2,4,8,16, etc.
	static GraphicsData graphicsDataStatic;		//!< static GraphicsData objects (info, Exudyn, etc.)
	//+++++++++++++++++++++++++++++++++++++++++
	//link to GraphicsData and Settings:
	static ResizableArray<GraphicsData*>* graphicsDataList;					//!< link to graphics data; only works for one MainSystem, but could also be realized for several systems
	static VisualizationSettings* visSettings;  //!< link to visualization settings
	static VisualizationSystemContainerBase* basicVisualizationSystemContainer;
    //+++++++++++++++++++++++++++++++++++++++++
    //for sensor traces:
    static Vector3DList sensorTracePositions;
    static Vector3DList sensorTraceVectors;		//!< synchronized with triads
    static Matrix3DList sensorTraceTriads;		//!< synchronized with vectors
    static Vector sensorTraceValues; //temporary storage for current sensor data
    //+++++++++++++++++++++++++++++++++++++++++

	//static Raytracer raytracer;
public:
	GlfwRenderer();
	~GlfwRenderer() 
	{	
		//glfwTerminate(); //move to destructor
	};

	//! Initializes and starts the Renderer in a separate thread;
	//  Returns false, if problems with glfw library or windows creation, otherwise true; 
	//  @todo test with apple and linux
	static bool StartRenderer(Index verbose = 0);

	//! stop the renderer engine and its thread; @todo StopRenderer currently also stops also main thread (python)
	static void StopRenderer();

	//! Opens window for additional viewID=1,2, ...; write window into renderViews[viewID]; return false in case of error
	static bool CreateViewWindow(Index viewID);

	//! Close window for viewID=1,2, ...
	static void CloseViewWindow(Index viewID);


    //! return copy of renderState object
    static RenderState State(Index viewID) { return *renderViews.State(viewID); }

	//static const RenderState* GetRenderStatePtr() { return state; }

	//! return copy of renderState object
	static RenderViewList* GetRenderViews() { return &renderViews; }

	//! get index
	static Index GetRendererTasksCount() { return rendererTasksCount; }

    //! return renderState object
    static VisualizationSettings* GetVisualizationSettings() { return visSettings; }
    //static const VisualizationSettings& GetVisualizationSettings() const { return *visSettings; }

	static VisualizationSystemContainerBase* GetVisualizationSystemContainer() { return basicVisualizationSystemContainer; }

	//! reset some state machines, e.g., left mouse click, item select, etc.
	static void ResetStateMachine();

	static bool IsGlfwInitAndRendererActive()
	{
		//only check for main window 0
		if (renderViews.GetWindow(mainViewID, false) && rendererActive) { return true; }
		else { return false; }
	}

	//! return verbose flag for outside; can be 0, 1 or larger
	static Index Verbose() { return verboseRenderer; }

	//! Links the Renderer to a specific GraphicsData/settings; 
	//! Only one data linked at one time
	//! Returns true on success and false, if data is already linked (==> call DetachVisualizationSystem first)
	static bool LinkVisualizationSystem(VisualizationSystemContainerBase* basicVisualizationSystemContainerInit)
	{
		if (basicVisualizationSystemContainer == nullptr)
		{
			basicVisualizationSystemContainer = basicVisualizationSystemContainerInit;

			graphicsDataList = &basicVisualizationSystemContainer->GetGraphicsDataList();
			visSettings = &basicVisualizationSystemContainer->GetVisualizationSettings();
			
			//link renderstates, they are kept as long as VSC is linked!
			for (Index viewID = 0; viewID < renderViews.NumberOfViews(); viewID++)
			{
				renderViews.SetRenderState(viewID, &basicVisualizationSystemContainer->GetRenderState(viewID));
			}
			basicVisualizationSystemContainer->SetComputeMaxSceneRequest(true); //computes maxSceneCoordinates for perspective and shadow; ok if done here, because needed for startup

			return true;
		}
		else { return false; }
	}

	//! Detach the GraphicsData/settings; enables the visualization of different MainSystems; return true on success
	static bool DetachVisualizationSystem(VisualizationSystemContainerBase* detachingVisualizationSystemContainer)
	{
		//only detach, if detachingVisualizationSystemContainer is still linked to GLFWrenderer
		if (detachingVisualizationSystemContainer == nullptr || basicVisualizationSystemContainer == detachingVisualizationSystemContainer)
		{
			StopRenderer();

			if (graphicsDataList == nullptr) { return false; }//this just shows that no system was linked yet
			else
			{
				graphicsDataList = nullptr;
				visSettings = nullptr;

				renderViews.Reset();

				basicVisualizationSystemContainer = nullptr;

				return true;
			}
		}
		return false;
	}

	static void UpdateGraphicsDataNow()
	{
		if (basicVisualizationSystemContainer)
		{
			basicVisualizationSystemContainer->UpdateGraphicsDataNow();
		}
	}

	//! write dictionary for selected item; return true if success; MAY ONLY BE CALLED FROM PYTHON THREAD!!!
	static bool PySetRendererSelectionDict(Index itemID);

	//! retrieve basic item information from MainSystemBacklink; return true if success; renderer thread safe (no Python calls)
	static bool GetItemInformation(Index itemID, STDstring& itemTypeName, STDstring& itemName);// , STDstring& itemInfo);

	//! print delayed via safe communication with main thread
	static void PrintDelayed(const STDstring& str, bool lineFeed = true, bool flush = false) 
	{
		if (useMultiThreadedRendering)
		{
			if (lineFeed)
			{
				//PyQueueExecutableString("print('" + str + "')\n");
				outputBuffer.WriteVisualization(str+'\n'); 
			}
			else
			{
				outputBuffer.WriteVisualization(str); 
			}
		}
		else
		{
			pout << str;
			if (lineFeed) { pout << "\n"; }
			if (flush) { outputBuffer.overflowFlush(0, true, false); }
		}
	};

	static STDstring OnOffFromBool(bool flag) { if (flag) { return "on"; } else { return "off"; } }

	//! add status message, e.g., if button is pressed
	static void ShowMessage(const STDstring& str, Real timeout = 0);

	//! run renderer idle for certain amount of time; use this for single-threaded, interactive animations; waitSeconds==-1 waits forever
	static void DoRendererIdleTasks(Real waitSeconds, bool graphicsUpdateAndRender=false);

	//! check if separate thread used:
	static bool UseMultiThreadedRendering() { return useMultiThreadedRendering; }

	//! access keypress user function from current SC; only available if SC linked:
	static std::function<bool(int, int, int)> GetKeyPressUserFunction() 
	{ 
		if (IsGlfwInitAndRendererActive() && visSettings != nullptr)
		{
			return visSettings->interactive.keyPressUserFunction;
		}
		else
		{
			return 0;
		}
	}

	//! set projection matrix from outside GlfwClient, used for OpenVR; this needs to be set before rendering objects!
	//! projectionInfo used to switch between different modes
	static void SetProjectionMatrix(const Matrix4DF& p, Index projectionInfo = 0, Index viewID=0) 
	{ 
		RenderState* state = renderViews.State(viewID);

		state->projectionMatrix = p; 
		state->projectionInfo = projectionInfo;
	};

	//! get current glfw window size
	static void GetWindowSize(GLFWwindow* window, int& width, int& height)
	{
		glfwGetFramebufferSize(window, &width, &height);
	}

	//! unified function to get screen ratio and zoom
	static void SetRenderStateScreenSize(Index viewID, int screenWidth, int screenHeight);

	//! Render 3D scence function called from Render(), containing 3D model without additional text, etc.; projection is supplied
	static void Render3Dobjects(Index viewID, int screenWidth, int screenHeight, float screenRatio, float zoom);

	//! draw sphere or simple representation for sphere
	static void DrawSphere(const GLSphere& item, bool highlight, Index highlightID, const Float4& otherColor2, const Float4& highlightColor2, bool showFaces);

	static const ResizableArray<GraphicsData*>* GetGraphicsDataList() { return graphicsDataList; }

	//DELETE: static GLFWwindow* GetGlfwWindow() { return window; }

private: //to be called internally only!
	static void error_callback(int error, const char* description)
	{
		std::cout << description << "\n";
	}

	//! lambda function to call something for all active windows
	//! ForEachOpenWindow([&](Index viewID) {this->UpdateCamera(viewID);});
	template<typename Func>
	static void ForEachWindowOpen(Func&& func) 
	{
		// Assuming renderViews is a static member of GlfwRenderer
		for (Index i = 0; i < renderViews.NumberOfViews(); ++i) 
		{
			if (renderViews.IsWindowOpen(i)) 
			{
				func(i);
			}
		}
	}

	//! lambda function to call something for all active windows
	//! ForEachEnabledView([&](Index viewID) {this->UpdateCamera(viewID);});
	template<typename Func>
	static void ForEachEnabledView(Func&& func)
	{
		// Assuming renderViews is a static member of GlfwRenderer
		for (Index i = 0; i < renderViews.NumberOfViews(); ++i)
		{
			if (renderViews.IsEnabledView(i))
			{
				func(i);
			}
		}
	}

	//! GLFW callback functions:
	static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
	static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
	static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
	static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
	static void window_close_callback(GLFWwindow* window);
	static void window_content_scale_callback(GLFWwindow* window, float xscale, float yscale);

	//! compute unified content scaling based on values provided by GLFW
	static void SetContentScaling(Index viewID, float xScale, float yScale);
	static float GetFontScaling(Index viewID);
    static void SetFontScaling(Index viewID, float scaling);

	//! return true, if joystick available and updated values are available; if joystickNumber==invalidIndex, chose a joystick; 
	//! if joystickNumber!=invalidIndex, it uses the fixed joystick until end of Renderer
	static bool GetJoystickValues(Vector3D& position, Vector3D& rotation, Index& joystickNumber);
	
	//! read joystick values; apply only to window which has focus; if changed, send refresh signal for graphics
	static void ProcessJoystick();
	//! perform autorotation if enabled
	static void DoAutoRotation(Index viewID);

	//! if callback function like mousemove is called, immediately refresh graphics independently of graphicsUpdateInterval
	static void SetCallBackRefreshSignal(bool flag = true) { callBackRefreshSignal = flag; }
	static bool GetCallBackRefreshSignal() { return callBackRefreshSignal; }
	
	//! zoom in to mouse position, used to render that area lateron (replacement for gluPickMatrix(...)
	static void SetViewOnMouseCursor(GLdouble x, GLdouble y, GLdouble deltax, GLdouble deltay, GLint viewport[4]);
	
	//! function to evaluate selection of items
	static void MouseSelectOpenGL(Index viewID, Index mouseX, Index mouseY, Index& itemID, float& zDepth);

	//! function to evaluate selection of items, return true, if item selected
	static bool MouseSelect(Index viewID, Index mouseX, Index mouseY, Index& itemID);

	//! GlfwInit and glfw->CreateWindow() calls; returns false, if functions fail
	static void InitCreateWindow();

	//! loop which checks for keyboard/mouse input; check for visualization updates (new data, window size changed, zoom, mouse move, etc.) and calls Render()
	static void RunLoop();

	//! tasks which are regularly called by RunLoop(), used if no separate thread used in GLFW; use wait in seconds to do this 
	//! for single-threaded renderer, an immediate rendering can be requested
	static void DoRendererTasks(bool graphicsUpdateAndRender = false);

	//! tasks which are done if renderer is shut down
	static void FinishRunLoop();

	//! Render function called for every update of OpenGl window
	static void Render(GLFWwindow* window); //GLFWwindow* needed in argument, because of glfwSetWindowRefreshCallback

	//! set background for scene (gradient background)
	static void AddGradientBackground(float zoom, float ratio);
	
	//! load GL_PROJECTION and set according to zoom, perspective, etc.; one function for render and mouse select
	static void SetProjection(Index viewID, int width, int height, float ratio, float zoom);

	//! set model view rotation and translation, unified for Render and mouse select
	static void SetModelRotationTranslation(Index viewID);
	
	//! check if frame shall be grabed and saved to file using visualization options
	static void SaveImage(Index viewID);

	//! save scene to a file with filename
	static void SaveSceneToFile(Index viewID, const STDstring& filename);
	
    //! Render particulary the graphics data of multibody system; selectionMode==true adds names
    static void RenderGraphicsData(Index viewID, bool selectionMode = false);

       //! render sensor traces if activated and available
    static void RenderSensorTraces(Index viewID);

    //! Render particulary the text of multibody system; selectionMode==true adds names
    static void RenderGraphicsDataText(Index viewID, GraphicsData* data, Index lastItemID, bool highlight, Index highlightID, Float4 highlightColor2, Float4 otherColor2, bool selectionMode=false);

	//! Render triangles with stencil shadow method (slow, but accurate and should be sufficient for some animations)
	static void DrawTrianglesWithShadow(Index viewID, GraphicsData* data);
	
	//! Zoom all graphics objects (for current configuration)
	static void ZoomAll(Index viewID, bool updateGraphicsData=true, bool computeMaxScene=true, bool render=true);

    //! Set all light functions for openGL
    static void SetGLLights(Index viewID);

    //! Set depth mask and track its state; used to track current state for font drawing
    static void SetGLdepthMask(bool flag) { depthMask = flag; glDepthMask(flag); };

    //! get internally stored state of depth mask
    static bool GetGLdepthMask() { return depthMask; };

	//! check whether a point is clipped (not visible due to clipping plane
	//! function assumes that valid index is already checked before!
	static bool IsClipped(Index viewID, const Float3& p)
	{
		return (p * GetSettingsView(viewID, *visSettings).camera.clippingPlaneNormal) > GetSettingsView(viewID, *visSettings).camera.clippingPlaneDistance;
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//FONTS
	//! initialize bitmap for bitmap font (loaded from characterBitmap.h
	static void InitFontBitmap(guint fontSize);// , guint fontSizeSmall, guint fontSizeLarge, guint fontSizeHuge);

	//! initialize some GLlists, e.g., for spheres
	static void InitGLlists();

	//! draw a 0-terminated text string with fontSize, including monitor scaling factor; (for line-characters: size=1: height=1; width=0.5 for one character; distance = 0.25)
	//! switches to strings drawn by textures (default) or lines
	//! offset is given relative to character width, height and scene max size; offset is not rotated
	static void DrawString(Index viewID, const char* text, float fontSizeScaled, const Float3& p, const Float3& offset, Float4 color, bool transparent = true);

	//! draw string with scalable bitmap fonts, using textures
	static void DrawStringWithTextures(const char* text, float fontSizeScaled, const Float3& p, const Float4& color,
		BitmapFont& font, ResizableArray<GLubyte>& charBuffer, GLuint listBase, bool transparent = true);

	//! create glTexImage2D objects for font characters, stored in textureNumberRGBbitmap
	static void CreateFontTextures();

	//! create glLists for texture with textureNumber
	static void CreateTexturedQuadsLists(GLuint& listBase, GLuint* textureNumber,
		guint nCharacters, guint wCharacter8, guint wCharacter, guint hCharacter, bool itemTags = false);

	//draw string with GLlists, previously created by CreateTexturedQuadsLists
	static void DrawStringWithGLlistTextures(const Float3& p, float fontSizeScaled, GLuint listBase, GLubyte *string, guint stringLen, GLuint listOffset);

	//! OpenGL testing functions for destructor
	static void DeleteFonts();
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! transform vertex 'in' with modelViewMatrix to only Z-coordinate
	static float TransformVertexZ(const Float3& in, const Matrix4DF& modelViewMatrix)
	{
		//state->modelRotation.GetDataPointer()
		const float* modelview = modelViewMatrix.GetDataPointer();
		return modelview[2] * in[0] + modelview[6] * in[1] + modelview[10] * in[2] + modelview[14];
	}


	static void ComputeSortedTriangleDepthIndices(const ResizableArray<GLTriangle>& triangles, ResizableArray<Index>& trigIndices, 
		const Matrix4DF& modelViewMatrix)
	{
		Index nt = triangles.NumberOfItems();
		ResizableArray<float> depths(nt);
		trigIndices.SetMaxNumberOfItems(nt);
		trigIndices.SetNumberOfItems(0);
		Index cnt = 0;
		for (const GLTriangle& trig : triangles)
		{
			float depth = 0;
			for (Index i = 0; i < 3; i++)
			{
				depth += TransformVertexZ_CM(trig.points[i], modelViewMatrix);
			}
			depth *= 1.f / 3.f;
			trigIndices.Append(cnt++);
			depths.Append(depth);
		}
		EXUstd::QuickSortIndexed(trigIndices, depths, false);
		//pout << "indices=" << trigIndices << ", depths=" << depths << "\n" << std::flush;
	}

public:	

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! transform vertex 'in' with modelViewMatrix
	static void TransformVertex(const Float3& in, const Matrix4DF& modelViewMatrix, Float3& out)
	{
		//state->modelRotation.GetDataPointer()
		const float* modelview = modelViewMatrix.GetDataPointer();
		float x = in[0];
		float y = in[1];
		float z = in[2];
		out[0] = modelview[0] * x + modelview[4] * y + modelview[8] * z + modelview[12];
		out[1] = modelview[1] * x + modelview[5] * y + modelview[9] * z + modelview[13];
		out[2] = modelview[2] * x + modelview[6] * y + modelview[10] * z + modelview[14];
	}
	//! template function for drawing triangles and triangle outlines
	template<bool highlight, bool transparent, bool edgeMode>
	static void DrawTriangles(const ResizableArray<GLTriangle>& triangles,
		const Float4& highlightColor,
		const Float4& otherColor,
		bool selectionMode,
		Index& lastItemID,
		Index highlightID,
		bool useClipping,
		GLFWwindow* window)
	{
		Index viewID = renderViews.GetViewID(window);
		const VSettingsView& settingsView = GetSettingsView(viewID, *visSettings);

		if (edgeMode) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		}
		const bool sortTriangles = visSettings->openGL.advanced.depthSorting;
		ResizableArray<Index> trigIndices;
		if (sortTriangles)
		{
			ComputeSortedTriangleDepthIndices(triangles, trigIndices, renderViews.State(viewID)->modelRotation);
		}
		for (Index k=0; k < triangles.NumberOfItems(); k++)
		{
			//pout << "trigIndices[k]=" << trigIndices[k] << ", k=" << k << "\n" << std::flush;
			const GLTriangle& trig = sortTriangles ? triangles[trigIndices[k]] : triangles[k];

			if (selectionMode && trig.itemID != lastItemID) {
				glLoadName(trig.itemID);
				lastItemID = trig.itemID;
			}

			bool showFace = (settingsView.scene.showFaces && !trig.isFiniteElement) ||
				(settingsView.scene.showMeshFaces && trig.isFiniteElement);
			bool showEdge = (settingsView.scene.showFaceEdges && !trig.isFiniteElement) ||
				(settingsView.scene.showMeshEdges && trig.isFiniteElement);

			if (!(edgeMode ? showEdge : showFace)) { continue; }

			if (useClipping && IsClipped(viewID, trig.points[0]) && IsClipped(viewID, trig.points[1]) && IsClipped(viewID, trig.points[2])) {
				{ continue; }
			}

			if constexpr (edgeMode) {
				if constexpr (!highlight) {
					Float4 edgeColor = visSettings->openGL.faceEdgesColor;
					glColor4f(edgeColor[0], edgeColor[1], edgeColor[2], edgeColor[3]);
				}
				else {
					glColor4fv((trig.itemID != highlightID ? otherColor : highlightColor).GetDataPointer());
				}
			}

			glBegin(GL_TRIANGLES);
			for (Index i = 0; i < 3; i++) {
				if constexpr (highlight && !edgeMode) {
					glColor4fv((trig.itemID != highlightID ? otherColor : highlightColor).GetDataPointer());
				}
				else if constexpr (!highlight && !transparent && !edgeMode) {
					glColor4fv(trig.colors[i].GetDataPointer());
				}
				else if constexpr (transparent) {
					Float4 col = trig.colors[i];
					if (col[3] > visSettings->openGL.faceTransparencyGlobal) { col[3] = visSettings->openGL.faceTransparencyGlobal; }
					glColor4fv(col.GetDataPointer());
				}

				glNormal3fv(trig.normals[i].GetDataPointer());
				glVertex3fv(trig.points[i].GetDataPointer());
			}
			glEnd();
		}
		if (edgeMode) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		}
	}

};

extern GlfwRenderer glfwRenderer; //this is the (static) location of the renderer class; could also be made dynamic



#endif //USE_GLFW_GRAPHICS
#endif //include once
