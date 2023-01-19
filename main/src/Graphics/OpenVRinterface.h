/** ***********************************************************************************************
* @brief		Interface to virtual reality
*
* @author		Aaron Bacher, Johannes Gerstmayr (modifications of original interface)
* @date			2022-12-07 (generated)
* @copyright    Copyright (c) 2015, Valve Corporation. See "LICENSE.txt" for more details.
*               For full information on Valve license see https://github.com/ValveSoftware/openvr/blob/master/LICENSE
* @note:	    Code (including dependend .cpp-file) heavily based on code from https://github.com/ValveSoftware/openvr/blob/master/samples/hellovr_opengl/hellovr_opengl_main.cpp
************************************************************************************************ */

#ifndef OPENVRINTERFACE__H
#define OPENVRINTERFACE__H


//*** first include GLFWclient and check macro

#if defined(__EXUDYN__WINDOWS__) || defined(__EXUDYN__LINUX__) //should also work on windows
//#define OPENVR_BUILD_STATIC
//#define VR_BUILD_SHARED
#include <openvr/openvr.h>
//#elif defined ( _WIN32 ) //?
//#include <openvr/openvr_mingw.hpp>
#endif

class GlfwRenderer;

#include <string>

//#include "glad/glad.h"
//
//#include "valve/utils/lodepng.h"
//#include "valve/utils/Matrices.h"
//#include "valve/utils/Vectors.h"
//#include "valve/utils/pathtools.h"
//
//#include "Logger.hpp"

//! internally used for openVR interface
enum class LogLevel {
    Error=0,
    Warning=1,
    Info=2,
    Debug=3
};

class OpenVRparameters
{
public:
    bool activate;

    //++++++++++++++++++++++++++++++++++++++++++++
    OpenVRparameters() { Initialize();}
    void Initialize()
    {
        activate = true;
    }
};

//DELETE
////! this is the data which is returned on request
////! HMD = Head-Mounted Display
////! this should be rather put into GlfwRenderer, as it will be part of renderstate and needs a conversion to Python
//class OpenVRstate
//{
//public:
//    Matrix4DF hmdPose;
//
//    //++++++++++++++++++++++++++++++++++++++++++++
//    OpenVRstate() { Initialize(); }
//    void Initialize()
//    {
//        hmdPose.SetScalarMatrix(4, 1.f);
//    }
//};

//class OpenVRinterface
//{
//public:
//	OpenVRinterface() {}
//    bool InitVR(); //this is the internal function
//};


class OpenVRinterface
{
public:
    OpenVRinterface();
	virtual ~OpenVRinterface();

    bool IsActivated() { return HmdPresent(); }

	//! general initialization function; return true, if successful and running
    bool InitOpenVR(GlfwRenderer* glfwRendererInit);
	bool RenderAndUpdateDevices();
    //void SetDataAndParameters(const OpenVRparameters& params) { openVRparameters = params; };

	// clean up system and shut down
    void ShutDown();

	//! return current information in openVR, mostly about transformations
	void GetState(OpenVRState& state);
	//! set log level to be used; lower means less output
	void SetLogLevel(Index level) 
	{ 
		logLevelUsed = (LogLevel)level;  
		Log(("OpenVR: set log level to " + EXUstd::ToString((int)logLevelUsed)).c_str(),LogLevel::Info);
	}

private: //member functions

	// interface for used logger class
	void Log(const char* logEntry, LogLevel logLevel);
	
	//! return backlink to renderer
	GlfwRenderer* GetGlfwRenderer() { return glfwRenderer; }
	
	// to be called right after constructing instance for setting up all relevant systems
    // pointer to companionWindow (GL-Desktop-Window) and its size are passed to make further access (rendering) possible
    bool InitVR(); //this is the internal function

	// init VR compositor, setup 
	bool InitCompositor();

	// detect currently available vr-devices and register them
	void DetectDevices();

	// handles all kinds of input (active, passive, events)
	bool HandleInput();

	// returns current view projection matrix of specific eye (0, 1)
	Matrix4DF GetCurrentViewProjectionMatrix( vr::Hmd_Eye nEye );

	// methods called by initGL
	void SetupCameras();
	bool SetupStereoRenderTargets();
	//void setupCompanionWindow();
	//bool createAllShaders();
	//GLuint compileGLShader( const char *pchShaderName, const char *pchVertexShader, const char *pchFragmentShader );	// called by createAllShaders()

	// input handling
	bool handleActionState();
	void ProcessVREvent( const vr::VREvent_t & event );
	void UpdateTrackedDevicePoses();

	// creates renderable coordinate system for controllers ==> will be done in GlfwClient!
	//void renderControllerAxes();

	// rendering on hmd and companion window
	void RenderStereoTargets();
	//void renderCompanionWindow();

	// called once per eye at setup to set projection matrices (projection, not orientation or translation)
	Matrix4DF GetHMDMatrixProjectionEye( vr::Hmd_Eye nEye );

	// called once per eye at setup to set head to eye translation
	Matrix4DF GetHMDMatrixPoseEye( vr::Hmd_Eye nEye );

	// returns true if HMD is available
	bool HmdPresent()
	{
		if (m_pHMD == nullptr) return false;
		else return true;
	}

private:
    LogLevel logLevelUsed;		//!< internally stored log level
    GlfwRenderer* glfwRenderer;			//!< backlink to GLFWRenderer

	// pointer to hmd object
    vr::IVRSystem *m_pHMD;

	// strings holding driver and display version, currently only needed for setting window title
	std::string m_strDriver;
	std::string m_strDisplay;

	GLuint eyeTextures[2]; //number of textures created by GL, used for left/right eye


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// VR devices (devices keep their ids (indices in these arrays) during runtime even if switched off and on again)
	// edit: seems as if they would keep id even after restarting prgram, so always have the same one???
	// maybe it depends on the order they have been connected to the vr system / switched on in the first place
	vr::TrackedDevicePose_t m_rTrackedDevicePose[ vr::k_unMaxTrackedDeviceCount ];
	Matrix4DF m_rmat4DevicePose[ vr::k_unMaxTrackedDeviceCount ];
	vr::ETrackedDeviceClass m_trackedDeviceClass[ vr::k_unMaxTrackedDeviceCount ];
	std::string m_strTrackedDeviceClass[ vr::k_unMaxTrackedDeviceCount ];
	// note: I can't make a struct out of those 4 types and then make an array of structs (instead of 4 single arrays), as VRCompositor needs array 
	// of vr::TrackedDevicePose_t which again would make things more complicated

	struct ControllerInfo_t
	{
		vr::VRInputValueHandle_t m_source = vr::k_ulInvalidInputValueHandle;
		vr::VRActionHandle_t m_actionPose = vr::k_ulInvalidActionHandle;
		vr::VRActionHandle_t m_actionHaptic = vr::k_ulInvalidActionHandle;
		Matrix4DF m_rmat4Pose;
		std::string m_sRenderModelName;
		bool m_bShowController;
	};

	// makelifeeasier-enumeration for left and right hand
	enum EHand
	{
		Left = 0,
		Right = 1,
	};

	ControllerInfo_t m_rHand[2];

	vr::VRActionHandle_t m_actionHideCubes = vr::k_ulInvalidActionHandle;
	vr::VRActionHandle_t m_actionHideThisController = vr::k_ulInvalidActionHandle;
	vr::VRActionHandle_t m_actionTriggerHaptic = vr::k_ulInvalidActionHandle;
	vr::VRActionHandle_t m_actionAnalongInput = vr::k_ulInvalidActionHandle;

	vr::VRActionSetHandle_t m_actionsetDemo = vr::k_ulInvalidActionSetHandle;

	// some flags
	Float2 m_vAnalogValue;	// set by controller
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//projection and other matrices

	// projection matrices
	Matrix4DF m_mat4HMDPose;
	Matrix4DF m_mat4eyePosLeft;
	Matrix4DF m_mat4eyePosRight;

	Matrix4DF m_mat4ProjectionCenter;
	Matrix4DF m_mat4ProjectionLeft;
	Matrix4DF m_mat4ProjectionRight;
	
	uint32_t m_nRenderWidth;
	uint32_t m_nRenderHeight;

	float m_fNearClip = 0.1f; //TODO: should be adjustable via visualizationSettings? combine with maxSceneSize?
	float m_fFarClip = 30.0f; //TODO: should be adjustable via visualizationSettings? combine with maxSceneSize?

};



#endif
