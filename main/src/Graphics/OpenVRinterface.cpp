/** ***********************************************************************************************
* @brief		Interface to virtual reality
*
* @author		Aaron Bacher, Johannes Gerstmayr (modifications of original interface)
* @date			2022-12-07 (generated)
* @copyright    Copyright (c) 2015, Valve Corporation. See "LICENSE.txt" for more details.
*               For full information on Valve license see https://github.com/ValveSoftware/openvr/blob/master/LICENSE
*               Copyright for modifications: This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note:	    Code (including dependend .cpp-file) heavily based on code from https://github.com/ValveSoftware/openvr/blob/master/samples/hellovr_opengl/hellovr_opengl_main.cpp
************************************************************************************************ */

//NOTES:
//to run test mode of openVR without Valve HMD:
//  start Steam?
//  start RiftCat 2.0 on Windows PC and connect to mobile phone with app installed
//set preprocessor flag:__EXUDYN_USE_OPENVR
//add lib: openvr_api.lib

//#include "C:/Users/c8501009/Desktop/delete/openvr-master/samples/thirdparty/glew/glew-1.11.0/include/GL/glew.h"

#include "Graphics/GlfwClient.h"

//#undef __EXUDYN_USE_OPENVR
#ifdef __EXUDYN_USE_OPENVR
#pragma message("====================================")
#pragma message("=  COMPILED WITH OPENVR            =")
#pragma message("====================================")


#include "Graphics/OpenVRinterface.h"
//#include <GL/glew.h>

OpenVRinterface glfwOpenVRinterface;


void ThreadSleep( unsigned long nMilliseconds )
{
    std::this_thread::sleep_for(std::chrono::milliseconds(nMilliseconds));
}

#ifndef _countof
#define _countof(x) (sizeof(x)/sizeof((x)[0]))
#endif

#if defined (__EXUDYN__LINUX__)
#define stricmp strcasecmp
#endif

//---------------------------------------------------------------------------------------------------------------------
// Purpose: Returns true if the action is active and had a rising edge
//---------------------------------------------------------------------------------------------------------------------
bool GetDigitalActionRisingEdge(vr::VRActionHandle_t action, vr::VRInputValueHandle_t *pDevicePath = nullptr )
{
	vr::InputDigitalActionData_t actionData;
	vr::VRInput()->GetDigitalActionData(action, &actionData, sizeof(actionData), vr::k_ulInvalidInputValueHandle );
	if (pDevicePath)
	{
		*pDevicePath = vr::k_ulInvalidInputValueHandle;
		if (actionData.bActive)
		{
			vr::InputOriginInfo_t originInfo;
			if (vr::VRInputError_None == vr::VRInput()->GetOriginTrackedDeviceInfo(actionData.activeOrigin, &originInfo, sizeof(originInfo)))
			{
				*pDevicePath = originInfo.devicePath;
			}
		}
	}
	return actionData.bActive && actionData.bChanged && actionData.bState;
}


//---------------------------------------------------------------------------------------------------------------------
// Purpose: Returns true if the action is active and had a falling edge
//---------------------------------------------------------------------------------------------------------------------
bool GetDigitalActionFallingEdge(vr::VRActionHandle_t action, vr::VRInputValueHandle_t *pDevicePath = nullptr )
{
	vr::InputDigitalActionData_t actionData;
	vr::VRInput()->GetDigitalActionData(action, &actionData, sizeof(actionData), vr::k_ulInvalidInputValueHandle );
	if (pDevicePath)
	{
		*pDevicePath = vr::k_ulInvalidInputValueHandle;
		if (actionData.bActive)
		{
			vr::InputOriginInfo_t originInfo;
			if (vr::VRInputError_None == vr::VRInput()->GetOriginTrackedDeviceInfo(actionData.activeOrigin, &originInfo, sizeof(originInfo)))
			{
				*pDevicePath = originInfo.devicePath;
			}
		}
	}
	return actionData.bActive && actionData.bChanged && !actionData.bState;
}


//---------------------------------------------------------------------------------------------------------------------
// Purpose: Returns true if the action is active and its state is true
//---------------------------------------------------------------------------------------------------------------------
bool GetDigitalActionState(vr::VRActionHandle_t action, vr::VRInputValueHandle_t *pDevicePath = nullptr )
{
	vr::InputDigitalActionData_t actionData;
	vr::VRInput()->GetDigitalActionData(action, &actionData, sizeof(actionData), vr::k_ulInvalidInputValueHandle );
	if (pDevicePath)
	{
		*pDevicePath = vr::k_ulInvalidInputValueHandle;
		if (actionData.bActive)
		{
			vr::InputOriginInfo_t originInfo;
			if (vr::VRInputError_None == vr::VRInput()->GetOriginTrackedDeviceInfo(actionData.activeOrigin, &originInfo, sizeof(originInfo)))
			{
				*pDevicePath = originInfo.devicePath;
			}
		}
	}
	return actionData.bActive && actionData.bState;
}

//-----------------------------------------------------------------------------
// Purpose: Helper to get a string from a tracked device property and turn it
//			into a std::string
//-----------------------------------------------------------------------------
std::string GetTrackedDeviceString( vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL )
{
	uint32_t unRequiredBufferLen = vr::VRSystem()->GetStringTrackedDeviceProperty( unDevice, prop, NULL, 0, peError );
	if( unRequiredBufferLen == 0 )
		return "";

	char *pchBuffer = new char[ unRequiredBufferLen ];
	unRequiredBufferLen = vr::VRSystem()->GetStringTrackedDeviceProperty( unDevice, prop, pchBuffer, unRequiredBufferLen, peError );
	std::string sResult = pchBuffer;
	delete [] pchBuffer;
	return sResult;
}

//-----------------------------------------------------------------------------
// Purpose: helper to get a string from a tracked device type class
//-----------------------------------------------------------------------------
std::string GetTrackedDeviceClassString(vr::ETrackedDeviceClass td_class) {

	std::string str_td_class = "Unknown class";

	switch (td_class)
	{
	case vr::TrackedDeviceClass_Invalid:			// = 0, the ID was not valid.
		str_td_class = "invalid";
		break;
	case vr::TrackedDeviceClass_HMD:				// = 1, Head-Mounted Displays
		str_td_class = "hmd";
		break;
	case vr::TrackedDeviceClass_Controller:			// = 2, Tracked controllers
		str_td_class = "controller";
		break;
	case vr::TrackedDeviceClass_GenericTracker:		// = 3, Generic trackers, similar to controllers
		str_td_class = "generic tracker";
		break;
	case vr::TrackedDeviceClass_TrackingReference:	// = 4, Camera and base stations that serve as tracking reference points
		str_td_class = "base station";
		break;
	case vr::TrackedDeviceClass_DisplayRedirect:	// = 5, Accessories that aren't necessarily tracked themselves, but may redirect video output from other tracked devices
		str_td_class = "display redirect";
		break;
	}

	return str_td_class;
}

//-----------------------------------------------------------------------------
// Purpose: Converts a SteamVR matrix to our local matrix class
//-----------------------------------------------------------------------------
Matrix4DF convertSteamVRMatrixToMatrix4(const vr::HmdMatrix34_t &matPose)
{
    Matrix4DF matrixObj(4,4,{
        matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], 0.0,
        matPose.m[0][1], matPose.m[1][1], matPose.m[2][1], 0.0,
        matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], 0.0,
        matPose.m[0][3], matPose.m[1][3], matPose.m[2][3], 1.0f
        });
	return matrixObj;
}


//-----------------------------------------------------------------------------
// Purpose: Constructor
//-----------------------------------------------------------------------------
OpenVRinterface::OpenVRinterface()
	: m_pHMD( NULL )
	, glfwRenderer(nullptr)
	, logLevelUsed(LogLevel::Error)
{
	// set initial class of all devices to 0 ("invalid")
	memset(m_trackedDeviceClass, 0, sizeof(m_trackedDeviceClass));
}

//-----------------------------------------------------------------------------
// Purpose: Destructor
//-----------------------------------------------------------------------------
OpenVRinterface::~OpenVRinterface()
{
	Log((char*)"OpenVRinterface Shutdown", LogLevel::Info);
}

//-----------------------------------------------------------------------------
// Purpose: Interface to logger class
//-----------------------------------------------------------------------------
void OpenVRinterface::Log(const char* logEntry, LogLevel logLevel)
{
	if ((int)logLevel <= (int)logLevelUsed)
	{
		//std::cout << "log " << (int)logLevel << "/" << (int)logLevelUsed << ":" << logEntry << "\n";
		outputBuffer.WriteVisualization(logEntry);
		outputBuffer.WriteVisualization("\n");
	}
}

bool OpenVRinterface::InitOpenVR(GlfwRenderer* glfwRendererInit) 
{ 
    glfwRenderer = glfwRendererInit;

	return InitVR();
};

//-----------------------------------------------------------------------------
// Purpose: initializes and sets up VR
// 			returns true if all initializations have been successfull, false otherwise
//-----------------------------------------------------------------------------
bool OpenVRinterface::InitVR()
{
    // Load SteamVR Runtime
	vr::EVRInitError eError = vr::VRInitError_None;
	m_pHMD = vr::VR_Init( &eError, vr::VRApplication_Scene );

	// check if successfull
	if ( eError != vr::VRInitError_None )
	{
		m_pHMD = NULL;
		std::string errorMsg = "Unable to init VR runtime: " + std::string(vr::VR_GetVRInitErrorAsEnglishDescription( eError ));
		Log(errorMsg.c_str(), LogLevel::Error);
		return false;
	} else 
    {
		Log((char*)"VR initialized", LogLevel::Info);
	}

	// get display and driver name
	m_strDriver = "No Driver";
	m_strDisplay = "No Display";

	m_strDriver = GetTrackedDeviceString( vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String );
	m_strDisplay = GetTrackedDeviceString( vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String );

	// initialize VR-Compositor (responsible for rendering on hmd?)
	if (!InitCompositor())
	{
		Log("Failed to initialize VR Compositor!", LogLevel::Error);
		return false;
	} else {
		Log((char*)"VR Compositor initialized", LogLevel::Info);
	}

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //TODO: 
	//this might be made more generic? 
	//what happens if a input device is not available?
	// A) ==> add to interactive.openVR settings ?
	// B) ==> add names to state and work with that?

    // VR Action Handles
	//this should be done via the STEAM partner site setting ?
	//vr::VRInput()->SetActionManifestPath( Path_MakeAbsolute( "../hellovr_actions.json", Path_StripFilename( Path_GetExecutablePath() ) ).c_str() );
	//vr::VRInput()->SetActionManifestPath("C:/DATA/cpp/DocumentationAndInformation/openVR/hellovr_actions.json");// / hellovr_actions.json" );
	vr::VRInput()->SetActionManifestPath(GetGlfwRenderer()->GetVisualizationSettings()->interactive.openVR.actionManifestFileName.c_str());

	vr::VRInput()->GetActionHandle( "/actions/demo/in/HideCubes", &m_actionHideCubes );
	vr::VRInput()->GetActionHandle( "/actions/demo/in/HideThisController", &m_actionHideThisController);
	vr::VRInput()->GetActionHandle( "/actions/demo/in/TriggerHaptic", &m_actionTriggerHaptic );
	vr::VRInput()->GetActionHandle( "/actions/demo/in/AnalogInput", &m_actionAnalongInput );

	vr::VRInput()->GetActionSetHandle( "/actions/demo", &m_actionsetDemo );

	vr::VRInput()->GetActionHandle( "/actions/demo/out/Haptic_Left", &m_rHand[Left].m_actionHaptic );
	vr::VRInput()->GetInputSourceHandle( "/user/hand/left", &m_rHand[Left].m_source );
	vr::VRInput()->GetActionHandle( "/actions/demo/in/Hand_Left", &m_rHand[Left].m_actionPose );

	vr::VRInput()->GetActionHandle( "/actions/demo/out/Haptic_Right", &m_rHand[Right].m_actionHaptic );
	vr::VRInput()->GetInputSourceHandle( "/user/hand/right", &m_rHand[Right].m_source );
	vr::VRInput()->GetActionHandle( "/actions/demo/in/Hand_Right", &m_rHand[Right].m_actionPose );
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// detect and register all currently available devices
	DetectDevices();

	// initialize projection matrices and stereo target textures
	SetupCameras();
	if (!SetupStereoRenderTargets()) { return false; }
	
	HandleInput(); //initialize HMDpose!

	return true;
}

//-----------------------------------------------------------------------------
// Purpose: free memory, reset pointers
//-----------------------------------------------------------------------------
void OpenVRinterface::ShutDown()
{
	// quit VR
	if( m_pHMD )
	{
		vr::VR_Shutdown();
		m_pHMD = NULL;
	}

	// companion window should be deleted in OpenGL-Application as it has also been created there
	// so just make sure that it can't be used again here
	//m_pCompanionWindow = nullptr;
}

//-----------------------------------------------------------------------------
// Purpose: Initialize Compositor. Returns true if the compositor was
//          successfully initialized, false otherwise.
//-----------------------------------------------------------------------------
bool OpenVRinterface::InitCompositor()
{
	vr::EVRInitError peError = vr::VRInitError_None;

	if ( !vr::VRCompositor() )
	{
		return false;
	}

	return true;
}

//-----------------------------------------------------------------------------
// Purpose: handles all sorts of VR-Input: active (buttons), passive (poses) and VR-events
//-----------------------------------------------------------------------------
bool OpenVRinterface::HandleInput()
{
	if (!HmdPresent()) { return false; }
	// process SteamVR events
	vr::VREvent_t event;
	while( m_pHMD->PollNextEvent( &event, sizeof( event ) ) )
	{
		ProcessVREvent( event );
	}

	// update tracked device positions and orientation
	UpdateTrackedDevicePoses();

	// process active input, return true if program should terminate
	return handleActionState();
}

//-----------------------------------------------------------------------------
// Purpose: handle active input, so controller buttons pushed etc
//-----------------------------------------------------------------------------
bool OpenVRinterface::handleActionState()
{
	//TODO: 
	//this needs to be connected to renderer?

	if (!HmdPresent()) { return false; }

	// Process SteamVR action state
	// UpdateActionState is called each frame to update the state of the actions themselves. The application
	// controls which action sets are active with the provided array of VRActiveActionSet_t structs.
	vr::VRActiveActionSet_t actionSet = { 0 };
	actionSet.ulActionSet = m_actionsetDemo;
	vr::VRInput()->UpdateActionState( &actionSet, sizeof(actionSet), 1 );

	//m_bShowCubes = !GetDigitalActionState( m_actionHideCubes );

	vr::VRInputValueHandle_t ulHapticDevice;
	if ( GetDigitalActionRisingEdge( m_actionTriggerHaptic, &ulHapticDevice ) )
	{
		if ( ulHapticDevice == m_rHand[Left].m_source )
		{
			vr::VRInput()->TriggerHapticVibrationAction( m_rHand[Left].m_actionHaptic, 0, 1, 4.f, 1.0f, vr::k_ulInvalidInputValueHandle );
		}
		if ( ulHapticDevice == m_rHand[Right].m_source )
		{
			vr::VRInput()->TriggerHapticVibrationAction( m_rHand[Right].m_actionHaptic, 0, 1, 4.f, 1.0f, vr::k_ulInvalidInputValueHandle );
		}
	}

	vr::InputAnalogActionData_t analogData;
	if ( vr::VRInput()->GetAnalogActionData( m_actionAnalongInput, &analogData, sizeof( analogData ), vr::k_ulInvalidInputValueHandle ) == vr::VRInputError_None && analogData.bActive )
	{
		m_vAnalogValue[0] = analogData.x;
		m_vAnalogValue[1] = analogData.y;
	}

	m_rHand[Left].m_bShowController = true;
	m_rHand[Right].m_bShowController = true;

	vr::VRInputValueHandle_t ulHideDevice;
	if ( GetDigitalActionState( m_actionHideThisController, &ulHideDevice ) )
	{
		if ( ulHideDevice == m_rHand[Left].m_source )
		{
			m_rHand[Left].m_bShowController = false;
		}
		if ( ulHideDevice == m_rHand[Right].m_source )
		{
			m_rHand[Right].m_bShowController = false;
		}
	}

	for ( EHand eHand = Left; eHand <= Right; ((int&)eHand)++ )
	{
		vr::InputPoseActionData_t poseData;
		if ( vr::VRInput()->GetPoseActionDataForNextFrame( m_rHand[eHand].m_actionPose, vr::TrackingUniverseStanding, &poseData, sizeof( poseData ), vr::k_ulInvalidInputValueHandle ) != vr::VRInputError_None
			|| !poseData.bActive || !poseData.pose.bPoseIsValid )
		{
			m_rHand[eHand].m_bShowController = false;
		}
		else
		{
			m_rHand[eHand].m_rmat4Pose = convertSteamVRMatrixToMatrix4( poseData.pose.mDeviceToAbsoluteTracking );

			vr::InputOriginInfo_t originInfo;
			if ( vr::VRInput()->GetOriginTrackedDeviceInfo( poseData.activeOrigin, &originInfo, sizeof( originInfo ) ) == vr::VRInputError_None 
				&& originInfo.trackedDeviceIndex != vr::k_unTrackedDeviceIndexInvalid )
			{
                //TODO:
                //here we can communicate with GLFW renderer to show hand

				//std::string sRenderModelName = GetTrackedDeviceString( originInfo.trackedDeviceIndex, vr::Prop_RenderModelName_String );
				//if ( sRenderModelName != m_rHand[eHand].m_sRenderModelName )
				//{
				//	m_rHand[eHand].m_pRenderModel = findOrLoadRenderModel( sRenderModelName.c_str() );
				//	m_rHand[eHand].m_sRenderModelName = sRenderModelName;
				//}
			}
		}
	}

	// atm there is no functionality to terminate program, so always return false
	return false;
}

//-----------------------------------------------------------------------------
// Purpose: called once at start of main loop; looks for and identifies devices
//			new devices tracked during runtime are handled by event-callbacks
//			devices keep their id during runtime even if switched off and on again
//-----------------------------------------------------------------------------
void OpenVRinterface::DetectDevices(){
	// provide some variables for device tracking
	int base_stations_count = 0;

	// check which devices are connected
	for (uint32_t nDevice=vr::k_unTrackedDeviceIndex_Hmd; nDevice<vr::k_unMaxTrackedDeviceCount; nDevice++) {

		if (m_pHMD->IsTrackedDeviceConnected(nDevice))
		{
			// determine type/class of detected device
			m_trackedDeviceClass[nDevice] = m_pHMD->GetTrackedDeviceClass(nDevice);
			m_strTrackedDeviceClass[nDevice] = GetTrackedDeviceClassString(m_trackedDeviceClass[nDevice]);

			Log(("Detected device (#" + std::to_string(nDevice) + ") of type " + m_strTrackedDeviceClass[nDevice]).c_str(), LogLevel::Info);

			// keep track of detected base stations (not sure if relevant)
			if (m_trackedDeviceClass[nDevice] == vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference) base_stations_count++;
		}
	}

	// 2 base stations should have been detected
	if(base_stations_count != 2){
		std::string msg = "number of detected Base stations: " + std::to_string(base_stations_count);
		Log(msg.c_str(), LogLevel::Warning);
	}
}


//-----------------------------------------------------------------------------
// Purpose: Processes a single VR event
//-----------------------------------------------------------------------------
void OpenVRinterface::ProcessVREvent( const vr::VREvent_t & event )
{
	if (!HmdPresent()) { return ; }

	switch( event.eventType )
	{
		// new device detected
		case vr::VREvent_TrackedDeviceActivated:
			m_trackedDeviceClass[event.trackedDeviceIndex] = m_pHMD->GetTrackedDeviceClass(event.trackedDeviceIndex);
			m_strTrackedDeviceClass[event.trackedDeviceIndex] = GetTrackedDeviceClassString(m_trackedDeviceClass[event.trackedDeviceIndex]);

			Log(("Detected device (#" + std::to_string(event.trackedDeviceIndex) + ") of type " + m_strTrackedDeviceClass[event.trackedDeviceIndex]).c_str(), LogLevel::Info);
			break;

		// lost connection to device
		case vr::VREvent_TrackedDeviceDeactivated:
			// reset device class
			m_trackedDeviceClass[event.trackedDeviceIndex] = vr::ETrackedDeviceClass::TrackedDeviceClass_Invalid;

			// make log entry
			Log(("Device #" + std::to_string(event.trackedDeviceIndex) + " detached.").c_str(), LogLevel::Info);
			break;

		// device updated, whatever that means
		case vr::VREvent_TrackedDeviceUpdated:
			Log(("Device #" + std::to_string(event.trackedDeviceIndex) + " updated.").c_str(), LogLevel::Info);
			break;
	}
}

//-----------------------------------------------------------------------------
// this data is fetched by the GLFWclient
//-----------------------------------------------------------------------------
void OpenVRinterface::GetState(OpenVRState& state)
{
	if (!HmdPresent()) { return ; }

	state.controllerPoses.SetNumberOfItems(0);
	state.trackerPoses.SetNumberOfItems(0);
	state.controllerActions.SetNumberOfItems(0);

	// iterate over all devices and insert the matrices of every found controller into matrices-vector
	for (int nDevice=0; nDevice<vr::k_unMaxTrackedDeviceCount; nDevice++){
		// controller
		if(m_trackedDeviceClass[nDevice] == vr::ETrackedDeviceClass::TrackedDeviceClass_Controller){
			state.controllerPoses.Append(m_rmat4DevicePose[nDevice]);
		}

		// tracker
		if(m_trackedDeviceClass[nDevice] == vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker){
			state.trackerPoses.Append(m_rmat4DevicePose[nDevice]);
		}
	}
	state.HMDpose = m_mat4HMDPose;
	bool rv = state.HMDpose.Invert(); //because m_mat4HMDPose is the inverted HMD pose!
	if (!rv) { Log("OpenVRinterface::GetState: inversion of HMDpose failed", LogLevel::Warning); }


	state.projectionLeft = m_mat4ProjectionLeft;
	state.eyePosLeft = m_mat4eyePosLeft;

	state.projectionRight = m_mat4ProjectionRight;
	state.eyePosRight = m_mat4eyePosRight;
}

//-----------------------------------------------------------------------------
// Purpose: render frame to desktop and hmd
//-----------------------------------------------------------------------------
bool OpenVRinterface::RenderAndUpdateDevices()
{
	if (!HmdPresent()) { return false; }

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Process SteamVR events
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	RenderStereoTargets();
	//renderCompanionWindow(); //done in GLFWrenderer

	vr::Texture_t leftEyeTexture = { (void*)(uintptr_t)eyeTextures[0], vr::TextureType_OpenGL, vr::ColorSpace_Linear }; //original:ColorSpace_Gamma
	vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture);
	vr::Texture_t rightEyeTexture = { (void*)(uintptr_t)eyeTextures[1], vr::TextureType_OpenGL, vr::ColorSpace_Linear };
	vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture);

	HandleInput(); //determine whether this should be done at the end or beginning
	return true;
}



//-----------------------------------------------------------------------------
// Purpose: setup matrices
//-----------------------------------------------------------------------------
void OpenVRinterface::SetupCameras()
{
	m_mat4ProjectionLeft = GetHMDMatrixProjectionEye( vr::Eye_Left );
	m_mat4ProjectionRight = GetHMDMatrixProjectionEye( vr::Eye_Right );
	m_mat4eyePosLeft = GetHMDMatrixPoseEye( vr::Eye_Left );
	m_mat4eyePosRight = GetHMDMatrixPoseEye( vr::Eye_Right );
	if (logLevelUsed >= LogLevel::Info)
	{
		Log("SetupCameras", LogLevel::Info);
		Log(("m_mat4ProjectionLeft:" + EXUstd::ToString(m_mat4ProjectionLeft) ).c_str(), LogLevel::Info);
		Log(("m_mat4eyePosLeft:" + EXUstd::ToString(m_mat4eyePosLeft) ).c_str(), LogLevel::Info);
		Log(("m_mat4ProjectionRight:" + EXUstd::ToString(m_mat4ProjectionRight)).c_str(), LogLevel::Info);
		Log(("m_mat4eyePosRight:" + EXUstd::ToString(m_mat4eyePosRight) ).c_str(), LogLevel::Info);
	}
}


//-----------------------------------------------------------------------------
// Purpose: create frame buffers for left and right eye
//-----------------------------------------------------------------------------
bool OpenVRinterface::SetupStereoRenderTargets()
{
	if (!HmdPresent()) { return false; }

	m_pHMD->GetRecommendedRenderTargetSize(&m_nRenderWidth, &m_nRenderHeight);
	STDstring sInfo = "OpenVR recommended render target size: w=" + EXUstd::ToString(m_nRenderWidth) + ", h=" + 
		EXUstd::ToString(m_nRenderHeight) + " (this should also be the Exudyn window.renderWindowSize)\n";

	Log(sInfo.c_str(), LogLevel::Info); //tests give w=1176, h=1320

	glGenTextures(2, &eyeTextures[0]);   //create one texture

	guint renderByteWidth = m_nRenderWidth / 8;
	guint w = renderByteWidth * 8; //make multiple of 8 (problems with overlapping area?)

	//create background (for pixels not shown) / test image
	for (Index k = 0; k < 2; k++)
	{
		const int colorSize = 4; //RGBA
		gchar* textureData;
		//guint w = characterWidth; 
		guint h = m_nRenderHeight;
		textureData = new gchar[w*h * colorSize]; //RGB bytes

		for (guint y = 0; y < h; y++)
		{
			for (guint x = 0; x < w; x++)
			{
				float xf = ((float)x / w) * 2.f - 1.f;
				float yf = ((float)y / h) * 2.f - 1.f;
				gchar a, b, c, d;
				if (false) //test
				{
					a = (gchar)(255.f*sqrt(xf*xf + yf * yf)*.5f);
					b = (gchar)(127.f*sin(xf * (10 + 10 * k)) + 127);
					c = (gchar)(127.f*cos(yf * (10 - 5 * k) + 127));
					d = 127;
				}
				else
				{ //dark grey background:
					a = 64;b = a;c = a;d = 255; //no transparency
				}
				textureData[y*w * colorSize + x * colorSize + 0] = a;
				textureData[y*w * colorSize + x * colorSize + 1] = b;
				textureData[y*w * colorSize + x * colorSize + 2] = c;
				textureData[y*w * colorSize + x * colorSize + 3] = d; 
			}
		}

		glBindTexture(GL_TEXTURE_2D, eyeTextures[k]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, renderByteWidth * 8,
			m_nRenderHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, textureData);

		delete[] textureData; //not needed lateron
	}


	return true;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void OpenVRinterface::RenderStereoTargets()
{
	if (!HmdPresent()) { return ; }

	for (Index kEye = 0; kEye < 2; kEye++)
	{
		vr::Hmd_Eye nEye;
		if (kEye == 0) { nEye = vr::Eye_Left; }
		else { nEye = vr::Eye_Right; }

		Matrix4DF hmd;
		hmd = GetCurrentViewProjectionMatrix(nEye);
		//hmd.Invert();
		GetGlfwRenderer()->SetProjectionMatrix(hmd);

		int width, height;
		GetGlfwRenderer()->GetWindowSize(width, height);
		//rendererOut << "current window: width=" << width << ", height=" << height << "\n";

		float ratio, zoom;
		GetGlfwRenderer()->Render3Dobjects(width, height, ratio, zoom);

		glFinish(); //we wait until buffer is drawn fully!

		glBindTexture(GL_TEXTURE_2D, eyeTextures[kEye]);
		glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 0, 0, m_nRenderWidth, m_nRenderHeight, 0); //render window shall have according recommended size of device
	}
	Matrix4DF eye4D;
	eye4D.SetScalarMatrix(4, 1.f);
	Matrix4DF hmd;
	//hmd = GetCurrentViewProjectionMatrix(vr::Eye_Left);
	hmd = m_mat4HMDPose;
	//hmd.Invert();
	GetGlfwRenderer()->SetProjectionMatrix(hmd);

}


//-----------------------------------------------------------------------------
// Purpose: Gets a Matrix Projection Eye with respect to nEye.
//-----------------------------------------------------------------------------
Matrix4DF OpenVRinterface::GetHMDMatrixProjectionEye( vr::Hmd_Eye nEye )
{
	if (!HmdPresent()) { Matrix4DF eye; eye.SetScalarMatrix(4, 1.f); return eye; }

	vr::HmdMatrix44_t mat = m_pHMD->GetProjectionMatrix( nEye, m_fNearClip, m_fFarClip );

    return Matrix4DF(4,4,{
        mat.m[0][0], mat.m[1][0], mat.m[2][0], mat.m[3][0],
        mat.m[0][1], mat.m[1][1], mat.m[2][1], mat.m[3][1],
        mat.m[0][2], mat.m[1][2], mat.m[2][2], mat.m[3][2],
        mat.m[0][3], mat.m[1][3], mat.m[2][3], mat.m[3][3] }
	);
}


//-----------------------------------------------------------------------------
// Purpose: Gets an HMDMatrixPoseEye with respect to nEye.
//-----------------------------------------------------------------------------
Matrix4DF OpenVRinterface::GetHMDMatrixPoseEye( vr::Hmd_Eye nEye )
{
	if (!HmdPresent()) { Matrix4DF eye; eye.SetScalarMatrix(4, 1.f); return eye; }

	vr::HmdMatrix34_t matEyeRight = m_pHMD->GetEyeToHeadTransform( nEye );
    Matrix4DF matrixObj(4, 4, {
        matEyeRight.m[0][0], matEyeRight.m[1][0], matEyeRight.m[2][0], 0.0,
        matEyeRight.m[0][1], matEyeRight.m[1][1], matEyeRight.m[2][1], 0.0,
        matEyeRight.m[0][2], matEyeRight.m[1][2], matEyeRight.m[2][2], 0.0,
        matEyeRight.m[0][3], matEyeRight.m[1][3], matEyeRight.m[2][3], 1.0f }
		);
    bool rv = matrixObj.Invert();
	if (!rv) { Log("OpenVRinterface::GetHMDMatrixPoseEye: inversion of matrixObj failed", LogLevel::Error); }

	return matrixObj;
}


//-----------------------------------------------------------------------------
// Purpose: Gets a Current View Projection Matrix with respect to nEye,
//          which may be an Eye_Left or an Eye_Right.
//-----------------------------------------------------------------------------
Matrix4DF OpenVRinterface::GetCurrentViewProjectionMatrix( vr::Hmd_Eye nEye )
{
	//std::cout << "m_mat4ProjectionLeft=" << m_mat4ProjectionLeft << "\n";
	//std::cout << "m_mat4eyePosLeft=" << m_mat4eyePosLeft << "\n";
	//std::cout << "m_mat4HMDPose=" << m_mat4HMDPose << "\n";

	Matrix4DF matMVP;
	if (m_mat4HMDPose.NumberOfRows() == 4 && m_mat4HMDPose.NumberOfColumns() == 4)
	{
		if (false)
		{   //this is the original mode with m_mat4ProjectionLeft; needs to be adjusted with Exudyn
			if (nEye == vr::Eye_Left)
			{
				matMVP = m_mat4ProjectionLeft * m_mat4eyePosLeft * m_mat4HMDPose;
			}
			else if (nEye == vr::Eye_Right)
			{
				matMVP = m_mat4ProjectionRight * m_mat4eyePosRight *  m_mat4HMDPose;
			}
		}
		else
		{   //this mode works somehow
			if (nEye == vr::Eye_Left)
			{
				matMVP = m_mat4eyePosLeft * m_mat4HMDPose;
			}
			else if (nEye == vr::Eye_Right)
			{
				matMVP = m_mat4eyePosRight * m_mat4HMDPose;
			}
		}
	}
	else
	{
		Log("GetCurrentViewProjectionMatrix: HMDPose has invalid size", LogLevel::Warning);
		matMVP.SetScalarMatrix(4, 1.f);
	}

	return matMVP;
}


//-----------------------------------------------------------------------------
// Purpose: Get current position and orientation of HMD
//-----------------------------------------------------------------------------
void OpenVRinterface::UpdateTrackedDevicePoses()
{
	if (!HmdPresent()) { return; }

	// update poses for further usage
	vr::VRCompositor()->WaitGetPoses(m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );

	for ( int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice )
	{
		if ( m_rTrackedDevicePose[nDevice].bPoseIsValid )
		{
			m_rmat4DevicePose[nDevice] = convertSteamVRMatrixToMatrix4( m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking );
		}
	}

	if ( m_rTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid )
	{
		m_mat4HMDPose = m_rmat4DevicePose[vr::k_unTrackedDeviceIndex_Hmd];
		bool rv = m_mat4HMDPose.Invert();
		if (!rv) { Log("OpenVRinterface::UpdateTrackedDevicePoses: inversion of HMDpose failed", LogLevel::Error); }
	}
}



#endif //__EXUDYN_USE_OPENVR

