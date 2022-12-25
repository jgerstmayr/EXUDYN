/** ***********************************************************************************************
* @brief		Interface to virtual reality
*
* @author		Aaron Bacher
* @date			2022-12-07 (generated)
* @copyright    Copyright (c) 2015, Valve Corporation. See "LICENSE.txt" for more details.
*               For full information on Valve license see https://github.com/ValveSoftware/openvr/blob/master/LICENSE
* @note:	    Code (including dependend .cpp-file) heavily based on code from https://github.com/ValveSoftware/openvr/blob/master/samples/hellovr_opengl/hellovr_opengl_main.cpp
************************************************************************************************ */

#ifndef OPENVRINTERFACE__H
#define OPENVRINTERFACE__H

/*
*** first include GLFWclient and check macro

#if defined( __linux__ )
# include <openvr/openvr.h>
#elif defined ( _WIN32 )
#include <openvr_mingw.hpp>
#endif

#include <string>

#include "glad/glad.h"

#include "valve/utils/lodepng.h"
#include "valve/utils/Matrices.h"
#include "valve/utils/Vectors.h"
#include "valve/utils/pathtools.h"

#include "Logger.hpp"

class GLFWwindow;	// forward declaration, defined in lib glfw3

class CGLRenderModel
{
public:
	CGLRenderModel( const std::string & sRenderModelName );
	~CGLRenderModel();

	bool init( const vr::RenderModel_t & vrModel, const vr::RenderModel_TextureMap_t & vrDiffuseTexture );
	void cleanup();
	void draw();
	const std::string & getName() const { return m_sModelName; }

private:
	GLuint m_glVertBuffer;
	GLuint m_glIndexBuffer;
	GLuint m_glVertArray;
	GLuint m_glTexture;
	GLsizei m_unVertexCount;
	std::string m_sModelName;
};

class OpenVRManager{
public:
    OpenVRManager(std::shared_ptr<TextLogger> logger);
	virtual ~OpenVRManager();

	// to be called right after constructing instance for setting up all relevant systems
	// poiner to companionWindow (GL-Desktop-Window) and its size are passed to make further access (rendering) possible
	bool initVR(GLFWwindow* companionWindowPointer, uint32_t windowWidth, uint32_t windowHeight);

	// clean up system
	void shutdown();

	// init VR compositor, setup 
	bool initCompositor();

	// detect currently available vr-devices and register them
	void detectDevices();

	// returns true if HMD is available
	bool HmdPresent();

	// handles all kinds of input (active, passive, events)
	bool handleInput();
	
	// renders frame on hmd and companion window
	void renderFrame();

	// reads and converts all poses to Matrix4 and writes them into passed vectors
	void getTrackedDevicePoseMatrices(std::vector<Matrix4> &Controller, std::vector<Matrix4> &Tracker);

	// returns current view projection matrix of specific eye (0, 1)
	Matrix4 getCurrentViewProjectionMatrix( vr::Hmd_Eye nEye );

	void setSceneVAO(GLuint vao);			// glad specific?
	void setTexture(GLuint tex);			// glad specific?
	void setVertCount(unsigned int count);	// glad specific?

private:

	// interface for used logger class
	static void log(const char* logEntry, LogLevel logLevel);

	// creates shaders, setups cameras and windows
	bool initGL();

	// methods called by initGL
	void setupCameras();
	bool setupStereoRenderTargets();
	void setupCompanionWindow();
	bool createAllShaders();
	GLuint compileGLShader( const char *pchShaderName, const char *pchVertexShader, const char *pchFragmentShader );	// called by createAllShaders()

	// input handling
	bool handleActionState();
	void processVREvent( const vr::VREvent_t & event );
	void updateTrackedDevicePoses();

	// creates renderable coordinate system for controllers
	void renderControllerAxes();

	// rendering on hmd and companion window
	void renderStereoTargets();
	void renderScene( vr::Hmd_Eye nEye );
	void renderCompanionWindow();

	// called once per eye at setup to set projection matrices (projection, not orientation or translation)
	Matrix4 getHMDMatrixProjectionEye( vr::Hmd_Eye nEye );

	// called once per eye at setup to set head to eye translation
	Matrix4 getHMDMatrixPoseEye( vr::Hmd_Eye nEye );

	CGLRenderModel *findOrLoadRenderModel( const char *pchRenderModelName );

private:

	inline static std::shared_ptr<TextLogger> s_logger;

	// pointer to hmd object
    vr::IVRSystem *m_pHMD;

	// strings holding driver and display version, currently only needed for setting window title
	std::string m_strDriver;
	std::string m_strDisplay;

	// VR devices (devices keep their ids (indices in these arrays) during runtime even if switched off and on again)
	// edit: seems as if they would keep id even after restarting prgram, so always have the same one???
	// maybe it depends on the order they have been connected to the vr system / switched on in the first place
	vr::TrackedDevicePose_t m_rTrackedDevicePose[ vr::k_unMaxTrackedDeviceCount ];
	Matrix4 m_rmat4DevicePose[ vr::k_unMaxTrackedDeviceCount ];
	vr::ETrackedDeviceClass m_trackedDeviceClass[ vr::k_unMaxTrackedDeviceCount ];
	std::string m_strTrackedDeviceClass[ vr::k_unMaxTrackedDeviceCount ];
	// note: I can't make a struct out of those 4 types and then make an array of structs (instead of 4 single arrays), as VRCompositor needs array 
	// of vr::TrackedDevicePose_t which again would make things more complicated

	struct ControllerInfo_t
	{
		vr::VRInputValueHandle_t m_source = vr::k_ulInvalidInputValueHandle;
		vr::VRActionHandle_t m_actionPose = vr::k_ulInvalidActionHandle;
		vr::VRActionHandle_t m_actionHaptic = vr::k_ulInvalidActionHandle;
		Matrix4 m_rmat4Pose;
		CGLRenderModel *m_pRenderModel = nullptr;
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

	struct VertexDataWindow
	{
		Vector2 position;
		Vector2 texCoord;

		VertexDataWindow( const Vector2 & pos, const Vector2 tex ) :  position(pos), texCoord(tex) {	}
	};

	vr::VRActionHandle_t m_actionHideCubes = vr::k_ulInvalidActionHandle;
	vr::VRActionHandle_t m_actionHideThisController = vr::k_ulInvalidActionHandle;
	vr::VRActionHandle_t m_actionTriggerHaptic = vr::k_ulInvalidActionHandle;
	vr::VRActionHandle_t m_actionAnalongInput = vr::k_ulInvalidActionHandle;

	vr::VRActionSetHandle_t m_actionsetDemo = vr::k_ulInvalidActionSetHandle;

	// some flags
	bool m_bShowCubes;
	Vector2 m_vAnalogValue;	// currently not in use

	// GLFW window
	GLFWwindow* m_pCompanionWindow;
	uint32_t m_nCompanionWindowWidth;
	uint32_t m_nCompanionWindowHeight;
	GLuint m_unCompanionWindowVAO;
	GLuint m_glCompanionWindowIDVertBuffer;
	GLuint m_glCompanionWindowIDIndexBuffer;
	unsigned int m_uiCompanionWindowIndexSize;

	// GL stuff
	GLuint m_unSceneVAO;
	GLuint m_glControllerVertBuffer;
	GLuint m_unControllerVAO;
	unsigned int m_uiControllerVertcount;

	// program = shader -> shader-ids
	GLuint m_unSceneProgramID;
	GLuint m_unCompanionWindowProgramID;
	GLuint m_unControllerTransformProgramID;
	GLuint m_unRenderModelProgramID;

	// also have to do something with shaders
	GLint m_nSceneMatrixLocation;
	GLint m_nControllerMatrixLocation;
	GLint m_nRenderModelMatrixLocation;

	// glad
	unsigned int m_uiVertcount;

	// texture id
	GLuint m_iTexture;

	// projection matrices
	Matrix4 m_mat4HMDPose;
	Matrix4 m_mat4eyePosLeft;
	Matrix4 m_mat4eyePosRight;

	Matrix4 m_mat4ProjectionCenter;
	Matrix4 m_mat4ProjectionLeft;
	Matrix4 m_mat4ProjectionRight;

	struct Framebuffer
	{
		GLuint m_nDepthBufferId;        // GLuint = unsigned int
		GLuint m_nRenderTextureId;
		GLuint m_nRenderFramebufferId;
		GLuint m_nResolveTextureId;
		GLuint m_nResolveFramebufferId;
	};

	bool createFrameBuffer( int nWidth, int nHeight, Framebuffer &framebufferDesc );

	// separate frame buffer for each eye
	Framebuffer leftEyeDesc;
	Framebuffer rightEyeDesc;
	
	uint32_t m_nRenderWidth;
	uint32_t m_nRenderHeight;

	float m_fNearClip = 0.1f;
	float m_fFarClip = 30.0f;

	std::vector< CGLRenderModel * > m_vecRenderModels;
};

*/

#endif
