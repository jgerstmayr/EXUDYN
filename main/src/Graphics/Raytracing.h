/** ***********************************************************************************************
* @class        Raytracing
* @brief        Softrenderer for animation and image generation
*
* @author       Gerstmayr Johannes
* @date         2025-05-23 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#ifndef RAYTRACING__H
#define RAYTRACING__H

#include "Graphics/GlfwClientBase.h"

#include "Graphics/VisualizationSystemContainer.h"
#include "Graphics/VisualizationPrimitives.h"

#include "Graphics/GlfwClientBitmapText.h"
#include "Graphics/characterBitmap.h"

#include "Linalg/Geometry.h"
#include "Linalg/SearchTree.h"


class GLTriangle;
class GlfwRenderer;
class VisualizationSettings;

typedef float RTfloat; //for raytracing
typedef SlimVectorBase<RTfloat, 3> RTVector3D;
typedef SlimVectorBase<RTfloat, 4> RTVector4D;
typedef ConstSizeMatrixBase<RTfloat, 9> RTMatrix3D;
typedef ConstSizeMatrixBase<RTfloat, 16> RTMatrix4D;

inline RTVector3D ColorRGBA2RGB(const RTVector4D& color4)
{
	return RTVector3D({ color4[0], color4[1], color4[2] });
}

typedef VSettingsMaterial GLMaterial;
typedef std::array<RTfloat, MAX_LIGHTS_GLFW> ShadowValue;

//! RaytracingLight: parameters have same meaning as in openGL.light
class RaytracingLight
{
public:
	//opengl parameters:
	RTVector4D position; //light pos/dir according to openGL
	RTfloat ambient; 
	RTfloat diffuse;
	RTfloat specular;
	RTfloat constantAttenuation;
	RTfloat linearAttenuation;
	RTfloat quadraticAttenuation;
	bool active;
	bool shadow;
	bool useCameraFrame;
	RTfloat lightRadius;
};

class RaytracingSettings
{

public:
	//+++++++++++++++++++++++++++++++++++++++++++++++++

	//Raytracing settings:
	Index verbose;
	bool keepWindowActive;  //!< flag to call pollEvents to avoid rendering timeouts; may lead to instability in regular use cases, but allows very long rendering (>>5 seconds)
	Index numberOfThreads;
	Index numberOfThreadsUsed;
	bool isCalledFromMainThread; //!< flag set at each call, to distinguish between calls from GlfwClient and from main thread

	static constexpr RTfloat epsilon = std::numeric_limits<RTfloat>::epsilon(); //least significant bit without exponent for float is 1.192093e-7
	static constexpr RTfloat epsilon5 = 5.f*std::numeric_limits<RTfloat>::epsilon(); //for testing of edge cases
	Index maxReflectionDepth;
	Index maxTransparencyDepth;
	Index imageSizeFactor;	// subsampling; faster rendering
	RTfloat zBiasLines;		//!< slight bias to make lines visible over coplanar geometry
	RTfloat zOffsetCamera;	//!< allows to go into scene
	//unchanged for now:
	RTfloat minZ;			//!< for triangles and reflections, etc.
	RTVector3D backgroundColorReflections; //!< for refraction
	RTVector3D ambientLightColor;
	RTfloat globalFogDensity;
	RTVector3D globalFogColor;
	RTfloat globalMaxAlpha; //!< for global transparency
	std::vector<VSettingsMaterial>* materials;
	Index tilesPerThread;

	//copy from visualizationSettings.openGL or general:
	Index AAfactor;			// 2x supersampling; higher quality
	bool showLines;
	RTfloat lineWidth;
	Index imageWidth;
	Index imageHeight;
	Index shadowWidth; // for precomputed shadow
	Index shadowHeight;// for precomputed shadow
	bool useGradientBackground;
	RTfloat perspectiveFactor;
	bool modelCentricView;
	RTVector3D backgroundColor;			//!< for zero-hits; could be white or black, ...
	RTVector3D backgroundColorBottom;	//!< for gradient background, bottom color
	bool useClippingPlane;
	RTVector3D clippingPlaneNormal;
	RTVector3D clippingPlanePoint;
	RTfloat clippingPlaneDistance;
	RTVector3D clippingPlaneColor; 
	Index useClippingPlaneColor; //!< 0=no, 1=clippingPlaneColor, 2=object inner color

	//set during initialization of a single scene:
	RTfloat tolParallel;
	RTfloat tolParallel2;
	RTfloat searchTreeBoxMaxRadius;
	RTfloat maxSceneSize; //same as in OpenGL (with clamping to minimal value)
	RTfloat zoom, ratio; //screen zoom and ratio
	RTfloat zMaxSceneFactor;
	RTVector3D nearFarPlaneOffset; //near and far plane overrides and flag
	RTfloat projectionNearMin; //! given in renderState
	Matrix4DF modelViewRM;	//!< model view matrix, row major (OpenGL is column-major!!!)
	Matrix3DF rotationView;	//!< rotation matrix of modelView, row-major; updated by caller
	RTVector3D translationView; //!< translation of modelView, row-major version; updated by caller
	RTfloat displayScaling;		//!< copy from renderState

	std::array<RaytracingLight, MAX_LIGHTS_GLFW> lights;
	
	Index lightRadiusVariations; //same for all lights
	static constexpr Index materialOffset = 1000;
	static const Index RTcolorDepth = 4;

	SearchTreeBase<RTfloat> searchTree;
	static const Index maxNThreads = 256;
	mutable ArrayIndex tempTrigIndices[maxNThreads];
	mutable ArrayIndex tempSearchTreeBins[maxNThreads];

	//! some temporary structures for rendering:
	ResizableArray<unsigned char> finalImageAA; //! for antializing, final smoothed image
	ResizableArray<unsigned char> pixelsRGBA;   //! for raytracing, the RGBA pixel buffer
	ResizableArray<RTfloat> zDepths;			//! for raytracing, storing zDepths for lateron adding texts
	ResizableArray<ShadowValue> shadowValues;	//! shadow values, per light, at possibly lower resolution
	ResizableArray<ShadowValue> smoothShadowValues; //! shadow values at original resolution
	Index shadowScalingFactor;					//! reduced resolution of shadows (AAfactor additionally multiplied)
	Index shadowSmoothingSteps;					//! number of smoothing steps at lower resolution shadow map
	bool preComputeShadow;						//! if yes, precomputes (smoothed) shadow with lower resolution

	//! convert existing color into material index, alpha and color
	inline void ColorRGBA2MaterialColor(Float4& color, Index& materialIndex, RTfloat& alpha) const
	{
		materialIndex = (Index)color[3] >= materialOffset ? EXUstd::Minimum((Index)color[3]- materialOffset, (Index)materials->size() - 1) : 0;
		alpha = (Index)color[3] >= materialOffset ? (*materials)[materialIndex].alpha : EXUstd::Clamp(color[3]);
		if (color[0] < 0.f) //in case of mixture of colors and material base colors, this will lead to artifacts ...
		{
			color[0] = (*materials)[materialIndex].baseColor[0];
			color[1] = (*materials)[materialIndex].baseColor[1];
			color[2] = (*materials)[materialIndex].baseColor[2];
		}
		alpha = EXUstd::Minimum(alpha, globalMaxAlpha); //for global transparency
		//==> now color is a RGBA color with no material index!
	}


	RaytracingSettings()
	{
		verbose = false;
		keepWindowActive = false;
		numberOfThreads = 1;
		numberOfThreadsUsed = 1;
		isCalledFromMainThread = false;

		materials = NULL;
		tilesPerThread = 12;
		zOffsetCamera = -0.01f; //always stay away a little bit from max scene
		displayScaling = 1.f;

		showLines = true;
		lightRadiusVariations = 1;
		useGradientBackground = false;
		AAfactor = 1; 
		imageSizeFactor = 1;
		perspectiveFactor = 0;
		modelCentricView = true;
		shadowScalingFactor = 2;
		shadowSmoothingSteps = 2;
		maxReflectionDepth = 0;
		maxTransparencyDepth = 0;
		globalMaxAlpha = 0;

		lineWidth = 2.f;
		minZ = 0.0f;

		zBiasLines = 1e-3f; //should be relative to sceen size
		tolParallel = 5e-7f; //will be set later when box radius is known
		tolParallel2 = tolParallel * tolParallel;
		searchTreeBoxMaxRadius = 1.f; //set when searchtree exists
		maxSceneSize = 1.f;
		zMaxSceneFactor = 2.f;
		nearFarPlaneOffset = Float3({ 0,0,0 });
		projectionNearMin = 0.1f; //set later!
	}
};

class IntersectionResult 
{
public:
	bool hit;
	bool hitFromBack;
	RTfloat distance;
	RTfloat u, v;
	RTVector3D normal;
	RTVector4D color;
	Index triangleIndex; 
	Index materialIndex; //from RGBA color
	RTfloat alpha;		 //from RGBA color or material.alpha

	inline void Init()
	{
		hit = false;
		distance = std::numeric_limits<RTfloat>::max();
	}
};

#include <vector>
#include <cmath>

//! function to map from smaller to larger image resolution:
template<class Tvalue>
inline void ScaleImageValues(const ResizableArray<Tvalue>& lowresValues, ResizableArray<Tvalue>& hiresValues,
	int lowresWidth, int lowresHeight,	int hiresWidth, int hiresHeight)
{

	float scaleX = static_cast<float>(lowresWidth) / hiresWidth;
	float scaleY = static_cast<float>(lowresHeight) / hiresHeight;
	hiresValues.SetNumberOfItems(hiresWidth * hiresHeight);

	for (int y = 0; y < hiresHeight; ++y) {
		for (int x = 0; x < hiresWidth; ++x) {
			// Map the pixel coordinates to the scaled lowresValues array
			int lowresX = static_cast<int>(x * scaleX);
			int lowresY = static_cast<int>(y * scaleY);

			// Ensure indices are within bounds
			lowresX = std::min(lowresX, lowresWidth - 1);
			lowresY = std::min(lowresY, lowresHeight - 1);

			// Assign the corresponding lowres value
			hiresValues[y * hiresWidth + x] = lowresValues[lowresY * lowresWidth + lowresX];
		}
	}
}

//! function to smooth the image values (like shadow values)
inline void SmoothImageValues(const ResizableArray<RTfloat>& imageValues, ResizableArray<RTfloat>& smoothedValues, 
	Index imageWidth, Index imageHeight, bool skipHardBoundaries = true) 
{

	// Create a copy of the original array to store smoothed values
	smoothedValues = imageValues;

	for (int y = 1; y < imageHeight - 1; ++y) {
		for (int x = 1; x < imageWidth - 1; ++x) {
			// Check the 3x3 neighborhood
			bool containsZero = false;
			bool containsOne = false;

			if (skipHardBoundaries)
			{
				for (int dy = -1; dy <= 1; ++dy) {
					for (int dx = -1; dx <= 1; ++dx) {
						float value = imageValues[(y + dy) * imageWidth + x + dx];
						if (value == 0.0f) { containsZero = true; }
						if (value == 1.0f) { containsOne = true; }
					}
				}
			}

			// If the neighborhood does not contain both 0 and 1, apply smoothing
			if (!(containsZero && containsOne)) {
				float sum = 0.0f;
				int count = 0;

				for (int dy = -1; dy <= 1; ++dy) {
					for (int dx = -1; dx <= 1; ++dx) {
						sum += imageValues[(y + dy)* imageWidth + x + dx];
						++count;
					}
				}

				smoothedValues[y * imageWidth + x] = sum / count; // Average of the 3x3 neighborhood
			}
		}
	}
}

//! function to smooth the image values (like shadow values)
inline void SmoothImageValues(const ResizableArray<ShadowValue>& imageValues, ResizableArray<ShadowValue>& smoothedValues,
	Index imageWidth, Index imageHeight, const RaytracingSettings& RTS, bool skipHardBoundaries = true)
{
	// Create a copy of the original array to store smoothed values
	smoothedValues = imageValues;

	for (Index lightID = 0; lightID < (Index)RTS.lights.size(); lightID++)
	{
		if (!RTS.lights[lightID].active || !RTS.lights[lightID].shadow) { continue; } //skip this channel

		for (int y = 1; y < imageHeight - 1; ++y) 
		{
			for (int x = 1; x < imageWidth - 1; ++x) 
			{
				// Check the 3x3 neighborhood
				bool containsZero = false;
				bool containsOne = false;

				if (skipHardBoundaries)
				{
					for (int dy = -1; dy <= 1; ++dy)
					{
						for (int dx = -1; dx <= 1; ++dx)
						{
							float value = imageValues[(y + dy) * imageWidth + x + dx][lightID];
							if (value == 0.0f) { containsZero = true; }
							if (value == 1.0f) { containsOne = true; }
						}
					}
				}

				// If the neighborhood does not contain both 0 and 1, apply smoothing
				if (!(containsZero && containsOne)) {
					float sum = 0.0f;
					int count = 9;

					for (int dy = -1; dy <= 1; ++dy) 
					{
						for (int dx = -1; dx <= 1; ++dx) 
						{
							sum += imageValues[(y + dy) * imageWidth + x + dx][lightID];
							//++count;
						}
					}

					smoothedValues[y * imageWidth + x][lightID] = sum / count; // Average of the 3x3 neighborhood
				}
			}
		}
	}
}

class Raytracer
{
private:
	//+++++++++++++++++++++++++++++++++++++++++++++++++
	//link to visualization system; this would have to be changed if used elsewhere
	VisualizationSystemContainerBase* basicVisualizationSystemContainer;
	//ResizableArray<GraphicsData*>* graphicsDataList;
	VisualizationSettings* visSettings;
	RenderState* renderState;

	RaytracingSettings RTS;

	BitmapFont bitmapFont;
	ResizableArray<gchar*> fontBitmaps;

	GraphicsData graphicsData;			//!< main graphics data
	GraphicsData graphicsDataStatic;	//!< static GraphicsData objects (info, Exudyn, etc.)

public:
	Raytracer();
	virtual ~Raytracer();

	unsigned char FloatColorToByte(RTfloat x) { return static_cast<unsigned char>((Index)(EXUstd::Clamp(x) * 255.0f)); }

	//! Perform ray-triangle intersection test
	inline void IntersectRayWithTriangle(const RTVector3D& rayOrigin, const RTVector3D& rayDirection,
		const GLTriangle& triangle, const RaytracingSettings& RTS, IntersectionResult& result);

	//! check intersection with list of triangles; 
	//! write information in closestHit structure; 
	//! use minDistanceZ to realize a nearplane, using Z-component of rayDirection
	//! searches at maximum rayLength*rayDirection
	void IntersectRayWithTriangles(const RTVector3D& rayOrigin, const RTVector3D& rayDirection, RTfloat rayLength,
		const ResizableArray<GLTriangle>& triangles, const RaytracingSettings& RTS, IntersectionResult& closestHit,
		bool returnOnSingleHit = false, RTfloat minDistanceZ = std::numeric_limits<RTfloat>::lowest());

	//! check if point is in shadow with given light
	bool IsInShadow(const RTVector3D& point, const RTVector3D& lightDirection, const ResizableArray<GLTriangle>& triangles, 
		const RaytracingSettings& RTS, RTfloat maxDistance);

	//! basic static triangle (colorbar)
	bool IsStaticTriangle(Index triangleIndex) const
	{
		if (triangleIndex > graphicsData.glTriangles.NumberOfItems()) { return false; }

		return graphicsData.glTriangles[triangleIndex].itemID == itemIDstaticObject;
	}

	//! shaded static triangle (arrow)
	bool IsStaticShadedTriangle(Index triangleIndex) const
	{
		if (EXUstd::IndexIsInRange(triangleIndex, 0, graphicsData.glTriangles.NumberOfItems())) { return false; }

		return (graphicsData.glTriangles[triangleIndex].itemID == itemIDstaticObject) || (graphicsData.glTriangles[triangleIndex].itemID == itemIDstaticObjectShaded);
	}

	//refraction for transparency
	RTVector3D Refract(const RTVector3D& I, const RTVector3D& N, RTfloat eta);

	//! compute one pixel color from Raytracing
	RTVector3D ComputePixelColor(const RTVector3D& rayOrigin, const RTVector3D& rayDirection, const ResizableArray<GLTriangle>& triangles,
		const RaytracingSettings& RTS, RTfloat& zDepth, ShadowValue& shadowValue, Index reflectionDepth = 0, Index transparencyDepth = 0,
		Index shadowMode = 0, //0=normal, 1=compute shadow (returns shadow in RTVector3D[0]), 2=use computed shadow
		const RTVector3D& rayOriginNormalized=RTVector3D({0.f,0.f,0.f}));

	//! draw line with points given in modelview
	void RasterizeLine(const RTVector3D& p0, const RTVector3D& p1, const RTVector4D& color0, const RTVector4D& color1,
		ResizableArray<RTfloat>& zdepths, ResizableArray<unsigned char>& pixelsRGBA, const RaytracingSettings& RTS, 
		bool isStaticObject, RTfloat nearPlaneZ);
	void RasterizeCharacter(int charIndex, const RTVector3D& posPixels, const RTVector4D& color,
		RTfloat fontFactor, bool alwaysInFront);

	//! draw a 0-terminated text string with fontSize, including monitor scaling factor; 
	//! position p is in modelview coordinates, like in OpenGL DrawString!
	//! offset is given relative to character width (X), height (Y) and scene Z-size (Z); offset is not rotated
	void DrawString(const char* text, float fontSizeScaled, const RTVector3D& p, const RTVector3D& offset, Float4 color,
		bool transparent = true, bool alwaysInFront = false, bool isStaticObject = false);

	//! use most of openGL settings also for RTS
	void CopyVisSettings2RTS(Index viewID);

	//! print delayed via safe communication with main thread
	void PrintDelayed(const STDstring& str, bool lineFeed = true, bool flush = false)
	{
		if (!RTS.isCalledFromMainThread && (visSettings == NULL || visSettings->general.useMultiThreadedRendering) )
		{
			if (lineFeed)
			{
				//PyQueueExecutableString("print('" + str + "')\n");
				outputBuffer.WriteVisualization(str + '\n');
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


	//! software renderer used instead of OpenGL renderer
	//! render view with viewID;
	//! either draw into existing GlfwClient window or store image in VisualizationSystemContainer
	void SoftwareRenderer(Index viewID, VisualizationSystemContainerBase* basicVisualizationSystemContainerInit, 
		const RenderStateMachine& stateMachine, bool storeImage=false, bool isCalledFromMainThread=false);


};


extern Raytracer raytracer;	//!< currently, use global raytracer to avoid sharing among BasicVisualizationSystemContainer


#endif //include once
