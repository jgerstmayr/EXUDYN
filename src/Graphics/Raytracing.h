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
#include "Graphics/TextureStore.h"
#include "Graphics/ScopePlot.h"

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

//! the points of a triangle as the intersection test needs them: v0 and the two edges, 36 bytes instead of the 128 of
//! a GLTriangle, in the order of the triangles, so that the traversal stays in the cache (#3474)
struct RTCompactTriangle
{
	RTVector3D v0, edge1, edge2;
};

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
	Index reflectionSamples;					//! reflected rays per pixel on a rough surface; 1 = sharp (#3473)
	Index maxTransparencyDepth;
	Index imageSizeFactor;	// subsampling; faster rendering
	RTfloat zBiasLines;		//!< slight bias to make lines visible over coplanar geometry
	RTfloat zOffsetCamera;	//!< allows to go into scene
	//unchanged for now:
	RTfloat minZ;			//!< for triangles and reflections, etc.
	RTVector3D backgroundColorReflections; //!< for refraction
	RTVector3D backFaceColor; //!< the color of faces seen from their back, if useBackFaceColor (#3275)
	bool useBackFaceColor = false;
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
	RTfloat eyeZ;				//!< perspective: the eye on the view axis, RenderState::ComputePerspectiveFrustum (#3000)
	RTfloat zNear;				//!< perspective: near plane, distance from the eye
	RTfloat zFar;				//!< perspective: far plane, distance from the eye
	RTfloat tanHalfAngle;		//!< perspective: tangent of half the vertical angle of view
	Matrix4DF modelViewRM;	//!< model view matrix, row major (OpenGL is column-major!!!)
	Matrix3DF rotationView;	//!< rotation matrix of modelView, row-major; updated by caller
	RTVector3D translationView; //!< translation of modelView, row-major version; updated by caller
	RTfloat displayScaling;		//!< copy from renderState

	std::array<RaytracingLight, MAX_LIGHTS_GLFW> lights;
	
	Index lightRadiusVariations; //same for all lights
	static constexpr Index materialOffset = 1000;
	static constexpr Index RTcolorDepth = 4;

	SearchTreeBase<RTfloat> searchTree;
	//constexpr; not const: Clamp() takes its arguments by const reference; so passing maxNThreads
	//binds a reference to it and that is an odr-use; which a plain in-class initializer does not
	//satisfy. At -O3 the value is folded and nothing notices; at -O1 the symbol stays undefined and
	//the module does not even load (#2506). constexpr is implicitly inline in C++17.
	static constexpr Index maxNThreads = 256;
	mutable ArrayIndex tempTrigIndices[maxNThreads];
	mutable ArrayIndex tempSearchTreeBins[maxNThreads];

	//! the search tree as one flat array (CSR): the items of cell g are binItems[binOffsets[g]] ... binItems[binOffsets[g+1]-1],
	//! and the triangles in their compact form (#3474)
	ArrayIndex binOffsets;
	ArrayIndex binItems;
	ResizableArray<RTCompactTriangle> compactTriangles;
	//! per thread and light the item that blocked the last shadow ray, tested first by the next one; -1 = none (#3474)
	mutable Index lastOccluder[maxNThreads][MAX_LIGHTS_GLFW];

	//! some temporary structures for rendering:
	ResizableArray<unsigned char> finalImageAA; //! for antializing, final smoothed image
	ResizableArray<unsigned char> pixelsRGBA;   //! for raytracing, the RGBA pixel buffer
	ResizableArray<RTfloat> zDepths;			//! for raytracing, storing zDepths for lateron adding texts
	ResizableArray<ShadowValue> shadowValues;	//! shadow values, per light, at possibly lower resolution
	ResizableArray<ShadowValue> smoothShadowValues; //! the second array of the smoothing passes at reduced resolution
	ResizableArray<RTfloat> shadowDepths;		//! the depths of the pixels of reduced resolution, for the depth-aware filter (#3477)
	RTfloat shadowDepthTolerance;				//! a depth difference of this size halves the weight of a neighbour (#3477)
	RTfloat shadowRayOffset;					//! a shadow ray starts this far from the hit, relative to the scene size (#3470)
	Index adaptiveShadowSamples;				//! the probe samples per pixel and light; 0 = all samples always (#3476)
	RTfloat minimumShadowWeight;				//! a secondary hit of a smaller weight casts no shadow rays (#3486)
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
		shadowDepthTolerance = 1.f;
		shadowRayOffset = 0.01f;
		adaptiveShadowSamples = 0;
		minimumShadowWeight = 0.f;
		maxReflectionDepth = 0;
		reflectionSamples = 1;
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
		eyeZ = 0.f;
		zNear = 0.1f;
		zFar = 1.f;
		tanHalfAngle = 1.f;
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
	//! the textures of the triangles of graphicsData (#3034): per triangle -1 or an index into texturedTriangles,
	//! empty if no triangle has a texture; the textures copied from the TextureStore by their number
	ResizableArray<Index> triangleTextures;
	ResizableArray<GLTexturedTriangle> texturedTriangles;
	std::vector<Texture> textures;
GraphicsData graphicsDataStatic;	//!< static GraphicsData objects (info, Exudyn, etc.)

public:
	Raytracer();
	virtual ~Raytracer();

	unsigned char FloatColorToByte(RTfloat x) { return static_cast<unsigned char>((Index)(EXUstd::Clamp(x) * 255.0f)); }

	//! ray-triangle intersection, the geometry only: true and t, u, v if the ray hits the triangle (#3468)
	inline bool IntersectRayWithTriangleGeometry(const RTVector3D& rayOrigin, const RTVector3D& rayDirection,
		const RTCompactTriangle& triangle, const RaytracingSettings& RTS, RTfloat& t, RTfloat& u, RTfloat& v);

	//! the shading data of a hit of a triangle at u, v: normal, color, material, alpha and the side; computed only for a
	//! hit that is closer than the closest one so far (#3468)
	inline void ShadeTriangleHit(const RTVector3D& rayDirection, const GLTriangle& triangle, const RaytracingSettings& RTS,
		RTfloat t, RTfloat u, RTfloat v, IntersectionResult& result);

	//! ray-sphere intersection, with the exact normal (#2709)
	inline void IntersectRayWithSphere(const RTVector3D& rayOrigin, const RTVector3D& rayDirection,
		const GLSphere& sphere, const RaytracingSettings& RTS, IntersectionResult& result);

	//! the item iInd of the search tree: a triangle, or with iInd >= triangles.NumberOfItems() a sphere of graphicsData.glSpheres;
	//! true and the shaded hit in result if it is hit closer than closestDistance and not before minDistanceZ (#3468)
	inline bool IntersectRayWithItem(const RTVector3D& rayOrigin, const RTVector3D& rayDirection, Index iInd,
		const ResizableArray<GLTriangle>& triangles, const RaytracingSettings& RTS, RTfloat closestDistance, RTfloat minDistanceZ,
		IntersectionResult& result);

	//! the color of a textured triangle at the hit (#3034): multiplied with the texture or replaced by it, the alpha of
	//! RGBA multiplied, and no hit where the mask of RGBAMask is below 0.5 - also for shadows
	void ApplyTexture(Index texturedIndex, IntersectionResult& result) const;

	//! check intersection with list of triangles; 
	//! write information in closestHit structure; 
	//! use minDistanceZ to realize a nearplane, using Z-component of rayDirection
	//! searches at maximum rayLength*rayDirection
	void IntersectRayWithTriangles(const RTVector3D& rayOrigin, const RTVector3D& rayDirection, RTfloat rayLength,
		const ResizableArray<GLTriangle>& triangles, const RaytracingSettings& RTS, IntersectionResult& closestHit,
		bool returnOnSingleHit = false, RTfloat minDistanceZ = std::numeric_limits<RTfloat>::lowest());

	//! the cells of the grid crossed by a ray, in their order, until visit(item) returns true or the next cell begins
	//! beyond cutOffDistance, which visit may decrease; each item once per ray as far as the mailbox remembers (#3468, #3474)
	template<class TVisit>
	void TraverseGrid(const RTVector3D& rayOrigin, const RTVector3D& rayDirection, const RaytracingSettings& RTS,
		const RTfloat& cutOffDistance, const TVisit& visit) const;

	//! the fraction of light lightID that reaches point: 0 behind an opaque item, the product of (1 - alpha) of the
	//! transparent items in between, 1 without any; the item that blocked the last shadow ray of this thread and light is
	//! tested first (#3474, #3471)
	RTfloat ShadowTransmission(const RTVector3D& point, const RTVector3D& lightDirection, const ResizableArray<GLTriangle>& triangles,
		const RaytracingSettings& RTS, RTfloat maxDistance, Index lightID);

	//! the precomputed shadow at a pixel of full resolution (normalized coordinates in [-1, 1]) and its depth: the four
	//! nearest values of reduced resolution, weighted bilinearly and by the similarity of their depths (#3477)
	ShadowValue UpsampleShadow(const RTVector3D& pixelNormalized, RTfloat zDepth) const;

	//! the flat search tree and the compact triangles from RTS.searchTree and the triangles (#3474)
	void BuildFlatSearchTree(const ResizableArray<GLTriangle>& triangles);

	//! basic static triangle (colorbar)
	bool IsStaticTriangle(Index triangleIndex) const
	{
		if (!EXUstd::IndexIsInRange(triangleIndex, 0, graphicsData.glTriangles.NumberOfItems())) { return false; } //spheres come after the triangles

		return graphicsData.glTriangles[triangleIndex].itemID == itemIDstaticObject;
	}

	//! shaded static triangle (arrow)
	bool IsStaticShadedTriangle(Index triangleIndex) const
	{
		if (!EXUstd::IndexIsInRange(triangleIndex, 0, graphicsData.glTriangles.NumberOfItems())) { return false; } //spheres come after the triangles

		return (graphicsData.glTriangles[triangleIndex].itemID == itemIDstaticObject) || (graphicsData.glTriangles[triangleIndex].itemID == itemIDstaticObjectShaded);
	}

	//refraction for transparency
	RTVector3D Refract(const RTVector3D& I, const RTVector3D& N, RTfloat eta);

	//! compute one pixel color from Raytracing
	RTVector3D ComputePixelColor(const RTVector3D& rayOrigin, const RTVector3D& rayDirection, const ResizableArray<GLTriangle>& triangles,
		const RaytracingSettings& RTS, RTfloat& zDepth, ShadowValue& shadowValue, Index reflectionDepth = 0, Index transparencyDepth = 0,
		Index shadowMode = 0, //0=normal, 1=compute shadow (returns shadow in RTVector3D[0]), 2=use computed shadow
		const RTVector3D& rayOriginNormalized=RTVector3D({0.f,0.f,0.f}),
		RTfloat weight = 1.f); //the share of this ray in the color of the pixel, for minimumShadowWeight (#3486)

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

	//! the primitives of the scopes, in pixels of the window, drawn into the image (#3237)
	void DrawScopes(const ScopePlot::Graphics& scopeGraphics, Index width, Index height, RTfloat nearPlaneZ);

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

	//! the sensor traces of visualizationSettings.sensors.traces as lines - position trace, vectors and triads -, global;
	//! the OpenGL renderer and the raytracer draw the same lines (#2877)
	//! viewID >= 0: none if the system of the traces is hidden in that view (#3442)
	static void SensorTraceLines(VisualizationSystemContainerBase* visualizationSystemContainer, const VisualizationSettings& visSettings,
		ResizableArray<GLLine>& lines, Index viewID = -1);


};


extern Raytracer raytracer;	//!< currently, use global raytracer to avoid sharing among BasicVisualizationSystemContainer


#endif //include once
