/** ***********************************************************************************************
* @class        VisualizationPrimitives
* @brief		
* @details		Details:
 				- helper classes to draw primitives such as springs, cynlinders, arrows, etc.
*
* @author		Gerstmayr Johannes
* @date			2020-02-08 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
* 				
*
* *** Example code ***
*
************************************************************************************************ */
#pragma once

//#include "Utilities/ReleaseAssert.h"
//#include "Utilities/BasicDefinitions.h" //includes stdoutput.h
//#include "Utilities/BasicFunctions.h"	//includes stdoutput.h
//#include "Linalg/BasicLinalg.h"		//includes Vector.h

//#include "Graphics/VisualizationSystemContainer.h"

class VisualizationSystem; //avoid including Visualization classes

namespace EXUvis {
	//default colors used in some elements, etc.
	const Float4 defaultColorFloat4 = { 0.f,0.f,0.f,1.f }; //default color black, if color is not defined
	const Float4 defaultColorBlue4 = { 0.4f,0.4f,0.9f,1.f }; //default color blue, if color is not defined for triangles

	//colors in exudyn
	const Float4 red = { 0.9f,0.1f,0.1f,1.f };
	const Float4 green = { 0.1f,0.9f,0.1f,1.f };
	const Float4 blue = { 0.1f,0.1f,0.9f,1.f };
	const Float4 cyan = { 0.f,1.f,1.f,1.f };
	const Float4 magenta = { 1.f,0.f,1.f,1.f };
	const Float4 yellow = { 1.f,1.f,0.f,1.f };
	const Float4 black = { 0.01f,0.01f,0.01f,1.f };
	const Float4 white = { 1.f,1.f,1.f,1.f };
	const Float4 grey1 = { 0.3f,0.3f,0.3f,1.f };
	const Float4 grey2 = { 0.55f,0.55f,0.55f,1.f };
	const Float4 grey3 = { 0.8f,0.8f,0.8f,1.f };
	const Float4 orange = { 1.f,0.5f,0.25f,1.f };
	const Float4 lila = { 0.5f,0.5f,1.f,1.f };
	const Float4 rose = { 1.f,0.5f,0.5f,1.f };
	const Float4 brown = { 0.5f,0.25f,0.25f,1.f };

	//! get color from index; used e.g. for axes numeration
	inline const Float4& GetColor(Index i);

	inline float modifyColorFactor = 0.25f; //!< standard value to modify color with ModifyColor
	//! modify a color by a factor: 1=original color, 0=grey; this can be used to visualize axes, joints, ... using one color definition represented in two colors
	inline Float4 ModifyColor(const Float4& color, float colorFactor = 1.f)
	{
		if (colorFactor == 1.f) { return color; }
		else
		{
			Float4 grey({ 0.5,0.5,0.5,color[3] });
			return Float4((1.f - colorFactor)*grey + colorFactor * color);
		}
	}

	//! draw a simple spring in 2D with given endpoints p0,p1 a width, a (normalized) normal vector for the width drawing and number of spring points numberOfPoints
	void DrawSpring2D(const Vector3D& p0, const Vector3D& p1, const Vector3D& vN, Index numberOfPoints, Real width, const Float4& color, GraphicsData& graphicsData);

	//! draw number for item at selected position and with label, such as 'N' for nodes, etc.
	void DrawItemNumber(const Vector3D& pos, VisualizationSystem* vSystem, Index itemNumber, const char* label = "", const Float4& color = Float4({ 0.f,0.f,0.f,1.f }));

	//! add a cylinder to graphicsData with reference point (pAxis0), axis vector (vAxis) and radius using triangle representation
	//! angleRange is used to draw only part of the cylinder; 
	//! if lastFace=true, a closing face is drawn in case of limited angle; 
	//! cutPlain=true: a plain cut through cylinder is made; false: draw the cake shape ...
	void DrawCylinder(const Vector3D& pAxis0, const Vector3D& vAxis, Real radius, const Float4& color, GraphicsData& graphicsData,
		Index nTiles = 12, Real innerRadius = 0, Vector2D angleRange = Vector2D({ 0., 2 * EXUstd::pi }), bool lastFace = true, bool cutPlain = true);

	//! add a cone to graphicsData with reference point (pAxis0), axis vector (vAxis) and radius using triangle representation
	//! cone starts at pAxis0, tip is at pAxis0+vAxis0
	void DrawCone(const Vector3D& pAxis0, const Vector3D& vAxis, Real radius, const Float4& color, GraphicsData& graphicsData, Index nTiles = 12);

	//! draw a sphere with center at p, radius and color; nTiles are in 2 dimensions (8 tiles gives 8x8 x 2 faces)
	void DrawSphere(const Vector3D& p, Real radius, const Float4& color, GraphicsData& graphicsData, Index nTiles = 8);

	//! draw orthonormal basis using a rotation matrix, which transforms local to global coordinates
	//! red=axisX, green=axisY, blue=axisZ
	//! length defines the length of each axis; radius is the radius of the shaft; arrowSize is diameter relative to radius
	//! colorfactor: 1=rgb color, 0=grey color (and any value between)
	void DrawOrthonormalBasis(const Vector3D& p, const Matrix3D& rot, Real length, Real radius, GraphicsData& graphicsData, float colorFactor = 1.f, bool draw3D = true, Index nTiles = 12, Real arrowSizeRelative = 2.5);

	//! draw arraw (for forces, etc.); doubleArrow for torques
	void DrawArrow(const Vector3D& p, const Vector3D& v, Real radius, const Float4& color, GraphicsData& graphicsData, Index nTiles = 12, bool doubleArrow = false);

} //EXUvis
