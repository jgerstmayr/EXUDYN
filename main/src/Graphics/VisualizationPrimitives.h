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
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
************************************************************************************************ */
#ifndef VISUALIZATIONPRIMITIVES__H
#define VISUALIZATIONPRIMITIVES__H

#include "Graphics/VisualizationBasics.h" //for colors

class VisualizationSystem; //avoid including Visualization classes

namespace EXUvis {
	//! compute normalized normal from triangle points; Vector3D version
	Vector3D ComputeTriangleNormal(const std::array<Vector3D, 3>& trigPoints);

	//! compute normalized normal from triangle points; Float3 version
	//void ComputeTriangleNormals(const std::array<Float3, 3>& trigPoints, std::array<Float3, 3>& normals);
	
	//! compute normalized normal from triangle points
	template<class TReal>
	void ComputeTriangleNormals(const std::array<SlimVectorBase<TReal, 3>, 3>& trigPoints, std::array<SlimVectorBase<TReal, 3>, 3>& normals)
	{
		SlimVectorBase<TReal, 3> v1 = trigPoints[1] - trigPoints[0];
		SlimVectorBase<TReal, 3> v2 = trigPoints[2] - trigPoints[0];
		SlimVectorBase<TReal, 3> n = v1.CrossProduct(v2); //@todo: need to check correct outward normal direction in openGL
		TReal len = n.GetL2Norm();
		if (len != 0.f) { n *= 1.f / len; }
		normals[0] = n;
		normals[1] = n;
		normals[2] = n;
	}

	//! draw a simple spring in 2D with given endpoints p0,p1 a width (=2*halfWidth), a (normalized) normal vector for the width drawing and number of spring points numberOfPoints
	void DrawSpring2D(const Vector3D& p0, const Vector3D& p1, const Vector3D& vN, Index numberOfPoints, Real halfWidth, const Float4& color, GraphicsData& graphicsData);

	//! draw a spring in 3D with given endpoints p0,p1, a width, windings and tiling
	void DrawSpring(const Vector3D& p0, const Vector3D& p1, Index numberOfWindings, Index nTilesPerWinding, 
		Real radius, const Float4& color, GraphicsData& graphicsData, bool draw3D = true);

	//! draw number for item at selected position and with label, such as 'N' for nodes, etc.
	void DrawItemNumber(const Vector3D& pos, VisualizationSystem* vSystem, Index itemNumber, const char* label = "", const Float4& color = Float4({ 0.f,0.f,0.f,1.f }));

	//! draw cube with midpoint and size in x,y and z direction
	void DrawOrthoCube(const Vector3D& midPoint, const Vector3D& size, const Float4& color, GraphicsData& graphicsData);
		
	//! add a cylinder to graphicsData with reference point (pAxis0), axis vector (vAxis) and radius using triangle representation
	//! angleRange is used to draw only part of the cylinder; 
	//! if lastFace=true, a closing face is drawn in case of limited angle; 
	//! cutPlain=true: a plain cut through cylinder is made; false: draw the cake shape ...
	void DrawCylinder(const Vector3D& pAxis0, const Vector3D& vAxis, Real radius, const Float4& color, GraphicsData& graphicsData,
		Index nTiles = 12, Real innerRadius = 0, Vector2D angleRange = Vector2D({ 0., 2 * EXUstd::pi }), bool lastFace = true, bool cutPlain = true, bool drawSmooth = true);

	//! add a cone to graphicsData with reference point (pAxis0), axis vector (vAxis) and radius using triangle representation
	//! cone starts at pAxis0, tip is at pAxis0+vAxis0
	void DrawCone(const Vector3D& pAxis0, const Vector3D& vAxis, Real radius, const Float4& color, GraphicsData& graphicsData, Index nTiles = 12, bool drawSmooth = true);

	//! draw a sphere with center at p, radius and color; nTiles are in 2 dimensions (8 tiles gives 8x8 x 2 faces)
	void DrawSphere(const Vector3D& p, Real radius, const Float4& color, GraphicsData& graphicsData, Index nTiles = 8, bool drawSmooth = true);

	//! draw orthonormal basis using a rotation matrix, which transforms local to global coordinates
	//! red=axisX, green=axisY, blue=axisZ
	//! length defines the length of each axis; radius is the radius of the shaft; arrowSize is diameter relative to radius
	//! colorfactor: 1=rgb color, 0=grey color (and any value between)
	void DrawOrthonormalBasis(const Vector3D& p, const Matrix3D& rot, Real length, Real radius, GraphicsData& graphicsData, 
		float colorFactor = 1.f, bool draw3D = true, Index nTiles = 12, Real arrowSizeRelative = 2.5, Index showNumber = EXUstd::InvalidIndex);

	//! draw arraw (for forces, etc.); doubleArrow for torques
	void DrawArrow(const Vector3D& p, const Vector3D& v, Real radius, const Float4& color, GraphicsData& graphicsData, 
		Index nTiles = 12, bool doubleArrow = false, bool draw3D = true);

	//! draw node either with 3 circles or with sphere at given point and with given radius
	void DrawNode(const Vector3D& p, Real radius, const Float4& color, GraphicsData& graphicsData, bool draw3D = true, Index nTiles = 12);
	//! draw marker either with 3 crosses or with cube at given point and with given size
	void DrawMarker(const Vector3D& p, Real size, const Float4& color, GraphicsData& graphicsData, bool draw3D = true);
	//! draw sensor as diamond
	void DrawSensor(const Vector3D& p, Real radius, const Float4& color, GraphicsData& graphicsData, bool draw3D = true);

} //EXUvis

#endif
