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

	//! compute normalized normal from triangle points; for function with single normal see Geometry.h
	template<class TReal>
	void ComputeTriangleNormals(const std::array<SlimVectorBase<TReal, 3>, 3>& trigPoints, std::array<SlimVectorBase<TReal, 3>, 3>& normals)
	{
		//SlimVectorBase<TReal, 3> v1 = trigPoints[1] - trigPoints[0];
		//SlimVectorBase<TReal, 3> v2 = trigPoints[2] - trigPoints[0];
		//SlimVectorBase<TReal, 3> n = v1.CrossProduct(v2); //@todo: need to check correct outward normal direction in openGL
		//TReal len = n.GetL2Norm();
		//if (len != 0.f) { n *= 1.f / len; }
		SlimVectorBase<TReal, 3> n = EXUmath::ComputeTriangleNormal(trigPoints[0], trigPoints[1], trigPoints[2]);

		normals[0] = n;
		normals[1] = n;
		normals[2] = n;
	}

	template<class TVector>
	void ComputeContourColor(const TVector& value, OutputVariableType outputVariableType, SignedIndex outputVariableComponent, Float4& contourColor)
	{
		if (outputVariableComponent == VisualizationSystem::GetContourPlotNormFlag())
		{
			if (!(outputVariableType == OutputVariableType::RotationMatrix || outputVariableType == OutputVariableType::StrainLocal))
			{
				Real contourValue = 0;
				if ((outputVariableType == OutputVariableType::StressLocal) && value.NumberOfItems() == 6)
				{
					Real Sx = value[0];
					Real Sy = value[1];
					Real Sz = value[2];
					Real Syz = value[3];
					Real Sxz = value[4];
					Real Sxy = value[5];
					//add fabs, if there are small roundoff errors which may lead to negative values in sqrt
					contourValue = sqrt(fabs(Sx*Sx + Sy * Sy + Sz * Sz - Sx * Sy - Sx * Sz - Sy * Sz + 3.*(Sxy*Sxy + Sxz * Sxz + Syz * Syz)));
				}
				else
				{
					contourValue = value.GetL2Norm();
				}
				contourColor = Float4({ (float)contourValue, 0.,0., VisualizationSystem::GetContourPlotFlag() });
			}
		}
		else if (outputVariableComponent >= 0 && outputVariableComponent < value.NumberOfItems())
		{
			float contourValue = (float)value[outputVariableComponent];
			contourColor = Float4({ contourValue, 0.,0., VisualizationSystem::GetContourPlotFlag() });
		}
	}

	//! copy bodyGraphicsData (of body) into global graphicsData (of system)
	void AddBodyGraphicsDataColored(const BodyGraphicsData& bodyGraphicsData, GraphicsData& graphicsData, 
		const Float3& position, const Matrix3DF& rotation, const Float3& refPosition, const Matrix3DF& refRotation, const Float3& velocity, const Float3& angularVelocity,
		Index itemID, const VisualizationSettings& visualizationSettings, bool contourColor);

	//! copy bodyGraphicsData (of body) into global graphicsData (of system)
	void AddBodyGraphicsData(const BodyGraphicsData& bodyGraphicsData, GraphicsData& graphicsData, const Float3& position,
		const Matrix3DF& rotation, Index itemID);
	//{
	//	AddBodyGraphicsDataColored(bodyGraphicsData, graphicsData, position, rotation, Float3(), Matrix3DF(), Float3(), Float3(), itemID, visualizationSettings, false);
	//}

	//! draw a simple spring in 2D with given endpoints p0,p1 a width (=2*halfWidth), a (normalized) normal vector for the width drawing and number of spring points numberOfPoints
	void DrawSpring2D(const Vector3D& p0, const Vector3D& p1, const Vector3D& vN, Index numberOfPoints, Real halfWidth, 
		const Float4& color, GraphicsData& graphicsData, Index itemID);

	//! draw a spring in 3D with given endpoints p0,p1, a width, windings and tiling
	void DrawSpring(const Vector3D& p0, const Vector3D& p1, Index numberOfWindings, Index nTilesPerWinding, 
		Real radius, const Float4& color, GraphicsData& graphicsData, Index itemID, bool draw3D = true);

	//! draw number for item at selected position and with label, such as 'N' for nodes, etc.
	void DrawItemNumber(const Float3& pos, VisualizationSystem* vSystem, Index itemID, const char* label = "", const Float4& color = Float4({ 0.f,0.f,0.f,1.f }));
	
	//! draw number for item at selected position and with label, such as 'N' for nodes, etc.
	inline void DrawItemNumber(const Vector3D& pos, VisualizationSystem* vSystem, Index itemID, const char* label = "", const Float4& color = Float4({ 0.f,0.f,0.f,1.f }))
	{
		DrawItemNumber(Float3({ (float)pos[0],(float)pos[1],(float)pos[2] }), vSystem, itemID, label, color);
	}

	////! draw number for item at selected position and with label, such as 'N' for nodes, etc.
	//void DrawItemNumber(const Vector3D& pos, VisualizationSystem* vSystem, Index itemNumber, const char* label = "", const Float4& color = Float4({ 0.f,0.f,0.f,1.f }));

	//! draw cube with midpoint and size in x,y and z direction
	void DrawOrthoCube(const Vector3D& midPoint, const Vector3D& size, const Float4& color, GraphicsData& graphicsData, 
		Index itemID, bool showFaces=true, bool showEdges=false);

	//! add a 3D circle to graphicsData with reference point (pMid) and rotation matrix rot, which serves as a transformation;
	//! points for circles are computed in x/y plane and hereafter rotated by rotation matrix rot
	void DrawCircle(const Vector3D& pMid, const Matrix3D& rot, Real radius, const Float4& color, GraphicsData& graphicsData, Index itemID, Index nTiles);

	//! add a cylinder to graphicsData with reference point (pAxis0), axis vector (vAxis) and radius using triangle representation
	//! angleRange is used to draw only part of the cylinder; 
	//! if lastFace=true, a closing face is drawn in case of limited angle; 
	//! cutPlain=true: a plain cut through cylinder is made; false: draw the cake shape ...
	void DrawCylinder(const Vector3D& pAxis0, const Vector3D& vAxis, Real radius, const Float4& color, GraphicsData& graphicsData, Index itemID,
		Index nTiles = 12, Real innerRadius = 0, Vector2D angleRange = Vector2D({ 0., 2 * EXUstd::pi }), bool lastFace = true, bool cutPlain = true, bool drawSmooth = true);

	//! add a cone to graphicsData with reference point (pAxis0), axis vector (vAxis) and radius using triangle representation
	//! cone starts at pAxis0, tip is at pAxis0+vAxis0
	void DrawCone(const Vector3D& pAxis0, const Vector3D& vAxis, Real radius, const Float4& color, GraphicsData& graphicsData, 
		Index itemID, Index nTiles = 12, bool drawSmooth = true);

	//! draw a sphere with center at p, radius and color; nTiles are in 2 dimensions (8 tiles gives 8x8 x 2 faces)
	void DrawSphere(const Vector3D& p, Real radius, const Float4& color, GraphicsData& graphicsData, 
		Index itemID, Index nTiles = 8, bool drawSmooth = true);

	//! draw orthonormal basis using a rotation matrix, which transforms local to global coordinates
	//! red=axisX, green=axisY, blue=axisZ
	//! length defines the length of each axis; radius is the radius of the shaft; arrowSize is diameter relative to radius
	//! colorfactor: 1=rgb color, 0=grey color (and any value between)
	void DrawOrthonormalBasis(const Vector3D& p, const Matrix3D& rot, Real length, Real radius, GraphicsData& graphicsData, Index itemID,
		float colorFactor = 1.f, bool draw3D = true, Index nTiles = 12, Real arrowSizeRelative = 2.5, Index showNumber = EXUstd::InvalidIndex, const char* preText = nullptr);

	//! draw arraw (for forces, etc.); doubleArrow for torques
	void DrawArrow(const Vector3D& p, const Vector3D& v, Real radius, const Float4& color, GraphicsData& graphicsData, Index itemID,
		Index nTiles = 12, bool doubleArrow = false, bool draw3D = true);

	//! draw node either with 3 circles or with sphere at given point and with given radius
	void DrawNode(const Vector3D& p, Real radius, const Float4& color, GraphicsData& graphicsData, 
		Index itemID, bool draw3D = true, Index nTiles = 12);
	//! draw marker either with 3 crosses or with cube at given point and with given size
	void DrawMarker(const Vector3D& p, Real size, const Float4& color, GraphicsData& graphicsData, 
		Index itemID, bool draw3D = true);
	//! draw sensor as diamond
	void DrawSensor(const Vector3D& p, Real radius, const Float4& color, GraphicsData& graphicsData, 
		Index itemID, bool draw3D = true);

} //EXUvis

#endif
