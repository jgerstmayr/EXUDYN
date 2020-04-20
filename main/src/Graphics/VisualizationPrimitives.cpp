/** ***********************************************************************************************
* @class        VisualizationPrimitives implementation
* @brief		
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

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h" //includes stdoutput.h
#include "Utilities/BasicFunctions.h"	//includes stdoutput.h
#include "Linalg/BasicLinalg.h"		//includes Vector.h

#include "Graphics/VisualizationSystemContainer.h"
#include "Graphics/VisualizationPrimitives.h"

//! class used to define standard colors
namespace EXUvis {

	//! get color from index; used e.g. for axes numeration
	const Float4& GetColor(Index i)
	{
		switch (i)
		{
		case 0: return red;
		case 1: return green;
		case 2: return blue;
		case 3: return cyan;
		case 4: return magenta;
		case 5: return yellow;
		case 6: return black;
		case 7: return orange;
		case 8: return lila;
		case 9: return grey2;
		default: return rose; //any other number gives this color
		}
	}

	//! compute normalized normal from triangle points, Vector3D version
	Vector3D ComputeTriangleNormal(const std::array<Vector3D, 3>& trigPoints)
	{
		Vector3D v1 = trigPoints[1] - trigPoints[0];
		Vector3D v2 = trigPoints[2] - trigPoints[0];
		Vector3D n = v1.CrossProduct(v2); //@todo: need to check correct outward normal direction in openGL
		Real len = n.GetL2Norm();
		if (len != 0.f) { n *= 1.f / len; }
		return n;
	}

	//! draw a simple spring in 2D with given endpoints p0,p1 a width, a (normalized) normal vector for the width drawing and number of spring points numberOfPoints
	void DrawSpring2D(const Vector3D& p0, const Vector3D& p1, const Vector3D& vN, Index numberOfPoints, Real halfWidth, const Float4& color, GraphicsData& graphicsData)
	{
		//2D drawing in XY plane
		Vector3D v0 = p1 - p0;

		Real L = v0.GetL2Norm(); //length of spring
		Real d = L / (Real)numberOfPoints; //split spring into pieces: shaft, (n-2) parts, end
		if (L != 0.f) { v0 /= L; }

		Vector3D pLast; //for last drawn point
		for (Index i = 0; i <= numberOfPoints; i++)
		{
			Vector3D pAct = p0 + v0 * (float)i*d;
			Real sign = (Real)(i % 2); //sign
			if (i > 1 && i < numberOfPoints - 1) { pAct += halfWidth * (sign*2.f - 1.f)* vN; }

			if (i > 0)
			{
				graphicsData.AddLine(pLast, pAct, color, color);
			}

			pLast = pAct;
		}

	}

	//! draw a spring in 3D with given endpoints p0,p1, a width, windings and tiling
	void DrawSpring(const Vector3D& p0, const Vector3D& p1, Index numberOfWindings, Index nTilesPerWinding,
		Real radius, const Float4& color, GraphicsData& graphicsData, bool draw3D)
	{
		Vector3D v0 = p1 - p0;

		Real L = v0.GetL2Norm(); //length of spring
		Real d = L / (Real)numberOfWindings; //split spring into pieces: shaft, (n-2) parts, end
		if (L != 0.f) 
		{ 
			v0 /= L;
			Vector3D n1, n2;
			EXUmath::ComputeOrthogonalBasis(v0, n1, n2);

			Vector3D pLast = p0;

			for (Index i = 0; i < numberOfWindings; i++)
			{
				for (Index j = 0; j < nTilesPerWinding; j++)
				{
					Real phi = 2 * EXUstd::pi * j / (Real)nTilesPerWinding;
					Vector3D p = p0 + d * ((Real)i + j / (Real)nTilesPerWinding)*v0 + radius*sin(phi)*n1 + radius*cos(phi)*n2;

					graphicsData.AddLine(pLast, p, color, color);
					pLast = p;
				}
			}
			graphicsData.AddLine(pLast, p1, color, color);

		}

	}

	//! draw number for item at selected position and with label, such as 'N' for nodes, etc.
	void DrawItemNumber(const Vector3D& pos, VisualizationSystem* vSystem, Index itemNumber, const char* label, const Float4& color)
	{
		float offx = 0.25f; //in text coordinates, relative to textsize
		float offy = 0.25f; //in text coordinates, relative to textsize
		float textSize = 0.f; //use default value
		vSystem->graphicsData.AddText(pos, color, label + EXUstd::ToString(itemNumber), textSize, offx, offy);
	}



	//! add a cylinder to graphicsData with reference point (pAxis0), axis vector (vAxis) and radius using triangle representation
	//! angleRange is used to draw only part of the cylinder; 
	//! if lastFace=true, a closing face is drawn in case of limited angle; 
	//! cutPlain=true: a plain cut through cylinder is made; false: draw the cake shape ...
	//! innerRadius: if > 0, then this is a cylinder with a hole
	void DrawCylinder(const Vector3D& pAxis0, const Vector3D& vAxis, Real radius, const Float4& color, GraphicsData& graphicsData,
		Index nTiles, Real innerRadius, Vector2D angleRange, bool lastFace, bool cutPlain, bool drawSmooth)
	{
		if (nTiles < 2) { nTiles = 2; } //less than 2 tiles makes no sense
		if (radius <= 0.) { return; } //just a line
		if (vAxis.GetL2NormSquared() == 0.) { return; } //too short

		//create points at left and right face
		//points0 = copy.deepcopy(pAxis) #[pAxis[0], pAxis[1], pAxis[2]] #avoid change of pAxis
		Vector3D pAxis1 = pAxis0 + vAxis;

		Vector3D basisN1, basisN2;
		EXUmath::ComputeOrthogonalBasis(vAxis, basisN1, basisN2);

		//#create normals at left and right face(pointing inwards)
		Real alpha = angleRange[1] - angleRange[0]; //angular range
		Real alpha0 = angleRange[0];

		Real fact = (Real)nTiles; //#create correct part of cylinder (closed/not closed
		if (alpha < 2.*EXUstd::pi) { fact = (Real)(nTiles - 1); } 

		std::array<Vector3D, 3> points;
		std::array<Vector3D, 3> normals;
		std::array<Float4, 3> colors({color,color,color}); //all triangles have same color

		Vector3D nF1 = vAxis;
		nF1.Normalize();

		std::array<Vector3D, 3> normalsFace0({ nF1,nF1,nF1 });
		nF1 = -nF1;
		std::array<Vector3D, 3> normalsFace1({ nF1,nF1,nF1 });
		Vector3D n0(0);
		Vector3D n1(0);


		for (Index i = 0; i < nTiles; i++)
		{
			Real phi0 = alpha0 + i * alpha / fact;
			Real phi1 = alpha0 + (i+1) * alpha / fact;

			Real x0 = radius * sin(phi0);
			Real y0 = radius * cos(phi0);
			Real x1 = radius * sin(phi1);
			Real y1 = radius * cos(phi1);
			Vector3D vv0 = x0 * basisN1 + y0 * basisN2;
			Vector3D vv1 = x1 * basisN1 + y1 * basisN2;
			Vector3D pzL0 = pAxis0 + vv0;
			Vector3D pzL1 = pAxis0 + vv1;
			Vector3D pzR0 = pAxis1 + vv0;
			Vector3D pzR1 = pAxis1 + vv1;
			if (drawSmooth)
			{
				n0 = -vv0;
				n1 = -vv1;
				n0.Normalize();
				n1.Normalize();
			}

			//+++++++++++++++++++++++++++++++
			//circumference:
			normals[0] = n0;
			normals[1] = n1;
			normals[2] = n0;
			points[0] = pzL0;
			points[1] = pzR1;
			points[2] = pzR0;
			graphicsData.AddTriangle(points, normals, colors);

			//normals[0] = n0;
			//normals[1] = n1;
			normals[2] = n1;
			//points[0] = pzL0;
			points[1] = pzL1;
			points[2] = pzR1;
			graphicsData.AddTriangle(points, normals, colors);

			if (innerRadius > 0.)
			{
				Vector3D pzL0i = pAxis0 + innerRadius * vv0;
				Vector3D pzL1i = pAxis0 + innerRadius * vv1;
				Vector3D pzR0i = pAxis1 + innerRadius * vv0;
				Vector3D pzR1i = pAxis1 + innerRadius * vv1;

				//+++++++++++++++++++++++++++++++
				//circumference:
				normals[0] = -n0;
				normals[1] = -n1;
				normals[2] = -n0;
				points[0] = pzL0i;
				points[1] = pzR0i;
				points[2] = pzR1i;
				graphicsData.AddTriangle(points, normals, colors);

				//normals[0] = -n0;
				//normals[1] = -n1;
				normals[2] = -n1;
				//points[0] = pzL0i;
				points[1] = pzR1i;
				points[2] = pzL1i;
				graphicsData.AddTriangle(points, normals, colors);

				//+++++++++++++++++++++++++++++++
				//side faces:
				points[0] = pzL0i;
				points[1] = pzL1;
				points[2] = pzL0;
				graphicsData.AddTriangle(points, normalsFace0, colors);
				points[0] = pzL0i;
				points[1] = pzL1i;
				points[2] = pzL1;
				graphicsData.AddTriangle(points, normalsFace0, colors);

				points[0] = pzR0i;
				points[1] = pzR0;
				points[2] = pzR1;
				graphicsData.AddTriangle(points, normalsFace1, colors);

				points[0] = pzR1i;
				points[1] = pzR0i;
				points[2] = pzR1;
				graphicsData.AddTriangle(points, normalsFace1, colors);
			}
			else
			{
				//+++++++++++++++++++++++++++++++
				//side faces:
				points[0] = pAxis0;
				points[1] = pzL1;
				points[2] = pzL0;
				graphicsData.AddTriangle(points, normalsFace0, colors);

				points[0] = pAxis1;
				points[1] = pzR0;
				points[2] = pzR1;
				graphicsData.AddTriangle(points, normalsFace1, colors);
			}
		}
	}

	//! draw a sphere with center at p, radius and color; nTiles are in 2 dimensions (8 tiles gives 8x8 x 2 faces)
	void DrawSphere(const Vector3D& p, Real radius, const Float4& color, GraphicsData& graphicsData, Index nTiles, bool drawSmooth)
	{
		if (nTiles < 2) { nTiles = 2; } //less than 2 tiles makes no sense
		if (radius <= 0.) { return; } //not visible
		
		std::array<Vector3D, 3> points;
		std::array<Vector3D, 3> normals; // = { Vector3D(0), Vector3D(0), Vector3D(0) };
		std::array<Float4, 3> colors({ color,color,color }); //all triangles have same color

		Index nTiles2 = 2 * nTiles;
		//create points for circles around z - axis with tiling
		for (Index i0 = 0; i0 < nTiles; i0++) //nTiles+1 in python, when generating points
		{
			for (Index iphi = 0; iphi < nTiles2; iphi++)
			{
				Real z0 = -radius * cos(EXUstd::pi * (Real)i0 / (Real)nTiles);    //runs from - r ..r(this is the coordinate of the axis of circles)
				Real fact0 = sin(EXUstd::pi*(Real)i0 / (Real)nTiles);
				Real z1 = -radius * cos(EXUstd::pi * (Real)(i0+1) / (Real)nTiles);    //runs from - r ..r(this is the coordinate of the axis of circles)
				Real fact1 = sin(EXUstd::pi*(Real)(i0 + 1) / (Real)nTiles);

				Real phiA = 2. * EXUstd::pi * (Real)iphi / (Real)nTiles2; //angle
				Real phiB = 2. * EXUstd::pi * (Real)(iphi+1) / (Real)nTiles2; //angle

				Real x0A = fact0 * radius * sin(phiA);
				Real y0A = fact0 * radius * cos(phiA);
				Real x1A = fact1 * radius * sin(phiA);
				Real y1A = fact1 * radius * cos(phiA);
				Real x0B = fact0 * radius * sin(phiB);
				Real y0B = fact0 * radius * cos(phiB);
				Real x1B = fact1 * radius * sin(phiB);
				Real y1B = fact1 * radius * cos(phiB);

				Vector3D v0A({ x0A, y0A, z0 });
				Vector3D v1A({ x1A, y1A, z1 });
				Vector3D v0B({ x0B, y0B, z0 });
				Vector3D v1B({ x1B, y1B, z1 });

				points[0] = p + v0A;
				points[1] = p + v1A;
				points[2] = p + v1B;
				//triangle1: 0A, 1B, 1A
				if (drawSmooth)
				{
					normals[0] = -v0A;
					normals[1] = -v1A;
					normals[2] = -v1B;
					normals[0].Normalize();
					normals[1].Normalize();
					normals[2].Normalize();
				}
				else
				{
					ComputeTriangleNormals(points, normals);
				}
				graphicsData.AddTriangle(points, normals, colors);

				points[0] = p + v0A;
				points[1] = p + v0B;
				points[2] = p + v1B;
				//triangle1: 0A, 0B, 1B
				if (drawSmooth)
				{
					normals[0] = -v0A;
					normals[1] = -v0B;
					normals[2] = -v1B;
					normals[0].Normalize();
					normals[1].Normalize();
					normals[2].Normalize();
				}
				else
				{
					ComputeTriangleNormals(points, normals);
				}
				graphicsData.AddTriangle(points, normals, colors);
			}
		}
	}

	//! draw cube with midpoint and size in x,y and z direction
	void DrawOrthoCube(const Vector3D& midPoint, const Vector3D& size, const Float4& color, GraphicsData& graphicsData)
	{
		//sketch of cube: (z goes upwards from node 1 to node 5)
		// bottom :         top:
		// ^ y				^ y
		// |				|
		// 3---2			7---6
		// |   |			|   |
		// |   |			|   |
		// 0---1--> x		4---5--> x

		//std::array<SlimArray<Index,3>, 12> //does not work with recursive initializer list
		const Index nTrigs = 12;
		//Index trigList[nTrigs][3] = { {0, 1, 2}, {0, 2, 3},  {6, 5, 4}, {6, 4, 7},  {0, 4, 1}, {1, 4, 5},  {1, 5, 2}, {2, 5, 6},  {2, 6, 3}, {3, 6, 7},  {3, 7, 0}, {0, 7, 4} };

		SlimVectorBase<Index, 12*3> trigList = { 0, 1, 2, 0, 2, 3, 6, 5, 4, 6, 4, 7, 0, 4, 1, 1, 4, 5, 1, 5, 2, 2, 5, 6, 2, 6, 3, 3, 6, 7, 3, 7, 0, 0, 7, 4 };

		Real x = 0.5*size[0];
		Real y = 0.5*size[1];
		Real z = 0.5*size[2];

		SlimVectorBase<Vector3D, 8> pc = { Vector3D({-x,-y,-z}), Vector3D({ x,-y,-z}), Vector3D({ x, y,-z}), Vector3D({-x, y,-z}),
										   Vector3D({-x,-y, z}), Vector3D({ x,-y, z}), Vector3D({ x, y, z}), Vector3D({-x, y, z}) }; //cube corner points

		for (Vector3D& point : pc)
		{
			point += midPoint;
		}

		std::array<Vector3D, 3> points;
		std::array<Vector3D, 3> normals = { Vector3D(0), Vector3D(0), Vector3D(0) };
		SlimVectorBase<Float4, 3> colors({ color,color,color }); //all triangles have same color
		//std::array<Vector3D, 8> pc = { Vector3D({-x,-y,-z}), Vector3D({ x,-y,-z}), Vector3D({ x, y,-z}), Vector3D({-x, y,-z}),
		//							   Vector3D({-x,-y, z}), Vector3D({ x,-y, z}), Vector3D({ x, y, z}), Vector3D({-x, y, z}) }; //cube corner points

		//std::array<Vector3D, 3> points;
		//std::array<Vector3D, 3> normals = { Vector3D(0), Vector3D(0), Vector3D(0) };
		//std::array<Float4, 3> colors({ color,color,color }); //all triangles have same color

		for (Index i = 0; i < nTrigs; i++)
		{
			points[0] = pc[trigList[i*3+0]];
			points[1] = pc[trigList[i*3+1]];
			points[2] = pc[trigList[i*3+2]];
			//points[0] = pc[trigList[i][0]];
			//points[1] = pc[trigList[i][1]];
			//points[2] = pc[trigList[i][2]];
			ComputeTriangleNormals(points, normals);
			graphicsData.AddTriangle(points, normals, colors);
		}
	}


	//! add a cone to graphicsData with reference point (pAxis0), axis vector (vAxis) and radius using triangle representation
	//! cone starts at pAxis0, tip is at pAxis0+vAxis0
	void DrawCone(const Vector3D& pAxis0, const Vector3D& vAxis, Real radius, const Float4& color, GraphicsData& graphicsData, Index nTiles, bool drawSmooth)
	{
		if (nTiles < 2) { nTiles = 2; } //less than 2 tiles makes no sense
		if (radius <= 0.) { return; } //just a line
		Real axisLength = vAxis.GetL2Norm();
		if (axisLength == 0.) { return; } //too short

		//create points at left and right face
		//points0 = copy.deepcopy(pAxis) #[pAxis[0], pAxis[1], pAxis[2]] #avoid change of pAxis
		Vector3D pAxis1 = pAxis0 + vAxis;

		Vector3D basisN1, basisN2;
		EXUmath::ComputeOrthogonalBasis(vAxis, basisN1, basisN2);

		//#create normals at left and right face(pointing inwards)
		Real alpha = 2.*EXUstd::pi;

		Real fact = (Real)nTiles; //#create correct part of cylinder (closed/not closed

		std::array<Vector3D, 3> points;
		std::array<Vector3D, 3> normals = { Vector3D(0), Vector3D(0), Vector3D(0) };
		std::array<Float4, 3> colors({ color,color,color }); //all triangles have same color

		Vector3D nF0 = vAxis;
		nF0.Normalize();

		std::array<Vector3D, 3> normalsFace0({ nF0,nF0,nF0 });

		for (Index i = 0; i < nTiles; i++)
		{
			Real phi0 = i * alpha / fact;
			Real phi1 = (i + 1) * alpha / fact;

			Real x0 = radius * sin(phi0);
			Real y0 = radius * cos(phi0);
			Real x1 = radius * sin(phi1);
			Real y1 = radius * cos(phi1);
			Vector3D vv0 = x0 * basisN1 + y0 * basisN2;
			Vector3D vv1 = x1 * basisN1 + y1 * basisN2;
			Vector3D pzL0 = pAxis0 + vv0;
			Vector3D pzL1 = pAxis0 + vv1;

			//+++++++++++++++++++++++++++++++
			//circumference:
			if (drawSmooth)
			{
				//normal to cone surface:
				Vector3D n0 = (-axisLength / radius)*vv0 + radius * nF0;
				Vector3D n1 = (-axisLength / radius)*vv1 + radius * nF0;
				n0.Normalize();
				n1.Normalize();

				normals[0] = n0;
				normals[1] = n1;
				normals[2] = n1;
			}
			points[0] = pzL0;
			points[1] = pzL1;
			points[2] = pAxis1;
			graphicsData.AddTriangle(points, normals, colors);

			//+++++++++++++++++++++++++++++++
			//side faces:
			points[0] = pAxis0;
			points[1] = pzL1;
			points[2] = pzL0;
			graphicsData.AddTriangle(points, normalsFace0, colors);
		}
	}

	//! draw orthonormal basis at point p using a rotation matrix, which transforms local to global coordinates
	//! red=axisX, green=axisY, blue=axisZ
	//! length defines the length of each axis; radius is the radius of the shaft; arrowSize is diameter relative to radius
	//! colorfactor: 1=rgb color, 0=grey color (and any value between)
	void DrawOrthonormalBasis(const Vector3D& p, const Matrix3D& rot, Real length, Real radius, 
		GraphicsData& graphicsData, float colorFactor, bool draw3D, Index nTiles, Real arrowSizeRelative, Index showNumber)
	{

		for (Index i = 0; i < 3; i++)
		{
			Vector3D v = rot.GetColumnVector<3>(i);
			Float4 color(ModifyColor(GetColor(i), colorFactor));
			if (draw3D)
			{
				DrawCylinder(p, length*v, radius, color, graphicsData, nTiles);
				DrawCone(p + length * v, (radius*arrowSizeRelative * 3)*v, arrowSizeRelative*radius, color, graphicsData, nTiles);
			} else //draw as simple line
			{
				graphicsData.AddLine(p, p + length * v, color, color);
			}
			if (showNumber != EXUstd::InvalidIndex)
			{
				graphicsData.AddText(p + (length + radius*arrowSizeRelative * 3) * v, color, EXUstd::ToString(showNumber), 0.f, 0.25f, 0.25f);
			}
		}
	}
	void DrawArrow(const Vector3D& p, const Vector3D& v, Real radius, const Float4& color, GraphicsData& graphicsData, 
		Index nTiles, bool doubleArrow, bool draw3D)
	{
		Real arrowSizeRelative = 2.5;
		Real len = v.GetL2Norm();

		if (len != 0)
		{
			Vector3D v0 = (1. / len)*v;

			if (!draw3D) //draw simplified vector
			{
				Vector3D v1 = (len - 3 * radius * arrowSizeRelative)*v0;
				Vector3D n1, n2;
				EXUmath::ComputeOrthogonalBasis(v0, n1, n2);

				graphicsData.AddLine(p, p + v, color, color);
				graphicsData.AddLine(p + v, p + v1 + radius * n1, color, color);
				graphicsData.AddLine(p + v, p + v1 - radius * n1, color, color);
				graphicsData.AddLine(p + v, p + v1 + radius * n2, color, color);
				graphicsData.AddLine(p + v, p + v1 - radius * n2, color, color);

				if (doubleArrow)
				{
					Vector3D v2 = (len - 2*3 * radius * arrowSizeRelative)*v0;

					graphicsData.AddLine(p + v1, p + v2 + radius * n1, color, color);
					graphicsData.AddLine(p + v1, p + v2 - radius * n1, color, color);
					graphicsData.AddLine(p + v1, p + v2 + radius * n2, color, color);
					graphicsData.AddLine(p + v1, p + v2 - radius * n2, color, color);
				}
			}
			else
			{
				if (!doubleArrow)
				{
					Vector3D v1 = (len - 3 * radius * arrowSizeRelative)*v0;
					DrawCylinder(p, v1, radius, color, graphicsData, nTiles);
					DrawCone(p + v1, (3 * radius * arrowSizeRelative) * v0, arrowSizeRelative*radius, color, graphicsData, nTiles);
				}
				else
				{
					Vector3D v1 = (len - 2 * 3 * radius * arrowSizeRelative)*v0;
					DrawCylinder(p, v1, radius, color, graphicsData, nTiles);
					DrawCone(p + v1, (3 * radius * arrowSizeRelative) * v0, arrowSizeRelative*radius, color, graphicsData, nTiles);
					DrawCone(p + v1 + (3 * radius * arrowSizeRelative) * v0, (3 * radius * arrowSizeRelative) * v0, arrowSizeRelative*radius, color, graphicsData, nTiles);
				}
			}
		}
	}

	//! draw node either with 3 circles or with sphere at given point and with given radius
	void DrawNode(const Vector3D& p, Real radius, const Float4& color, GraphicsData& graphicsData, bool draw3D, Index nTiles)
	{
		if (nTiles == 0)
		{
			graphicsData.AddPoint(p, color);
		}
		else if (draw3D)
		{
			DrawSphere(p, radius, color, graphicsData, nTiles);
		}
		else
		{
			Vector3D pPrevious[3]; //end points of previous segment
			Vector3D pAct[3];
			for (Index i = 0; i <= nTiles; i++)
			{
				Real phi = (Real)i / (Real)nTiles * 2. * EXUstd::pi;
				Real x = radius * sin(phi);
				Real y = radius * cos(phi);

				pAct[0] = p + Vector3D({ 0,x,y });
				pAct[1] = p + Vector3D({ x,0,y });
				pAct[2] = p + Vector3D({ x,y,0 });

				if (i > 0)
				{
					for (Index j = 0; j < 3; j++)
					{
						graphicsData.AddLine(pAct[j],pPrevious[j],color,color);
					}
				}
				for (Index j = 0; j < 3; j++)
				{
					pPrevious[j] = pAct[j];
				}
			}
		}
	}

	//! draw marker either with 3 crosses or with cube at given point and with given size
	void DrawMarker(const Vector3D& p, Real size, const Float4& color, GraphicsData& graphicsData, bool draw3D)
	{
		if (draw3D)
		{
			DrawOrthoCube(p, Vector3D({ size,size,size }), color, graphicsData);
			//DrawSphere(p, size, color, graphicsData, 2, false); //draw coarse and with flat shading
		}
		else
		{
			Real s = 0.5*size;
			graphicsData.AddLine(p + Vector3D({ s,s,0 }), p - Vector3D({ s,s,0 }), color, color);
			graphicsData.AddLine(p + Vector3D({ -s,s,0 }), p - Vector3D({ -s,s,0 }), color, color);

			graphicsData.AddLine(p + Vector3D({ s,0,s }), p - Vector3D({ s,0,s }), color, color);
			graphicsData.AddLine(p + Vector3D({ -s,0,s }), p - Vector3D({ -s,0,s }), color, color);

			graphicsData.AddLine(p + Vector3D({ 0,s,s }), p - Vector3D({ 0,s,s }), color, color);
			graphicsData.AddLine(p + Vector3D({ 0,-s,s }), p - Vector3D({ 0,-s,s }), color, color);

		}
	}

	//! draw sensor as diamond
	void DrawSensor(const Vector3D& p, Real radius, const Float4& color, GraphicsData& graphicsData, bool draw3D)
	{
		if (draw3D)
		{
			DrawSphere(p, radius, color, graphicsData, 2, false); //draw coarse and with flat shading
		}
		else
		{
			Real s = radius;
			graphicsData.AddLine(p + Vector3D({ s,0,0 }), p - Vector3D({ 0, s,0 }), color, color);
			graphicsData.AddLine(p + Vector3D({ s,0,0 }), p - Vector3D({ 0,-s,0 }), color, color);
			graphicsData.AddLine(p + Vector3D({ -s,0,0 }), p - Vector3D({ 0, s,0 }), color, color);
			graphicsData.AddLine(p + Vector3D({ -s,0,0 }), p - Vector3D({ 0,-s,0 }), color, color);

			graphicsData.AddLine(p + Vector3D({ s,0,0 }), p - Vector3D({ 0,0, s }), color, color);
			graphicsData.AddLine(p + Vector3D({ s,0,0 }), p - Vector3D({ 0,0,-s }), color, color);
			graphicsData.AddLine(p + Vector3D({-s,0,0 }), p - Vector3D({ 0,0, s }), color, color);
			graphicsData.AddLine(p + Vector3D({-s,0,0 }), p - Vector3D({ 0,0,-s }), color, color);

			graphicsData.AddLine(p + Vector3D({ 0, s,0 }), p - Vector3D({ 0,0, s }), color, color);
			graphicsData.AddLine(p + Vector3D({ 0, s,0 }), p - Vector3D({ 0,0,-s }), color, color);
			graphicsData.AddLine(p + Vector3D({ 0,-s,0 }), p - Vector3D({ 0,0, s }), color, color);
			graphicsData.AddLine(p + Vector3D({ 0,-s,0 }), p - Vector3D({ 0,0,-s }), color, color);
		}
	}

};
