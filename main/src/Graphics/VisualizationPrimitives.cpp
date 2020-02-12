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

	//! draw a simple spring in 2D with given endpoints p0,p1 a width, a (normalized) normal vector for the width drawing and number of spring points numberOfPoints
	void DrawSpring2D(const Vector3D& p0, const Vector3D& p1, const Vector3D& vN, Index numberOfPoints, Real width, const Float4& color, GraphicsData& graphicsData)
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
			if (i > 1 && i < numberOfPoints - 1) { pAct += width * (sign*2.f - 1.f)* vN; }

			if (i > 0)
			{
				graphicsData.AddLine(pLast, pAct, color, color);
			}

			pLast = pAct;
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
		Index nTiles, Real innerRadius, Vector2D angleRange, bool lastFace, bool cutPlain)
	{
		if (nTiles < 2) { nTiles = 2; } //less than 2 tiles makes no sense
		if (radius <= 0.) { return; } //just a line
		if (vAxis.GetL2NormSquared() == 0.) { return; } //too short

		//create points at left and right face
		//points0 = copy.deepcopy(pAxis) #[pAxis[0], pAxis[1], pAxis[2]] #avoid change of pAxis
		Vector3D pAxis1 = pAxis0 + vAxis;

		Vector3D n1, n2;
		EXUmath::ComputeOrthogonalBasis(vAxis, n1, n2);

		//#create normals at left and right face(pointing inwards)
		Real alpha = angleRange[1] - angleRange[0]; //angular range
		Real alpha0 = angleRange[0];

		Real fact = nTiles; //#create correct part of cylinder (closed/not closed
		if (alpha < 2.*EXUstd::pi) { fact = nTiles - 1; } 

		std::array<Vector3D, 3> points;
		std::array<Vector3D, 3> normals;
		std::array<Float4, 3> colors({color,color,color}); //all triangles have same color

		Vector3D nF1 = vAxis;
		nF1.Normalize();

		std::array<Vector3D, 3> normalsFace0({ nF1,nF1,nF1 });
		nF1 = -nF1;
		std::array<Vector3D, 3> normalsFace1({ nF1,nF1,nF1 });

		for (Index i = 0; i < nTiles; i++)
		{
			Real phi0 = alpha0 + i * alpha / fact;
			Real phi1 = alpha0 + (i+1) * alpha / fact;

			Real x0 = radius * sin(phi0);
			Real y0 = radius * cos(phi0);
			Real x1 = radius * sin(phi1);
			Real y1 = radius * cos(phi1);
			Vector3D vv0 = x0 * n1 + y0 * n2;
			Vector3D vv1 = x1 * n1 + y1 * n2;
			Vector3D pzL0 = pAxis0 + vv0;
			Vector3D pzL1 = pAxis0 + vv1;
			Vector3D pzR0 = pAxis1 + vv0;
			Vector3D pzR1 = pAxis1 + vv1;

			Vector3D n0 = -vv0;
			Vector3D n1 = -vv1;
			n0.Normalize();
			n1.Normalize();

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
	void DrawSphere(const Vector3D& p, Real radius, const Float4& color, GraphicsData& graphicsData, Index nTiles)
	{
		if (nTiles < 3) { nTiles = 3; } //less than 3 tiles makes no sense
		if (radius <= 0.) { return; } //not visible
		
		//const Vector3D& e0 = EXUmath::unitVecX;
		//const Vector3D& e1 = EXUmath::unitVecY;
		//const Vector3D& e2 = EXUmath::unitVecZ;

		std::array<Vector3D, 3> points;
		std::array<Vector3D, 3> normals;
		std::array<Float4, 3> colors({ color,color,color }); //all triangles have same color

		//create points for circles around z - axis with tiling
		for (Index i0 = 0; i0 < nTiles; i0++) //nTiles+1 in python, when generating points
		{
			for (Index iphi = 0; iphi < nTiles; iphi++)
			{
				Real z0 = -radius * cos(EXUstd::pi * (Real)i0 / (Real)nTiles);    //runs from - r ..r(this is the coordinate of the axis of circles)
				Real fact0 = sin(EXUstd::pi*(Real)i0 / (Real)nTiles);
				Real z1 = -radius * cos(EXUstd::pi * (Real)(i0+1) / (Real)nTiles);    //runs from - r ..r(this is the coordinate of the axis of circles)
				Real fact1 = sin(EXUstd::pi*(Real)(i0 + 1) / (Real)nTiles);

				Real phiA = 2. * EXUstd::pi * (Real)iphi / (Real)nTiles; //angle
				Real phiB = 2. * EXUstd::pi * (Real)(iphi+1) / (Real)nTiles; //angle

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

				//triangle1: 0A, 1B, 1A
				normals[0] = -v0A;
				normals[1] = -v1A;
				normals[2] = -v1B;
				normals[0].Normalize();
				normals[1].Normalize();
				normals[2].Normalize();
				points[0] = p + v0A;
				points[1] = p + v1A;
				points[2] = p + v1B;
				graphicsData.AddTriangle(points, normals, colors);

				//triangle1: 0A, 0B, 1B
				normals[0] = -v0A;
				normals[1] = -v0B;
				normals[2] = -v1B;
				normals[0].Normalize();
				normals[1].Normalize();
				normals[2].Normalize();
				points[0] = p + v0A;
				points[1] = p + v0B;
				points[2] = p + v1B;
				graphicsData.AddTriangle(points, normals, colors);
			}
		}
	}

//	def GraphicsDataCube(pList, color = [0., 0., 0., 1.], faces = [1, 1, 1, 1, 1, 1]) :
//		# bottom : (z goes upwards from node 1 to node 5)
//# ^y
//# |
//		# 3-- - 2
//# |   |
//# |   |
//		# 0-- - 1-- > x
//#
//		# top:
//# ^y
//# |
//	# 7-- - 6
//# |   |
//# |   |
//		# 4-- - 5-- > x
//#
//		# faces: bottom, top, sideface0, sideface1, sideface2, sideface3(sideface0 has nodes 0, 1, 4, 5)
//
//		colors = []
//		for i in range(8) :
//			colors = colors + color
//
//			points = []
//			for p in pList :
//	points += p
//		#    points = [xMin, yMin, zMin, xMax, yMin, zMin, xMax, yMax, zMin, xMin, yMax, zMin,
//		#              xMin, yMin, zMax, xMax, yMin, zMax, xMax, yMax, zMax, xMin, yMax, zMax]
//
//		#1 - based ... triangles = [1, 3, 2, 1, 4, 3, 5, 6, 7, 5, 7, 8, 1, 2, 5, 2, 6, 5, 2, 3, 6, 3, 7, 6, 3, 4, 7, 4, 8, 7, 4, 1, 8, 1, 5, 8]
//		#triangles = [0, 2, 1, 0, 3, 2, 6, 4, 5, 6, 7, 4, 0, 1, 4, 1, 5, 4, 1, 2, 5, 2, 6, 5, 2, 3, 6, 3, 7, 6, 3, 0, 7, 0, 4, 7]
//
//		#    triangles = [0, 1, 2, 0, 2, 3, 6, 5, 4, 6, 4, 7, 0, 4, 1, 1, 4, 5, 1, 5, 2, 2, 5, 6, 2, 6, 3, 3, 6, 7, 3, 7, 0, 0, 7, 4]
//		trigList = [[0, 1, 2, 0, 2, 3], [6, 5, 4, 6, 4, 7], [0, 4, 1, 1, 4, 5], [1, 5, 2, 2, 5, 6], [2, 6, 3, 3, 6, 7], [3, 7, 0, 0, 7, 4]]
//		triangles = []
//		for i in range(6) :
//			if faces[i] :
//				triangles += trigList[i]
//
//				data = { 'type':'TriangleList', 'colors' : colors, 'points' : points, 'triangles' : triangles }
//
//				return data

	//! add a cone to graphicsData with reference point (pAxis0), axis vector (vAxis) and radius using triangle representation
	//! cone starts at pAxis0, tip is at pAxis0+vAxis0
	void DrawCone(const Vector3D& pAxis0, const Vector3D& vAxis, Real radius, const Float4& color, GraphicsData& graphicsData, Index nTiles)
	{
		if (nTiles < 2) { nTiles = 2; } //less than 2 tiles makes no sense
		if (radius <= 0.) { return; } //just a line
		Real axisLength = vAxis.GetL2Norm();
		if (axisLength == 0.) { return; } //too short

		//create points at left and right face
		//points0 = copy.deepcopy(pAxis) #[pAxis[0], pAxis[1], pAxis[2]] #avoid change of pAxis
		Vector3D pAxis1 = pAxis0 + vAxis;

		Vector3D n1, n2;
		EXUmath::ComputeOrthogonalBasis(vAxis, n1, n2);

		//#create normals at left and right face(pointing inwards)
		Real alpha = 2.*EXUstd::pi;

		Real fact = nTiles; //#create correct part of cylinder (closed/not closed

		std::array<Vector3D, 3> points;
		std::array<Vector3D, 3> normals;
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
			Vector3D vv0 = x0 * n1 + y0 * n2;
			Vector3D vv1 = x1 * n1 + y1 * n2;
			Vector3D pzL0 = pAxis0 + vv0;
			Vector3D pzL1 = pAxis0 + vv1;

			//normal to cone surface:
			Vector3D n0 = (-axisLength/radius)*vv0 + radius*nF0;
			Vector3D n1 = (-axisLength / radius)*vv1 + radius * nF0;
			n0.Normalize();
			n1.Normalize();

			//+++++++++++++++++++++++++++++++
			//circumference:
			normals[0] = n0;
			normals[1] = n1;
			normals[2] = n1;
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
	void DrawOrthonormalBasis(const Vector3D& p, const Matrix3D& rot, Real length, Real radius, GraphicsData& graphicsData, float colorFactor, bool draw3D, Index nTiles, Real arrowSizeRelative)
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
		}
	}

};
