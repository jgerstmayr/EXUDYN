/** ***********************************************************************************************
* @brief		Helper functions for geometrical manipulation, such as shortest distance, projection, etc.
*
* @author		Gerstmayr Johannes
* @date			2019-07-01 (generated)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
*
************************************************************************************************ */

#pragma once

//#include "Linalg/BasicLinalg.h"	

namespace HGeometry {

	//! compute shortest distance of a point to line defined by two points; EXCLUDING endpoints; 
	//! on output, the relativePosition is a value between 0 and 1, showing the projected position (0=point0, 1=point1, other: point in between)
	//! should work for Vector2D, Vector3D, ConstVector<> ...; will be slow for Vector!
	//! UNTESTED
	template<class TVector>
	inline Real ShortestDistanceRelativePosition(const TVector& linePoint0, const TVector& linePoint1, const TVector& point, Real& relativePosition)
	{
		TVector vLinePoint0Point = point - linePoint0; //find projected point
		TVector lineVector = linePoint1 - linePoint0;  //vector defining line

		Real num = lineVector * vLinePoint0Point;
		Real den = lineVector * lineVector;

		if (den == 0.)  //2 line-points are identical ==> return distance of linePoint and point; relativePosition is arbitrary
		{
			relativePosition = 0; //not defined, but would lead to correct result: pp=linePoint0 + relativePosition*lineVector
			return vLinePoint0Point.GetL2Norm(); 
		}

		Real den2 = vLinePoint0Point * vLinePoint0Point;
		if (den2 == 0) //point == linePoint0
		{
			relativePosition = 0;
			return 0.;
		}

		relativePosition = num / sqrt(den * den2);

		return sqrt(vLinePoint0Point*vLinePoint0Point - num * num / den);
	}

	//! compute shortest distance of a point to line defined by two points; INCLUDING endpoints; 
	//! on output, the relativePosition is a value between 0 and 1, showing the projected position (0=point0, 1=point1, other: point in between)
	//!            furthermore, the vector from projected linePoint to point is returned in linePointPoint
	//! should work for Vector2D, Vector3D, ConstVector<> ...; will be slow for Vector!
	//! UNTESTED
	template<class TVector>
	inline Real ShortestDistanceEndPointsRelativePosition(const TVector& linePoint0, const TVector& linePoint1, const TVector& point, 
		Real& relativePosition, TVector& linePointPoint)
	{
		TVector vLinePoint0Point = point - linePoint0; //find projected point
		TVector lineVector = linePoint1 - linePoint0;  //vector defining line

		Real num = lineVector * vLinePoint0Point;
		Real den = lineVector * lineVector;

		if (den == 0.)  //2 line-points are identical ==> return distance of linePoint and point; relativePosition is arbitrary
		{
			relativePosition = 0; //not defined, but would lead to correct result: pp=linePoint0 + relativePosition*lineVector
			linePointPoint = vLinePoint0Point;
			return vLinePoint0Point.GetL2Norm();
		}

		relativePosition = num / den;

		if (relativePosition < 0.) 
		{
			relativePosition = 0.;
			linePointPoint = vLinePoint0Point;
			return vLinePoint0Point.GetL2Norm();
		}

		if (relativePosition >= 1.) //projected point outside line segment ==> chose point1
		{
			relativePosition = 1.;
			linePointPoint = point - linePoint1;
			return linePointPoint.GetL2Norm();
		}

		linePointPoint = point - (linePoint0 + relativePosition * lineVector);

		return sqrt(vLinePoint0Point*vLinePoint0Point - num * num / den);
	}

	//! compute shortest distance of a point to a line defined by two points; INCLUDING endpoints; 
	//! on output, 'point' is projected on line (considering only the interval of the two points!)
	//! should work for Vector2D, Vector3D, ConstVector<> ...; will be slow for Vector!
	//! UNTESTED
	template<class TVector>
	inline Real ShortestDistanceEndPointsProject(const TVector& linePoint0, const TVector& linePoint1, TVector& point)
	{
		TVector vLinePoint0Point = point - linePoint0; //find projected point
		TVector lineVector = linePoint1 - linePoint0;  //vector defining line

		Real num = lineVector * vLinePoint0Point;
		Real den = lineVector * lineVector;

		if (den == 0. || //2 line-points are identical ==> return distance of linePoint and point; project in plane is just the linePoint0 
			num <= 0.)   //projected point outside line segment ==> chose point0
		{
			point = linePoint0;
			return vLinePoint0Point.GetL2Norm();
		}

		if (num / den >=1) //projected point outside line segment ==> chose point1
		{
			Real distance = (point - linePoint1).GetL2Norm();
			point = linePoint1;
			return distance;
		}

		point = linePoint0 + (num / den)*lineVector; //projected point; den is correct, because division by length of lineVector is needed twice in this formula

		return sqrt(vLinePoint0Point*vLinePoint0Point - num * num / den);
	}

	//! compute shortest distance of a point to a line defined by two points; NOT INCLUDING endpoints; 
	//! on output, 'point' is projected on line (considering only the interval of the two points!)
	//! should work for Vector2D, Vector3D, ConstVector<> ...; will be slow for Vector!
	template<class TVector>
	inline Real ShortestDistanceProject(const TVector& linePoint0, const TVector& linePoint1, TVector& point)
	{
		TVector vLinePoint0Point = point - linePoint0; //find projected point
		TVector lineVector = linePoint1 - linePoint0;  //vector defining line

		Real num = lineVector * vLinePoint0Point;
		Real den = lineVector * lineVector;

		if (den == 0.)  //2 line-points are identical ==> return distance of linePoint and point; project in plane is just the linePoint0
		{
			point = linePoint0;
			return vLinePoint0Point.GetL2Norm();
		}

		point = linePoint0 + (num / den)*lineVector; //projected point; den is correct, because division by length of lineVector is needed twice in this formula

		return sqrt(vLinePoint0Point*vLinePoint0Point - num * num / den);
	}



	//inline void ComputeOrthogonalBasis(Vector3D vector0, Vector3D& normal1, Vector3D& normal2) { ... }


}



