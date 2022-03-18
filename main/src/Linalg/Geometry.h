/** ***********************************************************************************************
* @brief		Helper functions for geometrical manipulation, such as shortest distance, projection, etc.
*
* @author		Gerstmayr Johannes
* @date			2019-07-01 (generated)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
*
************************************************************************************************ */
#ifndef GEOMETRY__H
#define GEOMETRY__H

namespace HGeometry {
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++ LINES                                                                                      +++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
			linePointPoint = point - linePoint1; //****check this!
			return linePointPoint.GetL2Norm();
		}

		linePointPoint = point - (linePoint0 + relativePosition * lineVector); //****check this!

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

	//! compute line-circle intersection points as Reals; line given by two points p0, p1; circle given by circlePoint and radius r;
	//! relPos contains relative position(s) [0..1] of intersection points if existing
	//! will use boundary of line as intersection points, if intersection is at local coordinates < 0 or > 1
	inline void LineCircleIntersectionPoints(const Vector2D& p0, const Vector2D& p1,
		const Vector2D& circlePoint, Real r, ConstSizeVector<2>& relPos)
	{
		relPos.SetNumberOfItems(0);
		Vector2D vc = circlePoint - p0; //find projected point
		Vector2D vl = p1 - p0;  //vector defining line

		//Real num = lineVector * vLinePoint0Point;
		//Real den = lineVector * lineVector;
		Real vl2 = vl.GetL2NormSquared();

		if (vl2 == 0.)  //2 line-points are identical ==> return distance of linePoint and point; relativePosition is arbitrary
		{
			if (vc.GetL2NormSquared() < r*r)
			{
				relPos.AppendItem(0.);
			}
		}

		//vl=-d
		//vc=f
		Real a = vl2;
		Real b = -2*(vc*vl);
		Real c = vc * vc - r * r;

		Real det = (b*b - 4*a*c);

		if (det > 0.)
		{
			det = sqrt(det);
			Real t1 = (-b - det) / (2 * a); //smaller value, as a > 0 and det > 0
			Real t2 = (-b + det) / (2 * a);

			//pout << "t1=" << t1 << "t2=" << t2 << "\n";

			if ((t1 < 0 && t2 < 0) ||
				(t1 > 1 && t2 > 1)) {return; } //no contact

			relPos.AppendItem(EXUstd::Maximum(t1, 0.));
			relPos.AppendItem(EXUstd::Minimum(t2, 1.));
		}
		else if (det == 0.)
		{
			relPos.AppendItem((-b) / (2 * a));
		}
	}

	//! distance from line given by two line points, including boundary:
	inline Real MinDistToLinePoints(const Vector3D& lp1, const Vector3D& lp2, const Vector3D& p)
	{
		Vector3D v = lp2 - lp1;
		Vector3D vlp = p - lp1;

		Real num = v * vlp;
		Real den = v * v;

		if (num <= 0)
			return (lp1 - p).GetL2Norm();

		if (num >= den)
			return (lp2 - p).GetL2Norm();

		if (den > 0)
		{
			return sqrt(vlp*vlp - num * num / den);
		}
		else
			return vlp.GetL2Norm();
	}

	//! distance from line given by two line points, including boundary; compute also projected point pp
	inline Real MinDistToLinePoints(const Vector3D& lp1, const Vector3D& lp2, const Vector3D& p, Vector3D& pp)
	{
		Vector3D v = lp2 - lp1;
		Vector3D vlp = p - lp1;

		Real num = v * vlp;
		Real den = v * v;

		if (num <= 0)
		{
			pp = lp1;
			return vlp.GetL2Norm();
		}

		if (num >= den)
		{
			pp = lp2;
			return (lp2 - p).GetL2Norm();
		}

		if (den > 0)
		{
			pp = lp1 + (num / den)*v;
			return sqrt(vlp*vlp - num * num / den);
		}
		else
		{
			pp = lp1;
			return vlp.GetL2Norm();
		}
	}
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++ TRIANGLES                                                                                  +++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! compute normalized normal from array of triangle points (any 0-based array with [] operator), Vector3D version
	template<class ArrayVector3D>
	inline Vector3D ComputeTriangleNormalTemplate(const ArrayVector3D& trigPoints)
	{
		Vector3D v1 = trigPoints[1] - trigPoints[0];
		Vector3D v2 = trigPoints[2] - trigPoints[0];
		Vector3D n = v1.CrossProduct(v2); //@todo: need to check correct outward normal direction in openGL
		Real len = n.GetL2Norm();
		if (len != 0.f) { n *= 1.f / len; }
		return n;
	}

	inline Vector3D ComputeTriangleNormal(const std::array<Vector3D, 3>& trigPoints)
	{
		return ComputeTriangleNormalTemplate< std::array<Vector3D, 3> >(trigPoints);
	}

	//Minimal Distance of ppoint p to plane given by plane normal nPlane (any vector) and plane point pPlane
	inline Real DistanceToPlane(const Vector3D& p, const Vector3D& nPlane, const Vector3D& pPlane)
	{
		return fabs((pPlane - p) * nPlane) / nPlane.GetL2Norm();
	}

	//Minimal Distance of ppoint p to plane given by plane normal nPlane (normalized!) and plane point pPlane
	inline Real DistanceToPlaneNormalized(const Vector3D& p, const Vector3D& nPlane, const Vector3D& pPlane)
	{
		return fabs((pPlane - p) * nPlane);
	}

	//compute local triangle coordinates
	inline void LocalTriangleCoordinates(const Vector3D & e1, const Vector3D & e2,
		const Vector3D & v, Real & lam1, Real & lam2)
	{
		Real m11 = e1 * e1;
		Real m12 = e1 * e2;
		Real m22 = e2 * e2;
		Real rs1 = v * e1;
		Real rs2 = v * e2;

		Real det = m11 * m22 - m12 * m12;
		if (det != 0.)
		{
			lam1 = (rs1 * m22 - rs2 * m12) / det;
			lam2 = (m11 * rs2 - m12 * rs1) / det;
		}
		else
		{
			if (m11 != 0.) { lam1 = rs1 / m11; }
			else { lam1 = 0.; }
			if (m22 != 0.) { lam2 = rs2 / m22; }
			else { lam2 = 0.; }
		}
	}

	//minimum distance between point p and triangle given by points (tp1, tp2, tp3); inside=true, if projected point lies inside triangle
	inline Real MinDistTP(const Vector3D & tp1, const Vector3D & tp2, const Vector3D & tp3, const Vector3D & p)
	{
		Real lam1, lam2;
		Real res;

		LocalTriangleCoordinates(tp2 - tp1, tp3 - tp1, p - tp1, lam1, lam2);

		bool in1 = lam1 >= 0;
		bool in2 = lam2 >= 0;
		bool in3 = lam1 + lam2 <= 1;

		if (in1 && in2 && in3)
		{
			Vector3D pp = tp1 + lam1 * (tp2 - tp1) + lam2 * (tp3 - tp1);
			res = (p - pp).GetL2Norm();
		}
		else
		{
			res = (tp1 - p).GetL2Norm();
			if (!in1)
			{
				Real hv = MinDistToLinePoints(tp1, tp3, p);
				if (hv < res) res = hv;
			}
			if (!in2)
			{
				Real hv = MinDistToLinePoints(tp1, tp2, p);
				if (hv < res) res = hv;
			}
			if (!in3)
			{
				Real hv = MinDistToLinePoints(tp2, tp3, p);
				if (hv < res) res = hv;
			}
		}
		return res;
	}

	//!minimum distance between point p and triangle given by points (tp1, tp2, tp3); inside=1, if projected point lies inside triangle, 
	//! inside=2 if on edge, using some small tolerance, inside=0 if outside
	//!compute also projected point pp
	inline Real MinDistTP(const Vector3D & tp1, const Vector3D & tp2, const Vector3D & tp3, const Vector3D & p, Vector3D& pp, Index& inside)
	{
		Real lam1, lam2;
		Real res;

		LocalTriangleCoordinates(tp2 - tp1, tp3 - tp1, p - tp1, lam1, lam2);

		const Real TOL = 1e-15; //add very small tolerance in order to avoid problems if point is directly at edge
		bool in1 = lam1 >= -TOL;
		bool in2 = lam2 >= -TOL;
		bool in3 = lam1 + lam2 <= (1.+TOL);

		if (in1 && in2 && in3)
		{
			inside = 1;

			//check if point is on the edge, avoiding that two contacts are counted; this slows down but avoids artifacts:
			if ((lam1 <= TOL) || (lam2 <= TOL) || (lam1 + lam2 >= (1. - TOL))) { inside = 2; }

			pp = tp1 + lam1 * (tp2 - tp1) + lam2 * (tp3 - tp1);
			res = (p - pp).GetL2Norm();
		}
		else
		{
			inside = 0;
			res = (tp1 - p).GetL2Norm();
			if (!in1)
			{
				Real hv = MinDistToLinePoints(tp1, tp3, p, pp);
				if (hv < res) res = hv;
			}
			else if (!in2)
			{
				Real hv = MinDistToLinePoints(tp1, tp2, p, pp);
				if (hv < res) res = hv;
			}
			else if (!in3)
			{
				Real hv = MinDistToLinePoints(tp2, tp3, p, pp);
				if (hv < res) res = hv;
			}
		}
		return res;
	}



};


#endif
