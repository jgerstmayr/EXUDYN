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

namespace EGeometry {
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++ LINES                                                                                      +++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//! compute shortest distance of a point to line defined by two points; EXCLUDING endpoints; 
	//! on output, the relativePosition is a value between 0 and 1, showing the projected position (0=point0, 1=point1, other: point in between)
	//! should work for Vector2D, Vector3D, ConstVector<> ...; will be slow for Vector!
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

		relativePosition = num / den;
		Real val = vLinePoint0Point * vLinePoint0Point - num * relativePosition;
		if (val < 0.) { return 0.; } //for pathological cases?
		return sqrt(val);
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
	template<class TReal, class ArrayVector3D>
	inline SlimVectorBase<TReal, 3> ComputeTriangleNormalTemplate(const ArrayVector3D& trigPoints)
	{
		SlimVectorBase<TReal, 3> v1 = trigPoints[1] - trigPoints[0];
		SlimVectorBase<TReal, 3> v2 = trigPoints[2] - trigPoints[0];
		SlimVectorBase<TReal, 3> n = v1.CrossProduct(v2); //@todo: need to check correct outward normal direction in openGL
		TReal len = n.GetL2Norm();
		if (len != (TReal)0.) { n *= (TReal)1. / len; }
		return n;
	}

	inline Vector3D ComputeTriangleNormal(const std::array<Vector3D, 3>& trigPoints)
	{
		return ComputeTriangleNormalTemplate<Real, std::array<Vector3D, 3> >(trigPoints);
	}

	//Minimal Distance of ppoint p to plane given by plane normal nPlane (any vector) and plane point pPlane
	inline Real DistanceToPlane(const Vector3D& p, const Vector3D& nPlane, const Vector3D& pPlane)
	{
		return fabs((pPlane - p) * nPlane) / nPlane.GetL2Norm();
	}

	//Minimal Distance of ppoint p to plane given by plane normal nPlane0 (normalized!) and plane point pPlane
	inline Real DistanceToPlaneNormalized(const Vector3D& p, const Vector3D& nPlaneNormalized, const Vector3D& pPlane)
	{
		return fabs((pPlane - p) * nPlaneNormalized);
	}

	//having a plane point and a normalized plane normal, project p into plane
	inline void ProjectInPlane(const Vector3D& pPlane, const Vector3D& nPlaneNormalized, const Vector3D& p, Vector3D& pProjected)
	{
		pProjected = p - ((p - pPlane)* nPlaneNormalized);
	}

	//!Compute line-plane intersection of plane given by point and normal and line given by pLine and direction vLine
	//!return true if success and false if fails (line and plane are parallel)
	//!the resulting intersection point follows from pLine+relativeDistance*vLine
	//!if vLine has length 1, the relativeDistance gives the distance from pLine
	template<class TVector3D, class TReal>
	bool LinePlaneIntersection(const TVector3D& pPlane, const TVector3D& nPlane, const TVector3D& pLine, 
		const TVector3D& vLine, TReal& relativeDistance)
	{
		TReal den = nPlane * vLine;
		if (den == 0.)
		{
			relativeDistance = 0.;
			return false;
		}

		relativeDistance = ((pPlane - pLine) * nPlane) / den;
		return true;
	}

	//inline bool LinePlaneIntersection(const Vector3D& pPlane, const Vector3D& nPlane, const Vector3D& pLine, const Vector3D& vLine, Real& relativeDistance)
	//{
	//	return LinePlaneIntersectionTemplate(pPlane, nPlane, pLine, vLine, relativeDistance);
	//}

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
				if (hv < res) { res = hv; }
			}
			else if (!in2)
			{
				Real hv = MinDistToLinePoints(tp1, tp2, p, pp);
				if (hv < res) { res = hv; }
			}
			else if (!in3)
			{
				Real hv = MinDistToLinePoints(tp2, tp3, p, pp);
				if (hv < res) { res = hv; }
			}
		}
		return res;
	}

	//! minimum distance between point p and triangle given by points (tp1, tp2, tp3); special version for edges double detection
	//! inside=1, if projected point lies inside triangle, 
	//! onEdge=1 if on edge (tp1-tp2), 2 if on edge (tp2-tp3), 4 if on edge (tp3-tp1) (binary flags used)
	//! compute also projected point pp
	inline Real MinDistTrigEdges(const Vector3D& tp1, const Vector3D& tp2, const Vector3D& tp3, const Vector3D& p,
		Vector3D& pp, Index& inside, Index& onEdge)
	{
		Real lam1, lam2;
		Real res;

		LocalTriangleCoordinates(tp2 - tp1, tp3 - tp1, p - tp1, lam1, lam2);

		const Real TOL = 1e-15; //add very small tolerance in order to avoid problems if point is directly at edge
		bool in1 = lam1 >= -TOL;
		bool in2 = lam2 >= -TOL;
		bool in3 = lam1 + lam2 <= (1. + TOL);
		onEdge = 0; //no edge

		if (in1 && in2 && in3)
		{
			inside = 1;

			//check if point is on the edge, avoiding that two contacts are counted; this slows down but avoids artifacts:
			// this does not make sense for single triangle contacts
			//if ((lam1 <= TOL) || (lam2 <= TOL) || (lam1 + lam2 >= (1. - TOL))) { inside = 2; }

			pp = tp1 + lam1 * (tp2 - tp1) + lam2 * (tp3 - tp1);
			res = (p - pp).GetL2Norm();
		}
		else
		{
			inside = 0;
			res = (tp1 - p).GetL2Norm();
			if (!in1)
			{
				onEdge = 4;
				Real hv = MinDistToLinePoints(tp1, tp3, p, pp);
				if (hv < res) { res = hv; }
			}
			else if (!in2)
			{
				onEdge = 1;
				Real hv = MinDistToLinePoints(tp1, tp2, p, pp);
				if (hv < res) { res = hv; }
			}
			else if (!in3)
			{
				onEdge = 2;

				Real hv = MinDistToLinePoints(tp2, tp3, p, pp);
				if (hv < res) { res = hv; }
			}
		}
		return res;
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++ LINES                                                                                      +++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	//! compute relative cutting positions of two line segments defined by point and vector
	//! returns relative position along segment, where 0 <= relPos <= 1 means a cutting point
	//! returns false if line segments are parallel (and relPos is arbitrary), otherwise true
	inline bool CuttingOf2DLineSegments(const Vector2D& p0, const Vector2D& v0, 
		const Vector2D& p1, const Vector2D& v1, 
		Real& relPos0, Real& relPos1)
	{
		ConstSizeMatrix<4> A(2, 2, {v0[0], -v1[0], v0[1], -v1[1]});
		if (A(0, 0)*A(1, 1) - A(0, 1)*A(1, 0) == 0) { return false; }

		Vector2D r = A.GetInverse()*(p1-p0);
		relPos0 = r[0];
		relPos1 = r[1];
		return true;
	}

	//! compute common tangent of 2 spatial circles A and B defined by center point p, axis a, radius R; 
	//! computes global radius vector r;
	//! caseA and caseB shall be +1/-1 to define the required cases
	//! returns true if successful, false in case of any error
	inline bool CommonTangentOf2Circles(const Vector3D& pA, const Vector3D& pB, const Vector3D& aA, const Vector3D& aB, Real RA, Real RB,
		Vector3D& rA, Vector3D& rB, Real caseA = 1., Real caseB = 1., bool throwException = true) //, Real& phiA, Real& phiB)
	{
		//if both radii, it needs no iterations
		if (RA == 0. && RB == 0.)
		{
			rA = Vector3D(0.);
			rB = Vector3D(0.);
			return true;
		}

		Vector3D c = pB - pA;
		Real cLen = c.GetL2Norm();
		if (cLen == 0)
		{
			if (throwException) { CHECKandTHROWstring("CommonTangentOf2Circles: distance of center of two circles may not be zero; check your circles system"); }
			else { return false; }
		}

		Vector3D c0 = c * (1. / cLen);
		Vector3D nA = aA.CrossProduct(c0);
		Vector3D nB = aB.CrossProduct(c0);
		//nA needs to be normalized!!!
		Real nAlen = nA.GetL2Norm();
		Real nBlen = nA.GetL2Norm();
		if (nAlen == 0. || nBlen == 0.)
		{
			if (throwException) { CHECKandTHROWstring("CommonTangentOf2Circles: axes may not be parallel to vector between circle midpoints"); }
			else { return false; }
		}
		nA *= 1. / nAlen;
		nB *= 1. / nBlen;

		Vector3D tA = nA.CrossProduct(aA);
		Vector3D tB = nB.CrossProduct(aB);
		//pout << "common tangent:\n";
		//pout << "nA=" << nA.GetL2Norm() << ", nB=" << nB.GetL2Norm() 
		//	<< ", tA=" << tA.GetL2Norm() << ", tB=" << tB.GetL2Norm() 
		//	<< ", aA=" << aA.GetL2Norm() << ", aB=" << aB.GetL2Norm() << "\n";
		Vector2D x({ EXUstd::pi*0.5, EXUstd::pi*0.5 }); //phiA, phiB for Newton's method

		const Real tol = 1e-12;
		Real lastError = 1.;
		Real tolFact = (cLen + RA + RB)*(RA + RB); //factor for error, resulting from approx. residual (Res) norm
		Index nIt = 0;
		const Index maxIt = 8;
		while (lastError > tol && nIt++ < maxIt)
		{
			Real phiA = x[0];
			Real phiB = x[1];
			Real sinPhiA = sin(phiA);
			Real sinPhiB = sin(phiB);
			Real cosPhiA = cos(phiA);
			Real cosPhiB = cos(phiB);
			//residual:
			rA = RA * (tA*cosPhiA - caseA * nA * sinPhiA);
			rB = RB * (tB*cosPhiB - caseB * nB * sinPhiB);

			//Tangent: tc=pB+rB-pA-rA = c+rB-rA
			//Residual: tc*rA=0, tc*rB=0 
			//     ==>  (c+rB-rA)*rA=0, (c+rB-rA)*rB=0 
			Vector2D Res({ c*rA - RA * RA + rA * rB, c*rB + RB * RB - rA * rB });

			//derivatives:
			Vector3D rA_phiA = (-RA * sinPhiA)*tA - (RA*cosPhiA)*nA;
			Vector3D rB_phiB = (-RB * sinPhiB)*tB - (RB*cosPhiB)*nB;

			Matrix2D Jac(2, 2);
			Jac(0, 0) = c * rA_phiA + rA_phiA * rB;
			Jac(1, 0) = -rA_phiA * rB; //der. w.r.t. phiA
			Jac(0, 1) = rA * rB_phiB;
			Jac(1, 1) = c * rB_phiB - rA * rB_phiB; //der. w.r.t. phiB

			//special cases if one of the radii is zero
			if (RA == 0.) { Jac(0, 0) = 1.; Res[0] = 0.; } //in this case, 
			if (RB == 0.) { Jac(1, 1) = 1.; Res[1] = 0.; } //in this case, 

			Real det = Jac(0, 0)*Jac(1, 1) - Jac(0, 1)*Jac(1, 0);
			if (det == 0)
			{
				if (throwException) { CHECKandTHROW(det != 0., "CommonTangentOf2Circles: tangent not found; possibly sheaves are arranged in inappropriate configuration"); }
				else { return false; }
			}
			x -= Jac.GetInverse()*Res; //Newton step

			lastError = Res.GetL2Norm() / tolFact;

			//pout << "it" << nIt << ", phiA=" << phiA << ", phiB=" << phiB << ", rA=" << rA << ", rB=" << rB << ", err=" << lastError << "\n";
		}
		if (nIt == maxIt) { return false; }
		return true;
	}


};


#endif
