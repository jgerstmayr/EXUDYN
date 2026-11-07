/** ***********************************************************************************************
* @brief		The distance of two convex shapes: support functions, GJK and EPA (#3317)
* @details		Details:
*				- a ConvexShape is a sphere, capsule, cylinder, box, triangle or a set of points in its local frame;
*				  the sphere and the capsule are a point and a segment (the core) with the radius as margin
*				- ConvexDistance gives the signed distance of two shapes in their frames HT0 and HT1 (negative for a
*				  penetration), the closest points of both (the deepest points for a penetration) and the normal from
*				  shape 0 to shape 1
*				- GJK (Gilbert-Johnson-Keerthi) on the cores gives the distance and the closest points from the
*				  support functions; EPA (expanding polytope algorithm) the penetration depth where the cores overlap;
*				  the margins are subtracted from the distance of the cores
*				- a cylinder, whose side GJK and EPA approach slowly: the closest points refined by alternating
*				  projections, a penetration by moving shape 1 out along the normal and taking the distance
*				- the sphere and the cylinder also by the analytic formula, the check of GJK and the faster path
*				- used by exudyn.cUtilities.Distance, and by the contact items and the collision check of path
*				  planning (docs/dev/design/contactPrimitives.md)
*
* @author		Gerstmayr Johannes
* @date			2026-11-07
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#ifndef GEOMETRYCONVEX__H
#define GEOMETRYCONVEX__H

#include <vector>
#include "Linalg/BasicLinalg.h"
#include "Linalg/HomogeneousTransformation.h"

namespace EGeometry {

	enum class ConvexShapeType { Sphere, Capsule, Cylinder, Box, Triangle, Points };

	//! a convex shape in its local frame, centered at the origin of the frame except Triangle and Points
	class ConvexShape
	{
	public:
		ConvexShapeType type = ConvexShapeType::Sphere;
		Real radius = 0.;				//!< Sphere, Capsule, Cylinder
		Real length = 0.;				//!< Capsule: the segment between the centers of the caps, along z; Cylinder: along z
		Vector3D halfSize = Vector3D({ 0., 0., 0. });	//!< Box: half the edge lengths along x, y, z
		std::vector<Vector3D> points;	//!< Triangle: 3 points; Points: the points whose convex hull is the shape

		//! the radius added to the core: the sphere is a point, the capsule a segment
		Real Margin() const { return (type == ConvexShapeType::Sphere || type == ConvexShapeType::Capsule) ? radius : 0.; }

		//! the radius of a sphere about the origin of the frame that contains the shape
		Real BoundingRadius() const
		{
			switch (type)
			{
			case ConvexShapeType::Sphere: return radius;
			case ConvexShapeType::Capsule: return radius + 0.5 * length;
			case ConvexShapeType::Cylinder: return sqrt(radius * radius + 0.25 * length * length);
			case ConvexShapeType::Box: return halfSize.GetL2Norm();
			default:
			{
				Real r = 0.;
				for (const Vector3D& p : points) { r = EXUstd::Maximum(r, p.GetL2Norm()); }
				return r;
			}
			}
		}

		//! the point of the core farthest in the local direction d
		Vector3D SupportCore(const Vector3D& d) const
		{
			switch (type)
			{
			case ConvexShapeType::Sphere: return Vector3D({ 0., 0., 0. });
			case ConvexShapeType::Capsule: return Vector3D({ 0., 0., d[2] >= 0. ? 0.5 * length : -0.5 * length });
			case ConvexShapeType::Cylinder:
			{
				Real z = d[2] >= 0. ? 0.5 * length : -0.5 * length;
				Real rho = sqrt(d[0] * d[0] + d[1] * d[1]);
				if (rho <= 1e-15 * fabs(d[2])) { return Vector3D({ 0., 0., z }); } //along the axis: the center of the cap
				return Vector3D({ radius * d[0] / rho, radius * d[1] / rho, z });
			}
			case ConvexShapeType::Box:
				return Vector3D({ d[0] >= 0. ? halfSize[0] : -halfSize[0], d[1] >= 0. ? halfSize[1] : -halfSize[1],
					d[2] >= 0. ? halfSize[2] : -halfSize[2] });
			default:
			{
				Index best = 0;
				Real bestValue = points[0] * d;
				for (Index i = 1; i < (Index)points.size(); i++)
				{
					Real value = points[i] * d;
					if (value > bestValue) { bestValue = value; best = i; }
				}
				return points[best];
			}
			}
		}
	};

	//! the result of ConvexDistance; the points and the normal are global
	class ConvexDistanceResult
	{
	public:
		Real distance = 0.;			//!< signed: negative for a penetration, then minus the penetration depth
		Vector3D point0 = Vector3D({ 0., 0., 0. });	//!< the closest point of shape 0 (the deepest for a penetration)
		Vector3D point1 = Vector3D({ 0., 0., 0. });	//!< the closest point of shape 1
		Vector3D normal = Vector3D({ 0., 0., 1. });	//!< unit vector from shape 0 to shape 1
		Index iterations = 0;		//!< the iterations of GJK plus those of EPA
	};

	//! a vertex of the Minkowski difference C = A - B with the support points of both shapes
	class ConvexVertex
	{
	public:
		Vector3D w, a, b;
	};

	//! the two shapes in their global frames; the support functions of the cores and of their difference
	class ConvexPair
	{
	public:
		const ConvexShape& shape0;
		const ConvexShape& shape1;
		const HomogeneousTransformation& HT0;
		const HomogeneousTransformation& HT1;

		ConvexPair(const ConvexShape& s0, const HomogeneousTransformation& H0, const ConvexShape& s1, const HomogeneousTransformation& H1) :
			shape0(s0), shape1(s1), HT0(H0), HT1(H1) {}

		//! the vertex of C farthest in the global direction d: the support of A in d minus the support of B in -d
		ConvexVertex Support(const Vector3D& d) const
		{
			ConvexVertex v;
			v.a = HT0.GetTranslation() + HT0.RotateVector(shape0.SupportCore(HT0.RotateVectorTransposed(d)));
			v.b = HT1.GetTranslation() + HT1.RotateVector(shape1.SupportCore(HT1.RotateVectorTransposed(-1. * d)));
			v.w = v.a - v.b;
			return v;
		}
	};

	//! the closest point of the triangle (a, b, c) to the point p, with its barycentric coordinates (Ericson,
	//! Real-Time Collision Detection, 5.1.5); a degenerate triangle by its edges
	inline Vector3D ClosestPointTriangle(const Vector3D& p, const Vector3D& a, const Vector3D& b, const Vector3D& c, Vector3D& lambda)
	{
		Vector3D ab = b - a, ac = c - a, ap = p - a;
		Real d1 = ab * ap, d2 = ac * ap;
		if (d1 <= 0. && d2 <= 0.) { lambda = Vector3D({ 1., 0., 0. }); return a; }
		Vector3D bp = p - b;
		Real d3 = ab * bp, d4 = ac * bp;
		if (d3 >= 0. && d4 <= d3) { lambda = Vector3D({ 0., 1., 0. }); return b; }
		Real vc = d1 * d4 - d3 * d2;
		if (vc <= 0. && d1 >= 0. && d3 <= 0.)
		{
			Real t = d1 / (d1 - d3);
			lambda = Vector3D({ 1. - t, t, 0. });
			return a + t * ab;
		}
		Vector3D cp = p - c;
		Real d5 = ab * cp, d6 = ac * cp;
		if (d6 >= 0. && d5 <= d6) { lambda = Vector3D({ 0., 0., 1. }); return c; }
		Real vb = d5 * d2 - d1 * d6;
		if (vb <= 0. && d2 >= 0. && d6 <= 0.)
		{
			Real t = d2 / (d2 - d6);
			lambda = Vector3D({ 1. - t, 0., t });
			return a + t * ac;
		}
		Real va = d3 * d6 - d5 * d4;
		if (va <= 0. && (d4 - d3) >= 0. && (d5 - d6) >= 0.)
		{
			Real t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			lambda = Vector3D({ 0., 1. - t, t });
			return b + t * (c - b);
		}
		Real sum = va + vb + vc;
		if (sum <= 0.) //collinear points: the nearest of the three edges
		{
			Vector3D best = a;
			Real bestDistance = -1.;
			const Vector3D* q[3] = { &a, &b, &c };
			for (Index i = 0; i < 3; i++)
			{
				const Vector3D& q0 = *q[i];
				const Vector3D& q1 = *q[(i + 1) % 3];
				Vector3D e = q1 - q0;
				Real ee = e * e;
				Real t = ee > 0. ? EXUstd::Minimum(1., EXUstd::Maximum(0., ((p - q0) * e) / ee)) : 0.;
				Vector3D x = q0 + t * e;
				Real distance = (x - p).GetL2NormSquared();
				if (bestDistance < 0. || distance < bestDistance)
				{
					bestDistance = distance;
					best = x;
					lambda = Vector3D({ 0., 0., 0. });
					lambda[i] = 1. - t;
					lambda[(i + 1) % 3] = t;
				}
			}
			return best;
		}
		Real v = vb / sum, w = vc / sum;
		lambda = Vector3D({ 1. - v - w, v, w });
		return a + v * ab + w * ac;
	}

	//! the simplex of GJK: up to 4 vertices of C and the closest point v of their hull to the origin
	class ConvexSimplex
	{
	public:
		ConvexVertex vertex[4];
		Real lambda[4] = { 0., 0., 0., 0. };
		Index size = 0;

		Vector3D PointA() const { Vector3D p({ 0., 0., 0. }); for (Index i = 0; i < size; i++) { p += lambda[i] * vertex[i].a; } return p; }
		Vector3D PointB() const { Vector3D p({ 0., 0., 0. }); for (Index i = 0; i < size; i++) { p += lambda[i] * vertex[i].b; } return p; }

		//! keep the vertices with a weight, in their order
		void Reduce()
		{
			Index n = 0;
			for (Index i = 0; i < size; i++)
			{
				if (lambda[i] > 0.) { vertex[n] = vertex[i]; lambda[n] = lambda[i]; n++; }
			}
			size = n;
		}

		//! the closest point of the triangle of the vertices i0, i1, i2 to the origin; the weights in lambda
		Vector3D ClosestTriangle(Index i0, Index i1, Index i2, Real* weights) const
		{
			Vector3D l;
			Vector3D x = ClosestPointTriangle(Vector3D({ 0., 0., 0. }), vertex[i0].w, vertex[i1].w, vertex[i2].w, l);
			for (Index i = 0; i < 4; i++) { weights[i] = 0.; }
			weights[i0] = l[0]; weights[i1] = l[1]; weights[i2] = l[2];
			return x;
		}

		//! the closest point v of the hull to the origin, the weights, the simplex reduced to the vertices with a
		//! weight; returns true if the origin is inside the tetrahedron
		bool Closest(Vector3D& v)
		{
			bool inside = false;
			if (size == 1) { lambda[0] = 1.; v = vertex[0].w; }
			else if (size == 2)
			{
				Vector3D e = vertex[1].w - vertex[0].w;
				Real ee = e * e;
				Real t = ee > 0. ? EXUstd::Minimum(1., EXUstd::Maximum(0., -(vertex[0].w * e) / ee)) : 0.;
				lambda[0] = 1. - t; lambda[1] = t;
				v = vertex[0].w + t * e;
			}
			else if (size == 3) { v = ClosestTriangle(0, 1, 2, lambda); }
			else
			{
				//the faces of the tetrahedron that the origin is outside of; all faces if it is flat
				const Index faces[4][4] = { {0, 1, 2, 3}, {0, 1, 3, 2}, {0, 2, 3, 1}, {1, 2, 3, 0} };
				const Vector3D& w0 = vertex[0].w;
				Real volume = (vertex[1].w - w0).CrossProduct(vertex[2].w - w0) * (vertex[3].w - w0);
				Real scale = EXUstd::Maximum((vertex[1].w - w0).GetL2NormSquared(), EXUstd::Maximum((vertex[2].w - w0).GetL2NormSquared(),
					(vertex[3].w - w0).GetL2NormSquared()));
				bool flat = fabs(volume) <= 1e-14 * scale * sqrt(scale);
				Real best = -1.;
				inside = !flat;
				for (Index f = 0; f < 4; f++)
				{
					const Vector3D& a = vertex[faces[f][0]].w;
					Vector3D n = (vertex[faces[f][1]].w - a).CrossProduct(vertex[faces[f][2]].w - a);
					Real sideOrigin = -(n * a);
					Real sideOpposite = n * (vertex[faces[f][3]].w - a);
					if (flat || sideOrigin * sideOpposite < 0.)
					{
						inside = false;
						Real weights[4];
						Vector3D x = ClosestTriangle(faces[f][0], faces[f][1], faces[f][2], weights);
						Real distance = x.GetL2NormSquared();
						if (best < 0. || distance < best)
						{
							best = distance;
							v = x;
							for (Index i = 0; i < 4; i++) { lambda[i] = weights[i]; }
						}
					}
				}
				if (inside) { v = Vector3D({ 0., 0., 0. }); return true; }
			}
			Reduce();
			return inside;
		}
	};

	//! GJK on the cores of the pair: the simplex at the end and the closest point v of C to the origin; returns true if
	//! the cores overlap or touch (|v| below absoluteTolerance)
	inline bool ConvexGJK(const ConvexPair& pair, ConvexSimplex& simplex, Vector3D& v, Real absoluteTolerance, Index& iterations)
	{
		const Real relativeTolerance = 1e-12;	//on |v|^2 - v.w <= relativeTolerance |v|^2: the distance to 1e-12 relative
		const Index maxIterations = 128;
		Vector3D d = pair.HT0.GetTranslation() - pair.HT1.GetTranslation();
		if (d.GetL2NormSquared() == 0.) { d = Vector3D({ 1., 0., 0. }); }
		simplex.size = 1;
		simplex.vertex[0] = pair.Support(-1. * d);
		simplex.lambda[0] = 1.;
		v = simplex.vertex[0].w;
		Real vvLast = v.GetL2NormSquared() * 2. + 1.;
		for (iterations = 0; iterations < maxIterations; iterations++)
		{
			Real vv = v.GetL2NormSquared();
			if (vv <= absoluteTolerance * absoluteTolerance) { return true; }
			if (vv >= vvLast) { break; } //no progress by rounding: v of the last step
			vvLast = vv;
			ConvexVertex w = pair.Support(-1. * v);
			if (vv - v * w.w <= relativeTolerance * vv + absoluteTolerance * absoluteTolerance) { break; }
			bool known = false;
			for (Index i = 0; i < simplex.size; i++) { known = known || (w.w - simplex.vertex[i].w).GetL2NormSquared() == 0.; }
			if (known) { break; }
			simplex.vertex[simplex.size] = w;
			simplex.size++;
			if (simplex.Closest(v)) { return true; }
		}
		return false;
	}

	//! EPA on the cores of the pair, from the simplex of GJK whose hull contains the origin: the penetration depth,
	//! the normal of C at the deepest point (from shape 0 to shape 1) and the points of both cores; returns false if
	//! C is flat (the cores have no common volume), with the depth 0 and the normal of the flat C
	class ConvexEPA
	{
	public:
		class Face
		{
		public:
			Index i[3];
			Vector3D n;
			Real d;
			bool alive;
		};
		std::vector<ConvexVertex> vertices;
		std::vector<Face> faces;
		std::vector<Index> edges;	//directed edges of the horizon, two indices each
		std::vector<Index> visible;	//the faces seen by a new point
		std::vector<bool> takenFaces;

		bool AddFace(Index i0, Index i1, Index i2)
		{
			Face f;
			f.i[0] = i0; f.i[1] = i1; f.i[2] = i2;
			const Vector3D& a = vertices[i0].w;
			f.n = (vertices[i1].w - a).CrossProduct(vertices[i2].w - a);
			Real length = f.n.GetL2Norm();
			if (length == 0.) { return false; }
			f.n *= 1. / length;
			f.d = f.n * a;
			f.alive = true;
			faces.push_back(f);
			return true;
		}

		//! the point added to the hull: the faces it sees, connected to the face it sees most, removed and the
		//! horizon closed with new faces; nothing changed and false if it sees none or a new face is degenerate
		bool AddPoint(const ConvexVertex& w, Real tolerance)
		{
			const Index nFaces = (Index)faces.size();
			Index seed = -1;
			Real seedHeight = tolerance;
			for (Index j = 0; j < nFaces; j++)
			{
				Real height = faces[j].n * (w.w - vertices[faces[j].i[0]].w);
				faces[j].alive = height <= tolerance;		//alive: not seen
				if (height > seedHeight) { seedHeight = height; seed = j; }
			}
			if (seed == -1) { for (Face& f : faces) { f.alive = true; } return false; }
			//the seen faces connected to the seed by edges; the others stay
			visible.clear();
			visible.push_back(seed);
			std::vector<bool>& taken = takenFaces;
			taken.assign(nFaces, false);
			taken[seed] = true;
			for (Index k = 0; k < (Index)visible.size(); k++)
			{
				const Face& f = faces[visible[k]];
				for (Index j = 0; j < nFaces; j++)
				{
					if (taken[j] || faces[j].alive) { continue; }
					bool neighbor = false;
					for (Index a = 0; a < 3 && !neighbor; a++)
					{
						for (Index b = 0; b < 3 && !neighbor; b++)
						{
							neighbor = f.i[a] == faces[j].i[(b + 1) % 3] && f.i[(a + 1) % 3] == faces[j].i[b];
						}
					}
					if (neighbor) { taken[j] = true; visible.push_back(j); }
				}
			}
			//the horizon: the edges of the visible faces not shared with another visible face
			edges.clear();
			for (Index k : visible)
			{
				for (Index a = 0; a < 3; a++)
				{
					Index e0 = faces[k].i[a], e1 = faces[k].i[(a + 1) % 3];
					bool removed = false;
					for (Index j = 0; j < (Index)edges.size() && !removed; j += 2)
					{
						if (edges[j] == e1 && edges[j + 1] == e0)
						{
							edges.erase(edges.begin() + j, edges.begin() + j + 2);
							removed = true;
						}
					}
					if (!removed) { edges.push_back(e0); edges.push_back(e1); }
				}
			}
			//the new faces must not be degenerate
			bool valid = edges.size() >= 6;
			for (Index j = 0; j < (Index)edges.size() && valid; j += 2)
			{
				const Vector3D& a = vertices[edges[j]].w;
				Vector3D n = (vertices[edges[j + 1]].w - a).CrossProduct(w.w - a);
				valid = n.GetL2NormSquared() > 0. && n * (w.w - a) == n * (w.w - a); //not zero, not NaN
			}
			if (!valid) { for (Face& f : faces) { f.alive = true; } return false; }
			for (Index j = 0; j < nFaces; j++) { faces[j].alive = !taken[j]; }
			Index index = (Index)vertices.size();
			vertices.push_back(w);
			for (Index j = 0; j < (Index)edges.size(); j += 2) { AddFace(edges[j], edges[j + 1], index); }
			Index n = 0;
			for (Index j = 0; j < (Index)faces.size(); j++) { if (faces[j].alive) { faces[n++] = faces[j]; } }
			faces.resize(n);
			return true;
		}

		bool Compute(const ConvexPair& pair, const ConvexSimplex& simplex, Real tolerance, Real& depth, Vector3D& normal,
			Vector3D& pointA, Vector3D& pointB, Index& iterations)
		{
			const Index maxIterations = 128;
			vertices.clear();
			faces.clear();
			//the candidates: the simplex and the supports along the axes, whose hull contains the origin
			std::vector<ConvexVertex> candidates;
			for (Index i = 0; i < simplex.size; i++) { candidates.push_back(simplex.vertex[i]); }
			for (Index k = 0; k < 3; k++)
			{
				Vector3D e({ 0., 0., 0. });
				e[k] = 1.;
				candidates.push_back(pair.Support(e));
				candidates.push_back(pair.Support(-1. * e));
			}
			//the tetrahedron of the largest volume, greedily
			Index n = (Index)candidates.size();
			Index t[4] = { 0, 0, 0, 0 };
			Real best = 0.;
			for (Index i = 1; i < n; i++)
			{
				Real value = (candidates[i].w - candidates[0].w).GetL2NormSquared();
				if (value > best) { best = value; t[1] = i; }
			}
			Real scale = sqrt(best);
			Vector3D lineDirection = candidates[t[1]].w - candidates[0].w;
			best = 0.;
			for (Index i = 0; i < n; i++)
			{
				Real value = lineDirection.CrossProduct(candidates[i].w - candidates[0].w).GetL2NormSquared();
				if (value > best) { best = value; t[2] = i; }
			}
			Vector3D planeNormal = lineDirection.CrossProduct(candidates[t[2]].w - candidates[0].w);
			Real bestVolume = 0.;
			for (Index i = 0; i < n; i++)
			{
				Real value = fabs(planeNormal * (candidates[i].w - candidates[0].w));
				if (value > bestVolume) { bestVolume = value; t[3] = i; }
			}
			depth = 0.;
			pointA = simplex.PointA();
			pointB = simplex.PointB();
			if (scale == 0. || best <= 1e-20 * scale * scale * scale * scale || bestVolume <= 1e-10 * scale * sqrt(best))
			{
				//C flat: its plane normal, or a normal of its line
				if (scale != 0. && best > 1e-20 * scale * scale * scale * scale) { normal = planeNormal; }
				else
				{
					Vector3D n1, n2;
					Vector3D axis = scale != 0. ? lineDirection : Vector3D({ 0., 0., 1. });
					EXUmath::ComputeOrthogonalBasisVectors(axis, n1, n2);
					normal = scale != 0. ? n1 : axis;
				}
				normal *= 1. / normal.GetL2Norm();
				if (normal * (pair.HT1.GetTranslation() - pair.HT0.GetTranslation()) < 0.) { normal *= -1.; }
				return false;
			}
			for (Index k = 0; k < 4; k++) { vertices.push_back(candidates[t[k]]); }
			if ((vertices[1].w - vertices[0].w).CrossProduct(vertices[2].w - vertices[0].w) * (vertices[3].w - vertices[0].w) < 0.)
			{
				std::swap(vertices[1], vertices[2]);
			}
			AddFace(0, 2, 1); AddFace(0, 1, 3); AddFace(1, 2, 3); AddFace(0, 3, 2);
			for (Index i = 0; i < n; i++)
			{
				if (i != t[0] && i != t[1] && i != t[2] && i != t[3]) { AddPoint(candidates[i], tolerance); }
			}
			//expand towards the boundary of C nearest to the origin
			Index nearest = 0;
			for (iterations = 0; iterations < maxIterations; iterations++)
			{
				nearest = 0;
				for (Index j = 1; j < (Index)faces.size(); j++) { if (faces[j].d < faces[nearest].d) { nearest = j; } }
				Face f = faces[nearest];
				ConvexVertex w = pair.Support(f.n);
				if (w.w * f.n - f.d <= tolerance) { break; }
				if (!AddPoint(w, tolerance)) { break; }
			}
			nearest = 0;
			for (Index j = 1; j < (Index)faces.size(); j++) { if (faces[j].d < faces[nearest].d) { nearest = j; } }
			const Face& f = faces[nearest];
			depth = EXUstd::Maximum(0., f.d);
			normal = f.n;
			Vector3D lambda;
			ClosestPointTriangle(depth * f.n, vertices[f.i[0]].w, vertices[f.i[1]].w, vertices[f.i[2]].w, lambda);
			pointA = lambda[0] * vertices[f.i[0]].a + lambda[1] * vertices[f.i[1]].a + lambda[2] * vertices[f.i[2]].a;
			pointB = lambda[0] * vertices[f.i[0]].b + lambda[1] * vertices[f.i[1]].b + lambda[2] * vertices[f.i[2]].b;
			return true;
		}
	};

	//! the point of the core of the shape in the frame HT closest to the global point p; p itself if it is inside
	inline Vector3D ClosestPointCore(const ConvexShape& shape, const HomogeneousTransformation& HT, const Vector3D& p)
	{
		Vector3D x = HT.RotateVectorTransposed(p - HT.GetTranslation());
		Vector3D q = x;
		Real h = 0.5 * shape.length;
		switch (shape.type)
		{
		case ConvexShapeType::Sphere: q = Vector3D({ 0., 0., 0. }); break;
		case ConvexShapeType::Capsule: q = Vector3D({ 0., 0., EXUstd::Minimum(h, EXUstd::Maximum(-h, x[2])) }); break;
		case ConvexShapeType::Cylinder:
		{
			Real rho = sqrt(x[0] * x[0] + x[1] * x[1]);
			if (rho > shape.radius) { q[0] *= shape.radius / rho; q[1] *= shape.radius / rho; }
			q[2] = EXUstd::Minimum(h, EXUstd::Maximum(-h, x[2]));
			break;
		}
		case ConvexShapeType::Box:
			for (Index i = 0; i < 3; i++) { q[i] = EXUstd::Minimum(shape.halfSize[i], EXUstd::Maximum(-shape.halfSize[i], x[i])); }
			break;
		case ConvexShapeType::Triangle:
		{
			Vector3D lambda;
			q = ClosestPointTriangle(x, shape.points[0], shape.points[1], shape.points[2], lambda);
			break;
		}
		default: //the hull of the points by GJK of the point against it, finite for a polytope
		{
			ConvexShape point;
			HomogeneousTransformation HTpoint, HTshape;
			HTpoint.GetTranslation() = x;
			ConvexPair pair(point, HTpoint, shape, HTshape);
			ConvexSimplex simplex;
			Vector3D v;
			Index iterations;
			if (!ConvexGJK(pair, simplex, v, 0., iterations)) { q = simplex.PointB(); }
		}
		}
		return HT.GetTranslation() + HT.RotateVector(q);
	}

	//! the sphere (radius sphereRadius, center HTsphere) and the cylinder by the analytic formula; the result for the
	//! sphere as shape 0
	inline void DistanceSphereCylinder(Real sphereRadius, const Vector3D& center, const ConvexShape& cylinder,
		const HomogeneousTransformation& HTcylinder, ConvexDistanceResult& result)
	{
		Vector3D c = HTcylinder.RotateVectorTransposed(center - HTcylinder.GetTranslation());
		Real r = cylinder.radius, h = 0.5 * cylinder.length;
		Real rho = sqrt(c[0] * c[0] + c[1] * c[1]);
		Real zSign = c[2] >= 0. ? 1. : -1.;
		Vector3D radial = rho > 0. ? Vector3D({ c[0] / rho, c[1] / rho, 0. }) : Vector3D({ 1., 0., 0. });
		Vector3D q, n;		//the closest point of the cylinder, the normal from the cylinder to the sphere
		Real distance;
		if (rho <= r && fabs(c[2]) <= h) //the center inside: the nearer of the side and the cap
		{
			if (r - rho < h - fabs(c[2]))
			{
				q = Vector3D({ r * radial[0], r * radial[1], c[2] }); n = radial; distance = -(r - rho);
			}
			else
			{
				q = Vector3D({ c[0], c[1], zSign * h }); n = Vector3D({ 0., 0., zSign }); distance = -(h - fabs(c[2]));
			}
		}
		else if (rho <= r) { q = Vector3D({ c[0], c[1], zSign * h }); n = Vector3D({ 0., 0., zSign }); distance = fabs(c[2]) - h; }
		else if (fabs(c[2]) <= h) { q = Vector3D({ r * radial[0], r * radial[1], c[2] }); n = radial; distance = rho - r; }
		else
		{
			q = Vector3D({ r * radial[0], r * radial[1], zSign * h });
			n = c - q;
			distance = n.GetL2Norm();
			n *= 1. / distance;
		}
		n = HTcylinder.RotateVector(n);
		result.distance = distance - sphereRadius;
		result.point1 = HTcylinder.GetTranslation() + HTcylinder.RotateVector(q);
		result.point0 = center - sphereRadius * n;
		result.normal = -1. * n;
		result.iterations = 0;
	}

	//! the signed distance of the cores of two shapes (without the margins), their closest points and the normal from
	//! shape 0 to shape 1; with refine, a penetration involving a cylinder is refined from the result of EPA
	inline Real ConvexDistanceCores(const ConvexShape& shape0, const HomogeneousTransformation& HT0, const ConvexShape& shape1,
		const HomogeneousTransformation& HT1, Vector3D& pointA, Vector3D& pointB, Vector3D& normal, Index& iterations, bool refine)
	{
		ConvexPair pair(shape0, HT0, shape1, HT1);
		Real scale = shape0.BoundingRadius() + shape1.BoundingRadius() + (HT0.GetTranslation() - HT1.GetTranslation()).GetL2Norm();
		if (scale == 0.) { scale = 1.; }
		const Real absoluteTolerance = 1e-14 * scale;
		const bool cylinder = shape0.type == ConvexShapeType::Cylinder || shape1.type == ConvexShapeType::Cylinder;
		ConvexSimplex simplex;
		Vector3D v;
		Index count = 0;
		bool overlap = ConvexGJK(pair, simplex, v, absoluteTolerance, count);
		iterations = count;
		if (!overlap)
		{
			pointA = simplex.PointA();
			pointB = simplex.PointB();
			Real coreDistance = v.GetL2Norm();
			normal = (-1. / coreDistance) * v;
			//GJK stalls on the side of a cylinder, whose points are combinations of its rims at slightly different
			//angles; alternating projections of the closest points onto the cores, which only lower the distance
			if (cylinder)
			{
				for (Index k = 0; k < 64; k++)
				{
					Vector3D a = ClosestPointCore(shape0, HT0, pointB);
					Vector3D b = ClosestPointCore(shape1, HT1, a);
					Real distance = (b - a).GetL2Norm();
					if (!(distance < coreDistance) || distance == 0.) { break; }
					bool converged = coreDistance - distance <= 1e-15 * coreDistance;
					pointA = a;
					pointB = b;
					coreDistance = distance;
					normal = (1. / distance) * (b - a);
					iterations++;
					if (converged) { break; }
				}
			}
			return coreDistance;
		}
		ConvexEPA epa;
		Real depth;
		bool solid = epa.Compute(pair, simplex, 1e-10 * scale, depth, normal, pointA, pointB, count);
		iterations += count;
		if (!(refine && cylinder && solid)) { return -depth; }
		//EPA converges slowly on a cylinder: shape 1 moved by T = (d + delta) n out of shape 0, the closest point of
		//C = A - B to T is a point p of its boundary with the normal m; T along m again, until p is parallel to m.
		//p.m is the support value of C in m, an upper bound of the depth, as the support value in the normal of EPA:
		//the smallest bound is kept
		ConvexVertex w = pair.Support(normal);
		Real bestDepth = w.w * normal;
		Vector3D bestNormal = normal, bestA = pointA, bestB = pointB;
		Real delta = 0.1 * scale;
		Real d = bestDepth;
		HomogeneousTransformation HT1moved = HT1;
		for (Index k = 0; k < 32; k++)
		{
			Vector3D T = (d + delta) * normal;
			HT1moved.GetTranslation() = HT1.GetTranslation() + T;
			Vector3D a, b, m;
			Index separatedIterations;
			Real gap = ConvexDistanceCores(shape0, HT0, shape1, HT1moved, a, b, m, separatedIterations, false);
			iterations += separatedIterations;
			if (!(gap > 0.)) { delta *= 2.; continue; } //not out yet
			d = (T - gap * m) * m;
			normal = m;
			if (d < bestDepth)
			{
				bool converged = bestDepth - d <= 1e-14 * scale;
				bestDepth = d; bestNormal = m; bestA = a; bestB = b - T;
				if (converged) { break; }
			}
			else if (d > bestDepth + 1e-14 * scale) { break; } //away from the minimum
		}
		depth = EXUstd::Maximum(0., bestDepth);
		normal = bestNormal;
		pointA = bestA;
		pointB = bestB;
		return -depth;
	}

	//! the signed distance of two convex shapes in their frames, the closest points and the normal from shape 0 to
	//! shape 1; analytic: the sphere and the cylinder by the formula, else everything by GJK and EPA
	inline void ConvexDistance(const ConvexShape& shape0, const HomogeneousTransformation& HT0, const ConvexShape& shape1,
		const HomogeneousTransformation& HT1, ConvexDistanceResult& result, bool analytic = true)
	{
		if (analytic && shape0.type == ConvexShapeType::Sphere && shape1.type == ConvexShapeType::Cylinder)
		{
			DistanceSphereCylinder(shape0.radius, HT0.GetTranslation(), shape1, HT1, result);
			return;
		}
		if (analytic && shape0.type == ConvexShapeType::Cylinder && shape1.type == ConvexShapeType::Sphere)
		{
			DistanceSphereCylinder(shape1.radius, HT1.GetTranslation(), shape0, HT0, result);
			std::swap(result.point0, result.point1);
			result.normal *= -1.;
			return;
		}
		Real margin0 = shape0.Margin(), margin1 = shape1.Margin();
		Vector3D pointA, pointB;
		Real coreDistance = ConvexDistanceCores(shape0, HT0, shape1, HT1, pointA, pointB, result.normal, result.iterations, true);
		result.distance = coreDistance - margin0 - margin1;
		result.point0 = pointA + margin0 * result.normal;
		result.point1 = pointB - margin1 * result.normal;
	}

} //namespace EGeometry

#endif
