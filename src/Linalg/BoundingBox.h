/** ***********************************************************************************************
* @class	    SearchTree
* @brief		Class for boxed search
* @details		Details:
*               allows to find objects in space efficiently
*
* @author		Gerstmayr Johannes
* @date			2021-10-23 (generated)
* @pre			...
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
#ifndef BOUDNINGBOX__H
#define BOUDNINGBOX__H

#include "Linalg/BasicLinalg.h"		//includes Vector.h


template<typename T>
class Box3DBase
{
public:
	using Vector3T = SlimVectorBase<T, 3>;

	Box3DBase(bool clear = true)
	{
		if (clear)
		{
			Clear();
		}
	}

	Box3DBase(const Vector3T& p1, const Vector3T& p2)
	{
		pmin[0] = EXUstd::Minimum(p1[0], p2[0]);
		pmin[1] = EXUstd::Minimum(p1[1], p2[1]);
		pmin[2] = EXUstd::Minimum(p1[2], p2[2]);

		pmax[0] = EXUstd::Maximum(p1[0], p2[0]);
		pmax[1] = EXUstd::Maximum(p1[1], p2[1]);
		pmax[2] = EXUstd::Maximum(p1[2], p2[2]);
	}

	Box3DBase(const Box3DBase& b)
	{
		for (int i = 0; i < 3; ++i)
		{
			pmin[i] = b.pmin[i];
			pmax[i] = b.pmax[i];
		}
	}

	Box3DBase(const Vector3T& c, T r)
	{
		for (int i = 0; i < 3; ++i)
		{
			pmin[i] = pmax[i] = c[i];
		}
		Increase(r);
	}

	//! check if box is empty, only based on x-value!
	bool Empty() const
	{
		return pmin[0] == std::numeric_limits<T>::max();
	}

	void Clear()
	{
		for (int i = 0; i < 3; ++i)
		{
			pmin[i] = std::numeric_limits<T>::max();
			pmax[i] = std::numeric_limits<T>::lowest();
		}
	}

	void Add(const Vector3T& p)
	{
		if (Empty())
		{
			for (int i = 0; i < 3; ++i) { pmin[i] = pmax[i] = p[i]; }
		}
		else
		{
			for (int i = 0; i < 3; ++i)
			{
				pmin[i] = EXUstd::Minimum(pmin[i], p[i]);
				pmax[i] = EXUstd::Maximum(pmax[i], p[i]);
			}
		}
	}

	void Add(const Box3DBase& b)
	{
		if (b.Empty()) { return; }

		if (Empty())
		{
			for (int i = 0; i < 3; ++i)
			{
				pmin[i] = b.pmin[i];
				pmax[i] = b.pmax[i];
			}
		}
		else
		{
			for (int i = 0; i < 3; ++i)
			{
				pmin[i] = EXUstd::Minimum(pmin[i], b.pmin[i]);
				pmax[i] = EXUstd::Maximum(pmax[i], b.pmax[i]);
			}
		}
	}

	//! get c-arrays
	const T* PMinC() const { return pmin; }
	const T* PMaxC() const { return pmax; }
	
	//! convert to Vector3D; const added to raise error if assignment happens e.g. box.PMin() = ...
	const Vector3T PMin() const { return Vector3T({ pmin[0], pmin[1], pmin[2] }); }
	const Vector3T PMax() const { return Vector3T({ pmax[0], pmax[1], pmax[2] }); }

	//! direct read access to Reals:
	EXUINLINE const T& PMinX() const { return pmin[0]; }
	EXUINLINE const T& PMinY() const { return pmin[1]; }
	EXUINLINE const T& PMinZ() const { return pmin[2]; }
	EXUINLINE const T& PMaxX() const { return pmax[0]; }
	EXUINLINE const T& PMaxY() const { return pmax[1]; }
	EXUINLINE const T& PMaxZ() const { return pmax[2]; }

	//! direct write access to Reals:
	EXUINLINE T& PMinX() { return pmin[0]; }
	EXUINLINE T& PMinY() { return pmin[1]; }
	EXUINLINE T& PMinZ() { return pmin[2]; }
	EXUINLINE T& PMaxX() { return pmax[0]; }
	EXUINLINE T& PMaxY() { return pmax[1]; }
	EXUINLINE T& PMaxZ() { return pmax[2]; }

	//! set with Vector3D
	EXUINLINE void SetPMin(const Vector3T& p) { for (int i = 0; i < 3; ++i) pmin[i] = p[i]; }
	EXUINLINE void SetPMax(const Vector3T& p) { for (int i = 0; i < 3; ++i) pmax[i] = p[i]; }

	T SizeX() const { return pmax[0] - pmin[0]; }
	T SizeY() const { return pmax[1] - pmin[1]; }
	T SizeZ() const { return pmax[2] - pmin[2]; }

	Vector3T Center() const { return Vector3T({ (pmin[0] + pmax[0]) * (T)0.5, (pmin[1] + pmax[1]) * (T)0.5, (pmin[2] + pmax[2]) * (T)0.5 }); }
	T Radius() const { return (PMax() - PMin()).GetL2Norm() * (T)0.5; }

	void Increase(T x, T y, T z)
	{
		pmin[0] -= x; pmin[1] -= y; pmin[2] -= z;
		pmax[0] += x; pmax[1] += y; pmax[2] += z;
	}
	void Increase(T x) { Increase(x, x, x); }

	void InflateFactor(T x)
	{
		Vector3T pc = Center();
		for (int i = 0; i < 3; ++i)
		{
			pmin[i] = pc[i] + (pmin[i] - pc[i]) * x;
			pmax[i] = pc[i] + (pmax[i] - pc[i]) * x;
		}
	}

	//! check if box b intersects with this (boundary included)
	bool Intersect(const Box3DBase& b) const
	{
		//> and < changed to >= and <= in order to simplify problems with points on boundaries
		if (pmin[0] >= b.pmax[0] || pmax[0] <= b.pmin[0]
			|| pmin[1] >= b.pmax[1] || pmax[1] <= b.pmin[1]
			|| pmin[2] >= b.pmax[2] || pmax[2] <= b.pmin[2])
		{
			return false;
		}
		return true;
	}

	//! return true if point p in closure or on boundary
	bool IsInside(const Vector3T& p) const
	{
		for (int i = 0; i < 3; ++i)
		{
			if (p[i] < pmin[i] || p[i] > pmax[i]) return false;
		}
		return true;
	}

	//! Function to check intersection of Box3D and triangle given by vertices v0, v1 and v2
	//! returns true, if box and triangle intersect
	bool BoxTriangleIntersect(const Vector3T& v0, const Vector3T& v1, const Vector3T& v2)
	{
		Vector3T boxCenter = this->Center();
		Vector3T boxHalfSize = (this->PMax() - this->PMin()) * (T)0.5;

		// Move triangle relative to AABB center
		Vector3T v0rel = v0 - boxCenter;
		Vector3T v1rel = v1 - boxCenter;
		Vector3T v2rel = v2 - boxCenter;

		// Triangle edges
		Vector3T edge1 = v1rel - v0rel;
		Vector3T edge2 = v2rel - v1rel;
		Vector3T edge3 = v0rel - v2rel;

		// AABB face normals (x, y, z axes)
		for (int i = 0; i < 3; i++)
		{
			if (std::max(v0rel[i], std::max(v1rel[i], v2rel[i])) < -boxHalfSize[i] ||
				std::min(v0rel[i], std::min(v1rel[i], v2rel[i])) > boxHalfSize[i]) {
				return false; // Separating axis found
			}
		}

		// Triangle normal
		Vector3T triNormal = edge1.CrossProduct(edge2);
		if (!(triNormal == 0))
		{
			triNormal.NormalizeSafe();
			if (!EXUmath::TriangleOverlapOnAxis(triNormal, v0rel, v1rel, v2rel, boxHalfSize)) { return false; }
		}

		// Test cross products of box axes and triangle edges
		std::array<Vector3T, 3> boxAxes = { Vector3T({1, 0, 0}), Vector3T({0, 1, 0}), Vector3T({0, 0, 1}) };
		std::array<Vector3T, 3> edges = { edge1, edge2, edge3 };

		for (const auto& edge : edges)
		{
			for (const auto& axis : boxAxes)
			{
				Vector3T testAxis = axis.CrossProduct(edge);
				T axisLength = testAxis.GetL2Norm();
				if (axisLength > 1e-12)
				{
					if (!EXUmath::TriangleOverlapOnAxis(((T)1. / axisLength) * testAxis, v0rel, v1rel, v2rel, boxHalfSize))
					{
						return false;
					}
				}
			}
		}

		return true; // No separating axis found, intersection exists
	}


	//! @brief Output operator for Box3D
	friend std::ostream& operator<<(std::ostream& os, const Box3DBase& v)
	{
		os << "{" << v.PMin() << ", " << v.PMax() << "}";
		return os;
	}

private:
	T pmin[3];
	T pmax[3];
};

typedef Box3DBase<Real> Box3D;
typedef Box3DBase<float> Box3DF;


#endif
