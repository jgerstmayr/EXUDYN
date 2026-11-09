/** ***********************************************************************************************
* @brief		class PyCollisionModel: exudyn.cUtilities.CollisionModel, the collision check of a robot and its environment
*				for path planning (#3380)
* @details		Details:
*				- a KinematicChain, convex shapes on the links and on the base, obstacles fixed in the world
*				- the pairs of shapes: robot against the obstacles and links against each other, except a link and its
*				  parent and pairs excluded by ExcludeLinkPair; the shapes are filtered by bounding spheres
*				- the distances by EGeometry::ConvexDistance; bound in definitions/pybindCUtilities.py
*
* @author		Gerstmayr Johannes
* @date			2026-11-09
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#ifndef PYCOLLISIONMODEL__H
#define PYCOLLISIONMODEL__H

#include <set>
#include <utility>
#include <limits>
#include "Pymodules/PyKinematicChain.h"
#include "Pymodules/PyConvexShape.h"

class PyCollisionModel
{
	class Item
	{
	public:
		Index link;						//!< the link of the shape, -1 the base, -2 an obstacle
		PyConvexShape shape;
		HomogeneousTransformation local;	//!< in the frame of the link; of an obstacle the global frame
		Real boundingRadius;
	};
	PyKinematicChain chain;
	std::vector<Item> items;
	std::set<std::pair<Index, Index>> excluded;

	void AddItem(Index link, const PyConvexShape& shape, const py::object& HTpy, const char* name)
	{
		Item item{ link, shape, HomogeneousTransformation(), 0. };
		if (!HTpy.is_none()) { EPyUtils::HTFromPython(HTpy, item.local, name); }
		item.boundingRadius = shape.BoundingRadius();
		items.push_back(item);
	}

	//! false: this pair of links or base is not checked
	bool PairChecked(Index link0, Index link1) const
	{
		if (link0 == link1) { return false; }			//the same link or both fixed in the world
		if (link0 == -2 || link1 == -2) { return !(link0 < 0 && link1 < 0); } //an obstacle: not against the base
		if (link0 >= 0 && chain.GetParent(link0) == link1) { return false; }
		if (link1 >= 0 && chain.GetParent(link1) == link0) { return false; }
		return excluded.count({ EXUstd::Minimum(link0, link1), EXUstd::Maximum(link0, link1) }) == 0;
	}

	//! the smallest distance at q; stops if it finds one not larger than stopBelow
	Real Compute(const Vector& q, Real stopBelow) const
	{
		ResizableArray<HomogeneousTransformation> jointFrames, linkFrames;
		chain.ComputeFrames(q, jointFrames, linkFrames);
		const Index n = (Index)items.size();
		std::vector<HomogeneousTransformation> frames(items.size());
		for (Index i = 0; i < n; i++)
		{
			const Item& item = items[i];
			frames[i] = (item.link >= 0) ? linkFrames[item.link] * item.local : ((item.link == -1) ? chain.GetBaseHT() * item.local : item.local);
		}
		Real minimum = std::numeric_limits<Real>::max();
		for (Index i = 0; i < n; i++)
		{
			for (Index j = i + 1; j < n; j++)
			{
				if (!PairChecked(items[i].link, items[j].link)) { continue; }
				Real bound = (frames[i].GetTranslation() - frames[j].GetTranslation()).GetL2Norm() - items[i].boundingRadius - items[j].boundingRadius;
				if (bound >= minimum) { continue; }
				EGeometry::ConvexDistanceResult result;
				EGeometry::ConvexDistance(items[i].shape, frames[i], items[j].shape, frames[j], result);
				minimum = EXUstd::Minimum(minimum, result.distance);
				if (minimum <= stopBelow) { return minimum; }
			}
		}
		return minimum;
	}

public:
	static PyCollisionModel* ForbidConstructor()
	{
		PyError("exudyn.cUtilities.CollisionModel() needs a KinematicChain", PyErrorType::typeError);
		return nullptr; //never reached
	}

	PyCollisionModel(const PyKinematicChain& chainInit) : chain(chainInit) {}

	//! a shape on link linkIndex (-1 the base) in the frame HT of that link
	void AddLinkShape(Index linkIndex, const PyConvexShape& shape, const py::object& HT)
	{
		if (linkIndex < -1 || linkIndex >= chain.NumberOfLinks()) { PyError("CollisionModel.AddLinkShape: linkIndex must be -1 (the base) or a link of the chain, not " + EXUstd::ToString(linkIndex), PyErrorType::valueError); }
		AddItem(linkIndex, shape, HT, "CollisionModel.AddLinkShape: HT");
	}

	//! a shape fixed in the world, in the global frame HT
	void AddObstacle(const PyConvexShape& shape, const py::object& HT) { AddItem(-2, shape, HT, "CollisionModel.AddObstacle: HT"); }

	//! this pair of links is not checked
	void ExcludeLinkPair(Index link0, Index link1) { excluded.insert({ EXUstd::Minimum(link0, link1), EXUstd::Maximum(link0, link1) }); }

	Index NumberOfShapes() const { return (Index)items.size(); }

	//! the smallest signed distance over all pairs at the joint coordinates q
	Real MinimumDistance(const py::object& q) const { return Compute(chain.GetCoordinates(q, "CollisionModel.MinimumDistance"), -std::numeric_limits<Real>::max()); }

	//! true if all distances at q are larger than margin
	bool IsFree(const py::object& q, Real margin) const { return Compute(chain.GetCoordinates(q, "CollisionModel.IsFree"), margin) > margin; }

	//! true if the straight joint motion from q0 to q1 is free, checked at steps of at most maxStep in any joint coordinate
	bool IsEdgeFree(const py::object& q0py, const py::object& q1py, Real margin, Real maxStep) const
	{
		if (!(maxStep > 0.)) { PyError("CollisionModel.IsEdgeFree: maxStep must be positive", PyErrorType::valueError); }
		Vector q0 = chain.GetCoordinates(q0py, "CollisionModel.IsEdgeFree"), q1 = chain.GetCoordinates(q1py, "CollisionModel.IsEdgeFree");
		Real largest = 0.;
		for (Index i = 0; i < q0.NumberOfItems(); i++) { largest = EXUstd::Maximum(largest, fabs(q1[i] - q0[i])); }
		Index steps = EXUstd::Maximum((Index)ceil(largest / maxStep), (Index)1);
		Vector q(q0);
		for (Index k = 0; k <= steps; k++)
		{
			Real x = (Real)k / (Real)steps;
			for (Index i = 0; i < q.NumberOfItems(); i++) { q[i] = (1. - x) * q0[i] + x * q1[i]; }
			if (Compute(q, margin) <= margin) { return false; }
		}
		return true;
	}

	STDstring ToString() const { return "CollisionModel(" + EXUstd::ToString(chain.NumberOfLinks()) + " links, " + EXUstd::ToString((Index)items.size()) + " shapes)"; }
};

#endif
