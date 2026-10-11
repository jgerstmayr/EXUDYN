/** ***********************************************************************************************
* @class		VisualizationAccessCache
* @brief		positions and rotation matrices of a body at local positions in the visualization configuration,
*				for a renderer thread that may not call Python (#2986)
* @details		Details:
*				- a body whose access functions are Python user functions (ObjectUserBody) cannot compute a position
*				  in the renderer thread; the renderer asks the cache, which registers a local position it does not
*				  know, and the computation thread fills the cache when it draws the user functions
*				- the cache is guarded by an atomic flag; a copy starts empty
*
* @author		Gerstmayr Johannes
* @date			2026-10-12
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#ifndef VISUALIZATIONACCESSCACHE__H
#define VISUALIZATIONACCESSCACHE__H

#include <atomic>
#include <vector>
#include "Utilities/AdvancedStuff.h"
#include "Linalg/BasicLinalg.h"

class VisualizationAccessCache
{
public:
	static constexpr Index maxNumberOfEntries = 1000; //a body with more local positions is not drawn completely

	VisualizationAccessCache() {}
	VisualizationAccessCache(const VisualizationAccessCache& other) {}
	VisualizationAccessCache& operator=(const VisualizationAccessCache& other) { return *this; }

	//! position and rotation matrix at localPosition; if not available, localPosition is registered, position is zero
	//! and rotation the unit matrix, and false is returned
	bool Get(const Vector3D& localPosition, Vector3D& position, Matrix3D& rotation)
	{
		EXUstd::WaitAndLockSemaphore(lock);
		bool found = false;
		for (size_t i = 0; i < localPositions.size(); i++)
		{
			if (localPositions[i] == localPosition)
			{
				position = positions[i];
				rotation = rotations[i];
				found = true;
				break;
			}
		}
		if (!found)
		{
			position = Vector3D(0.);
			rotation = EXUmath::unitMatrix3D;
			if ((Index)localPositions.size() < maxNumberOfEntries)
			{
				localPositions.push_back(localPosition);
				positions.push_back(position);
				rotations.push_back(rotation);
			}
		}
		EXUstd::ReleaseSemaphore(lock);
		return found;
	}

	//! the local positions registered so far
	std::vector<Vector3D> GetLocalPositions()
	{
		EXUstd::WaitAndLockSemaphore(lock);
		std::vector<Vector3D> list = localPositions;
		EXUstd::ReleaseSemaphore(lock);
		return list;
	}

	//! set position and rotation matrix at localPosition, which GetLocalPositions returned
	void Set(const Vector3D& localPosition, const Vector3D& position, const Matrix3D& rotation)
	{
		EXUstd::WaitAndLockSemaphore(lock);
		for (size_t i = 0; i < localPositions.size(); i++)
		{
			if (localPositions[i] == localPosition)
			{
				positions[i] = position;
				rotations[i] = rotation;
				break;
			}
		}
		EXUstd::ReleaseSemaphore(lock);
	}

private:
	std::atomic_flag lock = ATOMIC_FLAG_INIT;
	std::vector<Vector3D> localPositions;
	std::vector<Vector3D> positions;
	std::vector<Matrix3D> rotations;
};

#endif
