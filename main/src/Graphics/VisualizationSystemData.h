/** ***********************************************************************************************
* @class        VisualizationSystemData
* @brief		A container similar to CSystemData, containing all visualization objects
* @details		Details:
* 				- a container for visualization items (nodes, objects, markers, ...)
*				- objects are created (allocate with new) and deleted here
*				- material is not visualized
*
* @author		Gerstmayr Johannes
* @date			2019-05-27 (generated)
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
#ifndef VISUALIZATIONSYSTEMDATA__H
#define VISUALIZATIONSYSTEMDATA__H

#include "Utilities/ResizableArray.h"

#include "System/VisualizationNode.h"
#include "System/VisualizationObject.h"
#include "System/VisualizationMarker.h"
#include "System/VisualizationLoad.h"
#include "System/VisualizationSensor.h"


class VisualizationSystemData
{
protected:
	ResizableArray<VisualizationObject*> vObjects;              //!< container for visualization objects
	ResizableArray<VisualizationNode*> vNodes;                  //!< container for visualization nodes
	ResizableArray<VisualizationMarker*> vMarkers;              //!< container for visualization markers
	ResizableArray<VisualizationLoad*> vLoads;                  //!< container for visualization loads
	ResizableArray<VisualizationSensor*> vSensors;              //!< container for visualization sensors

public:

	//! clone object; specifically for copying instances of derived class, for automatic memory management e.g. in ObjectContainer
	VisualizationSystemData* GetClone() const { return new VisualizationSystemData(*this); }
	//! Specific destructor do deallocate data (allocated in MainSystem/ObjectFactory)
	virtual ~VisualizationSystemData() { Reset(); }

	//! reset VisualizationSystemData and deallocate all memory (call this at end of system!)
	void Reset();

	//! Write (Reference) access to:container for visualization objects
	ResizableArray<VisualizationObject*>& GetVisualizationObjects() { return vObjects; }
	//! Read (Reference) access to:container for visualization objects
	const ResizableArray<VisualizationObject*>& GetVisualizationObjects() const { return vObjects; }

	//! Write (Reference) access to:container for visualization nodes
	ResizableArray<VisualizationNode*>& GetVisualizationNodes() { return vNodes; }
	//! Read (Reference) access to:container for visualization nodes
	const ResizableArray<VisualizationNode*>& GetVisualizationNodes() const { return vNodes; }

	//! Write (Reference) access to:container for visualization markers
	ResizableArray<VisualizationMarker*>& GetVisualizationMarkers() { return vMarkers; }
	//! Read (Reference) access to:container for visualization markers
	const ResizableArray<VisualizationMarker*>& GetVisualizationMarkers() const { return vMarkers; }

	//! Write (Reference) access to:container for visualization sensors
	ResizableArray<VisualizationSensor*>& GetVisualizationSensors() { return vSensors; }
	//! Read (Reference) access to:container for visualization sensors
	const ResizableArray<VisualizationSensor*>& GetVisualizationSensors() const { return vSensors; }

	//! Write (Reference) access to:container for visualization loads
	ResizableArray<VisualizationLoad*>& GetVisualizationLoads() { return vLoads; }
	//! Read (Reference) access to:container for visualization loads
	const ResizableArray<VisualizationLoad*>& GetVisualizationLoads() const { return vLoads; }

	//! print function mainly for debug reasons in Python
	virtual void Print(std::ostream& os) const;

	friend std::ostream& operator<<(std::ostream& os, const VisualizationSystemData& object)
	{
		object.Print(os);
		return os;
	}

};

#endif
