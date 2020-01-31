/** ***********************************************************************************************
* @class        VisualizationSystemData
* @brief		Implementation of VisualizationSystemData
* @details		Details:
*
* @author		Gerstmayr Johannes
* @date			2019-05-27 (generated)
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


#include "Graphics/VisualizationSystemData.h"


void VisualizationSystemData::Reset()
{
	for (auto item : vLoads) { delete item; }
	for (auto item : vMarkers) { delete item; }
	for (auto item : vNodes) { delete item; }
	for (auto item : vObjects) { delete item; }
	for (auto item : vSensors) { delete item; }

	vLoads.Flush();
	vMarkers.Flush();
	vNodes.Flush();
	vObjects.Flush();
	vSensors.Flush();
}

//! print function mainly for debug reasons in Python
void VisualizationSystemData::Print(std::ostream& os) const
{
	os << "CSystemData";
	os << "  cObjects = " << vObjects << "\n";
	os << "  cNodes = " << vNodes << "\n";
	os << "  cMarkers = " << vMarkers << "\n";
	os << "  cLoads = " << vLoads << "\n";
	os << "  cSensors = " << vSensors << "\n";
	os << "\n";
}



