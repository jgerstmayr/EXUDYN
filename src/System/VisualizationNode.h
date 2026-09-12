/** ***********************************************************************************************
* @class        VisualizationNode
* @brief		
* @details		Details:
* 				- a base class for visualization of a node
*				- all derived classes need to implement drawing functions to work with VisualizationSystem::UpdateGraphicsData
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
#ifndef VISUALIZATIONNODE__H
#define VISUALIZATIONNODE__H

class VisualizationSystem; //predefined to be used in definition of base visualizationItem classes
class VisualizationSettings; //predefined for update graphics

//! base class for visualization of node
class VisualizationNode
{
protected:
	bool show; //true: shall be drawn; false: no not draw; will be initialized in specialized class
public:
	virtual ~VisualizationNode() {} //added for correct deletion of derived classes
	//! Compute graphics update by adding graphics items to graphicsData in VisualizationSystem
	//  itemNumber: number according to itemNumber in CData
	//  future: remove direct links to VisualizationSystem and CSystemData
	virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) {};
	//! decides whether to draw the item
	virtual bool GetShow() const { return show; }
	virtual void SetShow(bool value) { show = value; }
	virtual bool& GetShow() { return show; }

	//! print function, used mainly for debug reasons and to be shown in Python
	virtual void Print(std::ostream& os) const {
		os << "CObject(";
		os << ")";
	}

};

//! the ostream operator<< is only defined in base class and calls the Print(...) method of the derived class; this calls the correct method also in the ResizableArray<CObject*>
inline std::ostream& operator<<(std::ostream& os, const VisualizationNode& item) {
	item.Print(os);
	return os;
}

#endif
