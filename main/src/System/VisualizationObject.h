/** ***********************************************************************************************
* @class        VisualizationObject
* @brief		
* @details		Details:
* 				- a base class for visualization of a object
*				- all derived classes need to implement drawing functions to work with VisualizationSystem::UpdateGraphicsData
*
* @author		Gerstmayr Johannes
* @date			2020-01-24 (generated)
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
#include "Linalg/BasicLinalg.h"

class VisualizationSystem;		//predefined to be used in definition of base visualizationItem classes
class VisualizationSettings;	//predefined for update graphics
class MainSystem;				//for user functions

//! base class for visualization of object
class VisualizationObject
{
protected:
	bool show; //true: shall be drawn; false: do not draw; will be initialized in specialized class
public:
	//! compute graphics update by adding graphics items to graphicsData in VisualizationSystem
	virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) {};
	
	//! overide this class to realize a user function for object
	virtual void CallUserFunction(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, const MainSystem& mainSystem, Index itemNumber) {};

	//! this function needs to be overwritten, if the object has a graphics user function
	virtual bool HasUserFunction() { return false; };

	//! decides whether to draw the item
	virtual bool GetShow() const { return show; }
	virtual void SetShow(bool value) { show = value; }
	virtual bool& GetShow() { return show; }

	virtual bool IsConnector() const { return false; } //returns true, if this visualizationObject is a connector

	//! print function, used mainly for debug reasons and to be shown in Python
	virtual void Print(std::ostream& os) const {
		os << "VisualizationObject(";
		os << ")";
	}

};

//! base class for visualization of object
class VisualizationObjectSuperElement: public VisualizationObject
{
protected:
	bool show; //true: shall be drawn; false: no not draw; will be initialized in specialized class
public:
	//! compute graphics update by adding graphics items to graphicsData in VisualizationSystem
	//implementation in VisuNodePoint
	virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber);

	virtual const bool& GetShowNodes() const = 0;
	//{ 
	//	CHECKandTHROWstring("ERROR: illegal call to VisualizationObjectSuperElement::GetShowNodes"); 
	//	return false; 
	//};
	virtual const MatrixI& GetTriangleMesh() const = 0;
	//{ 
	//	CHECKandTHROWstring("ERROR: illegal call to VisualizationObjectSuperElement::GetAccessFunctionBody"); 
	//	return EXUmath::unitMatrixI; 
	//};
	virtual const Float4& GetColor() const = 0;
	//{ 
	//	CHECKandTHROWstring("ERROR: illegal call to VisualizationObjectSuperElement::GetColor"); 
	//	return Float4({0.f,0.f,0.f,1.f}); 
	//};

	//! print function, used mainly for debug reasons and to be shown in Python
	virtual void Print(std::ostream& os) const {
		os << "VisualizationSuperElement(";
		os << ")";
	}

};

//! the ostream operator<< is only defined in base class and calls the Print(...) method of the derived class; this calls the correct method also in the ResizableArray<CObject*>
inline std::ostream& operator<<(std::ostream& os, const VisualizationObject& item) {
	item.Print(os);
	return os;
}

