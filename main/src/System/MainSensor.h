/** ***********************************************************************************************
* @class	    MainSensor
* @brief		Class for main Sensors (all handling, python interfaces, etc. to CSensors)
* @details		Details:
 				- Sensors measure OutputVariables of objects, nodes, markers and loads
*
* @author		Gerstmayr Johannes
* @date			2020-01-24 (generated)
* @pre			...
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

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

#include <pybind11/pybind11.h>      //! include pybind for dictionary access
#include <pybind11/stl.h>           //! needed for stl-casts; otherwise py::cast with std::vector<Real> crashes!!!

namespace py = pybind11;            //! "py" used throughout in code

//! this is the general CSensor interface class, needed to list all Sensor Objects
class MainSensor
{
protected:
	STDstring name;
public:
	virtual MainSensor* GetClone() const { return new MainSensor(*this); }

	virtual STDstring& GetName() { return name; }
	virtual const STDstring& GetName() const { return name; }

	virtual const char* GetTypeName() const { SysError("Invalid call to MainSensor::GetTypeName");  return "MainSensor::Invalid"; }

	virtual void SetWithDictionary(const py::dict& d) { SysError("Invalid call to MainSensor::SetWithDictionary"); }
	virtual py::dict GetDictionary() const { SysError("Invalid call to MainSensor::GetDictionary");  return py::dict(); }

	//! Get const pointer to computational base class item
	virtual CSensor* GetCSensor() const { SysError("Invalid call to MainSensor::GetCSensor");  return NULL; }
	//! Set pointer to computational base class item (do this only in object factory; type is NOT CHECKED!!!)
	virtual void SetCSensor(CSensor* pCSensor) { SysError("Invalid call to MainSensor::SetCSensor"); }

	//! Get const pointer to visualization base class item
	virtual VisualizationSensor* GetVisualizationSensor() const { SysError("Invalid call to MainSensor::GetVisualizationSensor");  return NULL; }
	//! Set pointer to computational base class item (do this only in object factory; type is NOT CHECKED!!!)
	virtual void SetVisualizationSensor(VisualizationSensor* pVisualizationSensor) { SysError("Invalid call to MainSensor::SetVisualizationSensor"); }

	//! Check consistency prior to CSystem::Assemble(); needs to find all possible violations such that Assemble() would fail; override by according classes
	virtual bool CheckPreAssembleConsistency(const MainSystem& mainSystem, STDstring& errorString) const { return true; }

	//! Get (read) parameter 'parameterName' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
	virtual py::object GetParameter(const STDstring& parameterName) const { SysError("Invalid call to MainSensor::GetParameter"); return py::object(); }
	//! Set (write) parameter 'parameterName' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
	virtual void SetParameter(const STDstring& parameterName, const py::object& value) { SysError("Invalid call to MainSensor::SetParameter(...)"); }

};

