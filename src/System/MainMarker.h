/** ***********************************************************************************************
* @class	    MainMarker
* @brief		Class for main Markers (all handling, python interfaces, etc. to CMarkers)
* @details		Details:
 				- Markers define coordinates of computational objects (CObjects)
                - Markers can be of one category: ODE1coordinates, ODE2coordinates, AEvariables, DataVariables
*
* @author		Gerstmayr Johannes
* @date			2018-05-17 (generated)
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
#ifndef MAINMARKER__H
#define MAINMARKER__H

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

#include <pybind11/pybind11.h>      //! include pybind for dictionary access
#include <pybind11/stl.h>           //! needed for stl-casts; otherwise py::cast with std::vector<Real> crashes!!!

namespace py = pybind11;            //! "py" used throughout in code

//! this is the general CMarker interface class, needed to list all Marker Objects
class MainMarker
{
protected:
	STDstring name;
public:
	virtual ~MainMarker() {} //added for correct deletion of derived classes
	virtual MainMarker* GetClone() const { return new MainMarker(*this); }

	virtual STDstring& GetName() { return name; }
	virtual const STDstring& GetName() const { return name; }

	virtual const char* GetTypeName() const { SysError("Invalid call to MainMarker::GetTypeName"); return "MainMarker::Invalid"; }

	virtual void SetWithDictionary(const py::dict& d) { SysError("Invalid call to MainMarker::SetWithDictionary"); }
	virtual py::dict GetDictionary() const { SysError("Invalid call to MainMarker::GetDictionary");  return py::dict(); }

	//! Get const pointer to computational base class item
	virtual CMarker* GetCMarker() const { SysError("Invalid call to MainMarker::GetCMarker");  return NULL; }
	//! Set pointer to computational base class item (do this only in object factory; type is NOT CHECKED!!!)
	virtual void SetCMarker(CMarker* pCMarker) { SysError("Invalid call to MainMarker::SetCMarker"); }

	//! Get const pointer to visualization base class item
	virtual VisualizationMarker* GetVisualizationMarker() const { SysError("Invalid call to MainMarker::GetVisualizationMarker");  return NULL; }
	//! Set pointer to computational base class item (do this only in object factory; type is NOT CHECKED!!!)
	virtual void SetVisualizationMarker(VisualizationMarker* pVisualizationMarker) { SysError("Invalid call to MainMarker::SetVisualizationMarker"); }

	//! Check consistency prior to CSystem::Assemble(); needs to find all possible violations such that Assemble() would fail; override by according classes
	virtual bool CheckPreAssembleConsistency(const MainSystem& mainSystem, STDstring& errorString) const { return true; }

	//! Get (read) parameter 'parameterName' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
	virtual py::object GetParameter(const STDstring& parameterName) const { SysError("Invalid call to MainMarker::GetParameter"); return py::object(); }
	//! Set (write) parameter 'parameterName' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
	virtual void SetParameter(const STDstring& parameterName, const py::object& value) { SysError("Invalid call to MainMarker::SetParameter(...)"); }

	//! GetOutputVariable with type and return value; restricted to certain number of types
	virtual py::object GetOutputVariable(const CSystemData& cSystemData, OutputVariableType variableType, ConfigurationType configuration) const;
};

#endif
