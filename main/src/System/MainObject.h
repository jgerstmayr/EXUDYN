/** ***********************************************************************************************
* @class	    MainObject
* @brief		Class for main Objects (all handling, python interfaces, etc. to CObjects)
* @details		Details:
 				- Objects define coordinates of computational objects (CObjects)
                - Objects can be of one category: ODE1coordinates, ODE2coordinates, AEvariables, DataVariables
*
* @author		Gerstmayr Johannes
* @date			2019-04-20 (generated)
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
//#include <pybind11/stl_bind.h>
//#include <pybind11/operators.h>
#include <pybind11/numpy.h>       //interface to numpy
namespace py = pybind11;            //! "py" used throughout in code

//! this is the general CObject interface class, needed to list all Objects
class MainObject
{
protected:
	STDstring name;

public:
	virtual MainObject* GetClone() const { return new MainObject(*this); }

	//! get string of object --> for handling and for python
	virtual const char* GetTypeName() const { return "CObject"; } //externally, shown as computational object!

	virtual STDstring& GetName() { return name; }
	virtual const STDstring& GetName() const { return name; }

	virtual void SetWithDictionary(const py::dict& d) { SysError("Invalid call to MainObject::SetWithDictionary"); }
	virtual py::dict GetDictionary() const { SysError("Invalid call to MainObject::GetDictionary");  return py::dict(); }

	//! Get const pointer to computational base class object
	virtual CObject* GetCObject() const { SysError("Invalid call to MainObject::GetCObject");  return NULL; }
	//! Set pointer to computational base class object (do this only in object factory; type is NOT CHECKED!!!)
	virtual void SetCObject(CObject* pCObject) { SysError("Invalid call to MainObject::SetCObject"); }

	//! Get const pointer to visualization base class item
	virtual VisualizationObject* GetVisualizationObject() const { SysError("Invalid call to MainObject::GetVisualizationObject");  return NULL; }
	//! Set pointer to computational base class object (do this only in object factory; type is NOT CHECKED!!!)
	virtual void SetVisualizationObject(VisualizationObject* pVisualizationObject) { SysError("Invalid call to MainObject::SetVisualizationObject"); }

	//! GetOutputVariable with type and return value; copies values==>slow!; can be scalar or vector-valued! maps to CObject GetOutputVariable(...)
	virtual py::object GetOutputVariable(OutputVariableType variableType) const;
	//! GetOutputVariable with type and return value; copies values==>slow!; can be scalar or vector-valued! maps to CObject GetOutputVariable(...)
	virtual py::object GetOutputVariableConnector(OutputVariableType variableType, const MarkerDataStructure& markerData) const;
	//! GetOutputVariable for a body with type, local position, configuration (reference, current, ...) and return value; copies values==>slow!
	virtual py::object GetOutputVariableBody(OutputVariableType variableType, const Vector3D& localPosition, ConfigurationType configuration) const
	{ SysError("Invalid call to MainObject::GetOutputVariableBody"); return py::object(); }

	//! Get (read) parameter 'parameterName' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
	virtual py::object GetParameter(const STDstring& parameterName) const { SysError("Invalid call to MainObject::GetParameter"); return py::object(); }
	//! Set (write) parameter 'parameterName' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
	virtual void SetParameter(const STDstring& parameterName, const py::object& value) { SysError("Invalid call to MainObject::SetParameter(...)"); }


	//! call pybind object function, possibly with arguments
	virtual py::object CallFunction(STDstring functionName, py::dict args) const { SysError("Invalid call to MainObject::CallFunction"); return py::object();  }

	//! Check consistency prior to CSystem::Assemble(); needs to find all possible violations such that Assemble() would fail; override by according classes
	virtual bool CheckPreAssembleConsistency(const MainSystem& mainSystem, STDstring& errorString) const { return true; }
};

class MainObjectBody : public MainObject
{
protected:
	//CMaterial* material; //put into MainObjectBody if needed
public:
	//! Get const pointer to computational base class object
	virtual CObjectBody* GetCObjectBody() const { return (CObjectBody*)GetCObject(); }

	//! Get local to global transformation for python; SLOW, do not use in computation
	virtual ArrayIndex GetODE2LocalToGlobalCoordinatesArray()
	{
		ArrayIndex ltg;
		GetCObjectBody()->GetODE2LocalToGlobalCoordinates(ltg);
		return ArrayIndex(ltg); //must be copied, because move constructor avoided for ResizableArray
	}

	//! 
	virtual py::object GetOutputVariableBody(OutputVariableType variableType, const Vector3D& localPosition, ConfigurationType configuration) const;
};

class MainObjectConnector : public MainObject
{
protected:
public:
	//! Get const pointer to computational base class object
	virtual CObjectConnector* GetCObjectConnector() const { return (CObjectConnector*)GetCObject(); }

	//! Get reference to marker numbers (const)
	virtual const ArrayIndex& GetMarkerNumbers() const { SysError("Invalid call to MainObjectConnector::GetMarkerNumbers");  ArrayIndex* v = new ArrayIndex(); return *v; }
	//! Set reference to marker numbers
	virtual void SetMarkerNumbers(const ArrayIndex& markerNumbersInit) { SysError("Invalid call to MainObjectConnector::SetMarkerNumbers"); }
	//! 
	//virtual py::object GetOutputVariableBody(OutputVariableType variableType, const Vector3D& localPosition, ConfigurationType configuration) const;
};

