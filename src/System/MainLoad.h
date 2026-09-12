/** ***********************************************************************************************
* @class	    MainLoad
* @brief		Class for main Loads (all handling, python interfaces, etc. to CLoads)
* @details		Details:
 				- Loads define coordinates of computational objects (CObjects)
                - Loads can be of one category: ODE1coordinates, ODE2coordinates, AEvariables, DataVariables
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
#ifndef MAINLOAD__H
#define MAINLOAD__H

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

#include <pybind11/pybind11.h>      //! include pybind for dictionary access
#include <pybind11/stl.h>           //! needed for stl-casts; otherwise py::cast with std::vector<Real> crashes!!!

namespace py = pybind11;            //! "py" used throughout in code

//! this is the general CLoad interface class, needed to list all Load Objects
class MainLoad
{
protected:
	STDstring name;
public:
	virtual ~MainLoad() {} //added for correct deletion of derived classes
	virtual MainLoad* GetClone() const { return new MainLoad(*this); }

	virtual STDstring& GetName() { return name; }
	virtual const STDstring& GetName() const { return name; }

	virtual const char* GetTypeName() const { SysError("Invalid call to MainLoad::GetTypeName");  return "MainLoad::Invalid"; }

	virtual void SetWithDictionary(const py::dict& d) { SysError("Invalid call to MainLoad::SetWithDictionary"); }
	virtual py::dict GetDictionary() const { SysError("Invalid call to MainLoad::GetDictionary");  return py::dict(); }

	//! Get const pointer to computational base class item
	virtual CLoad* GetCLoad() const { SysError("Invalid call to MainLoad::GetCLoad");  return NULL; }
	//! Set pointer to computational base class item (do this only in object factory; type is NOT CHECKED!!!)
	virtual void SetCLoad(CLoad* pCLoad) { SysError("Invalid call to MainLoad::SetCLoad"); }

	//! Get const pointer to visualization base class item
	virtual VisualizationLoad* GetVisualizationLoad() const { SysError("Invalid call to MainLoad::GetVisualizationLoad");  return NULL; }
	//! Set pointer to computational base class item (do this only in object factory; type is NOT CHECKED!!!)
	virtual void SetVisualizationLoad(VisualizationLoad* pVisualizationLoad) { SysError("Invalid call to MainLoad::SetVisualizationLoad"); }

	//! Check consistency prior to CSystem::Assemble(); needs to find all possible violations such that Assemble() would fail; override by according classes
	virtual bool CheckPreAssembleConsistency(const MainSystem& mainSystem, STDstring& errorString) const { return true; }

	//! Get current load value(s); copies values==>slow!; can be scalar or vector-valued!
	virtual py::object GetLoadValues(const MainSystemBase&, Real time) const;

	//! Get (read) parameter 'parameterName' via pybind / pyhton interface instead of obtaining the whole dictionary with GetDictionary
	virtual py::object GetParameter(const STDstring& parameterName) const { SysError("Invalid call to MainLoad::GetParameter"); return py::object(); }
	//! Set (write) parameter 'parameterName' to 'value' via pybind / pyhton interface instead of writing the whole dictionary with SetWithDictionary(...)
	virtual void SetParameter(const STDstring& parameterName, const py::object& value) { SysError("Invalid call to MainLoad::SetParameter(...)"); }

};

#endif
