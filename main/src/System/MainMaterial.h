/** ***********************************************************************************************
* @class	    MainMaterial
* @brief		Class for main Materials (all handling, python interfaces, etc. to CMaterials)
* @details		Details:
 				- Materials define coordinates of computational objects (CObjects)
                - Materials can be of one category: ODE1coordinates, ODE2coordinates, AEvariables, DataVariables
*
* @author		Gerstmayr Johannes
* @date			2018-05-17 (generated)
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
#ifndef MAINMATERIAL__H
#define MAINMATERIAL__H

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

#include <pybind11/pybind11.h>      //! include pybind for dictionary access
#include <pybind11/stl.h>           //! needed for stl-casts; otherwise py::cast with std::vector<Real> crashes!!!
namespace py = pybind11;            //! "py" used throughout in code

//! this is the general CMaterial interface class, needed to list all Material Objects
class MainMaterial
{
protected:
	STDstring name;
public:
	virtual ~MainMaterial() {} //added for correct deletion of derived classes
	virtual MainMaterial* GetClone() const { return new MainMaterial(*this); }

	virtual STDstring& GetName() { return name; }
	virtual const STDstring& GetName() const { return name; }

	virtual void SetWithDictionary(const py::dict& d) { SysError("Invalid call to MainMaterial::SetWithDictionary"); }
	virtual py::dict GetDictionary() const { SysError("Invalid call to MainMaterial::GetDictionary");  return py::dict(); }

	//! Get const pointer to computational base class object
	virtual CMaterial* GetCMaterial() const { SysError("Invalid call to MainMaterial::GetCMaterial");  return NULL; }
	//! Set pointer to computational base class object (do this only in object factory; type is NOT CHECKED!!!)
	virtual void SetCMaterial(CMaterial* pCMaterial) { SysError("Invalid call to MainMaterial::SetCMaterial"); }
};

#endif
