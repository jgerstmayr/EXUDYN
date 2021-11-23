/** ***********************************************************************************************
* @class		PyGeneralContact
* @brief		Pybind11 interface to GeneralContact
*
* @author		Gerstmayr Johannes
* @date			2020-05-11 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */

#ifndef PYGENERALCONTACT__H
#define PYGENERALCONTACT__H

#include "System/CContact.h"	

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>       //interface to numpy
#include <pybind11/buffer_info.h> //passing reference to matrix to numpy
namespace py = pybind11;            //! namespace 'py' used throughout in code
#include "Pymodules/PybindUtilities.h"

#ifdef USE_GENERAL_CONTACT


//! simple dense/sparse matrix container for simplistic operations; GeneralContact can be used as interface for both sparse and dense matrices
class PyGeneralContact: public GeneralContact
{

public:
	//! create empty (dense) container
	PyGeneralContact():GeneralContact() {}

	void PyFinalizeContact(const MainSystem& mainSystem, 
		const py::object& searchTreeSize,
		const py::object& frictionPairingsInit,
		const py::object& searchTreeBoxMin,
		const py::object& searchTreeBoxMax)
	{
		Index3 searchTreeSizeC;
		Matrix frictionPairingsC;
		Vector3D searchTreeBoxMinC;
		Vector3D searchTreeBoxMaxC;

		EPyUtils::SetSlimArraySafely<Index, 3>(searchTreeSize, searchTreeSizeC);
		EPyUtils::SetMatrixSafely(frictionPairingsInit, frictionPairingsC);

		EPyUtils::SetVector3DSafely(searchTreeBoxMin, searchTreeBoxMinC);
		EPyUtils::SetVector3DSafely(searchTreeBoxMax, searchTreeBoxMaxC);

		FinalizeContact(*(mainSystem.GetCSystem()), searchTreeSizeC, frictionPairingsC, 
			searchTreeBoxMinC, searchTreeBoxMaxC);
	}

	//!convert internal data of GeneralContact to 
	py::object GetPythonObject() const
	{

		//this function currently is very slow!
		auto d = py::dict();
		d["intraSpheresContact"] = intraSpheresContact;
		d["globalContactIndexOffsets"] = EPyUtils::ArrayIndex2NumPy(globalContactIndexOffsets);
		d["frictionPairings"] = EPyUtils::Matrix2NumPy(frictionPairings);
		d["maxFrictionMaterialIndex"] = maxFrictionMaterialIndex;

		//basic info on contact objects
		d["numberOfSpheresMarkerBased"] = spheresMarkerBased.NumberOfItems();
		d["numberOfANCFCable2D"] = ancfCable2D.NumberOfItems();
		d["numberOfSpheresMarkerBased"] = spheresMarkerBased.NumberOfItems();

		auto box = py::list();
		box.append(EPyUtils::SlimVector2NumPy<3>(searchTree.GetBox().PMin()));
		box.append(EPyUtils::SlimVector2NumPy<3>(searchTree.GetBox().PMax()));
		d["searchTreeBox"] = box;
		auto sizeList = py::list();
		sizeList.append(searchTree.SizeX());
		sizeList.append(searchTree.SizeY());
		sizeList.append(searchTree.SizeZ());
		d["searchTreeSize"] = sizeList;

		Index numberOfTreeItems;
		Real averageFill;
		Index numberOfZeros;
		Index maxFill;
		Index numberOf10average;

		searchTree.GetStatistics(numberOfTreeItems, averageFill, numberOfZeros, maxFill, numberOf10average);
		auto dSearchTree = py::dict();
		dSearchTree["numberOfTreeItems"] = numberOfTreeItems;
		dSearchTree["averageFill"] = averageFill;
		dSearchTree["numberOfZeros"] = numberOfZeros;
		dSearchTree["maxFill"] = maxFill;
		dSearchTree["numberOf10average"] = numberOf10average;

		d["searchTreeStatistics"] = dSearchTree;

		return d;
	}

	void Print(std::ostream& os) const {
		os << "GeneralContact:";
		//py::object pyObject = ;
		os << py::str(GetPythonObject());
	}

	friend std::ostream& operator<<(std::ostream& os, const PyGeneralContact& object) {
		object.Print(os);
		return os;
	}

};
#endif //USE_GENERAL_CONTACT


#endif
