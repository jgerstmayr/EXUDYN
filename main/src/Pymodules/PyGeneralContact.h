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
	PyGeneralContact(): GeneralContact() {}

	//! add contact object for Triangles attached to rigidBodyMarker
	//! contact is possible between sphere (circle) and Triangle but yet not between triangle and triangle!
	void PyAddTrianglesRigidBodyBased(Index rigidBodyMarkerIndexInit, Real contactStiffnessInit, Real contactDampingInit,
		Index frictionMaterialIndexInit, const py::object& pointListInit, const py::object& triangleListInit)
	{
		ResizableArray<Vector3D> pointList;
		ResizableArray<Index3> triangleList;

		EPyUtils::SetListOfArraysSafely<Vector3D, Real>(pointListInit, pointList);
		EPyUtils::SetListOfArraysSafely<Index3, Index>(triangleListInit, triangleList);

		AddTrianglesRigidBodyBased(rigidBodyMarkerIndexInit, contactStiffnessInit, contactDampingInit, frictionMaterialIndexInit, pointList, triangleList);
	}

	bool GetSphereSphereContact() const { return settings.sphereSphereContact; }
	void SetSphereSphereContact(bool flag) { settings.sphereSphereContact = flag; }

	bool GetSphereSphereFrictionRecycle() const { return settings.sphereSphereFrictionRecycle; }
	void SetSphereSphereFrictionRecycle(bool flag) { settings.sphereSphereFrictionRecycle = flag; }
	
	Real GetMinRelDistanceSpheresTriangles() const { return settings.minRelDistanceSpheresTriangles; }
	void SetMinRelDistanceSpheresTriangles(Real value) { settings.minRelDistanceSpheresTriangles = value; }

	Real GetFrictionProportionalZone() const { return settings.frictionProportionalZone; }
	void SetFrictionProportionalZone(Real value) { settings.frictionProportionalZone = value; }

	Real GetFrictionVelocityPenalty() const { return settings.frictionVelocityPenalty; }
	void SetFrictionVelocityPenalty(Real value) { settings.frictionVelocityPenalty = value; }

	//!< for consistent, closed meshes, we can exclude overlapping contact triangles (which would cause holes if mesh is overlapping and not consistent!!!)
	bool GetExcludeOverlappingTrigSphereContacts() const { return settings.excludeOverlappingTrigSphereContacts; }
	void SetExcludeOverlappingTrigSphereContacts(bool value) { settings.excludeOverlappingTrigSphereContacts = value; }
	
	//!< run additional checks for double contacts at edges or vertices, being more accurate but can cause additional costs if many contacts
	bool GetExcludeDuplicatedTrigSphereContactPoints() const { return settings.excludeDuplicatedTrigSphereContactPoints; }
	void SetExcludeDuplicatedTrigSphereContactPoints(bool value) { settings.excludeDuplicatedTrigSphereContactPoints = value; }

	//! if true, uses exact computation of intersection of 3rd order polynomials and contacting circles
	bool GetAncfCableUseExactMethod() const { return settings.ancfCableUseExactMethod; }
	void SetAncfCableUseExactMethod(bool value) { settings.ancfCableUseExactMethod = value; }

	//! if ancfCableUseExactMethod=false, then this specifies the number of contact segments for ANCF element
	Index GetAncfCableNumberOfContactSegments() const { return settings.ancfCableNumberOfContactSegments; }
	void SetAncfCableNumberOfContactSegments(Index value) { settings.ancfCableNumberOfContactSegments = value; }

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//interface functions for Python / PyGeneralContact
	//! set Coulomb friction coefficients for pairings of materials (e.g., use material 0,1, then the entries (0,1) and (1,0) define the friction coefficients for this pairing)
	//  matrix should be symmetric!
	void SetFrictionPairings(const py::object& frictionPairingsInit)
	{
		Matrix frictionPairingsC;
		EPyUtils::SetMatrixSafely(frictionPairingsInit, frictionPairingsC);
		if (frictionPairingsC.NumberOfRows() != frictionPairingsC.NumberOfColumns())
		{
			PyError("SetFrictionPairings: frictionPairings Matrix must be square (equal number of rows and columns)!");
		}
		if (!(frictionPairingsC.GetTransposed() == frictionPairingsC))
		{
			PyWarning("SetFrictionPairings: frictionPairings Matrix should be symmetric for Physics reasons!");
		}
		settings.frictionPairings = frictionPairingsC;
	}

	//! set number of cells of search tree (boxed search) in x, y and z direction
	void SetSearchTreeCellSize(const py::object& numberOfCells)
	{
		Index3 searchTreeSizeC;
		EPyUtils::SetSlimArraySafely<Index, 3>(numberOfCells, searchTreeSizeC);
		settings.searchTreeSizeInit = searchTreeSizeC;

		if (verboseMode >= 2)
		{
			pout << "Set search tree cells = " << settings.searchTreeSizeInit << "\n";
			pout << "  initial searchTreeBox=[ " << settings.searchTreeBoxMinInit << ", "
				<< settings.searchTreeBoxMaxInit << " ]\n";
		}
	}
	//void SetSearchTreeInitSize(Index searchTreeSizeX, Index searchTreeSizeY, Index searchTreeSizeZ)
	//{
	//	settings.searchTreeSizeInit = Index3({ searchTreeSizeX, searchTreeSizeY, searchTreeSizeZ });
	//}

	//! set geometric dimensions of searchTreeBox; if this box becomes smaller than the effective contact objects, contact computations may slow down significantly
	void SetSearchTreeBox(const py::object& pMin, const py::object& pMax)
	{
		Vector3D searchTreeBoxMinC;
		Vector3D searchTreeBoxMaxC;
		EPyUtils::SetVector3DSafely(pMin, searchTreeBoxMinC);
		EPyUtils::SetVector3DSafely(pMax, searchTreeBoxMaxC);
		settings.searchTreeBoxMinInit = searchTreeBoxMinC;
		settings.searchTreeBoxMaxInit = searchTreeBoxMaxC;
	}


	//!convert internal data of GeneralContact to 
	py::object GetPythonObject() const
	{

		//this function currently is very slow!
		auto d = py::dict();
		d["sphereSphereContact"] = settings.sphereSphereContact;
		d["sphereSphereFrictionRecycle"] = settings.sphereSphereFrictionRecycle;
		d["globalContactIndexOffsets"] = EPyUtils::ArrayIndex2NumPy(globalContactIndexOffsets);
		d["frictionPairings"] = EPyUtils::Matrix2NumPy(settings.frictionPairings);
		d["frictionProportionalZone "] = settings.frictionProportionalZone;

		//basic info on contact objects
		d["numberOfSpheresMarkerBased"] = spheresMarkerBased.NumberOfItems();
		d["numberOfANCFCable2D"] = ancfCable2D.NumberOfItems();
		d["numberOfTrigsRigidBodyBased"] = trigsRigidBodyBased.NumberOfItems();
		d["numberOfRigidBodyMarkerBased"] = rigidBodyMarkerBased.NumberOfItems();

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
