/** ***********************************************************************************************
* @class        MainSystemData
* @brief        All data defining the structure of the system for python integration; this is the place where Python objects really is live (link to this data if necessary)
*
* @author       Gerstmayr Johannes
* @date         2018-05-18 (generated)
* @date         2019-04-25 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
				- email: johannes.gerstmayr@uibk.ac.at
				- weblink: https://github.com/jgerstmayr/EXUDYN
				
************************************************************************************************ */
#ifndef MAINSYSTEMDATA__H
#define MAINSYSTEMDATA__H

#include <ostream>

#include "Main/CSystem.h"
#include "Graphics/VisualizationSystemData.h"

#include "System/MainMaterial.h"
#include "System/MainMarker.h"
#include "System/MainLoad.h"
#include "System/MainNode.h"
#include "System/MainObject.h"
#include "System/MainSensor.h"

#include "Main/OutputVariable.h"

class MainSystemData //
{
protected: //
	CSystemData* cSystemData;

	ResizableArray<MainLoad*> mainLoads;            //!< container for main loads
	ResizableArray<MainMarker*> mainMarkers;        //!< container for main markers
	ResizableArray<MainMaterial*> mainMaterials;    //!< container for main materials
	ResizableArray<MainNode*> mainNodes;            //!< container for main nodes
	ResizableArray<MainObject*> mainObjects;        //!< container for main objects
	ResizableArray<MainSensor*> mainSensors;        //!< container for main sensors

public: //

	//! Write (Reference) access to: cSystemData
	CSystemData& GetCSystemData() { return *cSystemData; }
	//! Read (Reference) access to: cSystemData
	const CSystemData& GetCSystemData() const { return *cSystemData; }
	//! Set access to: cSystemData
	void SetCSystemData(CSystemData* ptr) { cSystemData = ptr; }

	// access functions
	//! clone object; specifically for copying instances of derived class, for automatic memory management e.g. in ObjectContainer
	MainSystemData* GetClone() const { return new MainSystemData(*this); }

	//! reset all lists and deallocate data
	void Reset()
	{
		for (auto item : mainLoads) { delete item; }
		for (auto item : mainMarkers) { delete item; }
		for (auto item : mainMaterials) { delete item; }
		for (auto item : mainNodes) { delete item; }
		for (auto item : mainObjects) { delete item; }
		for (auto item : mainSensors) { delete item; }

		mainLoads.Flush();
		mainMarkers.Flush();
		mainMaterials.Flush();
		mainNodes.Flush();
		mainObjects.Flush();
		mainSensors.Flush();
	}

	//! Write (Reference) access to:container for main loads
	ResizableArray<MainLoad*>& GetMainLoads() { return mainLoads; }
	//! Read (Reference) access to:container for main loads
	const ResizableArray<MainLoad*>& GetMainLoads() const { return mainLoads; }

	//! Write (Reference) access to:container for main markers
	ResizableArray<MainMarker*>& GetMainMarkers() { return mainMarkers; }
	//! Read (Reference) access to:container for main markers
	const ResizableArray<MainMarker*>& GetMainMarkers() const { return mainMarkers; }

	//! Write (Reference) access to:container for main materials
	ResizableArray<MainMaterial*>& GetMainMaterials() { return mainMaterials; }
	//! Read (Reference) access to:container for main materials
	const ResizableArray<MainMaterial*>& GetMainMaterials() const { return mainMaterials; }

	//! Write (Reference) access to:container for main nodes
	ResizableArray<MainNode*>& GetMainNodes() { return mainNodes; }
	//! Read (Reference) access to:container for main nodes
	const ResizableArray<MainNode*>& GetMainNodes() const { return mainNodes; }
	//! Read CONST Reference access to single node ==> works in cases, where a const object is needed!
	const MainNode& GetMainNode(Index i) const { return *mainNodes[i]; }

	//! Write (Reference) access to:container for main objects
	ResizableArray<MainObject*>& GetMainObjects() { return mainObjects; }
	//! Read (Reference) access to:container for main objects
	const ResizableArray<MainObject*>& GetMainObjects() const { return mainObjects; }

	//! Write (Reference) access to:container for main sensors
	ResizableArray<MainSensor*>& GetMainSensors() { return mainSensors; }
	//! Read (Reference) access to:container for main sensors
	const ResizableArray<MainSensor*>& GetMainSensors() const { return mainSensors; }

	//py::object GetVector()
	//{
	//	Vector v({ 42.1234567890123456,43,44 }); //double precision maintained in NumPy array in python
	//	return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer()); //copy array (could also be referenced!)
	//}

	const CSystemState* GetCSystemState(ConfigurationType configurationType) const
	{
		if (configurationType == ConfigurationType::Reference) { return &(cSystemData->GetCData().GetReference()); }
		else if (configurationType == ConfigurationType::Initial) { return &(cSystemData->GetCData().GetInitial()); }
		else if (configurationType == ConfigurationType::Current) { return &(cSystemData->GetCData().GetCurrent()); }
		else if (configurationType == ConfigurationType::StartOfStep) { return &(cSystemData->GetCData().GetStartOfStep()); }
		else if (configurationType == ConfigurationType::Visualization) { return &(cSystemData->GetCData().GetVisualization()); }

		CHECKandTHROWstring("ERROR: no valid configurationType in MainSystemData::GetCSystemState (const)");
		return &(cSystemData->GetCData().GetInitial());
	}

	CSystemState* GetCSystemState(ConfigurationType configurationType)
	{
		if (configurationType == ConfigurationType::Reference) { return &(cSystemData->GetCData().GetReference()); }
		else if (configurationType == ConfigurationType::Initial) { return &(cSystemData->GetCData().GetInitial()); }
		else if (configurationType == ConfigurationType::Current) { return &(cSystemData->GetCData().GetCurrent()); }
		else if (configurationType == ConfigurationType::StartOfStep) { return &(cSystemData->GetCData().GetStartOfStep()); }
		else if (configurationType == ConfigurationType::Visualization) { return &(cSystemData->GetCData().GetVisualization()); }

		CHECKandTHROWstring("ERROR: no valid configurationType in MainSystemData::GetCSystemState");
		return &(cSystemData->GetCData().GetInitial());
	}

	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to ODE2Size
	Index PyODE2Size(ConfigurationType configurationType = ConfigurationType::Current) const
	{
		return GetCSystemState(configurationType)->GetODE2Coords().NumberOfItems();
	}

	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to ODE1Size
	Index PyODE1Size(ConfigurationType configurationType = ConfigurationType::Current) const
	{
		return GetCSystemState(configurationType)->GetODE1Coords().NumberOfItems();
	}

	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to AESize
	Index PyAEsize(ConfigurationType configurationType = ConfigurationType::Current) const
	{
		return GetCSystemState(configurationType)->GetAECoords().NumberOfItems();
	}

	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to DataSize
	Index PyDataSize(ConfigurationType configurationType = ConfigurationType::Current) const
	{
		return GetCSystemState(configurationType)->GetDataCoords().NumberOfItems();
	}

	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to SystemSize (ODE2+ODE1+AE)
	Index PySystemSize(ConfigurationType configurationType = ConfigurationType::Current) const
	{
		return GetCSystemState(configurationType)->GetODE2Coords().NumberOfItems() +
			GetCSystemState(configurationType)->GetODE1Coords().NumberOfItems() +
			GetCSystemState(configurationType)->GetAECoords().NumberOfItems();
	}

	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to state vectors (but not the non-state derivatives ODE1_t and ODE2_tt and the time); function is copying data - not highly efficient!
	//! format of pyList: [ODE2Coords, ODE2Coords_t, ODE1Coords, AEcoords, dataCoords]
	py::list PyGetSystemState(ConfigurationType configurationType = ConfigurationType::Current) const
	{
		py::list pyList;

		const Vector& vODE2 = GetCSystemState(configurationType)->GetODE2Coords();
		pyList.append(py::array_t<Real>(vODE2.NumberOfItems(), vODE2.GetDataPointer()));

		const Vector& vODE2_t = GetCSystemState(configurationType)->GetODE2Coords_t();
		pyList.append(py::array_t<Real>(vODE2_t.NumberOfItems(), vODE2_t.GetDataPointer()));
		
		const Vector& vODE1 = GetCSystemState(configurationType)->GetODE1Coords();
		pyList.append(py::array_t<Real>(vODE1.NumberOfItems(), vODE1.GetDataPointer()));

		const Vector& vAE = GetCSystemState(configurationType)->GetAECoords();
		pyList.append(py::array_t<Real>(vAE.NumberOfItems(), vAE.GetDataPointer()));

		const Vector& vData = GetCSystemState(configurationType)->GetDataCoords();
		pyList.append(py::array_t<Real>(vData.NumberOfItems(), vData.GetDataPointer()));

		return pyList;
	}

	//! pybind write access to state vectors (but not the non-state derivatives ODE1_t and ODE2_tt and the time); function is copying data - not highly efficient!
	//! format of pyList: [ODE2Coords, ODE2Coords_t, ODE1Coords, AEcoords]
	//! no type checks are performed for now!
	void PySetSystemState(const py::list systemStateList, ConfigurationType configurationType = ConfigurationType::Current)
	{
		py::object pyObjectODE2 = systemStateList[0];
		const std::vector<Real>& vODE2 = py::cast<std::vector<Real>>(pyObjectODE2);
		SetODE2Coords(vODE2, configurationType); //includes safety check
		//GetCSystemState(configurationType)->SetODE2Coords(vODE2);

		py::object pyObjectODE2_t = systemStateList[1];
		const std::vector<Real>& vODE2_t = py::cast<std::vector<Real>>(pyObjectODE2_t);
		SetODE2Coords_t(vODE2_t, configurationType); //includes safety check
		//GetCSystemState(configurationType)->SetODE2Coords_t(vODE2_t);

		py::object pyObjectODE1 = systemStateList[2];
		const std::vector<Real>& vODE1 = py::cast<std::vector<Real>>(pyObjectODE1);
		SetODE1Coords(vODE1, configurationType); //includes safety check
		//GetCSystemState(configurationType)->SetODE1Coords(vODE1);

		py::object pyObjectAE = systemStateList[3];
		const std::vector<Real>& vAE = py::cast<std::vector<Real>>(pyObjectAE);
		SetAECoords(vAE, configurationType); //includes safety check
		//GetCSystemState(configurationType)->SetAECoords(vAE);

		py::object pyObjectData = systemStateList[4];
		const std::vector<Real>& vData = py::cast<std::vector<Real>>(pyObjectData);
		SetDataCoords(vData, configurationType); //includes safety check


	}

	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to ODE2 coords
	py::array_t<Real> GetODE2Coords(ConfigurationType configurationType = ConfigurationType::Current) const
	{
		const Vector& v = GetCSystemState(configurationType)->GetODE2Coords();
		return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
	}

	//! pybind write access to ODE2 coords
	void SetODE2Coords(const std::vector<Real>& v, ConfigurationType configurationType = ConfigurationType::Current)
	{
		CHECKandTHROW((Index)v.size() == GetCSystemState(configurationType)->GetODE2Coords().NumberOfItems(),"SystemData::SetODE2Coords: incompatible size of vectors");
		GetCSystemState(configurationType)->SetODE2Coords(v);
	}

	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to ODE2_t coords
	py::array_t<Real> GetODE2Coords_t(ConfigurationType configurationType = ConfigurationType::Current) const
	{
		const Vector& v = GetCSystemState(configurationType)->GetODE2Coords_t();
		return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
	}

	//! pybind write access to ODE2_t coords
	void SetODE2Coords_t(const std::vector<Real>& v, ConfigurationType configurationType = ConfigurationType::Current)
	{
		CHECKandTHROW((Index)v.size() == GetCSystemState(configurationType)->GetODE2Coords_t().NumberOfItems(), "SystemData::SetODE2Coords_t: incompatible size of vectors");
		GetCSystemState(configurationType)->SetODE2Coords_t(v);
	}

	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to ODE2_t coords
	py::array_t<Real> GetODE2Coords_tt(ConfigurationType configurationType = ConfigurationType::Current) const
	{
		const Vector& v = GetCSystemState(configurationType)->GetODE2Coords_tt();
		return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
	}

	//! pybind write access to ODE2_t coords
	void SetODE2Coords_tt(const std::vector<Real>& v, ConfigurationType configurationType = ConfigurationType::Current)
	{
		CHECKandTHROW((Index)v.size() == GetCSystemState(configurationType)->GetODE2Coords_tt().NumberOfItems(), "SystemData::SetODE2Coords_tt: incompatible size of vectors");
		GetCSystemState(configurationType)->SetODE2Coords_tt(v);
	}

	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to ODE1 coords
	py::array_t<Real> GetODE1Coords(ConfigurationType configurationType = ConfigurationType::Current) const
	{
		const Vector& v = GetCSystemState(configurationType)->GetODE1Coords();
		return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
	}

	//! pybind write access to ODE1 coords
	void SetODE1Coords(const std::vector<Real>& v, ConfigurationType configurationType = ConfigurationType::Current)
	{
		CHECKandTHROW((Index)v.size() == GetCSystemState(configurationType)->GetODE1Coords().NumberOfItems(), "SystemData::SetODE1Coords: incompatible size of vectors");
		GetCSystemState(configurationType)->SetODE1Coords(v);
	}

	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to ODE2 coords
	py::array_t<Real> GetAECoords(ConfigurationType configurationType = ConfigurationType::Current) const
	{
		const Vector& v = GetCSystemState(configurationType)->GetAECoords();
		return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
	}

	//! pybind write access to AE coords
	void SetAECoords(const std::vector<Real>& v, ConfigurationType configurationType = ConfigurationType::Current)
	{
		CHECKandTHROW((Index)v.size() == GetCSystemState(configurationType)->GetAECoords().NumberOfItems(), "SystemData::SetAECoords: incompatible size of vectors");
		GetCSystemState(configurationType)->SetAECoords(v);
	}
	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to Data coords
	py::array_t<Real> GetDataCoords(ConfigurationType configurationType = ConfigurationType::Current) const
	{
		const Vector& v = GetCSystemState(configurationType)->GetDataCoords();
		return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
	}

	//! pybind write access to AE coords
	void SetDataCoords(const std::vector<Real>& v, ConfigurationType configurationType = ConfigurationType::Current)
	{
		CHECKandTHROW((Index)v.size() == GetCSystemState(configurationType)->GetDataCoords().NumberOfItems(), "SystemData::SetDataCoords: incompatible size of vectors");
		GetCSystemState(configurationType)->SetDataCoords(v);
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//local to global ODE2 transformation lists; returned as python list
	std::vector<Index> PyGetObjectLocalToGlobalODE2(Index objectNumber)
	{
		if (objectNumber >= cSystemData->GetLocalToGlobalODE2().NumberOfItems())
		{
			PyError("GetObjectLTGODE2: illegal index");
			return std::vector<Index>();
		}
		return cSystemData->GetLocalToGlobalODE2()[objectNumber];
	}
	//local to global ODE1 transformation lists; returned as python list
	std::vector<Index> PyGetObjectLocalToGlobalODE1(Index objectNumber)
	{
		if (objectNumber >= cSystemData->GetLocalToGlobalODE1().NumberOfItems())
		{
			PyError("GetObjectLTGODE1: illegal index");
			return std::vector<Index>();
		}
		return cSystemData->GetLocalToGlobalODE1()[objectNumber];
	}
	//local to global AE transformation lists; returned as python list
	std::vector<Index> PyGetObjectLocalToGlobalAE(Index objectNumber)
	{
		if (objectNumber >= cSystemData->GetLocalToGlobalAE().NumberOfItems())
		{
			PyError("GetObjectLTGAE: illegal index");
			return std::vector<Index>();
		}
		return cSystemData->GetLocalToGlobalAE()[objectNumber];
	}
	//local to global Data transformation lists; returned as python list
	std::vector<Index> PyGetObjectLocalToGlobalData(Index objectNumber)
	{
		if (objectNumber >= cSystemData->GetLocalToGlobalData().NumberOfItems())
		{
			PyError("GetObjectLTGData: illegal index");
			return std::vector<Index>();
		}
		return cSystemData->GetLocalToGlobalData()[objectNumber];
	}


	//! get current (computation) time
	Real PyGetCurrentTime()
	{
		PyWarning("mbs.systemData.GetCurrentTime() is DEPRECATED! use mbs.systemData.GetTime(exu.ConfigurationType.Current) instead");
		return cSystemData->GetCData().GetCurrent().GetTime();
	}

	//! set visualization time (for postprocessing, animations, etc.)
	void PySetVisualizationTime(Real vTime)
	{
		PyWarning("mbs.systemData.SetVisualizationTime() is DEPRECATED! use mbs.systemData.SetTime(exu.ConfigurationType.Visualization) instead");
		cSystemData->GetCData().GetVisualization().SetTime(vTime);
	}

	//! get current (computation) time
	Real PyGetStateTime(ConfigurationType configurationType = ConfigurationType::Current)
	{
		return GetCSystemState(configurationType)->time;
	}

	//! set visualization time (for postprocessing, animations, etc.)
	void PySetStateTime(Real vTime, ConfigurationType configurationType = ConfigurationType::Current)
	{
		GetCSystemState(configurationType)->time = vTime;
	}

	//NEEDED? can be done with CallFunction ...
	////! Access to global coordinate index (in ODE2 system vector)
	//Index PyNodeGetGlobalODE2CoordinateIndex(Index nodeNumber) const
	//{
	//	if (nodeNumber >= cSystemData->GetCNodes().NumberOfItems())
	//	{
	//		PyError("NodeGetGlobalODE2CoordinateIndex: illegal index");
	//		return EXUstd::InvalidIndex;
	//	}
	//	if (cSystemData->GetCNodes()[nodeNumber]->GetNodeGroup() != CNodeGroup::ODE2variables)
	//	{
	//		PyError("NodeGetGlobalODE2CoordinateIndex: only possible for nodes with ODE2 variables");
	//		return EXUstd::InvalidIndex;
	//	}
	//	return cSystemData->GetCNodes()[nodeNumber]->GetGlobalODE2CoordinateIndex();
	//}

	////! Access to global coordinate index (in AE system vector)
	//Index PyNodeGetGlobalAECoordinateIndex(Index nodeNumber) const
	//{
	//	if (nodeNumber >= cSystemData->GetCNodes().NumberOfItems())
	//	{
	//		PyError("NodeGetGlobalAECoordinateIndex: illegal index");
	//		return EXUstd::InvalidIndex;
	//	}
	//	if (cSystemData->GetCNodes()[nodeNumber]->GetNodeGroup() != CNodeGroup::AEvariables)
	//	{
	//		PyError("NodeGetGlobalAECoordinateIndex: only possible for nodes with AE variables");
	//		return EXUstd::InvalidIndex;
	//	}
	//	return cSystemData->GetCNodes()[nodeNumber]->GetGlobalAECoordinateIndex();
	//}

	////! Access to global coordinate index (in Data system vector)
	//Index PyNodeGetGlobalDataCoordinateIndex(Index nodeNumber) const
	//{
	//	if (nodeNumber >= cSystemData->GetCNodes().NumberOfItems())
	//	{
	//		PyError("NodeGetGlobalDataCoordinateIndex: illegal index");
	//		return EXUstd::InvalidIndex;
	//	}
	//	if (cSystemData->GetCNodes()[nodeNumber]->GetNodeGroup() != CNodeGroup::DataVariables)
	//	{
	//		PyError("NodeGetGlobalDataCoordinateIndex: only possible for nodes with data variables");
	//		return EXUstd::InvalidIndex;
	//	}
	//	return cSystemData->GetCNodes()[nodeNumber]->GetGlobalDataCoordinateIndex();
	//}


	//! will be removed when full python integration is ready
	void Print(std::ostream& os) const
	{
		os << "MainSystemData";
		os << "  mainObjects = " << mainObjects << "\n";
		os << "  mainNodes = " << mainNodes << "\n";
		//os << "  mainMaterials = " << mainMaterials << "\n";
		os << "  mainMarkers = " << mainMarkers << "\n";
		os << "  mainLoads = " << mainLoads << "\n\n";
		os << "  mainSensors = " << mainSensors << "\n\n";
		os << "  LTGODE2 = " << cSystemData->GetLocalToGlobalODE2() << "\n";
		os << "  LTGODE1 = " << cSystemData->GetLocalToGlobalODE1() << "\n";
		os << "  LTGAE = " << cSystemData->GetLocalToGlobalAE() << "\n";
		os << "\n";
	}

	friend std::ostream& operator<<(std::ostream& os, const MainSystemData& object)
	{
		object.Print(os);
		return os;
	}

	//! print detailed dict information on all objects, nodes, markers, ...
	STDstring PyInfoDetailed() const
	{
		STDstring info;
		Index cnt = 0;
		for (auto item : mainNodes) {
			info += "node" + EXUstd::ToString(cnt++) + ":\n    " + std::string(py::str(item->GetDictionary())) + "\n";
		}
		cnt = 0;
		for (auto item : mainObjects) {
			info += "object" + EXUstd::ToString(cnt++) + ":\n    " + std::string(py::str(item->GetDictionary())) + "\n";
		}
		//cnt = 0;
		//for (auto item : mainMaterials) {
		//	info += "material" + EXUstd::ToString(cnt++) + ":\n    " + std::string(py::str(item->GetDictionary())) + "\n";
		//}
		cnt = 0;
		for (auto item : mainMarkers) {
			info += "marker" + EXUstd::ToString(cnt++) + ":\n    " + std::string(py::str(item->GetDictionary())) + "\n";
		}
		cnt = 0;
		for (auto item : mainLoads) {
			info += "load" + EXUstd::ToString(cnt++) + ":\n    " + std::string(py::str(item->GetDictionary())) + "\n";
		}
		cnt = 0;
		for (auto item : mainSensors) {
			info += "sensor" + EXUstd::ToString(cnt++) + ":\n    " + std::string(py::str(item->GetDictionary())) + "\n";
		}

		return info;
	}

	//! print information summary on system
	STDstring PyInfoSummary() const
	{
		STDstring info;
		info += "  Number of nodes= "		+ EXUstd::Num2String(mainNodes.NumberOfItems()) + "\n";
		info += "  Number of objects = "	+ EXUstd::Num2String(mainObjects.NumberOfItems()) + "\n";
		//info += "  Number of materials = "+ EXUstd::Num2String(mainMaterials.NumberOfItems()) + "\n";
		info += "  Number of markers = "	+ EXUstd::Num2String(mainMarkers.NumberOfItems()) + "\n";
		info += "  Number of loads = " + EXUstd::Num2String(mainLoads.NumberOfItems()) + "\n";
		info += "  Number of sensors = " + EXUstd::Num2String(mainSensors.NumberOfItems()) + "\n";

		info += "  Number of ODE2 coordinates = " + EXUstd::Num2String(GetCSystemData().GetNumberOfCoordinatesODE2()) + "\n";
		info += "  Number of ODE1 coordinates = " + EXUstd::Num2String(GetCSystemData().GetNumberOfCoordinatesODE1()) + "\n";
		info += "  Number of AE coordinates   = " + EXUstd::Num2String(GetCSystemData().GetNumberOfCoordinatesAE()) + "\n";
		info += "  Number of data coordinates   = " + EXUstd::Num2String(GetCSystemData().GetNumberOfCoordinatesData()) + "\n";

		return info;
	}

};

#endif
