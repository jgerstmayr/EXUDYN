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
#include "Utilities/AdvancedStuff.h"
#include "Graphics/VisualizationSystemData.h"

#include "System/MainMaterial.h"
#include "System/MainMarker.h"
#include "System/MainLoad.h"
#include "System/MainNode.h"
#include "System/MainObject.h"
#include "System/MainSensor.h"

#include "Main/OutputVariable.h"
#include "Pymodules/PybindUtilities.h"

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
    //! forbid calls of MainSystemData constructor, as this would lead to an unusable system
    static MainSystemData* ForbidConstructor()
    {
        CHECKandTHROWstring("SystemData() may not be called. It is automatically created inside MainSystem and other usage of this class is not possible.");
        return new MainSystemData(); //this is never called
    }

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

	//! raise exception if configuration is illegal; used for several MainSystem functions called from Python
	void RaiseIfConfigurationIllegal(const char* functionName, ConfigurationType configuration, Index itemIndex, ItemType itemType) const;
	//! raise exception if configuration is not reference configuration; used for several MainSystem functions called from Python
	void RaiseIfNotConsistentNorReference(const char* functionName, ConfigurationType configuration, Index itemIndex, ItemType itemType) const;
	//! raise exception if system is not consistent or configuration is illegal; used for several MainSystem functions called from Python
	void RaiseIfNotConsistentOrIllegalConfiguration(const char* functionName, ConfigurationType configuration, Index itemIndex, ItemType itemType) const;
	//! raise exception if system is not consistent or configuration is illegal; used for several MainSystem functions called from Python
	void RaiseIfNotConsistent(const char* functionName, Index itemIndex, ItemType itemType) const;
	//! raise exception if reference configuration is used but variableType is not suited for reference configuration (e.g. velocitiy, etc.)
	void RaiseIfNotOutputVariableTypeForReferenceConfiguration(const char* functionName, OutputVariableType variableType, 
		ConfigurationType configuration, Index itemIndex, ItemType itemType) const;

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
	//! pybind read/write access to state vectors; data NOT COPIED
	//! format of dict: {'ODE2Coords':ODE2Coords, ...}
	py::dict PyGetSystemStateDict(ConfigurationType configurationType = ConfigurationType::Current,
		bool reference = false) const
	{
		auto pyDict = py::dict();

		pyDict["ODE1Coords"] = EPyUtils::VectorRef2NumPy<ResizableVectorParallel>(GetCSystemState(configurationType)->GetODE1Coords(), reference);
		pyDict["ODE1Coords_t"] = EPyUtils::VectorRef2NumPy<ResizableVectorParallel>(GetCSystemState(configurationType)->GetODE1Coords_t(), reference);
		pyDict["ODE2Coords"] = EPyUtils::VectorRef2NumPy<ResizableVectorParallel>(GetCSystemState(configurationType)->GetODE2Coords(), reference);
		pyDict["ODE2Coords_t"] = EPyUtils::VectorRef2NumPy<ResizableVectorParallel>(GetCSystemState(configurationType)->GetODE2Coords_t(), reference);
		pyDict["ODE2Coords_tt"] = EPyUtils::VectorRef2NumPy<ResizableVectorParallel>(GetCSystemState(configurationType)->GetODE2Coords_tt(), reference);
		pyDict["AECoords"] = EPyUtils::VectorRef2NumPy<ResizableVectorParallel>(GetCSystemState(configurationType)->GetAECoords(), reference);
		pyDict["dataCoords"] = EPyUtils::VectorRef2NumPy<ResizableVectorParallel>(GetCSystemState(configurationType)->GetDataCoords(), reference);

		//DOES NOT WORK: return time as reference:
		//would only work if reference to time is stored inside vector, then casted with VectorRef2NumPy
		//pyDict["time"] = py::cast(std::ref(GetCSystemState(configurationType)->time), py::return_value_policy::reference_internal);
		//pyDict["time"] = py::cast(&(GetCSystemState(configurationType)->time), py::return_value_policy::reference_internal);

		return pyDict;
	}



	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to ODE2 coords
	py::array_t<Real> GetODE2CoordsTotal(ConfigurationType configurationType = ConfigurationType::Current) const
	{
		//up to 100 coords, this is done without memory allocation:
		//this may be changed in future, if CData gets additionally ODE2CoordsTotal for at least current and visualization configuration!
		ResizableConstVectorBase<Real, 100> v = GetCSystemState(configurationType)->GetODE2Coords();
		if (configurationType != ConfigurationType::Reference)
		{
			v += GetCSystemState(ConfigurationType::Reference)->GetODE2Coords();
		}
		return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
	}

	//! pybind read access to ODE2 coords
	py::array_t<Real> GetODE2Coords(ConfigurationType configurationType = ConfigurationType::Current, bool copy = true) const
	{
		const Vector& v = GetCSystemState(configurationType)->GetODE2Coords();
		return EPyUtils::VectorRef2NumPy(v, !copy);
		//return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
	}

	//! pybind write access to ODE2 coords
	void SetODE2Coords(const std::vector<Real>& v, ConfigurationType configurationType = ConfigurationType::Current)
	{
		CHECKandTHROW((Index)v.size() == GetCSystemState(configurationType)->GetODE2Coords().NumberOfItems(),"SystemData::SetODE2Coords: incompatible size of vectors");
		GetCSystemState(configurationType)->SetODE2Coords(v);
	}

	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to ODE2_t coords
	py::array_t<Real> GetODE2Coords_t(ConfigurationType configurationType = ConfigurationType::Current, bool copy = true) const
	{
		const Vector& v = GetCSystemState(configurationType)->GetODE2Coords_t();
		return EPyUtils::VectorRef2NumPy(v, !copy);
		//return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
	}

	//! pybind write access to ODE2_t coords
	void SetODE2Coords_t(const std::vector<Real>& v, ConfigurationType configurationType = ConfigurationType::Current)
	{
		CHECKandTHROW((Index)v.size() == GetCSystemState(configurationType)->GetODE2Coords_t().NumberOfItems(), "SystemData::SetODE2Coords_t: incompatible size of vectors");
		GetCSystemState(configurationType)->SetODE2Coords_t(v);
	}

	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to ODE2_t coords
	py::array_t<Real> GetODE2Coords_tt(ConfigurationType configurationType = ConfigurationType::Current, bool copy = true) const
	{
		const Vector& v = GetCSystemState(configurationType)->GetODE2Coords_tt();
		return EPyUtils::VectorRef2NumPy(v, !copy);
		//return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
	}

	//! pybind write access to ODE2_t coords
	void SetODE2Coords_tt(const std::vector<Real>& v, ConfigurationType configurationType = ConfigurationType::Current)
	{
		CHECKandTHROW((Index)v.size() == GetCSystemState(configurationType)->GetODE2Coords_tt().NumberOfItems(), "SystemData::SetODE2Coords_tt: incompatible size of vectors");
		GetCSystemState(configurationType)->SetODE2Coords_tt(v);
	}

	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to ODE1 coords
	py::array_t<Real> GetODE1Coords(ConfigurationType configurationType = ConfigurationType::Current, bool copy = true) const
	{
		const Vector& v = GetCSystemState(configurationType)->GetODE1Coords();
		return EPyUtils::VectorRef2NumPy(v, !copy);
		//return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
	}

	//! pybind write access to ODE1 coords
	void SetODE1Coords(const std::vector<Real>& v, ConfigurationType configurationType = ConfigurationType::Current)
	{
		CHECKandTHROW((Index)v.size() == GetCSystemState(configurationType)->GetODE1Coords().NumberOfItems(), "SystemData::SetODE1Coords: incompatible size of vectors");
		GetCSystemState(configurationType)->SetODE1Coords(v);
	}

	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to ODE1_t coords
	py::array_t<Real> GetODE1Coords_t(ConfigurationType configurationType = ConfigurationType::Current, bool copy = true) const
	{
		const Vector& v = GetCSystemState(configurationType)->GetODE1Coords_t();
		return EPyUtils::VectorRef2NumPy(v, !copy);
		//return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
	}

	//! pybind write access to ODE1_t coords
	void SetODE1Coords_t(const std::vector<Real>& v, ConfigurationType configurationType = ConfigurationType::Current)
	{
		CHECKandTHROW((Index)v.size() == GetCSystemState(configurationType)->GetODE1Coords().NumberOfItems(), "SystemData::SetODE1Coords_t: incompatible size of vectors");
		GetCSystemState(configurationType)->SetODE1Coords_t(v);
	}

	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to ODE2 coords
	py::array_t<Real> GetAECoords(ConfigurationType configurationType = ConfigurationType::Current, bool copy = true) const
	{
		const Vector& v = GetCSystemState(configurationType)->GetAECoords();
		return EPyUtils::VectorRef2NumPy(v, !copy);
		//return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
	}

	//! pybind write access to AE coords
	void SetAECoords(const std::vector<Real>& v, ConfigurationType configurationType = ConfigurationType::Current)
	{
		CHECKandTHROW((Index)v.size() == GetCSystemState(configurationType)->GetAECoords().NumberOfItems(), "SystemData::SetAECoords: incompatible size of vectors");
		GetCSystemState(configurationType)->SetAECoords(v);
	}
	//+++++++++++++++++++++++++++++++++++
	//! pybind read access to Data coords
	py::array_t<Real> GetDataCoords(ConfigurationType configurationType = ConfigurationType::Current, bool copy = true) const
	{
		const Vector& v = GetCSystemState(configurationType)->GetDataCoords();
		return EPyUtils::VectorRef2NumPy(v, !copy);
		//return py::array_t<Real>(v.NumberOfItems(), v.GetDataPointer());
	}

	//! pybind write access to AE coords
	void SetDataCoords(const std::vector<Real>& v, ConfigurationType configurationType = ConfigurationType::Current)
	{
		CHECKandTHROW((Index)v.size() == GetCSystemState(configurationType)->GetDataCoords().NumberOfItems(), "SystemData::SetDataCoords: incompatible size of vectors");
		GetCSystemState(configurationType)->SetDataCoords(v);
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! local to global ODE2 transformation lists; returned as python list
	std::vector<Index> PyGetObjectLocalToGlobalODE2(Index objectNumber)
	{
		if (objectNumber >= cSystemData->GetLocalToGlobalODE2().NumberOfItems())
		{
			PyError("GetObjectLTGODE2: illegal index");
			return std::vector<Index>();
		}
		return cSystemData->GetLocalToGlobalODE2()[objectNumber];
	}
	//! local to global ODE1 transformation lists; returned as python list
	std::vector<Index> PyGetObjectLocalToGlobalODE1(Index objectNumber)
	{
		if (objectNumber >= cSystemData->GetLocalToGlobalODE1().NumberOfItems())
		{
			PyError("GetObjectLTGODE1: illegal index");
			return std::vector<Index>();
		}
		return cSystemData->GetLocalToGlobalODE1()[objectNumber];
	}
	//! local to global AE transformation lists; returned as python list
	std::vector<Index> PyGetObjectLocalToGlobalAE(Index objectNumber)
	{
		if (objectNumber >= cSystemData->GetLocalToGlobalAE().NumberOfItems())
		{
			PyError("GetObjectLTGAE: illegal index");
			return std::vector<Index>();
		}
		return cSystemData->GetLocalToGlobalAE()[objectNumber];
	}
	//! local to global Data transformation lists; returned as python list
	std::vector<Index> PyGetObjectLocalToGlobalData(Index objectNumber)
	{
		if (objectNumber >= cSystemData->GetLocalToGlobalData().NumberOfItems())
		{
			PyError("GetObjectLTGData: illegal index");
			return std::vector<Index>();
		}
		return cSystemData->GetLocalToGlobalData()[objectNumber];
	}

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //! create and return list of LTG list for node; slow as it creates a new std::vector
    std::vector<Index> PyGetNodeLocalToGlobalODE2(Index nodeNumber)
    {
        if (nodeNumber >= cSystemData->GetCNodes().NumberOfItems())
        {
            PyError("GetNodeLocalToGlobalODE2: illegal index");
            return std::vector<Index>();
        }
        std::vector<Index> indexList;
        Index n = cSystemData->GetCNode(nodeNumber).GetNumberOfODE2Coordinates();

        if (EXUstd::IsOfType(cSystemData->GetCNode(nodeNumber).GetNodeGroup(), CNodeGroup::ODE2variables) &&
            n != 0)
        {
            Index cntStart = cSystemData->GetCNode(nodeNumber).GetGlobalODE2CoordinateIndex();
            for (Index i = 0; i < n; i++)
            {
                indexList.push_back(cntStart++);
            }
        }
        return indexList;
    }

    //! create and return list of LTG list for node; slow as it creates a new std::vector
    std::vector<Index> PyGetNodeLocalToGlobalODE1(Index nodeNumber)
    {
        if (nodeNumber >= cSystemData->GetCNodes().NumberOfItems())
        {
            PyError("GetNodeLocalToGlobalODE1: illegal index");
            return std::vector<Index>();
        }
        std::vector<Index> indexList;
        Index n = cSystemData->GetCNode(nodeNumber).GetNumberOfODE1Coordinates();

        if (EXUstd::IsOfType(cSystemData->GetCNode(nodeNumber).GetNodeGroup(), CNodeGroup::ODE1variables) &&
            n != 0)
        {
            Index cntStart = cSystemData->GetCNode(nodeNumber).GetGlobalODE1CoordinateIndex();
            for (Index i = 0; i < n; i++)
            {
                indexList.push_back(cntStart++);
            }
        }
        return indexList;
    }

    //! create and return list of LTG list for node; slow as it creates a new std::vector
    std::vector<Index> PyGetNodeLocalToGlobalAE(Index nodeNumber)
    {
        if (nodeNumber >= cSystemData->GetCNodes().NumberOfItems())
        {
            PyError("GetNodeLocalToGlobalAE: illegal index");
            return std::vector<Index>();
        }
        std::vector<Index> indexList;
        Index n = cSystemData->GetCNode(nodeNumber).GetNumberOfAECoordinates();

        if (EXUstd::IsOfType(cSystemData->GetCNode(nodeNumber).GetNodeGroup(), CNodeGroup::AEvariables) &&
            n != 0)
        {
            Index cntStart = cSystemData->GetCNode(nodeNumber).GetGlobalAECoordinateIndex();
            for (Index i = 0; i < n; i++)
            {
                indexList.push_back(cntStart++);
            }
        }
        return indexList;
    }

    //! create and return list of LTG list for node; slow as it creates a new std::vector
    std::vector<Index> PyGetNodeLocalToGlobalData(Index nodeNumber)
    {
        if (nodeNumber >= cSystemData->GetCNodes().NumberOfItems())
        {
            PyError("GetNodeLocalToGlobalData: illegal index");
            return std::vector<Index>();
        }
        std::vector<Index> indexList;
        Index n = cSystemData->GetCNode(nodeNumber).GetNumberOfDataCoordinates();

        if (EXUstd::IsOfType(cSystemData->GetCNode(nodeNumber).GetNodeGroup(), CNodeGroup::DataVariables) &&
            n != 0)
        {
            Index cntStart = cSystemData->GetCNode(nodeNumber).GetGlobalDataCoordinateIndex();
            for (Index i = 0; i < n; i++)
            {
                indexList.push_back(cntStart++);
            }
        }
        return indexList;
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //Loads

    //! pybind add load dependencies
    void PyAddODE2LoadDependencies(Index loadNumber, const std::vector<Index>& globalODE2coordinates)
    {
        Index nLoads = cSystemData->GetCLoads().NumberOfItems();
        if (loadNumber >= nLoads)
        {
            PyError("AddODE2LoadDependencies: invalid load number");
        }

        //check if load dependencies are initialized:
        if (cSystemData->GetLoadsODE2dependencies().NumberOfItems() == 0)
        {
            ArrayIndex emptyArray;
            for (Index i=0; i < nLoads; i++)
            {
                cSystemData->GetLoadsODE2dependencies().Append(emptyArray);
            }
        }
        else if (cSystemData->GetLoadsODE2dependencies().NumberOfItems() != nLoads)
        {
            PyError("AddODE2LoadDependencies: inconsistent size of systemData.loadsODE2dependencies; call Assemble() first");
        }
        Index nODE2 = cSystemData->GetNumberOfCoordinatesODE2();
        for (Index k = 0; k < (Index)globalODE2coordinates.size(); k++)
        {
            Index c = globalODE2coordinates[k];
            if (!EXUstd::IndexIsInRange(c, 0, nODE2))
            {
                PyError(STDstring("AddODE2LoadDependencies: coordinate index ")+EXUstd::ToString(k)+" is "+ EXUstd::ToString(c)+" which is not in valid range [0,"+ EXUstd::ToString(nODE2)+"]");
            }
            cSystemData->GetLoadsODE2dependencies()[loadNumber].Append(c);
        }
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

    STDstring PyInfoLTG() const
    {
        STDstring info;
        Index cnt;
        cnt = 0;
        for (auto item : cSystemData->GetLocalToGlobalODE2()) {
            info += "object " + EXUstd::ToString(cnt++) + " ODE2 LTG=" + EXUstd::ToString(*item) + "\n";
        }
        cnt = 0;
        for (auto item : cSystemData->GetLocalToGlobalODE1()) {
            info += "object " + EXUstd::ToString(cnt++) + " ODE1 LTG=" + EXUstd::ToString(*item) + "\n";
        }
        cnt = 0;
        for (auto item : cSystemData->GetLocalToGlobalAE()) {
            info += "object " + EXUstd::ToString(cnt++) + " AE LTG  =" + EXUstd::ToString(*item) + "\n";
        }
        cnt = 0;
        for (auto item : cSystemData->GetLocalToGlobalData()) {
            info += "object " + EXUstd::ToString(cnt++) + " Data LTG=" + EXUstd::ToString(*item) + "\n";
        }

        //load dependencies:
        cnt = 0;
        for (auto item : cSystemData->GetLoadsODE2dependencies()) {
            info += "load " + EXUstd::ToString(cnt++) + " ODE2 dependencies=" + EXUstd::ToString(*item) + "\n";
        }
        cnt = 0;
        for (auto item : cSystemData->GetLoadsODE1dependencies()) {
            info += "load " + EXUstd::ToString(cnt++) + " ODE1 dependencies=" + EXUstd::ToString(*item) + "\n";
        }
        cnt = 0;
        for (auto item : cSystemData->GetLoadsAEdependencies()) {
            info += "load " + EXUstd::ToString(cnt++) + " AE dependencies  =" + EXUstd::ToString(*item) + "\n";
        }

        return info;
    }

	//! print information summary on system
	STDstring PyInfoSummary() const
	{
		STDstring info;
		info += "  Number of nodes= "		+ EXUstd::ToString(mainNodes.NumberOfItems()) + "\n";
		info += "  Number of objects = "	+ EXUstd::ToString(mainObjects.NumberOfItems()) + "\n";
		//info += "  Number of materials = "+ EXUstd::ToString(mainMaterials.NumberOfItems()) + "\n";
		info += "  Number of markers = "	+ EXUstd::ToString(mainMarkers.NumberOfItems()) + "\n";
		info += "  Number of loads = " + EXUstd::ToString(mainLoads.NumberOfItems()) + "\n";
		info += "  Number of sensors = " + EXUstd::ToString(mainSensors.NumberOfItems()) + "\n";

		info += "  Number of ODE2 coordinates = " + EXUstd::ToString(GetCSystemData().GetNumberOfCoordinatesODE2()) + "\n";
		info += "  Number of ODE1 coordinates = " + EXUstd::ToString(GetCSystemData().GetNumberOfCoordinatesODE1()) + "\n";
		info += "  Number of AE coordinates   = " + EXUstd::ToString(GetCSystemData().GetNumberOfCoordinatesAE()) + "\n";
		info += "  Number of data coordinates   = " + EXUstd::ToString(GetCSystemData().GetNumberOfCoordinatesData()) + "\n";

		return info;
	}

};

#endif
