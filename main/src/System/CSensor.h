/** ***********************************************************************************************
* @class        CSensor
* @brief        A sensor is used to measure output variables of nodes, objects and markers
*
* @author       Gerstmayr Johannes
* @date         2020-01-22 (generated)
* @date         2020-01-22 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
				- email: johannes.gerstmayr@uibk.ac.at
				- weblink: missing
				
************************************************************************************************ */
#ifndef CSENSOR__H
#define CSENSOR__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Linalg/BasicLinalg.h"

#include "Main/OutputVariable.h"


class CSensor
{
protected: 

public: 
	virtual ~CSensor() {} //added for correct deletion of derived classes

	//! clone object; specifically for copying instances of derived class, for automatic memory management e.g. in ObjectContainer
	virtual CSensor* GetClone() const { return new CSensor(*this); }

	//! determine type of marker in order to decide according action in assembly; to be filled in derived class
	virtual SensorType GetType() const {
		CHECKandTHROWstring("Invalid call to CSensor::GetType");
		return SensorType::_None;
	}

	//! get type dependent index (node, object, load, ...)-index in global lists
	virtual Index GetTypeDependentIndex() const
	{
		switch (GetType())
		{
		//case SensorType::_None:  return 0; //should not occur!
		case SensorType::Node:  return GetNodeNumber();
		case SensorType::Object: return GetObjectNumber();
		case SensorType::Body:  return GetObjectNumber();
		case SensorType::SuperElement:  return GetObjectNumber();
		case SensorType::Marker: return GetMarkerNumber();
		case SensorType::Load:  return GetLoadNumber();
		default: SysError("Sensor::GetTypeDependentIndex: invalid sensor type");  return 0;
		}
	}

	//! if object/body Sensor: get object number (otherwise assertion)
	virtual Index GetObjectNumber() const {
		CHECKandTHROWstring("Invalid call to CSensor::GetBodyNumber");
		return EXUstd::InvalidIndex;
	}

	//! if node Sensor: get node number (otherwise assertion)
	virtual Index GetNodeNumber() const {
		CHECKandTHROWstring("Invalid call to CSensor::GetNodeNumber");
		return EXUstd::InvalidIndex;
	}

	//! if load Sensor: get load number (otherwise assertion)
	virtual Index GetLoadNumber() const {
		CHECKandTHROWstring("Invalid call to CSensor::GetLoadNumber");
		return EXUstd::InvalidIndex;
	}

	//! if marker Sensor: get load number (otherwise assertion)
	virtual Index GetMarkerNumber() const {
		CHECKandTHROWstring("Invalid call to CSensor::GetMarkerNumber");
		return EXUstd::InvalidIndex;
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//common access functions for all types of sensors; NEED TO BE OVERWRITTEN!

	//! get OutputVariableType for node, body, object and marker sensors; unused for loads
	virtual OutputVariableType GetOutputVariableType() const {
		CHECKandTHROWstring("Invalid call to CSensor::GetOutputVariableType");
		return OutputVariableType::_None;
	}

	//! get sensor values into values vector
	virtual void GetSensorValues(const CSystemData& cSystemData, Vector& values, ConfigurationType configuration = ConfigurationType::Current) const {
		CHECKandTHROWstring("Invalid call to CSensor::GetSensorValues");
	}

	//localPosition stored in SensorBody
	////! get sensor values into values vector; special call for body sensors: additional localPosition needed
	//virtual void GetSensorValuesBody(const CSystemData& cSystemData, Vector& values, const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const {
	//	CHECKandTHROWstring("Invalid call to CSensor::GetSensorValuesBody");
	//}

	//! get sensor values into values vector; special call for body sensors: additional localPosition needed
	virtual bool GetWriteToFileFlag() const {
		CHECKandTHROWstring("Invalid call to CSensor::GetWriteToFileFlag");
		return false;
	}

	////! time period used to output sensor values (e.g.: 0 ... always, 0.01 .. every 10 milliseconds, ...)
	//virtual Real GetFileWritingInterval() const {
	//	CHECKandTHROWstring("Invalid call to CSensor::GetFileWritingInterval");
	//}

	//! directory and file name for sensor file output
	virtual STDstring GetFileName() const {
		CHECKandTHROWstring("Invalid call to CSensor::FileWritingInterval"); return "";
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	virtual void Print(std::ostream& os) const
	{
		os << "CSensor";
		os << "\n";
	}

	friend std::ostream& operator<<(std::ostream& os, const CSensor& object)
	{
		object.Print(os);
		return os;
	}

};


#endif
