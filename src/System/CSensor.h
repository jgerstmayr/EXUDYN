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
				- weblink: https://github.com/jgerstmayr/EXUDYN
				
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
	ResizableMatrix internalStorage; //!< resizable matrix for internal storage of sensor values; automatically resized by solver
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
		case SensorType::KinematicTree:  return GetObjectNumber();
		case SensorType::Marker: return GetMarkerNumber();
		case SensorType::Load:  return GetLoadNumber();
		case SensorType::UserFunction:  return 0; //would be several numbers ...
		default: SysError("Sensor::GetTypeDependentIndex: invalid sensor type");  return 0;
		}
	}
	//! check if sensor has object number
	virtual bool HasObjectNumber() const {
		if ((GetType() == SensorType::Object) || 
			(GetType() == SensorType::Body) ||
			(GetType() == SensorType::SuperElement) || 
			(GetType() == SensorType::KinematicTree)
			) {return true;}
		else { return false; }
	}

	//! if object/body Sensor: get object number (otherwise exception)
	virtual Index GetObjectNumber() const {
		CHECKandTHROWstring("Invalid call to CSensor::GetObjectNumber");
		return EXUstd::InvalidIndex;
	}

	//! if object/body Sensor: set object number (otherwise exception)
	virtual void SetObjectNumber(Index objectNumber) {
		CHECKandTHROWstring("Invalid call to CSensor::SetObjectNumber");
	}

	//! if node Sensor: get node number (otherwise exception)
	virtual Index GetNodeNumber() const {
		CHECKandTHROWstring("Invalid call to CSensor::GetNodeNumber");
		return EXUstd::InvalidIndex;
	}

	//! if node Sensor: set node number (otherwise exception)
	virtual void SetNodeNumber(Index nodeNumber) {
		CHECKandTHROWstring("Invalid call to CSensor::SetNodeNumber");
	}

	//! if load Sensor: get load number (otherwise exception)
	virtual Index GetLoadNumber() const {
		CHECKandTHROWstring("Invalid call to CSensor::GetLoadNumber");
		return EXUstd::InvalidIndex;
	}

	//! if load Sensor: set load number (otherwise exception)
	virtual void SetLoadNumber(Index loadNumber) {
		CHECKandTHROWstring("Invalid call to CSensor::SetLoadNumber");
	}

	//! if marker Sensor: get marker number (otherwise exception)
	virtual Index GetMarkerNumber() const {
		CHECKandTHROWstring("Invalid call to CSensor::GetMarkerNumber");
		return EXUstd::InvalidIndex;
	}

	//! if marker Sensor: set marker number (otherwise exception)
	virtual void SetMarkerNumber(Index markerNumber) {
		CHECKandTHROWstring("Invalid call to CSensor::SetMarkerNumber");
	}

	//! if user function Sensor: get sensor number (otherwise exception)
	virtual Index GetSensorNumber(Index localIndex) const {
		CHECKandTHROWstring("Invalid call to CSensor::GetSensorNumber");
		return EXUstd::InvalidIndex;
	}

	//! if node Sensor: set node number (otherwise exception)
	virtual void SetSensorNumber(Index localIndex, Index sensorNumber) {
		CHECKandTHROWstring("Invalid call to CSensor::SetSensorNumber");
	}

	//! if user function Sensor: get number of related sensors
	virtual Index GetNumberOfSensors() const {
		return 0; //ususally no dependent sensors
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

	//! get flag which determines if sensor data is written to file
	virtual bool GetWriteToFileFlag() const {
		CHECKandTHROWstring("Invalid call to CSensor::GetWriteToFileFlag");
		return false;
	}

	//! get flag which determines if sensor data is stored internally
	virtual bool GetStoreInternalFlag() const {
		CHECKandTHROWstring("Invalid call to CSensor::GetStoreInternalFlag");
		return false;
	}

	//! read access to internal storage
	virtual const ResizableMatrix& GetInternalStorage() const { return internalStorage; }

	//! write access to internal storage
	virtual ResizableMatrix& GetInternalStorage() { return internalStorage; }

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
