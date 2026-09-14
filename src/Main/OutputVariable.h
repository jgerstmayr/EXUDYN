/** ***********************************************************************************************
* @class	    OutputVariable
* @brief		
* @details		Details:
 				- ...
*
* @author		Gerstmayr Johannes
* @date			2018-05-17 (generated)
* @pre			continuously extended for new types
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
#ifndef OUTPUTVARIABLE__H
#define OUTPUTVARIABLE__H

#include "Utilities/ReleaseAssert.h"
#include <initializer_list>
#include "Utilities/BasicDefinitions.h" //defines Real
#include "Utilities/ResizableArray.h" 


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//keep these lists synchronized with PybindModule.cpp lists

namespace Marker { //==>put into pybindings file in future!
	//! Markers transfer observable and controllable quantities into object/node/... coordinates
	//  e.g. the MarkerBodyRigid (+ the according body function) defines how to transform a torque to the body 
	//  coordinates, or how to measure the orientation of the body;
	//  use unscoped enum to directly translate to Index
	//  all marker functions address ODE2 coordinates, flag ODE1 is not set
	enum Type {
		_None = 0, //marks that no type is used
		//bits to determine the item which is acted on (not relevant for connector, load, OutputVariable):
		//1+4==Body, 2==Node, 4==Object, 2+4=Node+Object
		Body = 1 << 0,						//!< 1==Body, 0==other (must be also object!!!)
		Node = 1 << 1,						//!< 2==Node, 0=other
		Object = 1 << 2,					//!< 4==Object, 0=other
		SuperElement = 1 << 3,				//!< Marker only applicable to SuperElements; accesses nodes (virtual nodes) of SuperElements
		KinematicTree = 1 << 4,				//!< Marker only applicable to KinematicTree; accesses nodes (virtual nodes) of KinematicTree
		//bits to determine the kind of quantity is involved (relevant for: connector, load, OutputVariable):
		//keep this list SYNCHRONIZED with AccessFunctionType:
		Position = 1 << 5,					//!< can measure position, apply Distance constraint
		Orientation = 1 << 6,				//!< can measure rotation, apply general rigid body constraint (if Position is set)
		Coordinate = 1 << 7,				//!< access any coordinate (always available)
		Coordinates = 1 << 8,				//!< access all coordinates (always available)
		//bits for geometrical dimension: force applied to volume, displacement of volume (center of mass ...)
		//BodyPoint = 1 << xx, //default is always point; not necessary for Body+Position!
		BodyLine = 1 << 9,					//!< represents a line load (vector load applied to line)
		BodySurface = 1 << 10,				//!< represents a surface load / connector (e.g. for revolute joint with FE-mesh)
		BodyVolume = 1 << 11,				//!< volume load ==> usually gravity
		BodyMass = 1 << 12,					//!< volume load ==> usually gravity
		BodySurfaceNormal = 1 << 13,		//!< for surface pressure (uses scalar load)
		//++++for SuperElementMarkers:
		MultiNodal = 1 << 14,				//!< multinodal marker uses a weighting matrix for transformation of node values to marker value (e.g., list of positions averaged to one position)
		ReducedCoordinates = 1 << 15,		//!< multinodal marker uses a weighting matrix for transformation of node values to marker value (e.g., list of positions averaged to one position)
		//Rotv1v2v3 = 1 << 1xx,				//!< for special joints that need a attached triad; in fact, a marker of orientation type must also provide Rotv1v2v3

		ODE1 = 1 << 16,						//!< marker addresses ODE1 Coordinate(s) (otherwise, standard is ODE2 for Coordinate(s) )
		//NOTE that SuperElementAlternativeRotationMode = (1 << 31) ==> do not use this value here!

		JacobianDerivativeNonZero = 1 << 17,//!< flag which informs that there is a derivative of the marker jacobian, being non-zero (e.g. for rotations)
		JacobianDerivativeAvailable = 1 << 18,//!< flag which informs that derivative of the marker jacobian is implemented
		HasPostNewton = 1 << 19,			//!< flag which informs that PostNewton function has to be called

		Beam2DShape = 1 << 20,				//!< for access to 2D beam shape
		Beam3DShape = 1 << 21,				//!< for access to 3D beam shape

		EndOfEnumList = 1 << 22				//!< KEEP THIS AS THE (2^i) MAXIMUM OF THE ENUM LIST!!!
		//available Types are, e.g.
		//Node: 2+4+16, 2+4+8, 2+16
		//Body: 1+4+16, 1+4+8, 1+16, 1+4+128, ...
	};
	//! transform type into string (e.g. for error messages); this is slow and cannot be used during computation!
	inline STDstring GetTypeString(Type var)
	{
		STDstring t; //empty string
		if (var == Marker::_None) { t = "_None/Undefined"; }
		if (var & Body) { t += "Body"; }
		if (var & Node) { t += "Node"; }
		if ((var & Object) && !(var & Body)) { t += "Object"; }
		if (var & SuperElement) { t += "SuperElement"; }
		if (var & KinematicTree) { t += "KinematicTree"; }
		if (var & Position) { t += "Position"; }
		if (var & Orientation) { t += "Orientation"; }
		if (var & Coordinate) { t += "Coordinate"; }
		if (var & Coordinates) { t += "Coordinates"; }
		if (var & BodyLine) { t += "Line"; } //'Body' already added via (var & Body)
		if (var & BodySurface) { t += "Surface"; } //'Body' already added via (var & Body)
		if (var & BodyVolume) { t += "Volume"; } //'Body' already added via (var & Body)
		if (var & BodyMass) { t += "Mass"; } //'Body' already added via (var & Body)
		if (var & BodySurfaceNormal) { t += "SurfaceNormal"; } //'Body' already added via (var & Body)
		
		if (var & MultiNodal) { t += "MultiNodal"; }
		if (var & ReducedCoordinates) { t += "ReducedCoordinates"; } //'Body' already added via (var & Body)
		if (var & ODE1) { t += "ODE1"; } //'Body' already added via (var & Body)

		//JacobianDerivativeNonZero, JacobianDerivativeAvailable, HasPostNewton not included on purpose (anyway, there is no reconstruction from string!)
		//if (var & JacobianDerivativeNonZero) { t += "JacobianDerivativeNonZero"; }
		//if (var & JacobianDerivativeAvailable) { t += "JacobianDerivativeAvailable"; }
		//if (var & HasPostNewton) { t += "HasPostNewton"; }

		if (var & Beam2DShape) { t += "Beam2DShape"; }
		if (var & Beam3DShape) { t += "Beam3DShape"; }

		if (t.length() == 0) { CHECKandTHROWstring("Marker::GetTypeString(...) called for invalid type!"); }

		return t;
	}

}

enum class AccessFunctionType { //determines which connectors/forces can be applied to object; underscores mark the derivative w.r.t. q
	_None = 0, //marks that no type is used

	//keep this list SYNCHRONIZED with MarkerType:
	TranslationalVelocity_qt = (Index)Marker::Position,			//for application of forces, position constraints
	AngularVelocity_qt = (Index)Marker::Orientation,			//for application of torques, rotational constraints
	Coordinate_q = (Index)Marker::Coordinate,					//for application of generalized forces
	DisplacementLineIntegral_q = (Index)Marker::BodyLine,		//for line loads
	DisplacementSurfaceIntegral_q = (Index)Marker::BodySurface, //for surface loads
	DisplacementVolumeIntegral_q = (Index)Marker::BodyVolume,	//for distributed (body-volume) loads
	DisplacementMassIntegral_q = (Index)Marker::BodyMass,		//for distributed (body-mass) loads
	DisplacementSurfaceNormalIntegral_q = (Index)Marker::BodySurfaceNormal, //for surface loads: CAUTION: pressure acts normal to surface!!!
	SuperElement = (Index)Marker::SuperElement,					//for super elements, using TranslationalVelocity_qt and AngularVelocity_qt
	KinematicTree = (Index)Marker::KinematicTree,				//for KinematicTree, using TranslationalVelocity_qt and AngularVelocity_qt
	//Rotv1v2v3_q = (Index)Marker::Rotv1v2v3,					//for joints, e.g., prismatic or rigid body; in fact, a marker of type orientation must also provide Rotv1v2v3
	
	JacobianTtimesVector_q = (1 << 30),							//access function to compute derivative of jacobian^T times vector (provided in markerData.vectorValue)
	SuperElementAlternativeRotationMode= (1 << 31)				//for super elements, using TranslationalVelocity_qt and AngularVelocity_qt

};

enum class LoadType {
    _None = 0, //marks that no type is used

	//keep this list SYNCHRONIZED with MarkerType:
	Force = (Index)Marker::Position,		//!< vector force applied to BodyPosition, BodyVolume, BodySurface, ...
	Torque = (Index)Marker::Orientation,	//!< vector torque applied to BodyPosition, BodyVolume, BodySurface, ...
	Coordinate = (Index)Marker::Coordinate,		//!< scalar force applied to Body/NodeCoordinate [usually N or Nm, depends on coordinate]
	ForcePerVolume = (Index)Marker::BodyVolume,	//!< vector force applied to BodyVolume, e.g.  as (rho*g) [N/m^3]
	ForcePerMass = (Index)Marker::BodyMass,		//!< vector force applied to BodyMass, e.g.  as (g) [m/s^2]
	SurfacePressure = (Index)Marker::BodySurfaceNormal,	//!< scalar force applied to BodySurface [N/m^2]
    //NOT VALID: EndOfEnumList = 1 << 6 //KEEP THIS AS THE (2^i) MAXIMUM OF THE ENUM LIST!!!
};


//! sensor types are used to identify what items are measured
enum class SensorType {
	_None = 0, //marks that no type is used
	Node   = 1 << 0, //!< use OutputVariableType
	Object = 1 << 1, //!< use OutputVariableType
	Body = 1 << 2, //!< use OutputVariableType; additionally has localPosition
	SuperElement = 1 << 3, //!< use OutputVariableType; additionally has localPosition
	KinematicTree = 1 << 4, //!< use OutputVariableType; additionally has localPosition+linkNumber
	Marker = 1 << 5, //!< NOT implemented yet, needs OutputVariableType in markers!
	Load = 1 << 6, //!< measure prescribed loads, in order to track, e.g., user defined loads or controlled loads
	UserFunction = 1 << 7, //!< user defined sensor, especially for sensor fusion
};

//! convert SensorType to a string (used for output, type comparison, ...)
inline const char* GetSensorTypeString(SensorType var)
{
	switch (var)
	{
	case SensorType::_None: return "_None";
	case SensorType::Node: return "Node";
	case SensorType::Object: return "Object";
	case SensorType::Body: return "Body";
	case SensorType::SuperElement: return "SuperElement";
	case SensorType::KinematicTree: return "KinematicTree";
	case SensorType::Marker: return "Marker";
	case SensorType::Load: return "Load";
	case SensorType::UserFunction: return "UserFunction";
	default: SysError("GetSensorTypeString: invalid variable type");  return "Invalid";
	}
}

//! used mainly to show which jacobians are available analytically in objects; can be combined binary to see, which jacobian is available
namespace JacobianType {
	//! used mainly to show which jacobians are available analytically in objects; can be combined binary to see, which jacobian is available
	enum Type {
		_None = 0,				//marks that no type is available
		ODE2_ODE2 = 1 << 1,		//derivative of ODE2 equations with respect to ODE2 variables
		ODE2_ODE2_t = 1 << 2,	//derivative of ODE2 equations with respect to ODE2_t (velocity) variables
		ODE1_ODE1 = 1 << 3,		//derivative of ODE1 equations with respect to ODE1 variables
		ODE1_ODE2 = 1 << 4,		//derivative of ODE1 equations with respect to ODE2 variables
		ODE1_ODE2_t = 1 << 5,	//derivative of ODE1 equations with respect to ODE2_t variables
		ODE2_ODE1 = 1 << 6,		//derivative of ODE2 equations with respect to ODE1 variables
		AE_ODE2 = 1 << 7,		//derivative of AE (algebraic) equations with respect to ODE2 variables
		AE_ODE2_t = 1 << 8,		//derivative of AE (algebraic) equations with respect to ODE2_t (velocity) variables
		AE_ODE1 = 1 << 9,		//derivative of AE (algebraic) equations with respect to ODE1 variables
		AE_AE = 1 << 10,			//derivative of AE (algebraic) equations with respect to AE variables
		//
		ODE2_ODE2_function = 1 << 11,	//function available for derivative of ODE2 equations with respect to ODE2 variables
		ODE2_ODE2_t_function = 1 << 12,	//function available for derivative of ODE2 equations with respect to ODE2_t (velocity) variables; MUST exist, if ODE2_ODE2_function exists!
		ODE1_ODE1_function = 1 << 13,	//function available for derivative of ODE1 equations with respect to ODE1 variables
		ODE1_ODE2_function = 1 << 14,	//...
		ODE1_ODE2_t_function = 1 << 15,	//...
		ODE2_ODE1_function = 1 << 16,	//...

		AE_ODE2_function = 1 << 17,		//function available for derivative of AE (algebraic) equations with respect to ODE2 variables
		AE_ODE2_t_function = 1 << 18,	//function available for derivative of AE (algebraic) equations with respect to ODE2_t (velocity) variables
		AE_ODE1_function = 1 << 19,		//function available for derivative of AE (algebraic) equations with respect to ODE1 variables
		AE_AE_function = 1 << 20,		//function available for derivative of AE (algebraic) equations with respect to AE variables

		ALL_AE_DERIV = AE_ODE2 + AE_ODE2_t + AE_ODE1 + AE_AE //sums up all bits for AE derivatives ==> for ObjectJacobianAE
	};

}


//! OutputVariable used for output data in objects, nodes, loads, ...
//! The enum, GetOutputVariableTypeString() and
//! IsOutputVariableTypeForReferenceConfiguration() are GENERATED from the registrator table
//! in definitions/outputVariableTypes.py, which is also what defines the Python enum and the
//! documentation table. Add an output variable there, not here.
#include "Autogenerated/OutputVariableTypes.h"

//yet unused
//enum class OutputVariableUnit {
//    NoUnit = 0, //OutputVariable has no unit (e.g. strain)
//    Length = 1, LengthPerTime = 2, LengthPerTimeSquared = 3,
//    Force = 4, ForceLength = 5, ForcePerLength = 6, ForcePerLengthSquared = 7,
//    Rotations = 8, RotationsPerTime = 9, RotationsPerTimeSquared = 10
//};

//yet unused
//inline Index2 GetOutputVariableDimension(OutputVariableType outputVariableType)
//{
//    switch (outputVariableType)
//    {
//        case OutputVariableType::_None: return Index2(0, 0);
//        case OutputVariableType::Position: return Index2(3, 0);
//        case OutputVariableType::Displacement: return Index2(3, 0);
//        case OutputVariableType::Velocity: return Index2(3, 0);
//        case OutputVariableType::Acceleration: return Index2(3, 0);
//        case OutputVariableType::RotationMatrix: return Index2(3, 3);
//        case OutputVariableType::AngularVelocity: return Index2(3, 0);
//        case OutputVariableType::AngularAcceleration: return Index2(3, 0);
//        default: CHECKandTHROWstring("GetOutputVariableDimension"); return Index2(0, 0);
//    }
//}
//
//inline OutputVariableUnit GetOutputVariableUnit(OutputVariableType outputVariableType)
//{
//    switch (outputVariableType)
//    {
//        case OutputVariableType::_None: return OutputVariableUnit::NoUnit;
//        case OutputVariableType::Position: return OutputVariableUnit::Length;
//        case OutputVariableType::Displacement: return OutputVariableUnit::Length;
//        case OutputVariableType::Velocity: return OutputVariableUnit::LengthPerTime;
//        case OutputVariableType::Acceleration: return OutputVariableUnit::LengthPerTimeSquared;
//        case OutputVariableType::RotationMatrix: return OutputVariableUnit::NoUnit;
//        case OutputVariableType::AngularVelocity: return OutputVariableUnit::RotationsPerTime;
//        case OutputVariableType::AngularAcceleration: return OutputVariableUnit::RotationsPerTimeSquared;
//        default: CHECKandTHROWstring("GetOutputVariableUnit"); return OutputVariableUnit::NoUnit;
//    }
//};

//the enums shared with Python and their string functions: generated from definitions/enumTypes.py
#include "Autogenerated/EnumTypes.h"

//! simple function to check for multiple configuration types
inline bool IsValidConfiguration(ConfigurationType configuration)
{
	if (configuration == ConfigurationType::Current ||
		configuration == ConfigurationType::Initial ||
		configuration == ConfigurationType::Reference ||
		configuration == ConfigurationType::StartOfStep || //2023-01-12: added; CSystem now initializes in Assemble()
		configuration == ConfigurationType::Visualization) {
		return true;
	}
	else { return false; }
}

//! simple function to check for multiple configuration types
inline bool IsValidConfigurationButNotReference(ConfigurationType configuration)
{
	if (configuration == ConfigurationType::Current || 
		configuration == ConfigurationType::Initial ||
		//configuration == ConfigurationType::Reference || //2023-01-12: was wrong here, removed
		configuration == ConfigurationType::StartOfStep || //2023-01-12: added; CSystem now initializes in Assemble()
		configuration == ConfigurationType::Visualization) {
		return true;
	}
	else { return false; }
}

const int index2ItemIDindexShift = 3;	//3 bits for item Type (5 values + 0)
const int type2ItemIDindexShift = 4;	//4 bits for mbs number (16 values)
const int itemIDinvalidValue = -1;		//ID that has no underlying mbs item
const int itemIDstaticObject = -2;			//invalid ID that represents static object (in MODELVIEW coordinates, always on top)
const int itemIDstaticObjectShaded = -3;	//invalid ID that represents static object (in MODELVIEW coordinates, always on top)
const int itemIDstaticObjectWithoutZoff = -4;	//invalid ID that represents static object (in MODELVIEW coordinates, with original depth)

//! conversion of mbsNumber (m), type (t) and index (i) bits for 32 bits: [iiiiiiii iiiiiiii iiiiiiii itttmmmm]
inline Index Index2ItemID(Index index, ItemType type, Index mbsNumber)
{
	if (mbsNumber == -1) { return itemIDinvalidValue; }
	return (index << (index2ItemIDindexShift + type2ItemIDindexShift)) +
		((Index)type << type2ItemIDindexShift) + mbsNumber;
}

//! inverse operation of Index2ItemID
inline void ItemID2IndexType(Index itemID, Index& index, ItemType& type, Index& mbsNumber)
{
	if (itemID != itemIDinvalidValue)
	{
		mbsNumber = (itemID & ((1 << type2ItemIDindexShift) - 1)); //logical and with mask for first 4 bits
		type = (ItemType)((itemID >> type2ItemIDindexShift) & ((1 << index2ItemIDindexShift) - 1)); //logical and with mask for  bits 5-7
		index = itemID >> (index2ItemIDindexShift+ type2ItemIDindexShift);
	}
	else
	{
		type = ItemType::_None;
		index = -1;
		mbsNumber = 0;
	}
}

//! define enum for Joint types, used in KinematicTree
namespace Joint { 
	inline bool IsRevolute(Type var) { return var >= RevoluteX && var <= RevoluteZ; }
	inline bool IsPrismatic(Type var) { return var >= PrismaticX && var <= PrismaticZ; }

	////this array maps joint types to joint axis:
	//Index map2AxisNumber[] = { -1, 0, 1,-1, 2,
	//						   -1,-1,-1, 0,-1,-1,-1,-1,-1,-1,-1, 1,
	//						   -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, 2 };
	//this array maps joint types to joint axis:
#ifndef __EXUDYN__APPLE__
	inline static const 
#else
	static const 
#endif
	Index map2AxisNumber[] = { -1, 0, 1, 2, 0, 1, 2, 3 };

	inline Index AxisNumber(Type var) 
	{
		CHECKandTHROW(var <= 6 and var > 0, "Joint::AxisNumber: joint out of range");
		return map2AxisNumber[(Index)var];
	}

	inline bool IsValid(Type var) { return var >= RevoluteX && var <= PrismaticZ && map2AxisNumber[(Index)var]!=-1; }

}

typedef std::vector<Joint::Type> JointTypeList;


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++                            SOLVER TYPES                                      ++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



#endif
