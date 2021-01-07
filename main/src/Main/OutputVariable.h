/** ***********************************************************************************************
* @class	    OutputVariable
* @brief		
* @details		Details:
 				- ...
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
	enum Type {
		_None = 0, //marks that no type is used
		//bits to determine the item which is acted on (not relevant for connector, load, OutputVariable):
		//1+4==Body, 2==Node, 4==Object, 2+4=Node+Object
		Body = 1 << 0,						//!< 1==Body, 0==other (must be also object!!!)
		Node = 1 << 1,						//!< 2==Node, 0=other
		Object = 1 << 2,					//!< 4==Object, 0=other
		SuperElement = 1 << 3,				//!< Marker only applicable to SuperElements; accesses nodes (virtual nodes) of SuperElements
		//bits to determine the kind of quantity is involved (relevant for: connector, load, OutputVariable):
		//keep this list SYNCHRONIZED with AccessFunctionType:
		Position = 1 << 4,					//!< can measure position, apply Distance constraint
		Orientation = 1 << 5,				//!< can measure rotation, apply general rigid body constraint (if Position is set)
		Coordinate = 1 << 6,				//!< access any coordinate (always available)
		//bits for geometrical dimension: force applied to volume, displacement of volume (center of mass ...)
		//BodyPoint = 1 << xx, //default is always point; not necessary for Body+Position!
		BodyLine = 1 << 7,					//!< represents a line load (vector load applied to line)
		BodySurface = 1 << 8,				//!< represents a surface load / connector (e.g. for revolute joint with FE-mesh)
		BodyVolume = 1 << 9,				//!< volume load ==> usually gravity
		BodyMass = 1 << 10,					//!< volume load ==> usually gravity
		BodySurfaceNormal = 1 << 11,		//!< for surface pressure (uses scalar load)
		//++++for SuperElementMarkers:
		MultiNodal = 1 << 12,				//!< multinodal marker uses a weighting matrix for transformation of node values to marker value (e.g., list of positions averaged to one position)
		ReducedCoordinates = 1 << 13,		//!< multinodal marker uses a weighting matrix for transformation of node values to marker value (e.g., list of positions averaged to one position)

		//Rotv1v2v3 = 1 << 1xx,				//!< for special joints that need a attached triad; in fact, a marker of orientation type must also provide Rotv1v2v3

		EndOfEnumList = 1 << 14				//KEEP THIS AS THE (2^i) MAXIMUM OF THE ENUM LIST!!!
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
		if (var & Position) { t += "Position"; }
		if (var & Orientation) { t += "Orientation"; }
		if (var & Coordinate) { t += "Coordinate"; }
		if (var & BodyLine) { t += "Line"; } //'Body' already added via (var & Body)
		if (var & BodySurface) { t += "Surface"; } //'Body' already added via (var & Body)
		if (var & BodyVolume) { t += "Volume"; } //'Body' already added via (var & Body)
		if (var & BodyMass) { t += "Mass"; } //'Body' already added via (var & Body)
		if (var & BodySurfaceNormal) { t += "SurfaceNormal"; } //'Body' already added via (var & Body)
		
		if (var & MultiNodal) { t += "MultiNodal"; }
		if (var & ReducedCoordinates) { t += "ReducedCoordinates"; }
		if (t.length() == 0) { CHECKandTHROWstring("Marker::GetTypeString(...) called for invalid type!"); }

		return t;
	}

}

enum class AccessFunctionType { //determines which connectors/forces can be applied to object; underscores mark the derivative w.r.t. q
	_None = 0, //marks that no type is used
	//TranslationalVelocity_qt = (1 << 0), //for application of forces, position constraints
	//AngularVelocity_qt = (1 << 1), //for application of torques, rotational constraints
	//DisplacementVolumeIntegral_q = (1 << 2), //for distributed (body) loads

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
	//Rotv1v2v3_q = (Index)Marker::Rotv1v2v3,						//for joints, e.g., prismatic or rigid body; in fact, a marker of type orientation must also provide Rotv1v2v3
	//NOT VALID: EndOfEnumList = (1 << 3) //KEEP THIS AS THE (2^i) MAXIMUM OF THE ENUM LIST!!!
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
	Marker = 1 << 4, //!< NOT implemented yet, needs OutputVariableType in markers!
	Load   = 1 << 5, //!< measure prescribed loads, in order to track, e.g., user defined loads or controlled loads
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
	case SensorType::Marker: return "Marker";
	case SensorType::Load: return "Load";
	default: SysError("GetSensorTypeString: invalid variable type");  return "Invalid";
	}
}


//! OutputVariable used for output data in objects, nodes, loads, ...
enum class OutputVariableType {
	//all cases are independent of 2D/3D, which is known by the object itself; TYPES CAN BE COMBINED (==> used for available types in element)
	_None = 0, //marks that no type is used
	Distance = 1 << 0,				//!< distance, e.g. of joint or spring-damper
	Position = 1 << 1,				//!< position vector, e.g. of node or body center of mass
	Displacement = 1 << 2,			//!< displacement vector, e.g. of node or body center of mass
	Velocity = 1 << 3,				//!< velocity vector, e.g. of node or body center of mass
	Acceleration = 1 << 4,			//!< acceleration vector, e.g. of node or body center of mass
	RotationMatrix = 1 << 5,		//!< rotation matrix, e.g. rigid body
	AngularVelocity = 1 << 6,		//!< angular velocity vector, e.g. rigid body; scalar quantity in 2D-elements
	AngularVelocityLocal = 1 << 7,	//!< angular velocity vector in local (body-fixed) coordinates
	AngularAcceleration = 1 << 8,	//!< angular acceleration vector, e.g. rigid body; scalar quantity in 2D-elements
	Rotation = 1 << 9,				//!< angle, e.g. joint angle; rotation parameters; scalar rotation in 2D rigid body
	Coordinates = 1 << 10,			//!< single object or node coordinate(s) as output
	Coordinates_t = 1 << 11,		//!< single object or node velocity coordinate(s) as output
	Coordinates_tt = 1 << 12,		//!< single object or node velocity coordinate(s) as output
	SlidingCoordinate = 1 << 13,	//!< scalar coordinate in sliding joint
	Director1 = 1 << 14,			//!< direction or (axial) slope vector 1 (in 2D-elements)
	Director2 = 1 << 15,			//!< direction or (normal1) slope vector 2 (in 2D-elements or shells)
	Director3 = 1 << 16,			//!< direction or (normal2) slope vector 3 (in 3D-elements or shells)
	Force = 1 << 17,				//!< force e.g. in connector/constraint or section force in beam
	Torque = 1 << 18,				//!< torque e.g. in connector/constraint or section moment/torque in beam
	Strain = 1 << 19,				//!< strain components (e.g. axial strain and shear strain in beam, or engineering strain components in finite element)
	Stress = 1 << 20,				//!< stress components (e.g. axial stress and shear stress in beam, or engineering stress components in finite element)
	Curvature = 1 << 21,			//!< curvature (components) in beam or shell
	DisplacementLocal = 1 << 22,	//!< local displacement vector (used e.g. in joints)
	VelocityLocal = 1 << 23,		//!< local velocity vector (used e.g. in joints)
	ForceLocal = 1 << 24,			//!< local force e.g. in connector/constraint or section force in beam
	TorqueLocal = 1 << 25,			//!< local torque e.g. in connector/constraint or section moment/torque in beam
	ConstraintEquation = 1 << 26,	//!< evaluates constraint equation (=current deviation or drift of constraint equation)
	//keep this list synchronized with function GetOutputVariableTypeString(...) !!!

    //SecondPiolaKirchoffStress = (1 << 7), GreenStrain = (1 << 8),
    //BeamStrain = (1 << 9), BeamCurvature = (1 << 10), //are both 3D-vectors containing axial and transverse components
    //FramePosition = (1 << 11), FrameOrientation = (1 << 12), //position and orientation of the reference point
    EndOfEnumList = 1 << 27 //KEEP THIS AS THE (2^i) MAXIMUM OF THE ENUM LIST!!!
};

//! OutputVariable string conversion
inline const char* GetOutputVariableTypeString(OutputVariableType var)
{
	switch (var)
	{
	case OutputVariableType::_None: return "_None";
	case OutputVariableType::Distance: return "Distance";
	case OutputVariableType::Position: return "Position";
	case OutputVariableType::Displacement: return "Displacement";
	case OutputVariableType::Velocity: return "Velocity";
	case OutputVariableType::Acceleration: return "Acceleration";
	case OutputVariableType::RotationMatrix: return "RotationMatrix";
	case OutputVariableType::AngularVelocity: return "AngularVelocity";
	case OutputVariableType::AngularVelocityLocal: return "AngularVelocityLocal";
	case OutputVariableType::AngularAcceleration: return "AngularAcceleration";
	//case OutputVariableType::ScalarPosition: return "ScalarPosition"; //use position instead!
	//case OutputVariableType::ScalarDisplacement: return "ScalarDisplacement";
	//case OutputVariableType::ScalarVelocity: return "ScalarVelocity";
	//case OutputVariableType::ScalarAcceleration: return "ScalarAcceleration";
	case OutputVariableType::Rotation: return "Rotation";
	//case OutputVariableType::ScalarAngularVelocity: return "ScalarAngularVelocity";
	//case OutputVariableType::ScalarAngularAcceleration: return "ScalarAngularAcceleration";
	case OutputVariableType::Coordinates: return "Coordinates";
	case OutputVariableType::Coordinates_t: return "Coordinates_t";
	case OutputVariableType::Coordinates_tt: return "Coordinates_tt";
	case OutputVariableType::SlidingCoordinate: return "SlidingCoordinate";
	case OutputVariableType::Director1: return "Director1";
	case OutputVariableType::Director2: return "Director2";
	case OutputVariableType::Director3: return "Director3";
	case OutputVariableType::Force: return "Force";
	case OutputVariableType::Torque: return "Torque";
	case OutputVariableType::Strain: return "Strain";
	case OutputVariableType::Stress: return "Stress";
	case OutputVariableType::Curvature: return "Curvature";
	case OutputVariableType::DisplacementLocal: return "DisplacementLocal";
	case OutputVariableType::VelocityLocal: return "VelocityLocal";
	case OutputVariableType::ForceLocal: return "ForceLocal";
	case OutputVariableType::TorqueLocal: return "TorqueLocal";
	case OutputVariableType::ConstraintEquation: return "ConstraintEquation";
	case OutputVariableType::EndOfEnumList: return "EndOfEnumList";
	default: SysError("GetOutputVariableTypeString: invalid variable type");  return "Invalid";
	}
}

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

//currently not used inside C++ code!
enum class DynamicSolverType {
	//_None = 0, //marks that no configuration is used
	GeneralizedAlpha = 1,	//an implicit solver for index 3 problems; allows to set variables also for Newmark and trapezoidal implicit index 2 solvers
	TrapezoidalIndex2 = 2,	//an implicit solver for index 3 problems with index2 reduction; uses generalized alpha solver with settings for Newmark with index2 reduction
	ExplicitEuler = 3,		//an explicit first order solver for systems without constraints
	RK45 = 4				//an explicit Runge Kutta solver of 4th order for systems without constraints; includes adaptive step selection
};

//key codes as defined in GLFW, used for Python keyPressUserFunction
enum class KeyCode {
	_None = 0,			//marks that no configuration is used
	SPACE = 32,			//see GLFW and Python interface};
	ESCAPE = 256,		//see GLFW and Python interface};
	ENTER = 257,		//see GLFW and Python interface};
	TAB = 258,			//see GLFW and Python interface};
	BACKSPACE = 259,	//see GLFW and Python interface};
	RIGHT = 262,		//see GLFW and Python interface};
	LEFT = 263,			//see GLFW and Python interface};
	DOWN = 264,			//see GLFW and Python interface};
	UP = 265,			//see GLFW and Python interface};
	F1 = 291,	//see GLFW and Python interface};
	F2 = 292,	//see GLFW and Python interface};
	F3 = 293,	//see GLFW and Python interface};
	F4 = 294,	//see GLFW and Python interface};
	F5 = 295,	//see GLFW and Python interface};
	F6 = 296,	//see GLFW and Python interface};
	F7 = 297,	//see GLFW and Python interface};
	F8 = 298,	//see GLFW and Python interface};
	F9 = 299,	//see GLFW and Python interface};
	F10 = 300	//see GLFW and Python interface};
};

enum class ConfigurationType {
	_None = 0, //marks that no configuration is used
	Initial = 1,
	Current = 2,
	Reference = 3,
	StartOfStep = 4,
	Visualization = 5,
	EndOfEnumList = 6 //KEEP THIS AS THE (consecutive) MAXIMUM OF THE ENUM LIST!!!
};

//! simple function to check for multiple configuration types
inline bool IsConfigurationInitialCurrentReferenceVisualization(ConfigurationType configuration)
{
	if (configuration == ConfigurationType::Initial ||
		configuration == ConfigurationType::Current ||
		configuration == ConfigurationType::Reference ||
		configuration == ConfigurationType::Visualization) {
		return true;
	}
	else { return false; }
}

//! simple function to check for multiple configuration types
inline bool IsConfigurationInitialCurrentVisualization(ConfigurationType configuration)
{
	if (configuration == ConfigurationType::Initial ||
		configuration == ConfigurationType::Current ||
		configuration == ConfigurationType::Reference ||
		configuration == ConfigurationType::Visualization) {
		return true;
	}
	else { return false; }
}


//! helper function to transform loadType and markerType to (necessary) AccessFunctionType
//! @todo move GetAccessFunctionType to CMarker / derived Marker classes !
//  now this can be translated mostly automatically
inline AccessFunctionType GetAccessFunctionType(LoadType loadType, Marker::Type markerType)
{
	switch (markerType)
    {
	case Marker::Position: 
        if (loadType == LoadType::Force) {
            return AccessFunctionType::TranslationalVelocity_qt;
        } else if (loadType == LoadType::Torque) {
            return AccessFunctionType::AngularVelocity_qt;
        }
        else {
            CHECKandTHROWstring("GetAccessFunctionType:  Marker::BodyPosition"); return AccessFunctionType::_None;
        }
    case Marker::BodyMass:
        if (loadType == LoadType::ForcePerVolume) {
            return AccessFunctionType::DisplacementVolumeIntegral_q;
        }
        else {
            CHECKandTHROWstring("GetAccessFunctionType:  Marker::ForcePerVolume"); return AccessFunctionType::_None;
        }

    default: CHECKandTHROWstring("GetAccessFunctionType"); return AccessFunctionType::_None;
    }

};

//! helper function to transform loadType and markerType to (necessary) AccessFunctionType
//! @todo move GetAccessFunctionType to CMarker / derived Marker classes !
//  now this can be translated mostly automatically
//inline AccessFunctionType GetAccessFunctionType(LoadType loadType, Marker::Type markerType)
//{
//	switch (markerType)
//	{
//	case Marker::Position:
//		if (loadType == LoadType::Force) {
//			return AccessFunctionType::TranslationalVelocity_qt;
//		}
//		else {
//			CHECKandTHROWstring("GetAccessFunctionType:  Marker::Position"); return AccessFunctionType::_None;
//		}
//
//	case Marker::Orientation:
//		if (loadType == LoadType::Torque) {
//			return AccessFunctionType::AngularVelocity_qt;
//		}
//		else {
//			CHECKandTHROWstring("GetAccessFunctionType:  Marker::Position"); return AccessFunctionType::_None;
//		}
//
//	case Marker::BodyMass:
//		if (loadType == LoadType::ForcePerMass) {
//			return AccessFunctionType::DisplacementMassIntegral_q;
//		}
//		else {
//			CHECKandTHROWstring("GetAccessFunctionType:  Marker::ForcePerMass"); return AccessFunctionType::_None;
//		}
//
//	default: CHECKandTHROWstring("GetAccessFunctionType illegal"); return AccessFunctionType::_None;
//	}
//};

//! helper function to determine AccessFunctionType and OutputVariableType requested by a certain markerType
//inline void GetRequestedAccessAndOutputType(Marker::Type markerType, AccessFunctionType& accessFunctionType, OutputVariableType& outputVariableType)
//{
//	switch (markerType)
//	{
//	case Marker::Position:
//	default: CHECKandTHROWstring("GetRequestedAccessAndOutputType"); 
//	}
//
//};


//! enum to determine how to set up the system matrix 
enum class LinearSolverType {
	_None = 0,			//marks that no type is used
	EXUdense = 1,		//use internal dense matrix (e.g. matrix inverse for factorization)
	EigenSparse = 2		//use Eigen::SparseMatrix
};

//! ostream operator for printing of enum class
inline std::ostream& operator<<(std::ostream& os, LinearSolverType value)
{
	switch (value)
	{
	case LinearSolverType::_None:			return os << "_None"; break;
	case LinearSolverType::EXUdense:			return os << "EXUdense"; break;
	case LinearSolverType::EigenSparse:		return os << "EigenSparse"; break;
	default: 		return os << "LinearSolverType::invalid";
	}
}

#endif
