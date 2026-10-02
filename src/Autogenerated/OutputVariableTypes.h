/** ********************************************************************************************
* @brief        AUTO GENERATED FILE - DO NOT EDIT
*
* @details      OutputVariableType, generated from definitions/outputVariableTypes.py
*               by tools/generators/outputVariableEmitter.py. Add an output variable
*               there, with the next free bit; never reuse a bit that was handed out.
*
* @author       Gerstmayr Johannes
* @copyright    This file is part of Exudyn. Exudyn is free software: see LICENSE.txt
*
************************************************************************************* */

#ifndef OUTPUTVARIABLETYPES__H
#define OUTPUTVARIABLETYPES__H

//! OutputVariable used for output data in objects, nodes, loads, ...
//! the type is a 64 bit mask: types can be combined, e.g. for the variables an item
//! provides. All cases are independent of 2D/3D, which the item itself knows.
enum class OutputVariableType : Index64 {
    _None                     = 0ull, //!< marks that no type is used
    Distance                  = 1ull << 0, //!< e.g., measure distance in spring damper connector
    Position                  = 1ull << 1, //!< measure 3D position, e.g., of node or body
    Displacement              = 1ull << 2, //!< measure displacement; usually difference between current position and reference position
    DisplacementLocal         = 1ull << 3, //!< measure local displacement, e.g., in local joint coordinates
    Velocity                  = 1ull << 4, //!< measure (translational) velocity of node or object
    VelocityLocal             = 1ull << 5, //!< measure local (translational) velocity, e.g., in local body or joint coordinates
    Acceleration              = 1ull << 6, //!< measure (translational) acceleration of node or object
    AccelerationLocal         = 1ull << 7, //!< measure (translational) acceleration of node or object in local coordinates
    RotationMatrix            = 1ull << 8, //!< measure rotation matrix of rigid body node or object
    Rotation                  = 1ull << 13, //!< measure, e.g., scalar rotation of 2D body, Euler angles of a 3D object or rotation within a joint
    AngularVelocity           = 1ull << 9, //!< measure angular velocity of node or object
    AngularVelocityLocal      = 1ull << 10, //!< measure local (body-fixed) angular velocity of node or object
    AngularAcceleration       = 1ull << 11, //!< measure angular acceleration of node or object
    AngularAccelerationLocal  = 1ull << 12, //!< measure angular acceleration of node or object in local coordinates
    CoordinatesTotal          = 1ull << 14, //!< measure the total coordinates (including reference configuration) of a node or object; otherwise the same as Coordinates
    Coordinates               = 1ull << 15, //!< measure the coordinates of a node or object; coordinates just contain displacements, but not the reference (position or rotation) values - see also definition of respective nodes or objects
    Coordinates_t             = 1ull << 16, //!< measure the time derivative of coordinates (= velocity coordinates) of a node or object
    Coordinates_tt            = 1ull << 17, //!< measure the second time derivative of coordinates (= acceleration coordinates) of a node or object
    SlidingCoordinate         = 1ull << 18, //!< measure sliding coordinate in sliding joint
    Director1                 = 1ull << 19, //!< measure a director (e.g., of a rigid body frame), or a slope vector in local 1 or x-direction
    Director2                 = 1ull << 20, //!< measure a director (e.g., of a rigid body frame), or a slope vector in local 2 or y-direction
    Director3                 = 1ull << 21, //!< measure a director (e.g., of a rigid body frame), or a slope vector in local 3 or z-direction
    Force                     = 1ull << 22, //!< measure global force, e.g., in joint or beam (resultant force), or generalized forces; see description of according object
    ForceLocal                = 1ull << 23, //!< measure local force, e.g., in joint or beam (resultant force)
    Torque                    = 1ull << 24, //!< measure torque, e.g., in joint or beam (resultant couple/moment)
    TorqueLocal               = 1ull << 25, //!< measure local torque, e.g., in joint or beam (resultant couple/moment)
    StrainLocal               = 1ull << 28, //!< measure local strain, e.g., axial strain in cross section frame of beam or Green-Lagrange strain
    StressLocal               = 1ull << 29, //!< measure local stress, e.g., axial stress in cross section frame of beam or Second Piola-Kirchoff stress; choosing component==-1 will result in the computation of the Mises stress
    CurvatureLocal            = 1ull << 30, //!< measure local curvature; may be scalar or vectorial: twist and curvature of beam in cross section frame
    ConstraintEquation        = 1ull << 31, //!< evaluates constraint equation (=current deviation or drift of constraint equation)
    KineticEnergy             = 1ull << 32, //!< measure kinetic energy of a body, position independent
    PotentialEnergy           = 1ull << 33, //!< measure potential (=elastic) energy of a body or connector, position independent
    HomogeneousTransformation = 1ull << 34, //!< measure the homogeneous transformation of a node, body point or marker: its rotation matrix A and position p as the 4x4 matrix [A p; 0 1]; the Get...Output functions return it as exu.HT, a sensor stores its 16 components row by row; every item with Position and RotationMatrix provides it

    //bits below are ALLOCATED and must not be reused:
    //Strain                  = 1ull << 26, //!< considered for finite elements or fluids; never implemented
    //Stress                  = 1ull << 27, //!< considered for finite elements or fluids; never implemented
};

//! return whether given OutputVariableType can be evaluated for reference configuration
inline bool IsOutputVariableTypeForReferenceConfiguration(OutputVariableType var)
{
    const Index64 refTypes =
        (Index64)OutputVariableType::Distance +
        (Index64)OutputVariableType::Position +
        (Index64)OutputVariableType::Displacement +
        (Index64)OutputVariableType::DisplacementLocal +
        (Index64)OutputVariableType::RotationMatrix +
        (Index64)OutputVariableType::Rotation +
        (Index64)OutputVariableType::Coordinates +
        (Index64)OutputVariableType::SlidingCoordinate +
        (Index64)OutputVariableType::Director1 +
        (Index64)OutputVariableType::Director2 +
        (Index64)OutputVariableType::Director3 +
        (Index64)OutputVariableType::ConstraintEquation +
        (Index64)OutputVariableType::HomogeneousTransformation;

    if (EXUstd::IsOfTypeAndNotNone(refTypes, (Index64)var)) { return true; }
    return false;
}

//! OutputVariable string conversion
inline const char* GetOutputVariableTypeString(OutputVariableType var)
{
    switch (var)
    {
    case OutputVariableType::_None: return "_None";
    case OutputVariableType::Distance: return "Distance";
    case OutputVariableType::Position: return "Position";
    case OutputVariableType::Displacement: return "Displacement";
    case OutputVariableType::DisplacementLocal: return "DisplacementLocal";
    case OutputVariableType::Velocity: return "Velocity";
    case OutputVariableType::VelocityLocal: return "VelocityLocal";
    case OutputVariableType::Acceleration: return "Acceleration";
    case OutputVariableType::AccelerationLocal: return "AccelerationLocal";
    case OutputVariableType::RotationMatrix: return "RotationMatrix";
    case OutputVariableType::Rotation: return "Rotation";
    case OutputVariableType::AngularVelocity: return "AngularVelocity";
    case OutputVariableType::AngularVelocityLocal: return "AngularVelocityLocal";
    case OutputVariableType::AngularAcceleration: return "AngularAcceleration";
    case OutputVariableType::AngularAccelerationLocal: return "AngularAccelerationLocal";
    case OutputVariableType::CoordinatesTotal: return "CoordinatesTotal";
    case OutputVariableType::Coordinates: return "Coordinates";
    case OutputVariableType::Coordinates_t: return "Coordinates_t";
    case OutputVariableType::Coordinates_tt: return "Coordinates_tt";
    case OutputVariableType::SlidingCoordinate: return "SlidingCoordinate";
    case OutputVariableType::Director1: return "Director1";
    case OutputVariableType::Director2: return "Director2";
    case OutputVariableType::Director3: return "Director3";
    case OutputVariableType::Force: return "Force";
    case OutputVariableType::ForceLocal: return "ForceLocal";
    case OutputVariableType::Torque: return "Torque";
    case OutputVariableType::TorqueLocal: return "TorqueLocal";
    case OutputVariableType::StrainLocal: return "StrainLocal";
    case OutputVariableType::StressLocal: return "StressLocal";
    case OutputVariableType::CurvatureLocal: return "CurvatureLocal";
    case OutputVariableType::ConstraintEquation: return "ConstraintEquation";
    case OutputVariableType::KineticEnergy: return "KineticEnergy";
    case OutputVariableType::PotentialEnergy: return "PotentialEnergy";
    case OutputVariableType::HomogeneousTransformation: return "HomogeneousTransformation";
    default: SysError("GetOutputVariableTypeString: invalid variable type");
        return "Invalid";
    }
}

#endif //OUTPUTVARIABLETYPES__H
