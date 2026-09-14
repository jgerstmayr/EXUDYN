/** ********************************************************************************************
* @brief        AUTO GENERATED FILE - DO NOT EDIT
*
* @details      The enumeration types shared by C++ and Python, and their string functions,
*               generated from definitions/enumTypes.py by tools/generators/enumEmitter.py.
*               Namespaces (Node, Joint, Contact) are reopened by the hand-written headers
*               for their further helper functions.
*
* @author       Gerstmayr Johannes
* @copyright    This file is part of Exudyn. Exudyn is free software: see LICENSE.txt
*
************************************************************************************* */

#ifndef ENUMTYPES__H
#define ENUMTYPES__H

#include <ostream>
#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

//! EndOfEnumList must remain the (consecutive) maximum of the list
enum class ConfigurationType {
    _None = 0,         //!< no configuration; usually not valid, but may be used, e.g., if no configurationType is required
    Initial = 1,       //!< initial configuration prior to static or dynamic solver; is computed during mbs.Assemble() or AssembleInitializeSystemCoordinates()
    Current = 2,       //!< current configuration during and at the end of the computation of a step (static or dynamic)
    Reference = 3,     //!< configuration used to define deformable bodies (reference configuration for finite elements) or joints (configuration for which some joints are defined)
    StartOfStep = 4,   //!< during computation, this refers to the solution at the start of the step = end of last step, to which the solver falls back if convergence fails
    Visualization = 5, //!< this is a state completely de-coupled from computation, used for visualization
    EndOfEnumList = 6  //!< this marks the end of the list, usually not important to the user
};

//! used e.g. for visualization; adapt index2ItemIDindexShift if values are added
enum class ItemType {
    _None = 0,  //!< item has no type
    Node = 1,   //!< item or index is of type Node
    Object = 2, //!< item or index is of type Object
    Marker = 3, //!< item or index is of type Marker
    Load = 4,   //!< item or index is of type Load
    Sensor = 5  //!< item or index is of type Sensor
};

//! ostream operator for printing of ItemType
inline std::ostream& operator<<(std::ostream& os, ItemType value)
{
    switch (value)
    {
    case ItemType::_None: return os << "_None";
    case ItemType::Node: return os << "Node";
    case ItemType::Object: return os << "Object";
    case ItemType::Marker: return os << "Marker";
    case ItemType::Load: return os << "Load";
    case ItemType::Sensor: return os << "Sensor";
    default: return os << "ItemType::invalid";
    }
}

enum class DynamicSolverType {
    GeneralizedAlpha = 1,  //!< an implicit solver for index 3 problems; intended to be used for solving directly the index 3 constraints using the spectralRadius sufficiently small (usually 0.5 .. 1)
    TrapezoidalIndex2 = 2, //!< an implicit solver for index 3 problems with index2 reduction; uses generalized alpha solver with settings for Newmark with index2 reduction
    ExplicitEuler = 3,     //!< an explicit 1st order solver (generally not compatible with constraints)
    ExplicitMidpoint = 4,  //!< an explicit 2nd order solver (generally not compatible with constraints)
    RK33 = 5,              //!< an explicit 3 stage 3rd order Runge-Kutta method, aka "Heun third order"; (generally not compatible with constraints)
    RK44 = 6,              //!< an explicit 4 stage 4th order Runge-Kutta method, aka "classical Runge Kutta" (generally not compatible with constraints), compatible with Lie group integration and elimination of CoordinateConstraints
    RK67 = 7,              //!< an explicit 7 stage 6th order Runge-Kutta method, see 'On Runge-Kutta Processes of High Order', J. C. Butcher, J. Austr Math Soc 4, (1964); can be used for very accurate (reference) solutions, but without step size control!
    ODE23 = 8,             //!< an explicit Runge Kutta method with automatic step size selection with 3rd order of accuracy and 2nd order error estimation, see Bogacki and Shampine, 1989; also known as ODE23 in MATLAB
    DOPRI5 = 9,            //!< an explicit Runge Kutta method with automatic step size selection with 5th order of accuracy and 4th order error estimation, see  Dormand and Prince, 'A Family of Embedded Runge-Kutta Formulae.', J. Comp. Appl. Math. 6, 1980
    DVERK6 = 10,           //!< [NOT IMPLEMENTED YET] an explicit Runge Kutta solver of 6th order with 5th order error estimation; includes adaptive step selection
    VelocityVerlet = 11    //!< [TEST phase] a special explicit time integration scheme, the 'velocity Verlet' method (similar to leap frog method), with second order accuracy for conservative second order differential equations, often used for particle dynamics and contact; implementation uses Explicit Euler for ODE1 equations
};

//! ostream operator for printing of DynamicSolverType
inline std::ostream& operator<<(std::ostream& os, DynamicSolverType value)
{
    switch (value)
    {
    case DynamicSolverType::GeneralizedAlpha: return os << "GeneralizedAlpha";
    case DynamicSolverType::TrapezoidalIndex2: return os << "TrapezoidalIndex2";
    case DynamicSolverType::ExplicitEuler: return os << "ExplicitEuler";
    case DynamicSolverType::ExplicitMidpoint: return os << "ExplicitMidpoint";
    case DynamicSolverType::RK33: return os << "RK33";
    case DynamicSolverType::RK44: return os << "RK44";
    case DynamicSolverType::RK67: return os << "RK67";
    case DynamicSolverType::ODE23: return os << "ODE23";
    case DynamicSolverType::DOPRI5: return os << "DOPRI5";
    case DynamicSolverType::DVERK6: return os << "DVERK6";
    case DynamicSolverType::VelocityVerlet: return os << "VelocityVerlet";
    default: return os << "DynamicSolverType::invalid";
    }
}

enum class CrossSectionType {
    _None = 0,         //!< no type is used
    Polygon = 1 << 0,  //!< cross section profile defined by polygon
    Circular = 1 << 1  //!< cross section is circle or elliptic
};

//! key codes as defined in GLFW, used for Python keyPressUserFunction
enum class KeyCode {
    _None = 0,       //!< no key
    SPACE = 32,      //!< space key
    ESCAPE = 256,    //!< escape key
    ENTER = 257,     //!< enter (return) key
    TAB = 258,       //!< 
    BACKSPACE = 259, //!< 
    RIGHT = 262,     //!< cursor right
    LEFT = 263,      //!< cursor left
    DOWN = 264,      //!< cursor down
    UP = 265,        //!< cursor up
    F1 = 291,        //!< function key F1
    F2 = 292,        //!< function key F2
    F3 = 293,        //!< function key F3
    F4 = 294,        //!< function key F4
    F5 = 295,        //!< function key F5
    F6 = 296,        //!< function key F6
    F7 = 297,        //!< function key F7
    F8 = 298,        //!< function key F8
    F9 = 299,        //!< function key F9
    F10 = 300        //!< function key F10
};

//! determines how to set up the system matrix
enum class LinearSolverType {
    _None = 0,                     //!< no value; used, e.g., if no solver is selected
    EXUdense = 1 << 0,             //!< use dense matrices and according solvers for densly populated matrices (usually the CPU time grows cubically with the number of unknowns)
    EigenSparse = 1 << 1,          //!< use sparse matrices and according solvers; additional overhead for very small multibody systems; specifically, memory allocation is performed during a factorization process
    EigenSparseSymmetric = 1 << 2, //!< use sparse matrices and according solvers; NOTE: this is the symmetric mode, which assumes symmetric system matrices; this is EXPERIMENTAL and should only be used of user knows that the system matrices are (nearly) symmetric; does not work with scaled GeneralizedAlpha matrices; does not work with constraints, as it must be symmetric positive definite
    EigenDense = 1 << 3,           //!< use Eigen's LU factorization with partial pivoting (faster than EXUdense) or full pivot (if linearSolverSettings.ignoreSingularJacobian=True; is much slower, but can resolve overdetermined and underdetermined problems!)
    Dense = (1 << 0) + (1 << 3)    //!< any dense solver
};

//! ostream operator for printing of LinearSolverType
inline std::ostream& operator<<(std::ostream& os, LinearSolverType value)
{
    switch (value)
    {
    case LinearSolverType::_None: return os << "_None";
    case LinearSolverType::EXUdense: return os << "EXUdense";
    case LinearSolverType::EigenSparse: return os << "EigenSparse";
    case LinearSolverType::EigenSparseSymmetric: return os << "EigenSparseSymmetric";
    case LinearSolverType::EigenDense: return os << "EigenDense";
    case LinearSolverType::Dense: return os << "Dense";
    default: return os << "LinearSolverType::invalid";
    }
}

namespace Node {
    //! node types are used for integrity checks to verify that a node is suitable for an object; bit 11 was LieGroupWithDataCoordinates (never used)
    enum Type {
        _None = 0,                          //!< node has no type
        Ground = 1 << 0,                    //!< ground node
        Position2D = 1 << 1,                //!< 2D position node 
        Orientation2D = 1 << 2,             //!< node with 2D rotation
        Point2DSlope1 = 1 << 3,             //!< 2D node with 1 slope vector
        Position = 1 << 4,                  //!< 3D position node
        Orientation = 1 << 5,               //!< 3D orientation node
        RigidBody = 1 << 6,                 //!< node that can be used for rigid bodies
        RotationEulerParameters = 1 << 7,   //!< node with 3D orientations that are modelled with Euler parameters (unit quaternions)
        RotationRxyz = 1 << 8,              //!< node with 3D orientations that are modelled with Tait-Bryan angles
        RotationRotationVector = 1 << 9,    //!< node with 3D orientations that are modelled with the rotation vector
        LieGroupWithDirectUpdate = 1 << 10, //!< node to be solved with Lie group methods, without data coordinates
        GenericODE2 = 1 << 12,              //!< node with general ODE2 variables
        GenericODE1 = 1 << 13,              //!< node with general ODE1 variables
        GenericAE = 1 << 14,                //!< node with general algebraic variables
        GenericData = 1 << 15,              //!< node with general data variables
        PointSlope1 = 1 << 16,              //!< node with 1 slope vector
        PointSlope12 = 1 << 17,             //!< node with 2 slope vectors in x and y direction
        PointSlope23 = 1 << 18              //!< node with 2 slope vectors in y and z direction
    };

    //! transform type into string (e.g. for error messages); this is slow and cannot be used during computation!
    inline STDstring GetTypeString(Type var)
    {
        STDstring t; //empty string
        if (var == _None) { t = "_None/Undefined"; }
        if (var & Ground) { t += "Ground"; }
        if (var & Position2D) { t += "Position2D"; }
        if (var & Orientation2D) { t += "Orientation2D"; }
        if (var & Point2DSlope1) { t += "Point2DSlope1"; }
        if (var & Position) { t += "Position"; }
        if (var & Orientation) { t += "Orientation"; }
        if (var & RigidBody) { t += "RigidBody"; }
        if (var & RotationEulerParameters) { t += "RotationEulerParameters"; }
        if (var & RotationRxyz) { t += "RotationRxyz"; }
        if (var & RotationRotationVector) { t += "RotationRotationVector"; }
        if (var & LieGroupWithDirectUpdate) { t += "LieGroupWithDirectUpdate"; }
        if (var & GenericODE2) { t += "GenericODE2"; }
        if (var & GenericODE1) { t += "GenericODE1"; }
        if (var & GenericAE) { t += "GenericAE"; }
        if (var & GenericData) { t += "GenericData"; }
        if (var & PointSlope1) { t += "PointSlope1"; }
        if (var & PointSlope12) { t += "PointSlope12"; }
        if (var & PointSlope23) { t += "PointSlope23"; }
        if (t.length() == 0) { CHECKandTHROWstring("Node::GetTypeString(...) called for invalid type!"); }
        return t;
    }
} //namespace Node

namespace Joint {
    //! used for KinematicTree
    enum Type {
        _None = 0,      //!< node has no type
        RevoluteX = 1,  //!< revolute joint type with rotation around local X axis
        RevoluteY = 2,  //!< revolute joint type with rotation around local Y axis
        RevoluteZ = 3,  //!< revolute joint type with rotation around local Z axis
        PrismaticX = 4, //!< prismatic joint type with translation along local X axis
        PrismaticY = 5, //!< prismatic joint type with translation along local Y axis
        PrismaticZ = 6  //!< prismatic joint type with translation along local Z axis
    };

    //! transform type into string (e.g. for error messages); this is slow and cannot be used during computation!
    inline STDstring GetTypeString(Type var)
    {
        switch (var)
        {
        case _None: return "_None/Undefined";
        case RevoluteX: return "RevoluteX";
        case RevoluteY: return "RevoluteY";
        case RevoluteZ: return "RevoluteZ";
        case PrismaticX: return "PrismaticX";
        case PrismaticY: return "PrismaticY";
        case PrismaticZ: return "PrismaticZ";
        default: CHECKandTHROWstring("Joint::GetTypeString(...) called for invalid type!"); return "";
        }
    }
} //namespace Joint

namespace Contact {
    //! type of attachment and type of contact element in GeneralContact
    enum Type {
        _None = 0,                  //!< no type is used
        MarkerBased = 1 << 0,       //!< attached to a marker
        NodeBased = 1 << 1,         //!< attached to a node
        ObjectBased = 1 << 2,       //!< attached to an object
        RigidBodyAttached = 1 << 3, //!< attached to rigid body (using marker)
        Sphere = 1 << 4,            //!< sphere (circle) attached e.g. to Marker
        ANCFCable2D = 1 << 5,       //!< very special contact, only accepting ANCFCable2D elements (using cubic spline)
        Line2D = 1 << 6,            //!< line (x/y line for 2D contact, represent planes with z in [-infty,+infty]
        Triangle = 1 << 7           //!< triangle
    };

    //! transform type into string (e.g. for error messages); this is slow and cannot be used during computation!
    inline STDstring GetTypeString(Type var)
    {
        STDstring t; //empty string
        if (var == _None) { t = "_None/Undefined"; }
        if (var & MarkerBased) { t += "MarkerBased"; }
        if (var & NodeBased) { t += "NodeBased"; }
        if (var & ObjectBased) { t += "ObjectBased"; }
        if (var & RigidBodyAttached) { t += "RigidBodyAttached"; }
        if (var & Sphere) { t += "Sphere"; }
        if (var & ANCFCable2D) { t += "ANCFCable2D"; }
        if (var & Line2D) { t += "Line2D"; }
        if (var & Triangle) { t += "Triangle"; }
        if (t.length() == 0) { CHECKandTHROWstring("Contact::GetTypeString(...) called for invalid type!"); }
        return t;
    }

    //! maps contact types to arrays in GeneralContact
    enum TypeIndex {
        IndexSpheresMarkerBased = 0,  //!< spheres attached to markers
        IndexANCFCable2D = 1,         //!< ANCFCable2D contact items
        IndexTrigsRigidBodyBased = 2, //!< triangles attached to rigid body (or rigid body marker)
        IndexEndOfEnumList = 3        //!< signals end of list
    };

    //! transform type into string (e.g. for error messages); this is slow and cannot be used during computation!
    inline STDstring GetTypeIndexString(TypeIndex var)
    {
        switch (var)
        {
        case IndexSpheresMarkerBased: return "SpheresMarkerBased";
        case IndexANCFCable2D: return "ANCFCable2D";
        case IndexTrigsRigidBodyBased: return "TrigsRigidBodyBased";
        case IndexEndOfEnumList: return "EndOfEnumList";
        default: CHECKandTHROWstring("Contact::GetTypeIndexString(...) called for invalid type!"); return "";
        }
    }
} //namespace Contact

#endif //ENUMTYPES__H
