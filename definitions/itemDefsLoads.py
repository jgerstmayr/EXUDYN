#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Load item definitions
#
# Details:  The input of the generators for load items.
#           This IS Python: import it and read "definitions", a list of dicts.
#
#           ORDER MATTERS. The generators emit in the order the definitions appear,
#           and the generated C++/pybind/RST is compared byte-for-byte, so
#           reordering this list changes generated files. Append at the end unless
#           you mean to reorder.
#
#           Only descriptions, LaTeX and C++ code are raw strings; every other field
#           is a name, a flag constant or a short literal and needs no escaping.
#
#           The constants come from definitionTypes.py, which is hand-written: a
#           value used here with no constant there stops the emit and says what to
#           add, so the two can never drift apart silently.
#
#           DESCRIPTIONS: read definitions/README.md, section "Writing a
#           description", before writing or changing one - what the text may
#           contain, and how it is checked.
#
# Contents: LoadForceVector, LoadTorqueVector, LoadMassProportional, LoadCoordinate
#
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from definitionTypes import *
from outputVariableTypes import *
from outputVariableDescriptions import *

definitions = []
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   LoadForceVector   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def LoadForceVector_loadVectorUserFunction(mbs: MainSystem, t: Real, loadVector: Vector3D) -> Vector3D:
    r"""A user function, which computes the force vector depending on time and object parameters, which is hereafter applied to object or node.

    Args:
        mbs: provides MainSystem mbs to which load belongs
        t: current time in mbs
        loadVector: $\fv$ copied from object; WARNING: this parameter does not work in combination with static computation, as it is changed by the solver over step time
    Returns:
        computed force vector
    """

definitions.append(ItemDefinition(
    className='LoadForceVector',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCLoad,
    classDescription=r'Load with (3D) force vector; attached to position-based marker.',
    classType=ClassTypeLoad,
    equations=r"""    #### Details

    The load vector acts on a body or node via the local (`bodyFixed = True`) or global coordinates of a body or at a node. 
    The marker transforms the (translational) force via the according jacobian matrix of the object (or node) to object (or node) coordinates.

""",
    mainParentClass=MainParentClassMainLoad,
    pythonShortName='Force',
    visuParentClass=VisuParentClassVisualizationLoad,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"load's unique name"),
        ItemParameter(type=TIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumber',
            defaultValue=DVInvalidIndex,
            description=r"marker's number to which load is applied"),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='loadVector',
            defaultValue=DVZeroVector3D,
            description=r"""$\fv$vector-valued load [SI:N]; in case of a user function, this vector is ignored"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='bodyFixed',
            defaultValue=False,
            description=r'if bodyFixed is true, the load is defined in body-fixed (local) coordinates, leading to a follower force; if false: global coordinates are used'),
        ItemParameter(type=TPyFunctionVector3DmbsScalarVector3D, destination=DestComp+DestParam,
            pythonName='loadVectorUserFunction',
            defaultValue=0,
            description=r"""$\mathrm{UF} \in \Rcal^3$A Python function which defines the time-dependent load and replaces loadVector; see description below; NOTE that in static computations, the loadFactor is always 1 for forces computed by user functions (this means for the static computation, that a user function returning [t*5,t*1,0] corresponds to loadVector=[5,1,0] without a user function); NOTE that forces are drawn using the value of loadVector; thus the current values according to the user function are NOT shown in the render window; however, a sensor (SensorLoad) returns the user function force which is applied to the object; to draw forces with current user function values, use a graphicsDataUserFunction of a ground object""",
            userFunction=LoadForceVector_loadVectorUserFunction,
            userFunctionExample=r'''
from math import sin, cos, pi
def UFforce(mbs, t, loadVector): 
    return [loadVector[0]*sin(t*10*2*pi),0,0]
'''),
        ItemFunctionDef('GetMarkerNumber',
            implementation='return parameters.markerNumber;'),
        ItemFunctionDef('SetMarkerNumber',
            implementation='parameters.markerNumber = markerNumberInit;'),
        ItemRequestedTypes('Marker', ['Position']),
        ItemFunction(type=TLoadType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (LoadType)((Index)LoadType::Force);',
            description=r'return force type'),
        ItemFunctionDef('IsVector',
            implementation='return true;'),
        ItemFunctionDef('GetLoadVector'),
        ItemFunctionDef('IsBodyFixed',
            implementation='return parameters.bodyFixed;'),
        ItemFunctionDef('HasUserFunction',
            implementation='return parameters.loadVectorUserFunction != 0;'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ForceVector";',
            description=r"Get type name of load (without keyword 'Load'...!); could also be realized via a string -> type conversion?"),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   LoadTorqueVector   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def LoadTorqueVector_loadVectorUserFunction(mbs: MainSystem, t: Real, loadVector: Vector3D) -> Vector3D:
    r"""A user function, which computes the torque vector depending on time and object parameters, which is hereafter applied to object or node.

    Args:
        mbs: provides MainSystem mbs to which load belongs
        t: current time in mbs
        loadVector: $\ttau$ copied from object; WARNING: this parameter does not work in combination with static computation, as it is changed by the solver over step time
    Returns:
        computed torque vector
    """

definitions.append(ItemDefinition(
    className='LoadTorqueVector',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCLoad,
    classDescription=r'Load with (3D) torque vector; attached to rigidbody-based marker.',
    classType=ClassTypeLoad,
    equations=r"""    #### Details

    The torque vector acts on a body or node via the local (`bodyFixed = True`) or global coordinates of a body or at a node. 
    The marker transforms the torque via the according jacobian matrix of the object (or node) to object (or node) coordinates.

""",
    mainParentClass=MainParentClassMainLoad,
    pythonShortName='Torque',
    visuParentClass=VisuParentClassVisualizationLoad,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"load's unique name"),
        ItemParameter(type=TIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumber',
            defaultValue=DVInvalidIndex,
            description=r"marker's number to which load is applied"),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='loadVector',
            defaultValue=DVZeroVector3D,
            description=r"""$\ttau$vector-valued load [SI:N]; in case of a user function, this vector is ignored"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='bodyFixed',
            defaultValue=False,
            description=r'if bodyFixed is true, the load is defined in body-fixed (local) coordinates, leading to a follower torque; if false: global coordinates are used'),
        ItemParameter(type=TPyFunctionVector3DmbsScalarVector3D, destination=DestComp+DestParam,
            pythonName='loadVectorUserFunction',
            defaultValue=0,
            description=r"""$\mathrm{UF} \in \Rcal^3$A Python function which defines the time-dependent load and replaces loadVector; see description below; see also notes on loadFactor and drawing in LoadForceVector! Example for Python function: def f(mbs, t, loadVector): return [loadVector[0]*np.sin(t*10*2*3.1415),0,0]""",
            userFunction=LoadTorqueVector_loadVectorUserFunction,
            userFunctionExample=r'''
from math import sin, cos, pi
def UFforce(mbs, t, loadVector): 
    return [loadVector[0]*sin(t*10*2*pi),0,0]
'''),
        ItemFunctionDef('GetMarkerNumber',
            implementation='return parameters.markerNumber;'),
        ItemFunctionDef('SetMarkerNumber',
            implementation='parameters.markerNumber = markerNumberInit;'),
        ItemRequestedTypes('Marker', ['Orientation']),
        ItemFunction(type=TLoadType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (LoadType)((Index)LoadType::Torque);',
            description=r'return load type'),
        ItemFunctionDef('IsVector',
            implementation='return true;'),
        ItemFunctionDef('GetLoadVector'),
        ItemFunctionDef('IsBodyFixed',
            implementation='return parameters.bodyFixed;'),
        ItemFunctionDef('HasUserFunction',
            implementation='return parameters.loadVectorUserFunction != 0;'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "TorqueVector";',
            description=r"Get type name of load (without keyword 'Load'...!); could also be realized via a string -> type conversion?"),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   LoadMassProportional   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def LoadMassProportional_loadVectorUserFunction(mbs: MainSystem, t: Real,
                                                loadVector: Vector3D) -> Vector3D:
    r"""A user function, which computes the mass proporitional load vector depending on time and object parameters, which is hereafter applied to object or node.

    Example of user function: functionality same as in `LoadForceVector`

    Args:
        mbs: provides MainSystem mbs to which load belongs
        t: current time in mbs
        loadVector: $\bv$ copied from object; WARNING: this parameter does not work in combination with static computation, as it is changed by the solver over step time
    Returns:
        computed load vector
    """

definitions.append(ItemDefinition(
    className='LoadMassProportional',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCLoad,
    classDescription=r'Load attached to MarkerBodyMass marker, applying a 3D vector load (e.g. the vector [0,-g,0] is used to apply gravitational loading of size g in negative y-direction).',
    classType=ClassTypeLoad,
    equations=r"""    #### Details

    The load applies a (translational) and distributed load proportional to the distributed body's density.
    The marker of type `MarkerBodyMass` transforms the loadVector via an according jacobian matrix to object coordinates.

""",
    mainParentClass=MainParentClassMainLoad,
    miniExample=r"""    node = mbs.AddNode(NodePoint(referenceCoordinates = [1,0,0]))
    body = mbs.AddObject(MassPoint(nodeNumber = node, physicsMass=2))
    mMass = mbs.AddMarker(MarkerBodyMass(bodyNumber=body))
    mbs.AddLoad(LoadMassProportional(markerNumber=mMass, loadVector=[0,0,-9.81]))

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic()

    #check result
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[2]
    #final z-coordinate of position shall be -g/2 due to constant acceleration with g=-9.81
    #result independent of mass
""",
    pythonShortName='Gravity',
    visuParentClass=VisuParentClassVisualizationLoad,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"load's unique name"),
        ItemParameter(type=TIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumber',
            defaultValue=DVInvalidIndex,
            description=r"marker's number to which load is applied"),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='loadVector',
            defaultValue=DVZeroVector3D,
            description=r"""$\bv$vector-valued load [SI:N/kg = m/s$^2$]; typically, this will be the gravity vector in global coordinates; in case of a user function, this v is ignored"""),
        ItemParameter(type=TPyFunctionVector3DmbsScalarVector3D, destination=DestComp+DestParam,
            pythonName='loadVectorUserFunction',
            defaultValue=0,
            description=r"""$\mathrm{UF} \in \Rcal^3$A Python function which defines the time-dependent load; see description below; see also notes on loadFactor and drawing in LoadForceVector!""",
            userFunction=LoadMassProportional_loadVectorUserFunction),
        ItemFunctionDef('GetMarkerNumber',
            implementation='return parameters.markerNumber;'),
        ItemFunctionDef('SetMarkerNumber',
            implementation='parameters.markerNumber = markerNumberInit;'),
        ItemRequestedTypes('Marker', ['BodyMass']),
        ItemFunction(type=TLoadType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (LoadType)((Index)LoadType::ForcePerMass);',
            description=r'return load type'),
        ItemFunctionDef('IsVector',
            implementation='return true;'),
        ItemFunctionDef('HasUserFunction',
            implementation='return parameters.loadVectorUserFunction != 0;'),
        ItemFunctionDef('GetLoadVector'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "MassProportional";',
            description=r"Get type name of load (without keyword 'Load'...!); could also be realized via a string -> type conversion?"),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   LoadCoordinate   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def LoadCoordinate_loadUserFunction(mbs: MainSystem, t: Real, load: Real) -> Real:
    r"""A user function, which computes the scalar load depending on time and the object's `load` parameter.

    Args:
        mbs: provides MainSystem mbs to which load belongs
        t: current time in mbs
        load: $\bv$ copied from object; WARNING: this parameter does not work in combination with static computation, as it is changed by the solver over step time
    Returns:
        computed load
    """

definitions.append(ItemDefinition(
    className='LoadCoordinate',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCLoad,
    classDescription=r'Load with scalar value, which is attached to a coordinate-based marker; the load can be used e.g. to apply a force to a single axis of a body, a nodal coordinate of a finite element  or a torque to the rotatory DOF of a rigid body.',
    classType=ClassTypeLoad,
    equations=r"""    #### Details

    The scalar `load` is applied on a coordinate defined by a Marker of type 'Coordinate', e.g., `MarkerNodeCoordinate`.
    This can be used to create simple 1D problems, or to simply apply a translational force on a Node or even a torque
    on a rotation coordinate (but take care for its meaning).

""",
    mainParentClass=MainParentClassMainLoad,
    visuParentClass=VisuParentClassVisualizationLoad,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"load's unique name"),
        ItemParameter(type=TIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumber',
            defaultValue=DVInvalidIndex,
            description=r"marker's number to which load is applied"),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='load',
            defaultValue=0.,
            description=r'$f$scalar load [SI:N]; in case of a user function, this value is ignored'),
        ItemParameter(type=TPyFunctionMbsScalar2, destination=DestComp+DestParam,
            pythonName='loadUserFunction',
            defaultValue=0,
            description=r"""$\mathrm{UF} \in \Rcal$A Python function which defines the time-dependent load and replaces the load; see description below; see also notes on loadFactor and drawing in LoadForceVector!""",
            userFunction=LoadCoordinate_loadUserFunction,
            userFunctionExample=r'''
from math import sin, cos, pi
#this example uses the object's stored parameter load to compute a time-dependent load
def UFload(mbs, t, load): 
    return load*sin(10*(2*pi)*t)

n0=mbs.AddNode(Point())
nodeMarker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0,coordinate=0))
mbs.AddLoad(LoadCoordinate(markerNumber = markerCoordinate,
                           load = 10,
                           loadUserFunction = UFload))
'''),
        ItemFunctionDef('GetMarkerNumber',
            implementation='return parameters.markerNumber;'),
        ItemFunctionDef('SetMarkerNumber',
            implementation='parameters.markerNumber = markerNumberInit;'),
        ItemRequestedTypes('Marker', ['Coordinate']),
        ItemFunction(type=TLoadType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (LoadType)((Index)LoadType::Coordinate);',
            description=r'return load type'),
        ItemFunctionDef('IsVector',
            implementation='return false;'),
        ItemFunctionDef('HasUserFunction',
            implementation='return parameters.loadUserFunction != 0;'),
        ItemFunctionDef('GetLoadValue'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "Coordinate";',
            description=r"Get type name of load (without keyword 'Load'...!); could also be realized via a string -> type conversion?"),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics',
            implementation=';'),
        ],
    ))
