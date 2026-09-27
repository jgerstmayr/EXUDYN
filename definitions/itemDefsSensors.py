#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Sensor item definitions
#
# Details:  The input of the generators for sensor items.
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
# Contents: SensorNode, SensorObject, SensorBody, SensorSuperElement, SensorKinematicTree, SensorMarker, ...
#
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from definitionTypes import *
from outputVariableTypes import *
from outputVariableDescriptions import *

definitions = []
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   SensorNode   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='SensorNode',
    cParentClass=ParentClassCSensor,
    overallDescription=r"""A sensor attached to a node, which measures one of the output variables of the node.""",
    classType=ClassTypeSensor,
    detailedDescription=r"""    #### Attached to

    The node `nodeNumber`: every node, with ABRV:ODE2, ABRV:ODE1 or other coordinates.

    #### Measures

    `outputVariableType` is one of the output variables of the node, as its page lists them under
    **Output variables** - `Position`, `Velocity`, `Coordinates`, and for a rigid body node also
    `RotationMatrix`, `AngularVelocity` and the like. The frame is the one the node page gives for the
    variable.
    """,
    mainParentClass=MainParentClassMainSensor,
    visuParentClass=VisuParentClassVisualizationSensor,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"sensor's unique name"),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'node number to which sensor is attached to'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='writeToFile',
            defaultValue=True,
            description=r"True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''"),
        ItemParameter(type=TString, destination=DestComp+DestParam,
            pythonName='fileName',
            defaultValue=NoDefaultValue,
            description=r'directory and file name for sensor file output; empty: no file is written; a relative name is placed in `exudyn.config.outputDirectory` if that is set; the directory is created if it does not exist'),
        ItemParameter(type=TOutputVariableType, destination=DestComp+DestParam,
            pythonName='outputVariableType',
            defaultValue='OutputVariableType::_None',
            description=r'OutputVariableType for sensor'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='storeInternal',
            defaultValue=False,
            description=r'true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available'),
        ItemFunctionDef('GetNodeNumber',
            implementation='return parameters.nodeNumber;'),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber = nodeNumber;'),
        ItemFunction(type=TSensorType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return SensorType::Node;',
            description=r'return sensor type'),
        ItemFunctionDef('GetWriteToFileFlag',
            implementation='return parameters.writeToFile;'),
        ItemFunctionDef('GetStoreInternalFlag',
            implementation='return parameters.storeInternal;'),
        ItemFunctionDef('GetFileName',
            implementation='return parameters.fileName;'),
        ItemFunctionDef('GetOutputVariableType',
            implementation='return parameters.outputVariableType;'),
        ItemFunctionDef('GetSensorValues'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "Node";',
            description=r"Get type name of sensor (without keyword 'Sensor'...!)"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   SensorObject   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='SensorObject',
    cParentClass=ParentClassCSensor,
    overallDescription=r"""A sensor attached to an object other than a body - a connector, a constraint, a joint - which measures one of the output variables of the object; a body is measured at a point, with SensorBody.""",
    classType=ClassTypeSensor,
    detailedDescription=r"""    #### Attached to

    The object `objectNumber`, usually a connector, constraint or joint, which is measured as a whole and
    not at a point.

    #### Measures

    `outputVariableType` is one of the output variables of the object, as its page lists them under
    **Output variables** - for a spring-damper e.g. `Force` and `Distance`, for a joint the reaction
    forces and torques in the frame its page gives.
    """,
    mainParentClass=MainParentClassMainSensor,
    visuParentClass=VisuParentClassVisualizationSensor,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"sensor's unique name"),
        ItemParameter(type=TIndex(ItemObject), destination=DestComp+DestParam,
            pythonName='objectNumber',
            defaultValue=DVInvalidIndex,
            description=r'object (e.g. connector) number to which sensor is attached to'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='writeToFile',
            defaultValue=True,
            description=r"True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''"),
        ItemParameter(type=TString, destination=DestComp+DestParam,
            pythonName='fileName',
            defaultValue=NoDefaultValue,
            description=r'directory and file name for sensor file output; empty: no file is written; a relative name is placed in `exudyn.config.outputDirectory` if that is set; the directory is created if it does not exist'),
        ItemParameter(type=TOutputVariableType, destination=DestComp+DestParam,
            pythonName='outputVariableType',
            defaultValue='OutputVariableType::_None',
            description=r'OutputVariableType for sensor'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='storeInternal',
            defaultValue=False,
            description=r'true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available'),
        ItemFunctionDef('GetObjectNumber',
            implementation='return parameters.objectNumber;'),
        ItemFunctionDef('SetObjectNumber',
            args='Index objectNumber',
            implementation='parameters.objectNumber = objectNumber;'),
        ItemFunction(type=TSensorType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return SensorType::Object;',
            description=r'return sensor type'),
        ItemFunctionDef('GetWriteToFileFlag',
            implementation='return parameters.writeToFile;'),
        ItemFunctionDef('GetStoreInternalFlag',
            implementation='return parameters.storeInternal;'),
        ItemFunctionDef('GetFileName',
            implementation='return parameters.fileName;'),
        ItemFunctionDef('GetOutputVariableType',
            implementation='return parameters.outputVariableType;'),
        ItemFunctionDef('GetSensorValues'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "Object";',
            description=r"Get type name of sensor (without keyword 'Sensor'...!)"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown; sensors can be shown at the position assiciated with the object - note that in some cases, there might be no such position (e.g. data object)!'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   SensorBody   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='SensorBody',
    cParentClass=ParentClassCSensor,
    overallDescription=r"""A sensor attached to a body at a local position $\pLocB$, which measures one of the output variables of the body at that point.""",
    classType=ClassTypeSensor,
    detailedDescription=r"""    #### Attached to

    The body `bodyNumber`, at the point `localPosition` $\pLocB$. The local position is given in the
    body frame and measured from the **reference point** of the body - the position of its node, which
    is `referencePosition` in `CreateRigidBody`. The center of mass of an `ObjectRigidBody` lies at
    `physicsCenterOfMass` from it, so a sensor at the center of mass has that local position.

    #### Measures

    `outputVariableType` is one of the output variables of the body, evaluated at $\pLocB$, as its page
    lists them under **Output variables** - e.g. `Position`, `Velocity`, `Displacement`,
    `AngularVelocity`; for a flexible body also strains or forces along its axis.
    """,
    mainParentClass=MainParentClassMainSensor,
    visuParentClass=VisuParentClassVisualizationSensor,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"sensor's unique name"),
        ItemParameter(type=TIndex(ItemObject), destination=DestComp+DestParam,
            pythonName='bodyNumber',
            defaultValue=DVInvalidIndex,
            description=r'body (=object) number to which sensor is attached to'),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='localPosition',
            defaultValue=DVZeroVector3D,
            description=r'$\pLocB$local (body-fixed) body position of sensor'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='writeToFile',
            defaultValue=True,
            description=r"True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''"),
        ItemParameter(type=TString, destination=DestComp+DestParam,
            pythonName='fileName',
            defaultValue=NoDefaultValue,
            description=r'directory and file name for sensor file output; empty: no file is written; a relative name is placed in `exudyn.config.outputDirectory` if that is set; the directory is created if it does not exist'),
        ItemParameter(type=TOutputVariableType, destination=DestComp+DestParam,
            pythonName='outputVariableType',
            defaultValue='OutputVariableType::_None',
            description=r'OutputVariableType for sensor'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='storeInternal',
            defaultValue=False,
            description=r'true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available'),
        ItemFunctionDef('GetObjectNumber',
            implementation='return parameters.bodyNumber;'),
        ItemFunctionDef('SetObjectNumber',
            args='Index bodyNumber',
            implementation='parameters.bodyNumber = bodyNumber;'),
        ItemFunction(type=TSensorType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return SensorType::Body;',
            description=r'return sensor type'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetBodyLocalPosition',
            implementation='return parameters.localPosition;',
            description=r'get local position'),
        ItemFunctionDef('GetWriteToFileFlag',
            implementation='return parameters.writeToFile;'),
        ItemFunctionDef('GetStoreInternalFlag',
            implementation='return parameters.storeInternal;'),
        ItemFunctionDef('GetFileName',
            implementation='return parameters.fileName;'),
        ItemFunctionDef('GetOutputVariableType',
            implementation='return parameters.outputVariableType;'),
        ItemFunctionDef('GetSensorValues'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "Body";',
            description=r"Get type name of sensor (without keyword 'Sensor'...!)"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   SensorSuperElement   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='SensorSuperElement',
    cParentClass=ParentClassCSensor,
    overallDescription=r"""A sensor attached to a mesh node of a superelement, which measures one of the output variables of the superelement at that mesh node.""",
    classType=ClassTypeSensor,
    detailedDescription=r"""    #### Attached to

    The superelement `bodyNumber` - `ObjectFFRF`, `ObjectFFRFreducedOrder`, `ObjectGenericODE2` - at its
    mesh node `meshNodeNumber`, a node number local to the object, starting with 0. The mesh node may be
    a node of the system (`ObjectFFRF`) or exist only in the object and be reconstructed from its
    coordinates (`ObjectFFRFreducedOrder`).

    #### Measures

    `outputVariableType` is one of the output variables the superelement provides for its mesh nodes, as
    its page lists them - the section on superelement output variables of `ObjectFFRFreducedOrder`, e.g.
    the displacement or position of the mesh node in the local or global frame.
    """,
    mainParentClass=MainParentClassMainSensor,
    visuParentClass=VisuParentClassVisualizationSensor,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"sensor's unique name"),
        ItemParameter(type=TIndex(ItemObject), destination=DestComp+DestParam,
            pythonName='bodyNumber',
            defaultValue=DVInvalidIndex,
            description=r'body (=object) number to which sensor is attached to'),
        ItemParameter(type=TIndex(minimum=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='meshNodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'mesh node number, which is a local node number with in the object (starting with 0); the node number may represent a real Node in mbs, or may be virtual and reconstructed from the object coordinates such as in ObjectFFRFreducedOrder'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='writeToFile',
            defaultValue=True,
            description=r"True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''"),
        ItemParameter(type=TString, destination=DestComp+DestParam,
            pythonName='fileName',
            defaultValue=NoDefaultValue,
            description=r'directory and file name for sensor file output; empty: no file is written; a relative name is placed in `exudyn.config.outputDirectory` if that is set; the directory is created if it does not exist'),
        ItemParameter(type=TOutputVariableType, destination=DestComp+DestParam,
            pythonName='outputVariableType',
            defaultValue='OutputVariableType::_None',
            description=r"""OutputVariableType for sensor, based on the output variables available for the mesh nodes (see special section for super element output variables, e.g, in ObjectFFRFreducedOrder, [](#sec-objectffrfreducedorder-superelementoutput))"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='storeInternal',
            defaultValue=False,
            description=r'true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available'),
        ItemFunctionDef('GetObjectNumber',
            implementation='return parameters.bodyNumber;'),
        ItemFunctionDef('SetObjectNumber',
            args='Index bodyNumber',
            implementation='parameters.bodyNumber = bodyNumber;'),
        ItemFunction(type=TSensorType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return SensorType::SuperElement;',
            description=r'return sensor type'),
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetMeshNodeNumber',
            implementation='return parameters.meshNodeNumber;',
            description=r'get local position'),
        ItemFunctionDef('GetWriteToFileFlag',
            implementation='return parameters.writeToFile;'),
        ItemFunctionDef('GetStoreInternalFlag',
            implementation='return parameters.storeInternal;'),
        ItemFunctionDef('GetFileName',
            implementation='return parameters.fileName;'),
        ItemFunctionDef('GetOutputVariableType',
            implementation='return parameters.outputVariableType;'),
        ItemFunctionDef('GetSensorValues'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "SuperElement";',
            description=r"Get type name of sensor (without keyword 'Sensor'...!)"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   SensorKinematicTree   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='SensorKinematicTree',
    cParentClass=ParentClassCSensor,
    overallDescription=r"""A sensor attached to a link $n_l$ of an ObjectKinematicTree at a local position $\pLocB$ in the frame of the link, which measures one of the output variables of the kinematic tree at that point.""",
    classType=ClassTypeSensor,
    detailedDescription=r"""    #### Attached to

    The `ObjectKinematicTree` `objectNumber`, at its link `linkNumber` $n_l$ and the point
    `localPosition` $\LU{l}{\bv}$, given in the frame of that link.

    #### Measures

    `outputVariableType` is one of the output variables the kinematic tree provides for a link, as its
    page lists them under the output variables of `SensorKinematicTree` - e.g. the position, velocity or
    rotation of that point of the link.
    """,
    mainParentClass=MainParentClassMainSensor,
    visuParentClass=VisuParentClassVisualizationSensor,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"sensor's unique name"),
        ItemParameter(type=TIndex(ItemObject), destination=DestComp+DestParam,
            pythonName='objectNumber',
            defaultValue=DVInvalidIndex,
            description=r'object number of KinematicTree to which sensor is attached to'),
        ItemParameter(type=TIndex(minimum=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='linkNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n_l$number of link in KinematicTree to measure quantities'),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='localPosition',
            defaultValue=DVZeroVector3D,
            description=r"""$\LU{l}{\bv}$local (link-fixed) position of sensor, defined in link ($n_l$) coordinate system"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='writeToFile',
            defaultValue=True,
            description=r"True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''"),
        ItemParameter(type=TString, destination=DestComp+DestParam,
            pythonName='fileName',
            defaultValue=NoDefaultValue,
            description=r'directory and file name for sensor file output; empty: no file is written; a relative name is placed in `exudyn.config.outputDirectory` if that is set; the directory is created if it does not exist'),
        ItemParameter(type=TOutputVariableType, destination=DestComp+DestParam,
            pythonName='outputVariableType',
            defaultValue='OutputVariableType::_None',
            description=r'OutputVariableType for sensor'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='storeInternal',
            defaultValue=False,
            description=r'true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available'),
        ItemFunctionDef('GetObjectNumber',
            implementation='return parameters.objectNumber;'),
        ItemFunctionDef('SetObjectNumber',
            args='Index objectNumber',
            implementation='parameters.objectNumber = objectNumber;'),
        ItemFunction(type=TSensorType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return SensorType::KinematicTree;',
            description=r'return sensor type'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetBodyLocalPosition',
            implementation='return parameters.localPosition;',
            description=r'get local position'),
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetLinkNumber',
            implementation='return parameters.linkNumber;',
            description=r'general access to link number'),
        ItemFunctionDef('GetWriteToFileFlag',
            implementation='return parameters.writeToFile;'),
        ItemFunctionDef('GetStoreInternalFlag',
            implementation='return parameters.storeInternal;'),
        ItemFunctionDef('GetFileName',
            implementation='return parameters.fileName;'),
        ItemFunctionDef('GetOutputVariableType',
            implementation='return parameters.outputVariableType;'),
        ItemFunctionDef('GetSensorValues'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "KinematicTree";',
            description=r"Get type name of sensor (without keyword 'Sensor'...!)"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   SensorMarker   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='SensorMarker',
    cParentClass=ParentClassCSensor,
    overallDescription=r"""A sensor attached to a marker, which measures what the marker provides, in the current configuration.""",
    classType=ClassTypeSensor,
    detailedDescription=r"""    #### Attached to

    The marker `markerNumber`, any kind of marker.

    #### Measures

    What the marker provides, which depends on its types, and only in the **current** configuration:

    | the marker provides | `outputVariableType` can be |
    |---|---|
    | a position (`MarkerBodyPosition`, `MarkerNodePosition`, ...) | `Position`, `Displacement`, `Velocity` |
    | an orientation (`MarkerBodyRigid`, `MarkerNodeRigid`, ...) | in addition `RotationMatrix`, `Rotation`, `AngularVelocity`, `AngularVelocityLocal` |
    | coordinates (`MarkerNodeCoordinate`, `MarkerNodeCoordinates`, ...) | `Coordinates`, `Coordinates_t` |
    | a relative coordinate of two bodies | `Coordinates`, `Coordinates_t` |

    Markers have no output variables of their own, which is why this sensor lists them here.
    """,
    mainParentClass=MainParentClassMainSensor,
    visuParentClass=VisuParentClassVisualizationSensor,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"sensor's unique name"),
        ItemParameter(type=TIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumber',
            defaultValue=DVInvalidIndex,
            description=r'marker number to which sensor is attached to'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='writeToFile',
            defaultValue=True,
            description=r"True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''"),
        ItemParameter(type=TString, destination=DestComp+DestParam,
            pythonName='fileName',
            defaultValue=NoDefaultValue,
            description=r'directory and file name for sensor file output; empty: no file is written; a relative name is placed in `exudyn.config.outputDirectory` if that is set; the directory is created if it does not exist'),
        ItemParameter(type=TOutputVariableType, destination=DestComp+DestParam,
            pythonName='outputVariableType',
            defaultValue='OutputVariableType::_None',
            description=r'OutputVariableType for sensor; output variables are only possible according to markertype, see general description of SensorMarker'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='storeInternal',
            defaultValue=False,
            description=r'true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available'),
        ItemFunctionDef('GetMarkerNumber',
            implementation='return parameters.markerNumber;',
            description='general access to marker number'),
        ItemFunctionDef('SetMarkerNumber',
            implementation='parameters.markerNumber = markerNumber;'),
        ItemFunction(type=TSensorType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return SensorType::Marker;',
            description=r'return sensor type'),
        ItemFunctionDef('GetWriteToFileFlag',
            implementation='return parameters.writeToFile;'),
        ItemFunctionDef('GetStoreInternalFlag',
            implementation='return parameters.storeInternal;'),
        ItemFunctionDef('GetFileName',
            implementation='return parameters.fileName;'),
        ItemFunctionDef('GetOutputVariableType',
            implementation='return parameters.outputVariableType;'),
        ItemFunctionDef('GetSensorValues'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "Marker";',
            description=r"Get type name of sensor (without keyword 'Sensor'...!)"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   SensorLoad   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='SensorLoad',
    cParentClass=ParentClassCSensor,
    overallDescription=r"""A sensor attached to a load, which measures the value of the load.""",
    classType=ClassTypeSensor,
    detailedDescription=r"""    #### Attached to

    The load `loadNumber`.

    #### Measures

    The value of the load at the current time: the vector or the scalar, as the load item computes it -
    from `loadVector` or `load`, or from its user function - and in the frame it is given in, which is
    the body frame for a load with `bodyFixed = True`. The load factor of a static computation is not
    included. There is no `outputVariableType`.
    """,
    mainParentClass=MainParentClassMainSensor,
    visuParentClass=VisuParentClassVisualizationSensor,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"sensor's unique name"),
        ItemParameter(type=TIndex(ItemLoad), destination=DestComp+DestParam,
            pythonName='loadNumber',
            defaultValue=DVInvalidIndex,
            description=r'load number to which sensor is attached to'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='writeToFile',
            defaultValue=True,
            description=r"True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''"),
        ItemParameter(type=TString, destination=DestComp+DestParam,
            pythonName='fileName',
            defaultValue=NoDefaultValue,
            description=r'directory and file name for sensor file output; empty: no file is written; a relative name is placed in `exudyn.config.outputDirectory` if that is set; the directory is created if it does not exist'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='storeInternal',
            defaultValue=False,
            description=r'true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available'),
        ItemFunctionDef('GetLoadNumber',
            implementation='return parameters.loadNumber;'),
        ItemFunctionDef('SetLoadNumber',
            implementation='parameters.loadNumber = loadNumber;'),
        ItemFunction(type=TSensorType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return SensorType::Load;',
            description=r'return sensor type'),
        ItemFunctionDef('GetWriteToFileFlag',
            implementation='return parameters.writeToFile;'),
        ItemFunctionDef('GetStoreInternalFlag',
            implementation='return parameters.storeInternal;'),
        ItemFunctionDef('GetFileName',
            implementation='return parameters.fileName;'),
        ItemFunctionDef('GetOutputVariableType',
            implementation='return OutputVariableType::_None;'),
        ItemFunctionDef('GetSensorValues'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "Load";',
            description=r"Get type name of sensor (without keyword 'Sensor'...!)"),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown; sensor visualization CURRENTLY NOT IMPLEMENTED'),
        ItemFunctionDef('UpdateGraphics',
            implementation=';'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   SensorUserFunction   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def SensorUserFunction_sensorUserFunction(mbs: MainSystem, t: Real, sensorNumbers: Array,
                                          factors: Vector, configuration: ConfigurationType) -> Vector:
    r"""A user function, which computes a sensor output from other sensor outputs (or from generic time dependent functions).

    The configuration in general will be the exudyn.ConfigurationType.Current, but others could be used as well except for SensorMarker.

    The user function arguments are as follows:

    Args:
        mbs: provides MainSystem mbs to which object belongs
        t: current time in mbs
        sensorNumbers: $\in \Ncal^n$ list of sensor numbers
        factors: $\in \Rcal^n$ list of factors that can be freely used for the user function
        configuration: usually the exudyn.ConfigurationType.Current, but could also be different in user defined functions.
    Returns:
        $\in \Rcal^{n_r}$ returns list or numpy array of sensor output values; size $n_r$ is implicitly defined by the returned list and may not be changed during simulation.
    """

definitions.append(ItemDefinition(
    className='SensorUserFunction',
    cParentClass=ParentClassCSensor,
    overallDescription=r'A sensor defined by a user function. The sensor is intended to collect sensor values of a list of given sensors and recombine the output into a new value for output or control purposes. It is also possible to use this sensor without any dependence on other sensors in order to generate output for, e.g., any quantities in mbs or solvers.',
    classType=ClassTypeSensor,
    detailedDescription=r"""    The sensor collects data via a user function, which completely describes the output itself.
    Note that the sensorNumbers and factors need to be consistent. 
    The return value of the user function is a list of `float` numbers which cast to a `std::vector` in pybind.
    This list can have arbitrary dimension, but should be kept constant during simulation.

""",
    mainParentClass=MainParentClassMainSensor,
    visuParentClass=VisuParentClassVisualizationSensor,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"sensor's unique name"),
        ItemParameter(type=TArrayIndex(ItemSensor), destination=DestComp+DestParam,
            pythonName='sensorNumbers',
            defaultValue='ArrayIndex()',
            description=r"""$\mathbf{n}_s = [s_0,\,\ldots,\,s_n]\tp$optional list of $n$ sensor numbers for use in user function"""),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='factors',
            defaultValue='Vector()',
            description=r"""$\mathbf{f}_s = [f_0,\,\ldots,\,f_m]\tp$optional list of $m$ factors which can be used, e.g., for weighting sensor values"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='writeToFile',
            defaultValue=True,
            description=r"True: write sensor output to file; flag is ignored (interpreted as False), if fileName=''"),
        ItemParameter(type=TString, destination=DestComp+DestParam,
            pythonName='fileName',
            defaultValue=NoDefaultValue,
            description=r'directory and file name for sensor file output; empty: no file is written; a relative name is placed in `exudyn.config.outputDirectory` if that is set; the directory is created if it does not exist'),
        ItemParameter(type=TPyFunctionVectorMbsScalarArrayIndexVectorConfiguration, destination=DestComp+DestParam,
            pythonName='sensorUserFunction',
            defaultValue=0,
            description=r'A Python function which defines the time-dependent user function, which usually evaluates one or several sensors and computes a new sensor value, see example',
            userFunction=SensorUserFunction_sensorUserFunction,
            userFunctionExample=r'''
import exudyn as exu
from exudyn.itemInterface import *
from math import pi, atan2
SC = exu.SystemContainer()
mbs = SC.AddSystem()
node = mbs.AddNode(NodePoint(referenceCoordinates = [1,1,0], 
                             initialCoordinates=[0,0,0],
                             initialVelocities=[0,-1,0]))
mbs.AddObject(MassPoint(nodeNumber = node, physicsMass=1))

sNode = mbs.AddSensor(SensorNode(nodeNumber=node, fileName='solution/sensorTest.txt',
                      outputVariableType=exu.OutputVariableType.Position))

#user function for sensor, convert position into angle:
def UFsensor(mbs, t, sensorNumbers, factors, configuration):
    val = mbs.GetSensorValues(sensorNumbers[0]) #x,y,z
    phi = atan2(val[1],val[0]) #compute angle from x,y: atan2(y,x)
    return [factors[0]*phi] #return angle in degree

sUser = mbs.AddSensor(SensorUserFunction(sensorNumbers=[sNode], factors=[180/pi], 
                                 fileName='solution/sensorTest2.txt',
                                 sensorUserFunction=UFsensor))

#assemble and solve system for default parameters
mbs.Assemble()
mbs.SolveDynamic()

if False:
    from exudyn.plot import PlotSensor
    PlotSensor(mbs, [sNode, sNode, sUser], [0, 1, 0])
'''),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='storeInternal',
            defaultValue=False,
            description=r'true: store sensor data in memory (faster, but may consume large amounts of memory); false: internal storage not available'),
        ItemFunctionDef('GetSensorNumber',
            implementation='return parameters.sensorNumbers[localIndex];'),
        ItemFunctionDef('GetNumberOfSensors',
            implementation='return parameters.sensorNumbers.NumberOfItems();'),
        ItemFunctionDef('SetSensorNumber',
            implementation='parameters.sensorNumbers[localIndex] = sensorNumber;'),
        ItemFunction(type=TSensorType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return SensorType::UserFunction;',
            description=r'return sensor type'),
        ItemFunctionDef('GetWriteToFileFlag',
            implementation='return parameters.writeToFile;'),
        ItemFunctionDef('GetStoreInternalFlag',
            implementation='return parameters.storeInternal;'),
        ItemFunctionDef('GetFileName',
            implementation='return parameters.fileName;'),
        ItemFunctionDef('GetOutputVariableType',
            implementation='return OutputVariableType::_None;'),
        ItemFunctionDef('GetSensorValues'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "UserFunction";',
            description=r"Get type name of sensor (without keyword 'Sensor'...!)"),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunction',
            args='Vector& sensorValues, const MainSystemBase& mainSystem, Real t, ConfigurationType configuration',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown; sensor visualization CURRENTLY NOT IMPLEMENTED'),
        ItemFunctionDef('UpdateGraphics',
            implementation=';'),
        ],
    ))
