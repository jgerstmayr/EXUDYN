#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Sensor item definitions
#
# Details:  8 definitions; the input of the generators (revision plan step 33).
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
    classDescription=r"""A sensor attached to a \hac{ODE2} or \hac{ODE1} node. The sensor measures OutputVariables and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.""",
    classType=ClassTypeSensor,
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
            description=r'directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist'),
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
    classDescription=r'A sensor attached to any object except bodies  (connectors, constraint, spring-damper, etc). As a difference to other SensorBody, the connector sensor measures quantities without a local position. The sensor measures OutputVariable and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.',
    classType=ClassTypeSensor,
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
            description=r'directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist'),
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
    classDescription=r"""A sensor attached to a body-object with local position $\pLocB$. As a difference to SensorObject, the body sensor needs a local position at which the sensor is attached to. The sensor measures OutputVariableBody and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.""",
    classType=ClassTypeSensor,
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
            description=r'directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist'),
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
    classDescription=r'A sensor attached to a SuperElement-object with mesh node number. As a difference to other ObjectSensors, the SuperElement sensor has a mesh node number at which the sensor is attached to. The sensor measures OutputVariableSuperElement and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.',
    classType=ClassTypeSensor,
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
            description=r'directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist'),
        ItemParameter(type=TOutputVariableType, destination=DestComp+DestParam,
            pythonName='outputVariableType',
            defaultValue='OutputVariableType::_None',
            description=r"""OutputVariableType for sensor, based on the output variables available for the mesh nodes (see special section for super element output variables, e.g, in ObjectFFRFreducedOrder, \refSection{sec:objectffrfreducedorder:superelementoutput})"""),
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
    classDescription=r"""A sensor attached to a KinematicTree with local position $\pLocB$ and link number $n_l$. As a difference to SensorBody, the KinematicTree sensor needs a local position and a link number, which defines the sub-body at which the sensor values are evaluated. The local position is given in sub-body (link) local coordinates. The sensor measures OutputVariableKinematicTree and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.""",
    classType=ClassTypeSensor,
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
            description=r'directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist'),
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
    classDescription=r'A sensor attached to a marker. The sensor measures the selected marker values and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Depending on markers, it can measure Coordinates (MarkerNodeCoordinate), Position and Velocity (MarkerXXXPosition), Position, Velocity, Rotation and AngularVelocityLocal (MarkerXXXRigid). Note that marker values are only available for the current configuration. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file',
    classType=ClassTypeSensor,
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
            description=r'directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist'),
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
    classDescription=r'A sensor attached to a load. The sensor measures the load values and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.',
    classType=ClassTypeSensor,
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
            description=r'directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist'),
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
definitions.append(ItemDefinition(
    className='SensorUserFunction',
    cParentClass=ParentClassCSensor,
    classDescription=r'A sensor defined by a user function. The sensor is intended to collect sensor values of a list of given sensors and recombine the output into a new value for output or control purposes. It is also possible to use this sensor without any dependence on other sensors in order to generate output for, e.g., any quantities in mbs or solvers.',
    classType=ClassTypeSensor,
    equations=r"""    The sensor collects data via a user function, which completely describes the output itself.
    Note that the sensorNumbers and factors need to be consistent. 
    The return value of the user function is a list of \texttt{float} numbers which cast to a \texttt{std::vector} in pybind.
    This list can have arbitrary dimension, but should be kept constant during simulation.
    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    \userFunction{sensorUserFunction(mbs, t, sensorNumbers, factors, configuration)}
    A user function, which computes a sensor output from other sensor outputs (or from generic time dependent functions).
    The configuration in general will be the exudyn.ConfigurationType.Current, but others could be used as well except for SensorMarker.
    %
    The user function arguments are as follows:
    \startTable{arguments /  return}{type or size}{description}
      \rowTable{\texttt{mbs}}{MainSystem}{provides MainSystem mbs to which object belongs}
      \rowTable{\texttt{t}}{Real}{current time in mbs}
      \rowTable{\texttt{sensorNumbers}}{Array $\in \Ncal^n$}{list of sensor numbers}
      \rowTable{\texttt{factors}}{Vector $\in \Rcal^n$}{list of factors that can be freely used for the user function}
      \rowTable{\texttt{configuration}}{exudyn.ConfigurationType}{usually the exudyn.ConfigurationType.Current, but could also be different in user defined functions.}
      \rowTable{\returnValue}{Vector $\in \Rcal^{n_r}$}{returns list or numpy array of sensor output values; size $n_r$ is implicitly defined by the returned list and may not be changed during simulation.}
    \finishTable
    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    \userFunctionExample{}
    \pythonstyle\begin{lstlisting}
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
        
    \end{lstlisting}
    %%RSTCOMPATIBLE
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
            description=r'directory and file name for sensor file output; default: empty string generates sensor + sensorNumber + outputVariableType; directory will be created if it does not exist'),
        ItemParameter(type=TPyFunctionVectorMbsScalarArrayIndexVectorConfiguration, destination=DestComp+DestParam,
            pythonName='sensorUserFunction',
            defaultValue=0,
            description=r'A Python function which defines the time-dependent user function, which usually evaluates one or several sensors and computes a new sensor value, see example'),
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
