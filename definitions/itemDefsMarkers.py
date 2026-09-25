#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Marker item definitions
#
# Details:  18 definitions; the input of the generators.
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
# Contents: MarkerBodyMass, MarkerBodyPosition, MarkerBodyRigid, MarkerNodePosition, MarkerNodeRigid, MarkerNodeCoordinate, ...
#
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from definitionTypes import *
from outputVariableTypes import *
from outputVariableDescriptions import *

definitions = []
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerBodyMass   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerBodyMass',
    cParentClass=ParentClassCMarker,
    classDescription=r'A marker attached to the body mass; use this marker to apply a body-load (e.g. gravitational force).',
    classType=ClassTypeMarker,
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TIndex(ItemObject), destination=DestComp+DestParam,
            pythonName='bodyNumber',
            defaultValue=DVInvalidIndex,
            description=r'body number to which marker is attached to'),
        ItemFunctionDef('GetObjectNumber',
            implementation='return parameters.bodyNumber;'),
        ItemFunctionDef('SetObjectNumber',
            args='Index objectNumber, Index localIndex = 0',
            implementation='parameters.bodyNumber = objectNumber;'),
        ItemFunctionDef('GetNumberOfObjects',
            implementation='return 1;'),
        ItemTypes('Marker', ['Body', 'Object', 'BodyMass'],
            description=r'return marker type (for body treatment in computation)'),
        ItemFunctionDef('GetDimension',
            implementation='return 3;'),
        ItemFunctionDef('GetPosition',
            description='return position of marker at local position (0,0,0) of the body'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "BodyMass";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerBodyPosition   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerBodyPosition',
    cParentClass=ParentClassCMarker,
    classDescription=r"""A position body-marker attached to a local (body-fixed) position $\pLocB = [b_0,\; b_1,\; b_2]$ ($x$, $y$, and $z$ coordinates) of the body. It provides position information as well as the according derivatives (=velocity and derivative of position w.r.t. body coordinates). It can be used for connectors, joints or loads where position is required. If connectors also require orientation information, use a MarkerBodyRigid.""",
    classType=ClassTypeMarker,
    equations=r"""    The body position marker provides an interface to a object of type body 
    (\texttt{ObjectGround}, \texttt{ObjectMassPoint}, \texttt{ObjectRigidBody}, ...)
    and provides access to kinematic quantities such as \mybold{position} and \mybold{velocity} 
    and to the \mybold{position jacobian}, using a \texttt{localPosition} $\pLocB$ which is defined within the 
    local coordinates of the body ($b$).
    The kinematic quantities are computed according to the definition of output variables in the respective bodies.
    
    The position jacobian represents the derivative of the node position $\pv_\mathrm{n}$ with all nodal coordinates,


    $$
    \LU{0}{\Jm_\mathrm{pos}} = \frac{\partial \LU{0}{\pv_\mathrm{n}}}{\partial \qv_\mathrm{n}}
    $$

    and it is usually computed as the derivative of the (global) translational velocity w.r.t.\ velocity coordinates,


    $$
    \LU{0}{\Jm_\mathrm{pos}} = \frac{\partial \LU{0}{\vv_\mathrm{n}}}{\partial \dot \qv_\mathrm{n}}
    $$


    As an example of the \texttt{ObjectRigidBody2D}, see [](#sec-item-objectrigidbody2d), the position and velocity are computed as


    $$
    \LU{0}{\pv}\cConfig(\pLocB) = \LU{0}{\pRef}\cConfig + \LU{0}{\pRef}\cRef + \LU{0b}{\Rot}\pLocB \, ,
    $$



    $$
    \LU{0}{\vv}\cConfig(\pLocB) = \LU{0}{\dot\uv}\cConfig + \LU{0b}{\Rot}(\LU{b}{\tomega} \times \pLocB\cConfig) \, .
    $$

    Thus, the position jacobian for \texttt{ObjectRigidBody2D} reads


    $$
    \LU{0}{\Jm_\mathrm{pos}^{\mathrm{NodeRigidBody2D}}} = \mr{1}{0}{-\sin\theta_0 \LU{b}{b_0} - \cos\theta_0 \LU{b}{b_1}} 
          {0}{1}{\cos\theta_0 \LU{b}{b_0} - \sin\theta_0 \LU{b}{b_1}} 
          {0}{0}{0}
    $$

    <!-- -->
    For details, see the respective definition of the body and the C++ implementation.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TIndex(ItemObject), destination=DestComp+DestParam,
            pythonName='bodyNumber',
            defaultValue=DVInvalidIndex,
            description=r'body number to which marker is attached to'),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='localPosition',
            defaultValue=DVZeroVector3D,
            description=r"""$\pLocB$local body position of marker; e.g. local (body-fixed) position where force is applied to"""),
        ItemFunctionDef('GetObjectNumber',
            implementation='return parameters.bodyNumber;'),
        ItemFunctionDef('SetObjectNumber',
            args='Index objectNumber, Index localIndex = 0',
            implementation='parameters.bodyNumber = objectNumber;'),
        ItemFunctionDef('GetNumberOfObjects',
            implementation='return 1;'),
        ItemTypes('Marker', ['Body', 'Object', 'Position', 'JacobianDerivativeAvailable', 'JacobianDerivativeNonZero'],
            description=r'return marker type (for body treatment in computation)'),
        ItemFunctionDef('GetDimension',
            implementation='return 3;'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunctionDef('ComputeMarkerDataJacobianDerivative'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "BodyPosition";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerBodyRigid   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerBodyRigid',
    cParentClass=ParentClassCMarker,
    classDescription=r"""A rigid-body (position+orientation) body-marker attached to a local (body-fixed) position $\pLocB = [b_0,\; b_1,\; b_2]$ ($x$, $y$, and $z$ coordinates) of the body. It provides position and orientation (rotation), as well as the according derivatives. It can be used for most connectors, joints or loads where either position, position and orientation, or orientation are required.""",
    classType=ClassTypeMarker,
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TIndex(ItemObject), destination=DestComp+DestParam,
            pythonName='bodyNumber',
            defaultValue=DVInvalidIndex,
            description=r'body number to which marker is attached to'),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='localPosition',
            defaultValue=DVZeroVector3D,
            description=r"""$\pLocB$local body position of marker; e.g. local (body-fixed) position where force is applied to"""),
        ItemFunctionDef('GetObjectNumber',
            implementation='return parameters.bodyNumber;'),
        ItemFunctionDef('SetObjectNumber',
            args='Index objectNumber, Index localIndex = 0',
            implementation='parameters.bodyNumber = objectNumber;'),
        ItemFunctionDef('GetNumberOfObjects',
            implementation='return 1;'),
        ItemTypes('Marker', ['Body', 'Object', 'Position', 'Orientation', 'JacobianDerivativeAvailable', 'JacobianDerivativeNonZero'],
            description=r'return marker type (for body treatment in computation)'),
        ItemFunctionDef('GetDimension',
            implementation='return 3;'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetRotationMatrix',
            description='return configuration dependent rotation matrix of node; returns always a 3D Matrix'),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetAngularVelocityLocal',
            description='return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunctionDef('ComputeMarkerDataJacobianDerivative'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "BodyRigid";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerNodePosition   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerNodePosition',
    cParentClass=ParentClassCMarker,
    classDescription=r'A node-Marker attached to a position-based node. It can be used for connectors, joints or loads where position is required. If connectors also require orientation information, use a MarkerNodeRigid.',
    classType=ClassTypeMarker,
    equations=r"""    The node position marker provides an interface to a node which contains a position
    (\texttt{NodePoint}, \texttt{NodePoint2D}, \texttt{NodeRigidBodyEP}, \texttt{NodePointSlope}, ...)
    and accesses \mybold{position}, \mybold{velocity} and the \mybold{position jacobian}.
    The position and velocity are computed according to the definition of output variables in the respective nodes.
    
    The position jacobian represents the derivative of the node position $\pv_\mathrm{n}$ with all nodal coordinates,


    $$
    \LU{0}{\Jm_\mathrm{pos}} = \frac{\partial \LU{0}{\pv_\mathrm{n}}}{\partial \qv_\mathrm{n}}
    $$

    For details, see the respective definition of the node and the C++ implementation.
    
    In examplary case of a \texttt{NodeRigidBody2D},  see [](#sec-item-noderigidbody2d), its coordinates are 
    $\qv_\mathrm{n}=[q_0,\;q_1,\;\psi_0,\;]\tp$, where $q_0$ represents the $x$-displacement 
    and $q_1$ represents the $y$-displacement, such that the jacobian for the 3D position vector reads


    $$
    \LU{0}{\Jm_\mathrm{pos}^{\mathrm{NodeRigidBody2D}}} = \mr{1}{0}{0} {0}{1}{0} {0}{0}{0}
    $$

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'node number to which marker is attached to'),
        ItemFunctionDef('GetNodeNumber',
            implementation='return parameters.nodeNumber;',
            description='general access to node number'),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber = nodeNumber;'),
        ItemTypes('Marker', ['Node', 'Position', 'JacobianDerivativeAvailable'],
            description=r'return marker type (for node treatment in computation)'),
        ItemFunctionDef('GetDimension',
            implementation='return 3;'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunctionDef('ComputeMarkerDataJacobianDerivative'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "NodePosition";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerNodeRigid   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerNodeRigid',
    cParentClass=ParentClassCMarker,
    classDescription=r'A rigid-body (position+orientation) node-marker attached to a rigid-body node. It provides position and orientation (rotation), as well as the according derivatives. It can be used for most connectors, joints or loads where either position, position and orientation, or orientation are required.',
    classType=ClassTypeMarker,
    equations=r"""    The node rigid body marker provides an interface to a node which contains a position and an orientation
    (\texttt{NodeRigidBodyEP}, \texttt{NodeRigidBody2D}, ...)
    and provides access to kinematic quantities such as \mybold{position}, \mybold{velocity}, \mybold{orientation} (rotation matrix),
    \mybold{angular velocity}. It also provides the \mybold{position jacobian} and the \mybold{rotation jacobian}.
    The kinematic quantities are computed according to the definition of output variables in the respective nodes.
    
    The position jacobian represents the derivative of the node position $\pv_\mathrm{n}$ with all nodal coordinates,


    $$
    \LU{0}{\Jm_\mathrm{pos}} = \frac{\partial \LU{0}{\pv_\mathrm{n}}}{\partial \qv_\mathrm{n}}
    $$

    and it is usually computed as the derivative of the (global) translational velocity w.r.t.\ velocity coordinates,


    $$
    \LU{0}{\Jm_\mathrm{pos}} = \frac{\partial \LU{0}{\vv_\mathrm{n}}}{\partial \dot \qv_\mathrm{n}}
    $$

    The rotation jacobian is computed as the derivative of the (global) angular velocity w.r.t.\ velocity coordinates,


    $$
    \LU{0}{\Jm_\mathrm{rot}} = \frac{\partial \LU{0}{\tomega_\mathrm{n}}}{\partial \dot \qv_\mathrm{n}}
    $$

    This usually results in the velocity transformation matrix.
    For details, see the respective definition of the node and the C++ implementation.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'node number to which marker is attached to'),
        ItemFunctionDef('GetNodeNumber',
            implementation='return parameters.nodeNumber;',
            description='general access to node number'),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber = nodeNumber;'),
        ItemTypes('Marker', ['Node', 'Position', 'Orientation', 'JacobianDerivativeAvailable', 'JacobianDerivativeNonZero'],
            description=r'return marker type (for node treatment in computation)'),
        ItemFunctionDef('GetDimension',
            implementation='return 3;'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetRotationMatrix',
            description='return configuration dependent rotation matrix of node; returns always a 3D Matrix'),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetAngularVelocityLocal',
            description='return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunctionDef('ComputeMarkerDataJacobianDerivative'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "NodeRigid";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerNodeCoordinate   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerNodeCoordinate',
    cParentClass=ParentClassCMarker,
    classDescription=r"""A node-Marker attached to a ABRV:ODE2 coordinate of a node; this marker allows to connect a coordinate-based constraint or connector to a nodal coordinate (also NodeGround); for ABRV:ODE1 coordinates use \texttt{MarkerNodeODE1Coordinate}.""",
    classType=ClassTypeMarker,
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'node number to which marker is attached to'),
        ItemParameter(type=TIndex(minimum=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='coordinate',
            defaultValue=DVInvalidIndex,
            description=r'coordinate of node to which marker is attached to'),
        ItemFunctionDef('GetNodeNumber',
            implementation='return parameters.nodeNumber;'),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber = nodeNumber;'),
        ItemFunctionDef('GetCoordinateNumber',
            implementation='return parameters.coordinate;'),
        ItemTypes('Marker', ['Node', 'Coordinate', 'JacobianDerivativeAvailable'],
            description=r'return marker type (for node treatment in computation)'),
        ItemFunctionDef('GetDimension',
            implementation='return 1;'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunctionDef('ComputeMarkerDataJacobianDerivative'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "NodeCoordinate";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics',
            implementation=';'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerNodeCoordinates   +++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerNodeCoordinates',
    cParentClass=ParentClassCMarker,
    classDescription=r"""A node-Marker attached to all ABRV:ODE2 coordinates of a node. IN CONTRAST to MarkerNodeCoordinate, the marker coordinates INCLUDE the reference values! For ABRV:ODE1 coordinates use \texttt{MarkerNodeODE1Coordinates}.""",
    classType=ClassTypeMarker,
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'node number to which marker is attached to'),
        ItemFunctionDef('GetNodeNumber',
            implementation='return parameters.nodeNumber;'),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber = nodeNumber;'),
        ItemTypes('Marker', ['Node', 'Coordinates', 'JacobianDerivativeAvailable'],
            description=r'return marker type (for node treatment in computation)'),
        ItemFunctionDef('GetDimension',
            implementation='return cSystemData.GetCNodes()[parameters.nodeNumber]->GetNumberOfODE2Coordinates();'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunctionDef('ComputeMarkerDataJacobianDerivative'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "NodeCoordinates";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics',
            implementation=';'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerNodeODE1Coordinate   ++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerNodeODE1Coordinate',
    cParentClass=ParentClassCMarker,
    classDescription=r'A node-Marker attached to a ABRV:ODE1 coordinate of a node.',
    classType=ClassTypeMarker,
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'node number to which marker is attached to'),
        ItemParameter(type=TIndex(minimum=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='coordinate',
            defaultValue=DVInvalidIndex,
            description=r'coordinate of node to which marker is attached to'),
        ItemFunctionDef('GetNodeNumber',
            implementation='return parameters.nodeNumber;'),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber = nodeNumber;'),
        ItemFunctionDef('GetCoordinateNumber',
            implementation='return parameters.coordinate;'),
        ItemTypes('Marker', ['Node', 'Coordinate', 'ODE1', 'JacobianDerivativeAvailable'],
            description=r'return marker type (for node treatment in computation)'),
        ItemFunctionDef('GetDimension',
            implementation='return 1;'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunctionDef('ComputeMarkerDataJacobianDerivative'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "NodeODE1Coordinate";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=False,
            description=r'currently not available; set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics',
            implementation=';'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerNodeRotationCoordinate   ++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerNodeRotationCoordinate',
    cParentClass=ParentClassCMarker,
    classDescription=r'A node-Marker attached to a a node containing rotation; the Marker measures a rotation coordinate (Tait-Bryan angles) or angular velocities on the velocity level.',
    classType=ClassTypeMarker,
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'node number to which marker is attached to'),
        ItemParameter(type=TIndex(minimum=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='rotationCoordinate',
            defaultValue=DVInvalidIndex,
            description=r'rotation coordinate: 0=x, 1=y, 2=z'),
        ItemFunctionDef('GetNodeNumber',
            implementation='return parameters.nodeNumber;'),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber = nodeNumber;'),
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetRotationCoordinateNumber',
            implementation='return parameters.rotationCoordinate;',
            description=r'access to coordinate index'),
        ItemTypes('Marker', ['Node', 'Coordinate'],
            description=r'return marker type (for node treatment in computation)'),
        ItemFunctionDef('GetDimension',
            implementation='return 1;'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "NodeRotationCoordinate";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics',
            implementation=';'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerBodiesRelativeTranslationCoordinate   +++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerBodiesRelativeTranslationCoordinate',
    cParentClass=ParentClassCMarker,
    classDescription=r'A coordinate-based Marker attached to two rigid bodies or beams which computes the relative translation between the bodies according to the given axis. This marker can be used together with coordinate-based constraints and connectors (e.g., CoordinateSpringDamper and CoordinateConstraint). NOTE: it is assumed that the two bodies can only move along the given axis (e.g., constrained by a prismatic joint) -- otherwise results may be unexpected. NOTE: this approach is not compatible with FFRF-based flexible bodies and currently requires and intermediate rigid body.',
    classType=ClassTypeMarker,
    equations=r"""    The marker consists of two bodies, body $b_0$ and body $b_1$ with respective global marker positions $\LU{0}{\pv}_{m0}$ and $\LU{0}{\pv}_{m1}$,
    depending on local positions $\LU{m_0}{\pv}_0$ and $\LU{m_1}{\pv}_1$, 
    and marker orientations $\LU{0,m_0}{\Rot}_{m0}$ and $\LU{0,m_1}{\Rot}_{m1}$.
    The global axis is computed as 


    $$
    \LU{0}{\av}_0 = \LU{0,m_0}{\Rot}_{m0} \LU{m_0}{\av}_0
    $$

    The relative translation marker computes the relative translation from the equation


    $$
    t =  \LU{0}{\av}_0\tp (\pv_{m1} - \pv_{m0}) - x_\mathrm{off}
    $$

    The translational velocity, which may be used in coordinate spring-dampers or for velocity-level constraints, is computed as


    $$
    \dot t = \LU{0}{\av}_0\tp (\dot \pv_{m1} - \dot \pv_{m0}) + \LU{0}{\dot \av}_0\tp (\pv_{m1} - \pv_{m0})
    $$

    Jacobians are computed according to the relative translational velocity, ignoring the $\dot \av_0$ part.
    Using this approach, coordinate constraints can be added to mechanisms to purely add internal drives, not affecting global momenta.
    Furthermore, coupling to a relative rotation marker MarkerBodiesRelativeRotationCoordinate can be used to 
    create advanced mechanisms and gears.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TArrayIndex(ItemObject), destination=DestComp+DestParam,
            pythonName='bodyNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r"""$[b_0,b_1]\tp$list of body numbers for which relative coordinate is computed"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='localPosition0',
            defaultValue=DVZeroVector3D,
            description=r"""$\LU{m_0}{\pv}_0$local position on body 0; i.e. local (body-fixed) position where position is measured and force is applied to"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='localPosition1',
            defaultValue=DVZeroVector3D,
            description=r"""$\LU{m_1}{\pv}_1$local position on body 1; i.e. local (body-fixed) position where position is measured and force is applied to"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='axis0',
            defaultValue='Vector3D({1.,0.,0.})',
            description=r"""$\LU{m_0}{\av}_0$axis defined in body 0, along which the relative translation is measured"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='offset',
            defaultValue=0.,
            description=r"""$x_\mathrm{off}$translation offset [SI:m] subtracted from the translation; can be used to change the zero position"""),
        ItemFunctionDef('GetObjectNumber',
            implementation='return parameters.bodyNumbers[localIndex];'),
        ItemFunctionDef('SetObjectNumber',
            args='Index objectNumber, Index localIndex = 0',
            implementation='parameters.bodyNumbers[localIndex] = objectNumber;'),
        ItemFunctionDef('GetNumberOfObjects',
            implementation='return 2;'),
        ItemTypes('Marker', ['Body', 'Object', 'Coordinate', 'Position', 'Orientation'],
            description=r'return marker type (for body treatment in computation)'),
        ItemFunctionDef('GetDimension',
            implementation='return 1;'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetRotationMatrix',
            description='return configuration dependent rotation matrix of node; returns always a 3D Matrix'),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetAngularVelocityLocal',
            description='return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunctionDef('ComputeMarkerDataJacobianDerivative'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "BodiesRelativeTranslationCoordinate";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerBodiesRelativeRotationCoordinate   ++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerBodiesRelativeRotationCoordinate',
    cParentClass=ParentClassCMarker,
    classDescription=r'A coordinate-based Marker attached to two rigid bodies or beams which computes the relative rotation between the bodies according to the given axis; this marker can be used together with coordinate-based constraints and connectors (e.g., CoordinateSpringDamper and CoordinateConstraint). NOTE: it is assumed that the two bodies can only rotate about the given axis (e.g., constrained by a revolute joint) -- otherwise results may be unexpected. NOTE: this approach is not compatible with FFRF-based flexible bodies and currently requires and intermediate rigid body.',
    classType=ClassTypeMarker,
    equations=r"""    The marker consists of two bodies, body $b_0$ and body $b_1$ with respective global marker positions $\LU{0}{\pv}_{m0}$ and $\LU{0}{\pv}_{m1}$,
    depending on local positions $\LU{m_0}{\pv}_0$ and $\LU{m_1}{\pv}_1$, 
    and marker orientations $\LU{0,m_0}{\Rot}_{m0}$ and $\LU{0,m_1}{\Rot}_{m1}$.
    From the given axis \texttt{axis0}, we compute an orthonormal basis (orthonormal to axis0) relative to marker $m_0$,


    $$
    \LU{m_0,b}{\Rot}_b = \left[\LU{m_0}{\xv}_b, \LU{m_0}{\yv}_b, \LU{m_0}{\av}_0\right]
    $$

    The relative rotation marker computes the relative rotation according to


    $$
    \LU{m_0,m_1}{\Rot}_{rel} = \LU{m_0,0}{\Rot}_{m0} \LU{0,m_1}{\Rot}_{m1}
    $$

    This relative rotation, which represents a rotation about axis $\LU{m_0}{\av}_0$ is then transformed into the orthonormal basis,


    $$
    \LU{b}{\Rot}_{rel} = \LU{b,m_0}{\Rot}_b \LU{m_0,m_1}{\Rot}_{rel} \LU{m_0,b}{\Rot}_b
    $$

    and contains the desired rotation about the z-axis, which can be extracted as


    $$
    \varphi = \mathrm{atan2}(\LU{b}{\Rot}_{rel}[1,0],\LU{b}{\Rot}_{rel}[0,0]) - x_\mathrm{off}
    $$

    The global axis is computed as 


    $$
    \LU{0}{\av}_0 = \LU{0,m_0}{\Rot}_{m0} \LU{m_0}{\av}_0
    $$
    
    Using the angular velocities at the two bodies, $\LU{m_0}{\tomega_0}$ and  $\LU{m_1}{\tomega_1}$, the relative angular velocity, which may be used in coordinate spring-dampers or for velocity-level constraints, 
    is simply computed as


    $$
    \dot \varphi = \LU{0}{\av}_0\tp \left( \LU{0,m_1}{\Rot}_{m1} \LU{m_1}{\tomega_1} - 
                            \LU{0,m_0}{\Rot}_{m0} \LU{m_0}{\tomega_0} \right)
    $$

    Jacobians are computed according to the relative rotation velocity.
    Using this approach, coordinate constraints can be added to mechanisms to purely add internal drives, not affecting global momenta.
    Furthermore, coupling to a relative translation can be used to create advanced mechanisms and gears.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TArrayIndex(ItemObject), destination=DestComp+DestParam,
            pythonName='bodyNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r"""$[b_0,b_1]\tp$list of body numbers for which relative coordinate is computed"""),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r"""node number of NodeGenericData with 1 coordinate which contains previous angle for continuation of angles (initialize accordingly if needed); if node is not supplied, angles will have jump outside $\pm \pi$"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='localPosition0',
            defaultValue=DVZeroVector3D,
            description=r"""$\LU{m_0}{\pv}_0$local position on body 0; i.e. local (body-fixed) position where position is measured and force is applied to"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='localPosition1',
            defaultValue=DVZeroVector3D,
            description=r"""$\LU{m_1}{\pv}_1$local position on body 1; i.e. local (body-fixed) position where position is measured and force is applied to"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='axis0',
            defaultValue='Vector3D({1.,0.,0.})',
            description=r"""$\LU{m_0}{\av}_0$axis defined in body 0, along which the relative rotation is measured"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='offset',
            defaultValue=0.,
            description=r"""$x_\mathrm{off}$rotation offset [SI:1] subtracted from the measured rotation; can be used to change the zero rotation"""),
        ItemFunctionDef('GetObjectNumber',
            implementation='return parameters.bodyNumbers[localIndex];'),
        ItemFunctionDef('SetObjectNumber',
            args='Index objectNumber, Index localIndex = 0',
            implementation='parameters.bodyNumbers[localIndex] = objectNumber;'),
        ItemFunctionDef('GetNumberOfObjects',
            implementation='return 2;'),
        ItemFunctionDef('GetNodeNumber',
            implementation='return parameters.nodeNumber;'),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber = nodeNumber;'),
        ItemTypes('Marker', ['Body', 'Object', 'Node', 'Coordinate', 'Position', 'Orientation', 'HasPostNewton'],
            description=r'return marker type (for body treatment in computation)'),
        ItemFunctionDef('GetDimension',
            implementation='return 1;'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetRotationMatrix',
            description='return configuration dependent rotation matrix of node; returns always a 3D Matrix'),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetAngularVelocityLocal',
            description='return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunctionDef('ComputeMarkerDataJacobianDerivative'),
        ItemFunctionDef('PostNewtonStep'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "BodiesRelativeTranslationCoordinate";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerSuperElementPosition   ++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerSuperElementPosition',
    cParentClass=ParentClassCMarker,
    classDescription=r'A position marker attached to a SuperElement, such as ObjectFFRF, ObjectGenericODE2 and ObjectFFRFreducedOrder (for which it is in its current implementation inefficient for large number of meshNodeNumbers). The marker acts on the mesh (interface) nodes, not on the underlying nodes of the object.',
    classType=ClassTypeMarker,
    equations=r"""    {\bf Definition of marker quantities}:

    | intermediate variables | symbol | description |
    |---|---|---|
    | number of mesh nodes | $n_m$ | size of \texttt{meshNodeNumbers} and \texttt{weightingFactors} which are marked; this must not be the number of mesh nodes in the marked object |
    | mesh node number | $i = k_i$ | abbreviation |
    | mesh node points | $\LU{0}{\pv}_{i}$ | position of mesh node $k_i$ in object $n_b$ |
    | mesh node velocities | $\LU{0}{\vv}_{i}$ | velocity of mesh node $i$ in object $n_b$ |
    | marker position | $\LU{0}{\pv}_{m} = \sum_i w_i \cdot \LU{0}{\pv_i}$ | current global position which is provided by marker |
    | marker velocity | $\LU{0}{\vv}_{m} = \sum_i w_i \cdot \LU{0}{\vv_i}$ | current global velocity which is provided by marker |

    <!-- -->
    \vspace{6pt}
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->

    #### Marker quantities

    The marker provides a 'position' jacobian, which is the derivative of the marker velocity w.r.t.\ the 
    object velocity coordinates $\dot \qv_{n_b}$,


    $$
    \Jm_{m,pos} = \frac{\partial \LU{0}{\vv}_{m}}{\partial \dot \qv_{n_b}}
          = \sum_i w_i \cdot \Jm_{i,pos}
    $$

    in which $\Jm_{i,pos}$ denotes the position jacobian of mesh node $i$,


    $$
    \Jm_{i,pos} = \frac{\partial \LU{0}{\vv}_{i}}{\partial \dot \qv_{n_b}}
    $$

    The jacobian $\Jm_{i,pos}$ usually contains mostly zeros for \texttt{ObjectGenericODE2}, because the jacobian only affects one single node.
    In \texttt{ObjectFFRFreducedOrder}, the jacobian may affect all reduced coordinates.

    Note that $\Jm_{m,pos}$ is actually computed by the
    \texttt{ObjectSuperElement} within the function \texttt{GetAccessFunctionSuperElement}.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainMarker,
    miniExample=r"""    #set up a mechanical system with two nodes; it has the structure: |~~M0~~M1
    #==>further examples see objectGenericODE2Test.py, objectFFRFTest2.py, etc.
    nMass0 = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,0]))
    nMass1 = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0]))
    mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [1,0,0]))

    mass = 0.5 * np.eye(3)      #mass of nodes
    stif = 5000 * np.eye(3)     #stiffness of nodes
    damp = 50 * np.eye(3)      #damping of nodes
    Z = 0. * np.eye(3)          #matrix with zeros
    #build mass, stiffness and damping matrices (:
    M = np.block([[mass,         0.*np.eye(3)],
                  [0.*np.eye(3), mass        ] ])
    K = np.block([[2*stif, -stif],
                  [ -stif,  stif] ])
    D = np.block([[2*damp, -damp],
                  [ -damp,  damp] ])
    
    oGenericODE2 = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[nMass0,nMass1], 
                                                   massMatrix=M, 
                                                   stiffnessMatrix=K,
                                                   dampingMatrix=D))
    
    #EXAMPLE for single node marker on super element body, mesh node 1; compare results to ObjectGenericODE2 example!!! 
    mSuperElement = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oGenericODE2, meshNodeNumbers=[1], weightingFactors=[1]))
    mbs.AddLoad(Force(markerNumber = mSuperElement, loadVector = [10, 0, 0])) 

    #assemble and solve system for default parameters
    mbs.Assemble()
    
    mbs.SolveDynamic(solverType = exu.DynamicSolverType.TrapezoidalIndex2)

    #check result at default integration time
    exu.sys['testResult'] = mbs.GetNodeOutput(nMass1, exu.OutputVariableType.Position)[0]
""",
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TIndex(ItemObject), destination=DestComp+DestParam,
            pythonName='bodyNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n_b$body number to which marker is attached to'),
        ItemParameter(type=TArrayIndex, destination=DestComp+DestParam,
            pythonName='meshNodeNumbers',
            defaultValue='ArrayIndex()',
            description=r"""$[k_0,\,\ldots,\,k_{n_m-1}]\tp$a list of $n_m$ mesh node numbers of superelement (=interface nodes) which are used to compute the body-fixed marker position; the related nodes must provide 3D position information, such as NodePoint, NodePoint2D, NodeRigidBody[..]; in order to retrieve the global node number, the generic body needs to convert local into global node numbers"""),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='weightingFactors',
            defaultValue='Vector()',
            description=r"""$[w_{0},\,\ldots,\,w_{n_m-1}]\tp$a list of $n_m$ weighting factors per node to compute the final local position; the sum of these weights shall be 1, such that a summation of all nodal positions times weights gives the average position of the marker"""),
        ItemFunctionDef('GetObjectNumber',
            implementation='return parameters.bodyNumber;'),
        ItemFunctionDef('SetObjectNumber',
            args='Index objectNumber, Index localIndex = 0',
            implementation='parameters.bodyNumber = objectNumber;'),
        ItemFunctionDef('GetNumberOfObjects',
            implementation='return 1;'),
        ItemTypes('Marker', ['Body', 'Object', 'Position', 'SuperElement'],
            description=r'return marker type (for node treatment in computation)'),
        ItemFunctionDef('GetDimension',
            implementation='return 3;'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "SuperElementPosition";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=TBool, destination=DestVisu,
            pythonName='showMarkerNodes',
            defaultValue=True,
            description=r'set true, if all nodes are shown (similar to marker, but with less intensity)'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerSuperElementRigid   +++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerSuperElementRigid',
    cParentClass=ParentClassCMarker,
    classDescription=r'A position and orientation (rigid-body) marker attached to a SuperElement, such as ObjectFFRF, ObjectGenericODE2 and ObjectFFRFreducedOrder (for which it may be inefficient). The marker acts on the mesh nodes, not on the underlying nodes of the object. Note that in contrast to the MarkerSuperElementPosition, this marker needs a set of interface nodes which are not aligned at one line, such that these node points can represent a rigid body motion. Note that definitions of marker positions are slightly different from MarkerSuperElementPosition.',
    classType=ClassTypeMarker,
    equations=r"""    {\bf Definition of marker quantities}:
    <!--\rowTable{marker velocity}{$\LU{0}{\vv}_{m} = \LU{0}{\dot \pv}_r + \LU{0r}{\Rot} \LU{r}{\tilde \tomega_r} \LU{r}{\pv_{0,ref}} + -->
    <!--\LU{0r}{\Rot} \left(\sum_i (w_i \cdot \LU{r}{\vv^{(i)}}) + \LU{r}{\tilde \tomega_r} \sum_i (w_i \cdot \LU{r}{\uv^{(i)}}) \right)$} -->
    <!--
    {current global velocity which is provided by marker}
    \rowTable{marker rotation matrix}{$\LU{0r}{\Rot}_{m} = \LU{0r}{\Rot} \mr{1}{-\theta_2}{\theta_1}{\theta_2}{1}{-\theta_0}{-\theta_1}{\theta_0}{1}$}{current rotation matrix, which transforms the local marker coordinates and adds the rigid body transformation of floating frames $\LU{0r}{\Rot}$; only valid for small (linearized rotations)!}
    -->

    | intermediate variables | symbol | description |
    |---|---|---|
    | number of mesh nodes | $n_m$ | size of \texttt{meshNodeNumbers} and \texttt{weightingFactors} which are marked; this must not be the number of mesh nodes in the marked object |
    | mesh node number | $i = k_i$ | abbreviation, runs over all marker mesh nodes |
    | mesh node local displacement | $\LU{r}{\uv^{(i)}}$ | current local (within reference frame $r$) displacement of mesh node $k_i$ in object $n_b$ |
    | mesh node local position | $\LU{r}{\pv^{(i)}} = \LU{r}{\xv^{(i)}\cRef} + \LU{r}{\uv^{(i)}}$ | current local (within reference frame $r$, which is the body frame $b$ ,e.g., in \texttt{ObjectFFRFreducedOrder}) position of mesh node $k_i$ in object $n_b$ |
    | mesh node local reference position | $\LU{r}{\xv^{(i)}\cRef}$ | local (within reference frame $r$) reference position of mesh node $k_i$ in object $n_b$, see e.g.\ \texttt{ObjectFFRFreducedOrder} |
    | averaged local reference position | $\LU{r}{\xv^\mathrm{avg}\cRef} = \sum_i w_i \LU{r}{\xv^{(i)}\cRef}$ | midpoint reference position of marker; averaged local reference positions of all mesh nodes $k_i$, using weighting for averaging; may not coincide with center point of your idealized joint surface (e.g., midpoint of cylinder), see [](#fig-markersuperelementrigid-sketch) |
    | marker centered mesh node local reference position | $\LU{r}{\pv^{(i)}\cRef} = \LU{r}{\xv^{(i)}\cRef}- \LU{r}{\xv^\mathrm{avg}\cRef}$ | local reference position of mesh node $k_i$ relative to the center position of marker |
    | mesh node local velocity | $\LU{r}{\vv^{(i)}}$ | current local (within reference frame $r$) velocity of mesh node $k_i$ in object $n_b$ |
    | super element reference point | $\LU{0}{\pv}_r$ ($=\LU{0}{\pv}\indt$ in \texttt{ObjectFFRFreduced- Order}) | current position (origin) of super element's floating frame (r), which is zero, if the object does not provide a reference frame (such as GenericODE2) |
    | super element rotation matrix | $\LU{0r}{\Rot}$ | current rigid body transformation matrix of super element's floating frame (r), which is the identity matrix, if the object does not provide a reference frame (such as GenericODE2) |
    | super element angular velocity | $\LU{r}{\tomega_r}$ | current local angular velocity of super element's floating frame (r), which is zero, if the object does not provide a reference frame (such as GenericODE2) |
    | marker position | $\LU{0}{\pv}_{m} \!=\! \LU{0}{\pv}_r + \LU{0r}{\Rot} \left(\LU{r}{\ov\cRef}\! +\! \sum_i w_i \cdot \LU{r}{\pv^{(i)}} \right)$ | current global position which is provided by marker; note offset $\LU{r}{\ov\cRef}$ added, if used as a correction of marker mesh nodes |
    | marker velocity | $\LU{0}{\vv}_{m} = \LU{0}{\dot \pv}_r $ $+ \LU{0r}{\Rot} \left( \LU{r}{\tilde \tomega_r} \left(\LU{r}{\ov\cRef}\! +\! \sum_i w_i \cdot \LU{r}{\pv^{(i)}} \right) + \right.$ $\left. \sum_i (w_i \cdot \LU{r}{\dot \uv^{(i)}}) \right)$ | current global velocity which is provided by marker |
    | marker rotation matrix | $\LU{0r}{\Rot}_{m} = \LU{0r}{\Rot} \cdot \mathbf{exp}(\LU{r}{\ttheta}_{m})$ | current rotation matrix, which transforms the local marker coordinates and adds the rigid body transformation of floating frames $\LU{0r}{\Rot}$; uses exponential map for SO3, assumes that $\ttheta$ represents a rotation vector |
    | marker local rotation | $\LU{r}{\ttheta}_{m}$ | current local linearized rotations (rotation vector); for the computation, see below for the standard and alternative approach |
    | marker local angular velocity | $\LU{r}{\tomega}_{m}$ | local angular velocity due to mesh node velocity only; for the computation, see below for the standard and alternative approach |
    | marker global angular velocity | $\LU{0}{\tomega}_{m} = \LU{0}{\tomega_{r}} + \LU{0r}{\Rot} \LU{r}{\tomega}_{m}$ | current global angular velocity |

    <!-- -->
    \vspace{6pt}
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->

    #### Marker background

    The marker allows to realize a multi-point constraint (assuming that the marker is used in a joint constraint), 
    connecting to averaged nodal displacements and rotations (also known as RBE3 in NASTRAN), see e.g.\ [CITE:HeirmanDesmet2010]. 
    However, using Craig-Bampton RBE2 modes, will create RBE2 multi-point constraints for \texttt{ObjectFFRFreducedOrder} objects.

    For more information on the various quantities and their coordinate systems, see table above and [](#fig-markersuperelementrigid-sketch).
    <!--++++++++++++++++++++++++ -->
    

    (fig-markersuperelementrigid-sketch)=
    ```{figure} /docs/figures/MarkerSuperElementRigid.png
    :width: 400

    Sketch of marker nodes, exemplary node $i$, reference coordinates and marker coordinate system; note the difference of the center of the marker 'surface' (rectangle) marked with the red cross, and the averaged of the averaged local reference position.
    ```

    <!--
    ++++++++++++++++++++++++
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->

    #### Marker quantities

    The marker provides a 'position' jacobian, which is the derivative of the global marker velocity w.r.t.\ the 
    object velocity coordinates $\dot \qv_{n_b}$,


    $$
    \LU{0}{\Jm_{m,pos}} = \frac{\partial \LU{0}{\vv}_{m}}{\dot \qv_{n_b}} \, .
    $$

    In case of \texttt{ObjectGenericODE2}, assuming pure displacement based nodes,
    the jacobian will consist of zeros and unit matrices $\Im$ ,


    $$
    \LU{0}{\Jm_{m,pos}^{GenericODE2}} = \frac{\partial \LU{0}{\vv}_{m}}{\dot \qv_{n_b}} 
          = \left[ \Null,\; \ldots,\; \Null,\; w_0 \Im,\; \Null,\; \ldots,\; \Null,\; w_1 \Im,\; \Null,\; \ldots,\; \Null \right]\, ,
    $$

    in which the $\Im$ matrices are placed at the according indices of marker nodes.

    In case of \texttt{ObjectFFRFreducedOrder}, this jacobian is computed as weighted sum 
    of the position jacobians, see \texttt{ObjectFFRFreducedOrder},


    $$
    \LU{0}{\Jm_{m,pos}^{FFRFreduced}} = \frac{\partial \LU{0}{\vv}_{m}}{\dot \qv_{n_b}}
          = \sum_i w_i \LU{0}{\Jm^{(i)}_\mathrm{pos}}
          = \left[\Im, \; -\LU{0r}{\Rot} \left(\LU{r}{\ov\cRef} + \sum_i \LU{r}{\pv^{(i)}} \right) \LU{r}{\Gm},\;
                  \sum_i w_i \LU{0r}{\Rot} \vr{\LU{r}{\tPsi_{r=3i}\tp}}{\LU{r}{\tPsi_{r=3i+1}\tp}}{\LU{r}{\tPsi_{r=3i+2}\tp}} \right] \, .
    $$

    <!--
    \sum_i w_i \Im = \Im !!!
    \be
     \LU{0}{\Jm_\mathrm{pos}^{(i)}} = \frac{\partial \LU{0}{\pv^{(i)}}}{\partial [\qv\indt, \;\ttheta, \;\tzeta]}
     = \left[\Im, \; -\LU{0b}{\Rot} \left(\LU{b}{\tilde\uv\indf^{(i)}} + \LU{b}{\tilde\xv^{(i)}\cRef} \right) \LU{b}{\Gm},\;
             \LU{0b}{\Rot} \vr{\LU{b}{\tPsi_{r=3i}\tp}}{\LU{b}{\tPsi_{r=3i+1}\tp}}{\LU{b}{\tPsi_{r=3i+2}\tp}}\right] \eqComma
    \ee
    -->
    In \texttt{ObjectFFRFreducedOrder}, the jacobian usually affects all reduced coordinates.
    
    <!--++++++++++++++++++++++++++++++++++++++++++++ -->

    #### Standard approach for computation of rotation (\texttt{useAlternativeApproach = False})

    <!-- -->
    As compared to \texttt{MarkerSuperElementPosition}, \texttt{MarkerSuperElementRigid} also links the marker to the orientation of 
    the set of nodes provided. For this reason, the check performed in \texttt{mbs.assemble()} will take care that the nodes are capable
    to describe rotations.
    The first approach, here called as a standard, follows the idea that displacements contribute to rotation are weighted by their quadratic distance, 
    cf.\ [CITE:HeirmanDesmet2010], and gives the (small rotation) rotation vector


    $$
    \LU{r}{\ttheta}_{m} = \frac{\sum_i w_i \LU{r}{\pv_{ref}^{(i)}} \times \LU{r}{\uv^{(i)}}}{\sum_i w_i |\LU{r}{\pv_{ref}^{(i)}}|^2}
    $$

    Note that $\pv_{ref}^{(i)}$ is not the reference position in the \texttt{ObjectFFRFreducedOrder} object, but it is relative to the midpoint reference position
    all marker nodes, given in $\LU{r}{\xv^\mathrm{avg}\cRef}$.
    <!-- -->
    Accordingly, the marker local angular velocity can be calculated as


    $$
    \LU{r}{\tomega}_{m} = \LU{r}{\dot \ttheta}_{m} = \frac{\sum_i w_i \LU{r}{\tilde \pv_{ref}^{(i)}} \LU{r}{\vv_i}}{\sum_i w_i |\LU{r}{\pv_{ref}^{(i)}}|^2}
    $$

    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    The marker also provides a `rotation' jacobian, which is the derivative of the marker angular velocity $\LU{0}{\tomega}_{m}$ w.r.t.\ the 
    object velocity coordinates $\dot \qv_{n_b}$,


    $$
    \LU{0}{\Jm_{m,rot}} = \frac{\partial \LU{0}{\tomega}_{m}}{\partial \dot \qv_{n_b}}
                      = \frac{\partial \LU{0r}{\Rot}(\LU{r}{\tomega_{r}} + \LU{r}{\tomega}_{m})}{\partial \dot \qv_{n_b}}
                      = \LU{0r}{\Rot} \left(\frac{\partial \LU{r}{\tomega}_{r}}{\partial \dot \qv_{n_b}} + 
                                       \frac{\sum_i w_i \LU{r}{\tilde \pv_{ref}^{(i)}} \LU{r}{\Jm_{pos}^{(i)}}}{\sum_i w_i |\LU{r}{\pv_{ref}^{(i)}}|^2} \right)
    $$

    In case of \texttt{ObjectFFRFreducedOrder}, this jacobian is computed as


    $$
    \LU{0}{\Jm_{m,rot}^{FFRFreduced}} = \left[\Null,\; \LU{0r}{\Rot} \LU{r}{\Gm_{local}},\; 
                                                    \LU{0r}{\Rot} \frac{\sum_i w_i \LU{r}{\tilde \pv_{ref}^{(i)}} \LU{r}{\Jm_{pos,f}^{(i)}}}{\sum_i w_i |\LU{r}{\pv_{ref}^{(i)}}|^2} \right]
    $$ (eq-markersuperelementrigid-jacrotstandard)

    in which you should know that
    \bi
      \item we used $\frac{\partial \LU{r}{\tomega_{r}} }{\partial \dot \ttheta_r} = \LU{r}{\Gm_{local}}$, 
      \item $\ttheta_{r}$ represent the rotation parameters for the rigid body node of \texttt{ObjectFFRFreducedOrder},
      \item $\LU{r}{\Jm_{pos,f}^{(i)}}$ is the {\bf local} jacobian, which only includes the flexible part of the local 
            jacobian for a single mesh node, $\LU{r}{\Jm_{pos}^{(i)}}$ (note the small $r$ on the upper left), 
            as defined in \texttt{ObjectFFRFreducedOrder}.
     \ei
    For further quantities also consult the according description in \texttt{ObjectFFRFreducedOrder}.
    \vspace{6pt}\\
    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->

    #### Alternative computation of rotation (\texttt{useAlternativeApproach = True})

    Note that this approach is {\bf still under development} and needs further validation. 
    However, tests show that this model is superior to the standard approach, as it improves the averaging of motion w.r.t.\ rotations
    at the marker nodes.

    In the alternative approach, the weighting matrix $\Wm$ 
    has the interpretation of an inertia tensor built from nodes using weights equal to node masses.
    In such an interpretation, the 'local angular momentum' w.r.t.\ the marker (averaged) position can be computed as 


    $$
    \Wm \LU{r}{\tomega}_{m} = \sum_i w_i \LU{r}{\tilde \pv_{ref}^{(i)}} \left(\LU{r}{\vv^{(i)}} - \LU{r}{\vv^\mathrm{avg}}\right)= 
           -\sum_i  \left( w_i \LU{r}{\tilde \pv_{ref}^{(i)}} \LU{r}{\tilde \pv_{ref}^{(i)}} \right) \LU{r}{\tomega}_{m}
    $$ (eq-markersuperelementrigid-omegaandwm)

    which implicitly defines the weighting matrix $\Wm$, which must be invertable (but it is only a $3 \times 3$ matrix!),


    $$
    \Wm = -\sum_i  w_i \LU{r}{\tilde \pv_{ref}^{(i)}} \LU{r}{\tilde \pv_{ref}^{(i)}}
    $$

    Furthermore, we need to introduce the averaged velocity of the marker averaged reference position, using $\LU{r}{\dot \uv^{(i)}} = \LU{r}{\vv^{(i)}}$, which is defined as


    $$
    \LU{r}{\vv^\mathrm{avg}} = \sum_i  w_i \LU{r}{\vv^{(i)}} \, ,
    $$

    similar to the averaged local reference position $\LU{r}{\xv^\mathrm{avg}\cRef}$ given in the table above, see also [](#fig-markersuperelementrigid-sketch).

    In the alternative approach, thus the marker local rotations read


    $$
    \LU{r}{\ttheta}_{m,alt} = \Wm^{-1} \sum_i w_i \LU{r}{\tilde \pv_{ref}^{(i)}} \left( \LU{r}{\uv^{(i)}} - \LU{r}{\xv^\mathrm{avg}\cRef} \right) \, ,
    $$

    and the marker local angular velocity is defined as


    $$
    \LU{r}{\tomega}_{m,alt} = \Wm^{-1} \sum_i w_i \LU{r}{\tilde \pv_{ref}^{(i)}} \left( \LU{r}{\vv^{(i)}} - \LU{r}{\vv^\mathrm{avg}} \right) \, .
    $$

    Note that, the average velocity $\LU{r}{\vv^\mathrm{avg}}$ would cancel out in a symmetric mesh, but would cause spurious 
    angular velocities in unsymmetric (w.r.t.\ the axis of rotation) distribition of mesh nodes. 
    This could even lead to spurious rotations or angular velocities in pure translatoric motion.

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    In the alternative mode, the Jacobian for the rotation / angular velocity is defined as


    $$
    \LU{0}{\Jm_{m,rot,alt}} = \frac{\partial \LU{0}{\tomega}_{m}}{\partial \dot \qv_{n_b}}
                      = \frac{\partial \LU{0r}{\Rot}(\LU{r}{\tomega_{r}} + \LU{r}{\tomega}_{m})}{\partial \dot \qv_{n_b}}
                      = \LU{0r}{\Rot} \left(\frac{\partial \LU{r}{\tomega}_{r}}{\partial \dot \qv_{n_b}}  + 
                                            \Wm^{-1} \sum_i w_i \LU{r}{\tilde \pv_{ref}^{(i)}} \LU{r}{\Jm_{pos}^{(i)}}\right)
    $$

    In case of \texttt{ObjectFFRFreducedOrder}, this jacobian is computed as


    $$
    \LU{0}{\Jm_{m,rot,alt}^{FFRFreduced}} = \left[\Null,\; \LU{0r}{\Rot} \LU{r}{\Gm_{local}},\; 
                                                    \LU{0r}{\Rot} \Wm^{-1} \sum_i w_i \LU{r}{\tilde \pv_{ref}^{(i)}} \LU{r}{\Jm_{pos,f}^{(i)}} \right]
    $$

    see also the descriptions given after {eq}`eq-markersuperelementrigid-jacrotstandard` in the 'standard' approach.
    <!-- -->
    \vspace{12pt}\\
    \noindent {\bf EXAMPLE for marker on body 4, mesh nodes 10,11,12,13}:\vspace{6pt}\\
    \texttt{MarkerSuperElementRigid(bodyNumber = 4, meshNodeNumber = [10, 11, 12, 13], weightingFactors = [0.25, 0.25, 0.25, 0.25], referencePosition=[0,0,0])}
    \vspace{12pt}\\
    \noindent For detailed examples, see \texttt{TestModels}.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TIndex(ItemObject), destination=DestComp+DestParam,
            pythonName='bodyNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n_b$body number to which marker is attached to'),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='offset',
            defaultValue=DVZeroVector3D,
            description=r"""$\LU{r}{\ov_{ref}}$local marker SuperElement reference position offset used to correct the center point of the marker, which is computed from the weighted average of reference node positions (which may have some offset to the desired joint position). Note that this offset shall be small and larger offsets can cause instability in simulation models (better to have symmetric meshes at joints)."""),
        ItemParameter(type=TArrayIndex, destination=DestComp+DestParam,
            pythonName='meshNodeNumbers',
            defaultValue='ArrayIndex()',
            description=r"""$[k_0,\,\ldots,\,k_{n_m-1}]\tp$a list of $n_m$ mesh node numbers of superelement (=interface nodes) which are used to compute the body-fixed marker position and orientation; the related nodes must provide 3D position information, such as NodePoint, NodePoint2D, NodeRigidBody[..]; in order to retrieve the global node number, the generic body needs to convert local into global node numbers"""),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='weightingFactors',
            defaultValue='Vector()',
            description=r"""$[w_{0},\,\ldots,\,w_{n_m-1}]\tp$a list of $n_m$ weighting factors per node to compute the final local position and orientation; these factors could be based on surface integrals of the constrained mesh faces"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='useAlternativeApproach',
            defaultValue=True,
            description=r'this flag switches between two versions for the computation of the rotation and angular velocity of the marker; alternative approach uses skew symmetric matrix of reference position; follows the inertia concept'),
        ItemParameter(type=TIndex, destination=DestComp+DestParam,
            pythonName='rotationsExponentialMap',
            defaultValue=2,
            description=r'Experimental flag (2 is the correct value and will be used in future, removing this flag): This value switches different behavior for computation of rotations and angular velocities: 0 uses linearized rotations and angular velocities, 1 uses the exponential map for rotations but linear angular velocities, 2 uses the exponential map for rotations and the according tangent map for angular velocities'),
        ItemFunctionDef('GetObjectNumber',
            implementation='return parameters.bodyNumber;'),
        ItemFunctionDef('SetObjectNumber',
            args='Index objectNumber, Index localIndex = 0',
            implementation='parameters.bodyNumber = objectNumber;'),
        ItemFunctionDef('GetNumberOfObjects',
            implementation='return 1;'),
        ItemTypes('Marker', ['Body', 'Object', 'Position', 'Orientation', 'SuperElement'],
            description=r'return marker type (for node treatment in computation)'),
        ItemFunctionDef('GetDimension',
            implementation='return 3;'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetRotationMatrix',
            description='return configuration dependent rotation matrix of node; returns always a 3D Matrix'),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetAngularVelocityLocal',
            description='return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "SuperElementRigid";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetFloatingFrameNodeData',
            args='const CSystemData& cSystemData, Vector3D& framePosition, Matrix3D& frameRotationMatrix, Vector3D& frameVelocity, Vector3D& frameAngularVelocityLocal, ConfigurationType configuration = ConfigurationType::Current',
            description=r'return parameters of underlying floating frame node (or default values for case that no frame exists)'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetWeightedRotations',
            args='const CSystemData& cSystemData, Vector3D& weightedRotations, ConfigurationType configuration = ConfigurationType::Current',
            description=r'return weighted (linearized) rotation from local mesh displacements'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetWeightedAngularVelocity',
            args='const CSystemData& cSystemData, Vector3D& weightedAngularVelocity, ConfigurationType configuration = ConfigurationType::Current',
            description=r'return weighted angular velocity from local mesh velocities'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=TBool, destination=DestVisu,
            pythonName='showMarkerNodes',
            defaultValue=True,
            description=r'set true, if all nodes are shown (similar to marker, but with less intensity)'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerKinematicTreeRigid   ++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerKinematicTreeRigid',
    cParentClass=ParentClassCMarker,
    classDescription=r'A position and orientation (rigid-body) marker attached to a kinematic tree. The marker is attached to the ObjectKinematicTree object and additionally needs a link number as well as a local position, similar to the SensorKinematicTree. The marker allows to attach loads (LoadForceVector and LoadTorqueVector) at arbitrary links or position. It also allows to attach connectors (e.g., spring dampers or actuators) to the kinematic tree. Finally, joint constraints can be attached, which allows for realization of closed loop structures. NOTE, however, that it is less efficient to attach many markers to a kinematic tree, therefor for forces or joint control use the structures available in kinematic tree whenever possible.',
    classType=ClassTypeMarker,
    equations=r"""    <!--
        {\bf Definition of marker quantities}:
        \startTable{intermediate variables}{symbol}{description}
        \rowTable{marker position}{$\LU{0}{\pv}_{m} \!=\! \LU{0}{\pv}_r + \LU{0r}{\Rot} \left(\LU{r}{\ov\cRef}\! +\! \sum_i w_i \cdot \LU{r}{\pv^{(i)}} \right)$}
                 {current global position which is provided by marker; note offset $\LU{r}{\ov\cRef}$ added, if used as a correction of marker mesh nodes}
        \rowTable{marker velocity}{$\LU{0}{\vv}_{m} = \LU{0}{\dot \pv}_r + \LU{0r}{\Rot} \left( \LU{r}{\tilde \tomega_r} \sum_i (w_i \cdot \LU{r}{\pv^{(i)}}) + \right.
    -->
    <!--             \left. \sum_i (w_i \cdot \LU{r}{\dot \uv^{(i)}}) \right)$} -->
    <!--
                 {current global velocity which is provided by marker}
        \rowTable{marker rotation matrix}{$\LU{0r}{\Rot}_{m} = \LU{0r}{\Rot} \cdot \mathbf{exp}(\LU{r}{\ttheta}_{m})$}{current rotation matrix, which transforms the local marker coordinates and adds the rigid body transformation of floating frames $\LU{0r}{\Rot}$; uses exponential map for SO3, assumes that $\ttheta$ represents a rotation vector}
        \rowTable{marker local rotation}{$\LU{r}{\ttheta}_{m}$}{current local linearized rotations (rotation vector); for the computation, see below for the standard and alternative approach}
    
        \rowTable{marker local angular velocity}{$\LU{r}{\tomega}_{m}$}{local angular velocity due to mesh node velocity only; for the computation, see below for the standard and alternative approach}
        \rowTable{marker global angular velocity}{$\LU{0}{\tomega}_{m} = \LU{0}{\tomega_{r}} + \LU{0r}{\Rot} \LU{r}{\tomega}_{m}$}{current global angular velocity}
        \finishTable
    
        \vspace{6pt}
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->

    #### Marker quantities

    More information will be added later. The marker computes jacobians according to \texttt{Jacobian} in \texttt{class Robot}.
    %%RSTCOMPATIBLE
    <!--
        The marker provides a 'position' jacobian, which is the derivative of the global marker velocity w.r.t.\ the
        object velocity coordinates $\dot \qv_{n_b}$,
        \be
          \LU{0}{\Jm_{m,pos}} = \frac{\partial \LU{0}{\vv}_{m}}{\dot \qv_{n_b}} \eqDot
        \ee
        In case of \texttt{ObjectGenericODE2}, assuming pure displacement based nodes,
        the jacobian will consist of zeros and unit matrices $\Im$ ,
        \be
          \LU{0}{\Jm_{m,pos}^{GenericODE2}} = \frac{\partial \LU{0}{\vv}_{m}}{\dot \qv_{n_b}}
          = \left[ \Null,\; \ldots,\; \Null,\; \Im,\; \Null,\; \ldots,\; \Null,\; \Im,\; \Null,\; \ldots,\; \Null \right]\eqComma
        \ee
        in which the $\Im$ matrices are placed at the according indices of marker nodes.
        In case of \texttt{ObjectFFRFreducedOrder}, this jacobian is computed as weighted sum
        of the position jacobians, see \texttt{ObjectFFRFreducedOrder},
        \be
          \LU{0}{\Jm_{m,pos}^{FFRFreduced}} = \frac{\partial \LU{0}{\vv}_{m}}{\dot \qv_{n_b}}
          = \sum_i w_i \LU{0}{\Jm^{(i)}_\mathrm{pos}}
          = \left[\Im, \; -\LU{0r}{\Rot} \left(\sum_i(\LU{r}{\tilde\uv\indf^{(i)}} + \LU{r}{\tilde\xv^{(i)}\cRef}) \right) \LU{r}{\Gm},\;
                  \sum_i w_i \LU{0r}{\Rot} \vr{\LU{r}{\tPsi_{r=3i}\tp}}{\LU{r}{\tPsi_{r=3i+1}\tp}}{\LU{r}{\tPsi_{r=3i+2}\tp}} \right] \eqDot
        \ee
        In \texttt{ObjectFFRFreducedOrder}, the jacobian usually affects all reduced coordinates.
    
    -->
""",
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TIndex(ItemObject), destination=DestComp+DestParam,
            pythonName='objectNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n_b$body number to which marker is attached to'),
        ItemParameter(type=TIndex(minimum=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='linkNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n_l$number of link in KinematicTree to which marker is attached to'),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='localPosition',
            defaultValue=DVZeroVector3D,
            description=r"""$\LU{l}{\bv}$local (link-fixed) position of marker at link $n_l$, using the link ($n_l$) coordinate system"""),
        ItemFunctionDef('GetObjectNumber',
            implementation='return parameters.objectNumber;'),
        ItemFunctionDef('SetObjectNumber',
            args='Index objectNumber, Index localIndex = 0',
            implementation='parameters.objectNumber = objectNumber;'),
        ItemFunctionDef('GetNumberOfObjects',
            implementation='return 1;'),
        ItemTypes('Marker', ['Body', 'Object', 'Position', 'Orientation', 'KinematicTree'],
            description=r'return marker type (for node treatment in computation)'),
        ItemFunctionDef('GetDimension',
            implementation='return 3;'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetRotationMatrix',
            description='return configuration dependent rotation matrix of node; returns always a 3D Matrix'),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetAngularVelocityLocal',
            description='return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunctionDef('ComputeMarkerDataJacobianDerivative'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "KinematicTreeRigid";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerObjectODE2Coordinates   +++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerObjectODE2Coordinates',
    cParentClass=ParentClassCMarker,
    classDescription=r'A Marker attached to all coordinates of an object (currently only body is possible), e.g. to apply special constraints or loads on all coordinates. The measured coordinates INCLUDE reference + current coordinates.',
    classType=ClassTypeMarker,
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TIndex(ItemObject), destination=DestComp+DestParam,
            pythonName='objectNumber',
            defaultValue=DVInvalidIndex,
            description=r'body number to which marker is attached to'),
        ItemFunctionDef('GetObjectNumber',
            implementation='return parameters.objectNumber;'),
        ItemFunctionDef('SetObjectNumber',
            args='Index objectNumber, Index localIndex = 0',
            implementation='parameters.objectNumber = objectNumber;'),
        ItemFunctionDef('GetNumberOfObjects',
            implementation='return 1;'),
        ItemTypes('Marker', ['Body', 'Object', 'Coordinates', 'JacobianDerivativeAvailable'],
            description=r'return marker type (for node treatment in computation)'),
        ItemFunctionDef('GetDimension'),
        ItemFunctionDef('GetPosition',
            implementation='position = Vector3D({0,0,0});'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunctionDef('ComputeMarkerDataJacobianDerivative'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ObjectODE2Coordinates";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetObjectODE2Coordinates',
            args='const CSystemData& cSystemData, Vector& objectCoordinates, Vector& objectCoordinates_t',
            description=r"""return the ABRV:ODE2 coordinate vectors (and derivative) of the attached object"""),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics',
            implementation=''),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerBodyCable2DShape   ++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerBodyCable2DShape',
    addProtectedC=r"""    static constexpr Index maxNumberOfSegments = 12; //maximum number of contact segments
""",
    cParentClass=ParentClassCMarker,
    classDescription=r'A special Marker attached to a 2D ANCF beam finite element with cubic interpolation and 8 coordinates.',
    classType=ClassTypeMarker,
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TIndex(ItemObject), destination=DestComp+DestParam,
            pythonName='bodyNumber',
            defaultValue=DVInvalidIndex,
            description=r'body number to which marker is attached to'),
        ItemParameter(type=TIndex(greaterThan=0), destination=DestComp+DestParam,
            pythonName='numberOfSegments',
            defaultValue=3,
            description=r'number of number of segments; each segment is a line and is associated to a data (history) variable; must be same as in according contact element'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='verticalOffset',
            defaultValue=0.,
            description=r'vertical offset from beam axis in positive (local) Y-direction; this offset accounts for consistent computation of positions and velocities at the surface of the beam'),
        ItemFunctionDef('GetObjectNumber',
            implementation='return parameters.bodyNumber;'),
        ItemFunctionDef('SetObjectNumber',
            args='Index bodyNumber, Index localIndex = 0',
            implementation='parameters.bodyNumber = bodyNumber;'),
        ItemFunctionDef('GetNumberOfObjects',
            implementation='return 1;'),
        ItemTypes('Marker', ['Body', 'Object', 'Coordinate', 'JacobianDerivativeAvailable'],
            description=r'return marker type (for node treatment in computation)'),
        ItemFunctionDef('GetDimension',
            implementation='return 2;'),
        ItemFunctionDef('GetPosition',
            description='return position of marker -> axis-midpoint of ANCF cable'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunctionDef('ComputeMarkerDataJacobianDerivative'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "BodyCable2DShape";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerBodyCable2DCoordinates   ++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerBodyCable2DCoordinates',
    cParentClass=ParentClassCMarker,
    classDescription=r'A special Marker attached to the coordinates of a 2D ANCF beam finite element with cubic interpolation.',
    classType=ClassTypeMarker,
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TIndex(ItemObject), destination=DestComp+DestParam,
            pythonName='bodyNumber',
            defaultValue=DVInvalidIndex,
            description=r'body number to which marker is attached to'),
        ItemFunctionDef('GetObjectNumber',
            implementation='return parameters.bodyNumber;'),
        ItemFunctionDef('SetObjectNumber',
            args='Index bodyNumber, Index localIndex = 0',
            implementation='parameters.bodyNumber = bodyNumber;'),
        ItemFunctionDef('GetNumberOfObjects',
            implementation='return 1;'),
        ItemTypes('Marker', ['Body', 'Object', 'Coordinate', 'JacobianDerivativeAvailable'],
            description=r'return marker type (for node treatment in computation)'),
        ItemFunctionDef('GetDimension',
            implementation='return 2;'),
        ItemFunctionDef('GetPosition',
            description='return position of marker -> axis-midpoint of ANCF cable'),
        ItemFunctionDef('ComputeMarkerData'),
        ItemFunctionDef('ComputeMarkerDataJacobianDerivative'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "BodyCable2DCoordinates";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerBodyBeamShape   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerBodyBeamShape',
    cParentClass=ParentClassCMarker,
    classDescription=r'A special Marker attached to a 3D beam finite element which provides at least position and tangent to the beam axis.',
    classType=ClassTypeMarker,
    mainParentClass=MainParentClassMainMarker,
    visuParentClass=VisuParentClassVisualizationMarker,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"marker's unique name"),
        ItemParameter(type=TIndex(ItemObject), destination=DestComp+DestParam,
            pythonName='bodyNumber',
            defaultValue=DVInvalidIndex,
            description=r'body number to which marker is attached to (beam type)'),
        ItemFunctionDef('GetObjectNumber',
            implementation='return parameters.bodyNumber;'),
        ItemFunctionDef('SetObjectNumber',
            args='Index bodyNumber, Index localIndex = 0',
            implementation='parameters.bodyNumber = bodyNumber;'),
        ItemFunctionDef('GetNumberOfObjects',
            implementation='return 1;',
            description='general access to local object number'),
        ItemTypes('Marker', ['Body', 'Object', 'Beam3DShape', 'JacobianDerivativeAvailable'],
            description=r'return marker type'),
        ItemFunctionDef('GetDimension',
            implementation='return 3;'),
        ItemFunctionDef('GetPosition',
            description='return position of marker -> axis-midpoint of beam element; mostly for drawing'),
        ItemFunctionDef('ComputeMarkerData',
            description='Compute marker data (e.g. position and positionJacobian, etc.) for a marker'),
        ItemFunctionDef('ComputeMarkerDataJacobianDerivative'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "BodyBeamShape";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemFunction(type=Tvoid, destination=DestComp, isVirtual=False, isStatic=True,
            pythonName='ComputeSlidingJointData',
            args='Real xBeam, Real lBeam, const ResizableVector& totalCoordinates, const ResizableVector& totalCoordinates_t, Vector3D& position, Vector3D& slopeVector, Vector3D& slopeVector_x, bool& beamHasTorsion, ConfigurationType configuration=ConfigurationType::Current',
            description=r'Compute all data for sliding joint computations'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))
