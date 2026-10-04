#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Marker item definitions
#
# Details:  The input of the generators for marker items.
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
    overallDescription=r'A marker attached to the body mass; use this marker to apply a body-load (e.g. gravitational force).',
    classType=ClassTypeMarker,
    miniExample=r"""    #gravity on a planar rigid body: the load acts on the mass of the whole body
    node = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0.5,0.2,0]))
    body = mbs.AddObject(ObjectRigidBody2D(nodeNumber=node, mass=2, inertia=0.1))
    mMass = mbs.AddMarker(MarkerBodyMass(bodyNumber=body))
    mbs.AddLoad(LoadMassProportional(markerNumber=mMass, loadVector=[0,-9.81,0]))

    mbs.Assemble()
    mbs.SolveDynamic()

    #free fall: y = -g/2*t^2 at t=1, independent of the mass
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Displacement)[1] #-4.905
    """,
    miniExamplePerformanceTest={'numberOfSteps': 1584910},
    examples=['Examples/basicTutorial2024.py', 'Examples/notebooks/tutorialSpringDamperCreate.py', 'Examples/notebooks/tutorialRigidBody.py', 'Examples/notebooks/tutorialRigidBodyCreate.py'],
    detailedDescription=r"""    #### Marker quantities

    None that a connector reads: the marker exists to take a load proportional to the mass of the body,
    `LoadMassProportional`, and provides only its Jacobian.

    #### Jacobians

    The mass-weighted integral of the position Jacobian over the body,

    $$
    \Jm_{m} = \int_V \rho\, \frac{\partial \LU{0}{\pv}}{\partial \qv}\, dV ,
    $$

    which the body computes (its access function `DisplacementMassIntegral_q`); a load vector
    $\LU{0}{\bv}$ per unit mass gives $\Qm = \Jm_{m}\tp \LU{0}{\bv}$. For a rigid body it is the mass
    times the position Jacobian of the center of mass.
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
    overallDescription=r"""A position body-marker attached to a local (body-fixed) position $\pLocB = [b_0,\; b_1,\; b_2]$ ($x$, $y$, and $z$ coordinates) of the body. It provides position information as well as the according derivatives (=velocity and derivative of position w.r.t. body coordinates). It can be used for connectors, joints or loads where position is required. If connectors also require orientation information, use a MarkerBodyRigid.""",
    classType=ClassTypeMarker,
    miniExample=r"""    #a point of a body - here of the ground, at a local position - connected to a mass point by a spring
    node = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0]))
    body = mbs.AddObject(ObjectMassPoint(nodeNumber=node, mass=1))
    mBody = mbs.AddMarker(MarkerBodyPosition(bodyNumber=body, localPosition=[0,0,0]))
    mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[1,0,0]))
    mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers=[mGround, mBody], stiffness=[100,100,100]))
    mbs.AddLoad(LoadForceVector(markerNumber=mBody, loadVector=[0,0,-10]))

    mbs.Assemble()
    mbs.SolveStatic()

    #the spring is stretched by F/k
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Displacement)[2] #-0.1
    """,
    miniExamplePerformanceTest={'numberOfSteps': 920110},
    examples=['Examples/notebooks/tutorialSpringDamperCreate.py', 'Examples/notebooks/tutorialRigidBodyCreate.py', 'Examples/rigidPendulum.py', 'Examples/pendulum2Dconstraint.py', 'Examples/cartesianSpringDamper.py'],
    detailedDescription=r"""    #### Marker quantities

    | quantity | symbol | as computed |
    |---|---|---|
    | position | $\LU{0}{\pv}_m$ | the output variable `Position` of the body at the local position $\pLocB$ |
    | velocity | $\LU{0}{\vv}_m$ | the output variable `Velocity` of the body at $\pLocB$ |

    Both are global. $\pLocB$ is given in the body frame, from the reference point of the body.

    #### Jacobians

    The position Jacobian is the derivative of the velocity of the point with respect to the velocity
    coordinates of the body,

    $$
    \LU{0}{\Jm_{pos}} = \frac{\partial \LU{0}{\vv}_m}{\partial \dot\qv} ,
    $$

    which the body computes (its access function `TranslationalVelocity_qt`). For a rigid body it
    contains the rotation of the local position, see the page of the body.
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
        ItemFunction(type=TBool, destination=DestComp, cFlags=CFConst,
            pythonName='GetLocalPosition', args='Vector3D& localPosition',
            implementation='localPosition = parameters.localPosition; return true;',
            description=r'the local position on the body, for the check of IsValidLocalPosition at Assemble() (#2744)'),
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst,
            pythonName='GetODE2Size', args='const CSystemData& cSystemData, MarkerTemp& temp',
            description=r'number of ODE2 coordinates of the body (#2745)'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst,
            pythonName='AddGeneralizedForce', args='const CSystemData& cSystemData, const Vector3D& force, MarkerTemp& temp, LinkedDataVector& ode2Lhs',
            description=r'add J_pos^T force at the local position to ode2Lhs, through the body (#2745)'),
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
    image='itemImages/MarkerBodyRigid.png', #the representative image of the page (#2830)
    cParentClass=ParentClassCMarker,
    overallDescription=r"""A rigid-body (position+orientation) body-marker attached to a local (body-fixed) position $\pLocB = [b_0,\; b_1,\; b_2]$ ($x$, $y$, and $z$ coordinates) of the body. It provides position and orientation (rotation), as well as the according derivatives. It can be used for most connectors, joints or loads where either position, position and orientation, or orientation are required.""",
    classType=ClassTypeMarker,
    miniExample=r"""    #position and orientation of a rigid body: a torque on it, held by a rigid body spring-damper
    inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
    node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[1,0,0]+eulerParameters0))
    body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(),
                                         inertia=inertia.GetInertia6D()))
    mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localPosition=[0,0,0]))
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[1,0,0]))
    mbs.AddObject(ObjectConnectorRigidBodySpringDamper(markerNumbers=[mGround, mBody],
                                                       stiffness=np.diag([1e4,1e4,1e4, 100,100,100]),
                                                       damping=np.zeros((6,6))))
    mbs.AddLoad(LoadTorqueVector(markerNumber=mBody, loadVector=[0,0,1]))

    mbs.Assemble()
    mbs.SolveStatic()

    #rotation about z: M/k_rot, for the small angle
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Rotation)[2] #0.01
    """,
    miniExamplePerformanceTest={'numberOfSteps': 226940},
    examples=['Examples/notebooks/tutorialSpringDamperCreate.py', 'Examples/notebooks/tutorialRigidBody.py', 'Examples/notebooks/tutorialRigidBodyCreate.py'],
    detailedDescription=r"""    #### Marker quantities

    | quantity | symbol | as computed |
    |---|---|---|
    | position | $\LU{0}{\pv}_m$ | the output variable `Position` of the body at the local position $\pLocB$ |
    | velocity | $\LU{0}{\vv}_m$ | the output variable `Velocity` of the body at $\pLocB$ |
    | rotation matrix | $\LU{0m}{\Rot} = \LU{0b}{\Rot} \LU{bm}{\Rot}$ | the rotation $\LU{0b}{\Rot}$ of the body at $\pLocB$ (for a rigid body the rotation of the body), turned by the rotation $\LU{bm}{\Rot}$ of `localHT` |
    | angular velocity | $\LU{m}{\tomega} = \LU{bm}{\Rot}\tp \LU{b}{\tomega}$ | the angular velocity of the body at $\pLocB$, in the marker frame |

    Position, velocity and rotation matrix are global quantities; the angular velocity is local.
    $\pLocB$ is given in the body frame, from the reference point of the body. The marker frame is
    `localHT` = $[\LU{bm}{\Rot}\;\pLocB;\;\Null\tp\;1]$ in the body frame: `localPosition` is its
    translation, and its rotation is the unit matrix unless `localHT` is given. The Jacobians are
    global and do not depend on $\LU{bm}{\Rot}$.

    #### Jacobians

    $$
    \LU{0}{\Jm_{pos}} = \frac{\partial \LU{0}{\vv}_m}{\partial \dot\qv} , \quad
    \LU{0}{\Jm_{rot}} = \frac{\partial \LU{0}{\tomega}_m}{\partial \dot\qv} ,
    $$

    the derivatives of the global velocity and angular velocity with respect to the velocity
    coordinates of the body (its access functions `TranslationalVelocity_qt` and
    `AngularVelocity_qt`). For `ObjectRigidBody` they are computed from its node directly, with the
    velocity transformation $\LU{0}{\Gm}$ of the rotation parameters in $\LU{0}{\Jm_{rot}}$.
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
            defaultValue=CppValue('Vector3D({0.,0.,0.})', 'None', 'None (zero)'),
            description=r"""$\pLocB$local body position of marker; e.g. local (body-fixed) position where force is applied to; the translation of localHT""",
            partOfHT='localHT'),
        ItemParameter(type=THomogeneousTransformation, destination=DestComp+DestParam,
            pythonName='localHT',
            defaultValue=CppValue('HomogeneousTransformation()', 'None', 'None (identity)'),
            description=r"""$\LU{b}{\Hm}_{m}$the frame of the marker in the body frame, as homogeneous transformation: its translation is localPosition, its rotation $\LU{bm}{\Rot}$ turns the marker frame against the body; a 4x4 matrix, its 16 values row by row or an exu.HT; None: not given; given together with localPosition, both must agree"""),
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
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst,
            pythonName='GetKinematicsRigid', args='const CSystemData& cSystemData, MarkerRigid<Real>& kinematics, MarkerTemp& temp',
            description=r'frame and velocities through the body, without the Jacobians; the body keeps in temp what it needs for AddGeneralizedForceTorque (#2745)'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst,
            pythonName='AddGeneralizedForceTorque', args='const CSystemData& cSystemData, const Vector3D& force, const Vector3D& torque, MarkerTemp& temp, LinkedDataVector& ode2Lhs',
            description=r'add J_pos^T force + J_rot^T torque at the local position to ode2Lhs, through the body (#2745)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "BodyRigid";',
            description=r"Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?"),
        ItemFunction(type=TBool, destination=DestComp, cFlags=CFConst,
            pythonName='GetLocalPosition', args='Vector3D& localPosition',
            implementation='localPosition = parameters.localHT.GetTranslation(); return true;',
            description=r'the local position on the body, for the check of IsValidLocalPosition at Assemble() (#2744)'),
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
    #the node types the node must provide, each entry a list of alternatives: it generates
    #GetRequestedNodeTypes() of the Main class, which Assemble checks and mbs.Inspect answers (#2817),
    #and the page of the marker and of the nodes say it (#2725)
    requestedNodeTypes=[['Position', 'Position2D']],
    overallDescription=r'A node-Marker attached to a position-based node. It can be used for connectors, joints or loads where position is required. If connectors also require orientation information, use a MarkerNodeRigid.',
    classType=ClassTypeMarker,
    miniExample=r"""    #the position of a node: a mass hanging on a spring from a ground node, released at rest
    nMass = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,-1]))
    mbs.AddObject(ObjectMassPoint(nodeNumber=nMass, mass=1))
    mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
    mFixed = mbs.AddMarker(MarkerNodePosition(nodeNumber=nGround))
    k = (2*np.pi)**2 #1 Hz
    mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[mFixed, mMass], stiffness=k, referenceLength=1))
    mbs.AddLoad(LoadForceVector(markerNumber=mMass, loadVector=[0,0,-9.81]))

    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.endTime = 0.5 #half a period
    mbs.SolveDynamic(simulationSettings)

    #lowest point: twice the static deflection, -2*g/k
    exu.sys['testResult'] = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Displacement)[2] #-0.497
    """,
    miniExamplePerformanceTest={'numberOfSteps': 956230},
    examples=['Examples/doublePendulum2D.py', 'Examples/pendulum2Dconstraint.py', 'Examples/interactiveTutorial.py', 'Examples/simple4linkPendulumBing.py', 'TestModels/connectorGravityTest.py'],
    detailedDescription=r"""    #### Marker quantities

    | quantity | symbol | as computed |
    |---|---|---|
    | position | $\LU{0}{\pv}_m$ | the position of the node, as its output variable `Position` |
    | velocity | $\LU{0}{\vv}_m$ | the velocity of the node, as its output variable `Velocity` |

    Both are global, or in the frame the object reads the node in (see the page of the node).

    #### Jacobians

    The position Jacobian of the node, $\LU{0}{\Jm_{pos}} = \partial \LU{0}{\pv} / \partial \qv$ with
    respect to the node's coordinates: the unit matrix for `NodePoint`, the first three columns for a
    rigid body node, and for a 2D node the $x$ and $y$ rows.
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
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst,
            pythonName='GetODE2Size', args='const CSystemData& cSystemData, MarkerTemp& temp',
            description=r'number of ODE2 coordinates of the node, 0 for a ground node (#2745)'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst,
            pythonName='AddGeneralizedForce', args='const CSystemData& cSystemData, const Vector3D& force, MarkerTemp& temp, LinkedDataVector& ode2Lhs',
            description=r'add J_pos^T force of the node to ode2Lhs, without the marker data (#2745)'),
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
    requestedNodeTypes=[['Position', 'Position2D'], ['Orientation', 'Orientation2D']],
    overallDescription=r'A rigid-body (position+orientation) node-marker attached to a rigid-body node. It provides position and orientation (rotation), as well as the according derivatives. It can be used for most connectors, joints or loads where either position, position and orientation, or orientation are required.',
    classType=ClassTypeMarker,
    miniExample=r"""    #position and orientation of a rigid body node: a torque spins the body up
    inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
    node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0.5,0.2,0.1, 0,0,0]))
    mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(),
                                  inertia=inertia.GetInertia6D()))
    mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node))
    mbs.AddLoad(LoadTorqueVector(markerNumber=mNode, loadVector=[0,0,1]))

    mbs.Assemble()
    mbs.SolveDynamic()

    #angle = M/(2*J_zz)*t^2 at t=1
    Jzz = inertia.GetInertia6D()[2]
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Rotation)[2]*2*Jzz #1
    """,
    miniExamplePerformanceTest={'numberOfSteps': 547540},
    examples=['TestModels/connectorRigidBodySpringDamperTest.py'],
    detailedDescription=r"""    #### Marker quantities

    | quantity | symbol | as computed |
    |---|---|---|
    | position | $\LU{0}{\pv}_m$ | the position of the node |
    | velocity | $\LU{0}{\vv}_m$ | the velocity of the node |
    | rotation matrix | $\LU{0m}{\Rot} = \LU{0n}{\Rot} \LU{nm}{\Rot}$ | the rotation matrix of the node, turned by the rotation $\LU{nm}{\Rot}$ of `localHT` |
    | angular velocity | $\LU{m}{\tomega} = \LU{nm}{\Rot}\tp \LU{n}{\tomega}$ | the angular velocity of the node, in the marker frame |

    The translation of `localHT` must be zero: the marker is at the node.

    #### Jacobians

    $$
    \LU{0}{\Jm_{pos}} = \frac{\partial \LU{0}{\pv}}{\partial \qv} , \quad
    \LU{0}{\Jm_{rot}} = \frac{\partial \LU{0}{\tomega}}{\partial \dot\qv} ,
    $$

    with respect to the coordinates of the node; for a rigid body node $\LU{0}{\Jm_{rot}}$ is the
    velocity transformation $\LU{0}{\Gm}$ of its rotation parameters in the rotation columns, see the
    page of the node. For a slope node the orientation is that of its slope vector(s).
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
        ItemParameter(type=THomogeneousTransformation, destination=DestComp+DestParam,
            pythonName='localHT',
            defaultValue=CppValue('HomogeneousTransformation()', 'None', 'None (identity)'),
            description=r"""$\LU{n}{\Hm}_{m}$the frame of the marker in the node frame, as homogeneous transformation: its rotation turns the marker frame against the node; its translation must be zero for now; a 4x4 matrix, its 16 values row by row or an exu.HT; None: the node frame"""),
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
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst,
            pythonName='GetKinematicsRigid', args='const CSystemData& cSystemData, MarkerRigid<Real>& kinematics, MarkerTemp& temp',
            description=r'frame and velocities of the node, without the Jacobians; a rigid body node keeps its rotation and G matrices in temp for AddGeneralizedForceTorque, other nodes go through the marker data (#2745)'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst,
            pythonName='AddGeneralizedForceTorque', args='const CSystemData& cSystemData, const Vector3D& force, const Vector3D& torque, MarkerTemp& temp, LinkedDataVector& ode2Lhs',
            description=r'add J_pos^T force + J_rot^T torque to ode2Lhs, the coordinates of the node (#2745)'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
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
    overallDescription=r"""A node-Marker attached to a ABRV:ODE2 coordinate of a node; this marker allows to connect a coordinate-based constraint or connector to a nodal coordinate (also NodeGround); for ABRV:ODE1 coordinates use `MarkerNodeODE1Coordinate`.""",
    classType=ClassTypeMarker,
    miniExample=r"""    #one coordinate of a node: a coordinate spring between the ground node and a 1D mass
    node = mbs.AddNode(Node1D(referenceCoordinates=[0]))
    mbs.AddObject(ObjectMass1D(nodeNumber=node, mass=1))
    mCoord = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=0))
    mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
    mbs.AddObject(ObjectConnectorCoordinateSpringDamper(markerNumbers=[mGround, mCoord], stiffness=100))
    mbs.AddLoad(LoadCoordinate(markerNumber=mCoord, load=10))

    mbs.Assemble()
    mbs.SolveStatic()

    #the spring is stretched by F/k
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates) #0.1
    """,
    miniExamplePerformanceTest={'numberOfSteps': 1414030},
    examples=['Examples/notebooks/tutorialSpringDamper.py', 'Examples/coordinateSpringDamper.py', 'Examples/SliderCrank.py', 'Examples/plotSensorExamples.py', 'Examples/SpringDamperMassUserFunction.py'],
    detailedDescription=r"""    #### Marker quantities

    | quantity | symbol | as computed |
    |---|---|---|
    | coordinate | $c = q_i$ | the **current** ABRV:ODE2 coordinate `coordinate` $= i$ of the node, **without** its reference value |
    | its velocity | $\dot c = \dot q_i$ | the time derivative of that coordinate |

    #### Jacobians

    $\Jm = \ev_i\tp$, a row of the unit matrix: a force $f$ acts on coordinate $i$ only, $Q_i = f$.

    On a node without ABRV:ODE2 coordinates - `NodePointGround` - the coordinate is zero and the
    Jacobian empty, so nothing acts: this is the ground side of a `CoordinateSpringDamper` or a
    `CoordinateConstraint`.
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
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst,
            pythonName='GetKinematicsCoordinate', args='const CSystemData& cSystemData, MarkerCoordinate<Real>& kinematics, MarkerTemp& temp',
            description=r'the coordinate and its velocity, read from the node, without the marker data (#2745)'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst,
            pythonName='AddGeneralizedForceCoordinate', args='const CSystemData& cSystemData, Real force, MarkerTemp& temp, LinkedDataVector& ode2Lhs',
            description=r'add the force to the coordinate (#2745)'),
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
    overallDescription=r"""A node-Marker attached to all ABRV:ODE2 coordinates of a node. IN CONTRAST to MarkerNodeCoordinate, the marker coordinates INCLUDE the reference values! For ABRV:ODE1 coordinates use `MarkerNodeODE1Coordinates`.""",
    classType=ClassTypeMarker,
    miniExample=r"""    #all coordinates of two nodes, tied by a coordinate vector constraint X1 qB - X0 qA = offset; the
    #coordinates INCLUDE the reference values, so qB - qA = [1,0,0] keeps the two points where they are
    nA = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,0]))
    nB = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0]))
    mbs.AddObject(ObjectMassPoint(nodeNumber=nA, mass=1))
    mbs.AddObject(ObjectMassPoint(nodeNumber=nB, mass=1))
    mA = mbs.AddMarker(MarkerNodeCoordinates(nodeNumber=nA))
    mB = mbs.AddMarker(MarkerNodeCoordinates(nodeNumber=nB))
    mbs.AddObject(ObjectConnectorCoordinateVector(markerNumbers=[mA, mB], scalingMarker0=np.eye(3),
                                                 scalingMarker1=np.eye(3), offset=[1,0,0]))
    mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=nA)), loadVector=[2,0,0]))

    mbs.Assemble()
    mbs.SolveDynamic()

    #both masses move together: a = F/(2m) = 1, x = a/2*t^2 at t=1
    exu.sys['testResult'] = mbs.GetNodeOutput(nB, exu.OutputVariableType.Displacement)[0] #0.5
    """,
    miniExamplePerformanceTest={'numberOfSteps': 757100},
    detailedDescription=r"""    #### Marker quantities

    | quantity | symbol | as computed |
    |---|---|---|
    | coordinates | $\cv = \qv\cRef + \qv$ | **all** ABRV:ODE2 coordinates of the node, **including** their reference values |
    | their velocities | $\dot\cv = \dot\qv$ | the time derivatives |

    Unlike `MarkerNodeCoordinate`, the values include the reference values.

    #### Jacobians

    The unit matrix of the size of the node's coordinates: a vector of forces acts on the coordinates
    one by one. On a node without ABRV:ODE2 coordinates the values and the Jacobian are
    empty.
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
    overallDescription=r'A node-Marker attached to a ABRV:ODE1 coordinate of a node.',
    classType=ClassTypeMarker,
    miniExample=r"""    #a coordinate of a first-order system: a constant input to q_t = -q + f
    node = mbs.AddNode(NodeGenericODE1(numberOfODE1Coordinates=1, referenceCoordinates=[0],
                                       initialCoordinates=[0]))
    mbs.AddObject(ObjectGenericODE1(nodeNumbers=[node], systemMatrix=[[-1]]))
    mCoord = mbs.AddMarker(MarkerNodeODE1Coordinate(nodeNumber=node, coordinate=0))
    mbs.AddLoad(LoadCoordinate(markerNumber=mCoord, load=1))

    mbs.Assemble()
    mbs.SolveDynamic(solverType=exu.DynamicSolverType.RK44)

    #q(1) = 1 - exp(-1)
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates) #0.632
    """,
    miniExamplePerformanceTest={'numberOfSteps': 1564820},
    detailedDescription=r"""    #### Marker quantities

    | quantity | symbol | as computed |
    |---|---|---|
    | coordinate | $c = y_i$ | the **current** ABRV:ODE1 coordinate `coordinate` $= i$ of the node |

    There is no velocity: an ABRV:ODE1 coordinate has no time derivative of its own in the solver.

    #### Jacobians

    $\Jm = \ev_i\tp$, a row of the unit matrix. On a node without ABRV:ODE1 coordinates the value is zero
    and the Jacobian empty.
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
    #checked by Assemble, as all requestedNodeTypes (#2817)
    requestedNodeTypes=[['Orientation']],
    overallDescription=r'A node-Marker attached to a a node containing rotation; the Marker measures a rotation coordinate (Tait-Bryan angles) or angular velocities on the velocity level.',
    classType=ClassTypeMarker,
    miniExample=r"""    #a rotation coordinate of a rigid body node, held by a coordinate constraint
    inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
    node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0,0,0, 0,0,0]))
    mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(),
                                  inertia=inertia.GetInertia6D()))
    mRotZ = mbs.AddMarker(MarkerNodeRotationCoordinate(nodeNumber=node, rotationCoordinate=2))
    mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
    oHold = mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mGround, mRotZ]))
    mbs.AddLoad(LoadTorqueVector(markerNumber=mbs.AddMarker(MarkerNodeRigid(nodeNumber=node)), loadVector=[0,0,2]))

    mbs.Assemble()
    mbs.SolveDynamic()

    #the constraint holds the rotation about z against the torque: its force is the reaction torque
    exu.sys['testResult'] = mbs.GetObjectOutput(oHold, exu.OutputVariableType.Force) #2
    """,
    miniExamplePerformanceTest={'numberOfSteps': 467030},
    detailedDescription=r"""    #### Marker quantities

    | quantity | symbol | as computed |
    |---|---|---|
    | rotation | $\varphi_i$ | the Tait-Bryan angle $i$ = `rotationCoordinate` (0: about $x$, 1: $y$, 2: $z$), computed from the rotation matrix of the node |
    | its velocity | $\omega_i$ | component $i$ of the global angular velocity of the node |

    The angle is **recomputed from the rotation matrix**, whatever the rotation parameters of the node
    are, so it lies in $(-\pi,\,\pi]$ and jumps after a full turn; and $\omega_i$ is the time derivative
    of $\varphi_i$ only while the rotations about the other two axes are small. Use the marker for
    rotations that stay in that range - a spring about one axis of a nearly planar motion - and a
    `MarkerBodiesRelativeRotationCoordinate` or an Euler angle coordinate otherwise.

    #### Jacobians

    Row $i$ of the rotation Jacobian of the node, $\Jm = \partial \omega_i / \partial \dot\qv$.
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
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst,
            pythonName='GetKinematicsCoordinate', args='const CSystemData& cSystemData, MarkerCoordinate<Real>& kinematics, MarkerTemp& temp',
            description=r'the rotation coordinate and its angular velocity, and the rotation Jacobian of the node into temp for AddGeneralizedForceCoordinate, without the marker data (#2745)'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst,
            pythonName='AddGeneralizedForceCoordinate', args='const CSystemData& cSystemData, Real force, MarkerTemp& temp, LinkedDataVector& ode2Lhs',
            description=r'add the torque about the axis of the rotation coordinate, projected by its row of the rotation Jacobian (#2745)'),
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
    overallDescription=r'A coordinate-based Marker attached to two rigid bodies or beams which computes the relative translation between the bodies according to the given axis. This marker can be used together with coordinate-based constraints and connectors (e.g., CoordinateSpringDamper and CoordinateConstraint). NOTE: it is assumed that the two bodies can only move along the given axis (e.g., constrained by a prismatic joint) -- otherwise results may be unexpected. NOTE: this approach is not compatible with FFRF-based flexible bodies and currently requires and intermediate rigid body.',
    classType=ClassTypeMarker,
    miniExample=r"""    #the translation of body 1 relative to body 0 along an axis of body 0, held by a coordinate constraint
    inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
    node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0,0,0, 0,0,0]))
    body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(),
                                         inertia=inertia.GetInertia6D()))
    mRel = mbs.AddMarker(MarkerBodiesRelativeTranslationCoordinate(bodyNumbers=[oGround, body], axis0=[1,0,0]))
    mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
    mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mGround, mRel], offset=0.3))

    mbs.Assemble()
    mbs.SolveDynamic()

    #the body is held 0.3 along x of the ground
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[0] #0.3
    """,
    miniExamplePerformanceTest={'numberOfSteps': 434380},
    detailedDescription=r"""    The marker consists of two bodies, body $b_0$ and body $b_1$ with respective global marker positions $\LU{0}{\pv}_{m0}$ and $\LU{0}{\pv}_{m1}$,
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

    It is a coordinate marker: coordinate connectors, coordinate constraints and `LoadCoordinate` use it.
    Body 0 must provide position and orientation, body 1 a position, both at a local point - a rigid body,
    and for body 1 also a mass point; `Assemble()` refuses other bodies.
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
        ItemTypes('Marker', ['Body', 'Object', 'Coordinate'],
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
    overallDescription=r'A coordinate-based Marker attached to two rigid bodies or beams which computes the relative rotation between the bodies according to the given axis; this marker can be used together with coordinate-based constraints and connectors (e.g., CoordinateSpringDamper and CoordinateConstraint). NOTE: it is assumed that the two bodies can only rotate about the given axis (e.g., constrained by a revolute joint) -- otherwise results may be unexpected. NOTE: this approach is not compatible with FFRF-based flexible bodies and currently requires and intermediate rigid body.',
    classType=ClassTypeMarker,
    miniExample=r"""    #the rotation of body 1 relative to body 0 about an axis of body 0, held by a coordinate constraint;
    #the data node continues the angle beyond +-pi
    inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
    node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0,0,0, 0,0,0]))
    body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(),
                                         inertia=inertia.GetInertia6D()))
    nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=1, initialCoordinates=[0]))
    mRel = mbs.AddMarker(MarkerBodiesRelativeRotationCoordinate(bodyNumbers=[oGround, body], axis0=[0,0,1],
                                                               nodeNumber=nData))
    mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
    oHold = mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mGround, mRel]))
    mbs.AddLoad(LoadTorqueVector(markerNumber=mbs.AddMarker(MarkerNodeRigid(nodeNumber=node)), loadVector=[0,0,2]))

    mbs.Assemble()
    mbs.SolveDynamic()

    #the constraint holds the relative rotation about z against the torque: its force is the reaction torque
    exu.sys['testResult'] = mbs.GetObjectOutput(oHold, exu.OutputVariableType.Force) #2
    """,
    miniExamplePerformanceTest={'numberOfSteps': 349140},
    detailedDescription=r"""    The marker consists of two bodies, body $b_0$ and body $b_1$ with respective global marker positions $\LU{0}{\pv}_{m0}$ and $\LU{0}{\pv}_{m1}$,
    depending on local positions $\LU{m_0}{\pv}_0$ and $\LU{m_1}{\pv}_1$, 
    and marker orientations $\LU{0,m_0}{\Rot}_{m0}$ and $\LU{0,m_1}{\Rot}_{m1}$.
    From the given axis `axis0`, we compute an orthonormal basis (orthonormal to axis0) relative to marker $m_0$,


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

    It is a coordinate marker: coordinate connectors, coordinate constraints and `LoadCoordinate` use it.
    Both bodies must provide position and orientation at a local point, as a rigid body does; `Assemble()`
    refuses other bodies.
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
        ItemTypes('Marker', ['Body', 'Object', 'Node', 'Coordinate', 'HasPostNewton'],
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
    overallDescription=r'A position marker attached to a SuperElement, such as ObjectFFRF, ObjectGenericODE2 and ObjectFFRFreducedOrder (for which it is in its current implementation inefficient for large number of meshNodeNumbers). The marker acts on the mesh (interface) nodes, not on the underlying nodes of the object.',
    classType=ClassTypeMarker,
    detailedDescription=r"""    **Definition of marker quantities**:

    | intermediate variables | symbol | description |
    |---|---|---|
    | number of mesh nodes | $n_m$ | size of `meshNodeNumbers` and `weightingFactors` which are marked; this must not be the number of mesh nodes in the marked object |
    | mesh node number | $i = k_i$ | abbreviation |
    | mesh node points | $\LU{0}{\pv}_{i}$ | position of mesh node $k_i$ in object $n_b$ |
    | mesh node velocities | $\LU{0}{\vv}_{i}$ | velocity of mesh node $i$ in object $n_b$ |
    | marker position | $\LU{0}{\pv}_{m} = \sum_i w_i \cdot \LU{0}{\pv_i}$ | current global position which is provided by marker |
    | marker velocity | $\LU{0}{\vv}_{m} = \sum_i w_i \cdot \LU{0}{\vv_i}$ | current global velocity which is provided by marker |

    

    #### Marker quantities

    The marker provides a 'position' jacobian, which is the derivative of the marker velocity w.r.t. the 
    object velocity coordinates $\dot \qv_{n_b}$,


    $$
                        \Jm_{m,pos} = \frac{\partial \LU{0}{\vv}_{m}}{\partial \dot \qv_{n_b}}
                              = \sum_i w_i \cdot \Jm_{i,pos}
                        $$

    in which $\Jm_{i,pos}$ denotes the position jacobian of mesh node $i$,


    $$
                        \Jm_{i,pos} = \frac{\partial \LU{0}{\vv}_{i}}{\partial \dot \qv_{n_b}}
                        $$

    The jacobian $\Jm_{i,pos}$ usually contains mostly zeros for `ObjectGenericODE2`, because the jacobian only affects one single node.
    In `ObjectFFRFreducedOrder`, the jacobian may affect all reduced coordinates.

    Note that $\Jm_{m,pos}$ is actually computed by the
    `ObjectSuperElement` within the function `GetAccessFunctionSuperElement`.
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
    miniExamplePerformanceTest={'numberOfSteps': 581100},
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
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst,
            pythonName='GetKinematicsJacobianPosition', args='const CSystemData& cSystemData, MarkerPosition<Real>& kinematics, MarkerTemp& temp',
            description=r'position and velocity, and the position Jacobian into temp, without the marker data (#2745)'),
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst,
            pythonName='GetODE2Size', args='const CSystemData& cSystemData, MarkerTemp& temp',
            description=r'number of ODE2 coordinates of the superelement, without forming the Jacobian (#2745)'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst,
            pythonName='AddGeneralizedForce', args='const CSystemData& cSystemData, const Vector3D& force, MarkerTemp& temp, LinkedDataVector& ode2Lhs',
            description=r'add J_pos^T force to ode2Lhs; the Jacobian formed here, so that the kinematics alone do not form it (#2745)'),
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
    overallDescription=r'A position and orientation (rigid-body) marker attached to a SuperElement, such as ObjectFFRF, ObjectGenericODE2 and ObjectFFRFreducedOrder (for which it may be inefficient). The marker acts on the mesh nodes, not on the underlying nodes of the object. Note that in contrast to the MarkerSuperElementPosition, this marker needs a set of interface nodes which are not aligned at one line, such that these node points can represent a rigid body motion. Note that definitions of marker positions are slightly different from MarkerSuperElementPosition.',
    classType=ClassTypeMarker,
    miniExample=r"""    #a rigid body marker on four mesh nodes of a super element, here four free mass points of a
    #ObjectGenericODE2; the marker averages their motion, a force on it is shared by the weights
    nodes = [mbs.AddNode(NodePoint(referenceCoordinates=p)) for p in [[0,0,0],[1,0,0],[1,1,0],[0,1,0]]]
    oSuper = mbs.AddObject(ObjectGenericODE2(nodeNumbers=nodes, massMatrix=np.eye(12)))
    mSuper = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=oSuper, meshNodeNumbers=[0,1,2,3],
                                                   weightingFactors=[0.25]*4))
    mbs.AddLoad(LoadForceVector(markerNumber=mSuper, loadVector=[4,0,0]))

    mbs.Assemble()
    mbs.SolveDynamic()

    #each node gets F/4: x = F/(4m)/2*t^2 at t=1
    exu.sys['testResult'] = mbs.GetNodeOutput(nodes[0], exu.OutputVariableType.Displacement)[0] #0.5
    """,
    miniExamplePerformanceTest={'numberOfSteps': 121470},
    detailedDescription=r"""    **Definition of marker quantities**:

    | intermediate variables | symbol | description |
    |---|---|---|
    | number of mesh nodes | $n_m$ | size of `meshNodeNumbers` and `weightingFactors` which are marked; this must not be the number of mesh nodes in the marked object |
    | mesh node number | $i = k_i$ | abbreviation, runs over all marker mesh nodes |
    | mesh node local displacement | $\LU{r}{\uv^{(i)}}$ | current local (within reference frame $r$) displacement of mesh node $k_i$ in object $n_b$ |
    | mesh node local position | $\LU{r}{\pv^{(i)}} = \LU{r}{\xv^{(i)}\cRef} + \LU{r}{\uv^{(i)}}$ | current local (within reference frame $r$, which is the body frame $b$ ,e.g., in `ObjectFFRFreducedOrder`) position of mesh node $k_i$ in object $n_b$ |
    | mesh node local reference position | $\LU{r}{\xv^{(i)}\cRef}$ | local (within reference frame $r$) reference position of mesh node $k_i$ in object $n_b$, see e.g. `ObjectFFRFreducedOrder` |
    | averaged local reference position | $\LU{r}{\xv^\mathrm{avg}\cRef} = \sum_i w_i \LU{r}{\xv^{(i)}\cRef}$ | midpoint reference position of marker; averaged local reference positions of all mesh nodes $k_i$, using weighting for averaging; may not coincide with center point of your idealized joint surface (e.g., midpoint of cylinder), see [](#fig-markersuperelementrigid-sketch) |
    | marker centered mesh node local reference position | $\LU{r}{\pv^{(i)}\cRef} = \LU{r}{\xv^{(i)}\cRef}- \LU{r}{\xv^\mathrm{avg}\cRef}$ | local reference position of mesh node $k_i$ relative to the center position of marker |
    | mesh node local velocity | $\LU{r}{\vv^{(i)}}$ | current local (within reference frame $r$) velocity of mesh node $k_i$ in object $n_b$ |
    | super element reference point | $\LU{0}{\pv}_r$ ($=\LU{0}{\pv}\indt$ in `ObjectFFRFreduced- Order`) | current position (origin) of super element's floating frame (r), which is zero, if the object does not provide a reference frame (such as GenericODE2) |
    | super element rotation matrix | $\LU{0r}{\Rot}$ | current rigid body transformation matrix of super element's floating frame (r), which is the identity matrix, if the object does not provide a reference frame (such as GenericODE2) |
    | super element angular velocity | $\LU{r}{\tomega_r}$ | current local angular velocity of super element's floating frame (r), which is zero, if the object does not provide a reference frame (such as GenericODE2) |
    | marker position | $\LU{0}{\pv}_{m} \!=\! \LU{0}{\pv}_r + \LU{0r}{\Rot} \left(\LU{r}{\ov\cRef}\! +\! \sum_i w_i \cdot \LU{r}{\pv^{(i)}} \right)$ | current global position which is provided by marker; note offset $\LU{r}{\ov\cRef}$ added, if used as a correction of marker mesh nodes |
    | marker velocity | $\LU{0}{\vv}_{m} = \LU{0}{\dot \pv}_r $ $+ \LU{0r}{\Rot} \left( \LU{r}{\tilde \tomega_r} \left(\LU{r}{\ov\cRef}\! +\! \sum_i w_i \cdot \LU{r}{\pv^{(i)}} \right) + \right.$ $\left. \sum_i (w_i \cdot \LU{r}{\dot \uv^{(i)}}) \right)$ | current global velocity which is provided by marker |
    | marker rotation matrix | $\LU{0r}{\Rot}_{m} = \LU{0r}{\Rot} \cdot \mathbf{exp}(\LU{r}{\ttheta}_{m}) \LU{rm}{\Rot}$ | current rotation matrix, which transforms the local marker coordinates and adds the rigid body transformation of floating frames $\LU{0r}{\Rot}$; uses exponential map for SO3, assumes that $\ttheta$ represents a rotation vector; $\LU{rm}{\Rot}$ is the rotation of `localHT`, which also turns the local angular velocity |
    | marker local rotation | $\LU{r}{\ttheta}_{m}$ | current local linearized rotations (rotation vector); for the computation, see below for the standard and alternative approach |
    | marker local angular velocity | $\LU{r}{\tomega}_{m}$ | local angular velocity due to mesh node velocity only; for the computation, see below for the standard and alternative approach |
    | marker global angular velocity | $\LU{0}{\tomega}_{m} = \LU{0}{\tomega_{r}} + \LU{0r}{\Rot} \LU{r}{\tomega}_{m}$ | current global angular velocity |

    The rotation matrix and the angular velocity depend on `rotationsExponentialMap`: with 0,
    $\LU{0r}{\Rot}_{m} = \LU{0r}{\Rot} (\Im + \LU{r}{\tilde \ttheta}_{m})$, linearized; with 1 and 2 the
    exponential map of the table; with 2 (the default) the local angular velocity is also transformed by the
    tangent operator of the exponential map, $\mathbf{T}_{\exp}(\LU{r}{\ttheta}_{m}) \LU{r}{\tomega}_{m}$.

    

    #### Marker background

    The marker allows to realize a multi-point constraint (assuming that the marker is used in a joint constraint), 
    connecting to averaged nodal displacements and rotations (also known as RBE3 in NASTRAN), see e.g. [CITE:HeirmanDesmet2010]. 
    However, using Craig-Bampton RBE2 modes, will create RBE2 multi-point constraints for `ObjectFFRFreducedOrder` objects.

    For more information on the various quantities and their coordinate systems, see table above and [](#fig-markersuperelementrigid-sketch).
    

    (fig-markersuperelementrigid-sketch)=
    ```{figure} /docs/figures/MarkerSuperElementRigid.*
    :width: 400

    Sketch of marker nodes, exemplary node $i$, reference coordinates and marker coordinate system; note the difference of the center of the marker 'surface' (rectangle) marked with the red cross, and the averaged of the averaged local reference position.
    ```


    #### Marker quantities

    The marker provides a 'position' jacobian, which is the derivative of the global marker velocity w.r.t. the 
    object velocity coordinates $\dot \qv_{n_b}$,


    $$
                        \LU{0}{\Jm_{m,pos}} = \frac{\partial \LU{0}{\vv}_{m}}{\dot \qv_{n_b}} \, .
                        $$

    In case of `ObjectGenericODE2`, assuming pure displacement based nodes,
    the jacobian will consist of zeros and unit matrices $\Im$ ,


    $$
                        \LU{0}{\Jm_{m,pos}^{GenericODE2}} = \frac{\partial \LU{0}{\vv}_{m}}{\dot \qv_{n_b}} 
                              = \left[ \Null,\; \ldots,\; \Null,\; w_0 \Im,\; \Null,\; \ldots,\; \Null,\; w_1 \Im,\; \Null,\; \ldots,\; \Null \right]\, ,
                        $$

    in which the $\Im$ matrices are placed at the according indices of marker nodes.

    In case of `ObjectFFRFreducedOrder`, this jacobian is computed as weighted sum 
    of the position jacobians, see `ObjectFFRFreducedOrder`,


    $$
                        \LU{0}{\Jm_{m,pos}^{FFRFreduced}} = \frac{\partial \LU{0}{\vv}_{m}}{\dot \qv_{n_b}}
                              = \sum_i w_i \LU{0}{\Jm^{(i)}_\mathrm{pos}}
                              = \left[\Im, \; -\LU{0r}{\Rot} \left(\LU{r}{\ov\cRef} + \sum_i \LU{r}{\pv^{(i)}} \right) \LU{r}{\Gm},\;
                                      \sum_i w_i \LU{0r}{\Rot} \vr{\LU{r}{\tPsi_{r=3i}\tp}}{\LU{r}{\tPsi_{r=3i+1}\tp}}{\LU{r}{\tPsi_{r=3i+2}\tp}} \right] \, .
                        $$

    In `ObjectFFRFreducedOrder`, the jacobian usually affects all reduced coordinates.
    

    #### Standard approach for computation of rotation (`useAlternativeApproach = False`)

    As compared to `MarkerSuperElementPosition`, `MarkerSuperElementRigid` also links the marker to the orientation of 
    the set of nodes provided. For this reason, the check performed in `mbs.assemble()` will take care that the nodes are capable
    to describe rotations.
    The first approach, here called as a standard, follows the idea that displacements contribute to rotation are weighted by their quadratic distance, 
    cf. [CITE:HeirmanDesmet2010], and gives the (small rotation) rotation vector


    $$
                        \LU{r}{\ttheta}_{m} = \frac{\sum_i w_i \LU{r}{\pv_{ref}^{(i)}} \times \LU{r}{\uv^{(i)}}}{\sum_i w_i |\LU{r}{\pv_{ref}^{(i)}}|^2}
                        $$

    Note that $\pv_{ref}^{(i)}$ is not the reference position in the `ObjectFFRFreducedOrder` object, but it is relative to the midpoint reference position
    all marker nodes, given in $\LU{r}{\xv^\mathrm{avg}\cRef}$.
    Accordingly, the marker local angular velocity can be calculated as


    $$
                        \LU{r}{\tomega}_{m} = \LU{r}{\dot \ttheta}_{m} = \frac{\sum_i w_i \LU{r}{\tilde \pv_{ref}^{(i)}} \LU{r}{\vv_i}}{\sum_i w_i |\LU{r}{\pv_{ref}^{(i)}}|^2}
                        $$

    The marker also provides a `rotation' jacobian, which is the derivative of the marker angular velocity $\LU{0}{\tomega}_{m}$ w.r.t. the 
    object velocity coordinates $\dot \qv_{n_b}$,


    $$
                        \LU{0}{\Jm_{m,rot}} = \frac{\partial \LU{0}{\tomega}_{m}}{\partial \dot \qv_{n_b}}
                                          = \frac{\partial \LU{0r}{\Rot}(\LU{r}{\tomega_{r}} + \LU{r}{\tomega}_{m})}{\partial \dot \qv_{n_b}}
                                          = \LU{0r}{\Rot} \left(\frac{\partial \LU{r}{\tomega}_{r}}{\partial \dot \qv_{n_b}} + 
                                                           \frac{\sum_i w_i \LU{r}{\tilde \pv_{ref}^{(i)}} \LU{r}{\Jm_{pos}^{(i)}}}{\sum_i w_i |\LU{r}{\pv_{ref}^{(i)}}|^2} \right)
                        $$

    In case of `ObjectFFRFreducedOrder`, this jacobian is computed as


    $$
    \LU{0}{\Jm_{m,rot}^{FFRFreduced}} = \left[\Null,\; \LU{0r}{\Rot} \LU{r}{\Gm_{local}},\; 
                                                    \LU{0r}{\Rot} \frac{\sum_i w_i \LU{r}{\tilde \pv_{ref}^{(i)}} \LU{r}{\Jm_{pos,f}^{(i)}}}{\sum_i w_i |\LU{r}{\pv_{ref}^{(i)}}|^2} \right]
    $$ (eq-markersuperelementrigid-jacrotstandard)

    in which you should know that
    
    - we used $\frac{\partial \LU{r}{\tomega_{r}} }{\partial \dot \ttheta_r} = \LU{r}{\Gm_{local}}$,
    - $\ttheta_{r}$ represent the rotation parameters for the rigid body node of `ObjectFFRFreducedOrder`,
    - $\LU{r}{\Jm_{pos,f}^{(i)}}$ is the **local** jacobian, which only includes the flexible part of the local jacobian for a single mesh node, $\LU{r}{\Jm_{pos}^{(i)}}$ (note the small $r$ on the upper left), as defined in `ObjectFFRFreducedOrder`.

    For further quantities also consult the according description in `ObjectFFRFreducedOrder`.




    #### Alternative computation of rotation (`useAlternativeApproach = True`)

    Note that this approach is **still under development** and needs further validation. 
    However, tests show that this model is superior to the standard approach, as it improves the averaging of motion w.r.t. rotations
    at the marker nodes.

    In the alternative approach, the weighting matrix $\Wm$ 
    has the interpretation of an inertia tensor built from nodes using weights equal to node masses.
    In such an interpretation, the 'local angular momentum' w.r.t. the marker (averaged) position can be computed as 


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
    angular velocities in unsymmetric (w.r.t. the axis of rotation) distribition of mesh nodes. 
    This could even lead to spurious rotations or angular velocities in pure translatoric motion.

    In the alternative mode, the Jacobian for the rotation / angular velocity is defined as


    $$
                        \LU{0}{\Jm_{m,rot,alt}} = \frac{\partial \LU{0}{\tomega}_{m}}{\partial \dot \qv_{n_b}}
                                          = \frac{\partial \LU{0r}{\Rot}(\LU{r}{\tomega_{r}} + \LU{r}{\tomega}_{m})}{\partial \dot \qv_{n_b}}
                                          = \LU{0r}{\Rot} \left(\frac{\partial \LU{r}{\tomega}_{r}}{\partial \dot \qv_{n_b}}  + 
                                                                \Wm^{-1} \sum_i w_i \LU{r}{\tilde \pv_{ref}^{(i)}} \LU{r}{\Jm_{pos}^{(i)}}\right)
                        $$

    In case of `ObjectFFRFreducedOrder`, this jacobian is computed as


    $$
                        \LU{0}{\Jm_{m,rot,alt}^{FFRFreduced}} = \left[\Null,\; \LU{0r}{\Rot} \LU{r}{\Gm_{local}},\; 
                                                                        \LU{0r}{\Rot} \Wm^{-1} \sum_i w_i \LU{r}{\tilde \pv_{ref}^{(i)}} \LU{r}{\Jm_{pos,f}^{(i)}} \right]
                        $$

    see also the descriptions given after {eq}`eq-markersuperelementrigid-jacrotstandard` in the 'standard' approach.



     **EXAMPLE for marker on body 4, mesh nodes 10,11,12,13**:


    `MarkerSuperElementRigid(bodyNumber = 4, meshNodeNumber = [10, 11, 12, 13], weightingFactors = [0.25, 0.25, 0.25, 0.25], referencePosition=[0,0,0])`



     For detailed examples, see `TestModels`.
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
            defaultValue=CppValue('Vector3D({0.,0.,0.})', 'None', 'None (zero)'),
            description=r"""$\LU{r}{\ov_{ref}}$local marker SuperElement reference position offset used to correct the center point of the marker, which is computed from the weighted average of reference node positions (which may have some offset to the desired joint position). Note that this offset shall be small and larger offsets can cause instability in simulation models (better to have symmetric meshes at joints). The translation of localHT.""",
            partOfHT='localHT'),
        ItemParameter(type=THomogeneousTransformation, destination=DestComp+DestParam,
            pythonName='localHT',
            defaultValue=CppValue('HomogeneousTransformation()', 'None', 'None (identity)'),
            description=r"""$\LU{r}{\Hm}_{m}$the frame of the marker against the frame the marker computes from the mesh nodes, as homogeneous transformation: its translation is offset, its rotation turns the marker frame; a 4x4 matrix, its 16 values row by row or an exu.HT; None: not given; given together with offset, both must agree"""),
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
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst,
            pythonName='GetKinematicsJacobianRigid', args='const CSystemData& cSystemData, MarkerRigid<Real>& kinematics, MarkerTemp& temp',
            description=r'frame and velocities, and the position and rotation Jacobians into temp, without the marker data (#2745)'),
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst,
            pythonName='GetKinematicsRigid', args='const CSystemData& cSystemData, MarkerRigid<Real>& kinematics, MarkerTemp& temp',
            description=r'frame and velocities, without the Jacobians, which AddGeneralizedForceTorque forms (#2745)'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst,
            pythonName='AddGeneralizedForceTorque', args='const CSystemData& cSystemData, const Vector3D& force, const Vector3D& torque, MarkerTemp& temp, LinkedDataVector& ode2Lhs',
            description=r'add J_pos^T force + J_rot^T torque to ode2Lhs; the Jacobians formed here (#2745)'),
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
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeRotationMatrix',
            args='const Matrix3D& frameRotationMatrix, const Vector3D& weightedRotations, Matrix3D& rotationMatrix',
            description=r'the rotation matrix of the marker from the rotation of the floating frame and the weighted rotations, as rotationsExponentialMap chooses'),
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
    overallDescription=r'A position and orientation (rigid-body) marker attached to a kinematic tree. The marker is attached to the ObjectKinematicTree object and additionally needs a link number as well as a local position, similar to the SensorKinematicTree. The marker allows to attach loads (LoadForceVector and LoadTorqueVector) at arbitrary links or position. It also allows to attach connectors (e.g., spring dampers or actuators) to the kinematic tree. Finally, joint constraints can be attached, which allows for realization of closed loop structures. NOTE, however, that it is less efficient to attach many markers to a kinematic tree, therefor for forces or joint control use the structures available in kinematic tree whenever possible.',
    classType=ClassTypeMarker,
    miniExample=r"""    #a rigid body marker on link 0 of a kinematic tree with one prismatic joint along x
    nTree = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0.], initialCoordinates=[0.],
                                        initialCoordinates_t=[0.], numberOfODE2Coordinates=1))
    oTree = mbs.AddObject(ObjectKinematicTree(nodeNumber=nTree, jointTypes=[exu.JointType.PrismaticX], linkParents=[-1],
                                              jointTransformations=exu.Matrix3DList([np.eye(3)]),
                                              jointOffsets=exu.Vector3DList([[0,0,0]]),
                                              linkInertiasCOM=exu.Matrix3DList([np.eye(3)]),
                                              linkCOMs=exu.Vector3DList([[0,0,0]]), linkMasses=[2.]))
    mLink = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oTree, linkNumber=0, localPosition=[0.5,0,0]))
    mbs.AddLoad(LoadForceVector(markerNumber=mLink, loadVector=[1,0,0]))

    mbs.Assemble()
    mbs.SolveDynamic()

    #q = F/(2m)*t^2 at t=1
    exu.sys['testResult'] = mbs.GetNodeOutput(nTree, exu.OutputVariableType.Coordinates) #0.25
    """,
    miniExamplePerformanceTest={'numberOfSteps': 638920},
    detailedDescription=r"""    #### Marker quantities

    The link frame of link $n_l$ - its position $\LU{0}{\pv}_{l}$, rotation $\LU{0l}{\Rot}$, velocity
    $\LU{l}{\vv}_{l}$ and angular velocity $\LU{l}{\tomega}_{l}$ in link coordinates - comes from the joint
    transformations of `ObjectKinematicTree`, evaluated from the base to the link.

    | marker quantity | symbol | description |
    |---|---|---|
    | marker position | $\LU{0}{\pv}_{m} = \LU{0}{\pv}_{l} + \LU{0l}{\Rot} \LU{l}{\bv}$ | global position of the local position $\LU{l}{\bv}$ on link $n_l$ |
    | marker velocity | $\LU{0}{\vv}_{m} = \LU{0l}{\Rot} \left(\LU{l}{\vv}_{l} + \LU{l}{\tomega}_{l} \times \LU{l}{\bv} \right)$ | global velocity |
    | marker rotation matrix | $\LU{0m}{\Rot} = \LU{0l}{\Rot} \LU{lm}{\Rot}$ | the rotation of the link frame, turned by the rotation $\LU{lm}{\Rot}$ of `localHT`; the local position does not rotate the marker |
    | marker angular velocity | $\LU{0}{\tomega}_{m} = \LU{0l}{\Rot} \LU{l}{\tomega}_{l}$ | global; the local angular velocity is $\LU{lm}{\Rot}\tp \LU{l}{\tomega}_{l}$ |

    #### Jacobians

    The Jacobians have one column per link of the tree, $\qv$ being the joint coordinates. Only the link
    $n_l$ and its parents down to the base have non-zero columns; for such a link $j$, with its joint axis
    $\av_j$ and the origin $\LU{0}{\pv}_j$ of its joint frame, both global,

    $$
    \text{revolute:} \quad \Jm_{pos,j} = \av_j \times \left(\LU{0}{\pv}_{m} - \LU{0}{\pv}_j\right) , \quad \Jm_{rot,j} = \av_j ;
    \qquad
    \text{prismatic:} \quad \Jm_{pos,j} = \av_j , \quad \Jm_{rot,j} = \Null .
    $$

    A force and a torque on the marker act on the joint coordinates as
    $\Qm = \Jm_{pos}\tp \LU{0}{\fv} + \Jm_{rot}\tp \LU{0}{\ttau}$.

    The derivative of the Jacobians is not implemented: a connector that needs it raises an error with this
    marker; `newton.numericalDifferentiation.forODE2Connectors = True` computes the connector's Jacobian
    numerically instead.
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
            defaultValue=CppValue('Vector3D({0.,0.,0.})', 'None', 'None (zero)'),
            description=r"""$\LU{l}{\bv}$local (link-fixed) position of marker at link $n_l$, using the link ($n_l$) coordinate system; the translation of localHT""",
            partOfHT='localHT'),
        ItemParameter(type=THomogeneousTransformation, destination=DestComp+DestParam,
            pythonName='localHT',
            defaultValue=CppValue('HomogeneousTransformation()', 'None', 'None (identity)'),
            description=r"""$\LU{l}{\Hm}_{m}$the frame of the marker in the link frame, as homogeneous transformation: its translation is localPosition, its rotation turns the marker frame against the link; a 4x4 matrix, its 16 values row by row or an exu.HT; None: not given; given together with localPosition, both must agree"""),
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
    overallDescription=r'A Marker attached to all coordinates of an object (currently only body is possible), e.g. to apply special constraints or loads on all coordinates. The measured coordinates INCLUDE reference + current coordinates.',
    classType=ClassTypeMarker,
    miniExample=r"""    #all coordinates of an object, here of a ObjectGenericODE2 with two free coordinates, tied by a
    #coordinate vector constraint X1 q - X0 q_ground = offset, which is q0 - q1 = 0
    node = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=2, referenceCoordinates=[0,0],
                                       initialCoordinates=[0,0], initialCoordinates_t=[0,0]))
    oGeneric = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node], massMatrix=np.eye(2)))
    mAll = mbs.AddMarker(MarkerObjectODE2Coordinates(objectNumber=oGeneric))
    mNone = mbs.AddMarker(MarkerNodeCoordinates(nodeNumber=nGround)) #the ground node has no coordinates
    mbs.AddObject(ObjectConnectorCoordinateVector(markerNumbers=[mNone, mAll], scalingMarker0=np.zeros((1,0)),
                                                 scalingMarker1=[[1,-1]], offset=[0]))
    mbs.AddLoad(LoadCoordinate(markerNumber=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=0)), load=2))

    mbs.Assemble()
    mbs.SolveDynamic()

    #both coordinates move together: a = F/2 = 1, q1 = a/2*t^2 at t=1
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)[1] #0.5
    """,
    miniExamplePerformanceTest={'numberOfSteps': 905650},
    detailedDescription=r"""    #### Marker quantities

    | quantity | symbol | as computed |
    |---|---|---|
    | coordinates | $\cv = \qv\cRef + \qv$ | all ABRV:ODE2 coordinates of the body, node after node in the order of its nodes, **including** their reference values |
    | their velocities | $\dot\cv$ | the time derivatives |

    #### Jacobians

    The unit matrix of the size of the body's coordinates. On a body without coordinates (ground) the
    values and the Jacobian are empty.
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
    overallDescription=r'A special Marker attached to a 2D ANCF beam finite element with cubic interpolation and 8 coordinates.',
    classType=ClassTypeMarker,
    miniExample=r"""    from exudyn.beams import GenerateStraightLineANCFCable2D
    #the shape of an ANCF cable element as line segments, for contact: a cantilever falls onto a circle
    cable = ObjectANCFCable2D(massPerLength=1, bendingStiffness=10, axialStiffness=1e4,
                              bendingDamping=0.1)
    [nodes, elements, *_] = GenerateStraightLineANCFCable2D(mbs, positionOfNode0=[0,0,0], positionOfNode1=[1,0,0],
                            numberOfElements=4, cableTemplate=cable, massProportionalLoad=[0,-9.81,0],
                            fixedConstraintsNode0=[1,1,0,1])
    mCircle = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0.8,-0.2,0]))
    nSegments = 4
    for e in elements:
        mShape = mbs.AddMarker(MarkerBodyCable2DShape(bodyNumber=e, numberOfSegments=nSegments))
        nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=nSegments, initialCoordinates=[0.1]*nSegments))
        mbs.AddObject(ObjectContactCircleCable2D(markerNumbers=[mCircle, mShape], nodeNumber=nData,
                                                 numberOfContactSegments=nSegments, circleRadius=0.1,
                                                 contactStiffness=1e4))

    mbs.Assemble()
    mbs.SolveDynamic()

    #the tip rests beyond the circle, whose top is at y=-0.1
    exu.sys['testResult'] = mbs.GetNodeOutput(nodes[-1], exu.OutputVariableType.Position)[1]
    """,
    miniExamplePerformanceTest={'numberOfSteps': 57080},
    detailedDescription=r"""    #### Attached to

    A planar ANCF cable element, `ObjectANCFCable2D` or `ObjectALEANCFCable2D`; the marker is made for
    the contact of a circle with the cable (`ObjectContactCircleCable2D`,
    `ObjectContactFrictionCircleCable2D`), which divides the element into `numberOfSegments` segments.
    `Assemble()` refuses it on any other body.

    #### Marker quantities

    The positions and velocities of the `numberOfSegments`+1 equidistant points of the element, at the
    distance `verticalOffset` from the axis in the local $y$-direction, as pairs $(x,\,y)$; and the
    length of the element and, for the ALE element, its axial coordinate, which the contact needs.

    #### Jacobians

    For each point, the two rows of the shape functions of the element, $\partial \LU{0}{\pv}_j /
    \partial \qv$: a force at a segment point acts on the element's coordinates through them.
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
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerBodyCable2DCoordinates   ++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerBodyCable2DCoordinates',
    cParentClass=ParentClassCMarker,
    overallDescription=r'A special Marker attached to the coordinates of a 2D ANCF beam finite element with cubic interpolation.',
    classType=ClassTypeMarker,
    miniExample=r"""    from exudyn.beams import GenerateStraightLineANCFCable2D
    #the coordinates of ANCF cable elements for a sliding joint: a mass point slides along a clamped, stiff cable
    cable = ObjectANCFCable2D(massPerLength=1, bendingStiffness=1e4, axialStiffness=1e6)
    [nodes, elements, *_] = GenerateStraightLineANCFCable2D(mbs, positionOfNode0=[0,0,0], positionOfNode1=[2,0,0],
                            numberOfElements=4, cableTemplate=cable,
                            fixedConstraintsNode0=[1,1,1,1], fixedConstraintsNode1=[1,1,1,1])
    nMass = mbs.AddNode(NodePoint2D(referenceCoordinates=[0.6,0]))
    mbs.AddObject(ObjectMassPoint2D(nodeNumber=nMass, mass=1))
    mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
    mbs.AddLoad(LoadForceVector(markerNumber=mMass, loadVector=[1,0,0]))

    cableMarkers = [mbs.AddMarker(MarkerBodyCable2DCoordinates(bodyNumber=e)) for e in elements]
    offsets = [0.5*i for i in range(4)] #the element length is 0.5
    nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=2, initialCoordinates=[1, 0.6])) #element 1, sliding coordinate
    mbs.AddObject(ObjectJointSliding2D(markerNumbers=[mMass, cableMarkers[1]], slidingMarkerNumbers=cableMarkers,
                                       slidingMarkerOffsets=offsets, nodeNumber=nData))

    mbs.Assemble()
    mbs.SolveDynamic()

    #the mass slides: x = 0.6 + F/(2m)*t^2 at t=1, the stiff cable deflects little
    exu.sys['testResult'] = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Position)[0] #1.1
    """,
    miniExamplePerformanceTest={'numberOfSteps': 78860},
    detailedDescription=r"""    #### Attached to

    A planar ANCF cable element, `ObjectANCFCable2D` or `ObjectALEANCFCable2D`; the marker is made for
    the joints that slide along a cable, `ObjectJointSliding2D` and `ObjectJointALEMoving2D`, which
    evaluate the shape functions themselves. `Assemble()` refuses it on any other body.

    #### Marker quantities

    The 8 nodal coordinates of the element - position and slope of both nodes, **including** their
    reference values - and their velocities, and the length of the element.

    #### Jacobians

    The unit matrix of the 8 coordinates: the joint computes the action on each coordinate.
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
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   MarkerBodyBeamShape   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='MarkerBodyBeamShape',
    cParentClass=ParentClassCMarker,
    overallDescription=r'A special Marker attached to a 3D beam finite element which provides at least position and tangent to the beam axis.',
    classType=ClassTypeMarker,
    miniExample=r"""    #the shape of 3D ANCF cable elements for a sliding joint: a mass point slides along a clamped, stiff cable
    from exudyn.beams import GenerateStraightLineANCFCable
    cable = ObjectANCFCable(massPerLength=1, bendingStiffness=1e4, axialStiffness=1e6)
    [nodes, elements, *_] = GenerateStraightLineANCFCable(mbs, positionOfNode0=[0,0,0], positionOfNode1=[2,0,0],
                            numberOfElements=4, cableTemplate=cable,
                            fixedConstraintsNode0=[1,1,1, 1,1,1], fixedConstraintsNode1=[1,1,1, 1,1,1])
    nMass = mbs.AddNode(NodePoint(referenceCoordinates=[0.6,0,0]))
    mbs.AddObject(ObjectMassPoint(nodeNumber=nMass, mass=1))
    mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
    mbs.AddLoad(LoadForceVector(markerNumber=mMass, loadVector=[1,0,0]))

    cableMarkers = [mbs.AddMarker(MarkerBodyBeamShape(bodyNumber=e)) for e in elements]
    offsets = [0.5*i for i in range(4)] #the element length is 0.5
    nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=2, initialCoordinates=[1, 0.6])) #element 1, sliding coordinate
    mbs.AddObject(ObjectJointSliding(markerNumbers=[mMass, cableMarkers[1]], slidingMarkerNumbers=cableMarkers,
                                     slidingMarkerOffsets=offsets, nodeNumber=nData, constrainRotations=[0,0,0]))

    mbs.Assemble()
    mbs.SolveDynamic()

    #the mass slides: x = 0.6 + F/(2m)*t^2 at t=1, the stiff cable deflects little
    exu.sys['testResult'] = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Position)[0] #1.1
    """,
    miniExamplePerformanceTest={'numberOfSteps': 27920},
    detailedDescription=r"""    #### Attached to

    A spatial ANCF cable element, `ObjectANCFCable`; the marker is made for `ObjectJointSliding`, which
    evaluates the shape functions itself, and it serves no other beam: the implementation evaluates the
    shape functions of `ObjectANCFCable`. `Assemble()` refuses it on any other body.

    #### Marker quantities

    The coordinates of the element - position and slope of both nodes, **including** their reference
    values - and their velocities, and the length of the element.

    #### Jacobians

    The unit matrix of the element's coordinates: the joint computes the action on each coordinate.
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
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))
