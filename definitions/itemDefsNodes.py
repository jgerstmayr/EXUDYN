#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Node item definitions
#
# Details:  The input of the generators for node items.
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
# Contents: NodePoint, NodePoint2D, NodeRigidBodyEP, NodeRigidBodyRxyz, NodeRigidBodyRotVecLG, NodeRigidBody2D, ...
#
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from definitionTypes import *
from outputVariableTypes import *
from outputVariableDescriptions import *

definitions = []
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   NodePoint   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='NodePoint',
    cParentClass=ParentClassCNodeODE2,
    overallDescription=r"""A 3D point node for point masses or solid finite elements which has 3 displacement degrees of freedom for ABRV:ODE2.""",
    classType=ClassTypeNode,
    examples=['Examples/basicTutorial2024.py', 'Examples/springDamperTutorial.py', 'Examples/springDamperTutorialNew.py', 'Examples/cartesianSpringDamper.py', 'Examples/coordinateSpringDamper.py'],
    miniExample=r"""    #a point mass moving freely: reference position, initial displacement and initial velocity
    node = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0],
                                 initialCoordinates=[0,0.5,0],   #displacement from the reference
                                 initialVelocities=[2,0,0]))
    mbs.AddObject(ObjectMassPoint(nodeNumber=node, physicsMass=1))

    mbs.Assemble()
    mbs.SolveDynamic() #default: 1 second

    #position = reference + displacement: [1+0+2*1, 0.5, 0]
    exu.sys['testResult'] = sum(mbs.GetNodeOutput(node, exu.OutputVariableType.Position)) #3.5
    """,
    miniExamplePerformanceTest={'numberOfSteps': 1673150},
    detailedDescription=r"""    #### Coordinates

    | index | symbol | kind | meaning | frame |
    |---|---|---|---|---|
    | 0, 1, 2 | $q_0,\,q_1,\,q_2$ | ABRV:ODE2 | displacement of the node in $x$, $y$ and $z$ | the frame of the object, usually global |

    #### Configuration

    In any configuration, the position of the node is its reference position plus its displacement,

    $$
    \pv\cConfig = \pv\cRef + \uv\cConfig, \quad \uv\cConfig = [q_0,\,q_1,\,q_2]\cConfig\tp .
    $$

    The coordinates are the displacements themselves, and their time derivatives are the velocity and
    the acceleration of the node.

    #### Frame and interpretation

    The node defines no frame; the object that uses it does. `ObjectMassPoint` reads the coordinates in
    the global frame. `ObjectFFRF` uses points as the nodes of its finite element mesh and reads their
    coordinates in the frame of its rigid body node (node 0): there, the global position of a mesh node
    follows from the object, not from the node alone.

    #### Action on the equations of motion

    The three coordinates lead to three ABRV:ODE2 equations, which the object provides; for
    `ObjectMassPoint` they are the residuals of the forces in the global frame. A force $\fv$ acting on
    the node through `MarkerNodePosition` enters them with the position Jacobian
    $\partial \pv / \partial \qv = \ImThree$, that is, as it is.

    **Example**: see [](#sec-item-objectmasspoint)
    """,
    mainParentClass=MainParentClassMainNode,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\pv\cConfig = [p_0,\,p_1,\,p_2]\cConfig\tp= \uv\cConfig + \pv\cRef$global 3D position vector of node; $\uv\cRef=0$"""),
        ItemOutputVariable(OVDisplacement, r"""$\uv\cConfig = [q_0,\,q_1,\,q_2]\cConfig\tp$global 3D displacement vector of node"""),
        ItemOutputVariable(OVVelocity, r"""$\vv\cConfig = [\dot q_0,\,\dot q_1,\,\dot q_2]\cConfig\tp$global 3D velocity vector of node"""),
        ItemOutputVariable(OVAcceleration, r"""$\av\cConfig = \ddot \qv\cConfig = [\ddot q_0,\,\ddot q_1,\,\ddot q_2]\cConfig\tp$global 3D acceleration vector of node"""),
        ItemOutputVariable(OVCoordinatesTotal, r"""$\cv\cConfig = \uv\cConfig + \pv\cRef$ displacement plus reference coordinates of node"""),
        ItemOutputVariable(OVCoordinates, r"""$\cv\cConfig = \uv\cConfig = [q_0,\,q_1,\,q_2]\tp\cConfig$ coordinate vector of node"""),
        ItemOutputVariable(OVCoordinates_t, r"""$\dot\cv\cConfig = \vv\cConfig = [\dot q_0,\,\dot q_1,\,\dot q_2]\tp\cConfig$ velocity coordinates vector of node"""),
        ItemOutputVariable(OVCoordinates_tt, r"""$\ddot\cv\cConfig = \av\cConfig = [\ddot q_0,\,\ddot q_1,\,\ddot q_2]\tp\cConfig$ acceleration coordinates vector of node"""),
        ItemOutputVariable(OVRotationMatrix, OVDIdentityMatrixForCompleteness),
        ItemOutputVariable(OVRotation, OVDZeroVectorForCompleteness),
        ItemOutputVariable(OVAngularVelocity, OVDZeroVectorForCompleteness),
        ItemOutputVariable(OVAngularVelocityLocal, OVDZeroVectorForCompleteness),
        ],
    pythonShortName='Point',
    visuParentClass=VisuParentClassVisualizationNode,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"node's unique name"),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='referenceCoordinates',
            defaultValue=DVZeroVector3D,
            description=r"""$\qv\cRef = [q_0,\,q_1,\,q_2]\tp\cRef = \pv\cRef = [r_0,\,r_1,\,r_2]\tp$reference coordinates of node, e.g. ref. coordinates for finite elements; global position of node without displacement"""),
        ItemParameter(type=TVectorND(3), destination=DestMain+DestParam,
            pythonName='initialCoordinates',
            defaultValue=DVZeroVector3D,
            description=r"""$\qv\cIni = [q_0,\,q_1,\,q_2]\cIni\tp = \uv\cIni = [u_0,\,u_1,\,u_2]\cIni\tp$initial displacement coordinate"""),
        ItemParameter(type=TVectorND(3), destination=DestMain+DestParam,
            pythonName='initialVelocities',
            cplusplusName='initialCoordinates_t',
            defaultValue=DVZeroVector3D,
            description=r"""$\dot\qv\cIni = \vv\cIni = [\dot q_0,\,\dot q_1,\,\dot q_2]\cIni\tp$initial velocity coordinate"""),
        ItemFunctionDef('GetNumberOfODE2Coordinates',
            implementation='return 3;'),
        ItemTypes('Node', ['Position'],
            description=r'return node type (for node treatment in computation)'),
        ItemFunctionDef('GetPosition',
            description='return configuration dependent position of node'),
        ItemFunctionDef('GetVelocity',
            description='return configuration dependent velocity of node'),
        ItemFunctionDef('GetAcceleration'),
        ItemFunctionDef('GetPositionJacobian',
            implementation='value.SetScalarMatrix(3,1.);'),
        ItemFunctionDef('GetRotationJacobianTTimesVector_q',
            implementation='jacobian_q.SetNumberOfRowsAndColumns(0, 0);'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "Point";',
            description=r"Get type name of node (without keyword 'Node'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('GetReferenceCoordinateVector',
            implementation='return parameters.referenceCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector',
            implementation='return parameters.initialCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector_t',
            implementation='return parameters.initialCoordinates_t;'),
        ItemFunctionDef('GetOutputVariable'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   NodePoint2D   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='NodePoint2D',
    cParentClass=ParentClassCNodeODE2,
    overallDescription=r"""A 2D point node for point masses or solid finite elements which has 2 displacement degrees of freedom for ABRV:ODE2.""",
    classType=ClassTypeNode,
    examples=['Examples/pendulum2Dconstraint.py', 'Examples/SliderCrank.py', 'Examples/SpringDamperMassUserFunction.py', 'Examples/slidercrankWithMassSpring.py'],
    miniExample=r"""    #a planar point mass under gravity, thrown with an initial velocity
    node = mbs.AddNode(NodePoint2D(referenceCoordinates=[0,0], initialVelocities=[1,2]))
    oMass = mbs.AddObject(ObjectMassPoint2D(nodeNumber=node, physicsMass=1))
    mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=node))
    mbs.AddLoad(LoadForceVector(markerNumber=mMass, loadVector=[0,-9.81,0]))

    mbs.Assemble()
    mbs.SolveDynamic()

    #y = v0*t - g/2*t^2 at t=1
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[1] #2-4.905=-2.905
    """,
    miniExamplePerformanceTest={'numberOfSteps': 1614470},
    detailedDescription=r"""    #### Coordinates

    | index | symbol | kind | meaning | frame |
    |---|---|---|---|---|
    | 0, 1 | $q_0,\,q_1$ | ABRV:ODE2 | displacement of the node in $x$ and $y$ | global |

    #### Configuration

    In any configuration, the position of the node is its reference position plus its displacement,
    with a third component that is always zero,

    $$
    \pv\cConfig = \vr{r_{0}}{r_{1}}{0}\cRef + \vr{q_0}{q_1}{0}\cConfig .
    $$

    #### Frame and interpretation

    The coordinates are displacements in the global $x$-$y$ plane; `ObjectMassPoint2D` is the object
    that uses them.

    #### Action on the equations of motion

    The two coordinates lead to two ABRV:ODE2 equations, the residuals of the forces in $x$ and $y$; a
    force of a load or connector enters them with its first two components.

    **Example**: see [](#sec-item-objectmasspoint2d)
    """,
    mainParentClass=MainParentClassMainNode,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\pv\cConfig = [p_0,\,p_1,\,0]\cConfig\tp= \uv\cConfig + \pv\cRef$global 3D position vector of node; $\uv\cRef=0$"""),
        ItemOutputVariable(OVDisplacement, r"""$\uv\cConfig = [q_0,\,q_1,\,0]\cConfig\tp$global 3D displacement vector of node"""),
        ItemOutputVariable(OVVelocity, r"""$\vv\cConfig = [\dot q_0,\,\dot q_1,\,0]\cConfig\tp$global 3D velocity vector of node"""),
        ItemOutputVariable(OVAcceleration, r"""$\av\cConfig = [\ddot q_0,\,\ddot q_1,\,0]\cConfig\tp$global 3D acceleration vector of node"""),
        ItemOutputVariable(OVCoordinatesTotal, r"""$\cv\cConfig = \uv\cConfig + \pv\cRef$ displacement plus reference coordinates of node"""),
        ItemOutputVariable(OVCoordinates, r"""$\cv\cConfig = [q_0,\,q_1]\tp\cConfig$ coordinate vector of node"""),
        ItemOutputVariable(OVCoordinates_t, r"""$\dot\cv\cConfig = [\dot q_0,\,\dot q_1]\tp\cConfig$ velocity coordinates vector of node"""),
        ItemOutputVariable(OVCoordinates_tt, r"""$\ddot\cv\cConfig = \av\cConfig = [\ddot q_0,\,\ddot q_1]\tp\cConfig$ acceleration coordinates vector of node"""),
        ItemOutputVariable(OVRotationMatrix, OVDIdentityMatrixForCompleteness),
        ItemOutputVariable(OVRotation, OVDZeroVectorForCompleteness),
        ItemOutputVariable(OVAngularVelocity, OVDZeroVectorForCompleteness),
        ItemOutputVariable(OVAngularVelocityLocal, OVDZeroVectorForCompleteness),
        ],
    pythonShortName='Point2D',
    visuParentClass=VisuParentClassVisualizationNode,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"node's unique name"),
        ItemParameter(type=TVectorND(2), destination=DestComp+DestParam,
            pythonName='referenceCoordinates',
            defaultValue='Vector2D({0.,0.})',
            description=r"""$\qv\cRef = [q_0,\,q_1]\tp\cRef = \pv\cRef = [r_0,\,r_1]\tp$reference coordinates of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement"""),
        ItemParameter(type=TVectorND(2), destination=DestMain+DestParam,
            pythonName='initialCoordinates',
            defaultValue='Vector2D({0.,0.})',
            description=r"""$\qv\cIni = [q_0,\,q_1]\cIni\tp = [u_0,\,u_1]\cIni\tp$initial displacement coordinate"""),
        ItemParameter(type=TVectorND(2), destination=DestMain+DestParam,
            pythonName='initialVelocities',
            cplusplusName='initialCoordinates_t',
            defaultValue='Vector2D({0.,0.})',
            description=r"""$\dot\qv\cIni = \vv\cIni = [\dot q_0,\,\dot q_1]\cIni\tp$initial velocity coordinate"""),
        ItemFunctionDef('GetNumberOfODE2Coordinates',
            implementation='return 2;'),
        ItemTypes('Node', ['Position2D'],
            description=r'return node type (for node treatment in computation)'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetAcceleration'),
        ItemFunctionDef('GetPositionJacobian',
            implementation='value.SetMatrix(3,2,{1.f,0.f,0.f,1.f,0.f,0.f});'),
        ItemFunctionDef('GetRotationJacobianTTimesVector_q',
            implementation='jacobian_q.SetNumberOfRowsAndColumns(0, 0);'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "Point2D";',
            description=r"Get type name of node (without keyword 'Node'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('GetReferenceCoordinateVector',
            implementation='return parameters.referenceCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector',
            implementation='return parameters.initialCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector_t',
            implementation='return parameters.initialCoordinates_t;'),
        ItemFunctionDef('GetOutputVariable'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   NodeRigidBodyEP   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='NodeRigidBodyEP',
    addProtectedC=r"""    static constexpr Index nRotationCoordinates = 4;//AUTO: 
    static constexpr Index nDisplacementCoordinates = 3;
    Index globalAECoordinateIndex;
""",
    addPublicC=r"""    static constexpr bool useNodeAE = true;//AUTO: decide old/new mode for EP constraints; will be always true in future
""",
    cParentClass=ParentClassCNodeRigidBody,
    overallDescription=r"""A 3D rigid body node based on Euler parameters for rigid bodies or beams. The node has 3 displacement coordinates (representing displacement of reference point $\LU{0}{\rv}$) and four rotation coordinates (Euler parameters = unit quaternions).""",
    classType=ClassTypeNode,
    examples=['Examples/rigidBodyTutorial.py', 'Examples/rigidBodyTutorial2.py', 'Examples/rigidBodyTutorial3.py', 'Examples/rigidBodyTutorial3withMarkers.py', 'Examples/fourBarMechanism3D.py'],
    miniExample=r"""    #a rigid body spinning about its z-axis; the velocity coordinates are the time derivatives of the Euler parameters
    omega = [0,0,0.5*np.pi]
    ep0 = eulerParameters0
    node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0.5,0.2,0.1]+ep0,
                                       initialVelocities=[0,0,0]+list(AngularVelocity2EulerParameters_t(omega, ep0))))
    inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
    mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(),
                                  physicsInertia=inertia.GetInertia6D()))

    mbs.Assemble()
    mbs.SolveDynamic()

    #the node adds the constraint of the Euler parameters itself; the angle about z after 1 second:
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Rotation)[2] #pi/2, to the accuracy of the time integration
    """,
    miniExamplePerformanceTest={'numberOfSteps': 650680},
    detailedDescription=r"""    #### Coordinates

    | index | symbol | kind | meaning | frame |
    |---|---|---|---|---|
    | 0, 1, 2 | $q_0,\,q_1,\,q_2$ | ABRV:ODE2 | displacement of the reference point of the body | global |
    | 3, 4, 5, 6 | $\psi_0,\,\psi_1,\,\psi_2,\,\psi_3$ | ABRV:ODE2 | change of the four Euler parameters (unit quaternion) against their reference values | - |
    | - | $\lambda_\theta$ | ABRV:AE | the Lagrange multiplier of the Euler parameter constraint, if `addConstraintEquation = True` | - |

    #### Configuration

    The position of the reference point and the Euler parameters $\ttheta$ are the sums of reference and
    current coordinates,

    $$
    \pv\cConfig = \pv\cRef + \uv\cConfig, \quad \ttheta\cConfig = \tpsi\cRef + \tpsi\cConfig .
    $$

    The reference Euler parameters must be a unit quaternion - $[1,\,0,\,0,\,0]$ for no rotation; the
    default of zeros is not one, which `CreateRigidBody` takes care of. The rotation matrix, as a
    function of $\ttheta=[\theta_0,\,\theta_1,\,\theta_2,\,\theta_3]\tp$, transforms a local
    (body-fixed) position $\pLocB = \LU{b}{[b_0,\,b_1,\,b_2]}\tp$ into the global frame,
    $\LU{0}{\pLoc}\cConfig = \LU{0b}{\Rot}\cConfig \LU{b}{\pLoc}$, with

    $$
    \LU{0b}{\Rot} = \mr{-2\theta_3^2 - 2\theta_2^2+1}{-2\theta_3\theta_0+2\theta_2\theta_1}{2\theta_3\theta_1+2\theta_2\theta_0}
                       {2\theta_3\theta_0+2\theta_2\theta_1}{-2\theta_3^2-2\theta_1^2+1}{2\theta_3\theta_2-2\theta_1\theta_0}
                       {-2\theta_2\theta_0+2\theta_3\theta_1}{2\theta_3\theta_2+2\theta_1\theta_0}{-2\theta_2^2-2\theta_1^2+1}
    $$

    #### Frame and interpretation

    The displacement is given in the global frame, and the Euler parameters describe the rotation of the
    body frame $b$ against the global frame. Every object using the node reads it this way.

    #### Action on the equations of motion

    The velocity transformation relates the time derivatives of the Euler parameters to the angular
    velocity, in the global or in the body frame,

    $$
    \begin{aligned}
    \LU{0}{\tomega} &= \LU{0}{\Gm} \dot \ttheta, \\
    \LU{b}{\tomega} &= \LU{b}{\Gm} \dot \ttheta.
    \end{aligned}
    $$ (eq-noderigidbodyep-gm)

    All seven coordinates lead to ABRV:ODE2 equations, which the object provides. The first three are
    the residuals of the forces in the global frame. The last four are the torque equations projected
    with the transposed velocity transformation: a torque $\LU{b}{\ttau}$ in the body frame enters
    them as $\LU{b}{\Gm\tp} \LU{b}{\ttau}$, a torque $\LU{0}{\ttau}$ in the global frame as
    $\LU{0}{\Gm\tp} \LU{0}{\ttau}$, see {eq}`eq-noderigidbodyep-gm` and the equations of motion of
    [](#sec-item-objectrigidbody).

    #### Constraint of the Euler parameters

    Four parameters for three rotations need one constraint. With `addConstraintEquation = True` the
    node adds it itself, as one algebraic equation with the multiplier $\lambda_\theta$: on the position
    level (index 3)

    $$
    \ttheta\tp \ttheta - 1 = 0,
    $$

    or on the velocity level (index 2) $2\,\ttheta\tp \dot\ttheta = 0$. With
    `addConstraintEquation = False` it is left to the model, e.g. a `CoordinateVectorConstraint`.

    For creating a `NodeRigidBodyEP` together with a rigid body, use `CreateRigidBody`, see
    [](#sec-mainsystemextensions-createrigidbody).
    """,
    mainParentClass=MainParentClassMainNode,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}\cConfig = \LU{0}{[p_0,\,p_1,\,p_2]}\cConfig\tp= \LU{0}{\uv}\cConfig + \LU{0}{\pv}\cRef$global 3D position vector of node; $\uv\cRef=0$"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv}\cConfig = [q_0,\,q_1,\,q_2]\cConfig\tp$global 3D displacement vector of node"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv}\cConfig = [\dot q_0,\,\dot q_1,\,\dot q_2]\cConfig\tp$global 3D velocity vector of node"""),
        ItemOutputVariable(OVAcceleration, OVDAccelerationNode),
        ItemOutputVariable(OVCoordinatesTotal, OVDCoordinatesTotalNodeRotation),
        ItemOutputVariable(OVCoordinates, r"""$\cv\cConfig = [q_0,\,q_1,\,q_2, \,\psi_0,\,\psi_1,\,\psi_2,\,\psi_3]\tp\cConfig$ coordinate vector of node, having 3 displacement coordinates and 4 Euler parameters"""),
        ItemOutputVariable(OVCoordinates_t, r"""$\dot\cv\cConfig = [\dot q_0,\,\dot q_1,\,\dot q_2, \,\dot \psi_0,\,\dot \psi_1,\,\dot \psi_2,\,\dot \psi_3]\tp\cConfig$ velocity coordinates vector of node"""),
        ItemOutputVariable(OVCoordinates_tt, r"""$\ddot\cv\cConfig = [\ddot q_0,\,\ddot q_1,\,\ddot q_2, \,\ddot \psi_0,\,\ddot \psi_1,\,\ddot \psi_2,\,\ddot \psi_3]\tp\cConfig$ acceleration coordinates vector of node"""),
        ItemOutputVariable(OVRotationMatrix, OVDRotationMatrixRowMajor),
        ItemOutputVariable(OVRotation, r"""$[\varphi_0,\,\varphi_1,\,\varphi_2]\tp\cConfig$vector with 3 components of the Euler/Tait-Bryan angles in xyz-sequence ($\LU{0b}{\Rot}\cConfig=:\Rot_0(\varphi_0) \cdot \Rot_1(\varphi_1) \cdot \Rot_2(\varphi_2)$), recomputed from rotation matrix"""),
        ItemOutputVariable(OVAngularVelocity, OVDAngularVelocityNode),
        ItemOutputVariable(OVAngularVelocityLocal, OVDAngularVelocityLocalNode),
        ItemOutputVariable(OVAngularAcceleration, r"""$\LU{0}{\talpha}\cConfig = \LU{0}{[\alpha_0,\,\alpha_1,\,\alpha_2]}\cConfig\tp$global 3D angular acceleration vector of node"""),
        ],
    pythonShortName='RigidEP',
    visuParentClass=VisuParentClassVisualizationNode,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"node's unique name"),
        ItemParameter(type=TVectorND(7), destination=DestComp+DestParam,
            pythonName='referenceCoordinates',
            defaultValue='Vector7D({0.,0.,0., 0.,0.,0.,0.})',
            description=r"""$\qv\cRef = [q_0,\,q_1,\,q_2,\,\psi_0,\,\psi_1,\,\psi_2,\,\psi_3]\tp\cRef = [\pv\tp\cRef,\,\tpsi\tp\cRef]\tp$reference coordinates (3 position coordinates and 4 Euler parameters) of node ==> e.g. ref. coordinates for finite elements or reference position of rigid body (e.g. for definition of joints)"""),
        ItemParameter(type=TVectorND(7), destination=DestMain+DestParam,
            pythonName='initialCoordinates',
            defaultValue='Vector7D({0.,0.,0., 0.,0.,0.,0.})',
            description=r"""$\qv\cIni = [q_0,\,q_1,\,q_2,\,\psi_0,\,\psi_1,\,\psi_2,\,\psi_3]\tp\cIni = [\uv\tp\cIni,\,\tpsi\tp\cIni]\tp$initial displacement coordinates and 4 Euler parameters relative to reference coordinates"""),
        ItemParameter(type=TVectorND(7), destination=DestMain+DestParam,
            pythonName='initialVelocities',
            cplusplusName='initialCoordinates_t',
            defaultValue='Vector7D({0.,0.,0., 0.,0.,0.,0.})',
            description=r"""$\dot \qv\cIni = [\dot q_0,\,\dot q_1,\,\dot q_2,\,\dot \psi_0,\,\dot \psi_1,\,\dot \psi_2,\,\dot \psi_3]\tp\cIni = [\dot \uv\tp\cIni,\,\dot \tpsi\tp\cIni]\tp$initial velocity coordinates: time derivatives of initial displacements and Euler parameters"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='addConstraintEquation',
            defaultValue=True,
            description=r'True: automatically add Euler parameter constraint for node; False: Euler parameter constraint is not added, must be done manually (e.g., with CoordinateVectorConstraint)'),
        ItemFunctionDef('SetGlobalAECoordinateIndex',
            implementation='globalAECoordinateIndex = globalIndex;'),
        ItemFunctionDef('GetGlobalAECoordinateIndex',
            implementation='return globalAECoordinateIndex;'),
        ItemFunctionDef('GetNumberOfODE2Coordinates',
            implementation='return 7;'),
        ItemFunctionDef('GetNumberOfAECoordinates',
            implementation='return (Index)parameters.addConstraintEquation;',
            description='return number of (internal) algebraic eq. coordinates'),
        ItemFunctionDef('GetNumberOfDisplacementCoordinates',
            implementation='return nDisplacementCoordinates;'),
        ItemFunctionDef('GetNumberOfRotationCoordinates',
            implementation='return nRotationCoordinates;'),
        ItemFunctionDef('GetAlgebraicEquationsSize',
            implementation='return (Index)(useNodeAE&&parameters.addConstraintEquation);',
            description=r"""number of ABRV:AE equations, may be different from algebraic coordinates: if only coordinates are provided, but equations provided by other objects (ObjectRigidBody)"""),
        ItemTypes('Node', ['Position', 'Orientation', 'RigidBody', 'RotationEulerParameters'],
            description=r'return node type (for node treatment in computation)'),
        ItemFunctionDef('GetNodeGroup',
            implementation='return (CNodeGroup)((Index)CNodeGroup::ODE2variables + (Index)CNodeGroup::AEvariables);'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetAcceleration'),
        ItemFunctionDef('GetRotationMatrix'),
        ItemFunctionDef('GetAngularVelocity',
            description='return configuration dependent angular velocity of node; returns always a 3D Vector'),
        ItemFunctionDef('GetAngularVelocityLocal'),
        ItemFunctionDef('GetAngularAcceleration'),
        ItemFunctionDef('GetPositionJacobian'),
        ItemFunctionDef('GetRotationJacobian'),
        ItemFunctionDef('GetRotationJacobianTTimesVector_q'),
        ItemFunctionDef('CollectCurrentNodeData1'),
        ItemFunctionDef('CollectCurrentNodeMarkerData'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "RigidBodyEP";',
            description=r"Get type name of node (without keyword 'Node'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('GetReferenceCoordinateVector',
            implementation='return parameters.referenceCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector',
            implementation='return parameters.initialCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector_t',
            implementation='return parameters.initialCoordinates_t;'),
        ItemFunctionDef('GetOutputVariable'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('ComputeAlgebraicEquations'),
        ItemFunctionDef('ComputeJacobianAE'),
        ItemFunctionDef('GetRotationParameters'),
        ItemFunctionDef('GetRotationParameters_t'),
        ItemFunctionDef('GetG'),
        ItemFunctionDef('GetGlocal'),
        ItemFunctionDef('GetG_t'),
        ItemFunctionDef('GetGlocal_t'),
        ItemFunctionDef('GetGTv_q'),
        ItemFunctionDef('GetGlocalTv_q'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   NodeRigidBodyRxyz   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='NodeRigidBodyRxyz',
    addProtectedC=r"""    static constexpr Index nRotationCoordinates = 3;
    static constexpr Index nDisplacementCoordinates = 3;
""",
    cParentClass=ParentClassCNodeRigidBody,
    overallDescription=r"""A 3D rigid body node based on Euler / Tait-Bryan angles for rigid bodies or beams. All coordinates lead to second order differential equations; NOTE: this node has a singularity if the second rotation parameter reaches $\psi_1 = (2k-1) \pi/2$, with $k \in \Ncal$ or $-k \in \Ncal$.""",
    classType=ClassTypeNode,
    examples=['TestModels/connectorRigidBodySpringDamperTest.py', 'TestModels/heavyTop.py'],
    miniExample=r"""    #a rigid body spinning about its z-axis; the rotation coordinates are Tait-Bryan angles
    node = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0.5,0.2,0.1, 0,0,0],
                                         initialVelocities=[0,0,0, 0,0,0.5*np.pi])) #angle rates
    inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
    mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(),
                                  physicsInertia=inertia.GetInertia6D()))

    mbs.Assemble()
    mbs.SolveDynamic()

    #the third rotation coordinate after 1 second:
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)[5] #pi/2
    """,
    miniExamplePerformanceTest={'numberOfSteps': 609950},
    detailedDescription=r"""    #### Coordinates

    | index | symbol | kind | meaning | frame |
    |---|---|---|---|---|
    | 0, 1, 2 | $q_0,\,q_1,\,q_2$ | ABRV:ODE2 | displacement of the reference point of the body | global |
    | 3, 4, 5 | $\psi_0,\,\psi_1,\,\psi_2$ | ABRV:ODE2 | change of the Tait-Bryan angles against their reference values: consecutive rotations about the $x$-, $y$- and $z$-axis | - |

    #### Configuration

    The position of the reference point and the angles $\ttheta$ are the sums of reference and current
    coordinates,

    $$
    \pv\cConfig = \pv\cRef + \uv\cConfig, \quad \ttheta\cConfig = \tpsi\cRef + \tpsi\cConfig ,
    $$

    and the rotation matrix, which transforms a local (body-fixed) position $\pLocB$ into the global
    frame, $\LU{0}{\pLoc}\cConfig = \LU{0b}{\Rot}\cConfig \LU{b}{\pLoc}$, is

    $$
    \LU{0b}{\Rot} = \LU{01}{\Rot_0}(\theta_0) \LU{12}{\Rot_1}(\theta_1) \LU{2b}{\Rot_2}(\theta_2) ,
    $$

    see [](#sec-symbolsitems) for the elementary rotation matrices $\Rot_0$, $\Rot_1$ and $\Rot_2$.

    #### Frame and interpretation

    The displacement is given in the global frame, and the angles describe the rotation of the body
    frame $b$ against the global frame.

    #### Action on the equations of motion

    The velocity transformation relates the time derivatives of the angles to the angular velocity,

    $$
    \begin{aligned}
    \LU{0}{\tomega} &= \LU{0}{\Gm} \dot \ttheta, \\
    \LU{b}{\tomega} &= \LU{b}{\Gm} \dot \ttheta.
    \end{aligned}
    $$

    All six coordinates lead to ABRV:ODE2 equations: the first three are the residuals of the forces in
    the global frame, the last three the torque equations projected with $\LU{b}{\Gm\tp}$ (body frame)
    or $\LU{0}{\Gm\tp}$ (global frame), see the equations of motion of [](#sec-item-objectrigidbody).
    There is no constraint.

    #### Singularity

    $\Gm$ is singular for $\theta_1 = \pm \pi/2$ (and every multiple of $\pi$ added): there the rotations
    about the first and the third axis coincide, and the equations cannot be solved. Use the node only
    for motions that stay away from it, or `NodeRigidBodyEP` or `NodeRigidBodyRotVecLG`.

    For creating a `NodeRigidBodyRxyz` together with a rigid body, use `CreateRigidBody`, see
    [](#sec-mainsystemextensions-createrigidbody).
    """,
    mainParentClass=MainParentClassMainNode,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}\cConfig = \LU{0}{[p_0,\,p_1,\,p_2]}\cConfig\tp= \LU{0}{\uv}\cConfig + \LU{0}{\pv}\cRef$global 3D position vector of node; $\uv\cRef=0$"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv}\cConfig = [q_0,\,q_1,\,q_2]\cConfig\tp$global 3D displacement vector of node"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv}\cConfig = [\dot q_0,\,\dot q_1,\,\dot q_2]\cConfig\tp$global 3D velocity vector of node"""),
        ItemOutputVariable(OVAcceleration, OVDAccelerationNode),
        ItemOutputVariable(OVCoordinatesTotal, OVDCoordinatesTotalNodeRotation),
        ItemOutputVariable(OVCoordinates, r"""$\cv\cConfig = [q_0,\,q_1,\,q_2, \,\psi_0,\,\psi_1,\,\psi_2]\tp\cConfig$ coordinate vector of node, having 3 displacement coordinates and 3 Euler angles"""),
        ItemOutputVariable(OVCoordinates_t, r"""$\dot\cv\cConfig = [\dot q_0,\,\dot q_1,\,\dot q_2, \,\dot \psi_0,\,\dot \psi_1,\,\dot \psi_2]\tp\cConfig$ velocity coordinates vector of node"""),
        ItemOutputVariable(OVCoordinates_tt, r"""$\ddot\cv\cConfig = [\ddot q_0,\,\ddot q_1,\,\ddot q_2, \,\ddot \psi_0,\,\ddot \psi_1,\,\ddot \psi_2]\tp\cConfig$ acceleration coordinates vector of node"""),
        ItemOutputVariable(OVRotationMatrix, OVDRotationMatrixRowMajor),
        ItemOutputVariable(OVRotation, r"""$[\varphi_0,\,\varphi_1,\,\varphi_2]\tp\cConfig = [\psi_0,\,\psi_1,\,\psi_2]\tp\cRef + [\psi_0,\,\psi_1,\,\psi_2]\tp\cConfig$vector with 3 components of the Euler / Tait-Bryan angles in xyz-sequence ($\LU{0b}{\Rot}\cConfig=:\Rot_0(\varphi_0) \cdot \Rot_1(\varphi_1) \cdot \Rot_2(\varphi_2)$)"""),
        ItemOutputVariable(OVAngularVelocity, OVDAngularVelocityNode),
        ItemOutputVariable(OVAngularVelocityLocal, OVDAngularVelocityLocalNode),
        ItemOutputVariable(OVAngularAcceleration, r"""$\LU{0}{\talpha}\cConfig = \LU{0}{[\alpha_0,\,\alpha_1,\,\alpha_2]}\cConfig\tp$global 3D angular acceleration vector of node"""),
        ],
    pythonShortName='RigidRxyz',
    visuParentClass=VisuParentClassVisualizationNode,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"node's unique name"),
        ItemParameter(type=TVectorND(6), destination=DestComp+DestParam,
            pythonName='referenceCoordinates',
            defaultValue='Vector6D({0.,0.,0., 0.,0.,0.})',
            description=r"""$\qv\cRef = [q_0,\,q_1,\,q_2,\,\psi_0,\,\psi_1,\,\psi_2]\tp\cRef = [\pv\tp\cRef,\,\tpsi\tp\cRef]\tp$reference coordinates (3 position and 3 xyz Euler angles) of node ==> e.g. ref. coordinates for finite elements or reference position of rigid body (e.g. for definition of joints)"""),
        ItemParameter(type=TVectorND(6), destination=DestMain+DestParam,
            pythonName='initialCoordinates',
            defaultValue='Vector6D({0.,0.,0., 0.,0.,0.})',
            description=r"""$\qv\cIni = [q_0,\,q_1,\,q_2,\,\psi_0,\,\psi_1,\,\psi_2]\tp\cIni = [\uv\tp\cIni,\,\tpsi\tp\cIni]\tp$initial displacement coordinates: ux,uy,uz and 3 Euler angles (xyz) relative to reference coordinates"""),
        ItemParameter(type=TVectorND(6), destination=DestMain+DestParam,
            pythonName='initialVelocities',
            cplusplusName='initialCoordinates_t',
            defaultValue='Vector6D({0.,0.,0., 0.,0.,0.})',
            description=r"""$\dot \qv\cIni = [\dot q_0,\,\dot q_1,\,\dot q_2,\,\dot \psi_0,\,\dot \psi_1,\,\dot \psi_2]\tp\cIni = [\dot \uv\tp\cIni,\,\dot \tpsi\tp\cIni]\tp$initial velocity coordinate: time derivatives of ux,uy,uz and of 3 Euler angles (xyz)"""),
        ItemFunctionDef('GetNumberOfODE2Coordinates',
            implementation='return 6;'),
        ItemFunctionDef('GetNumberOfDisplacementCoordinates',
            implementation='return nDisplacementCoordinates;'),
        ItemFunctionDef('GetNumberOfRotationCoordinates',
            implementation='return nRotationCoordinates;'),
        ItemTypes('Node', ['Position', 'Orientation', 'RigidBody', 'RotationRxyz'],
            description=r'return node type (for node treatment in computation)'),
        ItemFunctionDef('GetNodeGroup',
            implementation='return CNodeGroup::ODE2variables;'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetAcceleration'),
        ItemFunctionDef('GetRotationMatrix'),
        ItemFunctionDef('GetAngularVelocity',
            description='return configuration dependent angular velocity of node; returns always a 3D Vector'),
        ItemFunctionDef('GetAngularVelocityLocal'),
        ItemFunctionDef('GetAngularAcceleration'),
        ItemFunctionDef('GetPositionJacobian'),
        ItemFunctionDef('GetRotationJacobian'),
        ItemFunctionDef('GetRotationJacobianTTimesVector_q'),
        ItemFunctionDef('CollectCurrentNodeData1'),
        ItemFunctionDef('CollectCurrentNodeMarkerData'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "RigidBodyRxyz";',
            description=r"Get type name of node (without keyword 'Node'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('GetReferenceCoordinateVector',
            implementation='return parameters.referenceCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector',
            implementation='return parameters.initialCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector_t',
            implementation='return parameters.initialCoordinates_t;'),
        ItemFunctionDef('GetOutputVariable'),
        ItemFunctionDef('GetRotationParameters'),
        ItemFunctionDef('GetRotationParameters_t'),
        ItemFunctionDef('GetG'),
        ItemFunctionDef('GetGlocal'),
        ItemFunctionDef('GetG_t'),
        ItemFunctionDef('GetGlocal_t'),
        ItemFunctionDef('GetGTv_q'),
        ItemFunctionDef('GetGlocalTv_q'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   NodeRigidBodyRotVecLG   +++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='NodeRigidBodyRotVecLG',
    addProtectedC=r"""    static constexpr Index nRotationCoordinates = 3;
    static constexpr Index nDisplacementCoordinates = 3;
""",
    author=r'Gerstmayr Johannes, Holzinger Stefan',
    cParentClass=ParentClassCNodeRigidBody,
    overallDescription=r'A 3D rigid body node based on rotation vector and Lie group methods for rigid bodies. The node has 3 displacement coordinates and three rotation coordinates and can be used in combination with explicit Lie Group time integration methods.',
    classType=ClassTypeNode,
    miniExample=r"""    #a rigid body spinning about its z-axis, integrated with the Lie group integrator of the explicit solver
    node = mbs.AddNode(NodeRigidBodyRotVecLG(referenceCoordinates=[0.5,0.2,0.1, 0,0,0],
                                             initialVelocities=[0,0,0, 0,0,0.5*np.pi])) #angular velocity
    inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.2,0.1])
    mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=inertia.Mass(),
                                  physicsInertia=inertia.GetInertia6D()))

    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 100
    mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.RK44)

    #the rotation vector after 1 second:
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Rotation)[2] #pi/2
    """,
    miniExamplePerformanceTest={'numberOfSteps': 644750},
    detailedDescription=r"""    #### Coordinates

    | index | symbol | kind | meaning | frame |
    |---|---|---|---|---|
    | 0, 1, 2 | $q_0,\,q_1,\,q_2$ | ABRV:ODE2 | displacement of the reference point of the body | global |
    | 3, 4, 5 | $\nu_0,\,\nu_1,\,\nu_2$ | ABRV:ODE2 | change of the rotation vector against its reference value | - |

    #### Configuration

    The rotation vector combines the rotation angle $\varphi$ and the axis $\nv$,

    $$
    \tnu = \varphi \nv = \tnu\cConfig + \tnu\cRef ,
    $$

    and the rotation matrix $\LU{0b}{\Rot(\tnu)}$ transforms a local position into the global frame,
    $\LU{0}{\pLoc}\cConfig = \LU{0b}{\Rot(\tnu)}\cConfig \LU{b}{\pLoc}$; $\Rot(\tnu)$ is the function
    `RotationVector2RotationMatrix`, see [](#sec-rigidbodyutilities-rotationvector2rotationmatrix).

    #### Frame and interpretation

    The displacement is given in the global frame. The rotation coordinates are not a conventional
    parametrization: the node is meant for Lie group time integration, and it switches its rotation
    coordinates to it by itself, in explicit and in implicit integrators. For the formulation see
    Holzinger and Gerstmayr [CITE:HolzingerGerstmayr2020].

    #### Action on the equations of motion

    All six coordinates lead to ABRV:ODE2 equations: the first three are the residuals of the forces in
    the global frame, the last three the residuals of the torques in the body frame. In the Lie group
    update the rotation velocity coordinates are the local angular velocity $\LU{b}{\tomega}$, so that
    $\LU{b}{\Gm}$ is the identity matrix. There is no constraint.

    #### Singularity

    None in the Lie group update, which is why the node suits arbitrary rotations; it also reduces the
    nonlinearity of the equations, which can help implicit integration.

    For creating a `NodeRigidBodyRotVecLG` together with a rigid body, use `CreateRigidBody`, see
    [](#sec-mainsystemextensions-createrigidbody).
    """,
    mainParentClass=MainParentClassMainNode,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}\cConfig = \LU{0}{[p_0,\,p_1,\,p_2]}\cConfig\tp= \LU{0}{\uv}\cConfig + \LU{0}{\pv}\cRef$global 3D position vector of node; $\uv\cRef=0$"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv}\cConfig = [q_0,\,q_1,\,q_2]\cConfig\tp$global 3D displacement vector of node"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv}\cConfig = [\dot q_0,\,\dot q_1,\,\dot q_2]\cConfig\tp$global 3D velocity vector of node"""),
        ItemOutputVariable(OVAcceleration, OVDAccelerationNode),
        ItemOutputVariable(OVCoordinatesTotal, OVDCoordinatesTotalNodeRotation),
        ItemOutputVariable(OVCoordinates, r"""$\cv\cConfig = [q_0,\,q_1,\,q_2, \,\nu_0,\,\nu_1,\,\nu_2]\tp\cConfig$ coordinate vector of node, having 3 displacement coordinates and 3 Euler angles"""),
        ItemOutputVariable(OVCoordinates_t, r"""$\dot\cv\cConfig = [\dot q_0,\,\dot q_1,\,\dot q_2, \,\dot \nu_0,\,\dot \nu_1,\,\dot \nu_2]\tp\cConfig$ velocity coordinates vector of node"""),
        ItemOutputVariable(OVRotationMatrix, OVDRotationMatrixRowMajor),
        ItemOutputVariable(OVRotation, r"""$[\varphi_0,\,\varphi_1,\,\varphi_2]\tp\cConfig$vector with 3 components of the Euler/Tait-Bryan angles in xyz-sequence ($\LU{0b}{\Rot}\cConfig=:\Rot_0(\varphi_0) \cdot \Rot_1(\varphi_1) \cdot \Rot_2(\varphi_2)$), recomputed from rotation matrix"""),
        ItemOutputVariable(OVAngularVelocity, OVDAngularVelocityNode),
        ItemOutputVariable(OVAngularVelocityLocal, OVDAngularVelocityLocalNode),
        ],
    pythonShortName='RigidRotVecLG',
    visuParentClass=VisuParentClassVisualizationNode,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"node's unique name"),
        ItemParameter(type=TVectorND(6), destination=DestComp+DestParam,
            pythonName='referenceCoordinates',
            defaultValue='Vector6D({0.,0.,0., 0.,0.,0.})',
            description=r"""$\qv\cRef = [q_0,\,q_1,\,q_2,\,\nu_0,\,\nu_1,\,\nu_2]\tp\cRef = [\pv\tp\cRef,\,\tnu\tp\cRef]\tp$reference coordinates (position and rotation vector $\tnu$) of node ==> e.g. ref. coordinates for finite elements or reference position of rigid body (e.g. for definition of joints)"""),
        ItemParameter(type=TVectorND(6), destination=DestMain+DestParam,
            pythonName='initialCoordinates',
            defaultValue='Vector6D({0.,0.,0., 0.,0.,0.})',
            description=r"""$\qv\cIni = [q_0,\,q_1,\,q_2,\,\nu_0,\,\nu_1,\,\nu_2]\tp\cIni = [\uv\tp\cIni,\,\tnu\tp\cIni]\tp$initial displacement coordinates $\uv$ and rotation vector $\tnu$ relative to reference coordinates"""),
        ItemParameter(type=TVectorND(6), destination=DestMain+DestParam,
            pythonName='initialVelocities',
            cplusplusName='initialCoordinates_t',
            defaultValue='Vector6D({0.,0.,0., 0.,0.,0.})',
            description=r"""$\dot \qv\cIni = [\dot q_0,\,\dot q_1,\,\dot q_2,\,\dot \nu_0,\,\dot \nu_1,\,\dot \nu_2]\tp\cIni = [\dot \uv\tp\cIni,\,\dot \tnu\tp\cIni]\tp$initial velocity coordinate: time derivatives of displacement and angular velocity vector"""),
        ItemFunctionDef('GetNumberOfODE2Coordinates',
            implementation='return 6;'),
        ItemFunctionDef('GetNumberOfDisplacementCoordinates',
            implementation='return nDisplacementCoordinates;'),
        ItemFunctionDef('GetNumberOfRotationCoordinates',
            implementation='return nRotationCoordinates;'),
        ItemTypes('Node', ['Position', 'Orientation', 'RigidBody', 'RotationRotationVector', 'LieGroupWithDirectUpdate'],
            description=r'return node type (for node treatment in computation)'),
        ItemFunctionDef('GetNodeGroup',
            implementation='return CNodeGroup::ODE2variables;'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetAcceleration'),
        ItemFunctionDef('GetRotationMatrix'),
        ItemFunctionDef('GetAngularVelocity',
            description='return configuration dependent angular velocity of node; returns always a 3D Vector'),
        ItemFunctionDef('GetAngularVelocityLocal'),
        ItemFunctionDef('GetAngularAcceleration'),
        ItemFunctionDef('GetPositionJacobian'),
        ItemFunctionDef('GetRotationJacobian'),
        ItemFunctionDef('GetRotationJacobianTTimesVector_q'),
        ItemFunction(type=TMatrixND(3, 3), destination=DestComp, isVirtual=False, isStatic=True,
            pythonName='RotationVectorGTv_q',
            args='const CSVector4D& rotParameters, const Vector3D& v3D',
            description=r'static function to compute d(G^T*v)/dq for rotation vector (Glocal = I, G = RotationMatrix); using autodiff'),
        ItemFunctionDef('CollectCurrentNodeData1'),
        ItemFunctionDef('CollectCurrentNodeMarkerData'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "RigidBodyRotVecLG";',
            description=r"Get type name of node (without keyword 'Node'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('GetReferenceCoordinateVector',
            implementation='return parameters.referenceCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector',
            implementation='return parameters.initialCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector_t',
            implementation='return parameters.initialCoordinates_t;'),
        ItemFunctionDef('GetOutputVariable'),
        ItemFunctionDef('GetRotationParameters'),
        ItemFunctionDef('GetRotationParameters_t'),
        ItemFunctionDef('GetG'),
        ItemFunctionDef('GetGlocal'),
        ItemFunctionDef('GetG_t'),
        ItemFunctionDef('GetGlocal_t'),
        ItemFunctionDef('GetGTv_q'),
        ItemFunctionDef('GetGlocalTv_q'),
        ItemFunctionDef('CompositionRule'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   NodeRigidBody2D   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='NodeRigidBody2D',
    cParentClass=ParentClassCNodeODE2,
    overallDescription=r"""A 2D rigid body node for rigid bodies or beams. The node has 2 displacement degrees of freedom and one rotation coordinate (rotation around z-axis: $\psi_0$). All coordinates are ABRV:ODE2, used for second order differetial equations.""",
    classType=ClassTypeNode,
    examples=['Examples/rigidPendulum.py', 'Examples/doublePendulum2D.py', 'Examples/SliderCrank.py', 'Examples/simple4linkPendulumBing.py', 'Examples/slidercrankWithMassSpring.py'],
    miniExample=r"""    #a planar rigid body: x, y and the rotation angle, thrown with a spin
    node = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0.5,0.2,0], initialVelocities=[1,0,2]))
    mbs.AddObject(ObjectRigidBody2D(nodeNumber=node, physicsMass=2, physicsInertia=0.1))

    mbs.Assemble()
    mbs.SolveDynamic()

    #x = 1*t, angle = 2*t at t=1
    exu.sys['testResult'] = sum(mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)) #3
    """,
    miniExamplePerformanceTest={'numberOfSteps': 1747720},
    detailedDescription=r"""    #### Coordinates

    | index | symbol | kind | meaning | frame |
    |---|---|---|---|---|
    | 0, 1 | $q_0,\,q_1$ | ABRV:ODE2 | displacement of the reference point of the body in $x$ and $y$ | global |
    | 2 | $\psi_0$ | ABRV:ODE2 | change of the rotation angle about the $z$-axis | - |

    #### Configuration

    With the rotation angle $\theta_{0} = \psi_{0}\cRef + \psi_{0}\cConfig$, the rotation matrix is

    $$
    \LU{0b}{\Rot}\cConfig = \mr{\cos(\theta_0)}{-\sin(\theta_0)}{0}{\sin(\theta_0)}{\cos(\theta_0)}{0}{0}{0}{1}\cConfig
    $$

    #### Frame and interpretation

    The displacement is given in the global $x$-$y$ plane and the angle about the global $z$-axis;
    body frame and global frame share the $z$-axis.

    #### Action on the equations of motion

    The three coordinates lead to three ABRV:ODE2 equations: the residuals of the forces in $x$ and $y$,
    and the residual of the torque about the $z$-axis, which is the same in the body and in the global
    frame. The velocity transformation is the identity, $\omega_z = \dot\theta_0$.

    **Example**: see [](#sec-item-objectrigidbody2d)
    """,
    mainParentClass=MainParentClassMainNode,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}\cConfig = \LU{0}{[p_0,\,p_1,\,0]}\cConfig\tp= \LU{0}{\uv}\cConfig + \LU{0}{\pv}\cRef$global 3D position vector of node; $\uv\cRef=0$"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv}\cConfig = [q_0,\,q_1,\,0]\cConfig\tp$global 3D displacement vector of node"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv}\cConfig = [\dot q_0,\,\dot q_1,\,0]\cConfig\tp$global 3D velocity vector of node"""),
        ItemOutputVariable(OVAcceleration, r"""$\LU{0}{\av}\cConfig = [\ddot q_0,\,\ddot q_1,\,0]\cConfig\tp$global 3D acceleration vector of node"""),
        ItemOutputVariable(OVAngularVelocity, r"""$\LU{0}{\tomega}\cConfig = \LU{0}{[0,\,0,\,\dot \psi_0]}\cConfig\tp$global 3D angular velocity vector of node"""),
        ItemOutputVariable(OVCoordinatesTotal, OVDCoordinatesTotalNodeRotation),
        ItemOutputVariable(OVCoordinates, r"""$\cv\cConfig = [q_0,\,q_1,\,\psi_0]\tp\cConfig$ coordinate vector of node, having 2 displacement coordinates and 1 angle"""),
        ItemOutputVariable(OVCoordinates_t, r"""$\dot\cv\cConfig = [\dot q_0,\,\dot q_1,\,\dot \psi_0]\tp\cConfig$ velocity coordinates vector of node"""),
        ItemOutputVariable(OVCoordinates_tt, r"""$\ddot\cv\cConfig = [\ddot q_0,\,\ddot q_1,\,\ddot \psi_0]\tp\cConfig$ acceleration coordinates vector of node"""),
        ItemOutputVariable(OVRotationMatrix, OVDRotationMatrixRowMajor),
        ItemOutputVariable(OVRotation, r"""$[0,\,0,\,\theta_0]\tp\cConfig = [0,\,0,\,\psi_0]\tp\cRef + [0,\,0,\,\psi_0]\tp\cConfig$vector with 3rd angle around out of plane axis"""),
        ItemOutputVariable(OVAngularVelocityLocal, r"""$\LU{b}{\tomega}\cConfig = \LU{b}{[0,\,0,\,\dot \psi_0]}\cConfig\tp$local (body-fixed)  3D angular velocity vector of node"""),
        ItemOutputVariable(OVAngularAcceleration, r"""$\LU{0}{\talpha}\cConfig = \LU{0}{[0,\,0,\,\ddot \psi_0]}\cConfig\tp$global 3D angular acceleration vector of node"""),
        ],
    pythonShortName='Rigid2D',
    visuParentClass=VisuParentClassVisualizationNode,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"node's unique name"),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='referenceCoordinates',
            defaultValue=DVZeroVector3D,
            description=r"""$\qv\cRef = [q_0,\,q_1,\,\psi_0]\tp\cRef$reference coordinates (x-pos,y-pos and rotation) of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement"""),
        ItemParameter(type=TVectorND(3), destination=DestMain+DestParam,
            pythonName='initialCoordinates',
            defaultValue=DVZeroVector3D,
            description=r"""$\qv\cIni = [q_0,\,q_1,\,\psi_0]\tp\cIni$initial displacement coordinates and angle (relative to reference coordinates)"""),
        ItemParameter(type=TVectorND(3), destination=DestMain+DestParam,
            pythonName='initialVelocities',
            cplusplusName='initialCoordinates_t',
            defaultValue=DVZeroVector3D,
            description=r"""$\dot \qv\cIni = [\dot q_0,\,\dot q_1,\,\dot \psi_0]\tp\cIni =  [v_0,\,v_1,\,\omega_2]\tp\cIni$initial velocity coordinates"""),
        ItemFunctionDef('GetNumberOfODE2Coordinates',
            implementation='return 3;'),
        ItemTypes('Node', ['Position2D', 'Orientation2D', 'RigidBody'],
            description=r'return node type (for node treatment in computation)'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetAcceleration'),
        ItemFunctionDef('GetRotationMatrix'),
        ItemFunctionDef('GetAngularVelocity',
            description='return configuration dependent angular velocity of node; returns always a 3D Vector'),
        ItemFunctionDef('GetAngularVelocityLocal',
            implementation='return GetAngularVelocity(configuration);'),
        ItemFunctionDef('GetAngularAcceleration'),
        ItemFunctionDef('GetPositionJacobian',
            implementation='value.SetMatrix(3,3,{1.f,0.f,0.f, 0.f,1.f,0.f, 0.f,0.f,0.f});'),
        ItemFunctionDef('GetRotationJacobian',
            implementation='value.SetMatrix(3,3,{0.f,0.f,0.f, 0.f,0.f,0.f, 0.f,0.f,1.f});'),
        ItemFunctionDef('GetRotationJacobianTTimesVector_q',
            implementation='jacobian_q.SetNumberOfRowsAndColumns(0, 0);'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "RigidBody2D";',
            description=r"Get type name of node (without keyword 'Node'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('GetReferenceCoordinateVector',
            implementation='return parameters.referenceCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector',
            implementation='return parameters.initialCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector_t',
            implementation='return parameters.initialCoordinates_t;'),
        ItemFunctionDef('GetOutputVariable'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   Node1D   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='Node1D',
    cParentClass=ParentClassCNodeODE2,
    overallDescription=r"""A node with one ABRV:ODE2 coordinate for one dimensional (1D) problems. Use e.g. for scalar dynamic equations (Mass1D) and mass-spring-damper mechanisms, representing either translational or rotational degrees of freedom: in most cases, Node1D is equivalent to NodeGenericODE2 using one coordinate, however, it offers a transformation to 3D translational or rotational motion and allows to couple this node to 2D or 3D bodies.""",
    classType=ClassTypeNode,
    miniExample=r"""    #one coordinate, here the displacement of a 1D mass, pulled by a constant force
    node = mbs.AddNode(Node1D(referenceCoordinates=[0], initialVelocities=[1]))
    mbs.AddObject(ObjectMass1D(nodeNumber=node, physicsMass=2))
    mCoord = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=0))
    mbs.AddLoad(LoadCoordinate(markerNumber=mCoord, load=4))

    mbs.Assemble()
    mbs.SolveDynamic()

    #q = v0*t + F/(2m)*t^2 at t=1
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates) #2 (a scalar for one coordinate)
    """,
    miniExamplePerformanceTest={'numberOfSteps': 1715930},
    detailedDescription=r"""    #### Coordinates

    | index | symbol | kind | meaning | frame |
    |---|---|---|---|---|
    | 0 | $q_0$ | ABRV:ODE2 | displacement or rotation, as the object reads it | the frame of the object |

    #### Configuration

    The current position or rotation coordinate of the node is

    $$
    p_0 = {q_0}\cRef + {q_0}\cCur .
    $$

    For drawing and for markers, the node has a position and a velocity in 3D,
    $\pv\cConfig = [{p_0}\cConfig,\,0,\,0]\tp$ and $[{\dot p_0}\cConfig,\,0,\,0]\tp$.

    #### Frame and interpretation

    What the coordinate means is the object's: `ObjectMass1D` reads it as a translation along the local
    $x$-axis of the frame of its `referencePosition` and `referenceRotation`, `ObjectRotationalMass1D` as a rotation about its local axis. That
    is what couples a 1D node to 2D or 3D bodies, and what distinguishes it from a `NodeGenericODE2` with
    one coordinate.

    #### Action on the equations of motion

    The coordinate leads to one ABRV:ODE2 equation, the residual of the force or the torque the object
    assigns to it.
    """,
    mainParentClass=MainParentClassMainNode,
    outputVariables=[
        ItemOutputVariable(OVCoordinatesTotal, OVDCoordinatesTotalNode),
        ItemOutputVariable(OVCoordinates, r"""$\qv\cConfig = [q_0]\tp\cConfig$ABRV:ODE2 coordinate of node (in vector form)"""),
        ItemOutputVariable(OVCoordinates_t, r"""$\dot \qv\cConfig = [\dot q_0]\tp\cConfig$ABRV:ODE2 velocity coordinate of node (in vector form)"""),
        ItemOutputVariable(OVCoordinates_tt, r"""$\ddot \qv\cConfig = [\ddot q_0]\tp\cConfig$ABRV:ODE2 acceleration coordinate of node (in vector form)"""),
        ],
    visuParentClass=VisuParentClassVisualizationNode,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"node's unique name"),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='referenceCoordinates',
            defaultValue='Vector({0.})',
            description=r'$[q_0]\tp\cRef$reference coordinate of node (in vector form)'),
        ItemParameter(type=TVector, destination=DestMain+DestParam,
            pythonName='initialCoordinates',
            defaultValue='Vector({0.})',
            description=r"""$[q_0]\tp\cIni$initial displacement coordinate (in vector form)"""),
        ItemParameter(type=TVector, destination=DestMain+DestParam,
            pythonName='initialVelocities',
            cplusplusName='initialCoordinates_t',
            defaultValue='Vector({0.})',
            description=r"""$[\dot q_0]\tp\cIni$initial velocity coordinate (in vector form)"""),
        ItemFunctionDef('GetNumberOfODE2Coordinates',
            implementation='return 1;'),
        ItemTypes('Node', ['GenericODE2'],
            description=r'return node type (for node treatment in computation)'),
        ItemFunctionDef('GetPosition',
            description='return configuration dependent position of node; returns always a 3D Vector; gives the local (x) position for Node1D'),
        ItemFunctionDef('GetVelocity',
            description='return configuration dependent velocity of node; returns always a 3D Vector; gives the local (x) velocity for Node1D'),
        ItemFunctionDef('GetAcceleration'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "1D";',
            description=r"Get type name of node (without keyword 'Node'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('GetReferenceCoordinateVector',
            implementation='return parameters.referenceCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector',
            implementation='return parameters.initialCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector_t',
            implementation='return parameters.initialCoordinates_t;'),
        ItemFunctionDef('GetOutputVariable'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=False,
            description=r'set true, if item is shown in visualization and false if it is not shown; The node1D is represented as reference position and displacement along the global x-axis, which must not agree with the representation in the object using the Node1D'),
        ItemFunctionDef('UpdateGraphics',
            implementation=';',
            description='Empty graphics update for now'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   NodePoint2DSlope1   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='NodePoint2DSlope1',
    cParentClass=ParentClassCNodeODE2,
    overallDescription=r"""A 2D point/slope vector node for planar Bernoulli-Euler ANCF (absolute nodal coordinate formulation) beam elements. The node has 4 displacement degrees of freedom (2 for displacement of point node and 2 for the slope vector 'slopex'); all coordinates lead to second order differential equations; the slope vector defines the directional derivative w.r.t the local axial (x) coordinate, denoted as $()^\prime$; in straight configuration aligned at the global x-axis, the slope vector reads $\rv^\prime=[r_x^\prime\;\;r_y^\prime]^T=[1\;\;0]^T$.""",
    classType=ClassTypeNode,
    miniExample=r"""    #a cantilever of one ANCF cable element: position and slope (r_x) at each node
    L = 1; EI = 100; F = -0.1
    n0 = mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[0,0, 1,0])) #position, slope = axis
    n1 = mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[L,0, 1,0]))
    mbs.AddObject(ObjectANCFCable2D(nodeNumbers=[n0,n1], physicsLength=L, physicsMassPerLength=1,
                                    physicsBendingStiffness=EI, physicsAxialStiffness=1e5))
    mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
    for i in [0,1,3]: #clamped: x, y and the y-component of the slope
        mCoord = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=i))
        mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mGround, mCoord]))
    mTip = mbs.AddMarker(MarkerNodePosition(nodeNumber=n1))
    mbs.AddLoad(LoadForceVector(markerNumber=mTip, loadVector=[0,F,0]))

    mbs.Assemble()
    mbs.SolveStatic()

    #the cubic element is exact for a tip load: F*L^3/(3*EI) = -1/3000
    exu.sys['testResult'] = mbs.GetNodeOutput(n1, exu.OutputVariableType.Displacement)[1]*1000 #-1/3
    """,
    miniExamplePerformanceTest={'numberOfSteps': 330150},
    detailedDescription=r"""    #### Coordinates

    | index | symbol | kind | meaning | frame |
    |---|---|---|---|---|
    | 0, 1 | $q_0,\,q_1$ | ABRV:ODE2 | displacement of the node position $\rv$ | global |
    | 2, 3 | $q_2,\,q_3$ | ABRV:ODE2 | change of the slope vector $\rv^\prime$ | global |

    #### Configuration

    Position and slope vector are the sums of reference and current values,

    $$
    \rv = \rv\cRef + [q_0,\,q_1]\tp, \quad \rv^\prime = \rv^\prime\cRef + [q_2,\,q_3]\tp .
    $$

    #### The slope vector

    The slope vector is the derivative of the position of the beam axis with respect to the axial
    coordinate $x$ of the element in its reference configuration, $\rv^\prime = \partial \rv / \partial x$.
    In a straight beam along the global $x$-axis it is $[1,\;0]\tp$, which is the default of the
    reference coordinates. Its **direction** is the tangent of the beam axis, and so the rotation of the
    cross section in a Bernoulli-Euler beam; its **length** is one plus the axial strain,
    $\varepsilon = \|\rv^\prime\| - 1$, as `ObjectANCFCable2D` computes it. A reference slope of length
    other than one therefore describes a pre-strained beam.

    #### Frame and interpretation

    All four coordinates are global, with no rotation parameters: this is the absolute nodal coordinate
    formulation. The node is used by the planar ANCF cable elements, `ObjectANCFCable2D` and
    `ObjectALEANCFCable2D`, which share the node between neighbouring elements; the beam utilities
    (`exudyn.beams`) create them.

    #### Action on the equations of motion

    The four coordinates lead to four ABRV:ODE2 equations, which the element provides. A force at the
    node enters the first two; `MarkerNodeRigid` sees the node as a position with the orientation of the
    slope vector.
    """,
    mainParentClass=MainParentClassMainNode,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}\cConfig = [p_0,\, p_1,\,0]\cConfig\tp$global 3D position vector of node (=displacement+reference position)"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv}\cConfig = [q_0,\, q_1,\,0]\cConfig\tp$global 3D displacement vector of node"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv}\cConfig = [\dot q_0,\,\dot q_1,\,0]\cConfig\tp$global 3D velocity vector of node"""),
        ItemOutputVariable(OVAcceleration, r"""$\LU{0}{\av}\cConfig = [\ddot q_0,\,\ddot q_1,\,0]\cConfig\tp$global 3D acceleration vector of node"""),
        ItemOutputVariable(OVCoordinatesTotal, OVDCoordinatesTotalNode),
        ItemOutputVariable(OVCoordinates, 'coordinates vector of node (2 displacement coordinates + 2 slope vector coordinates)'),
        ItemOutputVariable(OVCoordinates_t, 'velocity coordinates vector of node (derivative of the 2 displacement coordinates + 2 slope vector coordinates)'),
        ItemOutputVariable(OVCoordinates_tt, 'acceleration coordinates vector of node (derivative of the 2 displacement coordinates + 2 slope vector coordinates)'),
        ],
    pythonShortName='Point2DS1',
    visuParentClass=VisuParentClassVisualizationNode,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"node's unique name"),
        ItemParameter(type=TVectorND(4), destination=DestComp+DestParam,
            pythonName='referenceCoordinates',
            defaultValue='Vector4D({0.,0.,1.,0.})',
            description=r'reference coordinates (x-pos,y-pos; x-slopex, y-slopex) of node; global position of node without displacement'),
        ItemParameter(type=TVectorND(4), destination=DestMain+DestParam,
            pythonName='initialCoordinates',
            defaultValue='Vector4D({0.,0.,0.,0.})',
            description=r"initial displacement coordinates: ux, uy and x/y 'displacements' of slopex"),
        ItemParameter(type=TVectorND(4), destination=DestMain+DestParam,
            pythonName='initialVelocities',
            cplusplusName='initialCoordinates_t',
            defaultValue='Vector4D({0.,0.,0.,0.})',
            description=r'initial velocity coordinates'),
        ItemFunctionDef('GetNumberOfODE2Coordinates',
            implementation='return 4;'),
        ItemTypes('Node', ['Position2D', 'Orientation2D', 'Point2DSlope1'],
            description=r'return node type (for node treatment in computation)'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetAcceleration'),
        ItemFunctionDef('GetRotationMatrix',
            description=r"""return configuration dependent rotation matrix of node; the slope vector $\rv^\prime = [1,0]$ is defines as zero angle ($\varphi = 0$), leading to a matrix $\Am = \mr{\cos\varphi}{-\sin\varphi}{0} {\sin\varphi}{\cos\varphi}{0} {0}{0}{1}$; the function always computes a 3D Matrix"""),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetAngularVelocityLocal',
            implementation='return GetAngularVelocity(configuration);'),
        ItemFunctionDef('GetPositionJacobian'),
        ItemFunctionDef('GetRotationJacobian'),
        ItemFunctionDef('GetRotationJacobianTTimesVector_q'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "Point2DSlope1";',
            description=r"Get type name of node (without keyword 'Node'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('GetReferenceCoordinateVector',
            implementation='return parameters.referenceCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector',
            implementation='return parameters.initialCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector_t',
            implementation='return parameters.initialCoordinates_t;'),
        ItemFunctionDef('GetOutputVariable'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   NodePointSlope1   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='NodePointSlope1',
    addPublicC=r"""    static constexpr Index nODE2coordinates = 6;//AUTO: number of coordinates, used for fixed-size templates
""",
    cParentClass=ParentClassCNodeODE2,
    overallDescription=r"""A 3D point/slope vector node for spatial Bernoulli-Euler ANCF (absolute nodal coordinate formulation) beam elements, with 3 position and 3 slope coordinates, all ABRV:ODE2; the slope vector is the derivative of the position with respect to the axial coordinate, $[1,\;0,\;0]\tp$ for a straight beam along the global $x$-axis.""",
    classType=ClassTypeNode,
    miniExample=r"""    #a cantilever of one 3D ANCF cable element: position and slope (r_x) at each node
    L = 1; EI = 100; F = -0.1
    n0 = mbs.AddNode(NodePointSlope1(referenceCoordinates=[0,0,0, 1,0,0])) #position, slope = axis
    n1 = mbs.AddNode(NodePointSlope1(referenceCoordinates=[L,0,0, 1,0,0]))
    mbs.AddObject(ObjectANCFCable(nodeNumbers=[n0,n1], physicsLength=L, physicsMassPerLength=1,
                                  physicsBendingStiffness=EI, physicsAxialStiffness=1e5))
    mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
    for i in [0,1,2,4,5]: #clamped: the position and the transverse components of the slope
        mCoord = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=i))
        mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mGround, mCoord]))
    mTip = mbs.AddMarker(MarkerNodePosition(nodeNumber=n1))
    mbs.AddLoad(LoadForceVector(markerNumber=mTip, loadVector=[0,0,F]))

    mbs.Assemble()
    mbs.SolveStatic()

    #the cubic element is exact for a tip load: F*L^3/(3*EI) = -1/3000
    exu.sys['testResult'] = mbs.GetNodeOutput(n1, exu.OutputVariableType.Displacement)[2]*1000 #-1/3
    """,
    miniExamplePerformanceTest={'numberOfSteps': 126950},
    detailedDescription=r"""    #### Coordinates

    | index | symbol | kind | meaning | frame |
    |---|---|---|---|---|
    | 0, 1, 2 | $q_0,\,q_1,\,q_2$ | ABRV:ODE2 | displacement of the node position $\rv$ | global |
    | 3, 4, 5 | $q_3,\,q_4,\,q_5$ | ABRV:ODE2 | change of the slope vector $\rv^\prime$ | global |

    #### Configuration

    $$
    \rv = \rv\cRef + [q_0,\,q_1,\,q_2]\tp, \quad \rv^\prime = \rv^\prime\cRef + [q_3,\,q_4,\,q_5]\tp .
    $$

    #### The slope vector

    The slope vector is the derivative of the position of the beam axis with respect to the axial
    coordinate $x$ of the element in its reference configuration, $\rv^\prime = \partial \rv / \partial x$.
    In a straight beam along the global $x$-axis it is $[1,\;0,\;0]\tp$, the default of the reference
    coordinates. Its direction is the tangent of the beam axis and its length one plus the axial
    strain, $\varepsilon = \|\rv^\prime\| - 1$. A single slope vector carries no rotation about the beam
    axis: the element that uses the node has no torsion.

    #### Frame and interpretation

    All six coordinates are global (absolute nodal coordinates). The node is used by the spatial ANCF
    cable element `ObjectANCFCable`.

    #### Action on the equations of motion

    The six coordinates lead to six ABRV:ODE2 equations, which the element provides; a force at the node
    enters the first three.
    """,
    mainParentClass=MainParentClassMainNode,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}\cConfig = [p_0,\, p_1,\, p_2]\cConfig\tp$global 3D position vector of node (=displacement+reference position)"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv}\cConfig = [q_0,\, q_1,\, q_2]\cConfig\tp$global 3D displacement vector of node"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\av}\cConfig = [\dot q_0,\,\dot q_1,\,\dot q_2]\cConfig\tp$global 3D velocity vector of node"""),
        ItemOutputVariable(OVAcceleration, OVDAccelerationNode),
        ItemOutputVariable(OVCoordinatesTotal, OVDCoordinatesTotalNode),
        ItemOutputVariable(OVCoordinates, 'coordinates vector of node (3 displacement coordinates + 3 slope vector coordinates)'),
        ItemOutputVariable(OVCoordinates_t, 'velocity coordinates vector of node (derivative of the 3 displacement coordinates + 3 slope vector coordinates)'),
        ItemOutputVariable(OVCoordinates_tt, 'acceleration coordinates vector of node (derivative of the 3 displacement coordinates + 3 slope vector coordinates)'),
        ],
    visuParentClass=VisuParentClassVisualizationNode,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"node's unique name"),
        ItemParameter(type=TVectorND(6), destination=DestComp+DestParam,
            pythonName='referenceCoordinates',
            defaultValue='Vector6D({0.,0.,0.,1.,0.,0.})',
            description=r'reference coordinates (x-pos,y-pos,z-pos; x-slopex, y-slopex, z-slopex) of node; global position of node without displacement'),
        ItemParameter(type=TVectorND(6), destination=DestMain+DestParam,
            pythonName='initialCoordinates',
            defaultValue='Vector6D({0.,0.,0.,0.,0.,0.})',
            description=r"initial displacement coordinates: ux, uy, uz and x/y/z 'displacements' of slopex"),
        ItemParameter(type=TVectorND(6), destination=DestMain+DestParam,
            pythonName='initialVelocities',
            cplusplusName='initialCoordinates_t',
            defaultValue='Vector6D({0.,0.,0.,0.,0.,0.})',
            description=r'initial velocity coordinates'),
        ItemFunctionDef('GetNumberOfODE2Coordinates',
            implementation='return 6;'),
        ItemTypes('Node', ['Position', 'PointSlope1'],
            description=r'return node type (for node treatment in computation)'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetAcceleration'),
        ItemFunctionDef('GetRotationMatrix',
            description=r"""return configuration dependent rotation matrix of node; the slope vector $\rv^\prime = [1,0]$ is defines as zero angle ($\varphi = 0$), leading to a matrix $\Am = \mr{\cos\varphi}{-\sin\varphi}{0} {\sin\varphi}{\cos\varphi}{0} {0}{0}{1}$; the function always computes a 3D Matrix"""),
        ItemFunctionDef('GetPositionJacobian'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "PointSlope1";',
            description=r"Get type name of node (without keyword 'Node'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('GetReferenceCoordinateVector',
            implementation='return parameters.referenceCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector',
            implementation='return parameters.initialCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector_t',
            implementation='return parameters.initialCoordinates_t;'),
        ItemFunctionDef('GetOutputVariable'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   NodePointSlope12   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='NodePointSlope12',
    addPublicC=r"""    static constexpr Index nODE2coordinates = 9;//AUTO: number of coordinates, used for fixed-size templates
""",
    cParentClass=ParentClassCNodeODE2,
    overallDescription=r"""A 3D point/slope vector node for thin ANCF (absolute nodal coordinate formulation) plate elements, with 3 position and 2 $\times$ 3 slope coordinates, all ABRV:ODE2; the slope vectors are the derivatives of the position with respect to the two in-plane coordinates of the plate.""",
    classType=ClassTypeNode,
    miniExample=r"""    #a square plate clamped at one edge, from ANCF thin plate elements with position and slopes r_x, r_y
    from exudyn.shells import ShellMesh
    plate = ShellMesh(vertices=[[0,0,0],[1,0,0],[1,1,0],[0,1,0]], numberOfElementsX=2, numberOfElementsY=2,
                      youngsModulus=2e9, poissonsRatio=0, density=1000, thickness=0.01)
    plate.CreateANCFThinPlateElements(mbs) #adds a NodePointSlope12 per mesh point
    for node in plate.boundaryNodeNumbers['left']:
        mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=node))
        mbs.CreateGenericJoint(bodyNumbers=[oGround, mNode]) #clamped: position and orientation
    for element in plate.elementNumbers:
        mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=element)),
                                         loadVector=[0,0,-9.81]))

    mbs.Assemble()
    mbs.SolveStatic()

    #a corner of the free edge; compare q*L^4/(8*D) = 0.0736 of a cantilever strip, D = E*h^3/12
    corner = plate.vertexNodeNumbers[1]
    exu.sys['testResult'] = mbs.GetNodeOutput(corner, exu.OutputVariableType.Displacement)[2]
    """,
    miniExamplePerformanceTest={'skip': True},
    detailedDescription=r"""    #### Coordinates

    | index | symbol | kind | meaning | frame |
    |---|---|---|---|---|
    | 0, 1, 2 | $q_0,\,q_1,\,q_2$ | ABRV:ODE2 | displacement of the node position $\rv$ | global |
    | 3, 4, 5 | $q_3,\,q_4,\,q_5$ | ABRV:ODE2 | change of the slope vector $\rv_x^\prime$ | global |
    | 6, 7, 8 | $q_6,\,q_7,\,q_8$ | ABRV:ODE2 | change of the slope vector $\rv_y^\prime$ | global |

    #### Configuration

    $$
    \rv = \rv\cRef + [q_0,\,q_1,\,q_2]\tp, \quad
    \rv_x^\prime = \rv_{x,\mathrm{ref}}^\prime + [q_3,\,q_4,\,q_5]\tp, \quad
    \rv_y^\prime = \rv_{y,\mathrm{ref}}^\prime + [q_6,\,q_7,\,q_8]\tp .
    $$

    #### The slope vectors

    The two slope vectors are the derivatives of the position of the mid-surface of a thin plate with
    respect to its two in-plane coordinates, $\rv_x^\prime = \partial \rv / \partial x$ and
    $\rv_y^\prime = \partial \rv / \partial y$. In a flat plate in the global $x$-$y$ plane they are
    $[1,\;0,\;0]\tp$ and $[0,\;1,\;0]\tp$. They span the tangent plane of the mid-surface, and their
    lengths and angle carry its in-plane strains. The default reference coordinates are those of a
    flat plate in the global $x$-$y$ plane; a model gives the reference coordinates of every node.

    #### Frame and interpretation

    All nine coordinates are global (absolute nodal coordinates). The node is used by
    `ObjectANCFThinPlate`, whose element scales the slopes by `slopesScalingX` and `slopesScalingY`.

    #### Action on the equations of motion

    The nine coordinates lead to nine ABRV:ODE2 equations, which the element provides; a force at the
    node enters the first three.
    """,
    mainParentClass=MainParentClassMainNode,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}\cConfig = \LU{0}{[p_0,\, p_1,\, p_2]}\cConfig\tp$global 3D position vector of node (=displacement+reference position)"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv}\cConfig = \LU{0}{[q_0,\, q_1,\, q_2]}\cConfig\tp$global 3D displacement vector of node"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\av}\cConfig = \LU{0}{[\dot q_0,\,\dot q_1,\,\dot q_2]}\cConfig\tp$global 3D velocity vector of node"""),
        ItemOutputVariable(OVAcceleration, r"""$\LU{0}{\av}\cConfig = \LU{0}{[\ddot q_0,\,\ddot q_1,\,\ddot q_2]}\cConfig\tp$global 3D acceleration vector of node"""),
        ItemOutputVariable(OVCoordinatesTotal, OVDCoordinatesTotalNode),
        ItemOutputVariable(OVCoordinates, 'coordinate vector of node (relative to reference configuration)'),
        ItemOutputVariable(OVCoordinates_t, 'velocity coordinates vector of node'),
        ItemOutputVariable(OVCoordinates_tt, 'acceleration coordinates vector of node'),
        ItemOutputVariable(OVRotationMatrix, OVDRotationMatrixRowMajor),
        ItemOutputVariable(OVRotation, r"""$[\varphi_0,\,\varphi_1,\,\varphi_2]\tp\cConfig$vector with 3 components of the Euler / Tait-Bryan angles in xyz-sequence"""),
        ItemOutputVariable(OVAngularVelocity, OVDAngularVelocityNode),
        ItemOutputVariable(OVAngularVelocityLocal, OVDAngularVelocityLocalNode),
        ],
    visuParentClass=VisuParentClassVisualizationNode,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"node's unique name"),
        ItemParameter(type=TVectorND(9), destination=DestComp+DestParam,
            pythonName='referenceCoordinates',
            defaultValue='Vector9D({0.,0.,0.,1.,0.,0.,0.,1.,0.})',
            description=r'reference coordinates (x-pos,y-pos,z-pos; x-slopeX, y-slopeX, z-slopeX; x-slopeY, y-slopeY, z-slopeY) of node; global position of node without displacement'),
        ItemParameter(type=TVectorND(9), destination=DestMain+DestParam,
            pythonName='initialCoordinates',
            defaultValue='Vector9D({0.,0.,0.,0.,0.,0.,0.,0.,0.})',
            description=r'initial displacement coordinates relative to reference coordinates'),
        ItemParameter(type=TVectorND(9), destination=DestMain+DestParam,
            pythonName='initialVelocities',
            cplusplusName='initialCoordinates_t',
            defaultValue='Vector9D({0.,0.,0.,0.,0.,0.,0.,0.,0.})',
            description=r'initial velocity coordinates'),
        ItemFunctionDef('GetNumberOfODE2Coordinates',
            implementation='return 9;'),
        ItemTypes('Node', ['Position', 'Orientation', 'PointSlope12'],
            description=r'return node type (for node treatment in computation)'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetAcceleration'),
        ItemFunctionDef('GetRotationMatrix'),
        ItemFunction(type=TMatrixND(3, 3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetRotationMatrix_t',
            args='ConfigurationType configuration = ConfigurationType::Current',
            description=r'return configuration dependent time derivative of rotation matrix of node'),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetAngularVelocityLocal'),
        ItemFunctionDef('GetPositionJacobian'),
        ItemFunctionDef('GetRotationJacobian'),
        ItemFunctionDef('GetRotationJacobianTTimesVector_q'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "PointSlope12";',
            description=r"Get type name of node (without keyword 'Node'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('GetReferenceCoordinateVector',
            implementation='return parameters.referenceCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector',
            implementation='return parameters.initialCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector_t',
            implementation='return parameters.initialCoordinates_t;'),
        ItemFunctionDef('GetOutputVariable'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   NodePointSlope23   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='NodePointSlope23',
    addPublicC=r"""    static constexpr Index nODE2coordinates = 9;//AUTO: number of coordinates, used for fixed-size templates
""",
    cParentClass=ParentClassCNodeODE2,
    overallDescription=r"""A 3D point/slope vector node for spatial, shear and cross-section deformable ANCF (absolute nodal coordinate formulation) beam elements, with 3 position and 2 $\times$ 3 slope coordinates, all ABRV:ODE2; the slope vectors are the derivatives of the position with respect to the two cross section coordinates $y$ and $z$.""",
    classType=ClassTypeNode,
    miniExample=r"""    #a cantilever of four ANCF beam elements: position and the slopes r_y, r_z of the cross section
    L = 1; nElements = 4; F = -0.1
    section = exu.BeamSection()
    section.stiffnessMatrix = np.diag([1e5, 1e4, 1e4, 100, 100, 100]) #EA, GA_y, GA_z, GJ, EI_y, EI_z
    section.massPerLength = 1
    section.inertia = 0.01*np.eye(3)
    mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
    n0 = mbs.AddNode(NodePointSlope23(referenceCoordinates=[0,0,0, 0,1,0, 0,0,1]))
    for i in range(9): #clamped: position and both slopes
        mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mGround,
                      mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=i))]))
    for k in range(nElements):
        n1 = mbs.AddNode(NodePointSlope23(referenceCoordinates=[L*(k+1)/nElements,0,0, 0,1,0, 0,0,1]))
        mbs.AddObject(ObjectANCFBeam(nodeNumbers=[n0,n1], physicsLength=L/nElements, sectionData=section))
        n0 = n1
    mTip = mbs.AddMarker(MarkerNodeRigid(nodeNumber=n1))
    mbs.AddLoad(LoadForceVector(markerNumber=mTip, loadVector=[0,F,0]))

    mbs.Assemble()
    mbs.SolveStatic()

    #converges to F*L^3/(3*EI_z) + F*L/GA_y = -0.3433e-3 as the number of elements grows
    exu.sys['testResult'] = mbs.GetNodeOutput(n1, exu.OutputVariableType.Displacement)[1]*1000 #-0.338
    """,
    miniExamplePerformanceTest={'numberOfSteps': 2940},
    detailedDescription=r"""    #### Coordinates

    | index | symbol | kind | meaning | frame |
    |---|---|---|---|---|
    | 0, 1, 2 | $q_0,\,q_1,\,q_2$ | ABRV:ODE2 | displacement of the node position $\rv$ | global |
    | 3, 4, 5 | $q_3,\,q_4,\,q_5$ | ABRV:ODE2 | change of the slope vector $\rv_y$ | global |
    | 6, 7, 8 | $q_6,\,q_7,\,q_8$ | ABRV:ODE2 | change of the slope vector $\rv_z$ | global |

    #### Configuration

    $$
    \rv = \rv\cRef + [q_0,\,q_1,\,q_2]\tp, \quad
    \rv_y = \rv_{y,\mathrm{ref}} + [q_3,\,q_4,\,q_5]\tp, \quad
    \rv_z = \rv_{z,\mathrm{ref}} + [q_6,\,q_7,\,q_8]\tp .
    $$

    #### The slope vectors

    Unlike the slopes of the cable nodes, the two slope vectors of this node are **not** taken along the
    beam axis: they are the derivatives of the position with respect to the two **cross section**
    coordinates $y$ and $z$, $\rv_y = \partial \rv / \partial y$ and $\rv_z = \partial \rv / \partial z$,
    so that a point of the cross section at $(y,\,z)$ is at $\rv + y\,\rv_y + z\,\rv_z$, as
    `ObjectANCFBeam` computes it. The axial direction follows from the positions of the two nodes of the
    element. In a beam along the global $x$-axis the slope vectors are $[0,\;1,\;0]\tp$ and
    $[0,\;0,\;1]\tp$; they span the cross section, and their lengths and angle carry its deformation -
    the element is shear and cross section deformable. The default reference coordinates are those
    of a beam along the global $x$-axis; a model gives the reference coordinates of every node.

    #### Frame and interpretation

    All nine coordinates are global (absolute nodal coordinates). The node is used by `ObjectANCFBeam`.

    #### Rotation

    `MarkerNodeRigid` and the output variables of the rotation take the orthonormal frame of the slopes:
    $\rv_z$ normalized is its $z$-axis, $\rv_y$ orthogonalized against it and normalized its $y$-axis, and the
    $x$-axis is their cross product. The angular velocity $\tomega$ and the rotation Jacobian
    $\Jm_{rot} = \partial \tomega / \partial \dot \qv$ are the derivatives of this frame, so joints and torques act
    consistently with it (#2763).

    #### Action on the equations of motion

    The nine coordinates lead to nine ABRV:ODE2 equations, which the element provides; a force at the
    node enters the first three.
    """,
    mainParentClass=MainParentClassMainNode,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}\cConfig = \LU{0}{[p_0,\, p_1,\, p_2]}\cConfig\tp$global 3D position vector of node (=displacement+reference position)"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv}\cConfig = \LU{0}{[q_0,\, q_1,\, q_2]}\cConfig\tp$global 3D displacement vector of node"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\av}\cConfig = \LU{0}{[\dot q_0,\,\dot q_1,\,\dot q_2]}\cConfig\tp$global 3D velocity vector of node"""),
        ItemOutputVariable(OVAcceleration, r"""$\LU{0}{\av}\cConfig = \LU{0}{[\ddot q_0,\,\ddot q_1,\,\ddot q_2]}\cConfig\tp$global 3D acceleration vector of node"""),
        ItemOutputVariable(OVCoordinatesTotal, OVDCoordinatesTotalNode),
        ItemOutputVariable(OVCoordinates, 'coordinate vector of node (relative to reference configuration)'),
        ItemOutputVariable(OVCoordinates_t, 'velocity coordinates vector of node'),
        ItemOutputVariable(OVCoordinates_tt, 'acceleration coordinates vector of node'),
        ItemOutputVariable(OVRotationMatrix, OVDRotationMatrixRowMajor),
        ItemOutputVariable(OVRotation, r"""$[\varphi_0,\,\varphi_1,\,\varphi_2]\tp\cConfig$vector with 3 components of the Euler / Tait-Bryan angles in xyz-sequence"""),
        ItemOutputVariable(OVAngularVelocity, OVDAngularVelocityNode),
        ItemOutputVariable(OVAngularVelocityLocal, OVDAngularVelocityLocalNode),
        ],
    visuParentClass=VisuParentClassVisualizationNode,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"node's unique name"),
        ItemParameter(type=TVectorND(9), destination=DestComp+DestParam,
            pythonName='referenceCoordinates',
            defaultValue='Vector9D({0.,0.,0.,0.,1.,0.,0.,0.,1.})',
            description=r'reference coordinates (x-pos,y-pos,z-pos; x-slopey, y-slopey, z-slopey; x-slopez, y-slopez, z-slopez) of node; global position of node without displacement'),
        ItemParameter(type=TVectorND(9), destination=DestMain+DestParam,
            pythonName='initialCoordinates',
            defaultValue='Vector9D({0.,0.,0.,0.,0.,0.,0.,0.,0.})',
            description=r'initial displacement coordinates relative to reference coordinates'),
        ItemParameter(type=TVectorND(9), destination=DestMain+DestParam,
            pythonName='initialVelocities',
            cplusplusName='initialCoordinates_t',
            defaultValue='Vector9D({0.,0.,0.,0.,0.,0.,0.,0.,0.})',
            description=r'initial velocity coordinates'),
        ItemFunctionDef('GetNumberOfODE2Coordinates',
            implementation='return 9;'),
        ItemTypes('Node', ['Position', 'Orientation', 'PointSlope23'],
            description=r'return node type (for node treatment in computation)'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetAcceleration'),
        ItemFunctionDef('GetRotationMatrix'),
        ItemFunction(type=TMatrixND(3, 3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetRotationMatrix_t',
            args='ConfigurationType configuration = ConfigurationType::Current',
            description=r'return configuration dependent time derivative of rotation matrix of node'),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetAngularVelocityLocal'),
        ItemFunctionDef('GetPositionJacobian'),
        ItemFunctionDef('GetRotationJacobian'),
        ItemFunctionDef('GetRotationJacobianTTimesVector_q'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "PointSlope23";',
            description=r"Get type name of node (without keyword 'Node'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('GetReferenceCoordinateVector',
            implementation='return parameters.referenceCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector',
            implementation='return parameters.initialCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector_t',
            implementation='return parameters.initialCoordinates_t;'),
        ItemFunctionDef('GetOutputVariable'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   NodeGenericODE2   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='NodeGenericODE2',
    cParentClass=ParentClassCNodeODE2,
    overallDescription=r"""A node containing a number of ABRV:ODE2 variables. Use this node e.g. for scalar dynamic equations (Mass1D), for ObjectGenericODE2 or for the Eulerian coordinate in the ALECable element. NOTE: referenceCoordinates and all initialCoordinates(\_t) must be initialized, because no default values exist.""",
    classType=ClassTypeNode,
    miniExample=r"""    #two coordinates of a user-defined second-order system: a mass on a spring and a free mass
    node = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=2, referenceCoordinates=[0,0],
                                       initialCoordinates=[0.1,0], initialCoordinates_t=[0,1]))
    M = np.diag([1,1])
    K = np.diag([(2*np.pi)**2, 0]) #eigenfrequency 1 Hz for the first coordinate
    mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node], massMatrix=M, stiffnessMatrix=K))

    mbs.Assemble()
    mbs.SolveDynamic()

    #after one period, q0 = 0.1 again; q1 = 1*t
    exu.sys['testResult'] = sum(mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)) #1.1
    """,
    miniExamplePerformanceTest={'numberOfSteps': 1316310},
    detailedDescription=r"""    #### Coordinates

    A number of ABRV:ODE2 coordinates, `numberOfODE2Coordinates`, whose meaning is **defined by the
    object that uses the node**: the modal coordinates of `ObjectFFRFreducedOrder`, the joint
    coordinates of `ObjectKinematicTree`, the coordinates of `ObjectGenericODE2`, or the axial motion of
    `ObjectALEANCFCable2D`. The current value of a coordinate is its reference value plus its current
    coordinate, $c_i = q_{i,\mathrm{ref}} + q_i$.

    #### Action on the equations of motion

    Each coordinate leads to one ABRV:ODE2 equation, which the object provides; the node itself adds
    nothing.
    """,
    mainParentClass=MainParentClassMainNode,
    outputVariables=[
        ItemOutputVariable(OVCoordinatesTotal, OVDCoordinatesTotalNode),
        ItemOutputVariable(OVCoordinates, r"""$\qv\cConfig = [q_0,\,\ldots,\,q_{nc}]\tp\cConfig$coordinates vector of node"""),
        ItemOutputVariable(OVCoordinates_t, r"""$\dot \qv\cConfig = [\dot q_0,\,\ldots,\,\dot q_{nc}]\tp\cConfig$velocity coordinates vector of node"""),
        ItemOutputVariable(OVCoordinates_tt, r"""$\ddot \qv\cConfig = [\ddot q_0,\,\ldots,\,\ddot q_{nc}]\tp\cConfig$acceleration coordinates vector of node"""),
        ],
    visuParentClass=VisuParentClassVisualizationNode,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"node's unique name"),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='referenceCoordinates',
            defaultValue='Vector()',
            description=r"""$\qv\cRef = [q_0,\,\ldots,\,q_{nc}]\tp\cRef$generic reference coordinates of node; must be consistent with numberOfODE2Coordinates"""),
        ItemParameter(type=TVector, destination=DestMain+DestParam,
            pythonName='initialCoordinates',
            defaultValue='Vector()',
            description=r"""$\qv\cIni = [q_0,\,\ldots,\,q_{nc}]\tp\cIni$initial displacement coordinates; must be consistent with numberOfODE2Coordinates"""),
        ItemParameter(type=TVector, destination=DestMain+DestParam,
            pythonName='initialCoordinates_t',
            defaultValue='Vector()',
            description=r"""$\dot \qv\cIni = [\dot q_0,\,\ldots,\,\dot q_{n_c}]\tp\cIni$initial velocity coordinates; must be consistent with numberOfODE2Coordinates"""),
        ItemParameter(type=TIndex(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='numberOfODE2Coordinates',
            defaultValue=0,
            description=r'$n_c$number of generic ABRV:ODE2 coordinates'),
        ItemFunctionDef('GetNumberOfODE2Coordinates',
            implementation='return parameters.numberOfODE2Coordinates;'),
        ItemTypes('Node', ['GenericODE2'],
            description=r'return node type (for node treatment in computation)'),
        ItemFunctionDef('GetPosition',
            implementation='return Vector3D({0.,0.,0.});',
            description="return configuration dependent position of node; returns always a 3D Vector; this makes no sense for NodeGenericODE2, but necessary for consistency; FUTURE: add 'drawable' flag to nodes in order to exclude drawing"),
        ItemFunctionDef('GetVelocity',
            implementation='return Vector3D({0.,0.,0.});',
            description='dummy function to avoid problems with markers, etc.'),
        ItemFunctionDef('GetAcceleration',
            implementation='return Vector3D({0.,0.,0.});',
            description='dummy function to avoid problems with markers, etc.'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "GenericODE2";',
            description=r"Get type name of node (without keyword 'Node'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('GetReferenceCoordinateVector',
            implementation='return parameters.referenceCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector',
            implementation='return parameters.initialCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector_t',
            implementation='return parameters.initialCoordinates_t;'),
        ItemFunctionDef('GetOutputVariable'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=False,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics',
            implementation=';',
            description='Empty graphics update for now'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   NodeGenericODE1   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='NodeGenericODE1',
    cParentClass=ParentClassCNodeODE1,
    overallDescription=r"""A node containing a number of ABRV:ODE1 variables. Use this node e.g. for linear state space systems. NOTE: referenceCoordinates and initialCoordinates must be initialized, because no default values exist.""",
    classType=ClassTypeNode,
    miniExample=r"""    #a first-order system q_t = A q, here an exponential decay
    node = mbs.AddNode(NodeGenericODE1(numberOfODE1Coordinates=1, referenceCoordinates=[0],
                                       initialCoordinates=[1]))
    mbs.AddObject(ObjectGenericODE1(nodeNumbers=[node], systemMatrix=[[-1]]))

    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.RK44)

    #q(1) = exp(-1)
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates) #0.3679
    """,
    miniExamplePerformanceTest={'numberOfSteps': 1666940},
    detailedDescription=r"""    #### Coordinates

    A number of ABRV:ODE1 coordinates, `numberOfODE1Coordinates`, whose meaning is **defined by the
    object that uses the node**, such as the states of `ObjectGenericODE1`. The current value of a
    coordinate is its reference value plus its current coordinate.

    #### Action on the equations of motion

    Each coordinate leads to one first order equation, which the object provides; a load acts on a
    coordinate through `MarkerNodeODE1Coordinate`.
    """,
    mainParentClass=MainParentClassMainNode,
    outputVariables=[
        ItemOutputVariable(OVCoordinatesTotal, OVDCoordinatesTotalNode),
        ItemOutputVariable(OVCoordinates, r"""$\yv\cConfig = [y_0,\,\ldots,\,y_{nc}]\tp\cConfig$ABRV:ODE1 coordinates vector of node"""),
        ItemOutputVariable(OVCoordinates_t, r"""$\dot \yv\cConfig = [\dot y_0,\,\ldots,\,\dot y_{nc}]\tp\cConfig$ABRV:ODE1 velocity coordinates vector of node"""),
        ],
    visuParentClass=VisuParentClassVisualizationNode,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"node's unique name"),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='referenceCoordinates',
            defaultValue='Vector()',
            description=r"""$\yv\cRef = [y_0,\,\ldots,\,y_{nc}]\tp\cRef$generic reference coordinates of node; must be consistent with numberOfODE1Coordinates"""),
        ItemParameter(type=TVector, destination=DestMain+DestParam,
            pythonName='initialCoordinates',
            defaultValue='Vector()',
            description=r"""$\yv\cIni = [y_0,\,\ldots,\,y_{nc}]\tp\cIni$initial displacement coordinates; must be consistent with numberOfODE1Coordinates"""),
        ItemParameter(type=TIndex(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='numberOfODE1Coordinates',
            defaultValue=0,
            description=r'$n_c$number of generic ABRV:ODE1 coordinates'),
        ItemFunctionDef('GetNumberOfODE1Coordinates',
            implementation='return parameters.numberOfODE1Coordinates;'),
        ItemTypes('Node', ['GenericODE1'],
            description=r'return node type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "GenericODE1";',
            description=r"Get type name of node (without keyword 'Node'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('GetReferenceCoordinateVector',
            implementation='return parameters.referenceCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector',
            implementation='return parameters.initialCoordinates;'),
        ItemFunctionDef('GetOutputVariable'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=False,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics',
            implementation=';',
            description='Empty graphics update for now'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   NodeGenericAE   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='NodeGenericAE',
    cParentClass=ParentClassCNodeAE,
    overallDescription=r"""A node containing a number of ABRV:AE variables. Use e.g. linear state space systems. NOTE: referenceCoordinates and initialCoordinates must be initialized, because no default values exist.""",
    classType=ClassTypeNode,
    detailedDescription=r"""    #### Coordinates

    A number of ABRV:AE coordinates, `numberOfAECoordinates`, whose meaning is **defined by the object
    that uses the node**; the number of algebraic equations may differ from the number of coordinates,
    if other objects provide the equations.

    #### Action on the equations of motion

    The coordinates are algebraic variables: they add algebraic equations, which the objects provide.
    """,
    mainParentClass=MainParentClassMainNode,
    outputVariables=[
        ItemOutputVariable(OVCoordinates, r"""$\yv\cConfig = [y_0,\,\ldots,\,y_{nc}]\tp\cConfig$ABRV:AE coordinates vector of node"""),
        ],
    visuParentClass=VisuParentClassVisualizationNode,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"node's unique name"),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='referenceCoordinates',
            defaultValue='Vector()',
            description=r"""$\yv\cRef = [y_0,\,\ldots,\,y_{nc}]\tp\cRef$generic reference coordinates of node; must be consistent with numberOfAECoordinates"""),
        ItemParameter(type=TVector, destination=DestMain+DestParam,
            pythonName='initialCoordinates',
            defaultValue='Vector()',
            description=r"""$\yv\cIni = [y_0,\,\ldots,\,y_{nc}]\tp\cIni$initial displacement coordinates; must be consistent with numberOfAECoordinates"""),
        ItemParameter(type=TIndex(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='numberOfAECoordinates',
            defaultValue=0,
            description=r'$n_c$number of generic ABRV:AE coordinates'),
        ItemFunctionDef('GetNumberOfAECoordinates',
            implementation='return parameters.numberOfAECoordinates;'),
        ItemTypes('Node', ['GenericAE'],
            description=r'return node type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "GenericAE";',
            description=r"Get type name of node (without keyword 'Node'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('GetReferenceCoordinateVector',
            implementation='return parameters.referenceCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector',
            implementation='return parameters.initialCoordinates;'),
        ItemFunctionDef('GetOutputVariable'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=False,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics',
            implementation=';',
            description='Empty graphics update for now'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   NodeGenericData   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='NodeGenericData',
    cParentClass=ParentClassCNodeData,
    overallDescription=r'A node containing a number of data (history) variables. Use this node e.g. for contact (active set), friction or plasticity (history variables).',
    classType=ClassTypeNode,
    miniExample=r"""    #data coordinates hold a state that is no degree of freedom and that the object updates after each step:
    #here the limit stop of a connector, which a mass is pushed against
    node = mbs.AddNode(Node1D(referenceCoordinates=[0]))
    mbs.AddObject(ObjectMass1D(nodeNumber=node, physicsMass=1))
    mMass = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=0))
    mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
    nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=3, initialCoordinates=[0,0,0]))
    mbs.AddObject(ObjectConnectorCoordinateSpringDamperExt(markerNumbers=[mGround, mMass], nodeNumber=nData,
                  damping=20, useLimitStops=True, limitStopsLower=-1, limitStopsUpper=0.05,
                  limitStopsStiffness=1e4, limitStopsDamping=100))
    mbs.AddLoad(LoadCoordinate(markerNumber=mMass, load=10))

    mbs.Assemble()
    mbs.SolveDynamic()

    #the mass rests at the stop, pressed into it by F/k_limits
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates) #0.05+0.001
    """,
    miniExamplePerformanceTest={'numberOfSteps': 1229520},
    detailedDescription=r"""    #### Coordinates

    A number of data coordinates, `numberOfDataCoordinates`, whose meaning is **defined by the object
    that uses the node**: the contact state of a contact object, the stick or slip state and the last
    position of friction, plastic strains. Data coordinates are no unknowns of the equations of motion;
    the object updates them between steps, in its post Newton step.

    #### Action on the equations of motion

    None directly: the data coordinates change the equations of the object that owns them, e.g. by
    switching a contact on or off.
    """,
    mainParentClass=MainParentClassMainNode,
    outputVariables=[
        ItemOutputVariable(OVCoordinates, r"""$\xv\cConfig = [x_0,\,\ldots,\,x_{nc}]\tp\cConfig$data coordinates (history variables) vector of node"""),
        ],
    visuParentClass=VisuParentClassVisualizationNode,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"node's unique name"),
        ItemParameter(type=TVector, destination=DestMain+DestParam,
            pythonName='initialCoordinates',
            defaultValue='Vector()',
            description=r"""$\xv\cIni = [x_0,\,\ldots,\,x_{n_c}]\tp\cIni$initial data coordinates"""),
        ItemParameter(type=TIndex(minimum=0), destination=DestComp+DestParam,
            pythonName='numberOfDataCoordinates',
            defaultValue=0,
            description=r'$n_c$number of generic data coordinates (history variables)'),
        ItemFunctionDef('GetNumberOfDataCoordinates',
            implementation='return parameters.numberOfDataCoordinates;'),
        ItemTypes('Node', ['GenericData'],
            description=r'return node type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "GenericData";',
            description=r"Get type name of node (without keyword 'Node'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('GetInitialCoordinateVector',
            implementation='return parameters.initialCoordinates;',
            description='return internally stored initial data coordinates of node'),
        ItemFunctionDef('GetOutputVariable'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=False,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics',
            implementation=';',
            description='Empty graphics update for now'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   NodePointGround   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='NodePointGround',
    cParentClass=ParentClassCNodeODE2,
    overallDescription=r"""A 3D point node fixed to ground which is similar to NodePoint, but it does not generate coordinates. Applied or reaction forces do not have any effect. This node can be used for 'blind' or 'dummy' ABRV:ODE2 and ABRV:ODE1 coordinates to which CoordinateSpringDamper or CoordinateConstraint objects are attached to.""",
    classType=ClassTypeNode,
    examples=['Examples/springDamperTutorial.py', 'Examples/coordinateSpringDamper.py', 'Examples/SliderCrank.py', 'Examples/plotSensorExamples.py', 'Examples/SpringDamperMassUserFunction.py'],
    miniExample=r"""    #a ground node: a fixed point that markers and connectors can use, without coordinates
    node = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0]))
    mbs.AddObject(ObjectMassPoint(nodeNumber=node, physicsMass=1))
    mNode = mbs.AddMarker(MarkerNodePosition(nodeNumber=node))
    nFixed = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))
    mFixed = mbs.AddMarker(MarkerNodePosition(nodeNumber=nFixed))
    mbs.AddObject(ObjectConnectorCartesianSpringDamper(markerNumbers=[mFixed, mNode], stiffness=[100,100,100],
                                                       offset=[1,0,0]))
    mbs.AddLoad(LoadForceVector(markerNumber=mNode, loadVector=[10,0,0]))

    mbs.Assemble()
    mbs.SolveStatic()

    #the spring is stretched by F/k
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Displacement)[0] #0.1
    """,
    miniExamplePerformanceTest={'numberOfSteps': 1001500},
    detailedDescription=r"""    #### Coordinates

    None: the node is fixed at its reference position $\pv\cRef$, and does not add a coordinate to the
    system. It provides a position (and, formally, an orientation) so that markers can be attached to
    it.

    #### Frame and interpretation

    The reference position is global. Forces applied to the node, and reaction forces of connectors or
    constraints attached to it, have no effect.

    #### Use

    The node is the ground for coordinate markers: a `CoordinateSpringDamper` or a `CoordinateConstraint`
    between a coordinate of a node and the ground needs a marker on both sides, and `MarkerNodeCoordinate`
    on a `NodePointGround` is that side. `CreateCoordinateConstraint`
    creates it when one side is the ground.
    """,
    mainParentClass=MainParentClassMainNode,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\pv\cConfig = [p_0,\,p_1,\,p_2]\cConfig\tp = \pv\cRef$global 3D position vector of node (=reference position)"""),
        ItemOutputVariable(OVDisplacement, r'$\uv\cConfig = [0,\,0,\,0]\cConfig\tp$zero 3D vector'),
        ItemOutputVariable(OVVelocity, r'$\vv\cConfig = [0,\,0,\,0]\cConfig\tp$zero 3D vector'),
        ItemOutputVariable(OVCoordinatesTotal, r'$\cv\cConfig =[]$vector of length zero'),
        ItemOutputVariable(OVCoordinates, r'$\cv\cConfig =[]$vector of length zero'),
        ItemOutputVariable(OVCoordinates_t, r'$\dot\cv\cConfig =[]$vector of length zero'),
        ItemOutputVariable(OVRotationMatrix, OVDIdentityMatrixForCompleteness),
        ItemOutputVariable(OVRotation, OVDZeroVectorForCompleteness),
        ItemOutputVariable(OVAngularVelocity, OVDZeroVectorForCompleteness),
        ItemOutputVariable(OVAngularVelocityLocal, OVDZeroVectorForCompleteness),
        ],
    pythonShortName='PointGround',
    visuParentClass=VisuParentClassVisualizationNode,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"node's unique name"),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='referenceCoordinates',
            defaultValue=DVZeroVector3D,
            description=r"""$\qv\cRef = [q_0,\,q_1,\,q_2]\tp\cRef = \pv\cRef = [r_0,\,r_1,\,r_2]\tp$reference coordinates of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement"""),
        ItemTypes('Node', ['Position', 'Position2D', 'Orientation', 'GenericODE2', 'Ground'],
            description=r'return node type (for node treatment in computation)'),
        ItemFunctionDef('GetPosition',
            implementation='return parameters.referenceCoordinates;',
            description='Returns position of node, which is the reference position for all configurations'),
        ItemFunctionDef('GetVelocity',
            implementation='return Vector3D(0.);',
            description='Returns zero velocity'),
        ItemFunctionDef('GetRotationMatrix',
            implementation='return EXUmath::unitMatrix3D;'),
        ItemFunctionDef('GetAngularVelocityLocal',
            implementation='return Vector3D(0.);'),
        ItemFunctionDef('GetAngularVelocity',
            implementation='return Vector3D(0.);'),
        ItemFunctionDef('GetPositionJacobian',
            implementation='value.SetNumberOfRowsAndColumns(0,0);'),
        ItemFunctionDef('GetRotationJacobian',
            implementation='value.SetNumberOfRowsAndColumns(0,0);'),
        ItemFunctionDef('GetRotationJacobianTTimesVector_q',
            implementation='jacobian_q.SetNumberOfRowsAndColumns(0, 0);'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "PointGround";',
            description=r"Get type name of node (without keyword 'Node'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('GetReferenceCoordinateVector',
            implementation='return parameters.referenceCoordinates;'),
        ItemFunctionDef('GetInitialCoordinateVector',
            implementation='return LinkedDataVector();',
            description='return empty vector, as there are no initial coordinates'),
        ItemFunctionDef('GetInitialCoordinateVector_t',
            implementation='return LinkedDataVector();',
            description='return empty vector, as there are no initial velocity coordinates'),
        ItemFunctionDef('GetOutputVariable'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used'),
        ],
    ))
