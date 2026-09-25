#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Node item definitions
#
# Details:  16 definitions; the input of the generators.
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
    classDescription=r"""A 3D point node for point masses or solid finite elements which has 3 displacement degrees of freedom for ABRV:ODE2.""",
    classType=ClassTypeNode,
    equations=r"""    \paragraph{Detailed information:}
    The node provides $n_c=3$ displacement coordinates. Equations of motion need to be provided by an according object (e.g., MassPoint, finite elements, ...).
    Usually, the nodal coordinates are provided in the global frame. However, the coordinate system is defined by the object (e.g. MassPoint uses global coordinates, but floating frame of reference objects use local frames).
    Note that for this very simple node, coordinates are identical to the nodal displacements, same for time derivatives. This is not the case, e.g. for nodes with orientation. \vspace{6pt}\\

    \noindent {\bf Example} for NodePoint: see ObjectMassPoint, [](#sec-item-objectmasspoint)
    %%RSTCOMPATIBLE
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
    classDescription=r"""A 2D point node for point masses or solid finite elements which has 2 displacement degrees of freedom for ABRV:ODE2.""",
    classType=ClassTypeNode,
    equations=r"""    \paragraph{Detailed information:}
    The node provides $n_c=2$ displacement coordinates. Equations of motion need to be provided by an according object (e.g., MassPoint2D).
    Coordinates are identical to the nodal displacements, except for the third coordinate $u_2$, which is zero, because $q_2$ does not exist. \vspace{6pt}\\
    Note that for this very simple node, coordinates are identical to the nodal displacements, same for time derivatives. This is not the case, e.g. for nodes with orientation. \vspace{6pt}\\
    
    \noindent {\bf Example} for NodePoint2D: see ObjectMassPoint2D, [](#sec-item-objectmasspoint2d)
    %%RSTCOMPATIBLE
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
    classDescription=r"""A 3D rigid body node based on Euler parameters for rigid bodies or beams. The node has 3 displacement coordinates (representing displacement of reference point $\LU{0}{\rv}$) and four rotation coordinates (Euler parameters = unit quaternions).""",
    classType=ClassTypeNode,
    equations=r"""    \paragraph{Detailed information:}
    All coordinates $\cv\cConfig$ lead to second order differential equations.
    The first 3 equations are residuals of translational forces in global coordinates,
    while the last 4 equations are residual of local torques left-multiplied with $\LU{b}{\Gm\tp}$ or
    global torques left-multiplied with $\LU{0}{\Gm\tp}$, see [](#eq-noderigidbodyep-gm), compare the equations of motion of
    the rigid body.
    
    There is one additional (algebraic) constraint equation for the quaternions.
    The additional constraint equation, which needs to be provided by the object, reads


    $$
    1 - \sum_{i=0}^{3} \theta_i^2 = 0.
    $$

    The rotation matrix $\LU{0b}{\Rot}\cConfig$ transforms a local (body-fixed) 3D position 
    $\pLocB = \LU{b}{[b_0,\,b_1,\,b_2]}\tp$ to global 3D positions,


    $$
    \LU{0}{\pLoc}\cConfig = \LU{0b}{\Rot}\cConfig \LU{b}{\pLoc}
    $$

    Note that the Euler parameters $\ttheta\cCur$ are computed as sum of current coordinates plus reference coordinates,


    $$
    \ttheta\cCur = \tpsi\cCur + \tpsi\cRef.
    $$

    The rotation matrix is defined as function of the rotation parameters $\ttheta=[\theta_0,\,\theta_1,\,\theta_2,\,\theta_3]\tp$


    $$
    \LU{0b}{\Rot} = \mr{-2\theta_3^2 - 2\theta_2^2+1}{-2\theta_3\theta_0+2\theta_2\theta_1}{2*\theta_3\theta_1+2*\theta_2\theta_0} 
                             {2\theta_3\theta_0+2\theta_2\theta_1}{-2\theta_3^2-2\theta_1^2+1}{2\theta_3\theta_2-2\theta_1\theta_0}
                             {-2\theta_2\theta_0+2\theta_3\theta_1}{2\theta_3\theta_2+2\theta_1\theta_0}{-2\theta_2^2-2\theta_1^2+1}
    $$

    The derivatives of the angular velocity vectors w.r.t.\ the rotation velocity coordinates $\dot \ttheta=[\dot \theta_0,\,\dot \theta_1,\,\dot \theta_2,\,\dot \theta_3]\tp$ lead to the $\Gm$ matrices, as used in the equations of motion for rigid bodies,


    $$
    \begin{aligned}
    \LU{0}{\tomega} &= \LU{0}{\Gm} \dot \ttheta, \\
          \LU{b}{\tomega} &= \LU{b}{\Gm} \dot \ttheta.
    \end{aligned}
    $$ (eq-noderigidbodyep-gm)

    For creating a \texttt{NodeRigidBodyEP} together with a rigid body, there is a \texttt{rigidBodyUtilities} function \texttt{CreateRigidBody}, 
    see [](#sec-mainsystemextensions-createrigidbody), which simplifies the setup of a rigid body significantely!
    %%RSTCOMPATIBLE
    %return ConstSizeMatrix<3*maxRotCoordinates>(3, 4, {  -2.*ep[1], 2.*ep[0],-2.*ep[3], 2.*ep[2],
    %                                    -2.*ep[2], 2.*ep[3], 2.*ep[0],-2.*ep[1],
    %                                    -2.*ep[3],-2.*ep[2], 2.*ep[1], 2.*ep[0] });
    %return ConstSizeMatrix<3*maxRotCoordinates>(3, 4, {  -2.*ep[1], 2.*ep[0], 2.*ep[3],-2.*ep[2],
    %                                    -2.*ep[2],-2.*ep[3], 2.*ep[0], 2.*ep[1],
    %                                    -2.*ep[3], 2.*ep[2],-2.*ep[1], 2.*ep[0] });
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
    classDescription=r"""A 3D rigid body node based on Euler / Tait-Bryan angles for rigid bodies or beams. All coordinates lead to second order differential equations; NOTE: this node has a singularity if the second rotation parameter reaches $\psi_1 = (2k-1) \pi/2$, with $k \in \Ncal$ or $-k \in \Ncal$.""",
    classType=ClassTypeNode,
    equations=r"""    \paragraph{Detailed information:}
    The node has 3 displacement coordinates $[q_0,\,q_1,\,q_2]\tp$ and 3 rotation coordinates $[\psi_0,\,\psi_1,\,\psi_2]\tp$ for consecutive rotations around the 0, 1 and 2-axis ($x$, $y$ and $z$).
    All coordinates $\cv\cConfig$ lead to second order differential equations.
    The rotation matrix $\LU{0b}{\Rot}\cConfig$ transforms a local (body-fixed) 3D position 
    $\pLocB = \LU{b}{[b_0,\,b_1,\,b_2]}\tp$ to global 3D positions,


    $$
    \LU{0}{\pLoc}\cConfig = \LU{0b}{\Rot}\cConfig \LU{b}{\pLoc}
    $$

    Note that the Euler angles $\ttheta\cCur$ are computed as sum of current coordinates plus reference coordinates,


    $$
    \ttheta\cCur = \tpsi\cCur + \tpsi\cRef.
    $$

    The rotation matrix is defined as function of the rotation parameters $\ttheta=[\theta_0,\,\theta_1,\,\theta_2]\tp$


    $$
    \LU{0b}{\Rot} = \LU{01}{\Rot_0}(\theta_0) \LU{12}{\Rot_1}(\theta_1) \LU{2b}{\Rot_2}(\theta_2)
    $$

    see [](#sec-symbolsitems) for definition of rotation matrices $\Rot_0$, $\Rot_1$ and $\Rot_2$.
    
    The derivatives of the angular velocity vectors w.r.t.\ the rotation velocity coordinates $\dot \ttheta=[\dot \theta_0,\,\dot \theta_1,\,\dot \theta_2]\tp$ lead to the $\Gm$ matrices, as used in the equations of motion for rigid bodies,


    $$
    \begin{aligned}
    \LU{0}{\tomega} &= \LU{0}{\Gm} \dot \ttheta, \\
          \LU{b}{\tomega} &= \LU{b}{\Gm} \dot \ttheta.
    \end{aligned}
    $$

    
    For creating a \texttt{NodeRigidBodyRxyz} together with a rigid body, there is a \texttt{rigidBodyUtilities} function \texttt{CreateRigidBody}, 
    see [](#sec-mainsystemextensions-createrigidbody), which simplifies the setup of a rigid body significantely!
    %%RSTCOMPATIBLE
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
    classDescription=r'A 3D rigid body node based on rotation vector and Lie group methods for rigid bodies. The node has 3 displacement coordinates and three rotation coordinates and can be used in combination with explicit Lie Group time integration methods.',
    classType=ClassTypeNode,
    equations=r"""    \paragraph{Detailed information:}
    For a detailed description on the rigid body dynamics formulation using this node, 
    see Holzinger and Gerstmayr [CITE:HolzingerGerstmayr2020].

    The node has 3 displacement coordinates $[q_0,\,q_1,\,q_2]\tp$ and three rotation coordinates, which is the rotation vector 


    $$
    \tnu = \varphi \nv = \tnu\cConfig + \tnu\cRef,
    $$

    with the rotation angle $\varphi$ and the rotation axis $\nv$.
    All coordinates $\cv\cConfig$ lead to second order differential equations, 
    However the rotation vector cannot be used as a conventional parameterization. 
    It must be computed within a nonlinear update, using appropriate Lie group methods.
    The first 3 equations are residuals of translational forces in global coordinates,
    while the last 3 equations are residual of local (body-fixed) torques, 
    compare the equations of motion of the rigid body.

    The rotation matrix $\LU{0b}{\Rot(\tnu)}\cConfig$ transforms a local (body-fixed) 3D position 
    $\pLocB = \LU{b}{[b_0,\,b_1,\,b_2]}\tp$ to global 3D positions,


    $$
    \LU{0}{\pLoc}\cConfig = \LU{0b}{\Rot(\tnu)}\cConfig \LU{b}{\pLoc}
    $$

    Note that $\Rot(\tnu)$ is defined in function \texttt{ RotationVector2RotationMatrix}, see [](#sec-rigidbodyutilities-rotationvector2rotationmatrix).
    
    A Lie group integrator must be used with this node, which is why the is used, the 
    rotation parameter velocities are identical to the local angular velocity $\LU{b}{\tomega}$ and thus the 
    matrix $ \LU{b}{\Gm}$ becomes the identity matrix.
    
    \mybold{Note}, that the node automatically switches to Lie group integration of its
    rotational coordinates, both in explicit integration as well as for implicit time integration.
    This node avoids typical singularities of rotations and is therefore perfectly suited
    for arbitrary motion. Furthermore, nonlinearities are reduced, which may improve
    implicit time integration performance.
    
    For creating a \texttt{NodeRigidBodyRotVecLG} together with a rigid body, there is a \texttt{rigidBodyUtilities} function \texttt{CreateRigidBody}, 
    see [](#sec-mainsystemextensions-createrigidbody), which simplifies the setup of a rigid body significantely!
    %%RSTCOMPATIBLE
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
    classDescription=r"""A 2D rigid body node for rigid bodies or beams. The node has 2 displacement degrees of freedom and one rotation coordinate (rotation around z-axis: $\psi_0$). All coordinates are ABRV:ODE2, used for second order differetial equations.""",
    classType=ClassTypeNode,
    equations=r"""    \paragraph{Detailed information:}
    The node provides 2 displacement coordinates (displacement of ABRV:COM, ($q_0,q_1$) ) and 1 rotation parameter ($\theta_0$). According equations need to be provided by an according object (e.g., RigidBody2D).
    The node leads to 3 ODE2 equations of motions, where the first 2 equations are
    residuals of global translational forces, and the third equation is the residual of the
    torque around the Z-axis (due to planar motion, local=global).

    Using the rotation parameter $\theta_{0\mathrm{config}} = \psi_{0ref} + \psi_{0\mathrm{config}}$, the rotation matrix is defined as


    $$
    \LU{0b}{\Rot}\cConfig = \mr{\cos(\theta_0)}{-\sin(\theta_0)}{0}{\sin(\theta_0)}{\cos(\theta_0)}{0}{0}{0}{1}\cConfig
    $$

    \noindent {\bf Example} for NodeRigidBody2D: see ObjectRigidBody2D
    %%RSTCOMPATIBLE
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
    classDescription=r"""A node with one ABRV:ODE2 coordinate for one dimensional (1D) problems. Use e.g. for scalar dynamic equations (Mass1D) and mass-spring-damper mechanisms, representing either translational or rotational degrees of freedom: in most cases, Node1D is equivalent to NodeGenericODE2 using one coordinate, however, it offers a transformation to 3D translational or rotational motion and allows to couple this node to 2D or 3D bodies.""",
    classType=ClassTypeNode,
    equations=r"""    \paragraph{Detailed information:}
    The current position/rotation coordinate of the 1D node is computed from


    $$
    p_0 = {q_0}\cRef + {q_0}\cCur
    $$

    The coordinate leads to one second order differential equation.
    The graphical representation and the (internal) position of the node is


    $$
    p\cConfig= \vr{{p_0}\cConfig}{0}{0}
    $$

    The (internal) velocity vector is $[{p_0}\cConfig,\,0,\,0]\tp$.
    %%RSTCOMPATIBLE
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
    classDescription=r"""A 2D point/slope vector node for planar Bernoulli-Euler ANCF (absolute nodal coordinate formulation) beam elements. The node has 4 displacement degrees of freedom (2 for displacement of point node and 2 for the slope vector 'slopex'); all coordinates lead to second order differential equations; the slope vector defines the directional derivative w.r.t the local axial (x) coordinate, denoted as $()^\prime$; in straight configuration aligned at the global x-axis, the slope vector reads $\rv^\prime=[r_x^\prime\;\;r_y^\prime]^T=[1\;\;0]^T$.""",
    classType=ClassTypeNode,
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
    classDescription=r"""A 3D point/slope vector node for spatial Bernoulli-Euler ANCF (absolute nodal coordinate formulation) beam elements. The node has 6 displacement degrees of freedom (3 for displacement of point node and 3 for the slope vector 'slopex'); all coordinates lead to second order differential equations; the slope vector defines the directional derivative w.r.t the local axial (x) coordinate, denoted as $()^\prime$; in straight configuration aligned at the global x-axis, the slope vector reads $\rv^\prime=[r_x^\prime\;\;r_y^\prime\;\;r_z^\prime]^T=[1\;\;0]^T$.""",
    classType=ClassTypeNode,
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
    classDescription=r"""A 3D point/slope vector node for thin ANCF (absolute nodal coordinate formulation) plate elements. The node has 9 ODE2 degrees of freedom (3 for displacement of point node and 2 $\times$ 3 for the slope vectors 'slopeX' and 'slopeY'); all coordinates lead to second order differential equations; the slopeX vector defines the directional derivative w.r.t the local axial (x) coordinate, etc.; in straight configuration aligned at the global x-axis, the slopeY vector reads $\rv_y^\prime=[0\;\;1\;\;0]^T$.""",
    classType=ClassTypeNode,
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
            defaultValue='Vector9D({0.,0.,0.,1.,0.,0.,1.,0.,0.})',
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
    classDescription=r"""A 3D point/slope vector node for spatial, shear and cross-section deformable ANCF (absolute nodal coordinate formulation) beam elements. The node has 9 ODE2 degrees of freedom (3 for displacement of point node and 2 $\times$ 3 for the slope vectors 'slopeY' and 'slopeZ'); all coordinates lead to second order differential equations; the slopeY vector defines the directional derivative w.r.t the local axial (y) coordinate, etc.; the slopeY vector reads $\rv_y^\prime=[0\;\;1\;\;0]^T$ and slopeZ gets $\rv_z^\prime=[0\;\;0\;\;1]^T$.""",
    classType=ClassTypeNode,
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
            defaultValue='Vector9D({0.,0.,0.,1.,0.,0.,1.,0.,0.})',
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
    classDescription=r"""A node containing a number of ABRV:ODE2 variables. Use this node e.g. for scalar dynamic equations (Mass1D), for ObjectGenericODE2 or for the Eulerian coordinate in the ALECable element. NOTE: referenceCoordinates and all initialCoordinates(\_t) must be initialized, because no default values exist.""",
    classType=ClassTypeNode,
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
    classDescription=r"""A node containing a number of ABRV:ODE1 variables. Use this node e.g. for linear state space systems. NOTE: referenceCoordinates and initialCoordinates must be initialized, because no default values exist.""",
    classType=ClassTypeNode,
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
    classDescription=r"""A node containing a number of ABRV:AE variables. Use e.g. linear state space systems. NOTE: referenceCoordinates and initialCoordinates must be initialized, because no default values exist.""",
    classType=ClassTypeNode,
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
    classDescription=r'A node containing a number of data (history) variables. Use this node e.g. for contact (active set), friction or plasticity (history variables).',
    classType=ClassTypeNode,
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
    classDescription=r"""A 3D point node fixed to ground which is similar to NodePoint, but it does not generate coordinates. Applied or reaction forces do not have any effect. This node can be used for 'blind' or 'dummy' ABRV:ODE2 and ABRV:ODE1 coordinates to which CoordinateSpringDamper or CoordinateConstraint objects are attached to.""",
    classType=ClassTypeNode,
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
