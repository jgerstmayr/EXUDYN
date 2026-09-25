#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Object item definitions
#
# Details:  51 definitions; the input of the generators.
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
# Contents: ObjectGround, ObjectMassPoint, ObjectMassPoint2D, ObjectMass1D, ObjectRotationalMass1D, ObjectRigidBody, ...
#
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from definitionTypes import *
from outputVariableTypes import *
from outputVariableDescriptions import *

definitions = []
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectGround   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectGround',
    addProtectedC=r"""    static constexpr Index nODE2coordinates = 0;
""",
    cParentClass=ParentClassCObjectBody,
    classDescription='A ground object behaving like a rigid body, but having no degrees of freedom. Used to attach body-connectors without an action. For examples see spring dampers and joints.',
    classType=ClassTypeObject,
    equations=r"""    #### Equations

    ObjectGround has no equations, as it only provides a static object, at which joints and connectors can be attached. 
    The object does not move (in general) and forces or torques do not have an effect.
    However, the reference position and rotation may be changed over time. This may prescribe
    motion, however, with the measured velocity still being zero at each time instant. Therefore,
    such manipulation of reference position or rotation shall be treated with care.
    
    In combination with markers, the \texttt{localPosition} $\pLocB$ is transformed by the \texttt{ObjectGround} to
    a global point $\LU{0}{\pv}$ using the reference point $\pRefG$,

    $$
    \LU{0}{\pv} = \pRefG + \LU{0b}{\Rot} \pLocB \, .
          %\LU{0}{\pv} = \pRefG + \LU{0b}{\ImThree} \pLocB
    $$

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `graphicsDataUserFunction(mbs, itemNumber)`
    A user function, which is called by the visualization thread in order to draw user-defined objects.
    The function can be used to generate any \texttt{BodyGraphicsData}, see Section [](#sec-graphicsdata).
    Use \texttt{exudyn.graphics} functions, see Section [](#sec-module-graphics), to create more complicated objects. 
    Note that \texttt{graphicsDataUserFunction} needs to copy lots of data and is therefore
    inefficient and only designed to enable simpler tests, but not large scale problems.
    <!-- -->

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides reference to mbs, which can be used in user function to access all data of the object |
    | \texttt{itemNumber} | Index | integer number of the object in mbs, allowing easy access |
    | **return value** | BodyGraphicsData | list of \texttt{GraphicsData} dictionaries, see Section [](#sec-graphicsdata) |

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    *Example*:
    
```python
import exudyn as exu
from math import sin, cos, pi
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics

SC = exu.SystemContainer()
mbs = SC.AddSystem()
#create simple system:
mbs.AddNode(NodePoint())
body = mbs.AddObject(MassPoint(physicsMass=1, nodeNumber=0))

#user function for moving graphics:
def UFgraphics(mbs, objectNum):
    t = mbs.systemData.GetTime(exu.ConfigurationType.Visualization) #get time if needed
    #draw moving sphere on ground
    graphics1=graphics.Sphere(point=[sin(t*2*pi), cos(t*2*pi), 0], 
                                 radius=0.1, color=graphics.color.red, nTiles=32)
    return [graphics1] 

#add object with graphics user function
ground = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsDataUserFunction=UFgraphics)))
mbs.Assemble()
sims=exu.SimulationSettings()
sims.timeIntegration.numberOfSteps = 10000000 #many steps to see graphics
SC.renderer.Start() #perform zoom all (press 'a' several times) after startup to see the sphere
mbs.SolveDynamic(sims)
SC.renderer.Stop()

```

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    objectType=ObjectTypeBody,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv} = \pRefG + \LU{0b}{\Rot} \pLocB$global position vector of translated local position"""),
        ItemOutputVariable(OVDisplacement, r'$\Null$global displacement vector of local position'),
        ItemOutputVariable(OVVelocity, r'$\Null$global velocity vector of local position'),
        ItemOutputVariable(OVAngularVelocity, r'$\Null$angular velocity of body'),
        ItemOutputVariable(OVRotationMatrix, r"""$\LU{0b}{\Rot}$rotation matrix in vector form (stored in row-major order)"""),
        ],
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='referencePosition',
            defaultValue=DVZeroVector3D,
            description=r"""$\pRefG$reference point = reference position for ground object; local position is added on top of reference position for a ground object"""),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp+DestParam,
            pythonName='referenceRotation',
            defaultValue='EXUmath::unitMatrix3D',
            description=r"""$\LU{0b}{\Rot} \in \Rcal^{3 \times 3}$the constant ground rotation matrix, which transforms body-fixed (b) to global (0) coordinates"""),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::_None);'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'AngularVelocity_qt', 'JacobianTtimesVector_q', 'DisplacementMassIntegral_q']),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetOutputVariableBody'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetDisplacement',
            implementation='return Vector3D({ 0.,0.,0. });'),
        ItemFunctionDef('GetVelocity',
            implementation='return Vector3D({ 0.,0.,0. });'),
        ItemFunctionDef('GetRotationMatrix',
            implementation='return parameters.referenceRotation;',
            description='return configuration dependent rotation matrix of node; returns always a 3D Matrix, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunctionDef('GetAngularVelocity',
            implementation='return Vector3D({ 0.,0.,0. });'),
        ItemFunctionDef('GetAngularVelocityLocal',
            implementation='return Vector3D({ 0.,0.,0. });',
            description='return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunctionDef('GetLocalCenterOfMass',
            implementation='return Vector3D({0.,0.,0.});',
            description='return the local position of the center of mass, needed for equations of motion and for massProportionalLoad -- not used for GroundObject'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "Ground";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(0, __EXUDYN_invalid_local_node0);
        return 0;""",
            description='No nodenumber can be returned for ground object!'),
        ItemFunctionDef('SetNodeNumber',
            implementation='CHECKandTHROW(0, __EXUDYN_invalid_local_node0);'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 0;'),
        ItemFunctionDef('GetODE2Size',
            implementation='return 0;'),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::Ground);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('CallUserFunction'),
        ItemFunctionDef('HasUserFunction',
            implementation='return graphicsDataUserFunction!=0;'),
        ItemParameter(type=TPyFunctionGraphicsData, destination=DestVisu,
            pythonName='graphicsDataUserFunction',
            defaultValue=0,
            description=r'A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function'),
        ItemParameter(type=TBodyGraphicsData, destination=DestVisu,
            pythonName='graphicsData',
            defaultValue=NoDefaultValue,
            description=r'Structure contains data for body visualization; data is defined in special list / dictionary structure'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectMassPoint   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectMassPoint',
    addProtectedC=r"""    static constexpr Index nODE2coordinates = 3;
""",
    cParentClass=ParentClassCObjectBody,
    classDescription=r'A 3D mass point which is attached to a position-based node, usually NodePoint.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities


    | intermediate variables | symbol | description |
    |---|---|---|
    | node position | $\LU{0}{\pRef}\cConfig + \LU{0}{\pRef}\cRef = \LU{0}{\pv}(n_0)\cConfig$ | position of mass point which is provided by node $n_0$ in any configuration |
    | node displacement | $\LU{0}{\uv}\cConfig = \LU{0}{\pRef}\cConfig = [q_0,\;q_1,\;q_2]\cConfig\tp = \LU{0}{\uv}(n_0)\cConfig$ | displacement of mass point which is provided by node $n_0$ in any configuration |
    | node velocity | $\LU{0}{\vv}\cConfig = [\dot q_0,\;\dot q_1,\;\dot q_2]\cConfig\tp = \LU{0}{\vv}(n_0)\cConfig$ | velocity of mass point which is provided by node $n_0$ in any configuration |
    | transformation matrix | $\LU{0b}{\Rot} = \ImThree$ | transformation of local body ($b$) coordinates to global (0) coordinates; this is the constant unit matrix, because local = global coordinates for the mass point |
    | residual forces | $\LU{0}{\fv} = [f_0,\;f_1,\;f_2]\tp$ | residual of all forces on mass point |
    | applied forces | $\LU{0}{\fv}_a = [f_0,\;f_1,\;f_2]\tp$ | applied forces (loads, connectors, joint reaction forces, ...) |


    #### Equations of motion



    $$
    \mr{m}{0}{0} {0}{m}{0} {0}{0}{m} \vr{\ddot q_0}{\ddot q_1}{\ddot q_2} = \vr{f_0}{f_1}{f_2}.
    $$

    For example, a LoadCoordinate on coordinate 1 of the node would add a term in $f_1$ on the RHS.
    
    Position-based markers can measure position $\pv\cConfig$. The {\bf position jacobian}  


    $$
    \Jm_{pos} = \partial \pv\cCur / \partial \cv\cCur = \mr{1}{0}{0} {0}{1}{0} {0}{0}{1}
    $$

    transforms the action of global applied forces $\LU{0}{\fv}_a$ of position-based markers on the coordinates $\cv$


    $$
    \Qm = \Jm_{pos}\tp \LU{0}{\fv}_a.
    $$

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    miniExample=r"""    node = mbs.AddNode(NodePoint(referenceCoordinates = [1,1,0], 
                                 initialCoordinates=[0.5,0,0],
                                 initialVelocities=[0.5,0,0]))
    mbs.AddObject(MassPoint(nodeNumber = node, physicsMass=1))

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic()

    #check result
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[0]
    #final x-coordinate of position shall be 2
""",
    objectType=ObjectTypeBody,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}\cConfig(\pLocB) = \LU{0}{\pRef}\cConfig + \LU{0}{\pRef}\cRef + \LU{0b}{\ImThree}\pLocB$global position vector of translated local position; local (body) coordinate system = global coordinate system"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv}\cConfig = [q_0,\;q_1,\;q_2]\cConfig\tp$global displacement vector of mass point"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv}\cConfig = \LU{0}{\dot\uv}\cConfig = [\dot q_0,\;\dot q_1,\;\dot q_2]\cConfig\tp$global velocity vector of mass point"""),
        ItemOutputVariable(OVAcceleration, r"""$\LU{0}{\av}\cConfig = \LU{0}{\ddot\uv}\cConfig = [\ddot q_0,\;\ddot q_1,\;\ddot q_2]\cConfig\tp$global acceleration vector of mass point"""),
        ItemOutputVariable(OVRotationMatrix, OVDIdentityMatrixForCompleteness),
        ItemOutputVariable(OVRotation, OVDZeroVectorForCompleteness),
        ItemOutputVariable(OVAngularVelocity, OVDZeroVectorForCompleteness),
        ItemOutputVariable(OVAngularVelocityLocal, OVDZeroVectorForCompleteness),
        ],
    pythonShortName='MassPoint',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsMass',
            defaultValue=0.,
            description=r'$m$mass [SI:kg] of mass point'),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n0$node number (type NodeIndex) for mass point'),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return JacobianType::_None;'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'JacobianTtimesVector_q', 'DisplacementMassIntegral_q']),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetOutputVariableBody'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetDisplacement'),
        ItemFunctionDef('GetVelocity'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetAcceleration',
            args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
            description=r"return the (global) acceleration of 'localPosition' according to configuration type"),
        ItemFunctionDef('GetLocalCenterOfMass',
            implementation='return Vector3D({0.,0.,0.});'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "MassPoint";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetODE2Size',
            implementation='return nODE2coordinates;'),
        ItemRequestedTypes('Node', ['Position']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::SingleNoded);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TBodyGraphicsData, destination=DestVisu,
            pythonName='graphicsData',
            defaultValue=NoDefaultValue,
            description=r'Structure contains data for body visualization; data is defined in special list / dictionary structure'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectMassPoint2D   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectMassPoint2D',
    addProtectedC=r"""    static constexpr Index nODE2coordinates = 2;
""",
    cParentClass=ParentClassCObjectBody,
    classDescription=r'A 2D mass point which is attached to a position-based 2D node.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities


    | intermediate variables | symbol | description |
    |---|---|---|
    | node position | $\LU{0}{\pRef}\cConfig + \LU{0}{\pRef}\cRef = \LU{0}{\pv}(n_0)\cConfig$ | position of mass point which is provided by node $n_0$ in any configuration (except reference) |
    | node displacement | $\LU{0}{\uv}\cConfig = \LU{0}{\pRef}\cConfig = [q_0,\;q_1,\;0]\cConfig\tp = \LU{0}{\uv}(n_0)\cConfig$ | displacement of mass point which is provided by node $n_0$ in any configuration |
    | node velocity | $\LU{0}{\vv}\cConfig = [\dot q_0,\;\dot q_1,\;0]\cConfig\tp = \LU{0}{\vv}(n_0)\cConfig$ | velocity of mass point which is provided by node $n_0$ in any configuration |
    | transformation matrix | $\LU{0b}{\Rot} = \ImThree$ | transformation of local body ($b$) coordinates to global (0) coordinates; this is the constant unit matrix, because local = global coordinates for the mass point |
    | residual forces | $\LU{0}{\fv} = [f_0,\;f_1]\tp$ | residual of all forces on mass point |
    | applied forces | $\LU{0}{\fv}_a = [f_0,\;f_1,\;f_2]\tp$ | applied forces (loads, connectors, joint reaction forces, ...) |

    <!-- -->

    #### Equations of motion



    $$
    \mp{m}{0} {0}{m} \vp{\ddot q_0}{\ddot q_1} = \vp{f_0}{f_1}.
    $$

    For example, a LoadCoordinate on coordinate 1 of the node would add a term in $f_1$ on the RHS.
    
    Position-based markers can measure position $\pv\cConfig$. The {\bf position jacobian}  


    $$
    \Jm_{pos} = \partial \pv\cCur / \partial \cv\cCur = 
          \left[\!\! \begin{array}{ccc}
          1 & 0 & 0 \vspace{0.1cm}\\ 
          0 & 1 & 0 \end{array} \!\!\right]
    $$

    transforms the action of global applied forces $\LU{0}{\fv}_a$ of position-based markers on the coordinates $\cv$


    $$
    \Qm = \Jm_{pos}\tp \LU{0}{\fv}_a.
    $$

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    miniExample=r"""    node = mbs.AddNode(NodePoint2D(referenceCoordinates = [1,1], 
                                 initialCoordinates=[0.5,0],
                                 initialVelocities=[0.5,0]))
    mbs.AddObject(MassPoint2D(nodeNumber = node, physicsMass=1))

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic()

    #check result
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[0]
    #final x-coordinate of position shall be 2
""",
    objectType=ObjectTypeBody,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}\cConfig(\pLocB) = \LU{0}{\pRef}\cConfig + \LU{0}{\pRef}\cRef + \LU{0b}{\ImTwo}\pLocB$global position vector of translated local position; local (body) coordinate system = global coordinate system"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv}\cConfig = [q_0,\;q_1,\;0]\cConfig\tp$global displacement vector of mass point"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv}\cConfig = \LU{0}{\dot\uv}\cConfig = [\dot q_0,\;\dot q_1,\;0]\cConfig\tp$global velocity vector of mass point"""),
        ItemOutputVariable(OVAcceleration, r"""$\LU{0}{\av}\cConfig = \LU{0}{\ddot\uv}\cConfig = [\ddot q_0,\;\ddot q_1,\;0]\cConfig\tp$global acceleration vector of mass point"""),
        ItemOutputVariable(OVRotationMatrix, OVDIdentityMatrixForCompleteness),
        ItemOutputVariable(OVRotation, OVDZeroVectorForCompleteness),
        ItemOutputVariable(OVAngularVelocity, OVDZeroVectorForCompleteness),
        ItemOutputVariable(OVAngularVelocityLocal, OVDZeroVectorForCompleteness),
        ],
    pythonShortName='MassPoint2D',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsMass',
            defaultValue=0.,
            description=r'$m$mass [SI:kg] of mass point'),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n0$node number (type NodeIndex) for mass point'),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return JacobianType::_None;'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'JacobianTtimesVector_q', 'DisplacementMassIntegral_q']),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetOutputVariableBody'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetDisplacement'),
        ItemFunctionDef('GetVelocity'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetAcceleration',
            args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
            description=r"return the (global) acceleration of 'localPosition' according to configuration type"),
        ItemFunctionDef('GetLocalCenterOfMass',
            implementation='return Vector3D({0.,0.,0.});'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "MassPoint2D";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetODE2Size',
            implementation='return nODE2coordinates;'),
        ItemRequestedTypes('Node', ['Position2D']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::SingleNoded);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TBodyGraphicsData, destination=DestVisu,
            pythonName='graphicsData',
            defaultValue=NoDefaultValue,
            description=r'Structure contains data for body visualization; data is defined in special list / dictionary structure'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectMass1D   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectMass1D',
    cParentClass=ParentClassCObjectBody,
    classDescription=r'A 1D (translational) mass which is attached to Node1D. Note, that the mass does not need to have the interpretation as a translational mass.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities


    | intermediate variables | symbol | description |
    |---|---|---|
    | position coordinate | ${p_0}\cConfig = {c_0}\cConfig + {c_0}\cRef$ | position coordinate of node (nodal coordinate $c_0$) in any configuration |
    | displacement coordinate | ${u_0}\cConfig = {c_0}\cConfig$ | displacement coordinate of mass node in any configuration |
    | velocity coordinate | ${u_0}\cConfig$ | velocity coordinate of mass node in any configuration |
    | Position | $\LU{0}{\pv}\cConfig =\LU{0}{\pRef_0} + \LU{0b}{\Rot_{0}} \LU{b}{\vr{p_0}{0}{0}}\cConfig$ | (translational) position of mass object in any configuration |
    | Displacement | $\LU{0}{\uv}\cConfig = \LU{0b}{\Rot_{0}} \LU{b}{\vr{q_0}{0}{0}}\cConfig$ | (translational) displacement of mass object in any configuration |
    | Velocity | $\LU{0}{\vv}\cConfig = \LU{0b}{\Rot_{0}} \LU{b}{\vr{\dot q_0}{0}{0}}\cConfig$ | (translational) velocity of mass object in any configuration |
    | residual force | $f$ | residual of all forces on mass object |
    | applied force | $\LU{0}{\fv}_a = [f_0,\;f_1,\;f_2]\tp$ | 3D applied force (loads, connectors, joint reaction forces, ...) |
    | applied torque | $\LU{0}{\ttau}_a = [\tau_0,\;\tau_1,\;\tau_2]\tp$ | 3D applied torque (loads, connectors, joint reaction forces, ...) |

    <!-- -->
    A rigid body marker (e.g., MarkerBodyRigid) may be attached to this object and forces/torques can be applied. 
    However, torques will have no effect and forces will only have effect in 'direction' of the coordinate.

    #### Equations of motion



    $$
    m \cdot \ddot q_0 = f.
    $$

    Note that $f$ is computed from all connectors and loads upon the object. E.g., a 3D force vector $\LU{0}{\fv}_a$ is 
    transformed to $f$ as


    $$
    f = \LU{b}{[1,\,0,\,0]} \LU{b0}{\Rot_{0}} \LU{0}{\fv}_a
    $$

    Thus, the {\bf position jacobian} reads 


    $$
    \Jm_{pos} = \partial \pv\cCur / \partial {q_0}\cCur = 
           \LU{b}{[1,\,0,\,0]} \LU{b0}{\Rot_{0}}
    $$

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    miniExample=r"""    node = mbs.AddNode(Node1D(referenceCoordinates = [1], 
                              initialCoordinates=[0.5],
                              initialVelocities=[0.5]))
    mass = mbs.AddObject(Mass1D(nodeNumber = node, physicsMass=1))

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic()

    #check result, get current mass position at local position [0,0,0]
    exu.sys['testResult'] = mbs.GetObjectOutputBody(mass, exu.OutputVariableType.Position, [0,0,0])[0]
    #final x-coordinate of position shall be 2
""",
    objectType=ObjectTypeBody,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}\cConfig$global position vector; for interpretation see intermediate variables"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv}\cConfig$global displacement vector; for interpretation see intermediate variables"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv}\cConfig $global velocity vector; for interpretation see intermediate variables"""),
        ItemOutputVariable(OVRotationMatrix, r"""$\LU{0b}{\Rot}$vector with 9 components of the rotation matrix (row-major format)"""),
        ItemOutputVariable(OVRotation, r"""vector with 3 components of the Euler/Tait-Bryan angles in xyz-sequence ($\LU{0b}{\Rot}\cConfig=:\Rot_0(\varphi_0) \cdot \Rot_1(\varphi_1) \cdot \Rot_2(\varphi_2)$), recomputed from rotation matrix $\LU{0b}{\Rot}$"""),
        ItemOutputVariable(OVAngularVelocity, OVDAngularVelocityBody),
        ItemOutputVariable(OVAngularVelocityLocal, OVDAngularVelocityLocalBody),
        ],
    pythonShortName='Mass1D',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsMass',
            defaultValue=0.,
            description=r'$m$mass [SI:kg] of mass'),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n0$node number (type NodeIndex) for Node1D'),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='referencePosition',
            defaultValue=DVZeroVector3D,
            description=r"""$\LU{0}{\pRef_0}$a reference position, used to transform the 1D coordinate to a position"""),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp+DestParam,
            pythonName='referenceRotation',
            defaultValue='EXUmath::unitMatrix3D',
            description=r"""$\LU{0b}{\Rot_{0}} \in \Rcal^{3 \times 3}$the constant body rotation matrix, which transforms body-fixed (b) to global (0) coordinates"""),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return JacobianType::_None;'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'AngularVelocity_qt', 'JacobianTtimesVector_q', 'DisplacementMassIntegral_q']),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetOutputVariableBody'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetDisplacement'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetRotationMatrix',
            implementation='return parameters.referenceRotation;',
            description='return configuration dependent rotation matrix of node; returns always a 3D Matrix, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunctionDef('GetAngularVelocity',
            implementation='return Vector3D({ 0.,0.,0. });'),
        ItemFunctionDef('GetAngularVelocityLocal',
            implementation='return Vector3D({ 0.,0.,0. });',
            description='return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunctionDef('GetLocalCenterOfMass',
            implementation='return Vector3D({0.,0.,0.});'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "Mass1D";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetODE2Size',
            implementation='return 1;'),
        ItemRequestedTypes('Node', ['GenericODE2']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::SingleNoded);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TBodyGraphicsData, destination=DestVisu,
            pythonName='graphicsData',
            defaultValue=NoDefaultValue,
            description=r'Structure contains data for body visualization; data is defined in special list / dictionary structure'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectRotationalMass1D   ++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectRotationalMass1D',
    cParentClass=ParentClassCObjectBody,
    classDescription=r'A 1D rotational inertia (mass) which is attached to Node1D.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities


    | intermediate variables | symbol | description |
    |---|---|---|
    | position coordinate | ${\theta_0}\cConfig = {c_0}\cConfig + {c_0}\cRef $ | total rotation coordinate of node (e.g., Node1D) in any configuration (nodal coordinate $c_0$) |
    | displacement coordinate | ${\psi_0}\cConfig = {c_0}\cConfig$ | change of rotation coordinate of mass node (e.g., Node1D) in any configuration (nodal coordinate $c_0$) |
    | velocity coordinate | ${\dot \psi_{0\cConfig}}$ | rotation velocity coordinate of mass node (e.g., Node1D) in any configuration |
    | Position | $\LU{0}{\pv}\cConfig =\LU{0}{\pRef_0}$ | constant (translational) position of mass object in any configuration |
    | Displacement | $\LU{0}{\uv}\cConfig = [0,0,0]\tp$ | (translational) displacement of mass object in any configuration |
    | Velocity | $\LU{0}{\vv}\cConfig = [0,0,0]\tp$ | (translational) velocity of mass object in any configuration |
    | AngularVelocity | $\LU{0}{\tomega}\cConfig = \LU{0i}{\Rot_{0}} \LU{i}{\vr{0}{0}{\dot \psi_0}}\tp$ |  |
    | AngularVelocityLocal | $\LU{b}{\tomega}\cConfig = \LU{i}{\vr{0}{0}{\dot \psi_0}}\tp$ |  |
    | RotationMatrix | $\LU{0b}{\Rot} = \LU{0i}{\Rot_{0}} \LU{ib}{\mr{\cos(\theta_0)}{-\sin(\theta_0)}{0} {\sin(\theta_0)}{\cos(\theta_0)}{0} {0}{0}{1}}$ | transformation of local body ($b$) coordinates to global (0) coordinates |
    | residual force | $\tau$ | residual of all forces on mass object |
    | applied force | $\LU{0}{\fv}_a = [f_0,\;f_1,\;f_2]\tp$ | 3D applied force (loads, connectors, joint reaction forces, ...) |
    | applied torque | $\LU{0}{\ttau}_a = [\tau_0,\;\tau_1,\;\tau_2]\tp$ | 3D applied torque (loads, connectors, joint reaction forces, ...) |

    <!-- -->
    A rigid body marker (e.g., MarkerBodyRigid) may be attached to this object and forces/torques can be applied. 
    However, forces will have no effect and torques will only have effect in 'direction' of the coordinate.

    #### Equations of motion



    $$
    J \cdot \ddot \psi_0 = \tau.
    $$

    Note that $\tau$ is computed from all connectors and loads upon the object. E.g., a 3D torque vector $\LU{0}{\ttau}_a$ is 
    transformed to $\tau$ as


    $$
    \tau = \LU{b}{[0,\,0,\,1]}\LU{b0}{\Rot_{0}} \LU{0}{\ttau}_a
    $$

    Thus, the {\bf rotation jacobian} reads 


    $$
    \Jm_{rot} = \partial \tomega\cCur / \partial \dot q_{0,cur} = 
           \LU{b}{[0,\,0,\,1]} \LU{b0}{\Rot_{0}}
    $$

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    miniExample=r"""    node = mbs.AddNode(Node1D(referenceCoordinates = [1], #\psi_0ref
                              initialCoordinates=[0.5],   #\psi_0ini
                              initialVelocities=[0.5]))   #\psi_t0ini
    rotor = mbs.AddObject(Rotor1D(nodeNumber = node, physicsInertia=1))

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic()

    #check result, get current rotor z-rotation at local position [0,0,0]
    exu.sys['testResult'] = mbs.GetObjectOutputBody(rotor, exu.OutputVariableType.Rotation, [0,0,0])
    #final z-angle of rotor shall be 2
""",
    objectType=ObjectTypeBody,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}\cConfig= \pRefG$global position vector; for interpretation see intermediate variables"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv}\cConfig$global displacement vector; for interpretation see intermediate variables"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv}\cConfig $global velocity vector; for interpretation see intermediate variables"""),
        ItemOutputVariable(OVRotationMatrix, r"""$\LU{0b}{\Rot}$vector with 9 components of the rotation matrix (row-major format)"""),
        ItemOutputVariable(OVRotation, r'$\theta$scalar rotation angle obtained from underlying node'),
        ItemOutputVariable(OVAngularVelocity, OVDAngularVelocityBody),
        ItemOutputVariable(OVAngularVelocityLocal, OVDAngularVelocityLocalBody),
        ],
    pythonShortName='Rotor1D',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsInertia',
            defaultValue=0.,
            description=r'$J$inertia components [SI:kgm$^2$] of rotor / rotational mass'),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r"""$n0$node number (type NodeIndex) of Node1D, providing rotation coordinate $\psi_0 = c_0$"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='referencePosition',
            defaultValue=DVZeroVector3D,
            description=r"""$\LU{0}{\pRef_0}$a constant reference position = reference point, used to assign joint constraints accordingly and for drawing"""),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp+DestParam,
            pythonName='referenceRotation',
            defaultValue='EXUmath::unitMatrix3D',
            description=r"""$\LU{0i}{\Rot_{0}} \in \Rcal^{3 \times 3}$an intermediate rotation matrix, which transforms the 1D coordinate into 3D, see description"""),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return JacobianType::_None;'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'AngularVelocity_qt', 'JacobianTtimesVector_q']),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetOutputVariableBody'),
        ItemFunction(type=TReal, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetRotationAngle',
            args='ConfigurationType configuration = ConfigurationType::Current',
            description=r'return the rotation angle (reference+current) according to configuration type'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetDisplacement'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetRotationMatrix',
            description='return configuration dependent rotation matrix of node; returns always a 3D Matrix, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetAngularVelocityLocal',
            description='return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunctionDef('GetLocalCenterOfMass',
            implementation='return Vector3D({0.,0.,0.});'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "RotationalMass1D";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetODE2Size',
            implementation='return 1;'),
        ItemRequestedTypes('Node', ['GenericODE2']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::SingleNoded);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TBodyGraphicsData, destination=DestVisu,
            pythonName='graphicsData',
            defaultValue=NoDefaultValue,
            description=r'Structure contains data for body visualization; data is defined in special list / dictionary structure'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectRigidBody   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectRigidBody',
    addProtectedC=r"""    static constexpr Index nDim3D = 3; //used to avoid pure 3 in code where dimensionality applies
    static constexpr Index nDisplacementCoordinates = 3; //code currently implemented for 3 displacemnet coordinates; this constant used to change this in future implementation
""",
    cParentClass=ParentClassCObjectBody,
    classDescription=r"""A 3D rigid body which is attached to a 3D rigid body node. The rotation parametrization of the rigid body follows the rotation parametrization of the node. Use Euler parameters in the general case (no singularities) in combination with implicit solvers (GeneralizedAlpha or TrapezoidalIndex2), Tait-Bryan angles for special cases, e.g., rotors where no singularities occur if you rotate about $x$ or $z$ axis, or use Lie-group formulation with rotation vector together with explicit solvers. REMARK: Use the class \texttt{RigidBodyInertia}, see [](#sec-rigidbodyutilities-rigidbodyinertia---init--) and \texttt{CreateRigidBody(...)}, see [](#sec-mainsystemextensions-createrigidbody), of \texttt{exudyn.rigidBodyUtilities} to handle inertia, ABRV:COM and mass. \addExampleImage{ObjectRigidBody}""",
    classType=ClassTypeObject,
    equations=r"""    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->

    #### Definition of quantities

    <!--
    \rowTable{(generalized) coordinates}{$\cv\cConfig = [q_0,q_1,\;\psi_0]\tp$}{generalized coordinates of body (= coordinates of node)}
    \rowTable{generalized forces}{$\LU{0}{\fv} = [f_0,\;f_1,\;\tau_2]\tp$}{generalized forces applied to body}
    -->

    | intermediate variables | symbol | description |
    |---|---|---|
    | inertia tensor | $\LU{b}{\Jm} = \LU{b}{\mr{J_{xx}}{J_{xy}}{J_{xz}} {J_{xy}}{J_{yy}}{J_{yz}} {J_{xz}}{J_{yz}}{J_{zz}}}$ | symmetric inertia tensor, based on components of $\LU{b}{\jv_6}$, in body-fixed (local) coordinates and w.r.t.\ body's reference point |
    | reference coordinates | $\qv\cRef = [\pRef\tp\cRef,\,\tpsi\tp\cRef]\tp$ | defines reference configuration, {\bf DIFFERENT} meaning from body's reference point! |
    | (relative) current coordinates | $\qv\cCur = [\pRef\tp\cCur,\,\tpsi\tp\cCur]\tp$ | unknowns in solver; {\bf relative} to the reference coordinates; current coordinates at initial configuration = initial coordinates $\qv\cIni$ |
    | current velocity coordinates | $\dot \qv\cCur = [\vv\tp\cCur,\,\dot \tpsi\tp\cCur]\tp = [\dot \pv\tp\cCur,\,\dot \ttheta\tp\cCur]\tp$ | current velocity coordinates |
    | body's reference point | $\pRefG\cConfig + \pRefG\cRef = \LU{0}{\pv}(n_0)\cConfig$ | position of {\bf body's reference point} provided by node $n_0$ in any configuration except for reference; if $\LU{b}{\bv_{COM}}==[0,\;0,\;0]\tp$, this position becomes equal to the ABRV:COM position |
    | reference body's reference point | $\pRefG\cRef = \LU{0}{\pv}(n_0)\cRef$ | position of {\bf body's reference point} in reference configuration |
    | body's reference point displacement | $\LU{0}{\uv}\cConfig = \pRefG\cConfig = [q_0,\;q_1,\;q_2]\cConfig\tp = \LU{0}{\uv}(n_0)\cConfig$ | displacement of {\bf body's reference point} which is provided by node $n_0$ in any configuration |
    | body's reference point velocity | $\LU{0}{\vv}\cConfig = \dot \pRefG\cConfig = [\dot q_0,\;\dot q_1,\;\dot q_2]\cConfig\tp = \LU{0}{\vv}(n_0)\cConfig$ | velocity of {\bf body's reference point} which is provided by node $n_0$ in any configuration |
    | body's reference point acceleration | $\LU{0}{\av}\cConfig = [\ddot q_0,\;\ddot q_1,\;\ddot q_2]\cConfig\tp$ | acceleration of {\bf body's reference point} which is provided by node $n_0$ in any configuration |
    | rotation coordinates | $\ttheta_{\mathrm{config}} = \tpsi(n_0)\cRef + \tpsi(n_0)\cConfig$ | (total) rotation parameters of body as provided by node $n_0$ in any configuration |
    | rotation parameters | $\ttheta_{\mathrm{config}} = \tpsi(n_0)\cRef + \tpsi(n_0)\cConfig$ | (total) rotation parameters of body as provided by node $n_0$ in any configuration |
    | body rotation matrix | $\LU{0b}{\Rot}\cConfig = \LU{0b}{\Rot}(n_0)\cConfig$ | rotation matrix which transforms local to global coordinates as given by node |
    | local position | $\pLocB = [\LU{b}{b_0},\,\LU{b}{b_1},\,\LU{b}{b_2}]\tp$ | local position as used by markers or sensors |
    | angular velocity | $\LU{0}{\tomega}\cConfig = \LU{0}{[\omega_0(n_0),\,\omega_1(n_0),\,\omega_2(n_0)]}\cConfig\tp$ | global angular velocity of body as provided by node $n_0$ in any configuration |
    | local angular velocity | $\LU{b}{\tomega}\cConfig$ | local angular velocity of body as provided by node $n_0$ in any configuration |
    | body angular acceleration | $\LU{0}{\talpha}\cConfig = \LU{0}{\dot \tomega}\cConfig$ | angular acceleratoin of body as provided by node $n_0$ in any configuration |
    | applied forces | $\LU{0}{\fv}_a = [f_0,\;f_1,\;f_2]\tp$ | calculated from loads, connectors, ... |
    | applied torques | $\LU{0}{\ttau}_a = [\tau_0,\;\tau_1,\;\tau_2]\tp$ | calculated from loads, connectors, ... |
    | constraint reaction forces | $\LU{0}{\fv}_\lambda = [f_{\lambda 0},\;f_{\lambda 1},\;f_{\lambda 2}]\tp$ | calculated from joints or constraint) |
    | constraint reaction torques | $\LU{0}{\ttau}_\lambda = [\tau_{\lambda 0},\;\tau_{\lambda 1},\;\tau_{\lambda 2}]\tp$ | calculated from joints or constraints |

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->

    #### Rotation parametrization

    The equations of motion of the rigid body build upon a specific parameterization of the rigid body coordinates.
    Rigid body coordinates are defined by the underlying node given by \texttt{nodeNumber} $n0$.
    Appropriate nodes are 
    \bi
      \item \texttt{NodeRigidBodyEP} (Euler parameters)
      \item \texttt{NodeRigidBodyRxyz} (Euler angles / Tait Bryan angles)
      \item \texttt{NodeRigidBodyRotVecLG} (Rotation vector with Lie group integration option)
    \ei
    Note that all operations for rotation parameters, such as the computation of the rotation matrix, must be performed with the 
    rotation parameters $\ttheta$, see table above, which are the sum of reference and current coordinates.
    
    The angular velocity in body-fixed coordinates is related to the rotation parameters by means of a matrix $\LU{b}{\Gm_{rp}}$,

    $$
    \LU{b}{\tomega} = \LU{b}{\Gm_{rp}} \dot \ttheta = \LU{b}{\Gm_{rp}} \dot \tpsi \, ,
    $$ (eq-objectrigidbody-omegalocal)

    and is specific for any rotation parametrization $rp$.
    The angular velocity in global coordinates is related to the rotation parameters by means of a matrix $\LU{0}{\Gm_{rp}}$,

    $$
    \LU{0}{\tomega} = \LU{0}{\Gm_{rp}} \dot \ttheta\, .
    $$ (eq-objectrigidbody-omega)

    The local angular accelerations follow as

    $$
    \LU{b}{\talpha} = \LU{b}{\dot \tomega}= \LU{b}{\Gm_{rp}} \ddot \ttheta + \LU{b}{\dot \Gm_{rp}} \dot \ttheta \, ,
    $$ (eq-objectrigidbody-alpha)

    remember that derivatives for angular velocities can also be done in the local frame. In case of Euler parameters and the Lie-group rotation vector we find that
    $\LU{b}{\dot \Gm_{rp}} \dot \ttheta = \Null$.
    
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->

    #### Equations of motion for ABRV:COM

    The equations of motion for a rigid body, the so-called Newton-Euler equations, can be written for the special case of the reference point $=$ ABRV:COM and split for translations and rotations, using a coordinate-free notation,

    $$
    \mp{m \ImThree}{\Null}{\Null}{\Jm} \vp{\av_{COM}}{\talpha} = \vp{\Null}{-\tilde \tomega \Jm \tomega} + \vp{\fv_a}{\ttau_a} + \vp{\fv_\lambda}{\ttau_\lambda}
    $$ (eq-objectrigidbody-eomcom0)

    with the $3\times 3$ unit matrix $\ImThree$ and forces $\fv$ resp.\ torques $\ttau$ as discribed in the table above.
    A change of the reference point, using the vector $\bv_{COM}$ from the body's reference point $\pv$ to the ABRV:COM position, is simple by replacing ABRV:COM accelerations using the common relation known from Euler

    $$
    \av_{COM} =  \av + \tilde \talpha \bv_{COM} + \tilde \tomega \tilde \tomega \bv_{COM} \, ,
    $$

    which is inserted into the first line of {eq}`eq-objectrigidbody-eomcom0`. Additionally, the second line of {eq}`eq-objectrigidbody-eomcom0`
    (second Euler equation related to rate of angular momentum) is rewritten for an arbitrary reference point, $\bv_{COM}$ denoting the vector from the body reference point to ABRV:COM, using the well known relation

    $$
    m \tilde \bv_{COM} \talpha +  \Jm \talpha + \tilde \tomega \Jm \tomega = \ttau_a + \ttau_\lambda
    $$

    
    #### Equations of motion for arbitrary reference point

    This immediately leads to the equations of motion for the rigid body with respect to an arbitrary reference point ($\neq$ ABRV:COM), 
    see e.g.\ [CITE:woernle2016](page 258ff.), which have the general coordinate-free form

    $$
    \mp{m \ImThree}{-m \tilde \bv_{COM}}{m \tilde \bv_{COM}}{\Jm} \vp{\av}{\talpha} = 
          \vp{-m \tilde \tomega \tilde \tomega \bv_{COM} }{-\tilde \tomega \Jm \tomega} + \vp{\fv_a}{\ttau_a} + \vp{\fv_\lambda}{\ttau_\lambda} \, ,
    $$ (eq-objectrigidbody-eomarbitrary)

    in which $\Jm$ is the inertia tensor w.r.t.\ the chosen reference point (which has local coordinates $\LU{b}{[0,0,0]\tp}$).
    {eq}`eq-objectrigidbody-eomarbitrary` can be written in the global frame (0),

    $$
    \mp{m \ImThree}{-m \LU{0}{\tilde \bv_{COM}}} {m \LU{0}{\tilde \bv_{COM}}}{\LU{0}{\Jm}} \vp{\LU{0}{\av}}{\LU{0}{\talpha}} = 
          \vp{-m \LU{0}{\tilde \tomega} \LU{0}{\tilde \tomega} \LU{0}{\bv_{COM}} }
          {-\LU{0}{\tilde \tomega} \LU{0}{\Jm} \LU{0}{\tomega}} + \vp{\LU{0}{\fv_a}}{\LU{0}{\ttau_a}} + \vp{\LU{0}{\fv_\lambda}}{\LU{0}{\ttau_\lambda}} \, .
    $$ (eq-objectrigidbody-eomglobal)

    Expressing the translational part (first line) of {eq}`eq-objectrigidbody-eomglobal` in the global frame (0), using local coordinates (b) for 
    quantities that are constant in the body-fixed frame, $\LU{b}{\Jm}$ and $\LU{b}{\bv_{COM}}$, thus expressing also the 
    angular velocity $\LU{b}{\tomega}$ in the body-fixed frame,
    applying {eq}`eq-objectrigidbody-omegalocal` and {eq}`eq-objectrigidbody-alpha`, and using the relations

    $$
    \begin{aligned}
    \LU{0}{\tilde \tomega}  \LU{0}{\tilde \tomega} \LU{0}{\bv_{COM}}
          &= \LU{0b}{\Rot} \LU{b}{\tilde \tomega} \LU{b}{\tilde \tomega} \LU{b}{\bv_{COM}} = - \LU{0b}{\Rot} \LU{b}{\tilde \tomega} \LU{b}{\tilde \bv_{COM}} \LU{b}{\tomega} 
          = -\LU{0b}{\Rot} \LU{b}{\tilde \tomega} \LU{b}{\tilde \bv_{COM}} \LU{b}{\Gm_{rp}} \dot \ttheta \, , \\
        %
          -m \LU{0}{\tilde \bv_{COM}} \LU{0}{\tilde \talpha} 
          &= -m \LU{0b}{\Rot} \LU{b}{\tilde \bv_{COM}} \LU{b}{\tilde \talpha}
          = -m \LU{0b}{\Rot} \LU{b}{\tilde \bv_{COM}} \left( \LU{b}{\Gm_{rp}} \ddot \ttheta + \LU{b}{\dot \Gm_{rp}} \dot \ttheta \right) \, ,
    \end{aligned}
    $$

    we obtain

    $$
    \begin{aligned}
    &&\mp{m \ImThree}  {-m \LU{0b}{\Rot} \LU{b}{\tilde \bv_{COM}}\LU{b}{\Gm_{rp}}}  {m \LU{b}{\Gm_{rp}\tp} \LU{b}{\tilde \bv_{COM}}\LU{0b}{\Rot\tp}}  {\LU{b}{\Gm_{rp}\tp}\LU{b}{\Jm}\LU{b}{\Gm_{rp}}} 
              \vp{\LU{0}{\av}}{\ddot \ttheta} \\
            &&= \vp{m \LU{0b}{\Rot} \LU{b}{\tilde \tomega} \LU{b}{\tilde \bv_{COM}} \LU{b}{\tomega}  + m \LU{0b}{\Rot} \LU{b}{\tilde \bv_{COM}}\LU{b}{\dot \Gm_{rp}} \dot \ttheta}  
                 {-\LU{b}{\Gm_{rp}\tp}\LU{b}{\tilde \tomega} \LU{b}{\Jm} \LU{b}{\tomega} - \LU{b}{\Gm_{rp}\tp} \LU{b}{\Jm} \LU{b}{\dot \Gm_{rp}} \dot \ttheta} + 
              \vp{\LU{0}{\fv}_a}{\LU{0}{\Gm_{rp}\tp}\LU{0}{\ttau}_a} + \vp{\LU{0}{\fv}_\lambda}{\fv_{\theta,\lambda}}
    \end{aligned}
    $$ (eq-objectrigidbody-eom)

    with constraint reaction forces $\fv_{\theta,\lambda}$ for the rotation parameters. 
    Note that <!--$ \LU{b}{\tilde \tomega}\LU{b}{\bv_{COM}} = -\LU{b}{\tilde \bv_{COM}} \LU{b}{\tomega}$ has been used, -->
    the last line has been pre-multiplied with $\LU{b}{\Gm_{rp}\tp}$ (in order to make the mass matrix symmetric) and that
    $\LU{b}{\dot \Gm_{rp}} \dot \ttheta = \Null$ in case of Euler parameters and the Lie-group rotation vector .
    
    #### Euler parameters

    In case of Euler parameters, a constraint equation is automatically added, reading for the index 3 case

    $$
    g_\theta(\ttheta) = \theta_0^2 + \theta_1^2 + \theta_2^2 + \theta_3^2 - 1 = 0
    $$ (eq-objectrigidbody-eulerparameters)

    and for the index 2 case

    $$
    \dot g_\theta(\ttheta) = 2 \theta_0 \dot \theta_0 + 2 \theta_1 \dot \theta_1 + 2 \theta_2 \dot \theta_2 + 2 \theta_3 \dot \theta_3 = 0
    $$ (eq-objectrigidbody-eulerparametersvel)

    Given a Lagrange parameter (algebraic variable) $\lambda_\theta$ related to the Euler parameter constraint {eq}`eq-objectrigidbody-eulerparameters`, the constraint reaction forces in {eq}`eq-objectrigidbody-eom` then read

    $$
    \fv_{\theta,\lambda} = \frac{\partial g_\theta}{\ttheta\tp} \lambda_\theta = [2\theta_0,\; 2\theta_1,\; 2\theta_2,\; 2\theta_3]\tp
    $$

    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    **Userfunction**: `graphicsDataUserFunction(mbs, itemNumber)`
    A user function, which is called by the visualization thread in order to draw user-defined objects.
    The function can be used to generate any \texttt{BodyGraphicsData}, see Section [](#sec-graphicsdata).
    Use \texttt{exudyn.graphics} functions, see Section [](#sec-module-graphics), to create more complicated objects. 
    Note that \texttt{graphicsDataUserFunction} needs to copy lots of data and is therefore
    inefficient and only designed to enable simpler tests, but not large scale problems.
    
    For an example for \texttt{graphicsDataUserFunction} see ObjectGround, [](#sec-item-objectground).
    <!-- -->

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides reference to mbs, which can be used in user function to access all data of the object |
    | \texttt{itemNumber} | Index | integer number of the object in mbs, allowing easy access |
    | **return value** | BodyGraphicsData | list of \texttt{GraphicsData} dictionaries, see Section [](#sec-graphicsdata) |

    
    For creating a \texttt{ObjectRigidBody}, there is a \texttt{rigidBodyUtilities} function \texttt{CreateRigidBody}, 
    see [](#sec-mainsystemextensions-createrigidbody), which simplifies the setup of a rigid body significantely!
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    objectType=ObjectTypeBody,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}\cConfig(\pLocB) = \LU{0}{\pRef}\cConfig + \LU{0}{\pRef}\cRef + \LU{0b}{\Rot}\pLocB$global position vector of body-fixed point given by local position vector $\pLocB$"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv}\cConfig + \LU{0b}{\Rot}\pLocB$global displacement vector of body-fixed point given by local position vector $\pLocB$"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv}\cConfig(\pLocB) = \LU{0}{\dot\uv}\cConfig + \LU{0b}{\Rot}(\LU{b}{\tomega} \times \pLocB\cConfig)$global velocity vector of body-fixed point given by local position vector $\pLocB$"""),
        ItemOutputVariable(OVVelocityLocal, r"""$\LU{b}{\vv}\cConfig(\pLocB) = \LU{b0}{\Rot} \LU{0}{\vv}\cConfig(\pLocB)$local (body-fixed) velocity vector of body-fixed point given by local position vector $\pLocB$"""),
        ItemOutputVariable(OVRotationMatrix, r"""$\mathrm{vec}(\LU{0b}{\Rot})=[A_{00},\,A_{01},\,A_{02},\,A_{10},\,\ldots,\,A_{21},\,A_{22}]\cConfig\tp$vector with 9 components of the rotation matrix (row-major format)"""),
        ItemOutputVariable(OVRotation, 'vector with 3 components of the Euler angles in xyz-sequence (R=Rx*Ry*Rz), recomputed from rotation matrix'),
        ItemOutputVariable(OVAngularVelocity, OVDAngularVelocityBody),
        ItemOutputVariable(OVAngularVelocityLocal, OVDAngularVelocityLocalBody),
        ItemOutputVariable(OVAcceleration, r"""$\LU{0}{\av}\cConfig(\pLocB) = \LU{0}{\ddot\uv} + \LU{0}{\talpha} \times (\LU{0b}{\Rot} \pLocB) +  \LU{0}{\tomega} \times ( \LU{0}{\tomega} \times(\LU{0b}{\Rot} \pLocB))$global acceleration vector of body-fixed point given by local position vector $\pLocB$"""),
        ItemOutputVariable(OVAccelerationLocal, r"""$\LU{b}{\av}\cConfig(\pLocB) = \LU{b0}{\Rot} \LU{0}{\av}\cConfig(\pLocB)$local (body-fixed) acceleration vector of body-fixed point given by local position vector $\pLocB$"""),
        ItemOutputVariable(OVAngularAcceleration, r'$\LU{0}{\talpha}\cConfig$angular acceleration vector of body'),
        ItemOutputVariable(OVAngularAccelerationLocal, r"""$\LU{b}{\talpha}\cConfig = \LU{b0}{\Rot} \LU{0}{\talpha}\cConfig$local angular acceleration vector of body"""),
        ],
    pythonShortName='RigidBody',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsMass',
            defaultValue=0.,
            description=r'$m$mass [SI:kg] of rigid body'),
        ItemParameter(type=TVectorND(6), destination=DestComp+DestParam,
            pythonName='physicsInertia',
            defaultValue='Vector6D({0.,0.,0., 0.,0.,0.})',
            description=r"""$\LU{b}{\jv_6}$inertia components [SI:kgm$^2$]: $[J_{xx}, J_{yy}, J_{zz}, J_{yz}, J_{xz}, J_{xy}]$ in body-fixed coordinate system and w.r.t. to the reference point of the body, NOT necessarily w.r.t. to ABRV:COM; use the class RigidBodyInertia of exudynRigidBodyUtilities.py and CreateRigidBody(...) of MainSystem to handle inertia, ABRV:COM and mass"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='physicsCenterOfMass',
            defaultValue=DVZeroVector3D,
            description=r"""$\LU{b}{\bv_{COM}}$local position of ABRV:COM relative to the body's reference point; if the vector of the ABRV:COM is [0,0,0], the computation will not consider additional terms for the ABRV:COM and it is faster"""),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n0$node number (type NodeIndex) for rigid body node'),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('ComputeAlgebraicEquations'),
        ItemFunctionDef('ComputeJacobianAE'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::AE_ODE2 + JacobianType::AE_ODE2_function + JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'AngularVelocity_qt', 'JacobianTtimesVector_q', 'DisplacementMassIntegral_q']),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetOutputVariableBody'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetDisplacement'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetAcceleration',
            args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
            description=r"return the (global) acceleration of 'localPosition' according to configuration type"),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetRotationMatrix',
            description='return configuration dependent rotation matrix of node; returns always a 3D Matrix, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetAngularVelocityLocal',
            description='return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetAngularAcceleration',
            args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
            description=r"return the (global) angular acceleration of 'localPosition' according to configuration type"),
        ItemFunctionDef('GetLocalCenterOfMass',
            implementation='return parameters.physicsCenterOfMass;'),
        ItemFunctionDef('ComputeRigidBodyMarkerData'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "RigidBody";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetODE2Size',
            description=r'number of ABRV:ODE2 coordinates; depends on node'),
        ItemFunctionDef('GetAlgebraicEquationsSize',
            description=r'number of ABRV:AE coordinates; depends on node'),
        ItemRequestedTypes('Node', ['Position', 'Orientation', 'RigidBody']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::SingleNoded);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('HasConstantMassMatrix'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('CallUserFunction'),
        ItemFunctionDef('HasUserFunction',
            implementation='return graphicsDataUserFunction!=0;'),
        ItemParameter(type=TPyFunctionGraphicsData, destination=DestVisu,
            pythonName='graphicsDataUserFunction',
            defaultValue=0,
            description=r'A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics elements need to be defined in the local body coordinates and are transformed by mbs to global coordinates'),
        ItemParameter(type=TBodyGraphicsData, destination=DestVisu,
            pythonName='graphicsData',
            defaultValue=NoDefaultValue,
            description=r'Structure contains data for body visualization; data is defined in special list / dictionary structure'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectRigidBody2D   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectRigidBody2D',
    addProtectedC=r"""    static constexpr Index nODE2coordinates = 3;
""",
    cParentClass=ParentClassCObjectBody,
    classDescription=r'A 2D rigid body which is attached to a rigid body 2D node. The body obtains coordinates, position, velocity, etc. from the underlying 2D node.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    | intermediate variables | symbol | description |
    |---|---|---|
    | reference position | $\pRefG\cConfig + \pRefG\cRef = \LU{0}{\pv}(n_0)\cConfig$ | reference point, only equal to the position of ABRV:COM if $\LU{b}{\bv_{COM}}=\Null$; provided by node $n_0$ in any configuration (except reference) |
    | reference point displacement | $\LU{0}{\uv}\cConfig =\pRefG\cConfig = [q_0,\;q_1,\;0]\cConfig\tp = \LU{0}{\uv}(n_0)\cConfig$ | displacement of reference point which is provided by node $n_0$ in any configuration; NOTE that for configurations other than reference, it is follows that $\pRefG\cRef - \pRefG\cConfig$ |
    | reference point velocity | $\LU{0}{\vv}\cConfig = [\dot q_0,\;\dot q_1,\;0]\cConfig\tp = \LU{0}{\vv}(n_0)\cConfig$ | velocity of reference point which is provided by node $n_0$ in any configuration |
    | body rotation | $\LU{0}{\theta}_{0\mathrm{config}} = \theta_0(n_0)\cConfig = \psi_0(n_0)\cRef + \psi_0(n_0)\cConfig$ | rotation of body as provided by node $n_0$ in any configuration |
    | body rotation matrix | $\LU{0b}{\Rot}\cConfig = \LU{0b}{\Rot}(n_0)\cConfig$ | rotation matrix which transforms local to global coordinates as given by node |
    | local position | $\pLocB = [\LU{b}{b_0},\,\LU{b}{b_1},\,0]\tp$ | local position as used by markers or sensors |
    | body angular velocity | $\LU{0}{\tomega}\cConfig = \LU{0}{[\omega_0(n_0),\,0,\,0]}\cConfig\tp$ | rotation of body as provided by node $n_0$ in any configuration |
    | (generalized) coordinates | $\cv\cConfig = [q_0,q_1,\;\psi_0]\tp$ | generalized coordinates of body (= coordinates of node) |
    | generalized forces | $\LU{0}{\fv} = [f_0,\;f_1,\;\tau_2]\tp$ | generalized forces applied to body |
    | applied forces | $\LU{0}{\fv}_a = [f_0,\;f_1,\;0]\tp$ | applied forces (loads, connectors, joint reaction forces, ...) |
    | applied torques | $\LU{0}{\ttau}_a = [0,\;0,\;\tau_2]\tp$ | applied torques (loads, connectors, joint reaction forces, ...) |

    <!-- -->

    #### Equations of motion

    The equations of motion in case that \texttt{physicsCenterOfMass}=$\Null$ read:

    $$
    \mr{m}{0}{0} {0}{m}{0} {0}{0}{J} \vr{\ddot q_0}{\ddot q_1}{\ddot \psi_0} = \vr{f_0}{f_1}{\tau_2} = \fv.
    $$

    if \texttt{physicsCenterOfMass} is nonzero, we resort to (not that $J$ represents the moment of inertia related to the reference point!):

    $$
    \mr{m}{0}{G_x} {0}{m}{G_y} {G_x}{G_y}{J} \vr{\ddot q_0}{\ddot q_1}{\ddot \psi_0} = \vr{m \dot \psi_0^2 b_x }{m \dot \psi_0^2 b_y}{0} + \vr{f_0}{f_1}{\tau_2} = \fv.
    $$

    where we use the relations caused by the non-zero center of mass

    $$
    \vp{G_x}{G_y} = m \vp{b_y}{-b_x} \quad \mathrm{and} \quad \vp{b_x}{b_y} = \LU{0}{\bv_{COM}}
    $$

    
    Position-based markers can measure position $\pv\cConfig(\pLocB)$ depending on the local position $\pLocB$. 
    The {\bf position jacobian} depends on the local position $\pLocB$ and is defined as,

    $$
    \LU{0}{\Jm_{pos}} = \partial \LU{0}{\pv}\cConfig(\pLocB)\cCur / \partial \cv\cCur = \mr{1}{0}{-\sin(\theta)\LU{b}{b_0} - \cos(\theta)\LU{b}{b_1}} 
                                                                 {0}{1}{\cos(\theta)\LU{b}{b_0}-\sin(\theta)\LU{b}{b_1}} {0}{0}{0}
    $$

    which transforms the action of global forces $\LU{0}{\fv}$ of position-based markers on the coordinates $\cv$,

    $$
    \Qm = \LU{0}{\Jm_{pos}\tp} \LU{0}{\fv}_a
    $$

    Note that a LoadCoordinate on coordinate 2 of the node would add a torque $\tau_2$ on the RHS.
    The {\bf rotation jacobian}, which is computed from angular velocity, reads

    $$
    \LU{0}{\Jm_{rot}} = \partial \LU{0}{\tomega}\cCur / \partial \dot \cv\cCur = \mr{0}{0}{0} {0}{0}{0} {0}{0}{1}
    $$

    and transforms the action of global torques $\LU{0}{\ttau}$ of orientation-based markers on the coordinates $\cv$,

    $$
    \Qm = \LU{0}{\Jm_{rot}\tp} \, \LU{0}{\ttau}_a
    $$

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `graphicsDataUserFunction(mbs, itemNumber)`
    A user function, which is called by the visualization thread in order to draw user-defined objects.
    The function can be used to generate any \texttt{BodyGraphicsData}, see Section [](#sec-graphicsdata).
    Use \texttt{exudyn.graphics} functions, see Section [](#sec-module-graphics), to create more complicated objects. 
    Note that \texttt{graphicsDataUserFunction} needs to copy lots of data and is therefore
    inefficient and only designed to enable simpler tests, but not large scale problems.

    For an example for \texttt{graphicsDataUserFunction} see ObjectGround, [](#sec-item-objectground).
    <!-- -->

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides reference to mbs, which can be used in user function to access all data of the object |
    | \texttt{itemNumber} | int | integer number of the object in mbs, allowing easy access |
    | **return value** | BodyGraphicsData | list of \texttt{GraphicsData} dictionaries, see Section [](#sec-graphicsdata) |

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    miniExample=r"""    node = mbs.AddNode(NodeRigidBody2D(referenceCoordinates = [1,1,0.25*np.pi], 
                                       initialCoordinates=[0.5,0,0],
                                       initialVelocities=[0.5,0,0.75*np.pi]))
    mbs.AddObject(RigidBody2D(nodeNumber = node, physicsMass=1, physicsInertia=2))

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic()

    #check result
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[0]
    exu.sys['testResult']+= mbs.GetNodeOutput(node, exu.OutputVariableType.Coordinates)[2]
    #final x-coordinate of position shall be 2, angle theta shall be np.pi
""",
    objectType=ObjectTypeBody,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}\cConfig(\pLocB) = \LU{0}{\pRef}\cConfig + \LU{0}{\pRef}\cRef + \LU{0b}{\Rot}\pLocB$global position vector of body-fixed point given by local position vector $\pLocB$"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv}\cConfig + \LU{0b}{\Rot}\pLocB$global displacement vector of body-fixed point given by local position vector $\pLocB$"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv}\cConfig(\pLocB) = \LU{0}{\dot\uv}\cConfig + \LU{0b}{\Rot}(\LU{b}{\tomega} \times \pLocB\cConfig)$global velocity vector of body-fixed point given by local position vector $\pLocB$"""),
        ItemOutputVariable(OVVelocityLocal, r"""$\LU{b}{\vv}\cConfig(\pLocB) = \LU{b0}{\Rot} \LU{0}{\vv}\cConfig(\pLocB)$local (body-fixed) velocity vector of body-fixed point given by local position vector $\pLocB$"""),
        ItemOutputVariable(OVRotationMatrix, r"""$\mathrm{vec}(\LU{0b}{\Rot})=[A_{00},\,A_{01},\,A_{02},\,A_{10},\,\ldots,\,A_{21},\,A_{22}]\cConfig\tp$vector with 9 components of the rotation matrix (row-major format)"""),
        ItemOutputVariable(OVRotation, r'$\theta_{0\mathrm{config}}$scalar rotation angle of body'),
        ItemOutputVariable(OVAngularVelocity, OVDAngularVelocityBody),
        ItemOutputVariable(OVAngularVelocityLocal, OVDAngularVelocityLocalBody),
        ItemOutputVariable(OVAcceleration, r"""$\LU{0}{\av}\cConfig(\pLocB) = \LU{0}{\ddot\uv} + \LU{0}{\talpha} \times (\LU{0b}{\Rot} \pLocB) +  \LU{0}{\tomega} \times ( \LU{0}{\tomega} \times(\LU{0b}{\Rot} \pLocB))$global acceleration vector of body-fixed point given by local position vector $\pLocB$"""),
        ItemOutputVariable(OVAccelerationLocal, r"""$\LU{b}{\av}\cConfig(\pLocB) = \LU{b0}{\Rot} \LU{0}{\av}\cConfig(\pLocB)$local (body-fixed) acceleration vector of body-fixed point given by local position vector $\pLocB$"""),
        ItemOutputVariable(OVAngularAcceleration, r'$\LU{0}{\talpha}\cConfig$angular acceleration vector of body'),
        ItemOutputVariable(OVAngularAccelerationLocal, r"""$\LU{b}{\talpha}\cConfig = \LU{b0}{\Rot} \LU{0}{\talpha}\cConfig$local angular acceleration vector of body"""),
        ],
    pythonShortName='RigidBody2D',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsMass',
            defaultValue=0.,
            description=r'$m$mass [SI:kg] of rigid body'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsInertia',
            defaultValue=0.,
            description=r'$J$inertia [SI:kgm$^2$] of rigid body w.r.t. reference point; this is equal to the center of mass, if physicsCenterOfMass = 0'),
        ItemParameter(type=TVectorND(2), destination=DestComp+DestParam,
            pythonName='physicsCenterOfMass',
            defaultValue='Vector2D({0.,0.})',
            description=r"""$\LU{b}{\bv_{COM}}$local position of ABRV:COM relative to the body's reference point; if the vector of the ABRV:COM is [0,0], the computation will not consider additional terms for the ABRV:COM and it is faster"""),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n_0$node number (type NodeIndex) for 2D rigid body node'),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='if (parameters.physicsCenterOfMass == 0.) {return JacobianType::_None;} else {return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);}'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'AngularVelocity_qt', 'DisplacementMassIntegral_q', 'JacobianTtimesVector_q']),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetOutputVariableBody'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetDisplacement'),
        ItemFunctionDef('GetVelocity'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetAcceleration',
            args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
            description=r"return the (global) acceleration of 'localPosition' according to configuration type"),
        ItemFunctionDef('GetRotationMatrix',
            description='return configuration dependent rotation matrix of node; returns always a 3D Matrix, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetAngularVelocityLocal',
            implementation='return GetAngularVelocity(localPosition, configuration);',
            description='return configuration dependent local (=body-fixed) angular velocity of node, which is the same as the global angular velocity vector in 2D; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetAngularAcceleration',
            args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
            description=r"return the (global) angular acceleration of 'localPosition' according to configuration type"),
        ItemFunctionDef('GetLocalCenterOfMass',
            implementation='return Vector3D({parameters.physicsCenterOfMass[0],parameters.physicsCenterOfMass[1],0.});'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "RigidBody2D";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetODE2Size',
            implementation='return nODE2coordinates;'),
        ItemRequestedTypes('Node', ['Position2D', 'Orientation2D']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::SingleNoded);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='if (parameters.physicsCenterOfMass == 0.) {return true;} else {return false;}'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('CallUserFunction'),
        ItemFunctionDef('HasUserFunction',
            implementation='return graphicsDataUserFunction!=0;'),
        ItemParameter(type=TPyFunctionGraphicsData, destination=DestVisu,
            pythonName='graphicsDataUserFunction',
            defaultValue=0,
            description=r'A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics elements need to be defined in the local body coordinates and are transformed by mbs to global coordinates'),
        ItemParameter(type=TBodyGraphicsData, destination=DestVisu,
            pythonName='graphicsData',
            defaultValue=NoDefaultValue,
            description=r'Structure contains data for body visualization; data is defined in special list / dictionary structure'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectGenericODE2   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectGenericODE2',
    addIncludesC=r"""//#include <pybind11/numpy.h>//for NumpyMatrix
//#include <pybind11/stl.h>//for NumpyMatrix
//#include <pybind11/pybind11.h>
//typedef py::array_t<Real> NumpyMatrix; 
#include "Pymodules/PyMatrixContainer.h"//for some ABRV:FFRF matrices
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCObjectSuperElement,
    classDescription=r"""A system of $n$ second order ordinary differential equations (ABRV:ODE2), having a mass matrix, damping/gyroscopic matrix, stiffness matrix and generalized forces. It can combine generic nodes, or node points. User functions can be used to compute mass matrix and generalized forces depending on given coordinates. NOTE: all matrices, vectors, etc. must have the same dimensions $n$ or $(n \times n)$, or they must be empty $(0 \times 0)$, except for the mass matrix which always needs to have dimensions $(n \times n)$.""",
    classType=ClassTypeObject,
    equations=r"""    #### Additional output variables for superelement node access

    Functions like \texttt{GetObjectOutputSuperElement(...)}, see [](#sec-mainsystem-object), 
    or \texttt{SensorSuperElement}, see [](#sec-mainsystem-sensor), directly access special output variables
    (\texttt{OutputVariableType}) of the mesh nodes of the superelement.
    Additionally, the contour drawing of the object can make use the \texttt{OutputVariableType} of the meshnodes.

    For this object, all nodes of \texttt{ObjectGenericODE2} map their \texttt{OutputVariableType} to the meshnode $\ra$
    see at the according node for the list of \texttt{OutputVariableType}.
    <!-- -->

    #### Equations of motion

    An object with node numbers $[n_0,\,\ldots,\,n_n]$ and according numbers of nodal coordinates $[n_{c_0},\,\ldots,\,n_{c_n}]$, the total number of equations (=coordinates) of the object is

    $$
    n = \sum_{i} n_{c_i},
    $$

    which is used throughout the description of this object.
    <!-- -->

    #### Equations of motion

    The equations of motion read,

    $$
    \Mm \ddot \qv + \Dm \dot \qv + \Km \qv = \fv + \fv_{user}(mbs, t, i_N,\qv,\dot \qv)
    $$ (eq-objectgenericode2-eom)

    Note that the user function $\fv_{user}(mbs, t, i_N,\qv,\dot \qv)$ may be empty (=0), and \texttt{iN} represents the itemNumber (=objectNumber). 
    
    In case that a user mass matrix is specified, {eq}`eq-objectgenericode2-eom` is replaced with

    $$
    \Mm_{user}(mbs, t, i_N, \qv,\dot \qv) \ddot \qv + \Dm \dot \qv + \Km \qv = \fv + \fv_{user}(mbs, t, i_N, \qv,\dot \qv)
    $$

    The (internal) Jacobian $\Jm$ of {eq}`eq-objectgenericode2-eom` (assuming $\fv$ to be constant!) reads

    $$
    \Jm = f_{ODE2}   \left(\Km - \frac{\partial \fv_{user}(mbs, t, i_N,\qv,\dot \qv)}{\partial \qv}\right) + 
                f_{ODE2_t} \left(\Dm - \frac{\partial \fv_{user}(mbs, t, i_N,\qv,\dot \qv)}{\partial \dot \qv} \right) +
    $$

    Chosing $f_{ODE2} = 1$ and $f_{ODE2_t}=0$ would immediately give the jacobian of position quantities.
    
    If no \texttt{jacobianUserFunction} is specified, the jacobian is -- as with many objects in \codeName\ -- computed 
    by means of numerical differentiation.
    In case that a \texttt{jacobianUserFunction} is specified, it must represent the jacobian of the ABRV:LHS of {eq}`eq-objectgenericode2-eom` 
    without $\Km$ and $\Dm$ (these matrices are added internally),

    $$
    \Jm_{user}(mbs, t, i_N, \qv, \dot \qv, f_{ODE2}, f_{ODE2_t}) =
                -f_{ODE2}   \left(\frac{\partial \fv_{user}(mbs, t, i_N,\qv,\dot \qv)}{\partial \qv} \right) - 
                 f_{ODE2_t} \left(\frac{\partial \fv_{user}(mbs, t, i_N,\qv,\dot \qv)}{\partial \dot \qv} \right)
    $$ (eq-objectgenericode2-jac)

    For clarification also see the \mybold{example} in \texttt{TestModels/linearFEMgenericODE2.py}.
    
    CoordinateLoads are added for the respective ABRV:ODE2 coordinate on the RHS of the latter equation.
    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    **Userfunction**: `forceUserFunction(mbs, t, itemNumber, q, q_t)`
    A user function, which computes a force vector depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.
    Note that itemNumber represents the index of the ObjectGenericODE2 object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.
    <!--
    
    The function takes the time, coordinates q (without reference values) and coordinate velocities q\_t
    -->

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs to which object belongs |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{q} | Vector $\in \Rcal^n$ | object coordinates (e.g., nodal displacement coordinates) in current configuration, without reference values |
    | \texttt{q\_t} | Vector $\in \Rcal^n$ | object velocity coordinates (time derivative of \texttt{q}) in current configuration |
    | **return value** | Vector $\in \Rcal^{n}$ | returns force vector for object |

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `massMatrixUserFunction(mbs, t, itemNumber, q, q_t)`
    A user function, which computes a mass matrix depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs to which object belongs to |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{q} | Vector $\in \Rcal^n$ | object coordinates (e.g., nodal displacement coordinates) in current configuration, without reference values |
    | \texttt{q\_t} | Vector $\in \Rcal^n$ | object velocity coordinates (time derivative of \texttt{q}) in current configuration |
    | **return value** | MatrixContainer $\in \Rcal^{n \times n}$ | returns mass matrix for object, as exu.MatrixContainer, numpy array or list of lists; use MatrixContainer sparse format for larger matrices to speed up computations. |

    \vspace{12pt}
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `jacobianUserFunction(mbs, t, itemNumber, q, q_t, fODE2, fODE2_t)`
    A user function, which computes the jacobian of the ABRV:LHS of the equations of motion, depending on current time, states of object and two
    factors which are used to distinguish between position level and velocity level derivatives. 
    Can be used to create any kind of mechanical system by using the object states.

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs to which object belongs to |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{q} | Vector $\in \Rcal^n$ | object coordinates (e.g., nodal displacement coordinates) in current configuration, without reference values |
    | \texttt{q\_t} | Vector $\in \Rcal^n$ | object velocity coordinates (time derivative of \texttt{q}) in current configuration |
    | \texttt{fODE2} | Real | factor to be multiplied with the position level jacobian, see {eq}`eq-objectgenericode2-jac` |
    | \texttt{fODE2\_t} | Real | factor to be multiplied with the velocity level jacobian, see {eq}`eq-objectgenericode2-jac` |
    | **return value** | MatrixContainer $\in \Rcal^{n \times n}$ | returns special jacobian for object, as exu.MatrixContainer, numpy array or list of lists; use MatrixContainer sparse format for larger matrices to speed up computations; NOTE that the format of returnValue must AGREE with (dense/sparse triplet) format of stiffnessMatrix and dampingMatrix; sparse triplets MAY NOT contain zero values! |

    \vspace{12pt}
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `graphicsDataUserFunction(mbs, itemNumber)`
    A user function, which is called by the visualization thread in order to draw user-defined objects.
    The function can be used to generate any \texttt{BodyGraphicsData}, see Section [](#sec-graphicsdata).
    Use \texttt{exudyn.graphics} functions, see Section [](#sec-module-graphics), to create more complicated objects. 
    Note that \texttt{graphicsDataUserFunction} needs to copy lots of data and is therefore
    inefficient and only designed to enable simpler tests, but not large scale problems.

    For an example for \texttt{graphicsDataUserFunction} see ObjectGround, [](#sec-item-objectground).
    <!-- -->

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides reference to mbs, which can be used in user function to access all data of the object |
    | \texttt{itemNumber} | Index | integer number of the object in mbs, allowing easy access |
    | **return value** | BodyGraphicsData | list of \texttt{GraphicsData} dictionaries, see Section [](#sec-graphicsdata) |

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    *Example*:
    
```python
#user function, using variables M, K, ... from mini example, replacing ObjectGenericODE2(...)
KD = numpy.diag([200,100])
#nonlinear force example; this force is added to right-hand-side ==> negative sign!
def UFforce(mbs, t, itemNumber, q, q_t): 
    return -np.dot(KD, q_t*q) #add nonlinear term for q_t and q, q_t*q gives vector

#non-constant mass matrix:
def UFmass(mbs, t, itemNumber, q, q_t): 
    return (q[0]+1)*M #uses mass matrix from mini example

#non-constant mass matrix:
def UFgraphics(mbs, itemNumber):
    t = mbs.systemData.GetTime(exu.ConfigurationType.Visualization) #get time if needed
    p = mbs.GetObjectOutputSuperElement(objectNumber=itemNumber, variableType = exu.OutputVariableType.Position,
                                        meshNodeNumber = 0, #get first node's position 
                                        configuration = exu.ConfigurationType.Visualization)
    graphics1=graphics.Sphere(point=p,radius=0.1, color=graphics.color.red)
        graphics2 = {'type':'Line', 'data': list(p)+[0,0,0], 'color':graphics.color.blue}
    return [graphics1, graphics2] 

#now add object instead of object in mini-example:
oGenericODE2 = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[nMass0,nMass1], 
                   massMatrix=M, stiffnessMatrix=K, dampingMatrix=D,
                   forceUserFunction=UFforce, massMatrixUserFunction=UFmass,
                   visualization=VObjectGenericODE2(graphicsDataUserFunction=UFgraphics)))

```

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    miniExample=r"""    #set up a mechanical system with two nodes; it has the structure: |~~M0~~M1
    nMass0 = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,0]))
    nMass1 = mbs.AddNode(NodePoint(referenceCoordinates=[1,0,0]))

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
    
    mNode1 = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass1))
    mbs.AddLoad(Force(markerNumber = mNode1, loadVector = [10, 0, 0])) #static solution=10*(1/5000+1/5000)=0.0004

    #assemble and solve system for default parameters
    mbs.Assemble()
    
    mbs.SolveDynamic(solverType = exu.DynamicSolverType.TrapezoidalIndex2)

    #check result at default integration time
    exu.sys['testResult'] = mbs.GetNodeOutput(nMass1, exu.OutputVariableType.Position)[0]
""",
    objectType=ObjectTypeSuperElement,
    outputVariables=[
        ItemOutputVariable(OVCoordinatesTotal, r"""all ABRV:ODE2 displacement plus reference coordinates of object"""),
        ItemOutputVariable(OVCoordinates, r'all ABRV:ODE2 (displacement) coordinates'),
        ItemOutputVariable(OVCoordinates_t, OVDVelocityCoordinatesODE2),
        ItemOutputVariable(OVCoordinates_tt, r'all ABRV:ODE2 acceleration coordinates'),
        ItemOutputVariable(OVForce, OVDGeneralizedForces),
        ],
    visuParentClass=VisuParentClassVisualizationObjectSuperElement,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TArrayIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumbers',
            defaultValue='ArrayIndex()',
            description=r"""$\mathbf{n}_n = [n_0,\,\ldots,\,n_n]\tp$node numbers which provide the coordinates for the object (consecutively as provided in this list)"""),
        ItemParameter(type=TPyMatrixContainer, destination=DestComp+DestParam,
            pythonName='massMatrix',
            defaultValue='PyMatrixContainer()',
            description=r"""$\Mm \in \Rcal^{n \times n}$mass matrix of object as MatrixContainer (or numpy array / list of lists)"""),
        ItemParameter(type=TPyMatrixContainer, destination=DestComp+DestParam,
            pythonName='stiffnessMatrix',
            defaultValue='PyMatrixContainer()',
            description=r"""$\Km \in \Rcal^{n \times n}$stiffness matrix of object as MatrixContainer (or numpy array / list of lists); NOTE that (dense/sparse triplets) format must agree with dampingMatrix and jacobianUserFunction"""),
        ItemParameter(type=TPyMatrixContainer, destination=DestComp+DestParam,
            pythonName='dampingMatrix',
            defaultValue='PyMatrixContainer()',
            description=r"""$\Dm \in \Rcal^{n \times n}$damping matrix of object as MatrixContainer (or numpy array / list of lists); NOTE that (dense/sparse triplets) format must agree with stiffnessMatrix and jacobianUserFunction"""),
        ItemParameter(type=TNumpyVector, destination=DestComp+DestParam,
            pythonName='forceVector',
            defaultValue='Vector()',
            description=r'$\fv \in \Rcal^{n}$generalized force vector added to RHS'),
        ItemParameter(type=TPyFunctionVectorMbsScalarIndex2Vector, destination=DestComp+DestParam,
            pythonName='forceUserFunction',
            defaultValue=0,
            description=r"""$\fv_{user} \in \Rcal^{n}$A Python user function which computes the generalized user force vector for the ABRV:ODE2 equations; see description below"""),
        ItemParameter(type=TPyFunctionMatrixContainerMbsScalarIndex2Vector, destination=DestComp+DestParam,
            pythonName='massMatrixUserFunction',
            defaultValue=0,
            description=r"""$\Mm_{user} \in \Rcal^{n\times n}$A Python user function which computes the mass matrix instead of the constant mass matrix given in $\Mm$; return numpy array or MatrixContainer; see description below"""),
        ItemParameter(type=TPyFunctionMatrixContainerMbsScalarIndex2Vector2Scalar, destination=DestComp+DestParam,
            pythonName='jacobianUserFunction',
            defaultValue=0,
            description=r"""$\Jm_{user} \in \Rcal^{n\times n}$A Python user function which computes the jacobian, i.e., the derivative of the left-hand-side object equation w.r.t.\ the coordinates (times $f_{ODE2}$) and w.r.t.\ the velocities (times $f_{ODE2_t}$). Terms on the RHS must be subtracted from the LHS equation; the respective terms for the stiffness matrix and damping matrix are automatically added; see description below"""),
        ItemParameter(type=TArrayIndex, destination=DestComp+DestParam, cFlags=CFReadOnly,
            pythonName='coordinateIndexPerNode',
            defaultValue='ArrayIndex()',
            description=r'this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed'),
        ItemParameter(type=TNumpyVector, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='tempCoordinates',
            defaultValue='Vector()',
            description=r"""$\cv_{temp} \in \Rcal^{n}$temporary vector containing coordinates"""),
        ItemParameter(type=TNumpyVector, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='tempCoordinates_t',
            defaultValue='Vector()',
            description=r"""$\dot \cv_{temp} \in \Rcal^{n}$temporary vector containing velocity coordinates"""),
        ItemParameter(type=TNumpyVector, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='tempCoordinates_tt',
            defaultValue='Vector()',
            description=r"""$\ddot \cv_{temp} \in \Rcal^{n}$temporary vector containing acceleration coordinates"""),
        ItemFunctionDef('HasUserFunction',
            destination=DestComp,
            implementation='return (parameters.forceUserFunction!=0) || (parameters.massMatrixUserFunction!=0) || (parameters.jacobianUserFunction!=0);'),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('ComputeJacobianODE2_ODE2'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'AngularVelocity_qt', 'DisplacementMassIntegral_q', 'SuperElement']),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetOutputVariableBody'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetDisplacement'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetLocalCenterOfMass',
            implementation='return Vector3D({0.,0.,0.});',
            description='return the local position of the center of mass, used for massProportionalLoad, which may NOT be appropriate for GenericODE2'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "GenericODE2";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation='return parameters.nodeNumbers[localIndex];'),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumbers[localIndex]=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return parameters.nodeNumbers.NumberOfItems();'),
        ItemFunctionDef('GetODE2Size'),
        ItemRequestedTypes('Node', []),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded + (Index)CObjectType::SuperElement);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='return (parameters.massMatrixUserFunction==0);'),
        ItemFunctionDef('ParametersHaveChanged',
            implementation='InitializeCoordinateIndices();',
            description='This flag is reset upon change of parameters; says that the vector of coordinate indices has changed'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('GetLocalODE2CoordinateIndexPerNode',
            implementation='return parameters.coordinateIndexPerNode[localNode];'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeObjectCoordinates',
            args='Vector& coordinates, Vector& coordinates_t, ConfigurationType configuration = ConfigurationType::Current',
            description=r'compute displacement and velocity object coordinates composed from all nodal coordinates; does not include reference coordinates'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeObjectCoordinates',
            args='Vector& coordinates, ConfigurationType configuration = ConfigurationType::Current',
            description=r'compute single object coordinates composed from all nodal coordinates; does not include reference coordinates'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeObjectCoordinates_tt',
            args='Vector& coordinates_tt, ConfigurationType configuration = ConfigurationType::Current',
            description=r'compute object acceleration coordinates composed from all nodal coordinates'),
        ItemFunction(type=Tvoid, destination=DestComp, isVirtual=False,
            pythonName='InitializeCoordinateIndices',
            description=r'initialize coordinateIndexPerNode array'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionForce',
            args='Vector& force, const MainSystemBase& mainSystem, Real t, Index objectNumber, const StdVector& coordinates, const StdVector& coordinates_t',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionMassMatrix',
            args='EXUmath::MatrixContainer& massMatrix, const MainSystemBase& mainSystem, Real t, Index objectNumber, const StdVector& coordinates, const StdVector& coordinates_t, const ArrayIndex& ltg',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionJacobian',
            args='EXUmath::MatrixContainer& jacobianODE2, const MainSystemBase& mainSystem, Real t, Index objectNumber, const StdVector& coordinates, const StdVector& coordinates_t, Real factorODE2, Real factorODE2_t, const ArrayIndex& ltg',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunctionDef('HasReferenceFrame',
            implementation='localReferenceFrameNode = 0; return false;'),
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst,
            pythonName='GetNumberOfMeshNodes',
            implementation='return GetNumberOfNodes();',
            description=r'return the number of mesh nodes, which is 1 less than the number of nodes if referenceFrame is used'),
        ItemFunctionDef('GetMeshNode'),
        ItemFunctionDef('GetMeshNodeLocalPosition'),
        ItemFunctionDef('GetMeshNodeLocalVelocity'),
        ItemFunctionDef('GetMeshNodePosition'),
        ItemFunctionDef('GetMeshNodeVelocity'),
        ItemFunctionDef('GetAccessFunctionSuperElement'),
        ItemFunctionDef('GetOutputVariableTypesSuperElement'),
        ItemFunctionDef('GetOutputVariableSuperElement'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used'),
        ItemParameter(type=TNumpyMatrixI, destination=DestVisu,
            pythonName='triangleMesh',
            defaultValue='MatrixI()',
            description=r'a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!'),
        ItemParameter(type=TBool, destination=DestVisu,
            pythonName='showNodes',
            defaultValue=False,
            description=r"set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'"),
        ItemFunctionDef('CallUserFunction'),
        ItemFunctionDef('HasUserFunction',
            destination=DestVisu,
            implementation='return graphicsDataUserFunction!=0;'),
        ItemParameter(type=TPyFunctionGraphicsData, destination=DestVisu,
            pythonName='graphicsDataUserFunction',
            defaultValue=0,
            description=r'A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics data is draw in global coordinates; it can be used to implement user element visualization, e.g., beam elements or simple mechanical systems; note that this user function may significantly slow down visualization'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectGenericODE1   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectGenericODE1',
    addIncludesC=r"""#include <pybind11/numpy.h>//for NumpyMatrix
#include <pybind11/stl.h>//for NumpyMatrix
#include <pybind11/pybind11.h>
typedef py::array_t<Real> NumpyMatrix; 
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCObject,
    classDescription=r"""A system of $n$ ABRV:ODE1, having a system matrix, a rhs vector, but mostly it will use a user function to describe special ABRV:ODE1 systems. It is based on NodeGenericODE1 nodes. NOTE that all matrices, vectors, etc. must have the same dimensions $n$ or $(n \times n)$, or they must be empty $(0 \times 0)$, using [] in Python.""",
    classType=ClassTypeObject,
    equations=r"""    #### Equations of motion

    An object with node numbers $[n_0,\,\ldots,\,n_n]$ and according numbers of nodal coordinates $[n_{c_0},\,\ldots,\,n_{c_n}]$, the total number of equations (=coordinates) of the object is

    $$
    n = \sum_{i} n_{c_i},
    $$

    which is used throughout the description of this object.
    <!-- -->

    #### Equations of motion

    $$
    \dot \qv = \fv + \fv_{user}(mbs, t, i_N, \qv)
    $$ (eq-objectgenericode1-eom)

    Note that the user function $\fv_{user}(mbs, t, i_N, \qv)$ may be empty (=0), and that \texttt{iN} represents the itemNumber (=objectNumber). 

    CoordinateLoads are added for the respective ABRV:ODE1 coordinate on the RHS of the latter equation.
    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    **Userfunction**: `rhsUserFunction(mbs, t, itemNumber, q)`
    A user function, which computes a RHS vector depending on current time and states of the object. 
    Can be used to create any kind of first order system, especially state space equations (inputs are added via CoordinateLoads to every node).
    Note that itemNumber represents the index of the ObjectGenericODE1 object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.
    <!-- -->

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs to which object belongs |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{q} | Vector $\in \Rcal^n$ | object coordinates (composed from ABRV:ODE1 nodal coordinates) in current configuration, without reference values |
    | **return value** | Vector $\in \Rcal^{n}$ | returns force vector for object |

    <!--
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    **Userfunction**: `graphicsDataUserFunction(mbs, itemNumber)`
    A user function, which is called by the visualization thread in order to draw user-defined objects.
    The function can be used to generate any \texttt{BodyGraphicsData}, see Section [](#sec-graphicsdata).
    Use \texttt{exudyn.graphics} functions, see Section [](#sec-module-graphics), to create more complicated objects.
    Note that \texttt{graphicsDataUserFunction} needs to copy lots of data and is therefore
    inefficient and only designed to enable simpler tests, but not large scale problems.
    
    For an example for \texttt{graphicsDataUserFunction} see ObjectGround, [](#sec-item-objectground).
    \startTable{arguments /  return}{type or size}{description}
      \rowTable{\texttt{mbs}}{MainSystem}{provides reference to mbs, which can be used in user function to access all data of the object}
      \rowTable{\texttt{itemNumber}}{Index}{integer number $i_N$ of the object in mbs, allowing easy access}
     \rowTable{**return value**}{BodyGraphicsData}{list of \texttt{GraphicsData} dictionaries, see Section [](#sec-graphicsdata)}
    \finishTable
    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    *Example*:
    
```python
A = numpy.diag([200,100])
#simple linear user function returning A*q + const
def UFrhs(mbs, t, itemNumber, q): 
    return np.dot(A, q) + np.array([0,2])
    
nODE1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0,0],
                                    initialCoordinates=[1,0], numberOfODE1Coordinates=2))

#now add object instead of object in mini-example:
oGenericODE1 = mbs.AddObject(ObjectGenericODE1(nodeNumbers=[nODE1], 
                   rhsUserFunction=UFrhs))
                             

```

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObject,
    miniExample=r"""    #set up a 2-DOF system
    nODE1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0,0],
                                        initialCoordinates=[1,0],
                                        numberOfODE1Coordinates=2))

    #build system matrix and force vector
    #undamped mechanical system with m=1, K=100, f=1
    A = np.array([[0,1],
                  [-100,0]])
    b = np.array([0,1])
    
    oGenericODE1 = mbs.AddObject(ObjectGenericODE1(nodeNumbers=[nODE1], 
                                                   systemMatrix=A, 
                                                   rhsVector=b))
    
    #assemble and solve system for default parameters
    mbs.Assemble()
    
    sims=exu.SimulationSettings()
    solverType = exu.DynamicSolverType.RK44
    mbs.SolveDynamic(solverType=solverType, simulationSettings=sims)

    #check result at default integration time
    exu.sys['testResult'] = mbs.GetNodeOutput(nODE1, exu.OutputVariableType.Coordinates)[0]
""",
    objectType=ObjectTypeObject,
    outputVariables=[
        ItemOutputVariable(OVCoordinatesTotal, r"""all ABRV:ODE2 displacement plus reference coordinates of object"""),
        ItemOutputVariable(OVCoordinates, r'all ABRV:ODE1 coordinates'),
        ItemOutputVariable(OVCoordinates_t, r'all ABRV:ODE1 velocity coordinates'),
        ],
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TArrayIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumbers',
            defaultValue='ArrayIndex()',
            description=r"""$\mathbf{n}_n = [n_0,\,\ldots,\,n_n]\tp$node numbers which provide the coordinates for the object (consecutively as provided in this list)"""),
        ItemParameter(type=TNumpyMatrix, destination=DestComp+DestParam,
            pythonName='systemMatrix',
            defaultValue='Matrix()',
            description=r"""$\Am \in \Rcal^{n \times n}$system matrix (state space matrix) of first order ODE"""),
        ItemParameter(type=TNumpyVector, destination=DestComp+DestParam,
            pythonName='rhsVector',
            defaultValue='Vector()',
            description=r"""$\fv \in \Rcal^{n}$a constant rhs vector (e.g., for constant input)"""),
        ItemParameter(type=TPyFunctionVectorMbsScalarIndexVector, destination=DestComp+DestParam,
            pythonName='rhsUserFunction',
            defaultValue=0,
            description=r"""$\fv_{user} \in \Rcal^{n}$A Python user function which computes the right-hand-side (rhs) of the first order ODE; see description below"""),
        ItemParameter(type=TArrayIndex, destination=DestComp+DestParam, cFlags=CFReadOnly,
            pythonName='coordinateIndexPerNode',
            defaultValue='ArrayIndex()',
            description=r'this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed'),
        ItemParameter(type=TNumpyVector, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='tempCoordinates',
            defaultValue='Vector()',
            description=r"""$\cv_{temp} \in \Rcal^{n}$temporary vector containing coordinates"""),
        ItemParameter(type=TNumpyVector, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='tempCoordinates_t',
            defaultValue='Vector()',
            description=r"""$\dot \cv_{temp} \in \Rcal^{n}$temporary vector containing velocity coordinates"""),
        ItemFunctionDef('HasUserFunction',
            implementation='return (parameters.rhsUserFunction!=0);'),
        ItemFunctionDef('ComputeODE1RHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE1_ODE1);'),
        ItemAccessFunctionTypes([]),
        ItemFunctionDef('GetAccessFunction'),
        ItemFunctionDef('GetOutputVariable'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "GenericODE1";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation='return parameters.nodeNumbers[localIndex];'),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumbers[localIndex]=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return parameters.nodeNumbers.NumberOfItems();'),
        ItemFunctionDef('GetODE1Size'),
        ItemRequestedTypes('Node', []),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::MultiNoded);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('ParametersHaveChanged',
            implementation='InitializeCoordinateIndices();',
            description='This flag is reset upon change of parameters; says that the vector of coordinate indices has changed'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetLocalODE1CoordinateIndexPerNode',
            args='Index localNode',
            implementation='return parameters.coordinateIndexPerNode[localNode];',
            description=r'read access to coordinate index array'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeObjectCoordinates',
            args='Vector& coordinates, Vector& coordinates_t, ConfigurationType configuration = ConfigurationType::Current',
            description=r'compute object coordinates composed from all nodal coordinates; does not include reference coordinates'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeObjectCoordinates',
            args='Vector& coordinates, ConfigurationType configuration = ConfigurationType::Current',
            description=r'compute object coordinates composed from all nodal coordinates; does not include reference coordinates'),
        ItemFunction(type=Tvoid, destination=DestComp, isVirtual=False,
            pythonName='InitializeCoordinateIndices',
            description=r'initialize coordinateIndexPerNode array'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionRHS',
            args='Vector& rhs, const MainSystemBase& mainSystem, Real t, Index objectNumber, const StdVector& coordinates',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectKinematicTree   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectKinematicTree',
    addIncludesC=r"""#include "Linalg/KinematicsBasics.h"//for transformations
#include "Pymodules/PyMatrixVector.h"//for some matrix and vector lists
#include "Main/OutputVariable.h"
#include <pybind11/numpy.h>//for NumpyMatrix
#include <pybind11/stl.h>//for NumpyMatrix
#include <pybind11/pybind11.h>
#include "Pymodules/PyMatrixVector.h"//for some matrix and vector lists
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    addPublicC=r"""    static constexpr Index noParent = -1;//AUTO: number which defines that this link has no parent
""",
    cParentClass=ParentClassCObjectSuperElement,
    classDescription=r"""A special object to represent open kinematic trees using minimal coordinate formulation. The kinematic tree is defined by lists of joint types, parents, inertia parameters (w.r.t. COM), etc.\ per link (body) and given joint (pre) transformations from the previous joint. Every joint / link is defined by the position and orientation of the previous joint and a coordinate transformation (incl.\ translation) from the previous link's to this link's joint coordinates. The joint can be combined with a marker, which allows to attach connectors as well as joints to represent closed loop mechanisms. Efficient models can be created by using tree structures in combination with constraints and very long chains should be avoided and replaced by (smaller) jointed chains if possible. The class Robot from exudyn.robotics can also be used to create kinematic trees, which are then exported as KinematicTree or as redundant multibody system. Use specialized settings in VisualizationSettings.bodies.kinematicTree for showing joint frames and other properties.""",
    classType=ClassTypeObject,
    equations=r"""    <!-- -->

    (sec-kinematictree-additionaloutput)=
    #### SensorKinematicTree output variables

    <!-- -->
    The following output variables are available with \texttt{SensorKinematicTree} for a specific link.
    Within the link $n_i$, a local position $\LU{n_i}{\pv_{n_i}}$ is required. All output variables are available for different
    configurations. Furthermore, $\LU{0,n_i}{\Tm}$ is the homogeneous transformation from link $n_i$ coordinates to global coordinates.

    | Kinematic tree output variables | symbol | description |
    |---|---|---|
    | Position | $\LU{0}{\pv_{n_i}} = \LU{0,n_i}{\Tm} \LU{n_i}{\pv_{n_i}}$ | global position of local position at link $n_i$ |
    | Displacement | $\LU{0}{\uv_{n_i}} = \LU{0,n_i}{\Tm} \LU{n_i}{\pv_{n_i}} - \LU{0}{\pv_{n_i,\cRef}}$ | global displacement of local position at link $n_i$ |
    | Rotation | $\tphi_{n_i}$ | Tait-Bryan angles of link $n_i$ |
    | RotationMatrix | $\LU{0,n_i}{\Rot_{n_i}}$ | rotation matrix of link $n_i$ |
    | VelocityLocal | $\LU{n_i}{\vv_{n_i}}$ | local velocity of local position at link $n_i$ |
    | Velocity | $\LU{0}{\vv_{n_i}} = \LU{0,n_i}{\dot\Tm} \LU{n_i}{\pv_{n_i}}$ | global velocity of local position at link $n_i$ |
    | VelocityLocal | $\LU{n_i}{\vv_{n_i}}$ | local velocity of local position at link $n_i$ |
    | Acceleration | $\LU{0}{\av_{n_i}} = \LU{0,n_i}{\dot\Tm} \LU{n_i}{\pv_{n_i}}$ | global acceleration of local position at link $n_i$ |
    | AccelerationLocal | $\LU{n_i}{\av_{n_i}}$ | local acceleration of local position at link $n_i$ |
    | AngularVelocity | $\LU{0}{\tomega_{n_i}}$ | global angular velocity of local position at link $n_i$ |
    | AngularVelocityLocal | $\LU{n_i}{\tomega_{n_i}}$ | local angular velocity of local position at link $n_i$ |
    | AngularAcceleration | $\LU{0}{\talpha_{n_i}}$ | global angular acceleration of local position at link $n_i$ |
    | AngularAccelerationLocal | $\LU{n_i}{\talpha_{n_i}}$ | local angular acceleration of local position at link $n_i$ |

    <!-- -->

    #### General notes

    The \texttt{KinematicTree} object is used to represent the equations of motion of a (open) tree-structured multibody system
    using a minimal set of coordinates. Even though that \codeName\ is based on redundant coordinates,
    the \texttt{KinematicTree} allows to efficiently model standard multibody models based on revolute and prismatic joints.
    Especially, a chain with 3 links leads to only 3 equations of motion, while a redundant formulation would lead
    to $3 \times 7$ coordinates using Euler Parameters and $3 \times 6$ constraints for joints and Euler parameters,
    which gives a set of 39 equations. However this set of equations is very sparse and the evaluation is much faster
    than the kinematic tree.

    The question, which formulation to chose cannot be answered uniquely. However, \texttt{KinematicTree} objects
    do not include constraints, so they can be solved with explicit solvers. Furthermore, the joint values (angels)
    can be addressed directly -- controllers or sensors are generally simpler.
    <!-- -->

    #### General

    The equations follow the description given in Chapters 2 and 3 in the handbook of robotics, 2016 edition [CITE:Siciliano2016].

    Functions like \texttt{GetObjectOutputSuperElement(...)}, see [](#sec-mainsystem-object), 
    or \texttt{SensorSuperElement}, see [](#sec-mainsystem-sensor), directly access special output variables
    (\texttt{OutputVariableType}) of the (mesh) nodes of the superelement. The mesh nodes are the links of the
    \texttt{KinematicTree}.
    
    Note, however, that some functionality is considerably different for \texttt{ObjectGenericODE2}.
    
    <!-- -->

    #### Equations of motion

    The \texttt{KinematicTree} has one node of type \texttt{NodeGenericODE2} with $n$ coordinates.
    <!-- -->
    The equations of motion are built by special multibody algorithms, following Featherstone [CITE:Featherstone2008]. 
    For a short introduction into this topic, see Chapter 3 of [CITE:Siciliano2016]. 
    
    The kinematic tree defines a set of rigid bodies connected by joints, having no loops.
    In this way, every body $i$, also denoted as link, has either a previous body $p(i) \neq \mathrm{-1}$ or not.
    The previous body for body $i$ is $p(i)$. The coordinates of joint $i$ are defined as $q_i$.

    The following joint transformations are considered (as homogeneous transformations):
    \bi
      \item $\Xm_J(i)$ $\ldots$ joint transformation due to rotation or translation
      \item $\Xm_L(i)$ $\ldots$ link transformation (e.g. given by kinematics of mechanism)
      \item $\LU{i,\mathrm{-1}}{\Xm}$ $\ldots$ transformation from global (-1) to local joint $i$ coordinates
      \item $\LU{i,p(i)}{\Xm}$ $\ldots$ transformation from previous joint to joint $i$ coordinates
    \ei
    Furthermore, we use
    \bi
      \item[] $\tPhi_i$ $\ldots$ motion subspace for joint $i$
    \ei
    which denotes the transformation from joint coordinate (scalar) to rotations and translations.
    We can compute the local joint angular velocity $\tomega_i$ and translational velocity $\wv_i$, as a 6D vector $\vv^J_i$, from

    $$
    \vv^J_i = \vp{\tomega_i}{\wv_i} = \tPhi_i \, \dot q_i
    $$

    <!-- -->
    The joint coordinates, which can be rotational or translational, are stored in the vector

    $$
    \qv = [q_0, \, \ldots,\, q_{N_B-1}]\tp \, ,
    $$

    and the vector of joint velocity coordinates reads

    $$
    \dot \qv = [\dot q_0, \, \ldots,\, \dot q_{N_B-1}]\tp \, .
    $$

    Knowing the motion subspace $\tPhi_i$ for joint $i$, the velocity of joint $i$ reads

    $$
    \vv_i = \vv_{p(i)} + \tPhi_i \, \dot q_i \, ,
    $$

    and accelerations follow as

    $$
    \av_i = \av_{p(i)} + \tPhi_i \, \ddot q_i + \dot \tPhi_i \, \dot q_i\, .
    $$

    Note that the previous formulas can be interpreted coordinate free, but they are usually implemented in joint coordinates.

    The local forces due to applied forces and inertia forces are computed, for now independently, for every link,

    $$
    \fv_i = \Im_i \av_i + \vv_i \times \Im_i \vv_i - \LU{i,\mathrm{-1}}{\Xm\tp} \!\cdot\! \LU{\mathrm{-1}}{\fv}^a
    $$

    The total forces can be computed from inverse dynamics. 
    At every free end of the tree, the forces are added up for the previous link, which needs to be done recursively starting at the leaves of the tree,

    $$
    \fv_{p(i)} \mathrel{+}=  \LU{i,p(i)}{\Xm\tp} \!\cdot \fv_i
    $$

    The mass matrix is then built by recursively computing the intertia of the links and adding the joint contributions by
    projecting the local inertia into the joint motion space, see the composite-rigid-body algorithm.
    
    Note that $\cdot$ for multiplication of matrices and vectors is added for clarity, especially in case of left and right indices.
    The whole algorithm for forward and inverse dynamics is given in the following figures.
    
     <!--ignoreRST -->
    

        **Recursive Newton-Euler algorithm** (acc.\ to Featherstone). It returns the joint forces
        $\tau$ for given $\qv$, $\dot \qv$, $\mathrm{MotionSubspace}(i)$, $\Xm_{L}$ and
        $\LU{\mathrm{-1}}{\fv}^a_i$, assuming $\dot\tPhi_i=0$:

        1. start with $\vv_{\mathrm{-1}} = \Null$ and $\av_{\mathrm{-1}} = -\gv$, the gravity vector.
        2. **Forward pass** over the $N_B$ bodies, $i=0$ to $N_B-1$: the transformations
           $\Xm_J(i) = \Xm_{JT}(i, q_i)$, $\LU{i,p(i)}{\Xm} = \Xm_J(i) \, \Xm_{L}(i)$ and
           $\tPhi_i = \mathrm{MotionSubspace}(i)$, with
           $\LU{i,\mathrm{-1}}{\Xm} = \LU{i,p(i)}{\Xm} \cdot \LU{p(i),\mathrm{-1}}{\Xm}$ if
           $p(i) \neq \mathrm{-1}$; then the kinematics
           $\vv_i = \vv_{p(i)} + \tPhi_i \, \dot q_i$ and
           $\av_i = \av_{p(i)} + \vv_i \times \tPhi_i \, \dot q_i$, where $\tPhi_i \, \ddot q_i$ is put
           on the right hand side; then the forces
           $\fv_i = \Im_i \av_i + \vv_i \times \Im_i \vv_i - \LU{i,\mathrm{-1}}{\Xm\tp} \!\cdot\! \LU{\mathrm{-1}}{\fv}^a_i$.
        3. **Backward pass**, $i=N_B-1$ down to $0$: the joint force (torque)
           $\tau_i = \tPhi_i\tp \cdot \fv_i$, and
           $\fv_{p(i)} \mathrel{+}= \LU{i,p(i)}{\Xm\tp} \fv_i$ if $p(i) \neq \mathrm{-1}$.

        **Composite-rigid-body algorithm** (acc.\ to Featherstone). It returns the mass matrix $\Mm$
        for given $\tPhi_i$, $\LU{i,p(i)}{\Xm}$ and $\Im_i$:

        1. start with $\Mm_0 = \Null$ and the 6D inertia tensors $\Im_i^C = \Im_i$ for every body.
        2. **Recursively update the inertias**, $i=N_B-1$ down to $0$: project the inertia into the
           motion subspace, $\Fm = \Im_i^C \, \tPhi_i$ and $\Mm_{ii} = \tPhi_i\tp \, \Fm$; add it to
           the parent, $\Im_{p(i)}^C \mathrel{+}= \LU{i,p(i)}{\Xm\tp} \cdot \Im_i^C \cdot \LU{i,p(i)}{\Xm}$,
           if $p(i) \neq \mathrm{-1}$.
        3. **The mass matrix terms** of the same pass: with $j=i$, while $p(j) \neq \mathrm{-1}$, set
           $\Fm = \LU{j,p(j)}{\Xm\tp} \cdot \Fm$, $j = p(j)$, $\Mm_{ij} = \Fm\tp \, \tPhi_i$ and
           $\Mm_{ji} = \Mm_{ij}$.

    #### Implementation and user functions

    Currently, there is only the so-called Composite-Rigid-Body (CRB) algorithm implemented.
    This algorithm does not show the highest performance, but creates the mass matrix $\Mm_{CRB}$ and forces $\fv_{CRB}$
    in a conventional form. The equations read

    $$
    \Mm_{CRB}(\qv) \ddot \qv = \fv_{CRB}(\qv,\dot \qv) + \fv + \fv_{PD} + \fv_{user}(mbs, t, i_N,\qv,\dot \qv)
    $$ (eq-kinematictree-eom)

    The term $\fv_{CRB}(\qv,\dot \qv)$ represents inertial terms, which are due to accelerations and 
    quadratic velocities and is computed by \texttt{ComputeODE2LHS}.
    Note that the user function $\fv_{user}(mbs, t, i_N,\qv,\dot \qv)$ may be empty (=0), 
    and \texttt{iN} represents the itemNumber (=objectNumber). 
    The force $\fv$ is given by the \texttt{jointForceVector}, which also may have zero length, causing it to be ignored.
    While $\fv$ is constant, it may be varied using a \texttt{mbs.preStepUserFunction}, which can
    then represent any force over time. Note that such changes are not considered in the object's jacobian.
    
    The user force $\fv_{user}$ is described below and may represent any force over time.
    Note that this force is considered in the object's jacobian, but it does not include external 
    dependencies -- if a control law is feeds back measured quantities and couples them to forces.
    This leads to worse performance (up to non-convergence) of implicit solvers.
    
    The control force $\fv_{PD}$ realizes a simple linear control law

    $$
    \fv_{PD} = \Pm \cdot (\uv_o - \qv) + \Dm \cdot (\vv_o - \dot \qv)
    $$

    Here, the '.' operator represents an element-wise multiplication of two vectors, resulting in a vector.
    The force $\fv_{PD}$ at the ABRV:RHS acts in direction of prescribed joint motion $\uv_o$ and
    prescribed joint velocities $\vv_o$ multiplied with proportional and 'derivative' factors $P$ and $D$.
    Omitting $\uv_o$ and $\vv_o$ and putting $\fv_{PD}$ on the ABRV:LHS, we immediately can interpret these
    terms as stiffness and damping on the single coordinates.
    The control force is also considered in the object's jacobian, which is currently computed by numerical
    differentiation.
        
    More detailed equations will be added later on. Follow exactly the description (and coordinate systems) of the object parameters,
    especially for describing the kinematic chain as well as the inertial parameters.

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `forceUserFunction(mbs, t, itemNumber, q, q_t)`
    A user function, which computes a force vector applied to the joint coordinates depending on current time and states of object. 
    Note that itemNumber represents the index of the ObjectKinematicTree object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.
    <!--
    
    The function takes the time, coordinates q (without reference values) and coordinate velocities q\_t
    -->

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs to which object belongs |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{q} | Vector $\in \Rcal^n$ | object coordinates (e.g., nodal displacement coordinates) in current configuration, without reference values |
    | \texttt{q\_t} | Vector $\in \Rcal^n$ | object velocity coordinates (time derivative of \texttt{q}) in current configuration |
    | **return value** | Vector $\in \Rcal^{n}$ | returns force vector for object |

    \vspace{12pt}
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    miniExample=r"""    #build 1R mechanism (pendulum)
    L = 1 #length of link
    RBinertia = InertiaCuboid(1000, [L,0.1*L,0.1*L])
    inertiaLinkCOM = RBinertia.InertiaCOM() #KinematicTree requires COM inertia
    linkCOM = np.array([0.5*L,0.,0.]) #if COM=0, gravity does not act on pendulum!

    offsetsList = exu.Vector3DList([[0,0,0]])
    rotList = exu.Matrix3DList([np.eye(3)])
    linkCOMs=exu.Vector3DList([linkCOM])
    linkInertiasCOM=exu.Matrix3DList([inertiaLinkCOM])
    
    
    nGeneric = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0.],initialCoordinates=[0.],
                                           initialCoordinates_t=[0.],numberOfODE2Coordinates=1))

    oKT = mbs.AddObject(ObjectKinematicTree(nodeNumber=nGeneric, jointTypes=[exu.JointType.RevoluteZ], linkParents=[-1],
                                      jointTransformations=rotList, jointOffsets=offsetsList, linkInertiasCOM=linkInertiasCOM,
                                      linkCOMs=linkCOMs, linkMasses=[RBinertia.mass], 
                                      baseOffset = [0.5,0.,0.], gravity=[0.,-9.81,0.]))

    #assemble and solve system for default parameters
    mbs.Assemble()
    
    simulationSettings = exu.SimulationSettings() #takes currently set values or default values
    simulationSettings.timeIntegration.numberOfSteps = 1000 #gives very accurate results
    mbs.SolveDynamic(simulationSettings , solverType=exu.DynamicSolverType.RK67) #highly accurate!

    #check final value of angle:
    q0 = mbs.GetNodeOutput(nGeneric, exu.OutputVariableType.Coordinates)
    #exu.Print(q0)
    exu.sys['testResult'] = q0 #-3.134018551808591; RigidBody2D with 2e6 time steps gives: -3.134018551809384
""",
    objectType=ObjectTypeSuperElement,
    outputVariables=[
        ItemOutputVariable(OVCoordinates, r"""all ABRV:ODE2 joint coordinates, including reference values (which is slightly inconsistent with CoordinatesTotal used in nodes); if you need values without reference part, read out the node; these are the minimal coordinates of the object"""),
        ItemOutputVariable(OVCoordinates_t, OVDVelocityCoordinatesODE2),
        ItemOutputVariable(OVCoordinates_tt, r'all ABRV:ODE2 acceleration coordinates'),
        ItemOutputVariable(OVForce, OVDGeneralizedForces),
        ],
    pythonShortName='KinematicTree',
    visuParentClass=VisuParentClassVisualizationObjectSuperElement,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r"""$n_0 \in \Ncal^n$node number (type NodeIndex) of GenericODE2 node containing the coordinates for the kinematic tree; $n$ being the number of minimal coordinates"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='gravity',
            defaultValue=DVZeroVector3D,
            description=r"""$\LU{0}{\gv} \in \Rcal^{3}$gravity vector in inertial coordinates; used to simply apply gravity as LoadMassProportional is not available for KinematicTree"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='baseOffset',
            defaultValue=DVZeroVector3D,
            description=r"""$\LU{0}{\pv_b} \in \Rcal^{3}$offset vector for base, in global coordinates"""),
        ItemParameter(type=TJointTypeList, destination=DestComp+DestParam,
            pythonName='jointTypes',
            defaultValue='JointTypeList()',
            description=r"""$\jv_T \in \Ncal^{n}$joint types of kinematic Tree joints, using exu.JointType, like exu.JointType.RevoluteZ; must be always set"""),
        ItemParameter(type=TArrayIndex, destination=DestComp+DestParam,
            pythonName='linkParents',
            defaultValue='ArrayIndex()',
            description=r"""$\iv_p = [p_0,\, p_1,\, \ldots] \in \Ncal^{n}$index of parent joint/link; if no parent exists, the value is $-1$; by default, $p_0=-1$ because the $i$th parent index must always fulfill $p_i<i$; must be always set"""),
        ItemParameter(type=TMatrix3DList, destination=DestComp+DestParam,
            pythonName='jointTransformations',
            defaultValue='Matrix3DList()',
            description=r"""$\Tm = [\LU{p_0,j_0}{\Tm_0},\, \LU{p_1,j_1}{\Tm_1},\, \ldots ] \in [\Rcal^{3 \times 3}, ...]$list of constant joint transformations from parent joint coordinates $p_0$ to this joint coordinates $j_0$; this allows to adjust the orientation of the joint axes (but it does not affect the joint offset); if no parent exists ($-1$), the base coordinate system $0$ is used; must be always set"""),
        ItemParameter(type=TVector3DList, destination=DestComp+DestParam,
            pythonName='jointOffsets',
            defaultValue='Vector3DList()',
            description=r"""$\Vm = [\LU{p_0}{o_0},\, \LU{p_1}{o_1},\, \ldots ] \in [\Rcal^{3}, ...]$list of constant joint offsets from parent joint to this joint; $p_0$, $p_1$, $\ldots$ denote the parent coordinate systems; this means that the joint offset is added prior to performing the joint transformation; if no parent exists ($-1$), the base coordinate system $0$ is used; must be always set"""),
        ItemParameter(type=TMatrix3DList, destination=DestComp+DestParam,
            pythonName='linkInertiasCOM',
            defaultValue='Matrix3DList()',
            description=r"""$\Jm_{COM} = [\LU{j_0}{\Jm_0},\, \LU{j_1}{\Jm_1},\, \ldots ] \in [\Rcal^{3 \times 3}, ...]$list of link inertia tensors w.r.t.\ ABRV:COM in joint/link $j_i$ coordinates; must be always set"""),
        ItemParameter(type=TVector3DList, destination=DestComp+DestParam,
            pythonName='linkCOMs',
            defaultValue='Vector3DList()',
            description=r"""$\Cm = [\LU{j_0}{\cv_0},\, \LU{j_1}{\cv_1},\, \ldots ] \in [\Rcal^{3}, ...]$list of vectors for center of mass (COM) in joint/link $j_i$ coordinates; must be always set"""),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='linkMasses',
            defaultValue='Vector()',
            description=r'$\mv \in \Rcal^{n}$masses of links; must be always set'),
        ItemParameter(type=TVector3DList, destination=DestComp+DestParam,
            pythonName='linkForces',
            defaultValue='Vector3DList()',
            description=r"""$\LU{0}{\Fm} \in [\Rcal^{3}, ...]$list of 3D force vectors per link in global coordinates acting on joint frame origin; use force-torque couple to realize off-origin forces; defaults to empty list $[]$, adding no forces"""),
        ItemParameter(type=TVector3DList, destination=DestComp+DestParam,
            pythonName='linkTorques',
            defaultValue='Vector3DList()',
            description=r"""$\LU{0}{\Fm_\tau} \in [\Rcal^{3}, ...]$list of 3D torque vectors per link in global coordinates; defaults to empty list $[]$, adding no torques"""),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='jointForceVector',
            defaultValue='Vector()',
            description=r"""$\fv \in \Rcal^{n}$generalized force vector per coordinate added to RHS of EOM; represents a torque around the axis of rotation in revolute joints and a force in prismatic joints; for a revolute joint $i$, the torque $f[i]$ acts positive (w.r.t.\ rotation axis) on link $i$ and negative on parent link $p_i$; must be either empty list/array $[]$ (default) or have size $n$"""),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='jointPositionOffsetVector',
            defaultValue='Vector()',
            description=r"""$\uv_o \in \Rcal^{n}$offset for joint coordinates used in P(D) control; acts in positive joint direction similar to jointForceVector; should be modified, e.g., in preStepUserFunction; must be either empty list/array $[]$ (default) or have size $n$"""),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='jointVelocityOffsetVector',
            defaultValue='Vector()',
            description=r"""$\vv_o \in \Rcal^{n}$velocity offset for joint coordinates used in (P)D control; acts in positive joint direction similar to jointForceVector; should be modified, e.g., in preStepUserFunction; must be either empty list/array $[]$ (default) or have size $n$"""),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='jointPControlVector',
            defaultValue='Vector()',
            description=r"""$\Pm \in \Rcal^{n}$proportional (P) control values per joint (multiplied with position error between joint value and offset $\uv_o$); note that more complicated control laws must be implemented with user functions; must be either empty list/array $[]$ (default) or have size $n$"""),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='jointDControlVector',
            defaultValue='Vector()',
            description=r"""$\Dm \in \Rcal^{n}$derivative (D) control values per joint (multiplied with velocity error between joint velocity and velocity offset $\vv_o$); note that more complicated control laws must be implemented with user functions; must be either empty list/array $[]$ (default) or have size $n$"""),
        ItemParameter(type=TPyFunctionVectorMbsScalarIndex2Vector, destination=DestComp+DestParam,
            pythonName='forceUserFunction',
            defaultValue=0,
            description=r"""$\fv_{user} \in \Rcal^{n}$A Python user function which computes the generalized force vector on RHS with identical action as jointForceVector; see description below"""),
        ItemParameter(type=TResizableVector, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='tempVector',
            defaultValue='ResizableVector()',
            description=r'temporary vector during computation of mass and ODE2LHS'),
        ItemParameter(type=TResizableVector, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='tempVector2',
            defaultValue='ResizableVector()',
            description=r'second temporary vector during computation of mass and ODE2LHS'),
        ItemParameter(type=TResizableMatrix, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='tempMatrix',
            defaultValue='ResizableMatrix()',
            description=r'temporary matrix during computation of inverse of mass matrix'),
        ItemParameter(type=TArrayIndex, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='tempArrayIndex',
            defaultValue='ArrayIndex()',
            description=r'temporary array during computation of inverse of mass matrix'),
        ItemParameter(type=TTransformation66List, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='jointTransformationsTemp',
            defaultValue='Transformation66List()',
            description=r"""$\Xm \in \Rcal^{n \times (6 \times 6)}$temporary list containing transformations (Pluecker transforms) per joint"""),
        ItemParameter(type=TVector6DList, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='jointVelocitiesTemp',
            defaultValue='Vector6DList()',
            description=r"""$\Vm_j \in \Rcal^{n \times 6}$temporary list containing 6D velocities per joint"""),
        ItemParameter(type=TVector6DList, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='jointAccelerationsTemp',
            defaultValue='Vector6DList()',
            description=r"""$\Am_j \in \Rcal^{n \times 6}$temporary list containing 6D accelerations per joint"""),
        ItemParameter(type=TTransformation66List, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='jointTransformationsTempVis',
            defaultValue='Transformation66List()',
            description=r"""$\Xm \in \Rcal^{n \times (6 \times 6)}$temporary list containing transformations (Pluecker transforms) per joint; for visualization!"""),
        ItemParameter(type=TVector6DList, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='jointVelocitiesTempVis',
            defaultValue='Vector6DList()',
            description=r"""$\Vm_j \in \Rcal^{n \times 6}$temporary list containing 6D velocities per joint; for visualization!"""),
        ItemParameter(type=TVector6DList, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='jointAccelerationsTempVis',
            defaultValue='Vector6DList()',
            description=r"""$\Am_j \in \Rcal^{n \times 6}$temporary list containing 6D accelerations per joint; for visualization!"""),
        ItemParameter(type=TInertiaList, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='linkInertias',
            defaultValue='InertiaList()',
            description=r"""$\Jm_{66} \in \Rcal^{n \times (6 \times 6)}$temporary list link inertias as Pluecker transforms per link"""),
        ItemParameter(type=TVector6DList, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='motionSubspaces',
            defaultValue='Vector6DList()',
            description=r"""$\Mm\Sm \in \Rcal^{n \times 6}$temporary list containing 6D motion subspaces per joint"""),
        ItemParameter(type=TTransformation66List, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='jointTempT66',
            defaultValue='Transformation66List()',
            description=r"""$\Xm_j \in \Rcal^{n \times 6}$temporary list containing 66 transformations per joint"""),
        ItemParameter(type=TVector6DList, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='jointForces',
            defaultValue='Vector6DList()',
            description=r"""$\Fm_j \in \Rcal^{n \times 6}$temporary list containing 6D torques/forces per joint/link"""),
        ItemFunctionDef('HasUserFunction',
            destination=DestComp,
            implementation='return (parameters.forceUserFunction!=0);'),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'AngularVelocity_qt', 'KinematicTree']),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetOutputVariableBody'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetDisplacement'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetLocalCenterOfMass',
            description='return the local position of the center of mass, used for massProportionalLoad, which may NOT be appropriate for GenericODE2'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetPositionKinematicTree',
            args='const Vector3D& localPosition, Index linkNumber, ConfigurationType configuration = ConfigurationType::Current',
            description=r"return the (global) position of 'localPosition' of linkNumber according to configuration type"),
        ItemFunction(type=TMatrixND(3, 3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetRotationMatrixKinematicTree',
            args='Index linkNumber, ConfigurationType configuration = ConfigurationType::Current',
            description=r'return the rotation matrix of of linkNumber according to configuration type'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetVelocityKinematicTree',
            args='const Vector3D& localPosition, Index linkNumber, ConfigurationType configuration = ConfigurationType::Current',
            description=r"return the (global) velocity of 'localPosition' and linkNumber according to configuration type"),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetAngularVelocityKinematicTree',
            args='Index linkNumber, ConfigurationType configuration = ConfigurationType::Current',
            description=r'return the (global) angular velocity of linkNumber according to configuration type'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetAngularVelocityLocalKinematicTree',
            args='Index linkNumber, ConfigurationType configuration = ConfigurationType::Current',
            description=r'return the (local) angular velocity of linkNumber according to configuration type'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetAccelerationKinematicTree',
            args='const Vector3D& localPosition, Index linkNumber, ConfigurationType configuration = ConfigurationType::Current',
            description=r"return the (global) acceleration of 'localPosition' and linkNumber according to configuration type"),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetAngularAccelerationKinematicTree',
            args='Index linkNumber, ConfigurationType configuration = ConfigurationType::Current',
            description=r'return the (global) angular acceleration of linkNumber according to configuration type'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "KinematicTree";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetODE2Size',
            implementation='return parameters.jointTransformations.NumberOfItems();',
            description=r'number of ABRV:ODE2 coordinates'),
        ItemRequestedTypes('Node', ['GenericODE2']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded + (Index)CObjectType::SuperElement + (Index)CObjectType::KinematicTree);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='return false;'),
        ItemFunctionDef('ParametersHaveChanged',
            implementation=';',
            description='This flag is reset upon change of parameters; says that the vector of coordinate indices has changed'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='NumberOfLinks',
            implementation='return parameters.jointTransformations.NumberOfItems();',
            description=r'number of links used in computation functions for kinematic tree'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionForce',
            args='Vector& force, const MainSystemBase& mainSystem, Real t, Index objectNumber, const StdVector& coordinates, const StdVector& coordinates_t',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetNegativeGravity6D',
            args='Vector6D& gravity6D',
            description=r'compute negative 6D gravity to be used in Pluecker transforms'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='JointTransformMotionSubspace66',
            args='Joint::Type jointType, Real q, Transformation66& T, Vector6D& MS',
            description=r'compute joint transformation T and motion subspace MS for jointType and joint value q'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeTreeTransformations',
            args='ConfigurationType configuration, bool computeVelocitiesAccelerations, bool computeAbsoluteTransformations, Transformation66List& Xup, Vector6DList& V, Vector6DList& A',
            description=r'compute list of Pluecker transformations Xup, 6D velocities and 6D acceleration terms (not joint accelerations) per joint'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeMassMatrixAndODE2LHS',
            args='ResizableMatrix* massMatrix, const ArrayIndex* ltg, Vector* ode2Lhs, Index objectNumber, bool computeMass',
            description=r'compute mass matrix if computeMass = true and compute ODE2LHS vector if computeMass=false'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='AddExternalForces6D',
            args='const Transformation66List& Xup, Vector6DList& Fvp',
            description=r'function which adds 3D torques/forces per joint to Fvp'),
        ItemFunctionDef('HasReferenceFrame',
            implementation='localReferenceFrameNode = 0; return false;'),
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst,
            pythonName='GetNumberOfMeshNodes',
            implementation='return parameters.jointTransformations.NumberOfItems();',
            description=r'return the number of mesh nodes; these are virtual nodes per link, emulating rigid bodies recomputed from kinematic tree'),
        ItemFunctionDef('GetMeshNodePosition'),
        ItemFunctionDef('GetMeshNodeVelocity'),
        ItemFunctionDef('GetAccessFunctionSuperElement'),
        ItemFunctionDef('GetOutputVariableTypesSuperElement'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetOutputVariableKinematicTree',
            args='OutputVariableType variableType, const Vector3D& localPosition, Index linkNumber, ConfigurationType configuration, Vector& value',
            description=r'get extended output variables for multi-nodal objects with mesh nodes'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeRigidBodyMarkerDataKT',
            args='const Vector3D& localPosition, Index linkNumber, bool computeJacobian, MarkerData& markerData',
            description=r'accelerator function for faster computation of MarkerData for rigid bodies/joints'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeJacobian',
            args='Index linkNumber, const Vector3D& position, const Transformation66List& jointTransformations, ResizableMatrix& positionJacobian, ResizableMatrix& rotationJacobian',
            description=r'compute rot+pos jacobian of (global) position at linkNumber, using pre-computed joint transformations'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=TBool, destination=DestVisu,
            pythonName='showLinks',
            defaultValue=True,
            description=r'set true, if links shall be shown; if graphicsDataList is empty, a standard drawing for links is used (drawing a cylinder from previous joint or base to next joint; size relative to frame size in KinematicTree visualization settings); else graphicsDataList are used per link; NOTE visualization of joint and COM frames can be modified via visualizationSettings.bodies.kinematicTree'),
        ItemParameter(type=TBool, destination=DestVisu,
            pythonName='showJoints',
            defaultValue=True,
            description=r'set true, if joints shall be shown; if graphicsDataList is empty, a standard drawing for joints is used (drawing a cylinder for revolute joints; size relative to frame size in KinematicTree visualization settings)'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used'),
        ItemParameter(type=TNumpyMatrixI, destination=DestVisu, cFlags=CFNoInterface,
            pythonName='triangleMesh',
            defaultValue='MatrixI()',
            description=r'unused in KinematicTree'),
        ItemParameter(type=TBool, destination=DestVisu, cFlags=CFNoInterface,
            pythonName='showNodes',
            defaultValue=False,
            description=r'unused in KinematicTree'),
        ItemFunctionDef('HasUserFunction',
            destination=DestVisu,
            implementation='return false;'),
        ItemParameter(type=TBodyGraphicsDataList, destination=DestVisu,
            pythonName='graphicsDataList',
            defaultValue=NoDefaultValue,
            description=r'Structure contains data for link/joint visualization; data is defined as list of BodyGraphicsData where every BodyGraphicsData corresponds to one link/joint; must either be emtpy list or length must agree with number of links'),
        ItemFunctionDef('UpdateGraphics'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectFFRF   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectFFRF',
    addIncludesC=r"""#include <pybind11/numpy.h>//for NumpyMatrix
#include <pybind11/stl.h>//for NumpyMatrix
#include <pybind11/pybind11.h>
typedef py::array_t<Real> NumpyMatrix; 
#include "Pymodules/PyMatrixContainer.h"//for some ABRV:FFRF matrices
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    addPublicC=r"""    static constexpr Index ffrfNodeDim = 3; //dimension of nodes (=displacement coordinates per node)
    static constexpr Index rigidBodyNodeNumber  = 0; //number of rigid body node (usually = 0)
""",
    author=r'Gerstmayr Johannes, Zwölfer Andreas',
    cParentClass=ParentClassCObjectSuperElement,
    classDescription=r"""This object is used to represent equations modelled by the ABRV:FFRF. It contains a RigidBodyNode (always node 0) and a list of other nodes representing the finite element nodes used in the ABRV:FFRF. Note that temporary matrices and vectors are subject of change in future. NOTE: Usually you SHOULD NOT USE THIS OBJECT - use the much more efficient ObjectFFRFreducedOrder object with modal reduction instead.""",
    classType=ClassTypeObject,
    equations=r"""    #### Additional output variables for superelement node access

    Functions like \texttt{GetObjectOutputSuperElement(...)}, see [](#sec-mainsystem-object), 
    or \texttt{SensorSuperElement}, see [](#sec-mainsystem-sensor), directly access special output variables
    (\texttt{OutputVariableType}) of the mesh nodes $n_i$ of the superelement.
    Additionally, the contour drawing of the object can make use the \texttt{OutputVariableType} of the meshnodes.
    <!-- -->

    (sec-objectffrf-superelementoutput)=
    #### Super element output variables

    <!-- -->

    | super element output variables | symbol | description |
    |---|---|---|
    | Position | $\LU{0}{\pv}\cConfig(n_i) = \LU{0}{\pRef\cConfig} + \LU{0b}{\Rot}\cConfig \LU{b}{\pv}\cConfig(n_i)$ | global position of mesh node $n_i$ including rigid body motion and flexible deformation |
    | Displacement | $\LU{0}{\cv}\cConfig(n_i) = \LU{0}{\pv\cConfig(n_i)} - \LU{0}{\pv\cRef(n_i)}$ | global displacement of mesh node $n_i$ including rigid body motion and flexible deformation |
    | Velocity | $\LU{0}{\vv}\cConfig(n_i) = \LU{0}{\dot \pRef\cConfig} + \LU{0b}{\Rot}\cConfig (\LU{b}{\dot \qv\indf}\cConfig(n_i) + \LU{b}{\tomega}\cConfig \times \LU{b}{\pv}\cConfig(n_i))$ | global velocity of mesh node $n_i$ including rigid body motion and flexible deformation |
    | Acceleration | $\begin{array}{l} \LU{0}{\av}\cConfig(n_i) = \LU{0}{\ddot \pRef\cConfig}\cConfig \\
                                  + \LU{0b}{\Rot}\cConfig \LU{b}{\ddot \qv\indf}\cConfig(n_i) \\
                                  + 2\LU{0}{\tomega}\cConfig \times \LU{0b}{\Rot}\cConfig \LU{b}{\dot \qv\indf}\cConfig(n_i) \\
                                  + \LU{0}{\talpha}\cConfig \times \LU{0}{\pv}\cConfig(n_i) \\
                                  + \LU{0}{\tomega}\cConfig \times (\LU{0}{\tomega}\cConfig \times \LU{0}{\pv}\cConfig(n_i)) \end{array}$ | global acceleration of mesh node $n_i$ including rigid body motion and flexible deformation; note that $\LU{0}{\pv}\cConfig(n_i) = \LU{0b}{\Rot} \LU{b}{\pv}\cConfig(n_i)$ |
    | DisplacementLocal | $\LU{b}{\dv}\cConfig(n_i) = \LU{b}{\pv}\cConfig(n_i) - \LU{b}{\xv}\cRef(n_i)$ | local displacement of mesh node $n_i$, representing the flexible deformation within the body frame; note that $\LU{0}{\uv}\cConfig \neq \LU{0b}{\Rot}\LU{b}{\dv}\cConfig$ ! |
    | VelocityLocal | $\LU{b}{\dot \qv\indf}\cConfig(n_i)$ | local velocity of mesh node $n_i$, representing the rate of flexible deformation within the body frame |

    <!--
    
    
    -->

    #### Definition of quantities

    | intermediate variables | symbol | description |
    |---|---|---|
    | object coordinates | $\qv = [\qv\indt\tp,\;\qv\indr\tp,\;\qv\indf\tp]\tp$ | object coordinates |
    | rigid body coordinates | $\qv\indrigid = [\qv\indt\tp,\;\qv\indr\tp]\tp =  [q_0,\,q_1,\,q_2,\,\psi_0,\,\psi_1,\,\psi_2,\,\psi_3]\tp$ | rigid body coordinates in case of Euler parameters |
    | reference frame (rigid body) position | $\LU{0}{\pRef\cConfig} = \LU{0}{\qv_\mathrm{t,config}}+\LU{0}{\qv_\mathrm{t,ref}}$ | global position of underlying rigid body node $n_0$ which defines the reference frame origin |
    | reference frame (rigid body) orientation | $\LU{0b}{\Rot(\ttheta)}\cConfig$ | transformation matrix for transformation of local (reference frame) to global coordinates, given by underlying rigid body node $n_0$ |
    | local nodal position | $\LU{b}{\pv^{(i)}} = \LU{b}{\xv^{(i)}}\cRef + \LU{b}{\qv\indf^{(i)}} $ | vector of body-fixed (local) position of node $(i)$, including flexible part |
    | local nodal positions | $\LU{b}{\pv} = \LU{b}{\xv}\cRef + \LU{b}{\qv\indf}$ | vector of all body-fixed (local) nodal positions including flexible part |
    | rotation coordinates | $\ttheta\cCur = [\psi_0,\,\psi_1,\,\psi_2,\,\psi_3]\tp\cRef + [\psi_0,\,\psi_1,\,\psi_2,\,\psi_3]\cCur\tp$ | rigid body coordinates in case of Euler parameters |
    | flexible coordinates | $\LU{b}{\qv\indf}$ | flexible, body-fixed coordinates |
    | transformation of flexible coordinates | $\LU{0b}{\Am_{bd}} = \mathrm{diag}([\LU{0b}{\Am},\;\ldots,\;\LU{0b}{\Am})$ | block diagonal transformation matrix, which transforms all flexible coordinates from local to global coordinates |

    <!--++++++++++++++++++++++++++++++++++++++ -->
    The derivations follow Zwölfer and Gerstmayr [CITE:ZwoelferGerstmayr2021] with only small modifications in the notation.

    #### Nodal coordinates

    Consider an object with $n = 1 + n_\mathrm{nf}$ nodes, $n_\mathrm{nf}$ being the number of 'flexible' nodes and one additional node is the rigid body node for the reference frame.
    The list if node numbers is $[n_0,\,\ldots,\,n_{n_\mathrm{nf}}]$ and the according numbers of 
    nodal coordinates are $[n_{c_0},\,\ldots,\,n_{c_n}]$, where $n_0$ denotes the rigid body node.
    This gives $n_c$ total nodal coordinates, 

    $$
    n_c = \sum_{i=0}^{n_\mathrm{nf}} n_{c_i} \, ,
    $$

    whereof the number of flexible coordinates is

    $$
    n\indf = 3 \cdot n_\mathrm{nf} \, .
    $$

    
    \noindent The total number of equations (=coordinates) of the object is $n_c$.
    The first node $n_0$ represents the rigid body motion of the underlying reference frame with $n_{c\indr} = n_{c_0}$ coordinates \footnote{e.g., 
    $n_{c\indr}=6$ coordinates for Euler angles and $n_{c\indr}=7$ coordinates in case of Euler parameters; currently only the Euler parameter
    case is implemented.}. 
    
    #### Kinematics

    We assume a finite element mesh with 
    The kinematics of the ABRV:FFRF is based on a splitting of 
    translational ($\cv_t \in \Rcal^{n\indf}$), rotational ($\cv\indr \in \Rcal^{n\indf}$) and flexible ($\cv\indf \in \Rcal^{n\indf}$) nodal displacements, 

    $$
    \LU{0}{\cv} = \LU{0}{\cv\indt} + \LU{0}{\cv\indr} + \LU{0}{\cv\indf} \, .
    $$ (eq-objectffrf-coordinatessplitting)

    which are written in global coordinates in {eq}`eq-objectffrf-coordinatessplitting` but will be transformed to other coordinates later on.
    
    In the present formulation of \texttt{ObjectFFRF}, we use the following set of object coordinates (unknowns)

    $$
    \qv = \left[\LU{0}{\qv\indt\tp} \;\; \ttheta\tp \;\; \LU{b}{\qv\indf\tp} \right]\tp \in \Rcal^{n_c}
    $$

    with $\LU{0}{\qv}\indt \in \Rcal^{3}$, $\ttheta \in \Rcal^{4}$ and $\LU{b}{\qv}\indf \in \Rcal^{n\indf}$.
    Note that parts of the coordinates $\qv$ can be already interpreted in specific coordinate systems, which is therefore added.
    
    With the relations 

    $$
    \begin{aligned}
    \tPhi\indt &= \left[\ImThree ,\; \ldots ,\; \ImThree \right]\tp \in \Rcal^{n\indf \times 3} \, ,\\
            \LU{0}{\cv\indt} &= \tPhi\indt \LU{0}{\qv\indt} \, ,\\
            \LU{0}{\cv\indr} &= \left(\LU{0b}{\Am_{bd}} - \Im_{bd}\right) \LU{b}{\xv\cRef} \, ,\\
            \LU{0}{\cv\indf} &= \LU{0b}{\Am_{bd}} \LU{b}{\qv\indf} \, , \mathrm{and}\\
            \Im_{bd} &= \mathrm{diag}(\ImThree, \; \ldots ,\; \ImThree) \in \Rcal^{n\indf \times n\indf}  \, ,
    \end{aligned}
    $$ (eq-objectffrf-phit)

    we obtain the total relation of (global) nodal displacements to the object coordinates

    $$
    \LU{0}{\cv} = \tPhi\indt \LU{0}{\qv\indt} + \left(\LU{0b}{\Am_{bd}} - \Im_{bd}\right) \LU{b}{\xv\cRef} + \LU{0b}{\Am_{bd}} \LU{b}{\qv\indf} \, .
    $$

    On velocity level, we have

    $$
    \LU{0}{\dot \cv} = \Lm \dot \qv \, ,
    $$

    with the matrix $\Lm \in \Rcal^{n\indf \times n_c}$

    $$
    \Lm = \left[\tPhi\indt ,\;\; -\LU{0b}{\Am_{bd}} \LU{b}{\tilde \pv} \LU{b}{\Gm} ,\;\; \LU{0b}{\Am_{bd}} \right]
    $$

    with the rotation parameters specific matrix $\LU{b}{\Gm}$, implicitly defined in the rigid body node by the relation $\LU{b}{\tomega} = \LU{b}{\Gm} \dot \ttheta$
    and the body-fixed nodal position vector (for node $i$)

    $$
    \LU{b}{\pv} = \LU{b}{\xv\cRef} + \LU{b}{\qv\indf}, \quad \LU{b}{\pv^{(i)}} = \LU{b}{\xv^{(i)}\cRef} + \LU{b}{\qv_{\mathrm{f},i}^{(i)}}
    $$

    and the special tilde matrix for vectors $\pv \in \Rcal^{3 {n_\mathrm{nf}}}$, 

    $$
    \LU{b}{\tilde \pv} = \vr{\LU{b}{\tilde\pv^{(i)}}}{\vdots}{\LU{b}{\tilde\pv^{(i)}}} \in \Rcal^{3{n_\mathrm{nf}} \times 3} \, .
    $$ (eq-objectffrf-specialtilde)

    with the tilde operator for a $\pv^{(i)} \in \Rcal^{3}$ defined in the common notations section.
    <!--+++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->

    #### Equations of motion

    <!-- -->
    We use the Lagrange equations extended for constraint $\gv$,

    $$
    \frac{d}{dt} \left( \frac{\partial T}{\partial \dot \qv\tp} \right) - \frac{\partial T}{\partial \qv\tp}
            + \frac{\partial V}{\partial \qv\tp} + \frac{\partial \tlambda\tp \gv}{\partial \qv\tp} = \frac{\partial W}{\partial \qv\tp}
    $$

    with the quantities

    $$
    \begin{aligned}
    T(\LU{0}{\dot \cv(\qv, \dot \qv)}) &= \frac{1}{2}\LU{0}{\dot \cv\tp} \LU{0}{\Mm}  \LU{0}{\dot \cv}  
            = \frac{1}{2}\LU{0}{\dot \cv\tp} \LU{0b}{\Am_{bd}} \LU{b}{\Mm} \LU{0b}{\Am_{bd}}\tp  \LU{0}{\dot \cv}
            = \frac{1}{2}\LU{0}{\dot \cv\tp} \LU{b}{\Mm}  \LU{0}{\dot \cv}\\
            V(\LU{0}{\qv\indf}) &= \frac{1}{2}\LU{b}{\qv\indf\tp} \LU{b}{\Km}  \LU{b}{\qv\indf}  \\
            \delta W(\LU{0}{\cv(\qv)},t) &= \LU{b}{\delta \cv \tp} \fv  \\
            \gv(\qv, t) &= \Null  \\
    \end{aligned}
    $$

    Note that $\LU{b}{\Mm}$ and $\LU{b}{\Km}$ are the conventional finite element mass an stiffness 
    matrices defined in the body frame.
    
    Elementary differentiation rules of the Lagrange equations lead to

    $$
    \Lm\tp \Mm \Lm \ddot \qv + \Lm\tp \Mm \dot \Lm \dot \qv + \hat \Km \qv + \frac{\partial \gv}{\partial \qv\tp} \tlambda = \Lm\tp \fv
    $$ (eq-objectffrf-leq)

    with $\Mm = \LU{b}{\Mm}$ and $\hat \Km$ becoming obvious in {eq}`eq-objectffrf-eom`. 
    Note that {eq}`eq-objectffrf-leq` is given in global coordinates for the translational part, in terms of rotation parameters
    for the rotation part and in body-fixed coordinates for the flexible part of the equations.
    
    In case that \texttt{computeFFRFterms = True}, {eq}`eq-objectffrf-leq` can be transformed into the equations of motion,

    $$
    \left(\Mm_{user}(mbs, t, i_N, \qv,\dot \qv) + \mr{\Mm\indtt}{\Mm\indtr}{\Mm\indtf} {}{\Mm\indrr}{\Mm\indrf} 
                        {\mathrm{sym.}}{}{\LU{b}{\Mm}} \right) \ddot \qv + 
                        \mr{0}{0}{0} {0}{0}{0} {0}{0}{\LU{b}{\Dm}} \dot \qv + \mr{0}{0}{0} {0}{0}{0} {0}{0}{\LU{b}{\Km}} \qv = 
                        \fv_{v}(\qv,\dot \qv) + \vp{\fv\indr}{\LURU{0b}{\Am}{bd}{\mathrm{T}} \fv\indf} + \fv_{user}(mbs, t, i_N, \qv, \dot \qv)
    $$ (eq-objectffrf-eom)

    in which \texttt{iN} represents the itemNumber (=objectNumber of ObjectFFRF in mbs) in the user function.
    The mass terms are given as

    $$
    \begin{aligned}
    \Mm\indtt &= \tPhi\indt\tp \LU{b}{\Mm} \tPhi\indt,\\
          \Mm\indtr &= -\LU{0b}{\Rot} \tPhi\indt\tp \LU{b}{\Mm} \LU{b}{\tilde \pv} \LU{b}{\Gm} ,\\
          \Mm\indtf &= \LU{0b}{\Rot} \tPhi\indt\tp \LU{b}{\Mm} ,\\
          \Mm\indrr &= \LU{b}{\Gm}\tp \LU{b}{\tilde \pv\tp} \LU{b}{\Mm} \LU{b}{\tilde \pv} \LU{b}{\Gm} ,\\
          \Mm\indrf &= - \LU{b}{\Gm}\tp \LU{b}{\tilde \pv\tp} \LU{b}{\Mm} \, .
    \end{aligned}
    $$

    In case that \texttt{computeFFRFterms = False}, the mass terms $\Mm\indtt, \Mm\indtr, \Mm\indtf, \Mm\indrr, 
    \Mm\indrf, \LU{b}{\Mm}$ in {eq}`eq-objectffrf-eom` are set to zero (and not computed) and
    the quadratic velocity vector $\fv_{v} = \Null$.
    Note that the user functions $\fv_{user}(mbs, t, i_N, \qv,\dot \qv)$ and $\Mm_{user}(mbs, t, i_N, \qv,\dot \qv)$ may be empty (=0). 
    The detailed equations of motion for this element can be found in [CITE:ZwoelferGerstmayr2020].

    The quadratic velocity vector follows as

    $$
    \fv_{v}(\qv,\dot \qv) = \vr
          {-\LU{0b}{\Rot} \tPhi\indt\tp \LU{b}{\Mm}\left( \omegaBDtilde \omegaBDtilde \LU{b}{\pv} + 
                                                         2 \omegaBDtilde \LU{b}{\dot \qv}\indf - 
                                                         \LU{b}{\tilde \pv} \LU{b}{\dot \Gm} \dot \ttheta \right)}
          {\LU{b}{\Gm}\tp \LU{b}{\tilde \pv\tp} \LU{b}{\Mm} \left( \omegaBDtilde \omegaBDtilde \LU{b}{\pv} + 
                                                         2 \omegaBDtilde \LU{b}{\dot \qv}\indf - 
                                                         \LU{b}{\tilde \pv} \LU{b}{\dot \Gm} \dot \ttheta \right)}
          {-\LU{b}{\Mm} \left( \omegaBDtilde \omegaBDtilde \LU{b}{\pv} + 
                                                         2 \omegaBDtilde \LU{b}{\dot \qv}\indf - 
                                                         \LU{b}{\tilde \pv} \LU{b}{\dot \Gm} \dot \ttheta \right)}
    $$

    with the special matrix

    $$
    \omegaBDtilde = \mathrm{diag}\left(\LU{b}{\tilde \tomega_\mathrm{bd}}, \; \ldots ,\; \LU{b}{\tilde \tomega_\mathrm{bd}}  \right)
          \in \Rcal^{n\indf \times n\indf}
    $$

    CoordinateLoads are added for each ABRV:ODE2 coordinate on the RHS of the latter equation. 
    
    \noindent If the rigid body node is using Euler parameters $\ttheta = [\theta_0,\,\theta_1,\,\theta_2,\,\theta_3]\tp$, an {\bf additional constraint} (constraint nr.\ 0) is 
    added automatically for the Euler parameter norm, reading

    $$
    1 - \sum_{i=0}^{3} \theta_i^2 = 0.
    $$

    
    <!--
    \noindent If \texttt{constrainRigidBodyMotion==True}, {\bf 6 algebraic constraints} (constraint nrs.\ $[1\ldots 6]$) are added to restrict rigid body motion:
    of the flexible coordinates, by applying the constraints of a Tisserand frame, giving 3 constraints for the position of the center of mass
    -->
    In order to suppress the rigid body motion of the mesh nodes, you should apply a ObjectConnectorCoordinateVector object with the following constraint
    equations which impose constraints of a so-called Tisserand frame, giving 3 constraints for the position of the center of mass

    $$
    \Phi\indt\tp \LU{b}{\Mm} \qv\indf = 0
    $$

    and 3 constraints for the rotation,

    $$
    \tilde\xv_{f}\tp \LU{b}{\Mm} \qv\indf = 0
    $$

    <!--
    
    ++++++++++++++++++++++++++++++++++++++
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    **Userfunction**: `forceUserFunction(mbs, t, itemNumber, q, q_t)`
    A user function, which computes a force vector depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.
    <!--
    
    The function takes the time, coordinates q (without reference values) and coordinate velocities q\_t
    -->

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs to which object belongs |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{q} | Vector $\in \Rcal^n_c$ | object coordinates (nodal displacement coordinates of rigid body and mesh nodes) in current configuration, without reference values |
    | \texttt{q\_t} | Vector $\in \Rcal^n_c$ | object velocity coordinates (time derivative of \texttt{q}) in current configuration |
    | **return value** | Vector $\in \Rcal^{n_c}$ | returns force vector for object |

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `massMatrixUserFunction(mbs, t, itemNumber, q, q_t)`
    A user function, which computes a mass matrix depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs to which object belongs |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{q} | Vector $\in \Rcal^n_c$ | object coordinates (nodal displacement coordinates of rigid body and mesh nodes) in current configuration, without reference values |
    | \texttt{q\_t} | Vector $\in \Rcal^n_c$ | object velocity coordinates (time derivative of \texttt{q}) in current configuration |
    | **return value** | NumpyMatrix $\in \Rcal^{n_c \times n_c}$ | returns mass matrix for object |

    \vspace{12pt}
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    objectType=ObjectTypeSuperElement,
    outputVariables=[
        ItemOutputVariable(OVCoordinates, r'all ABRV:ODE2 coordinates'),
        ItemOutputVariable(OVCoordinates_t, OVDVelocityCoordinatesODE2),
        ItemOutputVariable(OVCoordinates_tt, r'all ABRV:ODE2 acceleration coordinates'),
        ItemOutputVariable(OVForce, OVDGeneralizedForces),
        ],
    visuParentClass=VisuParentClassVisualizationObjectSuperElement,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TArrayIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumbers',
            defaultValue='ArrayIndex()',
            description=r"""$\mathbf{n}\indf = [n_0,\,\ldots,\,n_{n_\mathrm{nf}}]\tp$node numbers which provide the coordinates for the object (consecutively as provided in this list); the $(n_\mathrm{nf}+1)$ nodes represent the nodes of the FE mesh (except for node 0); the global nodal position needs to be reconstructed from the rigid-body motion of the reference frame"""),
        ItemParameter(type=TPyMatrixContainer, destination=DestComp+DestParam,
            pythonName='massMatrixFF',
            defaultValue='PyMatrixContainer()',
            description=r"""$\LU{b}{\Mm} \in \Rcal^{n\indf \times n\indf}$body-fixed and ONLY flexible coordinates part of mass matrix of object given in Python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format"""),
        ItemParameter(type=TPyMatrixContainer, destination=DestComp+DestParam,
            pythonName='stiffnessMatrixFF',
            defaultValue='PyMatrixContainer()',
            description=r"""$\LU{b}{\Km} \in \Rcal^{n\indf \times n\indf}$body-fixed and ONLY flexible coordinates part of stiffness matrix of object in Python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format"""),
        ItemParameter(type=TPyMatrixContainer, destination=DestComp+DestParam,
            pythonName='dampingMatrixFF',
            defaultValue='PyMatrixContainer()',
            description=r"""$\LU{b}{\Dm} \in \Rcal^{n\indf \times n\indf}$body-fixed and ONLY flexible coordinates part of damping matrix of object in Python numpy format (sparse (CSR) or dense, converted to sparse matrix); internally data is stored in triplet format"""),
        ItemParameter(type=TNumpyVector, destination=DestComp+DestParam,
            pythonName='forceVector',
            defaultValue='Vector()',
            description=r"""$\LU{0}{\fv} = [\LU{0}{\fv\indr},\; \LU{0}{\fv\indf}]\tp \in \Rcal^{n_c}$generalized, force vector added to RHS; the rigid body part $\fv_r$ is directly applied to rigid body coordinates while the flexible part $\fv\indf$ is transformed from global to local coordinates; note that this force vector only allows to add gravity forces for bodies with ABRV:COM at the origin of the reference frame"""),
        ItemParameter(type=TPyFunctionVectorMbsScalarIndex2Vector, destination=DestComp+DestParam,
            pythonName='forceUserFunction',
            defaultValue=0,
            description=r"""$\fv_{user} =  [\LU{0}{\fv_{\mathrm{r},user}},\; \LU{b}{\fv_{\mathrm{f},user}}]\tp \in \Rcal^{n_c}$A Python user function which computes the generalized user force vector for the ABRV:ODE2 equations; note the different coordinate systems for rigid body and flexible part; The function args are mbs, time, objectNumber, coordinates q (without reference values) and coordinate velocities q\_t; see description below"""),
        ItemParameter(type=TPyFunctionMatrixMbsScalarIndex2Vector, destination=DestComp+DestParam,
            pythonName='massMatrixUserFunction',
            defaultValue=0,
            description=r"""$\Mm_{user} \in \Rcal^{n_c\times n_c}$A Python user function which computes the TOTAL mass matrix (including reference node) and adds the local constant mass matrix; note the different coordinate systems as described in the ABRV:FFRF mass matrix; see description below"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='computeFFRFterms',
            defaultValue=True,
            description=r"""flag decides whether the standard ABRV:FFRF terms are computed; use this flag for user-defined definition of ABRV:FFRF terms in mass matrix and quadratic velocity vector"""),
        ItemParameter(type=TArrayIndex, destination=DestComp, cFlags=CFReadOnly,
            pythonName='coordinateIndexPerNode',
            defaultValue='ArrayIndex()',
            description=r'this list contains the local coordinate index for every node, which is needed, e.g., for markers; the list is generated automatically every time parameters have been changed'),
        ItemParameter(type=TBool, destination=DestComp,
            pythonName='objectIsInitialized',
            defaultValue=False,
            description=r"""ALWAYS set to False! flag used to correctly initialize all ABRV:FFRF matrices; as soon as this flag is False, internal (constant) ABRV:FFRF matrices are recomputed during Assemble()"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp, cFlags=CFReadOnly,
            pythonName='physicsMass',
            defaultValue=0.,
            description=r"""$m$total mass [SI:kg] of ABRV:FFRF object, auto-computed from mass matrix $\LU{b}{\Mm}$"""),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp, cFlags=CFReadOnly,
            pythonName='physicsInertia',
            defaultValue='EXUmath::unitMatrix3D',
            description=r"""$J_r \in \Rcal^{3 \times 3}$inertia tensor [SI:kgm$^2$] of rigid body w.r.t. to the reference point of the body, auto-computed from the mass matrix $\LU{b}{\Mm}$"""),
        ItemParameter(type=TVectorND(3), destination=DestComp, cFlags=CFReadOnly,
            pythonName='physicsCenterOfMass',
            defaultValue=DVZeroVector3D,
            description=r"""$\LU{b}{\bv}_{COM}$local position of center of mass (ABRV:COM); auto-computed from mass matrix $\LU{b}{\Mm}$"""),
        ItemParameter(type=TNumpyMatrix, destination=DestComp, cFlags=CFReadOnly,
            pythonName='PHItTM',
            defaultValue='Matrix()',
            description=r"""$\tPhi\indt\tp \in \Rcal^{n\indf \times 3}$projector matrix; may be removed in future"""),
        ItemParameter(type=TNumpyVector, destination=DestComp, cFlags=CFReadOnly,
            pythonName='referencePositions',
            defaultValue='Vector()',
            description=r"""$\xv\cRef \in \Rcal^{n\indf}$vector containing the reference positions of all flexible nodes"""),
        ItemParameter(type=TNumpyVector, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='tempVector',
            defaultValue='Vector()',
            description=r'$\vv_{temp} \in \Rcal^{n\indf}$temporary vector'),
        ItemParameter(type=TNumpyVector, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='tempCoordinates',
            defaultValue='Vector()',
            description=r"""$\cv_{temp} \in \Rcal^{n\indf}$temporary vector containing coordinates"""),
        ItemParameter(type=TNumpyVector, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='tempCoordinates_t',
            defaultValue='Vector()',
            description=r"""$\dot \cv_{temp} \in \Rcal^{n\indf}$temporary vector containing velocity coordinates"""),
        ItemParameter(type=TNumpyMatrix, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='tempRefPosSkew',
            defaultValue='Matrix()',
            description=r"""$\tilde\pv\indf \in \Rcal^{n\indf \times 3}$temporary matrix with skew symmetric local (deformed) node positions"""),
        ItemParameter(type=TNumpyMatrix, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='tempVelSkew',
            defaultValue='Matrix()',
            description=r"""$\dot{\tilde\cv}\indf \in \Rcal^{n\indf \times 3}$temporary matrix with skew symmetric local node velocities"""),
        ItemParameter(type=TResizableMatrix, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='tempMatrix',
            defaultValue='ResizableMatrix()',
            description=r'$\Xm_{temp} \in \Rcal^{n\indf \times 3}$temporary matrix'),
        ItemParameter(type=TResizableMatrix, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='tempMatrix2',
            defaultValue='ResizableMatrix()',
            description=r"""$\Xm_{temp2} \in \Rcal^{n\indf \times 4}$other temporary matrix"""),
        ItemFunctionDef('HasUserFunction',
            destination=DestComp,
            implementation='return (parameters.forceUserFunction!=0) || (parameters.massMatrixUserFunction!=0);'),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'AngularVelocity_qt', 'DisplacementMassIntegral_q', 'SuperElement']),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetOutputVariableBody'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetDisplacement'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetRotationMatrix',
            description='return configuration dependent rotation matrix of node; returns always a 3D Matrix, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetAngularVelocityLocal',
            description='return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunctionDef('GetLocalCenterOfMass',
            implementation='return physicsCenterOfMass;',
            description='return the local position of the center of mass, needed for massProportionalLoad; this is only the reference-frame part!'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "FFRF";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation='return parameters.nodeNumbers[localIndex];'),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumbers[localIndex]=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return parameters.nodeNumbers.NumberOfItems();'),
        ItemFunctionDef('GetODE2Size'),
        ItemRequestedTypes('Node', []),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded + (Index)CObjectType::SuperElement);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='return false;'),
        ItemFunctionDef('ParametersHaveChanged',
            implementation='objectIsInitialized = false;',
            description='This flag is reset upon change of parameters; says that the vector of coordinate indices has changed'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('PostAssemble',
            implementation='InitializeObject();'),
        ItemFunctionDef('GetLocalODE2CoordinateIndexPerNode',
            implementation='return coordinateIndexPerNode[localNode];'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeObjectCoordinates',
            args='Vector& coordinates, Vector& coordinates_t, ConfigurationType configuration = ConfigurationType::Current',
            description=r'compute object coordinates composed from all nodal coordinates; does not include reference coordinates'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeObjectCoordinates_tt',
            args='Vector& coordinates_tt, ConfigurationType configuration = ConfigurationType::Current',
            description=r'compute object acceleration coordinates composed from all nodal coordinates'),
        ItemFunction(type=Tvoid, destination=DestComp, isVirtual=False,
            pythonName='InitializeObject',
            description=r'initialize coordinateIndexPerNode array'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionForce',
            args='Vector& force, const MainSystemBase& mainSystem, Real t, Index objectNumber, const StdVector& coordinates, const StdVector& coordinates_t',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionMassMatrix',
            args='Matrix& massMatrix, const MainSystemBase& mainSystem, Real t, Index objectNumber, const StdVector& coordinates, const StdVector& coordinates_t',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunctionDef('HasReferenceFrame',
            implementation='localReferenceFrameNode = rigidBodyNodeNumber; return true;',
            description='always true, because ObjectFFRF; return according LOCAL node number'),
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst,
            pythonName='GetNumberOfMeshNodes',
            implementation='return GetNumberOfNodes()-1;',
            description=r'return the number of mesh nodes, which is 1 less than the number of nodes (but different in other SuperElements)'),
        ItemFunctionDef('GetMeshNode'),
        ItemFunctionDef('GetMeshNodeLocalPosition'),
        ItemFunctionDef('GetMeshNodeLocalVelocity'),
        ItemFunctionDef('GetMeshNodeLocalAcceleration'),
        ItemFunctionDef('GetMeshNodePosition'),
        ItemFunctionDef('GetMeshNodeVelocity'),
        ItemFunctionDef('GetMeshNodeAcceleration'),
        ItemFunctionDef('GetAccessFunctionSuperElement'),
        ItemFunctionDef('GetOutputVariableTypesSuperElement'),
        ItemFunctionDef('GetOutputVariableSuperElement'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown; use visualizationSettings.bodies.deformationScaleFactor to draw scaled (local) deformations; the reference frame node is shown with additional letters RF'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used'),
        ItemParameter(type=TNumpyMatrixI, destination=DestVisu,
            pythonName='triangleMesh',
            defaultValue='MatrixI()',
            description=r'a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!'),
        ItemParameter(type=TBool, destination=DestVisu,
            pythonName='showNodes',
            defaultValue=False,
            description=r"set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'"),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectFFRFreducedOrder   ++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectFFRFreducedOrder',
    addIncludesC=r"""#include <pybind11/numpy.h>//for NumpyMatrix
#include <pybind11/stl.h>//for NumpyMatrix
#include <pybind11/pybind11.h>
typedef py::array_t<Real> NumpyMatrix; 
#include "Pymodules/PyMatrixContainer.h"//for some ABRV:FFRF matrices
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    addPublicC=r"""    static constexpr Index ffrfNodeDim = 3; //dimension of nodes (=displacement coordinates per node)
    static constexpr Index rigidBodyNodeNumber = 0; //node number of rigid body node (usually = 0)
    static constexpr Index genericNodeNumber = 1;//node number for modal coordinates
""",
    author=r'Gerstmayr Johannes, Zwölfer Andreas',
    cParentClass=ParentClassCObjectSuperElement,
    classDescription=r"""This object is used to represent modally reduced flexible bodies using the ABRV:FFRF and the ABRV:CMS. It can be used to model real-life mechanical systems imported from finite element codes or Python tools such as NETGEN/NGsolve, see the \texttt{FEMinterface} in [](#sec-fem-feminterface---init--). It contains a RigidBodyNode (always node 0) and a NodeGenericODE2 representing the modal coordinates. Currently, equations must be defined within user functions, which are available in the FEM module, see class \texttt{ObjectFFRFreducedOrderInterface}, especially the user functions \texttt{UFmassFFRFreducedOrder} and \texttt{UFforceFFRFreducedOrder}, [](#sec-fem-objectffrfreducedorderinterface-addobjectffrfreducedorderwithuserfunctions).""",
    classType=ClassTypeObject,
    equations=r"""    <!--+++++++++++++++++++++++++++++++++++++ -->

    (sec-objectffrfreducedorder-superelementoutput)=
    #### Super element output variables

    Functions like \texttt{GetObjectOutputSuperElement(...)}, see [](#sec-mainsystem-object), 
    or \texttt{SensorSuperElement}, see [](#sec-mainsystem-sensor), directly access special output variables
    (\texttt{OutputVariableType}) of the mesh nodes of the superelement.
    Additionally, the contour drawing of the object can make use the \texttt{OutputVariableType} of the meshnodes.
    <!--
    +++++++++++++++++++++++++++++++++++++++++++++++++++
    #### Definition of quantities
    The object additionally provides the following output variables for mesh nodes (use \texttt{mbs.GetObjectOutputSuperElement(...)} or \texttt{SensorSuperElement}):
    -->

    | super element output variables | symbol | description |
    |---|---|---|
    | DisplacementLocal (mesh node $i$) | $\LU{b}{\uv\indf^{(i)}} = \left( \LU{b}{\tPsi} \tzeta\right)_{3\cdot i \ldots 3\cdot i+2}= \vr{\LU{b}{\qv_{\mathrm{f},i\cdot 3}}}{\LU{b}{\qv_{\mathrm{f},i\cdot 3+1}}}{\LU{b}{\qv_{\mathrm{f},i\cdot 3+2}}}$ | local nodal mesh displacement in reference (body) frame, measuring only flexible part of displacement |
    | VelocityLocal (mesh node $(i)$) | $\LU{b}{\dot \uv_\mathrm{f}^{(i)}} = \left( \LU{b}{\tPsi} \dot \tzeta\right)_{3\cdot i \ldots 3\cdot i+2}$ | local nodal mesh velocity in reference (body) frame, only for flexible part of displacement |
    | Displacement (mesh node $(i)$) | $\LU{0}{\uv\cConfig^{(i)}} = \LU{0}{\qv_{\mathrm{t,config}}} + \LU{0b}{\Am_\mathrm{config}} \LU{b}{\pv_\mathrm{f,config}^{(i)}} - (\LU{0}{\qv_{\mathrm{t,ref}}} + \LU{0b}{\Am_{ref}} \LU{b}{\xv\cRef^{(i)}})$ | nodal mesh displacement in global coordinates |
    | Position (mesh node $(i)$) | $\LU{0}{\pv^{(i)}} = \LU{0}{\pRef} + \LU{0b}{\Am} \LU{b}{\pv\indf^{(i)}}$ | nodal mesh position in global coordinates |
    | Velocity (mesh node $(i)$) | $\LU{0}{\dot \uv^{(i)}} = \LU{0}{\dot \qv\indt} + \LU{0b}{\Am} (\LU{b}{\dot \uv\indf^{(i)}} + \LU{b}{\tilde \tomega} \LU{b}{\pv\indf^{(i)}})$ | nodal mesh velocity in global coordinates |
    | Acceleration (mesh node $(i)$) | $\LU{0}{\av^{(i)}} = \LU{0}{\ddot \qv\indt} + 
                                                            \LU{0b}{\Rot} \LU{b}{\ddot \uv\indf^{(i)}} + 
                                                            2\LU{0}{\tomega} \times \LU{0b}{\Rot} \LU{b}{\dot \uv\indf^{(i)}} +
                                                            \LU{0}{\talpha} \times \LU{0}{\pv\indf^{(i)}} + 
                                                            \LU{0}{\tomega} \times (\LU{0}{\tomega} \times \LU{0}{\pv\indf^{(i)}})$ | global acceleration of mesh node $n_i$ including rigid body motion and flexible deformation; note that $\LU{0}{\xv}(n_i) = \LU{0b}{\Rot} \LU{b}{\xv}(n_i)$ |
    | StressLocal (mesh node $(i)$) | $\LU{b}{\tsigma^{(i)}} = (\LU{b}{\tPsi_{OV}} \tzeta)_{3\cdot i \ldots 3\cdot i+5}$ | linearized stress components of mesh node $(i)$ in reference frame; $\tsigma=[\sigma_{xx},\,\sigma_{yy},\,\sigma_{zz},\,\sigma_{yz},\,\sigma_{xz},\,\sigma_{xy}]\tp$; ONLY available, if $\LU{b}{\tPsi}_{OV}$ is provided and \texttt{outputVariableTypeModeBasis== exu.OutputVariableType.StressLocal} |
    | StrainLocal (mesh node $(i)$) | $\LU{b}{\teps^{(i)}} = (\LU{b}{\tPsi}_{OV} \tzeta)_{3\cdot i \ldots 3\cdot i+5}$ | linearized strain components of mesh node $(i)$ in reference frame; $\teps=[\varepsilon_{xx},\,\varepsilon_{yy},\,\varepsilon_{zz},\,\varepsilon_{yz},\,\varepsilon_{xz},\,\varepsilon_{xy}]\tp$; ONLY available, if $\LU{b}{\tPsi}_{OV}$ is provided and \texttt{outputVariableTypeModeBasis== exu.OutputVariableType.StrainLocal} |

    <!--
    
    +++++++++++++++++++++++++++++++++++++++++++++++++++
    \rowTable{local mesh displacements}{$\LU{b}{\uv\indf^{(i)}} = \vr{\LU{b}{\qv}_{\mathrm{f},i\cdot 3}}{\LU{b}{\qv}_{\mathrm{f},i\cdot 3+1}}{\LU{b}{\qv}_{\mathrm{f},i\cdot 3+2}}$}{nodal mesh displacement in local coordinates (body frame)}
    \rowTable{local mesh position}{$\LU{b}{\pv\indf^{(i)}} = \vr{\LU{b}{\qv_{\mathrm{f},i\cdot 3}}}{\LU{b}{\qv_{\mathrm{f},i\cdot 3+1}}}{\LU{b}{\qv_{\mathrm{f},i\cdot 3+2}}} + \vr{\LU{b}{\xv_{\mathrm{ref},i\cdot 3}}}{\LU{b}{\xv_{\mathrm{ref},i\cdot 3+1}}}{\LU{b}{\xv_{\mathrm{ref},i\cdot 3+2}}}$}{(deformed) nodal mesh position in local coordinates (body frame)}
    -->

    | intermediate variables | symbol | description |
    |---|---|---|
    | reference frame | $b$ | the body-fixed / local frame is always denoted by $b$ |
    | number of rigid body coordinates | $n\indrigid$ | number of rigid body node coordinates: 6 in case of Euler angles (not fully available for ObjectFFRFreducedOrder) and 7 in case of Euler parameters |
    | number of flexible / mesh coordinates | $n\indf = 3 \cdot n_n$ | with number of nodes $n_n$; relevant for visualization |
    | number of modal coordinates | $n_m \ll n\indf$ | the number of reduced or modal coordinates, computed from number of columns given in \texttt{modeBasis} |
    | total number object coordinates | $n_{ODE2} = n_m + n_{rigid}$ |  |
    | reference frame origin | $\LU{0}{\pRef} = \LU{0}{\qv_{\mathrm{t}}} + \LU{0}{\qv_{\mathrm{t,ref}}}$ | reference frame position (origin) |
    | reference frame rotation | $\ttheta\cConfig = \ttheta\cConfig + \ttheta_{ref}$ | reference frame rotation parameters in any configuration except reference |
    | reference frame orientation | $\LU{0b}{\Rot}\cConfig = \LU{0b}{\Rot}\cConfig(\ttheta\cConfig)$ | transformation matrix for transformation of local (reference frame) to global coordinates, given by underlying rigid body node $n_0$ |
    | local vector of flexible coordinates | $\LU{b}{\qv\indf} = \LU{b}{\tPsi} \tzeta$ | represents mesh displacements; vector of alternating x,y, an z coordinates of local (in body frame) mesh displacements reconstructed from modal coordinates $\tzeta$; only evaluated for selected node points (e.g., sensors) during computation; corresponds to same vector in \texttt{ObjectFFRF} |
    | local nodal positions | $\LU{b}{\pv\indf} = \LU{b}{\qv\indf} + \LU{b}{\xv\cRef}$ | vector of all body-fixed nodal positions including flexible part; only evaluated for selected node points during computation |
    | local position of node (i) | $\LU{b}{\pv\indf^{(i)}} = \LU{b}{\uv\indf^{(i)}} + \LU{b}{\xv^{(i)}\cRef} = \vr{\LU{b}{\qv_{\mathrm{f},i\cdot 3}}}{\LU{b}{\qv_{\mathrm{f},i\cdot 3+1}}}{\LU{b}{\qv_{\mathrm{f},i\cdot 3+2}}} + \vr{\LU{b}{\xv_{\mathrm{ref},i\cdot 3}}}{\LU{b}{\xv_{\mathrm{ref},i\cdot 3+1}}}{\LU{b}{\xv_{\mathrm{ref},i\cdot 3+2}}}$ | body-fixed, deformed nodal mesh position (including flexible part) |
    | vector of modal coordinates | $\tzeta = [\zeta_0,\,\ldots,\zeta_{n_m-1}]\tp$ | vector of modal or reduced coordinates; these coordinates can either represent amplitudes of eigenmodes, static modes or general modes, depending on your mode basis |
    | coordinate vector | $\qv = [\LU{0}{\qv\indt},\,\tpsi,\,\tzeta]$ | vector of object coordinates; $\qv\indt$ and $\tpsi$ are the translation and rotation part of displacements of the reference frame, provided by the rigid body node (node number 0) |
    | flexible coordinates transformation matrix | $\LU{0b}{\Am_{bd}} = \mathrm{diag}([\LU{0b}{\Am},\;\ldots,\;\LU{0b}{\Am}])$ | block diagonal transformation matrix, which transforms all flexible coordinates from local to global coordinates |

    <!-- -->

    #### Modal reduction and reduced inertia matrices

    The formulation is based on the EOM of \texttt{ObjectFFRF}, {\bf also regarding parts of notation} 
    and some input parameters, [](#sec-item-objectffrf), and 
    can be found in Zwölfer and Gerstmayr [CITE:ZwoelferGerstmayr2021] with only small modifications in the notation.
    The notation of kinematics quantities follows the floating frame of reference idea with
    quantities given in the tables above and sketched in [](#fig-objectffrfreducedorder-mesh).
    <!--++++++++++++++++++++++++ -->
    

    (fig-objectffrfreducedorder-mesh)=
    ```{figure} /docs/figures/ObjectFFRFsketch.png
    :width: 400

    Floating frame of reference with exemplary position of a mesh node *i*
    ```

    <!--++++++++++++++++++++++++ -->

                       
    The reduced order ABRV:FFRF formulation is based on an approximation of flexible coordinates $\LU{b}{\qv\indf}$ 
    by means of a reduction or mode basis $\LU{b}{\tPsi}$ (\texttt{modeBasis}) and the the modal coordinates $\tzeta$,

    $$
    \LU{b}{\qv\indf} \approx \LU{b}{\tPsi} \tzeta
    $$

    The mode basis $\LU{b}{\tPsi}$ contains so-called mode shape vectors in its columns, which may be computed from eigen analysis, static computation or more advanced techniques, 
    see the helper functions in module \texttt{exudyn.FEM}, within the class \text{FEMinterface}.
    To compute eigen modes, use \texttt{FEMinterface.ComputeEigenmodes(...)} or
    \texttt{FEMinterface.ComputeHurtyCraigBamptonModes(...)}. For details on model order reduction and component mode synthesis, see [](#sec-theory-cms).
    In many applications, $n_m$ typically ranges between 10 and 50, but also beyond -- depending on the desired accuracy of the model.
    
    The \texttt{ObjectFFRF} coordinates and {eq}`eq-objectffrf-eom`\footnote{this is not done for user functions and \texttt{forceVector}} can be reduced by the matrix $\Hm \in \Rcal^{(n\indf+n\indrigid) \times n_{ODE2}}$,

    $$
    \qv_{FFRF} = \vr{\qv\indt}{\ttheta}{\LU{b}{\qv\indf}} = \mr{\ImThree}{\Null}{\Null} {\Null}{\Im\indr}{\Null} {\Null}{\Null}{\LU{b}{\tPsi}} \vr{\qv\indt}{\ttheta}{\tzeta}
            = \Hm \, \qv
    $$

    with the $4\times 4$ identity matrix $\Im\indr$ in case of Euler parameters and the reduced coordinates $\qv$.
    
    The reduced equations follow from the reduction of system matrices in {eq}`eq-objectffrf-eom`,

    $$
    \begin{aligned}
    \Km\indred &= \LU{b}{\tPsi}\tp \LU{b}{\Km} \LU{b}{\tPsi} \, , \\
          \Mm\indred &= \LU{b}{\tPsi}\tp \LU{b}{\Mm} \LU{b}{\tPsi} \, , \\
    \end{aligned}
    $$

    the computation of rigid body inertia

    $$
    \begin{aligned}
    \LU{b}{\tTheta}\indu &= \LUX{b}{\tilde \xv}{\cRef\tp} \LU{b}{\Mm} \LU{b}{\tilde \xv\cRef}\\
    \end{aligned}
    $$

    the center of mass (and according tilde matrix), using $\tPhi\indt$ from {eq}`eq-objectffrf-phit`,

    $$
    \begin{aligned}
    \LU{b}{\tchi}\indu &= \frac{1}{m} \tPhi\tp\indt \LU{b}{\Mm} \LU{b}{\xv\cRef}\\
          \LU{b}{\tilde \tchi\indu} &= \frac{1}{m} \tPhi\tp\indt \LU{b}{\Mm} \LU{b}{\tilde \xv\cRef}\\
    \end{aligned}
    $$
 
    and seven inertia-like matrices [CITE:ZwoelferGerstmayr2021],

    $$
    \Mm_{AB} = \Am\tp \LU{b}{\Mm} \Bm, \quad \mathrm{using} \quad \Am\Bm \in \left[\tPsi\tPsi ,\; \widetilde{\tPsi}\tPsi,\; \widetilde{\tPsi}\widetilde{\tPsi},\; 
            \tPhi\indt\tPsi,\; \tPhi\indt\widetilde{\tPsi},\; \tilde\xv\cRef\tPsi,\; \tilde\xv\cRef\widetilde{\tPsi}\right]
    $$

    Note that the special tilde operator for vectors $\pv \in \Rcal^{n_f}$ of {eq}`eq-objectffrf-specialtilde` is frequently used.
    
    
    <!--
    +++++++++++++++++++++++++
    +++++++++++++++++++++++++
    +++++++++++++++++++++++++
    +++++++++++++++++++++++++
    -->

    #### Equations of motion

    Equations of motion, in case that \texttt{computeFFRFterms = True}:

    $$
    \begin{aligned}
    \left(\Mm_{user}(mbs, t,\qv,\dot \qv) + 
                        \mr{\Mm\indtt}{\Mm\indtr}{\Mm\indtf} {}{\Mm\indrr}{\Mm\indrf} {\mathrm{sym.}}{}{\Mm\indff} \right) \ddot \qv + 
                        \mr{0}{0}{0} {0}{0}{0} {0}{0}{\Dm\indff} \dot \qv + \mr{0}{0}{0} {0}{0}{0} {0}{0}{\Km\indff} \qv = &&\\ \fv_v(\qv,\dot \qv) + \fv_{user}(mbs, t,\qv,\dot \qv) &&
    \end{aligned}
    $$

    \footnote{NOTE that currently the internal (C++) computed terms are zero,

    $$
    \mr{\Mm\indtt}{\Mm\indtr}{\Mm\indtf} {}{\Mm\indrr}{\Mm\indrf} {\mathrm{sym.}}{}{\Mm\indff} = \Null \quad \mathrm{and} \quad
            \fv_v(\qv,\dot \qv) = \Null \, ,
    $$

    but they are implemented in predefined user functions, see \texttt{FEM.py}, [](#sec-fem-objectffrfreducedorderinterface-addobjectffrfreducedorderwithuserfunctions). In near future, these terms will be implemented in C++ and replace the user functions.}
    <!-- -->
    Note that in case of Euler parameters for the parameterization of rotations for the reference frame, the Euler parameter constraint equation is added automatically by this object.
    <!-- -->
    The single terms of the mass matrix are defined as[CITE:ZwoelferGerstmayr2021]

    $$
    \begin{aligned}
    \Mm\indtt &= m \ImThree \\
          \Mm\indtr &= -\LU{0b}{\Rot} \left[ m \LU{b}{\tilde \tchi\indu} + \Mm_{\Phi\indt\!{\widetilde\Psi}} 
                          \left( \tzeta \otimes \Im \right)  \right] \LU{b}{\Gm}\\
          \Mm\indtf &= \LU{0b}{\Rot} \Mm_{\Phi\indt\!\Psi} \\
          \Mm\indrr &= \LU{b}{\Gm\tp} \left[\LU{b}{\tTheta}\indu + 
                                              \Mm_{\tilde \xv\cRef{\widetilde\Psi}} \left( \tzeta \otimes \Im \right) +
                                                                                \left( \tzeta \otimes \Im \right)\tp \Mm_{\tilde \xv\cRef{\widetilde\Psi}}\tp +
                                                                                \left( \tzeta \otimes \Im \right)\tp \Mm_{{\widetilde\Psi}{\widetilde\Psi}}\left( \tzeta \otimes \Im \right)
                                                                                \right] \LU{b}{\Gm}\\
          \Mm\indrf &= -\LU{b}{\Gm\tp} \left[ \Mm_{\tilde \xv\cRef\Psi} + \left( \tzeta \otimes \Im \right)\tp \Mm_{{\widetilde\Psi}\Psi}  \right] \\ 
          \Mm\indff &= \Mm_{\Psi\Psi}
    \end{aligned}
    $$

    with the Kronecker product\footnote{In Python numpy module this is computed by \texttt{numpy.kron(zeta, Im).T}},

    $$
    \tzeta \otimes \Im = \vr{\zeta_0 \Im}{\vdots}{\zeta_{m-1} \Im}
    $$

    The quadratic velocity vector $\fv_v(\qv,\dot \qv) = \left[ \fv_{v\mathrm{t}}\tp,\; \fv_{v\mathrm{r}}\tp,\; \fv_{v\mathrm{f}}\tp \right]\tp$ reads

    $$
    \begin{aligned}
    \fv_{v\mathrm{t}} &= \LU{0b}{\Rot} \LU{b}{\tilde \tomega}\left[ m \LU{b}{\tilde \tchi\indu} + \Mm_{\Phi\indt\!{\widetilde\Psi}} 
                          \left( \tzeta \otimes \Im \right)  \right] \LU{b}{\tomega} + 
                                        2 \LU{0b}{\Rot} \Mm_{\Phi\indt\!{\widetilde\Psi}} \left( \dot \tzeta \otimes \Im \right)  \LU{b}{\tomega} \\
                                    && + \LU{0b}{\Rot} \left[ m \LU{b}{\tilde \tchi\indu} + \Mm_{\Phi\indt\!{\widetilde\Psi}} 
                          \left( \tzeta \otimes \Im \right)  \right] \LU{b}{\dot \Gm} \dot \ttheta \, , \\
            \fv_{v\mathrm{r}} &= -\LU{b}{\Gm\tp} \LU{b}{\tilde \tomega} \left[\LU{b}{\tTheta}\indu + 
                                              \Mm_{\tilde \xv\cRef{\widetilde\Psi}} \left( \tzeta \otimes \Im \right) +
                                                                                \left( \tzeta \otimes \Im \right)\tp \Mm_{\tilde \xv\cRef{\widetilde\Psi}}\tp +
                                                                                \left( \tzeta \otimes \Im \right)\tp \Mm_{{\widetilde\Psi}{\widetilde\Psi}}\left( \tzeta \otimes \Im \right)
                                                                                \right]\LU{b}{\tomega} \\
                                                && -2 \LU{b}{\Gm\tp} \left[ \Mm_{\tilde \xv\cRef{\widetilde\Psi}} \left( \dot \tzeta \otimes \Im \right) +
                                                                                            \left( \tzeta \otimes \Im \right)\tp \Mm_{{\widetilde\Psi}{\widetilde\Psi}}\left( \dot \tzeta \otimes \Im \right)
                                                                     \right] \LU{b}{\tomega} \\
                                                && -\LU{b}{\Gm\tp}\left[\LU{b}{\tTheta}\indu + 
                                              \Mm_{\tilde \xv\cRef{\widetilde\Psi}} \left( \tzeta \otimes \Im \right) +
                                                                                \left( \tzeta \otimes \Im \right)\tp \Mm_{\tilde \xv\cRef{\widetilde\Psi}}\tp +
                                                                                \left( \tzeta \otimes \Im \right)\tp \Mm_{{\widetilde\Psi}{\widetilde\Psi}}\left( \tzeta \otimes \Im \right)
                                                                                \right] \LU{b}{\dot \Gm} \dot \ttheta \, , \\
            \fv_{v\mathrm{f}} &= \left( \Im_\zeta \otimes \LU{b}{\tomega} \right)\tp 
                                        \left[ \Mm_{\tilde\xv\cRef{\widetilde\Psi}}\tp + \Mm_{{\widetilde\Psi}{\widetilde\Psi}}\left( \tzeta \otimes \Im \right) \right] \LU{b}{\tomega}
                                                                    +2 \Mm_{{\widetilde\Psi}{\Psi}}\tp\left( \dot\tzeta \otimes \Im \right) \LU{b}{\tomega} \\
                                                && + \left[ \Mm_{\tilde\xv\cRef{\Psi}}\tp + \Mm_{{\widetilde\Psi}{\Psi}}\tp\left( \tzeta \otimes \Im \right)
                                                     \right] \LU{b}{\dot \Gm} \dot \ttheta \, .
    \end{aligned}
    $$

    Note that terms including $\LU{b}{\dot \Gm} \dot \ttheta$ vanish in case of Euler parameters or in case that $\LU{b}{\dot \Gm} = \Null$,
    and we use another Kronecker product with the unit matrix $\Im_\zeta \in \Rcal^{n_m \times n_m}$,

    $$
    \Im_\zeta \otimes \LU{b}{\tomega} = \mr{\LU{b}{\tomega}}{}{} {}{\ddots}{} {}{}{\LU{b}{\tomega}} \in \Rcal^{3n_m \times n_m}
    $$

    
    <!--$\ra$ will be completed later, see according literature of Zwölfer and Gerstmayr [CITE:ZwoelferGerstmayr2021]. -->
    
    In case that \texttt{computeFFRFterms = False}, the mass terms $\Mm\indtt \ldots \Mm\indff$ are zero (not computed) and
    the quadratic velocity vector $\fv_Q = \Null$.
    Note that the user functions $\fv_{user}(mbs, t,\qv,\dot \qv)$ and 
    $\Mm_{user}(mbs, t,\qv,\dot \qv)$ may be empty (=0). 
    The detailed equations of motion for this element can be found in [CITE:ZwoelferGerstmayr2021].

    <!--
    +++++++++++++++++++++++++
    +++++++++++++++++++++++++
    -->

    #### Position Jacobian

    For joints and loads, the position jacobian of a node is needed in order to compute forces applied to averaged displacements and 
    rotations at nodes.
    Recall that the modal coordinates $\tzeta$ are transformed to node coordinates by means of the mode basis  $\LU{b}{\tPsi}$,

    $$
    \LU{b}{\qv\indf} = \LU{b}{\tPsi} \tzeta \, .
    $$

    The local displacements $\LU{b}{\uv\indf^{(i)}}$ of a specific node $i$ can be reconstructed in this way by means of

    $$
    \LU{b}{\uv\indf^{(i)}} = \vr{\LU{b}{\qv_{\mathrm{f},i\cdot 3}}}{\LU{b}{\qv_{\mathrm{f},i\cdot 3+1}}}{\LU{b}{\qv_{\mathrm{f},i\cdot 3+2}}} \, ,
    $$

    and the global position of a node, see tables above, reads

    $$
    \LU{0}{\pv^{(i)}} = \LU{0}{\pv\indt} + \LU{0b}{\Am} \left( \LU{b}{\uv\indf^{(i)}} + \LU{b}{\xv^{(i)}\cRef} \right)
    $$

    Thus, the jacobian of the global position reads

    $$
    \LU{0}{\Jm_\mathrm{pos}^{(i)}} = \frac{\partial \LU{0}{\pv^{(i)}}}{\partial [\qv\indt, \;\ttheta, \;\tzeta]}
         = \left[\ImThree, \; -\LU{0b}{\Rot} \left(\LU{b}{\tilde\uv\indf^{(i)}} + \LU{b}{\tilde\xv^{(i)}\cRef} \right) \LU{b}{\Gm},\;
                 \LU{0b}{\Rot} \vr{\LU{b}{\tPsi_{r=3i}\tp}}{\LU{b}{\tPsi_{r=3i+1}\tp}}{\LU{b}{\tPsi_{r=3i+2}\tp}}\right] \, ,
    $$

    in which $\LU{b}{\tPsi_{r=...}}$ represents the row $r$ of the mode basis (matrix) $\LU{b}{\Psi}$, and
    the matrix 

    $$
    \vr{\LU{b}{\tPsi_{r=3i}\tp}}{\LU{b}{\tPsi_{r=3i+1}\tp}}{\LU{b}{\tPsi_{r=3i+2}\tp}} \in \Rcal^{3 \times n_m}
    $$

    Furthermore, the jacobian of the local position reads

    $$
    \LU{b}{\Jm_\mathrm{pos}^{(i)}} = \frac{\partial \LU{b}{\pv\indf^{(i)}}}{\partial [\qv\indt, \;\ttheta, \;\tzeta]}
         = \left[\Null, \; \Null, \; \vr{\LU{b}{\tPsi_{r=3i}\tp}}{\LU{b}{\tPsi_{r=3i+1}\tp}}{\LU{b}{\tPsi_{r=3i+2}\tp}}\right] \, ,
    $$

    which is used in \texttt{MarkerSuperElementRigid}.
    
    
    <!--
    +++++++++++++++++++++++++
    +++++++++++++++++++++++++
    -->

    #### Joints and Loads

    Use special \texttt{MarkerSuperElementPosition} to apply forces, SpringDampers or spherical joints. This marker can be attached to a single node of the underlying
    mesh or to a set of nodes, which is then averaged, see the according marker description.
    
    Use special \texttt{MarkerSuperElementRigid} to apply torques or special joints (e.g., \texttt{JointGeneric}). 
    This marker must be attached to a set of nodes which can represent rigid body motion. The rigid body motion is then averaged for all of these nodes,
    see the according marker description.
    
    For application of mass proportional loads (gravity), you can use conventional MarkerBodyMass.
    However, {\bf do not use} \texttt{MarkerBodyPosition} or \texttt{MarkerBodyRigid} for ObjectFFRFreducedOrder, unless wanted, because it only attaches to the floating
    frame. This means, that a force to a \texttt{MarkerBodyPosition} would only be applied to the (rigid) floating frame, but not onto the deformable body and
    results depend strongly on the choice of the reference frame (or the underlying mode shapes).
    
    CoordinateLoads are added for each ABRV:ODE2 coordinate on the RHS of the equations of motion. 
    <!--++++++++++++++++++++++++++++++++++++++++ -->
    
    
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `forceUserFunction(mbs, t, itemNumber, q, q_t)`
    A user function, which computes a force vector depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.
    Note that itemNumber represents the index of the ObjectFFRFreducedOrder object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.
    <!-- -->

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs to which object belongs |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{q} | Vector $\in \Rcal^n_{ODE2}$ | ABRV:FFRF object coordinates (rigid body coordinates and reduced coordinates in a list) in current configuration, without reference values |
    | \texttt{q\_t} | Vector $\in \Rcal^n_{ODE2}$ | object velocity coordinates (time derivatives of \texttt{q}) in current configuration |
    | **return value** | Vector $\in \Rcal^{n_{ODE2}}$ | returns force vector for object |

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `massMatrixUserFunction(mbs, t, itemNumber, q, q_t)`
    A user function, which computes a mass matrix depending on current time and states of object. Can be used to create any kind of mechanical system by using the object states.

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs to which object belongs |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{q} | Vector $\in \Rcal^n_{ODE2}$ | ABRV:FFRF object coordinates (rigid body coordinates and reduced coordinates in a list) in current configuration, without reference values |
    | \texttt{q\_t} | Vector $\in \Rcal^n_{ODE2}$ | object velocity coordinates (time derivatives of \texttt{q}) in current configuration |
    | **return value** | NumpyMatrix $\in \Rcal^{n_{ODE2} \times n_{ODE2}}$ | returns mass matrix for object |

    \vspace{12pt}
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    objectType=ObjectTypeSuperElement,
    outputVariables=[
        ItemOutputVariable(OVCoordinates, r'all ABRV:ODE2 coordinates'),
        ItemOutputVariable(OVCoordinates_t, OVDVelocityCoordinatesODE2),
        ItemOutputVariable(OVForce, OVDGeneralizedForces),
        ],
    pythonShortName='CMSobject',
    visuParentClass=VisuParentClassVisualizationObjectSuperElement,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TArrayIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumbers',
            defaultValue='ArrayIndex()',
            description=r"""$\mathbf{n} = [n_0,\,n_1]\tp$node numbers of rigid body node and NodeGenericODE2 for modal coordinates; the global nodal position needs to be reconstructed from the rigid-body motion of the reference frame, the modal coordinates and the mode basis"""),
        ItemParameter(type=TPyMatrixContainer, destination=DestComp+DestParam,
            pythonName='massMatrixReduced',
            defaultValue='PyMatrixContainer()',
            description=r"""$\Mm\indred \in \Rcal^{n_m \times n_m}$body-fixed and ONLY flexible coordinates part of reduced mass matrix; provided as MatrixContainer(sparse/dense matrix)"""),
        ItemParameter(type=TPyMatrixContainer, destination=DestComp+DestParam,
            pythonName='stiffnessMatrixReduced',
            defaultValue='PyMatrixContainer()',
            description=r"""$\Km\indred \in \Rcal^{n_m \times n_m}$body-fixed and ONLY flexible coordinates part of reduced stiffness matrix; provided as MatrixContainer(sparse/dense matrix)"""),
        ItemParameter(type=TPyMatrixContainer, destination=DestComp+DestParam,
            pythonName='dampingMatrixReduced',
            defaultValue='PyMatrixContainer()',
            description=r"""$\Dm\indred \in \Rcal^{n_m \times n_m}$body-fixed and ONLY flexible coordinates part of reduced damping matrix; provided as MatrixContainer(sparse/dense matrix)"""),
        ItemParameter(type=TPyFunctionVectorMbsScalarIndex2Vector, destination=DestComp+DestParam,
            pythonName='forceUserFunction',
            defaultValue=0,
            description=r"""$\fv\induser \in \Rcal^{n_{ODE2}}$A Python user function which computes the generalized user force vector for the ABRV:ODE2 equations; see description below"""),
        ItemParameter(type=TPyFunctionMatrixMbsScalarIndex2Vector, destination=DestComp+DestParam,
            pythonName='massMatrixUserFunction',
            defaultValue=0,
            description=r"""$\Mm\induser \in \Rcal^{n_{ODE2}\times n_{ODE2}}$A Python user function which computes the TOTAL mass matrix (including reference node) and adds the local constant mass matrix; see description below"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='computeFFRFterms',
            defaultValue=True,
            description=r"""flag decides whether the standard ABRV:FFRF/ABRV:CMS terms are computed; use this flag for user-defined definition of ABRV:FFRF terms in mass matrix and quadratic velocity vector"""),
        ItemParameter(type=TNumpyMatrix, destination=DestComp+DestParam,
            pythonName='modeBasis',
            defaultValue='Matrix()',
            description=r"""$\LU{b}{\tPsi} \in \Rcal^{n\indf \times n_{m}}$mode basis, which transforms reduced coordinates to (full) nodal coordinates, written as a single vector $[u_{x,n_0},\,u_{y,n_0},\,u_{z,n_0},\,\ldots,\,u_{x,n_n},\,u_{y,n_n},\,u_{z,n_n}]\tp$"""),
        ItemParameter(type=TNumpyMatrix, destination=DestComp+DestParam,
            pythonName='outputVariableModeBasis',
            defaultValue='Matrix()',
            description=r"""$\LU{b}{\tPsi}_{OV} \in \Rcal^{n_n \times (n_{m}\cdot s_{OV})}$mode basis, which transforms reduced coordinates to output variables per mode and per node; $s_{OV}$ is the size of the output variable, e.g., 6 for stress modes ($S_{xx},...,S_{xy}$)"""),
        ItemParameter(type=TOutputVariableType, destination=DestComp+DestParam,
            pythonName='outputVariableTypeModeBasis',
            defaultValue='OutputVariableType::_None',
            description=r'this must be the output variable type of the outputVariableModeBasis, e.g. exu.OutputVariableType.Stress'),
        ItemParameter(type=TNumpyVector, destination=DestComp+DestParam,
            pythonName='referencePositions',
            defaultValue='Vector()',
            description=r"""$\LU{b}{\xv}\cRef \in \Rcal^{n\indf}$vector containing the reference positions of all flexible nodes, needed for graphics"""),
        ItemParameter(type=TBool, destination=DestComp,
            pythonName='objectIsInitialized',
            defaultValue=False,
            description=r"""ALWAYS set to False! flag used to correctly initialize all ABRV:FFRF matrices; as soon as this flag is False, some internal (constant) ABRV:FFRF matrices are recomputed during Assemble()"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp,
            pythonName='physicsMass',
            defaultValue=0.,
            description=r'$m$total mass [SI:kg] of FFRFreducedOrder object'),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp,
            pythonName='physicsInertia',
            defaultValue='EXUmath::unitMatrix3D',
            description=r"""$\Jm_r \in \Rcal^{3 \times 3}$inertia tensor [SI:kgm$^2$] of rigid body w.r.t. to the reference point of the body"""),
        ItemParameter(type=TVectorND(3), destination=DestComp,
            pythonName='physicsCenterOfMass',
            defaultValue=DVZeroVector3D,
            description=r"""$\LU{b}{\bv}_{COM}$local position of center of mass (ABRV:COM)"""),
        ItemParameter(type=TNumpyMatrix, destination=DestComp+DestParam,
            pythonName='mPsiTildePsi',
            defaultValue='Matrix()',
            description=r'special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface'),
        ItemParameter(type=TNumpyMatrix, destination=DestComp+DestParam,
            pythonName='mPsiTildePsiTilde',
            defaultValue='Matrix()',
            description=r'special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface'),
        ItemParameter(type=TNumpyMatrix, destination=DestComp+DestParam,
            pythonName='mPhitTPsi',
            defaultValue='Matrix()',
            description=r'special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface'),
        ItemParameter(type=TNumpyMatrix, destination=DestComp+DestParam,
            pythonName='mPhitTPsiTilde',
            defaultValue='Matrix()',
            description=r'special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface'),
        ItemParameter(type=TNumpyMatrix, destination=DestComp+DestParam,
            pythonName='mXRefTildePsi',
            defaultValue='Matrix()',
            description=r'special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface'),
        ItemParameter(type=TNumpyMatrix, destination=DestComp+DestParam,
            pythonName='mXRefTildePsiTilde',
            defaultValue='Matrix()',
            description=r'special FFRFreducedOrder matrix, computed in ObjectFFRFreducedOrderInterface'),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp,
            pythonName='physicsCenterOfMassTilde',
            defaultValue='EXUmath::zeroMatrix3D',
            description=r"""$\LU{b}{\tilde \bv}_{COM}$tilde matrix from local position of ABRV:COM; autocomputed during initialization"""),
        ItemParameter(type=TNumpyVector, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='tempUserFunctionForce',
            defaultValue='Vector()',
            description=r"""$\fv_{temp} \in \Rcal^{n_{ODE2}}$temporary vector for UF force"""),
        ItemParameter(type=TResizableVector, destination=DestComp, cFlags=CFMutable+CFReadOnly+CFNoInterface,
            pythonName='tempCoordinates',
            defaultValue='ResizableVector()',
            description=r"""$\pv_{temp} \in \Rcal^{n\indf}$temporary vector containing coordinates"""),
        ItemParameter(type=TResizableVector, destination=DestComp, cFlags=CFMutable+CFReadOnly+CFNoInterface,
            pythonName='tempCoordinates_t',
            defaultValue='ResizableVector()',
            description=r"""$\dot \pv_{temp} \in \Rcal^{n\indf}$temporary vector containing velocity coordinates"""),
        ItemParameter(type=TResizableMatrix, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='tempKronZetaI',
            defaultValue='ResizableMatrix()',
            description=r"""$(\tzeta \otimes \Im) \in \Rcal^{n\indf \times 3}$temporary coordinate dependent matrix"""),
        ItemParameter(type=TResizableMatrix, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='tempKronZetaI_t',
            defaultValue='ResizableMatrix()',
            description=r"""$(\tzeta \otimes \Im) \in \Rcal^{n\indf \times 3}$temporary coordinate dependent matrix"""),
        ItemParameter(type=TResizableMatrix, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='tempKronIZetaOmegaT',
            defaultValue='ResizableMatrix()',
            description=r"""$(\Im_zeta \otimes \tomega)^T \in \Rcal^{n\indf \times 3 n\indf}$temporary coordinate dependent matrix"""),
        ItemParameter(type=TResizableMatrix, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='tempMatrix',
            defaultValue='ResizableMatrix()',
            description=r"""$\Xm_{temp}$temporary matrix at several parts of computation MassMatrix, ODE2Lhs"""),
        ItemParameter(type=TResizableMatrix, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='tempMatrix2',
            defaultValue='ResizableMatrix()',
            description=r"""$\Xm_{temp2}$second temporary matrix at several parts of computation MassMatrix, ODE2Lhs"""),
        ItemParameter(type=TResizableVector, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='tempVector',
            defaultValue='ResizableVector()',
            description=r'$\vv_{temp}$temporary vector at computation of ODE2Lhs'),
        ItemParameter(type=TResizableVector, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='tempVector2',
            defaultValue='ResizableVector()',
            description=r"""$\vv_{temp2}$second temporary vector at computation of ODE2Lhs"""),
        ItemFunctionDef('HasUserFunction',
            destination=DestComp,
            implementation='return (parameters.forceUserFunction!=0) || (parameters.massMatrixUserFunction!=0);'),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'AngularVelocity_qt', 'DisplacementMassIntegral_q', 'SuperElement']),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetOutputVariableBody'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetDisplacement'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetRotationMatrix',
            description='return configuration dependent rotation matrix of node; returns always a 3D Matrix, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetAngularVelocityLocal',
            description='return configuration dependent local (=body-fixed) angular velocity of node; returns always a 3D Vector, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunctionDef('GetLocalCenterOfMass',
            implementation='return physicsCenterOfMass;',
            description='return the local position of the center of mass, needed for massProportionalLoad; this is only the reference-frame part!'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "FFRFreducedOrder";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation='return parameters.nodeNumbers[localIndex];'),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumbers[localIndex]=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return parameters.nodeNumbers.NumberOfItems();'),
        ItemFunctionDef('GetODE2Size'),
        ItemRequestedTypes('Node', []),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded + (Index)CObjectType::SuperElement);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='return false;'),
        ItemFunctionDef('ParametersHaveChanged',
            implementation='objectIsInitialized = false;',
            description='This flag is reset upon change of parameters; says that the vector of coordinate indices has changed'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('PostAssemble',
            implementation='InitializeObject();'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeObjectCoordinates',
            args='Vector& coordinates, ConfigurationType configuration = ConfigurationType::Current',
            description=r'compute object coordinates composed from all nodal coordinates; does not include reference coordinates'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeObjectCoordinates_t',
            args='Vector& coordinates_t, ConfigurationType configuration = ConfigurationType::Current',
            description=r'compute object velocity coordinates composed from all nodal coordinates'),
        ItemFunction(type=Tvoid, destination=DestComp, isVirtual=False,
            pythonName='InitializeObject',
            description=r'initialize FFRFreducedOrder matrices'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetMeshNodeCoordinates',
            args='Index nodeNumber, const Vector& coordinates',
            description=r'compute coordinates for nodeNumber (without reference coordinates) from modeBasis (=multiplication of according part of mode Basis with modal coordinates)'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionForce',
            args='Vector& force, const MainSystemBase& mainSystem, Real t, Index objectNumber, const StdVector& coordinates, const StdVector& coordinates_t',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionMassMatrix',
            args='Matrix& massMatrix, const MainSystemBase& mainSystem, Real t, Index objectNumber, const StdVector& coordinates, const StdVector& coordinates_t',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunctionDef('HasReferenceFrame',
            implementation='localReferenceFrameNode = rigidBodyNodeNumber; return true;',
            description=r"""always true, because ABRV:FFRF-based object; return according LOCAL node number"""),
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst,
            pythonName='GetNumberOfMeshNodes',
            implementation='return parameters.referencePositions.NumberOfItems()/3;',
            description=r'return the number of mesh nodes, which is given according to the node reference positions'),
        ItemFunctionDef('GetMeshNode'),
        ItemFunctionDef('GetMeshNodeLocalPosition'),
        ItemFunctionDef('GetMeshNodeLocalVelocity'),
        ItemFunctionDef('GetMeshNodeLocalAcceleration'),
        ItemFunctionDef('GetMeshNodePosition'),
        ItemFunctionDef('GetMeshNodeVelocity'),
        ItemFunctionDef('GetMeshNodeAcceleration'),
        ItemFunctionDef('GetAccessFunctionSuperElement'),
        ItemFunctionDef('GetOutputVariableTypesSuperElement'),
        ItemFunctionDef('GetOutputVariableSuperElement'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown; use visualizationSettings.bodies.deformationScaleFactor to draw scaled (local) deformations; the reference frame node is shown with additional letters RF'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA color for object; 4th value is alpha-transparency; R=-1.f means, that default color is used'),
        ItemParameter(type=TNumpyMatrixI, destination=DestVisu,
            pythonName='triangleMesh',
            defaultValue='MatrixI()',
            description=r'a matrix, containg node number triples in every row, referring to the node numbers of the GenericODE2 object; the mesh uses the nodes to visualize the underlying object; contour plot colors are still computed in the local frame!'),
        ItemParameter(type=TBool, destination=DestVisu,
            pythonName='showNodes',
            defaultValue=False,
            description=r"set true, nodes are drawn uniquely via the mesh, eventually using the floating reference frame, even in the visualization of the node is show=False; node numbers are shown with indicator 'NF'"),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectANCFCable   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectANCFCable',
    addProtectedC=r"""    mutable bool massMatrixComputed; //!< flag which shows that mass matrix has been computed; will be set to false at time when parameters are set
    mutable ConstSizeMatrix<12*12> precomputedMassMatrix; //!< if massMatrixComputed=true, this contains the (constant) mass matrix for faster computation
""",
    addPublicC=r"""    static constexpr Index nODE2coordinates = 12; //!< fixed size element coordinates used e.g. for ConstSizeVectors
    static constexpr Index nShapeFunctions = 4; //!< number of shape functions
    static constexpr Index nNodalCoordinates = 6; //!< number of nodal coordinates
""",
    cParentClass=ParentClassCObjectBody,
    classDescription=r"""A 3D cable finite element using 2 nodes of type NodePointSlope1. The localPosition of the beam with length $L$=physicsLength and height $h$ ranges in $X$-direction in range $[0, L]$ and in $Y$-direction in range $[-h/2,h/2]$ (which is in fact not needed in the ABRV:EOM). For description see ObjectANCFCable2D, which is almost identical to 3D case. NOTE: this element does not include torsion, therfore a torque cannot be applied along the local x-axis.""",
    classType=ClassTypeObject,
    equations=r"""    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    miniExample=r"""    from exudyn.beams import GenerateStraightLineANCFCable
    rhoA = 78.
    EA = 1000000.
    EI = 833.3333333333333
    cable = Cable(physicsMassPerLength=rhoA, 
                  physicsBendingStiffness=EI, 
                  physicsAxialStiffness=EA, 
                  )

    ancf=GenerateStraightLineANCFCable(mbs=mbs,
                  positionOfNode0=[0,0,0], positionOfNode1=[2,0,0],
                  numberOfElements=32, #converged to 4 digits
                  cableTemplate=cable, #this defines the beam element properties
                  massProportionalLoad = [0,-9.81,0],
                  fixedConstraintsNode0 = [1,1,1, 0,1,1], #add constraints for pos and rot (r'_y,r'_z)
                  )
    lastNode = ancf[0][-1]

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveStatic()

    #check result
    exu.sys['testResult'] = mbs.GetNodeOutput(lastNode, exu.OutputVariableType.Displacement)[0]
    #ux=-0.5013058140308901
""",
    objectType=ObjectTypeFiniteElement,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv\cConfig(x,0,0)} = \rv\cConfig(x) + y\cdot \nv\cConfig(x)$global position vector of local position $[x,0,0]$"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv\cConfig(x,0,0)} = \LU{0}{\pv\cConfig(x,0,0)} - \LU{0}{\pv\cRef(x,0,0)}$global displacement vector of local position"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv(x,0,0)} = \LU{0}{\dot \rv(x)}$global velocity vector of local position"""),
        ItemOutputVariable(OVDirector1, r"""$\rv'(x)$(axial) slope vector of local axis position (at $y$=0)"""),
        ItemOutputVariable(OVStrainLocal, r"""$\varepsilon$axial strain (scalar) of local axis position (at Y=Z=0)"""),
        ItemOutputVariable(OVCurvatureLocal, r'$[K_x, K_y, K_z]\tp$local curvature vector'),
        ItemOutputVariable(OVForceLocal, r"""$N$ (local) section normal force (scalar, including reference strains) (at $y$=$z$=0); note that strains are highly inaccurate when coupled to bending, thus consider useReducedOrderIntegration=2 and evaluate axial strain at nodes or at midpoint"""),
        ItemOutputVariable(OVTorqueLocal, r"""$M$ (local) bending moment (scalar) (at $y$=$z$=0), which are bending moments as there is no torque"""),
        ItemOutputVariable(OVAcceleration, r"""$\LU{0}{\av(x,0,0)} = \LU{0}{\ddot \rv(x)}$global acceleration vector of local position"""),
        ],
    pythonShortName='Cable',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsLength',
            defaultValue=0.,
            description=r"""$L$ [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives $\rho A L$; must be positive"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsMassPerLength',
            defaultValue=0.,
            description=r'$\rho A$ [SI:kg/m] mass per length of beam'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsBendingStiffness',
            defaultValue=0.,
            description=r"""$EI$ [SI:Nm$^2$] bending stiffness of beam; the bending moment is $m = EI (\kappa - \kappa_0)$, in which $\kappa$ is the material measure of curvature"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsAxialStiffness',
            defaultValue=0.,
            description=r"""$EA$ [SI:N] axial stiffness of beam; the axial force is $f_{ax} = EA (\varepsilon -\varepsilon_0)$, in which $\varepsilon = |\rv^\prime|-1$ is the axial strain"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsBendingDamping',
            defaultValue=0.,
            description=r"""$d_{K}$ [SI:Nm$^2$/s] bending damping of beam ; the additional virtual work due to damping is $\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx$"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsAxialDamping',
            defaultValue=0.,
            description=r"""$d_{\varepsilon}$ [SI:N/s] axial damping of beam; the additional virtual work due to damping is $\delta W_{\dot\varepsilon} = \int_0^L \dot \varepsilon \delta \varepsilon dx$"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='physicsReferenceAxialStrain',
            defaultValue=0.,
            description=r"""$\varepsilon_0$ [SI:1] reference axial strain of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference axial strain value"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='strainIsRelativeToReference',
            defaultValue=0.,
            description=r"""$f\cRef$ if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration plus the values of $\varepsilon_0$ and $\kappa_0$ serve as a reference geometry; allows also values between 0. and 1."""),
        ItemParameter(type=TIndexND(2, ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumbers',
            defaultValue='Index2({EXUstd::InvalidIndex, EXUstd::InvalidIndex})',
            description=r'two node numbers ANCF cable element'),
        ItemParameter(type=TIndex, destination=DestComp+DestParam,
            pythonName='useReducedOrderIntegration',
            defaultValue=0,
            description=r'0/false: use Gauss order 9 integration for virtual work of axial forces, order 5 for virtual work of bending moments; 1/true: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments'),
        ItemFunction(type=TReal, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetLength',
            implementation='return parameters.physicsLength;',
            description=r'access to individual element paramters for base class functions'),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunction(type='template<class TReal, Index ancfSize> void', destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeODE2LHStemplate',
            args='VectorBase<TReal>& ode2Lhs, const ConstSizeVectorBase<TReal, ancfSize>& qANCF, const ConstSizeVectorBase<TReal, ancfSize>& qANCF_t',
            description=r"Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'"),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t + JacobianType::ODE2_ODE2_function + JacobianType::ODE2_ODE2_t_function);'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'AngularVelocity_qt', 'DisplacementMassIntegral_q']),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetOutputVariableBody'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetDisplacement'),
        ItemFunctionDef('GetVelocity'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetAcceleration',
            args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
            description=r"return the (global) acceleration of 'localPosition' according to configuration type"),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetLocalCenterOfMass',
            implementation='return Vector3D({0.5*parameters.physicsLength,0.,0.});'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ANCFCable";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex <= 1, __EXUDYN_invalid_local_node1);
        return parameters.nodeNumbers[localIndex];"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumbers[localIndex]=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 2;'),
        ItemFunctionDef('GetODE2Size',
            implementation='return nODE2coordinates;'),
        ItemRequestedTypes('Node', ['Position', 'PointSlope1']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='return true;'),
        ItemFunctionDef('ParametersHaveChanged',
            implementation='massMatrixComputed = false;'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunction(type=TVectorND(3), destination=DestComp, isVirtual=False, isStatic=True,
            pythonName='MapCoordinates',
            args='const Vector4D& SV, const LinkedDataVector& q0, const LinkedDataVector& q1',
            description=r'map element coordinates (position or veloctiy level) given by nodal vectors q0 and q1 onto compressed shape function vector to compute position, etc.'),
        ItemFunction(type=TVectorND(4), destination=DestComp, isVirtual=False, isStatic=True,
            pythonName='ComputeShapeFunctions',
            args='Real x, Real L',
            description=r"""get compressed shape function vector $\Sm_v$, depending local position $x \in [0,L]$"""),
        ItemFunction(type=TVectorND(4), destination=DestComp, isVirtual=False, isStatic=True,
            pythonName='ComputeShapeFunctions_x',
            args='Real x, Real L',
            description=r"""get first derivative of compressed shape function vector $\frac{\partial \Sm_v}{\partial x}$, depending local position $x \in [0,L]$"""),
        ItemFunction(type=TVectorND(4), destination=DestComp, isVirtual=False, isStatic=True,
            pythonName='ComputeShapeFunctions_xx',
            args='Real x, Real L',
            description=r"""get second derivative of compressed shape function vector $\frac{\partial^2 \Sm_v}{\partial^2 x}$, depending local position $x \in [0,L]$"""),
        ItemFunction(type=TVectorND(4), destination=DestComp, isVirtual=False, isStatic=True,
            pythonName='ComputeShapeFunctions_xxx',
            args='Real x, Real L',
            description=r"""get third derivative of compressed shape function vector $\frac{\partial^3 \Sm_v}{\partial^3 x}$, depending local position $x \in [0,L]$"""),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeCurrentNodeCoordinates',
            args='ConstSizeVector<6>& qNode0, ConstSizeVector<6>& qNode1',
            description=r'Compute node coordinates in current configuration including reference coordinates'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeCurrentNodeVelocities',
            args='ConstSizeVector<6>& qNode0, ConstSizeVector<6>& qNode1',
            description=r'Compute node velocity coordinates in current configuration'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeCurrentObjectCoordinates',
            args='ConstSizeVector<nODE2coordinates>& qANCF',
            description=r'Compute object (finite element) coordinates in current configuration including reference coordinates'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeCurrentObjectVelocities',
            args='ConstSizeVector<nODE2coordinates>& qANCF_t',
            description=r'Compute object (finite element) velocities in current configuration'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeSlopeVector',
            args='Real x, ConfigurationType configuration',
            description=r'compute the slope vector at a certain position, for given configuration'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeSlopeVector_t',
            args='Real x, ConfigurationType configuration',
            description=r'compute the d(slope)/dt vector at a certain position, for given configuration'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeSlopeVector_x',
            args='Real x, ConfigurationType configuration',
            description=r'compute the d(slope)/dx vector at a certain position, for given configuration'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeSlopeVector_xt',
            args='Real x, ConfigurationType configuration',
            description=r'compute the d(slope)/dxdt vector at a certain position, for given configuration'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='PreComputeMassTerms',
            description=r'precompute mass terms if it has not been done yet'),
        ItemFunctionDef('ComputeJacobianODE2_ODE2'),
        ItemFunction(type=TReal, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeAxialStrain',
            args='Real x, ConfigurationType configuration',
            description=r'compute scalar axial strain at a certain position, for given configuration'),
        ItemFunction(type=TReal, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeAxialStrain_t',
            args='Real x, ConfigurationType configuration',
            description=r'compute scalar time derivative of axial strain at a certain position, for given configuration'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeCurvature',
            args='Real x, ConfigurationType configuration',
            description=r'compute vectorial curvature at a certain position, for given configuration'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeCurvature_t',
            args='Real x, ConfigurationType configuration',
            description=r'compute time derivative of vectorial curvature at a certain position, for given configuration'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown; note that all quantities are computed at the beam centerline, even if drawn on surface of cylinder of beam; this effects, e.g., Displacement or Velocity, which is drawn constant over cross section'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='radius',
            defaultValue=0.,
            description=r'if radius==0, only the centerline is drawn; else, a cylinder with radius is drawn; circumferential tiling follows general.cylinderTiling and beam axis tiling follows bodies.beams.axialTiling'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA color of the object; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectANCFCable2D   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectANCFCable2D',
    addIncludesC=r"""#include "ImplObjects/CObjectANCFCable2DBase.h"
class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    addPublicC=r"""    static constexpr Index nODE2coordinates = 8; //!< fixed size of coordinates used e.g. for ConstSizeVectors    static constexpr Index nShapeFunctions = 4; //!< number of shape functions
    static constexpr Index nNodalCoordinates = 4; //!< number of nodal coordinates
""",
    cParentClass=ParentClassCObjectANCFCable2DBase,
    classDescription=r"""A 2D cable finite element using 2 nodes of type NodePoint2DSlope1. The localPosition of the beam with length $L$=physicsLength and height $h$ ranges in $X$-direction in range $[0, L]$ and in $Y$-direction in range $[-h/2,h/2]$ (which is in fact not needed in the ABRV:EOM).""",
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    <!--
    \rowTable{}{$\LU{0}{\fv} $}{}
    \rowTable{}{$\LU{0}{\fv} $}{}
    -->

    | intermediate variables | symbol | description |
    |---|---|---|
    | beam height | $h$ | beam height used in several definitions, but effectively undefined. The geometry of the cross section has no influence except for drawing or contact. |
    | local beam position | $\pLocB=[x,\, y,\, 0]\tp$ | local position at axial coordinate $x \in [0,L]$ and cross section coordinate $y \in [-h/2, h/2]$. |
    | beam axis position | $\LU{0}{\rv(x)} = \rv(x) $ |  |
    | beam axis slope | $\LU{0}{\rv'(x)} = \rv'(x) $ |  |
    | beam axis tangent | $\LU{0}{\tv(x)} = \frac{\rv'(x)}{\Vert \rv(x)'\Vert} $ | this (normalized) vector is normal to cross section |
    | beam axis normal | $\LU{0}{\nv(x)} = [n_x,\, n_y]\tp = [-t_y,\, t_x]\tp  $ | this (normalized) vector lies within the cross section and defines positive $y$-direction. |
    | angular velocity | $\omega_2 = (-r'_y \cdot \dot r'_x + r'_x \cdot \dot r'_y) / \Vert \rv(x)'\Vert^2 $ |  |
    | rotation matrix | $\LU{0b}{\Rot}$ |  |

    The Bernoulli-Euler beam is capable of large axial and bendig deformation as it employs the material measure of curvature for the bending.
    <!-- -->

    #### Kinematics and interpolation

    <!-- -->
    Note that in this section, expressions are written in 2D, while output variables are in general 3D quantities, adding a zero for the $z$-coordinate.
    <!-- -->
    ANCF elements follow the original concept proposed by Shabana [CITE:shabana1997ancf].
    The present 2D element is based on the interpolation used by Berzeri and Shabana [CITE:berzeri2000], but the formulation (especially of the elastic forces) is according to
    Gerstmayr and Irschik [CITE:GerstmayrIrschik2008].
    Slight improvements for the integration of elastic forces and additional terms for off-axis forces and constraints are mentioned here.
    
    The current position of an arbitrary element at local axial position $x \in [0,L]$, where $L$ is the beam length, reads

    $$
    \rv=\rv(x, t),
    $$

    The derivative of the position w.r.t.\ the axial reference coordinate is denoted as slope vector,

    $$
    \rv'= \frac{\partial \rv(x, t)}{\partial x}
    $$

    The interpolation is based on cubic (spline) interpolation of position, displacements and velocities.
    The generalized coordinates $\qv \in \Rcal^8$ of the beam element is defined by

    $$
    \qv= \left[\, \rv_0^{T}\;\;\rv_0^{' T}\;\; \rv_1^{T}\;\; \rv_1^{' T}\, \right]^{T}.
    $$

    in which $\rv_0$ is the position of node 0 and $\rv_1$ is the position of node 1,
    $\rv'_0$ the slope at node 0 and $\rv'_1$ the slope at node 1.
    Note that ANCF coordinates in the present notation are computed as sum of reference and current coordinates

    $$
    \qv = \qv\cCur + \qv\cRef
    $$

    which is used throughout here. For time derivatives, it follows that $\dot \qv = \dot \qv\cCur$.
    
    Position and slope are interpolated with shape functions.
    The position and slope along the beam are interpolated by means of 

    $$
    \rv = \Sm \qv \qquad \mathrm{and} \qquad \rv'=\Sm' \qv.
    $$

    in which $\Sm$ is the shape function matrix,

    $$
    \Sm(x)= \left[\, S_1(x)\,\ImTwo\;\; S_2(x)\,\ImTwo\;\; S_3(x)\,\ImTwo\;\; S_4(x)\,\ImTwo\, \right].
    $$

    with identity matrix $\ImTwo \in \Rcal^{2 \times 2}$ and the shape functions

    $$
    \begin{aligned}
    S_1(x) &= 1-3\frac{x^2}{L^2}+2\frac{x^3}{L^3}, \quad
          S_2(x) = x-2\frac{x^2}{L}+\frac{x^3}{L^2}\\
          S_3(x) &= 3\frac{x^2}{L^2}-2\frac{x^3}{L^3}, \; \; \; \; \; \;  \quad
          S_4(x) = -\frac{x^2}{L}+\frac{x^3}{L^2}
    \end{aligned}
    $$ (eq-cable2d-shapefunctions)

    <!-- -->
    Velocity simply follows as 

    $$
    \frac{\partial \rv}{\partial t} = \dot \rv = \Sm \dot \qv.
    $$

    <!-- -->

    #### Mass matrix

    The mass matrix is constant and therefore precomputed at the first time it is needed (e.g., during computation of initial accelerations).
    The analytical form of the mass matrix reads

    $$
    \Mm_{analytic} = \int_0^L \rho A \Sm(x)^T \Sm(x) dx
    $$

    which is approximated using

    $$
    \Mm = \sum_{ip = 0}^{n_{ip}-1} w(x_{ip}) \frac{L}{2} \rho A \Sm(x_{ip})^T \Sm(x_{ip})
    $$

    with integration weights $w(x_{ip})$, $\sum w(x_{ip})=2$, and integration points $x_{ip}$, given as,

    $$
    x_{ip} = \frac{L}{2}\xi_{ip} + \frac{L}{2} \, .
    $$ (eq-ancfcable-iptransform)

    Here, we use the Gauss integration rule with order 7, having $n_{ip}=4$ Gauss points, see [](#sec-integrationpoints). 
    Due to the third order polynomials, the integration is exact up to round-off errors.
            
    #### Elastic forces

    The elastic forces $\Qm_e$ are implicitly defined by the relation to the 
    virtual work of elastic forces, $\delta W_e$, of applied forces, $\delta W_a$ and of viscous forces, $\delta W_v$, 

    $$
    \Qm_e^T \delta \qv = \delta W_e + \delta W_a + \delta W_v.
    $$ (eq-cable2d-elasticforces)

    The virtual work of elastic forces reads [CITE:GerstmayrIrschik2008],

    $$
    \delta W_e = \int_0^L (N \delta \varepsilon + M \delta K) \,dx,
    $$

    <!--\todo{compute $\delta W_e = \Qm_e^T \delta \qv$ } -->
    in which the axial strain is defined as [CITE:GerstmayrIrschik2008]

    $$
    \varepsilon=\Vert \rv'\Vert-1.
    $$
 
    and the material measure of curvature (bending strain) is given as

    $$
    K=\ev_3^T \frac{ \rv'\times \rv'' }{\Vert \rv'\Vert^2} .
    $$

    <!--\todo{define vector e3} -->
    in which $\ev_3$ is the unit vector which is perpendicular to the plane of the planar beam element.
    
    By derivation, we obtain the variation of axial strain

    $$
    \delta \varepsilon =\frac{\partial \varepsilon}{\partial q_i}\delta q_i
          %= \frac{\rv'^{T}\frac{\partial}{\partial q_i}\rv'}{\Vert \rv' \Vert} \delta q_i
        %=\frac{1}{\Vert \rv' \Vert}\rv'^{T}\frac{\partial \rv'}{\partial q_i}\delta q_i\\
            =\frac{1}{\Vert \rv'\Vert}\rv'^{T}\Sm'_i \delta q_i.
    $$ (eq-cable2d-deltaepsilon)

    and the variation of $K$

    $$
    \begin{aligned}
    \delta K &= \frac{\partial}{\partial q_i} \left( \frac{(\rv'^{T}\times \rv'' )^{T}\ev_{3}}{\Vert \rv' \Vert^2 }\right) \delta q_i\\
           &= \frac{1}{\Vert \rv' \Vert^4} \left[ \Vert \rv' \Vert^2 (\Sm'_i  \times \rv'' +\rv' \times \Sm''_i) -2 (\rv' \times \rv'') (\rv'^{T} \Sm'_i) \right]^{T} \ev_3 \delta q_i
    \end{aligned}
    $$ (eq-cable2d-deltakappa)

    The normal force (axial force) $N$ in the beam is defined as function of the current strain $\varepsilon$,

    $$
    N = EA \, (\varepsilon - \varepsilon_0 - f\cRef \cdot \varepsilon\cRef).
    $$ (eq-n)

    in which $\varepsilon_0$ includes the (pre-)stretch of the beam, e.g., due to temperature or plastic deformation and 
    $\varepsilon\cRef$ includes the strain of the reference configuration.
    As can be seen, the reference strain is only considered, if $f\cRef=1$, which allows to consider the reference configuration to be
    completely stress-free (but the default value is $f\cRef=0$ !).
    Note that -- due to the inherent nonlinearity of $\varepsilon$ -- a combination of $\varepsilon_0$ and $f\cRef=1$ is physically only meaningful for small strains.
    A factor $f\cRef<1$ allows to realize a smooth transition between deformed and straight reference configuration, e.g. for initial configurations.

    The bending moment $M$ in the beam is defined as function of the current material measure of curvature $K$,

    $$
    M = EI \, (K - K_0 - f\cRef \cdot K\cRef).
    $$ (eq-m)

    in which $K_0$ includes the (pre-)curvature of the undeformed beam and
    $K\cRef$ includes the curvature of the reference configuration, multiplied with the factor $f\cRef=1$, see the axial strain above.

    Using the latter definitions, the elastic forces follow from {eq}`eq-cable2d-elasticforces`.
    
    The virtual work of viscous damping forces, assuming viscous effects proportial to axial streching and bending, is defined as

    $$
    \delta W_v = \int_0^L \left( d_\varepsilon \dot \varepsilon \delta \varepsilon + d_K \dot K \delta K \right) \,d x.
    $$

    with material coefficients $d_\varepsilon$ and $d_K$.
    The time derivatives of axial strain $\dot \varepsilon_p$ follows by elementary differentiation

    $$
    \dot \varepsilon =  \frac{\partial }{\partial t}\left(\Vert \rv'\Vert-1 \right)
            %= \frac{\rv^{\prime T} \frac{\partial}{\partial t}\rv'}{\Vert \rv'\Vert} 
            = \frac{1}{\Vert \rv'\Vert} \rv^{\prime T} \Sm' \dot \qv
    $$

    as well as the derivative of the curvature,

    $$
    \begin{aligned}
    \dot K & = &  \frac{\partial }{\partial t}\left(\ev_3^T\frac{ \rv'\times \rv'' }{\Vert \rv'\Vert^2}\right) \\
                     & = &\frac{\ev_3^T}{(\rv'^T \rv')^2} \left( (\rv'^T \rv')   \frac{\partial \left( \rv' \times \rv'' \right)^T }{\partial t} -\left( \rv' \times \rv'' \right)^T  \frac{\partial  (\rv'^T \rv')}{\partial t} \right)\\
                     %& = & \frac{\ev_3^T}{(\rv'^T \rv')^2} \left((\rv'^T \rv') \left( \frac {\partial \rv''}{\partial t} \times \rv''+ \frac{\partial \rv''}{\partial t} \times \rv' \right)-\left( \rv' \times \rv'' \right) \left(2\rv'^T \frac{\partial \rv'}{\partial t}\right) \right) \\
                     & = &  \frac{\ev_3^T}{(\rv'^T \rv')^2}\left((\rv'^T \rv')\left((\Sm' \dot \qv) \times \rv'' + (\Sm'' \dot \qv) \times \rv'\right)-\left( \rv' \times \rv'' \right) (2\rv'^T (\Sm' \dot \qv)) \right) .
    \end{aligned}
    $$

    
    The virtual work of applied forces reads

    $$
    \delta W_a = \sum_i \fv_i^T \delta \rv_i(x_f) + \int_0^L \bv^T \delta \rv(x) \,d x \, ,
    $$ (eq-applied)

    in which $\fv_i$ are forces applied to a certain position $x_f$ at the beam centerline.
    The second term contains a load per length $\bv$, which is case of gravity vector $\gv$ reads

    $$
    \bv = \rho \gv.
    $$

    Note that the variation of $\rv$ simply follows as

    $$
    \delta \rv= \Sm\, \delta \qv
    $$

    #### Numerical integration of Elastic Forces

    The numerical integration of elastic forces $\Qm_e$ is split into terms due to $\delta \varepsilon$ and $\delta K$,

    $$
    \Qm_e = \int_0^L \left(\bullet(x) \frac{\partial \delta \varepsilon}{\partial \delta \qv} + \bullet(x) \frac{\partial \delta K}{\partial \delta \qv} \right) \,dx
    $$

    using different integration rules

    $$
    \Qm_e \approx  \sum_{ip = 0}^{n_{ip}^\varepsilon-1}  \left(\frac{L}{2}  \bullet(x_{ip}) \frac{\partial \delta \varepsilon}{\partial \delta \qv} \right)
                       + \sum_{ip = 0}^{n_{ip}^K-1} \left( \frac{L}{2}\bullet(x_{ip}) \frac{\partial \delta K}{\partial \delta \qv} \right) \,dx
    $$

    with the integration points $x_{ip}$ as defined in {eq}`eq-ancfcable-iptransform` and integration rules from [](#sec-integrationpoints).
    There are 3 different options for integration rules depending on the flag \texttt{useReducedOrderIntegration}:
    \bn
      \item \texttt{useReducedOrderIntegration} = 0: $n_{ip}^\varepsilon = 5$ (Gauss order 9), $n_{ip}^K = 3$ (Gauss order 5) -- this is considered as full integration, leading to very small approximations; certainly, due to the high nonlinearity of expressions, this is only an approximation.
      \item \texttt{useReducedOrderIntegration} = 1: $n_{ip}^\varepsilon = 4$ (Gauss order 7), $n_{ip}^K = 2$ (Gauss order 3) -- this is considered as reduced integration, which is usually sufficiently accurate but leads to slightly less computational efforts, especially for bending terms.
      \item \texttt{useReducedOrderIntegration} = 2: $n_{ip}^\varepsilon = 3$ (Lobatto order 3), $n_{ip}^K = 2$ (Gauss order 3) -- this is a further reduced integration, with the exceptional property that axial strain and bending strain terms are computed at completely disjointed locations: axial strain terms are evaluated at $0$, $L/2$ and $L$, while bending terms are evaluated at $\frac{L}{2} \pm \frac{L}{2}\sqrt{1/3}$. This allows axial strains to freely follow the bending terms at $\frac{L}{2} \pm \frac{L}{2}\sqrt{1/3}$, while axial strains are almost independent from bending terms at $0$, $L/2$ and $L$. However, due to the highly reduced integration, spurious (hourglass) modes may occur in certain applications!
    \en
    Note that the Jacobian of elastic forces is computed using automatic differentiation.
    
    #### Access functions

    For application of forces and constraints at any local beam position $\pLocB=[x,\, y,\, 0]\tp$, the position / velocity Jacobian reads

    $$
    \frac{\partial \LU{0}{\vv(x)}}{\dot \qv} = \Sm(x) + \left[ -y \cdot n_x S'_1(x) \frac{1}{\Vert \rv'\Vert} \LU{0}{\tv}, \,\, 
            -y \cdot n_y S'_1(x) \frac{1}{\Vert \rv'\Vert} \LU{0}{\tv}, \,\, -y \cdot n_x S'_2(x) \frac{1}{\Vert \rv'\Vert} \LU{0}{\tv}, \,\,\ldots \right]
    $$

    with the normalized beam axis normal $\LU{0}{\nv} = [n_x,\, n_y]\tp$, see table above.

    For application of torques at any axis point $x$, the rotation / angular velocity Jacobian $\frac{\partial \LU{0}{\omega(x)}}{\dot \qv} \in \Rcal^{3 \times 8}$ reads

    $$
    \frac{\partial \LU{0}{\omega(x)}}{\dot \qv} = 
           \left[\!\! \begin{array}{ccccc} 
          0 & 0 & 0 & \cdots & 0 \vspace{0.1cm}\\ 
          0 & 0 & 0 & \cdots & 0 \vspace{0.1cm}\\ 
          -r'_y \cdot S'_1(x) \frac{1}{\rv^{\prime 2}} & r'_x \cdot S'_1(x) \frac{1}{\rv^{\prime 2}} & 
          -r'_y \cdot S'_2(x) \frac{1}{\rv^{\prime 2}} & \cdots & r'_x \cdot S'_4(x) \frac{1}{\rv^{\prime 2}}  \end{array} \!\!\right]
    $$

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `axialForceUserFunction(mbs, t, itemNumber, axialPositionNormalized, axialStrain, axialStrain_t, axialStrainRef, physicsAxialStiffness, physicsAxialDamping, curvature, curvature_t, curvatureRef)`
    A user function, which computes the axial force depending on time, strains and curvatures and 
    object parameters (stiffness, damping).
    The object variables are provided to the function using the current values of the ANCFCable2D object.
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.
    \mybold{NOTE:} this function has a different interface as compared to the bending moment function.
    <!-- -->

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs to which object belongs |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{axialPositionNormalized} | Real | axial position at the cable where the user function is evaluated; range is [0,1] |
    | \texttt{axialStrain} | Real | $\varepsilon$ |
    | \texttt{axialStrain\_t} | Real | $\varepsilon_t$ |
    | \texttt{axialStrainRef} | Real | $\varepsilon_0 + f\cRef \cdot \varepsilon\cRef$ |
    | \texttt{physicsAxialStiffness} | Real | as given in object parameters |
    | \texttt{physicsAxialDamping} | Real | as given in object parameters |
    | \texttt{curvature} | Real | $K$ |
    | \texttt{curvature\_t} | Real | $\dot K$ |
    | \texttt{curvatureRef} | Real | $K_0 + f\cRef \cdot K\cRef$ |
    | **return value** | Real | scalar value of computed axial force |

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `bendingMomentUserFunction(mbs, t, itemNumber, axialPositionNormalized, curvature, curvature_t, curvatureRef, physicsBendingStiffness, physicsBendingDamping, axialStrain, axialStrain_t, axialStrainRef)`
    A user function, which computes the bending moment depending on time, strains and curvatures and 
    object parameters (stiffness, damping).
    The object variables are provided to the function using the current values of the ANCFCable2D object.
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.
    \mybold{NOTE:} this function has a different interface as compared to the axial force function.
    <!-- -->

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs to which object belongs |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{axialPositionNormalized} | Real | axial position at the cable where the user function is evaluated; range is [0,1] |
    | \texttt{curvature} | Real | $K$ |
    | \texttt{curvature\_t} | Real | $\dot K$ |
    | \texttt{curvatureRef} | Real | $K_0 + f\cRef \cdot K\cRef$ |
    | \texttt{physicsBendingStiffness} | Real | as given in object parameters |
    | \texttt{physicsBendingDamping} | Real | as given in object parameters |
    | \texttt{axialStrain} | Real | $\varepsilon$ |
    | \texttt{axialStrain\_t} | Real | $\varepsilon_t$ |
    | \texttt{axialStrainRef} | Real | $\varepsilon_0 + f\cRef \cdot \varepsilon\cRef$ |
    | **return value** | Real | scalar value of computed bending moment |

    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    *Example*:
    
```python
#define some material parameters
rhoA = 100.
EA =   1e7.
EI =   1e5

#example of bending moment user function
def bendingMomentUserFunction(mbs, t, itemNumber, axialPositionNormalized, 
           curvature, curvature_t, curvatureRef, physicsBendingStiffness, 
           physicsBendingDamping, axialStrain, axialStrain_t, axialStrainRef):
    fact = min(1,t) #runs from 0 to 1
    #change reference curvature of beam over time:
    kappa=(curvature-curvatureRef*fact) 
    return physicsBendingStiffness*(kappa) + physicsBendingDamping*curvature_t

def axialForceUserFunction(mbs, t, itemNumber, axialPositionNormalized, 
           axialStrain, axialStrain_t, axialStrainRef, physicsAxialStiffness, 
           physicsAxialDamping, curvature, curvature_t, curvatureRef):
    fact = min(1,t) #runs from 0 to 1
    return (physicsAxialStiffness*(axialStrain-fact*axialStrainRef) + 
            physicsAxialDamping*axialStrain_t)

cable = ObjectANCFCable2D(physicsMassPerLength=rhoA, 
                physicsBendingStiffness=EI, 
                physicsBendingDamping = EI*0.1,
                physicsAxialStiffness=EA,
                physicsAxialDamping=EA*0.05,
                physicsReferenceAxialStrain=0.1, #10 <!-- stretch -->
                physicsReferenceCurvature=1,     #radius=1
                bendingMomentUserFunction=bendingMomentUserFunction,
                axialForceUserFunction=axialForceUserFunction,
                )
#use  cable with GenerateStraightLineANCFCable(...)

```
 \vspace{12pt}
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    miniExample=r"""    rhoA = 78.
    EA = 1000000.
    EI = 833.3333333333333
    cable = Cable2D(physicsMassPerLength=rhoA, 
                    physicsBendingStiffness=EI, 
                    physicsAxialStiffness=EA, 
                    )

    ancf=GenerateStraightLineANCFCable2D(mbs=mbs,
                    positionOfNode0=[0,0,0], positionOfNode1=[2,0,0],
                    numberOfElements=32, #converged to 4 digits
                    cableTemplate=cable, #this defines the beam element properties
                    massProportionalLoad = [0,-9.81,0],
                    fixedConstraintsNode0 = [1,1,0,1], #add constraints for pos and rot (r'_y)
                    )
    lastNode = ancf[0][-1]

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveStatic()

    #check result
    exu.sys['testResult'] = mbs.GetNodeOutput(lastNode, exu.OutputVariableType.Displacement)[0]
    #ux=-0.5013058140308901
""",
    objectType=ObjectTypeFiniteElement,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv\cConfig(x,y,0)} = \rv\cConfig(x) + y\cdot \nv\cConfig(x)$global position vector of local position $[x,y,0]$"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv\cConfig(x,y,0)} = \LU{0}{\pv\cConfig(x,y,0)} - \LU{0}{\pv\cRef(x,y,0)}$global displacement vector of local position"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv(x,y,0)} = \LU{0}{\dot \rv(x)} - y \cdot \omega_2 \cdot\LU{0}{\tv(x)} $global velocity vector of local position"""),
        ItemOutputVariable(OVVelocityLocal, r"""$\LU{b}{\vv(x,y,0)} = \LU{b0}{\Rot}\LU{0}{\vv(x,y,0)}$local velocity vector of local position"""),
        ItemOutputVariable(OVRotation, r"""$\varphi = \mathrm{atan2}(r'_y, r'_x)$(scalar) rotation angle of axial slope vector (relative to global $x$-axis)"""),
        ItemOutputVariable(OVDirector1, r"""$\rv'(x)$(axial) slope vector of local axis position (at $y$=0)"""),
        ItemOutputVariable(OVStrainLocal, r"""$\varepsilon$axial strain (scalar) of local axis position (at Y=0)"""),
        ItemOutputVariable(OVCurvatureLocal, r"""$K$axial strain (scalar)"""),
        ItemOutputVariable(OVForceLocal, r"""$N$ (local) section normal force (scalar, including reference strains) (at $y$=0); note that strains are highly inaccurate when coupled to bending, thus consider useReducedOrderIntegration=2 and evaluate axial strain at nodes or at midpoint"""),
        ItemOutputVariable(OVTorqueLocal, r"""$M$ (local) bending moment (scalar) (at $y$=0)"""),
        ItemOutputVariable(OVAngularVelocity, r"""$\tomega = [0,\, ,0,\, \omega_2]$angular velocity of local axis position (at $y$=0)"""),
        ItemOutputVariable(OVAcceleration, r"""$\LU{0}{\av(x,y,0)} = \LU{0}{\ddot \rv(x)} - y \cdot \dot\omega_2 \cdot\LU{0}{\tv(x)}- y \cdot \omega_2 \cdot\LU{0}{\dot\tv(x)} $global acceleration vector of local position"""),
        ItemOutputVariable(OVAngularAcceleration, r"""$\talpha = [0,\, ,0,\, \dot\omega_2]$angular acceleration of local axis position"""),
        ],
    pythonShortName='Cable2D',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsLength',
            defaultValue=0.,
            description=r"""$L$ [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives $\rho A L$; must be positive"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsMassPerLength',
            defaultValue=0.,
            description=r'$\rho A$ [SI:kg/m] mass per length of beam'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsBendingStiffness',
            defaultValue=0.,
            description=r"""$EI$ [SI:Nm$^2$] bending stiffness of beam; the bending moment is $m = EI (\kappa - \kappa_0)$, in which $\kappa$ is the material measure of curvature"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsAxialStiffness',
            defaultValue=0.,
            description=r"""$EA$ [SI:N] axial stiffness of beam; the axial force is $f_{ax} = EA (\varepsilon -\varepsilon_0)$, in which $\varepsilon = |\rv^\prime|-1$ is the axial strain"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsBendingDamping',
            defaultValue=0.,
            description=r"""$d_{K}$ [SI:Nm$^2$/s] bending damping of beam ; the additional virtual work due to damping is $\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx$"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsAxialDamping',
            defaultValue=0.,
            description=r"""$d_{\varepsilon}$ [SI:N/s] axial damping of beam; the additional virtual work due to damping is $\delta W_{\dot\varepsilon} = \int_0^L \dot \varepsilon \delta \varepsilon dx$"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='physicsReferenceAxialStrain',
            defaultValue=0.,
            description=r"""$\varepsilon_0$ [SI:1] reference axial strain of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference axial strain value"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='physicsReferenceCurvature',
            defaultValue=0.,
            description=r"""$\kappa_0$ [SI:1/m] reference curvature of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference curvature value"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='strainIsRelativeToReference',
            defaultValue=0.,
            description=r"""$f\cRef$ if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration plus the values of $\varepsilon_0$ and $\kappa_0$ serve as a reference geometry; allows also values between 0. and 1."""),
        ItemParameter(type=TIndexND(2, ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumbers',
            defaultValue='Index2({EXUstd::InvalidIndex, EXUstd::InvalidIndex})',
            description=r'two node numbers ANCF cable element'),
        ItemParameter(type=TIndex, destination=DestComp+DestParam,
            pythonName='useReducedOrderIntegration',
            defaultValue=0,
            description=r'0/false: use Gauss order 9 integration for virtual work of axial forces, order 5 for virtual work of bending moments; 1/True: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments; 2: use mixed Lobatto/Gauss integration with exceptional quality of axial strain, however, spurious (hourglass) modes may occur!'),
        ItemParameter(type=TPyFunctionMbsScalarIndexScalar9, destination=DestComp+DestParam,
            pythonName='axialForceUserFunction',
            defaultValue=0,
            description=r"""$\mathrm{UF} \in \Rcal$A Python function which defines the (nonlinear relations) of local strains (including axial strain and bending strain) as well as time derivatives to the local axial force; see description below"""),
        ItemParameter(type=TPyFunctionMbsScalarIndexScalar9, destination=DestComp+DestParam,
            pythonName='bendingMomentUserFunction',
            defaultValue=0,
            description=r"""$\mathrm{UF} \in \Rcal$A Python function which defines the (nonlinear relations) of local strains (including axial strain and bending strain) as well as time derivatives to the local bending moment; see description below"""),
        ItemFunctionDef('GetLength',
            implementation='return parameters.physicsLength;'),
        ItemFunctionDef('GetMassPerLength',
            implementation='return parameters.physicsMassPerLength;'),
        ItemFunctionDef('GetMaterialParameters',
            implementation='physicsBendingStiffness = parameters.physicsBendingStiffness; physicsAxialStiffness = parameters.physicsAxialStiffness; physicsBendingDamping = parameters.physicsBendingDamping; physicsAxialDamping = parameters.physicsAxialDamping; physicsReferenceAxialStrain = parameters.physicsReferenceAxialStrain; physicsReferenceCurvature = parameters.physicsReferenceCurvature; physicsMovingMassFactor = 1.;'),
        ItemFunctionDef('UseReducedOrderIntegration',
            implementation='return parameters.useReducedOrderIntegration;'),
        ItemFunctionDef('StrainIsRelativeToReference',
            implementation='return parameters.strainIsRelativeToReference;'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'AngularVelocity_qt', 'DisplacementMassIntegral_q']),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ANCFCable2D";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex <= 1, __EXUDYN_invalid_local_node1);
        return parameters.nodeNumbers[localIndex];"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumbers[localIndex]=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 2;'),
        ItemFunctionDef('GetODE2Size',
            implementation='return nODE2coordinates;'),
        ItemRequestedTypes('Node', ['Position2D', 'Orientation2D', 'Point2DSlope1']),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='return true;'),
        ItemFunctionDef('HasForceUserFunction',
            implementation='return parameters.axialForceUserFunction!=0;'),
        ItemFunctionDef('HasTorqueUserFunction',
            implementation='return parameters.bendingMomentUserFunction!=0;'),
        ItemFunctionDef('ParametersHaveChanged',
            implementation='massMatrixComputed = false;'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionBendingMoment',
            args='Real& torque, const MainSystemBase& mainSystem, Real t, Index itemIndex, Real axialPositionNormalized, Real curvature, Real curvature_t, Real curvatureRef, Real physicsBendingStiffness, Real physicsBendingDamping, Real axialStrain, Real axialStrain_t, Real axialStrainRef',
            description=r'Safe interface to evaluation of user function'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionAxialForce',
            args='Real& force, const MainSystemBase& mainSystem, Real t, Index itemIndex, Real axialPositionNormalized, Real axialStrain, Real axialStrain_t, Real axialStrainRef, Real physicsAxialStiffness, Real physicsAxialDamping, Real curvature, Real curvature_t, Real curvatureRef',
            description=r'Safe interface to evaluation of user function'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawHeight',
            defaultValue=0.,
            description=r'if beam is drawn with rectangular shape, this is the drawing height'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA color of the object; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectALEANCFCable2D   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectALEANCFCable2D',
    addIncludesC=r"""#include "ImplObjects/CObjectANCFCable2DBase.h"
""",
    addProtectedC=r"""    mutable bool massTermsALEComputed; //!< flag which shows that ALE mass terms have been computed; will be set to false at time when parameters are set
    mutable ConstSizeMatrix<nODE2coordinates*nODE2coordinates> preComputedM1, preComputedM2, preComputedB1, preComputedB2; //!< if massTermsALEComputed=true, this contains the constant mass terms for faster computation
""",
    cParentClass=ParentClassCObjectANCFCable2DBase,
    classDescription=r"""A 2D cable finite element using 2 nodes of type NodePoint2DSlope1 and a axially moving coordinate of type NodeGenericODE2, which adds additional (redundant) motion in axial direction of the beam. This allows modeling pipes but also axially moving beams. The localPosition of the beam with length $L$=physicsLength and height $h$ ranges in $X$-direction in range $[0, L]$ and in $Y$-direction in range $[-h/2,h/2]$ (which is in fact not needed in the ABRV:EOM).""",
    classType=ClassTypeObject,
    equations=r"""    A 2D cable finite element using 2 nodes of type NodePoint2DSlope1 and an axially moving coordinate of type NodeGenericODE2.
    The element has 8+1 coordinates and uses cubic polynomials for position interpolation.
    In addition to ANCFCable2D the element adds an Eulerian axial velocity by the GenericODE2 coordiante.
    The parameter \texttt{physicsMovingMassFactor} allows to control the amount of mass, which moves with
    the Eulerian velocity (e.g., the fluid), and which is not moving (the pipe). 
    A factor of \texttt{physicsMovingMassFactor=1} gives an axially moving beam.

    The Bernoulli-Euler beam is capable of large deformation as it employs the material measure of curvature for the bending.
    Note that damping (physicsBendingDamping, physicsAxialDamping) only acts on the non-moving part of the beam, as it is the case for the pipe.
    
    Note that most functions act on the underlying cable finite element, which is not co-moving axially. E.g., if you apply constraints
    to the nodal coordinates, the cable can be fixed, while still the axial component is freely moving.
    If you apply a LoadForce using a MarkerPosition, the force is acting on the beam finite element, but not on the axially moving coordinate.
    In contrast to the latter, the ObjectJointALEMoving2D and the MarkerBodyMass are acting on the moving coordinate as well.

    A detailed paper on this element is yet under submission, but a similar formulation can be found in [CITE:PechsteinGerstmayr2013ale] and 
    the underlying beam element is identical to ObjectANCFCable2D.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    objectType=ObjectTypeFiniteElement,
    outputVariables=[
        ItemOutputVariable(OVPosition, 'global position vector of local position (in X/Y beam coordinates)'),
        ItemOutputVariable(OVDisplacement, 'global displacement vector of local position'),
        ItemOutputVariable(OVVelocity, 'global velocity vector of local position'),
        ItemOutputVariable(OVVelocityLocal, 'local velocity vector of local position'),
        ItemOutputVariable(OVRotation, '(scalar) rotation angle of axial slope vector (relative to global x-axis)'),
        ItemOutputVariable(OVDirector1, '(axial) slope vector of local axis position (at Y=0)'),
        ItemOutputVariable(OVStrainLocal, r"""$\varepsilon$axial strain (scalar) of local axis position (at Y=0)"""),
        ItemOutputVariable(OVCurvatureLocal, r"""$K$axial strain (scalar)"""),
        ItemOutputVariable(OVForceLocal, r"""$N$ (local) section normal force (scalar, including reference strains) (at Y=0); note that strains are highly inaccurate when coupled to bending, thus consider useReducedOrderIntegration=2 and evaluate axial strain at nodes or at midpoint"""),
        ItemOutputVariable(OVTorqueLocal, r"""$M$ (local) bending moment (scalar) (at Y=0)"""),
        ],
    pythonShortName='ALECable2D',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsLength',
            defaultValue=0.,
            description=r"""$L$ [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives $\rho A L$; must be positive"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsMassPerLength',
            defaultValue=0.,
            description=r"""$\rho A$ [SI:kg/m] total mass per length of beam (including axially moving parts / fluid)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsMovingMassFactor',
            defaultValue=1.,
            description=r"""this factor denotes the amount of $\rho A$ which is moving; physicsMovingMassFactor=1 means, that all mass is moving; physicsMovingMassFactor=0 means, that no mass is moving; factor can be used to simulate e.g. pipe conveying fluid, in which $\rho A$ is the mass of the pipe+fluid, while $physicsMovingMassFactor \cdot \rho A$ is the mass per unit length of the fluid"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsBendingStiffness',
            defaultValue=0.,
            description=r"""$EI$ [SI:Nm$^2$] bending stiffness of beam; the bending moment is $m = EI (\kappa - \kappa_0)$, in which $\kappa$ is the material measure of curvature"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsAxialStiffness',
            defaultValue=0.,
            description=r"""$EA$ [SI:N] axial stiffness of beam; the axial force is $f_{ax} = EA (\varepsilon -\varepsilon_0)$, in which $\varepsilon = |\rv^\prime|-1$ is the axial strain"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsBendingDamping',
            defaultValue=0.,
            description=r"""$d_{K}$ [SI:Nm$^2$/s] bending damping of beam ; the additional virtual work due to damping is $\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx$"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsAxialDamping',
            defaultValue=0.,
            description=r"""$d_{\varepsilon}$ [SI:N/s] axial damping of beam; the additional virtual work due to damping is $\delta W_{\dot\varepsilon} = \int_0^L \dot \varepsilon \delta \varepsilon dx$"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='physicsReferenceAxialStrain',
            defaultValue=0.,
            description=r"""$\varepsilon_0$ [SI:1] reference axial strain of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference axial strain value"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='physicsReferenceCurvature',
            defaultValue=0.,
            description=r"""$\kappa_0$ [SI:1/m] reference curvature of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference curvature value"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='physicsUseCouplingTerms',
            defaultValue=True,
            description=r'true: correct case, where all coupling terms due to moving mass are respected; false: only include constant mass for ALE node coordinate, but deactivate other coupling terms (behaves like ANCFCable2D then)'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='physicsAddALEvariation',
            defaultValue=True,
            description=r'true: correct case, where additional terms related to variation of strain and curvature are added'),
        ItemParameter(type=TIndexND(3, ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumbers',
            defaultValue='Index3({EXUstd::InvalidIndex, EXUstd::InvalidIndex, EXUstd::InvalidIndex})',
            description=r'two node numbers ANCF cable element, third node=ALE GenericODE2 node'),
        ItemParameter(type=TIndex, destination=DestComp+DestParam,
            pythonName='useReducedOrderIntegration',
            defaultValue=0,
            description=r'0/false: use Gauss order 9 integration for virtual work of axial forces, order 5 for virtual work of bending moments; 1/true: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='strainIsRelativeToReference',
            defaultValue=0.,
            description=r"""$f\cRef$ if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration plus the values of $\varepsilon_0$ and $\kappa_0$ serve as a reference geometry; allows also values between 0. and 1."""),
        ItemFunctionDef('GetLength',
            implementation='return parameters.physicsLength;'),
        ItemFunctionDef('GetMassPerLength',
            implementation='return parameters.physicsMassPerLength;'),
        ItemFunctionDef('GetMaterialParameters',
            implementation='physicsBendingStiffness = parameters.physicsBendingStiffness; physicsAxialStiffness = parameters.physicsAxialStiffness; physicsBendingDamping = parameters.physicsBendingDamping; physicsAxialDamping = parameters.physicsAxialDamping; physicsReferenceAxialStrain = parameters.physicsReferenceAxialStrain; physicsReferenceCurvature = parameters.physicsReferenceCurvature; physicsMovingMassFactor = parameters.physicsMovingMassFactor;'),
        ItemFunctionDef('UseReducedOrderIntegration',
            implementation='return parameters.useReducedOrderIntegration;'),
        ItemFunctionDef('StrainIsRelativeToReference',
            implementation='return parameters.strainIsRelativeToReference;'),
        ItemFunctionDef('AddALEvariation',
            implementation='return parameters.physicsAddALEvariation;'),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'AngularVelocity_qt', 'DisplacementMassIntegral_q']),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetVelocity'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ALEANCFCable2D";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex <= 2, __EXUDYN_invalid_local_node2);
        return parameters.nodeNumbers[localIndex];"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumbers[localIndex]=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 3;'),
        ItemFunctionDef('GetODE2Size',
            implementation='return nODE2coordinates+1;'),
        ItemRequestedTypes('Node', []),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='return false;'),
        ItemFunctionDef('ParametersHaveChanged',
            implementation='massTermsALEComputed = false; massMatrixComputed = false;'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('PreComputeMassTerms'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawHeight',
            defaultValue=0.,
            description=r'if beam is drawn with rectangular shape, this is the drawing height'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA color of the object; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectANCFBeam   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectANCFBeam',
    addIncludesC=r"""#include "Main/StructuralElementsDataStructures.h"
#include "Autogenerated/BeamSectionGeometry.h"
""",
    addIncludesMain=r"""#include "Autogenerated/PyStructuralElementsDataStructures.h"
""",
    addProtectedC=r"""    mutable bool massMatrixComputed; //!< flag which shows that mass matrix has been computed; will be set to false at time when parameters are set
""",
    addPublicC=r"""    static constexpr Index nODE2perNode = 9;//AUTO: number of element coordinates
    static constexpr Index nNodes = 2;//AUTO: number of nodes for templates
    static constexpr Index nODE2coordinates = nNodes * nODE2perNode;//AUTO: number of nodes for templates
    static constexpr Index nSFperNode = 3;//AUTO: number of shape functions per node
    mutable ConstSizeMatrix<nODE2coordinates*nODE2coordinates> precomputedMassMatrix; //!< if massMatrixComputed=true, this contains the (constant) mass matrix for faster computation (should be in protected area, but needs nODE2perNode)
""",
    cParentClass=ParentClassCObjectBody,
    classDescription=r"""A 3D beam finite element based on the absolute nodal coordinate formulation, using two nodes. The localPosition $x$ of the beam ranges from $-L/2$ (at node 0) to $L/2$ (at node 1). The axial coordinate is $x$ (first coordinate) and the cross section is spanned by local $y$/$z$ axes; assuming dimensions $w_y$ and $w_z$ in cross section, the local position range is $\in [[-L/2,L/2],\, [-wy/2,wy/2],\, [-wz/2,wz/2] ]$. NOTE: Requires further development and tests!""",
    classType=ClassTypeObject,
    equations=r"""    Detailed description coming later.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    objectType=ObjectTypeFiniteElement,
    outputVariables=[
        ItemOutputVariable(OVPosition, 'global position vector of local position vector'),
        ItemOutputVariable(OVDisplacement, 'global displacement vector of local position vector'),
        ItemOutputVariable(OVVelocity, 'global velocity vector of local position vector'),
        ItemOutputVariable(OVVelocityLocal, 'local (cross section) velocity vector of local position vector'),
        ItemOutputVariable(OVAngularVelocity, 'global angular velocity vector of local (axis) position vector'),
        ItemOutputVariable(OVAngularVelocityLocal, 'local angular velocity vector of local (axis) position vector'),
        ItemOutputVariable(OVAcceleration, 'global acceleration vector of local position vector'),
        ItemOutputVariable(OVRotation, '3D Tait-Bryan rotation components of cross section rotation'),
        ItemOutputVariable(OVRotationMatrix, 'rotation matrix of cross section rotation as 9D vector'),
        ],
    pythonShortName='ANCFBeam',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TIndexND(2, ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumbers',
            defaultValue='Index2({EXUstd::InvalidIndex, EXUstd::InvalidIndex})',
            description=r'two node numbers for beam element'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='physicsLength',
            defaultValue=0.,
            description=r"""$L$ [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives $\rho A L$; must be positive"""),
        ItemParameter(type=TBeamSection, destination=DestMain,
            pythonName='sectionData',
            defaultValue='BeamSection()',
            description=r'data as given by exudyn.BeamSection(), defining inertial, stiffness and damping parameters of beam section.'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam, cFlags=CFNoInterface,
            pythonName='physicsMassPerLength',
            defaultValue=0.,
            description=r"""$\rho A$ [SI:kg/m] mass per length of beam; this data is used internally for computation"""),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp+DestParam, cFlags=CFNoInterface,
            pythonName='physicsCrossSectionInertia',
            defaultValue='EXUmath::zeroMatrix3D',
            description=r"""$\rho \Jm$ [SI:kg m] cross section mass moment of inertia tensor; this data is used internally for computation"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam, cFlags=CFNoInterface,
            pythonName='physicsTorsionalBendingStiffness',
            defaultValue=DVZeroVector3D,
            description=r"""$k_\kappa = [GJ_x, \, EI_y, \, EI_z]\tp$ [SI:Nm$^2$] bending and torsional stiffness vector;"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam, cFlags=CFNoInterface,
            pythonName='physicsAxialShearStiffness',
            defaultValue=DVZeroVector3D,
            description=r"""$k_{as} = [EA, \, GA_y, \, GA_z]\tp$ [SI:N] axial and shear stiffness;"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='crossSectionPenaltyFactor',
            defaultValue='Vector3D({1.,1.,1.})',
            description=r"""$k_{cs} = [f_{yy},\,f_{zz},\,f_{yz}]\tp$ [SI:1] additional penalty factors for cross section deformation, which are in total $k_{cs} = [f_{yy}\cdot EA,\, f_{zz}\cdot EA,\, f_{yz}\cdot (GA_y+GA_z)]\tp$"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam, cFlags=CFNoInterface,
            pythonName='physicsTorsionalBendingDamping',
            defaultValue=DVZeroVector3D,
            description=r"""$d_\kappa = [d_{GJx}, \, d_{EIy}, \, d_{EIz}]\tp$ [SI:Nm$^2$] viscous damping of bending and torsional deformation, according to $k_\kappa$"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam, cFlags=CFNoInterface,
            pythonName='physicsAxialShearDamping',
            defaultValue=DVZeroVector3D,
            description=r"""$d_{as} = [d_{EA}, \, d_{GAy}, \, d_{GAz}]\tp$ [SI:N] viscous damping of axial and shear deformation, according to $k_{as}$"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='crossSectionDamping',
            defaultValue=DVZeroVector3D,
            description=r"""$d_{cs} = [d_{fyy},\,d_{fzz},\,d_{fyz}]\tp$ [SI:1] viscous damping according to penalty factors for cross section deformation; the damping is relative to the stiffness and should be thus usually much smaller than 1; the viscous damping factors read  $d_{cs} = [d_{fyy}\cdot EA,\, d_{fzz}\cdot EA,\, d_{fyz}\cdot (GA_y+GA_z)]\tp$"""),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunction(type='template<class TReal> void', destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeODE2LHStemplate',
            args='VectorBase<TReal>& ode2Lhs, const ConstSizeVectorBase<TReal, nODE2coordinates>& qANCF, const ConstSizeVectorBase<TReal, nODE2coordinates>& qANCF_t',
            description=r"Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'"),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'DisplacementMassIntegral_q']),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetOutputVariableBody'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetDisplacement'),
        ItemFunctionDef('GetVelocity'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetAcceleration',
            args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
            description=r"return the (global) acceleration of 'localPosition' according to configuration type"),
        ItemFunctionDef('GetRotationMatrix',
            description='return configuration dependent rotation matrix of node; returns always a 3D Matrix, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ObjectANCFBeam";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex <= 1, __EXUDYN_invalid_local_node1);
        return parameters.nodeNumbers[localIndex];"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumbers[localIndex]=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return nNodes;'),
        ItemFunctionDef('GetODE2Size',
            implementation='return nODE2coordinates;',
            description=r'number of ABRV:ODE2 coordinates'),
        ItemRequestedTypes('Node', ['Position', 'Orientation', 'PointSlope23']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='return true;'),
        ItemFunctionDef('ParametersHaveChanged',
            implementation='massMatrixComputed = false;'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='PreComputeMassTerms',
            description=r'precompute mass terms if it has not been done yet'),
        ItemFunction(type=THomogeneousTransformation, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetLocalPositionFrame',
            args='const Vector3D& localPosition, ConfigurationType configuration',
            description=r'Get frame as homogeneous transformation at some localPosition[0]'),
        ItemFunction(type='SlimVector<nSFperNode*nNodes>', destination=DestComp, isVirtual=False, isStatic=True,
            pythonName='ComputeShapeFunctions',
            args='const Vector3D& localPosition, Real L',
            description=r"""get compressed shape function vector $\Sm_v$, depending local position $\in [[-L/2,L/2],\, [-wy/2,wy/2],\, [-wz/2,wz/2] ]$"""),
        ItemFunction(type='SlimVector<nSFperNode*nNodes>', destination=DestComp, isVirtual=False, isStatic=True,
            pythonName='ComputeShapeFunctions_x',
            args='const Vector3D& localPosition, Real L',
            description=r"""get first derivative of compressed shape function vector $\frac{\partial \Sm_v}{\partial x}$, depending local position $\in [[-L/2,L/2],\, [-wy/2,wy/2],\, [-wz/2,wz/2] ]$"""),
        ItemFunction(type='SlimVector<nSFperNode*nNodes>', destination=DestComp, isVirtual=False, isStatic=True,
            pythonName='ComputeShapeFunctions_y',
            args='const Vector3D& localPosition, Real L',
            description=r"""get first derivative of compressed shape function vector $\frac{\partial \Sm_v}{\partial y}$, depending local position"""),
        ItemFunction(type='SlimVector<nSFperNode*nNodes>', destination=DestComp, isVirtual=False, isStatic=True,
            pythonName='ComputeShapeFunctions_z',
            args='const Vector3D& localPosition, Real L',
            description=r"""get first derivative of compressed shape function vector $\frac{\partial \Sm_v}{\partial z}$, depending local position"""),
        ItemFunction(type='SlimVector<nSFperNode*nNodes>', destination=DestComp, isVirtual=False, isStatic=True,
            pythonName='ComputeShapeFunctions_yx',
            args='const Vector3D& localPosition, Real L',
            description=r"""get first derivative of compressed shape function vector $\frac{\partial \Sm_v}{\partial y}$, depending local position"""),
        ItemFunction(type='SlimVector<nSFperNode*nNodes>', destination=DestComp, isVirtual=False, isStatic=True,
            pythonName='ComputeShapeFunctions_zx',
            args='const Vector3D& localPosition, Real L',
            description=r"""get first derivative of compressed shape function vector $\frac{\partial \Sm_v}{\partial z}$, depending local position"""),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeCurrentNodeCoordinates',
            args='ConstSizeVector<nODE2perNode>& qNode0, ConstSizeVector<nODE2perNode>& qNode1',
            description=r'Compute node coordinates in current configuration including reference coordinates'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeCurrentNodeVelocities',
            args='ConstSizeVector<nODE2perNode>& qNode0, ConstSizeVector<nODE2perNode>& qNode1',
            description=r'Compute node velocity coordinates in current configuration'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeCurrentObjectCoordinates',
            args='ConstSizeVector<2*nODE2perNode>& qANCF',
            description=r'Compute object (finite element) coordinates in current configuration including reference coordinates'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeCurrentObjectVelocities',
            args='ConstSizeVector<2*nODE2perNode>& qANCF_t',
            description=r'Compute object (finite element) velocities in current configuration'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeSlopeVectors',
            args='Real x, ConfigurationType configuration, Vector3D& slopeX,  Vector3D& slopeY,  Vector3D& slopeZ',
            description=r'compute the slope vector at a certain position, for given configuration'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetDeltaLocalTwistAndCurvature',
            args='Real x, ConstSizeMatrix<EXUstd::dim3D * nODE2coordinates>& deltaKappa, ConstSizeVector<EXUstd::dim3D>& kappa',
            description=r'compute twist and curvature and its variation'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetDeltaLocalAxialShearDeformation',
            args='Real x, ConstSizeMatrix<EXUstd::dim3D * nODE2coordinates>& deltaAxialShearDeformation, ConstSizeVector<EXUstd::dim3D>& axialShearDeformation',
            description=r'compute twist and curvature and its variation'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetDeltaCrossSectionDeformation',
            args='Real x, ConstSizeMatrix<EXUstd::dim3D * nODE2coordinates>& deltaCrossSectionDeformation, ConstSizeVector<EXUstd::dim3D>& crossSectionDeformation',
            description=r'compute twist and curvature and its variation'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown; geometry is defined by sectionGeometry'),
        ItemParameter(type='BeamSectionGeometry', destination=DestVisu,
            pythonName='sectionGeometry',
            defaultValue='BeamSectionGeometry()',
            description=r'defines cross section shape used for visualization and contact'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA color of the object; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectBeamGeometricallyExact2D   ++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectBeamGeometricallyExact2D',
    addProtectedC=r"""    static constexpr Index maxNNodes = 3; //!< max number of nodes
    static constexpr Index maxODE2coordinates = 9; //!< max size of coordinates used e.g. for ConstSizeVectors
    mutable bool massMatrixComputed; //!< flag which shows that mass matrix has been computed; will be set to false at time when parameters are set
    mutable ConstSizeMatrix<maxODE2coordinates*maxODE2coordinates> precomputedMassMatrix; //!< if massMatrixComputed=true, this contains the (constant) mass matrix for faster computation
""",
    cParentClass=ParentClassCObjectBody,
    classDescription=r"""A 2D geometrically exact beam finite element, using 2 or 3 nodes of type NodeRigidBody2D. Note that the orientation of the nodes need to follow the cross section orientation in case that includeReferenceRotations=True; e.g., an angle 0 represents the cross section aligned with the $y$-axis, while and angle $\pi/2$ means that the cross section points in negative $x$-direction. Pre-curvature can be included with physicsReferenceCurvature and axial pre-stress can be considered by using a physicsLength different from the reference configuration of the nodes. The localPosition of the beam with length $L$=physicsLength and height $h$ ranges in $X$-direction in range $[-L/2, L/2]$ and in $Y$-direction in range $[-h/2,h/2]$ (which is in fact not needed in the ABRV:EOM).""",
    classType=ClassTypeObject,
    equations=r"""    See paper of Simo and Vu-Quoc (1986).
    Detailed description coming later.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    objectType=ObjectTypeFiniteElement,
    outputVariables=[
        ItemOutputVariable(OVPosition, 'global position vector of local axis (X) and cross section (Y) position'),
        ItemOutputVariable(OVDisplacement, 'global displacement vector of local axis (X) and cross section (Y) position'),
        ItemOutputVariable(OVVelocity, 'global velocity vector of local axis (X) and cross section (Y) position'),
        ItemOutputVariable(OVRotation, r"""3D Tait-Bryan rotation components, containing rotation around $Z$-axis only"""),
        ItemOutputVariable(OVStrainLocal, r"""6 (local) strain components, containing only axial ($XX$, index 0) and shear strain ($XY$, index 5); evaluated at beam axis ONLY"""),
        ItemOutputVariable(OVCurvatureLocal, r"""3D vector of (local) curvature, only $Z$ component is non-zero"""),
        ItemOutputVariable(OVForceLocal, '3D vector of (local) section normal force, containing axial (X) and shear force (Y)'),
        ItemOutputVariable(OVTorqueLocal, '3D vector of (local) torques, containing only bending moment (Z)'),
        ],
    pythonShortName='Beam2D',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TArrayIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumbers',
            defaultValue='ArrayIndex()',
            description=r'two node numbers for beam element'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsLength',
            defaultValue=0.,
            description=r"""$L$ [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives $\rho A L$; must be positive"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsMassPerLength',
            defaultValue=0.,
            description=r'$\rho A$ [SI:kg/m] mass per length of beam'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsCrossSectionInertia',
            defaultValue=0.,
            description=r"""$\rho J$ [SI:kg m] cross section mass moment of inertia; inertia acting against rotation of cross section"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsBendingStiffness',
            defaultValue=0.,
            description=r"""$EI$ [SI:Nm$^2$] bending stiffness of beam; the bending moment is $m = EI (\kappa - \kappa_0)$, in which $\kappa$ is the material measure of curvature"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsAxialStiffness',
            defaultValue=0.,
            description=r"""$EA$ [SI:N] axial stiffness of beam; the axial force is $f_{ax} = EA (\varepsilon -\varepsilon_0)$, in which $\varepsilon$ is the axial strain"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsShearStiffness',
            defaultValue=0.,
            description=r'$GA$ [SI:N] effective shear stiffness of beam, including stiffness correction'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsBendingDamping',
            defaultValue=0.,
            description=r"""$d_{K}$ [SI:Nm$^2$/s] viscous damping of bending deformation; the additional virtual work due to damping is $\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx$"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsAxialDamping',
            defaultValue=0.,
            description=r"""$d_{\varepsilon}$ [SI:N/s] viscous damping of axial deformation"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsShearDamping',
            defaultValue=0.,
            description=r'$d_{\gamma}$ [SI:N/s] viscous damping of shear deformation'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='physicsReferenceCurvature',
            defaultValue=0.,
            description=r"""$\kappa_0$ [SI:1/m] reference curvature of beam (pre-deformation) of beam"""),
        ItemParameter(type=Tbool, destination=DestComp+DestParam,
            pythonName='includeReferenceRotations',
            defaultValue=False,
            description=r'if True, rotation of the cross section at the nodes includes node reference rotations (within referenceCoordinates of NodeRigidBody2D), which are used for the computation of bending strains (this means that a pre-curved beam is stress-free); if False, the reference rotation of the cross section is orthogonal to the reference slope vector. This allows to easily share nodes among several beams with different reference cross section orientation (i.e., only the change of rotation counts).'),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunction(type='template<class TReal> void', destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeODE2LHStemplate',
            args='VectorBase<TReal>& ode2Lhs, const ConstSizeVectorBase<TReal, maxODE2coordinates>& qBeamTotal, const ConstSizeVectorBase<TReal, maxODE2coordinates>& qBeam_t, const ConstSizeVectorBase<Real, maxODE2coordinates>& qBeamRef, Index objectNumber',
            description=r'templated function to enable automatic differentiation'),
        ItemFunctionDef('ComputeJacobianODE2_ODE2'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'AngularVelocity_qt', 'JacobianTtimesVector_q', 'DisplacementMassIntegral_q']),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetOutputVariableBody'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetDisplacement'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetRotationMatrix',
            description='return configuration dependent rotation matrix of beam; returns always a 3D Matrix, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunction(type=TReal, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetRotation',
            args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
            description=r'return configuration dependent rotation of beam (Tait-Bryan angles); returns 3D Vector with z-component'),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetLocalCenterOfMass',
            implementation='return Vector3D({0.,0.,0.});'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "BeamGeometricallyExact2D";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex < parameters.nodeNumbers.NumberOfItems(), __EXUDYN_invalid_local_node0);
        return parameters.nodeNumbers[localIndex];"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumbers[localIndex]=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return parameters.nodeNumbers.NumberOfItems();'),
        ItemFunctionDef('GetODE2Size',
            implementation='return parameters.nodeNumbers.NumberOfItems()*3;',
            description=r'number of ABRV:ODE2 coordinates'),
        ItemFunction(type=Tbool, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='IsLinear',
            implementation='return parameters.nodeNumbers.NumberOfItems() == 2;',
            description=r'Linear=2 node element, Quadratic (!Linear)=3 node element'),
        ItemRequestedTypes('Node', ['Position2D', 'Orientation2D']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='return true;'),
        ItemFunctionDef('ParametersHaveChanged',
            implementation='massMatrixComputed = false;'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeCurrentCoordinates',
            args='ConstSizeVectorBase<Real, maxODE2coordinates>& qBeamTotal, ConstSizeVectorBase<Real, maxODE2coordinates>& qBeam_t, ConstSizeVectorBase<Real, maxODE2coordinates>& qBeamRef, ConfigurationType configuration',
            description=r'compute object coordinates for configuration'),
        ItemFunction(type='template<class TReal, Index nODE2> SlimVectorBase<TReal, 3>', destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='MapCoordinates',
            args='const ConstSizeVector<maxNNodes>& SV, const ConstSizeVectorBase<TReal, nODE2>& qBeam',
            description=r'templated map of element coordinate vector to  [u0,u1,theta0]'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='MapCoordinatesLinear',
            args='const ConstSizeVector<maxNNodes>& SV, const LinkedDataVector& q0, const LinkedDataVector& q1',
            description=r'map element coordinates (position or velocity level) given by nodal vectors q0 and q1 onto compressed shape function vector to compute position, etc.; if SV=SV(x), it returns Vector of coordinates at certain position x: [p0,p1,theta0]'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='MapCoordinatesQuadratic',
            args='const ConstSizeVector<maxNNodes>& SV, const LinkedDataVector& q0, const LinkedDataVector& q1, const LinkedDataVector& q2',
            description=r'map element coordinates for 3-node element'),
        ItemFunction(type='ConstSizeVector<maxNNodes>', destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeShapeFunctions',
            args='Real x',
            description=r"""get compressed shape function vector $\Sm_v$, depending local position $x \in [0,L]$"""),
        ItemFunction(type='ConstSizeVector<maxNNodes>', destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeShapeFunctions_x',
            args='Real x',
            description=r"""get first derivative of compressed shape function vector $\frac{\partial \Sm_v}{\partial x}$, depending local position $x \in [0,L]$"""),
        ItemFunction(type=TMatrixND(2, 2), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetRotationMatrix2D',
            args='Real theta',
            description=r'compute rotation matrix from angle theta'),
        ItemFunction(type='template<class TReal> void', destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeGeneralizedStrains',
            args='Real x, TReal& theta, const ConstSizeVectorBase<TReal, maxODE2coordinates>& qBeamTotal, const ConstSizeVectorBase<TReal, maxODE2coordinates>& qBeam_t, const ConstSizeVectorBase<Real, maxODE2coordinates>& qBeamRef, ConstSizeVectorBase<Real,maxNNodes>& SV, ConstSizeVectorBase<Real, maxNNodes>& SV_x, TReal& gamma1, TReal& gamma2, TReal& theta_x, TReal& gamma1_t, TReal& gamma2_t, TReal& theta_xt, ConstSizeVectorBase<TReal, maxODE2coordinates>& deltaGamma1, ConstSizeVectorBase<TReal, maxODE2coordinates>& deltaGamma2',
            description=r'compute strains and variation of strains for given interpolated derivatives of displacement u1_x, u2_x, angle theta (incl. reference config.!), shape vector SV and shape vector derivatives SV_x and slope vector in reference configuration'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawHeight',
            defaultValue=0.,
            description=r'if beam is drawn with rectangular shape, this is the drawing height'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA color of the object; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectBeamGeometricallyExact   ++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectBeamGeometricallyExact',
    addIncludesC=r"""#include "Main/StructuralElementsDataStructures.h"
#include "Autogenerated/BeamSectionGeometry.h"
""",
    addIncludesMain=r"""#include "Autogenerated/PyStructuralElementsDataStructures.h"
""",
    cParentClass=ParentClassCObjectBody,
    classDescription=r'A 3D geometrically exact beam finite element, currently using two 3D rigid body nodes. The localPosition $x$ of the beam ranges from $-L/2$ (at node 0) to $L/2$ (at node 1). The axial coordinate is $x$ (first coordinate) and the cross section is spanned by local $y$/$z$ axes. NOTE: Requires further development and tests!',
    classType=ClassTypeObject,
    equations=r"""    Detailed description coming later.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    objectType=ObjectTypeFiniteElement,
    outputVariables=[
        ItemOutputVariable(OVPosition, 'global position vector of local axis (1) and cross section (2) position'),
        ItemOutputVariable(OVDisplacement, 'global displacement vector of local axis (1) and cross section (2) position'),
        ItemOutputVariable(OVVelocity, 'global velocity vector of local axis (1) and cross section (2) position'),
        ItemOutputVariable(OVRotation, r"""3D Tait-Bryan rotation components, containing rotation around $z$-axis only"""),
        ItemOutputVariable(OVStrainLocal, r"""6 strain components, containing only axial ($xx$) and shear strain ($xy$)"""),
        ItemOutputVariable(OVCurvatureLocal, r"""3D vector of curvature, containing only curvature w.r.t. $z$-axis"""),
        ],
    pythonShortName='Beam3D',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TIndexND(2, ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumbers',
            defaultValue='Index2({EXUstd::InvalidIndex, EXUstd::InvalidIndex})',
            description=r'two node numbers for beam element'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='physicsLength',
            defaultValue=0.,
            description=r"""$L$ [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives $\rho A L$; must be positive"""),
        ItemParameter(type=TBeamSection, destination=DestMain,
            pythonName='sectionData',
            defaultValue='BeamSection()',
            description=r'data as given by exudyn.BeamSection(), defining inertial, stiffness and damping parameters of beam section.'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam, cFlags=CFNoInterface,
            pythonName='physicsMassPerLength',
            defaultValue=0.,
            description=r"""$\rho A$ [SI:kg/m] mass per length of beam; this data is used internally for computation"""),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp+DestParam, cFlags=CFNoInterface,
            pythonName='physicsCrossSectionInertia',
            defaultValue='EXUmath::zeroMatrix3D',
            description=r"""$\rho \Jm$ [SI:kg m] cross section mass moment of inertia tensor; this data is used internally for computation"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam, cFlags=CFNoInterface,
            pythonName='physicsTorsionalBendingStiffness',
            defaultValue=0.,
            description=r"""$K_\kappa = [GJ_x, \, EI_y, \, EI_z]\tp$ [SI:Nm$^2$] bending and torsional stiffness vector;"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam, cFlags=CFNoInterface,
            pythonName='physicsAxialShearStiffness',
            defaultValue=0.,
            description=r"""$K_{as} = [EA, \, GA_y, \, GA_z]\tp$ [SI:N] axial and shear stiffness;"""),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('ComputeJacobianODE2_ODE2'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t + JacobianType::ODE2_ODE2_function + JacobianType::ODE2_ODE2_t_function);'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'AngularVelocity_qt', 'JacobianTtimesVector_q', 'DisplacementMassIntegral_q']),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetOutputVariableBody'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetDisplacement'),
        ItemFunctionDef('GetVelocity'),
        ItemFunctionDef('GetRotationMatrix',
            description='return configuration dependent rotation matrix of node; returns always a 3D Matrix, independent of 2D or 3D object; for rigid bodies, the argument localPosition has no effect'),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetLocalCenterOfMass',
            implementation='return Vector3D({0.,0.,0.});'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "BeamGeometricallyExact3D";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex <= 1, __EXUDYN_invalid_local_node1);
        return parameters.nodeNumbers[localIndex];"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumbers[localIndex]=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 2;'),
        ItemFunctionDef('GetODE2Size'),
        ItemRequestedTypes('Node', ['Position', 'Orientation']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='return false;'),
        ItemFunctionDef('ParametersHaveChanged',
            implementation=';'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='MapVectors',
            args='const Vector2D& SV, const Vector3D& q0, const Vector3D& q1',
            description=r'map two vectors q0 and q1 at nodes 0 and 1 onto shape vectors SV; if SV=SV(x), it returns Vector of interpolated coordinates at certain position x'),
        ItemFunction(type=THomogeneousTransformation, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetLocalPositionFrame',
            args='const Vector3D& localPosition, ConfigurationType configuration',
            description=r'Get frame as homogeneous transformation at some localPosition[0], using correct interpolation according to Lie groups'),
        ItemFunction(type=TVectorND(2), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeShapeFunctions',
            args='Real x',
            description=r"""get compressed shape function vector $\Sm_v$, depending local position $x \in [0,L]$"""),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown; geometry is defined by sectionGeometry'),
        ItemParameter(type='BeamSectionGeometry', destination=DestVisu,
            pythonName='sectionGeometry',
            defaultValue='BeamSectionGeometry()',
            description=r'defines cross section shape used for visualization and contact'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA color of the object; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectANCFThinPlate   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectANCFThinPlate',
    addProtectedC=r"""    static constexpr Index nODE2coordinates = 36; //!< fixed size of coordinates used e.g. for ConstSizeVectors
    mutable bool massMatrixComputed; //!< flag which shows that mass matrix has been computed; will be set to false at time when parameters are set
    mutable ConstSizeMatrix<nODE2coordinates*nODE2coordinates> precomputedMassMatrix; //!< if massMatrixComputed=true, this contains the (constant) mass matrix for faster computation
""",
    addPublicC=r"""    static constexpr Index nNodes = 4; //!< number of nodes
    static constexpr Index nSF = 12; //!< number of shape functions
    static constexpr Index nnc = 9; //!< number of node coordinates
""",
    cParentClass=ParentClassCObjectBody,
    classDescription=r'OBJECT UNDER CONSTRUCTION: A 3D thin Kirchhoff plate finite element based on the absolute nodal coordinate formulation, using 4 nodes of type NodePointSlope12. The geometry as well as (deformed and distorted) reference configuration is given by the nodes. The localPosition follows unit-coordinates in the range [-1,1] for X, Y and Z coordinates; the thickness of the plate is h; This element is under construction.',
    classType=ClassTypeObject,
    equations=r"""    Note: For output variables, the localPosition is defined in $[-1,-1,-1] ... [1,1,1]$, where $[-1,-1,0]$ is the position of node 0.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectBody,
    miniExample=r"""    #to be done

    #check result
    exu.sys['testResult'] = 0
""",
    objectType=ObjectTypeFiniteElement,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv\cConfig(x,y,z)}$global position vector of local position $[x,y,z]$"""),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\uv\cConfig(x,y,z)} = \LU{0}{\pv\cConfig(x,y,z)} - \LU{0}{\pv\cRef(x,y,z)}$global displacement vector of local position"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv(x,y,z)} = \LU{0}{\dot \rv(x,y,z)}$global velocity vector of local position"""),
        ItemOutputVariable(OVDirector1, r"""$\rv_x(x,y,z)$(axial) slope vector of local position (at $z$=0)"""),
        ItemOutputVariable(OVDirector2, r"""$\rv_y(x,y,z)$(axial) slope vector of local position (at $z$=0)"""),
        ItemOutputVariable(OVStrainLocal, r"""$\varepsilon$axial strain (scalar) of local axis position (at Z=0)"""),
        ItemOutputVariable(OVCurvatureLocal, r'$[K_x, K_y, K_z]\tp$local curvature vector'),
        ItemOutputVariable(OVForceLocal, r"""$N$ (local) section normal force per length (scalar, including reference strains) (at $z$=0)"""),
        ItemOutputVariable(OVTorqueLocal, r"""$M$ (local) bending moment per length (scalar) (at $z$=0), which are bending moments as there is no torque"""),
        ItemOutputVariable(OVStressLocal, 'local inplane stress components'),
        ItemOutputVariable(OVAcceleration, r"""$\LU{0}{\av(x,y,z)} = \LU{0}{\ddot \rv(x,y,z)}$global acceleration vector of local position"""),
        ],
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"objects's unique name"),
        ItemParameter(type=TNumpyVector, destination=DestComp+DestParam,
            pythonName='physicsThickness',
            defaultValue='Vector()',
            description=r'$h$ [SI:m] thickness of plate either provided as scalar or as vector (4 values, same order as local element node numbers) values that are linearly interpolated from nodal values; dimensionality must agree between thickness, strainCoefficients and curvatureCoefficients'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='physicsDensity',
            defaultValue=0.,
            description=r"""$\rho$ [SI:kg/m$^3$] density of the plate, possibly averaged over thickness"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='physicsMassProportionalDamping',
            defaultValue=0.,
            description=r"""mass-proportional damping coefficient $\alpha$ [SI:1/s]; adds massmatrix proportional damping forces $\fv_d = \alpha \Mm \dot{\qv}$"""),
        ItemParameter(type=TMatrix3DList, destination=DestComp+DestParam,
            pythonName='physicsStrainCoefficients',
            defaultValue='Matrix3DList()',
            description=r"""$\Dm_\varepsilon$ [SI:N/m] stiffness coefficients related to inplane normal and shear strains, integrated over height of the plate; either given as 3D Matrix (numpy array), or a list of 3D matrices at each nodal point, see thickness; dimensionality must agree between thickness, strainCoefficients and curvatureCoefficients"""),
        ItemParameter(type=TMatrix3DList, destination=DestComp+DestParam,
            pythonName='physicsCurvatureCoefficients',
            defaultValue='Matrix3DList()',
            description=r"""$\Dm_\kappa$ [SI:Nm] stiffness coefficients related to curvatures, integrated over height of the plate; either given as 3D Matrix (numpy array), or a list of 3D matrices at each nodal point, see thickness; dimensionality must agree between thickness, strainCoefficients and curvatureCoefficients"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='strainIsRelativeToReference',
            defaultValue=1.,
            description=r"""$f\cRef$ if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration serves as a reference geometry; allows also values between 0. and 1. to perform a transition during static computation"""),
        ItemParameter(type=TVectorND(4), destination=DestComp+DestParam,
            pythonName='slopesScalingX',
            defaultValue='Vector4D({-1.,-1.,-1.,-1.})',
            description=r'scaling of x-slopes at each element node; flat elements: half of the side length of the element; curved: optimal values such that curved geometry is best approximated; if negative (default) values are used, length is computed from node distances.'),
        ItemParameter(type=TVectorND(4), destination=DestComp+DestParam,
            pythonName='slopesScalingY',
            defaultValue='Vector4D({-1.,-1.,-1.,-1.})',
            description=r'scaling of y-slopes at each element node; flat elements: half of the side length of the element; curved: optimal values such that curved geometry is best approximated; if negative (default) values are used, length is computed from node distances.'),
        ItemParameter(type=TIndexND(4, ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumbers',
            defaultValue='Index4({EXUstd::InvalidIndex, EXUstd::InvalidIndex, EXUstd::InvalidIndex, EXUstd::InvalidIndex})',
            description=r'4 NodePointSlope12 node numbers, with local (xi,eta) coordinates as [(-1,-1),(1,-1),(1,1),(-1,1)]'),
        ItemParameter(type=TIndex, destination=DestComp+DestParam,
            pythonName='useReducedOrderIntegration',
            defaultValue=0,
            description=r'0/false: use highest Gauss integration for virtual work of strains'),
        ItemFunctionDef('ComputeMassMatrix'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunction(type='template<class TReal> void', destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeODE2LHStemplate',
            args='VectorBase<TReal>& ode2Lhs, const ConstSizeVectorBase<TReal, nODE2coordinates>& qANCFtotal, const ConstSizeVectorBase<TReal, nODE2coordinates>& qANCF_t',
            description=r"Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to 'ode2Lhs'"),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t + JacobianType::ODE2_ODE2_function + JacobianType::ODE2_ODE2_t_function);'),
        ItemAccessFunctionTypes(['TranslationalVelocity_qt', 'DisplacementMassIntegral_q']),
        ItemFunctionDef('GetAccessFunctionBody'),
        ItemFunctionDef('GetOutputVariableBody'),
        ItemFunctionDef('GetPosition'),
        ItemFunctionDef('GetDisplacement'),
        ItemFunctionDef('GetVelocity'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetAcceleration',
            args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
            description=r"return the (global) acceleration of 'localPosition' according to configuration type"),
        ItemFunctionDef('GetAngularVelocity'),
        ItemFunctionDef('GetLocalCenterOfMass',
            implementation='return Vector3D({0.,0.,0.});'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ANCFThinPlate";',
            description=r'Get type name of object; could also be realized via a string -> type conversion?'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex <= 3, __EXUDYN_invalid_local_node1);
        return parameters.nodeNumbers[localIndex];"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumbers[localIndex]=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 4;'),
        ItemFunctionDef('GetODE2Size',
            implementation='return nODE2coordinates;'),
        ItemRequestedTypes('Node', ['Position', 'PointSlope12']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::MultiNoded);',
            description=r'Get type of object, e.g. to categorize and distinguish during assembly and computation'),
        ItemFunctionDef('HasConstantMassMatrix',
            implementation='return true;'),
        ItemFunctionDef('ParametersHaveChanged',
            description='This function is called upon change of parameters'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunction(type='template<class TReal> SlimVectorBase<TReal, 3>', destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='MapCoordinates',
            args='const Vector12D& sf, const ConstSizeVectorBase<TReal, nODE2coordinates>& q',
            description=r'map element coordinates (position or veloctiy level) given by nodal vectors q0, ..., q3 onto shape function vector to compute position, etc.'),
        ItemFunction(type='template<class TReal> void', destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeKinematics',
            args='Real xi, Real eta, const ConstSizeVectorBase<Real, nODE2coordinates>& qANCFref, const ConstSizeVectorBase<TReal, nODE2coordinates>& qANCFtotal, SlimVectorBase<TReal, 3>& eps, SlimVectorBase<TReal, 3>& kappa',
            description=r'compute strains and curvatures relative to reference configuration'),
        ItemFunction(type='template<class TReal> TReal', destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeElementEnergy',
            args='const ConstSizeVectorBase<Real, nODE2coordinates>& qANCFref, const ConstSizeVectorBase<TReal, nODE2coordinates>& qANCFtotal',
            description=r'compute element energy integrating over Gauss points'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ScaleShapeFunctions',
            args='Vector12D& sf',
            description=r'scale shape functions accordingly (only reference configuration!)'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeShapeFunctions',
            args='Real xi, Real eta, Vector12D& sf, bool scaled=true',
            description=r"""get compressed shape function vector $\Sm_v$, depending on local position $[\xi, \eta] \in [-1,1] \times [-1,1]$ (in unit coordinates)"""),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeShapeFunctions_xy',
            args='Real xi, Real eta, Vector12D& sf_x, Vector12D& sf_y, bool scaled=true',
            description=r"""get first derivatives of compressed shape function vector $\Sm_v$, depending on local position $[\xi, \eta] \in [-1,1] \times [-1,1]$ (in unit coordinates)"""),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeShapeFunctions_xxyy',
            args='Real xi, Real eta, Vector12D& sf_xx, Vector12D& sf_yy, Vector12D& sf_xy, bool scaled=true',
            description=r"""get second derivatives of compressed shape function vector $\Sm_v$, depending on local position $[\xi, \eta] \in [-1,1] \times [-1,1]$ (in unit coordinates)"""),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeReferenceObjectCoordinates',
            args='ConstSizeVector<nODE2coordinates>& qANCF',
            description=r'Compute object (finite element) coordinates in reference configuration'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeCurrentTotalObjectCoordinates',
            args='ConstSizeVector<nODE2coordinates>& qANCF',
            description=r'Compute object (finite element) coordinates in current configuration including reference coordinates'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeObjectCoordinates',
            args='ConstSizeVector<nODE2coordinates>& qANCF, ConfigurationType configuration = ConfigurationType::Current',
            description=r'Compute object (finite element) coordinates in given configuration'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeObjectVelocities',
            args='ConstSizeVector<nODE2coordinates>& qANCF_t, ConfigurationType configuration = ConfigurationType::Current',
            description=r'Compute object (finite element) velocities in given configuration'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeObjectAccelerations',
            args='ConstSizeVector<nODE2coordinates>& qANCF_tt, ConfigurationType configuration = ConfigurationType::Current',
            description=r'Compute object (finite element) accelerations in given configuration'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetNormal',
            args='const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current',
            description=r"return the (global) normal at 'localPosition' according to configuration type"),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetSlopes',
            args='const Vector3D& localPosition, Vector3D& slopeX, Vector3D& slopeY, ConfigurationType configuration = ConfigurationType::Current',
            description=r"compute the (global) slope vectors at 'localPosition' according to configuration type"),
        ItemFunction(type='template<class TReal> ConstSizeMatrixBase<TReal, 4>', destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetElementJacobian',
            args='const SlimVectorBase<TReal, 3>& r_xi_ref, const SlimVectorBase<TReal, 3>& r_eta_ref, const SlimVectorBase<TReal, 3>& n0',
            description=r'compute 2x2 element jacobian matrix; uses inplane orthogonal basis vectors'),
        ItemFunction(type=TReal, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetElementJacobian',
            description=r'compute element jacobian for transformation of unit element derivatives to global derivatives'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='PreComputeMassTerms',
            description=r'precompute mass terms if it has not been done yet'),
        ItemFunction(type=TReal, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeThicknessAtPoint',
            args='Real xi, Real eta',
            description=r'compute thickness from local unit coordinates'),
        ItemFunction(type=TMatrixND(3, 3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeStrainCoefficientsAtPoint',
            args='Real xi, Real eta',
            description=r'compute strain coefficient matrix from local unit coordinates'),
        ItemFunction(type=TMatrixND(3, 3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeCurvatureCoefficientsAtPoint',
            args='Real xi, Real eta',
            description=r'compute curvature coefficient matrix from local unit coordinates'),
        ItemFunctionDef('ComputeJacobianODE2_ODE2'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown; note that all quantities are computed at the beam centerline, even if drawn on surface of cylinder of beam; this effects, e.g., Displacement or Velocity, which is drawn constant over cross section'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA color of the object; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectConnectorSpringDamper   +++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectConnectorSpringDamper',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCObjectConnector,
    classDescription=r'An simple spring-damper element with additional force, connecting to position-based markers.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | current global position which is provided by marker m0 |
    | marker m1 position | $\LU{0}{\pv}_{m1}$ |  |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ |  |
    | time derivative of distance | $\dot L$ | $\Delta\! \LU{0}{\vv}\tp \vv_{f}$ |

    | output variables | symbol | formula |
    |---|---|---|
    | Displacement | $\Delta\! \LU{0}{\pv}$ | $\LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0}$ |
    | Velocity | $\Delta\! \LU{0}{\vv}$ | $\LU{0}{\vv}_{m1} - \LU{0}{\vv}_{m0}$ |
    | Distance | $L$ | $|\Delta\! \LU{0}{\pv}|$ |
    | Force | $\fv$ | see below |

    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->

    #### Connector forces

    <!-- -->
    The unit vector in force direction reads (raises SysError if $L=0$),

    $$
    \vv_{f} = \frac{1}{L} \Delta\! \LU{0}{\pv}
    $$

    If \texttt{activeConnector = True}, the scalar spring force is computed as

    $$
    f_{SD} = k\cdot(L-L_0) + d \cdot(\dot L -\dot L_0)+ f_{a}
    $$

    If the springForceUserFunction $\mathrm{UF}$ is defined, $\fv$ instead becomes ($t$ is current time)

    $$
    f_{SD} = \mathrm{UF}(mbs, t, i_N, L-L_0, \dot L - \dot L_0, k, d, f_{a})
    $$

    and \texttt{iN} represents the itemNumber (=objectNumber). Note that, if \texttt{activeConnector = False}, $f_{SD}$ is set to zero.

    The vector of the spring-damper force applied at both markers finally reads

    $$
    \fv = f_{SD}\vv_{f}
    $$

    The virtual work of the connector force is computed from the virtual displacement 

    $$
    \delta \Delta\! \LU{0}{\pv} = \delta \LU{0}{\pv}_{m1} - \delta \LU{0}{\pv}_{m0} \, ,
    $$

    and the virtual work (note the transposed version here, because the resulting generalized forces shall be a column vector),

    $$
    \delta W_{SD} = \fv \delta \Delta\! \LU{0}{\pv} 
          = \left( k\cdot(L-L_0) + d \cdot (\dot L - \dot L_0) + f_{a} \right) \left(\delta \LU{0}{\pv}_{m1} - \delta \LU{0}{\pv}_{m0} \right)\tp \vv_{f} 
          \, .
    $$

    The generalized (elastic) forces thus result from

    $$
    \Qm_{SD} = \frac{\partial \LU{0}{\pv}}{\partial \qv_{SD}\tp} \fv 
          \, ,
    $$

    and read for the markers $m0$ and $m1$,

    $$
    \Qm_{SD, m0} 
          = -\left( k\cdot(L-L_0) + d \cdot (\dot L - \dot L_0) + f_{a} \right) \Jm_{pos,m0}\tp \vv_{f} , \quad
          \Qm_{SD, m1} 
          = \left( k\cdot(L-L_0) + d \cdot (\dot L - \dot L_0)+ f_{a} \right) \Jm_{pos,m1}\tp \vv_{f} 
          \, ,
    $$

    where $\Jm_{pos,m1}$ represents the derivative of marker $m1$ w.r.t.\ its associated coordinates $\qv_{m1}$, analogously $\Jm_{pos,m0}$.
    <!-- -->

    #### Connector Jacobian

    The position-level jacobian for the connector, involving all coordinates associated with markers $m0$ and $m1$, follows from 

    $$
    \Jm_{SD} = \mp{\frac{\partial \Qm_{SD, m0}}{\partial \qv_{m0}} }{\frac{\partial \Qm_{SD, m0}}{\partial \qv_{m1}}}
                        {\frac{\partial \Qm_{SD, m0}}{\partial \qv_{m1}} }{\frac{\partial \Qm_{SD, m1}}{\partial \qv_{m1}}}
    $$

    and the velocity level jacobian reads

    $$
    \Jm_{SD,t} = \mp{\frac{\partial \Qm_{SD, m0}}{\partial \dot \qv_{m0}} }{\frac{\partial \Qm_{SD, m0}}{\partial \dot \qv_{m1}}}
                        {\frac{\partial \Qm_{SD, m0}}{\partial \dot \qv_{m1}} }{\frac{\partial \Qm_{SD, m1}}{\partial \dot \qv_{m1}}}
    $$

    The sub-Jacobians follow from

    $$
    \frac{\partial \Qm_{SD, m0}}{\partial \qv_{m0}} = 
           -\frac{\partial \Jm_{pos,m0}\tp }{\partial \qv_{m0}} \vv_{f} \left( k\cdot(L-L_0) + d \cdot(\dot L - \dot L_0) + f_{a} \right) 
           -\Jm_{pos,m0}\tp \frac{\partial \vv_{f} \left( k\cdot(L-L_0) + d \cdot(\dot L - \dot L_0) + f_{a} \right)   }{\partial \qv_{m0}}
    $$

    in which the term $\frac{\partial \Jm_{pos,m0}\tp }{\partial \qv_{m0}}$ is computed from a special function provided by markers, that
    compute the derivative of the marker jacobian times a constant vector, in this case the spring force $\fv$; this jacobian term is usually less  
    dominant, but is included in the numerical as well as the analytical derivatives, see the general jacobian computation information.
    
    The other term, which is the dominant term, is computed as (dependence of velocity term on position coordinates and $\dot L_0$ term neglected),

    $$
    \begin{aligned}
    \frac{\partial \Qm_{SD, m0}}{\partial \qv_{m0}}
          &= -\Jm_{pos,m0}\tp \frac{\partial \vv_{f} \left( k\cdot(L-L_0) + d \cdot(\dot L - \dot L_0) + f_{a} \right)   }{\partial \qv_{m0}}
          \\
          &= -\Jm_{pos,m0}\tp \frac{\partial  \left( k\cdot \left( \Delta\! \LU{0}{\pv} - L_0 \vv_{f} \right)+ \vv_{f} \left(d \cdot \vv_{f}\tp \Delta\! \LU{0}{\vv}  + f_{a} \right) \right)   }{\partial \qv_{m0}} 
          \\
          &\approx& \Jm_{pos,m0}\tp \left(k\cdot \Im - k  \frac{L_0}{L}\left(\Im - \LU{0}{\vv_{f}} \otimes \LU{0}{\vv_{f}} \right)  +\frac{1}{L}\left(\Im - \LU{0}{\vv_{f}} \otimes \LU{0}{\vv_{f}} \right) \left(d \cdot \vv_{f}\tp \Delta\! \LU{0}{\vv}  + f_{a} \right) \right. \\
          &&\left. + d \LU{0}{\vv_{f}} \otimes \left(\frac{1}{L}\left(\Im - \LU{0}{\vv_{f}} \otimes 
          \LU{0}{\vv_{f}} \right) \LU{0}{\vv_{f}} \right) \right)
          \LU{0}{\Jm_{pos,m0}}
    \end{aligned}
    $$

    <!--+++++++++++++++++++++++++++++++++++++++++++ -->
    Alternatively (again $\dot L_0$ term neglected):

    $$
    \begin{aligned}
    \frac{\partial \Qm_{SD, m0}}{\partial \qv_{m0}}
          &= -\Jm_{pos,m0}\tp \frac{\partial \vv_{f} \left( k\cdot(L-L_0) + d \cdot(\dot L - \dot L_0) + f_{a} \right)   }{\partial \qv_{m0}}
          \\
          %+++
          &= \Jm_{pos,m0}\tp \frac{1}{L}\left(\Im - \LU{0}{\vv_{f}} \otimes \LU{0}{\vv_{f}} \right)
              \left( k\cdot(L-L_0) + d \cdot(\dot L - \dot L_0) + f_{a} \right) \Jm_{pos,m0}
          \\
          && +\Jm_{pos,m0}\tp \LU{0}{\vv_{f}}
              \otimes \left( k\cdot \LU{0}{\vv_{f}} + d \cdot\Delta\! \LU{0}{\vv} \frac{1}{L}\left(\Im - \LU{0}{\vv_{f}} \otimes \LU{0}{\vv_{f}} \right) 
              \right) \Jm_{pos,m0} - d \Jm_{pos,m0}\tp \LU{0}{\vv_{f}} \otimes \LU{0}{\vv_{f}} \frac{\partial \Delta\! \LU{0}{\vv}}{\partial \qv_{m0}}  \\
          %+++
          &= \Jm_{pos,m0}\tp \left(\frac{f_{SD}}{L}\left(\Im - \LU{0}{\vv_{f}} \otimes \LU{0}{\vv_{f}} \right)
               + k \LU{0}{\vv_{f}} \otimes \LU{0}{\vv_{f}} + 
              \frac{d}{L} \left(\LU{0}{\vv_{f}} \otimes \Delta\! \LU{0}{\vv}\right) 
                     \cdot \left(\Im - \LU{0}{\vv_{f}} \otimes \LU{0}{\vv_{f}} \right) + ...!
              \right) \Jm_{pos,m0}
    \end{aligned}
    $$

    <!--+++++++++++++++++++++++++++++++++++++++++++ -->
    Noting that $\frac{\partial \vv_{f} }{\partial \qv_{m0}} = 
    -\frac{1}{L}\left(\Im - \LU{0}{\vv_{f}} \otimes \LU{0}{\vv_{f}} \right) \LU{0}{\Jm_{pos,m0}}$ and 
    $\frac{\partial \vv_{f} }{\partial \qv_{m1}} = 
    \frac{1}{L}\left(\Im - \LU{0}{\vv_{f}} \otimes \LU{0}{\vv_{f}} \right) \LU{0}{\Jm_{pos,m1}}$.
    <!-- -->
    The Jacobian w.r.t.\ velocity coordinates follows as

    $$
    \begin{aligned}
    \frac{\partial \Qm_{SD, m0}}{\partial \dot \qv_{m0}}
          &= -\Jm_{pos,m0}\tp \frac{\partial \vv_{f} \left( k\cdot(L-L_0) + d \cdot(\dot L - \dot L_0) + f_{a} \right)   }{\partial \dot \qv_{m0}}
          \\
          &= \Jm_{pos,m0}\tp \left(d \vv_{f} \otimes \vv_{f} \right) \LU{0}{\Jm_{pos,m0}}
    \end{aligned}
    $$

    Note that in case that $L=0$, the term $\frac{1}{L} \left(\Im - \LU{0}{\vv_{f}} \otimes \LU{0}{\vv_{f}} \right)$ is replaced
    by the unit matrix, in order to avoid zero (singular) jacobian; this is a workaround and should only occur in exceptional cases.
    
    The term $\frac{\partial \Delta\! \LU{0}{\vv}}{\partial \qv_{m0}}$, which is important for large damping, yields

    $$
    \frac{\partial \Delta\! \LU{0}{\vv}}{\partial \qv_{m0}} = 
          \frac{\partial \Jm_{pos,m0} \dot \qv_{m0}}{\partial \qv_{m0}}=
          \frac{\partial \Jm_{pos,m0} }{\partial \qv_{m0}} \dot \qv_{m0}
    $$

    The latter term is currently neglected.
    
    Jacobians for markers $m1$ and mixed $m0$/$m1$ terms follow analogously.
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `springForceUserFunction(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force)`
    A user function, which computes the spring force depending on time, object variables (deltaL, deltaL\_t) and 
    object parameters (stiffness, damping, force).
    The object variables are provided to the function using the current values of the SpringDamper object.
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.
    <!-- -->

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs to which object belongs |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{deltaL} | Real | $L-L_0$, spring elongation |
    | \texttt{deltaL\_t} | Real | $(\dot L - \dot L_0)$, spring velocity, including offset |
    | \texttt{stiffness} | Real | copied from object |
    | \texttt{damping} | Real | copied from object |
    | \texttt{force} | Real | copied from object; constant force |
    | **return value** | Real | scalar value of computed spring force |

    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    *Example*:
    
```python
#define nonlinear force
def UFforce(mbs, t, itemNumber, u, v, k, d, F0): 
    return k*u + d*v + F0
#markerNumbers taken from mini example
mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[m0,m1],
                                          referenceLength = 1, 
                                          stiffness = 100, damping = 1,
                                          springForceUserFunction = UFforce))

```
 \vspace{12pt}
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    miniExample=r"""    node = mbs.AddNode(NodePoint(referenceCoordinates = [1.05,0,0]))
    oMassPoint = mbs.AddObject(MassPoint(nodeNumber = node, physicsMass=1))
    
    m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0]))
    m1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oMassPoint, localPosition=[0,0,0]))
    
    mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[m0,m1],
                                              referenceLength = 1, #shorter than initial distance
                                              stiffness = 100,
                                              damping = 1))

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic()

    #check result at default integration time
    exu.sys['testResult'] = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[0]
""",
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVDistance, 'distance between both points'),
        ItemOutputVariable(OVDisplacement, 'relative displacement between both points'),
        ItemOutputVariable(OVVelocity, 'relative velocity between both points'),
        ItemOutputVariable(OVForce, r'$\fv$3D spring-damper force vector'),
        ItemOutputVariable(OVForceLocal, r"""$f_{SD}$scalar spring-damper force"""),
        ],
    pythonShortName='SpringDamper',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"connector's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'$[m0,m1]\tp$list of markers used in connector'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='referenceLength',
            defaultValue=0.,
            description=r'$L_0$reference length [SI:m] of spring'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='stiffness',
            defaultValue=0.,
            description=r'$k$stiffness [SI:N/m] of spring; force acts against (length-initialLength)'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='damping',
            defaultValue=0.,
            description=r'$d$damping [SI:N/(m s)] of damper; force acts against d/dt(length)'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='force',
            defaultValue=0.,
            description=r'$f_{a}$added constant force [SI:N] of spring; scalar force; f=1 is equivalent to reducing initialLength by 1/stiffness; f > 0: tension; f < 0: compression; can be used to model actuator force'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='velocityOffset',
            defaultValue=0.,
            description=r"""$\dot L_0$velocity offset [SI:m/s] of damper, being equivalent to time change of reference length"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemParameter(type=TPyFunctionMbsScalarIndexScalar5, destination=DestComp+DestParam,
            pythonName='springForceUserFunction',
            defaultValue=0,
            description=r"""$\mathrm{UF} \in \Rcal$A Python function which defines the spring force with parameters; the Python function will only be evaluated, if activeConnector is true, otherwise the SpringDamper is inactive; see description below"""),
        ItemFunctionDef('HasUserFunction',
            implementation='return (parameters.springForceUserFunction!=0);'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('ComputeJacobianODE2_ODE2'),
        ItemFunctionDef('ComputeJacobianForce6D'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Position']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ConnectorSpringDamper";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeConnectorProperties',
            args='const MarkerDataStructure& markerData, Index itemIndex, Vector3D& relPos, Vector3D& relVel, Real& force, Vector3D& forceDirection',
            description=r'compute connector force and further properties (relative position, etc.) for unique functionality and output'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionForce',
            args='Real& force, const MainSystemBase& mainSystem, Real t, Index itemIndex, Real deltaL, Real deltaL_t',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = diameter of spring; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectConnectorCartesianSpringDamper   ++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectConnectorCartesianSpringDamper',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCObjectConnector,
    classDescription=r'An 3D spring-damper element, providing springs and dampers in three (global) directions (x,y,z); the connector can be attached to position-based markers.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | current global position which is provided by marker m0 |
    | marker m1 position | $\LU{0}{\pv}_{m1}$ |  |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ |  |

    <!--+++++++++++++++++++++++++++++++++++++++++++++++++++ -->

    #### Connector forces

    Connector forces are based on relative displacements and relative veolocities in global coordinates.
    Relative displacement between marker m0 to marker m1 positions is given by

    $$
    \Delta\! \LU{0}{\pv}= \LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0} \, ,
    $$ (eq-objectcartesianspringdamper-deltapos)

    and relative velocity reads

    $$
    \Delta\! \LU{0}{\vv}= \LU{0}{\vv}_{m1} - \LU{0}{\vv}_{m0} \, .
    $$

    If \texttt{activeConnector = True}, the spring force vector is computed as

    $$
    \LU{0}{\fv_{SD}} = \diag(\kv)\cdot(\Delta\! \LU{0}{\pv}-\LU{0}{\vv_{\mathrm{off}}}) + \diag(\dv) \cdot \Delta\! \LU{0}{\vv} \, .
    $$

    If the springForceUserFunction $\mathrm{UF}$ is defined, $\fv_{SD}$ instead becomes ($t$ is current time)

    $$
    \LU{0}{\fv_{SD}} = \mathrm{UF}(mbs, t, i_N, \Delta\! \LU{0}{\pv}, \Delta\! \LU{0}{\vv}, \kv, \dv, \vv_{\mathrm{off}}) \, ,
    $$

    and \texttt{iN} represents the itemNumber (=objectNumber).
    If \texttt{activeConnector = False}, $\fv_{SD}$ is set to zero.
    <!--+++++++++++++++++++++++++++++++++++++++++++++++++++ -->

    The force $\fv_{SD}$ acts via the markers' position jacobians $\Jm_{pos,m0}$ and $\Jm_{pos,m1}$.
    The generalized forces added to the ABRV:LHS equations read for marker $m0$,

    $$
    \fv_{LHS,m0} = -\LU{0}{\Jm_{pos,m0}\tp} \LU{0}{\fv_{SD}} \, ,
    $$

    and for marker $m1$,

    $$
    \fv_{LHS,m1} =  \LU{0}{\Jm_{pos,m1}\tp} \LU{0}{\fv_{SD}} \, .
    $$

    The ABRV:LHS equation parts are added accordingly using the ABRV:LTG mapping.
    Note that the different signs result from the signs in {eq}`eq-objectcartesianspringdamper-deltapos`.

    The connector also provides an analytic jacobian, which is used if \texttt{newton.numericalDifferentiation.forODE2 = False} 
    and if there is no springForceUserFunction (otherwise numerical differentiation is used).
    
    The anayltic jacobian for the coupled equation parts $\fv_{LHS,m0}$ and $\fv_{LHS,m1}$ is based on the local jacobians

    $$
    \begin{aligned}
    \Jm_{loc0} &= f_{ODE2}\frac{\partial \LU{0}{\fv_{SD}}}{\partial \LU{0}{\pv}_{m0}} +
                         f_{ODE2_t}\frac{\partial \LU{0}{\fv_{SD}}}{\partial \LU{0}{\vv}_{m0}}
                      = -f_{ODE2} \cdot \diag(\kv) - f_{ODE2_t} \cdot \diag(\dv) \, , \\
          \Jm_{loc1} &= f_{ODE2}\frac{\partial \LU{0}{\fv_{SD}}}{\partial \LU{0}{\pv}_{m1}} +
                         f_{ODE2_t}\frac{\partial \LU{0}{\fv_{SD}}}{\partial \LU{0}{\vv}_{m1}}
                      =  f_{ODE2} \cdot \diag(\kv) + f_{ODE2_t} \cdot \diag(\dv) \, .
    \end{aligned}
    $$

    Here, $f_{ODE2}$ is the factor for the position derivative and $f_{ODE2_t}$ is the factor for the velocity derivative, 
    which allows a computation of the computation for both the position as well as the velocity part at the same time.

    \noindent The complete jacobian for the ABRV:LHS equations then reads,

    $$
    \begin{aligned}
    \Jm_{CSD}&=\mp{\displaystyle \frac{\partial \fv_{LHS,m0}}{\partial  \qv_{m0}}}
                      {\displaystyle \frac{\partial \fv_{LHS,m0}}{\partial  \qv_{m1}}}
                      {\displaystyle \frac{\partial \fv_{LHS,m1}}{\partial  \qv_{m0}}}
                      {\displaystyle \frac{\partial \fv_{LHS,m1}}{\partial  \qv_{m1}}} + \Jm_{CSD'} \\
                &= \mp{-\LU{0}{\Jm_{pos,m0}\tp} \Jm_{loc0} \Jm_{pos,m0}}
                       {-\LU{0}{\Jm_{pos,m0}\tp} \Jm_{loc1} \Jm_{pos,m1}}
                       {\LU{0}{\Jm_{pos,m1}\tp} \Jm_{loc0} \Jm_{pos,m0}}
                       {\LU{0}{\Jm_{pos,m1}\tp} \Jm_{loc1} \Jm_{pos,m1}} + \Jm_{CSD'} \\
                &= \mp{\LU{0}{\Jm_{pos,m0}\tp} \Jm_{loc1} \Jm_{pos,m0}}
                       {-\LU{0}{\Jm_{pos,m0}\tp} \Jm_{loc1} \Jm_{pos,m1}}
                       {-\LU{0}{\Jm_{pos,m1}\tp} \Jm_{loc1} \Jm_{pos,m0}}
                       {\LU{0}{\Jm_{pos,m1}\tp} \Jm_{loc1} \Jm_{pos,m1}} + \Jm_{CSD'}
    \end{aligned}
    $$

    Here, $\qv_{m0}$ are the coordinates associated with marker $m0$ and $\qv_{m1}$ of marker $m1$.

    The second term $\Jm_{CSD'}$ is only non-zero if $\frac{\partial \LU{0}{\Jm_{pos,i}\tp}}{\partial \qv_{i}}$ is non-zero, using $i \in \{m0, \, m1\}$.
    As the latter terms would require to compute a 3-dimensional array, the second jacobian term is computed as 

    $$
    \Jm_{CSD'} = \mp{-f_{ODE2}\frac{\partial \left(\LU{0}{\Jm_{pos,m0}\tp} \fv' \right)}{\partial \qv_{m0}}}{\Null}{\Null}
                          { f_{ODE2}\frac{\partial \left(\LU{0}{\Jm_{pos,m1}\tp} \fv' \right)}{\partial \qv_{m1}}}
    $$ (eq-objectcartesianspringdamper-jacderiv)

    in which we set $\fv' = \LU{0}{\fv_{SD}}$, but the derivatives in {eq}`eq-objectcartesianspringdamper-jacderiv` are evaluated by setting $\fv' = const$.

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset)`
    A user function, which computes the 3D spring force vector depending on time, object variables (deltaL, deltaL\_t) and object parameters 
    (stiffness, damping, force).
    The object variables are provided to the function using the current values of the SpringDamper object.
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.
    <!-- -->

    | arguments / return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs in which underlying item is defined |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{displacement} | Vector3D | $\Delta\! \LU{0}{\pv}$ |
    | \texttt{velocity} | Vector3D | $\Delta\! \LU{0}{\vv}$ |
    | \texttt{stiffness} | Vector3D | copied from object |
    | \texttt{damping} | Vector3D | copied from object |
    | \texttt{offset} | Vector3D | copied from object |
    | **return value** | Vector3D | list or numpy array of computed spring force |

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    *Example*:
    
```python
#define simple force for spring-damper:
def UFforce(mbs, t, itemNumber, u, v, k, d, offset): 
    return [u[0]*k[0],u[1]*k[1],u[2]*k[2]]

#markerNumbers and parameters taken from mini example
mbs.AddObject(CartesianSpringDamper(markerNumbers = [mGround, mMass], 
                                    stiffness = [k,k,k], 
                                    damping = [0,k*0.05,0], offset = [0,0,0],
                                    springForceUserFunction = UFforce))

```

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    miniExample=r"""    #example with mass at [1,1,0], 5kg under load 5N in -y direction
    k=5000
    nMass = mbs.AddNode(NodePoint(referenceCoordinates=[1,1,0]))
    oMass = mbs.AddObject(MassPoint(physicsMass = 5, nodeNumber = nMass))
    
    mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
    mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [1,1,0]))
    mbs.AddObject(CartesianSpringDamper(markerNumbers = [mGround, mMass], 
                                        stiffness = [k,k,k], 
                                        damping = [0,k*0.05,0], offset = [0,0,0]))
    mbs.AddLoad(Force(markerNumber = mMass, loadVector = [0, -5, 0])) #static solution=-5/5000=-0.001m

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic()

    #check result at default integration time
    exu.sys['testResult'] = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Displacement)[1]
""",
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVDisplacement, r"""$\Delta\! \LU{0}{\pv} = \LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0}$relative displacement in global coordinates"""),
        ItemOutputVariable(OVDistance, r"""$L=|\Delta\! \LU{0}{\pv}|$scalar distance between both marker points"""),
        ItemOutputVariable(OVVelocity, r"""$\Delta\! \LU{0}{\vv} = \LU{0}{\vv}_{m1} - \LU{0}{\vv}_{m0}$relative translational velocity in global coordinates"""),
        ItemOutputVariable(OVForce, r'$\fv_{SD}$joint force in global coordinates, see equations'),
        ],
    pythonShortName='CartesianSpringDamper',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"connector's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'$[m0,m1]\tp$list of markers used in connector'),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='stiffness',
            defaultValue=DVZeroVector3D,
            description=r"""$\kv$stiffness [SI:N/m] of springs; act against relative displacements in 0, 1, and 2-direction"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='damping',
            defaultValue=DVZeroVector3D,
            description=r"""$\dv$damping [SI:N/(m s)] of dampers; act against relative velocities in 0, 1, and 2-direction"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='offset',
            defaultValue=DVZeroVector3D,
            description=r'$\vv_{\mathrm{off}}$offset between two springs'),
        ItemParameter(type=TPyFunctionVector3DmbsScalarIndexScalar4Vector3D, destination=DestComp+DestParam,
            pythonName='springForceUserFunction',
            defaultValue=0,
            description=r"""$\mathrm{UF} \in \Rcal^3$A Python function which computes the 3D force vector between the two marker points, if activeConnector=True; see description below"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('HasUserFunction',
            implementation='return (parameters.springForceUserFunction!=0);'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('ComputeJacobianODE2_ODE2'),
        ItemFunctionDef('ComputeJacobianForce6D'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Position']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeSpringForce',
            args='const MarkerDataStructure& markerData, Index itemIndex, Vector3D& vPos, Vector3D& vVel, Vector3D& fVec',
            description=r'compute spring damper force helper function'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionForce',
            args='Vector3D& force, const MainSystemBase& mainSystem, Real t, Index itemIndex, Vector3D& vPos, Vector3D& vVel',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ConnectorCartesianSpringDamper";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = diameter of spring; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectConnectorRigidBodySpringDamper   ++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectConnectorRigidBodySpringDamper',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCObjectConnector,
    classDescription=r'An 3D spring-damper element acting on relative displacements and relative rotations of two rigid body (position+orientation) markers. It represents a penalty-based rigid joint (or prismatic, revolute, etc.)',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    | input parameter | symbol | description |
    |---|---|---|
    | stiffness | $\kv \in \mathbb{R}^{6\times 6}$ | stiffness in $J0$ coordinates |
    | damping | $\dv \in \mathbb{R}^{6\times 6}$ | damping in $J0$ coordinates |
    | offset | $\LUR{J0}{\vv}{\mathrm{off}} \in \mathbb{R}^{6}$ | offset in $J0$ coordinates |
    | rotationMarker0 | $\LU{m0,J0}{\Rot}$ | rotation matrix which transforms from joint 0 into marker 0 coordinates |
    | rotationMarker1 | $\LU{m1,J1}{\Rot}$ | rotation matrix which transforms from joint 1 into marker 1 coordinates |
    | markerNumbers[0] | $m0$ | global marker number m0 |
    | markerNumbers[1] | $m1$ | global marker number m1 |

    <!--
    definition how output variables are computed:
    \rowTable{Rotation}{$\LU{J0}{\ttheta} = [\theta_0,\theta_1,\theta_2]$}{intrinsicFormulation=False: Tait-Bryan angles retrieved from relative rotation matrix; intrinsicFormulation=True: rotation vector of relative rotation matrix}
    \rowTable{ForceLocal}{$\LU{J0}{\fv}$}{see below}
    \rowTable{TorqueLocal}{$\LU{J0}{\mv}$}{see below}
    -->

    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | current global position which is provided by marker m0 |
    | marker m0 orientation | $\LU{0,m0}{\Rot}$ | current rotation matrix provided by marker m0 |
    | marker m1 position | $\LU{0}{\pv}_{m1}$ | accordingly |
    | marker m1 orientation | $\LU{0,m1}{\Rot}$ | current rotation matrix provided by marker m1 |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ | accordingly |
    | marker m0 velocity | $\LU{m0}{\tomega}_{m0}$ | current local angular velocity vector provided by marker m0 |
    | marker m1 velocity | $\LU{m1}{\tomega}_{m1}$ | current local angular velocity vector provided by marker m1 |
    | Displacement | $\LU{0}{\Delta\pv}$ | $\LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0}$ |
    | Velocity | $\LU{0}{\Delta\vv}$ | $\LU{0}{\vv}_{m1} - \LU{0}{\vv}_{m0}$ |
    | DisplacementLocal | $\LU{J0}{\Delta\pv}$ | $\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \LU{0}{\Delta\pv}$ |
    | VelocityLocal | $\LU{J0}{\Delta\vv}$ | $\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \LU{0}{\Delta\vv}$ |
    | AngularVelocityLocal | $\LU{J0}{\Delta\tomega}$ | $\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \left( \LU{0,m1}{\Rot} \LU{m1}{\tomega} - \LU{0,m0}{\Rot} \LU{m0}{\tomega} \right)$ |

    #### Connector forces

    If \texttt{activeConnector = True}, the vector spring force is computed as

    $$
    \vp{\LU{J0}{\fv_{SD}}}{\LU{J0}{\mv_{SD}}} = \kv \left( \vp{\LU{J0}{\Delta\pv}}{\LU{J0}{\ttheta}} - \LUR{J0}{\vv}{\mathrm{off}}\right) + 
                \dv \vp{\LU{J0}{\Delta\vv}}{\LU{J0}{\Delta\omega}}
    $$

    For the application of joint forces to markers, $[\LU{J0}{\fv_{SD}},\,\LU{J0}{\mv_{SD}}]\tp$ is transformed into global coordinates.
    if \texttt{activeConnector = False}, $\LU{J0}{\fv_{SD}}$ and  $\LU{J0}{\mv_{SD}}$ are set to zero.

    If the springForceTorqueUserFunction $\mathrm{UF}$ is defined and \texttt{activeConnector = True}, 
    $\fv_{SD}$ instead becomes ($t$ is current time)

    $$
    \fv_{SD} = \mathrm{UF}(mbs, t, i_N, \LU{J0}{\Delta\pv}, \LU{J0}{\ttheta}, \LU{J0}{\Delta\vv}, \LU{J0}{\Delta\tomega}, 
                                 \mathrm{stiffness}, \mathrm{damping}, \mathrm{rotationMarker0}, \mathrm{rotationMarker1}, \mathrm{offset})
    $$

    and \texttt{iN} represents the itemNumber (=objectNumber).
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `springForceTorqueUserFunction(mbs, t, itemNumber, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1, offset)`
    A user function, which computes the 6D spring-damper force-torque vector depending on mbs, time, local quantities 
    (displacement, rotation, velocity, angularVelocity, stiffness), which are evaluated at current time, which are relative quantities between 
    both markers and which are defined in joint J0 coordinates. 
    As relative rotations are defined by Tait-Bryan rotation parameters, it is recommended to use this connector for small relative rotations only 
    (except for rotations about one axis).
    Furthermore, the user function contains object parameters (stiffness, damping, rotationMarker0/1, offset).
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.
    
    Detailed description of the arguments and local quantities:
    <!-- -->

    | arguments / return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs in which underlying item is defined |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{displacement} | Vector3D | $\LU{J0}{\Delta\pv}$ |
    | \texttt{rotation} | Vector3D | $\LU{J0}{\ttheta}$ |
    | \texttt{velocity} | Vector3D | $\LU{J0}{\Delta\vv}$ |
    | \texttt{angularVelocity} | Vector3D | $\LU{J0}{\Delta\tomega}$ |
    | \texttt{stiffness} | Vector6D | copied from object |
    | \texttt{damping} | Vector6D | copied from object |
    | \texttt{rotJ0} | Matrix3D | rotationMarker0 copied from object |
    | \texttt{rotJ1} | Matrix3D | rotationMarker1 copied from object |
    | \texttt{offset} | Vector6D | copied from object |
    | **return value** | Vector6D | list or numpy array of computed spring force-torque |

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `postNewtonStepUserFunction(mbs, t, Index itemIndex, dataCoordinates, displacement, rotation, velocity, angularVelocity, stiffness, damping, rotJ0, rotJ1, offset)`
    A user function which computes the error of the PostNewtonStep $\varepsilon_{PN}$, a recommended for stepsize reduction $t_{recom}$ (use values > 0 to recommend step size or values < 0 else; 0 gives minimum step size) 
    and the updated dataCoordinates $\dv^k$ of \texttt{NodeGenericData} $n_d$.
    Except from \texttt{dataCoordinates}, the arguments are the same as in \texttt{springForceTorqueUserFunction}.
    The \texttt{postNewtonStepUserFunction} should be used together with the dataCoordinates in order to implement a active set or switching strategy
    for discontinuous events, such as in contact, friction, plasticity, fracture or similar.
    
    Detailed description of the arguments and local quantities:
    <!-- -->

    | arguments / return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs in which underlying item is defined |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{dataCoordinates} | Vector | $\dv^{k-1} = [d_0^{k-1},\; d_1^{k-1},\; \ldots]$ for previous post Newton step $k-1$ |
    | ... | ... | other arguements see \texttt{springForceTorqueUserFunction} |
    | **return value** | Vector | $\left[\varepsilon_{PN},\; t_{recom},\; d_0^{k},\; d_1^{k}, ...\right]$ where $k$ indicates the current step |

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    *Example*:
    
```python
#define simple force for spring-damper:
def UFforce(mbs, t, itemNumber, displacement, rotation, velocity, angularVelocity, 
            stiffness, damping, rotJ0, rotJ1, offset): 
    k = stiffness #passed as list
    u = displacement
    return [u[0]*k[0][0],u[1]*k[1][1],u[2]*k[2][2], 0,0,0]

#markerNumbers and parameters taken from mini example
mbs.AddObject(RigidBodySpringDamper(markerNumbers = [mGround, mBody], 
                                    stiffness = np.diag([k,k,k, 0,0,0]), 
                                    damping = np.diag([0,k*0.01,0, 0,0,0]), 
                                    offset = [0,0,0, 0,0,0],
                                    springForceTorqueUserFunction = UFforce))

```

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    miniExample=r"""    #example with rigid body at [0,0,0], 1kg under initial velocity
    k=500
    nBody = mbs.AddNode(RigidRxyz(initialVelocities=[0,1e3,0, 0,0,0]))
    oBody = mbs.AddObject(RigidBody(physicsMass=1, physicsInertia=[1,1,1,0,0,0], 
                                    nodeNumber=nBody))
    
    mBody = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nBody))
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                            localPosition = [0,0,0]))
    mbs.AddObject(RigidBodySpringDamper(markerNumbers = [mGround, mBody], 
                                        stiffness = np.diag([k,k,k, 0,0,0]), 
                                        damping = np.diag([0,k*0.01,0, 0,0,0]), 
                                        offset = [0,0,0, 0,0,0]))
    
    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic(exu.SimulationSettings())
    
    #check result at default integration time
    exu.sys['testResult'] = mbs.GetNodeOutput(nBody, exu.OutputVariableType.Displacement)[1] 
""",
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVDisplacementLocal, r"""$\LU{J0}{\Delta\pv}$relative displacement in local joint0 coordinates"""),
        ItemOutputVariable(OVVelocityLocal, OVDVelocityLocalJoint),
        ItemOutputVariable(OVRotation, r"""$\LU{J0}{\ttheta}= [\theta_0,\theta_1,\theta_2]\tp$relative rotation parameters (Tait Bryan Rxyz); these are the angles used for calculation of joint torques (e.g. if cX is the diagonal rotational stiffness, the moment for axis X reads mX=cX*phiX, etc.)"""),
        ItemOutputVariable(OVAngularVelocityLocal, r"""$\LU{J0}{\Delta\tomega}$relative angular velocity in local joint0 coordinates"""),
        ItemOutputVariable(OVForceLocal, r'$\LU{J0}{\fv}$joint force in local joint0 coordinates'),
        ItemOutputVariable(OVTorqueLocal, r'$\LU{J0}{\mv}$joint torque in in local joint0 coordinates'),
        ],
    pythonShortName='RigidBodySpringDamper',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"connector's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'list of markers used in connector'),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n_d$node number of a NodeGenericData (size depends on application) for dataCoordinates for user functions (e.g., implementing contact/friction user function)'),
        ItemParameter(type=TMatrixND(6, 6), destination=DestComp+DestParam,
            pythonName='stiffness',
            defaultValue='Matrix6D(6,6,0.)',
            description=r'stiffness [SI:N/m or Nm/rad] of translational, torsional and coupled springs; act against relative displacements in x, y, and z-direction as well as the relative angles (calculated as Euler angles); in the simplest case, the first 3 diagonal values correspond to the local stiffness in x,y,z direction and the last 3 diagonal values correspond to the rotational stiffness around x,y and z axis'),
        ItemParameter(type=TMatrixND(6, 6), destination=DestComp+DestParam,
            pythonName='damping',
            defaultValue='Matrix6D(6,6,0.)',
            description=r'damping [SI:N/(m/s) or Nm/(rad/s)] of translational, torsional and coupled dampers; very similar to stiffness, however, the rotational velocity is computed from the angular velocity vector'),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp+DestParam,
            pythonName='rotationMarker0',
            defaultValue='EXUmath::unitMatrix3D',
            description=r'local rotation matrix for marker 0; stiffness, damping, etc. components are measured in local coordinates relative to rotationMarker0'),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp+DestParam,
            pythonName='rotationMarker1',
            defaultValue='EXUmath::unitMatrix3D',
            description=r'local rotation matrix for marker 1; stiffness, damping, etc. components are measured in local coordinates relative to rotationMarker1'),
        ItemParameter(type=TVectorND(6), destination=DestComp+DestParam,
            pythonName='offset',
            defaultValue='Vector6D({0.,0.,0.,0.,0.,0.})',
            description=r'translational and rotational offset considered in the spring force calculation'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='intrinsicFormulation',
            defaultValue=False,
            description=r'if True, the joint uses the intrinsic formulation, which is independent on order of markers, using a mid-point and mid-rotation for evaluation and application of connector forces and torques; this uses a Lie group formulation; in this case, the force/torque vector is computed from the stiffness matrix times the 6-vector of the SE3 matrix logarithm between the two marker positions/rotations, see the equations'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemParameter(type=TPyFunctionVector6DmbsScalarIndex4Vector3D2Matrix6D2Matrix3DVector6D, destination=DestComp+DestParam,
            pythonName='springForceTorqueUserFunction',
            defaultValue=0,
            description=r"""$\mathrm{UF} \in \Rcal^6$A Python function which computes the 6D force-torque vector (3D force + 3D torque) between the two rigid body markers, if activeConnector=True; see description below"""),
        ItemParameter(type=TPyFunctionVectorMbsScalarIndex4VectorVector3D2Matrix6D2Matrix3DVector6D, destination=DestComp+DestParam,
            pythonName='postNewtonStepUserFunction',
            defaultValue=0,
            description=r"""$\mathrm{UF}_{PN} \in \Rcal$A Python function which computes the error of the PostNewtonStep; see description below"""),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return (parameters.postNewtonStepUserFunction!=0);'),
        ItemRequestedTypes('Node', ['GenericData']),
        ItemFunctionDef('HasUserFunction',
            implementation='return (parameters.springForceTorqueUserFunction!=0);'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Position', 'Orientation']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ConnectorRigidBodySpringDamper";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunctionDef('HasDiscontinuousIteration',
            implementation='return (parameters.postNewtonStepUserFunction!=0);'),
        ItemFunctionDef('PostNewtonStep'),
        ItemFunctionDef('PostDiscontinuousIterationStep',
            implementation=''),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeSpringForceTorque',
            args='const MarkerDataStructure& markerData, Index itemIndex, Matrix3D& Ajoint, Vector3D& vLocPos, Vector3D& vLocVel, Vector3D& vLocRot, Vector3D& vLocAngVel, Vector6D& fLocVec6D, bool computeForceTorque=true',
            description=r'compute spring damper force-torque helper function'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionForce',
            args='Vector6D& fLocVec6D, const MainSystemBase& mainSystem, Real t, Index itemIndex, Vector6D& uLoc6D, Vector6D& vLoc6D',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionPostNewtonStep',
            args='Vector& returnValue, const MainSystemBase& mainSystem, Real t, Index itemIndex, Vector& dataCoordinates, Vector6D& uLoc6D, Vector6D& vLoc6D',
            description=r'call to post Newton step user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = diameter of spring; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectConnectorLinearSpringDamper   +++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectConnectorLinearSpringDamper',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCObjectConnector,
    classDescription=r'An linear spring-damper element acting on relative translations along given axis of local joint0 coordinate system. It connects to position and orientation-based markers; the linear spring-damper is intended to act within prismatic joints or in situations where only one translational axis is free; if the two markers rotate relative to each other, the spring-damper will always act in the local joint0 coordinate system.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    <!--
    \rowTable{rotationMarker0}{$\LU{m0,J0}{\Rot}$}{rotation matrix which transforms from joint 0 into marker 0 coordinates}
    \rowTable{rotationMarker1}{$\LU{m1,J1}{\Rot}$}{rotation matrix which transforms from joint 1 into marker 1 coordinates}
    -->

    | input parameter | symbol | description |
    |---|---|---|
    | markerNumbers[0] | $m0$ | global marker number m0 |
    | markerNumbers[1] | $m1$ | global marker number m1 |

    <!--\rowTable{marker m1 orientation}{$\LU{0,m1}{\Rot}$}{current rotation matrix provided by marker m1} -->

    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 orientation | $\LU{0,m0}{\Rot}$ | current rotation matrix provided by marker m0 |
    | marker m0 position | $\LU{0}{\pv_{m0}}$ | current position matrix provided by marker m0 |
    | marker m1 position | $\LU{0}{\pv_{m1}}$ | current position matrix provided by marker m1 |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity vector provided by marker m0 |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ | current global velocity vector provided by marker m1 |
    | relative displacement | $\Delta x = (\LU{0,m0}{\Rot} \LU{m0}{\dv})\tp (\LU{0}{\pv_{m1}} - \LU{0}{\pv_{m0}})$ | scalar relative displacement |
    | relative velocity | $\Delta v = (\LU{0,m0}{\Rot} \LU{m0}{\dv})\tp (\LU{0}{\vv_{m1}} - \LU{0}{\vv_{m0}})$ | scalar relative velocity; note that this only corresponds to the time derivative of $\Delta x$ if the markers only move along the axis (in a prismatic joint) |

    #### Connector forces

    If \texttt{activeConnector = True}, the vector spring force is computed as

    $$
    f_{SD} = k \left(\Delta x - x_\mathrm{off} \right) + d \left(\Delta v - v_\mathrm{off} \right) + f_c
    $$

    if \texttt{activeConnector = False}, $f_{SD}$ is set zero.

    If the springForceUserFunction $\mathrm{UF}$ is defined and \texttt{activeConnector = True}, 
    $f_{SD}$ instead becomes ($t$ is current time)

    $$
    f_{SD} = \mathrm{UF}(mbs, t, i_N, \Delta x, \Delta v, \mathrm{stiffness}, \mathrm{damping}, \mathrm{offset})
    $$

    and \texttt{iN} represents the itemNumber (=objectNumber).
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset)`
    A user function, which computes the scalar torque depending on mbs, time, local quantities 
    (relative displacement, relative velocity), which are evaluated at current time. 
    Furthermore, the user function contains object parameters (stiffness, damping, offset).
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.
    
    Detailed description of the arguments and local quantities:
    <!-- -->

    | arguments / return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs in which underlying item is defined |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{displacement} | Real | $\Delta x$ |
    | \texttt{velocity} | Real | $\Delta v$ |
    | \texttt{stiffness} | Real | copied from object |
    | \texttt{damping} | Real | copied from object |
    | \texttt{offset} | Real | copied from object |
    | **return value** | Real | computed force |

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    *Example*:
    
```python
#define simple cubic force for spring-damper:
def UFforce(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset): 
    k = stiffness #passed as list
    return k*displacement + 0.1*k* displacement**3

#markerNumbers and parameters taken from mini example
mbs.AddObject(LinearSpringDamper(markerNumbers = [mGround, mBody], 
                                 stiffness = k, 
                                 damping = k*0.01, 
                                 offset = 0,
                                 springForceUserFunction = UFforce))

```

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    miniExample=r"""    #example with rigid body at [0,0,0], with torsional load
    k=2e3
    nBody = mbs.AddNode(RigidRxyz())
    oBody = mbs.AddObject(RigidBody(physicsMass=1, physicsInertia=[1,1,1,0,0,0], 
                                    nodeNumber=nBody))
    
    mBody = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nBody))
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                            localPosition = [0,0,0]))
    mbs.AddObject(PrismaticJointX(markerNumbers = [mGround, mBody])) #motion along ground X-axis
    mbs.AddObject(LinearSpringDamper(markerNumbers = [mGround, mBody], axisMarker0=[1,0,0],
                                     stiffness = k, damping = k*0.01, offset = 0))

    #force along x-axis; expect approx. Delta x = 1/k=0.0005
    mbs.AddLoad(Force(markerNumber = mBody, loadVector=[1,0,0])) 

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic(exu.SimulationSettings())
    
    #check result at default integration time
    exu.sys['testResult'] = mbs.GetNodeOutput(nBody, exu.OutputVariableType.Displacement)[0]
""",
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVDisplacementLocal, r"""$\Delta x$(scalar) relative displacement of the spring-damper"""),
        ItemOutputVariable(OVVelocityLocal, r'$\Delta v$(scalar) relative velocity of spring-damper'),
        ItemOutputVariable(OVForceLocal, r"""$f_{SD}$(scalar) spring-damper force"""),
        ],
    pythonShortName='LinearSpringDamper',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"connector's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'$[m0,\, m1]$list of markers used in connector'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='stiffness',
            defaultValue=0.,
            description=r'$k$torsional stiffness [SI:Nm/rad] against relative rotation'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='damping',
            defaultValue=0.,
            description=r'$d$torsional damping [SI:Nm/(rad/s)]'),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='axisMarker0',
            defaultValue='Vector3D({1,0,0})',
            description=r"""$\LU{m0}{\dv}$local axis of spring-damper in marker 0 coordinates; this axis will co-move with marker $m0$; if marker m0 is attached to ground, the spring-damper represents linear equations"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='offset',
            defaultValue=0.,
            description=r"""$x_\mathrm{off}$translational offset considered in the spring force calculation (this can be used as position control input!)"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='velocityOffset',
            defaultValue=0.,
            description=r"""$v_\mathrm{off}$velocity offset considered in the damper force calculation (this can be used as velocity control input!)"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='force',
            defaultValue=0.,
            description=r'$f_c$additional constant force [SI:Nm] added to spring-damper; this can be used to prescribe a force between the two attached bodies (e.g., for actuation and control)'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemParameter(type=TPyFunctionMbsScalarIndexScalar5, destination=DestComp+DestParam,
            pythonName='springForceUserFunction',
            defaultValue=0,
            description=r"""$\mathrm{UF} \in \Rcal$A Python function which computes the scalar force between the two rigid body markers along axisMarker0 in $m0$ coordinates, if activeConnector=True; see description below"""),
        ItemFunctionDef('HasUserFunction',
            implementation='return (parameters.springForceUserFunction!=0);'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Position', 'Orientation']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ConnectorLinearSpringDamper";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeSpringForce',
            args='const MarkerDataStructure& markerData, Index itemIndex, Matrix3D& A0, Real& displacement, Real& velocity, Real& force',
            description=r'compute spring damper force helper function'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionForce',
            args='Real& force, const MainSystemBase& mainSystem, Real t, Index itemIndex, Real displacement, Real velocity',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = diameter of spring; size == -1.f means that default connector size is used'),
        ItemParameter(type=TBool, destination=DestVisu,
            pythonName='drawAsCylinder',
            defaultValue=False,
            description=r'if this flag is True, the spring-damper is represented as cylinder; this may fit better if the spring-damper represents an actuator'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectConnectorTorsionalSpringDamper   ++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectConnectorTorsionalSpringDamper',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCObjectConnector,
    classDescription=r"""An torsional spring-damper element acting on relative rotations around Z-axis of local joint0 coordinate system. It connects to orientation-based markers; if other rotation axis than the local joint0 Z axis shall be used, the joint rotationMarker0 / rotationMarker1 may be used. The joint perfectly extends a RevoluteJoint with a spring-damper, which can also be used to represent feedback control in an elegant and efficient way, by chosing appropriate user functions. It also allows to measure continuous / infinite rotations by making use of a NodeGeneric which compensates $\pm \pi$ jumps in the measured rotation (\texttt{OutputVariableType.Rotation}).""",
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    | input parameter | symbol | description |
    |---|---|---|
    | rotationMarker0 | $\LU{m0,J0}{\Rot}$ | rotation matrix which transforms from joint 0 into marker 0 coordinates |
    | rotationMarker1 | $\LU{m1,J1}{\Rot}$ | rotation matrix which transforms from joint 1 into marker 1 coordinates |
    | markerNumbers[0] | $m0$ | global marker number m0 |
    | markerNumbers[1] | $m1$ | global marker number m1 |
    | nodeNumber | $n0$ | optional node number of a generic node (otherwise exu.InvalidIndex()) |

    <!--definition how output variables are computed: -->

    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 orientation | $\LU{0,m0}{\Rot}$ | current rotation matrix provided by marker m0 |
    | marker m1 orientation | $\LU{0,m1}{\Rot}$ | current rotation matrix provided by marker m1 |
    | marker m0 ang.\ velocity | $\LU{m0}{\tomega}_{m0}$ | current local angular velocity vector provided by marker m0 |
    | marker m1 ang.\ velocity | $\LU{m1}{\tomega}_{m1}$ | current local angular velocity vector provided by marker m1 |
    | AngularVelocityLocal | $\Delta\omega = \left( \LU{J0,m1}{\Rot} \LU{m1}{\tomega} - \LU{J0,m0}{\Rot} \LU{m0}{\tomega} \right)_Z$ | angular velocity around joint0 Z-axis |

    #### Connector forces

    If \texttt{activeConnector = True}, the vector spring force is computed as

    $$
    \tau_{SD} = k \left(\Delta\theta - \theta_\mathrm{off} \right) + d \left(\Delta\omega - \omega_\mathrm{off} \right) + \tau_c
    $$

    if \texttt{activeConnector = False}, $\tau_{SD}$ is set zero.

    If the springTorqueUserFunction $\mathrm{UF}$ is defined and \texttt{activeConnector = True}, 
    $\tau_{SD}$ instead becomes ($t$ is current time)

    $$
    \tau_{SD} = \mathrm{UF}(mbs, t, i_N, \Delta\theta, \Delta\omega, \mathrm{stiffness}, \mathrm{damping}, \mathrm{offset})
    $$

    and \texttt{iN} represents the itemNumber (=objectNumber).
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `springTorqueUserFunction(mbs, t, itemNumber, rotation, angularVelocity, stiffness, damping, offset)`
    A user function, which computes the scalar torque depending on mbs, time, local quantities 
    (relative rotation, relative angularVelocity), which are evaluated at current time. 
    Furthermore, the user function contains object parameters (stiffness, damping, offset).
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.
    
    Detailed description of the arguments and local quantities:
    <!-- -->

    | arguments / return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs in which underlying item is defined |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{rotation} | Real | $\Delta \theta$ |
    | \texttt{angularVelocity} | Real | $\Delta \omega$ |
    | \texttt{stiffness} | Real | copied from object |
    | \texttt{damping} | Real | copied from object |
    | \texttt{offset} | Real | copied from object |
    | **return value** | Real | computed torque |

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    *Example*:
    
```python
#define simple cubic force for spring-damper:
def UFforce(mbs, t, itemNumber, rotation, angularVelocity, stiffness, damping, offset): 
    k = stiffness #passed as list
    u = rotation
    return k*u + 0.1*k*u**3

#markerNumbers and parameters taken from mini example
mbs.AddObject(TorsionalSpringDamper(markerNumbers = [mGround, mBody], 
                                    stiffness = k, 
                                    damping = k*0.01, 
                                    offset = 0,
                                    springTorqueUserFunction = UFforce))

```

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    miniExample=r"""    #example with rigid body at [0,0,0], with torsional load
    k=2e3
    nBody = mbs.AddNode(RigidRxyz())
    oBody = mbs.AddObject(RigidBody(physicsMass=1, physicsInertia=[1,1,1,0,0,0], 
                                    nodeNumber=nBody))
    
    mBody = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nBody))
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                            localPosition = [0,0,0]))
    mbs.AddObject(RevoluteJointZ(markerNumbers = [mGround, mBody])) #rotation around ground Z-axis
    mbs.AddObject(TorsionalSpringDamper(markerNumbers = [mGround, mBody], 
                                        stiffness = k, damping = k*0.01, offset = 0))

    #torque around z-axis; expect approx. phiZ = 1/k=0.0005
    mbs.AddLoad(Torque(markerNumber = mBody, loadVector=[0,0,1])) 

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic(exu.SimulationSettings())
    
    #check result at default integration time
    exu.sys['testResult'] = mbs.GetNodeOutput(nBody, exu.OutputVariableType.Rotation)[2]
""",
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVRotation, r"""$\Delta\theta$relative rotation around the spring-damper Z-coordinate, enhanced to a continuous rotation (infinite rotations $>+\pi$ and $<-\pi$) if a NodeGeneric with 1 coordinate as added"""),
        ItemOutputVariable(OVAngularVelocityLocal, r"""$\Delta\omega$scalar relative angular velocity around joint0 Z-axis"""),
        ItemOutputVariable(OVTorqueLocal, r"""$\tau_{SD}$scalar spring-damper torque around the local joint0 Z-axis"""),
        ],
    pythonShortName='TorsionalSpringDamper',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"connector's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'list of markers used in connector'),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n_d$node number of a NodeGenericData with 1 dataCoordinate for continuous rotation reconstruction; if this node is left to invalid index, it will not be used'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='stiffness',
            defaultValue=0.,
            description=r'$k$torsional stiffness [SI:Nm/rad] against relative rotation'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='damping',
            defaultValue=0.,
            description=r'$d$torsional damping [SI:Nm/(rad/s)]'),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp+DestParam,
            pythonName='rotationMarker0',
            defaultValue='EXUmath::unitMatrix3D',
            description=r'local rotation matrix for marker 0; transforms joint into marker coordinates'),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp+DestParam,
            pythonName='rotationMarker1',
            defaultValue='EXUmath::unitMatrix3D',
            description=r'local rotation matrix for marker 1; transforms joint into marker coordinates'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='offset',
            defaultValue=0.,
            description=r"""$\theta_\mathrm{off}$rotational offset considered in the spring torque calculation (this can be used as rotation control input!)"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='velocityOffset',
            defaultValue=0.,
            description=r"""$\omega_\mathrm{off}$angular velocity offset considered in the damper torque calculation (this can be used as angular velocity control input!)"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='torque',
            defaultValue=0.,
            description=r"""$\tau_c$additional constant torque [SI:Nm] added to spring-damper; this can be used to prescribe a torque between the two attached bodies (e.g., for actuation and control)"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemParameter(type=TPyFunctionMbsScalarIndexScalar5, destination=DestComp+DestParam,
            pythonName='springTorqueUserFunction',
            defaultValue=0,
            description=r"""$\mathrm{UF} \in \Rcal$A Python function which computes the scalar torque between the two rigid body markers in local joint0 coordinates, if activeConnector=True; see description below"""),
        ItemFunctionDef('HasUserFunction',
            implementation='return (parameters.springTorqueUserFunction!=0);'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return (Index)(parameters.nodeNumber != EXUstd::InvalidIndex);'),
        ItemRequestedTypes('Node', ['GenericData']),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Orientation']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ConnectorTorsionalSpringDamper";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunctionDef('HasDiscontinuousIteration',
            implementation='return (parameters.nodeNumber != EXUstd::InvalidIndex);'),
        ItemFunctionDef('PostNewtonStep'),
        ItemFunctionDef('PostDiscontinuousIterationStep',
            implementation=''),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeSpringTorque',
            args='const MarkerDataStructure& markerData, Index itemIndex, Matrix3D& A0all, Real& angle, Real& omega, Real& torque',
            description=r'compute spring damper force-torque helper function'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionForce',
            args='Real& torque, const MainSystemBase& mainSystem, Real t, Index itemIndex, Real angle, Real omega',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = diameter of spring; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectConnectorCoordinateSpringDamper   +++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectConnectorCoordinateSpringDamper',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCObjectConnector,
    classDescription=r"""A 1D (scalar) spring-damper element acting on single ABRV:ODE2 coordinates and connecting to coordinate-based markers. NOTE that the coordinate markers only measure the coordinate (=displacement), but the reference position is not included as compared to position-based markers!; the spring-damper can also act on rotational coordinates.""",
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 coordinate | $q_{m0}$ | current displacement coordinate which is provided by marker m0; does NOT include reference coordinate! |
    | marker m1 coordinate | $q_{m1}$ |  |
    | marker m0 velocity coordinate | $v_{m0}$ | current velocity coordinate which is provided by marker m0 |
    | marker m1 velocity coordinate | $v_{m1}$ |  |

    #### Connector forces

    Displacement between marker m0 to marker m1 coordinates (does NOT include reference coordinates),

    $$
    \Delta q= q_{m1} - q_{m0}
    $$

    and relative velocity,

    $$
    \Delta v= v_{m1} - v_{m0}
    $$

    If \texttt{activeConnector = True}, the scalar spring force vector is computed as

    $$
    f_{SD} = k \left( \Delta q - l_\mathrm{off} \right) + d \cdot \Delta v % + f_\mathrm{friction}
    $$

    If the springForceUserFunction $\mathrm{UF}$ is defined, $\fv_{SD}$ instead becomes ($t$ is current time)

    $$
    f_{SD} = \mathrm{UF}(mbs, t, i_N, \Delta q, \Delta v, k, d, l_\mathrm{off})%, f_\mu, v_\mu)
    $$

    and \texttt{iN} represents the itemNumber (=objectNumber).

    If \texttt{activeConnector = False}, $f_{SD}$ is set to zero.

    NOTE \mysmall{ that until 2023-01-21 (exudyn V1.5.76), the CoordinateSpringDamper included the parameters dryFriction and dryFrictionProportionalZone.
    These parameters have been removed and they are only available in CoordinateSpringDamperExt, HOWEVER, with different names.
    In order to use CoordinateSpringDamperExt instead of the old CoordinateSpringDamper with the same friction behavior, we recoomend:
    \bi
      \item USE CoordinateSpringDamperExt.fDynamicFriction INSTEAD of CoordinateSpringDamper.dryFriction
      \item USE CoordinateSpringDamperExt.frictionProportionalZone INSTEAD of CoordinateSpringDamper.dryFrictionProportionalZone
      \item CoordinateSpringDamperExt.frictionProportionalZone has a different behavior in case that it is zero; 
            thus use 1e-16 in this case, to get as close as possible to previous behaviour
      \item the variables stiffness, damping and offset have the same interpretation in both objects
      \item keep every other friction, sticking and contact variables in CoordinateSpringDamperExt as default values
      \item user functions obtained a new interface in CoordinateSpringDamperExt, which just needs to be adapted
    \ei}
    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    **Userfunction**: `springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset, dryFriction, dryFrictionProportionalZone)`
    A user function, which computes the scalar spring force depending on time, object variables (displacement, velocity) 
    and object parameters .
    The object variables are passed to the function using the current values of the CoordinateSpringDamper object.
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.
    <!-- -->

    | arguments / return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs in which underlying item is defined |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{displacement} | Real | $\Delta q$ |
    | \texttt{velocity} | Real | $\Delta v$ |
    | \texttt{stiffness} | Real | copied from object |
    | \texttt{damping} | Real | copied from object |
    | \texttt{offset} | Real | copied from object |
    | **return value** | Real | scalar value of computed force |

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    *Example*:
    
```python
#see also mini example! NOTE changes above since 2023-01-23
def UFforce(mbs, t, itemNumber, u, v, k, d, offset):
    return k*(u-offset) + d*v

```

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    miniExample=r"""    #define user function:
    #NOTE: removed 2023-01-21: dryFriction, dryFrictionProportionalZone
    def springForce(mbs, t, itemNumber, u, v, k, d, offset): 
        return 0.1*k*u+k*u**3+v*d

    nMass=mbs.AddNode(Point(referenceCoordinates = [2,0,0]))
    massPoint = mbs.AddObject(MassPoint(physicsMass = 5, nodeNumber = nMass))
    
    groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
    nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nMass, coordinate = 0))
    
    #Spring-Damper between two marker coordinates
    mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundMarker, nodeMarker], 
                                         stiffness = 5000, damping = 80, 
                                         springForceUserFunction = springForce)) 
    loadCoord = mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker, load = 1)) #static linear solution:0.002

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic()

    #check result at default integration time
    exu.sys['testResult'] = mbs.GetNodeOutput(nMass, 
                                                 exu.OutputVariableType.Displacement)[0]
""",
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVDisplacement, r'$\Delta q$relative scalar displacement of marker coordinates'),
        ItemOutputVariable(OVVelocity, r'$\Delta v$difference of scalar marker velocity coordinates'),
        ItemOutputVariable(OVForce, r"""$f_{SD}$scalar force in connector"""),
        ],
    pythonShortName='CoordinateSpringDamper',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"connector's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'list of markers used in connector'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='stiffness',
            defaultValue=0.,
            description=r'$k$stiffness [SI:N/m] of spring; acts against relative value of coordinates'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='damping',
            defaultValue=0.,
            description=r'$d$damping [SI:N/(m s)] of damper; acts against relative velocity of coordinates'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='offset',
            defaultValue=0.,
            description=r"""$l_\mathrm{off}$offset between two coordinates (reference length of springs), see equation"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemParameter(type=TPyFunctionMbsScalarIndexScalar5, destination=DestComp+DestParam,
            pythonName='springForceUserFunction',
            defaultValue=0,
            description=r"""$\mathrm{UF} \in \Rcal$A Python function which defines the spring force with 8 parameters, see equations section / see description below"""),
        ItemFunctionDef('HasUserFunction',
            implementation='return (parameters.springForceUserFunction!=0);'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunctionDef('ComputeJacobianODE2_ODE2'),
        ItemFunctionDef('ComputeJacobianForce6D'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Coordinate']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ConnectorCoordinateSpringDamper";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeSpringForce',
            args='const MarkerDataStructure& markerData, Index itemIndex, Real& relPos, Real& relVel, Real& force',
            description=r'compute spring damper force helper function'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionForce',
            args='Real& force, const MainSystemBase& mainSystem, Real t, Index itemIndex, Real relPos, Real relVel',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = diameter of spring; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectConnectorCoordinateSpringDamperExt   ++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectConnectorCoordinateSpringDamperExt',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCObjectConnector,
    classDescription=r"""A 1D (scalar) spring-damper element acting on single ABRV:ODE2 coordinates, same as ObjectConnectorCoordinateSpringDamper but with extended features, such as limit stop and improved friction. It has different user function interface and additional data node as compared to ObjectConnectorCoordinateSpringDamper, but otherwise behaves very similar. The CoordinateSpringDamperExt is very useful for a single axis of a robot or similar machine modelled with a KinematicTree, as it can add friction and limits based on physical properties. It is highly recommended, to use the bristle model for friction with frictionProportionalZone=0 in case of implicit integrators (GeneralizedAlpha) as it converges better.""",
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 coordinate | $q_{m0}$ | current displacement coordinate which is provided by marker m0; does NOT include reference coordinate! |
    | marker m1 coordinate | $q_{m1}$ |  |
    | marker m0 velocity coordinate | $v_{m0}$ | current velocity coordinate which is provided by marker m0 |
    | marker m1 velocity coordinate | $v_{m1}$ |  |

    #### Connector forces

    Displacement between marker m0 to marker m1 coordinates (does NOT include reference coordinates),

    $$
    q= f_1 \cdot q_{m1} - f_0 \cdot q_{m0}
    $$

    and relative velocity,

    $$
    v= f_1 \cdot v_{m1} - f_0 \cdot v_{m0}
    $$

    The friction force is computed from given friction 'force' parameters, as there is no normal force in this model.
    This means, that \texttt{fDynamicFriction} represents $\mu_d \cdot F_N$ in which $\mu_d$ is the friction parameter and 
    $F_N$ is an according normal force.
    
    The friction force is computed for different cases:
    \bi
      \item CASE 1: \texttt{frictionProportionalZone != 0} ($v_\mathrm{reg} \neq 0$): \\
      This case works well for explicit integrators and represents simplified friction. It is suited best, e.g., for drives if considered
      for a specific velocity, but not for the velocity=0 (at which no friction force is produced).
      If $f_{\mu,\mathrm{d}} > 0$ or $f_{\mu,\mathrm{so}} > 0$ or $f_{\mu,\mathrm{v}} != 0$, the Stribeck friction model is used, with

      $$
      f_\mathrm{friction} = \begin{cases} 
                 (f_{\mu,\mathrm{d}} + f_{\mu,\mathrm{so}}) \frac{\Delta v}{v_\mathrm{reg}}, \quad \mathrm{if} \quad |v| <= v_\mathrm{reg} 
                       \quad \mathrm{and} \quad v_\mathrm{reg} \neq 0 \\
                 \mathrm{Sign}(v)\left(f_{\mu,\mathrm{d}} + f_{\mu,\mathrm{so}} \mathrm{e}^{-(|v|-v_{reg})/v_{exp}} + 
                 f_{\mu,\mathrm{v}} (|v|-v_\mathrm{reg}) \right), \quad \mathrm{else}
                 \end{cases}
      $$

    This case does not use a PostNewton iteration (which may be advantageous in constant step size explicit integration, 
    but may be problematic in implicit integration).\\
      \item CASE 2: \texttt{frictionProportionalZone != 0} (or \texttt{useLimitStops=True}): \\
      This case is perfectly suited for implicit integration, as it includes special switching variables that help to 
      avoid numerical problems due to switching (e.g., between stick and slip) during a Newton iteration. 
      In this case, a so-called bristle model is used, which requires the nodeNumber (data node) to be defined by a GenericDataNode, 
      which must contain 3 data variables. In case of sticking, the sticking force results from a spring-damper model with 
      parameters $k_\mathrm{limits}$ and $d_\mathrm{limits}$, which resolves sticking very well. The last sticking position
      is tracked, which allows to change between stick and slip; however, transition means a reduction of accuracy and
      requires additional computation of system Jacobians and Newton or discontinuous iterations.
      This case includes a PostNewton iteration to switch between stick and slip.
    \ei
    In CASE 2, the GenericDataNode has the 3 data variables (friction mode, last sticking position, limit stop state):
    \bi
      \item[0:] friction mode  $d_{\mu}$: 
      \item[]   $d_{\mu}=0$: stick, 
      \item[]   $d_{\mu}=\pm f_\mathrm{slip}$: slip (in according positive or negative direction); $f_\mathrm{slip}$ representing the slipping force
      <!--not possible $d_{\mu}=-2$: undefined; solver should determine -->
      \item[1:] last sticking position  $x_{lsp}$: contains relative coordinate $q$ at last sticking position; in the sticking case, any deviation from that position leads to an additional bristle force  \\

          $$
          f_\mathrm{friction}^* = (q-x_{lsp}) \cdot k_\mathrm{\mu} + v \cdot d_\mathrm{\mu}
          $$

      \item[2:] limit stop state $d_{ls}$: $d_{ls} = 0$: no limit reached (no contact, $d_{ls}<0$: limitStopsLower surpassed, $d_{ls}>0$: limitStopsUpper surpassed; $|d_{ls}|$ contains the penetration value of the soft contact model
    \ei
    Initialization of the GenericDataNode should be done such that the initial state (e.g. stick) is already set within this variable.
    Not doing so may change results (as the solver assumes that the model is already slipping) and requires additional iterations.
    NOTE, that in particular, if $d_{\mu}$ is initilized with 0 (stick) and $x_{lsp}$ (last sticking position) differs largely
    from the current $q$, a large initial force may result. 
    <!--not possible: In this case $d_{\mu}=-2$ is advantageous. -->
    
    The contact force $f_\mathrm{contact}$ is computed if limit stops are reached. 
    The contact is represented by a spring-damper, which is activated as soon as the limit is reached and deactivated, if the limit is left again.
    Contact forces are computed from stiffness $k_\mathrm{limits}$ and damping $d_\mathrm{limits}$, penetration into stop and velocity,

    $$
    f_\mathrm{contact} = 
              \begin{cases} 
                   k_\mathrm{limits} \cdot (q-s_\mathrm{upper}) +  d_\mathrm{limits} \cdot v \quad \mathrm{if} \quad q > s_\mathrm{upper}\\
                   k_\mathrm{limits} \cdot (q-s_\mathrm{lower}) +  d_\mathrm{limits} \cdot v \quad \mathrm{if} \quad q < s_\mathrm{lower}
                   \end{cases}
    $$

    <!-- -->
    NOTE: while a combination of friction and limit stop is possible, it may be wanted to put a friction with 
    \texttt{frictionProportionalZone != 0} and a limit stop into two different objects, as the combined behavior 
    would switch to a PostNewton method for the regularized friction model.
    
    If \texttt{activeConnector = True}, the scalar spring force vector is computed as

    $$
    f_{SD} = k \cdot \left( q - x_\mathrm{off} \right) + d \cdot \left( v - v_\mathrm{off} \right)
          + f_\mathrm{friction} + f_\mathrm{contact}
    $$

    If the springForceUserFunction $\mathrm{UF}$ is defined, $\fv_{SD}$ instead becomes ($t$ is current time)

    $$
    f_{SD} = \mathrm{UF}(mbs, t, i_N, q, v, k, d, x_\mathrm{off}, v_\mathrm{off}, 
                   f_{\mu,\mathrm{d}}, f_{\mu,\mathrm{so}}, v_\mathrm{exp}, f_{\mu,\mathrm{v}}, v_\mathrm{reg})
    $$

    and \texttt{iN} represents the itemNumber (=objectNumber).

    The virtual work of the connector force is computed from the virtual displacement 

    $$
    \delta q = f_1 \cdot \delta q_{m1} - f_0 \cdot \delta q_{m0} \, ,
    $$

    and the virtual work results as

    $$
    \delta W_{SD} = f_{SD} \cdot \delta q
          = f_{SD} \cdot \left( f_1 \cdot \delta q_{m1} - f_0 \cdot \delta q_{m0} \right)
          \, .
    $$

    The generalized (elastic) forces thus read for the markers $m0$ and $m1$,

    $$
    \Qm_{SD, m0} 
          = -f_{SD} \cdot f_0 \cdot \Jm_{coord,m0} \, , \quad
          \Qm_{SD, m1} 
          = f_{SD} \cdot f_1 \cdot \Jm_{coord,m1} \, ,
    $$

    in which $\Jm_{coord,m0}$ and $\Jm_{coord,m1}$ represent the coordinate Jacobians of the respective markers.
    As can be seen in generalized force $\Qm$, the factors $f_0$ and $f_1$ are added accordingly which increase the 
    force on 'slower' coordinates for certain gear ratios.

    If \texttt{activeConnector = False}, $f_{SD}$ is set to zero.
    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    **Userfunction**: `springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset, velocityOffset, 
    fDynamicFriction, fStaticFrictionOffset, exponentialDecayStatic, fViscousFriction, frictionProportionalZone)`
    A user function, which computes the scalar spring force depending on time, object variables (displacement, velocity) 
    and several object parameters.
    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.

    Only a subset of object variables is passed to the function using the current values of the CoordinateSpringDamperExt object.
    For parameters that are not passed via the user function interface, use mbs.GetObject(itemNumber) or, e.g.,
    mbs.GetObjectParameter(itemNumber, 'limitStopsUpper') to obtain these parameters inside the user function.
    <!-- -->

    | arguments / return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs in which underlying item is defined |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{displacement} | Real | $\Delta q$ |
    | \texttt{velocity} | Real | $\Delta v$ |
    | \texttt{stiffness} | Real | copied from object |
    | \texttt{damping} | Real | copied from object |
    | \texttt{offset} | Real | copied from object |
    | \texttt{velocityOffset} | Real | copied from object |
    | \texttt{fDynamicFriction} | Real | copied from object |
    | \texttt{fStaticFrictionOffset} | Real | copied from object |
    | \texttt{exponentialDecayStatic} | Real | copied from object |
    | \texttt{fViscousFriction} | Real | copied from object |
    | \texttt{frictionProportionalZone} | Real | copied from object, also called regularization velocity or regVel |
    | **return value** | Real | scalar value of computed force |

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    *Example*:
    
```python
#see also mini example! 
#For further parameters, use mbs.GetObject(itemNumber) or 
#  e.g. mbs.GetObjectParameter(itemNumber, 'limitStopsUpper')
def UFforce(mbs, t, itemNumber, u, v, k, d, offset, vOffset, muDynamic, myStaticOffset, muExpVel, muViscous, muRegVel):
    return k*(u-offset) + d*v

```

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVDisplacement, r'$\Delta q$relative scalar displacement of marker coordinates'),
        ItemOutputVariable(OVVelocity, r'$\Delta v$difference of scalar marker velocity coordinates'),
        ItemOutputVariable(OVForce, r"""$f_{SD}$scalar spring force"""),
        ],
    pythonShortName='CoordinateSpringDamperExt',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"connector's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'list of markers used in connector'),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'node number of a NodeGenericData for 3 data coordinates (friction mode, last sticking position, limit stop state), see description for details; must exist in case of bristle friction model or limit stops'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='stiffness',
            defaultValue=0.,
            description=r'$k$stiffness [SI:N/m] of spring; acts against relative value of coordinates'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='damping',
            defaultValue=0.,
            description=r'$d$damping [SI:N/(m s)] of damper; acts against relative velocity of coordinates'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='offset',
            defaultValue=0.,
            description=r"""$x_\mathrm{off}$offset between two coordinates (reference length of springs), see equation; it can be used to represent the pre-scribed drive coordinate"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='velocityOffset',
            defaultValue=0.,
            description=r"""$v_\mathrm{off}$offset between two coordinates; used to model D-control of a drive, where damping is not acting against prescribed velocity"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='factor0',
            defaultValue=1.,
            description=r'$f_0$marker 0 coordinate is multiplied with factor0'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='factor1',
            defaultValue=1.,
            description=r'$f_1$marker 1 coordinate is multiplied with factor1'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='fDynamicFriction',
            defaultValue=0.,
            description=r"""$f_{\mu,\mathrm{d}}$dynamic (viscous) friction force [SI:N] against relative velocity when sliding; assuming a normal force $f_N$, the friction force can be interpreted as $f_\mu = \mu f_N$"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='fStaticFrictionOffset',
            defaultValue=0.,
            description=r"""$f_{\mu,\mathrm{so}}$static (dry) friction offset force [SI:N]; assuming a normal force $f_N$, the friction force is limited by $f_\mu \le (\mu_{so} + \mu_d) f_N = f_{\mu_d} + f_{\mu_{so}}$"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='stickingStiffness',
            defaultValue=0.,
            description=r'$k_\mu$stiffness of bristles in sticking case  [SI:N/m]'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='stickingDamping',
            defaultValue=0.,
            description=r'$d_\mu$damping of bristles in sticking case  [SI:N/(m/s)]'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam,
            pythonName='exponentialDecayStatic',
            defaultValue=0.001,
            description=r"""$v_\mathrm{exp}$relative velocity for exponential decay of static friction offset force [SI:m/s] against relative velocity; at $\Delta v = v_\mathrm{exp}$, the static friction offset force is reduced to 36.8\%"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='fViscousFriction',
            defaultValue=0.,
            description=r"""$f_{\mu,\mathrm{v}}$viscous friction force part [SI:N/(m s)], acting against relative velocity in sliding case"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='frictionProportionalZone',
            defaultValue=0.,
            description=r"""$v_\mathrm{reg}$if non-zero, a regularized Stribeck model is used, regularizing friction force around zero velocity - leading to zero friction force in case of zero velocity; this does not require a data node at all; if zero, the bristle model is used, which requires a data node which contains previous friction state and last sticking position"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='limitStopsUpper',
            defaultValue=0.,
            description=r"""$s_\mathrm{upper}$upper (maximum) value [SI:m] of coordinate before limit is activated; defined relative to the two marker coordinates"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='limitStopsLower',
            defaultValue=0.,
            description=r"""$s_\mathrm{lower}$lower (minimum) value [SI:m] of coordinate before limit is activated; defined relative to the two marker coordinates"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='limitStopsStiffness',
            defaultValue=0.,
            description=r"""$k_\mathrm{limits}$stiffness [SI:N/m] of limit stop (contact stiffness); following a linear contact model"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='limitStopsDamping',
            defaultValue=0.,
            description=r"""$d_\mathrm{limits}$damping [SI:N/(m/s)] of limit stop (contact damping); following a linear contact model"""),
        ItemParameter(type=Tbool, destination=DestComp+DestParam,
            pythonName='useLimitStops',
            defaultValue=False,
            description=r'if True, limit stops are considered and parameters must be set accordingly; furthermore, the NodeGenericData must have 3 data coordinates'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemParameter(type=TPyFunctionMbsScalarIndexScalar11, destination=DestComp+DestParam,
            pythonName='springForceUserFunction',
            defaultValue=0,
            description=r"""$\mathrm{UF} \in \Rcal$A Python function which defines the spring force with 8 parameters, see equations section / see description below"""),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return (parameters.nodeNumber==EXUstd::InvalidIndex) ? 0 : 1;'),
        ItemFunctionDef('GetDataVariablesSize',
            implementation='return 3;',
            description='needed in order to create ltg-lists for data variable of connector'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('HasUserFunction',
            implementation='return (parameters.springForceUserFunction!=0);'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunctionDef('ComputeJacobianODE2_ODE2'),
        ItemFunctionDef('HasDiscontinuousIteration',
            implementation='return (( (parameters.fDynamicFriction != 0 || parameters.fStaticFrictionOffset != 0) && parameters.frictionProportionalZone == 0) || parameters.useLimitStops);'),
        ItemFunctionDef('PostNewtonStep'),
        ItemFunctionDef('PostDiscontinuousIterationStep'),
        ItemFunctionDef('ComputeJacobianForce6D'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Coordinate']),
        ItemRequestedTypes('Node', ['GenericData']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ConnectorCoordinateSpringDamperExt";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeSpringForce',
            args='const MarkerDataStructure& markerData, Index itemIndex, Real& relPos, Real& relVel, Real& force',
            description=r'compute spring damper force helper function'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionForce',
            args='Real& force, const MainSystemBase& mainSystem, Real t, Index itemIndex, Real relPos, Real relVel',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = diameter of spring; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectConnectorGravity   ++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectConnectorGravity',
    cParentClass=ParentClassCObjectConnector,
    classDescription=r'A connector for additing forces due to gravitational fields beween two bodies, which can be used for aerospace and small-scale astronomical problems. NOTE: DO NOT USE this connector for adding gravitational forces (loads), which should be using LoadMassProportional, which is acting global and always in the same direction.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities


    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | current global position which is provided by marker m0 |
    | marker m1 position | $\LU{0}{\pv}_{m1}$ |  |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ |  |


    | output variables | symbol | formula |
    |---|---|---|
    | Displacement | $\Delta\! \LU{0}{\pv}$ | $\LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0}$ |
    | Velocity | $\Delta\! \LU{0}{\vv}$ | $\LU{0}{\vv}_{m1} - \LU{0}{\vv}_{m0}$ |
    | Distance | $L$ | $|\Delta\! \LU{0}{\pv}|$ |
    | Force | $\fv$ | see below |

    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->

    #### Connector forces

    <!-- -->
    The unit vector in force direction reads (if $L=0$, singularity can be avoided using regularization),


    $$
    \vv_{f} = \frac{1}{L} \Delta\! \LU{0}{\pv}
    $$

    If \texttt{activeConnector = True}, and $L>=d_{min}$ the gravitational force is computed as


    $$
    f_G = - G \frac{mass_0 \cdot mass_1}{L^2}
    $$

    If \texttt{activeConnector = True}, and $L<d_{min}$ the gravitational force is computed as


    $$
    f_G = - G \frac{mass_0 \cdot mass_1}{L^2+(L-d_{min})^2}
    $$

    which results in a regularization for small distances, which is helpful if there are no restrictions in objects to keep apart.
    If $d_{min}=0$ and $L=0$, there a system error is raised.
    
    The vector of the gravitational force applied at both markers, pointing from marker $m0$ to marker $m1$, finally reads


    $$
    \fv = f_G \vv_{f}
    $$

    The virtual work of the connector force is computed from the virtual displacement 


    $$
    \delta \Delta\! \LU{0}{\pv} = \delta \LU{0}{\pv}_{m1} - \delta \LU{0}{\pv}_{m0} \, ,
    $$

    and the virtual work (not the transposed version here, because the resulting generalized forces shall be a column vector,


    $$
    \delta W_G = \fv \delta \Delta\! \LU{0}{\pv} 
          = -\left( - G \frac{mass_0 \cdot mass_1}{L^2} \right) \left(\delta \LU{0}{\pv}_{m1} - \delta \LU{0}{\pv}_{m0} \right)\tp \vv_{f} 
          \, .
    $$

    The generalized (elastic) forces thus result from


    $$
    \Qm_G = \frac{\partial \LU{0}{\pv}}{\partial \qv_G\tp} \fv 
          \, ,
    $$

    and read for the markers $m0$ and $m1$,


    $$
    \Qm_{G, m0} 
          = -\left( - G \frac{mass_0 \cdot mass_1}{L^2} \right) \Jm_{pos,m0}\tp \vv_{f} , \quad
          \Qm_{G, m1} 
          = \left( - G \frac{mass_0 \cdot mass_1}{L^2} \right) \Jm_{pos,m1}\tp \vv_{f} 
          \, ,
    $$

    where $\Jm_{pos,m1}$ represents the derivative of marker $m1$ w.r.t.\ its associated coordinates $\qv_{m1}$, analogously $\Jm_{pos,m0}$.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    miniExample=r"""    mass0 = 1e25
    mass1 = 1e3
    r = 1e5
    G = 6.6743e-11
    vInit = np.sqrt(G*mass0/r)
    tEnd = (r*0.5*np.pi)/vInit #quarter period
    node0 = mbs.AddNode(NodePoint(referenceCoordinates = [0,0,0])) #star
    node1 = mbs.AddNode(NodePoint(referenceCoordinates = [r,0,0], 
                                  initialVelocities=[0,vInit,0])) #satellite
    oMassPoint0 = mbs.AddObject(MassPoint(nodeNumber = node0, physicsMass=mass0))
    oMassPoint1 = mbs.AddObject(MassPoint(nodeNumber = node1, physicsMass=mass1))
    
    m0 = mbs.AddMarker(MarkerNodePosition(nodeNumber=node0))
    m1 = mbs.AddMarker(MarkerNodePosition(nodeNumber=node1))
    
    mbs.AddObject(ObjectConnectorGravity(markerNumbers=[m0,m1],
                                         mass0 = mass0, mass1=mass1))

    #assemble and solve system for default parameters
    mbs.Assemble()
    sims = exu.SimulationSettings()
    sims.timeIntegration.endTime = tEnd
    mbs.SolveDynamic(sims, solverType=exu.DynamicSolverType.RK67)

    #check result at default integration time
    #expect y=x after one period of orbiting (got: 100000.00000000479)
    exu.sys['testResult'] = mbs.GetNodeOutput(node1, exu.OutputVariableType.Position)[1]/100000
""",
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVDistance, r"""$L$distance between both points"""),
        ItemOutputVariable(OVDisplacement, r"""$\Delta\! \LU{0}{\pv}$relative displacement between both points"""),
        ItemOutputVariable(OVForce, r"""$\fv$gravity force vector, pointing from marker $m0$ to marker $m1$"""),
        ],
    pythonShortName='ConnectorGravity',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"connector's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'$[m0,m1]\tp$list of markers used in connector'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='gravitationalConstant',
            defaultValue=6.6743e-11,
            description=r'$G$gravitational constant [SI:m$^3$kg$^{-1}$s$^{-2}$)]; while not recommended, a negative constant gan represent a repulsive force'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='mass0',
            defaultValue=0.,
            description=r'$mass_0$mass [SI:kg] of object attached to marker $m0$'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='mass1',
            defaultValue=0.,
            description=r'$mass_1$mass [SI:kg] of object attached to marker $m1$'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='minDistanceRegularization',
            defaultValue=0.,
            description=r'$d_{min}$distance [SI:m] at which a regularization is added in order to avoid singularities, if objects come close'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('HasUserFunction',
            implementation='return false;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Position']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ConnectorGravity";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeConnectorProperties',
            args='const MarkerDataStructure& markerData, Index itemIndex, Vector3D& relPos,Real& force, Vector3D& forceDirection',
            description=r'compute connector force and further properties (relative position, etc.) for unique functionality and output'),
        ItemFunctionDef('UpdateGraphics',
            implementation=';'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=False,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = diameter of spring; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectConnectorHydraulicActuatorSimple   ++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectConnectorHydraulicActuatorSimple',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCObjectConnector,
    classDescription=r"""A basic hydraulic actuator with pressure build up equations. The actuator follows a valve input value, which results in a in- or outflow of fluid depending on the pressure difference. Valve values can be prescribed by user functions (not yet available) or with the \texttt{MainSystem} \texttt{PreStepUserFunction(...)}.""",
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities


    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | current global position which is provided by marker m0 |
    | marker m1 position | $\LU{0}{\pv}_{m1}$ |  |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ |  |
    | Displacement | $\Delta\! \LU{0}{\pv}$=$\LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0}$ | The relative vector between marker points, stored as Displacement in output variables |
    | current actuator length | $L$=$|\Delta\! \LU{0}{\pv}|$ | stored as Distance in output variables |
    | time derivative of actuator length | $\dot L$=$\Delta\! \LU{0}{\vv}\tp \vv_{f}$ |  |
    | Velocity | $\Delta\! \LU{0}{\vv}$=$\LU{0}{\vv}_{m1} - \LU{0}{\vv}_{m0}$ | The vectorial relative velocity |
    | Force | $\fv$ | see below |

    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->

    #### Connector forces

    <!-- -->
    The unit vector in force direction reads (raises SysError if $L=0$),


    $$
    \vv_{f} = \frac{1}{L} \Delta\! \LU{0}{\pv}
    $$

    The simple double-acting hydraulic actuator has two pressure chambers, one being denoted with 0 at the
    piston head (nut) and the other at the piston rod side denoted with 1. The pressure $p_0$ acts at the piston head at area $A_0$, 
    while the pressure $p_1$ counteracts on the opposite side with (usually smaller) area $A_1$.
    <!-- -->
    If \texttt{activeConnector = True}, the scalar actuator force (tension = positive) is computed as


    $$
    f_{HA} = -p_0 \cdot A_0 + p_1 \cdot A_1 + v \cdot d_HA
    $$

    where $v$ represents the actuator velocitiy and $d_HA$ is the viscous damping coefficient.

    The vector of the actuator force applied at both markers finally reads


    $$
    \fv = f_{HA}\vv_{f}
    $$

    The virtual work of the connector force is computed from the virtual displacement 


    $$
    \delta \Delta\! \LU{0}{\pv} = \delta \LU{0}{\pv}_{m1} - \delta \LU{0}{\pv}_{m0} \, ,
    $$

    and the virtual work (not the transposed version here, because the resulting generalized forces shall be a column vector),


    $$
    \delta W_{HA} = \fv \delta \Delta\! \LU{0}{\pv} 
          \, .
    $$
    
    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->

    #### Pressure build up equations

    <!-- -->
    The hydraulics model consists of a double-acting piston. It follows the paper of [CITE:RahikainenGonzalezNayaEtAl2020] 
    except for the friction and the additional valve, which are not available here.
    
    The hydraulic actuator contains internal states, namely pressures $p_0$ and $p_1$.
    The ABRV:ODE1 for pressures follows for the the case of laminar flow, based on system and tank pressure,
    valve positions as well as the actuator velocity and position (only for change of volume).
    
    The distance between the two marker points, which are usually the bushings or clevis mounts of the hydraulic cylinder, is
    denoted as $L$. The stroke length $s \in [0, L_s]$ is defined as


    $$
    s = L - L_o
    $$

    such that at zero stroke, the actuator length is $L_o$. The stroke velocity (positive value means extension) reads


    $$
    \dot s = \Delta\! \LU{0}{\vv\tp} \vv_{f}
    $$

    
    If \texttt{useChamberVolumeChange == True}, the volume change due to stroke change will be considered for the
    volume related to the stiffness of the fluid.
    The cylinder volumes in chambers 0 and 1 are then


    $$
    V_{0,cur} = V_{h,0} + A_0 \cdot s, \quad
          V_{1,cur} = V_{h,1} + A_1 \cdot (L_s - s)
    $$

    The effective bulk modulus for chamber $k \in {0,1}$ is computed as follows,


    $$
    K_{k,eff} = \frac{1}{ \frac{1}{K_{oil}} + \frac{V_{k,cur} - V_{h,k}}{V_{k,cur} \cdot K_{cyl}} + \frac{V_{h,k}}{V_{k,cur} \cdot K_{hose}} },
    $$ (eq-hydraulicactuator-effbulkmodulus)

    where we use a slightly different approach from [CITE:RahikainenGonzalezNayaEtAl2020] when computing the volume for the cylinder bulk modulus term for $k=1$.
    
    Note that in case of $K_{cyl}=0$ and/or $K_{hose}=0$, the according fractions in {eq}`eq-hydraulicactuator-effbulkmodulus`  
    are set to zero (which other wise would give infinity).

    Otherwise, if \texttt{useChamberVolumeChange == False}, $V_{0,cur}=V_{h,0}$, $V_{1,cur}=V_{h,1}$ and $K_{k,eff} = K_{oil}$ for chambers $k \in {0,1}$.
    
    The pressure equations (explicit ABRV:ODE1) have the structure


    $$
    \vp{\dot p_0}{\dot p_1} = \vp{f_0(p_0, s, \dot s)}{f_1(p_1, s, \dot s)}
    $$

    and follow for different cases and chambers / valves $k=\{0,1\}$, based on the simple model where 
    \bi
      \item $A_{v,k} = 0$: valve k closed
      \item $A_{v,k} > 0$: valve k opened towards system pressure (pump)
      \item $A_{v,k} < 0$: valve k opened towards tank pressure
    \ei
    Thus, the following equations are used\footnote{while it would happen rarely in regular operation, the arguments of the square roots could become negative; 
    thus, in the implementation we use $\mathrm{sqrts}(x) = \mathrm{sign}(x) \cdot \sqrt{\mathrm{abs}(x)}$.}:


    $$
    \dot p_0 = \frac{K_{0,eff}}{V_{0,cur}} \left( -A_0 \cdot \dot s + A_{v,0} \cdot Q_n \cdot \mathrm{sqrts}(p_s - p_0)  \right)  \quad \mathrm{if} \quad \mathrm A_{v,0} \ge 0
    $$



    $$
    \dot p_0 = \frac{K_{0,eff}}{V_{0,cur}} \left( -A_0 \cdot \dot s + A_{v,0} \cdot Q_n \cdot \mathrm{sqrts}(p_0 - p_t)  \right)  \quad \mathrm{if} \quad \mathrm A_{v,0} < 0
    $$

    <!-- -->


    $$
    \dot p_1 = \frac{K_{1,eff}}{V_{1,cur}} \left(  A_1 \cdot \dot s + A_{v,1} \cdot Q_n \cdot \mathrm{sqrts}(p_s - p_1)  \right)  \quad \mathrm{if} \quad \mathrm A_{v,1} \ge 0
    $$



    $$
    \dot p_1 = \frac{K_{1,eff}}{V_{1,cur}} \left(  A_1 \cdot \dot s + A_{v,1} \cdot Q_n \cdot \mathrm{sqrts}(p_1 - p_t)  \right)  \quad \mathrm{if} \quad \mathrm A_{v,1} < 0
    $$

    
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVDistance, r"""$L = |\Delta\! \LU{0}{\pv}|$distance between both marker points (usually the actuator bushings); current actuator length"""),
        ItemOutputVariable(OVDisplacement, 'relative displacement between both marker points'),
        ItemOutputVariable(OVVelocity, r'$\Delta\! \LU{0}{\vv}$relative velocity between both points'),
        ItemOutputVariable(OVVelocityLocal, r'$\dot L$actuator velocity, the derivative of actuator length'),
        ItemOutputVariable(OVForce, 'force in actuator resulting as the difference of both pressures times according cross sections'),
        ],
    pythonShortName='HydraulicActuatorSimple',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"connector's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'$[m0,m1]\tp$list of markers used in connector'),
        ItemParameter(type=TArrayIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumbers',
            defaultValue='ArrayIndex()',
            description=r"""$\mathbf{n}_n = [n_{ODE1}]\tp$currently a list with one node number of NodeGenericODE1 for 2 hydraulic pressures (reference values for this node must be zero); data node may be added in future for switching"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='offsetLength',
            defaultValue=0.,
            description=r'$L_o$offset length [SI:m] of cylinder, representing minimal distance between the two bushings at stroke=0'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='strokeLength',
            defaultValue=0.,
            description=r'$L_s$stroke length [SI:m] of cylinder, representing maximum extension relative to $L_o$; the measured distance between the markers is $L_s+L_o$'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='chamberCrossSection0',
            defaultValue=0.,
            description=r'$A_0$cross section [SI:m$^2$] of chamber (inner cylinder) at piston head (nut) side (0)'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='chamberCrossSection1',
            defaultValue=0.,
            description=r'$A_1$cross section [SI:m$^2$] of chamber at piston rod side (1); usually smaller than chamberCrossSection0'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='hoseVolume0',
            defaultValue=0.,
            description=r'$V_{h,0}$hose volume [SI:m$^3$] at piston head (nut) side (0); as the effective bulk modulus would go to infinity at stroke length zero, the hose volume must be greater than zero'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='hoseVolume1',
            defaultValue=0.,
            description=r'$V_{h,1}$hose volume [SI:m$^3$] at piston rod side (1); as the effective bulk modulus would go to infinity at max. stroke length, the hose volume must be greater than zero'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='valveOpening0',
            defaultValue=0.,
            description=r"""$A_{v,0}$relative opening of valve $[-1 \ldots 1]$ [SI:1] at piston head (nut) side (0); positive value is valve opening towards system pressure, negative value is valve opening towards tank pressure; zero means closed valve"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='valveOpening1',
            defaultValue=0.,
            description=r"""$A_{v,1}$relative opening of valve $[-1 \ldots 1]$ [SI:1] at piston rod side (1); positive value is valve opening towards system pressure, negative value is valve opening towards tank pressure; zero means closed valve"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='actuatorDamping',
            defaultValue=0.,
            description=r"""$d_{HA}$damping [SI:N/(m$\,$s)] of hydraulic actuator (against actuator axial velocity)"""),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='oilBulkModulus',
            defaultValue=0.,
            description=r'$K_{oil}$bulk modulus of oil [SI:N/(m$^2$)]'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='cylinderBulkModulus',
            defaultValue=0.,
            description=r'$K_{cyl}$bulk modulus of cylinder [SI:N/(m$^2$)]; in fact, this is value represents the effect of the cylinder stiffness on the effective bulk modulus'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='hoseBulkModulus',
            defaultValue=0.,
            description=r'$K_{hose}$bulk modulus of hose [SI:N/(m$^2$)]; in fact, this is value represents the effect of the hose stiffness on the effective bulk modulus'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='nominalFlow',
            defaultValue=0.,
            description=r'$Q_n$nominal flow of oil through valve [SI:m$^3$/s]'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='systemPressure',
            defaultValue=0.,
            description=r'$p_s$system pressure [SI:N/(m$^2$)]'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='tankPressure',
            defaultValue=0.,
            description=r'$p_t$tank pressure [SI:N/(m$^2$)]'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='useChamberVolumeChange',
            defaultValue=False,
            description=r'if True, the pressure build up equations include the change of oil stiffness due to change of chamber volume'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('ComputeODE1RHS'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Position']),
        ItemRequestedTypes('Node', []),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('GetNodeNumber',
            implementation='return parameters.nodeNumbers[localIndex];'),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumbers[localIndex]=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return parameters.nodeNumbers.NumberOfItems();'),
        ItemFunctionDef('GetODE1Size'),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "HydraulicActuatorSimple";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeConnectorProperties',
            args='const MarkerDataStructure& markerData, Index itemIndex, Vector3D& relPos, Vector3D& relVel, Real& linearVelocity, Real& force, Vector3D& forceDirection',
            description=r'compute connector force and further properties (relative position, etc.) for unique functionality and output'),
        ItemFunctionDef('UpdateGraphics'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='cylinderRadius',
            defaultValue=0.05,
            description=r'radius for drawing of cylinder'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='rodRadius',
            defaultValue=0.03,
            description=r'radius for drawing of rod'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='pistonRadius',
            defaultValue=0.04,
            description=r'radius for drawing of piston (if drawn transparent)'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='pistonLength',
            defaultValue=0.001,
            description=r'radius for drawing of piston (if drawn transparent)'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='rodMountRadius',
            defaultValue=0.,
            description=r'radius for drawing of rod mount sphere'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='baseMountRadius',
            defaultValue=0.,
            description=r'radius for drawing of base mount sphere'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='baseMountLength',
            defaultValue=0.,
            description=r'radius for drawing of base mount sphere'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='colorCylinder',
            defaultValue=DVDefaultColor,
            description=r'RGBA cylinder color; if R==-1, use default connector color'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='colorPiston',
            defaultValue='Float4({0.8f,0.8f,0.8f,1.f})',
            description=r'RGBA piston color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectConnectorReevingSystemSprings   +++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectConnectorReevingSystemSprings',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCObjectConnector,
    classDescription=r"""A rD reeving system defined by a list of torque-free and friction-free sheaves or points that are connected with one rope (modelled as massless spring). NOTE that the spring can undergo tension AND compression (in order to avoid compression, use a PreStepUserFunction to turn off stiffness and damping in this case!). The force is assumed to be constant all over the rope. The sheaves or connection points are defined by $nr$ rigid body markers $[m_0, \, m_1, \, \ldots, \, m_{nr-1}]$. At both ends of the rope there may be a prescribed motion coupled to a coordinate marker each, given by $m_{c0}$ and $m_{c1}$ .""",
    classType=ClassTypeObject,
    equations=r"""    <!--
    #### Definition of quantities
    \startTable{input parameter}{symbol}{description}
    \rowTable{stiffness}{$\kv \in \mathbb{R}^{6\times 6}$}{stiffness in $J0$ coordinates}
    \rowTable{TorqueLocal}{$\LU{J0}{\mv}$}{see below}
    \finishTable
    
    -->

    #### General model assumptions

    The \texttt{ConnectorReevingSystemSprings} model is based on a linear elastic, visco-elastic, and mass-less spring which
    is tangent to a list of rolls. The contact with the rolls is friction-less, causing no torque w.r.t.\ the rolling axis of the sheave.
    The force in the rope results from the difference of the total length $L$ compared to the reference length or the rope, which 
    may be changed by adding or subtracting rope length at the end points. All geometric operations are performed in 3D, allowing to model
    simple reeving systems in 3D.
    
    <!--++++++++++++++++++++++++ -->
    

    (fig-reevingsystemsprings-tangents)=
    ```{figure} /docs/figures/CommonTangents3D.png
    :width: 500

    Geometry of common tangent for two spatial circles defined by radii $R_A$ and $R_B$ as well as by the normalized axis vectors $\av_A$ and $\av_B$. The tangent is undefined, if one of the axis vectors is parallel to the vector $\cv$, which connects the two center points. The positive rotation sense is indicated by means of the angular velocities $\omega_A$ and $\omega_B$.
    ```

    <!--++++++++++++++++++++++++ -->

    #### Common tangent of two circles in 3D

    In order to compute the total length of the rope of the reeving system, the tangent of two arbitrary circles in space needs to be computed.
    Considering [](#fig-reevingsystemsprings-tangents), the relations are based on the
    center points of the circles $\pv_A$ and $\pv_B$, the radii $R_A$ and $R_B$ as well as
    the axis vectors $\av_A$ and $\av_B$, the latter vectors also defining the side at which the tangent contacts.
    For the definition of the tangent, the vectors $\rv_A$ and $\rv_B$ need to be computed.
    
    For the special case of $R_A=R_B=0$, it follows that $\rv_A=\pv_A$ and $\rv_B=\pv_B$.
    Otherwise, we first compute the vector between circle centers,

    $$
    \cv = \pv_B - \pv_A, \quad \mathrm{and} \quad \cv_0 = \frac{\cv}{|\cv|} \, ,
    $$

    and obtain the tangent vectors

    $$
    \tv_A = \tv_B = \cv_0 \, ,
    $$

    as well as the normal vectors

    $$
    \nv_A = \av_A \times \cv_0, \quad \mathrm{and} \quad
          \nv_B = \av_B \times \cv_0 \, .
    $$

    Note that the orientation of the axis vectors $\av_A$ and $\av_B$ defines the orientation of the normals.
    By definition, we assume the following conditions,

    $$
    \nv_A\tp \rv_A < 0, \quad \mathrm{and} \quad 
          \nv_B\tp \rv_B < 0 \, .
    $$

    For two circles with equal radius and axes orientations, the angles result in $\varphi_A=\varphi_B=\pi$.
    In general, the unknown vectors $\rv_A$ and $\rv_B$ are computed by means of Newton's method.
    The unknown tangent vector is given as 

    $$
    \tv_c = \pv_B + \rv_B - \pv_A - \rv_A = \cv + \rv_B - \rv_A \, .
    $$

    We now parameterize the two unknown vectors by means of unknown angles $\varphi_A$ and $\varphi_B$,

    $$
    \rv_A = -R_A \left( \cos(\varphi_A) \tv_A - \sin(\varphi_A) \nv_A \right),
          \quad \mathrm{and} \quad 
          \rv_B = -R_B \left( \cos(\varphi_B) \tv_B - \sin(\varphi_B) \nv_B \right) \, .
    $$

    As vectors $\rv_A$ and $\rv_B$ must be perpendicular to $\tv_c$, it follows that

    $$
    \rv_A\tp (\cv + \rv_B - \rv_A) = 0,
          \quad \mathrm{and} \quad 
          \rv_B\tp (\cv + \rv_B - \rv_A) = 0,
    $$

    or

    $$
    \rv_A\tp \cv + \rv_A\tp \rv_B - R_A^2 = 0,
          \quad \mathrm{and} \quad 
          \rv_B\tp \cv - \rv_B\tp\rv_A + R_B^2 = 0 \, .
    $$ (eq-reevingsystemsprings-newton)

    The relations {eq}`eq-reevingsystemsprings-newton` reduce to only one equation, if either $R_A=0$ or $R_B = 0$.
    The equations can be solved by Newton's method by computing the jacobian of $\Jm_{CT}$ of {eq}`eq-reevingsystemsprings-newton` w.r.t.\ the 
    unknown angles $\varphi_A$ and $\varphi_B$. The iterations are started with

    $$
    \varphi_A = \pi \quad \mathrm{and} \quad \varphi_B = \pi,
    $$

    and iterate until the error is below a certain tolerance, for details see the implementation in \texttt{Geometry.h}.
    
    #### Connector forces

    The current rope length results from the configuration of sheaves, including start and end position:

    $$
    L = d_{m_0-m_1} + C_{m_1} + d_{m_1-m_2} + C_{m_2} + \ldots  + d_{m_{nr-2}-m_{nr-1}}
    $$

    in which $d_{...}$ represents the free spans between two sheaves as computed from the common tangent in the previous section,
    and $C_{...}$ represents the length along the circumference of the according marker if the according radius $r$ is non-zero.
    The quantity $C_{...}$ can be computed easily as soon as the radius vectors to the tangents $\rv_A$ and $\rv_B$
    are known. Within a series of tangents, the previous to the current tangent will always enclose an angle between $0$ and $2\cdot \pi$.
    
    In case that \texttt{hasCoordinateMarkers=True}, the total reference length and its derivative result as

    $$
    L_0 = L_{ref} + f_0 \cdot q_{m_{c0}} + f_1 \cdot q_{m_{c1}}, \quad
          \dot L_0 = f_0 \cdot \dot q_{m_{c0}} + f_1 \cdot \dot q_{m_{c1}}, \quad
    $$

    while we set $L_0 = L_{ref}$ and $\dot L_0=0$ otherwise.
    The linear force in the reeving system (assumed to be constant all over the rope) is computed as

    $$
    F_{lin} = (L-L_{0}) \frac{EA}{L_0} + (\dot L - \dot L_0)\frac{DA}{L_0}
    $$

    The rope force is computed from

    $$
    F =   \begin{cases} F_{lin} \quad \mathrm{if} \quad F_{lin} > 0 \\
                              F_{reg} \cdot \mathrm{tanh}(F_{lin}/F_{reg})\quad \mathrm{else} 
                \end{cases}
    $$

    Which allows small compressive forces $F_{reg}$.
    In case that $F_{reg} < 0$, compressive forces are not regularized (linear spring).
    The case $F_{reg} = 0$ will be used in future only in combination with a data node, 
    which allows switching similar as in friction and contact elements.
    
    Note that in case of $L_0=0$, the term $\frac{1}{L_0}$ is replaced by $1000$.
    However, this case must be avoided by the user by choosing appropriate parameters for the system.

    Additional damping may be added via the parameters $DT$ and $DS$, which have to be treated carefully. The shearing parameter may
    be helpful to damp undesired oscillatory shearing motion, however, it may also damp rigid body motion of the overall mechanism.

    Further details are given in the implementation and examples are provided in the \texttt{Examples} and \texttt{TestModels} folders.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVDistance, r"""$L$current total length of rope"""),
        ItemOutputVariable(OVVelocityLocal, r"""$\dot L$scalar time derivative of current total length of rope"""),
        ItemOutputVariable(OVForceLocal, r"""$F$scalar force in reeving system (constant over length of rope)"""),
        ],
    pythonShortName='ReevingSystemSprings',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"connector's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r"""$[m_0, \, m_1, \, \ldots, \, m_{nr-1},\, m_{c0}, \, m_{c1}]\tp$list of position or rigid body markers used in reeving system and optional two coordinate markers ($m_{c0}, \, m_{c1}$); the first marker $m_0$ and the last rigid body marker $m_{nr-1}$ represent the ends of the rope and are directly connected to a position; the markers $m_1, \, \ldots, \, m_{nr-2}$ can be connected to sheaves, for which a radius and an axis can be prescribed. The coordinate markers are optional and represent prescribed length at the rope ends (marker $m_{c0}$ is added length at start, marker $m_{c1}$ is added length at end of the rope in the reeving system)"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='hasCoordinateMarkers',
            defaultValue=False,
            description=r'flag, which determines, the list of markers (markerNumbers) contains two coordinate markers at the end of the list, representing the prescribed change of length at both ends'),
        ItemParameter(type=TVectorND(2), destination=DestComp+DestParam,
            pythonName='coordinateFactors',
            defaultValue='Vector2D({1,1})',
            description=r"""$[f_0,\, f_1]\tp$factors which are multiplied with the values of coordinate markers; this can be used, e.g., to change directions or to transform rotations (revolutions of a sheave) into change of length"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='stiffnessPerLength',
            defaultValue=0.,
            description=r"""$EA$stiffness per length [SI:N/m/m] of rope; in case of cross section $A$ and Young's modulus $E$, this parameter results in $E\cdot A$; the effective stiffness of the reeving system is computed as $EA/L$ in which $L$ is the current length of the rope"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='dampingPerLength',
            defaultValue=0.,
            description=r'$DA$axial damping per length [SI:N/(m/s)/m] of rope; the effective damping coefficient of the reeving system is computed as $DA/L$ in which $L$ is the current length of the rope'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='dampingTorsional',
            defaultValue=0.,
            description=r'$DT$torsional damping [SI:Nms] between sheaves; this effect can damp rotations around the rope axis, pairwise between sheaves; this parameter is experimental'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='dampingShear',
            defaultValue=0.,
            description=r'$DS$damping of shear motion [SI:Ns] between sheaves; this effect can damp motion perpendicular to the rope between each pair of sheaves; this parameter is experimental'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='regularizationForce',
            defaultValue=0.1,
            description=r"""$F_{reg}$small regularization force [SI:N] in order to avoid large compressive forces; this regularization force can either be $<0$ (using a linear tension/compression spring model) or $>0$, which restricts forces in the rope to be always $\ge -F_{reg}$. Note that smaller forces lead to problems in implicit integrators and smaller time steps. For explicit integrators, this force can be chosen close to zero."""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='referenceLength',
            defaultValue=0.,
            description=r'$L_{ref}$reference length for computation of roped force'),
        ItemParameter(type=TVector3DList, destination=DestComp+DestParam,
            pythonName='sheavesAxes',
            defaultValue='Vector3DList()',
            description=r"""$\lv_a = [\LU{m0}{\av_0},\, \LU{m1}{\av_1},\, \ldots ] in [\Rcal^{3}, ...]$list of local vectors axes of sheaves; vectors refer to rigid body markers given in list of markerNumbers; first and last axes are ignored, as they represent the attachment of the rope ends"""),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='sheavesRadii',
            defaultValue='Vector()',
            description=r"""$\lv_r = [r_0,\, r_1,\, \ldots]\tp \in \Rcal^{n}$radius for each sheave, related to list of markerNumbers and list of sheaveAxes; first and last radii must always be zero."""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemParameter(type=TVector3DList, destination=DestComp, cFlags=CFMutable+CFNoInterface,
            pythonName='tempPositionsList',
            defaultValue='Vector3DList()',
            description=r'temporary list of vectors representing the rope local positions'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('RequestedNumberOfMarkers',
            implementation='return 0;'),
        ItemFunctionDef('HasUserFunction',
            implementation='return false;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', []),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ConnectorReevingSystemSprings";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeReevingGeometry',
            args='const MarkerDataStructure& markerData, Index itemIndex, Vector3DList& positionsList, Real& actualLength, Real& actualLength_t, bool storePositions',
            description=r'compute reeving geometry based on tempPositionsList, length, time derivative of length'),
        ItemFunction(type=TReal, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeForce',
            args='Real L, Real L0, Real L_t, Real L0_t, Real EA, Real DA',
            description=r'compute force in reeving system (including damping)'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='ropeRadius',
            defaultValue=0.001,
            description=r'radius of rope'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectConnectorDistance   +++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectConnectorDistance',
    cParentClass=ParentClassCObjectConstraint,
    classDescription=r'Connector which enforces constant or prescribed distance between two bodies/nodes.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities


    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | current global position which is provided by marker m0 |
    | marker m1 position | $\LU{0}{\pv}_{m1}$ | accordingly |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ | accordingly |
    | relative displacement | $\LU{0}{\Delta\pv}$ | $\LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0}$ |
    | relative velocity | $\LU{0}{\Delta\vv}$ | $\LU{0}{\vv}_{m1} - \LU{0}{\vv}_{m0}$ |
    | algebraicVariable | $\lambda_0$ | Lagrange multiplier = force in constraint |


    #### Connector forces constraint equations

    If \texttt{activeConnector = True}, the index 3 algebraic equation reads


    $$
    \left|\LU{0}{\Delta\pv}\right| - d_0 = 0
    $$

    Due to the fact that the force direction is given by


    $$
    \frac{1}{|\LU{0}{\Delta\pv}|}\LU{0}{\Delta\pv} \, ,
    $$

    the prescribed distance $d_0$ may not be zero. This would, otherwise, result in a change of the number of constraints.
    The index 2 (velocity level) algebraic equation reads


    $$
    \left(\frac{\LU{0}{\Delta\pv}}{\left|\LU{0}{\Delta\pv}\right|}\right)\tp \Delta\vv = 0
    $$

    if \texttt{activeConnector = False}, the algebraic equation reads


    $$
    \lambda_0 = 0
    $$

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    miniExample=r"""    #example with 1m pendulum, 50kg under gravity
    nMass = mbs.AddNode(NodePoint2D(referenceCoordinates=[1,0]))
    oMass = mbs.AddObject(MassPoint2D(physicsMass = 50, nodeNumber = nMass))
    
    mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
    mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [0,0,0]))
    oDistance = mbs.AddObject(DistanceConstraint(markerNumbers = [mGround, mMass], distance = 1))
    
    mbs.AddLoad(Force(markerNumber = mMass, loadVector = [0, -50*9.81, 0])) 

    #assemble and solve system for default parameters
    mbs.Assemble()
    
    sims=exu.SimulationSettings()
    sims.timeIntegration.generalizedAlpha.spectralRadius=0.7
    mbs.SolveDynamic(sims)

    #check result at default integration time
    exu.sys['testResult'] = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Position)[0]
""",
    objectType=ObjectTypeConstraint,
    outputVariables=[
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\Delta\pv}$relative displacement in global coordinates"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\Delta\vv}$relative translational velocity in global coordinates"""),
        ItemOutputVariable(OVDistance, r"""$|\LU{0}{\Delta\pv}|$distance between markers (should stay constant; shows constraint deviation)"""),
        ItemOutputVariable(OVForce, r'$\lambda_0$joint force (=scalar Lagrange multiplier)'),
        ],
    pythonShortName='DistanceConstraint',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'$[m0,m1]\tp$list of markers used in connector'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='distance',
            defaultValue=0.,
            description=r'$d_0$prescribed distance [SI:m] of the used markers; must by greater than zero'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return false;'),
        ItemFunctionDef('ComputeAlgebraicEquations',
            args='Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false'),
        ItemFunctionDef('ComputeJacobianAE'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Position']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);',
            description=r'return object type (for node treatment in computation)'),
        ItemFunctionDef('GetAlgebraicEquationsSize',
            implementation='return 1;'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ConnectorDistance";',
            description=r"Get type name of object (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = link size; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectConnectorCoordinate   +++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectConnectorCoordinate',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCObjectConstraint,
    classDescription=r'A coordinate constraint which constrains two (scalar) coordinates of Marker[Node|Body]Coordinates attached to nodes or bodies. The constraint acts directly on coordinates, but does not include reference values, e.g., of nodal values. This constraint is computationally efficient and should be used to constrain nodal coordinates.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 coordinate | $q_{m0}$ | current displacement coordinate which is provided by marker m0; does NOT include reference coordinate! |
    | marker m1 coordinate | $q_{m1}$ |  |
    | marker m0 velocity coordinate | $v_{m0}$ | current velocity coordinate which is provided by marker m0 |
    | marker m1 velocity coordinate | $v_{m1}$ |  |
    | difference of coordinates | $\Delta q = q_{m1} - q_{m0}$ | Displacement between marker m0 to marker m1 coordinates (does NOT include reference coordinates) |
    | difference of velocity coordinates | $\Delta v= v_{m1} - v_{m0}$ |  |

    #### Connector constraint equations

    If \texttt{activeConnector = True}, the index 3 algebraic equation reads

    $$
    \cv(q_{m0}, q_{m1}) = k_{m1} \cdot q_{m1} - q_{m0} - l_\mathrm{off} = 0
    $$

    If the offsetUserFunction $\mathrm{UF}$ is defined, $\cv$ instead becomes ($t$ is current time)

    $$
    \cv(q_{m0}, q_{m1}) = k_{m1} \cdot q_{m1} - q_{m0} -  \mathrm{UF}(mbs, t, i_N, l_\mathrm{off}) = 0
    $$

    The \texttt{activeConnector = True}, index 2 (velocity level) algebraic equation reads

    $$
    \dot \cv(\dot q_{m0}, \dot q_{m1}) = k_{m1} \cdot \dot q_{m1} - \dot q_{m0} - d = 0
    $$

    The factor $d$ in velocity level equations is zero, except if parameters.velocityLevel = True, then $d=l_\mathrm{off}$.
    If velocity level constraints are active and the velocity level offsetUserFunction\_t $\mathrm{UF}_t$ is defined, $\dot \cv$ instead becomes ($t$ is current time)

    $$
    \dot \cv(\dot q_{m0}, \dot q_{m1}) = k_{m1} \cdot \dot q_{m1} - \dot q_{m0} - \mathrm{UF}_t(mbs, t, i_N, l_\mathrm{off}) = 0
    $$

    and \texttt{iN} represents the itemNumber (=objectNumber).
    Note that the index 2 equations are used, if the solver uses index 2 formulation OR if the flag parameters.velocityLevel = True (or both).
    The user functions include dependency on time $t$, but this time dependency is not respected in the computation of initial accelerations. Therefore,
    it is recommended that $\mathrm{UF}$ and $\mathrm{UF}_t$ does not include initial accelerations.

    If \texttt{activeConnector = False}, the (index 1) algebraic equation reads for ALL cases:

    $$
    \cv(\lambda_0) = \lambda_0 = 0
    $$

    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    **Userfunction**: `offsetUserFunction(mbs, t, itemNumber, lOffset)`
    <!-- -->
    A user function, which computes scalar offset for the coordinate constraint, e.g., in order to move a node on a prescribed trajectory.
    It is NECESSARY to use sufficiently smooth functions, having {\bf initial offsets} consistent with {\bf initial configuration} of bodies, 
    either zero or compatible initial offset-velocity, and no initial accelerations.
    The \texttt{offsetUserFunction} is {\bf ONLY used} in case of static computation or index3 (generalizedAlpha) time integration.
    In order to be on the safe side, provide both  \texttt{offsetUserFunction} and  \texttt{offsetUserFunction\_t}.

    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.

    The user function gets time and the offset parameter as an input and returns the computed offset:
    <!-- -->

    | arguments / return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs in which underlying item is defined |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{lOffset} | Real | $l_\mathrm{off}$ |
    | **return value** | Real | computed offset for given time |

    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    **Userfunction**: `offsetUserFunction_t(mbs, t, itemNumber, lOffset)`
    <!-- -->
    A user function, which computes scalar offset {\bf velocity} for the coordinate constraint.
    It is NECESSARY to use sufficiently smooth functions, having {\bf initial offset velocities} consistent with {\bf initial velocities} of bodies.
    The \texttt{offsetUserFunction\_t} is used instead of \texttt{offsetUserFunction} in case of \texttt{velocityLevel = True}, 
    or for index2 time integration and needed for computation of initial accelerations in second order implicit time integrators.

    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.

    The user function gets time and the offset parameter as an input and returns the computed offset velocity:
    <!-- -->

    | arguments / return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs in which underlying item is defined |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{lOffset} | Real | $l_\mathrm{off}$ |
    | **return value** | Real | computed offset velocity for given time |

    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    *Example*:
    
```python
#see also mini example!
from math import sin, cos, pi
def UFoffset(mbs, t, itemNumber, lOffset): 
    return 0.5*lOffset*(1-cos(0.5*pi*t))

def UFoffset_t(mbs, t, itemNumber, lOffset): #time derivative of UFoffset
    return 0.5*lOffset*0.5*pi*sin(0.5*pi*t)

nMass=mbs.AddNode(Point(referenceCoordinates = [2,0,0]))
massPoint = mbs.AddObject(MassPoint(physicsMass = 5, nodeNumber = nMass))

groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nMass, coordinate = 0))

#Spring-Damper between two marker coordinates
mbs.AddObject(CoordinateConstraint(markerNumbers = [groundMarker, nodeMarker], 
                                   offset = 0.1, 
                                   offsetUserFunction = UFoffset, 
                                   offsetUserFunction_t = UFoffset_t)) 

```

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    miniExample=r"""    def OffsetUF(mbs, t, itemNumber, lOffset): #gives 0.05 at t=1
        return 0.5*(1-np.cos(2*3.141592653589793*0.25*t))*lOffset

    nMass=mbs.AddNode(Point(referenceCoordinates = [2,0,0]))
    massPoint = mbs.AddObject(MassPoint(physicsMass = 5, nodeNumber = nMass))
    
    groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
    nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nMass, coordinate = 0))
    
    #Spring-Damper between two marker coordinates
    mbs.AddObject(CoordinateConstraint(markerNumbers = [groundMarker, nodeMarker], 
                                       offset = 0.1, offsetUserFunction = OffsetUF)) 

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic()

    #check result at default integration time
    exu.sys['testResult']  = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Displacement)[0]
""",
    objectType=ObjectTypeConstraint,
    outputVariables=[
        ItemOutputVariable(OVDisplacement, r"""$\Delta q$relative scalar displacement of marker coordinates, not including factorValue1"""),
        ItemOutputVariable(OVVelocity, r"""$\Delta v$difference of scalar marker velocity coordinates, not including factorValue1"""),
        ItemOutputVariable(OVConstraintEquation, r'$\cv$(residuum of) constraint equation'),
        ItemOutputVariable(OVForce, r'$\lambda_0$scalar constraint force (Lagrange multiplier)'),
        ],
    pythonShortName='CoordinateConstraint',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'$[m0,m1]\tp$list of markers used in connector'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='offset',
            defaultValue=0.,
            description=r'$l_\mathrm{off}$An offset between the two values'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='factorValue1',
            defaultValue=1.,
            description=r'$k_{m1}$An additional factor multiplied with value1 used in algebraic equation'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='velocityLevel',
            defaultValue=False,
            description=r"""If true: connector constrains velocities (only works for ABRV:ODE2 coordinates!); offset is used between velocities; in this case, the offsetUserFunction\_t is considered and offsetUserFunction is ignored"""),
        ItemParameter(type=TPyFunctionMbsScalarIndexScalar, destination=DestComp+DestParam,
            pythonName='offsetUserFunction',
            defaultValue=0,
            description=r"""$\mathrm{UF} \in \Rcal$A Python function which defines the time-dependent offset; see description below"""),
        ItemParameter(type=TPyFunctionMbsScalarIndexScalar, destination=DestComp+DestParam,
            pythonName='offsetUserFunction_t',
            defaultValue=0,
            description=r"""$\mathrm{UF}_t \in \Rcal$time derivative of offsetUserFunction; needed for velocity level constraints; see description below"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('HasUserFunction',
            implementation='return (parameters.offsetUserFunction!=0) || (parameters.offsetUserFunction_t!=0);'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return false;'),
        ItemFunctionDef('IsTimeDependent',
            implementation='return (parameters.offsetUserFunction != 0 || parameters.offsetUserFunction_t != 0);'),
        ItemFunctionDef('UsesVelocityLevel',
            implementation='return parameters.velocityLevel;'),
        ItemFunctionDef('ComputeAlgebraicEquations',
            args='Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false'),
        ItemFunctionDef('ComputeJacobianAE'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Coordinate']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);',
            description=r'return object type (for node treatment in computation)'),
        ItemFunctionDef('GetAlgebraicEquationsSize',
            implementation='return 1;'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ConnectorCoordinate";',
            description=r"Get type name of object (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionOffset',
            args='Real& offset, const MainSystemBase& mainSystem, Real t, Index itemIndex',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionOffset_t',
            args='Real& offset, const MainSystemBase& mainSystem, Real t, Index itemIndex',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = link size; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectConnectorCoordinateVector   +++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectConnectorCoordinateVector',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    cParentClass=ParentClassCObjectConstraint,
    classDescription=r"""A constraint which constrains the coordinate vectors of two markers Marker[Node|Object|Body]Coordinates attached to nodes or bodies. The marker uses the objects ABRV:LTG-lists to build the according coordinate mappings.""",
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 coordinate vector | $\qv_{m0} \in \Rcal^{n_{q_{m0}}}$ | coordinate vector provided by marker $m0$; depending on the marker, the coordinates may or may not include reference coordinates |
    | marker m1 coordinate vector | $\qv_{m1} \in \Rcal^{n_{q_{m1}}}$ | coordinate vector provided by marker $m1$; depending on the marker, the coordinates may or may not include reference coordinates |
    | marker m0 velocity coordinate vector | $\dot \qv_{m0} \in \Rcal^{n_{q_{m0}}}$ | velocity coordinate vector provided by marker $m0$ |
    | marker m1 velocity coordinate vector | $\dot \qv_{m1} \in \Rcal^{n_{q_{m1}}}$ | velocity coordinate vector provided by marker $m1$ |
    | number of algebraic equations | $n_{ae}$ | number of algebraic equations must be same as number of rows in $\Xm_{m0}$ and $\Xm_{m1}$ |
    | difference of coordinates | $\Delta \qv = \qv_{m1} - \qv_{m0}$ | Displacement between marker m0 to marker m1 coordinates |
    | difference of velocity coordinates | $\Delta \vv= \dot \qv_{m1} - \dot \qv_{m0}$ |  |

    <!-- -->

    #### Remarks

    The number of algebraic equations depends on the maximum number of rows in $\Xm_{m0}$, $\Ym_{m0}$, $\Xm_{m1}$ and $\Ym_{m1}$. 
    The number of rows of the latter matrices must either be zero or the maximum of these rows.

    The number of columns in $\Xm_{m0}$ (or $\Ym_{m0}$) must agree with the length of the coordinate vector
    $\qv_{m0}$ and the number of columns in $\Xm_{m1}$ (or $\Ym_{m1}$) must agree with the length of the coordinate vector
    $\qv_{m1}$, if these matrices are not empty matrices. 
    If one marker $k$ is a ground marker (node/object), the length of $\qv_{m,k}$ is zero and also the according matrices
    $\Xm_{m,k}$, $\Ym_{m,k}$  have zero size and will not be considered in the computation of the constraint equations.

    <!--
    If the number of rows of $\Xm_{m0}$ plus the number of rows of $\Xm_{m1}$ is
    larger than the total number of coordinates ( $\qv_{m0}$ and  $\qv_{m1}$), the algebraic equations are
    underdetermined and probably not solvable.
    -->

    #### Connector constraint equations

    If \texttt{activeConnector = True} and no \texttt{constraintUserFunction} is defined, the index 3 algebraic equations

    $$
    \cv(\qv_{m0}, \qv_{m1}) = \Xm_{m1} \cdot \qv_{m1} 
          + \Ym_{m1} \cdot \qv^2_{m1} %quadratic terms have been excluded, as it could not be used for Euler Parameter constraints!
          - \Xm_{m0} \cdot\qv_{m0} 
          - \Ym_{m0} \cdot\qv^2_{m0} 
          - \vv_\mathrm{off} = 0
    $$

    Note that the squared coordinates are understood as $\qv^2_{m0} = [q^2_{0,m0}, \; q^2_{1,m0}, \; \ldots]\tp$, same for $\qv^2_{m1}$.

    The index 2 (velocity level) algebraic equation accordingly reads

    $$
    \dot \cv(\dot \qv_{m0}, \dot \qv_{m1}) = \Xm_{m1} \cdot \dot \qv_{m1} 
          + \Ym_{m1} \cdot \dot \qv^2_{m1} 
          - \Xm_{m0} \cdot \dot \qv_{m0} 
          - \Ym_{m0} \cdot \dot \qv^2_{m0} 
          - \dv_\mathrm{off} = 0
    $$

    The vector $\dv$ in velocity level equations is zero, except if \texttt{parameters.velocityLevel = True}, then $\dv=\vv_\mathrm{off}$.

    Note that the index 2 equations are used, if the solver uses index 2 formulation OR if the flag \texttt{parameters.velocityLevel = True} (or both).
    However, the \texttt{constraintUserFunction} has to be chosen accordingly by the user, either as position or as velocity level.
    The user functions include dependency on time $t$, but this time dependency is not respected in the computation of initial accelerations. Therefore,

    If \texttt{activeConnector = False}, the (index 1) algebraic equation reads for ALL cases:

    $$
    \cv(\tlambda) = \tlambda = 0
    $$

    If a \texttt{constraintUserFunction} is defined, it also requires an according \texttt{jacobianUserFunction} (and vice versa).
    <!--
    without $\Km$ and $\Dm$ (these matrices are added internally),
    \be \label{eq_ObjectGenericODE2_Jac}
      \Jm_{user}(mbs, t, i_N, \qv, \dot \qv, f_{ODE2}, f_{ODE2_t}) =
            -f_{ODE2}   \left(\frac{\partial \fv_{user}(mbs, t, i_N,\qv,\dot \qv)}{\partial \qv} \right) -
             f_{ODE2_t} \left(\frac{\partial \fv_{user}(mbs, t, i_N,\qv,\dot \qv)}{\partial \dot \qv} \right)
    \ee
    CoordinateLoads are added for the respective ABRV:ODE2 coordinate on the RHS of the latter equation.
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    **Userfunction**: `constraintUserFunction(mbs, t, itemNumber, q, q_t, velocityLevel)`
    A user function, which computes algebraic equations for the connector based on the marker coordinates stored in \texttt{q} and \texttt{q\_t}.
    Depending on \texttt{velocityLevel}, the user function needs to compute either the position-level (\texttt{velocityLevel=False}) or
    the velocity level (\texttt{velocityLevel=True}) constraint equations.
    Note that for Index 2 solvers, the \texttt{constraintUserFunction} may be called with \texttt{velocityLevel=True} but \texttt{jacobianUserFunction} 
    is called with \texttt{velocityLevel=False}.
    To define the number of algebraic equations, set \texttt{scalingMarker0} as a \texttt{numpy.zeros((nAE,1))} array with \texttt{nAE} being the number algebraic equations. 
    The returned vector of \texttt{constraintUserFunction} must have size \texttt{nAE}.

    Note that itemNumber represents the index of the ObjectGenericODE2 object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs to which object belongs to |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{q} | Vector $\in \Rcal^{(n_{q_{m0}}+n_{q_{m1}})}$ | connector coordinates, subsequently for marker $m0$ and marker $m1$, in current configuration |
    | \texttt{q\_t} | Vector $\in \Rcal^{(n_{q_{m0}}+n_{q_{m1}})}$ | connector velocity coordinates in current configuration |
    | \texttt{velocityLevel} | Bool | velocityLevel as currently stored in connector |
    | **return value** | Vector $\in \Rcal^{n_{ae}}$ | returns vector (numpy array or list) of evaluated constraint equations for connector |

    \vspace{12pt}
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    **Userfunction**: `jacobianUserFunction(mbs, t, itemNumber, q, q_t, velocityLevel)`
    A user function, which computes the jacobian of the algebraic equations w.r.t. the ODE2 coordiantes (ODE2\_t velocity coordinates if \texttt{velocityLevel=True}).
    The jacobian needs to exactly represent the derivative of the constraintUserFunction.
    The returned matrix of \texttt{jacobianUserFunction} must have \texttt{nAE} rows and \texttt{len(q)} columns.

    | arguments /  return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs to which object belongs to |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number $i_N$ of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{q} | Vector $\in \Rcal^{(n_{q_{m0}}+n_{q_{m1}})}$ | connector coordinates, subsequently for marker $m0$ and marker $m1$, in current configuration |
    | \texttt{q\_t} | Vector $\in \Rcal^{(n_{q_{m0}}+n_{q_{m1}})}$ | connector velocity coordinates in current configuration |
    | \texttt{velocityLevel} | Bool | velocityLevel as currently stored in connector |
    | **return value** | MatrixContainer $\in \Rcal^{(n_{q_{m0}}+n_{q_{m1}})\times n_{ae}}$ | returns special jacobian for connector, as exu.MatrixContainer, numpy array or list of lists; use MatrixContainer sparse format for larger matrices to speed up computations; sparse triplets MAY NOT contain zero values! |

    <!--
    \vspace{12pt}
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeConstraint,
    outputVariables=[
        ItemOutputVariable(OVDisplacement, r"""$\Delta \qv$relative scalar displacement of marker coordinates, not including scaling matrices"""),
        ItemOutputVariable(OVVelocity, r"""$\Delta \vv$difference of scalar marker velocity coordinates, not including scaling matrices"""),
        ItemOutputVariable(OVConstraintEquation, r'$\cv$(residuum of) constraint equations'),
        ItemOutputVariable(OVForce, r"""$\tlambda$constraint force vector (vector of Lagrange multipliers), resulting from action of constraint equations"""),
        ],
    pythonShortName='CoordinateVectorConstraint',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'$[m0,m1]\tp$list of markers used in connector'),
        ItemParameter(type=TNumpyMatrix, destination=DestComp+DestParam,
            pythonName='scalingMarker0',
            defaultValue='Matrix()',
            description=r"""$\Xm_{m0} \in \Rcal^{n_{ae} \times n_{q_{m0}}}$linear scaling matrix for coordinate vector of marker 0; matrix provided in Python numpy format"""),
        ItemParameter(type=TNumpyMatrix, destination=DestComp+DestParam,
            pythonName='scalingMarker1',
            defaultValue='Matrix()',
            description=r"""$\Xm_{m1} \in \Rcal^{n_{ae} \times n_{q_{m1}}}$linear scaling matrix for coordinate vector of marker 1; matrix provided in Python numpy format"""),
        ItemParameter(type=TNumpyMatrix, destination=DestComp+DestParam,
            pythonName='quadraticTermMarker0',
            defaultValue='Matrix()',
            description=r"""$\Ym_{m0} \in \Rcal^{n_{ae} \times n_{q_{m0}}}$quadratic scaling matrix for coordinate vector of marker 0; matrix provided in Python numpy format"""),
        ItemParameter(type=TNumpyMatrix, destination=DestComp+DestParam,
            pythonName='quadraticTermMarker1',
            defaultValue='Matrix()',
            description=r"""$\Ym_{m0} \in \Rcal^{n_{ae} \times n_{q_{m0}}}$quadratic scaling matrix for coordinate vector of marker 1; matrix provided in Python numpy format"""),
        ItemParameter(type=TNumpyVector, destination=DestComp+DestParam,
            pythonName='offset',
            defaultValue='Vector()',
            description=r"""$\vv_\mathrm{off} \in \Rcal^{n_{ae}}$offset added to constraint equation; only active, if no userFunction is defined"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='velocityLevel',
            defaultValue=False,
            description=r"""If true: connector constrains velocities (only works for ABRV:ODE2 coordinates!); offset is used between velocities; in this case, the offsetUserFunction\_t is considered and offsetUserFunction is ignored"""),
        ItemParameter(type=TPyFunctionVectorMbsScalarIndex2VectorBool, destination=DestComp+DestParam,
            pythonName='constraintUserFunction',
            defaultValue=0,
            description=r"""$\cv_{user} \in \Rcal^{n_{ae}}$A Python user function which computes the constraint equations; to define the number of algebraic equations, set scalingMarker0 as a numpy.zeros((nAE,1)) array with nAE being the number algebraic equations; see description below"""),
        ItemParameter(type=TPyFunctionMatrixContainerMbsScalarIndex2VectorBool, destination=DestComp+DestParam,
            pythonName='jacobianUserFunction',
            defaultValue=0,
            description=r"""$\Jm_{user} \in \Rcal^{(n_{q_{m0}}+n_{q_{m1}}) \times n_{ae}}$A Python user function which computes the jacobian, i.e., the derivative of the left-hand-side object equation w.r.t.\ the coordinates (times $f_{ODE2}$) and w.r.t.\ the velocities (times $f_{ODE2_t}$). Terms on the RHS must be subtracted from the LHS equation; the respective terms for the stiffness matrix and damping matrix are automatically added; see description below"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('HasUserFunction',
            implementation='return (parameters.constraintUserFunction!=0) || (parameters.jacobianUserFunction!=0);'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return false;'),
        ItemFunctionDef('IsTimeDependent',
            implementation='return false;'),
        ItemFunctionDef('UsesVelocityLevel',
            implementation='return parameters.velocityLevel;'),
        ItemFunctionDef('ComputeAlgebraicEquations',
            args='Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t,  Index itemIndex, bool velocityLevel = false'),
        ItemFunctionDef('ComputeJacobianAE'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Coordinates']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);',
            description=r'return object type (for node treatment in computation)'),
        ItemFunctionDef('GetAlgebraicEquationsSize'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ConnectorCoordinateVector";',
            description=r"Get type name of object (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionConstraint',
            args='Vector& algebraicEquations, const MainSystemBase& mainSystem, Real t, Index itemIndex, const ResizableVector& qMarker0, const ResizableVector& qMarker1, const ResizableVector& qMarker0_t, const ResizableVector& qMarker1_t, bool velocityLevel',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionJacobian',
            args='EXUmath::MatrixContainer& jacobian, const MainSystemBase& mainSystem, Real t, Index itemIndex, const ResizableVector& qMarker0, const ResizableVector& qMarker1, const ResizableVector& qMarker0_t, const ResizableVector& qMarker1_t, bool velocityLevel',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemFunctionDef('UpdateGraphics',
            implementation=';'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectConnectorRollingDiscPenalty   +++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectConnectorRollingDiscPenalty',
    addProtectedC=r"""    static constexpr Index nDataVariables = 3; //number of data variables for tangential and normal contact
""",
    cParentClass=ParentClassCObjectConnector,
    classDescription=r'A (flexible) connector representing a rolling rigid disc (marker 1) on a flat surface (marker 0, ground body, not moving) in global $x$-$y$ plane. The connector is based on a penalty formulation and adds friction and slipping. The contraints works for discs as long as the disc axis and the plane normal vector are not parallel. Parameters may need to be adjusted for better convergence (e.g., dryFrictionProportionalZone). The formulation for the arbitrary disc axis is still under development and needs further testing. Note that the rolling body must have the reference point at the center of the disc.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    <!--
    \rowTable{marker m0 velocity}{$\LU{0}{\vv}_{m0}$}{current global velocity which is provided by marker m0}
    \rowTable{marker m0 angular velocity}{$\LU{0}{\tomega}_{m0}$}{current angular velocity vector provided by marker m0}
    -->

    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | current global position which is provided by marker m0, any ground reference point; currently unused |
    | marker m0 orientation | $\LU{0,m0}{\Rot}$ | current rotation matrix provided by marker m0; currently unused |
    | marker m1 position | $\LU{0}{\pv}_{m1}$ | center of disc |
    | marker m1 orientation | $\LU{0,m1}{\Rot}$ | current rotation matrix provided by marker m1 |
    | data coordinates | $\xv=[x_0,\,x_1,\,x_2]\tp$ | data coordinates for $[x_0,\,x_1]$: hold the sliding velocity in lateral and longitudinal direction of last discontinuous iteration; $x_2$: represents gap of last discontinuous iteration (in contact normal direction) |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ | accordingly |
    | marker m1 angular velocity | $\LU{0}{\tomega}_{m1}$ | current angular velocity vector provided by marker m1 |
    | ground normal vector | $\LU{0}{\vv_{PN}} = \LU{0,m0}{\Am} \LU{m0}{\vv_{PN}}$ | normalized normal vector to the ground body (rotates with marker $m0$ if not fixed to ground) |
    | ground position B | $\LU{0}{\pv}_{B}$ | disc center point projected on ground (normal projection) |
    | ground position C | $\LU{0}{\pv}_{C}$ | contact point of disc with ground |
    | ground velocity C | $\LU{0}{\vv}_{C}$ | velocity of disc at ground contact point (must be zero at end of iteration) |
    | wheel axis vector | $\LU{0}{\wv_1} =\LU{0,m1}{\Rot} \LU{m1}{\wv_{1}} $ | normalized disc axis vector in global coordinates |
    | longitudinal vector | $\LU{0}{\wv_2}$ | vector in longitudinal (motion) direction |
    | contact point vector | $\LU{0}{\wv_3}$ | normalized vector from disc center point in direction of contact point C |
    | lateral vector | $\LU{0}{\wv_{lat}} = \LU{0}{\vv_{PN}} \times \LU{0}{\wv}_2$ | vector in lateral direction, parallel to ground plane |
    | $D1$ transformation matrix | $\LU{0,D1}{\Am} = [\LU{0}{\wv_1},\, \LU{0}{\wv_2},\, \LU{0}{\wv_3}]$ | transformation of special disc coordinates $D1$ to global coordinates |
    | connector forces | $\LU{J1}{\fv}=[f_{t,x},\,f_{t,y},\,f_n]\tp$ | joint force vector at contact point in joint 1 coordinates: x=lateral direction, y=longitudinal direction, z=plane normal (contact normal) |

    
    #### Geometric relations

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    \noindent The main geometrical setup is shown in the following figure:
    

    ```{figure} /docs/figures/ObjectJointRollingDiscSketch.png
    :width: 600
    ```


    First, the contact point $\LU{0}{\pv}_{C}$ must be computed.
    With the helper vector,


    $$
    \LU{0}{\xv} = \LU{0}{\wv}_1 \times \LU{0}{\vv_{PN}}
    $$

    we create a disc coordinate system $D1$ ($\LU{0}{\wv}_1, \; \LU{0}{\wv}_2, \; \LU{0}{\wv}_3$), with the longitudinal direction,


    $$
    \LU{0}{\wv}_2 = \frac{1}{|\LU{0}{\xv}|} \LU{0}{\xv}
    $$

    and the vector to the contact point,


    $$
    \LU{0}{\wv}_3 = \LU{0}{\wv}_1 \times \LU{0}{\wv}_2
    $$

    The vector from marker $m0$ position to the contact point can be computed from


    $$
    \LU{0}{\pv}_{C} = \LU{0}{\pv}_{m1} + r \cdot \LU{0}{\wv}_3 - \LU{0}{\pv}_{m0}
    $$

    The velocity of the contact point at the disc is computed from,


    $$
    \LU{0}{\vv}_{C} = \LU{0}{\vv}_{m1} + \LU{0}{\tomega}_{m1} \times (r\cdot \LU{0}{\wv}_3)
                            - \left(\LU{0}{\vv}_{m0} + \LU{0}{\tomega}_{m0} \times \LU{0}{\pv}_{C} \right)
    $$

    A second coordinate system, denoted as $J1$, is defined by vectors ($\LU{0}{\wv}_{lat}, \; \LU{0}{\wv}_2, \;  \LU{0}{\vv}_{PN}$), using


    $$
    \LU{0}{\wv}_{lat} = \LU{0}{\vv_{PN}} \times \LU{0}{\wv}_2
    $$

    Note that {\bf in the case that} the rolling axis $\LU{0}{\wv}_1$ lies in the rolling plane, we obtain the special case
    $\LU{0}{\wv}_{lat} = \LU{0}{\wv}_1$ and $\LU{0}{\wv}_3 = -\LU{0}{\vv}_{PN}$.
                                                                     
    #### Computation of normal and tangential forces

    The connector forces at the contact point $C$ are computed as follows. 
    The normal contact force reads


    $$
    f_n = \left(k_c \cdot \LU{0}{\pv}_{C} + d_c \cdot \LU{0}{\vv}_{C} \right)\tp \LU{0}{\vv_{PN}} \, .
    $$

    Note that due to the projection onto $\LU{0}{\vv_{PN}}$, this equation also works for inclined planes
    and reference points, that are not at $[0,0,0]\tp$.
    <!-- -->
    The inplane velocity in joint coordinates,


    $$
    \LU{J1}{\vv_t} = [\LU{0}{\vv}_{C}\tp \LU{0}{\wv}_{lat}, \; \LU{0}{\vv}_{C}\tp \LU{0}{\wv}_2 ]\tp \, ,
    $$

    is used for the computation of tangential forces,


    $$
    \LU{J1}{\fv_t} = [f_{t,x} ,\; f_{t,y}]\tp = \LU{J1}{\tmu} \cdot \left( \phi(|\vv_t|,v_\mu) \cdot f_n \cdot \LU{J1}{\ev_t} \right) \, ,
    $$

    with the regularization function, see Geradin and Cardona [CITE:GeradinCardona2001] (Sec.\ 7.9.3), if \texttt{useLinearProportionalZone=False},


    $$
    \phi(v, v_\mu) = 
            \left\{ 
                \begin{array}{ccl}
                    \displaystyle \left( 2-\frac{v}{v_\mu} \right)\frac{v}{v_\mu} & \mathrm{if} & v \le v_\mu \\
                    1 & \mathrm{if} & v > v_\mu \\
                \end{array}
                \right.
    $$

    and the linear regularization function, if \texttt{useLinearProportionalZone=True},


    $$
    \phi(v, v_\mu) = 
            \left\{ 
                \begin{array}{ccl}
                    \displaystyle \frac{v}{v_\mu} & \mathrm{if} & v \le v_\mu \\
                    1 & \mathrm{if} & v > v_\mu \\
                \end{array}
                \right.
    $$

    The direction of tangential slip is given as


    $$
    \LU{J1}{\ev_t} = 
            \left\{ 
                \begin{array}{ccl}
                    \displaystyle \frac{\LU{J1}{\vv_t}}{|\vv_t|} &\mathrm{if}& |\vv_t|>0 \\
                    %\left[0,\; 0\right]\tp &\mathrm{else}& \\
                    \vp{0}{0} &\mathrm{else}& \\
                \end{array}
                \right.
    $$

    The friction coefficient matrix $\LU{J1}{\tmu}$ is given in joint coordinates and computed from


    $$
    \LU{J1}{\tmu} = \mp{\mu_x + d_x \cdot |\vv_t|}{0}{0}{\mu_y + d_y \cdot |\vv_t|}
    $$

    where for isotropic behaviour of surface and wheel, it will give a diagonal matrix with the friction coefficient in the diagonal.
    In case that the dry friction angle $\alpha_t$ is not zero, the $\tmu$ changes to


    $$
    \LU{J1}{\tmu} = \mp{\cos(\alpha_t)}{\sin(\alpha_t)}{-\sin(\alpha_t)}{\cos(\alpha_t)} 
          \mp{\mu_x + d_x \cdot |\vv_t|}{0}{0}{\mu_y + d_y \cdot |\vv_t|} 
          \mp{\cos(\alpha_t)}{-\sin(\alpha_t)}{\sin(\alpha_t)}{\cos(\alpha_t)}
    $$

    <!-- -->

    #### Connector forces

    Finally, the connector forces read in joint coordinates


    $$
    \LU{J1}{\fv} = \vr{f_{t,x}}{f_{t,y}}{f_n}
    $$ (eq-connectorrollingdiscpenalty-forces)

    and in global coordinates, they are computed from


    $$
    \LU{0}{\fv} = f_{t,x}\LU{0}{\wv}_{lat} + f_{t,y} \LU{0}{\wv}_2 + f_n \LU{0}{\vv}_{PN}
    $$

    Due to the fact that the marker positions are not collocated with the contact point, 
    there are additional torques that need to be considered in the action on the body.
    The torque onto the disc (marker $m1$) is computed as


    $$
    \LU{0}{\ttau_{m1}} = (r\cdot \LU{0}{\wv}_3) \times \LU{0}{\fv}
    $$

    The torque onto the ground (marker $m0$) is computed as


    $$
    \LU{0}{\ttau_{m0}} = \LU{0}{\pv}_{C} \times \LU{0}{\fv}
    $$

    Note that if \texttt{activeConnector = False}, we replace {eq}`eq-connectorrollingdiscpenalty-forces` with


    $$
    \LU{J1}{\fv} = \Null
    $$

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}_{G}$current global position of contact point between rolling disc and ground"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv}_{trail}$current velocity of the trail (according to motion of the contact point along the trail!) in global coordinates; this is not the velocity of the contact point!"""),
        ItemOutputVariable(OVVelocityLocal, r"""$\LU{J1}{\vv}$relative slip velocity at contact point in special $J1$ joint coordinates"""),
        ItemOutputVariable(OVForceLocal, r"""$\LU{J1}{\fv} = \LU{0}{[f_{t,x},\, f_{t,y},\, f_{n}]\tp}$contact forces acting on disc, in special $J1$ joint coordinates, see section Connector Forces, $f_{t,x}$ being the lateral force (parallel to ground plane), $f_{t,y}$ being the longitudinal force and $f_{n}$ being the contact normal force"""),
        ItemOutputVariable(OVRotationMatrix, r"""$\LU{0,J1}{\Am} = [\LU{0}{\wv_{lat}},\, \LU{0}{\wv}_2,\, \LU{0}{\vv_{PN}}]$transformation matrix of special joint coordinates $J1$ to global coordinates"""),
        ],
    pythonShortName='RollingDiscPenalty',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker, size=2), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r"""$[m0,m1]\tp$list of markers used in connector; $m0$ represents a point at the plane surface (normal of surface plane defined by planeNormal); the ground can also be a moving rigid body; $m1$ represents the rolling body, which has its reference point (=local position [0,0,0]) at the disc center point"""),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n_d$node number of a NodeGenericData (size=3) for 3 dataCoordinates, needed for discontinuous iteration (friction and contact)'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='discRadius',
            defaultValue=0.,
            description=r'defines the disc radius'),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='discAxis',
            defaultValue='Vector3D({1,0,0})',
            description=r"""$\LU{m1}{\wv_{1}}, \;\; |\LU{m1}{\wv_{1}}| = 1$axis of disc defined in marker $m1$ frame"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='planeNormal',
            defaultValue='Vector3D({0,0,1})',
            description=r"""$\LU{m0}{\vv_{PN}}, \;\; |\LU{m0}{\vv_{PN}}| = 1$normal to the contact / rolling plane (ground); note that the plane reference point can be arbitrarily chosen by the location of the marker $m0$"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='dryFrictionAngle',
            defaultValue=0.,
            description=r"""$\alpha_t$angle [SI:1 (rad)] which defines a rotation of the local tangential coordinates dry friction; this allows to model Mecanum wheels with specified roll angle"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='contactStiffness',
            defaultValue=0.,
            description=r'$k_c$normal contact stiffness [SI:N/m]'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='contactDamping',
            defaultValue=0.,
            description=r'$d_c$normal contact damping [SI:N/(m s)]'),
        ItemParameter(type=TVectorND(2), destination=DestComp+DestParam,
            pythonName='dryFriction',
            defaultValue='Vector2D({0,0})',
            description=r"""$[\mu_x,\mu_y]\tp$dry friction coefficients [SI:1] in local marker 1 joint $J1$ coordinates; if $\alpha_t==0$, lateral direction $l=x$ and forward direction $f=y$; assuming a normal force $f_n$, the local friction force can be computed as $\LU{J1}{\vp{f_{t,x}}{f_{t,y}}} = \vp{\mu_x f_n}{\mu_y f_n}$"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='dryFrictionProportionalZone',
            defaultValue=0.,
            description=r"""$v_\mu$limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations)"""),
        ItemParameter(type=TVectorND(2), destination=DestComp+DestParam,
            pythonName='viscousFriction',
            defaultValue='Vector2D({0,0})',
            description=r"""$[d_x, d_y]\tp$viscous friction coefficients [SI:1/(m/s)] in local marker 1 joint $J1$ coordinates; proportional to slipping velocity, leading to increasing slipping friction force for increasing slipping velocity"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='rollingFrictionViscous',
            defaultValue=0.,
            description=r"""$\mu_r$rolling friction [SI:1], which acts against the velocity of the trail on ground and leads to a force proportional to the contact normal force; currently, only implemented for disc axis parallel to ground!"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='useLinearProportionalZone',
            defaultValue=False,
            description=r'if True, a linear proportional zone is used; the linear zone performs better in implicit time integration as the Jacobian has a constant tangent in the sticking case'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetDataVariablesSize',
            implementation='return nDataVariables;'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemFunctionDef('HasDiscontinuousIteration',
            implementation='return true;'),
        ItemFunctionDef('PostNewtonStep'),
        ItemFunctionDef('PostDiscontinuousIterationStep'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeContactForces',
            args='const MarkerDataStructure& markerData, const CObjectConnectorRollingDiscPenaltyParameters& parameters, bool computeCurrent, Vector3D& pC, Vector3D& vC, Vector3D& wLateral, Vector3D& w2, Vector3D& n0, Vector3D& w3, Vector3D& fContact, Vector2D& localSlipVelocity',
            description=r'compute contact kinematics and contact forces'),
        ItemFunction(type=TVectorND(2), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeSlipForce',
            args='const CObjectConnectorRollingDiscPenaltyParameters& parameters,     const Vector2D& localSlipVelocity, const Vector2D& dataLocalSlipVelocity, Real contactForce',
            description=r'compute slip force vector for specific states'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemRequestedTypes('Marker', ['Position', 'Orientation']),
        ItemRequestedTypes('Node', ['GenericData']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ConnectorRollingDiscPenalty";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='discWidth',
            defaultValue=0.1,
            description=r'width of disc for drawing'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectContactConvexRoll   +++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectContactConvexRoll',
    addIncludesC=r"""constexpr Index CObjectContactConvexRollMaxPolynomialCoefficients = 20; //maximum number of polynomial coefficients, polynomial order needs to be n-1
constexpr Index CObjectContactConvexRollMaxIterationsContact = 20; // maximum number of iterations t find roots of polynomial for contact
constexpr Index CObjectContactConvexRollNEvalConvexityCheck = 1000; // number of equidistant sample points to check convexity of given polynomial at assembly time.
""",
    addProtectedC=r"""    static constexpr Index nDataVariables = 3; //number of data variables for tangential and normal contact
    mutable bool objectIsInitialized; //!< flag which shows that polynomials have not been computed
""",
    author=r'Manzl Peter',
    cParentClass=ParentClassCObjectConnector,
    classDescription=r'A contact connector representing a convex roll (marker 1) on a flat surface (marker 0, ground body, not moving) in global $x$-$y$ plane. The connector is similar to ObjectConnectorRollingDiscPenalty, but includes a (strictly) convex shape of the roll defined by a polynomial. It is based on a penalty formulation and adds friction and slipping. The formulation is still under development and needs further testing. Note that the rolling body must have the reference point at the center of the disc.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    <!--
    \rowTable{marker m0 velocity}{$\LU{0}{\vv}_{m0}$}{current global velocity which is provided by marker m0}
    \rowTable{marker m0 angular velocity}{$\LU{0}{\tomega}_{m0}$}{current angular velocity vector provided by marker m0}
        \rowTable{ground position B}{$\LU{0}{\pv}_{B}$}{roll center point projected on ground (normal projection)}
        \rowTable{ground position C}{$\LU{0}{\pv}_{C}$}{contact point of disc with ground}
        \rowTable{ground velocity C}{$\LU{0}{\vv}_{C}$}{velocity of disc at ground contact point (must be zero at end of iteration)}
        \rowTable{wheel axis vector}{$\LU{0}{\wv}_1 =\LU{0,m1}{\Rot} \cdot [1,0,0]\tp $}{normalized disc axis vector, currently $[1,0,0]\tp$ in local coordinates}
        \rowTable{longitudinal vector}{$\LU{0}{\wv}_2$}{vector in longitudinal (motion) direction}
        \rowTable{lateral vector}{$\LU{0}{\wv}_l = \LU{0}{\vv_{PN}} \times \LU{0}{\wv}_2 = [-\wv_{2,y}, \wv_{2,x}, 0]$}{vector in lateral direction, lies in ground plane}
        \rowTable{contact point vector}{$\LU{0}{\wv}_3$}{normalized vector from disc center point in direction of contact point C}
        \rowTable{connector forces}{$\LU{J1}{\fv}=[f_{t,x},\,f_{t,y},\,f_n]\tp$}{joint force vector at contact point in joint 1 coordinates: x=lateral direction, y=longitudinal direction, z=plane normal (contact normal)}
    -->

    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | current global position which is provided by marker m0, any ground reference point; currently unused |
    | marker m0 orientation | $\LU{0,m0}{\Rot}$ | current rotation matrix provided by marker m0; currently unused |
    | marker m1 position | $\LU{0}{\pv}_{m1}$ | center of roll |
    | Contact position | $\LU{0}{\pv}_{C}$ | Position of the Contact point C in the global frame 0 |
    | Position marker m1 to contact | $\LU{0}{\pv}_{\mathrm{m1, C}}$ | Position of the contact point C relative to the marker m1 in global frame |
    | marker m1 orientation | $\LU{0,m1}{\Rot}$ | current rotation matrix provided by marker m1 |
    | data coordinates | $\xv=[x_0,\,x_1,\,x_2]\tp$ | data coordinates for $[x_0,\,x_1]$: hold the sliding velocity in lateral and longitudinal direction of last discontinuous iteration; $x_2$: represents gap of last discontinuous iteration (in contact normal direction) |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ | current global velocity which is provided by marker m1 |
    | marker m1 angular velocity | $\LU{0}{\tomega}_{m1}$ | current angular velocity vector provided by marker m1 |
    | ground normal vector | $\LU{0}{\nv}$ | normalized normal vector to the (moving, but not rotating) ground, by default [0,0,1] |

    <!-- -->

    #### Geometric relations

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    The geometrical setup is shown in [](#fig-objectcontactconvexroll-sketch). To calculate the contact point of the convex body of revolution the contact (ground) plane is rotated into the local frame of the body. In this local frame in which the generatrix of the body of revolution is described by the polynomial function


    $$
    \mathrm{r}(^bx) = \sum_{i=0}^n k_i \; x^{n-i}
    $$ (eq-connectorconvexrolling-polynomial)

    with the coefficients of the hull $a_i$. As a pre-Check for the contact two spheres are put into both ends of the object with the maximum radius and only if one of these is in contact. The contact point $^{\mathrm{b}}\pv_{\mathrm{m1,C}} $ is calculated relative to the bodies marker \texttt{m1} in the bodies local frame and transformed accordingly. 
    The contact point C can for be calculated convex bodies by matching the derivative of the polynomial $r(^bx)$ with the gradient of the contact plane, shown in [](#fig-objectcontactconvexroll-sketch), explained in detail in [CITE:ManzlGerstmayr2021]. 
    At the contact point a normal force $\fv_{\mathrm{N}} = [ 0 \; 0 \; \mathrm{f}_{\mathrm{N}} ]\tp$  with 


    $$
    \mathrm{f}_{\mathrm{N}} = \begin{cases}
        - (k_c \, z_{\mathrm{pen}} + d_c \,  \dot{z}_{\mathrm{pen}})  &\text{$z_{\mathrm{pen}}>0$} \\ % darstellen dämpfung 
        0 &\text{else} 
        \end{cases}
    $$ (eq-fpencontact)

    acts against the penetration of the ground. The penetration depth $z_{\mathrm{pen}}$ is the z-component of the position vector of the contact point relative to the ground frame ${^0\pv_{\mathrm{C}}}$. 
    

    (fig-objectcontactconvexroll-sketch)=
    ```{figure} /docs/figures/ConvexRolling.png
    :width: 600

    Sketch of the roller Dimensions. The rollers radius $r({^bx})$ is described by the polynomial \texttt{coefficientsHull}.
    ```


    \noindent
    The revolution results in a velocity of 


    $$
    ^{0}\vv_{C} ={^{0}{\tomega_{\mathrm{m1}}}} \times {^{0}{\pv_{\mathrm{m1,\,C}}}}
    $$

    in the contact point, while the tangential component of the velocity of the body itself with the normal Vector to the contact plane $\nv$ follows to


    $$
    \LURU{0}{\vv}{\mathrm{m1,\,t}}{} = \LU{0}{\vv_{\mathrm{m1}}} - {^0\nv} \, \left({^0\nv}^T \, \LU{0}{\vv_{\mathrm{m1}}}\right).
    $$
 
    Therefore the slip velocity of the body can be calculated with


    $$
    \LURU{0}{\vv}{\mathrm{s}}{} = \LURU{0}{\vv}{C}{} - {^0\vv_{\mathrm{m1,\,t}}}
    $$

    and points in the direction 


    $$
    \LURU{0}{\rv}{s}{} = \frac{1}{\left\lVert \LURU{0}{\vv}{\mathrm{s}}{}\right\rVert} {^0{\vv}_{\mathrm{s}}}.
    $$

    \noindent The slip force is then calculated


    $$
    ^0\fv_{\mathrm{s}} = \mu(\left\lVert\LU{}{^0\vv_{\mathrm{s}}}\right\rVert)  \, \mathrm{f}_{\mathrm{N}} \, {^0\rv_\mathrm{s}}
    $$

    and uses for the friction coefficient $\mu$ the regularized friction approach from the StribeckFunction, see [](#sec-module-physics). 
    The torque 


    $$
    ^0\ttau = {^0\pv_{\mathrm{m1,\,C}}} \times (^0\fv_{\mathrm{N}} + {^0\fv_{\mathrm{s}}})
    $$

    acts onto the body, resulting from the slip force acting not in the bodies center. 
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}_{C}$current global position of contact point between roller and ground"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv}_{C}$current velocity of the trail (contact) point in global coordinates; this is the velocity with which the contact moves over the ground plane"""),
        ItemOutputVariable(OVForce, r'$\LU{0}{\fv}$Roll-ground force in ground coordinates'),
        ItemOutputVariable(OVTorque, r'$\LU{0}{\mv}$Roll-ground torque in ground coordinates'),
        ],
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker, size=2), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r"""$[m0,m1]\tp$list of markers used in connector; $m0$ represents the ground, which can undergo translations but not rotations, and $m1$ represents the rolling body, which has its reference point (=local position [0,0,0]) at the roll's center point"""),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n_d$node number of a NodeGenericData (size=3) for 3 dataCoordinates, needed for discontinuous iteration (friction and contact)'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='contactStiffness',
            defaultValue=0.,
            description=r'$k_c$normal contact stiffness [SI:N/m]'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='contactDamping',
            defaultValue=0.,
            description=r'$d_c$normal contact damping [SI:N/(m s)]'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='dynamicFriction',
            defaultValue=0.,
            description=r"""$\mu_d$dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, [](#sec-module-physics)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='staticFrictionOffset',
            defaultValue=0.,
            description=r"""$\mu_{s_off}$static friction offset for friction model (static friction = dynamic friction + static offset), see StribeckFunction in exudyn.physics, [](#sec-module-physics)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='viscousFriction',
            defaultValue=0.,
            description=r"""$\mu_v$viscous friction coefficient (velocity dependent part) for friction model, see StribeckFunction in exudyn.physics, [](#sec-module-physics)"""),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam,
            pythonName='exponentialDecayStatic',
            defaultValue=0.001,
            description=r"""$v_{exp}$exponential decay of static friction offset (must not be zero!), see StribeckFunction in exudyn.physics (named expVel there!), [](#sec-module-physics)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='frictionProportionalZone',
            defaultValue=0.001,
            description=r"""$v_{reg}$limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), [](#sec-module-physics)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='rollLength',
            defaultValue=0.,
            description=r'$L$roll length [m], symmetric w.r.t.\ centerpoint'),
        ItemParameter(type=TNumpyVector, destination=DestComp+DestParam,
            pythonName='coefficientsHull',
            defaultValue=' Vector()',
            description=r"""$\kv \in \Rcal^{n_p}$a vector of polynomial coefficients, which provides the polynomial of the CONVEX hull of the roll; $\mathrm{hull}(x) = k_0 x^{n_p-1} + k x^{n_p-2} + \ldots + k_{n_p-2} x  + k_{n_p-1}$"""),
        ItemParameter(type=TNumpyVector, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='coefficientsHullDerivative',
            defaultValue='Vector()',
            description=r"""$\kv^\prime \in \Rcal^{n_p}$polynomial coefficients of the polynomial $\mathrm{hull}^\prime(x)$"""),
        ItemParameter(type=TNumpyVector, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='coefficientsHullDDerivative',
            defaultValue='Vector()',
            description=r'second derivative of the hull polynomial.'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp,
            pythonName='rBoundingSphere',
            defaultValue=0,
            description=r'The  radius of the bounding sphere for the contact pre-check, calculated from the polynomial coefficients of the hull'),
        ItemParameter(type=TVectorND(3), destination=DestComp, cFlags=CFReadOnly,
            pythonName='pContact',
            defaultValue='Vector3D({0,0,0})',
            description=r'The  current potential contact point. Contact occures if pContact[2] < 0. '),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetDataVariablesSize',
            implementation='return nDataVariables;'),
        ItemFunctionDef('HasDiscontinuousIteration',
            implementation='return true;'),
        ItemFunctionDef('PostNewtonStep'),
        ItemFunctionDef('PostDiscontinuousIterationStep'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeContactForces',
            args='const MarkerDataStructure& markerData, const CObjectContactConvexRollParameters& parameters, Vector3D& pC, Vector3D& vC, Vector3D& fContact, Vector3D& mContact, bool allowSwitching',
            description=r'compute contact kinematics and contact forces; allowSwitching set false for Newton'),
        ItemFunction(type=Tvoid, destination=DestComp, isVirtual=False,
            pythonName='InitializeObject',
            args='const CObjectContactConvexRollParameters& parameters',
            description=r'initialize parameters for contact check'),
        ItemFunction(type=Tbool, destination=DestComp, isVirtual=False,
            pythonName='CheckConvexityOfPolynomial',
            args='const CObjectContactConvexRollParameters& parameters',
            description=r'Check Convexity of the given polynomial before execution'),
        ItemFunction(type=Tbool, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='PreContactCheckRoller',
            args='const Matrix3D& Rotm, const Vector3D& displacement, Real lRoller, Real R, Vector3D& pC',
            description=r'Check if one of the bounding spheres at the end of the roller is in contact with the ground'),
        ItemFunction(type=TVectorND(3), destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='FindContactPoint',
            args='const Matrix3D& Rotm, const Vector& poly, Real lRoller',
            description=r'Find the point of the roller closest the ground, contact occures when return[2] < 0  '),
        ItemFunction(type=TReal, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='PolynomialRollXOfAngle',
            args='const Vector& poly, const Vector& dpoly, Real lRoller, Real angy',
            description=r'PolynomialRollXOFAngle: calculate the x-Value of the polynomial matching the slope of the contact'),
        ItemFunctionDef('ParametersHaveChanged',
            implementation='objectIsInitialized = false;',
            description='This flag is reset upon change of parameters; says that the vector of coordinate indices has changed'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('PostAssemble',
            implementation='InitializeObject(this->parameters);'),
        ItemRequestedTypes('Marker', ['Position', 'Orientation']),
        ItemRequestedTypes('Node', ['GenericData']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ContactConvexRoll";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectContactCoordinate   +++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectContactCoordinate',
    cParentClass=ParentClassCObjectConnector,
    classDescription=r"""A penalty-based contact condition for one coordinate; the contact gap $g$ is defined as $g=marker.value[1]- marker.value[0] - offset$; the contact force $f_c$ is zero for $gap>0$ and otherwise computed from $f_c = g*contactStiffness + \dot g*contactDamping$; during Newton iterations, the contact force is actived only, if $dataCoordinate[0] <= 0$; dataCoordinate is set equal to gap in nonlinear iterations, but not modified in Newton iterations.""",
    classType=ClassTypeObject,
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeConnector,
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"connector's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'markers define contact gap'),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'node number of a NodeGenericData for 1 dataCoordinate (used for active set strategy ==> holds the gap of the last discontinuous iteration)'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='contactStiffness',
            defaultValue=0.,
            description=r'contact (penalty) stiffness [SI:N/m]; acts only upon penetration'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='contactDamping',
            defaultValue=0.,
            description=r'contact damping [SI:N/(m s)]; acts only upon penetration'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='offset',
            defaultValue=0.,
            description=r'offset [SI:m] of contact'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetDataVariablesSize',
            implementation='return 1;',
            description='needed in order to create ltg-lists for data variable of connector'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=TReal, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeGap',
            args='const MarkerDataStructure& markerData',
            description=r'compute gap for given MarkerData --> done for different configurations (current, start of step, ...)'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemFunctionDef('HasDiscontinuousIteration',
            implementation='return true;'),
        ItemFunctionDef('PostNewtonStep'),
        ItemFunctionDef('PostDiscontinuousIterationStep'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('GetOutputVariableTypes'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Coordinate']),
        ItemRequestedTypes('Node', ['GenericData']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ContactCoordinate";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = diameter of spring; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectContactCircleCable2D   ++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectContactCircleCable2D',
    addIncludesC=r"""constexpr Index CObjectContactCircleCable2DmaxNumberOfSegments = 12; //maximum number of contact segments
""",
    cParentClass=ParentClassCObjectConnector,
    classDescription=r"""A very specialized penalty-based contact condition between a 2D circle (=marker0, any Position-marker) on a body and an ANCFCable2DShape (=marker1, Marker: BodyCable2DShape), in xy-plane. A node NodeGenericData is required with the number of cordinates according to the number of contact segments; the contact gap $g$ is integrated (piecewise linear) along the cable and circle; the contact force $f_c$ is zero for $gap>0$ and otherwise computed from $f_c = g*contactStiffness + \dot g*contactDamping$; during Newton iterations, the contact force is actived only, if $dataCoordinate[0] <= 0$; dataCoordinate is set equal to gap in nonlinear iterations, but not modified in Newton iterations.""",
    classType=ClassTypeObject,
    equations=r"""    #### Connector equations

    Geometry and equations are very similar to \texttt{ObjectContactFrictionCircleCable2D}, while friction is not used and no torque
    is transferred to the circle object.
<!-- -->
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeConnector,
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"connector's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'markers define contact gap'),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'node number of a NodeGenericData for nSegments dataCoordinates (used for active set strategy ==> hold the gap of the last discontinuous iteration and the friction state)'),
        ItemParameter(type=TIndex, destination=DestComp+DestParam,
            pythonName='numberOfContactSegments',
            defaultValue=3,
            description=r'number of linear contact segments to determine contact; each segment is a line and is associated to a data (history) variable; must be same as in according marker'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='contactStiffness',
            defaultValue=0.,
            description=r'contact (penalty) stiffness [SI:N/m/(contact segment)]; the stiffness is per contact segment; specific contact forces (per length) $f_N$ act in contact normal direction only upon penetration'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='contactDamping',
            defaultValue=0.,
            description=r'contact damping [SI:N/(m s)/(contact segment)]; the damping is per contact segment; acts in contact normal direction only upon penetration'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='circleRadius',
            defaultValue=0.,
            description=r'radius [SI:m] of contact circle'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='offset',
            defaultValue=0.,
            description=r'offset [SI:m] of contact, e.g. to include thickness of cable element'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetDataVariablesSize',
            implementation='return parameters.numberOfContactSegments;',
            description='Needs a data variable for every contact segment (tells if this segment is in contact or not)'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeGap',
            args='const MarkerDataStructure& markerData, ConstSizeVector<CObjectContactCircleCable2DmaxNumberOfSegments>& gapPerSegment, ConstSizeVector<CObjectContactCircleCable2DmaxNumberOfSegments>& referenceCoordinatePerSegment, ConstSizeVector<CObjectContactCircleCable2DmaxNumberOfSegments>& xDirectionGap, ConstSizeVector<CObjectContactCircleCable2DmaxNumberOfSegments>& yDirectionGap',
            description=r'compute gap for given MarkerData; done for every contact point (numberOfSegments+1) --> in order to decide contact state for every segment; in case of positive gap, the area is distance*segment_length'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemFunctionDef('HasDiscontinuousIteration',
            implementation='return true;'),
        ItemFunctionDef('PostNewtonStep'),
        ItemFunctionDef('PostDiscontinuousIterationStep'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('GetOutputVariableTypes'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', [],
            description=r"""provide requested markerType for connector; for different markerTypes in marker0/1 => set to ::\_None"""),
        ItemRequestedTypes('Node', ['GenericData']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ContactCircleCable2D";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=TBool, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='IsContactActive',
            description=r'return if contact is active-->avoids computation of ODE2LHS, speeds up computation'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=TBool, destination=DestVisu,
            pythonName='showContactCircle',
            defaultValue=True,
            description=r'if True and show=True, the underlying contact circle is shown; uses circleTiling*4 for tiling (from VisualizationSettings.general)'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = diameter of spring; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectContactFrictionCircleCable2D   ++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectContactFrictionCircleCable2D',
    addIncludesC=r"""constexpr Index CObjectContactFrictionCircleCable2DmaxNumberOfSegments = 12; //maximum number of contact segments
""",
    addPublicC=r"""    static const Index isStickCase = 0; //AUTO: value which represents stick
    static const Index isUndefinedCase = -2; //AUTO: value which represents undefined stick/slip
    static const Index absValueSlipCase = 1; //AUTO: slip may be +-1 !
""",
    cParentClass=ParentClassCObjectConnector,
    classDescription=r"""A very specialized penalty-based contact/friction condition between a 2D circle in the local x/y plane (=marker0, a RigidBody Marker, from node or object) on a body and an ANCFCable2DShape (=marker1, Marker: BodyCable2DShape), in xy-plane. A node NodeGenericData is required with 3$\times$(number of contact segments) -- containing per segment: [contact gap, stick/slip (stick=0, slip=+-1, undefined=-2), last friction position]. The connector works with Cable2D and ALECable2D, HOWEVER, due to conceptual differences the (tangential) frictionStiffness cannot be used with ALECable2D; if using, it gives wrong tangential stresses, even though it may work in general.""",
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    <!--\rowTable{marker m1 velocity}{$\LU{0}{\vv}_{m1}$}{} -->

    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | represents current global position of the circle's centerpoint |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | marker m1 |  | represents the 2D ANCF cable |
    | data node | $\xv=[x_{i},\; \ldots,\; x_{3 n_{cs} -1}]\tp$ | coordinates of node with node number $n_{GD}$ |
    | data coordinates for segment $i$ | $[x_i,\, x_{n_{cs}+ i},\, x_{2\cdot n_{cs}+ i}]\tp = [x_{gap},\, x_{isSlipStick},\, x_{lastStick}]\tp$, with $i \in [0,n_{cs}-1]$ | The data coordinates include the gap $x_{gap}$, the stick-slip state $x_{isSlipStick}$ and the previous sticking position $x_{lastStick}$ as computed in the PostNewtonStep, see description below. |
    | shortest distance to segment $s_i$ | $\dv_{g,i}$ | shortest distance of center of circle to contact segment, considering the endpoint of the segment |

    <!--++++++++++++++++++++++++ -->
    

    (fig-objectcontactfrictioncirclecable2d-sketch)=
    ```{figure} /docs/figures/ContactFrictionCircleCable2D.*
    :width: 600

    Sketch of cable, contact segments and circle; showing case without contact, $|\mathbf{d}_{g1}| > r$, while contact occurs with $|\mathbf{d}_{g1}| \le r$; the shortest distance vector $\mathbf{d}_{g1}$ is related to segment $s_1$ (which is perpendicular to the the segment line) and $\mathbf{d}_{g2}$ is the shortest distance to the end point of segment $s_2$, not being perpendicular
    ```

    <!--+++++++++++++++++++++++++++++++++++++++++++++++ -->

    #### Connector forces: contact geometry

    <!-- -->
    The connector represents a force element between a 'circle' (or cylinder) represented by a marker $m0$, which has position and orientation,
    and an \texttt{ANCFCable2D} beam element (denoted as 'cable') represented by a \texttt{MarkerBodyCable2DShape} $m1$.
    The cable with reference length $L$ is discretized by splitting into $n_{cs}$ straight segments $s_i$, located between points $p_i$ and $p_{i+1}$.
    Note that these points can be placed with an offset from the cable centerline, see \texttt{verticalOffset} defined in \texttt{MarkerBodyCable2DShape}.
    In order to compute the gap function for a line segment, the shortest distance of one line segment with
    points $\pv_i$, $\pv_{i+1}$ to the circle's centerpoint given by the marker $\pv_{m0}$ is computed. 
    All computations here are performed in the global coordinates system (0), 
    including edge points of every segment.

    With the intermediate quantities (all of them related to segment $s_i$)\footnote{we omit $s_i$ in some terms for brevity!},


    $$
    \vv_s = \pv_{i+1} - \pv_i, \quad
          \vv_p = \pv_{m0} - \pv_i, \quad
          n = \vv_s\tp \vv_p, \quad
          d = \vv_s\tp \vv_s
    $$

    and assuming that $d \neq 0$ (otherwise the two segment points would be identical and
    the shortest distance would be $d_g = |\vv_p|$),
    we find the relative position $\rho$ of the shortest (projected) point on the 
    segment, which runs from 0 to 1 if lying on the segment, as


    $$
    \rho = \frac{n}{d}
    $$

    We distinguish 3 cases (see also [](#fig-objectcontactfrictioncirclecable2d-sketch) for cases 1 and 2):
        \bn
        \item If $\rho \le 0$, the shortest distance would be the distance to point $\pv_p=\pv_i$,
        reading 


        $$
        d_g = |\pv_{m0} - \pv_i| \quad (\rho \le 0)
        $$

        \item If $\rho \ge 1$, the shortest distance would be the distance to point $\pv_p=\pv_{i+1}$,
        reading 


        $$
        d_g = |\pv_{m0} - \pv_{i+1}| \quad (\rho \ge 1)
        $$

        \item Finally, if $0 < \rho < 1$, then the shortest distance has a projected point somewhere
        on the segment with the point (projected on the segment)


        $$
        \pv_p = \pv_i + \rho \cdot \vv_s
        $$

        and the distance


        $$
        d_g = |\dv_g| = \sqrt{\vv_p\tp \vv_p - (n^2)/d}
        $$

    \en
    Here, the shortest distance vector for every segment results from the projected point $\pv_p$ 
    of the above mentioned cases, see also [](#fig-objectcontactfrictioncirclecable2d-sketch),
    with the relation


    $$
    \dv_g = \dv_{g,s_i}= \pv_{m0} - \pv_p \, .
    $$

    The contact gap for a specific point for segment $s_i$ is in general defined as


    $$
    g = g_{s_i} = d_g - r \, .
    $$ (objectcontactfrictioncirclecable2d-gap)

    using $d_g = |\dv_g|$.
    
    <!--++++++++++++++++++++++++++++++++++++++++++++++ -->

    #### Contact frame and relative motion

    <!--FRAME -->
    Irrespective of the choice of \texttt{useSegmentNormals}, the contact normal vector $\nv_{s_i}$ and tangential vector $\tv_{s_i}$ are defined per segment as


    $$
    \nv_{s_i} = \nv = [n_0, n_1]\tp = \frac{1}{|\dv_{g,s_i}|} \dv_{g,s_i}, \quad \tv_{s_i} = \tv = [-n_1, n_0]\tp
    $$

    The vectors $\tv_{s_i}$ and $\nv_{s_i}$ define the local (contact) frame for further computations.
    
    The velocity at the closest point of the segment $s_i$ is interpolated using $\rho$ and computed as


    $$
    \dot \pv_p = (1-\rho) \cdot \vv_i + \rho \cdot \vv_{i+1}
    $$

    Alternatively, $\dot \pv_p$ could be computed from the cable element by evaluating the velocity at the contact points, but we feel that
    this choice is more consistent with the computations at position level.
    
    The gap velocity $v_n$ ($\neq \dot g$) thus reads


    $$
    v_n = \left( \dot \pv_p - \dot \pv_{m0} \right) \nv
    $$

    In a similar, the tangential velocity reads


    $$
    v_t = \left( \dot \pv_p - \dot \pv_{m0} \right) \tv
    $$ (objectcontactfrictioncirclecable2d-vtangent)

    In case of \texttt{frictionStiffness != 0}, we continuously track the sticking position at which the cable element (or segment) and the circle 
    previously sticked together, similar as proposed by Lugr{\'i}s et al.~[CITE:LugrisEscalonaDC2011]. 
    The difference here to the latter reference, is that we explicitly exclude switching from Newton's method and that Lugr{\'i}s et al.~used
    contact points, while we use linear segments.
    For a simple 1D example using this position based approach for friction, see \texttt{Examples/lugreFrictionText.py}, 
    which compares the traditional LuGre friction model [CITE:CanudasDeWitEtAl1993] with the position based model with tangential stiffness. 
    <!--++++++++++++++++++++++++ -->
    

    (fig-objectcontactfrictioncirclecable2d-stickingpos)=
    ```{figure} /docs/figures/ContactFrictionCircleCable2DstickingPos.*
    :width: 600

    Calculation of last sticking position; blue parts mark the sticking position calculated as $x^*_{curStick}$.
    ```

    <!--++++++++++++++++++++++++ -->
    
    Because there is the chance to wind/unwind relative to the (last) sticking position without slipping,
    the following strategy is used.
    In case of sliding (which could be the last time sliding before sticking), 
    we compute the {\bf current sticking position}, see [](#fig-objectcontactfrictioncirclecable2d-stickingpos), as the sum of the relative position at the segment $s$


    $$
    x_{s,curStick} = \rho \cdot L_{seg}
    $$

    in which $\rho \in [0,1]$ denotes the relative position of contact at the segment with reference length $L_{seg}=\frac{L}{n_{cs}}$.
    The relative position at the circle $c$ is


    $$
    x_{c,curStick} = \alpha \cdot r
    $$

    We immediately see, that under pure rolling\footnote{neglecting the effects of small penetration, usually much smaller than shown for visibility in [](#fig-objectcontactfrictioncirclecable2d-stickingpos).},


    $$
    x_{s,curStick} + x_{c,curStick}  = \mathrm{const}.
    $$

    Note that the \texttt{verticalOffset} from the cable center line, as defined in the related \texttt{MarkerBodyCable2DShape},
    influences the behavior significantly, which is why we recommend to use \texttt{verticalOffset=0} whenever this is an 
    appropriate assumption.
    <!--
    include in paper:
    Even thought that we are convinced that this has some effect, especially for beams with larger height, a reduction of segment length reduces
    this effects as less (un-)winding occurs, a more consistent computation of this effect would require an
    integration of relative motion as stretch influences the local changes of the relative sticking position.
    -->
    Thus, the current sticking position $x_{curStick}$ is computed per segment as


    $$
    x^*_{curStick} = x_{s,curStick} + x_{c,curStick}, \quad
    $$ (objectcontactfrictioncirclecable2d-lastcurstick)

    <!-- -->
    Due to the possibility of switching of $\alpha+\phi$ between $-\pi$ and $\pi$, the result is normalized to


    $$
    x_{curStick} = x^*_{curStick} - \mathrm{floor}\left(\frac{x^*_{curStick} }{2 \pi \cdot r} + \frac{1}{2}\right) \cdot 2 \pi \cdot r, \quad
    $$ (objectcontactfrictioncirclecable2d-curstick)

    which gives $\bar x_{curStick} \in [-\pi \cdot r,\pi \cdot r]$, which is stored in the 3rd data variable (per segment).
    The function floor() is a standardized version of rounding, available in C and Python programming languages.
    In the \texttt{PostNewtonStep}, the last sticking position is computed, $x_{lastStick} = x_{curStick}$, and it is also available in the \texttt{startOfStep} state.

    <!--++++++++++++++++++++++++++++++++++++++++++++++ -->

    #### Contact forces: definition

    <!--FORCES -->
    The contact force $f_n$ is zero for $g > 0$ and otherwise computed from 


    $$
    f_n = k_c \cdot g + d_c \cdot v_n
    $$ (objectcontactfrictioncirclecable2d-contactforce)

    NOTE that currently, there is only a linear spring-damper model available, assuming that the impact dynamics 
    is not dominating (such as in belt drives or reeving systems).

    Friction forces are primarily based on relative (tangential) velocity at each segment.
    The 'linear' friction force, based on the velocity penalty parameter $\mu_v$ reads


    $$
    f_t^{(lin)} = \mu_v \cdot v_t \, ,
    $$
    
    <!--++++++++++++++++++++++++++++++++++++++++++++++ -->

    #### Post Newton Step

    In general, see the solver flow chart for the \texttt{DiscontinuousIteration}, see [](#fig-solver-discontinuous-iteration), should be considered when reading this description. Every step is started with values \texttt{startOfStep}, while current values are iterated and updated in the Newton or \texttt{DiscontinuousIteration}.
    
    The \texttt{PostNewtonStep} computes 3 values per segment, which are used for computation of contact forces, irrespectively of the 
    current geometryof the contact. 
    The \texttt{PostNewtonStep} is called after every full Newton method and evaluates the current state w.r.t. the assumed data variables.
    If the assumptions do not fit, new data variables are computed.
    This is necessary in order to avoid discontinuities in the equations, while otherwise the Newton iterations would not 
    (or only slowly) converge.

    The data variables per segment are


    $$
    [x_{gap},\, x_{isSlipStick},\, x_{lastStick}]
    $$

    Here, $x_{gap}$ contains the gap of the segment ($\le 0$ means contact), $x_{lastStick}$ is described in 
    {eq}`objectcontactfrictioncirclecable2d-curstick`, and 
    $x_{isSlipStick}$ defines the stick or slip case,
    \bi
      \item $x_{isSlipStick} = -2$: undefined, used for initialization
      \item $x_{isSlipStick} = 0$: sticking
      \item $x_{isSlipStick} = \pm 1$: slipping, sign defines slipping direction
    \ei
    
    The basic algorithm in the \texttt{PostNewtonStep}, with all operations given for any segment $s_i$, can be summarized as follows:
    \bi
      \item[I.] Evaluate gap per segment $g$ using {eq}`objectcontactfrictioncirclecable2d-gap` and store in data variable: 
            $x_{gap} = g$
      \item[II.] If $x_{gap} < 0$ and ($\mu_v \neq 0$ or  $\mu_k \neq 0$):
      \bn
        \item Compute contact force $f_n$ according to {eq}`objectcontactfrictioncirclecable2d-contactforce`
        \item Compute current sticking position $x_{curStick}$ according to {eq}`objectcontactfrictioncirclecable2d-lastcurstick`\footnote{terms are only evaluated if $\mu_k \neq 0$}
        \item Retrieve \texttt{startOfStep} sticking position\footnote{Importantly, the \texttt{PostNewtonStep} always refers to the \texttt{startOfStep} state in the sticking position, because in the discontinuous iterations, the algorithm could switch to slipping in between and override the last sticking position in the current step} in $x^{startOfStep}_{lastStick}$ and compute and normalize
        difference in sticking position\footnote{in case that $x_{isSlipStick} = -2$, meaning that there is no stored sticking position, we set $\Delta x_{stick} = 0$}:


        $$
        \Delta x^*_{stick} = x_{curStick} - x^{startOfStep}_{lastStick}, \quad
                  \Delta x_{stick} = \Delta x^*_{stick} - \mathrm{floor}\left(\frac{\Delta x^*_{stick} }{2 \pi \cdot r} + \frac{1}{2}\right) \cdot 2 \pi \cdot r
        $$

        \item Compute linear tangential force for friction stiffness and velocity penalty: 


          $$
          f_{t,lin} = \mu_v \cdot v_t + \mu_k \Delta x_{stick}
          $$

        \item Compute tangential force according to Coulomb friction model \footnote{note that the sign of $\Delta x_{stick}$ is used here, but
        alternatively we may also use the sign of $f_{t,lin}$}:


        $$
        f_t = 
                        \begin{cases} f_t^{(lin)}, \quad \quad \quad \quad \quad \quad \quad \mathrm{if} \quad 
                          |f_t^{(lin)}| \le \mu \cdot |f_n| \\ 
                          \mu \cdot |f_n| \cdot \mathrm{Sign}(\Delta x_{stick}), \quad \mathrm{else}
                        \end{cases}
        $$

        \item In the case of slipping, given by $|f_t^{(lin)}| > \mu \cdot |f_n|$, we update the last sticking position in the data variable, 
        such that the spring is pre-tensioned already,


        $$
        x_{lastStick} = x_{curStick} - \mathrm{Sign}(\Delta x_{stick}) \frac{\mu \cdot |f_n|}{\mu_k}, \quad 
                  x_{isSlipStick} = \mathrm{Sign}(\Delta x_{stick})
        $$

        \item In the case of sticking, given by $|f_t^{(lin)}| \le \mu \cdot |f_n|$: Set $x_{isSlipStick} = 0$ and, 
        if $x^{startOfStep}_{isSlipStick} = -2$ (undefined), we update $x_{lastStick} = x_{curStick}$, while otherwise, $x_{lastStick}$ is unchanged.
      \en
      \item[III. ] If $x_{gap} > 0$ or ($\mu_v == 0$ and $\mu_k == 0$), we set $x_{isSlipStick} = -2$ (undefined); this means that in the next step (if this step is accepted), there is no stored sticking position.
      \item[IV.] Compute an error $\varepsilon_{PNS} = \varepsilon^n_{PNS}+\varepsilon^t_{PNS}$,
                  with physical units forces (per segment point), for \texttt{PostNewtonStep}:
      \bn
        \item if gap $x_{gap,lastPNS}$ of previous \texttt{PostNewtonStep} had different sign to current gap, set


        $$
        \varepsilon^n_{PNS} = k_c \cdot \Vert x_{gap} - x_{gap,lastPNS}\Vert
        $$

    while otherwise $\varepsilon^n_{PNS}=0$.
        \item if stick-slip-state $x_{isSlipStick,lastPNS}$ of previous \texttt{PostNewtonStep} is different from current $x_{isSlipStick}$, set


        $$
        \varepsilon^t_{PNS} = \Vert \left(\Vert f_t^{(lin)} \Vert  - \mu \cdot |f_n| \right)\Vert
        $$

    while otherwise $\varepsilon^t_{PNS}=0$.
      \en
    \ei
    Note that the \texttt{PostNewtonStep} is iterated and the data variables are updated continuously until convergence, or until a max.\ number of iterations is reached. If \texttt{ignoreMaxIterations} == 0, computation will continue even if no convergence is reached after the given number of iterations. This will lead so larger errors in such steps, but may have less influence on the overall solution if such cases are rare. 

    <!--++++++++++++++++++++++++++++++++++++++++++++++ -->

    #### Computation of connector forces in Newton

    The computation of LHS terms, the action of forces produced by the contact-friction element, is done during Newton iterations and may not have
    discontinuous behavior, thus relating computations to data variables computed in the \texttt{PostNewtonStep}.
    For efficiency, the LHS computation is only performed, if the \texttt{PostNewtonStep} determined contact in any segment.

    The operations are similar to the \texttt{PostNewtonStep}, but without switching. The following operations are performed for each segment $s_i$, if 
    $x_{gap, s_i} <= 0$:
    \bi
      \item[I.] Compute contact force $f_n$, {eq}`objectcontactfrictioncirclecable2d-contactforce`.
      \item[II.] In case of sticking ($|x_{isSlipStick}|\neq 1$):
      \bi
        \item[II.1] the current sticking position $x_{curStick}$ is computed from {eq}`objectcontactfrictioncirclecable2d-lastcurstick`, and the difference of current and last sticking position reads\footnote{see the difference to the \texttt{PostNewtonStep}: we use $x_{lastStick}$ here, not the \texttt{startOfStep} variant.}:


        $$
        \Delta x^*_{stick} = x_{curStick} - x_{lastStick}, \quad
                  \Delta x_{stick} = x^*_{stick} - \mathrm{floor}\left(\frac{\Delta x^*_{stick} }{2 \pi \cdot r} + \frac{1}{2}\right) \cdot 2 \pi \cdot r
        $$

        \item[II.2] if the friction stiffness is $\mu_k==0$ or if $x_{isSlipStick} == -2$, we set $\Delta x_{stick}=0$
        \item[II.3] using the tangential velocity from {eq}`objectcontactfrictioncirclecable2d-vtangent`, the tangent force follows as (even if it is larger than the sticking limit)


        $$
        f_t = \mu_v \cdot v_t + \mu_k \Delta x_{stick}
        $$

    \ei
      \item[III.] In case of slipping ($|x_{isSlipStick}|=1$), the tangential firction force is set  as\footnote{see again difference to \texttt{PostNewtonStep}!},


      $$
      f_t = \mu \cdot |f_n| \cdot x_{isSlipStick}, \quad \mathrm{else}
      $$
 
    \ei
    Note that in the Newton method, the tangential force may be inconsistent with the Kuhn-Tucker conditions. However,
    the \texttt{PostNewtonStep} resolves this inconsistency.
    <!--++++++++++++++++++++++++++++++++++++++++++++++ -->

    #### Computation of LHS terms for circle and ANCF cable element

    If \texttt{activeConnector = True}, 
    contact forces $\fv_i$ with $i \in [0,n_{cs}]$ -- these are $(n_{cs}+1)$ forces -- are applied at the points $p_i$, and they are computed for every contact segments (i.e., two segments may contribute to contact forces of one point).
    For every contact computation, first all contact forces at segment points are set to zero. 
    We distinguish two cases SN and PWN. If \texttt{useSegmentNormals==True}, we use the SN case, while otherwise the PWN case is used, 
    compare [](#fig-objectcontactfrictioncirclecable2d-normals).
    <!--++++++++++++++++++++++++ -->
    

    (fig-objectcontactfrictioncirclecable2d-normals)=
    ```{figure} /docs/figures/ContactFrictionCircleCable2Dnormals.*
    :width: 700

    Choice of normals and tangent vectors for calculation of normal contact forces and tangential (friction) forces; note that the \texttt{useSegmentNormals=False} is not appropriate for this setup and would produce highly erroneous forces.
    ```

    <!--++++++++++++++++++++++++ -->
    
    Segment normals (=SN) lead to always good approximations for normal directions, irrespectively of short or extremely long segments as compared to the circle. However, in case of segments that are short as compared to the circle radius, normals computed from the center of the circle to the segment points (=PWN) are more consistent and produce tangents only in circumferential direction, which may improve behavior in some applications. The equations for the two cases read:
    \bi
    \item[] \mybold{CASE SN}: use \mybold{S}egment \mybold{N}ormals\\
    If there is contact in a segment $s_i$, i.e., gap state $x_{gap} \le 0$, see [](#fig-objectcontactfrictioncirclecable2d-sketch)(right), contact forces $\fv_{s_i}$ are computed per segment,


    $$
    \fv_{s_i} = f_n \cdot \nv_{s_i} + f_t \tv_{s_i}
    $$

    and added to every force at segment points according to


      $$
      \begin{aligned}
      \fv_i &\pluseq& (1-\rho) \cdot \fv_{s_i}      \\ \fv_{i+1} &\pluseq& \rho \cdot \fv_{s_i}
      \end{aligned}
      $$

    while in case $x_{gap}  > 0$ nothing is added.
    <!-- -->
    \item[] \mybold{CASE PWN}: use \mybold{P}oint \mybold{W}ise \mybold{N}ormals (at segment points)\\
    If there is contact in a segment $s_i$, i.e., gap $x_{gap} \le 0$, 
    see [](#fig-objectcontactfrictioncirclecable2d-sketch)(right), 
    intermediate contact forces $\fv^{l,r}_{i}$ are computed per segment point,


      $$
      \fv^l = f_n \cdot \nv_{l,s_i} + f_t \tv_{l,s_i}, \quad
              \fv^r = f_n \cdot \nv_{r,s_i} + f_t \tv_{r,s_i}
      $$

      in which $\nv_{l,s_i}$ is the vector from circle center to the left point ($i$) of the segment $s_i$,
      and $\nv_{l,s_i}$ to the right point ($i+1$). The tangent vectors are perpendicular to the normals.
    <!-- -->
      The forces are then applied to the contact forces $\fv_i$ using the parameter $\rho$, which takes into account the distance of contact to the left or right side of the segment,


      $$
      \begin{aligned}
      \fv_i &\pluseq& (1-\rho) \cdot \fv^l      \\ \fv_{i+1} &\pluseq& \rho \cdot \fv^r
      \end{aligned}
      $$

    while in case $x_{gap}  > 0$ nothing is added.
    \ei
    The forces $\fv_i$ are then applied through the marker to the \texttt{ObjectANCFCable2D} element as point loads via a position jacobian
    (using the according access function), for details see the C++ implementation.
    
    The forces on the circle marker $m0$ are computed as the total sum of all
    segment contact forces, 


    $$
    \fv_{m0} = -\sum_{s_i} \fv_{s_i}
    $$

    and additional torques on the circle's rotation simply follow from


    $$
    \tau_{m0} = -\sum_{s_i} r \cdot f_{t_{s_i}} \, .
    $$

    <!-- -->
    During Newton iterations, the contact forces for segment $s_i$ are considered only, if 
    $x_i <= 0$. The dataCoordinate $x_i$ is not modified during Newton iterations, but computed
    during the DiscontinuousIteration, see [](#fig-solver-discontinuous-iteration) in the solver description. 
    <!-- -->
    \vspace{12pt}\\
    If \texttt{activeConnector = False}, all contact and friction forces on the cable and the force and torque on the 
    circle's marker are set to zero.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVCoordinates, r"""$[u_{t,0},\, g_0,\, u_{t,1},\, g_1,\, \ldots,\, u_{t,n_{cs}},\, g_{n_{cs}}]\tp$local (relative) displacement in tangential ($\tv$) and normal ($\nv$) direction per segment ($n_{cs}$); values are only provided in case of contact, otherwise zero; tangential displacement is only non-zero in case of sticking!"""),
        ItemOutputVariable(OVCoordinates_t, r"""$[v_{t,0},\, v_{n,0},\, v_{t,1},\, v_{n,1},\, \ldots,\, v_{t,n_{cs}},\, v_{n,n_{cs}}]\tp$local (relative) velocity in tangential ($\tv$) and normal ($\nv$) direction per segment ($n_{cs}$); values are only provided in case of contact, otherwise zero"""),
        ItemOutputVariable(OVForceLocal, r"""$[f_{t,0},\, f_{n,0},\, f_{t,1},\, f_{n,1},\, \ldots,\, f_{t,n_{cs}},\, f_{n,n_{cs}}]\tp$local contact forces in tangential ($\tv$) and normal ($\nv$) direction per segment ($n_{cs}$)"""),
        ],
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"connector's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r"""$[m0,m1]\tp$a marker $m0$ with position and orientation and a marker $m1$ of type BodyCable2DShape; together defining the contact geometry"""),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r"""$n_g$node number of a NodeGenericData with 3 $\times n_{cs}$  dataCoordinates (used for active set strategy $\ra$ hold the gap of the last discontinuous iteration, friction state (+-1=slip, 0=stick, -2=undefined) and the last sticking position; initialize coordinates with list [0.1]*$n_{cs}$+[-2]*$n_{cs}$+[0.]*$n_{cs}$, meaning that there is no initial contact with undefined slip/stick"""),
        ItemParameter(type=TIndex(greaterThan=0), destination=DestComp+DestParam,
            pythonName='numberOfContactSegments',
            defaultValue=3,
            description=r'$n_{cs}$number of linear contact segments to determine contact; each segment is a line and is associated to a data (history) variable; must be same as in according marker'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='contactStiffness',
            defaultValue=0.,
            description=r'$k_c$contact (penalty) stiffness [SI:N/m/(contact segment)]; the stiffness is per contact segment; specific contact forces (per length) $f_n$ act in contact normal direction only upon penetration'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='contactDamping',
            defaultValue=0.,
            description=r'$d_c$contact damping [SI:N/(m s)/(contact segment)]; the damping is per contact segment; acts in contact normal direction only upon penetration'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='frictionVelocityPenalty',
            defaultValue=0.,
            description=r"""$\mu_v$tangential velocity dependent penalty coefficient for friction [SI:N/(m s)/(contact segment)]; the coefficient causes tangential (contact) forces against relative tangential velocities in the contact area"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='frictionStiffness',
            defaultValue=0.,
            description=r"""$\mu_k$tangential displacement dependent penalty/stiffness coefficient for friction [SI:N/m/(contact segment)]; the coefficient causes tangential (contact) forces against relative tangential displacements in the contact area"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='frictionCoefficient',
            defaultValue=0.,
            description=r"""$\mu$friction coefficient [SI: 1]; tangential specific friction forces (per length) $f_t$ must fulfill the condition $f_t \le \mu f_n$"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='circleRadius',
            defaultValue=0.,
            description=r'$r$radius [SI:m] of contact circle'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='useSegmentNormals',
            defaultValue=True,
            description=r' True: use normal and tangent according to linear segment; this is appropriate for very long (compared to circle) segments; False: use normals at segment points according to vector to circle center; this is more consistent for short segments, as forces are only applied in beam tangent and normal direction'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetDataVariablesSize',
            implementation='return 3*parameters.numberOfContactSegments;',
            description=r"""Needs a data variable for every contact segment (tells if this segment is in contact or not), every friction condition (stick = 1, slip = 0), and the last sticking position in tangential direction in terms of an angle $\varphi$ in the local circle coordinates ($\varphi = 0$, if the vector to the contact position is aligned with the x-axis)"""),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeGap',
            args='const MarkerDataStructure& markerData, ConstSizeVector<CObjectContactFrictionCircleCable2DmaxNumberOfSegments>& gapPerSegment, ConstSizeVector<CObjectContactFrictionCircleCable2DmaxNumberOfSegments>& referenceCoordinatePerSegment, ConstSizeVector<CObjectContactFrictionCircleCable2DmaxNumberOfSegments>& xDirectionGap, ConstSizeVector<CObjectContactFrictionCircleCable2DmaxNumberOfSegments>& yDirectionGap',
            description=r'compute gap for given MarkerData; done for every contact point (numberOfSegments+1) --> in order to decide contact state for every segment; in case of positive gap, the area is distance*segment_length'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemFunctionDef('HasDiscontinuousIteration',
            implementation='return true;'),
        ItemFunctionDef('PostNewtonStep'),
        ItemFunctionDef('PostDiscontinuousIterationStep'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', [],
            description=r"""provide requested markerType for connector; for different markerTypes in marker0/1 => set to ::\_None"""),
        ItemRequestedTypes('Node', ['GenericData']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ContactFrictionCircleCable2D";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=TBool, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='IsContactActive',
            description=r'return if contact is active-->avoids computation of ODE2LHS, speeds up computation'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r"""set True, if item is shown in visualization and false if it is not shown; note that only normal contact forces can be  drawn, which are approximated by $k_c \cdot g$ (neglecting damping term)"""),
        ItemParameter(type=TBool, destination=DestVisu,
            pythonName='showContactCircle',
            defaultValue=True,
            description=r'if True and show=True, the underlying contact circle is shown; uses circleTiling*4 for tiling (from VisualizationSettings.general)'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = diameter of spring; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectContactSphereSphere   +++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectContactSphereSphere',
    addProtectedC=r"""    static constexpr Index nDataVariables = 4; //number of data variables for tangential and normal contact
    static constexpr Index dataIndexGap = 0; //!< index in data node representing gap
    static constexpr Index dataIndexVtangent = 1; //!< index in data node representing tangent velocity
    static constexpr Index dataIndexImpactVel = 2; //!< index in data node representing last impact velocity
    static constexpr Index dataIndexDeltaPlastic = 3; //!< index in data node representing plastic deformation, according to elasto-plastic adhesion model
""",
    author=r'Gerstmayr Johannes, Weyrer Sebastian',
    cParentClass=ParentClassCObjectConnector,
    classDescription=r'A simple contact connector between two spheres, using various contact models and the option for contact of sphere inside hollow sphere (marker1). The connector implements at least the same functionality as in GeneralContact and is intended for simple setups and for testing, while GeneralContact is much more efficient due to parallelization approaches and efficient contact search.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities


    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | global position of sphere 0 center as provided by marker m0 |
    | marker m0 orientation | $\LU{0,m0}{\Rot}$ | current rotation matrix provided by marker m0 |
    | marker m1 position | $\LU{0}{\pv}_{m1}$ | global position of sphere 1 center as provided by marker m1 |
    | marker m1 orientation | $\LU{0,m1}{\Rot}$ | current rotation matrix provided by marker m1 |
    | data coordinates | $\xv=[x_0,\,x_1,\,x_2,\,x_3]\tp$ | hold the current gap (0), the (norm of the) tangential velocity (1), the impact velocity (2), and the plastic deformation (3) of the adhesion model |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ | current global velocity which is provided by marker m1 |
    | marker m0 angular velocity | $\LU{0}{\tomega}_{m0}$ | current angular velocity vector provided by marker m0 |
    | marker m1 angular velocity | $\LU{0}{\tomega}_{m1}$ | current angular velocity vector provided by marker m1 |


    #### Connector forces

    This section outlines the computation of the forces acting on the two spheres when they are in contact with each other. Two types of forces can act on the spheres due to the connector:
    \bi
    \item normal force computed according to the chosen impact model $m_\mathrm{impact}$ and with contact damping if $d_c\neq0$; this type of force does not create a torque acting on the spheres.
    \item tangential force due to a regularized friction law to model dry friction between the spheres; this type of force creates a torque acting on the spheres and is computed independently of the chosen impact model if $\mu_d\neq0$ is set. Note that in the implemented model, rolling deformations are not considered, i.e. the friction is only a function of the relative tangential velocity between the spheres at the contact point.
    \ei

    

    (fig-objectspherespherecontact)=
    ```{figure} /docs/figures/SphereSphereContact.png
    :width: 400

    Two spheres that are in contact, showing a force on marker 1 in normal direction due to overlap; forces on marker 0 act in opposite direction.
    ```

    Calculations reflect the case for outer contact of two spheres using $h_1=1$. In case that isHollowSphere1=True, we set $h_1=-1$ while the remaining formulas are unchanged. In Figure [](#fig-objectspherespherecontact) the sphere sphere and in Figure [](#fig-objectspherehollowspherecontact) the sphere hollowsphere contact case are shown.

    For the following, the gap $g$ between the two spheres is computed as


    $$
    g = h_1 || \LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0} || - (r_0 + h_1 r_1)
    $$

    and the overlap $\delta$ is the negated gap: $\delta=-g$. In the contact case, the overlap $\delta$ is positive. If the first sphere is a hollow sphere, the gap consequently reads


    $$
    g = r_1 - r_0 - || \LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0} || \, ,
    $$

    such that if sphere 0 is in contact with the inner side of sphere 1, i.e. $|| \LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0} ||\geq(r_1-r_0)$ holds, the gap is negative and the overlap $\delta$ is positive. The normal vector $\LU{0}{\nv}$ always points from marker 0 to the contact point:


    $$
    \LU{0}{\nv} = h_1 \frac{\LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0}}{|| \LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0} ||} \, .
    $$

    In the case of sphere-sphere contact, $\LU{0}{\nv}$ points from marker 0 to marker 1, and 
    in the case of sphere-hollowsphere contact, $\LU{0}{\nv}$ has the reversed direction as if it points from marker 1 to marker 0.

    The scalar normal (gap) velocity $v_\mathrm{\delta,n}$ is computed with the velocities $\LU{0}{\vv}_{m0}=\LU{0}{\dot{\pv}}_{m0}$ and $\LU{0}{\vv}_{m1}=\LU{0}{\dot{\pv}}_{m1}$


    $$
    v_\mathrm{\delta,n} = \left(\LU{0}{\vv}_{m1} - \LU{0}{\vv}_{m0}\right)\cdot \LU{0}{\nv}
    $$

    and the tangential (gap) velocity $\LU{0}{\vv}_\mathrm{\delta,t}$ at the contact point, that is needed for the friction model, reads


    $$
    \LU{0}{\vv}_\mathrm{\delta,t} = \left(\LU{0}{\vv}_{a1} - \LU{0}{\vv}_{a0}\right) - v_\mathrm{\delta,n} \cdot \LU{0}{\nv}, \qquad v_\mathrm{rel} = || \LU{0}{\vv}_\mathrm{\delta,t} || \, .
    $$ (eq-ossctangentialvelocity)


    

    (fig-objectspherehollowspherecontact)=
    ```{figure} /docs/figures/SphereHollowsphereContact.png
    :width: 400

    One sphere and one hollowsphere that are in contact, showing a force on marker 1 against normal direction due to overlap; forces on marker 0 act in opposite direction.
    ```


    To take the angular velocity of the spheres into account, the velocities $\LU{0}{\vv}_{a0}$ and $\LU{0}{\vv}_{a1}$ at the contact point are computed using Euler's theorem for kinematics:


    $$
    \LU{0}{\vv}_{a0} = \LU{0}{\vv}_{m0} + \LU{0}{\tomega}_{m0} \times \left(\LU{0}{\nv}\cdot \left(r_0-\frac{\delta}{2}\right)\right)
        , \qquad
        \LU{0}{\vv}_{a1} = \LU{0}{\vv}_{m1} + h_1 \LU{0}{\tomega}_{m1} \times \left(-\LU{0}{\nv}\cdot \left(r_1-h_1 \frac{\delta}{2}\right)\right) \, .
    $$

    For the velocity $\LU{0}{\vv}_{a0}$ of sphere 0 at the contact point the sphere-sphere and sphere-hollowsphere contact cases are computed equally. For the velocity $\LU{0}{\vv}_{a1}$ of sphere 1 at the contact point, one negative sign $h_1$ is needed since the vector pointing to the contact point has the same direction for both spheres and one negative sign $h_1$ is needed to correctly compute the length of the vector pointing from the center of sphere 1 to the contact point.

    The normal force acting on marker 1 is generally written as


    $$
    \LU{0}{\fv}_\mathrm{1,n} = \underbrace{(f_c + f_d)}_{f_\mathrm{1,n}} \cdot \LU{0}{\nv} \, ,
    $$

    where $f_c$ is the elastic and $f_d$ the damping part. The damping $f_d$ is always computed the same, independent of the chosen impact model:


    $$
    f_d = - d_c v_\mathrm{\delta,n} \, .
    $$

    The negative sign is because of the damping acting against the gap velocity: in the case of a positive normal (gap) velocity, the damping acts against $\LU{0}{\nv}$ for marker 1. As an illustrative case, the gap velocity is positive, if sphere 0 does not move, i.e. $\LU{0}{\vv}_{m0}=0$ holds, and sphere 1 in direction of the normal vector. Note that this holds for the sphere-sphere contact and sphere-hollowsphere contact cases. The elastic force $f_c$ is computed depending on the chosen impact model.

    CASE $m_\mathrm{impact}=0$: the Adhesive Elasto-Plastic model described in [CITE:Morrissey2014] is used. This model captures the key bulk behavior of cohesive powders and granular soils. For the impact model, the plastic overlap $\delta_p$ is needed. It is computed with


    $$
    \delta_p=\lambda_\mathrm{p}^{\frac{1}{n_\mathrm{exp}}}\delta \, .
    $$

    The Adhesive Elasto-Plastic model distinguishes three different cases, modeling the loading and unloading behavior of the spheres:


    $$
    f_c=
        \begin{cases}
            -f_\mathrm{adh} + k_c \delta^{n_\mathrm{exp}} & \text{if } k_2 \left(\delta^{n_\mathrm{exp}}-\delta_p^{n_\mathrm{exp}} \right) \geq k_c\delta^{n_\mathrm{exp}} \\
            -f_\mathrm{adh} + k_2 \left(\delta^{n_\mathrm{exp}}-\delta_p^{n_\mathrm{exp}} \right) & \text{if } k_c\delta^{n_\mathrm{exp}} > k_2 \left(\delta^{n_\mathrm{exp}}-\delta_p^{n_\mathrm{exp}}\right) > -k_\mathrm{adh}\delta^{n_\mathrm{adh}} \\
            -f_\mathrm{adh}-k_\mathrm{adh}\delta^{n_\mathrm{adh}} & \text{if } -k_\mathrm{adh}\delta^{n_\mathrm{adh}} > k_2 \left(\delta^{n_\mathrm{exp}}-\delta_p^{n_\mathrm{exp}} \right)
        \end{cases}\, .
    $$

    Note that $k_2$ is computed with $k_2 = k_c/(1-\lambda_\mathrm{P})$. The terms with the stiffness $k_c$ and $k_2$ have a positive sign, since they act in the direction of $\LU{0}{\nv}$ for marker 1. The constant adhesion force $f_\mathrm{adh}$ and the stiffness $k_\mathrm{adh}$ act against $\LU{0}{\nv}$, which corresponds to a force sticking the spheres together.

    CASE $m_\mathrm{impact}=1$: the restitution model proposed by Hunt and Crossley in [CITE:Hunt1975] is used to simulate the energy loss of the spheres during contact:


    $$
    f_c=k_c \delta^{n_\mathrm{exp}} + \lambda \delta^{n_\mathrm{exp}} v_\mathrm{\delta,n}
    $$

    with


    $$
    \lambda = \frac{k_c}{\dot\delta_\mathrm{-}}\frac{3}{2}(e_\mathrm{res}-1) \, .
    $$

    The restitution coefficient $e_\mathrm{res}$ describes the ration of the normal (gap) velocity before and after the impact of the spheres. In the case of $e_\mathrm{res}<1$, the impact has a plastic portion, resulting in a force acting against $\LU{0}{\nv}$ for marker 1, which is why $\lambda$ must be negative in that case. $\dot\delta_\mathrm{-}$ is the initial relative velocity, which is either the minimum impact velocity or the normal (negated gap) velocity:


    $$
    \dot\delta_\mathrm{-} = \max{\left(\dot\delta_\mathrm{-,min}; -v_\mathrm{\delta,n} \right)}
    $$

    Note that the Hunt-Crossley restitution is valid for a very small energy loss ($e_\mathrm{res}\approx1$) [CITE:Carvalho2019].

    CASE $m_\mathrm{impact}=2$: a generalization of the Hunt-Crossley restitution proposed by Carvalho and Martins in [CITE:Carvalho2019] is used for $e_\mathrm{res} > \frac{1}{3}$ and a model proposed by Gonthier et al. in [CITE:Gonthier2004] is used for impacts with a high plastic proportion, $e_\mathrm{res} < \frac{1}{3}$. Note that the two models are identical at $e_\mathrm{res} = \frac{1}{3}$. $\lambda$ is therefore computed as follows:


    $$
    \lambda=
        \begin{cases}
            \frac{k_c}{\dot\delta_\mathrm{-}}\frac{3}{2}(e_\mathrm{res}-1)\frac{11-e_\mathrm{res}}{1+9e_\mathrm{res}} & \text{if } e_\mathrm{res} > \frac{1}{3} \\
            \frac{k_c}{\dot\delta_\mathrm{-}}\frac{e_\mathrm{rep}^2-1}{e_\mathrm{rep}} & \text{if } e_\mathrm{res} > 0 \\
        \end{cases}\, .
    $$

    The tangential force acting on marker 1 due to the friction model acts against the tangential velocity $\vv_\mathrm{\delta,t}$, see the computation of $\vv_\mathrm{\delta,t}$ in Equation {eq}`eq-ossctangentialvelocity`. Thus, the tangential force for marker 1 is computed as


    $$
    \LU{0}{\fv}_\mathrm{1,t} = -\LU{0}{\vv}_\mathrm{\delta,t} \cdot
        \begin{cases}
            \frac{\mu_d f_\mathrm{1,n}}{v_{reg}} & \text{if } v_{rel} < v_{reg} \\
            \frac{\mu_d f_\mathrm{1,n}}{v_{rel}} & \text{else}\\
        \end{cases} \, .
    $$

    Note that the case distinction above is made to ensure that for very small relative velocities the friction force does not become implausibly high. Taken together, the force acting on marker 1 due to the connector is computed as


    $$
    \LU{0}{\fv}_{m1}=\LU{0}{\fv}_\mathrm{1,n}+\LU{0}{\fv}_\mathrm{1,t} \, ,
    $$

    the force acting on marker 0 is $\LU{0}{\fv}_{m0}=-\LU{0}{\fv}_{m1}$. The global torque $\LU{0}{\ttau}_{m1}$ acting on marker 1 due to the connector is computed as


    $$
    \LU{0}{\ttau}_{m1}=-h_1 \LU{0}{\nv}\left( r_1-h_1 \frac{1}{2}\delta \right) \times \LU{0}{\fv}_{m1} \, ,
    $$

    and on marker 0 as


    $$
    \LU{0}{\ttau}_{m0}=\LU{0}{\nv} \left(r_0-\frac{1}{2}\delta \right) \times \LU{0}{\fv}_{m0}= \LU{0}{\nv}\left( r_0-\frac{1}{2}\delta \right) \times \left( -\LU{0}{\fv}_{m1} \right) \, .
    $$

    It can be seen that the torque due to the connector is the same for both spheres, if $r_0=r_1$ applies.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVPosition, 'contact center point (also given for positive gap, when no contact occurs)'),
        ItemOutputVariable(OVDisplacement, 'global displacement vector between the two spheres midpoints'),
        ItemOutputVariable(OVDisplacementLocal, '1D Vector, containing only gap'),
        ItemOutputVariable(OVVelocity, 'global relative velocity between the two spheres midpoints'),
        ItemOutputVariable(OVForce, 'global contact force vector'),
        ItemOutputVariable(OVDirector1, 'contains normalized vector from marker 0 to marker 1'),
        ItemOutputVariable(OVTorque, r"""global torque due to friction on marker 0; to obetain torque on marker 1, multiply the torque with the factor $\frac{r_1+g/2}{r_0+g/2}$"""),
        ],
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker, size=2), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r"""$[m0,m1]\tp$list of markers representing centers of spheres, used in connector"""),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n_d$node number of a NodeGenericData with numberOfDataCoordinates = 4 dataCoordinates, needed for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the  gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is the plastic overlap of the Edinburgh Adhesive Elasto-Plastic Model, initialized usually with 0 and set back to 0 in case that spheres have been separated.'),
        ItemParameter(type=TVectorND(2), destination=DestComp+DestParam,
            pythonName='spheresRadii',
            defaultValue='Vector2D({-1.,-1.})',
            description=r"""$[r_0,r_1]\tp$list containing radius of sphere 0 and radius of sphere 1 [SI:m]."""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='isHollowSphere1',
            defaultValue=False,
            description=r'flag, which determines, if sphere attached to marker 1 (radius 1) is a hollow sphere.'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='dynamicFriction',
            defaultValue=0.,
            description=r"""$\mu_d$dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, [](#sec-module-physics)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='frictionProportionalZone',
            defaultValue=0.001,
            description=r"""$v_{reg}$limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), [](#sec-module-physics)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='contactStiffness',
            defaultValue=0.,
            description=r"""$k_c$normal contact stiffness [SI:N/m] (units in case that $n_\mathrm{exp}=1$)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='contactDamping',
            defaultValue=0.,
            description=r'$d_c$linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam,
            pythonName='contactStiffnessExponent',
            defaultValue=1.,
            description=r'$n_\mathrm{exp}$exponent in normal contact model [SI:1]'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='constantPullOffForce',
            defaultValue=0.,
            description=r"""$f_\mathrm{adh}$constant adhesion force [SI:N]; Edinburgh Adhesive Elasto-Plastic Model"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='contactPlasticityRatio',
            defaultValue=0.,
            description=r"""$\lambda_\mathrm{P}$ratio of contact stiffness for first loading and unloading/reloading [SI:1]; Edinburgh Adhesive Elasto-Plastic Model; $\lambda_\mathrm{P}=1-k_c/K2$, which gives the contact stiffness for unloading/reloading $K2 = k_c/(1-\lambda_\mathrm{P})$; set to 0 in order to fully deactivate Edinburgh Adhesive Elasto-Plastic Model model"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='adhesionCoefficient',
            defaultValue=0.,
            description=r"""$k_\mathrm{adh}$coefficient for adhesion [SI:N/m] (units in case that $n_\mathrm{adh}=1$); Edinburgh Adhesive Elasto-Plastic Model; set to 0 to deactivate adhesion model"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='adhesionExponent',
            defaultValue=1.,
            description=r"""$n_\mathrm{adh}$exponent for adhesion coefficient [SI:1]; Edinburgh Adhesive Elasto-Plastic Model"""),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam,
            pythonName='restitutionCoefficient',
            defaultValue=1.,
            description=r"""$e_\mathrm{res}$coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='minimumImpactVelocity',
            defaultValue=0.,
            description=r"""$\dot\delta_\mathrm{-,min}$minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)"""),
        ItemParameter(type=TIndex(minimum=0), destination=DestComp+DestParam,
            pythonName='impactModel',
            defaultValue=0,
            description=r"""$m_\mathrm{impact}$ number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetDataVariablesSize',
            implementation='return nDataVariables;'),
        ItemFunctionDef('HasDiscontinuousIteration',
            implementation='return true;'),
        ItemFunctionDef('PostNewtonStep'),
        ItemFunctionDef('PostDiscontinuousIterationStep'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemFunction(type='template<typename TReal> TReal', destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeContactForces',
            args='TReal gap, const SlimVectorBase<TReal, 3>& n0, TReal deltaVnormal, const SlimVectorBase<TReal, 3>& deltaVji, TReal dryFriction, bool frictionRegularizedRegion, SlimVectorBase<TReal, 3>& fVec, SlimVectorBase<TReal, 3>& fFriction, bool forceFrictionMode = true',
            description=r'unique function to compute contact forces'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeConnectorProperties',
            args='const MarkerDataStructure& markerData, Index itemIndex, const LinkedDataVector& data, Real& frictionCoeff, Real& gap, Vector3D& deltaP, Vector3D& deltaV, Vector3D& fVec, Vector3D& fFriction, Vector3D& n0, bool contactFromData = true',
            description=r'main function to compute contact kinematics and forces'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemRequestedTypes('Marker', ['Position'], conditional=[('Orientation', 'dynamicFriction')]),
        ItemRequestedTypes('Node', ['GenericData']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ContactSphereSphere";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=False,
            description=r'set true, if item is shown in visualization and false if it is not shown; draws spheres by given radii'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue='Float4({0.7f,0.7f,0.7f,1.f})',
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectContactSphereTorus   ++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectContactSphereTorus',
    addProtectedC=r"""    static constexpr Index nDataVariables = 4; //number of data variables for tangential and normal contact
    static constexpr Index dataIndexGap = 0; //!< index in data node representing gap
    static constexpr Index dataIndexVtangent = 1; //!< index in data node representing tangent velocity
    static constexpr Index dataIndexImpactVel = 2; //!< index in data node representing last impact velocity
    static constexpr Index dataIndexDeltaPlastic = 3; //!< index in data node representing plastic deformation, according to elasto-plastic adhesion model
""",
    author=r'Gerstmayr Johannes',
    cParentClass=ParentClassCObjectConnector,
    classDescription=r'A simple contact connector between a sphere (marker0) and a torus (marker1). The sphere is assumed to be placed inside of the torus (outer contact of sphere with torus currently not implemented!).',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities


    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | global position of torus 0 center as provided by marker m0 |
    | marker m0 orientation | $\LU{0,m0}{\Rot}$ | current rotation matrix provided by marker m0 |
    | marker m1 position | $\LU{0}{\pv}_{m1}$ | global position of sphere 1 center as provided by marker m1 |
    | marker m1 orientation | $\LU{0,m1}{\Rot}$ | current rotation matrix provided by marker m1 |
    | data coordinates | $\xv=[x_0,\,x_1,\,x_2,\,x_3]\tp$ | hold the current gap (0), the (norm of the) tangential velocity (1), the impact velocity (2), and (3) which is undefined |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ | current global velocity which is provided by marker m1 |
    | marker m0 angular velocity | $\LU{0}{\tomega}_{m0}$ | current angular velocity vector provided by marker m0 |
    | marker m1 angular velocity | $\LU{0}{\tomega}_{m1}$ | current angular velocity vector provided by marker m1 |


    #### Connector forces

    TBD
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVPosition, 'contact center point (also given for positive gap, when no contact occurs)'),
        ItemOutputVariable(OVDisplacement, 'global displacement vector between the two spheres midpoints'),
        ItemOutputVariable(OVDisplacementLocal, '1D Vector, containing only gap'),
        ItemOutputVariable(OVDirector1, 'normalized vector from marker 0 to marker 1'),
        ItemOutputVariable(OVDirector2, 'the normalized vector from marker 0 to marker 1 projected into the plane of the torus major circle'),
        ItemOutputVariable(OVDirector3, 'normalized vector from the projected point on the major circle (center of the minor circle) to marker 1, being in direction of the contact and normal to the surface'),
        ItemOutputVariable(OVForce, 'global contact force vector'),
        ItemOutputVariable(OVTorque, 'global torque due to friction on marker 0'),
        ],
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker, size=2), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r"""$[m0,m1]\tp$list of markers representing centers of sphere (marker 0) and center of torus (marker 1)"""),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n_d$node number of a NodeGenericData with numberOfDataCoordinates = 4 dataCoordinates, needed for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the  gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused.'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='radiusSphere',
            defaultValue=0.,
            description=r'$r_S$ radius of sphere [SI:m]'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='torusMajorRadius',
            defaultValue=0.,
            description=r'$r_{M}$ major radius of torus [SI:m], representing center of rotated circle'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='torusMinorRadius',
            defaultValue=0.,
            description=r'$r_{m}$ minor radius of torus [SI:m], representing radius of circle of ring'),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='torusAxis',
            defaultValue='Vector3D({0,0,0})',
            description=r"""$\vv_{axis}$Vector containing rotation axis of torus; must be a unit vector."""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='dynamicFriction',
            defaultValue=0.,
            description=r"""$\mu_d$dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, [](#sec-module-physics)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='frictionProportionalZone',
            defaultValue=0.001,
            description=r"""$v_{reg}$limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), [](#sec-module-physics)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='contactStiffness',
            defaultValue=0.,
            description=r"""$k_c$normal contact stiffness [SI:N/m] (units in case that $n_\mathrm{exp}=1$)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='contactDamping',
            defaultValue=0.,
            description=r'$d_c$linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam,
            pythonName='contactStiffnessExponent',
            defaultValue=1.,
            description=r'$n_\mathrm{exp}$exponent in normal contact model [SI:1]'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam,
            pythonName='restitutionCoefficient',
            defaultValue=1.,
            description=r"""$e_\mathrm{res}$coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='minimumImpactVelocity',
            defaultValue=0.,
            description=r"""$\dot\delta_\mathrm{-,min}$minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)"""),
        ItemParameter(type=TIndex(minimum=0), destination=DestComp+DestParam,
            pythonName='impactModel',
            defaultValue=0,
            description=r"""$m_\mathrm{impact}$ number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetDataVariablesSize',
            implementation='return nDataVariables;'),
        ItemFunctionDef('HasDiscontinuousIteration',
            implementation='return true;'),
        ItemFunctionDef('PostNewtonStep'),
        ItemFunctionDef('PostDiscontinuousIterationStep'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemFunction(type='template<typename TReal> TReal', destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeContactForces',
            args='TReal gap, const SlimVectorBase<TReal, 3>& n0, TReal deltaVnormal, const SlimVectorBase<TReal, 3>& deltaVji, TReal dryFriction, bool frictionRegularizedRegion, SlimVectorBase<TReal, 3>& fVec, SlimVectorBase<TReal, 3>& fFriction, bool forceFrictionMode = true',
            description=r'unique function to compute contact forces'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeConnectorProperties',
            args='const MarkerDataStructure& markerData, Index itemIndex, const LinkedDataVector& data, Real& frictionCoeff, Real& gap, Vector3D& deltaP, Vector3D& deltaV, Vector3D& pCircle1, Vector3D& contactPoint, Vector3D& fVec, Vector3D& fFriction, Vector3D& n0, bool contactFromData = true',
            description=r'main function to compute contact kinematics and forces'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemRequestedTypes('Marker', ['Position', 'Orientation']),
        ItemRequestedTypes('Node', ['GenericData']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ContactSphereSphere";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=False,
            description=r'set true, if item is shown in visualization and false if it is not shown; draws spheres by given radii'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue='Float4({0.7f,0.7f,0.7f,1.f})',
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectContactSphereTriangle   +++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectContactSphereTriangle',
    addProtectedC=r"""    static constexpr Index nDataVariables = 4; //number of data variables for tangential and normal contact
    static constexpr Index dataIndexGap = 0; //!< index in data node representing gap
    static constexpr Index dataIndexVtangent = 1; //!< index in data node representing tangent velocity
    static constexpr Index dataIndexImpactVel = 2; //!< index in data node representing last impact velocity
    static constexpr Index dataIndexDeltaPlastic = 3; //!< index in data node representing plastic deformation, according to elasto-plastic adhesion model
""",
    author=r'Gerstmayr Johannes',
    cParentClass=ParentClassCObjectConnector,
    classDescription=r'A simple contact connector between a sphere (marker0) and a triangle (marker1). Penalty-based contact is computed from penetration of the sphere with the triangle, including contact with edges if desired.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities


    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | global position of torus 0 center as provided by marker m0 |
    | marker m0 orientation | $\LU{0,m0}{\Rot}$ | current rotation matrix provided by marker m0 |
    | marker m1 position | $\LU{0}{\pv}_{m1}$ | global position of sphere 1 center as provided by marker m1 |
    | marker m1 orientation | $\LU{0,m1}{\Rot}$ | current rotation matrix provided by marker m1 |
    | data coordinates | $\xv=[x_0,\,x_1,\,x_2,\,x_3]\tp$ | hold the current gap (0), the (norm of the) tangential velocity (1), the impact velocity (2), and (3) which is undefined |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ | current global velocity which is provided by marker m1 |
    | marker m0 angular velocity | $\LU{0}{\tomega}_{m0}$ | current angular velocity vector provided by marker m0 |
    | marker m1 angular velocity | $\LU{0}{\tomega}_{m1}$ | current angular velocity vector provided by marker m1 |


    #### Connector forces

    TBD
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVPosition, 'contact center point (also given for positive gap, when no contact occurs)'),
        ItemOutputVariable(OVDisplacement, 'global displacement vector between the two spheres midpoints'),
        ItemOutputVariable(OVDisplacementLocal, '1D Vector, containing only gap'),
        ItemOutputVariable(OVDirector1, 'normalized vector from sphere midpoint (marker 0) to triangle contact point'),
        ItemOutputVariable(OVForce, 'global contact force vector'),
        ItemOutputVariable(OVTorque, 'global torque due to friction on marker 0'),
        ],
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker, size=2), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r"""$[m0,m1]\tp$list of markers representing the center of the sphere (marker 0) and the reference point of the triangle (marker 1), where triangle nodal positions are defined in the local coordinates of marker 1."""),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n_d$node number of a NodeGenericData with numberOfDataCoordinates = 4 dataCoordinates, needed for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the  gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused.'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='radiusSphere',
            defaultValue=0.,
            description=r'$r_S$ radius of sphere [SI:m]'),
        ItemParameter(type=TVector3DList, destination=DestComp+DestParam,
            pythonName='trianglePoints',
            defaultValue='Vector3DList()',
            description=r"""$[\LU{m_1}{\pv}_0,\LU{m_1}{\pv}_1,\LU{m_1}{\pv}_2]$ triangle points, defined in marker 1 local coordinates"""),
        ItemParameter(type=TIndex(minimum=0), destination=DestComp+DestParam,
            pythonName='includeEdges',
            defaultValue=7,
            description=r'Binary flag, where 1 defines contact with edges 0, 2 with edge 1 and 4 with edge 2; 7 means that contact with all edges is included; edge 0 is the edge between node 0 and node 1'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='dynamicFriction',
            defaultValue=0.,
            description=r"""$\mu_d$dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, [](#sec-module-physics)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='frictionProportionalZone',
            defaultValue=0.001,
            description=r"""$v_{reg}$limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), [](#sec-module-physics)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='contactStiffness',
            defaultValue=0.,
            description=r"""$k_c$normal contact stiffness [SI:N/m] (units in case that $n_\mathrm{exp}=1$)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='contactDamping',
            defaultValue=0.,
            description=r'$d_c$linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam,
            pythonName='contactStiffnessExponent',
            defaultValue=1.,
            description=r'$n_\mathrm{exp}$exponent in normal contact model [SI:1]'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam,
            pythonName='restitutionCoefficient',
            defaultValue=1.,
            description=r"""$e_\mathrm{res}$coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='minimumImpactVelocity',
            defaultValue=0.,
            description=r"""$\dot\delta_\mathrm{-,min}$minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)"""),
        ItemParameter(type=TIndex(minimum=0), destination=DestComp+DestParam,
            pythonName='impactModel',
            defaultValue=0,
            description=r"""$m_\mathrm{impact}$ number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetDataVariablesSize',
            implementation='return nDataVariables;'),
        ItemFunctionDef('HasDiscontinuousIteration',
            implementation='return true;'),
        ItemFunctionDef('PostNewtonStep'),
        ItemFunctionDef('PostDiscontinuousIterationStep'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemFunction(type='template<typename TReal> TReal', destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeContactForces',
            args='TReal gap, const SlimVectorBase<TReal, 3>& n0, TReal deltaVnormal, const SlimVectorBase<TReal, 3>& deltaVji, TReal dryFriction, bool frictionRegularizedRegion, SlimVectorBase<TReal, 3>& fVec, SlimVectorBase<TReal, 3>& fFriction, bool forceFrictionMode = true',
            description=r'unique function to compute contact forces'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeConnectorProperties',
            args='const MarkerDataStructure& markerData, Index itemIndex, const LinkedDataVector& data, Real& frictionCoeff, Real& gap, Vector3D& deltaP, Vector3D& deltaV, Vector3D& fVec, Vector3D& fFriction, Vector3D& n0, bool contactFromData = true',
            description=r'main function to compute contact kinematics and forces'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemRequestedTypes('Marker', ['Position'], conditional=[('Orientation', 'dynamicFriction')]),
        ItemRequestedTypes('Node', ['GenericData']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ContactSphereSphere";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=False,
            description=r'set true, if item is shown in visualization and false if it is not shown; draws spheres by given radii'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue='Float4({0.7f,0.7f,0.7f,1.f})',
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectContactCurveCircles   +++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectContactCurveCircles',
    addIncludesC=r"""#include "Pymodules/PyMatrixContainer.h"//for data matrices
constexpr Index CObjectContactCurveCirclesMaxConstSize = 100; //maximum number of markers upon which arrays do not require memory allocation
""",
    addPublicC=r"""    static constexpr Index nDataVariablesPerSegment = 3; //number of data variables per circle marker
    static constexpr Index dataIndexCircle = 0; //!< index in data node (per segment) representing circle number
    static constexpr Index dataIndexGap = 1; //!< index in data node (per segment) representing gap
    static constexpr Index dataIndexVtangent = 2; //!< index in data node (per segment) representing tangent velocity
""",
    cParentClass=ParentClassCObjectConnector,
    classDescription=r'A contact model between a curve defined by piecewise segments and a set of circles. The 2D curve may corotate in 3D with the underlying marker and also defines the plane of action for the circles. [REQUIRES FURTHER TESTING; friction not yet available]',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities


    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | global position of sphere 0 center as provided by marker m0 |
    | marker m0 orientation | $\LU{0,m0}{\Rot}$ | current rotation matrix provided by marker m0 |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | marker m0 angular velocity | $\LU{0}{\tomega}_{m0}$ | current angular velocity vector provided by marker m0 |
    | data coordinates | $\xv=[x_0,\,x_1,\, \ldots]\tp$ | data coordinates per number of circle markers |

    <!-- -->

    #### Geometric relations

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    tbd
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeConnector,
    outputVariables=[
        ItemOutputVariable(OVDisplacementLocal, 'vector containing the minimum distance to segments per circle midpoint (< 0 in case of contact, and -1 if not computed: if not in according vicinity in search tree)'),
        ItemOutputVariable(OVVelocityLocal, 'vector containing relative (normal) velocity per circle midpoint (or NaN if not computed)'),
        ItemOutputVariable(OVForceLocal, 'pairs of normal and tangential forces per circle or (Nan,Nan) if not computed'),
        ],
    pythonShortName='CamFollowerContactPlanar',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker, size=2), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r"""$[m0,m_{c0},m_{c1},\ldots]\tp$list of $n_c+1$ markers; marker $m0$ represents the marker carrying the curve; all other markers represent centers of $n_c$ circles, used in connector"""),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n_d$node number of a NodeGenericData with nDataVariablesPerSegment dataCoordinates per segment, needed for discontinuous iteration; data variables contain values from last PostNewton iteration: data[0+3*i] is the circle number, data[1+3*i] is the gap, data[2+3*i] is the tangential velocity (and thus contains information if it is stick or slip)'),
        ItemParameter(type=TNumpyVector, destination=DestComp+DestParam,
            pythonName='circlesRadii',
            defaultValue='Vector()',
            description=r"""$[r_{c0},r_{c1}, \ldots]\tp \in \Rcal^{n_c}$Vector containing radii of $n_c$ circles [SI:m]; number according to size of markerNumbers-1"""),
        ItemParameter(type=TPyMatrixContainer, destination=DestComp+DestParam,
            pythonName='segmentsData',
            defaultValue='PyMatrixContainer()',
            description=r"""$\Dm \in \Rcal^{n_s \times 4}$matrix containing a set of two planar point coordinates in each row, representing segments attached to marker $m0$ and undergoing contact with the circles; for segment $s0$ row 0 reads $[p_{0x,s0},\,p_{0y,s0},\,p_{1x,s0},\,p_{1y,s0}]$; note that the segments must be ordered such that going from $\pv_0$ to $\pv_1$, the exterior lies on the right (positive) side. MatrixContainer has to be provided in dense mode!"""),
        ItemParameter(type=TPyMatrixContainer, destination=DestComp+DestParam,
            pythonName='polynomialData',
            defaultValue='PyMatrixContainer()',
            description=r"""$\Pm \in \Rcal^{n_s \times n_p}$matrix containing coefficients for special polynomial enhancements of the linear segments; each row contains coefficients for polynomials for the according segment, prescribing slopes at beginning and end of segment as well as curvature at beginning and end of segment; slopes and curvatures are defined in a local x/y coordinate system where x is the segment axis (start: x=0; x-axis points towards end point) and the segment normal is in y-direction; MatrixContainer has to be provided in dense mode!"""),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp+DestParam,
            pythonName='rotationMarker0',
            defaultValue='EXUmath::unitMatrix3D',
            description=r'local rotation matrix for marker 0; used to rotate marker coordinates such that the curve lies in the $x-y$-plane'),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='dynamicFriction',
            defaultValue=0.,
            description=r"""$\mu_d$dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, [](#sec-module-physics)"""),
        ItemParameter(type=TReal(minimum=0), destination=DestComp+DestParam,
            pythonName='frictionProportionalZone',
            defaultValue=0.001,
            description=r"""$v_{reg}$limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), [](#sec-module-physics)"""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='contactStiffness',
            defaultValue=0.,
            description=r'$k_c$normal contact stiffness [SI:N/(m*m)]'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='contactDamping',
            defaultValue=0.,
            description=r'$d_c$linear normal contact damping [SI:N/(m s)]; this damping is a simplification of real contact dissipation and should be used with care.'),
        ItemParameter(type=TIndex(minimum=0), destination=DestComp+DestParam,
            pythonName='contactModel',
            defaultValue=0,
            description=r"""$m_\mathrm{contact}$number of contact model: 0) linear model for stiffness and damping, only proportional to penetration; contact force is computed from $l_\mathrm{seg}\left(p \cdot  \cdot k_c + \dot p \cdot d_c \right)$ as long as $p>0$; while this is numerically more stable, it gives jumps in forces when sliding over contact geometry 1) contact force proportional to integral over penetration area of circle with segments, giving a smoother contact force when sliding over geometry;"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemParameter(type=TNumpyVector, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='gapPerSegment',
            defaultValue='Vector()',
            description=r'temporary vector for computed gap'),
        ItemParameter(type=TNumpyVector, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='gapPerSegment_t',
            defaultValue='Vector()',
            description=r'temporary vector for computed gap velocity'),
        ItemParameter(type=TNumpyVector, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='segmentsForceLocalX',
            defaultValue='Vector()',
            description=r'temporary vector for contact force per segment in local X-direction'),
        ItemParameter(type=TNumpyVector, destination=DestComp, cFlags=CFMutable+CFReadOnly,
            pythonName='segmentsForceLocalY',
            defaultValue='Vector()',
            description=r'temporary vector for contact force per segment in local Y-direction'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('RequestedNumberOfMarkers',
            implementation='return 0;',
            description='number of markers; 0 means no requirements (standard would be 2; would fail in pre-checks!)'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetDataVariablesSize',
            implementation='return nDataVariablesPerSegment*GetNumberOfSegments();',
            description='data variables in total'),
        ItemFunctionDef('HasDiscontinuousIteration',
            implementation='return true;'),
        ItemFunctionDef('PostNewtonStep'),
        ItemFunctionDef('PostDiscontinuousIterationStep'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return true;'),
        ItemFunctionDef('ComputeODE2LHS'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::ODE2_ODE2 + JacobianType::ODE2_ODE2_t);'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetNumberOfCircles',
            implementation='return parameters.markerNumbers.NumberOfItems()-1;',
            description=r'returns number of circles, needed by other functions'),
        ItemFunction(type=TIndex, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='GetNumberOfSegments',
            implementation='return parameters.segmentsData.NumberOfRows();',
            description=r'number of segments determined by segmentsData'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeConnectorProperties',
            args='const MarkerDataStructure& markerData, Index itemIndex, LinkedDataVector& data, bool useDataStates, Vector2D& forceMarker0, Real& torqueMarker0, Vector& gapPerSegment, Vector& gapPerSegment_t, Vector& segmentsForceLocalX, Vector& segmentsForceLocalY',
            description=r'main function to compute contact kinematics and forces'),
        ItemFunction(type=TReal, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputePolynomials',
            args='Real x, Real c, Index segNum, const ResizableMatrix& polyCoeffs',
            description=r'compute sum of polynomials at x, with segment length c, segment number and polyCoeffs; implemented for 0, 2 or 4 coefficients'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemRequestedTypes('Marker', ['Position', 'Orientation']),
        ItemRequestedTypes('Node', ['GenericData']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return CObjectType::Connector;',
            description=r'return object type (for node treatment in computation)'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "ContactCurveCircles";',
            description=r"Get type name of node (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown; draws curve and circles with given radii; uses visualizationSettings circleTiling for circles and circleTiling/2 for tiling of non-straight segments'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectJointGeneric   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectJointGeneric',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    addProtectedC=r"""    static constexpr Index nConstraints = 6;
""",
    cParentClass=ParentClassCObjectConstraint,
    classDescription=r'A generic joint in 3D; constrains components of the absolute position and rotations of two points given by PointMarkers or RigidMarkers. An additional local rotation (rotationMarker) can be used to adjust the three rotation axes and/or sliding axes. \addExampleImage{UniversalJoint}',
    classType=ClassTypeObject,
    equations=r"""    (sec-objectjointgeneric-definitionofquantities)=
    #### Definition of quantities

    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | current global position which is provided by marker m0 |
    | marker m0 orientation | $\LU{0,m0}{\Rot}$ | current rotation matrix provided by marker m0 |
    | joint J0 orientation | $\LU{0,J0}{\Rot} = \LU{0,m0}{\Rot} \LU{m0,J0}{\Rot}$ | joint $J0$ rotation matrix |
    | joint J0 orientation vectors | $\LU{0,J0}{\Rot} = [\LU{0}{\tv_{x0}},\,\LU{0}{\tv_{y0}},\,\LU{0}{\tv_{z0}}]\tp$ | orientation vectors (represent local $x$, $y$, and $z$ axes) in global coordinates, used for definition of constraint equations |
    | marker m1 position | $\LU{0}{\pv}_{m1}$ | accordingly |
    | marker m1 orientation | $\LU{0,m1}{\Rot}$ | current rotation matrix provided by marker m1 |
    | joint J1 orientation | $\LU{0,J1}{\Rot} = \LU{0,m1}{\Rot} \LU{m1,J1}{\Rot}$ | joint $J1$ rotation matrix |
    | joint J1 orientation vectors | $\LU{0,J1}{\Rot} = [\LU{0}{\tv_{x1}},\,\LU{0}{\tv_{y1}},\,v\tv_{z1}]\tp$ | orientation vectors (represent local $x$, $y$, and $z$ axes) in global coordinates, used for definition of constraint equations |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ | accordingly |
    | marker m0 velocity | $\LU{b}{\tomega}_{m0}$ | current local angular velocity vector provided by marker m0 |
    | marker m1 velocity | $\LU{b}{\tomega}_{m1}$ | current local angular velocity vector provided by marker m1 |
    | Displacement | $\LU{0}{\Delta\pv}=\LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0}$ | used, if all translational axes are constrained |
    | Velocity | $\LU{0}{\Delta\vv} = \LU{0}{\vv}_{m1} - \LU{0}{\vv}_{m0}$ | used, if all translational axes are constrained (velocity level) |
    | DisplacementLocal | $\LU{J0}{\Delta\pv}$ | $\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \LU{0}{\Delta\pv}$ |
    | VelocityLocal | $\LU{J0}{\Delta\vv}$ | $\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \LU{0}{\Delta\vv}$ $\ldots$ note that this is the global relative velocity projected into the local $J0$ coordinate system |
    | AngularVelocityLocal | $\LU{J0}{\Delta\omega}$ | $\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \left( \LU{0,m1}{\Rot} \LU{m1}{\omega} - \LU{0,m0}{\Rot} \LU{m0}{\omega} \right)$ |
    | algebraic variables | $\zv=[\lambda_0,\,\ldots,\,\lambda_5]\tp$ | vector of algebraic variables (Lagrange multipliers) according to the algebraic equations |

    <!-- -->

    #### Connector constraint equations

    \paragraph{Equations for translational part (\texttt{activeConnector = True})}:\\
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    If $[j_0,\,\ldots,\,j_2] = [1,1,1]\tp$, meaning that all translational coordinates are fixed,
    the translational index 3 constraints read ($UF_{0,1,2}(mbs, t, \pv_{par})$ is the translational part of the user function $UF$),

    $$
    \LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0} - UF_{0,1,2}(mbs, t, i_N, \pv_{par}) = \Null
    $$

    and the translational index 2 constraints read

    $$
    \LU{0}{\vv}_{m1} - \LU{0}{\vv}_{m0} - UF_{t;0,1,2}(mbs, t, i_N, \pv_{par})= \Null
    $$

    and \texttt{iN} represents the itemNumber (=objectNumber).
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    If $[j_0,\,\ldots,\,j_2] \neq [1,1,1]\tp$, meaning that at least one translational coordinate is free,
    the translational index 3 constraints read for every component $k \in [0,1,2]$ of the vector $\LU{J0}{\Delta\pv}$

    $$
    \begin{aligned}
    \LU{J0}{\Delta p_k} - UF_{k}(mbs, t, i_N, \pv_{par}) &= 0 \quad \mathrm{if} \quad j_k = 1 \quad \mathrm{and}\\
          \lambda_k &= 0 \quad \mathrm{if} \quad j_k = 0 \\
    \end{aligned}
    $$

    and the translational index 2 constraints read for every component $k \in [0,1,2]$ of the vector $\LU{J0}{\Delta\vv}$

    $$
    \begin{aligned}
    \LU{J0}{\Delta v_k} - UF\_t_{k}(mbs, t, i_N, \pv_{par})  &= 0 \quad \mathrm{if} \quad j_k = 1 \quad \mathrm{and}\\
          \lambda_k &= 0 \quad \mathrm{if} \quad j_k = 0 \\
    \end{aligned}
    $$

    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    \paragraph{Equations for rotational part (\texttt{activeConnector = True})}:\\
    The following equations are exemplarily for certain constrained rotation axes configurations, which shall represent all other possibilities.
    Note that the axes are always given in global coordinates, compare the table in [](#sec-objectjointgeneric-definitionofquantities).
    
    Equations are only given for the index 3 case; the index 2 case can be derived from these equations easily (see C++ code...).
    In case of user functions, the additional rotation matrix $\LU{J0,J0U}{\Rot}(UF_{3,4,5}(mbs, t, \pv_{par}))$, in which the three components of 
    $UF_{3,4,5}$ are interpreted as Tait-Bryan angles that are added to the joint frame.
    
    If {\bf 3 rotation axes are constrained} (e.g., translational or planar joint),  $[j_3,\,\ldots,\,j_5] = [1,1,1]\tp$, the index 3 constraint equations read

    $$
    \begin{aligned}
    \LU{0}{\tv}_{z0}\tp \LU{0}{\tv}_{y1} &= 0 \\
           \LU{0}{\tv}_{z0}\tp \LU{0}{\tv}_{x1} &= 0 \\
           \LU{0}{\tv}_{x0}\tp \LU{0}{\tv}_{y1} &= 0
    \end{aligned}
    $$

    If {\bf 2 rotation axes are constrained} (revolute joint), e.g., $[j_3,\,\ldots,\,j_5] = [0,1,1]\tp$, the index 3 constraint equations read

    $$
    \begin{aligned}
    \lambda_3 &= 0 \\
           \LU{0}{\tv}_{x0}\tp \LU{0}{\tv}_{y1} &= 0 \\
           \LU{0}{\tv}_{x0}\tp \LU{0}{\tv}_{z1} &= 0
    \end{aligned}
    $$

    If {\bf 1 rotation axis is constrained} (universal joint), e.g.,  $[j_3,\,\ldots,\,j_5] = [1,0,0]\tp$, the index 3 constraint equations read

    $$
    \begin{aligned}
    \LU{0}{\tv}_{y0}\tp \LU{0}{\tv}_{z1} &= 0 \\
           \lambda_4 &= 0 \\
           \lambda_5 &= 0
    \end{aligned}
    $$

    <!-- -->
    if \texttt{activeConnector = False}, 

    $$
    \zv = \Null
    $$

    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    **Userfunction**: `offsetUserFunction(mbs, t, itemNumber, offsetUserFunctionParameters)`
    <!-- -->
    A user function, which computes scalar offset for relative joint translation and joint rotation for the GenericJoint, 
    e.g., in order to move or rotate a body on a prescribed trajectory.
    It is NECESSARY to use sufficiently smooth functions, having {\bf initial offsets} consistent with {\bf initial configuration} of bodies, 
    either zero or compatible initial offset-velocity, and no initial accelerations.
    The \texttt{offsetUserFunction} is {\bf ONLY used} in case of static computation or index3 (generalizedAlpha) time integration.
    In order to be on the safe side, provide both  \texttt{offsetUserFunction} and  \texttt{offsetUserFunction\_t}.

    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.

    The user function gets time and the offsetUserFunctionParameters as an input and returns the computed offset vector 
    for all relative translational and rotational joint coordinates:
    <!-- -->

    | arguments / return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs in which underlying item is defined |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{offsetUserFunctionParameters} | Real | $\pv_{par}$, set of parameters which can be freely used in user function |
    | **return value** | Real | computed offset vector for given time |

    <!--
    
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    -->
    **Userfunction**: `offsetUserFunction_t(mbs, t, itemNumber, offsetUserFunctionParameters)`
    <!-- -->
    A user function, which computes an offset {\bf velocity} vector for the GenericJoint.
    It is NECESSARY to use sufficiently smooth functions, having {\bf initial offset velocities} consistent with {\bf initial velocities} of bodies.
    The \texttt{offsetUserFunction\_t} is used instead of \texttt{offsetUserFunction} in case of \texttt{velocityLevel = True}, 
    or for index2 time integration and needed for computation of initial accelerations in second order implicit time integrators.

    Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
    \texttt{mbs.GetObjectParameter(itemNumber, ...)}, see the according description of \texttt{GetObjectParameter}.

    The user function gets time and the offsetUserFunctionParameters as an input and returns the computed offset velocity vector 
    for all relative translational and rotational joint coordinates:
    <!-- -->

    | arguments / return | type or size | description |
    |---|---|---|
    | \texttt{mbs} | MainSystem | provides MainSystem mbs in which underlying item is defined |
    | \texttt{t} | Real | current time in mbs |
    | \texttt{itemNumber} | Index | integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...) |
    | \texttt{offsetUserFunctionParameters} | Real | $\pv_{par}$, set of parameters which can be freely used in user function |
    | **return value** | Real | computed offset velocity vector for given time |

    <!-- -->

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    *Example*:
    
```python
#simple example, computing only the translational offset for x-coordinate
from math import sin, cos, pi
def UFoffset(mbs, t, itemNumber, offsetUserFunctionParameters): 
    return [offsetUserFunctionParameters[0]*(1 - cos(t*10*2*pi)), 0,0,0,0,0]

```

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeJoint,
    outputVariables=[
        ItemOutputVariable(OVPosition, OVDPositionMarker0),
        ItemOutputVariable(OVVelocity, OVDVelocityMarker0),
        ItemOutputVariable(OVDisplacementLocal, r"""$\LU{J0}{\Delta\pv}$relative displacement in local joint0 coordinates; uses local J0 coordinates even for spherical joint configuration"""),
        ItemOutputVariable(OVVelocityLocal, OVDVelocityLocalJoint),
        ItemOutputVariable(OVRotation, r"""$\LU{J0}{\ttheta}= [\theta_0,\theta_1,\theta_2]\tp$relative rotation parameters (Tait Bryan Rxyz); if all axes are fixed, this output represents the rotational drift; for a revolute joint with free Z-axis, it contains the rotation in the Z-component"""),
        ItemOutputVariable(OVAngularVelocityLocal, r"""$\LU{J0}{\Delta\tomega}$relative angular velocity in local joint0 coordinates; if all axes are fixed, this output represents the angular velocity constraint error; for a revolute joint, it contains the angular velocity of this axis"""),
        ItemOutputVariable(OVForceLocal, r'$\LU{J0}{\fv}$joint force in local $J0$ coordinates'),
        ItemOutputVariable(OVTorqueLocal, r"""$\LU{J0}{\mv}$joint torque in local $J0$ coordinates; depending on joint configuration, the result may not be the according torque vector"""),
        ],
    pythonShortName='GenericJoint',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker, size=2), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'$[m0,m1]\tp$list of markers used in connector'),
        ItemParameter(type=TArrayIndex(size=6), destination=DestComp+DestParam,
            pythonName='constrainedAxes',
            defaultValue='ArrayIndex({1,1,1,1,1,1})',
            description=r"""$\jv=[j_0,\,\ldots,\,j_5]$flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; for $j_i$, two values are possible: 0=free axis, 1=constrained axis"""),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp+DestParam,
            pythonName='rotationMarker0',
            defaultValue='EXUmath::unitMatrix3D',
            description=r"""$\LU{m0,J0}{\Rot}$local rotation matrix for marker $m0$; translation and rotation axes for marker $m0$ are defined in the local body coordinate system and additionally transformed by rotationMarker0"""),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp+DestParam,
            pythonName='rotationMarker1',
            defaultValue='EXUmath::unitMatrix3D',
            description=r"""$\LU{m1,J1}{\Rot}$local rotation matrix for marker $m1$; translation and rotation axes for marker $m1$ are defined in the local body coordinate system and additionally transformed by rotationMarker1"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemParameter(type=TVectorND(6), destination=DestComp+DestParam,
            pythonName='offsetUserFunctionParameters',
            defaultValue='Vector6D({0.,0.,0.,0.,0.,0.})',
            description=r"""$\pv_{par}$vector of 6 parameters for joint's offsetUserFunction"""),
        ItemParameter(type=TPyFunctionVector6DmbsScalarIndexVector6D, destination=DestComp+DestParam,
            pythonName='offsetUserFunction',
            defaultValue=0,
            description=r"""$\mathrm{UF} \in \Rcal^6$A Python function which defines the time-dependent (fixed) offset of translation (indices 0,1,2) and rotation (indices 3,4,5) joint coordinates with parameters (mbs, t, offsetUserFunctionParameters)"""),
        ItemParameter(type=TPyFunctionVector6DmbsScalarIndexVector6D, destination=DestComp+DestParam,
            pythonName='offsetUserFunction_t',
            defaultValue=0,
            description=r"""$\mathrm{UF} \in \Rcal^6$(NOT IMPLEMENTED YET)time derivative of offsetUserFunction using the same parameters"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='alternativeConstraints',
            defaultValue=False,
            description=r"""this is an experimental flag, may change in future: if uses alternative contraint equations for rotations, currently in case of 3 locked rotations: $\LU{0}{\tv}_{x0}\tp (\LU{0}{\tv}_{y1} \times \LU{0}{\tv}_{z0})$, $\LU{0}{\tv}_{y0}\tp (\LU{0}{\tv}_{z1} \times \LU{0}{\tv}_{x0})$, $\LU{0}{\tv}_{z0}\tp (\LU{0}{\tv}_{x1} \times \LU{0}{\tv}_{y0})$; this avoids 180\textdegree flips of the standard configuration in static computations, but leads to different values in Lagrange multipliers"""),
        ItemFunctionDef('HasUserFunction',
            implementation='return (parameters.offsetUserFunction!=0) || (parameters.offsetUserFunction_t!=0);'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return false;'),
        ItemFunctionDef('ComputeAlgebraicEquations',
            args='Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false'),
        ItemFunctionDef('ComputeJacobianAE'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Position', 'Orientation']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);',
            description=r'return object type (for node treatment in computation)'),
        ItemFunctionDef('GetAlgebraicEquationsSize',
            implementation='return 6;'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "JointGeneric";',
            description=r"Get type name of object (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionOffset',
            args='Vector6D& offset, const MainSystemBase& mainSystem, Real t, Index itemIndex',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionOffset_t',
            args='Vector6D& offset, const MainSystemBase& mainSystem, Real t, Index itemIndex',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='axesRadius',
            defaultValue=0.1,
            description=r'radius of joint axes to draw'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='axesLength',
            defaultValue=0.4,
            description=r'length of joint axes to draw'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectJointRevoluteZ   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectJointRevoluteZ',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    addProtectedC=r"""    static constexpr Index nConstraints = 5;
""",
    cParentClass=ParentClassCObjectConstraint,
    classDescription=r"""A revolute joint in 3D; constrains the position of two rigid body markers and the rotation about two axes, while the joint $z$-rotation axis (defined in local coordinates of marker 0 / joint J0 coordinates) can freely rotate. An additional local rotation (rotationMarker) can be used to transform the markers' coordinate systems into the joint coordinate system. For easier definition of the joint, use the exudyn.rigidbodyUtilities function AddRevoluteJoint(...), [](#sec-rigidbodyutilities-addrevolutejoint), for two rigid bodies (or ground). \addExampleImage{RevoluteJointZ} \addExampleImage{RevoluteJointZ2}""",
    classType=ClassTypeObject,
    equations=r"""    (sec-objectjointrevolutez-definitionofquantities)=
    #### Definition of quantities


    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | current global position which is provided by marker m0 |
    | marker m0 orientation | $\LU{0,m0}{\Rot}$ | current rotation matrix provided by marker m0 |
    | joint J0 orientation | $\LU{0,J0}{\Rot} = \LU{0,m0}{\Rot} \LU{m0,J0}{\Rot}$ | joint $J0$ rotation matrix |
    | joint J0 orientation vectors | $\LU{0,J0}{\Rot} = [\LU{0}{\tv_{x0}},\,\LU{0}{\tv_{y0}},\,\LU{0}{\tv_{z0}}]\tp$ | orientation vectors (represent local $x$, $y$, and $z$ axes) in global coordinates, used for definition of constraint equations |
    | marker m1 position | $\LU{0}{\pv}_{m1}$ | accordingly |
    | marker m1 orientation | $\LU{0,m1}{\Rot}$ | current rotation matrix provided by marker m1 |
    | joint J1 orientation | $\LU{0,J1}{\Rot} = \LU{0,m1}{\Rot} \LU{m1,J1}{\Rot}$ | joint $J1$ rotation matrix |
    | joint J1 orientation vectors | $\LU{0,J1}{\Rot} = [\LU{0}{\tv_{x1}},\,\LU{0}{\tv_{y1}},\,\LU{0}{\tv_{z1}}]\tp$ | orientation vectors (represent local $x$, $y$, and $z$ axes) in global coordinates, used for definition of constraint equations |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ | accordingly |
    | marker m0 velocity | $\LU{b}{\tomega}_{m0}$ | current local angular velocity vector provided by marker m0 |
    | marker m1 velocity | $\LU{b}{\tomega}_{m1}$ | current local angular velocity vector provided by marker m1 |
    | Displacement | $\LU{0}{\Delta\pv}=\LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0}$ | used, if all translational axes are constrained |
    | Velocity | $\LU{0}{\Delta\vv} = \LU{0}{\vv}_{m1} - \LU{0}{\vv}_{m0}$ | used, if all translational axes are constrained (velocity level) |
    | DisplacementLocal | $\LU{J0}{\Delta\pv}$ | $\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \LU{0}{\Delta\pv}$ |
    | VelocityLocal | $\LU{J0}{\Delta\vv}$ | $\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \LU{0}{\Delta\vv}$ $\ldots$ note that this is the global relative velocity projected into the local $J0$ coordinate system |
    | AngularVelocityLocal | $\LU{J0}{\Delta\omega}$ | $\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \left( \LU{0,m1}{\Rot} \LU{m1}{\omega} - \LU{0,m0}{\Rot} \LU{m0}{\omega} \right)$ |
    | algebraic variables | $\zv=[\lambda_0,\,\ldots,\,\lambda_5]\tp$ | vector of algebraic variables (Lagrange multipliers) according to the algebraic equations |

    <!-- -->

    #### Connector constraint equations

    \paragraph{Equations for translational part (\texttt{activeConnector = True})}:\\
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    The translational index 3 constraints read,


    $$
    \LU{0}{\Delta\pv} = \Null
    $$

    and the translational index 2 constraints read


    $$
    \LU{0}{\Delta \vv} = \Null
    $$

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    \paragraph{Equations for rotational part (\texttt{activeConnector = True})}:\\
    Note that the axes are always given in global coordinates, compare the table in [](#sec-objectjointrevolutez-definitionofquantities),
    and they include the transformations by $\LU{m0,J0}{\Rot}$ and $\LU{m1,J1}{\Rot}$.
    <!-- -->
    The index 3 constraint equations read


    $$
    \begin{aligned}
    \LU{0}{\tv}_{z0}\tp \LU{0}{\tv}_{x1} &= 0 \\
           \LU{0}{\tv}_{z0}\tp \LU{0}{\tv}_{y1} &= 0
    \end{aligned}
    $$ (eq-objectjointrevolutez-index3)

    The index 2 constraints follow from the derivative of {eq}`eq-objectjointrevolutez-index3` w.r.t.\ time, and are given in the C++ code.
    <!-- -->
    if \texttt{activeConnector = False}, 


    $$
    \zv = \Null
    $$

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    miniExample=r"""    #example with rigid body at [0,0,0], with torsional load
    nBody = mbs.AddNode(RigidRxyz())
    oBody = mbs.AddObject(RigidBody(physicsMass=1, physicsInertia=[1,1,1,0,0,0], 
                                    nodeNumber=nBody))
    
    mBody = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nBody))
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                            localPosition = [0,0,0]))
    mbs.AddObject(RevoluteJointZ(markerNumbers = [mGround, mBody])) #rotation around ground Z-axis

    #torque around z-axis; 
    mbs.AddLoad(Torque(markerNumber = mBody, loadVector=[0,0,1])) 

    #assemble and solve system for default parameters
    mbs.Assemble()
    mbs.SolveDynamic(exu.SimulationSettings())
    
    #check result at default integration time
    exu.sys['testResult'] = mbs.GetNodeOutput(nBody, exu.OutputVariableType.Rotation)[2]
""",
    objectType=ObjectTypeJoint,
    outputVariables=[
        ItemOutputVariable(OVPosition, OVDPositionMarker0),
        ItemOutputVariable(OVVelocity, OVDVelocityMarker0),
        ItemOutputVariable(OVDisplacementLocal, r"""$\LU{J0}{\Delta\pv}$relative displacement in local joint0 coordinates; uses local J0 coordinates even for spherical joint configuration"""),
        ItemOutputVariable(OVVelocityLocal, OVDVelocityLocalJoint),
        ItemOutputVariable(OVRotation, r"""$\LU{J0}{\ttheta}= [\theta_0,\theta_1,\theta_2]\tp$relative rotation parameters (Tait Bryan Rxyz); Z component represents rotation of joint, other components represent constraint drift"""),
        ItemOutputVariable(OVAngularVelocityLocal, r"""$\LU{J0}{\Delta\tomega}$relative angular velocity in joint J0 coordinates, giving a vector with Z-component only"""),
        ItemOutputVariable(OVForceLocal, r'$\LU{J0}{\fv}$joint force in local $J0$ coordinates'),
        ItemOutputVariable(OVTorqueLocal, r"""$\LU{J0}{\mv}$joint torques in local $J0$ coordinates; torque around Z is zero"""),
        ],
    pythonShortName='RevoluteJointZ',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker, size=2), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'$[m0,m1]\tp$list of markers used in connector'),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp+DestParam,
            pythonName='rotationMarker0',
            defaultValue='EXUmath::unitMatrix3D',
            description=r"""$\LU{m0,J0}{\Rot}$local rotation matrix for marker $m0$; translation and rotation axes for marker $m0$ are defined in the local body coordinate system and additionally transformed by rotationMarker0"""),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp+DestParam,
            pythonName='rotationMarker1',
            defaultValue='EXUmath::unitMatrix3D',
            description=r"""$\LU{m1,J1}{\Rot}$local rotation matrix for marker $m1$; translation and rotation axes for marker $m1$ are defined in the local body coordinate system and additionally transformed by rotationMarker1"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return false;'),
        ItemFunctionDef('ComputeAlgebraicEquations',
            args='Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false'),
        ItemFunctionDef('ComputeJacobianAE'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Position', 'Orientation']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);',
            description=r'return object type (for node treatment in computation)'),
        ItemFunctionDef('GetAlgebraicEquationsSize',
            implementation='return 5;'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "JointRevoluteZ";',
            description=r"Get type name of object (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionOffset',
            args='Vector6D& offset, const MainSystemBase& mainSystem, Real t, Index itemIndex',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionOffset_t',
            args='Vector6D& offset, const MainSystemBase& mainSystem, Real t, Index itemIndex',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='axisRadius',
            defaultValue=0.1,
            description=r'radius of joint axis to draw'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='axisLength',
            defaultValue=0.4,
            description=r'length of joint axis to draw'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectJointPrismaticX   +++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectJointPrismaticX',
    addIncludesC=r"""class MainSystem; //AUTO; for std::function / userFunction; avoid including MainSystem.h
""",
    addProtectedC=r"""    static constexpr Index nConstraints = 5;
""",
    cParentClass=ParentClassCObjectConstraint,
    classDescription=r"""A prismatic joint in 3D; constrains the relative rotation of two rigid body markers and relative motion w.r.t. the joint $y$ and $z$ axes, allowing a relative motion along the joint $x$ axis (defined in local coordinates of marker 0 / joint J0 coordinates). An additional local rotation (rotationMarker) can be used to transform the markers' coordinate systems into the joint coordinate system. For easier definition of the joint, use the exudyn.rigidbodyUtilities function AddPrismaticJoint(...), [](#sec-rigidbodyutilities-addprismaticjoint), for two rigid bodies (or ground). \addExampleImage{PrismaticJointX}""",
    classType=ClassTypeObject,
    equations=r"""    (sec-objectjointprismaticx-definitionofquantities)=
    #### Definition of quantities


    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | current global position which is provided by marker m0 |
    | marker m0 orientation | $\LU{0,m0}{\Rot}$ | current rotation matrix provided by marker m0 |
    | joint J0 orientation | $\LU{0,J0}{\Rot} = \LU{0,m0}{\Rot} \LU{m0,J0}{\Rot}$ | joint $J0$ rotation matrix |
    | joint J0 orientation vectors | $\LU{0,J0}{\Rot} = [\LU{0}{\tv_{x0}},\,\LU{0}{\tv_{y0}},\,\LU{0}{\tv_{z0}}]\tp$ | orientation vectors (represent local $x$, $y$, and $z$ axes) in global coordinates, used for definition of constraint equations |
    | marker m1 position | $\LU{0}{\pv}_{m1}$ | accordingly |
    | marker m1 orientation | $\LU{0,m1}{\Rot}$ | current rotation matrix provided by marker m1 |
    | joint J1 orientation | $\LU{0,J1}{\Rot} = \LU{0,m1}{\Rot} \LU{m1,J1}{\Rot}$ | joint $J1$ rotation matrix |
    | joint J1 orientation vectors | $\LU{0,J1}{\Rot} = [\LU{0}{\tv_{x1}},\,\LU{0}{\tv_{y1}},\,v\tv_{z1}]\tp$ | orientation vectors (represent local $x$, $y$, and $z$ axes) in global coordinates, used for definition of constraint equations |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ | accordingly |
    | marker m0 velocity | $\LU{b}{\tomega}_{m0}$ | current local angular velocity vector provided by marker m0 |
    | marker m1 velocity | $\LU{b}{\tomega}_{m1}$ | current local angular velocity vector provided by marker m1 |
    | Displacement | $\LU{0}{\Delta\pv}=\LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0}$ | used, if all translational axes are constrained |
    | DisplacementLocal | $\LU{J0}{\Delta\pv}$ | $\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \LU{0}{\Delta\pv}$ |
    | VelocityLocal | $\LU{J0}{\Delta\vv}$ | $\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \LU{0}{\Delta\vv}$ $\ldots$ note that this is the global relative velocity projected into the local $J0$ coordinate system |
    | AngularVelocityLocal | $\LU{J0}{\Delta\omega}$ | $\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \left( \LU{0,m1}{\Rot} \LU{m1}{\omega} - \LU{0,m0}{\Rot} \LU{m0}{\omega} \right)$ |
    | algebraic variables | $\zv=[\lambda_0,\,\ldots,\,\lambda_5]\tp$ | vector of algebraic variables (Lagrange multipliers) according to the algebraic equations |

    <!-- -->

    #### Connector constraint equations

    \paragraph{Equations for translational part (\texttt{activeConnector = True})}:\\
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    The two translational index 3 constraints for a free motion along the local $x$-axis read (in the coordinate system $J0$),


    $$
    \begin{aligned}
    \LU{J0}{\pv}_{y,m1} - \LU{J0}{\pv}_{y,m0} &= \Null \\
          \LU{J0}{\pv}_{z,m1} - \LU{J0}{\pv}_{z,m0} &= \Null
    \end{aligned}
    $$

    and the translational index 2 constraints read


    $$
    \begin{aligned}
    \LU{J0}{\vv}_{y,m1} - \LU{J0}{\vv}_{y,m0} &= \Null \\
          \LU{J0}{\vv}_{z,m1} - \LU{J0}{\vv}_{z,m0} &= \Null
    \end{aligned}
    $$

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    \paragraph{Equations for rotational part (\texttt{activeConnector = True})}:\\
    Note that the axes are always given in global coordinates, compare the table in 
    [](#sec-objectjointprismaticx-definitionofquantities).
    <!-- -->
    The index 3 constraint equations read


    $$
    \begin{aligned}
    \LU{0}{\tv}_{z0}\tp \LU{0}{\tv}_{y1} &= 0 \\
           \LU{0}{\tv}_{z0}\tp \LU{0}{\tv}_{x1} &= 0 \\
           \LU{0}{\tv}_{x0}\tp \LU{0}{\tv}_{y1} &= 0
    \end{aligned}
    $$ (eq-objectjointprismaticx-index3)

    The index 2 constraints follow from the derivative of {eq}`eq-objectjointprismaticx-index3` w.r.t., and are given in the C++ code.
    <!-- -->
    if \texttt{activeConnector = False}, 


    $$
    \zv = \Null
    $$

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeJoint,
    outputVariables=[
        ItemOutputVariable(OVPosition, OVDPositionMarker0),
        ItemOutputVariable(OVVelocity, OVDVelocityMarker0),
        ItemOutputVariable(OVDisplacementLocal, r"""$\LU{J0}{\Delta\pv}$relative displacement in local joint0 coordinates; uses local J0 coordinates even for spherical joint configuration"""),
        ItemOutputVariable(OVVelocityLocal, OVDVelocityLocalJoint),
        ItemOutputVariable(OVRotation, r"""$\LU{J0}{\ttheta}= [\theta_0,\theta_1,\theta_2]\tp$relative rotation parameters (Tait Bryan Rxyz); if all axes are fixed, this output represents the rotational drift; for a revolute joint, it contains the rotation of this axis"""),
        ItemOutputVariable(OVAngularVelocityLocal, r"""$\LU{J0}{\Delta\tomega}$relative angular velocity in local joint0 coordinates; if all axes are fixed, this output represents the angular velocity constraint error; for a revolute joint, it contains the angular velocity of this axis"""),
        ItemOutputVariable(OVForceLocal, r'$\LU{J0}{\fv}$joint force in local $J0$ coordinates'),
        ItemOutputVariable(OVTorqueLocal, r"""$\LU{J0}{\mv}$joint torque in local $J0$ coordinates; depending on joint configuration, the result may not be the according torque vector"""),
        ],
    pythonShortName='PrismaticJointX',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker, size=2), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'$[m0,m1]\tp$list of markers used in connector'),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp+DestParam,
            pythonName='rotationMarker0',
            defaultValue='EXUmath::unitMatrix3D',
            description=r"""$\LU{m0,J0}{\Rot}$local rotation matrix for marker $m0$; translation and rotation axes for marker $m0$ are defined in the local body coordinate system and additionally transformed by rotationMarker0"""),
        ItemParameter(type=TMatrixND(3, 3), destination=DestComp+DestParam,
            pythonName='rotationMarker1',
            defaultValue='EXUmath::unitMatrix3D',
            description=r"""$\LU{m1,J1}{\Rot}$local rotation matrix for marker $m1$; translation and rotation axes for marker $m1$ are defined in the local body coordinate system and additionally transformed by rotationMarker1"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return false;'),
        ItemFunctionDef('ComputeAlgebraicEquations',
            args='Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false'),
        ItemFunctionDef('ComputeJacobianAE'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Position', 'Orientation']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);',
            description=r'return object type (for node treatment in computation)'),
        ItemFunctionDef('GetAlgebraicEquationsSize',
            implementation='return 5;'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "JointPrismaticX";',
            description=r"Get type name of object (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionOffset',
            args='Vector6D& offset, const MainSystemBase& mainSystem, Real t, Index itemIndex',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunction(type=Tvoid, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='EvaluateUserFunctionOffset_t',
            args='Vector6D& offset, const MainSystemBase& mainSystem, Real t, Index itemIndex',
            description=r'call to user function implemented in separate file to avoid including pybind and MainSystem.h at too many places'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='axisRadius',
            defaultValue=0.1,
            description=r'radius of joint axis to draw'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='axisLength',
            defaultValue=0.4,
            description=r'length of joint axis to draw'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectJointSpherical   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectJointSpherical',
    addProtectedC=r"""    static constexpr Index nConstraints = 3;
""",
    cParentClass=ParentClassCObjectConstraint,
    classDescription=r'A spherical joint, which constrains the relative translation between two position based markers. \addExampleImage{SphericalJoint}',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities


    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | current global position which is provided by marker $m0$ |
    | marker m1 position | $\LU{0}{\pv}_{m1}$ | current global position which is provided by marker $m1$ |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker $m0$ |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ | current global velocity which is provided by marker $m1$ |
    | relative velocity | $\LU{0}{\Delta\vv} = \LU{0}{\vv}_{m1} - \LU{0}{\vv}_{m0}$ | constraint velocity error, or relative velocity if not all axes fixed |
    | algebraic variables | $\zv=[\lambda_0,\,\ldots,\,\lambda_2]\tp$ | vector of algebraic variables (Lagrange multipliers) according to the algebraic equations |

    <!-- -->

    #### Connector constraint equations

    \paragraph{\texttt{activeConnector = True}:}
    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    If $[j_0,\,\ldots,\,j_2] = [1,1,1]\tp$, meaning that all translational coordinates are fixed,
    the translational index 3 constraints read


    $$
    \LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0} = \Null
    $$

    and the translational index 2 constraints read


    $$
    \LU{0}{\vv}_{m1} - \LU{0}{\vv}_{m0} = \Null
    $$

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    If $[j_0,\,\ldots,\,j_2] \neq [1,1,1]\tp$, meaning that at least one translational coordinate is free,
    the translational index 3 constraints read for every component $k \in [0,1,2]$ of the vector $\LU{0}{\Delta\pv}$


    $$
    \begin{aligned}
    \LU{0}{\Delta p_k} &= 0 \quad \mathrm{if} \quad j_k = 1 \quad \mathrm{and}\\
          \lambda_k &= 0 \quad \mathrm{if} \quad j_k = 0 \\
    \end{aligned}
    $$

    and the translational index 2 constraints read for every component $k \in [0,1,2]$ of the vector $\LU{0}{\Delta\vv}$


    $$
    \begin{aligned}
    \LU{0}{\Delta v_k} &= 0 \quad \mathrm{if} \quad j_k = 1 \quad \mathrm{and}\\
          \lambda_k &= 0 \quad \mathrm{if} \quad j_k = 0 \\
    \end{aligned}
    $$

    <!-- -->
    \paragraph{\texttt{activeConnector = False}:}


    $$
    \zv = \Null
    $$

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->

    #### Example for body position marker

    <!-- -->
    In this example, we study the constraint equations for two body position marker, see [](#sec-item-markerbodyposition),
    based on rigid bodies, see [](#sec-item-objectrigidbody). 
    The markers $m_0$ and $m_1$ have the positions


    $$
    \LU{0}{\pv_0}(\pLocB_0) = \LU{0}{\rv_{\mathrm{ref},0}} + \LU{0}{\uv_{0}} + \LU{0b}{\Rot_0}\pLocB_0, \quad
          \LU{0}{\pv_1}(\pLocB_1) = \LU{0}{\rv_{\mathrm{ref},1}} + \LU{0}{\uv_{1}} + \LU{0b}{\Rot_1}\pLocB_1 \, .
    $$

    From there, we can derive the 3 constraint equation


    $$
    \LU{0}{\rv_{\mathrm{ref},1}} + \LU{0}{\uv_{1}} + \LU{0b}{\Rot_1}\pLocB_1 - 
          \left(\LU{0}{\rv_{\mathrm{ref},0}} + \LU{0}{\uv_{0}} + \LU{0b}{\Rot_0}\pLocB_0 \right) = \Null \, .
    $$

    The constraint jacobians simply follow from the position jacobians of the respective markers $\LU{0}{\Jm_\mathrm{pos,0}}$
    and  $\LU{0}{\Jm_\mathrm{pos,1}}$. 
    The position jacobians are added to the system jacobian at rows according to the global indices of the constraint equations
    and the columns are determined by the coordinate indices of the bodies' coordinates.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeJoint,
    outputVariables=[
        ItemOutputVariable(OVPosition, OVDPositionMarker0),
        ItemOutputVariable(OVVelocity, OVDVelocityMarker0),
        ItemOutputVariable(OVDisplacement, r"""$\LU{0}{\Delta\pv}=\LU{0}{\pv}_{m1} - \LU{0}{\pv}_{m0}$constraint drift or relative motion, if not all axes fixed"""),
        ItemOutputVariable(OVForce, r'$\LU{0}{\fv}$joint force in global coordinates'),
        ],
    pythonShortName='SphericalJoint',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker, size=2), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r"""$[m0,m1]\tp$list of markers used in connector; $m1$ is the moving coin rigid body and $m0$ is the marker for the ground body, which use the localPosition=[0,0,0] for this marker!"""),
        ItemParameter(type=TArrayIndex(size=3), destination=DestComp+DestParam,
            pythonName='constrainedAxes',
            defaultValue='ArrayIndex({1,1,1})',
            description=r"""$\jv=[j_0,\,\ldots,\,j_2]$flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; for $j_i$, two values are possible: 0=free axis, 1=constrained axis"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return false;'),
        ItemFunctionDef('ComputeAlgebraicEquations',
            args='Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false'),
        ItemFunctionDef('ComputeJacobianAE'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Position']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);',
            description=r'return object type (for node treatment in computation)'),
        ItemFunctionDef('GetAlgebraicEquationsSize',
            implementation='return 3;'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "JointSpherical";',
            description=r"Get type name of object (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='jointRadius',
            defaultValue=0.1,
            description=r'radius of joint to draw'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectJointRollingDisc   ++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectJointRollingDisc',
    addProtectedC=r"""    static constexpr Index nConstraints = 3;
""",
    cParentClass=ParentClassCObjectConstraint,
    classDescription=r'A joint representing a rolling rigid disc (marker 1) on a flat surface (marker 0, ground body) in global $x$-$y$ plane. The contraint is based on an idealized rolling formulation with no slip. The contraints works for discs as long as the disc axis and the plane normal vector are not parallel. It must be assured that the disc has contact to ground in the initial configuration (adjust z-position of body accordingly). The ground body can be a rigid body which is moving. In this case, the flat surface is assumed to be in the $x$-$y$-plane at $z=0$. Note that the rolling body must have the reference point at the center of the disc. NOTE: the cases of normal other than $z$-direction, wheel axis other than $x$-axis and moving ground body needs to be tested further, check your results!',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities


    | intermediate variables | symbol | description |
    |---|---|---|
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | current global position of marker $m0$; needed only if body $m0$ is not a ground body |
    | marker m0 orientation | $\LU{0,m0}{\Rot}$ | current rotation matrix provided by marker m0 (assumed to be rigid body) |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 (assumed to be rigid body) |
    | marker m0 angular velocity | $\LU{0}{\tomega}_{m0}$ | current angular velocity vector provided by marker m0 (assumed to be rigid body) |
    | marker m1 position | $\LU{0}{\pv}_{m1}$ | center of disc |
    | marker m1 orientation | $\LU{0,m1}{\Rot}$ | current rotation matrix provided by marker m1 |
    | marker m1 velocity | $\LU{0}{\vv}_{m1}$ | accordingly |
    | marker m1 angular velocity | $\LU{0}{\tomega}_{m1}$ | current angular velocity vector provided by marker m1 |
    | ground normal vector | $\LU{0}{\vv_{PN}} = \LU{0,m0}{\Am} \LU{m0}{\vv_{PN}}$ | normalized normal vector to the ground plane, moving with marker $m0$ |
    | ground position B | $\LU{0}{\pv}_{B}$ | disc center point projected on ground in plane normal ($z$-direction, $z=0$) |
    | ground position C | $\LU{0}{\pv}_{C}$ | contact point of disc with ground in global coordinates |
    | ground velocity C | $\LU{0}{\vv}_{Cm1}$ | velocity of disc (marker 1) at ground contact point (must be zero if ground does not move) |
    | ground velocity C | $\LU{0}{\vv}_{Cm2}$ | velocity of ground (marker 0) at ground contact point (is always zero if ground does not move) |
    | wheel axis vector | $\LU{0}{\wv_1} =\LU{0,m1}{\Rot} \LU{m1}{\wv_{1}} $ | normalized disc axis vector |
    | longitudinal vector | $\LU{0}{\wv_2}$ | vector in longitudinal (motion) direction |
    | lateral vector | $\LU{0}{\wv_{lat}} = \LU{0}{\vv_{PN}} \times \LU{0}{\wv}_2$ | vector in lateral direction, parallel to ground plane |
    | contact point vector | $\LU{0}{\wv_3}$ | normalized vector from disc center point in direction of contact point C |
    | $D1$ transformation matrix | $\LU{0,D1}{\Am} = [\LU{0}{\wv_1},\, \LU{0}{\wv_2},\, \LU{0}{\wv_3}]$ | transformation of special disc coordinates $D1$ to global coordinates |
    | $J1$ transformation matrix | $\LU{0,J1}{\Am} = [\LU{0}{\wv_{lat}},\, \LU{0}{\wv}_2,\, \LU{0}{\vv_{PN}}]$ | transformation of special joint $J1$ coordinates to global coordinates |
    | algebraic variables | $\zv=[\lambda_0,\,\lambda_1,\,\lambda_2]\tp$ | vector of algebraic variables (Lagrange multipliers) according to the algebraic equations |

    <!-- -->

    #### Geometric relations

    <!--++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ -->
    \noindent The main geometrical setup is shown in the following figure:
    First, the contact point $\LU{0}{\pv}_{C}$ must be computed.
    With the helper vector,


    $$
    \LU{0}{\xv} = \LU{0}{\wv}_1 \times \LU{0}{\vv_{PN}}
    $$

    we obtain a disc coordinate system, representing the longitudinal direction,


    $$
    \LU{0}{\wv}_2 = \frac{1}{|\LU{0}{\xv}|} \LU{0}{\xv}
    $$

    and the vector to the contact point,


    $$
    \LU{0}{\wv}_3 = \LU{0}{\wv}_1 \times \LU{0}{\wv}_2
    $$

    The contact point $C$ can be computed from


    $$
    \LU{0}{\pv}_{C} = \LU{0}{\pv}_{m1} + r \cdot \LU{0}{\wv}_3
    $$

    The velocity of the contact point at the disc is computed from,


    $$
    \LU{0}{\vv}_{Cm1} = \LU{0}{\vv}_{m1} + \LU{0}{\tomega}_{m1} \times (r\cdot \LU{0}{\wv}_3)
    $$

    If marker 0 body is (moving) rigid body instead of a ground body, the contact point $C$ is reconstructed in 
    body of marker 0,


    $$
    \LU{m0}{\pv}_{C} = \LU{m0,0}{\Rot} (\LU{0}{\pv}_{C} - \LU{0}{\pv}_{m0})
    $$

    The velocity of the contact point at the marker 0 body reads


    $$
    \LU{0}{\vv}_{Cm0} = \LU{0}{\vv}_{m0} + \LU{0}{\tomega}_{m0} \times \left( \LU{0,m0}{\Rot} \LU{m0}{\pv}_{C} \right)
    $$

    <!-- -->

    #### Connector constraint equations

    \noindent Constraints for \texttt{activeConnector = True}:\\
    <!-- -->
    The non-holonomic, index 2 constraints for the tangential and normal contact follow from (an index 3 formulation would be possible, but is not implemented yet because of mixing different jacobians)


    $$
    \LU{J1,0}{\Am} \left(\vr{\LU{0}{\vv}_{Cm1,x}}{\LU{0}{\vv}_{Cm1,y}}{\LU{0}{\vv}_{Cm1,z}} - \vr{\LU{0}{\vv}_{Cm0,x}}{\LU{0}{\vv}_{Cm0,y}}{\LU{0}{\vv}_{Cm0,z}} \right) = \Null
    $$

    \noindent In case that \texttt{activeConnector = False}, the Lagrange multipliers are set to zero:


    $$
    \zv = \Null
    $$

    Note that since version 1.8.27 the constraints can be turned on/off separately with \texttt{constrainedAxes=[b0,b1,b2]}, in which
    \texttt{b0} represents the flag for lateral motion, \texttt{b1} switches the constraint for forward motion and \texttt{b2} for motion in plane normal direction.
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeJoint,
    outputVariables=[
        ItemOutputVariable(OVPosition, r"""$\LU{0}{\pv}_{G}$current global position of contact point between rolling disc and ground"""),
        ItemOutputVariable(OVVelocity, r"""$\LU{0}{\vv}_{trail}$current velocity of the trail (according to motion of the contact point along the trail!) in global coordinates; this is not the velocity of the contact point; needs further testing for general case of relative moving bodies"""),
        ItemOutputVariable(OVForceLocal, r"""$\LU{J1}{\fv} = \LU{0}{[f_0,\, f_1,\, f_2]\tp}= [-\zv^T \LU{0}{\wv_{lat}}, \, -\zv^T \LU{0}{\wv_2}, \, -\zv^T \LU{0}{\vv_{PN}}]\tp$contact forces acting on disc, in special $J1$ joint coordinates, $f_0$ being the lateral force (parallel to ground plane), $f_1$ being the longitudinal force and $f_2$ being the normal force"""),
        ItemOutputVariable(OVRotationMatrix, r"""$\LU{0,J1}{\Am} = [\LU{0}{\wv_{lat}},\, \LU{0}{\wv}_2,\, \LU{0}{\vv_{PN}}]$transformation matrix of special joint coordinates $J1$ to global coordinates"""),
        ],
    pythonShortName='RollingDiscJoint',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker, size=2), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r"""$[m0,m1]\tp$list of markers used in connector; $m0$ represents the ground and $m1$ represents the rolling body, which has its reference point (=local position [0,0,0]) at the disc center point"""),
        ItemParameter(type=TArrayIndex(size=3), destination=DestComp+DestParam,
            pythonName='constrainedAxes',
            defaultValue='ArrayIndex({1,1,1})',
            description=r"""$\jv=[j_0,\,\ldots,\,j_2]$flags, which determine which constraints are active, in which $j_0$ represents lateral motion, $j_1$ longitudinal (forward/backward) motion and $j_2$ represents the normal (contact) direction"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemParameter(type=TReal(greaterThan=0), destination=DestComp+DestParam, cFlags=CFMustBeGiven,
            pythonName='discRadius',
            defaultValue=0,
            description=r'defines the disc radius'),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='discAxis',
            defaultValue='Vector3D({1,0,0})',
            description=r"""$\LU{m1}{\wv_{1}}, \;\; |\LU{m1}{\wv_{1}}| = 1$axis of disc defined in marker $m1$ frame"""),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='planeNormal',
            defaultValue='Vector3D({0,0,1})',
            description=r"""$\LU{m0}{\vv_{PN}}$normal to the contact / rolling plane defined in marker $m0$ coordinates"""),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return false;'),
        ItemFunctionDef('ComputeAlgebraicEquations',
            args='Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false'),
        ItemFunctionDef('ComputeJacobianAE'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunctionDef('UsesVelocityLevel',
            implementation='return true;'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Position', 'Orientation']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);',
            description=r'return object type (for node treatment in computation)'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('GetAlgebraicEquationsSize',
            implementation='return 3;'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "JointRollingDisc";',
            description=r"Get type name of object (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='discWidth',
            defaultValue=0.1,
            description=r'width of disc for drawing'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectJointRevolute2D   +++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectJointRevolute2D',
    cParentClass=ParentClassCObjectConstraint,
    classDescription=r'A revolute joint in 2D; constrains the absolute 2D position of two points given by PointMarkers or RigidMarkers',
    classType=ClassTypeObject,
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeJoint,
    pythonShortName='RevoluteJoint2D',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'list of markers used in connector'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return false;'),
        ItemFunctionDef('ComputeAlgebraicEquations',
            args='Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false'),
        ItemFunctionDef('ComputeJacobianAE'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunctionDef('GetOutputVariableTypes'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Position']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);',
            description=r'return object type (for node treatment in computation)'),
        ItemFunctionDef('GetAlgebraicEquationsSize',
            implementation='return 2;'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "JointRevolute2D";',
            description=r"Get type name of object (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = radius of revolute joint; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectJointPrismatic2D   ++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectJointPrismatic2D',
    cParentClass=ParentClassCObjectConstraint,
    classDescription=r'A prismatic joint in 2D; allows the relative motion of two bodies, using two RigidMarkers.',
    classType=ClassTypeObject,
    equations=r"""    #### Geometric relations

    The vector $\tv_0$ = axisMarker0 is given in local coordinates of the first marker's (body) frame and defines the prismatic axis.
    The vector $\mathbf{n}_1$ = normalMarker1 is given in the second marker's (body) frame and is the normal vector to the prismatic axis.
    Using the global position vector $\pv_0$ and rotation matrix $\Am_0$ of marker0 and 
    the global position vector $\pv_1$ rotation matrix $\Am_1$ of marker1, the equations for the prismatic joint follow as 


    $$
    (\pv_1-\pv_0)^T\cdot \Am_1 \cdot \mathbf{n}_1 = 0
    $$
  


    $$
    (\Am_0 \cdot \tv_0)^T \cdot \Am_1 \cdot \mathbf{n}_1 = 0
    $$
 
    The Lagrange multipliers follow for these two equations $[\lambda_0,\lambda_1]$, 
    in which $\lambda_0$ is the transverse force and $\lambda_1$ is the torque in the joint.
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeJoint,
    pythonShortName='PrismaticJoint2D',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r'list of markers used in connector'),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='axisMarker0',
            defaultValue='Vector3D({1.,0.,0.})',
            description=r'direction of prismatic axis, given as a 3D vector in Marker0 frame'),
        ItemParameter(type=TVectorND(3), destination=DestComp+DestParam,
            pythonName='normalMarker1',
            defaultValue='Vector3D({0.,1.,0.})',
            description=r'direction of normal to prismatic axis, given as a 3D vector in Marker1 frame'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='constrainRotation',
            defaultValue=True,
            description=r'flag, which determines, if the connector also constrains the relative rotation of the two objects; if set to false, the constraint will keep an algebraic equation set equal zero'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return false;'),
        ItemFunctionDef('ComputeAlgebraicEquations',
            args='Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false'),
        ItemFunctionDef('ComputeJacobianAE'),
        ItemFunctionDef('GetAvailableJacobians'),
        ItemFunctionDef('GetOutputVariableTypes'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', ['Position', 'Orientation']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);',
            description=r'return object type (for node treatment in computation)'),
        ItemFunctionDef('GetAlgebraicEquationsSize',
            implementation='return 2;'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "JointPrismatic2D";',
            description=r"Get type name of object (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = radius of revolute joint; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectJointSliding   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectJointSliding',
    addProtectedC=r"""    static constexpr Index slidingCoordinateIndex = 0; //!< index of alqebraic coordinate
    static constexpr Index forcesStartIndex = 1; //!< starting index of alqebraic coordinates for forces
    static constexpr Index torquesStartIndex = 4; //!< starting index of alqebraic coordinates for torques (if existing)
""",
    cParentClass=ParentClassCObjectConstraint,
    classDescription=r'A specialized 3D sliding joint between a list of beam elements (updated marker1) and a position-based marker (marker0); the data coordinate x[0] provides the current index in slidingMarkerNumbers, and x[1] the local position in the cable element at the beginning of the timestep.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    <!--
    
    \rowTable{cable coordinates}{$\qv_{ANCF,m1}$}{current coordiantes of the ANCF cable element with the current marker $m1$ is referring to}
    -->

    | intermediate variables | symbol | description |
    |---|---|---|
    | data node | $\xv=[x_{data0},\,x_{data1}]\tp$ | coordinates of node with node number $n_{GD}$ |
    | data coordinate 0 | $x_{data0}$ | the current index in slidingMarkerNumbers |
    | data coordinate 1 | $x_{data1}$ | the global sliding coordinate (ranging from 0 to the total length of all sliding elements) at {\bf start-of-step} - beginning of the timestep |
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | current global position which is provided by marker m0 |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | marker m0 orientation | $\LU{0,m0}{\Rot}$ | current rotation matrix provided by marker m0 (assumed to be rigid body) |
    | marker m0 angular velocity | $\LU{0}{\tomega}_{m0}$ | current angular velocity vector provided by marker m0 (assumed to be rigid body) |
    | sliding position | $\LUR{0}{\rv}{ANCF} = \Sm(s_{el})\qv_{ANCF,m1}$ | current global position at the ANCF cable element, evaluated at local sliding position $s_{el}$ |
    | sliding position slope | $\LURU{0}{\rv}{ANCF}{\prime} = \Sm^\prime(s_{el})\qv_{ANCF,m1} = [r^\prime_0,\,r^\prime_1,\,r^\prime_2]\tp$ | current global slope vector of the ANCF cable element, evaluated at local sliding position $s_{el}$ |
    | sliding velocity | $\LUR{0}{\vv}{ANCF} = \Sm(s_{el})\dot\qv_{ANCF,m1}$ | current global velocity at the ANCF cable element, evaluated at local sliding position $s_{el}$ ($s_{el}$ not differentiated!!!) |
    | sliding velocity slope | $\LURU{0}{\vv}{ANCF}{\prime} = \Sm^\prime(s_{el})\dot\qv_{ANCF,m1}$ | current global slope velocity vector of the ANCF cable element, evaluated at local sliding position $s_{el}$ |
    | algebraic coordinates | $\zv=[\lambda_0,\,\ldots,\,\lambda_5,\, s]\tp$ | algebraic coordinates composed of 3 Lagrange multipliers for forces $\lambda_{0..2}$, 3 multipliers for torques $\lambda_{3..5}$ and the current sliding coordinate $s$, which is local in the current cable element. |
    | local sliding coordinate | $s$ | local incremental sliding coordinate $s$: the (algebraic) sliding coordinate {\bf relative to the start-of-step value}. Thus, $s$ only contains small local increments. |


    | output variables | symbol | formula |
    |---|---|---|
    | Position | $\LU{0}{\pv}_{m0}$ | current global position of position marker $m0$ |
    | Velocity | $\LU{0}{\vv}_{m0}$ | current global velocity of position marker $m0$ |
    | SlidingCoordinate | $s_g = s + x_{data1}$ | current value of the global sliding coordinate |
    | Force | $\fv$ | see below |


    #### Geometric relations

    <!--cable -->
    Assume we have given the sliding coordinate $s$ (e.g., as a guess of the Newton method or beginning of the time step). 
    The element sliding coordinate (in the local coordinates of the current sliding element) is computed as


    $$
    s_{el} = s + x_{data1} - d_{m1} = s_g - d_{m1}.
    $$

    The vector (=difference; error) between the marker $m0$ and the marker $m1$ (=$\rv_{ANCF}$) positions reads


    $$
    \LU{0}{\Delta\pv} = \LUR{0}{\rv}{ANCF} - \LU{0}{\pv}_{m0}
    $$

    The vector (=difference; error) between the marker $m0$ and the marker $m1$ velocities reads


    $$
    \LU{0}{\Delta\vv} = \LUR{0}{\dot\rv}{ANCF} - \LU{0}{\vv}_{m0}
    $$

    <!--
    
    +++++++++++++++++++++++++++++++++++++++++++++
    -->

    #### Connector constraint equations (classicalFormulation=True)

    The 3D sliding joint is implemented having 7 equations, using the special algebraic coordinates $\zv$.
    The algebraic equations read


    $$
    \begin{aligned}
    \LU{0}{\Delta\pv} \!&=\! \Null, \quad \mbox{... three index 3 eqs $\ra$ sliding body stays on cable}\\
          \vr{\lambda_1}{\lambda_2}{\lambda_3} \cdot  \LURU{0}{\rv}{ANCF}{\prime} - |\LURU{0}{\rv}{ANCF}{\prime}| \cdot f_\mathrm{ax} \!&=\! 0, \quad \mbox{... three index 1 equ. 
                                                   $\ra$ force in sliding dir.=$f_\mathrm{ax}$}  \\
    \end{aligned}
    $$

    No index 2 case exists, because no time derivative exists for $s_{el}$. The jacobian matrices for algebraic and ABRV:ODE2 coordinates read
    <!--
    \be
      \Jm_{AE} = \mr{0}{0}{r^\prime_0} {0}{0}{r^\prime_1} {r^\prime_0}{r^\prime_1}{r^{\prime\prime}_0\lambda_0 + r^{\prime\prime}_1\lambda_1}    %\LURU{0}{\rv}{ANCF}{\prime\prime \mathrm{T}} \vp{\lambda_0}{\lambda_1}}
    \ee
    \be
      \Jm_{ODE2} = \mp{-J_{pos,m0}}{\Sm(s_{el})} {\Null\tp}{\left[\lambda_0,\,\lambda_1,\,\lambda_2\right]\cdot\Sm^\prime(s_{el}) }
    \ee
    -->
    if \texttt{activeConnector = False}, the algebraic equations are changed to:


    $$
    \begin{aligned}
    \lambda_0 &= 0,   \\
          \ldots && ,   \\
          \lambda_5 &= 0,   \\
          s &= 0
    \end{aligned}
    $$

    <!--
    +++++++++++++++++++++++++++++++++++++++++++++
    for (classicalFormulation=False), see 2D case!
    +++++++++++++++++++++++++++++++++++++++++++++
    -->
    In case that \texttt{constrainRotations=[0,0,0]}, the Lagrange multipliers for rotations are set


    $$
    \left[\lambda_3,\lambda_4,\lambda_5\right] = \Null
    $$

    In case that any flag in \texttt{constrainRotations} is equal to 1, the constraints read 
    for \texttt{constrainRotations[0] = 1}:


    $$
    \LURU{0}{\rv}{ANCF,y}{\mathrm{T}} \LU{0,m0}{\Rot} \vr{0}{0}{1} = 0
    $$

    for \texttt{constrainRotations[1] = 1}:


    $$
    \LURU{0}{\rv}{ANCF}{\prime\mathrm{T}} \LU{0,m0}{\Rot} \vr{0}{0}{1} = 0
    $$

    for \texttt{constrainRotations[2] = 0}:


    $$
    \LURU{0}{\rv}{ANCF}{\prime\mathrm{T}} \LU{0,m0}{\Rot} \vr{0}{1}{0} = 0
    $$


    <!--
    \noindent The index 2 case follows straightforward to
    \be
      \LURU{0}{\dot \rv}{ANCF}{\prime\mathrm{T}} \LU{0,m0}{\Rot} \vp{0}{1}  +
      \LURU{0}{\rv}{ANCF}{\prime\mathrm{T}} \LU{0,m0}{\Rot} \LU{0}{\tilde \tomega}_{m0} \vp{0}{1} = 0
    \ee
    again assuming, that $\LU{0}{\tilde \tomega}_{m0}$ is only a $2 \times 2$ matrix.
    +++++++++++++++++++++++++++++++++++++++++++++
    -->

    #### Post Newton Step

    After the Newton solver has converged, a PostNewtonStep is performed for the element, which
    updates the marker $m1$ index if necessary.


    $$
    \begin{aligned}
    s_{el} < 0 \quad \ra \quad x_{data0}\;-\!\!=1 \\
          s_{el} > L \quad \ra \quad x_{data0}\;+\!\!=1
    \end{aligned}
    $$

    Furthermore, it is checked, if $x_{data0}$ becomes smaller than zero, which raises a warning and keeps $x_{data0}=0$.
    The same results if $x_{data0}\ge sn$, then $x_{data0} = sn$.
    Finally, the data coordinate is updated in order to provide the starting value for the next step,


    $$
    x_{data1} \;+\!\!= s.
    $$

    <!--the data coordinates are \be \qv_{Data} = [i_{marker} \;\; s_{0}]^T \ee in which $i_{marker}$ is the current local index to the slidingMarkerNumber list and  $s_{0}$ is the sliding coordinate (which is the total sliding length along all cable elements in the cableMarkerNumber list) at the beginning of the solution step. -->
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeJoint,
    outputVariables=[
        ItemOutputVariable(OVPosition, 'position vector of joint given by marker0'),
        ItemOutputVariable(OVVelocity, 'velocity vector of joint given by marker0'),
        ItemOutputVariable(OVSlidingCoordinate, 'global sliding coordinate along all elements; the maximum sliding coordinate is equivalent to the reference lengths of all sliding elements'),
        ItemOutputVariable(OVForce, 'joint force vector (3D)'),
        ],
    pythonShortName='SlidingJoint',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r"""$[m0,m1]\tp$marker m0: position or rigid body marker of mass point or rigid body; marker m1: updated marker to Cable2D element, where the sliding joint currently is attached to; must be initialized with an appropriate (global) marker number according to the starting position of the sliding object; this marker changes with time (PostNewtonStep)"""),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='slidingMarkerNumbers',
            defaultValue='ArrayIndex()',
            description=r"""$[m_{s0}, \ldots, m_{sn}]\tp$these markers are used to update marker m1, if the sliding position exceeds the current cable's range; the markers must be sorted such that marker $m_{si}$ at x=cable(i).length is equal to marker(i+1) at x=0 of cable(i+1)"""),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='slidingMarkerOffsets',
            defaultValue='Vector()',
            description=r"""$[d_{s0}, \ldots, d_{sn}]$this list contains the offsets of every sliding object (given by slidingMarkerNumbers) w.r.t. to the initial position (0): marker m0: offset=0, marker m1: offset=Length(cable0), marker m2: offset=Length(cable0)+Length(cable1), ..."""),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n_{GD}$node number of a NodeGenericData for 1 dataCoordinate showing the according marker number which is currently active and the start-of-step (global) sliding position'),
        ItemParameter(type=TArrayIndex(size=3), destination=DestComp+DestParam,
            pythonName='constrainRotations',
            defaultValue='ArrayIndex({1,1,1})',
            description=r'flags for constrained rotation about x, y and z-axis: if flag=1, add constraint on rotation of marker m0 relative to respective axis; flag=0: sliding body can rotate freely about this axis; for ANCFCable, rotation about x-axis cannot be constrained'),
        ItemParameter(type=TArrayIndex(size=3), destination=DestComp+DestParam,
            pythonName='constrainTranslations',
            defaultValue='ArrayIndex({1,1,1})',
            description=r'flags for constrained translation in x, y and z-direction: if flag=1, add constraint on translation of marker m0 relative to respective axis; flag=0: sliding body can translate freely about this axis; along x-axis this should be usually 0, except for driven motion'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='axialForce',
            defaultValue=0,
            description=r"""$f_\mathrm{ax}$ONLY APPLIES if classicalFormulation==True; axialForce represents an additional sliding force acting between beam and marker m0 body in axial (beam) direction; this force can be used to drive a body on a beam, but can only be changed with user functions."""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetDataVariablesSize',
            implementation='return 2;',
            description='data variables: [0] showing the current (local) index in slidingMarkerNumber list --> providing the cable element active in sliding; coordinate [1] stores the previous sliding coordinate'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return false;'),
        ItemFunctionDef('ComputeAlgebraicEquations',
            args='Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false'),
        ItemFunctionDef('ComputeJacobianAE'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::AE_ODE2 + JacobianType::AE_ODE2_function + JacobianType::AE_AE + JacobianType::AE_AE_function);'),
        ItemFunctionDef('HasDiscontinuousIteration',
            implementation='return true;'),
        ItemFunctionDef('PostNewtonStep'),
        ItemFunctionDef('PostDiscontinuousIterationStep'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', [],
            description=r"""provide requested markerType for connector; for different markerTypes in marker0/1 => set to ::\_None"""),
        ItemRequestedTypes('Node', ['GenericData']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);',
            description=r'return object type (for node treatment in computation)'),
        ItemFunctionDef('GetAlgebraicEquationsSize',
            implementation='return 4 + 3*HasRotationConstraints();',
            description='one sliding coordinate, 3 translation, optional: 3 rotation constraints'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "JointSliding";',
            description=r"Get type name of object (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=TBool, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='HasRotationConstraints',
            implementation='return !(parameters.constrainRotations==0);',
            description=r'add 3 rotation constraints if any is non-zero'),
        ItemFunction(type=TReal, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeLocalSlidingCoordinate',
            description=r'compute the (local) sliding coordinate within the current cable element'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = radius of revolute joint; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectJointSliding2D   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectJointSliding2D',
    cParentClass=ParentClassCObjectConstraint,
    classDescription=r'A specialized sliding joint (without rotation) in 2D between a Cable2D (marker1) and a position-based marker (marker0); the data coordinate x[0] provides the current index in slidingMarkerNumbers, and x[1] the local position in the cable element at the beginning of the timestep.',
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    <!-- -->

    | intermediate variables | symbol | description |
    |---|---|---|
    | data node | $\xv=[x_{data0},\,x_{data1}]\tp$ | coordinates of node with node number $n_{GD}$ |
    | data coordinate 0 | $x_{data0}$ | the current index in slidingMarkerNumbers |
    | data coordinate 1 | $x_{data1}$ | the global sliding coordinate (ranging from 0 to the total length of all sliding elements) at {\bf start-of-step} - beginning of the timestep |
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | current global position which is provided by marker m0 |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | marker m0 orientation | $\LU{0,m0}{\Rot}$ | current rotation matrix provided by marker m0 (assumed to be rigid body) |
    | marker m0 angular velocity | $\LU{0}{\tomega}_{m0}$ | current angular velocity vector provided by marker m0 (assumed to be rigid body) |
    | cable coordinates | $\qv_{ANCF,m1}$ | current coordiantes of the ANCF cable element with the current marker $m1$ is referring to |
    | sliding position | $\LUR{0}{\rv}{ANCF} = \Sm(s_{el})\qv_{ANCF,m1}$ | current global position at the ANCF cable element, evaluated at local sliding position $s_{el}$ |
    | sliding position slope | $\LURU{0}{\rv}{ANCF}{\prime} = \Sm^\prime(s_{el})\qv_{ANCF,m1} = [r^\prime_0,\,r^\prime_1]\tp$ | current global slope vector of the ANCF cable element, evaluated at local sliding position $s_{el}$ |
    | sliding velocity | $\LUR{0}{\vv}{ANCF} = \Sm(s_{el})\dot\qv_{ANCF,m1}$ | current global velocity at the ANCF cable element, evaluated at local sliding position $s_{el}$ ($s_{el}$ not differentiated!!!) |
    | sliding velocity slope | $\LURU{0}{\vv}{ANCF}{\prime} = \Sm^\prime(s_{el})\dot\qv_{ANCF,m1}$ | current global slope velocity vector of the ANCF cable element, evaluated at local sliding position $s_{el}$ |
    | sliding normal vector | $\LU{0}{\nv} = [-r^\prime_1,\,r^\prime_0]$ | 2D normal vector computed from slope $\rv^\prime=\LURU{0}{\rv}{ANCF}{\prime}$ |
    | sliding normal velocity vector | $\LU{0}{\dot\nv} = [-\dot r^\prime_1,\,\dot r^\prime_0]$ | time derivative of 2D normal vector computed from slope velocity $\dot \rv^\prime=\LURU{0}{\dot \rv}{ANCF}{\prime}$ |
    | algebraic coordinates | $\zv=[\lambda_0,\,\lambda_1,\, s]\tp$ | algebraic coordinates composed of Lagrange multipliers $\lambda_0$ and $\lambda_1$ (in local cable coordinates: $\lambda_0$ is in axis direction) and the current sliding coordinate $s$, which is local in the current cable element. |
    | local sliding coordinate | $s$ | local incremental sliding coordinate $s$: the (algebraic) sliding coordinate {\bf relative to the start-of-step value}. Thus, $s$ only contains small local increments. |


    | output variables | symbol | formula |
    |---|---|---|
    | Position | $\LU{0}{\pv}_{m0}$ | current global position of position marker $m0$ |
    | Velocity | $\LU{0}{\vv}_{m0}$ | current global velocity of position marker $m0$ |
    | SlidingCoordinate | $s_g = s + x_{data1}$ | current value of the global sliding coordinate |
    | Force | $\fv$ | see below |


    #### Geometric relations

    <!--cable -->
    Assume we have given the sliding coordinate $s$ (e.g., as a guess of the Newton method or beginning of the time step). 
    The element sliding coordinate (in the local coordinates of the current sliding element) is computed as


    $$
    s_{el} = s + x_{data1} - d_{m1} = s_g - d_{m1}.
    $$

    The vector (=difference; error) between the marker $m0$ and the marker $m1$ (=$\rv_{ANCF}$) positions reads


    $$
    \LU{0}{\Delta\pv} = \LUR{0}{\rv}{ANCF} - \LU{0}{\pv}_{m0}
    $$

    The vector (=difference; error) between the marker $m0$ and the marker $m1$ velocities reads


    $$
    \LU{0}{\Delta\vv} = \LUR{0}{\dot\rv}{ANCF} - \LU{0}{\vv}_{m0}
    $$

    <!--
    
    +++++++++++++++++++++++++++++++++++++++++++++
    -->

    #### Connector constraint equations (classicalFormulation=True)

    The 2D sliding joint is implemented having 3 equations (4 if constrainRotation==True, see below), using the special algebraic coordinates $\zv$.
    The algebraic equations read


    $$
    \begin{aligned}
    \LU{0}{\Delta\pv} &= \Null, \quad \mbox{... two index 3 eqs $\ra$ sliding body stays on cable}\\
          \left[\lambda_0,\lambda_1\right] \cdot  \LURU{0}{\rv}{ANCF}{\prime} - |\LURU{0}{\rv}{ANCF}{\prime}| \cdot f_\mathrm{ax} &= 0, \quad \mbox{... one index 1 equ. 
                                                   $\ra$ force in sliding dir.=$f_\mathrm{ax}$}  \\
    \end{aligned}
    $$

    No index 2 case exists, because no time derivative exists for $s_{el}$. The jacobian matrices for algebraic and ABRV:ODE2 coordinates read


    $$
    \Jm_{AE} = \mr{0}{0}{r^\prime_0} {0}{0}{r^\prime_1} {r^\prime_0}{r^\prime_1}{r^{\prime\prime}_0\lambda_0 + r^{\prime\prime}_1\lambda_1}    %\LURU{0}{\rv}{ANCF}{\prime\prime \mathrm{T}} \vp{\lambda_0}{\lambda_1}}
    $$



    $$
    \Jm_{ODE2} = \mp{-J_{pos,m0}}{\Sm(s_{el})} {\Null\tp}{\left[\lambda_0,\,\lambda_1\right]\cdot\Sm^\prime(s_{el}) }
    $$

    if \texttt{activeConnector = False}, the algebraic equations are changed to:


    $$
    \begin{aligned}
    \lambda_0 &= 0,   \\
          \lambda_1 &= 0,   \\
          s &= 0
    \end{aligned}
    $$

    <!--
    the algebraic variables are \be \qv_{AE}=[\lambda_x\;\; \lambda_y \;\; s]^T \ee in which $\lambda_x$ and $\lambda_y$ are the Lagrange multipliers for the position of the sliding joint;
    +++++++++++++++++++++++++++++++++++++++++++++
    -->

    #### Connector constraint equations (classicalFormulation=False)

    The 2D sliding joint is implemented having 3 equations (first equation is dummy and could be eliminated; 4 equations if constrainRotation==True, see below), using the special algebraic coordinates $\zv$. 
    The algebraic equations read


    $$
    \begin{aligned}
    \lambda_0 &= 0, \quad \mbox{... eq.~not necessary, but can be used for switching to other modes}  \\
          \LU{0}{\Delta\pv\tp} \LU{0}{\nv} &= 0, \quad \mbox{... eq.~ensures that sliding body stays at cable centerline; index3}\\
          \LU{0}{\Delta\pv\tp} \LURU{0}{\rv}{ANCF}{\prime} &= 0. \quad \mbox{... resolves the sliding coordinate $s$; index1 eq.!}
    \end{aligned}
    $$

    In the index 2 case, the second equation reads


    $$
    \LU{0}{\Delta\vv\tp} \LU{0}{\nv}  + \LU{0}{\Delta\pv\tp} \LU{0}{\dot\nv}  = 0 \, .
    $$

    If \texttt{activeConnector = False}, the algebraic equations are changed to:


    $$
    \begin{aligned}
    \lambda_0 &= 0,   \\
          \lambda_1 &= 0,   \\
          s &= 0
    \end{aligned}
    $$
   
    <!--
    the algebraic variables are \be \qv_{AE}=[\lambda_x\;\; \lambda_y \;\; s]^T \ee in which $\lambda_x$ and $\lambda_y$ are the Lagrange multipliers for the position of the sliding joint;
    
    +++++++++++++++++++++++++++++++++++++++++++++
    -->
    In case that \texttt{constrainRotation = True}, an additional constraint is added for the relative rotation
    between the slope of the cable and the orientation of marker m0 body.
    Assuming that the orientation of marker m0 is a 2D matrix (taking only $x$ and $y$ coordinates), the constraint reads


    $$
    \LURU{0}{\rv}{ANCF}{\prime\mathrm{T}} \LU{0,m0}{\Rot} \vp{0}{1} = 0
    $$

    The index 2 case follows straightforward to 


    $$
    \LURU{0}{\dot \rv}{ANCF}{\prime\mathrm{T}} \LU{0,m0}{\Rot} \vp{0}{1}  + 
          \LURU{0}{\rv}{ANCF}{\prime\mathrm{T}} \LU{0,m0}{\Rot} \LU{0}{\tilde \tomega}_{m0} \vp{0}{1} = 0
    $$

    again assuming, that $\LU{0}{\tilde \tomega}_{m0}$ is only a $2 \times 2$ matrix.
    <!--+++++++++++++++++++++++++++++++++++++++++++++ -->

    #### Post Newton Step

    After the Newton solver has converged, a PostNewtonStep is performed for the element, which
    updates the marker $m1$ index if necessary.


    $$
    \begin{aligned}
    s_{el} < 0 \quad \ra \quad x_{data0}\;-\!\!=1 \\
          s_{el} > L \quad \ra \quad x_{data0}\;+\!\!=1
    \end{aligned}
    $$

    Furthermore, it is checked, if $x_{data0}$ becomes smaller than zero, which raises a warning and keeps $x_{data0}=0$.
    The same results if $x_{data0}\ge sn$, then $x_{data0} = sn$.
    Finally, the data coordinate is updated in order to provide the starting value for the next step,


    $$
    x_{data1} \;+\!\!= s.
    $$

    <!--the data coordinates are \be \qv_{Data} = [i_{marker} \;\; s_{0}]^T \ee in which $i_{marker}$ is the current local index to the slidingMarkerNumber list and  $s_{0}$ is the sliding coordinate (which is the total sliding length along all cable elements in the cableMarkerNumber list) at the beginning of the solution step. -->
    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeJoint,
    outputVariables=[
        ItemOutputVariable(OVPosition, 'position vector of joint given by marker0'),
        ItemOutputVariable(OVVelocity, 'velocity vector of joint given by marker0'),
        ItemOutputVariable(OVSlidingCoordinate, 'global sliding coordinate along all elements; the maximum sliding coordinate is equivalent to the reference lengths of all sliding elements'),
        ItemOutputVariable(OVForce, 'joint force vector (3D)'),
        ],
    pythonShortName='SlidingJoint2D',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r"""$[m0,m1]\tp$marker m0: position or rigid body marker of mass point or rigid body; marker m1: updated marker to Cable2D element, where the sliding joint currently is attached to; must be initialized with an appropriate (global) marker number according to the starting position of the sliding object; this marker changes with time (PostNewtonStep)"""),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='slidingMarkerNumbers',
            defaultValue='ArrayIndex()',
            description=r"""$[m_{s0}, \ldots, m_{sn}]\tp$these markers are used to update marker m1, if the sliding position exceeds the current cable's range; the markers must be sorted such that marker $m_{si}$ at x=cable(i).length is equal to marker(i+1) at x=0 of cable(i+1)"""),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='slidingMarkerOffsets',
            defaultValue='Vector()',
            description=r"""$[d_{s0}, \ldots, d_{sn}]$this list contains the offsets of every sliding object (given by slidingMarkerNumbers) w.r.t. to the initial position (0): marker m0: offset=0, marker m1: offset=Length(cable0), marker m2: offset=Length(cable0)+Length(cable1), ..."""),
        ItemParameter(type=TIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumber',
            defaultValue=DVInvalidIndex,
            description=r'$n_{GD}$node number of a NodeGenericData for 1 dataCoordinate showing the according marker number which is currently active and the start-of-step (global) sliding position'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='classicalFormulation',
            defaultValue=True,
            description=r'True: uses a formulation with 3 (+1) equations, including the force in sliding direction to be zero; forces in global coordinates, only index 3; False: use local formulation, which only needs 2 (+1) equations and can be used with index 2 formulation'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='constrainRotation',
            defaultValue=False,
            description=r'True: add constraint on rotation of marker m0 relative to slope (if True, marker m0 must be a rigid body marker); False: marker m0 body can rotate freely'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='axialForce',
            defaultValue=0,
            description=r"""$f_\mathrm{ax}$ONLY APPLIES if classicalFormulation==True; axialForce represents an additional sliding force acting between beam and marker m0 body in axial (beam) direction; this force can be used to drive a body on a beam, but can only be changed with user functions."""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex == 0, __EXUDYN_invalid_local_node);
        return parameters.nodeNumber;"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumber=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 1;'),
        ItemFunctionDef('GetDataVariablesSize',
            implementation='return 2;',
            description='data variables: [0] showing the current (local) index in slidingMarkerNumber list --> providing the cable element active in sliding; coordinate [1] stores the previous sliding coordinate'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return false;'),
        ItemFunctionDef('ComputeAlgebraicEquations',
            args='Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false'),
        ItemFunctionDef('ComputeJacobianAE'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::AE_ODE2 + JacobianType::AE_ODE2_function + JacobianType::AE_AE + JacobianType::AE_AE_function);'),
        ItemFunctionDef('HasDiscontinuousIteration',
            implementation='return true;'),
        ItemFunctionDef('PostNewtonStep'),
        ItemFunctionDef('PostDiscontinuousIterationStep'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', [],
            description=r"""provide requested markerType for connector; for different markerTypes in marker0/1 => set to ::\_None"""),
        ItemRequestedTypes('Node', ['GenericData']),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);',
            description=r'return object type (for node treatment in computation)'),
        ItemFunctionDef('GetAlgebraicEquationsSize',
            implementation='return 3 + (Index)parameters.constrainRotation;',
            description='q0=forceX of sliding joint, q1=forceY of sliding joint; q2=axial (sliding) coordinate at beam; (optional) q3=rotation constraint'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "JointSliding2D";',
            description=r"Get type name of object (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=TReal, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeLocalSlidingCoordinate',
            description=r'compute the (local) sliding coordinate within the current cable element'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = radius of revolute joint; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++   ObjectJointALEMoving2D   ++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
definitions.append(ItemDefinition(
    className='ObjectJointALEMoving2D',
    cParentClass=ParentClassCObjectConstraint,
    classDescription=r"""A specialized axially moving joint (without rotation) in 2D between a ALE Cable2D (marker1) and a position-based marker (marker0); ALE=Arbitrary Lagrangian Eulerian; the data coordinate x[0] provides the current index in slidingMarkerNumbers, and the ABRV:ODE2 coordinate q[0] provides the (given) moving coordinate in the cable element.""",
    classType=ClassTypeObject,
    equations=r"""    #### Definition of quantities

    <!--
    
    \rowTable{sliding normal vector}{$\LU{0}{\dot\nv} = [-\dot r^\prime_1,\,\dot r^\prime_0]$}{time derivative of 2D normal vector computed from slope velocity $\dot \rv^\prime=\LURU{0}{\dot \rv}{ANCF}{\prime}$}
    -->

    | intermediate variables | symbol | description |
    |---|---|---|
    | generic data node | $\xv=[x_{data0}]\tp$ | coordinates of node with node number $n_{GD}$ |
    | generic ABRV:ODE2 node | $\qv=[q_{0}]\tp$ | coordinates of node with node number $n_{ALE}$, which is shared with all ALE-ANCF and ALE sliding joint objects |
    | data coordinate | $x_{data0}$ | the current index in slidingMarkerNumbers |
    | ALE coordinate | $q_{ALE} = q_{0}$ | current ALE coordinate (in fact this is the Eulerian coordinate in the ALE formulation); note that reference coordinate of $q_{ALE}$ is ignored! |
    | marker m0 position | $\LU{0}{\pv}_{m0}$ | current global position which is provided by marker m0 |
    | marker m0 velocity | $\LU{0}{\vv}_{m0}$ | current global velocity which is provided by marker m0 |
    | cable coordinates | $\qv_{ANCF,m1}$ | current coordiantes of the ANCF cable element with the current marker $m1$ is referring to |
    | sliding position | $\LUR{0}{\rv}{ANCF} = \Sm(s_{el})\qv_{ANCF,m1}$ | current global position at the ANCF cable element, evaluated at local sliding position $s_{el}$ |
    | sliding position slope | $\LURU{0}{\rv}{ANCF}{\prime} = \Sm^\prime(s_{el})\qv_{ANCF,m1}$ | current global slope vector of the ANCF cable element, evaluated at local sliding position $s_{el}$ |
    | sliding velocity | $\LUR{0}{\vv}{ANCF} = \Sm(s_{el})\dot\qv_{ANCF,m1} + \dot q_{ALE} \LURU{0}{\rv}{ANCF}{\prime}$ | current global velocity at the ANCF cable element, evaluated at local sliding position $s_{el}$, including convective term |
    | sliding normal vector | $\LU{0}{\nv} = [-r^\prime_1,\,r^\prime_0]$ | 2D normal vector computed from slope $\rv^\prime=\LURU{0}{\rv}{ANCF}{\prime}$ |
    | algebraic variables | $\zv=[\lambda_0,\,\lambda_1]\tp$ | algebraic variables (Lagrange multipliers) according to the algebraic equations |

    <!--
    \startTable{output variables}{symbol}{formula}
    \rowTable{Position}{$\LU{0}{\pv}_{m0}$}{current global position of position marker $m0$}
    \rowTable{Velocity}{$\LU{0}{\vv}_{m0}$}{current global velocity of position marker $m0$}
    \rowTable{SlidingCoordinate}{$s_g = q_{ALE} + s_\mathrm{off}$}{current value of the global sliding ALE coordinate, including offset; note that reference coordinate of $q_{ALE}$ is ignored!}
    \rowTable{Coordinates}{$[x_{data0},\,q_{ALE}]\tp$}{}
    \rowTable{Coordinates\_t}{$[\dot q_{ALE}]\tp$}{}
    \rowTable{Force}{$\fv$}{see below}
    \finishTable
    -->

    #### Geometric relations

    The element sliding coordinate (in the local coordinates of the current sliding element) is computed from the ALE coordinate

    $$
    s_{el} = q_{ALE} + s_\mathrm{off} - d_{m1} = s_g - d_{m1}.
    $$

    For the description of the according quantities, see the description above. The distance $d_{m1}$ is obtained from the \texttt{slidingMarkerOffsets} list, using the current (local) index $x_{data0}$.
    The vector (=difference; error) between the marker $m0$ and the marker $m1$ (=$\rv_{ANCF}$) positions reads

    $$
    \LU{0}{\Delta\pv} = \LUR{0}{\rv}{ANCF} - \LU{0}{\pv}_{m0}
    $$

    Note that $\LU{0}{\pv}_{m0}$ represents the current position of the marker $m0$, which could represent the midpoint of a mass sliding along the beam.
    The position $\LUR{0}{\rv}{ANCF}$ is computed from the beam represented by marker $m1$, using the local beam coordinate $x=s_{el}$. 
    The marker and the according beam finite element changes during movement using the list \texttt{slidingMarkerNumbers} and the index is updated in the PostNewtonStep.
    The vector (=difference; error) between the marker $m0$ and the marker $m1$ velocities reads

    $$
    \LU{0}{\Delta\vv} = \LUR{0}{\vv}{ANCF} - \LU{0}{\vv}_{m0}
    $$

    <!--
    
    +++++++++++++++++++++++++++++++++++++++++++++
    -->

    #### Connector constraint equations

    The 2D sliding joint is implemented having 2 equations, using the Lagrange multipliers $\zv$. 
    The algebraic (index 3) equations read

    $$
    \LU{0}{\Delta\pv} = 0
    $$

    Note that the Lagrange multipliers $[\lambda_0,\,\lambda_1]\tp$are the global forces in the joint.
    In the index 2 case the algebraic equations read

    $$
    \LU{0}{\Delta\vv} = 0
    $$

    If \texttt{usePenalty = True}, the algebraic equations are changed to:

    $$
    \LU{0}{\Delta \pv} - \frac 1 k \zv = 0.
    $$

    <!--
    
    not realized yet, because ABRV:AE Jacobian becomes involved:
    If \texttt{usePenaltyFormulation = True}, the algebraic equations are changed to:
    \bea
      k_1 \LURU{0}{\rv}{ANCF}{\prime \mathrm{T}}   \LU{0}{\Delta\pv} - \lambda_0 &=& 0, \nonumber \\
      k_2 \LU{0}{\nv\tp}   \LU{0}{\Delta\pv}  - \lambda_1 &=& 0.
    \eea
    Note that in this case, the Lagrange multipliers $[\lambda_0,\,\lambda_1]\tp$are the local ($m1$) forces in the joint.
    -->

    \noindent If \texttt{activeConnector = False}, the algebraic equations are changed to:

    $$
    \begin{aligned}
    \lambda_0 &= 0,   \\
          \lambda_1 &= 0.
    \end{aligned}
    $$
   
    <!--
    
    +++++++++++++++++++++++++++++++++++++++++++++
    -->

    #### Post Newton Step

    After the Newton solver has converged, a PostNewtonStep is performed for the element, which
    updates the marker $m1$ index if necessary.

    $$
    \begin{aligned}
    s_{el} < 0 \quad \ra \quad x_{data0} \;-\!\!=1 \\
          s_{el} > L \quad \ra \quad x_{data0} \;+\!\!=1
    \end{aligned}
    $$

    Furthermore, it is checked, if $x_{data0}$ becomes smaller than zero, which raises a warning and keeps $x_{data0}=0$.
    The same results if $x_{data0}\ge sn$, then $x_{data0} = sn$.
    Finally, the data coordinate is updated in order to provide the starting value for the next step,

    $$
    x_{data1} \;+\!\!= s.
    $$

    %%RSTCOMPATIBLE
""",
    mainParentClass=MainParentClassMainObjectConnector,
    objectType=ObjectTypeJoint,
    outputVariables=[
        ItemOutputVariable(OVPosition, OVDPositionMarker0),
        ItemOutputVariable(OVVelocity, OVDVelocityMarker0),
        ItemOutputVariable(OVSlidingCoordinate, r"""$s_g = q_{ALE} + s_\mathrm{off}$current value of the global sliding ALE coordinate, including offset; note that reference coordinate of $q_{ALE}$ is ignored!"""),
        ItemOutputVariable(OVCoordinates, r"""$[x_{data0},\,q_{ALE}]\tp$provides two values: [0] = current sliding marker index, [1] = ALE sliding coordinate"""),
        ItemOutputVariable(OVCoordinates_t, r'$[\dot q_{ALE}]\tp$provides ALE sliding velocity'),
        ItemOutputVariable(OVForce, r'$\fv$joint force vector (3D)'),
        ],
    pythonShortName='ALEMovingJoint2D',
    visuParentClass=VisuParentClassVisualizationObject,
    members=[
        ItemParameter(type=TString, destination=DestMain, fromParent=True,
            pythonName='name',
            defaultValue=NoDefaultValue,
            description=r"constraints's unique name"),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='markerNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r"""$[m0,\,m1]\tp$marker m0: position-marker of mass point or rigid body; marker m1: updated marker to ANCF Cable2D element, where the sliding joint currently is attached to; must be initialized with an appropriate (global) marker number according to the starting position of the sliding object; this marker changes with time (PostNewtonStep)"""),
        ItemParameter(type=TArrayIndex(ItemMarker), destination=DestComp+DestParam,
            pythonName='slidingMarkerNumbers',
            defaultValue='ArrayIndex()',
            description=r"""$[m_{s0}, \ldots, m_{sn}]\tp$a list of sn (global) marker numbers which are are used to update marker1"""),
        ItemParameter(type=TVector, destination=DestComp+DestParam,
            pythonName='slidingMarkerOffsets',
            defaultValue='Vector()',
            description=r"""$[d_{s0}, \ldots, d_{sn}]$this list contains the offsets of every sliding object (given by slidingMarkerNumbers) w.r.t. to the initial position (0): marker0: offset=0, marker1: offset=Length(cable0), marker2: offset=Length(cable0)+Length(cable1), ..."""),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='slidingOffset',
            defaultValue=0.,
            description=r"""$s_\mathrm{off}$sliding offset [SI:m]: a scalar offset, which represents the (reference arc) length of all previous sliding cable elements"""),
        ItemParameter(type=TArrayIndex(ItemNode), destination=DestComp+DestParam,
            pythonName='nodeNumbers',
            defaultValue='ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })',
            description=r"""$[n_{GD}, n_{ALE}]$node number of NodeGenericData (GD) with one data coordinate and of NodeGenericODE2 (ALE) with one ABRV:ODE2 coordinate"""),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='usePenaltyFormulation',
            defaultValue=False,
            description=r'flag, which determines, if the connector is formulated with penalty, but still using algebraic equations (IsPenaltyConnector() still false)'),
        ItemParameter(type=TReal, destination=DestComp+DestParam,
            pythonName='penaltyStiffness',
            defaultValue=0.,
            description=r'$k$penalty stiffness [SI:N/m] used if usePenaltyFormulation=True'),
        ItemParameter(type=TBool, destination=DestComp+DestParam,
            pythonName='activeConnector',
            defaultValue=True,
            description=r'flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags=CFConst,
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetMarkerNumbers',
            cFlags='',
            implementation='return parameters.markerNumbers;'),
        ItemFunctionDef('GetNodeNumber',
            implementation="""CHECKandTHROW(localIndex <= 1, __EXUDYN_invalid_local_node1);
        return parameters.nodeNumbers[localIndex];"""),
        ItemFunctionDef('SetNodeNumber',
            implementation='parameters.nodeNumbers[localIndex]=nodeNumber;'),
        ItemFunctionDef('GetNumberOfNodes',
            implementation='return 2;'),
        ItemFunctionDef('GetDataVariablesSize',
            implementation='return 1;',
            description='data variables: [0] showing the current (local) index in slidingMarkerNumber list --> providing the cable element active in sliding; coordinate [1] stores the previous sliding coordinate'),
        ItemFunctionDef('CheckPreAssembleConsistency'),
        ItemFunctionDef('IsPenaltyConnector',
            implementation='return false;'),
        ItemFunctionDef('ComputeAlgebraicEquations',
            args='Vector& algebraicEquations, const MarkerDataStructure& markerData, Real t, Index itemIndex, bool velocityLevel = false'),
        ItemFunctionDef('ComputeJacobianAE'),
        ItemFunctionDef('GetAvailableJacobians',
            implementation='return (JacobianType::Type)(JacobianType::AE_ODE2 + JacobianType::AE_ODE2_function + JacobianType::AE_AE + JacobianType::AE_AE_function);'),
        ItemFunctionDef('HasDiscontinuousIteration',
            implementation='return true;'),
        ItemFunctionDef('PostNewtonStep'),
        ItemFunctionDef('PostDiscontinuousIterationStep'),
        ItemFunctionDef('GetOutputVariableConnector'),
        ItemRequestedTypes('Marker', [],
            description=r"""provide requested markerType for connector; for different markerTypes in marker0/1 => set to ::\_None"""),
        ItemRequestedTypes('Node', []),
        ItemFunction(type=TCObjectType, destination=DestComp, cFlags=CFConst,
            pythonName='GetType',
            implementation='return (CObjectType)((Index)CObjectType::Connector + (Index)CObjectType::Constraint);',
            description=r'return object type (for node treatment in computation)'),
        ItemFunctionDef('GetAlgebraicEquationsSize',
            implementation='return 2;',
            description='q0=forceX of sliding joint, q1=forceY of sliding joint'),
        ItemFunction(type='const char*', destination=DestMain, cFlags=CFConst,
            pythonName='GetTypeName',
            implementation='return "JointALEMoving2D";',
            description=r"Get type name of object (without keyword 'Object'...!); could also be realized via a string -> type conversion?"),
        ItemFunctionDef('IsActive',
            implementation='return parameters.activeConnector;'),
        ItemFunction(type=TReal, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeLocalSlidingCoordinate',
            description=r'compute the (local) sliding coordinate within the current cable element; this is calculated from (globalSlidingCoordinate - slidingMarkerOffset) of the cable'),
        ItemFunction(type=TReal, destination=DestComp, cFlags=CFConst, isVirtual=False,
            pythonName='ComputeLocalSlidingCoordinate_t',
            description=r'compute the (local=global) sliding velocity, which is equivalent to the ALE velocity!'),
        ItemFunctionDef('UpdateGraphics'),
        ItemFunctionDef('IsConnector',
            implementation='return true;'),
        ItemParameter(type=TBool, destination=DestVisu, fromParent=True,
            pythonName='show',
            defaultValue=True,
            description=r'set true, if item is shown in visualization and false if it is not shown'),
        ItemParameter(type=Tfloat, destination=DestVisu,
            pythonName='drawSize',
            defaultValue=-1.,
            description=r'drawing size = radius of revolute joint; size == -1.f means that default connector size is used'),
        ItemParameter(type=TFloat4, destination=DestVisu,
            pythonName='color',
            defaultValue=DVDefaultColor,
            description=r'RGBA connector color; if R==-1, use default color'),
        ],
    ))
