#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Item kind definitions
#
# Details:  What all items of one kind have in common - nodes, the kinds of objects, markers,
#           loads, sensors. Each entry is the page of the kind in the reference manual: its
#           overallDescription is the paragraph under the heading, its detailedDescription the
#           general section that every item of the kind refers to, so that the page of an item
#           says only what is its own (#2725).
#           This IS Python: import it and read "definitions", a list of dicts.
#
#           ORDER does not matter here: the pages of the kinds are ordered by the generator.
#
#           DESCRIPTIONS: read definitions/README.md, section "Writing a
#           description", before writing or changing one - what the text may
#           contain, and how it is checked. A heading of a detailedDescription here is written
#           with "##": the page of the kind has only its title above it.
#
# Contents: Nodes, Objects (Body, SuperElement, FiniteElement, Joint, Connector, Constraint,
#           Object), Markers, Loads, Sensors
#
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from definitionTypes import *

definitions = []

definitions.append(ItemKindDefinition(
    kind='Nodes',
    overallDescription=r"""Nodes provide coordinates for objects. Loads can be applied and Markers or Sensors can be attached to Nodes. The sorting of Nodes in the system (the order they are added to mbs) defines the order of system coordinates.""",
    drawing=r"""A node is drawn if `nodes.show` and its own `show` are True: as a sphere of diameter
    `nodes.defaultSize` with `nodes.tiling` segments, or as a point with `nodes.drawNodesAsPoint`, in
    `nodes.defaultColor` or its own `color`; a size of -1 takes a fraction of `openGL.advanced.initialMaxSceneSize`. In a
    contour plot (`contour.outputVariable`, `contour.outputVariableComponent`) with `contour.nodesColored`, it takes the
    color of its value. `nodes.showNumbers` writes its number. A node without a position - `Node1D`, the generic nodes -
    draws nothing.""",
    drawingSettings=['nodes.show', 'nodes.defaultSize', 'nodes.defaultColor', 'nodes.drawNodesAsPoint', 'nodes.tiling', 'nodes.showNumbers', 'contour.outputVariable', 'contour.outputVariableComponent', 'contour.nodesColored', 'openGL.advanced.initialMaxSceneSize'],
    detailedDescription=r"""
    ## What a node is

    A node provides coordinates and nothing else: no mass, no stiffness, no equations. The object that
    uses the node - a mass point, a rigid body, a finite element - provides the equations for its
    coordinates, and a node that no object uses leaves the system without equations for them. Each
    node page says, under **Action on the equations of motion**, which equations its coordinates get.

    ## Reference, initial and current coordinates

    Every node has `referenceCoordinates`, which define the reference configuration, and
    `initialCoordinates` and `initialVelocities`, which are relative to them: a `NodePoint` starts at
    `referenceCoordinates + initialCoordinates`. The coordinates of the system are the **current**
    coordinates, without the reference values - displacements, or changes of rotation parameters; the
    output variable `Coordinates` returns them and `CoordinatesTotal` adds the reference values. See
    [](#sec-overview-items-coordinates) and [](#sec-referenceandcurrentcoordinates).

    ## The kinds of coordinates

    | kind | what the solver does with it | nodes |
    |---|---|---|
    | ABRV:ODE2 | second order differential equations: positions, rotations, slopes | the point, rigid body and slope nodes, `Node1D`, `NodeGenericODE2` |
    | ABRV:ODE1 | first order differential equations: states | `NodeGenericODE1` |
    | ABRV:AE | algebraic variables | `NodeGenericAE`; `NodeRigidBodyEP` adds one for its constraint |
    | data | no unknowns: states an object updates between steps, such as contact or friction | `NodeGenericData` |

    `NodePointGround` has no coordinates at all.

    ## Frames

    The coordinates of a node are global, unless the object that uses the node reads them otherwise:
    `ObjectFFRF` reads the points of its mesh in the frame of its rigid body node. Every node page says
    under **Frame and interpretation** which objects deviate.

    ## Rotation parametrizations

    A rigid body is a rigid body node and an `ObjectRigidBody`, and the node decides how the rotation is
    parametrized; `CreateRigidBody(..., nodeType=...)` chooses it.

    | node | rotation coordinates | constraint | singularity | suited for |
    |---|---|---|---|---|
    | `NodeRigidBodyEP` | 4 Euler parameters | one, $\ttheta\tp\ttheta = 1$, added by the node | none | general 3D motion, implicit integration |
    | `NodeRigidBodyRxyz` | 3 Tait-Bryan angles | none | at $\theta_1 = \pm\pi/2$ | small or planar-like rotations, readable angles |
    | `NodeRigidBodyRotVecLG` | rotation vector | none | none in the Lie group update | arbitrary rotations, explicit and implicit Lie group integration |
    | `NodeRigidBody2D` | 1 angle about $z$ | none | none | planar motion |

    In all of them, the torque equations are the torques projected with the transposed velocity
    transformation $\Gm\tp$ of the node, $\tomega = \Gm \dot\ttheta$.

    ## Markers on nodes

    A node marker needs the node to provide what it measures: `MarkerNodePosition` a position,
    `MarkerNodeRigid` a position and an orientation, `MarkerNodeRotationCoordinate` an orientation. The
    **Interface** of each node lists the markers and the objects that fit. `MarkerNodeCoordinate` and
    `MarkerNodeCoordinates` act on single coordinates and fit every node with ABRV:ODE2 coordinates.
    """,
    ))

definitions.append(ItemKindDefinition(
    kind='Objects (Body)',
    overallDescription=r"""A Body is a special Object, which has physical properties such as mass. A localPosition can be measured w.r.t. the reference point of the body""",
    drawing=r"""A body is drawn if `bodies.show` and its own `show` are True: its `graphicsData`, in its frame,
    with the colors given there; `bodies.defaultColor` where a color is -1. In a contour plot (`contour.outputVariable`,
    `contour.outputVariableComponent`) with `contour.rigidBodiesColored`, the body takes the color of its value.
    `bodies.showNumbers` writes its number.""",
    drawingSettings=['bodies.show', 'bodies.showNumbers', 'bodies.defaultColor', 'contour.outputVariable', 'contour.outputVariableComponent', 'contour.rigidBodiesColored'],
    detailedDescription=r"""
    ## What a body is

    A body is an object with mass: it owns one or more nodes, whose coordinates it gives its equations of
    motion - a mass matrix, the forces that depend on its motion, and the forces that act on it. The
    bodies of this group have one node or none (the ground); the flexible bodies and the super elements
    have their own groups.

    ## Frames and the reference point

    A body has a **body frame** $b$ and a **reference point**, which is the position of its node. A
    `localPosition` $\pLocB$ - of a marker, a sensor, a graphics - is given in the body frame and
    measured from the reference point; the rotation matrix $\LU{0b}{\Rot}$ takes it into the global
    frame. The reference point is the center of mass only where the page of the body says so: an
    `ObjectRigidBody` has its center of mass at `centerOfMass` from it.

    ## Marker interfaces

    A marker on a body - `MarkerBodyPosition`, `MarkerBodyRigid`, `MarkerBodyMass` - gets from the body
    what the body's **output variables** define at the marker's local position: the position
    $\LU{0}{\pv}(\pLocB)$, the velocity $\LU{0}{\vv}(\pLocB)$, the rotation matrix and the angular
    velocity. A force or torque acts back through the **Jacobians** of the body, the derivatives of
    that velocity and angular velocity with respect to the velocity coordinates $\dot\qv$ of the body,

    $$
    \LU{0}{\Jm_{pos}}(\pLocB) = \frac{\partial \LU{0}{\vv}(\pLocB)}{\partial \dot\qv} , \quad
    \LU{0}{\Jm_{rot}} = \frac{\partial \LU{0}{\tomega}}{\partial \dot\qv} ,
    $$

    which give the generalized forces $\Qm = \LU{0}{\Jm_{pos}}\tp \LU{0}{\fv} + \LU{0}{\Jm_{rot}}\tp
    \LU{0}{\ttau}$. As the velocities are linear in $\dot\qv$, these are also the derivatives of the
    position and of the rotation parameters with respect to the coordinates. A body provides them as
    **access functions**, and a marker needs the ones for its types:

    | access function | what it is | needed by |
    |---|---|---|
    | `TranslationalVelocity_qt` | $\LU{0}{\Jm_{pos}}(\pLocB)$ | a marker with `Position` |
    | `AngularVelocity_qt` | $\LU{0}{\Jm_{rot}}$ | a marker with `Orientation` |
    | `DisplacementMassIntegral_q` | $\int_V \rho\, \LU{0}{\Jm_{pos}}\, dV$ | `MarkerBodyMass`, for a load per mass |
    | `JacobianTtimesVector_q` | $\partial (\Jm\tp \fv) / \partial \qv$ for a constant $\fv$ | the Jacobians of connectors in implicit solvers |

    The page of each body gives only its own Jacobians, under **Marker interfaces**; its **Interface**
    lists the body markers its access functions allow.

    ## Creating bodies

    `mbs.CreateGround`, `mbs.CreateMassPoint` and `mbs.CreateRigidBody` add a body with its node - and
    with a `gravity` argument a `MarkerBodyMass` with a `LoadMassProportional`.
    """,
    ))

definitions.append(ItemKindDefinition(
    kind='Objects (SuperElement)',
    overallDescription=r"""A SuperElement is a special Object which acts on a set of nodes. Essentially, SuperElements can be linked with special SuperElement markers. SuperElements may represent complex flexible bodies, based on finite element formulations.""",
    drawing=r"""A super element is drawn if `bodies.show` and its own `show` are True: its triangle mesh
    on its nodes, the deformation scaled by `bodies.deformationScaleFactor`, in its `color` or `bodies.defaultColor`;
    in a contour plot (`contour.outputVariable`, `contour.outputVariableComponent`) colored by the values at the nodes.
    `bodies.showNumbers` writes its number.""",
    drawingSettings=['bodies.show', 'bodies.showNumbers', 'bodies.defaultColor', 'bodies.deformationScaleFactor', 'contour.outputVariable', 'contour.outputVariableComponent'],
    detailedDescription=r"""
    ## What a super element is

    A super element is a body that stands for many points at once - the nodes of a finite element mesh,
    the links of a robot - and computes their motion from its own coordinates.

    | object | coordinates | what it is for |
    |---|---|---|
    | `ObjectGenericODE2` | the coordinates of its nodes | a linear or nonlinear system of second order equations: mass, damping and stiffness matrices, user functions |
    | `ObjectFFRF` | a rigid body node and the nodes of the mesh | a flexible body in the floating frame of reference formulation, with all mesh coordinates |
    | `ObjectFFRFreducedOrder` | a rigid body node and a `NodeGenericODE2` of modal coordinates | the same, reduced to a few modes: the flexible body of choice, imported with `FEMinterface` |
    | `ObjectKinematicTree` | a `NodeGenericODE2` of joint coordinates | an open tree of rigid links in minimal coordinates, e.g. a serial robot |

    ## Mesh nodes

    The points a super element stands for are its **mesh nodes**, numbered from 0 within the object. A
    mesh node may be a node of the system (`ObjectGenericODE2`, `ObjectFFRF`) or exist only in the
    object and be computed from its coordinates (`ObjectFFRFreducedOrder`: from the modes). Each mesh
    node has output variables of its own - its position, displacement, velocity, and for the FFRF
    objects stresses and strains where the modes carry them - which `SensorSuperElement` measures and
    the contour plot draws.

    ## Marker interfaces

    Forces, connectors and joints act on a super element through its **own markers**:
    `MarkerSuperElementPosition` averages the positions of a set of mesh nodes with weights, and
    `MarkerSuperElementRigid` also a rotation, from mesh nodes that can represent a rigid body motion;
    the kinematic tree has `MarkerKinematicTreeRigid` at a link. Their Jacobians are those of the mesh
    nodes, weighted. The general body markers act on the FFRF objects' **reference frame** only, and not
    at all on `ObjectGenericODE2` and `ObjectKinematicTree`, where `Assemble()` refuses them; the page of
    each object says which.
    """,
    ))

definitions.append(ItemKindDefinition(
    kind='Objects (FiniteElement)',
    overallDescription=r"""A FiniteElement is a special Object and Body, which is used to define deformable bodies, such as beams or solid finite elements. FiniteElements are usually linked to two or more nodes.""",
    drawing=r"""A finite element is drawn if `bodies.show` and its own `show` are True, in its `color`
    or `bodies.defaultColor`; in a contour plot (`contour.outputVariable`, `contour.outputVariableComponent`) colored by
    the values along it. `bodies.showNumbers` writes its number.""",
    drawingSettings=['bodies.show', 'bodies.showNumbers', 'bodies.defaultColor', 'contour.outputVariable', 'contour.outputVariableComponent'],
    detailedDescription=r"""
    ## What the finite elements have in common

    A finite element is a body with two or more nodes, whose coordinates it interpolates to describe a
    deformable body - a cable, a beam, a plate. It is a body in every other respect: the general
    section of the bodies describes its marker interfaces and Jacobians, and each element page gives
    only its own.

    | element | nodes | theory | deformation |
    |---|---|---|---|
    | `ObjectANCFCable2D` | 2 `NodePoint2DSlope1` | ANCF, Bernoulli-Euler | axial, bending |
    | `ObjectALEANCFCable2D` | 2 `NodePoint2DSlope1` + `NodeGenericODE2` | the same, with axially moving mass | axial, bending |
    | `ObjectANCFCable` | 2 `NodePointSlope1` | ANCF, Bernoulli-Euler, 3D | axial, bending; no torsion |
    | `ObjectANCFBeam` | 2 `NodePointSlope23` | ANCF, shear deformable (under development) | axial, shear, torsion, bending, cross section |
    | `ObjectBeamGeometricallyExact2D` | 2 or 3 `NodeRigidBody2D` | geometrically exact (Simo-Reissner) | axial, shear, bending |
    | `ObjectBeamGeometricallyExact` | 2 rigid body nodes | geometrically exact, 3D (under development) | |
    | `ObjectANCFThinPlate` | 4 `NodePointSlope12` | ANCF, Kirchhoff plate (under construction) | in-plane, bending |

    ## The page of an element

    Each element page follows the same order: **nodes and coordinates**, **kinematics and
    interpolation** - the shape functions and the position of a point -, **strains** and **elastic
    forces** with their integration rule, the **mass matrix**, **marker interfaces**, and
    **limitations**.

    ## Reference configuration and the local coordinates

    The reference configuration is given by the reference coordinates of the nodes, and the element
    coordinates are the sum of reference and current coordinates. Whether a curved or stretched
    reference is stress-free depends on the element: the ANCF cables subtract the reference strains
    with `strainIsRelativeToReference`, the geometrically exact beam has a reference curvature. The
    local axial coordinate runs over $[0,\,L]$ for the ANCF cables and over $[-L/2,\,L/2]$ for the other
    beams; `length` is the length of the element in its reference configuration.

    ## Meshes

    The elements share their nodes, and a beam is a chain of elements. `exudyn.beams` creates such
    chains, e.g. `GenerateStraightLineANCFCable2D` and `GenerateStraightLineANCFCable`, and the markers
    on a node of the chain connect it to the rest of the model.
    """,
    ))

definitions.append(ItemKindDefinition(
    kind='Objects (Joint)',
    overallDescription=r"""A Joint is a special Object, Connector and Constraint, which is attached to position or rigid body markers. The joint results in special algebraic equations and requires implicit time integration. Joints represent special constraints, as described in multibody system dynamics literature.""",
    drawing=r"""A joint is drawn as a connector: if `connectors.show` and its own `show` are True, at the
    positions of its markers, in its `color` or `connectors.defaultColor`, a `drawSize` of -1 takes
    `connectors.defaultSize`, and `connectors.showNumbers` writes its number. The 3D joints draw their axes as cylinders
    (`general.cylinderTiling`) and, with `connectors.showJointAxes`, the frames of their markers
    (`connectors.jointAxesLength`, `connectors.jointAxesRadius`, `general.axesTiling`); the 2D joints draw circles.""",
    drawingSettings=['connectors.show', 'connectors.showNumbers', 'connectors.defaultColor', 'connectors.defaultSize'],
    detailedDescription=r"""
    ## What a joint is

    A joint is a constraint between two rigid markers - `MarkerBodyRigid`, `MarkerNodeRigid`, a super
    element or kinematic tree marker - which fixes some of the relative motions of the two frames and
    leaves the others free: the revolute joint the rotation about one axis, the prismatic joint the
    translation along one axis. Everything the page of the constraints says - Lagrange multipliers,
    index 3 and index 2, `activeConnector`, redundant constraints - holds for the joints.

    ## Joint frames

    The free axes are those of a **joint frame** in each marker: `rotationMarker0` and
    `rotationMarker1` rotate the marker frames into the joint frames, and `ObjectJointRevoluteZ`, for
    example, turns about the local $z$-axis of the joint frame. `mbs.CreateRevoluteJoint`,
    `CreatePrismaticJoint`, `CreateSphericalJoint` and `CreateGenericJoint` take a global position and
    axis and compute the markers and the joint frames.

    ## Reaction forces

    The multipliers of a joint are its reaction forces and torques; the output variables give them -
    `ForceLocal` and `TorqueLocal` in the joint frame $J0$ of marker 0 for most joints, `Force` in the
    global frame for the spherical joint -, as the page of each joint lists them.
    """,
    ))

definitions.append(ItemKindDefinition(
    kind='Objects (Connector)',
    overallDescription=r"""A Connector is a special Object, which links two or more markers. A Connector which is not a Constraint, is a force element (e.g., spring-damper) or a penalty based joint.""",
    drawing=r"""A connector is drawn if `connectors.show` and its own `show` are True, between the positions
    of its markers, in its `color` or `connectors.defaultColor`; a `drawSize` of -1 takes `connectors.defaultSize`.
    `connectors.showNumbers` writes its number. Circles and cylinders take the tilings of `general`
    (`general.circleTiling`, `general.cylinderTiling`, `general.sphereTiling`, `general.axesTiling` for frames and
    arrows), items that are large compared to the others 4 times as many; space curves such as the windings of a spring
    take `connectors.curveTiling`. `connectors.drawSimplified` draws springs and the distance connector as lines.""",
    drawingSettings=['connectors.show', 'connectors.showNumbers', 'connectors.defaultColor', 'connectors.defaultSize'],
    detailedDescription=r"""
    ## The principle every connector follows

    A connector has no coordinates of its own. Its markers give it positions, orientations, velocities,
    coordinates - whatever its types request -, it computes a force from them, and the force goes back
    to the coordinates through the Jacobians of the markers: for a force $\LU{0}{\fv}$ at marker 1 and
    $-\LU{0}{\fv}$ at marker 0, the virtual work

    $$
    \delta W = \left(\delta\LU{0}{\pv}_{m1} - \delta\LU{0}{\pv}_{m0}\right)\tp \LU{0}{\fv}
    $$

    gives the generalized forces $\LU{0}{\Jm_{pos,m1}}\tp\LU{0}{\fv}$ and $-\LU{0}{\Jm_{pos,m0}}\tp\LU{0}{\fv}$
    on the coordinates of the two bodies or nodes, and a torque the same with the rotation Jacobians.
    This is the same for all connectors, so each page gives only its **force law**: how the force
    follows from the marker quantities. The connector adds its forces to the left-hand side of the
    equations of motion, and so to the Jacobian of the solver.

    ## Headings of a connector page

    **Definition of quantities** - the marker quantities and the intermediate variables -,
    **geometric relations** - distance, relative rotation, contact geometry -, and **connector forces**
    - the force law, and a user function where there is one.

    ## `activeConnector`

    A connector with `activeConnector = False` computes its kinematic quantities but no force; it can
    be switched on and off during a simulation, e.g. in a `preStepUserFunction`.

    ## Output variables

    `Force` is the force vector, usually on marker 1 and in the global frame; `ForceLocal` a force in
    the frame of the connector, or a scalar force; `Distance`, `Displacement` and `Velocity` the
    kinematic quantities the force law uses. The page of each connector says which it has and in which
    frame.

    The joints and the connectors of two points name the same quantity the same way (#2870): `Position`
    and `Velocity` are those of marker 0; `Displacement` is the global vector from marker 0 to marker 1;
    `DisplacementLocal` is that vector in the joint frame $J0$ - for a joint its drift -, and it exists
    only where the markers have a frame; `HomogeneousTransformation` is the joint frame $J0$ of the
    classical joints, and `HomogeneousTransformationLocal` the frame $J1$ relative to $J0$, for the joints
    and connectors on rigid markers. Both give an `exu.HT` and are stored by a sensor as 16 values row by
    row.

    ## Contact connectors

    The contact connectors - `ObjectContact...`, `ObjectConnectorRollingDiscPenalty` - are penalty
    formulations with a discontinuous contact state. They need a `NodeGenericData`, whose data
    coordinates hold the state of the last post Newton step - gap, friction regime, impact velocity -,
    and the solver repeats a step when the state changes (active set strategy). `mbs.CreateSphereSphereContact`
    and its relatives add the node, the markers and the connector.
    """,
    ))

definitions.append(ItemKindDefinition(
    kind='Objects (Constraint)',
    overallDescription=r"""A Constraint is a special Object and Connector, which links two or more markers. A Constraint leads to algebraic equations, which exactly fulfill special constraints on the kinematic behavior of the multibody syste, such as a constraint on a coordinate or a distance constraint.""",
    drawing=r"""A constraint is drawn as a connector: if `connectors.show` and its own `show` are True,
    in its `color` or `connectors.defaultColor`, a `drawSize` of -1 takes `connectors.defaultSize`, and
    `connectors.showNumbers` writes its number.""",
    drawingSettings=['connectors.show', 'connectors.showNumbers', 'connectors.defaultColor', 'connectors.defaultSize'],
    detailedDescription=r"""
    ## What a constraint is

    A constraint prescribes a relation between the coordinates of the bodies or nodes its markers sit
    on - two points coincide, a distance is fixed, a coordinate follows another - as **algebraic
    equations** $\gv(\qv, t) = \Null$. Each equation gets a **Lagrange multiplier** $\lambda$, an
    algebraic unknown of the system, and acts on the equations of motion with the transposed Jacobian of
    the constraint, $\left(\partial \gv / \partial \qv\right)\tp \tlambda$: the multipliers are the
    reaction forces and torques of the constraint, in the directions the page of each constraint says.
    This page holds for the joints as well.

    ## Index 3 and index 2

    A constraint is written on the **position level** (index 3), $\gv(\qv,t) = \Null$, or on the
    **velocity level** (index 2), $\dot\gv = \Null$; which one the solver uses is
    `timeIntegration.generalizedAlpha.useIndex2Constraints` and the like. On the velocity level the
    position may drift over long simulations, which the output variables of some joints show.
    Constraints need an **implicit** time integration (generalized-alpha, trapezoidal) or the static
    solver; the explicit integrators do not solve algebraic equations - they can only eliminate
    `ObjectConnectorCoordinate` constraints to the ground, such as fixed nodes (`timeIntegration.explicit.eliminateConstraints`).

    ## `activeConnector`

    With `activeConnector = False` a constraint replaces its equations by $\tlambda = \Null$: it
    remains in the system with its multipliers, which are zero, and can be switched on again.

    ## Redundant constraints

    Constraints that fix the same motion twice - two revolute joints on one axis, a closed loop of
    planar joints in 3D - make the Jacobian of the constraints singular, and the solver fails.
    `mbs.ComputeSystemDegreeOfFreedom()` counts the redundant constraints; the `EigenDense` linear solver
    with `linearSolver.ignoreSingularJacobian` can handle some of them.
    """,
    ))

definitions.append(ItemKindDefinition(
    kind='Objects (Object)',
    overallDescription=r"""A Object provides equations, using coordinates from Nodes. General objects lead to system equations, that do not represent physical Bodies or Connectors.""",
    drawing=r"""The objects of this kind - `ObjectGenericODE1` - have nothing to draw.""",
    drawingSettings=[],
    ))

definitions.append(ItemKindDefinition(
    kind='Markers',
    overallDescription=r"""A Marker provides an interface BETWEEN a large variety of Nodes / Bodies / Objects AND Connectors / Loads. To understand which markers are needed, see first the requested `Marker` type of the connector, constraint or joint. Hereafter, chose a `Marker` -- attached to a node, body or object -- with the according properties. The `Marker` may provide more information (e.g., position and orientation) than needed.""",
    drawing=r"""A marker with a position is drawn if `markers.show` and its own `show` are True: as a symbol of size
    `markers.defaultSize` - a fraction of `openGL.advanced.initialMaxSceneSize` if -1 -, three crossed lines with
    `markers.drawSimplified`, a cube otherwise, in `markers.defaultColor`; with `markers.showBasis` the frame of a marker
    that has an orientation, of length `markers.basisSize` (`general.axesTiling`). `markers.showNumbers` writes its
    number. A marker on coordinates has no position and draws nothing.""",
    drawingSettings=['markers.show', 'markers.defaultSize', 'markers.defaultColor', 'markers.drawSimplified', 'markers.showNumbers', 'markers.showBasis', 'markers.basisSize', 'general.axesTiling', 'openGL.advanced.initialMaxSceneSize'],
    detailedDescription=r"""
    ## What a marker is

    A marker is the interface between the items that have coordinates - nodes and bodies - and the items
    that act on them - connectors, constraints, joints and loads. A joint does not need to know whether
    it is attached to a node or to a point of a body: it asks its markers for what it needs. The
    **Interface** of every connector and load names the marker types it requests, and the Interface of
    every marker the items that can use it; the table below lists all markers.

    ## What a marker provides

    A marker computes, in the current configuration, the **marker quantities** its types promise:

    | type | quantities |
    |---|---|
    | `Position` | the global position $\LU{0}{\pv}_m$ and velocity $\LU{0}{\vv}_m$ |
    | `Orientation` | the rotation matrix $\LU{0m}{\Rot}$ and the local angular velocity $\LU{m}{\tomega}$ |
    | `Coordinate`, `Coordinates` | one or several coordinates and their time derivatives |
    | `BodyMass` | only a Jacobian, for a load proportional to the mass |

    and the **Jacobians** through which a force acts: the position Jacobian
    $\LU{0}{\Jm_{pos}} = \partial \LU{0}{\vv}_m / \partial \dot\qv$, the rotation Jacobian
    $\LU{0}{\Jm_{rot}} = \partial \LU{0}{\tomega}_m / \partial \dot\qv$, or for a coordinate a row of the
    unit matrix. A marker on a **body** takes position, orientation, velocity and angular velocity as the
    output variables of the body define them, at its local position, and the Jacobians from the body; a
    marker on a **node** takes them from the node.

    ## How a force reaches the coordinates

    The virtual work of a force $\LU{0}{\fv}$ at the marker, $\delta W = \delta \LU{0}{\pv}_m\tp
    \LU{0}{\fv}$, gives the generalized forces

    $$
    \Qm = \LU{0}{\Jm_{pos}}\tp\, \LU{0}{\fv} ,
    $$

    and a torque $\LU{0}{\ttau}$ gives $\Qm = \LU{0}{\Jm_{rot}}\tp\, \LU{0}{\ttau}$; a force $f$ on a
    coordinate marker gives $\Qm = \Jm\tp f$. A marker on the ground - a `NodePointGround`, an
    `ObjectGround` - has an empty Jacobian, and nothing acts there.

    ## With or without reference values

    `MarkerNodeCoordinate` and `MarkerNodeODE1Coordinate` give the **current** coordinate, without its
    reference value; `MarkerNodeCoordinates`, `MarkerObjectODE2Coordinates` and the cable and beam
    markers give the coordinates **including** the reference values. A coordinate constraint between two
    markers therefore means something different with the one and with the other.

    ## Markers in sensors, and markers that are created

    `SensorMarker` measures the marker quantities, in the current configuration. Most markers of a model
    are added by the Create functions: `CreateRevoluteJoint` adds two `MarkerBodyRigid`, `CreateForce` a
    `MarkerBodyPosition` or `MarkerBodyRigid`, `CreateRigidBody` with gravity a `MarkerBodyMass`.
    """,
    ))

definitions.append(ItemKindDefinition(
    kind='Loads',
    overallDescription=r"""A Load applies a (usually constant) force, torque, mass-proportional or generalized load onto Nodes or Objects via Markers. The requested `Marker` types need to be provided by the used Marker. The marker may provide more types than requested. For non-constant loads, use either a `load...UserFunction` or change the load in every step by means of a `preStepUserFunction` in the `MainSystem` (mbs).""",
    drawing=r"""A load on a marker with a position is drawn if `loads.show` and its own `show` are True: an arrow at the
    marker, of length `loads.defaultSize` - a fraction of `openGL.advanced.initialMaxSceneSize` if -1 -, or, with
    `loads.fixedLoadSize` False, of the length of the load times `loads.loadSizeFactor`; lines with
    `loads.drawSimplified`, otherwise a 3D arrow of radius `loads.defaultRadius` (`general.axesTiling`), in
    `loads.defaultColor`. With `loads.drawWithUserFunction`, the value of the user function is drawn - of a symbolic one
    always, of a Python one only with `general.useMultiThreadedRendering` False, as the render thread cannot call Python.
    `loads.showNumbers` writes its number. A load on a coordinate draws nothing.""",
    drawingSettings=['loads.show', 'loads.defaultSize', 'loads.defaultRadius', 'loads.defaultColor', 'loads.drawSimplified', 'loads.fixedLoadSize', 'loads.loadSizeFactor', 'loads.drawWithUserFunction', 'loads.showNumbers', 'general.axesTiling', 'general.useMultiThreadedRendering', 'openGL.advanced.initialMaxSceneSize'],
    detailedDescription=r"""
    ## How a load acts

    A load acts through a marker: the marker locates it - a point of a body, a node, a coordinate - and
    provides the Jacobian that takes the load to the coordinates of the body or node. For a force
    $\LU{0}{\fv}$ at a position marker this is the virtual work $\delta W = \delta \LU{0}{\pv}\tp
    \LU{0}{\fv}$, which gives the generalized forces

    $$
    \Qm = \LU{0}{\Jm_{pos}}\tp\, \LU{0}{\fv}, \quad \LU{0}{\Jm_{pos}} = \frac{\partial \LU{0}{\pv}}{\partial \qv} ;
    $$

    a torque uses the rotation Jacobian $\LU{0}{\Jm_{rot}} = \partial \LU{0}{\tomega} / \partial
    \dot\qv$ the same way. The Jacobians are global, so a load given in a local frame is transformed
    into the global frame first. Each load page gives its formula.

    ## Global and body-fixed loads

    A force or a torque is given in the global frame, or with `bodyFixed = True` in the frame of its
    marker, where it turns with the body or node - a follower load. A body-fixed load needs a marker
    with an orientation: `MarkerBodyRigid` on a body, `MarkerNodeRigid` on a node; a point node has no
    orientation and takes only global loads.

    ## Loads that change in time

    A load is constant, unless

    - a user function gives it: `loadVectorUserFunction(mbs, t, loadVector)` or
      `loadUserFunction(mbs, t, load)`, called at every evaluation with the current time, replaces the
      value of the load;
    - a `preStepUserFunction` of `mbs` changes it before every step, with `mbs.SetLoadParameter`.

    ## Loads in static computations

    The static solver increases the loads over its load steps (`staticSolver.numberOfLoadSteps`):
    every load is multiplied by the **load factor** of the current step, which reaches one in the last
    step. A load with a user function is **not** multiplied: its value is used as the function returns
    it, and the function gets the quasi-time of the load step (`staticSolver.loadStepDuration`), which
    it has to use itself to increase the load.

    ## Creating loads

    `mbs.CreateForce` and `mbs.CreateTorque` add the marker and the load in one call - for a force a
    `MarkerBodyPosition`, or a `MarkerBodyRigid` if it is body-fixed; for a torque a `MarkerBodyRigid` -
    unless a marker is given instead of a body; and the
    `gravity` argument of `CreateMassPoint` and `CreateRigidBody` adds a `MarkerBodyMass` with a
    `LoadMassProportional`.
    """,
    ))

definitions.append(ItemKindDefinition(
    kind='Sensors',
    overallDescription=r"""A Sensor is used to measure quantities during simulation. Sensors may be attached to Nodes, Objects, Markers or Loads. Sensor values may be directly read via mbs or can be continuously written to files or SensorRecorder during simulation. The exudyn.plot Python utility function PlotSensor(...) can be conveniently used to show Sensor values over time.""",
    drawing=r"""A sensor is drawn if `sensors.show` and its own `show` are True, at the position it measures: a symbol
    of size `sensors.defaultSize` - a fraction of `openGL.advanced.initialMaxSceneSize` if -1 -, simple with
    `sensors.drawSimplified`, in `sensors.defaultColor`. `sensors.showNumbers` writes its number. A sensor without a
    position draws nothing; traces of sensors are drawn with `sensors.traces`.""",
    drawingSettings=['sensors.show', 'sensors.defaultSize', 'sensors.defaultColor', 'sensors.drawSimplified', 'sensors.showNumbers', 'openGL.advanced.initialMaxSceneSize'],
    detailedDescription=r"""
    ## What a sensor is

    A sensor reads a value of the model - an output variable of a node, an object, a body at a point, a
    marker, or the value of a load - and does not act on the system. Every sensor page says only what it
    is attached to and what it measures; what follows is the same for all of them.

    ## What a sensor measures

    `outputVariableType` selects the quantity, an `exu.OutputVariableType`, and the item the sensor is
    attached to must provide it: the page of the item lists its output variables with their symbols and
    frames. Asking for one the item does not provide is an error of `mbs.Assemble()`.

    ## When and where the values go

    During a simulation, the solver evaluates every sensor at the times given by
    `simulationSettings.solution.sensors.writePeriod`, and

    - writes a line `time, value[0], value[1], ...` to the file `fileName`, if `writeToFile = True` and a
      file name is given; the directory is created if it does not exist, and a header and a footer
      describe the sensor (`sensorsWriteFileHeader`, `sensorsWriteFileFooter`);
    - stores the same rows in memory if `storeInternal = True`, which `mbs.GetSensorStoredData(sensor)`
      returns as an array, one row per time;
    - does neither if `solution.sensors.active = False`.

    `solution.sensors.append` appends to an existing file, or to the stored data, so that
    several simulations continue one record.

    ## Reading a value at any time

    `mbs.GetSensorValues(sensor, configuration)` returns the value now, in the current configuration by
    default, without storing it.

    ## Plotting

    `mbs.PlotSensor(sensor, ...)` plots the stored or written values over time, several sensors and
    components at once; the results monitor shows sensor files while a simulation runs.

    ## Values computed from other sensors

    `SensorUserFunction` computes its value with a Python function from the values of other sensors,
    e.g. to transform them into another frame or to combine them; it is stored and written like any
    other sensor.

    ## Drawing

    Sensors are drawn as small symbols; with `visualizationSettings.sensors.traces` the positions, and
    vectors or frames, of position sensors are drawn along their history.
    """,
    ))
