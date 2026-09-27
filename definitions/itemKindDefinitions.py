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
    ))

definitions.append(ItemKindDefinition(
    kind='Objects (SuperElement)',
    overallDescription=r"""A SuperElement is a special Object which acts on a set of nodes. Essentially, SuperElements can be linked with special SuperElement markers. SuperElements may represent complex flexible bodies, based on finite element formulations.""",
    ))

definitions.append(ItemKindDefinition(
    kind='Objects (FiniteElement)',
    overallDescription=r"""A FiniteElement is a special Object and Body, which is used to define deformable bodies, such as beams or solid finite elements. FiniteElements are usually linked to two or more nodes.""",
    ))

definitions.append(ItemKindDefinition(
    kind='Objects (Joint)',
    overallDescription=r"""A Joint is a special Object, Connector and Constraint, which is attached to position or rigid body markers. The joint results in special algebraic equations and requires implicit time integration. Joints represent special constraints, as described in multibody system dynamics literature.""",
    ))

definitions.append(ItemKindDefinition(
    kind='Objects (Connector)',
    overallDescription=r"""A Connector is a special Object, which links two or more markers. A Connector which is not a Constraint, is a force element (e.g., spring-damper) or a penalty based joint.""",
    ))

definitions.append(ItemKindDefinition(
    kind='Objects (Constraint)',
    overallDescription=r"""A Constraint is a special Object and Connector, which links two or more markers. A Constraint leads to algebraic equations, which exactly fulfill special constraints on the kinematic behavior of the multibody syste, such as a constraint on a coordinate or a distance constraint.""",
    ))

definitions.append(ItemKindDefinition(
    kind='Objects (Object)',
    overallDescription=r"""A Object provides equations, using coordinates from Nodes. General objects lead to system equations, that do not represent physical Bodies or Connectors.""",
    ))

definitions.append(ItemKindDefinition(
    kind='Markers',
    overallDescription=r"""A Marker provides an interface BETWEEN a large variety of Nodes / Bodies / Objects AND Connectors / Loads. To understand which markers are needed, see first the requested `Marker` type of the connector, constraint or joint. Hereafter, chose a `Marker` -- attached to a node, body or object -- with the according properties. The `Marker` may provide more information (e.g., position and orientation) than needed.""",
    ))

definitions.append(ItemKindDefinition(
    kind='Loads',
    overallDescription=r"""A Load applies a (usually constant) force, torque, mass-proportional or generalized load onto Nodes or Objects via Markers. The requested `Marker` types need to be provided by the used Marker. The marker may provide more types than requested. For non-constant loads, use either a `load...UserFunction` or change the load in every step by means of a `preStepUserFunction` in the `MainSystem` (mbs).""",
    ))

definitions.append(ItemKindDefinition(
    kind='Sensors',
    overallDescription=r"""A Sensor is used to measure quantities during simulation. Sensors may be attached to Nodes, Objects, Markers or Loads. Sensor values may be directly read via mbs or can be continuously written to files or SensorRecorder during simulation. The exudyn.plot Python utility function PlotSensor(...) can be conveniently used to show Sensor values over time.""",
    ))
