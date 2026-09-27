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
