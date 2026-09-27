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
    `simulationSettings.solutionSettings.sensorsWritePeriod`, and

    - writes a line `time, value[0], value[1], ...` to the file `fileName`, if `writeToFile = True` and a
      file name is given; the directory is created if it does not exist, and a header and a footer
      describe the sensor (`sensorsWriteFileHeader`, `sensorsWriteFileFooter`);
    - stores the same rows in memory if `storeInternal = True`, which `mbs.GetSensorStoredData(sensor)`
      returns as an array, one row per time;
    - does neither if `solutionSettings.sensorsStoreAndWriteFiles = False`.

    `solutionSettings.sensorsAppendToFile` appends to an existing file, or to the stored data, so that
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
