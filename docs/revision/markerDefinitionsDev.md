# Marker documentation (development document)

*Temporary, revision2026b step RG13.4.3 (#2721); what is common to every kind is in
[itemDefinitionsDev.md](itemDefinitionsDev.md). 18 markers in `definitions/itemDefsMarkers.py`.*

The maintainer, 2026-09-27: markers need **short equations**, and a **general section** that all
markers refer to.

## 1. What a reader needs to know about a marker

A reader rarely writes a marker: `CreateRevoluteJoint` adds two `MarkerBodyRigid`, `CreateForce`
adds `MarkerBodyRigid` or `MarkerBodyPosition` (depending on its arguments), `CreateMassPoint` and
`CreateRigidBody` add `MarkerBodyMass` for gravity. The marker is met when something does not fit:
*"requires a node with type Orientation"*, or a joint written by hand. The questions:

1. **What it provides**: position, orientation, velocity, angular velocity, a coordinate - and the
   **Jacobians** a connector needs to apply a force;
2. **where**: at which point (the local position on the body, the node, the mesh nodes of a
   superelement), and **in which frame** its quantities are;
3. **on what it can sit**: which nodes or bodies - and why a body marker on a body without
   `AngularVelocity_qt` fails;
4. **what can use it**: which connectors and loads - the other direction of the same rule;
5. **how a force on it reaches the coordinates**: $\Qm = \Jm\tp \fv$ for a position marker,
   $\Jm_{rot}\tp \tauv$ for an orientation - the one equation every marker is about.

## 2. What the pages say today

| | markers | |
|---|---|---|
| without any text beyond the class description | 10 of 18 | `MarkerBodyMass`, `MarkerBodyRigid`, the three node coordinate markers, the three cable and beam markers, `MarkerObjectODE2Coordinates`, `MarkerNodeRotationCoordinate` |
| with the quantities they provide, as equations or a table | 6 | `MarkerBodyPosition`, `MarkerNodePosition`, `MarkerNodeRigid`, the two relative-coordinate markers, `MarkerSuperElementPosition` |
| with output variables | 0 | markers have none; `SensorMarker` measures them, and its class description lists what (§4) |
| with a MiniExample | 1 | `MarkerSuperElementPosition` |

Textual findings:

- The class descriptions are good at **what a marker is for** (*"It can be used for connectors,
  joints or loads where position is required. If connectors also require orientation information,
  use a MarkerBodyRigid."*) - that sentence is on every position marker, in slightly different words;
  it belongs to the general section once, and the page to the generated "usable by" line.
- `MarkerBodyPosition` explains the position Jacobian with `ObjectRigidBody2D` as the example;
  `MarkerNodePosition` and `MarkerNodeRigid` repeat the same idea. The idea is general; the example
  is the body's page.
- `MarkerBodyRigid`, the most used marker (127 scripts), has no text beyond its class description.
- The generated type line lists internal flags: `JacobianDerivativeAvailable`,
  `JacobianDerivativeNonZero`, `HasPostNewton` (itemDefinitionsDev §3).
- `MarkerNodeCoordinates` and `MarkerObjectODE2Coordinates` say in capitals that their coordinates
  *INCLUDE* the reference values - unlike `MarkerNodeCoordinate`. A reader needs that; it is the kind
  of difference the general section's table (§4) makes visible at a glance.
- Two markers carry their quantity tables inside HTML comments (`MarkerSuperElementRigid`,
  `MarkerKinematicTreeRigid`) - text that is in the definition and on no page.

## 3. The ideal marker page

Short. After the common head (itemDefinitionsDev §4):

| section | contents | source |
|---|---|---|
| **Attached to** | node / body / superelement mesh nodes / two bodies; what it requires of it (node type, access functions) | generated from the declared types (needs `requestedNodeTypes`, itemDefinitionsDev §3) |
| **Usable by** | connectors and loads that request what it provides | generated |
| **Marker quantities** | a table: position, rotation matrix, velocity, angular velocity or the coordinate(s) - each as a formula of the item it sits on, with its frame | written - the short equations |
| **Jacobians** | position and/or rotation Jacobian, and $\Qm = \Jm\tp\fv$ for this marker | written, short; the general form is in the general section |
| **MiniExample** | a load or a connector on it | written |

For the coordinate markers the table is one line (*the coordinate $q_i$ of the node, without /
with its reference value*), and the Jacobian a unit vector.

## 4. The general marker section

Before the first marker, replacing today's paragraph of the index page:

- **What a marker is**: the interface between the item that has coordinates (node, body) and the item
  that acts on them (connector, load) - so that a joint does not need to know whether it is attached
  to a node or to a point of a body. The diagram of `introduction.md` §*Items*.
- **What a marker provides**: position, orientation, velocity, angular velocity, coordinates, and
  the Jacobians; which of them a connector or load requests (its *Requested marker type*).
- **How a force reaches the coordinates**: the virtual work, $\delta W = \delta\pv\tp\fv$, gives
  $\Qm = \Jm_{pos}\tp\fv$; the same with $\Jm_{rot}$ for a torque - once, with the frames.
- **The table of all markers**, generated: marker, attached to, provides (position / orientation /
  coordinate / mass), reference values included or not, used by. It answers the index page's *"see
  first the requested Marker type of the connector ... hereafter, chose a Marker"* without the reader
  doing it.
- **Markers in sensors**: `SensorMarker` measures what the marker provides, in the current
  configuration only.

## 5. Decided

- The document is agreed (maintainer, 2026-09-28).
- The table of all markers in the general section is **generated** from the declared types, like
  the Interface block of every page (RG13.5.0.3), which needs the declared node requirement - done
  in RG13.5.0.3 and tested in RG13.5.0.4.
