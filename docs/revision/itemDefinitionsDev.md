# Item documentation: what is common to every kind (development document)

*Temporary, revision2026b step RG13.4 (#2721). One of six documents - this one and
[node](nodeDefinitionsDev.md), [object](objectDefinitionsDev.md), [marker](markerDefinitionsDev.md),
[load](loadDefinitionsDev.md), [sensor](sensorDefinitionsDev.md) `DefinitionsDev.md`. They collect,
per kind of item, what its reference page must contain, measured against what the pages contain
today. When the group is done they are folded into one section of the developer documentation - what
documentation an item needs, what it contains and how it is structured - and removed.*

What is common to every kind is written here once; the documents per kind refer to it.

## 1. How a reader meets an item

Measured in the five tutorials of the manual (`docs/manual/tutorial*.md`) and in the `Create...`
functions of `exudyn.misc.mainSystemExtensions` (2026-09-27):

- **A reader meets most items without writing them.** The tutorials after the first one build their
  models with `mbs.CreateRigidBody`, `CreateRevoluteJoint`, `CreateForce`, `CreateSpringDamper`,
  and each of those adds two to five items. `CreateRigidBody` adds `NodeRigidBodyEP`,
  `ObjectRigidBody`, and - with gravity - `MarkerBodyMass` and `LoadMassProportional`;
  `CreateRevoluteJoint` adds two `MarkerBodyRigid` and an `ObjectJointRevoluteZ`. Only the
  mass-spring-damper tutorial and the rigid-body tutorial's *"first two possibilities"* write items
  one by one. The rigid body tutorial is the one place that says which items a Create function made
  (after `DrawSystemGraph`).
- **So the reader's question is the combined view**: *"my script says `CreateForce(bodyNumber=b,
  localPosition=...)` - what acts on my body, and in which frame?"*. The answer is on three pages -
  `LoadForceVector` (the load), `MarkerBodyRigid` or `MarkerBodyPosition` (which of the two depends on
  an argument), and `ObjectRigidBody` (how the force reaches the equations) - and **none of the three
  says that `CreateForce` makes it**, nor does the docstring of `CreateForce` name the items it adds.
- The chain a reader has to reconstruct is always the same - **node -> object -> marker -> connector
  or load**, and a sensor on any of them - and it is drawn once, as a diagram, in
  `docs/manual/introduction.md` §*Items*. The item pages do not refer to it.

What the item documentation needs from this, for every kind:

1. **"Created by"**: each item page names the Create functions that add it, and each Create function
   names the items it adds. Measured by a static scan of `mainSystemExtensions.py`: the 22 Create functions
   make 36 different items. A static scan misses what an argument selects (`CreateRigidBody` makes one
   of four nodes, `CreateForce` one of two markers), so the reliable source is **running** each Create
   function on a small model and recording what it added - a generator step, not a hand-written list.
2. **"Can be used with"**: the compatible items of the neighbouring kinds, generated from the type
   declarations (§3). This is the answer to *"which marker do I need for this joint"*, which the
   marker index text asks the reader to work out by hand today.
3. **A general section per kind**, before the first item of the kind (the index page of the kind),
   which says once what every item of the kind does, so that each page says only what is its own.
   The maintainer, 2026-09-27: all sensors behave alike, and loads and markers nearly so.

## 2. The frame text the generator writes today

`tools/generators/itemDocsEmitter.py` writes around the authored text of every page:

| where | text today | finding |
|---|---|---|
| after the class description | **Additional information for X**: | a heading that says nothing; the list under it is the item's *interface* |
| in that list | This `Node` has/provides the following types = `Position` | a type bit, not what it means; see §3 |
| | Requested `Marker` type = `Position` | the same, from the other side |
| | Requested `Node` type: read detailed information of item | for `_None`; tells the reader to look elsewhere |
| | **Short name** for Python = `Point`; **Short name** for Python visualization object = `VPoint` | two lines for one fact |
| before the parameters | The item **X** with type = 'Point' has the following parameters: | correct; the type string is the one of the dict |
| | The item VX has the following parameters: | the visualization parameters, without saying so |
| after the parameters | ## DESCRIPTION of X | one heading over three different things: output variables, equations, user functions |
| | **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**: | long; the same for every item |
| end | Relevant Examples (Ex) and TestModels (TM) with weblink to github: | up to 129 links (`NodePointGround`), in file order |
| index page of a kind | the `globalItemIntros` of the emitter, one paragraph | the only general text per kind, and a draft of the general section |

Proposed wording - the content stays generated:

- **Interface** instead of *Additional information*, with the lines of §3.
- **Python names**: `Point`, `VPoint` - one line.
- **Parameters** and **Visualization parameters** as two headings of their own.
- **Output variables**, **Equations** (or the kind's name for them, see the documents per kind) and
  **User functions** as headings of their own instead of one *DESCRIPTION*.
- The examples: the **five** that use the item most plainly first (the shortest scripts), the rest
  behind them; which ones is measured, not chosen by hand.

## 3. The types, and what they should say

Every item declares types in its definition, and the generator already reads them
(`itemTypes`, `requestedTypes`, `accessFunctionTypes`; measured table in the scratch inventory of
RG13.4). They are **the compatibility rules of the model**, and a page can say them in words:

| declaration | today on the page | what it means, and what the page could say |
|---|---|---|
| node `GetType` = `Position` | has/provides the following types = `Position` | *"markers: `MarkerNodePosition`, `MarkerNodeCoordinate(s)`; objects: `ObjectMassPoint`, ..."* - every marker and object that accepts it |
| object `GetRequestedNodeType` = `Position` | Requested `Node` type = `Position` | *"nodes: `NodePoint`, `NodeRigidBodyEP`, ..."* - every node that provides it |
| object `GetType` = `Body, SingleNoded` | has/provides ... = `Body`, `SingleNoded` | *"a body with one node"* |
| object `GetAccessFunctionTypes` | not shown | which body markers work: `MarkerBodyPosition` needs `TranslationalVelocity_qt`, `MarkerBodyRigid` also `AngularVelocity_qt`, `MarkerBodyMass` needs `DisplacementMassIntegral_q` (the rule is `CSystem::CheckSystemIntegrity`, `src/Main/CSystem.cpp`) |
| marker `GetType` = `Body, Object, Position, JacobianDerivativeAvailable, ...` | all of them | the user-facing part (`Position`, `Orientation`, `Coordinate`, ...) and *"usable by: connectors that request Position: ..., loads: `LoadForceVector`"*; `JacobianDerivativeAvailable`, `JacobianDerivativeNonZero`, `HasPostNewton` are internal and should not be listed |
| connector, load `GetRequestedMarkerType` = `Position` | Requested `Marker` type = `Position` | *"markers: `MarkerBodyPosition`, `MarkerBodyRigid`, `MarkerNodePosition`, ..."* |

**One rule is not declared**: which node a *node marker* needs. `MarkerNodePosition` needs a node of
type `Position` or `Position2D`, `MarkerNodeRigid` also `Orientation` or `Orientation2D` - and that is
written only in C++ (`src/Main/CSystem.cpp`, the marker loop of `CheckSystemIntegrity`), with a third
check for `MarkerNodeRotationCoordinate` in `checkPreAssembleConsistenciesMarkers.cpp`. Generating the
lines of the table needs it declared in the definition of the marker, as `requestedNodeTypes`, from
which the C++ check could then be generated as well.

## 4. What every page contains, whatever its kind

In this order; *generated* means the author writes nothing:

1. the **class description** - one or two sentences: what it is and what it is for *(written)*;
2. **Interface** - the Python names, the types in words (§3), "Created by" (§1) *(generated)*;
3. **Parameters**, **Visualization parameters** *(generated from the parameter descriptions)*;
4. **Output variables** *(generated; nodes, objects, markers only)*;
5. the part that belongs to the kind - equations, coordinates, what it measures - *(written, per the
   document of the kind)*, with its **frames stated**: every vector says in which frame it is given;
6. **User functions** *(generated since #2664)*;
7. **MiniExample** *(written, run by the test suite; RG13 wants one for every item)*;
8. **Examples** *(generated, see §2)*.

## 5. Open for the maintainer

- The Create function scan (§1.1) at generation time needs the module importable - run as part of
  `generate`, or as a separate step that writes a JSON file the emitter reads?
- `requestedNodeTypes` for node markers (§3): declare it, and generate the C++ check from it?
- Should the frame text change (§2) come as one step before the authoring, so that every page
  written in RG13 is written into the new frame?
