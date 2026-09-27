# Node documentation (development document)

*Temporary, revision2026b step RG13.4.1 (#2721); what is common to every kind is in
[itemDefinitionsDev.md](itemDefinitionsDev.md). 16 nodes in `definitions/itemDefsNodes.py`.*

## 1. What a reader needs to know about a node

A node is met through an object: `CreateMassPoint` makes a `NodePoint`, `CreateRigidBody` one of
`NodeRigidBodyEP`, `NodeRigidBodyRxyz`, `NodeRigidBodyRotVecLG`, `NodeRigidBody2D`, the beam
utilities make the slope nodes. A reader comes to a node page with one of four questions:

1. **What are its coordinates** - how many, of which kind (ODE2, ODE1, AE, data), in which order,
   and what each one is: a displacement, a rotation parameter, a slope?
2. **In which frame are they** - and who decides that. `NodePoint` says it: *"usually the nodal
   coordinates are provided in the global frame. However, the coordinate system is defined by the
   object"* (`ObjectMassPoint` global, `ObjectFFRF` local). That is the maintainer's point: **one node
   can be interpreted differently by different objects**, and the node page must say which
   interpretation is the default and which objects deviate.
3. **How does it act on the equations of motion** - the maintainer, 2026-09-27: *the coordinates,
   and how they act on the equations of motion; for Euler parameters, the equations are the global
   ones, projected with the velocity transformation*. A node has no equations of its own, but the
   choice of coordinates decides the form of the equations every object writes with it: which rows
   are forces and which are torques projected by $\Gm\tp$, and which constraint comes with it (the
   Euler parameter norm).
4. **What can I attach to it** - which markers (and so which loads and connectors), which objects,
   which sensors and output variables.

What a node page does **not** need: the equations of an object. It refers to them.

## 2. What the pages say today

Measured (RG13.1 table, `itemDocumentationState.md`, and the definitions, 2026-09-27):

| | nodes | |
|---|---|---|
| without any text beyond the class description | 9 of 16 | the four slope nodes, the four generic nodes, `NodePointGround` |
| with an equations text | 7 | all under one bold *Detailed information:*, none with a sub-heading |
| saying in which frame the coordinates are | 3 | `NodePoint`, `NodeRigidBodyEP` (*"residuals of translational forces in global coordinates"*), `NodeRigidBody2D` |
| saying how they act on the equations of motion | 3 | `NodeRigidBodyEP`, `NodeRigidBody2D`, `NodeRigidBodyRxyz` in part |
| with a MiniExample | 0 | |
| with a figure | 0 | |

What the rigid body nodes already do right, and what should be the rule: `NodeRigidBodyEP` lists
the coordinates, gives the rotation matrix as a function of them, the $\Gm$ matrices, the constraint
on the Euler parameters, and states that the rotational equations are torques left-multiplied with
$\Gm\tp$. `NodeRigidBodyRxyz` names its singularity in the class description.

Textual findings (not fixed here - rule 9; they are material for the authoring steps):

- `NodePointSlope1`: *"in straight configuration aligned at the global x-axis, the slope vector
  reads $[1\;0]^T$"* - a 3D node, three components.
- `NodePointSlope23`: *"the slopeY vector defines the directional derivative w.r.t the local axial
  (y) coordinate"* - whether y is axial for this node is what a reader needs said plainly.
- `NodeGenericODE2/ODE1/AE`: *"referenceCoordinates and all initialCoordinates(\_t) must be
  initialized, because no default values exist"* - a requirement in a class description, where the
  parameter table has the place for it: `numberOfODE2Coordinates` is marked *must be given* there
  (`CFMustBeGiven`, #2426), the coordinates are not.
- `NodePointGround`: *"Applied or reaction forces do not have any effect"* - true and important; it
  also provides `Orientation` (type list) without being able to rotate, which a reader sees on the
  page and cannot explain.
- The generated line *"This Node has/provides the following types = `Position`"*: see
  [itemDefinitionsDev.md](itemDefinitionsDev.md) §3 - the page should say *which markers and
  objects* that allows.

## 3. The ideal node page

After the common head (class description, interface, parameters; itemDefinitionsDev §4):

| section | contents | source |
|---|---|---|
| **Coordinates** | a table: index, symbol, kind (ODE2/ODE1/AE/data), meaning (displacement, rotation parameter, slope), frame | written; the count and kind could be checked against the definition |
| **Configuration** | reference + current = configuration, for the position and, if it has one, the rotation: $\pv = \pv\cRef + \uv$, $\ttheta = \tpsi\cRef + \tpsi$; the rotation matrix as a function of the coordinates | written |
| **Frame and interpretation** | the default frame of the coordinates, and **which objects interpret them differently** (FFRF, beams) | written - the part the maintainer asked to make systematic |
| **Action on the equations of motion** | which rows of the object's equations belong to which coordinates, and in which form: forces in global coordinates, torques projected by $\Gm\tp$; the velocity transformation $\tomega = \Gm \dot\ttheta$ | written, short - the equations themselves are the object's |
| **Constraints that come with the node** | e.g. the Euler parameter norm, who adds it (`addConstraintEquation`) | written |
| **Singularities and limits** | Tait-Bryan at $\psi_1 = \pm\pi/2$; slopes of zero length | written, where there are any |
| **Output variables** | generated | |
| **MiniExample** | a node alone does not simulate: the MiniExample is the smallest object on it | written |

For the generic nodes (`NodeGenericODE2`, `ODE1`, `AE`, `Data`) the table of coordinates is replaced
by one sentence: *the meaning of the coordinates is defined by the object that uses the node*, and a
list of the objects that do (`ObjectGenericODE2`, the contact objects for `NodeGenericData`, ...) -
which the "Can be used with" line generates.

## 4. The general node section

Before the first node (the index page of the nodes), replacing today's one paragraph:

- **What a node is**: it provides coordinates and nothing else - no mass, no stiffness; an object
  provides the equations for them. The order of the nodes gives the order of the system coordinates
  (today's paragraph says that).
- **Reference, initial, current**: the three sets of coordinates and what `referenceCoordinates +
  initialCoordinates` means - today in `introduction.md` §*Reference coordinates and displacements*;
  the section links it.
- **The kinds of coordinates**: ODE2, ODE1, AE, data - one line each, and what the solver does with
  them.
- **Frames**: the default is global; an object may interpret a node in its own frame; every node
  page says which.
- **Rotation parametrizations**: a table of the rigid body nodes - Euler parameters, Tait-Bryan
  angles, rotation vector (Lie group), 2D angle - with their number of coordinates, their constraint,
  their singularity and which integrator suits them, and a link to `theoryRotations.md`. The choice
  is made at `CreateRigidBody(nodeType=...)`; the reader choosing needs the comparison, not four
  pages.
- **Which markers attach to which node**: the rule of itemDefinitionsDev §3 in one table.

## 5. Open for the maintainer

- Coordinates as a **table** on every node page - agreed? It is the one element all 16 would share.
- The slope nodes (4) and the generic nodes (4) have no text: write them in the authoring step of
  the beams, or here with the nodes?
