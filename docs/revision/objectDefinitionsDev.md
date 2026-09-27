# Object documentation (development document)

*Temporary, revision2026b step RG13.4.2 (#2721); what is common to every kind is in
[itemDefinitionsDev.md](itemDefinitionsDev.md). 51 objects in `definitions/itemDefsObjects.py`, in
seven kinds - the `objectType` of the definition, which also sorts the reference manual.*

The maintainer, 2026-09-27: objects are the most important part - *"they really need their
equations"* - and they fall into groups that document alike: **bodies**, **flexible bodies** (beams,
shells, plates - nonlinear finite elements), **connectors**, which act on two or more markers, define
a force from the kinematics and apply it through what the markers provide, and **constraints and
joints**, which do the same with algebraic equations.

## 1. What a reader needs to know about an object

From the tutorials and the Create functions: a reader has written `CreateRigidBody(...)`,
`CreateRevoluteJoint(...)`, `CreateSpringDamper(...)` and asks

1. **what equations** the object adds - the equations of motion of a body, the force law of a
   connector, the constraint equations of a joint;
2. **in which quantities**: every symbol of those equations traced to a parameter, a node coordinate
   or a marker quantity, **with its frame**;
3. **how it couples to the rest**: a body through its node(s) and the markers placed on it, a
   connector through the markers it connects - *which* marker quantities it uses (position,
   orientation, their Jacobians) and how its force comes back to the coordinates ($\Jm\tp \fv$);
4. **what it outputs** and what the output variables mean for this object (the force of a joint is
   in which frame?);
5. **what can go wrong**: singular configurations ($L = 0$ of a spring-damper), redundant
   constraints, the index of the constraints and which integrator suits them.

## 2. What the pages say today

The pages are in better shape than those of any other kind (RG13.1): only 3 of 51 have no equations
text, and a de facto structure exists, measured from the sub-headings of the equations texts:

| kind | items | the sub-headings that recur | gaps |
|---|---|---|---|
| Body | 7 | *Definition of quantities* (6), *Equations of motion* (5) | none without text; `ObjectRigidBody` is the model page (850 words, a figure) |
| FiniteElement | 7 | only `ObjectANCFCable2D` has sections (*Kinematics and interpolation, Mass matrix, Elastic forces, ...*) | **`ObjectANCFBeam` 4 words, `ObjectBeamGeometricallyExact` 4, `...2D` 11, `ObjectANCFThinPlate` 15, `ObjectANCFCable` none** - the largest gap of all items |
| SuperElement | 4 | *Equations of motion*, *Definition of quantities*, varied | long and complete (FFRF 849 / 1224 words, KinematicTree 1090) |
| Connector | 19 | *Definition of quantities* (17), *Connector forces* (14), *Geometric relations* (3) | `ObjectContactCoordinate` none, `ObjectContactCircleCable2D` 24 words, `SphereTorus`, `SphereTriangle`, `CurveCircles` about 100 |
| Constraint | 3 | *Definition of quantities* (3), *Connector constraint equations* (2) | |
| Joint | 10 | *Definition of quantities* (8), *Connector constraint equations* (6), *Geometric relations* (5), *Post Newton Step* (3) | `ObjectJointRevolute2D` none, `ObjectJointPrismatic2D` 89 words |
| Object | 1 | *Equations of motion* | |

So the structure to prescribe is **the one most pages already follow**; the work is the pages that
do not, and the finite elements.

Textual findings:

- The table *Definition of quantities* has two forms: *intermediate variables / symbol /
  description* and a second one *output variables / symbol / formula* (`ObjectConnectorSpringDamper`).
  A reader profits from both; they should have fixed headings.
- Frames: `ObjectMassPoint` says *"local (body) coordinate system = global coordinate system"*;
  connectors write $\LU{0}{\pv}_{m0}$ with the frame in the symbol - consistent where the notation
  (`notation.md`) is used, not said in words where it is not.
- How the force reaches the coordinates is written out in some connectors (virtual work,
  $\Jm_{pos}\tp$ in `ObjectConnectorSpringDamper`) and the mass point, and assumed elsewhere. It is
  the same for all connectors and belongs to the general section once (§4).
- Output variables of connectors: `Displacement`, `Velocity`, `Distance` have no symbol in the
  output table of `ObjectConnectorSpringDamper` and get one in the quantities table below it - two
  places for one definition.

## 3. The ideal object page, per group

After the common head (itemDefinitionsDev §4). The headings are fixed so that a reader finds the
same thing in the same place on every page of the group.

### Bodies (rigid, mass points, 1D masses)

| section | contents |
|---|---|
| **Kinematics** | position and velocity of a local point $\pLocB$, rotation matrix - in terms of the node's coordinates, with frames |
| **Definition of quantities** | the table: symbol, meaning, frame, from which parameter or node |
| **Equations of motion** | mass matrix, quadratic velocity vector, applied forces; the form they take for the node's coordinates (refers to the node page for the rotation parametrization) |
| **Markers and loads** | the position Jacobian and rotation Jacobian through which a marker applies a force or torque |

### Flexible bodies (beams, cables, plates, shells - nonlinear finite elements)

| section | contents |
|---|---|
| **Nodes and coordinates** | which nodes, how many coordinates, their order in the element |
| **Kinematics and interpolation** | shape functions, the position of a point of the axis or mid-surface, the reference configuration |
| **Strains** | the strain measures, and which are in the model (axial, bending, shear, torsion) |
| **Mass matrix** | constant or not, how it is integrated |
| **Elastic forces** | the virtual work, the generalized forces, the integration rule |
| **Access functions** | what a marker on the element sees (`MarkerBodyPosition` at an axial coordinate, the beam shape markers) |
| **Limitations** | locking, the range of validity, what is not implemented |

`ObjectANCFCable2D` has this structure already and is the template.

### Connectors (spring-dampers, contacts, penalty joints)

| section | contents |
|---|---|
| **Markers** | the marker quantities used - $\pv_{m0}, \pv_{m1}$, rotation matrices, velocities - and which marker types provide them (generated line, itemDefinitionsDev §3) |
| **Definition of quantities** | the table |
| **Geometric relations** | distance, relative rotation, contact geometry - the kinematics of the connection |
| **Connector forces** | the force law, $f = k(L-L_0) + d\dot L + f_a$ and the like; the user function form |
| **Output variables** | what each output means for this connector, in which frame |

The step from the connector force to the generalized forces ($\Jm\tp\fv$ on each marker) is the same
for all and is in the general section; a page that deviates says so.

### Constraints and joints

| section | contents |
|---|---|
| **Markers** | as for connectors; the joint frames (`rotationMarker0/1`) |
| **Definition of quantities** | the table |
| **Geometric relations** | the constrained directions in the joint frame |
| **Constraint equations** | position level (index 3) and, where there is one, velocity level (index 2); which is used when (`velocityLevel`, `classicalFormulation`) |
| **Lagrange multipliers and output** | what the multipliers are physically - the joint force and torque, in which frame |
| **Post Newton step** | where there is one (the sliding joints) |

### Superelements and the general object

Their pages are long and individual; the rule is only that they have the same fixed headings where
they apply (*Definition of quantities*, *Equations of motion*, *Markers*).

## 4. The general object section, per group

Before the first object of each group (the index pages `objectBodyIndex`, `objectConnectorIndex`,
`objectJointIndex`, ... already exist, with one paragraph each):

- **Bodies**: what a body is to the solver (it owns nodes, provides mass matrix and forces); local
  and global frames; how a marker on a body gets position and Jacobian.
- **Connectors**: the principle every connector follows - *the markers provide positions,
  orientations and Jacobians; the connector computes a force from them; the force goes back through
  $\Jm\tp$* - once, with the virtual work, so that each connector page only gives its force law;
  `activeConnector`; what `Force` and `ForceLocal` mean.
- **Constraints and joints**: Lagrange multipliers, index 3 and index 2, redundant constraints and
  how to find them (`ComputeSystemDegreeOfFreedom`), which solvers handle them.
- **Finite elements**: the conventions of all elements - reference configuration, the axial
  coordinate, how elements share nodes, how to build a mesh (`exudyn.beams`).

## 5. Open for the maintainer

- The **finite elements** first? They have the largest gap and the equations are the maintainer's
  own work (ANCF, geometrically exact beams) - an authoring step that needs the maintainer more than
  any other.
- Fixed headings as a **check**: `checkDefinitions` could warn about an object whose equations text
  lacks the headings of its group - after the pages have them, not before.
