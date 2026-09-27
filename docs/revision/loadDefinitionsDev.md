# Load documentation (development document)

*Temporary, revision2026b step RG13.4.4 (#2721); what is common to every kind is in
[itemDefinitionsDev.md](itemDefinitionsDev.md). 4 loads in `definitions/itemDefsLoads.py`:
`LoadForceVector`, `LoadTorqueVector`, `LoadMassProportional`, `LoadCoordinate`.*

The maintainer, 2026-09-27: loads need **short equations**, and a **general section**.

## 1. What a reader needs to know about a load

Met through `CreateForce`, `CreateTorque` and the `gravity` argument of `CreateMassPoint` and
`CreateRigidBody`; written by hand as `LoadCoordinate` in the first tutorial. The questions:

1. **what vector acts, in which frame** - global, or body-fixed with `bodyFixed=True`, and then
   rotating with the body;
2. **at which point** - the marker's; for `CreateForce(localPosition=...)` a point of the body;
3. **how it enters the equations** - through the marker's Jacobian, $\Qm = \Jm\tp\fv$;
4. **how it changes in time** - the user function, or `preStepUserFunction`, and what a static
   solver does with it (the load factor over the load steps).

## 2. What the pages say today

All four have a *Details* section (30 to 43 words) and a user function block; none has an equation,
none a figure; one a MiniExample (`LoadMassProportional`). What they say is correct and brief:
*"The marker transforms the (translational) force via the according jacobian matrix of the object
(or node) to object (or node) coordinates."*

Textual findings:

- The four *Details* sentences say the same thing four times - *the marker transforms the load with
  its Jacobian* - which is the general section's.
- `bodyFixed`: `LoadForceVector` says *"via the local (`bodyFixed = True`) or global coordinates of a
  body or at a node"* - what `bodyFixed` means on a **node** (a rigid body node rotates, a point node
  does not) is what a reader asks next, and no page says it.
- The static case is on the user function argument (*"WARNING: this parameter does not work in
  combination with static computation, as it is changed by the solver over step time"*) - a property
  of every load in a static solve, not of that argument.
- `LoadMassProportional` gives the example $[0,-g,0]$ in its class description; its load is a force
  per mass, $\fv = \int_V \rho\, \bv \,dV$ - the equation a reader needs to know that it is the
  **distributed** body load, not a point force at the centre of mass.

## 3. The ideal load page

| section | contents |
|---|---|
| **Load** | the load vector or scalar, its frame (global / body-fixed), the formula: $\fv$, $\tauv$, $\rho\bv$, or the scalar $f$ |
| **Generalized forces** | $\Qm = \Jm_{pos}\tp\fv$ (force), $\Jm_{rot}\tp\tauv$ (torque), $\int \rho \Jm\tp \bv\,dV$ (mass proportional), $f$ on one coordinate - one line each |
| **Marker** | the requested marker type and the markers that provide it (generated) |
| **User function** | generated |
| **MiniExample** | written - the three missing ones are short |

## 4. The general load section

Before the first load, replacing today's paragraph of the index page:

- a load acts through a marker, and the marker's Jacobian takes it to the coordinates - once;
- global and body-fixed loads, on bodies and on nodes;
- time-dependent loads: the user function of the load, or `preStepUserFunction`;
- loads in static solves: the load factor and the load steps (`staticSolver`), which apply to every
  load, with or without a user function;
- `CreateForce` and `CreateTorque`, and what they add.
