# Symbolic parameters of every item: design (revision2026c step RG12.67, #3159)

Maintainer 2026-10-29: symbolic parameters are required in the long term; the inertia of a rigid body and the joints
need a general solution beyond the scalar parameters of the user items (#3152). Design first: parameters of any
structure as named `esym.Real` or expressions of them, a parameter set per system, the sensitivities. This note
describes what exists, proposes the model, the sub-steps and what is to be decided.

## 1. What exists (#3152)

- `UserItemCreator.CreateMassPoint`, `CreateSpringDamper` and `CreateTorsionalSpringDamper` take a named `esym.Real`
  for a scalar argument; the item gets its value, and `creator.parameterVariables[item]` keeps the list of the
  item's `parameters` vector with the variable in its place.
- `SymbolicSystem(mbs, parameters=creator.parameterVariables)` checks each list (its length, the values those of the
  item, one variable per name) and puts the variables into `Equations()` where it reads the `parameters` of the item;
  `Equations()['parameters']` lists them, `GetFlatProgram` takes them as arguments, the derivative by them is
  `EvaluateJacobian` of the expressions.
- `SetParameterValues({'m1': 2.})` sets the variables and the numbers of the items in mbs, and **builds the system
  again** - the composed recordings of the evaluation take the numbers (`_Backend(parameterVariables=False)`).

What is not covered:

| value of the model | where it lives | symbolic now |
|---|---|---|
| `parameters` of ObjectUserBody, ObjectConnectorUser, ObjectJointUser, ObjectGenericDAE | the object | named variables only, for three Create functions |
| the inertia of `CreateRigidBody` (mass, inertia, center of mass) | `parameters` of the user body, computed from a `RigidBodyInertia` | no: `InertiaCuboid(density, sideLengths)` computes numpy numbers |
| local positions and rotations of markers - the frames of joints and connectors | the markers | no |
| the axes and frames of the joints of UserItemCreator | the markers (`localHT`), the joint frame | no |
| load vectors | the loads | no |
| matrices and vectors of ObjectGenericODE2 and ObjectGenericDAE | the object | no |
| reference coordinates of the nodes | the nodes | no (12.3 of symbolicSystem.md: an option of its own) |
| expressions of parameters (mass = rho a b c) | - | no: a list entry must be a named variable |

## 2. The model

### 2.1 The parameter set

The parameters of a system are **named `esym.Real`** - `rho`, `a`, `L`, `k` - collected in one `esym.VariableSet` of
the system (`system.parameters`): the set gives the names, the values, and the order of a parameter vector p for
the Jacobians, the export and the ensembles of RG14. A name is one variable (as now). Vectors and matrices of
parameters are their components; a named `esym.Vector` of the set (#3149) may be used for a vector-valued one.

### 2.2 Bindings

A **binding** ties one number of the model - a *slot* - to an expression of the parameters:

```python
(itemType, itemNumber, field, component) -> expression
('object', 5, 'parameters', 0)     -> m1
('object', 5, 'parameters', 1)     -> rho*a*b*c*(b*b + c*c)/12     #the inertia Jxx of a cuboid
('marker', 3, 'localPosition', 0)  -> L/2
('load', 2, 'loadVector', 1)       -> -m1*g
```

- The item keeps the **number**, the value of the expression, as now: the C++ system runs unchanged, and the check of
  #3152 - the value of the expression equals the number of the item - is done for every slot.
- An expression may be any `esym.Real` of the named parameters - a variable, or an expression such as `rho*a*b*c`.
- `ParameterBindings` (a small class in `exudyn.symbolics.systemModel`): `Add(slot, expression)`, the set of the
  parameters found in the expressions, `Check(mbs)`, `Apply(mbs)` (the numbers into the items by
  `SetObjectParameter`, `SetMarkerParameter`, `SetLoadParameter`), and the expression of a slot for `Equations()`.
  `creator.parameterVariables` of #3152 is the special case of the field `'parameters'` of objects and is converted.
- **Structure stays Python.** A slot whose value changes the structure of the computation - a number of points, a
  formulation, a flag - cannot be bound; it is a Python constant of the recording (#3203). A binding is only for
  values that enter the equations as numbers.

### 2.3 Where bindings come from

1. **UserItemCreator**: every argument that becomes numbers of an item may be an `esym.Real` or contain them - the
   mass, the inertia, the positions and axes of joints and connectors, stiffness and damping matrices, load vectors.
   The creator evaluates them for the item and adds the bindings of the slots it writes; it knows which argument
   goes into which slot. `creator.parameters` replaces `parameterVariables`.
2. **By hand**, for an item built with mbs directly: `bindings.Add(('marker', m, 'localPosition', 2), h)`.
3. **The inertia** is computed from parameters: a symbolic counterpart of `InertiaCuboid`, `InertiaCylinder`, ... -
   the mass, the inertia tensor about the center of mass and the center of mass as expressions -, or the three given
   by the user as expressions (decision P3). `RigidBodyInertia` itself computes with numpy and stays numeric.

### 2.4 SymbolicSystem with bindings

- `SymbolicSystem(mbs, parameters=bindings)`: `Equations()` reads every bound slot as its expression - the
  `parameters` of the objects as now, and the local positions and rotations of the markers in the marker kinematics,
  the load vectors, the matrices and vectors of ObjectGenericODE2 and ObjectGenericDAE -, each where `_EquationLists`
  takes the number now (the `_ParameterVector` of #3152, generalized to a slot).
- **The parameters as an input of the composed system** (option P1 of symbolicSystem.md section 12.4): the composed
  recordings get p as an input beside q, q_t, t (and y, z, x). Then `SetParameterValues` sets values only and needs
  no rebuild; a parameterized model is evaluated for other p without recording again, also as an ensemble over p
  (RG14); and the one-pass derivative gives `ResidualParameterJacobian` = dr/dp, and dM/dp, with the derivative by
  q in the same pass.
- The recordings of the kinds stay as they are: the kind's recording takes `parameters` as an argument, one recording
  for all items of the kind; the composition substitutes the expressions of the bound slots (RG12.60's substitution of
  the arguments, done in Python by the composition today).

### 2.5 Sensitivities

- **Of the equations**: dr/dp, dM/dp, dc/dp from 2.4 - for linearization, optimization, identification.
- **Of the solution** (forward sensitivity, dq/dp): in the Python solvers, per step one more right-hand side of the
  factorized Newton matrix, J S = -dr/dp (exact at convergence of the Newton iteration), for the generalized-alpha
  and the explicit solvers; checked against central differences of whole simulations. Adjoint sensitivities are not
  proposed.

### 2.6 What is not proposed

- Symbolic parameters inside C++ items - an item parameter linked to a variable: the binding table lives in Python,
  the C++ items hold numbers, and `Apply` writes them. When the GPU evaluation (RG14) needs p, it takes p as an input
  of the composed program, not from the items.
- The reference coordinates of the nodes as parameters: symbolicSystem.md section 12.3, an option of its own.

## 3. Sub-steps proposed

1. This design (RG12.67.1).
2. `ParameterBindings`, its checks and `Apply`; #3152's `parameterVariables` converted to it; SymbolicSystem with the
   bound slots of objects, markers and loads in `Equations()`; a test model with a joint frame, a marker position and
   a load as expressions, against SetParameterValues and central differences.
3. UserItemCreator with symbolic arguments in all Create functions: the rigid body (mass, inertia, center of mass -
   decision P3), the joints and connectors (positions, axes, frames), the matrices of the spring-dampers, the loads.
4. The parameters as an input of the composed system: `SetParameterValues` without a rebuild,
   `ResidualParameterJacobian` (dr/dp) and dM/dp in one pass with the derivatives by the coordinates.
5. Forward sensitivities of the solution in `SolveGeneralizedAlpha` and `SolveExplicit`.

## 4. To be decided

- **P1** Bindings by slot (item type, number, field, component) for every value of the model, with #3152's
  per-object lists as their special case. Proposed.
- **P2** An expression of parameters, not only a named variable, for a slot. Proposed.
- **P3** The inertia: symbolic counterparts of the inertia functions of rigidBodyUtilities for the usual shapes
  (cuboid, cylinder, sphere, and the translation and rotation of an inertia) in `exudyn.symbolics.kinematics`, and the
  mass, inertia and center of mass as given expressions besides. Proposed: both, a small set of shapes first.
- **P4** The parameters as an input of the composed system, so that new values need no rebuild and the derivatives
  by p come with those by q. Proposed.
- **P5** Forward sensitivities of the solution as the last sub-step; adjoints not. Proposed.
