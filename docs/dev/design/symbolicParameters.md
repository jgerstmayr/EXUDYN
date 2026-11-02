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

A **binding** ties one number of the model - a *slot* - to an expression of the parameters. The slots are the
entries of the `parameters` vectors of the user items (2.3):

```python
(objectNumber, index) -> expression
(5, 0)     -> m1
(5, 1)     -> rho*a*b*c*(b*b + c*c)/12     #the inertia Jxx of a cuboid in the parameters of a user rigid body
(7, 2)     -> L/2                          #a local position among the parameters of a user joint
```

- The item keeps the **number**, the value of the expression, as now: the C++ system runs unchanged, and the check of
  #3152 - the value of the expression equals the number of the item - is done for every slot.
- An expression may be any `esym.Real` whose named variables are the parameters - a variable, or an expression such
  as `rho*a*b*c` (P2). The variables of an expression are found by a walk over it, as `esym.Function` finds them
  (`NamedVariablesOf`, #3198), which Python then needs as a function.
- `ParameterBindings` (a small class in `exudyn.symbolics`): the symbolic list of the `parameters` of each item, the
  named variables found in them - one variable per name, the scope of names of the system (symbolicVariableNames.md,
  N2) -, `Check(mbs)`, `Apply(mbs)` (the numbers into the items) and the expression of a slot for `Equations()`.
  `creator.parameterVariables` of #3152, not released, becomes `creator.parameterBindings`.
- **Structure stays Python.** A value that changes the structure of the computation - a number of points, a
  formulation, a flag - cannot be bound; it is a Python constant of the recording (#3203). A binding is only for
  values that enter the equations as numbers.

### 2.3 Where bindings come from: the parameters of the user items (revised 2026-11-02)

The maintainer's arguments of 2026-11-02:

1. The user items take symbolic parameters already: the items to be treated symbolically become user items, and
   their `parameters` carry the values. Loads, markers, ObjectGenericODE2 and ObjectGenericDAE have no user items
   yet; user loads and Python objects for the markers and the generic objects would bring their parameters with them,
   when they are needed.
2. Symbolic parameters are named variables, as derivatives are taken by them; expressions of them are possible as
   well.
3. The bookkeeping belongs to the symbolic layer, where it is now. MainSystem, its Add/Modify/Set functions and the
   C++ do not know SymbolicSystem, so `mbs.parameterBindings` (option B of the first revision) is the wrong place.
   The slot binding is kept, as a part of the symbolic layer, with classes of its own if needed.
4. The bookkeeping of UserItemCreator becomes simpler with one function that adds a user item: it does what each
   Create function does now by hand -

   ```python
   self.items['massPoint'].append(bodyNumber)
   self._KeepParameters(bodyNumber, [mass])
   self._massPointNodes[int(bodyNumber)] = nodeNumber
   ```

   - systematically: the item with the numbers of its parameters into mbs, its kind, and its symbolic parameters
   into the bindings. `_massPointNodes` goes (#3207): it puts a connector at the center of a user mass point on a
   marker of the node instead of the body, an optimization SymbolicSystem does not need, as it takes position markers
   of user bodies.

Not all dependencies are needed now: no workaround for the loads.

The model that follows (option D of 2.7, decision P1):

```python
m1 = esym.Real('m1', 13.5); L = esym.Real('L', 0.8)
creator = UserItemCreator(mbs)
b0 = creator.CreateMassPoint(referencePosition=[1, 0, 0], mass=m1)
b1 = creator.CreateRigidBody(inertia=InertiaCuboid(density=rho, sideLengths=[L, 0.1, 0.1]), ...)
creator.CreateSpringDamper(itemNumbers=[b0, b1], stiffness=k, damping=2.)
creator.Record()
system = SymbolicSystem(mbs, parameters=creator.parameterBindings)
```

- **`UserItemCreator._AddUserItem(kind, item, parameters)`**: `mbs.AddObject` of the item with the numbers of
  `parameters`, `items[kind]`, and the symbolic list into `parameterBindings`; every Create function calls it, so
  that a new Create function or a new kind gets the bindings without bookkeeping of its own.
- **The C++ items stay numeric**: the validated dicts carry numbers; MainSystem and the Add/Modify/Set functions are
  unchanged.
- **Readable**: `creator.parameterBindings[item]` gives `[m1, 2., m1*L]`, the expression where a slot is bound and
  the number otherwise; `creator.parameterBindings.Parameters()` lists the named variables.
- **The inertia** (decision P3): `RigidBodyInertia` and its shapes (`InertiaCuboid`, `InertiaCylinder`, ...) work
  with numbers and with symbolic values. Numbers give numpy arrays as now; symbolic values give `esym.Real`,
  `esym.Vector` and `esym.Matrix` members. Probed on 2026-11-01, they fail today in a few places:
  - the shape checks (`np.array(...).shape` of an `esym.Matrix`);
  - conditions on values (`if` in `InertiaCylinder`, `InertiaSphere`), which become decisions by type, as in
    `exudyn.symbolics.utilities.IfThenElse`.

  `creator.CreateRigidBody(inertia=...)` puts the symbolic mass, inertia and center of mass into the parameters of
  the user body.
- **The frames of joints and connectors**: symbolic only where they are parameters of the user item - a user joint
  that takes its local frames from its parameters, on markers at the reference frames of the bodies. Markers of their
  own with symbolic positions (Python objects for markers) are a later step (P7).
- **By hand** remains possible: `creator.parameterBindings.Keep(item, parameters)` for a user item added without the
  creator.

### 2.4 SymbolicSystem with bindings

- `SymbolicSystem(mbs, parameters=creator.parameterBindings)`: `Equations()` reads every bound slot as its expression,
  where `_EquationLists` takes the `parameters` of an item now (the `_ParameterVector` of #3152 with expressions).
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

- Bindings stored with MainSystem and made by its Add/Modify/Set functions (option B of 2.7): MainSystem does not know
  the symbolic layer.
- Symbolic values stored inside the C++ items (option C of 2.7).
- The reference coordinates of the nodes as parameters: symbolicSystem.md section 12.3, an option of its own.
- Parameters of loads, markers and the generic objects now: they come with user items for them (P7).

### 2.7 Options for the source of the bindings (evaluated 2026-11-01, revised 2026-11-02)

| | how a binding comes about | readable | effort | remark |
|---|---|---|---|---|
| **A** explicit (first proposal) | `bindings.Add(slot, expr)`, or the Create functions of UserItemCreator | no | small | every place that writes a number must also write the binding |
| **B** written into any item of mbs (revision of 2026-11-01) | an `esym.Real` in any field at Add/Modify/Set: the value to C++, the binding to `mbs.parameterBindings` | `symbolic=True` | small to medium | dropped: MainSystem and its functions would know the symbolic layer (argument 3) |
| **C** symbolic values stored in C++ | the C++ items hold `symbolic.Real`/`Vector` for their parameters | yes | large | every parameter type twice; the C++ evaluation needs the numbers anyway |
| **D** the parameters of the user items, kept by the creator (proposed) | the Create functions of UserItemCreator through `_AddUserItem`; by hand `parameterBindings.Keep` | `creator.parameterBindings` | small | the slots are the `parameters` of user items; what else is to be symbolic becomes a user item when needed |

- **D** answers the questions of 2026-11-01 as well: an `esym.Real` written into a user item's `parameters` by the
  creator is a binding, and the creator shows it again.
- **Limits of D**: only values in the `parameters` of user items are bound - the built-in items, the markers and the
  loads keep numbers until they have user items (P7); a user item added by `mbs.AddObject` directly needs `Keep`.

## 3. Sub-steps proposed

1. This design (RG12.67.1).
2. `ParameterBindings` in `exudyn.symbolics` with the symbolic `parameters` of the user items and one variable per
   name; `UserItemCreator._AddUserItem` for every Create function, `creator.parameterBindings` instead of
   `parameterVariables`; the connectors on user mass points through body markers, `_massPointNodes` removed (#3207);
   expressions as parameters (P2); SymbolicSystem with them in `Equations()`; a test model with expressions of
   parameters in a mass point, a spring-damper and a rigid body, against SetParameterValues and central differences.
3. `RigidBodyInertia` and its shapes for numbers and symbolic values (P3); `creator.CreateRigidBody` with the
   symbolic inertia, the user joints with their frames among their parameters where they are to be symbolic.
4. The parameters as an input of the composed system: `SetParameterValues` without a rebuild,
   `ResidualParameterJacobian` (dr/dp) and dM/dp in one pass with the derivatives by the coordinates.
5. Forward sensitivities of the solution in `SolveGeneralizedAlpha` and `SolveExplicit`.

Later, when needed (P7): user loads, Python objects for markers, and the parameters of ObjectGenericODE2 and
ObjectGenericDAE.

## 4. To be decided

- **P1** (revised 2026-11-02) The slots are the entries of the `parameters` of the user items; the bindings are
  kept in the symbolic layer, `ParameterBindings` of UserItemCreator, filled by one function that adds a user item
  (option D of 2.7); MainSystem and the C++ items stay numeric and know nothing of them. Proposed: D; B and C not.
- **P2** An expression of named parameters, not only a named variable, for a slot. Proposed.
- **P3** The inertia: `RigidBodyInertia` and its shapes for both numbers and symbolic values, instead of symbolic
  counterparts. Proposed.
- **P6** (revised 2026-11-02) Reading the expressions back from `creator.parameterBindings`, not from mbs. Proposed.
- **P7** (new) What else is to be symbolic - loads, markers, the generic objects - becomes a user item when it is
  needed, its values its parameters; nothing for it now. Proposed.
- **P4** The parameters as an input of the composed system, so that new values need no rebuild and the derivatives
  by p come with those by q. Proposed.
- **P5** Forward sensitivities of the solution as the last sub-step; adjoints not. Proposed.
