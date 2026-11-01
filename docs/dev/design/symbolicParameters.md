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
  `creator.parameterVariables` of #3152 is the special case of the field `'parameters'` of objects; it is replaced by
  the bindings written into the items (2.3).
- **Structure stays Python.** A slot whose value changes the structure of the computation - a number of points, a
  formulation, a flag - cannot be bound; it is a Python constant of the recording (#3203). A binding is only for
  values that enter the equations as numbers.

### 2.3 Where bindings come from: written into the items (revised 2026-11-01)

The maintainer asked whether a parameter such as `m1 = esym.Real('m1', 13.5)` can simply be written into an item,
for example into the `parameters` vector of a user item, and be recognized as a binding. It can, and this is now the
proposed way (decision P1, section 2.7):

```python
m1 = esym.Real('m1', 13.5); L = esym.Real('L', 0.8)
oBody = mbs.AddObject(ObjectConnectorUser(markerNumbers=[m0, m1], parameters=[m1, 2., m1*L], ...))
mMid = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b, localPosition=[L/2, 0, 0]))
system = SymbolicSystem(mbs)            #finds the bindings of mbs; parameters=... no longer needed
```

1. **Automatic, at the Python boundary.** `mbs.AddNode/AddObject/AddMarker/AddLoad/AddSensor`, the `Modify...`
   functions and `SetObjectParameter` (and the like) take any `esym.Real` - a named variable or an expression of
   them - wherever the item has a number, also as a component of a vector or matrix. The Python layer replaces it by
   its value before the dict reaches C++, and adds the binding of that slot. The slot is (item type, item number,
   field, component); the item number is known once the Add returns.
   - Today such a dict fails: `float()` of an `esym.Real` raises, so no model relies on the conversion and none
     changes.
   - A number written later into a bound slot removes its binding; a new `esym.Real` replaces it; `mbs.Reset()`
     clears all bindings.
   - The C++ items stay numeric and unchanged: the validated dicts carry numbers as before.
2. **The bindings live with the system**, `mbs.parameterBindings` (a `ParameterBindings` in `mbs.sys`). They are
   readable: `mbs.GetObject(o, symbolic=True)['parameters']` gives `[m1, 2., m1*L]`, the expression where a slot is
   bound and the number otherwise, and `mbs.parameterBindings.Parameters()` lists the named variables found.
3. **UserItemCreator** gets the bindings for free: its Create functions write their arguments into the items, which
   records the slots; `creator.parameterVariables` of #3152 becomes unnecessary and is removed.
4. **By hand** remains possible for a slot written without a symbolic value: `mbs.parameterBindings.Add(slot, expr)`.
5. **The inertia** (decision P3): `RigidBodyInertia` and its shapes (`InertiaCuboid`, `InertiaCylinder`, ...) work
   with numbers and with symbolic values. Numbers give numpy arrays as now; symbolic values give `esym.Real`,
   `esym.Vector` and `esym.Matrix` members. Probed on 2026-11-01, they fail today in a few places:
   - the shape checks (`np.array(...).shape` of an `esym.Matrix`);
   - conditions on values (`if` in `InertiaCylinder`, `InertiaSphere`), which become decisions by type, as in
     `exudyn.symbolics.utilities.IfThenElse`.

   `CreateRigidBody(inertia=...)` then writes the symbolic mass, inertia and center of mass into the body, and
   the bindings follow from 1.

### 2.4 SymbolicSystem with bindings

- `SymbolicSystem(mbs)` takes `mbs.parameterBindings`: `Equations()` reads every bound slot as its expression - the
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

- Symbolic values stored inside the C++ items (option C of section 2.7).
- The reference coordinates of the nodes as parameters: symbolicSystem.md section 12.3, an option of its own.

### 2.7 Options for the source of the bindings (evaluated 2026-11-01)

| | how a binding comes about | readable from the items | effort | remark |
|---|---|---|---|---|
| **A** explicit (first proposal) | `bindings.Add(slot, expr)`, or the Create functions of UserItemCreator | no | small | systematic, but every place that writes a number must also write the binding |
| **B** written into the items (proposed) | an `esym.Real` in any field at Add/Modify/Set: the value goes to C++, the binding to `mbs.parameterBindings` | yes, `symbolic=True` | small to medium: one hook in the Python layer of the Add/Modify/Set functions, a walk over the dict | A is its storage; every item, every field, every Create function at once |
| **C** symbolic values stored in C++ | the C++ items hold `symbolic.Real`/`Vector` for their parameters, e.g. a symbolic `parameters` of the user items | yes | large | every parameter type of the definitions twice, or a symbolic shadow of each item; the C++ evaluation needs the numbers anyway; the expressions are needed only by SymbolicSystem in Python |

- **B** answers both questions of the maintainer:
  - an `esym.Real` written into a user item's `parameters` is recognized as a binding;
  - the item shows it again with `symbolic=True`.
- A `parameters` vector of `esym.Real` stored in C++ (C) would give the same readability at much higher cost, and only
  for the user items.
- **Limits of B**:
  - The bindings are Python state of the system: they are not in a dict of the system that is written to a file, and a
    copy of mbs by its dicts loses them, unless the dict functions take `symbolic=True` as well.
  - A value that changes the structure of the computation stays a Python constant (2.2): a flag, a number of points or
    an index given as an `esym.Real` raises at the Add.

## 3. Sub-steps proposed

1. This design (RG12.67.1).
2. `ParameterBindings` in `mbs.parameterBindings`, filled by the Add/Modify/Set functions from the `esym.Real` written
   into an item (2.3), readable by `symbolic=True`; #3152's `parameterVariables` removed; SymbolicSystem with the
   bound slots of objects, markers and loads in `Equations()`; a test model with a user item's parameters, a joint
   frame, a marker position and a load as expressions, against SetParameterValues and central differences.
3. `RigidBodyInertia` and its shapes for numbers and symbolic values (P3); the Create functions of UserItemCreator
   and of mainSystemExtensions with symbolic arguments where they compute from them (the rigid body, the frames of
   joints and connectors, the matrices of the spring-dampers, the loads) - most need nothing beyond B.
4. The parameters as an input of the composed system: `SetParameterValues` without a rebuild,
   `ResidualParameterJacobian` (dr/dp) and dM/dp in one pass with the derivatives by the coordinates.
5. Forward sensitivities of the solution in `SolveGeneralizedAlpha` and `SolveExplicit`.

## 4. To be decided

- **P1** (revised 2026-11-01) Bindings by slot (item type, number, field, component) for every value of the model,
  stored with the system (`mbs.parameterBindings`), and **made by writing an `esym.Real` into the item** at
  Add/Modify/Set (option B of 2.7); explicit `Add` for the rest; #3152's `parameterVariables` removed. Proposed: B;
  C (symbolic values in the C++ items) not.
- **P2** An expression of parameters, not only a named variable, for a slot. Proposed.
- **P3** (revised 2026-11-01) The inertia: `RigidBodyInertia` and its shapes for both numbers and symbolic values,
  instead of symbolic counterparts. Proposed.
- **P6** (new) Reading the expressions back: `symbolic=True` for `GetNode/GetObject/GetMarker/GetLoad/GetSensor` and
  the dict functions of the system, so that a copy of a system by its dicts keeps its bindings. Proposed for `Get...`;
  the dicts later if needed.
- **P4** The parameters as an input of the composed system, so that new values need no rebuild and the derivatives
  by p come with those by q. Proposed.
- **P5** Forward sensitivities of the solution as the last sub-step; adjoints not. Proposed.
