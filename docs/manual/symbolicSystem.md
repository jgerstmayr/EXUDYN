(sec-symbolicsystem)=
# Symbolic system

`SymbolicSystem(mbs)` of `exudyn.symbolics.systemModel` collects the equations of motion of a system whose items are
user items with symbolic user functions,

$$
  \Mm(\qv, t)\,\ddot\qv = \fv(\qv, \dot\qv, \yv, t) - \Cm_{\qv}\tp\zv , \qquad
  \dot\yv = \gv(\qv, \dot\qv, \yv, t) , \qquad
  \cv(\qv, \dot\qv, \yv, \zv, t) = \Null ,
$$
with ODE2, ODE1 and algebraic coordinates and the data coordinates of the post-Newton steps. The symbolic system is a
replicate of `mbs`: it evaluates the same equations, and a state it computes is written back into `mbs`, where the
renderer, the sensors, the solution file and the output functions read it.

**What works**: mass points and rigid bodies (on Tait-Bryan angles) as user bodies; spring-dampers, torsional,
Cartesian and rigid-body spring-dampers and the sphere-sphere contact with friction as user connectors; revolute,
prismatic and spherical joints and distance constraints as user joints; coordinate constraints, ObjectGenericODE2,
ObjectGenericODE1 and ObjectGenericDAE; the force, torque, mass-proportional and coordinate loads. This is a small part
of the features of the C++ items. It is extended continuously and meant for experimentation, teaching and the export of
simpler models; a missing feature is easily added or patched in the Python implementation, also with an AI agent. What
a function takes and refuses is in its docstring.

**How**: the system is built with `UserItemCreator(mbs)` of `exudyn.symbolics.userItems`, whose Create functions take
the arguments of those of `mbs` (see {ref}`sec-useritemcreator`), recorded and assembled:

```python
creator = UserItemCreator(mbs)  #creator = mbs builds the same system of built-in items
...                             #creator.CreateMassPoint(...), creator.CreateSpringDamper(...), ...
creator.Record()
mbs.Assemble()
system = SymbolicSystem(mbs)
```

**What it gives**:

- `Equations()`: the mass matrix, the forces, the algebraic equations and the ODE1 equations as `exudyn.symbolic`
  expressions in the variables $\qv$, $\dot\qv$, $t$, or as sympy expressions;
- `MassMatrix`, `ODE2RHS`, `SystemResidual` and the exact `SystemJacobian`, evaluated at any state;
- `GetFlatProgram`: the equations as a flat program of scalar and vector operations in Python or C++, for an
  evaluation outside Exudyn;
- `SetSystemState`: a state written into `mbs`;
- the parameters of the user items given as expressions of named variables of the model,
  `esym.Real(name, value, scope=mbs)`, stay expressions of these variables in the equations,
  `SymbolicSystem(mbs, parameters=creator.parameterBindings)`, for derivatives by them. The evaluation takes the values
  of the parameters as an input: `SetParameterValues({'k': 650.})` sets new values without recording anything again,
  and `ODE2RHSParameterJacobian` and `MassMatrixParameterJacobian` give the derivatives of the forces and of the mass
  matrix by the parameters, in the order of `parameterNames` (#3159, #3208). Every parameter of a user item may be such an expression - the stiffness of a spring-damper,
  the radii of a contact, the matrices of a rigid-body spring-damper, a distance -, and the inertia of a rigid body:
  `RigidBodyInertia` and its shapes take symbolic values, `InertiaCuboid(density=rho, sideLengths=[L, 0.1, 0.1])`.
  The frames of joints and connectors and the loads are numbers.

The system is kept as one flat program per kind of item - the items whose user functions are the same recording -,
with the tables of their parameters and of their local-to-global indices, as Exudyn assembles its items; the global
matrices are sparse, their pattern known before any evaluation. For the evaluation, the forces and the mass matrix of
the whole system are composed into two recordings, which is several times faster than calling the recordings item by
item. How it is built is described in `docs/dev/internals/symbolicSystem.md` (on GitHub).

The solvers of {ref}`sec-symbolicsolvers` integrate the system. `NumpySystem` takes equations written by hand with the
same interface. The functions are described in {ref}`sec-module-symbolics.systemmodel`.

The following notebook builds two mass points on spring-dampers, shows their equations, integrates them with
`SolveGeneralizedAlpha` and shows the solution in `mbs`:

```{toctree}
:maxdepth: 1

/docs/generated/notebooks/tutorialSymbolicSystem
```
