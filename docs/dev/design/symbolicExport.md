# The flat program of expressions, and the export of recordings and of a model of user items

The evaluation and proposal of revision2026c step RG12.50.1 (#3086, maintainer 2026-10-25). A development note, not
documentation.

## 1. What exists

- `UserFunction.GetFlatProgram()` (#3053) writes the list of a **recorded user function** as text, one line per
  operation, numbers and argument components written inline:

  ```
  t3 = cos(q[2])
  t4 = sin(q[2])
  t5 = (-t4)
  t11 = [[t3, t5, 0], [t4, t3, 0], [0, 0, 1]]
  t12 = (t11 @ q)
  result = t12
  ```

  It is nearly Python already: with `from numpy import cos, sin` and `q` an array, most lines run as they are; `@` of
  nested lists, `IfThenElse`, `mod` and `sign` do not.
- An expression that is not recorded as a user function - `x = esym.Real('x', 1.)`, `y = esym.sin(x)*x` - has
  `str(y)` (the tree as text, every shared node written again) but no flat program: `FlatProgram::Build` takes any
  roots, it is only not bound to Python for them.
- `Diff` of exudyn.symbolic is a **number**, the forward derivative at the current values; there is no derivative as an
  expression.
- `mbs.GetDictionary()` carries recorded user functions as objects in memory (#1888), not as a file.

## 2. The flat program of expressions

No general function concept is needed: a module function

```python
esym.GetFlatProgram(expressions, names=None)   # one esym.Real, Vector or Matrix, or a list of them
```

builds the program for these roots and returns the text - the same `Build` and `ToString` as for a user function, the
named variables written by their names. Shared nodes between the expressions are computed once, as in a user
function. About 30 lines in Symbolic.cpp; with RG12.49 the optimized program.

## 3. Exporting the evaluation graph: the format

| format | for | assessment |
|---|---|---|
| the text of today | reading, diffs, documentation | stays; not meant to be parsed |
| **a Python structure** `GetFlatProgramData()`: a list of operations `{'op': 'cos', 'operands': [3], 'kind': 'Real'}`, the leaves with `'value'` or `'name'`/`'argument', 'component'`, and the results | every other format is written from it in Python; `json.dump` gives a file | **proposed** |
| Python source (numpy) | running a recording without Exudyn, checking | a writer of 50 lines over the structure |
| sympy expressions | simplifying, differentiating, `sympy.cse`, `lambdify`, `ccode`, LaTeX | a writer of 60 lines; sympy optional, imported at the point of use (CLAUDE.md rule 6) |
| CasADi, C code | optimal control, embedded controllers | CasADi writer like sympy; C code through sympy's `ccode` or `codegen` |
| an own file format, XML, ONNX | - | not proposed: nothing reads it, and the Python structure plus JSON already is a file |

C++ part: each of the about 60 expression classes gives the name of its operation (`virtual const char* FlatName()`;
for the 21 vector and matrix classes in the generated interface, for the 39 scalar classes one line each), and the
leaves their value or name. The writers live in Python, in `exudyn.symbolicUtilities` (or a module
`symbolicExport.py` beside it), and map the names: `IfThenElse` → `sympy.Piecewise`, `mod` → `Mod`, `min`/`max` →
`Min`/`Max`, `atan2`, `sign`, the comparisons; numpy-equivalent `@` and `.T` on `sympy.Matrix`.

The **round trip** is the test: each writer's result is evaluated (numpy, `sympy.lambdify`) at random arguments and
compared with `UserFunction.Evaluate`, for all expression classes - the cases of test_symbolicFlatEvaluation.py.

## 4. The symbolic model of a system of user items

Goal: from an `mbs` built only of user items with symbolic user functions, the equations of motion as one symbolic
program - $\Mm(\qv)\,\ddot\qv = \fv(\qv,\dot\qv,t) - \Cm_q\tp\,\lambdav$, $\gv(\qv,t) = \mathbf{0}$ - for control,
linearization, code generation.

`symbolicUtilities.SymbolicSystem(mbs)`, after `mbs.Assemble()`:

1. the global coordinates: `q[0..n-1]`, `q_t`, `t`, from the local-to-global maps of the objects
   (`GetObjectLTGODE2` and the ODE1 and AE equivalents);
2. per object, its recorded user functions exported as in section 3, with the argument leaves renamed - `q[i]` of the
   object becomes `q[ltg[i]]` - and their results added at the global places: the mass matrices into $\Mm$, the forces
   into $\fv$, the constraints into $\gv$;
3. the result as the Python structure of section 3, written to sympy, numpy or text by the same writers.

**What can be assembled, and what not:**

| item | how |
|---|---|
| ObjectGenericODE2, ObjectGenericODE1 with symbolic mass, force, Jacobian functions | directly |
| ObjectUserBody (and NodeUser) | its mass matrix and force functions in its coordinates; positions of points by its positionUserFunction |
| ObjectConnectorUser, ObjectJointUser, MarkerUser | the force or constraint is symbolic, but the **marker Jacobians are computed in C++** (numerically for MarkerUser), and exudyn.symbolic has no derivative as an expression: assembled through sympy, which differentiates the exported marker positions and rotations |
| built-in items (ObjectRigidBody, springs, ...) and Python (not symbolic) user functions | refused, with the list of items; the built-in items need their recording (the AD templates, #3048) - not proposed now |

So the assembly is built in two parts: (a) the export and the global renaming, without sympy - enough for systems of
generic ODEs and user bodies; (b) the connectors through sympy, where the derivatives come from.

## 5. Proposed sub-steps

- RG12.50.2: `esym.GetFlatProgram(expressions)` for expressions (section 2).
- RG12.50.3: `GetFlatProgramData()`, the operation names, the writers to Python source and sympy, the round-trip test
  (section 3).
- RG12.50.4: `SymbolicSystem(mbs)` for generic objects and user bodies, a test model (a double pendulum of two
  ObjectGenericODE2, exported to sympy, linearized, compared with the simulation) (section 4a).
- RG12.50.5: the connectors and joints through sympy (section 4b).

## 6. Decision needed

1. The Python structure as the export format, sympy (optional) as the first target - or text only.
2. Where the writers go: `symbolicUtilities.py` or a new `symbolicExport.py`.
3. Whether section 4b (connectors through sympy) is in scope, or only 4a.
