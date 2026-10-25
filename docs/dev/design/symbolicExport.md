# The flat program of expressions, and the export of recordings and of a model of user items

The evaluation and proposal of revision2026c step RG12.50.1 (#3086, maintainer 2026-10-25; refined with the
maintainer's questions of the same day). A development note, not documentation. Section 8 lists the decisions.

## 1. What exists

- **A recording is a graph, not a tree.** Each operation of exudyn.symbolic creates one node, an expression object
  that points to its operands. A Python variable used twice points twice to the same node:

  ```python
  x = esym.Real('x', 2.); y = esym.Real('y', 3.)  # named variables: leaves that keep their name and can change
  c = esym.cos(x)
  f = c*c + c*y                                   # 6 nodes: x, y, cos, c*c, c*y, +; the node cos is used 3 times
  ```

- **`str(f)`**, the C++ `ToString()`, writes the graph as a tree: `((cos(x) * cos(x)) + (cos(x) * y))`, with every shared
  node written again at every use.
- **`UserFunction.GetFlatProgram()`** (#3053) writes the flat program of a recorded user function. The flat program
  gives every node one number, its position in the order of evaluation, and writes one line `tK = ...` per operation;
  numbers, named variables and argument components are written inline:

  ```
  t3 = cos(q[2])
  t4 = sin(q[2])
  t5 = (-t4)
  t11 = [[t3, t5, 0], [t4, t3, 0], [0, 0, 1]]
  t12 = (t11 @ q)
  result = t12
  ```

  This text is nearly Python. It is not executable, for three reasons: the function names are bare (`cos`, `pow`), the
  arguments (`q`) are not declared, and a few functions mean something other than their namesakes in numpy or sympy
  (section 5).
- **Recording is on by default** (`esym.GetRecording()` is True), so `f` above is recorded as soon as it is written.
- **`Diff` is a number**: the forward derivative at the current values, not an expression.
- **A recorded user function knows its arguments**: `CreateSymbolicUserFunction` passes their names and types
  (`argList`, `argTypeList`: `t` a float, `q` a vector of a given size, `mbs` and `itemNumber`), and the type of its
  result.
- **`mbs.GetDictionary()`** carries a recorded user function as an object in memory (#1888), not as a file.

## 2. One structure behind every output

Proposed: **C++ provides only one thing, the flat program as a Python structure.** Every output - the text of today,
executable code in each format, the tree form of `str()` - is written from this structure by one writer in Python. So
`ToString` of an expression and of a flat program work the same way and share the code; a new format is one table in
Python, not a change in 60 C++ classes.

### 2.1 The structure: `GetFlatProgramData()`

It is the list of the flat program: **entry k is node k**, the `tK` of the text. An operation refers to its operands by
their numbers, and these are always smaller than k, because the operands are evaluated first. A writer is therefore one
loop over the list. For `f` of section 1:

```python
{'arguments':  [{'name': 'x', 'kind': 'Real', 'value': 2.0},       # the leaves that become the parameters
                {'name': 'y', 'kind': 'Real', 'value': 3.0}],
 'operations': [{'op': 'argument', 'kind': 'Real', 'name': 'x'},          # 0
                {'op': 'cos',      'kind': 'Real', 'operands': [0]},      # 1  used by 2 (twice) and by 4
                {'op': '*',        'kind': 'Real', 'operands': [1, 1]},   # 2
                {'op': 'argument', 'kind': 'Real', 'name': 'y'},          # 3
                {'op': '*',        'kind': 'Real', 'operands': [1, 3]},   # 4
                {'op': '+',        'kind': 'Real', 'operands': [2, 4]}],  # 5
 'results':    [{'name': 'result', 'kind': 'Real', 'node': 5}]}
```

- Leaves: `{'op': 'number', 'value': 4.0}`; vector and matrix numbers with a list as value; a named variable or an
  argument as `'argument'` with its name; a component as an operation `'component'` with the vector and the index
  (`q[1]`) as operands, as in the C++ classes. `'kind'` is `Real`, `Vector` or `Matrix`, with `'size'` where it is
  known.
- The operation names are those of exudyn.symbolic and its operators (`cos`, `pow`, `IfThenElse`, `mod`, `@`, `T`, ...).
  In C++ each expression class gives its name (`virtual const char* FlatName()`), about 60 one-line functions.
- `json.dump` of the structure is a file that is read without Exudyn.

### 2.2 The tree and the flat program from the same structure

The writer has two layouts:

- **flat**: one line per operation, as `GetFlatProgram()` today; a node used several times is computed once;
- **nested**: one expression, every operation written into its users; this is the tree of `str(f)`, with the
  shared nodes written again.

`str(f)` of today is the nested layout of the plain format. The test of the writer is that it reproduces `str()` and
`GetFlatProgram()` of today character by character, for all cases of test_symbolicFlatEvaluation.py. After that the
C++ `ToString()` may stay as it is (it is used for error messages) or be replaced by the writer.

## 3. The arguments of an executable program

Executable code needs a header and a return as well as the operations. Both come from the structure (`arguments`,
`results`), so no separate function is needed to export them:

- **of a user function**: the arguments of its Python signature, in that order - `def F(mbs, t, itemNumber, q, q_t)`.
  With the same signature, the exported function is a drop-in replacement for the Python user function, and the test
  compares the two directly. Unused arguments stay in the signature.
- **of expressions**: the named variables they depend on, in the order of their first appearance, or in the order
  given (below).
- **variables of `esym.variables` (`VariableSet`)** that a recording reads: keyword parameters with their current
  value as default - `def F(mbs, t, itemNumber, q, q_t, k=100.)` - so they stay changeable.

### 3.1 `GetFlatProgram` and `GetFlatProgramData` of expressions

```python
esym.GetFlatProgram(expressions, arguments=None, resultNames=None, format='plain', name='F')
esym.GetFlatProgramData(expressions, arguments=None, resultNames=None)
```

- **Input**: `expressions`, one `esym.Real`, `Vector` or `Matrix` or a list of them - the results. `arguments`, an
  optional list of the named variables (`[x, y]`) that become the parameters, in this order; it is checked that the
  expressions depend on no other named variables, and a listed variable that is not used is allowed. `resultNames`
  names the results.
- **Output**: the text in the format, or the structure.

Example, with `x`, `y` and `f` of section 1:

```python
e = x**2 + 4*x + y
esym.GetFlatProgram(e)                      # plain, as GetFlatProgram() of a user function today
```
```
t2 = pow(x, 2)
t4 = (4 * x)
t5 = (t2 + t4)
t6 = (t5 + y)
result = t6
```
```python
esym.GetFlatProgram([e, f], arguments=[x, y], resultNames=['e', 'f'], format='numpy', name='EF')
```
```python
import numpy as np
def EF(x, y):
    t2 = x**2
    t4 = 4*x
    t5 = t2 + t4
    t6 = t5 + y
    t8 = np.cos(x)
    t9 = t8*t8
    t11 = t8*y
    t12 = t9 + t11
    return t6, t12          # e, f
```

The node numbers are illustrative. The program of `e` and `f` together computes a node they share once.

## 4. The formats

`ToString(format=...)` on `esym.Real`, `Vector`, `Matrix` (nested layout by default) and `GetFlatProgram(format=...)` on
`UserFunction` and `esym` (flat layout by default) both call the same Python writer: `write(data, format, layout,
name)`. In C++, `ToString(format)` builds the data and calls the writer, imported from Python, for every format except
`'plain'`.

| format | the result | executable | for |
|---|---|---|---|
| `'plain'` | the text of today | no | reading, diffs, error messages |
| `'exudyn.symbolic'` | Python with `esym.` functions; called with symbolic arguments, it records the same graph again | yes | storing a recording as a .py file and loading it again; a round trip |
| `'numpy'` | Python with `np.` functions, arguments numpy arrays | yes, without Exudyn | running and checking a recording outside Exudyn |
| `'sympy'` | Python that builds sympy expressions, `sympy.Matrix` for vectors and matrices | yes, needs sympy | section 6 |

## 5. Functions that do not mean the same in every format

These functions must be written so that every format computes what Exudyn computes (`EM::` musl and `std::`,
docs/manual of exudyn.symbolic, #3074):

| exudyn.symbolic | numpy | sympy | C / C++ (through sympy) |
|---|---|---|---|
| `mod(x, y)` = C `fmod`, the sign of `x` | `np.fmod` (not `%`) | `x - y*trunc(x/y)` (sympy `Mod` has the sign of `y`) | `fmod` |
| `round(x)` = C `round`, halves away from zero | `np.copysign(np.floor(np.abs(x) + 0.5), x)` (`np.round` rounds halves to even) | `sign(x)*floor(abs(x) + 1/2)` | `round` |
| a comparison is the number 0 or 1, `(x > y) + 1` | `float(x > y)`, `np.where` for arrays | `Piecewise((1, x > y), (0, True))` | `(x > y)` |
| `IfThenElse(c, a, b)` | `a if c else b`, `np.where` for arrays | `Piecewise((a, c), (b, True))` | `c ? a : b` |
| `min`, `max`, `abs`, `sign`, `atan2` | `np.minimum`, `np.maximum`, `np.abs`, `np.sign`, `np.arctan2` | `Min`, `Max`, `Abs`, `sign`, `atan2` | the same names |
| `isfinite`, `Not` | `np.isfinite`, `not` | refused: a symbol is always finite; `Not` as `Piecewise` | `isfinite`, `!` |
| `@`, `.T`, `v[i]`, `M[i, j]` | numpy arrays | `sympy.Matrix` | expanded to scalars by sympy |

A refused function names itself and the format in the error.

## 6. Converting to other code without Exudyn: sympy as the common structure

sympy has what is asked for, so a conversion to sympy is enough for most targets:

- printers from one sympy expression to C (`ccode`), C++ (`cxxcode`), Fortran, Rust, Julia, Octave/Matlab, JavaScript,
  LaTeX, and Python for numpy, JAX or TensorFlow (`lambdify`);
- `sympy.cse` makes a flat program again, and `sympy.codegen` writes whole C functions with header and arguments;
- `diff`, `jacobian`, `simplify` and `series` for the linearization and the marker Jacobians (section 7).

What to know about sympy:

- **It simplifies while building**: `0*x` becomes `0`, `x + x` becomes `2*x`, and the operands of sums and products are
  reordered. The result is mathematically equal but not equal bit for bit: the order of floating-point additions
  changes. Tests compare with a tolerance.
- **It is an optional dependency** (CLAUDE.md rule 6), imported where it is used; the formats `'plain'`,
  `'exudyn.symbolic'` and `'numpy'` and the structure need no sympy.
- **Its time** grows with the size of the expressions. The graphs of the test models (300 operations) are small for
  sympy; a system of many user items is measured in RG12.50.4.

Not proposed: an own file format, XML or ONNX. Nothing reads them, and the JSON of the structure is already a file.
CasADi, which control engineers use, would be a fifth writer over the same structure if it is wanted.

## 7. The symbolic model of a system of user items

Goal: from an `mbs` built only of user items with symbolic user functions, the equations of motion as one symbolic
program - $\Mm(\qv)\,\ddot\qv = \fv(\qv,\dot\qv,t) - \Cm_q\tp\,\lambdav$, $\gv(\qv,t) = \mathbf{0}$ - for control,
linearization and code generation.

`SymbolicSystem(mbs)`, after `mbs.Assemble()`:

1. takes the global coordinates `q[0..n-1]`, `q_t` and `t` from the local-to-global maps of the objects
   (`GetObjectLTGODE2`, `GetObjectLTGODE1`, `GetObjectLTGAE`);
2. takes the structure of each object's user functions, renames its argument leaves (`q[i]` of the object becomes
   `q[ltg[i]]`), and adds its results at their global places: the mass matrices into $\Mm$, the forces into $\fv$, the
   constraints into $\gv$;
3. returns one structure with the arguments `t, q, q_t` and the results `M, f, g`, written by the same writers.

| item | how |
|---|---|
| ObjectGenericODE2, ObjectGenericODE1 with symbolic mass, force and Jacobian functions | directly |
| ObjectUserBody (and NodeUser) | its mass matrix and force functions in its coordinates; the positions of points by its positionUserFunction |
| ObjectConnectorUser, ObjectJointUser, MarkerUser | the force or constraint is symbolic, but the **marker Jacobians are computed in C++** (numerically for MarkerUser), and exudyn.symbolic has no derivative as an expression: through sympy, which differentiates the exported marker positions and rotations |
| built-in items (ObjectRigidBody, springs, ...) and Python user functions | refused, with the list of items; the built-in items would need a recording of their own (the AD templates, #3048), not proposed now |

So it is built in two parts: (a) the structure and the global renaming, without sympy, which is enough for generic
objects and user bodies; (b) the connectors through sympy, which provides the derivatives.

## 8. Decisions and recommendations

| # | decision | recommended | alternatives |
|---|---|---|---|
| 1 | what C++ provides | **one structure, `GetFlatProgramData()`**; every format and both layouts written from it in Python | formats written in C++, per class and per format |
| 2 | the call of the formats | **`ToString(format=...)` on expressions and `GetFlatProgram(format=...)`**, both calling the same Python writer; `'plain'` stays C++ | one Python function only, `symbolicExport.Write(x, format)` |
| 3 | the formats | **`plain`, `exudyn.symbolic`, `numpy`, `sympy`**; every other language through sympy's printers | also C++ directly, without sympy |
| 4 | the arguments of an exported user function | **its full Python signature** (drop-in, comparable with the Python function) | only the arguments the recording uses |
| 5 | VariableSet variables | **keyword parameters with their current value as default** | constants written into the code |
| 6 | exact meaning of `mod`, `round`, comparisons | **written to compute what Exudyn computes** (section 5) | the native function of each format |
| 7 | the C++ `ToString()` of today | **kept**; the writer is tested against it and may replace it later | replaced at once |
| 8 | where the writers go | **a new module `exudyn/symbolicExport.py`**, used by symbolicUtilities | `symbolicUtilities.py` |
| 9 | scope of the system model | **7a now** (generic objects, user bodies); 7b (connectors through sympy) after it | both at once; 7a only |

Sub-steps if recommended as above:

- RG12.50.2: `GetFlatProgramData()` (C++, the operation names) and `esym.GetFlatProgramData`/`GetFlatProgram` for
  expressions; the writer of `'plain'`, tested against `str()` and `GetFlatProgram()` of today.
- RG12.50.3: the formats `'exudyn.symbolic'`, `'numpy'` and `'sympy'`, `ToString(format=...)`, the round trip of every
  operation (each format evaluated at random arguments against `UserFunction.Evaluate`).
- RG12.50.4: `SymbolicSystem(mbs)` for generic objects and user bodies, a test model (a double pendulum of two
  ObjectGenericODE2, exported to sympy, linearized, compared with the simulation).
- RG12.50.5: the connectors and joints through sympy.
