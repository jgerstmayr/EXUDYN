# The flat program of expressions, and the export of recordings

The design of revision2026c step RG12.50 (#3086 the evaluation, #3087 and #3088 the implementation; maintainer
2026-10-25). A development note, not documentation: the user documentation is the reference page of exudyn.symbolic,
`exudyn.misc.symbolicExport` and docs/manual/introductionBasics.md. The symbolic model of a system of user items
(section 7 of the first version) is a separate design, written later when the symbolic steps are completed
(RG12.50.4).

## 1. The recording, and what existed

- **A recording is a graph, not a tree.** Each operation of exudyn.symbolic creates one node, an expression object
  that points to its operands; a Python variable used twice points twice to the same node.
- `str(f)`, the C++ `ToString()`, writes the graph as a tree, every shared node again at every use.
- `UserFunction.GetFlatProgram()` (#3053) writes the flat program: every node one number, its position in the order of
  evaluation, one line `tK = ...` per operation, numbers, named variables and argument components inline.
- `Diff` is a number, the forward derivative at the current values; `mbs.GetDictionary()` carries a recording as an
  object in memory (#1888).

## 2. Decided (maintainer 2026-10-25)

| decision | as built |
|---|---|
| one structure behind every output | `GetFlatProgramData()` in C++; every format written from it by Python |
| the formats | `'Python'` (the text of today), `'symbolic'`, `'numpy'`, `'sympy'`; other languages through sympy |
| `flatten` | `GetFlatProgram(..., flatten=True)`; `False` writes each result as one expression, convertible as the flat one |
| the module | `python/exudyn/misc/symbolicExport.py`; later `misc/symbolicSystem.py` and `misc/symbolicUserItems.py` beside it |
| the call | `ToString(format=...)` of esym.Real, Vector, Matrix and `GetFlatProgram(format=...)` both use `symbolicExport.Write`; `'Python'` is written by C++, and `Write` gives the same text, so either can be used |
| the speed | measured (section 5); fast enough, the C++ path for `'Python'` stays |
| arguments of an exported user function | its full Python signature |
| `exudyn.symbolic.variables` | read from there by the format `'symbolic'`, which keeps the signature and records them again; keyword arguments with their present value in `'numpy'` and `'sympy'` |
| `mod`, `round`, comparisons | computed as Exudyn computes them |
| the C++ `ToString()` | kept |
| the symbolic model of a system | later, its own design file |

## 3. The structure: `GetFlatProgramData()`

Entry k of `operations` is node k, the `tK` of the text. An operation refers to its operands by their numbers, which
are always smaller than k, so a writer is one loop. For `c = esym.cos(x); f = c*c + c*y`:

```python
{'arguments':  [{'name': 'x', 'kind': 'Real', 'value': 2.0}, {'name': 'y', 'kind': 'Real', 'value': 3.0}],
 'operations': [{'op': 'variable', 'kind': 'Real', 'name': 'x', 'value': 2.0, 'text': 'x', 'inline': True},  # 0
                {'op': 'cos', 'kind': 'Real', 'operands': [0], 'inline': False},                              # 1
                {'op': '*',   'kind': 'Real', 'operands': [1, 1], 'inline': False},                           # 2
                {'op': 'variable', 'kind': 'Real', 'name': 'y', ...},                                          # 3
                {'op': '*',   'kind': 'Real', 'operands': [1, 3], 'inline': False},                           # 4
                {'op': '+',   'kind': 'Real', 'operands': [2, 4], 'inline': False}],                          # 5
 'results':    [{'name': 'result', 'kind': 'Real', 'node': 5}]}
```

- Leaves: `'number'` (a value, a list for a vector, a list of rows for a matrix) and `'variable'` (a named variable or
  an argument); `'text'` is their C++ text, which the format `'Python'` uses. `'inline'` is the rule of
  `FlatProgram::Build`: leaves and components of arguments are written into their users.
- Operations: the name of each expression class (`FlatName()`, one line per class): `+ - * /`, the comparisons,
  `pos`, `neg`, the functions by their exudyn.symbolic names, `IfThenElse`, `vector` and `matrix` (from components,
  with `rows` and `columns`), `@`, `vector@matrix`, `multiply`, `NormL2`, `T`, `component`. The writer tells `@` of two
  vectors from the matrix products by the kinds of the operands.
- `UserFunction.GetFlatProgramData()` adds `name`, the arguments of the Python signature with their `type` and size,
  and `variables`, the named variables of `exudyn.symbolic.variables` that the recording reads.
  `CreateSymbolicUserFunction` passes the names of the signature as `argNameList`.
- `esym.GetFlatProgramData(expressions, arguments=None, resultNames=None)`: the arguments are the named variables, in
  the order given or of their first appearance; a variable the expressions use that is not in `arguments` is refused.

## 4. The formats

| format | example of `f.ToString(format)` | runs |
|---|---|---|
| `'Python'` | `((cos(x) * cos(x)) + (cos(x) * y))` | no: bare function names, as the text of today |
| `'symbolic'` | `((esym.cos(x) * esym.cos(x)) + (esym.cos(x) * y))` | yes, records the graph again |
| `'numpy'` | `((np.cos(x) * np.cos(x)) + (np.cos(x) * y))` | yes, without Exudyn |
| `'sympy'` | `((sympy.cos(x) * sympy.cos(x)) + (sympy.cos(x) * y))` | yes, needs sympy |

`GetFlatProgram(format=...)` of a user function or of expressions writes the imports and a function, its body flat or
nested:

```python
import numpy as np
def Force(mbs, t, itemNumber, q, q_t):
    t3 = np.cos(q[0])
    ...
    return t50
```

What a format computes differently from its own functions is written as Exudyn computes it:

| exudyn.symbolic | numpy | sympy |
|---|---|---|
| `mod(x, y)`, C `fmod` | `np.fmod` | `x - y*sign(x/y)*floor(Abs(x/y))` (`Mod` has the sign of y, `sympy.trunc` truncates polynomials) |
| `round(x)`, halves away from zero | `trunc(x) + sign(x)*(abs(x - trunc(x)) >= 0.5)` (`np.round` rounds halves to even) | `sign(x)*floor(Abs(x) + 1/2)` |
| a comparison, 0 or 1 | `1.0*(x > y)` | `Piecewise((1, Gt(x, y)), (0, True))` |
| `IfThenElse(c, a, b)` | `a if c else b` | `Piecewise((a, Ne(c, 0)), (b, True))` |
| `Not(x)`, `isfinite(x)` | `1.0*(x == 0)`, `1.0*np.isfinite(x)` | `Piecewise(...)`; isfinite refused, a symbol is finite |
| `@` of two vectors, `v @ M`, `multiply`, `NormL2` | as written | `a.dot(b)`, `M.T @ v`, `multiply_elementwise`, `norm()` |

`symbolicExport.ToFunction(data, format)` gives the function itself, `ToSympy(data)` the sympy expressions with a symbol
per argument (`q[0]`, `q[1]`, ... of a vector as a column `sympy.Matrix`), from which `sympy.lambdify`, `ccode`,
`sympy.cse` or `diff` go on.

Two operations exist in C++ but cannot be reached from Python and are therefore untested: the unary `+` of a vector or
matrix and `==`/`!=` of two vectors, which exudyn.symbolic refuses (#3015). The writer has them.

## 5. The speed of the translation (measured 2026-10-25, Windows workstation, Python 3.13)

A vector of n/6 independent terms `sin(q[0]*q[1] + c)*q[2]`, entries per second:

| entries | C++ `GetFlatProgram()` | `GetFlatProgramData()` | `Write('Python')` | `Write('numpy')` |
|---|---|---|---|---|
| 1828 | 4.8 M/s | 1.4 M/s | 1.7 M/s | 1.1 M/s |
| 18328 | 4.6 M/s | 1.4 M/s | 1.9 M/s | 1.2 M/s |
| 183328 | 3.8 M/s | 1.1 M/s | 1.7 M/s | 1.1 M/s |

The structure and the writer together are about 0.7 M entries per second, five to six times slower than the C++ text;
the 300 operations of userRigidBodyTest.py take half a millisecond. The C++ text stays the path of `'Python'`, and as
the writer gives the same text, a format could move to C++ again without a change of its result.

## 6. Found, and planned separately

- **IfThenElse is a leaf of the flat program** (#3089, RG12.49.3): it has no operand interface, so the flat evaluation
  computes it as a tree - nodes it shares with the rest are computed again - and `GetFlatProgram()` writes it inline as
  a whole tree. The structure gives it its three operands, so the writer's `'Python'` text differs from the C++ text
  only for a recording with IfThenElse.
- **Deep recordings overflow the C++ stack** (#3090, RG12.49.4): `str()` of about 32000 nested nodes, the recording of
  about 120000; the tree functions are recursive, the flat program and the structure are not.
