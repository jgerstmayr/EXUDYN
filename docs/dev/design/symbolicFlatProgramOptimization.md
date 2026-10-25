# Optimizing symbolic expressions: simplification and merging of duplicates

The evaluation and proposal of revision2026c step RG12.49.1 (#3085, maintainer 2026-10-25; revised with the
maintainer's refinements of the same day: the optimization belongs to the expressions, a function with levels, merging
of the leaves). A development note, not documentation. Section 7 lists the decisions.

## 1. What exists

- **A recording is a graph.** Every operation creates one node, which points to its operands. A Python variable used
  twice points twice to the same node.
- **The flat program** (`FlatProgram::Build` in `src/Linalg/Symbolic.h`, #3053, #3054) gives every node one slot,
  found by its **pointer**. A node used several times is computed once per evaluation. Nothing is kept between two
  evaluations, so D18 is not touched (#3048).
- **Nodes are equal only by pointer, also in the flat program.** So "comparison of duplicates" is not done today, at
  any depth. Measured with `GetFlatProgram()`:

  ```python
  a = esym.sin(q[1]+3)*esym.cos(q[2]-q[0])
  b = esym.sin(q[1]+3)*esym.cos(q[2]-q[0])     # the same text again
  return esym.Vector([a, b, a])
  ```
  ```
  t4 = (q[1] + 3)
  t5 = sin(t4)
  t10 = (q[2] - q[0])
  t11 = cos(t10)
  t12 = (t5 * t11)
  t16 = (q[1] + 3)        <- b: all five operations again
  t17 = sin(t16)
  t22 = (q[2] - q[0])
  t23 = cos(t22)
  t24 = (t17 * t23)
  t25 = [t12, t24, t12]   <- a, used twice, is computed once
  ```

  This is 11 operations, where 6 are needed. The slot numbers show the leaves too: the argument `q` is one node, but every
  `q[1]` is a new component node with a new number leaf `1` as its index, and every `3` is a new number leaf.
- **Numbers are folded already, when recording.** An operation whose operands are all plain numbers (an `SReal` without
  an expression, such as `esym.Real(13)`) is computed instead of recorded: `esym.sin(esym.Real(13))*q[2]` becomes
  `(0.420167 * q[2])`. Named variables (`esym.Real('k', 13)`, `esym.variables`) and arguments can change and stay
  symbolic.
- **Nothing is simplified**: `pow(q[0], 0)`, `pow(q[1], 1)` and `(0 * q[2])` are recorded and computed as written.

## 2. Measured

| recording | operations | trivial (`0*x`, `1*x`, `x+0`) | duplicates |
|---|---|---|---|
| the rotation matrix with `functions.cos(rot[2])` written twice | 11 | 0 | 3 (8 operations when the values are shared by hand) |
| `sin(q[1]+3)*cos(q[2]-q[0])` written twice, above | 11 | 0 | 5 |
| `np.array(3x3 diagonal) @ np.array([q0,q1,q2], dtype=object)` | 16 | 7 | 3 |
| the same product as a Python sum over lists | 19 | 10 | 4 |
| `su.RotationMatrixZ(q[2], functions=esym) @ q` | 5 | 0 | 0 |
| userRigidBodyTest.py, the 6 symbolic functions | 314 | 0 | 63 (20 %) |

An operation counts as a duplicate if an operation of the same class, with operands that are themselves already
replaced by their first occurrence, appeared before. The count therefore covers any depth: the `sin(...)*cos(...)`
above is 5 duplicates, its sum, `sin`, difference, `cos` and product.

- **Trivial operations** come from Python: numpy's product and `sum()` of object arrays multiply by every zero of a
  constant matrix. Products of `esym.Vector` and `esym.Matrix` are one node each, computed by the dense C++ product.
- **Duplicates** come from writing the same expression twice, which the symbolic rigid body, written for clarity, does
  for a fifth of its operations.

## 3. Proposal: optimizing an expression, `esym.Optimize`

The optimization belongs to the expressions, as the maintainer suggested. It **builds a new graph from an existing
one**, and the flat program is built from the new graph exactly as today, unchanged.

```python
expression = esym.Optimize(expression, level=1, mergeDuplicates=True)
[e1, e2]   = esym.Optimize([e1, e2], level=1, mergeDuplicates=True)   # duplicates merged across the results too
```

- **Input**: one `esym.Real`, `Vector` or `Matrix`, or a list of them. **Output**: the same, as new expressions. The
  input is unchanged.
- The new graph shares the parts that do not change with the old graph; nodes are reference counted. `str()`, `Diff`,
  `Evaluate`, the tree evaluation and the flat program work on it as on any recording.
- Users can therefore look at the result at the level of the expressions - `str(esym.Optimize(e))` against `str(e)` -
  without a user function or a flat program, as asked.

### 3.1 How it works: one pass from the leaves up

`Optimize` visits the graph with the operands before their operation, as `FlatProgram::Build` does, without recursion,
and keeps a map from old node to new node. For each node:

1. take the new operands, which are already optimized and merged;
2. apply the rules of the level (3.2); if a rule applies, the result is an operand or a number, and no node is created;
3. if `mergeDuplicates`, look up the **key** of the node in a hash map; if an equal node exists, take it, otherwise
   create the node and enter it:
   - an operation: its class (`std::type_index(typeid(*node))`) and the pointers of its new operands. For `+`, `*`,
     `==`, `!=`, `min` and `max` the two operands are sorted, so `a*b` and `b*a` merge; this is exact, as IEEE `+` and `*`
     are commutative;
   - a number: its bits; a vector or matrix number: its size and bits;
   - a named variable and an argument: its pointer. Two variables with the same name are different;
   - every expression class keeps all of its data in its operands today (the index of `q[1]` is an operand too), so the
     class and the operands are the whole key. A class that gets data outside its operands later overrides a virtual
     `OptimizeKey()`.

The cost is one hash lookup per node, linear in the size of the graph.

**`mergeDuplicates` as a level of comparison depth is not needed.** Because the operands are merged before their
operation, comparing one level - the class and the operand pointers - compares the whole subexpression, at any depth:
the second `sin(q[1]+3)*cos(q[2]-q[0])` above merges completely with the first. A depth limit could only merge less, at
the same cost. A bool is enough.

**Merging the leaves**: with `mergeDuplicates=True`, every `q[1]` becomes one node, because its operands, the argument
`q` and the number `1`, are equal. All further nodes refer to it. This makes sense, and it is what lets the
operations above it merge. A component costs little to compute, so the gain is in the operations above it.

### 3.2 The levels

A rule applies only where an operand is a **number**: a number leaf, or what a rule has made a number. It never applies
to a named variable, which can change.

| level | rule | result the same as without the rule |
|---|---|---|
| 0 | none (only `mergeDuplicates`) | - |
| 1 | `1*x`, `x*1`, `x/1`, `+x`, `x**1`, `-(-x)` → `x` | yes, bit for bit |
| 1 | `x**0` → `1` | yes: C `pow(x, 0)` is 1 for every x, also `nan` |
| 1 | an operation whose operands are all numbers → the number, computed by the same function | yes |
| 1 | `IfThenElse` of a number condition → its branch; a component of a vector number or of `esym.Vector([a, b, c])` → that component | yes |
| 1 | `x+0`, `0+x`, `x-0` → `x` | yes, except that `-0 + 0` gives `+0`, which nothing in Exudyn distinguishes |
| 2 | `0*x`, `x*0`, `0/x` → `0` | **no**: `0*inf` and `0*nan` are `nan`; with the rule a `nan` of `x` disappears |
| 2 | `x-x` → `0`, `x/x` → `1` (with merged duplicates, also where `x` was written twice) | no, for the same reason |
| 2 | numbers of products and sums combined: `2*(3*x)` → `6*x` | no: the rounding changes in general (`0.1*(0.2*x)` against `0.02*x`) |

Level 1 is exact, so it is always safe to apply. Level 2 changes a result only where `x` is `inf` or `nan`, or in the
last bit. Merging duplicates is exact at every level: a merged node computes the same function of the same operands.

Vectors and matrices: their nodes are merged with the same key. At level 1, `M.T.T` → `M`. Skipping the zeros inside
the dense products (of a constant matrix) is a different optimization and is not proposed.

## 4. For user functions

`CreateSymbolicUserFunction(..., optimize=...)` applies `Optimize` to the recorded result before it is stored. The
flat program is then built from the optimized graph. Proposed default: `level=1, mergeDuplicates=True`, the same as for
`Optimize`. Both are exact, so the results of all test models stay the same, and
test_symbolicFlatEvaluation.py still compares the tree and the flat evaluation bit for bit.

Expected, from the table of section 2: userRigidBodyTest.py 314 → about 250 operations; the rotation matrix 11 → 8;
the duplicated `sin(...)*cos(...)` 11 → 6; the numpy product 16 → 3 at level 2, and 16 → 12 at level 1, where only the
`1*q0` and the duplicates go.

The tree evaluation (`flatEvaluation=False`) gains from the rules but not from merging, because it evaluates a shared
node at every use.

## 5. Impact on the implementation

- **New**: `Optimize` in `src/Linalg/Symbolic*.h`/`.cpp`, about 200 lines - the pass, the key map and the rules; the
  binding with its overloads for Real, Vector, Matrix and lists.
- **Expression base classes**: a virtual `bool IsNumber(Real& value)` (true for the number leaves); a virtual
  `OptimizeKey()` with the default of 3.1; a way to create a node of the same class with new operands
  (`virtual ExpressionBase* NewWithOperands(...)`). The last one is the only addition to every one of the about 60
  classes: one line each, generated for the 21 vector and matrix classes as their operand interface already is.
- **Not changed**: the recording, `FlatProgram`, `Diff`, `ToString`, the evaluation.
- **Thread safety (RG7.8.3)** is not affected.
- **Tests**: a test model `symbolicOptimizeTest.py` in python/TestModels (a user-facing function, WORKFLOW.md §5) -
  the counts of the cases of section 2 per level, `Evaluate` of each optimized expression against the original
  (the same bits at level 1, `nan` cases at level 2), `str()` of small cases; userRigidBodyTest.py with its reference
  unchanged, and its time with 10000 steps before and after.

## 6. What it leaves to the export (RG12.50)

An optimized graph is exported like any other (symbolicExport.md). sympy simplifies on its own; the
`'exudyn.symbolic'` and `'numpy'` formats get the shorter program.

## 7. Decisions and recommendations

| # | decision | recommended | alternatives |
|---|---|---|---|
| 1 | where the optimization lives | **on the expressions**: `esym.Optimize` builds a new graph; the flat program is unchanged | in `FlatProgram::Build` only |
| 2 | the name | **`Optimize`** (CamelCase like `IfThenElse`, `GetRecording`; "simplify" means algebraic simplification in sympy, which this is not) | `Simplify`, `optimize` |
| 3 | the arguments | **`Optimize(expressions, level=1, mergeDuplicates=True)`**, expressions one or a list | further options per rule |
| 4 | `mergeDuplicates` | **a bool**: merging the operands first compares any depth at no extra cost | an integer depth |
| 5 | merging the leaves | **yes**: equal components (`q[1]`) and numbers become one node | operations only |
| 6 | `x+0` → `x` | **level 1** (only the sign of a zero changes) | level 2 |
| 7 | level 2 | **`0*x`, `0/x`, `x-x`, `x/x`, combining numbers** | without combining numbers |
| 8 | user functions | **`CreateSymbolicUserFunction(..., optimize=...)`**, by default `level=1, mergeDuplicates=True` | off by default |

## 8. Built (RG12.49.2, #3085; maintainer 2026-10-25: as recommended)

- `esym.Optimize(expressions, level=1, mergeDuplicates=True)` and `CreateSymbolicUserFunction(..., optimize=True)`
  (a dict passes the arguments of Optimize, False records as before), with the recommendations 1-8 of section 7.
- C++: `Optimizer` in Symbolic.cpp, one pass without recursion; `NewWithOperands()` in each expression class, one
  line, the copy with new operands; the nodes created and not used are freed at the end (each held once more while
  the list is freed in reverse order, so that no node is deleted by its user before its own turn). The numbers created
  keep the reference counting of the other nodes (ExpressionReal counts its first user in its constructor).
- The rules as in section 3.2. Measured (symbolicOptimizeTest.py): the rotation matrix 11 → 8, the duplicated
  `sin*cos` 11 → 6, the numpy product 16 → 13 (level 0, merging only) → 12 (level 1) → 3 (level 2); userRigidBodyTest.py
  314 → 227 operations, with its reference unchanged; all 215 test models unchanged with optimize on by default.
- Time: a rigid-body force with `RotXYZ2GLocal` written three times, ObjectGenericODE2, 200000 explicit steps:
  0.31 s → 0.15 s, 56 → 33 operations, the same coordinates.
- IfThenElse got the operand interface in the same change (#3089, RG12.49.3), so that Optimize and the flat program
  see its branches.
- Found: a named `esym.Matrix` is never freed (#3092, RG12.49.5).
