# Simplifying the flat program and merging equal subexpressions

The evaluation and proposal of revision2026c step RG12.49.1 (#3085, maintainer 2026-10-25). A development note, not
documentation. The flat program is the list of operations built from a recorded symbolic user function
(`FlatProgram::Build` in `src/Linalg/Symbolic.h`, #3053, #3054); `UserFunction.GetFlatProgram()` gives it as text.

## 1. What the flat program does today

- `Build` walks the recorded graph depth first and gives every **node** one slot, found by its pointer
  (`slotOfNode`). A node used twice - a Python variable `c2 = esym.cos(q[2])` used in four places - is computed
  once per evaluation. This is the reuse inside one evaluation that #3048 recommended; D18 is not touched, as nothing
  is kept between two evaluations.
- **Numbers are folded already, when recording.** An operation whose operands are all plain numbers (an `SReal` without
  an expression, as `esym.Real(13)`) is not recorded, but computed: `esym.sin(esym.Real(13))*q[2]` is recorded as
  `(0.420167 * q[2])`. What cannot be folded: a named variable (`esym.Real('k', 13)`, a `VariableSet` entry), which may
  be changed by `SetValue`, and the arguments of the user function.
- **Nothing else is simplified, and nodes are equal only by pointer.**

## 2. Measured (1.13.190, `GetFlatProgram()` of the recordings)

| recording | operations | of them trivial (`0*x`, `1*x`, `x+0`) | duplicates of an earlier operation |
|---|---|---|---|
| the rotation matrix of the question, `functions.cos(rot[2])` written twice | 11 | 0 | 3 (8 when the values are shared by hand) |
| `np.array(3x3 diagonal) @ np.array([q0,q1,q2], dtype=object)` | 16 | 7 | 3 |
| the same product as a Python sum over lists | 19 | 10 | 4 |
| `su.RotationMatrixZ(q[2], functions=esym) @ q` | 5 | 0 | 0 |
| userRigidBodyTest.py, the 6 symbolic functions | 314 | 0 | 63 (20 %) |

Duplicates are counted structurally: an operation is a duplicate if its operation and its operands, already replaced by
their first occurrence, appeared before - so `cos(q[1])` twice is one duplicate (`q[1]` is written inline and not
counted), and the products that use them are duplicates too.

- **Trivial operations** come from Python: numpy's product and `sum()` of object arrays multiply by every zero of a
  constant matrix and add the products. The products of `esym.Vector` and `esym.Matrix` are one node of the program
  each (`t11 @ q`), computed by the dense C++ product; their zeros cost a multiplication, not an operation of the list.
- **Duplicates** come from writing the same expression twice, which the symbolic rigid body, written for clarity, does
  for a fifth of its operations.
- How the time of an evaluation follows the number of operations is measured with the implementation; a `+` of the
  list costs a virtual call and the copying of its operands as a `cos` does.

## 3. Proposal: one optimizing pass in `FlatProgram::Build`

**Where.** In `Build`, not when recording: the recorded graph stays what the user wrote - its `ToString()`, the tree
evaluation (`flatEvaluation=False`) and `Diff` are unchanged, and the bitwise comparison of tree and flat evaluation in
test_symbolicFlatEvaluation.py stays meaningful. A switch `UserFunction.optimizeFlatProgram` (default True) turns it
off for a comparison.

**Merging equal operations (hash consing).** `Build` already visits operands before their operation. A second map
`slotOfKey` from a key to a slot:

- key of an operation: its class (`std::type_index(typeid(*node))`) and the slots of its operands. Because the operands
  are already merged, comparing one level compares the whole subexpression, **to any depth** - no depth limit is
  needed, and the cost is one hash lookup per node;
- key of a leaf: a number by its bits (`ExpressionReal`, `VectorExpressionReal`, `MatrixExpressionReal` by their
  values); a named variable and an argument by their pointer - two variables with the same name are different;
- `+`, `*`, `==`, `!=`, `min` and `max` sort their two operand slots first, so `a*b` and `b*a` merge; this is exact,
  IEEE addition and multiplication are commutative;
- every expression class keeps all of its data in its operands today (the component index of `q[1]` is an operand
  too), so `typeid` and the operand slots are the whole key. A class that gets data outside its operands later
  overrides a virtual `FlatKey()`.

A merged operation computes from the same operands with the same function, so **the result has the same bits** as the
tree evaluation. Only the list gets shorter.

**Simplifications**, where an operand is a number (a constant leaf, never a named variable):

| rule | exact in IEEE arithmetic |
|---|---|
| `1*x`, `x*1`, `x/1`, `+x`, `x**1` → `x`; `-(-x)` → `x` | yes |
| `x+0`, `0+x`, `x-0` → `x` | yes, except that `-0+0` is `+0`, which nothing in Exudyn distinguishes |
| `0*x`, `x*0`, `0/x` → `0` | **no**: `0*inf` and `0*nan` are `nan`; with the rule a `nan` of `x` disappears |
| `x-x` → `0`, `x/x` → `1` | no, for the same reason; and rare |
| an operation with only number operands → the number | yes (already done when recording, here also for what the rules produce) |

Proposed: the exact rules, and `0*x` → `0` too, because the numpy products need it (13 of their 16 operations go) and a
`nan` in a user function shows elsewhere (`nan` in the results, a failing solver); `x-x` and `x/x` not. Applied
repeatedly from the leaves up, a rule may make a node a number, which then folds its users.

**Vector and matrix nodes** are merged by the same key; their buffers belong to their nodes, so a merged node keeps one
buffer. Simplifying inside the dense products (skipping the zeros of a constant matrix) is a different optimization
and not proposed.

## 4. Impact on the implementation

- `FlatProgram::Build`: the key map and the rules, about 120 lines; `Evaluate` and the text are unchanged, the text
  shows the shorter list.
- `ExpressionBase`, `VectorExpressionBase`, `MatrixExpressionBase`: `virtual bool IsNumber(Real& value)` for the
  rules (true for the constant leaves), a virtual `FlatKey()` with the default above. No change to the 60 expression
  classes, to the recording, to `Diff` or to the Python interface beyond the switch.
- Thread safety (RG7.8.3) is not affected: the program stays one per recording.
- Tests: test_symbolicFlatEvaluation.py with the switch on and off (same bits, except `0*nan`), the operation counts of
  the cases above, userRigidBodyTest.py with its reference unchanged; the time of userRigidBodyTest.py with 10000 steps
  before and after.

Expected: userRigidBodyTest.py 314 → about 250 operations; the matrix of the question 11 → 8; a numpy object product
16 → 3.

## 5. Decision needed

1. The optimization in `Build`, on by default, with a switch - or when recording.
2. `0*x` → `0` (and with it the loss of a `nan` of `x`), or only the exact rules.
