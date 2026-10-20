# Intermediate values in symbolic expressions: the options

revision2026c step RG12.2.5 (#3048), 2026-10-20. How a recorded symbolic function could compute a value that it uses
several times only once, and what that means for the recording of the C++ items planned later. Decision D18 of the
revision applies: values are recomputed, nothing is stored or hashed **between** evaluations. The status of
`exudyn.symbolic` is in [symbolicStatus.md](symbolicStatus.md); the code is in `src/Linalg/Symbolic*.h`.

## 1. What happens now

A symbolic user function is recorded once: the Python function runs with named symbolic arguments, and every
operation creates a node (`ExpressionBase`) that points to its operands. A value that the Python code uses several
times - `c1 = cos(rot[1])`, a component of `omegaLocal` - is **one** node with several parents: the recording is a
graph, and the reference counter of each node already says how many parents it has.

The evaluation walks this graph as a tree. `ExpressionOperatorMul::Evaluate()` calls `Evaluate()` of both operands,
which call theirs, and a node with three parents is computed three times, its operands as often, and so on. Measured
with [symbolicGraphSize.py](symbolicGraphSize.py) on the functions of `userRigidBodyTest.py`:

| function | operations of the graph | operations the tree evaluation performs | ratio |
|---|---|---|---|
| force (quadratic velocity vector of a rigid body, 6 components) | 122 | 1171 | 9.6 |
| position of a point | 48 | 71 | 1.5 |

The force takes about 5 µs per call (from Python, with the call overhead), about 4 ns per operation of the tree. The
multiplication comes from nesting: `JomegaLocal = J omegaLocal` uses each component of `omegaLocal` three times, the
cross product uses those again, the torque `G^T (...)` again - each level multiplies.

Vector and matrix nodes behave differently: each of them writes its result into a buffer of its own
(`EvaluateRef`), and a parent reads that buffer. The buffer is per node and holds the value of the current
evaluation only; a vector node with two parents is still computed twice.

## 2. Variables and their scope, as they are

| kind | created by | written | read |
|---|---|---|---|
| argument of a user function (`q`, `t`, `localPosition`) | `CreateSymbolicUserFunction`, one set per function | by C++ before each call (`processArgument`) | by the nodes of this function |
| named variable of a `VariableSet` (`esym.variables`, `mbs.symbolic.variables`) | `AddVariable` in Python | by Python, between calls (`SetValue`, `variables['x'] = 1`) | by every function that recorded it |
| constant | a float during recording | never | by its parents |

So there are two scopes: the arguments are **local** to a function, the variables of a VariableSet are **global**.
A recorded function cannot write anything: `SetValue` on a variable during the recording runs once, at recording, and
is not part of the graph. A VariableSet can therefore carry state between evaluations - a preStepUserFunction in Python
writes a variable, the symbolic functions read it - which is algorithm-like state, but written from outside, explicitly
and traceably; it is not a way to keep intermediate values *inside* an evaluation, and writing it from within an
evaluation would be the global intermediate storage D18 rules out (order-dependent, not thread-safe).

## 3. The options

### 3.1 A value per node, kept for the current evaluation (the proposal of #3047)

Every node gets a value and the number of the evaluation it belongs to; a counter is raised when the arguments are
set, and a node computed in this evaluation returns its value. Works with little code (one check in each `Evaluate`,
or a wrapper), but: the state is in the shared graph, so two threads cannot evaluate one function at the same time; a
stale value after a variable of a VariableSet changed between two calls is exactly the kind of error D18 is about; every
node pays the check. Closed with #3047.

### 3.2 Flat evaluation: the graph compiled into a list of operations

After the recording - once, in `SetupUserFunction` - the graph is sorted topologically into a list of operations
`slot[k] = op(slot[i], slot[j])`: each node is one entry, its operands come before it, a node with three parents is still
one entry. The evaluation runs the list from the start into an array of slots that belongs to the call and holds the
values of this evaluation only; nothing survives the call. This is how compiled code evaluates `c1 = cos(q4)` and then
uses `c1`: a local variable of the function.

- **Scope**: the slots are the local scope of one evaluation, as the arguments are; the variables of a VariableSet stay
  global inputs, read at the start of each evaluation. No value is kept between evaluations (D18), nothing is hashed:
  the sharing is the one the recording created, known from the graph.
- **Cost**: the force would run 122 operations instead of 1171, through a `switch` over the operation instead of a
  virtual call per node.
- **Results**: the same operations on the same values, in an order in which each operand is ready - bit-identical to
  the tree evaluation, which stays as the reference and as the fallback for what is not compiled yet.
- **Parallel**: the list is read-only; each thread needs its own slots.
- **IfThenElse**: a conditional jump over the operations of the branch not taken, so that it stays lazy.
- **Derivatives**: `Diff` works on the tree as now; the list allows a forward or reverse sweep later.
- **Readability**: the list prints as a program - `t5 = cos(q4)`, `t6 = t5*q_t3` - which is easier to read than a
  tree of 9456 characters; this is what option 3.4 asks for, without the user writing it.
- **Effort**: scalar nodes first (one opcode per expression class, about 30), vector and matrix nodes either as
  components or kept as tree nodes evaluated by an operation of the list; a few hundred lines in
  `SymbolicUtilities.h`, a comparison test of both evaluations on the existing symbolic test models.

### 3.3 An algorithmic form: statements, assignments, loops

Expressions plus statements - `x = ...`, `for`, `while` - as in code. It allows iterations, e.g. a Newton iteration
inside a user function, and shorter functions; it costs a language: scopes, control flow, assignments, and the order of
evaluation becomes part of the meaning. `Diff` through a loop that runs until convergence is not defined; a GPU wants no
data-dependent control flow. What is possible today already: a Python loop with a fixed number of iterations is
unrolled by the recording, and `IfThenElse` selects. Not recommended now; if a case appears, a loop with a fixed count
as one operation of the list of 3.2 is the smallest step.

### 3.4 Intermediate variables by hand

`c1 = esym.Intermediate(cos(rot[1]))`: the user marks what is computed once. Traceable, but the user has to find the
shared values, and the graph already knows them; with 3.2 a mark adds nothing to the speed. It could still give a slot a
name for the printed program - optional, later.

### 3.5 Others

- **Merging equal subexpressions while recording** (structural, not by value): `cos(q4)` recorded twice in two helper
  calls becomes one node. A transformation of the recording, not of values; it needs a comparison of expressions and
  is not needed for the functions measured - the Python code computes `G` once per function.
- **Generated C++ or GPU code from the list of 3.2**: the list is the input such a generator needs. A compiler at run
  time is a heavy dependency; it belongs to the compiled user items (RG8).
- **Fewer calls instead of faster calls**: a body whose markers ask for position, velocity, rotation and the Jacobians
  one by one calls its user functions once for each; a combined function, as `ComputeRigidBodyMarkerData` of
  ObjectRigidBody, computes them at once. A matter of the item, not of `exudyn.symbolic`.

## 4. Recording the C++ items

Can the mass matrix or the residual of the common items be recorded, or would they have to be written again as
symbolic trees? Read from the code:

- **The connectors and joints** compute their law as templates of the number type, `template<class TReal>`, on the
  kinematics of the markers (`MarkerRigid<TReal>`, #2745): CObjectConnectorSpringDamper, CartesianSpringDamper,
  RigidBodySpringDamper, LinearSpringDamper, TorsionalSpringDamper, CoordinateSpringDamper, Distance, Gravity,
  JointGeneric, JointSpherical, the 2D joints, the contacts. They are instantiated with `EXUmath::AutoDiff<n, Real>` for
  their Jacobians.
- **ANCFCable2D** (and Cable, Beam, ThinPlate, BeamGeometricallyExact2D): `ComputeODE2LHStemplate<TReal, ancfSize>` is
  the whole residual as a template, used with AutoDiff for the Jacobian.
- **ObjectRigidBody**: the position and rotation are templates (`RigidBodyRotationMatrixAD`, `RigidBodyPositionAD`, #2744);
  the mass matrix and the quadratic velocity vector are written for `Real` and take `G` and `Glocal` from the node
  (`CNodeRigidBody::GetGlocal`, `GetGlocal_t`), which are not templates; RigidBodyMath.h has few templates
  (`RotationMatrix2RotXYZTemplate`, the symmetric basis).

So **no re-implementation**: the code that is already a template for automatic differentiation can be instantiated
with a recording number type - a C++ type that creates the nodes of `exudyn.symbolic` as `SReal` does. What it needs:

1. the recording type with the interface the AD templates use: arithmetic with `Real`, comparisons, `sqrt`, `sin`,
   `cos`, `Value()` (the `if constexpr (!std::is_same<TReal, Real>)` branches call it);
2. **branches on values** - `if (springLength2 != 0.)`, the zero-length direction of a spring - are taken at recording
   with the values of that moment; for a recording that is evaluated later they must become `IfThenElse`, or the
   recording is valid only while the branch stays the same. This is the main work per item, and it is a review of each
   template, not a rewrite;
3. the rigid body: `ComputeMassMatrix` and `ComputeODE2LHS` as templates, with the `G` matrices of the nodes as
   templates - for each kind of rotation parameters (moderate);
4. the system level - the assembly by the ltg lists, loads, `J^T f` of the markers - is a separate layer; recording an
   item's own residual on its coordinates is the first step, as the user functions do now.

**Consequence for the choice**: a recorded C++ item is a large graph with much sharing - every AD template computes
its intermediate values once as local variables, and a recording keeps them as shared nodes. Evaluated as a tree, it
would multiply the work as the force above does, only more. A flat list of operations (3.2) is therefore needed for
that plan anyway, and it is also the form a GPU or a code generator takes. Option 3.2 changes the evaluation, not the
recording, the classes, the interface or the results.

Recording itself also evaluates: the constructor `SReal(ExpressionBase*)` evaluates the new node at once, which for a
tree means its whole subtree. For the user functions this costs nothing worth measuring; for a recorded item it would
need the value computed from the operands' values instead - one operation per node.

## 5. Recommendation

1. **3.2, the flat evaluation of scalar expressions**, as the next step: the list built once after recording, slots
   per evaluation, tree evaluation kept as reference and fallback, a test that both give bit-identical results on the
   symbolic test models; `userRigidBodyTest.py` with 10000 steps measured before and after. Then vector and matrix nodes.
2. Not now: 3.3 (statements and loops), 3.4 (marks by hand), merging while recording.
3. Before recording C++ items: the recording number type and the review of the branches of one connector
   (SpringDamper) and of ANCFCable2D, as a feasibility step of its own.
