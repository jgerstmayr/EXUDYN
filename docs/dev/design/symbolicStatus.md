# exudyn.symbolic: status of the expressions and the symbolic user functions

revision2026c step RG12.1 (#2981), 2026-10-16. What `exudyn.symbolic` (`esym`) provides, which user
functions can be symbolic, what is missing and where the interface is awkward. The code is in `src/Linalg/Symbolic.h`
(scalars), `SymbolicVector.h`, `SymbolicMatrix.h`, `SymbolicUtilities.h` (the user functions) and `Symbolic.cpp` (the
bindings); the performance is in [userFunctionBoundary.md](userFunctionBoundary.md), section 12; the tests are listed
in section 5.

## 1. What a symbolic user function is

`CreateSymbolicUserFunction(mbs, function, userFunctionName, itemIndex)` (exudyn.advancedUtilities) calls the Python
function **once** with symbolic arguments - `esym.Real`, `esym.Vector`, `esym.Matrix` named after the arguments - while
`esym.SetRecording(True)`; every operation on them builds a node of an expression tree, and the returned expression is
the function. C++ evaluates the tree at every call: the arguments are copied into the named variables, the tree is
evaluated, the result goes to the item - without Python, at about the cost of a built-in force law (0.05 to 0.3
microseconds per call, against 0.35 to 2.4 for a Python function).

Consequences of recording once:

- **Python control flow is evaluated at recording**, with the recorded values (0 and empty vectors): an `if` on an
  argument, a `for` over `len(q)`, `math.sin` or `np.sin` of an argument do not record. Conditions are
  `esym.IfThenElse(condition, a, b)`, `esym.Not`, the comparisons `<`, `<=`, `==`, ... of `esym.Real`; the functions
  are those of `esym`.
- **A function can be written once for both**: with the math functions as a parameter - `exudyn.cMath` for Python,
  `esym` for the recording, both with `sin`, `cos`, `sqrt`, `atan2`, `abs`, `sign`, `min`, `max`, ... - and with
  components (`q[0]`, `A[0][1]`), as `userRigidBodyTest.py` and `symbolicUserFunctionSignaturesTest.py` do.
- **Parameters** that should change after the recording are `esym.variables` (a `VariableSet`: `Add`, `Set`, `Get`):
  a Python variable is a constant of the expression.
- **The vector arguments have no size while recording**: a component `q[i]` evaluates to 0 then (#3007); an operation
  that needs the size - `q + esym.Vector([1, 2, 3])` - fails while recording.

## 2. The expression types

| type | what | operators and functions |
|---|---|---|
| `esym.Real` | scalar | `+ - * /`, unary `-`, comparisons, the math functions of `esym` (`sin` ... `tanh`, `exp`, `log`, `sqrt`, `pow`, `atan2`, `abs`, `sign`, `floor`, `ceil`, `round`, `mod`, `min`, `max`, `isfinite`), `IfThenElse`, `Not`; `Diff(variable)`, the derivative, for scalars only |
| `esym.Vector` | vector of any size | `+ -` of vectors, `*` with a scalar, `*` of two vectors (scalar product), unary `-`, `v[i]`, `NormL2()`, `MultComponents(w)`, `==`, `!=`; built from a list of numbers and `esym.Real` |
| `esym.Matrix` | matrix | `+ -`, `*` with a scalar, matrix * matrix, matrix * vector and vector * matrix (the transposed product), unary `-`, `A[i, j]`, `Get(i, j)` |

Every vector node evaluates into its own buffer since #3006: no memory is allocated per call once the sizes are set.
Matrix nodes still return new matrices (RG12.3.7).

## 3. The user functions that can be symbolic

All user functions whose arguments are numbers, vectors or matrices of fixed size and whose result is a number, a
bool or a vector: 30 signatures (`userFunctionArgsDict`), each tested in `symbolicUserFunctionSignaturesTest.py`:

- **MainSystem**: `preStepUserFunction`, `postStepUserFunction`, `postNewtonFunction`;
- **connectors**: SpringDamper, LinearSpringDamper, TorsionalSpringDamper, CoordinateSpringDamper,
  CoordinateSpringDamperExt, CartesianSpringDamper (`springForceUserFunction`/`springTorqueUserFunction`),
  RigidBodySpringDamper (`springForceTorqueUserFunction`, `postNewtonStepUserFunction`), the offsets of
  CoordinateConstraint and JointGeneric (`offsetUserFunction`, `offsetUserFunction_t`), CoordinateVector
  (`constraintUserFunction`);
- **bodies and systems**: the `forceUserFunction` of ObjectGenericODE2, ObjectFFRF, ObjectFFRFreducedOrder,
  ObjectKinematicTree and ObjectUserBody, the `rhsUserFunction` of ObjectGenericODE1, ObjectUserBody's
  `positionUserFunction` and `outputVariableUserFunction`, the axial force and bending moment of ObjectANCFCable2D;
- **loads**: LoadCoordinate, LoadForceVector, LoadTorqueVector, LoadMassProportional.

Fixed with the tests (#2985): a `StdVector2D` result (`postNewtonFunction`), a `bool` argument
(`constraintUserFunction`'s `velocityLevel`) and an `OutputVariableType` argument (ObjectUserBody's
`outputVariableUserFunction`) could not be recorded, and `UserFunction.Evaluate` from Python took no matrix arguments.

## 4. What is missing, and where the interface is awkward

| | missing or awkward | where it belongs |
|---|---|---|
| 1 | **matrix results**: mass matrices, Jacobians, the rotation of ObjectUserBody, `jacobianUserFunction` - they return `MatrixContainer`/numpy and cannot be symbolic; a user body is symbolic only in its force, position and output variables (`userRigidBodyTest.py`) | RG12.3.7 |
| 2 | **the MainSystem in an expression**: the position of a node, a sensor value - a symbolic function sees only its arguments | RG12.4 |
| 3 | **matrix functions**: transpose, inverse, skew, cross product, `MultComponents` of matrices, matrix nodes into buffers | RG12.3.7 |
| 4 | **derivatives of vectors and matrices**: `Diff` exists for scalars only; analytic Jacobians of symbolic forces would need it | (not planned) |
| 5 | **parallel evaluation**: the named variables and the node buffers belong to the function object, so one function is evaluated by one thread at a time | RG7.8 |
| 6 | **graphics, sensors, matrix user functions** (`graphicsDataUserFunction`, `sensorUserFunction`, `massMatrixUserFunction`, ...) take or return Python objects | - |
| 7 | the sizes of the vector arguments are unknown while recording (section 1) - the recording could take them from the item when it is assembled | (raise when needed) |
| 8 | the docstring of `CreateSymbolicUserFunction` asks to keep the returned object alive; an item that gets it with `SetObjectParameter` or in its constructor holds it itself (checked: deleted and collected, the item still solves), only `TransferUserFunction2Item` does not - the note is stricter than needed | (raise when needed) |
| 9 | the result of a recording is printed as the expression (`print(symbolicFunction)`), but an error inside the tree names no Python line | - |

## 5. Tests

| test | what |
|---|---|
| `symbolicUserFunctionSignaturesTest.py` | all 30 signatures evaluated against Python; seven items simulated with both |
| `symbolicUserFunctionTest.py` | spring-damper and load, shared signatures |
| `symbolicVectorUserFunctionTest.py` | vectors above 7 items, every kind of vector node, components (#2998, #3006, #3007) |
| `userRigidBodyTest.py` | ObjectUserBody with symbolic force and position |
| `symbolicUserFunctionCopyTest.py`, `loadUserFunctionTest.py` | copying a symbolic function with its item, the load functions |
| `perfUserFunctions.py` (performance suite) | spring, Cartesian, load, ObjectGenericODE2 with 6, 10, 100 coordinates; Python and symbolic give the same results |
