(sec-symbolicuserfunctions)=
# Symbolic user functions

A symbolic user function is a Python user function that Exudyn records once and then computes in C++, without
calling Python - 5 to 44 times faster than the Python function, at about the cost of the built-in force laws, and in
the threads of the parallel loops. This chapter shows how to write, check and use one. The module, its types, what is
recorded and what to know are on the reference page {ref}`sec-cinterface-symbolic`; the user functions in general are
{ref}`sec-userfunctions`.

## A first symbolic user function

A spring-damper with a progressive spring and a smooth friction-like damping, first as Python function:

```python
import exudyn as exu
from exudyn.utilities import *
from exudyn.advancedUtilities import CreateSymbolicUserFunction
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()
ground = mbs.AddObject(ObjectGround())
node = mbs.AddNode(NodePoint(referenceCoordinates=[1, 0, 0]))
mass = mbs.AddObject(MassPoint(nodeNumber=node, mass=1))
m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=ground))
m1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=mass))

#np.tanh computes with numbers and records with symbolic values
def SpringForce(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force):
    return stiffness*deltaL*(1 + 10*deltaL**2) + damping*np.tanh(deltaL_t/0.01)

spring = mbs.AddObject(SpringDamper(markerNumbers=[m0, m1], referenceLength=1, stiffness=100, damping=1,
                                    springForceUserFunction=SpringForce))
```

Then recorded and given to the connector instead of the Python function:

```python
symbolicForce = CreateSymbolicUserFunction(mbs, SpringForce, 'springForceUserFunction', spring)
mbs.SetObjectParameter(spring, 'springForceUserFunction', symbolicForce)
```

`CreateSymbolicUserFunction` calls `SpringForce` once, with symbolic arguments named after its parameters, and keeps
the expression it returns. Keep `symbolicForce` in a variable as long as the item uses it. The item gives the
signature of the user function, and the size of the vector arguments that depend on it, such as the coordinates of a
body.

**Check it** against the Python function, at arguments of the range the simulation reaches:

```python
for (deltaL, deltaL_t) in [(0.1, 0.), (-0.05, 0.02), (0.2, -1.)]:
    args = (mbs, 0., 0, deltaL, deltaL_t, 100., 1., 0.)
    print(symbolicForce.Evaluate(*args) - SpringForce(*args))   #0.0
```

`print(symbolicForce)` shows the recorded expression, `symbolicForce.GetFlatProgram()` the operations in the order
they are computed, one per line:

```
#SpringForce: 9 operations
t2 = (stiffness * deltaL)
t6 = pow(deltaL, 2)
...
t16 = (t9 + t15)
result = t16
```

## Writing a function that records

The function runs once, at the recording, with symbolic values; what it computes from them is recorded, what it
decides is decided then.

- **Operators and functions**: the operators of Python and numpy's functions (`np.sin`, `np.tanh`, `K @ q`,
  `np.multiply`) record, as do the functions of `exudyn.symbolic` (`esym.sin`, ...); `math.sin(x)` and `float(x)` of a
  symbolic value raise a TypeError, as they would record a constant. `*` of two symbolic vectors, or of a vector and a
  matrix, raises as well, because numpy multiplies the components there: write `a @ b`, `a.dot(b)` or
  `np.multiply(a, b)` (`exudyn.special.deprecations.symbolicStarProduct = True` restores the old products for the
  transition, #3016).
- **Branches**: `esym.IfThenElse(deltaL > 0, a, b)` decides at every call. A Python `if`, `and`, `or` or `not` on a
  symbolic value raises a `TypeError` while recording, because it would take one branch for every call (#3096).
  `x.Evaluate()` gives the value at the recording, for a decision that is meant to be made once; an `if` on a Python
  value, such as a setting of the script, is decided at the recording as well.
- **Loops** are recorded as often as they run: a loop over the components of a vector is fine, a loop whose count
  depends on a symbolic value is not possible.
- **Constants and parameters**: a Python variable is a constant of the recording. A value that changes during the
  simulation is a named variable, `esym.variables.Add('k', 100.)` before the recording and `esym.variables.Set('k',
  200.)` later, read in the function with `esym.variables.Get('k')`.
- **One function for both**: the rotation matrices, G matrices and conversions of `exudyn.symbolicUtilities` take a
  last argument `functions`, so that with `import exudyn.symbolicUtilities as su`,
  `su.RotXYZ2RotationMatrix(rot, functions=esym)` records and `functions=exudyn.cMath` computes numbers with the same
  bits as C++ (#3069).

## Vectors, matrices and the user items

- A vector argument - the coordinates `q`, `q_t` of ObjectGenericODE2 or ObjectUserBody, a position - is an
  `esym.Vector` of the size of the item while recording; `K @ q` with a numpy matrix `K` records `K` as a constant.
- A matrix result - a mass matrix, a Jacobian - is an `esym.Matrix`; one built from components with constant zeros
  is evaluated as a sparse matrix (#3022).
- **The user items** (ObjectUserBody, NodeUser, MarkerBodyUser, ObjectJointUser) are written with symbolic user
  functions as with Python ones. One recording serves every item of a kind: the `parameters` of each item are an
  argument of the recording, named by a `ParameterLayout`, so a user beam records its functions once for all its
  elements, see {ref}`sec-useritems`.

## Reading other items

Inside a recorded function, `mbs.GetNodeOutput`, `mbs.GetObjectOutput` (of connectors and the other objects that have
outputs), `mbs.GetObjectOutputBody`, `mbs.GetMarkerOutput` and `mbs.GetSensorValues` are symbolic expressions: C++
reads the item at every call, from the state the solver is computing, without Python (#2984). A load that follows a node
of another body:

```python
def LoadFollowing(mbs, t, loadVector):
    p0 = mbs.GetNodeOutput(nodeMass, exu.OutputVariableType.Position)
    p1 = mbs.GetNodeOutput(nodeBody, exu.OutputVariableType.Position)
    return 20.*(p1 - p0) + loadVector

mbs.Assemble()
function = CreateSymbolicUserFunction(mbs, LoadFollowing, 'loadVectorUserFunction', load)
mbs.SetLoadParameter(load, 'loadVectorUserFunction', function)
mbs.Assemble()
```

- A scalar output is an `esym.Real`, the others an `esym.Vector`, a rotation matrix one of 9 components; an output of
  two dimensions, such as `HomogeneousTransformation`, is refused. The output is read once while recording, for its
  size: before `mbs.Assemble()` only in the reference configuration, so velocities and forces need the system
  assembled before the recording.
- Every other attribute of `mbs` raises an `AttributeError` while recording - its value would be a constant of the
  recording. `mbs.variables` are Python values, constants of the recording; a value that changes is a variable of
  `esym.variables`.
- A function that reads items is evaluated in the main thread, also with several threads. The Jacobian of an item does
  not see what it reads from other items: a coupling through such a read lags in Newton, which is fine for loads,
  controllers and outputs, and not for stiff coupling, which belongs in a connector. An item cannot read its own output
  in its user function.
- `UserFunction.Evaluate(mbs, ...)` reads the outputs from the `mbs` it gets; outside a call, an expression has the
  value of the recording.

## Speed and threads

- `CreateSymbolicUserFunction(..., optimize=True)`, the default, applies `esym.Optimize`: an expression written twice
  becomes one, `1*x` and `x+0` go, with the same results bit for bit; a rigid-body force written for clarity takes half
  the time. `optimize=dict(level=2)` also removes `0*x`, `x-x` and combines numbers, which may lose a `nan` and change
  the last bit; `optimize=False` keeps the recording as it is (#3085).
- With `simulationSettings.parallel.numberOfThreads = 4`, the items whose user functions are all symbolic are computed
  in the threads, with the same results as with one thread: a chain of 4000 spring-dampers with one symbolic
  springForceUserFunction takes 0.77 s instead of 1.30 s, 200 user rigid bodies 0.31 s instead of 0.77 s. One Python
  user function puts its item, and the items that read it, into the main thread (#2978).

## A recording as code

`symbolicForce.GetFlatProgram(format='numpy')` writes the recording as a Python function with the arguments of the
user function, which runs without Exudyn:

```python
import numpy as np
def SpringForce(mbs, t, itemNumber, deltaL, deltaL_t, stiffness, damping, force):
    t2 = (stiffness * deltaL)
    t6 = np.power(deltaL, 2.0)
    ...
    return t16
```

`format='symbolic'` writes it with `exudyn.symbolic`, so that it is recorded again, and `format='sympy'` builds sympy
expressions, from which sympy writes C, C++, Fortran, Matlab or LaTeX and computes derivatives; `flatten=False`
writes the result as one expression. Every format computes what Exudyn computes: `mod` is the C `fmod`, `round` rounds
halves away from zero, a comparison is the number 0 or 1 (#3087, #3088). `exudyn.symbolic.GetFlatProgram(expressions,
...)` does the same for expressions of named variables, see `exudyn.misc.symbolicExport`.
