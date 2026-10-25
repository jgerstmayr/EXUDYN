(sec-useritems)=
# User items

A **user item** is an item whose behaviour is written in Python: a body, a node or a connector whose equations, positions or
forces are user functions. It works with everything the built-in items work with - markers, loads, sensors, connectors,
the solvers and the renderer - because it provides the same interface to the rest of the system. What each kind of item
gives to the solver and takes from it is explained on the general page of its kind in the reference manual:
{doc}`nodes </docs/generated/items/nodeGeneral>`, {doc}`bodies </docs/generated/items/objectBodyGeneral>`,
{doc}`finite elements </docs/generated/items/objectFiniteElementGeneral>`,
{doc}`connectors </docs/generated/items/objectConnectorGeneral>`, {doc}`markers </docs/generated/items/markerGeneral>`,
{doc}`loads </docs/generated/items/loadGeneral>` and {doc}`sensors </docs/generated/items/sensorGeneral>`.

| user item | what the user functions define | typical use |
|---|---|---|
| {ref}`ObjectUserBody <sec-item-objectuserbody>` | mass matrix, generalized forces, position (and rotation) at a local position, their Jacobians, output variables, mass integral | a new finite element, a rigid body in other coordinates |
| {ref}`NodeUser <sec-item-nodeuser>` | position (and rotation) of the node from its coordinates | nodes that Exudyn does not have, e.g. an ANCF node with full gradients |
| {ref}`ObjectConnectorUser <sec-item-objectconnectoruser>` | the forces, applied through its markers | contact on flexible bodies, force laws between several markers |
| {ref}`ObjectJointUser <sec-item-objectjointuser>` | 1 to 6 constraint equations of the frames of two rigid markers | a screw joint, couplings of rotation and translation |
| {ref}`MarkerUser <sec-item-markeruser>` | position (and rotation) from the coordinates of several bodies, their Jacobians | a point between bodies or nodes, where built-in connectors, joints, loads and sensors act |
| ObjectGenericODE2, SensorUserFunction, the `...UserFunction` of the built-in items | a force law, a load, a sensor value | see {ref}`sec-userfunctions` |

## The arguments of a user function

Every user function of ObjectUserBody and NodeUser is called with

`(mbs, t, itemNumber, [what the function is about], qRef, qTotal, q_t, parameters)`

- `mbs, t, itemNumber`: the MainSystem, the time of the configuration asked for and the number of the item; everything
  else of the system a Python function reads through `mbs`, e.g. `mbs.GetObjectParameter(itemNumber, 'Vshow')`.
- what the function is about: `localPosition` for the position, rotation and Jacobian functions, `outputVariableType`
  and `localPosition` for the output variables.
- `qRef`, `qTotal`, `q_t`: the reference coordinates of the nodes, the **total** coordinates - reference plus displacement -
  and the velocities, of the configuration asked for; the outputVariableUserFunction gets the accelerations `q_tt` as
  well. A rigid body computes with `qTotal`, a finite element its displacements as `qTotal - qRef` and its reference
  geometry from `qRef`; a function with long expressions writes `q = qTotal` first.
- `parameters`: the parameter vector of the item, see below.

All vectors are read-only numpy arrays that view the memory of Exudyn ({ref}`sec-userfunctions`); a result is a list
or a numpy array, a matrix also a list of lists, and a result of the wrong size raises a ValueError that names the function.
The forceUserFunction of ObjectConnectorUser gets `(mbs, t, itemNumber, markers, dataCoordinates, parameters)` instead, see below.

## Parameters

An item has a vector `parameters` of any length - masses, lengths, stiffnesses, also flags as numbers - whose meaning the
user functions define. `exudyn.advancedUtilities.ParameterLayout` gives the entries names:
`layout = ParameterLayout([('mass', 1), ('inertia', 6)])`, `layout.ToVector({'mass': 2., 'inertia': [...]})` when the
item is added, `layout.ToDict(parameters)['mass']` inside the user function. `SetObjectParameter(item, 'parameters', ...)`
changes them during a simulation, e.g. in a parameter study.

## Symbolic user functions

The user functions of a user item can be recorded as symbolic functions with
`exudyn.advancedUtilities.CreateSymbolicUserFunction(mbs, function, 'forceUserFunction', item)`, which C++ evaluates
without calling Python. With the parameters as arguments, **one recording serves every item of a kind**, each with its
own parameters, and a changed parameter acts without recording again. A value that changes the structure of the
computation - a number of integration points, the size of a matrix, a Python `if` - has no value while the function
is recorded: it stays a constant of the Python code, and a structural variant is a recording of its own.
`exudyn.symbolicUtilities` has the rotation matrices and G matrices of `exudyn.cUtilities` with a last argument
`functions`: `su.RotXYZ2GLocal(rot, functions=exudyn.symbolic)` gives a symbolic matrix, with `numpy`, `math` or
`exudyn.cMath` a numpy array of numbers, so that a function is written once for both.
Measured on a chain of mass points, a Python force function of ObjectUserBody costs about 1 µs per call more than a
built-in force, a symbolic one 0.06 µs.

## What a user body provides

- **Equations of motion** $\Mm\, \ddot\qv = \fv + \fv_{loads}$ from `massMatrixUserFunction` and `forceUserFunction`;
  with `constantMassMatrix=True` the mass matrix is computed once and kept. The Jacobian is computed numerically.
- **The body interface** from `positionUserFunction` and `positionJacobianUserFunction` - and for rigid markers
  `rotationMatrixUserFunction` and `rotationJacobianUserFunction`: MarkerBodyPosition, MarkerBodyRigid, SensorBody, the
  loads on them and every connector work on the body.
- **Mass-proportional loads**: with `massIntegralUserFunction`, the mass integral of the position Jacobian
  $\int_V \rho\, \Jm_{pos}\, dV$, MarkerBodyMass and LoadMassProportional (gravity) act on the body.
- **Output variables**: with `outputVariableUserFunction`, every output variable the body does not compute itself -
  strains, stresses, curvatures, forces, potential energy, accelerations and `UserType0` to `UserType3`, which
  `exu.special.userTypes.SetName` can name.
- **Drawing**: `graphicsData` in VObjectUserBody is given in local coordinates and every point is mapped with the
  positionUserFunction, so that a deformable body is drawn deformed - with a symbolic positionUserFunction without
  Python; the graphicsDataUserFunction returns graphics data in global coordinates for anything else.

With more than one thread (`simulationSettings.parallel.numberOfThreads`), a user item whose user functions are all
symbolic is computed in the threads of the parallel loops, as are the connectors and loads on it; a user item with a
Python user function, and what depends on it, is computed in the main thread (#2978). The renderer draws the user
items and what depends on them in the main thread.

## A user connector and its markers

The forceUserFunction of ObjectConnectorUser gets `markers`, one {ref}`ConnectorMarker <sec-connectormarker>` per marker of
`markerNumbers`. It reads the marker - `GetPosition()`, `GetVelocity()`, `GetRotationMatrix()`, `GetAngularVelocity()`,
`GetCoordinates()`, and for a body marker `GetBodyPosition(localPosition)` anywhere on the body - and applies forces:
`ApplyForce(force)`, `ApplyTorque(torque)`, `ApplyBodyForce(force, localPosition)` and `ApplyCoordinateForces(forces)`.
A contact can so search the closest point on a flexible body and act there; userConnectorTest.py rests a ball on an
ANCF cable this way. A ConnectorMarker is valid during the call only.

With a NodeGenericData as `nodeNumber`, the user functions get its `dataCoordinates`, the state of a contact or of
friction, and a `postNewtonStepUserFunction(mbs, t, itemNumber, markers, dataCoordinates, dataCoordinatesStartOfStep,
parameters)` updates them after each Newton iteration: it returns `(dataCoordinates, discontinuousError)` or, with a
step size the solver shall reduce to, `(dataCoordinates, discontinuousError, recommendedStepSize)` (#3102).
userConnectorContactTest.py writes the contact of ObjectContactSphereSphere this way.

## A user joint and a user marker

ObjectJointUser constrains two rigid markers by `constraintUserFunction(mbs, t, itemNumber, position0, rotationMatrix0,
position1, rotationMatrix1, parameters)`, which returns `numberOfConstraints` equations; their derivative by the
translations and rotation increments of the markers is given by `jacobianUserFunction`, which an active joint needs -
a constraint Jacobian approximated numerically gives artifacts (#3101) -, and the Lagrange multipliers are the reaction
forces (#2976). userJointTest.py writes a screw joint over several turns: the coupling of angle and translation is
written periodic, as $\sin(\varphi - x/h) = 0$, because an angle computed by `atan2` jumps by $2\pi$ at half a turn and
gives no continuous constraint beyond it (#3100).

MarkerUser is a marker on any number of bodies: its `positionUserFunction(mbs, t, itemNumber, qRef, qTotal, q_t,
parameters)` gets the coordinates of the nodes of these bodies, body by body, and every built-in connector, joint, load
and sensor acts at that point (#3065); with a `rotationMatrixUserFunction` it is a rigid marker. The Jacobians are
computed by central differences if they are not given - then a Newton tolerance below about 1e-10 is not reached.

## Example

Two mass points on springs as user bodies of one kind - each with its own mass and stiffness, gravity through the mass
integral, drawn as spheres - with one symbolic recording of the force for both:

```python
import exudyn as exu
from exudyn.utilities import *
from exudyn.advancedUtilities import CreateSymbolicUserFunction, ParameterLayout
import exudyn.graphics as graphics
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#a mass on a spring as user body: the parameters named by a layout
layout = ParameterLayout([('mass', 1), ('stiffness', 1)])

def UFmass(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return layout.ToDict(parameters)['mass']*np.eye(3)

def UFforce(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return -layout.ToDict(parameters)['stiffness']*(qTotal - qRef)   #the spring acts on the displacement

def UFposition(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return qTotal + localPosition

def UFpositionJacobian(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return np.eye(3)

def UFmassIntegral(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return layout.ToDict(parameters)['mass']*np.eye(3)               #int(rho J_pos dV): gravity acts on it

bodies = []
for (i, value) in enumerate([{'mass': 2., 'stiffness': 50.}, {'mass': 0.5, 'stiffness': 200.}]):
    node = mbs.AddNode(NodePoint(referenceCoordinates=[i, 0, 0], initialCoordinates=[0, 0.1, 0]))
    bodies += [mbs.AddObject(ObjectUserBody(nodeNumbers=[node], parameters=layout.ToVector(value),
                                            constantMassMatrix=True, massMatrixUserFunction=UFmass,
                                            forceUserFunction=UFforce, massIntegralUserFunction=UFmassIntegral,
                                            positionUserFunction=UFposition, positionJacobianUserFunction=UFpositionJacobian,
                                            visualization=VObjectUserBody(graphicsData=[graphics.Sphere(radius=0.1)])))]
    mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=bodies[-1])),
                                     loadVector=[0, -9.81, 0]))

#one symbolic recording of the force serves both bodies, each with its own parameters
symbolicForce = CreateSymbolicUserFunction(mbs, UFforce, 'forceUserFunction', bodies[0])
for body in bodies:
    mbs.SetObjectParameter(body, 'forceUserFunction', symbolicForce)

mbs.Assemble()
mbs.SolveDynamic()
```

Every user item has a short example on its reference page. The test models are complete examples, each compared with a
built-in item or an exact solution, and run alone they show their bodies and open the SolutionViewer:
userRigidBodyTest.py (a rigid body), userANCFCable2DTest.py and userMassIntegralTest.py (a 2D ANCF cable element),
userANCFBeam3DTest.py (a fully parametrized ANCF beam on NodeUser), userTetrahedronTest.py (a solid element),
userParametersTest.py, userOutputVariablesTest.py, userGraphicsDataTest.py, userNodeTest.py, userConnectorTest.py,
userJointTest.py and userMarkerTest.py,
in `python/TestModels`.
