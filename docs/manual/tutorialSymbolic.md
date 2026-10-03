# Symbolic user function tutorial

The following tutorial demonstrates the setup of a nonlinear oscillator with a mass point and a force user function, using a Cartesian spring damper defined by symbolic user functions.
\
First, we import the necessary libraries, create a system container and a main system:

```python
  import exudyn as exu
  from exudyn.utilities import *  # includes itemInterface and rigidBodyUtilities
  import exudyn.graphics as graphics
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
```

We set an abbreviation for the symbolic library for convenient access;
often you can replace that with numpy:

```python
  esym = exu.symbolic
```

Now, define the parameters for the linear spring-damper system:

```python
  L = 0.5
  mass = 1.6
  k = 4000
  omega0 = 50  # sqrt(k / mass)
  dRel = 0.05
  d = dRel * 2 * omega0
  u0 = -0.08
  v0 = 1
  f = 80
```

Create the ground object and the mass point with initial conditions:

```python
  objectGround = mbs.CreateGround(referencePosition=[0, 0, 0])
  massPoint = mbs.CreateMassPoint(referencePosition=[L, 0, 0],
                                  initialDisplacement=[u0, 0, 0],
                                  initialVelocity=[v0, 0, 0],
                                  physicsMass=mass)
```

Set up the Cartesian spring damper between the ground and the mass point, and apply an external force on the mass point:

```python
  csd = mbs.CreateCartesianSpringDamper(bodyNumbers=[objectGround, massPoint],
                                        stiffness=[k, k, k],
                                        damping=[d, 0, 0],
                                        offset=[L, 0, 0])
  load = mbs.CreateForce(bodyNumber=massPoint, loadVector=[f, 0, 0])
```

Add a sensor to monitor the position of the mass point:

```python
  sMass = mbs.AddSensor(SensorBody(bodyNumber=massPoint,
                                   storeInternal=True,
                                   outputVariableType=exu.OutputVariableType.Position))
```

Define a user function for the Cartesian spring damper, which may use Python or symbolic expressions:

```python
  def springForceUserFunction(mbs, t, itemNumber, u, v, k, d, offset):
      return [0.5 * u[0]**2 * k[0] + esym.sign(v[0]) * 10, k[1] * u[1], k[2] * u[2]]
```

We assign `CSDuserFunction` to the Python user function. This is used if no symbolic user function is used:

```python
  CSDuserFunction = springForceUserFunction
```

Up to now, everything looks like regular user functions. We now add an optional way to create a symbolic user function, which runs much faster in this case:

```python
  doSymbolic = True
  if doSymbolic:
      CSDuserFunction = CreateSymbolicUserFunction(mbs, springForceUserFunction,
                                                   'springForceUserFunction', csd)
      # Check function:
      print('user function:\n', CSDuserFunction)
```

Set the user function to the object, assemble the system, and configure the simulation settings:

```python
  mbs.SetObjectParameter(csd, 'springForceUserFunction', CSDuserFunction)
  mbs.Assemble()

  simulationSettings = exu.SimulationSettings()
  tEnd = 2
  steps = 200000
  simulationSettings.timeIntegration.numberOfSteps = steps
  simulationSettings.timeIntegration.endTime = tEnd
  simulationSettings.timeIntegration.verboseMode = 1
  simulationSettings.solution.file.write = False
  simulationSettings.solution.sensors.writePeriod = 0.001
```

Finally, start the renderer and solver, then evaluate the solution:

```python
  SC.renderer.Start()
  mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.ExplicitMidpoint)
  SC.renderer.Stop()  # safely close rendering window!
  n1 = mbs.GetObject(massPoint)['nodeNumber']
  u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
  print('u=', u)
  mbs.PlotSensor(sMass)
```

NOTE: this tutorial has been mostly created with ChatGPT-4, and curated hereafter!
