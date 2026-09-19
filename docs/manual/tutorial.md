# Tutorial

This section will show:

- A basic tutorial for a 1D mass and spring-damper with initial displacements, shortest possible model with practically no special settings
- A more advanced rigid-body model, including 3D rigid bodies and revolute joints
- Links to examples section

A large number of examples, some of them quite advanced, can be found in:

- `python/Examples`
- `python/TestModels`

## Mass-Spring-Damper tutorial

The Python source code of the first tutorial can be found in the file:

- `python/Examples/springDamperTutorial.py`

A similar version based on a simplified approach (using a 3D mass point) is available as, which uses simplified approaches:

- `python/Examples/springDamperTutorialNew.py`

The following tutorial will set up a mass point and a spring damper, dynamically compute the solution and evaluate the reference solution.
\
We import the exudyn library and the interface for all nodes, objects, markers, loads and sensors:

```python
  import exudyn as exu
  from exudyn.utilities import Point, NodePointGround, MassPoint, MarkerNodeCoordinate,\
                               CoordinateSpringDamper, LoadCoordinate, SensorObject
  import exudyn.graphics as graphics #only import if it does not conflict
  import numpy as np #for postprocessing
```

Instead of the named import of `exudyn.utilities` functions and classes, you can use a star import,
which includes `itemInterface`, `rigidBodyUtilities` and some helper functions:

```python
  from exudyn.utilities import *
```

Next, we need a `SystemContainer`, which contains all computable systems and add a new MainSystem `mbs`.
Per default, you always should name your system 'mbs' (multibody system), in order to copy/paste code parts from other examples, tutorials and other projects:

```python
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
```

In order to check, which version you are using, you can printout the current Exudyn version.
The version shown is in line with the issue tracker and marks the number of open/closed issues added to Exudyn .
Adding `True` as argument will also print platform-specific information, which is helpful
in case of reporting some compatibility issues:

```python
  print('EXUDYN version='+exu.config.Version(True))
```

Using the powerful Python language, we can define some variables for our problem, which will also be used for the analytical solution:

```python
  L=0.5               #reference position of mass
  mass = 1.6          #mass in kg
  spring = 4000       #stiffness of spring-damper in N/m
  damper = 8          #damping constant in N/(m/s)
  f =80               #force on mass
```

For the simple spring-mass-damper system, we need initial displacements and velocities:

```python
  u0=-0.08            #initial displacement
  v0=1                #initial velocity
  x0=f/spring         #static displacement
  print('resonance frequency = '+str(np.sqrt(spring/mass)))
  print('static displacement = '+str(x0))
```

We first need to add nodes, which provide the coordinates (and the degrees of freedom) to the system.
The following line adds a 3D node for 3D mass point (Note: Point is an abbreviation for NodePoint, defined in `itemInterface.py`.):

```python
  n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0],
                       initialCoordinates = [u0,0,0],
                       initialVelocities = [v0,0,0]))
```

Here, `Point` (=`NodePoint`) is a Python class, which takes a number of arguments defined in the reference manual. The arguments here are `referenceCoordinates`, which are the coordinates for which the system is defined. The initial configuration is given by `referenceCoordinates + initialCoordinates`, while the initial state additionally gets `initialVelocities`.
The command `mbs.AddNode(...)` returns a `NodeIndex n1`, which basically contains an integer, which can only be used as node number. This node number will be used lateron to use the node in the object or in the marker.

While `Point` adds 3 unknown coordinates to the system, which need to be solved, we also can add ground nodes, which can be used similar to nodes, but they do not have unknown coordinates -- and therefore also have no initial displacements or velocities. The advantage of ground nodes (and ground bodies) is that no constraints are needed to fix these nodes.
Such a ground node is added via:

```python
  nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
```

In the next step, we add an object (For the moment, we just need to know that objects either depend on one or more nodes, which are usually bodies and finite elements, or they can be connectors, which connect (the coordinates of) objects via markers, see {ref}`sec-overview-modulestructure`.), which provides equations for coordinates. The `MassPoint` needs at least a mass (kg) and a node number to which the mass point is attached. Additionally, graphical objects could be attached:

```python
  massPoint = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = n1))
```

Note that instead of adding a `NodePoint` and a `MassPoint` with `mbs.AddNode(...)`
and `mbs.AddObject(...)`, there is also a convenient function `mbs.CreateMassPoint(...)`, which can do everything at once including the option to add gravity.

In order to apply constraints and loads, we need markers. These markers are used as local positions (and frames), where we can attach a constraint lateron. In this example, we work on the coordinate level, both for forces as well as for constraints.
Markers are attached to the according ground and regular node number, additionally using a coordinate number (0 ... first coordinate):

```python
  groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround,
                                                  coordinate = 0))
  #marker for springDamper for first (x-)coordinate:
  nodeMarker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1,
                                                  coordinate = 0))
```

This means that constraints are be applied to the first coordinate of node `n1` via marker with number `nodeMarker`, which is in fact of type `MarkerNodeCoordinate`.

Now we add a spring-damper to the markers with numbers `groundMarker` and the `nodeMarker`, providing stiffness and damping parameters:

```python
  nC = mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundMarker, nodeMarker],
                                       stiffness = spring,
                                       damping = damper))
```

A load is added to marker `nodeMarker`, with a scalar load with value `f`:

```python
  nLoad = mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker,
                                     load = f))
```

Again, instead of adding a `MarkerNodeCoordinate` and a `LoadCoordinate` with `mbs.AddLoad(...)`,
we could just use `mbs.CreateForce(...)` to add a 3D force vector.
For specific joints, there are also `mbs.Create...(...)` functions.

Finally, a sensor is added to the coordinate constraint object with number `nC`, requesting the `outputVariableType` `Force`:

```python
  mbs.AddSensor(SensorObject(objectNumber=nC, fileName='groundForce.txt',
                             outputVariableType=exu.OutputVariableType.Force))
```

Note that sensors can be attached, e.g., to nodes, bodies, objects (constraints) or loads.
As our system is fully set, we can print the overall information and assemble the system to make it ready for simulation:

```python
  print(mbs)     #show system properties
  mbs.Assemble() #prepare for simulation
```

We will use time integration and therefore define a number of steps (fixed step size; must be provided) and the total time span for the simulation:

```python
  tEnd = 1     #end time of simulation
  h = 0.001    #step size; leads to 1000 steps
```

All settings for simulation, see according reference section, can be provided in a structure given from `exu.SimulationSettings()`. Note that this structure will contain all default values, and only non-default values need to be provided:

```python
  simulationSettings = exu.SimulationSettings()
  simulationSettings.solutionSettings.solutionWritePeriod = 5e-3 #output interval general
  simulationSettings.solutionSettings.sensorsWritePeriod = 5e-3  #output interval of sensors
  simulationSettings.timeIntegration.numberOfSteps = tEnd/h
  simulationSettings.timeIntegration.endTime = tEnd
  simulationSettings.displayComputationTime = True               #show how fast
```

In order to see some solver output, we must set `verboseMode` to 1 (higher values gives detailed output per step).
Furthermore, we can show information on computation time (which may cost some overhead in computation!):

```python
  simulationSettings.timeIntegration.verboseMode = 1             #show some solver output
  simulationSettings.displayComputationTime = True               #show how fast
```

We are using a generalized alpha solver, where numerical damping is needed for index 3 constraints. As we have only spring-dampers, we can set the spectral radius to 1, meaning no numerical damping:

```python
  simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
```

In order to visualize the results online, a renderer can be started. As our computation will be very fast, it is a good idea to wait for the user to press SPACE, before starting the simulation (uncomment second line):

```python
  SC.renderer.Start()              #start graphics visualization
  #SC.renderer.DoIdleTasks()       #wait for SPACE bar or 'Q' to continue (in render window!)
```

As the simulation is still very fast, we will not see the motion of our node. Using a very small step size of, e.g., `h=1e-7` in the lines above allows us to visualize the resulting oscillations in realtime.

Finally, we start the solver, by telling which system to be solved, solver type and the simulation settings:

```python
  exu.SolveDynamic(mbs, simulationSettings)
```

After simulation, our renderer needs to be stopped (otherwise it will stop unsafely as soon as the Python kernel is stopped or restarted).
Sometimes you would like to wait until closing the render window, using `WaitForRenderEngineStopFlag()`:

```python
  #SC.renderer.DoIdleTasks()       #wait for pressing 'Q' to quit
  SC.renderer.Stop()               #safely close rendering window!
```

If you run this code, e.g. in Spyder or Visual Studio Code, it may take a 1-2 seconds to complete. However, the time spent is only related to some overhead in the Python environment and for the visualization. The simulation itself will only take around 3-10 milliseconds, in which a large overhead is due to file writing.

There are several ways to evaluate results, see the reference pages. In the following we take the final value of node `n1` and read its 3D position vector:

```python
  #evaluate final (=current) output values
  u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
  print('displacement=',u)
```

The following code generates a reference (exact) solution for our example:

```python
  import matplotlib.pyplot as plt
  import matplotlib.ticker as ticker

  omega0 = np.sqrt(spring/mass)          #eigen frequency of undamped system
  dRel = damper/(2*np.sqrt(spring*mass)) #dimensionless damping
  omega = omega0*np.sqrt(1-dRel**2)      #eigen freq of damped system
  C1 = u0-x0 #static solution needs to be considered!
  C2 = (v0+omega0*dRel*C1) / omega       #C1, C2 are coeffs for solution
  steps = int(tEnd/h)                    #use same steps for reference solution

  refSol = np.zeros((steps+1,2))
  for i in range(0,steps+1):
    t = tEnd*i/steps
    refSol[i,0] = t
    refSol[i,1] = np.exp(-omega0*dRel*t)*(C1*np.cos(omega*t)+C2*np.sin(omega*t))+x0

  plt.plot(refSol[:,0], refSol[:,1], 'r-', label='displacement (m); exact solution')
```

Now we can load our results from the default solution file `coordinatesSolution.txt`, which is in the same
directory as your Python tutorial file.
**Note** that the visualization of results can be simplified considerably using the `PlotSensor(...)` utility function as shown in the **Rigid body and joints tutorial**!

For reading the file containing commented lines (this does not work in binary mode!), we use a numpy feature and finally plot the displacement of coordinate 0 or our mass point (`data[:,0]` contains the simulation time, `data[:,1]` contains displacement of (global) coordinate 0, `data[:,2]` contains displacement of (global) coordinate 1, ...)):

```python
  data = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
  plt.plot(data[:,0], data[:,1], 'b-', label='displacement (m); numerical solution')
```

Note that the coordinates do not include the reference position (which is 0.5 in this case). For information on displacement and reference coordinates, see {ref}`sec-overview-items-coordinates`.

The sensor result can be loaded in the same way. The sensor output format contains time in the first column and sensor values in the remaining columns. The number of columns depends on the
sensor and the output quantity (scalar, vector, ...):

```python
  data = np.loadtxt('groundForce.txt', comments='#', delimiter=',')
  plt.plot(data[:,0], data[:,1]*1e-3, 'g-', label='force (kN)')
```

In order to get a nice plot within Spyder, the following options can be used (note, in some environments you need finally the command `plt.show()`):

```python
  ax=plt.gca() # get current axes
  ax.grid(True, 'major', 'both')
  ax.xaxis.set_major_locator(ticker.MaxNLocator(10))
  ax.yaxis.set_major_locator(ticker.MaxNLocator(10))
  plt.legend() #show labels as legend
  plt.tight_layout()
  plt.show()
```

The matplotlib output should look as shown in {ref}`fig-tutorial-springdamper`.

(fig-tutorial-springdamper)=
```{figure} /docs/theDoc/figures/plotSpringDamper.png
:width: 400

Output of spring-damper tutorial.
```

(sec-tutorial-rigidbodyjoints)=
## Rigid body and joints tutorial

The Python source code of the first tutorial, based on the simple description of revolute joints, can be found in the file:

- `python/Examples/rigidBodyTutorial3.py`

For alternative approaches, see

- `python/Examples/rigidBodyTutorial3withMarkers.py`
- `python/Examples/rigidBodyTutorial2.py`

**NOTE** that the youtube video uses a slightly older way of creating graphics using `GraphicsData...` functions, which
can be easily replaced by the newer `graphics. ...` commands shown here. Their interface is identical.

This tutorial will set up a multibody system containing a ground, two rigid bodies and two revolute joints driven by gravity, compare a 3D view of the example in {ref}`fig-rigidbodytutorialview`.

(fig-rigidbodytutorialview)=
```{figure} /docs/theDoc/figures/TutorialRigidBody1desc.png
:width: 400

Render view of rigid body tutorial, showing objects, nodes (N0, N1), and loads.
```

---
\
 We first import the exudyn library and the interface for all nodes, objects, markers, loads and sensors:

```python
  import exudyn as exu
  #import specific items, inertia class for general cuboid (hexahedral) block, etc.
  from exudyn.utilities import InertiaCuboid, ObjectRigidBody, MarkerBodyRigid, \
                               GenericJoint, VObjectJointGeneric, SensorBody
  import exudyn.graphics as graphics #only import as graphics if it does not conflict
  import numpy as np #for postprocessing
```

The submodule `exudyn.utilities` contains helper functions for graphics representation, 3D rigid bodies and joints.
For simplicity, many examples just use a star import instead, which is recommended for rapid model development:

```python
  from exudyn.utilities import *
```

 As in the first tutorial, we need a `SystemContainer` and add a new MainSystem `mbs`:

```python
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
```

 We define some geometrical parameters for lateron use.

```python
  #physical parameters
  g =     [0,-9.81,0] #gravity
  L = 1               #length
  w = 0.1             #width
  bodyDim=[L,w,w]     #body dimensions
  p0 =    [0,0,0]     #origin of pendulum
  pMid0 = np.array([L*0.5,0,0]) #center of mass, body0
```

 We add an empty ground body, using default values. It's origin is at [0,0,0] and here we use no visualization.

```python
  #ground body, located at specific position (there could be several ground objects)
  oGround = mbs.CreateGround(referencePosition=[0,0,0])
```

On the background, the `CreateGround` function creates an object, which is equivalent to:

```python
  oGround = mbs.AddObject(ObjectGround(referencePosition=[0,0,0]))
```

---
\
 For physical parameters of the rigid body, we can use the class `RigidBodyInertia`, which allows to define mass, center of mass (COM) and inertia parameters, as well as shifting COM or adding inertias.
The `RigidBodyInertia` can be used directly to create rigid bodies. Special derived classes can be use to define rigid body inertias for cylinders, cubes, etc., so we use a cube here:

```python
  #first link:
  #inertia for cubic body with dimensions in sideLengths
  iCube0 = InertiaCuboid(density=5000, sideLengths=bodyDim)
  iCube0 = iCube0.Translated([-0.25*L,0,0]) #transform COM, COM not at reference point!
```

Note that the COM is translated in axial direction, while it would be at the body's local position [0,0,0] by default!

 For visualization, we add some graphics for the body defined as a 3D cube with center point and dimensions; additionally we draw a basis (three RGB-vectors) at the COM:

```python
  #graphics for body
  graphicsBody0 = graphics.Brick(centerPoint=[0,0,0],size=[L,w,w],
                                 color=graphics.color.red)
  graphicsCOM0 = graphics.Basis(origin=iCube0.com, length=2*w)
```

 Now we have defined all data for the link (rigid body). We could use `mbs.AddNode(NodeRigidBodyEP(...))` and `mbs.AddObject(ObjectRigidBody(...))` to create a node and a body, but the `MainSystem` since V1.6.110 offers a much more comfortable function:

```python
  #create node, add body and gravity load:
  b0=mbs.CreateRigidBody(inertia = iCube0, #includes COM
                         referencePosition = pMid0,
                         gravity = g,
                         graphicsDataList = [graphicsCOM0, graphicsBody0])
```

which also adds a gravity load and could also set initial velocities, if wanted. Note that much more options are available for this function, e.g.,
we could define a `nodeType` for the underlying formulation of the rigid body node, see {ref}`sec-nodetype`.
We can use

- `RotationEulerParameters`: for fast computation, but leads to an additional algebraic equation and thus needs an implicit solver
- `RotationRxyz`: contains a singularity if the second angle reaches +/- 90 degrees, but no algebraic equations
- `RotationRotationVector`: for usage with Lie group integrators, especially with explicit integration, singularities are bypassed; leads to fewest unknowns and usually less Newton iterations

 We now add a revolute joint around the (global) z-axis.
We have several possibilities, which are shown in the following.
For the **first two possibilities only**, following `rigidBodyTutorial3withMarkers.py`, we need the following markers

```python
  #markers for ground and rigid body (not needed for option 3):
  markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
  markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[-0.5*L,0,0]))
```

 The very general **option 1** is to use the `GenericJoint`, that can be used to define any kind of joint with translations and rotations fixed or free,

```python
  #revolute joint option 1:
  mbs.AddObject(GenericJoint(markerNumbers=[markerGround, markerBody0J0],
                             constrainedAxes=[1,1,1,1,1,0],
                             visualization=VObjectJointGeneric(axesRadius=0.2*w,
                                                               axesLength=1.4*w)))
```

In addition, transformation matrices (`rotationMarker0/1`) can be added, see the joint description.

 **Option 2** is using the revolute joint, which allows a free rotation around the local z-axis of marker 0 (`markerGround` in our example)

```python
  #revolute joint option 2:
  mbs.AddObject(ObjectJointRevoluteZ(markerNumbers = [markerGround, markerBody0J0],
                                     rotationMarker0=np.eye(3),
                                     rotationMarker1=np.eye(3),
                                     visualization=VObjectJointRevoluteZ(axisRadius=0.2*w,
                                                                         axisLength=1.4*w)
                                     ))
```

Additional transformation matrices (`rotationMarker0/1`) can be added in order to chose any rotation axis.

 Note that an error in the definition of markers for the joints can be also detected in the render window (if you completed the example), e.g., if you change the following marker in the lines above,

```python
  #example if wrong marker position is chosen:
  markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[-0.4*L,0,0]))
```

$\ra$ you will see a misalignment of the two parts of the joint by `0.1*L`.
The latter approach is very general and will also work for any kind of flexible bodies.

 Due to the fact that the definition of markers for general joints is tedious, **option 3** is based on a MainSystem function, which allows to attach revolute joints immediately to **rigid bodies** and defining the rotation axis only once for the joint:

```python
  #revolute joint option 3 (simplest):
  mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b0], position=[0,0,0],
                          axis=[0,0,1], axisRadius=0.2*w, axisLength=1.4*w)
```

Note that `axis` and `position` are defined in global coordinates, and local coordinates are computed according to the reference configuration of the bodies.
There exist more arguments that may be specified, e.g., the axis and position can also be defined in the local frame of the first body.

---
\
 The second link and the according joint can be set up in a very similar way.
For visualization, we need to add some graphics for the body defined as a RigidLink graphics function:

```python
  #second link, simple graphics:
  graphicsBody1 = graphics.RigidLink(p0=[0,0,-0.5*L],p1=[0,0,0.5*L],
                                        axis0=[1,0,0], axis1=[0,0,0], radius=[0.06,0.05],
                                        thickness = 0.1, width = [0.12,0.12],
                                        color=graphics.color.lightgreen)

  b1=mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, sideLengths=[0.1,0.1,1]),
                         referencePosition = np.array([L,0,0.5*L]),
                         gravity = g,
                         graphicsDataList = [graphicsBody1])
```

 The revolute joint in this case has a free rotation around the global x-axis:

```python
  #revolute joint (free x-axis)
  mbs.CreateRevoluteJoint(bodyNumbers=[b0, b1], position=[L,0,0],
                          axis=[1,0,0], axisRadius=0.2*w, axisLength=1.4*w)
```

 Optionally, we could also add forces or torques onto bodies

```python
  #forces can be added like in the following
  force = [0,0.5,0]       #0.5N   in y-direction
  torque = [0.1,0,0]      #0.1Nm around x-axis
  mbs.CreateForce(bodyNumber=b1,
                  loadVector=force,
                  localPosition=[0,0,0.5], #at tip
                  bodyFixed=False) #if True, direction would corotate with body
  mbs.CreateTorque(bodyNumber=b1,
                  loadVector=torque,
                  localPosition=[0,0,0],   #at body's reference point/center
                  bodyFixed=False) #if True, direction would corotate with body
```

 Finally, we also add a sensor for some output of the double pendulum:

```python
  #position sensor at tip of body1
  sens1=mbs.AddSensor(SensorBody(bodyNumber = b1, localPosition = [0,0,0.5*L],
                                 fileName = 'solution/sensorPos.txt',
                                 outputVariableType = exu.OutputVariableType.Position))
```

---
\
 Before simulation, we need to call `Assemble()` for our system, which links objects, nodes, ..., assigns initial values and does further pre-computations and checks:

```python
  mbs.Assemble()
```

After `Assemble()`, markers, nodes, objects, etc. are linked and we can analyze the internal structure. First, we can print out useful information, either just typing `mbs` in the iPython console to print out overal information:

```
  <systemData:
    Number of nodes= 2
    Number of objects = 5
    Number of markers = 8
    Number of loads = 4
    Number of sensors = 1
    Number of ODE2 coordinates = 14
    Number of ODE1 coordinates = 0
    Number of AE coordinates   = 12
    Number of data coordinates   = 0
    For details see mbs.systemData, mbs.sys and mbs.variables
  >
```

Note that there are 2 nodes for the two rigid bodies. The five objects are due to ground object, 2 rigid bodies and 2 revolute joints.
The meaning of markers can be seen in the graphical representation described below.

 Furthermore, we can print the full internal information as a dictionary using:

```python
  mbs.systemData.Info() #show detailed information
```

which results in the following output (shortened):

```
  node0:
      {'nodeType': 'RigidBodyEP', 'referenceCoordinates': [0.5, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], 'addConstraintEquation': True, 'initialCoordinates': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'initialVelocities': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'name': 'node0', 'Vshow': True, 'VdrawSize': -1.0, 'Vcolor': [-1.0, -1.0, -1.0, -1.0]}
  node1:
      {'nodeType': 'RigidBodyEP', 'referenceCoordinates': [1.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0], 'addConstraintEquation': True, 'initialCoordinates': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'initialVelocities': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'name': 'node1', 'Vshow': True, 'VdrawSize': -1.0, 'Vcolor': [-1.0, -1.0, -1.0, -1.0]}
  object0:
      {'objectType': 'Ground', 'referencePosition': [0.0, 0.0, 0.0], 'name': 'object0', 'Vshow': True, 'VgraphicsDataUserFunction': 0, 'Vcolor': [-1.0, -1.0, -1.0, -1.0], 'VgraphicsData': {'TODO': 'Get graphics data to be implemented'}}
  object1:
      {'objectType': 'RigidBody', 'physicsMass': 50.0, 'physicsInertia': [0.08333333333333336, 7.333333333333334, 7.333333333333334, 0.0, 0.0, 0.0], 'physicsCenterOfMass': [-0.25, 0.0, 0.0], 'nodeNumber': 0, 'name': 'object1', 'Vshow': True, 'VgraphicsDataUserFunction': 0, 'VgraphicsData': {'TODO': 'Get graphics data to be implemented'}}
  object2:
      {'objectType': 'JointRevolute', 'markerNumbers': [3, 4], 'rotationMarker0': [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], 'rotationMarker1': [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], 'activeConnector': True, 'name': 'object2', 'Vshow': True, 'VaxisRadius': 0.019999999552965164, 'VaxisLength': 0.14000000059604645, 'Vcolor': [-1.0, -1.0, -1.0, -1.0]}
  object3:
  ...
```

 Sometimes it is hard to understand the degree of freedom for the constrained system. Furthermore, we may have added -- by error --
redundant constraints, which are not solvable or at least cause solver problems. Both can be checked with the command:

```python
  mbs.ComputeSystemDegreeOfFreedom(verbose=True) #print out DOF and further information
```

This will print:

```python
  ODE2 coordinates          = 14
  total constraints         = 12
  redundant constraints     = 0
  pure algebraic constraints= 0
  degree of freedom         = 2
```

We see that there are 14 ODE2 coordinates from the two nodes that are based on Euler parameters. The two joints add $2\times 5$ constraints and there are 2 additional Euler parameter constraints, giving a degree of freedom of 2 (as expected ...).

 You can try and duplicate the code for the second revolute joint:

```python
  #add a second constraint for bodies b0 and b1:
  mbs.CreateRevoluteJoint(bodyNumbers=[b0, b1], ...)
```

such that we have two identical joints (which would be unwanted, in general). This would give

```python
  ODE2 coordinates          = 14
  total constraints         = 17
  redundant constraints     = 5
  pure algebraic constraints= 0
  degree of freedom         = 2
```

which clearly shows the 5 redundant constraints, which will lead to a solver failure (except for the `EigenDense` solver, see there). In practical cases, redundant constraints may be much more involved, but can be detected in this way.

 A graphical representation of the internal structure of the model can be shown using the command `DrawSystemGraph`:

```python
  mbs.DrawSystemGraph(useItemTypes=True) #draw nice graph of system
```

For the output see {ref}`fig-drawsystemgraphexample`.
Note that obviously, markers are always needed to connect objects (or nodes) as well as loads. We can also see, that 2 markers MarkerBodyRigid1 and MarkerBodyRigid2 are unused, which is no further problem for the model and also does not require additional computational resources (except for some bytes of memory). Having isolated nodes or joints that are not connected (or having too many connections) may indicate that you did something wrong in setting up your model.
Furthermore, it can be seen that the function `CreateRigidBody` added a body `ObjectRigidBody`, a node `NodeRigidBodyEP`, a `LoadMassProportional` for gravity load with a `MarkerBodyMass`, and the function `CreateRevoluteJoint` created two `MarkerBodyRigid` and a `ObjectJointRevoluteZ` which represents a revolute joint about a Z-axis in the joint coordinate system. For further information, consult the respective pages in the Items reference manual.

(fig-drawsystemgraphexample)=
```{figure} /docs/theDoc/figures/DrawSystemGraphExample.png
:width: 600

System graph for rigid body tutorial (with option 3 for the first revolute joint). Numbers are always related to the node number, object number, etc.; note that colors are used to distinguish nodes, objects, markers, loads and sensors
```

---
\
 Before starting our simulation, we should adjust the solver parameters, especially the end time and the step size (no automatic step size for implicit solvers available!):

```python
  simulationSettings = exu.SimulationSettings() #takes currently set values or default values

  tEnd = 4 #simulation time
  h = 1e-3 #step size
  simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
  simulationSettings.timeIntegration.endTime = tEnd
  simulationSettings.timeIntegration.verboseMode = 1
  #simulationSettings.timeIntegration.simulateInRealtime = True
  simulationSettings.solutionSettings.solutionWritePeriod = 0.005 #store every 5 ms
```

The `verboseMode` tells the solver the amount of output during solving. Higher values (2, 3, ...) show residual vectors, jacobians, etc. for every time step, but slow down simulation significantly.
The option `simulateInRealtime` is used to view the model during simulation, while setting this false,
the simulation finishes after fractions of a second. It should be set to false in general,
while solution can be viewed using the `SolutionViewer()`.
With `solutionWritePeriod` you can adjust the frequency which is used to store the solution of the whole model,
which may lead to very large files and may slow down simulation, but is used in the `SolutionViewer()` to reload the solution after simulation.

 In order to improve visualization, there are hundreds of options, see Visualization settings in {ref}`sec-visualizationsettingsmain`, some of them used here:

```python
  SC.visualizationSettings.view0.window.renderWindowSize = [1600,1200]
  SC.visualizationSettings.openGL.multiSampling = 4  #improved OpenGL rendering
  SC.visualizationSettings.general.autoFitScene = False

  SC.visualizationSettings.nodes.drawNodesAsPoint = False
  SC.visualizationSettings.nodes.showBasis = True #shows three RGB (=xyz) lines for node basis
```

The option `autoFitScene` is used in order to avoid zooming while loading the last saved render state, see below.

 We can start the 3D visualization (Renderer) now:

```python
  SC.renderer.Start()
```

 In order to reload the model view of the last simulation (if there is any), we can use the following commands:

```python
  if 'renderState' in exu.sys: #reload old view
      SC.renderer.SetState(exu.sys['renderState'])

  SC.renderer.DoIdleTasks()    #stop before simulating
```

the function `WaitForUserToContinue()` waits with simulation until we press SPACE bar. This allows us to make some pre-checks.

 Finally, the **index 2** (velocity level) implicit time integration (simulation) is started with:

```python
  mbs.SolveDynamic(simulationSettings = simulationSettings,
                   solverType = exu.DynamicSolverType.TrapezoidalIndex2)
```

This solver is used in the present example, but should be considered with care as it leads to (small) drift of position constraints, linearly increasing in time. Using sufficiently small time steps, this effect is often negligible on the advantage of having a **energy-conserving integrator** (guaranteed for linear systems, but very often also for the nonlinear multibody system). Due to the velocity level, the integrator is less sensitive to consistent initial conditions on position level and compatible to frequent step size changes, however, initial jumps in velocities may never damp out in undamped systems.

 Alternatively, an **index 3** implicit time integration -- the generalized-$\alpha$ method -- is started with the default settings for `solverType`:

```python
  mbs.SolveDynamic(simulationSettings = simulationSettings)
```

Note that the **generalized-$\alpha$ method** includes numerical damping (adjusted with the spectral radius) for stabilization of index 3 constraints. This leads to effects every time the integrator is (re-)started, e.g., when adapting time step sizes. For fixed step sizes, this is **the recommended integrator**.

 After simulation, the library would immediately exit (and jump back to iPython or close the terminal window). In order to avoid this, we can use `WaitForRenderEngineStopFlag()` to wait until we press key 'Q'.

```python
  SC.renderer.DoIdleTasks()   #stop before closing
  SC.renderer.Stop()          #safely close rendering window!
```

If you entered everything correctly, the render window should show a nice animation of the 3D double pendulum after pressing the SPACE key.
If we do not stop the renderer (`SC.renderer.Stop()`), it will stay open for further simulations. However, it is safer to always close the renderer at the end.

 As the simulation will run very fast, if you did not set `simulateInRealtime` to true. However, you can reload the stored solution and view the stored steps interactively:

```python
  mbs.SolutionViewer()
  #alternatively, we could load solution from a file:
  #from exudyn.utilities import LoadSolutionFile
  #sol = LoadSolutionFile('coordinatesSolution.txt')
  #mbs.SolutionViewer(sol)
```

 Finally, we can plot our sensor, drawing the y-component of the sensor (check out the many options in `PlotSensor(...)` to conveniently represent results!):

```python
  mbs.PlotSensor(sensorNumbers=[sens1],components=[1],closeAll=True)
```

 **Congratulations**! You completed the rigid body tutorial, which gives you the ability to model multibody systems. Note that much more complicated models are possible, including feedback control or flexible bodies, see the Examples!

## Flexible beams tutorial

This tutorial briefly introduces two simple planar beams and how to work with them with utility functions.
The python source code of the beam tutorial can be found at:

- `python/Examples/beamTutorial.py`

The tutorial uses the GeometricallyExactBeam2D, which is a shear deformable Reissner-Timoshenko beam, and a thin cable ANCFCable2D, which represents a large deformation Bernoulli-Euler beam.

 The model includes two highly flexible planar with length 2m, height 0.005m, width 0.01m,
Young's modulus 1e9N/m$^2$ and density 2000kg/m$^3$.
The shear deformable beam is rigidly attached to ground and the cable is rigidly attached to a moving ground.

 We first import necessary libraries and set up a mbs. Note that utilities also include pi, sin, cos and sqrt.

```python
  import exudyn as exu
  from exudyn.utilities import *
  import numpy as np

  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
```

 We define a set of beam parameters, discretization and geometry for both beam models.

```python
  numberOfElements = 16
  L = 2                     # length of pendulum
  E=2e11                    # steel
  rho=7800                  # elastomer
  h=0.005                   # height of rectangular beam element in m
  b=0.01                    # width of rectangular beam element in m
  A=b*h                     # cross sectional area of beam element in m^2
  I=b*h**3/12               # second moment of area of beam element in m^4
  nu = 0.3                  # Poisson's ratio

  EI = E*I
  EA = E*A
  rhoA = rho*A
  rhoI = rho*I
  ks = 10*(1+nu)/(12+11*nu) # shear correction factor
  G = E/(2*(1+nu))          # shear modulus
  GA = ks*G*A               # shear stiffness of beam

  g = [0,-9.81,0]           # gravity vector

  positionOfNode0 = [0,0,0] # 3D vector
  positionOfNode1 = [L,0,0] # 3D vector
```

 In order to create beams, we usually need to create 2D rigid body nodes,
create beam elements, add constraints and loads.

 However, there is a utility function `GenerateStraightBeam(...)`,
which conveniently does this for straight beams, including discretization, constraints and gravity.
First, we create a beam template, which includes all beam parameters (this could also be another beam type):

```python
  beamTemplate = Beam2D(nodeNumbers = [-1,-1], #added later
                        physicsMassPerLength=rhoA,
                        physicsCrossSectionInertia=rhoI,
                        physicsBendingStiffness=EI,
                        physicsAxialStiffness=EA,
                        physicsShearStiffness=GA,
                        physicsBendingDamping=0.02*EI,
                        visualization=VObjectBeamGeometricallyExact2D(drawHeight = h))
```

 Now we use this template and call `GenerateStraightBeam`, which takes the nodal positions,
calculates according beam element lengths from discretization and could add constraints,
if `fixedConstraintsNode0` or `fixedConstraintsNode1` are not `None`.

```python
  beamData = GenerateStraightBeam(mbs, positionOfNode0, positionOfNode1,
                                  numberOfElements, beamTemplate, gravity= g,
                                  fixedConstraintsNode0=[1,1,1],
                                  fixedConstraintsNode1=None)
```

 We perform the same operations for ANCF cable elemente (Bernoulli-Euler),
but in this case, we do not add constraints:

```python
  beamTemplate = Cable2D(nodeNumbers = [-1,-1], #added later
                         physicsMassPerLength=rhoA,
                         physicsBendingStiffness=EI,
                         physicsAxialStiffness=EA,
                         physicsBendingDamping=0.02*EI,
                         visualization=VCable2D(drawHeight = h))

  cableData = GenerateStraightBeam(mbs, positionOfNode0, positionOfNode1,
                                   numberOfElements, beamTemplate, gravity= g,
                                   fixedConstraintsNode0=None,
                                   fixedConstraintsNode1=None)
```

 Now, we create a ground object and markers to attach cable with generic joint

```python
  oGround = mbs.CreateGround(referencePosition=[0,0,0])
  mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
  mCable = mbs.AddMarker(MarkerNodeRigid(nodeNumber=cableData[0][0]))
```

 As we like to move the cable relative to ground, we employ a simple
user function which prescribes relative rotation and (corotated) translation in the joint:

```python
  def UFoffset(mbs, t, itemNumber, offsetUserFunctionParameters):
      x = SmoothStep(t, 2, 4, 0, 0.5)   #translate in local joint coordinates
      phi = SmoothStep(t, 5, 10, 0, pi) #rotates frame of mGround
      return [x, 0,0,0,0,phi]
```

 Finally, we add the rigid joint (2D displacements and rotation around Z fixed) as GenericJoint.
Note that for 2D objects, we may only fix $X$- and $Y$-translations, as well as the $Z$-rotation

```python
  mbs.AddObject(GenericJoint(markerNumbers=[mGround, mCable],
                             constrainedAxes=[1,1,0, 0,0,1],
                             offsetUserFunction=UFoffset,
                             visualization=VGenericJoint(axesRadius=0.01,
                                                         axesLength=0.02)))
```

 As in the previous example, we just need to assemble and set up simulation parameters:

```python
  mbs.Assemble()

  simulationSettings = exu.SimulationSettings()

  tEnd = 10
  stepSize = 0.005
  simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
  simulationSettings.timeIntegration.endTime = tEnd
  simulationSettings.timeIntegration.verboseMode = 1
  simulationSettings.solutionSettings.solutionWritePeriod = 0.005
  simulationSettings.solutionSettings.writeSolutionToFile = True

  simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
  simulationSettings.timeIntegration.newton.useModifiedNewton = True #for faster simulation

  ## add some visualization settings
  SC.visualizationSettings.nodes.defaultSize = 0.01
  SC.visualizationSettings.nodes.drawNodesAsPoint = False #show beam nodes
  SC.visualizationSettings.bodies.beams.crossSectionFilled = True
```

 Now start the solver with visualization and run the solution viewer afterwards,
because simulation may be faster than you can follow:

```python
  SC.renderer.Start()
  mbs.SolveDynamic(simulationSettings)
  SC.renderer.Stop()

  ## visualize computed solution:
  mbs.SolutionViewer()
```

 The visualization window for the solution drawn at 6.5s is shown in {ref}`fig-tutorial-beams`.

(fig-tutorial-beams)=
```{figure} /docs/theDoc/figures/TutorialBeams.png
:width: 500

Render window showing the deformed state of the two beams at 6.5s. The lower beam is fixed at the left end, while the upper beam's support is translated and rotated.
```

 This example should give you a good starting point to create beam models.
See further examples, and more advanced functions, e.g., to create curved beam or reeving systems.
There are also a sliding joint as well as axially moving beams and contact between beams and sheaves.

 3D beams are still under development and include less functionality. In case, send a request at the GitHub discussion or issues.

## Symbolic user function tutorial

The following tutorial demonstrates the setup of a nonlinear oscillator with a mass point and a force user function, using a Cartesian spring damper defined by symbolic user functions.
\
First, we import the necessary libraries, create a system container and a main system:

```python
  import exudyn as exu
  from exudyn.utilities import *  # includes itemInterface and rigidBodyUtilities
  import exudyn.graphics as graphics  # only import if it does not conflict
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
  simulationSettings.solutionSettings.writeSolutionToFile = False
  simulationSettings.solutionSettings.sensorsWritePeriod = 0.001
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

## Flexible body -- FFRF tutorial

The following tutorial includes flexible bodies, using the floating frame of reference formulation (FFRF),
including Netgen and NGsolve for mesh and finite element data generation and uses modal reduction for simulation.
The tutorial will set up a body with Hurty-Craig-Bampton modes, giving a simple flexible pendulum meshed hinged with a revolute joint.

(fig-tutorial-ffrfpendulum)=
```{figure} /docs/theDoc/figures/TutorialFFRFpendulum.png
:width: 400

Screen shot of pendulum modeled with floating frame of reference formulation, using HCB modes, and meshed with Netgen.
```

\
We import the exudyn library, utilities, and other necessary modules:

```python
    import exudyn as exu
    from exudyn.utilities import *  # includes itemInterface and rigidBodyUtilities
    import exudyn.graphics as graphics  # only import if it does not conflict
    from exudyn.FEM import *
    import numpy as np
    import time
    import ngsolve as ngs
    from netgen.meshing import *
    from netgen.csg import *
```

Next, we need a `SystemContainer`, which contains all computable systems and adds a new MainSystem `mbs`:

```python
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
```

Define the parameters and setup the Netgen mesh, using Netgen's CSG geometry. We create a simple brick in order to simplify the application of boundary interfaces and joints:

```python
    useGraphics = True
    fileName = 'testData/netgenBrick'  # for load/save of FEM data

    a = 0.025  # height/width of beam
    b = a
    h = 0.5 * a
    L = 1     # Length of beam
    nModes = 8

    rho = 1000
    Emodulus = 1e7 * 10
    nu = 0.3
    meshCreated = False
    meshOrder = 1  # use order 2 for higher accuracy, but more unknowns

    geo = CSGeometry()
    block = OrthoBrick(Pnt(0, -a, -b), Pnt(L, a, b))
    geo.Add(block)
    mesh = ngs.Mesh(geo.GenerateMesh(maxh=h))
    mesh.Curve(1)
```

When creating the geometry and mesh, we sometimes would like to verify that with Netgen's GUI.
In Jupyter, this works smoother with `webgui_jupyter_widgets` -- see the Netgen documention. Here we use a simple loop, which has to set `True` if visualization shall run:

```python
    if False:  # set this to true, if you want to visualize the mesh inside netgen/ngsolve
        import netgen.gui
        ngs.Draw(mesh)
        for i in range(10000000):
            netgen.Redraw()  # this makes the window interactive
            time.sleep(0.05)
```

Use the FEM interface to import the FEM model and create the FFRF reduced-order data stored in fem.
Note that on importing the FEM structure from NGsolve (the FEM-module and solver of Netgen), we
have to specify the mechanical properties related to the mesh:

```python
    fem = FEMinterface()
    [bfM, bfK, fes] = fem.ImportMeshFromNGsolve(mesh, density=rho,
                                                youngsModulus=Emodulus,
                                                poissonsRatio=nu,
                                                meshOrder=meshOrder)
```

We could now just compute eigenmodes of the free bodies. However, as mentioned in the theory part, they do not respect boundary conditions and lead to low accuracy. Therefore, we use Hurty-Craig-Bampton modes.
For them, we have to define boundary interfaces given as lists of node numbers as well as weights.
We can use convenient functions from the `FEMinterface` class to retrieve nodes from planar or cylindrical surfaces (or we may define them in the finite element model ourselves):

```python
    pLeft = [0, -a, -b]
    pRight = [L, -a, -b]
    nTip = fem.GetNodeAtPoint(pRight) #for sensor
    nodesLeftPlane = fem.GetNodesInPlane(pLeft, [-1, 0, 0])
    weightsLeftPlane = fem.GetNodeWeightsFromSurfaceAreas(nodesLeftPlane)
    nodesRightPlane = fem.GetNodesInPlane(pRight, [-1, 0, 0])
    weightsRightPlane = fem.GetNodeWeightsFromSurfaceAreas(nodesRightPlane)
```

We define a list of boundaries, which are then passed to the function which computes modes.
Note that by default the first boundary modes are eliminated as they are fixed to the reference frame
of the FFRF object:

```python
    boundaryList = [nodesLeftPlane]

    print("nNodes=", fem.NumberOfNodes())
    print("compute HCB modes... ")
    start_time = time.time()
    fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryList,
                                      nEigenModes=nModes,
                                      useSparseSolver=True,
                                      computationMode=HCBstaticModeSelection.RBE2)
    print("HCB modes needed
```

Compute stress modes for postprocessing, which is not needed for simulation, but useful for postprocessing:

```python
    if True:
        mat = KirchhoffMaterial(Emodulus, nu, rho)
        varType = exu.OutputVariableType.StressLocal
        print("ComputePostProcessingModes ... (may take a while)")
        start_time = time.time()
        fem.ComputePostProcessingModesNGsolve(fes, material=mat,
                                              outputVariableType=varType)
        print("   ... needed
        SC.visualizationSettings.contour.reduceRange = False
        SC.visualizationSettings.contour.outputVariable = varType
        SC.visualizationSettings.contour.outputVariableComponent = 0  # x-component
    else:
        SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
        SC.visualizationSettings.contour.outputVariableComponent = 1
```

Having all data prepared now, we create the CMS element which is an object added then to `mbs`:

```python
    print("create CMS element ...")
    cms = ObjectFFRFreducedOrderInterface(fem)
    objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0, 0, 0],
                                            initialVelocity=[0, 0, 0],
                                            initialAngularVelocity=[0, 0, 0],
                                            gravity=[0, -9.81, 0],
                                            color=[0.1, 0.9, 0.1, 1.])
```

Add markers and revolute joint, using a superelement marker:

```python
    nodeDrawSize = 0.0025  # for joint drawing

    mRB = mbs.AddMarker(MarkerNodeRigid(nodeNumber=objFFRF['nRigidBody']))
    oGround = mbs.AddObject(ObjectGround(referencePosition=[0, 0, 0]))
    leftMidPoint = [0, 0, 0]
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=leftMidPoint))
    mLeft = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'],
                                                  meshNodeNumbers=np.array(nodesLeftPlane),
                                                  weightingFactors=weightsLeftPlane))
    mbs.AddObject(GenericJoint(markerNumbers=[mGround, mLeft],
                               constrainedAxes=[1, 1, 1, 1, 1, 1 * 0],
                               visualization=VGenericJoint(axesRadius=0.1 * a, axesLength=0.1 * a)))
```

Note that the marker is using weights, which are needed to compute accurate average (midpoint) positions from the non-uniform triangular surface mesh.

 Now we finally add a sensor and assemble the system:

```python
    fileDir = 'solution/'
    sensTipDispl = mbs.AddSensor(SensorSuperElement(bodyNumber=objFFRF['oFFRFreducedOrder'],
                                                    meshNodeNumber=nTip,
                                                    fileName=fileDir + 'nMidDisplacementCMS' + str(nModes) + 'Test.txt',
                                                    outputVariableType=exu.OutputVariableType.Displacement))

    mbs.Assemble()
```

Set simulation settings and run the simulation:

```python
    simulationSettings = exu.SimulationSettings()

    SC.visualizationSettings.nodes.defaultSize = nodeDrawSize
    SC.visualizationSettings.nodes.drawNodesAsPoint = False
    SC.visualizationSettings.connectors.defaultSize = 2 * nodeDrawSize
    SC.visualizationSettings.nodes.show = False
    SC.visualizationSettings.sensors.show = True
    SC.visualizationSettings.sensors.defaultSize = 0.01
    SC.visualizationSettings.markers.show = False
    SC.visualizationSettings.loads.drawSimplified = False

    h = 1e-3
    tEnd = 4

    simulationSettings.timeIntegration.numberOfSteps = int(tEnd / h)
    simulationSettings.timeIntegration.endTime = tEnd
    simulationSettings.timeIntegration.verboseMode = 1
    simulationSettings.timeIntegration.newton.useModifiedNewton = True
    simulationSettings.solutionSettings.sensorsWritePeriod = h
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
    simulationSettings.displayComputationTime = True

    mbs.SolveDynamic(simulationSettings=simulationSettings)

    uTip = mbs.GetSensorValues(sensTipDispl)[1]
    print("nModes=", nModes, ", tip displacement=", uTip)

    mbs.SolutionViewer()
```

When the solution viewer starts, it should show the stresses in a flexible swinging pendulum, see {ref}`fig-tutorial-ffrfpendulum`.

NOTE: this tutorial has been mostly created with ChatGPT-4, and curated hereafter!
