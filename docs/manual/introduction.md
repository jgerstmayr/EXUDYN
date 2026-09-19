(sec-overview)=
# Overview on Exudyn

This section provides a general overview on important parts of Exudyn. It is recommended to read these parts as it alleviates your life in creating models, understanding the behavior of the system and resolving errors.

(sec-overview-modulestructure)=
## Module structure

This section will show:

- Overview of modules
- Conventions: dimension of nodes, objects and vectors
- Coordinates: reference coordinates and displacements
- Nodes, Objects, Markers and Loads

For an introduction to the solvers, see {ref}`sec-solvers`.

(fig-exudyn-candpython)=
```{figure} /docs/theDoc/figures/overviewExudynModules.png
:width: 350

Overview on Exudyn C++ and Python modules
```

(sec-overview-overviewmodules)=
### Overview of modules

Currently, the Exudyn module structure is split into a C++ core part and a set of
Python parts, see {ref}`fig-exudyn-candpython`.

- **C++ parts**, see {ref}`fig-exudyn-cpp` and {ref}`fig-system-overview`:
  - `exudyn`: on this level, there are just very few functions: `SystemContainer()`, `SC.renderer.Start()`, `SC.renderer.Stop()`, `SolveStatic(...)`, `SolveDynamic(...)`, ... as well as system and user variable dictionaries `exudyn.variables` and `exudyn.sys`
  - `config`, `special`: substructures for configuration (global settings) and special settings; use e.g. `exudyn.config.outputPrecision=4`
  - `symbolic`: tools for symbolic computation in user functions (speedup!)
  - `SystemContainer`: contains the systems (most important), solvers (static, dynamics, ...), visualization settings
  - `MainSystem` `mbs`: {ref}`mbs <mbs>` created with `mbs = SC.AddSystem()`, this structure contains everything that defines a solvable multibody system; a large set of nodes, objects, markers, loads can added to the system, see {ref}`sec-item-reference-manual`;
  - `mbs.systemData`: contains the initial, current, visualization, ... states of the system and holds the items, see {ref}`fig-system-overview`
  - `SimulationSettings`: contains the systems (most important), solvers (static, dynamics, ...), visualization settings

- **Python parts** (this list is continuously extended, see {ref}`sec-pythonutilityfunctions`):
  - `exudyn.artificialIntelligence`: interface to stablebaselines, interface to pytorch training (coming soon)
  - `exudyn.basicUtilities`: contains basic helper classes, without importing numpy
  - `exudyn.beams`: helper functions for creation of beams along straight lines and curves, sliding joints, etc.
  - `exudyn.graphics`: provides some basic drawing utilities, definition of colors and basic drawing objects (including {ref}`STL <STL>` import); rotation/translation of graphicsData objects
  - `exudyn.interactive`: helper classes to create interactive models (e.g. for teaching or demos)
  - `exudyn.itemInterface`: contains the interface, which transfers Python classes (e.g., of a NodePoint) to dictionaries that can be understood by the C++ module
  - `exudyn.FEM`: everything related to finite element import and creation of model order reduction flexible bodies
  - `exudyn.lieGroupBasics`: a collection of Python functions for Lie group methods (SO3, SE3, log, exp, Texp, ...)
  - `exudyn.mainSystemExtensions`: mapping of some functions to MainSystem (mbs)
  - `exudyn.physics`: containing helper functions, which are physics related such as friction
  - `exudyn.plot`: contains PlotSensor(...), a very versatile interface to matplotlib and other valuable helper functions
  - `exudyn.processing`: methods for optimization, parameter variation, sensitivity analysis, etc.
  - `exudyn.rigidBodyUtilities`: contains important helper classes for creation of rigid body inertia, rigid bodies, and rigid body joints; includes helper functions for rotation parameterization, rotation matrices, homogeneous transformations, etc.
  - `exudyn.robotics`: submodule containing several helper modules related to manipulators (`robotics`, `robotics.models`), mobile robots (`robotics.mobile`), trajectory generation (`robotics.motion`), etc.
  - `exudyn.signalProcessing`: filters, FFT, etc.; interfaces to scipy and numpy methods
  - `exudyn.solver`: functions imported when loading `exudyn`, containing main solvers
  - `exudyn.utilities`: constains helper classes in Python and includes Exudyn main modules `basicUtilities`, `rigidBodyUtilities`, `graphics`, and `itemInterface`, which is recommended to be loaded at beginning of your model file in order to have most necessary functionality at hand

(fig-exudyn-cpp)=
```{figure} /docs/theDoc/figures/overviewExudynCppModule.png
:width: 500

Overview on Exudyn C++ module
```

(fig-system-overview)=
```{figure} /docs/theDoc/figures/overviewSystemData.png
:width: 550

Overview of systemData
```

SystemData connects items, states and stores the {ref}`LTG <LTG>`. Note that access to items is provided via functions in `MainSystem`.

(sec-overview-conventionsitems)=
### Conventions: items, indexes, coordinates

In this documentation, we will use the term **item** to identify nodes, objects, markers, loads and sensors:

  item $\in$ \{node, object, marker, load, sensor\}

 **Indexes: arrays and vectors starting with 0:** \
As known from Python, all **indexes** of arrays, vectors, matrices, ... are starting with 0. This means that the first component of the vector `v=[1,2,3]` is accessed with `v[0]` in Python (and also in the C++ part of Exudyn ). The range is usually defined as `range(0,3)`, in which '3' marks the index after the last valid component of an array or vector.

**Dimensionality of objects and vectors:** \
{ref}`2D <2D>` vs. {ref}`3D <3D>`

As a convention, quantities in Exudyn are 3D, such as nodes, objects, markers, loads, measured quantities, etc.
For that reason, we denote planar nodes, objects, etc. with the suffix 2D, but 3D objects do not get this suffix (There are some rare exceptions, such as Beam3D as the pure beam may easily lead to name space conflicts in Python).

Output and input to objects, markers, loads, etc. is usually given by 3D vectors (or matrices), such as (local) position, force, torque, rotation, etc. However, initial and reference values for nodes depend on their dimensionality.
As an example, consider a `NodePoint2D`:

- `referenceCoordinates` is a 2D vector (but could be any dimension in general nodes)
- measuring the current position of `NodePoint2D` gives a 3D vector
- when attaching a `MarkerNodePosition` and a `LoadForceVector`, the force will be still a 3D vector

Furthermore, the local position in 2D objects is provided by a 3D vector. Usually, the dimensionality is given in the reference manual. User errors in the dimensionality will be usually detected either by the Python interface (i.e., at the time the item is created) or by the system-preprocessor

(sec-overview-items)=
## Items: Nodes, Objects, Loads, Markers, Sensors, ...

In this section, the most important part of Exudyn are provided. An overview of the interaction of the items is given in {ref}`fig-items-interaction`

(fig-items-interaction)=
```{figure} /docs/theDoc/figures/itemsMultibodySystem.png
:width: 500

Interaction of items in a multibody system
```

Note that both, bodies and connectors (including constraints) are -- computational -- objects. The arrows indicate, that, e.g., object 1 has node 1 and node 2 (indexes) and that marker 0 is attached to object 0, while load 0 uses marker 0 to apply the load. Sensors could additionally be attached to certain items.

### Nodes

Nodes provide the coordinates (and the degrees of freedom) to the system. They have no mass, stiffness or whatsoever assigned.
Without nodes, the system has no unknown coordinates.
Adding a node provides (for the system unknown) coordinates. In addition we also need equations for every nodal coordinate -- otherwise the system cannot be computed (NOTE: this is currently not checked by the preprocessor).
In general, adding nodes and objects (e.g., to represent rigid bodies), leads to a **redundant coordinate formulation**.
In order to determine the degree of freedom, you may use the Gr{\"u}bler-Kutzbach criterion. This can also be done numerically, using the function `ComputeSystemDegreeOfFreedom`, see the module `exudyn.solver`.
Furthermore, **minimal** coordinates can be used for open tree systems, using `ObjectKinematicTree`.

### Objects

Objects are 'computational objects' and they provide equations to your system. Objects often provide derivatives and have measurable quantities (e.g. displacement) and they provide access, which can be used to apply, e.g., forces. Some of this functionality is only available in C++, but not in Python.

Objects can be a:

- general object (e.g. a controller, user defined object, ...; no example yet)
- body: has a mass or mass distribution; markers can be placed on bodies; loads can be applied; constraints can be attached via markers; bodies can be:
  - ground object: has no nodes
  - simple body: has one node (e.g. mass point, rigid body)
  - finite element and more complicated body (e.g. FFRF-object): has more than one node

- connector: uses markers to connect nodes and/or bodies; adds additional terms to system equations either based on stiffness/damping or with constraints (and Lagrange multipliers). Possible connectors:
  - algebraic constraint (e.g. constrain two coordinates: $q_1 = q_2$)
  - classical joint
  - spring-damper or penalty constraint

### Markers

Markers are interfaces between objects/nodes and constraints/loads.
A constraint (which is also an object) or load cannot act directly on a node or object without a marker.
As a benefit, the constraint or load does not need to know whether it is applied, e.g., to a node or to a local position of a body.

Typical situations are:

- Node -- Marker -- Load
- Node -- Marker -- Constraint (object)
- Body(object) -- Marker -- Load
- Body1 -- Marker1 -- Joint(object) -- Marker2 -- Body2

### Loads

Loads are used to apply forces and torques to the system. The load values are static values. However, you can use Python functionality to modify loads either by linearly increasing them during static computation or by using the 'mbs.SetPreStepUserFunction(...)' structure in order to modify loads in every integration step depending on time or on measured quantities (thus, creating a controller).

### Sensors

Sensors are only used to measure output variables (values) in order to simpler generate the requested output quantities.
They have a very weak influence on the system, because they are only evaluated after certain solver steps as requested by the user.

(sec-overview-items-coordinates)=
### Reference coordinates and displacements

Nodes usually have separated reference and initial quantities.
Here, `referenceCoordinates` are the coordinates for which the system is defined upon creation.
Reference coordinates are needed, e.g., for definition of joints and for the reference configuration of finite elements. In many cases it marks the undeformed configuration (e.g., with finite elements), but not, e.g., for `ObjectConnectorSpringDamper`, which has its own reference length.

Initial displacement (or rotation) values are provided separately, in order to start a system from a configuration different from the reference configuration.
As an example, the initial configuration of a `NodePoint` is given by `referenceCoordinates + initialCoordinates`, while the initial state of a dynamic system additionally needs `initialVelocities`.
See also {ref}`sec-referenceandcurrentcoordinates`!

Note that commonly the `OutputVariableType` `Coordinates` returns coordinates without reference values, which are usually displacements (or changes in the rotation parameters), which is required if you are interested, e.g., in motion of finite element nodes.
In contrast the `OutputVariableType` `CoordinatesTotal` returns (Since Exudyn1.9.25) the sum of reference and displacement (or rotation) coordinates for any configuration (e.g., current, initial or visualization).

(sec-overview-ltgmapping)=
## Mapping between local and global coordinate indices

The {ref}`LTG <LTG>`-index-mappings (local-to-global coordinate index mappings containing transformation from local object coordinate indices to global (system) coordinate indices; this is different for **coordinate transformations**!) between local coordinate **indices**, on node or object level, and global (=system) coordinate **indices** follows the following rules:

- {ref}`LTG <LTG>`-index-mappings are computed during `mbs.Assemble()` and are not available before.
- Nodes own a global index which relates the local coordinates to global (system) coordinate. E.g., for a {ref}`ODE2 <ODE2>` node with node number `i`, this index can be obtained via the function `mbs.GetNodeODE2Index(i)`.
- The order of global coordinates is simply following the node numbering. If we add three nodes `NodePoint`, the system will contain 9 coordinates, where the first triple (starting index 0) belongs to node 0, the second triple (starting index 3) belongs to node 1 and the third triple (starting index 6) belongs to node 2. After `mbs.Assemble()`, you can access the system coordinates via `mbs.systemData.GetODE2Coordinates()`, which returns a numpy array with 9 coordinates, containing the initial values provided in `NodePoint` (default: zero).
- Objects have their own {ref}`LTG <LTG>`-index-mappings for their respective coordinate types. The {ref}`ODE2 <ODE2>` coordinates of an object `j` can be retrieved via `mbs.systemData.GetObjectLTGODE2(j)`. For a body, these are the global {ref}`ODE2 <ODE2>` coordinates representing the body; for a connector, these are the coordinates to which the connector is linked (usually coordinates of two bodies); for a ground object, the {ref}`LTG <LTG>`-index-mapping is empty; see also {ref}`sec-systemdata-objectltg`.
- Constraints create algebraic variables (Lagrange multipliers) automatically. For a constraint with object number `k`, the global index to algebraic variables (of {ref}`AE <AE>`-type) can be accessed via `mbs.systemData.GetObjectLTGAE(k)`.

(sec-overview-basics)=
## Exudyn Basics

This section will show:

- Interaction with the Exudyn module
- Simulation settings
- Visualization settings
- Generating output and results
- Graphics pipeline
- Generating animations

(sec-overview-basics-interactionmodule)=
### Interaction with the Exudyn module

It is important that the Exudyn module is basically a state machine, where you create items on the C++ side using the Python interface. This helps you to easily set up models using many other Python modules (numpy, sympy, matplotlib, ...) while the computation will be performed in the end on the C++ side in a very efficient manner.
\
**Where do objects live?**\
Whenever a system container is created with `SC = exu.SystemContainer()`, the structure `SC` becomes a variable in the Python interpreter, but it is managed inside the C++ code and it can be modified via the Python interface.
Usually, the system container will hold at least one system, usually called `mbs`.
Commands such as `mbs.AddNode(...)` add objects to the system `mbs`.
The system will be prepared for simulation by `mbs.Assemble()` and can be solved (e.g., using `exu.SolveDynamic(...)`) and evaluated hereafter using the results files.
Using `mbs.Reset()` will clear the system and allows to set up a new system. Items can be modified (`ModifyObject(...)`) after first initialization, even during simulation.

(sec-overview-basics-simulationsettings)=
### Simulation settings

The simulation settings consists of a couple of substructures, e.g., for `solutionSettings`, `staticSolver`, `timeIntegration` as well as a couple of general options -- for details see {ref}`sec-solutionsettings` and {ref}`sec-simulationsettingsmain`.

Simulation settings are needed for every solver. They contain solver-specific parameters (e.g., the way how load steps are applied), information on how solution files are written, and very specific control parameters, e.g., for the Newton solver.

 The simulation settings structure is created with

```python
  simulationSettings = exu.SimulationSettings()
```

Hereafter, values of the structure can be modified, e.g.,

```python
  tEnd = 10 #10 seconds of simulation time:
  h = 0.01  #step size (gives 1000 steps)
  simulationSettings.timeIntegration.endTime = tEnd
  #steps for time integration must be integer:
  simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
  #assigns a new tolerance for Newton's method:
  simulationSettings.timeIntegration.newton.relativeTolerance = 1e-9
  #write some output while the solver is active (SLOWER):
  simulationSettings.timeIntegration.verboseMode = 2
  #write solution every 0.1 seconds:
  simulationSettings.solutionSettings.solutionWritePeriod = 0.1
  #use sparse matrix storage and solver (package Eigen):
  simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
```

### Generating output and results

The solvers provide a number of options in `solutionSettings` to generate a solution file. As a default, exporting the solution of all system coordinates (on position, velocity, ... level) to the solution file is activated with a writing period of 0.01 seconds.

 Typical output settings are:

```python
  #create a new simulationSettings structure:
  simulationSettings = exu.SimulationSettings()

  #activate writing to solution file:
  simulationSettings.solutionSettings.writeSolutionToFile = True
  #write results every 1ms:
  simulationSettings.solutionSettings.solutionWritePeriod = 0.001

  #assign new filename to solution file
  simulationSettings.solutionSettings.coordinatesSolutionFileName= "myOutput.txt"

  #do not export certain coordinates:
  simulationSettings.solutionSettings.exportDataCoordinates = False
```

Furthermore, you can use sensors to record particular information, e.g., the displacement of a body's local
position, forces or joint data. For viewing sensor results, use the `PlotSensor` function of the
`exudyn.plot` tool, see the rigid body and joints tutorial.
Finally, the render window allows to show traces (trajectories) of position sensors, sensor vector quantities (e.g., velocity vectors),
or triads given by rotation matrices. For further information, see the `sensors.traces` structure of `VisualizationSettings`, {ref}`sec-vsettingstraces`.

(sec-overview-basics-renderer)=
### Renderer and 3D graphics

A 3D renderer is attached to the simulation. Visualization is started with  `SC.renderer.Start()`, see the examples and tutorials.
In order to show your model in the render window, you have to provide 3D graphics data to the bodies. Flexible bodies (e.g., FFRF-like) can visualize their meshes. Further items (nodes, markers, ...) can be visualized with default settings, however, often you have to turn on drawing or enlarge default sizes to make items visible. Item number can also be shown.
Finally, since version 1.6.188, sensor traces (trajectories) can be shown in the render window, see the `VisualizationSettings` in  {ref}`sec-visualizationsettingsmain`.

The renderer uses an OpenGL window of a library called GLFW, which is platform-independent.
The renderer is set up in a minimalistic way, just to ensure that you can check that the modeling is correct.

 **Note**:

- For closing the render window, press key 'Q' or Escape or just close the window.
- There is no way to contruct models inside the renderer (no 'GUI').
- Try to avoid huge number of triangles in STL files or by creating large number of complex objects, such as spheres or cylinders.
- After `visualizationSettings.general.reallyQuitTimeLimit` seconds a 'do you really want to quit' dialog opens for safety on pressing 'Q'; if no tkinter is available, you just have to press 'Q' twice. For closing the window, you need to click a second time on the close button of the window after `reallyQuitTimeLimit` seconds (usually 900 seconds).

 Here are the **main features of the renderer**, using keyboard and mouse, for details see {ref}`sec-graphicsvisualization`:

- press key H to show help in renderer
- move model by pressing left mouse button and drag
- rotate model by pressing right mouse button and drag
- for further mouse functionality, see {ref}`sec-gui-sec-mouseinput`
- change visibility (wire frame, solid, transparent, ...) by pressing T
- zoom all: key A
- open visualization dialog: key V, see {ref}`sec-overview-basics-visualizationsettings`
- open Python command dialog: key X, see {ref}`sec-overview-basics-commandandhelp`
- show item number: click on graphics element with left mouse button
- show item dictionary: click on graphics element with right mouse button
- for further keys, see {ref}`sec-gui-sec-keyboardinput` or press H in renderer
- raytracing mode, see {ref}`sec-overview-basics-raytracing`

Depending on your model (size, place, ...), you **may need to adjust the following general visualization** and `openGL` **parameters** in `visualizationSettings`, see {ref}`sec-visualizationsettingsmain`:

- change window size
- light and light position; switch `openGL.lightPositionsInCameraFrame` to switch between model-fixed or camera-fixed lights
- shadow (turned off by using shadow=0; turned on by using, e.g., a value of 0.3) and shadow polygon offset; shadow slows down graphics performance by a factor of 2-3, depending on your graphics card
- visibility of nodes, markers, etc. in according bodies, nodes, markers, ..., `visualizationSettings`
- move camera with a selected marker: adjust `trackMarker` in `visualizationSettings.interactive`

**NOTE**: changing `visualizationSettings` is not thread-safe, as it allows direct access to the C++ variables.
In most cases, this is not problematic, e.g., turning on/off some view parameters my just lead to some short-time artifacts if
they are changed during redraw. However, more advanced quantities (e.g., `trackMarker` or changing strings) may lead to problems,
which is why it is strongly recommended to:

- set all `visualizationSettings` **before start of renderer**

(sec-overview-basics-visualizationsettings)=
### Visualization settings dialog

Visualization settings are used for user interaction with the model. E.g., the nodes, markers, loads, etc., can be visualized for every model. There are default values, e.g., for the size of nodes, which may be inappropriate for your model. Therefore, you can adjust those parameters. In some cases, huge models require simpler graphics representation, in order not to slow down performance -- e.g., the number of faces to represent a cylinder should be small if there are 10000s of cylinders drawn. Even computation performance can be slowed down, if visualization takes lots of CPU power. However, visualization is performed in a separate thread, which usually does not influence the computation exhaustively.

Details on visualization settings and its substructures are provided in {ref}`sec-visualizationsettingsmain`. These settings may also be edited by pressing 'V' in the active render window (does not work, if there is no active render loop using, e.g., `SC.renderer.DoIdleTasks()` ).
The visualization settings dialog is shown exemplarily in {ref}`fig-visualizationsettings`.
Note that this dialog is automatically created and uses Python's `tkinter`, which is lightweight, but not very well suited if display scalings are large (e.g., on high resolution laptop screens). If working with Spyder, it is recommended to restart Spyder, if display scaling is changed, in order to adjust scaling not only for Spyder but also for Exudyn.

The appearance of visualization settings dialogs may be adjusted by directly modifying `exudyn.GUI` variables (this may change in the future). For example write in your code before opening the render window (treeEdit and treeview both mean the settings dialog currently used for visualization settings and partially for right-mouse-click):

```python
  import exudyn.GUI
  exudyn.GUI.dialogDefaultWidth             #unscaled width of, e.g., right-mouse-button dialog
  exudyn.GUI.treeEditDefaultWidth = 800
  exudyn.GUI.treeEditDefaultHeight = 600
  exudyn.GUI.treeEditMaxInitialHeight = 600 #otherwise height is increased for larger screens
  exudyn.GUI.treeEditOpenItems = ['general','contact'] #these tree items are opened each time the dialog is opened
  #
  exudyn.GUI.treeviewDefaultFontSize        #this is the base font size of the dialog (also right-mouse-button dialog)
  exudyn.GUI.useRenderWindowDisplayScaling  #if True, the scaling will follow the current scaling of the render window; if False, it will use the `tkinter` internal scaling, which uses the main screen where the dialog is created (which won't scale well, if the window is moved to another screen).
  #
  exudyn.GUI.textHeightFactor = 1.45        #this factor is used to increase height of lines in tree view as compared to font size
```

(fig-visualizationsettings)=
```{figure} /docs/theDoc/figures/visualizationSettings.png
:width: 700

View of visualization settings
```

Note: Press 'V' in render window to open dialog.

The visualization settings structure can be accessed in the system container `SC` (access per reference, no copying!), accessing every value or structure directly, e.g.,

```python
  SC.visualizationSettings.nodes.defaultSize = 0.001      #draw nodes very small

  #change openGL parameters; current values can be obtained from SC.renderer.GetState()
  #change zoom factor:
  SC.visualizationSettings.openGL.advanced.initialZoom = 0.2
  #set the center point of the scene (can be attached to moving object):
  SC.visualizationSettings.openGL.advanced.initialCenterPoint = [0.192, -0.0039,-0.075]

  #turn of auto-fit:
  SC.visualizationSettings.general.autoFitScene = False

  #change smoothness of a cylinder:
  SC.visualizationSettings.general.cylinderTiling = 100

  #make round objects flat:
  SC.visualizationSettings.openGL.advanced.shadeModelSmooth = False

  #turn on coloured plot, using y-component of displacements:
  SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Displacement
  SC.visualizationSettings.contour.outputVariableComponent = 1 #0=x, 1=y, 2=z
```

(sec-overview-basics-commandandhelp)=
### Execute Command and Help

In addition to the Visualization settings dialog, a simple help window opens upon pressing key 'H'.
It is also possible to execute single Python commands during simulation by pressing 'X', which opens a dialog, saying 'Exudyn Command Window'.
Note that the dialog may appear behind the visualization window!
This dialog may be very helpful in long running computations or in case that you may evaluate variables for debugging.
The Python commands are evaluated in the global python scope, meaning that `mbs` or other variables of your scripts are available.
User errors are caught by exceptions, but in severe cases this may lead to crash.
To print values, always use `print(...)` to see the string representation of an object.

 Useful examples (single lines) may be:

```python
  x=5 #or change any other variable used in Python user functions
  print(mbs) #print current mbs overview
  print(mbs.GetSensorValues(0))
  #adjust simulation end time, in long-run simulations:
  mbs.sys['dynamicSolver'].it.endTime = 1
  #adjust output behavior
  mbs.sys['dynamicSolver'].output.verboseMode = 0
```

 You can also do quite fancy things during simulation, e.g., to deactivate joints (of course this may result in strange behavior):

```python
  n=mbs.systemData.NumberOfObjects()
  for i in range(n):
      d = mbs.GetObject(i)
      #if 'Joint' in d['objectType']:
      if 'activeConnector' in d:
          mbs.SetObjectParameter(i, 'activeConnector', False)
```

Note that you could also change `visualizationSettings` in this way, but the Visualization settings dialog is much more convenient.
Changing `simulationSettings` within the execute command is dangerous and must be treated with care.

Some parameters, such as `simulationSettings.timeIntegration.endTime` are copied into the internal solver's `mbs.sys['dynamicSolver'].it` structure.

Thus, changing `simulationSettings.timeIntegration.endTime` has no effect during simulation.
As a rule of thumb, all variables that are not stored inside the solvers structures may be adjusted by the `simulationSettings` passed to the solver (which are then not copied internally); see the C++ code for details. However, behavior may change in future and unexpected behavior or and changing `simulationSettings` will likely cause crashes if you do not know exactly the behavior, e.g., changing output format from text to binary ... !
Specifically, `newton` and `discontinuous` settings cannot be changed on the fly as they are copied internally.

(sec-overview-basics-graphicspipeline)=
### Graphics pipeline

There are basically two loops during simulation, which feed the graphics pipeline.
The solver runs a loop:

- compute step (or set up initial values)
- finish computation step; results are in current state
- copy current state to visualization state (thread safe)
- signal graphics pipeline that new visualization data is available
- the renderer may update the visualization depending on `graphicsUpdateInterval` in \ `visualizationSettings.general`

The openGL graphics thread (=separate thread) runs the following loop:

- render openGL scene with a given graphicsData structure (containing lines, faces, text, ...)
- go idle for some milliseconds
- check if openGL rendering needs an update (e.g. due to user interaction)
- $\ra$ if update is needed, the visualization of all items is updated -- stored in a graphicsData structure)
- check if new visualization data is available and the time since last update is larger than a presribed value, the graphicsData structure is updated with the new visualization state

(sec-overview-basics-raytracing)=
### Raytracing

In order to compensate the limited functionality (but high compatibility) of OpenGL 1.3, an option for CPU-based software rendering (raytracing) has been added.
This allows to include shadows and transparency correctly, with additional support for relections, refraction, emission, fog and materials.
In the future, textures may be added as well.

(fig-raytracerdemo)=
```{figure} /docs/theDoc/figures/raytracerDemo.jpg
:width: 400

Example image of raytraced renderer view.
```

The basic things to know are:

- Raytracing settings are collected in `SC.visualizationSettings.raytracer` (in the following, we omit 'SC.visualizationSettings').
- Raytracing is activated by setting `raytracer.enable=True`. Please make sure that you start with small render window sizes / complexity first.
- The render window size is adjusted by `window.renderWindowSize`. Be careful with this settings.
- Adjust the `raytracer.numberOfThreads` for optimal performance, use `raytracer.verbose` to see render times for different settings. For testing, use `raytracer.imageSizeFactor>1` to decrease the raytracer's resolution (with same image size), while `openGL.multiSampling` will increase the resolution (anti-aliasing). Note that switching from `openGL.multiSampling=1` and `raytracer.imageSizeFactor=4` to `openGL.multiSampling=3` and `raytracer.imageSizeFactor=1` increases computational costs by a factor $4\times 4\times 3\times 3 = 144$. Use only one light, if sufficient (set `openGL.enableLight1=False`).
- Scene, lights, shadow, clipping plane, etc. settings are taken form OpenGL settings and directly used in the software renderer, like `openGL.light0position`, `openGL.shadow`, `openGL.perspective`, `openGL.clippingPlaneNormal`, `openGL.showLines`, etc.;
- some settings are in general, like `general.backgroundColor` or `general.drawWorldBasis`;
- In order to see the advantages of the software renderer, materials have to be used, see below.

 **Materials**:

- Materials have the type `VSettingsMaterial`, see description in {ref}`sec-vsettingsmaterial`, for adjusting color, reflectivity, shininess, alpha-transparency, etc.;
- Materials can only be used within triangulated geometries (GraphicsData `TriangleList`) using a material-flag in the color, like `graphics.Sphere(..., color=graphics.material.chrome)`. In the RGBA color, the alpha-channel is replaced by a material index which starts at 1000 (where 1000 represents material index 0). Note that in the regular OpenGL-rendering, alpha$>$1 is equivalent to alpha$=$1. The first 10 materials are linked to `raytracer.material0 ... raytracer.material9`.
- The material's `baseColor` is used if the color red-channel is set to $-1$. Note that this allows to globally change the color of objects by changing `baseColor` in the material settings in `visualizationSettings`. Summarizing, using `color=[1,0,0,graphics.material.indexSteel]` chooses red color with steel material settings, while `color=[-1,-1,-1,graphics.material.indexSteel]` will use the color of steel (but will be black for OpenGL renderer), identical with `color=graphics.material.steel`.
- The setting `backgroundColorReflections` can be used to represent the background which is used for rendering, while the background is independently set to black or white. Otherwise, black background leads to black regions on highly reflective objects or very light regions for white backgrounds.
- System text messages (solver, version, etc.) are overlayed over raytracing and can be turned off using the settings in `general.showComputationInfo` and similar. However, note that **item texts are currently not shown** in raytracer, affecting node numbers, etc.!

 **Limitations and risks**:

- Raytracing is CPU-based and therefore slow. Do not use very high resolution (4K) together with multisampling $>1$. Start with small render window sizes (e.g. 600 $\times$ 400)
- Raytracing usually uses multithreading with speedups $>10$ on 16 cores. However, this cannot be combined with multithreaded simulations. It is therefore recommended to use raytracing in the solution viewer, not during simulation.
- If software rendering of a single frame gets to long (>4 seconds), timeouts become active and it may occasionally not work. There are some options to compensate, see above.
- In general, it is **recommended to start with default settings and experiment** with changes using the visualization settings dialog.

 To add raytracing to your project, do like this:

```python
  ...
  #sphere with chrome
  graphics.Sphere(radius=radius,
                  color=graphics.color.dodgerblue[0:3]+[graphics.material.indexChrome],
                  nTiles=32)
  ground = mbs.CreateGround(referencePosition=[0,0,0],
                            graphicsDataList=[gSphere])

  #add mbs components
  #assemble
  #solve
  ...
  #after computation, switch to raytracing
  SC.visualizationSettings.openGL.multiSampling = 1
  SC.visualizationSettings.openGL.imageSizeFactor = 3 #reduce resolution for first tests!
  SC.visualizationSettings.openGL.light1.enable = False
  SC.visualizationSettings.raytracer.numberOfThreads = 16 #adjust to your n-threads
  SC.visualizationSettings.view0.camera.useRaytracer = True

  mbs.SolutionViewer()
```

 Have fun!

(sec-overview-basics-storingmodelview)=
### Storing the model view

The **simplest way to store the model view** is to **press CTRL-F3** when the renderer is running, to get the code for setting the model view printed to the console, e.g.,

- `Set current view: SC.renderer.SetModelView(zoom=8.8,rotationVector=`\ `[-0.8120557,0.4727261,0.7176849],centerPoint=[1.562,-1.526,0])`

Then, just copy the code after `SC.renderer.Start`, see the following code snippet:

```python
  import exudyn as exu
  SC=exu.SystemContainer()
  SC.visualizationSettings.general.autoFitScene = False #prevent from autozoom
  SC.renderer.Start()
  SC.renderer.SetModelView(zoom=8.8,rotationVector=[-0.8120557,0.4727261,0.7176849],centerPoint=[1.562,-1.526,0])
  #+++++++++++++++
  #do simulation here
  #+++++++++++++++
  SC.renderer.Stop()
```

---

If you are using an interactive Python, there is a automated way to store and restore the current view (zoom, centerpoint, orientation, etc.) by using `SC.renderer.GetState()` and `SC.renderer.SetState()`,
see also {ref}`sec-renderstate`.
A simple way is to reload the stored render state (model view) after simulating your model once at the end of the simulation (note that `visualizationSettings.general.autoFitScene` should be set False if you want to use the stored zoom factor):

```python
  import exudyn as exu
  SC=exu.SystemContainer()
  SC.visualizationSettings.general.autoFitScene = False #prevent from autozoom
  SC.renderer.Start()
  if 'renderState' in exu.sys:
      SC.renderer.SetState(exu.sys['renderState'])
  #+++++++++++++++
  #do simulation here and adjust model view settings with mouse
  #+++++++++++++++

  #store model view for next run:
  SC.renderer.Stop() #stores render state in exu.sys['renderState']
```

---
 \
Whenever `SC.renderer.Start()` is called, the renderState is reset (because it is assumed that the model has been changed and the previous view is invalid). However, you always can store and restore the renderstate manually.
Since version 1.10.98, the `ZoomAll` and `SetModelView` also work without starting the renderer (using only the raytracer). However, note that `ZoomAll` and `SetModelView` have to be called before the raytracer call RedrawAndGetImage(True) or after renderer.Start() using regular OpenGL.

If you wish to include all details of your view, like to rotation, you can obtain the current model view from the console after a simulation, e.g.,

```python
  In[1] : SC.renderer.GetState()
  Out[1]:
  {'centerPoint': [1.0, 0.0, 0.0],
   'maxSceneSize': 2.0,
   'zoom': 1.0,
   'currentWindowSize': [1024, 768],
   'modelRotation': [[ 0.34202015,  0.        , 0.9396926 ],
                     [-0.60402274,  0.76604444, 0.21984631],
                     [-0.7198463 , -0.6427876 , 0.26200265]])}
```

which contains the last state of the renderer (NOTE: here, only part of the render state is shown for simplicity!).
Now copy the output and set this with `SC.renderer.SetState` in your Python code to have a fixed model view in every simulation (`SC.renderer.SetState` AFTER `SC.renderer.Start()`):

```python
  SC.visualizationSettings.general.autoFitScene = False #prevent from autozoom
  SC.renderer.Start()
  renderState={'centerPoint': [1.0, 0.0, 0.0],
               'maxSceneSize': 2.0,
               'zoom': 1.0,
               'currentWindowSize': [1024, 768],
               'modelRotation':     [[ 0.34202015,  0.        ,  0.9396926 ],
                                    [-0.60402274,  0.76604444,  0.21984631],
                                    [-0.7198463 , -0.6427876 ,  0.26200265]])
  SC.renderer.SetState(renderState)
  #.... further code for simulation here
```

Note that in the current version of Exudyn there is more data stored in render state, which is not used in `SC.renderer.SetState`,
see also {ref}`sec-renderstate`.

---

### Graphics user functions via Python

There are some user functions in order to customize drawing:

- You can assign graphicsData to the visualization to most bodies, such as rigid bodies in order to change the shape. Graphics can also be imported from files (`exu.graphics.FromSTLfileASCII`, `exu.graphics.FromSTLfile`, ) using the established format {ref}`STL <STL>` (STereoLithography or Standard Triangle Language; file format available in nearly all CAD systems).
- Some objects, e.g., `ObjectGenericODE2` or `ObjectRigidBody`, provide customized a function `graphicsDataUserFunction`. This user function just returns a list of GraphicsData, see {ref}`sec-graphicsdata`. With this function you can change the shape of the body in every step of the computation.
- Specifically, the `graphicsDataUserFunction` in `ObjectGround` can be used to draw any moving background in the scene.

Note that all kinds of `graphicsDataUserFunction`s need to be called from the main (=computation) process as Python functions may not be called from separate threads (GIL). Therefore, the computation thread is interrupted to execute the `graphicsDataUserFunction` between two time steps, such that the graphics Python user function can be executed. There is a timeout variable for this interruption of the computation with a warning if scenes get too complicated.

(sec-overview-basics-colorrgba)=
### Color, RGBA and alpha-transparency

Many functions and objects include color information. In order to allow alpha-transparency, all colors contain a list of 4 RGBA values, all values being in the range [0..1]:

- red (R) channel
- green (G) channel
- blue (B) channel
- alpha (A) value, representing the so-called **alpha-transparency** (A=0: fully transparent, A=1: solid)

E.g., red color with no transparency is obtained by the color=[1,0,0,1].
Color predefinitions are found in `graphics.py`, e.g., using `graphics.color.red` or `graphics.color.steelblue` as well a list of 16 colors `graphics.colorList`, which is convenient to be used in a loop creating objects.
Earlier, special colors were given in `exudyn.graphicsDataUtilities.py`, e.g., `color4red` or `color4steelblue` as well as `color4list`, which are marked as deprecated.

(sec-overview-basics-solutionviewer)=
### Solution viewer

Exudyn offers a convenient WYSIWYS -- 'What you See is What you Simulate' interface, showing you the computation results during simulation in the render window.
If you are running large models, it may be more convenient to watch results after simulation has been finished.
For this, you can use

- `interactive.SolutionViewer`, see {ref}`sec-mainsystemextensions-solutionviewer`
- `interactive.AnimateModes`, lets you view the animation of computed modes, see {ref}`sec-interactive-animatemodes`

shown exemplary in {ref}`fig-solutionviewer`.

(fig-solutionviewer)=
```{figure} /docs/theDoc/figures/solutionViewer.png
:width: 800

View of `SolutionViewer` (as of Exudyn 1.5.42.dev1)
```

The `SolutionViewer` adds a `tkinter` interactive dialog, which lets you interact with the model, with the following features:

- The SolutionViewer represents a 'Player' for the dynamic solution or a series of static solutions, which is available after simulation if `solutionSettings.writeSolutionToFile = True`
- The parameter `solutionSettings.solutionWritePeriod` represents the time period used to store solutions during dynamic computations.
- As soon as 'Run' is pressed, the player runs (and it may be started automatically as well)
- In the 'Static' mode, drag the slider 'Solution steps' to view the solution steps
- In the 'Continuous run' mode, the player runs in an infinite loop
- In the 'One cycle' mode, the player runs from the current position to the end; this is perfectly suited to record series of images for **creating animations**, see {ref}`sec-overview-basics-animations` and works together with the visualization settings dialog.
- In the 'Record animation' mode, the player records frames that are shown in the render window; before pressing on 'Record animation', press 'Stop' and switch to 'One cycle'. Then put the solution steps slider to the first frame and press 'Record animation', which stores images in the current subfolder 'images' as 'frame00001.png' with increasing number, using PNG by default. The number is increased and can only be reset after new start of SolutionViewer.
- Since Exudyn V1.9.83, the button 'Make mp4' allows to directly generate animation files, see next section.

The solution should be loaded with
`LoadSolutionFile('coordinatesSolution.txt')`, where 'coordinatesSolution.txt' represents the stored solution file,
see

- `exu.SimulationSettings().solutionSettings.coordinatesSolutionFileName`

You can call the `SolutionViewer` either in the model, or at the command line / IPython to load a previous solution (belonging to the same mbs underlying the solution!):

```python
  from exudyn.utilities import LoadSolutionFile
  sol = LoadSolutionFile('coordinatesSolution.txt')
  mbs.SolutionViewer(solution=sol)
```

**By default and as a recommended way**, if no solution is provided, `SolutionViewer` tries to reload the solution of the previous simulation that is referred to from `mbs.sys['simulationSettings']`:

```python
  #... mbs has been previously solved
  mbs.SolutionViewer()
```

An example for the `SolutionViewer` is integrated into the `Examples/` directory, see `solutionViewerTest.py`. \

(sec-overview-basics-animations)=
### Storing images and generating animations

In many dynamics simulations, it is very helpful to create animations in order to better understand the motion of bodies. Specifically, the animation can be used to visualize the model much slower or faster than the model is computed.

Images can be stored conveniently either in the way shown below for series of images, or using the SolutionViewer, {ref}`sec-overview-basics-solutionviewer`.
For single images, you can use

- `SC.renderer.RedrawAndGetImage()`

to obtain single images at dedicated time instants.
If the renderer is active, you directly get a snapshot of the current view.

#### Software rendering

Setting the flag `useRaytracer=True` in `RedrawAndGetImage`, the software raytracer will be used -- see {ref}`sec-overview-basics-raytracing` for more details.
If the renderer has not yet been started, you ONLY can use the raytracer for image retrieval, however, you should use `renderer.ZoomAll` and `renderer.SetModelView` to adjust the view previously.
However, the pure raytracer capability allows to retrieve images without opening the render window, which may be annoying in automated image retrieval or on HPC environments where openGL may not be available.

Retrieved images can be conveniently used with `matplotlib` for further manipulation or storing, also see examples:

```python
  import matplotlib.pyplot as plt

  #zoom all or set model view first!
  #...

  image=SC.renderer.RedrawAndGetImage()
  plt.imsave("testImage.jpg", image)
  plt.imshow(image)
  plt.axis('off')
  plt.show()
```

#### Generating Animations

Animations are created based on a series of images (frames, snapshots) taken during simulation. It is important, that the current view is used to record these images -- this means that the view should not be changed during the recording of images.
The easiest way to create animations, is using the SolutionViewer with its integrated features, see {ref}`sec-overview-basics-solutionviewer`.

To turn on recording of images during solving, set the following flag to a positive value

- `simulationSettings.solutionSettings.recordImagesInterval = 0.01`

which means, that after every 0.01 seconds of simulation time, an image of the current view is taken and stored in the directory and filename (without filename ending) specified by

- `SC.visualizationSettings.exportImages.saveImageFileName = "myFolder/frame"`

By default, a consecutive numbering is generated for the image, e.g., 'frame0000.png, frame0001.png,...'. Note that the standard file format PNG with ending '.png' uses compression libraries included in glfw, while the alternative TGA format produces '.tga' files which contain raw image data and therefore can become very large.

To create animation files, an external tool FFMPEG is used to efficiently convert a series of images into an animation. Since Exudyn V1.9.83, ffmpeg is integrated into the solution viewer (button 'Make mp4'), which requires prior installation using `pip install ffmpeg-python` .
Note that you may also need to install ffmpeg itself, depending on your platform.
$\ra$ see theDoc.pdf !

(sec-overview-basics-examplestestsuite)=
### Examples, test models and test suite

The main collection of examples and models is available under

- `python/Examples`
- `python/TestModels`

You can use these examples to build up your own realistic models of multibody systems.
Very often, these models show the way which already works. Alternative ways may exist, but
sometimes there are limitations in the underlying C++ code, such that they won't work as you expect.

We would like to note that, even that some examples and test models contain comparison to
papers of the literature or analytical solutions, there are many models which may not contain real
mechanical values and these models may not be converged in space or time
(in order to keep running our test suite in less than a minute).

Finally, note that the `python/TestModels` are often only intended to preserve functionality
in the Python and C++ code (e.g., if global methods are changed), but they should not be misinterpreted as validation of the
implemented methods. The `TestModels` are used in the Exudyn **TestSuite** `testing/runTestSuite.py`
which is run after a full build of Python versions. Output for very version is written
to `python/logs/testmodels` containing the Exudyn version and Python version. At the end of these
files, a summary is included to show if all models completed successfully (which means that a certain error level is achieved, which is rather small and different for the models).
There are also performance tests (e.g., if a certain implementation leads to a significant drop of performance).
However, the output of the performance tests is not stored on github.

We are trying hard to achieve error-free algorithms of physically correct models, but there may always be some errors in the code.

(sec-overview-basics-errors)=
### Errors: what Exudyn raises, and what to do about it

Every error that Exudyn reports from its C++ core arrives in Python as an exception with a
**type**. The type answers the first question a user has -- **whose mistake was it** --
before the message is even read.

All of them derive from `exudyn.ExudynError`, and each of them **also** derives from
the built-in exception that fits, so an `except ValueError` written before Exudyn 2.0
keeps working:

- `exudyn.ExudynTypeError` (also a `TypeError`): the object cannot be that parameter at all -- a list where a number belongs, a string where a function belongs.
- `exudyn.ExudynValueError` (also a `ValueError`): the kind of value is right and the value is not -- a vector of the wrong length, an unknown parameter name, `numberOfSteps=-1`, an output variable this item does not have.
- `exudyn.ExudynIndexError` (also an `IndexError`): an index outside its range -- `mbs.GetObject(99)` in a system with three objects.
- `exudyn.ModelError` (also a `ValueError`): the model **as built** does not hold together -- a load on a marker that does not exist, a node used by an object but never added, a function called before `mbs.Assemble()`. Most of these are raised by `Assemble()`, which checks the whole system.
- `exudyn.SolverError` (also a `RuntimeError`): the solver cannot continue -- a singular system matrix, no convergence, divergence. See {ref}`sec-overview-basics-convergenceproblems` for what to do.
- `exudyn.NotImplementedFeatureError` (also a `NotImplementedError`): the feature or the combination does not exist -- a jacobian that is not implemented for this element, a solver that cannot handle this constraint. **Neither your mistake nor a bug**; the message usually names a way around it.
- `exudyn.ExudynArithmeticError` (also an `ArithmeticError`): division by zero, root of a negative number.
- `exudyn.InternalError` (also a `RuntimeError`): an Exudyn invariant broke. **This one is a bug in Exudyn, not in your model** -- please report it with the message and, if possible, a model that shows it.

 So `except exudyn.ExudynError` catches everything Exudyn raises, while
`except IndexError` or `except ValueError` still do what they always did.

(sec-overview-basics-errors-solver)=
#### Catching a solver failure

The case with a concrete action behind it: a solve that fails is a normal event in a parameter
study, and it should not end the study.

```python
  try:
      mbs.SolveDynamic(simulationSettings)
  except exudyn.SolverError:
      #the solver stopped: retry with smaller steps
      simulationSettings.timeIntegration.numberOfSteps *= 10
      mbs.SolveDynamic(simulationSettings)
  except exudyn.ModelError as e:
      #the model itself is wrong - retrying will not help
      print('this model cannot be solved:', e)
```

 In a parameter variation, score the failed run instead of letting it stop the sweep:

```python
  def ParameterFunction(parameterDict):
      ...
      try: mbs.SolveDynamic(simulationSettings)
      except exudyn.ExudynError: return 1e10   #a very bad score, so the optimizer moves away from here
      return mbs.GetSensorValues(sensorNumber)[0]
```

 Note that `except exudyn.ExudynError` does **not** catch
`KeyboardInterrupt`: a long sweep can still be stopped with Ctrl-C.

#### An error inside your own user function

If a Python user function -- `springForceUserFunction` and its kin -- raises, Exudyn\
reports it as a `ModelError`, because a user function is part of the model. The original
exception is **not** lost: it is attached as `__cause__`, with its own traceback,
so Spyder and VS Code show the chain and jump to the line inside your function.

```python
  try:
      mbs.SolveDynamic(simulationSettings)
  except exudyn.ModelError as e:
      print(type(e.__cause__))   #<class 'ZeroDivisionError'>, raised in your user function
```

(sec-overview-basics-errors-where)=
#### Where the message is written

The exception carries the message and the location, so the **console shows it once** -- as
the Python traceback, and not a second time as a printed block. A caught exception therefore
prints nothing at all, which is what makes a parameter variation readable.

 The log files are the other way round: an error is written to **every open log
file**, whatever raised it.

- the `exudyn` output file, if one was opened with `exu.SetWriteToFile(...)`;
- the solver information file, if `solutionSettings.solverInformationFileName` is set.

 On a long unattended run those files are the only record that anything happened.

(sec-overview-basics-errors-deprecation)=
#### Deprecation warnings

A name that is on its way out raises a real Python `DeprecationWarning` -- once per place
in your code, not once per call. To find every use before a release removes the old name, run
your script with

```
  python -W error::DeprecationWarning yourModel.py
```

 which turns each of them into an error at the line that caused it.

(sec-overview-basics-errors-switches)=
#### Switches

- `exudyn.config.suppressWarnings = True`: no warnings, on the console or in a file.
- `exudyn.special.exceptions.parameterRangeChecks = False`: do not check parameter ranges; faster, and a wrong value then reaches the computation instead of being reported.
- `exudyn.special.exceptions.dictionaryVersionMismatch`, `.dictionaryNonCopyable`: turn those two specific errors off.

(sec-overview-basics-convergenceproblems)=
### Removing convergence problems and solver failures

Nonlinear formulations (such as most multibody systems, especially nonlinear finite elements) cause problems and there is no general nonlinear solver which may reliably and accurately solve such problems.
Tuning solver parameters is at hand of the user.
In general, the Newton solver tries to reduce the error by the factor given in

- `simulationSettings.staticSolver.newton.relativeTolerance` (for static solver),

which is not possible for very small (or zero) initial residuals. The absolute tolerance is helping out as a lower bound for the error, given in

- `simulationSettings.staticSolver.newton.absoluteTolerance` (for static solver),

which is by default rather low (1e-10) -- in order to achieve accurate results for small systems or small motion (in mm or $\mu$m regime). Increasing this value helps to solve such problems. Nevertheless, you should usually set tolerances as low as possible because otherwise, your solution may become inaccurate.

 The following hints / rules for described problems shall be followed.

- **static solver**: **load steps get very small** even if the solution seems to be smooth (or linear) and less steps are expected:
  - this may happen for **system without loads**; larger number of steps may happen for finer discretization;
  - you may adjust (increase) `.newton.relativeTolerance` / `.newton.absoluteTolerance` in static solver or in time integration to resolve such problems, but check if solution achieves according accuracy

- **static solver**:  load steps are reduced significantly for **highly nonlinear problems**:
  - solver repeatedly writes that steps are reduced $\ra$ try to use `loadStepGeometric` and use a large `loadStepGeometricRange`: this allows to start with very small loads in which the system is nearly linear (e.g. for thin strings or belts under gravity).

- **static solver**: system is (nearly) **kinematic**:
  - a static solution can be achieved using `stabilizerODE2term`, which adds mass-proportional stiffness terms during load steps $< 1$; see also hints for singular Jacobians below

- very small loads or even **zero loads** do not converge: `SolveDynamic` or `SolveStatic` **terminated due to errors**
  - the reason is the nonlinearity of formulations (nonlinear kinematics, nonlinear beam, etc.) and round off errors, which restrict Newton to achieve desired tolerances
  - adjust (increase) `.newton.relativeTolerance` / `.newton.absoluteTolerance` in static solver or in time integration
  - in many cases, especially for static problems, the `.newton.newtonResidualMode = 1` evaluates the increments; the nonlinear problems is assumed to be converged, if increments are within given absolute/relative tolerances; this also works usually better for kinematic solutions

- for **discontinuous problems**:
  - try to adjust solver parameters; especially the `discontinuous.iterationTolerance` and `discontinuous.maxIterations`; try to make smaller load or time steps in order to resolve switching points of contact or friction; generalized alpha solvers may cause troubles when reducing step sizes $\ra$ use TrapezoidalIndex2 solver
  - in case of **user functions**, make sure that there is no switching inside the user function (if or `sign` function); switching must be done in the PostNewtonStep, otherwise convergence severely suffers

- **singular Jacobians** or **redundant constraints**:
  - in case of systems that lead to a singular Jacobian due to redundant constraints or kinematic DOF in static solutions, you may switch to Eigen's FullPivotLU solver using:
  - `simulationSettings.linearSolverType = exu.LinearSolverType.EigenDense` and
  - `simulationSettings.linearSolverSettings.ignoreSingularJacobian=True` ;
  - however, check your results as they may be erroneous, because the solver tries to find an optimal solution / compromise which may not be what you intend to get!

- if you see further problems, please post them (including relevant example) at the Exudyn github page!

(sec-overview-basics-speedup)=
### Performance and ways to speed up computations

Multibody dynamics simulation should be accurate and reliable on the one hand side. Most solver settings are such that they lead to comparatively reliable results.
However, in some cases there is a significant possibility for speeding up computations, which are described in the following list. Not all recommendations may apply to your models.

The following examples refer to `simulationSettings = exu.SimulationSettings()`.
In general, to see where CPU time is lost, use the option turn on `simulationSettings.displayComputationTime = True` to see which parts of the solver need most of the time (deactivated in exudynFast versions!).
In addition to Exudyn's internal time measurements, in Spyder (or IPython) you can use magic commands such as `%timeit -n10 mbs.SolveDynamic()` to evaluate the time spent for a specific command with number of repetitions given after `-n`. This may be particularly interesting in Python user functions to see where time is lost.

To activate the Exudyn C++ versions without range checks, which may be approx. 30 percent faster in some situations, use the following code snippet before first import of `exudyn`:

```python
  import sys
  sys.exudynFast = True #this variable is used to signal to load the fast exudyn module
  import exudyn as exu
```

The faster versions are available for all release versions, but only for some `.dev1` development versions (Python 3.10), which can be determined by trying `import exudyn.exudynCPPfast`.

 However, there are many **ways to speed up Exudyn in general**:

- for models with more than 50 coordinates, switching to sparse solvers might greatly improve speed: `simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse`
- when preferring dense direct solvers, switching to Eigen's PartialPivLU solver might greatly improve speed: `simulationSettings.linearSolverType = exu.LinearSolverType.EigenDense`; however, the flag `simulationSettings.linearSolverSettings.ignoreSingularJacobian=True` will switch to the much slower (but more robust) Eigen's FullPivLU
- try to avoid Python functions or try to speed up Python functions; if this is not possible, see solutions below
- instead of user functions in objects or loads (computed in every iteration), some problems would also work if these parameters are only updated in `mbs.SetPreStepUserFunction(...)`
- Python user functions can be speed up (since Exudyn V1.7.40) by converting conventional Python functions into Exudyn (internal) symbolic user functions, which have similar performance as C++ functions with the ability to parallelize; see {ref}`sec-cinterface-symbolic`
- Alternatively, Python user functions can be speed up using the Python numba package, using `@jit` in front of functions (for more options, see [https://numba.pydata.org/numba-doc/dev/user/index.html](https://numba.pydata.org/numba-doc/dev/user/index.html)); Example given in `Examples/springDamperUserFunctionNumbaJIT.py` showing speedups of factor 4; more complicated Python functions may see speedups of 10 - 50
- for **discontinuous problems**, try to adjust solver parameters; especially the discontinuous.iterationTolerance which may be too tight and cause many iterations; iterations may be limited by discontinuous.maxIterations, which at larger values solely multiplies the computation time with a factor if all iterations are performed
- For multiple computations / multiple runs of Exudyn (parameter variation, optimization, compute sensitivities), you can use the processing sub module of Exudyn to parallelize computations and achieve speedups proporional to the number of cores/threads of your computer; specifically using the `multiThreading` option or even using a cluster (using `dispy`, see `ParameterVariation(...)` function)
- In case of multiprocessing and cluster computing, you may see a very high CPU usage of "Antimalware Service Executable", which is the Microsoft Defender Antivirus; you can turn off such problems by excluding `python.exe` from the defender (on your own risk!) in your settings:\ Settings $\ra$ Update & Security $\ra$ Windows Security $\ra$ Virus & threat protection settings $\ra$ Manage settings $\ra$ Exclusions $\ra$ Add or remove exclusions

**Possible speed ups for dynamic simulations**:

- for implicit integration, turn on **modified Newton**, which updates jacobians only if needed: `simulationSettings.timeIntegration.newton.useModifiedNewton = True`
- use **multi-threading**: `simulationSettings.parallel.numberOfThreads = ...`, depending on the number of cores (larger values usually do not help); improves greatly for contact problems, but also for some objects computed in parallel; will improve significantly in future
- decrease number of steps (`simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)`) by increasing the step size $h$ if not needed for accuracy reasons; not that in general, the solver will reduce steps in case of divergence, but not for accuracy reasons, which may still lead to divergence if step sizes are too large
- switch off measuring computation time, if not needed: `simulationSettings.displayComputationTime = False`
- try to switch to **explicit solvers**, if problem has no constraints and if problem is not stiff
- try to have **constant mass matrices** (see according objects, which have constant mass matrices; e.g. rigid bodies using RotationVector Lie group node have constant mass matrix)
- for explicit integration, set `computeEndOfStepAccelerations = False`, if you do not need accurate evaluation of accelerations at end of time step (will then be taken from beginning)
- for explicit integration, set `explicitIntegration.computeMassMatrixInversePerBody=True`, which avoids factorization and back substitution, which may speed up computations with many bodies / particles
- if you are sure that your mass matrix is constant, set:
- `simulationSettings.timeIntegration.reuseConstantMassMatrix = True`; check results!
- check that `simulationSettings.timeIntegration.simulateInRealtime = False`; if set True, it breaks down simulation to real time
- do not record images, if not needed: `simulationSettings.solutionSettings.recordImagesInterval = -1`
- in case of bad convergence, decreasing the step size might also help; check also other flags for adaptive step size and for Newton
- use `simulationSettings.timeIntegration.verboseMode = 1`; larger values create lots of output which drastically slows down
- use `simulationSettings.timeIntegration.verboseModeFile = 0`, otherwise output written to file
- adjust `simulationSettings.solutionSettings.sensorsWritePeriod` to avoid time spent on writing sensor files
- use `simulationSettings.timeIntegration.writeSolutionToFile = False`, otherwise much output may be written to file;
- if solution file is needed, adjust `simulationSettings.solutionSettings.solutionWritePeriod` to larger values and also adjust `simulationSettings.solutionSettings.outputPrecision`, e.g., to 6, in order to avoid larger files; also adjust `simulationSettings.solutionSettings.exportVelocities = False` and `simulationSettings.solutionSettings.exportAccelerations = False` to avoid large output files

(sec-overview-advanced)=
## Advanced topics

This section covers some advanced topics, which may be only relevant for a smaller group of people.
Functionality may be extended but also removed in future

(sec-overview-advanced-camerafollowing)=
### Camera following objects and interacting with model view

For some models, it may be advantageous to track the translation and/or rotation of certain bodies, e.g., for cars, (wheeled) robots or bicycles.
Since Exudyn 1.4.18 you can attach view to a marker, using the visualization setting

```python
  SC.visualizationSettings.view0.camera.trackMarker = nMarker
```

in which `nMarker` represents the desired marker number to follow.
See also related options in `SC.visualizationSettings.interactive` in {ref}`sec-vsettingsinteractive`.

The following paragraph represents a slower, slightly outdated approach, which may be interesting for advanced usage of object tracking.
To do so, the current render state (`SC.renderer.GetState()`, `SC.renderer.SetState(...)`) can be obtained and modified, in order to always follow a certain position.
As this needs to be done during redraw of every frame, it is conveniently done in a graphicsUserFunction, e.g., within the ground body. This is shown in the following example, in which `mbs.variables['nTrackNode']` is a node number to be tracked:

```python
  #mbs.variables['nTrackNode'] contains node number
  def UFgraphics(mbs, objectNum):
      n = mbs.variables['nTrackNode']
      p = mbs.GetNodeOutput(n,exu.OutputVariableType.Position,
                            configuration=exu.ConfigurationType.Visualization)
      rs=SC.renderer.GetState() #get current render state
      A = np.array(rs['modelRotation'])
      p = A.T @ p #transform point into model view coordinates
      rs['centerPoint']=[p[0],p[1],p[2]]
      SC.renderer.SetState(rs)  #modify render state
      return []

  #add object with graphics user function
  oGround2 = mbs.AddObject(ObjectGround(visualization=
                 VObjectGround(graphicsDataUserFunction=UFgraphics)))
  #.... further code for simulation here
```

NOTE that this approach is slower and it may lead to a (usually silient) crash after closing the renderer, as the renderer thread is somehow coupled to Python which is prohibited from Python side.

(sec-overview-advanced-contact)=
### Contact problems

Since Q4 2021 a contact module is available in Exudyn.
This separate module `GeneralContact` [**still under development, consider with care!**] is highly optimized and implemented with parallelization (multi-threaded) for certain types of contact elements.

.. _fig-contactexamples:
.. figure:: docs/theDoc/figures/contactTests.png
   :width: 450

```{figure} /docs/theDoc/figures/contactTests2.jpg
:width: 450

Some tests and examples using `GeneralContact`
```

 **Note**:

- `GeneralContact` is (in most cases) restricted to dynamic simulation (explicit or implicit [**still under development, consider with care!**] ) if friction is used; without friction, it also works in the static case
- in addition to `GeneralContact` there are special objects, in particular for rolling and simple 1D contacts, that are available as single objects, cf. `ObjectConnectorRollingDiscPenalty`
- `GeneralContact` is recommended to be used for large numbers of contacts, while the single objects are integrated more directly into mbs.

 Currently, `GeneralContact` includes:

- Sphere-Sphere contact (attached to any marker); may represent circle-circle contact in 2D
- Triangles mounted on rigid bodies, in contact with Spheres [only explicit]
- ANCFCable2D contacting with spheres (which then represent circles in 2D) [partially implicit, needs revision]

For details on the contact formulations, see {ref}`seccontacttheory`.

(sec-overview-advanced-openvr)=
### OpenVR

The general open source libraries from Valve, see

- https://github.com/ValveSoftware/openvr

have been linked to Exudyn. In order to get OpenVR fully integrated, you need to run `setup.py` Exudyn with the `--openvr` flag. For general installation instructions, see {ref}`sec-install-installinstructions`.

Running OpenVR either requires an according head mounted display (HMD) or a virtualization using, e.g., Riftcat 2 to use a mobile phone with an according adapter. Visualization settings are available in `interactive.openVR`, but need to be considered with care.
An example is provided in `openVRengine.py`, showing some optimal flags like locking the model rotation, zoom or translation.

Everything is experimental, but contributions are welcome!

(sec-overview-advanced-julia)=
### Interaction with Julia

The scientific community gets increasingly interested into the language Julia.
There is a very simple interoperability with julia -- at least from julia to Python -- which has been tests.
The other way -- calling Python from julia -- is also possible, but it is left to the reader.

After installing julia (tested on Windows 10 with julia 1.6.7), you need to add Python accessibility via `PyCall`
in **julia**:

```python
  using Pkg
  Pkg.add("PyCall")
```

Ideally, you have a certain Python installation where Exudyn is already installed (and for the following examples, you also need `matplotlib`). Find the according Python path in any **Python** console:

```python
  import sys
  print(sys.executable)
```

Use this path and adapt the following **julia** script ('raw' allows to use single backslash) in **julia**:

```python
  ENV["PYTHON"]=raw"C:\Users\username\.conda\envs\venvP38\python.exe"
  Pkg.build("PyCall")
```

Now we can interact with Python, using Python objects in **julia** almost natively, try:

```python
  py"""
  import exudyn
  from exudyn.demos import *

  Demo1()
  """
```

This will run the very simple Exudyn `Demo1`.
As `exudyn` is now imported into this Python session, you can access it, e.g., `py"exudyn".Help()`
will write the help message.

To show the interoperability with julia, test the following example (similar to `Demo1`) in **julia**:

```python
  py"""
  import exudyn as exu               #EXUDYN package including C++ core part
  import exudyn.itemInterface as eii #conversion of data to exudyn dictionaries

  SC = exu.SystemContainer()         #container of systems
  mbs = SC.AddSystem()               #add a new system to work with

  nMP = mbs.AddNode(eii.NodePoint2D(referenceCoordinates=[0,0]))
  mbs.AddObject(eii.ObjectMassPoint2D(physicsMass=10, nodeNumber=nMP ))
  mMP = mbs.AddMarker(eii.MarkerNodePosition(nodeNumber = nMP))
  mbs.AddLoad(eii.Force(markerNumber = mMP, loadVector=[0.001,0,0]))

  #add a sensor:
  s = mbs.AddSensor(eii.SensorNode(nodeNumber=nMP,
                    outputVariableType=exu.OutputVariableType.Position,
                    storeInternal=True))

  mbs.Assemble()                     #assemble system and solve
  simulationSettings = exu.SimulationSettings()
  simulationSettings.timeIntegration.verboseMode=1 #provide some output
  simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/demo1.txt'

  exu.SolveDynamic(mbs, simulationSettings)
  print('results can be found in local directory: solution/demo1.txt')
  """
```

We can access Python variables from julia via `py"..."` to read out, e.g., `mbs`:

```python
  py"mbs".systemData.Info()
```

We can use variables (or objects) directly in julia, e.g.,

```python
  mbs=py"mbs"
  print(mbs)
```

Finally, we can also plot values via `PlotSensor` (`matplotlib` in the background):

```python
  eplt=pyimport("exudyn.plot")
  eplt.PlotSensor(py"mbs", py"s")
```

We could also access the stored sensor data in julia, using

```python
  x = py"mbs".GetSensorStoredData(py"s")
```

and we could just print (or use) the first 10 rows of this data generated on the Python side, using it in **julia**:

```python
  x[1:10,:]
```

**NOTE** the 1-based indexing in julia, which highlights the limitations of this approach.

To finally check if the GLFW renderer also runs via julia, just use:

```python
  py"""
  from exudyn.demos import *
  Demo2()
  """
```

For the full range of possibilities, see [github.com/JuliaPy/PyCall.jl](https://github.com/JuliaPy/PyCall.jl).

(sec-overview-advanced-interactwithcodes)=
### Interaction with other codes

Interaction with other codes and computers (E.g., MATLAB or other C++ codes, or other Python versions)
is possible.
To connect to any other code, it is convenient to use a TCP/IP connection. This is enabled via
the `exudyn.utilities` functions

- `CreateTCPIPconnection`
- `TCPIPsendReceive`
- `CloseTCPIPconnection`

Basically, data can be transmitted in both directions, e.g., within a preStepUserFunction. In Examples, you can find
 TCPIPexudynMatlab.py which shows a basic example for such a connectivity.

(sec-overview-advanced-ros)=
### ROS

Basic interaction with ROS has been tested. However, make sure to use Python 3, as there is no (and will never be any) Python 2
support for Exudyn.

(sec-overview-cppcode)=
## C++ Code

This section covers some information on the C++ code. For a developer-level description of the internal structure, see `docs/dev/ARCHITECTURE.md` in the repository.

Exudyn was developed for the efficient simulation of flexible multi-body systems. Exudyn was designed for rapid implementation and testing of new formulations and algorithms in multibody systems, whereby these algorithms can be easily implemented in efficient C++ code. The code is applied to industry-related research projects and applications.

### Focus of the C++ code

The code focuses on four principles, starting with highest priority:

1. developer-friendly
2. error minimization
3. user-friendliness
4. efficiency

The focus is therefore on:

- A developer-friendly basic structure regarding the C++ class library and the possibility to add new components.
- The basic libraries are slim, but extensively tested; only the necessary components are available
- Complete unit tests are added to new program parts during development; for more complex processes, tests are available in Python
- In order to implement the sometimes difficult formulations and algorithms without errors, error avoidance is always prioritized.
- To generate efficient code, classes for parallelization (vectorization and multithreading) are provided. We live the principle that parallelization takes place on multi-core processors with a central main memory, and thus an increase in efficiency through parallelization is only possible with small systems, as long as the program runs largely in the cache of the processor cores. Vectorization is tailored to SIMD commands as they have Intel processors, but could also be extended to GPGPUs in the future.
- The user interface (Python) provides a nearly 1:1 image of the system and the processes running in it, which can be controlled with the extensive possibilities of Python.

### C++ Code structure

The following **entry points** into the C++ code can be found:

- Python -- C++: the creation of the module `exudyn` is found in:\ `main/src/Pymodules/PybindModule.cpp`\ it includes large header files, which are automatically created for binding C++ code with Python.
- The object factory for creation of items (calling `mbs.AddNode(...)` and similar): \ `main/src/Main/MainObjectFactory.h / .cpp`
- Using the VisualStudio `.sln` file and using the Debug mode allows you to smoothly walk from Python to C++ code (though that this takes some time to start up and it does not work always; and it does not work for graphics if it runs in a separate thread).

The functionality of the code is mainly based on systems (MainSystem and CSystem), items and solvers representing the multibody system or similar physical systems to be simulated. Parts of the core structure of Exudyn are:

- CSystem / MainSystem: a multibody system which consists of nodes, objects, markers, loads, etc.
- SystemContainer: holds a set of systems; connects to visualization (container)
- items: node, (computational) object, marker, load, sensor
- computational objects: efficient objects for computation = bodies, connectors, connectors, loads, nodes, ...
- visualization objects: interface between computational objects and 3D graphics
- main (manager) objects: do all tasks (e.g. interface to visualization objects, GUI, Python, ...) which are not needed during computation
- static solver, kinematic solver, time integration
- Python interface via pybind11; items are accessed with a dictionary interface; system structures and settings read/written by direct access to the structure (e.g. SimulationSettings, VisualizationSettings)
- interfaces to linear solvers; future: optimizer, eigenvalue solver, ... (mostly external or in Python)
- **autogenerated**: this folder in `main/src` contains many item definitions as well as other interface files; they are all automatically generated by some Python code and should not be changed manually as they will be overwritten.

### C++ Code: Modules

The following internal modules are used, which are represented by directories in `main/src`:

- Autogenerated: item (nodes, objects, markers and loads) classes split into main (management, Python connection), visualization and computation
- Graphics: a general data structure for 2D and 3D graphical objects and a tiny openGL visualization; linkage to GLFW
- Linalg: Linear algebra with vectors and matrices; separate classes for small vectors (SlimVector), large vectors (Vector and ResizableVector), vectors without copying data (LinkedDataVector), and vectors with constant size (ConstVector)
- Main: mainly contains SystemContainer, System and ObjectFactory
- Objects: contains the implementation part of the autogenerated items
- Pymodules: manually created libraries for linkage to Python via pybind; remaining linking to Python is located in autogenerated folder
- pythonGenerator: contains Python files for automatic generation of C++ interfaces and Python interfaces of items;
- Solver: contains all solvers for solving a CSystem
- System: contains core item files (e.g., MainNode, CNode, MainObject, CObject, ...)
- Tests: files for testing of internal linalg (vector/matrix), data structure libraries (array, etc.) and functions
- Utilities: array structures for administrative/managing tasks (indexes of objects ... bodies, forces, connectors, ...); basic classes with templates and definitions

The following main external libraries are linked to Exudyn:

- LEST: for testing of internal functions (e.g. linalg)
- GLFW: 3D graphics with openGL; cross-platform capabilities
- Eigen: linear algebra for large matrices, linear solvers, sparse matrices and link to special solvers
- pybind11: linking of C++ to Python

### Code style and conventions

This section provides general coding rules and conventions, partly applicable to the C++ and Python parts of the code. Many rules follow common conventions (e.g., google code style, but not always -- see notation):

- write simple code (no complicated structures or uncommon coding)
- write readable code (e.g., variables and functions with names that represent the content or functionality; AVOID abbreviations)
- put a header in every file, according to Doxygen format
- put a comment to every (global) function, member function, data member, template parameter
- ALWAYS USE curly brackets for single statements in 'if', 'for', etc.; example: if (i<n) \{i += 1;\}
- use Doxygen-style comments (use '//!' Qt style and '@ date' with '@' instead of '\' for commands)
- use Doxygen (with preceeding '@') 'test' for tests, 'todo' for todos and 'bug' for bugs
- USE 4-spaces-tab
- use C++11 standards when appropriate, but not exhaustively
- ONE class ONE file rule (except for some collectors of single implementation functions)
- add complete unit test to every function (every file has link to LEST library)
- avoid large classes (>30 member functions; > 15 data members)
- split up god classes (>60 member functions)
- mark changed code with your name and date
- REPLACE tabs by spaces: Extras->Options->C/C++->Tabstopps: tab stopp size = 4 (=standard) +  KEEP SPACES=YES

### Notation conventions

The following notation conventions are applied (**no exceptions!**):

- use lowerCamelCase for names of variables (including class member variables), consts, c-define variables, ...; EXCEPTION: for algorithms following formulas, e.g., $f = M*q_{tt} + K*q$, GBar, ...
- use UpperCamelCase for functions, classes, structs, ...
- Special cases for CamelCase (with some exceptions that happened in the past ...):
  - continue upper case after upper case abbreviations in case of **functions or classes**: 'ODESystem', 'Point2DClass', 'ANCFCable2D', 'ANCFALE', 'ComputeODE1Equations', ... (this is not always nice to read, but has become a standard and will be further used!)
  - for variables and class member variables continue **lower case**: 'nODE1variables', 'dim2Dspecial', 'ANCFsize'
  - abbreviations at beginning of expressions: for functions or classes use `ODEComputeCoords()`, for variables avoid 'ODE' at beginning: use 'nODE' or write 'odeCoordinates'

- '[...]Init' ... in arguments, for initialization of variables; e.g. 'valueInit' for initialization of member variable 'value'
- use American English throughout: Visualization, etc.
- AVOID consecutive capitalized words, e.g., avoid 'ODEAE'
- do not use '_' within variable or function names; exception: derivatives
- use name which exactly describes the function/variable: 'numberOfItems' instead of 'size' or 'l'
- examples for variable names: secondOrderSize, massMatrix, mThetaTheta
- examples for function/class names: `SecondOrderSize`, `EvaluateMassMatrix`, `Position(const Vector3D& localPosition)`
- use the Get/Set...() convention if data is retrieved from a class (Get) or something is set in a class (Set); Use `const T& Get()/T& Get` if direct access to variables is needed; Use Get/Set for pybind11
- example Get/Set: `Real* GetDataPointer()`, `Vector::SetAll(Real)`, `GetTransposed()`, \ `SetRotationalParameters(...)`, `SetColor(...)`, ...
- use 'Real' instead of double or float: for compatibility, also for AVX with SP/DP
- use 'Index' for array/vector size and index instead of size_t or int
- item: object, node, marker, load: anything handled within the computational/visualization systems
- Do not use numbers (3 for 3D or any other number which represents, e.g., the number of rotation parameters). Use const Index or constexpr to define constants.

### No-abbreviations-rule

The code uses a **minimum set of abbreviations**; however, the following abbreviation rules are used throughout:
In general: DO NOT ABBREVIATE function, class or variable names: GetDataPointer() instead of GetPtr(); exception: cnt, i, j, k, x or v in cases where it is really clear (short, 5-line member functions).

**Exceptions** to the NO-ABBREVIATIONS-RULE:

- {ref}`ODE <ODE>`
- {ref}`ODE2 <ODE2>`: marks parts related to second order differential equations (SOS2, EvalF2 in HOTINT)
- {ref}`ODE1 <ODE1>`: marks parts related to first order differential equations (ES, EvalF in HOTINT)
- {ref}`AE <AE>`; note: using the term 'AEcoordinates' for 'algebraicEquationsCoordinates'
- 'C[...]' ... Computational, e.g. for ComputationalNode ==> use 'CNode'
- {ref}`mbs <mbs>`
- {ref}`min <min>`, {ref}`max <max>`
- {ref}`abs <abs>`, {ref}`rel <rel>`
- {ref}`trig <trig>`
- {ref}`quad <quad>`
- {ref}`RHS <RHS>`
- {ref}`LHS <LHS>`
- {ref}`EP <EP>`
- {ref}`Rxyz <Rxyz>`
- {ref}`coeffs <coeffs>`
- {ref}`pos <pos>`
- {ref}`T66 <T66>`; based on $6\times 6$ matrix transformations
- write time derivatives with underscore: _t, _tt; example: Position_t, Position_tt, ...
- write space-wise derivatives ith underscore: _x, _xx, _y, ...
- if a scalar, write coordinate derivative with underscore: _q, _v (derivative w.r.t. velocity coordinates)
- for components, elements or entries of vectors, arrays, matrices: use 'item' throughout
- '[...]Init' ... in arguments, for initialization of variables; e.g. 'valueInit' for initialization of member variable 'value'

### Implementation of new computational items in C++

This section should sketch which changes will be needed to integrate new C++ items.
In general, it is recommended to first start with a Python implementation with user functions based on
`NodeGeneric...`, `ObjectGeneric...`, `ObjectConnectorCoordinateVector` for constraints and
any suitable connector for new nodes or objects. New sensors can be based on the `SensorUserFunction`.

If such an implementation is successful, but too slow, a C++ implementation can be considered.
In the following, two use cases are shown, which show the simplicity of the procedure:

- **Case 1**: user object (body):\ It is recommended to first search for a body with a similar behavior. Copy the definition of such an object in `definitions/itemDefsObjects.py` and edit the according lines. There is not much description of this file yet (except from the first lines of the file), as it will be transformed into another format in the future. Basically, you need to edit the interface, which contains parameters (which are linked to Python) and functions, which go to the header file. When you finished editing, run `pythonAutoGenerateObjects.py`. This generates the header file in `src/autogenerated` but also adds description to some docs files and adds the `pybind11` interface. Now copy the implementation (`.cpp`) file of the same connector from which you copied from and rename and edit all functions. For the body
  - `ComputeMassMatrix`: computes the mass matrix either in sparse or dense mode; this function is performance-critical if the mass matrix is non-constant
  - `ComputeODE2LHS`: computes the {ref}`LHS <LHS>` generalized forces of the body; this function is performance-critical
  - `GetAccessFunctionTypes`: specifies, which access functions are available in \ `GetAccessFunctionBody(...)`
  - `GetAccessFunctionBody`: needs to compute functions for 'access' to the body, in the sense that e.g. forces or torques can be applied.
  - `GetAvailableJacobians`: shall return the flags which jacobians of `ComputeODE2LHS` need to be computed and which are available as functions; binary flags added up
  - `GetOutputVariableBody`: function needs to implement the output variables, such as position, acceleration, forces, etc. as defined in `GetOutputVariableTypes()`
  - `HasConstantMassMatrix`: specifies, if mass matrix is constant
  - `GetNumberOfNodes`: number of nodes of object
  - `GetODE2Size`: total number of {ref}`ODE2 <ODE2>` coordinates
  - `GetType`: some flags for objects, such as `Body`, `SingleNoded`, `SuperElement`, ...; these flags are needed for connectivity and special treatment in the system
  - `GetPosition, GetVelocity, ...`: provide this functions as far as possible; rigid bodies need to provide positions and rotation matrix, as well as velocity and angular velocity for markers; if functions do not exist, some marker or sensor functions may fail
  - ...   possibly some helper functions, which you should implement for the functionality of your object.

- **Case 2**: user connector:\ It is recommended to search for a connector with similar behavior; first check, if you would like to implement an algebraic constraint or a spring-damper-like connector. Again, copy a similar connector in `definitions/itemDefsObjects.py` and edit the according lines. When you finished editing, run `pythonAutoGenerateObjects.py` and make a copy of the copied implementation (`.cpp`) file. The implementation file usually consists of
  - `ComputeODE2LHS`: this function shall compute the {ref}`LHS <LHS>` generalized forces on the two marker objects
  - `ComputeJacobianODE2_ODE2`: computes the `GetAvailableJacobians()` is not providing any '..._function' flag, which indicates that these jacobians are available as function
  - `GetOutputVariableConnector`: this function needs to compute all output variables as given in `GetOutputVariableTypes()`
  - ...   possibly some helper functions, which you should implement for the functionality of your object.
