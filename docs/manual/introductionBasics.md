(sec-overview-basics)=
# Exudyn basics

What every model needs: how the module is used, how the solver is told what to do, where the results
go, and how to look at the model while it runs.

- Interaction with the Exudyn module
- Simulation settings
- Generating output and results
- Seeing the model
- Examples, test models and test suite

Everything about the renderer and about drawing is in {ref}`sec-graphicsvisualization`; what to do
when a solver fails or a model is slow is in {ref}`sec-performance-errors`.

(sec-overview-basics-interactionmodule)=
## Interaction with the Exudyn module

It is important that the Exudyn module is basically a state machine, where you create items on the C++ side using the Python interface. This helps you to easily set up models using many other Python modules (numpy, sympy, matplotlib, ...) while the computation will be performed in the end on the C++ side in a very efficient manner.

**Where do objects live?**
Whenever a system container is created with `SC = exu.SystemContainer()`, the structure `SC` becomes a variable in the Python interpreter, but it is managed inside the C++ code and it can be modified via the Python interface.
Usually, the system container will hold at least one system, usually called `mbs`.
Commands such as `mbs.AddNode(...)` add objects to the system `mbs`.
The system will be prepared for simulation by `mbs.Assemble()` and can be solved (e.g., using `mbs.SolveDynamic(...)`) and evaluated hereafter using the results files.
Using `mbs.Reset()` will clear the system and allows to set up a new system. Items can be modified (`ModifyObject(...)`) after first initialization, even during simulation.

(sec-overview-basics-simulationsettings)=
## Simulation settings

The simulation settings consists of a couple of substructures, e.g., for `solution`, `staticSolver`, `timeIntegration` as well as a couple of general options -- for details see {ref}`sec-solutionsettings` and {ref}`sec-simulationsettingsmain`.

Simulation settings are needed for every solver. They contain solver-specific parameters (e.g., the way how load steps are applied), information on how solution files are written, and very specific control parameters, e.g., for the Newton solver.

There are many of them, so the fastest way to answer *what is this setting called* is to look at the
whole tree with its types and descriptions, without a model and without a renderer:

```
python -m exudyn dialogs sim
```

**CTRL-F** finds a setting by name and *copy line* gives the Python statement that sets it, ready to
paste into the script; see {ref}`sec-commandline`.

The simulation settings structure is created, and its values are modified, e.g.:

```{include} /docs/generated/notebooks/snippets/solving-settings.md
```

## Generating output and results

The solvers provide a number of options in `solution` to generate a solution file. As a default, exporting the solution of all system coordinates (on position, velocity, ... level) to the solution file is activated with a writing period of 0.01 seconds.

Typical output settings are:

```{include} /docs/generated/notebooks/snippets/solving-output.md
```

Furthermore, you can use sensors to record particular information, e.g., the displacement of a body's local
position, forces or joint data. For viewing sensor results, use the `PlotSensor` function of the
`exudyn.plot` tool, see the rigid body and joints tutorial.
Finally, the render window allows to show traces (trajectories) of position sensors, sensor vector quantities (e.g., velocity vectors),
or triads given by rotation matrices. For further information, see the `sensors.traces` structure of `VisualizationSettings`, {ref}`sec-vsettingstraces`.

### Continuing a simulation from its restart file

A long dynamic simulation can be stopped and continued, e.g. a job on a cluster with a time limit, or a run
stopped by `exudyn.special.solver.timeout`. With `solution.restart.write = True` the solver writes the restart file
`solution.restart.name` every `solution.restart.writePeriod` and at the end; with
`solution.restart.continueIfAvailable = True` the same script, started again, continues from that file if it exists:

```python
simulationSettings.solution.restart.write = True
simulationSettings.solution.restart.writePeriod = 10      #seconds of simulated time
simulationSettings.solution.restart.continueIfAvailable = True
mbs.SolveDynamic(simulationSettings)    #continues where the previous run left its restart file
```

The file holds the state - time, coordinates, velocities, accelerations, the algorithmic accelerations of the
generalized-alpha method - and what the solver carries from step to step, so that the continued run computes what an
uninterrupted one would: exactly, for the explicit solvers and for the implicit ones with
`timeIntegration.newton.useModifiedNewton = False`; with the modified Newton method the Jacobian is computed again at
the restart, and the results differ within the Newton tolerance. The solution and sensor files are shortened to what
was written up to the restart state and continued; sensors with `storeInternal` hold only the values after the
restart. The model is the script's: it is built again, with its user functions and their Python state, and a restart
file that does not fit it - other numbers of coordinates or items, another solver - raises an error. The static solver
writes no restart file.

(sec-overview-basics-seeingthemodel)=
## Seeing the model

A model is drawn by the renderer, which is started and stopped around the solver:

```{include} /docs/generated/notebooks/snippets/solving-renderer.md
```

The window, what it shows, how to save an image or an animation from it and how to
draw geometry of your own are in {ref}`sec-graphicsvisualization`.

(sec-overview-basics-examplestestsuite)=
## Examples, test models and test suite

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
