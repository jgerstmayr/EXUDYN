# Coupled systems solved in parallel

A design note of revision2026c step RG5.10 (#3433), for the maintainer's evaluation. It builds on the pre-analysis
[systemContainerParallel.md](systemContainerParallel.md) (RG5.10.1, #3440), which lists the state the systems and their
solvers share; that list is not repeated here.

## 1. Goal

The typical scenario: several machines or robots in a factory, each a multibody system of its own, with a loose
interaction between them - they hand over or handle common objects, touch, or are linked by a soft element. Solved as
one system, all of it runs on one thread (or on the threads of one Newton iteration); solved as coupled subsystems, each
subsystem runs on a thread of its own and they exchange data only at synchronization points.

Requirements, from the maintainer (2026-11-11):

1. More than a parallel solve of independent systems: **coupled** systems, whose interaction is loose.
2. **One result**: one solution file, one set of sensors and one picture in the renderer and in the SolutionViewer for
   the whole model, not one per subsystem.
3. **A solver per subsystem**: each subsystem has its own solver and settings, so that implicit-explicit coupling works
   at once (a stiff machine implicitly, a light mechanism explicitly with a small step).
4. Two starting points are to be compared: several systems of one `SystemContainer` (what exists) and decoupled parts
   of one system solved in parallel (seen by the user as one system).

Independent systems are the special case without coupling; they need no separate function.

## 2. The two starting points

| | A: several systems in one SystemContainer | B: one system, partitioned |
|---|---|---|
| the user builds | `mbs0`, `mbs1`, ... with `SC.AddSystem()` | one `mbs` as today, plus a partition |
| the coupling elements | must live somewhere: in no subsystem (they span two) | the connectors whose markers lie in two parts |
| the result | one per system today; a joint view must be built | one system: solution file, sensors, viewer unchanged |
| a solver per part | natural: one settings object per system | settings per part must be given |
| what exists | systems, renderer, `hideMbs` (#3442); serial co-simulation in `coSimulationTest.py` (#1912) | nothing |

Both need the same three things: the **whole model** (to draw it, to write it and to evaluate the coupling), the
**subsystems** (to solve them) and a **map** between the coordinates of the two. A and B differ only in which of the two
the user writes and which is derived. The concept therefore has one core (Sections 3-5) and two ways in (Section 6).

## 3. The core: a super-system, subsystems and their coordinate maps

- **The super-system `S`**: an ordinary `MainSystem` that holds the whole model - all bodies, all connectors,
  including the ones between subsystems, all sensors and all graphics. `S` is never integrated by an ordinary solver; it
  receives the coordinates of the subsystems at each synchronization point. It is the system that is drawn, whose
  solution file is written and whose sensors measure. The SolutionViewer, `PlotSensor` and the renderer work on it
  unchanged.
- **The subsystems `M_1 ... M_n`**: ordinary `MainSystem`s of the same `SystemContainer`, each holding one part of the
  bodies with the connectors, loads and constraints inside that part. Each has its own `simulationSettings` and solver
  (generalized-alpha, RK44, an explicit solver with its own step size, ...), and each is solved serially
  (`numberOfThreads = 1`): the subsystems are the parallelism. They write no files and are not drawn by default
  (`viewN.scene.hideMbs`, #3442).
- **The maps `P_i`**: for each subsystem and each kind of coordinate (ODE2, ODE1, AE, Data) an index array from the
  coordinates of `M_i` into those of `S`. Gathering is `q_S[P_i] = q_Mi`, scattering is `q_Mi = q_S[P_i]`; both are
  copies of a few vectors, negligible against a step. The Data coordinates go along so that contact states and plastic
  strains appear in `S`, the AE coordinates so that the constraint forces of `S` can be written.
- **The coupling items `C`**: the items of `S` that belong to no subsystem - the connectors and contacts whose markers
  lie in two subsystems. They are the only place where the subsystems interact.

### 3.1 The macro step

With the macro step size $H$ (the synchronization period) and $t_k = kH$, one macro step is:

1. **Coupling forces**: `S` holds the state of all subsystems at $t_k$. The generalized forces of the coupling items
   alone, $\mathbf{f}_C(\mathbf{q}_S, \dot{\mathbf{q}}_S, t_k)$, are evaluated in `S` and mapped to each subsystem,
   $\mathbf{f}_i = \mathbf{f}_C[P_i]$.
2. **Subsystems**, in parallel: each `M_i` integrates from $t_k$ to $t_{k+1}$ with its own solver and step size (the
   steps end exactly at $t_{k+1}$), with $\mathbf{f}_i$ as an additional generalized force held constant over the
   macro step.
3. **Gather**: the coordinates of all `M_i` at $t_{k+1}$ are copied into `S`.
4. **Output of `S`**: solution file, sensors, the renderer update and the user functions of the step - what
   `FinishStep` does for every solver today.

This is the explicit (Jacobi) co-simulation with a zero-order hold of the coupling forces: all subsystems run at the
same time, and the coupling is first order in $H$ - the order measured in `coSimulationTest.py`. It is stable as long
as $H$ resolves the coupling: for a coupling stiffness $k_C$ between masses $m$, about $H\sqrt{k_C/m} < 1$. Loose
interaction fulfills that with a macro step much larger than the steps inside the subsystems.

Improvements that need no other structure: the coupling forces extrapolated linearly from the last two macro steps
(second order), and, if needed later, the macro step repeated until the coupling forces converge (each `M_i` saved and
restored with `GetSystemState`/`SetSystemState`; this costs the repetitions).

### 3.2 `S` is solved by a coordinating solver

The macro step is a step of a solver of `S`: a new solver class next to the explicit and implicit ones, derived from
`CSolverBase`, whose step is items 1-3 above and which keeps everything of `CSolverBase` around the step - the
solution file, the sensors, the renderer, `stopSimulation`, the step and time user functions, the solver file. The user
calls it as any solver of `S`; the step size of `S` is $H$.

What this gives without further work:

- one solution file of the whole model, read by the SolutionViewer as it is today;
- sensors of `S` on any item, including the coupling connectors (the force between two robots and an object);
- the renderer shows `S`, with the hideMbs of the subsystems; the existing per-system render lock suffices, since the
  subsystems are not drawn;
- a failure in one subsystem is a failure of the coupled solve: the macro step reports it with the number of the
  subsystem and the solve stops for all. Independent subsystems behave the same way.

### 3.3 What the subsystems need

- **An input for the coupling forces**: one generalized force vector per subsystem, added to its right-hand side. One
  `LoadCoordinate` per coupled coordinate exists but would be many items; a vector of the system (set by the
  coordinating solver, zero otherwise) is a small addition to the right-hand side.
- **The evaluation of a subset of items**: $\mathbf{f}_C$ is the right-hand side of the coupling items alone. The
  right-hand side of `S` (`ComputeODE2RHS`) contains every item; a variant over a list of objects is new. The
  coupling connectors act on the coordinates of their markers only, so the result is nonzero only on the coupled
  bodies.
- **A solve over an interval**: exists. `SolveSteps` with `it.startTime`/`it.endTime` set per macro step is what
  `coSimulationTest.py` uses with `MainSolverExplicit`. For the generalized-alpha solver, whether its accelerations
  carry over correctly between such calls is to be checked in the prototype (`resetAccelerations` concerns the
  initialization).

### 3.4 Common objects

An object handled by two robots belongs to exactly one subsystem. While it touches or is held by the other robot
through a contact or a spring, that element is a coupling item and the scheme above applies. A **rigid constraint
across two subsystems** cannot be handled by an explicit force coupling: a constraint has no force law, its force
follows from both sides at once. For such a grasp there are two options, both later steps: the object moves into the
subsystem of the robot that grasps it (a repartition at a synchronization point, which needs the maps to change), or
the grasp is modelled as a stiff spring-damper with a correspondingly small $H$. The first version refuses a constraint
between subsystems with a message.

## 4. Threads

Each subsystem's steps run on a worker of the persistent pool (#3437), with `numberOfThreads = 1` inside; the coordinating
solver waits for all and then does items 1, 3 and 4 on its own thread. The pre-analysis lists what must change before
two solvers can run at the same time: the GIL released during the steps, the thread count and thread id per solver
thread instead of global, a per-solver error state, a line buffer per thread for `pout`, the `pySpecial` counters
atomic, the global timers off, the static matrix of `MatrixBase::Invert`. These are needed by every variant of RG5.10
and can be done first.

Python user functions take the GIL and serialize the subsystems; symbolic user functions do not. A model meant for
parallel subsystems uses symbolic user functions or none - the same advice as for the threads of one solve.

More subsystems than threads are fine: the pool takes them in turn. Unequal subsystems limit the speed-up to the
slowest one per macro step; a subsystem per robot is a natural balance.

## 5. Accuracy and limits

- The coupling error is first order in $H$ (second order with extrapolation). With $H \to$ the step size of the
  subsystems and one subsystem, the result tends to the monolithic solve; the test compares the two, as
  `coSimulationTest.py` does.
- Not covered: constraints across subsystems (Section 3.4), stiff coupling (an implicit coupling with interface
  Jacobians would be a step of its own), a step size control of the macro step.
- Memory: the items exist twice, in `S` and in one `M_i`. For machines of some thousand coordinates this does not
  matter.

## 6. Two ways in

**B, recommended first: one system, partitioned.** The user builds `S` exactly as today and adds the partition and the
settings of the parts:

```python
parts = [[robot0Bodies], [robot1Bodies], [conveyorBodies, objectBody]] #object numbers, or found automatically
solver = exu.CoupledSolver(mbs, parts, [settingsRobot0, settingsRobot1, settingsConveyor], macroStepSize=1e-3)
mbs.SolveDynamic(simulationSettings, solver=solver)  #or a function of exudyn.solver; the name is open
```

The subsystems are derived: each part's bodies with their nodes, and every marker, load, connector, constraint and
sensor whose objects all lie in that part; a connector between two parts is a coupling item; a body in no part is a
part of its own. The partition may also be found automatically as the connected components of the graph of bodies and
connectors, with the connectors the user marks as coupling cut. For the user, the model stays one system: the files,
the viewer and the sensors are those of today.

**A: several systems, joined.** A user who already has `mbs0`, `mbs1`, ... in one `SystemContainer` gets `S` by
joining them, plus the coupling items added to `S` afterwards. Same core, the other direction of the copy.

Both directions need **one helper**: copy an item from one system to another with its node, marker, object and load
numbers renumbered. The fields that hold such numbers are known from the definitions (their types `NodeIndex`,
`MarkerIndex`, `ObjectIndex`, arrays of them), so the renumbering can be generated rather than written per item. To
check: whether every item dictionary (`mbs.GetObject(i)`) can be added again as it is - user functions, symbolic user
functions, graphics data, sensor file names.

## 7. Steps proposed (sub-steps of RG5.10)

1. **RG5.10.2, this design**, evaluated by the maintainer.
2. **RG5.10.3, the thread prerequisites** of Section 4 (the GIL, thread-local thread count and id, per-solver error state
   and line buffer, counters, timers), tested with independent systems solved from two Python threads. Needed by every
   variant; can start before the decisions below.
3. **RG5.10.4, a serial prototype in Python** (way B): the partition, the derived subsystems, the maps, the macro step of
   Section 3.1 with `SolveSteps` per subsystem, the coupling forces from the right-hand side of `S` evaluated with and
   without the coupling connectors (their difference, until the subset evaluation exists), `S` written through a
   solver call per macro step. Measures accuracy against the monolithic solve on two robots handing over an object.
4. **RG5.10.5, the coordinating solver in C++**: the class of Section 3.2, the coupling force vector of the subsystems,
   the evaluation of a subset of items, the subsystems on the persistent pool. Measures the speed-up with 2, 4 and 8
   machines.
5. **RG5.10.6, way A, the automatic partition, the documentation** and an example in `python/Examples`.

## 8. Decisions for the maintainer

1. One core with the super-system `S` (Sections 3-5), entered from one partitioned system (B) first and from several
   systems (A) afterwards - or A first?
2. The explicit Jacobi coupling with a zero-order hold first, extrapolation as an option, the iterated macro step
   later - agreed?
3. A constraint between subsystems is refused in the first version (Section 3.4) - agreed?
4. The name of the user function: a solver object (`exu.CoupledSolver`) or a function in `exudyn.solver` (for example
   `SolveCoupled(mbs, parts, settingsList, macroStepSize)`)?
