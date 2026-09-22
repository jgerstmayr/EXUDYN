# Exudyn C++ architecture

The overall idea of the C++ side: what the pieces are, how a Python call reaches computation, and
where to look when changing something. **Not an API reference** — the per-item reference is
generated into `docs/generated/items/` from `definitions/`, and is far better at that job.

Related, deliberately not repeated here: repository layout in [README.md](README.md), build and
commit workflow in [WORKFLOW.md](WORKFLOW.md), naming and file headers in
[CODING_STYLE.md](CODING_STYLE.md).

## Why it looks like this

*(From the user manual, where this stood until the 2026 revision; revision2026b step RG3.1.)*

The C++ side follows four principles, in this order of priority:

1. **developer-friendly**  — first, because a formulation that cannot be implemented without a
   fight does not get implemented correctly;
2. **error minimization**;
3. **user-friendliness**;
4. **efficiency**.

What follows from that order: the basic libraries are slim and extensively tested rather than
general; new program parts get unit tests while they are written (LEST for C++, the test models
for everything above); parallelization classes exist, but the assumption behind them is a
multi-core processor with one main memory, so the gain is in what stays inside the caches, and
vectorization is written for SIMD as Intel processors have it; and the Python interface is a
nearly 1:1 image of the system and of what happens in it, so that anything Python can do can be
done to a model.

## The shape of the thing

A C++ computational core exposed to Python through pybind11. The boundary is deliberate and hard:
Python builds a **dictionary**, the C++ side validates it and constructs an item from a **type
string**. There is no per-class binding for items, which is why a new item needs no new pybind11
code — and why compiled user plugins are possible at all (revision2026 phase R9 of the revision plan).

Stated design priorities, in order: **developer-friendly, error minimization, user-friendliness,
efficiency**. Efficiency last is not an accident — see `introduction.tex`, "Focus of the C++ code".

## Items: the central abstraction

An **item** is one of five kinds: **node, object, marker, load, sensor**.

- **Nodes** carry coordinates — the degrees of freedom. They have no mass and no stiffness.
- **Objects** are everything with behaviour: *bodies* (ground = no node, simple body = one node,
  FE/FFRF = many nodes) and *connectors* (constraints, joints, spring-dampers).
- **Markers** are the glue, and the reason the design scales: a load or constraint attaches to a
  marker, so it **does not need to know** whether it acts on a node or on a local position of a
  body. `Load → Marker → Node` and `Body1 → Marker1 → Joint → Marker2 → Body2`.
- **Loads** apply forces/torques through markers.
- **Sensors** observe, with only weak influence on the system.

Coordinates are **redundant** (not minimal), and constraints add Lagrange multipliers
automatically. Index conventions — 0-based in both languages, 3D by default with a `2D` suffix for
planar items — are in [CODING_STYLE.md](CODING_STYLE.md#6-item-and-dimensionality-conventions).

## The three-fold split: C / Main / Visualization

Every item kind exists three times, and this is the single most important structural fact:

```
                     ┌───────────────────────────────────────────────┐
   Python  ──dict──▶ │  Main<Item>      management, Python interface │
                     │       │          (parameter get/set, checks)  │
                     │       ├─▶ C<Item>            computation      │
                     │       │                (the hot path only)    │
                     │       └─▶ Visualization<Item>   drawing       │
                     └───────────────────────────────────────────────┘
```

Visible directly in `src/System/`:

| kind | computation | management | visualization |
|---|---|---|---|
| node | `CNode.h` | `MainNode.h` | `VisualizationNode.h` |
| object | `CObject.h` | `MainObject.h` | `VisualizationObject.h` |
| marker | `CMarker.h` | `MainMarker.h` | `VisualizationMarker.h` |
| load | `CLoad.h` | `MainLoad.h` | `VisualizationLoad.h` |
| sensor | `CSensor.h` | `MainSensor.h` | `VisualizationSensor.h` |

`CObject` has two specialisations: **`CObjectBody.h`** and **`CObjectConnector.h`**.

**Why it matters:** the `C*` half is what runs inside the time-integration loop, so it holds no
Python state and does no management work. Everything not needed during computation — Python
conversion, parameter validation, the GUI link — lives in `Main*`. Putting work on the wrong side
is the most common way to make the solver slow.

## How `mbs.AddObject(...)` actually works

Not a switch statement — a **runtime registry of lambdas**, and this is what makes plugins feasible:

```
mbs.AddObject({'objectType':'MassPoint', ...})
        │
        ▼
MainObjectFactory::AddMainObject(MainSystem&, const py::dict&)      src/Main/MainObjectFactory.cpp
        │   reads the 'objectType' STRING from the dict
        ▼
CreateMainObject(mainSystem, objectType)
        │   looks the string up in
        ▼
ClassFactoryItemsSystemData<MainObject>::Get()                       registry, keyed by string
        │   entries registered at start-up by generated lambdas in
        ▼
src/Autogenerated/objectFactoryAutoReg.h                        //AUTO: do not modify
        │   each lambda constructs the triple:
        └─▶  new CObjectMassPoint()          + wire CData / CSystemData
             new MainObjectMassPoint()       + SetCObject
             new VisualizationObjectMassPoint() + SetVisualizationObject
```

So adding an item type adds a **registry entry**, not a binding. Nothing in `PybindModule.cpp`
changes.

> **One caveat that matters for revision2026 phase R9:** the registry singleton is a function-local static inside
> a header-only class template (`MainObjectFactory.h`), so **every binary gets its own private
> map**. That is the single blocker for compiled user plugins, and revision2026 step R9.1 of the revision plan
> moves the storage into one exported accessor.

## Modules — `src/`

| module | what it is |
|---|---|
| `Autogenerated/` | **generated — never edit.** The per-item `C*`/`Main*`/`Visu*` classes, the pybind bindings, settings structures, and the factory registration table. 319 files. |
| `System/` | the core item base classes (the table above), plus `CSystemData`, item indices, contact |
| `Main/` | `CSystem`, `MainSystem` (`mbs`), `MainSystemContainer`, `MainObjectFactory` |
| `ImplObjects/` | the hand-written *implementation* of the generated objects — `CObjectANCFCable2D.cpp` and friends, 53 files, each with the `UpdateGraphics` of its own visualization (revision2026 step R11.4.4) |
| `ImplNodes/`, `ImplMarkers/` | the same for the nodes (16) and markers (18). Loads and sensors are one short file each and live in `System/` |
| `Solver/` | static, explicit and implicit second-order solvers |
| `Linalg/` | `SlimVector` (small, fixed), `Vector`/`ResizableVector` (large), `LinkedDataVector` (no copy), `ConstSizeVector` (constant size), matrices, `RigidBodyMath`, `Symbolic` |
| `Graphics/` | 2D/3D graphics data plus a small OpenGL renderer via GLFW |
| `Pymodules/` | the hand-written pybind11 layer; the generated half lives in `Autogenerated/` |
| `Utilities/` | arrays, `BasicDefinitions.h`, threading, automatic differentiation |
| `Tests/` | LEST-based unit tests for linalg and data structures |
| `pythonGenerator/` | leftovers of the old generators; the generators are in `tools/generators/`, their input - *the real source* for items - in `definitions/` |

Vendored third-party (in `include/`): **Eigen** (dense/sparse linear algebra), **pybind11**,
**GLFW** (windowing/OpenGL), **LEST** (unit tests). No heavy dependencies — see the invariants in
[README.md](README.md#invariants).

## Coordinates: the LTG mapping

Each item owns *local* coordinates; the solver works on *global* ones. The **local-to-global (LTG)**
map is built during `mbs.Assemble()`: node ordering fixes the global coordinate order, and each
object records which global indices its local coordinates map to (`GetObjectLTGODE2`,
`GetObjectLTGAE`). Constraints get their Lagrange multipliers allocated at the same time.

Practical consequence: **nothing about global indices is valid before `Assemble()`**, and adding an
item invalidates the mapping.

## Solvers

Structure is documented in [docs/manual/solver.md](../manual/solver.md) ("General solver
structure"), not in the introduction. The shape is a template method:

```
SolveSystem()
   ├── InitializeSolver()
   │      PreInitializeSolverSpecific → InitializeSolverOutput → InitializeSolverPreChecks
   │      → InitializeSolverData → InitializeSolverInitialConditions → PostInitializeSolverSpecific
   ├── SolveSteps()          loop over time / load steps
   │      └── Newton iteration, with a discontinuous-iteration outer loop for contact/friction
   └── FinalizeSolver()
```

`CSolverBase` holds the common machinery; `CSolverStatic`, `CSolverExplicit` and
`CSolverImplicitSecondOrder` fill in the specifics.

## Experimental features: `PyExperimental`

`src/Main/Experimental.h` defines `class PyExperimental`, exposed to Python. It is **not**
leftover debug code — it is the mechanism for shipping a method **behind a switch** while it is
still incomplete: a formulation needed for a paper, or one whose "part B" is unfinished.

Flags default to off, so a release carries the code without advertising it, and a small group can
enable it. **Keep it.** A cleanup pass that judges by name will read it as stray debugging surface,
which is exactly the wrong conclusion.

## Where to start reading

| you want to | start at |
|---|---|
| see how the module is created | `src/Pymodules/PybindModule.cpp` |
| follow `mbs.AddObject` into C++ | `src/Main/MainObjectFactory.cpp` |
| add a new item | [CODING_STYLE.md §9](CODING_STYLE.md#9-adding-a-new-item-node-object-marker-load-sensor) — edit `definitions/`, never the generated header |
| understand the time loop | `src/Solver/CSolverImplicitSecondOrder.cpp` |
| debug Python→C++ | VS2022, `Debug|x64`, breakpoint in a `ComputeODE2LHS` |

Those four overview diagrams are in the user manual, as mermaid: the module overview and the
C++ module in `docs/manual/introduction.md`, `systemData` and the interaction of items in
the same chapter (revision2026 step R7.1.9). They used to be tikz pictures with a
hand-made PNG beside them.
