(sec-revisions)=
# Revisions

What changed between one release of Exudyn and the next, in prose, for somebody who has models
written against the previous one. Every single resolved issue is in the
[changelog](../../CHANGELOG.md), and the issues themselves are in the
{ref}`issue tracker <sec-issuetracker>`; this chapter is the short version, and a new section is
added on top of it at each release.

This chapter is deliberately short. The revision behind it is recorded **step by step** in a
plan and a log, which are working documents of the project rather than user documentation:
there are two of them, *revision2026* - finished, and what version 1.12 is - and
*revision2026b*, which carries what it did not finish. Both are linked from the
[developer documentation](../dev/README.md), together with the standing information document
that holds the measured facts and the decisions.

(sec-revisions-1-12)=
## Version 1.12

A revision of the whole project rather than a feature release: the layout of the repository, the
build, the tests, the documentation and the development tools. Most of it is invisible from a
model script. The parts that are not are first.

```{note}
The individual issues of this revision carry **1.11.x** version numbers, because the micro
version counts closed issues continuously and does not restart; **1.12.0** is the point at which
the revision was declared complete. 1.12 itself was **not published** — **1.13 is the first
release that carries it**.
```

### What can break a script

**`exudyn.AccessFunctionType.SuperElementAlternativeRotationMode` is gone**: it was a mode of
`MarkerSuperElementRigid` (`useAlternativeApproach`), not an access type, and is passed as an argument
now (#2744).

**Star imports export only what a module defines.** `from exudyn.utilities import *` used to drag
in everything that module had imported itself — `np`, `sin`, `cos`, `sqrt`, `graphics` and more.
Every module now declares `__all__`, so a script that relied on those names arriving *through*
Exudyn raises `NameError`. The fix is one line at the top of the script:

```python
import numpy as np
from math import sin, cos, sqrt
import exudyn.graphics as graphics
```

**`from exudyn.utilities import *` provides less** (#2756): no longer the beam generators, the
MainSystem extensions and the old graphics helpers, which each have their own place. A script that
used them without importing them gets a `NameError`, and the fix is one line:

| name | write instead |
|---|---|
| `GenerateStraightLineANCFCable2D`, `GenerateSlidingJoint`, `GenerateAleSlidingJoint`, `GenerateStraightBeam` | `from exudyn.beams import GenerateStraightBeam` (the one used) |
| `CreateDistanceSensorGeometry`, `CreateDistanceSensor`, `DrawSystemGraph` | `mbs.CreateDistanceSensor(...)`, `mbs.DrawSystemGraph(...)` |
| `color4red`, `color4steelblue`, ... | `graphics.color.red`, `graphics.color.steelblue`, ... |
| `GraphicsDataRectangle(x0, y0, x1, y1, color)` | `graphics.Lines([[x0,y0,0], [x1,y0,0], [x1,y1,0], [x0,y1,0], [x0,y0,0]], color=color)` |
| `GraphicsDataOrthoCubeLines(x0, y0, z0, x1, y1, z1, color)` | `graphics.BrickXYZ(x0, y0, z0, x1, y1, z1, addFaces=False, addEdges=True, edgeColor=color)` |
| `RefineMesh`, `ShrinkMeshNormalToSurface`, ... - the mesh functions of `exudyn.graphicsDataUtilities` | `from exudyn.graphicsDataUtilities import RefineMesh` (the one used) |

`GraphicsDataRectangle`, `GraphicsDataOrthoCubeLines` and the `color4...` names are deprecated; the
scripts of the repository use `exudyn.graphics` for them. `exudev scripts <folder>` names every such
line of a script.

**Names that are gone.** Eleven small vector helpers were removed from `exudyn.basicUtilities`
— numpy does all of them, faster and in one call; what they did is in the
[`basicUtilities.py` of Exudyn 1.11.0](https://github.com/jgerstmayr/EXUDYN/blob/e44aca1b4fe3e5f4d820ff407fb3fd30b6581c1c/main/pythonDev/exudyn/basicUtilities.py).
They returned lists, numpy returns arrays:

| removed | use |
|---|---|
| `NormL2(v)`, `VSum(v)` | `np.linalg.norm(v)`, `np.sum(v)` |
| `VAdd(a, b)`, `VSub(a, b)`, `ScalarMult(s, v)` | `np.array(a) + b`, `np.array(a) - b`, `s*np.array(v)` |
| `VMult(a, b)` (the scalar product) | `np.dot(a, b)` |
| `Vec2Tilde(v)`, `Tilde2Vec(m)` | `Skew(v)`, `Skew2Vec(m)` of `exudyn.rigidBodyUtilities` |
| `DiagonalMatrix(n, value)`, `eye2D`, `eye3D` | `value*np.eye(n)`, `np.eye(2)`, `np.eye(3)` |

The 23 deprecated `GraphicsData...` aliases in `exudyn.utilities` are gone as well; the current names
are in `exudyn.graphics` - mostly the old name without the prefix, `GraphicsDataSphere` is
`graphics.Sphere`. The exceptions: `GraphicsDataOrthoCubePoint` is `graphics.Brick`, `GraphicsDataCube`
`graphics.Cuboid`, `GraphicsDataOrthoCube` `graphics.BrickXYZ`, `GraphicsDataLine` `graphics.Lines`,
`GraphicsDataFromSTLfileTxt` `graphics.FromSTLfileASCII`, `GraphicsData2PointsAndTrigs`
`graphics.ToPointsAndTrigs`, `ExportGraphicsData2STL` `graphics.ExportSTL` and
`MergeGraphicsDataTriangleList` `graphics.MergeTriangleLists`
([the aliases of 1.11.0](https://github.com/jgerstmayr/EXUDYN/blob/e44aca1b4fe3e5f4d820ff407fb3fd30b6581c1c/main/pythonDev/exudyn/utilities.py)).

**Functions that moved inside the package**: from `exudyn.utilities` into `basicUtilities`,
`advancedUtilities` and `mainSystemExtensions`. A script that imports from `exudyn.utilities` in
the usual way notices nothing; one that imported such a function by its full module path imports it
from `exudyn.utilities` instead.

**The files a run writes by default are in `solution/`**: the solution file is
`solution/coordinatesSolution.txt`, and the solver information and the restart file are there as
well, so that a run writes nothing beside the script. A script that reads the solution file back as
`'coordinatesSolution.txt'` reads the new name `'solution/coordinatesSolution.txt'` instead; a script
that sets its own file names keeps them. `exudev scripts <folder>` finds both, and every file a script
names without a directory.

**Item dictionaries are more forgiving, not less**: a parameter that is left out keeps its default
or its current value, where it used to raise `KeyError`.

**A settings file changes what a script does, if you make one.** `~/.exudyn/config.json` is read at
import into `exudyn.special.overrideSettings` and can override `exudyn.config`, any plain
`visualizationSettings` — in **every** such structure that is created, not only in a
`SystemContainer` — the size and position of a dialog, and the settings of the results monitor, which
has no file of its own ([](#sec-overridesettings)). Nothing writes it by itself, `import exudyn`
prints one note naming what came from it, and `EXUDYN_NO_USER_SETTINGS=1` ignores it — which is what
a bug report needs. The test suites set that variable for themselves.

**`ObjectContactConvexRoll.rBoundingSphere` is read-only.** It is computed from
`coefficientsHull`, and setting it never had an effect: the value was recomputed whenever the
parameters changed. It is no longer an argument of the item and can no longer be set, so a script
that passed it says so instead of being quietly ignored - delete the argument. Reading it — and `pContact`, the current
potential contact point — works as before, with `mbs.GetObjectParameter(objectNumber, 'pContact')`.

**OpenVR is removed.** The `--openvr` build flag, the settings under
`visualizationSettings.interactive.openVR`, the `openVR` entry of the render state and the
example `openVRengine.py` are gone. It could only be used with a head mounted display or an
emulator, it was never part of a released wheel - it had to be compiled in - and it stood in the
way of the coming rendering work. A script that only sets `openVR` settings runs once those lines are
deleted; one that needs a head mounted display stays on Exudyn 1.11.

**The text export of an image is removed.** `exportImages.saveImageFormat = 'TXT'`, the four
`exportImages.saveImageAsText...` settings and `exudyn.plot.LoadImage` are gone.
`SC.renderer.GetGraphicsData()` gives the same drawing elements, and more of them, as numpy arrays,
without a render window; `exudyn.plot.PlotImage` draws what it returns, so a vector figure of a model
is `PlotImage(SC.renderer.GetGraphicsData(), fileName='model.pdf')`.

**GraphicsData comes as rows.** The functions of `exudyn.graphics` return points, normals, colors,
triangles and edges as 2D numpy arrays - one row per point, color or triangle - and so does
`mbs.GetObject(..., addGraphicsData=True)`; they had been flat lists. Given to Exudyn, both forms are
read. A script that indexes a returned list as flat, `g['points'][3*i+1]`, reshapes it first:
`np.array(g['points']).reshape(-1,3)[i,1]` works for both. `graphics.Sphere` returns the new type
`Spheres` for a whole sphere instead of a `TriangleList`; the functions of `exudyn.graphics` that need
triangles convert it (#2709).

**Round primitives are curved.** `graphics.Cylinder`, `SolidOfRevolution` (and with it `Arrow`, `Basis`,
`Frame`, `RigidLink`, `BallBearingRings`) and `Torus` return 6-node triangles (`triangles6`) and curved
rims (`edges3`) instead of flat triangles. `nTiles` keeps its meaning, the number of flat segments
around: half as many curved elements carry them, and the renderer draws at least those segments. A
script that reads `g['triangles']` of such a shape gets the flat ones with
`graphics.Triangles6ToTriangles(g)`; `graphics.ToPointsAndTrigs` does this itself, so a contact mesh
made from a cylinder with an even `nTiles` has the same facets as before, on more triangles (#2709).

**The rotation of an ANCF slope node is one frame.** `NodePointSlope23` and the cross section of
`ObjectANCFBeam` rotate with the orthonormal frame of their slopes ($
v_z$ normalized, $
v_y$
orthogonalized against it), and their angular velocity and rotation Jacobian are now its
derivatives; they were a least-squares fit of both slopes, which differs where the cross section
deforms. Joints and torques on these nodes converge as they should; the angular velocity output of a
deformed cross section, and results with joints on slope nodes, change slightly (#2763).

**`rigidBodyUtilities.HT` is gone; its name is `exudyn.HT` now.** The shortcut `HT` of the function
`HomogeneousTransformation(A, r)`, which `from exudyn.utilities import *` brought into a script, is
removed, so that `HT` means one thing - the class `exu.HT`. A script that called `HT(A, r)` for a 4x4
array writes `HomogeneousTransformation(A, r)` (#2781).

### What is new to use

**Renamed settings keep working.** A simulation setting that is renamed answers to its old name with a
`DeprecationWarning` that names the new one, as the visualization settings do: `parallel.multithreadedLLimitLoads`,
`...Residuals`, `...Jacobians` and `...MassMatrices` are `parallel.multithreadedLowerLimitLoads` and so on, until 2031 (#2588, #2800).
The same holds for item parameters: a renamed one is still taken under its old name - in the item class, in a
dictionary and by `mbs.GetObjectParameter`/`SetObjectParameter` - with a warning naming the new one; the page of the
item lists it (#2589).

**More markers on more bodies.** `ObjectRotationalMass1D` takes forces and connectors anywhere on
its table, not only on its axis; `ObjectANCFBeam` takes `MarkerBodyRigid`, so torques and joints with
rotations act on its cross sections (#2775).

**What an item computes, from Python.** `mbs.ComputeItem(item, what)` computes, at the current state,
what the solver computes for one object, node or marker - the position and rotation Jacobians of a
body at a local position, its mass matrix and right-hand side, the forces of a connector, the
equations, constraint Jacobian and reaction forces of a joint, the kinematics of a marker -, and
`mbs.ComputeItem(item)` lists what applies (`exu.ComputeItemType`).
`exudyn.advancedUtilities.NumericalJacobian` gives the numerical derivative to compare with, for
testing a model or an item of one's own (#2779).

**`exudyn.HT`, the homogeneous transformation of the C++ core.** `exu.HT(rotation=A, translation=p)`
composes with `*` (`H1*H2`, and `H*v` for a point), inverts (`Inverse()`), and converts to and from the
4x4 matrix (`HT44()`, `exu.HT(T44)`) - faster than the 4x4 numpy arrays of
`exudyn.rigidBodyUtilities`, whose function `HomogeneousTransformation` stays (#2780).

**The output variable `HomogeneousTransformation`.** Every node, body point, marker and connector that
gives `Position` and `RotationMatrix` also gives `HomogeneousTransformation`, the 4x4 matrix [A p; 0 1]:
`GetNodeOutput`, `GetObjectOutputBody` and `GetMarkerOutput` return it as an `exu.HT`, a sensor stores
its 16 values row by row, from which `exu.HT(values)` makes the HT (#2780, #2789, #2792).

**A frame as one parameter.** `ObjectGround` takes its frame as `referencePosition` and
`referenceRotation` or at once as `referenceHT` - a 4x4 matrix, its 16 values or an `exu.HT`; a
parameter left `None` is not given, and an HT given with one of its parts must agree with it.
`CreateGround` and `CreateRigidBody` take `referenceHT`, and `CreateRigidBody` also `initialHT` for
`initialRotationMatrix` and `initialDisplacement` (#2793, #2794). The rigid markers take `localHT`, a frame
with a rotation in the body, link or node: `MarkerBodyRigid` and `MarkerKinematicTreeRigid` with `localPosition` as
its translation, `MarkerSuperElementRigid` with `offset`, `MarkerNodeRigid` a rotation only - so a joint can take its
axis from its markers instead of `rotationMarker0/1` (#2795).
`ObjectKinematicTree` takes its joint transformations and offsets as one list `jointHTs`, and numpy reads an `exu.HT`
as its 4x4 matrix (`np.array(H)`), so the HT functions of `exudyn.rigidBodyUtilities` and the robotics classes take an
`exu.HT` wherever they take a 4x4 array (#2798, #2799).

**The rotation of a joint is its markers'.** `rotationMarker0/1` of `ObjectJointGeneric`, `ObjectJointRevoluteZ`,
`ObjectJointPrismaticX`, `ObjectConnectorRigidBodySpringDamper` and `ObjectConnectorTorsionalSpringDamper` are deprecated and removed in 2031: the rotation is given to the
markers as `localHT`, e.g. `MarkerBodyRigid(bodyNumber=b, localHT=HomogeneousTransformation(A, p))`. `Assemble()`
warns once per session about an item that still has one other than the unit matrix. The `Create...` functions,
`AddRevoluteJoint`, `AddPrismaticJoint`, `GetJointArgs` and the robotics classes put the rotation into the markers
they create (#2745). `ObjectConnectorRigidBodySpringDamper` applies `rotationMarker0` after the frame of marker 0,
as the joints do and as its page says; a model with a `rotationMarker0` other than the unit matrix moves differently
than before (#2801). `ObjectContactCurveCircles` has no `rotationMarker0` any more: it was used only for drawing,
and `Assemble()` refused any value other than the unit matrix; the curve lies in the frame of marker 0 (#2803).

**A system without coordinates** - only ground, sensors and user functions - is solved by every
solver: time advances, the user functions are called and the sensors record (#2790).

**The frame of a rigid marker** is drawn with `visualizationSettings.markers.showBasis` and
`basisSize`: three lines in red, green and blue, or three arrows with short heads (#2791).

**Curved shapes in GraphicsData.** 6-node (quadratic) triangles - the key `triangles6` of a
`TriangleList` - quadratic lines (`Lines` with `shape` `'quadratic'`) and quadratic edges (`edges3`)
are drawn curved: the renderers split them when they draw, as fine as
`visualizationSettings.openGL.advanced.curvedTriangleTilingAngle` asks, and a change of it shows at
once; each edge is split by its own curvature, so a surface curved in one direction is not split along the
other. The type `Spheres` draws many spheres at once, and the raytracer intersects them exactly. The round
shapes of `exudyn.graphics` - `Cylinder`, `Sphere` (also partial and hollow), `Torus`, `SolidOfRevolution`,
`Tube`, `LinkedCylinders` and the shapes built from them - consist of 6-node triangles, as do the surfaces of
quadratic NGsolve meshes and of Tet10 meshes (`FEMinterface.VolumeToSurfaceElements`) in the FFRF objects.
`python/Examples/graphicsCurvedShapes.py` shows them all (#2709).

**A command line for the installed package**: `python -m exudyn info` prints the version, where
the package is installed, which compiled module is loaded and which optional packages are present
— the first thing to put into a bug report. `monitor`, `plot` and `demo` are the other three
commands; see {ref}`sec-commandline`.

**Errors say what kind of problem they are.** Everything Exudyn raises derives from
`exudyn.ExudynError` *and* from the matching Python built-in, so `except ValueError` still works
and `except exudyn.SolverError` is now possible — see
{ref}`Errors: what Exudyn raises <sec-overview-basics-errors>`.

**Two environment variables keep a run quiet**, which matters for parameter studies and for
anything that runs unattended: `EXUDYN_SUPPRESS_UI_WINDOW_OPEN=1` opens no renderer, viewer, plot
or dialog window, and `EXUDYN_OUTPUTDIRECTORY=<path>` puts the files a model writes where you want
them.

**The windows can remember where you put them.** The settings dialog has a **store settings** and a
**store positions** button, the render window has a size and a position among its settings
(`view0.window.renderWindowSize` and `renderWindowPosition`), and
`exudyn.plot.StorePlotWindowGeometry()` keeps the plot windows of `PlotSensor` where they are
arranged. Everything goes into `~/.exudyn/config.json`, after showing what will be written
([](#sec-overridesettings)).

**The scene as data**: `SC.renderer.GetGraphicsData()` returns every line, sphere, circle, text and
triangle the renderer would draw, each with the item that drew it, and needs no window - for a test
that checks what a model looks like, or for a figure drawn with matplotlib.

**Stopping a simulation from a script**: `SC.renderer.StopSimulation()` does what closing the
render window does - the running simulation ends after its step, a later one before its first, and
neither is reported as a solver failure - from a user function, another thread or a test;
`mbs.SetRenderEngineStopFlag(False)` lets the next one run again.

**`exudyn.types`** answers questions about items from Python: which markers an object accepts,
which item types exist.

**`mbs.Inspect(itemIndex, what)`** asks an existing item what it provides and requests - its output
variables, its type flags, the node and marker types it requests, the access functions of a body -
as lists of the exported enumerations; `what` is a member of `exu.InspectType`, or `None` for all
that apply. The test model `inspectTest.py` shows it.

**Kinetic and potential energy as output variables** (`OutputVariableType.KineticEnergy`,
`PotentialEnergy`) of the bodies, beams, plates, superelements and spring-dampers, and
`exudyn.advancedUtilities.SystemEnergy` for the energy of a whole system with its loads; an item
that cannot compute its energy - with a user function defining its force - says so, and
`mbs.Inspect` does not list it. The test models `energiesTest.py` and `energiesFlexibleBodiesTest.py`
show them.

### What is new to read

The documentation is **Markdown** and is built with Sphinx for every release; the hand-written
chapters are `docs/manual/`, and everything under `docs/generated/` — the reference manual of all
items, the Python utilities, the examples and the test models — is written by the generators from
the definitions and the docstrings.

Until this release the documentation was **one PDF, `theDoc.pdf`** — a name that still appears in
older issues, in scripts and in printed notes. It is the HTML documentation now: the same content,
searchable, and one place per subject instead of a chapter and a PDF section saying it twice.

**Installing and building are told once each.** The installation instructions come before the
first example, and building from source - which used to be described in three places that
disagreed - is one page of the developer documentation, together with a page that takes a new
developer from cloning the repository to a passing test run and one on the git workflow (#2646).

**`CHANGELOG.md`** is new: every resolved issue of the **current** release, newest first, with
a table of every release above it. The version number says where an issue landed — the micro
version *is* the count of issues closed in a release, so 1.10.160 is the 160th issue closed in
release 1.10. The earlier releases are on the issue tracker page, which carries more about each
issue than the changelog does, and the two no longer print the same list twice (#2599).

### What changed behind the scenes

The wheels for Windows, Linux and macOS are built by continuous integration on every tagged
release, and the documentation is built the same way. Everything else a maintainer does — build,
test, documentation, issues, release — runs through one driver, `exudev`, which replaced sixteen
batch files. None of that changes anything for a user of a wheel; it changes how quickly a fix
reaches one.
