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

(sec-revisions-after-1-13)=
## After version 1.13

**Reproducible results on every platform.** The C++ code computes `sin`, `exp`, `atan2`, ... with its own portable
functions instead of those of the platform's library, which round some results differently; `exudyn.cMath` gives the
same functions to Python, and the test models use it for their initial values. The test models give the same results
on Windows and Linux to the last bit; models with contact or friction change slightly against 1.13. See
{ref}`sec-exudyn-cmath-why` (#2954, #2959, #2963).

**`exudyn.cUtilities`** computes the rotation and kinematics functions of `exudyn.rigidBodyUtilities` in C++, which
calls it: `ComputeOrthonormalBasisVectors` and `GramSchmidt` return a 3x3 array of the vectors as rows (unpacked as
before), and a wrong size or a singular input raises a `ValueError` (#2960, #2961).
`RotationVector2RotationMatrix` of a rotation vector longer than 2 pi rotates by its length; the angle was reduced by
2 pi without the vector, which gave no rotation matrix (#3073).

**`ObjectUserBody`**, a body whose mass matrix, forces, positions, rotations and Jacobians are Python user
functions; body markers, sensors, loads and connectors work on it, see {ref}`sec-item-objectuserbody` (#2986). It
allocates no memory in a time step, with Python or symbolic user functions (#3046).

**The user functions of `ObjectUserBody` and `NodeUser` get `qRef, qTotal, q_t` and `parameters`** (#3056, #3062): the
reference coordinates, the total coordinates and the velocities of the configuration asked for, and a parameter vector of the
item, which `exudyn.advancedUtilities.ParameterLayout` maps to named quantities and back; the
outputVariableUserFunction also gets `q_tt`, the graphicsDataUserFunction of ObjectUserBody the coordinates of the
visualization configuration. One recorded symbolic user function serves every body of a kind with its own parameters.
The outputVariableUserFunction of ObjectUserBody provides every output variable the body does not compute itself -
strains, curvatures, stresses, forces, torques, potential energy, accelerations, the user types (#3057).
`constantMassMatrix=True` calls its massMatrixUserFunction once and keeps the result (#3058). With a
`massIntegralUserFunction`, MarkerBodyMass and LoadMassProportional (gravity) act on it (#3063). Its
`graphicsData` is given in local coordinates and drawn with every point mapped by the positionUserFunction, so that a
deformable user body is drawn deformed (#3064).
**ObjectConnectorUser** is a connector whose Python user function gets its markers as `exudyn.ConnectorMarker`: it reads
them - also any point of a marker's body - and applies forces and torques through them, so that a contact can search the
closest point on a flexible body and act there (#2975).
The new page {ref}`sec-useritems` explains how the user items are set up (#2979).
**ObjectJointUser** constrains two rigid markers by 1 to 6 equations of their frames, written in Python, e.g. a screw
joint (#2976). **MarkerUser** is a marker on several bodies whose position and rotation are Python functions of their
coordinates; the built-in connectors, joints, loads and sensors act on it (#3065).
**`exudyn.symbolicUtilities`** has the rotation matrices, G matrices and their conversions of `exudyn.cUtilities` with
the same names and arguments and a last argument `functions`: with `exudyn.symbolic` they return symbolic matrices and
vectors for symbolic user functions, with `numpy`, `math` or `exudyn.cMath` numpy arrays (#3069).

**`NodeUser`**, a node with any number of coordinates whose position, and optionally orientation, are Python user
functions; the Jacobians are computed by central differences if they are not given; node markers, SensorNode, loads
and connectors work on it, e.g. for a node of a user element (#2974).

**User types**: `OutputVariableType`, `NodeType` and `MarkerType` have `UserType0` to `UserType3` for user items, which
`exu.special.userTypes.SetName` names, see {ref}`sec-exudyn-special-usertypes` (#2989).

**`SensorUserFunction.itemNumbers`**: the user function gets item numbers instead of sensor numbers and reads the
items through mbs, e.g. the energy of a group of bodies (#2977).

**Messages**: the exception of a failed `SolveDynamic` or `SolveStatic` carries the whole message, and nothing is
printed (#1977); an error in graphics data names the item and the entry of the list, and not-a-number is refused in
every type of graphics data (#2304, #2305). With `show.computationTime`, the CPU-time statistics list the time spent
in user functions (#2007). A line copied in the simulation or visualization dialog stays in the clipboard after
the dialog closes (#2994).

**The frame of two slopes** - of NodePointSlope12 and NodePointSlope23, ObjectANCFBeam and ObjectANCFThinPlate - is
their symmetric basis, `exudyn.rigidBodyUtilities.SymmetricBasis`: symmetric in the two slopes and independent of
their lengths, with a consistent angular velocity and rotation Jacobian; ObjectANCFThinPlate has a rotation, so
MarkerBodyRigid acts on it. Models with rigid markers on these items change slightly (#2228). ObjectANCFBeam measures its
strains in the same frame, so a torque on its node no longer shears the cross section; results change slightly
(#2992).

**`ShellMesh`** scales the slopes of each element edge so that the cubic edge follows the mapped surface - on a
cylinder about a thousand times closer than with the chord lengths (#2226).

**Consistent initial conditions**: the implicit solver checks the initial positions and velocities against the
constraints and warns, naming the item; `generalizedAlpha.projectInitialVelocities = True` projects the initial
velocities onto the constraints, see {ref}`sec-solver-initialconstraints` (#2234).

**Drift of the constraints**: with `generalizedAlpha.checkConstraintDrift`, the solver checks the drift of the
position-level constraints during the run and stops above `constraintDriftTolerance`; `SolveDynamic` switches it on
for `TrapezoidalIndex2`. `mbs.ComputeConstraintResiduals`, `ComputeItemType.ConstraintDrift` and
`AlgebraicEquations_t`, the output variable `ConstraintEquation` of every constraint and `SystemConstraintDrift` give
the drift at any time, see {ref}`sec-solver-constraintdrift` (#565).

**`exudyn.systemUtilities`**, the functions of a whole system, see {ref}`sec-systemquantities` (#2997):
`SystemEnergy` and `SystemConstraintDrift` with `Compute()`, `ComputeItems()` and `AddSensor()`,
`ComputeSystemGraph` - the graph without drawing - and `DrawSystemGraph`, `LoadPotentialEnergy`, `NumericalJacobian`
and `ComputeConstraintResiduals`; `from exudyn.utilities import *` includes it, and with it `DrawSystemGraph` again.
`SystemEnergy`, `LoadPotentialEnergy` and `NumericalJacobian` are no longer in `exudyn.advancedUtilities`;
`SystemEnergy.ComputeSystemEnergies()` is `Compute()`, `CreateLoadEnergySensor(mbs, load)` is
`SystemEnergy(mbs, items=[load]).AddSensor(components=['loads'])`. `NumericalJacobian` takes `systemCoordinates`,
`velocityLevel` and `epsilon = 1e-8` (#3004).

**Python user functions get numpy arrays**: a vector or matrix argument - `q`, `q_t` of ObjectGenericODE2 and
ObjectUserBody, the factors of SensorUserFunction, the load vector of a load, displacement, stiffness and offset of a
spring-damper - is a read-only numpy array that views the memory of Exudyn, valid during the call, instead of a list
(#3008, #3011); a function computing with single items, `u[0]`, is slower than before, one in vectorized form faster; `-k*q` and `K @ q` work without
`np.array(q)`, and a vector user function costs the same for any size. A function that writes into its arguments, adds
lists to them or keeps them for the next call has to copy them; `exudyn.special.deprecations.userFunctionListArguments
= True` gives the lists for the transition, see {ref}`sec-userfunctions` (#3008).

**Symbolic user functions** that return a vector run without memory allocation and several times faster (#3005,
#3006), and a component of a vector argument, `q[0]`, can be used in them (#3007); `postNewtonFunction`, the
`constraintUserFunction` of ObjectConnectorCoordinateVector and the `outputVariableUserFunction` of ObjectUserBody can
be symbolic now (#2985). `exudyn.symbolic.Vector` and `Matrix` have `dot()` and the operator `@` with numpy's meaning,
and a numpy array leaves `@` and `*` with a symbolic operand to it, so that one function computes with numpy arrays and
symbolic vectors (#3011).

**Symbolic user functions compute a shared value once per call** (#3053, #3054): the recording - numbers, vectors
and matrices - is evaluated as a list of operations instead of a tree; `GetFlatProgram()` shows the list,
`flatEvaluation = False` restores the tree evaluation.

**The operators of exudyn.symbolic are those of numpy** (#3015): `*` of two symbolic vectors, of two matrices and of a
matrix and a vector raises a TypeError, because numpy multiplies the components - write `a @ b` or `a.dot(b)` for the
scalar and matrix products and `a.multiply(b)` or `np.multiply(a, b)` for the product of the components;
`exudyn.special.deprecations.symbolicStarProduct = True` restores the old meaning for the transition (#3016). `==` of
two symbolic vectors raises a TypeError as well. Numpy's functions record with symbolic operands - `np.sin(x)`,
`np.sqrt(x)`, `np.arctan2(y, x)`, `np.multiply`, `np.dot`, a numpy array plus or times a symbolic vector -, a vector
divided by a scalar, `2**x`, `abs(x)`, `v[-1]`, `sum(v)` and `len(v)` work, and CreateSymbolicUserFunction records `q`
and `q_t` of ObjectGenericODE2, ObjectGenericODE1, ObjectUserBody, ObjectKinematicTree and the FFRF objects with their
size, so that they combine with constant vectors. The expression of a product prints as `@`.
While recording, a symbolic expression or named variable converted to float raises a TypeError - `math.sin(x)`,
`float(x)`, `np.array([x], dtype=float)` recorded a constant without a message; `np.sin(x)` and `esym.sin(x)` record,
and `x.Evaluate()` gives the value (#3017).
`esym.Matrix` has `A.T`, `transpose()` and rows `A[i]`, `esym.Vector` slices `v[a:b]`, a scalar or a list added, and
numpy's functions of each component, `np.sin(v)` (#3018).
`esym.cross(a, b)` is the cross product of two vectors, as `np.cross` (#3070), `esym.pi` is `math.pi` (#3071), and
`IfThenElse` takes numbers as the condition and the branches (#3072). The derivative `Diff` of `atan2` had the wrong
sign (#3076).

**Symbolic user functions with matrix results** (#3020): the mass matrices and Jacobians of ObjectGenericODE2 and
ObjectUserBody, the rotation matrix of ObjectUserBody, the mass matrices of ObjectFFRF and ObjectFFRFreducedOrder and
the `jacobianUserFunction` of ObjectConnectorCoordinateVector can be symbolic: the function returns an `esym.Matrix`,
a list of lists or a numpy array of symbolic items, and the item evaluates it without Python - an ObjectGenericODE2
with a 6 x 6 mass matrix function solves nine times faster than with the Python function. A matrix result built from
components, `[[m if i == j else 0. for j in range(n)] for i in range(n)]`, is sparse: its components that are the
constant 0 are left out when the function is recorded, and the others are added to the sparse system matrix as
triplets (#3022); a matrix computed with operations is dense. Symbolic matrix expressions allocate no memory per call
(#3021), and a numpy matrix returned by a Python user function is copied once into the item (#1864).

**Orienting imported triangles** (#2281): `graphics.OrientTriangles(graphicsData)` orients the triangles of a
TriangleList consistently - two triangles on an edge run it in opposite directions - and every closed part outward,
merging points by position, so that the triangles of an STL file, which have points of their own, are connected; it
recomputes the normals and returns a report on request; `graphics.CheckTriangleOrientation` reports without changing.
`FromSTLfile`, `FromSTLfileASCII` and `FromPointsAndTrigs` take `orientTriangles=True`, for STL files whose triangles are
not oriented consistently.

**A ruler in the render window** (#2244): `visualizationSettings.view0.window.showRuler` or the key F5 shows scales at
the bottom and the left border of an orthographic view (perspective 0), with ticks at 1, 2 or 5 times a power of 10;
in an axis-aligned view the labels are the coordinates of the view, as the mouse coordinates (F3) show them, else the
distance from the end of the scale. The raytracer draws it as well.

**Textures** (#2154, #2155, #3029 to #3036): `SC.renderer.AddTexture(name, image, interpretation, flags)` adds an
image - a numpy array of uint8 or of floats in [0, 1], grey, RGB or RGBA, e.g. from `graphics.ImageFromFile` - by its
name, shared by all SystemContainers; `exu.TextureInterpretation` says how its channels are drawn (A as opacity or as a
mask) and `exu.TextureFlags`, combined with `|`, how it is filtered, repeated and combined with the color of the
triangles. A
`TriangleList` with `'texture': name` and `'textureCoordinates'` (u, v per point) is drawn with it by OpenGL and by the
raytracer; 6-node triangles with a texture become four flat ones. `graphics.Quad` and `graphics.Cylinder` take
`texture=name`, and `graphics.ProjectTexture` gives any TriangleList texture coordinates by a planar, box, cylindrical
or spherical projection. `visualizationSettings.openGL.showTextures = False` draws the colors of the points instead.
`SC.renderer.GetGraphicsData()` returns the textured triangles under `texturedTriangles`. `texturesExample.py` shows a
wooden floor and a cylinder with a computed pattern, rendered with the raytracer.

**Bits combine with `|`**: the values of `OutputVariableType`, `NodeType`, `MarkerType`, `CrossSectionType`,
`LinearSolverType` and `TextureFlags` combine with `|` and `&` into an int, as in C++ (#3037).

**`graphics.CheckerBoard`** is one quad of two triangles with a texture of its two colors (#3036), drawn as before;
`useTexture=False` gives the triangles of the pattern, as needed for a contact mesh made from it with
`graphics.ToPointsAndTrigs`; colors that encode a material give the triangles as well.

**An icon bar in the render window** (#3026): it folds in from the right border when the mouse comes near it, with
icons for quit, zoom all, zoom to a region given by two clicks, the views, the ruler, the mouse coordinates, lines, edges, faces, transparency, the update
interval and the visibility of the items, see {ref}`sec-gui-sec-iconbar`; `visualizationSettings.view0.window.showIconBar`
switches it off, `general.iconBarScaling`, `iconBarFoldDistance` and `iconBarTimeout` adjust it. SHIFT and dragging
with the left mouse button zooms to the rectangle dragged (#3040).

**Stored configurations CFG0 to CFG3** (#3043, #3045): four sets of `visualizationSettings` and model
views in `~/.exudyn/config.json`, which the icons CFG0 to CFG3 of the icon bar switch to (CTRL: store the
current settings and view); stored from the dialog with the buttons → CFG0 ... of **store settings** and
**store model view**, or from a script with `exudyn.misc.GUI.StoreVisualizationSettings(SC, CFG=0)`,
see {ref}`sec-usersettings-storedconfigurations`. An icon of the icon bar switches the shadow (#3044).

**Zoom to an item** (#3027): CTRL+A in the render window centers the view on the item selected with the left mouse
button and zooms such that it fills the window; `SC.renderer.ZoomToItem(oBody)` does the same from Python.

**Drawing with user functions in a renderer thread**: the markers, connectors, loads and sensors on an ObjectUserBody
and every load with a user function are drawn in the main thread, together with the graphics user functions, so that
the render thread never calls Python; `loads.drawWithUserFunction` draws a Python user function with multithreaded
rendering as well (#3003).

**`SystemContainer(attachToRenderer=False)`** creates a container only to compute, which leaves the render window with
the container of the model; `InverseKinematicsNumerical` uses it (#2259).

(sec-revisions-1-13)=
(sec-revisions-1-12)=
## Version 1.13

A revision of the whole project rather than a feature release: the layout of the repository, the
build, the tests, the documentation and the development tools. Most of it is invisible from a
model script. The parts that are not are first.

```{note}
The individual issues of this revision carry **1.11.x** version numbers, because the micro
version counts closed issues continuously and does not restart; **1.12.0** is the point at which
the revision was declared complete. 1.12 itself was **not published**: **1.13.0** is the first
release after 1.11 and carries the revision together with what followed it.
```

### What can break a script

**`ObjectANCFThinPlate` computes other values.** Its curvatures use the material measure - for a strip
the curvature of `ObjectANCFCable2D` -, its integration rules changed (`useReducedOrderIntegration` 0: Gauss
5 x 5; 1 and 2: Lobatto 3 x 3 for the membrane, Gauss 2 x 2 for the bending), and a variable thickness gives
the stiffness from the local thickness. Results change slightly; the outputs change in form: `StrainLocal`
has 6 components through the thickness instead of the 3 membrane strains, `CurvatureLocal` and `TorqueLocal`
are in the order $xx,\, yy,\, xy$ with one sign - positive where the surface bends towards its normal -, and
`StressLocal` has the opposite sign of its bending part (#2857, #2864).

**`bodyOrNodeList` and `bodyList` of the Create functions are not accepted**: the items are given in
`itemNumbers`; a script that passes either gets a `TypeError` that says so, and `exudev scripts --fix` renames
them (#2863).

**Visualization parameters of items that draw nothing are gone**: `show` of `Node1D`, the generic nodes,
`ObjectGenericODE1`, the coordinate markers, `LoadCoordinate` and `SensorUserFunction`, `show` and `color` of
`ObjectConnectorCoordinateVector`, and `drawSize` of `ObjectConnectorGravity`. A script that gives them, e.g.
`visualization=VCoordinateVectorConstraint(show=False)`, leaves them out; `exudev scripts` finds them (#2843).

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

**`InitializeFromRestartFile` is gone** from `exudyn.utilities`: it raised "not fully implemented" in
every version. The restart file is still written (`simulationSettings.solution.restart`); how a
simulation continues from it is #2850.

**Item dictionaries are more forgiving, not less**: a parameter that is left out keeps its default
or its current value, where it used to raise `KeyError`.

**A settings file changes what a script does, if you make one.** `~/.exudyn/config.json` is read at
import into `exudyn.special.overrideSettings` and can override `exudyn.config`, any plain
`visualizationSettings` — in **every** such structure that is created, not only in a
`SystemContainer` — the size and position of a dialog, and the settings of the results monitor, which
has no file of its own ([](#sec-usersettings)). Nothing writes it by itself, `import exudyn`
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

**The time integration uses the modified Newton method by default.** `timeIntegration.newton.useModifiedNewton` is
`True`: the Jacobian is kept over the steps and computed again only when Newton converges badly, which is much faster
for most models. Results change within the tolerances of Newton - in most models from the eighth digit on, with
contact and friction earlier; set it to `False` for the full Newton method. `staticSolver.newton` keeps the full Newton method (#2815).

**The robotics classes compute with `exudyn.HT`.** `Robot.LinkHT`, `JointHT`, `COMHT`, `StdDH2HT`, `ModDHKK2HT`,
`InverseKinematicsNumerical.GetCurrentRobotHT` and `dictJointType2HT` return `exu.HT` instead of 4x4 numpy arrays, and
`RobotBase.HT`, `RobotTool.HT`, `RobotLink.localHT` and `preHT` are stored as `exu.HT`, given as `exu.HT` or 4x4 matrix
as before; the kinematics is about twice as fast. Scripts written for numpy arrays keep working: an `exu.HT` is indexed
as its 4x4 matrix (`H[0:3,3]`, also for writing, except the last row), `H1 @ H2` is the product and `H @ x` what numpy
computes, `copy.deepcopy` copies it, and `np.array(H)` gives the matrix. What no longer works is numpy arithmetic on it,
such as `H1 - H2`: use `np.array(H1) - np.array(H2)` (#2821).

**`rigidBodyUtilities.HT` is gone; its name is `exudyn.HT` now.** The shortcut `HT` of the function
`HomogeneousTransformation(A, r)`, which `from exudyn.utilities import *` brought into a script, is
removed, so that `HT` means one thing - the class `exu.HT`. A script that called `HT(A, r)` for a 4x4
array writes `HomogeneousTransformation(A, r)` (#2781).

### What is new to use

**Optional packages by name.** `pip install "exudyn[basic]"` installs scipy and matplotlib with Exudyn, which most
models need; `[common]` adds what frequently used features need, `[tests]`, `[all]` and `[rl]` the larger sets, see
{ref}`sec-install-extras`.

**Renamed settings keep working.** A simulation setting that is renamed answers to its old name with a
`DeprecationWarning` that names the new one, as the visualization settings do: `parallel.multithreadedLLimitLoads`,
`...Residuals`, `...Jacobians` and `...MassMatrices` are `parallel.multithreadedLowerLimitLoads` and so on, until 2031 (#2588, #2800).
The same holds for item parameters: a renamed one is still taken under its old name - in the item class, in a
dictionary and by `mbs.GetObjectParameter`/`SetObjectParameter` - with a warning naming the new one; the page of the
item lists it (#2589).

**`mbs.CreateLinearSpringDamper`** creates a linear spring-damper along an axis between two rigid bodies or rigid
markers, with the arguments of the joints - `itemNumbers`, `position`, `axis`, `useGlobalFrame` -, e.g. inside a
prismatic joint (#1953).

**A singular Jacobian with adaptive steps reduces the step.** With `adaptiveStep` (the default) or automatic step size,
the static and the implicit dynamic solvers take a singular system Jacobian as a failed step and reduce the step size,
instead of stopping at once; a singularity of the model still ends the run, at the minimum step size, with the
causing coordinate in the solver message and file (#1337). `SC.GetSystem(i)` keeps its `SystemContainer` alive, as
`SC.AddSystem()` does, so that a system returned from a function works after the container went out of scope (#2851).
`ObjectContactFrictionCircleCable2D` without a friction model - `frictionStiffness` and `frictionVelocityPenalty`
zero - has no tangential force, also when its data node starts in slip (#1290).

**A dynamic simulation continues from its restart file.** With `solution.restart.write` the solvers write the state
every `solution.restart.writePeriod` and at the end, and with `solution.restart.continueIfAvailable` the same script,
started again, continues from it - appending to its solution and sensor files -, for a job that is stopped and
resubmitted; see the user manual, *Continuing a simulation from its restart file* (#2850).

**The Create functions take bodies, nodes or markers in `itemNumbers`.** `CreateSpringDamper`,
`CreateCartesianSpringDamper`, `CreateRigidBodySpringDamper` and `CreateDistanceConstraint` take a body at its local
position, a node or a marker for each side; the joints and `CreateTorsionalSpringDamper` a body or a rigid marker;
`CreateCoordinateConstraint` a body - its coordinates counted over its nodes -, a node, or None for the ground; the
sphere contacts the sphere on a body, node or marker and the triangle or quad on a body; `CreateForce` and
`CreateTorque` one body, node or marker in `itemNumber`. `bodyNumbers` and `bodyNumber` are the deprecated names
until 2031, which `exudev scripts --fix` rewrites; the rolling discs keep `bodyNumbers`, as they take bodies only
(#2863, #2337).

**More markers on more bodies.** `ObjectRotationalMass1D` takes forces and connectors anywhere on
its table, not only on its axis; `ObjectANCFBeam` takes `MarkerBodyRigid`, so torques and joints with
rotations act on its cross sections (#2775).

**ANCF thin plates: damping, thickness and surface maps.** `ObjectANCFThinPlate` has Kelvin-Voigt damping of
the membrane and the bending stiffness (`stiffnessProportionalDamping`, `bendingStiffnessProportionalDamping`),
which does not damp a rigid motion, and takes 12 thickness values - a thickness and its two gradients per
node, which `ShellMesh` computes from a `thicknessFunction`. `ShellMesh(surfaceMap=...)` maps the mesh onto a
surface, with the maps of `exudyn.shells.SurfaceMap` - `SurfaceMap.Cylinder`, `ConeFrustum`,
`HemisphericalShell`, `ToroidalPanel`, ... - or a map of one's own; `exudyn.shells` has functions for
constraints, edge loads and hinges of plate nodes (`AddNodeConstraints`, `AddClampToGround`, `ApplyEdgeLoad`,
`AddEdgeSpringDamper`, ...). `visualizationSettings.bodies.shells` draws the contour at a thickness coordinate
(`contourZeta`), interpolated from the corners of the elements (`reducedInterpolation`), and along the normal
(`drawNormal`) (#2857, #2858, #2859, #2864).

**`GeneralContact` can keep a contact while the bodies penetrate.** With
`keepContactWhilePenetrating = True` a sphere-sphere or sphere-triangle contact acts as long as the
bodies penetrate, and its damping may pull at the end of the contact, as `ObjectContactSphereSphere` and
`ObjectContactSphereTriangle` do; by default it acts only while its force presses (#2848).

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
`exudyn.rigidBodyUtilities`, whose function `HomogeneousTransformation` stays (#2780). It converts the rotation from and to Euler
parameters, Tait-Bryan angles and the rotation vector, and the reference coordinates of the three rigid body nodes
(`GetCoordinatesEP()`, `SetCoordinatesRxyz(q)`, ...), gives the frame of one HT seen from another (`Relative`),
interpolates (`InterpolateSO3`, `InterpolateSE3`) and the angle and axis of its rotation; its Set functions return it,
so that `exu.HT().SetRotationAxis(axis, angle)` is one line (#2810). `LogSE3()` and `SetExpSE3(v)` give and take the
motion vector of a screw motion, `LogR3xSO3()` and `SetExpR3xSO3(v)` the translation and the rotation vector (#2819).
The example `homogeneousTransformationInterpolation.py` drives rigid bodies along both interpolations; `exu.HT(H)`
copies an HT (#2824).

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
markers as `localHT`, e.g. `MarkerBodyRigid(bodyNumber=b, localHT=HomogeneousTransformation(A, p))`; setting one
other than the unit matrix gives a `DeprecationWarning` at the line that adds the item. The `Create...` functions,
`AddRevoluteJoint`, `AddPrismaticJoint`, `GetJointArgs` and the robotics classes put the rotation into the markers
they create (#2745). `ObjectConnectorRigidBodySpringDamper` applies `rotationMarker0` after the frame of marker 0,
as the joints do and as its page says; a model with a `rotationMarker0` other than the unit matrix moves differently
than before (#2801). `ObjectContactCurveCircles` has no `rotationMarker0` any more: it was used only for drawing,
and `Assemble()` refused any value other than the unit matrix; the curve lies in the frame of marker 0 (#2803).

**Deprecations: once, counted, and found.** A deprecated setting, item parameter or function warns once per session
and name; `exu.special.deprecations.warnOnce = False` shows every use, to find all of them in a script. Every use is
counted in `exu.sys['deprecationUse']`, by source (`simulationSettings`, `visualizationSettings`, `items`, `functions`)
and name, so a test sees whether a model used anything deprecated (`recordUse`). `exudev scripts` also reports deprecated
and removed item parameters, as keyword of the item class and as key of an item dictionary (#2804, #2805, #2806).

**The deprecated functions of the Python library warn as well.** `AddRigidBody`, `AddRevoluteJoint`,
`AddPrismaticJoint`, `graphics.BrickXYZ`, `GenerateStraightLineANCFCable2D` and the others marked DEPRECATED in their
description, and the arguments `bodyList` of the `Create...` functions and `numberOfChildren`/`survivingIndividuals` of
`GeneticOptimization`, give a `DeprecationWarning` at the line of the script, once per session, and are counted in
`exu.sys['deprecationUse']['library']`; each says what to use instead. They, and the deprecated functions of the C++
module, are removed in 2029; `exudev scripts` reports them in a script (#2807).

**The simulation settings, reordered.** No substructure ends in `Settings` any more, everything of the solution file
is in `solution.file`, of the sensors in `solution.sensors`, of the restart file in `solution.restart`, and the solver of
`mbs.SolveDynamic` is a setting. Every old name still works until 2031, with a `DeprecationWarning` naming the new one,
and **`exudev scripts <folder> --fix`** rewrites a script to the new names (#2813):

| old | new |
|---|---|
| `solutionSettings` | `solution` |
| `solutionSettings.writeSolutionToFile`, `coordinatesSolutionFileName`, `solutionWritePeriod` | `solution.file.write`, `.name`, `.writePeriod` |
| `solutionSettings.appendToFile`, `binarySolutionFile`, `writeFileHeader`, `writeFileFooter`, `writeInitialValues` | `solution.file.append`, `.binary`, `.writeHeader`, `.writeFooter`, `.writeInitialValues` |
| `solutionSettings.solutionInformation`, `flushFilesDOF` | `solution.file.information`, `.flushAboveCoordinates` |
| `solutionSettings.exportVelocities`, `exportAccelerations`, ... | `solution.file.export.velocities`, `.accelerations`, ... |
| `solutionSettings.sensorsStoreAndWriteFiles`, `sensorsWritePeriod`, `sensorsAppendToFile`, `sensorsWriteFileHeader`, `sensorsWriteFileFooter` | `solution.sensors.active`, `.writePeriod`, `.append`, `.writeHeader`, `.writeFooter` |
| `solutionSettings.writeRestartFile`, `restartFileName`, `restartWritePeriod` | `solution.restart.write`, `.name`, `.writePeriod` |
| `solutionSettings.outputPrecision` (files) | `solution.precision` |
| `outputPrecision` (console) | `consolePrecision` |
| `linearSolverSettings`, `linearSolverType` | `linearSolver`, `linearSolver.solverType` |
| `displayComputationTime`, `displayGlobalTimers`, `displayStatistics` | `show.computationTime`, `show.globalTimers`, `show.statistics` |
| `timeIntegration.simulateInRealtime`, `realtimeFactor`, `realtimeWaitMicroseconds` | `timeIntegration.realtime.active`, `.factor`, `.waitMicroseconds` |
| `timeIntegration.explicitIntegration` | `timeIntegration.explicit` |
| `timeIntegration.explicitIntegration.dynamicSolverType` | `timeIntegration.solverType` |
| `newton.newtonResidualMode`, `newton.useNewtonSolver` | `newton.residualMode`, `newton.active` |
| `numericalDifferentiation.forODE2connectors`, `staticSolver.constrainODE1coordinates` | `forODE2Connectors`, `constrainODE1Coordinates` |

`timeIntegration.solverType` (default `GeneralizedAlpha`) is the solver `mbs.SolveDynamic` uses; a `solverType` given
to `SolveDynamic` is used for that run and the setting is set back afterwards - `SolveDynamic` no longer leaves the
explicit solver type written into the settings. An `exudyn.MainSolverExplicit` used directly takes the solver from
`timeIntegration.solverType` and refuses an implicit one with a message naming the setting.

The binary solution file (`solution.file.binary`) stores its numbers as `float` if `solution.precision` is below 8 and
as `double` otherwise (#2816).

**Item parameters without the prefix `physics`, and a few more renamed.** The parameters of the bodies and finite
elements lost their prefix - `physicsMass` is `mass`, `physicsInertia` is `inertia`, `physicsCenterOfMass` is
`centerOfMass`, `physicsLength`, `physicsMassPerLength`, `physicsAxialStiffness`, `physicsBendingDamping` and the others
of the beams and the plate likewise; `radiusSphere` is `sphereRadius` (`ContactSphereTorus`, `ContactSphereTriangle`),
the friction of `ObjectConnectorCoordinateSpringDamperExt` is `dynamicFrictionForce`, `staticFrictionOffsetForce` and
`viscousFrictionFactor` (N s/m), `factorValue1` of `ObjectConnectorCoordinate` is `factor1`, `rollingFrictionViscous`
of `ObjectConnectorRollingDiscPenalty` is `rollingViscousFriction` (s/m), and `intrinsicFormulation` and
`classicalFormulation` are `useIntrinsicFormulation` and `useClassicalFormulation`. The `Create...` functions take the
new names. Every old name still works until 2031 with a `DeprecationWarning` - in the item class, in a dictionary, in
`GetObjectParameter`/`SetObjectParameter` and as argument of a `Create...` function - and `exudev scripts --fix`
rewrites it (#2814).

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
([](#sec-usersettings)).

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
that apply. A node marker answers the node types its node must provide as requirements, each a list of
alternatives (#2817). The test model `inspectTest.py` shows it.

**Tutorials as notebooks.** `python/Notebooks/` holds the tutorials as Jupyter notebooks, stored with the outputs of
their last run; the documentation shows them with these outputs, and `python/Examples/notebooks/` holds the same code as
scripts. `exudev notebooks` runs them and stores the outputs. `exudyn.interactive.ShowImage(SC)` shows the scene in a notebook -
an image of the raytracer, without a window (#2831). The examples of the Python-C++ interface chapter are notebooks
of `python/Notebooks/reference/`, those of the user manual's pages on graphics, solving and the renderer notebooks of
`python/Notebooks/snippets/`; the tests run them, so each example on these pages shows what it printed.

**Kinetic and potential energy as output variables** (`OutputVariableType.KineticEnergy`,
`PotentialEnergy`) of the bodies, beams, plates, superelements and spring-dampers, and
`exudyn.advancedUtilities.SystemEnergy` for the energy of a whole system with its loads; an item
that cannot compute its energy - with a user function defining its force - says so, and
`mbs.Inspect` does not list it. The test models `energiesTest.py` and `energiesFlexibleBodiesTest.py`
show them.

**Springs and ropes drawn as tubes.** `visualizationSettings.connectors.curveTiling` is the number of segments of a
full turn of a spring winding or of the arc of a rope on a sheave, and `connectors.drawSimplified = False` draws the windings
of the springs as a tube instead of lines and `ObjectConnectorDistance` as a rod of its `drawSize`; the rope of
`ObjectConnectorReevingSystemSprings` is one closed tube along its spans and arcs (#2839, #2843). A 3D beam without
section geometry is drawn as a line, and its contour colors follow it (#2837, #2843). `ObjectConnectorGravity`, if
shown, is a line between its markers, and `SensorLoad` is drawn at the marker of its load (#2843).

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
