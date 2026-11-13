(sec-overview-advanced)=
# Advanced topics

This section covers some advanced topics, which may be only relevant for a smaller group of people.
Functionality may be extended but also removed in future

(sec-overview-basics-graphicspipeline)=
## Graphics pipeline

There are basically two loops during simulation, which feed the graphics pipeline.
The solver runs a loop:

- compute step (or set up initial values)
- finish computation step; results are in current state
- copy current state to visualization state (thread safe)
- signal graphics pipeline that new visualization data is available
- the renderer may update the visualization depending on `graphicsUpdateInterval` in  `visualizationSettings.general`

The openGL graphics thread (=separate thread) runs the following loop:

- render openGL scene with a given graphicsData structure (containing lines, faces, text, ...)
- go idle for some milliseconds
- check if openGL rendering needs an update (e.g. due to user interaction)
- $\ra$ if update is needed, the visualization of all items is updated -- stored in a graphicsData structure)
- check if new visualization data is available and the time since last update is larger than a presribed value, the graphicsData structure is updated with the new visualization state

(sec-overview-basics-raytracing)=
## Raytracing

The raytracer is a renderer on the CPU, an alternative to OpenGL for each view. It draws the shadows of every light,
soft shadows, sharp and blurred reflections, transparency with refraction, emission and fog, through the materials of
the faces ({ref}`sec-raytracer-materials`). It needs neither a window nor a graphics card, so it also gives images on a
computer without a display, e.g. for the images of a learning task or on a cluster. An image takes from a fraction of
a second to several seconds, depending on its size and on the effects used.

(fig-raytracerdemo)=
```{figure} /docs/figures/raytracerDemo.jpg
:width: 400

Example image of raytraced renderer view.
```

(sec-raytracer-usage)=
### Using the raytracer

- `SC.visualizationSettings.view0.camera.useRaytracer = True` draws the view with the raytracer, in the render window
  and in the SolutionViewer; `SC.renderer.RedrawAndGetImage(useRaytracer=True)` returns one image without any window,
  see {ref}`sec-sc-renderer`. Without a running renderer, set the view first with `SC.renderer.ZoomAll()` or
  `SC.renderer.SetModelView(...)`.
- Its own settings are `visualizationSettings.raytracer`, see {ref}`sec-vsettingsraytracer`, and
  `visualizationSettings.raytracer.advanced`, see {ref}`sec-vsettingsraytraceradvanced`; in the following,
  `visualizationSettings` is left out.
- It takes the scene from the OpenGL settings: the lights `openGL.light0` to `openGL.light3` with their `position`,
  `diffuse`, `specular`, attenuation, `shadow` (the raytracer only distinguishes 0 and > 0) and `lightRadius`; the
  ambient light `openGL.lightModelAmbient`; `openGL.lineWidth` and `openGL.faceTransparencyGlobal`; the background
  `general.backgroundColor`, `general.backgroundColorBottom` and `general.useGradientBackground`; and from the view the
  camera (`perspective`, the clipping plane) and the scene (`showLines`, `showFaceEdges`, `facesTransparent`).
- Faces without a material are drawn with material 0, `default`, which has no reflection; the effects need a
  material, {ref}`sec-raytracer-materials`.
- The raytracer has its own threads, `raytracer.numberOfThreads` (0: all logical processors), so it also renders while
  a solver runs. `raytracer.verbose = 1` prints the times of an image.
- The text of the render window (solver information, version, ...) is drawn over the image, see
  `view0.window.showComputationInfo`; `raytracer.advanced.showText = False` removes all text from raytraced images.

To add raytracing to a project, give the triangles a material, e.g.
`color=graphics.color.dodgerblue[0:3]+[graphics.material.indexChrome]`, and set `view0.camera.useRaytracer = True`
before `mbs.SolutionViewer()`; the example at the end of {ref}`sec-graphicsdata` does so and shows the image.

(sec-raytracer-quality)=
### Image quality and cost

The cost of an image is the number of rays traced. Every pixel traces at least one ray from the camera; each effect
adds rays where it is seen:

| what | setting | rays |
|---|---|---|
| image size | `view0.window.renderWindowSize` | one per pixel |
| smaller image | `raytracer.imageSizeFactor` = $k$ | $1/k^2$ of them, the image is $k$ times smaller |
| anti-aliasing | `raytracer.multiSampling` = $m$ | $m^2$ per pixel; smooths edges and lines, and the grain of soft shadows and blurred reflections |
| shadow | `openGL.light0.shadow` > 0 | one per light and hit |
| soft shadow | `openGL.light0.lightRadius` > 0, `raytracer.lightRadiusVariations` = $N$ > 1 | up to $N$ per light and hit, at the edges of shadows only (`raytracer.advanced.adaptiveShadowSamples`) |
| reflection | material `reflectivity` > 0 | one per reflecting hit, up to `raytracer.maxReflectionDepth` levels |
| blurred reflection | material `roughness` > 0 | `raytracer.reflectionSamples` per reflecting pixel |
| transparency | material `alpha` < 1 | one per transparent face, up to `raytracer.maxTransparencyDepth` faces |

**Soft shadows.** A light with a `lightRadius` is a disk; the raytracer takes `lightRadiusVariations` samples of it,
on a spiral that is rotated from hit to hit. Few samples therefore give grain instead of rings. The grain is removed
on a **shadow map** of a lower resolution, the resolution of the window divided by
`raytracer.advanced.shadowScalingFactor`: it is smoothed `raytracer.advanced.shadowSmoothingSteps` times, between
neighbours of similar depth only, and the pixels interpolate it. The map exists only if `shadowScalingFactor` times
`raytracer.multiSampling` is larger than 1; with `shadowScalingFactor = 1` and `multiSampling = 1`, every pixel takes
its own samples, the soft shadows are grainy and the smoothing has no effect. In the scene of
`raytracerShadowBenchmark.py` (1200 $\times$ 1000 pixels, 13 samples):

| `shadowScalingFactor`, `shadowSmoothingSteps` | time | soft shadow |
|---|---|---|
| 3, 3 (the defaults) | 0.10 s | smooth |
| 2, 3 | 0.12 s | smooth, sharper edges |
| 2, 0 | 0.11 s | grainy |
| 1, any, with `multiSampling = 1` | 0.21 s | grainy, no map |

More than 3 smoothing steps gain nothing; more samples (e.g. 31) give a little more detail at about 10 % more time.
Soft shadows are computed for the faces seen directly; a face seen in a reflection or through a transparent face gets a
hard shadow, and `raytracer.advanced.minimumShadowWeight` can save even these where the reflection is weak.

**Blurred reflections.** A material with a `roughness` > 0 reflects `raytracer.reflectionSamples` rays (default 8),
spread over a cone, for the faces seen directly. Fewer rays are faster in proportion but grainy: 2 or 3 rays suffice
for a roughness up to about 0.02, or together with `raytracer.multiSampling = 2`, whose sub-pixels average the grain.

Start with a small window or with `raytracer.imageSizeFactor` > 1 while adjusting a scene, and add
`multiSampling = 2` for the final images only: from `multiSampling = 1` and `imageSizeFactor = 4` to
`multiSampling = 3` and `imageSizeFactor = 1`, the cost grows by $4 \times 4 \times 3 \times 3 = 144$.

(sec-raytracer-materials)=
### Materials

A **material** decides how a face reflects light: its colors, the highlights, reflection, transparency with
refraction and emission. OpenGL does not use materials.

**Selecting a material.** A face selects a material by the fourth value of its RGBA color: $1000+i$ selects material
$i$, and the value of a usual alpha ($\le 1$) selects material 0. A red value of $-1$ takes the color of the face
from the material's `baseColor`. `graphics.material` gives both: the index, e.g. `graphics.material.indexSteel`
(1002), and the color, e.g. `graphics.material.steel`, which is `[-1,-1,-1,1002]`:

- `color=[1,0,0,graphics.material.indexSteel]`: red, with the reflection and highlights of steel;
- `color=graphics.material.steel`: steel in its own `baseColor`, which changes all such faces at once;
- OpenGL draws an alpha > 1 as 1 and a red value of $-1$ as black.

Materials act on faces (triangles, as in all `graphics` functions that create bodies); lines and text have none.

**The properties** of a material are those of {ref}`sec-vsettingsmaterial`, where their defaults are listed:

| property | effect in the raytracer |
|---|---|
| `name` | the name, by which `SC.renderer.materials.Get(...)` and `Set(...)` find the material |
| `baseColor` | RGB color of the faces whose color has the red value $-1$ |
| `specular` | RGB color of the highlights (Phong), multiplied by the `specular` value of each light |
| `shininess` | exponent of the highlights: below 5 broad and dull, above 50 small and sharp |
| `reflectivity` | share of the reflected color, 0 none, 1 a perfect mirror |
| `roughness` | angular radius in radians of the cone of the reflected rays: 0 sharp, 0.02 polished, 0.1 brushed |
| `alpha` | opacity, replacing the alpha of the color: 1 opaque, 0 fully transparent; shadows let light through in proportion to $1-\alpha$ |
| `ior` | index of refraction of a transparent material: 1 straight through, 1.5 glass; the ray enters at a face whose normal points against it and leaves at one whose normal points along it |
| `emission` | RGB color the face adds by itself, independent of the lights; it does not light other faces |

**The ten predefined materials** are `raytracer.material0` to `raytracer.material9`; their values are listed in
{ref}`sec-vsettingsraytracer`:

| index | name | `graphics.material` | character |
|---|---|---|---|
| 0 | `default` | `default`, `indexDefault` | blue, no reflection; used by faces without a material |
| 1 | `matt` | `matt`, `indexMatt` | green, dull |
| 2 | `steel` | `steel`, `indexSteel` | grey, slightly reflective |
| 3 | `plastic` | `plastic`, `indexPlastic` | red, slightly reflective |
| 4 | `chrome` | `chrome`, `indexChrome` | light grey, reflective |
| 5 | `shiny` | `shiny`, `indexShiny` | orange, strongly reflective |
| 6 | `transparent` | `transparent`, `indexTransparent` | transparent, slight refraction |
| 7 | `glass` | `glass`, `indexGlass` | transparent, reflective, refraction of glass |
| 8 | `mirror` | `mirror`, `indexMirror` | a mirror |
| 9 | `emission` | `emission`, `indexEmission` | light yellow, emitting |

**Changing and adding materials.** The materials are kept in `SC.renderer.materials`, a GraphicsMaterialList, see
{ref}`sec-graphicsmateriallist`. Its first ten are `raytracer.material0` to `material9`: `Set` writes into these
settings, and the raytracer copies the settings into the list before every image. Change a predefined material in one
of the two ways, not both in the same script:

```python
SC.visualizationSettings.raytracer.material4.reflectivity = 0.5   #chrome, by the settings
#or by name, as VSettingsMaterial:
chrome = SC.renderer.materials.Get('chrome')
chrome.roughness = 0.03
SC.renderer.materials.Set('chrome', chrome)
gold = SC.renderer.materials.New()                                 #a new material, index 10
gold.name = 'gold'
gold.baseColor = [0.9, 0.7, 0.2]
gold.reflectivity = 0.4
index = SC.renderer.materials.Append(gold)                         #used by the color [-1,-1,-1,1000+index]
```

The materials are stored with the renderer's dictionary, `SC.renderer.GetDictionary()`, and the first ten with the
visualization settings.

**Limitations.** The raytracer runs on the CPU: keep the image small (e.g. 600 $\times$ 400 to start) and do not
combine a large image (4K) with `multiSampling` > 1. An image that takes longer than a few seconds in the render window
may need `raytracer.keepWindowActive`. Start with the default settings, and change them in the visualization settings
dialog.

(sec-graphicsdata)=
## GraphicsData

All graphics objects are defined by a `GraphicsData` structure.
Note that currently the visualization is based on a very simple and ancient OpenGL implementation, as there is currently no simple platform independent alternative. However, most of the heavy load triangle-based operations are implemented in C++ and are realized by very efficient OpenGL commands. However, note that the number of triangles to represent the object should be kept in a feasible range ($<1000000$) in order to obtain a fast response of the renderer.

Many objects include a `GraphicsData` dictionary structure for definition of attached visualization of the object.
Note that objects expect a list of `GraphicsData`, which can be produced with `exudyn.graphics. ...` functions (until Exudyn 1.8.33 with `GraphicsData...(...)`, which are now deprecated).
Note that if reading out the `GraphicsData` from the object again, it usually has a different structure sorted by types of `GraphicsData`.
Typically, you can use primitives (cube, sphere, ...) or {ref}`STL <STL>` data to define the objects appearance.
`GraphicsData` dictionaries can be created with functions provided in the utility module `exudyn.graphics`, see {ref}`sec-module-graphics`.

`GraphicsData` can be transformed into points and triangles (mesh) and can be used for contact computation, as well.
**NOTE** that for correct rendering and correct contact computations, all triangle nodes must follow a strict local order and triangle normals -- if defined -- must point outwards, see {numref}`fig-trianglenormals`.

(fig-trianglenormals)=
```{figure} /docs/figures/triangleNormal.*
:width: 250

Definition of triangle normals and outside/inside regions in Exudyn
```

The normal to a triangle with vertex positions $\pv_0$, $\pv_1$, $\pv_2$ is computed from cross product as $\nv = \frac{(\pv_1-\pv_0) \times (\pv_2-\pv_0)}{|(\pv_1-\pv_0) \times (\pv_2-\pv_0)|}$;
the normal $\nv$ then points to the outside region of the mesh or body; the direction of $\nv$ just depends on the ordering of the vertex points (interchange of two points changes the normal direction); correct normals are needed for contact computations as well as for correct shading effects in visualization.

(sec-bodygraphicsdata)=
### BodyGraphicsData

`BodyGraphicsData` contains a list of `GraphicsData` items, i.e. `bodyGraphicsData = [graphicsItem1, graphicsItem2, ...]`. Every single `graphicsItem` may be defined as one of the following structures using a specific 'type'.
The following sections show the different possible types of `GraphicsData`.
Points, normals, colors, triangles and edges are given as **rows** - one row per point, color, triangle or edge,
as a list of lists or a 2D numpy array - and the functions of `exudyn.graphics` return them so; a flat list of the
same numbers is read as well. A script that indexes such a list as flat reshapes it first, e.g.
`np.array(g['points']).reshape(-1,3)`, which works for both forms.

### GraphicsData: Line

GraphicsData `'type' = 'Line'` draws a polygonal line between all specified points:

| **Name** | **type** | **default value** | **description** |
|---|---|---|---|
| color | list | [0,0,0,1] | list of 4 floats to define RGB-color and transparency |
| data | array | mandatory | the points of the polygonal line as rows of x,y,z coordinates |

### GraphicsData: Lines

GraphicsData `'type': 'Lines'` draws a list of $n$ lines, straight with 2 points each or quadratic (curved) with 3 points each:

| **Name** | **type** | **default value** | **description** |
|---|---|---|---|
| points | array | mandatory | the line points as rows of x,y,z coordinates, $2n \times 3$ for straight lines and $3n \times 3$ for quadratic ones (a flat list of the same numbers is read as well); a quadratic line is given by its end points and then its mid point |
| colors | array | mandatory | the RGBA colors of the line points as rows, $2n \times 4$ or $3n \times 4$ (or flat) |
| shape | string | 'linear' | 'linear' for straight lines, 'quadratic' for curved ones, each a quadratic curve through its three points; they are kept as they are and split into straight lines when drawn, with the settings of that moment: as many segments as the angle between the curve's end tangents needs against `visualizationSettings.openGL.advanced.curvedTriangleTilingAngle`, at most `curvedTriangleMaxTiling`; `graphics.Lines(..., shape='quadratic')` builds such a polyline from the points along the curve |

### GraphicsData: Circle

GraphicsData `'type' = 'Circle'` draws a polygonal line between all specified points:

| **Name** | **type** | **default value** | **description** |
|---|---|---|---|
| color | list | [0,0,0,1] | list of 4 floats to define RGB-color and transparency |
| radius | float | mandatory | radius of circle |
| position | list | mandatory | list of float triples of x,y,z coordinates of center point of the circle |

### GraphicsData: Text

GraphicsData `'type' = 'Text'` places the given text (mono-space font) at position:

| **Name** | **type** | **default value** | **description** |
|---|---|---|---|
| color | list | [0,0,0,1] | list of 4 floats to define RGB-color and transparency |
| text | string | mandatory | text to be displayed, using UTF-8 encoding (see {ref}`sec-utf8`); multiline texts can be written with line breaks |
| position | list | mandatory | list of float triples of [x,y,z] coordinates of the left upper position of the text; e.g. position=[20,10,0] |
| fontSize | float | 0 | scalar fontSize or 0 for default; default font size in Exudyn is 12 (visualizationSettings.view0.window.globalFontSize); display scaling increases font size |
| offset | list | [0,0] | offset in X/Y screen plane provided as list of 2 float values; this offset is not rotated with the model view and given relative to font size (offset [1,1] equals offset of one character to the right and up) |

### GraphicsData: Spheres

GraphicsData `'type' = 'Spheres'` draws $n$ spheres; the OpenGL renderer draws them with the given resolution, the raytracer intersects them exactly. `graphics.Sphere` returns this type for a whole sphere, `graphics.Spheres` for many at once (particles, point clouds); a sphere with edges, a part of a sphere or a hollow one is a `TriangleList`. The functions of `exudyn.graphics` that need triangles (`MergeTriangleLists`, `ExportSTL`, ...) convert spheres with `graphics.SpheresToTriangleList`, and `graphics.Move` with a rotation matrix that scales unequally (an ellipsoid) as well:

| **Name** | **type** | **default value** | **description** |
|---|---|---|---|
| points | array | mandatory | the centers as rows [x,y,z], $n \times 3$ |
| radii | list or float | 0.1 | one radius for all spheres, or a list of $n$ radii |
| colors | array | [0,0,0,1] | one RGBA color [R,G,B,A] for all spheres, or one row [R,G,B,A] per sphere, $n \times 4$; a material index in A as for `TriangleList` |
| resolution | int | 8 | number of segments of a half circle when drawn by OpenGL, rounded down to a power of 2 |

### GraphicsData: TriangleList

GraphicsData `'type' = 'TriangleList'` draws a mesh with flat triangles for given points and connectivity; triangles may look smoothened by using appropriate normals; edges may be added optionally:

| **Name** | **type** | **default value** | **description** |
|---|---|---|---|
| points | array | mandatory | the points (vertices) as rows [x,y,z], $n \times 3$, $n$ being the number of points |
| colors | array | [] | the RGBA color of each point as rows [R,G,B,A], $n \times 4$; if field 'colors' does not exist, default colors will be used |
| normals | array | [] | the normal at each point as rows [nx,ny,nz], $n \times 3$; if field 'normals' does not exist, the normals of the flat triangles are used, and for 6-node triangles at each point the mean of the normals of the geometry of the triangles sharing it |
| triangles | array | [] | the triangles as rows of 3 point indices (=connectivity), $n_{trig} \times 3$, counter-clockwise seen from outside; point indices start with 0 and must be $<$ the number of points; mandatory if there are no triangles6 |
| triangles6 | array | [] | rows [c0,c1,c2, m01,m12,m20] of 6 point indices, $n_{trig6} \times 6$, of 6-node (quadratic, curved) triangles: the corners counter-clockwise seen from outside, then the mid-side nodes between the corners 0-1, 1-2 and 2-0 (`graphics.FromPointsAndTrigs` with 6 indices per row, `graphics.NGsolveMesh2PointsAndTrigs(..., triangles6=True)`); they are kept as they are and split into flat triangles when drawn (OpenGL, raytracer; `SC.renderer.GetGraphicsData()` returns them as they are, or refined once), with the settings of that moment: each edge subdivided until the angle between its end tangents, and between the given normals of its nodes, is below `visualizationSettings.openGL.advanced.curvedTriangleTilingAngle`, at most `curvedTriangleMaxTiling` times, the inside following the three edges - a surface curved in one direction is not subdivided along the other |
| edges | array | [] | the edges drawn on the triangle mesh as rows of 2 point indices, $n_{lines} \times 2$ |
| edges3 | array | [] | quadratic (curved) edges, rows [p0, p1, m01] of three point indices: the end points, then the mid point - the order of `triangles6`; for feature edges on a surface of 6-node triangles, such as the rim of a cylinder; drawn as the quadratic lines of `Lines`, in `edgeColor` |
| edgeColor | list | [0,0,0,1] | list of 4 floats to define RGB-color and transparency of edges |
| texture | str | none | the name of a texture added before with `SC.renderer.AddTexture(...)`; the triangles are drawn with it, multiplied with the colors of their points unless the texture has `exu.TextureFlags.Replace`; 6-node triangles with a texture are drawn as the four flat triangles on their nodes (#3030) |
| textureCoordinates | array | mandatory with texture | the coordinates (u, v) of each point in the texture as rows, $n \times 2$; (0, 0) is the bottom left and (1, 1) the top right corner of the image, values outside repeat the image unless it has `exu.TextureFlags.Clamp`; a point with nan has none, and its triangles are drawn without the texture, as `graphics.MergeTriangleLists` gives the points of a list without texture (#3031) |

(sec-graphicsdata-example)=
### GraphicsData: an example of all types

One model with each type of `GraphicsData` - as a dictionary, and as the functions of `exudyn.graphics` return it -
and a chrome sphere, drawn with the raytracer. More are in the Python examples and in `graphics.py`, see Section
{ref}`sec-module-graphics`.

```{include} /docs/generated/notebooks/snippets/graphics.md
```

(sec-overview-advanced-contact)=
## Contact problems

Since Q4 2021 a contact module is available in Exudyn.
This separate module `GeneralContact` [**still under development, consider with care!**] is highly optimized and implemented with parallelization (multi-threaded) for certain types of contact elements.

(fig-contactexamples)=
```{figure} /docs/figures/contactTests.png
:width: 450

Some tests and examples using `GeneralContact`
```

```{figure} /docs/figures/contactTests2.jpg
:width: 450
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

(sec-overview-advanced-julia)=
## Interaction with Julia

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
  mbs.AddObject(eii.ObjectMassPoint2D(mass=10, nodeNumber=nMP ))
  mMP = mbs.AddMarker(eii.MarkerNodePosition(nodeNumber = nMP))
  mbs.AddLoad(eii.Force(markerNumber = mMP, loadVector=[0.001,0,0]))

  #add a sensor:
  s = mbs.AddSensor(eii.SensorNode(nodeNumber=nMP,
                    outputVariableType=exu.OutputVariableType.Position,
                    storeInternal=True))

  mbs.Assemble()                     #assemble system and solve
  simulationSettings = exu.SimulationSettings()
  simulationSettings.timeIntegration.verboseMode=1 #provide some output
  simulationSettings.solution.file.name = 'solution/demo1.txt'

  mbs.SolveDynamic(simulationSettings)
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
(sec-advanced-cppcore)=
## The C++ core

The computation happens in C++: `mbs.AddObject(...)` creates an object on the C++ side from a
validated dictionary, `mbs.SolveDynamic(...)` runs a C++ solver, and Python holds the model and
reads the results. Nothing of that needs to be known to use Exudyn - but two questions come up
often enough to say where they are answered, both in the
[developer documentation](../dev/README.md), which is part of this documentation:

- *How is the C++ side organised, and where do I start reading?* -
  [C++ architecture](../dev/ARCHITECTURE.md): the item abstraction, the split into computational,
  main and visualization classes, what each directory under `src/` is, and what happens when
  `mbs.AddObject(...)` is called.
- *I want to add my own item in C++.* - [coding style](../dev/CODING_STYLE.md), section 9: the two
  worked cases (a body and a connector), and the advice that comes before them - write it in
  Python first, with user functions, and go to C++ when that works and is too slow.

Those pages say what the C++ side is and how to work on it, and that is as far as prose goes: for
a deeper understanding of the core, and for any low-level change, it is inevitable to visit and
study the [GitHub project](https://github.com/jgerstmayr/EXUDYN) itself - the sources, the
generators that write parts of them, and the history that says why something is the way it is.

Building Exudyn from source is {ref}`sec-install-installinstructions`.

## Interaction with other codes

Interaction with other codes and computers (E.g., MATLAB or other C++ codes, or other Python versions)
is possible.
To connect to any other code, it is convenient to use a TCP/IP connection. This is enabled via
the `exudyn.utilities` functions

- `CreateTCPIPconnection`
- `TCPIPsendReceive`
- `CloseTCPIPconnection`

Basically, data can be transmitted in both directions, e.g., within a preStepUserFunction. In Examples, you can find
 TCPIPexudynMatlab.py which shows a basic example for such a connectivity.

The interface to ROS is in {ref}`sec-robotics-ros`.

(sec-overview-advanced-tools)=
## Tools that are not part of a model

Three things that are used beside a model rather than inside one: the command line of
the installed package, the results monitor, which watches a running simulation
from a second process, and the settings that persist between runs.

```{toctree}
:maxdepth: 2

commandLine
resultsMonitor
userSettings
```
