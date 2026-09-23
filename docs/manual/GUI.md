(sec-graphicsvisualization)=
# Graphics and visualization

The 3D OpenGL graphics renderer window is kept simple, but useful to see the animated results of the multibody system.
The graphics output is restricted to a 3D window (renderwindow) into which the renderer draws the visualization state of the `MainSystem` `mbs`.
Note that visualization parameters can be widely changed (more than 200 parameters ...), see {ref}`sec-overview-basics-visualizationsettings`.

(sec-gui-sec-mouseinput)=
## Mouse input

The following table includes the mouse functions; it is generated from the one table of bindings in
`exudyn.misc.keyBindings`, which the help dialog of the render window shows as well:

```{include} /docs/generated/mouseBindings.md
```

Current mouse coordinates can be obtained via `SystemContainer.renderer.GetMouseCoordinates()`.

### 6D mouse

Graphics engines, especially in CAD and finite elements allow input of special 3D or 6D mouse devices.
There is a basic interface for so-called 3D mouse / 6D mouse or space mouse, allowing to map the 6D joystick to translation and rotation,
see `visualizationSettings.interactive.useJoystickInput` and similar options.
The interface only works, if the device maps 6 coordinates to the joystick input of GLFW (tested with 3DCONNEXION mouse).

(sec-gui-sec-keyboardinput)=
## Keyboard input

The following table includes the keyboard shortcuts available in the window; it is generated from the
same table as the help dialog, which opens with the key **H**:

```{include} /docs/generated/keyBindings.md
```

(sec-renderstate)=
## Render state

The system container function `SC.renderer.GetState()` returns a dictionary with current information on the renderer.
This information is updated whenever the renderer performs redrawing or when according changes in the renderer are performed.

When starting with an empty `mbs` and calling `SC.renderer.Start()`, the `SC.renderer.GetState()` will return a dictionary similar to:

```python
  {'centerPoint': [0.0, 0.0, 0.0],
  'rotationCenterPoint': [0.0, 0.0, 0.0],
  'maxSceneSize': 1.0,
  'zoom': 0.4,
  'boundingBox': [[-1.0,-1.0,-1.0],[1.0,1.0,1.0]],
  'currentWindowSize': [1024, 768],
  'displayScaling': 1.0,
  'modelRotation': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
  'mouseCoordinates': [0.0, 0.0],
  'openGLcoordinates': [0.0, 0.0],
  'joystickPosition': [0.0, 0.0, 0.0],
  'joystickRotation': [0.0, 0.0, 0.0],
  'joystickAvailable': -1}
```

Note that in case that you compiled with OpenVR, there will be a separate key `openVRstate`, containing details on OpenVR, e.g., HMD pose, eye projections and controller poses.
Most entries in `renderState` are having single precision due to compatibility with values entered in OpenGL.
The most typical scenario for using `SC.renderer.SetState(...)` is to restore a previous view or to start a simulation with a specific view, projection or similar. Furthermore, mouse and joystick values can be used for interactive models.
Note that a simpler way to restore the model view is based on pressing CTRL-F3, to obtain the current model view values, see {ref}`sec-overview-basics-storingmodelview`.

There is a set of variables, which can be actively changed by calling  `SC.renderer.SetState(renderState)` with `renderState`
containing a modified dictionary:

- `centerPoint`: this is a 3D vector (list/numpy-array) containing the center point for the current view; modifying this vector allows to track objects in you simulation, **however**, it is highly recommended to use `trackMarker` in `visualizationSettings.interactive` for tracking of objects!
- `rotationCenterPoint`: the centerpoint for rotation with mouse (pressing right button)
- `maxSceneSize`: this value is used in the 3D view, clipping objects nearer or farer than this size; also used for perspective view; computed automatically based on the model
- `zoom`: this factor changes the zoom for the renderer, in fact for the size of the view; this leads to smaller objects with larger zoom values
- `boundingBox`: a list of two vectors `pMin` and `pMax`, representing the bounding box of the current view (rotated into the screen plane); thus, `pMax[0]-pMin[0]` is the width of the scene, `pMax[0]-pMin[0]` is the height of the scene, and `pMax[2]-pMin[2]` is the depth of the scene; this value cannot be set with `SC.renderer.SetState(...)`
- `modelRotation`: this is the $3 \times 3$ rotation matrix used for model rotation; changing this matrix allows to rotate the model in the view; overwriting modelRotation, centerPoint and zoom with stored values allows to reset to a certain (default) view
- `projectionMatrix`: the $4 \times 4$ matrix for camera projection (as a homogeneous transformation, according to classical OpenGL standard)

Note that other items in renderState are ignored when calling `SC.renderer.SetState(renderState)`. The read only variables in `SC.renderer.GetState()` are:

- `currentWindowSize`: contains current window size, which is different from default values in visualizationSettings, if window is scaled by user
- `displayScaling`$^*$: contains display scaling (monitor scaling; content scaling) as returned by GLFW and Windows (always 1 on Linux); used internally in renderer to scale fonts
- `mouseCoordinates`$^*$: returns 2D vector of current mouse coordinates on screen
- `openGLcoordinates`$^*$: returns 3D vector of current mouse coordinates
- `joystickAvailable`$^*$: set True, if a special 6D mouse is available (only works for special hardware, e.g., 3Dconnexion space mouse)
- `joystickPosition`$^*$: contains current joystick position vector information
- `joystickRotation`$^*$: contains current joystick rotation vector information (linearized rotation angles)

$^*$Note that values with an asterisk are only available if the renderer has already been started using `SC.renderer.Start()`.

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
**NOTE** that for correct rendering and correct contact computations, all triangle nodes must follow a strict local order and triangle normals -- if defined -- must point outwards, see {ref}`fig-trianglenormals`.

(fig-trianglenormals)=
```{figure} /docs/figures/triangleNormal.png
:width: 250

Definition of triangle normals and outside/inside regions in Exudyn
```

The normal to a triangle with vertex positions $\pv_0$, $\pv_1$, $\pv_2$ is computed from cross product as $\nv = \frac{(\pv_1-\pv_0) \times (\pv_2-\pv_0)}{|(\pv_1-\pv_0) \times (\pv_2-\pv_0)|}$;
the normal $\nv$ then points to the outside region of the mesh or body; the direction of $\nv$ just depends on the ordering of the vertex points (interchange of two points changes the normal direction); correct normals are needed for contact computations as well as for correct shading effects in visualization.

(sec-bodygraphicsdata)=
### BodyGraphicsData

`BodyGraphicsData` contains a list of `GraphicsData` items, i.e. `bodyGraphicsData = [graphicsItem1, graphicsItem2, ...]`. Every single `graphicsItem` may be defined as one of the following structures using a specific 'type'.
The following sections show the different possible types of `GraphicsData`.

### GraphicsData: Line

GraphicsData `'type' = 'Line'` draws a polygonal line between all specified points:

| **Name** | **type** | **default value** | **description** |
|---|---|---|---|
| color | list | [0,0,0,1] | list of 4 floats to define RGB-color and transparency |
| data | list | mandatory | list of float triples of x,y,z coordinates of the line floats to define RGB-color and transparency |

 **Example**:

```python
  #rectangle with side length 1:
  graphicsData = {'type':'Line',
                  'color': [1,0,0,1], #red
                  'data': [0,0,0,
                           1,0,0,
                           1,1,0,
                           0,1,0,
                           0,0,0]}

  vGround=VObjectGround(graphicsData=[graphicsData])
  oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0],
                                   visualization=vGround))
```

 Certainly this can be done **much more elegant and shorter with** `graphics.Lines`:

```python
  import exudyn.graphics as graphics
  graphicsData = graphics.Lines([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,0]],
                                color=graphics.color.red)
```

### GraphicsData: Lines

GraphicsData `'type': 'Lines'` draws a list of $n$ lines defined by 2 points each:

| **Name** | **type** | **default value** | **description** |
|---|---|---|---|
| colors | list | mandatory | list [R0,G0,B0,A0, R1,G2,B1,A1, ...] of $2\times n$ x 4 floats to define RGB-color and transparency of line points |
| points | list | mandatory | list of $2 \times n$ float triples of x,y,z coordinates of the line points; Example for two lines: data=[0,0,0, 1,0,0, 1,0,0, 1,1,0] ... draws a L-shape with side length 1 |

### GraphicsData: Circle

GraphicsData `'type' = 'Circle'` draws a polygonal line between all specified points:

| **Name** | **type** | **default value** | **description** |
|---|---|---|---|
| color | list | [0,0,0,1] | list of 4 floats to define RGB-color and transparency |
| radius | float | mandatory | radius of circle |
| position | list | mandatory | list of float triples of x,y,z coordinates of center point of the circle |

 **Example**:

```python
  graphicsData = {'type':'Circle',
                  'color': [0,0,1,1],  #blue
                  'radius': 0.5,
                  'position':[2,3,0]}
```

### GraphicsData: Text

GraphicsData `'type' = 'Text'` places the given text (mono-space font) at position:

| **Name** | **type** | **default value** | **description** |
|---|---|---|---|
| color | list | [0,0,0,1] | list of 4 floats to define RGB-color and transparency |
| text | string | mandatory | text to be displayed, using UTF-8 encoding (see {ref}`sec-utf8`); multiline texts can be written with line breaks |
| position | list | mandatory | list of float triples of [x,y,z] coordinates of the left upper position of the text; e.g. position=[20,10,0] |
| fontSize | float | 0 | scalar fontSize or 0 for default; default font size in Exudyn is 12 (visualizationSettings.view0.window.globalFontSize); display scaling increases font size |
| offset | list | [0,0] | offset in X/Y screen plane provided as list of 2 float values; this offset is not rotated with the model view and given relative to font size (offset [1,1] equals offset of one character to the right and up) |

### GraphicsData: TriangleList

GraphicsData `'type' = 'TriangleList'` draws a mesh with flat triangles for given points and connectivity; triangles may look smoothened by using appropriate normals; edges may be added optionally:

| **Name** | **type** | **default value** | **description** |
|---|---|---|---|
| points | list | mandatory | list [x0,y0,z0, x1,y1,z1, ...] containing $n \times 3$ floats (grouped x0,y0,z0, x1,y1,z1, ...) to define x,y,z coordinates of points, $n$ being the number of points (=vertices) |
| colors | list | [] | list [R0,G0,B0,A0, R1,G2,B1,A1, ...] containing $n \times 4$ floats to define RGB-color and transparency A of triangle vertices (points), where $n$ must be according to number of points; if field 'colors' does not exist, default colors will be used |
| normals | list | [] | list [n0x,n0y,n0z, ...] containing $n \times 3$ floats to define normal direction of triangles per point, where $n$ must be according to number of points; if field 'normals' does not exist, default normals [0,0,0] will be used |
| triangles | list | mandatory | list [T0point0, T0point1, T0point2, ...] containing $n_{trig} \times 3$ integers to define point indices of each vertex of the triangles (=connectivity); point indices start with index 0; the maximum index must be $\le$ points.size() |
| edges | list | [] | list [L0point0, L0point1, L1point0, L1point1, ...] containing $n_{lines} \times 2$ integers to define point indices of edges drawn on triangle mesh |
| edgeColor | list | [0,0,0,1] | list of 4 floats to define RGB-color and transparency of edges |

Examples of `GraphicsData` can be found in the Python examples and in the file `graphics.py`, see Section {ref}`sec-module-graphics`.

(sec-utf8)=
## Character encoding: UTF-8

Character encoding is a major issue in computer systems, as different languages need a huge amount of different characters,
see the amusing blog of Joel Spolsky:\
[The Absolute Minimum Every Software Developer Absolutely, Positively Must Know About Unicode ...](https://www.joelonsoftware.com/2003/10/08/the-absolute-minimum-every-software-developer-absolutely-positively-must-know-about-unicode-and-character-sets-no-excuses/)\
More about encoding can be found in [Wikipedia:UTF-8](https://en.wikipedia.org/wiki/UTF-8). UTF-8 encoding tables can be found within the wikipedia article and a comparison with the first 256 characters of unicode is provided at [UTF-8 char table](https://www.utf8-chartable.de/).

For short, Exudyn uses UTF-8 character encoding in texts / strings drawn in OpenGL renderer window.
However, the set of available UTF-8 characters in Exudyn is restricted to a very small set of characters (as compared to available characters in UTF-8).
For an example of available UTF-8 characters, see `examples/solutionViewerTest.py`.

Greek characters include all lower case characters (including variations) and only upper case characters, which are different from latin characters: $\alpha, \beta, \gamma, ... \sigma, \varphi, \varepsilon; \Gamma, \Delta, \Theta, \Lambda, \Xi, \Pi, \Sigma, \Phi, \Psi, \Omega$.
