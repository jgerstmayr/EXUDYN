# The Scope: a sensor viewer in the render window (revision2026c step RG6.20, #3237)

A design note of an open step: the maintainer's request as given, the decisions, what exists, the design and the
sub-steps. It leaves the repository when the step closes (WORKFLOW.md §8.4).

## 1. The request (maintainer 2026-11-04)

Several users built their own scopes in Exudyn with `graphicsDataUserFunction` on ground objects, which are not meant
for that. Therefore an integrated signal and sensor viewer, the **Scope**:

- a button in the icon bar to turn it on and off; key SHIFT+S;
- its own sub-settings in `view[0-3].window`;
- a list of sensors for the y-values: `[sensorY, componentY, sensorX=-1, componentX=0]` per line;
- only sensors with `storeInternal`; otherwise a message and a printed warning;
- the position relative to the width and height, `([2/3, 0], [1, 1/5])` for `([xmin, ymin], [xmax, ymax])`, default
  top right;
- a grey frame `[0.7, 0.7, 0.7, 0.9]`;
- `backgroundColor = [0.9, 0.9, 0.9, 0.9]` (RGBA);
- `axesColors = [0.9, 0.9, 0.9, 0.9]` (axes, ticks, labels, legends);
- `lineWidth = 2` for all plotted lines, axes, ...; sub-ticks with 50 % of it;
- ticks (with values) and sub-ticks automatic;
- `useGrid = 1` (0 off, 1 a grid at the ticks, 2 at ticks and sub-ticks);
- `verticalZoom = [ymin=0, ymax=0]`, both 0: fitted automatically;
- `horizontalZoom = [xmin=0, xmax=0]`, both 0: fitted automatically;
- `timeHistory = -1`: -1 all, otherwise e.g. 2 = the last 2 seconds;
- `xRange = 2` (default -1: fitted automatically; otherwise the last xRange seconds plotted);
- a legend, automatic: the sensor name and the component - X/Y/Z, XX/YY/... for position, stress and other sensors, or
  `[number]`;
- `xLabel = "time (s)"`, `yLabel = ""`;
- `xResolution = 4.`: how many values are drawn along the x-axis per pixel of the screen (0.25 a very low resolution);
- ideally, but probably not as editable settings - for later, possibly with an alternative to the usual fields of
  visualizationSettings -, in the coordinates of the graph, extending the plotted lines like an analytic reference or
  a threshold: a list of single lines from (x, y) to (x, y), with color and in front of or behind the other lines; a
  list of texts at (x, y) with a relative size and color; a list of points at (x, y) with a radius in pixels and a
  color (e.g. 16 flat triangles).

Further:

- the values above are the defaults of the settings;
- the colors of the plotted sensors follow a scheme of 16 colors, as PlotSensor, fixed for now; beyond 16 lines
  dotted, beyond 32 dashed, repeated every 48 lines (which should not happen);
- a sub-function or class, so that it works the same for the raytracer and OpenGL;
- for interactive and demonstration models: faster and more convenient than matplotlib, which updates slowly; the
  ultimate speed is not the aim, but it must react faster than matplotlib, and the graphics must be captured with the
  animations (saved images);
- the behaviour of the sensor traces with `showPast=True` and `showFuture=False`: plotted up to the present
  visualization time, so that the SolutionViewer keeps the plots in its animations;
- rather simple and robust; also check what is missing; a first sub-step with the open questions.

## 2. Decisions (maintainer 2026-11-04, on the eleven questions of the first evaluation)

1. **Three fixed scopes per view**, `scope0`, `scope1`, `scope2`.
2. **The settings in `view<i>.window.scope<j>`**, so that they stay out of the way (`view<i>.scopes.scope<j>` was
   the alternative; the request's place was taken).
3. **The sensor list** is a matrix of integers, `[[sensorY, componentY, sensorX, componentX, mbsNumber], ...]`,
   n x 5, with `sensorX = -1` for the time; n x 2 is accepted and means `sensorX = -1`, `componentX = 0`,
   `mbsNumber = 0`.
4. **`timeHistory` and `horizontalZoom` stay, `xRange` is dropped** (it was the same as `timeHistory`).
5. **The colors**: 16 colors, solid, then dotted, then dashed; the first seven those of `PlotLineCode` (red, green,
   blue, black, cyan, magenta, yellow, as matplotlib draws them); no markers for now.
6. **No key** for now. The Scope is switched in the settings and by an icon. Switched on while the sensor lists of
   all three scopes are empty, the scopes show the first three sensors of mbs 0 that store their data, one per scope,
   over time, with all their components (at most 3 x 16 lines) - a convenient look at the results without any
   setting.
7. **The legend**: the component names by OutputVariableType in C++ (X, Y, Z for vectors of 3, XX ... XY for 6,
   `[i]` otherwise); the Scope does not call Python.
8. **Thread safety of the sensor data**: reading the stored data without a lock while the solver appends rows is
   dangerous (a reallocation frees the memory being read) and must not be accepted, neither for the Scope nor for the
   traces. Copying the data would be slow and inconvenient for large data; therefore **a lock on the stored data of
   each sensor**, taken by the solver while it appends or resets the rows and by the renderer while it reads them,
   in the manner of the lock of the visualization state (`postProcessData->accessState`). Sensors are usually not
   written in every step, and an uncontended lock costs nanoseconds.
9. **Reset and deletion while the renderer runs**: to be checked (section 5).
10. **The extras** (lines, texts, points in plot coordinates): later, as Python lists on `SC.renderer` rather than
    visualizationSettings; a point is a circle drawn with the line width, so that the raytracer needs no triangles.
11. **`axesColor` defaults to `[0.1, 0.1, 0.1, 1]`**.
12. **A sensor without `storeInternal`**: a message once per run in the console and as renderer message, shown for
    30 seconds.

## 3. What exists

- **Sensor traces** (`visualizationSettings.sensors.traces`): `VisualizationSystemContainer::
  GetSensorsPositionsVectorsLists` reads `CSensor::GetInternalStorage()` (time in column 0) up to the visualization
  time of the system, in the render thread, without a lock; `Raytracer::SensorTraceLines` builds the lines for both
  renderers (#2877). The Scope takes the same data at the same time, so the SolutionViewer, which sets the
  visualization state, gives the plots of the animation.
- **The stored data** is a `ResizableMatrix` per sensor, reset by the solver at its start
  (`CSolverBase::InitializeSolverOutput`) and appended row by row (`CSolverBase::WriteSensorsToFile`), which
  reallocates when it grows; `MainSensor::GetInternalStorage` copies it for Python.
- **The overlay of the window**: `GetGraphicsDataStatic` builds the texts, lines and triangles of the overlay
  (computation info, ruler, colour bar) in modelview coordinates from pixels (`PixelsToModelViewCoordinates2D`) for
  both renderers; the OpenGL renderer draws it after the scene and before `SaveImage`, the icon bar after it (not in
  saved images, #3026). The raytracer draws lines into its image (`RasterizeLine`, without depth test for static
  objects) and texts (`DrawString`, `RasterizeCharacter`); the static lines all have one width.
- **Icon bar**: 26 icons drawn from code in IconBar.cpp, their state and action in GlfwClientExtended.cpp
  (`IconBarIconIsOn`, the switch of the actions); test_iconBar.py checks the order.
- **Renderer messages**: `GlfwRenderer::ShowMessage(text, timeout)`; console output from the render thread through
  `PrintDelayed`.
- **Settings types**: the structure generator knows `Float3`, `Float4`, `Index2`, `ArrayIndex` as list members;
  neither a `Float2` nor a matrix of integers exists yet.

## 4. The design

- **Settings** (`VSettingsScope`, in `view<i>.window.scope0`, `scope1`, `scope2`, and `view<i>.window.showScopes`,
  default False, switched by the icon):
  `sensors` (n x 5 or n x 2 integers, default empty), `region` (`[xmin, ymin, xmax, ymax]` relative to the window
  from its top left corner; defaults `[2/3, 0, 1, 1/5]`, `[2/3, 1/5, 1, 2/5]`, `[2/3, 2/5, 1, 3/5]`), `frameColor`
  `[0.7, 0.7, 0.7, 0.9]`, `backgroundColor` `[0.9, 0.9, 0.9, 0.9]`, `axesColor` `[0.1, 0.1, 0.1, 1]`, `lineWidth`
  2, `useGrid` 1, `verticalZoom` `[0, 0]`, `horizontalZoom` `[0, 0]`, `timeHistory` -1, `showLegend` True,
  `xLabel` `"time (s)"`, `yLabel` `""`, `xResolution` 4. The generator gets the types `Float2` and `MatrixI` (a
  list of lists of integers in Python).
  `componentY = -1` draws all components of the sensor, as the automatic mode does.
- **ScopePlot** (src/Graphics/ScopePlot.h/.cpp): the drawing of a scope as one list of primitives in **pixels of the
  window**, from its bottom left corner - filled rectangles, lines with their own width and color, texts with a size
  and color. The layout is computed once for both renderers:
  - the data of each line: the rows up to the visualization time (as the traces with `showPast`), within the last
    `timeHistory` seconds, read **under the lock of the sensor**; decimated to `xResolution` values per pixel of the
    plot width by keeping the minimum and the maximum of each pixel column in their order, so that peaks stay;
  - the x-range: `horizontalZoom`, else for time over the last `timeHistory` seconds (scrolling once the time
    exceeds it), else the data; the y-range: `verticalZoom`, else the data with 5 % margin, a constant signal $\pm$1 %
    of its value (or $\pm$1);
  - ticks at 1, 2, 5 times a power of ten, about one per 80 pixels in x and 40 in y, sub-ticks between them; the
    labels with the digits the spacing needs; the grid by `useGrid`;
  - the lines clipped to the plot area; dotted and dashed by the length along the line;
  - the legend in the top right corner of the plot area, as many entries as fit;
  - `yLabel` above the plot area (no rotated text), `xLabel` below the tick labels.
- **OpenGL**: the primitives converted to the modelview coordinates of the overlay and drawn after it, without depth
  test, **before `SaveImage`** - in saved images and animations, not under the icon bar.
- **Raytracer**: the same primitives drawn into its image after the texts, before the image is scaled down: the
  rectangles blended pixel by pixel, the lines by `RasterizeLine` as static objects with their width, the texts by
  `DrawString`. With the raytracer in the window, the window shows its image, so OpenGL draws nothing more.
- **The lock** (`CSensor::LockInternalStorage`, `ReleaseInternalStorage`): an atomic flag per sensor, taken by the
  solver while it resets or appends the stored rows, by the traces and the Scope while they read them, and by
  `MainSensor::GetInternalStorage` while it copies them.
- **The automatic mode** and **the message** as decided (section 2, 6 and 12); the message is reset when the
  renderer starts, so that it comes once per run.

## 5. Reset and deletion while the renderer runs (decision 9)

- **`SC.Reset()` and a deleted SystemContainer**: safe. `MainSystemContainer::Reset()` detaches the renderer first
  (`DetachFromRenderEngine` → `GlfwRenderer::DetachVisualizationSystem` → `StopRenderer`), which sets the stop flag,
  waits up to 1 s (20 s with the raytracer) for the render loop to end and joins the render thread; only then are
  the systems deleted.
- **`mbs.Reset()`, and adding, deleting or modifying items while the renderer runs**: not safe. `MainSystem::Reset`
  deletes the items, the sensors and their stored data without telling the renderer; the render thread may be in
  `UpdateGraphicsData` (which checks `IsSystemConsistent()` only once, at its start) or in the sensor traces (which
  check nothing) at that moment. Item changes from Python while the renderer runs are the same race, rarer because
  the items are not freed. Raised as #3239 (RG6.20.9): a lock of the system data, taken by Reset and the item
  changes and by the render thread while it reads the system, or a request to the render thread to finish its frame
  first.
- **Done (RG6.20.9)**: `PostProcessData::accessSystem`, an atomic flag per system with its owner thread, so that a
  nested lock in the same thread passes. The main thread takes it (`SystemAccessGuard`) in `MainSystem::Reset`, the
  `AddMain...`, `PyDelete...` and `PyModify...` functions of the five kinds of items, `Assemble`, and in
  `PySet...Parameter` for the names starting with `V` (visualization; the other parameters are set without the lock,
  as a pre-step function sets them every step and they are values in place); the render thread in
  `VisualizationSystem::UpdateGraphicsData` after the visualization state, in `GetSensorsPositionsVectorsLists` (the
  traces) and in `ScopePlot::BuildScopes` (all systems). While the main thread waits, it computes the user function
  drawings the render thread requests while holding the lock (else both would wait, the render thread 200 ms); inside
  these drawings the main thread does not lock (the render thread waits for them). Not covered: adding systems to a
  SystemContainer while the renderer runs.

## 6. Sub-steps (as numbered in the plan)

1. The decisions (section 2) and this design: closed.
2. VSettingsScope and the settings types Float2 and MatrixI in the definitions and the generator.
3. ScopePlot: the data, the decimation, the axes, ticks, grid and legend as one list of primitives.
4. OpenGL: the primitives drawn before the image is saved; the icon, `showScopes`, the automatic mode and the
   message.
5. The raytracer: the same primitives drawn into its image.
6. A test model (the raytracer image of a scope), the documentation in the GUI chapter.
7. Later: the extras (lines, texts, points in plot coordinates) as Python lists on `SC.renderer` - the proposal in
   section 7.
8. The lock of the stored sensor data, for the solver, the traces, the Scope and Python.
9. Reset and item changes while the renderer runs (#3239): done, section 5.

## 7. Static graphics in a scope (RG6.20.7, done)

**As done** (maintainer's decisions 2026-11-04): (a) polylines; (b) outside the automatic range; (c) a named line in
the legend; (d) one list of dicts, each with its `view` and `scope`, which each scope walks through; (e)
`SC.renderer.scopes.SetGraphics(items)`, `AddGraphics(item or items)`, `GetGraphics()`, `ClearGraphics(view=-1,
scope=-1)` (class `RendererScopes`). Each dict has `type` 'Line', 'Text' or 'Point'; defaults `color` [0,0,0,1],
`front` False (texts True), lines `width` -1 and `name` '', points `radius` 4 and `width` -1, texts `size` 1. Stored
in `VisualizationSystemContainer::scopeStaticGraphics` under an atomic flag, copied per view by `BuildScopes`; cleared
by `SC.Reset()`. scopeTest.py and scopeExample.py (the circle of the first mass, the reach of the second, the pivot).
The proposal as it was written:

The request (section 1): in the coordinates of the graph, extending the plotted lines like an analytic reference or
a threshold - lines from (x0, y0) to (x1, y1) with an RGBA color, in front of or behind the other lines; texts at
(x, y) with a relative size and color; points (circles) at (x, y) with a radius in pixels and a color - probably not
as editable settings, rather like a renderer function (decision 10: Python lists on `SC.renderer`).

- **Python interface**, functions of `SC.renderer`, each item a dict validated in C++ as the item dicts are (unknown
  keys and wrong types raise, with the name of the key):

      SC.renderer.SetScopeGraphics(view=0, scope=0,
          lines=[{'points': [[0, 1.5], [10, 1.5]], 'color': [1, 0, 0, 1], 'width': -1, 'front': False,
                  'name': 'threshold'}],
          texts=[{'position': [5, 1.6], 'text': 'limit', 'size': 1., 'color': [1, 0, 0, 1], 'front': True}],
          points=[{'position': [2.5, 0.8], 'radius': 4, 'color': [0, 0, 1, 1], 'front': True}])
      SC.renderer.GetScopeGraphics(view=0, scope=0)       #the lists as given
      SC.renderer.ClearScopeGraphics(view=-1, scope=-1)   #-1: all views or all scopes

  `SetScopeGraphics` replaces the lists of that scope; omitted lists stay. Defaults: `width` -1 (the `lineWidth` of
  the scope), `front` True for texts and points and False for lines, `size` 1 (the size of the tick labels),
  `radius` 4 pixels, `name` empty.
- **A line is a polyline**: `points` with two or more points, so that an analytic reference (e.g. 200 points of
  sin(t)) is one item; a single line is the case of two points. With a `name`, the line gets an entry in the legend,
  after the sensor lines.
- **Coordinates and range**: x and y in the units of the graph - time or the x-sensor, and the y-values. The items are
  clipped to the plot area and do not enter the automatic range (a threshold far away does not shrink the plot); a
  scope with `horizontalZoom` or `verticalZoom` shows them where they are. Over time with `timeHistory`, they scroll
  with the axis, as the sensor lines.
- **Order of drawing**: background, grid, the items with `front` False, the sensor lines, the items with `front`
  True, the axes, the legend - in the one list of primitives of `ScopePlot`, so that OpenGL, the raytracer and saved
  images show the same. A point is a circle of 16 segments drawn with the line width (decision 10; no triangles in
  the raytracer); a text uses the font of the tick labels, scaled by `size`, its position the left end of the
  baseline.
- **Storage and threads**: the lists live in C++ in `VisualizationSystemContainer`, per view and scope, not in
  `visualizationSettings` (not in `config.json`, not in `GetDictionary`); `SetScopeGraphics` replaces them under a lock
  that `ScopePlot` also takes while it reads them, as the lock of the stored sensor data. They stay until cleared or
  `SC.Reset()`, also through `mbs.SolutionViewer()`.
- **Test and documentation**: scopeTest.py with a threshold line, a reference polyline, a text and a point on the
  raytracer image (the pixel colors at their positions), the refusal of a wrong key; the GUI chapter
  (`sec-gui-scopes`) with an example in scopeExample.py (the analytic small-angle solution of the pendulum).
- **Sub-steps**: .1 the interface and the storage, .2 the primitives in `ScopePlot` (polylines, texts, circles, the
  clipping, the order, the legend entries), .3 the test and the documentation.
- **Decided**: (a) polylines; (b) outside the automatic range; (c) legend entries for named lines; (d) one list
  with the view and the scope in each item; (e) `SC.renderer.scopes.GetGraphics(...)` and its siblings.
