# The Scope: a sensor viewer in the render window (revision2026c step RG6.20, #3237)

A design note of an open step: the maintainer's request as given, then what exists, the evaluation, the open questions
and the sub-steps. It leaves the repository when the step closes (WORKFLOW.md §8.4).

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

## 2. What exists

- **Sensor traces** (`visualizationSettings.sensors.traces`, VSettingsTraces): `VisualizationSystemContainer::
  GetSensorsPositionsVectorsLists` reads `CSensor::GetInternalStorage()` (time in column 0) up to the visualization
  time of the system (`GetCData().GetVisualization().GetTime()`), with `showPast`/`showFuture`/`showCurrent`
  (`UseTraceData`). The Scope takes the same data at the same time, so the SolutionViewer, which sets the visualization
  state, gives the plots of the animation.
- **Per-view window settings**: `VSettingsView` (`view0`..`view3`) with `camera`, `scene`, `window`; `VSettingsWindow`
  has `showIconBar`, `showRuler`, `showMouseCoordinates`, `showRenderStateInfo`, `globalFontSize`, ... The Scope's
  settings fit as `view<i>.window.scope` (a structure of its own, VSettingsScope) - or `view<i>.scope`, see Q2.
- **Drawing in window coordinates**: the icon bar (src/Graphics/IconBar.cpp, `GlfwRenderer::DrawIconBar`) draws after
  the frame is saved, so it is not in saved images (#3026); the Scope must be drawn **before** the image is saved
  (requested: captured with the animations). Text: `GlfwRenderer::DrawString` (bitmap or texture fonts). An
  orthographic pass in window coordinates exists (`glOrtho(0, 1, 1, 0, 0, 1)` in GlfwClient.cpp for the shadow
  plane).
- **Raytracer** (src/Graphics/Raytracing.cpp): `RasterizeLine` (with z-buffer and colors), `RasterizeCharacter`,
  `DrawString` - lines and text in pixels exist, triangles in pixels (filled background, points) do not; sensor
  traces are drawn there by `SensorTraceLines`.
- **Icon bar**: 26 icons with names and tooltips in IconBar.cpp, drawn from code (no image files); a new icon "scope"
  is a drawing function and an entry. **Keys**: python/exudyn/misc/keyBindings.py is the one table of the help, the
  documentation and the check against GlfwClient.cpp (keyBindingsEmitter.py). S shows and hides the sensors, CTRL+S
  is bound as well; SHIFT+S is free in GlfwClient.cpp, but keyBindings.py treats SHIFT as "a variant of a binding,
  never one of its own", so SHIFT+S needs that rule extended (Q6).
- **Colors**: `exudyn.graphics.colorList` (16 colors) and `PlotLineCode` (28 codes, 7 colors x 4 line styles) of
  PlotSensor - not the same scheme; the request's 16 colors x 3 styles is a third (Q5).
- **Sensor names and components**: a sensor has a name; the component names of an output variable (X, Y, Z, XX, ...)
  are known in Python (PlotSensor labels them) - in C++ they would need a table by OutputVariableType and size (Q7).

## 3. Evaluation and recommendations

- **One drawing description for both renderers**: a C++ class `ScopePlot` that builds, from the settings, the sensor
  data and the window size in pixels, a list of 2D primitives - lines (with width and color, dashed/dotted as
  segments), filled rectangles and triangles, texts (position, size, color, alignment) - in pixel coordinates. OpenGL
  draws the list in an orthographic pass after the scene and before the image is saved; the raytracer rasterizes the
  same list into its image after the 3D scene (lines and text exist there; filled triangles in screen space are to be
  added, e.g. scanline, for the background and the points). The layout (ticks, labels, legend) is then computed once.
- **Data**: per line the stored data of `sensorY` (column 1 + componentY) against time or against the component of
  `sensorX`; only rows up to the visualization time (as traces with showPast), the last `timeHistory` seconds. The
  internal storage is written by the solver thread while the renderer reads it - the traces do the same today; the
  Scope reads under the same conditions (the visualization data lock of the renderer, Q8).
- **Resolution**: with `xResolution` the rows are decimated to about `xResolution * width` points by taking, per
  pixel column, the minimum and maximum (so peaks stay visible) - cheap and robust.
- **Axes**: automatic ticks by the "nice numbers" rule (1, 2, 5 x 10^k) for about 5 major ticks, 4 or 5 sub-ticks;
  labels with the fewest digits that distinguish the ticks; autofit with 5 % margin; a constant signal gets a range of
  +-1 around it (or +-1 % of its value).
- **Settings**: a structure VSettingsScope generated from definitions/structureDefsVisualizationSettings.py with the
  fields of section 1; the sensor list as `sensors` (a list of lists, Python only at first; C++ needs a fixed type:
  a matrix of n x 4 integers, Q3).
- **Several scopes**: the request has one per view; two plots in one view (e.g. positions and forces) would need a list
  of scopes - recommended later, with the same class (Q1).
- **Extras (lines, texts, points in plot coordinates)**: kept for later, as asked; a Python-side list on the scope
  object of the renderer, `SC.renderer.scope[i].AddLine(...)`, instead of visualizationSettings fields, would avoid
  the settings problem (Q9).
- **Not recommended**: drawing with matplotlib into a texture, or a second window - slower and not in the saved image.

## 4. Open questions (to be decided by the maintainer)

1. One scope per view (as requested), or a list of scopes per view, each with its own region?
2. The place of the settings: `view<i>.window.scope` (as requested, the window settings) or `view<i>.scope`, beside
   camera, scene and window? Recommended: `view<i>.scope`, as it is no window property.
3. The sensor list: `[[sensorY, componentY, sensorX, componentX], ...]` as a matrix of integers in C++ (`ResizableMatrix`
   of n x 4, -1 for "time"); and for systems with several mbs: the sensors of mbs 0 only, or a fifth column with the
   mbs number?
4. `xRange` and `horizontalZoom` and `timeHistory` overlap: `timeHistory` limits the data, `xRange` the shown range of
   time, `horizontalZoom` a fixed range. Recommended: keep `timeHistory` and `horizontalZoom`, drop `xRange`
   (`timeHistory` = 2 shows the last 2 s), or say which wins.
5. The color scheme: `graphics.colorList` (16 colors) with solid / dotted / dashed, as requested - or PlotLineCode of
   PlotSensor, so that a scope and a PlotSensor plot of the same sensors look alike? Recommended: the requested 16 x 3
   scheme, and PlotSensor unchanged.
6. SHIFT+S next to S (sensors on/off): fine, with keyBindings.py extended for SHIFT as a binding of its own - or
   another key?
7. The legend: the component names by OutputVariableType in C++ (X, Y, Z for vectors of 3, XX ... XY for 6, [i]
   otherwise) - enough, or the names of PlotSensor (which Python knows)?
8. Thread safety: the stored data of a sensor grows while the solver runs; the traces read it without a lock. Accept
   the same (rare glitches while a row is appended), or copy under the lock of the renderer?
9. The extras (lines, texts, points): later as Python lists on `SC.renderer` rather than visualizationSettings - agreed?
10. `backgroundColor` and `axesColors` both default to `[0.9, 0.9, 0.9, 0.9]` in the request - the axes would then be
    invisible on the background; intended default for `axesColors` e.g. `[0.1, 0.1, 0.1, 1]`?
11. Sensors without `storeInternal`: the message once per run in the console and as text in the scope region?

## 5. Sub-steps proposed

1. The open questions (section 4) decided.
2. VSettingsScope in the definitions, the regenerated settings and the dialog entries.
3. `ScopePlot`: the data, the decimation, the axes, the ticks, the legend - the list of 2D primitives; a C++/pytest
   test of the layout without a window (via the graphics data of the renderer).
4. OpenGL: the orthographic pass before the saved image; the icon and SHIFT+S.
5. Raytracer: the same list rasterized, filled triangles in screen space added.
6. A test model and an example (a pendulum with a scope of angle and velocity); the documentation in the GUI chapter.
7. Later: several scopes per view; the extras (lines, texts, points).
