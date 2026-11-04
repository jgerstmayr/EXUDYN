# The scopes of the render window

The scopes plot stored sensor values in regions of the render window, drawn by OpenGL and the raytracer and part of
saved images and of the animations of the solution viewer (#3237). How a user sets them is {ref}`sec-gui-scopes`;
this page is how they are built.

## 1. The settings

- **Per scope** (`VSettingsScope`, three per view: `view<i>.window.scope0`, `scope1`, `scope2`; shown with
  `view<i>.window.showScopes` or the scope icon of the icon bar):
  - `sensors`: a matrix of integers (`MatrixI`, a list of lists in Python), rows `[sensorY, componentY, sensorX,
    componentX, mbsNumber]`, `sensorX = -1` for the time, or rows of two, `[sensorY, componentY]`, over time in
    mbs 0; `componentY = -1` all components;
  - `location`: `[[xmin, ymin], [xmax, ymax]]` relative to the window from its top left corner (the settings type
    `StdArray22F`, #3251); the defaults are the right third of the width and a third of the height each, so that the
    three scopes fill the height (#3252);
  - `minMaxX`, `minMaxY`: the ranges, both 0 fitted; `timeHistory`: the last seconds, -1 all; `useGrid` (0, 1 at
    the ticks, 2 also at the sub-ticks); `showLegend`, `xLabel`, `yLabel`; `xResolution` (1).
- **Common to all scopes** (`VSettingsScopes` at `visualizationSettings.sensors.scopes`, #3251): `frameColor`,
  `backgroundColor`, `axesColor`, `lineWidth` (the plotted lines, axes and ticks; half of it for sub-ticks and grid)
  and `fontSizeFactor` (0.75), which multiplies `window.globalFontSize` of the view and the display scaling.
- **The automatic mode**: if the sensor lists of all three scopes of a view are empty, the scopes show the first
  three sensors of mbs 0 that store their values, one per scope, over time, with all components.

## 2. ScopePlot: one list of primitives for both renderers

`src/Graphics/ScopePlot.h/.cpp` builds the drawing of the scopes of a view as one list of primitives in **pixels of
the window** from its bottom left corner - filled rectangles, lines with their own width and color, texts with a
size and color (`ScopePlot::Graphics`). `BuildScopes` is called by both renderers:

- **The data of a line** (`LineSources`, `CollectCurve`): the stored rows of the sensor up to the visualization time
  of its system, as the sensor traces with `showPast` take them, within the last `timeHistory` seconds, read under
  the lock of the sensor. Over time they are reduced to `xResolution` values per pixel of the plot width by keeping
  the minimum and the maximum of each pixel column in their order, so that peaks stay, with one value on each side
  outside the range. Over another sensor every value is taken, every (1/`xResolution`)-th for `xResolution` < 1, and
  the last one (#3252).
- **The ranges** (`HorizontalRange`, `AddScope`): x from `minMaxX`, else over time the last `timeHistory` seconds
  (scrolling once the time exceeds them), else the values with 5 % margin; y from `minMaxY`, else the values with
  5 % margin, a constant signal +-1 % of its value (or +-1).
- **The layout** (`AddScope`): background and frame; ticks at 1, 2 or 5 times a power of ten
  (`TickSpacing`), sub-ticks between them, labels with the digits the spacing needs (`TickLabel`); the left margin
  from the widest tick label; the grid by `useGrid`; `yLabel` above the plot, `xLabel` below the tick labels; a small
  scope leaves the axis labels out.
- **The lines**: 16 colors, the first seven those of `PlotLineCode`, solid, beyond 16 dotted and beyond 32 dashed
  (`LineColor`, `LineStyle`), at most 48 lines; clipped to the plot area (Liang-Barsky), dotted and dashed by the
  length along the line (`AddPolyline`).
- **The legend** in the top right corner of the plot, as many entries as fit: the sensor name and, for a sensor of
  more than one component, `-` and the component (`m0.Pos-X`; `X`, `Y`, `Z` for vectors, `XX` ... `XY` for strain and
  stress, else `[i]`), and `m0.Pos-Y over m0.Pos-X` over another sensor (#3248); then the named static lines.
- **The message**: a line that cannot be shown (no such mbs or sensor, a sensor without `storeInternal`, a component
  out of range, rows of a wrong length) gives one message per start of the renderer, as a renderer message for 30 s
  and in the console (`MessageIsNew`, `ResetMessage`).

**OpenGL** (`GlfwRenderer::RenderScopes`) converts the primitives to the modelview coordinates of the overlay and
draws them after it, without depth test and before the image is saved - in saved images and animations, under the
icon bar. **The raytracer** (`Raytracer::DrawScopes`) draws them into its image after the texts, before the image is
scaled down: rectangles blended per pixel, lines as static lines with their width, texts with `DrawString`; a line
shorter than a pixel is drawn one pixel long.

## 3. Static graphics in the coordinates of the graph

`SC.renderer.scopes` (`MainRendererScopes`, Python `RendererScopes`): `SetGraphics(items)`, `AddGraphics(item or
items)`, `GetGraphics()`, `ClearGraphics(view=-1, scope=-1)`. One list of dicts, each with `type` `'Line'` (a
polyline of at least two points, with a `name` also in the legend), `'Text'` or `'Point'` (a circle of 16 lines with
`radius` pixels), and `view`, `scope`, `color`, `front`; every key is checked in C++ (`ScopeStaticItem`), and a list
is checked as a whole before it replaces the stored one.

The items are stored in `VisualizationSystemContainer::scopeStaticGraphics` (`ScopePlot::StaticGraphics`) under an
atomic flag; `BuildScopes` copies those of its view under it, and `AddScope` draws the items with `front` False after
the grid, then the sensor lines, then the items with `front` True, clipped to the plot area - a text only if its
position is inside. They do not enter the automatic ranges; they are not settings, and stay until `ClearGraphics()`
or `SC.Reset()`.

## 4. The locks

- **The stored values of a sensor** (`CSensor::LockInternalStorage`, `ReleaseInternalStorage`): an atomic flag per
  sensor, taken by the solver while it resets or appends rows (a growing matrix reallocates), by the traces and the
  scopes while they read them, and by `MainSensor::GetInternalStorage` while it copies them for Python.
- **The items of a system** (`PostProcessData::accessSystem` with its owner thread, `SystemAccessGuard`, #3239): taken
  by the main thread in `MainSystem::Reset`, the add, delete and modify functions of the five kinds of items,
  `Assemble` and `Set...Parameter` of visualization parameters (names starting with `V`; the others are values in
  place that a pre-step function may set every step); by the render thread in `VisualizationSystem::
  UpdateGraphicsData`, the sensor traces and `BuildScopes` (all systems of the container). A nested lock in the same
  thread passes. While the main thread waits for it, it computes the user function drawings that the render thread
  requests while holding it. Adding systems to a SystemContainer while the renderer runs is not covered;
  `SC.Reset()` detaches the renderer and joins its thread before it deletes the systems.

## 5. Tests

`scopeTest.py` checks the settings, the automatic mode, a sensor without `storeInternal`, clipping by `minMaxY`, a
line over another sensor and the static graphics on the raytracer image, which `RedrawAndGetImage` returns without a
window; `scopeExample.py` shows them with a double pendulum; `rendererItemChangesTest.py` exercises the lock of the
system.
