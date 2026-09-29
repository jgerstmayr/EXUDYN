(sec-dev-guimanualcheck)=
# Manual GUI check before a release

The tests check what the renderer draws (graphics data per item, low-resolution raytracer images,
#2704). **Nothing automatic checks that a person can use the render window and the dialogs**: keys,
mouse, tkinter windows, focus, fonts, window placement. That is this list.

- **Once per release on each of Windows, Ubuntu and macOS**; one person, about one hour.
- Do only what is written; if something looks wrong, note it and go on.
- Not checked here: what each setting draws (automatic). Checked here: that the **key or the dialog
  reaches the renderer** - one or two settings per path.

## 0. Preparation (5 min)

1. Install the release candidate wheel into a clean environment (`pip install exudyn-<version>-...whl`).
2. Run `python -m exudyn info` and paste the output into the report.
3. **Back up `~/.exudyn/config.json`** if it exists, then delete it, so that you test the defaults.
   Some checks below write this file; restore your backup at the end.
4. Copy `python/testing/guiManualCheckModel.py` and `python/Examples/nMassOscillatorEigenmodes.py`
   into an empty working directory.
5. `python -m exudyn demo` - the renderer opens, the demo runs, the window closes. If this fails,
   stop here and report it.

**Model used**: `guiManualCheckModel.py` - a mass point with spring-damper and force, and a 3D double
pendulum of two rigid bodies with revolute joints; it has at least one node, body, connector,
marker, load and sensor, and a sensor trace. It opens the renderer and **waits**; SPACE starts a
real-time simulation of 120 s, Q stops it, then SolutionViewer and PlotSensor follow.

Report each check as **OK / FAIL / n.a.** with the ID; for a FAIL, one line on what happened.

## 1. Render window: mouse and view keys (15 min)

`python guiManualCheckModel.py` - the window opens, the model is not moving yet.

| ID | action | expected |
|---|---|---|
| R1 | look at the window after start | model fully visible (zoom all), help hint shown for ~5 s, window title and status text readable (display scaling!) |
| R2 | left mouse: press, drag, release | model moves with the mouse in the screen plane |
| R3 | right mouse: press, drag, release | model rotates about the screen axes; release stops it |
| R4 | mouse wheel up/down; with CTRL | zoom in/out; CTRL gives small steps |
| R5 | left **click** (no drag) on the red mass; then on empty space | status line *Selected item: ...*; item highlighted for ~5 s; empty space: *no item selected* |
| R6 | right **click** (no drag) on a pendulum link | read-only dialog *properties of <...>* opens with the item dictionary; ESCAPE closes it; render window still reacts afterwards |
| R7 | keys `A`, `.` / `,` (and keypad `+` / `-` if present) | zoom all, zoom in, zoom out |
| R8 | cursor keys; CTRL+cursor; SHIFT+cursor; ALT+LEFT/RIGHT | move; small move; rotate about screen x/y; rotate about screen z |
| R9 | keypad 2/8, 4/6, 7/9 (if a keypad exists); same with CTRL | rotation about 1, 2, 3 axis; CTRL: small rotation |
| R10 | CTRL+1 ... CTRL+7, then SHIFT+CTRL+1 | the standard views (1-2 plane, 1-3, 2-3, ..., 3D view); SHIFT: the same plane from behind; a message names the view |
| R11 | pan with left mouse so that the pendulum joint is at the window center, press `O`, then rotate with right mouse | message *Set rotationCenterPoint ...*; the model now rotates about the new center |
| R12 | `R`, wait 3 s, `R` again | automatic rotation of the view starts and stops |
| R13 | F3, then left-click twice at two points (after CTRL+1) | mouse coordinates in the status line; second click shows `lastPos` and `dist`; F3 again switches off |
| R14 | CTRL+F3 | status line shows zoom / rotation / center; the console prints a `SC.renderer.SetModelView(...)` line |

## 2. Render window: item keys, simulation keys, raytracer, views (10 min)

| ID | action | expected |
|---|---|---|
| K1 | `N`, `B`, `C`, `M`, `L`, `S` - each pressed twice | nodes / bodies / connectors / markers / loads / sensors disappear and reappear; a message says *show ...: off/on* |
| K2 | CTRL+`N`, CTRL+`B`, CTRL+`C`, CTRL+`M`, CTRL+`L`, CTRL+`S` | the numbers of that item kind appear/disappear; text readable, not hidden behind the bodies |
| K3 | `T` pressed repeatedly (6-7 times) | cycles through transparent faces / face edges only / faces with edges / ... and back to the start; message states the mode |
| K4 | **SPACE** | simulation starts in real time; the sensor trace of the pendulum tip is drawn |
| K5 | SPACE again, wait, SPACE | simulation pauses and continues (message *switch pause on/off*) |
| K6 | during simulation: `1`, `5`, `2` | update interval 20 ms (smooth), 100 s (frozen), 100 ms (default) - message each time |
| K7 | CTRL+R, rotate a little, CTRL+R | raytraced image (shadows, slower update), then back to OpenGL |
| K8 | **CTRL+V** | a second window (view 1) opens and shows the running model; mouse and keys work in it independently; CTRL+R there only affects that window |
| K9 | in the view 1 window: `Q` | only view 1 closes, the simulation continues in the main window |
| K10 | F2, then `N`, then F2, then `N` | first `N` ignored (message *ignore keys mode switched on*), after F2 again `N` works |
| K11 | `Q` | simulation stops; console prints *simulation finished ...*; the window stays and can still be rotated |

## 3. Dialogs from the render window (15 min)

Still in phase 3 of the model (after K11), or start the model again.

| ID | action | expected |
|---|---|---|
| D1 | `H` | help window with mouse and key table; text readable, scrollable, ESCAPE closes it |
| D2 | `X`, type `print(mbs)`, CTRL+RETURN | command window opens (may be **behind** the render window - note it); the console shows the system summary |
| D3 | in the command window: `x = 42`, CTRL+RETURN; then `print(x)`, CTRL+RETURN; ESCAPE | prints 42 (assignments survive); window closes |
| D4 | `X` then `1/0`, CTRL+RETURN | error printed in the console, the dialog and the renderer survive |
| V1 | `V` | visualization settings dialog: tree with columns Name / Value / Type / Description, readable font, sensible column widths; render window blocked or updated while it is open (both are acceptable, note which) |
| V2 | open `nodes`, click the value of `show`, choose `False` | nodes disappear **in the render window** immediately; the row is marked as changed |
| V3 | double-click the same `nodes.show` value | toggles back to `True`; nodes reappear |
| V4 | `nodes.defaultSize`: type `0.2`, RETURN | nodes grow in the render window |
| V5 | `general.sphereTiling`: type `4`, RETURN | spheres (nodes) become visibly coarse; set back with **undo** |
| V6 | `connectors.showJointAxes` = `True`; `bodies.showNumbers` = `True` | joint axes drawn; body numbers shown |
| V7 | enter an invalid value, e.g. `abc` into `nodes.defaultSize`; then `-1` into `general.circleTiling` | value rejected with a message, old value kept, dialog survives |
| V8 | enum: `contour.outputVariable` - pick `Displacement` from the list; `contour.outputVariableComponent` = `1` | combo box shows short names (no `OutputVariableType.` prefix); bodies get a contour color and a color bar appears; set back to `_None` |
| V9 | hover over a row and over a folder (wait 0.5 s) | tooltip with the description (folder: its class description); tooltip is **in front** of the dialog |
| V10 | CTRL+F, type `trace`; RETURN; F3 | hits drop-down enabled; RETURN jumps to the first hit, F3 to the next |
| V11 | select a row, press **copy line**, paste into an editor | e.g. `SC.visualizationSettings.nodes.defaultSize = 0.2` |
| V12 | **diff to default**, **changes since start** | each opens a window **in front of** the dialog, listing exactly the changes made so far as code lines |
| V13 | **undo** (twice), **revert**, **reset** | undo steps back whole states; revert returns to the state when the dialog opened; reset to defaults - render window follows each |
| V14 | change `general.backgroundColor` to `[0.9,0.9,1,1]`, press **store settings**, confirm | list shows the change; console says *stored 1 setting(s) in ...config.json* |
| V15 | move/resize the dialog, press **store positions**, confirm; close with ESCAPE; `V` again | the dialog reopens at the stored position and size |
| V16 | CTRL+mouse wheel over the tree | font size changes, row height and columns follow |
| V17 | close the dialog with **close**, then `Q` / ESCAPE to leave phase 3 | render window closes; SolutionViewer (next section) starts |

(After V14: the next start of any model uses the stored background color - confirmed at S1. Delete
the file again at the end.)

## 4. SolutionViewer and PlotSensor (7 min)

The model continues with `mbs.SolutionViewer()` automatically.

| ID | action | expected |
|---|---|---|
| S1 | look at the renderer and the dialog | renderer reopens **with the previous view** and the background color stored in V14; dialog with sliders *Solution steps*, *Increment*, *update period*, run modes, record, *Make mp4* |
| S2 | let it run; then **Static**; drag the *Solution steps* slider | animation plays; in static mode the model follows the slider |
| S3 | *Increment* to 10; *update period* to max | animation faster (skips frames) / slower |
| S4 | **One cycle**, **Run** | plays once and stops |
| S5 | keys in the render window during the viewer (`B`, SPACE, `V`) | still work; `V` opens the settings dialog |
| S6 | *Record frames*, run one cycle, *No recording* | images appear in the `images` subfolder; (*Make mp4* only if ffmpeg is installed - optional) |
| S7 | close the dialog (window close button, or `q` / ESCAPE in the dialog) | dialog and renderer close, script continues |
| P1 | PlotSensor windows | two matplotlib figures: tip position x/y/z with legend and axis labels, and the spring force; windows can be zoomed, panned and closed; the script ends after closing them |

## 5. AnimateModes (5 min)

`python nMassOscillatorEigenmodes.py`

| ID | action | expected |
|---|---|---|
| A1 | dialog and renderer open | dialog with *Mode shape*, *Contour plot*, *Amplitude* (+ positive/negative), *update period*, run modes, mesh/faces, recording; renderer shows the masses |
| A2 | **Run**; move the *Mode shape* slider; move *Amplitude*; switch *negative* | mode animates; the mode changes; amplitude grows/shrinks; motion inverts |
| A3 | *Contour plot* = *Displacement*; *Static continuous* | masses colored with a color bar; the deformed shape stands still |
| A4 | close the dialog with `q` in the dialog | dialog and renderer close, script ends without error or hang |

## 6. Quitting (3 min)

| ID | action | expected |
|---|---|---|
| Q1 | `python guiManualCheckModel.py quitdialog`, SPACE, wait > 10 s, **ESCAPE** | dialog *WARNING - long running simulation!* in front; **No** keeps running; ESCAPE again, **Yes** stops the solver and closes the window; the script continues (SolutionViewer) or ends without hanging |
| Q2 | in the SolutionViewer of the same run: close the render window with the window close button | viewer stops, no hang, no crash of the Python process |

## 7. Optional, if time is left

| ID | action | expected |
|---|---|---|
| O1 | `python -m exudyn dialogs vis`, `... sim`, `... help` | the dialogs without a model; closing *vis* prints the changed settings as code |
| O2 | 3D mouse / 6-axis joystick (3Dconnexion) connected before the renderer starts | message *found 6-axis joystick ...*; the device moves and rotates the view (`interactive.useJoystickInput`, on by default) |
| O3 | `python/Examples/mouseInteractionExample.py` | drag the chain with the mouse; F2 / key user function works |
| O4 | `python -m exudyn monitor --last` while a model writes sensor files | results monitor opens and updates |
| O5 | display scaling 150-200 % (Windows) / HiDPI (Ubuntu, macOS Retina) | texts in the render window and the dialogs readable and not blurred; adjust `dialogs.fontScaling` in V |

## 8. Finish

1. Delete `~/.exudyn/config.json` and restore your backup.
2. Send the report: `python -m exudyn info` output, platform (OS version, X11/Wayland on Ubuntu,
   Intel/ARM on macOS, display scaling), the list of IDs with OK / FAIL / n.a., and screenshots of
   anything that looked wrong.

## Platform notes

- **macOS** always renders single-threaded: dialogs run inside the renderer's idle loop, so the
  render window does **not** update while a dialog is open - that is expected. There is no keypad on
  most keyboards: R9 is *n.a.*, use SHIFT+cursor (R8). CTRL means the **control** key, not CMD.
- **Ubuntu**: note whether the session is X11 or Wayland. tkinter dialogs may open behind the render
  window (D2, V1) - note it, it is known; a dialog that cannot be brought to front at all is a FAIL.
- **Windows**: check at least once with display scaling > 100 % (O5), because DPI awareness is set
  differently with and without the render window (#2634).
