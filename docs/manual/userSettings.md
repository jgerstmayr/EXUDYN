(sec-usersettings)=
# User settings that persist between runs
A setting that is changed on every start - the output directory, the multi-sampling of
the renderer, the size of the basis vectors - is stored once and taken by every run afterwards.
Exudyn reads one file for that, `~/.exudyn/config.json`:

```
{
  "version": 1,
  "config": {"outputDirectory": "solution/"},
  "visualizationSettings": {"openGL.multiSampling": 4, "nodes.basisSize": 0.5}
}
```

Nothing writes this file by itself. A script that behaves differently on another
machine, because something was stored there, is the one thing a settings file must not cause, so
storing is always asked for:

```python
from exudyn.misc import overrideSettings

SC.visualizationSettings.openGL.multiSampling = 4
overrideSettings.Store(SC)         #every run from now on starts with it
```

`Store(SC)` writes the settings that differ from the defaults - the same list
the settings dialog shows as *changed* - and `Store(config=exudyn.config)` does the same for
`exudyn.config`. `overrideSettings.Clear()` deletes the file.

**The `version` is the format of the file**, and it has to match: a file of another version is
ignored, with one note naming both. It is a plain integer that moves only when Exudyn has been
released *and* the meaning of something in this file has changed - not with the Exudyn version, which
moves on every resolved issue. Store your settings again to write a current file.

## What happens when the file is there

`import exudyn` reads it once into `exudyn.special.overrideSettings` and prints
**one note** naming how many settings it took:

```
NOTE: 1 config settings and 2 visualizationSettings read from ~/.exudyn/config.json
```

A count that is 0 is left out. `overrideSettings.Print()` lists what came from the file, which
is the answer to *why does this script behave differently here*, `overrideSettings.Applied()` gives
the same as a list, for a script that wants to print it into its own output, and
`EXUDYN_NO_USER_SETTINGS=1` runs a script as if there were no file.

**A file that should stay quiet** says so itself, beside its `version`:

```
{
  "version": 1,
  "suppressOverrideSettingsWarning": true,
  "visualizationSettings": {"openGL.multiSampling": 4}
}
```

The key does not exist unless it is written, and without it the note is printed. It suppresses the
note only: a setting that could not be applied is still reported, because that is a defect and not
information.

**What happens, in order:**

1. the whole file is read into `exudyn.special.overrideSettings`, once, by `import exudyn`;
2. what can be applied at once is applied at once, which is the `config` section;
3. the rest stays in the dictionary, because the things it sets do not exist yet;
4. every `VisualizationSettings` structure applies the stored settings **when it is created** - the
one a `SystemContainer` builds, and one built by `exu.VisualizationSettings()`. A setting that
names nothing is reported and changes nothing;
5. a dialog reads its size and position from the `dialogs` section when it opens, and the results
monitor its own settings from `resultsMonitor`;
6. nothing writes the file unless it is asked to.

**The file is read once**, so a file you edit while a session is running - or one that another
session stored - has no effect until you read it again. That is what makes a stored setting look as
if it had not been stored in a console that keeps its kernel, such as Spyder:

```python
from exudyn.misc import overrideSettings
overrideSettings.Reload()          #read it again and apply what can be applied
overrideSettings.Print()           #what came from it now
```

A reload does not **undo**: a setting that already reached `exudyn.config`, and a structure that
already exists, keep what they were given. A setting you removed from the file is seen by the
structures created after the reload; for everything else, start a new session.

A stored `visualizationSetting` is **not** a new default: `exu.VisualizationSettings()` carries it,
and the defaults the settings dialog compares against are still the defaults of Exudyn, so *diff to
default* shows a stored setting as a difference. That is the point - it is what the file changed.

**What may be stored are plain values**: a number, a flag, a string, or a list of numbers - and an
enum setting, `contour.outputVariable`, as the name of its value, `"StressLocal"`, which is read back
by that name; `interactive.highlightItemType` is the state of a highlight and is not stored. A setting
that holds graphics data, a user function or a matrix container is refused with a message and
changes nothing - such a value cannot be carried honestly by a JSON file. A key that names no
setting, a section nobody reads, a file that is not valid JSON: each of them is reported and none of
them stops `import exudyn`.

## Storing from the dialog

The settings dialog has a **store settings** button. It writes the `visualizationSettings` that
differ from the defaults, and the size and position of the dialog itself, and **nothing else** - not
`exudyn.config`, not the simulation settings - and it shows exactly what it is about to write before
it writes anything. It is the way to keep a look you have just made without switching
`storeDialogPositions` on.

**diff to default** stays a difference to the *default*, and a setting that the file already stores
is listed with the others and then named again under a comment line that says so. Comparing against
the defaults plus the file instead would hide exactly the settings the file is about.

## Placing a dialog, and a window, from a script

A script can say where a dialog opens, which is the same mechanism the dialogs use for themselves:

```python
from exudyn.misc import overrideSettings

#the visualization settings dialog, 1024x768 at (100, 80):
overrideSettings.StoreDialogGeometry('Visualization Settings', [1024, 768], [100, 80])
```

The name is the **title** of the dialog, and `overrideSettings.DialogKey(name)` is the key it is
stored under, so the same call places the solution viewer (`'Solution Viewer'`) or any other dialog.
The **size comes back always and the position only if the window would still be reachable** on the
screen you have now - a monitor that is gone must not put a dialog where its title bar cannot be
grabbed. This writes `~/.exudyn/config.json`, so it holds for every run afterwards.

The **render window** is not a dialog and is placed by its own settings, per view:

```python
SC.visualizationSettings.view0.window.renderWindowSize = [1024, 768]
SC.visualizationSettings.view0.window.renderWindowPosition = [100, 80]
```

A negative coordinate - the default - means the window manager places it. The position is that of the
OpenGL area rather than the title bar, so a small value hides part of the title bar and 0 hides it
completely, which is a way to have a view without one.

**The render window is the one window whose geometry lives in two places**: these settings, and -
because they are ordinary settings - the `visualizationSettings` section of the file. The file is
applied when the settings structure is created and a script speaks afterwards, so **what the script
sets wins**, and `SC.renderer.Start()` says so once when the two differ:

```
Python WARNING: the render window geometry stored in the settings file differs from what this session
set, and what the session set is used:
  view0.window.renderWindowPosition: the file says [100,80] and this session uses [500,400]
store the settings again to change the file, or remove them from it
```

It says nothing when they agree, which is the normal case for someone who stored the geometry and has
not touched it since. `view*.window.storeRenderWindowGeometry` writes where the window was back into
these settings when it closes, and `SC.renderer.GetState()['currentWindowPosition']` is where it is
while it is open.

## The plot windows of PlotSensor

A plot window is remembered by its **sequence**, the order `PlotSensor` made it in, because plot
windows have no title of their own: the first one of a run is stored as `'PlotSensor 1'`, the second
as `'PlotSensor 2'`, and `PlotSensor(..., closeAll=True)` starts that order over. They are stored in
the same `dialogs` section, with the same rules - the size comes back always, the position only if the
window would still be reachable - so the next run opens the plots where they were arranged.

Storing them is asked for, once the windows are where they should be:

```python
from exudyn.plot import StorePlotWindowGeometry

#after arranging the plot windows on the screen:
StorePlotWindowGeometry()          #returns how many windows it stored
```

This is the call to use when the plots are made after `SC.renderer.Stop()`, when the settings dialog
is gone; while it is open, its **store positions** button stores the plot windows together with the
other windows. `PlotSensorDefaults().storeWindowPositions
= True` in addition stores each window when it closes, one window at a time, which asks nothing but
also keeps whatever a window happened to be when it was closed.

A plot window is only placed by a backend that has one: with matplotlib on `Agg` - which
`EXUDYN_SUPPRESS_UI_WINDOW_OPEN` selects - there is no window, nothing is placed and nothing is
stored.

**For this run only**, a script can write into `exudyn.special.overrideSettings` instead of the file:

```python
exudyn.special.overrideSettings['dialogs'] = {
    'visualizationsettings': {'size': [1024, 768], 'position': [100, 80]}}
```

That is read by the next dialog that opens and the file is not touched. It is **not the recommended
way**: nothing checks what is put there, and a value of the wrong shape is simply not used - the
functions above are what say what they mean, and what a later Exudyn will keep working.

## Remembering a dialog, its columns and its font

`visualizationSettings.dialogs.storeDialogPositions = True` makes a settings dialog remember
where it was left, under `"dialogs"`, one entry per dialog. **The size comes back always; the
position only when the window would still be reachable.** A monitor that is unplugged, a laptop
undocked, a screen resolution that changed: each of them would otherwise put the dialog where nobody
can reach its title bar, and a dialog that cannot be closed is a stuck session. When the stored
position is not usable, the dialog opens where it would have opened anyway, at its remembered size.

The three fixed columns of a settings dialog take a share of its width, each a fraction in
`visualizationSettings.dialogs` - `columnWidthName`, `columnWidthValue`,
`columnWidthType` - and the description column takes what they leave; if the three together
would leave the description less than a tenth of the dialog, all three are scaled down to leave it
that much. **Ctrl and the mouse wheel change the font size** of an open dialog, about 10% per notch;
the wheel alone still scrolls.

The environment variables that change what Exudyn does before a script says anything are listed in
[](#sec-environmentvariables); `EXUDYN_NO_USER_SETTINGS` and `EXUDYN_CONFIG_FILE` are the two for this file.
The functions that read and write the file are in `exudyn.misc.overrideSettings`.
