(sec-usersettings)=
# User settings that persist between runs

A setting you change on every start — the output directory, the multi-sampling of the renderer, the
size of the basis vectors — can be stored once and taken by every run afterwards. Exudyn reads one
file for that, `~/.exudyn/config.json`:

```json
{
  "config": {"outputDirectory": "solution/"},
  "visualizationSettings": {"openGL.multiSampling": 4, "nodes.basisSize": 0.5}
}
```

Nothing writes this file by itself. A script that behaves differently on another machine, because
something was stored there, is the one thing a settings file must not cause, so storing is always
asked for:

```python
from exudyn.misc import overrideSettings

SC.visualizationSettings.openGL.multiSampling = 4
overrideSettings.Store(SC)         #every run from now on starts with it
```

`Store(SC)` writes the settings that differ from the defaults — the same list the settings dialog
shows as *changed* — and `Store(config=exudyn.config)` does the same for `exudyn.config`.
`overrideSettings.Clear()` deletes the file.

## What happens when the file is there

`import exudyn` reads it once and prints **one note** naming how many settings it took:

```
NOTE: 1 setting(s) from %USERPROFILE%\.exudyn\config.json, and 2 visualizationSettings for every
      SystemContainer (exudyn.misc.overrideSettings.Print() for the list;
      EXUDYN_NO_USER_SETTINGS=1 to ignore them)
```

What it read is in **`exudyn.special.overrideSettings`**, a dictionary with one key per section,
which both a script and the C++ core read — the file is opened once, by the import, and not again.

The `config` settings are applied at that moment. The `visualizationSettings` are applied to every
`SystemContainer` when it is created, because that is when they begin to exist.
`overrideSettings.Print()` lists them:

```
override settings file: %USERPROFILE%\.exudyn\config.json
  applied: config.outputDirectory = 'solution/'
  applied: visualizationSettings.openGL.multiSampling = 4
```

which is the answer to *why does this script behave differently here* — and
`overrideSettings.Applied()` gives the same as a list, for a script that wants to print it into its
own output.

## What may be stored

**Plain values**: a number, a flag, a string, or a list of numbers. A setting that holds graphics
data, a user function or a matrix container is refused with a message and changes nothing — such a
value cannot be carried honestly by a JSON file.

A key that names no setting, a section nobody reads, a file that is not valid JSON: each of them is
reported and none of them stops `import exudyn`.

## Remembering a dialog

`visualizationSettings.dialogs.storeDialogPositions = True` makes a settings dialog remember where
it was left. It is stored under `"dialogs"`, one entry per dialog:

```json
{"dialogs": {"visualizationsettings": {"size": [1024, 768], "position": [100, 80]}}}
```

**The size comes back always; the position only when the window would still be reachable.** A
monitor that is unplugged, a laptop undocked, a screen resolution that changed: each of them would
otherwise put the dialog where nobody can reach its title bar, and a dialog that cannot be closed
is a stuck session. When the stored position is not usable, the dialog opens where it would have
opened anyway, at its remembered size.

## The columns and the font of a dialog

The three fixed columns of a settings dialog take a share of its width, each a fraction in
`visualizationSettings.dialogs`, and the description column takes what they leave:

```python
SC.visualizationSettings.dialogs.columnWidthName = 0.4      #a long name needs more
SC.visualizationSettings.dialogs.columnWidthValue = 0.18
SC.visualizationSettings.dialogs.columnWidthType = 0.09
```

They are fractions, so they do not depend on the screen; if the three together would leave the
description less than a tenth of the dialog, all three are scaled down to leave it that much.

**Ctrl and the mouse wheel change the font size** of an open dialog, about 10% per notch. The row
height, the columns and the font of a changed row follow it. The wheel alone still scrolls.

## Switching it off

`EXUDYN_NO_USER_SETTINGS=1` ignores the file completely, and it is what makes a problem
reproducible on a machine that has stored something: run the script with the variable set and the
difference is either gone (the setting caused it) or still there (it did not).

**The test suites set it for themselves.** `runTestSuite.py`, `runTestExamples.py`,
`runPerformanceTests.py` and `pytest` run every model from the defaults, whatever is stored on the
machine, so a stored setting can never move a test result.

`EXUDYN_CONFIG_FILE` names a different file, for a second configuration or for a test.
