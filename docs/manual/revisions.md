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

**Star imports export only what a module defines.** `from exudyn.utilities import *` used to drag
in everything that module had imported itself — `np`, `sin`, `cos`, `sqrt`, `graphics` and more.
Every module now declares `__all__`, so a script that relied on those names arriving *through*
Exudyn raises `NameError`. The fix is one line at the top of the script:

```python
import numpy as np
from math import sin, cos, sqrt
import exudyn.graphics as graphics
```

**Names that are gone.** Eleven small vector helpers were removed from `exudyn.basicUtilities`
(`NormL2`, `VSum`, `VAdd`, `VSub`, `VMult`, `ScalarMult`, `Vec2Tilde`, `Tilde2Vec`,
`DiagonalMatrix`, `eye2D`, `eye3D`) — numpy does all of them, faster and in one call. The 23
deprecated `GraphicsData...` aliases in `exudyn.utilities` are gone as well; the current names are
in `exudyn.graphics`.

**Functions that moved inside the package**: from `exudyn.utilities` into `basicUtilities`,
`advancedUtilities` and `mainSystemExtensions`. A script that imports from `exudyn.utilities` in
the usual way notices nothing.

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
that passed it says so instead of being quietly ignored. Reading it — and `pContact`, the current
potential contact point — works as before, with `mbs.GetObjectParameter(objectNumber, 'pContact')`.

**OpenVR is removed.** The `--openvr` build flag, the settings under
`visualizationSettings.interactive.openVR`, the `openVR` entry of the render state and the
example `openVRengine.py` are gone. It could only be used with a head mounted display or an
emulator, it was never part of a released wheel - it had to be compiled in - and it stood in the
way of the coming rendering work. A script that needs it stays on Exudyn 1.11.

### What is new to use

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

**`exudyn.types`** answers questions about items from Python: which markers an object accepts,
which item types exist.

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
