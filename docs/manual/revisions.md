(sec-revisions)=
# Revisions

What changed between one release of Exudyn and the next, in prose, for somebody who has models
written against the previous one. Every single resolved issue is in the
[changelog](../../CHANGELOG.md), and the issues themselves are in the
{ref}`issue tracker <sec-issuetracker>`; this chapter is the short version, and a new section is
added on top of it at each release.

(sec-revisions-2-0)=
## Version 2.0

A revision of the whole project rather than a feature release: the layout of the repository, the
build, the tests, the documentation and the development tools. Most of it is invisible from a
model script. The parts that are not are first.

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
the definitions and the docstrings. There is no PDF any more: the same content is the HTML
documentation.

**`CHANGELOG.md`** is new: every resolved issue of every release, newest first. The version number
says where an issue landed — the micro version *is* the count of issues closed in a release, so
1.10.160 is the 160th issue closed in release 1.10.

### What changed behind the scenes

The wheels for Windows, Linux and macOS are built by continuous integration on every tagged
release, and the documentation is built the same way. Everything else a maintainer does — build,
test, documentation, issues, release — runs through one driver, `exudev`, which replaced sixteen
batch files. None of that changes anything for a user of a wheel; it changes how quickly a fix
reaches one.
