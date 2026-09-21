<!-- hand-written; the only file in this directory that is -->
# docs/generated

**Everything else in this directory is generated. Do not edit it.**

The pages here are written by the emitters in [`tools/generators/`](../../tools/generators/) and
by [`tools/issueTracker/issueTracker.py`](../../tools/issueTracker/issueTracker.py), from
`definitions/`, from the docstrings of the Python package, from `python/Examples/` and
`python/TestModels/`, and from the issue tracker's own log. Each file says so in its first line.
A hand edit here survives until the next `python tools/regenerate.py` and no longer.

| directory | written by | from |
|---|---|---|
| `items/` | `itemDocsEmitter.py` | `definitions/itemDefs*.py` |
| `structures/` | `structureDocsEmitter.py` | `definitions/structure*.py` |
| `cInterface/` | `pybindEmitter.py`, `mainSystemExtensionDocsEmitter.py` | `definitions/pybind*.py`, the `@extends` functions |
| `pythonUtilities/` | `utilityDocsEmitter.py` | the docstrings of `python/exudyn/` |
| `examples/`, `testModels/`, `abbreviations.md` | `examplesDocsEmitter.py` | `python/Examples/`, `python/TestModels/` |
| `trackerlog.md` | `issueTracker.py` | `tools/issueTracker/trackerlog.txt` |

To change a page, change what it is generated from. The hand-written documentation is in
[`docs/manual/`](../manual/), [`docs/dev/`](../dev/) and [`docs/howTo/`](../howTo/); the table of
contents is the hand-written `index.md` at the repository root.
