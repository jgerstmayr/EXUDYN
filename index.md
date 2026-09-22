# Exudyn documentation

**Exudyn** is a flexible multibody dynamics systems simulation code with Python and C++:
a C++ computational core exposed to Python, built for efficient simulation of rigid and flexible
multibody systems, and for automated model setup and parameter variation from Python.

This page is the table of contents, and it is **hand-written** — the only page of the
documentation that says what comes in which order. Everything under `docs/generated/` is written
by the emitters in `tools/generators/` and by the issue tracker; everything under `docs/manual/`,
`docs/dev/` and `docs/howTo/` is written by people.

New here? Read {ref}`Getting started <sec-installation-gettingstarted>`, then the
[Tutorial](docs/manual/tutorial.md). What changed between releases is in {ref}`Revisions <sec-revisions>`, every resolved
issue in the [changelog](CHANGELOG.md), and the issues themselves in the
{ref}`issue tracker <sec-issuetracker>`.

Searching on Read the Docs: add `*` or `~1` / `~2` to a term to search more generally, for example
`FEMinter*` for `FEMinterface`, or `objectffrf~3` to find `ObjectFFRF`. The search preview finds
fewer results than the search itself.

```{toctree}
:maxdepth: 3
:caption: Exudyn User Manual

README
docs/manual/gettingStarted
docs/manual/introduction
docs/manual/introductionBasics
docs/manual/introductionAdvanced
docs/manual/tutorial
docs/manual/GUI
docs/manual/commandLine
docs/manual/resultsMonitor
docs/manual/notation
docs/manual/theory
docs/manual/solver
docs/generated/cInterface/cInterfaceIndex
```

```{toctree}
:caption: Reference Manual

docs/generated/items/itemsIndex
docs/generated/pythonUtilities/utilitiesIndex
docs/generated/structures/structuresIndex
docs/generated/references
```

```{toctree}
:caption: Developer documentation

docs/dev/README
docs/dev/ARCHITECTURE
docs/dev/CODING_STYLE
docs/dev/WORKFLOW
definitions/README
tools/generators/README
tools/exudev/README
CONTRIBUTING
```

```{toctree}
:caption: How-to notes

docs/howTo/buildFromSource
docs/howTo/condaEnvironments
docs/howTo/sphinxDocs
```

Five further notes are written for whoever maintains Exudyn rather than for whoever uses it — the
Windows build quirks, what MSVC accepts where GCC does not, the Visual Studio 2022 components,
converting the demo videos with ffmpeg and the matplotlib recipes of the examples. They are in
the repository, in
[`docs/howTo/`](https://github.com/jgerstmayr/EXUDYN/tree/master/docs/howTo), and are listed in
the [developer documentation](docs/dev/README.md).

```{toctree}
:caption: Examples, models and issues

docs/generated/examples/examplesIndex
docs/generated/testModels/testModelsIndex
docs/generated/abbreviations
docs/manual/revisions
CHANGELOG
docs/generated/trackerlog
```

## Indices and tables

- {ref}`genindex`
- {ref}`search`
