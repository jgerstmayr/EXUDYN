(sec-pdfindex)=
# Exudyn

```{image} /docs/figures/ExudynLOGO1.9.jpg
:width: 300px
:align: center
```

**A flexible multibody dynamics systems simulation code with Python and C++**

**University of Innsbruck**, Department of Mechatronics, Innsbruck, Austria

**Since version 1.11.0, Exudyn is heavily developed with Anthropic's Claude Code**: code,
workflows, documentation, tests and examples.

Exudyn is a C++ computational core exposed to Python: rigid and flexible multibody systems,
solved efficiently, with the model built and varied from a Python script. It is free and open
source, pre-built for Python 3.10 – 3.14 under Windows, Linux and macOS, and it links to whatever
else the script needs — numpy and scipy, NGsolve, the Robotics Toolbox, the reinforcement learning
packages.

**How to cite:** Johannes Gerstmayr. *Exudyn – A C++ based Python package for flexible multibody
systems.* Multibody System Dynamics, Vol. 60, pp. 533–561, 2024.
[doi:10.1007/s11044-023-09937-1](https://doi.org/10.1007/s11044-023-09937-1)

For the license see `LICENSE.txt` in the repository.

**About this document.**
This is the documentation of the release named on the title page, as one file: a fixed,
citable version of what is otherwise the HTML documentation. Both are built from the same
sources, and **the HTML is the living one** — it is rebuilt continuously and it is what a link
into the documentation points at:

- **Read the Docs** — <https://exudyn.readthedocs.io/>
- **GitHub** — <https://github.com/jgerstmayr/EXUDYN>
- **PyPI** — <https://pypi.org/project/exudyn/>
- **YouTube** — the [tutorial videos](https://www.youtube.com/playlist?list=PLZduTa9mdcmOh5KVUqatD9GzVg_jtl6fx)

What this document holds, and the HTML does too: the user manual, the reference manual of every
item, the Python utility functions, the settings structures, the Python–C++ interface, the
developer documentation, and — at the end, and deliberately — the **complete issue history**, which
is where the reason for most changes is written down.

What it does **not** hold is the source text of the examples and the test models. That is a large
body of Python which belongs next to an editor rather than in a book; it is in the HTML
documentation, and it is in the repository under `python/Examples` and `python/TestModels`.

```{note}
Exudyn is an open source library developed largely in free time. Some models are simplifications
that suit the needs they were written for, some are under development — which the documentation
says where it is the case — and some have bugs. Do not rely on any part of it blindly.
```

```{toctree}
:maxdepth: 3
:caption: Exudyn User Manual

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
```

```{toctree}
:caption: How-to notes

docs/howTo/buildFromSource
docs/howTo/condaEnvironments
docs/howTo/sphinxDocs
```

```{toctree}
:caption: Revisions and issues

docs/generated/abbreviations
docs/manual/revisions
CHANGELOG
docs/generated/trackerlog
```
