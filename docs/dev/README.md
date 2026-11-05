# Exudyn developer documentation

Developer-facing notes. Users start at [`README.rst`](https://github.com/jgerstmayr/EXUDYN/blob/master/README.rst) and
[`INSTALL.txt`](../../INSTALL.txt) in the repository root, or at the
[online documentation](https://exudyn.readthedocs.io/).

| document | contents |
|---|---|
| [GETTING_STARTED.md](GETTING_STARTED.md) | from nothing to a build and a passing test run: clone, environment, `setupLocalWorkspace.py`; with [GIT.md](GIT.md), branches, commit messages and what a contribution provides |
| [BUILD.md](BUILD.md) | the build per platform, the editors and the driver `exudev`, and what to do when the build does not work |
| [ARCHITECTURE.md](ARCHITECTURE.md) | the C++ internals: items, the C/Main/Visualization split, the object factory, modules, solvers |
| [CODING_STYLE.md](CODING_STYLE.md) | naming, abbreviations, file headers, how to report an error from C++ (§10), what is deprecated |
| [NEW_ITEM.md](NEW_ITEM.md) | how to set up a new item: the definition, what is generated, the C++, what to run |
| [DEVELOPMENT.md](DEVELOPMENT.md) | the development and release workflow: the repository layout, [WORKFLOW.md](WORKFLOW.md) (issue tracker, versioning, the four commit gates), [RELEASE.md](RELEASE.md) (a release on GitHub, PyPI and the documentation) and [GUI_MANUAL_CHECK.md](GUI_MANUAL_CHECK.md) (the manual check of the render window and the dialogs before a release) |
| [../../definitions/README.md](../../definitions/README.md) | the item and structure definitions as Python, and the function declaration library |
| [../../tools/generators/README.md](../../tools/generators/README.md) | the generator code and how to re-emit the definitions |
| [../../tools/exudev/README.md](../../tools/exudev/README.md) | the commands of the driver `exudev` |
| [../../CONTRIBUTING.md](../../CONTRIBUTING.md) | how to contribute from outside |
| [internals/README.md](internals/README.md) | how parts of Exudyn work inside: exudyn.symbolic, the symbolic system, the user functions and user items, the system utilities, the symmetric basis, the system Jacobians, the scopes of the render window |
| `docs/dev/design/` | the design notes of the steps that are open - analyses, options, decisions, measurements, written while planning a step; not published. When the step closes, what it built goes into the manual and into internals/, and the note leaves the repository (WORKFLOW.md §8.4) |
| [exudynRevisionInfo2026.md](https://github.com/jgerstmayr/EXUDYN/blob/master/docs/revision/exudynRevisionInfo2026.md) | general information on the v1.11.0 → v2.0 restructuring: rules, facts, decisions |
| [exudynRevisionPlan2026c.md](https://github.com/jgerstmayr/EXUDYN/blob/master/docs/revision/exudynRevisionPlan2026c.md) | **the plan that is open**, by group; checked and partly written by `docs/revision/planManager.py` |
| [exudynRevisionPlan2026b.md](https://github.com/jgerstmayr/EXUDYN/blob/master/docs/revision/exudynRevisionPlan2026b.md) | the second plan, closed 2026-10-10 - one line per step |
| [exudynRevisionPlan2026.md](https://github.com/jgerstmayr/EXUDYN/blob/master/docs/revision/exudynRevisionPlan2026.md) | the finished plan of the 2026 revision, phases R0 ... R11 — a record; the revision completed as 1.12 |
| [exudynRevisionLog2026c.md](https://github.com/jgerstmayr/EXUDYN/blob/master/docs/revision/exudynRevisionLog2026c.md) / [exudynRevisionLog2026b.md](https://github.com/jgerstmayr/EXUDYN/blob/master/docs/revision/exudynRevisionLog2026b.md) / [exudynRevisionLog2026.md](https://github.com/jgerstmayr/EXUDYN/blob/master/docs/revision/exudynRevisionLog2026.md) | what each closed step actually turned out to be — the first is where the work of revision2026c is written up, the other two are closed with their plans |
| [CLAUDE.md](https://github.com/jgerstmayr/EXUDYN/blob/master/CLAUDE.md) | the working contract for Claude Code sessions |

The how-to notes - the conda environments, building the documentation, and under *Misc* the build quirks, what MSVC
accepts and GCC does not, Visual Studio 2022, ffmpeg and matplotlib - follow this part of the documentation.

```{toctree}
:hidden:
:maxdepth: 2

/docs/dev/GETTING_STARTED
/docs/dev/BUILD
/docs/dev/ARCHITECTURE
/docs/dev/CODING_STYLE
/docs/dev/NEW_ITEM
/docs/dev/DEVELOPMENT
/docs/generated/deprecations
/definitions/README
/tools/generators/README
/tools/exudev/README
/CONTRIBUTING
/docs/dev/internals/README
```

## Invariants

These are differentiators, not technical debt. Every restructuring step preserves them
(plan §7):

- Mixed Python/native debugging in VS2022 - the capability, not which editor is primary;
  the everyday work happens in VS Code.
- The setuptools-based fast build.
- No heavy C++ dependencies; vendored headers.
- The hard C++/Python split with dict-based validated interfaces.
- Runtime dependencies stay at numpy only. Development-only tools are unconstrained.
