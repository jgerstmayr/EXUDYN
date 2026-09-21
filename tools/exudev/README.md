# `exudev` — the Exudyn maintainer driver

One command for building, testing and documenting the repository. It replaces the sixteen batch
files that used to live in `tools/buildAndGenerate/` (issue #2503, revision2026 step R5.18).

```
exudev --help               the commands
exudev <command> --help     the options of one command
exudev -n <command>         print the commands it would run, and run nothing
```

On Windows type `exudev` from anywhere in the tree (`exudev.bat` sits in the repository root);
everywhere else — WSL, linux, macOS — type `python tools/exudev`.

**Quiet is the default**; `-v/--verbose` turns the tools' own output back on.

## Commands

| command | what it runs |
|---|---|
| `exudev generate [--check] [--no-run] [--all-checks]` | `tools/regenerate.py`; with `--all-checks` also `checkAll`, `checkExtras`, `checkPython`, `checkPython --stubs` and `gen_sources`, each with `--check` |
| `exudev build [--py P313] [--fast] [--complete]` | `pip wheel . -w dist --no-deps`, then installs exactly that wheel. **No clean, no regeneration, no docs, no tests** unless `--complete` |
| `exudev test [--py] [--fast] [--parallel [N]]` | `runTestSuite.py`, always with `--exit-code` |
| `exudev examples [--py] [--timeout S]` | `runTestExamples.py` (slow; default `venvP312`) |
| `exudev perf [--py] [--fast]` | `runPerformanceTests.py` |
| `exudev docs [--keep-cache] [--open]` | `sphinx-build -b html . _build -E` |
| `exudev linux [--manylinux \| --wsl-conda]` | the linux wheels through WSL; manylinux in docker is the release path |
| `exudev release [--dev] [--no-linux]` | `build --complete` over every version, with the guards a release needs |
| `exudev clean [--dist] [--linux] [--all]` | the build directories and eggs |
| `exudev env [--py]` | python, exudyn, numpy, scipy and matplotlib per environment |
| `exudev issue <verb>` | the issue tracker: `raise`, `extend`, `remark`, `resolve`, `close`, `show`, `list`, `modify`, `triage`, `serve`, `mode`. Runs in the current interpreter, without conda; `resolve` and `abandon` rewrite the version files (revision2026 step R8.3) |
| `exudev issue bump` | start a new release: `--minor` (1.11 → 1.12), `--major` (1.11 → 2.0) or `--to <version>`; appends it to `releases.json` and restarts the micro version (revision2026 step R8.4) |
| `exudev issue serve` | the same tracker as a local web page, for a backlog pass: list, search, filter, and raise/extend/remark/resolve through the same API (revision2026 step R8.5.1) |

## The three things worth knowing

**`-n/--dry-run` is the documentation.** It prints the real command lines — the conda call, the
environment variables it overrides, the working directory — and runs nothing. When you want to know
what a release does, `exudev -n release` answers in one screen and cannot be out of date, because
the printed line and the executed line are built by the same code.

**`--fast` is opt-in, and it turns the repository default OFF.** `pyproject.toml` has
`compileExudynFast = true`, so a plain `pip wheel .` builds `exudynCPPfast` as well; `exudev build`
sets `EXUDYN_COMPILE_EXUDYN_FAST=0` unless you ask for `--fast`. Two silent gates remain, and the
driver warns about both: on a `.dev` version `setup.py` builds the fast module for **Python 3.13
only**, and on macOS not at all.

**`exudev env` is the first thing to run when a test fails "only here".** A stale exudyn install or
a different numpy changes results — the two failing models of issues #2501 and #2502 differ between
`venvP313` (numpy 2.2.4) and `venvExuP313` (numpy 2.4.6) with a byte-identical binary.

## How the environment is chosen

`--py 313` / `P313` / `3.13` / `all` / `310,313` selects the `venvP3xx` environments;
`--env NAME` names one directly (not for `build`, which needs to know which `cp3xx` wheel belongs
to it). Generation and documentation default to `venvExuP313`.

Dispatch is `conda run -n <env> --no-capture-output …` — one process, the exit code propagated, no
activate/deactivate dance and no way to leave the shell in the wrong environment. The conda base is
found through `EXUDYN_CONDA_ROOT`, then `CONDA_EXE`, then `PATH`; `--no-conda` runs everything in the
current environment instead, which is what you want inside WSL.

## Passing something through

`test`, `examples` and `perf` forward everything after `--` to the runner, and say so:

```
exudev test --py P313 -- --timeout=5
```

Nothing is forwarded implicitly. The runners scan `sys.argv` by hand and only *print* a complaint
about an option they do not know, so a silently forwarded typo would run a full suite against the
wrong thing and report success.

## How a step is judged

Every step is judged by its **exit code**; the driver adds `--exit-code` to all three runners and
offers no way to turn that off. Until revision2026 step R5.18.1 that was not possible:
`runTestExamples.py` and `runPerformanceTests.py` always returned 0, and the driver had to read the
summary line out of the log they had just written. That guesswork is gone (#2504).

Two of the exit codes mean "nothing NEW broke", not "everything passed":

- `runTestSuite.py` excludes `SensitiveTests()` and, on Linux, `UnresolvedOnLinux()`;
- `runTestExamples.py` excludes `testRunnerTools.KnownExampleFailures()` - five examples today, three
  of them missing an optional package (#2507). A known failure that starts passing is reported as a
  **dead exclusion** by name, so the list shrinks rather than rots.

`runPerformanceTests.py` has no exclusions: every performance test passes, and one that does not is
worth going red for.

One thing the driver still cannot do: attribute a run if two `exudev` processes overlap. Don't run
two at once.

## The files

| file | what is in it |
|---|---|
| `__main__.py` | every option, every spelling, every help text; dispatch |
| `commands.py` | one function per subcommand — **read this to find out what a command does**. It builds `Step` objects and runs nothing |
| `runner.py` | the only file that knows about processes, conda and the file system; executes or prints the steps |
| `probe.py` | runs *inside* an environment and reports what is installed; the only part that imports exudyn |

Nothing except `probe.py` imports exudyn — the driver selects the environment, so it must run in any
interpreter. Standard library only, Python 3.8 syntax.

`manylinuxBuild.sh` and `buildManylinux.sh` stay in `tools/ci/`: they run inside the manylinux docker
image and have to be shell.
