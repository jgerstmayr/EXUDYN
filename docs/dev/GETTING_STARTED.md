(sec-dev-gettingstarted)=
# Getting started as a developer

From **nothing** to a built Exudyn and a passing test run, in order. It assumes no knowledge of the
project and no Python packaging experience; it does assume you can use a terminal.

If you only want to *use* Exudyn, you do not need any of this: `pip install exudyn`, see
{ref}`sec-install-installinstructions`.

| step | |
|---|---|
| 1 | [install git, a Python and a compiler](#what-you-need-installed) |
| 2 | [get the code](#get-the-code) |
| 3 | [create the environment](#create-the-environment) |
| 4 | [set up the clone](#set-up-the-clone-once) |
| 5 | [build once](#build-once) |
| 6 | [run the tests once](#run-the-tests-once) |
| 7 | [where to go next](#where-to-go-next) |

## What you need installed

- **git**. On Windows, [Git for Windows](https://git-scm.com/download/win) brings both the `git`
  command and the *Git Bash* shell.
- **A Python of a supported version**, 3.10 – 3.14, 64 bit. Anaconda or Miniconda is what the
  project uses, because the per-version test matrix is conda environments; any other Python of the
  right version works for a single build.
- **A C++17 compiler**: Visual Studio 2022 on Windows, GCC on Linux, the Xcode command line tools
  on macOS. Details, and the Linux OpenGL packages, are in {ref}`sec-dev-build`.
- **An editor.** The project is developed in **VS Code** for everything Python — the package, the
  definition files, the documentation — and in **Visual Studio 2022** for stepping from a Python
  script into the C++ with one debugger. Step 4 prepares both.

## Get the code

The public repository is on GitHub. Clone it over **https**, which works everywhere and needs
nothing set up:

```bash
git clone https://github.com/jgerstmayr/EXUDYN.git
cd EXUDYN
```

or over **ssh**, which needs a key in your GitHub account once and then never asks for a
password again:

```bash
git clone git@github.com:jgerstmayr/EXUDYN.git
```

Which one: **https** if you only read, or if you are behind a proxy that blocks ssh; **ssh** if you
push regularly. You can change your mind later with `git remote set-url origin <the other URL>`.
If https asks for a password when you push, it wants a *personal access token*, not your account
password — that is a GitHub rule, not an Exudyn one.

```{note}
Co-developers at the institute clone the **internal** repository instead, which carries the working
branch and the full history; ask the maintainer for the URL. The GitHub clone is the right one for
everybody else. Which branch is which, and what may be pushed where, is in
[WORKFLOW.md](WORKFLOW.md) §4 — the short version is that **nothing reaches GitHub before the
1.13 release**. Until then a GitHub clone is the 1.11 tree, which has neither this page nor the
`tools/exudev` driver it uses.
```

A directory named after the repository is fine anywhere; the project itself puts no constraint on
the path, and no path is compiled in.

## Create the environment

This is where the `venvExuP313` that the rest of the documentation names comes from. With conda:

```bash
conda create -n venvExuP313 python=3.13
conda activate venvExuP313
python -m pip install --upgrade pip    #--group needs pip >= 25.1
pip install --group dev                #the dependency groups of pyproject.toml, in the repository root
```

Run the commands of this page from the **repository root**, in a terminal where this environment is
active: on Windows the *Anaconda Prompt* (or a shell after `conda init`), elsewhere any shell after
`conda init`.

That one environment covers the generators, the documentation build, the checking tools and the
test suite. The per-version environments `venvP310` … `venvP314` exist only for the release test
matrix and are not needed to start.

The recipe in full — what each package is for, the package→feature map and the scipy pin — is in
[condaEnvironments.md](../howTo/condaEnvironments.md).

```{warning}
Do not work in the conda **base** environment. It carries whatever Exudyn was installed into it
last, and a test run from it silently tests old binaries. `python -m exudyn info` prints which
Exudyn is loaded and from where.
```

## Set up the clone, once

```bash
git config core.hooksPath tools/hooks
python tools/setupLocalWorkspace.py
```

The first line activates the tracked hooks in `tools/hooks/`. **Git hooks are not themselves
version controlled** — `.git/hooks/` never travels with a clone — so this is per clone and easy to
forget. The current hook is `pre-push`, and it refuses to push anything but `master`, `release/*`
and tags to the public repository.

The second line creates the **untracked** working files from their committed templates:

| created | from | what it is for |
|---|---|---|
| `exudyn.sln` | `exudynTemplate.sln` | the Visual Studio solution |
| `python/pytest.py` | `python/pytestTemplate.py` | the scratch file for trying something out under the debugger |
| `.vscode/c_cpp_properties.json` | `tools/vscodeCppPropertiesTemplate.json` | what lets **VS Code follow a C++ include** |

All three are in `.gitignore`, so an experiment cannot be committed by accident. An existing file
is never overwritten (`--force` does that deliberately). Without the third one the C/C++ extension
reports *"include errors detected"* and cannot navigate, because the vendored headers are reached
through subdirectories of `include/` (#2619).

## Build once

```bash
python tools/exudev build --env venvExuP313
```

About a minute. It builds the wheel, installs it into that environment and checks that the
installed version is the one just built. If it fails, or if you want to know what it does,
{ref}`sec-dev-build` is the page.

`exudev` is the project's driver for building, testing and the documentation; it lives in
`tools/exudev/` and is started as `python tools/exudev <command>` (on Windows `exudev <command>`
works as well, through `exudev.bat` in the repository root). `python tools/exudev --help` lists the
commands, and `-n` in front of a command prints what it would run without running it:
`python tools/exudev -n build --env venvExuP313`.

It runs each step in the named environment through conda. If it answers *conda installation not
found*, the terminal is not a conda shell: open one, or set `EXUDYN_CONDA_ROOT` to the conda
directory (the one containing `Scripts/` or `bin/`), or add `--no-conda` to use the Python that
started it.

Check it by hand:

```bash
python -m exudyn info          #version, package location, environment
python -m exudyn demo          #a small built-in model
```

## Run the tests once

```bash
python tools/exudev test --env venvExuP313
```

About 25 seconds for all test models and mini examples; it ends with `PASSED` or the list of what
failed. **Run this before you believe anything else works.** It is the gate every commit passes, and a failure here on a fresh clone is a problem
with the build or the environment, not with your change.

The other suites — the examples, the performance models, `pytest` — and when each of them is
required are in [WORKFLOW.md](WORKFLOW.md).

## Where to go next

| | |
|---|---|
| [GIT.md](GIT.md) | branches, commit messages, pull, push, merge — and what a contribution has to provide |
| [WORKFLOW.md](WORKFLOW.md) | the issue tracker, the version numbering, the CI and the gates a change passes |
| [CODING_STYLE.md](CODING_STYLE.md) | naming, headers, and how Exudyn reports an error |
| [ARCHITECTURE.md](ARCHITECTURE.md) | what the C++ core looks like and where the C++/Python boundary runs |
| {ref}`sec-dev-build` | the build itself, per platform |

The one thing to read before changing anything: large parts of the tree are **generated** —
`src/Autogenerated/` from `definitions/`, `docs/generated/` from the definitions and the
docstrings, `CHANGELOG.md` from the issue tracker. A hand edit to any of them survives until the
next generator run and no longer.
