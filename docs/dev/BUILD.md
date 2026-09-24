(sec-dev-build)=
# Building Exudyn from source

This is the **one** description of building Exudyn. Until revision2026b step RG3.12.2 (#2646) there
were three — two sections of the user manual, a how-to note and a paragraph in the developer
README — and they disagreed with each other and with the repository.

You need this page if you want to **change the C++ core**, or if there is **no wheel for your
platform**. To *use* Exudyn, `pip install exudyn` is enough; see
{ref}`sec-install-installinstructions`.

If you have not cloned the repository yet, start at
[GETTING_STARTED.md](GETTING_STARTED.md), which goes from *nothing* to a build and a passing test
run in order.

---

## What a build is, and the one command

`setup.py` in the repository root compiles the C++ sources listed in `sources.json` into the
`exudynCPP` extension module and packs it, together with the Python package under `python/exudyn`,
into a wheel. **About a minute** on a normal machine — that speed is a feature of the project, not
an accident, and it is one of the invariants (see [README.md](README.md)).

Most of the time you do not type any of the platform commands below:

```powershell
python -m exudev build --env venvExuP313
```

builds the wheel, installs it into that environment and then **checks that the installed version is
the one just built** — the failure mode this guards against is testing yesterday's binary. The rest
of this page is what that driver does, and what to do when it does not work.

## What you need, on every platform

| | |
|---|---|
| **Python** | 3.10 – 3.14, **64 bit**. 32-bit builds no longer exist (revision2026 step R2.6). |
| **A C++17 compiler** | Visual Studio 2022 on Windows, GCC on Linux, clang (Xcode command line tools) on macOS. |
| **numpy** | in the environment you build into; `wheel` as well if you build wheels by hand. |
| **The repository** | see [GETTING_STARTED.md](GETTING_STARTED.md). |

**Nothing else.** pybind11, GLFW, Eigen and LEST are **vendored** in `include/`, and the GLFW import
library for Windows is in `libs/`. What is *not* vendored, and what the platform sections below are
about, are the **OpenGL and X11 development packages on Linux**.

The environments the project builds and tests against are in
[condaEnvironments.md](../howTo/condaEnvironments.md). Conda is a recommendation, not a
requirement: any Python of a supported version works.

### The two commands that are the same everywhere

```bash
python -m pip wheel . -w dist --no-deps
pip install --force-reinstall --no-deps dist/<the wheel that was built>
```

The wheel's name carries the Python tag (`cp313`), and **it must match the interpreter you install
it into**. `--force-reinstall --no-deps` installs exactly that file and nothing else; installing by
name, or with `--find-links`, may quietly pick an older wheel out of `dist/`.

Close Spyder, Jupyter or any other process that has Exudyn imported before installing over it —
on Windows the file is locked, on Linux the old module stays loaded in that process.

To remove it again: `pip uninstall -y exudyn`.

---

## Windows

With the environment active — an Anaconda prompt, or `conda activate venvExuP313` — from the
repository root:

```powershell
python -m pip wheel . -w dist --no-deps
pip install --force-reinstall --no-deps dist\exudyn-1.12.61.dev1-cp313-cp313-win_amd64.whl
```

The compiler is Visual Studio 2022; which components it needs is in
[visualStudio2022.md](https://github.com/jgerstmayr/EXUDYN/blob/master/docs/howTo/visualStudio2022.md).
VS2019 is **not** supported: it has problems with Eigen and produces wrong results with the sparse
solver.

For building and debugging *inside* Visual Studio rather than with `pip wheel`, see
[GETTING_STARTED.md](GETTING_STARTED.md) — `tools/setupLocalWorkspace.py` creates the solution.

## Linux

The build is the same; what is different is that the OpenGL and X11 development packages have to
be installed. On a fresh Ubuntu:

```bash
sudo apt-get update
sudo apt install python3-pip
pip3 install numpy

#graphics: OpenGL, GLFW and X11 development packages
sudo apt-get install freeglut3 freeglut3-dev mesa-common-dev libglfw3 libglfw3-dev \
                     libx11-dev xorg-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-dev
```

`libglfw.so` and `libGL.so` are then in `/usr/lib/x86_64-linux-gnu`. Graphics can also be switched
off entirely in `src/Utilities/BasicDefinitions.h` (`USE_GLFW_GRAPHICS`), which removes the need for
all of the above — useful on a compute cluster.

Then:

```bash
python3 -m pip wheel . -w dist --no-deps
pip3 install --force-reinstall --no-deps dist/exudyn-*-cp313-cp313-linux_x86_64.whl
```

Use `pip3`/`python3` **consistently**; mixing them with `pip`/`python` is the most common way to end
up with the module installed for an interpreter you are not running.

### A specific GCC version

The GCC version Python was built with is printed when `python3` starts, and a module built with a
very different one can fail to load. To install and select another:

```bash
sudo apt install software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt install gcc-9 g++-9
export CC=gcc-9
export CXX=g++-9
```

### WSL

Building inside WSL **on a Windows directory** fails on file permissions: right-click the
repository folder → *Properties* → *Security*, and give *Authenticated Users* full control.

Under WSLg only the software OpenGL works, so set `export LIBGL_ALWAYS_SOFTWARE=1` (in `.bashrc`)
if the render window stays black.

### Other Linux platforms

Exudyn compiles on a **RaspberryPi 4b** (Ubuntu Mate, Python 3.8) with the same
`pip wheel . -w dist --no-deps`, uses all cores, and is fast enough to be interesting for a
realtime application — with a fan.

The **release** wheels for Linux are not built this way: they are built in the manylinux docker
image, so that they run on distributions older than the build machine. That is
`exudev linux`; see [`tools/exudev/README.md`](../../tools/exudev/README.md).

## macOS

Less frequently tested than Windows and Linux, but the wheels on PyPI are built for it and
compiling from source works, GLFW included. Requirements are a Python of a supported version
(Miniconda is the usual way) and the Xcode command line tools for the compiler.

```bash
python -m pip wheel . -w dist --no-deps
pip install --force-reinstall --no-deps dist/<the wheel that was built>
```

About 75 seconds on an Apple M1 with parallel compilation.

What is different on macOS, and is not a build problem:

- the **renderer is single-threaded** — GLFW cannot render from a second thread there. A script
  therefore has to hand the renderer time itself:

  ```python
  SC.renderer.Start()
  SC.renderer.DoIdleTasks()
  ```

- **multiprocessing** and the progress bars that use it do not work in all cases;
- the **fast module is not built** on macOS (`universal2` compiles both architectures in one pass,
  and AVX2 does not exist on ARM); `exudev build --fast` says so and continues.

## When the build works and the import does not

```
ImportError: .../exudyn.cpython-313-x86_64-linux-gnu.so: undefined symbol: ...
```

An undefined symbol at **import** time is a compile-time mistake that the linker of a shared object
does not catch — most often a `static const` used as a template argument, which needs to be
`constexpr`.
[gccVsMsvcTraps.md](https://github.com/jgerstmayr/EXUDYN/blob/master/docs/howTo/gccVsMsvcTraps.md)
collects that one and its relatives.

For the Windows side of *"it built but it does not work"*, see
[buildQuirks.md](https://github.com/jgerstmayr/EXUDYN/blob/master/docs/howTo/buildQuirks.md).

If `import exudyn` reports the **wrong version**, the wheel was installed into a different
interpreter than the one running: `python -m exudyn info` prints which Python, which Exudyn and
from where.

## Debugging the C++ from a Python run

On **Windows this is what Visual Studio 2022 is for**: mixed Python/native debugging, a breakpoint
in the C++ hit from a Python model, which is the project's most valuable development capability.
`tools/setupLocalWorkspace.py` creates `exudyn.sln` and the scratch file `python/pytest.py` for it;
see [GETTING_STARTED.md](GETTING_STARTED.md).

On **Linux**, build with debug information and run under gdb:

```bash
CFLAGS='-Wall -O0 -g' python3 -m pip wheel . -w dist --no-deps
sudo apt-get install gdb
gdb python3
(gdb) run myModel.py
```

For Python-level debugging, `python3 -m pdb myModel.py`.

## Cleaning up

```powershell
python -m exudev clean          #the build directories and eggs of this platform; keeps dist/
```

or by hand:

```bash
rm -r build dist exudyn.egg-info .eggs
```

The package tree under `build/lib.*` is removed **before every build** since revision2026 step
R5.18.8: a module deleted from `python/exudyn` used to survive there and be copied into the wheel,
which shipped a file the source did not have.
