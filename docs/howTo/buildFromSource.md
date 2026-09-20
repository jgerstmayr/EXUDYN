# Building Exudyn from source

Exudyn builds with plain setuptools: `setup.py` in the repository root compiles the C++ sources
listed in `sources.json` into the `exudynCPP` extension module and packs it, together with the
Python package under `python/exudyn`, into a wheel. About a minute on a normal machine — that speed
is a feature of the project, not an accident.

Most of the time you do not need any of the commands below:

```powershell
python -m exudev build --env venvExuP313
```

builds the wheel and installs it into that environment, then checks that the installed version is
the one just built. This page is what that driver does, and what to do when it does not work.

## Requirements

- **Python 3.10 – 3.14, 64 bit.** See [condaEnvironments.md](condaEnvironments.md) for the
  environments the project builds and tests against.
- **A C++17 compiler**: Visual Studio 2022 on Windows (see
  [visualStudio2022.md](visualStudio2022.md)), GCC on Linux.
- **numpy**, and `wheel` if you build wheels by hand.
- Nothing else: pybind11, GLFW, Eigen and LEST are **vendored** in `include/`, and the GLFW import
  library for Windows is in the repository.

## Windows

From the repository root, with the environment active (an Anaconda prompt, or
`conda activate venvExuP313`), and with Spyder or any other process that has Exudyn imported
closed:

```powershell
python -m pip wheel . -w dist --no-deps
pip install --force-reinstall --no-deps dist\exudyn-1.11.218.dev1-cp313-cp313-win_amd64.whl
```

The wheel's name carries the Python tag (`cp313`), and **it must match the interpreter you install
it into**. `--force-reinstall --no-deps` installs exactly that file and nothing else; installing by
name or with `--find-links` may quietly pick an older wheel from `dist/`, which is the whole
"I tested yesterday's binary" class of failure.

To remove it again:

```powershell
pip uninstall -y exudyn
```

## Linux

The build itself is the same; what is different is that the OpenGL and X11 development packages
have to be installed. On a fresh Ubuntu:

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
all of the above.

Then:

```bash
python3 -m pip wheel . -w dist --no-deps
pip3 install --force-reinstall --no-deps dist/exudyn-*-cp313-cp313-linux_x86_64.whl
```

Use `pip3`/`python3` consistently; mixing them with `pip`/`python` is the most common way to end up
with the module installed for an interpreter you are not running.

### A specific GCC version

The GCC version is printed when `python3` starts. To install and use another one:

```bash
sudo apt install software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt install gcc-9 g++-9
```

and before building:

```bash
export CC=gcc-9
export CXX=g++-9
```

### WSL

Building inside WSL on a Windows directory fails on file permissions. Right-click the root folder →
*Properties* → *Security*, and give *Authenticated Users* full control; then the wheel builds.

## When the build works and the import does not

```
ImportError: .../exudyn.cpython-313-x86_64-linux-gnu.so: undefined symbol: ...
```

An undefined symbol at **import** time is a compile-time mistake that the linker of a shared object
does not catch — most often a `static const` used as a template argument, which needs to be
`constexpr`. [gccVsMsvcTraps.md](gccVsMsvcTraps.md) collects that one and its relatives.

For the Windows side of "it built but it does not work", see [buildQuirks.md](buildQuirks.md).

## Debugging the C++ from a Python run

Build with debug information:

```bash
CFLAGS='-Wall -O0 -g' python3 -m pip wheel . -w dist --no-deps
```

and run the model under gdb:

```bash
sudo apt-get install gdb
gdb python3
(gdb) run myModel.py
```

On Windows this is what Visual Studio does much better: mixed Python/native debugging, a breakpoint
in the C++ hit from the Python model — see [visualStudio2022.md](visualStudio2022.md).

For Python-level debugging, `python3 -m pdb myModel.py`.

## Cleaning up

```powershell
python -m exudev clean          #build directories and eggs; keeps dist/
```

by hand:

```bash
rm -r build dist exudyn.egg-info .eggs
```

Note that **the package tree under `build/lib.*` is removed before every build** since revision2026
step R5.18.8: a module deleted from `python/exudyn` used to survive there and be copied into the
wheel.
