(sec-install-installinstructions)=
# Installation instructions

(sec-install-installinstructions-requirements)=
## Requirements for Exudyn ?

Exudyn is a Python package with a compiled core, so what you need is a Python it was built for.

**Wheels are published for Python 3.10, 3.11, 3.12, 3.13 and 3.14**, on Windows, Linux and
MacOS. Newer Python versions are usually added in line with the Python release cycle (every
October).

**64-bit only.** Exudyn has not been built for 32-bit systems for years, and since
revision2026 step R2.6 the 32-bit build configurations no longer exist. If you still read advice
about matching 32-bit and 64-bit installations, it is out of date.

- **conda environments** are what the developers use and what is tested most; the environment decides which Python you get, and `pip install exudyn` inside it installs the matching wheel.
- **Spyder** (any recent version), **Visual Studio Code** and **Jupyter** all work with default settings. Spyder works with all virtual environments.
- The Python running your script and the Python Exudyn was installed into must be the same one. In an environment that is automatic; outside one it is the usual source of `import exudyn` failing, see {ref}`sec-install-troubleshootingfaq`.

 If you plan to extend the C++ code, use **Microsoft Visual Studio 2022**, which is
the primary development environment of the project and supports mixed Python/C++
debugging (VS2019 has problems with the library 'Eigen' and leads to erroneous results
with the sparse solver; it is not supported.).

### Run without Anaconda

If you do not install Anaconda (e.g., under Linux), make sure that you have the according Python packages installed:

- `numpy` (used throughout the code, inevitable)
- `matplotlib` (for any plot, also PlotSensor(...))
- `tkinter` (for interactive dialogs, SolutionViewer, etc.)
- `scipy` (needed for eigenvalue computation)

You can install most of these packages using `pip install numpy` (Windows) or `pip3 install numpy` (Linux).
NOTE: as there is only `numpy` needed (but not for all sub-packages) and `numpy` supports many variants, we do not add a particular requirement for installation of depending packages. It is not necessary to install `scipy` as long as you are not using features of `scipy`. Same reason for `tkinter` and `matplotlib`.

For interaction (right-mouse-click, some key-board commands) you need the Python module `tkinter`. This is included in regular Anaconda distributions (recommended, see below), but on Ubuntu you need to type alike (do not forget the '3', otherwise it installs for Python2 ...):

- `sudo apt-get install python3-tk`

see also common blogs for your operating system.

(sec-install-installinstructions-pipinstall)=
## Install Exudyn with PIP INSTALLER (pypi.org)

Pre-built versions of Exudyn are hosted on `pypi.org`, see the project

- [https://pypi.org/project/exudyn](https://pypi.org/project/exudyn)

As with most other packages, in the regular case (if your binary has been pre-built) you just need to do

- `pip install exudyn`

On Ubuntu/Linux, make sure that pip is installed and up-to-date (**update pip to at least 20.3**; otherwise the manylinux wheels will not be accepted!):

- `sudo apt install python3-pip`
- `python3 -m pip install --upgrade`

Depending on installation the command may read `pip3` or `pip`:

- `pip3 install exudyn`

For pre-releases (use with care!), add `--pre` flag:

- `pip install exudyn --pre -U`

The `-U` (identical to `--upgrade`) flag ensures that the current installed version is also updated in case of a change of the micro version (e.g., from version 1.6.119 to version 1.6.164), otherwise, it will only update if you switch to a newer minor version.

In some cases (e.g. for AppleM1 or special Linux versions), your pre-built binary will not work due to some incompatibilities. Then you need to build from source as described in the 'Build and install' sections, {ref}`sec-install-installinstructions-buildwindows`.

### Troubleshooting pip install

Pip install may fail, if your linux version does not support the current manylinux version.
This was known for Red Hat, CentOS, Rocky Linux or simlilar systems which usually support manylinux2014. In this case, you had to build Exudyn from source, see {ref}`sec-install-installinstructions-buildubuntu`. Since version 1.7.116, the manylinux2014 version is supported and according problems should be solved.

Sometimes, you install exudyn, but when running python, the `import exudyn` fails.
In case of several environments, check where your installation goes. To guarantee that the pip install goes to the python call, use:

- `python -m pip install exudyn`

which ensures that the used python is calling its associated pip module.

If the PyPi index is not updated, it may help to use

- `pip install -i https://pypi.org/project/ exudyn`

(sec-install-installinstructions-wheel)=
## Install from specific Wheel (Ubuntu and Windows)

A way to install the Python package Exudyn is to use the so-called 'wheels' (file ending `.whl`).
NOTE that this approach usually is not required; usually, just use the pip installer of the previous section!

Wheels can be downloaded directly from
[https://pypi.org/project/exudyn/#files](https://pypi.org/project/exudyn/#files), one per
Python version and platform. The file name says which one it is: `cp313` means CPython 3.13,
and the last part names the platform.

 Check which Python you are about to install into:

- `python --version`

 and then install the matching wheel (the version number `1.11.0` below is an
example):

- **Windows**, Python 3.13: `pip install exudyn-1.11.0-cp313-cp313-win_amd64.whl`
- **Linux**, Python 3.13: `pip3 install exudyn-1.11.0-cp313-cp313-manylinux_2_28_x86_64.whl`
- **MacOS**, Python 3.13: `pip3 install exudyn-1.11.0-cp313-cp313-macosx_11_0_arm64.whl`

 The same pattern holds for `cp310` to `cp314`. Installing a wheel built
for a different Python version fails with a message about the wheel not being supported on this
platform, which is the intended outcome rather than a problem to work around.

 If no wheel works on your system, build Exudyn for it as described in
{ref}`sec-install-installinstructions-buildubuntu`.

(sec-install-installinstructions-buildwindows)=
## Build and install Exudyn under Windows 10

Note that there are a couple of pre-requisites, depending on your system and installed libraries. For Windows 10, the following steps proved to work:

- you need an appropriate compiler (Microsoft Visual Studio 2022; see {ref}`sec-install-installinstructions-requirements`)
- install your Anaconda distribution including Spyder or use VisualStudioCode; you can also use a miniconda (only numpy is really required and distribution tools)
- close all Python programs (e.g. Spyder, Jupyter, ...)
- run an Anaconda prompt (may need to be run as administrator, depends on installation)
- it is recommended to use conda environments!
- go to 'main' of your cloned github folder of Exudyn
- run: (Since version 1.7.116 a PEP518 compatible build is available. This should work with Windows, MacOS and linux. The `setupPyConfig.json` file includes some flags such as the parallel compilation, GLFW, etc.; the `-v` flag adds verbosity.) `pip wheel . -v -w dist --no-deps`
- then install exactly that wheel: `pip install dist/<the wheel that was built>`
- read the output; if there are errors, try to solve them by installing appropriate modules

You can also create your own wheels, doing the above steps to activate the according Python version and then calling:

- `python setup.py bdist_wheel --parallel`

This will add a wheel in the `dist` folder.

(sec-install-installinstructions-buildmacos)=
## Build and install Exudyn under Mac OS X

Installation and building on Mac OS X is less frequently tested, but successful compilation including GLFW has been achieved.
Requirements are an according Anaconda (or Miniconda) installation.

 **Tested configurations**:

- Mac OS 11.x 'Big Sur', Mac Mini (2021), Apple M1, 16GB Memory
- Miniconda with conda environments (x86 / i368 based with Rosetta 2) with Python 3.7 - 3.11
- Miniconda with conda environments (ARM) with Python 3.8 - 3.11
- $\ra$ wheels are available on pypi since Exudyn 1.5.0

 **NOTE**:

- New `universal2` wheels should support x86 (APPLE Intel and Python/Rosetta on APPLE Silicon)
- Multi-threading is not fully supported on MacOS, but may work in some applications
- On Apple M1 processors the newest Anaconda supports now all required features; environments with Python 3.8-3.11 have been successfully tested;
- The Rosetta (x86 emulation) mode on Apple M1 also works now without much restrictions; these files should also work on older Macs
- If you have a MacOS version$<11$, it worked to download wheels from PyPI, change wheel names, e.g., from `exudyn-1.7.116.dev1-cp311-cp311-macosx_11_0_x86_64.whl` to `exudyn-1.7.116.dev1-cp311-cp311-macosx_10_9_x86_64.whl`. This also works for universal2 files. Installation worked and wheels were running smoothly.
- `tkinter` has been adapted (some workarounds needed on MacOS!), available since Exudyn 1.5.15.dev1
- Some optimization and processing functions do not run (especially multiprocessing and tqdm);

Alternatively, we tested on:

- Mac OS X 10.11.6 'El Capitan', Mac Pro (2010), 3.33GHz 6-Core Intel Xeon, 4GB Memory, Anaconda Navigator 1.9.7, Python 3.7.0, Spyder 3.3.6

**Compile from source**:\
If you would like to compile from source, just use a bash terminal on your Mac, and do the following steps inside the `main` directory of your repository and type

- uninstall if old version exists (may need to repeat this!): `pip uninstall exudyn`
- remove the `build` directory if you would like to re-compile without changes
- to perform compilation from source, write: (the `--parallel` option performs parallel compilation on multithreaded CPUs and can speedup by 2x - 8x)
- Since version 1.7.116: `pip wheel . -v -w dist --no-deps`
- Until version 1.7.116: `python setup.py bdist_wheel --parallel`
- which takes 75 seconds on Apple M1 in parallel mode, otherwise 5 minutes. To install Exudyn, run
- then `pip install dist/<the wheel that was built>`; `python setup.py install` does not exist any more in current setuptools
- $\ra$ this will only install, but not re-compile. Otherwise, just use pip install from the created wheel in the dist folder
- **NOTE** that conda environments are highly recommended

Then just go to the `python/Examples` folder and run an example:

- `python springDamperUserFunctionTest.py`

If there are other issues, we are happy to receive your detailed bug reports.

 Note that you need to run

```python
  SC.renderer.Start()
  SC.renderer.DoIdleTasks()
```

in order to interact with the render window, as there is only a single-threaded version available for Mac OS.

(sec-install-installinstructions-buildubuntu)=
## Build and install Exudyn under Ubuntu

Having a new Ubuntu 18.04 standard installation (e.g. using a VM virtual box environment), the following steps need to be done (Python **3.6** is already installed on Ubuntu 18.04, otherwise use `sudo apt install python3`) (see also the youtube video: [https://www.youtube.com/playlist?list=PLZduTa9mdcmOh5KVUqatD9GzVg_jtl6fx](https://www.youtube.com/playlist?list=PLZduTa9mdcmOh5KVUqatD9GzVg_jtl6fx)):

 First update ...

```
  sudo apt-get update
```

Install necessary Python libraries and pip3; `matplotlib` and `scipy` are not required for installation but used in Exudyn examples:

```python
  sudo dpkg --configure -a
  sudo apt install python3-pip
  pip3 install numpy
  pip3 install matplotlib
  pip3 install scipy
```

 Install pybind11 (needed for running the setup.py file derived from the pybind11 example):

```python
  pip3 install pybind11
```

 To have dialogs enabled, you need to install `Tk`/`tkinter` (may be already installed in your case).
`Tk` is installed on Ubuntu via apt-get and should then be available in Python:

```python
  sudo apt-get install python3-tk
```

If graphics is used (`#define USE_GLFW_GRAPHICS` in `BasicDefinitions.h`), you must install the according GLFW libs:

```python
  sudo apt-get install libglfw3 libglfw3-dev
```

In some cases, it may be required to install OpenGL and some of the following libraries:

```python
  sudo apt-get install freeglut3 freeglut3-dev
  sudo apt-get install mesa-common-dev
  sudo apt-get install libx11-dev xorg-dev libglew1.5 libglew1.5-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev
```

With all of these libs, you can build the wheel (go to the root of `Exudyn_git`; until version 1.11 the sources lay in its `main/` subfolder), which takes some minutes for compilation (the --user option is used to install in local user folder) (the `--parallel` option performs parallel compilation on multithreaded CPUs and can speedup by 2x - 8x):

```python
  pip wheel . -v -w dist --no-deps && pip install dist/<the wheel that was built>
```

Since version 1.7.116, a PEP518 compatible way to compile sources and install the current repository has been added (the `-v` flag activates a verbose mode):

```python
  pip install . -v --no-deps
```

Congratulation! **Now, run a test example** (will also open an OpenGL window if successful):

- `python3 python/Examples/rigid3Dexample.py`

 You can also create a Ubuntu wheel which can be easily installed on the same machine (x64), same operating system (Ubuntu 18.04) and with same Python version (e.g., 3.6):

- `sudo pip3 install wheel`
- `sudo python3 setup.py bdist_wheel --parallel`

Note that the build mechanisms used for Exudyn, e.g., on GitHub or when building internal wheels use docker in order to build highly compatible multilinux wheels.

 Since version 1.7.116, the PEP518 compatible way which puts wheels into the `dist` folder reads:

- `pip wheel . -v -w dist --no-deps`

 **Exudyn under Ubuntu / WSL**:

- Note that Exudyn also nicely works under WSL (Windows subsystem for linux; tested for Ubuntu 18.04) and an according xserver (VcXsrv).
- In case of old WSL2, just set the display variable in your .bashrc file accordingly and you can enjoy the OpenGL windows and settings.
- It shall be noted that WSL + xserver works better than on MacOS, even for tkinter, multitasking, etc.! So, if you have troubles with your Mac, use a virtual machine with ubuntu and a xserver, that may do better
- In case of WSLg (since 2021), only the software-OpenGL works; therefore, you have to set (possibly in .bashrc file): `export LIBGL_ALWAYS_SOFTWARE=0`

 **Exudyn under RaspberryPi 4b**:

- Exudyn also compiles under RaspberryPi 4b, Ubuntu Mate 20.04, Python 3.8; current version should compile out of the box with `pip wheel . -v -w dist --no-deps`.
- Performance is quite ok and it is even capable to use all cores (but you should add a fan!)
- $\ra$ this could be used for a nice realtime application!

 **KNOWN issues for linux builds**:

- Using **WSL2** (Windows subsystem for linux), there occur some conflicts during build because of incompatible windows and linux file systems and builds will not be copied to the dist folder; workaround: go to explorer, right click on 'build' directory and set all rights for authenticated user to 'full access'
- **compiler (gcc,g++) conflicts**: It seems that Exudyn works well on Ubuntu 18.04 with the original `Python 3.6.9` and `gcc-7.5.0` version as well as with Ubuntu 20.04 with `Python 3.8.5` and `gcc-9.3.0`. Upgrading `gcc` on a Linux system with Python 3.6 to, e.g., `gcc-8.2` showed us a linker error when loading the Exudyn module in Python -- there are some common restriction using `gcc` versions different from those with which the Python version has been built. Starting `python` or `python3` on your linux machine shows you the `gcc` version it had been build with. Check your current `gcc` version with: `gcc --version`

(sec-install-installinstructions-uninstall)=
## Uninstall Exudyn

To uninstall exudyn under Windows, run (may require admin rights):

- `pip uninstall exudyn`

 To uninstall under Ubuntu, run:

- `sudo pip3 uninstall exudyn`

If you upgrade to a newer version, uninstall is usually not necessary!

## How to install Exudyn and use the C++ source code (advanced)?

Exudyn is still under intensive development of core modules.
There are several ways of using the code, but you **cannot** install Exudyn as compared to other executable programs and apps.
\
In order to make full usage of the C++ code and extending it, you can use:

- Windows / Microsoft Visual Studio 2017 and above:

  - get the files from git
  - put them into a local directory (recommended: `C:/DATA/cpp/EXUDYN_git`)
  - start `main_sln.sln` with Visual Studio (recommended version: 2017, otherwise you have to manually adapt)
  - compile the code and run `python/pytest.py` example code
  - adapt `pytest.py` for your applications
  - extend the C++ source code
  - link it to your own code
  - NOTE: on Linux systems, you mostly need to replace '$/$' with '$\backslash$'

- Linux, etc.: Use the build methods described above; Visual Studio Code may allow native Python and C++ debugging; switching to other build mechanisms (e.g. scikit-build-core).
