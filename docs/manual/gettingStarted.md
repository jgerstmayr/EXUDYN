(sec-installation-gettingstarted)=
# Installation and Getting Started

The overview of what Exudyn is and what it can do is the page before this one, `README.rst` -
the landing page of the repository and of the PyPI package, which is part of this documentation
rather than a second copy of it (revision2026 step R7.1.5).


 Exudyn is hosted on [GitHub](https://github.com) [EXUDYNgit]:

- [https://github.com/jgerstmayr/EXUDYN](https://github.com/jgerstmayr/EXUDYN)

 Online documentation is available at:

- [https://jgerstmayr.github.io/EXUDYN](https://jgerstmayr.github.io/EXUDYN)
- [https://exudyn.readthedocs.io](https://exudyn.readthedocs.io)

For any comments, requests, issues, bug reports, send an email to:

- email: `reply.exudyn@gmail.com`

Thanks for your contribution!

## Getting started

This section will show:

1. What is Exudyn ?
2. Who is developing Exudyn ?
3. How to install Exudyn
4. How to use Exudyn in Python
5. Goals of Exudyn
6. Run a simple example in Python
7. FAQ -- Frequently asked questions

### What is Exudyn ?

Exudyn -- (fl**EX**ible m**U**ltibody **DYN**amics  -- **EX**tend yo**U**r **DYN**amics) \
 Exudyn is a C++ based Python library for efficient simulation of flexible multibody dynamics systems.
It is the follow up code of the previously developed multibody code HOTINT, which Johannes Gerstmayr started during his PhD-thesis.
It seemed that the previous code HOTINT reached limits of further (efficient) development and it seemed impossible to continue from this code as it was outdated regarding programming techniques and the numerical formulation at the time Exudyn was started.

Exudyn is designed to easily set up complex multibody models, consisting of rigid and flexible bodies with joints, loads and other components. It shall enable automatized model setup and parameter variations, which are often necessary for system design but also for analysis of technical problems. The broad usability of Python allows to couple a multibody simulation with environments such as optimization, statistics, data analysis, machine learning and others.

The multibody formulation is mainly based on redundant coordinates. This means that computational objects (rigid bodies, flexible bodies, ...) are added as independent bodies to the system. Hereafter, connectors (e.g., springs or constraints) are used to interconnect the bodies. The connectors are using Markers on the bodies as interfaces, in order to transfer forces and displacements.
For details on the interaction of nodes, objects, markers and loads see {ref}`sec-overview-items`. For a non-redundant formulation, see `ObjectKinematicTree` -- this allows to create tree-structures with minimal coordinates in Exudyn.

There are several journal papers of the developers which were using Exudyn (list is incomplete -- see google scholar):

- J. Gerstmayr. Exudyn -- a C++-based Python package for flexible multibody systems. Multibody System Dynamics (2023). \url{https://doi.org/10.1007/s11044-023-09937-1}
- J. Gerstmayr, P. Manzl, M. Pieber. Multibody Models Generated from Natural Language, Multibody System Dynamics (2024). \url{https://doi.org/10.1007/s11044-023-09962-0}
- P. Manzl, O. Rogov, J. Gerstmayr, A. Mikkola, G. Orzechowski. Reliability Evaluation of Reinforcement Learning Methods for Mechanical Systems with Increasing Complexity.  Preprint, Research Square, 2023.  \url{https://doi.org/10.21203/rs.3.rs-3066420/v1}
- S. Holzinger, M. Arnold, J. Gerstmayr. Evaluation and Implementation of Lie Group Integration Methods for Rigid Multibody Systems. Preprint, Research Square, 2023.  \url{https://doi.org/10.21203/rs.3.rs-2715112/v1}
- M. Sereinig, P. Manzl, and J. Gerstmayr. Task Dependent Comfort Zone, a Base Placement Strategy for Autonomous Mobile Manipulators using Manipulability Measures, Robotics and Autonomous Systems, submitted. [Sereinig2023comfortZone]
- R. Neurauter, J. Gerstmayr. A novel motion reconstruction method for inertial sensors with constraints, Multibody System Dynamics, 2022. [NeurauterGerstmayr2023]
- M. Pieber, K. Ntarladima, R. Winkler, J. Gerstmayr. A Hybrid ALE Formulation for the Investigation of the Stability of Pipes Conveying Fluid and Axially Moving Beams, ASME Journal of Computational and Nonlinear Dynamics, 2022.
- S. Holzinger, M. Schieferle, C. Gutmann, M. Hofer, J. Gerstmayr. Modeling and Parameter Identification for a Flexible Rotor with Impacts. Journal of Computational and Nonlinear Dynamics, 2022.
- S. Holzinger, J. Gerstmayr. Time integration of rigid bodies modelled with three rotation parameters, Multibody System Dynamics, Vol. 53(5), 2021.
- A. Zw{\"o}lfer, J. Gerstmayr. The nodal-based floating frame of reference formulation with modal reduction. Acta Mechanica, Vol. 232, pp.  835--851 (2021).
- A. Zw{\"o}lfer, J. Gerstmayr. A concise nodal-based derivation of the floating frame of reference formulation for displacement-based solid finite elements, Journal of Multibody System Dynamics, Vol. 49(3), pp. 291 -- 313, 2020.
- S. Holzinger, J. Sch{\"o}berl, J. Gerstmayr. The equations of motion for a rigid body using non-redundant unified local velocity coordinates. Multibody System Dynamics, Vol. 48, pp. 283 -- 309, 2020.

### Developers of Exudyn and thanks

Exudyn is developed at the University of Innsbruck.
In general, most of the Exudyn is written by Johannes Gerstmayr, implementing ideas that followed from the project HOTINT [GerstmayrEtAl2013], but also creating many new concepts and approaches. 15 years of development of HOTINT led to a lot of lessons learned and after 20 years, a code must be re-designed.

Some important tests for the coupling between C++ and Python have been written by Stefan Holzinger. Stefan also helped to set up the previous upload to GitLab and to test parallelization features.
For the interoperability between C++ and Python, we extensively use **Pybind11**[pybind11], originally written by Jakob Wenzel, see `https://github.com/pybind/pybind11`. Without Pybind11 we couldn't have made this project -- Thanks a lot!

Important discussions with researchers from the community were important for the design and development of Exudyn , where we like to mention Joachim Sch{\"o}berl from TU-Vienna who boosted the design of the code with great concepts.

The cooperation and funding within the EU H2020-MSCA-ITN project 'Joint Training on Numerical Modelling of Highly Flexible Structures for Industrial Applications' contributes to the development of the code.

The following people have contributed to Python and C++ library implementations, testing, examples or theory:

- Joachim Sch{\"o}berl, TU Vienna (Providing specialized NGsolve [Schoeberl1997; NGsolve2014; NGsolve2022] core library with `taskmanager` for **multi-threaded parallelization**, which is now replaced by a simplified internal version but closely following the original implementation; NGsolve mesh and FE-matrices import; highly efficient eigenvector computations)
- Stefan Holzinger, University of Innsbruck (Lie group module and solvers in Python, Lie group node; helped with Lie group solvers, geometrically exact beam; testing)
- Peter Manzl, University of Innsbruck (ConvexRoll Python and C++ implementation; revised artificialIntelligence, ParameterVariation, robotics and MPI parallelization; providing many figures for theDoc; pip install on linux, wsl with graphics; several other fixes; examples)
- Andreas Zw{\"o}lfer, Technical University Munich (theory and examples for FFRF, CMS formulation and ANCF 2D cable prototypes in MATLAB)
- Michael Pieber, University of Innsbruck (helped in several Python libraries; ComputeODE2Eigenvalues with constraints, FEM and CMS testing; Abaqus import and test files; ANCFCable2D+ALE theory improvements and equations check; examples); Exudyn graphical user interface
- Martin Sereinig, University of Innsbruck (special robotics functionality, mobile robots, manipulability measures, robot models)
- Sebastian Weyrer, University of Innsbruck (ObjectContactSphereSphere; FEM RigidBodyInertia; some fixes; examples)
- Grzegorz Orzechowski, Lappeenranta University of Technology (coupling with openAI gym and running machine learning algorithms)
- Zhaowei Zhang, Chinese Academy of Sciences (ANCFThinPlate; found several issues and bugs)
- Aaron Bacher, University of Innsbruck (helped to integrated OpenVR, connection with Franka Emika Panda)
- Martin Arnold, Martin-Luther-University of Halle-Wittenberg (support for explicit and implicit Lie group solvers, especially to theory / jacobians and automatic step size)
- Konstantina Ntarladima, University of Innsbruck (ANCFCable2D+ALE theory improvements and equations check)
- Alexander Humer, Johannes Kepler University Linz (initial discussions on structure and C++ code)
- Qasim Khadim, University of Oulu (suggestion for improved model of HydraulicsActuatorSimple with effective bulk modulus)
- Michael Gerbl, University of Innsbruck (figures in the documentation, taken from lecture notes)
- further examples provided by: Manuel Schieferle, Martin Knapp, Lukas March, Dominik Sponring, David Wibmer, Simon Scheiber and other Master students

-- thanks a lot! --

(sec-install-installinstructions)=
## Installation instructions

(sec-install-installinstructions-requirements)=
### Requirements for Exudyn ?

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

#### Run without Anaconda

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
### Install Exudyn with PIP INSTALLER (pypi.org)

Pre-built versions of Exudyn are hosted on `pypi.org`, see the project

- [https://pypi.org/project/exudyn](https://pypi.org/project/exudyn)

As with most other packages, in the regular case (if your binary has been pre-built) you just need to do

- `pip install exudyn`

On Ubuntu/Linux, make sure that pip is installed and up-to-date (**update pip to at least 20.3**; otherwise the manylinux wheels will not be accepted!):

- `sudo apt install python3-pip`
- `python3 -m pip install --upgrade`

Depending on installation the command may read `pip3` or `pip`:

- `pip3 install exudyn`

For pre-releases (use with care!), add `-{}-pre` flag:

- `pip install exudyn -{}-pre -U`

The `-U` (identical to `--upgrade`) flag ensures that the current installed version is also updated in case of a change of the micro version (e.g., from version 1.6.119 to version 1.6.164), otherwise, it will only update if you switch to a newer minor version.

In some cases (e.g. for AppleM1 or special Linux versions), your pre-built binary will not work due to some incompatibilities. Then you need to build from source as described in the 'Build and install' sections, {ref}`sec-install-installinstructions-buildwindows`.

#### Troubleshooting pip install

Pip install may fail, if your linux version does not support the current manylinux version.
This was known for Red Hat, CentOS, Rocky Linux or simlilar systems which usually support manylinux2014. In this case, you had to build Exudyn from source, see {ref}`sec-install-installinstructions-buildubuntu`. Since version 1.7.116, the manylinux2014 version is supported and according problems should be solved.

Sometimes, you install exudyn, but when running python, the `import exudyn` fails.
In case of several environments, check where your installation goes. To guarantee that the pip install goes to the python call, use:

- `python -m pip install exudyn`

which ensures that the used python is calling its associated pip module.

If the PyPi index is not updated, it may help to use

- `pip install -i https://pypi.org/project/ exudyn`

(sec-install-installinstructions-wheel)=
### Install from specific Wheel (Ubuntu and Windows)

A way to install the Python package Exudyn is to use the so-called 'wheels' (file ending `.whl`).
NOTE that this approach usually is not required; usually, just use the pip installer of the previous section!

Wheels can be downloaded directly from
[https://pypi.org/project/exudyn/\#files](https://pypi.org/project/exudyn/\#files), one per
Python version and platform. The file name says which one it is: `cp313` means CPython 3.13,
and the last part names the platform.

 Check which Python you are about to install into:

- `python -{}-version`

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
### Build and install Exudyn under Windows 10

Note that there are a couple of pre-requisites, depending on your system and installed libraries. For Windows 10, the following steps proved to work:

- you need an appropriate compiler (Microsoft Visual Studio 2022; see {ref}`sec-install-installinstructions-requirements`)
- install your Anaconda distribution including Spyder or use VisualStudioCode; you can also use a miniconda (only numpy is really required and distribution tools)
- close all Python programs (e.g. Spyder, Jupyter, ...)
- run an Anaconda prompt (may need to be run as administrator, depends on installation)
- it is recommended to use conda environments!
- go to 'main' of your cloned github folder of Exudyn
- run: (Since version 1.7.116 a PEP518 compatible build is available. This should work with Windows, MacOS and linux. The `setupPyConfig.json` file includes some flags such as the parallel compilation, GLFW, etc.; the `-v` flag adds verbosity.) `pip wheel . -v -w dist --no-deps`
- Before version 1.7.116: run: (the `--parallel` option performs parallel compilation on multithreaded CPUs and can speedup by 2x - 8x) `python setup.py install --parallel`
- read the output; if there are errors, try to solve them by installing appropriate modules

You can also create your own wheels, doing the above steps to activate the according Python version and then calling:

- `python setup.py bdist_wheel --parallel`

This will add a wheel in the `dist` folder.

(sec-install-installinstructions-buildmacos)=
### Build and install Exudyn under Mac OS X

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
- `python setup.py install`
- $\ra$ this will only install, but not re-compile. Otherwise, just use pip install from the created wheel in the dist folder
- **NOTE** that conda environments are highly recommended

Then just go to the `pythonDev/Examples` folder and run an example:

- `python springDamperUserFunctionTest.py`

If there are other issues, we are happy to receive your detailed bug reports.

 Note that you need to run

```python
  SC.renderer.Start()
  SC.renderer.DoIdleTasks()
```

in order to interact with the render window, as there is only a single-threaded version available for Mac OS.

(sec-install-installinstructions-buildubuntu)=
### Build and install Exudyn under Ubuntu

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

If graphics is used (`\#define USE_GLFW_GRAPHICS` in `BasicDefinitions.h`), you must install the according GLFW libs:

```python
  sudo apt-get install libglfw3 libglfw3-dev
```

In some cases, it may be required to install OpenGL and some of the following libraries:

```python
  sudo apt-get install freeglut3 freeglut3-dev
  sudo apt-get install mesa-common-dev
  sudo apt-get install libx11-dev xorg-dev libglew1.5 libglew1.5-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev
```

With all of these libs, you can run the setup.py installer (go to `Exudyn_git/main` folder), which takes some minutes for compilation (the --user option is used to install in local user folder) (the `--parallel` option performs parallel compilation on multithreaded CPUs and can speedup by 2x - 8x):

```python
  sudo python3 setup.py install --user --parallel
```

Since version 1.7.116, a PEP518 compatible way to compile sources and install the current repository has been added (the `-v` flag activates a verbose mode):

```python
  pip install . -v --no-deps
```

Congratulation! **Now, run a test example** (will also open an OpenGL window if successful):

- `python3 pythonDev/Examples/rigid3Dexample.py`

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

- Exudyn also compiles under RaspberryPi 4b, Ubuntu Mate 20.04, Python 3.8; current version should compile out of the box using `python3 setup.py install` command.
- Performance is quite ok and it is even capable to use all cores (but you should add a fan!)
- $\ra$ this could be used for a nice realtime application!

 **KNOWN issues for linux builds**:

- Using **WSL2** (Windows subsystem for linux), there occur some conflicts during build because of incompatible windows and linux file systems and builds will not be copied to the dist folder; workaround: go to explorer, right click on 'build' directory and set all rights for authenticated user to 'full access'
- **compiler (gcc,g++) conflicts**: It seems that Exudyn works well on Ubuntu 18.04 with the original `Python 3.6.9` and `gcc-7.5.0` version as well as with Ubuntu 20.04 with `Python 3.8.5` and `gcc-9.3.0`. Upgrading `gcc` on a Linux system with Python 3.6 to, e.g., `gcc-8.2` showed us a linker error when loading the Exudyn module in Python -- there are some common restriction using `gcc` versions different from those with which the Python version has been built. Starting `python` or `python3` on your linux machine shows you the `gcc` version it had been build with. Check your current `gcc` version with: `gcc --version`

(sec-install-installinstructions-uninstall)=
### Uninstall Exudyn

To uninstall exudyn under Windows, run (may require admin rights):

- `pip uninstall exudyn`

 To uninstall under Ubuntu, run:

- `sudo pip3 uninstall exudyn`

If you upgrade to a newer version, uninstall is usually not necessary!

### How to install Exudyn and use the C++ source code (advanced)?

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

## Further notes

(sec-install-notes-goals)=
### Goals of Exudyn

After the first development phase (2019-2023), it

- is a moderately large  (wheels have only sizes of 2MB on Windows and 4MB on Linux, without fast Exudyn options) multibody library, which can be easily linked to other projects,
- contains basic multibody rigid bodies, flexible bodies, joints, contact, etc.,
- includes a large Python utility library for convenient building and post processing of models,
- allows to efficiently simulate small scale systems (compute $100\,000$s of time steps per second for systems with $n_{DOF}<10$),
- allows to efficiently simulate medium scaled systems for problems with $n_{DOF} < 1\,000\,000$,
- is a safe and widely accessible module for Python,
- allows to add user defined objects and solvers in C++,
- allows to add user defined objects and solvers in Python,
- allows multi-threaded parallel computing,
- includes Lie group integration,
- includes interfaces for robotics and ROS,
- includes interfaces for reinforcement learning (stable-baselines3), pytorch and artificial intelligence,
- includes kinematical trees with minimal coordinates.

Future goals (2024-2026) are:

- add specific and advanced connectors/constraints (extended wheels, contact, control connector)
- automatic step size selection for second order solvers (planned 2025),
- add 3D beams and plates (first attempts exist; planned 2024),
- export equations (planned, 2025),
- add GPU support (planned, 2025).

For solved issues (and new features), see section 'Issues and Bugs', {ref}`sec-issuetracker`.
For specific open issues, see `trackerlog.html` -- a document only intended for developers!

(sec-install-simpleexample)=
## Run a simple example in Python

After performing the steps of the previous section, this section shows a simplistic model which helps you to check if Exudyn runs on your computer.

In order to start, run the Python interpreter Spyder (or any preferred Python environment).
In order to test the following example, which creates a {ref}`mbs <mbs>`, adds a node, an object, a marker and a load and simulates everything with default values,

- open `myFirstExample.py` from your `Examples` folder.

Hereafter, press the play button or `F5` in Spyder.

If successful, the IPython Console of Spyder will print something like:

```python
  runfile('C:/DATA/cpp/EXUDYN_git/python/Examples/myFirstExample.py',
    wdir='C:/DATA/cpp/EXUDYN_git/python/Examples')
  +++++++++++++++++++++++++++++++
  EXUDYN V1.2.9 solver: implicit second order time integration
  STEP100, t = 1 sec, timeToGo = 0 sec, Nit/step = 1
  solver finished after 0.0007824 seconds.
```

If you check your current directory (where `myFirstExample.py` lies), you will find a new file `coordinatesSolution.txt`, which contains the results of your computation (with default values for time integration).
The beginning and end of the file should look like: \

```python
  #Exudyn implicit second order time integration solver solution file
  #simulation started=2022-04-07,19:02:19
  #columns contain: time, ODE2 displacements, ODE2 velocities, ODE2 accelerations
  #number of system coordinates [nODE2, nODE1, nAlgebraic, nData] = [2,0,0,0]
  #number of written coordinates [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData] = [2,2,2,0,0,0,0]
  #total columns exported  (excl. time) = 6
  #number of time steps (planned) = 100
  #Exudyn version = 1.11.158.dev1; Python3.13.15; Windows x86_64 AVX2 FLOAT64
  #
  0,0,0,0,0,0.0001,0
  0.01,5e-09,0,1e-06,0,0.0001,0
  0.02,2e-08,0,2e-06,0,0.0001,0
  0.03,4.5e-08,0,3e-06,0,0.0001,0
  0.04,8e-08,0,4e-06,0,0.0001,0
  0.05,1.25e-07,0,5e-06,0,0.0001,0

  ...

  0.96,4.608e-05,0,9.6e-05,0,0.0001,0
  0.97,4.7045e-05,0,9.7e-05,0,0.0001,0
  0.98,4.802e-05,0,9.8e-05,0,0.0001,0
  0.99,4.9005e-05,0,9.9e-05,0,0.0001,0
  1,5e-05,0,0.0001,0,0.0001,0
  #simulation finished=2022-04-07,19:02:19
  #Solver Info: stepReductionFailed(or step failed)=0,discontinuousIterationSuccessful=1,newtonSolutionDiverged=0,massMatrixNotInvertible=1,total time steps=100,total Newton iterations=100,total Newton jacobians=100
```

Within this file, the first column shows the simulation time and the following columns provide coordinates, their derivatives and Lagrange multipliers on system level. For relation of local to global coordinates, see {ref}`sec-overview-ltgmapping`. As expected, the $x$-coordinate of the point mass has constant acceleration $a=f/m=0.001/10=0.0001$, the velocity grows up to $0.0001$ after 1 second and the point mass moves $0.00005$ along the $x$-axis.

Note that line 8 contains the Exudyn and Python versions (as well as some other specific information on the platform and compilation settings (which may help you identify with which computer, etc., you created results)) provided in the solution file are the versions at which Exudyn has been compiled with.
The Python micro version (last digit) may be different from the Python version from which you were running Exudyn.
This information is also provided in the sensor output files.

(sec-install-troubleshootingfaq)=
## Trouble shooting and FAQ

### Trouble shooting

Exudyn has a solid exception handling and does lots of error checking on
inputs as well as during computations. This should usually not lead to an
unexpected crash as in early days of scientific codes.

For basic information on exception handling, see also the according section on
Exceptions and Error Messages. In the following, typical error messages are listed.

 **Python import errors**:

- Sometimes the Exudyn module cannot be loaded into Python. Typical **error messages if Python versions are not compatible** are: \ Typical **error messages if 32/64 bits versions are mixed**:\ **There are several reasons and workarounds**:

  ```
    Traceback (most recent call last):

      File "<ipython-input-14-df2a108166a6>", line 1, in <module>
        import exudynCPP

    ImportError: Module use of python36.dll conflicts with this version of Python.
  ```
  ```
    Traceback (most recent call last):

      File "<ipython-input-2-df2a108166a6>", line 1, in <module>
        import exudynCPP

    ImportError: DLL load failed: %1 is not a valid Win32 application.
  ```
  - You mixed up 32 and 64 bits version (see below)
  - You are using an exudyn version for Python $x_1.y_1$ (e.g., 3.13.$z_1$) different from the Python $x_2.y_2$ version in your Anaconda (e.g., 3.7.$z_2$); note that $x_1=x_2$ and $y_1=y_2$ must be obeyed while $z_1$ and $z_2$ may be different

- **Import of exudyn C++ module failed Warning: ...**:

  - ... and similar messages with: ModuleNotFoundError, Warning, with AVX2, without AVX2
  - A known reason is that your CPU **does not support AVX2**, while Exudyn is compiled with the AVX2 option (modern Intel Core-i3, Core-i5 and Core-i7 processors as well as AMD processors, especially Zen and Zen-2 architectures should have no problems with AVX2; however, low-cost Celeron, Pentium and older AMD processors do **not** support AVX2, e.g.,  Intel Celeron G3900, Intel core 2 quad q6600, Intel Pentium Gold G5400T; check the system settings of your computer to find out the processor type; typical CPU manufacturer pages or Wikipedia provide information on this).
  - **solution**: since Exudyn 2.0 there is nothing to do: the regular module `exudynCPP` is compiled for the baseline instruction set on every platform and runs on a CPU without AVX2. Only the optional `exudynCPPfast` module, which you get by setting `sys.exudynFast = True` before importing Exudyn, uses AVX2; that request is silently ignored on a CPU which does not support it. The separate `exudynCPPnoAVX` module and the `sys.exudynCPUhasAVX2` switch no longer exist.
  - you can also compile for your specific Python version without AVX if you adjust the `setup.py` file in the `main` folder.
  - The `ModuleNotFoundError` may also happen if something went wrong during installation (paths, problems with Anaconda, ..) $\ra$ very often a new installation of Anaconda and Exudyn helps.

 **Typical Python errors**:

- Typical Python **syntax error** with missing braces:

  ```
    File "C:\DATA\cpp\EXUDYN_git\python\Examples\springDamperTutorial.py", line 42
        nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
               ^
    SyntaxError: invalid syntax
  ```

- such an error points to the line of your code (line 42), but in fact the error may have been caused in previous code, such as in this case there was a missing brace in the line 40, which caused the error:

  ```python
    38  n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0],
    39                       initialCoordinates = [u0,0,0],
    40                       initialVelocities= [v0,0,0])
    41  #ground node
    42  nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
    43
  ```

- Typical Python **import error** message on Linux / Ubuntu if Python modules are missing:

  ```
    Python WARNING [file '/home/johannes/.local/lib/python3.13/site-packages/exudyn/solver.py', line 236]:
    Error when executing process ShowVisualizationSettingsDialog':
    ModuleNotFoundError: No module named 'tkinter'
  ```

- see installation instructions to install missing Python modules, {ref}`sec-install-installinstructions`.
- Problems with **tkinter**, especially on MacOS:\ Exudyn uses `tkinter`, based on tcl/tk, to provide some basic dialogs, such as visualizationSettings\ As Python is not suited for multithreading, this causes problems in window and dialog workflows. Especially on MacOS `tkinter` is less stable and compatible with the window manager. Especially, `tkinter` already needs to run before the application's OpenGL window (renderer) is opened. Therefore, on MacOS `tkinter.Tk()` is called before the renderer is started. In some cases, visualizationSettings dialog may not be available and changes have to be made inside the code.
- To resolve issues, the following visualizationSettings may help (before starting renderer!), but may reduce functionality: dialogs.multiThreadedDialogs = False, general.useMultiThreadedRendering = False

 **Typical solver errors**:

- Consider the  example for a mixed error message comes from the solver when called for a (possibly empty) `mbs` with no prior call to `mbs.Assemble()`:

  ```
    import exudyn as exu
    from exudyn.utilities import *
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    sims=exu.SimulationSettings()
    exu.SolveDynamic(mbs, sims)
  ```

- This will results in error messages similar to:

  ```
    =========================================
    User ERROR [file 'C:\Users\username\.conda\envs\venvP39\lib\site-packages\exudyn\solver.py', line 245]:
    Solver: system is inconsistent and cannot be solved (call Assemble() and check error messages)
    =========================================

    =========================================
    SYSTEM ERROR [file 'C:\Users\username\.conda\envs\venvP39\lib\site-packages\exudyn\solver.py', line 245]:
    EXUDYN raised internal error in 'CSolverBase::InitializeSolver':
    Exudyn: parsing of Python file terminated due to Python (user) error
    =========================================

    ******************************
    DYNAMIC SOLVER FAILED:
      use showHints=True to show helpful information
    ******************************

    Traceback (most recent call last):

      File "C:\Users\username\AppData\Local\Temp\ipykernel_24988\3348856385.py", line 1, in <module>
        exu.SolveDynamic(mbs, sims)

      File "C:\Users\username\.conda\envs\venvP39\lib\site-packages\exudyn\solver.py", line 255, in SolveDynamic
        raise ValueError("SolveDynamic terminated")

    ValueError: SolveDynamic terminated
  ```
  - it seems clear that you should read this error from top as it indicates that you just forgot to call `mbs.Assemble()`

- WSL/Ubuntu: render window crashes after left or right mouse click:

  - This happens, in some (all?) Linux installations during mouse selection
  - workaround: setting `SC.visualizationSettings.interactive.advanced.selectionLeftMouse = False` and `SC.visualizationSettings.interactive.advanced.selectionRightMouse = False` removes the option to select with mouse, but avoids crashes

- `SolveDynamic` or `SolveStatic` **terminated due to errors**:

  - use flag `showHints = True` in `SolveDynamic` or `SolveStatic`

- Very simple example **without loads** leads to error: `SolveDynamic` or `SolveStatic` **terminated due to errors**:

  - see also 'Convergence problems', {ref}`sec-overview-basics-convergenceproblems`
  - may be caused due to nonlinearity of formulation and round off errors, which restrict Newton to achieve desired tolerances; adjust  `.newton.relativeTolerance` / `.newton.absoluteTolerance` in static solver or in time integration

- Typical **solver error due to redundant constraints or missing inertia terms**, could read as follows:   which draws the according object in red and others gray/transparent (but sometimes objects may be hidden inside other objects!). See the command's description for further options, e.g., to highlight nodes. \

  ```
    =========================================
    SYSTEM ERROR [file 'C:\ProgramData\Anaconda3_64b37\lib\site-packages\exudyn\solver.py', line 207]:
    CSolverBase::Newton: System Jacobian seems to be singular / not invertible!
    time/load step #1, time = 0.0002
    causing system equation number (coordinate number) = 42
    =========================================
  ```
  - this solver error shows that equation 42 is not solvable. The according coordinate is shown later in such an error message:
  ```python
    ...
    The causing system equation 42 belongs to a algebraic variable (Lagrange multiplier)
    Potential object number(s) causing linear solver to fail: [7]
        object 7, name='object7', type=JointGeneric
  ```
  - object 7 seems to be the reason, possibly there are too much (joint) constraints applied to your system, check this object.
  - show typical REASONS and SOLUTIONS, by using `showHints=True` in `exu.SolveDynamic(...)` or `exu.SolveStatic(...)`
  - You can also **highlight** object 7 by using the following code in the iPython console:
  ```python
    SC.renderer.Start()
    HighlightItem(SC,mbs,7)
  ```

- Typical **solver error if Newton does not converge**:

  ```
    +++++++++++++++++++++++++++++++
    EXUDYN V1.0.200 solver: implicit second order time integration
      Newton (time/load step #1): convergence failed after 25 iterations; relative error = 0.079958, time = 2
      Newton (time/load step #1): convergence failed after 25 iterations; relative error = 0.0707764, time = 1
      Newton (time/load step #1): convergence failed after 25 iterations; relative error = 0.0185745, time = 0.5
      Newton (time/load step #2): convergence failed after 25 iterations; relative error = 0.332953, time = 0.5
      Newton (time/load step #2): convergence failed after 25 iterations; relative error = 0.0783815, time = 0.375
      Newton (time/load step #2): convergence failed after 25 iterations; relative error = 0.0879718, time = 0.3125
      Newton (time/load step #2): convergence failed after 25 iterations; relative error = 2.84704e-06, time = 0.28125
      Newton (time/load step #3): convergence failed after 25 iterations; relative error = 1.9894e-07, time = 0.28125
    STEP348, t = 20 sec, timeToGo = 0 sec, Nit/step = 7.00575
    solver finished after 0.258349 seconds.
  ```
  - this solver error is caused, because the nonlinear system cannot be solved using Newton's method.
  - the static or dynamic solver by default tries to reduce step size to overcome this problem, but may fail finally (at minimum step size).
  - possible reasons are: too large time steps (reduce step size by using more steps/second), inappropriate initial conditions, or inappropriate joints or constraints (remove joints to see if they are the reason), usually within a singular configuration. Sometimes a system may be just unsolvable in the way you set it up.
  - see also 'Convergence problems', {ref}`sec-overview-basics-convergenceproblems`

- Typical solver error if (e.g., syntax) **error in user function** (output may be very long, **read always message on top!**):

  ```
    =========================================
    SYSTEM ERROR [file 'C:\ProgramData\Anaconda3_64b37\lib\site-packages\exudyn\solver.py', line 214]:
    Error in Python USER FUNCTION 'LoadCoordinate::loadVectorUserFunction' (referred line number my be wrong!):
    NameError: name 'sin' is not defined

    At:
      C:\DATA\cpp\DocumentationAndInformation\tests\springDamperUserFunctionTest.py(48): Sweep
      C:\DATA\cpp\DocumentationAndInformation\tests\springDamperUserFunctionTest.py(54): userLoad
      C:\ProgramData\Anaconda3_64b37\lib\site-packages\exudyn\solver.py(214): SolveDynamic
      C:\DATA\cpp\DocumentationAndInformation\tests\springDamperUserFunctionTest.py(106): <module>
      C:\ProgramData\Anaconda3_64b37\lib\site-packages\spyder_kernels\customize\spydercustomize.py(377): exec_code
      C:\ProgramData\Anaconda3_64b37\lib\site-packages\spyder_kernels\customize\spydercustomize.py(476): runfile
      <ipython-input-14-323569bebfb4>(1): <module>
      C:\ProgramData\Anaconda3_64b37\lib\site-packages\IPython\core\interactiveshell.py(3331): run_code
    ...
    ...
    ; check your Python code!
    =========================================

    Solver stopped! use showHints=True to show helpful information
  ```
  - this indicates an error in the user function `LoadCoordinate::loadVectorUserFunction`, because `sin` function has not been defined (must be imported, e.g., from `math`). It indicates that the error occurred in line 48 in `springDamperUserFunctionTest.py` within function `Sweep`, which has been called from function `userLoad`, etc.

### FAQ

**Some frequently asked questions**:

1. When **importing** Exudyn in Python (windows) I get an error

  - see trouble shooting instructions above!

2. I do not understand the **Python errors** -- how can I find the reason of the error or crash?

  - Read trouble shooting section above!
  - First, you should read all error messages and warnings: from the very first to the last message. Very often, there is a definite line number which shows the error. Note, that if you are executing a string (or module) as a Python code, the line numbers refer to the local line number inside the script or module.
  - If everything fails, try to execute only part of the code to find out where the first error occurs. By omiting parts of the code, you should find the according source of the error.
  - If you think, it is a bug: send an email with a representative code snippet, version, etc. to ` reply.exudyn@gmail.com`

3. Spyder **console hangs** up, does not show error messages, ...:

  - very often a new start of Spyder helps; most times, it is sufficient to restart the kernel or to just press the 'x' in your IPython console, which closes the current session and restarts the kernel (this is much faster than restarting Spyder)
  - restarting the IPython console also brings back all error messages

4. Where do I find the **'.exe' file**?

  - Exudyn is only available via the Python interface as a module '`exudyn`', the C++ code being inside of `exudynCPP.pyd`, which is located in the exudyn folder where you installed the package. This means that you need to **run Python** (best: Spyder) and import the Exudyn module.

5. I get the error message 'check potential mixing of different (object, node, marker, ...) indices', what does it mean?

  - probably you used wrong item indexes, see beginning of command interface in {ref}`sec-pcpp-command-interface`.
  - E.g., an object number `oNum = mbs.AddObject(...)` is used at a place where a `NodeIndex` is expected, e.g., `mbs.AddObject(MassPoint(nodeNumber=oNum, ...))`
  - Usually, this is an ERROR in your code, it does not make sense to mix up these indexes!
  - In the exceptional case, that you want to convert numbers, see beginning of {ref}`sec-pcpp-command-interface`.

6. Why does **type auto completion** / intellisense not work for mbs (MainSystem)?

  - in earlier versions of Exudyn type completion did not work properly for more complex structures
  - since version 1.6.103 type completion works for most functions\, types and structures: tested in Spyder 5.2.2 and Visual Studio Code 1.78.1); with an added stub file (.pyi) the standard type completion fetches information about structures or functions; this even works for example with `SC.visualizationSettings.bodies.kinematicTree.showJointFrames`. If you still have problems, try to restart your environment / computer or switch to a different version, and create an issue on GitHub.

7. How to add graphics?

  - Graphics (lines, text, 3D triangular / {ref}`STL <STL>` mesh) can be added to all BodyGraphicsData items in objects. Graphics objects which are fixed with the background can be attached to a ObjectGround object. Moving objects must be attached to the BodyGraphicsData of a moving body. Other moving bodies can be realized, e.g., by adding a ObjectGround and changing its reference with time. Furthermore, ObjectGround allows to add fully user defined graphics.

8. In `GenerateStraightLineANCFCable2D`

  - coordinate constraints can be used to constrain position and rotation, e.g., `fixedConstraintsNode0 = [1,1,0,1]` for a beam aligned along the global x-axis;
  - this **does not work** for beams with arbitrary rotation in reference configuration, e.g., 45°. Use a GenericJoint with a rotationMarker instead.

9. What is the difference between MarkerBodyPosition and MarkerBodyRigid?

  - Position markers (and nodes) do not have information on the orientation (rotation). For that reason, there is a difference between position based and rigid-body based markers. In case of a rigid body attached to ground with a SpringDamper, you can use both, MarkerBodyPosition or MarkerBodyRigid, markers. For a prismatic joint, you will need a MarkerBodyRigid.

10. I get an error in `exu.SolveDynamic(mbs, ...)` OR in `exu.SolveStatic(mbs, ...)` but no further information -- how can I solve it?

  - Typical **time integration errors** may look like:
  ```python
      File "C:/DATA/cpp/EXUDYN_git/python/...<file name>", line XXX, in <module>
      solver.SolveSystem(...)
      SystemError: <built-in method SolveSystem of PyCapsule object at 0x0CC63590> returned a result with an error set
  ```
  - The pre-checks, which are performed to enable a crash-free simulation are insufficient for your model
  - As a first try, **restart the IPython console** in order to get all error messages, which may be blocked due to a previous run of Exudyn.
  - Very likely, you are using Python user functions inside Exudyn: They lead to an internal Python error, which is not always catched by Exudyn; e.g., a load user function UFload(mbs,~t,~load), which tries to access component load[3] of a load vector with 3 components will fail internally;
  - Use the print(...) command in Python at many places to find a possible error in user functions (e.g., put `print("Start user function XYZ")` at the beginning of every user function; test user functions from iPython console
  - It is also possible, that you are using inconsistent data, which leads to the crash. In that case, you should try to change your model: omit parts and find out which part is causing your error
  - see also **I do not understand the Python errors -- how can I find the cause?**

11. Why can't I get the focus of the simulation window on startup (render window hidden)?

  - Starting Exudyn out of Spyder might not bring the simulation window to front, because of specific settings in Spyder(version 3.2.8), e.g., Tools$\ra$Preferences$\ra$Editor$\ra$Advanced settings: uncheck 'Maintain focus in the Editor after running cells or selections'; Alternatively, set `SC.visualizationSettings.view0.window.alwaysOnTop=True` **before** starting the renderer with `SC.renderer.Start()`
