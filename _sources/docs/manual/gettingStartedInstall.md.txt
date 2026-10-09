(sec-install-installinstructions)=
# Installation instructions

(sec-install-installinstructions-requirements)=
## Requirements for Exudyn ?

Exudyn is a Python package with a compiled core, so what you need is a Python it was built for.

**Wheels are published for Python 3.10, 3.11, 3.12, 3.13 and 3.14**, on Windows, Linux and
MacOS. Newer Python versions are usually added in line with the Python release cycle (every
October).

**64-bit only.** Exudyn is not built for 32-bit systems. If you still read advice about
matching 32-bit and 64-bit installations, it is out of date.

- **conda environments** are what the developers use and what is tested most; the environment decides which Python you get, and `pip install exudyn` inside it installs the matching wheel.
- **Spyder** (any recent version), **Visual Studio Code** and **Jupyter** all work with default settings. Spyder works with all virtual environments.
- The Python running your script and the Python Exudyn was installed into must be the same one. In an environment that is automatic; outside one it is the usual source of `import exudyn` failing, see {ref}`sec-install-troubleshootingfaq`.

 If you plan to extend the **C++ code**, you need **Microsoft Visual Studio 2022** on
Windows: it is what compiles the sources, and it is the one debugger that steps from a Python
script into the C++ (VS2019 has problems with the library 'Eigen' and leads to erroneous results
with the sparse solver; it is not supported). The everyday development of Exudyn itself happens in
**Visual Studio Code**, which is also what most co-developers use; {ref}`sec-dev-gettingstarted`
sets both up.

(sec-install-installinstructions-pipinstall)=
## Install Exudyn with PIP INSTALLER (pypi.org)

Pre-built versions of Exudyn are hosted on `pypi.org`, see the project
[https://pypi.org/project/exudyn](https://pypi.org/project/exudyn). Install it with

- `pip install "exudyn[basic]"` - Exudyn with scipy and matplotlib, which most models need for eigenvalues, sparse
  matrices and plots: **recommended to start with**;
- `pip install "exudyn[common]"` - the above plus the packages of frequently used features: **to use most features**;
- `pip install exudyn` - Exudyn and numpy only.

The quotes keep shells such as zsh (macOS) from reading the brackets. What each set contains, and the larger ones,
is listed in {ref}`sec-install-extras`.

For pre-releases (use with care!), add `--pre`:

- `pip install exudyn --pre -U`

The `-U` (identical to `--upgrade`) flag ensures that the installed version is also updated for a new micro version
(e.g., from 1.6.119 to 1.6.164); otherwise, pip only updates when a newer minor version appears.

If something does not work, see the next section; if no pre-built version exists for your system (e.g. a special Linux),
build from source as described in {ref}`sec-dev-build`.

(sec-install-troubleshooting)=
### Troubleshooting pip install

- **pip or pip3**: depending on the installation, the command may read `pip3` instead of `pip`. `python -m pip install
  exudyn` is always right: it uses the pip of the Python that runs it.
- **`import exudyn` fails after the install**: with several environments, the installation went into another Python;
  `python -m pip install exudyn` installs into the Python you call, see also {ref}`sec-install-troubleshootingfaq`.
- **pip too old (Linux)**: the manylinux wheels need pip 20.3 or newer:
  `python -m pip install --upgrade pip`; a Python without pip gets it from the package manager,
  e.g. `apt install python3-pip`.
- **manylinux not supported**: Red Hat, CentOS, Rocky Linux and similar systems usually support manylinux2014, which
  Exudyn's wheels accept since 1.7.116; otherwise build from source, see {ref}`sec-dev-build`.
- **A new version does not show up** (a cached index, or a mirror configured for pip): `pip install --no-cache-dir
  --index-url https://pypi.org/simple -U exudyn` asks pypi.org itself.
- **sudo**: the commands on this page are written without `sudo`. The package manager of Linux (`apt ...`) needs it;
  pip needs it only when it installs into the global Python of the system, which should be avoided - use a conda or
  virtual environment instead.
- **Without Anaconda**: Exudyn needs numpy, which pip installs with it; the extras above add scipy and matplotlib. The
  dialogs (right mouse click, settings, SolutionViewer) need `tkinter`, which is part of Python on Windows and macOS and
  of the Anaconda Python; a Linux system Python needs the package `python3-tk` (`apt install python3-tk`).
- **Videos from recorded frames** need the program ffmpeg, on every platform: `conda install ffmpeg` and
  `pip install ffmpeg-python` (part of `[common]`), see {ref}`sec-overview-basics-animations`.

(sec-install-ubuntu)=
#### Linux: fonts of the dialogs and display scaling

- **Jagged fonts in the dialogs with conda**: the `tk` package of the Anaconda channel is built without Xft; Exudyn says
  so once when the first dialog opens. The Xft build of conda-forge draws smooth fonts:
  `conda install -c conda-forge "tk=*=xft_*"`. A system Python with `python3-tk` has Xft already.
- **Display scaling**: the scaling of the desktop, including *Fractional Scaling*, scales the texts of the render window
  and the dialogs (GLFW 3.3.6 or newer; Ubuntu 22.04 and later). To adjust by hand, while the renderer runs:
  `SC.visualizationSettings.general.displayScaleFactor` (a factor on the scaling of the system, for the texts of the
  render window), `window.globalFontSize` (the size of those texts) and `dialogs.fontScaling` (a factor on the font of
  every widget of the dialogs, relative to the scaling of the system; 0 = 1).

(sec-install-installinstructions-wheel)=
## Install from a specific wheel (Ubuntu and Windows)

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
{ref}`sec-dev-build`.

(sec-install-installinstructions-buildfromsource)=
## Build Exudyn from source

You need this only in two cases:

- there is **no wheel for your platform** - an unusual Linux, an old macOS, a Raspberry Pi;
- you want to **change the C++ core** itself.

Everything about it - what a build needs on each platform, the commands, what to do when the
build works and the import does not, and how to debug the C++ from a Python run - is in
{ref}`sec-dev-build`.

The short form, from the root of the cloned repository, with the environment you want to
install into active:

```bash
python -m pip wheel . -w dist --no-deps
pip install --force-reinstall --no-deps dist/<the wheel that was built>
```

It takes about a minute. On Linux the OpenGL and X11 development packages have to be
installed first, which is what {ref}`sec-dev-build` lists.

(sec-install-installinstructions-uninstall)=
## Uninstall Exudyn

To uninstall Exudyn, run

- `pip uninstall exudyn`

(with `sudo` only for the global Python of a Linux system, see {ref}`sec-install-troubleshooting`).

If you upgrade to a newer version, uninstall is usually not necessary!

