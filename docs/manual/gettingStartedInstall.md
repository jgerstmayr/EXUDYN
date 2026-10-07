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

(sec-install-ubuntu)=
### Working with Ubuntu

- **Smooth fonts in the dialogs with conda**: the `tk` package of the Anaconda channel is built without Xft, so the
  dialogs draw jagged fonts; Exudyn says so once when the first dialog opens. The Xft build of conda-forge draws them
  smoothly (checked on Ubuntu):

  ```bash
  conda install -c conda-forge "tk=*=xft_*"
  ```

  A system Python with `python3-tk` has Xft already.
- **Display scaling**: the scaling of the desktop, including *Fractional Scaling*, scales the texts of the render window
  and the dialogs (GLFW 3.3.6 or newer; Ubuntu 22.04 and later). To adjust by hand, while the renderer runs:
  `SC.visualizationSettings.general.displayScaleFactor` (a factor on the scaling of the system, for the texts of the
  render window), `window.globalFontSize` (the size of those texts) and `dialogs.fontScaling` (a factor on the font of
  every widget of the dialogs, relative to the scaling of the system; 0 = 1).
- **Videos from recorded frames**: `conda install ffmpeg` and `pip install ffmpeg-python`, see
  {ref}`sec-overview-basics-animations`.

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

In some cases (e.g. for AppleM1 or special Linux versions), your pre-built binary will not work due to some incompatibilities. Then you need to build from source as described in {ref}`sec-dev-build`.

### Troubleshooting pip install

Pip install may fail, if your linux version does not support the current manylinux version.
This was known for Red Hat, CentOS, Rocky Linux or simlilar systems which usually support manylinux2014. In this case, you had to build Exudyn from source, see {ref}`sec-dev-build`. Since version 1.7.116, the manylinux2014 version is supported and according problems should be solved.

Sometimes, you install exudyn, but when running python, the `import exudyn` fails.
In case of several environments, check where your installation goes. To guarantee that the pip install goes to the python call, use:

- `python -m pip install exudyn`

which ensures that the used python is calling its associated pip module.

If the PyPi index is not updated, it may help to use

- `pip install -i https://pypi.org/project/ exudyn`

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

To uninstall exudyn under Windows, run (may require admin rights):

- `pip uninstall exudyn`

 To uninstall under Ubuntu, run:

- `sudo pip3 uninstall exudyn`

If you upgrade to a newer version, uninstall is usually not necessary!

