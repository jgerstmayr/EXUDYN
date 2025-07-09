.. _sec-install-installinstructions:


Installation instructions
=========================


.. _sec-install-installinstructions-requirements:


Requirements for Exudyn ?
-------------------------


Exudyn only works with Python. Thus, you need an appropriate Python installation.
So far (2025-07), we thoroughly tested

+  Currently (2025-07), we support Python 3.9 - Python 3.13 \ **conda environments**\  on Windows, Linux and MacOS (please report configurations that do not work on GitHub/issues); the upgrades to newer Python versions will usually be done in line with new Python releases (every October).
+  \ **Anaconda 2025-06, 64bit, Python 3.12**\  with Spyder 6.0
+  \ **Anaconda 2023-07, 64bit, Python 3.11**\  with Spyder 5.4
+  \ **Anaconda 2021-11, 64bit, Python 3.9**\ \ (older Anaconda3 versions can be downloaded via the repository archive \ ``https://repo.anaconda.com/archive/``\ )
+  \ **Spyder 5.1.5**\  (with Python 3.9.7, 64bit) and \ **Spyder 4.1.3**\  (with Python 3.7.7, 64bit), which is included in the Anaconda installation\ (Note that it is important that Spyder, Python and Exudyn  are \ **either**\  32bit \ **or**\  64bit and are compiled up to the same minor version, i.e., 3.7.x. There will be a strange .DLL error, if you mix up 32/64bit. It is possible to install both, Anaconda 32bit and Anaconda 64bit -- then you should follow the recommendations of paths as suggested by Anaconda installer.); Spyder works with all virtual environments

Many alternative options exist:

+  Many users report successful use of Exudyn with \ **Visual Studio Code**\ . \ **Jupyter**\  has been tested with some examples; both environments should work with default settings.
+  Alternative option with more stable Spyder (as compared to Spyder 4.1.3): Anaconda, 64bit, Python 3.6.5)\ (Anaconda 64bit with Python3.6 can be downloaded via the repository archive \ ``https://repo.anaconda.com/archive/``\  choosing \ ``Anaconda3-5.2.0-Windows-x86_64.exe``\  for 64bit.)

If you plan to extend the C++ code, we recommend to use Microsoft Visual Studio (VS2022) and previously VS2017\ (Note: VS2019 has problems with the library 'Eigen' and therefore leads to erroneous results with the sparse solver.) to compile your code, which offers Python 3.x compatibility.
Remember that Python versions and the version of the Exudyn module must be identical (e.g., Python 3.9 64 bit \ **both**\  in the Exudyn module and e.g. in Spyder).


Run without Anaconda
^^^^^^^^^^^^^^^^^^^^

If you do not install Anaconda (e.g., under Linux), make sure that you have the according Python packages installed:

+  \ ``numpy``\  (used throughout the code, inevitable)
+  \ ``matplotlib``\  (for any plot, also PlotSensor(...))
+  \ ``tkinter``\  (for interactive dialogs, SolutionViewer, etc.)
+  \ ``scipy``\  (needed for eigenvalue computation)

You can install most of these packages using \ ``pip install numpy``\  (Windows) or \ ``pip3 install numpy``\  (Linux).
NOTE: as there is only \ ``numpy``\  needed (but not for all sub-packages) and \ ``numpy``\  supports many variants, we do not add a particular requirement for installation of depending packages. It is not necessary to install \ ``scipy``\  as long as you are not using features of \ ``scipy``\ . Same reason for \ ``tkinter``\  and \ ``matplotlib``\ .

For interaction (right-mouse-click, some key-board commands) you need the Python module \ ``tkinter``\ . This is included in regular Anaconda distributions (recommended, see below), but on Ubuntu you need to type alike (do not forget the '3', otherwise it installs for Python2 ...):

   \ ``sudo apt-get install python3-tk``\ 

see also common blogs for your operating system.


.. _sec-install-installinstructions-pipinstall:


Install Exudyn with PIP INSTALLER (pypi.org)
--------------------------------------------

Pre-built versions of Exudyn are hosted on \ ``pypi.org``\ , see the project

 +  `https://pypi.org/project/exudyn <https://pypi.org/project/exudyn>`_

As with most other packages, in the regular case (if your binary has been pre-built) you just need to do

   \ ``pip install exudyn``\ 

On Ubuntu/Linux, make sure that pip is installed and up-to-date (\ **update pip to at least 20.3**\ ; otherwise the manylinux wheels will not be accepted!):
 
   \ ``sudo apt install python3-pip``\ 
   \ ``python3 -m pip install --upgrade``\ 

Depending on installation the command may read \ ``pip3``\  or \ ``pip``\ : 

   \ ``pip3 install exudyn``\ 

For pre-releases (use with care!), add \ ``--pre``\  flag:

   \ ``pip install exudyn --pre -U``\ 

The \ ``-U``\  (identical to \ ``--upgrade``\ ) flag ensures that the current installed version is also updated in case of a change of the micro version (e.g., from version 1.6.119 to version 1.6.164), otherwise, it will only update if you switch to a newer minor version.

In some cases (e.g. for AppleM1 or special Linux versions), your pre-built binary will not work due to some incompatibilities. Then you need to build from source as described in the 'Build and install' sections, Section :ref:`sec-install-installinstructions-buildwindows`\ .


Troubleshooting pip install
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Pip install may fail, if your linux version does not support the current manylinux version.
This was known for Red Hat, CentOS, Rocky Linux or simlilar systems which usually support manylinux2014. In this case, you had to build Exudyn from source, see Section :ref:`sec-install-installinstructions-buildubuntu`\ . Since version 1.7.116, the manylinux2014 version is supported and according problems should be solved.

Sometimes, you install exudyn, but when running python, the \ ``import exudyn``\  fails.
In case of several environments, check where your installation goes. To guarantee that the pip install goes to the python call, use:
 
   \ ``python -m pip install exudyn``\  

which ensures that the used python is calling its associated pip module.

If the PyPi index is not updated, it may help to use 
 
   \ ``pip install -i https://pypi.org/project/ exudyn``\ 



.. _sec-install-installinstructions-wheel:


Install from specific Wheel (Ubuntu and Windows)
------------------------------------------------

A way to install the Python package Exudyn is to use the so-called 'wheels' (file ending \ ``.whl``\ ).
NOTE that this approach usually is not required; usually, just use the pip installer of the previous section!

\ **Ubuntu**\ :

Wheels can be downloaded directly from `https://pypi.org/project/exudyn/\#files <https://pypi.org/project/exudyn/\#files>`_, for many Python versions and architectures.


For Ubuntu 18.04 (which by default uses Python 3.6) this may read (version number 1.0.20 may be different):

+  \ ``Python 3.6, 64bit``\ : pip3 install dist\ :math:`\backslash`\ exudyn-1.0.20-cp36-cp36-linux_x86_64.whl

For Ubuntu 20.04 (which by default uses Python 3.8) this may read (version number 1.0.20 may be different):

+  \ ``Python 3.8, 64bit``\ : pip3 install dist\ :math:`\backslash`\ exudyn-1.0.20-cp38-cp38-linux_x86_64.whl

We regularly upgrade to newer Ubuntu versions (like 24.04) and we did not observe any issues.
NOTE that your installation may have environments with different Python versions, so install that Exudyn version appropriately!
If the wheel installation does not work on Ubuntu, it is highly recommended to build Exudyn for your specific system as given in Section :ref:`sec-install-installinstructions-buildubuntu`\ .

\ **Windows**\ :

First, open an Anaconda prompt:

+  EITHER calling the Anaconda prompt directly using: START->Anaconda->... OR go to anaconda/Scripts folder and call activate.bat
+  You can check your Python version then, by running \ ``python``\ \ (\ ``python3``\  under Ubuntu 18.04), the output reads like:
  
   \ ``Python 3.6.5 | Anaconda, Inc.| (default, Mar 29 2018, 13:32:41) [ MSC v.1900 64 bit (AMD64)] on win32``\ 
   ...
  
+  type \ ``exit()``\  to close Python

For Windows the installation commands may read (version number 1.0.20 may be different):

+  \ ``Python 3.6, 32bit``\ : pip install dist\ :math:`\backslash`\ exudyn-1.0.20-cp36-cp36m-win32.whl
+  \ ``Python 3.6, 64bit``\ : pip install dist\ :math:`\backslash`\ exudyn-1.0.20-cp36-cp36m-win_amd64.whl
+  \ ``Python 3.7, 64bit``\ : pip install dist\ :math:`\backslash`\ exudyn-1.0.20-cp37-cp37m-win_amd64.whl





.. _sec-install-installinstructions-buildwindows:


Build and install Exudyn under Windows 10
-----------------------------------------

Note that there are a couple of pre-requisites, depending on your system and installed libraries. For Windows 10, the following steps proved to work:

+  you need an appropriate compiler (tested with Microsoft Visual Studio; recommended: VS2017)
+  install your Anaconda distribution including Spyder
+  close all Python programs (e.g. Spyder, Jupyter, ...) 
+  run an Anaconda prompt (may need to be run as administrator)
+  if you cannot run Anaconda prompt directly, do:
  
+  open windows shell (cmd.exe) as administrator (START \ :math:`\ra`\  search for cmd.exe \ :math:`\ra`\  right click on app \ :math:`\ra`\  'run as administrator' if necessary) [may not be necessary]
+  go to your Scripts folder inside the Anaconda folder (e.g. \ ``C:\ProgramData\Anaconda\Scripts``\ ) [may not be necessary]
+  run 'activate.bat' [may not be necessary]
  
+  go to 'main' of your cloned github folder of Exudyn 
+  run:\ (Since version 1.7.116 a PEP518 compatible build is available. This should work with Windows, MacOS and linux. The \ ``setupPyConfig.json``\  file includes some flags such as the parallel compilation, GLFW, etc.; the \ ``-v``\  flag adds verbosity.) \ ``pip wheel . -v -w dist --no-deps``\ 
+  Before version 1.7.116: run:\ (the \ ``--parallel``\  option performs parallel compilation on multithreaded CPUs and can speedup by 2x - 8x) \ ``python setup.py install --parallel``\ 
+  read the output; if there are errors, try to solve them by installing appropriate modules

You can also create your own wheels, doing the above steps to activate the according Python version and then calling:

   \ ``python setup.py bdist_wheel --parallel``\ 

This will add a wheel in the \ ``dist``\  folder.


.. _sec-install-installinstructions-buildmacos:


Build and install Exudyn under Mac OS X
---------------------------------------

Installation and building on Mac OS X is less frequently tested, but successful compilation including GLFW has been achieved.
Requirements are an according Anaconda (or Miniconda) installation.

\ **Tested configurations**\ :

+  Mac OS 11.x 'Big Sur', Mac Mini (2021), Apple M1, 16GB Memory
+  Miniconda with conda environments (x86 / i368 based with Rosetta 2) with Python 3.7 - 3.11
+  Miniconda with conda environments (ARM) with Python 3.8 - 3.11
   \ :math:`\ra`\  wheels are available on pypi since Exudyn 1.5.0 

\ **NOTE**\ :

+  New \ ``universal2``\  wheels should support x86 (APPLE Intel and Python/Rosetta on APPLE Silicon)
+  Multi-threading is not fully supported on MacOS, but may work in some applications
+  On Apple M1 processors the newest Anaconda supports now all required features; environments with Python 3.8-3.11 have been successfully tested;
+  The Rosetta (x86 emulation) mode on Apple M1 also works now without much restrictions; these files should also work on older Macs
+  If you have a MacOS version\ :math:`<11`\ , it worked to download wheels from PyPI, change wheel names, e.g., from \ ``exudyn-1.7.116.dev1-cp311-cp311-macosx_11_0_x86_64.whl``\  to \ ``exudyn-1.7.116.dev1-cp311-cp311-macosx_10_9_x86_64.whl``\ . This also works for universal2 files. Installation worked and wheels were running smoothly.
+  \ ``tkinter``\  has been adapted (some workarounds needed on MacOS!), available since Exudyn 1.5.15.dev1
+  Some optimization and processing functions do not run (especially multiprocessing and tqdm); 


Alternatively, we tested on:

+  Mac OS X 10.11.6 'El Capitan', Mac Pro (2010), 3.33GHz 6-Core Intel Xeon, 4GB Memory, Anaconda Navigator 1.9.7, Python 3.7.0, Spyder 3.3.6


\ **Compile from source**\ :

If you would like to compile from source, just use a bash terminal on your Mac, and do the following steps inside the \ ``main``\  directory of your repository and type

+  uninstall if old version exists (may need to repeat this!): \ ``pip uninstall exudyn``\ 
+  remove the \ ``build``\  directory if you would like to re-compile without changes
+  to perform compilation from source, write:\ (the \ ``--parallel``\  option performs parallel compilation on multithreaded CPUs and can speedup by 2x - 8x)
+  Since version 1.7.116: \ ``pip wheel . -v -w dist --no-deps``\ 
+  Until version 1.7.116: \ ``python setup.py bdist_wheel --parallel``\ 
+  which takes 75 seconds on Apple M1 in parallel mode, otherwise 5 minutes. To install Exudyn, run
   \ ``python setup.py install``\ 
   \ :math:`\ra`\  this will only install, but not re-compile. Otherwise, just use pip install from the created wheel in the dist folder
   \ **NOTE**\  that conda environments are highly recommended

Then just go to the \ ``pythonDev/Examples``\  folder and run an example:

   \ ``python springDamperUserFunctionTest.py``\ 

If there are other issues, we are happy to receive your detailed bug reports. 

Note that you need to run 

.. code-block:: python

  SC.renderer.Start()
  SC.renderer.DoIdleTasks()


in order to interact with the render window, as there is only a single-threaded version available for Mac OS.


.. _sec-install-installinstructions-buildubuntu:


Build and install Exudyn under Ubuntu
-------------------------------------

Having a new Ubuntu 18.04 standard installation (e.g. using a VM virtual box environment), the following steps need to be done (Python \ **3.6**\  is already installed on Ubuntu 18.04, otherwise use \ ``sudo apt install python3``\ )\ (see also the youtube video: `https://www.youtube.com/playlist?list=PLZduTa9mdcmOh5KVUqatD9GzVg_jtl6fx <https://www.youtube.com/playlist?list=PLZduTa9mdcmOh5KVUqatD9GzVg_jtl6fx>`_):

First update ...


.. code-block:: 

  sudo apt-get update




Install necessary Python libraries and pip3; \ ``matplotlib``\  and \ ``scipy``\  are not required for installation but used in Exudyn examples:

.. code-block:: 

  sudo dpkg --configure -a
  sudo apt install python3-pip
  pip3 install numpy
  pip3 install matplotlib
  pip3 install scipy



Install pybind11 (needed for running the setup.py file derived from the pybind11 example):

.. code-block:: 

  pip3 install pybind11



To have dialogs enabled, you need to install \ ``Tk``\ /\ ``tkinter``\  (may be already installed in your case). 
\ ``Tk``\  is installed on Ubuntu via apt-get and should then be available in Python:

.. code-block:: 

  sudo apt-get install python3-tk




If graphics is used (\ ``\#define USE_GLFW_GRAPHICS``\  in \ ``BasicDefinitions.h``\ ), you must install the according GLFW libs:

.. code-block:: 

  sudo apt-get install libglfw3 libglfw3-dev



In some cases, it may be required to install OpenGL and some of the following libraries:

.. code-block:: 

  sudo apt-get install freeglut3 freeglut3-dev
  sudo apt-get install mesa-common-dev
  sudo apt-get install libx11-dev xorg-dev libglew1.5 libglew1.5-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev




With all of these libs, you can run the setup.py installer (go to \ ``Exudyn_git/main``\  folder), which takes some minutes for compilation (the --user option is used to install in local user folder)\ (the \ ``--parallel``\  option performs parallel compilation on multithreaded CPUs and can speedup by 2x - 8x):

.. code-block:: 

  sudo python3 setup.py install --user --parallel


Since version 1.7.116, a PEP518 compatible way to compile sources and install the current repository has been added (the \ ``-v``\  flag activates a verbose mode):

.. code-block:: 

  pip install . -v --no-deps




Congratulation! \ **Now, run a test example**\  (will also open an OpenGL window if successful):

   \ ``python3 pythonDev/Examples/rigid3Dexample.py``\ 


You can also create a Ubuntu wheel which can be easily installed on the same machine (x64), same operating system (Ubuntu 18.04) and with same Python version (e.g., 3.6):

   \ ``sudo pip3 install wheel``\ 
   \ ``sudo python3 setup.py bdist_wheel --parallel``\ 


Since version 1.7.116, the PEP518 compatible way which puts wheels into the \ ``dist``\  folder reads:

   \ ``pip wheel . -v -w dist --no-deps``\ 



\ **Exudyn under Ubuntu / WSL**\ :

+  Note that Exudyn also nicely works under WSL (Windows subsystem for linux; tested for Ubuntu 18.04) and an according xserver (VcXsrv).
+  In case of old WSL2, just set the display variable in your .bashrc file accordingly and you can enjoy the OpenGL windows and settings.
+  It shall be noted that WSL + xserver works better than on MacOS, even for tkinter, multitasking, etc.! So, if you have troubles with your Mac, use a virtual machine with ubuntu and a xserver, that may do better
+  In case of WSLg (since 2021), only the software-OpenGL works; therefore, you have to set (possibly in .bashrc file): \ ``export LIBGL_ALWAYS_SOFTWARE=0``\ 


\ **Exudyn under RaspberryPi 4b**\ :

+  Exudyn also compiles under RaspberryPi 4b, Ubuntu Mate 20.04, Python 3.8; current version should compile out of the box using \ ``python3 setup.py install``\  command.
+  Performance is quite ok and it is even capable to use all cores (but you should add a fan!)
+  \ :math:`\ra`\  this could lead to a nice cluster project!


\ **KNOWN issues for linux builds**\ :

+  Using \ **WSL2**\  (Windows subsystem for linux), there occur some conflicts during build because of incompatible windows and linux file systems and builds will not be copied to the dist folder; workaround: go to explorer, right click on 'build' directory and set all rights for authenticated user to 'full access'
+  \ **compiler (gcc,g++) conflicts**\ : It seems that Exudyn works well on Ubuntu 18.04 with the original \ ``Python 3.6.9``\  and \ ``gcc-7.5.0``\  version as well as with Ubuntu 20.04 with \ ``Python 3.8.5``\  and \ ``gcc-9.3.0``\ . Upgrading \ ``gcc``\  on a Linux system with Python 3.6 to, e.g., \ ``gcc-8.2``\  showed us a linker error when loading the Exudyn module in Python -- there are some common restriction using \ ``gcc``\  versions different from those with which the Python version has been built. Starting \ ``python``\  or \ ``python3``\  on your linux machine shows you the \ ``gcc``\  version it had been build with. Check your current \ ``gcc``\  version with: \ ``gcc --version``\ 



.. _sec-install-installinstructions-uninstall:


Uninstall Exudyn
----------------


To uninstall exudyn under Windows, run (may require admin rights):

   \ ``pip uninstall exudyn``\ 

To uninstall under Ubuntu, run:

   \ ``sudo pip3 uninstall exudyn``\ 


If you upgrade to a newer version, uninstall is usually not necessary!


How to install Exudyn and use the C++ source code (advanced)?
-------------------------------------------------------------

Exudyn is still under intensive development of core modules.
There are several ways of using the code, but you \ **cannot**\  install Exudyn as compared to other executable programs and apps.


In order to make full usage of the C++ code and extending it, you can use:

+  Windows / Microsoft Visual Studio 2017 and above:
  
+  get the files from git
+  put them into a local directory (recommended: \ ``C:/DATA/cpp/EXUDYN_git``\ )
+  start \ ``main_sln.sln``\  with Visual Studio (recommended version: 2017, otherwise you have to manually adapt)
+  compile the code and run \ ``main/pythonDev/pytest.py``\  example code
+  adapt \ ``pytest.py``\  for your applications
+  extend the C++ source code
+  link it to your own code
+  NOTE: on Linux systems, you mostly need to replace '\ :math:`/`\ ' with '\ :math:`\backslash`\ '
  
+  Linux, etc.: Use the build methods described above; Visual Studio Code may allow native Python and C++ debugging; switching to other build mechanisms (CMakeLists or scikit-build-core).

