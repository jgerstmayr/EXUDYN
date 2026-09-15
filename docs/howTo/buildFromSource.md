Setup tools under windows in oder to generate python module from source files:

Description for testSetupTools example

====================================================================
WINDOWS:
requirements:
- installed Anaconda (tested with Python 3.6, 64bit on Anaconda3, 5.1.0)
- close Spyder or any other python application during these tasks:
- NOT NEEDED if according Visual Studio version is installed: conda install -c anaconda vs2015_runtime
  ==> takes a while!
- NOT NEEDED: pip install pybind11
- precompiled glfw library is provided in exudyn repo (glfw3.lib); opengl32.lib needed from visual studio

#build WINDOWS wheel (does not need admin rights!):
- make sure that wheel is installed in your python distribution: 
  pip install wheel
- open Anaconda prompt (either from START->Anaconda->... OR go to anaconda/Scripts folder and call activate.bat):
- got to your setup.py directory and build wheel distribution: 

#all commands for setup.py must be called in the main directory, e.g.:
cd C:\DATA\cpp\EXUDYN_git\main

#build wheel distribution:
python setup.py bdist_wheel
#==>creates,e.g.:
  dist/exu-0.0.1-cp36-cp36m-win_amd64.whl

#in order to install this wheel, e.g. python3.7 64bits, use:
pip install dist\exudyn-1.0.8-cp37-cp37m-win_amd64.whl
#  or python3.6 64bits, use:
pip install dist\exudyn-1.0.8-cp36-cp36m-win_amd64.whl
#  or python3.6 32bits, use:
pip install dist\exudyn-1.0.8-cp36-cp36m-win32.whl

#+++++++++++++++++++++++++++++++++++++++++++++++++
#build WINDOWS executable installer (will only work, if python is installed also in registry):
python setup.py bdist_wininst
will create an executable installer, foo-1.0.win32.exe, in the current directory.

#build WINDOWS MSI installer:
python setup.py bdist_msi

#+++++++++++++++++++++++++++++++++++++++++++++++++
#just for completeness: build and install python lib (==> not needed, bdist_wheel does everything):
python setup.py install 

#+++++++++++++++++++++++++++++++++++++++++++++++++
#to uninstall, and suppress prompt 'y/n', use:
pip uninstall -y exudyn


#+++++++++++++++++++++++++++++++++++++++++++++++++
#advanced for Windows:
#copy VS2017 .pyd file on top of files built by setup tools:
copy exudynCPP.pyd to build\lib.win-amd64-3.6
and rename accordingly

====================================================================
#+++++++++++++++++++++++++++++++++++++++++++++++++
UBUNTU / linux (tested with clean UBUNTU 18.04 installation, "Normal installation" and option enabled to 
    "Install third-party software for graphics ....", 
	agree to "update" after first boot,
	"VirtualBox Guest Additions installation" for better usage of virtual box ):

#preliminary steps (not all may be necessary on your specific installation)
sudo apt-get update
sudo dpkg --configure -a
sudo apt install python3-pip
pip3 install pybind11
pip3 install numpy

#alternatively:
sudo apt-get update && sudo dpkg --configure -a && sudo apt install python3-pip && pip3 install pybind11 && pip3 install numpy


#needed for some examples:
pip3 install matplotlib && pip3 install scipy

#+++++++++++++++++++++++++++++++++++++++++++++++++
#add glfw library (if graphics activated: main/src/Utilities/BasicDefinitions.h: #define USE_GLFW_GRAPHICS):
#install OpenGL libs and OpenGL headers (gets necessary includes in usr/include/GL)::
sudo apt-get install freeglut3 freeglut3-dev mesa-common-dev

#install glfw libs:
sudo apt-get install libglfw3 libglfw3-dev

#install X11 libs:
sudo apt-get install libx11-dev xorg-dev libglew1.5 libglew1.5-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev

### ALL ### glfw and X11 packages:
sudo apt-get install freeglut3 freeglut3-dev mesa-common-dev libglfw3 libglfw3-dev libx11-dev xorg-dev libglew1.5 libglew1.5-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev


#(recommendation, usually not needed:)
sudo apt-get install cmake libx11-dev xorg-dev libglu1-mesa-dev freeglut3-dev libglew1.5 libglew1.5-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev libglfw3-dev libglfw3
#### end graphics

#+++++++++++++++++++++++++++++++++++++++++++++++++

#libglfw.so and libGL.so [.x...] should be available under:
/usr/lib/x86_64-linux-gnu

use pip3 and python3 for any python3.x version !!!

#+++++++++++++++++++++++++++++++++++++++++++++++++
#install:
#change to exudyn_git/main
#install exudyn with sudo (run in main directory, where setup.py is located):
sudo python3 setup.py install

#use to install exudyn in local site-packages (still needs sudo?):
sudo python3 setup.py install --user

#test exudyn, e.g. by typing (still in main folder):
python3 pythonDev/Examples/rigid3Dexample.py

#use prebuilt objects: ?not tested?
CC="ccache gcc" python setup.py install

#remove local folders and files created during build/install:
sudo rm -r build dist exudyn.egg-info tmp .eggs

#uninstall from packages:
sudo pip3 uninstall exudyn
sudo pip3 uninstall -y exudyn

#+++++++++++++++++++++++++++++++++++++++++++++++++
#WHEELS:
#install wheel python package, if you do not have it:
sudo pip3 install wheel
#create ubuntu wheel:
sudo python3 setup.py bdist_wheel

#install ubuntu wheel with given wheel exudynXYZ.whl (python versions must match!!!)
pip3 install dist\exudynXYZ.whl
#for version 1.0.8, this reads:
pip3 install dist\exudyn-1.0.8-cp36-cp36m-linux_x86_64.whl

#see also: "history on UBUNTU18.04 for new installation" in this file

#+++++++++++++++++++++++++++
#using setup tools / creating wheels with WSL:
#there are problems with file access
==> go right click on root folder->properties->security
for Authenticated Users->Edit-> set "Full control"
python3 setup.py bdist_wheel

====================================================================

====================================================================
steps to build and bind the pybind example on Windows (go to 64 bit anaconda):
- close Spyder or any other python application during this task!
- cd C:\ProgramData\Anaconda3_64b36\Scripts
- open Anaconda prompt (go to anaconda/Scripts folder and call activate.bat):
  - go to folder with setup.py
  - run: python setup.py install ==> this is not the correct way, but does the job
  - #####run: pip install ./python_example ==> does not work!

==> the package will be installed in the site-packages path. This depends on system and installation
==> if anaconda is installed with admin rights, it will be placed e.g. into 
    'C:\\ProgramData\\Anaconda3\\lib\\site-packages

- in visual studio, anaconda lies in a folder similar to this:
  C:\Program Files (x86)\Microsoft Visual Studio\Shared\Anaconda3_86\Scripts

====================================================================
debugging python programs:
sudo apt-get install gdb
gdb python3
==> in (gdb) call:
run example.py

- add debug information to build:
CFLAGS='-Wall -O0 -g' python setup.py build

- or for installation use:
CFLAGS='-Wall -O0 -g' sudo python3 setup.py install

debug in python:
python3 -m pdb test.py







the following notes are made during solving problems when adapting for linux/UBUNTU:
====================================================================
====================================================================
====================================================================
import error:
>>> import exudyn as e
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
ImportError: /usr/local/lib/python3.6/dist-packages/exudyn-0.1.368-py3.6-linux-x86_64.egg/exudyn.cpython-36m-x86_64-linux-gnu.so: undefined symbol: _ZN27CObjectContactCircleCable2D19maxNumberOfSegmentsE

with gcc-8.4.0:
johannes@exugen:~/cpp/testGCCexudynNoGLFW$ python3
Python 3.6.9 (default, Apr 18 2020, 01:56:04) 
[GCC 8.4.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import exudyn
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
ImportError: /home/johannes/.local/lib/python3.6/site-packages/exudyn-0.1.368-py3.6-linux-x86_64.egg/exudyn.cpython-36m-x86_64-linux-gnu.so: undefined symbol: _ZTVN10__cxxabiv117__class_type_infoE


====================================================================
install different gcc versions:

(python version + gcc version: shown on startup of python3)

sudo apt install software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt install gcc-8
==> installs gcc-8.4.0 on ubuntu 18.04
(sudo apt install gcc-7 g++-7 gcc-8 g++-8 gcc-9 g++-9)

modify setup.py:
import os
os.environ["CC"] = "gcc-8"
os.environ["CXX"] = "gcc-8"
====================================================================

#history on UBUNTU18.04 for new installation, proved to work:

sudo apt-get update
sudo apt install python3-pip
sudo dpkg --configure -a
sudo apt install python3-pip
pip3 install pybind11
pip3 install numpy
pip3 install matplotlib
pip3 install scipy

sudo apt-get install freeglut3
sudo apt-get install freeglut3-dev
sudo apt-get install libglfw3
sudo apt-get install libglfw3-dev
sudo apt-get install mesa-common-dev
sudo apt-get install libx11-dev xorg-dev libglew1.5 libglew1.5-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev

sudo python3 setup.py install
python3 pythonDev/Examples/rigid3Dexample.py 


====================================================================
====================================================================

#A ~130-line pair of console dumps comparing setuptools-built vs VS2017-built performance
#(Exudyn V1.0.3, writeSolution = 57.8% / 57.3%) was removed 2026-09-10. VS2017 is long gone and
#the numbers are six years stale. Current build-time and suite figures live in the revision plan
#(revision2026 step R0.4), and PerformanceLogs/ holds the maintained per-release measurements.
