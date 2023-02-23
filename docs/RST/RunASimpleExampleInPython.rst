.. _sec-install-simpleexample:


Run a simple example in Python
==============================

After performing the steps of the previous section, this section shows a simplistic model which helps you to check if Exudyn runs on your computer.

In order to start, run the Python interpreter Spyder (or any preferred Python environment).
In order to test the following example, which creates a \ :ref:`mbs <mbs>`\ , adds a node, an object, a marker and a load and simulates everything with default values, 


+  open \ ``myFirstExample.py``\  from your \ ``Examples``\  folder.

Hereafter, press the play button or \ ``F5``\  in Spyder.


If successful, the IPython Console of Spyder will print something like:

.. code-block:: 

  runfile('C:/DATA/cpp/EXUDYN_git/main/pythonDev/Examples/myFirstExample.py', 
    wdir='C:/DATA/cpp/EXUDYN_git/main/pythonDev/Examples')
  +++++++++++++++++++++++++++++++
  EXUDYN V1.2.9 solver: implicit second order time integration
  STEP100, t = 1 sec, timeToGo = 0 sec, Nit/step = 1
  solver finished after 0.0007824 seconds.



If you check your current directory (where \ ``myFirstExample.py``\  lies), you will find a new file \ ``coordinatesSolution.txt``\ , which contains the results of your computation (with default values for time integration).
The beginning and end of the file should look like: 


.. code-block:: 

  #Exudyn implicit second order time integration solver solution file
  #simulation started=2022-04-07,19:02:19
  #columns contain: time, ODE2 displacements, ODE2 velocities, ODE2 accelerations
  #number of system coordinates [nODE2, nODE1, nAlgebraic, nData] = [2,0,0,0]
  #number of written coordinates [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData] = [2,2,2,0,0,0,0]
  #total columns exported  (excl. time) = 6
  #number of time steps (planned) = 100
  #Exudyn version = 1.2.33.dev1; Python3.9.11; Windows AVX2 FLOAT64
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


Within this file, the first column shows the simulation time and the following columns provide coordinates, their derivatives and Lagrange multipliers on system level. For relation of local to global coordinates, see Section :ref:`sec-overview-ltgmapping`\ . As expected, the \ :math:`x`\ -coordinate of the point mass has constant acceleration \ :math:`a=f/m=0.001/10=0.0001`\ , the velocity grows up to \ :math:`0.0001`\  after 1 second and the point mass moves \ :math:`0.00005`\  along the \ :math:`x`\ -axis.

Note that line 8 contains the Exudyn and Python versions\ (as well as some other specific information on the platform and compilation settings (which may help you identify with which computer, etc., you created results)) provided in the solution file are the versions at which Exudyn has been compiled with.
The Python micro version (last digit) may be different from the Python version from which you were running Exudyn.
This information is also provided in the sensor output files.

