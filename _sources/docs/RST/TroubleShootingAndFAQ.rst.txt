.. _sec-install-troubleshootingfaq:


Trouble shooting and FAQ
========================



Trouble shooting
----------------

Exudyn has a solid exception handling and does lots of error checking on
inputs as well as during computations. This should usually not lead to an
unexpected crash as in early days of scientific codes.

For basic information on exception handling, see also the according section on
Exceptions and Error Messages. In the following, typical error messages are listed. 

\ **Python import errors**\ :

+  Sometimes the Exudyn module cannot be loaded into Python. Typical \ **error messages if Python versions are not compatible**\  are: 



.. code-block:: 

  Traceback (most recent call last):

    File "<ipython-input-14-df2a108166a6>", line 1, in <module>
      import exudynCPP

  ImportError: Module use of python36.dll conflicts with this version of Python.


Typical \ **error messages if 32/64 bits versions are mixed**\ :


.. code-block:: 

  Traceback (most recent call last):
  
    File "<ipython-input-2-df2a108166a6>", line 1, in <module>
      import exudynCPP

  ImportError: DLL load failed: \%1 is not a valid Win32 application.


\ **There are several reasons and workarounds**\ :

  |  →  You mixed up 32 and 64 bits version (see below) 
  |  →  You are using an exudyn version for Python \ :math:`x_1.y_1`\  (e.g., 3.6.\ :math:`z_1`\ ) different from the Python \ :math:`x_2.y_2`\  version in your Anaconda (e.g., 3.7.\ :math:`z_2`\ ); note that \ :math:`x_1=x_2`\  and \ :math:`y_1=y_2`\  must be obeyed while \ :math:`z_1`\  and \ :math:`z_2`\  may be different

+  \ **Import of exudyn C++ module failed Warning: ...**\ :

  |  →  ... and similar messages with: ModuleNotFoundError, Warning, with AVX2, without AVX2
  |  →  A known reason is that your CPU \ **does not support AVX2**\ , while Exudyn is compiled with the AVX2 option\ (modern Intel Core-i3, Core-i5 and Core-i7 processors as well as AMD processors, especially Zen and Zen-2 architectures should have no problems with AVX2; however, low-cost Celeron, Pentium and older AMD processors do \ **not**\  support AVX2, e.g.,  Intel Celeron G3900, Intel core 2 quad q6600, Intel Pentium Gold G5400T; check the system settings of your computer to find out the processor type; typical CPU manufacturer pages or Wikipedia provide information on this).
  |  →  \ **solution**\ : the release versions without the .dev1 ending in the wheel contain C++ libraries which are compiled without AVX/AVX2; the module loader will usually detect automatically, if your CPU supports AVX/AVX2; if not, it will load the exudynCPPnoAVX.cp ... .pyd file; if this does not work, try

\ ``import sys``\ 

\ ``sys.exudynCPUhasAVX2 = False``\ 

to explicitly load the version without AVX2.
  |  →  you can also compile for your specific Python version without AVX if you adjust the \ ``setup.py``\  file in the \ ``main``\  folder.
  |  →  \ **DEPRECATED workaround**\  to solve the AVX problem: use the Python 3.6 version (up to Exudyn V1.2.28 only the 32bit version), which is compiled without AVX2.
  |  →  The \ ``ModuleNotFoundError``\  may also happen if something went wrong during installation (paths, problems with Anaconda, ..) \ :math:`\ra`\  very often a new installation of Anaconda and Exudyn helps.


\ **Typical Python errors**\ :

+  Typical Python \ **syntax error**\  with missing braces:


.. code-block:: 

  File "C:\DATA\cpp\EXUDYN_git\main\pythonDev\Examples\springDamperTutorial.py", line 42
      nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
             ^
  SyntaxError: invalid syntax


\ 
  |  →  such an error points to the line of your code (line 42), but in fact the error may have been caused in previous code, such as in this case there was a missing brace in the line 40, which caused the error:

.. code-block:: python

  38  n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0],
  39                       initialCoordinates = [u0,0,0],
  40                       initialVelocities= [v0,0,0])
  41  #ground node
  42  nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))
  43  


+  Typical Python \ **import error**\  message on Linux / Ubuntu if Python modules are missing:


.. code-block:: 

  Python WARNING [file '/home/johannes/.local/lib/python3.6/site-packages/exudyn/solver.py', line 236]: 
  Error when executing process ShowVisualizationSettingsDialog':
  ModuleNotFoundError: No module named 'tkinter'


\ 
  |  →  see installation instructions to install missing Python modules, Section :ref:`sec-install-installinstructions`\ .

+  Problems with \ **tkinter**\ , especially on MacOS:

  Exudyn uses \ ``tkinter``\ , based on tcl/tk, to provide some basic dialogs, such as visualizationSettings

  As Python is not suited for multithreading, this causes problems in window and dialog workflows. Especially on MacOS
  \ ``tkinter``\  is less stable and compatible with the window manager. Especially, \ ``tkinter``\  already needs to run
  before the application's OpenGL window (renderer) is opened. Therefore, on MacOS \ ``tkinter.Tk()``\  is called before the 
  renderer is started.
  In some cases, visualizationSettings dialog may not be available and changes have to be made inside the code.
  
  |  →  To resolve issues, the following visualizationSettings may help (before starting renderer!), but may reduce functionality: 
     dialogs.multiThreadedDialogs = False, general.useMultiThreadedRendering = False
 



\ **Typical solver errors**\ :

+  Consider the  example for a mixed error message comes from the solver when called for a (possibly empty) \ ``mbs``\  with no prior call to \ ``mbs.Assemble()``\ :


.. code-block:: 

  import exudyn as exu
  from exudyn.utilities import *
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  sims=exu.SimulationSettings()
  exu.SolveDynamic(mbs, sims)


+  This will results in error messages similar to:

.. code-block:: 

  =========================================
  User ERROR [file 'C:\Users\username\.condavs\venvP39\lib\site-packages\exudyn\solver.py', line 245]: 
  Solver: system is inconsistent and cannot be solved (call Assemble() and check error messages)
  =========================================

  =========================================
  SYSTEM ERROR [file 'C:\Users\username\.condavs\venvP39\lib\site-packages\exudyn\solver.py', line 245]: 
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

    File "C:\Users\username\.condavs\venvP39\lib\site-packages\exudyn\solver.py", line 255, in SolveDynamic
      raise ValueError("SolveDynamic terminated")

  ValueError: SolveDynamic terminated


\ 

  |  →  it seems clear that you should read this error from top as it indicates that you just forgot to call \ ``mbs.Assemble()``\ 

+  WSL/Ubuntu: render window crashes after left or right mouse click:

  |  →  This happens, in some (all?) Linux installations during mouse selection
  |  →  workaround: setting \ ``SC.visualizationSettings.interactive.selectionLeftMouse = False``\  and \ ``SC.visualizationSettings.interactive.selectionRightMouse = False``\  removes the option to select with mouse, but avoids crashes

+  \ ``SolveDynamic``\  or \ ``SolveStatic``\  \ **terminated due to errors**\ :

  |  →  use flag \ ``showHints = True``\  in \ ``SolveDynamic``\  or \ ``SolveStatic``\ 

+  Very simple example \ **without loads**\  leads to error: \ ``SolveDynamic``\  or \ ``SolveStatic``\  \ **terminated due to errors**\ :

  |  →  see also 'Convergence problems', Section :ref:`sec-overview-basics-convergenceproblems`\ 
  |  →  may be caused due to nonlinearity of formulation and round off errors, which restrict Newton to achieve desired tolerances; adjust  \ ``.newton.relativeTolerance``\  / \ ``.newton.absoluteTolerance``\  in static solver or in time integration

+  Typical \ **solver error due to redundant constraints or missing inertia terms**\ , could read as follows:

.. code-block:: 

  =========================================
  SYSTEM ERROR [file 'C:\ProgramData\Anaconda3_64b37\lib\site-packages\exudyn\solver.py', line 207]: 
  CSolverBase::Newton: System Jacobian seems to be singular / not invertible!
  time/load step #1, time = 0.0002
  causing system equation number (coordinate number) = 42
  =========================================


\ 

  |  →  this solver error shows that equation 42 is not solvable. The according coordinate is shown later in such an error message:


.. code-block:: 

  ...
  The causing system equation 42 belongs to a algebraic variable (Lagrange multiplier)
  Potential object number(s) causing linear solver to fail: [7]
      object 7, name='object7', type=JointGeneric


\ 

  |  →  object 7 seems to be the reason, possibly there are too much (joint) constraints applied to your system, check this object.
  |  →  show typical REASONS and SOLUTIONS, by using \ ``showHints=True``\  in \ ``exu.SolveDynamic(...)``\  or \ ``exu.SolveStatic(...)``\ 
  |  →  You can also \ **highlight**\  object 7 by using the following code in the iPython console:


.. code-block:: python

  SC.renderer.Start()
  HighlightItem(SC,mbs,7)


which draws the according object in red and others gray/transparent (but sometimes objects may be hidden inside other objects!). See the command's description for further options, e.g., to highlight nodes.


+  Typical \ **solver error if Newton does not converge**\ :


.. code-block:: 

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


\ 

  |  →  this solver error is caused, because the nonlinear system cannot be solved using Newton's method.
  |  →  the static or dynamic solver by default tries to reduce step size to overcome this problem, but may fail finally (at minimum step size).
  |  →  possible reasons are: too large time steps (reduce step size by using more steps/second), inappropriate initial conditions, or inappropriate joints or constraints (remove joints to see if they are the reason), usually within a singular configuration. Sometimes a system may be just unsolvable in the way you set it up.
  |  →  see also 'Convergence problems', Section :ref:`sec-overview-basics-convergenceproblems`\ 

+  Typical solver error if (e.g., syntax) \ **error in user function**\  (output may be very long, \ **read always message on top!**\ ):

.. code-block:: 

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


\ 

  |  →  this indicates an error in the user function \ ``LoadCoordinate::loadVectorUserFunction``\ , because \ ``sin``\  function has not been defined (must be imported, e.g., from \ ``math``\ ). It indicates that the error occurred in line 48 in \ ``springDamperUserFunctionTest.py``\  within function \ ``Sweep``\ , which has been called from function \ ``userLoad``\ , etc.

 

FAQ
---

\ **Some frequently asked questions**\ :

+  When \ **importing**\  Exudyn in Python (windows) I get an error 

  |  →  see trouble shooting instructions above!

+  I do not understand the \ **Python errors**\  -- how can I find the reason of the error or crash?

  |  →  Read trouble shooting section above! 
  |  →  First, you should read all error messages and warnings: from the very first to the last message. Very often, there is a definite line number which shows the error. Note, that if you are executing a string (or module) as a Python code, the line numbers refer to the local line number inside the script or module.
  |  →  If everything fails, try to execute only part of the code to find out where the first error occurs. By omiting parts of the code, you should find the according source of the error.
  |  →  If you think, it is a bug: send an email with a representative code snippet, version, etc. to \ `` reply.exudyn@gmail.com``\ 

+  Spyder \ **console hangs**\  up, does not show error messages, ...:

  |  →  very often a new start of Spyder helps; most times, it is sufficient to restart the kernel or to just press the 'x' in your IPython console, which closes the current session and restarts the kernel (this is much faster than restarting Spyder)
  |  →  restarting the IPython console also brings back all error messages

+  Where do I find the \ **'.exe' file**\ ?

  |  →  Exudyn is only available via the Python interface as a module '\ ``exudyn``\ ', the C++ code being inside of \ ``exudynCPP.pyd``\ , which is located in the exudyn folder where you installed the package. This means that you need to \ **run Python**\  (best: Spyder) and import the Exudyn module.

+  I get the error message 'check potential mixing of different (object, node, marker, ...) indices', what does it mean?

  |  →  probably you used wrong item indexes, see beginning of command interface in Section :ref:`sec-pcpp-command-interface`\ . 
  |  →  E.g., an object number \ ``oNum = mbs.AddObject(...)``\  is used at a place where a \ ``NodeIndex``\  is expected, e.g., \ ``mbs.AddObject(MassPoint(nodeNumber=oNum, ...))``\ 
  |  →  Usually, this is an ERROR in your code, it does not make sense to mix up these indexes!
  |  →  In the exceptional case, that you want to convert numbers, see beginning of Section :ref:`sec-pcpp-command-interface`\ .

+  Why does \ **type auto completion**\  / intellisense not work for mbs (MainSystem)?

  |  →  in earlier versions of Exudyn type completion did not work properly for more complex structures
  |  →  since version 1.6.103 type completion works for most functions\, types and structures: tested in Spyder 5.2.2 and Visual Studio Code 1.78.1); with an added stub file (.pyi) the standard type completion fetches information about structures or functions; this even works for example with \ ``SC.visualizationSettings.bodies.kinematicTree.showJointFrames``\ . If you still have problems, try to restart your environment / computer or switch to a different version, and create an issue on GitHub.

+  How to add graphics?

  |  →  Graphics (lines, text, 3D triangular / \ :ref:`STL <STL>`\  mesh) can be added to all BodyGraphicsData items in objects. Graphics objects which are fixed with the background can be attached to a ObjectGround object. Moving objects must be attached to the BodyGraphicsData of a moving body. Other moving bodies can be realized, e.g., by adding a ObjectGround and changing its reference with time. Furthermore, ObjectGround allows to add fully user defined graphics.

+  In \ ``GenerateStraightLineANCFCable2D``\  

  |  →  coordinate constraints can be used to constrain position and rotation, e.g., \ ``fixedConstraintsNode0 = [1,1,0,1]``\  for a beam aligned along the global x-axis; 
  |  →  this \ **does not work**\  for beams with arbitrary rotation in reference configuration, e.g., 45°. Use a GenericJoint with a rotationMarker instead.

+  What is the difference between MarkerBodyPosition and MarkerBodyRigid?

  |  →  Position markers (and nodes) do not have information on the orientation (rotation). For that reason, there is a difference between position based and rigid-body based markers. In case of a rigid body attached to ground with a SpringDamper, you can use both, MarkerBodyPosition or MarkerBodyRigid, markers. For a prismatic joint, you will need a MarkerBodyRigid.

+  I get an error in \ ``exu.SolveDynamic(mbs, ...)``\  OR in \ ``exu.SolveStatic(mbs, ...)``\  but no further information -- how can I solve it?

  |  →  Typical \ **time integration errors**\  may look like:

.. code-block:: 

  File "C:/DATA/cpp/EXUDYN_git/main/pythonDev/...<file name>", line XXX, in <module>
  solver.SolveSystem(...)
  SystemError: <built-in method SolveSystem of PyCapsule object at 0x0CC63590> returned a result with an error set


  |  →  The pre-checks, which are performed to enable a crash-free simulation are insufficient for your model
  |  →  As a first try, \ **restart the IPython console**\  in order to get all error messages, which may be blocked due to a previous run of Exudyn.
  |  →  Very likely, you are using Python user functions inside Exudyn: They lead to an internal Python error, which is not always catched by Exudyn; e.g., a load user function UFload(mbs,~t,~load), which tries to access component load[3] of a load vector with 3 components will fail internally;
  |  →  Use the print(...) command in Python at many places to find a possible error in user functions (e.g., put \ ``print("Start user function XYZ")``\  at the beginning of every user function; test user functions from iPython console
  |  →  It is also possible, that you are using inconsistent data, which leads to the crash. In that case, you should try to change your model: omit parts and find out which part is causing your error
  |  →  see also \ **I do not understand the Python errors -- how can I find the cause?**\ 


+  Why can't I get the focus of the simulation window on startup (render window hidden)?

  |  →  Starting Exudyn out of Spyder might not bring the simulation window to front, because of specific settings in Spyder(version 3.2.8), e.g., Tools\ :math:`\ra`\ Preferences\ :math:`\ra`\ Editor\ :math:`\ra`\ Advanced settings: uncheck 'Maintain focus in the Editor after running cells or selections'; Alternatively, set \ ``SC.visualizationSettings.window.alwaysOnTop=True``\  \ **before**\  starting the renderer with \ ``SC.renderer.Start()``\ 




