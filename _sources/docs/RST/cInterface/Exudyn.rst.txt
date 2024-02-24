
******
Exudyn
******




These are the access functions to the Exudyn module. General usage is explained in Section :ref:`sec-generalpythoninterface`\  and examples are provided there. The C++ module \ ``exudyn``\  is the root level object linked between Python and C++.In the installed site-packages, the according file is usually denoted as \ ``exudynCPP.pyd``\  for the regular module, \ ``exudynCPPfast.pyd``\  for the module without range checks and \ ``exudynCPPnoAVX.pyd``\  for the module compiled without AVX vector extensions (may depend on your installation).

.. code-block:: python
   :linenos:
   
   #import exudyn module:
   import exudyn as exu
   #create systemcontainer and mbs:
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()

\ The class **exudyn** has the following **functions and structures**:

* | **GetVersionString**\ (\ *addDetails*\  = False): 
  | Get Exudyn built version as string (if addDetails=True, adds more information on compilation Python version, platform, etc.; the Python micro version may differ from that you are working with; AVX2 shows that you are running a AVX2 compiled version)
* | **Help**\ (): 
  | Show basic help information
* | **RequireVersion**\ (\ *requiredVersionString*\ ): 
  | Checks if the installed version is according to the required version. Major, micro and minor version must agree the required level. This function is defined in the \ ``__init__.py``\  file
  | *Example*:

  .. code-block:: python

     exu.RequireVersion("1.0.31")

* | **StartRenderer**\ (\ *verbose*\  = 0): 
  | Start OpenGL rendering engine (in separate thread) for visualization of rigid or flexible multibody system; use verbose=1 to output information during OpenGL window creation; verbose=2 produces more output and verbose=3 gives a debug level; some of the information will only be seen in windows command (powershell) windows or linux shell, but not inside iPython of e.g. Spyder
* | **StopRenderer**\ (): 
  | Stop OpenGL rendering engine
* | **IsRendererActive**\ (): 
  | returns True if GLFW renderer is available and running; otherwise False
* | **DoRendererIdleTasks**\ (\ *waitSeconds*\  = 0): 
  | Call this function in order to interact with Renderer window; use waitSeconds in order to run this idle tasks while animating a model (e.g. waitSeconds=0.04), use waitSeconds=0 without waiting, or use waitSeconds=-1 to wait until window is closed
* | **SolveStatic**\ (\ *mbs*\ , \ *simulationSettings*\  = exudyn.SimulationSettings(), \ *updateInitialValues*\  = False, \ *storeSolver*\  = True): 
  | Static solver function, mapped from module \ ``solver``\ , to solve static equations (without inertia terms) of constrained rigid or flexible multibody system; for details on the Python interface see Section :ref:`sec-mainsystemextensions-solvestatic`\ ; for background on solvers, see Section :ref:`sec-solvers`\ 
* | **SolveDynamic**\ (\ *mbs*\ , \ *simulationSettings*\  = exudyn.SimulationSettings(), \ *solverType*\  = exudyn.DynamicSolverType.GeneralizedAlpha, \ *updateInitialValues*\  = False, \ *storeSolver*\  = True): 
  | Dynamic solver function, mapped from module \ ``solver``\ , to solve equations of motion of constrained rigid or flexible multibody system; for details on the Python interface see Section :ref:`sec-mainsystemextensions-solvedynamic`\ ; for background on solvers, see Section :ref:`sec-solvers`\ 
* | **ComputeODE2Eigenvalues**\ (\ *mbs*\ , \ *simulationSettings*\  = exudyn.SimulationSettings(), \ *useSparseSolver*\  = False, \ *numberOfEigenvalues*\  = -1, \ *setInitialValues*\  = True, \ *convert2Frequencies*\  = False): 
  | Simple interface to scipy eigenvalue solver for eigenvalue analysis of the second order differential equations part in mbs, mapped from module \ ``solver``\ ; for details on the Python interface see Section :ref:`sec-mainsystemextensions-computeode2eigenvalues`\ 
* | **SetOutputPrecision**\ (\ *numberOfDigits*\ ): 
  | Set the precision (integer) for floating point numbers written to console (reset when simulation is started!); NOTE: this affects only floats converted to strings inside C++ exudyn; if you print a float from Python, it is usually printed with 16 digits; if printing numpy arrays, 8 digits are used as standard, to be changed with numpy.set_printoptions(precision=16); alternatively convert into a list
* | **SetLinalgOutputFormatPython**\ (\ *flagPythonFormat*\ ): 
  | True: use Python format for output of vectors and matrices; False: use matlab format
* | **SetWriteToConsole**\ (\ *flag*\ ): 
  | set flag to write (True) or not write to console; default = True
* | **SetWriteToFile**\ (\ *filename*\ , \ *flagWriteToFile*\  = True, \ *flagAppend*\  = False): 
  | set flag to write (True) or not write to console; default value of flagWriteToFile = False; flagAppend appends output to file, if set True; in order to finalize the file, write \ ``exu.SetWriteToFile('', False)``\  to close the output file
  | *Example*:

  .. code-block:: python

     exu.SetWriteToConsole(False) #no output to console
     exu.SetWriteToFile(filename='testOutput.log', flagWriteToFile=True, flagAppend=False)
     exu.Print('print this to file')
     exu.SetWriteToFile('', False) #terminate writing to file which closes the file

* | **SetPrintDelayMilliSeconds**\ (\ *delayMilliSeconds*\ ): 
  | add some delay (in milliSeconds) to printing to console, in order to let Spyder process the output; default = 0
* | **Print**\ (\ *\*args*\ ): 
  | this allows printing via exudyn with similar syntax as in Python print(args) except for keyword arguments: print('test=',42); allows to redirect all output to file given by SetWriteToFile(...); does not output in case that SetWriteToConsole is set to False
* | **SuppressWarnings**\ (\ *flag*\ ): 
  | set flag to suppress (=True) or enable (=False) warnings
* | **InfoStat**\ (\ *writeOutput*\  = True): 
  | Retrieve list of global information on memory allocation and other counts as list:[array_new_counts, array_delete_counts, vector_new_counts, vector_delete_counts, matrix_new_counts, matrix_delete_counts, linkedDataVectorCast_counts]; May be extended in future; if writeOutput==True, it additionally prints the statistics; counts for new vectors and matrices should not depend on numberOfSteps, except for some objects such as ObjectGenericODE2 and for (sensor) output to files; Not available if code is compiled with __FAST_EXUDYN_LINALG flag
* | **Go**\ (): 
  | Creates a SystemContainer SC and a main multibody system mbs
* | **Demo1**\ (\ *showAll*\ ): 
  | Run simple demo without graphics to check functionality, see exudyn/demos.py
* | **Demo2**\ (\ *showAll*\ ): 
  | Run advanced demo without graphics to check functionality, see exudyn/demos.py
* | **InvalidIndex**\ (): 
  | This function provides the invalid index, which may depend on the kind of 32-bit, 64-bit signed or unsigned integer; e.g. node index or item index in list; currently, the InvalidIndex() gives -1, but it may be changed in future versions, therefore you should use this function
* | **\_\_version\_\_**:
  | stores the current version of the Exudyn package
* | **symbolic**:
  | the symbolic submodule for creating symbolic variables in Python, see documentation of Symbolic; For details, see Section Symbolic.
* | **experimental**:
  | Experimental features, not intended for regular users; for available features, see the C++ code class PyExperimental
* | **special**:
  | special attributes and functions, such as global (solver) flags or helper functions; not intended for regular users; for available features, see the C++ code class PySpecial
* | **special.solver**:
  | special solver attributes and functions; not intended for regular users; for available features, see the C++ code class PySpecialSolver
* | **special.solver.timeout**:
  | if >= 0, the solver stops after reaching accoring CPU time specified with timeout; makes sense for parameter variation, automatic testing or for long-running simulations; default=-1 (no timeout)
* | **variables**:
  | this dictionary may be used by the user to store exudyn-wide data in order to avoid global Python variables; usage: exu.variables["myvar"] = 42 
* | **sys**:
  | this dictionary is used and reserved by the system, e.g. for testsuite, graphics or system function to store module-wide data in order to avoid global Python variables; the variable exu.sys['renderState'] contains the last render state after exu.StopRenderer() and can be used for subsequent simulations 



