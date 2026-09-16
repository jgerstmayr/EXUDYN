
******
Exudyn
******




These are the access functions to the Exudyn module. General usage is explained in Section :ref:`sec-generalpythoninterface`\  and examples are provided there. The C++ module \ ``exudyn``\  is the root level object linked between Python and C++.In the installed site-packages, the according file is usually denoted as \ ``exudynCPP.pyd``\  for the regular module, which is compiled for the baseline instruction set and runs on any 64-bit CPU, and \ ``exudynCPPfast.pyd``\  for the optional module without range checks, which additionally uses the AVX2 vector extensions (may depend on your installation).

.. code-block:: python
   :linenos:
   
   #import exudyn module:
   import exudyn as exu
   #create systemcontainer and mbs:
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()

\ The class **exudyn** has the following **functions and structures**:

* | **Help**\ (): 
  | Show basic help information
* | **RequireVersion**\ (\ *requiredVersionString*\ ): 
  | Checks if the installed version is according to the required version. Major, micro and minor version must agree the required level. This function is defined in the \ ``__init__.py``\  file
  | *Example*:

  .. code-block:: python

     exu.RequireVersion("1.0.31")

* | **SetWriteToFile**\ (\ *filename*\ , \ *flagWriteToFile*\  = True, \ *flagAppend*\  = False, \ *flagFlushAlways*\  = False): 
  | set flag to write (True) or not write to console; default value of flagWriteToFile = False; flagAppend appends output to file, if set True; in order to finalize the file, write \ ``exu.SetWriteToFile('', False)``\  to close the output file; in case of flagFlushAlways=True, file will be finalized immediately in every print command, but may be slower; the filename is relative to exudyn.config.outputDirectory, which is prepended when the file is opened; an absolute filename together with a non-empty outputDirectory raises an error;
  | *Example*:

  .. code-block:: python

     exudyn.config.printToConsole = False #no output to console
     exu.SetWriteToFile(filename='testOutput.log', flagWriteToFile=True, flagAppend=False, flagFlushAlways=False)
     exu.Print('print this to file')
     exu.SetWriteToFile('', False) #terminate writing to file which closes the file

* | **Print**\ (): 
  | this allows printing via exudyn with similar syntax as in Python print(args) except for keyword arguments: exu.Print('test=',42,sep=' ',end='',flush=True); allows to redirect all output to file given by SetWriteToFile(...); does not print to console in case that exudyn.config.printToConsole eis set to False
* | **InvalidIndex**\ (): 
  | This function provides the invalid index, which may depend on the kind of 32-bit, 64-bit signed or unsigned integer; e.g., node index or item index in list; currently, the InvalidIndex() gives -1, but it may be changed in future versions, therefore you should use this function
* | **\_\_version\_\_**:
  | contains the current version of the Exudyn package
* | **symbolic**:
  | the symbolic submodule for creating symbolic variables in Python, see documentation of Symbolic; For details, see Section Symbolic.
* | **config**:
  | global config settings, like precision, print behavior, warnings, etc.
* | **config.suppressWarnings**:
  | flag to suppress all warnings (default=False)
* | **config.outputDirectory**:
  | directory which is prepended to all files written by the solver: the coordinates solution file, the solver information file, sensor files and exported images; default="" (files are written exactly as given). An absolute file name together with a non-empty outputDirectory raises an error when the file is opened. NOTE: this setting is global and stays active as long as the exudyn module is loaded, so running two models one after the other in the same process will put both outputs into the same directory; normally you should specify the output folder directly in the file names and use this setting only for a test runner or a batch script. The rule is: everything WRITTEN as output of a run follows the setting, while a file is READ from there only if its name comes from Exudyn itself. Writing: the coordinates solution file, the solver information file, sensor files, exported images, the exudyn.Print log (SetWriteToFile), the figure saved by PlotSensor and the results file of ParameterVariation/GeneticOptimization. Reading: SolutionViewer (name taken from the simulation settings) and PlotSensor for a sensor given by its number (name taken from the sensor). NOT affected: a file name you pass yourself, e.g. to LoadSolutionFile, LoadBinarySolutionFile, RecoverSolutionFile, InitializeFromRestartFile or PlotSensor as a string, and all model data such as mesh import, FEMinterface/ObjectFFRFreducedOrderInterface SaveToFile/LoadFromFile and SaveDictToHDF5/LoadDictFromHDF5; use exudyn.basicUtilities.OutputFilePath(fileName, callerInfo) in your script if you want those in the output directory as well
* | **config.outputPrecision**:
  | change precision (number of digits) in C++ and Python output
* | **config.linalgOutputFormatPython**:
  | True (default): use Python format for output of vectors and matrices; False: use Matlab format
* | **config.printDelayMilliSeconds**:
  | add some delay (in milliSeconds) to printing to console (exudyn.Print), in order to let console (e.g., Spyder) process the output; default = 0
* | **config.printFlushAlways**:
  | flush always buffers when using exudyn.Print(...) to write to file or console; this is needed if you are streaming text or showing counters in parameter variation; default=False
* | **config.printToConsole**:
  | enables or disables writing to console with exudyn.Print(...); default=True
* | **config.printToFile**:
  | flag that shows if writing to file with exudyn.Print(...) is enabled; flag is readonly
* | **config.printFileName**:
  | file name for writing to file with exudyn.Print(...), as resolved when the file was opened: it is relative to config.outputDirectory, which is prepended by SetWriteToFile(...); flag is readonly
* | **config.printToFileAppend**:
  | flag that shows if append mode is used for writing to file with exudyn.Print(...); flag is readonly
* | **config.Version**\ (\ *addDetails*\  = False): 
  | Get Exudyn built version as string (if addDetails=True, adds more information on compilation Python version, platform, etc.; the Python micro version may differ from that you are working with; AVX2 shows that you are running a AVX2 compiled version)
* | **experimental**:
  | Experimental features, not intended for regular users; for available features, see the C++ code class PyExperimental
* | **special**:
  | special attributes and functions, such as global (solver) flags or helper functions; not intended for regular users; for available features, see the C++ code class PySpecial
* | **special.InfoStat**\ (\ *writeOutput*\  = True): 
  | Retrieve list of global information on memory allocation and other counts as list:[array_new_counts, array_delete_counts, vector_new_counts, vector_delete_counts, matrix_new_counts, matrix_delete_counts, linkedDataVectorCast_counts]; May be extended in future; if writeOutput==True, it additionally prints the statistics; counts for new vectors and matrices should not depend on numberOfSteps, except for some objects such as ObjectGenericODE2 and for (sensor) output to files; Not available if code is compiled with __FAST_EXUDYN_LINALG flag
* | **special.solver**:
  | special solver attributes and functions; not intended for regular users; for available features, see the C++ code class PySpecialSolver
* | **special.solver.timeout**:
  | if >= 0, the solver stops after reaching accoring CPU time specified with timeout; makes sense for parameter variation, automatic testing or for long-running simulations; default=-1 (no timeout)
* | **special.solver.multiThreadingLoadBalancing**:
  | if True (=default), multithreaded code parts (in particular solver and raytracing) use load balancing, which may give better performance in case of non-equilibrated loads; (mobile) Intel CPUs may perform significantly better without load balancing
* | **special.exceptions**:
  | special flags for exceptions and checks; not intended for regular users; for available features, see the C++ code class PySpecialExceptions
* | **special.exceptions.parameterRangeChecks**:
  | if True (=default), writing an item or settings parameter outside its range (e.g. a negative mass or a non-positive number of steps) raises an error, on every write path (item classes, dictionaries, SetObjectParameter, ...); set False to accept any value, e.g. if a range limit turns out to be wrong
* | **variables**:
  | this dictionary may be used by the user to store exudyn-wide data in order to avoid global Python variables; usage: exu.variables["myvar"] = 42; can be used in particular to exchange data between different mbs or between packages by importing exudyn.variables wherever needed.
* | **sys**:
  | this dictionary is used and reserved by the system, e.g., for testsuite, graphics or system function to store module-wide data in order to avoid global Python variables; the variable exu.sys['renderState'] contains the last render state after SC.renderer.Stop() and can be used for subsequent simulations 



