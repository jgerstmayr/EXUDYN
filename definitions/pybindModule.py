#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN definition file
#
# Details:  the functions and attributes of the exudyn module itself (exu.config, exu.special, ...).
#           The calls are recorded by PybindInterface (pybindTypes.py) and replayed by
#           tools/generators/pybindEmitter.py into pybind_manual_classes.h, the stub fragments and
#           the Python-C++ interface documentation.
#
#           DESCRIPTIONS: read definitions/README.md, section "Writing a
#           description", before writing or changing one - what the text may
#           contain, and how it is checked.
#
# Author:   Johannes Gerstmayr
# Date:     2018-05-18 (created in autoGeneratePyBindings.py), 2026-09-14 (moved to definitions/)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from pybindTypes import *

pb = PybindInterface()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Access functions to EXUDYN
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
pb.CreateNewRSTfile('Exudyn')
pb.DefPyStartClass('','', '')

pb.AddDocu(r"""These are the access functions to the Exudyn module. General usage is explained in [](#sec-generalpythoninterface) and examples are provided there. The C++ module `exudyn` is the root level object linked between Python and C++.In the installed site-packages, the according file is usually denoted as `exudynCPP.pyd` for the regular module, which is compiled for the baseline instruction set and runs on any 64-bit CPU, and `exudynCPPfast.pyd` for the optional module without range checks, which additionally uses the AVX2 vector extensions (may depend on your installation).""")

pb.AddDocuCodeBlock(code="""
#import exudyn module:
import exudyn as exu
#create systemcontainer and mbs:
SC = exu.SystemContainer()
mbs = SC.AddSystem()
""")
pb.DefStartTable('exudyn')

pb.DefPyFunctionAccess('', 'Help', 'PyHelp', 
                               description='Show basic help information',
                               returnType='None',
                               )

pb.DefPyFunctionAccess(cClass='', pyName='StartRenderer', cName='PyStartOpenGLRenderer', 
                                description="DEPRECATED; Start OpenGL rendering engine (in separate thread) for visualization of rigid or flexible multibody system; use verbose=1 to output information during OpenGL window creation; verbose=2 produces more output and verbose=3 gives a debug level; some of the information will only be seen in windows command (powershell) windows or linux shell, but not inside iPython of e.g., Spyder",
                                argList=['verbose','deprecationWarning'],
                                defaultArgs=['0','True'],
                                returnType='bool',
                                addDocu=False,
                                )

#new, defined in C++ as lambda function:
pb.DefPyFunctionAccess(cClass='', pyName='StopRenderer', cName='PyStopOpenGLRenderer',#'no direct link to C++ here', 
                                description="DEPRECATED; Stop OpenGL rendering engine",
                                argList=['deprecationWarning'],
                                defaultArgs=['True'],
                                returnType='None', #the only declaration that had none, so it was
                                #the only module function still missing from the stub, #2490
                                addDocu=False,
                                )

pb.DefPyFunctionAccess(cClass='', pyName='IsRendererActive', cName='PyIsRendererActive', 
                                description="DEPRECATED; returns True if GLFW renderer is available and running; otherwise False",
                                argList=['deprecationWarning'],
                                defaultArgs=['True'],
                                returnType='bool',
                                addDocu=False,
                                )

pb.DefPyFunctionAccess(cClass='', pyName='DoRendererIdleTasks', cName='PyDoRendererIdleTasks', 
                                description="DEPRECATED; Call this function in order to interact with Renderer window; use waitSeconds in order to run this idle tasks while animating a model (e.g., waitSeconds=0.04), use waitSeconds=0 without waiting, or use waitSeconds=-1 (default) to wait until window is closed",
                                argList=['waitSeconds','deprecationWarning'],
                                defaultArgs=['0','True'],
                                returnType='None',
                                addDocu=False,
                                )

pb.BeginCppWrittenByHand()
pb.DefPyFunctionAccess(cClass='', pyName='SolveStatic', cName='SolveDynamic', 
                               description=r"""DEPRECATED; Static solver function, mapped from module `solver`, to solve static equations (without inertia terms) of constrained rigid or flexible multibody system; for details on the Python interface see [](#sec-mainsystemextensions-solvestatic); for background on solvers, see [](#sec-solvers)""",
                               argList=['mbs', 'simulationSettings', 'updateInitialValues', 'storeSolver'],
                               defaultArgs=['','exudyn.SimulationSettings()','False','True'],
                               argTypes=['MainSystem','SimulationSettings', '', ''],
                               returnType='bool',
                               addDocu=False,
                               )
                
pb.DefPyFunctionAccess(cClass='', pyName='SolveDynamic', cName='SolveDynamic', 
                               description=r"""DEPRECATED; Dynamic solver function, mapped from module `solver`, to solve equations of motion of constrained rigid or flexible multibody system; for details on the Python interface see [](#sec-mainsystemextensions-solvedynamic); for background on solvers, see [](#sec-solvers)""",
                               argList=['mbs', 'simulationSettings', 'solverType', 'updateInitialValues', 'storeSolver'],
                               defaultArgs=['','exudyn.SimulationSettings()','exudyn.DynamicSolverType.GeneralizedAlpha','False','True'],
                               argTypes=['MainSystem','SimulationSettings', 'DynamicSolverType', '', ''],
                               returnType='bool',
                               addDocu=False,
                               )
                
pb.DefPyFunctionAccess(cClass='', pyName='ComputeODE2Eigenvalues', cName='ComputeODE2Eigenvalues', 
                               description=r"""DEPRECATED; Simple interface to scipy eigenvalue solver for eigenvalue analysis of the second order differential equations part in mbs, mapped from module `solver`; for details on the Python interface see [](#sec-mainsystemextensions-computeode2eigenvalues)""",
                               argList=['mbs', 'simulationSettings', 'useSparseSolver', 'numberOfEigenvalues', 'setInitialValues', 'convert2Frequencies'],
                               defaultArgs=['','exudyn.SimulationSettings()','False','-1','True','False'],
                               #argTypes=['MainSystem','SimulationSettings', 'bool', 'int', 'bool', 'bool'],
                               argTypes=['MainSystem','SimulationSettings', '', '', '', ''],
                               returnType='bool',
                               addDocu=False,
                               )
pb.EndCppWrittenByHand()

pb.BeginCppWrittenByHand()
pb.DefPyFunctionAccess('', 'RequireVersion', '', 
                               argList=['requiredVersionString'],
                               description = r"""Checks if the installed version is according to the required version. Major, micro and minor version must agree the required level. This function is defined in the `__init__.py` file""", 
                               example='exu.RequireVersion("1.0.31")',
                               argTypes=['str'],
                               returnType='None',
                               )
pb.EndCppWrittenByHand() #this function is defined in __init__.py ==> do not add to cpp bindings


#print('complete stub file for exudyn module? config?')


pb.DefPyFunctionAccess(cClass='', pyName='SetWriteToFile', cName='PySetWriteToFile', 
                            description=r"""set flag to write (True) or not write to console; default value of flagWriteToFile = False; flagAppend appends output to file, if set True; in order to finalize the file, write `exu.SetWriteToFile('', False)` to close the output file; in case of flagFlushAlways=True, file will be finalized immediately in every print command, but may be slower; the filename is relative to exudyn.config.outputDirectory, which is prepended when the file is opened; an absolute filename together with a non-empty outputDirectory raises an error;""",
                            argList=['filename', 'flagWriteToFile', 'flagAppend', 'flagFlushAlways'],
                            defaultArgs=['', 'True', 'False', 'False'],
                            example=r"""exudyn.config.printToConsole = False #no output to console\\exu.SetWriteToFile(filename='testOutput.log', flagWriteToFile=True, flagAppend=False, flagFlushAlways=False)\\exu.Print('print this to file')\\exu.SetWriteToFile('', False) #terminate writing to file which closes the file""",
                            argTypes=['str','','',''],
                            returnType='None',
                            )

pb.DefPyFunctionAccess(cClass='', pyName='Print', cName='PyPrint', 
                            description="this allows printing via exudyn with similar syntax as in Python print(args) except for keyword arguments: exu.Print('test=',42,sep=' ',end='',flush=True); allows to redirect all output to file given by SetWriteToFile(...); does not print to console in case that exudyn.config.printToConsole is set to False",
                            #argList=[], 
                            #this fails in C++ compilation: ['*args','**kwargs'], and also these:
                            #argList=['args','kwargs'], #shall be: ['py::arg("args" = py::args(), py::arg("kwargs") = py::kwargs()
                            #defaultArgs=['py::args()', 'py::kwargs()'],
                            returnType='None',
                            )

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#DEPRECATED:
pb.DefPyFunctionAccess(cClass='', pyName='SetOutputPrecision', cName='PySetOutputPrecisionOld', 
                                description="DEPRECATED; use set exudyn.config.precision instead",
                                argList=['numberOfDigits'],
                                argTypes=['int'],
                                returnType='None',
                                addDocu=False,
                                )

pb.DefPyFunctionAccess(cClass='', pyName='SetLinalgOutputFormatPython', cName='PySetLinalgOutputFormatPython', 
                                description="DEPRECATED; True: use Python format for output of vectors and matrices; False: use Matlab format",
                                argList=['flagPythonFormat'],
                                argTypes=['bool'],
                                returnType='None',
                                addDocu=False,
                                )

pb.DefPyFunctionAccess('', 'GetVersionString', 'PyGetVersionString', 
                               description='DEPRECATED; Get Exudyn built version as string (if addDetails=True, adds more information on compilation Python version, platform, etc.; the Python micro version may differ from that you are working with; AVX2 shows that you are running a AVX2 compiled version)',
                               argList=['addDetails'],
                               defaultArgs=['False'],
                               returnType='str',
                               addDocu=False,
                               )

pb.DefPyFunctionAccess(cClass='', pyName='SetPrintDelayMilliSeconds', cName='PySetPrintDelayMilliSeconds', 
                            description="DEPRECATED; add some delay (in milliSeconds) to printing to console, in order to let Spyder process the output; default = 0",
                            argList=['delayMilliSeconds'],
                            argTypes=['int'],
                            returnType='None',
                            addDocu=False,
                            )

pb.DefPyFunctionAccess(cClass='', pyName='SuppressWarnings', cName='PySuppressWarningsOld', 
                            description="DEPRECATED; set flag to suppress (=True) or enable (=False) warnings",
                            argList=['flag'],
                            argTypes=['bool'],
                            returnType='None',
                            addDocu=False,
                            )

pb.DefPyFunctionAccess(cClass='', pyName='InfoStat', cName='PythonInfoStatOld', 
                            description='DEPRECATED; Retrieve list of global information on memory allocation and other counts as list:[array_new_counts, array_delete_counts, vector_new_counts, vector_delete_counts, matrix_new_counts, matrix_delete_counts, linkedDataVectorCast_counts]; May be extended in future; if writeOutput==True, it additionally prints the statistics; counts for new vectors and matrices should not depend on numberOfSteps, except for some objects such as ObjectGenericODE2 and for (sensor) output to files; Not available if code is compiled with __FAST_EXUDYN_LINALG flag',
                            argList=['writeOutput'],
                            defaultArgs=['True'],
                            argTypes=[''],
                            returnType='List[int]',
                            addDocu=False,
                            )

pb.DefPyFunctionAccess(cClass='', pyName='SetWriteToConsole', cName='PySetWriteToConsole', 
                            description="DEPRECATED; set flag to write (True) or not write to console; default = True",
                            argList=['flag'],
                            argTypes=['bool'],
                            returnType='None',
                            addDocu=False,
                            )


# pb.DefPyFunctionAccess('', 'Go', 'PythonGo', 'Creates a SystemContainer SC and a main multibody system mbs',
#                             returnType='None',
#                             )

# pb.DefPyFunctionAccess(cClass='', pyName='demos.Demo1', cName='Demo1', 
#                             description="Run simple demo without graphics to check functionality, see exudyn/demos.py",
#                             argList=['showAll'], 
#                             argTypes=['bool'],
#                             returnType='[MainSystem, SystemContainer]',
#                             )
    
# pb.DefPyFunctionAccess(cClass='', pyName='demos.Demo2', cName='Demo2', 
#                             description="Run advanced demo without graphics to check functionality, see exudyn/demos.py",
#                             argList=['showAll'], 
#                             argTypes=['bool'],
#                             returnType='[MainSystem, SystemContainer]',
#                             )
    
pb.DefPyFunctionAccess('', 'InvalidIndex', 'GetInvalidIndex', 
                            "This function provides the invalid index, which may depend on the kind of 32-bit, 64-bit signed or unsigned integer; e.g., node index or item index in list; currently, the InvalidIndex() gives -1, but it may be changed in future versions, therefore you should use this function",
                            returnType='int',
                            )

pb.DefDataAccess('__version__','contains the current version of the Exudyn package',
                       dataType='str', isTopLevel = True,
                       )

pb.DefDataAccess('symbolic','the symbolic submodule for creating symbolic variables in Python, see documentation of Symbolic; For details, see Section Symbolic.',
                       dataType='', isTopLevel = True,
                       )

pb.AddDocu('', section='exudyn.config', sectionLevel=1, sectionLabel='sec-exudyn-config')
#++++++++++
#config
pb.BeginNoStub() #this would not work directly! 
#                  seems that pybind11 provides enough information to get 
#                  type completion working for config, special, ...!

pb.CppCode('        m.attr("config") = py::cast(&pyConfig);\n') 

pb.BeginCppWrittenByHand() #remaining config pybindings added only in C++
pb.DefDataAccess('config','global config settings, like precision, print behavior, warnings, etc.',
                       dataType='Config', isTopLevel = True)
pb.DefDataAccess('config.suppressWarnings','flag to suppress all warnings (default=False)',
                        dataType='int', isTopLevel = True)
pb.DefDataAccess('config.outputDirectory','directory which is prepended to all files written by the solver: the coordinates solution file, the solver information file, sensor files and exported images; default="" (files are written exactly as given). An absolute file name together with a non-empty outputDirectory raises an error when the file is opened. NOTE: this setting is global and stays active as long as the exudyn module is loaded, so running two models one after the other in the same process will put both outputs into the same directory; normally you should specify the output folder directly in the file names and use this setting only for a test runner or a batch script. The rule is: everything WRITTEN as output of a run follows the setting, while a file is READ from there only if its name comes from Exudyn itself. Writing: the coordinates solution file, the solver information file, sensor files, exported images, the exudyn.Print log (SetWriteToFile), the figure saved by PlotSensor and the results file of ParameterVariation/GeneticOptimization. Reading: SolutionViewer (name taken from the simulation settings) and PlotSensor for a sensor given by its number (name taken from the sensor). NOT affected: a file name you pass yourself, e.g. to LoadSolutionFile, LoadBinarySolutionFile, RecoverSolutionFile, InitializeFromRestartFile or PlotSensor as a string, and all model data such as mesh import, FEMinterface/ObjectFFRFreducedOrderInterface SaveToFile/LoadFromFile and SaveDictToHDF5/LoadDictFromHDF5; use exudyn.basicUtilities.OutputFilePath(fileName, callerInfo) in your script if you want those in the output directory as well',
                        dataType='str', isTopLevel = True)
pb.DefDataAccess('config.outputPrecision','change precision (number of digits) in C++ and Python output',
                        dataType='int', isTopLevel = True)
pb.DefDataAccess('config.linalgOutputFormatPython','True (default): use Python format for output of vectors and matrices; False: use Matlab format',
                        dataType='int', isTopLevel = True)
pb.DefDataAccess('config.printDelayMilliSeconds','add some delay (in milliSeconds) to printing to console (exudyn.Print), in order to let console (e.g., Spyder) process the output; default = 0',
                        dataType='int', isTopLevel = True)
pb.DefDataAccess('config.printFlushAlways','flush always buffers when using exudyn.Print(...) to write to file or console; this is needed if you are streaming text or showing counters in parameter variation; default=False',
                        dataType='bool', isTopLevel = True)
pb.DefDataAccess('config.printToConsole','enables or disables writing to console with exudyn.Print(...); default=True',
                        dataType='bool', isTopLevel = True)
pb.DefDataAccess('config.printToFile','flag that shows if writing to file with exudyn.Print(...) is enabled; flag is readonly',
                        dataType='bool', isTopLevel = True, readOnly = True)
pb.DefDataAccess('config.printFileName','file name for writing to file with exudyn.Print(...), as resolved when the file was opened: it is relative to config.outputDirectory, which is prepended by SetWriteToFile(...); flag is readonly',
                        dataType='str', isTopLevel = True, readOnly = True)
pb.DefDataAccess('config.printToFileAppend','flag that shows if append mode is used for writing to file with exudyn.Print(...); flag is readonly',
                        dataType='bool', isTopLevel = True, readOnly = True)


pb.DefPyFunctionAccess(cClass='', pyName='config.Version', cName='unused', 
                        argList=['addDetails'],
                        defaultArgs=['False'],
                        description='Get Exudyn built version as string (if addDetails=True, adds more information on compilation Python version, platform, etc.; the Python micro version may differ from that you are working with; AVX2 shows that you are running a AVX2 compiled version)',
                        returnType='str',
                        )
pb.DefPyFunctionAccess(cClass='', pyName='config.GetDictionary', cName='unused',
                        description='all settings of `exudyn.config` as a dictionary, the way a settings structure gives them; `printToFile`, `printFileName` and `printToFileAppend` are in it but only report what the output is doing',
                        returnType='dict',
                        )
pb.DefPyFunctionAccess(cClass='', pyName='config.SetDictionary', cName='unused',
                        argList=['values'],
                        description='set the settings named in the dictionary and leave the others; a name that `exudyn.config` does not have, and one of the three that only report, is ignored',
                        returnType='None',
                        )
pb.DefPyFunctionAccess(cClass='', pyName='config.GetDefaults', cName='unused',
                        description='the defaults of `exudyn.config`: what Exudyn started with, taken while the module was imported and before a script, an override setting or an environment variable could change one. This is what tells a setting somebody chose from one nobody touched, which is how `overrideSettings.Store(config=...)` knows what to store. They cannot be constructed - `exudyn.config` reads global state, so a second one reports the current values - so they are taken once',
                        returnType='dict',
                        )
pb.EndCppWrittenByHand()

pb.AddDocu('', section='exudyn.experimental', sectionLevel=1, sectionLabel='sec-exudyn-experimental')
#++++++++++
pb.CppCode('        m.attr("experimental") = py::cast(&pyExperimental);\n') 
pb.DefDataAccess('experimental','Experimental features, not intended for regular users; for available features, see the C++ code class PyExperimental',
                       dataType='Experimental', isTopLevel = True)

pb.DefDataAccess('experimental.eigenFullPivotLUsolverDebugLevel','debug output of the EigenDense solver with full pivoting: 0 (default) = none, 1 = rank and information, 2 = also the matrices',
                       dataType='int', isTopLevel = True)
pb.DefDataAccess('experimental.markerSuperElementRigidTexpSO3','if nonzero (default), MarkerSuperElementRigid uses the additional tangent operator TexpSO3 of the rotation parameters',
                       dataType='int', isTopLevel = True)

pb.AddDocu('', section='exudyn.special', sectionLevel=1, sectionLabel='sec-exudyn-special')
pb.CppCode('        m.attr("special") = py::cast(&pySpecial);\n') 
pb.DefDataAccess('special','special attributes and functions, such as global (solver) flags or helper functions; not intended for regular users; for available features, see the C++ code class PySpecial',
                        dataType='Special', isTopLevel = True)

pb.BeginCppWrittenByHand() #pybindings added only in C++
pb.DefPyFunctionAccess(cClass='', pyName='special.InfoStat', cName='unused', 
                        description='Retrieve list of global information on memory allocation and other counts as list:[array_new_counts, array_delete_counts, vector_new_counts, vector_delete_counts, matrix_new_counts, matrix_delete_counts, linkedDataVectorCast_counts]; May be extended in future; if writeOutput==True, it additionally prints the statistics; counts for new vectors and matrices should not depend on numberOfSteps, except for some objects such as ObjectGenericODE2 and for (sensor) output to files; Not available if code is compiled with __FAST_EXUDYN_LINALG flag',
                        argList=['writeOutput'],
                        defaultArgs=['True'],
                        argTypes=[''],
                        returnType='List[int]',
                        )
pb.EndCppWrittenByHand()

pb.DefDataAccess('special.currentRendererSystemContainer',r'the `SystemContainer` the renderer is attached to, or `None`; the render engine can hold one at a time, which is why this is module-wide. It is set when a container attaches to the render engine and cleared when it detaches or is destroyed, and it is what the dialogs of `exudyn.misc.GUI` ask for; not intended for regular users',
                        dataType='Any', isTopLevel = True, readOnly = True)

pb.DefDataAccess('special.overrideSettings',r'the settings that persist between runs, read once by `import exudyn` from `~/.exudyn/config.json`: a dictionary with one key per section, `config`, `visualizationSettings`, `dialogs` and `resultsMonitor`; it is empty unless something was stored, and both Python and the C++ side read it. See Section [](#sec-usersettings)',
                        dataType='dict', isTopLevel = True, readOnly = True)

pb.AddDocu('', section='exudyn.special.solver', sectionLevel=2, sectionLabel='sec-exudyn-special-solver')
pb.DefDataAccess('special.solver','special solver attributes and functions; not intended for regular users; for available features, see the C++ code class PySpecialSolver',
                        dataType='SpecialSolver', isTopLevel = True)

pb.DefDataAccess('special.solver.timeout','if >= 0, the solver stops after reaching accoring CPU time specified with timeout; makes sense for parameter variation, automatic testing or for long-running simulations; default=-1 (no timeout)',
                        dataType='float', isTopLevel = True)
pb.DefDataAccess('special.solver.throwErrorWithCtrlC','if True, pressing CTRL-C during a simulation raises an error in Python; if False (default), the solver stops and returns; the error works in a console, the default also in Spyder',
                        dataType='bool', isTopLevel = True)
pb.DefDataAccess('special.solver.multiThreadingLoadBalancing','if True (=default), multithreaded code parts (in particular solver and raytracing) use load balancing, which may give better performance in case of non-equilibrated loads; (mobile) Intel CPUs may perform significantly better without load balancing',
                        dataType='bool', isTopLevel = True)

pb.AddDocu('', section='exudyn.special.exceptions', sectionLevel=2, sectionLabel='sec-exudyn-special-exceptions')
pb.DefDataAccess('special.exceptions','special flags for exceptions and checks; not intended for regular users; for available features, see the C++ code class PySpecialExceptions',
                        dataType='SpecialExceptions', isTopLevel = True)
pb.DefDataAccess('special.exceptions.dictionaryVersionMismatch','if True (=default), SetDictionary(...) of a settings structure warns if the dictionary comes from another version of Exudyn',
                        dataType='bool', isTopLevel = True)
pb.DefDataAccess('special.exceptions.dictionaryNonCopyable','if True (=default), GetDictionary(...) raises an error if a value cannot be copied into the dictionary',
                        dataType='bool', isTopLevel = True)
pb.DefDataAccess('special.exceptions.parameterRangeChecks','if True (=default), writing an item or settings parameter outside its range (e.g. a negative mass or a non-positive number of steps) raises an error, on every write path (item classes, dictionaries, SetObjectParameter, ...); set False to accept any value, e.g. if a range limit turns out to be wrong',
                        dataType='bool', isTopLevel = True)

pb.AddDocu('', section='exudyn.special.beams', sectionLevel=2, sectionLabel='sec-exudyn-special-beams')
pb.DefDataAccess('special.beams','switches of the beam elements, for tests and comparisons; a model keeps the defaults; for available features, see the C++ code class PySpecialBeams',
                        dataType='SpecialBeams', isTopLevel = True)
pb.DefDataAccess('special.beams.geometricallyExactLumpedMass','if True, ObjectBeamGeometricallyExact uses the lumped mass matrix of its nodes and their quadratic velocity vector instead of the element-consistent ones (default False); for comparisons and tests',
                        dataType='bool', isTopLevel = True)

pb.AddDocu('', section='exudyn.special.userInterface', sectionLevel=2, sectionLabel='sec-exudyn-special-userinterface')
pb.DefDataAccess('special.userInterface','flags that stop Exudyn from opening windows; meant for automated runs (test runners, CI, AI-assisted development), where a window that waits for a human stops everything; not intended for regular users; for available features, see the C++ code class PySpecialUserInterface',
                        dataType='SpecialUserInterface', isTopLevel = True)
pb.DefDataAccess('special.userInterface.suppressRenderer','if True, SC.renderer.Start() returns immediately without opening a window, IsActive() is False - so that a "while SC.renderer.IsActive()" loop ends at once - and DoIdleTasks() does nothing; default=False',
                        dataType='bool', isTopLevel = True)
pb.DefDataAccess('special.userInterface.suppressSolutionViewer','if True, mbs.SolutionViewer(...) and AnimateModes(...) return immediately instead of opening the viewer; default=False',
                        dataType='bool', isTopLevel = True)
pb.DefDataAccess('special.userInterface.suppressPlots','if True, PlotSensor and the other plotting helpers do not show a plot window; figures are still drawn and a figure given a file name is still saved; setting the environment variable EXUDYN_SUPPRESS_UI_WINDOW_OPEN additionally switches matplotlib to the non-interactive Agg backend, which also silences a plt.show() written in a script; default=False',
                        dataType='bool', isTopLevel = True)
pb.DefDataAccess('special.userInterface.suppressDialogs','if True, InteractiveDialog and the other tkinter dialogs return their defaults instead of opening a window; default=False',
                        dataType='bool', isTopLevel = True)

pb.BeginCppWrittenByHand() #bound in Pybind_manual_classes.cpp
pb.DefPyFunctionAccess(cClass='', pyName='special.userInterface.SuppressAll', cName='unused',
                        description='set all four suppress flags at once; a run either wants windows or does not',
                        argList=['flag'],
                        defaultArgs=['True'],
                        argTypes=['bool'],
                        returnType='None',
                        )
pb.EndCppWrittenByHand()

pb.AddDocu('', section='exudyn.special.deprecations', sectionLevel=2, sectionLabel='sec-exudyn-special-deprecations')
pb.DefDataAccess('special.deprecations','how a deprecated name - a setting, an item parameter, a function - is reported: a DeprecationWarning once per session and name, or on every use, and a count of every use in the dictionary exudyn.sys[deprecationUse], by source (simulationSettings, visualizationSettings, items, functions) and name; see the C++ code class PySpecialDeprecations',
                        dataType='SpecialDeprecations', isTopLevel = True)
pb.DefDataAccess('special.deprecations.warnOnce','if True (=default), each deprecated name warns once per session; if False, every use warns, e.g. to find all places in a script that use it',
                        dataType='bool', isTopLevel = True)
pb.DefDataAccess('special.deprecations.recordUse','if True (=default), every use of a deprecated name is counted in exudyn.sys[deprecationUse], by source and name, also when no warning is shown; a test reads it instead of catching warnings',
                        dataType='bool', isTopLevel = True)

pb.BeginCppWrittenByHand() #bound in Pybind_manual_classes.cpp
pb.DefPyFunctionAccess(cClass='', pyName='special.deprecations.Reset', cName='unused',
                        description='forget which names were warned about in this session, so that each one warns once more',
                        argList=[],
                        defaultArgs=[],
                        argTypes=[],
                        returnType='None',
                        )
pb.DefPyFunctionAccess(cClass='', pyName='special.deprecations.Warn', cName='unused',
                        description='report the use of a deprecated name from Python, as the C++ side does: a DeprecationWarning, once per session unless warnOnce is False, and a count in exudyn.sys[deprecationUse]; stackLevel=2 attributes the warning to the caller of the function that calls Warn',
                        argList=['source', 'name', 'message', 'stackLevel'],
                        defaultArgs=['', '', '', '2'],
                        argTypes=['str', 'str', 'str', 'int'],
                        returnType='None',
                        )
pb.EndCppWrittenByHand()

pb.EndNoStub()

pb.AddDocu('', section='exudyn.variables and exudyn.sys', sectionLevel=1, sectionLabel='sec-exudyn-variables')
pb.CppCode('        m.attr("variables") = exudynVariables;\n') 
pb.DefDataAccess('variables','this dictionary may be used by the user to store exudyn-wide data in order to avoid global Python variables; usage: exu.variables["myvar"] = 42; can be used in particular to exchange data between different mbs or between packages by importing exudyn.variables wherever needed.',
                       dataType='dict', isTopLevel = True)

pb.CppCode('        m.attr("sys") = exudynSystemVariables;\n') 
pb.DefDataAccess('sys',"this dictionary is used and reserved by the system, e.g., for testsuite, graphics or system function to store module-wide data in order to avoid global Python variables; the variable exu.sys['renderState'] contains the last render state after SC.renderer.Stop() and can be used for subsequent simulations ",
                       dataType='dict', isTopLevel = True)


pb.DefPyFinishClass('')


pb.EndStubSection()
