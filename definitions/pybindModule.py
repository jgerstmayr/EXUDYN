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

#++++++++++
pb.CppCode('        m.attr("experimental") = py::cast(&pyExperimental);\n') 
pb.DefDataAccess('experimental','Experimental features, not intended for regular users; for available features, see the C++ code class PyExperimental',
                       dataType='Experimental', isTopLevel = True)

pb.DefDataAccess('experimental.eigenFullPivotLUsolverDebugLevel','debug output of the EigenDense solver with full pivoting: 0 (default) = none, 1 = rank and information, 2 = also the matrices',
                       dataType='int', isTopLevel = True)
pb.DefDataAccess('experimental.markerSuperElementRigidTexpSO3','if nonzero (default), MarkerSuperElementRigid uses the additional tangent operator TexpSO3 of the rotation parameters',
                       dataType='int', isTopLevel = True)

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

pb.DefDataAccess('special.solver','special solver attributes and functions; not intended for regular users; for available features, see the C++ code class PySpecialSolver',
                        dataType='SpecialSolver', isTopLevel = True)

pb.DefDataAccess('special.solver.timeout','if >= 0, the solver stops after reaching accoring CPU time specified with timeout; makes sense for parameter variation, automatic testing or for long-running simulations; default=-1 (no timeout)',
                        dataType='float', isTopLevel = True)
pb.DefDataAccess('special.solver.throwErrorWithCtrlC','if True, pressing CTRL-C during a simulation raises an error in Python; if False (default), the solver stops and returns; the error works in a console, the default also in Spyder',
                        dataType='bool', isTopLevel = True)
pb.DefDataAccess('special.solver.multiThreadingLoadBalancing','if True (=default), multithreaded code parts (in particular solver and raytracing) use load balancing, which may give better performance in case of non-equilibrated loads; (mobile) Intel CPUs may perform significantly better without load balancing',
                        dataType='bool', isTopLevel = True)

pb.DefDataAccess('special.exceptions','special flags for exceptions and checks; not intended for regular users; for available features, see the C++ code class PySpecialExceptions',
                        dataType='SpecialExceptions', isTopLevel = True)
pb.DefDataAccess('special.exceptions.dictionaryVersionMismatch','if True (=default), SetDictionary(...) of a settings structure warns if the dictionary comes from another version of Exudyn',
                        dataType='bool', isTopLevel = True)
pb.DefDataAccess('special.exceptions.dictionaryNonCopyable','if True (=default), GetDictionary(...) raises an error if a value cannot be copied into the dictionary',
                        dataType='bool', isTopLevel = True)
pb.DefDataAccess('special.exceptions.parameterRangeChecks','if True (=default), writing an item or settings parameter outside its range (e.g. a negative mass or a non-positive number of steps) raises an error, on every write path (item classes, dictionaries, SetObjectParameter, ...); set False to accept any value, e.g. if a range limit turns out to be wrong',
                        dataType='bool', isTopLevel = True)

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

pb.DefDataAccess('special.currentRendererSystemContainer',r'the `SystemContainer` the renderer is attached to, or `None`; the render engine can hold one at a time, which is why this is module-wide. It is set when a container attaches to the render engine and cleared when it detaches or is destroyed, and it is what the dialogs of `exudyn.misc.GUI` ask for; not intended for regular users',
                        dataType='Any', isTopLevel = True, readOnly = True)

pb.DefDataAccess('special.overrideSettings',r'the settings that persist between runs, read once by `import exudyn` from `~/.exudyn/config.json`: a dictionary with one key per section, `config`, `visualizationSettings`, `dialogs` and `resultsMonitor`; it is empty unless something was stored, and both Python and the C++ side read it. See Section [](#sec-overridesettings)',
                        dataType='dict', isTopLevel = True, readOnly = True)

pb.EndNoStub()

pb.CppCode('        m.attr("variables") = exudynVariables;\n') 
pb.DefDataAccess('variables','this dictionary may be used by the user to store exudyn-wide data in order to avoid global Python variables; usage: exu.variables["myvar"] = 42; can be used in particular to exchange data between different mbs or between packages by importing exudyn.variables wherever needed.',
                       dataType='dict', isTopLevel = True)

pb.CppCode('        m.attr("sys") = exudynSystemVariables;\n') 
pb.DefDataAccess('sys',"this dictionary is used and reserved by the system, e.g., for testsuite, graphics or system function to store module-wide data in order to avoid global Python variables; the variable exu.sys['renderState'] contains the last render state after SC.renderer.Stop() and can be used for subsequent simulations ",
                       dataType='dict', isTopLevel = True)


pb.DefPyFinishClass('')


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the override settings and the environment variables: the mechanisms that change what the module
#does before a script says anything (#2680)
pb.AddDocu(r"""A setting that is changed on every start - the output directory, the multi-sampling of
the renderer, the size of the basis vectors - is stored once and taken by every run afterwards.
Exudyn reads one file for that, `~/.exudyn/config.json`:""",
           section='Settings that persist between runs', sectionLevel=2,
           sectionLabel='sec-overridesettings')

pb.AddDocuCodeBlock(pythonStyle=False, code="""
{
  "version": 1,
  "config": {"outputDirectory": "solution/"},
  "visualizationSettings": {"openGL.multiSampling": 4, "nodes.basisSize": 0.5}
}
""")

pb.AddDocu(r"""Nothing writes this file by itself. A script that behaves differently on another
machine, because something was stored there, is the one thing a settings file must not cause, so
storing is always asked for:""")

pb.AddDocuCodeBlock(code="""
from exudyn.misc import overrideSettings

SC.visualizationSettings.openGL.multiSampling = 4
overrideSettings.Store(SC)         #every run from now on starts with it
""")

pb.AddDocu(r"""`Store(SC)` writes the settings that differ from the defaults - the same list
the settings dialog shows as *changed* - and `Store(config=exudyn.config)` does the same for
`exudyn.config`. `overrideSettings.Clear()` deletes the file.

**The `version` is the format of the file**, and it has to match: a file of another version is
ignored, with one note naming both. It is a plain integer that moves only when Exudyn has been
released *and* the meaning of something in this file has changed - not with the Exudyn version, which
moves on every resolved issue. Store your settings again to write a current file.

#### What happens when the file is there

`import exudyn` reads it once into `exudyn.special.overrideSettings` and prints
**one note** naming how many settings it took:""")

pb.AddDocuCodeBlock(pythonStyle=False, code="""
NOTE: 1 config settings and 2 visualizationSettings read from ~/.exudyn/config.json
""")

pb.AddDocu(r"""A count that is 0 is left out. `overrideSettings.Print()` lists what came from the file, which
is the answer to *why does this script behave differently here*, `overrideSettings.Applied()` gives
the same as a list, for a script that wants to print it into its own output, and
`EXUDYN_NO_USER_SETTINGS=1` runs a script as if there were no file.

**A file that should stay quiet** says so itself, beside its `version`:

```
{
  "version": 1,
  "suppressOverrideSettingsWarning": true,
  "visualizationSettings": {"openGL.multiSampling": 4}
}
```

The key does not exist unless it is written, and without it the note is printed. It suppresses the
note only: a setting that could not be applied is still reported, because that is a defect and not
information.

**What happens, in order:**

1. the whole file is read into `exudyn.special.overrideSettings`, once, by `import exudyn`;
2. what can be applied at once is applied at once, which is the `config` section;
3. the rest stays in the dictionary, because the things it sets do not exist yet;
4. every `VisualizationSettings` structure applies the stored settings **when it is created** - the
   one a `SystemContainer` builds, and one built by `exu.VisualizationSettings()`. A setting that
   names nothing is reported and changes nothing;
5. a dialog reads its size and position from the `dialogs` section when it opens, and the results
   monitor its own settings from `resultsMonitor`;
6. nothing writes the file unless it is asked to.

**The file is read once**, so a file you edit while a session is running - or one that another
session stored - has no effect until you read it again. That is what makes a stored setting look as
if it had not been stored in a console that keeps its kernel, such as Spyder:

```python
from exudyn.misc import overrideSettings
overrideSettings.Reload()          #read it again and apply what can be applied
overrideSettings.Print()           #what came from it now
```

A reload does not **undo**: a setting that already reached `exudyn.config`, and a structure that
already exists, keep what they were given. A setting you removed from the file is seen by the
structures created after the reload; for everything else, start a new session.

A stored `visualizationSetting` is **not** a new default: `exu.VisualizationSettings()` carries it,
and the defaults the settings dialog compares against are still the defaults of Exudyn, so *diff to
default* shows a stored setting as a difference. That is the point - it is what the file changed.

**What may be stored are plain values**: a number, a flag, a string, or a list of numbers - and an
enum setting, `contour.outputVariable`, as the name of its value, `"StressLocal"`, which is read back
by that name; `interactive.highlightItemType` is the state of a highlight and is not stored. A setting
that holds graphics data, a user function or a matrix container is refused with a message and
changes nothing - such a value cannot be carried honestly by a JSON file. A key that names no
setting, a section nobody reads, a file that is not valid JSON: each of them is reported and none of
them stops `import exudyn`.

#### Storing from the dialog

The settings dialog has a **store settings** button. It writes the `visualizationSettings` that
differ from the defaults, and the size and position of the dialog itself, and **nothing else** - not
`exudyn.config`, not the simulation settings - and it shows exactly what it is about to write before
it writes anything. It is the way to keep a look you have just made without switching
`storeDialogPositions` on.

**diff to default** stays a difference to the *default*, and a setting that the file already stores
is listed with the others and then named again under a comment line that says so. Comparing against
the defaults plus the file instead would hide exactly the settings the file is about.

#### Placing a dialog, and a window, from a script

A script can say where a dialog opens, which is the same mechanism the dialogs use for themselves:

```python
from exudyn.misc import overrideSettings

#the visualization settings dialog, 1024x768 at (100, 80):
overrideSettings.StoreDialogGeometry('Visualization Settings', [1024, 768], [100, 80])
```

The name is the **title** of the dialog, and `overrideSettings.DialogKey(name)` is the key it is
stored under, so the same call places the solution viewer (`'Solution Viewer'`) or any other dialog.
The **size comes back always and the position only if the window would still be reachable** on the
screen you have now - a monitor that is gone must not put a dialog where its title bar cannot be
grabbed. This writes `~/.exudyn/config.json`, so it holds for every run afterwards.

The **render window** is not a dialog and is placed by its own settings, per view:

```python
SC.visualizationSettings.view0.window.renderWindowSize = [1024, 768]
SC.visualizationSettings.view0.window.renderWindowPosition = [100, 80]
```

A negative coordinate - the default - means the window manager places it. The position is that of the
OpenGL area rather than the title bar, so a small value hides part of the title bar and 0 hides it
completely, which is a way to have a view without one.

**The render window is the one window whose geometry lives in two places**: these settings, and -
because they are ordinary settings - the `visualizationSettings` section of the file. The file is
applied when the settings structure is created and a script speaks afterwards, so **what the script
sets wins**, and `SC.renderer.Start()` says so once when the two differ:

```
Python WARNING: the render window geometry stored in the settings file differs from what this session
set, and what the session set is used:
  view0.window.renderWindowPosition: the file says [100,80] and this session uses [500,400]
store the settings again to change the file, or remove them from it
```

It says nothing when they agree, which is the normal case for someone who stored the geometry and has
not touched it since. `view*.window.storeRenderWindowGeometry` writes where the window was back into
these settings when it closes, and `SC.renderer.GetState()['currentWindowPosition']` is where it is
while it is open.

#### The plot windows of PlotSensor

A plot window is remembered by its **sequence**, the order `PlotSensor` made it in, because plot
windows have no title of their own: the first one of a run is stored as `'PlotSensor 1'`, the second
as `'PlotSensor 2'`, and `PlotSensor(..., closeAll=True)` starts that order over. They are stored in
the same `dialogs` section, with the same rules - the size comes back always, the position only if the
window would still be reachable - so the next run opens the plots where they were arranged.

Storing them is asked for, once the windows are where they should be:

```python
from exudyn.plot import StorePlotWindowGeometry

#after arranging the plot windows on the screen:
StorePlotWindowGeometry()          #returns how many windows it stored
```

This is the call to use when the plots are made after `SC.renderer.Stop()`, when the settings dialog
is gone; while it is open, its **store positions** button stores the plot windows together with the
other windows. `PlotSensorDefaults().storeWindowPositions
= True` in addition stores each window when it closes, one window at a time, which asks nothing but
also keeps whatever a window happened to be when it was closed.

A plot window is only placed by a backend that has one: with matplotlib on `Agg` - which
`EXUDYN_SUPPRESS_UI_WINDOW_OPEN` selects - there is no window, nothing is placed and nothing is
stored.

**For this run only**, a script can write into `exudyn.special.overrideSettings` instead of the file:

```python
exudyn.special.overrideSettings['dialogs'] = {
    'visualizationsettings': {'size': [1024, 768], 'position': [100, 80]}}
```

That is read by the next dialog that opens and the file is not touched. It is **not the recommended
way**: nothing checks what is put there, and a value of the wrong shape is simply not used - the
functions above are what say what they mean, and what a later Exudyn will keep working.

#### Remembering a dialog, its columns and its font

`visualizationSettings.dialogs.storeDialogPositions = True` makes a settings dialog remember
where it was left, under `"dialogs"`, one entry per dialog. **The size comes back always; the
position only when the window would still be reachable.** A monitor that is unplugged, a laptop
undocked, a screen resolution that changed: each of them would otherwise put the dialog where nobody
can reach its title bar, and a dialog that cannot be closed is a stuck session. When the stored
position is not usable, the dialog opens where it would have opened anyway, at its remembered size.

The three fixed columns of a settings dialog take a share of its width, each a fraction in
`visualizationSettings.dialogs` - `columnWidthName`, `columnWidthValue`,
`columnWidthType` - and the description column takes what they leave; if the three together
would leave the description less than a tenth of the dialog, all three are scaled down to leave it
that much. **Ctrl and the mouse wheel change the font size** of an open dialog, about 10\% per notch;
the wheel alone still scrolls.

(sec-environmentvariables)=
#### The environment variables

These change what Exudyn does before a script says anything, which is what makes them worth knowing
when a run behaves differently than it reads:

| variable | what it does |
|---|---|
| `EXUDYN_NO_USER_SETTINGS` | anything but empty or `0` ignores `~/.exudyn/config.json` completely, so that a run starts from the defaults. `runTestSuite.py`, `runTestExamples.py`, `runPerformanceTests.py` and `pytest` set it for themselves, so a stored setting can never move a test result |
| `EXUDYN_CONFIG_FILE` | names a different settings file, for a second configuration or for a test |
| `EXUDYN_OUTPUTDIRECTORY` | the initial value of `exudyn.config.outputDirectory`: every solution and sensor file a model writes goes there, which is how a model is run without writing into the working directory |
| `EXUDYN_SUPPRESS_UI_WINDOW_OPEN` | switches on all of `exudyn.special.userInterface`: no renderer, no solution viewer, no plot window and no dialog, and matplotlib is switched to its non-interactive backend. Meant for automated runs, where a window that waits for a human stops everything |
| `EXUDYN_MODULE` | `fast` loads `exudynCPPfast` - no range checks, AVX2 - instead of the regular module; it belongs to release testing, because without range checks a wrong index is undefined behaviour instead of an exception |
| `EXUDYN_IMPORT_VERBOSE` | 1 prints which compiled module was tried and what came of it, which is the whole answer to *it imports the wrong one* |

**Setting one for every run in Spyder**: Spyder starts its consoles itself, so a variable is set
where a console starts. In *Tools > Preferences > IPython console > Startup*, under *Run code*, the
lines

```python
import os; os.environ['EXUDYN_OUTPUTDIRECTORY'] = 'solution'
```

make every run of a restarted console write its output into `solution/` beside the script it runs -
with the working directory set to the directory of the file being executed, which is Spyder's
default. A script that writes to `solution/...` itself then writes to `solution/solution/...`.
Outside Spyder, `setx EXUDYN_OUTPUTDIRECTORY solution` sets the variable for every program the user
starts afterwards.

`EXUDYN_NO_USER_SETTINGS=1` is also what makes a problem reproducible on a machine that has
stored something: run the script with the variable set and the difference is either gone (the
stored setting caused it) or still there (it did not).
""")


pb.EndStubSection()
