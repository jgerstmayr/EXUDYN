# -*- coding: utf-8 -*-
"""
Created on Fri May 18 08:53:30 2018

@author: Johannes Gerstmayr

automatically generate pybindings for specific classes and functions AND latex documentation for these functions
"""

from autoGenerateHelper import DefPyFunctionAccess, DefPyStartClass, DefPyFinishClass, DefLatexStartClass, DefLatexFinishClass, GetDateStr, AddEnumValue


s = ''  #C++ pybind local includes
sL = '' #Latex documentation

#+++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++
#structures and enums:

sLenum = '\section{Type definitions}\nThis section defines a couple of structures, which are used to select, e.g., a configuration type or a variable type. In the background, these types are integer numbers, but for safety, the types should be used as type variables. \n\n'
sLenum+= 'Conversion to integer is possible: \n \\bi \n \\item[] \\texttt{x = int(exu.OutputVariableType.Displacement)} \n\\ei and also conversion from integer: \n \\bi \n \\item[] \\texttt{varType = exu.OutputVariableType(8)}\n \\ei\n'
s += '\n//        pybinding to enum classes:\n'

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'OutputVariableType'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for selecting output values, e.g. for GetObjectOutput(...) or for selecting variables for contour plot.\n\n'
descriptionStr += 'Available output variables and the interpreation of the output variable can be found at the object definitions. \n The OutputVariableType does not provide information about the size of the output variable, which can be either scalar or a list (vector). For vector output quantities, the contour plot option offers an additional parameter for selection of the component of the OutputVariableType.\n'

s +=	'  py::enum_<' + pyClass + '>(m, "' + pyClass + '")\n'
sLenum += DefLatexStartClass(sectionName = pyClass, 
                            description=descriptionStr, 
                            subSection=True)
#keep this list synchronized with the accoring enum structure in C++!!!
[s1,sL1] = AddEnumValue(pyClass, 'None', 'no value; used, e.g., to select no output variable in contour plot'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Distance', 'e.g., measure distance in spring damper connector'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Position', 'measure 3D position, e.g., of node or body'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Displacement', 'measure displacement; usually difference between current position and reference position'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Velocity', 'measure (translational) velocity of node or object'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Acceleration', 'measure (translational) acceleration of node or object'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'RotationMatrix', 'measure rotation matrix of rigid body node or object'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'AngularVelocity', 'measure angular velocity of node or object'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'AngularVelocityLocal', 'measure local (body-fixed) angular velocity of node or object'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'AngularAcceleration', 'measure angular acceleration of node or object'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Rotation', 'measure, e.g., scalar rotation of 2D body, Euler angles of a 3D object or rotation within a joint'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Coordinates', 'measure the coordinates of a node or object; coordinates usually just contain displacements, but not the position values'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Coordinates_t', 'measure the time derivative of coordinates (= velocity coordinates) of a node or object'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'SlidingCoordinate', 'measure sliding coordinate in sliding joint'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Director1', 'measure a director (e.g. of a rigid body frame), or a slope vector in local 1 or x-direction'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Director2', 'measure a director (e.g. of a rigid body frame), or a slope vector in local 2 or y-direction'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Director3', 'measure a director (e.g. of a rigid body frame), or a slope vector in local 3 or z-direction'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Force', 'measure force, e.g., in joint or beam (resultant force)'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Torque', 'measure torque, e.g., in joint or beam (resultant couple/moment)'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Strain', 'measure strain, e.g., axial strain in beam'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Stress', 'measure stress, e.g., axial stress in beam'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Curvature', 'measure curvature; may be scalar or vectorial: twist and curvature'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'EndOfEnumList', 'this marks the end of the list, usually not important to the user'); s+=s1; sLenum+=sL1

s +=	'		.export_values();\n\n'
sLenum += DefLatexFinishClass()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'ConfigurationType'
#	py::enum_<ConfigurationType>(m, "ConfigurationType")
#		.value("None", ConfigurationType::None)
#		.value("Initial", ConfigurationType::Initial)
#		.value("Current", ConfigurationType::Current)
#		.value("Reference", ConfigurationType::Reference)
#		.value("StartOfStep", ConfigurationType::StartOfStep)
#		.value("Visualization", ConfigurationType::Visualization)
#		.value("EndOfEnumList", ConfigurationType::EndOfEnumList)
#		.export_values();

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for selecting a configuration for reading or writing information to the module. Specifically, the ConfigurationType.Current configuration is usually used at the end of a solution process, to obtain result values, or the ConfigurationType.Initial is used to set initial values for a solution process.\n\n'

s +=	'  py::enum_<' + pyClass + '>(m, "' + pyClass + '")\n'
sLenum += DefLatexStartClass(sectionName = pyClass, 
                            description=descriptionStr, 
                            subSection=True)
#keep this list synchronized with the accoring enum structure in C++!!!
[s1,sL1] = AddEnumValue(pyClass, 'None', 'no configuration; usually not valid, but may be used, e.g., if no configurationType is required'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Initial', 'initial configuration prior to static or dynamic solver; is computed during mbs.Assemble() or AssembleInitializeSystemCoordinates()'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Current', 'current configuration during and at the end of the computation of a step (static or dynamic)'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Reference', 'configuration used to define deformable bodies (reference configuration for finite elements) or joints (configuration for which some joints are defined)'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'StartOfStep', 'during computation, this refers to the solution at the start of the step = end of last step, to which the solver falls back if convergence fails'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'Visualization', 'this is a state completely de-coupled from computation, used for visualization'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'EndOfEnumList', 'this marks the end of the list, usually not important to the user'); s+=s1; sLenum+=sL1

s +=	'		.export_values();\n\n'
sLenum += DefLatexFinishClass()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'LinearSolverType'
#	py::enum_<LinearSolverType>(m, "LinearSolverType")
#		.value("None", LinearSolverType::None)
#		.value("EXUdense", LinearSolverType::EXUdense)
#		.value("EigenSparse", LinearSolverType::EigenSparse)
#		.export_values();

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for selecting output values, e.g. for GetObjectOutput(...) or for selecting variables for contour plot.\n\n'

s +=	'  py::enum_<' + pyClass + '>(m, "' + pyClass + '")\n'
sLenum += DefLatexStartClass(sectionName = pyClass, 
                            description=descriptionStr, 
                            subSection=True)
#keep this list synchronized with the accoring enum structure in C++!!!
[s1,sL1] = AddEnumValue(pyClass, 'None', 'no value; used, e.g., if no solver is selected'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'EXUdense', 'use dense matrices and according solvers for densly populated matrices (usually the CPU time grows cubically with the number of unknowns)'); s+=s1; sLenum+=sL1
[s1,sL1] = AddEnumValue(pyClass, 'EigenSparse', 'use sparse matrices and according solvers; additional overhead for very small systems; specifically, memory allocation is performed during a factorization process'); s+=s1; sLenum+=sL1

s +=	'		.export_values();\n\n'
sLenum += DefLatexFinishClass()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Access functions to EXUDYN
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
[s1,sL1] = DefPyStartClass('','', 'These are the access functions to the \\codeName\\ module.'); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess('', 'Go', 'PythonGo', 'Creates a SystemContainer SC and a main system mbs'); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess('', 'InfoStat', 'PythonInfoStat', 'Print some global (debug) information: linear algebra, memory allocation, threads, computational efficiency, etc.'); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess('', 'StartRenderer', 'PyStartOpenGLRenderer', "Start OpenGL rendering engine (in separate thread)"); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess('', 'StopRenderer', 'PyStopOpenGLRenderer', "Stop OpenGL rendering engine"); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass='', pyName='SetOutputPrecision', cName='PySetOutputPrecision', 
                                description="Set the precision (integer) for floating point numbers written to console (reset when simulation is started!)",
                                argList=['numberOfDigits']); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass='', pyName='SetLinalgOutputFormatPython', cName='PySetLinalgOutputFormatPython', 
                                description="true: use python format for output of vectors and matrices; false: use matlab format",
                                argList=['flagPythonFormat']); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess('', 'InvalidIndex', 'GetInvalidIndex', 
                            "This function provides the invalid index, which depends on the kind of 32-bit, 64-bit signed or unsigned integer; e.g. node index or item index in list"); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass='', pyName='SetWriteToConsole', cName='PySetWriteToConsole', 
                            description="set flag to write (true) or not write to console; default = true",
                            argList=['flag']); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass='', pyName='SetWriteToFile', cName='PySetWriteToFile', 
                            description="set flag to write (true) or not write to console; default value of flagWriteToFile = false; flagAppend appends output to file, if set true; in order to finalize the file, write exu.SetWriteToFile('', False) to close the output file",
                            argList=['filename', 'flagWriteToFile', 'flagAppend'],
                            defaultArgs=['', 'true', 'false'],
                            example="exu.SetWriteToConsole(False) \\#no output to console\\\\exu.SetWriteToFile(filename='testOutput.log', flagWriteToFile=True, flagAppend=False)\\\\exu.Print('print this to file')\\\\exu.SetWriteToFile('', False) \\#terminate writing to file which closes the file"
                            ); s+=s1; sL+=sL1


[s1,sL1] = DefPyFunctionAccess(cClass='', pyName='SetPrintDelayMilliSeconds', cName='PySetPrintDelayMilliSeconds', 
                            description="add some delay (in milliSeconds) to printing to console, in order to let Spyder process the output; default = 0",
                            argList=['delayMilliSeconds']); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass='', pyName='Print', cName='PyPrint', 
                            description="this allows printing via exudyn with similar syntax as in python print(args) except for keyword arguments: print('test=',42); allows to redirect all output to file given by SetWriteToFile(...); does not output in case that SetWriteToConsole is set to false",
                            #argList=['pyObject'] #not compatible with py::args
                            ); s+=s1; sL+=sL1

#s += '        m.def_readwrite("variables", &exudynVariables, py::return_value_policy::reference)\n' 
#variables in the module itself are exported with "m.attr(...)" !
s += '        m.attr("variables") = exudynVariables;\n' 
sL += '  variables & this dictionary may be used by the user to store exudyn-wide data in order to avoid global python variables; usage: exu.variables["myvar"] = 42 \\\\ \\hline  \n'

s += '        m.attr("sys") = exudynSystemVariables;\n' 
sL += '  sys & this dictionary is used by the system, e.g. for testsuite or solvers to store exudyn-wide data in order to avoid global python variables \\\\ \\hline  \n'

[s1,sL1] = DefPyFinishClass('')
s+=s1; sL+=sL1





#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#currently, only latex binding:
pyClassStr = 'SystemContainer'
classStr = 'Main'+pyClassStr
[s1,sL1] = DefPyStartClass(classStr, pyClassStr, 'The SystemContainer is the top level of structures in \\codeName. The container holds all systems, solvers and all other data structures for computation. Currently, only one container shall be used. In future, multiple containers might be usable at the same time.' +
        ' \\\\ Example: \\\\ \\texttt{import exudyn as exu \\\\ SC = exu.SystemContainer() \\\\ mbs = SC.AddSystem()}')
sL+=sL1

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#GENERAL FUNCTIONS

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='AddSystem', cName='AddMainSystem', 
                                description="add a new computational system", options='py::return_value_policy::reference'); sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='Reset', cName='Reset', 
                                description="delete all systems and reset SystemContainer (including graphics)"); sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='NumberOfSystems', cName='NumberOfSystems', 
                                description="obtain number of systems available in system container"); sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetSystem', cName='GetMainSystem', 
                                description="obtain systems with index from system container",
                                argList=['systemNumber']); sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='WaitForRenderEngineStopFlag', cName='WaitForRenderEngineStopFlag', 
                                description="Wait for user to stop render engine (Press 'Q' or Escape-key)"); sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='RenderEngineZoomAll', cName='PyZoomAll', 
                                description="Send zoom all signal, which will perform zoom all at next redraw request"); sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetRenderState', cName='PyGetRenderState', 
                                description="Get dictionary with current render state (openGL zoom, modelview, etc.)",
                                example = "SC = exu.SystemContainer()\\\\d = SC.GetRenderState() \\\\print(d['zoom'])"
                                ); sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='RedrawAndSaveImage', cName='RedrawAndSaveImage', 
                                description="Redraw openGL scene and save image (command waits until process is finished)"); sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='TimeIntegrationSolve', cName="""[](MainSystemContainer& msc, MainSystem& ms, HString solverName, const SimulationSettings& simulationSettings) {
                            		pout.precision(simulationSettings.outputPrecision);
                            		if (solverName == "RungeKutta1")
                            			msc.GetSolvers().GetSolverRK1().SolveSystem(simulationSettings, *(ms.GetCSystem()));
                            		else if (solverName == "GeneralizedAlpha")
                            			msc.GetSolvers().GetSolverGeneralizedAlpha().SolveSystem(simulationSettings, *(ms.GetCSystem()));
                            		else
                            			PyError(HString("SystemContainer::TimeIntegrationSolve: invalid solverName '")+solverName+"'; options are: RungeKutta1 or GeneralizedAlpha");
                            		}""", 
                                argList=['mainSystem','solverName','simulationSettings'],
                                description="Call time integration solver for given system with solverName ('RungeKutta1'...explicit solver, 'GeneralizedAlpha'...implicit solver); use simulationSettings to individually configure the solver",
                                example = "simSettings = exu.SimulationSettings()\\\\simSettings.timeIntegration.numberOfSteps = 1000\\\\simSettings.timeIntegration.endTime = 2\\\\simSettings.timeIntegration.verboseMode = 1\\\\SC.TimeIntegrationSolve(mbs,'GeneralizedAlpha',simSettings)",
                                isLambdaFunction = True
                                ); sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='StaticSolve', cName="""[](MainSystemContainer& msc, MainSystem& ms, const SimulationSettings& simulationSettings) {
                                pout.precision(simulationSettings.outputPrecision);
                                msc.GetSolvers().GetSolverStatic().SolveSystem(simulationSettings, *(ms.GetCSystem()));
                                }""", 
                                argList=['mainSystem','simulationSettings'],
                                description="Call solver to compute a static solution of the system, considering acceleration and velocity coordinates to be zero (initial velocities may be considered by certain objects)",
                                example = "simSettings = exu.SimulationSettings()\\\\simSettings.staticSolver.newton.relativeTolerance = 1e-6\\\\SC.StaticSolve(mbs, simSettings)",
                                isLambdaFunction = True
                                ); sL+=sL1

#s += '        .def_property("visualizationSettings", &MainSystemContainer::PyGetVisualizationSettings, &MainSystemContainer::PySetVisualizationSettings)\n' 
sL += '  visualizationSettings & this structure is read/writeable and contains visualization settings, which are immediately applied to the rendering window. \\tabnewline\n    EXAMPLE:\\tabnewline\n    SC = exu.SystemContainer()\\tabnewline\n    SC.visualizationSettings.autoFitScene=False  \\\\ \\hline  \n'

sL += DefLatexFinishClass()#only finalize latex table



#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
classStr = 'MainSystem'
[s1,sL1] = DefPyStartClass(classStr, classStr, "This is the structure which defines a (multibody) system. In C++, there is a MainSystem (links to python) and a System (computational part). For that reason, the name is MainSystem on the python side, but it is often just called 'system'. It can be created, visualized and computed. " + "Use the following functions for system manipulation." +
        ' \\\\ \\\\ Usage: \\\\ \\\\ \\texttt{import exudyn as exu \\\\ SC = exu.SystemContainer() \\\\ mbs = SC.AddSystem()}')
s+=s1; sL+=sL1

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#GENERAL FUNCTIONS

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='Assemble', cName='Assemble', 
                                description="assemble items (nodes, bodies, markers, loads, ...); Calls CheckSystemIntegrity(...), AssembleCoordinates(), AssembleLTGLists(), and AssembleInitializeSystemCoordinates()"); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='AssembleCoordinates', cName='AssembleCoordinates', 
                                description="assemble coordinates: assign computational coordinates to nodes and constraints (algebraic variables)"); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='AssembleLTGLists', cName='AssembleLTGLists', 
                                description="build local-to-global (ltg) coordinate lists for objects (used to build global ODE2RHS, MassMatrix, etc. vectors and matrices)"); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='AssembleInitializeSystemCoordinates', cName='AssembleInitializeSystemCoordinates', 
                                description="initialize all system-wide coordinates based on initial values given in nodes"); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='Reset', cName='Reset', 
                                description="reset all lists of items (nodes, bodies, markers, loads, ...) and temporary vectors; deallocate memory"); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='WaitForUserToContinue', cName='WaitForUserToContinue', 
                                description="interrupt further computation until user input --> 'pause' function"); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='SendRedrawSignal', cName='SendRedrawSignal', 
                                description="this function is used to send a signal to the renderer that the scene shall be redrawn because the visualization state has been updated"); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetRenderEngineStopFlag', cName='GetRenderEngineStopFlag', 
                                description="get the current stop simulation flag; true=user wants to stop simulation"); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='SetRenderEngineStopFlag', cName='SetRenderEngineStopFlag', 
                                description="set the current stop simulation flag; set to false, in order to continue a previously user-interrupted simulation"); s+=s1; sL+=sL1

#++++++++++++++++

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='__repr__', cName='[](const MainSystem &ms) {\n            return "<systemData: \\n" + ms.GetMainSystemData().PyInfoSummary() + "\\nmainSystem:\\n  variables = " + EXUstd::ToString(ms.variables) + "\\n  sys = " + EXUstd::ToString(ms.systemVariables) + "\\n>\\n"; }', 
                                description="return the representation of the system, which can be, e.g., printed",
                                isLambdaFunction = True,
                                example = 'print(mbs)'); s+=s1; sL+=sL1

s += '        .def_property("systemIsConsistent", &MainSystem::GetFlagSystemIsConsistent, &MainSystem::SetFlagSystemIsConsistent)\n' 
sL += '  systemIsConsistent & this flag is used by solvers to decide, whether the system is in a solvable state; this flag is set to false as long as Assemble() has not been called; any modification to the system, such as Add...(), Modify...(), etc. will set the flag to false again; this flag can be modified (set to true), if a change of e.g.~an object (change of stiffness) or load (change of force) keeps the system consistent, but would normally lead to systemIsConsistent=False  \\\\ \\hline  \n'

s += '        .def_property("interactiveMode", &MainSystem::GetInteractiveMode, &MainSystem::SetInteractiveMode)\n' 
sL += '  systemIsConsistent & set this flag to true in order to invoke a Assemble() command in every system modification, e.g. AddNode, AddObject, ModifyNode, ...; this helps that the system can be visualized in interactive mode. \\\\ \\hline  \n'

s += '        .def_readwrite("variables", &MainSystem::variables, py::return_value_policy::reference)\n' 
sL += '  variables & this dictionary may be used by the user to store model-specific data, in order to avoid global python variables in complex models; mbs.variables["myvar"] = 42 \\\\ \\hline  \n'

s += '        .def_readwrite("sys", &MainSystem::systemVariables, py::return_value_policy::reference)\n' 
sL += '  sys & this dictionary is used by exudyn python libraries, e.g., solvers, to avoid global python variables \\\\ \\hline \n'

s += '        .def_property("solverSignalJacobianUpdate", &MainSystem::GetFlagSolverSignalJacobianUpdate, &MainSystem::SetFlagSolverSignalJacobianUpdate)\n' 
sL += '  solverSignalJacobianUpdate & this flag is used by solvers to decide, whether the jacobian should be updated; at beginning of simulation and after jacobian computation, this flag is set automatically to False; use this flag to indicate system changes, e.g. during time integration  \\\\ \\hline  \n'

s += '        .def_readwrite("systemData", &MainSystem::mainSystemData, py::return_value_policy::reference)\n' 
sL += '  systemData & Access to SystemData structure; enables access to number of nodes, objects, ... and to (current, initial, reference, ...) state variables (ODE2, AE, Data,...)\\\\ \\hline  \n'


sL += DefLatexFinishClass()#only finalize latex table

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#NODE
s += "\n//        NODES:\n"
sL+=DefLatexStartClass(classStr+': Node', 'This section provides functions for adding, reading and modifying nodes. Nodes are used to define coordinates (unknowns to the static system and degrees of freedom if constraints are not present). Nodes can provide various types of coordinates for second/first order differential equations (ODE2/ODE1), algebraic equations (AE) and for data (history) variables -- which are not providing unknowns in the nonlinear solver but will be solved in an additional nonlinear iteration for e.g. contact, friction or plasticity.', subSection=True)
[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='AddNode', cName='[](MainSystem& mainSystem, py::dict itemDict) {return mainSystem.AddMainNode(itemDict); }', 
                                description="add a node with nodeDefinition in dictionary format; returns (global) node number of newly added node",
                                argList=['itemDict'],
                                example="nodeDict = {'nodeType': 'Point', \\\\'referenceCoordinates': [1.0, 0.0, 0.0], \\\\'initialDisplacements': [0.0, 2.0, 0.0], \\\\'name': 'example node'} \\\\ mbs.AddNode(nodeDict)",
                                isLambdaFunction = True
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='AddNode', cName='[](MainSystem& mainSystem, py::object pyObject) {return mainSystem.AddMainNodePyClass(pyObject); }', 
                                description="add a node with nodeDefinition from Python node class; returns (global) node number of newly added node",
                                argList=['pyObject'],
                                example = "item = Rigid2D( referenceCoordinates= [1,0.5,0], initialVelocities= [10,0,0]) \\\\mbs.AddNode(item)",
                                isLambdaFunction = True
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetNodeNumber', cName='PyGetNodeNumber', 
                                description="get node's number by name (string)",
                                argList=['nodeName'],
                                example = "n = mbs.GetNodeNumber('example node')"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetNode', cName='PyGetNode', 
                                description="get node's dictionary by index",
                                argList=['nodeNumber'],
                                example = "nodeDict = mbs.GetNode(0)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='ModifyNode', cName='PyModifyNode', 
                                description="modify node's dictionary by index",
                                argList=['nodeNumber','nodeDict'],
                                example = "mbs.ModifyNode(nodeNumber, nodeDict)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetNodeDefaults', cName='PyGetNodeDefaults', 
                                description="get node's default values for a certain nodeType as (dictionary)",
                                argList=['typeName'],
                                example = "nodeType = 'Point'\\\\nodeDict = mbs.GetNodeDefaults(nodeType)"
                                ); s+=s1; sL+=sL1

#[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='CallNodeFunction', cName='PyCallNodeFunction', 
#                                description="call specific node function",
#                                argList=['nodeNumber', 'functionName', 'args'],
#                                defaultArgs=['', '', 'py::dict()']
#                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetNodeOutput', cName='PyGetNodeOutputVariable', 
                                description="get the ouput of the node specified with the OutputVariableType; default configuration = 'current'; output may be scalar or array (e.g. displacement vector)",
                                argList=['nodeNumber','variableType','configuration'],
                                defaultArgs=['','','ConfigurationType::Current'],
                                example = "mbs.GetNodeOutput(nodeNumber=0, variableType='exu.OutputVariable.Displacement')"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetNodeODE2Index', cName='PyGetNodeODE2Index', 
                                description="get index in the global ODE2 coordinate vector for the first node coordinate of the specified node",
                                argList=['nodeNumber'],
                                example = "mbs.GetNodeODE2Index(nodeNumber=0)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetNodeParameter', cName='PyGetNodeParameter', 
                                description="get nodes's parameter from nodeNumber and parameterName; parameter names can be found for the specific items in the reference manual",
                                argList=['nodeNumber', 'parameterName']
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='SetNodeParameter', cName='PySetNodeParameter', 
                                description="set parameter 'parameterName' of node with nodeNumber to value; parameter names can be found for the specific items in the reference manual",
                                argList=['nodeNumber', 'parameterName', 'value']
                                ); s+=s1; sL+=sL1

sL += DefLatexFinishClass()
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#OBJECT
s += "\n//        OBJECTS:\n"
sL += DefLatexStartClass(classStr+': Object', 'This section provides functions for adding, reading and modifying objects, which can be bodies (mass point, rigid body, finite element, ...), connectors (spring-damper or joint) or general objects. Objects provided terms to the residual of equations resulting from every coordinate given by the nodes. Single-noded objects (e.g.~mass point) provides exactly residual terms for its nodal coordinates. Connectors constrain or penalize two markers, which can be, e.g., position, rigid or coordinate markers. Thus, the dependence of objects is either on the coordinates of the marker-objects/nodes or on nodes which the objects possess themselves.', subSection=True)
[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='AddObject', cName='[](MainSystem& mainSystem, py::dict itemDict) {return mainSystem.AddMainObject(itemDict); }', 
                                description="add a object with objectDefinition in dictionary format; returns (global) object number of newly added object",
                                argList=['itemDict'],
                                example="objectDict = {'objectType': 'MassPoint', \\\\'physicsMass': 10, \\\\'nodeNumber': 0, \\\\'name': 'example object'} \\\\ mbs.AddObject(objectDict)",
                                isLambdaFunction = True
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='AddObject', cName='[](MainSystem& mainSystem, py::object pyObject) {return mainSystem.AddMainObjectPyClass(pyObject); }', 
                                description="add a object with objectDefinition from Python object class; returns (global) object number of newly added object",
                                argList=['pyObject'],
                                example = "item = MassPoint(name='heavy object', nodeNumber=0, physicsMass=100) \\\\mbs.AddObject(item)",
                                isLambdaFunction = True
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetObjectNumber', cName='PyGetObjectNumber', 
                                description="get object's number by name (string)",
                                argList=['objectName'],
                                example = "n = mbs.GetObjectNumber('heavy object')"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetObject', cName='PyGetObject', 
                                description="get object's dictionary by index",
                                argList=['objectNumber'],
                                example = "objectDict = mbs.GetObject(0)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='ModifyObject', cName='PyModifyObject', 
                                description="modify object's dictionary by index",
                                argList=['objectNumber','objectDict'],
                                example = "mbs.ModifyObject(objectNumber, objectDict)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetObjectDefaults', cName='PyGetObjectDefaults', 
                                description="get object's default values for a certain objectType as (dictionary)",
                                argList=['typeName'],
                                example = "objectType = 'MassPoint'\\\\objectDict = mbs.GetObjectDefaults(objectType)"
                                ); s+=s1; sL+=sL1

#[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='CallObjectFunction', cName='PyCallObjectFunction', 
#                                description="call specific object function",
#                                argList=['objectNumber', 'functionName', 'args'],
#                                defaultArgs=['', '', 'py::dict()']
#                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetObjectOutput', cName='PyGetObjectOutputVariable', 
                                description="get object's output variable from objectNumber and OutputVariableType",
                                argList=['objectNumber', 'variableType']
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetObjectOutputBody', cName='PyGetObjectOutputVariableBody', 
                                description="get body's output variable from objectNumber and OutputVariableType",
                                argList=['objectNumber', 'variableType', 'localPosition', 'configuration'],
                                example = "u = mbs.GetObjectOutputBody(objectNumber = 1, variableType = exu.OutputVariableType.Position, localPosition=[1,0,0], configuration = exu.ConfigurationType.Initial)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetObjectParameter', cName='PyGetObjectParameter', 
                                description="get objects's parameter from objectNumber and parameterName; parameter names can be found for the specific items in the reference manual",
                                argList=['objectNumber', 'parameterName']
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='SetObjectParameter', cName='PySetObjectParameter', 
                                description="set parameter 'parameterName' of object with objectNumber to value; parameter names can be found for the specific items in the reference manual",
                                argList=['objectNumber', 'parameterName', 'value']
                                ); s+=s1; sL+=sL1

sL += DefLatexFinishClass()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#MARKER
s += "\n//        MARKER:\n"
sL += DefLatexStartClass(classStr+': Marker', 'This section provides functions for adding, reading and modifying markers. Markers define how to measure primal kinematical quantities on objects or nodes (e.g., position, orientation or coordinates themselves), and how to act on the quantities which are dual to the kinematical quantities (e.g., force, torque and generalized forces). Markers provide unique interfaces for loads, sensors and constraints in order to address these quantities independently of the structure of the object or node (e.g., rigid or flexible body).', subSection=True)
[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='AddMarker', cName='[](MainSystem& mainSystem, py::dict itemDict) {return mainSystem.AddMainMarker(itemDict); }', 
                                description="add a marker with markerDefinition in dictionary format; returns (global) marker number of newly added marker",
                                argList=['itemDict'],
                                example="markerDict = {'markerType': 'NodePosition', \\\\ 'nodeNumber': 0, \\\\ 'name': 'position0'}\\\\ mbs.AddMarker(markerDict)",
                                isLambdaFunction = True
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='AddMarker', cName='[](MainSystem& mainSystem, py::object pyObject) {return mainSystem.AddMainMarkerPyClass(pyObject); }', 
                                description="add a marker with markerDefinition from Python marker class; returns (global) marker number of newly added marker",
                                argList=['pyObject'],
                                example = "item = MarkerNodePosition(name='my marker',nodeNumber=1) \\\\mbs.AddMarker(item)",
                                isLambdaFunction = True
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetMarkerNumber', cName='PyGetMarkerNumber', 
                                description="get marker's number by name (string)",
                                argList=['markerName'],
                                example = "n = mbs.GetMarkerNumber('my marker')"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetMarker', cName='PyGetMarker', 
                                description="get marker's dictionary by index",
                                argList=['markerNumber'],
                                example = "markerDict = mbs.GetMarker(0)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='ModifyMarker', cName='PyModifyMarker', 
                                description="modify marker's dictionary by index",
                                argList=['markerNumber','markerDict'],
                                example = "mbs.ModifyMarker(markerNumber, markerDict)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetMarkerDefaults', cName='PyGetMarkerDefaults', 
                                description="get marker's default values for a certain markerType as (dictionary)",
                                argList=['typeName'],
                                example = "markerType = 'NodePosition'\\\\markerDict = mbs.GetMarkerDefaults(markerType)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetMarkerParameter', cName='PyGetMarkerParameter', 
                                description="get markers's parameter from markerNumber and parameterName; parameter names can be found for the specific items in the reference manual",
                                argList=['markerNumber', 'parameterName']
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='SetMarkerParameter', cName='PySetMarkerParameter', 
                                description="set parameter 'parameterName' of marker with markerNumber to value; parameter names can be found for the specific items in the reference manual",
                                argList=['markerNumber', 'parameterName', 'value']
                                ); s+=s1; sL+=sL1


sL += DefLatexFinishClass()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#LOAD
s += "\n//        LOADS:\n"
sL += DefLatexStartClass(classStr+': Load', 'This section provides functions for adding, reading and modifying operating loads. Loads are used to act on the quantities which are dual to the primal kinematic quantities, such as displacement and rotation. Loads represent, e.g., forces, torques or generalized forces.', subSection=True)
[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='AddLoad', cName='[](MainSystem& mainSystem, py::dict itemDict) {return mainSystem.AddMainLoad(itemDict); }', 
                                description="add a load with loadDefinition in dictionary format; returns (global) load number of newly added load",
                                argList=['itemDict'],
                                example="loadDict = {'loadType': 'ForceVector',\\\\ 'markerNumber': 0,\\\\ 'loadVector': [1.0, 0.0, 0.0],\\\\ 'name': 'heavy load'} \\\\ mbs.AddLoad(loadDict)",
                                isLambdaFunction = True
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='AddLoad', cName='[](MainSystem& mainSystem, py::object pyObject) {return mainSystem.AddMainLoadPyClass(pyObject); }', 
                                description="add a load with loadDefinition from Python load class; returns (global) load number of newly added load",
                                argList=['pyObject'],
                                example = "item = mbs.AddLoad(LoadForceVector(loadVector=[1,0,0],markerNumber=0,name='heavy load')) \\\\mbs.AddLoad(item)",
                                isLambdaFunction = True
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetLoadNumber', cName='PyGetLoadNumber', 
                                description="get load's number by name (string)",
                                argList=['loadName'],
                                example = "n = mbs.GetLoadNumber('heavy load')"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetLoad', cName='PyGetLoad', 
                                description="get load's dictionary by index",
                                argList=['loadNumber'],
                                example = "loadDict = mbs.GetLoad(0)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='ModifyLoad', cName='PyModifyLoad', 
                                description="modify load's dictionary by index",
                                argList=['loadNumber','loadDict'],
                                example = "mbs.ModifyLoad(loadNumber, loadDict)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetLoadDefaults', cName='PyGetLoadDefaults', 
                                description="get load's default values for a certain loadType as (dictionary)",
                                argList=['typeName'],
                                example = "loadType = 'ForceVector'\\\\loadDict = mbs.GetLoadDefaults(loadType)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetLoadParameter', cName='PyGetLoadParameter', 
                                description="get loads's parameter from loadNumber and parameterName; parameter names can be found for the specific items in the reference manual",
                                argList=['loadNumber', 'parameterName']
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='SetLoadParameter', cName='PySetLoadParameter', 
                                description="set parameter 'parameterName' of load with loadNumber to value; parameter names can be found for the specific items in the reference manual",
                                argList=['loadNumber', 'parameterName', 'value']
                                ); s+=s1; sL+=sL1

sL += DefLatexFinishClass()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#SENSORS
s += "\n//        SENSORS:\n"
sL += DefLatexStartClass(classStr+': Sensor', 'This section provides functions for adding, reading and modifying operating sensors. Sensors are used to measure information in nodes, objects, markers, and loads for output in a file.', subSection=True)
[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='AddSensor', cName='[](MainSystem& mainSystem, py::dict itemDict) {return mainSystem.AddMainSensor(itemDict); }', 
                                description="add a sensor with sensor definition in dictionary format; returns (global) sensor number of newly added sensor",
                                argList=['itemDict'],
                                example="sensorDict = {'sensorType': 'Node',\\\\ 'nodeNumber': 0,\\\\ 'fileName': 'sensor.txt',\\\\ 'name': 'test sensor'} \\\\ mbs.AddSensor(sensorDict)",
                                isLambdaFunction = True
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='AddSensor', cName='[](MainSystem& mainSystem, py::object pyObject) {return mainSystem.AddMainSensorPyClass(pyObject); }', 
                                description="add a sensor with sensor definition from Python sensor class; returns (global) sensor number of newly added sensor",
                                argList=['pyObject'],
                                example = "item = mbs.AddSensor(SensorNode(sensorType=exu.SensorType.Node,nodeNumber=0,name='test sensor')) \\\\mbs.AddSensor(item)",
                                isLambdaFunction = True
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetSensorNumber', cName='PyGetSensorNumber', 
                                description="get sensor's number by name (string)",
                                argList=['sensorName'],
                                example = "n = mbs.GetSensorNumber('test sensor')"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetSensor', cName='PyGetSensor', 
                                description="get sensor's dictionary by index",
                                argList=['sensorNumber'],
                                example = "sensorDict = mbs.GetSensor(0)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='ModifySensor', cName='PyModifySensor', 
                                description="modify sensor's dictionary by index",
                                argList=['sensorNumber','sensorDict'],
                                example = "mbs.ModifySensor(sensorNumber, sensorDict)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetSensorDefaults', cName='PyGetSensorDefaults', 
                                description="get sensor's default values for a certain sensorType as (dictionary)",
                                argList=['typeName'],
                                example = "sensorType = 'Node'\\\\sensorDict = mbs.GetSensorDefaults(sensorType)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetSensorParameter', cName='PyGetSensorParameter', 
                                description="get sensors's parameter from sensorNumber and parameterName; parameter names can be found for the specific items in the reference manual",
                                argList=['sensorNumber', 'parameterName']
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='SetSensorParameter', cName='PySetSensorParameter', 
                                description="set parameter 'parameterName' of sensor with sensorNumber to value; parameter names can be found for the specific items in the reference manual",
                                argList=['sensorNumber', 'parameterName', 'value']
                                ); s+=s1; sL+=sL1

sL += DefLatexFinishClass() #Sensors

#now finalize pybind class, but do nothing on latex side (sL1 ignored)
[s1,sL1] = DefPyFinishClass('MainSystem'); s+=s1 #; sL+=sL1



#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
pyClassStr = 'SystemData'
classStr = 'Main'+pyClassStr
[s1,sL1] = DefPyStartClass(classStr,pyClassStr, 'This is the data structure of a system which contains Objects (bodies/constraints/...), Nodes, Markers and Loads. The SystemData structure allows advanced access to this data, which HAS TO BE USED WITH CARE, as unexpected results and system crash might happen.' +
        ' \\\\ \n Usage: \\\\ \\small \n\\texttt{\\#obtain current ODE2 system vector (e.g. after static simulation finished): \\\\ u = mbs.systemData.GetODE2Coordinates() \\\\ \\#set initial ODE2 vector for next simulation:\\\\ \nmbs.systemData.SetODE2Coordinates(coordinates=u,configurationType=exu.ConfigurationType.Initial)}\n')
s+=s1; sL+=sL1


s += "\n//        General functions:\n"
#sL += '\\\\ \n'+classStr+': General functions', 'These functions allow to obtain system information (e.g. for debug purposes)', subSection=True)

#+++++++++++++++++++++++++++++++++
#General functions:
[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='NumberOfLoads', cName='[](const MainSystemData& msd) {return msd.GetMainLoads().NumberOfItems(); }', 
                                description="return number of loads in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfLoads())'); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='NumberOfMarkers', cName='[](const MainSystemData& msd) {return msd.GetMainMarkers().NumberOfItems(); }', 
                                description="return number of markers in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfMarkers())'); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='NumberOfNodes', cName='[](const MainSystemData& msd) {return msd.GetMainNodes().NumberOfItems(); }', 
                                description="return number of nodes in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfNodes())'); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='NumberOfObjects', cName='[](const MainSystemData& msd) {return msd.GetMainObjects().NumberOfItems(); }', 
                                description="return number of objects in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfObjects())'); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetTime', cName='PyGetStateTime', 
                                description="get configuration dependent time.",
                                argList=['configurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "mbs.systemData.GetTime(exu.ConfigurationType.Initial)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='SetTime', cName='PySetStateTime', 
                                description="set configuration dependent time; use this access with care, e.g. in user-defined solvers.",
                                argList=['newTime','configurationType'],
                                defaultArgs=['', 'exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "mbs.systemData.SetTime(10., exu.ConfigurationType.Initial)"
                                ); s+=s1; sL+=sL1


[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetCurrentTime', cName='PyGetCurrentTime', 
                                description="DEPRICATED; get current (simulation) time; time is updated in time integration solvers and in static solver; use this function e.g. during simulation to define time-dependent loads",
                                example = "mbs.systemData.GetCurrentTime()"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='SetVisualizationTime', cName='PySetVisualizationTime', 
                                description="DEPRICATED; set time for render window (visualization)",
                                example = "mbs.systemData.SetVisualizationTime(1.3)"
                                ); s+=s1; sL+=sL1

#[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='InfoSummary', cName='[](const MainSystemData& msd) {pout << msd.PyInfoSummary(); }', 
#                                description="Print short system information; same as print(mbs)",
#                                isLambdaFunction = True,
#                                example = 'mbs.systemData.InfoSummary()'); s+=s1; sL+=sL1
#

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='Info', cName='[](const MainSystemData& msd) {pout << msd.PyInfoDetailed(); }', 
                                description="print detailed system information for every item; for short information use print(mbs)",
                                isLambdaFunction = True,
                                example = 'mbs.systemData.Info()'); s+=s1; sL+=sL1



sL += DefLatexFinishClass()

s += "\n//        Coordinate access:\n"
sL += DefLatexStartClass(pyClassStr+': Access coordinates', 'This section provides access functions to global coordinate vectors. Assigning invalid values or using wrong vector size might lead to system crash and unexpected results.', subSection=True)
#+++++++++++++++++++++++++++++++++
#coordinate access functions:

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetODE2Coordinates', cName='GetODE2Coords', 
                                description="get ODE2 system coordinates (displacements) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "uCurrent = mbs.systemData.GetODE2Coordinates()"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='SetODE2Coordinates', cName='SetODE2Coords', 
                                description="set ODE2 system coordinates (displacements) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE2Coordinates(uCurrent)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetODE2Coordinates_t', cName='GetODE2Coords_t', 
                                description="get ODE2 system coordinates (velocities) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "vCurrent = mbs.systemData.GetODE2Coordinates_t()"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='SetODE2Coordinates_t', cName='SetODE2Coords_t', 
                                description="set ODE2 system coordinates (velocities) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE2Coordinates_t(vCurrent)"
                                ); s+=s1; sL+=sL1


[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetODE1Coordinates', cName='GetODE1Coords', 
                                description="get ODE1 system coordinates (displacements) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "qCurrent = mbs.systemData.GetODE1Coordinates()"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='SetODE1Coordinates', cName='SetODE1Coords', 
                                description="set ODE1 system coordinates (displacements) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE1Coordinates(qCurrent)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetAECoordinates', cName='GetAECoords', 
                                description="get algebraic equations (AE) system coordinates for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "lambdaCurrent = mbs.systemData.GetAECoordinates()"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='SetAECoordinates', cName='SetAECoords', 
                                description="set algebraic equations (AE) system coordinates for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetAECoordinates(lambdaCurrent)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetDataCoordinates', cName='GetDataCoords', 
                                description="get system data coordinates for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "dataCurrent = mbs.systemData.GetDataCoordinates()"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='SetDataCoordinates', cName='SetDataCoords', 
                                description="set system data coordinates for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetDataCoordinates(dataCurrent)"
                                ); s+=s1; sL+=sL1



[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetSystemState', cName='PyGetSystemState', 
                                description="get system state for given configuration (default: exu.Configuration.Current); state vectors do not include the non-state derivatives ODE1_t and ODE2_tt and the time; function is copying data - not highly efficient; format of pyList: [ODE2Coords, ODE2Coords_t, ODE1Coords, AEcoords, dataCoords]",
                                argList=['configuration'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "sysStateList = mbs.systemData.GetSystemState()"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='SetSystemState', cName='PySetSystemState', 
                                description="set system data coordinates for given configuration (default: exu.Configuration.Current); invalid list of vectors / vector size may lead to system crash; write access to state vectors (but not the non-state derivatives ODE1_t and ODE2_tt and the time); function is copying data - not highly efficient; format of pyList: [ODE2Coords, ODE2Coords_t, ODE1Coords, AEcoords, dataCoords]",
                                argList=['systemStateList','configuration'],
                                defaultArgs=['','exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "mbs.systemData.SetDataCoordinates(sysStateList, configurationType = exu.ConfigurationType.Initial)"
                                ); s+=s1; sL+=sL1


sL += DefLatexFinishClass()

#+++++++++++++++++++++++++++++++++
#LTG-functions:
s += "\n//        LTG readout functions:\n"
sL += DefLatexStartClass(pyClassStr+': Get object local-to-global (LTG) coordinate mappings', 'This section provides access functions the LTG-lists for every object (body, constraint, ...) in the system.', subSection=True)

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetObjectLTGODE2', cName='PyGetObjectLocalToGlobalODE2', 
                                description="get local-to-global coordinate mapping (list of global coordinate indices) for ODE2 coordinates; only available after Assemble()",
                                argList=['objectNumber'],
                                example = "ltgObject4 = mbs.systemData.GetObjectLTGODE2(4)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetObjectLTGODE1', cName='PyGetObjectLocalToGlobalODE1', 
                                description="get local-to-global coordinate mapping (list of global coordinate indices) for ODE1 coordinates; only available after Assemble()",
                                argList=['objectNumber'],
                                example = "ltgObject4 = mbs.systemData.GetObjectLTGODE1(4)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetObjectLTGAE', cName='PyGetObjectLocalToGlobalAE', 
                                description="get local-to-global coordinate mapping (list of global coordinate indices) for algebraic equations (AE) coordinates; only available after Assemble()",
                                argList=['objectNumber'],
                                example = "ltgObject4 = mbs.systemData.GetObjectLTGODE2(4)"
                                ); s+=s1; sL+=sL1

[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='GetObjectLTGData', cName='PyGetObjectLocalToGlobalData', 
                                description="get local-to-global coordinate mapping (list of global coordinate indices) for data coordinates; only available after Assemble()",
                                argList=['objectNumber'],
                                example = "ltgObject4 = mbs.systemData.GetObjectLTGData(4)"
                                ); s+=s1; sL+=sL1

sL += DefLatexFinishClass()

#now finalize pybind class, but do nothing on latex side (sL1 ignored)
[s1,sL1] = DefPyFinishClass('SystemData'); s+=s1 #; sL+=sL1



#[s1,sL1] = DefPyFunctionAccess(cClass=classStr, pyName='', cName='', 
#                                description='',
#                                argList=[]); s+=s1; sL+=sL1



#+++++++++++++++++++++++++++++++++++++++++++++++++++

sL += sLenum #put latex description of enums after the systemData section

directoryString = '..\\Autogenerated\\'
pybindFile = directoryString + 'pybind_manual_classes.h'
latexFile = '..\\..\\..\\docs\\theDoc\\manual_interfaces.tex'

file=open(pybindFile,'w')  #clear file by one write access
file.write('// AUTO:  ++++++++++++++++++++++\n')
file.write('// AUTO:  pybind11 manual module includes; generated by Johannes Gerstmayr\n')
file.write('// AUTO:  last modified = '+ GetDateStr() + '\n')
file.write('// AUTO:  ++++++++++++++++++++++\n')
file.write(s)
file.close()

file=open(latexFile,'w')  #clear file by one write access
file.write('% ++++++++++++++++++++++')
file.write('% description of manual pybind interfaces; generated by Johannes Gerstmayr')
file.write('% ++++++++++++++++++++++')
file.write(sL)
file.close()

